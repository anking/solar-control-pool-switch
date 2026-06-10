#include "pump.h"
#include "config.h"
#include "nvs_store.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "pump";

#define NVS_KEY_STATE      "state"        // u16 0/1: last commanded state
#define NVS_KEY_THRESHOLD  "fb_thresh_mv" // u16: feedback on/off threshold

static adc_oneshot_unit_handle_t s_adc = NULL;
static adc_cali_handle_t         s_cali = NULL;
static bool                      s_cali_ok = false;

static SemaphoreHandle_t s_mutex = NULL;
static pump_reading_t    s_reading = {0};

static volatile bool s_commanded_on = (PUMP_DEFAULT_STATE_ON != 0);
static int           s_threshold_mv = PUMP_FEEDBACK_ON_THRESHOLD_MV;

// Set the moment commanded state changes; the mismatch check is suppressed
// until PUMP_SETTLE_MS after this so the relay has time to actually move.
static int64_t  s_last_change_us = 0;
static uint32_t s_on_seconds     = 0;
static uint32_t s_sample_count   = 0;

static inline void drive_output(bool on)
{
#if PUMP_OUTPUT_ACTIVE_LOW
    gpio_set_level(PUMP_OUTPUT_GPIO, on ? 0 : 1);
#else
    gpio_set_level(PUMP_OUTPUT_GPIO, on ? 1 : 0);
#endif
}

static inline int raw_to_mv(int raw)
{
    int mv = 0;
    if (s_cali_ok) {
        adc_cali_raw_to_voltage(s_cali, raw, &mv);
    } else {
        // Fallback: linear approximation if calibration scheme isn't available.
        // 12-bit ADC, DB_12 attenuation, ~3100 mV full-scale.
        mv = (raw * 3100) / 4095;
    }
    return mv;
}

static void sampler_task(void *arg)
{
    const TickType_t sample_period = pdMS_TO_TICKS(1000 / PUMP_FEEDBACK_OVERSAMPLE_HZ);
    const int        samples_per_s = PUMP_FEEDBACK_OVERSAMPLE_HZ;
    int    in_window = 0;
    int    sum_mv = 0;
    int    peak_mv = 0;
    TickType_t last = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last, sample_period);

        int raw = 0;
        esp_err_t err = adc_oneshot_read(s_adc, PUMP_FEEDBACK_ADC_CHANNEL, &raw);
        if (err != ESP_OK) continue;
        int mv = raw_to_mv(raw);
        s_sample_count++;

        sum_mv += mv;
        if (mv > peak_mv) peak_mv = mv;
        in_window++;

        if (in_window >= samples_per_s) {
            int avg_mv = sum_mv / in_window;
            int captured_peak = peak_mv;
            in_window = 0;
            sum_mv = 0;
            peak_mv = 0;

            bool    commanded   = s_commanded_on;
            bool    feedback_on = avg_mv >= s_threshold_mv;
            int64_t now_us      = esp_timer_get_time();

            // Only treat a disagreement as a fault once the relay has had time
            // to settle after the last command.
            bool settled  = (now_us - s_last_change_us) > (PUMP_SETTLE_MS * 1000LL);
            bool mismatch = settled && (commanded != feedback_on);

            if (commanded) s_on_seconds++;

            xSemaphoreTake(s_mutex, portMAX_DELAY);
            s_reading.valid          = true;
            s_reading.commanded_on   = commanded;
            s_reading.feedback_on    = feedback_on;
            s_reading.mismatch       = mismatch;
            s_reading.feedback_v     = avg_mv / 1000.0f;
            s_reading.feedback_mv    = avg_mv;
            s_reading.raw_mv         = mv;
            s_reading.peak_mv        = captured_peak;
            s_reading.saturated      = captured_peak >= PUMP_FEEDBACK_SATURATION_MV;
            s_reading.threshold_mv   = s_threshold_mv;
            s_reading.output_gpio    = PUMP_OUTPUT_GPIO;
            s_reading.feedback_gpio  = PUMP_FEEDBACK_GPIO;
            s_reading.on_seconds     = s_on_seconds;
            s_reading.sample_count   = s_sample_count;
            s_reading.last_sample_us = now_us;
            s_reading.last_change_us = s_last_change_us;
            xSemaphoreGive(s_mutex);
        }
    }
}

esp_err_t pump_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    // Restore persisted threshold + last commanded state.
    uint16_t stored;
    if (nvs_store_get_u16(NVS_NS_PUMP, NVS_KEY_THRESHOLD, &stored) == ESP_OK && stored > 0) {
        s_threshold_mv = (int)stored;
        ESP_LOGI(TAG, "Loaded feedback threshold: %d mV", s_threshold_mv);
    }
    if (nvs_store_get_u16(NVS_NS_PUMP, NVS_KEY_STATE, &stored) == ESP_OK) {
        s_commanded_on = (stored != 0);
        ESP_LOGI(TAG, "Restored last pump state: %s", s_commanded_on ? "ON" : "OFF");
    }

    // Control output — configure the level BEFORE switching the pin to output,
    // so we don't glitch the relay during the brief default-low window.
    gpio_config_t out_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PUMP_OUTPUT_GPIO),
        .pull_down_en = 0,
        .pull_up_en   = 0,
    };
    esp_err_t err = gpio_config(&out_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Output GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }
    drive_output(s_commanded_on);
    s_last_change_us = esp_timer_get_time();

    // Feedback ADC1 oneshot init.
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = PUMP_FEEDBACK_ADC_UNIT };
    err = adc_oneshot_new_unit(&init_cfg, &s_adc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc, PUMP_FEEDBACK_ADC_CHANNEL, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(err));
        return err;
    }

    // Curve-fitting calibration (supported on ESP32-C3).
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = PUMP_FEEDBACK_ADC_UNIT,
        .chan     = PUMP_FEEDBACK_ADC_CHANNEL,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali) == ESP_OK) {
        s_cali_ok = true;
        ESP_LOGI(TAG, "ADC calibration: curve-fitting");
    } else {
        ESP_LOGW(TAG, "ADC calibration unavailable, using linear approximation");
    }
#else
    ESP_LOGW(TAG, "ADC curve-fitting not compiled in, using linear approximation");
#endif

    xTaskCreate(sampler_task, "pump_smp", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Pump ready — output GPIO%d (%s=on), feedback GPIO%d (ADC1 ch%d, on>=%dmV)",
             PUMP_OUTPUT_GPIO, PUMP_OUTPUT_ACTIVE_LOW ? "LOW" : "HIGH",
             PUMP_FEEDBACK_GPIO, (int)PUMP_FEEDBACK_ADC_CHANNEL, s_threshold_mv);
    return ESP_OK;
}

void pump_get(pump_reading_t *out)
{
    if (!out || !s_mutex) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(out, &s_reading, sizeof(pump_reading_t));
    xSemaphoreGive(s_mutex);
}

esp_err_t pump_set(bool on)
{
    bool changed = (on != s_commanded_on);
    s_commanded_on = on;
    drive_output(on);
    s_last_change_us = esp_timer_get_time();

    // Reflect immediately so a status read between sampler ticks isn't stale.
    if (s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_reading.commanded_on   = on;
        s_reading.last_change_us = s_last_change_us;
        s_reading.mismatch       = false;  // suppressed during settle window
        xSemaphoreGive(s_mutex);
    }

    esp_err_t err = nvs_store_set_u16(NVS_NS_PUMP, NVS_KEY_STATE, on ? 1 : 0);
    ESP_LOGI(TAG, "Pump %s%s (%s)",
             on ? "ON" : "OFF",
             changed ? "" : " (no change)",
             err == ESP_OK ? "saved" : "save failed");
    return err;
}

esp_err_t pump_toggle(void)
{
    return pump_set(!s_commanded_on);
}

bool pump_is_on(void)
{
    return s_commanded_on;
}

bool pump_has_fault(void)
{
    if (!s_mutex) return false;
    bool fault;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    fault = s_reading.valid && s_reading.mismatch;
    xSemaphoreGive(s_mutex);
    return fault;
}

int pump_get_threshold_mv(void)
{
    return s_threshold_mv;
}

esp_err_t pump_set_threshold_mv(int mv)
{
    if (mv < 100 || mv > 3300) return ESP_ERR_INVALID_ARG;
    s_threshold_mv = mv;
    esp_err_t err = nvs_store_set_u16(NVS_NS_PUMP, NVS_KEY_THRESHOLD, (uint16_t)mv);
    ESP_LOGI(TAG, "Feedback threshold set: %d mV (%s)",
             mv, err == ESP_OK ? "saved" : "save failed");
    return err;
}

void pump_handle_command(const char *json, int len)
{
    if (!json || len <= 0) return;

    // Bounded local copy so we can use plain string ops on the payload.
    char buf[160];
    int n = len < (int)sizeof(buf) - 1 ? len : (int)sizeof(buf) - 1;
    memcpy(buf, json, n);
    buf[n] = '\0';
    ESP_LOGI(TAG, "Command: %s", buf);

    // {"toggle":true}
    const char *tg = strstr(buf, "\"toggle\"");
    if (tg && strstr(tg, "true")) {
        pump_toggle();
    }

    // {"pump":"on"|"off"} or {"pump":true|false}
    const char *p = strstr(buf, "\"pump\"");
    if (p) {
        const char *colon = strchr(p, ':');
        if (colon) {
            if (strstr(colon, "on") || strstr(colon, "true") || strstr(colon, "1")) {
                pump_set(true);
            } else if (strstr(colon, "off") || strstr(colon, "false") || strstr(colon, "0")) {
                pump_set(false);
            }
        }
    }

    // {"threshold_mv":1600}
    const char *th = strstr(buf, "\"threshold_mv\"");
    if (th) {
        const char *colon = strchr(th, ':');
        if (colon) {
            long v = strtol(colon + 1, NULL, 10);
            if (v >= 100 && v <= 3300) pump_set_threshold_mv((int)v);
        }
    }
}
