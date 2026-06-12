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
#include <stdint.h>

static const char *TAG = "pump";

#define NVS_KEY_THRESHOLD  "fb_thresh_mv" // u16: feedback on/off threshold

static adc_oneshot_unit_handle_t s_adc = NULL;
static adc_cali_handle_t          s_cali = NULL;            // feedback channel
static bool                       s_cali_ok = false;
static adc_cali_handle_t          s_cali_pressure = NULL;   // pressure channel
static bool                       s_cali_pressure_ok = false;

static SemaphoreHandle_t s_mutex = NULL;
static pump_reading_t    s_reading = {0};

// The pump ALWAYS boots OFF. Commanded state is never persisted — a power
// loss, reboot, or OTA must never silently re-energise the pump.
static volatile bool s_commanded_on = false;
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

static inline int raw_to_mv(adc_cali_handle_t cali, bool cali_ok, int raw)
{
    int mv = 0;
    if (cali_ok) {
        adc_cali_raw_to_voltage(cali, raw, &mv);
    } else {
        // Fallback: linear approximation if calibration scheme isn't available.
        // 12-bit ADC, DB_12 attenuation, ~3100 mV full-scale.
        mv = (raw * 3100) / 4095;
    }
    return mv;
}

// Recover the transducer voltage (mV) from the divided ADC voltage.
static inline float pressure_source_mv(int adc_mv)
{
    return adc_mv * PRESSURE_DIVIDER_MULT;
}

// Convert an averaged ADC voltage (mV, divided) to psi: undo the divider, then
// apply the transfer function, clamped at 0.
static inline float pressure_mv_to_psi(int adc_mv)
{
    float src_mv = pressure_source_mv(adc_mv);
    float psi = (src_mv - (float)PRESSURE_V_MIN_MV) /
                (float)(PRESSURE_V_MAX_MV - PRESSURE_V_MIN_MV) * PRESSURE_PSI_MAX;
    return psi < 0.0f ? 0.0f : psi;
}

static void sampler_task(void *arg)
{
    const TickType_t sample_period = pdMS_TO_TICKS(1000 / PUMP_FEEDBACK_OVERSAMPLE_HZ);
    const int        samples_per_s = PUMP_FEEDBACK_OVERSAMPLE_HZ;
    int    in_window = 0;
    int    sum_mv = 0;
    int    sum_pmv = 0;
    int    peak_mv = 0;
    TickType_t last = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last, sample_period);

        int raw = 0;
        esp_err_t err = adc_oneshot_read(s_adc, PUMP_FEEDBACK_ADC_CHANNEL, &raw);
        if (err != ESP_OK) continue;
        int mv = raw_to_mv(s_cali, s_cali_ok, raw);
        s_sample_count++;

        // Pressure transducer on the second ADC1 channel (same unit).
        int praw = 0;
        int pmv = 0;
        if (adc_oneshot_read(s_adc, PRESSURE_ADC_CHANNEL, &praw) == ESP_OK) {
            pmv = raw_to_mv(s_cali_pressure, s_cali_pressure_ok, praw);
        }

        sum_mv += mv;
        sum_pmv += pmv;
        if (mv > peak_mv) peak_mv = mv;
        in_window++;

        if (in_window >= samples_per_s) {
            int avg_mv = sum_mv / in_window;
            int avg_pmv = sum_pmv / in_window;
            int captured_peak = peak_mv;
            in_window = 0;
            sum_mv = 0;
            sum_pmv = 0;
            peak_mv = 0;

            int64_t now_us = esp_timer_get_time();

            // Everything shared with the command tasks (commanded state, the
            // threshold, the settle timestamp) is read under the mutex, and the
            // reading is written in the same critical section, so a command
            // arriving mid-update can't tear these values.
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            bool    commanded   = s_commanded_on;
            int     threshold   = s_threshold_mv;
            int64_t last_change = s_last_change_us;

            // Feedback with hysteresis: once ON, only fall back OFF below
            // (threshold - hysteresis), so a sense voltage hovering at the
            // threshold doesn't chatter the state (and spam retained MQTT).
            bool feedback_on = s_reading.feedback_on
                ? (avg_mv >= threshold - PUMP_FEEDBACK_HYSTERESIS_MV)
                : (avg_mv >= threshold);

            // Only treat a disagreement as a fault once the relay has had time
            // to settle after the last command.
            bool settled  = (now_us - last_change) > (PUMP_SETTLE_MS * 1000LL);
            bool mismatch = settled && (commanded != feedback_on);

            if (commanded) s_on_seconds++;

            s_reading.valid          = true;
            s_reading.commanded_on   = commanded;
            s_reading.feedback_on    = feedback_on;
            s_reading.mismatch       = mismatch;
            s_reading.feedback_v     = avg_mv / 1000.0f;
            s_reading.feedback_mv    = avg_mv;
            s_reading.raw_mv         = mv;
            s_reading.peak_mv        = captured_peak;
            s_reading.saturated      = captured_peak >= PUMP_FEEDBACK_SATURATION_MV;
            s_reading.threshold_mv   = threshold;
            // pressure_v is the recovered transducer voltage (0.5-4.5 V range);
            // pressure_mv stays the raw mV at the ADC pin for diagnostics.
            s_reading.pressure_v     = pressure_source_mv(avg_pmv) / 1000.0f;
            s_reading.pressure_mv    = avg_pmv;
            s_reading.pressure_psi   = pressure_mv_to_psi(avg_pmv);
            s_reading.output_gpio    = PUMP_OUTPUT_GPIO;
            s_reading.feedback_gpio  = PUMP_FEEDBACK_GPIO;
            s_reading.pressure_gpio  = PRESSURE_GPIO;
            s_reading.on_seconds     = s_on_seconds;
            s_reading.sample_count   = s_sample_count;
            s_reading.last_sample_us = now_us;
            s_reading.last_change_us = last_change;
            xSemaphoreGive(s_mutex);
        }
    }
}

esp_err_t pump_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    // Restore the persisted feedback threshold (calibration). The commanded
    // pump state is deliberately NOT restored — the pump always boots OFF.
    uint16_t stored;
    if (nvs_store_get_u16(NVS_NS_PUMP, NVS_KEY_THRESHOLD, &stored) == ESP_OK && stored > 0) {
        s_threshold_mv = (int)stored;
        ESP_LOGI(TAG, "Loaded feedback threshold: %d mV", s_threshold_mv);
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
    drive_output(s_commanded_on);  // s_commanded_on == false → boots OFF
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
    // Pressure transducer channel on the same unit (non-fatal if it fails —
    // the pump still controls; pressure just reads 0).
    err = adc_oneshot_config_channel(s_adc, PRESSURE_ADC_CHANNEL, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Pressure ADC channel config failed: %s", esp_err_to_name(err));
    }

    // Curve-fitting calibration (supported on ESP32-C3) — one handle per channel.
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

    adc_cali_curve_fitting_config_t pcali_cfg = {
        .unit_id  = PRESSURE_ADC_UNIT,
        .chan     = PRESSURE_ADC_CHANNEL,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&pcali_cfg, &s_cali_pressure) == ESP_OK) {
        s_cali_pressure_ok = true;
    }
#else
    ESP_LOGW(TAG, "ADC curve-fitting not compiled in, using linear approximation");
#endif

    xTaskCreate(sampler_task, "pump_smp", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Pump ready (boots OFF) — output GPIO%d (%s=on), feedback GPIO%d (ch%d), pressure GPIO%d (ch%d)",
             PUMP_OUTPUT_GPIO, PUMP_OUTPUT_ACTIVE_LOW ? "LOW" : "HIGH",
             PUMP_FEEDBACK_GPIO, (int)PUMP_FEEDBACK_ADC_CHANNEL,
             PRESSURE_GPIO, (int)PRESSURE_ADC_CHANNEL);
    return ESP_OK;
}

void pump_get(pump_reading_t *out)
{
    if (!out || !s_mutex) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(out, &s_reading, sizeof(pump_reading_t));
    xSemaphoreGive(s_mutex);
}

// Apply a commanded state. MUST be called with s_mutex held — drives the
// output, stamps the settle timer, and reflects the change into s_reading
// immediately so a status read between sampler ticks isn't stale.
static void apply_state_locked(bool on)
{
    s_commanded_on   = on;
    s_last_change_us = esp_timer_get_time();
    drive_output(on);
    s_reading.commanded_on   = on;
    s_reading.last_change_us = s_last_change_us;
    s_reading.mismatch       = false;  // suppressed during the settle window
}

esp_err_t pump_set(bool on)
{
    if (!s_mutex) return ESP_FAIL;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool changed = (on != s_commanded_on);
    apply_state_locked(on);
    xSemaphoreGive(s_mutex);

    // State is intentionally NOT persisted — the pump always boots OFF.
    ESP_LOGI(TAG, "Pump %s%s", on ? "ON" : "OFF", changed ? "" : " (no change)");
    return ESP_OK;
}

esp_err_t pump_toggle(void)
{
    if (!s_mutex) return ESP_FAIL;
    // Read-modify-write under one lock so two concurrent toggles (e.g. HTTP +
    // MQTT) can't both read the same value and cancel out.
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool target = !s_commanded_on;
    apply_state_locked(target);
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Pump %s (toggle)", target ? "ON" : "OFF");
    return ESP_OK;
}

bool pump_is_on(void)
{
    if (!s_mutex) return s_commanded_on;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool on = s_commanded_on;
    xSemaphoreGive(s_mutex);
    return on;
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
    if (!s_mutex) return s_threshold_mv;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int v = s_threshold_mv;
    xSemaphoreGive(s_mutex);
    return v;
}

// One-shot worker that commits the threshold to NVS off the caller's task, so
// the flash erase/write (tens of ms) never blocks the MQTT event loop (which
// would risk a missed keepalive). Deletes itself when done.
static void threshold_persist_task(void *arg)
{
    int mv = (int)(intptr_t)arg;
    esp_err_t err = nvs_store_set_u16(NVS_NS_PUMP, NVS_KEY_THRESHOLD, (uint16_t)mv);
    ESP_LOGI(TAG, "Feedback threshold persisted: %d mV (%s)",
             mv, err == ESP_OK ? "ok" : "failed");
    vTaskDelete(NULL);
}

esp_err_t pump_set_threshold_mv(int mv)
{
    if (mv < 100 || mv > 3300) return ESP_ERR_INVALID_ARG;

    if (s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_threshold_mv = mv;
        xSemaphoreGive(s_mutex);
    } else {
        s_threshold_mv = mv;
    }

    // Persist asynchronously; fall back to an inline write if the worker can't
    // be spawned (e.g. out of memory).
    if (xTaskCreate(threshold_persist_task, "thr_nvs", 3072,
                    (void *)(intptr_t)mv, 3, NULL) != pdPASS) {
        nvs_store_set_u16(NVS_NS_PUMP, NVS_KEY_THRESHOLD, (uint16_t)mv);
    }
    ESP_LOGI(TAG, "Feedback threshold set: %d mV", mv);
    return ESP_OK;
}

// Parse the value token immediately after `colon` (skipping spaces and an
// optional opening quote). Returns 1 for on/true/1, 0 for off/false/0, -1 if
// unrecognised — so a stray "on"/"1" elsewhere in the payload can't flip it.
static int parse_bool_token(const char *colon)
{
    const char *v = colon + 1;
    while (*v == ' ' || *v == '\t') v++;
    if (*v == '"') v++;
    if (strncmp(v, "on", 2) == 0 || strncmp(v, "true", 4) == 0 || *v == '1') return 1;
    if (strncmp(v, "off", 3) == 0 || strncmp(v, "false", 5) == 0 || *v == '0') return 0;
    return -1;
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
    if (tg) {
        const char *colon = strchr(tg, ':');
        if (colon && parse_bool_token(colon) == 1) pump_toggle();
    }

    // {"pump":"on"|"off"} or {"pump":true|false}
    const char *p = strstr(buf, "\"pump\"");
    if (p) {
        const char *colon = strchr(p, ':');
        if (colon) {
            int b = parse_bool_token(colon);
            if (b == 1)      pump_set(true);
            else if (b == 0) pump_set(false);
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
