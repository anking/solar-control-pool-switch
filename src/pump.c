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
#define NVS_KEY_MIN_PSI    "p_min_dpsi"   // u16: min pressure, deci-psi (psi*10)
#define NVS_KEY_MAX_PSI    "p_max_dpsi"   // u16: warn pressure, deci-psi
#define NVS_KEY_CRIT_PSI   "p_crit_dpsi"  // u16: critical pressure, deci-psi
#define NVS_KEY_LOCKOUT    "p_lockout"    // u16: 0 = clear, else pump_safety_reason_t
#define NVS_KEY_PRIME_S    "p_prime_s"    // u16: prime grace window, seconds
#define NVS_KEY_LOSS_EN    "p_loss_en"    // u16: mid-run pressure-loss rule (0/1)

// psi <-> deci-psi (u16 NVS storage). 0..100 psi -> 0..1000, fits a u16.
#define PSI_TO_DPSI(psi)   ((uint16_t)((psi) * 10.0f + 0.5f))
#define DPSI_TO_PSI(dpsi)  ((float)(dpsi) / 10.0f)

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
// Latched when a safety failsafe turned the pump off; cleared by the next real
// commanded change. Guarded by s_mutex.
static bool     s_failsafe_off   = false;

// Pressure safety. Thresholds (psi) are the cloud-pushed config; the lockout is
// latched on a trip and persists across reboot until a manual reset. All guarded
// by s_mutex.
static float    s_min_psi        = PUMP_PRESSURE_MIN_PSI_DEFAULT;
static float    s_max_psi        = PUMP_PRESSURE_MAX_PSI_DEFAULT;
static float    s_critical_psi   = PUMP_PRESSURE_CRITICAL_PSI_DEFAULT;
static int      s_prime_grace_s  = PUMP_PRIME_GRACE_S_DEFAULT;
static bool     s_pressure_loss_en = true;
static bool     s_safety_lockout = false;
static int      s_safety_reason  = PUMP_SAFETY_OK;
static bool     s_pressure_warning = false;

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
    int    p_count = 0;
    int    peak_mv = 0;
    TickType_t last = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last, sample_period);

        int raw = 0;
        esp_err_t err = adc_oneshot_read(s_adc, PUMP_FEEDBACK_ADC_CHANNEL, &raw);
        if (err != ESP_OK) continue;
        int mv = raw_to_mv(s_cali, s_cali_ok, raw);
        s_sample_count++;

        // Pressure transducer on the second ADC1 channel (same unit). Failed
        // reads are EXCLUDED from the average (a zero would drag the mean down
        // and read as falsely low pressure); p_count tracks the good samples so
        // a fully-dead channel is flagged invalid rather than reading 0 psi.
        int praw = 0;
        if (adc_oneshot_read(s_adc, PRESSURE_ADC_CHANNEL, &praw) == ESP_OK) {
            sum_pmv += raw_to_mv(s_cali_pressure, s_cali_pressure_ok, praw);
            p_count++;
        }

        sum_mv += mv;
        if (mv > peak_mv) peak_mv = mv;
        in_window++;

        if (in_window >= samples_per_s) {
            int avg_mv = sum_mv / in_window;
            bool p_valid = p_count > 0;
            int avg_pmv = p_valid ? sum_pmv / p_count : 0;
            int captured_peak = peak_mv;
            in_window = 0;
            sum_mv = 0;
            sum_pmv = 0;
            p_count = 0;
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
            s_reading.pressure_valid = p_valid;
            s_reading.pressure_v     = p_valid ? pressure_source_mv(avg_pmv) / 1000.0f : 0.0f;
            s_reading.pressure_mv    = avg_pmv;
            s_reading.pressure_psi   = p_valid ? pressure_mv_to_psi(avg_pmv) : 0.0f;
            s_reading.failsafe_off   = s_failsafe_off;
            s_reading.safety_lockout = s_safety_lockout;
            s_reading.safety_reason  = s_safety_reason;
            s_reading.pressure_warning = s_pressure_warning;
            s_reading.min_psi        = s_min_psi;
            s_reading.max_psi        = s_max_psi;
            s_reading.critical_psi   = s_critical_psi;
            s_reading.prime_grace_s  = s_prime_grace_s;
            s_reading.pressure_loss_enabled = s_pressure_loss_en;
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

    // Restore the pressure thresholds (cloud-pushed config). Missing keys keep
    // the compiled defaults; the cloud re-pushes on connect regardless.
    uint16_t dpsi;
    if (nvs_store_get_u16(NVS_NS_PUMP, NVS_KEY_MIN_PSI,  &dpsi) == ESP_OK) s_min_psi      = DPSI_TO_PSI(dpsi);
    if (nvs_store_get_u16(NVS_NS_PUMP, NVS_KEY_MAX_PSI,  &dpsi) == ESP_OK) s_max_psi      = DPSI_TO_PSI(dpsi);
    if (nvs_store_get_u16(NVS_NS_PUMP, NVS_KEY_CRIT_PSI, &dpsi) == ESP_OK) s_critical_psi = DPSI_TO_PSI(dpsi);

    // Prime grace window + mid-run pressure-loss switch (cloud-pushed config).
    // Out-of-range stored values (e.g. from a downgrade) fall back to defaults.
    uint16_t u16;
    if (nvs_store_get_u16(NVS_NS_PUMP, NVS_KEY_PRIME_S, &u16) == ESP_OK &&
        u16 >= PUMP_PRIME_GRACE_S_MIN && u16 <= PUMP_PRIME_GRACE_S_MAX) {
        s_prime_grace_s = (int)u16;
    }
    if (nvs_store_get_u16(NVS_NS_PUMP, NVS_KEY_LOSS_EN, &u16) == ESP_OK) {
        s_pressure_loss_en = (u16 != 0);
    }

    // Restore a latched safety lockout — a tripped failsafe stays tripped across
    // a reboot until a manual reset (the pump still boots OFF either way).
    uint16_t lock;
    if (nvs_store_get_u16(NVS_NS_PUMP, NVS_KEY_LOCKOUT, &lock) == ESP_OK && lock != 0) {
        s_safety_lockout = true;
        s_safety_reason  = (int)lock;
        ESP_LOGW(TAG, "Restored safety lockout (reason %d) — pump will not start until reset", s_safety_reason);
    }
    ESP_LOGI(TAG, "Pressure limits: min=%.1f warn=%.1f critical=%.1f psi, prime=%ds, loss-rule=%s",
             s_min_psi, s_max_psi, s_critical_psi, s_prime_grace_s,
             s_pressure_loss_en ? "on" : "off");

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
// output and reflects the change into s_reading immediately so a status read
// between sampler ticks isn't stale. The settle timer / mismatch suppression
// is reset ONLY on a real state change: a redundant re-send of the current
// state must not keep restarting the settle window (that would let a looping
// "on" command suppress stuck-relay fault detection indefinitely) nor reset
// the max-runtime failsafe clock. Returns whether the state actually changed.
static bool apply_state_locked(bool on)
{
    // Safety lockout: refuse to energise the pump. Turning OFF is always
    // allowed (and is a no-op if already off). This is the authoritative guard
    // — it blocks every "on" path: manual, MQTT command, and cloud schedule.
    if (on && s_safety_lockout) {
        ESP_LOGW(TAG, "Refusing pump ON — safety lockout active (reason %d)", s_safety_reason);
        return false;
    }

    bool changed = (on != s_commanded_on);
    s_commanded_on = on;
    drive_output(on);
    s_reading.commanded_on = on;
    if (changed) {
        s_last_change_us = esp_timer_get_time();
        s_reading.last_change_us = s_last_change_us;
        s_reading.mismatch       = false;  // suppressed during the settle window
        s_failsafe_off           = false;  // a real command supersedes a failsafe trip
        s_reading.failsafe_off   = false;
    }
    return changed;
}

esp_err_t pump_set(bool on)
{
    if (!s_mutex) return ESP_FAIL;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool changed = apply_state_locked(on);
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

// Threshold NVS persistence runs on a single worker so the flash erase/write
// (tens of ms) never blocks the MQTT event loop, and rapid successive commands
// can't spawn concurrent writers racing each other (last-set value always wins:
// the worker drains until what it wrote matches the latest request). Both
// fields are guarded by s_mutex.
static int  s_persist_mv      = -1;     // latest threshold awaiting persistence
static bool s_persist_running = false;

static void threshold_persist_task(void *arg)
{
    int last_written = -1;
    for (;;) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        int mv = s_persist_mv;
        if (mv == last_written) {
            s_persist_running = false;
            xSemaphoreGive(s_mutex);
            break;
        }
        xSemaphoreGive(s_mutex);

        esp_err_t err = nvs_store_set_u16(NVS_NS_PUMP, NVS_KEY_THRESHOLD, (uint16_t)mv);
        ESP_LOGI(TAG, "Feedback threshold persisted: %d mV (%s)",
                 mv, err == ESP_OK ? "ok" : "failed");
        last_written = mv;
    }
    vTaskDelete(NULL);
}

esp_err_t pump_set_threshold_mv(int mv)
{
    if (mv < 100 || mv > 3300) return ESP_ERR_INVALID_ARG;
    if (!s_mutex) return ESP_FAIL;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_threshold_mv = mv;
    s_persist_mv   = mv;
    bool spawn = !s_persist_running;
    if (spawn) s_persist_running = true;
    xSemaphoreGive(s_mutex);

    if (spawn && xTaskCreate(threshold_persist_task, "thr_nvs", 3072,
                             NULL, 3, NULL) != pdPASS) {
        // Worker couldn't start (out of memory): write inline as a fallback.
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_persist_running = false;
        xSemaphoreGive(s_mutex);
        nvs_store_set_u16(NVS_NS_PUMP, NVS_KEY_THRESHOLD, (uint16_t)mv);
    }
    ESP_LOGI(TAG, "Feedback threshold set: %d mV", mv);
    return ESP_OK;
}

void pump_note_failsafe_off(void)
{
    if (!s_mutex) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_failsafe_off = true;
    s_reading.failsafe_off = true;
    xSemaphoreGive(s_mutex);
}

// ---- Pressure safety --------------------------------------------------------

void pump_get_pressure_limits(float *min_psi, float *max_psi, float *critical_psi)
{
    if (!s_mutex) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (min_psi)      *min_psi      = s_min_psi;
    if (max_psi)      *max_psi      = s_max_psi;
    if (critical_psi) *critical_psi = s_critical_psi;
    xSemaphoreGive(s_mutex);
}

esp_err_t pump_set_pressure_limits(float min_psi, float max_psi, float critical_psi)
{
    // Must be ordered and in range — a degenerate set could disable a check or
    // (worse) trip constantly.
    if (!(min_psi >= PUMP_PRESSURE_PSI_FLOOR &&
          min_psi < max_psi && max_psi < critical_psi &&
          critical_psi <= PUMP_PRESSURE_PSI_CEIL)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_mutex) return ESP_FAIL;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    // Unchanged? Skip — the cloud re-pushes these on every reconnect, so a
    // network flap must not churn the flash. Compare on the stored deci-psi
    // resolution so sub-0.1 psi float noise doesn't count as a change.
    bool unchanged = PSI_TO_DPSI(s_min_psi)      == PSI_TO_DPSI(min_psi) &&
                     PSI_TO_DPSI(s_max_psi)      == PSI_TO_DPSI(max_psi) &&
                     PSI_TO_DPSI(s_critical_psi) == PSI_TO_DPSI(critical_psi);
    s_min_psi      = min_psi;
    s_max_psi      = max_psi;
    s_critical_psi = critical_psi;
    xSemaphoreGive(s_mutex);

    if (unchanged) return ESP_OK;

    // Persisted inline: pressure-limit changes are rare config pushes, so the
    // brief flash write is acceptable (unlike the per-command feedback threshold).
    nvs_store_set_u16(NVS_NS_PUMP, NVS_KEY_MIN_PSI,  PSI_TO_DPSI(min_psi));
    nvs_store_set_u16(NVS_NS_PUMP, NVS_KEY_MAX_PSI,  PSI_TO_DPSI(max_psi));
    nvs_store_set_u16(NVS_NS_PUMP, NVS_KEY_CRIT_PSI, PSI_TO_DPSI(critical_psi));
    ESP_LOGI(TAG, "Pressure limits set: min=%.1f warn=%.1f critical=%.1f psi",
             min_psi, max_psi, critical_psi);
    return ESP_OK;
}

int pump_get_prime_grace_s(void)
{
    if (!s_mutex) return s_prime_grace_s;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int v = s_prime_grace_s;
    xSemaphoreGive(s_mutex);
    return v;
}

esp_err_t pump_set_prime_grace_s(int seconds)
{
    if (seconds < PUMP_PRIME_GRACE_S_MIN || seconds > PUMP_PRIME_GRACE_S_MAX)
        return ESP_ERR_INVALID_ARG;
    if (!s_mutex) return ESP_FAIL;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool unchanged = (s_prime_grace_s == seconds);
    s_prime_grace_s = seconds;
    xSemaphoreGive(s_mutex);

    // Skip the flash write on a no-op re-push (cloud re-sends on reconnect).
    if (unchanged) return ESP_OK;

    nvs_store_set_u16(NVS_NS_PUMP, NVS_KEY_PRIME_S, (uint16_t)seconds);
    ESP_LOGI(TAG, "Prime grace window set: %ds", seconds);
    return ESP_OK;
}

bool pump_get_pressure_loss_enabled(void)
{
    if (!s_mutex) return s_pressure_loss_en;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool v = s_pressure_loss_en;
    xSemaphoreGive(s_mutex);
    return v;
}

esp_err_t pump_set_pressure_loss_enabled(bool enabled)
{
    if (!s_mutex) return ESP_FAIL;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool unchanged = (s_pressure_loss_en == enabled);
    s_pressure_loss_en = enabled;
    xSemaphoreGive(s_mutex);

    if (unchanged) return ESP_OK;

    nvs_store_set_u16(NVS_NS_PUMP, NVS_KEY_LOSS_EN, enabled ? 1 : 0);
    ESP_LOGI(TAG, "Mid-run pressure-loss rule %s", enabled ? "enabled" : "disabled");
    return ESP_OK;
}

void pump_trip_lockout(pump_safety_reason_t reason)
{
    if (!s_mutex || reason == PUMP_SAFETY_OK) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool was = s_safety_lockout;
    s_safety_lockout       = true;
    s_safety_reason        = reason;
    s_reading.safety_lockout = true;
    s_reading.safety_reason  = reason;
    xSemaphoreGive(s_mutex);

    if (!was) {
        // Persist so the lockout survives a reboot (a tripped safety stays
        // tripped). Only write on the rising edge to spare the flash.
        nvs_store_set_u16(NVS_NS_PUMP, NVS_KEY_LOCKOUT, (uint16_t)reason);
        ESP_LOGW(TAG, "SAFETY LOCKOUT (reason %d) — pump will not start until reset", reason);
    }
}

void pump_reset_lockout(void)
{
    if (!s_mutex) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool was = s_safety_lockout;
    s_safety_lockout         = false;
    s_safety_reason          = PUMP_SAFETY_OK;
    s_pressure_warning       = false;
    s_reading.safety_lockout = false;
    s_reading.safety_reason  = PUMP_SAFETY_OK;
    s_reading.pressure_warning = false;
    xSemaphoreGive(s_mutex);

    if (was) {
        nvs_store_set_u16(NVS_NS_PUMP, NVS_KEY_LOCKOUT, 0);
        ESP_LOGI(TAG, "Safety lockout reset");
    }
}

bool pump_is_locked_out(void)
{
    if (!s_mutex) return false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool locked = s_safety_lockout;
    xSemaphoreGive(s_mutex);
    return locked;
}

void pump_set_pressure_warning(bool on)
{
    if (!s_mutex) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_pressure_warning       = on;
    s_reading.pressure_warning = on;
    xSemaphoreGive(s_mutex);
}

const char *pump_safety_reason_str(int reason)
{
    switch (reason) {
        case PUMP_SAFETY_LOW_PRESSURE:  return "low_pressure";
        case PUMP_SAFETY_HIGH_PRESSURE: return "high_pressure";
        case PUMP_SAFETY_PRESSURE_LOSS: return "pressure_loss";
        default:                        return "none";
    }
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

    // {"reset_failsafe":true} — clear a tripped safety lockout.
    const char *rf = strstr(buf, "\"reset_failsafe\"");
    if (rf) {
        const char *colon = strchr(rf, ':');
        if (colon && parse_bool_token(colon) == 1) pump_reset_lockout();
    }

    // {"min_psi":5,"max_psi":25,"critical_psi":30} — cloud-pushed pressure
    // limits. All three are required; a partial set is ignored (so we never
    // apply a half-updated, mis-ordered config).
    const char *mn = strstr(buf, "\"min_psi\"");
    const char *mx = strstr(buf, "\"max_psi\"");
    const char *cr = strstr(buf, "\"critical_psi\"");
    if (mn && mx && cr) {
        const char *cmn = strchr(mn, ':');
        const char *cmx = strchr(mx, ':');
        const char *ccr = strchr(cr, ':');
        if (cmn && cmx && ccr) {
            float fmin = strtof(cmn + 1, NULL);
            float fmax = strtof(cmx + 1, NULL);
            float fcrit = strtof(ccr + 1, NULL);
            pump_set_pressure_limits(fmin, fmax, fcrit);  // validates ordering/range
        }
    }

    // {"prime_grace_s":15} — cloud-pushed prime grace window (seconds).
    const char *pg = strstr(buf, "\"prime_grace_s\"");
    if (pg) {
        const char *colon = strchr(pg, ':');
        if (colon) {
            long v = strtol(colon + 1, NULL, 10);
            pump_set_prime_grace_s((int)v);  // validates range
        }
    }

    // {"pressure_loss_en":true} — cloud-pushed mid-run pressure-loss switch.
    const char *pl = strstr(buf, "\"pressure_loss_en\"");
    if (pl) {
        const char *colon = strchr(pl, ':');
        if (colon) {
            int b = parse_bool_token(colon);
            if (b >= 0) pump_set_pressure_loss_enabled(b == 1);
        }
    }
}
