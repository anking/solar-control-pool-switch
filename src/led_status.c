#include "led_status.h"
#include "nvs_store.h"
#include "pump.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "led_status";

static volatile led_wifi_status_t s_wifi_status = LED_STATUS_WIFI_OFF;
static volatile bool              s_ap_active   = false;
static volatile led_mode_t        s_mode        = LED_MODE_ERRORS_ONLY;
static TaskHandle_t               s_task        = NULL;

static inline void led_drive(bool on) {
#if LED_ACTIVE_LOW
    gpio_set_level(LED_GPIO, on ? 0 : 1);
#else
    gpio_set_level(LED_GPIO, on ? 1 : 0);
#endif
}

static void flash(int on_ms) {
    led_drive(true);
    vTaskDelay(pdMS_TO_TICKS(on_ms));
    led_drive(false);
}

// LED_MODE_DEBUG mirrors the original "WiFi indicator" behavior.
static void tick_debug(void) {
    switch (s_wifi_status) {
        case LED_STATUS_WIFI_CONNECTED:
            led_drive(true);
            vTaskDelay(pdMS_TO_TICKS(200));
            break;
        case LED_STATUS_WIFI_CONNECTING:
            led_drive(true);  vTaskDelay(pdMS_TO_TICKS(1000));
            led_drive(false); vTaskDelay(pdMS_TO_TICKS(1000));
            break;
        case LED_STATUS_WIFI_RETRYING:
            led_drive(true);  vTaskDelay(pdMS_TO_TICKS(200));
            led_drive(false); vTaskDelay(pdMS_TO_TICKS(200));
            break;
        case LED_STATUS_WIFI_DISCONNECTED:
        case LED_STATUS_WIFI_OFF:
        default:
            led_drive(false);
            vTaskDelay(pdMS_TO_TICKS(200));
            break;
    }
}

// LED_MODE_ERRORS_ONLY: priority is pump fault > WiFi fault > AP fallback > normal.
// Each branch ends with a long "off" period so the duty cycle stays tiny.
static void tick_errors_only(void) {
    bool pump_fault   = pump_has_fault();
    bool wifi_fault   = (s_wifi_status == LED_STATUS_WIFI_RETRYING ||
                         s_wifi_status == LED_STATUS_WIFI_DISCONNECTED);
    bool sta_connected = (s_wifi_status == LED_STATUS_WIFI_CONNECTED);
    bool ap_only      = s_ap_active && !sta_connected;

    if (pump_fault) {
        // Three quick blinks: relay/feedback disagree (stuck or broken wire).
        flash(80); vTaskDelay(pdMS_TO_TICKS(120));
        flash(80); vTaskDelay(pdMS_TO_TICKS(120));
        flash(80); vTaskDelay(pdMS_TO_TICKS(3000));
    } else if (wifi_fault) {
        flash(80); vTaskDelay(pdMS_TO_TICKS(2000));
    } else if (ap_only) {
        flash(80); vTaskDelay(pdMS_TO_TICKS(5000));
    } else {
        led_drive(false);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void led_task(void *pvParameters) {
    // Boot flash: two quick blinks to confirm the board's alive.
    flash(120); vTaskDelay(pdMS_TO_TICKS(120));
    flash(120); vTaskDelay(pdMS_TO_TICKS(120));
    led_drive(false);

    while (1) {
        switch (s_mode) {
            case LED_MODE_OFF:
                led_drive(false);
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            case LED_MODE_DEBUG:
                tick_debug();
                break;
            case LED_MODE_ERRORS_ONLY:
            default:
                tick_errors_only();
                break;
        }
    }
}

esp_err_t led_status_init(void) {
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_GPIO),
        .pull_down_en = 0,
        .pull_up_en   = 0,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LED GPIO");
        return ret;
    }
    led_drive(false);

    // Load persisted mode (default = ERRORS_ONLY).
    uint16_t stored;
    if (nvs_store_get_u16(NVS_NS_DEVICE, "led_mode", &stored) == ESP_OK) {
        if (stored <= LED_MODE_DEBUG) s_mode = (led_mode_t)stored;
    }

    xTaskCreate(led_task, "led_status", 2048, NULL, 5, &s_task);

    ESP_LOGI(TAG, "LED ready (GPIO%d, active %s, mode: %s)",
             LED_GPIO, LED_ACTIVE_LOW ? "LOW" : "HIGH",
             led_status_mode_name(s_mode));
    return ESP_OK;
}

void led_status_set_wifi(led_wifi_status_t status) {
    s_wifi_status = status;
}

void led_status_set_ap_active(bool active) {
    s_ap_active = active;
}

led_mode_t led_status_get_mode(void) {
    return s_mode;
}

esp_err_t led_status_set_mode(led_mode_t mode) {
    if (mode > LED_MODE_DEBUG) return ESP_ERR_INVALID_ARG;
    s_mode = mode;
    esp_err_t err = nvs_store_set_u16(NVS_NS_DEVICE, "led_mode", (uint16_t)mode);
    ESP_LOGI(TAG, "LED mode set: %s (%s)",
             led_status_mode_name(mode),
             err == ESP_OK ? "saved" : "save failed");
    return err;
}

const char *led_status_mode_name(led_mode_t mode) {
    switch (mode) {
        case LED_MODE_OFF:         return "off";
        case LED_MODE_DEBUG:       return "debug";
        case LED_MODE_ERRORS_ONLY: return "errors";
        default:                   return "unknown";
    }
}

void led_status_set(bool on) {
    led_drive(on);
}
