#ifndef LED_STATUS_H
#define LED_STATUS_H

#include <stdbool.h>
#include "esp_err.h"
#include "config.h"

// WiFi state hints, fed from wifi_manager. The LED task interprets these
// together with the pump fault state and the user-selected mode below.
typedef enum {
    LED_STATUS_WIFI_CONNECTED,      // STA is up
    LED_STATUS_WIFI_CONNECTING,     // first connect attempt
    LED_STATUS_WIFI_RETRYING,       // retrying after failure
    LED_STATUS_WIFI_DISCONNECTED,   // gave up — STA configured but unreachable
    LED_STATUS_WIFI_OFF             // initial, or AP-only (use set_ap_active)
} led_wifi_status_t;

// User-selectable LED behavior. ERRORS_ONLY is the default.
typedef enum {
    LED_MODE_ERRORS_ONLY = 0,
    LED_MODE_OFF         = 1,
    LED_MODE_DEBUG       = 2,
} led_mode_t;

esp_err_t  led_status_init(void);

void       led_status_set_wifi(led_wifi_status_t status);
void       led_status_set_ap_active(bool active);

led_mode_t led_status_get_mode(void);
esp_err_t  led_status_set_mode(led_mode_t mode);
const char *led_status_mode_name(led_mode_t mode);

// Direct GPIO control (for bring-up only — overwritten by the task next tick).
void       led_status_set(bool on);

#endif // LED_STATUS_H
