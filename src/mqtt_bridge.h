#ifndef MQTT_BRIDGE_H
#define MQTT_BRIDGE_H

#include "esp_err.h"
#include "pump.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

esp_err_t mqtt_bridge_init(void);
bool mqtt_bridge_is_connected(void);
// True once a broker host has been configured (vs. local-only operation).
bool mqtt_bridge_is_configured(void);

// Publish the current pump state to pumps/{mac}/state (retained, QoS 1 — so a
// late MQTT subscriber always sees the latest commanded/feedback/fault state).
void mqtt_bridge_publish_state(const pump_reading_t *reading);

// Clear the retained command on pumps/{mac}/cmd (publishes an empty retained
// payload). Used after consuming a one-shot command so it doesn't re-fire.
void mqtt_bridge_clear_retained_cmd(void);

// Publishes pumps/{mac}/info (model, firmware, output/feedback GPIO, threshold).
// Auto-called on MQTT_EVENT_CONNECTED; expose publicly so the API can re-send
// after a config change.
void mqtt_bridge_publish_info(void);

esp_err_t mqtt_bridge_configure(const char *host, int port, const char *user, const char *pass);
esp_err_t mqtt_bridge_disconnect(void);
esp_err_t mqtt_bridge_clear_config(void);

typedef struct {
    bool     configured;
    bool     connected;
    char     host[64];
    int      port;
    char     mac[18];
    char     error[64];
    uint32_t publish_count;
    uint32_t publish_fail_count;
} mqtt_status_t;

void mqtt_bridge_get_status(mqtt_status_t *out);

#endif // MQTT_BRIDGE_H
