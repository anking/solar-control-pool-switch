#ifndef NVS_STORE_H
#define NVS_STORE_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#define NVS_NS_WIFI         "wifi_cfg"
#define NVS_NS_MQTT         "mqtt_cfg"
#define NVS_NS_PUMP         "pump_cfg"
#define NVS_NS_DEVICE       "device_cfg"

esp_err_t nvs_store_get_str(const char *ns, const char *key, char *out, size_t max_len);
esp_err_t nvs_store_set_str(const char *ns, const char *key, const char *value);
esp_err_t nvs_store_get_u16(const char *ns, const char *key, uint16_t *out);
esp_err_t nvs_store_set_u16(const char *ns, const char *key, uint16_t value);
esp_err_t nvs_store_get_blob(const char *ns, const char *key, void *out, size_t *len);
esp_err_t nvs_store_set_blob(const char *ns, const char *key, const void *data, size_t len);
esp_err_t nvs_store_erase_ns(const char *ns);

#endif // NVS_STORE_H
