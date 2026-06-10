#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

bool wifi_config_get(char *ssid, size_t ssid_len, char *password, size_t pass_len);
esp_err_t wifi_config_set(const char *ssid, const char *password);
esp_err_t wifi_config_clear(void);

#endif // WIFI_CONFIG_H
