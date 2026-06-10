#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    bool connected;
    char ssid[33];
    char hostname[32];
    int8_t rssi;
    uint8_t channel;
    char ip[16];
    char gateway[16];
    char netmask[16];
    char dns[16];
    uint8_t mac[6];
    uint32_t reconnect_attempts;
    int last_disconnect_reason;
    bool ap_active;
    char ap_ssid[33];
    char ap_ip[16];
    uint8_t ap_clients;
} wifi_status_t;

typedef struct {
    char ssid[33];
    int8_t rssi;
    uint8_t channel;
    uint8_t authmode;
} wifi_scan_result_t;

esp_err_t wifi_manager_init(void);
esp_err_t wifi_manager_start(const char *ssid, const char *password);
void wifi_manager_get_status(wifi_status_t *status);
bool wifi_manager_is_connected(void);
bool wifi_manager_ap_is_active(void);
int wifi_manager_scan(wifi_scan_result_t *results, int max_results);
esp_err_t wifi_manager_set_sta_config(const char *ssid, const char *password);
void wifi_manager_disconnect_sta(void);

#endif // WIFI_MANAGER_H
