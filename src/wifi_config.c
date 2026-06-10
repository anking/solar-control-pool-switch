#include "wifi_config.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "wifi_config";

#define NVS_NAMESPACE  "wifi_cfg"
#define NVS_KEY_SSID   "ssid"
#define NVS_KEY_PASS   "pass"

bool wifi_config_get(char *ssid, size_t ssid_len, char *password, size_t pass_len)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }

    size_t len = ssid_len;
    esp_err_t err = nvs_get_str(h, NVS_KEY_SSID, ssid, &len);
    if (err != ESP_OK || len <= 1) {
        nvs_close(h);
        return false;
    }

    len = pass_len;
    err = nvs_get_str(h, NVS_KEY_PASS, password, &len);
    if (err != ESP_OK) {
        nvs_close(h);
        return false;
    }

    nvs_close(h);
    ESP_LOGI(TAG, "Loaded WiFi config: SSID=\"%s\"", ssid);
    return true;
}

esp_err_t wifi_config_set(const char *ssid, const char *password)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_str(h, NVS_KEY_SSID, ssid);
    if (err != ESP_OK) goto fail;

    err = nvs_set_str(h, NVS_KEY_PASS, password ? password : "");
    if (err != ESP_OK) goto fail;

    err = nvs_commit(h);
    if (err != ESP_OK) goto fail;

    nvs_close(h);
    ESP_LOGI(TAG, "Saved WiFi config: SSID=\"%s\"", ssid);
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "NVS write failed: %s", esp_err_to_name(err));
    nvs_close(h);
    return err;
}

esp_err_t wifi_config_clear(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "WiFi config cleared");
    return ESP_OK;
}
