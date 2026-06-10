#include "nvs_store.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "nvs_store";

esp_err_t nvs_store_get_str(const char *ns, const char *key, char *out, size_t max_len)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(ns, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    size_t len = max_len;
    err = nvs_get_str(h, key, out, &len);
    nvs_close(h);
    return err;
}

esp_err_t nvs_store_set_str(const char *ns, const char *key, const char *value)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_str(h, key, value);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

esp_err_t nvs_store_get_u16(const char *ns, const char *key, uint16_t *out)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(ns, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    err = nvs_get_u16(h, key, out);
    nvs_close(h);
    return err;
}

esp_err_t nvs_store_set_u16(const char *ns, const char *key, uint16_t value)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_u16(h, key, value);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

esp_err_t nvs_store_get_blob(const char *ns, const char *key, void *out, size_t *len)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(ns, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    err = nvs_get_blob(h, key, out, len);
    nvs_close(h);
    return err;
}

esp_err_t nvs_store_set_blob(const char *ns, const char *key, const void *data, size_t len)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(h, key, data, len);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

esp_err_t nvs_store_erase_ns(const char *ns)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Erased namespace: %s", ns);
    return ESP_OK;
}
