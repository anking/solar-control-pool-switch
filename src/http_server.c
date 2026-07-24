#include "http_server.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/param.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "wifi_manager.h"
#include "wifi_config.h"
#include "pump.h"
#include "mqtt_bridge.h"
#include "led_status.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "http_server";
static httpd_handle_t server_handle = NULL;

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

// ---- WebSocket client tracking ----------------------------------------------
#define STATUS_WS_MAX 4
static int s_status_ws_fds[STATUS_WS_MAX];
static size_t s_status_ws_count = 0;
static SemaphoreHandle_t s_status_ws_mutex = NULL;

static void ws_mutex_init(void)
{
    if (!s_status_ws_mutex) s_status_ws_mutex = xSemaphoreCreateMutex();
}

static void ws_add_client(int fd)
{
    if (!s_status_ws_mutex) return;
    if (xSemaphoreTake(s_status_ws_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    if (s_status_ws_count < STATUS_WS_MAX) {
        for (size_t i = 0; i < s_status_ws_count; i++) {
            if (s_status_ws_fds[i] == fd) { xSemaphoreGive(s_status_ws_mutex); return; }
        }
        s_status_ws_fds[s_status_ws_count++] = fd;
    }
    xSemaphoreGive(s_status_ws_mutex);
}

static void ws_remove_client(int fd)
{
    if (!s_status_ws_mutex) return;
    if (xSemaphoreTake(s_status_ws_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    for (size_t i = 0; i < s_status_ws_count; i++) {
        if (s_status_ws_fds[i] == fd) {
            s_status_ws_fds[i] = s_status_ws_fds[s_status_ws_count - 1];
            s_status_ws_count--;
            break;
        }
    }
    xSemaphoreGive(s_status_ws_mutex);
}

void http_server_ws_broadcast_status(const char *json, size_t len)
{
    if (!s_status_ws_mutex || !server_handle || !json || len == 0) return;
    int local_fds[STATUS_WS_MAX];
    size_t n = 0;

    if (xSemaphoreTake(s_status_ws_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    n = s_status_ws_count;
    for (size_t i = 0; i < n; i++) local_fds[i] = s_status_ws_fds[i];
    xSemaphoreGive(s_status_ws_mutex);

    httpd_ws_frame_t frame = {
        .payload = (uint8_t *)json,
        .len = len,
        .type = HTTPD_WS_TYPE_TEXT,
    };
    for (size_t i = 0; i < n; i++) {
        esp_err_t err = httpd_ws_send_frame_async(server_handle, local_fds[i], &frame);
        if (err != ESP_OK) ws_remove_client(local_fds[i]);
    }
}

// ---- JSON helpers -----------------------------------------------------------

static bool json_extract_str(const char *json, const char *key, char *out, size_t out_len)
{
    char search[64];
    snprintf(search, sizeof(search), "\"%s\"", key);
    const char *p = strstr(json, search);
    if (!p) return false;
    p = strchr(p + strlen(search), '"');
    if (!p) return false;
    p++;
    const char *end = strchr(p, '"');
    if (!end) return false;
    size_t len = end - p;
    if (len >= out_len) len = out_len - 1;
    strncpy(out, p, len);
    out[len] = '\0';
    return true;
}

static int json_extract_int(const char *json, const char *key, int default_val)
{
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return default_val;
    p += strlen(search);
    while (*p == ' ' || *p == '\t') p++;
    return atoi(p);
}

static bool json_extract_bool(const char *json, const char *key, bool *out)
{
    char search[64];
    snprintf(search, sizeof(search), "\"%s\"", key);
    const char *p = strstr(json, search);
    if (!p) return false;
    const char *colon = strchr(p + strlen(search), ':');
    if (!colon) return false;
    colon++;
    while (*colon == ' ' || *colon == '\t') colon++;
    if (strncmp(colon, "true", 4) == 0 || *colon == '1') { *out = true;  return true; }
    if (strncmp(colon, "false", 5) == 0 || *colon == '0') { *out = false; return true; }
    return false;
}

// Build the shared pump status JSON (used by /api/status and the WS push).
static int build_status_json(char *buf, size_t size)
{
    pump_reading_t r;
    pump_get(&r);
    return snprintf(buf, size,
        "{\"valid\":%s,\"commanded_on\":%s,\"feedback_on\":%s,\"mismatch\":%s,"
        "\"feedback_v\":%.3f,\"feedback_mv\":%d,\"raw_mv\":%d,\"peak_mv\":%d,"
        "\"saturated\":%s,\"threshold_mv\":%d,"
        "\"pressure_psi\":%.1f,\"pressure_v\":%.3f,\"pressure_valid\":%s,\"pressure_gpio\":%d,"
        "\"failsafe_off\":%s,"
        "\"safety_lockout\":%s,\"safety_reason\":\"%s\",\"pressure_warning\":%s,"
        "\"min_psi\":%.1f,\"max_psi\":%.1f,\"critical_psi\":%.1f,"
        "\"prime_grace_s\":%d,\"pressure_loss_en\":%s,"
        "\"output_gpio\":%d,\"feedback_gpio\":%d,"
        "\"on_seconds\":%lu,\"samples\":%lu}",
        r.valid ? "true" : "false",
        r.commanded_on ? "true" : "false",
        r.feedback_on ? "true" : "false",
        r.mismatch ? "true" : "false",
        r.feedback_v, r.feedback_mv, r.raw_mv, r.peak_mv,
        r.saturated ? "true" : "false",
        r.threshold_mv,
        r.pressure_psi, r.pressure_v, r.pressure_valid ? "true" : "false", r.pressure_gpio,
        r.failsafe_off ? "true" : "false",
        r.safety_lockout ? "true" : "false",
        pump_safety_reason_str(r.safety_reason),
        r.pressure_warning ? "true" : "false",
        r.min_psi, r.max_psi, r.critical_psi,
        r.prime_grace_s, r.pressure_loss_enabled ? "true" : "false",
        r.output_gpio, r.feedback_gpio,
        (unsigned long)r.on_seconds,
        (unsigned long)r.sample_count);
}

// ---- Page + health ----------------------------------------------------------

static esp_err_t index_handler(httpd_req_t *req)
{
    size_t len = index_html_end - index_html_start;
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)index_html_start, len);
}

static esp_err_t health_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
}

// ---- WiFi -------------------------------------------------------------------

static const char *auth_mode_str(uint8_t mode)
{
    switch (mode) {
        case 0: return "OPEN";
        case 1: return "WEP";
        case 2: return "WPA_PSK";
        case 3: return "WPA2_PSK";
        case 4: return "WPA_WPA2_PSK";
        case 5: return "WPA2_ENTERPRISE";
        case 6: return "WPA3_PSK";
        case 7: return "WPA2_WPA3_PSK";
        default: return "UNKNOWN";
    }
}

static esp_err_t wifi_scan_handler(httpd_req_t *req)
{
    wifi_scan_result_t results[20];
    int count = wifi_manager_scan(results, 20);

    char *buf = malloc(4096);
    if (!buf) { httpd_resp_send_500(req); return ESP_FAIL; }

    int pos = 0;
    pos += snprintf(buf + pos, 4096 - pos, "[");
    for (int i = 0; i < count; i++) {
        if (i > 0) pos += snprintf(buf + pos, 4096 - pos, ",");
        pos += snprintf(buf + pos, 4096 - pos,
            "{\"ssid\":\"%s\",\"rssi\":%d,\"channel\":%d,\"auth\":\"%s\",\"open\":%s}",
            results[i].ssid, results[i].rssi, results[i].channel,
            auth_mode_str(results[i].authmode),
            results[i].authmode == 0 ? "true" : "false");
    }
    pos += snprintf(buf + pos, 4096 - pos, "]");

    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_sendstr(req, buf);
    free(buf);
    return ret;
}

static esp_err_t wifi_connect_handler(httpd_req_t *req)
{
    char buf[256] = {0};
    int received = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
        return ESP_FAIL;
    }
    buf[received] = '\0';

    char ssid[33] = {0};
    char password[65] = {0};
    json_extract_str(buf, "ssid", ssid, sizeof(ssid));
    json_extract_str(buf, "password", password, sizeof(password));

    if (ssid[0] == '\0') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID required");
        return ESP_FAIL;
    }

    esp_err_t err = wifi_config_set(ssid, password);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save credentials");
        return ESP_FAIL;
    }

    err = wifi_manager_set_sta_config(ssid, password);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to apply config");
        return ESP_FAIL;
    }

    char resp[128];
    snprintf(resp, sizeof(resp), "{\"status\":\"ok\",\"ssid\":\"%s\"}", ssid);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, resp);
}

static esp_err_t wifi_forget_handler(httpd_req_t *req)
{
    wifi_config_clear();
    wifi_manager_disconnect_sta();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
}

static esp_err_t wifi_status_handler(httpd_req_t *req)
{
    wifi_status_t status;
    wifi_manager_get_status(&status);

    char buf[512];
    snprintf(buf, sizeof(buf),
        "{\"connected\":%s,\"ssid\":\"%s\",\"ip\":\"%s\","
        "\"rssi\":%d,\"channel\":%d,"
        "\"ap_active\":%s,\"ap_ssid\":\"%s\",\"ap_ip\":\"%s\","
        "\"ap_clients\":%d,\"hostname\":\"%s\","
        "\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\"}",
        status.connected ? "true" : "false",
        status.ssid, status.ip, status.rssi, status.channel,
        status.ap_active ? "true" : "false",
        status.ap_ssid, status.ap_ip, status.ap_clients, status.hostname,
        status.mac[0], status.mac[1], status.mac[2],
        status.mac[3], status.mac[4], status.mac[5]);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

// ---- System info ------------------------------------------------------------

static esp_err_t system_info_handler(httpd_req_t *req)
{
    const esp_app_desc_t *app = esp_app_get_description();
    char buf[512];
    snprintf(buf, sizeof(buf),
        "{\"firmware_version\":\"%s\",\"git_hash\":\"%s\",\"idf_version\":\"%s\","
        "\"build_date\":\"%s\",\"build_time\":\"%s\","
        "\"free_heap\":%lu,\"min_free_heap\":%lu,"
        "\"board\":\"esp32-c3-supermini\",\"output_gpio\":%d,\"feedback_gpio\":%d}",
        FIRMWARE_VERSION, GIT_HASH, esp_get_idf_version(),
        app->date, app->time,
        (unsigned long)esp_get_free_heap_size(),
        (unsigned long)esp_get_minimum_free_heap_size(),
        PUMP_OUTPUT_GPIO, PUMP_FEEDBACK_GPIO);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

// ---- Pump status + control --------------------------------------------------

static esp_err_t api_status_handler(httpd_req_t *req)
{
    char buf[640];
    build_status_json(buf, sizeof(buf));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

static esp_err_t api_pump_post_handler(httpd_req_t *req)
{
    char body[128] = {0};
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body"); return ESP_FAIL; }

    bool toggle = false;
    bool on;
    bool reset = false;
    if (json_extract_bool(body, "reset_failsafe", &reset) && reset) {
        // Clear a tripped safety lockout locally — works even with the cloud
        // unreachable. (pump_set "on" is refused while locked out, so a reset
        // is the only way back without a reboot, and a reboot keeps it latched.)
        pump_reset_lockout();
    } else if (json_extract_bool(body, "toggle", &toggle) && toggle) {
        pump_toggle();
    } else if (json_extract_bool(body, "on", &on)) {
        pump_set(on);
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Provide {\"on\":true|false}, {\"toggle\":true} or {\"reset_failsafe\":true}");
        return ESP_FAIL;
    }

    // Push the new state out over MQTT right away so the cloud stays in sync.
    pump_reading_t r;
    pump_get(&r);
    mqtt_bridge_publish_state(&r);

    char buf[640];
    build_status_json(buf, sizeof(buf));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

static esp_err_t api_calibrate_handler(httpd_req_t *req)
{
    char body[128] = {0};
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body"); return ESP_FAIL; }

    int threshold = json_extract_int(body, "threshold_mv", -1);
    if (threshold < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Provide threshold_mv");
        return ESP_FAIL;
    }
    if (pump_set_threshold_mv(threshold) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "threshold_mv must be 100-3300");
        return ESP_FAIL;
    }

    // Re-publish info so the cloud picks up the new threshold immediately.
    mqtt_bridge_publish_info();

    char resp[64];
    snprintf(resp, sizeof(resp), "{\"threshold_mv\":%d}", pump_get_threshold_mv());
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, resp);
}

// ---- MQTT -------------------------------------------------------------------

static esp_err_t api_mqtt_get_handler(httpd_req_t *req)
{
    mqtt_status_t st;
    mqtt_bridge_get_status(&st);

    char safe_error[64] = {0};
    for (int i = 0, j = 0; st.error[i] && j < (int)sizeof(safe_error) - 1; i++) {
        char c = st.error[i];
        if (c >= ' ' && c != '"' && c != '\\') safe_error[j++] = c;
    }

    char buf[384];
    snprintf(buf, sizeof(buf),
        "{\"configured\":%s,\"connected\":%s,\"host\":\"%s\",\"port\":%d,\"mac\":\"%s\","
        "\"error\":\"%s\",\"pub_ok\":%lu,\"pub_fail\":%lu}",
        st.configured ? "true" : "false",
        st.connected ? "true" : "false",
        st.host, st.port, st.mac, safe_error,
        (unsigned long)st.publish_count, (unsigned long)st.publish_fail_count);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

static esp_err_t api_mqtt_post_handler(httpd_req_t *req)
{
    char body[256] = {0};
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body"); return ESP_FAIL; }

    char host[64] = {0}, user[32] = {0}, pass[64] = {0};
    json_extract_str(body, "host", host, sizeof(host));
    json_extract_str(body, "username", user, sizeof(user));
    json_extract_str(body, "password", pass, sizeof(pass));
    int port = json_extract_int(body, "port", 1883);

    if (host[0] == '\0') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "host required");
        return ESP_FAIL;
    }

    mqtt_bridge_configure(host, port, user, pass);

    mqtt_status_t st;
    mqtt_bridge_get_status(&st);
    char resp[256];
    snprintf(resp, sizeof(resp),
        "{\"configured\":%s,\"host\":\"%s\",\"port\":%d}",
        st.configured ? "true" : "false", st.host, st.port);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, resp);
}

static esp_err_t api_mqtt_delete_handler(httpd_req_t *req)
{
    mqtt_bridge_clear_config();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"configured\":false,\"connected\":false}");
}

// ---- LED mode ---------------------------------------------------------------

static esp_err_t api_led_get_handler(httpd_req_t *req)
{
    led_mode_t m = led_status_get_mode();
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"mode\":\"%s\"}", led_status_mode_name(m));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

static esp_err_t api_led_post_handler(httpd_req_t *req)
{
    char body[64] = {0};
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body"); return ESP_FAIL; }

    char mode_str[16] = {0};
    json_extract_str(body, "mode", mode_str, sizeof(mode_str));

    led_mode_t mode;
    if      (strcmp(mode_str, "errors") == 0) mode = LED_MODE_ERRORS_ONLY;
    else if (strcmp(mode_str, "off")    == 0) mode = LED_MODE_OFF;
    else if (strcmp(mode_str, "debug")  == 0) mode = LED_MODE_DEBUG;
    else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "mode must be errors|off|debug");
        return ESP_FAIL;
    }

    if (led_status_set_mode(mode) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Save failed");
        return ESP_FAIL;
    }

    char resp[64];
    snprintf(resp, sizeof(resp), "{\"mode\":\"%s\"}", led_status_mode_name(mode));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, resp);
}

// ---- Restart ----------------------------------------------------------------

static void reboot_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();
}

static esp_err_t api_restart_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Restart requested via API");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\",\"message\":\"Restarting...\"}");
    xTaskCreate(reboot_task, "reboot", 2048, NULL, 5, NULL);
    return ESP_OK;
}

// ---- OTA firmware update ----------------------------------------------------

#define OTA_RECV_BUF_SIZE 1024

static esp_err_t api_ota_handler(httpd_req_t *req)
{
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "OTA: no update partition (single-app build?)");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition available");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA: receiving %d bytes into '%s' @ 0x%08lx",
             req->content_len, update_partition->label,
             (unsigned long)update_partition->address);

    if (req->content_len > 0 && (size_t)req->content_len > update_partition->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Image larger than partition");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_begin failed");
        return ESP_FAIL;
    }

    char *buf = malloc(OTA_RECV_BUF_SIZE);
    if (!buf) {
        esp_ota_abort(ota_handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    int total = 0;
    while (remaining > 0) {
        int received = httpd_req_recv(req, buf, MIN(remaining, OTA_RECV_BUF_SIZE));
        if (received == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (received <= 0) {
            ESP_LOGE(TAG, "OTA: recv failed at %d bytes", total);
            free(buf);
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive failed");
            return ESP_FAIL;
        }
        err = esp_ota_write(ota_handle, buf, received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            free(buf);
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash write failed");
            return ESP_FAIL;
        }
        total += received;
        remaining -= received;
    }
    free(buf);

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        if (err == ESP_ERR_OTA_VALIDATE_FAILED)
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Image validation failed (not a valid firmware)");
        else
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_end failed");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Could not set boot partition");
        return ESP_FAIL;
    }

    // Read the version + build stamp embedded in the image we just flashed so
    // the UI can confirm exactly what's about to boot.
    char new_ver[32] = "unknown";
    char new_date[16] = "", new_time[16] = "";
    esp_app_desc_t new_desc;
    if (esp_ota_get_partition_description(update_partition, &new_desc) == ESP_OK) {
        snprintf(new_ver, sizeof(new_ver), "%.31s", new_desc.version);
        snprintf(new_date, sizeof(new_date), "%.15s", new_desc.date);
        snprintf(new_time, sizeof(new_time), "%.15s", new_desc.time);
    }

    ESP_LOGI(TAG, "OTA: %d bytes written, new firmware v%s (%s %s), booting '%s' after restart",
             total, new_ver, new_date, new_time, update_partition->label);
    char resp[256];
    snprintf(resp, sizeof(resp),
        "{\"status\":\"ok\",\"version\":\"%s\",\"running\":\"%s\","
        "\"build_date\":\"%s\",\"build_time\":\"%s\","
        "\"message\":\"Update applied, restarting...\"}",
        new_ver, FIRMWARE_VERSION, new_date, new_time);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    xTaskCreate(reboot_task, "reboot", 2048, NULL, 5, NULL);
    return ESP_OK;
}

// ---- WebSocket --------------------------------------------------------------

static esp_err_t ws_status_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        ws_add_client(fd);
        ESP_LOGI(TAG, "WS /ws client connected (fd=%d, count=%d)", fd, (int)s_status_ws_count);
        return ESP_OK;
    }

    httpd_ws_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    uint8_t buf[128];
    frame.payload = buf;
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, sizeof(buf));
    if (ret != ESP_OK) {
        int fd = httpd_req_to_sockfd(req);
        ws_remove_client(fd);
        ESP_LOGI(TAG, "WS /ws client disconnected (fd=%d)", fd);
    }
    return ret;
}

// ---- Server start -----------------------------------------------------------

esp_err_t http_server_start(void)
{
    if (server_handle != NULL) {
        ESP_LOGW(TAG, "Server already running");
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 24;
    config.max_open_sockets = 5;
    config.stack_size = 8192;
    config.lru_purge_enable = true;

    esp_err_t ret = httpd_start(&server_handle, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return ret;
    }

    httpd_uri_t routes[] = {
        { .uri = "/",                     .method = HTTP_GET,    .handler = index_handler },
        { .uri = "/health",               .method = HTTP_GET,    .handler = health_handler },
        { .uri = "/api/wifi/scan",        .method = HTTP_GET,    .handler = wifi_scan_handler },
        { .uri = "/api/wifi/connect",     .method = HTTP_POST,   .handler = wifi_connect_handler },
        { .uri = "/api/wifi/forget",      .method = HTTP_POST,   .handler = wifi_forget_handler },
        { .uri = "/api/wifi/status",      .method = HTTP_GET,    .handler = wifi_status_handler },
        { .uri = "/api/system",           .method = HTTP_GET,    .handler = system_info_handler },
        { .uri = "/api/status",           .method = HTTP_GET,    .handler = api_status_handler },
        { .uri = "/api/pump",             .method = HTTP_POST,   .handler = api_pump_post_handler },
        { .uri = "/api/calibrate",        .method = HTTP_POST,   .handler = api_calibrate_handler },
        { .uri = "/api/mqtt",             .method = HTTP_GET,    .handler = api_mqtt_get_handler },
        { .uri = "/api/mqtt",             .method = HTTP_POST,   .handler = api_mqtt_post_handler },
        { .uri = "/api/mqtt",             .method = HTTP_DELETE, .handler = api_mqtt_delete_handler },
        { .uri = "/api/led",              .method = HTTP_GET,    .handler = api_led_get_handler },
        { .uri = "/api/led",              .method = HTTP_POST,   .handler = api_led_post_handler },
        { .uri = "/api/restart",          .method = HTTP_POST,   .handler = api_restart_handler },
        { .uri = "/api/ota",              .method = HTTP_POST,   .handler = api_ota_handler },
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        httpd_register_uri_handler(server_handle, &routes[i]);
    }

    ws_mutex_init();
    httpd_uri_t ws_status_uri = {
        .uri = "/ws", .method = HTTP_GET, .handler = ws_status_handler,
        .is_websocket = true,
    };
    httpd_register_uri_handler(server_handle, &ws_status_uri);

    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
    return ESP_OK;
}

void http_server_stop(void)
{
    if (server_handle) {
        httpd_stop(server_handle);
        server_handle = NULL;
        ESP_LOGI(TAG, "HTTP server stopped");
    }
}

httpd_handle_t http_server_get_handle(void)
{
    return server_handle;
}
