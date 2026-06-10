#include "wifi_manager.h"
#include <string.h>
#include <stdio.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "mdns.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "led_status.h"
#include <time.h>

static const char *TAG = "wifi_manager";

static wifi_status_t g_wifi_status = {0};
static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;
static const int WIFI_FAIL_BIT = BIT1;
static esp_event_handler_instance_t s_instance_any_id = NULL;
static esp_event_handler_instance_t s_instance_got_ip = NULL;
static bool s_sntp_started = false;
static bool s_mdns_started = false;

static esp_netif_t *s_sta_netif = NULL;
static esp_netif_t *s_ap_netif  = NULL;

static volatile bool s_ap_active       = false;
static volatile int  s_ap_client_count = 0;
static TaskHandle_t  s_ap_task_handle  = NULL;

// STA recovery + health monitoring.
static volatile bool s_has_sta_credentials = false;
static volatile bool s_ever_connected      = false;
static TaskHandle_t  s_reconnect_task_handle = NULL;
static TaskHandle_t  s_watchdog_task_handle  = NULL;

#define AP_INITIAL_KEEP_MS       (5 * 60 * 1000)
#define AP_POST_CONNECT_KEEP_MS  (5 * 60 * 1000)
#define AP_CHECK_INTERVAL_MS     5000
#define AP_MAX_CONNECTIONS       4
#define AP_CHANNEL               1

// Reconnect backoff: 1s → 2s → 4s → 8s → 16s → 30s (cap).
#define RECONNECT_BACKOFF_INITIAL_MS  1000
#define RECONNECT_BACKOFF_MAX_MS      30000

// Watchdog: poll AP info this often. If query keeps failing while we think
// we're connected, force a reconnect. If we stay offline this long, reboot.
#define WATCHDOG_CHECK_INTERVAL_MS    30000
#define WATCHDOG_SILENT_LOSS_FAILS    4              // 4 × 30s = 2 min
#define WATCHDOG_OFFLINE_REBOOT_MS    (10 * 60 * 1000)

#define MDNS_HOSTNAME_AP   "poolpump"

static char s_hostname_sta[32] = {0};

static void update_mdns_hostname(bool sta_connected)
{
    const char *name = sta_connected ? s_hostname_sta : MDNS_HOSTNAME_AP;

    if (!s_mdns_started) {
        esp_err_t err = mdns_init();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "mDNS init failed: %s", esp_err_to_name(err));
            return;
        }
        s_mdns_started = true;
        (void)mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    }

    (void)mdns_hostname_set(name);
    (void)mdns_instance_name_set(name);

    strncpy(g_wifi_status.hostname, name, sizeof(g_wifi_status.hostname) - 1);
    g_wifi_status.hostname[sizeof(g_wifi_status.hostname) - 1] = '\0';

    ESP_LOGI(TAG, "mDNS hostname: http://%s.local/", name);
}

static void start_sntp_if_needed(void)
{
    if (s_sntp_started) return;
    s_sntp_started = true;

    setenv("TZ", "UTC0", 1);
    tzset();

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    ESP_LOGI(TAG, "SNTP started (pool.ntp.org)");
}

static void wifi_manager_stop_ap(void)
{
    if (!s_ap_active) return;

    ESP_LOGI(TAG, "Stopping soft AP...");
    s_ap_active = false;
    g_wifi_status.ap_active  = false;
    g_wifi_status.ap_clients = 0;
    led_status_set_ap_active(false);

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to switch to STA mode: %s", esp_err_to_name(err));
    }

    s_ap_client_count = 0;
    ESP_LOGI(TAG, "Soft AP stopped");
}

static void ap_management_task(void *arg)
{
    int64_t boot_ms       = esp_timer_get_time() / 1000;
    int64_t ap_keep_until = boot_ms + AP_INITIAL_KEEP_MS;
    bool    sta_prev      = g_wifi_status.connected;

    ESP_LOGI(TAG, "AP management: hotspot will stay active for at least 5 minutes");

    while (s_ap_active) {
        vTaskDelay(pdMS_TO_TICKS(AP_CHECK_INTERVAL_MS));
        if (!s_ap_active) break;

        int64_t now_ms  = esp_timer_get_time() / 1000;
        bool sta_now    = g_wifi_status.connected;
        int  clients    = s_ap_client_count;

        if (sta_now && !sta_prev) {
            int64_t ext = now_ms + AP_POST_CONNECT_KEEP_MS;
            if (ext > ap_keep_until) {
                ap_keep_until = ext;
                ESP_LOGI(TAG, "AP management: STA connected, AP extended 5 more minutes");
            }
        }
        sta_prev = sta_now;

        if (now_ms < ap_keep_until) continue;

        if (!sta_now) {
            ap_keep_until = now_ms + AP_POST_CONNECT_KEEP_MS;
            ESP_LOGI(TAG, "AP management: STA not connected, extending AP 5 more minutes");
            continue;
        }

        if (clients > 0) continue;

        ESP_LOGI(TAG, "AP management: STA connected, no AP clients — shutting down AP");
        wifi_manager_stop_ap();
        break;
    }

    s_ap_task_handle = NULL;
    vTaskDelete(NULL);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {

        case WIFI_EVENT_STA_START:
            led_status_set_wifi(LED_STATUS_WIFI_CONNECTING);
            // Kick off the very first connection attempt. All subsequent
            // retries are owned by wifi_reconnect_task — do NOT call
            // esp_wifi_connect() from the event handler again.
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *disc =
                (wifi_event_sta_disconnected_t *)event_data;
            g_wifi_status.last_disconnect_reason = disc->reason;
            g_wifi_status.connected = false;

            ESP_LOGW(TAG, "Wi-Fi disconnected, reason: %d", disc->reason);

            led_status_set_wifi(LED_STATUS_WIFI_RETRYING);
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            update_mdns_hostname(false);

            // On a clear credential/AP failure, signal the startup waiter so it
            // returns fast (and brings up the AP fallback) instead of always
            // blocking the full 30 s timeout. The reconnect task keeps retrying
            // regardless, in case the AP is only temporarily unreachable.
            if (disc->reason == WIFI_REASON_AUTH_FAIL ||
                disc->reason == WIFI_REASON_NO_AP_FOUND ||
                disc->reason == WIFI_REASON_AUTH_EXPIRE ||
                disc->reason == WIFI_REASON_HANDSHAKE_TIMEOUT) {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }

            // Hand off to the reconnect task. Never block the event loop.
            if (s_reconnect_task_handle) {
                xTaskNotifyGive(s_reconnect_task_handle);
            }
            break;
        }

        case WIFI_EVENT_AP_STACONNECTED: {
            wifi_event_ap_staconnected_t *ev =
                (wifi_event_ap_staconnected_t *)event_data;
            s_ap_client_count++;
            g_wifi_status.ap_clients = (uint8_t)s_ap_client_count;
            ESP_LOGI(TAG, "AP: station joined (MAC: " MACSTR ", AID: %d, total: %d)",
                     MAC2STR(ev->mac), ev->aid, s_ap_client_count);
            break;
        }

        case WIFI_EVENT_AP_STADISCONNECTED: {
            wifi_event_ap_stadisconnected_t *ev =
                (wifi_event_ap_stadisconnected_t *)event_data;
            if (s_ap_client_count > 0) s_ap_client_count--;
            g_wifi_status.ap_clients = (uint8_t)s_ap_client_count;
            ESP_LOGI(TAG, "AP: station left  (MAC: " MACSTR ", AID: %d, total: %d)",
                     MAC2STR(ev->mac), ev->aid, s_ap_client_count);
            break;
        }

        default:
            break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));

        snprintf(g_wifi_status.ip, sizeof(g_wifi_status.ip),
                 IPSTR, IP2STR(&event->ip_info.ip));
        snprintf(g_wifi_status.gateway, sizeof(g_wifi_status.gateway),
                 IPSTR, IP2STR(&event->ip_info.gw));
        snprintf(g_wifi_status.netmask, sizeof(g_wifi_status.netmask),
                 IPSTR, IP2STR(&event->ip_info.netmask));

        g_wifi_status.connected = true;
        s_ever_connected = true;
        led_status_set_wifi(LED_STATUS_WIFI_CONNECTED);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);

        start_sntp_if_needed();
        update_mdns_hostname(true);
    }
}

// ----------------------------------------------------------------------------
// Reconnect task: owns all STA reconnect attempts with exponential backoff.
// Driven by xTaskNotifyGive() from the disconnect handler. Runs even without
// notifications (poll loop) so it can catch silent stalls.
// ----------------------------------------------------------------------------
static void wifi_reconnect_task(void *arg)
{
    int backoff_ms = RECONNECT_BACKOFF_INITIAL_MS;

    while (1) {
        // Wake on disconnect notification OR after the current backoff window.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(backoff_ms));

        if (!s_has_sta_credentials) {
            backoff_ms = RECONNECT_BACKOFF_INITIAL_MS;
            continue;
        }
        if (g_wifi_status.connected) {
            backoff_ms = RECONNECT_BACKOFF_INITIAL_MS;
            continue;
        }

        g_wifi_status.reconnect_attempts++;
        ESP_LOGI(TAG, "Reconnect attempt #%lu (backoff was %d ms)",
                 (unsigned long)g_wifi_status.reconnect_attempts, backoff_ms);

        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK && err != ESP_ERR_WIFI_CONN) {
            ESP_LOGW(TAG, "esp_wifi_connect() returned %s", esp_err_to_name(err));
        }

        backoff_ms *= 2;
        if (backoff_ms > RECONNECT_BACKOFF_MAX_MS) {
            backoff_ms = RECONNECT_BACKOFF_MAX_MS;
        }
    }
}

// ----------------------------------------------------------------------------
// Watchdog task: detects two failure modes that the reconnect loop can't.
//   1. Silent link loss — we still think we're connected, but the radio /
//      router state is broken. We notice esp_wifi_sta_get_ap_info() failing
//      and force a disconnect so the normal recovery path kicks in.
//   2. Persistent total offline — STA hasn't come back for >10 min. Reboot.
// Only acts once we've ever successfully connected; avoids reboot loops on
// fresh installs where credentials may be wrong.
// ----------------------------------------------------------------------------
static void wifi_watchdog_task(void *arg)
{
    int64_t offline_since_us = 0;
    int     ap_info_fail_count = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(WATCHDOG_CHECK_INTERVAL_MS));

        if (!s_has_sta_credentials) {
            offline_since_us   = 0;
            ap_info_fail_count = 0;
            continue;
        }

        if (g_wifi_status.connected) {
            offline_since_us = 0;

            wifi_ap_record_t ap_info;
            esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
            if (err == ESP_OK) {
                ap_info_fail_count = 0;
            } else {
                ap_info_fail_count++;
                ESP_LOGW(TAG, "AP info query failed (%d/%d): %s",
                         ap_info_fail_count, WATCHDOG_SILENT_LOSS_FAILS,
                         esp_err_to_name(err));
                if (ap_info_fail_count >= WATCHDOG_SILENT_LOSS_FAILS) {
                    ESP_LOGE(TAG, "Silent link loss — forcing reconnect");
                    ap_info_fail_count = 0;
                    g_wifi_status.connected = false;
                    led_status_set_wifi(LED_STATUS_WIFI_RETRYING);
                    esp_wifi_disconnect();  // triggers normal recovery path
                }
            }
        } else {
            ap_info_fail_count = 0;
            int64_t now_us = esp_timer_get_time();
            if (offline_since_us == 0) {
                offline_since_us = now_us;
            }
            int64_t offline_ms = (now_us - offline_since_us) / 1000;

            if (s_ever_connected && offline_ms > WATCHDOG_OFFLINE_REBOOT_MS) {
                ESP_LOGE(TAG, "Offline for >%d ms after a prior connection — rebooting",
                         WATCHDOG_OFFLINE_REBOOT_MS);
                vTaskDelay(pdMS_TO_TICKS(200));  // let the log line flush
                esp_restart();
            }
        }
    }
}

esp_err_t wifi_manager_init(void)
{
    memset(&g_wifi_status, 0, sizeof(wifi_status_t));

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif  = esp_netif_create_default_wifi_ap();

    uint8_t mac[6] = {0};
    esp_err_t mac_err = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (mac_err == ESP_OK) {
        snprintf(s_hostname_sta, sizeof(s_hostname_sta),
                 "poolpump-%02x%02x%02x", mac[3], mac[4], mac[5]);
    } else {
        strncpy(s_hostname_sta, "poolpump", sizeof(s_hostname_sta) - 1);
    }

    ESP_ERROR_CHECK(esp_netif_set_hostname(s_sta_netif, s_hostname_sta));

    strncpy(g_wifi_status.hostname, MDNS_HOSTNAME_AP,
            sizeof(g_wifi_status.hostname) - 1);

    if (mac_err == ESP_OK) {
        snprintf(g_wifi_status.ap_ssid, sizeof(g_wifi_status.ap_ssid),
                 "PoolPump-%02X%02X%02X", mac[3], mac[4], mac[5]);
    } else {
        strncpy(g_wifi_status.ap_ssid, "PoolPump",
                sizeof(g_wifi_status.ap_ssid) - 1);
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,
        &wifi_event_handler, NULL, &s_instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP,
        &wifi_event_handler, NULL, &s_instance_got_ip));

    return ESP_OK;
}

esp_err_t wifi_manager_start(const char *ssid, const char *password)
{
    bool has_sta = (ssid != NULL && ssid[0] != '\0');

    if (has_sta) {
        strncpy(g_wifi_status.ssid, ssid, sizeof(g_wifi_status.ssid) - 1);
        g_wifi_status.ssid[sizeof(g_wifi_status.ssid) - 1] = '\0';
    }

    wifi_config_t ap_cfg = {
        .ap = {
            .channel        = AP_CHANNEL,
            .max_connection = AP_MAX_CONNECTIONS,
            .authmode       = WIFI_AUTH_OPEN,
            .pmf_cfg        = { .required = false },
        },
    };
    strncpy((char *)ap_cfg.ap.ssid, g_wifi_status.ap_ssid,
            sizeof(ap_cfg.ap.ssid) - 1);
    ap_cfg.ap.ssid_len = strlen(g_wifi_status.ap_ssid);

    if (has_sta) {
        wifi_config_t sta_cfg = {
            .sta = {
                .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            },
        };
        strncpy((char *)sta_cfg.sta.ssid, ssid,
                sizeof(sta_cfg.sta.ssid) - 1);
        if (password) {
            strncpy((char *)sta_cfg.sta.password, password,
                    sizeof(sta_cfg.sta.password) - 1);
        }

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    } else {
        wifi_config_t empty_sta = {0};
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &empty_sta));
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Mains-powered, but keep MIN_MODEM power save: it still wakes for every
    // DTIM beacon so the device stays responsive while trimming idle current.
    esp_err_t ps_err = esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    if (ps_err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_ps(MIN_MODEM) failed: %s", esp_err_to_name(ps_err));
    }

    s_has_sta_credentials = has_sta;

    // Spin up the reconnect + watchdog tasks once. They self-gate on
    // s_has_sta_credentials, so creating them even in AP-only mode is fine.
    if (s_reconnect_task_handle == NULL) {
        xTaskCreate(wifi_reconnect_task, "wifi_reconnect", 3072, NULL, 4,
                    &s_reconnect_task_handle);
    }
    if (s_watchdog_task_handle == NULL) {
        xTaskCreate(wifi_watchdog_task, "wifi_wdog", 3072, NULL, 3,
                    &s_watchdog_task_handle);
    }

    s_ap_active              = true;
    g_wifi_status.ap_active  = true;
    g_wifi_status.ap_clients = 0;
    led_status_set_ap_active(true);

    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(s_ap_netif, &ip_info) == ESP_OK) {
        snprintf(g_wifi_status.ap_ip, sizeof(g_wifi_status.ap_ip),
                 IPSTR, IP2STR(&ip_info.ip));
    } else {
        strncpy(g_wifi_status.ap_ip, "192.168.4.1",
                sizeof(g_wifi_status.ap_ip) - 1);
    }

    update_mdns_hostname(false);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Hotspot active for at least 5 minutes");
    ESP_LOGI(TAG, "  SSID:     %s", g_wifi_status.ap_ssid);
    ESP_LOGI(TAG, "  Password: (none - open network)");
    ESP_LOGI(TAG, "  Web UI:   http://%s/ or http://%s.local/",
             g_wifi_status.ap_ip, MDNS_HOSTNAME_AP);
    ESP_LOGI(TAG, "========================================");

    xTaskCreate(ap_management_task, "ap_mgmt", 3072, NULL, 5, &s_ap_task_handle);

    esp_wifi_get_mac(WIFI_IF_STA, g_wifi_status.mac);

    if (!has_sta) {
        ESP_LOGI(TAG, "AP-only mode - no STA credentials configured");
        led_status_set_wifi(LED_STATUS_WIFI_DISCONNECTED);
        return ESP_OK;
    }

    led_status_set_wifi(LED_STATUS_WIFI_CONNECTING);
    ESP_LOGI(TAG, "Wi-Fi APSTA mode started. Connecting to %s...", ssid);

    const TickType_t timeout_ticks = pdMS_TO_TICKS(30000);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           timeout_ticks);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID: %s", ssid);

        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            g_wifi_status.rssi    = ap_info.rssi;
            g_wifi_status.channel = ap_info.primary;
        }
        strncpy(g_wifi_status.dns, g_wifi_status.gateway,
                sizeof(g_wifi_status.dns) - 1);

        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID: %s (AP still active for fallback)",
                 ssid);
        led_status_set_wifi(LED_STATUS_WIFI_DISCONNECTED);
        return ESP_FAIL;
    } else {
        ESP_LOGW(TAG, "Wi-Fi connection timeout (still trying in background, AP active)");
        led_status_set_wifi(LED_STATUS_WIFI_RETRYING);
        return ESP_OK;
    }
}

void wifi_manager_get_status(wifi_status_t *status)
{
    if (status == NULL) return;

    if (g_wifi_status.connected) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            g_wifi_status.rssi    = ap_info.rssi;
            g_wifi_status.channel = ap_info.primary;
        }
    }

    g_wifi_status.ap_active  = s_ap_active;
    g_wifi_status.ap_clients = (uint8_t)s_ap_client_count;

    memcpy(status, &g_wifi_status, sizeof(wifi_status_t));
}

bool wifi_manager_is_connected(void)
{
    return g_wifi_status.connected;
}

bool wifi_manager_ap_is_active(void)
{
    return s_ap_active;
}

int wifi_manager_scan(wifi_scan_result_t *results, int max_results)
{
    if (results == NULL || max_results <= 0) return 0;

    wifi_scan_config_t scan_cfg = {
        .show_hidden = false,
        .scan_type   = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time   = { .active = { .min = 100, .max = 300 } },
    };

    esp_err_t err = esp_wifi_scan_start(&scan_cfg, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WiFi scan failed: %s", esp_err_to_name(err));
        return 0;
    }

    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    if (ap_count == 0) return 0;

    uint16_t fetch = (ap_count < (uint16_t)max_results) ? ap_count : (uint16_t)max_results;
    wifi_ap_record_t *records = calloc(fetch, sizeof(wifi_ap_record_t));
    if (records == NULL) return 0;

    esp_wifi_scan_get_ap_records(&fetch, records);

    int written = 0;
    for (uint16_t i = 0; i < fetch; i++) {
        bool dup = false;
        for (int j = 0; j < written; j++) {
            if (strcmp(results[j].ssid, (const char *)records[i].ssid) == 0) {
                dup = true;
                break;
            }
        }
        if (dup) continue;
        if (records[i].ssid[0] == '\0') continue;

        strncpy(results[written].ssid, (const char *)records[i].ssid, 32);
        results[written].ssid[32] = '\0';
        results[written].rssi     = records[i].rssi;
        results[written].channel  = records[i].primary;
        results[written].authmode = (uint8_t)records[i].authmode;
        written++;
        if (written >= max_results) break;
    }

    free(records);
    ESP_LOGI(TAG, "WiFi scan: %d unique networks found", written);
    return written;
}

esp_err_t wifi_manager_set_sta_config(const char *ssid, const char *password)
{
    if (ssid == NULL) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Applying new STA config: SSID=\"%s\"", ssid);

    strncpy(g_wifi_status.ssid, ssid, sizeof(g_wifi_status.ssid) - 1);
    g_wifi_status.ssid[sizeof(g_wifi_status.ssid) - 1] = '\0';

    wifi_config_t sta_cfg = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)sta_cfg.sta.ssid, ssid, sizeof(sta_cfg.sta.ssid) - 1);
    if (password) {
        strncpy((char *)sta_cfg.sta.password, password, sizeof(sta_cfg.sta.password) - 1);
    }

    if (!password || password[0] == '\0') {
        sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }

    esp_wifi_disconnect();
    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set STA config: %s", esp_err_to_name(err));
        return err;
    }

    g_wifi_status.connected = false;
    s_has_sta_credentials   = true;
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    esp_wifi_connect();
    return ESP_OK;
}

void wifi_manager_disconnect_sta(void)
{
    ESP_LOGI(TAG, "Disconnecting STA and clearing config");
    s_has_sta_credentials = false;
    esp_wifi_disconnect();

    wifi_config_t empty = {0};
    esp_wifi_set_config(WIFI_IF_STA, &empty);

    g_wifi_status.connected = false;
    g_wifi_status.ssid[0] = '\0';
    g_wifi_status.ip[0] = '\0';
    g_wifi_status.gateway[0] = '\0';
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    update_mdns_hostname(false);
}
