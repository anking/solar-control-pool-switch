#include "mqtt_bridge.h"
#include "nvs_store.h"
#include "config.h"
#include "pump.h"
#include "wifi_manager.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_app_desc.h"
#include "mqtt_client.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "mqtt_bridge";

static esp_mqtt_client_handle_t s_client = NULL;
// Written by the MQTT event task, read from HTTP handlers / the broadcaster.
static volatile bool s_connected = false;
static char     s_mac_str[18] = {0};
static char     s_host[64] = {0};
static int      s_port = 1883;
static char     s_user[32] = {0};
static char     s_pass[64] = {0};
static char     s_error[64] = {0};
static uint32_t s_publish_count = 0;
static uint32_t s_publish_fail_count = 0;

static void get_mac_string(void)
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_mac_str, sizeof(s_mac_str), "%02x-%02x-%02x-%02x-%02x-%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void load_config(void)
{
    nvs_store_get_str(NVS_NS_MQTT, "host", s_host, sizeof(s_host));
    uint16_t port;
    if (nvs_store_get_u16(NVS_NS_MQTT, "port", &port) == ESP_OK) s_port = port;
    nvs_store_get_str(NVS_NS_MQTT, "user", s_user, sizeof(s_user));
    nvs_store_get_str(NVS_NS_MQTT, "pass", s_pass, sizeof(s_pass));
}

static void save_config(void)
{
    nvs_store_set_str(NVS_NS_MQTT, "host", s_host);
    nvs_store_set_u16(NVS_NS_MQTT, "port", (uint16_t)s_port);
    nvs_store_set_str(NVS_NS_MQTT, "user", s_user);
    nvs_store_set_str(NVS_NS_MQTT, "pass", s_pass);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch (event_id) {
    case MQTT_EVENT_CONNECTED: {
        s_connected = true;
        s_error[0] = '\0';
        ESP_LOGI(TAG, "Connected to broker");

        char topic[64];
        snprintf(topic, sizeof(topic), "pumps/%s/status", s_mac_str);
        esp_mqtt_client_publish(s_client, topic, "{\"online\":true}", 0, 1, 1);

        // Subscribe to the retained command topic so a pump command is
        // delivered the moment we connect.
        snprintf(topic, sizeof(topic), "pumps/%s/cmd", s_mac_str);
        esp_mqtt_client_subscribe(s_client, topic, 1);

        // Publish device info + the current state so the cloud picks them up.
        mqtt_bridge_publish_info();
        pump_reading_t r;
        pump_get(&r);
        mqtt_bridge_publish_state(&r);
        break;
    }
    case MQTT_EVENT_DATA: {
        // Payloads larger than the MQTT RX buffer arrive fragmented across
        // multiple events (only the first carries the topic). Real commands are
        // tiny, so anything fragmented is garbage — ignore it rather than parse
        // a partial JSON.
        if (event->total_data_len != event->data_len) {
            ESP_LOGW(TAG, "Ignoring fragmented MQTT payload (%d of %d bytes)",
                     event->data_len, event->total_data_len);
            break;
        }
        // Only the command topic carries inbound messages.
        if (event->topic_len >= 4 &&
            memcmp(event->topic + event->topic_len - 4, "/cmd", 4) == 0) {
            // A message delivered with the retain flag set is a *stored*
            // command the broker replays to every new subscriber on connect.
            // The pump must never auto-resume from one (that's how a stale
            // retained "on" from before the non-retained switch would turn the
            // pump on at every boot). Ignore it and delete it from the broker
            // so it can't fire again. Only live (non-retained) commands switch
            // the pump.
            if (event->retain) {
                ESP_LOGW(TAG, "Ignoring + clearing retained command (pump never auto-resumes)");
                mqtt_bridge_clear_retained_cmd();
            } else {
                pump_handle_command(event->data, event->data_len);
            }
        }
        break;
    }
    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        ESP_LOGW(TAG, "Disconnected from broker");
        break;

    case MQTT_EVENT_ERROR: {
        esp_mqtt_error_codes_t *err = event->error_handle;
        if (err->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            if (err->connect_return_code == MQTT_CONNECTION_REFUSE_NOT_AUTHORIZED ||
                err->connect_return_code == MQTT_CONNECTION_REFUSE_BAD_USERNAME) {
                strncpy(s_error, "Authentication failed", sizeof(s_error) - 1);
            } else if (err->connect_return_code == MQTT_CONNECTION_REFUSE_SERVER_UNAVAILABLE) {
                strncpy(s_error, "Server unavailable", sizeof(s_error) - 1);
            } else if (err->esp_transport_sock_errno) {
                snprintf(s_error, sizeof(s_error), "Connection error: %d", err->esp_transport_sock_errno);
            } else {
                strncpy(s_error, "Connection failed", sizeof(s_error) - 1);
            }
            ESP_LOGW(TAG, "MQTT error: %s", s_error);
        }
        break;
    }
    default:
        break;
    }
}

static void start_client(void)
{
    if (s_host[0] == '\0') return;
    s_error[0] = '\0';
    if (s_client) {
        esp_mqtt_client_stop(s_client);
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
    }

    char uri[128];
    snprintf(uri, sizeof(uri), "mqtt://%s:%d", s_host, s_port);

    char client_id[32];
    snprintf(client_id, sizeof(client_id), "pool-%s", s_mac_str);

    char lwt_topic[64];
    snprintf(lwt_topic, sizeof(lwt_topic), "pumps/%s/status", s_mac_str);

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = uri,
        .credentials.client_id = client_id,
        .credentials.username = s_user[0] ? s_user : NULL,
        .credentials.authentication.password = s_pass[0] ? s_pass : NULL,
        .session.last_will.topic = lwt_topic,
        .session.last_will.msg = "{\"online\":false}",
        .session.last_will.qos = 1,
        .session.last_will.retain = true,
    };

    s_client = esp_mqtt_client_init(&cfg);
    if (s_client) {
        esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
        esp_mqtt_client_start(s_client);
        ESP_LOGI(TAG, "Connecting to %s:%d", s_host, s_port);
    }
}

esp_err_t mqtt_bridge_init(void)
{
    get_mac_string();
    load_config();

    if (s_host[0] != '\0') {
        start_client();
    } else {
        ESP_LOGI(TAG, "MQTT not configured");
    }
    return ESP_OK;
}

bool mqtt_bridge_is_connected(void)
{
    return s_connected;
}

bool mqtt_bridge_is_configured(void)
{
    return s_host[0] != '\0';
}

void mqtt_bridge_publish_info(void)
{
    if (!s_connected || !s_client) return;

    char topic[64];
    snprintf(topic, sizeof(topic), "pumps/%s/info", s_mac_str);

    pump_reading_t r;
    pump_get(&r);

    const char *fw = esp_app_get_description()->version;

    // Deep-link targets for the on-device UI. ui_url uses the current IP because
    // mDNS (.local) does not resolve reliably off-LAN or on some clients; the IP
    // is the dependable clickable link. ui_host carries the stable mDNS name
    // (e.g. "poolpump-8dabd4.local") for display.
    wifi_status_t wifi;
    wifi_manager_get_status(&wifi);

    // Pick a reachable IP for the deep link: STA when connected, the AP
    // fallback address otherwise. Empty string (no IP at all) rather than a
    // malformed "http:///" — the cloud treats an empty ui_url as absent.
    char ui_url[40] = "";
    if (wifi.connected && wifi.ip[0]) {
        snprintf(ui_url, sizeof(ui_url), "http://%s/", wifi.ip);
    } else if (wifi.ap_active && wifi.ap_ip[0]) {
        snprintf(ui_url, sizeof(ui_url), "http://%s/", wifi.ap_ip);
    }

    char buf[384];
    int len = snprintf(buf, sizeof(buf),
        "{\"model\":\"esp32-c3-pool-switch\",\"firmware\":\"%s\","
        "\"output_gpio\":%d,\"feedback_gpio\":%d,\"pressure_gpio\":%d,\"threshold_mv\":%d,"
        "\"ui_url\":\"%s\",\"ui_host\":\"%s.local\"}",
        fw, PUMP_OUTPUT_GPIO, PUMP_FEEDBACK_GPIO, PRESSURE_GPIO, pump_get_threshold_mv(),
        ui_url, wifi.hostname);

    int msg_id = esp_mqtt_client_publish(s_client, topic, buf, len, 1, 1);
    if (msg_id >= 0) {
        ESP_LOGI(TAG, "Device info published to %s", topic);
    } else {
        ESP_LOGW(TAG, "Device info publish failed");
    }
}

void mqtt_bridge_publish_state(const pump_reading_t *r)
{
    if (!s_connected || !s_client || !r) return;

    char topic[64];
    snprintf(topic, sizeof(topic), "pumps/%s/state", s_mac_str);

    char buf[320];
    int len = snprintf(buf, sizeof(buf),
        "{\"commanded_on\":%s,\"feedback_on\":%s,\"mismatch\":%s,"
        "\"feedback_v\":%.3f,\"feedback_mv\":%d,\"saturated\":%s,"
        "\"threshold_mv\":%d,\"pressure_psi\":%.1f,\"pressure_v\":%.3f,"
        "\"pressure_valid\":%s,\"failsafe_off\":%s,"
        "\"on_seconds\":%lu}",
        r->commanded_on ? "true" : "false",
        r->feedback_on  ? "true" : "false",
        r->mismatch     ? "true" : "false",
        r->feedback_v, r->feedback_mv,
        r->saturated ? "true" : "false",
        r->threshold_mv,
        r->pressure_psi, r->pressure_v,
        r->pressure_valid ? "true" : "false",
        r->failsafe_off ? "true" : "false",
        (unsigned long)r->on_seconds);

    // Retained QoS 1: the latest state is always available to a new subscriber.
    int msg_id = esp_mqtt_client_publish(s_client, topic, buf, len, 1, 1);
    if (msg_id >= 0) {
        s_publish_count++;
    } else {
        s_publish_fail_count++;
        ESP_LOGW(TAG, "State publish failed (fails=%lu)", (unsigned long)s_publish_fail_count);
    }
}

void mqtt_bridge_clear_retained_cmd(void)
{
    if (!s_connected || !s_client) return;
    char topic[64];
    snprintf(topic, sizeof(topic), "pumps/%s/cmd", s_mac_str);
    // Zero-length retained message deletes the retained value on the broker.
    esp_mqtt_client_publish(s_client, topic, "", 0, 1, 1);
}

esp_err_t mqtt_bridge_configure(const char *host, int port, const char *user, const char *pass)
{
    if (!host) return ESP_ERR_INVALID_ARG;

    if (s_client) {
        if (s_connected) {
            char topic[64];
            snprintf(topic, sizeof(topic), "pumps/%s/status", s_mac_str);
            esp_mqtt_client_publish(s_client, topic, "{\"online\":false}", 0, 1, 1);
        }
        esp_mqtt_client_stop(s_client);
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
        s_connected = false;
    }

    strncpy(s_host, host, sizeof(s_host) - 1);
    s_port = port;
    strncpy(s_user, user ? user : "", sizeof(s_user) - 1);
    strncpy(s_pass, pass ? pass : "", sizeof(s_pass) - 1);
    save_config();

    start_client();
    return ESP_OK;
}

esp_err_t mqtt_bridge_disconnect(void)
{
    if (s_client) {
        esp_mqtt_client_stop(s_client);
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
        s_connected = false;
    }
    return ESP_OK;
}

esp_err_t mqtt_bridge_clear_config(void)
{
    mqtt_bridge_disconnect();
    memset(s_host, 0, sizeof(s_host));
    memset(s_user, 0, sizeof(s_user));
    memset(s_pass, 0, sizeof(s_pass));
    s_port = 1883;
    save_config();
    return ESP_OK;
}

void mqtt_bridge_get_status(mqtt_status_t *out)
{
    if (!out) return;
    out->configured = (s_host[0] != '\0');
    out->connected = s_connected;
    strncpy(out->host, s_host, sizeof(out->host) - 1);
    out->port = s_port;
    strncpy(out->mac, s_mac_str, sizeof(out->mac) - 1);
    strncpy(out->error, s_error, sizeof(out->error) - 1);
    out->publish_count = s_publish_count;
    out->publish_fail_count = s_publish_fail_count;
}
