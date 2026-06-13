#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "config.h"
#include "led_status.h"
#include "wifi_manager.h"
#include "wifi_config.h"
#include "http_server.h"
#include "pump.h"
#include "mqtt_bridge.h"

static const char *TAG = "main";

// Daily reboot policy.
//   - Don't reboot in the first 5 minutes of uptime (rules out reboot loops).
//   - If SNTP has synced (year >= 2025), reboot at the next UTC midnight.
//   - Otherwise, fall back to a 24h-uptime trigger.
//   - Hard cap at 25h uptime regardless — last-resort if midnight calc misfires.
// The pump always boots OFF after the reboot — its state is never persisted, so
// a reboot can't silently re-energise the pump (see pump.c).
#define DAILY_REBOOT_MIN_UPTIME_MS   (5LL * 60 * 1000)
#define DAILY_REBOOT_CHECK_PERIOD_MS (60LL * 1000)
#define DAILY_REBOOT_UPTIME_CAP_MS   (25LL * 60 * 60 * 1000)
#define DAILY_REBOOT_FALLBACK_MS     (24LL * 60 * 60 * 1000)
#define DAILY_REBOOT_MIDNIGHT_WINDOW_S 60   // fire within this window past 00:00:00 UTC

static void daily_reboot_task(void *arg)
{
    const int64_t boot_us = esp_timer_get_time();

    // Don't even consider rebooting in the first few minutes.
    vTaskDelay(pdMS_TO_TICKS(DAILY_REBOOT_MIN_UPTIME_MS));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(DAILY_REBOOT_CHECK_PERIOD_MS));

        int64_t uptime_ms = (esp_timer_get_time() - boot_us) / 1000;

        // Hard cap. Catches the case where time was synced briefly then lost.
        if (uptime_ms > DAILY_REBOOT_UPTIME_CAP_MS) {
            ESP_LOGW(TAG, "Daily reboot: 25h uptime cap reached");
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_restart();
        }

        time_t now;
        time(&now);
        struct tm tm_utc;
        gmtime_r(&now, &tm_utc);
        bool time_synced = (tm_utc.tm_year + 1900) >= 2025;

        if (time_synced) {
            int sec_into_day = tm_utc.tm_hour * 3600
                             + tm_utc.tm_min  * 60
                             + tm_utc.tm_sec;
            if (sec_into_day < DAILY_REBOOT_MIDNIGHT_WINDOW_S) {
                ESP_LOGW(TAG, "Daily reboot: hit midnight UTC (uptime %lld ms)",
                         (long long)uptime_ms);
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_restart();
            }
        } else {
            if (uptime_ms > DAILY_REBOOT_FALLBACK_MS) {
                ESP_LOGW(TAG, "Daily reboot: 24h uptime fallback (no time sync)");
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_restart();
            }
        }
    }
}

// Status broadcaster.
//   - WebSocket: pushes the live pump status every second (local; only
//     transmits if a dashboard client is actually connected).
//   - MQTT: report-by-exception. We publish the moment any meaningful field
//     changes (commanded state, measured feedback, or the fault flag), plus a
//     periodic heartbeat so the backend knows we're still alive.
static void status_broadcaster_task(void *arg)
{
    static char buf[640];
    bool    have_sent      = false;
    bool    last_commanded = false;
    bool    last_feedback  = false;
    bool    last_mismatch  = false;
    bool    last_lockout   = false;
    bool    last_warning   = false;
    float   last_pressure  = 0.0f;
    int64_t last_pub_ms    = esp_timer_get_time() / 1000;
    int64_t mqtt_down_since_ms = 0;   // 0 = broker currently reachable

    // Pressure-failsafe per-run state (see the failsafe block below).
    bool    fs_was_on        = false;
    bool    fs_reached_min   = false;
    int64_t fs_high_since_ms = 0;
    int64_t fs_warn_since_ms = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        pump_reading_t r;
        pump_get(&r);

        // ---- Safety failsafes (only meaningful while the pump is running) ----
        int64_t fs_now_ms = esp_timer_get_time() / 1000;

        // 1. Dead-man's switch: broker unreachable for too long → turn OFF, so a
        //    lost internet/broker connection can't leave the pump running
        //    indefinitely. Gated on a broker being configured (local-only use is
        //    unaffected). When connectivity returns the cloud schedule re-asserts.
#if PUMP_FAILSAFE_MQTT_TIMEOUT_S > 0
        if (r.valid && r.commanded_on && mqtt_bridge_is_configured()) {
            if (mqtt_bridge_is_connected()) {
                mqtt_down_since_ms = 0;
            } else {
                if (mqtt_down_since_ms == 0) mqtt_down_since_ms = fs_now_ms;
                else if (fs_now_ms - mqtt_down_since_ms >= (int64_t)PUMP_FAILSAFE_MQTT_TIMEOUT_S * 1000) {
                    ESP_LOGW(TAG, "FAILSAFE: broker unreachable for %ds while running — pump OFF",
                             PUMP_FAILSAFE_MQTT_TIMEOUT_S);
                    pump_set(false);
                    pump_note_failsafe_off();   // so the cloud can tell this from a normal off
                    mqtt_down_since_ms = 0;
                    pump_get(&r);   // refresh so the publish below reflects OFF
                }
            }
        } else {
            mqtt_down_since_ms = 0;
        }
#endif

        // 2. Hard runtime cap: OFF after N seconds of continuous running.
#if PUMP_MAX_RUNTIME_S > 0
        if (r.valid && r.commanded_on) {
            int64_t on_ms = (esp_timer_get_time() - r.last_change_us) / 1000;
            if (on_ms >= (int64_t)PUMP_MAX_RUNTIME_S * 1000) {
                ESP_LOGW(TAG, "FAILSAFE: max runtime %ds reached — pump OFF", PUMP_MAX_RUNTIME_S);
                pump_set(false);
                pump_note_failsafe_off();
                pump_get(&r);
            }
        }
#endif

        // 3. Pressure failsafes (the device is the authority). Only while
        //    running with a trustworthy reading and not already locked out. A
        //    LOW or HIGH trip switches the pump off and latches a persisted
        //    lockout — it can't restart (manual / MQTT / schedule) until reset.
        //    A WARNING just sets a flag the cloud emails on. The cloud mirrors
        //    these reported states; it does no independent detection.
        if (r.valid && r.commanded_on && r.pressure_valid && !r.safety_lockout) {
            if (!fs_was_on) {   // pump just started — arm a fresh run
                fs_reached_min = false;
                fs_high_since_ms = 0;
                fs_warn_since_ms = 0;
            }
            int64_t on_ms = (esp_timer_get_time() - r.last_change_us) / 1000;

            // LOW: must clear min psi within the grace window of starting.
            if (!fs_reached_min) {
                if (r.pressure_psi >= r.min_psi) {
                    fs_reached_min = true;
                } else if (on_ms >= (int64_t)PUMP_LOW_PRESSURE_GRACE_S * 1000) {
                    ESP_LOGW(TAG, "FAILSAFE: low pressure %.1f < %.1f psi %ds after start — LOCKOUT",
                             r.pressure_psi, r.min_psi, PUMP_LOW_PRESSURE_GRACE_S);
                    pump_set(false);
                    pump_trip_lockout(PUMP_SAFETY_LOW_PRESSURE);
                    pump_get(&r);
                }
            }

            // HIGH: at/above critical psi held past the sustain window.
            if (!r.safety_lockout && r.pressure_psi >= r.critical_psi) {
                if (fs_high_since_ms == 0) fs_high_since_ms = fs_now_ms;
                else if (fs_now_ms - fs_high_since_ms >= (int64_t)PUMP_HIGH_PRESSURE_SUSTAIN_S * 1000) {
                    ESP_LOGW(TAG, "FAILSAFE: high pressure %.1f >= %.1f psi sustained %ds — LOCKOUT",
                             r.pressure_psi, r.critical_psi, PUMP_HIGH_PRESSURE_SUSTAIN_S);
                    pump_set(false);
                    pump_trip_lockout(PUMP_SAFETY_HIGH_PRESSURE);
                    pump_get(&r);
                }
            } else {
                fs_high_since_ms = 0;
            }

            // WARNING: above max psi held past the warn window → flag only.
            if (!r.safety_lockout && r.pressure_psi > r.max_psi) {
                if (fs_warn_since_ms == 0) fs_warn_since_ms = fs_now_ms;
                else if (fs_now_ms - fs_warn_since_ms >= (int64_t)PUMP_PRESSURE_WARN_SUSTAIN_S * 1000) {
                    pump_set_pressure_warning(true);
                }
            } else {
                fs_warn_since_ms = 0;
                if (r.pressure_warning) pump_set_pressure_warning(false);
            }

            fs_was_on = true;
        } else {
            // Off (or already locked out) — clear the per-run timers, and drop a
            // stale warning flag once the pump is no longer running.
            fs_was_on = false;
            fs_reached_min = false;
            fs_high_since_ms = 0;
            fs_warn_since_ms = 0;
            if (!r.commanded_on && r.pressure_warning) pump_set_pressure_warning(false);
        }

        int len = snprintf(buf, sizeof(buf),
            "{\"valid\":%s,\"commanded_on\":%s,\"feedback_on\":%s,\"mismatch\":%s,"
            "\"feedback_v\":%.3f,\"feedback_mv\":%d,\"raw_mv\":%d,\"peak_mv\":%d,"
            "\"saturated\":%s,\"threshold_mv\":%d,"
            "\"pressure_psi\":%.1f,\"pressure_v\":%.3f,\"pressure_valid\":%s,\"pressure_gpio\":%d,"
            "\"failsafe_off\":%s,"
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
            r.output_gpio, r.feedback_gpio,
            (unsigned long)r.on_seconds,
            (unsigned long)r.sample_count);

        if (len > 0 && len < (int)sizeof(buf)) {
            http_server_ws_broadcast_status(buf, (size_t)len);
        }

        if (!r.valid) continue;

        int64_t now_ms    = esp_timer_get_time() / 1000;
        bool    first     = !have_sent;
        bool    changed   = have_sent && (r.commanded_on   != last_commanded ||
                                          r.feedback_on    != last_feedback  ||
                                          r.mismatch       != last_mismatch  ||
                                          r.safety_lockout != last_lockout   ||
                                          r.pressure_warning != last_warning);
        bool    pressure_moved = have_sent &&
            fabsf(r.pressure_psi - last_pressure) >= REPORT_PRESSURE_DELTA_PSI;
        bool    heartbeat = (now_ms - last_pub_ms) >= (REPORT_HEARTBEAT_SECONDS * 1000);

        if (first || changed || pressure_moved || heartbeat) {
            mqtt_bridge_publish_state(&r);
            have_sent      = true;
            last_commanded = r.commanded_on;
            last_feedback  = r.feedback_on;
            last_mismatch  = r.mismatch;
            last_lockout   = r.safety_lockout;
            last_warning   = r.pressure_warning;
            last_pressure  = r.pressure_psi;
            last_pub_ms    = now_ms;
        }
    }
}

void app_main(void)
{
    esp_reset_reason_t reset_reason = esp_reset_reason();
    if (reset_reason == ESP_RST_POWERON || reset_reason == ESP_RST_BROWNOUT) {
        vTaskDelay(pdMS_TO_TICKS(500));
    } else {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Pool Switch ESP32-C3 v%s (cores: %d, rev: %d)",
             FIRMWARE_VERSION, chip_info.cores, chip_info.revision);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
    }

    // LED first so we can show WiFi state during init.
    ret = led_status_init();
    if (ret != ESP_OK) ESP_LOGE(TAG, "LED init failed: %s", esp_err_to_name(ret));

    // Pump output + feedback sampler. Always boots OFF (state is not persisted).
    ret = pump_init();
    if (ret != ESP_OK) ESP_LOGE(TAG, "Pump init failed: %s", esp_err_to_name(ret));

    // WiFi (STA + AP fallback hotspot).
    ret = wifi_manager_init();
    if (ret != ESP_OK) ESP_LOGE(TAG, "Wi-Fi manager init failed: %s", esp_err_to_name(ret));

    char saved_ssid[33] = {0};
    char saved_pass[65] = {0};
    if (wifi_config_get(saved_ssid, sizeof(saved_ssid), saved_pass, sizeof(saved_pass))) {
        ESP_LOGI(TAG, "Connecting to saved Wi-Fi: %s", saved_ssid);
        wifi_manager_start(saved_ssid, saved_pass);
    } else {
        ESP_LOGW(TAG, "No saved Wi-Fi credentials - AP-only mode");
        wifi_manager_start("", "");
    }

    // MQTT (delivers retained pump commands and publishes state).
    mqtt_bridge_init();

    // HTTP server (dashboard + APIs + /ws).
    ret = http_server_start();
    if (ret != ESP_OK) ESP_LOGE(TAG, "HTTP server failed: %s", esp_err_to_name(ret));

    // 1 Hz WebSocket push + report-by-exception MQTT state publishing.
    xTaskCreate(status_broadcaster_task, "ws_status", 4096, NULL, 5, NULL);

    // Daily reboot — runs in the background, gates itself on uptime/time-sync.
    xTaskCreate(daily_reboot_task, "daily_reboot", 3072, NULL, 2, NULL);

    wifi_status_t wifi_status;
    wifi_manager_get_status(&wifi_status);
    if (wifi_status.connected) {
        ESP_LOGI(TAG, "======================================");
        ESP_LOGI(TAG, "  Dashboard: http://%s/", wifi_status.ip);
        if (wifi_status.hostname[0]) {
            ESP_LOGI(TAG, "  mDNS:      http://%s.local/", wifi_status.hostname);
        }
        ESP_LOGI(TAG, "======================================");
    } else if (wifi_status.ap_active) {
        ESP_LOGI(TAG, "======================================");
        ESP_LOGI(TAG, "  Join hotspot: %s", wifi_status.ap_ssid);
        ESP_LOGI(TAG, "  Dashboard:    http://%s/", wifi_status.ap_ip);
        ESP_LOGI(TAG, "======================================");
    }

    // Idle loop — periodic heartbeat log.
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000));
        wifi_status_t s;
        wifi_manager_get_status(&s);
        pump_reading_t r;
        pump_get(&r);
        ESP_LOGI(TAG, "Heartbeat: wifi=%s ip=%s pump=%s feedback=%s%s fb=%.2fV heap=%lu",
                 s.connected ? "ok" : "down", s.ip,
                 r.commanded_on ? "ON" : "OFF",
                 r.feedback_on ? "ON" : "OFF",
                 r.mismatch ? " (MISMATCH!)" : "",
                 r.feedback_v,
                 (unsigned long)esp_get_free_heap_size());
    }
}
