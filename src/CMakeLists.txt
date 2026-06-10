idf_component_register(
    SRCS
        "main.c"
        "led_status.c"
        "wifi_manager.c"
        "wifi_config.c"
        "nvs_store.c"
        "http_server.c"
        "pump.c"
        "mqtt_bridge.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_adc
        esp_wifi
        esp_event
        esp_netif
        esp_http_server
        nvs_flash
        mqtt
        driver
        app_update
        esp_hw_support
        esp_timer
)

target_add_binary_data(${COMPONENT_LIB} "index.html" TEXT)
