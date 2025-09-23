idf_component_register(
    SRCS "src/ble_proxy.c" "src/ble_connection.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash bt freertos esp_timer
)