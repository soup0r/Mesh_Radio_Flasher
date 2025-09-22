idf_component_register(
    SRCS "src/ble_proxy.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash bt freertos esp_timer
)