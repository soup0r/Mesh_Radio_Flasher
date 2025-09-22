idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        swd
        power
        safety
        web
        hex
        utils
        ble_proxy
        nvs_flash
        esp_wifi
        driver
        freertos
        esp_timer
)