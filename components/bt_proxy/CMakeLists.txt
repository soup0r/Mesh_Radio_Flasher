idf_component_register(
    SRCS "src/bt_proxy.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES 
        bt
        esp_common
        driver
        lwip
        freertos
        nvs_flash
    REQUIRES 
        bt
)