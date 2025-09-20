idf_component_register(
    SRCS "src/power_mgmt.c"
    INCLUDE_DIRS "include"
    REQUIRES driver nvs_flash esp_wifi esp_timer swd esp_adc
)