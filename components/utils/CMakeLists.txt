idf_component_register(
    SRCS "src/recovery.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash esp_system
)
