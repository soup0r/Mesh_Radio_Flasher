idf_component_register(
    SRCS "src/swd_core.c" "src/swd_mem.c" "src/swd_flash.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_timer
)
