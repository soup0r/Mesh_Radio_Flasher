idf_component_register(
    SRCS "src/web_server.c" "src/web_handlers.c" "src/web_upload.c" "src/web_ble.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server swd safety hex power json ble_proxy
)