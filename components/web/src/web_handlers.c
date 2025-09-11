#include "web_server.h"
#include "esp_log.h"

static const char *TAG = "WEB_HANDLERS";

esp_err_t register_swd_handlers(httpd_handle_t server) {
    ESP_LOGI(TAG, "SWD handlers registered");
    return ESP_OK;
}

esp_err_t register_flash_handlers(httpd_handle_t server) {
    ESP_LOGI(TAG, "Flash handlers registered");
    return ESP_OK;
}