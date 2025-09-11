#include "recovery.h"
#include "esp_log.h"

static const char *TAG = "RECOVERY";

esp_err_t recovery_init(void) {
    ESP_LOGI(TAG, "Recovery system initialized");
    return ESP_OK;
}

recovery_trigger_t recovery_check_trigger(void) {
    return RECOVERY_TRIGGER_NONE;
}