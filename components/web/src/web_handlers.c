#include "web_server.h"
#include "esp_log.h"
#include "power_mgmt.h"
#include "cJSON.h"

static const char *TAG = "WEB_HANDLERS";

// Power control handlers
static esp_err_t power_on_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Power on request");

    esp_err_t ret = power_target_on();

    cJSON *json = cJSON_CreateObject();
    if (ret == ESP_OK) {
        cJSON_AddBoolToObject(json, "success", true);
        cJSON_AddStringToObject(json, "message", "Power turned on");
    } else {
        cJSON_AddBoolToObject(json, "success", false);
        cJSON_AddStringToObject(json, "message", "Failed to turn on power");
    }

    char *json_string = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free(json_string);
    cJSON_Delete(json);
    return ESP_OK;
}

static esp_err_t power_off_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Power off request");

    esp_err_t ret = power_target_off();

    cJSON *json = cJSON_CreateObject();
    if (ret == ESP_OK) {
        cJSON_AddBoolToObject(json, "success", true);
        cJSON_AddStringToObject(json, "message", "Power turned off");
    } else {
        cJSON_AddBoolToObject(json, "success", false);
        cJSON_AddStringToObject(json, "message", "Failed to turn off power");
    }

    char *json_string = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free(json_string);
    cJSON_Delete(json);
    return ESP_OK;
}

static esp_err_t power_reboot_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Power reboot request");

    esp_err_t ret = power_target_reset();

    cJSON *json = cJSON_CreateObject();
    if (ret == ESP_OK) {
        cJSON_AddBoolToObject(json, "success", true);
        cJSON_AddStringToObject(json, "message", "Reboot cycle started");
    } else {
        cJSON_AddBoolToObject(json, "success", false);
        cJSON_AddStringToObject(json, "message", "Failed to reboot");
    }

    char *json_string = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free(json_string);
    cJSON_Delete(json);
    return ESP_OK;
}

static esp_err_t power_cycle_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Power cycle request");

    // Parse query parameter for time
    char query[64] = {0};
    uint32_t off_time = 15000; // Default 15 seconds

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char param[16];
        if (httpd_query_key_value(query, "time", param, sizeof(param)) == ESP_OK) {
            off_time = (uint32_t)atoi(param);
            // Clamp to reasonable limits
            if (off_time < 100) off_time = 100;
            if (off_time > 60000) off_time = 60000;
        }
    }

    ESP_LOGI(TAG, "Power cycling for %lu ms", off_time);
    esp_err_t ret = power_target_cycle(off_time);

    cJSON *json = cJSON_CreateObject();
    if (ret == ESP_OK) {
        cJSON_AddBoolToObject(json, "success", true);
        cJSON_AddStringToObject(json, "message", "Power cycle complete");
        cJSON_AddNumberToObject(json, "off_time_ms", off_time);
    } else {
        cJSON_AddBoolToObject(json, "success", false);
        cJSON_AddStringToObject(json, "message", "Failed to cycle power");
    }

    char *json_string = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free(json_string);
    cJSON_Delete(json);
    return ESP_OK;
}

esp_err_t register_power_handlers(httpd_handle_t server) {
    httpd_uri_t power_on_uri = {
        .uri = "/power_on",
        .method = HTTP_POST,
        .handler = power_on_handler,
        .user_ctx = NULL
    };

    httpd_uri_t power_off_uri = {
        .uri = "/power_off",
        .method = HTTP_POST,
        .handler = power_off_handler,
        .user_ctx = NULL
    };

    httpd_uri_t power_reboot_uri = {
        .uri = "/power_reboot",
        .method = HTTP_POST,
        .handler = power_reboot_handler,
        .user_ctx = NULL
    };

    httpd_uri_t power_cycle_uri = {
        .uri = "/power_cycle",
        .method = HTTP_POST,
        .handler = power_cycle_handler,
        .user_ctx = NULL
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &power_on_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &power_off_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &power_reboot_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &power_cycle_uri));

    ESP_LOGI(TAG, "Power control handlers registered");
    return ESP_OK;
}

esp_err_t register_swd_handlers(httpd_handle_t server) {
    ESP_LOGI(TAG, "SWD handlers registered");
    return ESP_OK;
}

esp_err_t register_flash_handlers(httpd_handle_t server) {
    ESP_LOGI(TAG, "Flash handlers registered");
    return ESP_OK;
}