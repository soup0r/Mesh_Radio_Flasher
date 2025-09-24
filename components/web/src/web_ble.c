#include "esp_http_server.h"
#include "esp_log.h"
#include "ble_proxy.h"
#include "cJSON.h"
#include <string.h>

static const char *TAG = "WEB_BLE";

// External declaration for connection handlers
extern esp_err_t register_ble_connect_handlers(httpd_handle_t server);

// Start BLE scan handler
static esp_err_t ble_scan_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "BLE scan requested from web interface");

    // Parse duration from query string (default 10 seconds)
    char query[64] = {0};
    uint32_t duration = 10;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char param[16];
        if (httpd_query_key_value(query, "duration", param, sizeof(param)) == ESP_OK) {
            duration = (uint32_t)atoi(param);
            if (duration < 1) duration = 1;
            if (duration > 30) duration = 30;
        }
    }

    // Clear previous results
    ble_proxy_clear_devices();

    // Start scan
    esp_err_t ret = ble_proxy_start_scan(duration);

    cJSON *json = cJSON_CreateObject();
    if (ret == ESP_OK) {
        cJSON_AddBoolToObject(json, "success", true);
        cJSON_AddNumberToObject(json, "duration", duration);
        cJSON_AddStringToObject(json, "message", "Scan started");
        ESP_LOGI(TAG, "BLE scan started for %lu seconds", duration);
    } else {
        cJSON_AddBoolToObject(json, "success", false);
        cJSON_AddStringToObject(json, "error", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Failed to start BLE scan: %s", esp_err_to_name(ret));
    }

    char *json_str = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));

    free(json_str);
    cJSON_Delete(json);
    return ESP_OK;
}

// Stop BLE scan handler
static esp_err_t ble_stop_scan_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Stop BLE scan requested");

    esp_err_t ret = ble_proxy_stop_scan();

    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "success", ret == ESP_OK);
    if (ret != ESP_OK) {
        cJSON_AddStringToObject(json, "error", esp_err_to_name(ret));
    }

    char *json_str = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));

    free(json_str);
    cJSON_Delete(json);
    return ESP_OK;
}

// Get BLE scan status
static esp_err_t ble_scan_status_handler(httpd_req_t *req) {
    bool is_scanning = ble_proxy_is_scanning();
    uint16_t device_count = ble_proxy_get_device_count();

    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "scanning", is_scanning);
    cJSON_AddNumberToObject(json, "device_count", device_count);

    char *json_str = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));

    free(json_str);
    cJSON_Delete(json);
    return ESP_OK;
}

// Get discovered BLE devices
static esp_err_t ble_devices_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "BLE devices list requested");

    ble_device_info_t devices[BLE_MAX_DEVICES];
    uint16_t count = ble_proxy_get_devices(devices, BLE_MAX_DEVICES);

    cJSON *json = cJSON_CreateObject();
    cJSON *devices_array = cJSON_CreateArray();

    for (int i = 0; i < count; i++) {
        cJSON *device = cJSON_CreateObject();

        // Format MAC address
        char mac_str[18];
        snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                devices[i].addr[5], devices[i].addr[4], devices[i].addr[3],
                devices[i].addr[2], devices[i].addr[1], devices[i].addr[0]);

        cJSON_AddStringToObject(device, "mac", mac_str);
        cJSON_AddNumberToObject(device, "rssi", devices[i].rssi);

        if (devices[i].has_name) {
            cJSON_AddStringToObject(device, "name", devices[i].name);

            // Check if it's a Meshtastic device
            if (strstr(devices[i].name, "Meshtastic") != NULL) {
                cJSON_AddBoolToObject(device, "is_meshtastic", true);
            }
        } else {
            cJSON_AddStringToObject(device, "name", "Unknown");
        }

        cJSON_AddNumberToObject(device, "last_seen", devices[i].last_seen);

        // Calculate signal strength category
        const char *signal_strength;
        if (devices[i].rssi > -60) signal_strength = "Excellent";
        else if (devices[i].rssi > -70) signal_strength = "Good";
        else if (devices[i].rssi > -80) signal_strength = "Fair";
        else signal_strength = "Weak";
        cJSON_AddStringToObject(device, "signal", signal_strength);

        cJSON_AddItemToArray(devices_array, device);
    }

    cJSON_AddItemToObject(json, "devices", devices_array);
    cJSON_AddNumberToObject(json, "count", count);
    cJSON_AddBoolToObject(json, "scanning", ble_proxy_is_scanning());

    char *json_str = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));

    free(json_str);
    cJSON_Delete(json);

    ESP_LOGI(TAG, "Sent %d BLE devices", count);
    return ESP_OK;
}

// Clear BLE devices
static esp_err_t ble_clear_devices_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Clear BLE devices requested");

    ble_proxy_clear_devices();

    const char *response = "{\"success\":true}";
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));

    return ESP_OK;
}

// Register all BLE handlers
esp_err_t register_ble_handlers(httpd_handle_t server) {
    httpd_uri_t ble_scan_uri = {
        .uri = "/ble/scan",
        .method = HTTP_POST,
        .handler = ble_scan_handler,
        .user_ctx = NULL
    };

    httpd_uri_t ble_stop_scan_uri = {
        .uri = "/ble/stop_scan",
        .method = HTTP_POST,
        .handler = ble_stop_scan_handler,
        .user_ctx = NULL
    };

    httpd_uri_t ble_scan_status_uri = {
        .uri = "/ble/scan_status",
        .method = HTTP_GET,
        .handler = ble_scan_status_handler,
        .user_ctx = NULL
    };

    httpd_uri_t ble_devices_uri = {
        .uri = "/ble/devices",
        .method = HTTP_GET,
        .handler = ble_devices_handler,
        .user_ctx = NULL
    };

    httpd_uri_t ble_clear_uri = {
        .uri = "/ble/clear",
        .method = HTTP_POST,
        .handler = ble_clear_devices_handler,
        .user_ctx = NULL
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ble_scan_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ble_stop_scan_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ble_scan_status_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ble_devices_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ble_clear_uri));

    // Register connection handlers
    register_ble_connect_handlers(server);

    ESP_LOGI(TAG, "BLE web handlers registered");
    return ESP_OK;
}