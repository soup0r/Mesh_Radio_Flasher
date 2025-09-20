// web_bt_handlers.c - Web API handlers for Bluetooth Proxy
#include "esp_http_server.h"
#include "esp_log.h"
#include "bt_proxy.h"
#include "cJSON.h"

static const char *TAG = "WEB_BT";

// GET /api/bt/status - Get Bluetooth proxy status
static esp_err_t bt_status_handler(httpd_req_t *req) {
    bt_proxy_stats_t stats;
    bt_proxy_get_stats(&stats);
    
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "connected", stats.ble_connected);
    cJSON_AddNumberToObject(root, "tcp_clients", stats.tcp_clients);
    cJSON_AddNumberToObject(root, "bytes_transferred", stats.bytes_proxied);
    cJSON_AddNumberToObject(root, "reconnect_attempts", stats.reconnect_attempts);
    
    if (stats.ble_connected) {
        char addr_str[18];
        snprintf(addr_str, sizeof(addr_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                stats.device_addr[0], stats.device_addr[1], stats.device_addr[2],
                stats.device_addr[3], stats.device_addr[4], stats.device_addr[5]);
        cJSON_AddStringToObject(root, "device_address", addr_str);
    }
    
    char *json_str = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));
    
    free(json_str);
    cJSON_Delete(root);
    return ESP_OK;
}

// POST /api/bt/scan - Start scanning for mesh radios
static esp_err_t bt_scan_handler(httpd_req_t *req) {
    char content[100];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request");
        return ESP_OK;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_OK;
    }
    
    const char *device_name = "MESH";  // Default
    cJSON *name_item = cJSON_GetObjectItem(json, "device_name");
    if (name_item && cJSON_IsString(name_item)) {
        device_name = name_item->valuestring;
    }
    
    ESP_LOGI(TAG, "Starting BLE scan for: %s", device_name);
    esp_err_t err = bt_proxy_scan_and_connect(device_name);
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", err == ESP_OK);
    cJSON_AddStringToObject(response, "scanning_for", device_name);
    
    if (err != ESP_OK) {
        cJSON_AddStringToObject(response, "error", esp_err_to_name(err));
    }
    
    char *resp_str = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp_str, strlen(resp_str));
    
    free(resp_str);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ESP_OK;
}

// POST /api/bt/disconnect - Disconnect from BLE device
static esp_err_t bt_disconnect_handler(httpd_req_t *req) {
    esp_err_t err = bt_proxy_disconnect();
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", err == ESP_OK);
    
    if (err != ESP_OK) {
        cJSON_AddStringToObject(response, "error", esp_err_to_name(err));
    }
    
    char *resp_str = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp_str, strlen(resp_str));
    
    free(resp_str);
    cJSON_Delete(response);
    return ESP_OK;
}

// POST /api/bt/send - Send data to mesh radio via BLE
static esp_err_t bt_send_handler(httpd_req_t *req) {
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request");
        return ESP_OK;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_OK;
    }
    
    cJSON *data_item = cJSON_GetObjectItem(json, "data");
    if (!data_item || !cJSON_IsString(data_item)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing data field");
        return ESP_OK;
    }
    
    esp_err_t err = bt_proxy_send_command(data_item->valuestring);
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", err == ESP_OK);
    cJSON_AddNumberToObject(response, "bytes_sent", 
                          err == ESP_OK ? strlen(data_item->valuestring) : 0);
    
    if (err != ESP_OK) {
        cJSON_AddStringToObject(response, "error", esp_err_to_name(err));
    }
    
    char *resp_str = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp_str, strlen(resp_str));
    
    free(resp_str);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ESP_OK;
}

// Register Bluetooth proxy handlers
esp_err_t register_bt_proxy_handlers(httpd_handle_t server) {
    httpd_uri_t bt_status = {
        .uri = "/api/bt/status",
        .method = HTTP_GET,
        .handler = bt_status_handler,
        .user_ctx = NULL
    };
    
    httpd_uri_t bt_scan = {
        .uri = "/api/bt/scan",
        .method = HTTP_POST,
        .handler = bt_scan_handler,
        .user_ctx = NULL
    };
    
    httpd_uri_t bt_disconnect = {
        .uri = "/api/bt/disconnect",
        .method = HTTP_POST,
        .handler = bt_disconnect_handler,
        .user_ctx = NULL
    };
    
    httpd_uri_t bt_send = {
        .uri = "/api/bt/send",
        .method = HTTP_POST,
        .handler = bt_send_handler,
        .user_ctx = NULL
    };
    
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &bt_status));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &bt_scan));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &bt_disconnect));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &bt_send));
    
    ESP_LOGI(TAG, "Bluetooth proxy handlers registered");
    return ESP_OK;
}