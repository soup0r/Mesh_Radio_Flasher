#include "esp_http_server.h"
#include "esp_log.h"
#include "ble_proxy.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

static const char *TAG = "WEB_BLE_CONN";

// Connect to device handler
static esp_err_t ble_connect_handler(httpd_req_t *req) {
    char content[128];
    char addr_str[18] = {0};

    // Read POST data
    int ret = httpd_req_recv(req, content, MIN(req->content_len, sizeof(content) - 1));
    if (ret <= 0) {
        ESP_LOGE(TAG, "Failed to receive POST data");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive data");
        return ESP_FAIL;
    }
    content[ret] = '\0';

    ESP_LOGI(TAG, "Received POST data: %s", content);

    // Parse the address manually from "addr=XX:XX:XX:XX:XX:XX" format
    char *addr_start = strstr(content, "addr=");
    if (!addr_start) {
        ESP_LOGE(TAG, "No addr parameter found");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing address parameter");
        return ESP_FAIL;
    }

    addr_start += 5; // Skip "addr="

    // URL decode the address
    char *src = addr_start;
    char *dst = addr_str;
    int dst_idx = 0;

    while (*src && dst_idx < 17) {
        if (*src == '%' && src[1] && src[2]) {
            // URL-encoded character
            char hex[3] = {src[1], src[2], '\0'};
            *dst++ = (char)strtol(hex, NULL, 16);
            src += 3;
            dst_idx++;
        } else if (*src == '&' || *src == ' ') {
            // End of parameter
            break;
        } else {
            // Regular character
            *dst++ = *src++;
            dst_idx++;
        }
    }
    *dst = '\0';

    ESP_LOGI(TAG, "Decoded address: %s (length: %d)", addr_str, strlen(addr_str));

    if (strlen(addr_str) != 17) {
        ESP_LOGE(TAG, "Invalid address length: %d", strlen(addr_str));
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid address format");
        return ESP_FAIL;
    }

    // Parse MAC address
    uint8_t addr[6];
    int scan_res = sscanf(addr_str, "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
                         &addr[5], &addr[4], &addr[3], &addr[2], &addr[1], &addr[0]);

    if (scan_res != 6) {
        ESP_LOGE(TAG, "Failed to parse MAC address, sscanf returned: %d", scan_res);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid MAC address");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Connecting to: %02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

    // IMPORTANT: Stop scan before connecting
    extern esp_err_t ble_proxy_stop_scan(void);
    ble_proxy_stop_scan();
    vTaskDelay(pdMS_TO_TICKS(200));  // Give it time to stop

    esp_err_t conn_ret = ble_proxy_connect(addr);

    cJSON *json = cJSON_CreateObject();
    if (conn_ret == ESP_OK) {
        cJSON_AddBoolToObject(json, "success", true);
        cJSON_AddStringToObject(json, "message", "Connecting...");
    } else {
        cJSON_AddBoolToObject(json, "success", false);
        cJSON_AddStringToObject(json, "error", esp_err_to_name(conn_ret));
    }

    char *json_str = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));

    free(json_str);
    cJSON_Delete(json);
    return ESP_OK;
}

// Disconnect handler
static esp_err_t ble_disconnect_handler(httpd_req_t *req) {
    esp_err_t ret = ble_proxy_disconnect(0);

    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "success", ret == ESP_OK);

    char *json_str = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));

    free(json_str);
    cJSON_Delete(json);
    return ESP_OK;
}

// Connection status handler
static esp_err_t ble_conn_status_handler(httpd_req_t *req) {
    ble_connection_t conn_info;
    ble_proxy_get_connection_info(&conn_info);

    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "connected", conn_info.state == BLE_STATE_CONNECTED);
    cJSON_AddNumberToObject(json, "state", conn_info.state);

    if (conn_info.state == BLE_STATE_CONNECTED) {
        char mac_str[18];
        snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                conn_info.peer_addr[5], conn_info.peer_addr[4],
                conn_info.peer_addr[3], conn_info.peer_addr[2],
                conn_info.peer_addr[1], conn_info.peer_addr[0]);
        cJSON_AddStringToObject(json, "peer_addr", mac_str);
    }

    char *json_str = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));

    free(json_str);
    cJSON_Delete(json);
    return ESP_OK;
}

// Passkey input handler
static esp_err_t ble_passkey_handler(httpd_req_t *req) {
    char query[64] = {0};
    char pin_str[16] = {0};

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "pin", pin_str, sizeof(pin_str));
    }

    uint32_t pin = (uint32_t)atoi(pin_str);

    ESP_LOGI(TAG, "PIN entry: %06lu", pin);

    esp_err_t ret = ble_proxy_input_passkey(0, pin);

    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "success", ret == ESP_OK);

    char *json_str = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));

    free(json_str);
    cJSON_Delete(json);
    return ESP_OK;
}

// Register connection handlers
esp_err_t register_ble_connect_handlers(httpd_handle_t server) {
    httpd_uri_t connect_uri = {
        .uri = "/ble/connect",
        .method = HTTP_POST,
        .handler = ble_connect_handler
    };

    httpd_uri_t disconnect_uri = {
        .uri = "/ble/disconnect",
        .method = HTTP_POST,
        .handler = ble_disconnect_handler
    };

    httpd_uri_t status_uri = {
        .uri = "/ble/conn_status",
        .method = HTTP_GET,
        .handler = ble_conn_status_handler
    };

    httpd_uri_t passkey_uri = {
        .uri = "/ble/passkey",
        .method = HTTP_POST,
        .handler = ble_passkey_handler
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &connect_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &disconnect_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &status_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &passkey_uri));

    ESP_LOGI(TAG, "BLE connection handlers registered");
    return ESP_OK;
}