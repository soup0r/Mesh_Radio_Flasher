// bt_proxy.c - Bluetooth to WiFi Proxy for RAK4631
#ifdef CONFIG_IDF_TARGET_ESP32C3
// ESP32C3 doesn't support Bluedroid - stub implementation
#warning "Bluetooth proxy not available on ESP32C3"

esp_err_t bt_proxy_init(uint16_t tcp_port) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t bt_proxy_scan_and_connect(const char *device_name) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t bt_proxy_disconnect(void) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t bt_proxy_send_command(const char *command) { return ESP_ERR_NOT_SUPPORTED; }
void bt_proxy_get_stats(bt_proxy_stats_t *stats) { if (stats) memset(stats, 0, sizeof(bt_proxy_stats_t)); }
esp_err_t bt_proxy_deinit(void) { return ESP_ERR_NOT_SUPPORTED; }
void bt_proxy_set_auto_reconnect(bool enable) {}
void bt_proxy_set_target_name(const char *name) {}
#include <string.h>

#else
// Original ESP32/ESP32S3 implementation follows...
#include "bt_proxy.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatt_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include <string.h>
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"

static const char *TAG = "BT_PROXY";

// Nordic UART Service (NUS) UUIDs - commonly used by RAK4631
#define NUS_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR_UUID        "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // Nordic TX (we receive)
#define NUS_RX_CHAR_UUID        "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // Nordic RX (we send)

// BLE Client state
typedef enum {
    BLE_STATE_IDLE = 0,
    BLE_STATE_SCANNING,
    BLE_STATE_CONNECTING,
    BLE_STATE_CONNECTED,
    BLE_STATE_DISCOVERING,
    BLE_STATE_READY
} ble_state_t;

// Connection info
typedef struct {
    esp_bd_addr_t remote_bda;
    esp_ble_addr_type_t addr_type;
    uint16_t conn_id;
    esp_gatt_if_t gattc_if;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t tx_char_handle;  // Handle for receiving from RAK
    uint16_t rx_char_handle;  // Handle for sending to RAK
    uint16_t tx_descr_handle; // For enabling notifications
    bool is_connected;
    ble_state_t state;
} ble_connection_t;

// TCP Server state
typedef struct {
    int listen_sock;
    int client_socks[MAX_PROXY_CLIENTS];
    int client_count;
    uint16_t port;
    bool running;
    SemaphoreHandle_t mutex;
} tcp_server_t;

// Proxy state
typedef struct {
    ble_connection_t ble;
    tcp_server_t tcp;
    QueueHandle_t ble_to_tcp_queue;
    QueueHandle_t tcp_to_ble_queue;
    char target_name[32];
    bool auto_reconnect;
    uint32_t bytes_proxied;
    uint32_t reconnect_attempts;
} proxy_state_t;

static proxy_state_t proxy = {0};
static esp_gatt_if_t g_gattc_if = ESP_GATT_IF_NONE;

// UUID conversion helper
static void str_to_uuid128(const char *str, esp_bt_uuid_t *uuid) {
    uuid->len = ESP_UUID_LEN_128;
    // Parse UUID string like "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
    sscanf(str, "%2hhx%2hhx%2hhx%2hhx-%2hhx%2hhx-%2hhx%2hhx-%2hhx%2hhx-%2hhx%2hhx%2hhx%2hhx%2hhx%2hhx",
           &uuid->uuid.uuid128[15], &uuid->uuid.uuid128[14], &uuid->uuid.uuid128[13], &uuid->uuid.uuid128[12],
           &uuid->uuid.uuid128[11], &uuid->uuid.uuid128[10], &uuid->uuid.uuid128[9], &uuid->uuid.uuid128[8],
           &uuid->uuid.uuid128[7], &uuid->uuid.uuid128[6], &uuid->uuid.uuid128[5], &uuid->uuid.uuid128[4],
           &uuid->uuid.uuid128[3], &uuid->uuid.uuid128[2], &uuid->uuid.uuid128[1], &uuid->uuid.uuid128[0]);
}

// Forward data from BLE to TCP clients
static void forward_ble_to_tcp(const uint8_t *data, uint16_t len) {
    if (!proxy.tcp.running || proxy.tcp.client_count == 0) {
        return;
    }
    
    xSemaphoreTake(proxy.tcp.mutex, portMAX_DELAY);
    
    for (int i = 0; i < MAX_PROXY_CLIENTS; i++) {
        if (proxy.tcp.client_socks[i] > 0) {
            int sent = send(proxy.tcp.client_socks[i], data, len, MSG_DONTWAIT);
            
            if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                // Client disconnected
                ESP_LOGW(TAG, "TCP client %d disconnected", i);
                close(proxy.tcp.client_socks[i]);
                proxy.tcp.client_socks[i] = 0;
                proxy.tcp.client_count--;
            } else if (sent > 0) {
                proxy.bytes_proxied += sent;
            }
        }
    }
    
    xSemaphoreGive(proxy.tcp.mutex);
}

// Send data to RAK4631 via BLE
static esp_err_t send_to_ble(const uint8_t *data, uint16_t len) {
    if (proxy.ble.state != BLE_STATE_READY || !proxy.ble.is_connected) {
        ESP_LOGW(TAG, "BLE not ready for sending");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Nordic UART Service typically has 20-byte MTU limit
    const uint16_t chunk_size = 20;
    uint16_t offset = 0;
    
    while (offset < len) {
        uint16_t to_send = (len - offset) > chunk_size ? chunk_size : (len - offset);
        
        esp_err_t ret = esp_ble_gattc_write_char(
            proxy.ble.gattc_if,
            proxy.ble.conn_id,
            proxy.ble.rx_char_handle,
            to_send,
            (uint8_t*)data + offset,
            ESP_GATT_WRITE_TYPE_NO_RSP,
            ESP_GATT_AUTH_REQ_NONE
        );
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write to BLE: %s", esp_err_to_name(ret));
            return ret;
        }
        
        offset += to_send;
        proxy.bytes_proxied += to_send;
        
        // Small delay between chunks to avoid overwhelming the connection
        if (offset < len) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    
    return ESP_OK;
}

// GAP callback for scanning
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Scan parameters set, starting scan for RAK4631...");
            esp_ble_gap_start_scanning(30);  // Scan for 30 seconds
            break;
            
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Scan started successfully");
                proxy.ble.state = BLE_STATE_SCANNING;
            } else {
                ESP_LOGE(TAG, "Scan start failed: %d", param->scan_start_cmpl.status);
            }
            break;
            
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
            
            if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                // Check if this is our target device
                char name[32] = {0};
                uint8_t *adv_name = NULL;
                uint8_t adv_name_len = 0;
                
                adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                    ESP_BLE_AD_TYPE_NAME_CMPL,
                                                    &adv_name_len);
                
                if (adv_name) {
                    memcpy(name, adv_name, adv_name_len < 31 ? adv_name_len : 31);
                    
                    // Check if this is our target RAK4631
                    if (strstr(name, proxy.target_name) || strstr(name, "RAK")) {
                        ESP_LOGI(TAG, "Found target device: %s", name);
                        ESP_LOGI(TAG, "Address: %02X:%02X:%02X:%02X:%02X:%02X",
                                scan_result->scan_rst.bda[0], scan_result->scan_rst.bda[1],
                                scan_result->scan_rst.bda[2], scan_result->scan_rst.bda[3],
                                scan_result->scan_rst.bda[4], scan_result->scan_rst.bda[5]);
                        
                        // Stop scanning
                        esp_ble_gap_stop_scanning();
                        
                        // Save device info
                        memcpy(proxy.ble.remote_bda, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
                        proxy.ble.addr_type = scan_result->scan_rst.ble_addr_type;
                        
                        // Connect to device
                        proxy.ble.state = BLE_STATE_CONNECTING;
                        esp_ble_gattc_open(proxy.ble.gattc_if, proxy.ble.remote_bda, 
                                          proxy.ble.addr_type, true);
                    }
                }
            }
            break;
        }
            
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            ESP_LOGI(TAG, "Scan stopped");
            if (proxy.ble.state == BLE_STATE_SCANNING) {
                proxy.ble.state = BLE_STATE_IDLE;
                
                if (proxy.auto_reconnect && !proxy.ble.is_connected) {
                    ESP_LOGW(TAG, "Target not found, retrying in 5 seconds...");
                    vTaskDelay(pdMS_TO_TICKS(5000));
                    bt_proxy_scan_and_connect(proxy.target_name);
                }
            }
            break;
            
        default:
            break;
    }
}

// GATT Client callback
static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                               esp_ble_gattc_cb_param_t *param) {
    switch (event) {
        case ESP_GATTC_REG_EVT:
            ESP_LOGI(TAG, "GATT client registered, IF %d", gattc_if);
            proxy.ble.gattc_if = gattc_if;
            break;
            
        case ESP_GATTC_OPEN_EVT:
            if (param->open.status == ESP_GATT_OK) {
                ESP_LOGI(TAG, "Connected to BLE device");
                proxy.ble.conn_id = param->open.conn_id;
                proxy.ble.is_connected = true;
                proxy.ble.state = BLE_STATE_CONNECTED;
                proxy.reconnect_attempts = 0;
                
                // Update connection parameters for better throughput
                esp_ble_conn_update_params_t conn_params = {
                    .latency = 0,
                    .max_int = 0x20,  // 40ms
                    .min_int = 0x10,  // 20ms
                    .timeout = 400    // 4 seconds
                };
                memcpy(conn_params.bda, proxy.ble.remote_bda, ESP_BD_ADDR_LEN);
                esp_ble_gap_update_conn_params(&conn_params);
                
                // Start service discovery
                esp_ble_gattc_search_service(gattc_if, param->open.conn_id, NULL);
                proxy.ble.state = BLE_STATE_DISCOVERING;
            } else {
                ESP_LOGE(TAG, "Failed to connect: %d", param->open.status);
                proxy.ble.is_connected = false;
                proxy.ble.state = BLE_STATE_IDLE;
                
                if (proxy.auto_reconnect) {
                    proxy.reconnect_attempts++;
                    vTaskDelay(pdMS_TO_TICKS(1000 * proxy.reconnect_attempts));
                    bt_proxy_scan_and_connect(proxy.target_name);
                }
            }
            break;
            
        case ESP_GATTC_SEARCH_RES_EVT: {
            // Look for Nordic UART Service
            esp_bt_uuid_t nus_uuid;
            str_to_uuid128(NUS_SERVICE_UUID, &nus_uuid);
            
            if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128) {
                if (memcmp(param->search_res.srvc_id.uuid.uuid.uuid128, 
                          nus_uuid.uuid.uuid128, ESP_UUID_LEN_128) == 0) {
                    ESP_LOGI(TAG, "Found Nordic UART Service");
                    proxy.ble.service_start_handle = param->search_res.start_handle;
                    proxy.ble.service_end_handle = param->search_res.end_handle;
                }
            }
            break;
        }
            
        case ESP_GATTC_SEARCH_CMPL_EVT:
            if (param->search_cmpl.status == ESP_GATT_OK) {
                ESP_LOGI(TAG, "Service discovery complete");
                
                // Get characteristics
                esp_gattc_char_elem_t char_elem[10];
                uint16_t char_count = 10;
                
                esp_gatt_status_t status = esp_ble_gattc_get_all_char(
                    gattc_if, proxy.ble.conn_id,
                    proxy.ble.service_start_handle,
                    proxy.ble.service_end_handle,
                    char_elem, &char_count, 0
                );
                
                if (status == ESP_GATT_OK) {
                    ESP_LOGI(TAG, "Found %d characteristics", char_count);
                    
                    esp_bt_uuid_t tx_uuid, rx_uuid;
                    str_to_uuid128(NUS_TX_CHAR_UUID, &tx_uuid);
                    str_to_uuid128(NUS_RX_CHAR_UUID, &rx_uuid);
                    
                    for (int i = 0; i < char_count; i++) {
                        if (char_elem[i].uuid.len == ESP_UUID_LEN_128) {
                            if (memcmp(char_elem[i].uuid.uuid.uuid128, 
                                      tx_uuid.uuid.uuid128, ESP_UUID_LEN_128) == 0) {
                                proxy.ble.tx_char_handle = char_elem[i].char_handle;
                                ESP_LOGI(TAG, "Found TX characteristic (handle 0x%04X)", 
                                        proxy.ble.tx_char_handle);
                                
                                // Register for notifications
                                esp_ble_gattc_register_for_notify(gattc_if, 
                                                                 proxy.ble.remote_bda,
                                                                 proxy.ble.tx_char_handle);
                            } else if (memcmp(char_elem[i].uuid.uuid.uuid128, 
                                            rx_uuid.uuid.uuid128, ESP_UUID_LEN_128) == 0) {
                                proxy.ble.rx_char_handle = char_elem[i].char_handle;
                                ESP_LOGI(TAG, "Found RX characteristic (handle 0x%04X)", 
                                        proxy.ble.rx_char_handle);
                            }
                        }
                    }
                    
                    if (proxy.ble.tx_char_handle && proxy.ble.rx_char_handle) {
                        proxy.ble.state = BLE_STATE_READY;
                        ESP_LOGI(TAG, "BLE proxy ready!");
                    }
                }
            }
            break;
            
        case ESP_GATTC_REG_FOR_NOTIFY_EVT:
            if (param->reg_for_notify.status == ESP_GATT_OK) {
                ESP_LOGI(TAG, "Registered for notifications");
                
                // Enable notifications by writing to descriptor
                uint8_t notify_en[2] = {0x01, 0x00};
                esp_ble_gattc_write_char_descr(
                    gattc_if, proxy.ble.conn_id,
                    proxy.ble.tx_char_handle + 1,  // Descriptor usually at handle+1
                    sizeof(notify_en), notify_en,
                    ESP_GATT_WRITE_TYPE_RSP,
                    ESP_GATT_AUTH_REQ_NONE
                );
            }
            break;
            
        case ESP_GATTC_NOTIFY_EVT:
            // Received data from RAK4631
            ESP_LOGD(TAG, "Received %d bytes from BLE", param->notify.value_len);
            forward_ble_to_tcp(param->notify.value, param->notify.value_len);
            break;
            
        case ESP_GATTC_DISCONNECT_EVT:
            ESP_LOGW(TAG, "Disconnected from BLE device");
            proxy.ble.is_connected = false;
            proxy.ble.state = BLE_STATE_IDLE;
            
            if (proxy.auto_reconnect) {
                ESP_LOGI(TAG, "Attempting reconnection...");
                vTaskDelay(pdMS_TO_TICKS(2000));
                bt_proxy_scan_and_connect(proxy.target_name);
            }
            break;
            
        default:
            break;
    }
}

// TCP to BLE forwarding task
static void tcp_to_ble_task(void *arg) {
    uint8_t buffer[256];
    
    while (proxy.tcp.running) {
        xSemaphoreTake(proxy.tcp.mutex, portMAX_DELAY);
        
        for (int i = 0; i < MAX_PROXY_CLIENTS; i++) {
            if (proxy.tcp.client_socks[i] > 0) {
                int len = recv(proxy.tcp.client_socks[i], buffer, sizeof(buffer), MSG_DONTWAIT);
                
                if (len > 0) {
                    // Forward to BLE
                    xSemaphoreGive(proxy.tcp.mutex);  // Release mutex before BLE operation
                    
                    esp_err_t ret = send_to_ble(buffer, len);
                    if (ret != ESP_OK) {
                        ESP_LOGW(TAG, "Failed to send to BLE: %s", esp_err_to_name(ret));
                    }
                    
                    xSemaphoreTake(proxy.tcp.mutex, portMAX_DELAY);  // Re-acquire mutex
                } else if (len == 0) {
                    // Client disconnected
                    ESP_LOGI(TAG, "TCP client %d disconnected", i);
                    close(proxy.tcp.client_socks[i]);
                    proxy.tcp.client_socks[i] = 0;
                    proxy.tcp.client_count--;
                }
            }
        }
        
        xSemaphoreGive(proxy.tcp.mutex);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    vTaskDelete(NULL);
}

// TCP server task
static void tcp_server_task(void *arg) {
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    
    // Create socket
    proxy.tcp.listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (proxy.tcp.listen_sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        vTaskDelete(NULL);
        return;
    }
    
    // Set socket options
    int opt = 1;
    setsockopt(proxy.tcp.listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    // Set non-blocking
    int flags = fcntl(proxy.tcp.listen_sock, F_GETFL, 0);
    fcntl(proxy.tcp.listen_sock, F_SETFL, flags | O_NONBLOCK);
    
    // Bind
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(proxy.tcp.port);
    
    if (bind(proxy.tcp.listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Socket bind failed");
        close(proxy.tcp.listen_sock);
        vTaskDelete(NULL);
        return;
    }
    
    // Listen
    if (listen(proxy.tcp.listen_sock, MAX_PROXY_CLIENTS) < 0) {
        ESP_LOGE(TAG, "Socket listen failed");
        close(proxy.tcp.listen_sock);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "TCP proxy server listening on port %d", proxy.tcp.port);
    proxy.tcp.running = true;
    
    // Start forwarding task
    xTaskCreate(tcp_to_ble_task, "tcp_to_ble", 4096, NULL, 5, NULL);
    
    while (proxy.tcp.running) {
        int client_sock = accept(proxy.tcp.listen_sock, 
                                (struct sockaddr *)&client_addr, &client_len);
        
        if (client_sock >= 0) {
            // Set non-blocking
            flags = fcntl(client_sock, F_GETFL, 0);
            fcntl(client_sock, F_SETFL, flags | O_NONBLOCK);
            
            xSemaphoreTake(proxy.tcp.mutex, portMAX_DELAY);
            
            // Add to client list
            bool added = false;
            for (int i = 0; i < MAX_PROXY_CLIENTS; i++) {
                if (proxy.tcp.client_socks[i] == 0) {
                    proxy.tcp.client_socks[i] = client_sock;
                    proxy.tcp.client_count++;
                    added = true;
                    
                    ESP_LOGI(TAG, "TCP client connected from %s:%d (slot %d)",
                            inet_ntoa(client_addr.sin_addr),
                            ntohs(client_addr.sin_port), i);
                    
                    // Send status message
                    const char *msg = proxy.ble.is_connected ? 
                        "BLE Proxy Connected to RAK4631\r\n" : 
                        "BLE Proxy - RAK4631 Not Connected\r\n";
                    send(client_sock, msg, strlen(msg), 0);
                    break;
                }
            }
            
            if (!added) {
                ESP_LOGW(TAG, "Max TCP clients reached");
                const char *msg = "Max connections reached\r\n";
                send(client_sock, msg, strlen(msg), 0);
                close(client_sock);
            }
            
            xSemaphoreGive(proxy.tcp.mutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    close(proxy.tcp.listen_sock);
    vTaskDelete(NULL);
}

// Initialize BLE proxy
esp_err_t bt_proxy_init(uint16_t tcp_port) {
    ESP_LOGI(TAG, "Initializing Bluetooth proxy on port %d", tcp_port);
    
    // Initialize proxy state
    memset(&proxy, 0, sizeof(proxy_state_t));
    proxy.tcp.port = tcp_port;
    proxy.auto_reconnect = true;
    strcpy(proxy.target_name, "RAK");  // Default search pattern
    
    // Create mutex
    proxy.tcp.mutex = xSemaphoreCreateMutex();
    if (!proxy.tcp.mutex) {
        return ESP_ERR_NO_MEM;
    }
    
    // Create queues
    proxy.ble_to_tcp_queue = xQueueCreate(32, sizeof(ble_data_t));
    proxy.tcp_to_ble_queue = xQueueCreate(32, sizeof(ble_data_t));
    
    // Initialize Bluetooth
    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGE(TAG, "Failed to release classic BT memory: %s", esp_err_to_name(ret));
        return ret;
    }
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Failed to init BT controller: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "Failed to enable BT controller: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Failed to init bluedroid: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Failed to enable bluedroid: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register callbacks
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "Failed to register GAP callback: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_ble_gattc_register_callback(gattc_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "Failed to register GATTC callback: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_ble_gattc_app_register(0);
    if (ret) {
        ESP_LOGE(TAG, "Failed to register GATTC app: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set MTU
    ret = esp_ble_gatt_set_local_mtu(200);
    if (ret) {
        ESP_LOGE(TAG, "Failed to set MTU: %s", esp_err_to_name(ret));
    }
    
    // Start TCP server
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Bluetooth proxy initialized");
    return ESP_OK;
}

// Scan and connect to RAK4631
esp_err_t bt_proxy_scan_and_connect(const char *device_name) {
    if (device_name) {
        strncpy(proxy.target_name, device_name, sizeof(proxy.target_name) - 1);
    }
    
    ESP_LOGI(TAG, "Scanning for device: %s", proxy.target_name);
    
    // Set scan parameters
    esp_ble_scan_params_t scan_params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,  // 50ms
        .scan_window = 0x30,    // 30ms
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
    };
    
    esp_err_t ret = esp_ble_gap_set_scan_params(&scan_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set scan params: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

// Get proxy statistics
void bt_proxy_get_stats(bt_proxy_stats_t *stats) {
    if (!stats) return;
    
    stats->ble_connected = proxy.ble.is_connected;
    stats->tcp_clients = proxy.tcp.client_count;
    stats->bytes_proxied = proxy.bytes_proxied;
    stats->reconnect_attempts = proxy.reconnect_attempts;
    
    if (proxy.ble.is_connected) {
        memcpy(stats->device_addr, proxy.ble.remote_bda, ESP_BD_ADDR_LEN);
    } else {
        memset(stats->device_addr, 0, ESP_BD_ADDR_LEN);
    }
}

// Disconnect from BLE device
esp_err_t bt_proxy_disconnect(void) {
    if (proxy.ble.is_connected) {
        ESP_LOGI(TAG, "Disconnecting from BLE device");
        proxy.auto_reconnect = false;
        esp_ble_gattc_close(proxy.ble.gattc_if, proxy.ble.conn_id);
        return ESP_OK;
    }
    return ESP_ERR_INVALID_STATE;
}

// Send command to RAK4631
esp_err_t bt_proxy_send_command(const char *command) {
    if (!command) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return send_to_ble((const uint8_t*)command, strlen(command));
}

// Deinitialize proxy
esp_err_t bt_proxy_deinit(void) {
    ESP_LOGI(TAG, "Deinitializing Bluetooth proxy");
    
    // Stop auto-reconnect
    proxy.auto_reconnect = false;
    
    // Disconnect if connected
    if (proxy.ble.is_connected) {
        bt_proxy_disconnect();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Stop TCP server
    proxy.tcp.running = false;
    if (proxy.tcp.listen_sock > 0) {
        close(proxy.tcp.listen_sock);
    }
    
    // Close all client connections
    xSemaphoreTake(proxy.tcp.mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_PROXY_CLIENTS; i++) {
        if (proxy.tcp.client_socks[i] > 0) {
            close(proxy.tcp.client_socks[i]);
        }
    }
    xSemaphoreGive(proxy.tcp.mutex);
    
    // Cleanup Bluetooth
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    
    // Delete queues and mutex
    if (proxy.ble_to_tcp_queue) {
        vQueueDelete(proxy.ble_to_tcp_queue);
    }
    if (proxy.tcp_to_ble_queue) {
        vQueueDelete(proxy.tcp_to_ble_queue);
    }
    if (proxy.tcp.mutex) {
        vSemaphoreDelete(proxy.tcp.mutex);
    }
    
    ESP_LOGI(TAG, "Bluetooth proxy deinitialized");
    return ESP_OK;
}#endif
