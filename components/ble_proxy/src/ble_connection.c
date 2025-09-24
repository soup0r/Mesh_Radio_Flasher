#include "ble_proxy.h"
#include "esp_log.h"
#include "host/ble_sm.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include <string.h>

static const char *TAG = "BLE_CONN";

// Nordic UART Service UUIDs
static ble_uuid128_t nordic_uart_svc_uuid;
static ble_uuid128_t nordic_uart_tx_chr_uuid;
static ble_uuid128_t nordic_uart_rx_chr_uuid;

// Initialize UUIDs from strings (call this once during init)
static void init_nordic_uart_uuids(void) {
    ble_uuid_from_str((ble_uuid_any_t *)&nordic_uart_svc_uuid,
                      "6e400001-b5a3-f393-e0a9-e50e24dcca9e");
    ble_uuid_from_str((ble_uuid_any_t *)&nordic_uart_tx_chr_uuid,
                      "6e400003-b5a3-f393-e0a9-e50e24dcca9e");
    ble_uuid_from_str((ble_uuid_any_t *)&nordic_uart_rx_chr_uuid,
                      "6e400002-b5a3-f393-e0a9-e50e24dcca9e");
}

// Connection state
static ble_connection_t current_conn = {
    .conn_handle = BLE_HS_CONN_HANDLE_NONE,
    .state = BLE_STATE_IDLE
};

// Callbacks
static ble_connect_cb_t connect_callback = NULL;
static ble_disconnect_cb_t disconnect_callback = NULL;
static ble_passkey_cb_t passkey_callback = NULL;
static ble_data_received_cb_t data_callback = NULL;

// GATT handles for Nordic UART
static uint16_t uart_svc_handle = 0;
static uint16_t uart_tx_handle = 0;
static uint16_t uart_rx_handle = 0;

// Connection callback
static int ble_proxy_gap_connect_event(struct ble_gap_event *event, void *arg);

// GATT discovery callbacks
static int ble_proxy_on_svc_disc(uint16_t conn_handle,
                                 const struct ble_gatt_error *error,
                                 const struct ble_gatt_svc *service,
                                 void *arg);

static int ble_proxy_on_chr_disc(uint16_t conn_handle,
                                 const struct ble_gatt_error *error,
                                 const struct ble_gatt_chr *chr,
                                 void *arg);

// Debug discovery callbacks
static int ble_proxy_on_all_svc_disc(uint16_t conn_handle,
                                     const struct ble_gatt_error *error,
                                     const struct ble_gatt_svc *service,
                                     void *arg);

static int ble_proxy_on_debug_chr_disc(uint16_t conn_handle,
                                       const struct ble_gatt_error *error,
                                       const struct ble_gatt_chr *chr,
                                       void *arg);

static int ble_proxy_on_subscribe(uint16_t conn_handle,
                                  const struct ble_gatt_error *error,
                                  struct ble_gatt_attr *attr,
                                  void *arg);

// Connect to a BLE device
esp_err_t ble_proxy_connect(const uint8_t *addr) {
    // Initialize UUIDs on first call
    static bool uuids_initialized = false;
    if (!uuids_initialized) {
        init_nordic_uart_uuids();
        uuids_initialized = true;
        ESP_LOGI(TAG, "Nordic UART UUIDs initialized");
    }

    if (current_conn.state != BLE_STATE_IDLE) {
        ESP_LOGW(TAG, "Already connected or connecting");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Connecting to %02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

    // CRITICAL: Cancel any ongoing discovery first
    int rc = ble_gap_disc_cancel();
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "Failed to cancel discovery: %d", rc);
    }

    // Small delay to ensure scan is stopped
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set up peer address - try RANDOM first (most nRF52/Meshtastic devices use RANDOM)
    ble_addr_t peer_addr;
    memcpy(peer_addr.val, addr, 6);
    peer_addr.type = BLE_ADDR_RANDOM;  // Most nRF52/Meshtastic devices use RANDOM

    current_conn.state = BLE_STATE_CONNECTING;
    memcpy(current_conn.peer_addr, addr, 6);

    // Use default connection parameters (IMPORTANT!)
    struct ble_gap_conn_params conn_params;
    memset(&conn_params, 0, sizeof(conn_params));
    conn_params.scan_itvl = 0x0010;
    conn_params.scan_window = 0x0010;
    conn_params.itvl_min = BLE_GAP_INITIAL_CONN_ITVL_MIN;
    conn_params.itvl_max = BLE_GAP_INITIAL_CONN_ITVL_MAX;
    conn_params.latency = 0;
    conn_params.supervision_timeout = 0x0100;
    conn_params.min_ce_len = 0;
    conn_params.max_ce_len = 0;

    rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &peer_addr,
                        30000,  // 30 second timeout
                        &conn_params,
                        ble_proxy_gap_connect_event,
                        NULL);

    if (rc != 0) {
        ESP_LOGE(TAG, "Connection failed: %d", rc);
        current_conn.state = BLE_STATE_IDLE;

        // If random address failed, we could retry with PUBLIC
        if (rc == BLE_HS_ENOTCONN) {
            ESP_LOGI(TAG, "Retrying with PUBLIC address type");
            peer_addr.type = BLE_ADDR_PUBLIC;
            rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &peer_addr,
                                30000, &conn_params,
                                ble_proxy_gap_connect_event, NULL);
            if (rc == 0) {
                current_conn.state = BLE_STATE_CONNECTING;
                return ESP_OK;
            }
        }
        return ESP_FAIL;
    }

    return ESP_OK;
}

// Disconnect
esp_err_t ble_proxy_disconnect(uint16_t conn_handle) {
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        conn_handle = current_conn.conn_handle;
    }

    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Disconnecting handle %d", conn_handle);
    current_conn.state = BLE_STATE_DISCONNECTING;

    int rc = ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "Disconnect failed: %d", rc);
        return ESP_FAIL;
    }

    return ESP_OK;
}

// GAP event handler
static int ble_proxy_gap_connect_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "Connected to peer");
            current_conn.conn_handle = event->connect.conn_handle;
            current_conn.state = BLE_STATE_CONNECTED;

            // Discover ALL services first to see what's available
            ESP_LOGI(TAG, "Starting full service discovery...");
            ble_gattc_disc_all_svcs(current_conn.conn_handle,
                                   ble_proxy_on_all_svc_disc,  // New callback
                                   NULL);

            if (connect_callback) {
                connect_callback(current_conn.conn_handle, current_conn.peer_addr);
            }
        } else {
            // Better error logging
            const char *error_msg;
            switch(event->connect.status) {
                case BLE_HS_ETIMEOUT:
                    error_msg = "Connection timeout - device may not be in pairing mode";
                    break;
                case BLE_HS_EDONE:
                    error_msg = "Operation already completed";
                    break;
                case BLE_HS_EBUSY:
                    error_msg = "Connection busy";
                    break;
                case BLE_HS_EREJECT:
                    error_msg = "Connection rejected by device";
                    break;
                default:
                    error_msg = "Unknown error";
                    break;
            }
            ESP_LOGE(TAG, "Connection failed: %s (status=%d)", error_msg, event->connect.status);
            current_conn.state = BLE_STATE_IDLE;
            current_conn.conn_handle = BLE_HS_CONN_HANDLE_NONE;
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected: reason=%d", event->disconnect.reason);
        if (disconnect_callback) {
            disconnect_callback(current_conn.conn_handle, event->disconnect.reason);
        }
        current_conn.state = BLE_STATE_IDLE;
        current_conn.conn_handle = BLE_HS_CONN_HANDLE_NONE;
        break;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        ESP_LOGI(TAG, "Passkey action: %d", event->passkey.params.action);

        // Auto-inject Meshtastic default PIN
        if (event->passkey.params.action == BLE_SM_IOACT_INPUT) {
            struct ble_sm_io io = {
                .action = BLE_SM_IOACT_INPUT,
                .passkey = 123456  // Meshtastic default
            };

            int rc = ble_sm_inject_io(event->passkey.conn_handle, &io);
            ESP_LOGI(TAG, "Auto-injected PIN 123456: %s",
                    rc == 0 ? "SUCCESS" : "FAILED");

            if (rc != 0 && passkey_callback) {
                passkey_callback(event->passkey.conn_handle, 0);
            }
        } else if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
            uint32_t passkey = 123456;  // Use default for display
            ESP_LOGI(TAG, "Display passkey: %06lu", passkey);
            if (passkey_callback) {
                passkey_callback(event->passkey.conn_handle, passkey);
            }
        }
        break;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        ESP_LOGI(TAG, "Notification from handle %d, len=%d",
                 event->notify_rx.attr_handle,
                 OS_MBUF_PKTLEN(event->notify_rx.om));

        // Extract data from mbuf chain
        uint8_t data[256];
        uint16_t data_len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (data_len > sizeof(data)) data_len = sizeof(data);

        os_mbuf_copydata(event->notify_rx.om, 0, data_len, data);

        ESP_LOG_BUFFER_HEX("BLE_RX", data, data_len);

        // Forward this data to registered callback
        if (data_callback) {
            data_callback(current_conn.conn_handle, data, data_len);
        }
        break;
    }
    }

    return 0;
}

// Service discovery callback
static int ble_proxy_on_svc_disc(uint16_t conn_handle,
                                 const struct ble_gatt_error *error,
                                 const struct ble_gatt_svc *service,
                                 void *arg) {
    if (error->status == 0 && service != NULL) {
        ESP_LOGI(TAG, "Found Nordic UART service: start=%d, end=%d",
                service->start_handle, service->end_handle);
        uart_svc_handle = service->start_handle;

        // Discover characteristics
        ble_gattc_disc_all_chrs(conn_handle,
                                service->start_handle,
                                service->end_handle,
                                ble_proxy_on_chr_disc,
                                NULL);
        return 0;
    } else if (error->status == BLE_HS_EDONE) {
        // Discovery complete
        ESP_LOGI(TAG, "Service discovery complete");
        if (uart_svc_handle == 0) {
            ESP_LOGW(TAG, "Nordic UART service not found!");
        }
    } else {
        ESP_LOGE(TAG, "Service discovery error: %d", error->status);
    }
    return 0;
}

// Characteristic discovery callback
static int ble_proxy_on_chr_disc(uint16_t conn_handle,
                                 const struct ble_gatt_error *error,
                                 const struct ble_gatt_chr *chr,
                                 void *arg) {
    if (chr != NULL) {
        // Just check if it has the properties we need
        if (chr->properties & BLE_GATT_CHR_PROP_NOTIFY) {
            uart_tx_handle = chr->val_handle;
            ESP_LOGI(TAG, "Found TX (notify) handle: %d", uart_tx_handle);

            // Subscribe immediately
            uint8_t val[2] = {0x01, 0x00};
            ble_gattc_write_flat(conn_handle, chr->val_handle + 1,
                                val, 2, NULL, NULL);
        }
        if (chr->properties & BLE_GATT_CHR_PROP_WRITE_NO_RSP) {
            uart_rx_handle = chr->val_handle;
            ESP_LOGI(TAG, "Found RX (write) handle: %d", uart_rx_handle);
        }

        // Once we have both, we're ready
        if (uart_tx_handle && uart_rx_handle) {
            ESP_LOGI(TAG, "✅ BLE ready - starting TCP proxy on port 4403");
            // TODO: start_tcp_proxy();  // Start the network bridge
        }
    }
    return 0;
}

// Subscribe callback
static int ble_proxy_on_subscribe(uint16_t conn_handle,
                                  const struct ble_gatt_error *error,
                                  struct ble_gatt_attr *attr,
                                  void *arg) {
    if (error->status == 0) {
        ESP_LOGI(TAG, "Subscribed to notifications");
    }
    return 0;
}

// Register callbacks
void ble_proxy_register_connect_cb(ble_connect_cb_t cb) {
    connect_callback = cb;
}

void ble_proxy_register_disconnect_cb(ble_disconnect_cb_t cb) {
    disconnect_callback = cb;
}

void ble_proxy_register_passkey_cb(ble_passkey_cb_t cb) {
    passkey_callback = cb;
}

void ble_proxy_register_data_cb(ble_data_received_cb_t cb) {
    data_callback = cb;
}

// Get connection info
esp_err_t ble_proxy_get_connection_info(ble_connection_t *conn_info) {
    if (conn_info) {
        *conn_info = current_conn;
        return ESP_OK;
    }
    return ESP_ERR_INVALID_ARG;
}

bool ble_proxy_is_connected(void) {
    return current_conn.state == BLE_STATE_CONNECTED;
}

// Input passkey
esp_err_t ble_proxy_input_passkey(uint16_t conn_handle, uint32_t passkey) {
    // Check if we're actually connected first
    if (current_conn.state != BLE_STATE_CONNECTED) {
        ESP_LOGE(TAG, "Cannot input passkey - not connected (state=%d)", current_conn.state);
        return ESP_ERR_INVALID_STATE;
    }

    if (conn_handle == 0) {
        conn_handle = current_conn.conn_handle;
    }

    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGE(TAG, "No valid connection handle");
        return ESP_ERR_INVALID_STATE;
    }

    struct ble_sm_io io;
    io.action = BLE_SM_IOACT_INPUT;
    io.passkey = passkey;

    int rc = ble_sm_inject_io(conn_handle, &io);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to inject passkey: %d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Passkey entered: %06lu", passkey);
    return ESP_OK;
}

// Set passkey (stub implementation)
esp_err_t ble_proxy_set_passkey(uint32_t passkey) {
    ESP_LOGI(TAG, "Set passkey: %06lu", passkey);
    return ESP_OK;
}

// Confirm passkey (stub implementation)
esp_err_t ble_proxy_confirm_passkey(uint16_t conn_handle, bool confirm) {
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        conn_handle = current_conn.conn_handle;
    }

    struct ble_sm_io io = {
        .action = BLE_SM_IOACT_NUMCMP,
        .numcmp_accept = confirm
    };

    int rc = ble_sm_inject_io(conn_handle, &io);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to confirm passkey: %d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Passkey confirmed: %s (action: %d)", confirm ? "yes" : "no", io.action);
    return ESP_OK;
}

// Send data to the connected device
esp_err_t ble_proxy_send_data(const uint8_t *data, uint16_t len) {
    if (current_conn.state != BLE_STATE_CONNECTED || uart_rx_handle == 0) {
        ESP_LOGE(TAG, "Not connected or RX characteristic not found");
        return ESP_ERR_INVALID_STATE;
    }

    // BLE MTU limit - typically 20 bytes for legacy, up to 512 for extended
    const uint16_t max_chunk = 20;  // Safe default

    uint16_t offset = 0;
    while (offset < len) {
        uint16_t chunk_len = (len - offset > max_chunk) ? max_chunk : (len - offset);

        int rc = ble_gattc_write_no_rsp_flat(current_conn.conn_handle,
                                             uart_rx_handle,
                                             data + offset,
                                             chunk_len);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to write data: %d", rc);
            return ESP_FAIL;
        }

        offset += chunk_len;
        if (offset < len) {
            vTaskDelay(pdMS_TO_TICKS(20));  // Small delay between chunks
        }
    }

    ESP_LOGI(TAG, "Sent %d bytes to device", len);
    return ESP_OK;
}

// Test function to verify data communication
void test_meshtastic_communication(void) {
    if (current_conn.state != BLE_STATE_CONNECTED) {
        ESP_LOGW(TAG, "Cannot test - not connected");
        return;
    }

    // Send a simple test message (Meshtastic protocol would go here)
    const char *test_msg = "Hello Meshtastic!";
    esp_err_t ret = ble_proxy_send_data((uint8_t*)test_msg, strlen(test_msg));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Test message sent successfully");
    } else {
        ESP_LOGE(TAG, "Failed to send test message");
    }
}

// Debug service discovery callback to discover ALL services
static int ble_proxy_on_all_svc_disc(uint16_t conn_handle,
                                     const struct ble_gatt_error *error,
                                     const struct ble_gatt_svc *service,
                                     void *arg) {
    if (error->status == 0 && service != NULL) {
        char buf[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&service->uuid.u, buf);
        ESP_LOGI(TAG, "Found service: UUID=%s, handles=%d-%d",
                buf, service->start_handle, service->end_handle);

        // Check if it's Nordic UART
        if (ble_uuid_cmp(&service->uuid.u, &nordic_uart_svc_uuid.u) == 0) {
            ESP_LOGI(TAG, "Found Nordic UART, discovering characteristics...");
            uart_svc_handle = service->start_handle;

            // Just discover the two characteristics we need
            ble_gattc_disc_all_chrs(conn_handle,
                                    service->start_handle,
                                    service->end_handle,
                                    ble_proxy_on_chr_disc,
                                    NULL);
        } else {
            // For any service, discover its characteristics to see what's available
            ble_gattc_disc_all_chrs(conn_handle,
                                    service->start_handle,
                                    service->end_handle,
                                    ble_proxy_on_debug_chr_disc,  // Debug callback
                                    NULL);
        }
        return 0;
    } else if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "Service discovery complete - found %s",
                uart_svc_handle ? "Nordic UART" : "no Nordic UART");
    }
    return 0;
}

// Debug callback to see ALL characteristics
static int ble_proxy_on_debug_chr_disc(uint16_t conn_handle,
                                       const struct ble_gatt_error *error,
                                       const struct ble_gatt_chr *chr,
                                       void *arg) {
    if (error->status == 0 && chr != NULL) {
        char buf[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&chr->uuid.u, buf);

        // Check properties
        char props[64] = {0};
        if (chr->properties & BLE_GATT_CHR_PROP_READ) strcat(props, "READ ");
        if (chr->properties & BLE_GATT_CHR_PROP_WRITE_NO_RSP) strcat(props, "WRITE_NO_RSP ");
        if (chr->properties & BLE_GATT_CHR_PROP_WRITE) strcat(props, "WRITE ");
        if (chr->properties & BLE_GATT_CHR_PROP_NOTIFY) strcat(props, "NOTIFY ");
        if (chr->properties & BLE_GATT_CHR_PROP_INDICATE) strcat(props, "INDICATE ");

        ESP_LOGI(TAG, "  Char: %s, handle=%d, props=[%s]",
                buf, chr->val_handle, props);

        // If we find ANY characteristic with WRITE+NOTIFY, we can use it
        if ((chr->properties & BLE_GATT_CHR_PROP_NOTIFY) &&
            (chr->properties & (BLE_GATT_CHR_PROP_WRITE | BLE_GATT_CHR_PROP_WRITE_NO_RSP))) {
            ESP_LOGI(TAG, "    -> Could use this for serial communication!");
        }
    }
    return 0;
}