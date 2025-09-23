#include "ble_proxy.h"
#include "esp_log.h"
#include "host/ble_sm.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include <string.h>

static const char *TAG = "BLE_CONN";

// Nordic UART Service UUIDs (used by Meshtastic)
static const ble_uuid128_t nordic_uart_svc_uuid =
    BLE_UUID128_INIT(0x6e, 0x40, 0x00, 0x01, 0xb5, 0xa3, 0xf3, 0x93,
                     0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e);

static const ble_uuid128_t nordic_uart_tx_chr_uuid =
    BLE_UUID128_INIT(0x6e, 0x40, 0x00, 0x02, 0xb5, 0xa3, 0xf3, 0x93,
                     0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e);

static const ble_uuid128_t nordic_uart_rx_chr_uuid =
    BLE_UUID128_INIT(0x6e, 0x40, 0x00, 0x03, 0xb5, 0xa3, 0xf3, 0x93,
                     0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e);

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

static int ble_proxy_on_subscribe(uint16_t conn_handle,
                                  const struct ble_gatt_error *error,
                                  struct ble_gatt_attr *attr,
                                  void *arg);

// Connect to a BLE device
esp_err_t ble_proxy_connect(const uint8_t *addr) {
    if (current_conn.state != BLE_STATE_IDLE) {
        ESP_LOGW(TAG, "Already connected or connecting");
        return ESP_ERR_INVALID_STATE;
    }

    ble_addr_t peer_addr;
    memcpy(peer_addr.val, addr, 6);
    peer_addr.type = BLE_ADDR_PUBLIC;

    ESP_LOGI(TAG, "Connecting to %02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

    current_conn.state = BLE_STATE_CONNECTING;
    memcpy(current_conn.peer_addr, addr, 6);

    int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &peer_addr,
                             30000, // 30 second timeout
                             NULL,
                             ble_proxy_gap_connect_event,
                             NULL);

    if (rc != 0) {
        ESP_LOGE(TAG, "Connection failed: %d", rc);
        current_conn.state = BLE_STATE_IDLE;
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

            // Start service discovery for Nordic UART
            ble_gattc_disc_svc_by_uuid(current_conn.conn_handle,
                                       &nordic_uart_svc_uuid.u,
                                       ble_proxy_on_svc_disc,
                                       NULL);

            if (connect_callback) {
                connect_callback(current_conn.conn_handle, current_conn.peer_addr);
            }
        } else {
            ESP_LOGE(TAG, "Connection failed: status=%d", event->connect.status);
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
        ESP_LOGI(TAG, "Passkey action required: %d", event->passkey.params.action);
        if (event->passkey.params.action == BLE_SM_IOACT_INPUT) {
            ESP_LOGI(TAG, "Enter passkey on device");
            if (passkey_callback) {
                passkey_callback(event->passkey.conn_handle, 0);
            }
        } else if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
            // Generate a random passkey for display
            uint32_t passkey = 123456;  // Default for testing
            ESP_LOGI(TAG, "Display passkey: %06lu", passkey);
            if (passkey_callback) {
                passkey_callback(event->passkey.conn_handle, passkey);
            }
        }
        break;
    }

    return 0;
}

// Service discovery callback
static int ble_proxy_on_svc_disc(uint16_t conn_handle,
                                 const struct ble_gatt_error *error,
                                 const struct ble_gatt_svc *service,
                                 void *arg) {
    if (error->status == 0 && service != NULL) {
        ESP_LOGI(TAG, "Found Nordic UART service");
        uart_svc_handle = service->start_handle;

        // Discover characteristics
        ble_gattc_disc_all_chrs(conn_handle,
                                service->start_handle,
                                service->end_handle,
                                ble_proxy_on_chr_disc,
                                NULL);
    }
    return 0;
}

// Characteristic discovery callback
static int ble_proxy_on_chr_disc(uint16_t conn_handle,
                                 const struct ble_gatt_error *error,
                                 const struct ble_gatt_chr *chr,
                                 void *arg) {
    if (error->status == 0 && chr != NULL) {
        if (ble_uuid_cmp(&chr->uuid.u, &nordic_uart_tx_chr_uuid.u) == 0) {
            ESP_LOGI(TAG, "Found TX characteristic");
            uart_tx_handle = chr->val_handle;

            // Subscribe to notifications by writing to CCCD
            uint8_t notify_on[2] = {0x01, 0x00};
            ble_gattc_write_flat(conn_handle, chr->val_handle + 1,
                                notify_on, sizeof(notify_on),
                                ble_proxy_on_subscribe, NULL);
        } else if (ble_uuid_cmp(&chr->uuid.u, &nordic_uart_rx_chr_uuid.u) == 0) {
            ESP_LOGI(TAG, "Found RX characteristic");
            uart_rx_handle = chr->val_handle;
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
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        conn_handle = current_conn.conn_handle;
    }

    struct ble_sm_io io = {
        .action = BLE_SM_IOACT_INPUT,
        .passkey = passkey
    };

    int rc = ble_sm_inject_io(conn_handle, &io);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to inject passkey: %d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Passkey entered: %06lu", (unsigned long)io.passkey);
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