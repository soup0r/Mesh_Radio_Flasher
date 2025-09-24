#include "ble_proxy.h"
#include "esp_log.h"
#include "host/ble_sm.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include <string.h>

static const char *TAG = "BLE_CONN";

// Connection state structure
typedef struct {
    uint16_t conn_handle;
    uint16_t svc_start, svc_end;
    uint16_t tx_val;    // TX value handle (device->ESP notify)
    uint16_t rx_val;    // RX value handle (ESP->device write)
    uint16_t tx_cccd;   // CCCD descriptor handle
    uint8_t tx_props;   // TX characteristic properties (for indicate vs notify)
    bool have_serial_service;   // found target service (NUS or Meshtastic)
    bool chars_done;            // characteristics discovery finished
    bool dsc_done;              // descriptor discovery finished
    bool encrypted;             // link is encrypted
    bool notify_enabled;        // CCCD write completed successfully
} uart_ctx_t;

static uart_ctx_t uart = {
    .conn_handle = BLE_HS_CONN_HANDLE_NONE
};

// Proxy state management (simplified)
static volatile proxy_state_t s_state = PROXY_IDLE;

// Service and characteristic UUIDs
static ble_uuid128_t UUID_NUS_SVC, UUID_NUS_TX, UUID_NUS_RX;
static ble_uuid128_t UUID_MESH_SVC; // Meshtastic 6ba1b218-15a8-461f-9fa8-5dcae273eafd

static void init_uuids(void) {
    static bool inited = false;
    if (inited) return;
    inited = true;
    ble_uuid_from_str((ble_uuid_any_t*)&UUID_NUS_SVC,  "6e400001-b5a3-f393-e0a9-e50e24dcca9e");
    ble_uuid_from_str((ble_uuid_any_t*)&UUID_NUS_TX,   "6e400003-b5a3-f393-e0a9-e50e24dcca9e");
    ble_uuid_from_str((ble_uuid_any_t*)&UUID_NUS_RX,   "6e400002-b5a3-f393-e0a9-e50e24dcca9e");
    ble_uuid_from_str((ble_uuid_any_t*)&UUID_MESH_SVC, "6ba1b218-15a8-461f-9fa8-5dcae273eafd");
}

// State accessor functions
proxy_state_t ble_proxy_get_state(void) {
    return s_state;
}

bool ble_proxy_gatt_ready(void) {
    return uart.chars_done && uart.tx_val && uart.rx_val && uart.notify_enabled;
}

uint16_t ble_proxy_get_rx_handle(void) {
    return uart.rx_val;
}

uint16_t ble_proxy_get_tx_handle(void) {
    return uart.tx_val;
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




// Debug discovery callbacks
static int ble_proxy_on_all_svc_disc(uint16_t conn_handle,
                                     const struct ble_gatt_error *error,
                                     const struct ble_gatt_svc *service,
                                     void *arg);

static int ble_proxy_on_debug_chr_disc(uint16_t conn_handle,
                                       const struct ble_gatt_error *error,
                                       const struct ble_gatt_chr *chr,
                                       void *arg);

// New discovery callback forward declarations
static int on_disc_svc(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *service, void *arg);
static int on_disc_chr(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg);
static int on_disc_dsc(uint16_t ch, const struct ble_gatt_error *err, uint16_t chr_def_handle, const struct ble_gatt_dsc *dsc, void *arg);
static int on_cccd_written(uint16_t ch, const struct ble_gatt_error *err, struct ble_gatt_attr *attr, void *arg);


// Forward declarations for new discovery callbacks
static int on_disc_svc(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *service, void *arg);
static int on_disc_chr(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg);

// Connect to a BLE device
esp_err_t ble_proxy_connect(const uint8_t *addr) {
    init_uuids();

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
    s_state = PROXY_CONNECTING;
    // Reset uart context
    uart = (uart_ctx_t){.conn_handle = BLE_HS_CONN_HANDLE_NONE};
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

// Service discovery callback - only pick target service & start char discovery
static int on_disc_svc(uint16_t ch, const struct ble_gatt_error *err, const struct ble_gatt_svc *svc, void *arg)
{
    if (err->status == 0 && svc) {
        // Is this NUS or Meshtastic?
        bool is_nus  = (ble_uuid_cmp(&svc->uuid.u, &UUID_NUS_SVC.u)  == 0);
        bool is_mesh = (ble_uuid_cmp(&svc->uuid.u, &UUID_MESH_SVC.u) == 0);

        if (is_nus || is_mesh) {
            uart.have_serial_service = true;
            uart.svc_start = svc->start_handle;
            uart.svc_end   = svc->end_handle ? svc->end_handle : (svc->start_handle + 20); // guard
            ESP_LOGI(TAG, "Target service found (%s): handles %u-%u",
                     is_nus ? "NUS" : "Meshtastic", uart.svc_start, uart.svc_end);

            // Kick char discovery for this service range
            ble_gattc_disc_all_chrs(ch, uart.svc_start, uart.svc_end, on_disc_chr, NULL);
            return 0; // keep pumping; NimBLE will call EDONE when done
        }

        // (Optional) log other services for diagnostics
        return 0;
    }

    if (err->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "Service discovery finished.");
        // DO NOT disconnect here. Char discovery callback will decide.
        return 0;
    }

    ESP_LOGE(TAG, "Service discovery error: %d", err->status);
    return 0;
}

// Characteristic discovery callback - decide here by properties
static int on_disc_chr(uint16_t ch, const struct ble_gatt_error *err, const struct ble_gatt_chr *chr, void *arg)
{
    if (err->status == 0 && chr) {
        // TX = NOTIFY/INDICATE (device->ESP), RX = WRITE or WRITE_NO_RSP (ESP->device)
        if (chr->properties & (BLE_GATT_CHR_PROP_NOTIFY | BLE_GATT_CHR_PROP_INDICATE)) {
            uart.tx_val = chr->val_handle;
            uart.tx_props = chr->properties;  // store properties
            const char* type = (chr->properties & BLE_GATT_CHR_PROP_NOTIFY) ? "notify" : "indicate";
            ESP_LOGI(TAG, "TX (%s) val_handle = %u (def=%u)", type, chr->val_handle, chr->def_handle);
        }

        if (chr->properties & (BLE_GATT_CHR_PROP_WRITE | BLE_GATT_CHR_PROP_WRITE_NO_RSP)) {
            uart.rx_val = chr->val_handle;
            ESP_LOGI(TAG, "RX (write)  val_handle = %u", uart.rx_val);
        }

        return 0;
    }

    if (err->status == BLE_HS_EDONE) {
        uart.chars_done = true;
        ESP_LOGI(TAG, "Characteristic discovery complete");

        // Now discover descriptors for TX characteristic if we found one
        if (uart.tx_val) {
            ESP_LOGI(TAG, "Discovering descriptors for TX characteristic...");
            ble_gattc_disc_all_dscs(ch, uart.tx_val, uart.svc_end, on_disc_dsc, NULL);
        } else {
            ESP_LOGW(TAG, "No TX characteristic found");
        }
        return 0;
    }

    ESP_LOGE(TAG, "Char discovery error: %d", err->status);
    return 0;
}

// Descriptor discovery callback
static int on_disc_dsc(uint16_t ch, const struct ble_gatt_error *err, uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
    if (err->status == 0 && dsc) {
        if (ble_uuid_u16(&dsc->uuid.u) == 0x2902) {
            uart.tx_cccd = dsc->handle;
            ESP_LOGI(TAG, "Found CCCD at handle %u", dsc->handle);
        }
        return 0;
    }
    if (err->status == BLE_HS_EDONE) {
        uart.dsc_done = true;
        ESP_LOGI(TAG, "Descriptor discovery complete");

        if (!uart.tx_cccd) {
            ESP_LOGW(TAG, "No CCCD found for TX (val_handle=%u)", chr_val_handle);
            return 0;
        }

        // Only write CCCD if we have encryption (or don't require it)
        if (uart.encrypted || !uart.encrypted) {  // Try regardless of encryption for now
            // Choose notify (0x0001) or indicate (0x0002) based on properties
            uint8_t cccd_val[2];
            if (uart.tx_props & BLE_GATT_CHR_PROP_NOTIFY) {
                cccd_val[0] = 0x01; cccd_val[1] = 0x00;  // Enable notify
                ESP_LOGI(TAG, "Enabling notifications on CCCD %u...", uart.tx_cccd);
            } else if (uart.tx_props & BLE_GATT_CHR_PROP_INDICATE) {
                cccd_val[0] = 0x02; cccd_val[1] = 0x00;  // Enable indicate
                ESP_LOGI(TAG, "Enabling indications on CCCD %u...", uart.tx_cccd);
            } else {
                ESP_LOGW(TAG, "TX char has no notify/indicate properties: 0x%02x", uart.tx_props);
                return 0;
            }

            return ble_gattc_write_flat(ch, uart.tx_cccd, cccd_val, sizeof cccd_val, on_cccd_written, NULL);
        } else {
            ESP_LOGW(TAG, "Link not encrypted, waiting for encryption before CCCD write");
        }
        return 0;
    }
    ESP_LOGE(TAG, "Descriptor discovery error: %d", err->status);
    return 0;
}

// CCCD write completion callback
static int on_cccd_written(uint16_t ch, const struct ble_gatt_error *err, struct ble_gatt_attr *attr, void *arg)
{
    if (err->status == 0) {
        uart.notify_enabled = true;
        ESP_LOGI(TAG, "🔔 Notifications enabled (CCCD=%u)", attr->handle);

        // Now check if we have everything we need to start TCP proxy
        if (uart.tx_val && uart.rx_val && uart.notify_enabled) {
            ESP_LOGI(TAG, "✅ Serial over BLE ready (TX=%u RX=%u CCCD=%u). Starting TCP proxy…",
                     uart.tx_val, uart.rx_val, uart.tx_cccd);
            extern void start_tcp_proxy(void);
            start_tcp_proxy();
        }
    } else {
        ESP_LOGE(TAG, "CCCD write failed (%d) on %u", err->status, attr ? attr->handle : 0);
    }
    return 0;
}

// GAP event handler
static int ble_proxy_gap_connect_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "Connected to peer");
            current_conn.conn_handle = event->connect.conn_handle;
            current_conn.state = BLE_STATE_CONNECTED;

            // Reset and setup uart context
            uart = (uart_ctx_t){0}; // reset
            uart.conn_handle = event->connect.conn_handle;

            ESP_LOGI(TAG, "Connected, exchanging MTU...");
            ble_gattc_exchange_mtu(uart.conn_handle, NULL, NULL);

            // Initiate security/pairing before GATT operations
            ESP_LOGI(TAG, "Initiating security...");
            int rc = ble_gap_security_initiate(event->connect.conn_handle);
            if (rc != 0) {
                ESP_LOGW(TAG, "Security initiate failed: %d, proceeding anyway", rc);
                // Fallback: start service discovery without encryption
                ESP_LOGI(TAG, "Starting service discovery...");
                ble_gattc_disc_all_svcs(uart.conn_handle, on_disc_svc, NULL);
            }

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
            s_state = PROXY_IDLE;
            uart = (uart_ctx_t){.conn_handle = BLE_HS_CONN_HANDLE_NONE};
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "Disconnected: reason=%d", event->disconnect.reason);

        // Check for NULL before calling
        if (disconnect_callback != NULL) {
            disconnect_callback(current_conn.conn_handle, event->disconnect.reason);
        }

        // Stop TCP proxy first
        extern void stop_tcp_proxy(void);
        stop_tcp_proxy();

        // Reset state
        current_conn.state = BLE_STATE_IDLE;
        current_conn.conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_state = PROXY_IDLE;
        uart = (uart_ctx_t){0}; // clear handles
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

    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "Encryption change: status=%d", event->enc_change.status);

        if (event->enc_change.status == 0) {
            uart.encrypted = true;
            ESP_LOGI(TAG, "✅ Link encrypted, starting service discovery...");
            ble_gattc_disc_all_svcs(uart.conn_handle, on_disc_svc, NULL);
        } else {
            ESP_LOGW(TAG, "Encryption failed: %d, trying discovery anyway", event->enc_change.status);
            // Some devices don't require encryption, try anyway
            ble_gattc_disc_all_svcs(uart.conn_handle, on_disc_svc, NULL);
        }
        break;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (!len) break;

        uint8_t buf[512];
        if (len > sizeof buf) len = sizeof buf;
        os_mbuf_copydata(event->notify_rx.om, 0, len, buf);

        ESP_LOGI(TAG, "BLE ← notify: %u bytes", len);
        extern void tcp_forward_ble_data(uint8_t *data, uint16_t len);
        tcp_forward_ble_data(buf, len);

        // Also forward to registered callback if any
        if (data_callback != NULL) {
            data_callback(current_conn.conn_handle, buf, len);
        }
        break;
    }
    }

    return 0;
}


// Characteristic discovery callback
static int ble_proxy_on_chr_disc(uint16_t conn_handle,
                                 const struct ble_gatt_error *error,
                                 const struct ble_gatt_chr *chr,
                                 void *arg) {
    if (chr != NULL) {
        ESP_LOGI(TAG, "Found char: val_handle=%d, props=0x%02x",
                chr->val_handle, chr->properties);

        // Store ACTUAL handles from discovery
        if (chr->properties & BLE_GATT_CHR_PROP_NOTIFY) {
            uart_tx_handle = chr->val_handle;
            ESP_LOGI(TAG, "TX handle: %d", uart_tx_handle);
        }
        if (chr->properties & BLE_GATT_CHR_PROP_WRITE_NO_RSP) {
            uart_rx_handle = chr->val_handle;
            ESP_LOGI(TAG, "RX handle: %d", uart_rx_handle);
        }
    } else if (error->status == BLE_HS_EDONE) {
        if (uart_tx_handle && uart_rx_handle) {
            // Enable notifications using write_flat
            uint8_t notify[2] = {0x01, 0x00};
            int rc = ble_gattc_write_flat(conn_handle,
                                          uart_tx_handle + 1,  // CCCD
                                          notify, 2,
                                          NULL, NULL);  // Add callback params
            ESP_LOGI(TAG, "Enable notifications: %d", rc);

            ESP_LOGI(TAG, "✅ Starting TCP proxy with RX=%d, TX=%d",
                    uart_rx_handle, uart_tx_handle);
            extern void start_tcp_proxy(void);
            start_tcp_proxy();
        } else {
            ESP_LOGE(TAG, "❌ Characteristics not found - NOT starting proxy");
        }
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

    struct ble_sm_io io = {
        .action = BLE_SM_IOACT_INPUT,
        .passkey = passkey
    };

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

// Write helper - simple and clean
esp_err_t ble_proxy_send_data(const uint8_t *data, uint16_t len) {
    if (!ble_proxy_is_connected() || !uart.rx_val) {
        return ESP_ERR_INVALID_STATE;
    }

    int rc = ble_gattc_write_no_rsp_flat(uart.conn_handle, uart.rx_val, data, len);
    return rc == 0 ? ESP_OK : ESP_FAIL;
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

