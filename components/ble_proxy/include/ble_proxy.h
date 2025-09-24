#ifndef BLE_PROXY_H
#define BLE_PROXY_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"

// BLE device info structure
typedef struct {
    uint8_t addr[6];
    int8_t rssi;
    char name[32];
    bool has_name;
    uint32_t last_seen;
} ble_device_info_t;

// Maximum devices to track
#define BLE_MAX_DEVICES 20

// Initialize BLE proxy
esp_err_t ble_proxy_init(void);

// Start/stop scanning
esp_err_t ble_proxy_start_scan(uint32_t duration_sec);
esp_err_t ble_proxy_stop_scan(void);
bool ble_proxy_is_scanning(void);

// Get discovered devices
uint16_t ble_proxy_get_devices(ble_device_info_t *devices, uint16_t max_devices);

// Clear device list
void ble_proxy_clear_devices(void);

// Get device count
uint16_t ble_proxy_get_device_count(void);

// Connection states
typedef enum {
    BLE_STATE_IDLE = 0,
    BLE_STATE_SCANNING,
    BLE_STATE_CONNECTING,
    BLE_STATE_CONNECTED,
    BLE_STATE_DISCONNECTING
} ble_state_t;

// Proxy states
typedef enum {
    PROXY_IDLE = 0,
    PROXY_CONNECTING,
    PROXY_GATT_READY,     // Service + chrs + CCCD enabled + MTU exchanged
    PROXY_RUNNING         // TCP proxy is running
} proxy_state_t;

// Connection info
typedef struct {
    uint16_t conn_handle;
    uint8_t peer_addr[6];
    char peer_name[32];
    bool secured;
    bool bonded;
    ble_state_t state;
} ble_connection_t;

// Callbacks for connection events
typedef void (*ble_connect_cb_t)(uint16_t conn_handle, const uint8_t *addr);
typedef void (*ble_disconnect_cb_t)(uint16_t conn_handle, uint8_t reason);
typedef void (*ble_passkey_cb_t)(uint16_t conn_handle, uint32_t passkey);
typedef void (*ble_data_received_cb_t)(uint16_t conn_handle, const uint8_t *data, uint16_t len);

// Connection management
esp_err_t ble_proxy_connect(const uint8_t *addr);
esp_err_t ble_proxy_disconnect(uint16_t conn_handle);
esp_err_t ble_proxy_get_connection_info(ble_connection_t *conn_info);
bool ble_proxy_is_connected(void);

// Security/Pairing
esp_err_t ble_proxy_set_passkey(uint32_t passkey);
esp_err_t ble_proxy_input_passkey(uint16_t conn_handle, uint32_t passkey);
esp_err_t ble_proxy_confirm_passkey(uint16_t conn_handle, bool confirm);

// Callback registration
void ble_proxy_register_connect_cb(ble_connect_cb_t cb);
void ble_proxy_register_disconnect_cb(ble_disconnect_cb_t cb);
void ble_proxy_register_passkey_cb(ble_passkey_cb_t cb);
void ble_proxy_register_data_cb(ble_data_received_cb_t cb);

// Proxy state functions
proxy_state_t ble_proxy_get_state(void);
bool ble_proxy_gatt_ready(void);

// Handle accessor functions (read-only accessors for tcp_proxy.c)
uint16_t ble_proxy_get_rx_handle(void);
uint16_t ble_proxy_get_tx_handle(void);

// Safe send (chunks to MTU-3, returns ESP_OK/ESP_FAIL)
esp_err_t ble_proxy_send_data(const uint8_t *data, uint16_t len);

// Test function
void test_meshtastic_communication(void);

// TCP proxy functions (safe to call multiple times)
void start_tcp_proxy(void);
void stop_tcp_proxy(void);
void tcp_forward_ble_data(uint8_t *data, uint16_t len);

#endif // BLE_PROXY_H