// bt_proxy.h - Bluetooth to WiFi Proxy for mesh radios
#ifndef BT_PROXY_H
#define BT_PROXY_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_bt_defs.h"

// Maximum number of simultaneous TCP clients
#define MAX_PROXY_CLIENTS 4

// Data packet for queue
typedef struct {
    uint8_t data[256];
    uint16_t len;
} ble_data_t;

// Proxy statistics
typedef struct {
    bool ble_connected;           // Is BLE connected to mesh radio
    uint8_t device_addr[6];        // BLE address of connected device
    uint32_t tcp_clients;          // Number of connected TCP clients
    uint32_t bytes_proxied;        // Total bytes transferred
    uint32_t reconnect_attempts;   // Number of reconnection attempts
} bt_proxy_stats_t;

// Initialize Bluetooth proxy
// tcp_port: TCP port to listen on for proxy connections
esp_err_t bt_proxy_init(uint16_t tcp_port);

// Scan for and connect to mesh radio
// device_name: Name or name pattern to search for (e.g., "MESH", "RADIO")
esp_err_t bt_proxy_scan_and_connect(const char *device_name);

// Disconnect from current BLE device
esp_err_t bt_proxy_disconnect(void);

// Send command/data to mesh radio via BLE
esp_err_t bt_proxy_send_command(const char *command);

// Get proxy statistics
void bt_proxy_get_stats(bt_proxy_stats_t *stats);

// Deinitialize proxy
esp_err_t bt_proxy_deinit(void);

// Configuration functions
void bt_proxy_set_auto_reconnect(bool enable);
void bt_proxy_set_target_name(const char *name);

#endif // BT_PROXY_H