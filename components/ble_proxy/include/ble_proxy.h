#ifndef BLE_PROXY_H
#define BLE_PROXY_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

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

#endif // BLE_PROXY_H