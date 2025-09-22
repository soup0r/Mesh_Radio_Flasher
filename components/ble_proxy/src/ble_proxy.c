#include "ble_proxy.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "host/ble_store.h"
#include <string.h>

static const char *TAG = "BLE_PROXY";

// Device tracking
static ble_device_info_t discovered_devices[BLE_MAX_DEVICES];
static uint16_t device_count = 0;
static bool is_scanning = false;
static bool ble_initialized = false;

// Forward declarations
static void ble_app_on_sync(void);
static void ble_app_on_reset(int reason);
static void nimble_host_task(void *param);

// Helper to find device by address
static int find_device_by_addr(const uint8_t *addr) {
    for (int i = 0; i < device_count; i++) {
        if (memcmp(discovered_devices[i].addr, addr, 6) == 0) {
            return i;
        }
    }
    return -1;
}

// BLE GAP event handler
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    struct ble_hs_adv_fields fields;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        // Parse advertisement data
        ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);

        // Check if we already have this device
        int idx = find_device_by_addr(event->disc.addr.val);
        if (idx < 0 && device_count < BLE_MAX_DEVICES) {
            // New device
            idx = device_count++;
            memcpy(discovered_devices[idx].addr, event->disc.addr.val, 6);
            discovered_devices[idx].has_name = false;
        }

        if (idx >= 0) {
            // Update device info
            discovered_devices[idx].rssi = event->disc.rssi;
            discovered_devices[idx].last_seen = esp_timer_get_time() / 1000000; // seconds

            // Check for device name
            if (fields.name != NULL && fields.name_len > 0) {
                size_t name_len = fields.name_len;
                if (name_len > sizeof(discovered_devices[idx].name) - 1) {
                    name_len = sizeof(discovered_devices[idx].name) - 1;
                }
                memcpy(discovered_devices[idx].name, fields.name, name_len);
                discovered_devices[idx].name[name_len] = '\0';
                discovered_devices[idx].has_name = true;

                // Log Meshtastic devices specially
                if (strstr(discovered_devices[idx].name, "Meshtastic") != NULL) {
                    ESP_LOGI(TAG, "📱 Found Meshtastic device: %s, RSSI: %d",
                            discovered_devices[idx].name, discovered_devices[idx].rssi);
                }
            }

            ESP_LOGD(TAG, "BLE Device: %02X:%02X:%02X:%02X:%02X:%02X, RSSI: %d, Name: %s",
                    discovered_devices[idx].addr[5], discovered_devices[idx].addr[4],
                    discovered_devices[idx].addr[3], discovered_devices[idx].addr[2],
                    discovered_devices[idx].addr[1], discovered_devices[idx].addr[0],
                    discovered_devices[idx].rssi,
                    discovered_devices[idx].has_name ? discovered_devices[idx].name : "N/A");
        }
        break;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(TAG, "BLE scan complete. Found %d devices", device_count);
        is_scanning = false;
        break;

    default:
        break;
    }

    return 0;
}

// Start BLE scan
esp_err_t ble_proxy_start_scan(uint32_t duration_sec) {
    if (!ble_initialized) {
        ESP_LOGE(TAG, "BLE not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (is_scanning) {
        ESP_LOGW(TAG, "Scan already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting BLE scan for %lu seconds", duration_sec);

    // Configure scan parameters
    struct ble_gap_disc_params disc_params = {
        .filter_duplicates = 0,  // Report duplicates to track RSSI changes
        .passive = 0,            // Active scan to get device names
        .itvl = 0,               // Use default interval
        .window = 0,             // Use default window
        .filter_policy = 0,      // No whitelist
        .limited = 0,            // Not limited discovery
    };

    // Start discovery
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, duration_sec * 1000,
                          &disc_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start scan: %d", rc);
        return ESP_FAIL;
    }

    is_scanning = true;
    return ESP_OK;
}

// Stop BLE scan
esp_err_t ble_proxy_stop_scan(void) {
    if (!is_scanning) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping BLE scan");
    int rc = ble_gap_disc_cancel();
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "Failed to stop scan: %d", rc);
        return ESP_FAIL;
    }

    is_scanning = false;
    return ESP_OK;
}

// Check if scanning
bool ble_proxy_is_scanning(void) {
    return is_scanning;
}

// Get discovered devices
uint16_t ble_proxy_get_devices(ble_device_info_t *devices, uint16_t max_devices) {
    uint16_t count = device_count < max_devices ? device_count : max_devices;
    if (devices != NULL && count > 0) {
        memcpy(devices, discovered_devices, count * sizeof(ble_device_info_t));
    }
    return count;
}

// Clear device list
void ble_proxy_clear_devices(void) {
    device_count = 0;
    memset(discovered_devices, 0, sizeof(discovered_devices));
}

// Get device count
uint16_t ble_proxy_get_device_count(void) {
    return device_count;
}

// NimBLE host task
static void nimble_host_task(void *param) {
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// BLE stack sync callback - simplified for ESP32-C3
static void ble_app_on_sync(void) {
    ESP_LOGI(TAG, "BLE host synced");
    ESP_LOGI(TAG, "BLE proxy ready");
}

// BLE stack reset callback
static void ble_app_on_reset(int reason) {
    ESP_LOGE(TAG, "BLE host reset: reason=%d", reason);
}

// Initialize BLE
esp_err_t ble_proxy_init(void) {
    if (ble_initialized) {
        ESP_LOGW(TAG, "BLE already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing BLE proxy...");
    esp_err_t ret;

    // Check if BLE is enabled in menuconfig
    #ifndef CONFIG_BT_ENABLED
        ESP_LOGE(TAG, "CONFIG_BT_ENABLED is not set!");
        return ESP_FAIL;
    #endif

    #ifndef CONFIG_BT_NIMBLE_ENABLED
        ESP_LOGE(TAG, "CONFIG_BT_NIMBLE_ENABLED is not set!");
        return ESP_FAIL;
    #endif

    ESP_LOGI(TAG, "Free heap before BLE init: %d bytes", esp_get_free_heap_size());

    // CRITICAL: For ESP32-C3, we must initialize in this exact order:
    // 1. NimBLE port init (which initializes the controller internally)
    // 2. Configure host
    // 3. Initialize services
    // 4. Start the task

    ESP_LOGI(TAG, "Initializing NimBLE port (this will init controller)...");
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", ret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "NimBLE port initialized");

    // Configure host callbacks
    ESP_LOGI(TAG, "Configuring NimBLE host...");
    ble_hs_cfg.reset_cb = ble_app_on_reset;
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    // Remove store_status_cb as it might cause issues
    ESP_LOGI(TAG, "NimBLE host configured");

    // Initialize GAP service
    ESP_LOGI(TAG, "Initializing GAP service...");
    ble_svc_gap_init();
    ESP_LOGI(TAG, "GAP service initialized");

    // Initialize device tracking
    ESP_LOGI(TAG, "Initializing device tracking...");
    device_count = 0;
    memset(discovered_devices, 0, sizeof(discovered_devices));
    ESP_LOGI(TAG, "Device tracking initialized");

    // Start NimBLE host task
    ESP_LOGI(TAG, "Starting NimBLE host task...");
    nimble_port_freertos_init(nimble_host_task);
    ESP_LOGI(TAG, "NimBLE host task started");

    // Give the host time to start
    vTaskDelay(pdMS_TO_TICKS(200));

    ble_initialized = true;
    ESP_LOGI(TAG, "✅ BLE proxy initialized successfully");
    ESP_LOGI(TAG, "Free heap after BLE init: %d bytes", esp_get_free_heap_size());

    return ESP_OK;
}