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
// Removed: #include "services/gap/ble_svc_gap.h" - Not needed for central role
#include "host/ble_store.h"
#include "store/config/ble_store_config.h"
#include <string.h>

// Forward declaration - function exists but not declared in header
void ble_store_config_init(void);

// Helper macro for stringification
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// SYSTEMATIC DEBUGGING: Check what's actually being compiled
#pragma message "=== SECURITY MANAGER COMPILATION CHECK ==="

#ifdef MYNEWT_VAL_BLE_SM
#pragma message "✅ MYNEWT_VAL_BLE_SM is defined as: " TOSTRING(MYNEWT_VAL_BLE_SM)
#else
#pragma message "❌ MYNEWT_VAL_BLE_SM is NOT DEFINED"
#endif

#ifdef MYNEWT_VAL_BLE_SM_LEGACY
#pragma message "✅ MYNEWT_VAL_BLE_SM_LEGACY is defined as: " TOSTRING(MYNEWT_VAL_BLE_SM_LEGACY)
#else
#pragma message "❌ MYNEWT_VAL_BLE_SM_LEGACY is NOT DEFINED"
#endif

#ifdef MYNEWT_VAL_BLE_SM_SC
#pragma message "✅ MYNEWT_VAL_BLE_SM_SC is defined as: " TOSTRING(MYNEWT_VAL_BLE_SM_SC)
#else
#pragma message "❌ MYNEWT_VAL_BLE_SM_SC is NOT DEFINED"
#endif

#ifdef MYNEWT_VAL_BLE_STORE_CONFIG
#pragma message "✅ MYNEWT_VAL_BLE_STORE_CONFIG is defined as: " TOSTRING(MYNEWT_VAL_BLE_STORE_CONFIG)
#else
#pragma message "❌ MYNEWT_VAL_BLE_STORE_CONFIG is NOT DEFINED"
#endif

// Check if Security Manager functions are available at compile time
#ifdef BLE_SM_IOACT_INPUT
#pragma message "✅ BLE_SM_IOACT_INPUT is available"
#else
#pragma message "❌ BLE_SM_IOACT_INPUT is NOT available"
#endif

#pragma message "=== END SECURITY MANAGER COMPILATION CHECK ==="

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

    // RUNTIME DEBUGGING: Check actual MYNEWT_VAL values
    ESP_LOGI(TAG, "=== RUNTIME SECURITY MANAGER CHECK ===");

    #ifdef MYNEWT_VAL_BLE_SM
        ESP_LOGI(TAG, "✅ MYNEWT_VAL_BLE_SM runtime value: %d", MYNEWT_VAL_BLE_SM);
    #else
        ESP_LOGE(TAG, "❌ MYNEWT_VAL_BLE_SM not defined at runtime");
    #endif

    #ifdef MYNEWT_VAL_BLE_SM_LEGACY
        ESP_LOGI(TAG, "✅ MYNEWT_VAL_BLE_SM_LEGACY runtime value: %d", MYNEWT_VAL_BLE_SM_LEGACY);
    #else
        ESP_LOGE(TAG, "❌ MYNEWT_VAL_BLE_SM_LEGACY not defined at runtime");
    #endif

    #ifdef MYNEWT_VAL_BLE_SM_SC
        ESP_LOGI(TAG, "✅ MYNEWT_VAL_BLE_SM_SC runtime value: %d", MYNEWT_VAL_BLE_SM_SC);
    #else
        ESP_LOGE(TAG, "❌ MYNEWT_VAL_BLE_SM_SC not defined at runtime");
    #endif

    #ifdef MYNEWT_VAL_BLE_STORE_CONFIG
        ESP_LOGI(TAG, "✅ MYNEWT_VAL_BLE_STORE_CONFIG runtime value: %d", MYNEWT_VAL_BLE_STORE_CONFIG);
    #else
        ESP_LOGE(TAG, "❌ MYNEWT_VAL_BLE_STORE_CONFIG not defined at runtime");
    #endif

    // Check if Security Manager functions are available
    ESP_LOGI(TAG, "=== SECURITY MANAGER FUNCTION CHECK ===");
    #ifdef BLE_SM_IOACT_INPUT
        ESP_LOGI(TAG, "✅ BLE_SM_IOACT_INPUT available: %d", BLE_SM_IOACT_INPUT);
    #else
        ESP_LOGE(TAG, "❌ BLE_SM_IOACT_INPUT not available");
    #endif

    #ifdef BLE_SM_IOACT_OOB
        ESP_LOGI(TAG, "✅ BLE_SM_IOACT_OOB available: %d", BLE_SM_IOACT_OOB);
    #else
        ESP_LOGE(TAG, "❌ BLE_SM_IOACT_OOB not available");
    #endif

    ESP_LOGI(TAG, "=== END RUNTIME SM CHECK ===");

    // CONFIGURATION VERIFICATION: Check ESP-IDF config matches sdkconfig.defaults
    ESP_LOGI(TAG, "=== NIMBLE CONFIGURATION VERIFICATION ===");

    #ifdef CONFIG_BT_NIMBLE_SECURITY_ENABLE
        ESP_LOGI(TAG, "✅ CONFIG_BT_NIMBLE_SECURITY_ENABLE: enabled");
    #else
        ESP_LOGE(TAG, "❌ CONFIG_BT_NIMBLE_SECURITY_ENABLE: not enabled");
    #endif

    #ifdef CONFIG_BT_NIMBLE_SM_LEGACY
        ESP_LOGI(TAG, "✅ CONFIG_BT_NIMBLE_SM_LEGACY: enabled");
    #else
        ESP_LOGE(TAG, "❌ CONFIG_BT_NIMBLE_SM_LEGACY: not enabled");
    #endif

    #ifdef CONFIG_BT_NIMBLE_SM_SC
        ESP_LOGI(TAG, "✅ CONFIG_BT_NIMBLE_SM_SC: enabled");
    #else
        ESP_LOGE(TAG, "❌ CONFIG_BT_NIMBLE_SM_SC: not enabled");
    #endif

    ESP_LOGI(TAG, "CONFIG_BT_NIMBLE_MAX_BONDS: %d", CONFIG_BT_NIMBLE_MAX_BONDS);
    ESP_LOGI(TAG, "CONFIG_BT_NIMBLE_MAX_CONNECTIONS: %d", CONFIG_BT_NIMBLE_MAX_CONNECTIONS);

    #ifdef CONFIG_BT_NIMBLE_NVS_PERSIST
        ESP_LOGI(TAG, "✅ CONFIG_BT_NIMBLE_NVS_PERSIST: enabled");
    #else
        ESP_LOGE(TAG, "❌ CONFIG_BT_NIMBLE_NVS_PERSIST: not enabled");
    #endif

    ESP_LOGI(TAG, "=== END NIMBLE CONFIG VERIFICATION ===");

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
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;  // Add this

    // CRITICAL: For Meshtastic compatibility based on NimBLE docs
    // Meshtastic expects KEYBOARD_ONLY or DISP_YES_NO for fixed PIN
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_KEYBOARD_ONLY;  // We input PIN
    ble_hs_cfg.sm_bonding = 1;                            // Enable bonding
    ble_hs_cfg.sm_mitm = 1;                              // MITM required for passkey
    ble_hs_cfg.sm_sc = 1;                                // Secure connections
    ble_hs_cfg.sm_sc_only = 0;                           // Allow legacy pairing

    // Key distribution as per NimBLE docs
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ESP_LOGI(TAG, "Security: IO_CAP=KEYBOARD_ONLY, MITM=1, SC=1, Bonding=1");

    // REMOVED GAP service initialization - NOT NEEDED FOR CENTRAL ROLE
    // GAP service (ble_svc_gap_init) is only for peripheral devices that advertise
    // We still have FULL GAP functionality for:
    // - Scanning (ble_gap_disc)
    // - Connecting (ble_gap_connect)
    // - Security (ble_gap_security_initiate)
    // All of these work WITHOUT the GAP Service

    // CRITICAL: Initialize bond storage BEFORE callbacks
    // This was missing and is essential for pairing!
    ESP_LOGI(TAG, "Initializing bond storage...");
    ble_store_config_init();  // THIS IS CRITICAL!
    ESP_LOGI(TAG, "Bond storage initialized");

    // Set storage callbacks (these use the storage initialized above)
    ble_hs_cfg.store_read_cb = ble_store_config_read;
    ble_hs_cfg.store_write_cb = ble_store_config_write;
    ble_hs_cfg.store_delete_cb = ble_store_config_delete;
    ESP_LOGI(TAG, "Bond storage callbacks configured");

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
    ESP_LOGI(TAG, "✅ BLE proxy initialized successfully with ccache");
    ESP_LOGI(TAG, "Free heap after BLE init: %d bytes", esp_get_free_heap_size());

    return ESP_OK;
}