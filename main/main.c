// main.c - Resilient Field Flasher Main Application
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#include "nrf52_hal.h"
#include "wifi_credentials.h"
#include "esp_http_server.h"
#include "web_upload.h"

// Custom modules
#include "swd_core.h"
#include "swd_mem.h"
#include "swd_flash.h"
#include "power_mgmt.h"
#include "flash_safety.h"

static const char *TAG = "FLASHER";

// Event group for system state
static EventGroupHandle_t system_events;
#define WIFI_CONNECTED_BIT  BIT0
#define SWD_CONNECTED_BIT   BIT1
#define FLASH_BUSY_BIT      BIT2
#define ERROR_STATE_BIT     BIT3
#define RECOVERY_MODE_BIT   BIT4

// Global variables
static char device_ip[16] = "Not connected";
static httpd_handle_t web_server = NULL;

// System configuration
typedef struct {
    char wifi_ssid[32];
    char wifi_password[64];
    uint32_t sleep_timeout_sec;
    uint32_t watchdog_timeout_sec;
    bool auto_recovery;
    bool deep_sleep_enabled;
} system_config_t;

static system_config_t sys_config = {
    .wifi_ssid = "",
    .wifi_password = "",
    .sleep_timeout_sec = 300,
    .watchdog_timeout_sec = 0,
    .auto_recovery = true,
    .deep_sleep_enabled = false
};

// Global state
static bool swd_initialized = false;
static esp_timer_handle_t watchdog_timer = NULL;
static esp_timer_handle_t sleep_timer = NULL;
static uint32_t error_count = 0;
static uint32_t recovery_count = 0;

// Function declarations
static void init_system(void);
static void wifi_event_handler(void* arg, esp_event_base_t event_base, 
                              int32_t event_id, void* event_data);
static void system_health_task(void *arg);
static esp_err_t try_swd_connection(void);
static void handle_critical_error(const char *context, esp_err_t error);
static void test_swd_functions(void);
static void test_memory_regions(void);
static esp_err_t start_webserver(void);
static void stop_webserver(void);

// Initialize configuration from wifi_credentials.h
static void init_config(void) {
    strcpy(sys_config.wifi_ssid, WIFI_SSID);
    strcpy(sys_config.wifi_password, WIFI_PASSWORD);
}

// Enhanced web server handler
static esp_err_t root_handler(httpd_req_t *req) {
    const char* html_part1 = 
        "<!DOCTYPE html><html><head><title>RAK4631 Flasher</title>"
        "<style>"
        "body{font-family:Arial;margin:20px;background:#f0f0f0;}"
        "h1{color:#333;}"
        "table{border-collapse:collapse;width:100%;margin:20px 0;}"
        "td,th{border:1px solid #ddd;padding:8px;text-align:left;}"
        "th{background:#4CAF50;color:white;}"
        ".section{border:1px solid #ddd;padding:20px;margin:20px 0;background:white;}"
        ".progress-bar{width:100%;height:30px;background:#eee;border-radius:5px;overflow:hidden;}"
        ".progress-fill{height:100%;background:#4CAF50;transition:width 0.3s;}"
        ".btn{padding:10px 20px;margin:5px;background:#4CAF50;color:white;border:none;cursor:pointer;}"
        ".btn:hover{background:#45a049;}"
        ".btn-danger{background:#f44336;}"
        ".btn-danger:hover{background:#da190b;}"
        ".btn:disabled{background:#ccc;cursor:not-allowed;}"
        ".warning{color:#f44336;font-weight:bold;}"
        "</style></head>"
        "<body>"
        "<h1>RAK4631 Field Flasher</h1>";
    
    httpd_resp_send_chunk(req, html_part1, strlen(html_part1));
    
    // Status section
    char status_html[512];
    EventBits_t bits = xEventGroupGetBits(system_events);
    uint32_t approtect = 0xFFFFFFFF;
    
    if (bits & SWD_CONNECTED_BIT) {
        swd_mem_read32(UICR_APPROTECT, &approtect);
    }
    
    snprintf(status_html, sizeof(status_html),
        "<h2>System Status</h2>"
        "<table>"
        "<tr><th>Parameter</th><th>Value</th></tr>"
        "<tr><td>Device IP</td><td>%s</td></tr>"
        "<tr><td>SWD Status</td><td>%s</td></tr>"
        "<tr><td>Target Device</td><td>nRF52840</td></tr>"
        "<tr><td>APPROTECT</td><td class='%s'>%s</td></tr>"
        "<tr><td>Free Heap</td><td>%lu bytes</td></tr>"
        "</table>",
        device_ip,
        (bits & SWD_CONNECTED_BIT) ? "Connected" : "Not Connected",
        (approtect == 0xFFFFFF5A) ? "" : "warning",
        (approtect == 0xFFFFFF5A) ? "Disabled (Ready for flashing)" : "ENABLED (Flash protected!)",
        esp_get_free_heap_size());
    
    httpd_resp_send_chunk(req, status_html, strlen(status_html));
    
    // Protection management section
    const char* protection_html = 
        "<h2>Protection Management</h2>"
        "<div class='section'>"
        "<p class='warning'>⚠️ Warning: These operations will erase data!</p>"
        "<button class='btn btn-danger' onclick='disableProtection()'>Disable APPROTECT</button>"
        "<button class='btn btn-danger' onclick='eraseAll()'>Full Chip Erase</button>"
        "<button class='btn' onclick='checkStatus()'>Refresh Status</button>"
        "<button class='btn' onclick='checkSWD()'>Check SWD Connection</button>"
        "<div id='protStatus' style='margin-top:10px;'></div>"
        "</div>";
    
    httpd_resp_send_chunk(req, protection_html, strlen(protection_html));
    
    // Upload section
    const char* upload_html = 
        "<h2>Firmware Upload</h2>"
        "<div class='section'>"
        "<select id='fwType'>"
        "<option value='app'>Application (0x26000)</option>"
        "<option value='softdevice'>SoftDevice (0x1000)</option>"
        "<option value='bootloader'>Bootloader (0xF4000)</option>"
        "<option value='full'>Full Image (from hex)</option>"
        "</select>"
        "<input type='file' id='hexFile' accept='.hex'>"
        "<button class='btn' onclick='uploadFirmware()'>Upload & Flash</button>"
        "<div style='margin-top:20px;'>"
        "<div class='progress-bar'>"
        "<div id='uploadProgress' class='progress-fill' style='width:0%;'></div>"
        "</div>"
        "<div style='margin-top:5px;'>"
        "<div class='progress-bar'>"
        "<div id='flashProgress' class='progress-fill' style='width:0%;background:#2196F3;'></div>"
        "</div>"
        "</div>"
        "<div id='status' style='margin-top:10px;'>Ready</div>"
        "</div>"
        "</div>";
    
    httpd_resp_send_chunk(req, upload_html, strlen(upload_html));
    
    // JavaScript
    const char* script = 
        "<script>"
        "let progressTimer=null;"
        "function checkStatus(){"
        "location.reload();"
        "}"
        "function disableProtection(){"
        "if(!confirm('This will erase UICR and disable APPROTECT. Continue?'))return;"
        "document.getElementById('protStatus').innerText='Disabling protection...';"
        "fetch('/disable_protection').then(r=>r.json()).then(data=>{"
        "document.getElementById('protStatus').innerText=data.message;"
        "setTimeout(checkStatus,2000);"
        "});"
        "}"
        "function eraseAll(){"
        "if(!confirm('This will ERASE the entire chip! Continue?'))return;"
        "document.getElementById('protStatus').innerText='Erasing chip...';"
        "fetch('/erase_all').then(r=>r.json()).then(data=>{"
        "document.getElementById('protStatus').innerText=data.message;"
        "});"
        "}"
        "function updateProgress(){"
        "fetch('/progress').then(r=>r.json()).then(data=>{"
        "if(data.in_progress){"
        "document.getElementById('uploadProgress').style.width=data.upload_percent+'%';"
        "document.getElementById('flashProgress').style.width=data.flash_percent+'%';"
        "document.getElementById('status').innerText="
        "'Uploading: '+data.upload_percent+'% | Flashing: '+data.flash_percent+'%';"
        "}else{"
        "clearInterval(progressTimer);"
        "document.getElementById('status').innerText=data.message||'Complete';"
        "document.querySelector('.btn').disabled=false;"
        "}});"
        "}"
        "function uploadFirmware(){"
        "const file=document.getElementById('hexFile').files[0];"
        "const type=document.getElementById('fwType').value;"
        "if(!file){alert('Please select a hex file');return;}"
        "document.querySelector('.btn').disabled=true;"
        "document.getElementById('status').innerText='Starting upload...';"
        "document.getElementById('uploadProgress').style.width='0%';"
        "document.getElementById('flashProgress').style.width='0%';"
        "progressTimer=setInterval(updateProgress,500);"
        "const xhr=new XMLHttpRequest();"
        "xhr.onload=function(){clearInterval(progressTimer);updateProgress();};"
        "xhr.open('POST','/upload?type='+type);"
        "xhr.send(file);"
        "}"
        "function checkSWD(){"
        "document.getElementById('protStatus').innerText='Checking SWD...';"
        "fetch('/check_swd').then(r=>r.json()).then(data=>{"
        "document.getElementById('protStatus').innerText='SWD: '+(data.connected?'Connected':'Disconnected')+' | APPROTECT: '+data.status;"
        "if(data.connected)setTimeout(checkStatus,1000);"
        "});"
        "}"
        "</script>"
        "</body></html>";
    
    httpd_resp_send_chunk(req, script, strlen(script));
    httpd_resp_send_chunk(req, NULL, 0);
    
    return ESP_OK;
}

// Update start_webserver() to register these handlers:
static esp_err_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 12;  // Increased
    config.recv_wait_timeout = 10;
    config.stack_size = 8192;
    
    if (httpd_start(&web_server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_handler,
            .user_ctx = NULL
        };
        
        httpd_uri_t disable_prot_uri = {
            .uri = "/disable_protection",
            .method = HTTP_GET,
            .handler = disable_protection_handler,
            .user_ctx = NULL
        };
        
        httpd_uri_t erase_all_uri = {
            .uri = "/erase_all",
            .method = HTTP_GET,
            .handler = erase_all_handler,
            .user_ctx = NULL
        };
        
        httpd_register_uri_handler(web_server, &root_uri);
        httpd_register_uri_handler(web_server, &disable_prot_uri);
        httpd_register_uri_handler(web_server, &erase_all_uri);
        
        register_upload_handlers(web_server);
        
        ESP_LOGI(TAG, "Web server started successfully");
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to start web server");
    return ESP_FAIL;
}

// Stop web server
static void stop_webserver(void) {
    if (web_server) {
        httpd_stop(web_server);
        web_server = NULL;
    }
}

// Comprehensive memory testing
static void test_memory_regions(void) {
    if (!swd_is_connected()) {
        ESP_LOGW(TAG, "SWD not connected for memory testing");
        return;
    }
    
    ESP_LOGI(TAG, "=== Comprehensive Memory Test ===");
    
    uint32_t data;
    
    // Test Flash regions
    ESP_LOGI(TAG, "--- Flash Memory Test ---");
    uint32_t flash_addrs[] = {
        0x00000000,  // Start of flash (reset vector)
        0x00001000,  // Typical bootloader location
        0x00010000,  // Application start
        0x000FC000,  // Near end of 1MB flash
    };
    
    for (int i = 0; i < 4; i++) {
        if (swd_mem_read32(flash_addrs[i], &data) == ESP_OK) {
            ESP_LOGI(TAG, "Flash[0x%08lX] = 0x%08lX", flash_addrs[i], data);
        } else {
            ESP_LOGE(TAG, "Failed to read Flash[0x%08lX]", flash_addrs[i]);
        }
    }
    
    // Test RAM regions
    ESP_LOGI(TAG, "--- RAM Memory Test ---");
    uint32_t ram_addrs[] = {
        0x20000000,  // Start of RAM
        0x20000100,  // Safe test area
        0x20001000,  // 4KB into RAM
        0x2003FF00,  // Near end of 256KB RAM
    };
    
    for (int i = 0; i < 4; i++) {
        if (swd_mem_read32(ram_addrs[i], &data) == ESP_OK) {
            ESP_LOGI(TAG, "RAM[0x%08lX] = 0x%08lX", ram_addrs[i], data);
            
            // Try write test on safe area only
            if (ram_addrs[i] == 0x20000100) {
                uint32_t test_patterns[] = {0xDEADBEEF, 0x12345678, 0xAAAA5555};
                for (int j = 0; j < 3; j++) {
                    if (swd_mem_write32(ram_addrs[i], test_patterns[j]) == ESP_OK) {
                        uint32_t readback;
                        if (swd_mem_read32(ram_addrs[i], &readback) == ESP_OK) {
                            if (readback == test_patterns[j]) {
                                ESP_LOGI(TAG, "  ✓ Pattern 0x%08lX verified", test_patterns[j]);
                            } else {
                                ESP_LOGE(TAG, "  ✗ Pattern failed: wrote 0x%08lX, read 0x%08lX", 
                                        test_patterns[j], readback);
                            }
                        }
                    }
                }
                // Restore original
                swd_mem_write32(ram_addrs[i], data);
            }
        } else {
            ESP_LOGE(TAG, "Failed to read RAM[0x%08lX]", ram_addrs[i]);
        }
    }
    
    // Test Peripheral regions
    ESP_LOGI(TAG, "--- Peripheral Memory Test ---");
    struct {
        uint32_t addr;
        const char *name;
    } periph_regs[] = {
        {0x40000000, "CLOCK"},
        {0x40001000, "RADIO"},
        {0x40002000, "UARTE0"},
        {0x40003000, "SPIM0/SPIS0/TWIM0/TWIS0"},
        {0x4001E000, "NVMC"},
        {0x40024000, "SPIM2/SPIS2"},
        {0x4002D000, "USBD"},
        {0x50000000, "GPIO P0"},
        {0x50000300, "GPIO P1"},
    };
    
    for (int i = 0; i < 9; i++) {
        if (swd_mem_read32(periph_regs[i].addr, &data) == ESP_OK) {
            ESP_LOGI(TAG, "%s[0x%08lX] = 0x%08lX", 
                    periph_regs[i].name, periph_regs[i].addr, data);
        }
    }
    
    // Read Device ID and info
    ESP_LOGI(TAG, "--- Device Information ---");
    uint32_t deviceid[2];
    if (swd_mem_read32(FICR_DEVICEID0, &deviceid[0]) == ESP_OK &&
        swd_mem_read32(FICR_DEVICEID1, &deviceid[1]) == ESP_OK) {
        ESP_LOGI(TAG, "Device ID: 0x%08lX%08lX", deviceid[1], deviceid[0]);
    }
    
    // Read MAC address
    uint32_t mac[2];
    if (swd_mem_read32(FICR_DEVICEADDR0, &mac[0]) == ESP_OK &&
        swd_mem_read32(FICR_DEVICEADDR1, &mac[1]) == ESP_OK) {
        ESP_LOGI(TAG, "BLE MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                (uint8_t)(mac[1] >> 8), (uint8_t)mac[1],
                (uint8_t)(mac[0] >> 24), (uint8_t)(mac[0] >> 16),
                (uint8_t)(mac[0] >> 8), (uint8_t)mac[0]);
    }
    
    ESP_LOGI(TAG, "=== Memory Test Complete ===");
}

// WiFi initialization - STA mode only
static void init_wifi(void) {
    ESP_LOGI(TAG, "=== Starting WiFi Initialization (STA only) ===");
    ESP_LOGI(TAG, "Connecting to SSID: '%s'", sys_config.wifi_ssid);
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, 
                                              &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, 
                                              &wifi_event_handler, NULL));
    
    wifi_config_t sta_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    strcpy((char*)sta_config.sta.ssid, sys_config.wifi_ssid);
    strcpy((char*)sta_config.sta.password, sys_config.wifi_password);
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialized in STA mode");
}

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
                
            case WIFI_EVENT_STA_DISCONNECTED:
                xEventGroupClearBits(system_events, WIFI_CONNECTED_BIT);
                ESP_LOGI(TAG, "WiFi disconnected, retrying...");
                strcpy(device_ip, "Not connected");
                stop_webserver();
                vTaskDelay(pdMS_TO_TICKS(2000));
                esp_wifi_connect();
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        snprintf(device_ip, sizeof(device_ip), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Got IP: %s", device_ip);
        ESP_LOGI(TAG, "Web interface: http://%s", device_ip);
        xEventGroupSetBits(system_events, WIFI_CONNECTED_BIT);
        
        // Start web server when we get IP
        start_webserver();
    }
}

// Test SWD functions
static void test_swd_functions(void) {
    if (!swd_is_connected()) {
        ESP_LOGW(TAG, "SWD not connected for testing");
        return;
    }
    
    ESP_LOGI(TAG, "=== SWD Function Test ===");
    
    uint32_t data;
    
    // Read device info
    if (swd_mem_read32(FICR_INFO_PART, &data) == ESP_OK) {
        ESP_LOGI(TAG, "Part Number: 0x%08lX (nRF52840)", data);
    }
    
    if (swd_mem_read32(FICR_INFO_RAM, &data) == ESP_OK) {
        ESP_LOGI(TAG, "RAM Size: %lu KB", data);
    }
    
    if (swd_mem_read32(FICR_INFO_FLASH, &data) == ESP_OK) {
        ESP_LOGI(TAG, "Flash Size: %lu KB", data);
    }
    
    // Check protection
    if (swd_mem_read32(UICR_APPROTECT, &data) == ESP_OK) {
        if (data == 0xFFFFFF5A) {
            ESP_LOGI(TAG, "APPROTECT: 0x%08lX (DISABLED - Good!)", data);
        } else {
            ESP_LOGW(TAG, "APPROTECT: 0x%08lX (ENABLED - Flash operations restricted)", data);
        }
    }
    
    ESP_LOGI(TAG, "=== SWD Test Complete ===");
}

// SWD connection with retry logic
static esp_err_t try_swd_connection(void) {
    ESP_LOGI(TAG, "=== Starting SWD Connection Attempt ===");
    
    if (swd_initialized && swd_is_connected()) {
        ESP_LOGI(TAG, "SWD already connected");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Attempting SWD connection...");
    
    if (!swd_initialized) {
        ESP_LOGI(TAG, "Initializing SWD interface...");
        
        swd_config_t swd_cfg = {
            .pin_swclk = 8,
            .pin_swdio = 9,
            .pin_reset = 7,
            .delay_cycles = 2
        };
        
        esp_err_t ret = swd_init(&swd_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SWD init failed: %s (0x%x)", esp_err_to_name(ret), ret);
            return ret;
        }
        ESP_LOGI(TAG, "SWD interface initialized");
        swd_initialized = true;
    }
    
    ESP_LOGI(TAG, "Trying direct connection...");
    esp_err_t ret = swd_connect();
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Direct connect failed, trying reset...");
        ret = swd_reset_target();
        if (ret == ESP_OK) {
            ret = swd_connect();
        }
    }
    
    if (ret == ESP_OK) {
        xEventGroupSetBits(system_events, SWD_CONNECTED_BIT);
        ESP_LOGI(TAG, "✓ SWD connected successfully!");
        
        uint32_t idcode = swd_get_idcode();
        ESP_LOGI(TAG, "Target IDCODE: 0x%08lX", idcode);
        
        ret = swd_flash_init();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Flash init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Flash interface initialized");
        }
        
        test_swd_functions();
        test_memory_regions();  // Run comprehensive memory test on connection
    } else {
        xEventGroupClearBits(system_events, SWD_CONNECTED_BIT);
        ESP_LOGE(TAG, "✗ SWD connection failed with error: 0x%x", ret);
    }
    
    ESP_LOGI(TAG, "=== SWD Connection Attempt Complete ===");
    return ret;
}

// System health monitoring task
static void system_health_task(void *arg) {
    ESP_LOGI(TAG, "System health task started");
    
    system_health_t health;
    
    // Do initial system check
    power_get_health_status(&health);
    ESP_LOGI(TAG, "Initial Health: SWD=%d Flash=%d Net=%d Errors=%lu",
            health.swd_failures, health.flash_failures, 
            health.network_failures, error_count);
    
    size_t free_heap = esp_get_free_heap_size();
    ESP_LOGI(TAG, "Heap: free=%d", free_heap);
    
    // Now just monitor for critical issues
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));  // Check every 5 seconds
        
        // Only check for critical heap issues
        free_heap = esp_get_free_heap_size();
        if (free_heap < 20000) {
            ESP_LOGW(TAG, "Low memory warning: %d bytes", free_heap);
        }
    }
}

// Critical error handler
static void handle_critical_error(const char *context, esp_err_t error) {
    ESP_LOGE(TAG, "Critical error in %s: %s", context, esp_err_to_name(error));
    recovery_count++;
    
    char error_msg[128];
    snprintf(error_msg, sizeof(error_msg), "%s: %s", context, esp_err_to_name(error));
    power_log_error(error_msg);
    
    if (recovery_count > 3) {
        ESP_LOGE(TAG, "Too many recovery attempts");
        xEventGroupSetBits(system_events, RECOVERY_MODE_BIT);
    }
}

// System initialization
static void init_system(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    init_config();
    system_events = xEventGroupCreate();
    
    power_config_t power_cfg = {
        .target_power_gpio = 10,
        .power_on_delay_ms = 100,
        .reset_hold_ms = 50,
        .sleep_duration_sec = sys_config.sleep_timeout_sec,
        .wifi_check_interval_ms = 5000,
        .wifi_timeout_ms = 10000,
        .wake_ssid = sys_config.wifi_ssid,
        .watchdog_timeout_sec = 0,
        .enable_brownout_detect = true,
        .max_retry_count = 3,
        .error_cooldown_ms = 1000
    };
    ESP_ERROR_CHECK(power_mgmt_init(&power_cfg));
    
    wake_reason_t wake_reason = power_get_wake_reason();
    ESP_LOGI(TAG, "Wake reason: %d", wake_reason);
    
    init_wifi();
    
    ESP_LOGI(TAG, "Initializing SWD connection...");
    try_swd_connection();
    
    xTaskCreate(system_health_task, "health", 4096, NULL, 5, NULL);
}

// Main application entry
void app_main(void) {
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "RAK4631 Field Flasher v1.0");
    ESP_LOGI(TAG, "Build: %s %s", __DATE__, __TIME__);
    ESP_LOGI(TAG, "=================================");
    
    init_system();
    
    ESP_LOGI(TAG, "System initialized successfully");
    
    // Print status once
    EventBits_t bits = xEventGroupGetBits(system_events);
    ESP_LOGI(TAG, "Status - SWD:%s WiFi:%s IP:%s", 
            (bits & SWD_CONNECTED_BIT) ? "Connected" : "Disconnected",
            (bits & WIFI_CONNECTED_BIT) ? "Connected" : "Disconnected",
            device_ip);
    
    // Main loop - just keep alive, no constant logging
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));  // 30 seconds
        // Silent unless there's an error
    }
}