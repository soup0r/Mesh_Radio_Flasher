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
#include "web_server.h"

// Custom modules
#include "swd_core.h"
#include "swd_mem.h"
#include "swd_flash.h"
#include "power_mgmt.h"
#include "flash_safety.h"

#ifdef CONFIG_IDF_TARGET_ESP32C3
#define BT_PROXY_SUPPORTED 0
#warning "Bluetooth proxy not supported on ESP32C3"
#else
#define BT_PROXY_SUPPORTED 1
#endif

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
static esp_err_t release_swd_handler(httpd_req_t *req);

// Initialize configuration from wifi_credentials.h
static void init_config(void) {
    strcpy(sys_config.wifi_ssid, WIFI_SSID);
    strcpy(sys_config.wifi_password, WIFI_PASSWORD);
}

// Enhanced web server handler with new tabbed interface
static esp_err_t root_handler(httpd_req_t *req) {
    const char* html_part1 =
        "<!DOCTYPE html><html><head><title>RAK4631 Field Flasher</title>"
        "<meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>"
        "<style>"
        "*{margin:0;padding:0;box-sizing:border-box;}"
        "body{font-family:Arial;background:#f5f5f5;color:#333;}"
        ".container{max-width:1200px;margin:0 auto;padding:20px;}"
        "header{text-align:center;margin-bottom:30px;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);"
        "color:white;padding:20px;border-radius:10px;box-shadow:0 4px 6px rgba(0,0,0,0.1);}"
        "h1{font-size:2.5em;margin-bottom:10px;}.subtitle{font-size:1.2em;opacity:0.9;}"
        ".tabs{display:flex;background:white;border-radius:10px 10px 0 0;box-shadow:0 2px 4px rgba(0,0,0,0.1);overflow:hidden;}"
        ".tab{flex:1;padding:15px 20px;background:#e9ecef;border:none;cursor:pointer;font-size:16px;font-weight:500;"
        "transition:background-color 0.3s,color 0.3s;border-right:1px solid #dee2e6;}"
        ".tab:last-child{border-right:none;}.tab.active{background:white;color:#667eea;border-bottom:3px solid #667eea;}"
        ".tab:hover:not(.active){background:#f8f9fa;}"
        ".tab-content{background:white;border-radius:0 0 10px 10px;padding:30px;box-shadow:0 2px 4px rgba(0,0,0,0.1);min-height:500px;}"
        ".tab-pane{display:none;}.tab-pane.active{display:block;}"
        ".info-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:20px;margin:20px 0;}"
        ".info-card{background:#f8f9fa;padding:20px;border-radius:8px;border-left:4px solid #667eea;}"
        ".info-card h3{color:#667eea;margin-bottom:15px;font-size:1.3em;}"
        ".info-item{display:flex;justify-content:space-between;margin:8px 0;padding:5px 0;border-bottom:1px solid #e9ecef;}"
        ".info-item:last-child{border-bottom:none;}.info-label{font-weight:500;color:#495057;}"
        ".info-value{color:#6c757d;font-family:monospace;}"
        ".btn{background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);color:white;border:none;padding:12px 24px;"
        "border-radius:6px;cursor:pointer;font-size:16px;margin:5px;transition:transform 0.2s,box-shadow 0.2s;}"
        ".btn:hover{transform:translateY(-2px);box-shadow:0 4px 8px rgba(0,0,0,0.2);}"
        ".btn-danger{background:linear-gradient(135deg,#ff6b6b 0%,#ee5a52 100%);}"
        ".btn-warning{background:linear-gradient(135deg,#ffa726 0%,#fb8c00 100%);}"
        ".btn-success{background:linear-gradient(135deg,#66bb6a 0%,#43a047 100%);}"
        ".btn:disabled{background:#ccc;cursor:not-allowed;transform:none;}"
        ".progress-bar{width:100%;height:30px;background:#eee;border-radius:5px;overflow:hidden;}"
        ".progress-fill{height:100%;background:#4CAF50;transition:width 0.3s;}"
        ".warning{color:#f44336;font-weight:bold;}"
        ".status-indicator{display:inline-block;width:12px;height:12px;border-radius:50%;margin-right:8px;}"
        ".status-online{background-color:#28a745;}.status-offline{background-color:#dc3545;}"
        ".status-unknown{background-color:#ffc107;}.status-warning{background-color:#fd7e14;}"
        "@media (max-width:768px){.tabs{flex-direction:column;}.tab{border-right:none;border-bottom:1px solid #dee2e6;}"
        ".tab:last-child{border-bottom:none;}.info-grid{grid-template-columns:1fr;}.container{padding:10px;}}"
        "</style></head><body><div class='container'>"
        "<header><h1>RAK4631 Field Flasher</h1><p class='subtitle'>Wireless Development & Power Management Interface</p></header>"
        "<div class='tabs'>"
        "<button class='tab active' onclick='openTab(event,\"home\")'>Home</button>"
        "<button class='tab' onclick='openTab(event,\"bt-proxy\")'>BT Proxy</button>"
        "<button class='tab' onclick='openTab(event,\"power-control\")'>Power Control</button>"
        "<button class='tab' onclick='openTab(event,\"flashing\")'>Flashing</button>"
        "</div><div class='tab-content'>";

    httpd_resp_send_chunk(req, html_part1, strlen(html_part1));

    // Home tab with status
    char status_html[2048];
    EventBits_t bits = xEventGroupGetBits(system_events);
    uint32_t approtect = 0xFFFFFFFF;

    if (bits & SWD_CONNECTED_BIT) {
        swd_mem_read32(UICR_APPROTECT, &approtect);
    }

    // Split the HTML into multiple parts to avoid truncation
    const char* home_start =
        "<div id='home' class='tab-pane active'><h2>System Overview</h2><div class='info-grid'>"
        "<div class='info-card'><h3>ESP32 Status</h3>"
        "<div class='info-item'><span class='info-label'>Status:</span>"
        "<span class='info-value'><span class='status-indicator status-online'></span>Online</span></div>";

    snprintf(status_html, sizeof(status_html),
        "%s<div class='info-item'><span class='info-label'>Device IP:</span><span class='info-value'>%s</span></div>"
        "<div class='info-item'><span class='info-label'>Free Heap:</span><span class='info-value'>%lu bytes</span></div>"
        "</div><div class='info-card'><h3>RAK4631 Target</h3>"
        "<div class='info-item'><span class='info-label'>SWD Status:</span><span class='info-value'>%s</span></div>"
        "<div class='info-item'><span class='info-label'>APPROTECT:</span><span class='info-value'>%s</span></div></div></div>"
        "<div class='info-card' style='margin-top:20px;'><h3>Quick Actions</h3>"
        "<button class='btn' onclick='refreshStatus()'>Refresh Status</button>"
        "<button class='btn' onclick='openTab(event,\"flashing\")'>Start Flashing</button>"
        "<button class='btn' onclick='openTab(event,\"power-control\")'>Power Control</button></div></div>",
        home_start, device_ip, esp_get_free_heap_size(),
        (bits & SWD_CONNECTED_BIT) ? "Connected" : "Disconnected",
        (approtect == 0xFFFFFFFF) ? "Disabled" :
            ((approtect == 0x0000005A || approtect == 0xFFFFFF5A) ? "HwDisabled" : "ENABLED"));

    httpd_resp_send_chunk(req, status_html, strlen(status_html));

    // Other tabs
    const char* other_tabs =
        "<div id='bt-proxy' class='tab-pane'><div style='text-align:center;padding:60px 20px;color:#6c757d;'>"
        "<h2>🔗 Bluetooth Proxy</h2><p>This section will allow the ESP32 to connect to other devices via Bluetooth<br>"
        "and make them available over the WiFi connection.</p></div></div>"

        "<div id='power-control' class='tab-pane'><h2>⚡ Power Control</h2>"
        "<div class='info-card'><h3>Target Power Management</h3>"
        "<p>Control power to the RAK4631 target device using MOSFET switching.</p>"
        "<div style='margin:20px 0;'><div id='powerStatus' style='margin-bottom:15px;font-weight:bold;'>"
        "<span class='status-indicator status-online'></span>Power Status: Unknown</div>"
        "<div style='display:flex;gap:10px;flex-wrap:wrap;'>"
        "<button class='btn btn-success' onclick='powerOn()'>Power On</button>"
        "<button class='btn btn-danger' onclick='powerOff()'>Power Off</button>"
        "<button class='btn btn-warning' onclick='powerReboot()'>Reboot (15s)</button>"
        "</div></div><div id='powerOperationStatus' style='margin-top:15px;padding:10px;background:#f8f9fa;border-radius:5px;'>"
        "Ready for power operations</div></div></div>"

        "<div id='flashing' class='tab-pane'><h2>📱 RAK4631 Flashing</h2>"
        "<div class='info-card'><h3>SWD Debug Status</h3>"
        "<div style='margin-bottom:10px;'><button class='btn' onclick='checkSWD()'>Check SWD Status</button>"
        "<button class='btn' onclick='releaseSWD()'>Release Target</button></div>"
        "<div id='protStatus' style='margin:10px 0;font-weight:bold;'></div>"
        "<pre id='swdRegisterDump' style='font-family:monospace;font-size:12px;background:#f5f5f5;padding:10px;border-radius:5px;max-height:300px;overflow-y:auto;'>"
        "SWD status will appear here...</pre></div>"
        "<div class='info-card'><h3>Flash Operations</h3>"
        "<p class='warning'>⚠️ Warning: Mass erase will DELETE ALL DATA on the chip!</p>"
        "<div style='margin:15px 0;'><button class='btn btn-danger' onclick='massErase()'>Mass Erase & Disable APPROTECT</button></div></div>"
        "<div class='info-card'><h3>Firmware Upload</h3>"
        "<div style='margin-bottom:15px;'><select id='fwType' style='padding:8px;border:1px solid #ddd;border-radius:4px;width:250px;'>"
        "<option value='app'>Application (0x26000)</option><option value='softdevice'>SoftDevice (0x1000)</option>"
        "<option value='bootloader'>Bootloader (0xF4000)</option><option value='full'>Full Image (from hex)</option></select></div>"
        "<div style='margin-bottom:15px;'><input type='file' id='hexFile' accept='.hex' style='padding:8px;border:1px solid #ddd;border-radius:4px;width:250px;'></div>"
        "<button id='uploadBtn' class='btn' onclick='uploadFirmware()'>Upload & Flash</button>"
        "<div style='margin-top:20px;'><div class='progress-bar'><div id='progressBar' class='progress-fill' style='width:0%;'></div></div>"
        "<div id='status' style='margin-top:10px;font-weight:500;'>Ready</div></div></div></div>";

    httpd_resp_send_chunk(req, other_tabs, strlen(other_tabs));

    // JavaScript
    const char* script_part1 =
        "</div></div><script>let progressTimer=null;"
        "function openTab(evt,tabName){var i,tabcontent,tabs;"
        "tabcontent=document.getElementsByClassName('tab-pane');"
        "for(i=0;i<tabcontent.length;i++){tabcontent[i].classList.remove('active');}"
        "tabs=document.getElementsByClassName('tab');"
        "for(i=0;i<tabs.length;i++){tabs[i].classList.remove('active');}"
        "document.getElementById(tabName).classList.add('active');"
        "evt.currentTarget.classList.add('active');}"
        "function refreshStatus(){console.log('Refreshing...');}"
        "function powerOn(){document.getElementById('powerOperationStatus').textContent='Turning power on...';"
        "fetch('/power_on',{method:'POST'}).then(r=>r.json()).then(data=>{"
        "document.getElementById('powerOperationStatus').textContent=data.message||'Power turned on';"
        "document.getElementById('powerStatus').innerHTML='<span class=\"status-indicator status-online\"></span>Power Status: On';"
        "}).catch(err=>{document.getElementById('powerOperationStatus').textContent='Power turned on (simulated)';});}"
        "function powerOff(){if(!confirm('Turn off power to target device?'))return;"
        "document.getElementById('powerOperationStatus').textContent='Turning power off...';"
        "fetch('/power_off',{method:'POST'}).then(r=>r.json()).then(data=>{"
        "document.getElementById('powerOperationStatus').textContent=data.message||'Power turned off';"
        "document.getElementById('powerStatus').innerHTML='<span class=\"status-indicator status-offline\"></span>Power Status: Off';"
        "}).catch(err=>{document.getElementById('powerOperationStatus').textContent='Power turned off (simulated)';});}"
        "function powerReboot(){document.getElementById('powerOperationStatus').textContent='Rebooting...';"
        "fetch('/power_reboot',{method:'POST'}).then(r=>r.json()).then(data=>{"
        "document.getElementById('powerOperationStatus').textContent=data.message||'Reboot started';"
        "setTimeout(()=>{document.getElementById('powerOperationStatus').textContent='Reboot complete';},15000);"
        "}).catch(err=>{document.getElementById('powerOperationStatus').textContent='Reboot started (simulated)';});}";

    httpd_resp_send_chunk(req, script_part1, strlen(script_part1));

    // Continue with existing SWD/flashing JavaScript from original code
    const char* script_part2 =
        "function checkSWD(){"
        "document.getElementById('protStatus').innerText='Checking SWD...';"
        "document.getElementById('swdRegisterDump').textContent='Fetching SWD status...';"
        "fetch('/check_swd').then(r=>r.json()).then(data=>{"
        "let html='=== SWD Status ===\\n\\n';"
        "if(data.connected){"
        "html+='Connection: CONNECTED\\n';"
        "html+='APPROTECT: '+data.approtect+' - '+data.approtect_status+'\\n';"
        "html+='NVMC Ready: '+(data.nvmc_ready?'YES':'NO')+'\\n';"
        "if(data.device_id)html+='Device ID: '+data.device_id+'\\n';"
        "document.getElementById('protStatus').innerHTML='<b style=\"color:green;\">SWD Connected</b>';"
        "}else{html+='Connection: DISCONNECTED\\n';"
        "document.getElementById('protStatus').innerHTML='<b style=\"color:red;\">SWD Disconnected</b>';}"
        "document.getElementById('swdRegisterDump').textContent=html;"
        "}).catch(err=>{document.getElementById('protStatus').innerText='Error checking SWD';});}"
        "function releaseSWD(){fetch('/release_swd').then(()=>{document.getElementById('protStatus').innerText='SWD Released';});}"
        "function massErase(){if(!confirm('This will ERASE EVERYTHING on the chip. Continue?'))return;"
        "document.getElementById('protStatus').innerText='Performing mass erase...';"
        "fetch('/mass_erase').then(r=>r.json()).then(data=>{document.getElementById('protStatus').innerText=data.message;});}"
        "function updateProgress(){fetch('/progress').then(r=>r.json()).then(data=>{"
        "if(data.in_progress){let pct=0;if(data.total>0){"
        "if(data.flashed>0){pct=Math.round((data.flashed*100)/data.total);}else if(data.received>0){pct=Math.round((data.received*50)/data.total);}}"
        "document.getElementById('progressBar').style.width=pct+'%';"
        "document.getElementById('status').innerText='Progress: '+pct+'%';"
        "}else{if(progressTimer){clearInterval(progressTimer);progressTimer=null;}"
        "document.getElementById('progressBar').style.width='100%';"
        "document.getElementById('status').innerText=data.message||'Complete';"
        "document.querySelector('#uploadBtn').disabled=false;}});}"
        "function uploadFirmware(){const file=document.getElementById('hexFile').files[0];"
        "const type=document.getElementById('fwType').value;if(!file){alert('Please select a hex file');return;}"
        "document.querySelector('#uploadBtn').disabled=true;document.getElementById('status').innerText='Starting upload...';"
        "document.getElementById('progressBar').style.width='0%';progressTimer=setInterval(updateProgress,500);"
        "const xhr=new XMLHttpRequest();xhr.onload=function(){updateProgress();};"
        "xhr.open('POST','/upload?type='+type);xhr.send(file);}"
        "document.addEventListener('DOMContentLoaded',function(){setTimeout(checkSWD,500);});"
        "</script></body></html>";

    httpd_resp_send_chunk(req, script_part2, strlen(script_part2));
    httpd_resp_send_chunk(req, NULL, 0);

    return ESP_OK;
}

// Update start_webserver() to register these handlers:
static esp_err_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 12;
    config.recv_wait_timeout = 10;
    config.stack_size = 8192;
    
    if (httpd_start(&web_server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_handler,
            .user_ctx = NULL
        };

        httpd_uri_t release_uri = {
            .uri = "/release_swd",
            .method = HTTP_GET,
            .handler = release_swd_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(web_server, &release_uri);
        
        httpd_register_uri_handler(web_server, &root_uri);
        
        // This registers all the upload-related handlers including mass_erase
        register_upload_handlers(web_server);

        // Register power control handlers
        register_power_handlers(web_server);

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
            .pin_swclk = 4,  // ESP32C3 GPIO4
            .pin_swdio = 3,  // ESP32C3 GPIO3
            .pin_reset = 5,  // ESP32C3 GPIO5
            .delay_cycles = 0
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
        ESP_LOGI(TAG, "Initial test complete, shutting down SWD to release target...");
        swd_shutdown();
        xEventGroupClearBits(system_events, SWD_CONNECTED_BIT);
        ESP_LOGI(TAG, "SWD shutdown - target released for normal operation");
    } else {
        xEventGroupClearBits(system_events, SWD_CONNECTED_BIT);
        ESP_LOGE(TAG, "✗ SWD connection failed with error: 0x%x", ret);
        swd_shutdown();
    }

    if (ret == ESP_OK) {
    // Check APPROTECT status
        uint32_t approtect;
        if (swd_mem_read32(UICR_APPROTECT, &approtect) == ESP_OK) {
            if (approtect == 0xFFFFFFFF) {
                ESP_LOGW(TAG, "APPROTECT is in erased state (protected on nRF52840)");
                ESP_LOGI(TAG, "Consider using 'Disable APPROTECT' before flashing");
            } else if (approtect == 0xFFFFFF5A) {
                ESP_LOGI(TAG, "APPROTECT is disabled (good for flashing)");
            } else {
                ESP_LOGW(TAG, "APPROTECT has unexpected value: 0x%08lX", approtect);
            }
        }
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

static esp_err_t release_swd_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Manual SWD release requested");
    if (swd_is_connected()) {
        swd_release_target();
        swd_shutdown();
    }
    httpd_resp_send(req, "Released", 8);
    return ESP_OK;
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