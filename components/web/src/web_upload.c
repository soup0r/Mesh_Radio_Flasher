#include "web_upload.h"
#include "esp_log.h"
#include "hex_parser.h"
#include "swd_flash.h"
#include "swd_mem.h"
#include "swd_core.h"
#include "nrf52_hal.h"
#include <string.h>

static const char *TAG = "WEB_UPLOAD";

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#define PAGE_BUFFER_SIZE (16 * 1024)
#define NRF52_PAGE_SIZE 4096

// Type definitions
typedef struct {
    bool in_progress;
    uint32_t total_bytes;
    uint32_t received_bytes;
    uint32_t flashed_bytes;
    uint32_t start_addr;
    uint32_t current_addr;
    hex_stream_parser_t *parser;
    uint8_t *page_buffer;
    uint32_t buffer_start_addr;
    uint32_t buffer_data_len;
    char status_msg[128];
    bool error;
} upload_context_t;

static upload_context_t *g_upload_ctx = NULL;

// Helper function to ensure SWD is ready
static esp_err_t ensure_swd_ready(void) {
    if (!swd_is_initialized()) {
        ESP_LOGI(TAG, "Reinitializing SWD for operation...");
        
        swd_config_t swd_cfg = {
            .pin_swclk = 8,
            .pin_swdio = 9,
            .pin_reset = 7,
            .delay_cycles = 0
        };
        
        esp_err_t ret = swd_init(&swd_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to reinitialize SWD");
            return ret;
        }
    } else {
        swd_reinit();
    }
    
    if (!swd_is_connected()) {
        ESP_LOGI(TAG, "Connecting to target...");
        esp_err_t ret = swd_connect();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to connect to target");
            return ret;
        }
        
        ret = swd_flash_init();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Flash init failed: %s", esp_err_to_name(ret));
        }
    }
    
    return ESP_OK;
}

// Helper function for SWD reconnection
static esp_err_t check_and_reconnect_swd(void) {
    if (swd_is_connected()) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Attempting SWD reconnection...");
    
    // Try to reconnect
    esp_err_t ret = swd_connect();
    if (ret != ESP_OK) {
        // Try with reset
        ret = swd_reset_target();
        if (ret == ESP_OK) {
            ret = swd_connect();
        }
    }
    
    if (ret == ESP_OK) {
        ret = swd_flash_init();
    }
    
    return ret;
}

// Flush buffer to flash
static esp_err_t flush_buffer(upload_context_t *ctx) {
    if (ctx->buffer_data_len == 0) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Flushing buffer: addr=0x%08lX, len=%lu", 
             ctx->buffer_start_addr, ctx->buffer_data_len);
    
    // Erase pages
    uint32_t start_page = ctx->buffer_start_addr & ~(NRF52_PAGE_SIZE - 1);
    uint32_t end_addr = ctx->buffer_start_addr + ctx->buffer_data_len - 1;
    uint32_t end_page = end_addr & ~(NRF52_PAGE_SIZE - 1);
    
    for (uint32_t page = start_page; page <= end_page; page += NRF52_PAGE_SIZE) {
        ESP_LOGI(TAG, "Erasing page 0x%08lX", page);
        esp_err_t ret = swd_flash_erase_page(page);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase page 0x%08lX", page);
            return ret;
        }
    }
    
    // Write data
    esp_err_t ret = swd_flash_write_buffer(ctx->buffer_start_addr, 
                                           ctx->page_buffer, 
                                           ctx->buffer_data_len, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write buffer");
        return ret;
    }
    
    ctx->flashed_bytes += ctx->buffer_data_len;
    
    // Clear buffer
    memset(ctx->page_buffer, 0xFF, PAGE_BUFFER_SIZE);
    ctx->buffer_data_len = 0;
    
    return ESP_OK;
}

// Hex record callback
static void hex_flash_callback(hex_record_t *record, uint32_t abs_addr, void *ctx) {
    upload_context_t *uctx = (upload_context_t*)ctx;
    
    switch (record->type) {
        case HEX_TYPE_DATA: {
            // Check if this data fits in current buffer
            uint32_t offset_in_buffer = abs_addr - uctx->buffer_start_addr;
            
            if (uctx->buffer_data_len == 0) {
                // First data - initialize buffer
                uctx->buffer_start_addr = abs_addr;
                offset_in_buffer = 0;
            } else if (abs_addr < uctx->buffer_start_addr || 
                      offset_in_buffer + record->byte_count > PAGE_BUFFER_SIZE) {
                // Data doesn't fit - flush current buffer
                flush_buffer(uctx);
                uctx->buffer_start_addr = abs_addr;
                offset_in_buffer = 0;
            }
            
            // Copy data to buffer
            memcpy(uctx->page_buffer + offset_in_buffer, record->data, record->byte_count);
            
            // Update buffer length
            uint32_t new_end = offset_in_buffer + record->byte_count;
            if (new_end > uctx->buffer_data_len) {
                uctx->buffer_data_len = new_end;
            }
            
            uctx->current_addr = abs_addr + record->byte_count;
            break;
        }
        
        case HEX_TYPE_EOF:
            // Flush any remaining data
            flush_buffer(uctx);
            ESP_LOGI(TAG, "Upload complete: %lu bytes flashed", uctx->flashed_bytes);
            snprintf(uctx->status_msg, sizeof(uctx->status_msg),
                "Success: Flashed %lu bytes", uctx->flashed_bytes);
    
            // ADD THIS:
            ESP_LOGI(TAG, "Flashing complete, releasing target for normal boot...");
            swd_release_target();
            swd_shutdown();
            ESP_LOGI(TAG, "Target released - should now boot normally");
            break;
            
        case HEX_TYPE_EXT_LIN_ADDR:
            // Flush buffer before address change
            if (uctx->buffer_data_len > 0) {
                flush_buffer(uctx);
            }
            break;
    }
}

// Dump NRF52 registers for diagnostics
static void dump_nrf52_registers(char *buffer, size_t max_len) {
    uint32_t val;
    size_t offset = 0;
    
    ESP_LOGI(TAG, "=== NRF52 Register Dump ===");
    
    // Helper macro for safe string append
    #define APPEND_REG(name, addr) do { \
        if (swd_mem_read32(addr, &val) == ESP_OK) { \
            ESP_LOGI(TAG, "%s: 0x%08lX", name, val); \
            offset += snprintf(buffer + offset, max_len - offset, \
                "\"%s\":\"0x%08lX\",", name, val); \
        } else { \
            ESP_LOGE(TAG, "%s: READ FAILED", name); \
            offset += snprintf(buffer + offset, max_len - offset, \
                "\"%s\":\"ERROR\",", name); \
        } \
    } while(0)
    
    // NVMC Registers
    APPEND_REG("NVMC_READY", NVMC_READY);
    APPEND_REG("NVMC_READYNEXT", NVMC_READYNEXT);
    APPEND_REG("NVMC_CONFIG", NVMC_CONFIG);
    
    // UICR Registers
    APPEND_REG("UICR_APPROTECT", UICR_APPROTECT);
    APPEND_REG("UICR_BOOTLOADERADDR", UICR_BOOTLOADERADDR);
    APPEND_REG("UICR_NRFFW0", UICR_NRFFW0);
    APPEND_REG("UICR_NRFFW1", UICR_NRFFW1);
    
    // FICR Registers (Device Info)
    APPEND_REG("FICR_CODEPAGESIZE", FICR_CODEPAGESIZE);
    APPEND_REG("FICR_CODESIZE", FICR_CODESIZE);
    APPEND_REG("FICR_DEVICEID0", FICR_DEVICEID0);
    APPEND_REG("FICR_DEVICEID1", FICR_DEVICEID1);
    APPEND_REG("FICR_INFO_PART", FICR_INFO_PART);
    APPEND_REG("FICR_INFO_VARIANT", FICR_INFO_VARIANT);
    APPEND_REG("FICR_INFO_RAM", FICR_INFO_RAM);
    APPEND_REG("FICR_INFO_FLASH", FICR_INFO_FLASH);
    
    // Debug registers
    APPEND_REG("DHCSR", DHCSR_ADDR);
    APPEND_REG("DEMCR", DEMCR_ADDR);
    
    // Sample flash locations
    APPEND_REG("Flash[0x0000]", 0x00000000);
    APPEND_REG("Flash[0x1000]", 0x00001000);
    
    #undef APPEND_REG
    
    ESP_LOGI(TAG, "=== Register Dump Complete ===");
}

// Check SWD connection handler
esp_err_t check_swd_handler(httpd_req_t *req) {
    char resp[2048];
    ESP_LOGI(TAG, "=== SWD Status Check Requested ===");
    
    // Check and try to reconnect if needed
    esp_err_t ret = ensure_swd_ready();
    bool connected = (ret == ESP_OK);
    
    if (connected) {
        ESP_LOGI(TAG, "SWD Connected - Reading registers...");
        ESP_LOGI(TAG, "=== NRF52 Register Dump ===");
        
        // Read all important registers
        uint32_t nvmc_ready = 0, nvmc_readynext = 0, nvmc_config = 0;
        uint32_t approtect = 0, bootloader_addr = 0, nrffw0 = 0, nrffw1 = 0;
        uint32_t codepagesize = 0, codesize = 0, deviceid0 = 0, deviceid1 = 0;
        uint32_t info_part = 0, info_variant = 0, info_ram = 0, info_flash = 0;
        uint32_t dhcsr = 0, demcr = 0;
        uint32_t flash_0x0 = 0, flash_0x1000 = 0, flash_0xF4000 = 0;
        
        // NVMC registers
        swd_mem_read32(NVMC_READY, &nvmc_ready);
        ESP_LOGI(TAG, "NVMC_READY: 0x%08lX", nvmc_ready);
        
        swd_mem_read32(NVMC_READYNEXT, &nvmc_readynext);
        ESP_LOGI(TAG, "NVMC_READYNEXT: 0x%08lX", nvmc_readynext);
        
        swd_mem_read32(NVMC_CONFIG, &nvmc_config);
        ESP_LOGI(TAG, "NVMC_CONFIG: 0x%08lX", nvmc_config);
        
        // UICR registers
        swd_mem_read32(UICR_APPROTECT, &approtect);
        ESP_LOGI(TAG, "UICR_APPROTECT: 0x%08lX", approtect);
        
        swd_mem_read32(UICR_BOOTLOADERADDR, &bootloader_addr);
        ESP_LOGI(TAG, "UICR_BOOTLOADERADDR: 0x%08lX", bootloader_addr);
        
        swd_mem_read32(UICR_NRFFW0, &nrffw0);
        ESP_LOGI(TAG, "UICR_NRFFW0: 0x%08lX", nrffw0);
        
        swd_mem_read32(UICR_NRFFW1, &nrffw1);
        ESP_LOGI(TAG, "UICR_NRFFW1: 0x%08lX", nrffw1);
        
        // FICR registers
        swd_mem_read32(FICR_CODEPAGESIZE, &codepagesize);
        ESP_LOGI(TAG, "FICR_CODEPAGESIZE: 0x%08lX", codepagesize);
        
        swd_mem_read32(FICR_CODESIZE, &codesize);
        ESP_LOGI(TAG, "FICR_CODESIZE: 0x%08lX", codesize);
        
        swd_mem_read32(FICR_DEVICEID0, &deviceid0);
        ESP_LOGI(TAG, "FICR_DEVICEID0: 0x%08lX", deviceid0);
        
        swd_mem_read32(FICR_DEVICEID1, &deviceid1);
        ESP_LOGI(TAG, "FICR_DEVICEID1: 0x%08lX", deviceid1);
        
        swd_mem_read32(FICR_INFO_PART, &info_part);
        ESP_LOGI(TAG, "FICR_INFO_PART: 0x%08lX", info_part);
        
        swd_mem_read32(FICR_INFO_VARIANT, &info_variant);
        ESP_LOGI(TAG, "FICR_INFO_VARIANT: 0x%08lX", info_variant);
        
        swd_mem_read32(FICR_INFO_RAM, &info_ram);
        ESP_LOGI(TAG, "FICR_INFO_RAM: 0x%08lX", info_ram);
        
        swd_mem_read32(FICR_INFO_FLASH, &info_flash);
        ESP_LOGI(TAG, "FICR_INFO_FLASH: 0x%08lX", info_flash);
        
        // Debug registers
        swd_mem_read32(DHCSR_ADDR, &dhcsr);
        ESP_LOGI(TAG, "DHCSR: 0x%08lX", dhcsr);
        
        swd_mem_read32(DEMCR_ADDR, &demcr);
        ESP_LOGI(TAG, "DEMCR: 0x%08lX", demcr);
        
        // Flash content samples
        swd_mem_read32(0x00000000, &flash_0x0);
        ESP_LOGI(TAG, "Flash[0x0000]: 0x%08lX", flash_0x0);
        
        swd_mem_read32(0x00001000, &flash_0x1000);
        ESP_LOGI(TAG, "Flash[0x1000]: 0x%08lX", flash_0x1000);
        
        swd_mem_read32(0x000F4000, &flash_0xF4000);
        ESP_LOGI(TAG, "Flash[0xF4000]: 0x%08lX", flash_0xF4000);
        
        ESP_LOGI(TAG, "=== Register Dump Complete ===");
        
        // Determine APPROTECT status string
        const char *approtect_status;
        if (approtect == 0xFFFFFF5A) {
            approtect_status = "HwDisabled (Ready for flashing)";
        } else if (approtect == 0xFFFFFFFF) {
            approtect_status = "Erased (Protected on nRF52840!)";
        } else if (approtect == 0xFFFFFF00) {
            approtect_status = "Enabled (Fully protected)";
        } else {
            approtect_status = "Unknown/Custom";
        }
        
        // Determine NVMC state
        const char *nvmc_state;
        if ((nvmc_config & 0x3) == 0x00) {
            nvmc_state = "Read-only";
        } else if ((nvmc_config & 0x3) == 0x01) {
            nvmc_state = "Write enabled";
        } else if ((nvmc_config & 0x3) == 0x02) {
            nvmc_state = "Erase enabled";
        } else {
            nvmc_state = "Unknown";
        }
        
        // Check if core is halted
        bool core_halted = (dhcsr & DHCSR_S_HALT) != 0;
        
        // Build JSON response with all register data
        snprintf(resp, sizeof(resp),
            "{"
            "\"connected\":true,"
            "\"status\":\"Connected\","
            "\"approtect\":\"0x%08lX\","
            "\"approtect_status\":\"%s\","
            "\"nvmc_ready\":%s,"
            "\"nvmc_state\":\"%s\","
            "\"core_halted\":%s,"
            "\"bootloader_addr\":\"0x%08lX\","
            "\"device_id\":\"0x%08lX%08lX\","
            "\"flash_size\":%lu,"
            "\"ram_size\":%lu,"
            "\"registers\":{"
                "\"nvmc_ready\":\"0x%08lX\","
                "\"nvmc_config\":\"0x%08lX\","
                "\"approtect\":\"0x%08lX\","
                "\"bootloader_addr\":\"0x%08lX\","
                "\"dhcsr\":\"0x%08lX\","
                "\"flash_0x0\":\"0x%08lX\","
                "\"flash_0x1000\":\"0x%08lX\","
                "\"flash_0xF4000\":\"0x%08lX\""
            "}"
            "}",
            approtect,
            approtect_status,
            (nvmc_ready & 0x1) ? "true" : "false",
            nvmc_state,
            core_halted ? "true" : "false",
            bootloader_addr,
            deviceid1, deviceid0,
            info_flash * 1024UL,  // Convert from KB to bytes
            info_ram * 1024UL,     // Convert from KB to bytes
            nvmc_ready,
            nvmc_config,
            approtect,
            bootloader_addr,
            dhcsr,
            flash_0x0,
            flash_0x1000,
            flash_0xF4000
        );
        
    } else {
        ESP_LOGE(TAG, "SWD Not Connected");
        snprintf(resp, sizeof(resp),
            "{\"connected\":false,\"status\":\"Disconnected\",\"error\":\"%s\"}",
            esp_err_to_name(ret));
    }
    
    if (connected) {
        ESP_LOGI(TAG, "Status check complete, releasing SWD...");
        swd_shutdown();
    }

    ESP_LOGI(TAG, "=== SWD Status Check Complete ===");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

// Mass erase handler
esp_err_t mass_erase_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "=== Mass Erase Request from Web Interface ===");
    
    char resp[256];
    
    // First check SWD connection
    esp_err_t ret = ensure_swd_ready();

    // Perform mass erase (which also handles APPROTECT)
    ret = swd_flash_disable_approtect();
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Mass erase successful, APPROTECT disabled");
        strcpy(resp, "{\"success\":true,\"message\":\"Mass erase complete, APPROTECT disabled\"}");
    } else {
        ESP_LOGE(TAG, "Mass erase failed: %s", esp_err_to_name(ret));
        snprintf(resp, sizeof(resp), 
            "{\"success\":false,\"message\":\"Mass erase failed: %s\"}", 
            esp_err_to_name(ret));
    }
    
    ESP_LOGI(TAG, "Mass erase complete, releasing target...");
    swd_release_target();
    swd_shutdown();
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

// Upload handler
static esp_err_t upload_post_handler(httpd_req_t *req) {
    char buf[1024];
    int remaining = req->content_len;
    
    ESP_LOGI(TAG, "Starting hex upload: %d bytes", remaining);
    
    // Clean up any previous context
    if (g_upload_ctx) {
        if (g_upload_ctx->parser) {
            hex_stream_free(g_upload_ctx->parser);
        }
        free(g_upload_ctx->page_buffer);
        free(g_upload_ctx);
    }
    
    // Allocate new context
    g_upload_ctx = calloc(1, sizeof(upload_context_t));
    if (!g_upload_ctx) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }
    
    g_upload_ctx->page_buffer = malloc(PAGE_BUFFER_SIZE);
    if (!g_upload_ctx->page_buffer) {
        free(g_upload_ctx);
        g_upload_ctx = NULL;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }
    
    memset(g_upload_ctx->page_buffer, 0xFF, PAGE_BUFFER_SIZE);
    
    // Parse query string for target type
    char query[64] = {0};
    httpd_req_get_url_query_str(req, query, sizeof(query));
    
    if (strstr(query, "type=bootloader")) {
        // Bootloader hex files typically contain the full flash image
        // including MBR at 0x0, SoftDevice, and bootloader at 0xF4000
        ESP_LOGI(TAG, "Flashing bootloader - using addresses from hex file");
        g_upload_ctx->start_addr = 0xFFFFFFFF; // Use hex file addresses
    } else if (strstr(query, "type=app")) {
        g_upload_ctx->start_addr = 0x26000;  // After SoftDevice
        ESP_LOGI(TAG, "Flashing application at 0x26000");
    } else if (strstr(query, "type=softdevice")) {
        g_upload_ctx->start_addr = 0x1000;
        ESP_LOGI(TAG, "Flashing SoftDevice at 0x1000");
    } else {
        g_upload_ctx->start_addr = 0xFFFFFFFF; // Use hex addresses
        ESP_LOGI(TAG, "Flashing at addresses from hex file");
    }
    
    // Add after parsing query string, before creating hex parser:
    esp_err_t ret = ensure_swd_ready();
    if (ret != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SWD not ready");
        return ESP_FAIL;
    }

    // Create hex parser
    g_upload_ctx->parser = hex_stream_create(hex_flash_callback, g_upload_ctx);
    g_upload_ctx->in_progress = true;
    g_upload_ctx->total_bytes = remaining;
    
    // Process upload
    while (remaining > 0) {
        int recv_len = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));
        
        if (recv_len <= 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE(TAG, "Upload receive failed");
            g_upload_ctx->error = true;
            snprintf(g_upload_ctx->status_msg, sizeof(g_upload_ctx->status_msg),
                    "Error: Upload failed");
            break;
        }
        
        // Parse hex data
        esp_err_t ret = hex_stream_parse(g_upload_ctx->parser, (uint8_t*)buf, recv_len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Hex parse failed");
            g_upload_ctx->error = true;
            snprintf(g_upload_ctx->status_msg, sizeof(g_upload_ctx->status_msg),
                    "Error: Invalid hex file");
            break;
        }
        
        g_upload_ctx->received_bytes += recv_len;
        remaining -= recv_len;
        
        // Log progress
        if ((g_upload_ctx->received_bytes % 10240) == 0) {
            int percent = (g_upload_ctx->received_bytes * 100) / g_upload_ctx->total_bytes;
            ESP_LOGI(TAG, "Upload: %d%% (%lu/%lu bytes)", 
                    percent, g_upload_ctx->received_bytes, g_upload_ctx->total_bytes);
        }
    }
    
    g_upload_ctx->in_progress = false;
    
    // Send response
    char resp[256];
    if (!g_upload_ctx->error) {
        snprintf(resp, sizeof(resp), 
                "{\"status\":\"success\",\"message\":\"%s\"}", 
                g_upload_ctx->status_msg);
    } else {
        snprintf(resp, sizeof(resp), 
                "{\"status\":\"error\",\"message\":\"%s\"}", 
                g_upload_ctx->status_msg);
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    
    return ESP_OK;
}

// Progress handler
static esp_err_t progress_handler(httpd_req_t *req) {
    char resp[256];
    
    if (g_upload_ctx && g_upload_ctx->in_progress) {
        int upload_percent = (g_upload_ctx->received_bytes * 100) / g_upload_ctx->total_bytes;
        int flash_percent = (g_upload_ctx->flashed_bytes * 100) / g_upload_ctx->total_bytes;
        
        snprintf(resp, sizeof(resp),
                "{\"in_progress\":true,\"upload_percent\":%d,\"flash_percent\":%d,"
                "\"received\":%lu,\"flashed\":%lu,\"total\":%lu}",
                upload_percent, flash_percent,
                g_upload_ctx->received_bytes, g_upload_ctx->flashed_bytes,
                g_upload_ctx->total_bytes);
    } else if (g_upload_ctx) {
        snprintf(resp, sizeof(resp),
                "{\"in_progress\":false,\"message\":\"%s\"}",
                g_upload_ctx->status_msg);
    } else {
        strcpy(resp, "{\"in_progress\":false,\"message\":\"Ready\"}");
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

// In components/web/src/web_upload.c, replace the existing disable_protection_handler with:

esp_err_t disable_protection_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "=== Mass Erase Request from Web Interface ===");
    
    char resp[256];
    
    // Check connection first
    if (!swd_is_connected()) {
        ESP_LOGE(TAG, "SWD not connected");
        strcpy(resp, "{\"success\":false,\"message\":\"SWD not connected\"}");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, resp, strlen(resp));
        return ESP_OK;
    }
    
    // Call the new CTRL-AP mass erase function
    esp_err_t ret = swd_flash_mass_erase_ctrl_ap();
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Mass erase successful");
        strcpy(resp, "{\"success\":true,\"message\":\"Mass erase complete. Device fully erased and unlocked.\"}");
    } else {
        ESP_LOGE(TAG, "✗ Mass erase failed: %s", esp_err_to_name(ret));
        snprintf(resp, sizeof(resp), 
                "{\"success\":false,\"message\":\"Mass erase failed: %s\"}", 
                esp_err_to_name(ret));
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

// Also in components/web/src/web_upload.c, update erase_all_handler:

esp_err_t erase_all_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "=== Full Chip Erase Request ===");
    
    char resp[256];
    
    // Check connection first
    if (!swd_is_connected()) {
        ESP_LOGE(TAG, "SWD not connected");
        strcpy(resp, "{\"success\":false,\"message\":\"SWD not connected\"}");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, resp, strlen(resp));
        return ESP_OK;
    }
    
    // Use the same CTRL-AP mass erase for full chip erase
    esp_err_t ret = swd_flash_mass_erase_ctrl_ap();
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Chip erase successful");
        strcpy(resp, "{\"success\":true,\"message\":\"Chip erased successfully. All memory cleared.\"}");
    } else {
        ESP_LOGE(TAG, "✗ Chip erase failed: %s", esp_err_to_name(ret));
        snprintf(resp, sizeof(resp), 
                "{\"success\":false,\"message\":\"Chip erase failed: %s\"}", 
                esp_err_to_name(ret));
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

// Register all handlers
esp_err_t register_upload_handlers(httpd_handle_t server) {
    httpd_uri_t upload_uri = {
        .uri = "/upload",
        .method = HTTP_POST,
        .handler = upload_post_handler,
        .user_ctx = NULL
    };
    
    httpd_uri_t progress_uri = {
        .uri = "/progress",
        .method = HTTP_GET,
        .handler = progress_handler,
        .user_ctx = NULL
    };

    httpd_uri_t check_swd_uri = {
        .uri = "/check_swd",
        .method = HTTP_GET,
        .handler = check_swd_handler,
        .user_ctx = NULL
    };
    
    httpd_uri_t mass_erase_uri = {
        .uri = "/mass_erase",
        .method = HTTP_GET,
        .handler = mass_erase_handler,
        .user_ctx = NULL
    };
    
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &upload_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &progress_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &check_swd_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &mass_erase_uri));
    
    ESP_LOGI(TAG, "All handlers registered");
    return ESP_OK;
}