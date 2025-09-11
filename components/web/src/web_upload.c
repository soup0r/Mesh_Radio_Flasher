#include "web_upload.h"
#include "esp_log.h"
#include "hex_parser.h"
#include "swd_flash.h"
#include "swd_mem.h"
#include "swd_core.h"
#include "nrf52_hal.h"
#include <string.h>

static const char *TAG = "WEB_UPLOAD";

// Add this macro definition
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#define PAGE_BUFFER_SIZE (16 * 1024)  // 16KB buffer for multiple pages
#define NRF52_PAGE_SIZE 4096

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
            break;
            
        case HEX_TYPE_EXT_LIN_ADDR:
            // Flush buffer before address change
            if (uctx->buffer_data_len > 0) {
                flush_buffer(uctx);
            }
            break;
    }
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
    
    if (strstr(query, "type=app")) {
        g_upload_ctx->start_addr = 0x26000;  // After SoftDevice
        ESP_LOGI(TAG, "Flashing application at 0x26000");
    } else if (strstr(query, "type=softdevice")) {
        g_upload_ctx->start_addr = 0x1000;
        ESP_LOGI(TAG, "Flashing SoftDevice at 0x1000");
    } else if (strstr(query, "type=bootloader")) {
        g_upload_ctx->start_addr = 0xF4000;
        ESP_LOGI(TAG, "Flashing bootloader at 0xF4000");
    } else {
        g_upload_ctx->start_addr = 0x0;
        ESP_LOGI(TAG, "Flashing at address from hex file");
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

// Register handlers
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
    
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &check_swd_uri));
    
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &upload_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &progress_uri));
    
    ESP_LOGI(TAG, "Upload handlers registered");
    return ESP_OK;
}

esp_err_t disable_protection_handler(httpd_req_t *req) {
    char resp[128];
    esp_err_t ret = swd_flash_disable_approtect();
    
    if (ret == ESP_OK) {
        strcpy(resp, "{\"success\":true,\"message\":\"APPROTECT disabled successfully\"}");
    } else {
        strcpy(resp, "{\"success\":false,\"message\":\"Failed to disable APPROTECT\"}");
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

esp_err_t erase_all_handler(httpd_req_t *req) {
    char resp[128];
    esp_err_t ret = swd_flash_erase_all();
    
    if (ret == ESP_OK) {
        strcpy(resp, "{\"success\":true,\"message\":\"Chip erased successfully\"}");
    } else {
        strcpy(resp, "{\"success\":false,\"message\":\"Failed to erase chip\"}");
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

esp_err_t check_swd_handler(httpd_req_t *req) {
    char resp[256];
    
    // Check and try to reconnect if needed
    esp_err_t ret = check_and_reconnect_swd();
    bool connected = (ret == ESP_OK);
    
    uint32_t approtect = 0xFFFFFFFF;
    if (connected) {
        swd_mem_read32(UICR_APPROTECT, &approtect);
    }
    
    snprintf(resp, sizeof(resp),
        "{\"connected\":%s,\"approtect\":\"0x%08lX\",\"status\":\"%s\"}",
        connected ? "true" : "false",
        approtect,
        (approtect == 0xFFFFFF5A) ? "HwDisabled" : 
        (approtect == 0xFFFFFFFF) ? "Erased (Protected)" : 
        (approtect == 0xFFFFFF00) ? "Enabled" : "Unknown");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}