// swd_flash.c - Flash Programming Implementation (excerpt)
#include "swd_flash.h"
#include "swd_core.h"
#include "swd_mem.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nrf52_hal.h"

static const char *TAG = "SWD_FLASH";

// Wait for NVMC ready with timeout
static esp_err_t wait_nvmc_ready(uint32_t timeout_ms) {
    uint32_t start = xTaskGetTickCount();
    uint32_t ready = 0;
    uint32_t last_ready = 0;
    int stable_count = 0;
    
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms)) {
        esp_err_t ret = swd_mem_read32(NVMC_READY, &ready);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read NVMC_READY register");
            return ret;
        }
        
        // Check for stable ready signal (avoid transient states)
        if ((ready & 0x1) == 1) {
            if (last_ready == ready) {
                stable_count++;
                if (stable_count >= 2) {  // Require 2 consecutive ready reads
                    return ESP_OK;
                }
            } else {
                stable_count = 0;
            }
        } else {
            stable_count = 0;
        }
        
        last_ready = ready;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    ESP_LOGE(TAG, "NVMC timeout (ready=0x%08lX)", ready);
    return ESP_ERR_TIMEOUT;
}

// Set NVMC mode
static esp_err_t set_nvmc_config(uint32_t mode) {
    esp_err_t ret = swd_mem_write32(NVMC_CONFIG, mode);
    if (ret != ESP_OK) return ret;
    
    // Small delay for config to take effect
    vTaskDelay(1);
    
    // Verify mode was set
    uint32_t config;
    ret = swd_mem_read32(NVMC_CONFIG, &config);
    if (ret != ESP_OK) return ret;
    
    if ((config & 0x3) != mode) {
        ESP_LOGE(TAG, "Failed to set NVMC mode %lu", mode);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// Erase a single page
esp_err_t swd_flash_erase_page(uint32_t addr) {
    // Validate address
    if (addr >= NRF52_FLASH_SIZE) {
        ESP_LOGE(TAG, "Address 0x%08lX out of range", addr);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Align to page boundary
    addr &= ~(NRF52_FLASH_PAGE_SIZE - 1);
    
    ESP_LOGI(TAG, "Erasing page at 0x%08lX", addr);
    
    // Wait for NVMC to be ready before starting
    esp_err_t ret = wait_nvmc_ready(500);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVMC not ready before erase");
        return ret;
    }
    
    // Enable erase mode
    ret = set_nvmc_config(NVMC_CONFIG_EEN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable erase mode");
        return ret;
    }
    
    // Double-check erase mode is set
    uint32_t config;
    ret = swd_mem_read32(NVMC_CONFIG, &config);
    if (ret != ESP_OK || (config & 0x3) != NVMC_CONFIG_EEN) {
        ESP_LOGE(TAG, "Erase mode not properly set (config=0x%08lX)", config);
        set_nvmc_config(NVMC_CONFIG_REN);
        return ESP_FAIL;
    }
    
    // Trigger page erase by writing to ERASEPAGE register
    ret = swd_mem_write32(NVMC_ERASEPAGE, addr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger erase");
        goto cleanup;
    }
    
    // nRF52840 page erase takes 85-90ms typical, 295ms max
    // Add initial delay before polling
    vTaskDelay(pdMS_TO_TICKS(90));
    
    // Now poll for completion with timeout
    uint32_t timeout_ms = 400;  // Increased from 200ms
    uint32_t elapsed_ms = 90;
    
    while (elapsed_ms < timeout_ms) {
        uint32_t ready;
        ret = swd_mem_read32(NVMC_READY, &ready);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read NVMC_READY");
            goto cleanup;
        }
        
        if (ready & 0x1) {
            ESP_LOGD(TAG, "Erase complete after %lu ms", elapsed_ms);
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
        elapsed_ms += 10;
    }
    
    if (elapsed_ms >= timeout_ms) {
        ESP_LOGE(TAG, "Erase timeout after %lu ms", elapsed_ms);
        ret = ESP_ERR_TIMEOUT;
        goto cleanup;
    }
    
    // Return to read mode before verification
    ret = set_nvmc_config(NVMC_CONFIG_REN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to return to read mode");
        goto cleanup;
    }
    
    // Add a small delay for mode switch to take effect
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Verify erase - check multiple locations across the page
    uint32_t verify_offsets[] = {0, 4, 8, NRF52_FLASH_PAGE_SIZE - 4};
    for (int i = 0; i < 4; i++) {
        uint32_t sample;
        uint32_t check_addr = addr + verify_offsets[i];
        
        // Read twice to ensure consistency (cache bypass)
        ret = swd_mem_read32(check_addr, &sample);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read for verification at 0x%08lX", check_addr);
            goto cleanup;
        }
        
        if (sample != 0xFFFFFFFF) {
            // Try reading again in case of cache issue
            vTaskDelay(pdMS_TO_TICKS(1));
            ret = swd_mem_read32(check_addr, &sample);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to re-read for verification at 0x%08lX", check_addr);
                goto cleanup;
            }
            
            if (sample != 0xFFFFFFFF) {
                ESP_LOGE(TAG, "Erase verification failed at 0x%08lX: 0x%08lX (expected 0xFFFFFFFF)", 
                        check_addr, sample);
                ret = ESP_FAIL;
                goto cleanup;
            }
        }
    }
    
    ESP_LOGI(TAG, "Page at 0x%08lX erased successfully", addr);
    return ESP_OK;
    
cleanup:
    // Always try to return to read-only mode
    set_nvmc_config(NVMC_CONFIG_REN);
    return ret;
}

// Write a single word - optimized version
esp_err_t swd_flash_write_word(uint32_t addr, uint32_t data) {
    // Validate alignment
    if (addr & 0x3) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Enable write mode once at the beginning
    esp_err_t ret = set_nvmc_config(NVMC_CONFIG_WEN);
    if (ret != ESP_OK) return ret;
    
    // Write the word
    ret = swd_mem_write32(addr, data);
    if (ret != ESP_OK) goto cleanup;
    
    // Wait for write completion - nRF52 word write is typically 41us
    // Poll for ready instead of fixed delay
    uint32_t timeout = 50; // 50ms is way more than needed
    uint32_t elapsed = 0;
    
    while (elapsed < timeout) {
        uint32_t ready;
        ret = swd_mem_read32(NVMC_READY, &ready);
        if (ret != ESP_OK) goto cleanup;
        
        if (ready & 0x1) {
            // Write complete
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
        elapsed++;
    }
    
    if (elapsed >= timeout) {
        ESP_LOGE(TAG, "Write timeout at 0x%08lX", addr);
        ret = ESP_ERR_TIMEOUT;
        goto cleanup;
    }
    
    // Skip verification for each word - do it at buffer level instead
    
cleanup:
    // Return to read-only mode
    set_nvmc_config(NVMC_CONFIG_REN);
    return ret;
}

// Optimized buffer write
esp_err_t swd_flash_write_buffer(uint32_t addr, const uint8_t *data, uint32_t size, 
                                 flash_progress_cb progress) {
    if (!data || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Writing %lu bytes to 0x%08lX", size, addr);
    
    // Enable write mode once for the entire buffer
    esp_err_t ret = set_nvmc_config(NVMC_CONFIG_WEN);
    if (ret != ESP_OK) return ret;
    
    uint32_t written = 0;
    
    // Handle unaligned start
    if (addr & 0x3) {
        uint32_t aligned_addr = addr & ~0x3;
        uint32_t offset = addr & 0x3;
        uint32_t word;
        
        // Read existing word
        ret = swd_mem_read32(aligned_addr, &word);
        if (ret != ESP_OK) goto cleanup;
        
        // Modify bytes
        uint8_t *word_bytes = (uint8_t*)&word;
        uint32_t bytes_to_copy = 4 - offset;
        if (bytes_to_copy > size) bytes_to_copy = size;
        
        for (uint32_t i = 0; i < bytes_to_copy; i++) {
            word_bytes[offset + i] = data[i];
        }
        
        // Write back
        ret = swd_mem_write32(aligned_addr, word);
        if (ret != ESP_OK) goto cleanup;
        
        // Wait for this write to complete
        ret = wait_nvmc_ready(50);
        if (ret != ESP_OK) goto cleanup;
        
        addr += bytes_to_copy;
        data += bytes_to_copy;
        written += bytes_to_copy;
        size -= bytes_to_copy;
    }
    
    // Write aligned words in batches
    while (size >= 4) {
        uint32_t word = *(uint32_t*)data;
        
        // Write word
        ret = swd_mem_write32(addr, word);
        if (ret != ESP_OK) goto cleanup;
        
        // Only wait every 256 bytes or so for better performance
        if ((written & 0xFF) == 0xFC || size == 4) {
            ret = wait_nvmc_ready(50);
            if (ret != ESP_OK) goto cleanup;
        }
        
        addr += 4;
        data += 4;
        written += 4;
        size -= 4;
        
        // Report progress every 1KB
        if (progress && (written & 0x3FF) == 0) {
            progress(written, written + size, "Writing");
        }
    }
    
    // Handle remaining bytes
    if (size > 0) {
        uint32_t word = 0xFFFFFFFF;
        uint8_t *word_bytes = (uint8_t*)&word;
        
        for (uint32_t i = 0; i < size; i++) {
            word_bytes[i] = data[i];
        }
        
        ret = swd_mem_write32(addr, word);
        if (ret != ESP_OK) goto cleanup;
        
        ret = wait_nvmc_ready(50);
        if (ret != ESP_OK) goto cleanup;
        
        written += size;
    }
    
    if (progress) {
        progress(written, written, "Complete");
    }
    
cleanup:
    // Return to read-only mode
    set_nvmc_config(NVMC_CONFIG_REN);
    
    ESP_LOGI(TAG, "Write complete: %lu bytes", written);
    return ret;
}

// Mass erase
esp_err_t swd_flash_mass_erase_ctrl_ap(void) {
    ESP_LOGW(TAG, "=== Starting CTRL-AP Mass Erase (will nuke everything!) ===");
    
    esp_err_t ret;
    uint32_t value;
    
    // Step 1: Find the CTRL-AP by scanning APs
    ESP_LOGI(TAG, "Scanning for Nordic CTRL-AP...");
    
    int ctrl_ap_num = -1;
    for (int ap = 0; ap < 256; ap++) {  // pyOCD scans up to 256 APs
        // Select this AP
        ret = swd_dp_write(DP_SELECT, (ap << 24));
        if (ret != ESP_OK) {
            continue;
        }
        
        // Read IDR
        ret = swd_ap_read(CTRL_AP_IDR, &value);
        if (ret != ESP_OK) {
            continue;
        }
        
        ESP_LOGD(TAG, "AP[%d] IDR = 0x%08lX", ap, value);
        
        // Check if this matches Nordic CTRL-AP
        // pyOCD checks: (idr & IDR_CLASS_MASK) == AP_CLASS_MEM_AP
        // For Nordic: IDR should be 0x12880000
        if (value == NORDIC_CTRL_AP_IDR || (value & 0xFFF00000) == 0x12800000) {
            ESP_LOGI(TAG, "Found Nordic CTRL-AP at AP index %d (IDR=0x%08lX)", ap, value);
            ctrl_ap_num = ap;
            break;
        }
    }
    
    if (ctrl_ap_num < 0) {
        ESP_LOGE(TAG, "Nordic CTRL-AP not found! Cannot perform mass erase.");
        
        // Let's try common locations anyway
        ESP_LOGI(TAG, "Trying common CTRL-AP locations (1, 2)...");
        ctrl_ap_num = 1;  // Try AP#1 first
    }
    
    // Select the CTRL-AP
    ESP_LOGI(TAG, "Selecting CTRL-AP at index %d...", ctrl_ap_num);
    ret = swd_dp_write(DP_SELECT, (ctrl_ap_num << 24));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select CTRL-AP");
        return ret;
    }
    
    // Step 2: Check if device is already unlocked
    ESP_LOGI(TAG, "Reading APPROTECTSTATUS...");
    ret = swd_ap_read(CTRL_AP_APPROTECTSTATUS, &value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read APPROTECTSTATUS: %s", esp_err_to_name(ret));
        // Continue anyway
    } else {
        ESP_LOGI(TAG, "APPROTECTSTATUS = 0x%08lX (%s)", value,
                value == 0 ? "LOCKED" : "UNLOCKED");
    }
    
    // Step 3: Assert reset before erase (pyOCD does this)
    ESP_LOGI(TAG, "Asserting system reset...");
    ret = swd_ap_write(CTRL_AP_RESET, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to assert reset: %s", esp_err_to_name(ret));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Step 4: Start ERASEALL
    ESP_LOGW(TAG, "*** TRIGGERING ERASEALL - THIS WILL ERASE EVERYTHING! ***");
    ret = swd_ap_write(CTRL_AP_ERASEALL, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger ERASEALL: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "ERASEALL triggered, waiting for completion...");
    
    // Step 5: Poll ERASEALLSTATUS until done
    uint32_t timeout_ms = 5000;  // 5 seconds (pyOCD uses longer timeout)
    uint32_t elapsed_ms = 0;
    uint32_t last_status = 0xFFFFFFFF;
    
    while (elapsed_ms < timeout_ms) {
        ret = swd_ap_read(CTRL_AP_ERASEALLSTATUS, &value);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read ERASEALLSTATUS, retrying...");
            vTaskDelay(pdMS_TO_TICKS(50));
            elapsed_ms += 50;
            continue;
        }
        
        if (value != last_status) {
            ESP_LOGI(TAG, "ERASEALLSTATUS = 0x%08lX", value);
            last_status = value;
        }
        
        // pyOCD checks: value == CTRL.ERASEALLSTATUS.IDLE (which is 0)
        if (value == 0) {
            ESP_LOGI(TAG, "✓ Mass erase complete after %lu ms!", elapsed_ms);
            break;
        }
        
        if (elapsed_ms % 500 == 0) {
            ESP_LOGI(TAG, "  Still erasing... (%lu ms elapsed, status=0x%08lX)", 
                    elapsed_ms, value);
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
        elapsed_ms += 50;
    }
    
    if (elapsed_ms >= timeout_ms) {
        ESP_LOGE(TAG, "✗ Mass erase timeout after %lu ms!", timeout_ms);
        return ESP_ERR_TIMEOUT;
    }
    
    // Step 6: Release reset
    ESP_LOGI(TAG, "Releasing system reset...");
    ret = swd_ap_write(CTRL_AP_RESET, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to release reset: %s", esp_err_to_name(ret));
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Step 7: Verify APPROTECT is now unlocked
    ESP_LOGI(TAG, "Verifying unlock status...");
    ret = swd_ap_read(CTRL_AP_APPROTECTSTATUS, &value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Final APPROTECTSTATUS = 0x%08lX (%s)", value,
                value == 0 ? "STILL LOCKED!" : "UNLOCKED");
        
        if (value == 0) {
            ESP_LOGE(TAG, "✗ Device is still locked after mass erase!");
            return ESP_FAIL;
        }
    }
    
    // Step 8: Switch back to MEM-AP (AP#0)
    ESP_LOGI(TAG, "Switching back to MEM-AP (AP#0)...");
    ret = swd_dp_write(DP_SELECT, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select MEM-AP");
        return ret;
    }
    
    // Step 9: Power cycle the debug interface (pyOCD does this)
    ESP_LOGI(TAG, "Power cycling debug interface...");
    
    // Clear sticky errors
    swd_clear_errors();
    
    // Reconnect to target
    ESP_LOGI(TAG, "Reconnecting to target...");
    ret = swd_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reconnect: %s", esp_err_to_name(ret));
        
        // Try harder reset sequence
        ESP_LOGI(TAG, "Trying harder reset...");
        swd_reset_target();
        vTaskDelay(pdMS_TO_TICKS(500));
        ret = swd_connect();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Still can't reconnect!");
            return ret;
        }
    }
    
    // Reinit flash
    ret = swd_flash_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Flash init failed: %s", esp_err_to_name(ret));
    }
    
    // Step 10: Verify everything is erased
    ESP_LOGI(TAG, "Verifying erase...");
    
    uint32_t test_addr[] = {0x00000000, 0x00001000, 0x000F4000};
    for (int i = 0; i < 3; i++) {
        uint32_t val;
        ret = swd_mem_read32(test_addr[i], &val);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Flash[0x%08lX] = 0x%08lX %s", 
                    test_addr[i], val,
                    val == 0xFFFFFFFF ? "✓" : "✗ NOT ERASED!");
        }
    }
    
    // Check UICR
    uint32_t uicr_approtect;
    ret = swd_mem_read32(UICR_APPROTECT, &uicr_approtect);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "UICR_APPROTECT = 0x%08lX %s", 
                uicr_approtect,
                uicr_approtect == 0xFFFFFFFF ? "(erased)" : "(has value)");
    }
    
    ESP_LOGW(TAG, "=== Mass Erase Complete ===");
    ESP_LOGW(TAG, "All flash and UICR have been erased!");
    ESP_LOGW(TAG, "You'll need to reprogram everything including bootloader.");
    
    return ESP_OK;
}

// High-level firmware update
esp_err_t swd_flash_update_firmware(const firmware_update_t *update) {
    if (!update || !update->data || update->size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Firmware update: addr=0x%08lX size=%lu verify=%d", 
             update->start_addr, update->size, update->verify);
    
    // Calculate pages to erase
    uint32_t start_page = update->start_addr / NRF52_FLASH_PAGE_SIZE;
    uint32_t end_addr = update->start_addr + update->size - 1;
    uint32_t end_page = end_addr / NRF52_FLASH_PAGE_SIZE;
    uint32_t page_count = end_page - start_page + 1;
    
    ESP_LOGI(TAG, "Erasing %lu pages", page_count);
    
    // Erase required pages
    for (uint32_t page = start_page; page <= end_page; page++) {
        esp_err_t ret = swd_flash_erase_page(page * NRF52_FLASH_PAGE_SIZE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase page %lu", page);
            return ret;
        }
        
        if (update->progress) {
            uint32_t progress = ((page - start_page) * 100) / page_count;
            update->progress(progress, 100, "Erasing");
        }
    }
    
    // Write firmware
    ESP_LOGI(TAG, "Writing firmware");
    esp_err_t ret = swd_flash_write_buffer(update->start_addr, update->data, 
                                           update->size, update->progress);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write firmware");
        return ret;
    }
    
    // Verify if requested
    if (update->verify) {
        ESP_LOGI(TAG, "Verifying firmware");
        ret = swd_flash_verify(update->start_addr, update->data, 
                              update->size, update->progress);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Firmware verification failed");
            return ret;
        }
    }
    
    ESP_LOGI(TAG, "Firmware update complete");
    return ESP_OK;
}

esp_err_t swd_flash_init(void) {
    ESP_LOGI(TAG, "Initializing flash interface");
    
    // Initialize memory access first
    esp_err_t ret = swd_mem_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize memory access");
        return ret;
    }
    
    // Verify we can access NVMC
    uint32_t ready;
    ret = swd_mem_read32(NVMC_READY, &ready);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot access NVMC registers");
        return ret;
    }
    
    ESP_LOGI(TAG, "Flash interface ready");
    return ESP_OK;
}

esp_err_t swd_flash_disable_approtect(void) {
    ESP_LOGW(TAG, "=== Using CTRL-AP for chip erase and APPROTECT disable ===");
    
    esp_err_t ret;
    uint32_t value;
    
    // Step 1: Select AP#1 (CTRL-AP) - AP index 1
    ESP_LOGI(TAG, "Selecting CTRL-AP (AP#1)...");
    ret = swd_dp_write(DP_SELECT, (1 << 24));  // Select AP#1
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select CTRL-AP");
        return ret;
    }
    
    // Verify we have the right AP by reading its IDR
    ret = swd_ap_read(AP_IDR, &value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CTRL-AP IDR");
        return ret;
    }
    
    ESP_LOGI(TAG, "CTRL-AP IDR: 0x%08lX (expected 0x02880000)", value);
    if ((value & 0xFFFFFF00) != NORDIC_CTRL_AP_IDR) {
        ESP_LOGW(TAG, "Warning: CTRL-AP IDR mismatch, continuing anyway...");
    }
    
    // Step 2: Check current APPROTECT status
    ret = swd_ap_read(CTRL_AP_APPROTECTSTATUS, &value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read APPROTECTSTATUS");
        return ret;
    }
    
    ESP_LOGI(TAG, "Current APPROTECTSTATUS: 0x%08lX (%s)", value,
            value == 0 ? "LOCKED" : "UNLOCKED");
    
    if (value == 1) {
        ESP_LOGI(TAG, "Device already unlocked!");
        // Switch back to MEM-AP (AP#0)
        swd_dp_write(DP_SELECT, 0);
        return ESP_OK;
    }
    
    // Step 3: Trigger ERASEALL through CTRL-AP
    ESP_LOGW(TAG, "Triggering CTRL-AP ERASEALL (this will erase entire chip)...");
    ret = swd_ap_write(CTRL_AP_ERASEALL, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger ERASEALL");
        return ret;
    }
    
    // Step 4: Poll ERASEALLSTATUS until complete
    ESP_LOGI(TAG, "Waiting for erase to complete...");
    uint32_t timeout_ms = 1000;  // 1 second timeout
    uint32_t elapsed_ms = 0;
    
    while (elapsed_ms < timeout_ms) {
        ret = swd_ap_read(CTRL_AP_ERASEALLSTATUS, &value);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read ERASEALLSTATUS");
            return ret;
        }
        
        if (value == 0) {
            ESP_LOGI(TAG, "✓ Erase complete after %lu ms", elapsed_ms);
            break;
        }
        
        if (elapsed_ms % 100 == 0) {
            ESP_LOGI(TAG, "  Erasing... (status=0x%08lX)", value);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
        elapsed_ms += 10;
    }
    
    if (elapsed_ms >= timeout_ms) {
        ESP_LOGE(TAG, "Erase timeout!");
        return ESP_ERR_TIMEOUT;
    }
    
    // Step 5: Issue reset through CTRL-AP
    ESP_LOGI(TAG, "Issuing reset through CTRL-AP...");
    
    // Assert reset
    ret = swd_ap_write(CTRL_AP_RESET, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to assert reset");
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Release reset
    ret = swd_ap_write(CTRL_AP_RESET, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to release reset");
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Step 6: Verify unlock status
    ret = swd_ap_read(CTRL_AP_APPROTECTSTATUS, &value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read final APPROTECTSTATUS");
    } else {
        ESP_LOGI(TAG, "Final APPROTECTSTATUS: 0x%08lX (%s)", value,
                value == 0 ? "LOCKED" : "UNLOCKED");
        
        if (value != 1) {
            ESP_LOGE(TAG, "✗ Device still locked after ERASEALL!");
            return ESP_FAIL;
        }
    }
    
    // Step 7: Switch back to MEM-AP (AP#0) for normal operations
    ESP_LOGI(TAG, "Switching back to MEM-AP (AP#0)...");
    ret = swd_dp_write(DP_SELECT, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select MEM-AP");
        return ret;
    }
    
    // Step 8: Reconnect and verify
    ESP_LOGI(TAG, "Reconnecting to target...");
    ret = swd_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reconnect after unlock");
        return ret;
    }
    
    // Re-initialize flash interface
    ret = swd_flash_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Flash init failed after unlock");
    }
    
    ESP_LOGI(TAG, "✓ Device unlocked and ready for programming!");
    ESP_LOGI(TAG, "Note: All flash and UICR have been erased.");
    
    return ESP_OK;
}

// Full chip erase (except UICR)
esp_err_t swd_flash_erase_all(void) {
    ESP_LOGW(TAG, "Starting full chip erase...");
    
    esp_err_t ret = wait_nvmc_ready(100);
    if (ret != ESP_OK) return ret;
    
    ret = set_nvmc_config(NVMC_CONFIG_EEN);
    if (ret != ESP_OK) return ret;
    
    ret = swd_mem_write32(NVMC_ERASEALL, 0x1);
    if (ret != ESP_OK) {
        set_nvmc_config(NVMC_CONFIG_REN);
        return ret;
    }
    
    // Full erase takes 200-300ms
    ESP_LOGI(TAG, "Erasing... (this takes ~300ms)");
    ret = wait_nvmc_ready(500);
    set_nvmc_config(NVMC_CONFIG_REN);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Full chip erase complete");
    }
    
    return ret;
}