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
    // New (correct) - UICR is at 0x10001000 and is valid
    if (addr >= NRF52_FLASH_SIZE && addr != 0x10001000) {
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

// Optimized single word write
esp_err_t swd_flash_write_word(uint32_t addr, uint32_t data) {
    if (addr & 0x3) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = set_nvmc_config(NVMC_CONFIG_WEN);
    if (ret != ESP_OK) return ret;
    
    ret = swd_mem_write32(addr, data);
    
    set_nvmc_config(NVMC_CONFIG_REN);
    return ret;
}

// Fast flash write with proper alignment handling
esp_err_t swd_flash_write_buffer(uint32_t addr, const uint8_t *data, uint32_t size, 
                                 flash_progress_cb progress) {
    if (!data || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Writing %lu bytes to 0x%08lX", size, addr);
    uint32_t start_tick = xTaskGetTickCount();
    
    esp_err_t ret;
    uint32_t written = 0;
    
    // Enable write mode
    ret = swd_mem_write32(NVMC_CONFIG, NVMC_CONFIG_WEN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable write mode");
        return ret;
    }
    vTaskDelay(1);
    
    // Handle unaligned start
    if (addr & 0x3) {
        uint32_t aligned_addr = addr & ~0x3;
        uint32_t offset = addr & 0x3;
        uint32_t word;
        
        ret = swd_mem_read32(aligned_addr, &word);
        if (ret != ESP_OK) goto cleanup;
        
        uint8_t *word_bytes = (uint8_t*)&word;
        uint32_t bytes_to_copy = 4 - offset;
        if (bytes_to_copy > size) bytes_to_copy = size;
        
        memcpy(&word_bytes[offset], data, bytes_to_copy);
        
        ret = swd_mem_write32(aligned_addr, word);
        if (ret != ESP_OK) goto cleanup;
        
        addr += bytes_to_copy;
        data += bytes_to_copy;
        written += bytes_to_copy;
        size -= bytes_to_copy;
    }
    
    // Allocate aligned buffer for word writes
    // This ensures we have properly aligned data for swd_mem_write_block32
    uint32_t max_words = 1024;  // Write up to 4KB at a time
    uint32_t *word_buffer = malloc(max_words * sizeof(uint32_t));
    if (!word_buffer) {
        ESP_LOGE(TAG, "Failed to allocate write buffer");
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    
    // Write aligned words in chunks
    while (size >= 4) {
        // Determine chunk size (up to max_words)
        uint32_t words_in_chunk = size / 4;
        if (words_in_chunk > max_words) {
            words_in_chunk = max_words;
        }
        
        // Copy data to aligned word buffer
        memcpy(word_buffer, data, words_in_chunk * 4);
        
        // This is the key optimization - writes many words in one transaction
        ret = swd_mem_write_block32(addr, word_buffer, words_in_chunk);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Block write failed at 0x%08lX", addr);
            free(word_buffer);
            goto cleanup;
        }
        
        uint32_t bytes_written = words_in_chunk * 4;
        addr += bytes_written;
        data += bytes_written;
        written += bytes_written;
        size -= bytes_written;
        
        // Report progress
        if (progress && (written & 0xFFF) == 0) {  // Every 4KB
            progress(written, written + size, "Writing");
        }
    }
    
    free(word_buffer);
    
    // Handle remaining bytes (less than a word)
    if (size > 0) {
        uint32_t word = 0xFFFFFFFF;
        memcpy(&word, data, size);
        
        ret = swd_mem_write32(addr, word);
        if (ret != ESP_OK) goto cleanup;
        
        written += size;
    }
    
    // Wait for final write to complete
    uint32_t timeout = 100;
    while (timeout--) {
        uint32_t ready;
        ret = swd_mem_read32(NVMC_READY, &ready);
        if (ret != ESP_OK) break;
        if (ready & 0x1) break;
        vTaskDelay(1);
    }
    
    if (progress) {
        progress(written, written, "Complete");
    }
    
cleanup:
    // Return to read mode
    swd_mem_write32(NVMC_CONFIG, NVMC_CONFIG_REN);
    
    uint32_t elapsed_ms = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
    float speed_kbps = elapsed_ms > 0 ? (float)(written * 1000) / (elapsed_ms * 1024) : 0;
    ESP_LOGI(TAG, "Write complete: %lu bytes in %lu ms (%.1f KB/s)", 
            written, elapsed_ms, speed_kbps);
    
    return ret;
}

// Mass erase
esp_err_t swd_flash_mass_erase_ctrl_ap(void) {
    // Just call the main implementation
    return swd_flash_disable_approtect();
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
    ESP_LOGW(TAG, "=== Starting CTRL-AP Mass Erase ===");
    
    if (!swd_is_connected()) {
        ESP_LOGE(TAG, "SWD not connected!");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Checking connection...");
    swd_clear_errors();
    
    uint32_t idcode = swd_get_idcode();
    if (idcode == 0 || idcode == 0xFFFFFFFF) {
        ESP_LOGE(TAG, "Invalid IDCODE: 0x%08lX", idcode);
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "DP IDCODE: 0x%08lX", idcode);
    
    esp_err_t ret;
    uint32_t value;
    
    // Step 1: Power up debug
    ESP_LOGI(TAG, "Powering up debug...");
    ret = swd_dp_write(DP_CTRL_STAT, 0x50000000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to power up debug");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Step 2: Read CTRL-AP IDR (at 0xFC, which is in bank 15)
    // SELECT register format: [31:24]=APSEL, [7:4]=APBANKSEL
    // For AP#1, bank 15 (0xF): APSEL=1, APBANKSEL=0xF
    ESP_LOGI(TAG, "Reading CTRL-AP IDR (AP#1, bank 15)...");
    ret = swd_dp_write(DP_SELECT, (1 << 24) | (0xF << 4));  // AP#1, Bank 15
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select AP#1 bank 15");
        return ret;
    }
    
    // Read IDR at offset 0x0C within bank 15 (0xFC = 0xF0 + 0x0C)
    ret = swd_ap_read(0x0C, &value);  // 0x0C within the bank
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CTRL-AP IDR");
        return ret;
    }
    
    ESP_LOGI(TAG, "CTRL-AP IDR = 0x%08lX", value);
    
    // Check if this is a Nordic CTRL-AP (mask out version bits)
    if ((value & 0x0FFFFFFF) != 0x02880000) {
        ESP_LOGE(TAG, "Not a Nordic CTRL-AP! Expected 0x02880000, got 0x%08lX", value);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Step 3: Switch to bank 0 for control registers
    ESP_LOGI(TAG, "Switching to bank 0 for control registers...");
    ret = swd_dp_write(DP_SELECT, (1 << 24) | (0 << 4));  // AP#1, Bank 0
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select AP#1 bank 0");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Step 4: Read APPROTECTSTATUS
    ret = swd_ap_read(0x0C, &value);  // CTRL_AP_APPROTECTSTATUS
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "APPROTECTSTATUS = 0x%08lX (%s)", value,
                value == 0 ? "Protected" : "Not Protected");
    }
    
    // Step 5: Trigger mass erase
    ESP_LOGI(TAG, "Writing ERASEALL = 1...");
    ret = swd_ap_write(0x04, 1);  // CTRL_AP_ERASEALL = 1
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write ERASEALL!");
        return ret;
    }
    
    // Force write completion
    uint32_t dummy;
    swd_dp_read(DP_RDBUFF, &dummy);
    
    // Step 6: Poll ERASEALLSTATUS
    ESP_LOGI(TAG, "Waiting for erase completion...");
    uint32_t timeout_ms = 15000;  // 15 seconds as per pyOCD
    uint32_t elapsed_ms = 0;
    bool complete = false;
    
    while (elapsed_ms < timeout_ms) {
        ret = swd_ap_read(0x08, &value);  // CTRL_AP_ERASEALLSTATUS
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read ERASEALLSTATUS, retrying...");
            // Re-select AP
            swd_dp_write(DP_SELECT, (1 << 24) | (0 << 4));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        if (value == 0) {  // CTRL_AP_ERASEALLSTATUS_READY
            ESP_LOGI(TAG, "✓ Mass erase complete in %lu ms!", elapsed_ms);
            complete = true;
            break;
        }
        
        // Log progress periodically
        if ((elapsed_ms % 1000) == 0) {
            ESP_LOGI(TAG, "Erasing... %lu ms elapsed (status=0x%08lX)", elapsed_ms, value);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // Poll every 100ms like pyOCD
        elapsed_ms += 100;
    }
    
    if (!complete) {
        ESP_LOGE(TAG, "✗ Mass erase timeout!");
        return ESP_ERR_TIMEOUT;
    }
    
    // Step 7: Reset sequence (as per pyOCD)
    ESP_LOGI(TAG, "Performing reset sequence...");
    swd_ap_write(0x00, 1);  // CTRL_AP_RESET = 1
    swd_dp_read(DP_RDBUFF, &dummy);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    swd_ap_write(0x00, 0);  // CTRL_AP_RESET = 0
    swd_dp_read(DP_RDBUFF, &dummy);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    swd_ap_write(0x04, 0);  // CTRL_AP_ERASEALL = 0
    swd_dp_read(DP_RDBUFF, &dummy);
    
    // Step 8: Switch back to MEM-AP
    ESP_LOGI(TAG, "Switching back to MEM-AP...");
    ret = swd_dp_write(DP_SELECT, 0x00000000);  // AP#0, Bank 0
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select MEM-AP");
    }
    
    // Step 9: Reconnect
    ESP_LOGI(TAG, "Reconnecting...");
    swd_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ret = swd_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reconnect - power cycle the device!");
        return ret;
    }
    
    swd_mem_init();
    swd_flash_init();
    
    // Step 10: Verify
    ESP_LOGI(TAG, "Verifying erase...");
    uint32_t val;
    ret = swd_mem_read32(0x00000000, &val);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Flash[0x0] = 0x%08lX %s", val, 
                (val == 0xFFFFFFFF) ? "✓ ERASED" : "✗ NOT ERASED");
    }
    
    ret = swd_mem_read32(0x10001208, &val);  // UICR.APPROTECT
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "APPROTECT = 0x%08lX %s", val,
                (val == 0xFFFFFFFF) ? "✓ ERASED" : "✗ NOT ERASED");
    }
    
    ESP_LOGW(TAG, "=== Mass Erase Complete ===");
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

esp_err_t swd_flash_reset_and_run(void) {
    ESP_LOGI(TAG, "Performing post-flash reset sequence...");

    // 1. Ensure NVMC is in read-only mode
    esp_err_t ret = swd_mem_write32(NVMC_CONFIG, NVMC_CONFIG_REN);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set NVMC to read-only");
    }

    // 2. Wait for NVMC to be ready
    vTaskDelay(pdMS_TO_TICKS(10));

    // 3. Clear instruction cache
    uint32_t icachecnf = 0x00000001;  // Enable cache
    ret = swd_mem_write32(NVMC_ICACHECNF, icachecnf);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 4. Invalidate cache
    icachecnf = 0x00000003;  // Enable + Invalidate
    ret = swd_mem_write32(NVMC_ICACHECNF, icachecnf);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 5. Reset vector table to start of flash
    ret = swd_mem_write32(NRF52_VTOR, 0x00000000);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set VTOR");
    }

    // 6. Release and reset the target with full DP disconnect
    return swd_release_target();
}