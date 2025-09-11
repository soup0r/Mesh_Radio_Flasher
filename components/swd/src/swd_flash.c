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
    ESP_LOGW(TAG, "=== Starting CTRL-AP Mass Erase (WILL ERASE EVERYTHING!) ===");
    
    esp_err_t ret;
    uint32_t value;
    
    // Step 1: Find the CTRL-AP by scanning APs
    ESP_LOGI(TAG, "Scanning for Nordic CTRL-AP...");
    
    int ctrl_ap_num = -1;
    for (int ap = 0; ap < 16; ap++) {  // Nordic typically uses AP#1 or AP#2
        // Select this AP
        ret = swd_dp_write(DP_SELECT, (ap << 24));
        if (ret != ESP_OK) {
            continue;
        }
        
        // Read IDR
        ret = swd_ap_read(AP_IDR, &value);
        if (ret != ESP_OK) {
            continue;
        }
        
        ESP_LOGD(TAG, "AP[%d] IDR = 0x%08lX", ap, value);
        
        // Nordic CTRL-AP should have IDR 0x02880000 or 0x12880000
        // But sometimes the upper bits vary, so check for the pattern
        if ((value & 0x0FFF0000) == 0x02880000 || 
            (value & 0x0FFF0000) == 0x12880000) {
            ESP_LOGI(TAG, "Found Nordic CTRL-AP at AP index %d (IDR=0x%08lX)", ap, value);
            ctrl_ap_num = ap;
            break;
        }
    }
    
    // If not found by IDR, try common locations (AP#1 is typical for nRF52)
    if (ctrl_ap_num < 0) {
        ESP_LOGW(TAG, "CTRL-AP not found by IDR, trying AP#1 (common for nRF52)");
        ctrl_ap_num = 1;
    }
    
    // Select the CTRL-AP
    ESP_LOGI(TAG, "Using CTRL-AP at index %d", ctrl_ap_num);
    ret = swd_dp_write(DP_SELECT, (ctrl_ap_num << 24));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select CTRL-AP");
        return ret;
    }
    
    // Step 2: Read current protection status (but don't trust it!)
    ESP_LOGI(TAG, "Reading APPROTECTSTATUS...");
    ret = swd_ap_read(CTRL_AP_APPROTECTSTATUS, &value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "APPROTECTSTATUS = 0x%08lX (%s)", value,
                value == 0 ? "Shows as LOCKED" : "Shows as UNLOCKED");
    }
    
    // Step 3: ALWAYS perform the erase, regardless of status!
    ESP_LOGW(TAG, "*** PERFORMING FULL CHIP ERASE ***");
    ESP_LOGW(TAG, "This will erase EVERYTHING including bootloader and SoftDevice!");
    
    // Assert reset during erase (some versions need this)
    ESP_LOGI(TAG, "Asserting system reset...");
    ret = swd_ap_write(CTRL_AP_RESET, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to assert reset, continuing anyway");
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Step 4: Trigger ERASEALL - THIS IS THE KEY OPERATION
    ESP_LOGI(TAG, "Writing to ERASEALL register...");
    ret = swd_ap_write(CTRL_AP_ERASEALL, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write ERASEALL register!");
        
        // Try alternative approach: write 0x00000001 to offset 0x04
        ESP_LOGI(TAG, "Trying alternative ERASEALL trigger...");
        ret = swd_ap_write(0x04, 0x00000001);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Alternative ERASEALL also failed!");
            return ret;
        }
    }
    
    ESP_LOGI(TAG, "ERASEALL triggered, waiting for completion...");
    ESP_LOGI(TAG, "This can take 20-90 seconds for a full chip erase!");
    
    // Step 5: Poll ERASEALLSTATUS with longer timeout for full erase
    uint32_t timeout_ms = 120000;  // 2 minutes (full erase can be slow)
    uint32_t elapsed_ms = 0;
    uint32_t poll_interval = 100;  // Check every 100ms
    uint32_t last_status = 0xFFFFFFFF;
    int unchanged_count = 0;
    
    while (elapsed_ms < timeout_ms) {
        ret = swd_ap_read(CTRL_AP_ERASEALLSTATUS, &value);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read ERASEALLSTATUS, retrying...");
            vTaskDelay(pdMS_TO_TICKS(poll_interval));
            elapsed_ms += poll_interval;
            continue;
        }
        
        if (value != last_status) {
            ESP_LOGI(TAG, "[%lu ms] ERASEALLSTATUS = 0x%08lX", elapsed_ms, value);
            last_status = value;
            unchanged_count = 0;
        } else {
            unchanged_count++;
        }
        
        // Status = 0 means erase complete
        if (value == 0) {
            ESP_LOGI(TAG, "✓ ERASEALL complete after %lu ms!", elapsed_ms);
            break;
        }
        
        // If status hasn't changed for a while, show progress
        if (unchanged_count >= 50) {  // 5 seconds of no change
            ESP_LOGI(TAG, "  Still erasing... %lu seconds elapsed", elapsed_ms / 1000);
            unchanged_count = 0;
        }
        
        vTaskDelay(pdMS_TO_TICKS(poll_interval));
        elapsed_ms += poll_interval;
    }
    
    if (elapsed_ms >= timeout_ms) {
        ESP_LOGE(TAG, "✗ Erase timeout after %lu ms!", timeout_ms);
        ESP_LOGE(TAG, "Last status was: 0x%08lX", value);
        return ESP_ERR_TIMEOUT;
    }
    
    // Step 6: Release reset
    ESP_LOGI(TAG, "Releasing system reset...");
    ret = swd_ap_write(CTRL_AP_RESET, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to release reset");
    }
    
    // Give the chip time to come out of reset
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Step 7: Power cycle the debug interface
    ESP_LOGI(TAG, "Power cycling debug interface...");
    
    // Disconnect and reconnect
    swd_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Clear errors and reconnect
    swd_clear_errors();
    
    ESP_LOGI(TAG, "Reconnecting to target...");
    ret = swd_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reconnect after erase");
        
        // Try a hard reset
        ESP_LOGI(TAG, "Attempting hard reset sequence...");
        swd_reset_target();
        vTaskDelay(pdMS_TO_TICKS(1000));
        ret = swd_connect();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Still can't reconnect - chip may need power cycle");
            return ret;
        }
    }
    
    // Step 8: Switch back to MEM-AP (AP#0)
    ESP_LOGI(TAG, "Switching back to MEM-AP...");
    ret = swd_dp_write(DP_SELECT, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select MEM-AP");
        return ret;
    }
    
    // Reinitialize memory and flash interfaces
    ret = swd_mem_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Memory init failed: %s", esp_err_to_name(ret));
    }
    
    ret = swd_flash_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Flash init failed: %s", esp_err_to_name(ret));
    }
    
    // Step 9: Verify the erase worked
    ESP_LOGI(TAG, "=== Verifying Full Chip Erase ===");
    
    // Check various memory locations
    uint32_t test_locations[] = {
        0x00000000,  // Start of flash (reset vector)
        0x00001000,  // MBR/Bootloader area  
        0x00010000,  // Application area
        0x000F4000,  // Bootloader location
        0x10001000 + 0x208  // UICR APPROTECT
    };
    
    const char *location_names[] = {
        "Flash Start",
        "MBR/Bootloader",
        "Application",
        "Bootloader",
        "UICR_APPROTECT"
    };
    
    bool all_erased = true;
    for (int i = 0; i < 5; i++) {
        uint32_t val;
        ret = swd_mem_read32(test_locations[i], &val);
        if (ret == ESP_OK) {
            bool erased = (val == 0xFFFFFFFF);
            ESP_LOGI(TAG, "%s [0x%08lX] = 0x%08lX %s", 
                    location_names[i], test_locations[i], val,
                    erased ? "✓ ERASED" : "✗ NOT ERASED!");
            if (!erased) {
                all_erased = false;
            }
        } else {
            ESP_LOGE(TAG, "Failed to read %s [0x%08lX]", 
                    location_names[i], test_locations[i]);
        }
    }
    
    if (all_erased) {
        ESP_LOGW(TAG, "=== SUCCESS: Full Chip Erase Complete ===");
        ESP_LOGW(TAG, "All flash memory has been erased!");
        ESP_LOGW(TAG, "APPROTECT has been disabled!");
        ESP_LOGW(TAG, "Device is ready for programming.");
    } else {
        ESP_LOGE(TAG, "=== WARNING: Some areas may not be fully erased ===");
        ESP_LOGE(TAG, "Try power cycling the device and running erase again.");
    }
    
    return all_erased ? ESP_OK : ESP_FAIL;
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