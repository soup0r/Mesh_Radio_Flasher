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
    
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms)) {
        if (swd_mem_read32(NVMC_READY, &ready) != ESP_OK) {
            return ESP_FAIL;
        }
        if (ready & 0x1) {
            return ESP_OK;
        }
        vTaskDelay(1);
    }
    
    ESP_LOGE(TAG, "NVMC timeout");
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
        return ESP_ERR_INVALID_ARG;
    }
    
    // Align to page boundary
    addr &= ~(NRF52_FLASH_PAGE_SIZE - 1);
    
    ESP_LOGI(TAG, "Erasing page at 0x%08lX", addr);
    
    // Wait for ready
    esp_err_t ret = wait_nvmc_ready(100);
    if (ret != ESP_OK) return ret;
    
    // Enable erase
    ret = set_nvmc_config(NVMC_CONFIG_EEN);
    if (ret != ESP_OK) return ret;
    
    // Trigger page erase by writing address to ERASEPAGE register
    ret = swd_mem_write32(NVMC_ERASEPAGE, addr);
    if (ret != ESP_OK) goto cleanup;
    
    // Wait for erase completion (page erase takes ~85ms)
    ret = wait_nvmc_ready(200);
    if (ret != ESP_OK) goto cleanup;
    
    // Verify erase (all bytes should be 0xFF)
    uint32_t sample;
    ret = swd_mem_read32(addr, &sample);
    if (ret != ESP_OK) goto cleanup;
    
    if (sample != 0xFFFFFFFF) {
        ESP_LOGE(TAG, "Erase verification failed at 0x%08lX: 0x%08lX", addr, sample);
        ret = ESP_FAIL;
    }
    
cleanup:
    // Return to read-only mode
    set_nvmc_config(NVMC_CONFIG_REN);
    return ret;
}

// Write a single word
esp_err_t swd_flash_write_word(uint32_t addr, uint32_t data) {
    // Validate alignment
    if (addr & 0x3) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Wait for ready
    esp_err_t ret = wait_nvmc_ready(100);
    if (ret != ESP_OK) return ret;
    
    // Enable write
    ret = set_nvmc_config(NVMC_CONFIG_WEN);
    if (ret != ESP_OK) return ret;
    
    // Write the word
    ret = swd_mem_write32(addr, data);
    if (ret != ESP_OK) goto cleanup;
    
    // Wait for write completion
    ret = wait_nvmc_ready(100);
    if (ret != ESP_OK) goto cleanup;
    
    // Verify write
    uint32_t readback;
    ret = swd_mem_read32(addr, &readback);
    if (ret != ESP_OK) goto cleanup;
    
    if (readback != data) {
        ESP_LOGE(TAG, "Write verification failed at 0x%08lX: wrote 0x%08lX, read 0x%08lX", 
                 addr, data, readback);
        ret = ESP_FAIL;
    }
    
cleanup:
    // Return to read-only mode
    set_nvmc_config(NVMC_CONFIG_REN);
    return ret;
}

// Write buffer with progress
esp_err_t swd_flash_write_buffer(uint32_t addr, const uint8_t *data, uint32_t size, 
                                 flash_progress_cb progress) {
    if (!data || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validate address range
    if (addr + size > NRF52_FLASH_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    ESP_LOGI(TAG, "Writing %lu bytes to 0x%08lX", size, addr);
    
    uint32_t written = 0;
    esp_err_t ret = ESP_OK;
    
    // Handle unaligned start
    if (addr & 0x3) {
        uint32_t aligned_addr = addr & ~0x3;
        uint32_t offset = addr & 0x3;
        uint32_t word;
        
        // Read existing word
        ret = swd_mem_read32(aligned_addr, &word);
        if (ret != ESP_OK) return ret;
        
        // Modify bytes
        uint8_t *word_bytes = (uint8_t*)&word;
        uint32_t bytes_to_copy = 4 - offset;
        if (bytes_to_copy > size) bytes_to_copy = size;
        
        for (uint32_t i = 0; i < bytes_to_copy; i++) {
            word_bytes[offset + i] = data[i];
        }
        
        // Write back
        ret = swd_flash_write_word(aligned_addr, word);
        if (ret != ESP_OK) return ret;
        
        addr += bytes_to_copy;
        data += bytes_to_copy;
        written += bytes_to_copy;
        size -= bytes_to_copy;
    }
    
    // Write aligned words
    while (size >= 4) {
        uint32_t word = *(uint32_t*)data;
        
        ret = swd_flash_write_word(addr, word);
        if (ret != ESP_OK) return ret;
        
        addr += 4;
        data += 4;
        written += 4;
        size -= 4;
        
        // Report progress
        if (progress && (written & 0xFF) == 0) {
            progress(written, written + size, "Writing");
        }
    }
    
    // Handle remaining bytes
    if (size > 0) {
        uint32_t word = 0xFFFFFFFF;  // Unprogrammed flash is 0xFF
        uint8_t *word_bytes = (uint8_t*)&word;
        
        for (uint32_t i = 0; i < size; i++) {
            word_bytes[i] = data[i];
        }
        
        ret = swd_flash_write_word(addr, word);
        if (ret != ESP_OK) return ret;
        
        written += size;
    }
    
    if (progress) {
        progress(written, written, "Complete");
    }
    
    ESP_LOGI(TAG, "Write complete: %lu bytes", written);
    return ESP_OK;
}

// Mass erase
esp_err_t swd_flash_mass_erase(flash_progress_cb progress) {
    ESP_LOGW(TAG, "Starting mass erase - this will erase EVERYTHING!");
    
    if (progress) progress(0, 100, "Starting mass erase");
    
    // Wait for ready
    esp_err_t ret = wait_nvmc_ready(100);
    if (ret != ESP_OK) return ret;
    
    // Enable erase
    ret = set_nvmc_config(NVMC_CONFIG_EEN);
    if (ret != ESP_OK) return ret;
    
    // Trigger mass erase
    ret = swd_mem_write32(NVMC_ERASEALL, 0x1);
    if (ret != ESP_OK) goto cleanup;
    
    // Mass erase takes ~40-250ms depending on flash size
    uint32_t elapsed = 0;
    while (elapsed < 500) {
        vTaskDelay(pdMS_TO_TICKS(50));
        elapsed += 50;
        
        if (progress) {
            progress(elapsed / 5, 100, "Erasing");
        }
        
        uint32_t ready;
        if (swd_mem_read32(NVMC_READY, &ready) == ESP_OK && (ready & 0x1)) {
            break;
        }
    }
    
    ret = wait_nvmc_ready(100);
    if (ret != ESP_OK) goto cleanup;
    
    if (progress) progress(100, 100, "Mass erase complete");
    
    ESP_LOGI(TAG, "Mass erase complete");
    
cleanup:
    // Return to read-only mode
    set_nvmc_config(NVMC_CONFIG_REN);
    return ret;
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
    ESP_LOGW(TAG, "Disabling APPROTECT - requires mass erase!");
    
    uint32_t current;
    esp_err_t ret = swd_mem_read32(UICR_APPROTECT, &current);
    if (ret != ESP_OK) return ret;
    
    // Check current value
    if (current == 0xFFFFFF5A) {
        ESP_LOGI(TAG, "APPROTECT already in HwDisabled state");
        return ESP_OK;
    }
    
    // On nRF52840, APPROTECT can only be disabled via mass erase
    ESP_LOGI(TAG, "Performing mass erase to disable APPROTECT...");
    
    // Wait for NVMC ready first
    ret = wait_nvmc_ready(1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVMC not ready before erase");
        return ret;
    }
    
    // Enable erase
    ret = set_nvmc_config(NVMC_CONFIG_EEN);
    if (ret != ESP_OK) return ret;
    
    // Trigger mass erase
    ret = swd_mem_write32(NVMC_ERASEALL, 0x1);
    if (ret != ESP_OK) {
        set_nvmc_config(NVMC_CONFIG_REN);
        return ret;
    }
    
    // Wait for erase (takes 200-300ms)
    ESP_LOGI(TAG, "Erasing chip (takes ~300ms)...");
    ret = wait_nvmc_ready(1000);
    set_nvmc_config(NVMC_CONFIG_REN);
    if (ret != ESP_OK) return ret;
    
    // Now write APPROTECT disable value to UICR
    ESP_LOGI(TAG, "Writing APPROTECT HwDisabled value...");
    ret = wait_nvmc_ready(500);
    if (ret != ESP_OK) return ret;
    
    ret = set_nvmc_config(NVMC_CONFIG_WEN);
    if (ret != ESP_OK) return ret;
    
    ret = swd_mem_write32(UICR_APPROTECT, 0xFFFFFF5A);
    if (ret != ESP_OK) {
        set_nvmc_config(NVMC_CONFIG_REN);
        return ret;
    }
    
    ret = wait_nvmc_ready(500);
    set_nvmc_config(NVMC_CONFIG_REN);
    
    ESP_LOGI(TAG, "Resetting target...");
    swd_reset_target();
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Reconnect
    ret = swd_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reconnect after reset");
        return ret;
    }
    
    // Re-init flash
    ret = swd_flash_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reinit flash after reset");
        return ret;
    }
    
    // Verify
    ret = swd_mem_read32(UICR_APPROTECT, &current);
    if (ret == ESP_OK && current == 0xFFFFFF5A) {
        ESP_LOGI(TAG, "APPROTECT successfully set to HwDisabled");
    } else {
        ESP_LOGE(TAG, "Failed to set APPROTECT (read: 0x%08lX)", current);
        return ESP_FAIL;
    }
    
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