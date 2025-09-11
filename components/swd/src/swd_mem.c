// swd_mem.c - Memory Access Implementation
#include "swd_mem.h"
#include "swd_core.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "SWD_MEM";

// Initialize memory access
esp_err_t swd_mem_init(void) {
    // Select AP 0 (typically AHB-AP)
    esp_err_t ret = swd_dp_write(DP_SELECT, 0x00000000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select AP0");
        return ret;
    }
    
    // Configure CSW for 32-bit auto-increment
    ret = swd_ap_write(AP_CSW, CSW_DEFAULT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CSW");
        return ret;
    }
    
    ESP_LOGI(TAG, "Memory access initialized");
    return ESP_OK;
}

// Read single 32-bit word
esp_err_t swd_mem_read32(uint32_t addr, uint32_t *data) {
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Set target address
    esp_err_t ret = swd_ap_write(AP_TAR, addr);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Initiate read (dummy read to trigger transfer)
    uint32_t dummy;
    ret = swd_ap_read(AP_DRW, &dummy);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Read actual data from RDBUFF
    ret = swd_dp_read(DP_RDBUFF, data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from 0x%08lX", addr);
    }
    
    return ret;
}

// Write single 32-bit word
esp_err_t swd_mem_write32(uint32_t addr, uint32_t data) {
    // Set target address
    esp_err_t ret = swd_ap_write(AP_TAR, addr);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Write data
    ret = swd_ap_write(AP_DRW, data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write 0x%08lX to 0x%08lX", data, addr);
        return ret;
    }
    
    // Read RDBUFF to ensure write completes
    uint32_t dummy;
    ret = swd_dp_read(DP_RDBUFF, &dummy);
    
    return ret;
}

// Read buffer
esp_err_t swd_mem_read_buffer(uint32_t addr, uint8_t *buffer, uint32_t size) {
    if (!buffer || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = ESP_OK;
    
    // Handle unaligned start
    if (addr & 0x3) {
        uint32_t aligned_addr = addr & ~0x3;
        uint32_t offset = addr & 0x3;
        uint32_t word;
        
        ret = swd_mem_read32(aligned_addr, &word);
        if (ret != ESP_OK) return ret;
        
        uint8_t *word_bytes = (uint8_t*)&word;
        uint32_t bytes_to_copy = 4 - offset;
        if (bytes_to_copy > size) bytes_to_copy = size;
        
        memcpy(buffer, &word_bytes[offset], bytes_to_copy);
        
        addr += bytes_to_copy;
        buffer += bytes_to_copy;
        size -= bytes_to_copy;
    }
    
    // Read aligned words
    while (size >= 4) {
        uint32_t word;
        ret = swd_mem_read32(addr, &word);
        if (ret != ESP_OK) return ret;
        
        memcpy(buffer, &word, 4);
        addr += 4;
        buffer += 4;
        size -= 4;
    }
    
    // Handle remaining bytes
    if (size > 0) {
        uint32_t word;
        ret = swd_mem_read32(addr, &word);
        if (ret != ESP_OK) return ret;
        
        memcpy(buffer, &word, size);
    }
    
    return ESP_OK;
}

// Write buffer
esp_err_t swd_mem_write_buffer(uint32_t addr, const uint8_t *buffer, uint32_t size) {
    if (!buffer || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
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
        
        memcpy(&word_bytes[offset], buffer, bytes_to_copy);
        
        // Write back
        ret = swd_mem_write32(aligned_addr, word);
        if (ret != ESP_OK) return ret;
        
        addr += bytes_to_copy;
        buffer += bytes_to_copy;
        size -= bytes_to_copy;
    }
    
    // Write aligned words
    while (size >= 4) {
        uint32_t word;
        memcpy(&word, buffer, 4);
        
        ret = swd_mem_write32(addr, word);
        if (ret != ESP_OK) return ret;
        
        addr += 4;
        buffer += 4;
        size -= 4;
    }
    
    // Handle remaining bytes
    if (size > 0) {
        uint32_t word;
        
        // Read existing word
        ret = swd_mem_read32(addr, &word);
        if (ret != ESP_OK) return ret;
        
        // Modify bytes
        memcpy(&word, buffer, size);
        
        // Write back
        ret = swd_mem_write32(addr, word);
        if (ret != ESP_OK) return ret;
    }
    
    return ESP_OK;
}

// Verify memory
esp_err_t swd_mem_verify(uint32_t addr, const uint8_t *expected, uint32_t size) {
    if (!expected || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Allocate temporary buffer
    uint8_t *read_buffer = malloc(256);
    if (!read_buffer) {
        return ESP_ERR_NO_MEM;
    }
    
    uint32_t verified = 0;
    esp_err_t ret = ESP_OK;
    
    while (verified < size) {
        uint32_t chunk = size - verified;
        if (chunk > 256) chunk = 256;
        
        ret = swd_mem_read_buffer(addr + verified, read_buffer, chunk);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read at 0x%08lX", addr + verified);
            break;
        }
        
        if (memcmp(read_buffer, expected + verified, chunk) != 0) {
            // Find first mismatch
            for (uint32_t i = 0; i < chunk; i++) {
                if (read_buffer[i] != expected[verified + i]) {
                    ESP_LOGE(TAG, "Verify mismatch at 0x%08lX: expected 0x%02X, got 0x%02X",
                            addr + verified + i, expected[verified + i], read_buffer[i]);
                    break;
                }
            }
            ret = ESP_ERR_INVALID_CRC;
            break;
        }
        
        verified += chunk;
    }
    
    free(read_buffer);
    return ret;
}

// Fill memory with pattern
esp_err_t swd_mem_fill(uint32_t addr, uint32_t pattern, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        esp_err_t ret = swd_mem_write32(addr + (i * 4), pattern);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

// Read core register
esp_err_t swd_read_core_register(uint32_t reg, uint32_t *value) {
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Wait for register ready
    uint32_t dhcsr;
    esp_err_t ret = swd_mem_read32(DHCSR_ADDR, &dhcsr);
    if (ret != ESP_OK) return ret;
    
    if (!(dhcsr & DHCSR_S_HALT)) {
        ESP_LOGE(TAG, "Core must be halted to read registers");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Select register
    ret = swd_mem_write32(DCRSR_ADDR, reg);
    if (ret != ESP_OK) return ret;
    
    // Wait for ready
    for (int i = 0; i < 100; i++) {
        ret = swd_mem_read32(DHCSR_ADDR, &dhcsr);
        if (ret != ESP_OK) return ret;
        
        if (dhcsr & DHCSR_S_REGRDY) {
            break;
        }
        vTaskDelay(1);
    }
    
    if (!(dhcsr & DHCSR_S_REGRDY)) {
        ESP_LOGE(TAG, "Register read timeout");
        return ESP_ERR_TIMEOUT;
    }
    
    // Read data
    return swd_mem_read32(DCRDR_ADDR, value);
}

// Write core register
esp_err_t swd_write_core_register(uint32_t reg, uint32_t value) {
    // Wait for register ready
    uint32_t dhcsr;
    esp_err_t ret = swd_mem_read32(DHCSR_ADDR, &dhcsr);
    if (ret != ESP_OK) return ret;
    
    if (!(dhcsr & DHCSR_S_HALT)) {
        ESP_LOGE(TAG, "Core must be halted to write registers");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Write data
    ret = swd_mem_write32(DCRDR_ADDR, value);
    if (ret != ESP_OK) return ret;
    
    // Select register for write (bit 16 = write)
    ret = swd_mem_write32(DCRSR_ADDR, reg | (1 << 16));
    if (ret != ESP_OK) return ret;
    
    // Wait for ready
    for (int i = 0; i < 100; i++) {
        ret = swd_mem_read32(DHCSR_ADDR, &dhcsr);
        if (ret != ESP_OK) return ret;
        
        if (dhcsr & DHCSR_S_REGRDY) {
            return ESP_OK;
        }
        vTaskDelay(1);
    }
    
    ESP_LOGE(TAG, "Register write timeout");
    return ESP_ERR_TIMEOUT;
}

// Halt core
esp_err_t swd_halt_core(void) {
    // Enable debug and request halt
    esp_err_t ret = swd_mem_write32(DHCSR_ADDR, DHCSR_DBGKEY | DHCSR_C_DEBUGEN | DHCSR_C_HALT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to halt core");
        return ret;
    }
    
    // Wait for halt
    for (int i = 0; i < 100; i++) {
        uint32_t dhcsr;
        ret = swd_mem_read32(DHCSR_ADDR, &dhcsr);
        if (ret != ESP_OK) return ret;
        
        if (dhcsr & DHCSR_S_HALT) {
            ESP_LOGI(TAG, "Core halted");
            return ESP_OK;
        }
        vTaskDelay(1);
    }
    
    ESP_LOGE(TAG, "Core halt timeout");
    return ESP_ERR_TIMEOUT;
}

// Resume core
esp_err_t swd_resume_core(void) {
    // Keep debug enabled but clear halt
    esp_err_t ret = swd_mem_write32(DHCSR_ADDR, DHCSR_DBGKEY | DHCSR_C_DEBUGEN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to resume core");
        return ret;
    }
    
    // Verify resumed
    uint32_t dhcsr;
    ret = swd_mem_read32(DHCSR_ADDR, &dhcsr);
    if (ret != ESP_OK) return ret;
    
    if (dhcsr & DHCSR_S_HALT) {
        ESP_LOGE(TAG, "Core still halted");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Core resumed");
    return ESP_OK;
}

// Check if core is halted
bool swd_is_halted(void) {
    uint32_t dhcsr;
    esp_err_t ret = swd_mem_read32(DHCSR_ADDR, &dhcsr);
    if (ret != ESP_OK) {
        return false;
    }
    
    return (dhcsr & DHCSR_S_HALT) != 0;
}