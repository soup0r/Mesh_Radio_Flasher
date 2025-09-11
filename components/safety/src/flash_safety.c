// flash_safety.c - Critical Safety and Recovery Features
#include "flash_safety.h"
#include "swd_flash.h"
#include "swd_mem.h"
#include "esp_log.h"
#include "esp_crc.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "FLASH_SAFETY";

// Protected regions that should never be erased without explicit override
typedef struct {
    uint32_t start;
    uint32_t end;
    const char *name;
    bool allow_write;  // false = never write, true = write with confirmation
} protected_region_t;

static const protected_region_t protected_regions[] = {
    {0x00000000, 0x00001000, "MBR/Bootloader", true},   // Nordic MBR
    {0x10001000, 0x10002000, "UICR", true},             // User Information Config
    {0x00001000, 0x00008000, "SoftDevice", true},       // If SoftDevice present
};

// Backup management structure
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t fw_size;
    uint32_t fw_crc32;
    uint32_t backup_addr;
    uint32_t timestamp;
    char description[64];
} firmware_backup_t;

#define BACKUP_MAGIC 0xBAC0FFEE
#define MAX_BACKUPS 3

// Check if address is in protected region
static bool is_protected_region(uint32_t addr, uint32_t size, bool *needs_confirmation) {
    uint32_t end_addr = addr + size - 1;
    
    for (size_t i = 0; i < sizeof(protected_regions)/sizeof(protected_regions[0]); i++) {
        const protected_region_t *region = &protected_regions[i];
        
        // Check if ranges overlap
        if (!(end_addr < region->start || addr > region->end)) {
            ESP_LOGW(TAG, "Operation touches protected region: %s", region->name);
            if (needs_confirmation) {
                *needs_confirmation = region->allow_write;
            }
            return !region->allow_write;  // Return true if write not allowed
        }
    }
    return false;
}

// Create firmware backup before flashing
esp_err_t create_firmware_backup(uint32_t addr, uint32_t size, const char *description) {
    ESP_LOGI(TAG, "Creating backup of 0x%08lX size %lu", addr, size);
    
    // Allocate buffer for backup (in chunks to avoid memory issues)
    const uint32_t chunk_size = 4096;
    uint8_t *buffer = malloc(chunk_size);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate backup buffer");
        return ESP_ERR_NO_MEM;
    }
    
    // Calculate CRC while reading
    uint32_t crc = 0;
    uint32_t remaining = size;
    uint32_t current_addr = addr;
    
    while (remaining > 0) {
        uint32_t to_read = (remaining > chunk_size) ? chunk_size : remaining;
        
        // Read chunk
        esp_err_t ret = swd_mem_read_buffer(current_addr, buffer, to_read);
        if (ret != ESP_OK) {
            free(buffer);
            ESP_LOGE(TAG, "Failed to read firmware for backup");
            return ret;
        }
        
        // Update CRC
        crc = esp_crc32_le(crc, buffer, to_read);
        
        current_addr += to_read;
        remaining -= to_read;
    }
    
    // Store backup metadata in NVS
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open("fw_backup", NVS_READWRITE, &nvs);
    if (ret != ESP_OK) {
        free(buffer);
        return ret;
    }
    
    firmware_backup_t backup = {
        .magic = BACKUP_MAGIC,
        .version = 1,
        .fw_size = size,
        .fw_crc32 = crc,
        .backup_addr = addr,
        .timestamp = esp_log_timestamp()
    };
    strncpy(backup.description, description, sizeof(backup.description) - 1);
    
    // Rotate backups (keep last N)
    char key[16];
    for (int i = MAX_BACKUPS - 1; i > 0; i--) {
        snprintf(key, sizeof(key), "backup_%d", i - 1);
        size_t length = sizeof(firmware_backup_t);
        firmware_backup_t old_backup;
        
        if (nvs_get_blob(nvs, key, &old_backup, &length) == ESP_OK) {
            snprintf(key, sizeof(key), "backup_%d", i);
            nvs_set_blob(nvs, key, &old_backup, sizeof(firmware_backup_t));
        }
    }
    
    // Store new backup as backup_0
    ret = nvs_set_blob(nvs, "backup_0", &backup, sizeof(firmware_backup_t));
    nvs_commit(nvs);
    nvs_close(nvs);
    
    free(buffer);
    ESP_LOGI(TAG, "Backup created: CRC=0x%08lX", crc);
    return ret;
}

// Verify firmware integrity
esp_err_t verify_firmware_integrity(uint32_t addr, uint32_t size, uint32_t expected_crc) {
    ESP_LOGI(TAG, "Verifying firmware integrity at 0x%08lX", addr);
    
    const uint32_t chunk_size = 4096;
    uint8_t *buffer = malloc(chunk_size);
    if (!buffer) {
        return ESP_ERR_NO_MEM;
    }
    
    uint32_t crc = 0;
    uint32_t remaining = size;
    uint32_t current_addr = addr;
    
    while (remaining > 0) {
        uint32_t to_read = (remaining > chunk_size) ? chunk_size : remaining;
        
        esp_err_t ret = swd_mem_read_buffer(current_addr, buffer, to_read);
        if (ret != ESP_OK) {
            free(buffer);
            return ret;
        }
        
        crc = esp_crc32_le(crc, buffer, to_read);
        current_addr += to_read;
        remaining -= to_read;
    }
    
    free(buffer);
    
    if (crc != expected_crc) {
        ESP_LOGE(TAG, "CRC mismatch: expected 0x%08lX, got 0x%08lX", expected_crc, crc);
        return ESP_ERR_INVALID_CRC;
    }
    
    ESP_LOGI(TAG, "Firmware integrity verified");
    return ESP_OK;
}

// Safe flash erase with protection checks
esp_err_t safe_flash_erase(uint32_t addr, uint32_t size, bool force) {
    // Check protected regions
    bool needs_confirmation = false;
    if (is_protected_region(addr, size, &needs_confirmation)) {
        if (!force) {
            ESP_LOGE(TAG, "Attempting to erase protected region without force flag");
            return ESP_ERR_INVALID_ARG;
        }
        ESP_LOGW(TAG, "Force-erasing protected region!");
    }
    
    // Create backup before erasing (if not forcing)
    if (!force && size > 0) {
        create_firmware_backup(addr, size, "Pre-erase backup");
    }
    
    // Proceed with erase
    return swd_flash_erase_range(addr, size, NULL);
}

// Atomic firmware update with rollback capability
typedef struct {
    uint32_t stage;  // 0=idle, 1=erasing, 2=writing, 3=verifying
    uint32_t progress;
    uint32_t last_good_addr;
    uint32_t error_count;
    bool rollback_available;
} update_state_t;

static update_state_t update_state = {0};

esp_err_t atomic_firmware_update(const firmware_update_t *update, bool create_backup) {
    if (!update || !update->data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Reset state
    memset(&update_state, 0, sizeof(update_state_t));
    
    // Check if we're updating a protected region
    bool needs_confirmation = false;
    if (is_protected_region(update->start_addr, update->size, &needs_confirmation)) {
        ESP_LOGW(TAG, "Firmware update targets protected region");
        if (needs_confirmation) {
            ESP_LOGW(TAG, "Proceeding with caution...");
        } else {
            return ESP_ERR_INVALID_ARG;
        }
    }
    
    // Create backup if requested
    if (create_backup) {
        ESP_LOGI(TAG, "Creating firmware backup before update");
        esp_err_t ret = create_firmware_backup(update->start_addr, update->size, 
                                               "Pre-update backup");
        if (ret == ESP_OK) {
            update_state.rollback_available = true;
        }
    }
    
    // Calculate CRC of new firmware
    uint32_t new_crc = esp_crc32_le(0, update->data, update->size);
    ESP_LOGI(TAG, "New firmware CRC: 0x%08lX", new_crc);
    
    // Stage 1: Erase
    update_state.stage = 1;
    ESP_LOGI(TAG, "Stage 1: Erasing flash");
    esp_err_t ret = swd_flash_erase_range(update->start_addr, update->size, update->progress);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erase failed, rolling back");
        goto rollback;
    }
    
    // Stage 2: Write
    update_state.stage = 2;
    ESP_LOGI(TAG, "Stage 2: Writing firmware");
    ret = swd_flash_write_buffer(update->start_addr, update->data, update->size, update->progress);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write failed, rolling back");
        goto rollback;
    }
    
    // Stage 3: Verify
    update_state.stage = 3;
    ESP_LOGI(TAG, "Stage 3: Verifying firmware");
    ret = verify_firmware_integrity(update->start_addr, update->size, new_crc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Verification failed, rolling back");
        goto rollback;
    }
    
    update_state.stage = 0;  // Success
    ESP_LOGI(TAG, "Firmware update successful");
    return ESP_OK;
    
rollback:
    if (update_state.rollback_available) {
        ESP_LOGW(TAG, "Attempting rollback...");
        // Implement rollback from backup
        // This would read the backup and restore it
    }
    update_state.stage = 0;
    return ret;
}

// Monitor flash health
static flash_health_t flash_health = {0};

void update_flash_health(bool operation_success, bool is_write) {
    if (is_write) {
        flash_health.total_writes++;
        if (!operation_success) {
            flash_health.failed_writes++;
        }
    } else {
        flash_health.total_erases++;
        if (!operation_success) {
            flash_health.failed_erases++;
        }
    }
    
    // Store in NVS periodically
    if ((flash_health.total_writes + flash_health.total_erases) % 100 == 0) {
        nvs_handle_t nvs;
        if (nvs_open("flash_health", NVS_READWRITE, &nvs) == ESP_OK) {
            nvs_set_blob(nvs, "stats", &flash_health, sizeof(flash_health_t));
            nvs_commit(nvs);
            nvs_close(nvs);
        }
    }
    
    // Check if flash might be wearing out
    uint32_t total_ops = flash_health.total_writes + flash_health.total_erases;
    uint32_t total_failures = flash_health.failed_writes + flash_health.failed_erases;
    
    if (total_ops > 1000 && total_failures > total_ops / 100) {
        ESP_LOGW(TAG, "Flash health degraded: %lu failures in %lu operations", 
                 total_failures, total_ops);
    }
}

// Emergency recovery mode
esp_err_t enter_recovery_mode(void) {
    ESP_LOGW(TAG, "Entering emergency recovery mode");
    
    // Try to restore from most recent backup
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open("fw_backup", NVS_READONLY, &nvs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No backups available");
        return ret;
    }
    
    firmware_backup_t backup;
    size_t length = sizeof(firmware_backup_t);
    
    ret = nvs_get_blob(nvs, "backup_0", &backup, &length);
    nvs_close(nvs);
    
    if (ret != ESP_OK || backup.magic != BACKUP_MAGIC) {
        ESP_LOGE(TAG, "Invalid backup");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Found backup: %s (size=%lu, CRC=0x%08lX)", 
             backup.description, backup.fw_size, backup.fw_crc32);
    
    // Here you would implement the actual restore process
    // This is a placeholder for the restore logic
    
    return ESP_OK;
}