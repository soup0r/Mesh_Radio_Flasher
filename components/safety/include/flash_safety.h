#ifndef FLASH_SAFETY_H
#define FLASH_SAFETY_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "swd_flash.h"

// Backup operations
esp_err_t create_firmware_backup(uint32_t addr, uint32_t size, const char *description);
esp_err_t restore_firmware_backup(uint32_t backup_index);
esp_err_t list_firmware_backups(void);

// Integrity checks
esp_err_t verify_firmware_integrity(uint32_t addr, uint32_t size, uint32_t expected_crc);
uint32_t calculate_firmware_crc(uint32_t addr, uint32_t size);

// Safe operations
esp_err_t safe_flash_erase(uint32_t addr, uint32_t size, bool force);
esp_err_t safe_flash_write(uint32_t addr, const uint8_t *data, uint32_t size);
esp_err_t atomic_firmware_update(const firmware_update_t *update, bool create_backup);

// Recovery
esp_err_t enter_recovery_mode(void);
esp_err_t check_recovery_trigger(void);

// Flash health monitoring
typedef struct {
    uint32_t total_writes;
    uint32_t failed_writes;
    uint32_t total_erases;
    uint32_t failed_erases;
    uint32_t verify_failures;
} flash_health_t;

void update_flash_health(bool operation_success, bool is_write);
void get_flash_health(flash_health_t *health);

#endif // FLASH_SAFETY_H