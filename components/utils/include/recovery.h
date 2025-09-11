#ifndef RECOVERY_H
#define RECOVERY_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Recovery triggers
typedef enum {
    RECOVERY_TRIGGER_NONE = 0,
    RECOVERY_TRIGGER_BUTTON,
    RECOVERY_TRIGGER_WATCHDOG,
    RECOVERY_TRIGGER_PANIC,
    RECOVERY_TRIGGER_CORRUPT_CONFIG,
    RECOVERY_TRIGGER_FLASH_FAILURE,
    RECOVERY_TRIGGER_USER_REQUEST
} recovery_trigger_t;

// Recovery actions
typedef enum {
    RECOVERY_ACTION_RESTART = 0,
    RECOVERY_ACTION_FACTORY_RESET,
    RECOVERY_ACTION_SAFE_MODE,
    RECOVERY_ACTION_ROLLBACK,
    RECOVERY_ACTION_WAIT_CONFIG
} recovery_action_t;

// Initialize recovery system
esp_err_t recovery_init(void);

// Check for recovery conditions
recovery_trigger_t recovery_check_trigger(void);

// Execute recovery
esp_err_t recovery_execute(recovery_action_t action);

// Error logging (survives reboot)
esp_err_t recovery_log_error(const char *context, esp_err_t error);
esp_err_t recovery_get_last_errors(char *buffer, size_t size);
void recovery_clear_errors(void);

// Self-test
esp_err_t recovery_self_test(void);

// Panic handler
void recovery_panic_handler(const char *reason);

#endif // RECOVERY_H