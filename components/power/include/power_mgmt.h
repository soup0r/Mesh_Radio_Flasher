// power_mgmt.h - Power and Recovery Management
#ifndef POWER_MGMT_H
#define POWER_MGMT_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Power management configuration
typedef struct {
    // Target control
    int target_power_gpio;      // GPIO for MOSFET/transistor control
    uint32_t power_on_delay_ms; // Delay after power on
    uint32_t reset_hold_ms;     // Reset pulse duration
    
    // Deep sleep settings
    uint32_t sleep_duration_sec;    // Deep sleep duration
    uint32_t wifi_check_interval_ms; // How often to check WiFi in active mode
    uint32_t wifi_timeout_ms;       // Max time to wait for WiFi connection
    const char *wake_ssid;          // SSID to check for wake condition
    
    // Watchdog settings
    uint32_t watchdog_timeout_sec;  // Hardware watchdog timeout
    bool enable_brownout_detect;    // Enable brownout detection
    
    // Recovery settings
    uint32_t max_retry_count;       // Max retries before deep sleep
    uint32_t error_cooldown_ms;     // Delay between error retries
} power_config_t;

// System state
typedef enum {
    SYSTEM_STATE_INIT,
    SYSTEM_STATE_ACTIVE,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_ERROR,
    SYSTEM_STATE_RECOVERY,
    SYSTEM_STATE_DEEP_SLEEP
} system_state_t;

// Initialize power management
esp_err_t power_mgmt_init(const power_config_t *config);

// Target power control
esp_err_t power_target_on(void);
esp_err_t power_target_off(void);
esp_err_t power_target_reset(void);
esp_err_t power_target_cycle(uint32_t off_time_ms);

// Deep sleep management
esp_err_t power_enter_deep_sleep(uint32_t duration_sec);
esp_err_t power_schedule_sleep(void);
void power_cancel_sleep(void);
bool power_should_stay_awake(void);

// Wake reason
typedef enum {
    WAKE_REASON_TIMER,
    WAKE_REASON_GPIO,
    WAKE_REASON_UART,
    WAKE_REASON_RESET,
    WAKE_REASON_UNKNOWN
} wake_reason_t;

wake_reason_t power_get_wake_reason(void);

// Watchdog management
esp_err_t power_watchdog_init(uint32_t timeout_sec);
void power_watchdog_feed(void);
void power_watchdog_disable(void);

// Recovery mechanisms
typedef struct {
    uint32_t swd_failures;
    uint32_t flash_failures;
    uint32_t network_failures;
    uint32_t watchdog_resets;
    uint32_t brownout_resets;
    uint32_t total_resets;
    uint32_t uptime_seconds;
} system_health_t;

esp_err_t power_recovery_init(void);
esp_err_t power_handle_error(esp_err_t error, const char *context);
void power_get_health_status(system_health_t *health);
esp_err_t power_self_test(void);

// Persistent error logging (survives deep sleep)
esp_err_t power_log_error(const char *error_msg);
esp_err_t power_get_last_errors(char *buffer, size_t size);
void power_clear_error_log(void);

// State management
system_state_t power_get_state(void);
const char* power_get_state_string(system_state_t state);

// Energy monitoring (if available on your board)
float power_get_battery_voltage(void);
float power_get_current_draw(void);

#endif // POWER_MGMT_H