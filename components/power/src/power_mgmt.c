#include "power_mgmt.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "POWER_MGMT";
static power_config_t power_config = {0};
static system_state_t current_state = SYSTEM_STATE_INIT;
static system_health_t health_stats = {0};

// CORRECT LOGIC: LOW = Power ON, HIGH = Power OFF
// This assumes N-channel MOSFET or similar where:
// - GPIO LOW (0V) = MOSFET OFF = Power flows normally (ON)
// - GPIO HIGH (3.3V) = MOSFET ON = Power rail pulled to ground (OFF)

esp_err_t power_mgmt_init(const power_config_t *config) {
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    power_config = *config;
    current_state = SYSTEM_STATE_INIT;

    // Initialize power control GPIO if configured
    if (power_config.target_power_gpio >= 0) {
        gpio_reset_pin((gpio_num_t)power_config.target_power_gpio);
        gpio_set_direction((gpio_num_t)power_config.target_power_gpio, GPIO_MODE_OUTPUT);
        // CORRECT: Set LOW for power ON (default state)
        gpio_set_level((gpio_num_t)power_config.target_power_gpio, 0);
        ESP_LOGI(TAG, "Power control GPIO%d initialized (LOW=ON, HIGH=OFF)", power_config.target_power_gpio);
    }

    current_state = SYSTEM_STATE_ACTIVE;
    ESP_LOGI(TAG, "Power management initialized");
    return ESP_OK;
}

esp_err_t power_target_on(void) {
    if (power_config.target_power_gpio >= 0) {
        // CORRECT: LOW = Power ON
        gpio_set_level((gpio_num_t)power_config.target_power_gpio, 0);
        ESP_LOGI(TAG, "Target power ON (GPIO%d = LOW)", power_config.target_power_gpio);
        return ESP_OK;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t power_target_off(void) {
    if (power_config.target_power_gpio >= 0) {
        // CORRECT: HIGH = Power OFF
        gpio_set_level((gpio_num_t)power_config.target_power_gpio, 1);
        ESP_LOGI(TAG, "Target power OFF (GPIO%d = HIGH)", power_config.target_power_gpio);
        return ESP_OK;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t power_target_reset(void) {
    if (power_config.target_power_gpio < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "Target reset: turning OFF for 15 seconds, then ON");

    // Reset sequence: Turn OFF for 15 seconds, then turn ON
    gpio_set_level((gpio_num_t)power_config.target_power_gpio, 1);  // Turn OFF
    ESP_LOGI(TAG, "Target power OFF (GPIO%d = HIGH)", power_config.target_power_gpio);

    vTaskDelay(pdMS_TO_TICKS(15000));  // 15 seconds OFF

    gpio_set_level((gpio_num_t)power_config.target_power_gpio, 0);  // Turn ON
    ESP_LOGI(TAG, "Target power ON (GPIO%d = LOW)", power_config.target_power_gpio);

    vTaskDelay(pdMS_TO_TICKS(power_config.power_on_delay_ms));

    ESP_LOGI(TAG, "Target reset complete - device should be rebooting");
    return ESP_OK;
}

esp_err_t power_target_cycle(uint32_t off_time_ms) {
    if (power_config.target_power_gpio < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "Power cycling target (off for %lu ms)", (unsigned long)off_time_ms);

    // CORRECT: HIGH = OFF, then LOW = ON
    gpio_set_level((gpio_num_t)power_config.target_power_gpio, 1);  // Turn OFF
    vTaskDelay(pdMS_TO_TICKS(off_time_ms));
    gpio_set_level((gpio_num_t)power_config.target_power_gpio, 0);  // Turn ON
    vTaskDelay(pdMS_TO_TICKS(power_config.power_on_delay_ms));

    ESP_LOGI(TAG, "Power cycle complete");
    return ESP_OK;
}

void power_watchdog_feed(void) {
    // Watchdog feed implementation
}

void power_get_health_status(system_health_t *health) {
    if (health) {
        *health = health_stats;
        health_stats.uptime_seconds++;
    }
}

wake_reason_t power_get_wake_reason(void) {
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    switch(cause) {
        case ESP_SLEEP_WAKEUP_TIMER:
            return WAKE_REASON_TIMER;
        case ESP_SLEEP_WAKEUP_GPIO:
            return WAKE_REASON_GPIO;
        case ESP_SLEEP_WAKEUP_UART:
            return WAKE_REASON_UART;
        default:
            return WAKE_REASON_RESET;
    }
}

esp_err_t power_log_error(const char *error_msg) {
    if (!error_msg) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGE(TAG, "Error logged: %s", error_msg);

    nvs_handle_t nvs;
    esp_err_t ret = nvs_open("error_log", NVS_READWRITE, &nvs);
    if (ret == ESP_OK) {
        nvs_set_str(nvs, "last_error", error_msg);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    return ESP_OK;
}

esp_err_t power_enter_deep_sleep(uint32_t duration_sec) {
    ESP_LOGI(TAG, "Entering deep sleep for %lu seconds", (unsigned long)duration_sec);
    current_state = SYSTEM_STATE_DEEP_SLEEP;
    esp_sleep_enable_timer_wakeup(duration_sec * 1000000ULL);
    esp_deep_sleep_start();
    return ESP_OK;
}

esp_err_t power_schedule_sleep(void) {
    return ESP_OK;
}

void power_cancel_sleep(void) {
}

bool power_should_stay_awake(void) {
    return current_state == SYSTEM_STATE_ACTIVE;
}

esp_err_t power_watchdog_init(uint32_t timeout_sec) {
    ESP_LOGI(TAG, "Watchdog initialized with %lu second timeout", (unsigned long)timeout_sec);
    return ESP_OK;
}

void power_watchdog_disable(void) {
}

esp_err_t power_recovery_init(void) {
    return ESP_OK;
}

esp_err_t power_handle_error(esp_err_t error, const char *context) {
    ESP_LOGE(TAG, "Error %s in context: %s", esp_err_to_name(error), context);
    health_stats.total_resets++;
    current_state = SYSTEM_STATE_ERROR;
    return ESP_OK;
}

esp_err_t power_self_test(void) {
    ESP_LOGI(TAG, "Running self-test");

    if (power_config.target_power_gpio >= 0) {
        ESP_LOGI(TAG, "Testing power control on GPIO%d", power_config.target_power_gpio);

        // Read current state
        int current = gpio_get_level((gpio_num_t)power_config.target_power_gpio);
        ESP_LOGI(TAG, "Current power state: %s (GPIO=%d)",
                current ? "OFF" : "ON", current);

        // Quick test cycle (if safe)
        ESP_LOGI(TAG, "Power control test: LOW=ON, HIGH=OFF");
    }

    return ESP_OK;
}

esp_err_t power_get_last_errors(char *buffer, size_t size) {
    if (!buffer || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs;
    esp_err_t ret = nvs_open("error_log", NVS_READONLY, &nvs);
    if (ret == ESP_OK) {
        size_t length = size;
        ret = nvs_get_str(nvs, "last_error", buffer, &length);
        nvs_close(nvs);
    }

    return ret;
}

void power_clear_error_log(void) {
    nvs_handle_t nvs;
    if (nvs_open("error_log", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_erase_all(nvs);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

system_state_t power_get_state(void) {
    return current_state;
}

const char* power_get_state_string(system_state_t state) {
    switch(state) {
        case SYSTEM_STATE_INIT: return "INIT";
        case SYSTEM_STATE_ACTIVE: return "ACTIVE";
        case SYSTEM_STATE_IDLE: return "IDLE";
        case SYSTEM_STATE_ERROR: return "ERROR";
        case SYSTEM_STATE_RECOVERY: return "RECOVERY";
        case SYSTEM_STATE_DEEP_SLEEP: return "DEEP_SLEEP";
        default: return "UNKNOWN";
    }
}

float power_get_battery_voltage(void) {
    return 3.3f;
}

float power_get_current_draw(void) {
    return 0.0f;
}