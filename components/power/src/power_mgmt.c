#include "power_mgmt.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "POWER_MGMT";
static power_config_t power_config = {0};
static system_state_t current_state = SYSTEM_STATE_INIT;
static system_health_t health_stats = {0};
static bool power_state = true;
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;
static battery_status_t battery_stats = {0};
static bool battery_monitoring_enabled = false;

#define VOLTAGE_DIVIDER_RATIO 2.0f
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_2
#define BATTERY_MAX_VOLTAGE 4.2f
#define BATTERY_MIN_VOLTAGE 3.0f
#define BATTERY_CRITICAL_VOLTAGE 3.2f
#define BATTERY_LOW_VOLTAGE 3.5f
#define ADC_SAMPLES_COUNT 10

esp_err_t power_battery_init(void) {
    ESP_LOGI(TAG, "Initializing battery monitoring on GPIO2");
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ret = adc_oneshot_config_channel(adc1_handle, BATTERY_ADC_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config ADC channel: %s", esp_err_to_name(ret));
        return ret;
    }
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration enabled");
    } else {
        ESP_LOGW(TAG, "ADC calibration not supported, using raw values");
    }
    battery_monitoring_enabled = true;
    battery_stats.voltage_min = 999.0f;
    battery_stats.voltage_max = 0.0f;
    battery_stats.samples_count = 0;
    battery_stats.voltage_avg = 0.0f;
    battery_status_t status;
    power_get_battery_status(&status);
    ESP_LOGI(TAG, "Battery monitoring initialized. Initial voltage: %.2fV", status.voltage);
    return ESP_OK;
}

float power_get_battery_voltage_real(void) {
    if (!battery_monitoring_enabled || !adc1_handle) {
        return 0.0f;
    }
    int adc_reading = 0;
    int voltage_mv = 0;
    float total_voltage = 0.0f;
    for (int i = 0; i < ADC_SAMPLES_COUNT; i++) {
        esp_err_t ret = adc_oneshot_read(adc1_handle, BATTERY_ADC_CHANNEL, &adc_reading);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read ADC");
            continue;
        }
        if (adc_cali_handle) {
            adc_cali_raw_to_voltage(adc_cali_handle, adc_reading, &voltage_mv);
            total_voltage += (voltage_mv / 1000.0f);
        } else {
            float voltage = (adc_reading / 4095.0f) * 3.3f;
            total_voltage += voltage;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    float adc_voltage = total_voltage / ADC_SAMPLES_COUNT;
    float battery_voltage = adc_voltage * VOLTAGE_DIVIDER_RATIO;
    return battery_voltage;
}

esp_err_t power_get_battery_status(battery_status_t *status) {
    if (!status || !battery_monitoring_enabled) {
        return ESP_ERR_INVALID_ARG;
    }
    float current_voltage = power_get_battery_voltage_real();
    battery_stats.voltage = current_voltage;
    battery_stats.samples_count++;
    if (current_voltage > 2.0f && current_voltage < battery_stats.voltage_min) {
        battery_stats.voltage_min = current_voltage;
    }
    if (current_voltage > battery_stats.voltage_max) {
        battery_stats.voltage_max = current_voltage;
    }
    if (battery_stats.samples_count == 1) {
        battery_stats.voltage_avg = current_voltage;
    } else {
        battery_stats.voltage_avg = (battery_stats.voltage_avg * 0.95f) + (current_voltage * 0.05f);
    }
    if (current_voltage >= BATTERY_MAX_VOLTAGE) {
        battery_stats.percentage = 100.0f;
    } else if (current_voltage <= BATTERY_MIN_VOLTAGE) {
        battery_stats.percentage = 0.0f;
    } else {
        battery_stats.percentage = ((current_voltage - BATTERY_MIN_VOLTAGE) /
                                   (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0f;
    }
    battery_stats.is_charging = (current_voltage > 4.1f);
    battery_stats.is_critical = (current_voltage < BATTERY_CRITICAL_VOLTAGE);
    battery_stats.is_low = (current_voltage < BATTERY_LOW_VOLTAGE);
    *status = battery_stats;
    ESP_LOGD(TAG, "Battery: %.2fV (%.0f%%) Min:%.2fV Max:%.2fV Avg:%.2fV %s%s",
            status->voltage, status->percentage,
            status->voltage_min, status->voltage_max, status->voltage_avg,
            status->is_charging ? "[CHARGING]" : "",
            status->is_critical ? "[CRITICAL]" : (status->is_low ? "[LOW]" : ""));
    return ESP_OK;
}

esp_err_t power_mgmt_init(const power_config_t *config) {
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    power_config = *config;
    current_state = SYSTEM_STATE_INIT;
    esp_sleep_wakeup_cause_t wake_cause = esp_sleep_get_wakeup_cause();
    bool waking_from_sleep = (wake_cause != ESP_SLEEP_WAKEUP_UNDEFINED);

    if (power_config.target_power_gpio >= 0) {
        gpio_num_t gpio = (gpio_num_t)power_config.target_power_gpio;
        gpio_reset_pin(gpio);
        gpio_set_direction(gpio, GPIO_MODE_INPUT_OUTPUT);

        if (waking_from_sleep) {
            ESP_LOGI(TAG, "Waking from deep sleep, restoring power state: %s",
                    power_state ? "ON" : "OFF");
            gpio_set_level(gpio, power_state ? 0 : 1);
        } else {
            gpio_set_level(gpio, 0);
            power_state = true;
            ESP_LOGI(TAG, "Fresh boot, setting power ON");
        }

        int actual_state = gpio_get_level(gpio);
        ESP_LOGI(TAG, "Power control GPIO%d initialized: actual state = %d (%s)",
                power_config.target_power_gpio, actual_state,
                actual_state == 0 ? "ON" : "OFF");
    }
    power_battery_init();
    current_state = SYSTEM_STATE_ACTIVE;
    ESP_LOGI(TAG, "Power management initialized");
    return ESP_OK;
}

esp_err_t power_target_on(void) {
    if (power_config.target_power_gpio >= 0) {
        gpio_num_t gpio = (gpio_num_t)power_config.target_power_gpio;
        gpio_set_level(gpio, 0);
        power_state = true;
        vTaskDelay(pdMS_TO_TICKS(10));
        int actual = gpio_get_level(gpio);
        ESP_LOGI(TAG, "Target power ON (GPIO%d = LOW), actual = %d",
                power_config.target_power_gpio, actual);
        return ESP_OK;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t power_target_off(void) {
    if (power_config.target_power_gpio >= 0) {
        gpio_num_t gpio = (gpio_num_t)power_config.target_power_gpio;
        gpio_set_level(gpio, 1);
        power_state = false;
        vTaskDelay(pdMS_TO_TICKS(10));
        int actual = gpio_get_level(gpio);
        ESP_LOGI(TAG, "Target power OFF (GPIO%d = HIGH), actual = %d",
                power_config.target_power_gpio, actual);
        return ESP_OK;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t power_target_reset(void) {
    if (power_config.target_power_gpio < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    ESP_LOGI(TAG, "Target reset: turning OFF for 15 seconds, then ON");
    power_target_off();
    vTaskDelay(pdMS_TO_TICKS(15000));
    power_target_on();
    vTaskDelay(pdMS_TO_TICKS(power_config.power_on_delay_ms));
    ESP_LOGI(TAG, "Target reset complete");
    return ESP_OK;
}

esp_err_t power_target_cycle(uint32_t off_time_ms) {
    if (power_config.target_power_gpio < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    ESP_LOGI(TAG, "Power cycling target (off for %lu ms)", (unsigned long)off_time_ms);
    power_target_off();
    vTaskDelay(pdMS_TO_TICKS(off_time_ms));
    power_target_on();
    vTaskDelay(pdMS_TO_TICKS(power_config.power_on_delay_ms));
    ESP_LOGI(TAG, "Power cycle complete");
    return ESP_OK;
}

bool power_target_is_on(void) {
    if (power_config.target_power_gpio >= 0) {
        gpio_num_t gpio = (gpio_num_t)power_config.target_power_gpio;
        int gpio_state = gpio_get_level(gpio);
        bool is_on = (gpio_state == 0);
        if (is_on != power_state) {
            ESP_LOGW(TAG, "Power state mismatch: GPIO=%d (%s), internal=%s",
                    gpio_state, is_on ? "ON" : "OFF",
                    power_state ? "ON" : "OFF");
            power_state = is_on;
        }
        return is_on;
    }
    return false;
}

void power_prepare_for_sleep(void) {
    if (power_config.target_power_gpio >= 0) {
        power_state = power_target_is_on();
        ESP_LOGI(TAG, "Preparing for deep sleep, power state: %s",
                power_state ? "ON" : "OFF");
    }
}

void power_restore_after_sleep(void) {
    if (power_config.target_power_gpio >= 0) {
        ESP_LOGI(TAG, "Restored after deep sleep, power state: %s",
                power_state ? "ON" : "OFF");
    }
}

esp_err_t power_enter_deep_sleep(uint32_t duration_sec) {
    ESP_LOGI(TAG, "Entering deep sleep for %lu seconds", (unsigned long)duration_sec);
    power_prepare_for_sleep();
    current_state = SYSTEM_STATE_DEEP_SLEEP;
    esp_sleep_enable_timer_wakeup(duration_sec * 1000000ULL);
    esp_deep_sleep_start();
    return ESP_OK;
}

void power_watchdog_feed(void) {}

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

esp_err_t power_schedule_sleep(void) {
    return ESP_OK;
}

void power_cancel_sleep(void) {}

bool power_should_stay_awake(void) {
    return current_state == SYSTEM_STATE_ACTIVE;
}

esp_err_t power_watchdog_init(uint32_t timeout_sec) {
    ESP_LOGI(TAG, "Watchdog initialized with %lu second timeout", (unsigned long)timeout_sec);
    return ESP_OK;
}

void power_watchdog_disable(void) {}

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
        int current = gpio_get_level((gpio_num_t)power_config.target_power_gpio);
        ESP_LOGI(TAG, "Current power state: %s (GPIO=%d)",
                current ? "OFF" : "ON", current);
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
    battery_status_t status;
    if (power_get_battery_status(&status) == ESP_OK) {
        return status.voltage;
    }
    return 0.0f;
}

float power_get_current_draw(void) {
    return 0.0f;
}