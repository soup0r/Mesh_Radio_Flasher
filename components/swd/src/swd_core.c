// swd_core.c - Complete SWD Protocol Implementation
#include "swd_core.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SWD_CORE";

// GPIO fast access macros
#define SWCLK_MASK (1UL << config.pin_swclk)
#define SWDIO_MASK (1UL << config.pin_swdio)

#define SWCLK_H()    GPIO.out_w1ts = SWCLK_MASK
#define SWCLK_L()    GPIO.out_w1tc = SWCLK_MASK
#define SWDIO_H()    GPIO.out_w1ts = SWDIO_MASK
#define SWDIO_L()    GPIO.out_w1tc = SWDIO_MASK
#define SWDIO_DRIVE()   GPIO.enable_w1ts = SWDIO_MASK
#define SWDIO_RELEASE() GPIO.enable_w1tc = SWDIO_MASK
#define READ_SWDIO()    ((GPIO.in & SWDIO_MASK) != 0)

// Configuration storage
static swd_config_t config = {0};
static bool initialized = false;
static bool connected = false;
static bool drive_phase = true;
static portMUX_TYPE swd_mutex = portMUX_INITIALIZER_UNLOCKED;

// Timing delay
static inline void swd_delay(void) {
    for (int i = 0; i < config.delay_cycles; i++) {
        __asm__ __volatile__("nop");
    }
}

// Clock pulse
static inline void clock_pulse(void) {
    SWCLK_H();
    swd_delay();
    SWCLK_L();
    swd_delay();
}

// Calculate parity
static inline bool parity32(uint32_t x) {
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    x &= 0xF;
    return (0x6996 >> x) & 1;
}

// Turn-around cycle
static void swd_turnaround(bool to_write) {
    if (to_write) {
        SWDIO_H();
        SWDIO_RELEASE();
        clock_pulse();
        SWDIO_DRIVE();
    } else {
        SWDIO_H();
        SWDIO_RELEASE();
        clock_pulse();
    }
    drive_phase = to_write;
}

// Write bits LSB first
static void write_bits(uint32_t value, uint8_t count) {
    if (!drive_phase) {
        swd_turnaround(true);
    }
    
    while (count--) {
        if (value & 1) {
            SWDIO_H();
        } else {
            SWDIO_L();
        }
        clock_pulse();
        value >>= 1;
    }
}

// Read bits LSB first
static uint32_t read_bits(uint8_t count) {
    if (drive_phase) {
        swd_turnaround(false);
    }
    
    uint32_t result = 0;
    uint32_t bit = 1;
    
    while (count--) {
        if (READ_SWDIO()) {
            result |= bit;
        }
        clock_pulse();
        bit <<= 1;
    }
    
    return result;
}

// Send SWD request
static void send_request(uint8_t addr, bool ap, bool read) {
    uint8_t request = 0x81;  // Start bit and park bit
    
    if (ap) request |= (1 << 1);
    if (read) request |= (1 << 2);
    
    request |= (addr & 0x0C) << 1;
    
    // Calculate parity
    bool parity = ap ^ read ^ ((addr >> 2) & 1) ^ ((addr >> 3) & 1);
    if (parity) request |= (1 << 5);
    
    write_bits(request, 8);
}

// Write parking bit
static void write_parking(void) {
    if (!drive_phase) {
        swd_turnaround(true);
    }
    SWDIO_L();
    clock_pulse();
}

// Line reset (50+ clocks high)
static void line_reset(void) {
    SWDIO_DRIVE();
    SWDIO_H();
    for (int i = 0; i < 60; i++) {
        clock_pulse();
    }
    SWDIO_L();
    clock_pulse();
}

// JTAG to SWD sequence
static void jtag_to_swd(void) {
    SWDIO_DRIVE();
    
    // Send magic sequence
    uint32_t sequence = 0xE79E;  // 16-bit JTAG-to-SWD
    for (int i = 0; i < 16; i++) {
        if (sequence & (1 << i)) {
            SWDIO_H();
        } else {
            SWDIO_L();
        }
        clock_pulse();
    }
    
    // Additional reset
    line_reset();
}

// Dormant wakeup sequence
static void dormant_wakeup(void) {
    SWDIO_DRIVE();
    
    // 8 cycles high
    SWDIO_H();
    for (int i = 0; i < 8; i++) {
        clock_pulse();
    }
    
    // 128-bit selection alert sequence
    uint32_t alert[4] = {0x49CF9046, 0xA9B4A161, 0x97F5BBC7, 0x45703D98};
    
    for (int w = 0; w < 4; w++) {
        uint32_t word = alert[w];
        for (int b = 31; b >= 0; b--) {
            if (word & (1UL << b)) {
                SWDIO_H();
            } else {
                SWDIO_L();
            }
            clock_pulse();
        }
    }
    
    // 4 cycles low
    SWDIO_L();
    for (int i = 0; i < 4; i++) {
        clock_pulse();
    }
    
    // Activation code (0x58 for SWD)
    uint8_t activation = 0x58;
    for (int i = 7; i >= 0; i--) {
        if (activation & (1 << i)) {
            SWDIO_H();
        } else {
            SWDIO_L();
        }
        clock_pulse();
    }
    
    // Line reset
    line_reset();
}

// Initialize SWD interface
esp_err_t swd_init(const swd_config_t *cfg) {
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }
    
    config = *cfg;
    
    // Initialize GPIOs
    gpio_reset_pin((gpio_num_t)config.pin_swclk);
    gpio_reset_pin((gpio_num_t)config.pin_swdio);
    
    gpio_set_direction((gpio_num_t)config.pin_swclk, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)config.pin_swdio, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_pull_mode((gpio_num_t)config.pin_swdio, GPIO_PULLUP_ONLY);
    
    SWCLK_L();
    SWDIO_H();
    SWDIO_DRIVE();
    
    // Optional reset pin
    if (config.pin_reset >= 0) {
        gpio_reset_pin((gpio_num_t)config.pin_reset);
        gpio_set_direction((gpio_num_t)config.pin_reset, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)config.pin_reset, 1);
    }
    
    drive_phase = true;
    initialized = true;
    connected = false;
    
    ESP_LOGI(TAG, "SWD initialized: SWCLK=%d, SWDIO=%d, nRST=%d",
             config.pin_swclk, config.pin_swdio, config.pin_reset);
    
    return ESP_OK;
}

// Raw SWD transfer
swd_ack_t swd_transfer_raw(uint8_t addr, bool ap, bool read, uint32_t *data) {
    portENTER_CRITICAL(&swd_mutex);
    
    // Send request
    send_request(addr, ap, read);
    
    // Read ACK
    if (drive_phase) {
        swd_turnaround(false);
    }
    
    uint8_t ack = (uint8_t)read_bits(3);
    
    if (ack == SWD_ACK_OK) {
        if (read) {
            // Read data and parity
            uint32_t value = read_bits(32);
            uint8_t parity_bit = (uint8_t)read_bits(1);
            
            // Turnaround to write
            swd_turnaround(true);
            write_parking();
            
            portEXIT_CRITICAL(&swd_mutex);
            
            // Verify parity
            if (parity_bit != parity32(value)) {
                ESP_LOGW(TAG, "Parity error on read");
                return SWD_ACK_FAULT;
            }
            
            *data = value;
        } else {
            // Turnaround to write
            swd_turnaround(true);
            
            // Write data and parity
            write_bits(*data, 32);
            write_bits(parity32(*data), 1);
            write_parking();
            
            portEXIT_CRITICAL(&swd_mutex);
        }
    } else {
        // Error - send dummy clocks
        swd_turnaround(true);
        write_bits(0, 32);
        write_parking();
        
        portEXIT_CRITICAL(&swd_mutex);
    }
    
    return (swd_ack_t)ack;
}

// DP read with retry
esp_err_t swd_dp_read(uint8_t addr, uint32_t *data) {
    if (!initialized || !data) {
        return ESP_ERR_INVALID_STATE;
    }
    
    for (int retry = 0; retry < 10; retry++) {
        swd_ack_t ack = swd_transfer_raw(addr, false, true, data);
        
        if (ack == SWD_ACK_OK) {
            return ESP_OK;
        } else if (ack == SWD_ACK_WAIT) {
            vTaskDelay(1);
        } else if (ack == SWD_ACK_FAULT) {
            // Clear sticky error
            swd_clear_errors();
        }
    }
    
    ESP_LOGE(TAG, "DP read failed: addr=0x%02X", addr);
    return ESP_FAIL;
}

// DP write with retry
esp_err_t swd_dp_write(uint8_t addr, uint32_t data) {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    for (int retry = 0; retry < 10; retry++) {
        swd_ack_t ack = swd_transfer_raw(addr, false, false, &data);
        
        if (ack == SWD_ACK_OK) {
            return ESP_OK;
        } else if (ack == SWD_ACK_WAIT) {
            vTaskDelay(1);
        } else if (ack == SWD_ACK_FAULT) {
            swd_clear_errors();
        }
    }
    
    ESP_LOGE(TAG, "DP write failed: addr=0x%02X data=0x%08lX", addr, data);
    return ESP_FAIL;
}

// AP read with retry
esp_err_t swd_ap_read(uint8_t addr, uint32_t *data) {
    if (!initialized || !data) {
        return ESP_ERR_INVALID_STATE;
    }
    
    for (int retry = 0; retry < 10; retry++) {
        swd_ack_t ack = swd_transfer_raw(addr, true, true, data);
        
        if (ack == SWD_ACK_OK) {
            // Need to read RDBUFF for the actual data
            return swd_dp_read(DP_RDBUFF, data);
        } else if (ack == SWD_ACK_WAIT) {
            vTaskDelay(1);
        } else if (ack == SWD_ACK_FAULT) {
            swd_clear_errors();
        }
    }
    
    ESP_LOGE(TAG, "AP read failed: addr=0x%02X", addr);
    return ESP_FAIL;
}

// AP write with retry
esp_err_t swd_ap_write(uint8_t addr, uint32_t data) {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    for (int retry = 0; retry < 10; retry++) {
        swd_ack_t ack = swd_transfer_raw(addr, true, false, &data);
        
        if (ack == SWD_ACK_OK) {
            return ESP_OK;
        } else if (ack == SWD_ACK_WAIT) {
            vTaskDelay(1);
        } else if (ack == SWD_ACK_FAULT) {
            swd_clear_errors();
        }
    }
    
    ESP_LOGE(TAG, "AP write failed: addr=0x%02X data=0x%08lX", addr, data);
    return ESP_FAIL;
}

// Connect to target
esp_err_t swd_connect(void) {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Attempting SWD connection...");
    
    // Try dormant wakeup first
    dormant_wakeup();
    
    uint32_t idcode = 0;
    esp_err_t ret = swd_dp_read(DP_IDCODE, &idcode);
    
    if (ret != ESP_OK || idcode == 0 || idcode == 0xFFFFFFFF) {
        ESP_LOGW(TAG, "Dormant wakeup failed, trying JTAG-to-SWD");
        
        // Try JTAG to SWD sequence
        line_reset();
        jtag_to_swd();
        
        ret = swd_dp_read(DP_IDCODE, &idcode);
        if (ret != ESP_OK || idcode == 0 || idcode == 0xFFFFFFFF) {
            ESP_LOGE(TAG, "Failed to connect to target");
            return ESP_FAIL;
        }
    }
    
    ESP_LOGI(TAG, "Connected: IDCODE=0x%08lX", idcode);
    
    // Power up debug domain
    ret = swd_power_up();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to power up debug");
        return ret;
    }
    
    connected = true;
    return ESP_OK;
}

// Disconnect from target
esp_err_t swd_disconnect(void) {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    connected = false;
    
    // Send line reset to leave target in known state
    line_reset();
    
    ESP_LOGI(TAG, "Disconnected from target");
    return ESP_OK;
}

// Check connection status
bool swd_is_connected(void) {
    if (!connected) {
        return false;
    }
    
    // Verify by reading IDCODE
    uint32_t idcode = 0;
    esp_err_t ret = swd_dp_read(DP_IDCODE, &idcode);
    
    if (ret != ESP_OK || idcode == 0 || idcode == 0xFFFFFFFF) {
        connected = false;
        return false;
    }
    
    return true;
}

// Reset target using reset pin
esp_err_t swd_reset_target(void) {
    if (config.pin_reset < 0) {
        ESP_LOGW(TAG, "No reset pin configured");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    ESP_LOGI(TAG, "Resetting target...");
    
    gpio_set_level((gpio_num_t)config.pin_reset, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_num_t)config.pin_reset, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Reconnect after reset
    return swd_connect();
}

// Clear sticky errors
esp_err_t swd_clear_errors(void) {
    // Write ABORT register to clear errors
    // ORUNERRCLR, WDERRCLR, STKERRCLR, STKCMPCLR, DAPABORT
    uint32_t abort_val = 0x1E;
    return swd_dp_write(DP_ABORT, abort_val);
}

// Get IDCODE
uint32_t swd_get_idcode(void) {
    uint32_t idcode = 0;
    swd_dp_read(DP_IDCODE, &idcode);
    return idcode;
}

// Power up debug domain
esp_err_t swd_power_up(void) {
    // Clear any errors first
    swd_clear_errors();
    
    // Request debug and system power
    esp_err_t ret = swd_dp_write(DP_CTRL_STAT, 0x50000000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for power up acknowledgment
    for (int i = 0; i < 100; i++) {
        uint32_t status = 0;
        ret = swd_dp_read(DP_CTRL_STAT, &status);
        if (ret != ESP_OK) {
            return ret;
        }
        
        // Check CSYSPWRUPACK and CDBGPWRUPACK
        if ((status & 0xA0000000) == 0xA0000000) {
            ESP_LOGI(TAG, "Debug powered up: status=0x%08lX", status);
            return ESP_OK;
        }
        
        vTaskDelay(1);
    }
    
    ESP_LOGE(TAG, "Power up timeout");
    return ESP_ERR_TIMEOUT;
}