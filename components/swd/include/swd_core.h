// swd_core.h - Core SWD Protocol Interface
#ifndef SWD_CORE_H
#define SWD_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"


// SWD Configuration
typedef struct {
    int pin_swclk;
    int pin_swdio;
    int pin_reset;  // -1 if not used
    int delay_cycles;  // Clock delay (0 for fastest)
} swd_config_t;

// SWD ACK responses
typedef enum {
    SWD_ACK_OK    = 0x1,
    SWD_ACK_WAIT  = 0x2,
    SWD_ACK_FAULT = 0x4,
    SWD_ACK_NACK  = 0x7
} swd_ack_t;

// DP/AP Register addresses
#define DP_IDCODE    0x00
#define DP_ABORT     0x00
#define DP_CTRL_STAT 0x04
#define DP_SELECT    0x08
#define DP_RDBUFF    0x0C

#define AP_CSW       0x00
#define AP_TAR       0x04
#define AP_DRW       0x0C
#define AP_IDR       0xFC  // AP Identification Register

// Initialize SWD interface
esp_err_t swd_init(const swd_config_t *config);

// Connection management
esp_err_t swd_connect(void);
esp_err_t swd_disconnect(void);
bool swd_is_connected(void);

// Low-level transfers (with automatic retry)
esp_err_t swd_dp_read(uint8_t addr, uint32_t *data);
esp_err_t swd_dp_write(uint8_t addr, uint32_t data);
esp_err_t swd_ap_read(uint8_t addr, uint32_t *data);
esp_err_t swd_ap_write(uint8_t addr, uint32_t data);

// Raw transfer (single attempt, no retry)
swd_ack_t swd_transfer_raw(uint8_t addr, bool ap, bool read, uint32_t *data);

// Reset and recovery
esp_err_t swd_reset_target(void);
esp_err_t swd_clear_errors(void);

// Utility
uint32_t swd_get_idcode(void);
esp_err_t swd_power_up(void);

#endif // SWD_CORE_H