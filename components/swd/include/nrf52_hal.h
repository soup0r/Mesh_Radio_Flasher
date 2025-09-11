#ifndef NRF52_HAL_H
#define NRF52_HAL_H

#include <stdint.h>

// Memory map
#define NRF52_FLASH_BASE    0x00000000
#define NRF52_FLASH_SIZE    (1024 * 1024)  // 1MB
#define NRF52_SRAM_BASE     0x20000000
#define NRF52_SRAM_SIZE     (256 * 1024)   // 256KB

// System registers
#define NRF52_CPUID         0xE000ED00
#define NRF52_ICSR          0xE000ED04
#define NRF52_VTOR          0xE000ED08
#define NRF52_AIRCR         0xE000ED0C
#define NRF52_SCR           0xE000ED10
#define NRF52_CCR           0xE000ED14
#define NRF52_SHCSR         0xE000ED24
#define NRF52_CFSR          0xE000ED28
#define NRF52_HFSR          0xE000ED2C
#define NRF52_DFSR          0xE000ED30
#define NRF52_MMFAR         0xE000ED34
#define NRF52_BFAR          0xE000ED38
#define NRF52_AFSR          0xE000ED3C

// FICR (Factory Information Config Registers)
#define FICR_BASE           0x10000000
#define FICR_CODEPAGESIZE   (FICR_BASE + 0x010)
#define FICR_CODESIZE       (FICR_BASE + 0x014)
#define FICR_DEVICEID0      (FICR_BASE + 0x060)
#define FICR_DEVICEID1      (FICR_BASE + 0x064)
#define FICR_DEVICEADDR0    (FICR_BASE + 0x0A4)
#define FICR_DEVICEADDR1    (FICR_BASE + 0x0A8)
#define FICR_INFO_PART      (FICR_BASE + 0x100)
#define FICR_INFO_VARIANT   (FICR_BASE + 0x104)
#define FICR_INFO_PACKAGE   (FICR_BASE + 0x108)
#define FICR_INFO_RAM       (FICR_BASE + 0x10C)
#define FICR_INFO_FLASH     (FICR_BASE + 0x110)

// UICR (User Information Config Registers)
#define UICR_BASE           0x10001000
#define UICR_BOOTLOADERADDR (UICR_BASE + 0x014)
#define UICR_NRFFW0         (UICR_BASE + 0x018)
#define UICR_NRFFW1         (UICR_BASE + 0x01C)
#define UICR_CUSTOMER0      (UICR_BASE + 0x080)
#define UICR_PSELRESET0     (UICR_BASE + 0x200)
#define UICR_PSELRESET1     (UICR_BASE + 0x204)
#define UICR_APPROTECT      (UICR_BASE + 0x208)

// Protection values
#define APPROTECT_HW_DISABLED  0xFFFFFF5A  // Hardware disabled (can be re-enabled via register)
#define APPROTECT_ENABLED      0xFFFFFF00  // Protection enabled
#define APPROTECT_DISABLED     0xFFFFFFFF  // Default erased value (still protected on nRF52840!)

#define NRF52_AIRCR         0xE000ED0C  // Application Interrupt and Reset Control Register

// CTRL-AP registers (AP1)
#define CTRLAP_ERASE        0x04        // Erase control register
#define CTRLAP_ERASEALL     0x01        // Mass erase command


#endif // NRF52_HAL_H