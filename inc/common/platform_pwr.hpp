#pragma once

#include "common/platform.hpp"

namespace Platform {
namespace PWR {

constexpr uint32_t PWR_BASE = 0x40007000UL;

// Power controller register structure
struct Registers {
    volatile uint32_t CR;    // Power control register
    volatile uint32_t CSR;   // Power control/status register
};

// Power Control Register (PWR_CR) bit definitions
enum class CR : uint32_t {
    LPDS = (1UL << 0),       // Low-power deep sleep
    PDDS = (1UL << 1),       // Power down deep sleep
    CWUF = (1UL << 2),       // Clear wakeup flag
    CSBF = (1UL << 3),       // Clear standby flag
    PVDE = (1UL << 4),       // Power voltage detector enable
    PLS_2_0V = (0UL << 5),   // PVD level: 2.0V
    PLS_2_1V = (1UL << 5),   // PVD level: 2.1V
    PLS_2_3V = (2UL << 5),   // PVD level: 2.3V
    PLS_2_5V = (3UL << 5),   // PVD level: 2.5V
    PLS_2_6V = (4UL << 5),   // PVD level: 2.6V
    PLS_2_7V = (5UL << 5),   // PVD level: 2.7V
    PLS_2_8V = (6UL << 5),   // PVD level: 2.8V
    PLS_2_9V = (7UL << 5),   // PVD level: 2.9V
    PLS_MSK = (7UL << 5),    // PVD level mask
    DBP = (1UL << 8),        // Disable backup domain write protection
    FPDS = (1UL << 9),       // Flash power down in Stop mode
    LPLVDS = (1UL << 10),    // Low-power regulator low voltage in deep sleep
    MRLVDS = (1UL << 11),    // Main regulator low voltage in deep sleep
    ADCDC1 = (1UL << 13),    // ADC DC1 (not on all STM32F4)
    VOS = (1UL << 14),       // Regulator voltage scaling output selection
    VOS_SCALE3 = (0UL << 14),// Voltage scale 3 mode (lowest power)
    VOS_SCALE2 = (1UL << 14),// Voltage scale 2 mode (balance)
    VOS_SCALE1 = (3UL << 14),// Voltage scale 1 mode (highest performance)
    VOS_MSK = (3UL << 14),   // Voltage scaling mask
    ODEN = (1UL << 16),      // Over-drive enable (high performance mode)
    ODSWEN = (1UL << 17),    // Over-drive switching enabled
    UDEN = (3UL << 18),      // Under-drive enable in stop mode
};

// Power Control/Status Register (PWR_CSR) bit definitions
enum class CSR : uint32_t {
    WUF = (1UL << 0),        // Wakeup flag
    SBF = (1UL << 1),        // Standby flag
    PVDO = (1UL << 2),       // PVD output
    BRR = (1UL << 3),        // Backup regulator ready
    EWUP1 = (1UL << 8),      // Enable WKUP1 pin
    EWUP2 = (1UL << 9),      // Enable WKUP2 pin
    EWUP3 = (1UL << 10),     // Enable WKUP3 pin (not on all STM32F4)
    BRE = (1UL << 9),        // Backup regulator enable
    VOSRDY = (1UL << 14),    // Voltage scaling select ready bit
    ODRDY = (1UL << 16),     // Over-drive mode ready
    ODSWRDY = (1UL << 17),   // Over-drive mode switching ready
    UDRDY = (3UL << 18),     // Under-drive ready flag
};
// Helper functions for bit manipulation
constexpr uint32_t getBitValue(CR bit) {
    return static_cast<uint32_t>(bit);
}

constexpr uint32_t getBitValue(CSR bit) {
    return static_cast<uint32_t>(bit);
}

// Operator overloads for combining flags
constexpr CR operator|(CR a, CR b) {
    return static_cast<CR>(
        static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

constexpr CSR operator|(CSR a, CSR b) {
    return static_cast<CSR>(
        static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

// Access function for PWR registers
inline Registers* getRegisters() {
    return reinterpret_cast<Registers*>(PWR_BASE);
}
}
}