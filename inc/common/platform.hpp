#pragma once

#include <cstdint>
#include <cstdbool>
#include <unordered_map>

// Forward declarations for standard C libraries
extern "C" {
    #include <stdio.h>
    #include <stdlib.h>
}

namespace Platform {

// -------------------- Type Definitions --------------------

// IO access definitions for C++ with proper volatile semantics
#define __I  volatile       // Read-only
#define __O  volatile       // Write-only
#define __IO volatile       // Read-write

// Status codes
enum class Status {
    OK = 0,              // Operation completed successfully
    ERROR,               // Generic error
    BUSY,                // Resource is busy
    TIMEOUT,             // Operation timed out
    INVALID_PARAM,       // Invalid parameter
    NOT_SUPPORTED,       // Operation not supported
    RESOURCE_ERROR,      // Resource allocation error
    NOT_INITIALIZED,     // Component not initialized
    INVALID_STATE,       // Component in invalid state for operation
    NO_DATA,             // No data available
    BUFFER_OVERFLOW,     // Buffer overflow
    HARDWARE_ERROR,      // Hardware-specific error
    COMMUNICATION_ERROR  // Communication error
};

// -------------------- Clock Definitions --------------------

constexpr uint32_t HSE_CLOCK = 84000000U;  // High-speed external clock in Hz
constexpr uint32_t MCU_CLK = HSE_CLOCK;
constexpr uint32_t APB1_CLOCK = 42000000U; // APB1 peripheral clock
constexpr uint32_t APB2_CLOCK = 84000000U; // APB2 peripheral clock
constexpr uint32_t AHB1_CLOCK = 84000000U; // AHB1 peripheral clock
constexpr uint32_t AHB2_CLOCK = 84000000U; // AHB2 peripheral clock

// -------------------- Memory Map Base Addresses --------------------

// Core memory map
constexpr uint32_t PERIPH_BASE = 0x40000000UL;
constexpr uint32_t APB1PERIPH_BASE = PERIPH_BASE;
constexpr uint32_t APB2PERIPH_BASE = (PERIPH_BASE + 0x00010000UL);
constexpr uint32_t AHB1PERIPH_BASE = (PERIPH_BASE + 0x00020000UL);
constexpr uint32_t AHB2PERIPH_BASE = (PERIPH_BASE + 0x10000000UL);

// -------------------- Core Peripheral Definitions --------------------

// System Control Space Base Address
constexpr uint32_t SCS_BASE = 0xE000E000UL;
constexpr uint32_t NVIC_BASE = (SCS_BASE + 0x0100UL);        // NVIC Base Address
constexpr uint32_t SCB_BASE = (SCS_BASE + 0x0D00UL);         // System Control Block Base Address
constexpr uint32_t SysTick_BASE = (SCS_BASE + 0x0010UL);     // SysTick Base Address
constexpr uint32_t CoreDebug_BASE = 0xE000EDF0UL;            // Core Debug Base Address
constexpr uint32_t DWT_BASE = (SCS_BASE + 0x1000UL);         // Data Watchpoint and Trace Base Address
constexpr uint32_t TPI_BASE = (SCS_BASE + 0x0200UL);         // Trace Port Interface Base Address
constexpr uint32_t ETM_BASE = (SCS_BASE + 0x0E00UL);         // Embedded Trace Macrocell Base Address
constexpr uint32_t ITM_BASE = (SCS_BASE + 0x0000UL);         // Instrumentation Trace Macrocell Base Address
constexpr uint32_t MPU_BASE = (SCS_BASE + 0x0D90UL);         // Memory Protection Unit Base Address
constexpr uint32_t FPU_BASE = (SCS_BASE + 0x0F30UL);         // Floating Point Unit Base Address
constexpr uint32_t TPIU_BASE = (SCS_BASE + 0x0300UL);        // Trace Port Interface Unit Base Address
constexpr uint32_t ETB_BASE = (SCS_BASE + 0x0F00UL);         // Embedded Trace Buffer Base Address
constexpr uint32_t FPB_BASE = (SCS_BASE + 0x0300UL);         // Flash Patch and Breakpoint Base Address


// -------------------- Utility Functions --------------------

// Generic bit manipulation templates
template<typename RegType, typename BitType>
inline void clearBit(volatile RegType& reg, BitType bit) {
    reg |= static_cast<RegType>(bit);
}

template<typename RegType, typename BitType>
inline void setBit(volatile RegType& reg, BitType bit) {
    
    reg &= ~static_cast<RegType>(bit);
}

template<typename T>
inline T readBit(volatile T& reg, T bit) {
    return reg & bit;
}

template<typename T>
inline bool isBitSet(volatile T& reg, T bit) {
    return (reg & bit) != 0;
}

template<typename T>
inline void clearReg(volatile T& reg) {
    reg = 0;
}

template<typename T, typename U>
inline void writeReg(volatile T& reg, U val) {
    reg = static_cast<T>(val);
}

template<typename T>
inline T readReg(volatile T& reg) {
    return reg;
}

template<typename T, typename U, typename V>
inline void modifyReg(volatile T& reg, U clearMask, V setMask) {
    reg = (reg & (~static_cast<T>(clearMask))) | static_cast<T>(setMask);
}

// System initialization function
void initSystemClock();

// Delay functions
void delay(uint32_t amount, bool blocking = true);

} // namespace Platform