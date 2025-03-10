// inc/hardware_abstraction/power.hpp
#pragma once

#include "hw_interface.hpp"
#include "common/platform.hpp"
#include "common/platform_pwr.hpp"
#include <memory>

namespace Platform {
namespace PWR {

/**
 * @brief Power domain voltage scale options
 * Controls the core voltage and affects maximum frequency
 */
enum class VoltageScale {
    Scale1 = 1,  // Highest performance, highest power consumption
    Scale2 = 2,  // Balance of performance and power consumption
    Scale3 = 3   // Lowest power consumption, lowest performance
};

/**
 * @brief Power controller regulator modes
 */
enum class RegulatorMode {
    Normal,     // Normal regulator mode - standard performance
    LowPower    // Low power regulator mode - reduced performance
};

/**
 * @brief Sleep entry modes
 */
enum class SleepEntryMode {
    WFI,        // Wait for Interrupt
    WFE         // Wait for Event
};

/**
 * @brief Wake-up pin configuration
 */
struct WakeupPinConfig {
    uint8_t pin_number;      // Wake-up pin number (1-3 for STM32F4)
    bool enabled;            // Enable status
    bool active_high;        // True if wake-up on rising edge, false for falling edge
};

/**
 * @brief Power controller configuration
 */
struct PowerConfig {
    VoltageScale voltage_scale;          // Voltage scaling option
    RegulatorMode regulator_mode;        // Regulator mode
    bool backup_domain_write_enable;     // Enable write access to backup domain
    bool backup_regulator_enable;        // Enable backup regulator (for RTC/backup registers in VBAT mode)
    bool sleep_on_exit;                  // Enter sleep mode on return from ISR
    bool wakeup_pin_config[3];           // Configuration for wake-up pins
    bool pvd_enable;                     // Power voltage detector enable
    uint8_t pvd_level;                   // Power voltage detector level (0-7)
};

/**
 * @brief Power event types for callback registration
 */
enum class PowerEvent {
    PVDChanged,        // Power voltage detector state changed
    WakeupPinTriggered,// Wakeup pin triggered
    Max                // Must be last - used for array sizing
};

/**
 * @brief Power controller interface that provides control over power modes and features
 */
class PowerInterface : public HwInterface {
public:
    // Get singleton instance
    static std::shared_ptr<PowerInterface> GetInstance();
    
    // Power mode control methods
    virtual Platform::Status EnterSleepMode(SleepEntryMode entry_mode) = 0;
    virtual Platform::Status EnterStopMode(RegulatorMode mode, SleepEntryMode entry_mode) = 0;
    virtual Platform::Status EnterStandbyMode() = 0;
    
    // Voltage scaling methods
    virtual Platform::Status SetVoltageScale(VoltageScale scale) = 0;
    virtual Platform::Status GetVoltageScale(VoltageScale& scale) = 0;
    
    // Regulator control methods
    virtual Platform::Status SetRegulatorMode(RegulatorMode mode) = 0;
    virtual Platform::Status GetRegulatorMode(RegulatorMode& mode) = 0;
    
    // Wake-up pin control methods
    virtual Platform::Status ConfigureWakeupPin(const WakeupPinConfig& config) = 0;
    virtual Platform::Status EnableWakeupPin(uint8_t pin_number) = 0;
    virtual Platform::Status DisableWakeupPin(uint8_t pin_number) = 0;
    
    // Backup domain control methods
    virtual Platform::Status EnableBackupDomainWrite() = 0;
    virtual Platform::Status DisableBackupDomainWrite() = 0;
    virtual bool IsBackupDomainWriteEnabled() const = 0;
    
    // Power voltage detector methods
    virtual Platform::Status ConfigurePVD(bool enable, uint8_t level) = 0;
    virtual Platform::Status GetPVDStatus(bool& triggered) const = 0;
    
    // Reset cause methods
    virtual bool IsWakeupFromStandby() const = 0;
    virtual bool IsWakeupFromStop() const = 0;
    virtual uint8_t GetWakeupPinFlag() const = 0;
    virtual Platform::Status ClearWakeupFlags() = 0;
};

// Power control command identifiers (for HwInterface::Control)
constexpr uint32_t POWER_CTRL_ENTER_SLEEP = 0x0601;
constexpr uint32_t POWER_CTRL_ENTER_STOP = 0x0602;
constexpr uint32_t POWER_CTRL_ENTER_STANDBY = 0x0603;
constexpr uint32_t POWER_CTRL_SET_VOLTAGE_SCALE = 0x0604;
constexpr uint32_t POWER_CTRL_GET_VOLTAGE_SCALE = 0x0605;
constexpr uint32_t POWER_CTRL_SET_REGULATOR_MODE = 0x0606;
constexpr uint32_t POWER_CTRL_GET_REGULATOR_MODE = 0x0607;
constexpr uint32_t POWER_CTRL_CONFIG_WAKEUP_PIN = 0x0608;
constexpr uint32_t POWER_CTRL_ENABLE_WAKEUP_PIN = 0x0609;
constexpr uint32_t POWER_CTRL_DISABLE_WAKEUP_PIN = 0x060A;
constexpr uint32_t POWER_CTRL_ENABLE_BACKUP_DOMAIN_WRITE = 0x060B;
constexpr uint32_t POWER_CTRL_DISABLE_BACKUP_DOMAIN_WRITE = 0x060C;
constexpr uint32_t POWER_CTRL_IS_BACKUP_DOMAIN_WRITE_ENABLED = 0x060D;
constexpr uint32_t POWER_CTRL_CONFIG_PVD = 0x060E;
constexpr uint32_t POWER_CTRL_GET_PVD_STATUS = 0x060F;
constexpr uint32_t POWER_CTRL_IS_WAKEUP_FROM_STANDBY = 0x0610;
constexpr uint32_t POWER_CTRL_IS_WAKEUP_FROM_STOP = 0x0611;
constexpr uint32_t POWER_CTRL_GET_WAKEUP_PIN_FLAG = 0x0612;
constexpr uint32_t POWER_CTRL_CLEAR_WAKEUP_FLAGS = 0x0613;

/**
 * @brief Platform-specific register definitions for the power controller
 */

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

} // namespace PWR
} // namespace Platform