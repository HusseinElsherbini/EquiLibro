﻿// src/hardware_abstraction/power.cpp
#include "hardware_abstraction/pwr.hpp"
#include "hardware_abstraction/rcc.hpp"
#include "common/platform_cmsis.hpp"
#include <mutex>

namespace Platform {
namespace PWR {

// Singleton implementation
class PowerInterfaceImpl : public PowerInterface {
private:
    // Internal state tracking
    bool initialized;
    PowerConfig config;

    // Grant access to the GetInstance method
    friend std::shared_ptr<PowerInterface> PowerInterface::GetInstance();
    
    // Mutex for thread safety
    std::mutex power_mutex;
    
    // Callback table
    struct CallbackEntry {
        void (*callback)(void* param);
        void* param;
        bool enabled;
    };
    
    CallbackEntry callbacks[static_cast<size_t>(PowerEvent::Max)];
    
    // Private constructor for singleton pattern
    PowerInterfaceImpl();
    
    // Deleted copy constructor and assignment operator
    PowerInterfaceImpl(const PowerInterfaceImpl&) = delete;
    PowerInterfaceImpl& operator=(const PowerInterfaceImpl&) = delete;
    
public:
    // Destructor
    ~PowerInterfaceImpl() override;
    
    // Interface implementation
    Platform::Status Init(void* config) override;
    Platform::Status DeInit() override;
    Platform::Status Control(uint32_t command, void* param) override;
    Platform::Status Read(void* buffer, uint16_t size, uint32_t timeout) override;
    Platform::Status Write(const void* data, uint16_t size, uint32_t timeout) override;
    Platform::Status RegisterCallback(uint32_t eventId, void (*callback)(void* param), void* param) override;
    
    // Power mode control methods
    Platform::Status EnterSleepMode(SleepEntryMode entry_mode) override;
    Platform::Status EnterStopMode(RegulatorMode mode, SleepEntryMode entry_mode) override;
    Platform::Status EnterStandbyMode() override;
    
    // Voltage scaling methods
    Platform::Status SetVoltageScale(VoltageScale scale) override;
    Platform::Status GetVoltageScale(VoltageScale& scale) override;
    
    // Regulator control methods
    Platform::Status SetRegulatorMode(RegulatorMode mode) override;
    Platform::Status GetRegulatorMode(RegulatorMode& mode) override;
    
    // Wake-up pin control methods
    Platform::Status ConfigureWakeupPin(const WakeupPinConfig& config) override;
    Platform::Status EnableWakeupPin(uint8_t pin_number) override;
    Platform::Status DisableWakeupPin(uint8_t pin_number) override;
    
    // Backup domain control methods
    Platform::Status EnableBackupDomainWrite() override;
    Platform::Status DisableBackupDomainWrite() override;
    bool IsBackupDomainWriteEnabled() const override;
    
    // Power voltage detector methods
    Platform::Status ConfigurePVD(bool enable, uint8_t level) override;
    Platform::Status GetPVDStatus(bool& triggered) const override;
    
    // Reset cause methods
    bool IsWakeupFromStandby() const override;
    bool IsWakeupFromStop() const override;
    uint8_t GetWakeupPinFlag() const override;
    Platform::Status ClearWakeupFlags() override;
};

// Static instance
static std::shared_ptr<PowerInterfaceImpl> power_instance = nullptr;
static std::mutex instance_mutex;

// Get singleton instance
std::shared_ptr<PowerInterface> PowerInterface::GetInstance() {
    std::lock_guard<std::mutex> lock(instance_mutex);
    
    if (!power_instance) {
        power_instance = std::shared_ptr<PowerInterfaceImpl>(new PowerInterfaceImpl());
    }
    
    return power_instance;
}

// Constructor
PowerInterfaceImpl::PowerInterfaceImpl()
    : initialized(false) {
    
    // Initialize callbacks to null
    for (auto& callback : callbacks) {
        callback.callback = nullptr;
        callback.param = nullptr;
        callback.enabled = false;
    }
}

// Destructor
PowerInterfaceImpl::~PowerInterfaceImpl() {
    if (initialized) {
        DeInit();
    }
}

// Initialize the power controller
Platform::Status PowerInterfaceImpl::Init(void* config) {
    std::lock_guard<std::mutex> lock(power_mutex);
    
    if (initialized) {
        return Platform::Status::OK; // Already initialized
    }
    
    // Enable PWR clock in RCC
    auto& rcc = Platform::RCC::RccInterface::GetInstance();
    Platform::Status status = rcc.EnablePeripheralClock(Platform::RCC::RccPeripheral::PWR);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Apply configuration if provided
    if (config != nullptr) {
        this->config = *static_cast<PowerConfig*>(config);
        
        // Apply voltage scale if specified
        status = SetVoltageScale(this->config.voltage_scale);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Apply regulator mode if specified
        status = SetRegulatorMode(this->config.regulator_mode);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Configure backup domain write access
        if (this->config.backup_domain_write_enable) {
            status = EnableBackupDomainWrite();
        } else {
            status = DisableBackupDomainWrite();
        }
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Configure backup regulator
        Registers* pwr = getRegisters();
        if (this->config.backup_regulator_enable) {
            pwr->CSR |= getBitValue(CSR::BRE);
            
            // Wait for backup regulator ready
            uint32_t timeout = 50000;
            while (!(pwr->CSR & getBitValue(CSR::BRR))) {
                if (--timeout == 0) {
                    return Platform::Status::TIMEOUT;
                }
            }
        } else {
            pwr->CSR &= ~getBitValue(CSR::BRE);
        }
        
        // Configure sleep on exit option
        if (this->config.sleep_on_exit) {
            Platform::CMSIS::SCB::getRegisters()->SCR |= (1 << 1); // Set SLEEPONEXIT bit
        } else {
            Platform::CMSIS::SCB::getRegisters()->SCR &= ~(1 << 1); // Clear SLEEPONEXIT bit
        }
        
        // Configure wake-up pins
        for (uint8_t i = 0; i < 3; i++) {
            if (i < 2 || (i == 2 && this->config.wakeup_pin_config[2])) { // WKUP3 not available on all STM32F4
                WakeupPinConfig wakeup_config;
                wakeup_config.pin_number = i + 1;
                wakeup_config.enabled = this->config.wakeup_pin_config[i];
                wakeup_config.active_high = false; // Default to active low
                
                status = ConfigureWakeupPin(wakeup_config);
                if (status != Platform::Status::OK) {
                    return status;
                }
            }
        }
        
        // Configure PVD
        status = ConfigurePVD(this->config.pvd_enable, this->config.pvd_level);
        if (status != Platform::Status::OK) {
            return status;
        }
    }
    
    initialized = true;
    return Platform::Status::OK;
}

// Deinitialize the power controller
Platform::Status PowerInterfaceImpl::DeInit() {
    std::lock_guard<std::mutex> lock(power_mutex);
    
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Reset PWR peripheral
    auto& rcc = Platform::RCC::RccInterface::GetInstance();
    Platform::Status status = rcc.DisablePeripheralClock(Platform::RCC::RccPeripheral::PWR);
    
    initialized = false;
    return status;
}

// Control function for HwInterface compatibility
Platform::Status PowerInterfaceImpl::Control(uint32_t command, void* param) {
    if (!initialized && command != POWER_CTRL_SET_VOLTAGE_SCALE) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    switch (command) {
        case POWER_CTRL_ENTER_SLEEP: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            SleepEntryMode entry_mode = *static_cast<SleepEntryMode*>(param);
            return EnterSleepMode(entry_mode);
        }
        
        case POWER_CTRL_ENTER_STOP: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            struct StopModeParams {
                RegulatorMode regulator_mode;
                SleepEntryMode entry_mode;
            };
            
            StopModeParams* stop_params = static_cast<StopModeParams*>(param);
            return EnterStopMode(stop_params->regulator_mode, stop_params->entry_mode);
        }
        
        case POWER_CTRL_ENTER_STANDBY: {
            return EnterStandbyMode();
        }
        
        case POWER_CTRL_SET_VOLTAGE_SCALE: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            VoltageScale scale = *static_cast<VoltageScale*>(param);
            return SetVoltageScale(scale);
        }
        
        case POWER_CTRL_GET_VOLTAGE_SCALE: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            VoltageScale* scale = static_cast<VoltageScale*>(param);
            return GetVoltageScale(*scale);
        }
        
        // Implement all other command handlers
        
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
}

// Not typically used for power controller
Platform::Status PowerInterfaceImpl::Read(void* buffer, uint16_t size, uint32_t timeout) {
    return Platform::Status::NOT_SUPPORTED;
}

// Not typically used for power controller
Platform::Status PowerInterfaceImpl::Write(const void* data, uint16_t size, uint32_t timeout) {
    return Platform::Status::NOT_SUPPORTED;
}

// Register callback for power events
Platform::Status PowerInterfaceImpl::RegisterCallback(uint32_t eventId, void (*callback)(void* param), void* param) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (eventId >= static_cast<uint32_t>(PowerEvent::Max)) {
        return Platform::Status::INVALID_PARAM;
    }
    
    std::lock_guard<std::mutex> lock(power_mutex);
    
    callbacks[eventId].callback = callback;
    callbacks[eventId].param = param;
    callbacks[eventId].enabled = (callback != nullptr);
    
    return Platform::Status::OK;
}

// Enter Sleep mode
Platform::Status PowerInterfaceImpl::EnterSleepMode(SleepEntryMode entry_mode) {
    std::lock_guard<std::mutex> lock(power_mutex);
    
    // Clear SLEEPDEEP bit to enter Sleep mode instead of Deep Sleep
    Platform::CMSIS::SCB::getRegisters()->SCR &= ~(1 << 2);
    
    // Enter sleep mode using the specified method
    if (entry_mode == SleepEntryMode::WFI) {
        __asm volatile("wfi");
    } else {
        __asm volatile("wfe");
    }
    
    return Platform::Status::OK;
}

// Enter Stop mode
Platform::Status PowerInterfaceImpl::EnterStopMode(RegulatorMode mode, SleepEntryMode entry_mode) {
    std::lock_guard<std::mutex> lock(power_mutex);
    
    Registers* pwr = getRegisters();
    
    // Configure the regulator mode in Stop mode
    if (mode == RegulatorMode::LowPower) {
        pwr->CR |= getBitValue(CR::LPDS);  // Low-power regulator in Stop mode
    } else {
        pwr->CR &= ~getBitValue(CR::LPDS); // Main regulator in Stop mode
    }
    
    // Clear PDDS bit to enter Stop mode instead of Standby mode
    pwr->CR &= ~getBitValue(CR::PDDS);
    
    // Set SLEEPDEEP bit to enter Deep Sleep
    Platform::CMSIS::SCB::getRegisters()->SCR |= (1 << 2);
    
    // Enter stop mode using the specified method
    if (entry_mode == SleepEntryMode::WFI) {
        __asm volatile("wfi");
    } else {
        __asm volatile("wfe");
    }
    
    // Clear SLEEPDEEP bit after waking up
    Platform::CMSIS::SCB::getRegisters()->SCR &= ~(1 << 2);
    
    return Platform::Status::OK;
}

// Enter Standby mode
Platform::Status PowerInterfaceImpl::EnterStandbyMode() {
    std::lock_guard<std::mutex> lock(power_mutex);
    
    Registers* pwr = getRegisters();
    
    // Clear wake-up flag
    pwr->CR |= getBitValue(CR::CWUF);
    
    // Clear standby flag
    pwr->CR |= getBitValue(CR::CSBF);
    
    // Set PDDS bit to enter Standby mode
    pwr->CR |= getBitValue(CR::PDDS);
    
    // Set SLEEPDEEP bit to enter Deep Sleep
    Platform::CMSIS::SCB::getRegisters()->SCR |= (1 << 2);
    
    // Ensure all outstanding memory accesses complete
    __asm volatile("dsb");
    
    // Enter standby mode
    __asm volatile("wfi");
    
    // Code should not reach here as the device will reset on wakeup from Standby
    return Platform::Status::OK;
}

// Set voltage scaling
Platform::Status PowerInterfaceImpl::SetVoltageScale(VoltageScale scale) {
    std::lock_guard<std::mutex> lock(power_mutex);
    
    Registers* pwr = getRegisters();
    
    // Modify VOS bits in PWR_CR
    uint32_t cr_value = pwr->CR;
    cr_value &= ~getBitValue(CR::VOS_MSK);
    
    switch (scale) {
        case VoltageScale::Scale1:
            cr_value |= getBitValue(CR::VOS_SCALE1);
            break;
        case VoltageScale::Scale2:
            cr_value |= getBitValue(CR::VOS_SCALE2);
            break;
        case VoltageScale::Scale3:
            cr_value |= getBitValue(CR::VOS_SCALE3);
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    pwr->CR = cr_value;
    
    // Wait for voltage scaling to complete
    uint32_t timeout = 50000;
    while (!(pwr->CSR & getBitValue(CSR::VOSRDY))) {
        if (--timeout == 0) {
            return Platform::Status::TIMEOUT;
        }
    }
    
    return Platform::Status::OK;
}

// Get current voltage scaling
Platform::Status PowerInterfaceImpl::GetVoltageScale(VoltageScale& scale) {
    std::lock_guard<std::mutex> lock(power_mutex);
    
    Registers* pwr = getRegisters();
    
    // Read VOS bits from PWR_CR
    uint32_t vos_bits = pwr->CR & getBitValue(CR::VOS_MSK);
    
    if (vos_bits == getBitValue(CR::VOS_SCALE1)) {
        scale = VoltageScale::Scale1;
    } else if (vos_bits == getBitValue(CR::VOS_SCALE2)) {
        scale = VoltageScale::Scale2;
    } else {
        scale = VoltageScale::Scale3;
    }
    
    return Platform::Status::OK;
}

// Set regulator mode
Platform::Status PowerInterfaceImpl::SetRegulatorMode(RegulatorMode mode) {
    std::lock_guard<std::mutex> lock(power_mutex);
    
    Registers* pwr = getRegisters();
    
    // Configure the regulator mode
    if (mode == RegulatorMode::LowPower) {
        pwr->CR |= getBitValue(CR::LPDS);  // Low-power regulator
    } else {
        pwr->CR &= ~getBitValue(CR::LPDS); // Main regulator
    }
    
    return Platform::Status::OK;
}

// Get current regulator mode
Platform::Status PowerInterfaceImpl::GetRegulatorMode(RegulatorMode& mode) {
    std::lock_guard<std::mutex> lock(power_mutex);
    
    Registers* pwr = getRegisters();
    
    // Read LPDS bit from PWR_CR
    if (pwr->CR & getBitValue(CR::LPDS)) {
        mode = RegulatorMode::LowPower;
    } else {
        mode = RegulatorMode::Normal;
    }
    
    return Platform::Status::OK;
}

// Configure wake-up pin
Platform::Status PowerInterfaceImpl::ConfigureWakeupPin(const WakeupPinConfig& config) {
    std::lock_guard<std::mutex> lock(power_mutex);
    
    // Validate pin number (STM32F4 has up to 3 wake-up pins, depending on the model)
    if (config.pin_number < 1 || config.pin_number > 3) {
        return Platform::Status::INVALID_PARAM;
    }
    
    Registers* pwr = getRegisters();
    
    // Enable or disable the wake-up pin
    switch (config.pin_number) {
        case 1:
            if (config.enabled) {
                pwr->CSR |= getBitValue(CSR::EWUP1);
            } else {
                pwr->CSR &= ~getBitValue(CSR::EWUP1);
            }
            break;
        case 2:
            if (config.enabled) {
                pwr->CSR |= getBitValue(CSR::EWUP2);
            } else {
                pwr->CSR &= ~getBitValue(CSR::EWUP2);
            }
            break;
        case 3:
            // Check if WKUP3 is available on this STM32F4 model
            if (static_cast<uint32_t>(CSR::EWUP3) == 0) {
                return Platform::Status::NOT_SUPPORTED;
            }
            
            if (config.enabled) {
                pwr->CSR |= getBitValue(CSR::EWUP3);
            } else {
                pwr->CSR &= ~getBitValue(CSR::EWUP3);
            }
            break;
    }
    
    // Note: The polarity of the wake-up pin is fixed in hardware (active low)
    // Some STM32 models support configuring the polarity through GPIO EXTI settings
    // but the base wake-up pin functionality is active low
    
    return Platform::Status::OK;
 }
 
 // Enable a specific wake-up pin
 Platform::Status PowerInterfaceImpl::EnableWakeupPin(uint8_t pin_number) {
    std::lock_guard<std::mutex> lock(power_mutex);
    
    // Validate pin number
    if (pin_number < 1 || pin_number > 3) {
        return Platform::Status::INVALID_PARAM;
    }
    
    Registers* pwr = getRegisters();
    
    // Enable the specified wake-up pin
    switch (pin_number) {
        case 1:
            pwr->CSR |= getBitValue(CSR::EWUP1);
            break;
        case 2:
            pwr->CSR |= getBitValue(CSR::EWUP2);
            break;
        case 3:
            // Check if WKUP3 is available on this STM32F4 model
            if (static_cast<uint32_t>(CSR::EWUP3) == 0) {
                return Platform::Status::NOT_SUPPORTED;
            }
            pwr->CSR |= getBitValue(CSR::EWUP3);
            break;
    }
    
    return Platform::Status::OK;
 }
 
 // Disable a specific wake-up pin
 Platform::Status PowerInterfaceImpl::DisableWakeupPin(uint8_t pin_number) {
    std::lock_guard<std::mutex> lock(power_mutex);
    
    // Validate pin number
    if (pin_number < 1 || pin_number > 3) {
        return Platform::Status::INVALID_PARAM;
    }
    
    Registers* pwr = getRegisters();
    
    // Disable the specified wake-up pin
    switch (pin_number) {
        case 1:
            pwr->CSR &= ~getBitValue(CSR::EWUP1);
            break;
        case 2:
            pwr->CSR &= ~getBitValue(CSR::EWUP2);
            break;
        case 3:
            // Check if WKUP3 is available on this STM32F4 model
            if (static_cast<uint32_t>(CSR::EWUP3) == 0) {
                return Platform::Status::NOT_SUPPORTED;
            }
            pwr->CSR &= ~getBitValue(CSR::EWUP3);
            break;
    }
    
    return Platform::Status::OK;
 }
 
 // Enable write access to backup domain
 Platform::Status PowerInterfaceImpl::EnableBackupDomainWrite() {
    std::lock_guard<std::mutex> lock(power_mutex);
    
    Registers* pwr = getRegisters();
    
    // Set DBP bit to enable backup domain write access
    pwr->CR |= getBitValue(CR::DBP);
    
    return Platform::Status::OK;
 }
 
 // Disable write access to backup domain
 Platform::Status PowerInterfaceImpl::DisableBackupDomainWrite() {
    std::lock_guard<std::mutex> lock(power_mutex);
    
    Registers* pwr = getRegisters();
    
    // Clear DBP bit to disable backup domain write access
    pwr->CR &= ~getBitValue(CR::DBP);
    
    return Platform::Status::OK;
 }
 
 // Check if backup domain write access is enabled
 bool PowerInterfaceImpl::IsBackupDomainWriteEnabled() const {
    Registers* pwr = getRegisters();
    
    // Check if DBP bit is set
    return (pwr->CR & getBitValue(CR::DBP)) != 0;
 }
 
 // Configure the Power Voltage Detector (PVD)
 Platform::Status PowerInterfaceImpl::ConfigurePVD(bool enable, uint8_t level) {
    std::lock_guard<std::mutex> lock(power_mutex);
    
    // Validate level parameter
    if (level > 7) {
        return Platform::Status::INVALID_PARAM;
    }
    
    Registers* pwr = getRegisters();
    
    // Configure PVD level
    uint32_t cr_value = pwr->CR;
    cr_value &= ~getBitValue(CR::PLS_MSK);
    cr_value |= (level << 5); // PLS field starts at bit 5
    
    // Enable or disable PVD
    if (enable) {
        cr_value |= getBitValue(CR::PVDE);
    } else {
        cr_value &= ~getBitValue(CR::PVDE);
    }
    
    pwr->CR = cr_value;
    
    return Platform::Status::OK;
 }
 
 // Get PVD status
 Platform::Status PowerInterfaceImpl::GetPVDStatus(bool& triggered) const {
    Registers* pwr = getRegisters();
    
    // Check PVDO bit in CSR
    triggered = (pwr->CSR & getBitValue(CSR::PVDO)) != 0;
    
    return Platform::Status::OK;
 }
 
 // Check if the system woke up from Standby mode
 bool PowerInterfaceImpl::IsWakeupFromStandby() const {
    Registers* pwr = getRegisters();
    
    // Check SBF (Standby Flag)
    return (pwr->CSR & getBitValue(CSR::SBF)) != 0;
 }
 
 // Check if the system woke up from Stop mode
 bool PowerInterfaceImpl::IsWakeupFromStop() const {
    // STM32F4 doesn't have a specific flag for wakeup from Stop mode.
    // Usually, this is determined by checking if wake-up flag (WUF) is set
    // and standby flag (SBF) is not set.
    Registers* pwr = getRegisters();
    
    return ((pwr->CSR & getBitValue(CSR::WUF)) != 0) && 
           ((pwr->CSR & getBitValue(CSR::SBF)) == 0);
 }
 
 // Get the wake-up pin flag
 uint8_t PowerInterfaceImpl::GetWakeupPinFlag() const {
    // STM32F4 doesn't have individual flags for each wake-up pin in the PWR peripheral.
    // It only has a single WUF flag. For more advanced wake-up source detection,
    // you would typically check the EXTI pending register.
    
    Registers* pwr = getRegisters();
    
    return (pwr->CSR & getBitValue(CSR::WUF)) ? 1 : 0;
 }
 
 // Clear wake-up flags
 Platform::Status PowerInterfaceImpl::ClearWakeupFlags() {
    std::lock_guard<std::mutex> lock(power_mutex);
    
    Registers* pwr = getRegisters();
    
    // Clear wake-up flag
    pwr->CR |= getBitValue(CR::CWUF);
    
    // Clear standby flag
    pwr->CR |= getBitValue(CR::CSBF);
    
    return Platform::Status::OK;
 }
 
 } // namespace PWR
 } // namespace Platform