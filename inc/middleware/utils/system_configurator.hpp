// system_configurator.hpp
#pragma once

#include "system_services/system_manager.hpp"
#include "hardware_abstraction/flash.hpp"
#include "hardware_abstraction/pwr.hpp"
#include "hardware_abstraction/rcc.hpp"

namespace Middleware {
namespace SystemServices {

/**
 * @brief Fluent configuration interface for system settings
 * 
 * SystemConfigurator provides an intuitive interface for configuring
 * system parameters with sensible defaults and automatic validation.
 */
class SystemConfigurator {
private:
    // The configuration we're building
    SystemInitConfig config;
    
    // Internal validation functions
    void validateClockSettings();
    void validateFlashSettings();
    void validatePowerSettings();
    void ensureCompatibleSettings();
    
public:
    /**
     * @brief Create a configurator with default system settings
     */
    SystemConfigurator();
    
    /**
     * @brief Create a configurator from a predefined profile
     * 
     * @param profile The system profile to use as a starting point
     */
    enum class Profile {
        Default,
        LowPower,
        HighPerformance,
        Debug
    };
    
    static SystemConfigurator FromProfile(Profile profile);
    
    /**
     * @brief Create a configurator from an existing configuration
     * 
     * @param existingConfig Configuration to use as a starting point
     */
    static SystemConfigurator FromExistingConfig(const SystemInitConfig& existingConfig);
    
    // Clock configuration methods
    SystemConfigurator& withSystemClock(uint32_t frequency);
    SystemConfigurator& withExternalCrystal(uint32_t frequency);
    SystemConfigurator& withInternalOscillator();
    SystemConfigurator& withPLLSource(Platform::RCC::RccClockSource source);
    
    // Bus configuration methods
    SystemConfigurator& withAHBPrescaler(uint8_t prescaler);
    SystemConfigurator& withAPB1Prescaler(uint8_t prescaler);
    SystemConfigurator& withAPB2Prescaler(uint8_t prescaler);
    
    // Flash configuration methods
    SystemConfigurator& withFlashLatency(Platform::FLASH::FlashLatency latency);
    SystemConfigurator& withAutoFlashLatency(bool enable = true);
    SystemConfigurator& enablePrefetch(bool enable = true);
    SystemConfigurator& enableInstructionCache(bool enable = true);
    SystemConfigurator& enableDataCache(bool enable = true);
    
    // Power configuration methods
    SystemConfigurator& withPowerMode(Platform::PWR::PowerMode mode);
    SystemConfigurator& withVoltageScale(Platform::PWR::VoltageScale scale);
    SystemConfigurator& withRegulatorMode(Platform::PWR::RegulatorMode mode);
    
    // System features methods
    SystemConfigurator& enableSysTick(bool enable = true);
    SystemConfigurator& withSysTickInterval(uint32_t intervalMicroseconds);
    SystemConfigurator& enableMPU(bool enable = true);
    SystemConfigurator& enableFPU(bool enable = true);
    
    // Finalization methods
    SystemInitConfig build();
    Platform::Status applyToSystem();
};

// Helper functions
inline SystemConfigurator CreateDefaultConfigurator() {
    return SystemConfigurator();
}

inline SystemConfigurator CreateLowPowerConfigurator() {
    return SystemConfigurator::FromProfile(SystemConfigurator::Profile::LowPower);
}

inline SystemConfigurator CreateHighPerformanceConfigurator() {
    return SystemConfigurator::FromProfile(SystemConfigurator::Profile::HighPerformance);
}

} // namespace SystemServices
} // namespace Middleware