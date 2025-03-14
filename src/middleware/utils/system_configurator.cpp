// system_configurator.cpp
#include "system_configurator.hpp"

namespace Middleware {
namespace SystemServices {

SystemConfigurator::SystemConfigurator() {
    // Start with the system's default configuration
    config = SystemManager::GetDefaultConfig();
}

SystemConfigurator SystemConfigurator::FromProfile(Profile profile) {
    SystemConfigurator configurator;
    
    switch (profile) {
        case Profile::LowPower:
            configurator.config = SystemManager::GetLowPowerConfig();
            break;
            
        case Profile::HighPerformance:
            configurator.config = SystemManager::GetMaxPerformanceConfig();
            break;
            
        case Profile::Debug:
            // Create a debug-optimized configuration
            configurator.config = SystemManager::GetDefaultConfig();
            configurator.config.FlashConfig.latency = Platform::FLASH::FlashLatency::WS0;
            configurator.config.SysClockConfig.systemClockHz = 16000000; // Use HSI directly
            configurator.config.SysClockConfig.clockSource = Platform::RCC::RccClockSource::HSI;
            configurator.config.SysClockConfig.pllConfig.autoCalculate = false;
            configurator.config.sysTickInterval = 1000; // 1ms interval for accurate debugging
            break;
            
        case Profile::Default:
        default:
            configurator.config = SystemManager::GetDefaultConfig();
            break;
    }
    
    return configurator;
}

SystemConfigurator& SystemConfigurator::enablePrefetch(bool enable) {
    config.FlashConfig.prefetch_enable = enable;
    return *this;
}

SystemConfigurator& SystemConfigurator::enableDataCache(bool enable) {
    config.FlashConfig.dcache_enable = enable;
    return *this;
}

SystemConfigurator& SystemConfigurator::enableSysTick(bool enable) {
    config.enableSysTick = enable;
    return *this;
}

SystemConfigurator& SystemConfigurator::withSysTickInterval(uint32_t intervalMicroseconds) {
    config.sysTickInterval = intervalMicroseconds;
    return *this;
}
void SystemConfigurator::validatePowerSettings() {
    // Validate power mode settings
    
    // Check if power mode is valid
    if (config.PowerConfig.power_mode != Platform::PWR::PowerMode::Run &&
        config.PowerConfig.power_mode != Platform::PWR::PowerMode::LowPower &&
        config.PowerConfig.power_mode != Platform::PWR::PowerMode::Sleep &&
        config.PowerConfig.power_mode != Platform::PWR::PowerMode::DeepSleep &&
        config.PowerConfig.power_mode != Platform::PWR::PowerMode::Standby) {
        // Default to Run mode if invalid
        config.PowerConfig.power_mode = Platform::PWR::PowerMode::Run;
    }
    
    // Ensure voltage scale is compatible with power mode
    if (config.PowerConfig.power_mode == Platform::PWR::PowerMode::LowPower) {
        // In low power mode, scale 3 (lowest power) is recommended
        // If currently set to a higher power scale, adjust it
        if (config.PowerConfig.voltage_scale == Platform::PWR::VoltageScale::Scale1) {
            config.PowerConfig.voltage_scale = Platform::PWR::VoltageScale::Scale3;
        }
    } else if (config.PowerConfig.power_mode == Platform::PWR::PowerMode::Run) {
        // In run mode with high frequency, scale 1 (highest performance) may be needed
        // Check if clock frequency is high and adjust voltage scale if needed
        if (config.SysClockConfig.systemClockHz > 60000000 && 
            config.PowerConfig.voltage_scale != Platform::PWR::VoltageScale::Scale1) {
            config.PowerConfig.voltage_scale = Platform::PWR::VoltageScale::Scale1;
        }
    }
    
    // Check backup regulator settings for standby mode
    if (config.PowerConfig.power_mode == Platform::PWR::PowerMode::Standby) {
        // In standby mode, you might want to ensure backup regulator settings
        // are appropriate for maintaining backup domain
        if (config.PowerConfig.backup_domain_write_enable) {
            // Allow backup domain writes for configuration
        }
    }
    
    // Validate wakeup pin configurations if in sleep, deep sleep, or standby modes
    if (config.PowerConfig.power_mode == Platform::PWR::PowerMode::Sleep ||
        config.PowerConfig.power_mode == Platform::PWR::PowerMode::DeepSleep ||
        config.PowerConfig.power_mode == Platform::PWR::PowerMode::Standby) {
        // At least one wakeup source should be enabled for these modes
        // Otherwise, the system might not be able to wake up
        bool any_wakeup_enabled = false;
        for (int i = 0; i < 3; i++) {
            if (config.PowerConfig.wakeup_pin_config[i]) {
                any_wakeup_enabled = true;
                break;
            }
        }
        
        // If no wakeup pin is enabled and we're entering a mode that needs one,
        // enable at least one wakeup source (WKUP1 by default)
        if (!any_wakeup_enabled && 
            config.PowerConfig.power_mode == Platform::PWR::PowerMode::Standby) {
            config.PowerConfig.wakeup_pin_config[0] = true;
        }
    }
    
    // PVD (Power Voltage Detector) validation
    if (config.PowerConfig.pvd_enable) {
        // Ensure PVD level is within valid range (0-7)
        if (config.PowerConfig.pvd_level > 7) {
            config.PowerConfig.pvd_level = 7;
        }
    }
}
SystemConfigurator SystemConfigurator::FromExistingConfig(const SystemInitConfig& existingConfig) {
    SystemConfigurator configurator;
    configurator.config = existingConfig;
    return configurator;
}

// Clock configuration methods
SystemConfigurator& SystemConfigurator::withSystemClock(uint32_t frequency) {
    config.SysClockConfig.systemClockHz = frequency;
    
    // Auto-determine if we should use PLL based on frequency
    if (frequency > 16000000) { // More than HSI frequency
        config.SysClockConfig.clockSource = Platform::RCC::RccClockSource::PLL;
        config.SysClockConfig.pllConfig.autoCalculate = true;
    } else {
        config.SysClockConfig.clockSource = Platform::RCC::RccClockSource::HSI;
    }
    
    // Auto-adjust flash latency based on frequency
    if (frequency <= 24000000) {
        config.FlashConfig.latency = Platform::FLASH::FlashLatency::WS0;
    } else if (frequency <= 48000000) {
        config.FlashConfig.latency = Platform::FLASH::FlashLatency::WS1;
    } else if (frequency <= 72000000) {
        config.FlashConfig.latency = Platform::FLASH::FlashLatency::WS2;
    } else {
        config.FlashConfig.latency = Platform::FLASH::FlashLatency::WS3;
    }
    
    return *this;
}

// Example flash configuration method
SystemConfigurator& SystemConfigurator::enableInstructionCache(bool enable) {
    config.FlashConfig.icache_enable = enable;
    return *this;
}

// Example power configuration method
SystemConfigurator& SystemConfigurator::withPowerMode(Platform::PWR::PowerMode mode) {
    config.PowerConfig.power_mode = mode;
    
    // Automatically adjust related settings for certain power modes
    if (mode == Platform::PWR::PowerMode::LowPower) {
        // Lower voltage scaling for power savings
        config.PowerConfig.voltage_scale = Platform::PWR::VoltageScale::Scale3;
        
        // Reduce clock frequency for low power mode if it's high
        if (config.SysClockConfig.systemClockHz > 24000000) {
            config.SysClockConfig.systemClockHz = 24000000;
        }
    }
    
    return *this;
}

// Finalization methods
SystemInitConfig SystemConfigurator::build() {
    // Validate settings before returning
    validateClockSettings();
    validateFlashSettings();
    validatePowerSettings();
    ensureCompatibleSettings();
    
    return config;
}

Platform::Status SystemConfigurator::applyToSystem() {
    // Validate before applying
    validateClockSettings();
    validateFlashSettings();
    validatePowerSettings();
    ensureCompatibleSettings();
    
    // Apply the configuration to the system
    return SystemManager::GetInstance().Init(&config);
}

// Validation methods
void SystemConfigurator::validateClockSettings() {
    // Ensure clock frequency is within valid range
    if (config.SysClockConfig.systemClockHz > 84000000) {
        config.SysClockConfig.systemClockHz = 84000000; // Clamp to maximum
    }
    
    // Ensure PLL settings are valid if using PLL
    // Ensure PLL settings are valid if using PLL
    if (config.SysClockConfig.clockSource == Platform::RCC::RccClockSource::PLL) {
        if (!config.SysClockConfig.pllConfig.autoCalculate) {
            // Validate manual PLL settings
            
            // Validate PLLM (division factor) - must be between 2 and 63
            if (config.SysClockConfig.pllConfig.m < 2) {
                config.SysClockConfig.pllConfig.m = 2;
            } else if (config.SysClockConfig.pllConfig.m > 63) {
                config.SysClockConfig.pllConfig.m = 63;
            }
            
            // Validate PLLN (multiplication factor) - must be between 50 and 432
            if (config.SysClockConfig.pllConfig.n < 50) {
                config.SysClockConfig.pllConfig.n = 50;
            } else if (config.SysClockConfig.pllConfig.n > 432) {
                config.SysClockConfig.pllConfig.n = 432;
            }
            
            // Validate PLLP (division factor for system clock) - must be 2, 4, 6, or 8
            if (config.SysClockConfig.pllConfig.p != 2 && 
                config.SysClockConfig.pllConfig.p != 4 && 
                config.SysClockConfig.pllConfig.p != 6 && 
                config.SysClockConfig.pllConfig.p != 8) {
                // Default to nearest valid value
                if (config.SysClockConfig.pllConfig.p < 2) {
                    config.SysClockConfig.pllConfig.p = 2;
                } else if (config.SysClockConfig.pllConfig.p < 4) {
                    config.SysClockConfig.pllConfig.p = 4;
                } else if (config.SysClockConfig.pllConfig.p < 6) {
                    config.SysClockConfig.pllConfig.p = 6;
                } else {
                    config.SysClockConfig.pllConfig.p = 8;
                }
            }
            
            // Validate PLLQ (division factor for USB, etc.) - must be between 2 and 15
            if (config.SysClockConfig.pllConfig.q < 2) {
                config.SysClockConfig.pllConfig.q = 2;
            } else if (config.SysClockConfig.pllConfig.q > 15) {
                config.SysClockConfig.pllConfig.q = 15;
            }
            
            // Validate VCO input frequency (PLLM should result in 1-2 MHz VCO input)
            uint32_t vco_input_freq;
            if (config.SysClockConfig.pllConfig.source == Platform::RCC::RccClockSource::HSE) {
                vco_input_freq = config.SysClockConfig.hseFrequencyHz / config.SysClockConfig.pllConfig.m;
            } else {
                // HSI is fixed at 16 MHz
                vco_input_freq = 16000000 / config.SysClockConfig.pllConfig.m;
            }
            
            if (vco_input_freq < 1000000 || vco_input_freq > 2000000) {
                // Adjust PLLM to get VCO input in the valid range
                uint32_t source_freq = (config.SysClockConfig.pllConfig.source == Platform::RCC::RccClockSource::HSE) 
                    ? config.SysClockConfig.hseFrequencyHz 
                    : 16000000;
                    
                config.SysClockConfig.pllConfig.m = source_freq / 2000000; // Aim for 2 MHz
                if (config.SysClockConfig.pllConfig.m < 2) {
                    config.SysClockConfig.pllConfig.m = 2;
                }
            }
            
            // Validate VCO output frequency (PLLN should result in 100-432 MHz VCO output)
            uint32_t recalc_vco_input = (config.SysClockConfig.pllConfig.source == Platform::RCC::RccClockSource::HSE)
                ? (config.SysClockConfig.hseFrequencyHz / config.SysClockConfig.pllConfig.m)
                : (16000000 / config.SysClockConfig.pllConfig.m);
                
            uint32_t vco_output_freq = recalc_vco_input * config.SysClockConfig.pllConfig.n;
            
            if (vco_output_freq < 100000000 || vco_output_freq > 432000000) {
                // Adjust PLLN to get VCO output in the valid range
                if (vco_output_freq < 100000000) {
                    config.SysClockConfig.pllConfig.n = 100000000 / recalc_vco_input;
                    if (config.SysClockConfig.pllConfig.n < 50) {
                        config.SysClockConfig.pllConfig.n = 50;
                    }
                } else {
                    config.SysClockConfig.pllConfig.n = 432000000 / recalc_vco_input;
                    if (config.SysClockConfig.pllConfig.n > 432) {
                        config.SysClockConfig.pllConfig.n = 432;
                    }
                }
            }
            
            // Ensure system clock frequency matches what's expected from PLL settings
            uint32_t expected_sysclk = vco_output_freq / config.SysClockConfig.pllConfig.p;
            if (config.SysClockConfig.systemClockHz != expected_sysclk) {
                // Update system clock to match PLL settings
                config.SysClockConfig.systemClockHz = expected_sysclk;
            }
        }
    }
}

void SystemConfigurator::validateFlashSettings() {
    // Ensure flash latency is appropriate for the clock frequency
    uint32_t freq = config.SysClockConfig.systemClockHz;
    
    // Only auto-adjust if it doesn't match the frequency
    if ((config.FlashConfig.latency == Platform::FLASH::FlashLatency::WS0 && freq > 24000000) ||
        (config.FlashConfig.latency == Platform::FLASH::FlashLatency::WS1 && freq > 48000000) ||
        (config.FlashConfig.latency == Platform::FLASH::FlashLatency::WS2 && freq > 72000000)) {
        
        // Adjust latency to match clock frequency
        if (freq <= 24000000) {
            config.FlashConfig.latency = Platform::FLASH::FlashLatency::WS0;
        } else if (freq <= 48000000) {
            config.FlashConfig.latency = Platform::FLASH::FlashLatency::WS1;
        } else if (freq <= 72000000) {
            config.FlashConfig.latency = Platform::FLASH::FlashLatency::WS2;
        } else {
            config.FlashConfig.latency = Platform::FLASH::FlashLatency::WS3;
        }
    }
}

void SystemConfigurator::ensureCompatibleSettings() {
    // Ensure power mode and clock settings are compatible
    if (config.PowerConfig.power_mode == Platform::PWR::PowerMode::LowPower &&
        config.SysClockConfig.systemClockHz > 24000000) {
        
        // Low power mode requires lower clock frequency
        config.SysClockConfig.systemClockHz = 24000000;
        
        // Update flash latency to match reduced frequency
        config.FlashConfig.latency = Platform::FLASH::FlashLatency::WS0;
    }
    
    // Ensure bus prescalers won't exceed maximum frequency limits
    if (config.SysClockConfig.systemClockHz / config.SysClockConfig.apb1Prescaler > 42000000) {
        // APB1 must not exceed 42MHz, adjust prescaler if needed
        if (config.SysClockConfig.systemClockHz <= 84000000) {
            config.SysClockConfig.apb1Prescaler = 2; // Divide by 2
        } else {
            config.SysClockConfig.apb1Prescaler = 4; // Divide by 4
        }
    }
}

} // namespace SystemServices
} // namespace Middleware