// inc/middleware/system_init.hpp

#pragma once

#include "common/platform.hpp"
#include "hardware_abstraction/rcc.hpp" 
#include "middleware/middleware_module.hpp"
#include "hardware_abstraction/flash.hpp"
#include "hardware_abstraction/pwr.hpp"
#include "system_services/system_manager.hpp"   
#include "common/platform_cmsis.hpp"
#include "hardware_abstraction/systick.hpp"

namespace Middleware {
namespace SystemServices {


/**
 * @brief System initialization configuration
 */

 struct SystemInfo {
    uint32_t systemClock;           // Current system clock frequency in Hz
    uint32_t ahbClock;              // AHB bus clock frequency in Hz
    uint32_t apb1Clock;             // APB1 bus clock frequency in Hz
    uint32_t apb2Clock;             // APB2 bus clock frequency in Hz
    Platform::RCC::RccClockSource clockSource;        // Current clock source
    bool pllEnabled;                // Whether PLL is enabled
    bool mpuEnabled;                // Whether MPU is enabled
    bool fpuEnabled;                // Whether FPU is enabled
    bool icacheEnabled;             // Whether instruction cache is enabled
    bool dcacheEnabled;             // Whether data cache is enabled
    bool prefetchEnabled;           // Whether flash prefetch is enabled
    Platform::PWR::PowerMode currentPowerMode;     // Current power mode
    Platform::FLASH::FlashLatency flashLatency;           // Current flash latency (wait states)
    uint64_t uptimeMs;              // System uptime in milliseconds
    uint32_t resetCause;            // Cause of the last reset
};

struct SystemInitConfig {

    Platform::RCC::RccConfig SysClockConfig;    // System clock configuration
    Platform::FLASH::FlashConfig FlashConfig;   // Flash configuration
    Platform::PWR::PowerConfig PowerConfig;     // Power management configuration

    // Core peripheral configuration
    bool enableSysTick;             // Enable SysTick timer
    uint32_t sysTickInterval;       // SysTick interval in microseconds
    bool enableMPU;                 // Enable Memory Protection Unit
    bool enableFPU;                 // Enable Floating Point Unit
    
};

/**
 * @brief Reset causes
 */
enum class ResetCause {
    PowerOn,                        // Power-on reset
    External,                       // External reset pin
    Watchdog,                       // Watchdog reset
    Software,                       // Software reset
    LowPower,                       // Low-power reset
    BrownOut,                       // Brown-out reset
    Unknown                         // Unknown cause
};

/**
 * @brief Callback events for SystemInit
 */
enum class SystemEvent {
    ClockConfigured,                // System clock has been configured
    ResetDetected,                  // System reset has been detected
    PowerModeChanged,               // Power mode has changed
    ErrorDetected,                  // System error has been detected
    Count                           // Number of event types
};

/**
 * @brief System initialization module
 * 
 * This class provides system initialization services, including clock
 * configuration, flash setup, core peripheral initialization, and
 * power management.
 */
class SystemManager : public MiddlewareModule {
public:
    // Get singleton instance
    static SystemManager& GetInstance();
    
    // Default system configuration preset for common scenarios
    static SystemInitConfig GetDefaultConfig();
    static SystemInitConfig GetLowPowerConfig();
    static SystemInitConfig GetMaxPerformanceConfig();
    
    // Clock management functions
    virtual uint32_t GetSystemClock() const = 0;
    virtual uint32_t GetAHBClock() const = 0;
    virtual uint32_t GetAPB1Clock() const = 0;
    virtual uint32_t GetAPB2Clock() const = 0;
    virtual Platform::RCC::RccClockSource GetClockSource() const = 0;
    virtual bool IsPLLEnabled() const = 0;
    
    // Power management functions
    virtual Platform::Status SetPowerMode(Platform::PWR::PowerMode mode) = 0;
    virtual Platform::PWR::PowerMode GetPowerMode() const = 0;
    
    // System functions
    virtual Platform::Status SoftwareReset() = 0;
    virtual ResetCause GetResetCause() const = 0;
    virtual uint64_t GetUptime() const = 0;
    virtual Platform::Status GetSystemInfo(SystemInfo& info) const = 0;
    
    // MPU configuration (if enabled)
    virtual Platform::Status ConfigureMPURegion(uint8_t region, uint32_t address, 
                                               uint32_t size, uint32_t attributes) = 0;
    
    // Low-level register access (for advanced usage)
    virtual Platform::Status ReadSystemRegister(uint32_t reg_addr, uint32_t& value) const = 0;
    virtual Platform::Status WriteSystemRegister(uint32_t reg_addr, uint32_t value) = 0;
};

// Module parameter and information IDs
constexpr uint32_t SYSTEM_PARAM_CLOCK_SOURCE = 0x2501;
constexpr uint32_t SYSTEM_PARAM_PLL_CONFIG = 0x2502;
constexpr uint32_t SYSTEM_PARAM_FLASH_LATENCY = 0x2503;
constexpr uint32_t SYSTEM_PARAM_POWER_MODE = 0x2504;
constexpr uint32_t SYSTEM_PARAM_MPU_CONFIG = 0x2505;
constexpr uint32_t SYSTEM_PARAM_CACHE_CONTROL = 0x2506;

constexpr uint32_t SYSTEM_INFO_CLOCK_FREQUENCIES = 0x2601;
constexpr uint32_t SYSTEM_INFO_RESET_CAUSE = 0x2602;
constexpr uint32_t SYSTEM_INFO_SYSTEM_STATUS = 0x2603;
constexpr uint32_t SYSTEM_INFO_UPTIME = 0x2604;

// Event IDs for callback registration
constexpr uint32_t SYSTEM_EVENT_CLOCK_CHANGE = 0x2701;
constexpr uint32_t SYSTEM_EVENT_RESET = 0x2702;
constexpr uint32_t SYSTEM_EVENT_POWER_MODE = 0x2703;
constexpr uint32_t SYSTEM_EVENT_ERROR = 0x2704;

// Global convenience functions
inline uint32_t GetSystemClock() {
    return SystemManager::GetInstance().GetSystemClock();
}

inline uint32_t GetAHBClock() {
    return SystemManager::GetInstance().GetAHBClock();
}

inline uint32_t GetAPB1Clock() {
    return SystemManager::GetInstance().GetAPB1Clock();
}

inline uint32_t GetAPB2Clock() {
    return SystemManager::GetInstance().GetAPB2Clock();
}

inline Platform::PWR::PowerMode GetPowerMode() {
    return SystemManager::GetInstance().GetPowerMode();
}

inline uint64_t GetSystemUptime() {
    return SystemManager::GetInstance().GetUptime();
}

class SystemManagerImpl : public SystemManager {
    private:
        // Internal state tracking
        bool initialized;
        SystemInitConfig config;
        
        // System state information
        SystemInfo system_state;

        // Hardware interface references
        Platform::FLASH::FlashInterface* flash_interface;
        Platform::PWR::PowerInterface* power_interface;
        Platform::RCC::RccInterface* rcc_interface;
        Platform::CMSIS::SysTick::SysTickInterface* systick_interface;
        
        // Registered callbacks
        struct CallbackEntry {
            void (*callback)(void* param);
            void* user_data;
            bool active;
        };
    
        CallbackEntry callbacks[static_cast<size_t>(SystemEvent::Count)];
        
        // Private methods
        Platform::Status ConfigureSystemClock();
        Platform::Status ConfigureFlashLatency(uint32_t system_clock_freq, Platform::FLASH::FlashLatency latency);
        Platform::Status ConfigureSysTick(uint32_t interval_us);
        Platform::Status ConfigureMPU();
        Platform::Status ConfigureFPU();
        Platform::Status ConfigureCache();
        ResetCause DetermineResetCause();
        void ClearResetFlags();
        
    public:
        // Constructor/Destructor
        SystemManagerImpl();
        ~SystemManagerImpl();
        
        // MiddlewareModule interface implementation
        Platform::Status Init(void* config) override;
        Platform::Status Process(void* input, void* output) override;
        Platform::Status Configure(uint32_t param_id, void* value) override;
        Platform::Status GetInfo(uint32_t info_id, void* buffer, uint32_t* size) override;
        Platform::Status Reset() override;
        Platform::Status RegisterCallback(uint32_t event, void (*callback)(void* param), void* param) override;
        
        // SystemInit interface implementation
        uint32_t GetSystemClock() const override;
        uint32_t GetAHBClock() const override;
        uint32_t GetAPB1Clock() const override;
        uint32_t GetAPB2Clock() const override;
        Platform::RCC::RccClockSource GetClockSource() const override;
        bool IsPLLEnabled() const override;
        
        Platform::Status SetPowerMode(Platform::PWR::PowerMode mode) override;
        Platform::PWR::PowerMode GetPowerMode() const override;
        
        Platform::Status SoftwareReset() override;
        ResetCause GetResetCause() const override;
        uint64_t GetUptime() const override;
        Platform::Status GetSystemInfo(SystemInfo& info) const override;
        
        Platform::Status ConfigureMPURegion(uint8_t region, uint32_t address, 
                                           uint32_t size, uint32_t attributes) override;
        
        Platform::Status ReadSystemRegister(uint32_t reg_addr, uint32_t& value) const override;
        Platform::Status WriteSystemRegister(uint32_t reg_addr, uint32_t value) override;

    };
} // namespace Middleware
} // namespace SystemServices