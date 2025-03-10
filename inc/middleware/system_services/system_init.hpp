﻿// inc/middleware/system_init.hpp

#pragma once

#include "common/platform.hpp"
#include "middleware/middleware_module.hpp"
#include "hardware_abstraction/flash.hpp"
#include "hardware_abstraction/pwr.hpp"
namespace Middleware {
namespace SystemServices {

/**
 * @brief Power mode configuration
 */
enum class PowerMode {
    Run,            // Normal run mode
    LowPower,       // Low power run mode
    Sleep,          // Sleep mode (CPU off, peripherals on)
    DeepSleep,      // Deep sleep mode (CPU off, some peripherals off)
    Standby         // Standby mode (lowest power, only RTC and backup registers)
};



/**
 * @brief System initialization configuration
 */
struct SystemInitConfig {

    Platform::RCC::RccConfig SysClockConfig;
    Platform::FLASH::FlashConfig FlashConfig;
    Platform::PWR::PowerConfig PowerConfig;

    // Bus dividers
    uint8_t ahbDivider;             // AHB bus clock divider (1, 2, 4, 8, 16, 64, 128, 256, 512)
    uint8_t apb1Divider;            // APB1 bus clock divider (1, 2, 4, 8, 16)
    uint8_t apb2Divider;            // APB2 bus clock divider (1, 2, 4, 8, 16)
    
    // Core peripheral configuration
    bool enableSysTick;             // Enable SysTick timer
    uint32_t sysTickInterval;       // SysTick interval in microseconds
    bool enableMPU;                 // Enable Memory Protection Unit
    bool enableFPU;                 // Enable Floating Point Unit
    bool enableICache;              // Enable instruction cache
    bool enableDCache;              // Enable data cache
    bool enablePrefetch;            // Enable flash prefetch
    
    // Power management
    PowerMode initialPowerMode;     // Initial power mode after initialization
};

/**
 * @brief System information structure
 */
struct SystemInfo {
    uint32_t systemClock;           // Current system clock frequency in Hz
    uint32_t ahbClock;              // AHB bus clock frequency in Hz
    uint32_t apb1Clock;             // APB1 bus clock frequency in Hz
    uint32_t apb2Clock;             // APB2 bus clock frequency in Hz
    ClockSource clockSource;        // Current clock source
    bool pllEnabled;                // Whether PLL is enabled
    bool mpuEnabled;                // Whether MPU is enabled
    bool fpuEnabled;                // Whether FPU is enabled
    bool icacheEnabled;             // Whether instruction cache is enabled
    bool dcacheEnabled;             // Whether data cache is enabled
    bool prefetchEnabled;           // Whether flash prefetch is enabled
    PowerMode currentPowerMode;     // Current power mode
    uint8_t flashLatency;           // Current flash latency (wait states)
    uint64_t uptimeMs;              // System uptime in milliseconds
    uint32_t resetCause;            // Cause of the last reset
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
class SystemInit : public MiddlewareModule {
public:
    // Get singleton instance
    static SystemInit& GetInstance();
    
    // Default system configuration preset for common scenarios
    static SystemInitConfig GetDefaultConfig();
    static SystemInitConfig GetLowPowerConfig();
    static SystemInitConfig GetMaxPerformanceConfig();
    
    // Clock management functions
    virtual uint32_t GetSystemClock() const = 0;
    virtual uint32_t GetAHBClock() const = 0;
    virtual uint32_t GetAPB1Clock() const = 0;
    virtual uint32_t GetAPB2Clock() const = 0;
    virtual ClockSource GetClockSource() const = 0;
    virtual bool IsPLLEnabled() const = 0;
    
    // Power management functions
    virtual Platform::Status SetPowerMode(PowerMode mode) = 0;
    virtual PowerMode GetPowerMode() const = 0;
    
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
    return SystemInit::GetInstance().GetSystemClock();
}

inline uint32_t GetAHBClock() {
    return SystemInit::GetInstance().GetAHBClock();
}

inline uint32_t GetAPB1Clock() {
    return SystemInit::GetInstance().GetAPB1Clock();
}

inline uint32_t GetAPB2Clock() {
    return SystemInit::GetInstance().GetAPB2Clock();
}

inline PowerMode GetPowerMode() {
    return SystemInit::GetInstance().GetPowerMode();
}

inline uint64_t GetSystemUptime() {
    return SystemInit::GetInstance().GetUptime();
}

class SystemInitImpl : public SystemInit {
    private:
        // Internal state tracking
        bool initialized;
        SystemInitConfig config;
        
        // System state information
        struct {
            uint32_t system_clock_freq;
            uint32_t ahb_clock_freq;
            uint32_t apb1_clock_freq;
            uint32_t apb2_clock_freq;
            ClockSource clock_source;
            bool pll_enabled;
            PowerMode current_power_mode;
            uint8_t flash_latency;
            uint32_t reset_cause;
            bool mpu_enabled;
            bool fpu_enabled;
            bool icache_enabled;
            bool dcache_enabled;
            bool prefetch_enabled;
        } system_state;
        
        // Hardware interface references
        std::shared_ptr<Platform::RCC::RccInterface> rcc_interface;
        std::shared_ptr<SysTickInterface> systick_interface;
        
        // Registered callbacks
        struct CallbackEntry {
            void (*callback)(void* param);
            void* user_data;
            bool active;
        };
    
        CallbackEntry callbacks[static_cast<size_t>(SystemEvent::Count)];
        
        // Private methods
        Platform::Status ConfigureSystemClock();
        Platform::Status ConfigureFlashLatency(uint32_t system_clock_freq, FlashLatency latency);
        Platform::Status ConfigureSysTick(uint32_t interval_us);
        Platform::Status ConfigureMPU();
        Platform::Status ConfigureFPU();
        Platform::Status ConfigureCache();
        ResetCause DetermineResetCause();
        void ClearResetFlags();
        
    public:
        // Constructor/Destructor
        SystemInitImpl();
        ~SystemInitImpl();
        
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
        ClockSource GetClockSource() const override;
        bool IsPLLEnabled() const override;
        
        Platform::Status SetPowerMode(PowerMode mode) override;
        PowerMode GetPowerMode() const override;
        
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