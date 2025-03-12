#pragma once

#include "hw_interface.hpp"
#include "common/platform.hpp"
#include "common/platform_rcc.hpp"


namespace Platform {
namespace RCC {

/**
 * Clock source type enumeration
 */
enum class RccClockSource : uint32_t {
    HSI,  // High-speed internal oscillator (16 MHz)
    HSE,  // High-speed external oscillator (crystal)
    PLL   // Phase-locked loop
};

/**
 * PLL configuration structure
 */
struct RccPllConfig {
    bool autoCalculate;        // Whether to auto-calculate PLL settings from target frequency
    uint32_t m;                // PLLM: Division factor for PLL input (2-63)
    uint32_t n;                // PLLN: Multiplication factor for VCO (50-432)
    uint32_t p;                // PLLP: Division factor for main system clock (2, 4, 6, or 8)
    uint32_t q;                // PLLQ: Division factor for USB, SDIO, etc. (2-15)
    RccClockSource source;     // PLL source (HSI or HSE)
};

enum RccResetFlags : uint32_t {
    RCC_RESET_FLAG_WWDGRST = 0x80000000,
    RCC_RESET_FLAG_IWDGRST = 0x40000000,
    RCC_RESET_FLAG_SFTRST = 0x20000000,
    RCC_RESET_FLAG_PORRST = 0x10000000,
    RCC_RESET_FLAG_PINRST = 0x08000000,
    RCC_RESET_FLAG_BORRST = 0x04000000,
    RCC_RESET_FLAG_LPWRRST = 0x02000000
};
/**
 * RCC configuration structure
 */
struct RccConfig {
    uint32_t systemClockHz;    // Desired system clock frequency in Hz
    RccClockSource clockSource; // System clock source
    uint32_t hseFrequencyHz;   // External oscillator frequency (if used)
    
    // Bus prescalers
    uint32_t ahbPrescaler;     // AHB clock divider (1, 2, 4, 8, 16, 64, 128, 256, 512)
    uint32_t apb1Prescaler;    // APB1 clock divider (1, 2, 4, 8, 16)
    uint32_t apb2Prescaler;    // APB2 clock divider (1, 2, 4, 8, 16)
    
    // PLL configuration (if used)
    RccPllConfig pllConfig;    // PLL configuration
};
    struct SysCLockFreqs {
        uint32_t systemClock;
        uint32_t ahbClock;
        uint32_t apb1Clock;
        uint32_t apb2Clock;
    };
/**
 * RCC hardware interface implementation
 * Provides configuration and control of system clocks and peripheral clocks
 */
class RccInterface : public HwInterface {
private:
    // Internal state tracking
    bool initialized;
    
    // Clock configuration state
    RccConfig activeConfig;

    // Clock frequency state
    SysCLockFreqs clockFreqs;

    // Peripheral clock state tracking for efficient power management
    uint32_t enabledPeripheralsAHB1;
    uint32_t enabledPeripheralsAHB2;
    uint32_t enabledPeripheralsAPB1;
    uint32_t enabledPeripheralsAPB2;
    
    // Helper methods
    Platform::Status ConfigureSystemClock(const RccConfig& config);
    Platform::Status CalculatePllSettings(RccConfig& config);
    Platform::Status ConfigureFlashLatency(uint32_t systemClockHz);
    
public:    

    // Singleton pattern for RCC interface (only need one instance)
    static RccInterface& GetInstance();
    // Constructor
    RccInterface();
    
    // Destructor
    ~RccInterface() override;
    
    // Interface implementation
    Platform::Status Init(void* config) override;
    Platform::Status DeInit() override;
    Platform::Status Control(uint32_t command, void* param) override;
    Platform::Status Read(void* buffer, uint16_t size, uint32_t timeout) override;
    Platform::Status Write(const void* data, uint16_t size, uint32_t timeout) override;
    Platform::Status RegisterCallback(uint32_t eventId, void (*callback)(void* param), void* param) override;
    
    // RCC-specific methods
    Platform::Status EnablePeripheralClock(Platform::RCC::RccPeripheral peripheral);
    Platform::Status DisablePeripheralClock(Platform::RCC::RccPeripheral peripheral);
    Platform::Status ConfigureClockFrequency(uint32_t frequencyHz);
    Platform::Status IsPeripheralClockEnabled(Platform::RCC::RccPeripheral peripheral, bool& enabled) const;
    Platform::Status ClearResetFlags(void);

    // Clock frequency getters
    uint32_t GetSystemClockFrequency() const;
    uint32_t GetAhbClockFrequency() const;
    uint32_t GetApb1ClockFrequency() const;
    uint32_t GetApb2ClockFrequency() const;
    
    uint32_t GetResetFlags() const;

    // Default configuration creator
    static RccConfig CreateDefaultConfig();
    

};

// RCC control command identifiers
constexpr uint32_t RCC_CTRL_ENABLE_PERIPHERAL = 0x0401;
constexpr uint32_t RCC_CTRL_DISABLE_PERIPHERAL = 0x0402;
constexpr uint32_t RCC_CTRL_GET_CLOCK_FREQUENCY = 0x0403;
constexpr uint32_t RCC_CTRL_SET_CLOCK_FREQUENCY = 0x0404;
constexpr uint32_t RCC_CTRL_ENTER_LOW_POWER = 0x0405;
constexpr uint32_t RCC_CTRL_EXIT_LOW_POWER = 0x0406;
constexpr uint32_t RCC_CTRL_GET_RESET_FLAGS = 0x0407;
constexpr uint32_t RCC_CTRL_CONFIGURE_SYSTEM_CLOCK = 0x0408;
constexpr uint32_t RCC_CTRL_GET_ALL_CLOCK_FREQUENCIES = 0x0409;
constexpr uint32_t RCC_CTRL_GET_CLOCK_SOURCE = 0x040A;
constexpr uint32_t RCC_CTRL_CLEAR_RESET_FLAGS = 0x040B;

}// namespace RCC       
}// namespace Platform
