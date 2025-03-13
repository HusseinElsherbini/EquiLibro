// src/middleware/system_init.cpp

#include "system_services/system_init.hpp"
#include "hardware_abstraction/rcc.hpp"
#include "hardware_abstraction/systick.hpp"
#include "hardware_abstraction/gpio.hpp"
#include "common/platform_flash.hpp"
//#include "hardware_abstraction/pwr.hpp"
#include "common/platform.hpp"
#include "common/platform_cmsis.hpp"
#include <cstring>
#include <memory>

namespace Middleware {
namespace SystemServices {
// Static instance for singleton pattern
SystemInit& SystemInit::GetInstance() {
    static SystemInitImpl instance;
    return instance;
}

// Default configuration preset
SystemInitConfig SystemInit::GetDefaultConfig() {
    SystemInitConfig config = {};
    
    // STM32F401 defaults

    config.SysClockConfig.systemClockHz = 84000000;  // 84 MHz
    config.SysClockConfig.clockSource = Platform::RCC::RccClockSource::PLL;
    config.SysClockConfig.hseFrequencyHz = 25000000;  // 25 MHz external crystal
    config.SysClockConfig.pllConfig.autoCalculate = true;
    config.SysClockConfig.pllConfig.source = Platform::RCC::RccClockSource::HSE;

    config.SysClockConfig.pllConfig.m = 25;    // 25MHz / 25 = 1MHz PLL input
    config.SysClockConfig.pllConfig.n = 336;   // 1MHz * 336 = 336MHz VCO
    config.SysClockConfig.pllConfig.p = 4;     // 336MHz / 4 = 84MHz system clock
    config.SysClockConfig.pllConfig.q = 7;     // 336MHz / 7 = 48MHz (for USB)

    config.SysClockConfig.ahbPrescaler = 1;
    config.SysClockConfig.apb1Prescaler = 2;  // APB1 is limited to 42MHz max
    config.SysClockConfig.apb2Prescaler = 1;

    config.FlashConfig.latency = Platform::FLASH::FlashLatency::WS4;
    config.FlashConfig.prefetch_enable = true;
    config.FlashConfig.icache_enable = true;
    config.FlashConfig.dcache_enable = true;

    config.PowerConfig.voltage_scale = Platform::PWR::VoltageScale::Scale1;
    config.PowerConfig.regulator_mode = Platform::PWR::RegulatorMode::Normal;
    config.PowerConfig.power_mode = Platform::PWR::PowerMode::Run;

    config.enableSysTick = true;
    config.sysTickInterval = 1000;  // 1ms SysTick interval
    config.enableMPU = false;
    config.enableFPU = true;

    
    return config;
}

// Low power configuration preset
SystemInitConfig SystemInit::GetLowPowerConfig() {

    SystemInitConfig config = GetDefaultConfig();
    
    // Reduce clock frequency for lower power
    config.SysClockConfig.systemClockHz = 16000000;       // 16 MHz
    config.SysClockConfig.clockSource = Platform::RCC::RccClockSource::HSI;     // Use internal oscillator
    config.SysClockConfig.pllConfig.autoCalculate = false;                  // No PLL needed
    
    // Adjust flash latency for lower frequency
    config.FlashConfig.latency = Platform::FLASH::FlashLatency::WS0;   // 0 wait states for 16MHz
    config.FlashConfig.dcache_enable = false;                         // Disable data cache
    config.FlashConfig.icache_enable = false;                         // Disable instruction cache

    // Bus dividers optimized for low power
    config.SysClockConfig.ahbPrescaler = 1;                     // AHB = SYSCLK
    config.SysClockConfig.apb1Prescaler = 1;                    // APB1 = HCLK
    config.SysClockConfig.apb2Prescaler = 1;                    // APB2 = HCLK

    // Set low power mode
    config.PowerConfig.power_mode = Platform::PWR::PowerMode::LowPower;
    
    return config;
}

// Maximum performance configuration preset
SystemInitConfig SystemInit::GetMaxPerformanceConfig() {

    SystemInitConfig config = GetDefaultConfig();
    
    // Maximum clock for STM32F401
    config.SysClockConfig.systemClockHz = 84000000;  // 84 MHz
    config.SysClockConfig.clockSource = Platform::RCC::RccClockSource::PLL;
    config.SysClockConfig.hseFrequencyHz = 25000000;  // 25 MHz external crystal
    config.SysClockConfig.pllConfig.autoCalculate = true;
    config.SysClockConfig.pllConfig.source = Platform::RCC::RccClockSource::HSE;

    
    // Enable all performance enhancing features
    config.FlashConfig.prefetch_enable = true;
    config.FlashConfig.icache_enable = true;
    config.FlashConfig.dcache_enable = true;

    config.enableFPU = true;
    
    return config;
}

// Constructor
SystemInitImpl::SystemInitImpl()
    : initialized(false) {
    
    // Initialize state tracking
    system_state.system_clock_freq = 16000000; // Default to HSI
    system_state.ahb_clock_freq = 16000000;
    system_state.apb1_clock_freq = 16000000;
    system_state.apb2_clock_freq = 16000000;
    system_state.clock_source = Platform::RCC::RccClockSource::HSI;
    system_state.pll_enabled = false;
    system_state.current_power_mode = Platform::PWR::PowerMode::Run;
    system_state.flash_latency = 0;
    system_state.reset_cause = static_cast<uint32_t>(ResetCause::PowerOn);
    system_state.mpu_enabled = false;
    system_state.fpu_enabled = false;
    system_state.icache_enabled = false;
    system_state.dcache_enabled = false;
    system_state.prefetch_enabled = false;
    
    // Initialize callbacks
    for (auto& callback : callbacks) {
        callback.callback = nullptr;
        callback.user_data = nullptr;
        callback.active = false;
    }
}

// Destructor
SystemInitImpl::~SystemInitImpl() {
    if (initialized) {
        // No specific cleanup needed
    }
}

// Initialize the system
Platform::Status SystemInitImpl::Init(void* config) {
    if (initialized) {
        return Platform::Status::OK; // Already initialized
    }
    
    // Validate input configuration
    if (config == nullptr) {
        // Use default configuration if none provided
        this->config = GetDefaultConfig();
    } else {
        this->config = *static_cast<SystemInitConfig*>(config);
    }
        // Acquire hardware interfaces
    rcc_interface = &Platform::RCC::RccInterface::GetInstance();
    if (!rcc_interface) {
        return Platform::Status::ERROR;
    }
    
    power_interface = &Platform::PWR::PowerInterface::GetInstance();
    if (!power_interface) {
        return Platform::Status::ERROR;
    }
    
    flash_interface = &Platform::FLASH::FlashInterface::GetInstance();
    if (!flash_interface) {
        return Platform::Status::ERROR;
    }
    
    systick_interface = &Platform::CMSIS::SysTick::SysTickInterface::GetInstance();
    if (!systick_interface) {
        return Platform::Status::ERROR;
    }
    // Determine the cause of the reset
    system_state.reset_cause = static_cast<uint32_t>(DetermineResetCause());
    ClearResetFlags();

    // Configure the system clock
    Platform::Status status = ConfigureSystemClock();
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Configure flash latency based on system clock
    status = ConfigureFlashLatency(system_state.system_clock_freq, this->config.FlashConfig.latency);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Configure SysTick if requested
    if (this->config.enableSysTick) {
        status = ConfigureSysTick(this->config.sysTickInterval);
        if (status != Platform::Status::OK) {
            return status;
        }
    }
    
    // Configure MPU if requested
    if (this->config.enableMPU) {
        status = ConfigureMPU();
        if (status != Platform::Status::OK) {
            return status;
        }
    }
    
    // Configure FPU if requested
    if (this->config.enableFPU) {
        status = ConfigureFPU();
        if (status != Platform::Status::OK) {
            return status;
        }
    }
    
    // Configure cache settings
    status = ConfigureCache();
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Set initial power mode
    status = SetPowerMode(this->config.PowerConfig.power_mode);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Trigger clock configured callback if registered
    if (callbacks[static_cast<uint32_t>(SystemEvent::ClockConfigured)].active) {
        callbacks[static_cast<uint32_t>(SystemEvent::ClockConfigured)].callback(
            callbacks[static_cast<uint32_t>(SystemEvent::ClockConfigured)].user_data
        );
    }
    
    initialized = true;
    return Platform::Status::OK;
}

// Process function - for periodic tasks
Platform::Status SystemInitImpl::Process(void* input, void* output) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // No regular processing needed for this module
    return Platform::Status::OK;
}

// Configure system parameters
Platform::Status SystemInitImpl::Configure(uint32_t param_id, void* value) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (value == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    switch (param_id) {
        case SYSTEM_PARAM_CLOCK_SOURCE: {
            Platform::RCC::RccClockSource* new_source = static_cast<Platform::RCC::RccClockSource*>(value);
            config.SysClockConfig.clockSource = *new_source;
            return ConfigureSystemClock();
        }
        
        case SYSTEM_PARAM_PLL_CONFIG: {
            Platform::RCC::RccPllConfig* new_config = static_cast<Platform::RCC::RccPllConfig*>(value);
            config.SysClockConfig.pllConfig = *new_config;
            return ConfigureSystemClock();
        }
        
        case SYSTEM_PARAM_FLASH_LATENCY: {
            Platform::FLASH::FlashLatency* new_latency = static_cast<Platform::FLASH::FlashLatency*>(value);
            config.FlashConfig.latency = *new_latency;
            return ConfigureFlashLatency(system_state.system_clock_freq, *new_latency);
        }
        
        case SYSTEM_PARAM_POWER_MODE: {
            Platform::PWR::PowerMode* new_mode = static_cast<Platform::PWR::PowerMode*>(value);
            return SetPowerMode(*new_mode);
        }
        
        case SYSTEM_PARAM_CACHE_CONTROL: {
            // Example structure for cache control
            Platform::FLASH::FlashConfig* cache_config = static_cast<Platform::FLASH::FlashConfig*>(value);
            config.FlashConfig.icache_enable = cache_config->icache_enable;
            config.FlashConfig.dcache_enable = cache_config->dcache_enable;
            config.FlashConfig.prefetch_enable = cache_config->prefetch_enable;
            return ConfigureCache();
        }
        
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
}

// Get system information
Platform::Status SystemInitImpl::GetInfo(uint32_t info_id, void* buffer, uint32_t* size) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (buffer == nullptr || size == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    switch (info_id) {
        case MIDDLEWARE_INFO_VERSION:
            // Return version information
            if (*size < sizeof(uint32_t)) {
                return Platform::Status::BUFFER_OVERFLOW;
            }
            *static_cast<uint32_t*>(buffer) = 0x00010000; // Version 1.0.0
            *size = sizeof(uint32_t);
            return Platform::Status::OK;
            
        case SYSTEM_INFO_CLOCK_FREQUENCIES: {
            // Return clock frequencies structure
            struct ClockFrequencies {
                uint32_t system_clock;
                uint32_t ahb_clock;
                uint32_t apb1_clock;
                uint32_t apb2_clock;
            };
            
            if (*size < sizeof(ClockFrequencies)) {
                return Platform::Status::BUFFER_OVERFLOW;
            }
            
            ClockFrequencies* freqs = static_cast<ClockFrequencies*>(buffer);
            freqs->system_clock = system_state.system_clock_freq;
            freqs->ahb_clock = system_state.ahb_clock_freq;
            freqs->apb1_clock = system_state.apb1_clock_freq;
            freqs->apb2_clock = system_state.apb2_clock_freq;
            *size = sizeof(ClockFrequencies);
            return Platform::Status::OK;
        }
            
        case SYSTEM_INFO_RESET_CAUSE:
            // Return reset cause
            if (*size < sizeof(uint32_t)) {
                return Platform::Status::BUFFER_OVERFLOW;
            }
            *static_cast<uint32_t*>(buffer) = system_state.reset_cause;
            *size = sizeof(uint32_t);
            return Platform::Status::OK;
            
        case SYSTEM_INFO_SYSTEM_STATUS: {
            // Return complete system info
            if (*size < sizeof(SystemInfo)) {
                return Platform::Status::BUFFER_OVERFLOW;
            }
            
            SystemInfo* info = static_cast<SystemInfo*>(buffer);
            info->systemClock = system_state.system_clock_freq;
            info->ahbClock = system_state.ahb_clock_freq;
            info->apb1Clock = system_state.apb1_clock_freq;
            info->apb2Clock = system_state.apb2_clock_freq;
            info->clockSource = system_state.clock_source;
            info->pllEnabled = system_state.pll_enabled;
            info->mpuEnabled = system_state.mpu_enabled;
            info->fpuEnabled = system_state.fpu_enabled;
            info->icacheEnabled = system_state.icache_enabled;
            info->dcacheEnabled = system_state.dcache_enabled;
            info->prefetchEnabled = system_state.prefetch_enabled;
            info->currentPowerMode = system_state.current_power_mode;
            info->flashLatency = system_state.flash_latency;
            info->uptimeMs = GetUptime();
            info->resetCause = system_state.reset_cause;
            
            *size = sizeof(SystemInfo);
            return Platform::Status::OK;
        }
            
        case SYSTEM_INFO_UPTIME:
            // Return system uptime
            if (*size < sizeof(uint64_t)) {
                return Platform::Status::BUFFER_OVERFLOW;
            }
            *static_cast<uint64_t*>(buffer) = GetUptime();
            *size = sizeof(uint64_t);
            return Platform::Status::OK;
            
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
}

// Reset the system
Platform::Status SystemInitImpl::Reset() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Trigger reset callback if registered
    if (callbacks[static_cast<uint32_t>(SystemEvent::ResetDetected)].active) {
        callbacks[static_cast<uint32_t>(SystemEvent::ResetDetected)].callback(
            callbacks[static_cast<uint32_t>(SystemEvent::ResetDetected)].user_data
        );
    }
    
    // Perform software reset
    return SoftwareReset();
}

// Register a callback for system events
Platform::Status SystemInitImpl::RegisterCallback(uint32_t event, void (*callback)(void* param), void* param) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Map event ID to array index
    uint32_t index;
    switch (event) {
        case SYSTEM_EVENT_CLOCK_CHANGE:
            index = static_cast<uint32_t>(SystemEvent::ClockConfigured);
            break;
        case SYSTEM_EVENT_RESET:
            index = static_cast<uint32_t>(SystemEvent::ResetDetected);
            break;
        case SYSTEM_EVENT_POWER_MODE:
            index = static_cast<uint32_t>(SystemEvent::PowerModeChanged);
            break;
        case SYSTEM_EVENT_ERROR:
            index = static_cast<uint32_t>(SystemEvent::ErrorDetected);
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Register the callback
    callbacks[index].callback = callback;
    callbacks[index].user_data = param;
    callbacks[index].active = (callback != nullptr);
    
    return Platform::Status::OK;
}

// Get the current system clock frequency
uint32_t SystemInitImpl::GetSystemClock() const {
    return system_state.system_clock_freq;
}

// Get the current AHB clock frequency
uint32_t SystemInitImpl::GetAHBClock() const {
    return system_state.ahb_clock_freq;
}

// Get the current APB1 clock frequency
uint32_t SystemInitImpl::GetAPB1Clock() const {
    return system_state.apb1_clock_freq;
}

// Get the current APB2 clock frequency
uint32_t SystemInitImpl::GetAPB2Clock() const {
    return system_state.apb2_clock_freq;
}

// Get the current clock source
Platform::RCC::RccClockSource SystemInitImpl::GetClockSource() const {
    return system_state.clock_source;
}

// Check if PLL is enabled
bool SystemInitImpl::IsPLLEnabled() const {
    return system_state.pll_enabled;
}

// Set the system power mode
Platform::Status SystemInitImpl::SetPowerMode(Platform::PWR::PowerMode mode) {

    using namespace Platform::PWR;
    if (!initialized && mode != Platform::PWR::PowerMode::Run) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Only implement the modes that are supported on STM32F401
    switch (mode) {
        case PowerMode::Run:
            // Normal run mode - exit any low power modes
            // Implementation depends on the current mode
            break;
            
        case PowerMode::LowPower:
            // Low power run mode - reduce clock frequency
            // Requires VOS bit in PWR_CR register
            {
                // Set voltage scaling to range 2 (lower power)
                // This requires accessing PWR peripheral registers
                // Read PWR_CR register to modify VOS bits
                volatile Registers* pwr_regs = getRegisters();

                // Set VOS bits to range 2
                pwr_regs->CR = (pwr_regs->CR & ~(static_cast<uint32_t>(CR::VOS_MSK))) | (static_cast<uint32_t>(CR::VOS_SCALE2));
                
                // Wait for stabilization
                while (!(pwr_regs->CSR & (static_cast<uint32_t>(CSR::VOSRDY)))) {
                    // Wait for VOSRDY bit
                }
            }
            break;
            
        case PowerMode::Sleep:
            // Execute WFI (Wait For Interrupt) instruction
            __asm volatile("wfi");
            return Platform::Status::OK;
            
        case PowerMode::DeepSleep:
            // Set SLEEPDEEP bit in System Control Register
            Platform::CMSIS::SCB::getRegisters()->SCR |= (1 << 2);
            
            // Execute WFI (Wait For Interrupt) instruction
            __asm volatile("wfi");
            
            // Clear SLEEPDEEP bit after waking up
            Platform::CMSIS::SCB::getRegisters()->SCR &= ~(1 << 2);
            return Platform::Status::OK;
            
        case PowerMode::Standby:
            // Set SLEEPDEEP bit and PDDS bit
            Platform::CMSIS::SCB::getRegisters()->SCR |= (1 << 2); // SLEEPDEEP
            
            // Set PDDS bit in PWR_CR
            volatile Registers* pwr_regs = getRegisters();
            pwr_regs->CR |= (static_cast<uint32_t>(CR::PDDS)); // PDDS
            
            // Execute WFI instruction
            __asm volatile("wfi");
            return Platform::Status::OK;
            
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
    
    // Update the current power mode state
    system_state.current_power_mode = mode;
    
    // Trigger power mode change callback if registered
    if (callbacks[static_cast<uint32_t>(SystemEvent::PowerModeChanged)].active) {
        callbacks[static_cast<uint32_t>(SystemEvent::PowerModeChanged)].callback(
            callbacks[static_cast<uint32_t>(SystemEvent::PowerModeChanged)].user_data
        );
    }
    
    return Platform::Status::OK;
}

// Get the current power mode
Platform::PWR::PowerMode SystemInitImpl::GetPowerMode() const {
    return system_state.current_power_mode;
}

// Perform a software reset
Platform::Status SystemInitImpl::SoftwareReset() {
    // Trigger NVIC system reset
    Platform::CMSIS::SCB::getRegisters()->AIRCR = (0x5FA << 16) | (1 << 2);
    
    // This should never be reached, but added for completeness
    while (true) {
        // Wait for reset to occur
    }
    
    return Platform::Status::OK;
}

// Get the cause of the last reset
ResetCause SystemInitImpl::GetResetCause() const {
    return static_cast<ResetCause>(system_state.reset_cause);
}

// Get system uptime in milliseconds
uint64_t SystemInitImpl::GetUptime() const {
    // Use SysTick to get uptime if available
    if (systick_interface) {
        // Number of SysTick ticks since system start
        uint64_t ticks = 0;
        systick_interface->Read(&ticks, sizeof(ticks), 0);
        
        // Convert to milliseconds based on SysTick interval
        return ticks * (config.sysTickInterval / 1000);
    }
    
    // Alternative implementation if SysTick not available
    // Here you could use a hardware timer or other source
    return 0;
}

// Get comprehensive system information
Platform::Status SystemInitImpl::GetSystemInfo(SystemInfo& info) const {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Fill in the system information structure
    info.systemClock = system_state.system_clock_freq;
    info.ahbClock = system_state.ahb_clock_freq;
    info.apb1Clock = system_state.apb1_clock_freq;
    info.apb2Clock = system_state.apb2_clock_freq;
    info.clockSource = system_state.clock_source;
    info.pllEnabled = system_state.pll_enabled;
    info.mpuEnabled = system_state.mpu_enabled;
    info.fpuEnabled = system_state.fpu_enabled;
    info.icacheEnabled = system_state.icache_enabled;
    info.dcacheEnabled = system_state.dcache_enabled;
    info.prefetchEnabled = system_state.prefetch_enabled;
    info.currentPowerMode = system_state.current_power_mode;
    info.flashLatency = system_state.flash_latency;
    info.uptimeMs = GetUptime();
    info.resetCause = system_state.reset_cause;
    
    return Platform::Status::OK;
}

// Configure an MPU region
Platform::Status SystemInitImpl::ConfigureMPURegion(uint8_t region, uint32_t address, 
                                                   uint32_t size, uint32_t attributes) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (!system_state.mpu_enabled) {
        return Platform::Status::ERROR;
    }
    
    // Check for valid region number (STM32F4 has 8 MPU regions)
    if (region >= 8) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Get a pointer to the MPU registers
    volatile uint32_t* mpu_rbar = reinterpret_cast<volatile uint32_t*>(0xE000ED9C);
    volatile uint32_t* mpu_rasr = reinterpret_cast<volatile uint32_t*>(0xE000EDA0);
    
    // Calculate size register value (must be power of 2)
    uint32_t size_bits = 0;
    uint32_t size_val = size;
    
    if (size_val < 32) {
        return Platform::Status::INVALID_PARAM; // Minimum region size is 32 bytes
    }
    
    while ((size_val & 1) == 0) {
        size_val >>= 1;
        size_bits++;
    }
    
    if (size_val != 1) {
        return Platform::Status::INVALID_PARAM; // Size must be power of 2
    }
    
    // Select the region
    *mpu_rbar = (address & 0xFFFFFFE0) | (region & 0xF) | (1 << 4);
    
    // Set the region attributes
    *mpu_rasr = attributes | ((size_bits - 1) << 1) | (1 << 0); // SIZE and ENABLE bits
    
    return Platform::Status::OK;
}

// Read from a system register
Platform::Status SystemInitImpl::ReadSystemRegister(uint32_t reg_addr, uint32_t& value) const {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Check if the register address is valid
    if (reg_addr % 4 != 0) {
        return Platform::Status::INVALID_PARAM; // Must be 4-byte aligned
    }
    
    // Read from the register address
    value = *reinterpret_cast<volatile uint32_t*>(reg_addr);
    
    return Platform::Status::OK;
}

// Write to a system register
Platform::Status SystemInitImpl::WriteSystemRegister(uint32_t reg_addr, uint32_t value) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Check if the register address is valid
    if (reg_addr % 4 != 0) {
        return Platform::Status::INVALID_PARAM; // Must be 4-byte aligned
    }
    
    // Write to the register address
    *reinterpret_cast<volatile uint32_t*>(reg_addr) = value;
    
    return Platform::Status::OK;
}

// Configure the system clock
Platform::Status SystemInitImpl::ConfigureSystemClock() {


    // Configure the system clock based on the provided configuration
    // Initialize RCC with the configuration
    if (!rcc_interface) {
        // Handle the error case - maybe try to initialize rcc_interface or return an error
        return Platform::Status::ERROR;
    }

    Platform::Status status = rcc_interface->Control(Platform::RCC::RCC_CTRL_CONFIGURE_SYSTEM_CLOCK, &config.SysClockConfig);
    if (status != Platform::Status::OK) {
        // Trigger error callback if registered
        if (callbacks[static_cast<uint32_t>(SystemEvent::ErrorDetected)].active) {
            callbacks[static_cast<uint32_t>(SystemEvent::ErrorDetected)].callback(
                callbacks[static_cast<uint32_t>(SystemEvent::ErrorDetected)].user_data
            );
        }
        return status;
    }
    
    // Get the actual system and bus clocks that were set
    uint32_t system_clock;
    status = rcc_interface->Control(Platform::RCC::RCC_CTRL_GET_CLOCK_FREQUENCY, &system_clock);
    
    if (status != Platform::Status::OK) {
        return status;
    }


    // Update system state with the actual clock frequencies

    Platform::RCC::SysCLockFreqs clock_freqs;
    status = rcc_interface->Control(Platform::RCC::RCC_CTRL_GET_ALL_CLOCK_FREQUENCIES, &clock_freqs);

    if (status != Platform::Status::OK) {
        return status;
    }

    system_state.system_clock_freq = clock_freqs.systemClock;
    system_state.ahb_clock_freq = clock_freqs.ahbClock;
    system_state.apb1_clock_freq = clock_freqs.apb1Clock;
    system_state.apb2_clock_freq = clock_freqs.apb2Clock;
    
    Platform::RCC::RccClockSource clock_source;
    // Update clock source and PLL status in system state
    status = rcc_interface->Control(Platform::RCC::RCC_CTRL_GET_CLOCK_SOURCE, &clock_source);
    if (status != Platform::Status::OK) {
        return status;
    }

    system_state.clock_source = clock_source;
    system_state.pll_enabled = config.SysClockConfig.pllConfig.autoCalculate;
    
    return Platform::Status::OK;
}
// Configure flash latency based on system clock frequency
Platform::Status SystemInitImpl::ConfigureFlashLatency(uint32_t system_clock_freq, Platform::FLASH::FlashLatency latency) {

    if (!flash_interface) {
        return Platform::Status::ERROR;
    }
    // Calculate appropriate latency based on system clock if auto mode is selected
    uint8_t flash_wait_states;
    
    if (latency == Platform::FLASH::FlashLatency::AUTOCALCULATE) {
        // For STM32F401 (3.3V operation):
        // 0 WS: 0 < HCLK ≤ 24 MHz
        // 1 WS: 24 MHz < HCLK ≤ 48 MHz
        // 2 WS: 48 MHz < HCLK ≤ 72 MHz
        // 3 WS: 72 MHz < HCLK ≤ 84 MHz
        if (system_clock_freq <= 24000000) {
            flash_wait_states = 0;
        } else if (system_clock_freq <= 48000000) {
            flash_wait_states = 1;
        } else if (system_clock_freq <= 72000000) {
            flash_wait_states = 2;
        } else {
            flash_wait_states = 3; // For up to 84 MHz
        }
    } else {
        // Use the provided latency value
        flash_wait_states = static_cast<uint8_t>(latency);
    }
    

    Platform::FLASH::FlashConfig flash_config = {
        .latency = static_cast<Platform::FLASH::FlashLatency>(flash_wait_states),
        .prefetch_enable = config.FlashConfig.prefetch_enable,
        .icache_enable = config.FlashConfig.icache_enable,
        .dcache_enable = config.FlashConfig.dcache_enable,
    };

    // Configure flash latency
    Platform::Status status = flash_interface->Control(Platform::FLASH::FLASH_CTRL_CONFIGURE, nullptr);

    if (status != Platform::Status::OK) {
        return status;
    }

    // Access flash registers to verify the latency setting
    volatile Platform::FLASH::Registers* flash_regs = Platform::FLASH::getRegisters();

    if(!flash_regs) {
        return Platform::Status::ERROR;
    }

    // Verify that the latency was set correctly
    if ((flash_regs->ACR & static_cast<uint32_t>(Platform::FLASH::ACR::LATENCY_MSK)) != flash_wait_states) {
        return Platform::Status::ERROR;
    }
    
    // Update system state
    system_state.flash_latency = flash_wait_states;
    system_state.prefetch_enabled = config.FlashConfig.prefetch_enable;
    system_state.icache_enabled = config.FlashConfig.icache_enable;
    system_state.dcache_enabled = config.FlashConfig.dcache_enable;
    
    return Platform::Status::OK;
}

// Configure SysTick timer
Platform::Status SystemInitImpl::ConfigureSysTick(uint32_t interval_us) {

    if (!systick_interface) {
        return Platform::Status::ERROR;
    }
    // Get SysTick interface
    systick_interface = &Platform::CMSIS::SysTick::SysTickInterface::GetInstance();
    
    // Calculate reload value based on system clock and desired interval
    uint32_t reload_value = (system_state.system_clock_freq / 1000000) * interval_us;
    
    // Check if reload value is within valid range (24-bit counter)
    if (reload_value > 0x00FFFFFF) {
        // If the value is too large, cap it at the maximum
        reload_value = 0x00FFFFFF;
        
        // Trigger error callback if registered
        if (callbacks[static_cast<uint32_t>(SystemEvent::ErrorDetected)].active) {
            callbacks[static_cast<uint32_t>(SystemEvent::ErrorDetected)].callback(
                callbacks[static_cast<uint32_t>(SystemEvent::ErrorDetected)].user_data
            );
        }
    }
    
    // Configure SysTick
    Platform::CMSIS::SysTick::SysTickConfig systick_config = {
        .reload_value = reload_value,
        .enable_interrupt = true,
        .use_processor_clock = true
    };
    
    // Initialize SysTick
    Platform::Status status = systick_interface->Init(&systick_config);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Start SysTick
    status = systick_interface->Control(Platform::CMSIS::SysTick::SYSTICK_CTRL_START, nullptr);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    return Platform::Status::OK;
}

// Configure MPU if enabled
Platform::Status SystemInitImpl::ConfigureMPU() {
    if (!config.enableMPU) {
        return Platform::Status::OK;
    }
    
    // Access MPU registers
    volatile uint32_t* mpu_type = reinterpret_cast<volatile uint32_t*>(0xE000ED90);
    volatile uint32_t* mpu_ctrl = reinterpret_cast<volatile uint32_t*>(0xE000ED94);
    
    // Check if MPU is present
    if ((*mpu_type & 0xFF) == 0) {
        // MPU not present in this device
        system_state.mpu_enabled = false;
        return Platform::Status::NOT_SUPPORTED;
    }
    
    // Disable MPU during configuration
    *mpu_ctrl = 0;
    
    // Configure default regions if needed
    // (This would be application-specific)
    
    // Enable MPU with default options
    *mpu_ctrl = (1 << 0);  // Enable MPU
    
    // Optional: Enable background region for privileged access
    // *mpu_ctrl = (1 << 0) | (1 << 2);
    
    // Update system state
    system_state.mpu_enabled = true;
    
    return Platform::Status::OK;
}

// Configure FPU if enabled
Platform::Status SystemInitImpl::ConfigureFPU() {
    if (!initialized || !config.enableFPU) {
        return Platform::Status::OK;
    }
    
    // We should ideally have an FPU interface, but if not available,
    // we can create a system register interface function that's more consistent
    
    auto fpu_regs = Platform::CMSIS::FPU::getRegisters(); // CPACR register address
    uint32_t cpacr_value = 0;
    
    // Read current value
    ReadSystemRegister(reinterpret_cast<uintptr_t>(&(fpu_regs->CPACR)), cpacr_value);
    
    // Enable CP10 and CP11 (FPU) access for privileged and user mode
    cpacr_value |= ((3UL << 10 * 2) | (3UL << 11 * 2));
    
    // Write updated value
    WriteSystemRegister(reinterpret_cast<uintptr_t>(&(fpu_regs->CPACR)), cpacr_value);
    
    // Make sure changes are applied
    __asm volatile("DSB");
    __asm volatile("ISB");
    
    // Update system state
    system_state.fpu_enabled = true;
    
    return Platform::Status::OK;
}

// Configure cache settings
Platform::Status SystemInitImpl::ConfigureCache() {
    // Set cache configuration based on settings

    if(!flash_interface) {
        return Platform::Status::ERROR;
    }
    
    Platform::FLASH::FlashConfig flash_config = {
        .latency = static_cast<Platform::FLASH::FlashLatency>(system_state.flash_latency),
        .prefetch_enable = config.FlashConfig.prefetch_enable,
        .icache_enable = config.FlashConfig.icache_enable,
        .dcache_enable = config.FlashConfig.dcache_enable,
    };

    // Configure flash latency
    Platform::Status status = flash_interface->Control(Platform::FLASH::FLASH_CTRL_CONFIGURE, &flash_config);

    if (status != Platform::Status::OK) {
        return status;
    }

    // Update system state
    system_state.flash_latency = static_cast<uint32_t>(flash_config.latency);
    system_state.prefetch_enabled = flash_config.prefetch_enable;
    system_state.icache_enabled = flash_config.icache_enable;
    system_state.dcache_enabled = flash_config.dcache_enable;
    
    return Platform::Status::OK;
}

// Determine the cause of the reset
// Determine the cause of the reset
ResetCause SystemInitImpl::DetermineResetCause() {
    // Get RCC interface
    if (!rcc_interface) {
        // Fallback to default if interface isn't available yet
        return ResetCause::PowerOn;
    }
    
    // Use RCC interface to read reset flags
    uint32_t reset_flags = 0;
    Platform::Status status = rcc_interface->Control(Platform::RCC::RCC_CTRL_GET_RESET_FLAGS, &reset_flags);
    
    if (status != Platform::Status::OK) {
        // Handle error in reading flags
        return ResetCause::Unknown;
    }
    
    // Check reset flags in order of priority
    if (reset_flags & Platform::RCC::RccResetFlags::RCC_RESET_FLAG_LPWRRST) {  // Low-power reset flag
        return ResetCause::LowPower;
    } else if (reset_flags &  Platform::RCC::RccResetFlags::RCC_RESET_FLAG_WWDGRST) {  // Window watchdog reset flag
        return ResetCause::Watchdog;
    } else if (reset_flags & Platform::RCC::RccResetFlags::RCC_RESET_FLAG_IWDGRST) {  // Independent watchdog reset flag
        return ResetCause::Watchdog;
    } else if (reset_flags & Platform::RCC::RccResetFlags::RCC_RESET_FLAG_SFTRST) {  // Software reset flag
        return ResetCause::Software;
    } else if (reset_flags & Platform::RCC::RccResetFlags::RCC_RESET_FLAG_PORRST) {  // POR/PDR reset flag
        return ResetCause::PowerOn;
    } else if (reset_flags & Platform::RCC::RccResetFlags::RCC_RESET_FLAG_PINRST) {  // PIN reset flag
        return ResetCause::External;
    } else if (reset_flags & Platform::RCC::RccResetFlags::RCC_RESET_FLAG_BORRST) {  // BOR reset flag
        return ResetCause::BrownOut;
    } else {
        return ResetCause::Unknown;
    }
}

// Clear reset flags
void SystemInitImpl::ClearResetFlags() {
    // Use RCC interface to clear reset flags
    if (rcc_interface) {
        rcc_interface->Control(Platform::RCC::RCC_CTRL_CLEAR_RESET_FLAGS, nullptr);
    }
}

} // namespace SystemServices
} // MiddleWare
