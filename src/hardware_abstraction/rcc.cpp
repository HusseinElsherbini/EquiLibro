// rcc.cpp
#include "hardware_abstraction/rcc.hpp"
#include "common/platform.hpp"
#include "common/platform_rcc.hpp"
#include "common/platform_flash.hpp"
#include <memory>
#include <mutex>
namespace Platform {
namespace RCC {

    // Static instance
static std::shared_ptr<RccInterface> rcc_instance = nullptr;
static std::mutex instance_mutex;
// Constructor
RccInterface::RccInterface() 
    : initialized(false), 
      clockFreqs(), 
      enabledPeripheralsAHB1(0),
      enabledPeripheralsAHB2(0),
      enabledPeripheralsAPB1(0),
      enabledPeripheralsAPB2(0) {
}

// Destructor
RccInterface::~RccInterface() {
    if (initialized) {
        DeInit();
    }
}

// Singleton instance getter
std::shared_ptr<RccInterface> RccInterface::GetInstance() {

    std::lock_guard<std::mutex> lock(instance_mutex);
    if (!rcc_instance) {
        rcc_instance = std::shared_ptr<RccInterface>(new RccInterface());
    }
    return rcc_instance;
}

// Create default RCC configuration for the MCU
// This gives a reliable starting point for clock configuration
RccConfig RccInterface::CreateDefaultConfig() {
    RccConfig config;
    
    // Default system configuration for STM32F401
    config.systemClockHz = 84000000;  // 84 MHz
    config.clockSource = RccClockSource::PLL;
    config.hseFrequencyHz = 25000000;  // 25 MHz external crystal
    
    // Default bus prescalers
    config.ahbPrescaler = 1;
    config.apb1Prescaler = 2;  // APB1 is limited to 42MHz max
    config.apb2Prescaler = 1;
    
    // Default PLL configuration
    config.pllConfig.autoCalculate = true;
    config.pllConfig.source = RccClockSource::HSE;
    
    // These values will be overwritten if autoCalculate is true
    config.pllConfig.m = 25;    // 25MHz / 25 = 1MHz PLL input
    config.pllConfig.n = 336;   // 1MHz * 336 = 336MHz VCO
    config.pllConfig.p = 4;     // 336MHz / 4 = 84MHz system clock
    config.pllConfig.q = 7;     // 336MHz / 7 = 48MHz (for USB)
    
    return config;
}

// Implementation of interface methods
Platform::Status RccInterface::Init(void* config) {
    // Check if already initialized
    if (initialized) {
        return Platform::Status::OK; // Already initialized
    }
    
    RccConfig* rccConfig = static_cast<RccConfig*>(config);
    
    // If no config provided, use default
    if (rccConfig == nullptr) {
        activeConfig = CreateDefaultConfig();
    } else {
        activeConfig = *rccConfig;
    }
    
    // Apply the configuration
    Platform::Status status = ConfigureSystemClock(activeConfig);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    initialized = true;
    return Platform::Status::OK;
}

Platform::Status RccInterface::ConfigureClockFrequency(uint32_t frequencyHz) {
    // Create a config based on the requested frequency
    RccConfig config = CreateDefaultConfig();
    config.systemClockHz = frequencyHz;
    
    // Apply the configuration
    return ConfigureSystemClock(config);
}

uint32_t RccInterface::GetSystemClockFrequency() const {
    return clockFreqs.systemClock;
}

uint32_t RccInterface::GetAhbClockFrequency() const {
    return clockFreqs.ahbClock;
}

uint32_t RccInterface::GetApb1ClockFrequency() const {
    return clockFreqs.apb1Clock;
}

uint32_t RccInterface::GetApb2ClockFrequency() const {
    return clockFreqs.apb2Clock;
}

uint32_t RccInterface::GetResetFlags() const {
    Platform::RCC::Registers* rcc = Platform::RCC::getRegisters();
    return rcc->CSR;
}

Platform::Status RccInterface::ConfigureSystemClock(const RccConfig& config) {
    Platform::RCC::Registers* rcc = Platform::RCC::getRegisters();
    
    // Make a local copy that we can modify
    RccConfig localConfig = config;
    
    // If PLL is used and auto-calculate is enabled, calculate PLL settings
    if (config.clockSource == RccClockSource::PLL && config.pllConfig.autoCalculate) {
        Platform::Status status = CalculatePllSettings(localConfig);
        if (status != Platform::Status::OK) {
            return status;
        }
    }
    
    // Configure flash latency based on target frequency
    Platform::Status status = ConfigureFlashLatency(localConfig.systemClockHz);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Step 1: Enable the necessary oscillator (HSE or HSI)
    if (localConfig.clockSource == RccClockSource::HSE || 
        (localConfig.clockSource == RccClockSource::PLL && 
         localConfig.pllConfig.source == RccClockSource::HSE)) {
        // Enable HSE
        Platform::setBit(rcc->CR, Platform::RCC::getBitValue(Platform::RCC::CR::HSEON));
        
        // Wait for HSE to stabilize
        uint32_t timeout = 50000;
        while (!Platform::isBitSet(rcc->CR, Platform::RCC::getBitValue(Platform::RCC::CR::HSERDY))) {
            if (--timeout == 0) {
                return Platform::Status::TIMEOUT;
            }
        }
    } else {
        // Ensure HSI is enabled (should be enabled by default after reset)
        Platform::setBit(rcc->CR, Platform::RCC::getBitValue(Platform::RCC::CR::HSION));
        
        // Wait for HSI to stabilize
        uint32_t timeout = 50000;
        while (!Platform::isBitSet(rcc->CR, Platform::RCC::getBitValue(Platform::RCC::CR::HSIRDY))) {
            if (--timeout == 0) {
                return Platform::Status::TIMEOUT;
            }
        }
    }
    
    // Step 2: Configure PLL if needed
    if (localConfig.clockSource == RccClockSource::PLL) {
        // Disable PLL before configuring
        Platform::clearBit(rcc->CR, Platform::RCC::getBitValue(Platform::RCC::CR::PLLON));
        
        // Wait for PLL to disable
        uint32_t timeout = 50000;
        while (Platform::isBitSet(rcc->CR, Platform::RCC::getBitValue(Platform::RCC::CR::PLLRDY))) {
            if (--timeout == 0) {
                return Platform::Status::TIMEOUT;
            }
        }
        
        // Configure PLL
        uint32_t pllcfgr = 0;
        
        // Set PLL source
        if (localConfig.pllConfig.source == RccClockSource::HSE) {
            pllcfgr |= Platform::RCC::getBitValue(Platform::RCC::PLLCFGR::PLLSRC_HSE);
        }
        
        // Set PLLM (division factor for the main PLL input clock)
        pllcfgr |= (localConfig.pllConfig.m & 0x3F); // 6 bits
        
        // Set PLLN (main PLL multiplication factor for VCO)
        pllcfgr |= ((localConfig.pllConfig.n & 0x1FF) << 6); // 9 bits
        
        // Set PLLP (main PLL division factor for main system clock)
        uint32_t p_value;
        switch (localConfig.pllConfig.p) {
            case 2: p_value = 0; break;
            case 4: p_value = 1; break;
            case 6: p_value = 2; break;
            case 8: p_value = 3; break;
            default: return Platform::Status::INVALID_PARAM;
        }
        pllcfgr |= (p_value << 16); // 2 bits
        
        // Set PLLQ (main PLL division factor for USB, SDIO, etc.)
        pllcfgr |= ((localConfig.pllConfig.q & 0xF) << 24); // 4 bits
        
        // Write PLL configuration
        rcc->PLLCFGR = pllcfgr;
        
        // Enable PLL
        Platform::setBit(rcc->CR, Platform::RCC::getBitValue(Platform::RCC::CR::PLLON));
        
        // Wait for PLL to stabilize
        timeout = 50000;
        while (!Platform::isBitSet(rcc->CR, Platform::RCC::getBitValue(Platform::RCC::CR::PLLRDY))) {
            if (--timeout == 0) {
                return Platform::Status::TIMEOUT;
            }
        }
    }
    
    // Step 3: Configure bus prescalers (AHB, APB1, APB2)
    uint32_t cfgr = 0;
    
    // Configure AHB prescaler
    switch (localConfig.ahbPrescaler) {
        case 1: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::HPRE_DIV1); break;
        case 2: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::HPRE_DIV2); break;
        case 4: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::HPRE_DIV4); break;
        case 8: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::HPRE_DIV8); break;
        case 16: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::HPRE_DIV16); break;
        case 64: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::HPRE_DIV64); break;
        case 128: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::HPRE_DIV128); break;
        case 256: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::HPRE_DIV256); break;
        case 512: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::HPRE_DIV512); break;
        default: return Platform::Status::INVALID_PARAM;
    }
    
    // Configure APB1 prescaler
    switch (localConfig.apb1Prescaler) {
        case 1: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::PPRE1_DIV1); break;
        case 2: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::PPRE1_DIV2); break;
        case 4: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::PPRE1_DIV4); break;
        case 8: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::PPRE1_DIV8); break;
        case 16: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::PPRE1_DIV16); break;
        default: return Platform::Status::INVALID_PARAM;
    }
    
    // Configure APB2 prescaler
    switch (localConfig.apb2Prescaler) {
        case 1: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::PPRE2_DIV1); break;
        case 2: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::PPRE2_DIV2); break;
        case 4: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::PPRE2_DIV4); break;
        case 8: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::PPRE2_DIV8); break;
        case 16: cfgr |= Platform::RCC::getBitValue(Platform::RCC::CFGR::PPRE2_DIV16); break;
        default: return Platform::Status::INVALID_PARAM;
    }
    
    // Step 4: Select the clock source and apply configuration
    // First update everything except the system clock source
    rcc->CFGR = cfgr;
    
    // Then update the clock source
    switch (localConfig.clockSource) {
        case RccClockSource::HSI:
            Platform::modifyReg(rcc->CFGR, 
                               Platform::RCC::getBitValue(Platform::RCC::CFGR::SW_MSK),
                               Platform::RCC::getBitValue(Platform::RCC::CFGR::SW_HSI));
            break;
        case RccClockSource::HSE:
            Platform::modifyReg(rcc->CFGR, 
                               Platform::RCC::getBitValue(Platform::RCC::CFGR::SW_MSK),
                               Platform::RCC::getBitValue(Platform::RCC::CFGR::SW_HSE));
            break;
        case RccClockSource::PLL:
            Platform::modifyReg(rcc->CFGR, 
                               Platform::RCC::getBitValue(Platform::RCC::CFGR::SW_MSK),
                               Platform::RCC::getBitValue(Platform::RCC::CFGR::SW_PLL));
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Wait for clock switch to complete
    uint32_t timeout = 50000;
    uint32_t status_mask = 0;
    
    switch (localConfig.clockSource) {
        case RccClockSource::HSI:
            status_mask = Platform::RCC::getBitValue(Platform::RCC::CFGR::SWS_HSI);
            break;
        case RccClockSource::HSE:
            status_mask = Platform::RCC::getBitValue(Platform::RCC::CFGR::SWS_HSE);
            break;
        case RccClockSource::PLL:
            status_mask = Platform::RCC::getBitValue(Platform::RCC::CFGR::SWS_PLL);
            break;
    }
    
    while ((rcc->CFGR & Platform::RCC::getBitValue(Platform::RCC::CFGR::SWS_MSK)) != status_mask) {
        if (--timeout == 0) {
            return Platform::Status::TIMEOUT;
        }
    }
    
    // Update cached clock frequencies
    if (localConfig.clockSource == RccClockSource::HSI) {
        clockFreqs.systemClock = 16000000; // HSI is fixed at 16 MHz
    } else if (localConfig.clockSource == RccClockSource::HSE) {
        clockFreqs.systemClock = localConfig.hseFrequencyHz;
    } else if (localConfig.clockSource == RccClockSource::PLL) {
        // Calculate actual system clock from PLL settings
        uint32_t vco_input;
        if (localConfig.pllConfig.source == RccClockSource::HSE) {
            vco_input = localConfig.hseFrequencyHz / localConfig.pllConfig.m;
        } else {
            vco_input = 16000000 / localConfig.pllConfig.m; // HSI is 16 MHz
        }
        
        uint32_t vco_output = vco_input * localConfig.pllConfig.n;
        clockFreqs.systemClock = vco_output / localConfig.pllConfig.p;
    }
    
    // Update bus frequencies
    clockFreqs.ahbClock = clockFreqs.systemClock / localConfig.ahbPrescaler;
    clockFreqs.apb1Clock = clockFreqs.ahbClock / localConfig.apb1Prescaler;
    clockFreqs.apb2Clock = clockFreqs.ahbClock / localConfig.apb2Prescaler;
    
    // Save the active configuration
    activeConfig = localConfig;
    
    return Platform::Status::OK;
}

Platform::Status RccInterface::CalculatePllSettings(RccConfig& config) {
    // Target frequency must be within valid range
    if (config.systemClockHz < 24000000 || config.systemClockHz > 84000000) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Determine PLL input clock source and frequency
    uint32_t inputFreq;
    if (config.pllConfig.source == RccClockSource::HSE) {
        inputFreq = config.hseFrequencyHz;
    } else {
        inputFreq = 16000000; // HSI is 16 MHz
    }
    
    // Best VCO input frequency is 2 MHz for better jitter performance
    // Calculate M divider to get close to 2 MHz
    config.pllConfig.m = inputFreq / 2000000;
    if (config.pllConfig.m < 2) {
        config.pllConfig.m = 2; // Minimum value
    } else if (config.pllConfig.m > 63) {
        config.pllConfig.m = 63; // Maximum value
    }
    
    // Calculate actual VCO input
    uint32_t vcoInput = inputFreq / config.pllConfig.m;
    if (vcoInput < 1000000 || vcoInput > 2000000) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Find a suitable P value (must be 2, 4, 6, or 8)
    // Try with smallest first for best performance
    config.pllConfig.p = 2;
    
    // Calculate required VCO output to achieve target system clock
    uint32_t vcoOutput = config.systemClockHz * config.pllConfig.p;
    
    // Check if VCO output is within valid range (100-432 MHz)
    if (vcoOutput > 432000000) {
        // Try higher P values
        config.pllConfig.p = 4;
        vcoOutput = config.systemClockHz * config.pllConfig.p;
        
        if (vcoOutput > 432000000) {
            config.pllConfig.p = 6;
            vcoOutput = config.systemClockHz * config.pllConfig.p;
            
            if (vcoOutput > 432000000) {
                config.pllConfig.p = 8;
                vcoOutput = config.systemClockHz * config.pllConfig.p;
                
                if (vcoOutput > 432000000) {
                    return Platform::Status::INVALID_PARAM; // Cannot achieve target frequency
                }
            }
        }
    } else if (vcoOutput < 100000000) {
        return Platform::Status::INVALID_PARAM; // VCO output too low
    }
    
    // Calculate PLLN to achieve the desired VCO output
    config.pllConfig.n = vcoOutput / vcoInput;
    
    // Check if PLLN is within valid range (50-432)
    if (config.pllConfig.n < 50 || config.pllConfig.n > 432) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Calculate USB frequency Q divider (48MHz output for USB)
    if (vcoOutput % 48000000 == 0) {
        config.pllConfig.q = vcoOutput / 48000000;
        if (config.pllConfig.q < 2 || config.pllConfig.q > 15) {
            // If can't get exact 48MHz, approximate
            config.pllConfig.q = vcoOutput / 48000000;
            if (config.pllConfig.q < 2) config.pllConfig.q = 2;
            if (config.pllConfig.q > 15) config.pllConfig.q = 15;
        }
    } else {
        // Approximate division for USB
        config.pllConfig.q = vcoOutput / 48000000;
        if (config.pllConfig.q < 2) config.pllConfig.q = 2;
        if (config.pllConfig.q > 15) config.pllConfig.q = 15;
    }
    
    return Platform::Status::OK;
}
// Implement the ConfigureFlashLatency method
Platform::Status RccInterface::ConfigureFlashLatency(uint32_t systemClockHz) {

    using namespace Platform:: FLASH;

    // Determine proper flash latency based on system clock frequency
    uint8_t flashLatency;
    
    // For STM32F401 (3.3V operation):
    // 0 WS: 0 < HCLK ≤ 24 MHz
    // 1 WS: 24 MHz < HCLK ≤ 48 MHz
    // 2 WS: 48 MHz < HCLK ≤ 72 MHz
    // 3 WS: 72 MHz < HCLK ≤ 84 MHz
    if (systemClockHz <= 24000000) {
        flashLatency = 0;
    } else if (systemClockHz <= 48000000) {
        flashLatency = 1;
    } else if (systemClockHz <= 72000000) {
        flashLatency = 2;
    } else if (systemClockHz <= 84000000) {
        flashLatency = 3;
    } else {
        return Platform::Status::INVALID_PARAM; // Clock frequency too high
    }
    
    // Get a pointer to the Flash Access Control Register
    volatile uint32_t* flashAcr = reinterpret_cast<volatile uint32_t*>(Platform::FLASH::FLASH_ACR);
    
    // Apply new flash latency value while preserving other bits
    uint32_t regValue = *flashAcr;
    regValue &= ~0xF; // Clear latency bits (first 4 bits)
    regValue |= flashLatency; // Set new latency
    
    // Enable prefetch, instruction cache, and data cache for better performance
    regValue |= static_cast<uint32_t>(ACR::PRFTEN | ACR::DCEN | ACR::DCEN); // PRFTEN, ICEN, DCEN
    
    // Write new configuration
    *flashAcr = regValue;

    // Verify that the new latency setting was applied
    if ((*flashAcr & 0xF) != flashLatency) {
        return Platform::Status::ERROR;
    }
    
    return Platform::Status::OK;
}

Platform::Status RccInterface::ClearResetFlags(void) {
    Platform::RCC::Registers* rcc = Platform::RCC::getRegisters();
    
    // Clear all flags
    rcc->CSR = 0x0F; // Clear all reset flags
    
    return Platform::Status::OK;
}

Platform::Status RccInterface::DeInit() {
    // Reset all clock configuration back to reset state
    // Careful: this can cause system to stop running if not done correctly
    
    // Disable all peripheral clocks
    Platform::RCC::Registers* rcc = Platform::RCC::getRegisters();
    
    // Disable all AHB1 peripheral clocks
    rcc->AHB1ENR = 0;
    enabledPeripheralsAHB1 = 0;
    
    // Disable all AHB2 peripheral clocks
    rcc->AHB2ENR = 0;
    enabledPeripheralsAHB2 = 0;
    
    // Disable all APB1 peripheral clocks
    rcc->APB1ENR = 0;
    enabledPeripheralsAPB1 = 0;
    
    // Disable all APB2 peripheral clocks
    rcc->APB2ENR = 0;
    enabledPeripheralsAPB2 = 0;
    
    // Reset clock configuration
    // Note: Don't disable system clock completely, switch to HSI
    
    // Disable PLL
    Platform::clearBit(rcc->CR, Platform::RCC::getBitValue(Platform::RCC::CR::PLLON));
    
    // Wait for PLL to stop
    while (Platform::readBit(rcc->CR, Platform::RCC::getBitValue(Platform::RCC::CR::PLLRDY))) {
        // Wait loop
    }
    
    // Switch to HSI as system clock
    Platform::modifyReg(rcc->CFGR, 
                      Platform::RCC::getBitValue(Platform::RCC::CFGR::SW_MSK),
                      Platform::RCC::getBitValue(Platform::RCC::CFGR::SW_HSI));
    
    // Wait for clock switch to complete
    while ((rcc->CFGR & Platform::RCC::getBitValue(Platform::RCC::CFGR::SWS_MSK)) != 
           Platform::RCC::getBitValue(Platform::RCC::CFGR::SWS_HSI)) {
        // Wait loop
    }
    
    // Reset bus prescalers
    Platform::modifyReg(rcc->CFGR,
                      Platform::RCC::getBitValue(Platform::RCC::CFGR::HPRE_MSK) |
                      Platform::RCC::getBitValue(Platform::RCC::CFGR::PPRE1_MSK) |
                      Platform::RCC::getBitValue(Platform::RCC::CFGR::PPRE2_MSK),
                      Platform::RCC::getBitValue(Platform::RCC::CFGR::HPRE_DIV1) |
                      Platform::RCC::getBitValue(Platform::RCC::CFGR::PPRE1_DIV1) |
                      Platform::RCC::getBitValue(Platform::RCC::CFGR::PPRE2_DIV1));
    
    // Disable HSE if it was on
    Platform::clearBit(rcc->CR, Platform::RCC::getBitValue(Platform::RCC::CR::HSEON));
    
    // Update cached frequencies
    clockFreqs.systemClock = 16000000;  // HSI frequency
    clockFreqs.ahbClock = 16000000;
    clockFreqs.apb1Clock = 16000000;
    clockFreqs.apb2Clock = 16000000;
    
    initialized = false;
    return Platform::Status::OK;
}

Platform::Status RccInterface::Control(uint32_t command, void* param) {
    if (!initialized && command != RCC_CTRL_SET_CLOCK_FREQUENCY) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (param == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    using namespace Platform::RCC;

    switch (command) {
        case RCC_CTRL_ENABLE_PERIPHERAL: {
            RccPeripheral peripheral = *static_cast<RccPeripheral*>(param);
            return EnablePeripheralClock(peripheral);
        }
        
        case RCC_CTRL_DISABLE_PERIPHERAL: {
            RccPeripheral peripheral = *static_cast<RccPeripheral*>(param);
            return DisablePeripheralClock(peripheral);
        }
        
        case RCC_CTRL_GET_CLOCK_FREQUENCY: {
            struct {
                RccPeripheral peripheral;
                uint32_t frequency;
            }* freq_info = static_cast<decltype(freq_info)>(param);
            
            // Determine which bus the peripheral is on and return appropriate frequency

            auto it = peripheralMap.find(freq_info->peripheral);
            if (it == peripheralMap.end()) {
                return Platform::Status::INVALID_PARAM;
            }
            RccBusType bus = it->second.bus;
            uint8_t bitPos = it->second.bitPosition;
            
            switch (bus) {
                case RccBusType::AHB1 :  // AHB1
                    freq_info->frequency = clockFreqs.ahbClock;
                    break;
                case RccBusType::AHB2:  // AHB2
                    freq_info->frequency = clockFreqs.ahbClock;
                    break;
                case RccBusType::APB1:  // APB1
                    freq_info->frequency = clockFreqs.apb1Clock;
                    break;
                case RccBusType::APB2:  // APB2
                    freq_info->frequency = clockFreqs.apb2Clock;
                    break;
                default:
                    return Platform::Status::INVALID_PARAM;
            }
            return Platform::Status::OK;
        }
        
        case RCC_CTRL_SET_CLOCK_FREQUENCY: {
            uint32_t frequency = *static_cast<uint32_t*>(param);
            return ConfigureClockFrequency(frequency);
        }
        case RCC_CTRL_CLEAR_RESET_FLAGS: {
            return ClearResetFlags();
        }
        case RCC_CTRL_CONFIGURE_SYSTEM_CLOCK: {
            RccConfig* config = static_cast<RccConfig*>(param);
            return ConfigureSystemClock(*config);
        }
        case RCC_CTRL_GET_RESET_FLAGS: {
            uint32_t* flags = static_cast<uint32_t*>(param);
            *flags = GetResetFlags();
            return Platform::Status::OK;
        }
        case RCC_CTRL_GET_ALL_CLOCK_FREQUENCIES: {
            SysCLockFreqs* freqs = static_cast<SysCLockFreqs*>(param);
            freqs->systemClock = GetSystemClockFrequency();
            freqs->ahbClock = GetAhbClockFrequency();
            freqs->apb1Clock = GetApb1ClockFrequency();
            freqs->apb2Clock = GetApb2ClockFrequency();
            return Platform::Status::OK;
        }
        case RCC_CTRL_GET_CLOCK_SOURCE: {
            RccClockSource* source = static_cast<RccClockSource*>(param);
            *source = activeConfig.clockSource;
            return Platform::Status::OK;
        }
        case RCC_CTRL_ENTER_LOW_POWER:
        case RCC_CTRL_EXIT_LOW_POWER:
            // Not implemented for this example, but could be added
            return Platform::Status::NOT_SUPPORTED;
            
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
}

Platform::Status RccInterface::Read(void* buffer, uint16_t size, uint32_t timeout) {
    // Direct Read operation not typically used for RCC
    return Platform::Status::NOT_SUPPORTED;
}

Platform::Status RccInterface::Write(const void* data, uint16_t size, uint32_t timeout) {
    // Direct Write operation not typically used for RCC
    return Platform::Status::NOT_SUPPORTED;
}

Platform::Status RccInterface::RegisterCallback(uint32_t eventId, void (*callback)(void* param), void* param) {
    // Callbacks not typically used for RCC
    return Platform::Status::NOT_SUPPORTED;
}

// RCC-specific methods

Platform::Status RccInterface::EnablePeripheralClock(Platform::RCC::RccPeripheral peripheral) {

    using namespace Platform::RCC;

    Registers* rcc = getRegisters();

    // get bus and bit position from map 

    auto it = peripheralMap.find(peripheral);
    if (it == peripheralMap.end()) {
        return Platform::Status::INVALID_PARAM;
    }
    RccBusType bus = it->second.bus;
    uint8_t bitPos = it->second.bitPosition;
    
    // Enable the peripheral clock and update the enabled peripherals bitmask
    switch (bus) {
        case RccBusType::AHB1:

            Platform::setBit(rcc->AHB1ENR, (1UL << bitPos));
            enabledPeripheralsAHB1 |= (1UL << bitPos);
            break;
            
        case RccBusType::AHB2:

            Platform::setBit(rcc->AHB2ENR, (1UL << bitPos));
            enabledPeripheralsAHB2 |= (1UL << bitPos);
            break;

        case RccBusType::APB1:

            Platform::setBit(rcc->APB1ENR, (1UL << bitPos));
            enabledPeripheralsAPB1 |= (1UL << bitPos);
            break;
        
        case RccBusType::APB2:

            Platform::setBit(rcc->APB2ENR, (1UL << bitPos));
            enabledPeripheralsAPB2 |= (1UL << bitPos);
            break;
        
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    return Platform::Status::OK;
}

Platform::Status RccInterface::IsPeripheralClockEnabled(
    Platform::RCC::RccPeripheral peripheral, 
    bool& enabled) const 
{
    using namespace Platform::RCC;
    
    // Initialize output parameter to a safe value
    enabled = false;
    
    // Find the peripheral in the map
    auto it = peripheralMap.find(peripheral);
    if (it == peripheralMap.end()) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Extract bus type and bit position
    RccBusType bus = it->second.bus;
    uint8_t bitPos = it->second.bitPosition;
    
    // Create bit mask for this peripheral
    uint32_t bitMask = (1UL << bitPos);
    
    // Check the appropriate tracking variable
    switch (bus) {
        case RccBusType::AHB1:
            enabled = (enabledPeripheralsAHB1 & bitMask) != 0;
            break;
        case RccBusType::AHB2:
            enabled = (enabledPeripheralsAHB2 & bitMask) != 0;
            break;
        case RccBusType::APB1:
            enabled = (enabledPeripheralsAPB1 & bitMask) != 0;
            break;
        case RccBusType::APB2:
            enabled = (enabledPeripheralsAPB2 & bitMask) != 0;
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    return Platform::Status::OK;
}

Platform::Status RccInterface::DisablePeripheralClock(Platform::RCC::RccPeripheral peripheral) {

    using namespace Platform::RCC;

    Platform::RCC::Registers* rcc = Platform::RCC::getRegisters();
    
    // Determine which bus this peripheral is on
    // get bus and bit position from map 
    auto it = peripheralMap.find(peripheral);
    if (it == peripheralMap.end()) {
        return Platform::Status::INVALID_PARAM;
    }
    RccBusType bus = it->second.bus;
    uint8_t bitPos = it->second.bitPosition;
    
    uint32_t periph_val = static_cast<uint32_t>(peripheral);
    uint8_t bus = (periph_val >> 8) & 0xFF;
    uint8_t periph_bit = periph_val & 0xFF;
    
    // Disable the appropriate clock
    switch (bus) {
        case RccBusType::AHB1:  // AHB1

            Platform::clearBit(rcc->AHB1ENR, (1UL << bitPos));
            enabledPeripheralsAHB1 &= ~(1 << static_cast<uint8_t>(periph_bit - 1));
            break;
        
        case RccBusType::AHB2:  // AHB2

            Platform::clearBit(rcc->AHB2ENR, (1UL << bitPos));
            enabledPeripheralsAHB2 &= ~(1 << static_cast<uint8_t>(periph_bit - 1));
            break;

        case RccBusType::APB1:  // APB1

            Platform::clearBit(rcc->APB1ENR, (1UL << bitPos));
            enabledPeripheralsAPB1 &= ~(1 << static_cast<uint8_t>(periph_bit - 1));
            break;

        case RccBusType::APB2:  // APB2

            Platform::clearBit(rcc->APB2ENR, (1UL << bitPos));
            enabledPeripheralsAPB2 &= ~(1 << static_cast<uint8_t>(periph_bit - 1));
            break;

        default:

            return Platform::Status::INVALID_PARAM;
            
        }
    
    return Platform::Status::OK;
}
}