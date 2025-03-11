// src/hardware_abstraction/systick.cpp

#include "hardware_abstraction/systick.hpp"
#include <memory>
#include <mutex>
#include "common/platform.hpp"
namespace Platform {
namespace CMSIS {
namespace SysTick {

// Static instance for singleton pattern
std::shared_ptr<SysTickInterface> systick_instance = nullptr;
std::mutex instance_mutex;


// Constructor
SysTickInterface::SysTickInterface() 
    : initialized(false), 
      tick_count(0),
      tick_frequency(0) {
    
    // Initialize callbacks to nullptr
    for (auto& cb : callbacks) {
        cb.callback = nullptr;
        cb.param = nullptr;
    }
}

// Destructor
SysTickInterface::~SysTickInterface() {
    if (initialized) {
        // Ensure SysTick is disabled before destruction
        DeInit();
    }
}

// Singleton pattern implementation
std::shared_ptr<SysTickInterface> SysTickInterface::GetInstance() {

    std::lock_guard<std::mutex> lock(instance_mutex);
    if (!systick_instance) {
        systick_instance = std::shared_ptr<SysTickInterface>(new SysTickInterface());
    }
    
    return systick_instance;
}

// Initialize SysTick with specific configuration
Status SysTickInterface::Init(void* config) {
    // Check if already initialized
    if (initialized) {
        return Platform::Status::OK; // Already initialized
    }
    
    // Validate input parameter
    if (config == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    SysTickConfig* systick_config = static_cast<SysTickConfig*>(config);
    
    // Validate reload value (must be between 1 and 0x00FFFFFF)
    if (systick_config->reload_value < 1 || systick_config->reload_value > 0x00FFFFFF) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Get pointer to SysTick registers
    auto systick = Platform::CMSIS::SysTick::getRegisters();
    
    // Stop SysTick if it's running
    systick->CTRL = 0;
    
    // Clear the current value
    systick->VAL = 0;
    
    // Set the reload value
    systick->LOAD = systick_config->reload_value;
    
    // Calculate tick frequency
    uint32_t clock;
    if (systick_config->use_processor_clock) {
        clock = Platform::MCU_CLK;  // Processor clock
    } else {
        clock = Platform::MCU_CLK / 8;  // Processor clock / 8
    }
    
    tick_frequency = clock / systick_config->reload_value;
    
    // Set control register based on configuration
    uint32_t ctrl = 0;
    if (systick_config->enable_interrupt) {
        ctrl |= Platform::CMSIS::SysTick::CTRL_TICKINT;
    }
    if (systick_config->use_processor_clock) {
        ctrl |= Platform::CMSIS::SysTick::CTRL_CLKSOURCE;
    }
    
    // Save configuration
    this->config = *systick_config;
    
    // Don't start the counter yet - will be started with SYSTICK_CTRL_START command
    
    // Mark as initialized
    initialized = true;
    
    return Platform::Status::OK;
}

// De-initialize SysTick
Platform::Status SysTickInterface::DeInit() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Get pointer to SysTick registers
    auto systick = Platform::CMSIS::SysTick::getRegisters();
    
    // Stop SysTick
    systick->CTRL = 0;
    
    // Reset state
    initialized = false;
    tick_count = 0;
    tick_frequency = 0;
    
    return Platform::Status::OK;
}

// SysTick control function
Platform::Status SysTickInterface::Control(uint32_t command, void* param) {
    if (!initialized && command != SYSTICK_CTRL_START) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    auto systick = Platform::CMSIS::SysTick::getRegisters();
    
    switch (command) {
        case SYSTICK_CTRL_START: {
            // Start the SysTick counter
            uint32_t ctrl = systick->CTRL;
            if (config.enable_interrupt) {
                ctrl |= Platform::CMSIS::SysTick::CTRL_TICKINT;
            }
            if (config.use_processor_clock) {
                ctrl |= Platform::CMSIS::SysTick::CTRL_CLKSOURCE;
            }
            ctrl |= Platform::CMSIS::SysTick::CTRL_ENABLE;
            systick->CTRL = ctrl;
            return Platform::Status::OK;
        }
            
        case SYSTICK_CTRL_STOP: {
            // Stop the SysTick counter
            uint32_t ctrl = systick->CTRL;
            ctrl &= ~Platform::CMSIS::SysTick::CTRL_ENABLE;
            systick->CTRL = ctrl;
            return Platform::Status::OK;
        }
            
        case SYSTICK_CTRL_RELOAD: {
            // Reload the SysTick counter
            systick->VAL = 0;
            return Platform::Status::OK;
        }
            
        case SYSTICK_CTRL_GET_TICKFREQ: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            *static_cast<uint32_t*>(param) = tick_frequency;
            return Platform::Status::OK;
        }
            
        case SYSTICK_CTRL_SET_CALLBACK: {
            // This is redundant since we have RegisterCallback, but included for completeness
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            // Cast param to callback structure
            struct SysTickCallbackConfig {
                SysTickCallbackType type;
                void (*callback)(void *param);
                void *callback_param;
            };
            
            auto callback_config = static_cast<SysTickCallbackConfig*>(param);
            
            if (static_cast<uint32_t>(callback_config->type) >= static_cast<uint32_t>(SysTickCallbackType::Max)) {
                return Platform::Status::INVALID_PARAM;
            }
            
            callbacks[static_cast<uint32_t>(callback_config->type)].callback = callback_config->callback;
            callbacks[static_cast<uint32_t>(callback_config->type)].param = callback_config->callback_param;
            
            return Platform::Status::OK;
        }
            
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
}

// Read SysTick counter or tick count
Platform::Status SysTickInterface::Read(void* buffer, uint16_t size, uint32_t timeout) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (buffer == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    if (size >= sizeof(uint64_t)) {
        // Read current tick count
        *static_cast<uint64_t*>(buffer) = tick_count;
        return Platform::Status::OK;
    } else if (size >= sizeof(uint32_t)) {
        // Read current counter value
        *static_cast<uint32_t*>(buffer) = GetCurrentValue();
        return Platform::Status::OK;
    }
    
    return Platform::Status::INVALID_PARAM;
}

// Write to SysTick counter (not generally used)
Platform::Status SysTickInterface::Write(const void* data, uint16_t size, uint32_t timeout) {
    // SysTick counter is typically read-only
    return Platform::Status::NOT_SUPPORTED;
}

// Register callback for SysTick events
Platform::Status SysTickInterface::RegisterCallback(uint32_t eventId, void (*callback)(void* param), void* param) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (eventId >= static_cast<uint32_t>(SysTickCallbackType::Max)) {
        return Platform::Status::INVALID_PARAM;
    }
    
    callbacks[eventId].callback = callback;
    callbacks[eventId].param = param;
    
    return Platform::Status::OK;
}

// Get the current SysTick counter value
uint32_t SysTickInterface::GetCurrentValue() const {
    auto systick = Platform::CMSIS::SysTick::getRegisters();
    return systick->VAL;
}

// Get the number of SysTick ticks that have occurred since initialization
uint64_t SysTickInterface::GetTickCount() const {
    return tick_count;
}

// Convert milliseconds to SysTick ticks
uint32_t SysTickInterface::MillisecondsToTicks(uint32_t ms) const {
    return (ms * tick_frequency) / 1000;
}

// Calculate a timeout point in the future
uint64_t SysTickInterface::CalculateTimeout(uint32_t ms) const {
    uint32_t ticks = MillisecondsToTicks(ms);
    return tick_count + ticks;
}

// Check if a timeout has occurred
bool SysTickInterface::HasTimeoutOccurred(uint64_t timeout) const {
    return tick_count >= timeout;
}

// SysTick interrupt handler
// This should be called from the SysTick_Handler() in startup code
extern "C" void SysTick_IRQHandler(void) {
    // Increment tick count
    systick_instance.tick_count++;
    
    // Call tick callback if registered
    auto& callback = systick_instance.callbacks[static_cast<uint32_t>(SysTickCallbackType::Tick)];
    if (callback.callback != nullptr) {
        callback.callback(callback.param);
    }
}
} // namespace SysTick
} // namespace CMSIS

