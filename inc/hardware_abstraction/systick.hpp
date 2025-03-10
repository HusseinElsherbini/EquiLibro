// inc/hardware_abstraction/systick.hpp

#ifndef SYSTICK_HPP
#define SYSTICK_HPP

#include "hw_interface.hpp"
#include "common/platform.hpp"
#include "common/platform_cmsis.hpp"

/**
 * Configuration structure for SysTick timer
 */
struct SysTickConfig {
    uint32_t reload_value;       // SysTick reload value (1 to 0x00FFFFFF)
    bool enable_interrupt;       // Enable SysTick interrupt
    bool use_processor_clock;    // true: use processor clock, false: use external clock (HCLK/8)
};

/**
 * SysTick event callback types
 */
enum class SysTickCallbackType {
    Tick,      // Called on each SysTick tick
    Max        // Number of callback types (keep this last)
};

/**
 * SysTick hardware interface implementation
 * 
 * Provides a C++ object-oriented interface to the ARM Cortex-M SysTick timer
 * peripheral. This interface follows the single-instance pattern since there is
 * only one SysTick timer in the Cortex-M architecture.
 */
class SysTickInterface : public HwInterface {
private:
    // Internal state tracking
    bool initialized;
    SysTickConfig config;
    uint64_t tick_count;
    uint32_t tick_frequency;
    
    // Callback table
    struct CallbackEntry {
        void (*callback)(void* param);
        void* param;
    };
    
    CallbackEntry callbacks[static_cast<uint32_t>(SysTickCallbackType::Max)];

    // Private constructor for singleton pattern
    SysTickInterface();
    
    // Deleted copy constructor and assignment operator
    SysTickInterface(const SysTickInterface&) = delete;
    SysTickInterface& operator=(const SysTickInterface&) = delete;
    
public:
    // Destructor
    ~SysTickInterface() override;
    
    // Interface implementation
    Platform::Status Init(void* config) override;
    Platform::Status DeInit() override;
    Platform::Status Control(uint32_t command, void* param) override;
    Platform::Status Read(void* buffer, uint16_t size, uint32_t timeout) override;
    Platform::Status Write(const void* data, uint16_t size, uint32_t timeout) override;
    Platform::Status RegisterCallback(uint32_t eventId, void (*callback)(void* param), void* param) override;
    
    // SysTick-specific methods
    uint32_t GetCurrentValue() const;
    uint64_t GetTickCount() const;
    uint32_t MillisecondsToTicks(uint32_t ms) const;
    uint64_t CalculateTimeout(uint32_t ms) const;
    bool HasTimeoutOccurred(uint64_t timeout) const;
    
    // Allow SysTick_IRQHandler to access private state
    friend void SysTick_IRQHandler(void);
    
    // Singleton pattern for SysTick interface (only need one instance)
    static SysTickInterface& GetInstance();
};

// SysTick control command identifiers
constexpr uint32_t SYSTICK_CTRL_START = 0x0801;     // Start SysTick counter
constexpr uint32_t SYSTICK_CTRL_STOP = 0x0802;      // Stop SysTick counter
constexpr uint32_t SYSTICK_CTRL_RELOAD = 0x0803;    // Reload SysTick counter
constexpr uint32_t SYSTICK_CTRL_GET_TICKFREQ = 0x0804; // Get tick frequency in Hz
constexpr uint32_t SYSTICK_CTRL_SET_CALLBACK = 0x0805; // Set callback for tick event

#endif /* SYSTICK_HPP */