// timer_interface.hpp
#pragma once

#include "hw_interface.hpp"
#include "common/platform.hpp"
#include "common/platform_tim.hpp"
#include <memory>

using namespace Platform::TIM;

namespace Platform {
    namespace TIM {
/**
 * Possible timer operations that can be performed
 * This replaces arbitrary uint32_t command codes with type-safe enumeration
 */
enum class TimerOperation {
    Start,
    Stop,
    ConfigureChannel,
    SetPeriod,
    SetPrescaler,
    SetCompareValue,
    EnableInterrupt,
    DisableInterrupt,
    ReadCounter,
    WriteCounter,
    EnableCapture,
    DisableCapture,
    GetStatus,
    SetMode,
    EnableChannel,
    DisableChannel,
};
constexpr uint32_t TIMER_CTRL_START = 0x0201;
constexpr uint32_t TIMER_CTRL_STOP = 0x0202;
constexpr uint32_t TIMER_CTRL_CONFIG_CHANNEL = 0x0203;
constexpr uint32_t TIMER_CTRL_SET_PERIOD = 0x0204;
constexpr uint32_t TIMER_CTRL_SET_PRESCALER = 0x0205;
constexpr uint32_t TIMER_CTRL_SET_COMPARE = 0x0206;
constexpr uint32_t TIMER_CTRL_ENABLE_INTERRUPT = 0x0207;
constexpr uint32_t TIMER_CTRL_DISABLE_INTERRUPT = 0x0208;
constexpr uint32_t TIMER_CTRL_READ_COUNTER = 0x0209;
constexpr uint32_t TIMER_CTRL_WRITE_COUNTER = 0x020A;
constexpr uint32_t TIMER_CTRL_ENABLE_CAPTURE = 0x020B;
constexpr uint32_t TIMER_CTRL_DISABLE_CAPTURE = 0x020C;
constexpr uint32_t TIMER_CTRL_GET_STATUS = 0x020D;
constexpr uint32_t TIMER_CTRL_GET_FEATURES = 0x020E;
constexpr uint32_t TIMER_CTRL_SET_MODE = 0x0210;
constexpr uint32_t TIMER_CTRL_ENABLE_CHANNEL = 0x0211;
constexpr uint32_t TIMER_CTRL_DISABLE_CHANNEL = 0x0212;
/**
 * Timer capabilities/features enumeration
 * Represents the various functional capabilities that a timer might support
 */
enum class TimerFeature {
    PWM,              // Pulse Width Modulation generation
    InputCapture,     // Input capture for measuring external signals
    OutputCompare,    // Output compare for generating precise timing events
    OnePulse,         // One-pulse mode for generating a single pulse
    EncoderInterface, // Encoder interface mode for rotary encoders
    BreakFunction,    // Break function for motor control
    DMA,              // Direct Memory Access support
    SlaveMode,        // External triggering and synchronization
    MasterMode,       // Trigger output to synchronize other timers
    ComplNChannels,   // Complementary outputs for channels
    DeadtimeGeneration, // Deadtime insertion between complementary outputs
    RepetitionCounter, // Repetition counter for update events
    Bit32Counter,     // 32-bit counter capability (only TIM2/TIM5)
    CenterAligned,    // Center-aligned counting mode
    CascadeCounting,  // Cascaded timer counting capability
    ExternalClock,    // External clock input capability
    Prescaler16bit,   // 16-bit programmable prescaler
    AsyncPrescaler,   // Asynchronous prescaler
    QadriEncoder      // Quadrature encoder inputs
};

// Timer compare value configuration
struct TimerCompareConfig {
    Channel channel;    // Timer channel
    uint32_t value;     // Compare value
};

/**
 * Timer counter access structure
 * Provides explicit structure for counter access rather than arbitrary read/write
 */
struct TimerCounterAccess {
    uint32_t value;      // Counter value to read or write
    bool reset_on_read;  // Whether to reset counter after reading
};

/**
 * Timer status information structure
 * Provides detailed status information rather than just a counter value
 */
struct TimerStatus {
    uint32_t counter_value;
    uint32_t prescaler_value;
    uint32_t period_value;
    bool is_running;
    uint32_t update_event_count;
    bool overflow_occurred;
    uint32_t flags;
    uint32_t capture_values[4];  // Capture values for up to 4 channels
};
struct TimerConfig {
    uint32_t desiredFrequency;
    TimerInstance timerInstance;  // Timer instance (1-5, 9-11)
    Mode mode;                    // Timer operating mode
    ClockDivision clockDivision;  // Clock division
    Direction direction;          // Count direction
    Alignment alignment;          // Alignment mode
    uint32_t prescaler;           // Timer prescaler
    uint32_t period;              // Timer period (auto-reload value)
    bool autoReloadPreload;       // Auto-reload preload enable
};
struct TimerChannelConfig {
    Channel channel;          // Timer channel
    OCMode ocMode;            // Output compare mode
    uint32_t pulse;           // Pulse value (capture/compare register)
    bool ocPreload;           // Output compare preload enable
    bool complementaryOutput; // Enable complementary output
};
/**
 * Enhanced Timer hardware interface implementation
 * Provides a comprehensive API for timer operations with explicit methods
 * for different functionality rather than generic read/write.
 */
class TimerInterface : public HwInterface {
private:
    // Internal state tracking
    bool initialized;
    
    // Timer instance this interface is controlling
    TimerInstance timer_instance;
    
    // Current configuration
    TimerConfig config;
    
    // Timer status tracking
    TimerStatus status;
    
    // Callback table for timer events
    struct CallbackEntry {
        void (*callback)(void* param);
        void* param;
        bool enabled;
    };
    
    CallbackEntry callbacks[static_cast<size_t>(TimerEvent::Count)];
    
    // Helper methods
    Platform::TIM::Registers* GetTimerRegister() const;
    Platform::Status ConfigureChannel(Platform::TIM::Registers* tim, const TimerChannelConfig& channelConfig);
    Platform::Status EnableClock();
    
    // Enhanced feature tracking
    bool is_running;
    TimerType timer_type;  // Advanced, GeneralPurpose, or Basic
    uint8_t available_channels;  // Number of available channels on this timer
    
    // Thread safety
    void LockAccess();  // For thread-safe operations
    void UnlockAccess();
    
public:
    // Constructor with timer type detection
    explicit TimerInterface(TimerInstance instance);
    
    // Destructor with proper cleanup
    ~TimerInterface() override;
    
    // Core HwInterface implementation
    Platform::Status Init(void* config) override;
    Platform::Status DeInit() override;
    
    // Enhanced control method with type-safe command enumeration
    Platform::Status Control(TimerOperation operation, void* param);
    
    // Implement required HwInterface method but mark as deprecated
    [[deprecated("Use Control(TimerOperation, void*) instead")]]
    Platform::Status Control(uint32_t command, void* param) override;
    
    // Replace generic Read/Write with explicit counter operations
    [[deprecated("Use GetCounterValue() or Control(TimerOperation::ReadCounter) instead")]]
    Platform::Status Read(void* buffer, uint16_t size, uint32_t timeout) override;
    
    [[deprecated("Use SetCounterValue() or Control(TimerOperation::WriteCounter) instead")]]
    Platform::Status Write(const void* data, uint16_t size, uint32_t timeout) override;
    
    // Enhanced callback registration with error details
    Platform::Status RegisterCallback(uint32_t eventId, void (*callback)(void* param), void* param) override;
    
    // Explicit timer operations with better error details
    Platform::Status Start();
    Platform::Status Stop();
    Platform::Status SetPeriod(uint32_t period);
    Platform::Status SetPrescaler(uint32_t prescaler);
    Platform::Status ConfigureChannel(const TimerChannelConfig& channelConfig);
    Platform::Status EnableInterrupt(TimerEvent event);
    Platform::Status DisableInterrupt(TimerEvent event);
    
    // Counter value access methods
    Platform::Status GetCounterValue(uint32_t& value);
    Platform::Status SetCounterValue(uint32_t value);
    
    // Enhanced status method
    Platform::Status GetStatus(TimerStatus& status);
    
    // Timer features information
    bool SupportsFeature(TimerFeature feature) const;
    uint8_t GetAvailableChannels() const;
    TimerType GetTimerType() const;
    
    // Safe singleton factory
    static TimerInterface& GetInstance(TimerInstance instance);
    
};

}
}