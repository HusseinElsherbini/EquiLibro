#pragma once

#include "hw_interface.hpp"
#include "middleware/os/mutex.hpp"
#include "hardware_abstraction/timer.hpp"
#include "hardware_abstraction/gpio.hpp"
#include <memory>
#include <array>
#include <vector>
#include <mutex>

namespace Platform {
namespace PWM {

/**
 * PWM Channel Enumeration
 * Maps directly to the timer channels
 */
enum class PWMChannel : uint8_t {
    Channel1 = 0,
    Channel2 = 1,
    Channel3 = 2,
    Channel4 = 3
};

/**
 * PWM Polarity Configuration
 */
enum class PWMPolarity : uint8_t {
    ActiveHigh = 0, // Output active when counter < compare value (default)
    ActiveLow = 1   // Output active when counter > compare value
};

/**
 * PWM Channel Events
 */
enum class PWMEvent : uint32_t {
    PulseFinished = 0, // PWM pulse period completed
    Update = 1,        // Timer update event
    Error = 2,         // Error occurred
    Max = 3            // Used for array sizing
};

/**
 * PWM Alignment Mode
 */
enum class PWMAlignment : uint8_t {
    EdgeAligned = 0,    // Traditional edge-aligned PWM
    CenterAligned = 1   // Center-aligned PWM (symmetrical)
};

/**
 * PWM Operating Mode
 */
enum class PWMMode : uint8_t {
    Standard = 0,       // Standard PWM mode
    OnePulse = 1,       // One-pulse mode (stops after one cycle)
    Complementary = 2   // Complementary outputs (only available on certain timers)
};

/**
 * PWM Configuration Structure
 */
struct PWMConfig {
    TIM::TimerInstance timer_instance; // Timer to use for this PWM instance
    uint32_t frequency;                // PWM frequency in Hz
    uint8_t resolution_bits;           // Resolution in bits (determines period value)
    PWMAlignment alignment;            // PWM alignment mode
    PWMMode mode;                      // PWM operating mode
    bool use_dead_time;                // Use dead time insertion between complementary outputs
    uint16_t dead_time_ns;             // Dead time in nanoseconds (for complementary mode)
};

/**
 * PWM Channel Configuration
 */
struct PWMChannelConfig {
    PWMChannel channel;                // Channel number
    uint32_t duty_cycle;               // Duty cycle in 0.01% (0-10000, where 10000 = 100.00%)
    PWMPolarity polarity;              // Channel output polarity
    bool complementary_output;         // Whether to use complementary output (if available)
    GPIO::Port gpio_port;              // GPIO port for the PWM output pin
    uint8_t gpio_pin;                  // GPIO pin number for the PWM output
    GPIO::AlternateFunction gpio_af;   // Alternate function for GPIO pin
};

// PWM Control Commands - map to common HW interface standard
constexpr uint32_t PWM_CTRL_START              = HW_CTRL_ENABLE;                 // Start PWM generation
constexpr uint32_t PWM_CTRL_STOP               = HW_CTRL_DISABLE;                // Stop PWM generation
constexpr uint32_t PWM_CTRL_SET_FREQUENCY      = 0x0101;                         // Set PWM frequency
constexpr uint32_t PWM_CTRL_SET_DUTY_CYCLE     = 0x0102;                         // Set duty cycle for a channel
constexpr uint32_t PWM_CTRL_SET_CHANNEL_CONFIG = 0x0103;                         // Configure a PWM channel
constexpr uint32_t PWM_CTRL_ENABLE_CHANNEL     = 0x0104;                         // Enable a PWM channel
constexpr uint32_t PWM_CTRL_DISABLE_CHANNEL    = 0x0105;                         // Disable a PWM channel
constexpr uint32_t PWM_CTRL_SET_POLARITY       = 0x0106;                         // Set output polarity
constexpr uint32_t PWM_CTRL_ENABLE_DEADTIME    = 0x0107;                         // Enable dead time
constexpr uint32_t PWM_CTRL_DISABLE_DEADTIME   = 0x0108;                         // Disable dead time
constexpr uint32_t PWM_CTRL_SET_MODE           = 0x0109;                         // Set PWM mode
constexpr uint32_t PWM_CTRL_SET_ALL_DUTY_CYCLE = 0x010A;                         // Set duty cycle for all active channels

/**
 * PWM Interface Implementation
 * Provides a hardware abstraction for PWM control using the underlying timer hardware
 */
class PWMInterface : public HwInterface {
private:
    // Stores whether the PWM interface is initialized
    bool initialized;
    
    // Keeps track of which channels are active
    bool channel_active[4];
    
    // The underlying timer instance used for PWM generation
    Platform::TIM::TimerInterface * timer_interface;
    
    // GPIO interface for configuring PWM output pins
    Platform::GPIO::GpioInterface * gpio_interface;
    
    // Current PWM configuration
    PWMConfig config;
    
    // Channel configurations
    std::array<PWMChannelConfig, 4> channel_configs;
    
    // Timer counter period value (calculated from frequency and resolution)
    uint32_t period_value;
    
    // Prescaler value for the timer
    uint32_t prescaler_value;
    
    // Callback storage
    struct CallbackEntry {
        void (*callback)(void* param);
        void* param;
        bool enabled;
    };
    
    std::array<CallbackEntry, static_cast<size_t>(PWMEvent::Max)> callbacks;
    
    // Mutex for thread safety
    static OS::mutex instances_mutex;
    
    // Helper methods
    
    /**
     * Calculate timer parameters (period and prescaler) based on desired frequency
     * 
     * @param frequency Desired PWM frequency in Hz
     * @return Status indicating success or failure
     */
    Platform::Status CalculateTimerParameters(uint32_t frequency);
    
    /**
     * Configure a GPIO pin for PWM output
     * 
     * @param config Channel configuration containing GPIO information
     * @return Status indicating success or failure
     */
    Platform::Status ConfigurePWMPin(const PWMChannelConfig& config);
    
    /**
     * Configure a timer channel for PWM operation
     * 
     * @param config Channel configuration
     * @return Status indicating success or failure
     */
    Platform::Status ConfigureTimerChannel(const PWMChannelConfig& config);
    
    /**
     * Convert a duty cycle percentage to a compare value
     * 
     * @param duty_cycle_percent Duty cycle in 0.01% (0-10000)
     * @return Compare value for the timer
     */
    uint32_t DutyCycleToCompareValue(uint32_t duty_cycle_percent) const;
    
    /**
     * Convert a compare value to a duty cycle percentage
     * 
     * @param compare_value Timer compare value
     * @return Duty cycle in 0.01% (0-10000)
     */
    uint32_t CompareValueToDutyCycle(uint32_t compare_value) const;
    
    /**
     * Static callback handler for timer events
     * 
     * @param param Pointer to PWM interface instance
     */
    static void TimerEventCallback(void* param);
    
    // Constructor for singleton pattern
    explicit PWMInterface();
    PWMInterface(const PWMInterface&) = delete;
    PWMInterface& operator=(const PWMInterface&) = delete;
    
public:
    /**
     * Get the singleton instance of the PWM interface
     * 
     * @return Reference to the PWM interface instance
     */
    static PWMInterface& GetInstance();
    
    /**
     * Destructor ensures proper cleanup
     */
    ~PWMInterface() override;
    
    /**
     * Initialize the PWM interface
     * 
     * @param config Pointer to PWM configuration
     * @return Status code indicating success or failure
     */
    Platform::Status Init(void* config) override;
    
    /**
     * De-initialize the PWM interface and release resources
     * 
     * @return Status code indicating success or failure
     */
    Platform::Status DeInit() override;
    
    /**
     * Control the PWM interface with specific commands
     * 
     * @param command Control command identifier
     * @param param Command-specific parameter
     * @return Status code indicating success or failure
     */
    Platform::Status Control(uint32_t command, void* param) override;
    
    /**
     * Read PWM status information
     * Not typically used for PWM operation, primarily redirects to Configure method
     * 
     * @param buffer Buffer to store the read data
     * @param size Size of data to read (in bytes)
     * @param timeout Timeout for the operation (in milliseconds)
     * @return Status code indicating success or failure
     */
    Platform::Status Read(void* buffer, uint16_t size, uint32_t timeout) override;
    
    /**
     * Write PWM configuration data
     * Not typically used for PWM operation, primarily redirects to Configure method
     * 
     * @param data Data to write
     * @param size Size of data to write (in bytes)
     * @param timeout Timeout for the operation (in milliseconds)
     * @return Status code indicating success or failure
     */
    Platform::Status Write(const void* data, uint16_t size, uint32_t timeout) override;
    
    /**
     * Register a callback function for PWM events
     * 
     * @param eventId Event identifier
     * @param callback Callback function pointer
     * @param param Parameter to pass to the callback function
     * @return Status code indicating success or failure
     */
    Platform::Status RegisterCallback(uint32_t eventId, 
                                     void (*callback)(void* param), 
                                     void* param) override;
    
    /**
     * Start PWM generation
     * 
     * @return Status code indicating success or failure
     */
    Platform::Status Start();
    
    /**
     * Stop PWM generation
     * 
     * @return Status code indicating success or failure
     */
    Platform::Status Stop();
    
    /**
     * Set PWM frequency
     * 
     * @param frequency PWM frequency in Hz
     * @return Status code indicating success or failure
     */
    Platform::Status SetFrequency(uint32_t frequency);
    
    /**
     * Set duty cycle for a specific channel
     * 
     * @param channel PWM channel
     * @param duty_cycle Duty cycle in 0.01% (0-10000, where 10000 = 100.00%)
     * @return Status code indicating success or failure
     */
    Platform::Status SetDutyCycle(PWMChannel channel, uint32_t duty_cycle);
    
    /**
     * Configure a PWM channel
     * 
     * @param config Channel configuration
     * @return Status code indicating success or failure
     */
    Platform::Status ConfigureChannel(const PWMChannelConfig& config);
    
    /**
     * Enable a specific PWM channel
     * 
     * @param channel PWM channel to enable
     * @return Status code indicating success or failure
     */
    Platform::Status EnableChannel(PWMChannel channel);
    
    /**
     * Disable a specific PWM channel
     * 
     * @param channel PWM channel to disable
     * @return Status code indicating success or failure
     */
    Platform::Status DisableChannel(PWMChannel channel);
    
    /**
     * Set the operating mode of the PWM
     * 
     * @param mode PWM operating mode
     * @return Status code indicating success or failure
     */
    Platform::Status SetMode(PWMMode mode);
    
    /**
     * Get the current frequency of the PWM in Hz
     * 
     * @return Current PWM frequency
     */
    uint32_t GetFrequency() const;
    
    /**
     * Get the current duty cycle of a specific channel
     * 
     * @param channel PWM channel
     * @return Duty cycle in 0.01% (0-10000, where 10000 = 100.00%)
     */
    uint32_t GetDutyCycle(PWMChannel channel) const;
    
    /**
     * Check if a specific channel is active
     * 
     * @param channel PWM channel
     * @return true if channel is active, false otherwise
     */
    bool IsChannelActive(PWMChannel channel) const;
    
    /**
     * Check if the PWM generator is running
     * 
     * @return true if PWM is running, false otherwise
     */
    bool IsRunning() const;
};

} // namespace PWM
} // namespace Platform