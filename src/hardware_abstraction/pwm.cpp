#include "hardware_abstraction/pwm.hpp"
#include "hardware_abstraction/rcc.hpp"
#include "hardware_abstraction/timer.hpp"
#include "common/platform_tim.hpp"
#include "middleware/system_services/system_timing.hpp"
#include <algorithm>

namespace Platform {
namespace PWM {

// Initialize static mutex
OS::mutex PWMInterface::instances_mutex;

// Constructor
PWMInterface::PWMInterface() : initialized(false), period_value(0), prescaler_value(0) {
    // Initialize channel activity tracking
    for (bool& active : channel_active) {
        active = false;
    }
    
    // Initialize callbacks
    for (auto& cb : callbacks) {
        cb.callback = nullptr;
        cb.param = nullptr;
        cb.enabled = false;
    }
    
    // Get required interfaces
    gpio_interface = &Platform::GPIO::GpioInterface::GetInstance();
}

// Destructor
PWMInterface::~PWMInterface() {
    if (initialized) {
        DeInit();
    }
}

// Singleton instance getter
PWMInterface& PWMInterface::GetInstance() {
    static PWMInterface instance;
    return instance;
}

// Initialize the PWM interface
Platform::Status PWMInterface::Init(void* config_ptr) {
    // Check parameters
    if (!config_ptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // If already initialized, deinitialize first
    if (initialized) {
        DeInit();
    }
    
    // Copy configuration
    config = *static_cast<PWMConfig*>(config_ptr);
    
    // Validate and update configuration
    if (config.resolution_bits < 8 || config.resolution_bits > 16) {
        // Default to 10-bit resolution if invalid
        config.resolution_bits = 10;
    }
    
    // Get the timer interface
    timer_interface = timer_interface = &Platform::TIM::TimerInterface::GetInstance(config.timer_instance);;
    if (!timer_interface) {
        return Platform::Status::ERROR;
    }
    
    // Check if timer supports the requested mode
    if (config.mode == PWMMode::Complementary) {
        // Only advanced timers (TIM1, TIM8) support complementary outputs
        Platform::TIM::TimerType timer_type = timer_interface->GetTimerType();
        if (timer_type != Platform::TIM::TimerType::Advanced) {
            return Platform::Status::NOT_SUPPORTED;
        }
    }
    
    // Calculate timer parameters based on frequency
    Platform::Status status = CalculateTimerParameters(config.frequency);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Configure the timer
    TIM::TimerConfig timer_config = {
        .desiredFrequency = 0, // We'll set prescaler and period directly
        .timerInstance = config.timer_instance,
        .mode = TIM::Mode::PWM,
        .clockDivision = TIM::ClockDivision::Div1,
        .direction = (config.alignment == PWMAlignment::CenterAligned) ? TIM::Direction::Up : TIM::Direction::Up,
        .alignment = (config.alignment == PWMAlignment::CenterAligned) ? TIM::Alignment::Center1 : TIM::Alignment::Edge,
        .prescaler = prescaler_value,
        .period = period_value,
        .autoReloadPreload = true
    };
    
    // Initialize the timer
    status = timer_interface->Init(&timer_config);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // If using one-pulse mode, configure it
    if (config.mode == PWMMode::OnePulse) {
        // Enable one-pulse mode in timer CR1 register
        TIM::TimerOperation op = TIM::TimerOperation::ConfigureChannel;
        uint32_t one_pulse_mode = 1; // One-pulse mode value
        timer_interface->Control(TIMER_CTRL_SET_MODE, &one_pulse_mode);
    }
    
    // If using dead time insertion, configure it (for advanced timers)
    if (config.use_dead_time && config.mode == PWMMode::Complementary) {
        // TODO: Configure dead time insertion in BDTR register
        // This would be specific to advanced timers and requires additional implementation
    }
    
    // Mark as initialized
    initialized = true;
    
    return Platform::Status::OK;
}

// Deinitialize the PWM interface
Platform::Status PWMInterface::DeInit() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Stop PWM generation
    Stop();
    
    // Reset configuration and state
    for (bool& active : channel_active) {
        active = false;
    }
    
    // Deinitialize timer
    if (timer_interface) {
        timer_interface->DeInit();
    }
    
    // Clear internal state
    initialized = false;
    
    return Platform::Status::OK;
}

// Calculate timer parameters based on desired frequency
Platform::Status PWMInterface::CalculateTimerParameters(uint32_t frequency) {
    if (frequency == 0) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Get timer clock frequency
    auto& rcc = RCC::RccInterface::GetInstance();
    uint32_t timer_clock;
    
    // Determine which bus this timer is on
    if (config.timer_instance == TIM::TimerInstance::TIM1 || 
        config.timer_instance == TIM::TimerInstance::TIM9 || 
        config.timer_instance == TIM::TimerInstance::TIM10 || 
        config.timer_instance == TIM::TimerInstance::TIM11) {
        timer_clock = rcc.GetApb2ClockFrequency();
        
        // If APB2 prescaler is not 1, the timer clock is doubled
        if (timer_clock < rcc.GetSystemClockFrequency()) {
            timer_clock *= 2;
        }
    } else {
        timer_clock = rcc.GetApb1ClockFrequency();
        
        // If APB1 prescaler is not 1, the timer clock is doubled
        if (timer_clock < rcc.GetSystemClockFrequency()) {
            timer_clock *= 2;
        }
    }
    
    // Calculate period value based on resolution
    // Period = 2^resolution - 1
    period_value = (1 << config.resolution_bits) - 1;
    
    // Calculate prescaler value based on frequency
    // prescaler = (timer_clock / (frequency * (period + 1))) - 1
    uint32_t prescaler_raw = (timer_clock / (frequency * (period_value + 1)));
    
    // Check if the calculation results in a valid prescaler value
    if (prescaler_raw == 0 || prescaler_raw > 65536) {
        // Try to adjust period value to get a valid prescaler
        
        // If prescaler is too small, decrease the period
        if (prescaler_raw == 0) {
            // Find a smaller period value that gives a valid prescaler
            for (uint32_t test_period = period_value / 2; test_period >= 100; test_period /= 2) {
                prescaler_raw = (timer_clock / (frequency * (test_period + 1)));
                if (prescaler_raw >= 1 && prescaler_raw <= 65536) {
                    period_value = test_period;
                    break;
                }
            }
        }
        // If prescaler is too large, increase the period
        else {
            // Find a larger period value that gives a valid prescaler
            for (uint32_t test_period = period_value * 2; test_period <= 65535; test_period *= 2) {
                prescaler_raw = (timer_clock / (frequency * (test_period + 1)));
                if (prescaler_raw >= 1 && prescaler_raw <= 65536) {
                    period_value = test_period;
                    break;
                }
            }
        }
        
        // If we couldn't find a valid combination
        if (prescaler_raw == 0 || prescaler_raw > 65536) {
            return Platform::Status::INVALID_PARAM;
        }
    }
    
    // Store the calculated prescaler value
    prescaler_value = prescaler_raw - 1;
    
    return Platform::Status::OK;
}

// Configure a GPIO pin for PWM output
Platform::Status PWMInterface::ConfigurePWMPin(const PWMChannelConfig& config) {
    if (!gpio_interface) {
        return Platform::Status::ERROR;
    }
    
    // Create GPIO configuration for PWM output
    GPIO::GpioConfig gpio_config = {
        .port = config.gpio_port,
        .pin = config.gpio_pin,
        .mode = GPIO::Mode::AlternateFunction,
        .outputType = GPIO::OutputType::PushPull,
        .pull = GPIO::Pull::None,
        .speed = GPIO::Speed::High,
        .af = config.gpio_af
    };
    
    // Configure the GPIO pin
    return gpio_interface->ConfigurePin(gpio_config);
}

// Configure a timer channel for PWM operation
Platform::Status PWMInterface::ConfigureTimerChannel(const PWMChannelConfig& config) {
    if (!timer_interface) {
        return Platform::Status::ERROR;
    }
    
    // Map PWM channel to timer channel
    TIM::Channel timer_channel;
    switch (config.channel) {
        case PWMChannel::Channel1:
            timer_channel = TIM::Channel::Channel1;
            break;
        case PWMChannel::Channel2:
            timer_channel = TIM::Channel::Channel2;
            break;
        case PWMChannel::Channel3:
            timer_channel = TIM::Channel::Channel3;
            break;
        case PWMChannel::Channel4:
            timer_channel = TIM::Channel::Channel4;
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Calculate compare value from duty cycle
    uint32_t compare_value = DutyCycleToCompareValue(config.duty_cycle);
    
    // Create timer channel configuration
    TIM::TimerChannelConfig timer_config = {
        .channel = timer_channel,
        .ocMode = (config.polarity == PWMPolarity::ActiveHigh) ? TIM::OCMode::PWM1 : TIM::OCMode::PWM2,
        .pulse = compare_value,
        .ocPreload = true,
        .complementaryOutput = config.complementary_output
    };
    
    // Configure the timer channel
    return timer_interface->ConfigureChannel(timer_config);
}

// Convert duty cycle percentage to compare value
uint32_t PWMInterface::DutyCycleToCompareValue(uint32_t duty_cycle_percent) const {
    // Clamp duty cycle to valid range (0-10000)
    duty_cycle_percent = std::min<uint32_t>(duty_cycle_percent, 10000U);
    
    // Calculate compare value based on period and duty cycle
    // Compare = (period * duty_cycle) / 10000
    return (period_value * duty_cycle_percent) / 10000;
}

// Convert compare value to duty cycle percentage
uint32_t PWMInterface::CompareValueToDutyCycle(uint32_t compare_value) const {
    // Calculate duty cycle based on compare value and period
    // Duty cycle = (compare * 10000) / period
    return (compare_value * 10000) / period_value;
}

// Static callback handler for timer events
void PWMInterface::TimerEventCallback(void* param) {
    PWMInterface* pwm = static_cast<PWMInterface*>(param);
    if (pwm && pwm->callbacks[static_cast<uint32_t>(PWMEvent::Update)].enabled) {
        pwm->callbacks[static_cast<uint32_t>(PWMEvent::Update)].callback(
            pwm->callbacks[static_cast<uint32_t>(PWMEvent::Update)].param
        );
    }
}

// Start PWM generation
Platform::Status PWMInterface::Start() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (!timer_interface) {
        return Platform::Status::ERROR;
    }
    
    // Start the timer
    return timer_interface->Start();
}

// Stop PWM generation
Platform::Status PWMInterface::Stop() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (!timer_interface) {
        return Platform::Status::ERROR;
    }
    
    // Stop the timer
    return timer_interface->Stop();
}

// Set PWM frequency
Platform::Status PWMInterface::SetFrequency(uint32_t frequency) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (!timer_interface) {
        return Platform::Status::ERROR;
    }
    
    // Calculate new timer parameters
    Platform::Status status = CalculateTimerParameters(frequency);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Set new prescaler
    status = timer_interface->SetPrescaler(prescaler_value);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Set new period
    status = timer_interface->SetPeriod(period_value);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Update duty cycles for all active channels to maintain the same percentage
    for (uint8_t i = 0; i < 4; i++) {
        if (channel_active[i]) {
            PWMChannel channel = static_cast<PWMChannel>(i);
            uint32_t duty_cycle = GetDutyCycle(channel);
            SetDutyCycle(channel, duty_cycle);
        }
    }
    
    // Update config
    config.frequency = frequency;
    
    return Platform::Status::OK;
}

// Set duty cycle for a specific channel
Platform::Status PWMInterface::SetDutyCycle(PWMChannel channel, uint32_t duty_cycle) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (!timer_interface) {
        return Platform::Status::ERROR;
    }
    
    // Check if channel is active
    if (!channel_active[static_cast<uint8_t>(channel)]) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Clamp duty cycle to valid range (0-10000)
    duty_cycle = std::min<uint32_t>(duty_cycle, 10000U);
    
    // Calculate compare value
    uint32_t compare_value = DutyCycleToCompareValue(duty_cycle);
    
    // Map PWM channel to timer channel
    TIM::Channel timer_channel;
    switch (channel) {
        case PWMChannel::Channel1:
            timer_channel = TIM::Channel::Channel1;
            break;
        case PWMChannel::Channel2:
            timer_channel = TIM::Channel::Channel2;
            break;
        case PWMChannel::Channel3:
            timer_channel = TIM::Channel::Channel3;
            break;
        case PWMChannel::Channel4:
            timer_channel = TIM::Channel::Channel4;
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Create compare config
    TIM::TimerCompareConfig compare_config = {
        .channel = timer_channel,
        .value = compare_value
    };
    
    // Set compare value
    Platform::Status status = timer_interface->Control(TIMER_CTRL_SET_COMPARE, &compare_config);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Update channel configuration
    channel_configs[static_cast<uint8_t>(channel)].duty_cycle = duty_cycle;
    
    return Platform::Status::OK;
}

// Configure a PWM channel
Platform::Status PWMInterface::ConfigureChannel(const PWMChannelConfig& config) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    uint8_t channel_idx = static_cast<uint8_t>(config.channel);
    
    // Configure the GPIO pin for PWM output
    Platform::Status status = ConfigurePWMPin(config);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Configure the timer channel for PWM
    status = ConfigureTimerChannel(config);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Store channel configuration
    channel_configs[channel_idx] = config;
    channel_active[channel_idx] = true;
    
    return Platform::Status::OK;
}

// Enable a specific PWM channel
Platform::Status PWMInterface::EnableChannel(PWMChannel channel) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (!timer_interface) {
        return Platform::Status::ERROR;
    }
    
    // Map PWM channel to timer channel
    TIM::Channel timer_channel;
    switch (channel) {
        case PWMChannel::Channel1:
            timer_channel = TIM::Channel::Channel1;
            break;
        case PWMChannel::Channel2:
            timer_channel = TIM::Channel::Channel2;
            break;
        case PWMChannel::Channel3:
            timer_channel = TIM::Channel::Channel3;
            break;
        case PWMChannel::Channel4:
            timer_channel = TIM::Channel::Channel4;
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Check if channel has been configured
    if (!channel_active[static_cast<uint8_t>(channel)]) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Enable the timer channel
    return timer_interface->Control(TIMER_CTRL_ENABLE_CHANNEL, &timer_channel);
}

// Disable a specific PWM channel
Platform::Status PWMInterface::DisableChannel(PWMChannel channel) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (!timer_interface) {
        return Platform::Status::ERROR;
    }
    
    // Map PWM channel to timer channel
    TIM::Channel timer_channel;
    switch (channel) {
        case PWMChannel::Channel1:
            timer_channel = TIM::Channel::Channel1;
            break;
        case PWMChannel::Channel2:
            timer_channel = TIM::Channel::Channel2;
            break;
        case PWMChannel::Channel3:
            timer_channel = TIM::Channel::Channel3;
            break;
        case PWMChannel::Channel4:
            timer_channel = TIM::Channel::Channel4;
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Check if channel is active
    if (!channel_active[static_cast<uint8_t>(channel)]) {
        return Platform::Status::OK; // Already disabled
    }
    
    // Disable the timer channel
    Platform::Status status = timer_interface->Control(TIMER_CTRL_DISABLE_CHANNEL, &timer_channel);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Mark channel as inactive
    channel_active[static_cast<uint8_t>(channel)] = false;
    
    return Platform::Status::OK;
}

// Set the operating mode of the PWM
Platform::Status PWMInterface::SetMode(PWMMode mode) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (!timer_interface) {
        return Platform::Status::ERROR;
    }
    
    // Check if the requested mode is supported by the timer
    if (mode == PWMMode::Complementary) {
        Platform::TIM::TimerType timer_type = timer_interface->GetTimerType();
        if (timer_type != Platform::TIM::TimerType::Advanced) {
            return Platform::Status::NOT_SUPPORTED;
        }
    }
    
    // Special handling for one-pulse mode
    if (mode == PWMMode::OnePulse) {
        // Enable one-pulse mode in timer CR1 register
        uint32_t one_pulse_mode = 1; // One-pulse mode value
        timer_interface->Control(TIMER_CTRL_SET_MODE, &one_pulse_mode);
    } else {
        // Disable one-pulse mode
        uint32_t normal_mode = 0; // Normal mode value
        timer_interface->Control(TIMER_CTRL_SET_MODE, &normal_mode);
    }
    
    // Update configuration
    config.mode = mode;
    
    return Platform::Status::OK;
}

// Get the current frequency of the PWM in Hz
uint32_t PWMInterface::GetFrequency() const {
    return config.frequency;
}

// Get the current duty cycle of a specific channel
uint32_t PWMInterface::GetDutyCycle(PWMChannel channel) const {
    if (!initialized || !timer_interface) {
        return 0;
    }
    
    uint8_t channel_idx = static_cast<uint8_t>(channel);
    
    // Check if channel is active
    if (!channel_active[channel_idx]) {
        return 0;
    }
    
    // Get the current compare value from the timer
    TIM::Channel timer_channel;
    switch (channel) {
        case PWMChannel::Channel1:
            timer_channel = TIM::Channel::Channel1;
            break;
        case PWMChannel::Channel2:
            timer_channel = TIM::Channel::Channel2;
            break;
        case PWMChannel::Channel3:
            timer_channel = TIM::Channel::Channel3;
            break;
        case PWMChannel::Channel4:
            timer_channel = TIM::Channel::Channel4;
            break;
        default:
            return 0;
    }
    
    // We need to read directly from the timer's CCR register
    // Since there's no direct method in the TimerInterface, we'll use the stored config
    return channel_configs[channel_idx].duty_cycle;
}

// Check if a specific channel is active
bool PWMInterface::IsChannelActive(PWMChannel channel) const {
    if (!initialized) {
        return false;
    }
    
    return channel_active[static_cast<uint8_t>(channel)];
}

// Check if the PWM generator is running
bool PWMInterface::IsRunning() const {
    if (!initialized || !timer_interface) {
        return false;
    }
    
    // Get timer status
    TIM::TimerStatus timer_status;
    timer_interface->Control(TIMER_CTRL_GET_STATUS, &timer_status);
    
    return timer_status.is_running;
}

// Control function implementation
Platform::Status PWMInterface::Control(uint32_t command, void* param) {
    if (!initialized && command != PWM_CTRL_START) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    switch (command) {
        case PWM_CTRL_START:
            return Start();
            
        case PWM_CTRL_STOP:
            return Stop();
            
        case PWM_CTRL_SET_FREQUENCY:
            if (!param) return Platform::Status::INVALID_PARAM;
            return SetFrequency(*static_cast<uint32_t*>(param));
            
        case PWM_CTRL_SET_DUTY_CYCLE: {
            if (!param) return Platform::Status::INVALID_PARAM;
            
            // Param is a struct with channel and duty cycle
            struct DutyCycleParam {
                PWMChannel channel;
                uint32_t duty_cycle;
            };
            
            auto* duty_param = static_cast<DutyCycleParam*>(param);
            return SetDutyCycle(duty_param->channel, duty_param->duty_cycle);
        }
            
        case PWM_CTRL_SET_CHANNEL_CONFIG:
            if (!param) return Platform::Status::INVALID_PARAM;
            return ConfigureChannel(*static_cast<PWMChannelConfig*>(param));
            
        case PWM_CTRL_ENABLE_CHANNEL:
            if (!param) return Platform::Status::INVALID_PARAM;
            return EnableChannel(*static_cast<PWMChannel*>(param));
            
        case PWM_CTRL_DISABLE_CHANNEL:
            if (!param) return Platform::Status::INVALID_PARAM;
            return DisableChannel(*static_cast<PWMChannel*>(param));
            
        case PWM_CTRL_SET_POLARITY: {
            if (!param) return Platform::Status::INVALID_PARAM;
            
            // Param is a struct with channel and polarity
            struct PolarityParam {
                PWMChannel channel;
                PWMPolarity polarity;
            };
            
            auto* pol_param = static_cast<PolarityParam*>(param);
            uint8_t channel_idx = static_cast<uint8_t>(pol_param->channel);
            
            // Update channel configuration
            if (channel_active[channel_idx]) {
                channel_configs[channel_idx].polarity = pol_param->polarity;
                
                // Reconfigure the channel to apply the new polarity
                return ConfigureChannel(channel_configs[channel_idx]);
            }
            
            return Platform::Status::NOT_INITIALIZED;
        }
            
        case PWM_CTRL_ENABLE_DEADTIME: {
            if (!param) return Platform::Status::INVALID_PARAM;
            
            // Only available on advanced timers
            Platform::TIM::TimerType timer_type = timer_interface->GetTimerType();
            if (timer_type != Platform::TIM::TimerType::Advanced) {
                return Platform::Status::NOT_SUPPORTED;
            }
            
            // Enable dead time with specified value
            config.use_dead_time = true;
            config.dead_time_ns = *static_cast<uint16_t*>(param);
            
            // TODO: Implement actual dead time configuration in BDTR register
            // This would be specific to advanced timers and requires additional implementation
            
            return Platform::Status::OK;
        }
            
        case PWM_CTRL_DISABLE_DEADTIME: {
            // Only available on advanced timers
            Platform::TIM::TimerType timer_type = timer_interface->GetTimerType();
            if (timer_type != Platform::TIM::TimerType::Advanced) {
                return Platform::Status::NOT_SUPPORTED;
            }
            
            // Disable dead time
            config.use_dead_time = false;
            
            // TODO: Implement actual dead time disabling in BDTR register
            
            return Platform::Status::OK;
        }
            
        case PWM_CTRL_SET_MODE:
            if (!param) return Platform::Status::INVALID_PARAM;
            return SetMode(*static_cast<PWMMode*>(param));
            
        case PWM_CTRL_SET_ALL_DUTY_CYCLE: {
            if (!param) return Platform::Status::INVALID_PARAM;
            
            // Set same duty cycle for all active channels
            uint32_t duty_cycle = *static_cast<uint32_t*>(param);
            
            // Iterate through all channels
            Platform::Status overall_status = Platform::Status::OK;
            for (uint8_t i = 0; i < 4; i++) {
                if (channel_active[i]) {
                    Platform::Status status = SetDutyCycle(static_cast<PWMChannel>(i), duty_cycle);
                    if (status != Platform::Status::OK) {
                        overall_status = status;
                    }
                }
            }
            
            return overall_status;
        }
            
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
}

// Read implementation - providing PWM status information
Platform::Status PWMInterface::Read(void* buffer, uint16_t size, uint32_t timeout) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // We'll define a struct for the PWM status info
    struct PWMStatus {
        uint32_t frequency;
        uint32_t period;
        uint32_t prescaler;
        uint32_t duty_cycles[4];
        bool channel_active[4];
        bool is_running;
        PWMMode mode;
    };
    
    // Check buffer size
    if (!buffer || size < sizeof(PWMStatus)) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Get channel duty cycles
    PWMStatus* status = static_cast<PWMStatus*>(buffer);
    status->frequency = config.frequency;
    status->period = period_value;
    status->prescaler = prescaler_value;
    status->is_running = IsRunning();
    status->mode = config.mode;
    
    for (uint8_t i = 0; i < 4; i++) {
        status->channel_active[i] = channel_active[i];
        status->duty_cycles[i] = channel_active[i] ? GetDutyCycle(static_cast<PWMChannel>(i)) : 0;
    }
    
    return Platform::Status::OK;
}

// Write implementation - mostly for convenience, maps to other explicit methods
Platform::Status PWMInterface::Write(const void* data, uint16_t size, uint32_t timeout) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Define possible write operations
    enum class WriteOperation : uint8_t {
        SetDutyCycle = 0,
        SetFrequency = 1
    };
    
    // Define a structure for the Write operation
    struct WriteData {
        WriteOperation operation;
        union {
            struct {
                PWMChannel channel;
                uint32_t duty_cycle;
            } duty_cycle;
            uint32_t frequency;
        };
    };
    
    // Check data size
    if (!data || size < sizeof(WriteData)) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Process the operation
    const WriteData* write_data = static_cast<const WriteData*>(data);
    
    switch (write_data->operation) {
        case WriteOperation::SetDutyCycle:
            return SetDutyCycle(write_data->duty_cycle.channel, write_data->duty_cycle.duty_cycle);
            
        case WriteOperation::SetFrequency:
            return SetFrequency(write_data->frequency);
            
        default:
            return Platform::Status::INVALID_PARAM;
    }
}

// Register callback function
Platform::Status PWMInterface::RegisterCallback(uint32_t eventId, void (*callback)(void* param), void* param) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Validate event ID
    if (eventId >= static_cast<uint32_t>(PWMEvent::Max)) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Store callback information
    callbacks[eventId].callback = callback;
    callbacks[eventId].param = param;
    callbacks[eventId].enabled = (callback != nullptr);
    
    // If we're registering an update event callback, link it to the timer
    if (eventId == static_cast<uint32_t>(PWMEvent::Update) && callback) {
        // Register with timer interrupt
        return timer_interface->RegisterCallback(
            static_cast<uint32_t>(TIM::TimerEvent::Update),
            TimerEventCallback,
            this
        );
    }
    
    return Platform::Status::OK;
}

} // namespace PWM
} // namespace Platform