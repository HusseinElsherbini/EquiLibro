#include "actuators/vnh5019.hpp"

namespace Drivers {
namespace Motor {

VNH5019Driver::VNH5019Driver() 
    : initialized(false)
    , current_direction(Direction::Coast)
    , target_speed(0)
    , current_speed(0)
    , fault_detected(false)
    , pwm_interface(nullptr)
    , gpio_interface(nullptr)
    , adc_interface(nullptr) {
}

VNH5019Driver::~VNH5019Driver() {
    if (initialized) {
        Disable();
    }
}

Platform::Status VNH5019Driver::Init(const VNH5019Config* config_ptr) {
    if (!config_ptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Store configuration
    config = *config_ptr;
    
    // Get interfaces
    pwm_interface = &Platform::PWM::PWMInterface::GetInstance();
    gpio_interface = &Platform::GPIO::GpioInterface::GetInstance();
    
    if (config.use_current_sensing) {
        adc_interface = &Platform::ADC::AdcInterface::GetInstance();
    }

    
    Platform::Status status = pwm_interface->ConfigureChannel(config.pwm_ch_config);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    status = pwm_interface->EnableChannel(config.pwm_ch_config.channel);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Configure direction control pins (INA, INB)
    Platform::GPIO::GpioConfig ina_config = {
        .port = config.ina_port,
        .pin = config.ina_pin,
        .mode = Platform::GPIO::Mode::Output,
        .outputType = Platform::GPIO::OutputType::PushPull,
        .pull = Platform::GPIO::Pull::None,
        .speed = Platform::GPIO::Speed::Medium
    };
    
    status = gpio_interface->ConfigurePin(ina_config);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    Platform::GPIO::GpioConfig inb_config = {
        .port = config.inb_port,
        .pin = config.inb_pin,
        .mode = Platform::GPIO::Mode::Output,
        .outputType = Platform::GPIO::OutputType::PushPull,
        .pull = Platform::GPIO::Pull::None,
        .speed = Platform::GPIO::Speed::Medium
    };
    
    status = gpio_interface->ConfigurePin(inb_config);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Configure enable pin if used
    if (config.use_enable_pin) {
        Platform::GPIO::GpioConfig enable_config = {
            .port = config.enable_port,
            .pin = config.enable_pin,
            .mode = Platform::GPIO::Mode::Output,
            .outputType = Platform::GPIO::OutputType::PushPull,
            .pull = Platform::GPIO::Pull::None,
            .speed = Platform::GPIO::Speed::Medium
        };
        
        status = gpio_interface->ConfigurePin(enable_config);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Start with motor disabled
        gpio_interface->ResetPin(config.enable_port, config.enable_pin);
    }
    
    // Configure diagnostic pin if used
    if (config.use_diag_pin) {
        Platform::GPIO::GpioConfig diag_config = {
            .port = config.diag_port,
            .pin = config.diag_pin,
            .mode = Platform::GPIO::Mode::Input,
            .pull = Platform::GPIO::Pull::PullUp,  // VNH5019 DIAG is active-low
        };
        
        status = gpio_interface->ConfigurePin(diag_config);
        if (status != Platform::Status::OK) {
            return status;
        }
        
    }
    
    // Initialize current sensing if used
    if (config.use_current_sensing) {
        // Configure ADC for current sensing
        // Implementation depends on your ADC interface
    }
    
    // Default to coast mode (both INA and INB low)
    SetDirectionPins(Direction::Coast);
    
    // Start PWM generation (with 0% duty cycle)
    status = pwm_interface->Start();
    if (status != Platform::Status::OK) {
        return status;
    }
    
    initialized = true;
    return Platform::Status::OK;
}

Platform::Status VNH5019Driver::Enable() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (config.use_enable_pin) {
        gpio_interface->SetPin(config.enable_port, config.enable_pin);
    }
    
    return Platform::Status::OK;
}

Platform::Status VNH5019Driver::Disable() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Set direction pins to coast mode
    SetDirectionPins(Direction::Coast);
    
    // Set PWM duty cycle to 0
    pwm_interface->SetDutyCycle(config.pwm_ch_config.channel, 0);
    
    if (config.use_enable_pin) {
        gpio_interface->ResetPin(config.enable_port, config.enable_pin);
    }
    
    current_speed = 0;
    target_speed = 0;
    current_direction = Direction::Coast;
    
    return Platform::Status::OK;
}

void VNH5019Driver::SetDirectionPins(Direction direction) {
    switch (direction) {
        case Direction::Forward:
            gpio_interface->SetPin(config.ina_port, config.ina_pin);     // INA = HIGH
            gpio_interface->ResetPin(config.inb_port, config.inb_pin);   // INB = LOW
            break;
            
        case Direction::Reverse:
            gpio_interface->ResetPin(config.ina_port, config.ina_pin);   // INA = LOW
            gpio_interface->SetPin(config.inb_port, config.inb_pin);     // INB = HIGH
            break;
            
        case Direction::Brake:
            gpio_interface->SetPin(config.ina_port, config.ina_pin);     // INA = HIGH
            gpio_interface->SetPin(config.inb_port, config.inb_pin);     // INB = HIGH
            break;
            
        case Direction::Coast:
        default:
            gpio_interface->ResetPin(config.ina_port, config.ina_pin);   // INA = LOW
            gpio_interface->ResetPin(config.inb_port, config.inb_pin);   // INB = LOW
            break;
    }
    
    current_direction = direction;
}

Platform::Status VNH5019Driver::SetSpeed(int32_t speed) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Clamp speed to valid range (-10000 to +10000)
    speed = std::max<uint32_t>(-10000, std::min<uint32_t>(10000, speed));
    
    // Determine direction based on speed sign
    Direction new_direction;
    if (speed > 0) {
        new_direction = Direction::Forward;
    } else if (speed < 0) {
        new_direction = Direction::Reverse;
    } else {
        new_direction = Direction::Brake;  // Zero speed means brake
    }
    
    // Set direction pins if direction changed
    if (new_direction != current_direction) {
        SetDirectionPins(new_direction);
    }
    
    // Set target speed (absolute value)
    target_speed = static_cast<uint32_t>(std::abs(speed));
    
    // If we're not gradually ramping speed, set it immediately
    if (config.acceleration_rate == 0) {
        current_speed = target_speed;
        
        // Apply min/max limits
        uint32_t limited_speed = std::max(config.min_duty_cycle, 
                                        std::min(target_speed, config.max_duty_cycle));
        
        // Set PWM duty cycle
        pwm_interface->SetDutyCycle(config.pwm_ch_config.channel, limited_speed);
    }
    
    return Platform::Status::OK;
}

Platform::Status VNH5019Driver::SetSpeedAndDirection(uint32_t speed, Direction direction) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Clamp speed to valid range (0 to 10000)
    speed = std::min<uint32_t>(10000U, speed);
    
    // Set direction pins if direction changed
    if (direction != current_direction) {
        SetDirectionPins(direction);
    }
    
    // Set target speed
    target_speed = speed;
    
    // If we're not gradually ramping speed, set it immediately
    if (config.acceleration_rate == 0) {
        current_speed = target_speed;
        
        // Apply min/max limits
        uint32_t limited_speed = std::max(config.min_duty_cycle, 
                                        std::min(target_speed, config.max_duty_cycle));
        
        // Only apply PWM if not in coast mode
        if (direction != Direction::Coast) {
            pwm_interface->SetDutyCycle(config.pwm_ch_config.channel, limited_speed);
        } else {
            pwm_interface->SetDutyCycle(config.pwm_ch_config.channel, 0);
        }
    }
    
    return Platform::Status::OK;
}

Platform::Status VNH5019Driver::Stop(bool brake) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Set target speed to 0
    target_speed = 0;
    
    // If immediate stop requested
    if (config.acceleration_rate == 0) {
        current_speed = 0;
        pwm_interface->SetDutyCycle(config.pwm_ch_config.channel, 0);
    }
    
    // Set direction pins according to brake mode
    SetDirectionPins(brake ? Direction::Brake : Direction::Coast);
    
    return Platform::Status::OK;
}

int32_t VNH5019Driver::GetSpeed() const {
    // Return signed speed value based on direction
    if (current_direction == Direction::Forward) {
        return static_cast<int32_t>(current_speed);
    } else if (current_direction == Direction::Reverse) {
        return -static_cast<int32_t>(current_speed);
    } else {
        return 0;  // Brake or Coast
    }
}

Direction VNH5019Driver::GetDirection() const {
    return current_direction;
}

bool VNH5019Driver::IsFaultDetected() const {
    return fault_detected;
}

Platform::Status VNH5019Driver::ClearFault() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Check if fault is still present
    if (config.use_diag_pin) {
        Platform::GPIO::GpioPinState diag_state;
        Platform::Status status = gpio_interface->ReadPin(config.diag_port, config.diag_pin, diag_state);
        
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // If DIAG pin is still low, fault is still present
        if (static_cast<uint8_t>(diag_state) == 0) {
            return Platform::Status::ERROR;
        }
    }
    
    // Clear fault flag
    fault_detected = false;
    
    return Platform::Status::OK;
}

float VNH5019Driver::ReadMotorCurrent() {
    if (!initialized || !config.use_current_sensing || !adc_interface) {
        return 0.0f;
    }
    
    // Read current sense value
    uint32_t adc_value = ReadCurrentSenseFeedback();
    
    // Convert ADC value to current in Amps
    // Implementation depends on your ADC reference voltage and VNH5019 current sense ratio
    // Typical VNH5019 current sense is 0.14 V/A
    float voltage = (adc_value * 3.3f) / 4096.0f;  // Assuming 12-bit ADC with 3.3V reference
    float current = voltage / config.current_sense_ratio;
    
    return current;
}

void VNH5019Driver::Process() {
    if (!initialized) {
        return;
    }
    
    // Check for faults
    if (config.use_diag_pin && !fault_detected) {
        fault_detected = CheckForFault();
        
        // If fault detected, put motor in safe state
        if (fault_detected) {
            SetDirectionPins(Direction::Coast);
            pwm_interface->SetDutyCycle(config.pwm_ch_config.channel, 0);
            current_speed = 0;
            
            // You could also trigger an event/callback here to notify the application
        }
    }
    
    // Skip speed ramping if fault detected
    if (fault_detected) {
        return;
    }
    
    // Handle speed ramping if enabled
    if (config.acceleration_rate > 0 && current_speed != target_speed) {
        // Get system time for acceleration calculation
        auto& timing = Middleware::SystemServices::SystemTiming::GetInstance();
        static uint64_t last_update_time = timing.GetMilliseconds();
        
        uint64_t current_time = timing.GetMilliseconds();
        uint32_t elapsed_ms = static_cast<uint32_t>(current_time - last_update_time);
        
        if (elapsed_ms > 0) {
            // Calculate max change in speed for this time period
            uint32_t max_speed_change = (config.acceleration_rate * elapsed_ms) / 1000;
            
            // Accelerate or decelerate towards target
            if (current_speed < target_speed) {
                current_speed = std::min(target_speed, current_speed + max_speed_change);
            } else if (current_speed > target_speed) {
                current_speed = (current_speed > max_speed_change) ? 
                    current_speed - max_speed_change : 0;
            }
            
            // Apply min/max limits
            uint32_t limited_speed = std::max(config.min_duty_cycle, 
                                          std::min(current_speed, config.max_duty_cycle));
            
            // Only apply PWM if not in coast mode
            if (current_direction != Direction::Coast) {
                pwm_interface->SetDutyCycle(config.pwm_ch_config.channel, limited_speed);
            }
            
            last_update_time = current_time;
        }
    }
}

uint32_t VNH5019Driver::ReadCurrentSenseFeedback() {
    // Implementation depends on your ADC interface
    // This is a placeholder
    if (adc_interface) {
        uint32_t adc_value = 0;
        adc_interface->ReadChannel(config.current_sense_channel, &adc_value);
        return adc_value;
    }
    return 0;
}

bool VNH5019Driver::CheckForFault() {
    if (!config.use_diag_pin) {
        return false;
    }
    
    Platform::GPIO::GpioPinState diag_state;
    Platform::Status status = gpio_interface->ReadPin(config.diag_port, config.diag_pin, diag_state);
    
    if (status != Platform::Status::OK) {
        return false;
    }
    
    // DIAG pin is active low (0 = fault)
    return (static_cast<uint8_t>(diag_state) == 0);
}

void VNH5019Driver::DiagPinChangeCallback(void* param) {
    if (!param) {
        return;
    }
    
    VNH5019Driver* driver = static_cast<VNH5019Driver*>(param);
    
    // Set fault flag to be handled in the Process function
    driver->fault_detected = true;
}

} // namespace Motor
} // namespace Drivers