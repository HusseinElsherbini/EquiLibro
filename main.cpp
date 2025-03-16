// src/main.cpp

#include "application/robot_logic/app.hpp"
#include "middleware/utils/system_configurator.hpp"
#include "middleware/system_services/system_manager.hpp"
#include "middleware/system_services/system_timing.hpp"
#include "hardware_abstraction/gpio.hpp"
#include "hardware_abstraction/i2c.hpp"
#include "common/platform_i2c.hpp"
#include "hardware_abstraction/pwm.hpp"

constexpr uint8_t MPU6050_ADDRESS = 0x68;        // Default I2C address
constexpr uint8_t WHO_AM_I_REG = 0x75;           // Register that contains device ID
constexpr uint8_t PWR_MGMT_1_REG = 0x6B;         // Power management register
constexpr uint8_t MPU6050_RESET_VALUE = 0x00;    // Value to reset/wake up the MPU6050

// Completion flag for non-blocking transfers
volatile bool transfer_done = false;
volatile bool transfer_successful = false;

void pwm_motor_control_example() {
    // Get PWM interface
    auto& pwm = Platform::PWM::PWMInterface::GetInstance();
    
    // Configure PWM for motor control
    // Using a higher frequency is typical for motor control to reduce audible noise
    Platform::PWM::PWMConfig pwm_config = {
        .timer_instance = Platform::TIM::TimerInstance::TIM1,  // You can use any timer with PWM capability
        .frequency = 20000,                                    // 20kHz is still a good frequency choice
        .resolution_bits = 10,                                 // 10-bit resolution (0-1023)
        .alignment = Platform::PWM::PWMAlignment::EdgeAligned, // Edge-aligned is simpler and fine for basic PWM
        .mode = Platform::PWM::PWMMode::Standard,              // Standard PWM mode instead of complementary
        .use_dead_time = false,                                // No need for dead time with single PWM signal
        .dead_time_ns = 0                                      // No dead time
    };
    // Configure channel for motor phase B
    Platform::PWM::PWMChannelConfig motor_pwm_config = {
        .channel = Platform::PWM::PWMChannel::Channel3,        // Use any available channel
        .duty_cycle = 0,                                       // Start with 0% duty cycle (motor off)
        .polarity = Platform::PWM::PWMPolarity::ActiveHigh,    // Normal polarity (HIGH = motor on)
        .complementary_output = false,                         // No complementary output
        .gpio_port = Platform::GPIO::Port::PORTA,              // GPIO port for your PWM output
        .gpio_pin = 10,                                         // Pin number (change to your actual pin)
        .gpio_af = Platform::GPIO::AlternateFunction::AF1      // Alternate function for timer (check datasheet)
    };
    
    // Initialize PWM
    Platform::Status status = pwm.Init(&pwm_config);
    if (status != Platform::Status::OK) {
        // Handle initialization error
        return;
    }

    status = pwm.ConfigureChannel(motor_pwm_config);

    if (status != Platform::Status::OK) {
        // Handle initialization error
        return;
    }

    status = pwm.EnableChannel(motor_pwm_config.channel);

    if (status != Platform::Status::OK) {
        // Handle initialization error
        return;
    }

    pwm.Start();

    auto& timing = Middleware::SystemServices::SystemTiming::GetInstance();
    // Slowly ramp up speed from 0% to 80%
    for (uint32_t duty = 0; duty <= 8000; duty += 100) {
        pwm.SetDutyCycle(Platform::PWM::PWMChannel::Channel3, duty);
        timing.DelayMilliseconds(20);  // Smooth acceleration
    }
}
// Global instance, but not initialized yet
APP::GpioToggleApp *g_gpio_toggle_app = nullptr;  // Default constructor should be "hardware-safe"

// System initialization function
Platform::Status SystemInit() {
    // System configuration and initialization
    auto system_config = Middleware::SystemServices::SystemConfigurator()
        .withSystemClock(84000000)       // 84 MHz
        .enablePrefetch()                // Enable flash prefetch
        .enableInstructionCache()        // Enable instruction cache
        .enableDataCache()               // Enable data cache
        .withPowerMode(Platform::PWR::PowerMode::Run)
        .enableSysTick(true)
        .withSysTickInterval(1000)       // 1ms interval
        .build();

    // Initialize system services
    auto& system_manager = Middleware::SystemServices::SystemManager::GetInstance();
    Platform::Status status = system_manager.Init(&system_config);
    if(status != Platform::Status::OK) {
        return status;
    }
    
    // Initialize timing service
    auto& timing_service = Middleware::SystemServices::SystemTiming::GetInstance();
    Middleware::SystemServices::SystemTimingConfig time_service_config = {
        .instance = Platform::TIM::TimerInstance::TIM5,
    };
    status = timing_service.Init(&time_service_config);
    if(status != Platform::Status::OK) {
        return status;
    }
    return Platform::Status::OK;
}

int main() {
    // Initialize system and application
    Platform::Status status = SystemInit();
    
    if(status != Platform::Status::OK) {
        // Error handling
        while(1) {
            // Perhaps blink an error code on an LED
        }
    }

    pwm_motor_control_example();
    // Main application loop
    while (1) {

        // Run application tasks
        // This could call g_gpio_toggle_app->Process() or similar
    }
    
    return 0;
}


