#pragma once

#include "hardware_abstraction/gpio.hpp"
#include "hardware_abstraction/pwm.hpp"
#include "hardware_abstraction/adc.hpp"
#include "middleware/system_services/system_timing.hpp"

namespace Drivers {
namespace Motor {

// Direction of motor rotation
enum class Direction {
    Forward,
    Reverse,
    Brake,
    Coast
};

// VNH5019 Motor Driver configuration
struct VNH5019Config {
    
    // PWM configuration
    Platform::PWM::PWMConfig pwm_config;   
    Platform::PWM::PWMChannelConfig pwm_ch_config;   
   
    // Direction control pins
    Platform::GPIO::Port ina_port;                // GPIO port for INA pin
    uint8_t ina_pin;                              // GPIO pin for INA
    Platform::GPIO::Port inb_port;                // GPIO port for INB pin
    uint8_t inb_pin;                              // GPIO pin for INB
    
    // Optional enable pin (can be used if your VNH5019 requires it)
    bool use_enable_pin;                          // Whether to use a dedicated enable pin
    Platform::GPIO::Port enable_port;             // GPIO port for enable pin
    uint8_t enable_pin;                           // GPIO pin for enable
    
    // Current sensing (optional)
    bool use_current_sensing;                     // Whether to use current sensing
    Platform::ADC::AdcChannel current_sense_channel; // ADC channel for current sensing
    float current_sense_ratio;                    // Conversion ratio mV/A
    
    // Fault detection (optional)
    bool use_diag_pin;                            // Whether to use diagnostic pins
    Platform::GPIO::Port diag_port;               // GPIO port for diagnostic pin
    uint8_t diag_pin;                             // GPIO pin for diagnostic

    // Speed control parameters
    uint32_t min_duty_cycle;                      // Minimum duty cycle (0-10000)
    uint32_t max_duty_cycle;                      // Maximum duty cycle (0-10000)
    uint32_t acceleration_rate;                   // Rate of change for ramping (0-10000 per second)
};

// VNH5019 Motor Driver class
class VNH5019Driver {
private:
    // State tracking
    bool initialized;
    Direction current_direction;
    uint32_t target_speed;                        // Target speed (0-10000)
    uint32_t current_speed;                       // Current speed (0-10000)
    bool fault_detected;
    
    // Configuration
    VNH5019Config config;
    
    // Interfaces
    Platform::PWM::PWMInterface* pwm_interface;
    Platform::GPIO::GpioInterface* gpio_interface;
    Platform::ADC::AdcInterface* adc_interface;   // For current sensing
    
    // Helper methods
    void SetDirectionPins(Direction direction);
    uint32_t ReadCurrentSenseFeedback();
    bool CheckForFault();
    
    // Callback function for fault detection
    static void DiagPinChangeCallback(void* param);
    
public:
    // Constructor
    VNH5019Driver();
    
    // Destructor
    ~VNH5019Driver();
    
    // Initialize motor driver
    Platform::Status Init(const VNH5019Config* config);
    
    // Enable/disable motor driver
    Platform::Status Enable();
    Platform::Status Disable();
    
    // Set motor speed and direction
    Platform::Status SetSpeed(int32_t speed);     // -10000 to +10000, sign indicates direction
    Platform::Status SetSpeedAndDirection(uint32_t speed, Direction direction);
    
    // Get current motor speed
    int32_t GetSpeed() const;
    
    // Get current motor direction
    Direction GetDirection() const;
    
    // Stop motor (brake or coast)
    Platform::Status Stop(bool brake = true);
    
    // Check if fault has been detected
    bool IsFaultDetected() const;
    
    // Clear fault condition
    Platform::Status ClearFault();
    
    // Read motor current (if current sensing is available)
    float ReadMotorCurrent();
    
    // Process function for smooth acceleration/deceleration
    // Call this regularly in your main loop or from a timer
    void Process();
};

} // namespace Motor
} // namespace Drivers