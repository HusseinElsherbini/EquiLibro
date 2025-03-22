#include "robot_logic/sbr_app.hpp"
#include "middleware/utils/system_configurator.hpp"
#include "middleware/system_services/system_manager.hpp"
#include "middleware/system_services/system_timing.hpp"
#include "hardware_abstraction/gpio.hpp"
#include "hardware_abstraction/i2c.hpp"
#include "hardware_abstraction/pwm.hpp"
#include "drivers/sensors/mpu6050.hpp"
#include "drivers/actuators/vnh5019.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <memory>

// Task handles for FreeRTOS tasks
static TaskHandle_t s_balance_task_handle = nullptr;
static TaskHandle_t s_monitor_task_handle = nullptr;
static Platform::TIM::Registers* TIM5_REGS = Platform::TIM::getTimer(5);
static Platform::GPIO::GpioInterface *test_gpio  = &Platform::GPIO::GpioInterface::GetInstance();

// FreeRTOS task function for the balancing task
void BalanceTaskFunction(void* params) {
    // Get the balance robot application instance
    // The singleton is already initialized, so we don't need to pass IMU again
    APP::BalanceRobotApp& balance_app = APP::BalanceRobotApp::GetInstance();
    
    // Run the balance control loop
    while (true) {
        // Process the balance application
        balance_app.Process(nullptr);
        
        // Short delay to yield to other tasks
        vTaskDelay(1);
    }
}

// FreeRTOS task function for the monitoring task
void MonitorTaskFunction(void* params) {
    // Get the balance robot application instance
    // The singleton is already initialized, so we don't need to pass IMU again
    APP::BalanceRobotApp& balance_app = APP::BalanceRobotApp::GetInstance();
    
    // Get timing service for delays
    auto& timing = Middleware::SystemServices::SystemTiming::GetInstance();
    
    while (true) {
        // Check application state
        APP::AppState state = balance_app.GetState();
        
        // If the application is in error state, try to restart
        if (state == APP::AppState::Error) {
            // Try to re-initialize
            balance_app.Init(nullptr);
        }
        
        // If the application is idle, try to start balancing
        if (state == APP::AppState::Idle) {
            balance_app.Start();
        }
        
        // Delay for monitoring interval (100ms)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

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
        .use_rtos_timing = true,
        .enable_power_saving = false,
        .enable_interrupt = false, 
    };
    status = timing_service.Init(&time_service_config);

    if(status != Platform::Status::OK) {
        return status;
    }

    Platform::GPIO::GpioConfig test_gpio_config = {
        .port = Platform::GPIO::Port::PORTC,
        .pin = 0,
        .mode = Platform::GPIO::Mode::Output,
        .outputType = Platform::GPIO::OutputType::PushPull,
        .pull = Platform::GPIO::Pull::None
    };

    Platform::GPIO::GpioInterface *test_gpio  = &Platform::GPIO::GpioInterface::GetInstance();

    status = test_gpio->ConfigurePin(test_gpio_config);
    if(status != Platform::Status::OK) {
        return status;
    }
    //TODO: DEBUG CODE, ERASE
    return Platform::Status::OK;
}

// Initialize the self-balancing robot application
Platform::Status InitBalanceRobotApp() {

    // Initialize I2C interface for IMU
    auto& i2c_interface = Platform::I2C::I2CInterface::GetInstance(Platform::I2C::I2CInstance::I2C1);
    Platform::I2C::I2CConfig i2c_config = {
        .i2c_instance = Platform::I2C::I2CInstance::I2C1,
        .mode = Platform::I2C::Mode::Master,
        .speed = Platform::I2C::Speed::Fast,       // 400kHz for MPU6050
        .addressing_mode = Platform::I2C::AddrMode::Addr7Bit,
        .own_address = 0x00,                      // Not used in master mode
        .enable_dma = false,                      
        .enable_interrupts = true,
        .analog_filter = true,
        .digital_filter = 0,
        .stretch_clock = true
    };

    Platform::Status status  = i2c_interface.Init(&i2c_config);
    if(status != Platform::Status::OK) {
        return status;
    }

    // Initialize PWM interfaces for motors
    auto& pwm_interface = Platform::PWM::PWMInterface::GetInstance();
    Platform::PWM::PWMConfig pwm_config = {
        .timer_instance = Platform::TIM::TimerInstance::TIM1,
        .frequency = 20000,                       // 20kHz PWM frequency
        .resolution_bits = 10,                    // 10-bit resolution (0-1023)
        .alignment = Platform::PWM::PWMAlignment::EdgeAligned,
        .mode = Platform::PWM::PWMMode::Standard,
        .use_dead_time = false,
        .dead_time_ns = 0
    };
    status = pwm_interface.Init(&pwm_config);
    if(status != Platform::Status::OK) {
        return status;
    }

    // Create and configure the self-balancing robot application
    auto& balance_app = APP::BalanceRobotApp::GetInstance();
    
    // Configure the motors
    Drivers::Motor::VNH5019Config motor_left_config = {
        // PWM configuration for left motor
        .pwm_timer = Platform::TIM::TimerInstance::TIM1,
        .pwm_frequency = 20000,                      // 20kHz
        .pwm_channel = Platform::PWM::PWMChannel::Channel3,
        .pwm_port = Platform::GPIO::Port::PORTA,
        .pwm_pin = 10,                                // PA8, matching the schematic
        .pwm_af = Platform::GPIO::AlternateFunction::AF1,
        
        // Direction control pins
        .ina_port = Platform::GPIO::Port::PORTC,
        .ina_pin = 6,                               // PB14, matching the schematic
        .inb_port = Platform::GPIO::Port::PORTC,
        .inb_pin = 7,                               // PB15, matching the schematic
        
        // Enable pin - not needed for VNH5019 (tied to VCC)
        .use_enable_pin = false,
        
        // Current sensing (optional)
        .use_current_sensing = true,
        .current_sense_channel = Platform::ADC::AdcChannel::Channel9,
        .current_sense_ratio = 0.14f,                // VNH5019 current sense is 0.14 V/A
        
        // No fault detection for simplicity
        .use_diag_pin = false,
        
        // Speed control parameters
        .min_duty_cycle = 0,
        .max_duty_cycle = 9000,                      // 90% max duty cycle for safety
        .acceleration_rate = 2000                     // Moderate acceleration
    };

    // Configure the motors
    Drivers::Motor::VNH5019Config motor_right_config = {
        // PWM configuration for left motor
        .pwm_timer = Platform::TIM::TimerInstance::TIM1,
        .pwm_frequency = 20000,                      // 20kHz
        .pwm_channel = Platform::PWM::PWMChannel::Channel2,
        .pwm_port = Platform::GPIO::Port::PORTA,
        .pwm_pin = 9,                                // PA9, matching the schematic
        .pwm_af = Platform::GPIO::AlternateFunction::AF1,
        
        // Direction control pins
        .ina_port = Platform::GPIO::Port::PORTA,
        .ina_pin = 8,                              
        .inb_port = Platform::GPIO::Port::PORTB,
        .inb_pin = 2,                              
        
        // Enable pin - not needed for VNH5019 (tied to VCC)
        .use_enable_pin = false,
        
        // Current sensing (optional)
        .use_current_sensing = true,
        .current_sense_channel = Platform::ADC::AdcChannel::Channel4,
        .current_sense_ratio = 0.14f,                // VNH5019 current sense is 0.14 V/A
        
        // No fault detection for simplicity
        .use_diag_pin = false,
        
        // Speed control parameters
        .min_duty_cycle = 0,
        .max_duty_cycle = 9000,                      // 90% max duty cycle for safety
        .acceleration_rate = 2000                     // Moderate acceleration
    }; 
    
    // Configure the IMU (MPU6050)
    Drivers::Sensors::MPU6050Config imu_config = { 
        .i2c_interface = &i2c_interface,
        .device_address = 0x68,                      // Default address when AD0 is GND
        .gyro_range = 0,                             // ±250 deg/s - suitable for balancing
        .accel_range = 0,                            // ±4g - suitable for balancing
        .dlpf_mode = 3,                              // 44Hz bandwidth - good balance of response and noise
        .sample_rate_div = 4,                        // 200Hz sample rate (1kHz / (1 + 4))
        .interrupt_enabled = true,                   // Use data ready interrupt for efficiency
        .clock_source = 1,                           // PLL with X-axis gyro as reference
        .use_fifo = false,                           // No FIFO for simplicity
        .enable_motion_detect = false,               // No motion detection
        .motion_threshold = 0,
        .i2c_timeout_ms = 10                         // 10ms timeout for I2C operations
    };
    
    // Configure the PID controller
    APP::PIDConfig pid_config = {
        .kp = 25.0f,                                 // Proportional gain
        .ki = 2.0f,                                  // Integral gain
        .kd = 0.5f,                                  // Derivative gain
        .setpoint = 0.0f,                            // Target angle (upright)
        .output_limit = 9000.0f,                     // Max motor output
        .integral_limit = 200.0f                     // Integral windup limit
    };
    
    // Create the balance robot configuration
    APP::BalanceRobotConfig balance_config = {
        .imu_config = imu_config,
        .motor_left_config = motor_left_config,
        .motor_right_config = motor_right_config,
        .pid_config = pid_config,
        .control_loop_interval_ms = 5,               // 200Hz control loop
        .enable_debug_output = false
    };
    
    // Initialize the balance robot application
    status = balance_app.Init(&balance_config);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    return Platform::Status::OK;
}

// Fall detection callback
void FallDetectedCallback(void* param) {
    // Get the balance app
    // The singleton is already initialized, so we don't need to pass IMU again
    APP::BalanceRobotApp& balance_app = APP::BalanceRobotApp::GetInstance();
    
    // Stop the robot immediately
    balance_app.Stop();
    
    // Could add additional safety measures here if needed
}

int main() {
    // Initialize the system
    Platform::Status status = SystemInit();
    if (status != Platform::Status::OK) {
        // Error handling - could blink an LED or log error
        while (1) {
            // If system init fails, we can't proceed
        }
    }
    // Initialize the self-balancing robot application
    status = InitBalanceRobotApp();
    if (status != Platform::Status::OK) {
        // Error handling - could blink an LED or log error
        while (1) {
            // If app init fails, we can't proceed
        }
    }
    
    // Get the balance robot application instance
    // The singleton is already initialized in InitBalanceRobotApp
    APP::BalanceRobotApp& balance_app = APP::BalanceRobotApp::GetInstance();
    
    // Register fall detection callback
    balance_app.RegisterCallback(APP::EVENT_FALL_DETECTED, FallDetectedCallback, nullptr);
    
    // Create FreeRTOS tasks
    // Balance task - higher priority as it needs to run at consistent intervals
    BaseType_t result = xTaskCreate(
        BalanceTaskFunction,           // Task function
        "BalanceTask",                 // Task name
        256,                           // Stack size (words)
        nullptr,                       // Parameters
        tskIDLE_PRIORITY + 3,          // Priority
        &s_balance_task_handle         // Task handle
    );
    
    if (result != pdPASS) {
        // Task creation failed
        while (1) {
            // Can't continue without the balance task
        }
    }
    
    // Monitor task - lower priority, mostly for supervision
    result = xTaskCreate(
        MonitorTaskFunction,           // Task function
        "MonitorTask",                 // Task name
        128,                           // Stack size (words)
        nullptr,                       // Parameters
        tskIDLE_PRIORITY + 1,          // Priority
        &s_monitor_task_handle         // Task handle
    );
    
    if (result != pdPASS) {
        // Task creation failed - not critical, could continue without monitor
    }
    
    // Start the robot initially
    balance_app.Start();
    
    // Start the FreeRTOS scheduler
    vTaskStartScheduler();
    
    // We should never get here as the scheduler should take over
    while (1) {
        // Fallback loop in case scheduler fails
    }
    
    return 0;
}

extern "C" void TIM5_IRQHandler(void) {
    // Check if update interrupt flag is set
    if (TIM5_REGS->SR & static_cast<uint32_t>(Platform::TIM::SR::UIF)) {
        // Clear the interrupt flag
        TIM5_REGS->SR &= ~static_cast<uint32_t>(Platform::TIM::SR::UIF);
        
        test_gpio->TogglePin(Platform::GPIO::Port::PORTC, 0);

    }
}