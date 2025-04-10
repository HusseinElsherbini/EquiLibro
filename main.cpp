#include "os/FreeRTOSConfig.h"
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
#include "sbr_app_tasks.hpp"
#include "storage_manager.hpp"
#include "SEGGER_RTT.h"

#ifndef __DSB
  #define __DSB() __asm__ volatile ("dsb sy" ::: "memory")
#endif

#ifndef __ISB
  #define __ISB() __asm__ volatile ("isb sy" ::: "memory")
#endif

void disable_write_buffer_local_defs(void) {
    // 1. Define the absolute address of the ACTLR register locally
    const uint32_t actlr_address = 0xE000E008UL;

    // 2. Define the bit mask for the DISDEFWBUF bit (bit 1) locally
    const uint32_t actlr_disdefwbuf_bit = (1UL << 1);

    // 3. Create the volatile pointer locally, casting the address
    volatile uint32_t * const pACTLR = (volatile uint32_t *)actlr_address;

    // 4. Perform the read-modify-write operation
    *pACTLR |= actlr_disdefwbuf_bit;

    // 5. Memory barriers (compiler intrinsics)
    __DSB(); // Data Synchronization Barrier
    __ISB(); // Instruction Synchronization Barrier
}

/**
 * @brief Enables the Write Buffer using direct memory access.
 *        All definitions are local to the function.
 * @note Modifying ACTLR can impact performance and behavior. Use with caution.
 */
void enable_write_buffer_local_defs(void) {

    // 1. Define the absolute address of the ACTLR register locally
    const uint32_t actlr_address = 0xE000E008UL;

    // 2. Define the bit mask for the DISDEFWBUF bit (bit 1) locally
    const uint32_t actlr_disdefwbuf_bit = (1UL << 1);

    // 3. Create the volatile pointer locally, casting the address
    volatile uint32_t * const pACTLR = (volatile uint32_t *)actlr_address;

    // 4. Perform the read-modify-write operation
    *pACTLR &= ~actlr_disdefwbuf_bit;

    // 5. Memory barriers (compiler intrinsics)
    __DSB(); // Data Synchronization Barrier
    __ISB(); // Instruction Synchronization Barrier
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

    // initialize storage manager
    Middleware::Storage::StorageManager& storage = Middleware::Storage::StorageManager::GetInstance();

    status = storage.Init();

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
    
    // Configure the motors
    Drivers::Motor::VNH5019Config motor_left_config = {
        // PWM configuration for left motor
        .pwm_config = {
            .timer_instance = Platform::TIM::TimerInstance::TIM1,
            .frequency = 20000,                       // 20kHz PWM frequency
            .resolution_bits = 10,                    // 10-bit resolution (0-1023)
            .alignment = Platform::PWM::PWMAlignment::EdgeAligned,
            .mode = Platform::PWM::PWMMode::Standard,
            .use_dead_time = false,
            .dead_time_ns = 0
        },

        .pwm_ch_config = {
            .channel = Platform::PWM::PWMChannel::Channel3,
            .gpio_port = Platform::GPIO::Port::PORTA,
            .gpio_pin = 10,                                
            .gpio_af = Platform::GPIO::AlternateFunction::AF1,
        },

        // Direction control pins
        .ina_port = Platform::GPIO::Port::PORTC,
        .ina_pin = 6,                               
        .inb_port = Platform::GPIO::Port::PORTC,
        .inb_pin = 7,                               
        
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

        .pwm_config = {
            .timer_instance = Platform::TIM::TimerInstance::TIM1,
            .frequency = 20000,                       // 20kHz PWM frequency
            .resolution_bits = 10,                    // 10-bit resolution (0-1023)
            .alignment = Platform::PWM::PWMAlignment::EdgeAligned,
            .mode = Platform::PWM::PWMMode::Standard,
            .use_dead_time = false,
            .dead_time_ns = 0
        },
        .pwm_ch_config = {
            .channel = Platform::PWM::PWMChannel::Channel2,
            .gpio_port = Platform::GPIO::Port::PORTA,
            .gpio_pin = 9,                                
            .gpio_af = Platform::GPIO::AlternateFunction::AF1,
        },
        
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

        .i2c_config = {
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
        },
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
        .i2c_timeout_ms = 10,                        // 10ms timeout for I2C operations
        .operating_mode = Drivers::Sensors::MPU6050_OperatingMode::MPU6050_MODE_AUTO,
        .enable_data_ready_interrupt = true,
        .data_ready_port = Platform::GPIO::Port::PORTB,
        .data_ready_pin = 8,
        
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
    };
    
    // Create and configure the self-balancing robot application
    auto& balance_app = APP::BalanceRobotApp::GetInstance();
    // Initialize the balance robot application
    Platform::Status status = balance_app.Init(&balance_config);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    return Platform::Status::OK;
}

int main() {

    disable_write_buffer_local_defs(); // Disable write buffer for flash access
    // Initialize the system
    Platform::Status status = SystemInit();
    if (status != Platform::Status::OK) {
        // Error handling - could blink an LED or log error
        while (1) {
            // If system init fails, we can't proceed
        }
    }
    SEGGER_RTT_Init();
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

    // Create and start tasks
    balance_app.InitializeTasks();
    
    // Start the application
    balance_app.Start();  // This will call balance_controller->Start()
    
    // Start the scheduler
    vTaskStartScheduler();
    
    // Should never reach here
    while(1) {}
    
    return 0;
}

extern "C" void TIM5_IRQHandler(void) {

    static Platform::TIM::Registers* TIM5_REGS = Platform::TIM::getTimer(5);
    static Platform::GPIO::GpioInterface *test_gpio  = &Platform::GPIO::GpioInterface::GetInstance();
    // Check if update interrupt flag is set
    if (TIM5_REGS->SR & static_cast<uint32_t>(Platform::TIM::SR::UIF)) {
        // Clear the interrupt flag
        TIM5_REGS->SR &= ~static_cast<uint32_t>(Platform::TIM::SR::UIF);
        
        test_gpio->TogglePin(Platform::GPIO::Port::PORTC, 0);

    }
}
