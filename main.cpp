// src/main.cpp

#include "application/robot_logic/app.hpp"
#include "middleware/utils/system_configurator.hpp"
#include "middleware/system_services/system_manager.hpp"
#include "middleware/system_services/system_timing.hpp"
#include "hardware_abstraction/gpio.hpp"


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
    
    // Now that hardware is initialized, configure the application
    APP::GpioToggleConfig gpio_toggle_config = {
        .gpio_port = Platform::GPIO::Port::PORTB,
        .gpio_pin = 13,
        .toggle_interval_ms = 1000,
        .start_state = true,
    };
    // get app instance
    g_gpio_toggle_app = &APP::GpioToggleApp::GetGpioToggleApp();
    
    // Initialize the application with hardware-dependent operations
    return g_gpio_toggle_app->Init(&gpio_toggle_config);
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
    
    // Main application loop
    while (1) {
        // Run application tasks
        // This could call g_gpio_toggle_app->Process() or similar
    }
    
    return 0;
}


