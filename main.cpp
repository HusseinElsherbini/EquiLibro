// src/main.cpp

#include "application/robot_logic/app.hpp"
#include "middleware/utils/system_configurator.hpp"
#include "middleware/system_services/system_manager.hpp"
#include "middleware/system_services/system_timing.hpp"
#include "hardware_abstraction/gpio.hpp"


// Global instances
static APP::GpioToggleApp gpio_toggle_app;

int main() {
    // Configure the system using SystemConfigurator
    auto system_config = Middleware::SystemServices::SystemConfigurator()
        .withSystemClock(84000000)       // 84 MHz
        .enablePrefetch()                // Enable flash prefetch
        .enableInstructionCache()        // Enable instruction cache
        .enableDataCache()               // Enable data cache
        .withPowerMode(Platform::PWR::PowerMode::Run)
        .enableSysTick(true)
        .withSysTickInterval(1000)       // 1ms interval
        .build();
    
    // Initialize system with the configuration
    auto& system_manager = Middleware::SystemServices::SystemManager::GetInstance();
    
    Platform::Status status = system_manager.Init(&system_config);
    
    if(status != Platform::Status::OK) {

        // failed intializing system manager
        while(1);
    }

    // Configure PB13 as toggle led
    APP::GpioToggleConfig gpio_toggle_config = {

        .gpio_port = Platform::GPIO::Port::PORTB,
        .gpio_pin = 13,
        .toggle_interval_ms = 1000,
        .start_state = true,
    };
    // Get system services
    auto& timing_service = Middleware::SystemServices::SystemTiming::GetInstance();

    Middleware::SystemServices::SystemTimingConfig time_service_config = {

        .instance = Platform::TIM::TimerInstance::TIM5,
        .timer_input_clk_freq = 84000000,
    };

    status = timing_service.Init(&time_service_config); 
    if(status != Platform::Status::OK) {

        // failed intializing system manager
        while(1);
    }

    // initialize application 
    gpio_toggle_app.Init(&gpio_toggle_config);

    // Should never reach here
    while (1) {
        // Infinite loop
    }
    
    return 0;
}



