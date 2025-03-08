/**
 * @file main.c
 * @brief Entry point for the self-balancing robot test application
 * 
 * This test application demonstrates the firmware architecture by
 * initializing the system and blinking LEDs to verify GPIO functionality.
 */

 #include "common/types.h"
 #include "common/error_codes.h"
 #include "middleware/system_services/system_init.h"
 #include "hardware_abstraction/rcc.h"
 #include "hardware_abstraction/gpio.h"
 #include "common/platform.h"
 #include "system_services/delay.h"
 #include "application/robot_logic/app_gpio_test.h"
 
 /**
  * @brief Application initialization function
  * 
  * This function initializes all components required for the application:
  * - Core system (clocks, flash, SysTick)
  * - Peripherals (GPIO)
  * - Application modules
  * 
  * @return Status_t Initialization status
  */
 static Status_t App_Init(void) {
     Status_t status;
     
     // 1. Initialize core system with custom configuration
     SystemCore_Config_t system_config = SystemCore_DefaultConfig;
     system_config.use_hse = true;  // Use external oscillator (24MHz on board)
     system_config.hse_frequency = 24000000;
     system_config.target_system_clock = 84000000;  // Target 84MHz operation
     
     status = SystemCore_Init(&system_config);
     if (status != STATUS_OK) {
         return status;
     }
     
     // 2. Initialize delay service
     Delay_Config_t delay_config = {
         .timer_instance = 0,  // Use SysTick for delay
         .timer_frequency = SystemCore_GetSystemClock()  // Use actual system clock
     };
     
     status = Delay_Init(&delay_config);
     if (status != STATUS_OK) {
         return status;
     }
     
     // 3. Initialize application module
     ApplicationModule_t *gpio_test_app = GPIOTest_GetInterface();
     if (gpio_test_app == NULL) {
         return STATUS_ERROR;
     }
     
     status = gpio_test_app->Init(NULL);
     if (status != STATUS_OK) {
         return status;
     }
     
     // Start the application module
     status = gpio_test_app->Start();
     if (status != STATUS_OK) {
         return status;
     }
     
     return STATUS_OK;
 }
 
 /**
  * @brief Main application entry point
  */
 int main(void) {
     Status_t status;
     ApplicationModule_t *gpio_test_app;
     
     // Initialize the application
     status = App_Init();
     if (status != STATUS_OK) {
         // Handle initialization error - flash error pattern
         while (1) {
             // Error state - cannot proceed
         }
     }
     
     // Get the GPIO test application module
     gpio_test_app = GPIOTest_GetInterface();
     
     // Main application loop
     while (1) {
         // Process the application
         status = gpio_test_app->Process(NULL);
         if (status != STATUS_OK) {
             // Handle processing error
         }
         
         // Add a short delay for responsiveness
         Delay_Milliseconds(100, true);
     }
     
     return 0;
 }