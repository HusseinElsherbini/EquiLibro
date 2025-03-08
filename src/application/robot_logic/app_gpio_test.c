/**
 * @file app_gpio_test.c
 * @brief GPIO testing application implementation
 * 
 * This file implements a test application for verifying GPIO functionality
 * by blinking LEDs in various patterns.
 */

 #include "application/robot_logic/app_gpio_test.h"
 #include "hardware_abstraction/gpio.h"
 #include "hardware_abstraction/rcc.h"
 #include "common/platform.h"
 #include "system_services/delay.h"
 
 // Application state structure
 typedef struct {
     AppState_t state;
     HW_Interface_t *gpio_interface;
     HW_Interface_t *rcc_interface;
     
     // GPIO configurations for LEDs (according to schematic)
     GPIO_Config_t mcu_led;    // PB13
     GPIO_Config_t rx_led;     // PA2
     GPIO_Config_t tx_led;     // PA3
     
     // Test pattern configuration
     uint32_t pattern_index;
     uint32_t pattern_count;
     uint32_t cycle_count;
     uint32_t blink_delay_ms;
 } GPIOTest_State_t;
 
 // Static instance of the application state
 static GPIOTest_State_t gpio_test_state;
 
 // Static instance of the application module interface
 static ApplicationModule_t gpio_test_module;
 
 // Forward declarations for interface functions
 static Status_t GPIOTest_Init(void *config);
 static Status_t GPIOTest_Start(void);
 static Status_t GPIOTest_Stop(void);
 static Status_t GPIOTest_Process(void *params);
 static Status_t GPIOTest_HandleCommand(uint32_t cmd_id, void *params);
 static Status_t GPIOTest_GetStatus(void *status, uint32_t *size);
 static Status_t GPIOTest_RegisterCallback(uint32_t event, void (*callback)(void *param), void *param);
 
 /**
  * @brief Initialize the LED pins for testing
  * 
  * @return Status_t Initialization status
  */
 static Status_t GPIOTest_InitializeGPIO(void) {
     Status_t status;
     
     // Enable GPIO clocks needed for LEDs
     RCC_Peripheral_t periph = RCC_PERIPH_GPIOA;  // Both LEDs are on GPIOA
     status = gpio_test_state.rcc_interface->Control(
         gpio_test_state.rcc_interface->state,
         RCC_CTRL_ENABLE_PERIPHERAL_CLOCK,
         &periph
     );
     
     if (status != STATUS_OK) {
         return status;
     }
     
     // Configure MCU LED (PA15 according to schematic)
     gpio_test_state.mcu_led.port = GPIOB;
     gpio_test_state.mcu_led.pin = 13;
     gpio_test_state.mcu_led.mode = GPIO_MODE_OUTPUT;
     gpio_test_state.mcu_led.outputType = GPIO_OUTPUT_PUSH_PULL;
     gpio_test_state.mcu_led.pull = GPIO_PULL_NONE;
     gpio_test_state.mcu_led.speed = GPIO_SPEED_LOW;
     
     status = gpio_test_state.gpio_interface->Control(
         gpio_test_state.gpio_interface->state,
         GPIO_CTRL_CONFIG_PIN,
         &gpio_test_state.mcu_led
     );
     
     if (status != STATUS_OK) {
         return status;
     }
     
     // Configure RX LED (PA4 according to schematic)
     gpio_test_state.rx_led.port = GPIOA;
     gpio_test_state.rx_led.pin = 2;
     gpio_test_state.rx_led.mode = GPIO_MODE_OUTPUT;
     gpio_test_state.rx_led.outputType = GPIO_OUTPUT_PUSH_PULL;
     gpio_test_state.rx_led.pull = GPIO_PULL_NONE;
     gpio_test_state.rx_led.speed = GPIO_SPEED_LOW;
     
     status = gpio_test_state.gpio_interface->Control(
         gpio_test_state.gpio_interface->state,
         GPIO_CTRL_CONFIG_PIN,
         &gpio_test_state.rx_led
     );
     
     if (status != STATUS_OK) {
         return status;
     }
     
     // Configure TX LED (PA3 according to schematic)
     gpio_test_state.tx_led.port = GPIOA;
     gpio_test_state.tx_led.pin = 3;
     gpio_test_state.tx_led.mode = GPIO_MODE_OUTPUT;
     gpio_test_state.tx_led.outputType = GPIO_OUTPUT_PUSH_PULL;
     gpio_test_state.tx_led.pull = GPIO_PULL_NONE;
     gpio_test_state.tx_led.speed = GPIO_SPEED_LOW;
     
     status = gpio_test_state.gpio_interface->Control(
         gpio_test_state.gpio_interface->state,
         GPIO_CTRL_CONFIG_PIN,
         &gpio_test_state.tx_led
     );
     
     if (status != STATUS_OK) {
         return status;
     }
     
     // Ensure all LEDs are off initially
     gpio_test_state.gpio_interface->Control(
         gpio_test_state.gpio_interface->state, 
         GPIO_CTRL_RESET_PIN, 
         &gpio_test_state.mcu_led
     );
     
     gpio_test_state.gpio_interface->Control(
         gpio_test_state.gpio_interface->state, 
         GPIO_CTRL_RESET_PIN, 
         &gpio_test_state.rx_led
     );
     
     gpio_test_state.gpio_interface->Control(
         gpio_test_state.gpio_interface->state, 
         GPIO_CTRL_RESET_PIN, 
         &gpio_test_state.tx_led
     );
     
     return STATUS_OK;
 }
 
 /**
  * @brief Initialize the GPIO Test application
  * 
  * @param config Configuration parameters (unused)
  * @return Status_t Initialization status
  */
 static Status_t GPIOTest_Init(void *config) {
     Status_t status;
     
     // Get interfaces needed by this module
     gpio_test_state.gpio_interface = GPIO_GetInterface();
     if (gpio_test_state.gpio_interface == NULL) {
         return STATUS_ERROR;
     }
     
     gpio_test_state.rcc_interface = RCC_GetInterface();
     if (gpio_test_state.rcc_interface == NULL) {
         return STATUS_ERROR;
     }
     
     // Initialize the GPIO interface
     status = gpio_test_state.gpio_interface->Init(gpio_test_state.gpio_interface->state, NULL);
     if (status != STATUS_OK) {
         return status;
     }
     
     // Initialize the LED pins
     status = GPIOTest_InitializeGPIO();
     if (status != STATUS_OK) {
         return status;
     }
     
     // Initialize test pattern state
     gpio_test_state.pattern_index = 0;
     gpio_test_state.pattern_count = 4;  // Number of different LED patterns
     gpio_test_state.cycle_count = 0;
     gpio_test_state.blink_delay_ms = 200;  // Default blink delay
     
     // Set initial state
     gpio_test_state.state = APP_STATE_INITIALIZING;
     
     return STATUS_OK;
 }
 
 /**
  * @brief Start the GPIO Test application
  * 
  * @return Status_t Start status
  */
 static Status_t GPIOTest_Start(void) {
     gpio_test_state.state = APP_STATE_RUNNING;
     return STATUS_OK;
 }
 
 /**
  * @brief Stop the GPIO Test application
  * 
  * @return Status_t Stop status
  */
 static Status_t GPIOTest_Stop(void) {
     // Turn off all LEDs
     gpio_test_state.gpio_interface->Control(
         gpio_test_state.gpio_interface->state, 
         GPIO_CTRL_RESET_PIN, 
         &gpio_test_state.mcu_led
     );
     
     gpio_test_state.gpio_interface->Control(
         gpio_test_state.gpio_interface->state, 
         GPIO_CTRL_RESET_PIN, 
         &gpio_test_state.rx_led
     );
     
     gpio_test_state.gpio_interface->Control(
         gpio_test_state.gpio_interface->state, 
         GPIO_CTRL_RESET_PIN, 
         &gpio_test_state.tx_led
     );
     
     gpio_test_state.state = APP_STATE_IDLE;
     return STATUS_OK;
 }
 
 /**
  * @brief Process function for GPIO Test application
  * 
  * This function is called periodically to update LED patterns
  * 
  * @param params Processing parameters (unused)
  * @return Status_t Processing status
  */
 static Status_t GPIOTest_Process(void *params) {
     Status_t status = STATUS_OK;
     
     if (gpio_test_state.state != APP_STATE_RUNNING) {
         return STATUS_OK;  // Nothing to do if not running
     }
     
     // Process different LED patterns based on current pattern index
     switch (gpio_test_state.pattern_index) {
         case 0:
             // Pattern 0: Blink MCU LED
             status = gpio_test_state.gpio_interface->Control(
                 gpio_test_state.gpio_interface->state, 
                 GPIO_CTRL_TOGGLE_PIN, 
                 &gpio_test_state.mcu_led
             );
             
             // Reset other LEDs
             gpio_test_state.gpio_interface->Control(
                 gpio_test_state.gpio_interface->state, 
                 GPIO_CTRL_RESET_PIN, 
                 &gpio_test_state.rx_led
             );
             
             gpio_test_state.gpio_interface->Control(
                 gpio_test_state.gpio_interface->state, 
                 GPIO_CTRL_RESET_PIN, 
                 &gpio_test_state.tx_led
             );
             break;
             
         case 1:
             // Pattern 1: Blink RX LED
             status = gpio_test_state.gpio_interface->Control(
                 gpio_test_state.gpio_interface->state, 
                 GPIO_CTRL_TOGGLE_PIN, 
                 &gpio_test_state.rx_led
             );
             
             // Reset other LEDs
             gpio_test_state.gpio_interface->Control(
                 gpio_test_state.gpio_interface->state, 
                 GPIO_CTRL_RESET_PIN, 
                 &gpio_test_state.mcu_led
             );
             
             gpio_test_state.gpio_interface->Control(
                 gpio_test_state.gpio_interface->state, 
                 GPIO_CTRL_RESET_PIN, 
                 &gpio_test_state.tx_led
             );
             break;
             
         case 2:
             // Pattern 2: Blink TX LED
             status = gpio_test_state.gpio_interface->Control(
                 gpio_test_state.gpio_interface->state, 
                 GPIO_CTRL_TOGGLE_PIN, 
                 &gpio_test_state.tx_led
             );
             
             // Reset other LEDs
             gpio_test_state.gpio_interface->Control(
                 gpio_test_state.gpio_interface->state, 
                 GPIO_CTRL_RESET_PIN, 
                 &gpio_test_state.mcu_led
             );
             
             gpio_test_state.gpio_interface->Control(
                 gpio_test_state.gpio_interface->state, 
                 GPIO_CTRL_RESET_PIN, 
                 &gpio_test_state.rx_led
             );
             break;
             
         case 3:
             // Pattern 3: Blink all LEDs together
             status = gpio_test_state.gpio_interface->Control(
                 gpio_test_state.gpio_interface->state, 
                 GPIO_CTRL_TOGGLE_PIN, 
                 &gpio_test_state.mcu_led
             );
             
             gpio_test_state.gpio_interface->Control(
                 gpio_test_state.gpio_interface->state, 
                 GPIO_CTRL_TOGGLE_PIN, 
                 &gpio_test_state.rx_led
             );
             
             gpio_test_state.gpio_interface->Control(
                 gpio_test_state.gpio_interface->state, 
                 GPIO_CTRL_TOGGLE_PIN, 
                 &gpio_test_state.tx_led
             );
             break;
             
         default:
             // Reset pattern index on invalid value
             gpio_test_state.pattern_index = 0;
             break;
     }
     
     // Update cycle and pattern counters
     gpio_test_state.cycle_count++;
     
     // Change pattern every 10 cycles
     if (gpio_test_state.cycle_count >= 10) {
         gpio_test_state.cycle_count = 0;
         gpio_test_state.pattern_index = (gpio_test_state.pattern_index + 1) % gpio_test_state.pattern_count;
     }
     
     return status;
 }
 
 /**
  * @brief Handle application commands
  * 
  * @param cmd_id Command identifier
  * @param params Command parameters
  * @return Status_t Command handling status
  */
 static Status_t GPIOTest_HandleCommand(uint32_t cmd_id, void *params) {
     switch (cmd_id) {
         case APP_CMD_SET_MODE:
             if (params == NULL) {
                 return STATUS_INVALID_PARAM;
             }
             
             uint32_t pattern = *((uint32_t*)params);
             if (pattern < gpio_test_state.pattern_count) {
                 gpio_test_state.pattern_index = pattern;
                 gpio_test_state.cycle_count = 0;
                 return STATUS_OK;
             }
             return STATUS_INVALID_PARAM;
             
         case APP_CMD_GET_MODE:
             if (params == NULL) {
                 return STATUS_INVALID_PARAM;
             }
             
             *((uint32_t*)params) = gpio_test_state.pattern_index;
             return STATUS_OK;
         
         case APP_CMD_SET_PARAMETER: {
             // For setting blink delay
             typedef struct {
                 uint32_t param_id;
                 uint32_t value;
             } Parameter_t;
             
             Parameter_t *param = (Parameter_t*)params;
             if (param == NULL) {
                 return STATUS_INVALID_PARAM;
             }
             
             if (param->param_id == 1) { // 1 = blink delay
                 gpio_test_state.blink_delay_ms = param->value;
                 return STATUS_OK;
             }
             
             return STATUS_INVALID_PARAM;
         }
             
         default:
             return STATUS_NOT_SUPPORTED;
     }
 }
 
 /**
  * @brief Get application status
  * 
  * @param status Pointer to store status information
  * @param size Size of the status buffer
  * @return Status_t Status retrieval result
  */
 static Status_t GPIOTest_GetStatus(void *status, uint32_t *size) {
     if (status == NULL || size == NULL || *size < sizeof(AppState_t)) {
         return STATUS_INVALID_PARAM;
     }
     
     *((AppState_t*)status) = gpio_test_state.state;
     *size = sizeof(AppState_t);
     
     return STATUS_OK;
 }
 
 /**
  * @brief Register callback for application events
  * 
  * @param event Event identifier
  * @param callback Callback function pointer
  * @param param Parameter to pass to the callback
  * @return Status_t Registration status
  */
 static Status_t GPIOTest_RegisterCallback(uint32_t event, void (*callback)(void *param), void *param) {
     // No callbacks implemented for this simple test
     return STATUS_NOT_SUPPORTED;
 }
 
 /**
  * @brief Get the GPIO Test application module interface
  * 
  * @return ApplicationModule_t* Pointer to the application module interface
  */
 ApplicationModule_t* GPIOTest_GetInterface(void) {
     // Initialize interface function pointers
     gpio_test_module.state = gpio_test_state.state;
     gpio_test_module.app_state = &gpio_test_state;
     gpio_test_module.Init = GPIOTest_Init;
     gpio_test_module.Start = GPIOTest_Start;
     gpio_test_module.Stop = GPIOTest_Stop;
     gpio_test_module.Process = GPIOTest_Process;
     gpio_test_module.HandleCommand = GPIOTest_HandleCommand;
     gpio_test_module.GetStatus = GPIOTest_GetStatus;
     gpio_test_module.RegisterCallback = GPIOTest_RegisterCallback;
     
     return &gpio_test_module;
 }