/**
 * @file app_gpio_test.h
 * @brief GPIO testing application for self-balancing robot
 * 
 * This file contains declarations for the GPIO testing application module.
 */

 #ifndef APP_GPIO_TEST_H
 #define APP_GPIO_TEST_H
 
 #include "common/types.h"
 #include "common/error_codes.h"
 #include "application/app_module.h"
 
 // Application-specific command IDs
 // (in addition to the common ones defined in app_module.h)
 #define APP_CMD_SET_BLINK_DELAY         0x3100
 
 /**
  * @brief Get the GPIO Test application module interface
  * 
  * @return ApplicationModule_t* Pointer to the GPIO test application module interface
  */
 ApplicationModule_t* GPIOTest_GetInterface(void);
 
 #endif /* APP_GPIO_TEST_H */