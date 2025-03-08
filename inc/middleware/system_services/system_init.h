/**
 * @file system_init.h
 * @brief System initialization services
 * 
 * This file provides functions for initializing core system components
 * including clocks, power management, and core peripherals.
 */

 #ifndef SYSTEM_INIT_H
 #define SYSTEM_INIT_H
 
 #include "common/types.h"
 #include "common/error_codes.h"
 
 /**
  * @brief System configuration structure
  */
 typedef struct {
     uint32_t target_system_clock;   // Desired system clock frequency in Hz
     bool use_hse;                   // Whether to use external oscillator
     uint32_t hse_frequency;         // External oscillator frequency (if used)
     bool enable_pll;                // Whether to use PLL
     uint8_t flash_latency;          // Optional flash latency override (0xFF for auto)
     
     // Bus dividers
     uint32_t ahb_divider;           // AHB clock divider (1, 2, 4, 8, etc.)
     uint32_t apb1_divider;          // APB1 clock divider (1, 2, 4, 8, etc.)
     uint32_t apb2_divider;          // APB2 clock divider (1, 2, 4, 8, etc.)
     
     // SysTick configuration
     bool enable_systick;            // Whether to enable SysTick timer
     uint32_t systick_interval_ms;   // SysTick interrupt interval in milliseconds
 } SystemCore_Config_t;
 
 /**
  * @brief Default system configuration
  * 
  * This provides reasonable default values for system initialization.
  * The default is 84MHz system clock using PLL and HSI (internal oscillator).
  */
 extern const SystemCore_Config_t SystemCore_DefaultConfig;
 
 /**
  * @brief Initialize the core system components
  * 
  * This function initializes only essential core system components:
  * - System clocks
  * - Flash timing
  * - SysTick timer (if enabled in config)
  * 
  * It does NOT enable peripheral clocks - that should be done by
  * peripheral drivers when they are initialized.
  * 
  * @param config Pointer to core system configuration
  * @return Status_t Initialization status
  */
 Status_t SystemCore_Init(const SystemCore_Config_t *config);
 
 /**
  * @brief De-initialize the core system components
  * 
  * This function resets core system components to their default state.
  * Note that this will reset the entire system, so it should be used
  * with caution.
  * 
  * @return Status_t De-initialization status
  */
 Status_t SystemCore_DeInit(void);
 
 /**
  * @brief Get the current system clock frequency
  * 
  * @return uint32_t Current system clock frequency in Hz
  */
 uint32_t SystemCore_GetSystemClock(void);
 
 /**
  * @brief Get the current AHB clock frequency
  * 
  * @return uint32_t Current AHB clock frequency in Hz
  */
 uint32_t SystemCore_GetAHBClock(void);
 
 /**
  * @brief Get the current APB1 clock frequency
  * 
  * @return uint32_t Current APB1 clock frequency in Hz
  */
 uint32_t SystemCore_GetAPB1Clock(void);
 
 /**
  * @brief Get the current APB2 clock frequency
  * 
  * @return uint32_t Current APB2 clock frequency in Hz
  */
 uint32_t SystemCore_GetAPB2Clock(void);
 
 /**
  * @brief Check if system is using HSE as clock source
  * 
  * @return bool true if HSE is being used, false otherwise
  */
 bool SystemCore_IsUsingHSE(void);
 
 /**
  * @brief Check if system is using PLL as clock source
  * 
  * @return bool true if PLL is being used, false otherwise
  */
 bool SystemCore_IsUsingPLL(void);
 
 /**
  * @brief Configure flash latency based on system clock frequency
  * 
  * This function sets the appropriate flash wait states based on
  * the current system clock frequency.
  * 
  * @param system_clock_freq System clock frequency in Hz
  * @param override_latency Flash latency override value (0xFF for auto)
  * @return Status_t Configuration status
  */
 Status_t SystemCore_ConfigureFlashLatency(uint32_t system_clock_freq, uint8_t override_latency);
 
 /**
  * @brief Configure SysTick timer
  * 
  * This function configures the SysTick timer for system timing.
  * 
  * @param interval_ms SysTick interrupt interval in milliseconds
  * @return Status_t Configuration status
  */
 Status_t SystemCore_ConfigureSysTick(uint32_t interval_ms);
 
 #endif /* SYSTEM_INIT_H */