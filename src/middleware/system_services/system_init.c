/**
 * @file system_init.c
 * @brief System initialization implementation
 * 
 * This file implements the core system initialization services,
 * providing functions to set up the fundamental system components
 * like clocks, flash timing, and the SysTick timer.
 */

 #include "middleware/system_services/system_init.h"
 #include "hardware_abstraction/rcc.h"
 #include "hardware_abstraction/systick.h"
 #include "platform.h"
 
 // System state tracking
 typedef struct {
     bool initialized;
     uint32_t system_clock_freq;
     uint32_t ahb_clock_freq;
     uint32_t apb1_clock_freq;
     uint32_t apb2_clock_freq;
     bool using_hse;
     bool using_pll;
     uint8_t flash_latency;
 } SystemCore_State_t;
 
 // Static instance of system state
 static SystemCore_State_t system_state = {false};
 
 // Default system configuration
 const SystemCore_Config_t SystemCore_DefaultConfig = {
     .target_system_clock = 84000000,  // 84 MHz
     .use_hse = false,                 // Use internal oscillator by default
     .hse_frequency = 24000000,        // 24 MHz HSE (if used)
     .enable_pll = true,               // Use PLL
     .flash_latency = 0xFF,            // Auto configuration
     .ahb_divider = 1,                 // AHB = SYSCLK
     .apb1_divider = 2,                // APB1 = HCLK/2 (42 MHz max)
     .apb2_divider = 1,                // APB2 = HCLK
     .enable_systick = true,           // Enable SysTick
     .systick_interval_ms = 1          // 1ms SysTick interval
 };
 
 /**
  * @brief Initialize the core system components
  * 
  * @param config Pointer to core system configuration
  * @return Status_t Initialization status
  */
 Status_t SystemCore_Init(const SystemCore_Config_t *config) {
     Status_t status;
     HW_Interface_t *rcc_interface;
     
     // Use default configuration if none provided
     SystemCore_Config_t local_config;
     if (config == NULL) {
         local_config = SystemCore_DefaultConfig;
     } else {
         local_config = *config;
     }
     
     // Get the RCC interface
     rcc_interface = RCC_GetInterface();
     if (rcc_interface == NULL) {
         return STATUS_ERROR;
     }
     
     // Configure RCC for system clock
     RCC_Config_t rcc_config = {0};
     
     // Set clock source
     if (local_config.use_hse) {
         rcc_config.hse_frequency = local_config.hse_frequency;
     } else {
         rcc_config.hse_frequency = 0; // Use HSI
     }
     
     // Set bus dividers
     rcc_config.ahb_divider = local_config.ahb_divider;
     rcc_config.apb1_divider = local_config.apb1_divider;
     rcc_config.apb2_divider = local_config.apb2_divider;
     
     // Configure PLL if requested
     rcc_config.use_pll = local_config.enable_pll;
     if (local_config.enable_pll) {
         rcc_config.target_frequency = local_config.target_system_clock;
         rcc_config.pll_config.auto_calculate = true;
     }
     
     // Initialize RCC
     status = rcc_interface->Init(rcc_interface->state, &rcc_config);
     if (status != STATUS_OK) {
         return status;
     }
     
     // Get the actual system and bus clocks that were set
     uint32_t system_clock;
     status = rcc_interface->Control(rcc_interface->state, RCC_CTRL_GET_SYSTEM_CLOCK, &system_clock);
     if (status != STATUS_OK) {
         return status;
     }
     
     // Store the clock frequencies
     system_state.system_clock_freq = system_clock;
     system_state.ahb_clock_freq = system_clock / local_config.ahb_divider;
     system_state.apb1_clock_freq = system_state.ahb_clock_freq / local_config.apb1_divider;
     system_state.apb2_clock_freq = system_state.ahb_clock_freq / local_config.apb2_divider;
     system_state.using_hse = local_config.use_hse;
     system_state.using_pll = local_config.enable_pll;
     
     // Configure flash latency based on system clock
     status = SystemCore_ConfigureFlashLatency(system_state.system_clock_freq, local_config.flash_latency);
     if (status != STATUS_OK) {
         return status;
     }
     
     // Configure SysTick if requested
     if (local_config.enable_systick) {
         status = SystemCore_ConfigureSysTick(local_config.systick_interval_ms);
         if (status != STATUS_OK) {
             return status;
         }
     }
     
     // Mark as initialized
     system_state.initialized = true;
     
     return STATUS_OK;
 }
 
 /**
  * @brief De-initialize the core system components
  * 
  * @return Status_t De-initialization status
  */
 Status_t SystemCore_DeInit(void) {
     Status_t status;
     HW_Interface_t *rcc_interface;
     
     // Get the RCC interface
     rcc_interface = RCC_GetInterface();
     if (rcc_interface == NULL) {
         return STATUS_ERROR;
     }
     
     // De-initialize RCC
     status = rcc_interface->DeInit(rcc_interface->state);
     if (status != STATUS_OK) {
         return status;
     }
     
     // Reset system state
     system_state.initialized = false;
     system_state.system_clock_freq = HSI_CLOCK_FREQ; // Revert to HSI
     system_state.ahb_clock_freq = HSI_CLOCK_FREQ;
     system_state.apb1_clock_freq = HSI_CLOCK_FREQ;
     system_state.apb2_clock_freq = HSI_CLOCK_FREQ;
     system_state.using_hse = false;
     system_state.using_pll = false;
     
     return STATUS_OK;
 }
 
 /**
  * @brief Get the current system clock frequency
  * 
  * @return uint32_t Current system clock frequency in Hz
  */
 uint32_t SystemCore_GetSystemClock(void) {
     return system_state.system_clock_freq;
 }
 
 /**
  * @brief Get the current AHB clock frequency
  * 
  * @return uint32_t Current AHB clock frequency in Hz
  */
 uint32_t SystemCore_GetAHBClock(void) {
     return system_state.ahb_clock_freq;
 }
 
 /**
  * @brief Get the current APB1 clock frequency
  * 
  * @return uint32_t Current APB1 clock frequency in Hz
  */
 uint32_t SystemCore_GetAPB1Clock(void) {
     return system_state.apb1_clock_freq;
 }
 
 /**
  * @brief Get the current APB2 clock frequency
  * 
  * @return uint32_t Current APB2 clock frequency in Hz
  */
 uint32_t SystemCore_GetAPB2Clock(void) {
     return system_state.apb2_clock_freq;
 }
 
 /**
  * @brief Check if system is using HSE as clock source
  * 
  * @return bool true if HSE is being used, false otherwise
  */
 bool SystemCore_IsUsingHSE(void) {
     return system_state.using_hse;
 }
 
 /**
  * @brief Check if system is using PLL as clock source
  * 
  * @return bool true if PLL is being used, false otherwise
  */
 bool SystemCore_IsUsingPLL(void) {
     return system_state.using_pll;
 }
 
 /**
  * @brief Configure flash latency based on system clock frequency
  * 
  * @param system_clock_freq System clock frequency in Hz
  * @param override_latency Flash latency override value (0xFF for auto)
  * @return Status_t Configuration status
  */
 Status_t SystemCore_ConfigureFlashLatency(uint32_t system_clock_freq, uint8_t override_latency) {
     uint8_t latency;
     
     // If override is provided and valid, use it
     if (override_latency != 0xFF && override_latency <= 7) {
         latency = override_latency;
     } else {
         // Otherwise auto-calculate based on clock speed (assuming 3.3V operation)
         if (system_clock_freq <= 24000000) {
             latency = 0; // 0 WS
         } else if (system_clock_freq <= 48000000) {
             latency = 1; // 1 WS
         } else if (system_clock_freq <= 72000000) {
             latency = 2; // 2 WS
         } else {
             latency = 3; // 3 WS (for up to 84 MHz)
         }
     }
     
     // Access flash control register
     volatile uint32_t *flash_acr = (volatile uint32_t*)(FLASH_ACR);
     
     // Configure flash latency
     uint32_t reg_value = *flash_acr;
     reg_value &= ~(FLASH_ACR_LATENCY_MSK);  // Clear latency bits
     reg_value |= (latency << FLASH_ACR_LATENCY_POS);  // Set new latency
     
     // Enable prefetch, instruction cache, and data cache for better performance
     reg_value |= FLASH_ACR_PRFTEN_MSK | FLASH_ACR_ICEN_MSK | FLASH_ACR_DCEN_MSK;
     
     // Write the new value
     *flash_acr = reg_value;
     
     // Store the flash latency
     system_state.flash_latency = latency;
     
     return STATUS_OK;
 }
 
 /**
  * @brief Configure SysTick timer
  * 
  * @param interval_ms SysTick interrupt interval in milliseconds
  * @return Status_t Configuration status
  */
 Status_t SystemCore_ConfigureSysTick(uint32_t interval_ms) {

     Status_t status;
     
     // Get the SysTick interface
     HW_Interface_t *systick_interface = SysTick_GetInterface();
     if (systick_interface == NULL) {
         return STATUS_ERROR;
     }
     
     // Calculate reload value for the given interval
     uint32_t reload_value = (system_state.system_clock_freq / 1000) * interval_ms;
     
     // Check if reload value is within valid range
     if (reload_value > 0x00FFFFFF) {
         return STATUS_INVALID_PARAM; // SysTick has a 24-bit counter
     }
     
     // Configure SysTick
     SysTick_Config_t systick_config = {
         .reload_value = reload_value,
         .enable_interrupt = true,
         .use_processor_clock = true  // Use processor clock
     };
     
     // Initialize SysTick
     status = systick_interface->Init(systick_interface->state, &systick_config);
     if (status != STATUS_OK) {
         return status;
     }
     
     // Start SysTick
     status = systick_interface->Control(systick_interface->state, SYSTICK_CTRL_START, NULL);
     if (status != STATUS_OK) {
         return status;
     }
     
     return STATUS_OK;
 }