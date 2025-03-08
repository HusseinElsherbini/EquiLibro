/**
 * @file rcc.c
 * @brief RCC hardware interface implementation
 * 
 * This file implements the hardware interface for the Reset and Clock Control (RCC)
 * peripheral, providing functions for configuring system clocks and enabling
 * peripheral clocks.
 */

 #include "hardware_abstraction/rcc.h"
 #include "platform.h"
 
 // Internal state structure for RCC interface
 typedef struct {
     bool initialized;
     RCC_Config_t config;
     uint32_t system_clock_freq;
     uint32_t ahb_clock_freq;
     uint32_t apb1_clock_freq;
     uint32_t apb2_clock_freq;
 } RCC_State_t;
 
 // Static instance of RCC state
 static RCC_State_t rcc_state = {false};
 
 // Static instance of RCC hardware interface
 static HW_Interface_t rcc_interface;
 
// Forward declarations for interface functions and helpers
static Status_t RCC_Init(void *state, void *config);
static Status_t RCC_DeInit(void *state);
static Status_t RCC_Control(void *state, uint32_t command, void *param);
static Status_t RCC_Read(void *state, void *buffer, uint16_t size, uint32_t timeout);
static Status_t RCC_Write(void *state, const void *data, uint16_t size, uint32_t timeout);
static Status_t RCC_RegisterCallback(void *state, uint32_t eventId, void (*Callback)(void *param), void *param);
static Status_t RCC_CalculatePLLSettings(RCC_Config_t *config);

 /**
  * @brief Initialize the RCC peripheral with the specified configuration
  * 
  * @param state Pointer to the RCC state
  * @param config Pointer to the RCC configuration
  * @return Status_t Initialization status
  */
 /**
 * @brief Initialize the RCC peripheral with the specified configuration
 * 
 * @param state Pointer to the RCC state
 * @param config Pointer to the RCC configuration
 * @return Status_t Initialization status
 */
static Status_t RCC_Init(void *state, void *config) {
    RCC_State_t *rcc_state = (RCC_State_t *)state;
    RCC_Config_t *rcc_config = (RCC_Config_t *)config;
    
    if (rcc_config == NULL) {
        return STATUS_INVALID_PARAM;
    }
    
    // Calculate PLL settings if auto-calculate is enabled
    if (rcc_config->use_pll && rcc_config->pll_config.auto_calculate) {
        Status_t status = RCC_CalculatePLLSettings(rcc_config);
        if (status != STATUS_OK) {
            return status;
        }
    }
    
    // Save the configuration
    rcc_state->config = *rcc_config;
    
    // Configure the RCC based on the configuration
    
    // 1. Enable HSE if requested
    if (rcc_config->hse_frequency > 0) {
        // Enable HSE
        SET_BIT(RCC_REGS->CR, RCC_CR_HSEON_MSK); // HSEON bit
        
        // Wait for HSE to be ready
        uint32_t timeout_counter = 0;
        while (!(RCC_REGS->CR & RCC_CR_HSERDY_MSK)) { // HSERDY bit
            timeout_counter++;
            if (timeout_counter > 50000) {
                return STATUS_TIMEOUT; // HSE failed to start
            }
        }
    }
    
    // 2. Configure PLL if requested
    if (rcc_config->use_pll) {
        // Disable PLL before configuring
        CLEAR_BIT(RCC_REGS->CR, RCC_CR_PLLON_MSK); // PLLON bit
        
        // Wait for PLL to be disabled
        uint32_t timeout_counter = 0;
        while (RCC_REGS->CR & RCC_CR_PLLRDY_MSK) { // PLLRDY bit
            timeout_counter++;
            if (timeout_counter > 50000) {
                return STATUS_TIMEOUT; // PLL failed to stop
            }
        }
        
        // Calculate p_value from p_divider for PLLCFGR register
        uint32_t p_value;
        switch (rcc_config->pll_config.p_divider) {
            case 2: p_value = 0; break;
            case 4: p_value = 1; break;
            case 6: p_value = 2; break;
            case 8: p_value = 3; break;
            default: return STATUS_INVALID_PARAM; // Invalid PLLP
        }
        
        // Configure PLL
        RCC_REGS->PLLCFGR = 
            (rcc_config->pll_config.m_divider << 0) | 
            (rcc_config->pll_config.n_multiplier << 6) |
            (p_value << 16) |
            (rcc_config->pll_config.source << 22) |
            (rcc_config->pll_config.q_divider << 24);
        
        // Enable PLL
        SET_BIT(RCC_REGS->CR, RCC_CR_PLLON_MSK); // PLLON bit
        
        // Wait for PLL to be ready
        timeout_counter = 0;
        while (!(RCC_REGS->CR & RCC_CR_PLLRDY_MSK)) { // PLLRDY bit
            timeout_counter++;
            if (timeout_counter > 50000) {
                return STATUS_TIMEOUT; // PLL failed to start
            }
        }
        
        // Calculate actual system clock frequency
        uint32_t pll_input;
        if (rcc_config->pll_config.source == 1) { // HSE
            pll_input = rcc_config->hse_frequency;
        } else { // HSI
            pll_input = 16000000; // 16 MHz internal oscillator
        }
        
        uint32_t vco_input = pll_input / rcc_config->pll_config.m_divider;
        uint32_t vco_output = vco_input * rcc_config->pll_config.n_multiplier;
        uint32_t system_clock = vco_output / rcc_config->pll_config.p_divider;
        
        rcc_state->system_clock_freq = system_clock;
    } else if (rcc_config->hse_frequency > 0) {
        // Using HSE directly
        rcc_state->system_clock_freq = rcc_config->hse_frequency;
    } else {
        // Using HSI
        rcc_state->system_clock_freq = 16000000; // 16 MHz HSI
    }
    
    // 3. Configure flash latency based on the system clock frequency
    // Flash access time configuration
    uint32_t flash_latency;
    
    // Set flash latency based on system clock (assuming 3.3V operation)
    if (rcc_state->system_clock_freq <= 24000000) {
        flash_latency = 0; // 0 WS
    } else if (rcc_state->system_clock_freq <= 48000000) {
        flash_latency = 1; // 1 WS
    } else if (rcc_state->system_clock_freq <= 72000000) {
        flash_latency = 2; // 2 WS
    } else {
        flash_latency = 3; // 3 WS (for up to 84 MHz)
    }
    
    // Configure Flash access control register (ACR)
    // First read current ACR value
    volatile uint32_t *flash_acr = (volatile uint32_t*)FLASH_ACR_REG_ADDR;
    // Clear latency bits
    *flash_acr &= ~(0xF);
    // Set new latency
    *flash_acr |= flash_latency;
    
    // 4. Configure AHB, APB1, and APB2 prescalers
    uint32_t cfgr = 0;
    
    // AHB prescaler (HPRE)
    switch (rcc_config->ahb_divider) {
        case 1: cfgr |= (0 << 4); break;
        case 2: cfgr |= (8 << 4); break;
        case 4: cfgr |= (9 << 4); break;
        case 8: cfgr |= (10 << 4); break;
        case 16: cfgr |= (11 << 4); break;
        case 64: cfgr |= (12 << 4); break;
        case 128: cfgr |= (13 << 4); break;
        case 256: cfgr |= (14 << 4); break;
        case 512: cfgr |= (15 << 4); break;
        default: return STATUS_INVALID_PARAM;
    }
    
    // APB1 prescaler (PPRE1)
    switch (rcc_config->apb1_divider) {
        case 1: cfgr |= (0 << 10); break;
        case 2: cfgr |= (4 << 10); break;
        case 4: cfgr |= (5 << 10); break;
        case 8: cfgr |= (6 << 10); break;
        case 16: cfgr |= (7 << 10); break;
        default: return STATUS_INVALID_PARAM;
    }
    
    // APB2 prescaler (PPRE2)
    switch (rcc_config->apb2_divider) {
        case 1: cfgr |= (0 << 13); break;
        case 2: cfgr |= (4 << 13); break;
        case 4: cfgr |= (5 << 13); break;
        case 8: cfgr |= (6 << 13); break;
        case 16: cfgr |= (7 << 13); break;
        default: return STATUS_INVALID_PARAM;
    }
    
    // 5. Select system clock source
    if (rcc_config->use_pll) {
        cfgr |= (2 << 0); // PLL as system clock
    } else if (rcc_config->hse_frequency > 0) {
        cfgr |= (1 << 0); // HSE as system clock
    } else {
        cfgr |= (0 << 0); // HSI as system clock
    }
    
    // Apply the configuration
    RCC_REGS->CFGR = cfgr;
    
    // Wait for the system clock source to be used
    uint32_t timeout_counter = 0;
    if (rcc_config->use_pll) {
        while ((RCC_REGS->CFGR & (3 << 2)) != (2 << 2)) { // PLL used as system clock
            timeout_counter++;
            if (timeout_counter > 50000) {
                return STATUS_TIMEOUT;
            }
        }
    } else if (rcc_config->hse_frequency > 0) {
        while ((RCC_REGS->CFGR & (3 << 2)) != (1 << 2)) { // HSE used as system clock
            timeout_counter++;
            if (timeout_counter > 50000) {
                return STATUS_TIMEOUT;
            }
        }
    } else {
        while ((RCC_REGS->CFGR & (3 << 2)) != (0 << 2)) { // HSI used as system clock
            timeout_counter++;
            if (timeout_counter > 50000) {
                return STATUS_TIMEOUT;
            }
        }
    }
    
    // Calculate and store bus frequencies
    rcc_state->ahb_clock_freq = rcc_state->system_clock_freq / rcc_config->ahb_divider;
    rcc_state->apb1_clock_freq = rcc_state->ahb_clock_freq / rcc_config->apb1_divider;
    rcc_state->apb2_clock_freq = rcc_state->ahb_clock_freq / rcc_config->apb2_divider;
    
    // Mark as initialized
    rcc_state->initialized = true;
    
    return STATUS_OK;
}
 /**
 * @brief Calculate optimal PLL settings for a target frequency
 * 
 * @param config Pointer to the RCC configuration to update
 * @return Status_t Configuration status
 */
static Status_t RCC_CalculatePLLSettings(RCC_Config_t *config) {
    if (config == NULL) {
        return STATUS_INVALID_PARAM;
    }
    
    uint32_t target_frequency = config->target_frequency;
    
    // Check if target frequency is within valid range for STM32F401
    if (target_frequency > 84000000) {
        return STATUS_INVALID_PARAM; // Exceeds maximum frequency
    }
    
    // Default to using HSE if available
    uint32_t pll_source_freq;
    if (config->hse_frequency > 0) {
        pll_source_freq = config->hse_frequency;
        config->pll_config.source = 1; // HSE
    } else {
        pll_source_freq = 16000000; // 16 MHz internal oscillator
        config->pll_config.source = 0; // HSI
    }
    
    // Find the best VCO input frequency (target 2MHz for best jitter performance)
    // Find the largest PLLM value that gives VCO input between 1-2MHz
    config->pll_config.m_divider = pll_source_freq / 2000000;
    if (config->pll_config.m_divider < 2) {
        config->pll_config.m_divider = 2; // Minimum PLLM value
    }
    uint32_t vco_input = pll_source_freq / config->pll_config.m_divider;
    // Should add a check here to ensure 1MHz <= vco_input <= 2MHz
    if(vco_input < 1000000 || vco_input > 2000000) {
        return STATUS_INVALID_PARAM; // Cannot achieve target frequency
    }
    
    // Choose a PLLP value
    // PLLP can be 2, 4, 6, or 8
    // We'll choose the smallest valid PLLP
    uint32_t vco_output;
    
    // Try with P = 2
    config->pll_config.p_divider = 2;
    vco_output = target_frequency * config->pll_config.p_divider;
    
    // Check if VCO output is in range (100-432 MHz)
    if (vco_output > 432000000) {
        // Try with P = 4
        config->pll_config.p_divider = 4;
        vco_output = target_frequency * config->pll_config.p_divider;
        
        if (vco_output > 432000000) {
            // Try with P = 6
            config->pll_config.p_divider = 6;
            vco_output = target_frequency * config->pll_config.p_divider;
            
            if (vco_output > 432000000) {
                // Try with P = 8
                config->pll_config.p_divider = 8;
                vco_output = target_frequency * config->pll_config.p_divider;
                
                if (vco_output > 432000000) {
                    return STATUS_INVALID_PARAM; // Cannot achieve target frequency
                }
            }
        }
    }
    
    // Calculate PLLN to achieve the desired VCO output
    config->pll_config.n_multiplier = vco_output / vco_input;
    
    // Check if PLLN is in valid range (50-432)
    if (config->pll_config.n_multiplier < 50) {
        config->pll_config.n_multiplier = 50; // Minimum PLLN
    } else if (config->pll_config.n_multiplier > 432) {
        config->pll_config.n_multiplier = 432; // Maximum PLLN
    }
    
    // Recalculate actual VCO output with the rounded PLLN
    vco_output = vco_input * config->pll_config.n_multiplier;
    
    // Calculate actual PLL output (this might differ slightly from target due to rounding)
    uint32_t pll_output = vco_output / config->pll_config.p_divider;
    
    // Set the PLLQ value for USB (if used)
    // USB requires a 48MHz clock
    config->pll_config.q_divider = vco_output / 48000000;
    if (config->pll_config.q_divider < 2) {
        config->pll_config.q_divider = 2; // Minimum PLLQ
    } else if (config->pll_config.q_divider > 15) {
        config->pll_config.q_divider = 15; // Maximum PLLQ
    }
    
    // Update the target frequency to the actual achievable frequency
    config->target_frequency = pll_output;
    config->use_pll = true;
    
    return STATUS_OK;
}

/**
 * @brief Configure the system clock for a specific frequency
 * 
 * This is a helper function that simplifies clock configuration by allowing the user
 * to specify only the desired system clock frequency. The function will automatically
 * calculate the optimal PLL settings.
 * 
 * @param target_frequency Desired system clock frequency in Hz (max 84 MHz for STM32F401)
 * @param use_hse Whether to use the external oscillator (true) or internal oscillator (false)
 * @return Status_t Status of the operation
 */
Status_t RCC_ConfigureSystemClock(uint32_t target_frequency, bool use_hse) {
    HW_Interface_t *rcc_interface = RCC_GetInterface();
    if (rcc_interface == NULL) {
        return STATUS_ERROR;
    }
    
    RCC_Config_t rcc_config = {0};
    
    // Set up basic configuration
    if (use_hse) {
        rcc_config.hse_frequency = 24000000; // Using 24 MHz external crystal oscillator
    } else {
        rcc_config.hse_frequency = 0; // Use HSI
    }
    
    // Bus prescalers - set reasonable defaults
    rcc_config.ahb_divider = 1;
    
    // Ensure APB1 doesn't exceed 42 MHz
    if (target_frequency > 42000000) {
        rcc_config.apb1_divider = 2; // APB1 at half speed
    } else {
        rcc_config.apb1_divider = 1;
    }
    
    rcc_config.apb2_divider = 1; // APB2 at full speed
    
    // Set up PLL for auto-calculation
    rcc_config.use_pll = true;
    rcc_config.target_frequency = target_frequency;
    rcc_config.pll_config.auto_calculate = true;
    
    // First try to calculate PLL settings
    Status_t status = RCC_CalculatePLLSettings(&rcc_config);
    if (status != STATUS_OK) {
        return status; // Return error from calculation
    }
    
    // Initialize RCC with the calculated configuration
    return rcc_interface->Init(rcc_interface->state, &rcc_config);
}
 /**
  * @brief De-initialize the RCC peripheral
  * 
  * @param state Pointer to the RCC state
  * @return Status_t De-initialization status
  */
 static Status_t RCC_DeInit(void *state) {
     RCC_State_t *rcc_state = (RCC_State_t *)state;
     
     // Reset the RCC configuration
     // Note: This is typically not done in practice as it would reset the entire system
     
     // Mark as not initialized
     rcc_state->initialized = false;
     
     return STATUS_OK;
 }
 
 /**
  * @brief Control the RCC peripheral
  * 
  * @param state Pointer to the RCC state
  * @param command Control command
  * @param param Command parameters
  * @return Status_t Control operation status
  */
 static Status_t RCC_Control(void *state, uint32_t command, void *param) {
     RCC_State_t *rcc_state = (RCC_State_t *)state;
     
     if (!rcc_state->initialized) {
         return STATUS_NOT_INITIALIZED;
     }
     
     if (param == NULL) {
         return STATUS_INVALID_PARAM;
     }
     
     switch (command) {
         case RCC_CTRL_ENABLE_PERIPHERAL_CLOCK: {
             RCC_Peripheral_t periph = *((RCC_Peripheral_t *)param);
             
             // Enable the peripheral clock based on the peripheral ID
             switch (periph) {
                 // AHB1 peripherals
                 case RCC_PERIPH_GPIOA:
                     SET_BIT(RCC_REGS->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
                     break;
                 case RCC_PERIPH_GPIOB:
                     SET_BIT(RCC_REGS->AHB1ENR, RCC_AHB1ENR_GPIOBEN);
                     break;
                 case RCC_PERIPH_GPIOC:
                     SET_BIT(RCC_REGS->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
                     break;
                 case RCC_PERIPH_GPIOD:
                     SET_BIT(RCC_REGS->AHB1ENR, RCC_AHB1ENR_GPIODEN);
                     break;
                 case RCC_PERIPH_GPIOE:
                     SET_BIT(RCC_REGS->AHB1ENR, RCC_AHB1ENR_GPIOEEN);
                     break;
                 case RCC_PERIPH_GPIOH:
                     SET_BIT(RCC_REGS->AHB1ENR, RCC_AHB1ENR_GPIOHEN);
                     break;
                 case RCC_PERIPH_DMA1:
                     SET_BIT(RCC_REGS->AHB1ENR, RCC_AHB1ENR_DMA1EN);
                     break;
                 case RCC_PERIPH_DMA2:
                     SET_BIT(RCC_REGS->AHB1ENR, RCC_AHB1ENR_DMA2EN);
                     break;
                 
                 // APB1 peripherals
                 case RCC_PERIPH_TIM2:
                     SET_BIT(RCC_REGS->APB1ENR, RCC_APB1ENR_TIM2EN);
                     break;
                 case RCC_PERIPH_TIM3:
                     SET_BIT(RCC_REGS->APB1ENR, RCC_APB1ENR_TIM3EN);
                     break;
                 case RCC_PERIPH_TIM4:
                     SET_BIT(RCC_REGS->APB1ENR, RCC_APB1ENR_TIM4EN);
                     break;
                 case RCC_PERIPH_TIM5:
                     SET_BIT(RCC_REGS->APB1ENR, RCC_APB1ENR_TIM5EN);
                     break;
                 case RCC_PERIPH_SPI2:
                     SET_BIT(RCC_REGS->APB1ENR, RCC_APB1ENR_SPI2EN);
                     break;
                 case RCC_PERIPH_SPI3:
                     SET_BIT(RCC_REGS->APB1ENR, RCC_APB1ENR_SPI3EN);
                     break;
                 case RCC_PERIPH_USART2:
                     SET_BIT(RCC_REGS->APB1ENR, RCC_APB1ENR_USART2EN);
                     break;
                 case RCC_PERIPH_I2C1:
                     SET_BIT(RCC_REGS->APB1ENR, RCC_APB1ENR_I2C1EN);
                     break;
                 case RCC_PERIPH_I2C2:
                     SET_BIT(RCC_REGS->APB1ENR, RCC_APB1ENR_I2C2EN);
                     break;
                 case RCC_PERIPH_I2C3:
                     SET_BIT(RCC_REGS->APB1ENR, RCC_APB1ENR_I2C3EN);
                     break;
                 case RCC_PERIPH_PWR:
                     SET_BIT(RCC_REGS->APB1ENR, RCC_APB1ENR_PWREN);
                     break;
                 
                 // APB2 peripherals
                 case RCC_PERIPH_TIM1:
                     SET_BIT(RCC_REGS->APB2ENR, RCC_APB2ENR_TIM1EN);
                     break;
                 case RCC_PERIPH_USART1:
                     SET_BIT(RCC_REGS->APB2ENR, RCC_APB2ENR_USART1EN);
                     break;
                 case RCC_PERIPH_USART6:
                     SET_BIT(RCC_REGS->APB2ENR, RCC_APB2ENR_USART6EN);
                     break;
                 case RCC_PERIPH_ADC1:
                     SET_BIT(RCC_REGS->APB2ENR, RCC_APB2ENR_ADC1EN);
                     break;
                 case RCC_PERIPH_SPI1:
                     SET_BIT(RCC_REGS->APB2ENR, RCC_APB2ENR_SPI1EN);
                     break;
                 case RCC_PERIPH_SPI4:
                     SET_BIT(RCC_REGS->APB2ENR, RCC_APB2ENR_SPI4EN);
                     break;
                 case RCC_PERIPH_TIM9:
                     SET_BIT(RCC_REGS->APB2ENR, RCC_APB2ENR_TIM9EN);
                     break;
                 case RCC_PERIPH_TIM10:
                     SET_BIT(RCC_REGS->APB2ENR, RCC_APB2ENR_TIM10EN);
                     break;
                 case RCC_PERIPH_TIM11:
                     SET_BIT(RCC_REGS->APB2ENR, RCC_APB2ENR_TIM11EN);
                     break;
                 
                 default:
                     return STATUS_INVALID_PARAM;
             }
             
             return STATUS_OK;
         }
         
         case RCC_CTRL_DISABLE_PERIPHERAL_CLOCK: {
             RCC_Peripheral_t periph = *((RCC_Peripheral_t *)param);
             
             // Disable the peripheral clock based on the peripheral ID
             // Similar to enable but clear the bit instead of setting it
             // Implementation omitted for brevity
             
             return STATUS_OK;
         }
         
         case RCC_CTRL_GET_SYSTEM_CLOCK: {
             uint32_t *clock_freq = (uint32_t *)param;
             *clock_freq = rcc_state->system_clock_freq;
             return STATUS_OK;
         }
         
         case RCC_CTRL_GET_PERIPHERAL_CLOCK: {
             // Get the peripheral clock frequency
             // Implementation omitted for brevity
             return STATUS_OK;
         }
         
         default:
             return STATUS_NOT_SUPPORTED;
     }
 }
 
 /**
  * @brief Read function for RCC interface (not used)
  */
 static Status_t RCC_Read(void *state, void *buffer, uint16_t size, uint32_t timeout) {
     return STATUS_NOT_SUPPORTED; // RCC does not support direct reads
 }
 
 /**
  * @brief Write function for RCC interface (not used)
  */
 static Status_t RCC_Write(void *state, const void *data, uint16_t size, uint32_t timeout) {
     return STATUS_NOT_SUPPORTED; // RCC does not support direct writes
 }
 
 /**
  * @brief Register callback function for RCC interface (not used)
  */
 static Status_t RCC_RegisterCallback(void *state, uint32_t eventId, void (*Callback)(void *param), void *param) {
     return STATUS_NOT_SUPPORTED; // RCC does not support callbacks
 }
 
 /**
  * @brief Get the RCC hardware interface
  * 
  * @return HW_Interface_t* Pointer to the RCC hardware interface
  */
 HW_Interface_t* RCC_GetInterface(void) {
     // Initialize the interface function pointers
     rcc_interface.state = &rcc_state;
     rcc_interface.Init = RCC_Init;
     rcc_interface.DeInit = RCC_DeInit;
     rcc_interface.Control = RCC_Control;
     rcc_interface.Read = RCC_Read;
     rcc_interface.Write = RCC_Write;
     rcc_interface.RegisterCallback = RCC_RegisterCallback;
     
     return &rcc_interface;
 }