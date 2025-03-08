/**
 * @file rcc.h
 * @brief Reset and Clock Control (RCC) hardware interface
 * 
 * This file provides an interface to the RCC peripheral for configuring
 * system clocks and enabling peripheral clocks.
 */

 #ifndef RCC_H
 #define RCC_H
 
 #include "hw_interface.h"
 #include "common/types.h"
 #include "common/error_codes.h"
 
 // RCC control commands
 #define RCC_CTRL_ENABLE_PERIPHERAL_CLOCK   0x0401
 #define RCC_CTRL_DISABLE_PERIPHERAL_CLOCK  0x0402
 #define RCC_CTRL_GET_SYSTEM_CLOCK          0x0403
 #define RCC_CTRL_GET_PERIPHERAL_CLOCK      0x0404
 
 // Peripheral identifiers for enabling/disabling clocks
 typedef enum {
     // AHB1 peripherals
     RCC_PERIPH_GPIOA,
     RCC_PERIPH_GPIOB,
     RCC_PERIPH_GPIOC,
     RCC_PERIPH_GPIOD,
     RCC_PERIPH_GPIOE,
     RCC_PERIPH_GPIOH,
     RCC_PERIPH_DMA1,
     RCC_PERIPH_DMA2,
     
     // APB1 peripherals
     RCC_PERIPH_TIM2,
     RCC_PERIPH_TIM3,
     RCC_PERIPH_TIM4,
     RCC_PERIPH_TIM5,
     RCC_PERIPH_SPI2,
     RCC_PERIPH_SPI3,
     RCC_PERIPH_USART2,
     RCC_PERIPH_I2C1,
     RCC_PERIPH_I2C2,
     RCC_PERIPH_I2C3,
     RCC_PERIPH_PWR,
     
     // APB2 peripherals
     RCC_PERIPH_TIM1,
     RCC_PERIPH_USART1,
     RCC_PERIPH_USART6,
     RCC_PERIPH_ADC1,
     RCC_PERIPH_SPI1,
     RCC_PERIPH_SPI4,
     RCC_PERIPH_TIM9,
     RCC_PERIPH_TIM10,
     RCC_PERIPH_TIM11,
     
     // Add more peripherals as needed
 } RCC_Peripheral_t;

 // RCC configuration structure
typedef struct {
    // Clock sources configuration
    uint32_t hse_frequency;      // HSE crystal frequency in Hz (0 if not used)
    bool use_pll;                // Whether to use the PLL for the system clock
    
    // Target frequency (used to auto-calculate PLL settings)
    uint32_t target_frequency;   // Desired system clock frequency in Hz
    
    // Bus prescalers
    uint32_t ahb_divider;        // AHB divider (1, 2, 4, 8, 16, 64, 128, 256, 512)
    uint32_t apb1_divider;       // APB1 divider (1, 2, 4, 8, 16)
    uint32_t apb2_divider;       // APB2 divider (1, 2, 4, 8, 16)
    
    // PLL configuration (can be set manually or auto-calculated)
    struct {
        uint32_t source;         // PLL source: 0=HSI, 1=HSE
        uint32_t m_divider;      // PLLM: Division factor for the main PLL input clock (2-63)
        uint32_t n_multiplier;   // PLLN: Main PLL multiplication factor (50-432)
        uint32_t p_divider;      // PLLP: Main PLL division factor for main system clock (2, 4, 6, 8)
        uint32_t q_divider;      // PLLQ: Main PLL division factor for USB, SDIO, RNG (2-15)
        bool auto_calculate;     // Whether to auto-calculate PLL settings from target_frequency
    } pll_config;
} RCC_Config_t;

 /**
  * @brief Get the RCC hardware interface
  * 
  * @return HW_Interface_t* Pointer to the RCC hardware interface
  */
 HW_Interface_t* RCC_GetInterface(void);
 Status_t RCC_ConfigureSystemClock(uint32_t target_frequency, bool use_hse);
 
 #endif /* RCC_H */