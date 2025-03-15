#pragma once

#include "platform.hpp"

namespace Platform {
namespace RCC {
    constexpr uint32_t RCC_BASE = (AHB1PERIPH_BASE + 0x3800UL);

    // RCC register structure (based on RM0368)
    struct Registers {
        volatile uint32_t CR;            // Clock control register
        volatile uint32_t PLLCFGR;       // PLL configuration register
        volatile uint32_t CFGR;          // Clock configuration register
        volatile uint32_t CIR;           // Clock interrupt register
        volatile uint32_t AHB1RSTR;      // AHB1 peripheral reset register
        volatile uint32_t AHB2RSTR;      // AHB2 peripheral reset register
        uint32_t RESERVED0[2];
        volatile uint32_t APB1RSTR;      // APB1 peripheral reset register
        volatile uint32_t APB2RSTR;      // APB2 peripheral reset register
        uint32_t RESERVED1[2];
        volatile uint32_t AHB1ENR;       // AHB1 peripheral clock enable register
        volatile uint32_t AHB2ENR;       // AHB2 peripheral clock enable register
        uint32_t RESERVED2[2];
        volatile uint32_t APB1ENR;       // APB1 peripheral clock enable register
        volatile uint32_t APB2ENR;       // APB2 peripheral clock enable register
        uint32_t RESERVED3[2];
        volatile uint32_t AHB1LPENR;     // AHB1 peripheral clock enable in low power mode register
        volatile uint32_t AHB2LPENR;     // AHB2 peripheral clock enable in low power mode register
        uint32_t RESERVED4[2];
        volatile uint32_t APB1LPENR;     // APB1 peripheral clock enable in low power mode register
        volatile uint32_t APB2LPENR;     // APB2 peripheral clock enable in low power mode register
        uint32_t RESERVED5[2];
        volatile uint32_t BDCR;          // Backup domain control register
        volatile uint32_t CSR;           // Clock control & status register
        uint32_t RESERVED6[2];
        volatile uint32_t SSCGR;         // Spread spectrum clock generation register
        volatile uint32_t PLLI2SCFGR;    // PLLI2S configuration register
        uint32_t RESERVED7;
        volatile uint32_t DCKCFGR;       // Dedicated Clocks Configuration Register
    };

    // RCC_CR register bits
    enum class CR : uint32_t {
        HSION = (1UL << 0),      // Internal high-speed clock enable
        HSIRDY = (1UL << 1),     // Internal high-speed clock ready flag
        HSITRIM_MSK = (0x1FUL << 3), // Internal high-speed clock trimming mask
        HSICAL_MSK = (0xFFUL << 8),  // Internal high-speed clock calibration mask
        HSEON = (1UL << 16),     // HSE clock enable
        HSERDY = (1UL << 17),    // HSE clock ready flag
        HSEBYP = (1UL << 18),    // HSE clock bypass
        CSSON = (1UL << 19),     // Clock security system enable
        PLLON = (1UL << 24),     // Main PLL enable
        PLLRDY = (1UL << 25),    // Main PLL clock ready flag
        PLLI2SON = (1UL << 26),  // PLLI2S enable
        PLLI2SRDY = (1UL << 27)  // PLLI2S clock ready flag
    };

    // PLLCFGR register bits
    enum class PLLCFGR : uint32_t {
        PLLM_MSK = (0x3FUL << 0),    // Division factor for the main PLL input clock
        PLLN_MSK = (0x1FFUL << 6),   // Main PLL multiplication factor for VCO
        PLLP_MSK = (0x3UL << 16),    // Main PLL division factor for main system clock
        PLLSRC_HSI = 0UL,            // HSI clock selected as PLL entry clock source
        PLLSRC_HSE = (1UL << 22),    // HSE oscillator clock selected as PLL entry clock source
        PLLQ_MSK = (0xFUL << 24)     // Main PLL division factor for USB OTG FS, SDIO, RNG
    };

    // Clock configuration register (CFGR) bits
    enum class CFGR : uint32_t {
        SW_HSI = (0UL << 0),         // HSI selected as system clock
        SW_HSE = (1UL << 0),         // HSE selected as system clock
        SW_PLL = (2UL << 0),         // PLL selected as system clock
        SW_MSK = (0x3UL << 0),       // System clock switch mask
        SWS_HSI = (0UL << 2),        // HSI used as system clock
        SWS_HSE = (1UL << 2),        // HSE used as system clock
        SWS_PLL = (2UL << 2),        // PLL used as system clock
        SWS_MSK = (0x3UL << 2),      // System clock switch status mask
        HPRE_DIV1 = (0UL << 4),      // AHB prescaler: SYSCLK not divided
        HPRE_DIV2 = (8UL << 4),      // AHB prescaler: SYSCLK divided by 2
        HPRE_DIV4 = (9UL << 4),      // AHB prescaler: SYSCLK divided by 4
        HPRE_DIV8 = (10UL << 4),     // AHB prescaler: SYSCLK divided by 8
        HPRE_DIV16 = (11UL << 4),    // AHB prescaler: SYSCLK divided by 16
        HPRE_DIV64 = (12UL << 4),    // AHB prescaler: SYSCLK divided by 64
        HPRE_DIV128 = (13UL << 4),   // AHB prescaler: SYSCLK divided by 128
        HPRE_DIV256 = (14UL << 4),   // AHB prescaler: SYSCLK divided by 256
        HPRE_DIV512 = (15UL << 4),   // AHB prescaler: SYSCLK divided by 512
        HPRE_MSK = (0xFUL << 4),     // AHB prescaler mask
        PPRE1_DIV1 = (0UL << 10),    // APB1 prescaler: HCLK not divided
        PPRE1_DIV2 = (4UL << 10),    // APB1 prescaler: HCLK divided by 2
        PPRE1_DIV4 = (5UL << 10),    // APB1 prescaler: HCLK divided by 4
        PPRE1_DIV8 = (6UL << 10),    // APB1 prescaler: HCLK divided by 8
        PPRE1_DIV16 = (7UL << 10),   // APB1 prescaler: HCLK divided by 16
        PPRE1_MSK = (0x7UL << 10),   // APB1 prescaler mask
        PPRE2_DIV1 = (0UL << 13),    // APB2 prescaler: HCLK not divided
        PPRE2_DIV2 = (4UL << 13),    // APB2 prescaler: HCLK divided by 2
        PPRE2_DIV4 = (5UL << 13),    // APB2 prescaler: HCLK divided by 4
        PPRE2_DIV8 = (6UL << 13),    // APB2 prescaler: HCLK divided by 8
        PPRE2_DIV16 = (7UL << 13),   // APB2 prescaler: HCLK divided by 16
        PPRE2_MSK = (0x7UL << 13),   // APB2 prescaler mask
        RTCPRE_MSK = (0x1FUL << 16), // HSE division factor for RTC clock
        MCO1_HSI = (0UL << 21),      // HSI clock selected as MCO1 source
        MCO1_LSE = (1UL << 21),      // LSE clock selected as MCO1 source
        MCO1_HSE = (2UL << 21),      // HSE clock selected as MCO1 source
        MCO1_PLL = (3UL << 21),      // PLL clock selected as MCO1 source
        MCO1_MSK = (0x3UL << 21),    // MCO1 source selection mask
        MCO1PRE_MSK = (0x7UL << 24), // MCO1 prescaler mask
        MCO2PRE_MSK = (0x7UL << 27), // MCO2 prescaler mask
        MCO2_SYSCLK = (0UL << 30),   // System clock selected as MCO2 source
        MCO2_PLLI2S = (1UL << 30),   // PLLI2S clock selected as MCO2 source
        MCO2_HSE = (2UL << 30),      // HSE clock selected as MCO2 source
        MCO2_PLL = (3UL << 30),      // PLL clock selected as MCO2 source
        MCO2_MSK = (0x3UL << 30)     // MCO2 source selection mask
    };

    // AHB1 peripheral clock enable register (AHB1ENR) bits
    enum class AHB1ENR : uint32_t {
        GPIOAEN = (1UL << 0),
        GPIOBEN = (1UL << 1),
        GPIOCEN = (1UL << 2),
        GPIODEN = (1UL << 3),
        GPIOEEN = (1UL << 4),
        GPIOHEN = (1UL << 7),
        CRCEN = (1UL << 12),
        DMA1EN = (1UL << 21),
        DMA2EN = (1UL << 22)
    };

    // AHB2 peripheral clock enable register (AHB2ENR) bits
    enum class AHB2ENR : uint32_t {
        OTGFSEN = (1UL << 7)
    };

    // APB1 peripheral clock enable register (APB1ENR) bits
    enum class APB1ENR : uint32_t {
        TIM2EN = (1UL << 0),
        TIM3EN = (1UL << 1),
        TIM4EN = (1UL << 2),
        TIM5EN = (1UL << 3),
        WWDGEN = (1UL << 11),
        SPI2EN = (1UL << 14),
        SPI3EN = (1UL << 15),
        USART2EN = (1UL << 17),
        I2C1EN = (1UL << 21),
        I2C2EN = (1UL << 22),
        I2C3EN = (1UL << 23),
        PWREN = (1UL << 28)
    };

    // APB2 peripheral clock enable register (APB2ENR) bits
    enum class APB2ENR : uint32_t {
        TIM1EN = (1UL << 0),
        USART1EN = (1UL << 4),
        USART6EN = (1UL << 5),
        ADC1EN = (1UL << 8),
        SDIOEN = (1UL << 11),
        SPI1EN = (1UL << 12),
        SPI4EN = (1UL << 13),
        SYSCFGEN = (1UL << 14),
        TIM9EN = (1UL << 16),
        TIM10EN = (1UL << 17),
        TIM11EN = (1UL << 18)
    };

    // BDCR register bits
    enum class BDCR : uint32_t {
        LSEON = (1UL << 0),     // External low-speed oscillator enable
        LSERDY = (1UL << 1),    // External low-speed oscillator ready
        LSEBYP = (1UL << 2),    // External low-speed oscillator bypass
        RTCSEL_NONE = (0UL << 8), // No clock
        RTCSEL_LSE = (1UL << 8),  // LSE oscillator clock used as RTC clock
        RTCSEL_LSI = (2UL << 8),  // LSI oscillator clock used as RTC clock
        RTCSEL_HSE = (3UL << 8),  // HSE oscillator clock divided by a prescaler used as RTC clock
        RTCSEL_MSK = (3UL << 8),  // RTC clock source selection mask
        RTCEN = (1UL << 15),    // RTC clock enable
        BDRST = (1UL << 16)     // Backup domain software reset
    };

    // CSR register bits
    enum class CSR : uint32_t {
        LSION = (1UL << 0),     // Internal low-speed oscillator enable
        LSIRDY = (1UL << 1),    // Internal low-speed oscillator ready
        RMVF = (1UL << 24),     // Remove reset flag
        BORRSTF = (1UL << 25),  // BOR reset flag
        PINRSTF = (1UL << 26),  // PIN reset flag
        PORRSTF = (1UL << 27),  // POR/PDR reset flag
        SFTRSTF = (1UL << 28),  // Software reset flag
        WDGRSTF = (1UL << 29),  // Independent watchdog reset flag
        WWDGRSTF = (1UL << 30), // Window watchdog reset flag
        LPWRRSTF = (1UL << 31)  // Low-power reset flag
    };

    enum class RccBusType : uint8_t {
        AHB1 = 0x00,
        AHB2 = 0x01,
        APB1 = 0x02,
        APB2 = 0x03
    };

    struct RccPeripheralInfo {
        RccBusType bus;
        uint8_t bitPosition;
    };
    enum class RccPeripheral : uint32_t {
        // AHB1 peripherals - actual bit positions in RCC_AHB1ENR
        GPIOA = 0x00000000,  // Bit 0
        GPIOB = 0x00000001,  // Bit 1
        GPIOC = 0x00000002,  // Bit 2
        GPIOD = 0x00000003,  // Bit 3
        GPIOE = 0x00000004,  // Bit 4
        GPIOH = 0x00000007,  // Bit 7
        CRC = 0x0000000C,    // Bit 12
        DMA1 = 0x00000015,   // Bit 21
        DMA2 = 0x00000016,   // Bit 22
        
        // AHB2 peripherals - actual bit positions in RCC_AHB2ENR
        OTGFS = 0x00000107,  // Bit 7
        
        // APB1 peripherals - actual bit positions in RCC_APB1ENR
        TIM2 = 0x00000200,   // Bit 0
        TIM3 = 0x00000201,   // Bit 1
        TIM4 = 0x00000202,   // Bit 2
        TIM5 = 0x00000203,   // Bit 3
        WWDG = 0x0000020B,   // Bit 11
        SPI2 = 0x0000020E,   // Bit 14
        SPI3 = 0x0000020F,   // Bit 15
        USART2 = 0x00000211, // Bit 17
        I2C1 = 0x00000215,   // Bit 21
        I2C2 = 0x00000216,   // Bit 22
        I2C3 = 0x00000217,   // Bit 23
        PWR = 0x0000021C,    // Bit 28
        
        // APB2 peripherals - actual bit positions in RCC_APB2ENR
        TIM1 = 0x00000300,   // Bit 0
        USART1 = 0x00000304, // Bit 4
        USART6 = 0x00000305, // Bit 5
        ADC1 = 0x00000308,   // Bit 8
        SDIO = 0x0000030B,   // Bit 11
        SPI1 = 0x0000030C,   // Bit 12
        SPI4 = 0x0000030D,   // Bit 13
        SYSCFG = 0x0000030E, // Bit 14
        TIM9 = 0x00000310,   // Bit 16
        TIM10 = 0x00000311,  // Bit 17
        TIM11 = 0x00000312   // Bit 18
    };
    
    // Get RCC registers
    inline Registers* getRegisters() {
        return reinterpret_cast<Registers*>(RCC_BASE);
    }
    constexpr RccBusType GetBusType(RccPeripheral peripheral) {
        return static_cast<RccBusType>((static_cast<uint32_t>(peripheral) >> 8) & 0xFF);
    }
    
    constexpr uint8_t GetBitPosition(RccPeripheral peripheral) {
        return static_cast<uint8_t>(static_cast<uint32_t>(peripheral) & 0xFF) - 1;
    }
    // Helper functions for bit manipulation
    constexpr uint32_t getBitValue(CR bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(PLLCFGR bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(CFGR bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(AHB1ENR bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(AHB2ENR bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(APB1ENR bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(APB2ENR bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(BDCR bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(CSR bit) {
        return static_cast<uint32_t>(bit);
    }

    // Operator overloads for combining flags
    constexpr AHB1ENR operator|(AHB1ENR a, AHB1ENR b) {
        return static_cast<AHB1ENR>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr AHB2ENR operator|(AHB2ENR a, AHB2ENR b) {
        return static_cast<AHB2ENR>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr APB1ENR operator|(APB1ENR a, APB1ENR b) {
        return static_cast<APB1ENR>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr APB2ENR operator|(APB2ENR a, APB2ENR b) {
        return static_cast<APB2ENR>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr CR operator|(CR a, CR b) {
        return static_cast<CR>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr PLLCFGR operator|(PLLCFGR a, PLLCFGR b) {
        return static_cast<PLLCFGR>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr CFGR operator|(CFGR a, CFGR b) {
        return static_cast<CFGR>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr BDCR operator|(BDCR a, BDCR b) {
        return static_cast<BDCR>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr CSR operator|(CSR a, CSR b) {
        return static_cast<CSR>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }
}
}