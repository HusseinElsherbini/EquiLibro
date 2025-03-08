#pragma once

#include <cstdint>
#include <cstdbool>

// Forward declarations for standard C libraries
extern "C" {
    #include <stdio.h>
    #include <stdlib.h>
}

namespace Platform {

// -------------------- Type Definitions --------------------

// IO access definitions for C++ with proper volatile semantics
#define __I  volatile       // Read-only
#define __O  volatile       // Write-only
#define __IO volatile       // Read-write

// Status codes
enum class Status {
    OK = 0,              // Operation completed successfully
    ERROR,               // Generic error
    BUSY,                // Resource is busy
    TIMEOUT,             // Operation timed out
    INVALID_PARAM,       // Invalid parameter
    NOT_SUPPORTED,       // Operation not supported
    RESOURCE_ERROR,      // Resource allocation error
    NOT_INITIALIZED,     // Component not initialized
    INVALID_STATE,       // Component in invalid state for operation
    NO_DATA,             // No data available
    BUFFER_OVERFLOW,     // Buffer overflow
    HARDWARE_ERROR,      // Hardware-specific error
    COMMUNICATION_ERROR  // Communication error
};

// -------------------- Clock Definitions --------------------

constexpr uint32_t HSE_CLOCK = 84000000U;  // High-speed external clock in Hz
constexpr uint32_t MCU_CLK = HSE_CLOCK;
constexpr uint32_t APB1_CLOCK = 42000000U; // APB1 peripheral clock
constexpr uint32_t APB2_CLOCK = 84000000U; // APB2 peripheral clock
constexpr uint32_t AHB1_CLOCK = 84000000U; // AHB1 peripheral clock
constexpr uint32_t AHB2_CLOCK = 84000000U; // AHB2 peripheral clock

// -------------------- Memory Map Base Addresses --------------------

// Core memory map
constexpr uint32_t PERIPH_BASE = 0x40000000UL;
constexpr uint32_t APB1PERIPH_BASE = PERIPH_BASE;
constexpr uint32_t APB2PERIPH_BASE = (PERIPH_BASE + 0x00010000UL);
constexpr uint32_t AHB1PERIPH_BASE = (PERIPH_BASE + 0x00020000UL);
constexpr uint32_t AHB2PERIPH_BASE = (PERIPH_BASE + 0x10000000UL);

// -------------------- Core Peripheral Definitions --------------------

// System Control Space Base Address
constexpr uint32_t SCS_BASE = 0xE000E000UL;
constexpr uint32_t NVIC_BASE = (SCS_BASE + 0x0100UL);        // NVIC Base Address
constexpr uint32_t SCB_BASE = (SCS_BASE + 0x0D00UL);         // System Control Block Base Address
constexpr uint32_t SysTick_BASE = (SCS_BASE + 0x0010UL);     // SysTick Base Address

// -------------------- NVIC Definitions --------------------

namespace NVIC {
    // IRQ numbers for STM32F4 - used by hardware abstraction layer
    enum class IRQn : int32_t {
        WWDG = 0,                   // Window WatchDog Interrupt
        PVD = 1,                    // PVD through EXTI Line detection Interrupt
        TAMP_STAMP = 2,             // Tamper and TimeStamp interrupts through the EXTI line
        RTC_WKUP = 3,               // RTC Wakeup interrupt through the EXTI line
        FLASH = 4,                  // FLASH global Interrupt
        RCC = 5,                    // RCC global Interrupt
        EXTI0 = 6,                  // EXTI Line0 Interrupt
        EXTI1 = 7,                  // EXTI Line1 Interrupt
        EXTI2 = 8,                  // EXTI Line2 Interrupt
        EXTI3 = 9,                  // EXTI Line3 Interrupt
        EXTI4 = 10,                 // EXTI Line4 Interrupt
        DMA1_Stream0 = 11,          // DMA1 Stream 0 global Interrupt
        DMA1_Stream1 = 12,          // DMA1 Stream 1 global Interrupt
        DMA1_Stream2 = 13,          // DMA1 Stream 2 global Interrupt
        DMA1_Stream3 = 14,          // DMA1 Stream 3 global Interrupt
        DMA1_Stream4 = 15,          // DMA1 Stream 4 global Interrupt
        DMA1_Stream5 = 16,          // DMA1 Stream 5 global Interrupt
        DMA1_Stream6 = 17,          // DMA1 Stream 6 global Interrupt
        ADC = 18,                   // ADC1, ADC2 and ADC3 global Interrupts
        EXTI9_5 = 23,               // External Line[9:5] Interrupts
        TIM1_BRK_TIM9 = 24,         // TIM1 Break interrupt and TIM9 global interrupt
        TIM1_UP_TIM10 = 25,         // TIM1 Update Interrupt and TIM10 global interrupt
        TIM1_TRG_COM_TIM11 = 26,    // TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt
        TIM1_CC = 27,               // TIM1 Capture Compare Interrupt
        TIM2 = 28,                  // TIM2 global Interrupt
        TIM3 = 29,                  // TIM3 global Interrupt
        TIM4 = 30,                  // TIM4 global Interrupt
        I2C1_EV = 31,               // I2C1 Event Interrupt
        I2C1_ER = 32,               // I2C1 Error Interrupt
        I2C2_EV = 33,               // I2C2 Event Interrupt
        I2C2_ER = 34,               // I2C2 Error Interrupt
        SPI1 = 35,                  // SPI1 global Interrupt
        SPI2 = 36,                  // SPI2 global Interrupt
        USART1 = 37,                // USART1 global Interrupt
        USART2 = 38,                // USART2 global Interrupt
        EXTI15_10 = 40,             // External Line[15:10] Interrupts
        RTC_Alarm = 41,             // RTC Alarm (A and B) through EXTI Line Interrupt
        OTG_FS_WKUP = 42,           // USB OTG FS Wakeup through EXTI line interrupt
        DMA1_Stream7 = 47,          // DMA1 Stream7 Interrupt
        SDIO = 49,                  // SDIO global Interrupt
        TIM5 = 50,                  // TIM5 global Interrupt
        SPI3 = 51,                  // SPI3 global Interrupt
        DMA2_Stream0 = 56,          // DMA2 Stream 0 global Interrupt
        DMA2_Stream1 = 57,          // DMA2 Stream 1 global Interrupt
        DMA2_Stream2 = 58,          // DMA2 Stream 2 global Interrupt
        DMA2_Stream3 = 59,          // DMA2 Stream 3 global Interrupt
        DMA2_Stream4 = 60,          // DMA2 Stream 4 global Interrupt
        OTG_FS = 67,                // USB OTG FS global Interrupt
        DMA2_Stream5 = 68,          // DMA2 Stream 5 global interrupt
        DMA2_Stream6 = 69,          // DMA2 Stream 6 global interrupt
        DMA2_Stream7 = 70,          // DMA2 Stream 7 global interrupt
        USART6 = 71,                // USART6 global interrupt
        I2C3_EV = 72,               // I2C3 event interrupt
        I2C3_ER = 73,               // I2C3 error interrupt
        FPU = 81,                   // FPU global interrupt
        SPI4 = 84                   // SPI4 global Interrupt
    };

    // NVIC register structure
    struct Registers {
        volatile uint32_t ISER[8];      // Interrupt Set Enable Register
        uint32_t RESERVED0[24];
        volatile uint32_t ICER[8];      // Interrupt Clear Enable Register
        uint32_t RESERVED1[24];
        volatile uint32_t ISPR[8];      // Interrupt Set Pending Register
        uint32_t RESERVED2[24];
        volatile uint32_t ICPR[8];      // Interrupt Clear Pending Register
        uint32_t RESERVED3[24];
        volatile uint32_t IABR[8];      // Interrupt Active Bit Register
        uint32_t RESERVED4[56];
        volatile uint8_t IP[240];       // Interrupt Priority Register (8-bit)
        uint32_t RESERVED5[644];
        volatile uint32_t STIR;         // Software Trigger Interrupt Register
    };

    // NVIC priority levels
    constexpr uint8_t PRIORITY_HIGHEST = 0x00U;
    constexpr uint8_t PRIORITY_HIGH = 0x40U;
    constexpr uint8_t PRIORITY_MEDIUM = 0x80U;
    constexpr uint8_t PRIORITY_LOW = 0xC0U;

    // NVIC priority grouping
    constexpr uint32_t PRIORITYGROUP_0 = 0x00000007U; // 0 bits for pre-emption priority, 4 bits for subpriority
    constexpr uint32_t PRIORITYGROUP_1 = 0x00000006U; // 1 bits for pre-emption priority, 3 bits for subpriority
    constexpr uint32_t PRIORITYGROUP_2 = 0x00000005U; // 2 bits for pre-emption priority, 2 bits for subpriority
    constexpr uint32_t PRIORITYGROUP_3 = 0x00000004U; // 3 bits for pre-emption priority, 1 bits for subpriority
    constexpr uint32_t PRIORITYGROUP_4 = 0x00000003U; // 4 bits for pre-emption priority, 0 bits for subpriority

    // Access function for NVIC registers
    inline Registers* getRegisters() {
        return reinterpret_cast<Registers*>(NVIC_BASE);
    }
    
    // Helper functions for NVIC operations
    inline void enableIRQ(IRQn irq) {
        getRegisters()->ISER[static_cast<uint32_t>(irq) >> 5] = 
            (1UL << (static_cast<uint32_t>(irq) & 0x1F));
    }
    
    inline void disableIRQ(IRQn irq) {
        getRegisters()->ICER[static_cast<uint32_t>(irq) >> 5] = 
            (1UL << (static_cast<uint32_t>(irq) & 0x1F));
    }
    
    inline void setPriority(IRQn irq, uint32_t priority) {
        // Set priority for Cortex-M system interrupts
        if (static_cast<int32_t>(irq) < 0) {

            SCB::getRegisters()->SHP[(static_cast<uint32_t>(irq) & 0xFUL)-4UL] = 
                static_cast<uint8_t>((priority << (8U - 4)) & 0xFFU);
                SCB::getRegisters()->SHP[(static_cast<uint32_t>(irq) & 0xFUL)-4UL] = 
                static_cast<uint8_t>((priority << (8U - 4)) & 0xFFU);
        } else {
            // Set priority for device specific interrupts
            getRegisters()->IP[static_cast<uint32_t>(irq)] = 
                static_cast<uint8_t>((priority << (8U - 4)) & 0xFFU);
        }
    }
}

// -------------------- SCB Definitions --------------------

namespace SCB {
    // System Control Block register structure
    struct Registers {
        volatile uint32_t CPUID;        // CPUID Base Register
        volatile uint32_t ICSR;         // Interrupt Control and State Register
        volatile uint32_t VTOR;         // Vector Table Offset Register
        volatile uint32_t AIRCR;        // Application Interrupt and Reset Control Register
        volatile uint32_t SCR;          // System Control Register
        volatile uint32_t CCR;          // Configuration Control Register
        volatile uint8_t  SHP[12];      // System Handlers Priority Registers (8-bit)
        volatile uint32_t SHCSR;        // System Handler Control and State Register
        volatile uint32_t CFSR;         // Configurable Fault Status Register
        volatile uint32_t HFSR;         // HardFault Status Register
        volatile uint32_t DFSR;         // Debug Fault Status Register
        volatile uint32_t MMFAR;        // MemManage Fault Address Register
        volatile uint32_t BFAR;         // BusFault Address Register
        volatile uint32_t AFSR;         // Auxiliary Fault Status Register
        volatile uint32_t PFR[2];       // Processor Feature Register
        volatile uint32_t DFR;          // Debug Feature Register
        volatile uint32_t ADR;          // Auxiliary Feature Register
        volatile uint32_t MMFR[4];      // Memory Model Feature Register
        volatile uint32_t ISAR[5];      // Instruction Set Attributes Register
        uint32_t RESERVED0[5];
        volatile uint32_t CPACR;        // Coprocessor Access Control Register
    };

    // Access function for SCB registers
    inline Registers* getRegisters() {
        return reinterpret_cast<Registers*>(SCB_BASE);
    }
}

// -------------------- SysTick Definitions --------------------

namespace SysTick {
    // SysTick Timer register structure
    struct Registers {
        volatile uint32_t CTRL;         // SysTick Control and Status Register
        volatile uint32_t LOAD;         // SysTick Reload Value Register
        volatile uint32_t VAL;          // SysTick Current Value Register
        volatile uint32_t CALIB;        // SysTick Calibration Register
    };

    // SysTick Control Register bits
    constexpr uint32_t CTRL_ENABLE = (1UL << 0);
    constexpr uint32_t CTRL_TICKINT = (1UL << 1);
    constexpr uint32_t CTRL_CLKSOURCE = (1UL << 2);
    constexpr uint32_t CTRL_COUNTFLAG = (1UL << 16);

    // Access function for SysTick registers
    inline Registers* getRegisters() {
        return reinterpret_cast<Registers*>(SysTick_BASE);
    }
}

// -------------------- GPIO Definitions --------------------

namespace GPIO {
    // GPIO port base addresses
    constexpr uint32_t GPIOA_BASE = (AHB1PERIPH_BASE + 0x0000UL);
    constexpr uint32_t GPIOB_BASE = (AHB1PERIPH_BASE + 0x0400UL);
    constexpr uint32_t GPIOC_BASE = (AHB1PERIPH_BASE + 0x0800UL);
    constexpr uint32_t GPIOD_BASE = (AHB1PERIPH_BASE + 0x0C00UL);
    constexpr uint32_t GPIOE_BASE = (AHB1PERIPH_BASE + 0x1000UL);
    constexpr uint32_t GPIOH_BASE = (AHB1PERIPH_BASE + 0x1C00UL);

    // GPIO port identifiers
    enum class Port {
        PORTA = 0,
        PORTB = 1,
        PORTC = 2,
        PORTD = 3,
        PORTE = 4,
        PORTH = 5
    };

    // GPIO mode definitions
    enum class Mode : uint32_t {
        Input = 0x00,
        Output = 0x01,
        AlternateFunction = 0x02,
        Analog = 0x03
    };

    // GPIO output type definitions
    enum class OutputType : uint32_t {
        PushPull = 0x00,
        OpenDrain = 0x01
    };

    // GPIO pull-up/pull-down definitions
    enum class Pull : uint32_t {
        None = 0x00,
        PullUp = 0x01,
        PullDown = 0x02
    };

    // GPIO speed definitions
    enum class Speed : uint32_t {
        Low = 0x00,
        Medium = 0x01,
        High = 0x02,
        VeryHigh = 0x03
    };

    // GPIO alternate function definitions
    enum class AlternateFunction : uint32_t {
        AF0 = 0x00,
        AF1 = 0x01,
        AF2 = 0x02,
        AF3 = 0x03,
        AF4 = 0x04,
        AF5 = 0x05,
        AF6 = 0x06,
        AF7 = 0x07,
        AF8 = 0x08,
        AF9 = 0x09,
        AF10 = 0x0A,
        AF11 = 0x0B,
        AF12 = 0x0C,
        AF13 = 0x0D,
        AF14 = 0x0E,
        AF15 = 0x0F
    };

    // GPIO register structure
    struct Registers {
        volatile uint32_t MODER;     // GPIO port mode register
        volatile uint32_t OTYPER;    // GPIO port output type register
        volatile uint32_t OSPEEDR;   // GPIO port output speed register
        volatile uint32_t PUPDR;     // GPIO port pull-up/pull-down register
        volatile uint32_t IDR;       // GPIO port input data register
        volatile uint32_t ODR;       // GPIO port output data register
        volatile uint32_t BSRR;      // GPIO port bit set/reset register
        volatile uint32_t LCKR;      // GPIO port configuration lock register
        volatile uint32_t AFRL;      // GPIO alternate function low register
        volatile uint32_t AFRH;      // GPIO alternate function high register
    };

    // Get register structures for each GPIO port
    inline Registers* getPortA() {
        return reinterpret_cast<Registers*>(GPIOA_BASE);
    }

    inline Registers* getPortB() {
        return reinterpret_cast<Registers*>(GPIOB_BASE);
    }

    inline Registers* getPortC() {
        return reinterpret_cast<Registers*>(GPIOC_BASE);
    }

    inline Registers* getPortD() {
        return reinterpret_cast<Registers*>(GPIOD_BASE);
    }

    inline Registers* getPortE() {
        return reinterpret_cast<Registers*>(GPIOE_BASE);
    }

    inline Registers* getPortH() {
        return reinterpret_cast<Registers*>(GPIOH_BASE);
    }

    // Helper function to get port registers by port enum
    inline Registers* getPort(Port port) {
        switch (port) {
            case Port::PORTA: return getPortA();
            case Port::PORTB: return getPortB();
            case Port::PORTC: return getPortC();
            case Port::PORTD: return getPortD();
            case Port::PORTE: return getPortE();
            case Port::PORTH: return getPortH();
            default: return nullptr;
        }
    }

    // Helper functions for GPIO bit manipulation
    constexpr uint32_t getModeMask(uint8_t pin, Mode mode) {
        return (static_cast<uint32_t>(mode) << (pin * 2));
    }

    constexpr uint32_t getPinMask(uint8_t pin) {
        return (1UL << pin);
    }

    constexpr uint32_t getPinResetMask(uint8_t pin) {
        return (1UL << (pin + 16));
    }
}

// -------------------- RCC Definitions --------------------

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

    // Get RCC registers
    inline Registers* getRegisters() {
        return reinterpret_cast<Registers*>(RCC_BASE);
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
// -------------------- Timer Definitions --------------------

namespace TIM {
    // Timer base addresses
    constexpr uint32_t TIM1_BASE = (APB2PERIPH_BASE + 0x0000UL);
    constexpr uint32_t TIM2_BASE = (APB1PERIPH_BASE + 0x0000UL);
    constexpr uint32_t TIM3_BASE = (APB1PERIPH_BASE + 0x0400UL);
    constexpr uint32_t TIM4_BASE = (APB1PERIPH_BASE + 0x0800UL);
    constexpr uint32_t TIM5_BASE = (APB1PERIPH_BASE + 0x0C00UL);
    constexpr uint32_t TIM9_BASE = (APB2PERIPH_BASE + 0x4000UL);
    constexpr uint32_t TIM10_BASE = (APB2PERIPH_BASE + 0x4400UL);
    constexpr uint32_t TIM11_BASE = (APB2PERIPH_BASE + 0x4800UL);

    // Timer register structure
    struct Registers {
        volatile uint32_t CR1;           // Control register 1
        volatile uint32_t CR2;           // Control register 2
        volatile uint32_t SMCR;          // Slave mode control register
        volatile uint32_t DIER;          // DMA/Interrupt enable register
        volatile uint32_t SR;            // Status register
        volatile uint32_t EGR;           // Event generation register
        volatile uint32_t CCMR1;         // Capture/compare mode register 1
        volatile uint32_t CCMR2;         // Capture/compare mode register 2
        volatile uint32_t CCER;          // Capture/compare enable register
        volatile uint32_t CNT;           // Counter
        volatile uint32_t PSC;           // Prescaler
        volatile uint32_t ARR;           // Auto-reload register
        volatile uint32_t RCR;           // Repetition counter register
        volatile uint32_t CCR1;          // Capture/compare register 1
        volatile uint32_t CCR2;          // Capture/compare register 2
        volatile uint32_t CCR3;          // Capture/compare register 3
        volatile uint32_t CCR4;          // Capture/compare register 4
        volatile uint32_t BDTR;          // Break and dead-time register
        volatile uint32_t DCR;           // DMA control register
        volatile uint32_t DMAR;          // DMA address for full transfer
    };

    // Timer mode definitions
    enum class Mode {
        Basic,
        PWM,
        InputCapture,
        OutputCompare
    };

    // Timer channel definitions
    enum class Channel {
        Channel1 = 1,
        Channel2 = 2,
        Channel3 = 3,
        Channel4 = 4
    };

    // Timer clock division
    enum class ClockDivision {
        Div1 = 0,
        Div2 = 1,
        Div4 = 2
    };

    // Timer direction
    enum class Direction {
        Up = 0,
        Down = 1
    };

    // Timer alignment mode
    enum class Alignment {
        Edge = 0,
        Center1 = 1,
        Center2 = 2,
        Center3 = 3
    };

    // Timer output compare mode
    enum class OCMode {
        Frozen = 0,
        Active = 1,
        Inactive = 2,
        Toggle = 3,
        ForceInactive = 4,
        ForceActive = 5,
        PWM1 = 6,
        PWM2 = 7
    };

    // CR1 register bits
    enum class CR1 : uint32_t {
        CEN = (1UL << 0),         // Counter enable
        UDIS = (1UL << 1),        // Update disable
        URS = (1UL << 2),         // Update request source
        OPM = (1UL << 3),         // One-pulse mode
        DIR = (1UL << 4),         // Direction
        CMS_EDGE = (0UL << 5),    // Edge-aligned mode
        CMS_CENTER1 = (1UL << 5), // Center-aligned mode 1
        CMS_CENTER2 = (2UL << 5), // Center-aligned mode 2
        CMS_CENTER3 = (3UL << 5), // Center-aligned mode 3
        CMS_MSK = (3UL << 5),     // CMS mask
        ARPE = (1UL << 7),        // Auto-reload preload enable
        CKD_DIV1 = (0UL << 8),    // Clock division 1
        CKD_DIV2 = (1UL << 8),    // Clock division 2
        CKD_DIV4 = (2UL << 8),    // Clock division 4
        CKD_MSK = (3UL << 8)      // CKD mask
    };

    // DIER register bits
    enum class DIER : uint32_t {
        UIE = (1UL << 0),    // Update interrupt enable
        CC1IE = (1UL << 1),  // Capture/Compare 1 interrupt enable
        CC2IE = (1UL << 2),  // Capture/Compare 2 interrupt enable
        CC3IE = (1UL << 3),  // Capture/Compare 3 interrupt enable
        CC4IE = (1UL << 4),  // Capture/Compare 4 interrupt enable
        COMIE = (1UL << 5),  // COM interrupt enable
        TIE = (1UL << 6),    // Trigger interrupt enable
        BIE = (1UL << 7),    // Break interrupt enable
        UDE = (1UL << 8),    // Update DMA request enable
        CC1DE = (1UL << 9),  // Capture/Compare 1 DMA request enable
        CC2DE = (1UL << 10), // Capture/Compare 2 DMA request enable
        CC3DE = (1UL << 11), // Capture/Compare 3 DMA request enable
        CC4DE = (1UL << 12), // Capture/Compare 4 DMA request enable
        COMDE = (1UL << 13), // COM DMA request enable
        TDE = (1UL << 14)    // Trigger DMA request enable
    };

    // SR register bits
    enum class SR : uint32_t {
        UIF = (1UL << 0),    // Update interrupt flag
        CC1IF = (1UL << 1),  // Capture/Compare 1 interrupt flag
        CC2IF = (1UL << 2),  // Capture/Compare 2 interrupt flag
        CC3IF = (1UL << 3),  // Capture/Compare 3 interrupt flag
        CC4IF = (1UL << 4),  // Capture/Compare 4 interrupt flag
        COMIF = (1UL << 5),  // COM interrupt flag
        TIF = (1UL << 6),    // Trigger interrupt flag
        BIF = (1UL << 7),    // Break interrupt flag
        CC1OF = (1UL << 9),  // Capture/Compare 1 overcapture flag
        CC2OF = (1UL << 10), // Capture/Compare 2 overcapture flag
        CC3OF = (1UL << 11), // Capture/Compare 3 overcapture flag
        CC4OF = (1UL << 12)  // Capture/Compare 4 overcapture flag
    };

    // EGR register bits
    enum class EGR : uint32_t {
        UG = (1UL << 0),     // Update generation
        CC1G = (1UL << 1),   // Capture/Compare 1 generation
        CC2G = (1UL << 2),   // Capture/Compare 2 generation
        CC3G = (1UL << 3),   // Capture/Compare 3 generation
        CC4G = (1UL << 4),   // Capture/Compare 4 generation
        COMG = (1UL << 5),   // Capture/Compare control update generation
        TG = (1UL << 6),     // Trigger generation
        BG = (1UL << 7)      // Break generation
    };

    // Access functions for timer registers
    inline Registers* getTIM1() {
        return reinterpret_cast<Registers*>(TIM1_BASE);
    }

    inline Registers* getTIM2() {
        return reinterpret_cast<Registers*>(TIM2_BASE);
    }

    inline Registers* getTIM3() {
        return reinterpret_cast<Registers*>(TIM3_BASE);
    }

    inline Registers* getTIM4() {
        return reinterpret_cast<Registers*>(TIM4_BASE);
    }

    inline Registers* getTIM5() {
        return reinterpret_cast<Registers*>(TIM5_BASE);
    }

    inline Registers* getTIM9() {
        return reinterpret_cast<Registers*>(TIM9_BASE);
    }

    inline Registers* getTIM10() {
        return reinterpret_cast<Registers*>(TIM10_BASE);
    }

    inline Registers* getTIM11() {
        return reinterpret_cast<Registers*>(TIM11_BASE);
    }

    // Helper function to get timer registers by timer number
    inline Registers* getTimer(uint8_t timerNumber) {
        switch (timerNumber) {
            case 1: return getTIM1();
            case 2: return getTIM2();
            case 3: return getTIM3();
            case 4: return getTIM4();
            case 5: return getTIM5();
            case 9: return getTIM9();
            case 10: return getTIM10();
            case 11: return getTIM11();
            default: return nullptr;
        }
    }

    // Helper function for bit manipulation
    constexpr uint32_t getBitValue(CR1 bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(DIER bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(SR bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(EGR bit) {
        return static_cast<uint32_t>(bit);
    }

    // Operator overloads for combining flags
    constexpr CR1 operator|(CR1 a, CR1 b) {
        return static_cast<CR1>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr DIER operator|(DIER a, DIER b) {
        return static_cast<DIER>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr SR operator|(SR a, SR b) {
        return static_cast<SR>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr EGR operator|(EGR a, EGR b) {
        return static_cast<EGR>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }
}

// -------------------- I2C Definitions --------------------

namespace I2C {
    // I2C base addresses
    constexpr uint32_t I2C1_BASE = (APB1PERIPH_BASE + 0x5400UL);
    constexpr uint32_t I2C2_BASE = (APB1PERIPH_BASE + 0x5800UL);
    constexpr uint32_t I2C3_BASE = (APB1PERIPH_BASE + 0x5C00UL);

    // I2C register structure
    struct Registers {
        volatile uint32_t CR1;        // Control register 1
        volatile uint32_t CR2;        // Control register 2
        volatile uint32_t OAR1;       // Own address register 1
        volatile uint32_t OAR2;       // Own address register 2
        volatile uint32_t DR;         // Data register
        volatile uint32_t SR1;        // Status register 1
        volatile uint32_t SR2;        // Status register 2
        volatile uint32_t CCR;        // Clock control register
        volatile uint32_t TRISE;      // TRISE register
        volatile uint32_t FLTR;       // FLTR register
    };

    // I2C addressing mode
    enum class AddrMode {
        Addr7Bit,
        Addr10Bit
    };

    // I2C speed mode
    enum class Speed {
        Standard,  // 100 KHz
        Fast       // 400 KHz
    };

    // I2C mode
    enum class Mode {
        Master,
        Slave
    };

    // I2C transfer state
    enum class TransferState {
        Idle,
        StartSent,
        RepeatedStartSent,
        AddrSentW,
        AddrSentR,
        RegAddrSent,
        Writing,
        Reading,
        Error
    };

    // I2C operation type
    enum class Operation {
        None,
        Read,
        Write
    };

    // CR1 register bits
    enum class CR1 : uint32_t {
        PE = (1UL << 0),        // Peripheral enable
        SMBUS = (1UL << 1),     // SMBus mode
        SMBTYPE = (1UL << 3),   // SMBus type
        ENARP = (1UL << 4),     // ARP enable
        ENPEC = (1UL << 5),     // PEC enable
        ENGC = (1UL << 6),      // General call enable
        NOSTRETCH = (1UL << 7), // Clock stretching disable
        START = (1UL << 8),     // Start generation
        STOP = (1UL << 9),      // Stop generation
        ACK = (1UL << 10),      // Acknowledge enable
        POS = (1UL << 11),      // Acknowledge/PEC position
        PEC = (1UL << 12),      // Packet error checking
        ALERT = (1UL << 13),    // SMBus alert
        SWRST = (1UL << 15)     // Software reset
    };

    // CR2 register bits
    enum class CR2 : uint32_t {
        FREQ_MSK = (0x3FUL << 0),  // Peripheral clock frequency mask
        ITERREN = (1UL << 8),      // Error interrupt enable
        ITEVTEN = (1UL << 9),      // Event interrupt enable
        ITBUFEN = (1UL << 10),     // Buffer interrupt enable
        DMAEN = (1UL << 11),       // DMA requests enable
        LAST = (1UL << 12)         // DMA last transfer
    };

    // SR1 register bits
    enum class SR1 : uint32_t {
        SB = (1UL << 0),        // Start bit (Master mode)
        ADDR = (1UL << 1),      // Address sent (master mode)/matched (slave mode)
        BTF = (1UL << 2),       // Byte transfer finished
        ADD10 = (1UL << 3),     // 10-bit header sent (Master mode)
        STOPF = (1UL << 4),     // Stop detection (slave mode)
        RXNE = (1UL << 6),      // Data register not empty (receivers)
        TXE = (1UL << 7),       // Data register empty (transmitters)
        BERR = (1UL << 8),      // Bus error
        ARLO = (1UL << 9),      // Arbitration lost (master mode)
        AF = (1UL << 10),       // Acknowledge failure
        OVR = (1UL << 11),      // Overrun/Underrun
        PECERR = (1UL << 12),   // PEC Error in reception
        TIMEOUT = (1UL << 14),  // Timeout or Tlow error
        SMBALERT = (1UL << 15)  // SMBus alert
    };

    // SR2 register bits
    enum class SR2 : uint32_t {
        MSL = (1UL << 0),         // Master/slave
        BUSY = (1UL << 1),        // Bus busy
        TRA = (1UL << 2),         // Transmitter/receiver
        GENCALL = (1UL << 4),     // General call address (slave mode)
        SMBDEFAULT = (1UL << 5),  // SMBus device default address (slave mode)
        SMBHOST = (1UL << 6),     // SMBus host header (slave mode)
        DUALF = (1UL << 7),       // Dual flag (slave mode)
        PEC_MSK = (0xFFUL << 8)   // Packet error checking register mask
    };

    // CCR register bits
    enum class CCR : uint32_t {
        CCR_MSK = (0xFFFUL << 0),  // Clock control register mask
        DUTY = (1UL << 14),        // Fast mode duty cycle
        FS = (1UL << 15)           // I2C master mode selection
    };

    // Access functions for I2C registers
    inline Registers* getI2C1() {
        return reinterpret_cast<Registers*>(I2C1_BASE);
    }

    inline Registers* getI2C2() {
        return reinterpret_cast<Registers*>(I2C2_BASE);
    }

    inline Registers* getI2C3() {
        return reinterpret_cast<Registers*>(I2C3_BASE);
    }

    // Helper function to get I2C registers by instance number
    inline Registers* getI2C(uint8_t instance) {
        switch (instance) {
            case 1: return getI2C1();
            case 2: return getI2C2();
            case 3: return getI2C3();
            default: return nullptr;
        }
    }

    // Helper functions for bit manipulation
    constexpr uint32_t getBitValue(CR1 bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(CR2 bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(SR1 bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(SR2 bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(CCR bit) {
        return static_cast<uint32_t>(bit);
    }

    // Operator overloads for combining flags
    constexpr CR1 operator|(CR1 a, CR1 b) {
        return static_cast<CR1>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr CR2 operator|(CR2 a, CR2 b) {
        return static_cast<CR2>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr SR1 operator|(SR1 a, SR1 b) {
        return static_cast<SR1>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr SR2 operator|(SR2 a, SR2 b) {
        return static_cast<SR2>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr CCR operator|(CCR a, CCR b) {
        return static_cast<CCR>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }
}

// -------------------- ADC Definitions --------------------

namespace ADC {
    constexpr uint32_t ADC1_BASE = (APB2PERIPH_BASE + 0x2000UL);
    constexpr uint32_t ADC_COMMON_BASE = (ADC1_BASE + 0x300UL);

    // ADC channel definitions
    enum class Channel : uint32_t {
        Channel0 = 0UL,
        Channel1 = 1UL,
        Channel2 = 2UL,
        Channel3 = 3UL,
        Channel4 = 4UL,
        Channel5 = 5UL,
        Channel6 = 6UL,
        Channel7 = 7UL,
        Channel8 = 8UL,
        Channel9 = 9UL,
        Channel10 = 10UL,
        Channel11 = 11UL,
        Channel12 = 12UL,
        Channel13 = 13UL,
        Channel14 = 14UL,
        Channel15 = 15UL,
        Channel16 = 16UL,      // Temperature sensor
        Channel17 = 17UL,      // VREFINT
        Channel18 = 18UL       // VBAT
    };

    // Sample time definitions
    enum class SampleTime : uint32_t {
        Cycles3 = 0UL,
        Cycles15 = 1UL,
        Cycles28 = 2UL,
        Cycles56 = 3UL,
        Cycles84 = 4UL,
        Cycles112 = 5UL,
        Cycles144 = 6UL,
        Cycles480 = 7UL
    };

    // Resolution definitions
    enum class Resolution : uint32_t {
        Bits12 = 0UL,
        Bits10 = 1UL,
        Bits8 = 2UL,
        Bits6 = 3UL
    };

    // Data alignment
    enum class Alignment : uint32_t {
        Right = 0UL,
        Left = 1UL
    };

    // External trigger edge
    enum class ExternalTriggerEdge : uint32_t {
        Disabled = 0UL,
        RisingEdge = 1UL,
        FallingEdge = 2UL,
        BothEdges = 3UL
    };

    // External trigger sources for regular group
    enum class ExternalTrigger : uint32_t {
        Timer1_CC1 = 0UL,
        Timer1_CC2 = 1UL,
        Timer1_CC3 = 2UL,
        Timer2_CC2 = 3UL,
        Timer2_CC3 = 4UL,
        Timer2_CC4 = 5UL,
        Timer2_TRGO = 6UL,
        Timer3_CC1 = 7UL,
        Timer3_TRGO = 8UL,
        Timer4_CC4 = 9UL,
        Timer5_CC1 = 10UL,
        Timer5_CC2 = 11UL,
        Timer5_CC3 = 12UL,
        Timer8_CC1 = 13UL,
        Timer8_TRGO = 14UL,
        EXTI_Line11 = 15UL
    };

    // ADC register structure
    struct ADC_Registers {
        volatile uint32_t SR;           // ADC status register
        volatile uint32_t CR1;          // ADC control register 1
        volatile uint32_t CR2;          // ADC control register 2
        volatile uint32_t SMPR1;        // ADC sample time register 1
        volatile uint32_t SMPR2;        // ADC sample time register 2
        volatile uint32_t JOFR1;        // ADC injected channel data offset register 1
        volatile uint32_t JOFR2;        // ADC injected channel data offset register 2
        volatile uint32_t JOFR3;        // ADC injected channel data offset register 3
        volatile uint32_t JOFR4;        // ADC injected channel data offset register 4
        volatile uint32_t HTR;          // ADC watchdog higher threshold register
        volatile uint32_t LTR;          // ADC watchdog lower threshold register
        volatile uint32_t SQR1;         // ADC regular sequence register 1
        volatile uint32_t SQR2;         // ADC regular sequence register 2
        volatile uint32_t SQR3;         // ADC regular sequence register 3
        volatile uint32_t JSQR;         // ADC injected sequence register
        volatile uint32_t JDR1;         // ADC injected data register 1
        volatile uint32_t JDR2;         // ADC injected data register 2
        volatile uint32_t JDR3;         // ADC injected data register 3
        volatile uint32_t JDR4;         // ADC injected data register 4
        volatile uint32_t DR;           // ADC regular data register
    };

    // ADC common registers structure
    struct ADC_Common_Registers {
        volatile uint32_t CSR;          // ADC common status register
        volatile uint32_t CCR;          // ADC common control register
        volatile uint32_t CDR;          // ADC common regular data register for dual and triple modes
    };

    // ADC_SR register bits
    enum class SR : uint32_t {
        AWD = (1UL << 0),        // Analog watchdog flag
        EOC = (1UL << 1),        // End of conversion
        JEOC = (1UL << 2),       // End of injected conversion
        JSTRT = (1UL << 3),      // Injected channel start flag
        STRT = (1UL << 4),       // Regular channel start flag
        OVR = (1UL << 5)         // Overrun
    };

    // ADC_CR1 register bits
    enum class CR1 : uint32_t {
        AWDCH_MSK = (0x1FUL << 0),   // Analog watchdog channel select mask
        EOCIE = (1UL << 5),          // Interrupt enable for EOC
        AWDIE = (1UL << 6),          // Analog watchdog interrupt enable
        JEOCIE = (1UL << 7),         // Interrupt enable for JEOC
        SCAN = (1UL << 8),           // Scan mode
        AWDSGL = (1UL << 9),         // Enable watchdog on a single channel in scan mode
        JAUTO = (1UL << 10),         // Automatic injected group conversion
        DISCEN = (1UL << 11),        // Discontinuous mode on regular channels
        JDISCEN = (1UL << 12),       // Discontinuous mode on injected channels
        DISCNUM_MSK = (0x7UL << 13), // Discontinuous mode channel count mask
        JAWDEN = (1UL << 22),        // Analog watchdog enable on injected channels
        AWDEN = (1UL << 23),         // Analog watchdog enable on regular channels
        RES_12BIT = (0UL << 24),     // 12-bit resolution
        RES_10BIT = (1UL << 24),     // 10-bit resolution
        RES_8BIT = (2UL << 24),      // 8-bit resolution
        RES_6BIT = (3UL << 24),      // 6-bit resolution
        RES_MSK = (3UL << 24),       // Resolution mask
        OVRIE = (1UL << 26)          // Overrun interrupt enable
    };

    // ADC_CR2 register bits
    enum class CR2 : uint32_t {
        ADON = (1UL << 0),           // A/D converter ON / OFF
        CONT = (1UL << 1),           // Continuous conversion
        DMA = (1UL << 8),            // Direct memory access mode
        DDS = (1UL << 9),            // DMA disable selection
        EOCS = (1UL << 10),          // End of conversion selection
        ALIGN = (1UL << 11),         // Data alignment
        JEXTSEL_MSK = (0xFUL << 16), // External event select for injected group mask
        JEXTEN_MSK = (0x3UL << 20),  // External trigger enable for injected channels mask
        JSWSTART = (1UL << 22),      // Start conversion of injected channels
        EXTSEL_MSK = (0xFUL << 24),  // External event select for regular group mask
        EXTEN_MSK = (0x3UL << 28),   // External trigger enable for regular channels mask
        SWSTART = (1UL << 30)        // Start conversion of regular channels
    };

    // ADC_CCR register bits (Common)
    enum class CCR : uint32_t {
        ADCPRE_DIV2 = (0UL << 16),  // PCLK2 divided by 2
        ADCPRE_DIV4 = (1UL << 16),  // PCLK2 divided by 4
        ADCPRE_DIV6 = (2UL << 16),  // PCLK2 divided by 6
        ADCPRE_DIV8 = (3UL << 16),  // PCLK2 divided by 8
        ADCPRE_MSK = (3UL << 16),   // ADC prescaler mask
        VBATE = (1UL << 22),        // VBAT enable
        TSVREFE = (1UL << 23)       // Temperature sensor and VREFINT enable
    };

    // Helper methods for configuring sample time registers
    constexpr uint32_t getSMPR1SampleTimeBits(Channel channel, SampleTime time) {
        // SMPR1 handles channels 10-18
        uint8_t chIdx = static_cast<uint8_t>(channel);
        if (chIdx < 10 || chIdx > 18) {
            return 0; // Not in this register
        }
        // Each channel has 3 bits, positioned starting from LSB
        uint8_t position = (chIdx - 10) * 3;
        return (static_cast<uint32_t>(time) << position);
    }

    constexpr uint32_t getSMPR2SampleTimeBits(Channel channel, SampleTime time) {
        // SMPR2 handles channels 0-9
        uint8_t chIdx = static_cast<uint8_t>(channel);
        if (chIdx > 9) {
            return 0; // Not in this register
        }
        // Each channel has 3 bits, positioned starting from LSB
        uint8_t position = chIdx * 3;
        return (static_cast<uint32_t>(time) << position);
    }

    // Helper methods for configuring regular sequence registers
    constexpr uint32_t setSQR1SequenceBits(uint8_t sqr, Channel channel) {
        // SQR1 handles 13th-16th conversion in sequence
        if (sqr < 13 || sqr > 16) {
            return 0; // Not in this register
        }
        uint8_t position = (16 - sqr) * 5; // Positions from right to left
        return (static_cast<uint32_t>(channel) << position);
    }

    constexpr uint32_t setSQR2SequenceBits(uint8_t sqr, Channel channel) {
        // SQR2 handles 7th-12th conversion in sequence
        if (sqr < 7 || sqr > 12) {
            return 0; // Not in this register
        }
        uint8_t position = (12 - sqr) * 5; // Positions from right to left
        return (static_cast<uint32_t>(channel) << position);
    }

    constexpr uint32_t setSQR3SequenceBits(uint8_t sqr, Channel channel) {
        // SQR3 handles 1st-6th conversion in sequence
        if (sqr < 1 || sqr > 6) {
            return 0; // Not in this register
        }
        uint8_t position = (6 - sqr) * 5; // Positions from right to left
        return (static_cast<uint32_t>(channel) << position);
    }

    // Set sequence length (1-16 conversions) in SQR1
    constexpr uint32_t setSequenceLength(uint8_t length) {
        // Length is stored in bits 20-23 of SQR1
        // Value stored is length-1
        if (length < 1 || length > 16) {
            return 0; // Invalid length
        }
        return ((length - 1) << 20);
    }

    // Get ADC registers
    inline ADC_Registers* getADC1Registers() {
        return reinterpret_cast<ADC_Registers*>(ADC1_BASE);
    }

    // Get ADC common registers
    inline ADC_Common_Registers* getCommonRegisters() {
        return reinterpret_cast<ADC_Common_Registers*>(ADC_COMMON_BASE);
    }

    // Helper functions for bit manipulation
    constexpr uint32_t getBitValue(SR bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(CR1 bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(CR2 bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(CCR bit) {
        return static_cast<uint32_t>(bit);
    }

    // Operator overloads for combining flags
    constexpr SR operator|(SR a, SR b) {
        return static_cast<SR>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr CR1 operator|(CR1 a, CR1 b) {
        return static_cast<CR1>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr CR2 operator|(CR2 a, CR2 b) {
        return static_cast<CR2>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr CCR operator|(CCR a, CCR b) {
        return static_cast<CCR>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }
}

// -------------------- DMA Definitions --------------------

namespace DMA {
    constexpr uint32_t DMA1_BASE = (AHB1PERIPH_BASE + 0x6000UL);
    constexpr uint32_t DMA2_BASE = (AHB1PERIPH_BASE + 0x6400UL);
    
    // DMA stream register structure
    struct DMA_Stream {
        volatile uint32_t CR;           // DMA stream x configuration register
        volatile uint32_t NDTR;         // DMA stream x number of data register
        volatile uint32_t PAR;          // DMA stream x peripheral address register
        volatile uint32_t M0AR;         // DMA stream x memory 0 address register
        volatile uint32_t M1AR;         // DMA stream x memory 1 address register
        volatile uint32_t FCR;          // DMA stream x FIFO control register
    };

    // DMA register structure
    struct Registers {
        volatile uint32_t LISR;          // DMA low interrupt status register
        volatile uint32_t HISR;          // DMA high interrupt status register
        volatile uint32_t LIFCR;         // DMA low interrupt flag clear register
        volatile uint32_t HIFCR;         // DMA high interrupt flag clear register
        DMA_Stream   STREAM[8];          // DMA streams
    };

    // Stream identifier (for easier use in APIs)
    enum class Stream : uint32_t {
        Stream0 = 0,
        Stream1 = 1,
        Stream2 = 2,
        Stream3 = 3,
        Stream4 = 4,
        Stream5 = 5,
        Stream6 = 6,
        Stream7 = 7
    };

    // DMA channel selection (for multiplexed requests)
    enum class Channel : uint32_t {
        Channel0 = 0UL << 25,
        Channel1 = 1UL << 25,
        Channel2 = 2UL << 25,
        Channel3 = 3UL << 25,
        Channel4 = 4UL << 25,
        Channel5 = 5UL << 25,
        Channel6 = 6UL << 25,
        Channel7 = 7UL << 25,
        ChannelMask = 7UL << 25
    };

    // Data transfer direction
    enum class Direction : uint32_t {
        PeripheralToMemory = 0UL << 6,
        MemoryToPeripheral = 1UL << 6,
        MemoryToMemory = 2UL << 6,
        DirectionMask = 3UL << 6
    };

    // Data item size
    enum class DataSize : uint32_t {
        Byte = 0UL << 11,      // 8-bit
        HalfWord = 1UL << 11,  // 16-bit
        Word = 2UL << 11,      // 32-bit
        SizeMask = 3UL << 11
    };

    // Memory burst transfer configuration
    enum class MemoryBurst : uint32_t {
        Single = 0UL << 23,
        Incr4 = 1UL << 23,
        Incr8 = 2UL << 23,
        Incr16 = 3UL << 23,
        BurstMask = 3UL << 23
    };

    // Peripheral burst transfer configuration
    enum class PeripheralBurst : uint32_t {
        Single = 0UL << 21,
        Incr4 = 1UL << 21,
        Incr8 = 2UL << 21,
        Incr16 = 3UL << 21,
        BurstMask = 3UL << 21
    };

    // FIFO threshold level
    enum class FIFOThreshold : uint32_t {
        Quarter = 0UL << 0,
        Half = 1UL << 0,
        ThreeQuarters = 2UL << 0,
        Full = 3UL << 0,
        ThresholdMask = 3UL << 0
    };

    // Priority level
    enum class Priority : uint32_t {
        Low = 0UL << 16,
        Medium = 1UL << 16,
        High = 2UL << 16,
        VeryHigh = 3UL << 16,
        PriorityMask = 3UL << 16
    };

    // DMA_SxCR register bits
    enum class CR : uint32_t {
        EN = (1UL << 0),              // Stream enable
        DMEIE = (1UL << 1),           // Direct mode error interrupt enable
        TEIE = (1UL << 2),            // Transfer error interrupt enable
        HTIE = (1UL << 3),            // Half transfer interrupt enable
        TCIE = (1UL << 4),            // Transfer complete interrupt enable
        PFCTRL = (1UL << 5),          // Peripheral flow controller
        DIR_P2M = (0UL << 6),         // Peripheral to memory
        DIR_M2P = (1UL << 6),         // Memory to peripheral
        DIR_M2M = (2UL << 6),         // Memory to memory
        DIR_MSK = (3UL << 6),         // Direction mask
        CIRC = (1UL << 8),            // Circular mode
        PINC = (1UL << 9),            // Peripheral increment mode
        MINC = (1UL << 10),           // Memory increment mode
        PSIZE_8BIT = (0UL << 11),     // Peripheral data size: Byte (8-bits)
        PSIZE_16BIT = (1UL << 11),    // Peripheral data size: Half-word (16-bits)
        PSIZE_32BIT = (2UL << 11),    // Peripheral data size: Word (32-bits)
        PSIZE_MSK = (3UL << 11),      // Peripheral data size mask
        MSIZE_8BIT = (0UL << 13),     // Memory data size: Byte (8-bits)
        MSIZE_16BIT = (1UL << 13),    // Memory data size: Half-word (16-bits)
        MSIZE_32BIT = (2UL << 13),    // Memory data size: Word (32-bits)
        MSIZE_MSK = (3UL << 13),      // Memory data size mask
        PINCOS = (1UL << 15),         // Peripheral increment offset size
        PL_LOW = (0UL << 16),         // Priority level: Low
        PL_MEDIUM = (1UL << 16),      // Priority level: Medium
        PL_HIGH = (2UL << 16),        // Priority level: High
        PL_VERY_HIGH = (3UL << 16),   // Priority level: Very high
        PL_MSK = (3UL << 16),         // Priority level mask
        DBM = (1UL << 18),            // Double buffer mode
        CT = (1UL << 19),             // Current target (only in double buffer mode)
        PBURST_SINGLE = (0UL << 21),  // Peripheral burst: Single transfer
        PBURST_INCR4 = (1UL << 21),   // Peripheral burst: Incremental burst of 4 beats
        PBURST_INCR8 = (2UL << 21),   // Peripheral burst: Incremental burst of 8 beats
        PBURST_INCR16 = (3UL << 21),  // Peripheral burst: Incremental burst of 16 beats
        PBURST_MSK = (3UL << 21),     // Peripheral burst mask
        MBURST_SINGLE = (0UL << 23),  // Memory burst: Single transfer
        MBURST_INCR4 = (1UL << 23),   // Memory burst: Incremental burst of 4 beats
        MBURST_INCR8 = (2UL << 23),   // Memory burst: Incremental burst of 8 beats
        MBURST_INCR16 = (3UL << 23),  // Memory burst: Incremental burst of 16 beats
        MBURST_MSK = (3UL << 23),     // Memory burst mask
        CHSEL_0 = (0UL << 25),        // Channel 0 selection
        CHSEL_1 = (1UL << 25),        // Channel 1 selection
        CHSEL_2 = (2UL << 25),        // Channel 2 selection
        CHSEL_3 = (3UL << 25),        // Channel 3 selection
        CHSEL_4 = (4UL << 25),        // Channel 4 selection
        CHSEL_5 = (5UL << 25),        // Channel 5 selection
        CHSEL_6 = (6UL << 25),        // Channel 6 selection
        CHSEL_7 = (7UL << 25),        // Channel 7 selection
        CHSEL_MSK = (7UL << 25)       // Channel selection mask
    };

    // DMA_SxFCR register bits
    enum class FCR : uint32_t {
        FTH_1_4 = (0UL << 0),          // FIFO threshold: 1/4 full
        FTH_1_2 = (1UL << 0),          // FIFO threshold: 1/2 full
        FTH_3_4 = (2UL << 0),          // FIFO threshold: 3/4 full
        FTH_FULL = (3UL << 0),         // FIFO threshold: Full
        FTH_MSK = (3UL << 0),          // FIFO threshold mask
        DMDIS = (1UL << 2),            // Direct mode disable
        FS_LT_1_4 = (0UL << 3),        // FIFO status: Less than 1/4 full
        FS_1_4_TO_1_2 = (1UL << 3),    // FIFO status: 1/4 to 1/2 full
        FS_1_2_TO_3_4 = (2UL << 3),    // FIFO status: 1/2 to 3/4 full
        FS_3_4_TO_FULL = (3UL << 3),   // FIFO status: 3/4 full to full
        FS_EMPTY = (4UL << 3),         // FIFO status: Empty
        FS_FULL = (5UL << 3),          // FIFO status: Full
        FS_MSK = (7UL << 3),           // FIFO status mask
        FEIE = (1UL << 7)              // FIFO error interrupt enable
    };

    // DMA LISR/HISR register bits layout (per stream)
    enum class ISR_Bits : uint32_t {
        FEIF0 = (1UL << 0),           // Stream 0 FIFO error interrupt flag
        DMEIF0 = (1UL << 2),          // Stream 0 direct mode error interrupt flag
        TEIF0 = (1UL << 3),           // Stream 0 transfer error interrupt flag
        HTIF0 = (1UL << 4),           // Stream 0 half transfer interrupt flag
        TCIF0 = (1UL << 5),           // Stream 0 transfer complete interrupt flag
        FEIF1 = (1UL << 6),           // Stream 1 FIFO error interrupt flag
        DMEIF1 = (1UL << 8),          // Stream 1 direct mode error interrupt flag
        TEIF1 = (1UL << 9),           // Stream 1 transfer error interrupt flag
        HTIF1 = (1UL << 10),          // Stream 1 half transfer interrupt flag
        TCIF1 = (1UL << 11),          // Stream 1 transfer complete interrupt flag
        FEIF2 = (1UL << 16),          // Stream 2 FIFO error interrupt flag
        DMEIF2 = (1UL << 18),         // Stream 2 direct mode error interrupt flag
        TEIF2 = (1UL << 19),          // Stream 2 transfer error interrupt flag
        HTIF2 = (1UL << 20),          // Stream 2 half transfer interrupt flag
        TCIF2 = (1UL << 21),          // Stream 2 transfer complete interrupt flag
        FEIF3 = (1UL << 22),          // Stream 3 FIFO error interrupt flag
        DMEIF3 = (1UL << 24),         // Stream 3 direct mode error interrupt flag
        TEIF3 = (1UL << 25),          // Stream 3 transfer error interrupt flag
        HTIF3 = (1UL << 26),          // Stream 3 half transfer interrupt flag
        TCIF3 = (1UL << 27)           // Stream 3 transfer complete interrupt flag
    };

    // DMA LIFCR/HIFCR register bits layout (per stream)
    enum class IFCR_Bits : uint32_t {
        CFEIF0 = (1UL << 0),          // Stream 0 clear FIFO error interrupt flag
        CDMEIF0 = (1UL << 2),         // Stream 0 clear direct mode error interrupt flag
        CTEIF0 = (1UL << 3),          // Stream 0 clear transfer error interrupt flag
        CHTIF0 = (1UL << 4),          // Stream 0 clear half transfer interrupt flag
        CTCIF0 = (1UL << 5),          // Stream 0 clear transfer complete interrupt flag
        CFEIF1 = (1UL << 6),          // Stream 1 clear FIFO error interrupt flag
        CDMEIF1 = (1UL << 8),         // Stream 1 clear direct mode error interrupt flag
        CTEIF1 = (1UL << 9),          // Stream 1 clear transfer error interrupt flag
        CHTIF1 = (1UL << 10),         // Stream 1 clear half transfer interrupt flag
        CTCIF1 = (1UL << 11),         // Stream 1 clear transfer complete interrupt flag
        CFEIF2 = (1UL << 16),         // Stream 2 clear FIFO error interrupt flag
        CDMEIF2 = (1UL << 18),        // Stream 2 clear direct mode error interrupt flag
        CTEIF2 = (1UL << 19),         // Stream 2 clear transfer error interrupt flag
        CHTIF2 = (1UL << 20),         // Stream 2 clear half transfer interrupt flag
        CTCIF2 = (1UL << 21),         // Stream 2 clear transfer complete interrupt flag
        CFEIF3 = (1UL << 22),         // Stream 3 clear FIFO error interrupt flag
        CDMEIF3 = (1UL << 24),        // Stream 3 clear direct mode error interrupt flag
        CTEIF3 = (1UL << 25),         // Stream 3 clear transfer error interrupt flag
        CHTIF3 = (1UL << 26),         // Stream 3 clear half transfer interrupt flag
        CTCIF3 = (1UL << 27)          // Stream 3 clear transfer complete interrupt flag
    };

    // Get DMA registers
    inline Registers* getDMA1Registers() {
        return reinterpret_cast<Registers*>(DMA1_BASE);
    }

    inline Registers* getDMA2Registers() {
        return reinterpret_cast<Registers*>(DMA2_BASE);
    }

    // Get specific stream configuration register
    inline DMA_Stream* getStream(int controller, Stream stream) {
        if (controller == 1) {
            return &(getDMA1Registers()->STREAM[static_cast<uint32_t>(stream)]);
        } else if (controller == 2) {
            return &(getDMA2Registers()->STREAM[static_cast<uint32_t>(stream)]);
        }
        return nullptr;
    }

    // Helper functions for bit manipulation
    constexpr uint32_t getBitValue(CR bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(FCR bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(ISR_Bits bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(IFCR_Bits bit) {
        return static_cast<uint32_t>(bit);
    }

    constexpr uint32_t getBitValue(Channel channel) {
        return static_cast<uint32_t>(channel);
    }

    constexpr uint32_t getBitValue(Direction direction) {
        return static_cast<uint32_t>(direction);
    }

    constexpr uint32_t getBitValue(DataSize size) {
        return static_cast<uint32_t>(size);
    }

    constexpr uint32_t getBitValue(Priority priority) {
        return static_cast<uint32_t>(priority);
    }

    // Operator overloads for combining flags
    constexpr CR operator|(CR a, CR b) {
        return static_cast<CR>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr FCR operator|(FCR a, FCR b) {
        return static_cast<FCR>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr ISR_Bits operator|(ISR_Bits a, ISR_Bits b) {
        return static_cast<ISR_Bits>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    constexpr IFCR_Bits operator|(IFCR_Bits a, IFCR_Bits b) {
        return static_cast<IFCR_Bits>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }

    // Helper functions to check interrupt flags for specific streams
    inline bool isTransferComplete(int controller, Stream stream) {
        const auto streamIdx = static_cast<uint32_t>(stream);
        if (streamIdx < 4) {
            // Streams 0-3 are in LISR
            const uint32_t bitPos = 5 + (streamIdx * 6); // TCIF bit positions
            return (controller == 1) ? 
                (getDMA1Registers()->LISR & (1UL << bitPos)) :
                (getDMA2Registers()->LISR & (1UL << bitPos));
        } else {
            // Streams 4-7 are in HISR
            const uint32_t bitPos = 5 + ((streamIdx - 4) * 6); // TCIF bit positions
            return (controller == 1) ? 
                (getDMA1Registers()->HISR & (1UL << bitPos)) :
                (getDMA2Registers()->HISR & (1UL << bitPos));
        }
    }

    // Helper function to clear all interrupt flags for a stream
    inline void clearAllFlags(int controller, Stream stream) {
        const auto streamIdx = static_cast<uint32_t>(stream);
        if (streamIdx < 4) {
            // Streams 0-3 are in LIFCR
            const uint32_t flagsMask = 0x3D << (streamIdx * 6); // All flags for this stream
            if (controller == 1) {
                getDMA1Registers()->LIFCR = flagsMask;
            } else {
                getDMA2Registers()->LIFCR = flagsMask;
            }
        } else {
            // Streams 4-7 are in HIFCR
            const uint32_t flagsMask = 0x3D << ((streamIdx - 4) * 6); // All flags for this stream
            if (controller == 1) {
                getDMA1Registers()->HIFCR = flagsMask;
            } else {
                getDMA2Registers()->HIFCR = flagsMask;
            }
        }
    }
}
// -------------------- Flash Definitions --------------------

namespace FLASH {
    constexpr uint32_t FLASH_BASE = 0x40023C00UL;
    constexpr uint32_t FLASH_ACR_OFFSET = 0x00UL;
    constexpr uint32_t FLASH_ACR = (FLASH_BASE + FLASH_ACR_OFFSET);
    
    // Flash access control register (ACR) bits
    enum class ACR : uint32_t {
        LATENCY_0WS = (0UL << 0),   // Zero wait states
        LATENCY_1WS = (1UL << 0),   // One wait state
        LATENCY_2WS = (2UL << 0),   // Two wait states
        LATENCY_3WS = (3UL << 0),   // Three wait states
        LATENCY_4WS = (4UL << 0),   // Four wait states
        LATENCY_5WS = (5UL << 0),   // Five wait states
        LATENCY_6WS = (6UL << 0),   // Six wait states
        LATENCY_7WS = (7UL << 0),   // Seven wait states
        LATENCY_MSK = (7UL << 0),   // Latency mask
        PRFTEN = (1UL << 8),        // Prefetch enable
        ICEN = (1UL << 9),          // Instruction cache enable
        DCEN = (1UL << 10),         // Data cache enable
        ICRST = (1UL << 11),        // Instruction cache reset
        DCRST = (1UL << 12)         // Data cache reset
    };
    
    // Flash register structure
    struct Registers {
        volatile uint32_t ACR;      // Flash access control register
        volatile uint32_t KEYR;     // Flash key register
        volatile uint32_t OPTKEYR;  // Flash option key register
        volatile uint32_t SR;       // Flash status register
        volatile uint32_t CR;       // Flash control register
        volatile uint32_t OPTCR;    // Flash option control register
    };
    
    // Access function for Flash registers
    inline Registers* getRegisters() {
        return reinterpret_cast<Registers*>(FLASH_BASE);
    }
    
    // Helper functions for bit manipulation
    constexpr uint32_t getBitValue(ACR bit) {
        return static_cast<uint32_t>(bit);
    }
    
    // Operator overload for combining flags
    constexpr ACR operator|(ACR a, ACR b) {
        return static_cast<ACR>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }
}



// -------------------- Utility Functions --------------------

// Generic bit manipulation templates
template<typename T>
inline void setBit(volatile T& reg, T bit) {
    reg |= bit;
}

template<typename T>
inline void clearBit(volatile T& reg, T bit) {
    reg &= ~bit;
}

template<typename T>
inline T readBit(volatile T& reg, T bit) {
    return reg & bit;
}

template<typename T>
inline bool isBitSet(volatile T& reg, T bit) {
    return (reg & bit) != 0;
}

template<typename T>
inline void clearReg(volatile T& reg) {
    reg = 0;
}

template<typename T, typename U>
inline void writeReg(volatile T& reg, U val) {
    reg = static_cast<T>(val);
}

template<typename T>
inline T readReg(volatile T& reg) {
    return reg;
}

template<typename T, typename U, typename V>
inline void modifyReg(volatile T& reg, U clearMask, V setMask) {
    reg = (reg & (~static_cast<T>(clearMask))) | static_cast<T>(setMask);
}

// System initialization function
void initSystemClock();

// Delay functions
void delay(uint32_t amount, bool blocking = true);

} // namespace Platform