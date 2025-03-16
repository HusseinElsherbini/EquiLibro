#pragma once     
#include "platform.hpp"

namespace Platform {
namespace GPIO {
    // GPIO port base addresses
    constexpr uint32_t GPIOA_BASE = (AHB1PERIPH_BASE + 0x0000UL);
    constexpr uint32_t GPIOB_BASE = (AHB1PERIPH_BASE + 0x0400UL);
    constexpr uint32_t GPIOC_BASE = (AHB1PERIPH_BASE + 0x0800UL);
    constexpr uint32_t GPIOD_BASE = (AHB1PERIPH_BASE + 0x0C00UL);
    constexpr uint32_t GPIOE_BASE = (AHB1PERIPH_BASE + 0x1000UL);
    constexpr uint32_t GPIOH_BASE = (AHB1PERIPH_BASE + 0x1C00UL);

    
    /**
     * GPIO pin state enumeration
     */
    enum class GpioPinState : uint8_t {
        Low = 0,
        High = 1
    };
    
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
    struct GpioConfig {
        Platform::GPIO::Port port;            // GPIO port (A, B, C, etc.)
        uint8_t pin;                          // Pin number (0-15)
        Platform::GPIO::Mode mode;            // Pin mode (Input, Output, Alternate, Analog)
        Platform::GPIO::OutputType outputType; // Output type (Push-Pull, Open-Drain)
        Platform::GPIO::Pull pull;            // Pull-up/Pull-down configuration
        Platform::GPIO::Speed speed;          // GPIO speed
        Platform::GPIO::AlternateFunction af; // Alternate function (if mode is AlternateFunction)
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
}