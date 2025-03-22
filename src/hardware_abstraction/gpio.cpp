// gpio.cpp - Updated to use RCC Interface
#include "hardware_abstraction/gpio.hpp"
#include "hardware_abstraction/rcc.hpp"
#include "common/platform.hpp"
#include "common/platform_cmsis.hpp"
#include <memory>
#include <mutex>

namespace Platform {
namespace GPIO {
// Constructor
GpioInterface::GpioInterface() : initialized(false) {
    // Nothing else to initialize here
}

// Destructor
GpioInterface::~GpioInterface() {
    // Clean up resources if needed
    if (initialized) {
        DeInit();
    }
}

// Singleton instance getter
GpioInterface& GpioInterface::GetInstance() {

    // Static instance of the interface, created on first use
    static GpioInterface instance;
    return instance;
}

// Map port enum to register base addresses
Platform::GPIO::Registers* GpioInterface::GetPortAddress(Platform::GPIO::Port port) {
    return Platform::GPIO::getPort(port);
}

// Configure a GPIO pin
Platform::Status GpioInterface::ConfigPin(const Platform::GPIO::GpioConfig& config) {

    using Platform::GPIO::Registers;
    using Platform::GPIO::getPort;
    using Platform::RCC::RccPeripheral;
    // etc.
    // Get port register base address
    Platform::GPIO::Registers* port = GetPortAddress(config.port);
    if (port == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }

    // Enable clock to GPIO port using RCC interface
    Platform::RCC::RccInterface* rcc = &Platform::RCC::RccInterface::GetInstance();

    if (rcc == nullptr) {
        return Platform::Status::ERROR;
    }    
    // Map GPIO port to RCC peripheral
    RccPeripheral rccPeripheral;
    switch (config.port) {
        case Platform::GPIO::Port::PORTA:
            rccPeripheral = RccPeripheral::GPIOA;
            break;
        case Platform::GPIO::Port::PORTB:
            rccPeripheral = RccPeripheral::GPIOB;
            break;
        case Platform::GPIO::Port::PORTC:
            rccPeripheral = RccPeripheral::GPIOC;
            break;
        case Platform::GPIO::Port::PORTD:
            rccPeripheral = RccPeripheral::GPIOD;
            break;
        case Platform::GPIO::Port::PORTE:
            rccPeripheral = RccPeripheral::GPIOE;
            break;
        case Platform::GPIO::Port::PORTH:
            rccPeripheral = RccPeripheral::GPIOH;
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Enable the peripheral clock
    Platform::Status status = rcc->EnablePeripheralClock(rccPeripheral);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Configure pin mode (input, output, alternate function, analog)
    // Clear the mode bits for the pin (2 bits per pin)
    port->MODER &= ~(0x3UL << (config.pin * 2));
    // Set the mode for the pin
    port->MODER |= (static_cast<uint32_t>(config.mode) << (config.pin * 2));

    // Configure output type (push-pull or open-drain) if output or alternate function
    if (config.mode == Platform::GPIO::Mode::Output || config.mode == Platform::GPIO::Mode::AlternateFunction) {
        if (config.outputType == Platform::GPIO::OutputType::OpenDrain) {
            port->OTYPER |= (1UL << config.pin);
        } else {
            port->OTYPER &= ~(1UL << config.pin);
        }
    }

    // Configure pull-up/pull-down
    // Clear the pull bits for the pin (2 bits per pin)
    port->PUPDR &= ~(0x3UL << (config.pin * 2));
    // Set the pull mode for the pin
    port->PUPDR |= (static_cast<uint32_t>(config.pull) << (config.pin * 2));

    // Configure speed
    // Clear the speed bits for the pin (2 bits per pin)
    port->OSPEEDR &= ~(0x3UL << (config.pin * 2));
    // Set the speed for the pin
    port->OSPEEDR |= (static_cast<uint32_t>(config.speed) << (config.pin * 2));

    // Configure alternate function if needed
    if (config.mode == Platform::GPIO::Mode::AlternateFunction) {
        uint8_t shift_amount = (config.pin % 8) * 4;
        if (config.pin < 8) {
            // Clear the AF bits for the pin (4 bits per pin)
            port->AFRL &= ~(0xFUL << shift_amount);
            // Set the AF for the pin
            port->AFRL |= (static_cast<uint32_t>(config.af) << shift_amount);
        } else {
            // Clear the AF bits for the pin (4 bits per pin)
            port->AFRH &= ~(0xFUL << shift_amount);
            // Set the AF for the pin
            port->AFRH |= (static_cast<uint32_t>(config.af) << shift_amount);
        }
    }

    return Platform::Status::OK;
}

// Implementation of interface methods

Platform::Status GpioInterface::Init(void* config) {
    // Check if already initialized
    if (initialized) {
        return Platform::Status::OK; // Already initialized
    }    
    initialized = true;
    return Platform::Status::OK;
}

Platform::Status GpioInterface::DeInit() {
    // Nothing specific to clean up
    initialized = false;
    return Platform::Status::OK;
}

Platform::Status GpioInterface::Control(uint32_t command, void* param) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }

    // For some commands, param is allowed to be null
    if (param == nullptr && command != GPIO_CTRL_TOGGLE_PIN) {
        return Platform::Status::INVALID_PARAM;
    }

    // Process the command
    switch (command) {
        case GPIO_CTRL_CONFIG_PIN: {
            Platform::GPIO::GpioConfig* pin_config = static_cast<Platform::GPIO::GpioConfig*>(param);
            return ConfigPin(*pin_config);
        }
            
        case GPIO_CTRL_SET_PIN: {
            Platform::GPIO::GpioConfig* pin_config = static_cast<Platform::GPIO::GpioConfig*>(param);
            return SetPin(pin_config->port, pin_config->pin);
        }
            
        case GPIO_CTRL_RESET_PIN: {
            Platform::GPIO::GpioConfig* pin_config = static_cast<Platform::GPIO::GpioConfig*>(param);
            return ResetPin(pin_config->port, pin_config->pin);
        }
            
        case GPIO_CTRL_TOGGLE_PIN: {
            Platform::GPIO::GpioConfig* pin_config = static_cast<Platform::GPIO::GpioConfig*>(param);
            return TogglePin(pin_config->port, pin_config->pin);
        }
            
        case GPIO_CTRL_READ_PIN: {
            // Param is both input (config) and output (state)
            Platform::GPIO::GpioConfig* pin_config = static_cast<Platform::GPIO::GpioConfig*>(param);
            Platform::GPIO::GpioPinState* state = reinterpret_cast<Platform::GPIO::GpioPinState*>(
                reinterpret_cast<char*>(param) + sizeof(Platform::GPIO::GpioConfig)
            );
            return ReadPin(pin_config->port, pin_config->pin, *state);
        }
            
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
}

Platform::Status GpioInterface::Read(void* buffer, uint16_t size, uint32_t timeout) {
    // Direct Read operation not typical for GPIO - use Control with READ_PIN instead
    return Platform::Status::NOT_SUPPORTED;
}

Platform::Status GpioInterface::Write(const void* data, uint16_t size, uint32_t timeout) {
    // Direct Write operation not typical for GPIO - use Control with SET/RESET_PIN instead
    return Platform::Status::NOT_SUPPORTED;
}

Platform::Status GpioInterface::RegisterCallback(uint32_t eventId, void (*callback)(void* param), void* param) {
    // Callbacks not typically supported for basic GPIO - would be used for GPIO interrupts
    // which could be implemented separately
    return Platform::Status::NOT_SUPPORTED;
}

// GPIO-specific methods

// atomic set pin
Platform::Status GpioInterface::SetPin(Platform::GPIO::Port port, uint8_t pin) {
    Platform::GPIO::Registers* gpio_port = GetPortAddress(port);
    if (gpio_port == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Set the pin using BSRR register (Bit Set/Reset Register)
    // Writing 1 to bits 0-15 sets the corresponding pin
    gpio_port->BSRR = (1UL << pin);
    
    return Platform::Status::OK;
}

// atomic reset pin
Platform::Status GpioInterface::ResetPin(Platform::GPIO::Port port, uint8_t pin) {
    Platform::GPIO::Registers* gpio_port = GetPortAddress(port);
    if (gpio_port == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Reset the pin using BSRR register
    // Writing 1 to bits 16-31 resets the corresponding pin (pin+16)
    gpio_port->BSRR = (1UL << (pin + 16));
    
    return Platform::Status::OK;
}

Platform::Status GpioInterface::TogglePin(Platform::GPIO::Port port, uint8_t pin) {
    Platform::GPIO::Registers* gpio_port = GetPortAddress(port);
    if (gpio_port == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Toggle the pin by XORing the ODR register
    gpio_port->ODR ^= (1UL << pin);
    
    return Platform::Status::OK;
}

Platform::Status GpioInterface::ReadPin(Platform::GPIO::Port port, uint8_t pin, Platform::GPIO::GpioPinState& state) {
    Platform::GPIO::Registers* gpio_port = GetPortAddress(port);
    if (gpio_port == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Read the pin state from the IDR register
    state = (gpio_port->IDR & (1UL << pin)) ? Platform::GPIO::GpioPinState::High : Platform::GPIO::GpioPinState::Low;
    
    return Platform::Status::OK;
}

Platform::Status GpioInterface::ConfigurePin(const Platform::GPIO::GpioConfig& config) {
    return ConfigPin(config);
}

// Configure interrupt for a GPIO pin
Platform::Status GpioInterface::ConfigureInterrupt(Port port, uint8_t pin, InterruptTrigger trigger) {
    if (pin > 15) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Enable SYSCFG clock (needed for EXTI configuration)
    Platform::RCC::RccInterface* rcc = &Platform::RCC::RccInterface::GetInstance();
    rcc->EnablePeripheralClock(Platform::RCC::RccPeripheral::SYSCFG);
    
    // Select GPIO port for EXTI line
    uint8_t port_index = static_cast<uint8_t>(port);
    uint8_t exti_cr_index = pin / 4;
    uint8_t exti_cr_position = (pin % 4) * 4;
    
    // Clear the EXTICR bits for this pin
    Platform::GPIO::getSYSCFG()->EXTICR[exti_cr_index] &= ~(0xF << exti_cr_position);
    // Set the port selection for this pin
    Platform::GPIO::getSYSCFG()->EXTICR[exti_cr_index] |= (port_index << exti_cr_position);
    
    // Configure the trigger type
    switch (trigger) {
        case InterruptTrigger::Rising:
            Platform::GPIO::getEXTI()->RTSR |= (1 << pin);  // Enable rising trigger
            Platform::GPIO::getEXTI()->FTSR &= ~(1 << pin); // Disable falling trigger
            break;
        case InterruptTrigger::Falling:
            Platform::GPIO::getEXTI()->RTSR &= ~(1 << pin); // Disable rising trigger
            Platform::GPIO::getEXTI()->FTSR |= (1 << pin);  // Enable falling trigger
            break;
        case InterruptTrigger::Both:
            Platform::GPIO::getEXTI()->RTSR |= (1 << pin);  // Enable rising trigger
            Platform::GPIO::getEXTI()->FTSR |= (1 << pin);  // Enable falling trigger
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    return Platform::Status::OK;
}

// Enable or disable interrupt for a GPIO pin
Platform::Status GpioInterface::EnableInterrupt(Port port, uint8_t pin, bool enable) {
    if (pin > 15) {
        return Platform::Status::INVALID_PARAM;
    }
    Platform::CMSIS::NVIC::IRQn irq;
    
    if (enable) {
        // Enable EXTI line interrupt
        Platform::GPIO::getEXTI()->IMR |= (1 << pin);
        
        if (pin <= 4) {
            // Use a lookup table or switch statement for individual EXTI0-EXTI4 IRQs
            switch (pin) {
                case 0: irq = Platform::CMSIS::NVIC::IRQn::EXTI0; break;
                case 1: irq = Platform::CMSIS::NVIC::IRQn::EXTI1; break;
                case 2: irq = Platform::CMSIS::NVIC::IRQn::EXTI2; break;
                case 3: irq = Platform::CMSIS::NVIC::IRQn::EXTI3; break;
                case 4: irq = Platform::CMSIS::NVIC::IRQn::EXTI4; break;
            }
        } else if (pin <= 9) {
            irq = Platform::CMSIS::NVIC::IRQn::EXTI9_5;
        } else {
            irq = Platform::CMSIS::NVIC::IRQn::EXTI15_10;
        }
        
        Platform::CMSIS::NVIC::enableIRQ(irq);
    } else {
        // Disable EXTI line interrupt
        Platform::GPIO::getEXTI()->IMR &= ~(1 << pin);
        
        // We don't disable the NVIC interrupt as other pins might be using it
    }
    
    return Platform::Status::OK;
}

// Register callback for a GPIO pin interrupt
Platform::Status GpioInterface::RegisterInterruptCallback(Port port, uint8_t pin, void (*callback)(void* param), void* param) {
    if (pin > 15) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Store the callback
    exti_callbacks[pin].callback = callback;
    exti_callbacks[pin].param = param;
    exti_callbacks[pin].enabled = (callback != nullptr);
    
    return Platform::Status::OK;
}

// Static handler for external interrupts
void GpioInterface::HandleExternalInterrupt(uint32_t pin_mask) {
    // Get instance
    GpioInterface& gpio = GpioInterface::GetInstance();
    
    // Process each pin in the mask
    for (uint8_t pin = 0; pin < 16; pin++) {
        if (pin_mask & (1 << pin)) {
            // Clear the pending flag first
            Platform::GPIO::getEXTI()->PR = (1 << pin);
            
            // Call the registered callback if enabled
            if (gpio.exti_callbacks[pin].enabled && gpio.exti_callbacks[pin].callback) {
                gpio.exti_callbacks[pin].callback(gpio.exti_callbacks[pin].param);
            }
        }
    }
}
}
}

extern "C" {
    void EXTI0_IRQHandler(void) {
        // Handle EXTI0 interrupt
        Platform::GPIO::GpioInterface::HandleExternalInterrupt(Platform::GPIO::EXTI_LINE0);
    }
    
    void EXTI1_IRQHandler(void) {
        // Handle EXTI1 interrupt
        Platform::GPIO::GpioInterface::HandleExternalInterrupt(Platform::GPIO::EXTI_LINE1);
    }
    
    void EXTI2_IRQHandler(void) {
        // Handle EXTI2 interrupt
        Platform::GPIO::GpioInterface::HandleExternalInterrupt(Platform::GPIO::EXTI_LINE2);
    }
    
    void EXTI3_IRQHandler(void) {
        // Handle EXTI3 interrupt
        Platform::GPIO::GpioInterface::HandleExternalInterrupt(Platform::GPIO::EXTI_LINE3);
    }
    
    void EXTI4_IRQHandler(void) {
        // Handle EXTI4 interrupt
        Platform::GPIO::GpioInterface::HandleExternalInterrupt(Platform::GPIO::EXTI_LINE4);
    }
    
    void EXTI9_5_IRQHandler(void) {
        // Get pending interrupts for pins 5-9
        uint16_t pending = Platform::GPIO::getEXTI()->PR & Platform::GPIO::getEXTI()->IMR & 0x03E0; // Mask for pins 5-9
        
        // Handle EXTI5-9 interrupts
        Platform::GPIO::GpioInterface::HandleExternalInterrupt(pending);
    }
    
    void EXTI15_10_IRQHandler(void) {
        // Get pending interrupts for pins 10-15
        uint16_t pending = Platform::GPIO::getEXTI()->PR & Platform::GPIO::getEXTI()->IMR & 0xFC00; // Mask for pins 10-15
        
        // Handle EXTI10-15 interrupts
        Platform::GPIO::GpioInterface::HandleExternalInterrupt(pending);
    }
}