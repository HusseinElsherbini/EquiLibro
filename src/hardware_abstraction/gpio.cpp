// gpio.cpp - Updated to use RCC Interface
#include "hardware_abstraction/gpio.hpp"
#include "hardware_abstraction/rcc.hpp"
#include "common/platform.hpp"
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
}
}