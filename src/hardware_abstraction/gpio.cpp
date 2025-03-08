// src/hardware_abstraction/gpio.c

#include "hardware_abstraction/gpio.hpp"
#include "common/platform.hpp"  // Includes MCU-specific register definitions

// Static instance for singleton pattern
GpioInterface& GpioInterface::GetInstance() {
    static GpioInterface instance;
    return instance;
}

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

// Map STM32 port names to port base addresses
Platform::GPIO::Registers* GpioInterface::GetPortAddress(Platform::GPIO::Port port) {

    Platform::GPIO::getPort(port); 

}

// Configure a GPIO pin
Platform::Status GpioInterface::ConfigPin(const GpioConfig& config) {
    GPIO_REGS_t* port = GetPortAddress(config.port);
    if (port == nullptr) {
        return STATUS_INVALID_PARAM;
    }

    // Enable clock to GPIO port
    ENABLE_BUS_TO_GPIO_PORT(config.port);

    // Set the pin mode (input, output, alternate function, analog)
    uint32_t mode_value;
    switch (config.mode) {
        case GpioMode::Input:     mode_value = GPIO_INPUT_MODE; break;
        case GpioMode::Output:    mode_value = GPIO_OUTPUT_MODE; break;
        case GpioMode::Alternate: mode_value = GPIO_AF_MODE; break;
        case GpioMode::Analog:    mode_value = GPIO_ANALOG_MODE; break;
        default: return STATUS_INVALID_PARAM;
    }
    
    // Clear the bits for the pin
    port->MODER &= ~GPIO_MODE_MSK(config.pin, 3U);
    // Set the mode for the pin
    port->MODER |= GPIO_MODE_MSK(config.pin, mode_value);

    // Configure output type (push-pull or open-drain)
    if (config.mode == GpioMode::Output || config.mode == GpioMode::Alternate) {
        if (config.outputType == GpioOutputType::OpenDrain) {
            SET_BIT(port->OTYPER, (1U << config.pin));
        } else {
            CLEAR_BIT(port->OTYPER, (1U << config.pin));
        }
    }

    // Configure pull-up/pull-down
    uint32_t pull_value;
    switch (config.pull) {
        case GpioPull::None: pull_value = NOPULLUP_NOPULLDOWN; break;
        case GpioPull::Up:   pull_value = PULL_UP; break;
        case GpioPull::Down: pull_value = PULL_DOWN; break;
        default: return STATUS_INVALID_PARAM;
    }
    
    // Clear the bits for the pin
    port->PUPDR &= ~GENERIC_SET_MSK(3U, (2U)*config.pin);
    // Set the pull mode for the pin
    MODIFY_REG(port->PUPDR, GENERIC_SET_MSK(3U, (2U)*config.pin), GENERIC_SET_MSK(pull_value, (2U)*config.pin));

    // Configure speed
    uint32_t speed_value;
    switch (config.speed) {
        case GpioSpeed::Low:      speed_value = LOW_SPEED; break;
        case GpioSpeed::Medium:   speed_value = MED_SPEED; break;
        case GpioSpeed::High:     speed_value = HIGH_SPEED; break;
        case GpioSpeed::VeryHigh: speed_value = MAX_SPEED; break;
        default: return STATUS_INVALID_PARAM;
    }
    port->OSPEEDR |= GENERIC_SET_MSK(speed_value, config.pin*2U);

    // Configure alternate function
    if (config.mode == GpioMode::Alternate) {
        uint16_t shift_amount = (4U * (config.pin % 8U));
        if (config.pin < 8) {
            // Clear the bits for the pin
            port->AFRL &= ~GENERIC_SET_MSK(15U, shift_amount);
            // Set the alternate function for the pin
            MODIFY_REG(port->AFRL, (15U << shift_amount), (static_cast<uint32_t>(config.af) << shift_amount));
        } else {
            // Clear the bits for the pin
            port->AFRH &= ~GENERIC_SET_MSK(15U, shift_amount);
            // Set the alternate function for the pin
            MODIFY_REG(port->AFRH, (15U << shift_amount), (static_cast<uint32_t>(config.af) << shift_amount));
        }
    }

    return STATUS_OK;
}

// Implementation of GPIO initialization
Status_t GpioInterface::Init(void* config) {
    // No specific initialization needed beyond configuring individual pins
    initialized = true;
    return STATUS_OK;
}

// Implementation of GPIO de-initialization
Status_t GpioInterface::DeInit() {
    initialized = false;
    return STATUS_OK;
}

// Implementation of GPIO control function
Status_t GpioInterface::Control(uint32_t command, void* param) {
    if (!initialized) {
        return STATUS_NOT_INITIALIZED;
    }

    if (param == nullptr && command != GPIO_CTRL_TOGGLE_PIN) {
        return STATUS_INVALID_PARAM;
    }

    GpioConfig* pin_config;
    GpioPinState* pin_state;
    GPIO_REGS_t* port;

    switch (command) {
        case GPIO_CTRL_CONFIG_PIN:
            pin_config = static_cast<GpioConfig*>(param);
            return ConfigPin(*pin_config);
            
        case GPIO_CTRL_SET_PIN:
            pin_config = static_cast<GpioConfig*>(param);
            port = GetPortAddress(pin_config->port);
            if (port == nullptr) {
                return STATUS_INVALID_PARAM;
            }
            SET_BIT(port->BSRR, (1U << pin_config->pin));
            return STATUS_OK;
            
        case GPIO_CTRL_RESET_PIN:
            pin_config = static_cast<GpioConfig*>(param);
            port = GetPortAddress(pin_config->port);
            if (port == nullptr) {
                return STATUS_INVALID_PARAM;
            }
            SET_BIT(port->BSRR, (1U << (pin_config->pin + 16))); // BSRR[31:16] are reset bits
            return STATUS_OK;
            
        case GPIO_CTRL_TOGGLE_PIN:
            pin_config = static_cast<GpioConfig*>(param);
            port = GetPortAddress(pin_config->port);
            if (port == nullptr) {
                return STATUS_INVALID_PARAM;
            }
            WRITE_REG(port->ODR, port->ODR ^ (1U << pin_config->pin));
            return STATUS_OK;
            
        case GPIO_CTRL_READ_PIN:
            pin_config = static_cast<GpioConfig*>(param);
            port = GetPortAddress(pin_config->port);
            if (port == nullptr) {
                return STATUS_INVALID_PARAM;
            }
            pin_state = static_cast<GpioPinState*>(param); // Reuse param as output
            *pin_state = static_cast<GpioPinState>((READ_BIT(port->IDR, (1U << pin_config->pin)) >> pin_config->pin) & 0x01);
            return STATUS_OK;
            
        default:
            return STATUS_NOT_SUPPORTED;
    }
}

// Implementation of GPIO read function - not used for GPIO
Status_t GpioInterface::Read(void* buffer, uint16_t size, uint32_t timeout) {
    return STATUS_NOT_SUPPORTED; // Direct reads not supported, use Control function
}

// Implementation of GPIO write function - not used for GPIO
Status_t GpioInterface::Write(const void* data, uint16_t size, uint32_t timeout) {
    return STATUS_NOT_SUPPORTED; // Direct writes not supported, use Control function
}

// Implementation of GPIO callback registration - not used for GPIO
Status_t GpioInterface::RegisterCallback(uint32_t eventId, void (*callback)(void* param), void* param) {
    return STATUS_NOT_SUPPORTED; // Callbacks not supported for GPIO
}