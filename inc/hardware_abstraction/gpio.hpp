// gpio.hpp
#ifndef GPIO_HPP
#define GPIO_HPP

#include "hw_interface.hpp"
#include "common/platform.hpp"

// Forward declarations
namespace Platform {
    enum class Status;
}

/**
 * GPIO pin configuration structure
 */
struct GpioConfig {
    Platform::GPIO::Port port;            // GPIO port (A, B, C, etc.)
    uint8_t pin;                          // Pin number (0-15)
    Platform::GPIO::Mode mode;            // Pin mode (Input, Output, Alternate, Analog)
    Platform::GPIO::OutputType outputType; // Output type (Push-Pull, Open-Drain)
    Platform::GPIO::Pull pull;            // Pull-up/Pull-down configuration
    Platform::GPIO::Speed speed;          // GPIO speed
    Platform::GPIO::AlternateFunction af; // Alternate function (if mode is AlternateFunction)
};

/**
 * GPIO pin state enumeration
 */
enum class GpioPinState {
    Low = 0,
    High = 1
};

/**
 * GPIO hardware interface implementation
 * Provides a C++ object-oriented interface to GPIO peripherals
 */
class GpioInterface : public HwInterface {
private:
    // Internal state tracking
    bool initialized;
    
    // Helper methods
    Platform::Status ConfigPin(const GpioConfig& config);
    Platform::GPIO::Registers* GetPortAddress(Platform::GPIO::Port port);
    
public:
    // Constructor
    GpioInterface();
    
    // Destructor
    ~GpioInterface() override;
    
    // Interface implementation
    Platform::Status Init(void* config) override;
    Platform::Status DeInit() override;
    Platform::Status Control(uint32_t command, void* param) override;
    Platform::Status Read(void* buffer, uint16_t size, uint32_t timeout) override;
    Platform::Status Write(const void* data, uint16_t size, uint32_t timeout) override;
    Platform::Status RegisterCallback(uint32_t eventId, void (*callback)(void* param), void* param) override;
    
    // GPIO-specific methods 
    Platform::Status SetPin(Platform::GPIO::Port port, uint8_t pin);
    Platform::Status ResetPin(Platform::GPIO::Port port, uint8_t pin);
    Platform::Status TogglePin(Platform::GPIO::Port port, uint8_t pin);
    Platform::Status ReadPin(Platform::GPIO::Port port, uint8_t pin, GpioPinState& state);
    Platform::Status ConfigurePin(const GpioConfig& config);
    
    // Singleton pattern for GPIO interface (only need one instance)
    static GpioInterface& GetInstance();
};

// GPIO control command identifiers
constexpr uint32_t GPIO_CTRL_SET_PIN = 0x0101;
constexpr uint32_t GPIO_CTRL_RESET_PIN = 0x0102;
constexpr uint32_t GPIO_CTRL_TOGGLE_PIN = 0x0103;
constexpr uint32_t GPIO_CTRL_READ_PIN = 0x0104;
constexpr uint32_t GPIO_CTRL_CONFIG_PIN = 0x0105;

#endif /* GPIO_HPP */