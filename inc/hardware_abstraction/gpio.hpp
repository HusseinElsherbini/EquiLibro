// gpio.hpp
#ifndef GPIO_HPP
#define GPIO_HPP

#include "hw_interface.hpp"
#include "platform.hpp"  


// GPIO control command identifiers
constexpr uint32_t GPIO_CTRL_SET_PIN = 0x0101;
constexpr uint32_t GPIO_CTRL_RESET_PIN = 0x0102;
constexpr uint32_t GPIO_CTRL_TOGGLE_PIN = 0x0103;
constexpr uint32_t GPIO_CTRL_READ_PIN = 0x0104;
constexpr uint32_t GPIO_CTRL_CONFIG_PIN = 0x0105;

/**
 * GPIO hardware interface implementation
 */
class GpioInterface : public HwInterface {
private:
    // Internal state
    bool initialized;
    
    // Helper methods
    Platform::Status ConfigPin(const GpioConfig& config);
    Platform::GPIO::Registers* GpioInterface::GetPortAddress(Platform::GPIO::Port port);
    
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
    
    // Singleton pattern for GPIO interface (only need one instance)
    static GpioInterface& GetInstance();
};

#endif /* GPIO_HPP */