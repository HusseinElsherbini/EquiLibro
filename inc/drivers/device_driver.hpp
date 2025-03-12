#pragma once

#include "hardware_abstraction/hw_interface.hpp"
#include "hardware_abstraction/gpio.hpp"
#include "hardware_abstraction/i2c.hpp"
#include "hardware_abstraction/timer.hpp"
#include "middleware/system_services/system_timing.hpp"
#include <memory>
#include <map>
#include <string>
#include <functional>

namespace Drivers {

/**
 * Device state enumeration defining the possible states of a device
 */
enum class DeviceState {
    Uninitialized,   // Device has not been initialized yet
    Initialized,     // Device is initialized but not active
    Running,         // Device is active and operational
    LowPower,        // Device is in low power mode
    Error,           // Device is in error state
    Suspended        // Device operation is temporarily suspended
};

/**
 * Device power modes defining the power consumption states
 */
enum class DevicePower {
    Full,            // Full power operation mode
    Low,             // Low power operation mode
    Off              // Device powered off
};

/**
 * Role that a hardware interface plays in a device driver
 */
enum class InterfaceRole {
    Primary,         // Primary communication interface
    Secondary,       // Secondary communication interface
    Control,         // Interface for control signals (e.g., reset, enable)
    Data,            // Interface for data transfer
    Interrupt,       // Interface for interrupt handling
    Timing,          // Interface for timing services
    Clock,           // Interface for clock generation
    Power,           // Interface for power management
    Auxiliary        // Additional support interface
};

/**
 * Device driver interface that provides a consistent API
 * for interacting with sensors, actuators, and communication devices.
 */
class DeviceDriver {
public:
    /**
     * Virtual destructor to ensure proper cleanup of derived classes
     */
    virtual ~DeviceDriver() = default;

    /**
     * Get the current state of the device
     * 
     * @return Current device state
     */
    virtual DeviceState GetState() const = 0;
    
    /**
     * Get the hardware interface for a specific role
     * 
     * @param role The role of the hardware interface to retrieve
     * @return Shared pointer to the hardware interface, or nullptr if not available
     */
    virtual std::shared_ptr<Platform::HwInterface> GetInterfaceByRole(InterfaceRole role) const = 0;
    
    /**
     * Get the primary hardware interface for the device
     * 
     * @return Shared pointer to the primary hardware interface
     */
    virtual std::shared_ptr<Platform::HwInterface> GetPrimaryInterface() const = 0;

    /**
     * Initialize the device with specific configuration
     * 
     * @param config Pointer to device-specific configuration
     * @return Status code indicating success or failure
     */
    virtual Platform::Status Init(void* config) = 0;

    /**
     * Control device power state
     * 
     * @param state Power state to set
     * @return Status code indicating success or failure
     */
    virtual Platform::Status PowerControl(DevicePower state) = 0;

    /**
     * Control function for device-specific commands
     * 
     * @param command Command identifier
     * @param arg Command-specific argument
     * @return Status code indicating success or failure
     */
    virtual Platform::Status Control(uint32_t command, void* arg) = 0;

    /**
     * Read data from the device
     * 
     * @param data Buffer to store read data
     * @param size Size of data to read
     * @return Status code indicating success or failure
     */
    virtual Platform::Status ReadData(void* data, uint32_t size) = 0;

    /**
     * Write data to the device
     * 
     * @param data Data to write
     * @param size Size of data to write
     * @return Status code indicating success or failure
     */
    virtual Platform::Status WriteData(const void* data, uint32_t size) = 0;

    /**
     * Register callback for device events
     * 
     * @param event Event identifier
     * @param callback Callback function
     * @param param Parameter to pass to callback
     * @return Status code indicating success or failure
     */
    virtual Platform::Status RegisterCallback(uint32_t event, void (*callback)(void* param), void* param) = 0;
    
    /**
     * Register callback for device events with std::function
     * 
     * @param event Event identifier
     * @param callback Callback function
     * @return Status code indicating success or failure
     */
    virtual Platform::Status RegisterCallback(uint32_t event, std::function<void()> callback) = 0;
};

// Common device control commands
constexpr uint32_t DEVICE_CTRL_CALIBRATE = 0x1001;       // Calibrate the device
constexpr uint32_t DEVICE_CTRL_SELF_TEST = 0x1002;       // Perform device self test
constexpr uint32_t DEVICE_CTRL_GET_INFO = 0x1003;        // Get device information
constexpr uint32_t DEVICE_CTRL_SET_PARAMETER = 0x1004;   // Set device parameter
constexpr uint32_t DEVICE_CTRL_GET_PARAMETER = 0x1005;   // Get device parameter
constexpr uint32_t DEVICE_CTRL_RESET_DEVICE = 0x1006;    // Reset the device

/**
 * Base implementation of a device driver with common functionality
 * and support for multiple hardware interfaces
 */
class DeviceDriverBase : public DeviceDriver {
protected:
    // Device state
    DeviceState state;
    
    // Configuration storage
    void* config_data;
    
    // Type-specific hardware interfaces
    std::shared_ptr<Platform::I2C::I2CInterface> i2c_interface;
    std::shared_ptr<Platform::GPIO::GpioInterface> gpio_interface;
    std::shared_ptr<TimerInterface> timer_interface;
    std::shared_ptr<Middleware::SystemServices::SystemTiming> timing_service;
    
    // Role mapping for interfaces
    std::map<InterfaceRole, std::shared_ptr<Platform::HwInterface>> interface_roles;
    
    // Callback storage
    struct CallbackInfo {
        void (*callback)(void* param);
        void* param;
        std::function<void()> func_callback;
        bool is_std_function;
    };
    
    std::map<uint32_t, CallbackInfo> callbacks;
    
    /**
     * Helper method to ensure all required interfaces are available
     * 
     * @return Status code indicating if all required interfaces are available
     */
    virtual Platform::Status ValidateInterfaces() const {
        // Base implementation checks if primary interface is set
        if (GetPrimaryInterface() == nullptr) {
            return Platform::Status::ERROR;
        }
        return Platform::Status::OK;
    }
    
    /**
     * Internal method to execute a callback
     * 
     * @param event Event identifier
     * @return Status code indicating success or if callback doesn't exist
     */
    Platform::Status TriggerCallback(uint32_t event) {
        auto it = callbacks.find(event);
        if (it != callbacks.end()) {
            if (it->second.is_std_function) {
                it->second.func_callback();
            } else {
                if (it->second.callback) {
                    it->second.callback(it->second.param);
                }
            }
            return Platform::Status::OK;
        }
        return Platform::Status::ERROR;
    }

public:
    /**
     * Default constructor with no interfaces
     */
    DeviceDriverBase() 
        : state(DeviceState::Uninitialized),
          config_data(nullptr) {
        
        // Get system timing service
        timing_service = Middleware::SystemServices::SystemTiming::GetSharedInstance();
    }
    
    /**
     * Constructor with type-specific interfaces
     * 
     * @param i2c I2C interface
     * @param gpio GPIO interface
     * @param spi SPI interface
     * @param uart UART interface
     * @param timer Timer interface
     */
    DeviceDriverBase(
        std::shared_ptr<Platform::I2C::I2CInterface> i2c = nullptr,
        std::shared_ptr<Platform::GPIO::GpioInterface> gpio = nullptr,
        std::shared_ptr<Platform::TIM::TimerInterface> timer = nullptr
    ) : state(DeviceState::Uninitialized),
        config_data(nullptr),
        i2c_interface(i2c),
        gpio_interface(gpio),
        timer_interface(timer) {
        
        // Setup interface roles based on provided interfaces
        if (i2c) SetInterfaceRole(i2c, InterfaceRole::Primary);
        if (gpio) SetInterfaceRole(gpio, InterfaceRole::Control);
        if (timer) SetInterfaceRole(timer, InterfaceRole::Timing);
        
        // Get system timing service
        timing_service = Middleware::SystemServices::SystemTiming::GetSharedInstance();
    }
    
    /**
     * Destructor ensures proper cleanup
     */
    virtual ~DeviceDriverBase() {
        if (state != DeviceState::Uninitialized) {
            // Attempt to power down the device
            PowerControl(DevicePower::Off);
        }
        
        // Clear configuration data
        config_data = nullptr;
        
        // Clear interfaces
        interface_roles.clear();
        i2c_interface = nullptr;
        gpio_interface = nullptr;
        timer_interface = nullptr;
        
        // Callbacks will be cleared automatically
    }
    
    /**
     * Get the current state of the device
     * 
     * @return Current device state
     */
    DeviceState GetState() const override {
        return state;
    }
    
    /**
     * Get the hardware interface for a specific role
     * 
     * @param role The role of the hardware interface to retrieve
     * @return Shared pointer to the hardware interface, or nullptr if not available
     */
    std::shared_ptr<Platform::HwInterface> GetInterfaceByRole(InterfaceRole role) const override {
        auto it = interface_roles.find(role);
        if (it != interface_roles.end()) {
            return it->second;
        }
        return nullptr;
    }
    
    /**
     * Get the primary hardware interface for the device
     * 
     * @return Shared pointer to the primary hardware interface
     */
    std::shared_ptr<Platform::HwInterface> GetPrimaryInterface() const override {
        return GetInterfaceByRole(InterfaceRole::Primary);
    }
    
    /**
     * Set the role for a hardware interface
     * 
     * @param interface Hardware interface
     * @param role Role to assign to the interface
     */
    void SetInterfaceRole(std::shared_ptr<Platform::HwInterface> interface, InterfaceRole role) {
        if (!interface) return;
        
        interface_roles[role] = interface;
        
        // Also update type-specific pointers
        if (auto i2c = std::dynamic_pointer_cast<Platform::I2C::I2CInterface>(interface)) {
            i2c_interface = i2c;
        }
        else if (auto gpio = std::dynamic_pointer_cast<Platform::GPIO::GpioInterface>(interface)) {
            gpio_interface = gpio;
        }
        else if (auto timer = std::dynamic_pointer_cast<Platform::TIM::TimerInterface>(interface)) {
            timer_interface = timer;
        }
    }
    
    /**
     * Type-specific getters for hardware interfaces
     */
    std::shared_ptr<Platform::I2C::I2CInterface> GetI2CInterface() const {
        return i2c_interface;
    }
    
    std::shared_ptr<Platform::GPIO::GpioInterface> GetGPIOInterface() const {
        return gpio_interface;
    }
      
    std::shared_ptr<TimerInterface> GetTimerInterface() const {
        return timer_interface;
    }
    
    std::shared_ptr<Middleware::SystemServices::SystemTiming> GetTimingService() const {
        return timing_service;
    }
    
    /**
     * Base implementation of callback registration
     * 
     * @param event Event identifier
     * @param callback Callback function
     * @param param Parameter to pass to callback
     * @return Status code indicating success or failure
     */
    Platform::Status RegisterCallback(uint32_t event, void (*callback)(void* param), void* param) override {
        if (!callback) {
            return Platform::Status::INVALID_PARAM;
        }
        
        callbacks[event] = {callback, param, {}, false};
        return Platform::Status::OK;
    }
    
    /**
     * Register callback using std::function
     * 
     * @param event Event identifier
     * @param callback Callback function
     * @return Status code indicating success or failure
     */
    Platform::Status RegisterCallback(uint32_t event, std::function<void()> callback) override {
        if (!callback) {
            return Platform::Status::INVALID_PARAM;
        }
        
        callbacks[event] = {nullptr, nullptr, callback, true};
        return Platform::Status::OK;
    }
    
    /**
     * Basic initialization implementation - derived classes should extend this
     * 
     * @param config Pointer to device-specific configuration
     * @return Status code indicating success or failure
     */
    Platform::Status Init(void* config) override {
        // Store configuration
        config_data = config;
        
        // Validate interfaces
        Platform::Status status = ValidateInterfaces();
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Check timing service
        if (!timing_service || !timing_service->IsInitialized()) {
            return Platform::Status::DEPENDENCY_NOT_INITIALIZED;
        }
        
        // Initialize timing service if not already initialized
        if (!timing_service->IsInitialized()) {
            Middleware::SystemServices::SystemTimingConfig timing_config = {};
            timing_config.high_precision_timer = 2;  // TIM2
            timing_config.timer_frequency = Platform::MCU_CLK;
            timing_config.use_rtos_timing = false;
            timing_config.enable_power_saving = false;
            
            status = timing_service->Init(&timing_config);
            if (status != Platform::Status::OK) {
                return status;
            }
        }
        
        // Update state
        state = DeviceState::Initialized;
        return Platform::Status::OK;
    }
    
    /**
     * Basic power control implementation - derived classes should extend this
     * 
     * @param power_state Power state to set
     * @return Status code indicating success or failure
     */
    Platform::Status PowerControl(DevicePower power_state) override {
        // Check if initialized
        if (state == DeviceState::Uninitialized && power_state != DevicePower::Off) {
            return Platform::Status::NOT_INITIALIZED;
        }
        
        // Update state based on power mode
        switch (power_state) {
            case DevicePower::Full:
                state = DeviceState::Running;
                break;
                
            case DevicePower::Low:
                state = DeviceState::LowPower;
                break;
                
            case DevicePower::Off:
                state = DeviceState::Initialized; // Return to initialized but inactive state
                break;
                
            default:
                return Platform::Status::INVALID_PARAM;
        }
        
        return Platform::Status::OK;
    }
    
    /**
     * Default implementation of Control method - derived classes must override
     * 
     * @param command Command identifier
     * @param arg Command-specific argument
     * @return Status code indicating success or failure
     */
    Platform::Status Control(uint32_t command, void* arg) override {
        // Base implementation handles common commands
        switch (command) {
            case DEVICE_CTRL_RESET_DEVICE:
                state = DeviceState::Initialized;
                return Platform::Status::OK;
                
            default:
                return Platform::Status::NOT_SUPPORTED;
        }
    }
    
    /**
     * Default implementation of ReadData - derived classes must override
     * 
     * @param data Buffer to store read data
     * @param size Size of data to read
     * @return Status code indicating success or failure
     */
    Platform::Status ReadData(void* data, uint32_t size) override {
        return Platform::Status::NOT_SUPPORTED;
    }
    
    /**
     * Default implementation of WriteData - derived classes must override
     * 
     * @param data Data to write
     * @param size Size of data to write
     * @return Status code indicating success or failure
     */
    Platform::Status WriteData(const void* data, uint32_t size) override {
        return Platform::Status::NOT_SUPPORTED;
    }
};

} // namespace Drivers