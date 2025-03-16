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
    virtual Platform::HwInterface * GetInterfaceByRole(InterfaceRole role) const = 0;
    
    /**
     * Get the primary hardware interface for the device
     * 
     * @return Shared pointer to the primary hardware interface
     */
    virtual Platform::HwInterface * GetPrimaryInterface() const = 0;

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

    
} // namespace Drivers