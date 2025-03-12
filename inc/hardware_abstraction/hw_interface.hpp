#pragma once

#include "platform.hpp"

namespace Platform {
/**
 * Hardware interface abstract base class that provides a consistent API
 * for interacting with hardware peripherals.
 */
class HwInterface {
public:
    // Virtual destructor ensures proper cleanup for derived classes
    virtual ~HwInterface() = default;
    
    /**
     * Initialize the hardware peripheral with specific configuration
     * 
     * @param config Pointer to peripheral-specific configuration
     * @return Status code indicating success or failure
     */
    virtual Platform::Status Init(void* config) = 0;
    
    /**
     * De-initialize the hardware peripheral and release resources
     * 
     * @return Status code indicating success or failure
     */
    virtual Platform::Status DeInit() = 0;
    
    /**
     * Control the hardware peripheral with specific commands
     * 
     * @param command Control command identifier
     * @param param Command-specific parameter
     * @return Status code indicating success or failure
     */
    virtual Platform::Status Control(uint32_t command, void* param) = 0;
    
    /**
     * Read data from the hardware peripheral
     * 
     * @param buffer Buffer to store the read data
     * @param size Size of data to read (in bytes)
     * @param timeout Timeout for the operation (in milliseconds)
     * @return Status code indicating success or failure
     */
    virtual Platform::Status Read(void* buffer, uint16_t size, uint32_t timeout) = 0;
    
    /**
     * Write data to the hardware peripheral
     * 
     * @param data Data to write
     * @param size Size of data to write (in bytes)
     * @param timeout Timeout for the operation (in milliseconds)
     * @return Status code indicating success or failure
     */
    virtual Platform::Status Write(const void* data, uint16_t size, uint32_t timeout) = 0;
    
    /**
     * Register a callback function for hardware events
     * 
     * @param eventId Event identifier
     * @param callback Callback function pointer
     * @param param Parameter to pass to the callback function
     * @return Status code indicating success or failure
     */
    virtual Platform::Status RegisterCallback(uint32_t eventId, 
                                     void (*callback)(void* param), 
                                     void* param) = 0;
};

// Common hardware control commands - keeping these as constants
constexpr uint32_t HW_CTRL_ENABLE = 0x0001;
constexpr uint32_t HW_CTRL_DISABLE = 0x0002;
constexpr uint32_t HW_CTRL_RESET = 0x0003;
constexpr uint32_t HW_CTRL_SET_CONFIG = 0x0004;
constexpr uint32_t HW_CTRL_GET_STATUS = 0x0005;
constexpr uint32_t HW_CTRL_SET_POWER_MODE = 0x0006;

}