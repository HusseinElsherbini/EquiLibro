// inc/drivers/device_driver.h

#ifndef DEVICE_DRIVER_H
#define DEVICE_DRIVER_H

#include "common/types.h"
#include "common/error_codes.h"
#include "hardware_abstraction/hw_interface.h"

// Device state enumeration
typedef enum {
    DEVICE_STATE_UNINITIALIZED,
    DEVICE_STATE_INITIALIZED,
    DEVICE_STATE_RUNNING,
    DEVICE_STATE_LOW_POWER,
    DEVICE_STATE_ERROR,
    DEVICE_STATE_SUSPENDED
} DeviceState_t;

// Device power modes
typedef enum {
    DEVICE_POWER_FULL,
    DEVICE_POWER_LOW,
    DEVICE_POWER_OFF
} DevicePower_t;

/**
 * Device driver interface that provides a consistent API
 * for interacting with sensors, actuators, and communication devices.
 */
typedef struct {
    // Current state of the device
    DeviceState_t state;
    
    // Reference to the hardware interface used by this device
    HW_Interface_t *hw_interface;
    
    // Device configuration data
    void *config;
    
    // Initialize the device with specific configuration
    Status_t (*Init)(void *config);
    
    // Control device power state
    Status_t (*PowerControl)(DevicePower_t state);
    
    // Control function for device-specific commands
    Status_t (*Control)(uint32_t command, void *arg);
    
    // Read data from the device
    Status_t (*ReadData)(void *data, uint32_t size);
    
    // Write data to the device
    Status_t (*WriteData)(const void *data, uint32_t size);
    
    // Register callback for device events
    Status_t (*RegisterCallback)(uint32_t event, void (*callback)(void *param), void *param);
    
} DeviceDriver_t;

// Common device control commands
#define DEVICE_CTRL_CALIBRATE       0x1001
#define DEVICE_CTRL_SELF_TEST       0x1002
#define DEVICE_CTRL_GET_INFO        0x1003
#define DEVICE_CTRL_SET_PARAMETER   0x1004
#define DEVICE_CTRL_GET_PARAMETER   0x1005
#define DEVICE_CTRL_RESET_DEVICE    0x1006

#endif /* DEVICE_DRIVER_H */