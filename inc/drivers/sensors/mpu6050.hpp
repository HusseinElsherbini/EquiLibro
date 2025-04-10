﻿#pragma once

#include "device_driver.hpp"
#include "hardware_abstraction/i2c.hpp"
#include "middleware/system_services/system_timing.hpp"
#include <array>
#include <cstdint>
#include <functional>
#include <memory>

namespace Drivers {
namespace Sensors {

    // Add to your MPU6050.hpp file
struct MPU6050Configuration {
    // Power Management
    bool device_reset;
    bool sleep;
    bool cycle;
    bool temp_dis;
    uint8_t clock_source;
    
    // Gyroscope Configuration
    uint8_t gyro_self_test;
    uint8_t gyro_range;
    float gyro_scale_factor;
    
    // Accelerometer Configuration
    uint8_t accel_self_test;
    uint8_t accel_range;
    float accel_scale_factor;
    
    // Digital Low Pass Filter Configuration
    uint8_t ext_sync;
    uint8_t dlpf_config;
    
    // Sample Rate Configuration
    uint8_t sample_rate_div;
    
    // Interrupt Configuration
    bool interrupt_data_rdy_en;
    bool interrupt_fifo_oflow_en;
    
    // INT Pin Configuration
    bool int_level;
    bool int_open;
    bool latch_int_en;
    bool int_rd_clear;
    bool fsync_int_level;
    bool fsync_int_en;
    bool i2c_bypass_en;
    
    // User Control
    bool fifo_en;
    bool i2c_mst_en;
    bool i2c_if_dis;
    bool fifo_reset;
    bool i2c_mst_reset;
    bool sig_cond_reset;
    
    // Offsets
    int16_t gyro_x_offset;
    int16_t gyro_y_offset;
    int16_t gyro_z_offset;
    int16_t accel_x_offset;
    int16_t accel_y_offset;
    int16_t accel_z_offset;
};
/**
 * MPU6050 Register Address Map
 */
namespace MPU6050Reg {
    constexpr uint8_t SELF_TEST_X      = 0x0D;  // Self test registers
    constexpr uint8_t SELF_TEST_Y      = 0x0E;
    constexpr uint8_t SELF_TEST_Z      = 0x0F;
    constexpr uint8_t SELF_TEST_A      = 0x10;
    constexpr uint8_t SMPLRT_DIV       = 0x19;  // Sample rate divider
    constexpr uint8_t CONFIG           = 0x1A;  // Configuration
    constexpr uint8_t GYRO_CONFIG      = 0x1B;  // Gyroscope configuration
    constexpr uint8_t ACCEL_CONFIG     = 0x1C;  // Accelerometer configuration
    constexpr uint8_t FIFO_EN          = 0x23;  // FIFO enable
    constexpr uint8_t I2C_MST_CTRL     = 0x24;  // I2C master control
    constexpr uint8_t I2C_SLV0_ADDR    = 0x25;  // I2C slave 0 address
    constexpr uint8_t I2C_SLV0_REG     = 0x26;  // I2C slave 0 register
    constexpr uint8_t I2C_SLV0_CTRL    = 0x27;  // I2C slave 0 control
    constexpr uint8_t INT_PIN_CFG      = 0x37;  // Interrupt pin configuration
    constexpr uint8_t INT_ENABLE       = 0x38;  // Interrupt enable
    constexpr uint8_t INT_STATUS       = 0x3A;  // Interrupt status
    constexpr uint8_t ACCEL_XOUT_H     = 0x3B;  // Accel X axis high byte
    constexpr uint8_t ACCEL_XOUT_L     = 0x3C;  // Accel X axis low byte
    constexpr uint8_t ACCEL_YOUT_H     = 0x3D;  // Accel Y axis high byte
    constexpr uint8_t ACCEL_YOUT_L     = 0x3E;  // Accel Y axis low byte
    constexpr uint8_t ACCEL_ZOUT_H     = 0x3F;  // Accel Z axis high byte
    constexpr uint8_t ACCEL_ZOUT_L     = 0x40;  // Accel Z axis low byte
    constexpr uint8_t TEMP_OUT_H       = 0x41;  // Temperature high byte
    constexpr uint8_t TEMP_OUT_L       = 0x42;  // Temperature low byte
    constexpr uint8_t GYRO_XOUT_H      = 0x43;  // Gyro X axis high byte
    constexpr uint8_t GYRO_XOUT_L      = 0x44;  // Gyro X axis low byte
    constexpr uint8_t GYRO_YOUT_H      = 0x45;  // Gyro Y axis high byte
    constexpr uint8_t GYRO_YOUT_L      = 0x46;  // Gyro Y axis low byte
    constexpr uint8_t GYRO_ZOUT_H      = 0x47;  // Gyro Z axis high byte
    constexpr uint8_t GYRO_ZOUT_L      = 0x48;  // Gyro Z axis low byte
    constexpr uint8_t SIGNAL_PATH_RESET = 0x68; // Signal path reset
    constexpr uint8_t USER_CTRL        = 0x6A;  // User control
    constexpr uint8_t PWR_MGMT_1       = 0x6B;  // Power management 1
    constexpr uint8_t PWR_MGMT_2       = 0x6C;  // Power management 2
    constexpr uint8_t FIFO_COUNTH      = 0x72;  // FIFO count high byte
    constexpr uint8_t FIFO_COUNTL      = 0x73;  // FIFO count low byte
    constexpr uint8_t FIFO_R_W         = 0x74;  // FIFO read/write
    constexpr uint8_t WHO_AM_I         = 0x75;  // Device ID (should be 0x68)
    // Accelerometer offset registers
    constexpr uint8_t XA_OFFS_H = 0x06;
    constexpr uint8_t XA_OFFS_L = 0x07;
    constexpr uint8_t YA_OFFS_H = 0x08;
    constexpr uint8_t YA_OFFS_L = 0x09;
    constexpr uint8_t ZA_OFFS_H = 0x0A;
    constexpr uint8_t ZA_OFFS_L = 0x0B;

    // Gyroscope offset registers
    constexpr uint8_t XG_OFFS_USRH = 0x13;
    constexpr uint8_t XG_OFFS_USRL = 0x14;
    constexpr uint8_t YG_OFFS_USRH = 0x15;
    constexpr uint8_t YG_OFFS_USRL = 0x16;
    constexpr uint8_t ZG_OFFS_USRH = 0x17;
    constexpr uint8_t ZG_OFFS_USRL = 0x18;
}

enum MPU6050Event {
    MPU6050_EVENT_DATA_READY = 0,    // New sensor data is available
    MPU6050_EVENT_DATA_AVAILABLE,    // Data transfer complete
    MPU6050_SENSOR_DATA_AVAILABLE,   // Sensor Data avalibale
    MPU6050_EVENT_MOTION_DETECTED,   // Motion detection triggered
    MPU6050_EVENT_FIFO_OVERFLOW,     // FIFO buffer overflow
    MPU6050_EVENT_ERROR,             // Error occurred
    MPU6050_EVENT_MAX                // Always the last value - not an actual event
};
// MPU6050 Control Commands
constexpr uint32_t MPU6050_CTRL_SET_GYRO_RANGE = 0x2001;
constexpr uint32_t MPU6050_CTRL_SET_ACCEL_RANGE = 0x2002;
constexpr uint32_t MPU6050_CTRL_SET_SAMPLE_RATE = 0x2003;
constexpr uint32_t MPU6050_CTRL_SET_DLPF_MODE = 0x2004;
constexpr uint32_t MPU6050_CTRL_ENABLE_INTERRUPT = 0x2005;
constexpr uint32_t MPU6050_CTRL_DISABLE_INTERRUPT = 0x2006;
constexpr uint32_t MPU6050_CTRL_GET_MOTION = 0x2007;
constexpr uint32_t MPU6050_CTRL_GET_ACCEL = 0x2008;
constexpr uint32_t MPU6050_CTRL_GET_GYRO = 0x2009;
constexpr uint32_t MPU6050_CTRL_GET_TEMP = 0x200A;
constexpr uint32_t MPU6050_CTRL_RESET_FIFO = 0x200B;
constexpr uint32_t MPU6050_CTRL_GET_FIFO_COUNT = 0x200C;
constexpr uint32_t MPU6050_CTRL_READ_FIFO = 0x200D;
constexpr uint32_t MPU6050_CTRL_ENABLE_MOTION_DETECT = 0x200E;
constexpr uint32_t MPU6050_CTRL_DISABLE_MOTION_DETECT = 0x200F;
constexpr uint32_t MPU6050_CTRL_RESET_DEVICE = 0x2010;
constexpr uint32_t MPU6050_CTRL_SET_CLOCK_SOURCE = 0x2011;
constexpr uint32_t MPU6050_CTRL_ENABLE_TEMP_SENSOR = 0x2012;
constexpr uint32_t MPU6050_CTRL_DISABLE_TEMP_SENSOR = 0x2013;
constexpr uint32_t MPU6050_CTRL_CALIBRATE_GYRO = 0x2014;
constexpr uint32_t MPU6050_CTRL_CALIBRATE_ACCEL = 0x2015;
constexpr uint32_t MPU6050_CTRL_SET_GYRO_OFFSETS = 0x2016;
constexpr uint32_t MPU6050_CTRL_GET_GYRO_OFFSETS = 0x2017;
constexpr uint32_t MPU6050_CTRL_SET_ACCEL_OFFSETS = 0x2018;
constexpr uint32_t MPU6050_CTRL_GET_ACCEL_OFFSETS = 0x2019;
constexpr uint32_t MPU6050_CTRL_RESET_OFFSETS = 0x201A;
constexpr uint32_t MPU6050_CTRL_CALIBRATE_ITERATIVE = 0x201B;

enum class OperationType {
    None,
    SensorDataRead,      // Reading sensor data
    RegisterConfig,      // Configuring a register
    CalibrationOp,       // Calibration operation
    DeviceCheck          // Device identification/status check
};

typedef enum {
    MPU6050_MODE_MANUAL,   // Application triggers readings explicitly
    MPU6050_MODE_AUTO      // Driver automatically reads on data-ready interrupt
} MPU6050_OperatingMode;
/**
 * MPU6050 Configuration Structure
 */
struct MPU6050Config {
    Platform::I2C::I2CConfig i2c_config;
    uint8_t device_address;   // I2C device address (default is 0x68, or 0x69 if AD0 pin is high)
    uint8_t gyro_range;       // Gyroscope range (0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s)
    uint8_t accel_range;      // Accelerometer range (0=±2g, 1=±4g, 2=±8g, 3=±16g)
    uint8_t dlpf_mode;        // Digital Low Pass Filter setting (0-7)
    uint8_t sample_rate_div;  // Sample Rate Divider (sample_rate = 1kHz / (1 + sample_rate_div))
    bool interrupt_enabled;   // Whether to enable data ready interrupt
    uint8_t clock_source;     // Clock source (0-7, default is 0 = internal 8MHz)
    bool use_fifo;            // Whether to use FIFO buffer
    bool enable_motion_detect; // Motion detection feature
    uint8_t motion_threshold; // Motion detection threshold (0-255)
    uint32_t i2c_timeout_ms;  // I2C operation timeout in milliseconds
    MPU6050_OperatingMode operating_mode; // Data acquisition mode
    bool enable_data_ready_interrupt;// Data ready interrupt configuration
    Platform::GPIO::Port data_ready_port;
    uint8_t data_ready_pin;
};
// In the MPU6050 configuration struct


/**
 * MPU6050 Sensor Data Structure
 */
struct MPU6050Data {
    int16_t accel_x;          // Accelerometer X-axis raw value
    int16_t accel_y;          // Accelerometer Y-axis raw value
    int16_t accel_z;          // Accelerometer Z-axis raw value
    int16_t temp;             // Temperature raw value
    int16_t gyro_x;           // Gyroscope X-axis raw value
    int16_t gyro_y;           // Gyroscope Y-axis raw value
    int16_t gyro_z;           // Gyroscope Z-axis raw value
    
    float accel_x_g;          // Accelerometer X-axis in g
    float accel_y_g;          // Accelerometer Y-axis in g
    float accel_z_g;          // Accelerometer Z-axis in g
    float temp_c;             // Temperature in degrees Celsius
    float gyro_x_dps;         // Gyroscope X-axis in degrees per second
    float gyro_y_dps;         // Gyroscope Y-axis in degrees per second
    float gyro_z_dps;         // Gyroscope Z-axis in degrees per second
    
    float filtered_angle;

    uint64_t timestamp_ms;    // Timestamp in milliseconds
};

/**
 * MPU6050 Calibration Data Structure
 */
struct MPU6050CalibrationData {
    int16_t accel_x_offset;   // Accelerometer X-axis offset
    int16_t accel_y_offset;   // Accelerometer Y-axis offset
    int16_t accel_z_offset;   // Accelerometer Z-axis offset
    int16_t gyro_x_offset;    // Gyroscope X-axis offset
    int16_t gyro_y_offset;    // Gyroscope Y-axis offset
    int16_t gyro_z_offset;    // Gyroscope Z-axis offset
};

/**
 * MPU6050 Driver Class
 * 
 * Provides a C++ interface to the MPU6050 6-axis motion tracking device.
 */
class MPU6050 : public DeviceDriver {
private:

    // Device state 
    DeviceState state;
    
    volatile OperationType current_operation = OperationType::None;

    // Default I2C address for MPU6050
    static constexpr uint8_t MPU6050_DEFAULT_ADDRESS = 0x68;
    
    // Device ID value returned by WHO_AM_I register
    static constexpr uint8_t MPU6050_DEVICE_ID = 0x68;
    
    struct CallbackInfo {
        void (*callback)(void* param);
        void* param;
        bool enabled;
    };
    
    // Inside MPU6050 class
    CallbackInfo callbacks[MPU6050_EVENT_MAX];

    // Configuration data
    MPU6050Config config;
    
    // Sensor data
    MPU6050Data sensor_data;
    
    // Calibration data
    MPU6050CalibrationData calibration;
    
    // Conversion factors for different ranges
    float accel_scale_factor;  // LSB per g
    float gyro_scale_factor;   // LSB per deg/s
    
    // Buffer for reading sensor data
    std::array<uint8_t, 14> sensor_buffer;
    
    // Non-blocking transfer tracking
    bool transfer_in_progress;
    
    // i2c interface pointer 
    Platform::I2C::I2CInterface* i2c_interface;

    // system timing interface pointer 
    Middleware::SystemServices::SystemTiming* timing_service;

    // Private helper methods
    Platform::Status ReadRegister(uint8_t reg_addr, uint8_t* data, uint32_t timeout = 100);
    Platform::Status WriteRegister(uint8_t reg_addr, uint8_t data, uint32_t timeout = 100);
    Platform::Status ReadRegisters(uint8_t reg_addr, uint8_t* data, uint8_t len, uint32_t timeout = 100);
    
    void ConvertRawData();
    void ParseSensorData();
    
    Platform::Status SetGyroRange(uint8_t range);
    Platform::Status SetAccelRange(uint8_t range);
    Platform::Status SetDLPFMode(uint8_t mode);
    Platform::Status SetSampleRateDiv(uint8_t rate_div);
    Platform::Status SetClockSource(uint8_t source);
    Platform::Status ResetDevice();
    Platform::Status VerifyDeviceID();
    Platform::Status CalibrateIterative();
    // Operating mode and state tracking
    MPU6050_OperatingMode current_mode;
    bool data_ready_pending;    // Flag for manual mode to indicate data is ready
 
public:
    /**
     * Constructor with I2C interface
     * 
     * @param i2c I2C interface
     */
    MPU6050();
    
    /**
     * Destructor
     */
    ~MPU6050() override;
    
    /**
     * Initialize the MPU6050 device
     * 
     * @param config_ptr Pointer to MPU6050 configuration
     * @return Status code indicating success or failure
     */
    Platform::Status Init(void* config_ptr) override;
    
    /**
     * Control MPU6050 power state
     * 
     * @param power_state Power state to set
     * @return Status code indicating success or failure
     */
    Platform::Status PowerControl(DevicePower power_state) override;
    
    /**
     * Read sensor data from MPU6050
     * 
     * @param data Buffer to store read data
     * @param size Size of data to read (must be sizeof(MPU6050Data))
     * @return Status code indicating success or failure
     */
    Platform::Status ReadData(void* data, uint32_t size) override;
    
    /**
     * Read sensor data from MPU6050 asynchronously
     * 
     * @return Status code indicating success or failure
     */
    Platform::Status ReadDataAsync();
    
    /**
     * Write data to MPU6050 (not typically used except for configuration)
     * 
     * @param data Data to write
     * @param size Size of data to write
     * @return Status code indicating success or failure
     */
    Platform::Status WriteData(const void* data, uint32_t size) override;
    
    /**
     * Control function for MPU6050 specific commands
     * 
     * @param command Command identifier
     * @param arg Command-specific argument
     * @return Status code indicating success or failure
     */
    Platform::Status Control(uint32_t command, void* arg) override;
    
    DeviceState GetState(void) const override;

    Platform::HwInterface* GetInterfaceByRole(InterfaceRole role) const override;

    Platform::HwInterface* GetPrimaryInterface() const override;

    Platform::Status RegisterCallback(uint32_t event, void (*callback)(void* param), void* param) override;

    Platform::Status RegisterCallback(uint32_t event, std::function<void()> callback) override;

    void GetProcessedData(MPU6050Data* data);

    // Gyroscope offset functions
    Platform::Status SetGyroXOffset(int16_t offset);
    Platform::Status SetGyroYOffset(int16_t offset);
    Platform::Status SetGyroZOffset(int16_t offset);
    Platform::Status SetGyroOffsets(int16_t x_offset, int16_t y_offset, int16_t z_offset);

    Platform::Status GetGyroXOffset(int16_t* offset);
    Platform::Status GetGyroYOffset(int16_t* offset);
    Platform::Status GetGyroZOffset(int16_t* offset);
    Platform::Status GetGyroOffsets(int16_t* x_offset, int16_t* y_offset, int16_t* z_offset);

    // Accelerometer offset functions
    Platform::Status SetAccelXOffset(int16_t offset);
    Platform::Status SetAccelYOffset(int16_t offset);
    Platform::Status SetAccelZOffset(int16_t offset);
    Platform::Status SetAccelOffsets(int16_t x_offset, int16_t y_offset, int16_t z_offset);

    Platform::Status GetAccelXOffset(int16_t* offset);
    Platform::Status GetAccelYOffset(int16_t* offset);
    Platform::Status GetAccelZOffset(int16_t* offset);
    Platform::Status GetAccelOffsets(int16_t* x_offset, int16_t* y_offset, int16_t* z_offset);

    // Combined function for setting all offsets at once
    Platform::Status SetAllOffsets(
        int16_t accel_x_offset, int16_t accel_y_offset, int16_t accel_z_offset,
        int16_t gyro_x_offset, int16_t gyro_y_offset, int16_t gyro_z_offset);

    // Helper for factory reset of offset registers
    Platform::Status ResetOffsets();
    bool IsDataReady();
    
    // Method to change operating mode at runtime
    Platform::Status SetOperatingMode(MPU6050_OperatingMode mode);

    static void DataReadyInterruptHandler(void* param);
    void HandleDataReadyInterrupt();
    static void I2CTransferCompleteHandler(void* param);
    void HandleI2CTransferComplete();
    
    MPU6050Data ProcessIMURawData();
    Platform::Status Configure(const MPU6050Configuration& config);
    Platform::Status ReadConfiguration(MPU6050Configuration* config_out);

    void TriggerCallback(MPU6050Event event);
    /**
     * Factory function for creating an MPU6050 driver instance
     * 
     * @param i2c I2C interface
     * @return static reference to MPU6050 driver
     */
    static MPU6050& CreateMPU6050(void);
};

} // namespace Sensors
} // namespace Drivers