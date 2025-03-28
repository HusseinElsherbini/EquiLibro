#pragma once

#include "device_driver.hpp"
#include "sensors/mpu6050.hpp"
#include "hardware_abstraction/i2c.hpp"
#include "middleware/system_services/system_timing.hpp"
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <cmath>
#include "platform.hpp"
namespace Drivers {
namespace Sensors {

/*
 * MPU6050 Driver Implementation
 */
    /*
     * Read a single byte from MPU6050 register
     * 
     * @param reg_addr Register address
     * @param data Pointer to store the read byte
     * @param timeout Timeout in milliseconds
     * @return Status code indicating success or failure
     */
    Platform::Status MPU6050::ReadRegister(uint8_t reg_addr, uint8_t* data, uint32_t timeout) {
        if (!i2c_interface || !data) {
            return Platform::Status::ERROR;
        }

        return i2c_interface->MemoryRead(config.device_address, reg_addr, 1, data, 1, timeout);
    }

    /**
     * Write a single byte to MPU6050 register
     * 
     * @param reg_addr Register address
     * @param data Data byte to write
     * @param timeout Timeout in milliseconds
     * @return Status code indicating success or failure
     */
    Platform::Status MPU6050::WriteRegister(uint8_t reg_addr, uint8_t data, uint32_t timeout) {
        if (!i2c_interface) {
            return Platform::Status::ERROR;
        }

        return i2c_interface->MemoryWrite(config.device_address, reg_addr, 1, &data, 1, timeout);
    }
    
    /**
     * Read multiple bytes from MPU6050 registers
     * 
     * @param reg_addr Starting register address
     * @param data Buffer to store the read bytes
     * @param len Number of bytes to read
     * @param timeout Timeout in milliseconds
     * @return Status code indicating success or failure
     */
    Platform::Status MPU6050::ReadRegisters(uint8_t reg_addr, uint8_t* data, uint8_t len, uint32_t timeout) {
        if (!i2c_interface || !data || len == 0) {
            return Platform::Status::ERROR;
        }

        return i2c_interface->MemoryRead(config.device_address, reg_addr, 1, data, len, timeout);
    }
    
    /**
     * Convert raw sensor data to physical units
     */
    void MPU6050::ConvertRawData() {
        // Convert accelerometer values to g
        sensor_data.accel_x_g = static_cast<float>(sensor_data.accel_x) / accel_scale_factor;
        sensor_data.accel_y_g = static_cast<float>(sensor_data.accel_y) / accel_scale_factor;
        sensor_data.accel_z_g = static_cast<float>(sensor_data.accel_z) / accel_scale_factor;
        
        // Convert temperature to Celsius: temp = (TEMP_OUT / 340) + 36.53
        sensor_data.temp_c = static_cast<float>(sensor_data.temp) / 340.0f + 36.53f;
        
        // Convert gyroscope values to degrees per second
        sensor_data.gyro_x_dps = static_cast<float>(sensor_data.gyro_x) / gyro_scale_factor;
        sensor_data.gyro_y_dps = static_cast<float>(sensor_data.gyro_y) / gyro_scale_factor;
        sensor_data.gyro_z_dps = static_cast<float>(sensor_data.gyro_z) / gyro_scale_factor;
    }
    
    /**
     * Parse raw sensor data from buffer
     */
    void MPU6050::ParseSensorData() {
        // Get timestamp
        sensor_data.timestamp_ms = timing_service->GetMilliseconds();
        
        // Parse accelerometer data (big-endian)
        sensor_data.accel_x = (static_cast<int16_t>(sensor_buffer[0]) << 8) | sensor_buffer[1];
        sensor_data.accel_y = (static_cast<int16_t>(sensor_buffer[2]) << 8) | sensor_buffer[3];
        sensor_data.accel_z = (static_cast<int16_t>(sensor_buffer[4]) << 8) | sensor_buffer[5];
        
        // Parse temperature data
        sensor_data.temp = (static_cast<int16_t>(sensor_buffer[6]) << 8) | sensor_buffer[7];
        
        // Parse gyroscope data
        sensor_data.gyro_x = (static_cast<int16_t>(sensor_buffer[8]) << 8) | sensor_buffer[9];
        sensor_data.gyro_y = (static_cast<int16_t>(sensor_buffer[10]) << 8) | sensor_buffer[11];
        sensor_data.gyro_z = (static_cast<int16_t>(sensor_buffer[12]) << 8) | sensor_buffer[13];
    
        // Convert to physical units
        ConvertRawData();
    }
    
    MPU6050Data MPU6050::ProcessIMURawData() {
        // Calculate the filtered angle using your complementary filter logic
        float accel_angle = atan2f(sensor_data.accel_x_g, 
                              sqrtf(sensor_data.accel_y_g * sensor_data.accel_y_g + 
                                    sensor_data.accel_z_g * sensor_data.accel_z_g)) * 180.0f / Platform::PI;
        
        
        // Timestamp might need adjustment for proper timing
        uint64_t now = timing_service->GetMicroseconds();
        static uint64_t last_time = now;
        float dt = (now - last_time) / 1000000.0f;
        last_time = now;
        
        if (sensor_data.timestamp_ms == 0) {
            sensor_data.filtered_angle = accel_angle;
        } else {
            sensor_data.filtered_angle = 0.98f * (sensor_data.filtered_angle + sensor_data.gyro_x_dps * dt) + 0.02f * accel_angle;
        } 
        return sensor_data;
    }
    /**
     * Set gyroscope full-scale range
     * 
     * @param range Range setting (0-3)
     * @return Status code indicating success or failure
     */
    Platform::Status MPU6050::SetGyroRange(uint8_t range) {
        if (range > 3) {
            return Platform::Status::INVALID_PARAM;
        }
        
        uint8_t value;
        Platform::Status status = ReadRegister(MPU6050Reg::GYRO_CONFIG, &value);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Clear bits 3 and 4 and set new range
        value = (value & 0xE7) | (range << 3);
        status = WriteRegister(MPU6050Reg::GYRO_CONFIG, value);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Update scale factor based on range
        switch (range) {
            case 0: // ±250°/s
                gyro_scale_factor = 131.0f;
                break;
            case 1: // ±500°/s
                gyro_scale_factor = 65.5f;
                break;
            case 2: // ±1000°/s
                gyro_scale_factor = 32.8f;
                break;
            case 3: // ±2000°/s
                gyro_scale_factor = 16.4f;
                break;
        }
        
        // Update config
        config.gyro_range = range;
        
        return Platform::Status::OK;
    }
    /**
     * Triggers registered callback for a specific event
     * 
     * @param event Event identifier to trigger callback for
     */
    void MPU6050::TriggerCallback(MPU6050Event event) {
        // First check if the event is valid
        if (event >= MPU6050_EVENT_MAX) {
            // Invalid event, do nothing
            return;
        }
        
        // Check if a callback is registered and enabled for this event
        if (callbacks[event].enabled && callbacks[event].callback != nullptr) {
            // Call the registered callback with its parameter
            callbacks[event].callback(callbacks[event].param);
        }
        
    }
    /**
     * Set accelerometer full-scale range
     * 
     * @param range Range setting (0-3)
     * @return Status code indicating success or failure
     */
    Platform::Status MPU6050::SetAccelRange(uint8_t range) {
        if (range > 3) {
            return Platform::Status::INVALID_PARAM;
        }
        
        uint8_t value;
        Platform::Status status = ReadRegister(MPU6050Reg::ACCEL_CONFIG, &value);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Clear bits 3 and 4 and set new range
        value = (value & 0xE7) | (range << 3);
        status = WriteRegister(MPU6050Reg::ACCEL_CONFIG, value);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Update scale factor based on range
        switch (range) {
            case 0: // ±2g
                accel_scale_factor = 16384.0f;
                break;
            case 1: // ±4g
                accel_scale_factor = 8192.0f;
                break;
            case 2: // ±8g
                accel_scale_factor = 4096.0f;
                break;
            case 3: // ±16g
                accel_scale_factor = 2048.0f;
                break;
        }
        
        // Update config
        config.accel_range = range;
        
        return Platform::Status::OK;
    }
    
    /**
     * Set digital low-pass filter mode
     * 
     * @param mode DLPF mode (0-7)
     * @return Status code indicating success or failure
     */
    Platform::Status MPU6050::SetDLPFMode(uint8_t mode) {
        if (mode > 7) {
            return Platform::Status::INVALID_PARAM;
        }
        
        uint8_t value;
        Platform::Status status = ReadRegister(MPU6050Reg::CONFIG, &value);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Clear bits 0-2 and set new mode
        value = (value & 0xF8) | mode;
        status = WriteRegister(MPU6050Reg::CONFIG, value);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Update config
        config.dlpf_mode = mode;
        
        return Platform::Status::OK;
    }
    
    /**
     * Set sample rate divider
     * 
     * @param rate_div Sample rate divider value
     * @return Status code indicating success or failure
     */
    Platform::Status MPU6050::SetSampleRateDiv(uint8_t rate_div) {
        Platform::Status status = WriteRegister(MPU6050Reg::SMPLRT_DIV, rate_div);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Update config
        config.sample_rate_div = rate_div;
        
        return Platform::Status::OK;
    }
    
    /**
     * Set clock source
     * 
     * @param source Clock source (0-7)
     * @return Status code indicating success or failure
     */
    Platform::Status MPU6050::SetClockSource(uint8_t source) {
        if (source > 7) {
            return Platform::Status::INVALID_PARAM;
        }
        
        uint8_t value;
        Platform::Status status = ReadRegister(MPU6050Reg::PWR_MGMT_1, &value);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Clear bits 0-2 and set new source
        value = (value & 0xF8) | source;
        status = WriteRegister(MPU6050Reg::PWR_MGMT_1, value);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Update config
        config.clock_source = source;
        
        return Platform::Status::OK;
    }
    
    /**
     * Reset device
     * 
     * @return Status code indicating success or failure
     */
    Platform::Status MPU6050::ResetDevice() {

        // Set reset bit in PWR_MGMT_1 register
        Platform::Status status = WriteRegister(MPU6050Reg::PWR_MGMT_1, 0x80);
        if (status != Platform::Status::OK) {
            return status;
        }

        // Wait for reset to complete
        timing_service->DelayMilliseconds(100);
        
        // Clear reset bit and set clock source
        status = SetClockSource(config.clock_source);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        return Platform::Status::OK;
    }
    
    /**
     * Verify device identity by reading WHO_AM_I register
     * 
     * @return Status code indicating success or failure
     */
    Platform::Status MPU6050::VerifyDeviceID() {
        uint8_t who_am_i = 0;
        Platform::Status status = ReadRegister(MPU6050Reg::WHO_AM_I, &who_am_i);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        if (who_am_i != MPU6050_DEVICE_ID) {
            return Platform::Status::ERROR;
        }
        
        return Platform::Status::OK;
    }
    
    /**
     * Constructor with I2C interface
     * 
     * @param i2c I2C interface
     */
    MPU6050::MPU6050()
        : transfer_in_progress(false) {
        
        // Initialize with default values
        config.device_address = MPU6050_DEFAULT_ADDRESS;
        config.gyro_range = 0;
        config.accel_range = 0;
        config.dlpf_mode = 0;
        config.sample_rate_div = 0;
        config.interrupt_enabled = false;
        config.clock_source = 1;  // PLL with X-axis gyro reference
        config.use_fifo = false;
        config.enable_motion_detect = false;
        config.motion_threshold = 0;
        config.i2c_timeout_ms = 100;
        
        // Initialize scale factors for default ranges
        accel_scale_factor = 16384.0f;  // Default ±2g
        gyro_scale_factor = 131.0f;     // Default ±250°/s
        
        // Initialize calibration data
        calibration.accel_x_offset = 0;
        calibration.accel_y_offset = 0;
        calibration.accel_z_offset = 0;
        calibration.gyro_x_offset = 0;
        calibration.gyro_y_offset = 0;
        calibration.gyro_z_offset = 0;

        timing_service = &Middleware::SystemServices::SystemTiming::GetInstance();
    }
    
    /**
     * Destructor
     */
    MPU6050::~MPU6050() {
        // Make sure we try to power down the device
        if (state != DeviceState::Uninitialized) {
            PowerControl(DevicePower::Off);
        }
    }
    /**
     * Create an MPU6050 instance with the specified I2C interface
     * 
     * @param i2c Pointer to the I2C interface to use for communication
     * @return Reference to the newly created MPU6050 instance
     */
    MPU6050& MPU6050::CreateMPU6050(void) {
        // Create a static instance to ensure it persists beyond function scope
        static Drivers::Sensors::MPU6050 mpu6050_instance;

        return mpu6050_instance;
    }
    /**
     * Initialize the MPU6050 device
     * 
     * @param config_ptr Pointer to MPU6050 configuration
     * @return Status code indicating success or failure
     */
    Platform::Status MPU6050::Init(void* config_ptr) {

        // Copy configuration if provided
        if (config_ptr) {
            config = *static_cast<MPU6050Config*>(config_ptr);
        }

        Platform::Status status;
        this->i2c_interface = &Platform::I2C::I2CInterface::GetInstance(config.i2c_config.i2c_instance);
        // Check if we have a valid I2C interface
        if (!this->i2c_interface) {
            state = DeviceState::Error;
            return Platform::Status::ERROR;
        }
        
        // check if i2c interface is initialized 
        if(!this->i2c_interface->IsInitialized()){
            return Platform::Status::DEPENDENCY_NOT_INITIALIZED;
        }
        // Configure data ready interrupt if enabled
        if (config.enable_data_ready_interrupt) {
            // Configure GPIO pin for interrupt
            Platform::GPIO::GpioInterface& gpio = Platform::GPIO::GpioInterface::GetInstance();
            
            // Configure the pin as input
            Platform::GPIO::GpioConfig pin_config = {
                .port = config.data_ready_port,
                .pin = config.data_ready_pin,
                .mode = Platform::GPIO::Mode::Input,
                .pull = Platform::GPIO::Pull::PullDown, // Typically INT is active high
            };
            gpio.ConfigurePin(pin_config);
            
            // Configure and enable interrupt
            gpio.ConfigureInterrupt(config.data_ready_port, config.data_ready_pin, 
                                Platform::GPIO::InterruptTrigger::Rising);
            
            // Register callback
            gpio.RegisterInterruptCallback(config.data_ready_port, config.data_ready_pin,
                                        &DataReadyInterruptHandler, this);
            
            // Disable on startup
            gpio.EnableInterrupt(config.data_ready_port, config.data_ready_pin, false);
            
            // save sensor data acquisition mode (Manual or Auto)
            this->current_mode = config.operating_mode;
        }
        // Reset device to ensure clean state
        status = ResetDevice();
        if (status != Platform::Status::OK) {
            state = DeviceState::Error;
            return status;
        }
        
        // Wait for device to stabilize after reset
        timing_service->DelayMilliseconds(100);
        
        // Verify device identity
        status = VerifyDeviceID();
        if (status != Platform::Status::OK) {
            state = DeviceState::Error;
            return status;
        }
        
        // Wake up the device (clear sleep bit)
        status = WriteRegister(MPU6050Reg::PWR_MGMT_1, config.clock_source);
        if (status != Platform::Status::OK) {
            state = DeviceState::Error;
            return status;
        }
        
        // Configure device
        status = SetGyroRange(config.gyro_range);
        if (status != Platform::Status::OK) {
            state = DeviceState::Error;
            return status;
        }
        
        status = SetAccelRange(config.accel_range);
        if (status != Platform::Status::OK) {
            state = DeviceState::Error;
            return status;
        }
        
        status = SetDLPFMode(config.dlpf_mode);
        if (status != Platform::Status::OK) {
            state = DeviceState::Error;
            return status;
        }
        
        status = SetSampleRateDiv(config.sample_rate_div);
        if (status != Platform::Status::OK) {
            state = DeviceState::Error;
            return status;
        }
        
        // Configure FIFO if enabled
        if (config.use_fifo) {
            // Enable FIFO for accelerometer, temperature, and gyroscope
            status = WriteRegister(MPU6050Reg::FIFO_EN, 0x78);
            if (status != Platform::Status::OK) {
                state = DeviceState::Error;
                return status;
            }
            
            // Enable FIFO in user control register
            status = WriteRegister(MPU6050Reg::USER_CTRL, 0x40);
            if (status != Platform::Status::OK) {
                state = DeviceState::Error;
                return status;
            }
        }
        
        // Configure interrupts if enabled
        if (config.interrupt_enabled) {
            // Enable data ready interrupt
            status = WriteRegister(MPU6050Reg::INT_ENABLE, 0x01);
            if (status != Platform::Status::OK) {
                state = DeviceState::Error;
                return status;
            }
            
            // Configure interrupt pin (active high, push-pull, etc.)
            status = WriteRegister(MPU6050Reg::INT_PIN_CFG, 0x00);
            if (status != Platform::Status::OK) {
                state = DeviceState::Error;
                return status;
            }
        }
        
        // Configure motion detection if enabled
        if (config.enable_motion_detect) {
            // Set motion threshold
            status = WriteRegister(MPU6050Reg::ACCEL_CONFIG, 0x01); // Enable accel hardware intelligence
            if (status != Platform::Status::OK) {
                state = DeviceState::Error;
                return status;
            }
            
            status = WriteRegister(0x1F, config.motion_threshold); // Motion threshold register
            if (status != Platform::Status::OK) {
                state = DeviceState::Error;
                return status;
            }
            
            // Enable motion interrupt
            status = WriteRegister(MPU6050Reg::INT_ENABLE, 0x40); // Motion interrupt
            if (status != Platform::Status::OK) {
                state = DeviceState::Error;
                return status;
            }
        }
        
        // Register I2C transfer complete callback
        i2c_interface->RegisterCallback(
            static_cast<uint32_t>(Platform::I2C::I2CEvent::TransferComplete),
            I2CTransferCompleteHandler,
            this
        );
        
        // Update state
        state = DeviceState::Initialized;
        
        return Platform::Status::OK;
    }
    
    /**
     * Control MPU6050 power state
     * 
     * @param power_state Power state to set
     * @return Status code indicating success or failure
     */
    Platform::Status MPU6050::PowerControl(DevicePower power_state) {
        // Check if initialized
        if (state == DeviceState::Uninitialized && power_state != DevicePower::Off) {
            return Platform::Status::NOT_INITIALIZED;
        }
        
        // Get current power management register value
        uint8_t pwr_mgmt_1;
        Platform::Status status = ReadRegister(MPU6050Reg::PWR_MGMT_1, &pwr_mgmt_1);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Update power state based on requested mode
        switch (power_state) {
            case DevicePower::Full:
                // Clear sleep bit (bit 6) and set clock source
                pwr_mgmt_1 &= ~(1 << 6);
                pwr_mgmt_1 = (pwr_mgmt_1 & 0xF8) | config.clock_source;
                
                // Write updated register
                status = WriteRegister(MPU6050Reg::PWR_MGMT_1, pwr_mgmt_1);
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                // Disable cycle mode and wake up all sensors
                status = WriteRegister(MPU6050Reg::PWR_MGMT_2, 0x00);
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                state = DeviceState::Running;
                break;
                
            case DevicePower::Low:
                // Enable cycle mode (bit 5) and clear sleep bit (bit 6)
                pwr_mgmt_1 &= ~(1 << 6);
                pwr_mgmt_1 |= (1 << 5);
                
                // Write updated register
                status = WriteRegister(MPU6050Reg::PWR_MGMT_1, pwr_mgmt_1);
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                // Configure cycle rate and which sensors to keep active
                // For example, keep accelerometer active but disable gyro and temp
                status = WriteRegister(MPU6050Reg::PWR_MGMT_2, 0x07);
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                state = DeviceState::LowPower;
                break;
                
            case DevicePower::Off:
                // Set sleep bit (bit 6)
                pwr_mgmt_1 |= (1 << 6);
                
                // Write updated register
                status = WriteRegister(MPU6050Reg::PWR_MGMT_1, pwr_mgmt_1);
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                state = DeviceState::Initialized;
                break;
                
            default:
                return Platform::Status::INVALID_PARAM;
        }
        
        return Platform::Status::OK;
    }
    
    /**
     * Read sensor data from MPU6050
     * 
     * @param data Buffer to store read data
     * @param size Size of data to read (must be sizeof(MPU6050Data)) (blocking)
     * @return Status code indicating success or failure
     */
    Platform::Status MPU6050::ReadData(void* data, uint32_t size) {
        // Check if initialized
        if (state != DeviceState::Running) {
            return Platform::Status::NOT_INITIALIZED;
        }
        
        // Check parameters
        if (!data || size != sizeof(MPU6050Data)) {
            return Platform::Status::INVALID_PARAM;
        }
        
        // Check if a transfer is already in progress
        if (transfer_in_progress) {
            return Platform::Status::BUSY;
        }
        
        // Read all sensor data (accel, temp, gyro) in one transaction
        Platform::Status status = ReadRegisters(
            MPU6050Reg::ACCEL_XOUT_H, 
            sensor_buffer.data(), 
            sensor_buffer.size(), 
            config.i2c_timeout_ms
        );
        
        if (status != Platform::Status::OK) {
            return status;
        }
    
        ParseSensorData();

        *static_cast<MPU6050Data*>(data) = sensor_data;

        return Platform::Status::OK;
    }
    
    /**
     * Read sensor data from MPU6050 asynchronously
     * 
     * @return Status code indicating success or failure
     */
    Platform::Status MPU6050::ReadDataAsync() {
        // Check if initialized
        if (state != DeviceState::Initialized) {
            return Platform::Status::NOT_INITIALIZED;
        }
        
        // Check if a transfer is already in progress
        if (transfer_in_progress) {
            return Platform::Status::BUSY;
        }
        
        current_operation = OperationType::SensorDataRead;
        
        // Mark transfer as in progress
        transfer_in_progress = true;
        
        // Start non-blocking I2C read (timeout = 0 for non-blocking)
        Platform::Status status = i2c_interface->MemoryRead(
            config.device_address,
            MPU6050Reg::ACCEL_XOUT_H,
            1,
            sensor_buffer.data(),
            sensor_buffer.size(),
            0  // Non-blocking
        );
        
        if (status != Platform::Status::OK) {
            transfer_in_progress = false;
            return status;
        }
        
        return Platform::Status::OK;
    }
    
    void MPU6050::GetProcessedData(MPU6050Data* data){
    
            // Process the data (filter angles, etc.)
            *data = ProcessIMURawData();
            
        }
    /**
     * Write data to MPU6050 (not typically used except for configuration)
     * 
     * @param data Data to write
     * @param size Size of data to write
     * @return Status code indicating success or failure
     */
    Platform::Status MPU6050::WriteData(const void* data, uint32_t size)  {
        // This method is not typically used for MPU6050 general operation
        // Configuration is done through the Control method
        return Platform::Status::NOT_SUPPORTED;
    }
    
    /**
     * Control function for MPU6050 specific commands
     * 
     * @param command Command identifier
     * @param arg Command-specific argument
     * @return Status code indicating success or failure
     */
    Platform::Status MPU6050::Control(uint32_t command, void* arg) {
        // Check if initialized (except for reset command)
        if (state == DeviceState::Uninitialized && command != MPU6050_CTRL_RESET_DEVICE) {
            return Platform::Status::NOT_INITIALIZED;
        }
        
        // Process command
        switch (command) {
            
            case MPU6050_CTRL_SET_GYRO_RANGE: {
                    if (!arg) return Platform::Status::INVALID_PARAM;
                    return SetGyroRange(*static_cast<uint8_t*>(arg));
                }
            
            case MPU6050_CTRL_SET_ACCEL_RANGE: {
                    if (!arg) return Platform::Status::INVALID_PARAM;
                    return SetAccelRange(*static_cast<uint8_t*>(arg));
                }
            
            case MPU6050_CTRL_SET_SAMPLE_RATE: {
                    if (!arg) return Platform::Status::INVALID_PARAM;
                    return SetSampleRateDiv(*static_cast<uint8_t*>(arg));
                }
            
            case MPU6050_CTRL_SET_DLPF_MODE: {
                    if (!arg) return Platform::Status::INVALID_PARAM;
                    return SetDLPFMode(*static_cast<uint8_t*>(arg));
                }
            
            case MPU6050_CTRL_SET_CLOCK_SOURCE: {
                if (!arg) return Platform::Status::INVALID_PARAM;
                return SetClockSource(*static_cast<uint8_t*>(arg));
                }
           
            case MPU6050_CTRL_RESET_DEVICE: {
                return ResetDevice();
            }
           
            case MPU6050_CTRL_GET_MOTION: {
                if (!arg) return Platform::Status::INVALID_PARAM;
                
                // Read all motion data
                return ReadData(arg, sizeof(MPU6050Data));
            }
          
            case MPU6050_CTRL_GET_ACCEL: {
                if (!arg) return Platform::Status::INVALID_PARAM;
                
                // Read only accelerometer data
                std::array<uint8_t, 6> accel_buffer;
                Platform::Status status = ReadRegisters(
                    MPU6050Reg::ACCEL_XOUT_H, 
                    accel_buffer.data(), 
                    accel_buffer.size(), 
                    config.i2c_timeout_ms
                );
                
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                // Parse data
                float* accel_data = static_cast<float*>(arg);
                int16_t accel_x = (static_cast<int16_t>(accel_buffer[0]) << 8) | accel_buffer[1];
                int16_t accel_y = (static_cast<int16_t>(accel_buffer[2]) << 8) | accel_buffer[3];
                int16_t accel_z = (static_cast<int16_t>(accel_buffer[4]) << 8) | accel_buffer[5];
                
                // Apply calibration
                accel_x -= calibration.accel_x_offset;
                accel_y -= calibration.accel_y_offset;
                accel_z -= calibration.accel_z_offset;
                
                // Convert to g
                accel_data[0] = static_cast<float>(accel_x) / accel_scale_factor;
                accel_data[1] = static_cast<float>(accel_y) / accel_scale_factor;
                accel_data[2] = static_cast<float>(accel_z) / accel_scale_factor;
                
                return Platform::Status::OK;
            }
                
            case MPU6050_CTRL_GET_GYRO: {
                if (!arg) return Platform::Status::INVALID_PARAM;
                
                // Read only gyroscope data
                std::array<uint8_t, 6> gyro_buffer;
                Platform::Status status = ReadRegisters(
                    MPU6050Reg::GYRO_XOUT_H, 
                    gyro_buffer.data(), 
                    gyro_buffer.size(), 
                    config.i2c_timeout_ms
                );
                
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                // Parse data
                float* gyro_data = static_cast<float*>(arg);
                int16_t gyro_x = (static_cast<int16_t>(gyro_buffer[0]) << 8) | gyro_buffer[1];
                int16_t gyro_y = (static_cast<int16_t>(gyro_buffer[2]) << 8) | gyro_buffer[3];
                int16_t gyro_z = (static_cast<int16_t>(gyro_buffer[4]) << 8) | gyro_buffer[5];
                
                // Apply calibration
                gyro_x -= calibration.gyro_x_offset;
                gyro_y -= calibration.gyro_y_offset;
                gyro_z -= calibration.gyro_z_offset;
                
                // Convert to degrees per second
                gyro_data[0] = static_cast<float>(gyro_x) / gyro_scale_factor;
                gyro_data[1] = static_cast<float>(gyro_y) / gyro_scale_factor;
                gyro_data[2] = static_cast<float>(gyro_z) / gyro_scale_factor;
                
                return Platform::Status::OK;
            }
                
            case MPU6050_CTRL_GET_TEMP: {
                if (!arg) return Platform::Status::INVALID_PARAM;
                
                // Read only temperature data
                std::array<uint8_t, 2> temp_buffer;
                Platform::Status status = ReadRegisters(
                    MPU6050Reg::TEMP_OUT_H, 
                    temp_buffer.data(), 
                    temp_buffer.size(), 
                    config.i2c_timeout_ms
                );
                
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                // Parse data
                int16_t temp = (static_cast<int16_t>(temp_buffer[0]) << 8) | temp_buffer[1];
                
                // Convert to Celsius
                *static_cast<float*>(arg) = static_cast<float>(temp) / 340.0f + 36.53f;
                
                return Platform::Status::OK;
            }
                
            case MPU6050_CTRL_ENABLE_INTERRUPT: {
                // Enable data ready interrupt
                Platform::Status status = WriteRegister(MPU6050Reg::INT_ENABLE, 0x01);
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                config.interrupt_enabled = true;

                if(config.enable_data_ready_interrupt){
                    
                    Platform::GPIO::GpioInterface& gpio = Platform::GPIO::GpioInterface::GetInstance();

                    status = gpio.EnableInterrupt(config.data_ready_port, config.data_ready_pin, true);
                    if (status != Platform::Status::OK) {
                        return status;
                    }
                }
                return Platform::Status::OK;
            }
                
            case MPU6050_CTRL_DISABLE_INTERRUPT: {
                // Disable interrupts
                Platform::Status status = WriteRegister(MPU6050Reg::INT_ENABLE, 0x00);
                if (status != Platform::Status::OK) {
                    return status;
                }
                if(config.enable_data_ready_interrupt){
                    this->config.enable_data_ready_interrupt = false;
                    Platform::GPIO::GpioInterface& gpio = Platform::GPIO::GpioInterface::GetInstance();

                    status = gpio.EnableInterrupt(config.data_ready_port, config.data_ready_pin, false);
                    if (status != Platform::Status::OK) {
                        return status;
                    }
                }
                config.interrupt_enabled = false;
                return Platform::Status::OK;
            }
                
            case MPU6050_CTRL_RESET_FIFO: {
                // Reset FIFO
                uint8_t user_ctrl;
                Platform::Status status = ReadRegister(MPU6050Reg::USER_CTRL, &user_ctrl);
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                // Set FIFO reset bit
                user_ctrl |= (1 << 2);
                status = WriteRegister(MPU6050Reg::USER_CTRL, user_ctrl);
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                // Clear FIFO reset bit
                user_ctrl &= ~(1 << 2);
                status = WriteRegister(MPU6050Reg::USER_CTRL, user_ctrl);
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                return Platform::Status::OK;
            }
                
            case MPU6050_CTRL_GET_FIFO_COUNT: {
                if (!arg) return Platform::Status::INVALID_PARAM;
                
                // Read FIFO count
                std::array<uint8_t, 2> fifo_count_buffer;
                Platform::Status status = ReadRegisters(
                    MPU6050Reg::FIFO_COUNTH, 
                    fifo_count_buffer.data(), 
                    fifo_count_buffer.size(), 
                    config.i2c_timeout_ms
                );
                
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                // Calculate count
                uint16_t count = (static_cast<uint16_t>(fifo_count_buffer[0]) << 8) | fifo_count_buffer[1];
                *static_cast<uint16_t*>(arg) = count;
                
                return Platform::Status::OK;
            }
                
            case MPU6050_CTRL_READ_FIFO: {
                if (!arg) return Platform::Status::INVALID_PARAM;
                
                // Structure to hold FIFO read parameters
                struct FifoReadParams {
                    uint8_t* buffer;
                    uint16_t count;
                };
                
                FifoReadParams* params = static_cast<FifoReadParams*>(arg);
                
                // Read FIFO data
                Platform::Status status = i2c_interface->MemoryRead(
                    config.device_address,
                    MPU6050Reg::FIFO_R_W,
                    1,
                    params->buffer,
                    params->count,
                    config.i2c_timeout_ms
                );
                
                return status;
            }
                
            case MPU6050_CTRL_ENABLE_MOTION_DETECT: {
                if (!arg) return Platform::Status::INVALID_PARAM;
                
                uint8_t threshold = *static_cast<uint8_t*>(arg);
                
                // Set motion threshold
                Platform::Status status = WriteRegister(0x1F, threshold);
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                // Enable motion detection in accelerometer config
                uint8_t accel_config;
                status = ReadRegister(MPU6050Reg::ACCEL_CONFIG, &accel_config);
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                accel_config |= 0x01;  // Enable accel hardware intelligence
                status = WriteRegister(MPU6050Reg::ACCEL_CONFIG, accel_config);
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                // Enable motion interrupt
                status = WriteRegister(MPU6050Reg::INT_ENABLE, 0x40);
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                config.enable_motion_detect = true;
                config.motion_threshold = threshold;
                
                return Platform::Status::OK;
            }
                
            case MPU6050_CTRL_DISABLE_MOTION_DETECT: {
                // Disable motion interrupt
                Platform::Status status = WriteRegister(MPU6050Reg::INT_ENABLE, 0x00);
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                config.enable_motion_detect = false;
                
                return Platform::Status::OK;
            }
                
            case MPU6050_CTRL_CALIBRATE_GYRO: {
                // Calibrate gyroscope by measuring offset at rest
                
                // Make sure device is in full power mode
                if (state != DeviceState::Running) {
                    Platform::Status status = PowerControl(DevicePower::Full);
                    if (status != Platform::Status::OK) {
                        return status;
                    }
                }
                
                // Set gyro range to most sensitive
                uint8_t original_range = config.gyro_range;
                Platform::Status status = SetGyroRange(0);  // ±250°/s
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                // Take multiple samples and average
                constexpr int num_samples = 100;
                int32_t gyro_x_sum = 0;
                int32_t gyro_y_sum = 0;
                int32_t gyro_z_sum = 0;
                
                for (int i = 0; i < num_samples; i++) {
                    std::array<uint8_t, 6> gyro_buffer;
                    status = ReadRegisters(
                        MPU6050Reg::GYRO_XOUT_H, 
                        gyro_buffer.data(), 
                        gyro_buffer.size(), 
                        config.i2c_timeout_ms
                    );
                    
                    if (status != Platform::Status::OK) {
                        // Restore original range
                        SetGyroRange(original_range);
                        return status;
                    }
                    
                    int16_t gyro_x = (static_cast<int16_t>(gyro_buffer[0]) << 8) | gyro_buffer[1];
                    int16_t gyro_y = (static_cast<int16_t>(gyro_buffer[2]) << 8) | gyro_buffer[3];
                    int16_t gyro_z = (static_cast<int16_t>(gyro_buffer[4]) << 8) | gyro_buffer[5];
                    
                    gyro_x_sum += gyro_x;
                    gyro_y_sum += gyro_y;
                    gyro_z_sum += gyro_z;
                    
                    // Small delay between samples
                    timing_service->DelayMilliseconds(10);
                }
                
                // Calculate average
                calibration.gyro_x_offset = gyro_x_sum / num_samples;
                calibration.gyro_y_offset = gyro_y_sum / num_samples;
                calibration.gyro_z_offset = gyro_z_sum / num_samples;
                
                // Restore original range
                status = SetGyroRange(original_range);
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                return Platform::Status::OK;
            }
                
            case MPU6050_CTRL_CALIBRATE_ACCEL: {
                // Calibrate accelerometer assuming device is flat (Z-axis = 1g)
                
                // Make sure device is in full power mode
                if (state != DeviceState::Running) {
                    Platform::Status status = PowerControl(DevicePower::Full);
                    if (status != Platform::Status::OK) {
                        return status;
                    }
                }
                
                // Set accel range to most sensitive
                uint8_t original_range = config.accel_range;
                Platform::Status status = SetAccelRange(0);  // ±2g
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                // Take multiple samples and average
                constexpr int num_samples = 100;
                int32_t accel_x_sum = 0;
                int32_t accel_y_sum = 0;
                int32_t accel_z_sum = 0;
                
                for (int i = 0; i < num_samples; i++) {
                    std::array<uint8_t, 6> accel_buffer;
                    status = ReadRegisters(
                        MPU6050Reg::ACCEL_XOUT_H, 
                        accel_buffer.data(), 
                        accel_buffer.size(), 
                        config.i2c_timeout_ms
                    );
                    
                    if (status != Platform::Status::OK) {
                        // Restore original range
                        SetAccelRange(original_range);
                        return status;
                    }
                    
                    int16_t accel_x = (static_cast<int16_t>(accel_buffer[0]) << 8) | accel_buffer[1];
                    int16_t accel_y = (static_cast<int16_t>(accel_buffer[2]) << 8) | accel_buffer[3];
                    int16_t accel_z = (static_cast<int16_t>(accel_buffer[4]) << 8) | accel_buffer[5];
                    
                    accel_x_sum += accel_x;
                    accel_y_sum += accel_y;
                    accel_z_sum += accel_z;
                    
                    // Small delay between samples
                    timing_service->DelayMilliseconds(10);
                }
                
                // Calculate average
                calibration.accel_x_offset = accel_x_sum / num_samples;
                calibration.accel_y_offset = accel_y_sum / num_samples;
                
                // For Z axis, we need to account for gravity (1g)
                int16_t accel_z_avg = accel_z_sum / num_samples;
                int16_t expected_1g = static_cast<int16_t>(accel_scale_factor);  // Expected value for 1g
                calibration.accel_z_offset = accel_z_avg - expected_1g;
                
                // Restore original range
                status = SetAccelRange(original_range);
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                return Platform::Status::OK;
            }

            case MPU6050_CTRL_SET_GYRO_OFFSETS: {
                if (!arg) return Platform::Status::INVALID_PARAM;
                
                int16_t* offsets = static_cast<int16_t*>(arg);
                return SetGyroOffsets(offsets[0], offsets[1], offsets[2]);
            }

            case MPU6050_CTRL_GET_GYRO_OFFSETS: {
                if (!arg) return Platform::Status::INVALID_PARAM;
                
                int16_t* offsets = static_cast<int16_t*>(arg);
                return GetGyroOffsets(&offsets[0], &offsets[1], &offsets[2]);
            }

            case MPU6050_CTRL_SET_ACCEL_OFFSETS: {
                if (!arg) return Platform::Status::INVALID_PARAM;
                
                int16_t* offsets = static_cast<int16_t*>(arg);
                return SetAccelOffsets(offsets[0], offsets[1], offsets[2]);
            }

            case MPU6050_CTRL_GET_ACCEL_OFFSETS: {
                if (!arg) return Platform::Status::INVALID_PARAM;
                
                int16_t* offsets = static_cast<int16_t*>(arg);
                return GetAccelOffsets(&offsets[0], &offsets[1], &offsets[2]);
            }

            case MPU6050_CTRL_RESET_OFFSETS: {
                return ResetOffsets();
            }  
            
            case MPU6050_CTRL_CALIBRATE_ITERATIVE: {
                return CalibrateIterative();
            }
            
            default: {
                // Try the base class implementation for common commands
                return Platform::Status::INVALID_PARAM;
            }
        }
    }

    DeviceState MPU6050::GetState() const {
        return state;
    }
    
    Platform::HwInterface* MPU6050::GetInterfaceByRole(InterfaceRole role) const {
        if (role == InterfaceRole::Primary) {
            return i2c_interface;
        }
        return nullptr;
    }
    
    Platform::HwInterface* MPU6050::GetPrimaryInterface() const {
        return i2c_interface;
    }
    
    Platform::Status MPU6050::RegisterCallback(uint32_t event, void (*callback)(void* param), void* param) {
        if (event >= MPU6050_EVENT_MAX) {
            return Platform::Status::INVALID_PARAM;
        }
        
        callbacks[event].callback = callback;
        callbacks[event].param = param;
        callbacks[event].enabled = (callback != nullptr);
        
        return Platform::Status::OK;
    }
    
    Platform::Status MPU6050::RegisterCallback(uint32_t event, std::function<void()> callback) {
        return Platform::Status::NOT_SUPPORTED;
    }
    // Add the static interrupt handler method
    void MPU6050::DataReadyInterruptHandler(void* param) {
        MPU6050* instance = static_cast<MPU6050*>(param);
        if (instance) {
            instance->HandleDataReadyInterrupt();
        }
    }

    void MPU6050::HandleDataReadyInterrupt() {

        // If a callback is registered for data-ready events, trigger it
        TriggerCallback(MPU6050Event::MPU6050_EVENT_DATA_READY);
        
        // Start an I2C transaction if in auto mode
        if (current_mode == MPU6050_MODE_AUTO) {
            Platform::Status status = ReadDataAsync();
            if(status == Platform::Status::BUSY){
                //TODO:: add error checking mechanism 
            }
        }
        else{
            data_ready_pending = true;
        }
    }

    void MPU6050::I2CTransferCompleteHandler(void* param){
        MPU6050* instance = static_cast<MPU6050*>(param);
        if (instance) {
            instance->HandleI2CTransferComplete();
        }
    }
    
    Platform::Status MPU6050::SetOperatingMode(MPU6050_OperatingMode mode) {
        current_mode = mode;
        return Platform::Status::OK;
    }   
    
    void MPU6050::HandleI2CTransferComplete(void) {

        // Save the operation type before resetting it
        volatile OperationType completed_operation = this->current_operation;
        
        // Reset the transfer state and operation type immediately
        transfer_in_progress = false;
        current_operation = OperationType::None;  // Reset here
        
        // Only process sensor data operations
        if (completed_operation == OperationType::SensorDataRead) {
            // Process the raw data
            ParseSensorData();
            ProcessIMURawData();
            
            // Trigger the correct event for sensor data
            TriggerCallback(MPU6050Event::MPU6050_EVENT_DATA_AVAILABLE);
        }
        // Handle other operation types as needed
        else if (completed_operation == OperationType::CalibrationOp) {
            // Maybe trigger a calibration complete event
        }
    }

    bool MPU6050::IsDataReady() {
        // If interrupts are disabled, check the MPU6050 status register directly
        if (!config.enable_data_ready_interrupt) {
            uint8_t status = 0;
            Platform::Status result = ReadRegister(MPU6050Reg::INT_STATUS, &status);
            return (result == Platform::Status::OK) && (status & 0x01);
        }
        
        // Otherwise return the flag set by the interrupt handler
        return data_ready_pending;
    }

    // Gyroscope offset functions
    Platform::Status MPU6050::SetGyroXOffset(int16_t offset) {
        // Write high byte
        Platform::Status status = WriteRegister(MPU6050Reg::XG_OFFS_USRH, static_cast<uint8_t>((offset >> 8) & 0xFF));
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Write low byte
        status = WriteRegister(MPU6050Reg::XG_OFFS_USRL, static_cast<uint8_t>(offset & 0xFF));
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Update calibration data
        calibration.gyro_x_offset = offset;
        
        return Platform::Status::OK;
    }

    Platform::Status MPU6050::SetGyroYOffset(int16_t offset) {
        // Write high byte
        Platform::Status status = WriteRegister(MPU6050Reg::YG_OFFS_USRH, static_cast<uint8_t>((offset >> 8) & 0xFF));
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Write low byte
        status = WriteRegister(MPU6050Reg::YG_OFFS_USRL, static_cast<uint8_t>(offset & 0xFF));
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Update calibration data
        calibration.gyro_y_offset = offset;
        
        return Platform::Status::OK;
    }

    Platform::Status MPU6050::SetGyroZOffset(int16_t offset) {
        // Write high byte
        Platform::Status status = WriteRegister(MPU6050Reg::ZG_OFFS_USRH, static_cast<uint8_t>((offset >> 8) & 0xFF));
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Write low byte
        status = WriteRegister(MPU6050Reg::ZG_OFFS_USRL, static_cast<uint8_t>(offset & 0xFF));
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Update calibration data
        calibration.gyro_z_offset = offset;
        
        return Platform::Status::OK;
    }

    Platform::Status MPU6050::SetGyroOffsets(int16_t x_offset, int16_t y_offset, int16_t z_offset) {
        Platform::Status status;
        
        status = SetGyroXOffset(x_offset);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        status = SetGyroYOffset(y_offset);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        status = SetGyroZOffset(z_offset);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        return Platform::Status::OK;
    }

    Platform::Status MPU6050::GetGyroXOffset(int16_t* offset) {
        if (offset == nullptr) {
            return Platform::Status::INVALID_PARAM;
        }
        
        uint8_t high_byte, low_byte;
        
        // Read high byte
        Platform::Status status = ReadRegister(MPU6050Reg::XG_OFFS_USRH, &high_byte);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Read low byte
        status = ReadRegister(MPU6050Reg::XG_OFFS_USRL, &low_byte);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Combine bytes into 16-bit value
        *offset = (static_cast<int16_t>(high_byte) << 8) | low_byte;
        
        return Platform::Status::OK;
    }

    Platform::Status MPU6050::GetGyroYOffset(int16_t* offset) {
        if (offset == nullptr) {
            return Platform::Status::INVALID_PARAM;
        }
        
        uint8_t high_byte, low_byte;
        
        // Read high byte
        Platform::Status status = ReadRegister(MPU6050Reg::YG_OFFS_USRH, &high_byte);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Read low byte
        status = ReadRegister(MPU6050Reg::YG_OFFS_USRL, &low_byte);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Combine bytes into 16-bit value
        *offset = (static_cast<int16_t>(high_byte) << 8) | low_byte;
        
        return Platform::Status::OK;
    }

    Platform::Status MPU6050::GetGyroZOffset(int16_t* offset) {
        if (offset == nullptr) {
            return Platform::Status::INVALID_PARAM;
        }
        
        uint8_t high_byte, low_byte;
        
        // Read high byte
        Platform::Status status = ReadRegister(MPU6050Reg::ZG_OFFS_USRH, &high_byte);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Read low byte
        status = ReadRegister(MPU6050Reg::ZG_OFFS_USRL, &low_byte);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Combine bytes into 16-bit value
        *offset = (static_cast<int16_t>(high_byte) << 8) | low_byte;
        
        return Platform::Status::OK;
    }

    Platform::Status MPU6050::GetGyroOffsets(int16_t* x_offset, int16_t* y_offset, int16_t* z_offset) {
        if (x_offset == nullptr || y_offset == nullptr || z_offset == nullptr) {
            return Platform::Status::INVALID_PARAM;
        }
        
        Platform::Status status;
        
        status = GetGyroXOffset(x_offset);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        status = GetGyroYOffset(y_offset);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        status = GetGyroZOffset(z_offset);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        return Platform::Status::OK;
    }

    // Accelerometer offset functions

    Platform::Status MPU6050::SetAccelXOffset(int16_t offset) {
        // For accelerometer, bit 0 of the low byte must be 0 (reserved)
        offset &= 0xFFFE;
        
        // Write high byte
        Platform::Status status = WriteRegister(MPU6050Reg::XA_OFFS_H, static_cast<uint8_t>((offset >> 8) & 0xFF));
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Write low byte
        status = WriteRegister(MPU6050Reg::XA_OFFS_L, static_cast<uint8_t>(offset & 0xFF));
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Update calibration data
        calibration.accel_x_offset = offset;
        
        return Platform::Status::OK;
    }

    Platform::Status MPU6050::SetAccelYOffset(int16_t offset) {
        // For accelerometer, bit 0 of the low byte must be 0 (reserved)
        offset &= 0xFFFE;
        
        // Write high byte
        Platform::Status status = WriteRegister(MPU6050Reg::YA_OFFS_H, static_cast<uint8_t>((offset >> 8) & 0xFF));
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Write low byte
        status = WriteRegister(MPU6050Reg::YA_OFFS_L, static_cast<uint8_t>(offset & 0xFF));
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Update calibration data
        calibration.accel_y_offset = offset;
        
        return Platform::Status::OK;
    }

    Platform::Status MPU6050::SetAccelZOffset(int16_t offset) {
        // For accelerometer, bit 0 of the low byte must be 0 (reserved)
        offset &= 0xFFFE;
        
        // Write high byte
        Platform::Status status = WriteRegister(MPU6050Reg::ZA_OFFS_H, static_cast<uint8_t>((offset >> 8) & 0xFF));
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Write low byte
        status = WriteRegister(MPU6050Reg::ZA_OFFS_L, static_cast<uint8_t>(offset & 0xFF));
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Update calibration data
        calibration.accel_z_offset = offset;
        
        return Platform::Status::OK;
    }

    Platform::Status MPU6050::SetAccelOffsets(int16_t x_offset, int16_t y_offset, int16_t z_offset) {
        Platform::Status status;
        
        status = SetAccelXOffset(x_offset);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        status = SetAccelYOffset(y_offset);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        status = SetAccelZOffset(z_offset);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        return Platform::Status::OK;
    }

    Platform::Status MPU6050::GetAccelXOffset(int16_t* offset) {
        if (offset == nullptr) {
            return Platform::Status::INVALID_PARAM;
        }
        
        uint8_t high_byte, low_byte;
        
        // Read high byte
        Platform::Status status = ReadRegister(MPU6050Reg::XA_OFFS_H, &high_byte);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Read low byte
        status = ReadRegister(MPU6050Reg::XA_OFFS_L, &low_byte);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Combine bytes into 16-bit value
        *offset = (static_cast<int16_t>(high_byte) << 8) | low_byte;
        
        return Platform::Status::OK;
    }

    Platform::Status MPU6050::GetAccelYOffset(int16_t* offset) {
        if (offset == nullptr) {
            return Platform::Status::INVALID_PARAM;
        }
        
        uint8_t high_byte, low_byte;
        
        // Read high byte
        Platform::Status status = ReadRegister(MPU6050Reg::YA_OFFS_H, &high_byte);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Read low byte
        status = ReadRegister(MPU6050Reg::YA_OFFS_L, &low_byte);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Combine bytes into 16-bit value
        *offset = (static_cast<int16_t>(high_byte) << 8) | low_byte;
        
        return Platform::Status::OK;
    }

    Platform::Status MPU6050::GetAccelZOffset(int16_t* offset) {
        if (offset == nullptr) {
            return Platform::Status::INVALID_PARAM;
        }
        
        uint8_t high_byte, low_byte;
        
        // Read high byte
        Platform::Status status = ReadRegister(MPU6050Reg::ZA_OFFS_H, &high_byte);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Read low byte
        status = ReadRegister(MPU6050Reg::ZA_OFFS_L, &low_byte);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Combine bytes into 16-bit value
        *offset = (static_cast<int16_t>(high_byte) << 8) | low_byte;
        
        return Platform::Status::OK;
    }

    Platform::Status MPU6050::GetAccelOffsets(int16_t* x_offset, int16_t* y_offset, int16_t* z_offset) {
        if (x_offset == nullptr || y_offset == nullptr || z_offset == nullptr) {
            return Platform::Status::INVALID_PARAM;
        }
        
        Platform::Status status;
        
        status = GetAccelXOffset(x_offset);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        status = GetAccelYOffset(y_offset);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        status = GetAccelZOffset(z_offset);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        return Platform::Status::OK;
    }

    // Combined function for setting all offsets at once
    Platform::Status MPU6050::SetAllOffsets(
        int16_t accel_x_offset, int16_t accel_y_offset, int16_t accel_z_offset,
        int16_t gyro_x_offset, int16_t gyro_y_offset, int16_t gyro_z_offset) {
        
        Platform::Status status;
        
        // Set accelerometer offsets
        status = SetAccelOffsets(accel_x_offset, accel_y_offset, accel_z_offset);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Set gyroscope offsets
        status = SetGyroOffsets(gyro_x_offset, gyro_y_offset, gyro_z_offset);
        if (status != Platform::Status::OK) {
            return status;
        }
        
        return Platform::Status::OK;
    }

    // Factory reset for all offset registers
    Platform::Status MPU6050::ResetOffsets() {
        return SetAllOffsets(0, 0, 0, 0, 0, 0);
    }
    /**
     * Performs iterative calibration of the MPU6050
     * This calibration uses the hardware offset registers for best results
     * 
     * @return Status code indicating success or failure
     */
    Platform::Status MPU6050::CalibrateIterative() {
        // Check if initialized
        if (state != DeviceState::Initialized) {
            return Platform::Status::NOT_INITIALIZED;
        }
        
        // Parameters for calibration
        const int acc_deadzone = 8;     // Acceptable accelerometer error
        const int gyro_deadzone = 1;    // Acceptable gyroscope error
        const int max_iterations = 30;  // Maximum iterations to prevent infinite loops
        
        // Initial offsets (start with zeros or current values)
        int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
        int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;
        
        // Reset current offsets first
        Platform::Status status = ResetOffsets();
        if (status != Platform::Status::OK) {
            return status;
        }
        
        // Allow sensor to stabilize after offset reset
        timing_service->DelayMilliseconds(100);
        
        // Buffers for sensor readings
        int16_t ax, ay, az, gx, gy, gz;
        long mean_ax = 0, mean_ay = 0, mean_az = 0;
        long mean_gx = 0, mean_gy = 0, mean_gz = 0;
        
        // Iterative calibration loop
        for (int iteration = 0; iteration < max_iterations; iteration++) {
            // Apply current offsets
            status = SetAllOffsets(
                ax_offset, ay_offset, az_offset,
                gx_offset, gy_offset, gz_offset
            );
            
            if (status != Platform::Status::OK) {
                return status;
            }
            
            // Allow time for offsets to take effect
            timing_service->DelayMilliseconds(100);
            
            // Take multiple readings and average
            const int num_samples = 100;
            mean_ax = 0; mean_ay = 0; mean_az = 0;
            mean_gx = 0; mean_gy = 0; mean_gz = 0;
            
            for (int i = 0; i < num_samples; i++) {
                // Read raw values directly
                std::array<uint8_t, 14> buffer;
                status = ReadRegisters(MPU6050Reg::ACCEL_XOUT_H, buffer.data(), buffer.size(), config.i2c_timeout_ms);
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                // Parse values
                ax = (buffer[0] << 8) | buffer[1];
                ay = (buffer[2] << 8) | buffer[3];
                az = (buffer[4] << 8) | buffer[5];
                gx = (buffer[8] << 8) | buffer[9];
                gy = (buffer[10] << 8) | buffer[11];
                gz = (buffer[12] << 8) | buffer[13];
                
                mean_ax += ax;
                mean_ay += ay;
                mean_az += az;
                mean_gx += gx;
                mean_gy += gy;
                mean_gz += gz;
                
                // Small delay between readings
                timing_service->DelayMilliseconds(2);
            }
            
            // Calculate means
            mean_ax /= num_samples;
            mean_ay /= num_samples;
            mean_az /= num_samples;
            mean_gx /= num_samples;
            mean_gy /= num_samples;
            mean_gz /= num_samples;
            
            // Check if we're within acceptable limits
            bool ready = true;
            
            // Accel X
            if (abs(mean_ax) > acc_deadzone) {
                ax_offset = ax_offset - mean_ax / 8;  // Adjust offset by 1/8th of the error
                ready = false;
            }
            
            // Accel Y
            if (abs(mean_ay) > acc_deadzone) {
                ay_offset = ay_offset - mean_ay / 8;  // Adjust offset by 1/8th of the error
                ready = false;
            }
            
            // Accel Z (should read 1g = 16384 at rest)
            int16_t target_z = 16384;  // 1g at default sensitivity
            if (abs(target_z - mean_az) > acc_deadzone) {
                az_offset = az_offset + (target_z - mean_az) / 8;  // Adjust offset by 1/8th of the error
                ready = false;
            }
            
            // Gyro X
            if (abs(mean_gx) > gyro_deadzone) {
                gx_offset = gx_offset - mean_gx / 4;  // Adjust offset by 1/4th of the error
                ready = false;
            }
            
            // Gyro Y
            if (abs(mean_gy) > gyro_deadzone) {
                gy_offset = gy_offset - mean_gy / 4;  // Adjust offset by 1/4th of the error
                ready = false;
            }
            
            // Gyro Z
            if (abs(mean_gz) > gyro_deadzone) {
                gz_offset = gz_offset - mean_gz / 4;  // Adjust offset by 1/4th of the error
                ready = false;
            }
            
            // If all measurements are within acceptable limits, we're done
            if (ready) {
                // Set final offsets one more time
                status = SetAllOffsets(
                    ax_offset, ay_offset, az_offset,
                    gx_offset, gy_offset, gz_offset
                );
                
                if (status != Platform::Status::OK) {
                    return status;
                }
                
                // Update internal calibration structure
                calibration.accel_x_offset = ax_offset;
                calibration.accel_y_offset = ay_offset;
                calibration.accel_z_offset = az_offset;
                calibration.gyro_x_offset = gx_offset;
                calibration.gyro_y_offset = gy_offset;
                calibration.gyro_z_offset = gz_offset;
                
                return Platform::Status::OK;
            }
        }
        
        // If we get here, we've exceeded max_iterations without achieving calibration
        return Platform::Status::TIMEOUT;
    }
}
}
