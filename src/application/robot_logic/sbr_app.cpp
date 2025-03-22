// src/application/robot_logic/balance_app.cpp

#include "robot_logic/sbr_app.hpp"
#include "middleware/system_services/system_timing.hpp"
#include "system_services/error.hpp"
#include "hardware_abstraction/i2c.hpp"
#include "hardware_abstraction/gpio.hpp"
#include "middleware/system_services/error.hpp"
#include <math.h>

namespace APP {

// Singleton instance
static BalanceRobotApp* s_instance = nullptr;

// Constructor
BalanceRobotApp::BalanceRobotApp() 
    : current_state(AppState::Initializing),
      prev_error(0.0f),
      integral_sum(0.0f),
      last_control_time(0),
      imu(nullptr),
      motor_left(nullptr),
      motor_right(nullptr),
      timing_service(nullptr) {
    
    // Initialize status data
    status.current_angle = 0.0f;
    status.target_angle = 0.0f;
    status.motor_speed = 0.0f;
    status.pid_output = 0.0f;
    status.p_term = 0.0f;
    status.i_term = 0.0f;
    status.d_term = 0.0f;
    status.loop_time_us = 0;
    status.is_balanced = false;
    status.motor_enabled = false;
}

// Destructor
BalanceRobotApp::~BalanceRobotApp() {
    // The peripheral interfaces are singletons, no need to delete them
    s_instance = nullptr;
}

// Singleton accessor
BalanceRobotApp& BalanceRobotApp::GetInstance() {
    static BalanceRobotApp instance;
    return instance;
}

// Get current application state
AppState BalanceRobotApp::GetState() const {
    return current_state;
}

// Initialize the self-balancing robot
Platform::Status BalanceRobotApp::Init(void* config_ptr) {
    if (current_state != AppState::Initializing && current_state != AppState::Error) {
        return Platform::Status::INVALID_STATE;
    }

    // Set default configuration if none provided
    if (config_ptr != nullptr) {
        config = *static_cast<BalanceRobotConfig*>(config_ptr);
    } else {
        // Default configuration
        config.imu_config.device_address = 0x68; // Default MPU6050 address
        config.imu_config.gyro_range = 1;        // ±500°/s
        config.imu_config.accel_range = 1;       // ±4g
        config.imu_config.dlpf_mode = 3;         // 44Hz low pass filter
        config.imu_config.sample_rate_div = 9;   // 100Hz sampling (1kHz / (1 + 9))
        config.imu_config.interrupt_enabled = true;
        
        // Default PID settings - these will need tuning for your specific robot
        config.pid_config.kp = 10.0f;
        config.pid_config.ki = 0.1f;
        config.pid_config.kd = 0.5f;
        config.pid_config.setpoint = 0.0f;      // Upright position
        config.pid_config.output_limit = 100.0f; // Max motor output
        config.pid_config.integral_limit = 50.0f; // Prevent integral windup
        
        config.control_loop_interval_ms = 10;    // 100Hz control loop
        config.enable_debug_output = true;
    }

    // Get hardware interfaces
    timing_service = &Middleware::SystemServices::SystemTiming::GetInstance();
    if (!timing_service->IsInitialized()) {
        return Platform::Status::DEPENDENCY_NOT_INITIALIZED;
    }
    
    // Initialize IMU (MPU6050)
    imu = &Drivers::Sensors::MPU6050::CreateMPU6050();
    
    Platform::Status status = imu->Init(&config.imu_config);
    if (status != Platform::Status::OK) {
        // Log error and return
        Middleware::SystemServices::ERROR::ERROR_LOG(
            Middleware::SystemServices::ERROR::MODULE_APPLICATION_MAIN,
            Middleware::SystemServices::ERROR::ERR_INITIALIZATION,
            status
        );
        current_state = AppState::Error;
        return status;
    }
    
    // Register IMU data ready callback
    imu->RegisterCallback(Drivers::Sensors::MPU6050_EVENT_DATA_READY, IMUDataReadyCallback, this);
    
    // Initialize motor drivers
    motor_left = new Drivers::Motor::VNH5019Driver();
    status = motor_left->Init(&config.motor_left_config);
    if (status != Platform::Status::OK) {
        current_state = AppState::Error;
        return status;
    }
    
    motor_right = new Drivers::Motor::VNH5019Driver();
    status = motor_right->Init(&config.motor_right_config);
    if (status != Platform::Status::OK) {
        current_state = AppState::Error;
        return status;
    }
    
    // Initialize PID controller state
    ResetPIDState();
    
    // Set target angle from configuration
    this->status.target_angle = config.pid_config.setpoint;
    
    // Set state to initialized but idle
    current_state = AppState::Idle;
    return Platform::Status::OK;
}

// Start balancing operation
Platform::Status BalanceRobotApp::Start() {
    if (current_state != AppState::Idle && current_state != AppState::Calibration) {
        return Platform::Status::INVALID_STATE;
    }
    
    // Reset PID state before starting
    ResetPIDState();
    
    // Enable motors
    motor_left->Enable();
    motor_right->Enable();
    status.motor_enabled = true;
    
    // Update last control time
    last_control_time = timing_service->GetMicroseconds();
    
    // Set state to running
    current_state = AppState::Running;
    return Platform::Status::OK;
}

// Stop balancing operation
Platform::Status BalanceRobotApp::Stop() {
    if (current_state != AppState::Running) {
        return Platform::Status::INVALID_STATE;
    }
    
    // Stop motors
    motor_left->Stop(true);  // Brake
    motor_right->Stop(true); // Brake
    status.motor_enabled = false;
    
    // Set state to idle
    current_state = AppState::Idle;
    return Platform::Status::OK;
}

// Process function - called periodically to update state
Platform::Status BalanceRobotApp::Process(void* params) {
    if (current_state != AppState::Running) {
        return Platform::Status::OK; // Nothing to do if not running
    }
    
    // Get current time for control loop timing
    uint64_t current_time = timing_service->GetMicroseconds();
    uint64_t elapsed_time = current_time - last_control_time;
    
    // Check if it's time to run the control loop
    if (elapsed_time < (config.control_loop_interval_ms * 1000)) {
        return Platform::Status::OK; // Not time yet
    }
    
    // Update control loop timing
    last_control_time = current_time;
    
    // Measure execution time
    uint64_t start_time = timing_service->GetMicroseconds();
    
    // Read current angle from IMU
    float current_angle = ReadAngle();
    status.current_angle = current_angle;
    
    // Check if robot has fallen over
    if (CheckFallDetection(current_angle)) {
        // Robot has fallen, stop motors
        motor_left->Stop(true);
        motor_right->Stop(true);
        status.is_balanced = false;
        
        // Trigger fall detected event
        if (callbacks.find(EVENT_FALL_DETECTED) != callbacks.end() && 
            callbacks[EVENT_FALL_DETECTED].active) {
            callbacks[EVENT_FALL_DETECTED].callback(callbacks[EVENT_FALL_DETECTED].param);
        }
        
        // Set state to idle, waiting for restart
        current_state = AppState::Idle;
        return Platform::Status::OK;
    }
    
    // If we get here, the robot is still upright
    status.is_balanced = true;
    
    // Run PID controller to get motor output
    float pid_output = RunPIDController(current_angle);
    status.pid_output = pid_output;
    
    // Apply motor output to keep robot balanced
    SetMotorSpeeds(pid_output);
    
    // Calculate loop execution time
    status.loop_time_us = timing_service->GetMicroseconds() - start_time;
    
    // Trigger balance update event if callback registered
    if (callbacks.find(EVENT_BALANCE_UPDATE) != callbacks.end() && 
        callbacks[EVENT_BALANCE_UPDATE].active) {
        callbacks[EVENT_BALANCE_UPDATE].callback(callbacks[EVENT_BALANCE_UPDATE].param);
    }
    
    // Debug output if enabled
    if (config.enable_debug_output) {
        DebugOutput(status);
    }
    
    return Platform::Status::OK;
}

// Handle application commands
Platform::Status BalanceRobotApp::HandleCommand(uint32_t cmd_id, void* params) {
    switch (cmd_id) {
        case BALANCE_ROBOT_CMD_UPDATE_PID:
            if (params == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            return UpdatePIDParameters(*static_cast<PIDConfig*>(params));
            
        case BALANCE_ROBOT_CMD_CALIBRATE_IMU:
            return CalibrateIMU();
            
        case BALANCE_ROBOT_CMD_SET_TARGET_ANGLE:
            if (params == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            return SetTargetAngle(*static_cast<float*>(params));
            
        case BALANCE_ROBOT_CMD_ENABLE_MOTORS:
            if (params == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            return EnableMotors(*static_cast<bool*>(params));
            
        case BALANCE_ROBOT_CMD_GET_BALANCE_STATUS:
            if (params == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            *static_cast<BalanceRobotStatus*>(params) = status;
            return Platform::Status::OK;
            
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
}

// Get application status
Platform::Status BalanceRobotApp::GetStatus(void* status_ptr, uint32_t* size) {
    if (status_ptr == nullptr || size == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    if (*size < sizeof(BalanceRobotStatus)) {
        *size = sizeof(BalanceRobotStatus);
        return Platform::Status::BUFFER_OVERFLOW;
    }
    
    *static_cast<BalanceRobotStatus*>(status_ptr) = status;
    *size = sizeof(BalanceRobotStatus);
    
    return Platform::Status::OK;
}

// Register callback for application events
Platform::Status BalanceRobotApp::RegisterCallback(uint32_t event, void (*callback)(void* param), void* param) {
    if (callback == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Store callback
    callbacks[event].callback = callback;
    callbacks[event].param = param;
    callbacks[event].active = true;
    
    return Platform::Status::OK;
}

// Update PID parameters
Platform::Status BalanceRobotApp::UpdatePIDParameters(const PIDConfig& pid_params) {
    config.pid_config = pid_params;
    
    // Update target angle in status
    status.target_angle = pid_params.setpoint;
    
    // Reset integral term when changing PID parameters
    integral_sum = 0.0f;
    
    return Platform::Status::OK;
}

// Calibrate the IMU sensor
Platform::Status BalanceRobotApp::CalibrateIMU() {
    if (current_state == AppState::Running) {
        // Need to stop balancing to calibrate
        Stop();
    }
    
    // Set state to calibration
    current_state = AppState::Calibration;
    
    // Perform IMU calibration - this might take some time
    Platform::Status status = imu->Control(Drivers::Sensors::MPU6050_CTRL_CALIBRATE_GYRO, nullptr);
    if (status != Platform::Status::OK) {
        current_state = AppState::Error;
        return status;
    }
    
    // Calibrate accelerometer
    status = imu->Control(Drivers::Sensors::MPU6050_CTRL_CALIBRATE_ACCEL, nullptr);
    if (status != Platform::Status::OK) {
        current_state = AppState::Error;
        return status;
    }
    
    // Reset PID state
    ResetPIDState();
    
    // Return to idle state
    current_state = AppState::Idle;
    return Platform::Status::OK;
}

// Set target angle (for adjusting balance point)
Platform::Status BalanceRobotApp::SetTargetAngle(float angle) {
    // Limit target angle to reasonable values (-15 to +15 degrees)
    if (angle < -15.0f || angle > 15.0f) {
        return Platform::Status::INVALID_PARAM;
    }
    
    config.pid_config.setpoint = angle;
    status.target_angle = angle;
    
    return Platform::Status::OK;
}

// Toggle motor enable/disable
Platform::Status BalanceRobotApp::EnableMotors(bool enable) {
    if (enable) {
        motor_left->Enable();
        motor_right->Enable();
        status.motor_enabled = true;
    } else {
        motor_left->Disable();
        motor_right->Disable();
        status.motor_enabled = false;
    }
    
    return Platform::Status::OK;
}

// Private methods

// Read sensor data and compute the current angle
float BalanceRobotApp::ReadAngle() {
    // Read IMU sensor data
    Drivers::Sensors::MPU6050Data imu_data;
    imu->ReadData(&imu_data, sizeof(imu_data));
    
    // Calculate tilt angle using accelerometer and gyroscope data (complementary filter)
    // First, calculate accelerometer-based angle (from gravity)
    float accel_angle = atan2f(imu_data.accel_x_g, sqrtf(imu_data.accel_y_g * imu_data.accel_y_g + 
                                                         imu_data.accel_z_g * imu_data.accel_z_g)) * 180.0f / M_PI;
    
    // Get time delta in seconds for gyro integration
    float dt = (imu_data.timestamp_ms - last_control_time / 1000.0f) / 1000.0f;
    if (dt <= 0.0f || dt > 0.1f) {
        // Invalid time delta, use a reasonable default
        dt = 0.01f;
    }
    
    // Complementary filter: combine accelerometer angle and gyro rate
    // Typically 98% gyro and 2% accelerometer for smooth, drift-free angle
    static float filtered_angle = 0.0f;
    filtered_angle = 0.98f * (filtered_angle + imu_data.gyro_x_dps * dt) + 0.02f * accel_angle;
    
    return filtered_angle;
}

// Run PID controller to calculate motor output
float BalanceRobotApp::RunPIDController(float current_angle) {
    // Calculate error (how far from setpoint)
    float error = status.target_angle - current_angle;
    
    // Calculate time delta in seconds
    uint64_t current_time = timing_service->GetMicroseconds();
    float dt = (current_time - last_control_time) / 1000000.0f;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f; // Default to 10ms if timing is problematic
    }
    
    // Calculate proportional term
    float p_term = config.pid_config.kp * error;
    
    // Calculate integral term
    integral_sum += error * dt;
    
    // Apply integral limit to prevent windup
    if (integral_sum > config.pid_config.integral_limit) {
        integral_sum = config.pid_config.integral_limit;
    } else if (integral_sum < -config.pid_config.integral_limit) {
        integral_sum = -config.pid_config.integral_limit;
    }
    
    float i_term = config.pid_config.ki * integral_sum;
    
    // Calculate derivative term (on error)
    float derivative = (error - prev_error) / dt;
    float d_term = config.pid_config.kd * derivative;
    
    // Save current error for next iteration
    prev_error = error;
    
    // Calculate total output
    float output = p_term + i_term + d_term;
    
    // Apply output limit
    if (output > config.pid_config.output_limit) {
        output = config.pid_config.output_limit;
    } else if (output < -config.pid_config.output_limit) {
        output = -config.pid_config.output_limit;
    }
    
    // Update status with PID terms for debugging
    status.p_term = p_term;
    status.i_term = i_term;
    status.d_term = d_term;
    
    return output;
}

// Set motor speeds based on PID output
void BalanceRobotApp::SetMotorSpeeds(float pid_output) {
    // Convert PID output to motor speed (normalized to -100 to 100 range)
    float motor_speed = pid_output;
    
    // Update status
    status.motor_speed = motor_speed;
    
    // Set motor speeds (both motors get the same speed for straight balancing)
    // In practice, there might be slight mechanical differences requiring tuning
    if (motor_speed > 0) {
        // Forward direction
        motor_left->SetSpeedAndDirection(static_cast<uint32_t>(fabs(motor_speed)), Drivers::Motor::Direction::Forward);
        motor_right->SetSpeedAndDirection(static_cast<uint32_t>(fabs(motor_speed)), Drivers::Motor::Direction::Forward);
    } else {
        // Backward direction
        motor_left->SetSpeedAndDirection(static_cast<uint32_t>(fabs(motor_speed)), Drivers::Motor::Direction::Reverse);
        motor_right->SetSpeedAndDirection(static_cast<uint32_t>(fabs(motor_speed)), Drivers::Motor::Direction::Reverse);
    }
}

// Check if robot has fallen over
bool BalanceRobotApp::CheckFallDetection(float angle) {
    // Consider the robot fallen if angle exceeds threshold (e.g., ±45 degrees)
    const float FALL_THRESHOLD = 45.0f;
    return (angle < -FALL_THRESHOLD || angle > FALL_THRESHOLD);
}

// Reset integral term and other state when restarting balance
void BalanceRobotApp::ResetPIDState() {
    integral_sum = 0.0f;
    prev_error = 0.0f;
    
    // Reset PID terms in status
    status.p_term = 0.0f;
    status.i_term = 0.0f;
    status.d_term = 0.0f;
    status.pid_output = 0.0f;
    
    // Set target angle from configuration
    status.target_angle = config.pid_config.setpoint;
}

// Debug output helper
void BalanceRobotApp::DebugOutput(const BalanceRobotStatus& status) {
    // In a real implementation, this would log to the debug interface
    // For simplicity, we're not implementing actual debug output here

    // Could toggle an LED at a specific rate to indicate activity
    // Or send data via a serial interface if available
}

// IMU data callback handler
void BalanceRobotApp::IMUDataReadyCallback(void* param) {
    BalanceRobotApp* app = static_cast<BalanceRobotApp*>(param);
    if (app == nullptr) {
        return;
    }
    
    // This is called when new IMU data is available
    // We could trigger an immediate update here, but for consistency
    // we'll stick to the regular control loop timing
}

} // namespace APP