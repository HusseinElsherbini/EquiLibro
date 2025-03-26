// sbr_app_types.hpp
#pragma once

#include "common/platform.hpp"
#include "drivers/sensors/mpu6050.hpp"
#include "drivers/actuators/vnh5019.hpp"
#include <map>

namespace APP {



struct CalibrationData {
    // Hardware offset values
    int16_t hw_gyro_offset[3];     // X, Y, Z gyroscope offsets stored in hardware registers
    int16_t hw_accel_offset[3];    // X, Y, Z accelerometer offsets stored in hardware registers
    
    // Calibration process tracking
    uint16_t calib_samples;        // Number of samples collected during calibration
    uint8_t calibration_progress;  // 0-100% completion percentage
    bool calibration_complete;     // Flag indicating whether calibration is finished
    
    // Quality metrics
    float calibration_quality;     // 0.0-1.0 quality metric
    
    // Timing information
    uint32_t calib_start_time_ms;  // When calibration started
    uint32_t calib_duration_ms;    // How long calibration took
    
    // Function to reset calibration data
    void Reset() {
        // Reset hardware offset values
        for (int i = 0; i < 3; i++) {
            hw_gyro_offset[i] = 0;
            hw_accel_offset[i] = 0;
        }
        
        // Reset calibration tracking
        calib_samples = 0;
        calibration_progress = 0;
        calibration_complete = false;
        
        // Reset quality metrics
        calibration_quality = 0.0f;
        
        // Reset timing information
        calib_start_time_ms = 0;
        calib_duration_ms = 0;
    }
};
// Component state enumerations
enum class BalanceState {
    Idle,
    Initializing,
    Balancing,
    Fallen,
    Error
};

struct StateTransitionInfo {

    BalanceState old_state;
    BalanceState new_state;
};
// Emergency types
enum class EmergencyType {
    None,
    PossibleFall,
    BatteryLow,
    MotorOverheat,
    ControllerTimeout,
    UserTriggered
};

// Process types for selective execution
enum class ProcessType {
    All,
    Balancing,
    Communication,
    Monitoring,
    ErrorLogging
};

// PID controller configuration
struct PIDConfig {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float output_limit;
    float integral_limit;
    
    // Runtime state (could be separated)
    float integral_sum;
    float prev_error;
    float curr_p_term;
    float curr_i_term;
    float curr_d_term;
    float current_output;
};

// Robot configuration structure
struct BalanceRobotConfig {
    Drivers::Sensors::MPU6050Config imu_config;
    Drivers::Motor::VNH5019Config motor_left_config;
    Drivers::Motor::VNH5019Config motor_right_config;
    PIDConfig pid_config;
};

// Component timing tracking
struct ComponentTiming {
    uint64_t last_execution_timestamp;
    uint32_t last_execution_time_us;
    uint32_t min_execution_time_us;
    uint32_t max_execution_time_us;
    uint32_t avg_execution_time_us;
    uint32_t execution_count;
    uint32_t execution_frequency_hz;
};

// Robot timing structure
struct RobotTiming {
    uint64_t system_start_time;
    ComponentTiming main_loop;
    ComponentTiming balance_control;
    ComponentTiming communication;
    ComponentTiming monitoring;
    ComponentTiming error_logging;
    ComponentTiming imu_callback;
};

// Status information for the self-balancing robot
struct BalanceStatus {
    Drivers::Sensors::MPU6050Data imu_data;    // latest imu data
    PIDConfig pid_output;
    float motor_speed;              // Current motor speed
    uint32_t upright_samples;      
    uint32_t init_samples; 
    bool is_balanced;               // Whether the robot is balanced
    bool is_first_reading;
    bool motor_enabled;             // Whether the motors are enabled
    float last_execution_time;
};

// Application status structure
struct RobotAppStatus {
    bool balance_controller_ready;
    bool communication_ready;
    bool monitoring_ready;
    bool motor_enabled;
    bool is_balanced;
    bool emergency_stop_active;
    float battery_voltage;
    uint8_t battery_percentage;
    uint8_t cpu_usage_percent;
    float steering_input;
    float throttle_input;
    uint64_t uptime_ms;
    uint32_t main_loop_freq_hz;
    uint32_t balance_loop_freq_hz;
    uint32_t comm_loop_freq_hz;
    uint32_t monitor_loop_freq_hz;
    float motor_speed;
    PIDConfig pid_output;
    Drivers::Sensors::MPU6050Data imu_data;
};

// Callback information
struct CallbackEntry {
    void (*callback)(void* param);
    void* param;
    bool active;
};


} // namespace APP