// inc/application/robot_logic/balance_app.hpp

#pragma once

#include "application/app_module.hpp"
#include "hardware_abstraction/gpio.hpp"
#include "hardware_abstraction/i2c.hpp"
#include "hardware_abstraction/pwm.hpp"
#include "middleware/system_services/system_timing.hpp"
#include "drivers/sensors/mpu6050.hpp"
#include "drivers/actuators/vnh5019.hpp"
#include <memory>
#include <array>
#include <unordered_map>

namespace APP {

// PID controller configuration
struct PIDConfig {
    float kp;            // Proportional gain
    float ki;            // Integral gain
    float kd;            // Derivative gain
    float setpoint;      // Desired angle (usually 0 for upright position)
    float output_limit;  // Limit the output range (-output_limit to +output_limit)
    float integral_limit; // Limit the integral term to prevent windup
};

// Configuration structure for self-balancing robot
struct BalanceRobotConfig {
    // MPU6050 configuration
    Drivers::Sensors::MPU6050Config imu_config;
    
    // Motor driver configuration
    Drivers::Motor::VNH5019Config motor_left_config;
    Drivers::Motor::VNH5019Config motor_right_config;
    
    // PID configuration
    PIDConfig pid_config;

    // Timing configuration
    uint32_t control_loop_interval_ms;  // Control loop interval in milliseconds
    
    // Debug settings
    bool enable_debug_output;           // Enable debug messages (via LED or serial)
};

// Status information for the self-balancing robot
struct BalanceRobotStatus {
    float current_angle;            // Current measured angle
    float target_angle;             // Target angle (setpoint)
    float motor_speed;              // Current motor speed
    float pid_output;               // Current PID controller output
    float p_term;                   // Proportional term
    float i_term;                   // Integral term
    float d_term;                   // Derivative term
    uint32_t loop_time_us;          // Execution time of the control loop
    bool is_balanced;               // Whether the robot is balanced
    bool motor_enabled;             // Whether the motors are enabled
};

// Self-balancing robot application class
class BalanceRobotApp : public ApplicationModule {
private:
    // Configuration
    BalanceRobotConfig config;
    
    // State tracking
    AppState current_state;
    BalanceRobotStatus status;
    
    // Previous error for derivative calculation
    float prev_error;
    
    // Running sum for integral calculation
    float integral_sum;
    
    // Last control loop time for dt calculation
    uint64_t last_control_time;
    
    // Hardware interfaces
    Drivers::Sensors::MPU6050* imu;
    Drivers::Motor::VNH5019Driver* motor_left;
    Drivers::Motor::VNH5019Driver* motor_right;
    Middleware::SystemServices::SystemTiming* timing_service;
    
    // Callback storage
    struct CallbackEntry {
        void (*callback)(void* param);
        void* param;
        bool active;
    };
    
    // Define application events
    static constexpr uint32_t EVENT_BALANCE_UPDATE = 0x1;
    static constexpr uint32_t EVENT_FALL_DETECTED = 0x2;
    static constexpr uint32_t EVENT_IMU_ERROR = 0x3;
    static constexpr uint32_t EVENT_MOTOR_ERROR = 0x4;
    
    std::unordered_map<uint32_t, CallbackEntry> callbacks;
    
    // Private methods
    
    // Read sensor data and compute the current angle
    float ReadAngle();
    
    // Run PID controller to calculate motor output
    float RunPIDController(float current_angle);
    
    // Set motor speeds based on PID output
    void SetMotorSpeeds(float pid_output);
    
    // Check if robot has fallen over
    bool CheckFallDetection(float angle);
    
    // Reset integral term and other state when restarting balance
    void ResetPIDState();
    
    // Debug output helper
    void DebugOutput(const BalanceRobotStatus& status);
    
    // IMU data callback handler
    static void IMUDataReadyCallback(void* param);
    
public:
    // Constructor
    BalanceRobotApp();
    
    // Destructor
    ~BalanceRobotApp() override;
    
    // Singleton accessor
    static BalanceRobotApp& GetInstance();
    
    // ApplicationModule interface implementation
    
    // Get current application state
    AppState GetState() const override;
    
    // Initialize the self-balancing robot
    Platform::Status Init(void* config) override;
    
    // Start balancing operation
    Platform::Status Start() override;
    
    // Stop balancing operation
    Platform::Status Stop() override;
    
    // Process function - called periodically to update state
    Platform::Status Process(void* params) override;
    
    // Handle application commands
    Platform::Status HandleCommand(uint32_t cmd_id, void* params) override;
    
    // Get application status
    Platform::Status GetStatus(void* status, uint32_t* size) override;
    
    // Register callback for application events
    Platform::Status RegisterCallback(uint32_t event, void (*callback)(void* param), void* param) override;
    
    // Application-specific methods
    
    // Update PID parameters
    Platform::Status UpdatePIDParameters(const PIDConfig& pid_params);
    
    // Calibrate the IMU sensor
    Platform::Status CalibrateIMU();
    
    // Set target angle (for adjusting balance point)
    Platform::Status SetTargetAngle(float angle);
    
    // Toggle motor enable/disable
    Platform::Status EnableMotors(bool enable);
};

// Command IDs specific to balance robot application
constexpr uint32_t BALANCE_ROBOT_CMD_UPDATE_PID = 0x5001;
constexpr uint32_t BALANCE_ROBOT_CMD_CALIBRATE_IMU = 0x5002;
constexpr uint32_t BALANCE_ROBOT_CMD_SET_TARGET_ANGLE = 0x5003;
constexpr uint32_t BALANCE_ROBOT_CMD_ENABLE_MOTORS = 0x5004;
constexpr uint32_t BALANCE_ROBOT_CMD_GET_BALANCE_STATUS = 0x5005;

} // namespace APP