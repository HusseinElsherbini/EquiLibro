// sbr_app.hpp
#pragma once

#include "common/platform.hpp"
#include "app_module.hpp"
#include "balance_controller.hpp"
#include "drivers/sensors/mpu6050.hpp"
#include "drivers/actuators/vnh5019.hpp"
#include "middleware/system_services/system_timing.hpp"
#include "sbr_app_types.hpp"
#include "sbr_app_events.hpp"
#include "sbr_app_tasks.hpp"

#include <memory>
#include <map>

namespace APP {


/**
 * Self-balancing robot application class
 * Manages the application lifecycle, state, and components
 */
class BalanceRobotApp : public ApplicationModule {
public:
    // Singleton accessor
    static BalanceRobotApp& GetInstance();
    
    //============= ApplicationModule Interface Implementation =============//
    
    // Get current application state
    AppState GetState() const override;
    
    // Initialize the application module
    Platform::Status Init(void* config) override;
    
    // Start the module operation
    Platform::Status Start() override;
    
    // Stop the module operation
    Platform::Status Stop() override;
    
    // Main processing function
    Platform::Status Process(void* params) override;
    
    // Handle a command
    Platform::Status HandleCommand(uint32_t cmd_id, void* params) override;
    
    // Get module status
    Platform::Status GetStatus(void* status, uint32_t* size) override;
    
    // Register callback for application events
    Platform::Status RegisterCallback(uint32_t event, void (*callback)(void* param), void* param) override;
    
    //================= Task Management Functions =================//
    
    // Initialize FreeRTOS tasks
    Platform::Status InitializeTasks();
    
    // Start tasks
    Platform::Status StartTasks();
    
    // Stop tasks
    Platform::Status StopTasks();
    
    // Suspend balancing task
    Platform::Status SuspendBalancingTask();
    
    // Resume balancing task
    Platform::Status ResumeBalancingTask();

    // Register a task to be notified
    void RegisterBalancingTask(TaskHandle_t handle);
    
    //================= Public Utility Methods =================//
    
    // Set target angle (for adjusting balance point)
    Platform::Status SetTargetAngle(float angle);
    
    // Update PID parameters
    Platform::Status SetPIDParameters(const PIDConfig& config);
    
    // Emergency stop
    Platform::Status EmergencyStop(bool user_triggered);
    
    // Recover from emergency
    Platform::Status RecoverFromEmergency();
    
    // Enable/disable motors
    Platform::Status EnableMotors(bool enable);
    
    // Get performance metrics
    Platform::Status GetPerformanceMetrics(RobotTiming& timing);
    
    // Calibrate IMU
    Platform::Status CalibrateIMU();
    
    // Friend task functions so they can access private members
    friend void vBalancingTask(void* params);
    friend void vMonitoringTask(void* params);
    friend void vCommunicationTask(void* params);
    
private:
    // Private constructor for singleton
    BalanceRobotApp();
    ~BalanceRobotApp();
    
    // Prevent copying and assignment
    BalanceRobotApp(const BalanceRobotApp&) = delete;
    BalanceRobotApp& operator=(const BalanceRobotApp&) = delete;
    
    //================= Hardware Initialization =================//
    
    // Initialize hardware interfaces
    Platform::Status InitializeHardware();
    
    // Initialize components
    Platform::Status InitializeComponents();
    
    //================= Process Functions =================//
    
    // Process balancing functionality
    Platform::Status ProcessBalancing(void* params = nullptr);
    
    // Process communication functionality
    Platform::Status ProcessCommunication(void* params = nullptr);
    
    // Process monitoring functionality
    Platform::Status ProcessMonitoring(void* params = nullptr);
    
    // Process error logging functionality
    Platform::Status ProcessErrorLogging(void* params = nullptr);
    
    //================= Utility Functions =================//
    
    // Handle emergency condition
    Platform::Status HandleEmergencyCondition(EmergencyType emergency);
    
    // Start timing measurement
    void StartTiming(ComponentTiming& timing);
    
    // End timing measurement and update statistics
    void EndTiming(ComponentTiming& timing);
    
    
    void EndProcessTiming(ProcessType process_type);

    //================= Static Callbacks =================//
    
    // IMU data ready callback
    static void IMUDataReadyCallback(void* param);
    
    // IMU data available callback
    static void IMUDataAvailableCallback(void* param);
    
    // Motor fault callback
    static void MotorFaultCallback(void* param);
    
    // Remote command callback
    static void RemoteCommandCallback(void* param);
    
    // Battery status callback
    static void BatteryStatusCallback(void* param);
    
    //================= State Variables =================//
    
    // Static instance pointer for singleton
    static BalanceRobotApp* instance;
    
    // Current application state
    AppState current_state;
    
    // Balance robot configuration
    BalanceRobotConfig RobotConfig;
    
    // Current status
    RobotAppStatus status;
    
    // Timing information
    RobotTiming robot_timing;
    
    // Current emergency type
    EmergencyType current_emergency;
    
    // Fall detection threshold
    float fall_detection_threshold;
    
    //================= Hardware Interfaces =================//
    
    // IMU interface
    Drivers::Sensors::MPU6050* imu;
    
    // Motor interfaces
    Drivers::Motor::VNH5019Driver* motor_left;
    Drivers::Motor::VNH5019Driver* motor_right;
    
    // Timing service
    Middleware::SystemServices::SystemTiming* timing_service;
    
    //================= Components =================//
    
    // Balance controller component
    std::unique_ptr<BalanceController> balance_controller;
    
    // TODO: Future components
    //std::unique_ptr<SBUSCommunicator> sbus_comm;
    //std::unique_ptr<SystemMonitor> system_monitor;
    //std::unique_ptr<ErrorLogger> error_logger;
    
    //================= Task Handles and Configuration =================//
    
    // Task handles
    TaskHandle_t xBalancingTaskHandle = nullptr;
    TaskHandle_t xMonitoringTaskHandle = nullptr;
    TaskHandle_t xCommunicationTaskHandle = nullptr;
    
    // Task configurations
    TaskConfig balancing_task_config;
    TaskConfig monitoring_task_config;
    TaskConfig communication_task_config;
    
    //================= Callback Registry =================//
    
    // Map of registered callbacks
    std::map<uint32_t, CallbackEntry> callbacks;
};

} // namespace APP