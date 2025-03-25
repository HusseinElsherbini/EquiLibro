// src/application/robot_logic/sbr_app.cpp

#include "application/robot_logic/sbr_app.hpp"
#include "math.h"
#include "storage_manager.hpp"
namespace APP {

// Initialize static instance pointer
BalanceRobotApp* BalanceRobotApp::instance = nullptr;

// Constructor
BalanceRobotApp::BalanceRobotApp() 
    : current_state(AppState::Initializing),
      imu(nullptr),
      motor_left(nullptr),
      motor_right(nullptr),
      timing_service(nullptr),
      current_emergency(EmergencyType::None),
      fall_detection_threshold(150.0f) {
    
    // Store system start time
    timing_service = &Middleware::SystemServices::SystemTiming::GetInstance();
    robot_timing.system_start_time = timing_service->GetMilliseconds();
}

// Destructor
BalanceRobotApp::~BalanceRobotApp() {
    // Ensure clean shutdown
    if (current_state != AppState::Shutdown) {
        Stop();
    }
    
    // Clean up any dynamically allocated resources
    if (motor_left) delete motor_left;
    if (motor_right) delete motor_right;
    
    // Reset singleton instance
    instance = nullptr;
}

// Singleton accessor
BalanceRobotApp& BalanceRobotApp::GetInstance() {
    if (!instance) {
        instance = new BalanceRobotApp();
    }
    return *instance;
}

// Get current application state
AppState BalanceRobotApp::GetState() const {
    return current_state;
}

// Initialize the application
Platform::Status BalanceRobotApp::Init(void* config_ptr) {
    // Set default configuration if none provided
    if (config_ptr != nullptr) {
        RobotConfig = *static_cast<BalanceRobotConfig*>(config_ptr);
    } else {
        // Configure IMU
        RobotConfig.imu_config.i2c_config.i2c_instance = Platform::I2C::I2CInstance::I2C1;
        RobotConfig.imu_config.device_address = 0x68; // Default MPU6050 address
        RobotConfig.imu_config.gyro_range = 0;        // ±250°/s
        RobotConfig.imu_config.accel_range = 0;       // ±2g
        RobotConfig.imu_config.dlpf_mode = 3;         // 44Hz low pass filter
        RobotConfig.imu_config.sample_rate_div = 9;   // 100Hz sampling (1kHz / (1 + 9))
        RobotConfig.imu_config.interrupt_enabled = true;
        RobotConfig.imu_config.enable_data_ready_interrupt = true;
        RobotConfig.imu_config.data_ready_port = Platform::GPIO::Port::PORTB;
        RobotConfig.imu_config.data_ready_pin = 8;
        
        // Configure left motor
        RobotConfig.motor_left_config.pwm_config.timer_instance = Platform::TIM::TimerInstance::TIM1;
        RobotConfig.motor_left_config.pwm_config.frequency = 20000;  // 20kHz
        RobotConfig.motor_left_config.pwm_ch_config.channel = Platform::PWM::PWMChannel::Channel3;
        RobotConfig.motor_left_config.pwm_ch_config.gpio_port = Platform::GPIO::Port::PORTA;
        RobotConfig.motor_left_config.pwm_ch_config.gpio_pin = 10;
        RobotConfig.motor_left_config.pwm_ch_config.gpio_af = Platform::GPIO::AlternateFunction::AF1;
        RobotConfig.motor_left_config.ina_port = Platform::GPIO::Port::PORTC;
        RobotConfig.motor_left_config.ina_pin = 6;
        RobotConfig.motor_left_config.inb_port = Platform::GPIO::Port::PORTC;
        RobotConfig.motor_left_config.inb_pin = 7;
        RobotConfig.motor_left_config.use_enable_pin = false;
        RobotConfig.motor_left_config.use_current_sensing = false;
        RobotConfig.motor_left_config.use_diag_pin = false;
        RobotConfig.motor_left_config.min_duty_cycle = 0;
        RobotConfig.motor_left_config.max_duty_cycle = 9000;
        RobotConfig.motor_left_config.acceleration_rate = 2000;
        
        // Configure right motor (similar to left but with different pins)
        RobotConfig.motor_right_config = RobotConfig.motor_left_config;
        RobotConfig.motor_right_config.pwm_ch_config.channel = Platform::PWM::PWMChannel::Channel2;
        RobotConfig.motor_right_config.pwm_ch_config.gpio_pin = 9;
        RobotConfig.motor_right_config.ina_port = Platform::GPIO::Port::PORTA;
        RobotConfig.motor_right_config.ina_pin = 8;
        RobotConfig.motor_right_config.inb_port = Platform::GPIO::Port::PORTB;
        RobotConfig.motor_right_config.inb_pin = 2;
    }
    
    // Initialize hardware interfaces
    Platform::Status status = InitializeHardware();
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Initialize components
    status = InitializeComponents();
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Update status
    this->status.balance_controller_ready = true;
    //TODO:this->status.communication_ready = true;
    //TODO:this->status.monitoring_ready = true;
    
    // Set state to initialized but idle
    current_state = AppState::Idle;
    return Platform::Status::OK;
}

// Initialize hardware interfaces
Platform::Status BalanceRobotApp::InitializeHardware() {
    // Initialize I2C for IMU
    Platform::I2C::I2CInterface* i2c_interface = &Platform::I2C::I2CInterface::GetInstance(RobotConfig.imu_config.i2c_config.i2c_instance);
    Platform::Status status = i2c_interface->Init(&RobotConfig.imu_config);
    if (status != Platform::Status::OK) {
        current_state = AppState::Error;
        return status;
    }
    
    // Initialize IMU
    imu = &Drivers::Sensors::MPU6050::CreateMPU6050();
    status = imu->Init(&RobotConfig.imu_config);
    if (status != Platform::Status::OK) {
        current_state = AppState::Error;
        return status;
    }
    
    // Register IMU data ready callback
    imu->RegisterCallback(
        Drivers::Sensors::MPU6050_EVENT_DATA_AVAILABLE,
        IMUDataReadyCallback,
        this
    );
    
    imu->RegisterCallback(
        Drivers::Sensors::MPU6050_EVENT_DATA_AVAILABLE,
        IMUDataAvailableCallback,
        this
    );
    // Initialize PWM interface for motors
    Platform::PWM::PWMInterface* pwm_interface = &Platform::PWM::PWMInterface::GetInstance();
    status = pwm_interface->Init(&RobotConfig.motor_left_config.pwm_config);
    if (status != Platform::Status::OK) {
        current_state = AppState::Error;
        return status;
    }
    
    // Initialize motor drivers
    motor_left = new Drivers::Motor::VNH5019Driver();
    status = motor_left->Init(&RobotConfig.motor_left_config);
    if (status != Platform::Status::OK) {
        current_state = AppState::Error;
        return status;
    }
    
    motor_right = new Drivers::Motor::VNH5019Driver();
    status = motor_right->Init(&RobotConfig.motor_right_config);
    if (status != Platform::Status::OK) {
        current_state = AppState::Error;
        return status;
    }
    
    return Platform::Status::OK;
}

// Initialize components
Platform::Status BalanceRobotApp::InitializeComponents() {
    // Create and initialize balance controller
    balance_controller = std::make_unique<BalanceController>();
    Platform::Status status = balance_controller->Init(static_cast<Drivers::Sensors::MPU6050*>(imu), \
                                                       static_cast<Drivers::Motor::VNH5019Driver*>(motor_left), \
                                                       static_cast<Drivers::Motor::VNH5019Driver*>(motor_right), \
                                                       static_cast<APP::PIDConfig*>(&RobotConfig.pid_config));
    if (status != Platform::Status::OK) {
        current_state = AppState::Error;
        return status;
    }
    /*//TODO::IMPLEMENT SBUS AND SYSTEM MONITORING
    // Create and initialize SBUS communicator
    sbus_comm = std::make_unique<SBUSCommunicator>();
    status = sbus_comm->Init(nullptr);  // Pass actual UART interface if needed
    if (status != Platform::Status::OK) {
        current_state = AppState::Error;
        return status;
    }
    
    // Create and initialize system monitor
    system_monitor = std::make_unique<SystemMonitor>();
    status = system_monitor->Init(nullptr);
    if (status != Platform::Status::OK) {
        current_state = AppState::Error;
        return status;
    }
    
    // Create and initialize error logger
    error_logger = std::make_unique<ErrorLogger>();
    status = error_logger->Init(nullptr);
    if (status != Platform::Status::OK) {
        current_state = AppState::Error;
        return status;
    }*/
    
    return Platform::Status::OK;
}

// Start the application
Platform::Status BalanceRobotApp::Start() {
    if (current_state != AppState::Idle && current_state != AppState::Calibration) {
        return Platform::Status::INVALID_STATE;
    }
    
    // Start components
    Platform::Status status = balance_controller->Start();
    if (status != Platform::Status::OK) {
        return status;
    }
    /*//TODO::IMPLEMENT SBUS AND SYSTEM MONITORING
    status = sbus_comm->Start();
    if (status != Platform::Status::OK) {
        balance_controller->Stop();
        return status;
    }
    
    status = system_monitor->Start();
    if (status != Platform::Status::OK) {
        balance_controller->Stop();
        sbus_comm->Stop();
        return status;
    }
    */
    // Set state to running
    current_state = AppState::Running;
    return Platform::Status::OK;
}

// Stop the application
Platform::Status BalanceRobotApp::Stop() {
    if (current_state != AppState::Running) {
        return Platform::Status::INVALID_STATE;
    }
    
    // Stop components
    balance_controller->Stop();
    /*//TODO::IMPLEMENT SBUS AND SYSTEM MONITORING
    sbus_comm->Stop();
    system_monitor->Stop();
    */
    // Set state to idle
    current_state = AppState::Idle;
    return Platform::Status::OK;
}

Platform::Status BalanceRobotApp::Process(void* params) {
    // Determine which processes to run
    ProcessType process_type = ProcessType::All;
    if (params != nullptr) {
        process_type = *static_cast<ProcessType*>(params);
    }
    
    // Start timing based on process type
    switch (process_type) {
        case ProcessType::Balancing:
            StartTiming(robot_timing.balance_control);
            break;
        case ProcessType::Communication:
            StartTiming(robot_timing.communication);
            break;
        case ProcessType::Monitoring:
            StartTiming(robot_timing.monitoring);
            break;
        case ProcessType::ErrorLogging:
            StartTiming(robot_timing.error_logging);
            break;
        case ProcessType::All:
            StartTiming(robot_timing.main_loop);
            break;
    }
    
    // Check for emergency conditions first
    if (current_emergency != EmergencyType::None) {
        HandleEmergencyCondition(current_emergency);
        
        // End timing based on process type
        EndProcessTiming(process_type);
        return Platform::Status::OK;
    }
    
    // Process based on current state and requested process type
    switch (process_type) {
        case ProcessType::Balancing:
            if (current_state == AppState::Running) {
                ProcessBalancing();
            }
            break;
            
        case ProcessType::Communication:
            ProcessCommunication();
            break;
            
        case ProcessType::Monitoring:
            ProcessMonitoring();
            break;
            
        case ProcessType::ErrorLogging:
            ProcessErrorLogging();
            break;
            
        case ProcessType::All:
            // Process everything based on current state
            switch (current_state) {
                case AppState::Running:
                    ProcessBalancing();
                    ProcessCommunication();
                    ProcessMonitoring();
                    ProcessErrorLogging();
                    break;
                    
                case AppState::Idle:
                    // No balancing in idle state
                    ProcessCommunication();
                    ProcessMonitoring();
                    ProcessErrorLogging();
                    break;
                    
                case AppState::Calibration:
                    // Only monitoring during calibration
                    ProcessMonitoring();
                    ProcessErrorLogging();
                    break;
                    
                case AppState::Error:
                    // Only error logging in error state
                    ProcessErrorLogging();
                    break;
                    
                default:
                    break;
            }
            break;
    }   
    // Update system metrics
    status.uptime_ms = timing_service->GetMilliseconds() - robot_timing.system_start_time;
    
    // Update frequency metrics based on process type
    switch (process_type) {
        case ProcessType::Balancing:
            status.balance_loop_freq_hz = robot_timing.balance_control.execution_frequency_hz;
            break;
        case ProcessType::Communication:
            status.comm_loop_freq_hz = robot_timing.communication.execution_frequency_hz;
            break;
        case ProcessType::All:
            status.main_loop_freq_hz = robot_timing.main_loop.execution_frequency_hz;
            break;
    }
    
    // End timing based on process type
    EndProcessTiming(process_type);
    return Platform::Status::OK;
}

// Helper function to end timing based on process type
void BalanceRobotApp::EndProcessTiming(ProcessType process_type) {
    switch (process_type) {
        case ProcessType::Balancing:
            EndTiming(robot_timing.balance_control);
            break;
        case ProcessType::Communication:
            EndTiming(robot_timing.communication);
            break;
        case ProcessType::Monitoring:
            EndTiming(robot_timing.monitoring);
            break;
        case ProcessType::ErrorLogging:
            EndTiming(robot_timing.error_logging);
            break;
        case ProcessType::All:
            EndTiming(robot_timing.main_loop);
            break;
    }
}

// Process balancing functionality
Platform::Status BalanceRobotApp::ProcessBalancing(void* params) {
    // Only process balancing in appropriate states
    if (current_state != AppState::Running) {
        return Platform::Status::INVALID_STATE;
    }
    
    // Get current timestamp for consistent timing
    uint64_t current_timestamp_us = timing_service->GetMicroseconds();
    
    // Extract IMU data from parameters
    Drivers::Sensors::MPU6050Data imu_data;
    if (params != nullptr) {
        imu_data = *static_cast<Drivers::Sensors::MPU6050Data*>(params);
    } else {
        // If no data provided, could use most recent data from status
        imu_data = status.imu_data;
    }
    
    // Process balance control - pass down the current timestamp
    Platform::Status status = balance_controller->Process(imu_data, current_timestamp_us);
    
    // Record the timestamp for tracking execution frequency
    robot_timing.balance_control.last_execution_time_us = 
        timing_service->GetMicroseconds() - current_timestamp_us;
    
    return status;
}

// Process communication functionality
Platform::Status BalanceRobotApp::ProcessCommunication(void* params) {

    /*//TODO::IMPLEMENT SBUS AND SYSTEM MONITORING
    // Can process communication in most states
    if (current_state == AppState::Error) {
        return Platform::Status::INVALID_STATE;
    }
    
    // Time the communication processing
    StartTiming(robot_timing.communication);
    
    // Process communication
    Platform::Status status = sbus_comm->Process();
    
    // Update control inputs from remote
    if (status == Platform::Status::OK) {
        status.steering_input = sbus_comm->GetSteeringCommand();
        status.throttle_input = sbus_comm->GetThrottleCommand();
        
        // Check for emergency stop command from remote
        if (sbus_comm->IsEmergencyStopActive()) {
            EmergencyStop(true);
        }
    }
    
    // End timing
    EndTiming(robot_timing.communication);
    
    return status;
    */
}

// Process monitoring functionality
Platform::Status BalanceRobotApp::ProcessMonitoring(void* params) {
    // Can process monitoring in any state
    /*//TODO::IMPLEMENT SBUS AND SYSTEM MONITORING
    // Time the monitoring processing
    StartTiming(robot_timing.monitoring);
    
    // Process system monitoring
    Platform::Status status = system_monitor->Process();
    
    // Update battery status
    if (status == Platform::Status::OK) {
        status.battery_voltage = system_monitor->GetBatteryVoltage();
        status.battery_percentage = system_monitor->GetBatteryPercentage();
        
        // Check for low battery condition
        if (status.battery_percentage < 10 && !status.emergency_stop_active) {
            current_emergency = EmergencyType::BatteryLow;
        }
    }
    
    // Update CPU usage
    status.cpu_usage_percent = system_monitor->GetCPUUsage();
    
    // End timing
    EndTiming(robot_timing.monitoring);
    
    return status;
    */
}

// Process error logging functionality
Platform::Status BalanceRobotApp::ProcessErrorLogging(void* params) {
    // Can process error logging in any state
    /*//TODO::IMPLEMENT SBUS AND SYSTEM MONITORING
    // Time the error logging processing
    StartTiming(robot_timing.error_logging);
    
    // Process error logging
    Platform::Status status = error_logger->Process();
    
    // Log current state if in error state
    if (current_state == AppState::Error) {
        error_logger->LogError("System in error state", 0);
    }
    
    // End timing
    EndTiming(robot_timing.error_logging);
    
    return status;
    */
}

// Handle application commands
Platform::Status BalanceRobotApp::HandleCommand(uint32_t cmd_id, void* params) {
    switch (cmd_id) {
        case APP_CMD_SET_MODE: {
            if (params == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            AppState new_state = *static_cast<AppState*>(params);
            if (new_state == AppState::Running) {
                return Start();
            } else if (new_state == AppState::Idle) {
                return Stop();
            }
            break;
        }
        case APP_CMD_GET_MODE: {
            if (params == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            *static_cast<AppState*>(params) = current_state;
            return Platform::Status::OK;
        }
        case APP_CMD_START_CALIBRATION:
            if (current_state != AppState::Idle) {
                return Platform::Status::INVALID_STATE;
            }
            
            current_state = AppState::Calibration;
            return CalibrateIMU();
            
        case APP_CMD_START_SELF_TEST:
            if (current_state != AppState::Idle) {
                return Platform::Status::INVALID_STATE;
            }
            
            current_state = AppState::SelfTest;
            // TODO: Implement self-test
            current_state = AppState::Idle;
            return Platform::Status::OK;
            
        // Custom commands for balance robot
        case 0x4001:  // Example: Set target angle
            if (params == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            return SetTargetAngle(*static_cast<float*>(params));
            
        case 0x4002:  // Example: Emergency stop
            return EmergencyStop(true);
            
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
    
    return Platform::Status::OK;
}

// Get application status
Platform::Status BalanceRobotApp::GetStatus(void* status_ptr, uint32_t* size) {
    if (status_ptr == nullptr || size == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    if (*size < sizeof(RobotAppStatus)) {
        *size = sizeof(RobotAppStatus);
        return Platform::Status::BUFFER_OVERFLOW;
    }
    
    // Copy status to output buffer
    *static_cast<RobotAppStatus*>(status_ptr) = status;
    *size = sizeof(RobotAppStatus);
    
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

// IMU data ready callback
// Data ready callback - Just initiate read if needed
void BalanceRobotApp::IMUDataReadyCallback(void* param) {
    BalanceRobotApp* app = static_cast<BalanceRobotApp*>(param);
    if (!app) return;
    
    // Start timing
    app->StartTiming(app->robot_timing.imu_callback);
    
    // If in manual mode, trigger read
    if (app->RobotConfig.imu_config.operating_mode == Drivers::Sensors::MPU6050_MODE_MANUAL) {
        app->imu->ReadDataAsync();
    }
    
    // End timing
    app->EndTiming(app->robot_timing.imu_callback);
}

// Data available callback - Just wake up task
void BalanceRobotApp::IMUDataAvailableCallback(void* param) {
    BalanceRobotApp* app = static_cast<BalanceRobotApp*>(param);
    if (!app) return;
    
    // Start timing
    app->StartTiming(app->robot_timing.imu_callback);
    
    app->imu->GetProcessedData(&app->status.imu_data);

    if (app->xBalancingTaskHandle) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(app->xBalancingTaskHandle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
    
    // End timing
    app->EndTiming(app->robot_timing.imu_callback);
}
// Start timing measurement
void BalanceRobotApp::StartTiming(ComponentTiming& timing) {
    timing.last_execution_timestamp = timing_service->GetMicroseconds();
}

// End timing measurement and update statistics
void BalanceRobotApp::EndTiming(ComponentTiming& timing) {
    uint64_t end_time = timing_service->GetMicroseconds();
    uint32_t execution_time = end_time - timing.last_execution_timestamp;
    
    // Update timing statistics
    timing.last_execution_time_us = execution_time;
    
    if (execution_time < timing.min_execution_time_us || timing.execution_count == 0) {
        timing.min_execution_time_us = execution_time;
    }
    
    if (execution_time > timing.max_execution_time_us) {
        timing.max_execution_time_us = execution_time;
    }
    
    // Update moving average (e.g., 90% old value, 10% new value)
    timing.avg_execution_time_us = timing.execution_count == 0 ? 
        execution_time : 
        (timing.avg_execution_time_us * 9 + execution_time) / 10;
    
    // Update execution count
    timing.execution_count++;
    
    // Calculate frequency if we have multiple executions
    static uint64_t prev_timestamp = 0;
    if (prev_timestamp != 0) {
        uint32_t period_us = end_time - prev_timestamp;
        if (period_us > 0) {  // Prevent division by zero
            timing.execution_frequency_hz = 1000000 / period_us;
        }
    }
    prev_timestamp = end_time;
}

// Emergency handling
Platform::Status BalanceRobotApp::HandleEmergencyCondition(EmergencyType emergency) {
    // Handle based on emergency type
    switch (emergency) {
        case EmergencyType::PossibleFall:
            // Stop motors to prevent damage
            motor_left->Stop(true);  // Brake
            motor_right->Stop(true); // Brake
            
            // Log the emergency
             //TODO::error_logger->LogError("Fall detected", 0x1001);
            break;
            
            case EmergencyType::BatteryLow:
            // Reduce power usage
             //TODO::balance_controller->SetPowerSaving(true);
            
            // Notify user
             //TODO::error_logger->LogWarning("Low battery detected", 0x2001);
            
            // Continue operation in power-saving mode
            break;
            
        case EmergencyType::MotorOverheat:
            // Reduce motor power
             //TODO::balance_controller->SetPowerLimit(50);  // 50% power limit
            
            // Log the warning
             //TODO::error_logger->LogWarning("Motor temperature high", 0x2002);
            break;
            
        case EmergencyType::ControllerTimeout:
            // Safe shutdown of balance control
            balance_controller->Stop();
            
            // Log the error
             //TODO::error_logger->LogError("Controller communication timeout", 0x1002);
            
            // Switch to idle state
            current_state = AppState::Idle;
            break;
            
        case EmergencyType::UserTriggered:
            // Full emergency stop
            motor_left->Stop(true);  // Brake
            motor_right->Stop(true); // Brake
            
            // Set emergency flag
            status.emergency_stop_active = true;
            
            // Log the event
             //TODO::error_logger->LogInfo("User emergency stop activated", 0x3001);
            
            // Switch to idle state
            current_state = AppState::Idle;
            break;
            
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    return Platform::Status::OK;
}

// Recover from emergency
Platform::Status BalanceRobotApp::RecoverFromEmergency() {
    // Clear emergency state
    EmergencyType previous_emergency = current_emergency;
    current_emergency = EmergencyType::None;
    
    // Take recovery actions based on previous emergency
    switch (previous_emergency) {
        case EmergencyType::PossibleFall:
            // Reset balance controller
             //TODO::balance_controller->Reset();
            
            // Return to idle state, requiring manual restart
            current_state = AppState::Idle;
            break;
            
        case EmergencyType::BatteryLow:
            // Keep power saving enabled until battery is charged
            // Only clear emergency flag
            break;
            
        case EmergencyType::MotorOverheat:
            // Restore normal power limits after cooldown period
             //TODO::balance_controller->SetPowerLimit(100);  // 100% power
            break;
            
        case EmergencyType::ControllerTimeout:
            // Require manual restart after timeout
            current_state = AppState::Idle;
            break;
            
        case EmergencyType::UserTriggered:
            // Clear emergency stop flag
            status.emergency_stop_active = false;
            
            // Require manual restart
            current_state = AppState::Idle;
            break;
            
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Log recovery
    //TODO::error_logger->LogInfo("Recovered from emergency condition", 0x3002);
    
    return Platform::Status::OK;
}

// Emergency stop
Platform::Status BalanceRobotApp::EmergencyStop(bool user_triggered) {
    // Set emergency type
    current_emergency = user_triggered ? 
        EmergencyType::UserTriggered : 
        EmergencyType::ControllerTimeout;
    
    // Handle the emergency
    return HandleEmergencyCondition(current_emergency);
}

// Set target angle
Platform::Status BalanceRobotApp::SetTargetAngle(float angle) {
    // Forward to balance controller
    return balance_controller->SetTargetAngle(angle);
}

// Set PID parameters
Platform::Status BalanceRobotApp::SetPIDParameters(const PIDConfig& config) {
    // Forward to balance controller
    return balance_controller->UpdatePIDParameters(config);
}

// Calibrate IMU
Platform::Status BalanceRobotApp::CalibrateIMU(void) {

    // Static variable to track if calibration is in progress
    static bool calibration_in_progress = false;
    
    // Get direct access to the MPU6050 instance
    Drivers::Sensors::MPU6050* mpu = static_cast<Drivers::Sensors::MPU6050*>(imu);
    
    Middleware::SystemServices::SystemTiming* timing_service = &Middleware::SystemServices::GetSystemTiming();

    if (!calibration_in_progress) {
        // Initialize calibration
        status.calibration.Reset();
        status.calibration.calib_start_time_ms = timing_service->GetMilliseconds();
        
        // Start the calibration process
        calibration_in_progress = true;
    }
    
    // Check calibration status
    Platform::Status calib_status = mpu->Control(
        Drivers::Sensors::MPU6050_CTRL_CALIBRATE_ITERATIVE, 
        nullptr
    );
    
    // Handle calibration result
    if (calib_status == Platform::Status::OK) {
        // Calibration completed successfully
        
        // Read the final offsets
        int16_t gyro_offsets[3];
        int16_t accel_offsets[3];

        calib_status = mpu->Control(Drivers::Sensors::MPU6050_CTRL_GET_GYRO_OFFSETS, gyro_offsets);
        if(calib_status != Platform::Status::OK){
            return Platform::Status::ERROR;
        }
        calib_status = mpu->Control(Drivers::Sensors::MPU6050_CTRL_GET_ACCEL_OFFSETS, accel_offsets);
        if(calib_status != Platform::Status::OK){
            return Platform::Status::ERROR;
        }
        // Store the offsets in the calibration structure
        status.calibration.hw_gyro_offset[0] = gyro_offsets[0];
        status.calibration.hw_gyro_offset[1] = gyro_offsets[1];
        status.calibration.hw_gyro_offset[2] = gyro_offsets[2];
        
        status.calibration.hw_accel_offset[0] = accel_offsets[0];
        status.calibration.hw_accel_offset[1] = accel_offsets[1];
        status.calibration.hw_accel_offset[2] = accel_offsets[2];
        
        // Save calibration duration
        status.calibration.calib_duration_ms = 
            timing_service->GetMilliseconds() - status.calibration.calib_start_time_ms;
        
        // Mark calibration as complete
        status.calibration.calibration_complete = true;
        
        // Save to flash memory
        Middleware::Storage::StorageManager& storage = Middleware::Storage::StorageManager::GetInstance();
        
        storage.SaveCalibrationData(status.calibration);
        
        // Reset for next time
        calibration_in_progress = false;
        
    }
    else if (calib_status == Platform::Status::ERROR) {
        // Calibration failed
        
        // Mark as not in progress
        calibration_in_progress = false;
        
    }
    return calib_status;
}

// Enable/disable motors
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

// Get performance metrics
Platform::Status BalanceRobotApp::GetPerformanceMetrics(RobotTiming& timing) {
    // Copy timing information to output
    timing = robot_timing;
    return Platform::Status::OK;
}

// Motor fault callback
void BalanceRobotApp::MotorFaultCallback(void* param) {
    BalanceRobotApp* app = static_cast<BalanceRobotApp*>(param);
    if (!app) return;
    
    // Handle motor fault as emergency
    app->current_emergency = EmergencyType::MotorOverheat;
    app->HandleEmergencyCondition(app->current_emergency);
    
    // Trigger application callback if registered
    if (app->callbacks.find(EVENT_ERROR_DETECTED) != app->callbacks.end() && 
        app->callbacks[EVENT_ERROR_DETECTED].active) {
        app->callbacks[EVENT_ERROR_DETECTED].callback(app->callbacks[EVENT_ERROR_DETECTED].param);
    }
}

// Remote command callback
void BalanceRobotApp::RemoteCommandCallback(void* param) {
    /*//TODO::IMPLEMENT SBUS AND SYSTEM MONITORING
    BalanceRobotApp* app = static_cast<BalanceRobotApp*>(param);
    if (!app) return;
    
    // Update control inputs from remote
    app->status.steering_input = app->sbus_comm->GetSteeringCommand();
    app->status.throttle_input = app->sbus_comm->GetThrottleCommand();
    
    // Check for emergency stop command from remote
    if (app->sbus_comm->IsEmergencyStopActive()) {
        app->EmergencyStop(true);
    }
    
    // Trigger application callback if registered
    if (app->callbacks.find(EVENT_REMOTE_COMMAND_RECEIVED) != app->callbacks.end() && 
        app->callbacks[EVENT_REMOTE_COMMAND_RECEIVED].active) {
        app->callbacks[EVENT_REMOTE_COMMAND_RECEIVED].callback(
            app->callbacks[EVENT_REMOTE_COMMAND_RECEIVED].param
        );
    }
        */
}

// Battery status callback
void BalanceRobotApp::BatteryStatusCallback(void* param) {
    /*//TODO::IMPLEMENT SBUS AND SYSTEM MONITORING
    BalanceRobotApp* app = static_cast<BalanceRobotApp*>(param);
    if (!app) return;
    
    // Check for low battery condition
    float battery_voltage = app->system_monitor->GetBatteryVoltage();
    app->status.battery_voltage = battery_voltage;
    app->status.battery_percentage = app->system_monitor->GetBatteryPercentage();
    
    if (app->status.battery_percentage < 10 && !app->status.emergency_stop_active &&
        app->current_emergency == EmergencyType::None) {
        app->current_emergency = EmergencyType::BatteryLow;
        app->HandleEmergencyCondition(app->current_emergency);
    }
    
    // Trigger application callback if registered
    if (app->callbacks.find(EVENT_BATTERY_LOW) != app->callbacks.end() && 
        app->callbacks[EVENT_BATTERY_LOW].active) {
        app->callbacks[EVENT_BATTERY_LOW].callback(app->callbacks[EVENT_BATTERY_LOW].param);
    }
        */
}

} // namespace APP