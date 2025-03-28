#include "balance_controller.hpp"
#include "platform.hpp"
#include "system_timing.hpp"
#include <cmath>
#include "storage_manager.hpp"

namespace APP {

    // Constructor only initializes internal state, not hardware
    BalanceController::BalanceController() : 
        imu(nullptr), 
        motor_left(nullptr), 
        motor_right(nullptr),
        Initialized(false),
        balance_state(BalanceState::Initializing) {
    }
    
    BalanceController::~BalanceController() {
        // First, stop any active balancing to prevent the robot from falling
        if (balance_state == BalanceState::Balancing) {
            // Stop motors safely - using false for no braking to allow gentle slowdown
            motor_left->Stop(false);
            motor_right->Stop(false);
        }
        
        // Ensure all motors are disabled
        if (motor_left && motor_right) {
            motor_left->Disable();
            motor_right->Disable();
        }
        
        // Clear any registered callbacks to prevent dangling references
        callbacks.clear();
        
        // Note: We don't delete the motor or IMU pointers since we don't own them
        // (they were passed to us via dependency injection)
        
        // Set state variables to indicate component is no longer operational
        balance_state = BalanceState::Idle;
        Initialized = false;
    }   
    // Explicit init method with dependency injection
    Platform::Status BalanceController::Init(Drivers::Sensors::MPU6050* imu_interface,
                        Drivers::Motor::VNH5019Driver* left_motor,
                        Drivers::Motor::VNH5019Driver* right_motor,
                        const PIDConfig* config) {
        
        // Check for valid interfaces
        if (!imu_interface || !left_motor || !right_motor) {
            return Platform::Status::INVALID_PARAM;
        }
        // Store hardware interfaces
        this->imu = imu_interface;
        this->motor_left = left_motor;
        this->motor_right = right_motor;        
        // Apply custom config if provided
        if (config) {
            this->status.pid_output = *config;
        }
        
        this->Initialized = true;
        
        // Transition to idle state after successful initialization
        TransitionToState(BalanceState::Idle);
        
        return Platform::Status::OK;
    }

    bool BalanceController::CheckFallDetection(float angle) {
        // Consider the robot fallen if angle exceeds threshold (e.g., ±45 degrees)
        const float FALL_THRESHOLD = 45.0f;
        return (angle < -FALL_THRESHOLD || angle > FALL_THRESHOLD);
    }
    // Process balancing based on provided IMU data
    Platform::Status BalanceController::Process(const Drivers::Sensors::MPU6050Data& imu_data, uint64_t current_timestamp_us) {
        // Check if initialized
        if (!Initialized) {
            return Platform::Status::NOT_INITIALIZED;
        }
        
        // Store received IMU data for diagnostics
        this->status.imu_data = imu_data;
        
        // Store the previous timestamp and update with current one
        uint64_t previous_timestamp_us = this->status.last_execution_time;
        this->status.last_execution_time = current_timestamp_us;
        
        // Calculate time delta in seconds
        float dt = CalculateTimeDelta(current_timestamp_us, previous_timestamp_us);

        // Get the current angle from IMU data
        float current_angle = this->status.imu_data.filtered_angle;

        // Execute the current state
        switch (balance_state) {
            case BalanceState::Initializing:
                ProcessInitializingState(current_angle, dt);
                break;
                
            case BalanceState::Balancing:
                ProcessBalancingState(current_angle, dt);
                break;
                
            case BalanceState::Fallen:
                ProcessFallenState(current_angle, dt);
                break;
                
            case BalanceState::Error:
                ProcessErrorState(current_angle, dt);
                break;
                
            case BalanceState::Idle:
                ProcessIdleState(current_angle, dt);
                break;
                
            default:
                // Unknown state - transition to error
                TransitionToState(BalanceState::Error);
                break;
        }
        
        return Platform::Status::OK;
    }

    void BalanceController::TransitionToState(BalanceState new_state) {

        Middleware::SystemServices::SystemTiming *timing_service = &Middleware::SystemServices::SystemTiming::GetInstance();

        // Don't transition to the same state
        if (new_state == balance_state) {
            return;
        }
        
        // Exit actions for current state
        switch (balance_state) {
            case BalanceState::Balancing:
                // Reset PID controller when exiting balancing
                status.pid_output.integral_sum = 0.0f;
                status.pid_output.prev_error = 0.0f;
                break;
                
            // Other exit actions as needed
            default:
                break;
        }
        
        // Update state
        BalanceState old_state = balance_state;
        balance_state = new_state;
        
        // Entry actions for new state
        switch (new_state) {
            case BalanceState::Initializing:
                // Reset initialization counters
                status.init_samples = 0;
                status.is_first_reading = true;
                break;
                
            case BalanceState::Balancing:
                // Enable motors and reset PID
                motor_left->Enable();
                motor_right->Enable();
                status.pid_output.integral_sum = 0.0f;
                status.pid_output.prev_error = 0.0f;
                break;
                
            case BalanceState::Fallen:
                // Trigger fall detected event
                if (callbacks.find(EVENT_FALL_DETECTED) != callbacks.end() && 
                    callbacks[EVENT_FALL_DETECTED].active) {
                    callbacks[EVENT_FALL_DETECTED].callback(callbacks[EVENT_FALL_DETECTED].param);
                }
                status.upright_samples = 0;
                break;
                
            case BalanceState::Error:
                // Log error
                // TODO: Add error logging
                break;
                
            default:
                break;
        }
        
        // Notify state transition if callback registered
        if (callbacks.find(EVENT_STATE_CHANGED) != callbacks.end() && 
            callbacks[EVENT_STATE_CHANGED].active) {
            // Pass old and new states as parameter 
            StateTransitionInfo transition = {
                .old_state = old_state,
                .new_state = new_state
            };
            callbacks[EVENT_STATE_CHANGED].callback(&transition);
        }
    }
    
    void BalanceController::ProcessBalancingState(float current_angle, float dt)  {

              
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
            balance_state = BalanceState::Fallen;
            return;
        }
        
        // If we get here, the robot is still upright
        status.is_balanced = true;
        
        // Run PID controller to get motor output
        float pid_output = RunPIDController(current_angle, dt);
        
        // Apply motor output to keep robot balanced
        SetMotorSpeeds(pid_output);
        
        
        // Trigger balance update event if callback registered
        if (callbacks.find(EVENT_BALANCE_UPDATE) != callbacks.end() && 
            callbacks[EVENT_BALANCE_UPDATE].active) {
            callbacks[EVENT_BALANCE_UPDATE].callback(callbacks[EVENT_BALANCE_UPDATE].param);
        }

    }   
        // State processing functions
    void BalanceController::ProcessInitializingState(float current_angle, float dt) {
        // In initializing state, we're waiting for the first readings to stabilize
                    
        this->status.init_samples++;
        
        // After enough samples, transition to next state
        if (this->status.init_samples > 100) {  // About 0.5s at 200Hz
            // If upright (within ±30°), transition to balancing
            if (fabs(this->status.imu_data.filtered_angle) < 30.0f) {
                TransitionToState(BalanceState::Balancing);
            } else {
                // Not upright, transition to fallen state
                TransitionToState(BalanceState::Fallen);
            }
        }
        
        
        // Keep motors disabled during initialization
        motor_left->Stop(false);  // No active braking
        motor_right->Stop(false);
    }
        
    void BalanceController::ProcessFallenState(float current_angle, float dt) {
        // Fallen state - robot has tipped over
        
        // Ensure motors are stopped
        motor_left->Stop(true);  // Active braking
        motor_right->Stop(true);
 
        // Check if robot has been returned to upright position
        if (fabs(current_angle) < 10.0f) {
            // Count consecutive upright samples
            this->status.upright_samples++;
            
            // If enough consecutive upright samples, transition to idle
            if (this->status.upright_samples > 100) {  // 0.5s of upright position
                this->status.upright_samples = 0;
                TransitionToState(BalanceState::Idle);
            }
        } else {
            // Reset upright samples counter
            this->status.upright_samples = 0;
        }
    }
    
    void BalanceController::ProcessIdleState(float current_angle, float dt) {
        // Idle state - ready but not actively balancing
        
        // Keep motors disabled
        motor_left->Stop(false);
        motor_right->Stop(false);
        
        // Check for manual transition to balancing (would be triggered externally)
        // No auto-transition to balancing from here
    }
    
    void BalanceController::ProcessErrorState(float current_angle, float dt){

        //TODO
    }
    // Run PID controller to calculate motor output    
    float BalanceController::RunPIDController(float current_angle, float dt) {

        // Calculate error (how far from setpoint)
        float error = status.pid_output.setpoint - current_angle;
                
        // Calculate proportional term
        float p_term = status.pid_output.kp * error;
        
        // Calculate integral term
        status.pid_output.integral_sum += error * dt;
        
        // Apply integral limit to prevent windup
        if (status.pid_output.integral_sum > status.pid_output.integral_limit) {
            status.pid_output.integral_sum = status.pid_output.integral_limit;
        } else if (status.pid_output.integral_sum < -status.pid_output.integral_limit) {
            status.pid_output.integral_sum = -status.pid_output.integral_limit;
        }
        
        float i_term = status.pid_output.ki * status.pid_output.integral_sum;
        
        // Calculate derivative term (on error)
        float derivative = (error - status.pid_output.prev_error) / dt;
        float d_term = status.pid_output.kd * derivative;
        
        // Save current error for next iteration
        status.pid_output.prev_error = error;
        
        // Calculate total output
        float output = p_term + i_term + d_term;
        
        // Apply output limit
        if (output > status.pid_output.output_limit) {
            output = status.pid_output.output_limit;
        } else if (output < -status.pid_output.output_limit) {
            output = -status.pid_output.output_limit;
        }
        
        // Update status with PID terms for debugging
        status.pid_output.curr_p_term = p_term;
        status.pid_output.curr_i_term = i_term;
        status.pid_output.curr_d_term = d_term;
        status.pid_output.current_output = output;
        return output;
    }
    // Set motor speeds based on PID output
    void BalanceController::SetMotorSpeeds(float pid_output) {
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
    // Read sensor data and compute the current angle
    float BalanceController::ReadAngle() {
        // Read IMU sensor data
        Drivers::Sensors::MPU6050Data imu_data;
        imu->ReadData(&imu_data, sizeof(imu_data));
        
        // Calculate tilt angle using accelerometer and gyroscope data (complementary filter)
        // First, calculate accelerometer-based angle (from gravity)
        float accel_angle = atan2f(imu_data.accel_x_g, sqrtf(imu_data.accel_y_g * imu_data.accel_y_g + 
                                                            imu_data.accel_z_g * imu_data.accel_z_g)) * 180.0f / Platform::PI;
        
        // Get time delta in seconds for gyro integration
        float dt = (imu_data.timestamp_ms - status.last_execution_time / 1000.0f) / 1000.0f;
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
    // Helper to calculate time delta between executions
    float BalanceController::CalculateTimeDelta(uint64_t current_timestamp_us, uint64_t previous_timestamp_us) {
        // Calculate time delta in seconds
        float dt = 0.0f;
        
        // Check for valid timestamps
        if (previous_timestamp_us > 0 && current_timestamp_us >= previous_timestamp_us) {
            dt = static_cast<float>(current_timestamp_us - previous_timestamp_us) / 1000000.0f;
            
            // Validate delta is reasonable (between 0 and 100ms)
            if (dt > 0.1f) {
                dt = 0.01f; // Cap at 10ms if too large
            }
        } else {
            // First execution or timestamp error
            dt = 0.01f; // Default to 10ms
        }
        
        return dt;
    }
    // Set target angle (for adjusting balance point)
    Platform::Status BalanceController::SetTargetAngle(float angle){
        status.pid_output.setpoint = angle;
    }
    // Update PID parameters
    Platform::Status BalanceController::UpdatePIDParameters(const PIDConfig& pid_params) {
        status.pid_output.kp = pid_params.kp;
        status.pid_output.ki = pid_params.ki;
        status.pid_output.kd = pid_params.kd;
        // Update target angle in status
        status.pid_output.setpoint = pid_params.setpoint;
        
        // Reset integral term when changing PID parameters
        status.pid_output.integral_sum = 0.0f;
        
        return Platform::Status::OK;
    }
    
    Platform::Status BalanceController::Start() {
        // Check if we're in a valid state to start balancing
        if (balance_state != BalanceState::Idle && balance_state != BalanceState::Fallen) {
            return Platform::Status::INVALID_STATE;
        }
        
        // Check if all components are ready
        if (!imu || !motor_left || !motor_right) {
            return Platform::Status::NOT_INITIALIZED;
        }
        
        // Reset PID controller values
        status.pid_output.integral_sum = 0.0f;
        status.pid_output.prev_error = 0.0f;
        
        // Reset initialization counters
        status.init_samples = 0;
        status.is_first_reading = true;
        
        // Transition to initializing state first (safer)
        TransitionToState(BalanceState::Initializing);
        
        return Platform::Status::OK;
    }

    Platform::Status BalanceController::Stop() {
        // Check if we're in a valid state to stop balancing
        if (balance_state != BalanceState::Balancing) {
            return Platform::Status::INVALID_STATE;
        }
        // Safely stop motors
        motor_left->Stop(false);  // false = no braking, coast to stop
        motor_right->Stop(false);
        // Transition to idle state
        TransitionToState(BalanceState::Idle);
        
        return Platform::Status::OK;
    }
}