#include "platform.hpp"
#include "actuators/vnh5019.hpp"
#include "sensors/mpu6050.hpp"
#include "sbr_app_types.hpp"
#include "sbr_app_events.hpp"

namespace APP{


// A component that handles balance control
class BalanceController {

    bool Initialized;

    private:
        BalanceState balance_state;
        
        // Controller components
        Drivers::Sensors::MPU6050* imu;
        Drivers::Motor::VNH5019Driver* motor_left;
        Drivers::Motor::VNH5019Driver* motor_right;
    
        
        // struct to hold controller config parameters and latest sensor data ..etc 
        BalanceStatus status;        
        
        struct CallbackEntry {
            void (*callback)(void* param);
            void* param;
            bool active;
        };

        std::unordered_map<uint32_t, CallbackEntry> callbacks;

        // Internal methods
        float ReadAngle();
        float RunPIDController(float current_angle, float dt);
        void SetMotorSpeeds(float output);
        bool CheckFallDetection(float angle);
        void TransitionToState(BalanceState new_state);
        void ProcessBalancingState(float current_angle, float dt);
        void ProcessInitializingState(float current_angle, float dt);
        void ProcessFallenState(float current_angle, float dt);
        void ProcessIdleState(float current_angle, float dt);
        void ProcessErrorState(float current_angle, float dt);
        float CalculateTimeDelta(uint64_t current_timestamp_us, uint64_t previous_timestamp_us);

    public:
    // Constructor only initializes internal state, not hardware
        BalanceController();
        ~BalanceController();

        // Explicit init method with dependency injection
        Platform::Status Init(Drivers::Sensors::MPU6050* imu_interface,
                            Drivers::Motor::VNH5019Driver* left_motor,
                            Drivers::Motor::VNH5019Driver* right_motor,
                            const PIDConfig* config = nullptr);         
        // Main processing function - called by RobotApp
        Platform::Status Process(const Drivers::Sensors::MPU6050Data& imu_data, uint64_t current_timestamp_us);;
        
        // Calibrate the IMU sensor
        Platform::Status CalibrateIMU();    

        // State control
        Platform::Status Start();
        Platform::Status Stop();
        
        // Configuration
        Platform::Status UpdatePIDParameters(const PIDConfig& config);
        Platform::Status SetTargetAngle(float angle);
        
        // Status information
        BalanceState GetState() const { return balance_state; }
        float GetCurrentAngle() const { return this->status.imu_data.filtered_angle; }

;
        // Toggle motor enable/disable
        Platform::Status EnableMotors(bool enable);
    };

}