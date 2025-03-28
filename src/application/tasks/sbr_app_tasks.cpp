// sbr_app_tasks.cpp
#include "sbr_app.hpp"
#include "sbr_app_tasks.hpp"

namespace APP {

Platform::Status BalanceRobotApp::InitializeTasks() {
    // Configure balancing task
    balancing_task_config = {
        .stack_size = APP::BALANCING_TASK_STACK_SIZE,
        .priority = APP::TASK_PRIORITY_BALANCE,
        .name = "BalanceTask",
        .auto_start = false
    };
    
    // Configure monitoring task
    monitoring_task_config = {
        .stack_size = APP::MONITOR_TASK_STACK_SIZE,  // Use constant from header
        .priority = APP::TASK_PRIORITY_LOGGING,      // Use appropriate priority
        .name = "MonitorTask",
        .auto_start = false
    };
    
    // Create communication task
    communication_task_config = {
        .stack_size = APP::MGM_TASK_STACK_SIZE,
        .priority = APP::TASK_PRIORITY_MGM_COMM,
        .name = "CommTask",
        .auto_start = false
    };
    
    // Create tasks but don't start them yet
    BaseType_t result = xTaskCreate(
        vBalancingTask,
        balancing_task_config.name,
        balancing_task_config.stack_size,
        this,  // Pass the BalanceRobotApp instance as parameter
        balancing_task_config.priority,
        &xBalancingTaskHandle
    );
    
    if (result != pdPASS) {
        return Platform::Status::ERROR;
    }
    /*//TODO:
    result = xTaskCreate(
        vMonitoringTask,
        monitoring_task_config.name,
        monitoring_task_config.stack_size,
        this,
        monitoring_task_config.priority,
        &xMonitoringTaskHandle
    );
    
    if (result != pdPASS) {
        // Could log warning but continue
    }
    
    result = xTaskCreate(
        vCommunicationTask,
        communication_task_config.name,
        communication_task_config.stack_size,
        this,
        communication_task_config.priority,
        &xCommunicationTaskHandle
    );
    
    if (result != pdPASS) {
        // Could log warning but continue
    }
    */
    return Platform::Status::OK;
}

Platform::Status BalanceRobotApp::StartTasks() {
    if (xBalancingTaskHandle != nullptr) {
        vTaskResume(xBalancingTaskHandle);
    }
    
    if (xMonitoringTaskHandle != nullptr) {
        vTaskResume(xMonitoringTaskHandle);
    }
    
    return Platform::Status::OK;
}

Platform::Status BalanceRobotApp::StopTasks() {
    if (xBalancingTaskHandle != nullptr) {
        vTaskSuspend(xBalancingTaskHandle);
    }
    
    if (xMonitoringTaskHandle != nullptr) {
        vTaskSuspend(xMonitoringTaskHandle);
    }
    
    return Platform::Status::OK;
}

void BalanceRobotApp::RegisterBalancingTask(TaskHandle_t handle) {
    this->xBalancingTaskHandle = handle;
    
    // Now safe to enable interrupts
    RobotConfig.imu_config.interrupt_enabled = true;
    imu->Control(Drivers::Sensors::MPU6050_CTRL_ENABLE_INTERRUPT, nullptr);
}

void vBalancingTask(void* params) {
    // Get the app instance that was passed during task creation
    BalanceRobotApp* app = static_cast<BalanceRobotApp*>(params);
    ProcessType processType = ProcessType::Balancing;
    Platform::GPIO::GpioInterface *test_gpio = &Platform::GPIO::GpioInterface::GetInstance();

    // Register this task's handle so IMU callbacks can notify it
    app->RegisterBalancingTask(xTaskGetCurrentTaskHandle());
    
    // Initialize counters for monitoring
    uint32_t successful_notifications = 0;
    uint32_t timeout_count = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // Wait for notification with timeout
        uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(IMU_I2C_TIMEOUT_TICKS));
        
        if (notificationValue > 0) {
            // We received a proper notification from the IMU data callback
            successful_notifications++;
            
            // Process the balance application with the latest IMU data
            //app->Process(&processType);
            
            // Toggle LED to indicate successful data processing
            test_gpio->TogglePin(Platform::GPIO::Port::PORTC, 0);
        } else {
            // Timeout occurred - no notification received within the specified time
            timeout_count++;
            
            // Handle timeout - maybe check if IMU is still responding
            if (timeout_count > 10) {  // After several consecutive timeouts
                // Log error or attempt recovery
                // app->CheckIMUStatus();
                timeout_count = 0;  // Reset counter after attempting recovery
            }
            
            // You might want to toggle a different pin to indicate timeout
            test_gpio->TogglePin(Platform::GPIO::Port::PORTB, 13);
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(BALANCING_TASK_PERIOD_MS));  
    }
}

void vMonitoringTask(void* params) {
    BalanceRobotApp* app = static_cast<BalanceRobotApp*>(params);
    
    while (true) {
        // Check application state
        AppState state = app->GetState();
        
        // If the application is in error state, try to restart
        if (state == AppState::Error) {
            // Try to re-initialize
            app->Init(nullptr);
        }
        
        // If the application is idle, try to start balancing
        if (state == AppState::Idle) {
            app->Start();
        }
        
        // Delay for monitoring interval (100ms)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

} // namespace APP