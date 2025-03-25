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
    /*//TODO: 
    // Configure monitoring task
    monitoring_task_config = {
        .stack_size = 128,
        .priority = tskIDLE_PRIORITY + 1,
        .name = "MonitorTask",
        .auto_start = false
    };
    */
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
    xBalancingTaskHandle = handle;
    
    // Now safe to enable interrupts
    RobotConfig.imu_config.interrupt_enabled = true;
    imu->Control(Drivers::Sensors::MPU6050_CTRL_ENABLE_INTERRUPT, nullptr);
}

void vBalancingTask(void* params) {
    // Get the app instance that was passed during task creation
    BalanceRobotApp* app = static_cast<BalanceRobotApp*>(params);
    ProcessType processType = ProcessType::Balancing;
    
    app->RegisterBalancingTask(xTaskGetCurrentTaskHandle());

    while (true) {

        uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(IMU_I2C_TIMEOUT_TICKS));
        // Process the balance application
        app->Process(&processType);
        
        // Short delay to yield to other tasks
        vTaskDelay(APP::BALANCING_TASK_PERIOD_MS);
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