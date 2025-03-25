// sbr_app_tasks.hpp
#pragma once

#include "FreeRTOS.h"
#include "task.h"

namespace APP {
// Task notifications
constexpr uint32_t NOTIFY_IMU_DATA = (1 << 3);
constexpr uint32_t NOTIFY_SBUS_DATA = (1 << 0);    // Bit 0 for SBUS data
constexpr uint32_t NOTIFY_SPORT_POLL = (1 << 1);    // Bit 1 for SmartPort poll
constexpr uint32_t NOTIFY_LOGGER_TASK_ADC = (1 << 0);

// Task priorities (higher number = higher priority)
constexpr UBaseType_t TASK_PRIORITY_BALANCE = 4;  // High - needs consistent timing
constexpr UBaseType_t TASK_PRIORITY_RC_RECEIVE = 3;  // Medium - user input
constexpr UBaseType_t TASK_PRIORITY_MGM_COMM = 1;  // Low - non-critical display data
constexpr UBaseType_t TASK_PRIORITY_LOGGING = 2;  // non-critical

// Task periods
constexpr uint32_t BALANCING_TASK_PERIOD_MS = 5;    // 200Hz for IMU sampling
constexpr uint32_t RC_TASK_PERIOD_MS = 5;    // 200Hz RC reading
constexpr uint32_t MGM_TASK_PERIOD_MS = 100;  // 10Hz display updates
constexpr uint32_t LOG_TASK_PERIOD_MS = 10;   // 100Hz Logging task

// Stack sizes
constexpr uint32_t BALANCING_TASK_STACK_SIZE = (configMINIMAL_STACK_SIZE * 2);  // Larger for filter calculations
constexpr uint32_t RC_TASK_STACK_SIZE = configMINIMAL_STACK_SIZE;
constexpr uint32_t MGM_TASK_STACK_SIZE = configMINIMAL_STACK_SIZE;
constexpr uint32_t MONITOR_TASK_STACK_SIZE = (configMINIMAL_STACK_SIZE * 2);
constexpr uint32_t LOG_CRITICAL_DATA_TASK_STACK_SIZE = (configMINIMAL_STACK_SIZE * 8);

// IMU task special defines
// Constants
constexpr TickType_t IMU_I2C_TIMEOUT_TICKS = pdMS_TO_TICKS(3);    // 3ms timeout max
constexpr uint32_t MPU6050_TRANSFER_TIME_US = 300;                // Typical I2C transfer time for MPU6050
constexpr TickType_t LOGGING_TIMEOUT_TICKS = pdMS_TO_TICKS(10);


// Task configuration structure
struct TaskConfig {
    uint16_t stack_size;
    UBaseType_t priority;
    const char* name;
    bool auto_start;
};

// Forward declaration for friendship
class BalanceRobotApp;

// Task function declarations
void vBalancingTask(void* params);
void vMonitoringTask(void* params);
void vCommunicationTask(void* params);

} // namespace APP

