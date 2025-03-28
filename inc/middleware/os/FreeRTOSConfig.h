// FreeRTOSConfig.h
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15   // Lowest priority 
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5    // Highest priority that can use FreeRTOS API
#define configPRIO_BITS       4

// Application specific definitions
#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      84000000 // 84 MHz
#define configTICK_RATE_HZ                      1000     // 1kHz tick rate (1ms resolution)
#define configMAX_PRIORITIES                    5
#define configMINIMAL_STACK_SIZE                128
#define configTOTAL_HEAP_SIZE                   8192
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               0
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIME_SLICING                  1
#define configUSE_NEWLIB_REENTRANT              0
#define configUSE_TASK_FPU_SUPPORT              1
#define configCHECK_HANDLER_INSTALLATION        0

// Hook function related definitions
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            0

// Run time and task stats gathering related definitions
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

// Co-routine related definitions
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         1

// Software timer related definitions
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               3
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            configMINIMAL_STACK_SIZE

#define configKERNEL_INTERRUPT_PRIORITY         ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

// Define the proper assert behavior
#define configASSERT(x) if ((x) == 0) { taskDISABLE_INTERRUPTS(); for(;;); }

// FreeRTOS MPU specific definitions.
#define configINCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS 0
#define configTOTAL_MPU_REGIONS                               8
#define configENABLE_FPU                                      1

// Optional functions
#define INCLUDE_vTaskPrioritySet             1
#define INCLUDE_uxTaskPriorityGet            1
#define INCLUDE_vTaskDelete                  1
#define INCLUDE_vTaskSuspend                 1
#define INCLUDE_vTaskDelayUntil              1
#define INCLUDE_vTaskDelay                   1
#define INCLUDE_xTaskGetSchedulerState       1
#define INCLUDE_xTaskGetCurrentTaskHandle    1
#define INCLUDE_uxTaskGetStackHighWaterMark  1
#define INCLUDE_xTaskGetIdleTaskHandle       0
#define INCLUDE_eTaskGetState                1
#define INCLUDE_xTimerPendFunctionCall       1
#define INCLUDE_xTaskAbortDelay              0
#define INCLUDE_xTaskGetHandle               0
#define INCLUDE_xSemaphoreGetMutexHolder     1

// Define the map from interrupt priority bits to logical priority levels

// Definitions needed when configGENERATE_RUN_TIME_STATS is on
#if configGENERATE_RUN_TIME_STATS == 1
    #define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() // Setup timer if needed
    #define portGET_RUN_TIME_COUNTER_VALUE() 0       // Timer value for measuring time
#endif

// Define the function that will be used to clear a line on the debug console
#define configPRINT_STRING(X) // printf or similar debug output

#endif /* FREERTOS_CONFIG_H */