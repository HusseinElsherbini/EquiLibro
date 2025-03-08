// delay.h - Part of system utilities

#ifndef DELAY_H
#define DELAY_H

#include "common/types.h"
#include "common/error_codes.h"
#include "hardware_abstraction/hw_interface.h"

/**
 * Configuration struct for the delay module
 */
typedef struct {
    uint8_t timer_instance;    // Which timer to use (0 for SysTick)
    uint32_t timer_frequency;  // Timer frequency in Hz
} Delay_Config_t;

// Module state
typedef struct {
    bool initialized;
    uint8_t timer_instance;
    HW_Interface_t *timer_interface;
    uint32_t last_end_time;

    // For SysTick-specific timing
    uint64_t last_end_tick;       // Target tick count
    uint32_t last_remainder_us;   // Microseconds within a tick
    uint32_t last_start_count;    // SysTick starting counter value
} Delay_State_t;

/**
 * Initialize the delay service
 * @param config Configuration parameters
 * @return Status code
 */
Status_t Delay_Init(Delay_Config_t *config);

/**
 * Delay for a specified number of microseconds
 * @param us Delay duration in microseconds
 * @param blocking If true, function waits until delay completes; if false, returns immediately
 * @return Status code
 */
Status_t Delay_Microseconds(uint32_t us, bool blocking);

/**
 * Delay for a specified number of milliseconds
 * @param ms Delay duration in milliseconds
 * @param blocking If true, function waits until delay completes; if false, returns immediately
 * @return Status code
 */
Status_t Delay_Milliseconds(uint32_t ms, bool blocking);

/**
 * Check if a non-blocking delay has completed
 * @return true if delay has completed, false otherwise
 */
bool Delay_IsCompleted(void);

/**
 * Get current timestamp in microseconds
 * @return Current timestamp
 */
uint32_t Delay_GetTimestamp(void);

#endif /* DELAY_H */