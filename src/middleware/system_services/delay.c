// delay.c - Part of system utilities

#include "system_services/delay.h"
#include "hardware_abstraction/timer.h"
#include "hardware_abstraction/systick.h"
#include "platform.h"


static Delay_State_t delay_state = {0};

/**
 * Initialize the delay service
 */
Status_t Delay_Init(Delay_Config_t *config) {
    if (config == NULL) {
        return STATUS_INVALID_PARAM;
    }
    
    if (delay_state.initialized) {
        return STATUS_OK; // Already initialized
    }
    
    delay_state.timer_instance = config->timer_instance;
    
    // Check if using SysTick
    if (config->timer_instance == 0) {
        // Get the SysTick interface
        delay_state.timer_interface = SysTick_GetInterface();
        if (delay_state.timer_interface == NULL) {
            return STATUS_ERROR;
        }
        
        // Configure SysTick for microsecond precision
        // For STM32F401RC at 84MHz, if we want 1µs resolution:
        // Each tick should be 1µs, so we need a reload value of 84 (assuming processor clock)
        SysTick_Config_t systick_config = {
            .reload_value = 84,               // 84 clock cycles = 1µs at 84MHz
            .enable_interrupt = false,        // We don't need interrupts for delay functions
            .use_processor_clock = true       // Use the processor clock (not divided by 8)
        };
        
        // Initialize SysTick with our configuration
        Status_t status = delay_state.timer_interface->Init(delay_state.timer_interface->state, &systick_config);
        if (status != STATUS_OK) {
            return status;
        }
        
        // Start the SysTick counter
        status = delay_state.timer_interface->Control(delay_state.timer_interface->state, SYSTICK_CTRL_START, NULL);
        if (status != STATUS_OK) {
            return status;
        }
        
        delay_state.initialized = true;
        return STATUS_OK;
    }
    
    // Using a timer peripheral
    Timer_Config_t timer_config = {
        .timer_instance = config->timer_instance,
        .mode = TIMER_MODE_BASIC,
        .div_clk = TIMER_CLK_DIV_1,
        .direction = TIMER_DIR_UP,
        .alignment = TIMER_ALIGN_EDGE,
        .prescaler = (config->timer_frequency / 1000000) - 1, // 1MHz timer clock (1µs period)
        .period = 0xFFFFFFFF, // Use maximum period
        .auto_reload_preload = true
    };
    
    // Get timer interface
    delay_state.timer_interface = Timer_GetInterface(config->timer_instance);
    if (delay_state.timer_interface == NULL) {
        return STATUS_ERROR;
    }
    
    // Initialize timer
    Status_t status = delay_state.timer_interface->Init(delay_state.timer_interface->state, &timer_config);
    if (status != STATUS_OK) {
        return status;
    }
    
    // Start the timer
    status = delay_state.timer_interface->Control(delay_state.timer_interface->state, TIMER_CTRL_START, NULL);
    if (status != STATUS_OK) {
        return status;
    }
    
    delay_state.initialized = true;
    return STATUS_OK;
}

/**
 * Delay for a specified number of microseconds
 */
Status_t Delay_Microseconds(uint32_t us, bool blocking) {
    if (!delay_state.initialized) {
        return STATUS_NOT_INITIALIZED;
    }
    // special handling for systick
    if (delay_state.timer_instance == 0) {
        // SysTick counts down, not up
        uint32_t start_count = SysTick_GetCurrentValue();
        uint32_t reload_value = SYSTICK_LOAD + 1; // +1 because it counts from LOAD to 0
        uint64_t current_tick = SysTick_GetTickCount();
        
        // Calculate end point in ticks and microseconds
        uint64_t end_tick = current_tick + (us / reload_value);
        uint32_t remainder_us = us % reload_value;
        
        // For non-blocking delay, store timing info and return
        if (!blocking) {
            delay_state.last_end_tick = end_tick;
            delay_state.last_remainder_us = remainder_us;
            delay_state.last_start_count = start_count;
            return STATUS_OK;
        }
        
        // For blocking delay, wait for the calculated number of ticks
        while (SysTick_GetTickCount() < end_tick) {
            // Wait for full ticks to elapse
        }
        
        // Handle the remainder microseconds within the current tick
        if (remainder_us > 0) {
            uint32_t end_count;
            if (remainder_us >= start_count) {
                // We need to wait for a reload and then some more
                while (SysTick_GetCurrentValue() > start_count) {
                    // Wait for reload to occur
                }
                // Now wait for the remaining count
                end_count = reload_value - (remainder_us - start_count);
                while (SysTick_GetCurrentValue() > end_count) {
                    // Wait for final count
                }
            } else {
                // We can satisfy the remainder in this tick
                end_count = start_count - remainder_us;
                while (SysTick_GetCurrentValue() > end_count && 
                       SysTick_GetCurrentValue() < start_count) {
                    // Wait while being careful about wrap-around
                }
            }
        }
        
        return STATUS_OK;
    }
    // Get current timer value
    uint32_t current_time;
    delay_state.timer_interface->Read(delay_state.timer_interface->state, &current_time, sizeof(current_time), 0);
    
    // Calculate end time (handling 32-bit overflow properly)
    uint32_t end_time = current_time + us;
    
    // For non-blocking delay, just store the end time and return
    if (!blocking) {
        delay_state.last_end_time = end_time;
        return STATUS_OK;
    }
    
    // For blocking delay, wait until the timer reaches the end time
    while (1) {
        delay_state.timer_interface->Read(delay_state.timer_interface->state, &current_time, sizeof(current_time), 0);
        
        // Check if time has elapsed, handling 32-bit overflow
        if ((int32_t)(current_time - end_time) >= 0) {
            break;
        }
    }
    
    return STATUS_OK;
}

/**
 * Delay for a specified number of milliseconds
 */
Status_t Delay_Milliseconds(uint32_t ms, bool blocking) {
    return Delay_Microseconds(ms * 1000, blocking);
}

/**
 * Check if a non-blocking delay has completed
 */
bool Delay_IsCompleted(void) {
    if (!delay_state.initialized) {
        return true; // Assume completed if not initialized
    }
    // Special handling for SysTick
    if (delay_state.timer_instance == 0) {
        uint64_t current_tick = SysTick_GetTickCount();
        
        // Check if the tick count has reached our target
        if (current_tick < delay_state.last_end_tick) {
            return false;
        }
        
        // If exact tick, check remainder
        if (current_tick == delay_state.last_end_tick && delay_state.last_remainder_us > 0) {
            uint32_t reload_value = SYSTICK_LOAD + 1;
            uint32_t start_count = delay_state.last_start_count;
            uint32_t remainder_us = delay_state.last_remainder_us;
            uint32_t current_count = SysTick_GetCurrentValue();
            
            // Complex logic to handle SysTick counting down and wrap-around
            if (remainder_us >= start_count) {
                // Need to have gone through at least one reload
                if (current_count > start_count) {
                    // Has not reloaded yet
                    return false;
                }
                // Now check remaining count
                uint32_t end_count = reload_value - (remainder_us - start_count);
                return current_count <= end_count;
            } else {
                // Can complete within one tick
                uint32_t end_count = start_count - remainder_us;
                return current_count <= end_count || current_count > start_count;
            }
        }
        
        return true; // Timeout has occurred
    }
    // Get current timer value
    uint32_t current_time;
    delay_state.timer_interface->Read(delay_state.timer_interface->state, &current_time, sizeof(current_time), 0);
    
    // Check if time has elapsed, handling 32-bit overflow
    return (int32_t)(current_time - delay_state.last_end_time) >= 0;
}

/**
 * Get current timestamp in microseconds
 */
uint32_t Delay_GetTimestamp(void) {
    if (!delay_state.initialized) {
        return 0;
    }
    
    uint32_t current_time;
    delay_state.timer_interface->Read(delay_state.timer_interface->state, &current_time, sizeof(current_time), 0);
    return current_time;
}