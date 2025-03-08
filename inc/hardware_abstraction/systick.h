// inc/hardware_abstraction/systick.h

#ifndef SYSTICK_H
#define SYSTICK_H

#include "hw_interface.h"
#include "common/types.h"
#include "common/error_codes.h"

/**
 * SysTick configuration structure
 */


typedef struct {
    uint32_t reload_value;       // SysTick reload value (1 to 0x00FFFFFF)
    bool enable_interrupt;       // Enable SysTick interrupt
    bool use_processor_clock;    // true: use processor clock, false: use external clock (HCLK/8)
} SysTick_Config_t;

/**
 * SysTick event callback types
 */
typedef enum {
    SYSTICK_CALLBACK_TICK,      // Called on each SysTick tick
    SYSTICK_CALLBACK_MAX
} SysTick_CallbackType_t;

// Internal state structure for SysTick interface
typedef struct {
    bool initialized;
    SysTick_Config_t config;
    uint64_t tick_count;
    uint32_t tick_frequency;
    void (*callbacks[SYSTICK_CALLBACK_MAX])(void *param);
    void *callback_params[SYSTICK_CALLBACK_MAX];
} SysTick_State_t;

/**
 * Initialize SysTick hardware interface
 * 
 * @return Pointer to hardware interface structure
 */
HW_Interface_t* SysTick_GetInterface(void);

/**
 * SysTick interrupt handler - should be called from the SysTick_Handler()
 */
void SysTick_IRQHandler(void);

/**
 * Get the current SysTick counter value
 * 
 * @return Current counter value (0 to reload_value)
 */
uint32_t SysTick_GetCurrentValue(void);

/**
 * Get the number of SysTick ticks that have occurred since initialization
 * 
 * @return Tick count
 */
uint64_t SysTick_GetTickCount(void);

/**
 * Convert milliseconds to SysTick ticks
 * 
 * @param ms Milliseconds to convert
 * @return Equivalent number of ticks
 */
uint32_t SysTick_MillisecondsToTicks(uint32_t ms);

/**
 * Calculate a timeout point in the future
 * 
 * @param ms Milliseconds in the future
 * @return Tick value representing the timeout point
 */
uint64_t SysTick_CalculateTimeout(uint32_t ms);

/**
 * Check if a timeout has occurred
 * 
 * @param timeout Timeout point previously calculated with SysTick_CalculateTimeout
 * @return true if timeout has occurred, false otherwise
 */
bool SysTick_HasTimeoutOccurred(uint64_t timeout);

// Control commands for SysTick interface
#define SYSTICK_CTRL_START              0x0801    // Start SysTick counter
#define SYSTICK_CTRL_STOP               0x0802    // Stop SysTick counter
#define SYSTICK_CTRL_RELOAD             0x0803    // Reload SysTick counter
#define SYSTICK_CTRL_GET_TICKFREQ       0x0804    // Get tick frequency in Hz
#define SYSTICK_CTRL_SET_CALLBACK       0x0805    // Set callback for tick event

#endif /* SYSTICK_H */