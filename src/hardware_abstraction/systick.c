// src/hardware_abstraction/systick.c

#include "hardware_abstraction/systick.h"
#include "platform.h"    // For processor-specific definitions


// SysTick Control Register bits
#define SYSTICK_CTRL_ENABLE           (1UL << 0)
#define SYSTICK_CTRL_TICKINT          (1UL << 1)
#define SYSTICK_CTRL_CLKSOURCE        (1UL << 2)
#define SYSTICK_CTRL_COUNTFLAG        (1UL << 16)


// Static instance of SysTick state
static SysTick_State_t systick_state = {false};

// Static instance of SysTick hardware interface
static HW_Interface_t systick_interface;

// Forward declarations for interface functions
static Status_t SysTick_Init(void *state, void *config);
static Status_t SysTick_DeInit(void *state);
static Status_t SysTick_Control(void *state, uint32_t command, void *param);
static Status_t SysTick_Read(void *state, void *buffer, uint16_t size, uint32_t timeout);
static Status_t SysTick_Write(void *state, const void *data, uint16_t size, uint32_t timeout);
static Status_t SysTick_RegisterCallback(void *state, uint32_t eventId, void (*Callback)(void *param), void *param);

/**
 * Initialize SysTick
 */
static Status_t SysTick_Init(void *state, void *config) {
    SysTick_Config_t *systick_config = (SysTick_Config_t*)config;
    if (systick_config == NULL) {
        return STATUS_INVALID_PARAM;
    }
    
    // Validate parameters
    if (systick_config->reload_value < 1 || systick_config->reload_value > 0x00FFFFFF) {
        return STATUS_INVALID_PARAM;
    }
    
    // Stop SysTick if it's running
    SYSTICK_CTRL = 0;
    
    // Clear the current value
    SYSTICK_VAL = 0;
    
    // Set the reload value
    SYSTICK_LOAD = systick_config->reload_value;
    
    // Calculate tick frequency
    uint32_t clock;
    if (systick_config->use_processor_clock) {
        clock = MCU_CLK;  // Processor clock
    } else {
        clock = MCU_CLK / 8;  // Processor clock / 8
    }
    
    systick_state.tick_frequency = clock / systick_config->reload_value;
    
    // Set control register based on configuration
    uint32_t ctrl = 0;
    if (systick_config->enable_interrupt) {
        ctrl |= SYSTICK_CTRL_TICKINT;
    }
    if (systick_config->use_processor_clock) {
        ctrl |= SYSTICK_CTRL_CLKSOURCE;
    }
    
    // Save configuration
    systick_state.config = *systick_config;
    systick_state.tick_count = 0;
    
    // Don't start the counter yet - will be started with SYSTICK_CTRL_START command
    
    // Mark as initialized
    systick_state.initialized = true;
    
    return STATUS_OK;
}

/**
 * De-initialize SysTick
 */
static Status_t SysTick_DeInit(void *state) {
    // Stop SysTick
    SYSTICK_CTRL = 0;
    
    // Reset state
    systick_state.initialized = false;
    
    return STATUS_OK;
}

/**
 * SysTick control function
 */
static Status_t SysTick_Control(void *state, uint32_t command, void *param) {
    if (!systick_state.initialized) {
        return STATUS_NOT_INITIALIZED;
    }
    
    switch (command) {
        case SYSTICK_CTRL_START:
            // Start the SysTick counter
            SYSTICK_CTRL |= SYSTICK_CTRL_ENABLE;
            return STATUS_OK;
            
        case SYSTICK_CTRL_STOP:
            // Stop the SysTick counter
            SYSTICK_CTRL &= ~SYSTICK_CTRL_ENABLE;
            return STATUS_OK;
            
        case SYSTICK_CTRL_RELOAD:
            // Reload the SysTick counter
            SYSTICK_VAL = 0;
            return STATUS_OK;
            
        case SYSTICK_CTRL_GET_TICKFREQ:
            if (param == NULL) {
                return STATUS_INVALID_PARAM;
            }
            *(uint32_t*)param = systick_state.tick_frequency;
            return STATUS_OK;
            
        case SYSTICK_CTRL_SET_CALLBACK:
            // This is redundant since we have RegisterCallback, but included for completeness
            if (param == NULL) {
                return STATUS_INVALID_PARAM;
            }
            
            // Cast param to callback structure
            typedef struct {
                SysTick_CallbackType_t type;
                void (*callback)(void *param);
                void *callback_param;
            } SysTick_CallbackConfig_t;
            
            SysTick_CallbackConfig_t *callback_config = (SysTick_CallbackConfig_t*)param;
            
            if (callback_config->type >= SYSTICK_CALLBACK_MAX) {
                return STATUS_INVALID_PARAM;
            }
            
            systick_state.callbacks[callback_config->type] = callback_config->callback;
            systick_state.callback_params[callback_config->type] = callback_config->callback_param;
            
            return STATUS_OK;
            
        default:
            return STATUS_NOT_SUPPORTED;
    }
}

/**
 * Read SysTick counter
 */
static Status_t SysTick_Read(void *state, void *buffer, uint16_t size, uint32_t timeout) {
    if (!systick_state.initialized) {
        return STATUS_NOT_INITIALIZED;
    }
    
    if (buffer == NULL || size < sizeof(uint64_t)) {
        return STATUS_INVALID_PARAM;
    }
    
    // Read current tick count
    *(uint64_t*)buffer = systick_state.tick_count;
    
    return STATUS_OK;
}

/**
 * Write to SysTick counter (not generally used)
 */
static Status_t SysTick_Write(void *state, const void *data, uint16_t size, uint32_t timeout) {
    // SysTick counter is typically read-only
    return STATUS_NOT_SUPPORTED;
}

/**
 * Register callback for SysTick events
 */
static Status_t SysTick_RegisterCallback(void *state, uint32_t eventId, void (*Callback)(void *param), void *param) {
    if (!systick_state.initialized) {
        return STATUS_NOT_INITIALIZED;
    }
    
    if (eventId >= SYSTICK_CALLBACK_MAX) {
        return STATUS_INVALID_PARAM;
    }
    
    systick_state.callbacks[eventId] = Callback;
    systick_state.callback_params[eventId] = param;
    
    return STATUS_OK;
}

/**
 * Get the SysTick hardware interface
 */
HW_Interface_t* SysTick_GetInterface(void) {
    // Initialize interface function pointers
    systick_interface.state = &systick_state;
    systick_interface.Init = SysTick_Init;
    systick_interface.DeInit = SysTick_DeInit;
    systick_interface.Control = SysTick_Control;
    systick_interface.Read = SysTick_Read;
    systick_interface.Write = SysTick_Write;
    systick_interface.RegisterCallback = SysTick_RegisterCallback;
    
    return &systick_interface;
}

/**
 * SysTick interrupt handler
 */
void SysTick_IRQHandler(void) {
    // Increment tick count
    systick_state.tick_count++;
    
    // Call tick callback if registered
    if (systick_state.callbacks[SYSTICK_CALLBACK_TICK] != NULL) {
        systick_state.callbacks[SYSTICK_CALLBACK_TICK](systick_state.callback_params[SYSTICK_CALLBACK_TICK]);
    }
}

/**
 * Get the current SysTick counter value
 */
uint32_t SysTick_GetCurrentValue(void) {
    return SYSTICK_VAL;
}

/**
 * Get the number of SysTick ticks that have occurred since initialization
 */
uint64_t SysTick_GetTickCount(void) {
    return systick_state.tick_count;
}

/**
 * Convert milliseconds to SysTick ticks
 */
uint32_t SysTick_MillisecondsToTicks(uint32_t ms) {
    return (ms * systick_state.tick_frequency) / 1000;
}

/**
 * Calculate a timeout point in the future
 */
uint64_t SysTick_CalculateTimeout(uint32_t ms) {
    uint32_t ticks = SysTick_MillisecondsToTicks(ms);
    return systick_state.tick_count + ticks;
}

/**
 * Check if a timeout has occurred
 */
bool SysTick_HasTimeoutOccurred(uint64_t timeout) {
    return systick_state.tick_count >= timeout;
}