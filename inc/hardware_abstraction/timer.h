#ifndef TIMER_H
#define TIMER_H

#include "hw_interface.hpp"

// Maximum number of timer instances
#define MAX_TIMER_INSTANCES 5

// Timer mode enumeration
typedef enum {
    TIMER_MODE_BASIC,         // Basic timer
    TIMER_MODE_PWM,           // PWM generation
    TIMER_MODE_INPUT_CAPTURE, // Input capture
    TIMER_MODE_OUTPUT_COMPARE // Output compare
} Timer_Mode_t;

// Timer channel enumeration
typedef enum {
    TIMER_CHANNEL_1 = 1,
    TIMER_CHANNEL_2 = 2,
    TIMER_CHANNEL_3 = 3,
    TIMER_CHANNEL_4 = 4
} Timer_Channel_t;

// Timer clock division enumeration
typedef enum {
    TIMER_CLK_DIV_1,
    TIMER_CLK_DIV_2,
    TIMER_CLK_DIV_4
} Timer_ClockDivision_t;

// Timer direction enumeration
typedef enum {
    TIMER_DIR_UP,
    TIMER_DIR_DOWN
} Timer_Direction_t;

// Timer alignment mode enumeration
typedef enum {
    TIMER_ALIGN_EDGE,
    TIMER_ALIGN_CENTER1,
    TIMER_ALIGN_CENTER2,
    TIMER_ALIGN_CENTER3
} Timer_Alignment_t;

// Timer output compare mode enumeration
typedef enum {
    TIMER_OC_MODE_FROZEN,
    TIMER_OC_MODE_ACTIVE,
    TIMER_OC_MODE_INACTIVE,
    TIMER_OC_MODE_TOGGLE,
    TIMER_OC_MODE_FORCE_INACTIVE,
    TIMER_OC_MODE_FORCE_ACTIVE,
    TIMER_OC_MODE_PWM1,
    TIMER_OC_MODE_PWM2
} Timer_OCMode_t;

// Timer base configuration structure
typedef struct {
    uint8_t timer_instance;       // Timer instance (1-5)
    Timer_Mode_t mode;            // Timer operating mode
    Timer_ClockDivision_t div_clk; // Clock division
    Timer_Direction_t direction;  // Count direction
    Timer_Alignment_t alignment;  // Alignment mode
    uint32_t prescaler;           // Timer prescaler
    uint32_t period;              // Timer period (auto-reload value)
    bool auto_reload_preload;     // Auto-reload preload enable
} Timer_Config_t;

// Timer channel configuration structure
typedef struct {
    Timer_Channel_t channel;      // Timer channel
    Timer_OCMode_t oc_mode;       // Output compare mode
    uint32_t pulse;               // Pulse value (capture/compare register)
    bool oc_preload;              // Output compare preload enable
    bool complementary_output;    // Enable complementary output
} Timer_ChannelConfig_t;

// Timer event callback types
typedef enum {
    TIMER_EVENT_UPDATE,          // Update event
    TIMER_EVENT_CC1,             // Capture/Compare 1 event
    TIMER_EVENT_CC2,             // Capture/Compare 2 event
    TIMER_EVENT_CC3,             // Capture/Compare 3 event
    TIMER_EVENT_CC4,             // Capture/Compare 4 event
    TIMER_EVENT_TRIGGER,         // Trigger event
    TIMER_EVENT_BREAK            // Break event
} Timer_Event_t;

// Timer callback data structure
typedef struct {
    void (*callback)(void *param);
    void *param;
} Timer_Callback_t;

// Internal state structure for Timer interface
typedef struct {
    bool initialized;
    Timer_Config_t config;
    Timer_Callback_t callbacks[6]; // One for each event type (UPDATE, CC1-CC4, TRIGGER)
} Timer_State_t;

// Timer control commands
#define TIMER_CTRL_START                0x0201
#define TIMER_CTRL_STOP                 0x0202
#define TIMER_CTRL_CONFIG_CHANNEL       0x0203
#define TIMER_CTRL_SET_PERIOD           0x0204
#define TIMER_CTRL_SET_PRESCALER        0x0205
#define TIMER_CTRL_SET_COMPARE_VALUE    0x0206
#define TIMER_CTRL_ENABLE_IT            0x0207
#define TIMER_CTRL_DISABLE_IT           0x0208

// Initialize Timer hardware interface
HW_Interface_t* Timer_GetInterface(uint8_t timer_instance);

#endif /* TIMER_H */