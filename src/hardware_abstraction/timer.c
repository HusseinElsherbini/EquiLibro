#include "hardware_abstraction/timer.h"
#include "platform.h"  // Includes MCU-specific register definitions


// Static instances of Timer state for each timer
static Timer_State_t timer_states[MAX_TIMER_INSTANCES] = {0};

// Static instances of Timer hardware interface for each timer
static HW_Interface_t timer_interfaces[MAX_TIMER_INSTANCES];

// Forward declarations for interface functions
static Status_t Timer_Init(void *state, void *config);
static Status_t Timer_DeInit(void *state);
static Status_t Timer_Control(void *state, uint32_t command, void *param);
static Status_t Timer_Read(void *buffer, uint16_t size, uint32_t timeout);
static Status_t Timer_Write(const void *data, uint16_t size, uint32_t timeout);
static Status_t Timer_RegisterCallback(void *state, uint32_t eventId, void (*Callback)(void *param), void *param);


// Map timer instance to TIM peripheral
static TIM_REGS_T* Timer_GetTimerRegister(uint8_t timer_instance) {
    switch (timer_instance) {
        case 1: return TIM1;
        case 2: return TIM2;
        case 3: return TIM3;
        case 4: return TIM4;
        case 5: return TIM5;
        default: return NULL;
    }
}

// Map timer instance to timer state array index
static int Timer_GetTimerIndex(uint8_t timer_instance) {
    // Timer instances are typically 1-based, our array is 0-based
    if (timer_instance >= 1 && timer_instance <= MAX_TIMER_INSTANCES) {
        return timer_instance - 1;
    }
    return -1;
}

// Enable clock to timer
static Status_t Timer_EnableClock(uint8_t timer_instance) {
    switch (timer_instance) {
        case 1:
            SET_BIT(RCC_REGS->APB2ENR, RCC_APB2ENR_TIM1EN);
            break;
        case 2:
            SET_BIT(RCC_REGS->APB1ENR, RCC_APB1ENR_TIM2EN);
            break;
        case 3:
            SET_BIT(RCC_REGS->APB1ENR, RCC_APB1ENR_TIM3EN);
            break;
        case 4:
            SET_BIT(RCC_REGS->APB1ENR, RCC_APB1ENR_TIM4EN);
            break;
        case 5:
            SET_BIT(RCC_REGS->APB1ENR, RCC_APB1ENR_TIM5EN);
            break;
        default:
            return STATUS_INVALID_PARAM;
    }
    return STATUS_OK;
}

// Configure timer channel
static Status_t Timer_ConfigChannel(TIM_REGS_T *tim, const Timer_ChannelConfig_t *channel_config) {
    uint32_t ccmrx_offset;
    uint32_t ccer_bit_offset;
    
    // Determine which CCMR register to use based on channel
    if (channel_config->channel == TIMER_CHANNEL_1 || channel_config->channel == TIMER_CHANNEL_2) {
        ccmrx_offset = 0; // CCMR1
    } else {
        ccmrx_offset = 1; // CCMR2
    }
    
    // Calculate bit positions within the register
    uint32_t ccmr_channel_offset = ((channel_config->channel - 1) % 2) * 8;
    ccer_bit_offset = (channel_config->channel - 1) * 4;
    
    // Configure the output compare mode
    if (ccmrx_offset == 0) {
        // CCMR1 register
        MODIFY_REG(tim->CCMR1, 
                   (0x7 << (ccmr_channel_offset + 4)), 
                   (channel_config->oc_mode << (ccmr_channel_offset + 4)));
        
        // Configure output compare preload
        if (channel_config->oc_preload) {
            SET_BIT(tim->CCMR1, (1 << (ccmr_channel_offset + 3)));
        } else {
            CLEAR_BIT(tim->CCMR1, (1 << (ccmr_channel_offset + 3)));
        }
    } else {
        // CCMR2 register
        MODIFY_REG(tim->CCMR2, 
                   (0x7 << (ccmr_channel_offset + 4)), 
                   (channel_config->oc_mode << (ccmr_channel_offset + 4)));
        
        // Configure output compare preload
        if (channel_config->oc_preload) {
            SET_BIT(tim->CCMR2, (1 << (ccmr_channel_offset + 3)));
        } else {
            CLEAR_BIT(tim->CCMR2, (1 << (ccmr_channel_offset + 3)));
        }
    }
    
    // Set capture/compare register value
    switch (channel_config->channel) {
        case TIMER_CHANNEL_1:
            tim->CCR1 = channel_config->pulse;
            break;
        case TIMER_CHANNEL_2:
            tim->CCR2 = channel_config->pulse;
            break;
        case TIMER_CHANNEL_3:
            tim->CCR3 = channel_config->pulse;
            break;
        case TIMER_CHANNEL_4:
            tim->CCR4 = channel_config->pulse;
            break;
        default:
            return STATUS_INVALID_PARAM;
    }
    
    // Enable/disable complementary output
    if (channel_config->complementary_output) {
        SET_BIT(tim->CCER, (1 << (ccer_bit_offset + 2)));
    } else {
        CLEAR_BIT(tim->CCER, (1 << (ccer_bit_offset + 2)));
    }
    
    // Enable channel
    SET_BIT(tim->CCER, (1 << ccer_bit_offset));
    
    return STATUS_OK;
}

// Implementation of Timer initialization
static Status_t Timer_Init(void *state, void *config) {
    Timer_State_t *timer_state = (Timer_State_t*)state;
    Timer_Config_t *timer_config = (Timer_Config_t*)config;

    if (timer_state == NULL) {
        return STATUS_INVALID_PARAM;
    }

    if (timer_config == NULL) {
        return STATUS_INVALID_PARAM;
    }
    
    int timer_idx = Timer_GetTimerIndex(timer_config->timer_instance);
    if (timer_idx < 0) {
        return STATUS_INVALID_PARAM;
    }
    
    TIM_REGS_T *tim = Timer_GetTimerRegister(timer_config->timer_instance);
    if (tim == NULL) {
        return STATUS_INVALID_PARAM;
    }
    
    // Save the configuration
    timer_state->config = *timer_config;
    
    // Enable timer clock
    Status_t status = Timer_EnableClock(timer_config->timer_instance);
    if (status != STATUS_OK) {
        return status;
    }
    
    // Reset timer registers
    tim->CR1 = 0;
    
    // Configure clock division
    uint32_t clock_div_bits;
    switch (timer_config->div_clk) {
        case TIMER_CLK_DIV_1: clock_div_bits = 0; break;
        case TIMER_CLK_DIV_2: clock_div_bits = 1; break;
        case TIMER_CLK_DIV_4: clock_div_bits = 2; break;
        default: return STATUS_INVALID_PARAM;
    }
    MODIFY_REG(tim->CR1, TIM_CR1_CKD_MASK, (clock_div_bits << 8));
    
    // Configure counter direction
    uint32_t dir_bit = (timer_config->direction == TIMER_DIR_DOWN) ? 1 : 0;
    MODIFY_REG(tim->CR1, TIM_CR1_DIR_MASK, (dir_bit << 4));
    
    // Configure alignment mode
    uint32_t align_bits;
    switch (timer_config->alignment) {
        case TIMER_ALIGN_EDGE:    align_bits = 0; break;
        case TIMER_ALIGN_CENTER1: align_bits = 1; break;
        case TIMER_ALIGN_CENTER2: align_bits = 2; break;
        case TIMER_ALIGN_CENTER3: align_bits = 3; break;
        default: return STATUS_INVALID_PARAM;
    }
    MODIFY_REG(tim->CR1, TIM_CR1_CMS_MASK, (align_bits << 5));
    
    // Configure auto-reload preload
    if (timer_config->auto_reload_preload) {
        SET_BIT(tim->CR1, TIM_CR1_ARPE_Msk);
    } else {
        CLEAR_BIT(tim->CR1, TIM_CR1_ARPE_Msk);
    }
    
    // Set prescaler
    tim->PSC = timer_config->prescaler;
    
    // Set auto-reload value (period)
    tim->ARR = timer_config->period;
    
    // Generate update event to load prescaler and period
    SET_BIT(tim->EGR, TIM_EGR_UG);
    
    // Mark as initialized
    timer_state->initialized = true;
    
    return STATUS_OK;
}

// Implementation of Timer de-initialization
static Status_t Timer_DeInit(void *state) {

    Timer_State_t *timer_state = (Timer_State_t*)state;


    int timer_idx = Timer_GetTimerIndex(timer_state->config.timer_instance);
    
    if (timer_idx < 0 || !timer_states[timer_idx].initialized) {
        return STATUS_NOT_INITIALIZED;
    }
    
    TIM_REGS_T *tim = Timer_GetTimerRegister(timer_state->config.timer_instance);
    if (tim == NULL) {
        return STATUS_ERROR;
    }
    
    // Disable counter
    CLEAR_BIT(tim->CR1, TIM_CR1_CEN);
    
    // Reset all registers
    tim->CR1 = 0;
    tim->CR2 = 0;
    tim->SMCR = 0;
    tim->DIER = 0;
    tim->SR = 0;
    tim->EGR = 0;
    tim->CCMR1 = 0;
    tim->CCMR2 = 0;
    tim->CCER = 0;
    tim->CNT = 0;
    tim->PSC = 0;
    tim->ARR = 0;
    tim->CCR1 = 0;
    tim->CCR2 = 0;
    tim->CCR3 = 0;
    tim->CCR4 = 0;
    
    // Mark as not initialized
    timer_state->initialized = false;
    
    return STATUS_OK;
}

// Implementation of Timer control function
static Status_t Timer_Control(void *state, uint32_t command, void *param) {

    Timer_State_t *timer_state = (Timer_State_t*)state;
    Timer_Config_t *timer_config = &timer_state->config;

    int timer_idx = Timer_GetTimerIndex(timer_config->timer_instance);
    
    if (timer_idx < 0 || !timer_states[timer_idx].initialized) {
        return STATUS_NOT_INITIALIZED;
    }
    
    TIM_REGS_T *tim = Timer_GetTimerRegister(timer_config->timer_instance);
    if (tim == NULL) {
        return STATUS_ERROR;
    }
    
    switch (command) {
        case TIMER_CTRL_START:
            SET_BIT(tim->CR1, TIM_CR1_CEN);
            return STATUS_OK;
            
        case TIMER_CTRL_STOP:
            CLEAR_BIT(tim->CR1, TIM_CR1_CEN);
            return STATUS_OK;
            
        case TIMER_CTRL_CONFIG_CHANNEL:
            if (param == NULL) {
                return STATUS_INVALID_PARAM;
            }
            return Timer_ConfigChannel(tim, (Timer_ChannelConfig_t*)param);
            
        case TIMER_CTRL_SET_PERIOD:
            if (param == NULL) {
                return STATUS_INVALID_PARAM;
            }
            tim->ARR = *((uint32_t*)param);
            return STATUS_OK;
            
        case TIMER_CTRL_SET_PRESCALER:
            if (param == NULL) {
                return STATUS_INVALID_PARAM;
            }
            tim->PSC = *((uint32_t*)param);
            return STATUS_OK;
            
        case TIMER_CTRL_SET_COMPARE_VALUE:
            if (param == NULL) {
                return STATUS_INVALID_PARAM;
            }
            // Param is a struct with channel and value
            Timer_ChannelConfig_t *cc_config = (Timer_ChannelConfig_t*)param;
            Timer_Event_t *event = (Timer_Event_t*)param;
           
            // Enable interrupt based on event type
            switch (*event) {
                case TIMER_EVENT_UPDATE:
                    SET_BIT(tim->DIER, TIM_DIER_UIE);
                    break;
                case TIMER_EVENT_CC1:
                    SET_BIT(tim->DIER, TIM_DIER_CC1IE);
                    break;
                case TIMER_EVENT_CC2:
                    SET_BIT(tim->DIER, TIM_DIER_CC2IE);
                    break;
                case TIMER_EVENT_CC3:
                    SET_BIT(tim->DIER, TIM_DIER_CC3IE);
                    break;
                case TIMER_EVENT_CC4:
                    SET_BIT(tim->DIER, TIM_DIER_CC4IE);
                    break;
                case TIMER_EVENT_TRIGGER:
                    SET_BIT(tim->DIER, TIM_DIER_TIE);
                    break;
                default:
                    return STATUS_INVALID_PARAM;
            }
            return STATUS_OK;
            
        case TIMER_CTRL_DISABLE_IT:
            if (param == NULL) {
                return STATUS_INVALID_PARAM;
            }
            
            Timer_Event_t *disable_event = (Timer_Event_t*)param;
            
            // Disable interrupt based on event type
            switch (*disable_event) {
                case TIMER_EVENT_UPDATE:
                    CLEAR_BIT(tim->DIER, TIM_DIER_UIE);
                    break;
                case TIMER_EVENT_CC1:
                    CLEAR_BIT(tim->DIER, TIM_DIER_CC1IE);
                    break;
                case TIMER_EVENT_CC2:
                    CLEAR_BIT(tim->DIER, TIM_DIER_CC2IE);
                    break;
                case TIMER_EVENT_CC3:
                    CLEAR_BIT(tim->DIER, TIM_DIER_CC3IE);
                    break;
                case TIMER_EVENT_CC4:
                    CLEAR_BIT(tim->DIER, TIM_DIER_CC4IE);
                    break;
                case TIMER_EVENT_TRIGGER:
                    CLEAR_BIT(tim->DIER, TIM_DIER_TIE);
                    break;
                default:
                    return STATUS_INVALID_PARAM;
            }
            return STATUS_OK;
            
        default:
            return STATUS_NOT_SUPPORTED;
    }
 }
 
 // Implementation of Timer read function
 static Status_t Timer_Read(void *buffer, uint16_t size, uint32_t timeout) {
    Timer_Config_t *timer_config = (Timer_Config_t*)timer_interfaces[0].state;
    int timer_idx = Timer_GetTimerIndex(timer_config->timer_instance);
    
    if (timer_idx < 0 || !timer_states[timer_idx].initialized) {
        return STATUS_NOT_INITIALIZED;
    }
    
    if (buffer == NULL || size < sizeof(uint32_t)) {
        return STATUS_INVALID_PARAM;
    }
    
    TIM_REGS_T *tim = Timer_GetTimerRegister(timer_config->timer_instance);
    if (tim == NULL) {
        return STATUS_ERROR;
    }
    
    // Read current counter value
    *((uint32_t*)buffer) = tim->CNT;
    
    return STATUS_OK;
 }
 
 // Implementation of Timer write function - not typically used
 static Status_t Timer_Write(const void *data, uint16_t size, uint32_t timeout) {
    Timer_Config_t *timer_config = (Timer_Config_t*)timer_interfaces[0].state;
    int timer_idx = Timer_GetTimerIndex(timer_config->timer_instance);
    
    if (timer_idx < 0 || !timer_states[timer_idx].initialized) {
        return STATUS_NOT_INITIALIZED;
    }
    
    if (data == NULL || size < sizeof(uint32_t)) {
        return STATUS_INVALID_PARAM;
    }
    
    TIM_REGS_T *tim = Timer_GetTimerRegister(timer_config->timer_instance);
    if (tim == NULL) {
        return STATUS_ERROR;
    }
    
    // Write to counter register
    tim->CNT = *((uint32_t*)data);
    
    return STATUS_OK;
 }
 
 // Implementation of Timer callback registration
 static Status_t Timer_RegisterCallback(void *state, uint32_t eventId, void (*Callback)(void *param), void *param) {

    Timer_State_t *timer_state = (Timer_State_t*)state;
    Timer_Config_t *timer_config = &timer_state->config;

    int timer_idx = Timer_GetTimerIndex(timer_config->timer_instance);
    
    if (timer_idx < 0 || !timer_states[timer_idx].initialized) {
        return STATUS_NOT_INITIALIZED;
    }
    
    if (eventId >= 6) { // We have 6 callback slots (UPDATE, CC1-4, TRIGGER)
        return STATUS_INVALID_PARAM;
    }
    
    // Register the callback
    timer_states[timer_idx].callbacks[eventId].callback = Callback;
    timer_states[timer_idx].callbacks[eventId].param = param;
    
    return STATUS_OK;
 }
 
 // Get the Timer hardware interface
 HW_Interface_t* Timer_GetInterface(uint8_t timer_instance) {
    int timer_idx = Timer_GetTimerIndex(timer_instance);
    if (timer_idx < 0) {
        return NULL;
    }
    
    // Initialize interface function pointers for this timer instance
    HW_Interface_t *interface = &timer_interfaces[timer_idx];
    interface->state = &timer_states[timer_idx];
    interface->Init = Timer_Init;
    interface->DeInit = Timer_DeInit;
    interface->Control = Timer_Control;
    interface->Read = Timer_Read;
    interface->Write = Timer_Write;
    interface->RegisterCallback = Timer_RegisterCallback;
    
    return interface;
 }