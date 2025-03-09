// timer.cpp
#include "hardware_abstraction/timer.hpp"
#include "hardware_abstraction/rcc.hpp"
#include "common/platform.hpp"



// Array of TimerInterface instances (one per timer)
static TimerInterface* timerInstances[Platform::TIM::TIM_CHANNEL_COUNT] = {nullptr};

// Constructor
TimerInterface::TimerInterface(uint8_t instance) 
    : initialized(false), 
      timerInstance(instance) {
    
    // Initialize callbacks array
    for (auto& cb : callbacks) {
        cb.callback = nullptr;
        cb.param = nullptr;
    }
}

// Destructor
TimerInterface::~TimerInterface() {
    if (initialized) {
        DeInit();
    }
}

// Get TimerInterface singleton for a specific timer instance
TimerInterface& TimerInterface::GetInstance(uint8_t instance) {
    if (instance < 1 || instance > 5) {
        // Invalid instance, return timer 1 as fallback
        instance = 1;
    }
    
    // Create instance if it doesn't exist
    if (timerInstances[instance - 1] == nullptr) {
        timerInstances[instance - 1] = new TimerInterface(instance);
    }
    
    return *timerInstances[instance - 1];
}

// Get timer peripheral registers
Platform::TIM::Registers* TimerInterface::GetTimerRegister() {
    return Platform::TIM::getTimer(timerInstance);
}

// Enable clock to the timer peripheral
Platform::Status TimerInterface::EnableClock() {
    RccInterface& rcc = RccInterface::GetInstance();
    
    // Map timer instance to RCC peripheral
    Platform::RCC::RccPeripheral rccPeripheral;
    switch (timerInstance) {
        case 1:
            rccPeripheral = Platform::RCC::RccPeripheral::TIM1;
            break;
        case 2:
            rccPeripheral = Platform::RCC::RccPeripheral::TIM2;
            break;
        case 3:
            rccPeripheral = Platform::RCC::RccPeripheral::TIM3;
            break;
        case 4:
            rccPeripheral = Platform::RCC::RccPeripheral::TIM4;
            break;
        case 5:
            rccPeripheral = Platform::RCC::RccPeripheral::TIM5;
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Enable the peripheral clock
    return rcc.EnablePeripheralClock(rccPeripheral);
}

// Configure a timer channel
Platform::Status TimerInterface::ConfigureChannel(Platform::TIM::Registers* tim, const TimerChannelConfig& channelConfig) {
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    uint32_t ccmrx_offset;
    uint32_t ccer_bit_offset;
    
    // Determine which CCMR register to use based on channel
    if (static_cast<int>(channelConfig.channel) <= 2) {
        ccmrx_offset = 0; // CCMR1
    } else {
        ccmrx_offset = 1; // CCMR2
    }
    
    // Calculate bit positions within the register
    uint32_t ccmr_channel_offset = ((static_cast<int>(channelConfig.channel) - 1) % 2) * 8;
    ccer_bit_offset = (static_cast<int>(channelConfig.channel) - 1) * 4;
    
    // Configure the output compare mode
    if (ccmrx_offset == 0) {
        // CCMR1 register
        uint32_t ccmr1 = tim->CCMR1;
        ccmr1 &= ~(0x7UL << (ccmr_channel_offset + 4));
        ccmr1 |= (static_cast<uint32_t>(channelConfig.ocMode) << (ccmr_channel_offset + 4));
        
        // Configure output compare preload
        if (channelConfig.ocPreload) {
            ccmr1 |= (1UL << (ccmr_channel_offset + 3));
        } else {
            ccmr1 &= ~(1UL << (ccmr_channel_offset + 3));
        }
        
        tim->CCMR1 = ccmr1;
    } else {
        // CCMR2 register
        uint32_t ccmr2 = tim->CCMR2;
        ccmr2 &= ~(0x7UL << (ccmr_channel_offset + 4));
        ccmr2 |= (static_cast<uint32_t>(channelConfig.ocMode) << (ccmr_channel_offset + 4));
        
        // Configure output compare preload
        if (channelConfig.ocPreload) {
            ccmr2 |= (1UL << (ccmr_channel_offset + 3));
        } else {
            ccmr2 &= ~(1UL << (ccmr_channel_offset + 3));
        }
        
        tim->CCMR2 = ccmr2;
    }
    
    // Set capture/compare register value
    switch (channelConfig.channel) {
        case Platform::TIM::Channel::Channel1:
            tim->CCR1 = channelConfig.pulse;
            break;
        case Platform::TIM::Channel::Channel2:
            tim->CCR2 = channelConfig.pulse;
            break;
        case Platform::TIM::Channel::Channel3:
            tim->CCR3 = channelConfig.pulse;
            break;
        case Platform::TIM::Channel::Channel4:
            tim->CCR4 = channelConfig.pulse;
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Enable/disable complementary output
    uint32_t ccer = tim->CCER;
    if (channelConfig.complementaryOutput) {
        ccer |= (1UL << (ccer_bit_offset + 2));
    } else {
        ccer &= ~(1UL << (ccer_bit_offset + 2));
    }
    
    // Enable channel
    ccer |= (1UL << ccer_bit_offset);
    tim->CCER = ccer;
    
    return Platform::Status::OK;
}

// Initialize timer
Platform::Status TimerInterface::Init(void* config) {
    // Check if already initialized
    if (initialized) {
        return Platform::Status::OK; // Already initialized
    }
    
    TimerConfig* timerConfig = static_cast<TimerConfig*>(config);
    if (timerConfig == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Get timer registers
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Save the configuration
    this->config = *timerConfig;
    
    // Enable timer clock
    Platform::Status status = EnableClock();
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Reset timer registers
    tim->CR1 = 0;
    
    // Configure clock division
    uint32_t clock_div_bits;
    switch (timerConfig->clockDivision) {
        case Platform::TIM::ClockDivision::Div1: clock_div_bits = 0; break;
        case Platform::TIM::ClockDivision::Div2: clock_div_bits = 1; break;
        case Platform::TIM::ClockDivision::Div4: clock_div_bits = 2; break;
        default: return Platform::Status::INVALID_PARAM;
    }
    
    uint32_t cr1 = tim->CR1;
    cr1 &= ~Platform::TIM::getBitValue(Platform::TIM::CR1::CKD_MSK);
    cr1 |= (clock_div_bits << 8);
    
    // Configure counter direction
    uint32_t dir_bit = (timerConfig->direction == Platform::TIM::Direction::Down) ? 1 : 0;
    cr1 &= ~Platform::TIM::getBitValue(Platform::TIM::CR1::DIR);
    cr1 |= (dir_bit << 4);
    
    // Configure alignment mode
    uint32_t align_bits;
    switch (timerConfig->alignment) {
        case Platform::TIM::Alignment::Edge:    align_bits = 0; break;
        case Platform::TIM::Alignment::Center1: align_bits = 1; break;
        case Platform::TIM::Alignment::Center2: align_bits = 2; break;
        case Platform::TIM::Alignment::Center3: align_bits = 3; break;
        default: return Platform::Status::INVALID_PARAM;
    }
    cr1 &= ~Platform::TIM::getBitValue(Platform::TIM::CR1::CMS_MSK);
    cr1 |= (align_bits << 5);
    
    // Configure auto-reload preload
    if (timerConfig->autoReloadPreload) {
        cr1 |= Platform::TIM::getBitValue(Platform::TIM::CR1::ARPE);
    } else {
        cr1 &= ~Platform::TIM::getBitValue(Platform::TIM::CR1::ARPE);
    }
    
    // Apply CR1 configuration
    tim->CR1 = cr1;
    
    // Set prescaler
    tim->PSC = timerConfig->prescaler;
    
    // Set auto-reload value (period)
    tim->ARR = timerConfig->period;
    
    // Generate update event to load prescaler and period
    tim->EGR = Platform::TIM::getBitValue(Platform::TIM::EGR::UG);
    
    // Mark as initialized
    initialized = true;
    
    return Platform::Status::OK;
}

// Deinitialize timer
Platform::Status TimerInterface::DeInit() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Disable counter
    tim->CR1 &= ~Platform::TIM::getBitValue(Platform::TIM::CR1::CEN);
    
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
    initialized = false;
    
    return Platform::Status::OK;
}

// Control timer
Platform::Status TimerInterface::Control(uint32_t command, void* param) {
    if (!initialized && command != TIMER_CTRL_START) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    switch (command) {
        case TIMER_CTRL_START:
            return Start();
            
        case TIMER_CTRL_STOP:
            return Stop();
            
        case TIMER_CTRL_CONFIG_CHANNEL:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            return ConfigureChannel(*static_cast<TimerChannelConfig*>(param));
            
        case TIMER_CTRL_SET_PERIOD:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            return SetPeriod(*static_cast<uint32_t*>(param));
            
        case TIMER_CTRL_SET_PRESCALER:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            return SetPrescaler(*static_cast<uint32_t*>(param));
            
        case TIMER_CTRL_ENABLE_IT:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            return EnableInterrupt(*static_cast<TimerEvent*>(param));
            
        case TIMER_CTRL_DISABLE_IT:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            return DisableInterrupt(*static_cast<TimerEvent*>(param));
            
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
}

// Read timer counter value
Platform::Status TimerInterface::Read(void* buffer, uint16_t size, uint32_t timeout) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (buffer == nullptr || size < sizeof(uint32_t)) {
        return Platform::Status::INVALID_PARAM;
    }
    
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Read current counter value
    *static_cast<uint32_t*>(buffer) = tim->CNT;
    
    return Platform::Status::OK;
}

// Write to timer counter
Platform::Status TimerInterface::Write(const void* data, uint16_t size, uint32_t timeout) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (data == nullptr || size < sizeof(uint32_t)) {
        return Platform::Status::INVALID_PARAM;
    }
    
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Write to counter register
    tim->CNT = *static_cast<const uint32_t*>(data);
    
    return Platform::Status::OK;
}

// Register callback for timer events
Platform::Status TimerInterface::RegisterCallback(uint32_t eventId, void (*callback)(void* param), void* param) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (eventId >= 7) { // We have 7 callback slots (UPDATE, CC1-4, TRIGGER, BREAK)
        return Platform::Status::INVALID_PARAM;
    }
    
    // Register the callback
    callbacks[eventId].callback = callback;
    callbacks[eventId].param = param;
    
    return Platform::Status::OK;
}

// Start timer
Platform::Status TimerInterface::Start() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Start the timer
    tim->CR1 |= Platform::TIM::getBitValue(Platform::TIM::CR1::CEN);
    
    return Platform::Status::OK;
}

// Stop timer
Platform::Status TimerInterface::Stop() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Stop the timer
    tim->CR1 &= ~Platform::TIM::getBitValue(Platform::TIM::CR1::CEN);
    
    return Platform::Status::OK;
}

// Set timer period
Platform::Status TimerInterface::SetPeriod(uint32_t period) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Set auto-reload value
    tim->ARR = period;
    
    // Update the stored configuration
    config.period = period;
    
    return Platform::Status::OK;
}

// Set timer prescaler
Platform::Status TimerInterface::SetPrescaler(uint32_t prescaler) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Set prescaler value
    tim->PSC = prescaler;
    
    // Update the stored configuration
    config.prescaler = prescaler;
    
    return Platform::Status::OK;
}

// Configure a timer channel
Platform::Status TimerInterface::ConfigureChannel(const TimerChannelConfig& channelConfig) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    return ConfigureChannel(tim, channelConfig);
}

// Enable timer interrupt
Platform::Status TimerInterface::EnableInterrupt(TimerEvent event) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Enable interrupt based on event type
    switch (event) {
        case TimerEvent::Update:
            tim->DIER |= Platform::TIM::getBitValue(Platform::TIM::DIER::UIE);
            break;
        case TimerEvent::CC1:
            tim->DIER |= Platform::TIM::getBitValue(Platform::TIM::DIER::CC1IE);
            break;
        case TimerEvent::CC2:
            tim->DIER |= Platform::TIM::getBitValue(Platform::TIM::DIER::CC2IE);
            break;
        case TimerEvent::CC3:
            tim->DIER |= Platform::TIM::getBitValue(Platform::TIM::DIER::CC3IE);
            break;
        case TimerEvent::CC4:
            tim->DIER |= Platform::TIM::getBitValue(Platform::TIM::DIER::CC4IE);
            break;
        case TimerEvent::Trigger:
            tim->DIER |= Platform::TIM::getBitValue(Platform::TIM::DIER::TIE);
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    return Platform::Status::OK;
}

// Disable timer interrupt
Platform::Status TimerInterface::DisableInterrupt(TimerEvent event) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Disable interrupt based on event type
    switch (event) {
        case TimerEvent::Update:
            tim->DIER &= ~Platform::TIM::getBitValue(Platform::TIM::DIER::UIE);
            break;
        case TimerEvent::CC1:
            tim->DIER &= ~Platform::TIM::getBitValue(Platform::TIM::DIER::CC1IE);
            break;
        case TimerEvent::CC2:
            tim->DIER &= ~Platform::TIM::getBitValue(Platform::TIM::DIER::CC2IE);
            break;
        case TimerEvent::CC3:
            tim->DIER &= ~Platform::TIM::getBitValue(Platform::TIM::DIER::CC3IE);
            break;
        case TimerEvent::CC4:
            tim->DIER &= ~Platform::TIM::getBitValue(Platform::TIM::DIER::CC4IE);
            break;
        case TimerEvent::Trigger:
            tim->DIER &= ~Platform::TIM::getBitValue(Platform::TIM::DIER::TIE);
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    return Platform::Status::OK;
}