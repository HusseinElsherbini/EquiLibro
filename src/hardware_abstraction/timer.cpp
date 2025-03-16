// timer_interface.cpp - partial implementation showing key changes

#include "hardware_abstraction/timer.hpp"
#include "hardware_abstraction/rcc.hpp"
#include <vector>
#include <mutex>
#include <memory>
#include "os/mutex.hpp"
#include "platform_cmsis.hpp"
namespace Platform {
namespace TIM {


static OS::mutex timer_instances_mutex;

// Constructor with timer type detection
TimerInterface::TimerInterface(TimerInstance instance) 
    : initialized(false), 
      timer_instance(instance),
      is_running(false) {
    
    // Detect timer type and available features
    timer_type = Platform::TIM::getTimerType(instance);
    available_channels = Platform::TIM::getTimerChannelCount(instance);
    
    // Initialize callback table
    for (auto& cb : callbacks) {
        cb.callback = nullptr;
        cb.param = nullptr;
        cb.enabled = false;
    }
    
    // Initialize status
    status = {};
}
// Implementation for TimerInterface::Init
Platform::Status TimerInterface::Init(void* config) {
    // Check if already initialized
    if (initialized) {
        return Platform::Status::OK;
    }
    
    // Validate input parameter
    if (config == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Cast to the correct configuration type
    TimerConfig* timer_config = static_cast<TimerConfig*>(config);
    
    // Store configuration
    this->config = *timer_config;
    this->timer_instance = timer_config->timerInstance;
    
    // Enable the timer clock
    Platform::Status status = EnableClock();
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Get timer registers
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Reset timer registers to default values
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
    if (timer_config->desiredFrequency > 0) {
        // Get the actual timer clock frequency from RCC
        auto& rcc = Platform::RCC::RccInterface::GetInstance();
        uint32_t timer_clock;
        
        // Determine which bus this timer is on
        if (this->timer_instance == TimerInstance::TIM1 || 
            this->timer_instance == TimerInstance::TIM9 || 
            this->timer_instance == TimerInstance::TIM10 || 
            this->timer_instance == TimerInstance::TIM11) {
            timer_clock = rcc.GetApb2ClockFrequency();
        } else {
            timer_clock = rcc.GetApb1ClockFrequency();
        }
        
        // For timers on APB1/APB2 with a prescaler, the actual timer clock might be multiplied
        if (timer_clock < rcc.GetSystemClockFrequency()) {
            // When APB prescaler is not 1, timer clock is doubled
            timer_clock *= 2;
        }
        
        // Calculate the prescaler to achieve the desired frequency
        timer_config->prescaler = (timer_clock / timer_config->desiredFrequency) - 1;
    }   
    // Configure basic timer parameters
    // Set prescaler
    tim->PSC = timer_config->prescaler;
    
    // Set period (auto-reload value)
    tim->ARR = timer_config->period;
    
    // Configure clock division
    uint32_t cr1 = 0;
    cr1 |= (static_cast<uint32_t>(timer_config->clockDivision) & 0x3) << 8;  // CKD bits
    
    // Configure counting direction
    if (timer_config->direction == Direction::Down) {
        cr1 |= getBitValue(CR1::DIR);  // Set DIR bit for down-counting
    }
    
    // Configure center-aligned mode
    if (timer_config->alignment != Alignment::Edge) {
        cr1 &= ~getBitValue(CR1::CMS_MSK);  // Clear CMS bits
        switch (timer_config->alignment) {
            case Alignment::Center1:
                cr1 |= getBitValue(CR1::CMS_CENTER1);
                break;
            case Alignment::Center2:
                cr1 |= getBitValue(CR1::CMS_CENTER2);
                break;
            case Alignment::Center3:
                cr1 |= getBitValue(CR1::CMS_CENTER3);
                break;
            default:
                break;
        }
    }
    
    // Enable auto-reload preload if requested
    if (timer_config->autoReloadPreload) {
        cr1 |= getBitValue(CR1::ARPE);
    }
    
    // Write configuration to CR1 register
    tim->CR1 = cr1;
    
    // Mark as initialized
    initialized = true;
    is_running = false;
    
    return Platform::Status::OK;
}

// Implementation for TimerInterface::DeInit
Platform::Status TimerInterface::DeInit() {
    // Check if initialized
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Get timer registers
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Stop the timer if running
    if (is_running) {
        Stop();
    }
    
    // Reset all timer registers
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
    
    // Disable timer peripheral clock to save power
    Platform::RCC::RccInterface* rcc = &Platform::RCC::RccInterface::GetInstance();
    Platform::RCC::RccPeripheral timer_peripheral;
    
    // Map timer instance to RCC peripheral
    switch (timer_instance) {
        case TimerInstance::TIM1:
            timer_peripheral = Platform::RCC::RccPeripheral::TIM1;
            break;
        case TimerInstance::TIM2:
            timer_peripheral = Platform::RCC::RccPeripheral::TIM2;
            break;
        case TimerInstance::TIM3:
            timer_peripheral = Platform::RCC::RccPeripheral::TIM3;
            break;
        case TimerInstance::TIM4:
            timer_peripheral = Platform::RCC::RccPeripheral::TIM4;
            break;
        case TimerInstance::TIM5:
            timer_peripheral = Platform::RCC::RccPeripheral::TIM5;
            break;
        // Add cases for other timers as needed
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Disable peripheral clock
    rcc->DisablePeripheralClock(timer_peripheral);
    
    // Reset state
    initialized = false;
    is_running = false;
    
    return Platform::Status::OK;
}

// Implementation for TimerInterface::GetTimerRegister
Platform::TIM::Registers* TimerInterface::GetTimerRegister() const {
    return Platform::TIM::getTimer(static_cast<uint8_t>(timer_instance));
}

// Implementation for TimerInterface::EnableClock
Platform::Status TimerInterface::EnableClock() {
    Platform::RCC::RccInterface* rcc = &Platform::RCC::RccInterface::GetInstance();
    Platform::RCC::RccPeripheral timer_peripheral;
    
    // Map timer instance to RCC peripheral
    switch (timer_instance) {
        case TimerInstance::TIM1:
            timer_peripheral = Platform::RCC::RccPeripheral::TIM1;
            break;
        case TimerInstance::TIM2:
            timer_peripheral = Platform::RCC::RccPeripheral::TIM2;
            break;
        case TimerInstance::TIM3:
            timer_peripheral = Platform::RCC::RccPeripheral::TIM3;
            break;
        case TimerInstance::TIM4:
            timer_peripheral = Platform::RCC::RccPeripheral::TIM4;
            break;
        case TimerInstance::TIM5:
            timer_peripheral = Platform::RCC::RccPeripheral::TIM5;
            break;
        // Add cases for other timers as needed
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Enable peripheral clock
    return rcc->EnablePeripheralClock(timer_peripheral);
}

// Implementation for TimerInterface::Start
Platform::Status TimerInterface::Start() {
    // Check if initialized
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Get timer registers
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Enable counter by setting CEN bit in CR1
    tim->CR1 |= getBitValue(CR1::CEN);
    
    // Update state
    is_running = true;
    
    return Platform::Status::OK;
}

// Implementation for TimerInterface::Stop
Platform::Status TimerInterface::Stop() {
    // Check if initialized
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Get timer registers
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Disable counter by clearing CEN bit in CR1
    tim->CR1 &= ~getBitValue(CR1::CEN);
    
    // Update state
    is_running = false;
    
    return Platform::Status::OK;
}

Platform::TIM::TimerType Platform::TIM::TimerInterface::GetTimerType() const {
    return timer_type;
}
// Implementation for TimerInterface::SetPeriod
Platform::Status TimerInterface::SetPeriod(uint32_t period) {
    // Check if initialized
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Get timer registers
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Check if period is within range for this timer
    // For 16-bit timers, max period is 0xFFFF
    if (!Platform::TIM::hasTimer32BitCounter(timer_instance) && period > 0xFFFF) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Set the auto-reload register (ARR)
    tim->ARR = period;
    
    // Update configuration
    config.period = period;
    
    // Generate update event to load the new ARR value
    // This is needed if auto-reload preload is enabled
    tim->EGR |= getBitValue(EGR::UG);
    
    return Platform::Status::OK;
}

// Implementation for TimerInterface::SetPrescaler
Platform::Status TimerInterface::SetPrescaler(uint32_t prescaler) {
    // Check if initialized
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Get timer registers
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Check if prescaler is within valid range (16-bit value)
    if (prescaler > 0xFFFF) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Set the prescaler register (PSC)
    tim->PSC = prescaler;
    
    // Update configuration
    config.prescaler = prescaler;
    
    // Generate update event to load the new PSC value
    tim->EGR |= getBitValue(EGR::UG);
    
    return Platform::Status::OK;
}

// Implementation for TimerInterface::ConfigureChannel
Platform::Status TimerInterface::ConfigureChannel(const TimerChannelConfig& channelConfig) {
    // Check if initialized
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Validate channel number
    if (static_cast<uint8_t>(channelConfig.channel) > available_channels) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Get timer registers
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Configure channel
    return ConfigureChannel(tim, channelConfig);
}

// Implementation for TimerInterface::ConfigureChannel (helper method)
Platform::Status TimerInterface::ConfigureChannel(Platform::TIM::Registers* tim, const TimerChannelConfig& channelConfig) {
    // Validate parameter
    if (tim == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Configure channel based on channel number
    uint8_t channel_num = static_cast<uint8_t>(channelConfig.channel);
    
    // Set pulse value (CCRx register)
    switch (channelConfig.channel) {
        case Channel::Channel1:
            tim->CCR1 = channelConfig.pulse;
            break;
        case Channel::Channel2:
            tim->CCR2 = channelConfig.pulse;
            break;
        case Channel::Channel3:
            tim->CCR3 = channelConfig.pulse;
            break;
        case Channel::Channel4:
            tim->CCR4 = channelConfig.pulse;
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Configure output compare mode
    if (channel_num <= 2) {
        // Channels 1 and 2 are in CCMR1
        uint32_t ccmr1 = tim->CCMR1;
        
        if (channel_num == 1) {
            // Clear previous mode
            ccmr1 &= ~(getBitValue(CCMR1_Output::CC1S_MSK) | getBitValue(CCMR1_Output::OC1M_MSK));
            
            // Set output mode (CC1S = 00 for output)
            ccmr1 |= getBitValue(CCMR1_Output::CC1S_OUTPUT);
            
            // Set output compare mode
            switch (channelConfig.ocMode) {
                case OCMode::Frozen:
                    ccmr1 |= getBitValue(CCMR1_Output::OC1M_FROZEN);
                    break;
                case OCMode::Active:
                    ccmr1 |= getBitValue(CCMR1_Output::OC1M_ACTIVE);
                    break;
                case OCMode::Inactive:
                    ccmr1 |= getBitValue(CCMR1_Output::OC1M_INACTIVE);
                    break;
                case OCMode::Toggle:
                    ccmr1 |= getBitValue(CCMR1_Output::OC1M_TOGGLE);
                    break;
                case OCMode::ForceInactive:
                    ccmr1 |= getBitValue(CCMR1_Output::OC1M_FORCE_INACTIVE);
                    break;
                case OCMode::ForceActive:
                    ccmr1 |= getBitValue(CCMR1_Output::OC1M_FORCE_ACTIVE);
                    break;
                case OCMode::PWM1:
                    ccmr1 |= getBitValue(CCMR1_Output::OC1M_PWM1);
                    break;
                case OCMode::PWM2:
                    ccmr1 |= getBitValue(CCMR1_Output::OC1M_PWM2);
                    break;
            }
            
            // Set preload enable
            if (channelConfig.ocPreload) {
                ccmr1 |= getBitValue(CCMR1_Output::OC1PE);
            }
        } else { // channel_num == 2
            // Clear previous mode
            ccmr1 &= ~(getBitValue(CCMR1_Output::CC2S_MSK) | getBitValue(CCMR1_Output::OC2M_MSK));
            
            // Set output mode (CC2S = 00 for output)
            ccmr1 |= getBitValue(CCMR1_Output::CC2S_OUTPUT);
            
            // Set output compare mode
            switch (channelConfig.ocMode) {
                case OCMode::Frozen:
                    ccmr1 |= getBitValue(CCMR1_Output::OC2M_FROZEN);
                    break;
                case OCMode::Active:
                    ccmr1 |= getBitValue(CCMR1_Output::OC2M_ACTIVE);
                    break;
                case OCMode::Inactive:
                    ccmr1 |= getBitValue(CCMR1_Output::OC2M_INACTIVE);
                    break;
                case OCMode::Toggle:
                    ccmr1 |= getBitValue(CCMR1_Output::OC2M_TOGGLE);
                    break;
                case OCMode::ForceInactive:
                    ccmr1 |= getBitValue(CCMR1_Output::OC2M_FORCE_INACTIVE);
                    break;
                case OCMode::ForceActive:
                    ccmr1 |= getBitValue(CCMR1_Output::OC2M_FORCE_ACTIVE);
                    break;
                case OCMode::PWM1:
                    ccmr1 |= getBitValue(CCMR1_Output::OC2M_PWM1);
                    break;
                case OCMode::PWM2:
                    ccmr1 |= getBitValue(CCMR1_Output::OC2M_PWM2);
                    break;
            }
            
            // Set preload enable
            if (channelConfig.ocPreload) {
                ccmr1 |= getBitValue(CCMR1_Output::OC2PE);
            }
        }
        
        // Apply changes
        tim->CCMR1 = ccmr1;
    } else { // Channels 3 and 4 are in CCMR2
        uint32_t ccmr2 = tim->CCMR2;
        
        if (channel_num == 3) {
            // Clear previous mode
            ccmr2 &= ~(getBitValue(CCMR2_Output::CC3S_MSK) | getBitValue(CCMR2_Output::OC3M_MSK));
            
            // Set output mode (CC3S = 00 for output)
            ccmr2 |= getBitValue(CCMR2_Output::CC3S_OUTPUT);
            
            // Set output compare mode
            switch (channelConfig.ocMode) {
                case OCMode::Frozen:
                    ccmr2 |= getBitValue(CCMR2_Output::OC3M_FROZEN);
                    break;
                case OCMode::Active:
                    ccmr2 |= getBitValue(CCMR2_Output::OC3M_ACTIVE);
                    break;
                case OCMode::Inactive:
                    ccmr2 |= getBitValue(CCMR2_Output::OC3M_INACTIVE);
                    break;
                case OCMode::Toggle:
                    ccmr2 |= getBitValue(CCMR2_Output::OC3M_TOGGLE);
                    break;
                case OCMode::ForceInactive:
                    ccmr2 |= getBitValue(CCMR2_Output::OC3M_FORCE_INACTIVE);
                    break;
                case OCMode::ForceActive:
                    ccmr2 |= getBitValue(CCMR2_Output::OC3M_FORCE_ACTIVE);
                    break;
                case OCMode::PWM1:
                    ccmr2 |= getBitValue(CCMR2_Output::OC3M_PWM1);
                    break;
                case OCMode::PWM2:
                    ccmr2 |= getBitValue(CCMR2_Output::OC3M_PWM2);
                    break;
            }
            
            // Set preload enable
            if (channelConfig.ocPreload) {
                ccmr2 |= getBitValue(CCMR2_Output::OC3PE);
            }
        } else { // channel_num == 4
            // Clear previous mode
            ccmr2 &= ~(getBitValue(CCMR2_Output::CC4S_MSK) | getBitValue(CCMR2_Output::OC4M_MSK));
            
            // Set output mode (CC4S = 00 for output)
            ccmr2 |= getBitValue(CCMR2_Output::CC4S_OUTPUT);
            
            // Set output compare mode
            switch (channelConfig.ocMode) {
                case OCMode::Frozen:
                    ccmr2 |= getBitValue(CCMR2_Output::OC4M_FROZEN);
                    break;
                case OCMode::Active:
                    ccmr2 |= getBitValue(CCMR2_Output::OC4M_ACTIVE);
                    break;
                case OCMode::Inactive:
                    ccmr2 |= getBitValue(CCMR2_Output::OC4M_INACTIVE);
                    break;
                case OCMode::Toggle:
                    ccmr2 |= getBitValue(CCMR2_Output::OC4M_TOGGLE);
                    break;
                case OCMode::ForceInactive:
                    ccmr2 |= getBitValue(CCMR2_Output::OC4M_FORCE_INACTIVE);
                    break;
                case OCMode::ForceActive:
                    ccmr2 |= getBitValue(CCMR2_Output::OC4M_FORCE_ACTIVE);
                    break;
                case OCMode::PWM1:
                    ccmr2 |= getBitValue(CCMR2_Output::OC4M_PWM1);
                    break;
                case OCMode::PWM2:
                    ccmr2 |= getBitValue(CCMR2_Output::OC4M_PWM2);
                    break;
            }
            
            // Set preload enable
            if (channelConfig.ocPreload) {
                ccmr2 |= getBitValue(CCMR2_Output::OC4PE);
            }
        }
        
        // Apply changes
        tim->CCMR2 = ccmr2;
    }
    
    // Configure output polarity and enable in CCER register
    uint32_t ccer = tim->CCER;
    uint32_t channel_shift = (channel_num - 1) * 4;
    
    // Clear previous configuration
    ccer &= ~(0xF << channel_shift);
    
    // Set channel enable bit
    ccer |= (1UL << channel_shift);
    
    // Configure complementary output if needed (only for TIM1)
    if (channelConfig.complementaryOutput && timer_type == TimerType::Advanced) {
        ccer |= (1UL << (channel_shift + 2));
    }
    
    // SET MOE BIT if using output capture 
    if (timer_type == Platform::TIM::TimerType::Advanced && 
        channelConfig.ocMode != TIM::OCMode::Frozen) {
        
        // Enable MOE bit in BDTR register
        tim->BDTR |= Platform::TIM::getBitValue(Platform::TIM::BDTR::MOE);
    }
    // Apply changes
    tim->CCER = ccer;
    
    return Platform::Status::OK;
}

// Implementation for TimerInterface::EnableInterrupt
Platform::Status TimerInterface::EnableInterrupt(TimerEvent event) {
    // Check if initialized
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Get timer registers
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Enable the appropriate interrupt in DIER register
    switch (event) {
        case TimerEvent::Update:
            tim->DIER |= getBitValue(DIER::UIE);
            break;
        case TimerEvent::CC1:
            tim->DIER |= getBitValue(DIER::CC1IE);
            break;
        case TimerEvent::CC2:
            tim->DIER |= getBitValue(DIER::CC2IE);
            break;
        case TimerEvent::CC3:
            tim->DIER |= getBitValue(DIER::CC3IE);
            break;
        case TimerEvent::CC4:
            tim->DIER |= getBitValue(DIER::CC4IE);
            break;
        case TimerEvent::Trigger:
            tim->DIER |= getBitValue(DIER::TIE);
            break;
        case TimerEvent::Break:
            // Only valid for advanced timers (TIM1)
            if (timer_type != TimerType::Advanced) {
                return Platform::Status::NOT_SUPPORTED;
            }
            tim->DIER |= getBitValue(DIER::BIE);
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Enable NVIC interrupt for this timer
    Platform::CMSIS::NVIC::IRQn irq;
    
    // Map timer instance to NVIC IRQ number
    switch (timer_instance) {
        case TimerInstance::TIM1:
            // TIM1 has multiple IRQ lines, select based on event
            if (event == TimerEvent::Update) {
                irq = Platform::CMSIS::NVIC::IRQn::TIM1_UP_TIM10;
            } else if (event == TimerEvent::CC1 || event == TimerEvent::CC2 || 
                       event == TimerEvent::CC3 || event == TimerEvent::CC4) {
                irq = Platform::CMSIS::NVIC::IRQn::TIM1_CC;
            } else if (event == TimerEvent::Break) {
                irq = Platform::CMSIS::NVIC::IRQn::TIM1_BRK_TIM9;
            } else {
                irq = Platform::CMSIS::NVIC::IRQn::TIM1_TRG_COM_TIM11;
            }
            break;
        case TimerInstance::TIM2:
            irq = Platform::CMSIS::NVIC::IRQn::TIM2;
            break;
        case TimerInstance::TIM3:
            irq = Platform::CMSIS::NVIC::IRQn::TIM3;
            break;
        case TimerInstance::TIM4:
            irq = Platform::CMSIS::NVIC::IRQn::TIM4;
            break;
        case TimerInstance::TIM5:
            irq = Platform::CMSIS::NVIC::IRQn::TIM5;
            break;
        // Add cases for other timers as needed
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Enable interrupt in NVIC
    Platform::CMSIS::NVIC::enableIRQ(irq);
    
    return Platform::Status::OK;
}

// Implementation for TimerInterface::DisableInterrupt
Platform::Status TimerInterface::DisableInterrupt(TimerEvent event) {
    // Check if initialized
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Get timer registers
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Disable the appropriate interrupt in DIER register
    switch (event) {
        case TimerEvent::Update:
            tim->DIER &= ~getBitValue(DIER::UIE);
            break;
        case TimerEvent::CC1:
            tim->DIER &= ~getBitValue(DIER::CC1IE);
            break;
        case TimerEvent::CC2:
            tim->DIER &= ~getBitValue(DIER::CC2IE);
            break;
        case TimerEvent::CC3:
            tim->DIER &= ~getBitValue(DIER::CC3IE);
            break;
        case TimerEvent::CC4:
            tim->DIER &= ~getBitValue(DIER::CC4IE);
            break;
        case TimerEvent::Trigger:
            tim->DIER &= ~getBitValue(DIER::TIE);
            break;
        case TimerEvent::Break:
            // Only valid for advanced timers (TIM1)
            if (timer_type != TimerType::Advanced) {
                return Platform::Status::NOT_SUPPORTED;
            }
            tim->DIER &= ~getBitValue(DIER::BIE);
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Check if all interrupts are disabled, and if so, disable the NVIC interrupt
    if ((tim->DIER & 0x7F) == 0) {
        // All timer interrupt sources are disabled, so disable NVIC interrupt
        Platform::CMSIS::NVIC::IRQn irq;
        
        // Map timer instance to NVIC IRQ number
        switch (timer_instance) {
            case TimerInstance::TIM1:
                // TIM1 has multiple IRQ lines, we would need to disable all
                Platform::CMSIS::NVIC::disableIRQ(Platform::CMSIS::NVIC::IRQn::TIM1_UP_TIM10);
                Platform::CMSIS::NVIC::disableIRQ(Platform::CMSIS::NVIC::IRQn::TIM1_CC);
                Platform::CMSIS::NVIC::disableIRQ(Platform::CMSIS::NVIC::IRQn::TIM1_BRK_TIM9);
                Platform::CMSIS::NVIC::disableIRQ(Platform::CMSIS::NVIC::IRQn::TIM1_TRG_COM_TIM11);
                break;
            case TimerInstance::TIM2:
                Platform::CMSIS::NVIC::disableIRQ(Platform::CMSIS::NVIC::IRQn::TIM2);
                break;
            case TimerInstance::TIM3:
                Platform::CMSIS::NVIC::disableIRQ(Platform::CMSIS::NVIC::IRQn::TIM3);
                break;
            case TimerInstance::TIM4:
                Platform::CMSIS::NVIC::disableIRQ(Platform::CMSIS::NVIC::IRQn::TIM4);
                break;
            case TimerInstance::TIM5:
                Platform::CMSIS::NVIC::disableIRQ(Platform::CMSIS::NVIC::IRQn::TIM5);
                break;
            // Add cases for other timers as needed
            default:
                return Platform::Status::INVALID_PARAM;
        }
    }
    
    return Platform::Status::OK;
}

// Implementation for TimerInterface::RegisterCallback (from HwInterface)
Platform::Status TimerInterface::RegisterCallback(uint32_t eventId, void (*callback)(void* param), void* param) {
    // Check if initialized
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Validate event ID
    if (eventId >= static_cast<uint32_t>(TimerEvent::Count)) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Store callback information
    callbacks[eventId].callback = callback;
    callbacks[eventId].param = param;
    callbacks[eventId].enabled = (callback != nullptr);
    
    return Platform::Status::OK;
}

// Implementation for TimerInterface::LockAccess and UnlockAccess
// These methods would ideally use a mutex for thread safety, but for now
// we'll implement simple versions

void TimerInterface::LockAccess() {
    // In a real implementation, acquire a mutex here
    // For example: timer_mutex.lock();
    
    // For now, this is a no-op
}

void TimerInterface::UnlockAccess() {
    // In a real implementation, release the mutex here
    // For example: timer_mutex.unlock();
    
    // For now, this is a no-op
}
// Destructor with proper cleanup
TimerInterface::~TimerInterface() {
    if (initialized) {
        // Ensure timer is stopped before destruction
        Stop();
        DeInit();
    }
}

// Safe singleton factory 
// Replace the implementation in timer.cpp:
TimerInterface& TimerInterface::GetInstance(TimerInstance instance) {
    if (!Platform::TIM::isValidTimerInstance(instance)) {
        // Since we can't return nullptr with references, default to timer 1
        instance = TimerInstance::TIM1;
    }
    
    // Convert to zero-based index
    size_t index = static_cast<uint8_t>(instance) - 1;
    
    OS::lock_guard<OS::mutex> lock(timer_instances_mutex);
    
    // static array of actual instances
    static TimerInterface instances[TIM_CHANNEL_COUNT] = {
        TimerInterface(TimerInstance::TIM1), TimerInterface(TimerInstance::TIM2), TimerInterface(TimerInstance::TIM3),
        TimerInterface(TimerInstance::TIM4), TimerInterface(TimerInstance::TIM5), TimerInterface(TimerInstance::TIM9),
        TimerInterface(TimerInstance::TIM10), TimerInterface(TimerInstance::TIM11)
    };
    
    return instances[index];
}

// Enhanced control method with type-safe command enumeration
// Timer interface control function with type-safe enum
Platform::Status TimerInterface::Control(TimerOperation operation, void* param) {
    if (!initialized && operation != TimerOperation::Start) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    Platform::Status status = Platform::Status::OK;
    
    switch (operation) {
        case TimerOperation::Start:
            status = Start();
            break;
            
        case TimerOperation::Stop:
            status = Stop();
            break;
            
        case TimerOperation::ConfigureChannel:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            status = ConfigureChannel(*static_cast<TimerChannelConfig*>(param));
            break;
            
        case TimerOperation::SetPeriod:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            status = SetPeriod(*static_cast<uint32_t*>(param));
            break;
            
        case TimerOperation::SetPrescaler:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            status = SetPrescaler(*static_cast<uint32_t*>(param));
            break;
            
        case TimerOperation::SetCompareValue:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            {
                TimerCompareConfig* compare_config = static_cast<TimerCompareConfig*>(param);
                
                Platform::TIM::Registers* tim = GetTimerRegister();
                if (tim == nullptr) {
                    return Platform::Status::ERROR;
                }
                
                // Set the compare value for the specified channel
                switch (compare_config->channel) {
                    case Channel::Channel1:
                        tim->CCR1 = compare_config->value;
                        break;
                    case Channel::Channel2:
                        tim->CCR2 = compare_config->value;
                        break;
                    case Channel::Channel3:
                        tim->CCR3 = compare_config->value;
                        break;
                    case Channel::Channel4:
                        tim->CCR4 = compare_config->value;
                        break;
                    default:
                        return Platform::Status::INVALID_PARAM;
                }
            }
            break;
            
        case TimerOperation::EnableInterrupt:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            status = EnableInterrupt(*static_cast<TimerEvent*>(param));
            break;
            
        case TimerOperation::DisableInterrupt:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            status = DisableInterrupt(*static_cast<TimerEvent*>(param));
            break;
            
        case TimerOperation::ReadCounter:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            status = GetCounterValue(*static_cast<uint32_t*>(param));
            break;
            
        case TimerOperation::WriteCounter:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            status = SetCounterValue(*static_cast<uint32_t*>(param));
            break;
            
        case TimerOperation::EnableCapture:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            {
                Channel* channel = static_cast<Channel*>(param);
                Platform::TIM::Registers* tim = GetTimerRegister();
                if (tim == nullptr) {
                    return Platform::Status::ERROR;
                }
                
                // Enable capture for the specified channel
                uint32_t channel_bit = static_cast<uint32_t>(*channel) * 4;
                LockAccess();
                Platform::setBit(tim->CCER, (1UL << channel_bit));
                UnlockAccess();
            }
            break;
            
        case TimerOperation::DisableCapture:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            {
                Channel* channel = static_cast<Channel*>(param);
                Platform::TIM::Registers* tim = GetTimerRegister();
                if (tim == nullptr) {
                    return Platform::Status::ERROR;
                }
                
                // Disable capture for the specified channel
                uint32_t channel_bit = static_cast<uint32_t>(*channel) * 4;
                //LockAccess();
                Platform::clearBit(tim->CCER, (1UL << channel_bit));
                //UnlockAccess();
            }
            break;
            
        case TimerOperation::GetStatus:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            {
                TimerStatus* timer_status = static_cast<TimerStatus*>(param);
                Platform::TIM::Registers* tim = GetTimerRegister();
                if (tim == nullptr) {
                    return Platform::Status::ERROR;
                }
                
                // Fill in the status structure
                //LockAccess();
                timer_status->counter_value = tim->CNT;
                timer_status->prescaler_value = tim->PSC;
                timer_status->period_value = tim->ARR;
                timer_status->is_running = (tim->CR1 & (1UL << 0)) != 0;
                timer_status->flags = tim->SR;
                //UnlockAccess();
            }
            break;
            case TimerOperation::SetMode:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            {
                uint32_t mode = *static_cast<uint32_t*>(param);
                Platform::TIM::Registers* tim = GetTimerRegister();
                if (tim == nullptr) {
                    return Platform::Status::ERROR;
                }
                
                // Update CR1 register for one-pulse mode
                if (mode == 1) {
                    // Enable one-pulse mode
                    tim->CR1 |= getBitValue(CR1::OPM);
                } else {
                    // Disable one-pulse mode
                    tim->CR1 &= ~getBitValue(CR1::OPM);
                }
            }
            break;
            
        case TimerOperation::EnableChannel:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            {
                Channel channel = *static_cast<Channel*>(param);
                Platform::TIM::Registers* tim = GetTimerRegister();
                if (tim == nullptr) {
                    return Platform::Status::ERROR;
                }
                
                // Enable channel in CCER register
                uint32_t channel_bit = (static_cast<uint32_t>(channel) - 1) * 4;
                tim->CCER |= (1UL << channel_bit);
            }
            break;

        case TimerOperation::DisableChannel:
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            {
                Channel channel = *static_cast<Channel*>(param);
                Platform::TIM::Registers* tim = GetTimerRegister();
                if (tim == nullptr) {
                    return Platform::Status::ERROR;
                }
                
                // Disable channel in CCER register
                uint32_t channel_bit = (static_cast<uint32_t>(channel) - 1) * 4;
                tim->CCER &= ~(1UL << channel_bit);
            }
            break;    
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
    
    return status;
}

// Backward compatibility for HwInterface
Platform::Status TimerInterface::Control(uint32_t command, void* param) {
    // Map old uint32_t commands to new TimerOperation enum
    switch (command) {
        case 0x0201: // TIMER_CTRL_START
            return Control(TimerOperation::Start, param);
            
        case 0x0202: // TIMER_CTRL_STOP
            return Control(TimerOperation::Stop, param);
            
        case 0x0203: // TIMER_CTRL_CONFIG_CHANNEL
            return Control(TimerOperation::ConfigureChannel, param);
            
        case 0x0204: // TIMER_CTRL_SET_PERIOD
            return Control(TimerOperation::SetPeriod, param);
            
        case 0x0205: // TIMER_CTRL_SET_PRESCALER
            return Control(TimerOperation::SetPrescaler, param);

        case 0x0210: // TIMER_CTRL_SET_MODE
            return Control(TimerOperation::SetMode, param);
            
        case 0x0211: // TIMER_CTRL_ENABLE_CHANNEL
            return Control(TimerOperation::EnableChannel, param);
            
        case 0x0212: // TIMER_CTRL_DISABLE_CHANNEL
            return Control(TimerOperation::DisableChannel, param);     

        case 0x0206: // TIMER_CTRL_SET_COMPARE
            return Control(TimerOperation::SetCompareValue, param);
        // Map other old commands...
            
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
}

// Replace generic Read with explicit counter operations
Platform::Status TimerInterface::Read(void* buffer, uint16_t size, uint32_t timeout) {
    if (buffer == nullptr || size < sizeof(uint32_t)) {
        return Platform::Status::INVALID_PARAM;
    }
    
    return GetCounterValue(*static_cast<uint32_t*>(buffer));
}

// Replace generic Write with explicit counter operations
Platform::Status TimerInterface::Write(const void* data, uint16_t size, uint32_t timeout) {
    if (data == nullptr || size < sizeof(uint32_t)) {
        return Platform::Status::INVALID_PARAM;
    }
    
    return SetCounterValue(*static_cast<const uint32_t*>(data));
}

// Explicit counter value access methods
Platform::Status TimerInterface::GetCounterValue(uint32_t& value) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Thread-safe access
    //LockAccess();
    value = tim->CNT;
    //UnlockAccess();
    
    return Platform::Status::OK;
}

Platform::Status TimerInterface::SetCounterValue(uint32_t value) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    Platform::TIM::Registers* tim = GetTimerRegister();
    if (tim == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Check if the value is within valid range based on timer type
    if (!Platform::TIM::hasTimer32BitCounter(timer_instance) && value > 0xFFFF) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Thread-safe access
    //LockAccess();
    tim->CNT = value;
    //UnlockAccess();
    
    return Platform::Status::OK;
}

// TODO: Implement other timer operations...
// timer_interface.cpp - partial implementation showing key changes
// Feature check method
bool TimerInterface::SupportsFeature(TimerFeature feature) const {
    switch (feature) {
        case TimerFeature::PWM:
            // All timers support PWM except basic timers
            return timer_type != TimerType::Basic;
            
        case TimerFeature::OnePulse:
            // All timers support one-pulse mode
            return true;
            
        case TimerFeature::BreakFunction:
            // Only advanced timers support break function
            return timer_type == TimerType::Advanced;
            
        case TimerFeature::DMA:
            // All timers except TIM9, TIM10, TIM11 support DMA
            return timer_instance <= TimerInstance::TIM5;
            
        // Check other features...
            
        default:
            return false;
    }
}
}
}