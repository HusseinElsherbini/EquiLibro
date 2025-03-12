// timer_interface.cpp - partial implementation showing key changes

#include "hardware_abstraction/timer.hpp"
#include "hardware_abstraction/rcc.hpp"
#include <vector>
#include <mutex>
#include <memory>

namespace Platform {
namespace TIM {
// Static instance management with shared_ptr
static std::vector<std::weak_ptr<TimerInterface>> timer_instances(Platform::TIM::TIM_CHANNEL_COUNT);
static std::mutex timer_instances_mutex;

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

// Destructor with proper cleanup
TimerInterface::~TimerInterface() {
    if (initialized) {
        // Ensure timer is stopped before destruction
        Stop();
        DeInit();
    }
}

// Safe singleton factory with shared_ptr
// Replace the implementation in timer.cpp:
TimerInterface& TimerInterface::GetInstance(uint8_t instance) {
    if (!Platform::TIM::isValidTimerInstance(instance)) {
        // Since we can't return nullptr with references, default to timer 1
        instance = 1;
    }
    
    // Convert to zero-based index
    size_t index = instance - 1;
    
    std::lock_guard<std::mutex> lock(timer_instances_mutex);
    
    // Replace dynamic vector with static array of actual instances
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