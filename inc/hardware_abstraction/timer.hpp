#pragma once    

#include "hw_interface.hpp"
#include "common/platform.hpp"
#include "common/platform_tim.hpp"

using namespace Platform::TIM;
/**
 * Timer hardware interface implementation
 */
class TimerInterface : public HwInterface {
private:
    // Internal state tracking
    bool initialized;
    
    // Timer instance this interface is controlling
    uint8_t timerInstance;
    
    // Current configuration
    TimerConfig config;
    
    // Callback table for timer events
    struct {
        void (*callback)(void* param);
        void* param;
    } callbacks[7]; // One for each TimerEvent
    
    // Helper methods
    Platform::TIM::Registers* GetTimerRegister();
    Platform::Status ConfigureChannel(Platform::TIM::Registers* tim, const TimerChannelConfig& channelConfig);
    Platform::Status EnableClock();
    
public:
    // Constructor
    explicit TimerInterface(uint8_t instance);
    
    // Destructor
    ~TimerInterface() override;
    
    // Interface implementation
    Platform::Status Init(void* config) override;
    Platform::Status DeInit() override;
    Platform::Status Control(uint32_t command, void* param) override;
    Platform::Status Read(void* buffer, uint16_t size, uint32_t timeout) override;
    Platform::Status Write(const void* data, uint16_t size, uint32_t timeout) override;
    Platform::Status RegisterCallback(uint32_t eventId, void (*callback)(void* param), void* param) override;
    
    // Timer-specific methods
    Platform::Status Start();
    Platform::Status Stop();
    Platform::Status SetPeriod(uint32_t period);
    Platform::Status SetPrescaler(uint32_t prescaler);
    Platform::Status ConfigureChannel(const TimerChannelConfig& channelConfig);
    Platform::Status EnableInterrupt(TimerEvent event);
    Platform::Status DisableInterrupt(TimerEvent event);
    
    // Static function to get a timer interface for a specific instance
    static TimerInterface& GetInstance(uint8_t instance);
};

// Timer control command identifiers
constexpr uint32_t TIMER_CTRL_START = 0x0201;
constexpr uint32_t TIMER_CTRL_STOP = 0x0202;
constexpr uint32_t TIMER_CTRL_CONFIG_CHANNEL = 0x0203;
constexpr uint32_t TIMER_CTRL_SET_PERIOD = 0x0204;
constexpr uint32_t TIMER_CTRL_SET_PRESCALER = 0x0205;
constexpr uint32_t TIMER_CTRL_SET_COMPARE_VALUE = 0x0206;
constexpr uint32_t TIMER_CTRL_ENABLE_IT = 0x0207;
constexpr uint32_t TIMER_CTRL_DISABLE_IT = 0x0208;
