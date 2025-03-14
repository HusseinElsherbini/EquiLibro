// inc/middleware/system_timing.hpp

#pragma once

#include "common/platform.hpp"
#include "common/platform_tim.hpp"
#include "middleware/middleware_module.hpp"
#include "hardware_abstraction/timer.hpp"
#include <functional>
#include <vector>
#include "os/mutex.hpp"
#include <atomic>

namespace Middleware {
namespace SystemServices {
/**
 * @brief Callback type for timer events
 * 
 * @param user_data User data provided during callback registration
 */
using TimerCallback = std::function<void(void* user_data)>;

/**
 * @brief Timer handle structure for managing timer callbacks
 */
struct TimerHandle {
    uint32_t id;               // Unique timer identifier
    bool active;               // Timer active state
    bool periodic;             // Whether timer repeats
    uint32_t period_us;        // Period in microseconds
    uint64_t next_trigger;     // Next trigger timestamp
    TimerCallback callback;    // Callback function
    void* user_data;           // User data for callback
};

/**
 * @brief Time unit enumeration for better readability
 */
enum class TimeUnit {
    Microseconds,
    Milliseconds,
    Seconds
};

/**
 * @brief Configuration structure for system timing
 */
struct SystemTimingConfig {
    Platform::TIM::TimerInstance instance;
    uint32_t timer_input_clk_freq;         // Timer clock frequency in Hz
    bool use_rtos_timing;             // Whether to use RTOS timing when available
    bool enable_power_saving;         // Enable power saving features
};

/**
 * @brief System Timing module providing central timing services
 * 
 * This class provides a comprehensive set of timing services including:
 * - Precise delays (blocking and non-blocking)
 * - High-resolution timestamps
 * - Timer callbacks
 * - Time measurement utilities
 */
class SystemTiming : public MiddlewareModule {
private:
    // Private implementation details
    bool initialized;
    SystemTimingConfig config;
    
    // Hardware timer interface for precise timing
    Platform::TIM::TimerInterface* precision_timer;
    
    // Timer scaling and conversion factors
    uint32_t us_per_tick;
    uint32_t ticks_per_us;
    uint32_t ticks_per_ms;
    
    // Timestamp tracking
    std::atomic<uint64_t> microsecond_counter;
    uint32_t last_timer_count;
    bool timer_wrapped;
    
    // Timer callback management
    std::vector<TimerHandle> timers;
    uint32_t next_timer_id;
    OS::mutex timer_mutex;
    
    // Delay state tracking
    struct DelayState {
        bool in_progress;
        uint64_t end_timestamp;
    };
    DelayState delay_state;
    
    // Private methods
    void UpdateTimestamp();
    void CheckAndTriggerTimers();
    void TimerOverflowCallback(void* param);
    
    // Singleton pattern
    SystemTiming();
    ~SystemTiming();
    SystemTiming(const SystemTiming&) = delete;
    SystemTiming& operator=(const SystemTiming&) = delete;
    
public:
    // Get singleton instance
    static SystemTiming& GetInstance();
    static std::shared_ptr<SystemTiming> GetSharedInstance();
    // MiddlewareModule interface implementation
    Platform::Status Init(void* config) override;
    Platform::Status Process(void* input, void* output) override;
    Platform::Status Configure(uint32_t param_id, void* value) override;
    Platform::Status GetInfo(uint32_t info_id, void* buffer, uint32_t* size) override;
    Platform::Status Reset() override;
    Platform::Status RegisterCallback(uint32_t event, void (*callback)(void* param), void* param) override;
    
    // Core timing functions
    void Delay(uint32_t amount, TimeUnit unit = TimeUnit::Milliseconds);
    bool StartTimeout(uint32_t amount, TimeUnit unit = TimeUnit::Milliseconds);
    bool IsTimeoutComplete();
    bool IsInitialized();
    // High-precision timing functions
    void DelayMicroseconds(uint32_t us);
    void DelayMilliseconds(uint32_t ms);
    bool StartTimeoutMicroseconds(uint32_t us);
    bool StartTimeoutMilliseconds(uint32_t ms);
    
    // Timestamp functions
    uint64_t GetMicroseconds();
    uint64_t GetMilliseconds();
    uint32_t GetSeconds();
    
    // Time measurement utilities
    uint64_t MeasureExecutionTime(std::function<void()> function, TimeUnit unit = TimeUnit::Microseconds);
    
    // Timer callback functions
    uint32_t CreateTimer(uint32_t period_us, TimerCallback callback, void* user_data = nullptr, bool periodic = false);
    bool StartTimer(uint32_t timer_id);
    bool StopTimer(uint32_t timer_id);
    bool DeleteTimer(uint32_t timer_id);
    
    // RTOS integration
    void SetRtosActive(bool active);
    
    // Helper functions for time conversion
    static uint32_t ConvertTime(uint32_t value, TimeUnit from_unit, TimeUnit to_unit);
    Platform::Status EnsureInitialized();
};

// Inline function to get system timing instance
inline SystemTiming& GetSystemTiming() {
    return SystemTiming::GetInstance();
}

// Global convenience functions
inline void DelayMs(uint32_t ms) {
    GetSystemTiming().DelayMilliseconds(ms);
}

inline void DelayUs(uint32_t us) {
    GetSystemTiming().DelayMicroseconds(us);
}

inline uint64_t GetTimestampUs() {
    return GetSystemTiming().GetMicroseconds();
}

inline uint64_t GetTimestampMs() {
    return GetSystemTiming().GetMilliseconds();
}

} // namespace Middleware
} // namespace SystemServices