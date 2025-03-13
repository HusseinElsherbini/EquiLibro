// src/middleware/system_timing.cpp

#include "system_services/system_timing.hpp"
#include "hardware_abstraction/timer.hpp"
#include "hardware_abstraction/systick.hpp"
#include "hardware_abstraction/rcc.hpp"
#include <memory>

#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

namespace Middleware {
namespace SystemServices {
// Static instance for singleton pattern
SystemTiming& SystemTiming::GetInstance() {
    
    static SystemTiming instance;
    return instance;
}
// Get a shared_ptr to the static instance
static std::shared_ptr<SystemTiming> GetSharedInstance() {
    // Get the raw pointer to the static instance
    SystemTiming* raw_ptr = &SystemTiming::GetInstance();
    
    // Create a shared_ptr that doesn't own/delete the static instance
    // We use a custom deleter that does nothing
    return std::shared_ptr<SystemTiming>(raw_ptr, [](SystemTiming*) {
        // Empty deleter - we don't want to delete the static instance
        // when the shared_ptr is destroyed
    });
}

bool SystemTiming::IsInitialized() {
    return initialized;
}
// Constructor
SystemTiming::SystemTiming()
    : initialized(false),
      microsecond_counter(0),
      last_timer_count(0),
      timer_wrapped(false),
      next_timer_id(1),
      precision_timer(nullptr) {
    
    // Initialize delay state
    delay_state.in_progress = false;
    delay_state.end_timestamp = 0;
}

// Destructor
SystemTiming::~SystemTiming() {
    if (initialized) {
        // Clean up resources
    }
}

// Initialize the system timing module
Platform::Status SystemTiming::Init(void* config) {
    if (initialized) {
        return Platform::Status::OK; // Already initialized
    }
    
    // Validate input configuration
    if (config == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    this->config = *static_cast<SystemTimingConfig*>(config);
    
    // Get a timer for high-precision timing
    // TIM2 or TIM5 are 32-bit timers which are ideal for this purpose
    precision_timer = &TimerInterface::GetInstance(this->config.high_precision_timer);
    
    // Configure the timer for microsecond precision
    TimerConfig timer_config = {};
    timer_config.timerInstance = this->config.high_precision_timer;
    timer_config.mode = Platform::TIM::Mode::Basic;
    timer_config.direction = Platform::TIM::Direction::Up;
    timer_config.alignment = Platform::TIM::Alignment::Edge;
    
    // Calculate the prescaler to get microsecond resolution
    // For example, if the timer clock is 84MHz, we need a prescaler of 84 to get 1MHz timer frequency
    uint32_t timer_clock = this->config.timer_frequency;
    timer_config.prescaler = timer_clock / 1000000UL - 1; // Prescaler for 1MHz (1us resolution)
    timer_config.period = 0xFFFFFFFF; // Maximum period for a 32-bit timer
    timer_config.autoReloadPreload = true;
    
    // Initialize the timer
    Platform::Status status = precision_timer->Init(&timer_config);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Calculate timing conversion factors
    us_per_tick = 1; // Each tick is 1us with our prescaler
    ticks_per_us = 1;
    ticks_per_ms = 1000;
    
    // Register callback for timer overflow
    precision_timer->RegisterCallback(static_cast<uint32_t>(TimerEvent::Update), 
                                    [](void* param) {
                                        SystemTiming* timing = static_cast<SystemTiming*>(param);
                                        timing->TimerOverflowCallback(nullptr);
                                    },
                                    this);
    
    // Start the timer
    precision_timer->Control(TimerOperation::Start, nullptr);
    
    // Initialize timestamp tracking
    uint32_t initial_count;
    precision_timer->GetCounterValue(initial_count);
    last_timer_count = initial_count;
    microsecond_counter = 0;
    
    // Initialize the timer callback vector
    timers.clear();
    next_timer_id = 1;
    
    initialized = true;
    return Platform::Status::OK;
}

// Process function - called periodically to update timers and timestamps
Platform::Status SystemTiming::Process(void* input, void* output) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Update the microsecond counter
    UpdateTimestamp();
    
    // Check and trigger any elapsed timers
    CheckAndTriggerTimers();
    
    return Platform::Status::OK;
}

// Configure the timing module with various parameters
Platform::Status SystemTiming::Configure(uint32_t param_id, void* value) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (value == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    switch (param_id) {
        case MIDDLEWARE_PARAM_RESET_STATE:
            // Reset the timing module state
            return Reset();
            
        case MIDDLEWARE_PARAM_UPDATE_RATE:
            // Adjust update rate/precision
            // Example: Adjust timer prescaler for different precision
            {
                uint32_t update_rate = *static_cast<uint32_t*>(value);
                // Reconfigure timer based on new update rate
                // ...
            }
            return Platform::Status::OK;
            
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
}

// Get various timing module information
Platform::Status SystemTiming::GetInfo(uint32_t info_id, void* buffer, uint32_t* size) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (buffer == nullptr || size == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    switch (info_id) {
        case MIDDLEWARE_INFO_VERSION:
            // Return version information
            if (*size < sizeof(uint32_t)) {
                return Platform::Status::BUFFER_OVERFLOW;
            }
            *static_cast<uint32_t*>(buffer) = 0x00010000; // Version 1.0.0
            *size = sizeof(uint32_t);
            return Platform::Status::OK;
            
        case MIDDLEWARE_INFO_CAPABILITIES:
            // Return capabilities information
            if (*size < sizeof(uint32_t)) {
                return Platform::Status::BUFFER_OVERFLOW;
            }
            *static_cast<uint32_t*>(buffer) = 0x0000001F; // Example capabilities bitmap
            *size = sizeof(uint32_t);
            return Platform::Status::OK;
            
        case MIDDLEWARE_INFO_STATISTICS:
            // Return timing statistics
            // Example: timer accuracy, jitter, etc.
            // ...
            return Platform::Status::OK;
            
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
}

// Reset the timing module to initial state
Platform::Status SystemTiming::Reset() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Reset timer counters
    precision_timer->Control(TimerOperation::Stop, nullptr);
    
    // Reset counter value
    uint32_t zero = 0;
    precision_timer->Control(TimerOperation::WriteCounter, &zero);
    
    // Reset microsecond counter
    microsecond_counter = 0;
    last_timer_count = 0;
    timer_wrapped = false;
    
    // Clear delay state
    delay_state.in_progress = false;
    delay_state.end_timestamp = 0;
    
    // Clear all timers
    {
        std::lock_guard<std::mutex> lock(timer_mutex);
        timers.clear();
        next_timer_id = 1;
    }
    
    // Restart the timer
    precision_timer->Control(TimerOperation::Start, nullptr);
    
    return Platform::Status::OK;
}

// Register a callback for timing events
Platform::Status SystemTiming::RegisterCallback(uint32_t event, void (*callback)(void* param), void* param) {
    // This implementation is simplified - in a real system, you would have a more sophisticated
    // callback registration system for different timing events
    
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Register the callback
    switch (event) {
        case 0: // Example: Timer overflow event
            // Store the callback for timer overflow
            // ...
            return Platform::Status::OK;
            
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
}

// Update internal timestamp counter based on timer value
void SystemTiming::UpdateTimestamp() {
    uint32_t current_count;
    precision_timer->GetCounterValue(current_count);
    
    // Calculate elapsed ticks, handling overflow
    uint32_t elapsed;
    if (current_count >= last_timer_count) {
        elapsed = current_count - last_timer_count;
    } else {
        // Timer has wrapped around
        elapsed = (0xFFFFFFFF - last_timer_count) + current_count + 1;
        timer_wrapped = true;
    }
    
    // Update microsecond counter
    if (elapsed > 0) {
        microsecond_counter += elapsed * us_per_tick;
        last_timer_count = current_count;
    }
}

// Check and trigger elapsed timer callbacks
void SystemTiming::CheckAndTriggerTimers() {
    std::lock_guard<std::mutex> lock(timer_mutex);
    
    uint64_t current_time = GetMicroseconds();
    
    for (auto& timer : timers) {
        if (timer.active && current_time >= timer.next_trigger) {
            // Timer has elapsed, trigger callback
            if (timer.callback) {
                timer.callback(timer.user_data);
            }
            
            // Update next trigger time for periodic timers
            if (timer.periodic) {
                timer.next_trigger = current_time + timer.period_us;
            } else {
                timer.active = false;
            }
        }
    }
}

// Callback for timer overflow events
void SystemTiming::TimerOverflowCallback(void* param) {
    // This is called when the hardware timer overflows
    // Update our internal counter for precise long-term timing
    timer_wrapped = true;
}

// Blocking delay function with configurable time unit
void SystemTiming::Delay(uint32_t amount, TimeUnit unit) {
    switch (unit) {
        case TimeUnit::Microseconds:
            DelayMicroseconds(amount);
            break;
        case TimeUnit::Milliseconds:
            DelayMilliseconds(amount);
            break;
        case TimeUnit::Seconds:
            DelayMilliseconds(amount * 1000);
            break;
    }
}

// High-precision microsecond delay
void SystemTiming::DelayMicroseconds(uint32_t us) {
    if (us == 0) {
        return;
    }
    
#ifdef USE_FREERTOS
    // For very short delays, use busy-waiting even when RTOS is active
    if (us < 1000 && config.use_rtos_timing) {
        // Short delays use busy-waiting for better precision
        uint64_t start_time = GetMicroseconds();
        while ((GetMicroseconds() - start_time) < us) {
            // This is a busy-wait loop
            __asm__ volatile("nop");
        }
    } else if (config.use_rtos_timing) {
        // Convert to ticks and use RTOS delay
        TickType_t ticks = (us / 1000) / portTICK_PERIOD_MS;
        if (ticks > 0) {
            vTaskDelay(ticks);
        } else {
            // For delays less than one tick, use busy-waiting
            uint64_t start_time = GetMicroseconds();
            while ((GetMicroseconds() - start_time) < us) {
                // Allow other tasks to run
                taskYIELD();
            }
        }
    } else {
#endif
        // Direct hardware timer-based delay for maximum precision
        uint64_t start_time = GetMicroseconds();
        while ((GetMicroseconds() - start_time) < us) {
            // This is a busy-wait loop
            __asm("nop");
        }
#ifdef USE_FREERTOS
    }
#endif
}

// Millisecond delay function
void SystemTiming::DelayMilliseconds(uint32_t ms) {
    if (ms == 0) {
        return;
    }
    
#ifdef USE_FREERTOS
    if (config.use_rtos_timing) {
        // Use FreeRTOS scheduler for delays if active
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
#endif
        // Fall back to microsecond delay
        DelayMicroseconds(ms * 1000);
#ifdef USE_FREERTOS
    }
#endif
}

// Start a non-blocking timeout
bool SystemTiming::StartTimeout(uint32_t amount, TimeUnit unit) {
    switch (unit) {
        case TimeUnit::Microseconds:
            return StartTimeoutMicroseconds(amount);
        case TimeUnit::Milliseconds:
            return StartTimeoutMilliseconds(amount);
        case TimeUnit::Seconds:
            return StartTimeoutMilliseconds(amount * 1000);
        default:
            return false;
    }
}

// Start a non-blocking microsecond timeout
bool SystemTiming::StartTimeoutMicroseconds(uint32_t us) {
    if (delay_state.in_progress) {
        return false; // Timeout already in progress
    }
    
    delay_state.in_progress = true;
    delay_state.end_timestamp = GetMicroseconds() + us;
    
    return true;
}

// Start a non-blocking millisecond timeout
bool SystemTiming::StartTimeoutMilliseconds(uint32_t ms) {
    return StartTimeoutMicroseconds(ms * 1000);
}

// Check if a non-blocking timeout has completed
bool SystemTiming::IsTimeoutComplete() {
    if (!delay_state.in_progress) {
        return true; // No timeout in progress, so it's "complete"
    }
    
    if (GetMicroseconds() >= delay_state.end_timestamp) {
        delay_state.in_progress = false;
        return true;
    }
    
    return false;
}

// Get timestamp in microseconds
uint64_t SystemTiming::GetMicroseconds() {
    // Update timestamp before returning
    UpdateTimestamp();
    return microsecond_counter;
}

// Get timestamp in milliseconds
uint64_t SystemTiming::GetMilliseconds() {
    return GetMicroseconds() / 1000;
}

// Get timestamp in seconds
uint32_t SystemTiming::GetSeconds() {
    return static_cast<uint32_t>(GetMicroseconds() / 1000000);
}

// Measure execution time of a function
uint64_t SystemTiming::MeasureExecutionTime(std::function<void()> function, TimeUnit unit) {
    uint64_t start_time = GetMicroseconds();
    
    // Execute the function
    function();
    
    uint64_t end_time = GetMicroseconds();
    uint64_t elapsed_us = end_time - start_time;
    
    // Convert to requested time unit
    switch (unit) {
        case TimeUnit::Microseconds:
            return elapsed_us;
        case TimeUnit::Milliseconds:
            return elapsed_us / 1000;
        case TimeUnit::Seconds:
            return elapsed_us / 1000000;
        default:
            return elapsed_us;
    }
}

// Create a timer with callback
uint32_t SystemTiming::CreateTimer(uint32_t period_us, TimerCallback callback, void* user_data, bool periodic) {
    if (!initialized || !callback) {
        return 0; // Invalid timer ID
    }
    
    std::lock_guard<std::mutex> lock(timer_mutex);
    
    // Create a new timer handle
    TimerHandle timer;
    timer.id = next_timer_id++;
    timer.active = false;
    timer.periodic = periodic;
    timer.period_us = period_us;
    timer.next_trigger = 0;
    timer.callback = callback;
    timer.user_data = user_data;
    
    // Add to the timers vector
    timers.push_back(timer);
    
    return timer.id;
}

// Start a timer
bool SystemTiming::StartTimer(uint32_t timer_id) {
    if (!initialized) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(timer_mutex);
    
    // Find the timer by ID
    for (auto& timer : timers) {
        if (timer.id == timer_id) {
            timer.active = true;
            timer.next_trigger = GetMicroseconds() + timer.period_us;
            return true;
        }
    }
    
    return false; // Timer not found
}

// Stop a timer
bool SystemTiming::StopTimer(uint32_t timer_id) {
    if (!initialized) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(timer_mutex);
    
    // Find the timer by ID
    for (auto& timer : timers) {
        if (timer.id == timer_id) {
            timer.active = false;
            return true;
        }
    }
    
    return false; // Timer not found
}

// Delete a timer
bool SystemTiming::DeleteTimer(uint32_t timer_id) {
    if (!initialized) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(timer_mutex);
    
    // Find and remove the timer by ID
    for (auto it = timers.begin(); it != timers.end(); ++it) {
        if (it->id == timer_id) {
            timers.erase(it);
            return true;
        }
    }
    
    return false; // Timer not found
}

// Set RTOS active state
void SystemTiming::SetRtosActive(bool active) {
    if (initialized) {
        config.use_rtos_timing = active;
    }
}

// Convert time between different units
uint32_t SystemTiming::ConvertTime(uint32_t value, TimeUnit from_unit, TimeUnit to_unit) {
    // Convert to microseconds first
    uint64_t us;
    switch (from_unit) {
        case TimeUnit::Microseconds:
            us = value;
            break;
        case TimeUnit::Milliseconds:
            us = static_cast<uint64_t>(value) * 1000;
            break;
        case TimeUnit::Seconds:
            us = static_cast<uint64_t>(value) * 1000000;
            break;
        default:
            return 0;
    }
    
    // Then convert to target unit
    switch (to_unit) {
        case TimeUnit::Microseconds:
            return static_cast<uint32_t>(us);
        case TimeUnit::Milliseconds:
            return static_cast<uint32_t>(us / 1000);
        case TimeUnit::Seconds:
            return static_cast<uint32_t>(us / 1000000);
        default:
            return 0;
    }
}

} // namespace Middleware
} // namespace SystemServices