#pragma once 

#include "app_module.hpp"


/**
    * App that tests the hardware abstraction peripherals
 */

// inc/application/gpio_toggle_app.hpp

#pragma once

#include "application/app_module.hpp"
#include "hardware_abstraction/gpio.hpp"
#include "middleware/system_services/system_timing.hpp"
#include <memory>

namespace APP {

/**
 * Configuration structure for the GPIO toggle application
 */
struct GpioToggleConfig {
    // GPIO configuration
    Platform::GPIO::Port gpio_port;     // Which GPIO port to use
    uint8_t gpio_pin;                   // Which pin to toggle
    
    // Timing configuration
    uint32_t toggle_interval_ms;        // How often to toggle the pin (milliseconds)
    
    // Optional parameters
    bool start_state;                   // Initial state of the pin (true = high, false = low)
    bool enable_debug_output;           // Enable debug messages
};

/**
 * Simple application that toggles a GPIO pin at a specified interval
 */
class GpioToggleApp : public ApplicationModule {
private:
    // Configuration
    GpioToggleConfig config;
    
    // State tracking
    AppState current_state;
    bool is_pin_high;
    uint64_t last_toggle_time;
    
    // Hardware interfaces
    Platform::GPIO::GpioInterface* gpio_interface;
    Middleware::SystemServices::SystemTiming* timing_service;
    
    // Callback storage
    struct CallbackEntry {
        void (*callback)(void* param);
        void* param;
        bool active;
    };
    
    // Define your events
    static constexpr uint32_t EVENT_PIN_TOGGLED = 0x1;
    static constexpr uint32_t EVENT_ERROR = 0x2;
    
    std::unordered_map<uint32_t, CallbackEntry> callbacks;

public:
    /**
     * Constructor
     * 
     * @param gpio GPIO interface to use
     * @param timing Timing service to use
     */
    GpioToggleApp();
    
    /**
     * Destructor
     */
    ~GpioToggleApp() override;
    
    // ApplicationModule interface implementation
    
    /**
     * Get current application state
     */
    AppState GetState() const override;
    
    /**
     * Initialize the GPIO toggle application
     * 
     * @param config Pointer to GpioToggleConfig or nullptr for default
     * @return Status code indicating success or failure
     */
    Platform::Status Init(void* config) override;
    
    /**
     * Start the GPIO toggle operation
     * 
     * @return Status code indicating success or failure
     */
    Platform::Status Start() override;
    
    /**
     * Stop the GPIO toggle operation
     * 
     * @return Status code indicating success or failure
     */
    Platform::Status Stop() override;
    
    /**
     * Process function - called periodically to update state
     * 
     * @param params Additional parameters (not used)
     * @return Status code indicating success or failure
     */
    Platform::Status Process(void* params) override;
    
    /**
     * Handle application commands
     * 
     * @param cmd_id Command identifier
     * @param params Command parameters
     * @return Status code indicating success or failure
     */
    Platform::Status HandleCommand(uint32_t cmd_id, void* params) override;
    
    /**
     * Get application status
     * 
     * @param status Buffer to store status information
     * @param size Size of the buffer
     * @return Status code indicating success or failure
     */
    Platform::Status GetStatus(void* status, uint32_t* size) override;
    
    /**
     * Register callback for application events
     * 
     * @param event Event identifier
     * @param callback Callback function
     * @param param Parameter to pass to callback
     * @return Status code indicating success or failure
     */
    Platform::Status RegisterCallback(uint32_t event, void (*callback)(void* param), void* param) override;
    
    // Application-specific methods
    
    /**
     * Set toggle interval
     * 
     * @param interval_ms New interval in milliseconds
     * @return Status code indicating success or failure
     */
    Platform::Status SetToggleInterval(uint32_t interval_ms);
    
    /**
     * Force a pin toggle immediately
     * 
     * @return Status code indicating success or failure
     */
    Platform::Status ForceToggle();
};

// Command IDs specific to this application
constexpr uint32_t GPIO_TOGGLE_CMD_SET_INTERVAL   = 0x4001;
constexpr uint32_t GPIO_TOGGLE_CMD_FORCE_TOGGLE   = 0x4002;
constexpr uint32_t GPIO_TOGGLE_CMD_GET_INTERVAL   = 0x4003;

} // namespace Application