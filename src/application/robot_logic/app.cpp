#include "application/robot_logic/app.hpp"


namespace APP {


constexpr uint32_t APP::GpioToggleApp::EVENT_ERROR;
constexpr uint32_t APP::GpioToggleApp::EVENT_PIN_TOGGLED;


GpioToggleApp::GpioToggleApp()
    : current_state(AppState::Initializing),
      is_pin_high(false),
      last_toggle_time(0)
    {
        
}

GpioToggleApp::~GpioToggleApp() {
    // Ensure pin is set to a safe state on destruction
    if (gpio_interface) {
        gpio_interface->ResetPin(config.gpio_port, config.gpio_pin);
    }
}

GpioToggleApp& GpioToggleApp::GetGpioToggleApp() {
    // This will be created after main() starts, when heap is available
    static APP::GpioToggleApp instance;
    return instance;
}

AppState GpioToggleApp::GetState() const {
    return current_state;
}

Platform::Status GpioToggleApp::Init(void* config_ptr) {

    gpio_interface = &Platform::GPIO::GpioInterface::GetInstance();
    timing_service = &Middleware::SystemServices::SystemTiming::GetInstance();

    config = {};
    // Check if dependencies are available
    if (!gpio_interface || !timing_service) {
        current_state = AppState::Error;
        return Platform::Status::DEPENDENCY_NOT_INITIALIZED;
    }
    
    // Apply configuration if provided
    if (config_ptr != nullptr) {
        this->config = *static_cast<GpioToggleConfig*>(config_ptr);
    }
    
    // Configure the GPIO pin
    Platform::GPIO::GpioConfig pin_config;
    pin_config.port = config.gpio_port;
    pin_config.pin = config.gpio_pin;
    pin_config.mode = Platform::GPIO::Mode::Output;
    pin_config.outputType = Platform::GPIO::OutputType::PushPull;
    pin_config.pull = Platform::GPIO::Pull::None;
    pin_config.speed = Platform::GPIO::Speed::Low;
    
    Platform::Status status = gpio_interface->ConfigurePin(pin_config);
    if (status != Platform::Status::OK) {
        current_state = AppState::Error;
        return status;
    }
    
    // Set initial pin state
    is_pin_high = config.start_state;
    if (is_pin_high) {
        gpio_interface->SetPin(config.gpio_port, config.gpio_pin);
    } else {
        gpio_interface->ResetPin(config.gpio_port, config.gpio_pin);
    }
    
    // Update state
    current_state = AppState::Idle;
    
    return Platform::Status::OK;
}

Platform::Status GpioToggleApp::Start() {
    // Check if already running
    if (current_state == AppState::Running) {
        return Platform::Status::OK;
    }
    
    // Only start from idle state
    if (current_state != AppState::Idle) {
        return Platform::Status::INVALID_STATE;
    }
    
    // Record start time
    last_toggle_time = timing_service->GetMilliseconds();
    
    // Update state
    current_state = AppState::Running;
    
    return Platform::Status::OK;
}

Platform::Status GpioToggleApp::Stop() {
    // Check if not running
    if (current_state != AppState::Running) {
        return Platform::Status::OK;
    }
    
    // Update state
    current_state = AppState::Idle;
    
    return Platform::Status::OK;
}

Platform::Status GpioToggleApp::Process(void* params) {
    // Skip if not in running state
    if (current_state != AppState::Running) {
        return Platform::Status::OK;
    }
    
    // Get current time
    uint64_t current_time = timing_service->GetMilliseconds();
    
    // Check if it's time to toggle
    if (current_time - last_toggle_time >= config.toggle_interval_ms) {
        // Toggle the pin
        Platform::Status status;
        if (is_pin_high) {
            status = gpio_interface->ResetPin(config.gpio_port, config.gpio_pin);
        } else {
            status = gpio_interface->SetPin(config.gpio_port, config.gpio_pin);
        }
        
        if (status != Platform::Status::OK) {
            // Handle error
            current_state = AppState::Error;
            
            // Trigger error callback if registered
            auto it = callbacks.find(EVENT_ERROR);
            if (it != callbacks.end() && it->second.active) {
                it->second.callback(it->second.param);
            }
            
            return status;
        }
        
        // Update state
        is_pin_high = !is_pin_high;
        last_toggle_time = current_time;
        
        // Trigger toggle callback if registered
        auto it = callbacks.find(EVENT_PIN_TOGGLED);
        if (it != callbacks.end() && it->second.active) {
            it->second.callback(it->second.param);
        }
    }
    
    return Platform::Status::OK;
}

Platform::Status GpioToggleApp::HandleCommand(uint32_t cmd_id, void* params) {
    switch (cmd_id) {
        case APP_CMD_SET_MODE: {
            if (params == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            AppState new_state = *static_cast<AppState*>(params);
            
            if (new_state == AppState::Running) {
                return Start();
            } else if (new_state == AppState::Idle) {
                return Stop();
            } else {
                // Other state transitions not supported
                return Platform::Status::INVALID_PARAM;
            }
        }
        
        case APP_CMD_GET_MODE: {
            if (params == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            *static_cast<AppState*>(params) = current_state;
            return Platform::Status::OK;
        }
        
        case GPIO_TOGGLE_CMD_SET_INTERVAL: {
            if (params == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            uint32_t interval_ms = *static_cast<uint32_t*>(params);
            return SetToggleInterval(interval_ms);
        }
        
        case GPIO_TOGGLE_CMD_FORCE_TOGGLE: {
            return ForceToggle();
        }
        
        case GPIO_TOGGLE_CMD_GET_INTERVAL: {
            if (params == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            *static_cast<uint32_t*>(params) = config.toggle_interval_ms;
            return Platform::Status::OK;
        }
        
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
}

Platform::Status GpioToggleApp::GetStatus(void* status, uint32_t* size) {
    if (status == nullptr || size == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Define status structure
    struct GpioToggleStatus {
        AppState state;
        bool is_pin_high;
        uint32_t toggle_interval_ms;
        uint64_t last_toggle_time;
        uint64_t next_toggle_time;
    };
    
    // Check buffer size
    if (*size < sizeof(GpioToggleStatus)) {
        *size = sizeof(GpioToggleStatus);
        return Platform::Status::BUFFER_OVERFLOW;
    }
    
    // Fill status structure
    GpioToggleStatus* app_status = static_cast<GpioToggleStatus*>(status);
    app_status->state = current_state;
    app_status->is_pin_high = is_pin_high;
    app_status->toggle_interval_ms = config.toggle_interval_ms;
    app_status->last_toggle_time = last_toggle_time;
    
    // Calculate next toggle time
    if (current_state == AppState::Running) {
        app_status->next_toggle_time = last_toggle_time + config.toggle_interval_ms;
    } else {
        app_status->next_toggle_time = 0;
    }
    
    *size = sizeof(GpioToggleStatus);
    
    return Platform::Status::OK;
}

Platform::Status GpioToggleApp::RegisterCallback(uint32_t event, void (*callback)(void* param), void* param) {
    // Store callback
    CallbackEntry entry{callback, param, callback != nullptr};
    callbacks[event] = entry;
    
    return Platform::Status::OK;
}

Platform::Status GpioToggleApp::SetToggleInterval(uint32_t interval_ms) {
    // Validate interval
    if (interval_ms == 0) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Update configuration
    config.toggle_interval_ms = interval_ms;
    
    return Platform::Status::OK;
}

Platform::Status GpioToggleApp::ForceToggle() {
    // Toggle the pin regardless of timing
    Platform::Status status;
    if (is_pin_high) {
        status = gpio_interface->ResetPin(config.gpio_port, config.gpio_pin);
    } else {
        status = gpio_interface->SetPin(config.gpio_port, config.gpio_pin);
    }
    
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Update state
    is_pin_high = !is_pin_high;
    last_toggle_time = timing_service->GetMilliseconds();
    
    // Trigger toggle callback if registered
    auto it = callbacks.find(EVENT_PIN_TOGGLED);
    if (it != callbacks.end() && it->second.active) {
        it->second.callback(it->second.param);
    }
    
    return Platform::Status::OK;
}

} // namespace Application