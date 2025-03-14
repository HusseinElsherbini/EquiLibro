#pragma once

#include "common/platform.hpp"

namespace APP {

/**
 * Application states
 */
enum class AppState {
    Initializing,
    Idle,
    Running,
    Error,
    SelfTest,
    Calibration,
    Shutdown
};

/**
 * Application module interface that provides consistent API
 * for high-level robot logic, system states, and application tasks.
 */
class ApplicationModule {
public:
    // Virtual destructor ensures proper cleanup in derived classes
    virtual ~ApplicationModule() = default;
    
    // Get current application state
    virtual AppState GetState() const = 0;
    
    // Initialize the application module
    virtual Platform::Status Init(void* config) = 0;
    
    // Start the module operation
    virtual Platform::Status Start() = 0;
    
    // Stop the module operation
    virtual Platform::Status Stop() = 0;
    
    // Main processing function
    virtual Platform::Status Process(void* params) = 0;
    
    // Handle a command
    virtual Platform::Status HandleCommand(uint32_t cmd_id, void* params) = 0;
    
    // Get module status
    virtual Platform::Status GetStatus(void* status, uint32_t* size) = 0;
    
    // Register callback for application events
    virtual Platform::Status RegisterCallback(uint32_t event, void (*callback)(void* param), void* param) = 0;
};

// Common application commands
constexpr uint32_t APP_CMD_SET_MODE               = 0x3001;
constexpr uint32_t APP_CMD_GET_MODE               = 0x3002;
constexpr uint32_t APP_CMD_START_CALIBRATION      = 0x3003;
constexpr uint32_t APP_CMD_START_SELF_TEST        = 0x3004;
constexpr uint32_t APP_CMD_SET_PARAMETER          = 0x3005;
constexpr uint32_t APP_CMD_GET_PARAMETER          = 0x3006;
constexpr uint32_t APP_CMD_SAVE_CONFIGURATION     = 0x3007;
constexpr uint32_t APP_CMD_LOAD_CONFIGURATION     = 0x3008;

} // namespace Application
