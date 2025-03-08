// inc/application/application_module.h

#ifndef APPLICATION_MODULE_H
#define APPLICATION_MODULE_H

#include "common/types.h"
#include "common/error_codes.h"

// Application states
typedef enum {
    APP_STATE_INITIALIZING,
    APP_STATE_IDLE,
    APP_STATE_RUNNING,
    APP_STATE_ERROR,
    APP_STATE_SELF_TEST,
    APP_STATE_CALIBRATION,
    APP_STATE_SHUTDOWN
} AppState_t;

/**
 * Application module interface that provides consistent API
 * for high-level robot logic, system states, and application tasks.
 */
typedef struct {
    // Application state
    AppState_t state;
    
    // Module-specific state data
    void *app_state;
    
    // Initialize the application module
    Status_t (*Init)(void *config);
    
    // Start the module operation
    Status_t (*Start)(void);
    
    // Stop the module operation
    Status_t (*Stop)(void);
    
    // Main processing function
    Status_t (*Process)(void *params);
    
    // Handle a command
    Status_t (*HandleCommand)(uint32_t cmd_id, void *params);
    
    // Get module status
    Status_t (*GetStatus)(void *status, uint32_t *size);
    
    // Register callback for application events
    Status_t (*RegisterCallback)(uint32_t event, void (*callback)(void *param), void *param);
    
} ApplicationModule_t;

// Common application commands
#define APP_CMD_SET_MODE               0x3001
#define APP_CMD_GET_MODE               0x3002
#define APP_CMD_START_CALIBRATION      0x3003
#define APP_CMD_START_SELF_TEST        0x3004
#define APP_CMD_SET_PARAMETER          0x3005
#define APP_CMD_GET_PARAMETER          0x3006
#define APP_CMD_SAVE_CONFIGURATION     0x3007
#define APP_CMD_LOAD_CONFIGURATION     0x3008

#endif /* APPLICATION_MODULE_H */