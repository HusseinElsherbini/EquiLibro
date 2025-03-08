// inc/middleware/middleware_module.h

#ifndef MIDDLEWARE_MODULE_H
#define MIDDLEWARE_MODULE_H

#include "common/types.h"
#include "common/error_codes.h"

/**
 * Middleware module interface that provides consistent API
 * for services like signal processing, OS abstractions, and system services.
 */
typedef struct {
    // Module state
    void *state;
    
    // Initialize the middleware module
    Status_t (*Init)(void *config);
    
    // Process function - specific to the middleware module purpose
    Status_t (*Process)(void *input, void *output);
    
    // Configure module parameters
    Status_t (*Configure)(uint32_t param_id, void *value);
    
    // Get information from the module
    Status_t (*GetInfo)(uint32_t info_id, void *buffer, uint32_t *size);
    
    // Reset the module state
    Status_t (*Reset)(void);
    
    // Register callback for module events
    Status_t (*RegisterCallback)(uint32_t event, void (*callback)(void *param), void *param);
    
} MiddlewareModule_t;

// Common middleware control commands
#define MIDDLEWARE_PARAM_RESET_STATE        0x2001
#define MIDDLEWARE_PARAM_UPDATE_RATE        0x2002
#define MIDDLEWARE_PARAM_FILTER_CONSTANTS   0x2003
#define MIDDLEWARE_INFO_VERSION             0x2101
#define MIDDLEWARE_INFO_CAPABILITIES        0x2102
#define MIDDLEWARE_INFO_STATISTICS          0x2103

#endif /* MIDDLEWARE_MODULE_H */