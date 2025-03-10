#pragma once 

#include "common/platform.hpp"

/**
 * Middleware module interface that provides consistent API
 * for services like signal processing, OS abstractions, and system services.
 */

class MiddlewareModule {

public:
    
    virtual ~MiddlewareModule() = default;

    virtual Platform::Status Init(void *config) = 0;
    virtual Platform::Status Process(void *input, void *output) = 0;
    virtual Platform::Status Configure(uint32_t param_id, void *value) = 0;
    virtual Platform::Status GetInfo(uint32_t info_id, void *buffer, uint32_t *size) = 0;
    virtual Platform::Status Reset(void) = 0;
    virtual Platform::Status RegisterCallback(uint32_t event, void (*callback)(void *param), void *param) = 0;
};

// Common middleware control commands
#define MIDDLEWARE_PARAM_RESET_STATE        0x2001
#define MIDDLEWARE_PARAM_UPDATE_RATE        0x2002
#define MIDDLEWARE_PARAM_FILTER_CONSTANTS   0x2003
#define MIDDLEWARE_INFO_VERSION             0x2101
#define MIDDLEWARE_INFO_CAPABILITIES        0x2102
#define MIDDLEWARE_INFO_STATISTICS          0x2103
