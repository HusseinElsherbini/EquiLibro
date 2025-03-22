#pragma once

#include "common/platform.hpp"
#include <array>
#include <atomic>
#include <cassert>
#include <functional>

namespace Middleware {
namespace SystemServices {
    namespace ERROR {        
        
        constexpr uint8_t MAX_ERROR_HANDLERS = 5;
        constexpr size_t ERROR_HISTORY_SIZE = 32;  // Adjust based on memory constraints
        
        enum class ErrorVerbosityLevel {
            Debug,      // All errors with full context
            Normal,     // Important errors with context
            Critical,   // Only critical system errors
            None        // Logging disabled
        };
        struct ErrorInfo {
            Platform::Status status_code;    // Basic error code
            uint32_t module_id;              // Identifies the subsystem (e.g., GPIO=1, I2C=2)
            uint32_t error_id;               // Module-specific error identifier
            const char* location;            // File/function where error occurred
            uint32_t line;                   // Line number
            uint64_t timestamp;              // When the error occurred
            uint32_t additional_info;        // Error-specific context data
        };
        class IErrorHandler {
            public:
                virtual ~IErrorHandler() = default;
                
                virtual void HandleError(const ErrorInfo& error) = 0;
                
                // Filter to determine if this handler should process the error
                virtual bool ShouldHandleError(const ErrorInfo& error) = 0;
            };
        
        class ErrorManager {
            private:
                static std::array<ErrorInfo, ERROR_HISTORY_SIZE> error_history;
                static std::atomic<size_t> error_index;
                static std::array<IErrorHandler*, MAX_ERROR_HANDLERS> error_handlers;
                static std::atomic<size_t> handler_count;
                static ErrorVerbosityLevel verbosity_level;
                
                // Non-volatile storage for preserving errors across resets
                static constexpr uint32_t ERROR_LOG_SENTINEL = 0xE44C06D0;  // "ERRLOG"
                static constexpr uint32_t ERROR_LOG_NV_ADDRESS = 0x0800F000; // Flash address for error log
                
            public:

                
                // Core logging function
                static void LogError(const ErrorInfo& error);
                
                // Get current timestamp
                static uint64_t GetTimestamp();
                
                // Register handler for different error processing
                static void RegisterErrorHandler(IErrorHandler* handler);
                
                // Set verbosity level
                static void SetVerbosityLevel(ErrorVerbosityLevel level);
                
                // Retrieve recent errors
                static size_t GetErrorHistory(ErrorInfo* buffer, size_t max_errors);
                
                // Save errors to non-volatile storage
                static void SaveErrorsToNonvolatileStorage();
                
                // Restore errors after reset
                static void RestoreErrorsFromNonvolatileStorage();
                
                // Clear error history
                static void ClearErrorHistory();
            };

        /**
         * @brief Error status enumeration
         * 
         * This enumeration defines the basic error status codes that can be returned
         * by functions in the system. Each status code has a unique value that can be
         * used to identify the type of error that occurred.
         */


                // Module IDs
        constexpr uint32_t MODULE_HAL = 0x1000;
        constexpr uint32_t MODULE_MIDDLEWARE = 0x2000;
        constexpr uint32_t MODULE_DRIVER = 0x3000;
        constexpr uint32_t MODULE_APPLICATION = 0x4000;
        
        // Specific module sub-ranges
        constexpr uint32_t MODULE_GPIO = MODULE_HAL | 0x0100;
        constexpr uint32_t MODULE_I2C = MODULE_HAL | 0x0200;
        constexpr uint32_t MODULE_FLASH = MODULE_HAL | 0x0300;
        constexpr uint32_t MODULE_PWR = MODULE_HAL | 0x0400;
        constexpr uint32_t MODULE_SYSTICK = MODULE_HAL | 0x0500;
        constexpr uint32_t MODULE_TIMER  = MODULE_HAL | 0x0600;
        constexpr uint32_t MODULE_ADC = MODULE_HAL | 0x0700;
        constexpr uint32_t MODULE_DAC = MODULE_HAL | 0x0800;
        constexpr uint32_t MODULE_UART = MODULE_HAL | 0x0900;
        constexpr uint32_t MODULE_SPI = MODULE_HAL | 0x0A00;
        constexpr uint32_t MODULE_RTC = MODULE_HAL | 0x0B00;
        constexpr uint32_t MODULE_WDT = MODULE_HAL | 0x0C00;
        constexpr uint32_t MODULE_USB = MODULE_HAL | 0x0D00;
        constexpr uint32_t MODULE_ETH = MODULE_HAL | 0x0E00;
        constexpr uint32_t MODULE_CAN = MODULE_HAL | 0x0F00;
        constexpr uint32_t MODULE_SDIO = MODULE_HAL | 0x1000;


        constexpr uint32_t MODULE_SYSTEM = MODULE_MIDDLEWARE | 0x0100;
        constexpr uint32_t MODULE_SYSTEM_INIT = MODULE_MIDDLEWARE | 0x0200;
        constexpr uint32_t MODULE_SYSTEM_TIMING = MODULE_MIDDLEWARE | 0x0300;
        constexpr uint32_t MODULE_SYSTEM_MANAGER = MODULE_MIDDLEWARE | 0x0400;
        constexpr uint32_t MODULE_DEVICE_DRIVER = MODULE_DRIVER | 0x0100;
        constexpr uint32_t MODULE_DEVICE_MANAGER = MODULE_DRIVER | 0x0200;
        constexpr uint32_t MODULE_APPLICATION_MAIN = MODULE_APPLICATION | 0x0100;

        
        // Common error patterns within modules
        constexpr uint32_t ERR_INITIALIZATION = 0x0001;
        constexpr uint32_t ERR_TIMEOUT = 0x0002;
        constexpr uint32_t ERR_INVALID_PARAM = 0x0003;
        constexpr uint32_t ERR_HARDWARE_FAILURE = 0x0004;
        constexpr uint32_t ERR_BUSY = 0x0005;
        
        // Combined error codes
        constexpr uint32_t GPIO_INITIALIZATION_FAILED = MODULE_GPIO | ERR_INITIALIZATION;
        constexpr uint32_t GPIO_TIMEOUT = MODULE_GPIO | ERR_TIMEOUT;
        constexpr uint32_t GPIO_INVALID_PIN = MODULE_GPIO | ERR_INVALID_PARAM;
        constexpr uint32_t GPIO_HARDWARE_FAILURE = MODULE_GPIO | ERR_HARDWARE_FAILURE;
        constexpr uint32_t GPIO_BUSY = MODULE_GPIO | ERR_BUSY;

        constexpr uint32_t I2C_TIMEOUT = MODULE_I2C | ERR_TIMEOUT;
        
        // Conversion to Status values
        Platform::Status ToStatus(uint32_t error_code);
        uint32_t FromStatus(Platform::Status status);
        
        // Error descriptions
        const char* GetErrorDescription(uint32_t error_code);

        inline void ERROR_CONTEXT(Platform::Status status) {
            ErrorInfo{ 
                status, 
                0, 
                0,
                __FILE__, 
                __LINE__, 
                ErrorManager::GetTimestamp(), 
                0 
            };
        }

        inline void ERROR_LOG(uint32_t module_id, uint32_t error_id, Platform::Status status) {
            ErrorManager::LogError({
                status,
                module_id,
                error_id,
                __FILE__,
                __LINE__,
                ErrorManager::GetTimestamp(),
                0
            });
        }

        inline void ERROR_LOG_WITH_INFO(uint32_t module_id, uint32_t error_id, Platform::Status status) {
            ErrorManager::LogError({
                status,
                module_id,
                error_id,
                __FILE__,
                __LINE__,
                ErrorManager::GetTimestamp(),
                0
            });
        }
        #define RETURN_IF_ERROR(expr) \
            do { \
                Platform::Status status = (expr); \
                if (status != Platform::Status::OK) { \
                    ERROR_LOG_WITH_INFO(MODULE_ID, ErrorRegistry::FromStatus(status), status, 0); \
                    return status; \
                } \
            } while(0)
        
        template<typename T>
        class Result {
        private:
            union {
                T value;
                ErrorInfo error;
            };
            bool is_error;
            
            // Constructor helpers
            void construct_value(const T& v) {
                new (&value) T(v);
                is_error = false;
            }
            
            void construct_error(const ErrorInfo& e) {
                new (&error) ErrorInfo(e);
                is_error = true;
            }
            
            void destroy() {
                if (is_error) {
                    error.~ErrorInfo();
                } else {
                    value.~T();
                }
            }
            
        public:
            // Constructors
            Result(const T& v) { construct_value(v); }
            Result(const ErrorInfo& e) { construct_error(e); }
            
            // Copy constructor
            Result(const Result& other) {
                if (other.is_error) {
                    construct_error(other.error);
                } else {
                    construct_value(other.value);
                }
            }
            
            // Move constructor
            Result(Result&& other) noexcept {
                if (other.is_error) {
                    construct_error(std::move(other.error));
                } else {
                    construct_value(std::move(other.value));
                }
            }
            
            // Destructor
            ~Result() {
                destroy();
            }
            
            // Assignment operators
            Result& operator=(const Result& other) {
                if (this != &other) {
                    destroy();
                    if (other.is_error) {
                        construct_error(other.error);
                    } else {
                        construct_value(other.value);
                    }
                }
                return *this;
            }
            
            Result& operator=(Result&& other) noexcept {
                if (this != &other) {
                    destroy();
                    if (other.is_error) {
                        construct_error(std::move(other.error));
                    } else {
                        construct_value(std::move(other.value));
                    }
                }
                return *this;
            }
            
            // Error checking
            bool isError() const { return is_error; }
            
            // Value access
            const T& getValue() const {
                if (is_error) {
                    // Trap in debug builds
                    #ifdef DEBUG
                    assert(!is_error && "Accessing value when result contains an error");
                    #endif
                    // For release, provide a valid but default value
                    static T default_value = T{};
                    return default_value;
                }
                return value;
            }
            
            T& getValue() {
                if (is_error) {
                    #ifdef DEBUG
                    assert(!is_error && "Accessing value when result contains an error");
                    #endif
                    static T default_value = T{};
                    return default_value;
                }
                return value;
            }
            
            // Error access
            const ErrorInfo& getError() const {
                if (!is_error) {
                    static ErrorInfo default_error = {Platform::Status::OK};
                    return default_error;
                }
                return error;
            }
            
            // Boolean conversion for easier checking
            explicit operator bool() const { return !is_error; }
            
            // Helper for propagating errors
            template<typename U>
            Result<U> mapValue(std::function<U(const T&)> f) const {
                if (is_error) {
                    return Result<U>(error);
                }
                return Result<U>(f(value));
            }
            
            // Helper for handling errors
            Result<T> orElse(std::function<Result<T>(const ErrorInfo&)> f) const {
                if (is_error) {
                    return f(error);
                }
                return *this;
            }
        };


}
}
}