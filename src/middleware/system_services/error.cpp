#include "system_services/error.hpp"
#include "system_services/system_timing.hpp"
#include "system_services/system_manager.hpp"

namespace Middleware {
namespace SystemServices {
    namespace ERROR {

// Static member initialization
std::array<ErrorInfo, ERROR_HISTORY_SIZE> ErrorManager::error_history;
std::atomic<size_t> ErrorManager::error_index(0);
std::array<IErrorHandler*, MAX_ERROR_HANDLERS> ErrorManager::error_handlers;
std::atomic<size_t> ErrorManager::handler_count(0);
ErrorVerbosityLevel ErrorManager::verbosity_level = ErrorVerbosityLevel::Normal;

void ErrorManager::LogError(const ErrorInfo& error) {
    // Only log based on verbosity level
    switch (verbosity_level) {
        case ErrorVerbosityLevel::None:
            return;
        case ErrorVerbosityLevel::Critical:
            if (error.status_code != Platform::Status::HARDWARE_ERROR &&
                error.status_code != Platform::Status::COMMUNICATION_ERROR) {
                return;
            }
            break;
        case ErrorVerbosityLevel::Normal:
            if (error.status_code == Platform::Status::OK) {
                return;
            }
            break;
        case ErrorVerbosityLevel::Debug:
            // Log everything in debug mode
            break;
    }
    
    // Store in circular buffer
    size_t index = error_index.fetch_add(1) % ERROR_HISTORY_SIZE;
    error_history[index] = error;
    
    // Notify all registered handlers
    for (size_t i = 0; i < handler_count; ++i) {
        IErrorHandler* handler = error_handlers[i];
        if (handler && handler->ShouldHandleError(error)) {
            handler->HandleError(error);
        }
    }
}

uint64_t ErrorManager::GetTimestamp() {
    // Get timestamp from system timing service
    return Middleware::SystemServices::SystemTiming::GetInstance().GetMilliseconds();
}

void ErrorManager::RegisterErrorHandler(IErrorHandler* handler) {
    if (!handler) return;
    
    size_t index = handler_count.fetch_add(1);
    if (index < MAX_ERROR_HANDLERS) {
        error_handlers[index] = handler;
    } else {
        // Too many handlers, roll back the counter
        handler_count.fetch_sub(1);
    }
}

void ErrorManager::SaveErrorsToNonvolatileStorage() {
    // Check if we have a flash interface
    Platform::FLASH::FlashInterface* flash = &Platform::FLASH::FlashInterface::GetInstance();

    // Prepare data structure for saving
    struct ErrorLogHeader {
        uint32_t sentinel;
        uint32_t count;
        uint32_t checksum;
    };
    
    ErrorLogHeader header;
    header.sentinel = ERROR_LOG_SENTINEL;
    header.count = std::min(static_cast<size_t>(error_index.load()), ERROR_HISTORY_SIZE);
    
    // Calculate a simple checksum
    header.checksum = 0;
    for (size_t i = 0; i < header.count; i++) {
        size_t idx = (error_index.load() - i - 1) % ERROR_HISTORY_SIZE;
        header.checksum += error_history[idx].module_id ^ error_history[idx].error_id;
    }
    
    // Write header and errors to flash
    // This is a simplified example - proper flash handling would require more care
    flash->Unlock();

    Platform::FLASH::FindSectorParams params = {.address = ERROR_LOG_NV_ADDRESS};
    Platform::Status status = flash->Control(Platform::FLASH::FLASH_CTRL_FIND_SECTOR, &params);
    if (status != Platform::Status::OK) {
        flash->Lock();
        return;
    }
    flash->EraseSector(params.sector);
    
    // Write header
    flash->WriteData(ERROR_LOG_NV_ADDRESS, &header, sizeof(header));
    
    // Write error records
    uint32_t addr = ERROR_LOG_NV_ADDRESS + sizeof(header);
    for (size_t i = 0; i < header.count; i++) {
        size_t idx = (error_index.load() - i - 1) % ERROR_HISTORY_SIZE;
        flash->WriteData(addr, &error_history[idx], sizeof(ErrorInfo));
        addr += sizeof(ErrorInfo);
    }
    
    flash->Lock();
}

void ErrorManager::RestoreErrorsFromNonvolatileStorage() {
    // Check if we have a flash interface
    Platform::FLASH::FlashInterface* flash = &Platform::FLASH::FlashInterface::GetInstance();
    if (!flash) return;
    
    // Define header structure
    struct ErrorLogHeader {
        uint32_t sentinel;
        uint32_t count;
        uint32_t checksum;
    };
    
    // Read header
    ErrorLogHeader header;
    flash->ReadData(ERROR_LOG_NV_ADDRESS, &header, sizeof(header));
    
    // Validate header
    if (header.sentinel != ERROR_LOG_SENTINEL || header.count > ERROR_HISTORY_SIZE) {
        return; // Invalid header
    }
    
    // Clear current errors
    ClearErrorHistory();
    
    // Read error records
    uint32_t addr = ERROR_LOG_NV_ADDRESS + sizeof(header);
    for (size_t i = 0; i < header.count; i++) {
        ErrorInfo error;
        flash->ReadData(addr, &error, sizeof(ErrorInfo));
        
        // Add to current error history
        size_t index = error_index.fetch_add(1) % ERROR_HISTORY_SIZE;
        error_history[index] = error;
        
        addr += sizeof(ErrorInfo);
    }
    
    // Validate checksum
    uint32_t calculated_checksum = 0;
    for (size_t i = 0; i < header.count; i++) {
        size_t idx = (error_index.load() - i - 1) % ERROR_HISTORY_SIZE;
        calculated_checksum += error_history[idx].module_id ^ error_history[idx].error_id;
    }
    
    if (calculated_checksum != header.checksum) {
        // Checksum mismatch, data might be corrupted
        ClearErrorHistory();
    }
}

void ErrorManager::ClearErrorHistory() {
    error_index.store(0);
    for (auto& error : error_history) {
        error = ErrorInfo{Platform::Status::OK};
    }
}
}
}
}