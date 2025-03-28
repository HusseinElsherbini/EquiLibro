// In a new file: storage_manager.cpp
#include "storage_manager.hpp"
#include "middleware/system_services/system_timing.hpp"
#include <cstring>

namespace Middleware {
namespace Storage {

StorageManager& StorageManager::GetInstance() {
    static StorageManager instance;
    return instance;
}

StorageManager::StorageManager() 
    : initialized(false), 
      flash(Platform::FLASH::FlashInterface::GetInstance()) {
    // Constructor implementation
}

Platform::Status StorageManager::Init() {
    if (initialized) {
        return Platform::Status::OK;
    }
    
    // Get information about the calibration storage sector
    Platform::Status status = flash.GetSectorInfo(CALIBRATION_STORAGE_SECTOR, calibration_sector);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    initialized = true;
    return Platform::Status::OK;
}
__attribute__((optimize("O0"))) 
StorageStatus StorageManager::SaveCalibrationData(APP::CalibrationData& calibration_data) {
    if (!initialized) {
        return StorageStatus::NOT_INITIALIZED;
    }    
    
    // Get current timestamp
    Middleware::SystemServices::SystemTiming& timing = 
        Middleware::SystemServices::SystemTiming::GetInstance();


    // Create a storage wrapper structure
    Middleware::Storage::StoredCalibrationData calData = {
        .signature = Middleware::Storage::CALIBRATION_SIGNATURE,
        .version = Middleware::Storage::CALIBRATION_VERSION,
        .crc = 0,  
        .data = calibration_data,  
        .save_count = 1,  
        .timestamp = timing.GetMilliseconds()
    };
    // Prepare data structure for storage
    // Calculate CRC
    calData.crc = CalculateCRC32(&calibration_data, sizeof(APP::CalibrationData));
    
    // Get storage address
    volatile uint32_t storage_addr = GetCalibrationStorageAddress();
    
    // Unlock flash
    volatile Platform::Status status = flash.Unlock();
    if (status != Platform::Status::OK) {
        return StorageStatus::WRITE_ERROR;
    }
    
    // Erase sector before writing
    status = flash.EraseSector(CALIBRATION_STORAGE_SECTOR);
    if (status != Platform::Status::OK) {
        flash.Lock();
        return StorageStatus::ERASE_ERROR;
    }
    
    // Write data to flash
    status = flash.WriteData(storage_addr, &calData, sizeof(StoredCalibrationData));
    
    // Lock flash when done
    flash.Lock();
    
    if (status != Platform::Status::OK) {
        return StorageStatus::WRITE_ERROR;
    }
    
    return StorageStatus::OK;
}

StorageStatus StorageManager::LoadCalibrationData(StoredCalibrationData& calibration_data) {
    if (!initialized) {
        return StorageStatus::NOT_INITIALIZED;
    }
    
    // Get storage address
    uint32_t storage_addr = GetCalibrationStorageAddress();
    
    Platform::Status status = flash.ReadData(storage_addr, &calibration_data, sizeof(StoredCalibrationData));
    
    if (status != Platform::Status::OK) {
        return StorageStatus::READ_ERROR;
    }
    
    // Verify signature
    if (calibration_data.signature != CALIBRATION_SIGNATURE) {
        return StorageStatus::NO_DATA_FOUND;
    }
    
    // Verify version compatibility
    if (calibration_data.version != CALIBRATION_VERSION) {
        return StorageStatus::INVALID_DATA;
    }
    
    // Verify CRC
    uint32_t calculated_crc = CalculateCRC32(&calibration_data.data, sizeof(APP::CalibrationData));
    if (calculated_crc != calibration_data.crc) {
        return StorageStatus::INVALID_DATA;
    }
    
    
    return StorageStatus::OK;
}

bool StorageManager::HasValidCalibrationData(void) {
    if (!initialized) {
        return false;
    }
    
    // Get storage address
    uint32_t storage_addr = GetCalibrationStorageAddress();
    
    // Read just the header portion
    StoredCalibrationData stored_data;
    Platform::Status status = flash.ReadData(storage_addr, &stored_data, sizeof(StoredCalibrationData));
    
    if (status != Platform::Status::OK) {
        return false;
    }
    
    // Verify signature
    if (stored_data.signature != CALIBRATION_SIGNATURE) {
        return false;
    }
    
    // Verify version compatibility
    if (stored_data.version != CALIBRATION_VERSION) {
        return false;
    }
    
    // Verify CRC
    uint32_t calculated_crc = CalculateCRC32(&stored_data.data, sizeof(APP::CalibrationData));
    if (calculated_crc != stored_data.crc) {
        return false;
    }
    
    return true;
}

StorageStatus StorageManager::EraseCalibrationData() {
    if (!initialized) {
        return StorageStatus::NOT_INITIALIZED;
    }
    
    // Unlock flash
    Platform::Status status = flash.Unlock();
    if (status != Platform::Status::OK) {
        return StorageStatus::ERASE_ERROR;
    }
    
    // Erase sector
    status = flash.EraseSector(CALIBRATION_STORAGE_SECTOR);
    
    // Lock flash when done
    flash.Lock();
    
    if (status != Platform::Status::OK) {
        return StorageStatus::ERASE_ERROR;
    }
    
    return StorageStatus::OK;
}

uint32_t StorageManager::CalculateCRC32(const void* data, uint32_t size) {
    const uint8_t* buf = static_cast<const uint8_t*>(data);
    uint32_t crc = 0xFFFFFFFF;
    
    for (uint32_t i = 0; i < size; i++) {
        crc = (crc >> 8) ^ crc32_table[(crc & 0xFF) ^ buf[i]];
    }
    
    return crc ^ 0xFFFFFFFF;
}

uint32_t StorageManager::GetCalibrationStorageAddress() {
    return calibration_sector.start_address + CALIBRATION_STORAGE_OFFSET;
}

} // namespace Storage
}