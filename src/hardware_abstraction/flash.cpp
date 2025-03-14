// src/hardware_abstraction/flash.cpp

#include "hardware_abstraction/flash.hpp"
#include "common/platform_flash.hpp"
#include "middleware/system_services/system_timing.hpp"
#include "os/mutex.hpp"
#include <cstring>
namespace Platform {
namespace FLASH {

FlashInterface& GetInstance() {
    // Static instance of the interface, created on first use
    static FlashInterfaceImpl instance;
    return instance;
}
// Destructor
FlashInterfaceImpl::~FlashInterfaceImpl() {
    if (initialized) {
        // Lock flash on destruction for security
        Lock();
    }
}


// Initialize the flash interface
Platform::Status FlashInterfaceImpl::Init(void* config) {
    if (initialized) {
        return Platform::Status::OK; // Already initialized
    }
    
    // Use default configuration if none provided
    if (config == nullptr) {
        this->config.latency = FlashLatency::WS0;
        this->config.prefetch_enable = true;
        this->config.icache_enable = true;
        this->config.dcache_enable = true;
    } else {
        this->config = *static_cast<FlashConfig*>(config);
    }
    
    // Apply the configuration
    Platform::Status status = Configure(this->config);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Flash is locked by default after reset
    locked = true;
    
    initialized = true;
    return Platform::Status::OK;
}

// De-initialize the flash interface
Platform::Status FlashInterfaceImpl::DeInit() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Lock flash before de-initialization
    Lock();
    
    initialized = false;
    return Platform::Status::OK;
}

// Flash interface control function
Platform::Status FlashInterfaceImpl::Control(uint32_t command, void* param) {
    if (!initialized && command != FLASH_CTRL_CONFIGURE) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    switch (command) {
        case FLASH_CTRL_CONFIGURE: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            return Configure(*static_cast<FlashConfig*>(param));
        }
        
        case FLASH_CTRL_READ_DATA: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            struct ReadParams {
                uint32_t address;
                void* data;
                uint32_t size;
            };
            
            ReadParams* read_params = static_cast<ReadParams*>(param);
            return ReadData(read_params->address, read_params->data, read_params->size);
        }
        
        case FLASH_CTRL_WRITE_DATA: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            struct WriteParams {
                uint32_t address;
                const void* data;
                uint32_t size;
            };
            
            WriteParams* write_params = static_cast<WriteParams*>(param);
            return WriteData(write_params->address, write_params->data, write_params->size);
        }
        case FLASH_CTRL_FIND_SECTOR: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            FindSectorParams* find_params = static_cast<FindSectorParams*>(param);

            find_params->sector = FindSectorNumber(find_params->address);

            if(find_params->sector == 0xFF) {
                return Platform::Status::ERROR;
            }
            return Platform::Status::OK;
        }
        case FLASH_CTRL_ERASE_SECTOR: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            uint8_t sector = *static_cast<uint8_t*>(param);
            return EraseSector(sector);
        }
        
        case FLASH_CTRL_ERASE_BANK: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            uint8_t bank = *static_cast<uint8_t*>(param);
            return EraseBank(bank);
        }
        
        case FLASH_CTRL_MASS_ERASE: {
            return MassErase();
        }
        
        case FLASH_CTRL_GET_SECTOR_INFO: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            struct SectorInfoParams {
                uint8_t sector;
                FlashSector* sector_info;
            };
            
            SectorInfoParams* info_params = static_cast<SectorInfoParams*>(param);
            return GetSectorInfo(info_params->sector, *info_params->sector_info);
        }
        
        case FLASH_CTRL_GET_TOTAL_SIZE: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            *static_cast<uint32_t*>(param) = GetTotalSize();
            return Platform::Status::OK;
        }
        
        case FLASH_CTRL_GET_LATENCY: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            *static_cast<uint8_t*>(param) = GetLatency();
            return Platform::Status::OK;
        }
        
        case FLASH_CTRL_IS_BUSY: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            *static_cast<bool*>(param) = IsBusy();
            return Platform::Status::OK;
        }
        
        case FLASH_CTRL_IS_LOCKED: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            *static_cast<bool*>(param) = IsLocked();
            return Platform::Status::OK;
        }
        
        case FLASH_CTRL_LOCK: {
            return Lock();
        }
        
        case FLASH_CTRL_UNLOCK: {
            return Unlock();
        }
        
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
}

// Direct read access - maps to ReadData
Platform::Status FlashInterfaceImpl::Read(void* buffer, uint16_t size, uint32_t timeout) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (buffer == nullptr || size == 0) {
        return Platform::Status::INVALID_PARAM;
    }
    
    //TODO: Implement a timeout for flash reads
    (void)timeout;
    // For direct reads, use a default address in flash
    // This is generally not the preferred method - use ReadData with explicit address instead
    return ReadData(0x08000000, buffer, size);
}

// Direct write access - maps to WriteData
Platform::Status FlashInterfaceImpl::Write(const void* data, uint16_t size, uint32_t timeout) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (data == nullptr || size == 0) {
        return Platform::Status::INVALID_PARAM;
    }
    //TODO: Implement a timeout for flash reads
    (void)timeout;
    // For direct writes, use a default address in flash
    // This is generally not the preferred method - use WriteData with explicit address instead
    return WriteData(0x08000000, data, size);
}

// Register a callback for flash events
Platform::Status FlashInterfaceImpl::RegisterCallback(uint32_t event_id, void (*callback)(void* param), void* param) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (event_id >= static_cast<uint32_t>(FlashEvent::Max)) {
        return Platform::Status::INVALID_PARAM;
    }
    
    callbacks[event_id].callback = callback;
    callbacks[event_id].user_data = param;
    callbacks[event_id].active = (callback != nullptr);
    
    return Platform::Status::OK;
}

// Configure flash settings
Platform::Status FlashInterfaceImpl::Configure(const FlashConfig& config) {
    OS::lock_guard<OS::mutex> lock(flash_mutex);
    
    // Store the configuration
    this->config = config;
    
    // Get direct access to the flash registers
    volatile Platform::FLASH::Registers* flash_regs = Platform::FLASH::getRegisters();
    if (flash_regs == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Read current ACR value
    uint32_t reg_value = flash_regs->ACR;
    
    // Clear latency bits
    reg_value &= ~static_cast<uint32_t>(Platform::FLASH::ACR::LATENCY_MSK);
    
    // Set new latency
    reg_value |= (static_cast<uint32_t>(config.latency) & 0x07); // 3 bits of latency (0-7)
    
    // Configure prefetch
    if (config.prefetch_enable) {
        reg_value |= static_cast<uint32_t>(Platform::FLASH::ACR::PRFTEN);
    } else {
        reg_value &= ~static_cast<uint32_t>(Platform::FLASH::ACR::PRFTEN);
    }
    
    // Configure instruction cache
    if (config.icache_enable) {
        reg_value |= static_cast<uint32_t>(Platform::FLASH::ACR::ICEN);
    } else {
        reg_value &= ~static_cast<uint32_t>(Platform::FLASH::ACR::ICEN);
    }
    
    // Configure data cache
    if (config.dcache_enable) {
        reg_value |= static_cast<uint32_t>(Platform::FLASH::ACR::DCEN);
    } else {
        reg_value &= ~static_cast<uint32_t>(Platform::FLASH::ACR::DCEN);
    }
    
    // Write updated configuration
    flash_regs->ACR = reg_value;
    
    // Verify the configuration was applied
    if ((flash_regs->ACR & 0x07) != static_cast<uint32_t>(config.latency)) {
        return Platform::Status::ERROR;
    }
    
    return Platform::Status::OK;
}

// Read data from flash
Platform::Status FlashInterfaceImpl::ReadData(uint32_t address, void* data, uint32_t size) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (data == nullptr || size == 0) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Validate address range
    if (address < 0x08000000 || address + size > 0x08080000) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Flash reads are direct memory access
    std::memcpy(data, reinterpret_cast<const void*>(address), size);
    
    return Platform::Status::OK;
}

// Write data to flash
Platform::Status FlashInterfaceImpl::WriteData(uint32_t address, const void* data, uint32_t size) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (data == nullptr || size == 0) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Validate address range
    if (address < 0x08000000 || address + size > 0x08080000) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Check if flash is locked
    if (locked) {
        return Platform::Status::ERROR;
    }
    
    OS::lock_guard<OS::mutex> lock(flash_mutex);
    
    // Get FLASH registers
    volatile Platform::FLASH::Registers* flash_regs = Platform::FLASH::getRegisters();
    
    // Wait for any ongoing operations to complete
    if (WaitForOperation(5000) != Platform::Status::OK) {
        return Platform::Status::TIMEOUT;
    }
    
    // Set program mode in CR register
    uint32_t cr_value = flash_regs->CR;
    cr_value &= ~(0x03 << 8); // Clear PG, SER, MER bits
    cr_value |= (1 << 0);     // Set PG bit for programming
    flash_regs->CR = cr_value;
    
    // Program flash one byte/halfword/word at a time depending on alignment
    const uint8_t* src_ptr = static_cast<const uint8_t*>(data);
    Platform::Status status = Platform::Status::OK;
    
    for (uint32_t i = 0; i < size && status == Platform::Status::OK; ) {
        if ((address + i) % 4 == 0 && i + 4 <= size) {
            // Word-aligned write
            uint32_t word_value = *reinterpret_cast<const uint32_t*>(src_ptr + i);
            *reinterpret_cast<volatile uint32_t*>(address + i) = word_value;
            i += 4;
        } else if ((address + i) % 2 == 0 && i + 2 <= size) {
            // Half-word aligned write
            uint16_t halfword_value = *reinterpret_cast<const uint16_t*>(src_ptr + i);
            *reinterpret_cast<volatile uint16_t*>(address + i) = halfword_value;
            i += 2;
        } else {
            // Byte write
            *reinterpret_cast<volatile uint8_t*>(address + i) = src_ptr[i];
            i += 1;
        }
        
        // Wait for the operation to complete
        status = WaitForOperation(1000);
    }
    
    // Clear PG bit
    cr_value = flash_regs->CR;
    cr_value &= ~(1 << 0); // Clear PG bit
    flash_regs->CR = cr_value;
    
    // Trigger callback if registered
    if (status == Platform::Status::OK && callbacks[static_cast<uint32_t>(FlashEvent::OperationComplete)].active) {
        callbacks[static_cast<uint32_t>(FlashEvent::OperationComplete)].callback(
            callbacks[static_cast<uint32_t>(FlashEvent::OperationComplete)].user_data
        );
    } else if (status != Platform::Status::OK && callbacks[static_cast<uint32_t>(FlashEvent::OperationError)].active) {
        callbacks[static_cast<uint32_t>(FlashEvent::OperationError)].callback(
            callbacks[static_cast<uint32_t>(FlashEvent::OperationError)].user_data
        );
    }
    
    return status;
}

// Erase a flash sector
Platform::Status FlashInterfaceImpl::EraseSector(uint8_t sector) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Validate sector number
    if (sector >= 8) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Check if flash is locked
    if (locked) {
        return Platform::Status::ERROR;
    }
    
    OS::lock_guard<OS::mutex> lock(flash_mutex);
    
    // Get FLASH registers
    volatile Platform::FLASH::Registers* flash_regs = Platform::FLASH::getRegisters();
    
    // Wait for any ongoing operations to complete
    if (WaitForOperation(5000) != Platform::Status::OK) {
        return Platform::Status::TIMEOUT;
    }
    
    // Set sector erase mode in CR register
    uint32_t cr_value = flash_regs->CR;
    cr_value &= ~(0x07 << 0); // Clear PG, SER, MER bits
    cr_value |= (1 << 1);     // Set SER bit for sector erase
    
    // Set sector number (bits 3-6)
    cr_value &= ~(0x0F << 3); // Clear previous sector number
    cr_value |= (sector << 3); // Set new sector number
    
    flash_regs->CR = cr_value;
    
    // Start the erase operation
    cr_value |= (1 << 16); // Set STRT bit to start operation
    flash_regs->CR = cr_value;
    
    // Wait for the operation to complete
    Platform::Status status = WaitForOperation(30000); // Sector erase can take longer
    
    // Clear SER bit
    cr_value = flash_regs->CR;
    cr_value &= ~(1 << 1); // Clear SER bit
    flash_regs->CR = cr_value;
    
    // Trigger callback if registered
    if (status == Platform::Status::OK && callbacks[static_cast<uint32_t>(FlashEvent::OperationComplete)].active) {
        callbacks[static_cast<uint32_t>(FlashEvent::OperationComplete)].callback(
            callbacks[static_cast<uint32_t>(FlashEvent::OperationComplete)].user_data
        );
    } else if (status != Platform::Status::OK && callbacks[static_cast<uint32_t>(FlashEvent::OperationError)].active) {
        callbacks[static_cast<uint32_t>(FlashEvent::OperationError)].callback(
            callbacks[static_cast<uint32_t>(FlashEvent::OperationError)].user_data
        );
    }
    
    return status;
}

// Erase a flash bank
Platform::Status FlashInterfaceImpl::EraseBank(uint8_t bank) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Validate bank number (STM32F401 has only 1 bank)
    if (bank != 0) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // STM32F401 doesn't have a separate bank erase command
    // Use mass erase instead
    return MassErase();
}

// Erase all flash
Platform::Status FlashInterfaceImpl::MassErase() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Check if flash is locked
    if (locked) {
        return Platform::Status::ERROR;
    }
    
    OS::lock_guard<OS::mutex> lock(flash_mutex);
    
    // Get FLASH registers
    volatile Platform::FLASH::Registers* flash_regs = Platform::FLASH::getRegisters();
    
    // Wait for any ongoing operations to complete
    if (WaitForOperation(5000) != Platform::Status::OK) {
        return Platform::Status::TIMEOUT;
    }
    
    // Set mass erase mode in CR register
    uint32_t cr_value = flash_regs->CR;
    cr_value &= ~(0x07 << 0); // Clear PG, SER, MER bits
    cr_value |= (1 << 2);     // Set MER bit for mass erase
    
    flash_regs->CR = cr_value;
    
    // Start the erase operation
    cr_value |= (1 << 16); // Set STRT bit to start operation
    flash_regs->CR = cr_value;
    
    // Wait for the operation to complete
    Platform::Status status = WaitForOperation(60000); // Mass erase takes a while
    
    // Clear MER bit
    cr_value = flash_regs->CR;
    cr_value &= ~(1 << 2); // Clear MER bit
    flash_regs->CR = cr_value;
    
    // Trigger callback if registered
    if (status == Platform::Status::OK && callbacks[static_cast<uint32_t>(FlashEvent::OperationComplete)].active) {
        callbacks[static_cast<uint32_t>(FlashEvent::OperationComplete)].callback(
            callbacks[static_cast<uint32_t>(FlashEvent::OperationComplete)].user_data
        );
    } else if (status != Platform::Status::OK && callbacks[static_cast<uint32_t>(FlashEvent::OperationError)].active) {
        callbacks[static_cast<uint32_t>(FlashEvent::OperationError)].callback(
            callbacks[static_cast<uint32_t>(FlashEvent::OperationError)].user_data
        );
    }
    
    return status;
}

// Get information about a flash sector
Platform::Status FlashInterfaceImpl::GetSectorInfo(uint8_t sector, FlashSector& sector_info) const {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Validate sector number
    if (sector >= 8) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Fill in sector information from the mapping table
    sector_info.sector_number = sector_map[sector].number;
    sector_info.start_address = sector_map[sector].address;
    sector_info.size = sector_map[sector].size;
    
    return Platform::Status::OK;
}

// Get total flash size
uint32_t FlashInterfaceImpl::GetTotalSize() const {
    if (!initialized) {
        return 0;
    }
    
    // STM32F401 has 512KB flash
    return 512 * 1024;
}

// Get current flash latency
uint8_t FlashInterfaceImpl::GetLatency() const {
    if (!initialized) {
        return 0;
    }
    
    // Read latency from flash registers
    volatile Platform::FLASH::Registers* flash_regs = Platform::FLASH::getRegisters();
    return (flash_regs->ACR & 0x07);
}

// Check if flash is busy
bool FlashInterfaceImpl::IsBusy() const {
    if (!initialized) {
        return false;
    }
    
    // Read the BSY bit from the status register
    volatile Platform::FLASH::Registers* flash_regs = Platform::FLASH::getRegisters();
    return (flash_regs->SR & (1 << 16)) != 0;
}

// Check if flash is locked
bool FlashInterfaceImpl::IsLocked() const {
    return locked;
}

// Lock flash memory
Platform::Status FlashInterfaceImpl::Lock() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    OS::lock_guard<OS::mutex> lock(flash_mutex);
    
    // Set the lock bit in the control register
    volatile Platform::FLASH::Registers* flash_regs = Platform::FLASH::getRegisters();
    flash_regs->CR |= (1 << 31); // Set LOCK bit
    
    // Verify lock status
    if ((flash_regs->CR & (1 << 31)) == 0) {
        return Platform::Status::ERROR;
    }
    
    locked = true;
    return Platform::Status::OK;
}

// Unlock flash memory
Platform::Status FlashInterfaceImpl::Unlock() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    OS::lock_guard<OS::mutex> lock(flash_mutex);
    
    // Write unlock keys to key register
    volatile Platform::FLASH::Registers* flash_regs = Platform::FLASH::getRegisters();
    
    // Check if already unlocked
    if ((flash_regs->CR & (1 << 31)) == 0) {
        locked = false;
        return Platform::Status::OK;
    }
    
    // Write unlock sequence
    flash_regs->KEYR = FLASH_KEY1; // Key 1
    flash_regs->KEYR = FLASH_KEY2; // Key 2
    
    // Verify unlock status
    if ((flash_regs->CR & (1 << 31)) != 0) {
        return Platform::Status::ERROR;
    }
    
    locked = false;
    return Platform::Status::OK;
 }
 
 // Wait for flash operation to complete
 Platform::Status FlashInterfaceImpl::WaitForOperation(uint32_t timeout_ms) {
    // Get start time
    Middleware::SystemServices::SystemTiming &timing = Middleware::SystemServices::SystemTiming::GetInstance();

    uint64_t start_time = timing.GetMilliseconds();
    
    // Wait for BSY bit to clear or timeout
    volatile Platform::FLASH::Registers* flash_regs = Platform::FLASH::getRegisters();
    
    while ((flash_regs->SR & (1 << 16)) != 0) { // BSY bit is bit 16
        // Check for timeout
        if ((timing.GetMilliseconds() - start_time) > timeout_ms) {
            return Platform::Status::TIMEOUT;
        }
        
        // Check for error flags
        uint32_t error_flags = flash_regs->SR & ((1 << 2) | (1 << 4) | (1 << 5) | (1 << 7));
        if (error_flags != 0) {
            // Clear error flags
            flash_regs->SR = error_flags;
            
            // Trigger error callback if registered
            if (callbacks[static_cast<uint32_t>(FlashEvent::OperationError)].active) {
                callbacks[static_cast<uint32_t>(FlashEvent::OperationError)].callback(
                    callbacks[static_cast<uint32_t>(FlashEvent::OperationError)].user_data
                );
            }
            
            return Platform::Status::ERROR;
        }
        
        // Short delay to prevent tight polling
        timing.DelayMicroseconds(10);
    }
    
    return Platform::Status::OK;
 }
 
 // Find which sector contains a given address
 uint8_t FlashInterfaceImpl::FindSectorNumber(uint32_t address) const {
    // Check if address is in flash range
    if (address < Platform::FLASH::FLASH_BASE_ADDRESS || address >= Platform::FLASH::FLASH_END_ADDRESS) {
        return Platform::FLASH::INVALID_SECTOR; // Invalid sector
    }
    
    // Find the sector that contains this address
    for (uint8_t i = 0; i < 8; i++) {
        uint32_t sector_end = sector_map[i].address + sector_map[i].size;
        if (address >= sector_map[i].address && address < sector_end) {
            return i;
        }
    }
    
    return Platform::FLASH::INVALID_SECTOR; // Address not found in any sector
 }

} // namespace FLASH
} // namespace Platform