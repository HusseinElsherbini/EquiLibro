#pragma once 

#include "hw_interface.hpp"
#include "common/platform.hpp"
#include "common/platform_flash.hpp"
#include <memory>

namespace Platform {
namespace FLASH {

/**
 * @brief Flash memory configuration structure
 * 
 * Contains all parameters needed to configure flash memory access
 */
struct FlashConfig {
    FlashLatency latency;           // Flash latency (wait states)
    bool prefetch_enable;      // Enable prefetch buffer
    bool icache_enable;        // Enable instruction cache
    bool dcache_enable;        // Enable data cache
};

/**
 * @brief Flash operation mode for programming/erase
 */
enum class FlashOperationMode {
    ProgramWord,       // Program a 32-bit word
    ProgramHalfWord,   // Program a 16-bit half-word
    ProgramByte,       // Program an 8-bit byte
    EraseSector,       // Erase a flash sector
    EraseBank,         // Erase an entire flash bank
    MassErase          // Erase all flash memory
};

enum class FlashLatency {
    WS0 = 0,    // Zero wait states
    WS1 = 1,    // One wait state
    WS2 = 2,    // Two wait states
    WS3 = 3,    // Three wait states
    WS4 = 4,    // Four wait states
    WS5 = 5,    // Five wait states
    AUTOCALCULATE = 0xFF // Special value for auto-calculate
};

/**
 * @brief Flash sector information structure
 */
struct FlashSector {
    uint8_t sector_number;     // Sector identifier
    uint32_t start_address;    // Start address of sector
    uint32_t size;             // Size of sector in bytes
};

/**
 * @brief Flash callback event types
 */
enum class FlashEvent {
    OperationComplete,     // Flash operation completed
    OperationError,        // Error occurred during flash operation
    Max                    // Must be last - used for array sizing
};

/**
 * @brief Flash interface providing hardware abstraction for flash memory
 * 
 * This class provides a consistent interface for flash memory operations
 * including configuration, read, write, and erase functions.
 */
class FlashInterface : public HwInterface {
public:
    // Get singleton instance
    static FlashInterface& GetInstance();
    
    // Flash-specific methods
    virtual Platform::Status Configure(const FlashConfig& config) = 0;
    virtual Platform::Status ReadData(uint32_t address, void* data, uint32_t size) = 0;
    virtual Platform::Status WriteData(uint32_t address, const void* data, uint32_t size) = 0;
    virtual Platform::Status EraseSector(uint8_t sector) = 0;
    virtual Platform::Status EraseBank(uint8_t bank) = 0;
    virtual Platform::Status MassErase() = 0;
    virtual Platform::Status GetSectorInfo(uint8_t sector, FlashSector& sector_info) const = 0;
    virtual uint32_t GetTotalSize() const = 0;
    virtual uint8_t GetLatency() const = 0;
    virtual bool IsBusy() const = 0;
    virtual bool IsLocked() const = 0;
    virtual Platform::Status Lock() = 0;
    virtual Platform::Status Unlock() = 0;
};

// Flash control command identifiers
constexpr uint32_t FLASH_CTRL_CONFIGURE = 0x0701;
constexpr uint32_t FLASH_CTRL_READ_DATA = 0x0702;
constexpr uint32_t FLASH_CTRL_WRITE_DATA = 0x0703;
constexpr uint32_t FLASH_CTRL_ERASE_SECTOR = 0x0704;
constexpr uint32_t FLASH_CTRL_ERASE_BANK = 0x0705;
constexpr uint32_t FLASH_CTRL_MASS_ERASE = 0x0706;
constexpr uint32_t FLASH_CTRL_GET_SECTOR_INFO = 0x0707;
constexpr uint32_t FLASH_CTRL_GET_TOTAL_SIZE = 0x0708;
constexpr uint32_t FLASH_CTRL_GET_LATENCY = 0x0709;
constexpr uint32_t FLASH_CTRL_IS_BUSY = 0x070A;
constexpr uint32_t FLASH_CTRL_IS_LOCKED = 0x070B;
constexpr uint32_t FLASH_CTRL_LOCK = 0x070C;
constexpr uint32_t FLASH_CTRL_UNLOCK = 0x070D;

/**
 * @brief Private implementation of FlashInterface
 * 
 * This class contains the actual implementation of the FlashInterface
 * for STM32F4 microcontrollers.
 */
class FlashInterfaceImpl : public Platform::FLASH::FlashInterface {
    private:
        // Internal state tracking
        bool initialized;
        FlashConfig config;
        bool locked;
        
        // Callback table for flash events
        struct CallbackEntry {
            void (*callback)(void* param);
            void* user_data;
            bool active;
        };
        
        CallbackEntry callbacks[static_cast<size_t>(FlashEvent::Max)];
        
        // Flash sector mapping for STM32F401
        struct {
            uint8_t number;
            uint32_t address;
            uint32_t size;
        } sector_map[8] = {
            {0, 0x08000000, 16 * 1024},    // 16KB
            {1, 0x08004000, 16 * 1024},    // 16KB
            {2, 0x08008000, 16 * 1024},    // 16KB
            {3, 0x0800C000, 16 * 1024},    // 16KB
            {4, 0x08010000, 64 * 1024},    // 64KB
            {5, 0x08020000, 128 * 1024},   // 128KB
            {6, 0x08040000, 128 * 1024},   // 128KB
            {7, 0x08060000, 128 * 1024}    // 128KB
        };
        
        // Mutex for thread safety
        std::mutex flash_mutex;
        
        // Helper methods
        Platform::Status WaitForOperation(uint32_t timeout_ms);
        Platform::Status StartOperation(FlashOperationMode mode, uint32_t address, const void* data = nullptr);
        uint8_t FindSectorNumber(uint32_t address) const;
        
    public:
        // Constructor/Destructor
        FlashInterfaceImpl();
        ~FlashInterfaceImpl();
        
        // HwInterface implementation
        Platform::Status Init(void* config) override;
        Platform::Status DeInit() override;
        Platform::Status Control(uint32_t command, void* param) override;
        Platform::Status Read(void* buffer, uint16_t size, uint32_t timeout) override;
        Platform::Status Write(const void* data, uint16_t size, uint32_t timeout) override;
        Platform::Status RegisterCallback(uint32_t event_id, void (*callback)(void* param), void* param) override;
        
        // FlashInterface implementation
        Platform::Status Configure(const FlashConfig& config) override;
        Platform::Status ReadData(uint32_t address, void* data, uint32_t size) override;
        Platform::Status WriteData(uint32_t address, const void* data, uint32_t size) override;
        Platform::Status EraseSector(uint8_t sector) override;
        Platform::Status EraseBank(uint8_t bank) override;
        Platform::Status MassErase() override;
        Platform::Status GetSectorInfo(uint8_t sector, FlashSector& sector_info) const override;
        uint32_t GetTotalSize() const override;
        uint8_t GetLatency() const override;
        bool IsBusy() const override;
        bool IsLocked() const override;
        Platform::Status Lock() override;
        Platform::Status Unlock() override;
};
}
}

