// inc/hardware_abstraction/i2c.hpp

#pragma once

#include "hw_interface.hpp"
#include "common/platform.hpp"
#include "common/platform_i2c.hpp"
#include <mutex>
#include <vector>
#include <memory>

/**
 * I2C transfer request structure
 * Contains all parameters needed for an I2C transfer operation
 */
struct I2CTransferRequest {
    uint16_t device_address;     // Target device address (7 or 10 bit)
    uint16_t memory_address;     // Register/memory address within device
    uint8_t memory_address_size; // Size of memory address (1 or 2 bytes)
    uint8_t* data;               // Data buffer for read/write
    uint16_t data_size;          // Size of data buffer
    bool is_read;                // true = read operation, false = write operation
    uint32_t timeout_ms;         // Timeout in milliseconds
    bool use_dma;                // Use DMA for transfer if available
};

/**
 * I2C configuration structure
 */
struct I2CConfig {
    uint8_t i2c_instance;                    // I2C peripheral instance (1-3)
    Platform::I2C::Mode mode;                // Master or slave mode
    Platform::I2C::Speed speed;              // Standard (100kHz) or Fast (400kHz)
    Platform::I2C::AddrMode addressing_mode; // 7-bit or 10-bit addressing
    uint16_t own_address;                    // Own address in slave mode
    bool enable_dma;                         // Enable DMA for transfers
    bool enable_interrupts;                  // Enable interrupt-driven transfers
    bool analog_filter;                      // Enable analog noise filter
    uint8_t digital_filter;                  // Digital filter (0-15, where 0 = off)
    bool stretch_clock;                      // Enable clock stretching (usually true)
};

/**
 * I2C event types for callback registration
 */
enum class I2CEvent {
    TransferComplete,    // Transfer completed successfully
    TransferError,       // Error occurred during transfer
    AddressMatched,      // Address matched in slave mode
    RxComplete,          // Receive complete
    TxComplete,          // Transmit complete
    Max                  // Must be last - used for array sizing
};

/**
 * I2C Error types
 */
enum class I2CError {
    None,                // No error
    BusError,            // Bus error
    ArbitrationLoss,     // Arbitration lost
    AcknowledgeFail,     // No acknowledge received
    Overrun,             // Overrun/underrun error
    Timeout,             // Timeout error
    PECError,            // PEC error
    InvalidParam,        // Invalid parameter
    NotInitialized,      // I2C interface not initialized
    Busy                 // I2C bus busy
};

    // Current transfer state
    enum class CurrentTransferState {
        Idle,
        StartSent,
        RepeatedStartSent,
        AddrSentW,
        AddrSentR,
        RegAddrSent,
        Writing,
        Reading,
        Error
    };
/**
 * I2C hardware interface implementation
 * 
 * Provides a C++ object-oriented interface to I2C peripherals
 */
class I2CInterface : public HwInterface {
private:
    // Internal state tracking
    bool initialized;
    uint8_t instance;
    I2CConfig config;
    

    struct TransferState {
        bool in_progress;
        CurrentTransferState state;
        uint16_t data_index;
        I2CError last_error;
        bool repeated_start;
        
        // Parameters for the current transfer
        uint16_t device_address;
        uint16_t memory_address;
        uint8_t memory_address_size;
        uint8_t* data_buffer;
        uint16_t data_size;
        uint32_t timeout_ms;
        uint32_t start_time;    // For timeout tracking in interrupts
        
        // For FreeRTOS task notification
        void* notify_task;
    };
    TransferState transfer_state;
    
    // Callback table
    struct CallbackEntry {
        void (*callback)(void* param);
        void* param;
        bool enabled;
    };
    
    CallbackEntry callbacks[static_cast<uint32_t>(I2CEvent::Max)];
    
    // Mutex for thread safety
    std::mutex transfer_mutex;
    
    // Helper methods
    Platform::Status ConfigurePins();
    Platform::Status CalculateTimingParameters();
    Platform::Status StartTransfer(const I2CTransferRequest& request);
    Platform::Status PollForStatus(uint32_t status_bit, bool set_state, uint32_t timeout_ms);
    void HandleError(I2CError error);
    
    // Get hardware registers
    Platform::I2C::Registers* GetI2CRegisters() const;
    
    // Low-level I2C operations
    Platform::Status SendStart();
    Platform::Status SendAddress(uint16_t address, bool read_operation);
    Platform::Status SendByte(uint8_t byte);
    Platform::Status ReceiveByte(uint8_t& byte);
    Platform::Status SendStop();
    Platform::Status RecoverBus();
    
    // Constructor for limited instantiation
    explicit I2CInterface(uint8_t instance);
    
    // Deleted copy constructor and assignment operator
    I2CInterface(const I2CInterface&) = delete;
    I2CInterface& operator=(const I2CInterface&) = delete;
    
    // Static array of instances
    static std::vector<std::weak_ptr<I2CInterface>> instances;
    static std::mutex instances_mutex;
    
public:
    // Destructor
    ~I2CInterface() override;
    
    // Interface implementation
    Platform::Status Init(void* config) override;
    Platform::Status DeInit() override;
    Platform::Status Control(uint32_t command, void* param) override;
    Platform::Status Read(void* buffer, uint16_t size, uint32_t timeout) override;
    Platform::Status Write(const void* data, uint16_t size, uint32_t timeout) override;
    Platform::Status RegisterCallback(uint32_t eventId, void (*callback)(void* param), void* param) override;
    
    // I2C-specific methods
    Platform::Status MasterTransmit(uint16_t dev_addr, const uint8_t* data, uint16_t size, uint32_t timeout);
    Platform::Status MasterReceive(uint16_t dev_addr, uint8_t* data, uint16_t size, uint32_t timeout);
    Platform::Status MemoryWrite(uint16_t dev_addr, uint16_t mem_addr, uint8_t mem_addr_size, 
                                 const uint8_t* data, uint16_t size, uint32_t timeout);
    Platform::Status MemoryRead(uint16_t dev_addr, uint16_t mem_addr, uint8_t mem_addr_size, 
                                uint8_t* data, uint16_t size, uint32_t timeout);
    Platform::Status IsDeviceReady(uint16_t dev_addr, uint32_t trials, uint32_t timeout);
    
    // Getters
    uint8_t GetInstance() const { return instance; }
    bool IsBusy() const { return transfer_state.in_progress; }
    I2CError GetLastError() const { return transfer_state.last_error; }
    
    // Factory method for getting I2C interface
    static std::shared_ptr<I2CInterface> GetInstance(uint8_t instance);
    
    // Allow interrupt handlers to access private state
    friend void I2C1_EV_IRQHandler();
    friend void I2C1_ER_IRQHandler();
    friend void I2C2_EV_IRQHandler();
    friend void I2C2_ER_IRQHandler();
    friend void I2C3_EV_IRQHandler();
    friend void I2C3_ER_IRQHandler();
};

// I2C control command identifiers
constexpr uint32_t I2C_CTRL_MASTER_TRANSMIT = 0x0301;
constexpr uint32_t I2C_CTRL_MASTER_RECEIVE = 0x0302;
constexpr uint32_t I2C_CTRL_MEM_WRITE = 0x0303;
constexpr uint32_t I2C_CTRL_MEM_READ = 0x0304;
constexpr uint32_t I2C_CTRL_SET_TARGET_ADDR = 0x0305;
constexpr uint32_t I2C_CTRL_RECOVER_BUS = 0x0306;
constexpr uint32_t I2C_CTRL_IS_DEVICE_READY = 0x0307;
constexpr uint32_t I2C_CTRL_ENABLE_DMA = 0x0308;
constexpr uint32_t I2C_CTRL_ENABLE_INTERRUPTS = 0x0309;
constexpr uint32_t I2C_CTRL_ANALOG_FILTER = 0x030A;
constexpr uint32_t I2C_CTRL_DIGITAL_FILTER = 0x030B;
constexpr uint32_t I2C_CTRL_STRETCH_CLOCK = 0x030C;
constexpr uint32_t I2C_CTRL_SET_OWN_ADDRESS = 0x030D;
constexpr uint32_t I2C_CTRL_SET_SPEED = 0x030E;
constexpr uint32_t I2C_CTRL_SET_MODE = 0x030F;
constexpr uint32_t I2C_CTRL_SET_ADDRESSING_MODE = 0x0310;
constexpr uint32_t I2C_CTRL_SET_CONFIG = 0x0311;
constexpr uint32_t I2C_CTRL_GET_STATUS = 0x0312;
constexpr uint32_t I2C_CTRL_GET_LAST_ERROR = 0x0313;
