// src/hardware_abstraction/i2c.cpp

#include "hardware_abstraction/i2c.hpp"
#include "hardware_abstraction/gpio.hpp"
#include "hardware_abstraction/rcc.hpp"
#include "system_services/system_timing.hpp"
#include "common/platform_cmsis.hpp"
#include <algorithm>
#include "os/mutex.hpp"


namespace Platform {
namespace I2C {

OS::mutex transfer_mutex;
static OS::mutex instances_mutex;

// Constructor
I2CInterface::I2CInterface(I2CInstance instance) 
    : initialized(false), 
      instance(instance){
    
    // Initialize transfer state
    transfer_state.in_progress = false;
    transfer_state.state = CurrentTransferState::Idle;
    transfer_state.data_index = 0;
    transfer_state.last_error = I2CError::None;
    transfer_state.repeated_start = false;
    transfer_state.notify_task = nullptr;
    
    // Initialize callbacks
    for (auto& callback : callbacks) {
        callback.callback = nullptr;
        callback.param = nullptr;
        callback.enabled = false;
    }
}

// Destructor
I2CInterface::~I2CInterface() {
    if (initialized) {
        DeInit();
    }
}

// Factory method for getting or creating I2C interface instance
I2CInterface& I2CInterface::GetInstance(I2CInstance instance) {

    if (instance < I2CInstance::I2C1 || instance > I2CInstance::I2C3) {
        // Since we can't return nullptr with references, default to I2C1
        instance = I2CInstance::I2C1;
    }
    // Convert to zero-based index
    size_t index = static_cast<uint8_t>(instance) - 1;
    
    OS::lock_guard<OS::mutex> lock(instances_mutex);
    
    // static array of actual instances
    static I2CInterface instances[I2C_INSTANCE_COUNT] = {

        I2CInterface(I2CInstance::I2C1), I2CInterface(I2CInstance::I2C2), I2CInterface(I2CInstance::I2C3)
    };
    
    return instances[index];
}
bool I2CInterface::IsInitialized(void){
    return initialized;
}
// Definition of copy constructor, declared private to prevent copying outside of the getInstance function
I2CInterface::I2CInterface(const I2CInterface& other)
    : initialized(false), // Start uninitialized regardless of source
      instance(other.instance) {
    
    // Reset transfer state instead of copying it
    transfer_state.in_progress = false;
    transfer_state.state = CurrentTransferState::Idle;
    transfer_state.data_index = 0;
    transfer_state.last_error = I2CError::None;
    transfer_state.repeated_start = false;
    transfer_state.notify_task = nullptr;
    
    // Initialize callbacks to empty instead of copying
    for (auto& callback : callbacks) {
        callback.callback = nullptr;
        callback.param = nullptr;
        callback.enabled = false;
    }
}

// Get I2C register base address for this instance
Platform::I2C::Registers* I2CInterface::GetI2CRegisters() const {
    return Platform::I2C::getI2C(instance);
}

// Configure GPIO pins for I2C
Platform::Status I2CInterface::ConfigurePins() {
    // Get GPIO interface
    auto& gpio = Platform::GPIO::GpioInterface::GetInstance();
    
    // Define the I2C pin mapping structure
    struct I2CPinMapping {
        Platform::GPIO::Port scl_port;
        uint8_t scl_pin;
        Platform::GPIO::Port sda_port;
        uint8_t sda_pin;
        Platform::GPIO::AlternateFunction af;
    };
    
    // Define pin mappings for each I2C instance
    static const I2CPinMapping i2c_pins[] = {
        // I2C1
        {
            Platform::GPIO::Port::PORTB,
            6,
            Platform::GPIO::Port::PORTB,
            7,
            Platform::GPIO::AlternateFunction::AF4
        },
        // I2C2
        {
            Platform::GPIO::Port::PORTB,
            10,
            Platform::GPIO::Port::PORTB,
            11,
            Platform::GPIO::AlternateFunction::AF4
        },
        // I2C3
        {
            Platform::GPIO::Port::PORTA,
            8,
            Platform::GPIO::Port::PORTC,
            9,
            Platform::GPIO::AlternateFunction::AF4
        }
    };
    
    // Get the pin mapping for this I2C instance
    const I2CPinMapping& pins = i2c_pins[static_cast<uint8_t>(instance) - 1];
    
    // Common configuration for I2C pins
    Platform::GPIO::GpioConfig pin_config = {
        .mode = Platform::GPIO::Mode::AlternateFunction,
        .outputType = Platform::GPIO::OutputType::OpenDrain,
        .pull = Platform::GPIO::Pull::PullUp,
        .speed = Platform::GPIO::Speed::High,
        .af = pins.af
    };
    
    // Configure SCL pin
    pin_config.port = pins.scl_port;
    pin_config.pin = pins.scl_pin;
    Platform::Status status = gpio.ConfigurePin(pin_config);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Configure SDA pin
    pin_config.port = pins.sda_port;
    pin_config.pin = pins.sda_pin;
    status = gpio.ConfigurePin(pin_config);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    return Platform::Status::OK;
}

Platform::Status I2CInterface::ConfigureInterrupts(bool enable) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Only proceed if interrupts were prepared during initialization
    if (!this->config.enable_interrupts) {
        return Platform::Status::NOT_SUPPORTED;
    }
    
    // Determine interrupt lines for this instance
    Platform::CMSIS::NVIC::IRQn ev_irq, er_irq;
    
    switch (this->instance) {
        case I2CInstance::I2C1:
            ev_irq = Platform::CMSIS::NVIC::IRQn::I2C1_EV;
            er_irq = Platform::CMSIS::NVIC::IRQn::I2C1_ER;
            break;
        case I2CInstance::I2C2:
            ev_irq = Platform::CMSIS::NVIC::IRQn::I2C2_EV;
            er_irq = Platform::CMSIS::NVIC::IRQn::I2C2_ER;
            break;
        case I2CInstance::I2C3:
            ev_irq = Platform::CMSIS::NVIC::IRQn::I2C3_EV;
            er_irq = Platform::CMSIS::NVIC::IRQn::I2C3_ER;
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Enable or disable interrupts in NVIC as requested
    if (enable) {
        Platform::CMSIS::NVIC::enableIRQ(ev_irq);
        Platform::CMSIS::NVIC::enableIRQ(er_irq);
    } else {
        Platform::CMSIS::NVIC::disableIRQ(ev_irq);
        Platform::CMSIS::NVIC::disableIRQ(er_irq);
    }
    
    return Platform::Status::OK;
}
// Calculate I2C timing parameters based on peripheral clock and desired speed
Platform::Status I2CInterface::CalculateTimingParameters() {
    // Get I2C registers
    auto i2c = GetI2CRegisters();
    if (i2c == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Get RCC interface to determine peripheral clock frequency
    Platform::RCC::RccInterface* rcc = &Platform::RCC::RccInterface::GetInstance();

    // Determine peripheral clock frequency
    uint32_t pclk1_freq = rcc->GetApb1ClockFrequency();
    
    // Set peripheral clock frequency in CR2
    uint32_t cr2 = i2c->CR2;
    cr2 &= ~Platform::I2C::getBitValue(Platform::I2C::CR2::FREQ_MSK);
    cr2 |= (pclk1_freq / 1000000); // Set FREQ bits (MHz value)
    i2c->CR2 = cr2;
    
    // Calculate CCR value
    uint32_t ccr_value = 0;
    
    if (config.speed == Platform::I2C::Speed::Standard) {
        // Standard mode (100 KHz)
        // Thigh = Tlow = CCR * TPCLK1
        // For 100 KHz: Thigh + Tlow = 10µs, so Thigh = Tlow = 5µs
        ccr_value = pclk1_freq / (100000 * 2);
        if (ccr_value < 4) ccr_value = 4; // Minimum CCR value
    } else {
        // Fast mode (400 KHz)
        // In fast mode with Duty=1 (Tlow:Thigh = 16:9)
        // Set DUTY bit and F/S bit
        ccr_value = Platform::I2C::getBitValue(Platform::I2C::CCR::DUTY) | 
                    Platform::I2C::getBitValue(Platform::I2C::CCR::FS);
        
        // Calculate CCR value for fast mode
        // Thigh = 9 * CCR * TPCLK1, Tlow = 16 * CCR * TPCLK1
        // For 400 KHz: Thigh + Tlow = 2.5µs
        uint32_t ccr = pclk1_freq / (400000 * 25);
        if (ccr < 1) ccr = 1; // Minimum CCR value
        
        ccr_value |= ccr;
    }
    
    // Update CCR register
    i2c->CCR = ccr_value;
    
    // Calculate TRISE value
    uint32_t trise;
    if (config.speed == Platform::I2C::Speed::Standard) {
        // For standard mode, max rise time is 1000ns
        // TRISE = (1000ns / TPCLK1) + 1
        trise = (pclk1_freq / 1000000) + 1;
    } else {
        // For fast mode, max rise time is 300ns
        // TRISE = (300ns / TPCLK1) + 1
        trise = ((pclk1_freq * 300) / 1000000000) + 1;
    }
    
    // Update TRISE register
    i2c->TRISE = trise;
    
    return Platform::Status::OK;
}

// Initialize I2C interface
Platform::Status I2CInterface::Init(void* config) {

    Middleware::SystemServices::SystemTiming* timing_service =  &Middleware::SystemServices::SystemTiming::GetInstance();

    if (!timing_service->IsInitialized()) {
        return Platform::Status::DEPENDENCY_NOT_INITIALIZED;
    }
    // Check if already initialized
    if (this->initialized) {
        return Platform::Status::OK; // Already initialized
    }
    
    // Validate input parameter
    if (config == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Save configuration
    this->config = *static_cast<I2CConfig*>(config);
    
    // Enable I2C peripheral clock
    Platform::RCC::RccInterface* rcc = &Platform::RCC::RccInterface::GetInstance();
    Platform::RCC::RccPeripheral i2c_clock;
    
    switch (this->instance) {
        case I2CInstance::I2C1:
            i2c_clock = Platform::RCC::RccPeripheral::I2C1;
            break;
        case I2CInstance::I2C2:
            i2c_clock = Platform::RCC::RccPeripheral::I2C2;
            break;
        case I2CInstance::I2C3:
            i2c_clock = Platform::RCC::RccPeripheral::I2C3;
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Enable peripheral clock
    Platform::Status status = rcc->EnablePeripheralClock(i2c_clock);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Configure GPIO pins for I2C
    status = ConfigurePins();
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Get I2C registers
    auto i2c = GetI2CRegisters();
    if (i2c == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Reset the I2C peripheral
    i2c->CR1 = Platform::I2C::getBitValue(Platform::I2C::CR1::SWRST);
    i2c->CR1 = 0; // Clear the reset bit
    
    // Calculate and set timing parameters
    status = CalculateTimingParameters();
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Configure addressing mode and own address
    if (this->config.addressing_mode == Platform::I2C::AddrMode::Addr7Bit) {
        // 7-bit addressing mode
        i2c->OAR1 = (this->config.own_address << 1); // Shift address to bits 7:1
    } else {
        // 10-bit addressing mode
        i2c->OAR1 = this->config.own_address | (1 << 15); // Set ADDMODE bit (bit 15)
    }
    
    // Configure interrupts if enabled
    if (this->config.enable_interrupts) {
        // Configure CR2 register for interrupt enable
        uint32_t cr2 = i2c->CR2;
        
        // Enable event and error interrupts
        cr2 |= Platform::I2C::getBitValue(Platform::I2C::CR2::ITEVTEN) | 
               Platform::I2C::getBitValue(Platform::I2C::CR2::ITERREN);
        
        // Enable buffer interrupts if needed for RXNE and TXE events
        cr2 |= Platform::I2C::getBitValue(Platform::I2C::CR2::ITBUFEN);
        
        i2c->CR2 = cr2;
        
        // Enable I2C interrupts in NVIC
        Platform::CMSIS::NVIC::IRQn ev_irq, er_irq;
        
        switch (this->instance) {
            case I2CInstance::I2C1:
                ev_irq = Platform::CMSIS::NVIC::IRQn::I2C1_EV;
                er_irq = Platform::CMSIS::NVIC::IRQn::I2C1_ER;
                break;
            case I2CInstance::I2C2:
                ev_irq = Platform::CMSIS::NVIC::IRQn::I2C2_EV;
                er_irq = Platform::CMSIS::NVIC::IRQn::I2C2_ER;
                break;
            case I2CInstance::I2C3:
                ev_irq = Platform::CMSIS::NVIC::IRQn::I2C3_EV;
                er_irq = Platform::CMSIS::NVIC::IRQn::I2C3_ER;
                break;
            default:
                return Platform::Status::INVALID_PARAM;
        }
        
        // Set interrupt priorities
        Platform::CMSIS::NVIC::setPriority(ev_irq, Platform::CMSIS::NVIC::PRIORITY_MEDIUM);
        Platform::CMSIS::NVIC::setPriority(er_irq, Platform::CMSIS::NVIC::PRIORITY_MEDIUM);
        
        // Enable the interrupts
        Platform::CMSIS::NVIC::enableIRQ(ev_irq);
        Platform::CMSIS::NVIC::enableIRQ(er_irq);
    }
    
    // Enable I2C peripheral
    i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::PE);
    
    // Wait a bit to ensure peripheral is fully enabled
    for (volatile int i = 0; i < 1000; i++) { }
    
    // Mark as initialized
    initialized = true;
    
    return Platform::Status::OK;
}

// De-initialize I2C interface
Platform::Status I2CInterface::DeInit() {
    // Check if initialized
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Get I2C registers
    auto i2c = GetI2CRegisters();
    if (i2c == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Disable I2C peripheral
    i2c->CR1 &= ~Platform::I2C::getBitValue(Platform::I2C::CR1::PE);
    
    // Disable interrupts in NVIC if they were enabled
    if (config.enable_interrupts) {
        Platform::CMSIS::NVIC::IRQn ev_irq, er_irq;
        
        switch (this->instance) {
            case I2CInstance::I2C1:
                ev_irq = Platform::CMSIS::NVIC::IRQn::I2C1_EV;
                er_irq = Platform::CMSIS::NVIC::IRQn::I2C1_ER;
                break;
            case I2CInstance::I2C2:
                ev_irq = Platform::CMSIS::NVIC::IRQn::I2C2_EV;
                er_irq = Platform::CMSIS::NVIC::IRQn::I2C2_ER;
                break;
            case I2CInstance::I2C3:
                ev_irq = Platform::CMSIS::NVIC::IRQn::I2C3_EV;
                er_irq = Platform::CMSIS::NVIC::IRQn::I2C3_ER;
                break;
            default:
                return Platform::Status::INVALID_PARAM;
        }
        
        // Disable the interrupts
        Platform::CMSIS::NVIC::disableIRQ(ev_irq);
        Platform::CMSIS::NVIC::disableIRQ(er_irq);
    }
    
    // Disable I2C peripheral clock
    Platform::RCC::RccInterface* rcc = &Platform::RCC::RccInterface::GetInstance();
    Platform::RCC::RccPeripheral i2c_clock;
    
    switch (this->instance) {
        case I2CInstance::I2C1:
            i2c_clock = Platform::RCC::RccPeripheral::I2C1;
            break;
        case I2CInstance::I2C2:
            i2c_clock = Platform::RCC::RccPeripheral::I2C2;
            break;
        case I2CInstance::I2C3:
            i2c_clock = Platform::RCC::RccPeripheral::I2C3;
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Disable peripheral clock
    Platform::Status status = rcc->DisablePeripheralClock(i2c_clock);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Reset internal state
    initialized = false;
    transfer_state.in_progress = false;
    transfer_state.state = CurrentTransferState::Idle;
    
    return Platform::Status::OK;
}

// I2C Control function
Platform::Status I2CInterface::Control(uint32_t command, void* param) {
    // Check if initialized
    if (!this->initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Process command
    switch (command) {
        case I2C_CTRL_MASTER_TRANSMIT: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            auto request = static_cast<I2CTransferRequest*>(param);
            return MasterTransmit(request->device_address, 
                                  request->data, 
                                  request->data_size, 
                                  request->timeout_ms);
        }
        
        case I2C_CTRL_MASTER_RECEIVE: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            auto request = static_cast<I2CTransferRequest*>(param);
            return MasterReceive(request->device_address, 
                                 request->data, 
                                 request->data_size, 
                                 request->timeout_ms);
        }
        
        case I2C_CTRL_MEM_WRITE: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            auto request = static_cast<I2CTransferRequest*>(param);
            return MemoryWrite(request->device_address, 
                               request->memory_address, 
                               request->memory_address_size, 
                               request->data, 
                               request->data_size, 
                               request->timeout_ms);
        }
        
        case I2C_CTRL_MEM_READ: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            auto request = static_cast<I2CTransferRequest*>(param);
            return MemoryRead(request->device_address, 
                              request->memory_address, 
                              request->memory_address_size, 
                              request->data, 
                              request->data_size, 
                              request->timeout_ms);
        }
        
        case I2C_CTRL_RECOVER_BUS:
            return RecoverBus();
        
        case I2C_CTRL_IS_DEVICE_READY: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            struct DeviceReadyParams {
                uint16_t device_address;
                uint32_t trials;
                uint32_t timeout_ms;
            };
            
            auto params = static_cast<DeviceReadyParams*>(param);
            return IsDeviceReady(params->device_address, 
                                 params->trials, 
                                 params->timeout_ms);
        }
        
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
}

// Read from I2C device
Platform::Status I2CInterface::Read(void* buffer, uint16_t size, uint32_t timeout) {
    // In this implementation, Read is just a simple wrapper around MasterReceive
    // with a default device address. For a complete implementation, you would need
    // to store the target device address in the class state.
    
    // Check if initialized
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Check parameters
    if (buffer == nullptr || size == 0) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // For simplicity, we assume the first byte in buffer is the memory address
    // and the rest is the data to be read.
    uint8_t mem_addr = *static_cast<uint8_t*>(buffer);
    uint8_t* data_buffer = static_cast<uint8_t*>(buffer) + 1;
    uint16_t data_size = size - 1;
    
    // For a complete implementation, the device address would be stored in the class state
    // or passed as a parameter. Here we use a default for demonstration.
    uint16_t dev_addr = 0x00; // Replace with actual stored device address
    
    return MemoryRead(dev_addr, mem_addr, 1, data_buffer, data_size, timeout);
}

// Write to I2C device
Platform::Status I2CInterface::Write(const void* data, uint16_t size, uint32_t timeout) {
    // Similar to Read, this is a simple wrapper around MasterTransmit
    
    // Check if initialized
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Check parameters
    if (data == nullptr || size == 0) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // For simplicity, we assume the first byte in data is the memory address
    // and the rest is the data to be written.
    uint8_t mem_addr = *static_cast<const uint8_t*>(data);
    const uint8_t* data_buffer = static_cast<const uint8_t*>(data) + 1;
    uint16_t data_size = size - 1;
    
    // For a complete implementation, the device address would be stored in the class state
    // or passed as a parameter. Here we use a default for demonstration.
    uint16_t dev_addr = 0x00; // Replace with actual stored device address
    
    return MemoryWrite(dev_addr, mem_addr, 1, data_buffer, data_size, timeout);
}

// Register callback for I2C events
Platform::Status I2CInterface::RegisterCallback(uint32_t eventId, void (*callback)(void* param), void* param) {
    // Check if initialized
    if (!this->initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Check parameters
    if (eventId >= static_cast<uint32_t>(I2CEvent::Max)) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Register callback
    callbacks[eventId].callback = callback;
    callbacks[eventId].param = param;
    callbacks[eventId].enabled = (callback != nullptr);
    
    return Platform::Status::OK;
}

// Transmit data to I2C device
Platform::Status I2CInterface::MasterTransmit(uint16_t dev_addr, const uint8_t* data, uint16_t size, uint32_t timeout) {

    Middleware::SystemServices::SystemTiming* timing_service =  &Middleware::SystemServices::SystemTiming::GetInstance();

    if (!timing_service->IsInitialized()) {
        return Platform::Status::DEPENDENCY_NOT_INITIALIZED;
    }

    // Check if initialized
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Check parameters
    if (data == nullptr || size == 0) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Check if bus is busy
    if (transfer_state.in_progress) {
        return Platform::Status::BUSY;
    }
    
    // Lock the transfer mutex for thread safety
    OS::lock_guard<OS::mutex> lock(transfer_mutex);
    
    // Get I2C registers
    auto i2c = GetI2CRegisters();
    if (i2c == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Wait until bus is free

    uint64_t start_time = timing_service->GetMilliseconds();
    while (i2c->SR2 & Platform::I2C::getBitValue(Platform::I2C::SR2::BUSY)) {
        // Check for timeout
        if (timing_service->GetMilliseconds() - start_time > timeout) {
            return Platform::Status::TIMEOUT;
        }
        
        // Short delay to prevent tight polling
        timing_service->DelayMicroseconds(10);
    }
    
    // Update transfer state
    transfer_state.in_progress = true;
    transfer_state.state = CurrentTransferState::Idle;
    transfer_state.data_index = 0;
    transfer_state.last_error = I2CError::None;

    bool non_blocking = (timeout == 0);

    // For non-blocking operations, we'll use interrupts if they're enabled
    if (non_blocking && config.enable_interrupts) {

        // enable interrupts
        ConfigureInterrupts(true);
        // Wait until bus is free (this part is still blocking)
        uint64_t start_time = timing_service->GetMilliseconds();

        while (i2c->SR2 & Platform::I2C::getBitValue(Platform::I2C::SR2::BUSY)) {
            // Short delay to prevent tight polling
            timing_service->DelayMicroseconds(10);
            
            // Check for timeout with a reasonable default
            if (timing_service->GetMilliseconds() - start_time > 100) { // 100ms timeout for bus availability
                transfer_state.in_progress = false;
                transfer_state.last_error = I2CError::Timeout;
                return Platform::Status::TIMEOUT;
            }
        }
        
        // Enable acknowledgement
        i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::ACK);
        
        // Enable required interrupts for asynchronous operation
        i2c->CR2 |= Platform::I2C::getBitValue(Platform::I2C::CR2::ITEVTEN) | // Event interrupts
                    Platform::I2C::getBitValue(Platform::I2C::CR2::ITERREN) | // Error interrupts
                    Platform::I2C::getBitValue(Platform::I2C::CR2::ITBUFEN);  // Buffer interrupts
        
        // Start the transfer by sending START condition
        i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::START);
        
        // Now return immediately - the rest will be handled by interrupts
        return Platform::Status::OK;
    }

    // disable interrupts 
    ConfigureInterrupts(false);
    // *** Start the transfer sequence ***
    
    // 1. Send START condition
    i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::START);
    
    // 2. Wait for START to be generated (SB flag)
    if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::SB), true, timeout) != Platform::Status::OK) {
        transfer_state.in_progress = false;
        transfer_state.last_error = I2CError::Timeout;
        return Platform::Status::TIMEOUT;
    }
    
    // 3. Send slave address with write bit (LSB = 0)
    i2c->DR = (dev_addr << 1) & 0xFE;
    transfer_state.state = CurrentTransferState::AddrSentW;
    
    // 4. Wait for address to be sent (ADDR flag)
    if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::ADDR), true, timeout) != Platform::Status::OK) {
        // Check for acknowledge failure (AF flag)
        if (i2c->SR1 & Platform::I2C::getBitValue(Platform::I2C::SR1::AF)) {
            // Generate STOP condition
            i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
            // Clear AF flag
            i2c->SR1 &= ~Platform::I2C::getBitValue(Platform::I2C::SR1::AF);
            
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::AcknowledgeFail;
            return Platform::Status::ERROR;
        }
        
        transfer_state.in_progress = false;
        transfer_state.last_error = I2CError::Timeout;
        return Platform::Status::TIMEOUT;
    }
    
    // 5. Clear ADDR flag by reading SR1 and SR2
    (void)i2c->SR1;
    (void)i2c->SR2;
    
    // 6. Send all data bytes
    transfer_state.state = CurrentTransferState::Writing;
    for (uint16_t i = 0; i < size; i++) {
        // Wait for TXE flag (transmit buffer empty)
        if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::TXE), true, timeout) != Platform::Status::OK) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        // Send byte
        i2c->DR = data[i];
        transfer_state.data_index = i + 1;
        
        // Check for acknowledge failure (AF flag)
        if (i2c->SR1 & Platform::I2C::getBitValue(Platform::I2C::SR1::AF)) {
            // Generate STOP condition
            i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
            // Clear AF flag
            i2c->SR1 &= ~Platform::I2C::getBitValue(Platform::I2C::SR1::AF);
            
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::AcknowledgeFail;
            return Platform::Status::ERROR;
        }
    }
    
    // 7. Wait for BTF flag (all bytes transmitted)
    if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::BTF), true, timeout) != Platform::Status::OK) {
        transfer_state.in_progress = false;
        transfer_state.last_error = I2CError::Timeout;
        return Platform::Status::TIMEOUT;
    }
    
    // 8. Generate STOP condition
    i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
    
    // 9. Wait for STOP bit to be cleared
    start_time = timing_service->GetMilliseconds();
    while (i2c->CR1 & Platform::I2C::getBitValue(Platform::I2C::CR1::STOP)) {
        // Check for timeout
        if (timing_service->GetMilliseconds() - start_time > timeout) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        // Short delay to prevent tight polling
        timing_service->DelayMicroseconds(10);
    }
    // Reset transfer state
    transfer_state.in_progress = false;
    transfer_state.state = CurrentTransferState::Idle;
    
    // Call completion callback if registered
    if (callbacks[static_cast<uint32_t>(I2CEvent::TransferComplete)].enabled) {
        callbacks[static_cast<uint32_t>(I2CEvent::TransferComplete)].callback(
            callbacks[static_cast<uint32_t>(I2CEvent::TransferComplete)].param
        );
    }
    
    return Platform::Status::OK;
}

// Receive data from I2C device
Platform::Status I2CInterface::MasterReceive(uint16_t dev_addr, uint8_t* data, uint16_t size, uint32_t timeout) {

    Middleware::SystemServices::SystemTiming* timing_service =  &Middleware::SystemServices::SystemTiming::GetInstance();

    if (!timing_service->IsInitialized()) {
        return Platform::Status::DEPENDENCY_NOT_INITIALIZED;
    }
    // Check if initialized
    if (!this->initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Check parameters
    if (data == nullptr || size == 0) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Check if bus is busy
    if (transfer_state.in_progress) {
        return Platform::Status::BUSY;
    }
    
    // Lock the transfer mutex for thread safety
    OS::lock_guard<OS::mutex> lock(transfer_mutex);
    
    // Get I2C registers
    auto i2c = GetI2CRegisters();
    if (i2c == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Wait until bus is free
    uint64_t start_time = timing_service->GetMilliseconds(); // Convert to ms
    while (i2c->SR2 & Platform::I2C::getBitValue(Platform::I2C::SR2::BUSY)) {
        // Check for timeout
        if (timing_service->GetMilliseconds() - start_time > timeout) {
            return Platform::Status::TIMEOUT;
        }
        
        // Short delay to prevent tight polling
        timing_service->DelayMicroseconds(10);
    }

    // Enable acknowledgement
    i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::ACK);
    
    // Update transfer state
    transfer_state.in_progress = true;
    transfer_state.state = CurrentTransferState::Idle;
    transfer_state.data_index = 0;
    transfer_state.last_error = I2CError::None;
    
    bool non_blocking = (timeout == 0);

    // For non-blocking operations, we'll use interrupts if they're enabled
    if (non_blocking && config.enable_interrupts) {

        ConfigureInterrupts(true);
        // Wait until bus is free (this part is still blocking)
        uint64_t start_time = timing_service->GetMilliseconds();
        while (i2c->SR2 & Platform::I2C::getBitValue(Platform::I2C::SR2::BUSY)) {
            // Short delay to prevent tight polling
            timing_service->DelayMicroseconds(10);
            
            // Check for timeout with a reasonable default
            if (timing_service->GetMilliseconds() - start_time > 100) { // 100ms timeout for bus availability
                transfer_state.in_progress = false;
                transfer_state.last_error = I2CError::Timeout;
                return Platform::Status::TIMEOUT;
            }
        }
        
        // Enable acknowledgement
        i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::ACK);
        
        // Enable required interrupts for asynchronous operation
        i2c->CR2 |= Platform::I2C::getBitValue(Platform::I2C::CR2::ITEVTEN) | // Event interrupts
                    Platform::I2C::getBitValue(Platform::I2C::CR2::ITERREN) | // Error interrupts
                    Platform::I2C::getBitValue(Platform::I2C::CR2::ITBUFEN);  // Buffer interrupts
        
        // Start the transfer by sending START condition
        i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::START);
        
        // Now return immediately - the rest will be handled by interrupts
        return Platform::Status::OK;
    }

    ConfigureInterrupts(false);
    // *** Start the transfer sequence ***
    
    // 1. Send START condition
    i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::START);
    
    // 2. Wait for START to be generated (SB flag)
    if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::SB), true, timeout) != Platform::Status::OK) {
        transfer_state.in_progress = false;
        transfer_state.last_error = I2CError::Timeout;
        return Platform::Status::TIMEOUT;
    }
    
    // 3. Send slave address with read bit (LSB = 1)
    i2c->DR = (dev_addr << 1) | 0x01;
    transfer_state.state = CurrentTransferState::AddrSentR;
    
    // 4. Wait for address to be sent (ADDR flag)
    if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::ADDR), true, timeout) != Platform::Status::OK) {
        // Check for acknowledge failure (AF flag)
        if (i2c->SR1 & Platform::I2C::getBitValue(Platform::I2C::SR1::AF)) {
            // Generate STOP condition
            i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
            // Clear AF flag
            i2c->SR1 &= ~Platform::I2C::getBitValue(Platform::I2C::SR1::AF);
            
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::AcknowledgeFail;
            return Platform::Status::ERROR;
        }
        
        transfer_state.in_progress = false;
        transfer_state.last_error = I2CError::Timeout;
        return Platform::Status::TIMEOUT;
    }
    
    // 5. Clear ADDR flag by reading SR1 and SR2
    (void)i2c->SR1;
    (void)i2c->SR2;
    
    // Apply different procedures depending on the data size
    if (size == 1) {
        // Only one byte to receive - special case
        
        // Disable Acknowledge bit before clearing ADDR
        i2c->CR1 &= ~Platform::I2C::getBitValue(Platform::I2C::CR1::ACK);
        
        // Generate STOP right after clearing ADDR
        i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
        
        // Wait for RXNE flag
        if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::RXNE), true, timeout) != Platform::Status::OK) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        // Read the byte
        *data = i2c->DR;
        transfer_state.data_index = 1;
        
    } else if (size == 2) {
        // Two bytes to receive - another special case
        
        // 1. Enable POS bit and disable ACK before clearing ADDR
        i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::POS);
        i2c->CR1 &= ~Platform::I2C::getBitValue(Platform::I2C::CR1::ACK);
        
        // 2. Wait for BTF flag (both bytes received)
        if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::BTF), true, timeout) != Platform::Status::OK) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        // 3. Generate STOP
        i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
        
        // 4. Read both bytes
        data[0] = i2c->DR;
        data[1] = i2c->DR;
        transfer_state.data_index = 2;
        
    } else {
        // More than 2 bytes
        transfer_state.state = CurrentTransferState::Reading;
        
        // Read bytes until the last 3
        for (uint16_t i = 0; i < size - 3; i++) {
            // Wait for RXNE flag
            if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::RXNE), true, timeout) != Platform::Status::OK) {
                transfer_state.in_progress = false;
                transfer_state.last_error = I2CError::Timeout;
                return Platform::Status::TIMEOUT;
            }
            
            // Read the byte
            data[i] = i2c->DR;
            transfer_state.data_index = i + 1;
        }
        
        // Read the n-2 byte (size - 3)
        if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::RXNE), true, timeout) != Platform::Status::OK) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        // Wait for BTF (n-2 byte in DR, n-1 in shift register)
        if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::BTF), true, timeout) != Platform::Status::OK) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        // Disable ACK
        i2c->CR1 &= ~Platform::I2C::getBitValue(Platform::I2C::CR1::ACK);
        
        // Read n-2 byte
        data[size - 3] = i2c->DR;
        
        // Wait for BTF (n-1 byte in DR, n in shift register)
        if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::BTF), true, timeout) != Platform::Status::OK) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        // Generate STOP
        i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
        
        // Read n-1 byte
        data[size - 2] = i2c->DR;
        
        // Read n byte
        data[size - 1] = i2c->DR;
        
        transfer_state.data_index = size;
    }
    
    // Reset POS bit if it was set
    i2c->CR1 &= ~Platform::I2C::getBitValue(Platform::I2C::CR1::POS);
    
    // Wait for STOP bit to be cleared
    start_time = timing_service->GetMilliseconds();
    while (i2c->CR1 & Platform::I2C::getBitValue(Platform::I2C::CR1::STOP)) {
        // Check for timeout
        if (timing_service->GetMilliseconds() - start_time > timeout) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        // Short delay to prevent tight polling
        timing_service->DelayMicroseconds(10);
    }
    
    // Reset transfer state
    transfer_state.in_progress = false;
    transfer_state.state = CurrentTransferState::Idle;
    
    // Call completion callback if registered
    if (callbacks[static_cast<uint32_t>(I2CEvent::TransferComplete)].enabled) {
        callbacks[static_cast<uint32_t>(I2CEvent::TransferComplete)].callback(
            callbacks[static_cast<uint32_t>(I2CEvent::TransferComplete)].param
        );
    }
    
    return Platform::Status::OK;
}

// Write to a specific memory address in an I2C device
Platform::Status I2CInterface::MemoryWrite(uint16_t dev_addr, uint16_t mem_addr, uint8_t mem_addr_size, 
                                         const uint8_t* data, uint16_t size, uint32_t timeout) {
    // Check if initialized

    Middleware::SystemServices::SystemTiming* timing_service =  &Middleware::SystemServices::SystemTiming::GetInstance();

    if (!timing_service->IsInitialized()) {
        return Platform::Status::DEPENDENCY_NOT_INITIALIZED;
    }
    
    if (!this->initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Check parameters
    if (data == nullptr || size == 0 || (mem_addr_size != 1 && mem_addr_size != 2)) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Check if bus is busy
    if (transfer_state.in_progress) {
        return Platform::Status::BUSY;
    }
    
    // Lock the transfer mutex for thread safety
    OS::lock_guard<OS::mutex> lock(transfer_mutex);
    
    // Get I2C registers
    auto i2c = GetI2CRegisters();
    if (i2c == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Wait until bus is free
    uint64_t start_time = timing_service->GetMilliseconds(); 


    while (i2c->SR2 & Platform::I2C::getBitValue(Platform::I2C::SR2::BUSY)) {
        // Check for timeout
        if (timing_service->GetMilliseconds() - start_time > timeout) {
            return Platform::Status::TIMEOUT;
        }
        
        // Short delay to prevent tight polling
        timing_service->DelayMicroseconds(10);
    }
    
    // Update transfer state
    transfer_state.in_progress = true;
    transfer_state.state = CurrentTransferState::Idle;
    transfer_state.data_index = 0;
    transfer_state.last_error = I2CError::None;
    
    // Check if this is a non-blocking operation (timeout == 0)
    bool non_blocking = (timeout == 0);

    // For non-blocking operations, we'll use interrupts if they're enabled

    if (non_blocking && config.enable_interrupts) {

        ConfigureInterrupts(true);
        // Wait until bus is free (this part is still blocking)
        uint64_t start_time = timing_service->GetMilliseconds();
        while (i2c->SR2 & Platform::I2C::getBitValue(Platform::I2C::SR2::BUSY)) {
            // Short delay to prevent tight polling
            timing_service->DelayMicroseconds(10);
            
            // Check for timeout with a reasonable default
            if (timing_service->GetMilliseconds() - start_time > 100) { // 100ms timeout for bus availability
                transfer_state.in_progress = false;
                transfer_state.last_error = I2CError::Timeout;
                return Platform::Status::TIMEOUT;
            }
        }
        
        // Enable acknowledgement
        i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::ACK);
        
        // Enable required interrupts for asynchronous operation
        i2c->CR2 |= Platform::I2C::getBitValue(Platform::I2C::CR2::ITEVTEN) | // Event interrupts
                    Platform::I2C::getBitValue(Platform::I2C::CR2::ITERREN) | // Error interrupts
                    Platform::I2C::getBitValue(Platform::I2C::CR2::ITBUFEN);  // Buffer interrupts
        
        // Start the transfer by sending START condition
        i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::START);
        
        // Now return immediately - the rest will be handled by interrupts
        return Platform::Status::OK;
    }

    ConfigureInterrupts(false);
    // *** Start the transfer sequence ***
    
    // 1. Send START condition
    i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::START);
    
    // 2. Wait for START to be generated (SB flag)
    if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::SB), true, timeout) != Platform::Status::OK) {
        transfer_state.in_progress = false;
        transfer_state.last_error = I2CError::Timeout;
        return Platform::Status::TIMEOUT;
    }
    
    // 3. Send slave address with write bit (LSB = 0)
    i2c->DR = (dev_addr << 1) & 0xFE;
    transfer_state.state = CurrentTransferState::AddrSentW;
    
    // 4. Wait for address to be sent (ADDR flag)
    if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::ADDR), true, timeout) != Platform::Status::OK) {
        // Check for acknowledge failure (AF flag)
        if (i2c->SR1 & Platform::I2C::getBitValue(Platform::I2C::SR1::AF)) {
            // Generate STOP condition
            i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
            // Clear AF flag
            i2c->SR1 &= ~Platform::I2C::getBitValue(Platform::I2C::SR1::AF);
            
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::AcknowledgeFail;
            return Platform::Status::ERROR;
        }
        
        transfer_state.in_progress = false;
        transfer_state.last_error = I2CError::Timeout;
        return Platform::Status::TIMEOUT;
    }
    
    // 5. Clear ADDR flag by reading SR1 and SR2
    (void)i2c->SR1;
    (void)i2c->SR2;
    
    // 6. Send memory address (1 or 2 bytes)
    if (mem_addr_size == 1) {
        // 8-bit memory address
        if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::TXE), true, timeout) != Platform::Status::OK) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        i2c->DR = mem_addr & 0xFF;
    } else {
        // 16-bit memory address, send MSB first
        if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::TXE), true, timeout) != Platform::Status::OK) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        i2c->DR = (mem_addr >> 8) & 0xFF;
        
        if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::TXE), true, timeout) != Platform::Status::OK) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        i2c->DR = mem_addr & 0xFF;
    }
    
    // 7. Wait for TXE flag
    if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::TXE), true, timeout) != Platform::Status::OK) {
        transfer_state.in_progress = false;
        transfer_state.last_error = I2CError::Timeout;
        return Platform::Status::TIMEOUT;
    }
    
    // 8. Send data bytes
    transfer_state.state = CurrentTransferState::Writing;
    for (uint16_t i = 0; i < size; i++) {
        // Wait for TXE flag
        if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::TXE), true, timeout) != Platform::Status::OK) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        // Send byte
        i2c->DR = data[i];
        transfer_state.data_index = i + 1;
        
        // Check for acknowledge failure (AF flag)
        if (i2c->SR1 & Platform::I2C::getBitValue(Platform::I2C::SR1::AF)) {
            // Generate STOP condition
            i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
            // Clear AF flag
            i2c->SR1 &= ~Platform::I2C::getBitValue(Platform::I2C::SR1::AF);
            
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::AcknowledgeFail;
            return Platform::Status::ERROR;
        }
    }
    
    // 9. Wait for BTF flag (all bytes transmitted)
    if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::BTF), true, timeout) != Platform::Status::OK) {
        transfer_state.in_progress = false;
        transfer_state.last_error = I2CError::Timeout;
        return Platform::Status::TIMEOUT;
    }
    
    // 10. Generate STOP condition
    i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
    
    // Reset transfer state
    transfer_state.in_progress = false;
    transfer_state.state = CurrentTransferState::Idle;
    
    // Call completion callback if registered
    if (callbacks[static_cast<uint32_t>(I2CEvent::TransferComplete)].enabled) {
        callbacks[static_cast<uint32_t>(I2CEvent::TransferComplete)].callback(
            callbacks[static_cast<uint32_t>(I2CEvent::TransferComplete)].param
        );
    }
    
    return Platform::Status::OK;
}

Platform::Status I2CInterface::MemoryRead(uint16_t dev_addr, uint16_t mem_addr, uint8_t mem_addr_size, 
                                        uint8_t* data, uint16_t size, uint32_t timeout) {
    // Check if initialized
    Middleware::SystemServices::SystemTiming* timing_service =  &Middleware::SystemServices::SystemTiming::GetInstance();

    if (!timing_service->IsInitialized()) {
        return Platform::Status::DEPENDENCY_NOT_INITIALIZED;
    }
    if (!this->initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Check parameters
    if (data == nullptr || size == 0 || (mem_addr_size != 1 && mem_addr_size != 2)) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Check if bus is busy
    if (transfer_state.in_progress) {
        return Platform::Status::BUSY;
    }
    
    // Lock the transfer mutex for thread safety
    OS::lock_guard<OS::mutex> lock(transfer_mutex);
    
    // Get I2C registers
    auto i2c = GetI2CRegisters();
    if (i2c == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Set up the transfer state
    transfer_state.in_progress = true;
    transfer_state.state = CurrentTransferState::Idle;
    transfer_state.data_index = 0;
    transfer_state.last_error = I2CError::None;
    transfer_state.repeated_start = true;
    
    // Store all transfer parameters for use by the interrupt handlers
    transfer_state.device_address = dev_addr;
    transfer_state.memory_address = mem_addr;
    transfer_state.memory_address_size = mem_addr_size;
    transfer_state.data_buffer = data;
    transfer_state.data_size = size;
    transfer_state.timeout_ms = timeout;
    transfer_state.start_time = timing_service->GetMilliseconds(); // For timeout tracking
    
    // Check if this is a non-blocking operation (timeout == 0)
    bool non_blocking = (timeout == 0);
    
    // For non-blocking operations, we'll use interrupts if they're enabled
    if (non_blocking && config.enable_interrupts) {

        ConfigureInterrupts(true);
        // Wait until bus is free (this part is still blocking)
        // We could make this part non-blocking too with more state machine complexity
        uint64_t start_time = timing_service->GetMilliseconds();
        while (i2c->SR2 & Platform::I2C::getBitValue(Platform::I2C::SR2::BUSY)) {
            // Short delay to prevent tight polling
            timing_service->DelayMicroseconds(10);
            
            // Check for timeout with a reasonable default
            if (timing_service->GetMilliseconds() - start_time > 100) { // 100ms timeout for bus availability
                transfer_state.in_progress = false;
                transfer_state.last_error = I2CError::Timeout;
                return Platform::Status::TIMEOUT;
            }
        }
        
        // Enable acknowledgement
        i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::ACK);
        
        // Enable required interrupts for asynchronous operation
        i2c->CR2 |= Platform::I2C::getBitValue(Platform::I2C::CR2::ITEVTEN) | // Event interrupts
                   Platform::I2C::getBitValue(Platform::I2C::CR2::ITERREN) | // Error interrupts
                   Platform::I2C::getBitValue(Platform::I2C::CR2::ITBUFEN);  // Buffer interrupts
        
        // Start the transfer by sending START condition
        i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::START);
        
        // Now return immediately - the rest will be handled by interrupts
        return Platform::Status::OK;
    }
    
    // For blocking operations, use the polling approach
    ConfigureInterrupts(false);
    // Wait until bus is free
    uint64_t start_time = timing_service->GetMilliseconds();
    while (i2c->SR2 & Platform::I2C::getBitValue(Platform::I2C::SR2::BUSY)) {
        // Check for timeout
        if (timing_service->GetMilliseconds() - start_time > timeout) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        // Short delay to prevent tight polling
        timing_service->DelayMicroseconds(10);
    }
    
    // Enable acknowledgement
    i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::ACK);
    
    // *** Start the transfer sequence - Part 1: Write memory address ***
    
    // 1. Send START condition
    i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::START);
    
    // 2. Wait for START to be generated (SB flag)
    if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::SB), true, timeout) != Platform::Status::OK) {
        transfer_state.in_progress = false;
        transfer_state.last_error = I2CError::Timeout;
        return Platform::Status::TIMEOUT;
    }
    
    // 3. Send slave address with write bit (LSB = 0)
    i2c->DR = (dev_addr << 1) & 0xFE;
    transfer_state.state = CurrentTransferState::AddrSentW;
    
    // 4. Wait for address to be sent (ADDR flag)
    if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::ADDR), true, timeout) != Platform::Status::OK) {
        // Check for acknowledge failure (AF flag)
        if (i2c->SR1 & Platform::I2C::getBitValue(Platform::I2C::SR1::AF)) {
            // Generate STOP condition
            i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
            // Clear AF flag
            i2c->SR1 &= ~Platform::I2C::getBitValue(Platform::I2C::SR1::AF);
            
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::AcknowledgeFail;
            return Platform::Status::ERROR;
        }
        
        transfer_state.in_progress = false;
        transfer_state.last_error = I2CError::Timeout;
        return Platform::Status::TIMEOUT;
    }
    
    // 5. Clear ADDR flag by reading SR1 and SR2
    (void)i2c->SR1;
    (void)i2c->SR2;
    
    // 6. Send memory address (1 or 2 bytes)
    transfer_state.state = CurrentTransferState::RegAddrSent;
    if (mem_addr_size == 1) {
        // 8-bit memory address
        if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::TXE), true, timeout) != Platform::Status::OK) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        i2c->DR = mem_addr & 0xFF;
    } else {
        // 16-bit memory address, send MSB first
        if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::TXE), true, timeout) != Platform::Status::OK) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        i2c->DR = (mem_addr >> 8) & 0xFF;
        
        if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::TXE), true, timeout) != Platform::Status::OK) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        i2c->DR = mem_addr & 0xFF;
    }
    
    // 7. Wait for TXE flag
    if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::TXE), true, timeout) != Platform::Status::OK) {
        transfer_state.in_progress = false;
        transfer_state.last_error = I2CError::Timeout;
        return Platform::Status::TIMEOUT;
    }
    
    // *** Start the transfer sequence - Part 2: Read data with repeated START ***
    
    // 8. Send repeated START condition
    i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::START);
    transfer_state.state = CurrentTransferState::RepeatedStartSent;
    
    // 9. Wait for START to be generated (SB flag)
    if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::SB), true, timeout) != Platform::Status::OK) {
        transfer_state.in_progress = false;
        transfer_state.last_error = I2CError::Timeout;
        return Platform::Status::TIMEOUT;
    }
    
    // 10. Send slave address with read bit (LSB = 1)
    i2c->DR = (dev_addr << 1) | 0x01;
    transfer_state.state = CurrentTransferState::AddrSentR;
    
    // 11. Wait for address to be sent (ADDR flag)
    if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::ADDR), true, timeout) != Platform::Status::OK) {
        // Check for acknowledge failure (AF flag)
        if (i2c->SR1 & Platform::I2C::getBitValue(Platform::I2C::SR1::AF)) {
            // Generate STOP condition
            i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
            // Clear AF flag
            i2c->SR1 &= ~Platform::I2C::getBitValue(Platform::I2C::SR1::AF);
            
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::AcknowledgeFail;
            return Platform::Status::ERROR;
        }
        
        transfer_state.in_progress = false;
        transfer_state.last_error = I2CError::Timeout;
        return Platform::Status::TIMEOUT;
    }
    
    // 12. Apply appropriate procedure depending on data size
    if (size == 1) {
        // Only one byte to receive
        
        // Disable Acknowledge bit before clearing ADDR
        i2c->CR1 &= ~Platform::I2C::getBitValue(Platform::I2C::CR1::ACK);
        
        // Clear ADDR flag by reading SR1 and SR2
        (void)i2c->SR1;
        (void)i2c->SR2;
        
        // Generate STOP right after clearing ADDR
        i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
        
        // Wait for RXNE flag
        if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::RXNE), true, timeout) != Platform::Status::OK) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        // Read the byte
        *data = i2c->DR;
        transfer_state.data_index = 1;
        
    } else if (size == 2) {
        // Two bytes to receive
        
        // Enable POS bit and disable ACK before clearing ADDR
        i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::POS);
        i2c->CR1 &= ~Platform::I2C::getBitValue(Platform::I2C::CR1::ACK);
        
        // Clear ADDR flag by reading SR1 and SR2
        (void)i2c->SR1;
        (void)i2c->SR2;
        
        // Wait for BTF flag (both bytes received)
        if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::BTF), true, timeout) != Platform::Status::OK) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        // Generate STOP
        i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
        
        // Read both bytes
        data[0] = i2c->DR;
        data[1] = i2c->DR;
        transfer_state.data_index = 2;
        
    } else {
        // More than 2 bytes
        transfer_state.state = CurrentTransferState::Reading;
        
        // Clear ADDR flag by reading SR1 and SR2
        (void)i2c->SR1;
        (void)i2c->SR2;
        
        // Read bytes until the last 3
        for (uint16_t i = 0; i < size - 3; i++) {
            // Wait for RXNE flag
            if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::RXNE), true, timeout) != Platform::Status::OK) {
                transfer_state.in_progress = false;
                transfer_state.last_error = I2CError::Timeout;
                return Platform::Status::TIMEOUT;
            }
            
            // Read the byte
            data[i] = i2c->DR;
            transfer_state.data_index = i + 1;
        }
        
        // Wait for BTF (n-2 byte in DR, n-1 in shift register)
        if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::BTF), true, timeout) != Platform::Status::OK) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        // Disable ACK
        i2c->CR1 &= ~Platform::I2C::getBitValue(Platform::I2C::CR1::ACK);
        
        // Read n-2 byte
        data[size - 3] = i2c->DR;
        
        // Wait for BTF (n-1 byte in DR, n in shift register)
        if (PollForStatus(Platform::I2C::getBitValue(Platform::I2C::SR1::BTF), true, timeout) != Platform::Status::OK) {
            transfer_state.in_progress = false;
            transfer_state.last_error = I2CError::Timeout;
            return Platform::Status::TIMEOUT;
        }
        
        // Generate STOP
        i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
        
        // Read n-1 byte
        data[size - 2] = i2c->DR;
        
        // Read n byte
        data[size - 1] = i2c->DR;
        
        transfer_state.data_index = size;
    }
    
    // Reset POS bit if it was set
    i2c->CR1 &= ~Platform::I2C::getBitValue(Platform::I2C::CR1::POS);
    
    // Reset transfer state
    transfer_state.in_progress = false;
    transfer_state.state = CurrentTransferState::Idle;
    transfer_state.repeated_start = false;
    
    // Call completion callback if registered
    if (callbacks[static_cast<uint32_t>(I2CEvent::TransferComplete)].enabled) {
        callbacks[static_cast<uint32_t>(I2CEvent::TransferComplete)].callback(
            callbacks[static_cast<uint32_t>(I2CEvent::TransferComplete)].param
        );
    }
    
    return Platform::Status::OK;
}
// Check if a device is ready by sending its address and checking for ACK
Platform::Status I2CInterface::IsDeviceReady(uint16_t dev_addr, uint32_t trials, uint32_t timeout) {
    // Check if initialized
    Middleware::SystemServices::SystemTiming* timing_service =  &Middleware::SystemServices::SystemTiming::GetInstance();

    if (!timing_service->IsInitialized()) {
        return Platform::Status::DEPENDENCY_NOT_INITIALIZED;
    }
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Check if bus is busy
    if (transfer_state.in_progress) {
        return Platform::Status::BUSY;
    }
    
    // Lock the transfer mutex for thread safety
    OS::lock_guard<OS::mutex> lock(transfer_mutex);
    
    // Get I2C registers
    auto i2c = GetI2CRegisters();
    if (i2c == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Set timeout timestamp
    uint64_t start_time = timing_service->GetMilliseconds();
    uint32_t end_time = start_time + timeout;
    
    // Try for the number of trials
    while (trials > 0) {
        // Check timeout
        if (timing_service->GetMilliseconds() >= end_time) {
            return Platform::Status::TIMEOUT;
        }
        
        // Wait until bus is free
        while (i2c->SR2 & Platform::I2C::getBitValue(Platform::I2C::SR2::BUSY)) {
            // Check for timeout
            if (timing_service->GetMilliseconds() >= end_time) {
                return Platform::Status::TIMEOUT;
            }
            
            // Short delay to prevent tight polling
            timing_service->DelayMicroseconds(10);
        }
        
        // Send START condition
        i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::START);
        
        // Wait for START to be generated (SB flag)
        while (!(i2c->SR1 & Platform::I2C::getBitValue(Platform::I2C::SR1::SB))) {
            // Check for timeout
            if (timing_service->GetMilliseconds() >= end_time) {
                return Platform::Status::TIMEOUT;
            }
        }
        
        // Send slave address with write bit (LSB = 0)
        i2c->DR = (dev_addr << 1) & 0xFE;
        
        // Wait for ADDR or AF flag
        while (!(i2c->SR1 & (Platform::I2C::getBitValue(Platform::I2C::SR1::ADDR) | 
                             Platform::I2C::getBitValue(Platform::I2C::SR1::AF)))) {
            // Check for timeout
            if (timing_service->GetMilliseconds() >= end_time) {
                // Generate STOP condition
                i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
                return Platform::Status::TIMEOUT;
            }
        }
        
        // Check if ADDR flag is set (address matched, ACK received)
        if (i2c->SR1 & Platform::I2C::getBitValue(Platform::I2C::SR1::ADDR)) {
            // Clear ADDR flag by reading SR1 and SR2
            (void)i2c->SR1;
            (void)i2c->SR2;
            
            // Generate STOP condition
            i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
            
            // Wait for STOP bit to be cleared
            while (i2c->CR1 & Platform::I2C::getBitValue(Platform::I2C::CR1::STOP)) {
                // Check for timeout
                if (timing_service->GetMilliseconds() >= end_time) {
                    return Platform::Status::TIMEOUT;
                }
            }
            
            // Device is ready
            return Platform::Status::OK;
        } else {
            // Address not acknowledged (AF flag set)
            // Clear AF flag
            i2c->SR1 &= ~Platform::I2C::getBitValue(Platform::I2C::SR1::AF);
            
            // Generate STOP condition
            i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
            
            // Wait for STOP bit to be cleared
            while (i2c->CR1 & Platform::I2C::getBitValue(Platform::I2C::CR1::STOP)) {
                // Check for timeout
                if (timing_service->GetMilliseconds() >= end_time) {
                    return Platform::Status::TIMEOUT;
                }
            }
            
            // Try again
            trials--;
            
            // Short delay between trials
            timing_service->DelayMilliseconds(10);
        }
    }
    
    // All trials failed
    return Platform::Status::ERROR;
}

// Recover the I2C bus from errors
Platform::Status I2CInterface::RecoverBus() {

    Middleware::SystemServices::SystemTiming* timing_service =  &Middleware::SystemServices::SystemTiming::GetInstance();

    if (!timing_service->IsInitialized()) {
        return Platform::Status::DEPENDENCY_NOT_INITIALIZED;
    }
    // Check if initialized
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Get the GPIO interface
    auto& gpio = Platform::GPIO::GpioInterface::GetInstance();
    
    // Define the I2C pin mapping structure
    struct I2CPinMapping {
        Platform::GPIO::Port scl_port;
        uint8_t scl_pin;
        Platform::GPIO::Port sda_port;
        uint8_t sda_pin;
    };
    
    // Define pin mappings for each I2C instance
    static const I2CPinMapping i2c_pins[] = {
        // I2C1
        {
            Platform::GPIO::Port::PORTB,
            6,
            Platform::GPIO::Port::PORTB,
            7
        },
        // I2C2
        {
            Platform::GPIO::Port::PORTB,
            10,
            Platform::GPIO::Port::PORTB,
            11
        },
        // I2C3
        {
            Platform::GPIO::Port::PORTA,
            8,
            Platform::GPIO::Port::PORTC,
            9
        }
    };
    
    // Get the pin mapping for this I2C instance
    const I2CPinMapping& pins = i2c_pins[static_cast<uint8_t>(this->instance) - 1];
    
    // Get I2C registers
    auto i2c = GetI2CRegisters();
    if (i2c == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Disable I2C peripheral
    i2c->CR1 &= ~Platform::I2C::getBitValue(Platform::I2C::CR1::PE);
    
    // Configure SCL as GPIO output open-drain
    Platform::GPIO::GpioConfig scl_config = {
        .port = pins.scl_port,
        .pin = pins.scl_pin,
        .mode = Platform::GPIO::Mode::Output,
        .outputType = Platform::GPIO::OutputType::OpenDrain,
        .pull = Platform::GPIO::Pull::PullUp,
        .speed = Platform::GPIO::Speed::High
    };
    
    // Configure SDA as GPIO output open-drain
    Platform::GPIO::GpioConfig sda_config = {
        .port = pins.sda_port,
        .pin = pins.sda_pin,
        .mode = Platform::GPIO::Mode::Output,
        .outputType = Platform::GPIO::OutputType::OpenDrain,
        .pull = Platform::GPIO::Pull::PullUp,
        .speed = Platform::GPIO::Speed::High
    };
    
    // Configure SCL and SDA as GPIO outputs
    gpio.ConfigurePin(scl_config);
    gpio.ConfigurePin(sda_config);
    
    // Set both lines high
    gpio.SetPin(scl_config.port, scl_config.pin);
    gpio.SetPin(sda_config.port, sda_config.pin);
    timing_service->DelayMicroseconds(10);
    
    // Generate clock pulses to make slave release SDA line
    // Slave devices should release SDA after at most 9 clock cycles
    for (int i = 0; i < 9; i++) {
        // Pull SCL low
        gpio.ResetPin(scl_config.port, scl_config.pin);
        timing_service->DelayMicroseconds(10);
        
        // Pull SCL high
        gpio.SetPin(scl_config.port, scl_config.pin);
        timing_service->DelayMicroseconds(10);
    }
    
    // Generate a STOP condition (SDA low to high while SCL is high)
    // First, ensure SCL is high
    gpio.SetPin(scl_config.port, scl_config.pin);
    timing_service->DelayMicroseconds(10);
    
    // Pull SDA low
    gpio.ResetPin(sda_config.port, sda_config.pin);
    timing_service->DelayMicroseconds(10);
    
    // Pull SDA high while SCL is high (STOP condition)
    gpio.SetPin(sda_config.port, sda_config.pin);
    timing_service->DelayMicroseconds(10);
    
    // Reconfigure the pins for I2C mode
    scl_config.mode = Platform::GPIO::Mode::AlternateFunction;
    scl_config.af = Platform::GPIO::AlternateFunction::AF4;
    sda_config.mode = Platform::GPIO::Mode::AlternateFunction;
    sda_config.af = Platform::GPIO::AlternateFunction::AF4;
    
    gpio.ConfigurePin(scl_config);
    gpio.ConfigurePin(sda_config);
    
    // Re-enable I2C peripheral
    i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::PE);
    
    return Platform::Status::OK;
}

// Poll for a status bit with timeout
Platform::Status I2CInterface::PollForStatus(uint32_t status_bit, bool set_state, uint32_t timeout_ms) {
    auto i2c = GetI2CRegisters();
    if (i2c == nullptr) {
        return Platform::Status::ERROR;
    }
    Middleware::SystemServices::SystemTiming* timing_service =  &Middleware::SystemServices::SystemTiming::GetInstance();

    if (!timing_service->IsInitialized()) {
        return Platform::Status::DEPENDENCY_NOT_INITIALIZED;
    }
    uint64_t start_time = timing_service->GetMilliseconds(); // Convert to ms
    
    while (true) {
        // Check if the status bit is in the desired state
        bool is_set = (i2c->SR1 & status_bit) != 0;
        if (is_set == set_state) {
            return Platform::Status::OK;
        }
        
        // Check for timeout
        if (timing_service->GetMilliseconds() - start_time > timeout_ms) {
            return Platform::Status::TIMEOUT;
        }
        
        // Short delay to prevent tight polling
        timing_service->DelayMicroseconds(10);
    }
}

// Handle I2C error
void I2CInterface::HandleError(I2CError error) {
    // Set error in transfer state
    transfer_state.last_error = error;
    
    // Reset transfer state
    transfer_state.in_progress = false;
    transfer_state.state = CurrentTransferState::Idle;
    
    // Call error callback if registered
    if (callbacks[static_cast<uint32_t>(I2CEvent::TransferError)].enabled) {
        callbacks[static_cast<uint32_t>(I2CEvent::TransferError)].callback(
            callbacks[static_cast<uint32_t>(I2CEvent::TransferError)].param
        );
    }
}
}
}// I2C interrupt handlers (Event)
void I2C1_EV_IRQHandler() {
    // Get I2C instance
    Platform::I2C::I2CInterface* i2c_interface = &Platform::I2C::I2CInterface::GetInstance(Platform::I2C::I2CInstance::I2C1);
    if (!i2c_interface || !i2c_interface->initialized) {
        return;
    }
    
    Platform::I2C::Registers* i2c = i2c_interface->GetI2CRegisters();
    uint32_t sr1 = i2c->SR1;
    uint32_t sr2 = i2c->SR2;

    // Start condition generated
    if (sr1 & Platform::I2C::getBitValue(Platform::I2C::SR1::SB)) {
        if (i2c_interface->transfer_state.state == Platform::I2C::CurrentTransferState::RepeatedStartSent) {
            // Send slave address with read bit
            i2c->DR = (i2c_interface->transfer_state.device_address << 1) | 0x01;
            i2c_interface->transfer_state.repeated_start = false;
            i2c_interface->transfer_state.state = Platform::I2C::CurrentTransferState::AddrSentR;
        } else {
            // Send slave address with write bit
            i2c->DR = (i2c_interface->transfer_state.device_address << 1) & 0xFE;
            i2c_interface->transfer_state.state = Platform::I2C::CurrentTransferState::AddrSentW;
        }
    }
    // Address sent
    else if (sr1 & Platform::I2C::getBitValue(Platform::I2C::SR1::ADDR)) {
        if (i2c_interface->transfer_state.state == Platform::I2C::CurrentTransferState::AddrSentW) {
            // Clear ADDR flag by reading SR1 and SR2
            (void)i2c->SR1;
            (void)i2c->SR2;
            i2c_interface->transfer_state.state = Platform::I2C::CurrentTransferState::RegAddrSent;
        } 
        else if(i2c_interface->transfer_state.state == Platform::I2C::CurrentTransferState::AddrSentR) {
            if(i2c_interface->transfer_state.data_size > 1) {
                // Enable ACK for multiple byte read
                i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::ACK);                
            }
            else {
                // Disable ACK and send stop for single byte read 
                i2c->CR1 &= ~Platform::I2C::getBitValue(Platform::I2C::CR1::ACK);
                i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
            }
            
            // Clear ADDR flag by reading SR1 and SR2
            (void)i2c->SR1;
            (void)i2c->SR2;
            i2c_interface->transfer_state.state = Platform::I2C::CurrentTransferState::Reading;
        }
    }
    // Transmit interrupt (TXE) for write operation
    else if(sr1 & Platform::I2C::getBitValue(Platform::I2C::SR1::TXE)) {
        // If we just cleared the address flag, send target register
        if(i2c_interface->transfer_state.state == Platform::I2C::CurrentTransferState::RegAddrSent) {
            i2c->DR = i2c_interface->transfer_state.memory_address;  
            i2c_interface->transfer_state.state = Platform::I2C::CurrentTransferState::Writing;
        }
        // If target register sent, and this is a read op, generate restart
        else if(i2c_interface->transfer_state.state == Platform::I2C::CurrentTransferState::Writing) {
            if(i2c_interface->transfer_state.repeated_start) {
                i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::START);
                i2c_interface->transfer_state.state = Platform::I2C::CurrentTransferState::RepeatedStartSent;               
            }
            else {
                // Handle write data
                if(i2c_interface->transfer_state.data_index == i2c_interface->transfer_state.data_size) {
                    // Disable interrupts
                    i2c->CR2 &= ~(Platform::I2C::getBitValue(Platform::I2C::CR2::ITEVTEN) | 
                                 Platform::I2C::getBitValue(Platform::I2C::CR2::ITERREN) | 
                                 Platform::I2C::getBitValue(Platform::I2C::CR2::ITBUFEN));
                    // Send stop
                    i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
                    i2c_interface->transfer_state.state = Platform::I2C::CurrentTransferState::Idle;    
                    i2c_interface->transfer_state.in_progress = false;
                    
                    // Call completion callback if registered
                    if (i2c_interface->callbacks[static_cast<uint32_t>(Platform::I2C::I2CEvent::TransferComplete)].enabled) {
                        i2c_interface->callbacks[static_cast<uint32_t>(Platform::I2C::I2CEvent::TransferComplete)].callback(
                            i2c_interface->callbacks[static_cast<uint32_t>(Platform::I2C::I2CEvent::TransferComplete)].param
                        );
                    }
                }
                else {
                    i2c->DR = i2c_interface->transfer_state.data_buffer[i2c_interface->transfer_state.data_index++]; 
                }
            }   
        }
    }
    // Receive interrupt (RXNE) for read operation
    else if (sr1 & Platform::I2C::getBitValue(Platform::I2C::SR1::RXNE)) {
        // Read the data and increment buffer index
        i2c_interface->transfer_state.data_buffer[i2c_interface->transfer_state.data_index++] = i2c->DR;
        
        // If all bytes received
        if(i2c_interface->transfer_state.data_index == i2c_interface->transfer_state.data_size) {
            if(i2c_interface->transfer_state.data_size > 1) {
                // Generate stop after receiving last byte 
                i2c->CR1 |= Platform::I2C::getBitValue(Platform::I2C::CR1::STOP);
            }
            
            // Disable interrupts
            i2c->CR2 &= ~(Platform::I2C::getBitValue(Platform::I2C::CR2::ITEVTEN) | 
                         Platform::I2C::getBitValue(Platform::I2C::CR2::ITERREN) | 
                         Platform::I2C::getBitValue(Platform::I2C::CR2::ITBUFEN));
            
            i2c_interface->transfer_state.state = Platform::I2C::CurrentTransferState::Idle;
            i2c_interface->transfer_state.in_progress = false;
            
            // Call completion callback if registered
            if (i2c_interface->callbacks[static_cast<uint32_t>(Platform::I2C::I2CEvent::TransferComplete)].enabled) {
                i2c_interface->callbacks[static_cast<uint32_t>(Platform::I2C::I2CEvent::TransferComplete)].callback(
                    i2c_interface->callbacks[static_cast<uint32_t>(Platform::I2C::I2CEvent::TransferComplete)].param
                );
            }
        }
        // If byte before last, send NACK
        else if(i2c_interface->transfer_state.data_index == i2c_interface->transfer_state.data_size - 1) {
            i2c->CR1 &= ~Platform::I2C::getBitValue(Platform::I2C::CR1::ACK);  // Disable ACK
        }
    }
}
void I2C2_EV_IRQHandler() {
    // Get I2C instance
    Platform::I2C::I2CInterface* i2c_interface = &Platform::I2C::I2CInterface::GetInstance(Platform::I2C::I2CInstance::I2C2);
    if (!i2c_interface || !i2c_interface->initialized) {
        return;
    }
    
    // Forward to the instance's event handler
    // Implement the interrupt handling logic here if needed
}

void I2C3_EV_IRQHandler() {
    // Get I2C instance
    Platform::I2C::I2CInterface* i2c_interface = &Platform::I2C::I2CInterface::GetInstance(Platform::I2C::I2CInstance::I2C3);
    if (!i2c_interface || !i2c_interface->initialized) {
        return;
    }
    
    // Forward to the instance's event handler
    // Implement the interrupt handling logic here if needed
}

// I2C interrupt handlers (Error)
void I2C1_ER_IRQHandler() {
    // Get I2C instance
    Platform::I2C::I2CInterface* i2c_interface = &Platform::I2C::I2CInterface::GetInstance(Platform::I2C::I2CInstance::I2C1);
    if (!i2c_interface || !i2c_interface->initialized) {
        return;
    }
    
    // Forward to the instance's error handler
    // Implement the error handling logic here if needed
}

void I2C2_ER_IRQHandler() {
    // Get I2C instance
    Platform::I2C::I2CInterface* i2c_interface = &Platform::I2C::I2CInterface::GetInstance(Platform::I2C::I2CInstance::I2C2);
    if (!i2c_interface || !i2c_interface->initialized) {
        return;
    }
    
    // Forward to the instance's error handler
    // Implement the error handling logic here if needed
}

void I2C3_ER_IRQHandler() {

    // Get I2C instance
    Platform::I2C::I2CInterface* i2c_interface = &Platform::I2C::I2CInterface::GetInstance(Platform::I2C::I2CInstance::I2C3);
    if (!i2c_interface || !i2c_interface->initialized) {
        return;
    }
    
    // Forward to the instance's error handler
    // Implement the error handling logic here if needed
}