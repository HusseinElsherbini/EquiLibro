// src/hardware_abstraction/i2c.c

#include "hardware_abstraction/i2c.h"
#include "hardware_abstraction/hw_interface.h"
#include "hardware_abstraction/gpio.h"
#include "common/error_codes.h"
#include "platform.h"
#include "FreeRTOS.h"
#include "task.h"
#include "common/cmsis_core_minimial.h"

// Static instances of I2C state for each interface
static I2C_State_t i2c_states[MAX_I2C_INSTANCES] = {0};

// Static instances of I2C hardware interface for each interface
static HW_Interface_t i2c_interfaces[MAX_I2C_INSTANCES];


// Helper functions
static I2C_Registers_t* I2C_GetRegisterBase(uint8_t i2c_instance);
static int I2C_GetInstanceIndex(uint8_t i2c_instance);
static Status_t I2C_EnableClock(uint8_t i2c_instance);
static Status_t I2C_ConfigureGPIO(I2C_Config_t *config);
static Status_t I2C_StartTransfer(I2C_State_t *i2c_state, I2C_Operation_t operation, uint8_t *buffer, uint16_t size, bool blocking, uint32_t timeout);
static void I2C_RecoverBus(I2C_State_t *i2c_state);
static Status_t I2C_Init(void *state, void *config);
static Status_t I2C_DeInit(void *state);
static Status_t I2C_Control(void *state, uint32_t command, void *param);
static Status_t I2C_Read(void *state, void *buffer, uint16_t size, uint32_t timeout);
static Status_t I2C_Write(void *state, const void *data, uint16_t size, uint32_t timeout);
static Status_t I2C_RegisterCallback(void *state, uint32_t eventId, void (*Callback)(void *param), void *param);
static Status_t I2C_PollTransfer(I2C_State_t *i2c_state, I2C_Operation_t operation, uint8_t *buffer, uint16_t size, uint32_t timeout);

// Map I2C instance to peripheral base address
static I2C_Registers_t* I2C_GetRegisterBase(uint8_t i2c_instance) {
    switch (i2c_instance) {
        case 1: return (I2C_Registers_t*)I2C1_BASE;
        case 2: return (I2C_Registers_t*)I2C2_BASE;
        case 3: return (I2C_Registers_t*)I2C3_BASE;
        default: return NULL;
    }
}

// Map I2C instance to state array index
static int I2C_GetInstanceIndex(uint8_t i2c_instance) {
    if (i2c_instance >= 1 && i2c_instance <= MAX_I2C_INSTANCES) {
        return i2c_instance - 1;
    }
    return -1;
}

// Enable clock to I2C peripheral
static Status_t I2C_EnableClock(uint8_t i2c_instance) {
    switch (i2c_instance) {
        case 1:
            SET_BIT(RCC_REGS->APB1ENR, RCC_APB1ENR_I2C1EN);
            break;
        case 2:
            SET_BIT(RCC_REGS->APB1ENR, RCC_APB1ENR_I2C2EN);
            break;
        case 3:
            SET_BIT(RCC_REGS->APB1ENR, RCC_APB1ENR_I2C3EN);
            break;
        default:
            return STATUS_INVALID_PARAM;
    }
    return STATUS_OK;
}

// Configure GPIO pins for I2C
static Status_t I2C_ConfigureGPIO(I2C_Config_t *config) {
    // Get the GPIO interface
    HW_Interface_t *gpio_interface = GPIO_GetInterface();
    if (gpio_interface == NULL) {
        return STATUS_ERROR;
    }
    
    // Define the I2C pin mapping structure
    typedef struct {
        uint8_t scl_port;
        uint8_t scl_pin;
        uint8_t sda_port;
        uint8_t sda_pin;
        GPIO_AlternateFunction_t af;
    } I2C_PinMapping_t;
    
    // Define pin mappings for each I2C instance
    static const I2C_PinMapping_t i2c_pins[] = {
        // I2C1
        {
            .scl_port = GPIOB,
            .scl_pin = 6,
            .sda_port = GPIOB,
            .sda_pin = 7,
            .af = GPIO_AF_4
        },
        // I2C2
        {
            .scl_port = GPIOB,
            .scl_pin = 10,
            .sda_port = GPIOB,
            .sda_pin = 11,
            .af = GPIO_AF_4
        },
        // I2C3
        {
            .scl_port = GPIOA,
            .scl_pin = 8,
            .sda_port = GPIOC,
            .sda_pin = 9,
            .af = GPIO_AF_4
        }
    };
    
    // Check if the I2C instance is valid
    if (config->i2c_instance < 1 || config->i2c_instance > 3) {
        return STATUS_INVALID_PARAM;
    }
    
    // Get the pin mapping for this I2C instance
    const I2C_PinMapping_t *pins = &i2c_pins[config->i2c_instance - 1];
    
    // Common configuration for I2C pins
    GPIO_Config_t pin_config = {
        .mode = GPIO_MODE_ALTERNATE,
        .outputType = GPIO_OUTPUT_OPEN_DRAIN,
        .pull = GPIO_PULL_UP,
        .speed = GPIO_SPEED_HIGH,
        .af = pins->af
    };
    
    // Configure SCL pin
    pin_config.port = pins->scl_port;
    pin_config.pin = pins->scl_pin;
    Status_t status = gpio_interface->Control(gpio_interface->state, GPIO_CTRL_CONFIG_PIN, &pin_config);
    if (status != STATUS_OK) {
        return status;
    }
    
    // Configure SDA pin
    pin_config.port = pins->sda_port;
    pin_config.pin = pins->sda_pin;
    status = gpio_interface->Control(gpio_interface->state, GPIO_CTRL_CONFIG_PIN, &pin_config);
    if (status != STATUS_OK) {
        return status;
    }
    
    return STATUS_OK;
}
// Implementation of I2C initialization
static Status_t I2C_Init(void *state, void *config) {

    I2C_State_t *i2c_state = (I2C_State_t*)state;
    I2C_Config_t *i2c_config = (I2C_Config_t*)config;
    if (i2c_config == NULL) {
        return STATUS_INVALID_PARAM;
    }
    
    int i2c_idx = I2C_GetInstanceIndex(i2c_config->i2c_instance);
    if (i2c_idx < 0) {
        return STATUS_INVALID_PARAM;
    }
    
    I2C_Registers_t *i2c = I2C_GetRegisterBase(i2c_config->i2c_instance);
    if (i2c == NULL) {
        return STATUS_INVALID_PARAM;
    }
    
    // Save the configuration
    i2c_state->config = *i2c_config;

    // Enable I2C clock
    Status_t status = I2C_EnableClock(i2c_config->i2c_instance);
    if (status != STATUS_OK) {
        return status;
    }
    
    // Configure GPIO pins for I2C
    status = I2C_ConfigureGPIO(i2c_config);
    if (status != STATUS_OK) {
        return status;
    }
    
    // Reset the I2C peripheral
    SET_BIT(i2c->I2C_CR1, (1 << 15)); // Set the SWRST bit
    __asm("nop"); // Small delay
    CLEAR_BIT(i2c->I2C_CR1, (1 << 15)); // Clear the SWRST bit
    
    // Set peripheral clock frequency in CR2 (assuming APB1 clock)
    MODIFY_REG(i2c->I2C_CR2, 0x3F, 42); // Set peripheral clock frequency (APB1)
    
    // Configure I2C speed
    uint32_t ccr_value = 0;
    
    if (i2c_config->speed == I2C_SPEED_STANDARD) {
        // Standard mode (100 KHz)
        ccr_value = 42 * 5; // PCLK1(MHz) / (100KHz * 2)
    } else {
        // Fast mode (400 KHz)
        SET_BIT(i2c->I2C_CCR, (1 << 15)); // Set the F/S bit for fast mode
        SET_BIT(i2c->I2C_CCR, (1 << 14)); // Set the DUTY bit for Tlow/Thigh = 16/9
        ccr_value = 42 * 25 / 10; // PCLK1(MHz) * 25 / 10
    }
    
    // Set CCR value
    MODIFY_REG(i2c->I2C_CCR, 0xFFF, ccr_value);
    
    // Configure TRISE
    if (i2c_config->speed == I2C_SPEED_STANDARD) {
        i2c->I2C_TRISE = 43; // Maximum rise time in standard mode
    } else {
        i2c->I2C_TRISE = 43 * 300 / 1000 + 1; // Maximum rise time in fast mode
    }
    
    // Configure addressing mode and own address (for slave mode)
    CLEAR_BIT(i2c->I2C_OAR1, (1 << 15)); // Clear the ADDMODE bit by default
    
    if (i2c_config->addr_mode == I2C_ADDR_10BIT) {
        SET_BIT(i2c->I2C_OAR1, (1 << 15)); // Set the ADDMODE bit for 10-bit addressing
    }
    
    // Set own address (always set bit 14 per datasheet)
    MODIFY_REG(i2c->I2C_OAR1, 0x3FF, i2c_config->own_address | (1 << 14));
    
    // Enable the I2C peripheral
    SET_BIT(i2c->I2C_CR1, (1 << 0)); // Set the PE bit
    
    // Enable interrupts if needed
    if (i2c_config->enable_interrupts) {
        // Set the interrupt priority
        NVIC_SetPriority(I2C1_EV_IRQn + (i2c_config->i2c_instance - 1) * 2, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
        NVIC_SetPriority(I2C1_ER_IRQn + (i2c_config->i2c_instance - 1) * 2, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
        
        // Enable the I2C event and error interrupts
        NVIC_EnableIRQ(I2C1_EV_IRQn + (i2c_config->i2c_instance - 1) * 2);
        NVIC_EnableIRQ(I2C1_ER_IRQn + (i2c_config->i2c_instance - 1) * 2);
    }
    
    // Initialize transfer state
    i2c_states[i2c_idx].current_transfer.state = I2C_STATE_IDLE;
    i2c_states[i2c_idx].current_transfer.operation = I2C_OP_NONE;
    
    // Mark as initialized
    i2c_state->initialized = true;
    
    return STATUS_OK;
}

// Implementation of I2C de-initialization
static Status_t I2C_DeInit(void *state) {

    I2C_State_t *i2c_state = (I2C_State_t*)state;
    I2C_Config_t *i2c_config = &i2c_state->config;

    int i2c_idx = I2C_GetInstanceIndex(i2c_config->i2c_instance);
    
    if (!i2c_state->initialized) {
        return STATUS_NOT_INITIALIZED;
    }
    
    I2C_Registers_t *i2c = I2C_GetRegisterBase(i2c_config->i2c_instance);
    if (i2c == NULL) {
        return STATUS_ERROR;
    }
    
    // Disable the I2C peripheral
    CLEAR_BIT(i2c->I2C_CR1, (1 << 0)); // Clear the PE bit
    
    // Disable interrupts if they were enabled
    if (i2c_config->enable_interrupts) {
        NVIC_DisableIRQ(I2C1_EV_IRQn + (i2c_config->i2c_instance - 1) * 2);
        NVIC_DisableIRQ(I2C1_ER_IRQn + (i2c_config->i2c_instance - 1) * 2);
    }
    
    // Mark as not initialized
    i2c_state->initialized = false;
    
    return STATUS_OK;
}

// Implementation of I2C bus recovery
static void I2C_RecoverBus(I2C_State_t *i2c_state) {
    I2C_Registers_t *i2c = I2C_GetRegisterBase(i2c_state->config.i2c_instance);
    
    // Get the GPIO interface
    HW_Interface_t *gpio_interface = GPIO_GetInterface();
    if (gpio_interface == NULL) {
        return; // Cannot recover without GPIO interface
    }
    
    // Determine which pins to use based on I2C instance
    uint8_t scl_port, scl_pin, sda_port, sda_pin;
    
    switch (i2c_state->config.i2c_instance) {
        case 1:
            scl_port = GPIOB;
            scl_pin = 6;
            sda_port = GPIOB;
            sda_pin = 7;
            break;
        case 2:
            scl_port = GPIOB;
            scl_pin = 10;
            sda_port = GPIOB;
            sda_pin = 11;
            break;
        case 3:
            scl_port = GPIOA;
            scl_pin = 8;
            sda_port = GPIOC;
            sda_pin = 9;
            break;
        default:
            return;
    }
    
    // 1. Disable I2C peripheral
    CLEAR_BIT(i2c->I2C_CR1, (1 << 0));
    
    // 2. Configure SCL as output
    GPIO_Config_t scl_config = {
        .port = scl_port,
        .pin = scl_pin,
        .mode = GPIO_MODE_OUTPUT,
        .outputType = GPIO_OUTPUT_OPEN_DRAIN,
        .pull = GPIO_PULL_UP,
        .speed = GPIO_SPEED_HIGH
    };
    
    GPIO_Config_t sda_config = {
        .port = sda_port,
        .pin = sda_pin,
        .mode = GPIO_MODE_OUTPUT,
        .outputType = GPIO_OUTPUT_OPEN_DRAIN,
        .pull = GPIO_PULL_UP,
        .speed = GPIO_SPEED_HIGH
    };
    
    // Configure SCL as output
    gpio_interface->Control(gpio_interface->state, GPIO_CTRL_CONFIG_PIN, &scl_config);
    
    // 3. Toggle SCL to force slave to release SDA
    // I2C bus typically operates at 100KHz or 400KHz
    // For 100KHz, one clock cycle is 10µs (5µs high, 5µs low)
    for (int i = 0; i < 9; i++) {
        // Pull SCL high
        gpio_interface->Control(gpio_interface->state, GPIO_CTRL_SET_PIN, &scl_config);
        Delay_Microseconds(5, true); // 5µs delay (high period)
        
        // Pull SCL low
        gpio_interface->Control(gpio_interface->state, GPIO_CTRL_RESET_PIN, &scl_config);
        Delay_Microseconds(5, true); // 5µs delay (low period)
    }
    
    // 4. Generate STOP condition manually
    // Configure SDA as output
    gpio_interface->Control(gpio_interface->state, GPIO_CTRL_CONFIG_PIN, &sda_config);
    
    // Pull SDA low while SCL is low
    gpio_interface->Control(gpio_interface->state, GPIO_CTRL_RESET_PIN, &sda_config);
    Delay_Microseconds(5, true);
    
    // Pull SCL high
    gpio_interface->Control(gpio_interface->state, GPIO_CTRL_SET_PIN, &scl_config);
    Delay_Microseconds(5, true);
    
    // Pull SDA high while SCL is high (STOP condition)
    gpio_interface->Control(gpio_interface->state, GPIO_CTRL_SET_PIN, &sda_config);
    Delay_Microseconds(5, true);
    
    // 5. Restore I2C configuration
    // Reset both pins to AF mode
    scl_config.mode = GPIO_MODE_ALTERNATE;
    scl_config.af = GPIO_AF_4; // I2C alternate function
    sda_config.mode = GPIO_MODE_ALTERNATE;
    sda_config.af = GPIO_AF_4; // I2C alternate function
    
    gpio_interface->Control(gpio_interface->state, GPIO_CTRL_CONFIG_PIN, &scl_config);
    gpio_interface->Control(gpio_interface->state, GPIO_CTRL_CONFIG_PIN, &sda_config);
    
    // 6. Re-enable I2C peripheral
    SET_BIT(i2c->I2C_CR1, (1 << 0));
}

// Function to start a transfer
static Status_t I2C_StartTransfer(I2C_State_t *i2c_state, I2C_Operation_t operation, uint8_t *buffer, uint16_t size, bool blocking, uint32_t timeout) {
    I2C_Registers_t *i2c = I2C_GetRegisterBase(i2c_state->config.i2c_instance);
    
    // Check if a transfer is already in progress
    if (i2c_state->current_transfer.state != I2C_STATE_IDLE) {
        return STATUS_BUSY;
    }
    
    // Set up transfer
    i2c_state->current_transfer.buffer = buffer;
    i2c_state->current_transfer.size = size;
    i2c_state->current_transfer.index = 0;
    i2c_state->current_transfer.operation = operation;
    i2c_state->current_transfer.state = I2C_STATE_START_SENT;
    
    // Check if the scheduler is running
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        // Scheduler is running - we can use FreeRTOS APIs
        
        // For blocking transfers, set up task notification
        if (blocking) {
            i2c_state->current_transfer.notify_task = xTaskGetCurrentTaskHandle();
        } else {
            i2c_state->current_transfer.notify_task = NULL;
        }
        
        // Enable interrupts if they're configured
        if (i2c_state->config.enable_interrupts) {
            SET_BIT(i2c->I2C_CR2, (1 << 9) | (1 << 10)); // Event and buffer interrupts
            SET_BIT(i2c->I2C_CR2, (1 << 8)); // Error interrupt
        }
        
        // Start the transfer
        SET_BIT(i2c->I2C_CR1, (1 << 8)); // Send START
        
        // For blocking transfers, wait for completion
        if (blocking) {
            uint32_t notification;
            if (xTaskNotifyWait(0, 0xFFFFFFFF, &notification, pdMS_TO_TICKS(timeout)) == pdTRUE) {
                if (notification == 1) {
                    return STATUS_OK;
                } else {
                    return STATUS_ERROR;
                }
            } else {
                // Timeout occurred
                i2c_state->current_transfer.state = I2C_STATE_IDLE;
                return STATUS_TIMEOUT;
            }
        }
    } else {
        // Scheduler is not running - must use polling
        return I2C_PollTransfer(i2c_state, operation, buffer, size, timeout);
    }
    
    return STATUS_OK;
}

/**
 * @brief  Perform an I2C transfer in polling mode with hardware timer-based delays
 * @param  i2c_state: Pointer to I2C state structure
 * @param  operation: Operation type (read or write)
 * @param  buffer: Pointer to data buffer
 * @param  size: Size of data to transfer
 * @param  timeout: Timeout in milliseconds
 * @retval Status code
 */
static Status_t I2C_PollTransfer(I2C_State_t *i2c_state, I2C_Operation_t operation, 
                               uint8_t *buffer, uint16_t size, uint32_t timeout) {
    I2C_Registers_t *i2c = I2C_GetRegisterBase(i2c_state->config.i2c_instance);
    I2C_Transfer_t *transfer = &i2c_state->current_transfer;
    
    // Calculate absolute timeout point using our delay service
    uint64_t timeout_point = SysTick_CalculateTimeout(timeout);
    
    // Update transfer state
    transfer->buffer = buffer;
    transfer->size = size;
    transfer->index = 0;
    transfer->operation = operation;
    transfer->state = I2C_STATE_START_SENT;
    
    // Before starting a new transfer, make sure the bus is free
    while (i2c->I2C_SR2 & (1 << 1)) { // Check BUSY flag
        if (SysTick_HasTimeoutOccurred(timeout_point)) {
            transfer->state = I2C_STATE_IDLE;
            return STATUS_TIMEOUT;
        }
    }
    
    // Generate START condition
    SET_BIT(i2c->I2C_CR1, (1 << 8)); // Set the START bit
    
    // Wait for the START condition to be generated (SB flag)
    while (!(i2c->I2C_SR1 & (1 << 0))) {
        if (SysTick_HasTimeoutOccurred(timeout_point)) {
            transfer->state = I2C_STATE_IDLE;
            return STATUS_TIMEOUT;
        }
    }
    
    // Send slave address with write bit (even for read operations, we first write the register address)
    i2c->I2C_DR = (transfer->dev_addr << 1) | 0; // LSB = 0 for write
    transfer->state = I2C_STATE_ADDR_SENT_W;
    
    // Wait for address to be sent (ADDR flag)
    while (!(i2c->I2C_SR1 & (1 << 1))) {
        // Check for acknowledge failure (AF flag)
        if (i2c->I2C_SR1 & (1 << 10)) {
            // Generate STOP condition
            SET_BIT(i2c->I2C_CR1, (1 << 9));
            // Clear AF flag
            CLEAR_BIT(i2c->I2C_SR1, (1 << 10));
            
            transfer->state = I2C_STATE_IDLE;
            return STATUS_ERROR;
        }
        
        if (SysTick_HasTimeoutOccurred(timeout_point)) {
            transfer->state = I2C_STATE_IDLE;
            return STATUS_TIMEOUT;
        }
    }
    
    // Clear ADDR flag by reading SR1 and SR2
    (void)i2c->I2C_SR1;
    (void)i2c->I2C_SR2;
    
    // Send register address
    i2c->I2C_DR = transfer->reg_addr;
    
    // Wait for the register address to be sent (TXE flag)
    while (!(i2c->I2C_SR1 & (1 << 7))) {
        if (SysTick_HasTimeoutOccurred(timeout_point)) {
            transfer->state = I2C_STATE_IDLE;
            return STATUS_TIMEOUT;
        }
    }
    
    // If it's a read operation, send a repeated START
    if (operation == I2C_OP_READ) {
        transfer->state = I2C_STATE_REPEATED_START_SENT;
        
        // Generate repeated START condition
        SET_BIT(i2c->I2C_CR1, (1 << 8));
        
        // Wait for the START condition to be generated (SB flag)
        while (!(i2c->I2C_SR1 & (1 << 0))) {
            if (SysTick_HasTimeoutOccurred(timeout_point)) {
                transfer->state = I2C_STATE_IDLE;
                return STATUS_TIMEOUT;
            }
        }
        
        // Send slave address with read bit
        i2c->I2C_DR = (transfer->dev_addr << 1) | 1; // LSB = 1 for read
        transfer->state = I2C_STATE_ADDR_SENT_R;
        
        // Wait for address to be sent (ADDR flag)
        while (!(i2c->I2C_SR1 & (1 << 1))) {
            // Check for acknowledge failure (AF flag)
            if (i2c->I2C_SR1 & (1 << 10)) {
                // Generate STOP condition
                SET_BIT(i2c->I2C_CR1, (1 << 9));
                // Clear AF flag
                CLEAR_BIT(i2c->I2C_SR1, (1 << 10));
                
                transfer->state = I2C_STATE_IDLE;
                return STATUS_ERROR;
            }
            
            if (SysTick_HasTimeoutOccurred(timeout_point)) {
                transfer->state = I2C_STATE_IDLE;
                return STATUS_TIMEOUT;
            }
        }
        
        // Handle different read sizes
        if (size == 1) {
            // For single byte read:
            
            // 1. Disable ACK before clearing ADDR
            CLEAR_BIT(i2c->I2C_CR1, (1 << 10));
            
            // 2. Clear ADDR flag
            (void)i2c->I2C_SR1;
            (void)i2c->I2C_SR2;
            
            // 3. Set STOP bit
            SET_BIT(i2c->I2C_CR1, (1 << 9));
            
            // 4. Wait for RXNE flag
            while (!(i2c->I2C_SR1 & (1 << 6))) {
                if (SysTick_HasTimeoutOccurred(timeout_point)) {
                    transfer->state = I2C_STATE_IDLE;
                    return STATUS_TIMEOUT;
                }
            }
            
            // 5. Read the data byte
            *buffer = i2c->I2C_DR;
            
        } else if (size == 2) {
            // For two-byte read:
            
            // 1. Enable POS bit and disable ACK before clearing ADDR
            SET_BIT(i2c->I2C_CR1, (1 << 11)); // POS bit
            CLEAR_BIT(i2c->I2C_CR1, (1 << 10)); // ACK bit
            
            // 2. Clear ADDR flag
            (void)i2c->I2C_SR1;
            (void)i2c->I2C_SR2;
            
            // 3. Wait for BTF flag (both bytes received)
            while (!(i2c->I2C_SR1 & (1 << 2))) {
                if (SysTick_HasTimeoutOccurred(timeout_point)) {
                    transfer->state = I2C_STATE_IDLE;
                    return STATUS_TIMEOUT;
                }
            }
            
            // 4. Set STOP bit
            SET_BIT(i2c->I2C_CR1, (1 << 9));
            
            // 5. Read both data bytes
            buffer[0] = i2c->I2C_DR;
            buffer[1] = i2c->I2C_DR;
            
        } else {
            // For multi-byte read (more than 2 bytes)
            
            // 1. Enable ACK before clearing ADDR
            SET_BIT(i2c->I2C_CR1, (1 << 10));
            
            // 2. Clear ADDR flag
            (void)i2c->I2C_SR1;
            (void)i2c->I2C_SR2;
            
            // 3. Read all bytes except the last two
            for (uint16_t i = 0; i < size - 3; i++) {
                // Wait for RXNE flag
                while (!(i2c->I2C_SR1 & (1 << 6))) {
                    if (SysTick_HasTimeoutOccurred(timeout_point)) {
                        transfer->state = I2C_STATE_IDLE;
                        return STATUS_TIMEOUT;
                    }
                }
                
                // Read data byte
                buffer[i] = i2c->I2C_DR;
                transfer->index = i + 1;
            }
            
            // 4. Wait for BTF flag (N-2 byte received, N-1 in DR, N in shift register)
            while (!(i2c->I2C_SR1 & (1 << 2))) {
                if (SysTick_HasTimeoutOccurred(timeout_point)) {
                    transfer->state = I2C_STATE_IDLE;
                    return STATUS_TIMEOUT;
                }
            }
            
            // 5. Disable ACK
            CLEAR_BIT(i2c->I2C_CR1, (1 << 10));
            
            // 6. Read N-2 byte
            buffer[size - 3] = i2c->I2C_DR;
            
            // 7. Wait for BTF flag (N-1 byte received, N in shift register)
            while (!(i2c->I2C_SR1 & (1 << 2))) {
                if (SysTick_HasTimeoutOccurred(timeout_point)) {
                    transfer->state = I2C_STATE_IDLE;
                    return STATUS_TIMEOUT;
                }
            }
            
            // 8. Set STOP bit
            SET_BIT(i2c->I2C_CR1, (1 << 9));
            
            // 9. Read the last two bytes
            buffer[size - 2] = i2c->I2C_DR;
            buffer[size - 1] = i2c->I2C_DR;
        }
        
    } else {
        // WRITE OPERATIONS
        
        // Skip the register address that was already sent
        transfer->state = I2C_STATE_WRITING;
        
        // Send data bytes
        for (uint16_t i = 0; i < size; i++) {
            // Wait for TXE flag
            while (!(i2c->I2C_SR1 & (1 << 7))) {
                if (SysTick_HasTimeoutOccurred(timeout_point)) {
                    transfer->state = I2C_STATE_IDLE;
                    return STATUS_TIMEOUT;
                }
            }
            
            // Send data byte
            i2c->I2C_DR = buffer[i];
            transfer->index = i + 1;
            
            // Check for acknowledge failure (AF flag)
            if (i2c->I2C_SR1 & (1 << 10)) {
                // Generate STOP condition
                SET_BIT(i2c->I2C_CR1, (1 << 9));
                // Clear AF flag
                CLEAR_BIT(i2c->I2C_SR1, (1 << 10));
                
                transfer->state = I2C_STATE_IDLE;
                return STATUS_ERROR;
            }
        }
        
        // Wait for BTF flag (all bytes transmitted)
        while (!(i2c->I2C_SR1 & (1 << 2))) {
            if (SysTick_HasTimeoutOccurred(timeout_point)) {
                transfer->state = I2C_STATE_IDLE;
                return STATUS_TIMEOUT;
            }
        }
        
        // Generate STOP condition
        SET_BIT(i2c->I2C_CR1, (1 << 9));
    }
    
    // Wait for the STOP condition to be completed
    while (i2c->I2C_CR1 & (1 << 9)) {
        if (SysTick_HasTimeoutOccurred(timeout_point)) {
            // This is not a critical error, so just break
            break;
        }
    }
    
    // Reset transfer state
    transfer->state = I2C_STATE_IDLE;
    
    return STATUS_OK;
}
// Implementation of I2C control function
static Status_t I2C_Control(void *state, uint32_t command, void *param) {

    I2C_State_t *i2c_state = (I2C_State_t*)state;
    I2C_Config_t *i2c_config = &i2c_state->config;

    int i2c_idx = I2C_GetInstanceIndex(i2c_config->i2c_instance);
    
    if (i2c_idx < 0 || !i2c_state->initialized) {
        return STATUS_NOT_INITIALIZED;
    }
    
    if (param == NULL && command != I2C_CTRL_RECOVER_BUS) {
        return STATUS_INVALID_PARAM;
    }
    
    // Common variables used by multiple cases
    I2C_Transfer_t *transfer;
    bool blocking;
    uint32_t timeout;
    Status_t status = STATUS_OK;
    
    // Process the command
    switch (command) {
        case I2C_CTRL_MASTER_TRANSMIT:
        case I2C_CTRL_MASTER_RECEIVE:
            // Common transfer setup for both transmit and receive
            transfer = (I2C_Transfer_t *)param;
            
            // Use the explicit blocking flag from the transfer structure
            blocking = transfer->blocking;
            timeout = transfer->timeout;
            
            // Store the notification task and value in the I2C state
            i2c_state->current_transfer.notify_task = transfer->notify_task;
            i2c_state->current_transfer.notification_value = transfer->notification_value;
            
            // Determine operation type based on command
            I2C_Operation_t operation = (command == I2C_CTRL_MASTER_TRANSMIT) ? 
                                       I2C_OP_WRITE : I2C_OP_READ;
            
            // Start the transfer with appropriate operation type
            status = I2C_StartTransfer(i2c_state, operation, 
                                    transfer->buffer, transfer->size, 
                                    blocking, timeout);
            break;
            
        case I2C_CTRL_MEM_WRITE:
        case I2C_CTRL_MEM_READ:
            // Memory read/write operations using register address
            typedef struct {
                uint16_t mem_addr;
                uint8_t *data;
                uint16_t size;
                bool blocking;
                uint32_t timeout;
            } I2C_MemTransfer_t;
            
            I2C_MemTransfer_t *mem_transfer = (I2C_MemTransfer_t *)param;
            
            // Set the register address for the transfer
            i2c_state->current_transfer.reg_addr = mem_transfer->mem_addr;
            
            // Determine operation type based on command
            I2C_Operation_t mem_operation = (command == I2C_CTRL_MEM_WRITE) ? 
                                          I2C_OP_WRITE : I2C_OP_READ;
            
            // Start the transfer
            status = I2C_StartTransfer(i2c_state, mem_operation, 
                                    mem_transfer->data, mem_transfer->size, 
                                    mem_transfer->blocking, mem_transfer->timeout);
            break;
            
        case I2C_CTRL_SET_TARGET_ADDR:
            // Set the I2C device address for subsequent transfers
            uint16_t *addr = (uint16_t *)param;
            i2c_state->current_transfer.dev_addr = *addr;
            status = STATUS_OK;
            break;
            
        case I2C_CTRL_RECOVER_BUS:
            // Recover the I2C bus in case of errors
            I2C_RecoverBus(i2c_state);
            status = STATUS_OK;
            break;
            
        default:
            status = STATUS_NOT_SUPPORTED;
            break;
    }
    return status;
}

// Implementation of I2C read function
static Status_t I2C_Read(void *state, void *buffer, uint16_t size, uint32_t timeout) {

    I2C_State_t *i2c_state = (I2C_State_t*)state;
    I2C_Config_t *i2c_config = &i2c_state->config;

    if (!i2c_state->initialized) {
        return STATUS_NOT_INITIALIZED;
    }
    
    if (buffer == NULL || size < 2) { // Need at least register address and one data byte
        return STATUS_INVALID_PARAM;
    }
    
    // First byte is register address
    i2c_state->current_transfer.reg_addr = *((uint8_t*)buffer);
    
    bool blocking = (timeout > 0);
    return I2C_StartTransfer(i2c_state, I2C_OP_READ, (uint8_t*)buffer + 1, size - 1, blocking, timeout);
}

// Implementation of I2C write function
static Status_t I2C_Write(void *state, const void *data, uint16_t size, uint32_t timeout) {

    I2C_State_t *i2c_state = (I2C_State_t*)state;
    I2C_Config_t *i2c_config = &i2c_state->config;

 
    if (!i2c_state->initialized) {
        return STATUS_NOT_INITIALIZED;
    }
    
    if (data == NULL || size < 2) { // Need at least register address and one data byte
        return STATUS_INVALID_PARAM;
    }
    
    // First byte is register address
    i2c_state->current_transfer.reg_addr = *((uint8_t*)data);
    
    bool blocking = (timeout > 0);
    return I2C_StartTransfer(i2c_state, I2C_OP_WRITE, (uint8_t*)data, size, blocking, timeout);
}

// Implementation of I2C callback registration
static Status_t I2C_RegisterCallback(void *state, uint32_t eventId, void (*Callback)(void *param), void *param) {
    
    I2C_State_t *i2c_state = (I2C_State_t*)state;
    I2C_Config_t *i2c_config = &i2c_state->config;
    
    if (!i2c_state->initialized) {
        return STATUS_NOT_INITIALIZED;
    }
    
    if (eventId >= I2C_CALLBACK_MAX) {
        return STATUS_INVALID_PARAM;
    }
    
    // Register the callback
    i2c_state->callbacks[eventId] = Callback;
    i2c_state->callback_params[eventId] = param;
    
    return STATUS_OK;
}

// I2C interrupt handler for events
void I2C_EV_IRQHandler(uint8_t i2c_instance) {
    int i2c_idx = I2C_GetInstanceIndex(i2c_instance);
    if (i2c_idx < 0 || !i2c_states[i2c_idx].initialized) {
        return;
    }
    
    I2C_Registers_t *i2c = I2C_GetRegisterBase(i2c_instance);
    I2C_State_t *i2c_state = &i2c_states[i2c_idx];
    I2C_Transfer_t *transfer = &i2c_state->current_transfer;
    BaseType_t higher_priority_task_woken = pdFALSE;
    
    uint32_t sr1 = i2c->I2C_SR1;
    uint32_t sr2 = i2c->I2C_SR2;
    
    // Start condition generated
    if (sr1 & (1 << 0)) { // SB bit
        if (transfer->state == I2C_STATE_REPEATED_START_SENT) {
            // For repeated start in read operation, send slave address with read bit
            i2c->I2C_DR = (transfer->dev_addr << 1) | 1;
            transfer->state = I2C_STATE_ADDR_SENT_R;
        } else {
            // For normal start, send slave address with write bit
            // For normal start, send slave address with write bit
           i2c->I2C_DR = (transfer->dev_addr << 1) | 0;
           transfer->state = I2C_STATE_ADDR_SENT_W;
       }
   }
   
   // Address sent
   else if (sr1 & (1 << 1)) { // ADDR bit
       if (transfer->state == I2C_STATE_ADDR_SENT_W) {
           // Clear ADDR flag by reading SR1 and SR2
           (void)i2c->I2C_SR1;
           (void)i2c->I2C_SR2;
           
           // Send register address
           i2c->I2C_DR = transfer->reg_addr;
           
           if (transfer->operation == I2C_OP_READ) {
               // For read operations, we'll need to send a repeated start after this
               transfer->state = I2C_STATE_REG_ADDR_SENT;
               transfer->repeated_start = true;
           } else {
               // For write operations, we'll start sending data
               transfer->state = I2C_STATE_WRITING;
               transfer->index = 1; // Skip register address which is at index 0
           }
       } 
       else if (transfer->state == I2C_STATE_ADDR_SENT_R) {
           // Read operation with address sent
           
           if (transfer->size == 1) {
               // For single byte read, disable ACK before clearing ADDR
               CLEAR_BIT(i2c->I2C_CR1, (1 << 10)); // Disable ACK
               
               // Clear ADDR flag
               (void)i2c->I2C_SR1;
               (void)i2c->I2C_SR2;
               
               // Immediately set STOP bit
               SET_BIT(i2c->I2C_CR1, (1 << 9));
           } 
           else {
               // For multi-byte read, clear ADDR first
               (void)i2c->I2C_SR1;
               (void)i2c->I2C_SR2;
               
               // Enable ACK for receiving multiple bytes
               SET_BIT(i2c->I2C_CR1, (1 << 10));
           }
           
           transfer->state = I2C_STATE_READING;
       }
   }
   
   // Transmit buffer empty (TXE)
   else if (sr1 & (1 << 7)) {
       if (transfer->state == I2C_STATE_REG_ADDR_SENT) {
           // After sending register address for read operation, generate repeated start
           SET_BIT(i2c->I2C_CR1, (1 << 8)); // Generate start
           transfer->state = I2C_STATE_REPEATED_START_SENT;
       } 
       else if (transfer->state == I2C_STATE_WRITING) {
           if (transfer->index < transfer->size) {
               // Send the next data byte
               i2c->I2C_DR = transfer->buffer[transfer->index++];
           } 
           else {
               // All data sent, generate STOP
               SET_BIT(i2c->I2C_CR1, (1 << 9));
               
               // Transfer complete
               transfer->state = I2C_STATE_IDLE;
               
               // Disable interrupts
               CLEAR_BIT(i2c->I2C_CR2, (1 << 9) | (1 << 10) | (1 << 8));
               
               // Call callback if registered
               if (i2c_state->callbacks[I2C_CALLBACK_TRANSFER_COMPLETE]) {
                   i2c_state->callbacks[I2C_CALLBACK_TRANSFER_COMPLETE](i2c_state->callback_params[I2C_CALLBACK_TRANSFER_COMPLETE]);
               }
               
               // Notify waiting task if any
               if (transfer->notify_task != NULL) {
                   xTaskNotifyFromISR(transfer->notify_task, 1, eSetValueWithOverwrite, &higher_priority_task_woken);
                   portYIELD_FROM_ISR(higher_priority_task_woken);
               }
           }
       }
   }
   
   // Receive buffer not empty (RXNE)
   else if (sr1 & (1 << 6)) {
       if (transfer->state == I2C_STATE_READING) {
           // Read the received byte
           transfer->buffer[transfer->index++] = i2c->I2C_DR;
           
           if (transfer->index == transfer->size - 1) {
               // One byte left to read, prepare to send NACK
               CLEAR_BIT(i2c->I2C_CR1, (1 << 10)); // Disable ACK
               // Prepare to send STOP after the next byte
               SET_BIT(i2c->I2C_CR1, (1 << 9));
           } 
           else if (transfer->index == transfer->size) {
               // Last byte read, transfer complete
               transfer->state = I2C_STATE_IDLE;
               
               // Disable interrupts
               CLEAR_BIT(i2c->I2C_CR2, (1 << 9) | (1 << 10) | (1 << 8));
               
               // Call callback if registered
               if (i2c_state->callbacks[I2C_CALLBACK_TRANSFER_COMPLETE]) {
                   i2c_state->callbacks[I2C_CALLBACK_TRANSFER_COMPLETE](i2c_state->callback_params[I2C_CALLBACK_TRANSFER_COMPLETE]);
               }
               
               // Notify waiting task if any
               if (transfer->notify_task != NULL) {
                   xTaskNotifyFromISR(transfer->notify_task, 1, eSetValueWithOverwrite, &higher_priority_task_woken);
                   portYIELD_FROM_ISR(higher_priority_task_woken);
               }
           }
       }
   }
}

// I2C interrupt handler for errors
void I2C_ER_IRQHandler(uint8_t i2c_instance) {
   int i2c_idx = I2C_GetInstanceIndex(i2c_instance);
   if (i2c_idx < 0 || !i2c_states[i2c_idx].initialized) {
       return;
   }
   
   I2C_Registers_t *i2c = I2C_GetRegisterBase(i2c_instance);
   I2C_State_t *i2c_state = &i2c_states[i2c_idx];
   I2C_Transfer_t *transfer = &i2c_state->current_transfer;
   BaseType_t higher_priority_task_woken = pdFALSE;
   
   uint32_t sr1 = i2c->I2C_SR1;
   
   // Bus error (BERR)
   if (sr1 & (1 << 8)) {
       // Clear the flag
       CLEAR_BIT(i2c->I2C_SR1, (1 << 8));
   }
   
   // Arbitration lost error (ARLO)
   if (sr1 & (1 << 9)) {
       // Clear the flag
       CLEAR_BIT(i2c->I2C_SR1, (1 << 9));
   }
   
   // Acknowledge failure (AF)
   if (sr1 & (1 << 10)) {
       // Clear the flag
       CLEAR_BIT(i2c->I2C_SR1, (1 << 10));
       
       // Generate STOP
       SET_BIT(i2c->I2C_CR1, (1 << 9));
   }
   
   // Overrun/Underrun error (OVR)
   if (sr1 & (1 << 11)) {
       // Clear the flag
       CLEAR_BIT(i2c->I2C_SR1, (1 << 11));
   }
   
   // Timeout error (TIMEOUT)
   if (sr1 & (1 << 14)) {
       // Clear the flag
       CLEAR_BIT(i2c->I2C_SR1, (1 << 14));
   }
   
   // Mark transfer as error
   transfer->state = I2C_STATE_ERROR;
   
   // Disable interrupts
   CLEAR_BIT(i2c->I2C_CR2, (1 << 9) | (1 << 10) | (1 << 8));
   
   // Try to recover the bus
   I2C_RecoverBus(i2c_state);
   
   // Call error callback if registered
   if (i2c_state->callbacks[I2C_CALLBACK_ERROR]) {
       i2c_state->callbacks[I2C_CALLBACK_ERROR](i2c_state->callback_params[I2C_CALLBACK_ERROR]);
   }
   
   // Notify waiting task if any (with error status)
   if (transfer->notify_task != NULL) {
       xTaskNotifyFromISR(transfer->notify_task, 0, eSetValueWithOverwrite, &higher_priority_task_woken);
       portYIELD_FROM_ISR(higher_priority_task_woken);
   }
   
   // Reset transfer state
   transfer->state = I2C_STATE_IDLE;
   transfer->operation = I2C_OP_NONE;
}

// I2C1 event interrupt handler
void I2C1_EV_IRQHandler_Wrapper(void) {
   I2C_EV_IRQHandler(1);
}

// I2C1 error interrupt handler
void I2C1_ER_IRQHandler_Wrapper(void) {
   I2C_ER_IRQHandler(1);
}

// I2C2 event interrupt handler
void I2C2_EV_IRQHandler_Wrapper(void) {
   I2C_EV_IRQHandler(2);
}

// I2C2 error interrupt handler
void I2C2_ER_IRQHandler_Wrapper(void) {
   I2C_ER_IRQHandler(2);
}

// I2C3 event interrupt handler
void I2C3_EV_IRQHandler_Wrapper(void) {
   I2C_EV_IRQHandler(3);
}

// I2C3 error interrupt handler
void I2C3_ER_IRQHandler_Wrapper(void) {
   I2C_ER_IRQHandler(3);
}

// Get the I2C hardware interface
HW_Interface_t* I2C_GetInterface(uint8_t i2c_instance) {
   int i2c_idx = I2C_GetInstanceIndex(i2c_instance);
   if (i2c_idx < 0) {
       return NULL;
   }
   
   // Initialize interface function pointers for this I2C instance
   HW_Interface_t *interface = &i2c_interfaces[i2c_idx];
   interface->state = &i2c_states[i2c_idx];
   interface->Init = I2C_Init;
   interface->DeInit = I2C_DeInit;
   interface->Control = I2C_Control;
   interface->Read = I2C_Read;
   interface->Write = I2C_Write;
   interface->RegisterCallback = I2C_RegisterCallback;
   
   return interface;
}