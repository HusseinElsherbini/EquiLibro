#ifndef HARDWARE_ABSTRACTION_I2C_H
#define HARDWARE_ABSTRACTION_I2C_H

#include "hw_interface.h"
#include "common/types.h"
#include "common/error_codes.h"
#include "FreeRTOS.h"
#include "task.h"

// Maximum number of I2C instances
#define MAX_I2C_INSTANCES 3

// I2C addressing mode
typedef enum {
    I2C_ADDR_7BIT,
    I2C_ADDR_10BIT
} I2C_AddrMode_t;

// I2C speed enumeration
typedef enum {
    I2C_SPEED_STANDARD,   // 100 KHz
    I2C_SPEED_FAST        // 400 KHz
} I2C_Speed_t;

// I2C mode enumeration
typedef enum {
    I2C_MODE_MASTER,
    I2C_MODE_SLAVE
} I2C_Mode_t;


// I2C configuration structure
typedef struct {
    uint8_t i2c_instance;     // I2C instance number (1-3)
    I2C_Mode_t mode;          // Master or slave mode
    I2C_Speed_t speed;        // Speed mode
    I2C_AddrMode_t addr_mode; // Addressing mode
    uint16_t own_address;     // Own address (for slave mode)
    bool enable_dma;          // Enable DMA for transfers
    bool enable_interrupts;   // Enable interrupt-driven transfers
} I2C_Config_t;

// I2C state machine states
typedef enum {
    I2C_STATE_IDLE,
    I2C_STATE_START_SENT,
    I2C_STATE_REPEATED_START_SENT,
    I2C_STATE_ADDR_SENT_W,
    I2C_STATE_ADDR_SENT_R,
    I2C_STATE_REG_ADDR_SENT,
    I2C_STATE_WRITING,
    I2C_STATE_READING,
    I2C_STATE_ERROR
} I2C_TransferState_t;

// I2C operation types
typedef enum {
    I2C_OP_NONE,
    I2C_OP_READ,
    I2C_OP_WRITE
} I2C_Operation_t;

// Internal structure for tracking transfer progress
typedef struct {
    uint8_t *buffer;              // Data buffer
    uint16_t size;                // Size of data to transfer
    uint16_t reg_addr;            // Register address for read/write
    uint16_t dev_addr;            // Device address
    I2C_TransferState_t state;    // Current state of transfer
    I2C_Operation_t operation;    // Current operation
    uint16_t index;               // Current index in buffer
    bool repeated_start;          // A repeated start is in progress
    bool blocking;                // Whether transfer should block
    uint32_t timeout;             // Timeout in milliseconds (for blocking transfers)
    TaskHandle_t notify_task;     // Task to notify on completion (for non-blocking transfers)
    uint32_t notification_value;  // Notification value
} I2C_Transfer_t;

// I2C callback types
typedef enum {
    I2C_CALLBACK_TRANSFER_COMPLETE,
    I2C_CALLBACK_ERROR,
    I2C_CALLBACK_MAX
} I2C_CallbackType_t;

// Internal state structure for I2C interface
typedef struct {
    bool initialized;
    I2C_Config_t config;
    I2C_Transfer_t current_transfer;
    void (*callbacks[I2C_CALLBACK_MAX])(void *param);
    void *callback_params[I2C_CALLBACK_MAX];
} I2C_State_t;




// I2C control commands
#define I2C_CTRL_MASTER_TRANSMIT   0x0301
#define I2C_CTRL_MASTER_RECEIVE    0x0302
#define I2C_CTRL_MEM_WRITE         0x0303
#define I2C_CTRL_MEM_READ          0x0304
#define I2C_CTRL_SET_TARGET_ADDR   0x0305
#define I2C_CTRL_RECOVER_BUS       0x0306

// Register IRQ handlers - should be called in the startup code
void I2C1_EV_IRQHandler_Wrapper(void);
void I2C1_ER_IRQHandler_Wrapper(void);
void I2C2_EV_IRQHandler_Wrapper(void);
void I2C2_ER_IRQHandler_Wrapper(void);
void I2C3_EV_IRQHandler_Wrapper(void);
void I2C3_ER_IRQHandler_Wrapper(void);


// Initialize and get I2C hardware interface
HW_Interface_t* I2C_GetInterface(uint8_t i2c_instance);

#endif