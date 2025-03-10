#pragma once

#include "platform.hpp"

namespace Platform {
    namespace I2C {
        // I2C base addresses
        constexpr uint32_t I2C1_BASE = (APB1PERIPH_BASE + 0x5400UL);
        constexpr uint32_t I2C2_BASE = (APB1PERIPH_BASE + 0x5800UL);
        constexpr uint32_t I2C3_BASE = (APB1PERIPH_BASE + 0x5C00UL);

        // I2C register structure
        struct Registers {
            volatile uint32_t CR1;        // Control register 1
            volatile uint32_t CR2;        // Control register 2
            volatile uint32_t OAR1;       // Own address register 1
            volatile uint32_t OAR2;       // Own address register 2
            volatile uint32_t DR;         // Data register
            volatile uint32_t SR1;        // Status register 1
            volatile uint32_t SR2;        // Status register 2
            volatile uint32_t CCR;        // Clock control register
            volatile uint32_t TRISE;      // TRISE register
            volatile uint32_t FLTR;       // FLTR register
        };

        // I2C addressing mode
        enum class AddrMode {
            Addr7Bit,
            Addr10Bit
        };

        // I2C speed mode
        enum class Speed {
            Standard,  // 100 KHz
            Fast       // 400 KHz
        };

        // I2C mode
        enum class Mode {
            Master,
            Slave
        };

        // I2C operation type
        enum class Operation {
            None,
            Read,
            Write
        };

        // CR1 register bits
        enum class CR1 : uint32_t {
            PE = (1UL << 0),        // Peripheral enable
            SMBUS = (1UL << 1),     // SMBus mode
            SMBTYPE = (1UL << 3),   // SMBus type
            ENARP = (1UL << 4),     // ARP enable
            ENPEC = (1UL << 5),     // PEC enable
            ENGC = (1UL << 6),      // General call enable
            NOSTRETCH = (1UL << 7), // Clock stretching disable
            START = (1UL << 8),     // Start generation
            STOP = (1UL << 9),      // Stop generation
            ACK = (1UL << 10),      // Acknowledge enable
            POS = (1UL << 11),      // Acknowledge/PEC position
            PEC = (1UL << 12),      // Packet error checking
            ALERT = (1UL << 13),    // SMBus alert
            SWRST = (1UL << 15)     // Software reset
        };

        // CR2 register bits
        enum class CR2 : uint32_t {
            FREQ_MSK = (0x3FUL << 0),  // Peripheral clock frequency mask
            ITERREN = (1UL << 8),      // Error interrupt enable
            ITEVTEN = (1UL << 9),      // Event interrupt enable
            ITBUFEN = (1UL << 10),     // Buffer interrupt enable
            DMAEN = (1UL << 11),       // DMA requests enable
            LAST = (1UL << 12)         // DMA last transfer
        };

        // SR1 register bits
        enum class SR1 : uint32_t {
            SB = (1UL << 0),        // Start bit (Master mode)
            ADDR = (1UL << 1),      // Address sent (master mode)/matched (slave mode)
            BTF = (1UL << 2),       // Byte transfer finished
            ADD10 = (1UL << 3),     // 10-bit header sent (Master mode)
            STOPF = (1UL << 4),     // Stop detection (slave mode)
            RXNE = (1UL << 6),      // Data register not empty (receivers)
            TXE = (1UL << 7),       // Data register empty (transmitters)
            BERR = (1UL << 8),      // Bus error
            ARLO = (1UL << 9),      // Arbitration lost (master mode)
            AF = (1UL << 10),       // Acknowledge failure
            OVR = (1UL << 11),      // Overrun/Underrun
            PECERR = (1UL << 12),   // PEC Error in reception
            TIMEOUT = (1UL << 14),  // Timeout or Tlow error
            SMBALERT = (1UL << 15)  // SMBus alert
        };

        // SR2 register bits
        enum class SR2 : uint32_t {
            MSL = (1UL << 0),         // Master/slave
            BUSY = (1UL << 1),        // Bus busy
            TRA = (1UL << 2),         // Transmitter/receiver
            GENCALL = (1UL << 4),     // General call address (slave mode)
            SMBDEFAULT = (1UL << 5),  // SMBus device default address (slave mode)
            SMBHOST = (1UL << 6),     // SMBus host header (slave mode)
            DUALF = (1UL << 7),       // Dual flag (slave mode)
            PEC_MSK = (0xFFUL << 8)   // Packet error checking register mask
        };

        // CCR register bits
        enum class CCR : uint32_t {
            CCR_MSK = (0xFFFUL << 0),  // Clock control register mask
            DUTY = (1UL << 14),        // Fast mode duty cycle
            FS = (1UL << 15)           // I2C master mode selection
        };

        // Access functions for I2C registers
        inline Registers* getI2C1() {
            return reinterpret_cast<Registers*>(I2C1_BASE);
        }

        inline Registers* getI2C2() {
            return reinterpret_cast<Registers*>(I2C2_BASE);
        }

        inline Registers* getI2C3() {
            return reinterpret_cast<Registers*>(I2C3_BASE);
        }

        // Helper function to get I2C registers by instance number
        inline Registers* getI2C(uint8_t instance) {
            switch (instance) {
                case 1: return getI2C1();
                case 2: return getI2C2();
                case 3: return getI2C3();
                default: return nullptr;
            }
        }

        // Helper functions for bit manipulation
        constexpr uint32_t getBitValue(CR1 bit) {
            return static_cast<uint32_t>(bit);
        }

        constexpr uint32_t getBitValue(CR2 bit) {
            return static_cast<uint32_t>(bit);
        }

        constexpr uint32_t getBitValue(SR1 bit) {
            return static_cast<uint32_t>(bit);
        }

        constexpr uint32_t getBitValue(SR2 bit) {
            return static_cast<uint32_t>(bit);
        }

        constexpr uint32_t getBitValue(CCR bit) {
            return static_cast<uint32_t>(bit);
        }

        // Operator overloads for combining flags
        constexpr CR1 operator|(CR1 a, CR1 b) {
            return static_cast<CR1>(
                static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
        }

        constexpr CR2 operator|(CR2 a, CR2 b) {
            return static_cast<CR2>(
                static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
        }

        constexpr SR1 operator|(SR1 a, SR1 b) {
            return static_cast<SR1>(
                static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
        }

        constexpr SR2 operator|(SR2 a, SR2 b) {
            return static_cast<SR2>(
                static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
        }

        constexpr CCR operator|(CCR a, CCR b) {
            return static_cast<CCR>(
                static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
        }
    }
}