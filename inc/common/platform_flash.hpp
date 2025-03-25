#pragma once 

#include "platform.hpp"


namespace Platform {
    namespace FLASH {
        constexpr uint32_t FLASH_BASE = 0x40023C00UL;
        constexpr uint32_t FLASH_ACR_OFFSET = 0x00UL;
        constexpr uint32_t FLASH_ACR = (FLASH_BASE + FLASH_ACR_OFFSET);
        constexpr uintptr_t FLASH_SIZE_REG_ADDR = 0x1FFF7A22;
        inline const uint16_t* FLASH_SIZE_REG = reinterpret_cast<const uint16_t*>(FLASH_SIZE_REG_ADDR);
    
        inline uint32_t GetFlashEndAddress() {
            return 0x08000000 + (*FLASH_SIZE_REG * 1024);
        }
        constexpr uint32_t FLASH_BASE_ADDRESS = 0x08000000;
        constexpr uint8_t  FLASH_SECTOR_COUNT = 8;
        constexpr uint8_t  INVALID_SECTOR = 0xFF;

        constexpr uint32_t FLASH_KEY1 = 0x45670123UL;
        constexpr uint32_t FLASH_KEY2 = 0xCDEF89ABUL;

        // Flash access control register (ACR) bits
        enum class ACR : uint32_t {
            LATENCY_0WS = (0UL << 0),   // Zero wait states
            LATENCY_1WS = (1UL << 0),   // One wait state
            LATENCY_2WS = (2UL << 0),   // Two wait states
            LATENCY_3WS = (3UL << 0),   // Three wait states
            LATENCY_4WS = (4UL << 0),   // Four wait states
            LATENCY_5WS = (5UL << 0),   // Five wait states
            LATENCY_6WS = (6UL << 0),   // Six wait states
            LATENCY_7WS = (7UL << 0),   // Seven wait states
            LATENCY_MSK = (7UL << 0),   // Latency mask
            PRFTEN = (1UL << 8),        // Prefetch enable
            ICEN = (1UL << 9),          // Instruction cache enable
            DCEN = (1UL << 10),         // Data cache enable
            ICRST = (1UL << 11),        // Instruction cache reset
            DCRST = (1UL << 12)         // Data cache reset
        };
        
        // Flash register structure
        struct Registers {
            volatile uint32_t ACR;      // Flash access control register
            volatile uint32_t KEYR;     // Flash key register
            volatile uint32_t OPTKEYR;  // Flash option key register
            volatile uint32_t SR;       // Flash status register
            volatile uint32_t CR;       // Flash control register
            volatile uint32_t OPTCR;    // Flash option control register
        };
        
        // Access function for Flash registers
        inline Registers* getRegisters() {
            return reinterpret_cast<Registers*>(FLASH_BASE);
        }
        
        // Helper functions for bit manipulation
        constexpr uint32_t getBitValue(ACR bit) {
            return static_cast<uint32_t>(bit);
        }
        
        // Operator overload for combining flags
        constexpr ACR operator|(ACR a, ACR b) {
            return static_cast<ACR>(
                static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
        }
    }
        
}