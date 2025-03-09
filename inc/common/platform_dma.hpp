#pragma once 


#include "platform.hpp"



namespace Platform {

    namespace DMA {
        constexpr uint32_t DMA1_BASE = (AHB1PERIPH_BASE + 0x6000UL);
        constexpr uint32_t DMA2_BASE = (AHB1PERIPH_BASE + 0x6400UL);
        
        // DMA stream register structure
        struct DMA_Stream {
            volatile uint32_t CR;           // DMA stream x configuration register
            volatile uint32_t NDTR;         // DMA stream x number of data register
            volatile uint32_t PAR;          // DMA stream x peripheral address register
            volatile uint32_t M0AR;         // DMA stream x memory 0 address register
            volatile uint32_t M1AR;         // DMA stream x memory 1 address register
            volatile uint32_t FCR;          // DMA stream x FIFO control register
        };
    
        // DMA register structure
        struct Registers {
            volatile uint32_t LISR;          // DMA low interrupt status register
            volatile uint32_t HISR;          // DMA high interrupt status register
            volatile uint32_t LIFCR;         // DMA low interrupt flag clear register
            volatile uint32_t HIFCR;         // DMA high interrupt flag clear register
            DMA_Stream   STREAM[8];          // DMA streams
        };
    
        // Stream identifier (for easier use in APIs)
        enum class Stream : uint32_t {
            Stream0 = 0,
            Stream1 = 1,
            Stream2 = 2,
            Stream3 = 3,
            Stream4 = 4,
            Stream5 = 5,
            Stream6 = 6,
            Stream7 = 7
        };
    
        // DMA channel selection (for multiplexed requests)
        enum class Channel : uint32_t {
            Channel0 = 0UL << 25,
            Channel1 = 1UL << 25,
            Channel2 = 2UL << 25,
            Channel3 = 3UL << 25,
            Channel4 = 4UL << 25,
            Channel5 = 5UL << 25,
            Channel6 = 6UL << 25,
            Channel7 = 7UL << 25,
            ChannelMask = 7UL << 25
        };
    
        // Data transfer direction
        enum class Direction : uint32_t {
            PeripheralToMemory = 0UL << 6,
            MemoryToPeripheral = 1UL << 6,
            MemoryToMemory = 2UL << 6,
            DirectionMask = 3UL << 6
        };
    
        // Data item size
        enum class DataSize : uint32_t {
            Byte = 0UL << 11,      // 8-bit
            HalfWord = 1UL << 11,  // 16-bit
            Word = 2UL << 11,      // 32-bit
            SizeMask = 3UL << 11
        };
    
        // Memory burst transfer configuration
        enum class MemoryBurst : uint32_t {
            Single = 0UL << 23,
            Incr4 = 1UL << 23,
            Incr8 = 2UL << 23,
            Incr16 = 3UL << 23,
            BurstMask = 3UL << 23
        };
    
        // Peripheral burst transfer configuration
        enum class PeripheralBurst : uint32_t {
            Single = 0UL << 21,
            Incr4 = 1UL << 21,
            Incr8 = 2UL << 21,
            Incr16 = 3UL << 21,
            BurstMask = 3UL << 21
        };
    
        // FIFO threshold level
        enum class FIFOThreshold : uint32_t {
            Quarter = 0UL << 0,
            Half = 1UL << 0,
            ThreeQuarters = 2UL << 0,
            Full = 3UL << 0,
            ThresholdMask = 3UL << 0
        };
    
        // Priority level
        enum class Priority : uint32_t {
            Low = 0UL << 16,
            Medium = 1UL << 16,
            High = 2UL << 16,
            VeryHigh = 3UL << 16,
            PriorityMask = 3UL << 16
        };
    
        // DMA_SxCR register bits
        enum class CR : uint32_t {
            EN = (1UL << 0),              // Stream enable
            DMEIE = (1UL << 1),           // Direct mode error interrupt enable
            TEIE = (1UL << 2),            // Transfer error interrupt enable
            HTIE = (1UL << 3),            // Half transfer interrupt enable
            TCIE = (1UL << 4),            // Transfer complete interrupt enable
            PFCTRL = (1UL << 5),          // Peripheral flow controller
            DIR_P2M = (0UL << 6),         // Peripheral to memory
            DIR_M2P = (1UL << 6),         // Memory to peripheral
            DIR_M2M = (2UL << 6),         // Memory to memory
            DIR_MSK = (3UL << 6),         // Direction mask
            CIRC = (1UL << 8),            // Circular mode
            PINC = (1UL << 9),            // Peripheral increment mode
            MINC = (1UL << 10),           // Memory increment mode
            PSIZE_8BIT = (0UL << 11),     // Peripheral data size: Byte (8-bits)
            PSIZE_16BIT = (1UL << 11),    // Peripheral data size: Half-word (16-bits)
            PSIZE_32BIT = (2UL << 11),    // Peripheral data size: Word (32-bits)
            PSIZE_MSK = (3UL << 11),      // Peripheral data size mask
            MSIZE_8BIT = (0UL << 13),     // Memory data size: Byte (8-bits)
            MSIZE_16BIT = (1UL << 13),    // Memory data size: Half-word (16-bits)
            MSIZE_32BIT = (2UL << 13),    // Memory data size: Word (32-bits)
            MSIZE_MSK = (3UL << 13),      // Memory data size mask
            PINCOS = (1UL << 15),         // Peripheral increment offset size
            PL_LOW = (0UL << 16),         // Priority level: Low
            PL_MEDIUM = (1UL << 16),      // Priority level: Medium
            PL_HIGH = (2UL << 16),        // Priority level: High
            PL_VERY_HIGH = (3UL << 16),   // Priority level: Very high
            PL_MSK = (3UL << 16),         // Priority level mask
            DBM = (1UL << 18),            // Double buffer mode
            CT = (1UL << 19),             // Current target (only in double buffer mode)
            PBURST_SINGLE = (0UL << 21),  // Peripheral burst: Single transfer
            PBURST_INCR4 = (1UL << 21),   // Peripheral burst: Incremental burst of 4 beats
            PBURST_INCR8 = (2UL << 21),   // Peripheral burst: Incremental burst of 8 beats
            PBURST_INCR16 = (3UL << 21),  // Peripheral burst: Incremental burst of 16 beats
            PBURST_MSK = (3UL << 21),     // Peripheral burst mask
            MBURST_SINGLE = (0UL << 23),  // Memory burst: Single transfer
            MBURST_INCR4 = (1UL << 23),   // Memory burst: Incremental burst of 4 beats
            MBURST_INCR8 = (2UL << 23),   // Memory burst: Incremental burst of 8 beats
            MBURST_INCR16 = (3UL << 23),  // Memory burst: Incremental burst of 16 beats
            MBURST_MSK = (3UL << 23),     // Memory burst mask
            CHSEL_0 = (0UL << 25),        // Channel 0 selection
            CHSEL_1 = (1UL << 25),        // Channel 1 selection
            CHSEL_2 = (2UL << 25),        // Channel 2 selection
            CHSEL_3 = (3UL << 25),        // Channel 3 selection
            CHSEL_4 = (4UL << 25),        // Channel 4 selection
            CHSEL_5 = (5UL << 25),        // Channel 5 selection
            CHSEL_6 = (6UL << 25),        // Channel 6 selection
            CHSEL_7 = (7UL << 25),        // Channel 7 selection
            CHSEL_MSK = (7UL << 25)       // Channel selection mask
        };
    
        // DMA_SxFCR register bits
        enum class FCR : uint32_t {
            FTH_1_4 = (0UL << 0),          // FIFO threshold: 1/4 full
            FTH_1_2 = (1UL << 0),          // FIFO threshold: 1/2 full
            FTH_3_4 = (2UL << 0),          // FIFO threshold: 3/4 full
            FTH_FULL = (3UL << 0),         // FIFO threshold: Full
            FTH_MSK = (3UL << 0),          // FIFO threshold mask
            DMDIS = (1UL << 2),            // Direct mode disable
            FS_LT_1_4 = (0UL << 3),        // FIFO status: Less than 1/4 full
            FS_1_4_TO_1_2 = (1UL << 3),    // FIFO status: 1/4 to 1/2 full
            FS_1_2_TO_3_4 = (2UL << 3),    // FIFO status: 1/2 to 3/4 full
            FS_3_4_TO_FULL = (3UL << 3),   // FIFO status: 3/4 full to full
            FS_EMPTY = (4UL << 3),         // FIFO status: Empty
            FS_FULL = (5UL << 3),          // FIFO status: Full
            FS_MSK = (7UL << 3),           // FIFO status mask
            FEIE = (1UL << 7)              // FIFO error interrupt enable
        };
    
        // DMA LISR/HISR register bits layout (per stream)
        enum class ISR_Bits : uint32_t {
            FEIF0 = (1UL << 0),           // Stream 0 FIFO error interrupt flag
            DMEIF0 = (1UL << 2),          // Stream 0 direct mode error interrupt flag
            TEIF0 = (1UL << 3),           // Stream 0 transfer error interrupt flag
            HTIF0 = (1UL << 4),           // Stream 0 half transfer interrupt flag
            TCIF0 = (1UL << 5),           // Stream 0 transfer complete interrupt flag
            FEIF1 = (1UL << 6),           // Stream 1 FIFO error interrupt flag
            DMEIF1 = (1UL << 8),          // Stream 1 direct mode error interrupt flag
            TEIF1 = (1UL << 9),           // Stream 1 transfer error interrupt flag
            HTIF1 = (1UL << 10),          // Stream 1 half transfer interrupt flag
            TCIF1 = (1UL << 11),          // Stream 1 transfer complete interrupt flag
            FEIF2 = (1UL << 16),          // Stream 2 FIFO error interrupt flag
            DMEIF2 = (1UL << 18),         // Stream 2 direct mode error interrupt flag
            TEIF2 = (1UL << 19),          // Stream 2 transfer error interrupt flag
            HTIF2 = (1UL << 20),          // Stream 2 half transfer interrupt flag
            TCIF2 = (1UL << 21),          // Stream 2 transfer complete interrupt flag
            FEIF3 = (1UL << 22),          // Stream 3 FIFO error interrupt flag
            DMEIF3 = (1UL << 24),         // Stream 3 direct mode error interrupt flag
            TEIF3 = (1UL << 25),          // Stream 3 transfer error interrupt flag
            HTIF3 = (1UL << 26),          // Stream 3 half transfer interrupt flag
            TCIF3 = (1UL << 27)           // Stream 3 transfer complete interrupt flag
        };
    
        // DMA LIFCR/HIFCR register bits layout (per stream)
        enum class IFCR_Bits : uint32_t {
            CFEIF0 = (1UL << 0),          // Stream 0 clear FIFO error interrupt flag
            CDMEIF0 = (1UL << 2),         // Stream 0 clear direct mode error interrupt flag
            CTEIF0 = (1UL << 3),          // Stream 0 clear transfer error interrupt flag
            CHTIF0 = (1UL << 4),          // Stream 0 clear half transfer interrupt flag
            CTCIF0 = (1UL << 5),          // Stream 0 clear transfer complete interrupt flag
            CFEIF1 = (1UL << 6),          // Stream 1 clear FIFO error interrupt flag
            CDMEIF1 = (1UL << 8),         // Stream 1 clear direct mode error interrupt flag
            CTEIF1 = (1UL << 9),          // Stream 1 clear transfer error interrupt flag
            CHTIF1 = (1UL << 10),         // Stream 1 clear half transfer interrupt flag
            CTCIF1 = (1UL << 11),         // Stream 1 clear transfer complete interrupt flag
            CFEIF2 = (1UL << 16),         // Stream 2 clear FIFO error interrupt flag
            CDMEIF2 = (1UL << 18),        // Stream 2 clear direct mode error interrupt flag
            CTEIF2 = (1UL << 19),         // Stream 2 clear transfer error interrupt flag
            CHTIF2 = (1UL << 20),         // Stream 2 clear half transfer interrupt flag
            CTCIF2 = (1UL << 21),         // Stream 2 clear transfer complete interrupt flag
            CFEIF3 = (1UL << 22),         // Stream 3 clear FIFO error interrupt flag
            CDMEIF3 = (1UL << 24),        // Stream 3 clear direct mode error interrupt flag
            CTEIF3 = (1UL << 25),         // Stream 3 clear transfer error interrupt flag
            CHTIF3 = (1UL << 26),         // Stream 3 clear half transfer interrupt flag
            CTCIF3 = (1UL << 27)          // Stream 3 clear transfer complete interrupt flag
        };
    
        // Get DMA registers
        inline Registers* getDMA1Registers() {
            return reinterpret_cast<Registers*>(DMA1_BASE);
        }
    
        inline Registers* getDMA2Registers() {
            return reinterpret_cast<Registers*>(DMA2_BASE);
        }
    
        // Get specific stream configuration register
        inline DMA_Stream* getStream(int controller, Stream stream) {
            if (controller == 1) {
                return &(getDMA1Registers()->STREAM[static_cast<uint32_t>(stream)]);
            } else if (controller == 2) {
                return &(getDMA2Registers()->STREAM[static_cast<uint32_t>(stream)]);
            }
            return nullptr;
        }
    
        // Helper functions for bit manipulation
        constexpr uint32_t getBitValue(CR bit) {
            return static_cast<uint32_t>(bit);
        }
    
        constexpr uint32_t getBitValue(FCR bit) {
            return static_cast<uint32_t>(bit);
        }
    
        constexpr uint32_t getBitValue(ISR_Bits bit) {
            return static_cast<uint32_t>(bit);
        }
    
        constexpr uint32_t getBitValue(IFCR_Bits bit) {
            return static_cast<uint32_t>(bit);
        }
    
        constexpr uint32_t getBitValue(Channel channel) {
            return static_cast<uint32_t>(channel);
        }
    
        constexpr uint32_t getBitValue(Direction direction) {
            return static_cast<uint32_t>(direction);
        }
    
        constexpr uint32_t getBitValue(DataSize size) {
            return static_cast<uint32_t>(size);
        }
    
        constexpr uint32_t getBitValue(Priority priority) {
            return static_cast<uint32_t>(priority);
        }
    
        // Operator overloads for combining flags
        constexpr CR operator|(CR a, CR b) {
            return static_cast<CR>(
                static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
        }
    
        constexpr FCR operator|(FCR a, FCR b) {
            return static_cast<FCR>(
                static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
        }
    
        constexpr ISR_Bits operator|(ISR_Bits a, ISR_Bits b) {
            return static_cast<ISR_Bits>(
                static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
        }
    
        constexpr IFCR_Bits operator|(IFCR_Bits a, IFCR_Bits b) {
            return static_cast<IFCR_Bits>(
                static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
        }
    
        // Helper functions to check interrupt flags for specific streams
        inline bool isTransferComplete(int controller, Stream stream) {
            const auto streamIdx = static_cast<uint32_t>(stream);
            if (streamIdx < 4) {
                // Streams 0-3 are in LISR
                const uint32_t bitPos = 5 + (streamIdx * 6); // TCIF bit positions
                return (controller == 1) ? 
                    (getDMA1Registers()->LISR & (1UL << bitPos)) :
                    (getDMA2Registers()->LISR & (1UL << bitPos));
            } else {
                // Streams 4-7 are in HISR
                const uint32_t bitPos = 5 + ((streamIdx - 4) * 6); // TCIF bit positions
                return (controller == 1) ? 
                    (getDMA1Registers()->HISR & (1UL << bitPos)) :
                    (getDMA2Registers()->HISR & (1UL << bitPos));
            }
        }
    
        // Helper function to clear all interrupt flags for a stream
        inline void clearAllFlags(int controller, Stream stream) {
            const auto streamIdx = static_cast<uint32_t>(stream);
            if (streamIdx < 4) {
                // Streams 0-3 are in LIFCR
                const uint32_t flagsMask = 0x3D << (streamIdx * 6); // All flags for this stream
                if (controller == 1) {
                    getDMA1Registers()->LIFCR = flagsMask;
                } else {
                    getDMA2Registers()->LIFCR = flagsMask;
                }
            } else {
                // Streams 4-7 are in HIFCR
                const uint32_t flagsMask = 0x3D << ((streamIdx - 4) * 6); // All flags for this stream
                if (controller == 1) {
                    getDMA1Registers()->HIFCR = flagsMask;
                } else {
                    getDMA2Registers()->HIFCR = flagsMask;
                }
            }
        }
    }
}