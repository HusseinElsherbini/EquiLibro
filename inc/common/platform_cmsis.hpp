#pragma once

#include "platform.hpp"



namespace Platform {
    namespace CMSIS {
        // CMSIS register definitions for STM32F4

        
        namespace SCB {
            // System Control Block register structure
            struct Registers {
                volatile uint32_t CPUID;        // CPUID Base Register
                volatile uint32_t ICSR;         // Interrupt Control and State Register
                volatile uint32_t VTOR;         // Vector Table Offset Register
                volatile uint32_t AIRCR;        // Application Interrupt and Reset Control Register
                volatile uint32_t SCR;          // System Control Register
                volatile uint32_t CCR;          // Configuration Control Register
                volatile uint8_t  SHP[12];      // System Handlers Priority Registers (8-bit)
                volatile uint32_t SHCSR;        // System Handler Control and State Register
                volatile uint32_t CFSR;         // Configurable Fault Status Register
                volatile uint32_t HFSR;         // HardFault Status Register
                volatile uint32_t DFSR;         // Debug Fault Status Register
                volatile uint32_t MMFAR;        // MemManage Fault Address Register
                volatile uint32_t BFAR;         // BusFault Address Register
                volatile uint32_t AFSR;         // Auxiliary Fault Status Register
                volatile uint32_t PFR[2];       // Processor Feature Register
                volatile uint32_t DFR;          // Debug Feature Register
                volatile uint32_t ADR;          // Auxiliary Feature Register
                volatile uint32_t MMFR[4];      // Memory Model Feature Register
                volatile uint32_t ISAR[5];      // Instruction Set Attributes Register
                uint32_t RESERVED0[5];
                volatile uint32_t CPACR;        // Coprocessor Access Control Register
            };

            // Access function for SCB registers
            inline Registers* getRegisters() {
                return reinterpret_cast<Registers*>(SCB_BASE);
            }
        }        
        //  NVIC name space
        namespace NVIC {
            // IRQ numbers for STM32F4 - used by hardware abstraction layer
            enum class IRQn : int32_t {
                WWDG = 0,                   // Window WatchDog Interrupt
                PVD = 1,                    // PVD through EXTI Line detection Interrupt
                TAMP_STAMP = 2,             // Tamper and TimeStamp interrupts through the EXTI line
                RTC_WKUP = 3,               // RTC Wakeup interrupt through the EXTI line
                FLASH = 4,                  // FLASH global Interrupt
                RCC = 5,                    // RCC global Interrupt
                EXTI0 = 6,                  // EXTI Line0 Interrupt
                EXTI1 = 7,                  // EXTI Line1 Interrupt
                EXTI2 = 8,                  // EXTI Line2 Interrupt
                EXTI3 = 9,                  // EXTI Line3 Interrupt
                EXTI4 = 10,                 // EXTI Line4 Interrupt
                DMA1_Stream0 = 11,          // DMA1 Stream 0 global Interrupt
                DMA1_Stream1 = 12,          // DMA1 Stream 1 global Interrupt
                DMA1_Stream2 = 13,          // DMA1 Stream 2 global Interrupt
                DMA1_Stream3 = 14,          // DMA1 Stream 3 global Interrupt
                DMA1_Stream4 = 15,          // DMA1 Stream 4 global Interrupt
                DMA1_Stream5 = 16,          // DMA1 Stream 5 global Interrupt
                DMA1_Stream6 = 17,          // DMA1 Stream 6 global Interrupt
                ADC = 18,                   // ADC1, ADC2 and ADC3 global Interrupts
                EXTI9_5 = 23,               // External Line[9:5] Interrupts
                TIM1_BRK_TIM9 = 24,         // TIM1 Break interrupt and TIM9 global interrupt
                TIM1_UP_TIM10 = 25,         // TIM1 Update Interrupt and TIM10 global interrupt
                TIM1_TRG_COM_TIM11 = 26,    // TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt
                TIM1_CC = 27,               // TIM1 Capture Compare Interrupt
                TIM2 = 28,                  // TIM2 global Interrupt
                TIM3 = 29,                  // TIM3 global Interrupt
                TIM4 = 30,                  // TIM4 global Interrupt
                I2C1_EV = 31,               // I2C1 Event Interrupt
                I2C1_ER = 32,               // I2C1 Error Interrupt
                I2C2_EV = 33,               // I2C2 Event Interrupt
                I2C2_ER = 34,               // I2C2 Error Interrupt
                SPI1 = 35,                  // SPI1 global Interrupt
                SPI2 = 36,                  // SPI2 global Interrupt
                USART1 = 37,                // USART1 global Interrupt
                USART2 = 38,                // USART2 global Interrupt
                EXTI15_10 = 40,             // External Line[15:10] Interrupts
                RTC_Alarm = 41,             // RTC Alarm (A and B) through EXTI Line Interrupt
                OTG_FS_WKUP = 42,           // USB OTG FS Wakeup through EXTI line interrupt
                DMA1_Stream7 = 47,          // DMA1 Stream7 Interrupt
                SDIO = 49,                  // SDIO global Interrupt
                TIM5 = 50,                  // TIM5 global Interrupt
                SPI3 = 51,                  // SPI3 global Interrupt
                DMA2_Stream0 = 56,          // DMA2 Stream 0 global Interrupt
                DMA2_Stream1 = 57,          // DMA2 Stream 1 global Interrupt
                DMA2_Stream2 = 58,          // DMA2 Stream 2 global Interrupt
                DMA2_Stream3 = 59,          // DMA2 Stream 3 global Interrupt
                DMA2_Stream4 = 60,          // DMA2 Stream 4 global Interrupt
                OTG_FS = 67,                // USB OTG FS global Interrupt
                DMA2_Stream5 = 68,          // DMA2 Stream 5 global interrupt
                DMA2_Stream6 = 69,          // DMA2 Stream 6 global interrupt
                DMA2_Stream7 = 70,          // DMA2 Stream 7 global interrupt
                USART6 = 71,                // USART6 global interrupt
                I2C3_EV = 72,               // I2C3 event interrupt
                I2C3_ER = 73,               // I2C3 error interrupt
                FPU = 81,                   // FPU global interrupt
                SPI4 = 84                   // SPI4 global Interrupt
            };
        
            // NVIC register structure
            struct Registers {
                volatile uint32_t ISER[8];      // Interrupt Set Enable Register
                uint32_t RESERVED0[24];
                volatile uint32_t ICER[8];      // Interrupt Clear Enable Register
                uint32_t RESERVED1[24];
                volatile uint32_t ISPR[8];      // Interrupt Set Pending Register
                uint32_t RESERVED2[24];
                volatile uint32_t ICPR[8];      // Interrupt Clear Pending Register
                uint32_t RESERVED3[24];
                volatile uint32_t IABR[8];      // Interrupt Active Bit Register
                uint32_t RESERVED4[56];
                volatile uint8_t IP[240];       // Interrupt Priority Register (8-bit)
                uint32_t RESERVED5[644];
                volatile uint32_t STIR;         // Software Trigger Interrupt Register
            };
        
            // NVIC priority levels
            constexpr uint8_t PRIORITY_HIGHEST = 0x00U;
            constexpr uint8_t PRIORITY_HIGH = 0x40U;
            constexpr uint8_t PRIORITY_MEDIUM = 0x80U;
            constexpr uint8_t PRIORITY_LOW = 0xC0U;
        
            // NVIC priority grouping
            constexpr uint32_t PRIORITYGROUP_0 = 0x00000007U; // 0 bits for pre-emption priority, 4 bits for subpriority
            constexpr uint32_t PRIORITYGROUP_1 = 0x00000006U; // 1 bits for pre-emption priority, 3 bits for subpriority
            constexpr uint32_t PRIORITYGROUP_2 = 0x00000005U; // 2 bits for pre-emption priority, 2 bits for subpriority
            constexpr uint32_t PRIORITYGROUP_3 = 0x00000004U; // 3 bits for pre-emption priority, 1 bits for subpriority
            constexpr uint32_t PRIORITYGROUP_4 = 0x00000003U; // 4 bits for pre-emption priority, 0 bits for subpriority
        
            // Access function for NVIC registers
            inline Registers* getRegisters() {
                return reinterpret_cast<Registers*>(NVIC_BASE);
            }
            
            // Helper functions for NVIC operations
            inline void enableIRQ(IRQn irq) {
                getRegisters()->ISER[static_cast<uint32_t>(irq) >> 5] = 
                    (1UL << (static_cast<uint32_t>(irq) & 0x1F));
            }
            
            inline void disableIRQ(IRQn irq) {
                getRegisters()->ICER[static_cast<uint32_t>(irq) >> 5] = 
                    (1UL << (static_cast<uint32_t>(irq) & 0x1F));
            }
            
            inline void setPriority(IRQn irq, uint32_t priority) {
                // Set priority for Cortex-M system interrupts
                if (static_cast<int32_t>(irq) < 0) {
        
                    SCB::getRegisters()->SHP[(static_cast<uint32_t>(irq) & 0xFUL)-4UL] = 
                        static_cast<uint8_t>((priority << (8U - 4)) & 0xFFU);
                        SCB::getRegisters()->SHP[(static_cast<uint32_t>(irq) & 0xFUL)-4UL] = 
                        static_cast<uint8_t>((priority << (8U - 4)) & 0xFFU);
                } else {
                    // Set priority for device specific interrupts
                    getRegisters()->IP[static_cast<uint32_t>(irq)] = 
                        static_cast<uint8_t>((priority << (8U - 4)) & 0xFFU);
                }
            }
        }
        
        // SCB name space

        
        // SysTick name space
        namespace SysTick {
            // SysTick Timer register structure
            struct Registers {
                volatile uint32_t CTRL;         // SysTick Control and Status Register
                volatile uint32_t LOAD;         // SysTick Reload Value Register
                volatile uint32_t VAL;          // SysTick Current Value Register
                volatile uint32_t CALIB;        // SysTick Calibration Register
            };

            // SysTick Control Register bits
            constexpr uint32_t CTRL_ENABLE = (1UL << 0);
            constexpr uint32_t CTRL_TICKINT = (1UL << 1);
            constexpr uint32_t CTRL_CLKSOURCE = (1UL << 2);
            constexpr uint32_t CTRL_COUNTFLAG = (1UL << 16);

            // Access function for SysTick registers
            inline Registers* getRegisters() {
                return reinterpret_cast<Registers*>(SysTick_BASE);
            }
        }
    
        // CoreDebug name space
        namespace CoreDebug {
            // Core Debug register structure
            struct Registers {
                volatile uint32_t DHCSR;        // Debug Halting Control and Status Register
                volatile uint32_t DCRSR;        // Debug Core Register Selector Register
                volatile uint32_t DCRDR;        // Debug Core Register Data Register
                volatile uint32_t DEMCR;        // Debug Exception and Monitor Control Register
            };

            // Core Debug register bit definitions
            constexpr uint32_t DHCSR_C_DEBUGEN = (1UL << 0);
            constexpr uint32_t DHCSR_C_HALT = (1UL << 1);
            constexpr uint32_t DHCSR_C_STEP = (1UL << 2);
            constexpr uint32_t DHCSR_C_MASKINTS = (1UL << 3);
            constexpr uint32_t DHCSR_C_SNAPSTALL = (1UL << 5);
            constexpr uint32_t DHCSR_S_REGRDY = (1UL << 16);
            constexpr uint32_t DHCSR_S_HALT = (1UL << 17);
            constexpr uint32_t DHCSR_S_SLEEP = (1UL << 18);
            constexpr uint32_t DHCSR_S_LOCKUP = (1UL << 19);
            constexpr uint32_t DEMCR_TRCENA = (1UL << 24);

            // Access function for CoreDebug registers
            inline Registers* getRegisters() {
                return reinterpret_cast<Registers*>(CoreDebug_BASE);
            }
        }

        // TPI name space
        namespace TPI {
            // TPI register structure
            struct Registers {
                volatile uint32_t SSPSR;        // Supported Parallel Port Size Register
                volatile uint32_t CSPSR;        // Current Parallel Port Size Register
                uint32_t RESERVED0[2];
                volatile uint32_t ACPR;         // Asynchronous Clock Prescaler Register
                uint32_t RESERVED1[55];
                volatile uint32_t SPPR;         // Selected Pin Protocol Register
                uint32_t RESERVED2[131];
                volatile uint32_t FFSR;         // Formatter and Flush Status Register
                volatile uint32_t FFCR;         // Formatter and Flush Control Register
                uint32_t RESERVED3[786];
                volatile uint32_t TRIGGER;      // TRIGGER Register
                uint32_t RESERVED4[15];
                volatile uint32_t FIFO0;        // Integration ETM Data Register 0
                volatile uint32_t ITATBCTR2;    // ITATBCTR2 Register
                uint32_t RESERVED5[1];
                volatile uint32_t ITATBCTR0;    // ITATBCTR0 Register
                volatile uint32_t FIFO1;        // Integration ETM Data Register 1
                uint32_t RESERVED6[1];
                volatile uint32_t ITCTRL;       // Integration Mode Control Register
                uint32_t RESERVED7[39];
                volatile uint32_t CLAIMSET;     // Claim Tag Set Register
                volatile uint32_t CLAIMCLR;     // Claim Tag Clear Register
                uint32_t RESERVED8[8];
                volatile uint32_t DEVID;        // Device ID Register
                volatile uint32_t DEVTYPE;      // Device Type Register
            };

            // Access function for TPI registers
            inline Registers* getRegisters() {
                return reinterpret_cast<Registers*>(TPI_BASE);
            }
        }

        // ITM name space
        namespace ITM {
            // ITM register structure
            struct Registers {
                volatile uint32_t STIM[32];     // Stimulus Port Registers
                volatile uint32_t RESERVED0[864];
                volatile uint32_t TER;          // Trace Enable Register
                volatile uint32_t TPR;          // Trace Privilege Register
                volatile uint32_t TCR;          // Trace Control Register
                volatile uint32_t IWR;          // Integration Write Register
                volatile uint32_t IRR;          // Integration Read Register
                volatile uint32_t IMCR;         // Integration Mode Control Register
                volatile uint32_t LAR;          // Lock Access Register
                volatile uint32_t LSR;          // Lock Status Register
                volatile uint32_t RESERVED1[2];
                volatile uint32_t PID4;         // Peripheral Identification Register 4
                volatile uint32_t PID5;         // Peripheral Identification Register 5
                volatile uint32_t PID6;         // Peripheral Identification Register 6
                volatile uint32_t PID7;         // Peripheral Identification Register 7
                volatile uint32_t PID0;         // Peripheral Identification Register 0
                volatile uint32_t PID1;         // Peripheral Identification Register 1
                volatile uint32_t PID2;         // Peripheral Identification Register 2
                volatile uint32_t PID3;         // Peripheral Identification Register 3
                volatile uint32_t CID0;         // Component Identification Register 0
                volatile uint32_t CID1;         // Component Identification Register 1
                volatile uint32_t CID2;         // Component Identification Register 2
                volatile uint32_t CID3;         // Component Identification Register 3
            };

            // ITM register bit definitions
            constexpr uint32_t TER_STIMENA = (1UL << 0);

            // Access function for ITM registers
            inline Registers* getRegisters() {
                return reinterpret_cast<Registers*>(ITM_BASE);
            }
        }

        // DWT name space
        namespace DWT {
            // Data Watchpoint and Trace register structure
            struct Registers {
                volatile uint32_t CTRL;         // Control Register
                volatile uint32_t CYCCNT;       // Cycle Count Register
                volatile uint32_t CPICNT;       // CPI Count Register
                volatile uint32_t EXCCNT;       // Exception Overhead Count Register
                volatile uint32_t SLEEPCNT;     // Sleep Count Register
                volatile uint32_t LSUCNT;       // LSU Count Register
                volatile uint32_t FOLDCNT;      // Folded-instruction Count Register
                volatile uint32_t PCSR;         // Program Counter Sample Register
                volatile uint32_t COMP0;        // Comparator Register 0
                volatile uint32_t MASK0;        // Mask Register 0
                volatile uint32_t FUNCTION0;    // Function Register 0
                uint32_t RESERVED0[1];
                volatile uint32_t COMP1;        // Comparator Register 1
                volatile uint32_t MASK1;        // Mask Register 1
                volatile uint32_t FUNCTION1;    // Function Register 1
                uint32_t RESERVED1[1];
                volatile uint32_t COMP2;        // Comparator Register 2
                volatile uint32_t MASK2;        // Mask Register 2
                volatile uint32_t FUNCTION2;    // Function Register 2
                uint32_t RESERVED2[1];
                volatile uint32_t COMP3;        // Comparator Register 3
                volatile uint32_t MASK3;        // Mask Register 3
                volatile uint32_t FUNCTION3;    // Function Register 3
            };

            // DWT Control Register bits
            constexpr uint32_t CTRL_CYCCNTENA = (1UL << 0);

            // Access function for DWT registers
            inline Registers* getRegisters() {
                return reinterpret_cast<Registers*>(DWT_BASE);
            }
        }

        // FPB name space
        namespace FPB {
            // Flash Patch and Breakpoint register structure
            struct Registers {
                volatile uint32_t CTRL;         // Control Register
                volatile uint32_t REMAP;        // Remap Register
                volatile uint32_t COMP[8];      // Comparator Register
                uint32_t RESERVED0[24];
                volatile uint32_t PID4;         // Peripheral Identification Register 4
                volatile uint32_t PID5;         // Peripheral Identification Register 5
                volatile uint32_t PID6;         // Peripheral Identification Register 6
                volatile uint32_t PID7;         // Peripheral Identification Register 7
                volatile uint32_t PID0;         // Peripheral Identification Register 0
                volatile uint32_t PID1;         // Peripheral Identification Register 1
                volatile uint32_t PID2;         // Peripheral Identification Register 2
                volatile uint32_t PID3;         // Peripheral Identification Register 3
                volatile uint32_t CID0;         // Component Identification Register 0
                volatile uint32_t CID1;         // Component Identification Register 1
                volatile uint32_t CID2;         // Component Identification Register 2
                volatile uint32_t CID3;         // Component Identification Register 3
            };

            // Access function for FPB registers
            inline Registers* getRegisters() {
                return reinterpret_cast<Registers*>(FPB_BASE);
            }
        }
    
        // MPU name space
        namespace MPU {
            // Memory Protection Unit register structure
            struct Registers {
                volatile uint32_t TYPE;         // MPU Type Register
                volatile uint32_t CTRL;         // MPU Control Register
                volatile uint32_t RNR;          // MPU Region Number Register
                volatile uint32_t RBAR;         // MPU Region Base Address Register
                volatile uint32_t RASR;         // MPU Region Attribute and Size Register
                volatile uint32_t RBAR_A1;      // Alias 1 Region Base Address Register
                volatile uint32_t RASR_A1;      // Alias 1 Region Attribute and Size Register
                volatile uint32_t RBAR_A2;      // Alias 2 Region Base Address Register
                volatile uint32_t RASR_A2;      // Alias 2 Region Attribute and Size Register
                volatile uint32_t RBAR_A3;      // Alias 3 Region Base Address Register
                volatile uint32_t RASR_A3;      // Alias 3 Region Attribute and Size Register
            };

            // MPU Control Register bits
            constexpr uint32_t CTRL_ENABLE = (1UL << 0);
            constexpr uint32_t CTRL_HFNMIENA = (1UL << 1);
            constexpr uint32_t CTRL_PRIVDEFENA = (1UL << 2);

            // Access function for MPU registers
            inline Registers* getRegisters() {
                return reinterpret_cast<Registers*>(MPU_BASE);
            }
        }

        // FPU name space
        namespace FPU {
            // Floating Point Unit register structure
            struct Registers {
                volatile uint32_t CPACR;        // Coprocessor Access Control Register
            };

            // FPU register bit definitions
            constexpr uint32_t CPACR_CP10 = (3UL << 20);
            constexpr uint32_t CPACR_CP11 = (3UL << 22);

            // Access function for FPU registers
            inline Registers* getRegisters() {
                return reinterpret_cast<Registers*>(FPU_BASE);
            }
        }
    }

}