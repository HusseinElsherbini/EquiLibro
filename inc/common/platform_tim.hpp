// platform_tim.hpp - Timer definitions for STM32F4 series

#ifndef PLATFORM_TIM_HPP
#define PLATFORM_TIM_HPP

#include "platform.hpp"
#include <cstdint>

namespace Platform {
namespace TIM {

    constexpr uint8_t TIM_CHANNEL_COUNT = 8; // Number of timer channels    
    // Base addresses for timer peripherals
    constexpr uint32_t TIM1_BASE = 0x40010000UL;  // Advanced-control timer
    constexpr uint32_t TIM2_BASE = 0x40000000UL;  // General-purpose timer
    constexpr uint32_t TIM3_BASE = 0x40000400UL;  // General-purpose timer
    constexpr uint32_t TIM4_BASE = 0x40000800UL;  // General-purpose timer
    constexpr uint32_t TIM5_BASE = 0x40000C00UL;  // General-purpose timer
    constexpr uint32_t TIM9_BASE = 0x40014000UL;  // General-purpose timer
    constexpr uint32_t TIM10_BASE = 0x40014400UL; // General-purpose timer
    constexpr uint32_t TIM11_BASE = 0x40014800UL; // General-purpose timer

    struct TimerConfig {
        uint8_t timerInstance;             // Timer instance (1-5)
        Mode mode;                    // Timer operating mode
        ClockDivision clockDivision;  // Clock division
        Direction direction;          // Count direction
        Alignment alignment;          // Alignment mode
        uint32_t prescaler;                // Timer prescaler
        uint32_t period;                   // Timer period (auto-reload value)
        bool autoReloadPreload;            // Auto-reload preload enable
    };
    
    /**
     * Timer channel configuration structure
     */
    struct TimerChannelConfig {
        Channel channel;          // Timer channel
        OCMode ocMode;            // Output compare mode
        uint32_t pulse;                // Pulse value (capture/compare register)
        bool ocPreload;                // Output compare preload enable
        bool complementaryOutput;      // Enable complementary output
    };
    // Timer operating modes
    enum class Mode {
        Basic,          // Basic timer operation
        PWM,            // PWM generation
        InputCapture,   // Input capture mode
        OutputCompare   // Output compare mode
    };

    // Timer channel selection
    enum class Channel {
        Channel1 = 1,   // Timer channel 1
        Channel2 = 2,   // Timer channel 2
        Channel3 = 3,   // Timer channel 3
        Channel4 = 4    // Timer channel 4
    };

    // Clock division factor
    enum class ClockDivision {
        Div1 = 0,       // tDTS = tCK_INT
        Div2 = 1,       // tDTS = 2 × tCK_INT
        Div4 = 2        // tDTS = 4 × tCK_INT
    };

    // Counter direction
    enum class Direction {
        Up = 0,         // Counter counts up
        Down = 1        // Counter counts down
    };

    // Center-aligned mode selection
    enum class Alignment {
        Edge = 0,       // Edge-aligned mode
        Center1 = 1,    // Center-aligned mode 1
        Center2 = 2,    // Center-aligned mode 2
        Center3 = 3     // Center-aligned mode 3
    };

    // Output compare mode
    enum class OCMode {
        Frozen = 0,         // Frozen - compare has no effect on output
        Active = 1,         // Set active level on match
        Inactive = 2,       // Set inactive level on match
        Toggle = 3,         // Toggle output on match
        ForceInactive = 4,  // Force output to inactive level
        ForceActive = 5,    // Force output to active level
        PWM1 = 6,           // PWM mode 1 (active if CNT < CCR, inactive otherwise)
        PWM2 = 7            // PWM mode 2 (inactive if CNT < CCR, active otherwise)
    };

    // Timer Register Definitions
    // These follow the structure provided in the STM32F4 reference manual

    // Timer register structure
    struct Registers {
        volatile uint32_t CR1;      // 0x00 - Control register 1
        volatile uint32_t CR2;      // 0x04 - Control register 2
        volatile uint32_t SMCR;     // 0x08 - Slave mode control register
        volatile uint32_t DIER;     // 0x0C - DMA/Interrupt enable register
        volatile uint32_t SR;       // 0x10 - Status register
        volatile uint32_t EGR;      // 0x14 - Event generation register
        volatile uint32_t CCMR1;    // 0x18 - Capture/compare mode register 1
        volatile uint32_t CCMR2;    // 0x1C - Capture/compare mode register 2
        volatile uint32_t CCER;     // 0x20 - Capture/compare enable register
        volatile uint32_t CNT;      // 0x24 - Counter
        volatile uint32_t PSC;      // 0x28 - Prescaler
        volatile uint32_t ARR;      // 0x2C - Auto-reload register
        volatile uint32_t RCR;      // 0x30 - Repetition counter register (TIM1 only)
        volatile uint32_t CCR1;     // 0x34 - Capture/compare register 1
        volatile uint32_t CCR2;     // 0x38 - Capture/compare register 2
        volatile uint32_t CCR3;     // 0x3C - Capture/compare register 3
        volatile uint32_t CCR4;     // 0x40 - Capture/compare register 4
        volatile uint32_t BDTR;     // 0x44 - Break and dead-time register (TIM1 only)
        volatile uint32_t DCR;      // 0x48 - DMA control register
        volatile uint32_t DMAR;     // 0x4C - DMA address for full transfer
        volatile uint32_t OR;       // 0x50 - Option register (TIM2, TIM5, TIM10, TIM11 only)
    };

    // CR1 register bit positions and masks
    enum class CR1 : uint32_t {
        CEN = (1UL << 0),       // Counter enable
        UDIS = (1UL << 1),      // Update disable
        URS = (1UL << 2),       // Update request source
        OPM = (1UL << 3),       // One-pulse mode
        DIR = (1UL << 4),       // Direction
        CMS_EDGE = (0UL << 5),  // Edge-aligned mode
        CMS_CENTER1 = (1UL << 5), // Center-aligned mode 1
        CMS_CENTER2 = (2UL << 5), // Center-aligned mode 2
        CMS_CENTER3 = (3UL << 5), // Center-aligned mode 3
        CMS_MSK = (3UL << 5),   // CMS mask
        ARPE = (1UL << 7),      // Auto-reload preload enable
        CKD_DIV1 = (0UL << 8),  // Clock division 1
        CKD_DIV2 = (1UL << 8),  // Clock division 2
        CKD_DIV4 = (2UL << 8),  // Clock division 4
        CKD_MSK = (3UL << 8)    // CKD mask
    };

    // CR2 register bit positions and masks
    enum class CR2 : uint32_t {
        CCPC = (1UL << 0),      // Capture/compare preloaded control (TIM1 only)
        CCUS = (1UL << 2),      // Capture/compare control update selection (TIM1 only)
        CCDS = (1UL << 3),      // Capture/compare DMA selection
        MMS_RESET = (0UL << 4), // Master mode selection - Reset
        MMS_ENABLE = (1UL << 4), // Master mode selection - Enable
        MMS_UPDATE = (2UL << 4), // Master mode selection - Update
        MMS_COMPARE_PULSE = (3UL << 4), // Master mode selection - Compare pulse
        MMS_COMPARE_OC1REF = (4UL << 4), // Master mode selection - Compare OC1REF
        MMS_COMPARE_OC2REF = (5UL << 4), // Master mode selection - Compare OC2REF
        MMS_COMPARE_OC3REF = (6UL << 4), // Master mode selection - Compare OC3REF
        MMS_COMPARE_OC4REF = (7UL << 4), // Master mode selection - Compare OC4REF
        MMS_MSK = (7UL << 4),    // MMS mask
        TI1S = (1UL << 7)        // TI1 selection
    };

    // DIER register bit positions and masks
    enum class DIER : uint32_t {
        UIE = (1UL << 0),       // Update interrupt enable
        CC1IE = (1UL << 1),     // Capture/Compare 1 interrupt enable
        CC2IE = (1UL << 2),     // Capture/Compare 2 interrupt enable
        CC3IE = (1UL << 3),     // Capture/Compare 3 interrupt enable
        CC4IE = (1UL << 4),     // Capture/Compare 4 interrupt enable
        COMIE = (1UL << 5),     // COM interrupt enable (TIM1 only)
        TIE = (1UL << 6),       // Trigger interrupt enable
        BIE = (1UL << 7),       // Break interrupt enable (TIM1 only)
        UDE = (1UL << 8),       // Update DMA request enable
        CC1DE = (1UL << 9),     // Capture/Compare 1 DMA request enable
        CC2DE = (1UL << 10),    // Capture/Compare 2 DMA request enable
        CC3DE = (1UL << 11),    // Capture/Compare 3 DMA request enable
        CC4DE = (1UL << 12),    // Capture/Compare 4 DMA request enable
        COMDE = (1UL << 13),    // COM DMA request enable (TIM1 only)
        TDE = (1UL << 14)       // Trigger DMA request enable
    };

    // SR register bit positions and masks
    enum class SR : uint32_t {
        UIF = (1UL << 0),       // Update interrupt flag
        CC1IF = (1UL << 1),     // Capture/Compare 1 interrupt flag
        CC2IF = (1UL << 2),     // Capture/Compare 2 interrupt flag
        CC3IF = (1UL << 3),     // Capture/Compare 3 interrupt flag
        CC4IF = (1UL << 4),     // Capture/Compare 4 interrupt flag
        COMIF = (1UL << 5),     // COM interrupt flag (TIM1 only)
        TIF = (1UL << 6),       // Trigger interrupt flag
        BIF = (1UL << 7),       // Break interrupt flag (TIM1 only)
        CC1OF = (1UL << 9),     // Capture/Compare 1 overcapture flag
        CC2OF = (1UL << 10),    // Capture/Compare 2 overcapture flag
        CC3OF = (1UL << 11),    // Capture/Compare 3 overcapture flag
        CC4OF = (1UL << 12)     // Capture/Compare 4 overcapture flag
    };

    // EGR register bit positions and masks
    enum class EGR : uint32_t {
        UG = (1UL << 0),        // Update generation
        CC1G = (1UL << 1),      // Capture/Compare 1 generation
        CC2G = (1UL << 2),      // Capture/Compare 2 generation
        CC3G = (1UL << 3),      // Capture/Compare 3 generation
        CC4G = (1UL << 4),      // Capture/Compare 4 generation
        COMG = (1UL << 5),      // Capture/Compare control update generation (TIM1 only)
        TG = (1UL << 6),        // Trigger generation
        BG = (1UL << 7)         // Break generation (TIM1 only)
    };

    // CCMR1 register bit positions and masks (Output Mode)
    enum class CCMR1_Output : uint32_t {
        // CC1S (Capture/Compare 1 Selection)
        CC1S_OUTPUT = (0UL << 0),     // CC1 channel is configured as output
        CC1S_MSK = (3UL << 0),        // CC1S mask
        
        // Output Compare 1 settings
        OC1FE = (1UL << 2),           // Output Compare 1 fast enable
        OC1PE = (1UL << 3),           // Output Compare 1 preload enable
        OC1M_FROZEN = (0UL << 4),     // Frozen - no change on output
        OC1M_ACTIVE = (1UL << 4),     // Set channel to active on match
        OC1M_INACTIVE = (2UL << 4),   // Set channel to inactive on match
        OC1M_TOGGLE = (3UL << 4),     // Toggle
        OC1M_FORCE_INACTIVE = (4UL << 4), // Force inactive level
        OC1M_FORCE_ACTIVE = (5UL << 4),   // Force active level
        OC1M_PWM1 = (6UL << 4),       // PWM mode 1
        OC1M_PWM2 = (7UL << 4),       // PWM mode 2
        OC1M_MSK = (7UL << 4),        // OC1M mask
        OC1CE = (1UL << 7),           // Output Compare 1 clear enable
        
        // CC2S (Capture/Compare 2 Selection)
        CC2S_OUTPUT = (0UL << 8),     // CC2 channel is configured as output
        CC2S_MSK = (3UL << 8),        // CC2S mask
        
        // Output Compare 2 settings
        OC2FE = (1UL << 10),          // Output Compare 2 fast enable
        OC2PE = (1UL << 11),          // Output Compare 2 preload enable
        OC2M_FROZEN = (0UL << 12),    // Frozen - no change on output
        OC2M_ACTIVE = (1UL << 12),    // Set channel to active on match
        OC2M_INACTIVE = (2UL << 12),  // Set channel to inactive on match
        OC2M_TOGGLE = (3UL << 12),    // Toggle
        OC2M_FORCE_INACTIVE = (4UL << 12), // Force inactive level
        OC2M_FORCE_ACTIVE = (5UL << 12),   // Force active level
        OC2M_PWM1 = (6UL << 12),      // PWM mode 1
        OC2M_PWM2 = (7UL << 12),      // PWM mode 2
        OC2M_MSK = (7UL << 12),       // OC2M mask
        OC2CE = (1UL << 15)           // Output Compare 2 clear enable
    };
    enum class TimerEvent {
        Update,     // Update event
        CC1,        // Capture/Compare 1 event
        CC2,        // Capture/Compare 2 event
        CC3,        // Capture/Compare 3 event
        CC4,        // Capture/Compare 4 event
        Trigger,    // Trigger event
        Break,      // Break event
        Count,      // number of events
    };
    // CCMR1 register bit positions and masks (Input Mode)
    enum class CCMR1_Input : uint32_t {
        // CC1S (Capture/Compare 1 Selection)
        CC1S_TI1 = (1UL << 0),        // CC1 channel is configured as input, IC1 is mapped on TI1
        CC1S_TI2 = (2UL << 0),        // CC1 channel is configured as input, IC1 is mapped on TI2
        CC1S_TRC = (3UL << 0),        // CC1 channel is configured as input, IC1 is mapped on TRC
        CC1S_MSK = (3UL << 0),        // CC1S mask
        
        // Input Capture 1 settings
        IC1PSC_DIV1 = (0UL << 2),     // No prescaler
        IC1PSC_DIV2 = (1UL << 2),     // Capture once every 2 events
        IC1PSC_DIV4 = (2UL << 2),     // Capture once every 4 events
        IC1PSC_DIV8 = (3UL << 2),     // Capture once every 8 events
        IC1PSC_MSK = (3UL << 2),      // IC1PSC mask
        
        IC1F_NOFILTER = (0UL << 4),   // No filter
        IC1F_CK_INT_N2 = (1UL << 4),  // fSAMPLING = fCK_INT, N = 2
        IC1F_CK_INT_N4 = (2UL << 4),  // fSAMPLING = fCK_INT, N = 4
        IC1F_CK_INT_N8 = (3UL << 4),  // fSAMPLING = fCK_INT, N = 8
        IC1F_DTS_DIV2_N6 = (4UL << 4), // fSAMPLING = fDTS / 2, N = 6
        IC1F_DTS_DIV2_N8 = (5UL << 4), // fSAMPLING = fDTS / 2, N = 8
        IC1F_DTS_DIV4_N6 = (6UL << 4), // fSAMPLING = fDTS / 4, N = 6
        IC1F_DTS_DIV4_N8 = (7UL << 4), // fSAMPLING = fDTS / 4, N = 8
        IC1F_DTS_DIV8_N6 = (8UL << 4), // fSAMPLING = fDTS / 8, N = 6
        IC1F_DTS_DIV8_N8 = (9UL << 4), // fSAMPLING = fDTS / 8, N = 8
        IC1F_DTS_DIV16_N5 = (10UL << 4), // fSAMPLING = fDTS / 16, N = 5
        IC1F_DTS_DIV16_N6 = (11UL << 4), // fSAMPLING = fDTS / 16, N = 6
        IC1F_DTS_DIV16_N8 = (12UL << 4), // fSAMPLING = fDTS / 16, N = 8
        IC1F_DTS_DIV32_N5 = (13UL << 4), // fSAMPLING = fDTS / 32, N = 5
        IC1F_DTS_DIV32_N6 = (14UL << 4), // fSAMPLING = fDTS / 32, N = 6
        IC1F_DTS_DIV32_N8 = (15UL << 4), // fSAMPLING = fDTS / 32, N = 8
        IC1F_MSK = (15UL << 4),       // IC1F mask
        
        // CC2S (Capture/Compare 2 Selection)
        CC2S_TI2 = (1UL << 8),        // CC2 channel is configured as input, IC2 is mapped on TI2
        CC2S_TI1 = (2UL << 8),        // CC2 channel is configured as input, IC2 is mapped on TI1
        CC2S_TRC = (3UL << 8),        // CC2 channel is configured as input, IC2 is mapped on TRC
        CC2S_MSK = (3UL << 8),        // CC2S mask
        
        // Input Capture 2 settings
        IC2PSC_DIV1 = (0UL << 10),    // No prescaler
        IC2PSC_DIV2 = (1UL << 10),    // Capture once every 2 events
        IC2PSC_DIV4 = (2UL << 10),    // Capture once every 4 events
        IC2PSC_DIV8 = (3UL << 10),    // Capture once every 8 events
        IC2PSC_MSK = (3UL << 10),     // IC2PSC mask
        
        IC2F_NOFILTER = (0UL << 12),  // No filter
        IC2F_CK_INT_N2 = (1UL << 12), // fSAMPLING = fCK_INT, N = 2
        IC2F_CK_INT_N4 = (2UL << 12), // fSAMPLING = fCK_INT, N = 4
        IC2F_CK_INT_N8 = (3UL << 12), // fSAMPLING = fCK_INT, N = 8
        IC2F_DTS_DIV2_N6 = (4UL << 12), // fSAMPLING = fDTS / 2, N = 6
        IC2F_DTS_DIV2_N8 = (5UL << 12), // fSAMPLING = fDTS / 2, N = 8
        IC2F_DTS_DIV4_N6 = (6UL << 12), // fSAMPLING = fDTS / 4, N = 6
        IC2F_DTS_DIV4_N8 = (7UL << 12), // fSAMPLING = fDTS / 4, N = 8
        IC2F_DTS_DIV8_N6 = (8UL << 12), // fSAMPLING = fDTS / 8, N = 6
        IC2F_DTS_DIV8_N8 = (9UL << 12), // fSAMPLING = fDTS / 8, N = 8
        IC2F_DTS_DIV16_N5 = (10UL << 12), // fSAMPLING = fDTS / 16, N = 5
        IC2F_DTS_DIV16_N6 = (11UL << 12), // fSAMPLING = fDTS / 16, N = 6
        IC2F_DTS_DIV16_N8 = (12UL << 12), // fSAMPLING = fDTS / 16, N = 8
        IC2F_DTS_DIV32_N5 = (13UL << 12), // fSAMPLING = fDTS / 32, N = 5
        IC2F_DTS_DIV32_N6 = (14UL << 12), // fSAMPLING = fDTS / 32, N = 6
        IC2F_DTS_DIV32_N8 = (15UL << 12), // fSAMPLING = fDTS / 32, N = 8
        IC2F_MSK = (15UL << 12)       // IC2F mask
    };

    // CCMR2 register bit positions and masks (Output Mode)
    enum class CCMR2_Output : uint32_t {
        // CC3S (Capture/Compare 3 Selection)
        CC3S_OUTPUT = (0UL << 0),     // CC3 channel is configured as output
        CC3S_MSK = (3UL << 0),        // CC3S mask
        
        // Output Compare 3 settings
        OC3FE = (1UL << 2),           // Output Compare 3 fast enable
        OC3PE = (1UL << 3),           // Output Compare 3 preload enable
        OC3M_FROZEN = (0UL << 4),     // Frozen - no change on output
        OC3M_ACTIVE = (1UL << 4),     // Set channel to active on match
        OC3M_INACTIVE = (2UL << 4),   // Set channel to inactive on match
        OC3M_TOGGLE = (3UL << 4),     // Toggle
        OC3M_FORCE_INACTIVE = (4UL << 4), // Force inactive level
        OC3M_FORCE_ACTIVE = (5UL << 4),   // Force active level
        OC3M_PWM1 = (6UL << 4),       // PWM mode 1
        OC3M_PWM2 = (7UL << 4),       // PWM mode 2
        OC3M_MSK = (7UL << 4),        // OC3M mask
        OC3CE = (1UL << 7),           // Output Compare 3 clear enable
        
        // CC4S (Capture/Compare 4 Selection)
        CC4S_OUTPUT = (0UL << 8),     // CC4 channel is configured as output
        CC4S_MSK = (3UL << 8),        // CC4S mask
        
        // Output Compare 4 settings
        OC4FE = (1UL << 10),          // Output Compare 4 fast enable
        OC4PE = (1UL << 11),          // Output Compare 4 preload enable
        OC4M_FROZEN = (0UL << 12),    // Frozen - no change on output
        OC4M_ACTIVE = (1UL << 12),    // Set channel to active on match
        OC4M_INACTIVE = (2UL << 12),  // Set channel to inactive on match
        OC4M_TOGGLE = (3UL << 12),    // Toggle
        OC4M_FORCE_INACTIVE = (4UL << 12), // Force inactive level
        OC4M_FORCE_ACTIVE = (5UL << 12),   // Force active level
        OC4M_PWM1 = (6UL << 12),      // PWM mode 1
        OC4M_PWM2 = (7UL << 12),      // PWM mode 2
        OC4M_MSK = (7UL << 12),       // OC4M mask
        OC4CE = (1UL << 15)           // Output Compare 4 clear enable
    };

    // CCMR2 register bit positions and masks (Input Mode)
    enum class CCMR2_Input : uint32_t {
        // CC3S (Capture/Compare 3 Selection)
        CC3S_TI3 = (1UL << 0),        // CC3 channel is configured as input, IC3 is mapped on TI3
        CC3S_TI4 = (2UL << 0),        // CC3 channel is configured as input, IC3 is mapped on TI4
        CC3S_TRC = (3UL << 0),        // CC3 channel is configured as input, IC3 is mapped on TRC
        CC3S_MSK = (3UL << 0),        // CC3S mask
        
        // Input Capture 3 settings
        IC3PSC_DIV1 = (0UL << 2),     // No prescaler
        IC3PSC_DIV2 = (1UL << 2),     // Capture once every 2 events
        IC3PSC_DIV4 = (2UL << 2),     // Capture once every 4 events
        IC3PSC_DIV8 = (3UL << 2),     // Capture once every 8 events
        IC3PSC_MSK = (3UL << 2),      // IC3PSC mask
        
        IC3F_NOFILTER = (0UL << 4),   // No filter
        IC3F_CK_INT_N2 = (1UL << 4),  // fSAMPLING = fCK_INT, N = 2
        IC3F_CK_INT_N4 = (2UL << 4),  // fSAMPLING = fCK_INT, N = 4
        IC3F_CK_INT_N8 = (3UL << 4),  // fSAMPLING = fCK_INT, N = 8
        IC3F_DTS_DIV2_N6 = (4UL << 4), // fSAMPLING = fDTS / 2, N = 6
        IC3F_DTS_DIV2_N8 = (5UL << 4), // fSAMPLING = fDTS / 2, N = 8
        IC3F_DTS_DIV4_N6 = (6UL << 4), // fSAMPLING = fDTS / 4, N = 6
        IC3F_DTS_DIV4_N8 = (7UL << 4), // fSAMPLING = fDTS / 4, N = 8
        IC3F_DTS_DIV8_N6 = (8UL << 4), // fSAMPLING = fDTS / 8, N = 6
        IC3F_DTS_DIV8_N8 = (9UL << 4), // fSAMPLING = fDTS / 8, N = 8
        IC3F_DTS_DIV16_N5 = (10UL << 4), // fSAMPLING = fDTS / 16, N = 5
        IC3F_DTS_DIV16_N6 = (11UL << 4), // fSAMPLING = fDTS / 16, N = 6
        IC3F_DTS_DIV16_N8 = (12UL << 4), // fSAMPLING = fDTS / 16, N = 8
        IC3F_DTS_DIV32_N5 = (13UL << 4), // fSAMPLING = fDTS / 32, N = 5
        IC3F_DTS_DIV32_N6 = (14UL << 4), // fSAMPLING = fDTS / 32, N = 6
        IC3F_DTS_DIV32_N8 = (15UL << 4), // fSAMPLING = fDTS / 32, N = 8
        IC3F_MSK = (15UL << 4),       // IC3F mask
        
        // CC4S (Capture/Compare 4 Selection)
        CC4S_TI4 = (1UL << 8),        // CC4 channel is configured as input, IC4 is mapped on TI4
        CC4S_TI3 = (2UL << 8),        // CC4 channel is configured as input, IC4 is mapped on TI3
        CC4S_TRC = (3UL << 8),        // CC4 channel is configured as input, IC4 is mapped on TRC
        CC4S_MSK = (3UL << 8),        // CC4S mask
        
        // Input Capture 4 settings
        IC4PSC_DIV1 = (0UL << 10),    // No prescaler
        IC4PSC_DIV2 = (1UL << 10),    // Capture once every 2 events
        IC4PSC_DIV4 = (2UL << 10),    // Capture once every 4 events
        IC4PSC_DIV8 = (3UL << 10),    // Capture once every 8 events
        IC4PSC_MSK = (3UL << 10),     // IC4PSC mask
        
        IC4F_NOFILTER = (0UL << 12),  // No filter
        IC4F_CK_INT_N2 = (1UL << 12), // fSAMPLING = fCK_INT, N = 2
        IC4F_CK_INT_N4 = (2UL << 12), // fSAMPLING = fCK_INT, N = 4
        IC4F_CK_INT_N8 = (3UL << 12), // fSAMPLING = fCK_INT, N = 8
        IC4F_DTS_DIV2_N6 = (4UL << 12), // fSAMPLING = fDTS / 2, N = 6
        IC4F_DTS_DIV2_N8 = (5UL << 12), // fSAMPLING = fDTS / 2, N = 8
        IC4F_DTS_DIV4_N6 = (6UL << 12), // fSAMPLING = fDTS / 4, N = 6
        IC4F_DTS_DIV4_N8 = (7UL << 12), // fSAMPLING = fDTS / 4, N = 8
        IC4F_DTS_DIV8_N6 = (8UL << 12), // fSAMPLING = fDTS / 8, N = 6
        IC4F_DTS_DIV8_N8 = (9UL << 12), // fSAMPLING = fDTS / 8, N = 8
        IC4F_DTS_DIV16_N5 = (10UL << 12), // fSAMPLING = fDTS / 16, N = 5
        IC4F_DTS_DIV16_N6 = (11UL << 12), // fSAMPLING = fDTS / 16, N = 6
        IC4F_DTS_DIV16_N8 = (12UL << 12), // fSAMPLING = fDTS / 16, N = 8
        IC4F_DTS_DIV32_N5 = (13UL << 12), // fSAMPLING = fDTS / 32, N = 5
        IC4F_DTS_DIV32_N6 = (14UL << 12), // fSAMPLING = fDTS / 32, N = 6
        IC4F_DTS_DIV32_N8 = (15UL << 12), // fSAMPLING = fDTS / 32, N = 8
        IC4F_MSK = (15UL << 12)       // IC4F mask
    };
    
    // CCER register bit positions and masks
    enum class CCER : uint32_t {
        CC1E = (1UL << 0),            // Capture/Compare 1 output enable
        CC1P = (1UL << 1),            // Capture/Compare 1 output polarity
        CC1NE = (1UL << 2),           // Capture/Compare 1 complementary output enable (TIM1 only)
        CC1NP = (1UL << 3),           // Capture/Compare 1 complementary output polarity (TIM1 only)
        CC2E = (1UL << 4),            // Capture/Compare 2 output enable
        CC2P = (1UL << 5),            // Capture/Compare 2 output polarity
        CC2NE = (1UL << 6),           // Capture/Compare 2 complementary output enable (TIM1 only)
        CC2NP = (1UL << 7),           // Capture/Compare 2 complementary output polarity (TIM1 only)
        CC3E = (1UL << 8),            // Capture/Compare 3 output enable
        CC3P = (1UL << 9),            // Capture/Compare 3 output polarity
        CC3NE = (1UL << 10),          // Capture/Compare 3 complementary output enable (TIM1 only)
        CC3NP = (1UL << 11),          // Capture/Compare 3 complementary output polarity (TIM1 only)
        CC4E = (1UL << 12),           // Capture/Compare 4 output enable
        CC4P = (1UL << 13),           // Capture/Compare 4 output polarity
        CC4NP = (1UL << 15)           // Capture/Compare 4 complementary output polarity (TIM1 only)
    };
    
    // BDTR register bit positions and masks (TIM1 only)
    enum class BDTR : uint32_t {
        DTG_MSK = (0xFF << 0),        // Dead-time generator setup mask
        LOCK_OFF = (0UL << 8),        // No lock
        LOCK_LEVEL1 = (1UL << 8),     // Lock level 1
        LOCK_LEVEL2 = (2UL << 8),     // Lock level 2
        LOCK_LEVEL3 = (3UL << 8),     // Lock level 3
        LOCK_MSK = (3UL << 8),        // Lock configuration mask
        OSSI = (1UL << 10),           // Off-state selection for Idle mode
        OSSR = (1UL << 11),           // Off-state selection for Run mode
        BKE = (1UL << 12),            // Break enable
        BKP = (1UL << 13),            // Break polarity
        AOE = (1UL << 14),            // Automatic output enable
        MOE = (1UL << 15)             // Main output enable
    };
    
    // Timer type enumeration
    enum class TimerType {
        Advanced,       // TIM1
        GeneralPurpose, // TIM2-TIM5
        Basic           // TIM9-TIM11
    };
    
    // Helper functions
    
    // Get bit value from enum class element
    template<typename T>
    constexpr uint32_t getBitValue(T bit) {
        return static_cast<uint32_t>(bit);
    }
    
    // Operator overloads for bit manipulation with enum classes
    template<typename T>
    constexpr T operator|(T a, T b) {
        return static_cast<T>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    }
    
    template<typename T>
    constexpr T operator&(T a, T b) {
        return static_cast<T>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
    }
    
    template<typename T>
    constexpr T operator~(T a) {
        return static_cast<T>(~static_cast<uint32_t>(a));
    }
    
    // Function to get timer register base pointer from timer instance number
    inline Registers* getTimer(uint8_t instance) {
        switch (instance) {
            case 1: return reinterpret_cast<Registers*>(TIM1_BASE);
            case 2: return reinterpret_cast<Registers*>(TIM2_BASE);
            case 3: return reinterpret_cast<Registers*>(TIM3_BASE);
            case 4: return reinterpret_cast<Registers*>(TIM4_BASE);
            case 5: return reinterpret_cast<Registers*>(TIM5_BASE);
            case 9: return reinterpret_cast<Registers*>(TIM9_BASE);
            case 10: return reinterpret_cast<Registers*>(TIM10_BASE);
            case 11: return reinterpret_cast<Registers*>(TIM11_BASE);
            default: return nullptr;
        }
    }
    
    // Function to determine timer type from timer instance
    inline TimerType getTimerType(uint8_t instance) {
        if (instance == 1) {
            return TimerType::Advanced;
        } else if (instance >= 2 && instance <= 5) {
            return TimerType::GeneralPurpose;
        } else if (instance >= 9 && instance <= 11) {
            return TimerType::Basic;
        } else {
            // Default to basic timer for unknown instances
            return TimerType::Basic;
        }
    }
    
    // Function to check if a timer instance is valid
    inline bool isValidTimerInstance(uint8_t instance) {
        return (instance == 1) || 
            (instance >= 2 && instance <= 5) || 
            (instance >= 9 && instance <= 11);
    }
    
    // Function to get number of channels supported by timer instance
    inline uint8_t getTimerChannelCount(uint8_t instance) {
        switch (instance) {
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
                return 4;  // TIM1-TIM5 support 4 channels
            case 9:
                return 2;  // TIM9 supports 2 channels
            case 10:
            case 11:
                return 1;  // TIM10, TIM11 support 1 channel
            default:
                return 0;
        }
    }
    
    // Function to check if timer supports 32-bit counter
    inline bool hasTimer32BitCounter(uint8_t instance) {
        return (instance == 2 || instance == 5);  // TIM2 and TIM5 are 32-bit
    }
    
    } // namespace TIM
 } // namespace Platform
 
 #endif // PLATFORM_TIM_HPP