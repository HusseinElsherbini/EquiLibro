#pragma once

#include "platform.hpp"

namespace Platform {   
   namespace ADC {
        constexpr uint32_t ADC1_BASE = (APB2PERIPH_BASE + 0x2000UL);
        constexpr uint32_t ADC_COMMON_BASE = (ADC1_BASE + 0x300UL);

        // ADC channel definitions
        enum class Channel : uint32_t {
            Channel0 = 0UL,
            Channel1 = 1UL,
            Channel2 = 2UL,
            Channel3 = 3UL,
            Channel4 = 4UL,
            Channel5 = 5UL,
            Channel6 = 6UL,
            Channel7 = 7UL,
            Channel8 = 8UL,
            Channel9 = 9UL,
            Channel10 = 10UL,
            Channel11 = 11UL,
            Channel12 = 12UL,
            Channel13 = 13UL,
            Channel14 = 14UL,
            Channel15 = 15UL,
            Channel16 = 16UL,      // Temperature sensor
            Channel17 = 17UL,      // VREFINT
            Channel18 = 18UL       // VBAT
        };

        // Sample time definitions
        enum class SampleTime : uint32_t {
            Cycles3 = 0UL,
            Cycles15 = 1UL,
            Cycles28 = 2UL,
            Cycles56 = 3UL,
            Cycles84 = 4UL,
            Cycles112 = 5UL,
            Cycles144 = 6UL,
            Cycles480 = 7UL
        };

        // Resolution definitions
        enum class Resolution : uint32_t {
            Bits12 = 0UL,
            Bits10 = 1UL,
            Bits8 = 2UL,
            Bits6 = 3UL
        };

        // Data alignment
        enum class Alignment : uint32_t {
            Right = 0UL,
            Left = 1UL
        };

        // External trigger edge
        enum class ExternalTriggerEdge : uint32_t {
            Disabled = 0UL,
            RisingEdge = 1UL,
            FallingEdge = 2UL,
            BothEdges = 3UL
        };

        // External trigger sources for regular group
        enum class ExternalTrigger : uint32_t {
            Timer1_CC1 = 0UL,
            Timer1_CC2 = 1UL,
            Timer1_CC3 = 2UL,
            Timer2_CC2 = 3UL,
            Timer2_CC3 = 4UL,
            Timer2_CC4 = 5UL,
            Timer2_TRGO = 6UL,
            Timer3_CC1 = 7UL,
            Timer3_TRGO = 8UL,
            Timer4_CC4 = 9UL,
            Timer5_CC1 = 10UL,
            Timer5_CC2 = 11UL,
            Timer5_CC3 = 12UL,
            Timer8_CC1 = 13UL,
            Timer8_TRGO = 14UL,
            EXTI_Line11 = 15UL
        };

        // ADC register structure
        struct ADC_Registers {
            volatile uint32_t SR;           // ADC status register
            volatile uint32_t CR1;          // ADC control register 1
            volatile uint32_t CR2;          // ADC control register 2
            volatile uint32_t SMPR1;        // ADC sample time register 1
            volatile uint32_t SMPR2;        // ADC sample time register 2
            volatile uint32_t JOFR1;        // ADC injected channel data offset register 1
            volatile uint32_t JOFR2;        // ADC injected channel data offset register 2
            volatile uint32_t JOFR3;        // ADC injected channel data offset register 3
            volatile uint32_t JOFR4;        // ADC injected channel data offset register 4
            volatile uint32_t HTR;          // ADC watchdog higher threshold register
            volatile uint32_t LTR;          // ADC watchdog lower threshold register
            volatile uint32_t SQR1;         // ADC regular sequence register 1
            volatile uint32_t SQR2;         // ADC regular sequence register 2
            volatile uint32_t SQR3;         // ADC regular sequence register 3
            volatile uint32_t JSQR;         // ADC injected sequence register
            volatile uint32_t JDR1;         // ADC injected data register 1
            volatile uint32_t JDR2;         // ADC injected data register 2
            volatile uint32_t JDR3;         // ADC injected data register 3
            volatile uint32_t JDR4;         // ADC injected data register 4
            volatile uint32_t DR;           // ADC regular data register
        };

        // ADC common registers structure
        struct ADC_Common_Registers {
            volatile uint32_t CSR;          // ADC common status register
            volatile uint32_t CCR;          // ADC common control register
            volatile uint32_t CDR;          // ADC common regular data register for dual and triple modes
        };

        // ADC_SR register bits
        enum class SR : uint32_t {
            AWD = (1UL << 0),        // Analog watchdog flag
            EOC = (1UL << 1),        // End of conversion
            JEOC = (1UL << 2),       // End of injected conversion
            JSTRT = (1UL << 3),      // Injected channel start flag
            STRT = (1UL << 4),       // Regular channel start flag
            OVR = (1UL << 5)         // Overrun
        };

        // ADC_CR1 register bits
        enum class CR1 : uint32_t {
            AWDCH_MSK = (0x1FUL << 0),   // Analog watchdog channel select mask
            EOCIE = (1UL << 5),          // Interrupt enable for EOC
            AWDIE = (1UL << 6),          // Analog watchdog interrupt enable
            JEOCIE = (1UL << 7),         // Interrupt enable for JEOC
            SCAN = (1UL << 8),           // Scan mode
            AWDSGL = (1UL << 9),         // Enable watchdog on a single channel in scan mode
            JAUTO = (1UL << 10),         // Automatic injected group conversion
            DISCEN = (1UL << 11),        // Discontinuous mode on regular channels
            JDISCEN = (1UL << 12),       // Discontinuous mode on injected channels
            DISCNUM_MSK = (0x7UL << 13), // Discontinuous mode channel count mask
            JAWDEN = (1UL << 22),        // Analog watchdog enable on injected channels
            AWDEN = (1UL << 23),         // Analog watchdog enable on regular channels
            RES_12BIT = (0UL << 24),     // 12-bit resolution
            RES_10BIT = (1UL << 24),     // 10-bit resolution
            RES_8BIT = (2UL << 24),      // 8-bit resolution
            RES_6BIT = (3UL << 24),      // 6-bit resolution
            RES_MSK = (3UL << 24),       // Resolution mask
            OVRIE = (1UL << 26)          // Overrun interrupt enable
        };

        // ADC_CR2 register bits
        enum class CR2 : uint32_t {
            ADON = (1UL << 0),           // A/D converter ON / OFF
            CONT = (1UL << 1),           // Continuous conversion
            DMA = (1UL << 8),            // Direct memory access mode
            DDS = (1UL << 9),            // DMA disable selection
            EOCS = (1UL << 10),          // End of conversion selection
            ALIGN = (1UL << 11),         // Data alignment
            JEXTSEL_MSK = (0xFUL << 16), // External event select for injected group mask
            JEXTEN_MSK = (0x3UL << 20),  // External trigger enable for injected channels mask
            JSWSTART = (1UL << 22),      // Start conversion of injected channels
            EXTSEL_MSK = (0xFUL << 24),  // External event select for regular group mask
            EXTEN_MSK = (0x3UL << 28),   // External trigger enable for regular channels mask
            SWSTART = (1UL << 30)        // Start conversion of regular channels
        };

        // ADC_CCR register bits (Common)
        enum class CCR : uint32_t {
            ADCPRE_DIV2 = (0UL << 16),  // PCLK2 divided by 2
            ADCPRE_DIV4 = (1UL << 16),  // PCLK2 divided by 4
            ADCPRE_DIV6 = (2UL << 16),  // PCLK2 divided by 6
            ADCPRE_DIV8 = (3UL << 16),  // PCLK2 divided by 8
            ADCPRE_MSK = (3UL << 16),   // ADC prescaler mask
            VBATE = (1UL << 22),        // VBAT enable
            TSVREFE = (1UL << 23)       // Temperature sensor and VREFINT enable
        };

        // Helper methods for configuring sample time registers
        constexpr uint32_t getSMPR1SampleTimeBits(Channel channel, SampleTime time) {
            // SMPR1 handles channels 10-18
            uint8_t chIdx = static_cast<uint8_t>(channel);
            if (chIdx < 10 || chIdx > 18) {
                return 0; // Not in this register
            }
            // Each channel has 3 bits, positioned starting from LSB
            uint8_t position = (chIdx - 10) * 3;
            return (static_cast<uint32_t>(time) << position);
        }

        constexpr uint32_t getSMPR2SampleTimeBits(Channel channel, SampleTime time) {
            // SMPR2 handles channels 0-9
            uint8_t chIdx = static_cast<uint8_t>(channel);
            if (chIdx > 9) {
                return 0; // Not in this register
            }
            // Each channel has 3 bits, positioned starting from LSB
            uint8_t position = chIdx * 3;
            return (static_cast<uint32_t>(time) << position);
        }

        // Helper methods for configuring regular sequence registers
        constexpr uint32_t setSQR1SequenceBits(uint8_t sqr, Channel channel) {
            // SQR1 handles 13th-16th conversion in sequence
            if (sqr < 13 || sqr > 16) {
                return 0; // Not in this register
            }
            uint8_t position = (16 - sqr) * 5; // Positions from right to left
            return (static_cast<uint32_t>(channel) << position);
        }

        constexpr uint32_t setSQR2SequenceBits(uint8_t sqr, Channel channel) {
            // SQR2 handles 7th-12th conversion in sequence
            if (sqr < 7 || sqr > 12) {
                return 0; // Not in this register
            }
            uint8_t position = (12 - sqr) * 5; // Positions from right to left
            return (static_cast<uint32_t>(channel) << position);
        }

        constexpr uint32_t setSQR3SequenceBits(uint8_t sqr, Channel channel) {
            // SQR3 handles 1st-6th conversion in sequence
            if (sqr < 1 || sqr > 6) {
                return 0; // Not in this register
            }
            uint8_t position = (6 - sqr) * 5; // Positions from right to left
            return (static_cast<uint32_t>(channel) << position);
        }

        // Set sequence length (1-16 conversions) in SQR1
        constexpr uint32_t setSequenceLength(uint8_t length) {
            // Length is stored in bits 20-23 of SQR1
            // Value stored is length-1
            if (length < 1 || length > 16) {
                return 0; // Invalid length
            }
            return ((length - 1) << 20);
        }

        // Get ADC registers
        inline ADC_Registers* getADC1Registers() {
            return reinterpret_cast<ADC_Registers*>(ADC1_BASE);
        }

        // Get ADC common registers
        inline ADC_Common_Registers* getCommonRegisters() {
            return reinterpret_cast<ADC_Common_Registers*>(ADC_COMMON_BASE);
        }

        // Helper functions for bit manipulation
        constexpr uint32_t getBitValue(SR bit) {
            return static_cast<uint32_t>(bit);
        }

        constexpr uint32_t getBitValue(CR1 bit) {
            return static_cast<uint32_t>(bit);
        }

        constexpr uint32_t getBitValue(CR2 bit) {
            return static_cast<uint32_t>(bit);
        }

        constexpr uint32_t getBitValue(CCR bit) {
            return static_cast<uint32_t>(bit);
        }

        // Operator overloads for combining flags
        constexpr SR operator|(SR a, SR b) {
            return static_cast<SR>(
                static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
        }

        constexpr CR1 operator|(CR1 a, CR1 b) {
            return static_cast<CR1>(
                static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
        }

        constexpr CR2 operator|(CR2 a, CR2 b) {
            return static_cast<CR2>(
                static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
        }

        constexpr CCR operator|(CCR a, CCR b) {
            return static_cast<CCR>(
                static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
        }
    }
}