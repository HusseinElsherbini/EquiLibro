// adc.cpp - Implementation of ADC interface
#include "hardware_abstraction/adc.hpp"
#include "hardware_abstraction/gpio.hpp"
#include "hardware_abstraction/rcc.hpp"
#include "common/platform_adc.hpp"  
#include "middleware/system_services/system_timing.hpp"
#include "os/mutex.hpp"
#include <algorithm>
#include "common/platform_adc.hpp"
#include "common/platform_cmsis.hpp"

namespace Platform {
namespace ADC {

// Initialize static members
AdcInterface* AdcInterface::instances[ADC_INSTANCE_COUNT] = {nullptr, nullptr, nullptr};
OS::mutex AdcInterface::instances_mutex;

// Constructor
AdcInterface::AdcInterface() 
    : initialized(false),
      adc_instance(AdcInstance::ADC1) {
    
    // Initialize callbacks
    for (auto& callback : callbacks) {
        callback.callback = nullptr;
        callback.param = nullptr;
        callback.enabled = false;
    }
}

// Destructor
AdcInterface::~AdcInterface() {
    if (initialized) {
        DeInit();
    }
}

// Static method to get ADC instance
AdcInterface& AdcInterface::GetInstance(AdcInstance instance) {
    // Validate instance
    if (static_cast<uint8_t>(instance) < 1 || static_cast<uint8_t>(instance) > ADC_INSTANCE_COUNT) {
        // Default to ADC1 if invalid
        instance = AdcInstance::ADC1;
    }
    
    // Get the zero-based instance index
    uint8_t index = static_cast<uint8_t>(instance) - 1;
    
    // Use mutex for thread safety
    OS::lock_guard<OS::mutex> lock(instances_mutex);
    
    // Create instance if it doesn't exist
    if (instances[index] == nullptr) {
        static AdcInterface adc_instances[ADC_INSTANCE_COUNT];
        instances[index] = &adc_instances[index];
        instances[index]->adc_instance = instance;
    }
    
    return *instances[index];
}

// Initialize the ADC
Platform::Status AdcInterface::Init(void* config_ptr) {
    // Validate parameters
    if (config_ptr == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // If already initialized, deinitialize first
    if (initialized) {
        DeInit();
    }
    
    // Copy configuration
    config = *static_cast<AdcConfig*>(config_ptr);
    
    // Enable ADC clock
    RCC::RccInterface& rcc = RCC::RccInterface::GetInstance();
    RCC::RccPeripheral adc_clock;
    
    switch (config.adc_instance) {
        case AdcInstance::ADC1:
            adc_clock = RCC::RccPeripheral::ADC1;
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Enable peripheral clock
    Platform::Status status = rcc.EnablePeripheralClock(adc_clock);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Get ADC registers
    Platform::ADC::Registers* adc_regs = nullptr;
    Platform::ADC::ADC_Common_Registers* adc_common_regs = getCommonRegisters();
    switch (config.adc_instance) {
        case AdcInstance::ADC1:
            adc_regs = Platform::ADC::getADC1Registers();
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    if (adc_regs == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Reset ADC configuration
    adc_regs->CR1 = 0;
    adc_regs->CR2 = 0;
    
    // Configure resolution
    ConfigureResolution(config.resolution);
    
    // Configure scan mode
    if (config.scan_mode == AdcScanMode::MultiChannel) {
        adc_regs->CR1 |= Platform::ADC::getBitValue(Platform::ADC::CR1::SCAN);
    }
    
    // Configure conversion mode
    if (config.conv_mode == AdcConversionMode::Continuous) {
        adc_regs->CR2 |= Platform::ADC::getBitValue(Platform::ADC::CR2::CONT);
    }
    
    // Configure external trigger
    if (config.trigger_source != AdcTriggerSource::Software) {
        uint32_t extsel = static_cast<uint32_t>(config.trigger_source) << 24;
        adc_regs->CR2 |= extsel & Platform::ADC::getBitValue(Platform::ADC::CR2::EXTSEL_MSK);
        
        // Configure trigger edge
        uint32_t exten = static_cast<uint32_t>(config.trigger_edge) << 28;
        adc_regs->CR2 |= exten & Platform::ADC::getBitValue(Platform::ADC::CR2::EXTEN_MSK);
    }
    
    // Configure channels
    for (uint8_t i = 0; i < config.num_channels; i++) {
        const AdcChannelConfig& channel_config = config.channel_config[i];
        
        // Configure GPIO for external channels
        if (channel_config.channel <= AdcChannel::Channel15) {
            ConfigureGPIO(channel_config.channel);
        }
        
        // Configure sampling time
        ConfigureSamplingTime(channel_config.channel, channel_config.sampling_time);
        
        // Configure analog watchdog if enabled
        if (channel_config.use_watchdog) {
            ConfigureWatchdog(channel_config.channel, 
                             channel_config.watchdog_low_threshold, 
                             channel_config.watchdog_high_threshold);
        }
        
        // Add channel to sequence
        uint8_t sequence_position = i + 1; // 1-based position
        
        // Configure channel sequence
        if (sequence_position <= 6) {
            // SQ1-SQ6 are in SQR3
            uint32_t shift = (sequence_position - 1) * 5;
            uint32_t channel_value = static_cast<uint32_t>(channel_config.channel) << shift;
            adc_regs->SQR3 &= ~(0x1F << shift);
            adc_regs->SQR3 |= channel_value;
        } else if (sequence_position <= 12) {
            // SQ7-SQ12 are in SQR2
            uint32_t shift = (sequence_position - 7) * 5;
            uint32_t channel_value = static_cast<uint32_t>(channel_config.channel) << shift;
            adc_regs->SQR2 &= ~(0x1F << shift);
            adc_regs->SQR2 |= channel_value;
        } else {
            // SQ13-SQ16 are in SQR1
            uint32_t shift = (sequence_position - 13) * 5;
            uint32_t channel_value = static_cast<uint32_t>(channel_config.channel) << shift;
            adc_regs->SQR1 &= ~(0x1F << shift);
            adc_regs->SQR1 |= channel_value;
        }
    }
    

    // Clear the sequence length bits
    adc_regs->SQR1 &= ~Platform::ADC::SQR1_Bits::L_MSK;

    // Set new sequence length
    adc_regs->SQR1 |= ((config.num_channels - 1) << Platform::ADC::SQR1_Bits::L_POS) & Platform::ADC::SQR1_Bits::L_MSK;
    
    // Configure special channels if needed
    if (config.enable_temp_sensor || config.enable_vrefint) {
        // Enable temperature sensor and Vrefint
        adc_common_regs->CCR |= Platform::ADC::getBitValue(Platform::ADC::CCR::TSVREFE);
    }
    
    if (config.enable_vbat) {
        // Enable battery voltage measurement
        adc_common_regs->CCR |= Platform::ADC::getBitValue(Platform::ADC::CCR::VBATE);
    }
    
    // Configure DMA if needed
    if (config.enable_dma) {
        adc_regs->CR2 |= Platform::ADC::getBitValue(Platform::ADC::CR2::DMA);
        
        // Configure DMA for continuous requests if in continuous mode
        if (config.conv_mode == AdcConversionMode::Continuous) {
            adc_regs->CR2 |= Platform::ADC::getBitValue(Platform::ADC::CR2::DDS);
        }
        
        // Note: Additional DMA configuration would be needed here
        // This example doesn't include the DMA setup
    }
    
    // Configure overrun detection
    if (config.enable_overrun_detection) {
        adc_regs->CR1 |= Platform::ADC::getBitValue(Platform::ADC::CR1::OVRIE);
    } else {
        // Disable overrun detection (overwrite previous data)
        adc_regs->CR2 |= Platform::ADC::getBitValue(Platform::ADC::CR2::ALIGN);
    }
    
    // Configure interrupts
    ConfigureInterrupts(true);
    
    // Enable ADC
    adc_regs->CR2 |= Platform::ADC::getBitValue(Platform::ADC::CR2::ADON);
    
    // Wait for ADC to stabilize
    Middleware::SystemServices::SystemTiming& timing = 
        Middleware::SystemServices::SystemTiming::GetInstance();
    timing.DelayMicroseconds(10);
    
    initialized = true;
    return Platform::Status::OK;
}

// Deinitialize the ADC
Platform::Status AdcInterface::DeInit() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Get ADC registers
    Platform::ADC::Registers* adc_regs = nullptr;
    switch (config.adc_instance) {
        case AdcInstance::ADC1:
            adc_regs = Platform::ADC::getADC1Registers();
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    if (adc_regs == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Disable ADC
    adc_regs->CR2 &= ~Platform::ADC::getBitValue(Platform::ADC::CR2::ADON);
    
    // Disable interrupts
    ConfigureInterrupts(false);
    
    // Disable ADC clock
    RCC::RccInterface& rcc = RCC::RccInterface::GetInstance();
    RCC::RccPeripheral adc_clock;
    
    switch (config.adc_instance) {
        case AdcInstance::ADC1:
            adc_clock = RCC::RccPeripheral::ADC1;
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    // Disable peripheral clock
    Platform::Status status = rcc.DisablePeripheralClock(adc_clock);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    initialized = false;
    return Platform::Status::OK;
}

// Read ADC values
Platform::Status AdcInterface::Read(void* buffer, uint16_t size, uint32_t timeout) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (buffer == nullptr || size == 0) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Ensure buffer is large enough for all channels
    if (size < config.num_channels * sizeof(uint32_t)) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Start conversion
    Platform::Status status = StartConversion();
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Wait for conversion to complete
    status = WaitForConversion(timeout);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Read all channels
    uint8_t num_channels = config.num_channels;
    status = ReadChannels(static_cast<uint32_t*>(buffer), &num_channels);
    
    return status;
}

// Write is not applicable for ADC, return error
Platform::Status AdcInterface::Write(const void* data, uint16_t size, uint32_t timeout) {
    return Platform::Status::NOT_SUPPORTED;
}

// ADC control function
Platform::Status AdcInterface::Control(uint32_t command, void* param) {
    if (!initialized && command != ADC_CTRL_CALIBRATE) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    switch (command) {
        case ADC_CTRL_START_CONVERSION:
            return StartConversion();
            
        case ADC_CTRL_STOP_CONVERSION:
            return StopConversion();
            
        case ADC_CTRL_GET_STATE: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            // Get ADC registers
            Platform::ADC::Registers* adc_regs = nullptr;
            switch (config.adc_instance) {
                case AdcInstance::ADC1:
                    adc_regs = Platform::ADC::getADC1Registers();
                    break;
                default:
                    return Platform::Status::INVALID_PARAM;
            }
            
            if (adc_regs == nullptr) {
                return Platform::Status::ERROR;
            }
            
            bool* is_converting = static_cast<bool*>(param);
            *is_converting = (adc_regs->SR & Platform::ADC::getBitValue(Platform::ADC::SR::STRT)) != 0;
            
            return Platform::Status::OK;
        }
            
        case ADC_CTRL_SET_CHANNEL_CONFIG: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            return ConfigureChannel(*static_cast<AdcChannelConfig*>(param));
        }
            
        case ADC_CTRL_ENABLE_CHANNEL: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            return EnableChannel(*static_cast<AdcChannel*>(param));
        }
            
        case ADC_CTRL_DISABLE_CHANNEL: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            return DisableChannel(*static_cast<AdcChannel*>(param));
        }
            
        case ADC_CTRL_SET_WATCHDOG: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            struct WatchdogConfig {
                AdcChannel channel;
                uint16_t low_threshold;
                uint16_t high_threshold;
            };
            
            WatchdogConfig* wdg_config = static_cast<WatchdogConfig*>(param);
            ConfigureWatchdog(wdg_config->channel, wdg_config->low_threshold, wdg_config->high_threshold);
            
            return Platform::Status::OK;
        }
            
        case ADC_CTRL_CALIBRATE:
            return Calibrate();
            
        case ADC_CTRL_GET_VOLTAGE: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            struct VoltageRequest {
                AdcChannel channel;
                float* voltage;
            };
            
            VoltageRequest* req = static_cast<VoltageRequest*>(param);
            
            uint32_t adc_value;
            Platform::Status status = ReadChannel(req->channel, &adc_value);
            if (status != Platform::Status::OK) {
                return status;
            }
            
            *(req->voltage) = ConvertToVoltage(adc_value);
            return Platform::Status::OK;
        }
            
        case ADC_CTRL_GET_TEMPERATURE: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            return GetTemperature(static_cast<float*>(param));
        }
            
        case ADC_CTRL_GET_VBAT: {
            if (param == nullptr) {
                return Platform::Status::INVALID_PARAM;
            }
            
            return GetBatteryVoltage(static_cast<float*>(param));
        }
            
        default:
            return Platform::Status::NOT_SUPPORTED;
    }
}

// Register callback function
Platform::Status AdcInterface::RegisterCallback(uint32_t eventId, void (*callback)(void* param), void* param) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (eventId >= static_cast<uint32_t>(AdcEvent::Max)) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Store callback
    callbacks[eventId].callback = callback;
    callbacks[eventId].param = param;
    callbacks[eventId].enabled = (callback != nullptr);
    
    // Configure interrupts based on callback registration
    if (callback != nullptr) {
        // Get ADC registers
        Platform::ADC::Registers* adc_regs = nullptr;
        switch (config.adc_instance) {
            case AdcInstance::ADC1:
                adc_regs = Platform::ADC::getADC1Registers();
                break;
            default:
                return Platform::Status::INVALID_PARAM;
        }
        
        if (adc_regs == nullptr) {
            return Platform::Status::ERROR;
        }
        
        // Enable appropriate interrupt sources
        switch (static_cast<AdcEvent>(eventId)) {
            case AdcEvent::ConversionComplete:
                adc_regs->CR1 |= Platform::ADC::getBitValue(Platform::ADC::CR1::EOCIE);
                break;
                
            case AdcEvent::SequenceComplete:
                adc_regs->CR1 |= Platform::ADC::getBitValue(Platform::ADC::CR1::EOCIE) | 
                                 Platform::ADC::getBitValue(Platform::ADC::CR1::SCAN);
                break;
                
            case AdcEvent::Watchdog:
                adc_regs->CR1 |= Platform::ADC::getBitValue(Platform::ADC::CR1::AWDIE);
                break;
                
            case AdcEvent::Overrun:
                adc_regs->CR1 |= Platform::ADC::getBitValue(Platform::ADC::CR1::OVRIE);
                break;
                
            default:
                break;
        }
    }
    
    return Platform::Status::OK;
}

// Start ADC conversion
Platform::Status AdcInterface::StartConversion() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Get ADC registers
    Platform::ADC::Registers* adc_regs = nullptr;
    switch (config.adc_instance) {
        case AdcInstance::ADC1:
            adc_regs = Platform::ADC::getADC1Registers();
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    if (adc_regs == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Start conversion
    if (config.trigger_source == AdcTriggerSource::Software) {
        // Software trigger
        adc_regs->CR2 |= Platform::ADC::getBitValue(Platform::ADC::CR2::SWSTART);
    } else {
        // For external trigger, just make sure the ADC is enabled
        adc_regs->CR2 |= Platform::ADC::getBitValue(Platform::ADC::CR2::ADON);
    }
    
    return Platform::Status::OK;
 }
 
 // Stop ADC conversion
 Platform::Status AdcInterface::StopConversion() {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Get ADC registers
    Platform::ADC::Registers* adc_regs = nullptr;
    switch (config.adc_instance) {
        case AdcInstance::ADC1:
            adc_regs = Platform::ADC::getADC1Registers();
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    if (adc_regs == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Disable ADC to stop conversions
    adc_regs->CR2 &= ~Platform::ADC::getBitValue(Platform::ADC::CR2::ADON);
    
    return Platform::Status::OK;
 }
 
 // Wait for conversion to complete (blocking)
 Platform::Status AdcInterface::WaitForConversion(uint32_t timeout) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Get ADC registers
    Platform::ADC::Registers* adc_regs = nullptr;
    switch (config.adc_instance) {
        case AdcInstance::ADC1:
            adc_regs = Platform::ADC::getADC1Registers();
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    if (adc_regs == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Get timing service for timeout management
    Middleware::SystemServices::SystemTiming& timing = 
        Middleware::SystemServices::SystemTiming::GetInstance();
    
    uint64_t start_time = timing.GetMilliseconds();
    
    // Wait for end of conversion or timeout
    while (!(adc_regs->SR & Platform::ADC::getBitValue(Platform::ADC::SR::EOC))) {
        if (timeout > 0 && (timing.GetMilliseconds() - start_time > timeout)) {
            return Platform::Status::TIMEOUT;
        }
        
        // Short delay to prevent tight polling
        timing.DelayMicroseconds(10);
    }
    
    return Platform::Status::OK;
 }
 
 // Read single channel
 Platform::Status AdcInterface::ReadChannel(AdcChannel channel, uint32_t* value) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (value == nullptr) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Get ADC registers
    Platform::ADC::Registers* adc_regs = nullptr;
    switch (config.adc_instance) {
        case AdcInstance::ADC1:
            adc_regs = Platform::ADC::getADC1Registers();
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    if (adc_regs == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // For single channel mode, configure the channel in SQR3
    adc_regs->SQR3 = static_cast<uint32_t>(channel);
    
    // Set sequence length to 1
    adc_regs->SQR1 &= ~Platform::ADC::SQR1_Bits::L_MSK;
    
    // Start conversion
    adc_regs->CR2 |= Platform::ADC::getBitValue(Platform::ADC::CR2::SWSTART);
    
    // Wait for end of conversion
    while (!(adc_regs->SR & Platform::ADC::getBitValue(Platform::ADC::SR::EOC)));
    
    // Read converted value
    *value = adc_regs->DR;
    
    return Platform::Status::OK;
 }
 
 // Read multiple channels
 Platform::Status AdcInterface::ReadChannels(uint32_t* values, uint8_t* num_channels) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    if (values == nullptr || num_channels == nullptr || *num_channels == 0) {
        return Platform::Status::INVALID_PARAM;
    }
    
    // Limit to actual number of configured channels
    *num_channels = std::min(*num_channels, config.num_channels);
    
    // Get ADC registers
    Platform::ADC::Registers* adc_regs = nullptr;
    switch (config.adc_instance) {
        case AdcInstance::ADC1:
            adc_regs = Platform::ADC::getADC1Registers();
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    if (adc_regs == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // If using DMA, the values would be already in the buffer
    // For polling mode, we need to read each channel
    if (!config.enable_dma) {
        for (uint8_t i = 0; i < *num_channels; i++) {
            // In scan mode, we'd need to read DR after each conversion
            // This assumes that conversions have completed and EOC was set for each channel
            values[i] = adc_regs->DR;
            
            // For single-channel mode, we'd need to configure and start each channel separately
            if (config.scan_mode == AdcScanMode::SingleChannel && i < (*num_channels - 1)) {
                // Configure next channel and start conversion
                // This code assumes channels are stored in the configuration
                adc_regs->SQR3 = static_cast<uint32_t>(config.channel_config[i + 1].channel);
                adc_regs->CR2 |= Platform::ADC::getBitValue(Platform::ADC::CR2::SWSTART);
                
                // Wait for end of conversion
                while (!(adc_regs->SR & Platform::ADC::getBitValue(Platform::ADC::SR::EOC)));
            }
        }
    } else {
        // For DMA mode, the buffer would already be filled by DMA
        // This is a simplified approach - actual implementation would depend on DMA configuration
        return Platform::Status::NOT_SUPPORTED;
    }
    
    return Platform::Status::OK;
 }
 
 // Configure channel
 Platform::Status AdcInterface::ConfigureChannel(const AdcChannelConfig& channel_config) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Configure GPIO for external channels
    if (channel_config.channel <= AdcChannel::Channel15) {
        ConfigureGPIO(channel_config.channel);
    }
    
    // Configure sampling time
    ConfigureSamplingTime(channel_config.channel, channel_config.sampling_time);
    
    // Configure analog watchdog if enabled
    if (channel_config.use_watchdog) {
        ConfigureWatchdog(channel_config.channel, 
                         channel_config.watchdog_low_threshold, 
                         channel_config.watchdog_high_threshold);
    }
    
    return Platform::Status::OK;
 }
 
 // Enable channel
 Platform::Status AdcInterface::EnableChannel(AdcChannel channel) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // Channel is enabled by adding it to the sequence
    // This is a simplified version - would need to handle actual sequence management
    // Get ADC registers
    Platform::ADC::Registers* adc_regs = nullptr;
    switch (config.adc_instance) {
        case AdcInstance::ADC1:
            adc_regs = Platform::ADC::getADC1Registers();
            break;
        default:
            return Platform::Status::INVALID_PARAM;
    }
    
    if (adc_regs == nullptr) {
        return Platform::Status::ERROR;
    }
    
    // Add channel to sequence at next available position
    uint8_t current_length = ((adc_regs->SQR1 & Platform::ADC::SQR1_Bits::L_MSK) >> Platform::ADC::SQR1_Bits::L_POS) + 1;
    
    if (current_length >= ADC_CHANNELS_PER_INSTANCE) {
        return Platform::Status::ERROR;  // No room in sequence
    }
    
    // Add channel to sequence
    uint8_t sequence_position = current_length + 1; // 1-based position
    
    // Configure channel sequence
    if (sequence_position <= 6) {
        // SQ1-SQ6 are in SQR3
        uint32_t shift = (sequence_position - 1) * 5;
        uint32_t channel_value = static_cast<uint32_t>(channel) << shift;
        adc_regs->SQR3 &= ~(0x1F << shift);
        adc_regs->SQR3 |= channel_value;
    } else if (sequence_position <= 12) {
        // SQ7-SQ12 are in SQR2
        uint32_t shift = (sequence_position - 7) * 5;
        uint32_t channel_value = static_cast<uint32_t>(channel) << shift;
        adc_regs->SQR2 &= ~(0x1F << shift);
        adc_regs->SQR2 |= channel_value;
    } else {
        // SQ13-SQ16 are in SQR1
        uint32_t shift = (sequence_position - 13) * 5;
        uint32_t channel_value = static_cast<uint32_t>(channel) << shift;
        adc_regs->SQR1 &= ~(0x1F << shift);
        adc_regs->SQR1 |= channel_value;
    }

    // Update sequence length
    adc_regs->SQR1 |= (current_length << Platform::ADC::SQR1_Bits::L_POS) & Platform::ADC::SQR1_Bits::L_MSK;
    
    return Platform::Status::OK;
 }
 
 // Disable channel
 Platform::Status AdcInterface::DisableChannel(AdcChannel channel) {
    if (!initialized) {
        return Platform::Status::NOT_INITIALIZED;
    }
    
    // This function would need to remove the channel from the sequence
    // and reorder remaining channels. This is complex and depends on the
    // current sequence configuration. It's left as a placeholder.
    
    return Platform::Status::NOT_SUPPORTED;
 }
 
 // Get raw to voltage conversion factor
 float AdcInterface::GetVoltageFactor() const {
    // Calculate voltage factor based on ADC resolution
    float max_value;
    
    switch (config.resolution) {
        case AdcResolution::Bits12:
            max_value = 4095.0f;  // 2^12 - 1
            break;
        case AdcResolution::Bits10:
            max_value = 1023.0f;  // 2^10 - 1
            break;
        case AdcResolution::Bits8:
            max_value = 255.0f;   // 2^8 - 1
            break;
        case AdcResolution::Bits6:
            max_value = 63.0f;    // 2^6 - 1
            break;
        default:
            max_value = 4095.0f;  // Default to 12-bit
    }
    
    // Assume 3.3V reference voltage
    return 3.3f / max_value;
 }
 
 // Convert raw ADC value to voltage
 float AdcInterface::ConvertToVoltage(uint32_t raw_value) const {
    return raw_value * GetVoltageFactor();
 }
 
 // Get MCU internal temperature from sensor
 Platform::Status AdcInterface::GetTemperature(float* temperature) {
    if (!initialized || !temperature) {
        return Platform::Status::INVALID_PARAM;
    }
    
    if (!config.enable_temp_sensor) {
        return Platform::Status::NOT_SUPPORTED;
    }
    
    // Read temperature sensor channel
    uint32_t adc_value;
    Platform::Status status = ReadChannel(AdcChannel::TempSensor, &adc_value);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Convert to voltage
    float voltage = ConvertToVoltage(adc_value);
    
    // Convert to temperature using STM32F4 formula
    // T = ((V_SENSE - V_25) / Avg_Slope) + 25
    // Where V_25 is typically 0.76V and Avg_Slope is typically 2.5 mV/°C
    *temperature = ((voltage - 0.76f) / 0.0025f) + 25.0f;
    
    return Platform::Status::OK;
 }
 
 // Get Vbat value
 Platform::Status AdcInterface::GetBatteryVoltage(float* voltage) {
    if (!initialized || !voltage) {
        return Platform::Status::INVALID_PARAM;
    }
    
    if (!config.enable_vbat) {
        return Platform::Status::NOT_SUPPORTED;
    }
    
    // Read battery voltage channel
    uint32_t adc_value;
    Platform::Status status = ReadChannel(AdcChannel::VBat, &adc_value);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Convert to voltage and scale by battery voltage divider (1/4 on STM32F4)
    *voltage = ConvertToVoltage(adc_value) * 4.0f;
    
    return Platform::Status::OK;
 }
 
 // Get internal reference voltage
 Platform::Status AdcInterface::GetVrefIntVoltage(float* voltage) {
    if (!initialized || !voltage) {
        return Platform::Status::INVALID_PARAM;
    }
    
    if (!config.enable_vrefint) {
        return Platform::Status::NOT_SUPPORTED;
    }
    
    // Read internal reference channel
    uint32_t adc_value;
    Platform::Status status = ReadChannel(AdcChannel::VRefInt, &adc_value);
    if (status != Platform::Status::OK) {
        return status;
    }
    
    // Convert to voltage
    *voltage = ConvertToVoltage(adc_value);
    
    return Platform::Status::OK;
 }
 
 // Calibrate ADC
 Platform::Status AdcInterface::Calibrate() {
    // Note: STM32F4 ADCs don't have a built-in calibration procedure
    // This function could implement a software calibration by measuring
    // known reference voltages and calculating correction factors
    
    return Platform::Status::NOT_SUPPORTED;
 }
 
 // Configure GPIO pins for ADC channels
 void AdcInterface::ConfigureGPIO(AdcChannel channel) {
    // Get GPIO interface
    Platform::GPIO::GpioInterface& gpio = Platform::GPIO::GpioInterface::GetInstance();
    
    // ADC channel to GPIO pin mapping
    // This mapping depends on the specific STM32 device
    // Here's a typical mapping for STM32F4 series
    Platform::GPIO::Port port;
    uint8_t pin;
    
    // Map ADC channel to GPIO pin
    switch (channel) {
        case AdcChannel::Channel0:
            port = Platform::GPIO::Port::PORTA;
            pin = 0;
            break;
        case AdcChannel::Channel1:
            port = Platform::GPIO::Port::PORTA;
            pin = 1;
            break;
        case AdcChannel::Channel2:
            port = Platform::GPIO::Port::PORTA;
            pin = 2;
            break;
        case AdcChannel::Channel3:
            port = Platform::GPIO::Port::PORTA;
            pin = 3;
            break;
        case AdcChannel::Channel4:
            port = Platform::GPIO::Port::PORTA;
            pin = 4;
            break;
        case AdcChannel::Channel5:
            port = Platform::GPIO::Port::PORTA;
            pin = 5;
            break;
        case AdcChannel::Channel6:
            port = Platform::GPIO::Port::PORTA;
            pin = 6;
            break;
        case AdcChannel::Channel7:
            port = Platform::GPIO::Port::PORTA;
            pin = 7;
            break;
        case AdcChannel::Channel8:
            port = Platform::GPIO::Port::PORTB;
            pin = 0;
            break;
        case AdcChannel::Channel9:
            port = Platform::GPIO::Port::PORTB;
            pin = 1;
            break;
        case AdcChannel::Channel10:
            port = Platform::GPIO::Port::PORTC;
            pin = 0;
            break;
        case AdcChannel::Channel11:
            port = Platform::GPIO::Port::PORTC;
            pin = 1;
            break;
        case AdcChannel::Channel12:
            port = Platform::GPIO::Port::PORTC;
            pin = 2;
            break;
        case AdcChannel::Channel13:
            port = Platform::GPIO::Port::PORTC;
            pin = 3;
            break;
        case AdcChannel::Channel14:
            port = Platform::GPIO::Port::PORTC;
            pin = 4;
            break;
        case AdcChannel::Channel15:
            port = Platform::GPIO::Port::PORTC;
            pin = 5;
            break;
        default:
            // Internal channels don't need GPIO configuration
            return;
    }
    
    // Configure the GPIO pin for analog mode
    Platform::GPIO::GpioConfig pin_config = {
        .port = port,
        .pin = pin,
        .mode = Platform::GPIO::Mode::Analog,
        .pull = Platform::GPIO::Pull::None
    };
    
    gpio.ConfigurePin(pin_config);
 }
 
 // Configure interrupts
 void AdcInterface::ConfigureInterrupts(bool enable) {
    // Get NVIC interface
    Platform::CMSIS::NVIC::IRQn irq;
    
    // Determine IRQ based on ADC instance
    switch (config.adc_instance) {
        case AdcInstance::ADC1:
            irq = Platform::CMSIS::NVIC::IRQn::ADC;
            break;
        default:
            return;
    }
    
    // Enable or disable interrupt in NVIC
    if (enable) {
        Platform::CMSIS::NVIC::setPriority(irq, Platform::CMSIS::NVIC::PRIORITY_MEDIUM);
        Platform::CMSIS::NVIC::enableIRQ(irq);
    } else {
        Platform::CMSIS::NVIC::disableIRQ(irq);
    }
 }
 
 // Configure ADC resolution
 void AdcInterface::ConfigureResolution(AdcResolution resolution) {
    // Get ADC registers
    Platform::ADC::Registers* adc_regs = nullptr;
    switch (config.adc_instance) {
        case AdcInstance::ADC1:
            adc_regs = Platform::ADC::getADC1Registers();
            break;
        default:
            return;
    }
    
    if (adc_regs == nullptr) {
        return;
    }
    
    // Clear existing resolution bits
    adc_regs->CR1 &= ~Platform::ADC::getBitValue(Platform::ADC::CR1::RES_MSK);
    
    // Set new resolution
    uint32_t res_bits = static_cast<uint32_t>(resolution) << 24;
    adc_regs->CR1 |= res_bits & Platform::ADC::getBitValue(Platform::ADC::CR1::RES_MSK);
 }
 
 // Configure channel sampling time
 void AdcInterface::ConfigureSamplingTime(AdcChannel channel, AdcSamplingTime sampling_time) {
    // Get ADC registers
    Platform::ADC::Registers* adc_regs = nullptr;
    switch (config.adc_instance) {
        case AdcInstance::ADC1:
            adc_regs = Platform::ADC::getADC1Registers();
            break;
        default:
            return;
    }
    
    if (adc_regs == nullptr) {
        return;
    }
    
    // Determine register and bit position based on channel
    volatile uint32_t* smpr_reg;
    uint8_t pos;
    
    if (static_cast<uint8_t>(channel) < 10) {
        // Channels 0-9 use SMPR2
        smpr_reg = &adc_regs->SMPR2;
        pos = static_cast<uint8_t>(channel) * 3;
    } else if (static_cast<uint8_t>(channel) <= 18) {
        // Channels 10-18 use SMPR1
        smpr_reg = &adc_regs->SMPR1;
        pos = (static_cast<uint8_t>(channel) - 10) * 3;
    } else {
        // Invalid channel
        return;
    }
    
    // Clear existing bits
    *smpr_reg &= ~(0x7UL << pos);
    
    // Set new sampling time
    *smpr_reg |= (static_cast<uint32_t>(sampling_time) << pos);
 }
 
 // Configure analog watchdog
 void AdcInterface::ConfigureWatchdog(AdcChannel channel, uint16_t low_threshold, uint16_t high_threshold) {
    // Get ADC registers
    Platform::ADC::Registers* adc_regs = nullptr;
    switch (config.adc_instance) {
        case AdcInstance::ADC1:
            adc_regs = Platform::ADC::getADC1Registers();
            break;
        default:
            return;
    }
    
    if (adc_regs == nullptr) {
        return;
    }
    
    // Set watchdog thresholds
    adc_regs->HTR = high_threshold & 0xFFF;
    adc_regs->LTR = low_threshold & 0xFFF;
    
    // Configure watchdog channel and enable
    adc_regs->CR1 &= ~(Platform::ADC::getBitValue(Platform::ADC::CR1::AWDSGL) | 
                       Platform::ADC::getBitValue(Platform::ADC::CR1::AWDCH_MSK));
    
    // Enable single channel mode and set the channel
    adc_regs->CR1 |= Platform::ADC::getBitValue(Platform::ADC::CR1::AWDSGL) | 
                     (static_cast<uint32_t>(channel) & 0x1F);
    
    // Enable analog watchdog
    adc_regs->CR1 |= Platform::ADC::getBitValue(Platform::ADC::CR1::AWDEN);
 }
 
 // Interrupt handler for ADC
 void AdcInterface::HandleInterrupt() {
    // Get ADC registers
    Platform::ADC::Registers* adc_regs = nullptr;
    switch (config.adc_instance) {
        case AdcInstance::ADC1:
            adc_regs = Platform::ADC::getADC1Registers();
            break;
        default:
            return;
    }
    
    if (adc_regs == nullptr) {
        return;
    }
    
    // Check for end of conversion
    if (adc_regs->SR & Platform::ADC::getBitValue(Platform::ADC::SR::EOC)) {
        // Clear flag by reading data register
        volatile uint32_t tmp = adc_regs->DR;
        
        // Call conversion complete callback
        if (callbacks[static_cast<uint32_t>(AdcEvent::ConversionComplete)].enabled) {
            callbacks[static_cast<uint32_t>(AdcEvent::ConversionComplete)].callback(
                callbacks[static_cast<uint32_t>(AdcEvent::ConversionComplete)].param
            );
        }
    }
    
    // Check for end of sequence
    if (adc_regs->SR & Platform::ADC::getBitValue(Platform::ADC::SR::STRT)) {
        // Call sequence complete callback
        if (callbacks[static_cast<uint32_t>(AdcEvent::SequenceComplete)].enabled) {
            callbacks[static_cast<uint32_t>(AdcEvent::SequenceComplete)].callback(
                callbacks[static_cast<uint32_t>(AdcEvent::SequenceComplete)].param
            );
        }
    }
    
    // Check for analog watchdog event
    if (adc_regs->SR & Platform::ADC::getBitValue(Platform::ADC::SR::AWD)) {
        // Clear flag
        adc_regs->SR &= ~Platform::ADC::getBitValue(Platform::ADC::SR::AWD);
        
        // Call watchdog callback
        if (callbacks[static_cast<uint32_t>(AdcEvent::Watchdog)].enabled) {
            callbacks[static_cast<uint32_t>(AdcEvent::Watchdog)].callback(
                callbacks[static_cast<uint32_t>(AdcEvent::Watchdog)].param
            );
        }
    }
    
    // Check for overrun
    if (adc_regs->SR & Platform::ADC::getBitValue(Platform::ADC::SR::OVR)) {
        // Clear flag
        adc_regs->SR &= ~Platform::ADC::getBitValue(Platform::ADC::SR::OVR);
        
        // Call overrun callback
        if (callbacks[static_cast<uint32_t>(AdcEvent::Overrun)].enabled) {
            callbacks[static_cast<uint32_t>(AdcEvent::Overrun)].callback(
                callbacks[static_cast<uint32_t>(AdcEvent::Overrun)].param
            );
        }
    }
 }
 
 } // namespace ADC
 } // namespace Platform
 
 // ADC interrupt handler
 extern "C" void ADC_IRQHandler(void) {
    // Determine which ADC triggered the interrupt
    // All three ADCs (ADC1, ADC2, ADC3) share the same interrupt vector
    
    // Get instance of each ADC interface
    Platform::ADC::AdcInterface& adc1 = Platform::ADC::AdcInterface::GetInstance(Platform::ADC::AdcInstance::ADC1);
    Platform::ADC::AdcInterface& adc2 = Platform::ADC::AdcInterface::GetInstance(Platform::ADC::AdcInstance::ADC2);
    Platform::ADC::AdcInterface& adc3 = Platform::ADC::AdcInterface::GetInstance(Platform::ADC::AdcInstance::ADC3);
    
    // Check status register of each ADC to determine which one triggered the interrupt
    volatile uint32_t sr1 = Platform::ADC::getADC1Registers()->SR;
    
    // Call appropriate handler
    if (sr1 & (Platform::ADC::getBitValue(Platform::ADC::SR::EOC) | 
               Platform::ADC::getBitValue(Platform::ADC::SR::AWD) | 
               Platform::ADC::getBitValue(Platform::ADC::SR::OVR))) {
        adc1.HandleInterrupt();
    }
    /*
    if (sr2 & (Platform::ADC::getBitValue(Platform::ADC::SR::EOC) | 
               Platform::ADC::getBitValue(Platform::ADC::SR::AWD) | 
               Platform::ADC::getBitValue(Platform::ADC::SR::OVR))) {
        adc2.HandleInterrupt();
    }
    
    if (sr3 & (Platform::ADC::getBitValue(Platform::ADC::SR::EOC) | 
    Platform::ADC::getBitValue(Platform::ADC::SR::AWD) | 
    Platform::ADC::getBitValue(Platform::ADC::SR::OVR))) 
    {
        adc3.HandleInterrupt();
    }*/
}