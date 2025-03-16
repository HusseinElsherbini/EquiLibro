// adc.hpp - Analog-to-Digital Converter Interface
#pragma once

#include "hardware_abstraction/hw_interface.hpp"
#include "common/platform.hpp"
#include <array>
#include <functional>
#include "os/mutex.hpp"

namespace Platform {
namespace ADC {

// Maximum number of ADC instances on the STM32F4 family
constexpr uint8_t ADC_INSTANCE_COUNT = 3;

// Maximum number of channels per ADC
constexpr uint8_t ADC_CHANNELS_PER_INSTANCE = 16;

// ADC instance enumeration
enum class AdcInstance : uint8_t {
    ADC1 = 1,
    ADC2 = 2,
    ADC3 = 3
};

// ADC channel enumeration
enum class AdcChannel : uint8_t {
    Channel0 = 0,
    Channel1 = 1,
    Channel2 = 2,
    Channel3 = 3,
    Channel4 = 4,
    Channel5 = 5,
    Channel6 = 6,
    Channel7 = 7,
    Channel8 = 8,
    Channel9 = 9,
    Channel10 = 10,
    Channel11 = 11,
    Channel12 = 12,
    Channel13 = 13,
    Channel14 = 14,
    Channel15 = 15,
    // Special internal channels
    TempSensor = 16,
    VRefInt = 17,
    VBat = 18
};

// ADC resolution options
enum class AdcResolution : uint8_t {
    Bits12 = 0,    // 12-bit resolution (0-4095)
    Bits10 = 1,    // 10-bit resolution (0-1023)
    Bits8 = 2,     // 8-bit resolution (0-255)
    Bits6 = 3      // 6-bit resolution (0-63)
};

// ADC scan mode
enum class AdcScanMode : uint8_t {
    SingleChannel = 0,   // Only one channel is converted
    MultiChannel = 1     // Multiple channels are scanned in sequence
};

// ADC conversion mode
enum class AdcConversionMode : uint8_t {
    Single = 0,    // Single conversion mode (convert once when triggered)
    Continuous = 1 // Continuous conversion mode (keep converting)
};

// ADC trigger source
enum class AdcTriggerSource : uint8_t {
    Software = 0,  // Software trigger
    Timer1_CC1 = 1,
    Timer1_CC2 = 2,
    Timer1_CC3 = 3,
    Timer2_CC2 = 4,
    Timer2_CC3 = 5,
    Timer2_CC4 = 6,
    Timer2_TRGO = 7,
    Timer3_CC1 = 8,
    Timer3_TRGO = 9,
    Timer4_CC4 = 10,
    Timer5_CC1 = 11,
    Timer5_CC2 = 12,
    Timer5_CC3 = 13,
    ExternalPin = 15
};

// ADC trigger edge selection
enum class AdcTriggerEdge : uint8_t {
    None = 0,      // No external trigger
    Rising = 1,    // Rising edge trigger
    Falling = 2,   // Falling edge trigger
    Both = 3       // Both edges trigger
};

// ADC sampling time
enum class AdcSamplingTime : uint8_t {
    Cycles3 = 0,   // 3 ADC clock cycles
    Cycles15 = 1,  // 15 ADC clock cycles
    Cycles28 = 2,  // 28 ADC clock cycles
    Cycles56 = 3,  // 56 ADC clock cycles
    Cycles84 = 4,  // 84 ADC clock cycles
    Cycles112 = 5, // 112 ADC clock cycles
    Cycles144 = 6, // 144 ADC clock cycles
    Cycles480 = 7  // 480 ADC clock cycles
};

// ADC events for callbacks
enum class AdcEvent : uint8_t {
    ConversionComplete = 0, // Single conversion complete
    SequenceComplete = 1,   // Sequence of conversions complete
    Watchdog = 2,           // Analog watchdog event
    Overrun = 3,            // Overrun event
    Max = 4                 // Maximum number of events (for array sizing)
};

// ADC channel configuration
struct AdcChannelConfig {
    AdcChannel channel;             // Channel to configure
    AdcSamplingTime sampling_time;  // Sampling time for this channel
    bool use_watchdog;              // Whether to use analog watchdog on this channel
    uint16_t watchdog_high_threshold; // Watchdog high threshold (if used)
    uint16_t watchdog_low_threshold;  // Watchdog low threshold (if used)
};

// ADC configuration structure
struct AdcConfig {
    AdcInstance adc_instance;       // ADC instance to use
    AdcResolution resolution;       // ADC resolution
    AdcScanMode scan_mode;          // Single or multi-channel mode
    AdcConversionMode conv_mode;    // Single or continuous conversion
    AdcTriggerSource trigger_source; // Trigger source
    AdcTriggerEdge trigger_edge;    // Trigger edge
    bool enable_dma;                // Whether to use DMA for transfers
    bool enable_overrun_detection;  // Whether to enable overrun detection
    bool enable_temp_sensor;        // Whether to enable temperature sensor
    bool enable_vrefint;            // Whether to enable internal reference voltage
    bool enable_vbat;               // Whether to enable battery voltage measurement
    uint8_t num_channels;           // Number of channels to scan
    std::array<AdcChannelConfig, ADC_CHANNELS_PER_INSTANCE> channel_config; // Channel configs
};

// ADC control commands
constexpr uint32_t ADC_CTRL_START_CONVERSION = 0x0301;
constexpr uint32_t ADC_CTRL_STOP_CONVERSION = 0x0302;
constexpr uint32_t ADC_CTRL_GET_STATE = 0x0303;
constexpr uint32_t ADC_CTRL_SET_CHANNEL_CONFIG = 0x0304;
constexpr uint32_t ADC_CTRL_ENABLE_CHANNEL = 0x0305;
constexpr uint32_t ADC_CTRL_DISABLE_CHANNEL = 0x0306;
constexpr uint32_t ADC_CTRL_SET_WATCHDOG = 0x0307;
constexpr uint32_t ADC_CTRL_CALIBRATE = 0x0308;
constexpr uint32_t ADC_CTRL_GET_VOLTAGE = 0x0309;
constexpr uint32_t ADC_CTRL_GET_TEMPERATURE = 0x030A;
constexpr uint32_t ADC_CTRL_GET_VBAT = 0x030B;

// Analog-to-Digital Converter Interface
class AdcInterface : public HwInterface {
private:
    // Private state tracking
    bool initialized;
    AdcInstance adc_instance;
    AdcConfig config;
    
    // Callback storage
    struct CallbackEntry {
        void (*callback)(void* param);
        void* param;
        bool enabled;
    };
    
    std::array<CallbackEntry, static_cast<size_t>(AdcEvent::Max)> callbacks;
    
    // Private methods to handle hardware specifics
    void ConfigureGPIO(AdcChannel channel);
    void ConfigureInterrupts(bool enable);
    void ConfigureResolution(AdcResolution resolution);
    void ConfigureSamplingTime(AdcChannel channel, AdcSamplingTime sampling_time);
    void ConfigureWatchdog(AdcChannel channel, uint16_t low_threshold, uint16_t high_threshold);
    
    // Singleton implementation
    static void* operator new(size_t size) = delete;
    
    // Private constructor for singleton pattern
    AdcInterface();
    
    // Static instance storage
    static AdcInterface* instances[ADC_INSTANCE_COUNT];
    
    // Mutex for thread safety
    static OS::mutex instances_mutex;
    
public:
    // Static method to get ADC instance
    static AdcInterface& GetInstance(AdcInstance instance = AdcInstance::ADC1);
    
    // Destructor
    ~AdcInterface() override;
    
    // Implement the core HwInterface methods
    Platform::Status Init(void* config) override;
    Platform::Status DeInit() override;
    Platform::Status Read(void* buffer, uint16_t size, uint32_t timeout) override;
    Platform::Status Write(const void* data, uint16_t size, uint32_t timeout) override;
    Platform::Status Control(uint32_t command, void* param) override;
    Platform::Status RegisterCallback(uint32_t eventId, void (*callback)(void* param), void* param) override;
    
    // ADC-specific methods
    
    // Start ADC conversion
    Platform::Status StartConversion();
    
    // Stop ADC conversion
    Platform::Status StopConversion();
    
    // Wait for conversion to complete (blocking)
    Platform::Status WaitForConversion(uint32_t timeout);
    
    // Read single channel
    Platform::Status ReadChannel(AdcChannel channel, uint32_t* value);
    
    // Read multiple channels
    Platform::Status ReadChannels(uint32_t* values, uint8_t* num_channels);
    
    // Configure channel
    Platform::Status ConfigureChannel(const AdcChannelConfig& channel_config);
    
    // Enable/Disable channel
    Platform::Status EnableChannel(AdcChannel channel);
    Platform::Status DisableChannel(AdcChannel channel);
    
    // Get raw to voltage conversion factor
    float GetVoltageFactor() const;
    
    // Convert raw ADC value to voltage
    float ConvertToVoltage(uint32_t raw_value) const;
    
    // Get MCU internal temperature from sensor
    Platform::Status GetTemperature(float* temperature);
    
    // Get Vbat value
    Platform::Status GetBatteryVoltage(float* voltage);
    
    // Get internal reference voltage
    Platform::Status GetVrefIntVoltage(float* voltage);
    
    // Calibrate ADC
    Platform::Status Calibrate();
    
    // Interrupt handler for this ADC instance
    void HandleInterrupt();
};

} // namespace ADC
} // namespace Platform