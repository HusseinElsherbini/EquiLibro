// sbr_app_events.hpp
#pragma once
#include <cstdint>

namespace APP {

// Event definitions for callbacks
constexpr uint32_t EVENT_FALL_DETECTED          = 0x0001;
constexpr uint32_t EVENT_BALANCE_UPDATE         = 0x0002;
constexpr uint32_t EVENT_REMOTE_COMMAND_RECEIVED = 0x0003;
constexpr uint32_t EVENT_ERROR_DETECTED         = 0x0004;
constexpr uint32_t EVENT_BATTERY_LOW            = 0x0005;
constexpr uint32_t EVENT_STATE_CHANGED          = 0x0006;
constexpr uint32_t EVENT_CALIB_COMPLETE         = 0x0007;
} // namespace APP