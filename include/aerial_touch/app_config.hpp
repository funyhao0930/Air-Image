#pragma once

#include "aerial_touch/keypad.hpp"
#include "aerial_touch/touch_state_machine.hpp"

#include <filesystem>

namespace aerial_touch {

struct DepthSamplingConfig {
    int sample_radius{ 2 };
};

struct CalibrationConfig {
    float minimum_point_distance_mm{ 80.0F };
};

struct AppConfig {
    DepthSamplingConfig depth;
    TouchConfig touch;
    KeypadConfig keypad;
    CalibrationConfig calibration;
};

AppConfig load_app_config(const std::filesystem::path& path);

}  // namespace aerial_touch
