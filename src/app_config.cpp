#include "aerial_touch/app_config.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

namespace aerial_touch {
namespace {

template<typename T>
T required_value(const YAML::Node& root, const char* section, const char* key) {
    const auto node = root[section][key];
    if(!node) {
        throw std::runtime_error(std::string("Missing configuration value: ") + section + "." + key);
    }
    return node.as<T>();
}

}  // namespace

AppConfig load_app_config(const std::filesystem::path& path) {
    const YAML::Node root = YAML::LoadFile(path.string());

    AppConfig config;
    config.depth.sample_radius = required_value<int>(root, "depth", "sample_radius");
    config.touch.touch_threshold_mm = required_value<float>(root, "touch", "touch_threshold_mm");
    config.touch.release_threshold_mm = required_value<float>(root, "touch", "release_threshold_mm");
    config.touch.min_approach_velocity_mm_s =
        required_value<float>(root, "touch", "min_approach_velocity_mm_s");
    config.touch.tracking_timeout_ms = required_value<std::int64_t>(root, "touch", "tracking_timeout_ms");
    config.keypad.key_width_mm = required_value<float>(root, "keypad", "key_width_mm");
    config.keypad.key_height_mm = required_value<float>(root, "keypad", "key_height_mm");
    config.keypad.horizontal_gap_mm = required_value<float>(root, "keypad", "horizontal_gap_mm");
    config.keypad.vertical_gap_mm = required_value<float>(root, "keypad", "vertical_gap_mm");
    config.calibration.minimum_point_distance_mm =
        required_value<float>(root, "calibration", "minimum_point_distance_mm");

    if(config.depth.sample_radius < 0 || config.touch.touch_threshold_mm < 0.0F
       || config.touch.release_threshold_mm <= config.touch.touch_threshold_mm
       || config.touch.min_approach_velocity_mm_s < 0.0F || config.touch.tracking_timeout_ms <= 0
       || config.keypad.key_width_mm <= 0.0F || config.keypad.key_height_mm <= 0.0F
       || config.keypad.horizontal_gap_mm < 0.0F || config.keypad.vertical_gap_mm < 0.0F
       || config.calibration.minimum_point_distance_mm <= 0.0F) {
        throw std::runtime_error("Configuration contains invalid geometry or threshold values");
    }
    return config;
}

}  // namespace aerial_touch
