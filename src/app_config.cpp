#include "aerial_touch/app_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>

#ifdef _WIN32
#include <windows.h>
#endif

namespace aerial_touch {
namespace {

template<typename T>
T required_value(const YAML::Node& root, const char* section, const char* key) {
    const auto node = root[section][key];
    if(!node) {
        throw std::runtime_error(std::string(u8"缺少設定值：") + section + "." + key);
    }
    return node.as<T>();
}

}  // namespace

void validate_app_config(const AppConfig& config) {
    if(config.depth.sample_radius < 0 || !std::isfinite(config.touch.touch_threshold_mm)
       || !std::isfinite(config.touch.release_threshold_mm)
       || !std::isfinite(config.touch.min_approach_velocity_mm_s)
       || !std::isfinite(config.keypad.key_width_mm) || !std::isfinite(config.keypad.key_height_mm)
       || !std::isfinite(config.keypad.horizontal_gap_mm) || !std::isfinite(config.keypad.vertical_gap_mm)
       || !std::isfinite(config.calibration.minimum_point_distance_mm)
       || config.touch.touch_threshold_mm < 0.0F
       || config.touch.release_threshold_mm <= config.touch.touch_threshold_mm
       || config.touch.min_approach_velocity_mm_s < 0.0F || config.touch.tracking_timeout_ms <= 0
       || config.keypad.key_width_mm <= 0.0F || config.keypad.key_height_mm <= 0.0F
       || config.keypad.horizontal_gap_mm < 0.0F || config.keypad.vertical_gap_mm < 0.0F
       || config.calibration.minimum_point_distance_mm <= 0.0F) {
        throw std::runtime_error(u8"設定包含無效的幾何尺寸或臨界值");
    }
}

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

    validate_app_config(config);
    return config;
}

void save_app_config(const AppConfig& config, const std::filesystem::path& path) {
    validate_app_config(config);
    if(path.empty()) {
        throw std::runtime_error(u8"設定檔路徑不可為空");
    }

    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "depth" << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "sample_radius" << YAML::Value << config.depth.sample_radius;
    emitter << YAML::EndMap;
    emitter << YAML::Key << "touch" << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "touch_threshold_mm" << YAML::Value << config.touch.touch_threshold_mm;
    emitter << YAML::Key << "release_threshold_mm" << YAML::Value << config.touch.release_threshold_mm;
    emitter << YAML::Key << "min_approach_velocity_mm_s" << YAML::Value << config.touch.min_approach_velocity_mm_s;
    emitter << YAML::Key << "tracking_timeout_ms" << YAML::Value << config.touch.tracking_timeout_ms;
    emitter << YAML::EndMap;
    emitter << YAML::Key << "keypad" << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "key_width_mm" << YAML::Value << config.keypad.key_width_mm;
    emitter << YAML::Key << "key_height_mm" << YAML::Value << config.keypad.key_height_mm;
    emitter << YAML::Key << "horizontal_gap_mm" << YAML::Value << config.keypad.horizontal_gap_mm;
    emitter << YAML::Key << "vertical_gap_mm" << YAML::Value << config.keypad.vertical_gap_mm;
    emitter << YAML::EndMap;
    emitter << YAML::Key << "calibration" << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "minimum_point_distance_mm" << YAML::Value
            << config.calibration.minimum_point_distance_mm;
    emitter << YAML::EndMap;
    emitter << YAML::EndMap;

    if(!emitter.good()) {
        throw std::runtime_error(u8"無法產生設定檔內容");
    }

    const std::filesystem::path temporary_path = path.wstring() + L".tmp";
    std::ofstream output(temporary_path, std::ios::binary | std::ios::trunc);
    if(!output) {
        throw std::runtime_error(u8"無法寫入設定檔暫存檔");
    }
    output << emitter.c_str();
    output.flush();
    if(!output) {
        output.close();
        std::error_code ignored;
        std::filesystem::remove(temporary_path, ignored);
        throw std::runtime_error(u8"寫入設定檔失敗");
    }
    output.close();

#ifdef _WIN32
    if(!MoveFileExW(temporary_path.wstring().c_str(), path.wstring().c_str(),
                    MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
        std::error_code ignored;
        std::filesystem::remove(temporary_path, ignored);
        throw std::runtime_error(u8"無法替換原設定檔");
    }
#else
    std::error_code error;
    std::filesystem::rename(temporary_path, path, error);
    if(error) {
        std::filesystem::remove(temporary_path);
        throw std::runtime_error(u8"無法替換原設定檔");
    }
#endif
}

}  // namespace aerial_touch
