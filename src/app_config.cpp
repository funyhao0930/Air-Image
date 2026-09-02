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

template<typename T>
T optional_value(const YAML::Node& root, const char* section, const char* key, const T& fallback) {
    const auto section_node = root[section];
    if(!section_node) {
        return fallback;
    }
    const auto node = section_node[key];
    return node ? node.as<T>() : fallback;
}

}  // namespace

void validate_app_config(const AppConfig& config) {
    if(config.camera.preferred_fps <= 0
       || (config.camera.rgb_power_line_frequency_hz != 50 && config.camera.rgb_power_line_frequency_hz != 60)
       || config.depth.sample_radius < 0 || config.depth.median_window_size == 0U
       || config.depth.invalid_reset_frames == 0U || !std::isfinite(config.depth.max_jump_mm)
       || config.depth.max_jump_mm <= 0.0F
       || !std::isfinite(config.fingertip.min_cutoff_hz) || config.fingertip.min_cutoff_hz <= 0.0F
       || !std::isfinite(config.fingertip.beta) || config.fingertip.beta < 0.0F
       || !std::isfinite(config.fingertip.derivative_cutoff_hz)
       || config.fingertip.derivative_cutoff_hz <= 0.0F || config.fingertip.display_hold_ms < 0
       || !std::isfinite(config.touch.touch_threshold_mm)
       || !std::isfinite(config.touch.release_threshold_mm)
       || !std::isfinite(config.touch.min_approach_velocity_mm_s)
       || !std::isfinite(config.keypad.boundary_hysteresis_mm)
       || !std::isfinite(config.calibration.minimum_point_distance_mm)
       || config.calibration.required_samples < 15U || config.calibration.required_samples > 20U
       || !std::isfinite(config.calibration.mad_multiplier)
       || !std::isfinite(config.calibration.minimum_outlier_threshold_mm)
       || config.touch.touch_threshold_mm < 0.0F
       || config.touch.release_threshold_mm <= config.touch.touch_threshold_mm
       || config.touch.min_approach_velocity_mm_s < 0.0F || config.touch.tracking_timeout_ms <= 0
       || config.keypad.boundary_hysteresis_mm < 0.0F
       || config.calibration.minimum_point_distance_mm <= 0.0F
       || config.calibration.mad_multiplier <= 0.0F
       || config.calibration.minimum_outlier_threshold_mm <= 0.0F) {
        throw std::runtime_error(u8"設定包含無效的幾何尺寸或臨界值");
    }
}

AppConfig load_app_config(const std::filesystem::path& path) {
    const YAML::Node root = YAML::LoadFile(path.string());

    AppConfig config;
    config.camera.depth_work_mode =
        optional_value<std::string>(root, "camera", "depth_work_mode", config.camera.depth_work_mode);
    config.camera.depth_precision =
        optional_value<std::string>(root, "camera", "depth_precision", config.camera.depth_precision);
    config.camera.preferred_fps = optional_value<int>(root, "camera", "preferred_fps", config.camera.preferred_fps);
    config.camera.sdk_temporal_filter =
        optional_value<bool>(root, "camera", "sdk_temporal_filter", config.camera.sdk_temporal_filter);
    config.camera.sdk_spatial_filter =
        optional_value<bool>(root, "camera", "sdk_spatial_filter", config.camera.sdk_spatial_filter);
    config.camera.hole_filling_filter =
        optional_value<bool>(root, "camera", "hole_filling_filter", config.camera.hole_filling_filter);
    config.camera.rgb_power_line_frequency_hz =
        optional_value<int>(root, "camera", "rgb_power_line_frequency_hz",
                            config.camera.rgb_power_line_frequency_hz);
    config.depth.sample_radius = required_value<int>(root, "depth", "sample_radius");
    config.depth.median_window_size =
        optional_value<std::size_t>(root, "depth", "median_window_size", config.depth.median_window_size);
    config.depth.max_jump_mm = optional_value<float>(root, "depth", "max_jump_mm", config.depth.max_jump_mm);
    config.depth.invalid_reset_frames =
        optional_value<std::size_t>(root, "depth", "invalid_reset_frames", config.depth.invalid_reset_frames);
    config.fingertip.min_cutoff_hz =
        optional_value<float>(root, "fingertip", "min_cutoff_hz", config.fingertip.min_cutoff_hz);
    config.fingertip.beta = optional_value<float>(root, "fingertip", "beta", config.fingertip.beta);
    config.fingertip.derivative_cutoff_hz =
        optional_value<float>(root, "fingertip", "derivative_cutoff_hz",
                              config.fingertip.derivative_cutoff_hz);
    config.fingertip.display_hold_ms =
        optional_value<std::int64_t>(root, "fingertip", "display_hold_ms", config.fingertip.display_hold_ms);
    config.touch.touch_threshold_mm = required_value<float>(root, "touch", "touch_threshold_mm");
    config.touch.release_threshold_mm = required_value<float>(root, "touch", "release_threshold_mm");
    config.touch.min_approach_velocity_mm_s =
        required_value<float>(root, "touch", "min_approach_velocity_mm_s");
    config.touch.tracking_timeout_ms = required_value<std::int64_t>(root, "touch", "tracking_timeout_ms");
    config.keypad.boundary_hysteresis_mm =
        optional_value<float>(root, "keypad", "boundary_hysteresis_mm", config.keypad.boundary_hysteresis_mm);
    config.calibration.minimum_point_distance_mm =
        required_value<float>(root, "calibration", "minimum_point_distance_mm");
    config.calibration.required_samples =
        optional_value<std::size_t>(root, "calibration", "required_samples", config.calibration.required_samples);
    config.calibration.mad_multiplier =
        optional_value<float>(root, "calibration", "mad_multiplier", config.calibration.mad_multiplier);
    config.calibration.minimum_outlier_threshold_mm =
        optional_value<float>(root, "calibration", "minimum_outlier_threshold_mm",
                              config.calibration.minimum_outlier_threshold_mm);

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
    emitter << YAML::Key << "camera" << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "depth_work_mode" << YAML::Value << config.camera.depth_work_mode;
    emitter << YAML::Key << "depth_precision" << YAML::Value << config.camera.depth_precision;
    emitter << YAML::Key << "preferred_fps" << YAML::Value << config.camera.preferred_fps;
    emitter << YAML::Key << "sdk_temporal_filter" << YAML::Value << config.camera.sdk_temporal_filter;
    emitter << YAML::Key << "sdk_spatial_filter" << YAML::Value << config.camera.sdk_spatial_filter;
    emitter << YAML::Key << "hole_filling_filter" << YAML::Value << config.camera.hole_filling_filter;
    emitter << YAML::Key << "rgb_power_line_frequency_hz" << YAML::Value
            << config.camera.rgb_power_line_frequency_hz;
    emitter << YAML::EndMap;
    emitter << YAML::Key << "depth" << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "sample_radius" << YAML::Value << config.depth.sample_radius;
    emitter << YAML::Key << "median_window_size" << YAML::Value << config.depth.median_window_size;
    emitter << YAML::Key << "max_jump_mm" << YAML::Value << config.depth.max_jump_mm;
    emitter << YAML::Key << "invalid_reset_frames" << YAML::Value << config.depth.invalid_reset_frames;
    emitter << YAML::EndMap;
    emitter << YAML::Key << "fingertip" << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "min_cutoff_hz" << YAML::Value << config.fingertip.min_cutoff_hz;
    emitter << YAML::Key << "beta" << YAML::Value << config.fingertip.beta;
    emitter << YAML::Key << "derivative_cutoff_hz" << YAML::Value << config.fingertip.derivative_cutoff_hz;
    emitter << YAML::Key << "display_hold_ms" << YAML::Value << config.fingertip.display_hold_ms;
    emitter << YAML::EndMap;
    emitter << YAML::Key << "touch" << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "touch_threshold_mm" << YAML::Value << config.touch.touch_threshold_mm;
    emitter << YAML::Key << "release_threshold_mm" << YAML::Value << config.touch.release_threshold_mm;
    emitter << YAML::Key << "min_approach_velocity_mm_s" << YAML::Value << config.touch.min_approach_velocity_mm_s;
    emitter << YAML::Key << "tracking_timeout_ms" << YAML::Value << config.touch.tracking_timeout_ms;
    emitter << YAML::EndMap;
    emitter << YAML::Key << "keypad" << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "boundary_hysteresis_mm" << YAML::Value << config.keypad.boundary_hysteresis_mm;
    emitter << YAML::EndMap;
    emitter << YAML::Key << "calibration" << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "minimum_point_distance_mm" << YAML::Value
            << config.calibration.minimum_point_distance_mm;
    emitter << YAML::Key << "required_samples" << YAML::Value << config.calibration.required_samples;
    emitter << YAML::Key << "mad_multiplier" << YAML::Value << config.calibration.mad_multiplier;
    emitter << YAML::Key << "minimum_outlier_threshold_mm" << YAML::Value
            << config.calibration.minimum_outlier_threshold_mm;
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
