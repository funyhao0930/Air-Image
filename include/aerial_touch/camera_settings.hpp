#pragma once

#include <algorithm>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace aerial_touch {

struct CameraConfig {
    std::string depth_work_mode;
    std::string depth_precision{ "1mm" };
    int preferred_fps{ 30 };
    bool sdk_temporal_filter{ true };
    bool sdk_spatial_filter{ true };
    bool hole_filling_filter{ false };
    int rgb_power_line_frequency_hz{ 60 };
};

struct CameraCapabilities {
    std::string current_depth_work_mode;
    std::string current_depth_precision;
    int current_fps{};
    std::vector<std::string> depth_work_modes;
    std::vector<std::string> depth_precisions;
    std::vector<int> fps_values;
    std::vector<int> rgb_power_line_frequencies_hz;
    bool temporal_filter_available{ false };
    bool spatial_filter_available{ false };
    bool hole_filling_filter_available{ false };
};

enum class DepthFilterKind {
    Temporal,
    Spatial,
    HoleFilling,
};

struct CameraProfileOption {
    std::size_t index{};
    int color_fps{};
    int depth_fps{};
    bool color_is_rgb{ false };
};

inline std::vector<int> fps_fallback_order(const int preferred_fps) {
    std::vector<int> values{ preferred_fps };
    if(preferred_fps != 30) {
        values.push_back(30);
    }
    values.push_back(0);
    return values;
}

inline std::optional<std::size_t> select_camera_profile_option(
    const int preferred_fps, const std::vector<CameraProfileOption>& options) {
    for(const int target_fps : fps_fallback_order(preferred_fps)) {
        const auto selected = std::find_if(options.begin(), options.end(), [&](const CameraProfileOption& option) {
            return option.color_is_rgb && option.color_fps == option.depth_fps
                   && (target_fps == 0 || option.color_fps == target_fps);
        });
        if(selected != options.end()) {
            return selected->index;
        }
    }
    return std::nullopt;
}

inline std::vector<int> supported_camera_fps(const std::vector<CameraProfileOption>& options) {
    std::vector<int> values;
    for(const auto& option : options) {
        if(!option.color_is_rgb || option.color_fps != option.depth_fps || option.color_fps <= 0) {
            continue;
        }
        if(std::find(values.begin(), values.end(), option.color_fps) == values.end()) {
            values.push_back(option.color_fps);
        }
    }
    std::sort(values.begin(), values.end());
    return values;
}

inline std::vector<DepthFilterKind> depth_filter_plan(const CameraConfig& config,
                                                       const bool temporal_available,
                                                       const bool spatial_available,
                                                       const bool hole_filling_available) {
    std::vector<DepthFilterKind> stages;
    if(config.sdk_temporal_filter && temporal_available) {
        stages.push_back(DepthFilterKind::Temporal);
    }
    if(config.sdk_spatial_filter && spatial_available) {
        stages.push_back(DepthFilterKind::Spatial);
    }
    if(config.hole_filling_filter && hole_filling_available) {
        stages.push_back(DepthFilterKind::HoleFilling);
    }
    return stages;
}

template<typename Frame>
struct ResilientFilterResult {
    Frame frame;
    std::vector<DepthFilterKind> failed_stages;
};

template<typename Frame, typename Processor>
ResilientFilterResult<Frame> process_resilient_filter_chain(Frame input,
                                                             const std::vector<DepthFilterKind>& stages,
                                                             Processor&& process) {
    ResilientFilterResult<Frame> result{ std::move(input), {} };
    for(const auto stage : stages) {
        try {
            auto candidate = process(stage, result.frame);
            if(candidate) {
                result.frame = std::move(candidate);
            }
            else {
                result.failed_stages.push_back(stage);
            }
        }
        catch(...) {
            result.failed_stages.push_back(stage);
        }
    }
    return result;
}

template<typename T>
std::optional<T> select_supported_setting(const T& requested, const std::vector<T>& supported) {
    if(std::find(supported.begin(), supported.end(), requested) == supported.end()) {
        return std::nullopt;
    }
    return requested;
}

}  // namespace aerial_touch
