#pragma once

#include "aerial_touch/camera_settings.hpp"
#include "aerial_touch/rgbd_frame.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace aerial_touch {

struct CameraRuntimeInfo {
    std::string depth_work_mode{ u8"裝置預設" };
    std::string depth_precision{ u8"未知" };
    int depth_width{};
    int depth_height{};
    int color_width{};
    int color_height{};
    int fps{};
    bool hardware_alignment{ false };
    bool temporal_filter{ false };
    bool spatial_filter{ false };
    bool hole_filling_filter{ false };
    std::vector<std::string> warnings;
};

class OrbbecCamera {
public:
    OrbbecCamera();
    ~OrbbecCamera();

    OrbbecCamera(const OrbbecCamera&) = delete;
    OrbbecCamera& operator=(const OrbbecCamera&) = delete;

    bool start(const CameraConfig& config = {});
    void stop();
    void reset_depth_filters();
    std::optional<RgbdFrame> capture(std::uint32_t timeout_ms = 100);
    std::optional<Vec3> deproject(const RgbdFrame& frame, Vec2 pixel, float depth_mm) const;

    bool running() const;
    bool hardware_alignment() const;
    const CameraRuntimeInfo& runtime_info() const;
    const std::string& error() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace aerial_touch
