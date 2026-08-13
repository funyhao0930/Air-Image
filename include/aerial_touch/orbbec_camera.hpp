#pragma once

#include "aerial_touch/rgbd_frame.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace aerial_touch {

class OrbbecCamera {
public:
    OrbbecCamera();
    ~OrbbecCamera();

    OrbbecCamera(const OrbbecCamera&) = delete;
    OrbbecCamera& operator=(const OrbbecCamera&) = delete;

    bool start();
    void stop();
    std::optional<RgbdFrame> capture(std::uint32_t timeout_ms = 100);
    std::optional<Vec3> deproject(const RgbdFrame& frame, Vec2 pixel, float depth_mm) const;

    bool running() const;
    bool hardware_alignment() const;
    const std::string& error() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace aerial_touch
