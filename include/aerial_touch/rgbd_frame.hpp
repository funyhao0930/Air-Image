#pragma once

#include "aerial_touch/types.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace aerial_touch {

struct CameraIntrinsics {
    float fx{};
    float fy{};
    float cx{};
    float cy{};
    int width{};
    int height{};
};

struct CameraExtrinsics {
    std::array<float, 9> rotation{};
    std::array<float, 3> translation_mm{};
};

struct RgbdFrame {
    std::vector<std::uint8_t> rgb;
    std::vector<std::uint16_t> depth;
    int color_width{};
    int color_height{};
    int depth_width{};
    int depth_height{};
    std::int64_t timestamp_ms{};
    float depth_unit_mm{};
    CameraIntrinsics depth_intrinsics;
    CameraExtrinsics depth_to_color;
    bool profiles_valid{ false };

    bool valid() const {
        return color_width > 0 && color_height > 0 && color_width == depth_width && color_height == depth_height
               && rgb.size() == static_cast<std::size_t>(color_width) * static_cast<std::size_t>(color_height) * 3U
               && depth.size() == static_cast<std::size_t>(depth_width) * static_cast<std::size_t>(depth_height)
               && depth_unit_mm > 0.0F && profiles_valid;
    }
};

}  // namespace aerial_touch
