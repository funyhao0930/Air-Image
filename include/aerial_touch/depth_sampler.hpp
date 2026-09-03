#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace aerial_touch {

struct DepthPixelSample {
    int x{};
    int y{};
    float depth_mm{};
};

std::optional<float> sample_depth_median_mm(
    const std::vector<std::uint16_t>& depth_values,
    int width,
    int height,
    int pixel_x,
    int pixel_y,
    int sample_radius,
    float depth_unit_mm);

std::vector<DepthPixelSample> sample_depth_annulus_mm(
    const std::vector<std::uint16_t>& depth_values,
    int width,
    int height,
    int pixel_x,
    int pixel_y,
    int inner_radius,
    int outer_radius,
    float depth_unit_mm);

}  // namespace aerial_touch
