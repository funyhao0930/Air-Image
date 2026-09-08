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

// `minimum_depth_mm` drops every sample at or in front of that depth. The surface scan uses it to
// keep only pixels *behind* the fingertip: a ring a few pixels wide around the tip lands largely on
// the finger itself (a finger is roughly 12-19 px across at typical working distances), and that
// contamination is what starves the plane fit of its required inlier ratio. The table is always
// farther from the camera than the finger occluding it, so one depth comparison separates them.
std::vector<DepthPixelSample> sample_depth_annulus_mm(
    const std::vector<std::uint16_t>& depth_values,
    int width,
    int height,
    int pixel_x,
    int pixel_y,
    int inner_radius,
    int outer_radius,
    float depth_unit_mm,
    float minimum_depth_mm = 0.0F);

}  // namespace aerial_touch
