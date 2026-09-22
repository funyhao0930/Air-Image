#pragma once

#include "aerial_touch/types.hpp"

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

// Region the surface scan reads to fit the target plane.
//
// The radius has to be a large fraction of the keypad, not a few pixels: a plane fitted to a narrow
// ribbon of points is well determined along the ribbon and almost unconstrained across it, so its
// normal tilts freely. Inlier count and RMS residual cannot detect that -- a ribbon hugs *some*
// plane beautifully -- which is why `SurfacePlaneFitConfig::minimum_extent_mm` exists as well.
struct SurfaceSampleRegion {
    int center_x{};
    int center_y{};
    int radius_px{ 150 };
    int stride_px{ 5 };
    float minimum_depth_mm{ 0.0F };  // drop samples nearer than this (the pointing finger)
    float maximum_depth_mm{ 0.0F };  // 0 = unbounded; drops background well behind the target
};

// Grid-samples a disc of depth pixels, skipping anything within `exclusion_radius_px` of any point
// in `excluded_points`. The scan passes the hand landmarks there: a hand held against the target is
// nearly coplanar with it, so a depth threshold alone will not separate the two.
std::vector<DepthPixelSample> sample_depth_surface_grid_mm(
    const std::vector<std::uint16_t>& depth_values,
    int width,
    int height,
    const SurfaceSampleRegion& region,
    float depth_unit_mm,
    const std::vector<Vec2>& excluded_points,
    float exclusion_radius_px);

}  // namespace aerial_touch
