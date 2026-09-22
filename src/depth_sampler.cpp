#include "aerial_touch/depth_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace aerial_touch {

std::optional<float> sample_depth_median_mm(
    const std::vector<std::uint16_t>& depth_values,
    const int width,
    const int height,
    const int pixel_x,
    const int pixel_y,
    const int sample_radius,
    const float depth_unit_mm) {
    if(width <= 0 || height <= 0 || sample_radius < 0 || !std::isfinite(depth_unit_mm) || depth_unit_mm <= 0.0F
       || depth_values.size() < static_cast<std::size_t>(width) * static_cast<std::size_t>(height)) {
        return std::nullopt;
    }

    const int left   = std::max(0, pixel_x - sample_radius);
    const int right  = std::min(width - 1, pixel_x + sample_radius);
    const int top    = std::max(0, pixel_y - sample_radius);
    const int bottom = std::min(height - 1, pixel_y + sample_radius);
    if(left > right || top > bottom) {
        return std::nullopt;
    }

    std::vector<float> valid_depths;
    valid_depths.reserve(static_cast<std::size_t>((right - left + 1) * (bottom - top + 1)));
    for(int y = top; y <= bottom; ++y) {
        for(int x = left; x <= right; ++x) {
            const std::uint16_t raw_depth = depth_values[static_cast<std::size_t>(y) * static_cast<std::size_t>(width)
                                                         + static_cast<std::size_t>(x)];
            if(raw_depth != 0U) {
                valid_depths.push_back(static_cast<float>(raw_depth) * depth_unit_mm);
            }
        }
    }

    if(valid_depths.empty()) {
        return std::nullopt;
    }

    const auto middle = valid_depths.begin() + static_cast<std::ptrdiff_t>(valid_depths.size() / 2U);
    std::nth_element(valid_depths.begin(), middle, valid_depths.end());
    if(valid_depths.size() % 2U == 1U) {
        return *middle;
    }

    const float upper_middle = *middle;
    const auto lower_middle  = std::max_element(valid_depths.begin(), middle);
    return (*lower_middle + upper_middle) / 2.0F;
}

std::vector<DepthPixelSample> sample_depth_surface_grid_mm(
    const std::vector<std::uint16_t>& depth_values,
    const int width,
    const int height,
    const SurfaceSampleRegion& region,
    const float depth_unit_mm,
    const std::vector<Vec2>& excluded_points,
    const float exclusion_radius_px) {
    if(width <= 0 || height <= 0 || region.radius_px <= 0 || region.stride_px <= 0
       || !std::isfinite(depth_unit_mm) || depth_unit_mm <= 0.0F
       || !std::isfinite(region.minimum_depth_mm) || region.minimum_depth_mm < 0.0F
       || !std::isfinite(region.maximum_depth_mm) || region.maximum_depth_mm < 0.0F
       || !std::isfinite(exclusion_radius_px) || exclusion_radius_px < 0.0F
       || depth_values.size() < static_cast<std::size_t>(width) * static_cast<std::size_t>(height)) {
        return {};
    }

    const int left   = std::max(0, region.center_x - region.radius_px);
    const int right  = std::min(width - 1, region.center_x + region.radius_px);
    const int top    = std::max(0, region.center_y - region.radius_px);
    const int bottom = std::min(height - 1, region.center_y + region.radius_px);
    if(left > right || top > bottom) {
        return {};
    }

    const int radius_squared = region.radius_px * region.radius_px;
    const float exclusion_squared = exclusion_radius_px * exclusion_radius_px;

    std::vector<DepthPixelSample> samples;
    samples.reserve(static_cast<std::size_t>(((right - left) / region.stride_px + 1))
                    * static_cast<std::size_t>(((bottom - top) / region.stride_px + 1)));

    // Walk the grid from the centre outwards in both axes so the stride stays anchored on the
    // pointing location rather than on the clipped window edge.
    const int first_x = region.center_x - ((region.center_x - left) / region.stride_px) * region.stride_px;
    const int first_y = region.center_y - ((region.center_y - top) / region.stride_px) * region.stride_px;

    for(int y = first_y; y <= bottom; y += region.stride_px) {
        for(int x = first_x; x <= right; x += region.stride_px) {
            const int offset_x = x - region.center_x;
            const int offset_y = y - region.center_y;
            if(offset_x * offset_x + offset_y * offset_y > radius_squared) {
                continue;
            }

            bool excluded = false;
            for(const Vec2 point : excluded_points) {
                if(!std::isfinite(point.x) || !std::isfinite(point.y)) {
                    continue;
                }
                const float dx = static_cast<float>(x) - point.x;
                const float dy = static_cast<float>(y) - point.y;
                if(dx * dx + dy * dy <= exclusion_squared) {
                    excluded = true;
                    break;
                }
            }
            if(excluded) {
                continue;
            }

            const std::uint16_t raw_depth = depth_values[static_cast<std::size_t>(y) * static_cast<std::size_t>(width)
                                                         + static_cast<std::size_t>(x)];
            if(raw_depth == 0U) {
                continue;
            }
            const float depth_mm = static_cast<float>(raw_depth) * depth_unit_mm;
            if(depth_mm < region.minimum_depth_mm) {
                continue;
            }
            if(region.maximum_depth_mm > 0.0F && depth_mm > region.maximum_depth_mm) {
                continue;
            }
            samples.push_back({ x, y, depth_mm });
        }
    }
    return samples;
}

}  // namespace aerial_touch
