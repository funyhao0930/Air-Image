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

}  // namespace aerial_touch
