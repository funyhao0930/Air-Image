#include "aerial_touch/keypad.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace aerial_touch {

Keypad::Keypad(const KeypadGeometry geometry) {
    const std::array<std::array<const char*, 3>, 3> rows{ {
        { "1", "2", "3" },
        { "4", "5", "6" },
        { "7", "8", "9" },
    } };

    for(std::size_t row = 0; row < rows.size(); ++row) {
        for(std::size_t column = 0; column < rows[row].size(); ++column) {
            const float u = static_cast<float>(column) * (geometry.key_width_mm + geometry.horizontal_gap_mm);
            const float v = static_cast<float>(row) * (geometry.key_height_mm + geometry.vertical_gap_mm);
            regions_.push_back({ rows[row][column], u, u + geometry.key_width_mm, v,
                                 v + geometry.key_height_mm });
        }
    }

    const float zero_u = geometry.key_width_mm + geometry.horizontal_gap_mm;
    const float zero_v = 3.0F * (geometry.key_height_mm + geometry.vertical_gap_mm);
    regions_.push_back({ "0", zero_u, zero_u + geometry.key_width_mm, zero_v,
                         zero_v + geometry.key_height_mm });
}

std::optional<std::string> Keypad::key_at(const Vec2 uv_mm) const {
    for(const auto& region : regions_) {
        if(uv_mm.x >= region.u_min_mm && uv_mm.x < region.u_max_mm && uv_mm.y >= region.v_min_mm
           && uv_mm.y < region.v_max_mm) {
            return region.key;
        }
    }
    return std::nullopt;
}

std::optional<std::string> Keypad::key_at(const Vec2 uv_mm,
                                          const std::optional<std::string>& previous_key,
                                          const float boundary_hysteresis_mm) const {
    if(!std::isfinite(boundary_hysteresis_mm) || boundary_hysteresis_mm < 0.0F) {
        throw std::invalid_argument(u8"按鍵邊界遲滯設定無效");
    }
    if(previous_key.has_value()) {
        const auto previous_region = std::find_if(regions_.begin(), regions_.end(), [&](const KeyRegion& region) {
            return region.key == *previous_key;
        });
        if(previous_region != regions_.end()
           && uv_mm.x >= previous_region->u_min_mm - boundary_hysteresis_mm
           && uv_mm.x < previous_region->u_max_mm + boundary_hysteresis_mm
           && uv_mm.y >= previous_region->v_min_mm - boundary_hysteresis_mm
           && uv_mm.y < previous_region->v_max_mm + boundary_hysteresis_mm) {
            return previous_key;
        }
    }
    return key_at(uv_mm);
}

const std::vector<KeyRegion>& Keypad::regions() const {
    return regions_;
}

}  // namespace aerial_touch
