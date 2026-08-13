#include "aerial_touch/keypad.hpp"

#include <array>

namespace aerial_touch {

Keypad::Keypad(const KeypadConfig config) {
    const std::array<std::array<const char*, 3>, 3> rows{ {
        { "1", "2", "3" },
        { "4", "5", "6" },
        { "7", "8", "9" },
    } };

    for(std::size_t row = 0; row < rows.size(); ++row) {
        for(std::size_t column = 0; column < rows[row].size(); ++column) {
            const float u = static_cast<float>(column) * (config.key_width_mm + config.horizontal_gap_mm);
            const float v = static_cast<float>(row) * (config.key_height_mm + config.vertical_gap_mm);
            regions_.push_back({ rows[row][column], u, u + config.key_width_mm, v, v + config.key_height_mm });
        }
    }

    const float zero_u = config.key_width_mm + config.horizontal_gap_mm;
    const float zero_v = 3.0F * (config.key_height_mm + config.vertical_gap_mm);
    regions_.push_back({ "0", zero_u, zero_u + config.key_width_mm, zero_v, zero_v + config.key_height_mm });
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

const std::vector<KeyRegion>& Keypad::regions() const {
    return regions_;
}

}  // namespace aerial_touch
