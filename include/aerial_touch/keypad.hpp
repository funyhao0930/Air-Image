#pragma once

#include "aerial_touch/types.hpp"

#include <optional>
#include <string>
#include <vector>

namespace aerial_touch {

struct KeypadConfig {
    float boundary_hysteresis_mm{ 2.0F };
};

struct KeypadGeometry {
    float total_width_mm{};
    float total_height_mm{};
    float key_width_mm{};
    float key_height_mm{};
    float horizontal_gap_mm{};
    float vertical_gap_mm{};
};

struct KeyRegion {
    std::string key;
    float u_min_mm{};
    float u_max_mm{};
    float v_min_mm{};
    float v_max_mm{};
};

class Keypad {
public:
    explicit Keypad(KeypadGeometry geometry);

    std::optional<std::string> key_at(Vec2 uv_mm) const;
    std::optional<std::string> key_at(Vec2 uv_mm,
                                      const std::optional<std::string>& previous_key,
                                      float boundary_hysteresis_mm) const;
    const std::vector<KeyRegion>& regions() const;

private:
    std::vector<KeyRegion> regions_;
};

}  // namespace aerial_touch
