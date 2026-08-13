#pragma once

#include "aerial_touch/types.hpp"

#include <optional>
#include <string>
#include <vector>

namespace aerial_touch {

struct KeypadConfig {
    float key_width_mm{ 30.0F };
    float key_height_mm{ 30.0F };
    float horizontal_gap_mm{ 5.0F };
    float vertical_gap_mm{ 5.0F };
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
    explicit Keypad(KeypadConfig config);

    std::optional<std::string> key_at(Vec2 uv_mm) const;
    const std::vector<KeyRegion>& regions() const;

private:
    std::vector<KeyRegion> regions_;
};

}  // namespace aerial_touch
