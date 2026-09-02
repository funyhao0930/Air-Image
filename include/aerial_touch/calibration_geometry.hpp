#pragma once

#include "aerial_touch/keypad.hpp"
#include "aerial_touch/plane.hpp"

#include <array>
#include <optional>

namespace aerial_touch {

struct KeypadCalibrationResult {
    Plane plane;
    KeypadGeometry geometry;
};

std::optional<KeypadCalibrationResult> calibrate_keypad(
    const std::array<Vec3, 7>& points,
    float minimum_point_distance_mm = 80.0F);

}  // namespace aerial_touch
