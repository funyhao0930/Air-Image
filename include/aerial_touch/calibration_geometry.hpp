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

enum class KeypadCalibrationFailure {
    None,
    InvalidPlane,
    InvalidDimensions,
    OverlappingKeys,
    BottomBoundaryOutsideZeroColumn,
    TopBoundaryMismatch,
    LeftBoundaryMismatch,
    TotalSizeMismatch,
};

struct KeypadCalibrationAttempt {
    std::optional<KeypadCalibrationResult> result;
    KeypadCalibrationFailure failure{ KeypadCalibrationFailure::None };
};

KeypadCalibrationAttempt calibrate_keypad_detailed(
    const std::array<Vec3, 7>& points,
    float minimum_point_distance_mm = 80.0F);

std::optional<KeypadCalibrationResult> calibrate_keypad(
    const std::array<Vec3, 7>& points,
    float minimum_point_distance_mm = 80.0F);

}  // namespace aerial_touch
