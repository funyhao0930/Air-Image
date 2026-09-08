#pragma once

#include "aerial_touch/keypad.hpp"
#include "aerial_touch/plane.hpp"

#include <array>
#include <cstddef>
#include <optional>

namespace aerial_touch {

// Three points fully determine the keypad: an origin, the far end of the top edge, and the
// bottom edge. Everything else (key size) follows from dividing the rectangle 3 x 4.
//
// The earlier seven-point form measured key size and inter-key gap as independent quantities,
// but `Keypad` only ever lays out a uniform grid from them, so the four extra points bought
// nothing except a dead zone between keys -- and in mid-air, where pointing accuracy is a few
// millimetres, a dead zone only lowers the hit rate. They also added four more chances for a
// cross-consistency check to fail and discard the whole session.
constexpr std::size_t kKeypadCalibrationPointCount = 3U;

// Index meanings for the calibration point array.
constexpr std::size_t kKeypadOriginPoint = 0U;         // top-left corner of key "1"
constexpr std::size_t kKeypadTopRightPoint = 1U;       // top-right corner of key "3"
constexpr std::size_t kKeypadBottomPoint = 2U;         // bottom edge, directly below key "0"

struct KeypadCalibrationResult {
    Plane plane;
    KeypadGeometry geometry;
};

enum class KeypadCalibrationFailure {
    None,
    InvalidPlane,
    InvalidDimensions,
    BottomPointOutsideKeypad,
};

struct KeypadCalibrationAttempt {
    std::optional<KeypadCalibrationResult> result;
    KeypadCalibrationFailure failure{ KeypadCalibrationFailure::None };
};

KeypadCalibrationAttempt calibrate_keypad_detailed(
    const std::array<Vec3, kKeypadCalibrationPointCount>& points,
    float minimum_point_distance_mm = 80.0F);

std::optional<KeypadCalibrationResult> calibrate_keypad(
    const std::array<Vec3, kKeypadCalibrationPointCount>& points,
    float minimum_point_distance_mm = 80.0F);

}  // namespace aerial_touch
