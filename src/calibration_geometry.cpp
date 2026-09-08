#include "aerial_touch/calibration_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace aerial_touch {
namespace {

constexpr float kMinimumDimensionMm = 0.001F;
// The bottom point is aimed below key "0", i.e. the horizontal middle of the keypad. Half a key
// width of slack on each side still catches a grossly mis-aimed point without being fussy about
// a few millimetres of pointing error.
constexpr float kBottomPointSlackRatio = 0.5F;

bool finite(const PlanePoint point) {
    return std::isfinite(point.u_mm) && std::isfinite(point.v_mm) && std::isfinite(point.signed_distance_mm);
}

}  // namespace

KeypadCalibrationAttempt calibrate_keypad_detailed(
    const std::array<Vec3, kKeypadCalibrationPointCount>& points,
    const float minimum_point_distance_mm) {
    const auto plane = Plane::from_calibration_points(points[kKeypadOriginPoint], points[kKeypadTopRightPoint],
                                                      points[kKeypadBottomPoint], minimum_point_distance_mm);
    if(!plane.has_value()) {
        return { std::nullopt, KeypadCalibrationFailure::InvalidPlane };
    }

    std::array<PlanePoint, kKeypadCalibrationPointCount> projected{};
    for(std::size_t index = 0; index < points.size(); ++index) {
        projected[index] = plane->project(points[index]);
        if(!finite(projected[index])) {
            return { std::nullopt, KeypadCalibrationFailure::InvalidDimensions };
        }
    }

    // The plane is built so that the origin projects to (0, 0), the top-right point lies on the
    // +u axis, and the bottom point's v component is its orthogonal distance from the top edge.
    const float total_width  = projected[kKeypadTopRightPoint].u_mm;
    const float total_height = projected[kKeypadBottomPoint].v_mm;
    if(!std::isfinite(total_width) || !std::isfinite(total_height) || total_width <= kMinimumDimensionMm
       || total_height <= kMinimumDimensionMm) {
        return { std::nullopt, KeypadCalibrationFailure::InvalidDimensions };
    }

    const float key_width  = total_width / 3.0F;
    const float key_height = total_height / 4.0F;
    if(!std::isfinite(key_width) || !std::isfinite(key_height) || key_width <= kMinimumDimensionMm
       || key_height <= kMinimumDimensionMm) {
        return { std::nullopt, KeypadCalibrationFailure::InvalidDimensions };
    }

    const float slack = key_width * kBottomPointSlackRatio;
    const float bottom_u = projected[kKeypadBottomPoint].u_mm;
    if(bottom_u < -slack || bottom_u > total_width + slack) {
        return { std::nullopt, KeypadCalibrationFailure::BottomPointOutsideKeypad };
    }

    return { KeypadCalibrationResult{
                 *plane,
                 { total_width, total_height, key_width, key_height, 0.0F, 0.0F },
             },
             KeypadCalibrationFailure::None };
}

std::optional<KeypadCalibrationResult> calibrate_keypad(
    const std::array<Vec3, kKeypadCalibrationPointCount>& points,
    const float minimum_point_distance_mm) {
    return calibrate_keypad_detailed(points, minimum_point_distance_mm).result;
}

}  // namespace aerial_touch
