#include "aerial_touch/calibration_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace aerial_touch {
namespace {

constexpr std::size_t kOriginPoint = 0U;
constexpr std::size_t kRightBoundaryPoint = 1U;
constexpr std::size_t kBottomBoundaryPoint = 2U;
constexpr std::size_t kFirstKeyRightPoint = 3U;
constexpr std::size_t kSecondKeyRightPoint = 4U;
constexpr std::size_t kFirstKeyBottomPoint = 5U;
constexpr std::size_t kSecondRowBottomPoint = 6U;
constexpr float kMinimumDimensionMm = 0.001F;

bool finite(const PlanePoint point) {
    return std::isfinite(point.u_mm) && std::isfinite(point.v_mm) && std::isfinite(point.signed_distance_mm);
}

bool close_enough(const float actual, const float expected, const float tolerance) {
    return std::fabs(actual - expected) <= tolerance;
}

}  // namespace

std::optional<KeypadCalibrationResult> calibrate_keypad(
    const std::array<Vec3, 7>& points,
    const float minimum_point_distance_mm) {
    const auto plane = Plane::from_calibration_points(points[kOriginPoint], points[kRightBoundaryPoint],
                                                       points[kBottomBoundaryPoint], minimum_point_distance_mm);
    if(!plane.has_value()) {
        return std::nullopt;
    }

    std::array<PlanePoint, 7> projected{};
    for(std::size_t index = 0; index < points.size(); ++index) {
        projected[index] = plane->project(points[index]);
        if(!finite(projected[index])) {
            return std::nullopt;
        }
    }

    const float total_width = projected[kRightBoundaryPoint].u_mm;
    const float total_height = projected[kBottomBoundaryPoint].v_mm;
    const float key_width = projected[kFirstKeyRightPoint].u_mm;
    const float horizontal_pitch = projected[kSecondKeyRightPoint].u_mm - key_width;
    const float raw_horizontal_gap = horizontal_pitch - key_width;
    const float key_height = projected[kFirstKeyBottomPoint].v_mm;
    const float vertical_pitch = projected[kSecondRowBottomPoint].v_mm - key_height;
    const float raw_vertical_gap = vertical_pitch - key_height;

    if(!std::isfinite(total_width) || !std::isfinite(total_height) || !std::isfinite(key_width)
       || !std::isfinite(horizontal_pitch) || !std::isfinite(raw_horizontal_gap) || !std::isfinite(key_height)
       || !std::isfinite(vertical_pitch) || !std::isfinite(raw_vertical_gap)
       || total_width <= kMinimumDimensionMm || total_height <= kMinimumDimensionMm
       || key_width <= kMinimumDimensionMm || key_height <= kMinimumDimensionMm
       || horizontal_pitch <= kMinimumDimensionMm || vertical_pitch <= kMinimumDimensionMm) {
        return std::nullopt;
    }

    const float tolerance = std::max(5.0F, 0.15F * std::min(key_width, key_height));
    if(raw_horizontal_gap < -tolerance || raw_vertical_gap < -tolerance) {
        return std::nullopt;
    }

    const float horizontal_gap = std::max(0.0F, raw_horizontal_gap);
    const float vertical_gap = std::max(0.0F, raw_vertical_gap);
    if(!close_enough(projected[kRightBoundaryPoint].v_mm, 0.0F, tolerance)
       || !close_enough(projected[kBottomBoundaryPoint].u_mm, total_width / 2.0F, tolerance)
       || !close_enough(projected[kFirstKeyRightPoint].v_mm, 0.0F, tolerance)
       || !close_enough(projected[kSecondKeyRightPoint].v_mm, 0.0F, tolerance)
       || !close_enough(projected[kFirstKeyBottomPoint].u_mm, 0.0F, tolerance)
       || !close_enough(projected[kSecondRowBottomPoint].u_mm, 0.0F, tolerance)
       || !close_enough(total_width, 3.0F * key_width + 2.0F * horizontal_gap, tolerance)
       || !close_enough(total_height, 4.0F * key_height + 3.0F * vertical_gap, tolerance)) {
        return std::nullopt;
    }

    return KeypadCalibrationResult{
        *plane,
        { total_width, total_height, key_width, key_height, horizontal_gap, vertical_gap },
    };
}

}  // namespace aerial_touch
