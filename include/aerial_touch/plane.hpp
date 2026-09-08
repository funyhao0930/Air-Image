#pragma once

#include "aerial_touch/types.hpp"

#include <optional>

namespace aerial_touch {

struct PlanePoint {
    float u_mm{};
    float v_mm{};
    float signed_distance_mm{};
};

class Plane {
public:
    static std::optional<Plane> from_calibration_points(
        Vec3 origin,
        Vec3 u_reference,
        Vec3 v_reference,
        float minimum_point_distance_mm = 80.0F);

    PlanePoint project(Vec3 point) const;

    // Inverse of project(): rebuild the camera-space point that sits at (u, v) on the plane,
    // offset by signed_distance_mm along the camera-facing normal. Used to draw the calibrated
    // keypad back onto the physical surface in the video frame.
    Vec3 unproject(Vec2 uv_mm, float signed_distance_mm = 0.0F) const;

private:
    Plane(Vec3 origin, Vec3 u_axis, Vec3 v_axis, Vec3 normal);

    Vec3 origin_{};
    Vec3 u_axis_{};
    Vec3 v_axis_{};
    Vec3 normal_{};
};

}  // namespace aerial_touch
