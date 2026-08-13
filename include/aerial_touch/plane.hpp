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

private:
    Plane(Vec3 origin, Vec3 u_axis, Vec3 v_axis, Vec3 normal);

    Vec3 origin_{};
    Vec3 u_axis_{};
    Vec3 v_axis_{};
    Vec3 normal_{};
};

}  // namespace aerial_touch
