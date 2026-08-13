#include "aerial_touch/plane.hpp"

#include <cmath>

namespace aerial_touch {
namespace {

Vec3 subtract(const Vec3 a, const Vec3 b) {
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}

Vec3 scale(const Vec3 value, const float factor) {
    return { value.x * factor, value.y * factor, value.z * factor };
}

float dot(const Vec3 a, const Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3 a, const Vec3 b) {
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

float length(const Vec3 value) {
    return std::sqrt(dot(value, value));
}

std::optional<Vec3> normalized(const Vec3 value) {
    const float magnitude = length(value);
    if(!std::isfinite(magnitude) || magnitude < 0.001F) {
        return std::nullopt;
    }
    return scale(value, 1.0F / magnitude);
}

}  // namespace

Plane::Plane(const Vec3 origin, const Vec3 u_axis, const Vec3 v_axis, const Vec3 normal)
    : origin_(origin), u_axis_(u_axis), v_axis_(v_axis), normal_(normal) {}

std::optional<Plane> Plane::from_calibration_points(const Vec3 origin,
                                                    const Vec3 u_reference,
                                                    const Vec3 v_reference,
                                                    const float minimum_point_distance_mm) {
    const Vec3 raw_u = subtract(u_reference, origin);
    const Vec3 raw_v = subtract(v_reference, origin);
    if(!std::isfinite(minimum_point_distance_mm) || minimum_point_distance_mm <= 0.0F
       || length(raw_u) < minimum_point_distance_mm || length(raw_v) < minimum_point_distance_mm) {
        return std::nullopt;
    }

    const auto u_axis = normalized(raw_u);
    if(!u_axis.has_value()) {
        return std::nullopt;
    }

    const Vec3 v_orthogonal = subtract(raw_v, scale(*u_axis, dot(raw_v, *u_axis)));
    const auto v_axis       = normalized(v_orthogonal);
    if(!v_axis.has_value() || length(v_orthogonal) / length(raw_v) < 0.1F) {
        return std::nullopt;
    }

    auto normal = normalized(cross(*u_axis, *v_axis));
    if(!normal.has_value()) {
        return std::nullopt;
    }

    const Vec3 toward_camera = scale(origin, -1.0F);
    if(dot(*normal, toward_camera) < 0.0F) {
        *normal = scale(*normal, -1.0F);
    }

    return Plane(origin, *u_axis, *v_axis, *normal);
}

PlanePoint Plane::project(const Vec3 point) const {
    const Vec3 relative = subtract(point, origin_);
    return { dot(relative, u_axis_), dot(relative, v_axis_), dot(relative, normal_) };
}

}  // namespace aerial_touch
