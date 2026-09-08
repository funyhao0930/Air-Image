#include "aerial_touch/camera_projection.hpp"

#include <cmath>

namespace aerial_touch {
namespace {

bool finite(const Vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool usable(const CameraIntrinsics& intrinsics) {
    return std::isfinite(intrinsics.fx) && std::isfinite(intrinsics.fy) && std::isfinite(intrinsics.cx)
           && std::isfinite(intrinsics.cy) && intrinsics.fx > 0.0F && intrinsics.fy > 0.0F;
}

bool usable(const CameraExtrinsics& extrinsics) {
    for(const float value : extrinsics.rotation) {
        if(!std::isfinite(value)) {
            return false;
        }
    }
    for(const float value : extrinsics.translation_mm) {
        if(!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

// rotation is row-major: target = R * source + t
Vec3 rotate(const std::array<float, 9>& rotation, const Vec3 value) {
    return {
        rotation[0] * value.x + rotation[1] * value.y + rotation[2] * value.z,
        rotation[3] * value.x + rotation[4] * value.y + rotation[5] * value.z,
        rotation[6] * value.x + rotation[7] * value.y + rotation[8] * value.z,
    };
}

// source = R^T * (target - t)
Vec3 rotate_transposed(const std::array<float, 9>& rotation, const Vec3 value) {
    return {
        rotation[0] * value.x + rotation[3] * value.y + rotation[6] * value.z,
        rotation[1] * value.x + rotation[4] * value.y + rotation[7] * value.z,
        rotation[2] * value.x + rotation[5] * value.y + rotation[8] * value.z,
    };
}

}  // namespace

std::optional<Vec3> deproject_pixel(const CameraIntrinsics& intrinsics,
                                    const CameraExtrinsics& source_to_target,
                                    const Vec2 pixel,
                                    const float depth_mm) {
    if(!usable(intrinsics) || !usable(source_to_target) || !std::isfinite(pixel.x) || !std::isfinite(pixel.y)
       || !std::isfinite(depth_mm) || depth_mm <= 0.0F) {
        return std::nullopt;
    }

    const Vec3 in_source{
        (pixel.x - intrinsics.cx) / intrinsics.fx * depth_mm,
        (pixel.y - intrinsics.cy) / intrinsics.fy * depth_mm,
        depth_mm,
    };
    const Vec3 rotated = rotate(source_to_target.rotation, in_source);
    const Vec3 result{
        rotated.x + source_to_target.translation_mm[0],
        rotated.y + source_to_target.translation_mm[1],
        rotated.z + source_to_target.translation_mm[2],
    };
    return finite(result) ? std::optional<Vec3>{ result } : std::nullopt;
}

std::optional<Vec2> project_point(const CameraIntrinsics& intrinsics,
                                  const CameraExtrinsics& source_to_target,
                                  const Vec3 point_mm) {
    if(!usable(intrinsics) || !usable(source_to_target) || !finite(point_mm)) {
        return std::nullopt;
    }

    const Vec3 translated{
        point_mm.x - source_to_target.translation_mm[0],
        point_mm.y - source_to_target.translation_mm[1],
        point_mm.z - source_to_target.translation_mm[2],
    };
    const Vec3 in_source = rotate_transposed(source_to_target.rotation, translated);
    if(!finite(in_source) || in_source.z <= 0.0F) {
        // Behind the camera (or exactly on the pupil plane); there is no pixel for it.
        return std::nullopt;
    }

    const Vec2 pixel{
        in_source.x / in_source.z * intrinsics.fx + intrinsics.cx,
        in_source.y / in_source.z * intrinsics.fy + intrinsics.cy,
    };
    if(!std::isfinite(pixel.x) || !std::isfinite(pixel.y)) {
        return std::nullopt;
    }
    return pixel;
}

}  // namespace aerial_touch
