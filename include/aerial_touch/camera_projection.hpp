#pragma once

#include "aerial_touch/rgbd_frame.hpp"
#include "aerial_touch/types.hpp"

#include <optional>

namespace aerial_touch {

// Pinhole de-projection and projection shared by the runtime camera and the tests.
//
// These two functions are exact inverses of each other. `deproject_pixel` matches the
// model the Orbbec SDK uses for `transformation2dto3d` (no distortion parameter is taken
// there, so the mapping is plain pinhole): the pixel and depth are lifted into the source
// camera frame, then the extrinsic maps them into the target frame. `project_point` walks
// the same chain backwards, so a point produced by `deproject_pixel` lands back on the
// pixel it came from.
//
// Keeping both in one place is what lets the on-table keypad overlay use *exactly* the
// camera model the touch distances are computed with: if the intrinsics are wrong, the
// drawn keypad visibly drifts off the physical target instead of failing silently.

std::optional<Vec3> deproject_pixel(const CameraIntrinsics& intrinsics,
                                    const CameraExtrinsics& source_to_target,
                                    Vec2 pixel,
                                    float depth_mm);

std::optional<Vec2> project_point(const CameraIntrinsics& intrinsics,
                                  const CameraExtrinsics& source_to_target,
                                  Vec3 point_mm);

}  // namespace aerial_touch
