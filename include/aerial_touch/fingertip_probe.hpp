#pragma once

#include "aerial_touch/types.hpp"

#include <functional>
#include <optional>

namespace aerial_touch {

// MediaPipe landmark 8 (the index fingertip) sits on the *silhouette edge* of the finger,
// which is the worst place on the image to read a depth camera: the ROI straddles finger and
// background, so the median mixes two populations. That error peaks in the 10-20 mm band --
// exactly where the touch/release decision happens.
//
// Instead we probe the depth a little way back along the finger axis, where the finger is a
// solid blob, and extrapolate the linear depth gradient forward to the tip. The tip *pixel* is
// still used for the plane u/v (it decides which key), only the depth comes from the probes.

struct FingertipDepthProbeConfig {
    float near_ratio{ 0.35F };                 // fraction of the tip->joint span for the near probe
    float far_ratio{ 0.70F };                  // fraction for the far probe (used for the gradient)
    float maximum_extrapolation_mm{ 40.0F };   // clamp so a noisy gradient cannot fling the tip away
    float minimum_pixel_span{ 4.0F };          // below this the finger points at the camera; skip extrapolation
};

struct FingertipDepthEstimate {
    float depth_mm{};
    Vec2 near_pixel{};
    Vec2 far_pixel{};
    bool extrapolated{ false };  // false when only the near probe was usable
};

bool valid_fingertip_probe_config(const FingertipDepthProbeConfig& config);

// `sample` reads a stabilised depth at a pixel and returns nullopt where the depth is invalid.
std::optional<FingertipDepthEstimate> estimate_fingertip_depth_mm(
    Vec2 tip_pixel,
    Vec2 joint_pixel,
    const FingertipDepthProbeConfig& config,
    const std::function<std::optional<float>(Vec2)>& sample);

}  // namespace aerial_touch
