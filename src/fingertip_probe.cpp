#include "aerial_touch/fingertip_probe.hpp"

#include <algorithm>
#include <cmath>

namespace aerial_touch {
namespace {

Vec2 blend(const Vec2 from, const Vec2 to, const float ratio) {
    return { from.x + (to.x - from.x) * ratio, from.y + (to.y - from.y) * ratio };
}

bool finite(const Vec2 value) {
    return std::isfinite(value.x) && std::isfinite(value.y);
}

}  // namespace

bool valid_fingertip_probe_config(const FingertipDepthProbeConfig& config) {
    return std::isfinite(config.near_ratio) && std::isfinite(config.far_ratio)
           && std::isfinite(config.maximum_extrapolation_mm) && std::isfinite(config.minimum_pixel_span)
           && config.near_ratio > 0.0F && config.far_ratio > config.near_ratio && config.far_ratio <= 1.0F
           && config.maximum_extrapolation_mm > 0.0F && config.minimum_pixel_span >= 0.0F;
}

std::optional<FingertipDepthEstimate> estimate_fingertip_depth_mm(
    const Vec2 tip_pixel,
    const Vec2 joint_pixel,
    const FingertipDepthProbeConfig& config,
    const std::function<std::optional<float>(Vec2)>& sample) {
    if(!valid_fingertip_probe_config(config) || !finite(tip_pixel) || !finite(joint_pixel) || !sample) {
        return std::nullopt;
    }

    const Vec2 near_pixel = blend(tip_pixel, joint_pixel, config.near_ratio);
    const Vec2 far_pixel  = blend(tip_pixel, joint_pixel, config.far_ratio);

    const auto near_depth = sample(near_pixel);
    if(!near_depth.has_value() || !std::isfinite(*near_depth) || *near_depth <= 0.0F) {
        return std::nullopt;
    }

    FingertipDepthEstimate estimate{ *near_depth, near_pixel, far_pixel, false };

    const float span = std::hypot(joint_pixel.x - tip_pixel.x, joint_pixel.y - tip_pixel.y);
    if(!std::isfinite(span) || span < config.minimum_pixel_span) {
        // The finger points roughly at the camera, so tip and joint land on the same pixels and
        // the near probe already reads the tip. Extrapolating here would amplify pure noise.
        return estimate;
    }

    const auto far_depth = sample(far_pixel);
    if(!far_depth.has_value() || !std::isfinite(*far_depth) || *far_depth <= 0.0F) {
        return estimate;
    }

    // depth(t) is taken to be linear in t along the finger; solve for depth(0), the tip.
    const float gradient_span = config.far_ratio - config.near_ratio;
    const float tip_depth =
        *near_depth + (*near_depth - *far_depth) * (config.near_ratio / gradient_span);
    if(!std::isfinite(tip_depth)) {
        return estimate;
    }

    const float offset = std::clamp(tip_depth - *near_depth, -config.maximum_extrapolation_mm,
                                    config.maximum_extrapolation_mm);
    const float corrected = *near_depth + offset;
    if(!std::isfinite(corrected) || corrected <= 0.0F) {
        return estimate;
    }

    estimate.depth_mm     = corrected;
    estimate.extrapolated = true;
    return estimate;
}

}  // namespace aerial_touch
