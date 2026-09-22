#pragma once

#include "aerial_touch/camera_settings.hpp"
#include "aerial_touch/keypad.hpp"
#include "aerial_touch/touch_state_machine.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>

namespace aerial_touch {

struct DepthSamplingConfig {
    int sample_radius{ 2 };
    std::size_t median_window_size{ 5U };
    float max_jump_mm{ 80.0F };
    std::size_t invalid_reset_frames{ 3U };
    // Surface-scan samples more than this far *in front of* the fingertip are treated as finger,
    // not table. The finger rises away from the surface behind the tip, so its pixels read nearer
    // to the camera; table pixels read at the tip's depth or farther, including when the fingertip
    // is resting on the surface.
    float finger_clearance_mm{ 3.0F };
    // Radius of the surface-scan sampling disc, in pixels. This must cover a large fraction of the
    // keypad: a plane fitted to a narrow strip of points leaves its normal free to rotate about the
    // strip, which no residual or inlier count can detect.
    int surface_scan_radius_px{ 150 };
    int surface_scan_stride_px{ 5 };
    // Pixels within this distance of any hand landmark are never treated as surface. A hand held
    // against the target is nearly coplanar with it, so depth alone cannot separate them.
    float hand_exclusion_px{ 40.0F };
    // Samples farther behind the fingertip than this are background, not the target surface.
    float surface_depth_window_mm{ 150.0F };
};

struct FingertipConfig {
    float min_cutoff_hz{ 1.0F };
    float beta{ 0.12F };
    float derivative_cutoff_hz{ 1.0F };
    std::int64_t display_hold_ms{ 100 };
    // Where along the tip -> DIP-joint span the depth probes sit. The tip pixel itself is on the
    // finger's silhouette edge, so its depth mixes finger and background.
    float depth_probe_near_ratio{ 0.35F };
    float depth_probe_far_ratio{ 0.70F };
};

struct CalibrationConfig {
    float minimum_point_distance_mm{ 80.0F };
    std::size_t required_samples{ 18U };
    float mad_multiplier{ 3.5F };
    float minimum_outlier_threshold_mm{ 2.0F };
};

struct AppConfig {
    CameraConfig camera;
    DepthSamplingConfig depth;
    FingertipConfig fingertip;
    TouchConfig touch;
    KeypadConfig keypad;
    CalibrationConfig calibration;
};

AppConfig load_app_config(const std::filesystem::path& path);
void save_app_config(const AppConfig& config, const std::filesystem::path& path);
void validate_app_config(const AppConfig& config);

}  // namespace aerial_touch
