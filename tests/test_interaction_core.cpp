#include "aerial_touch/app_config.hpp"
#include "aerial_touch/alignment_mode.hpp"
#include "aerial_touch/camera_projection.hpp"
#include "aerial_touch/camera_settings.hpp"
#include "aerial_touch/calibration_geometry.hpp"
#include "aerial_touch/depth_sampler.hpp"
#include "aerial_touch/fingertip_probe.hpp"
#include "aerial_touch/keypad.hpp"
#include "aerial_touch/hand_tracker.hpp"
#include "aerial_touch/plane.hpp"
#include "aerial_touch/keypad_overlay.hpp"
#include "aerial_touch/rgbd_frame.hpp"
#include "aerial_touch/settings_window.hpp"
#include "aerial_touch/surface_plane.hpp"
#include "aerial_touch/surface_scan.hpp"
#include "aerial_touch/touch_state_machine.hpp"

#include <cmath>
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

namespace {

bool approximately_equal(const float actual, const float expected) {
    return std::fabs(actual - expected) < 0.001F;
}

bool plane_projection_uses_camera_facing_normal() {
    const auto plane = aerial_touch::Plane::from_calibration_points(
        { 0.0F, 0.0F, 1000.0F },
        { 100.0F, 0.0F, 1000.0F },
        { 0.0F, 100.0F, 1000.0F });
    if(!plane.has_value()) {
        return false;
    }

    const auto projected = plane->project({ 30.0F, 40.0F, 990.0F });
    return approximately_equal(projected.u_mm, 30.0F) && approximately_equal(projected.v_mm, 40.0F)
           && approximately_equal(projected.signed_distance_mm, 10.0F);
}

bool plane_uses_configured_minimum_point_distance() {
    const auto plane = aerial_touch::Plane::from_calibration_points(
        { 0.0F, 0.0F, 1000.0F },
        { 50.0F, 0.0F, 1000.0F },
        { 0.0F, 50.0F, 1000.0F },
        40.0F);
    return plane.has_value();
}

bool plane_rejects_nearly_collinear_points() {
    const auto plane = aerial_touch::Plane::from_calibration_points(
        { 0.0F, 0.0F, 1000.0F },
        { 100.0F, 0.0F, 1000.0F },
        { 100.0F, 1.0F, 1000.0F });
    return !plane.has_value();
}

bool surface_plane_fit_removes_fingertip_height_offsets() {
    std::vector<aerial_touch::Vec3> samples;
    for(int x = 0; x <= 120; x += 20) {
        for(int y = 0; y <= 80; y += 20) {
            samples.push_back({ static_cast<float>(x), static_cast<float>(y), 1000.0F });
        }
    }
    samples.push_back({ 40.0F, 40.0F, 920.0F });
    samples.push_back({ 80.0F, 40.0F, 1080.0F });

    aerial_touch::SurfacePlaneFitConfig config;
    config.minimum_samples = 20U;
    config.inlier_threshold_mm = 3.0F;
    config.maximum_rms_residual_mm = 1.0F;
    aerial_touch::SurfacePlaneFitQuality quality;
    const auto plane = aerial_touch::SurfacePlane::fit(samples, config, &quality);
    if(!plane.has_value() || quality.inlier_samples != 35U || quality.total_samples != 37U) {
        return false;
    }

    const auto projected = plane->project_to_surface({ 60.0F, 40.0F, 965.0F });
    const auto hit = plane->intersect_ray({ 60.0F, 40.0F, 900.0F }, { 0.0F, 0.0F, 1.0F });
    return projected.has_value() && hit.has_value() && approximately_equal(projected->x, 60.0F)
           && approximately_equal(projected->y, 40.0F) && approximately_equal(projected->z, 1000.0F)
           && approximately_equal(hit->x, 60.0F) && approximately_equal(hit->y, 40.0F)
           && approximately_equal(hit->z, 1000.0F);
}

bool surface_projection_keeps_keypad_geometry_with_varying_fingertip_heights() {
    std::vector<aerial_touch::Vec3> samples;
    for(int x = 0; x <= 240; x += 40) {
        for(int y = 0; y <= 320; y += 40) {
            samples.push_back({ static_cast<float>(x), static_cast<float>(y), 1000.0F });
        }
    }
    aerial_touch::SurfacePlaneFitConfig config;
    config.minimum_samples = 30U;
    config.inlier_threshold_mm = 2.0F;
    config.maximum_rms_residual_mm = 0.5F;
    const auto surface = aerial_touch::SurfacePlane::fit(samples, config);
    if(!surface.has_value()) {
        return false;
    }

    // Deliberately varied z: the point of ray-casting onto the scanned surface is that the
    // fingertip's own height must not leak into the keypad geometry.
    const std::array<aerial_touch::Vec3, aerial_touch::kKeypadCalibrationPointCount> fingertips{
        aerial_touch::Vec3{ 0.0F, 0.0F, 982.0F },
        aerial_touch::Vec3{ 240.0F, 0.0F, 1017.0F },
        aerial_touch::Vec3{ 120.0F, 320.0F, 989.0F },
    };
    std::array<aerial_touch::Vec3, aerial_touch::kKeypadCalibrationPointCount> points{};
    for(std::size_t index = 0; index < fingertips.size(); ++index) {
        const auto projected = surface->project_to_surface(fingertips[index]);
        if(!projected.has_value()) {
            return false;
        }
        points[index] = *projected;
    }
    const auto result = aerial_touch::calibrate_keypad(points, 80.0F);
    return result.has_value() && approximately_equal(result->geometry.total_width_mm, 240.0F)
           && approximately_equal(result->geometry.total_height_mm, 320.0F)
           && approximately_equal(result->geometry.key_width_mm, 80.0F)
           && approximately_equal(result->geometry.key_height_mm, 80.0F);
}

bool surface_plane_fit_rejects_competing_depth_planes() {
    std::vector<aerial_touch::Vec3> samples;
    for(int x = 0; x < 8; ++x) {
        for(int y = 0; y < 8; ++y) {
            samples.push_back({ static_cast<float>(x * 20), static_cast<float>(y * 20), 1000.0F });
            samples.push_back({ static_cast<float>(x * 20), static_cast<float>(y * 20), 1040.0F });
        }
    }
    aerial_touch::SurfacePlaneFitConfig config;
    config.minimum_samples = 60U;
    config.minimum_inlier_ratio = 0.70F;
    config.inlier_threshold_mm = 3.0F;
    return !aerial_touch::SurfacePlane::fit(samples, config).has_value();
}

bool surface_plane_fit_supports_rotated_target_and_ray_intersection() {
    const aerial_touch::Vec3 origin{ 100.0F, 50.0F, 900.0F };
    const aerial_touch::Vec3 u_axis{ 0.8660254F, 0.0F, 0.5F };
    const aerial_touch::Vec3 v_axis{ -0.25F, 0.8660254F, 0.4330127F };
    const aerial_touch::Vec3 normal{ -0.4330127F, -0.5F, 0.75F };
    const auto point = [&](const float u, const float v) {
        return aerial_touch::Vec3{ origin.x + u_axis.x * u + v_axis.x * v,
                                   origin.y + u_axis.y * u + v_axis.y * v,
                                   origin.z + u_axis.z * u + v_axis.z * v };
    };

    std::vector<aerial_touch::Vec3> samples;
    for(int u = 0; u <= 240; u += 40) {
        for(int v = 0; v <= 320; v += 40) {
            samples.push_back(point(static_cast<float>(u), static_cast<float>(v)));
        }
    }
    samples.push_back({ 10.0F, 20.0F, 400.0F });
    samples.push_back({ 600.0F, 300.0F, 1200.0F });
    aerial_touch::SurfacePlaneFitConfig config;
    config.minimum_samples = 30U;
    config.inlier_threshold_mm = 2.0F;
    config.maximum_rms_residual_mm = 0.5F;
    const auto surface = aerial_touch::SurfacePlane::fit(samples, config);
    const auto expected = point(120.0F, 160.0F);
    const aerial_touch::Vec3 ray_origin{ expected.x - normal.x * 200.0F, expected.y - normal.y * 200.0F,
                                          expected.z - normal.z * 200.0F };
    const auto hit = surface.has_value() ? surface->intersect_ray(ray_origin, normal) : std::nullopt;
    return hit.has_value() && approximately_equal(hit->x, expected.x) && approximately_equal(hit->y, expected.y)
           && approximately_equal(hit->z, expected.z);
}

bool surface_scan_completes_only_after_minimum_duration_and_valid_plane() {
    std::vector<aerial_touch::Vec3> samples;
    for(int x = 0; x < 10; ++x) {
        for(int y = 0; y < 10; ++y) {
            samples.push_back({ static_cast<float>(x * 20), static_cast<float>(y * 20), 1000.0F });
        }
    }
    aerial_touch::SurfaceScanConfig config;
    config.minimum_duration_ms = 100;
    config.timeout_ms = 1000;
    config.plane_fit.minimum_samples = 60U;
    config.plane_fit.inlier_threshold_mm = 2.0F;
    config.plane_fit.maximum_rms_residual_mm = 0.5F;
    aerial_touch::SurfaceScanCollector scan(config);
    scan.begin(0);
    if(scan.add(samples, 99).state != aerial_touch::SurfaceScanState::Collecting) {
        return false;
    }
    const auto progress = scan.add(samples, 100);
    return progress.state == aerial_touch::SurfaceScanState::Complete && progress.quality.inlier_samples >= 60U
           && scan.surface_plane().has_value();
}

bool surface_scan_fails_after_timeout_without_a_valid_plane() {
    aerial_touch::SurfaceScanConfig config;
    config.minimum_duration_ms = 100;
    config.timeout_ms = 200;
    config.plane_fit.minimum_samples = 10U;
    aerial_touch::SurfaceScanCollector scan(config);
    scan.begin(0);
    return scan.add({}, 201).state == aerial_touch::SurfaceScanState::Failed && !scan.surface_plane().has_value();
}

bool surface_scan_throttles_failed_plane_fit_attempts() {
    std::vector<aerial_touch::Vec3> samples;
    for(int x = 0; x < 10; ++x) {
        for(int y = 0; y < 10; ++y) {
            samples.push_back({ static_cast<float>(x * 20), static_cast<float>(y * 20), 1000.0F });
        }
    }
    aerial_touch::SurfaceScanConfig config;
    config.minimum_duration_ms = 100;
    config.timeout_ms = 1000;
    config.fit_interval_ms = 100;
    config.plane_fit.minimum_samples = 60U;
    config.plane_fit.inlier_threshold_mm = 2.0F;
    config.plane_fit.maximum_rms_residual_mm = 0.5F;
    aerial_touch::SurfaceScanCollector scan(config);
    scan.begin(0);
    if(scan.add({}, 100).state != aerial_touch::SurfaceScanState::Collecting) {
        return false;
    }
    if(scan.add(samples, 150).state != aerial_touch::SurfaceScanState::Collecting) {
        return false;
    }
    return scan.add(samples, 200).state == aerial_touch::SurfaceScanState::Complete;
}

bool surface_scan_retries_after_camera_timestamp_moves_backward() {
    std::vector<aerial_touch::Vec3> samples;
    for(int x = 0; x < 10; ++x) {
        for(int y = 0; y < 10; ++y) {
            samples.push_back({ static_cast<float>(x * 20), static_cast<float>(y * 20), 1000.0F });
        }
    }
    aerial_touch::SurfaceScanConfig config;
    config.minimum_duration_ms = 100;
    config.timeout_ms = 1000;
    config.fit_interval_ms = 100;
    config.plane_fit.minimum_samples = 60U;
    config.plane_fit.inlier_threshold_mm = 2.0F;
    config.plane_fit.maximum_rms_residual_mm = 0.5F;
    aerial_touch::SurfaceScanCollector scan(config);
    scan.begin(0);
    if(scan.add({}, 200).state != aerial_touch::SurfaceScanState::Collecting) {
        return false;
    }
    return scan.add(samples, 150).state == aerial_touch::SurfaceScanState::Complete;
}

std::array<aerial_touch::Vec3, aerial_touch::kKeypadCalibrationPointCount>
rectangular_keypad_calibration_points() {
    return {
        aerial_touch::Vec3{ 0.0F, 0.0F, 1000.0F },      // top-left of key "1"
        aerial_touch::Vec3{ 240.0F, 0.0F, 1000.0F },    // top-right of key "3"
        aerial_touch::Vec3{ 120.0F, 320.0F, 1000.0F },  // bottom edge, below key "0"
    };
}

bool keypad_calibration_derives_contiguous_cells() {
    const auto result = aerial_touch::calibrate_keypad(rectangular_keypad_calibration_points(), 80.0F);
    if(!result.has_value()) {
        return false;
    }
    const aerial_touch::Keypad keypad(result->geometry);
    return approximately_equal(result->geometry.total_width_mm, 240.0F)
           && approximately_equal(result->geometry.total_height_mm, 320.0F)
           && approximately_equal(result->geometry.key_width_mm, 80.0F)
           && approximately_equal(result->geometry.key_height_mm, 80.0F)
           && approximately_equal(result->geometry.horizontal_gap_mm, 0.0F)
           && approximately_equal(result->geometry.vertical_gap_mm, 0.0F)
           && keypad.key_at({ 40.0F, 40.0F }).value_or("?") == "1"
           && keypad.key_at({ 120.0F, 40.0F }).value_or("?") == "2"
           && keypad.key_at({ 200.0F, 40.0F }).value_or("?") == "3"
           && keypad.key_at({ 120.0F, 280.0F }).value_or("?") == "0";
}

// The reason for dropping the four extra points: with contiguous cells every point inside the top
// three rows belongs to some key, so a few millimetres of pointing error can no longer land in a
// dead band and silently do nothing.
bool keypad_calibration_leaves_no_dead_band_between_keys() {
    const auto result = aerial_touch::calibrate_keypad(rectangular_keypad_calibration_points(), 80.0F);
    if(!result.has_value()) {
        return false;
    }
    const aerial_touch::Keypad keypad(result->geometry);
    for(int u = 1; u < 240; ++u) {
        for(const float v : { 40.0F, 120.0F, 200.0F }) {
            if(!keypad.key_at({ static_cast<float>(u), v }).has_value()) {
                return false;
            }
        }
    }
    return true;
}

bool keypad_calibration_supports_rotated_3d_plane() {
    const aerial_touch::Vec3 origin{ 100.0F, 100.0F, 1000.0F };
    const aerial_touch::Vec3 u_axis{ 0.8660254F, 0.0F, 0.5F };
    const aerial_touch::Vec3 v_axis{ -0.25F, 0.8660254F, 0.4330127F };
    const auto point = [&](const float u, const float v) {
        return aerial_touch::Vec3{ origin.x + u_axis.x * u + v_axis.x * v,
                                   origin.y + u_axis.y * u + v_axis.y * v,
                                   origin.z + u_axis.z * u + v_axis.z * v };
    };
    const std::array<aerial_touch::Vec3, aerial_touch::kKeypadCalibrationPointCount> points{
        point(0.0F, 0.0F), point(240.0F, 0.0F), point(120.0F, 320.0F),
    };
    const auto result = aerial_touch::calibrate_keypad(points, 80.0F);
    return result.has_value() && approximately_equal(result->geometry.key_width_mm, 80.0F)
           && approximately_equal(result->geometry.key_height_mm, 80.0F)
           && approximately_equal(result->geometry.horizontal_gap_mm, 0.0F)
           && approximately_equal(result->geometry.vertical_gap_mm, 0.0F);
}

// The bottom point is aimed below key "0" by eye. Its lateral offset is discarded by the plane's
// Gram-Schmidt step, so the pad height must survive a sloppy sideways aim.
bool keypad_calibration_ignores_lateral_error_in_bottom_point() {
    auto points = rectangular_keypad_calibration_points();
    points[aerial_touch::kKeypadBottomPoint] = { 155.0F, 320.0F, 1000.0F };

    const auto result = aerial_touch::calibrate_keypad(points, 80.0F);
    return result.has_value() && approximately_equal(result->geometry.total_height_mm, 320.0F)
           && approximately_equal(result->geometry.key_height_mm, 80.0F);
}

bool keypad_calibration_rejects_invalid_geometry() {
    // Top edge shorter than the minimum calibration span.
    auto points = rectangular_keypad_calibration_points();
    points[aerial_touch::kKeypadTopRightPoint] = { 50.0F, 0.0F, 1000.0F };
    if(aerial_touch::calibrate_keypad(points, 80.0F).has_value()) {
        return false;
    }
    // Bottom point nearly collinear with the top edge: no usable v axis.
    points = rectangular_keypad_calibration_points();
    points[aerial_touch::kKeypadBottomPoint] = { 120.0F, 5.0F, 1000.0F };
    if(aerial_touch::calibrate_keypad(points, 80.0F).has_value()) {
        return false;
    }
    // Bottom point aimed far outside the pad's horizontal span.
    points = rectangular_keypad_calibration_points();
    points[aerial_touch::kKeypadBottomPoint] = { 340.0F, 320.0F, 1000.0F };
    return !aerial_touch::calibrate_keypad(points, 80.0F).has_value();
}

bool keypad_calibration_reports_bottom_point_outside_keypad() {
    auto points = rectangular_keypad_calibration_points();
    points[aerial_touch::kKeypadBottomPoint] = { 340.0F, 320.0F, 1000.0F };
    const auto attempt = aerial_touch::calibrate_keypad_detailed(points, 80.0F);
    return !attempt.result.has_value()
           && attempt.failure == aerial_touch::KeypadCalibrationFailure::BottomPointOutsideKeypad;
}

bool keypad_maps_uv_to_expected_number() {
    const aerial_touch::Keypad keypad({ 100.0F, 135.0F, 30.0F, 30.0F, 5.0F, 5.0F });
    return keypad.key_at({ 15.0F, 15.0F }).value_or("?") == "1"
           && keypad.key_at({ 50.0F, 15.0F }).value_or("?") == "2"
           && !keypad.key_at({ 105.0F, 15.0F }).has_value();
}

bool touch_requires_release_before_repeat_press() {
    aerial_touch::TouchStateMachine touch({ 10.0F, 20.0F, 0.0F, 300 });

    const auto far      = touch.update({ 0, 50.0F, "5" });
    const auto approach = touch.update({ 10, 25.0F, "5" });
    const auto press    = touch.update({ 20, 10.0F, "5" });
    const auto hold     = touch.update({ 30, 5.0F, "5" });
    const auto release  = touch.update({ 40, 21.0F, "5" });
    const auto press2   = touch.update({ 50, 10.0F, "5" });

    return !far.has_value() && !approach.has_value() && press.has_value() && press->key == "5" && !hold.has_value()
           && !release.has_value() && press2.has_value() && press2->key == "5";
}

bool touch_does_not_press_outside_keypad() {
    aerial_touch::TouchStateMachine touch({ 10.0F, 20.0F, 0.0F, 300 });

    touch.update({ 0, 30.0F, std::nullopt });
    const auto outside = touch.update({ 10, 5.0F, std::nullopt });
    return !outside.has_value() && touch.armed();
}

bool touch_uses_elapsed_time_for_approach_velocity() {
    aerial_touch::TouchStateMachine touch({ 10.0F, 20.0F, 100.0F, 300 });

    touch.update({ 0, 30.0F, "5" });
    touch.update({ 100, 21.0F, "5" });
    const auto press = touch.update({ 200, 10.0F, "5" });
    return press.has_value() && press->key == "5";
}

bool keypad_overlay_prioritizes_pressed_key() {
    const std::optional<std::string> hovered_key{ "5" };
    const std::optional<std::string> pressed_key{ "5" };
    using aerial_touch::KeypadKeyVisualState;
    return aerial_touch::keypad_key_visual_state("5", hovered_key, pressed_key) == KeypadKeyVisualState::Pressed
           && aerial_touch::keypad_key_visual_state("5", hovered_key, std::nullopt) == KeypadKeyVisualState::Hover
           && aerial_touch::keypad_key_visual_state("4", hovered_key, pressed_key) == KeypadKeyVisualState::Idle;
}

bool keypad_overlay_clears_pressed_key_when_not_currently_held() {
    const std::optional<std::string> last_pressed_key{ "5" };
    return aerial_touch::currently_pressed_key(last_pressed_key, false, true, false, last_pressed_key)
               == last_pressed_key
           && !aerial_touch::currently_pressed_key(last_pressed_key, true, true, false, last_pressed_key).has_value()
           && !aerial_touch::currently_pressed_key(last_pressed_key, false, false, false, last_pressed_key).has_value()
           && !aerial_touch::currently_pressed_key(last_pressed_key, false, true, true, last_pressed_key).has_value()
           && !aerial_touch::currently_pressed_key(last_pressed_key, false, true, false,
                                                   std::optional<std::string>{ "6" }).has_value();
}

bool fixed_keypad_overlay_layout_has_default_size() {
    const auto layout = aerial_touch::fixed_keypad_overlay_layout();
    return layout.width_px == 100 && layout.height_px == 135 && layout.margin_px == 12
           && layout.regions.front().key == "1" && layout.regions.front().x_px == 0
           && layout.regions.front().y_px == 0 && layout.regions.front().width_px == 30
           && layout.regions.front().height_px == 30 && layout.regions.back().key == "0"
           && layout.regions.back().x_px == 35 && layout.regions.back().y_px == 105;
}

bool touch_config_update_preserves_armed_state() {
    aerial_touch::TouchStateMachine touch({ 10.0F, 20.0F, 0.0F, 300 });
    touch.update({ 0, 30.0F, "5" });
    touch.set_config({ 15.0F, 25.0F, 0.0F, 300 });
    const auto press = touch.update({ 10, 10.0F, "5" });
    return press.has_value() && press->key == "5";
}

bool tracking_loss_requires_release_before_pressing_again() {
    aerial_touch::TouchStateMachine touch({ 10.0F, 20.0F, 0.0F, 300 });

    touch.update({ 0, 30.0F, "5" });
    touch.mark_tracking_lost(301);
    const auto blocked = touch.update({ 310, 5.0F, "5" });
    touch.update({ 320, 25.0F, "5" });
    const auto press = touch.update({ 330, 10.0F, "5" });
    return !blocked.has_value() && press.has_value();
}

bool press_event_keeps_fingertip_and_plane_coordinates() {
    aerial_touch::TouchStateMachine touch({ 10.0F, 20.0F, 0.0F, 300 });
    touch.update({ 0, 30.0F, "7", { 1.0F, 2.0F, 3.0F }, { 4.0F, 5.0F } });
    const auto press = touch.update({ 10, 10.0F, "7", { 10.0F, 20.0F, 30.0F }, { 40.0F, 50.0F } });
    return press.has_value() && approximately_equal(press->fingertip_xyz_mm.x, 10.0F)
           && approximately_equal(press->fingertip_xyz_mm.y, 20.0F)
           && approximately_equal(press->fingertip_xyz_mm.z, 30.0F)
           && approximately_equal(press->plane_uv_mm.x, 40.0F) && approximately_equal(press->plane_uv_mm.y, 50.0F);
}

bool missing_hand_tracker_dll_is_safe() {
    aerial_touch::HandTracker tracker("definitely-missing-hand-bridge.dll", "missing-model.task");
    const unsigned char pixel[3]{ 0, 0, 0 };
    return !tracker.available() && !tracker.detect_rgb(pixel, 1, 1, 3, 0).detected && !tracker.error().empty();
}

bool yaml_config_loads_all_runtime_thresholds() {
    const auto config = aerial_touch::load_app_config(
        std::filesystem::path(TEST_SOURCE_DIR) / "tests" / "data" / "config.yaml");
    return config.camera.depth_work_mode == "Near"
           && config.camera.depth_precision == "0.8mm"
           && config.camera.preferred_fps == 30
           && config.camera.sdk_temporal_filter && config.camera.sdk_spatial_filter
           && !config.camera.hole_filling_filter && config.camera.rgb_power_line_frequency_hz == 60
           && config.depth.sample_radius == 3 && config.depth.median_window_size == 5U
           && approximately_equal(config.depth.max_jump_mm, 80.0F)
           && config.depth.invalid_reset_frames == 3U
           && approximately_equal(config.depth.finger_clearance_mm, 7.5F)
           && approximately_equal(config.fingertip.min_cutoff_hz, 1.0F)
           && approximately_equal(config.fingertip.beta, 0.12F)
           && approximately_equal(config.fingertip.derivative_cutoff_hz, 1.0F)
           && config.fingertip.display_hold_ms == 100
           && approximately_equal(config.fingertip.depth_probe_near_ratio, 0.30F)
           && approximately_equal(config.fingertip.depth_probe_far_ratio, 0.65F)
           && approximately_equal(config.touch.touch_threshold_mm, 9.0F)
           && approximately_equal(config.touch.release_threshold_mm, 22.0F)
           && approximately_equal(config.touch.min_approach_velocity_mm_s, 45.0F)
           && config.touch.tracking_timeout_ms == 275 && config.touch.dwell_ms == 420
           && approximately_equal(config.keypad.boundary_hysteresis_mm, 2.0F)
           && approximately_equal(config.calibration.minimum_point_distance_mm, 85.0F)
           && config.calibration.required_samples == 18U
           && approximately_equal(config.calibration.mad_multiplier, 3.5F)
           && approximately_equal(config.calibration.minimum_outlier_threshold_mm, 2.0F);
}

bool yaml_config_round_trips_through_save() {
    const auto source = aerial_touch::load_app_config(
        std::filesystem::path(TEST_SOURCE_DIR) / "tests" / "data" / "config.yaml");
    const auto path = std::filesystem::temp_directory_path() / "aerial_touch_config_roundtrip.yaml";
    aerial_touch::save_app_config(source, path);
    std::ifstream saved(path);
    const std::string saved_text((std::istreambuf_iterator<char>(saved)), std::istreambuf_iterator<char>());
    saved.close();
    const auto restored = aerial_touch::load_app_config(path);
    std::filesystem::remove(path);
    return saved_text.find("key_width_mm") == std::string::npos
           && saved_text.find("key_height_mm") == std::string::npos
           && saved_text.find("horizontal_gap_mm") == std::string::npos
           && saved_text.find("vertical_gap_mm") == std::string::npos
           && restored.camera.depth_work_mode == source.camera.depth_work_mode
           && restored.camera.depth_precision == source.camera.depth_precision
           && restored.camera.preferred_fps == source.camera.preferred_fps
           && restored.camera.sdk_temporal_filter == source.camera.sdk_temporal_filter
           && restored.camera.sdk_spatial_filter == source.camera.sdk_spatial_filter
           && restored.camera.hole_filling_filter == source.camera.hole_filling_filter
           && restored.camera.rgb_power_line_frequency_hz == source.camera.rgb_power_line_frequency_hz
           && restored.depth.sample_radius == source.depth.sample_radius
           && restored.depth.median_window_size == source.depth.median_window_size
           && approximately_equal(restored.depth.max_jump_mm, source.depth.max_jump_mm)
           && restored.depth.invalid_reset_frames == source.depth.invalid_reset_frames
           && approximately_equal(restored.depth.finger_clearance_mm, source.depth.finger_clearance_mm)
           && approximately_equal(restored.fingertip.min_cutoff_hz, source.fingertip.min_cutoff_hz)
           && approximately_equal(restored.fingertip.beta, source.fingertip.beta)
           && approximately_equal(restored.fingertip.derivative_cutoff_hz, source.fingertip.derivative_cutoff_hz)
           && restored.fingertip.display_hold_ms == source.fingertip.display_hold_ms
           && approximately_equal(restored.fingertip.depth_probe_near_ratio,
                                  source.fingertip.depth_probe_near_ratio)
           && approximately_equal(restored.fingertip.depth_probe_far_ratio,
                                  source.fingertip.depth_probe_far_ratio)
           && approximately_equal(restored.touch.touch_threshold_mm, source.touch.touch_threshold_mm)
           && approximately_equal(restored.touch.release_threshold_mm, source.touch.release_threshold_mm)
           && approximately_equal(restored.touch.min_approach_velocity_mm_s, source.touch.min_approach_velocity_mm_s)
           && restored.touch.tracking_timeout_ms == source.touch.tracking_timeout_ms
           && restored.touch.dwell_ms == source.touch.dwell_ms
           && approximately_equal(restored.keypad.boundary_hysteresis_mm, source.keypad.boundary_hysteresis_mm)
           && approximately_equal(restored.calibration.minimum_point_distance_mm,
                                  source.calibration.minimum_point_distance_mm)
           && restored.calibration.required_samples == source.calibration.required_samples
           && approximately_equal(restored.calibration.mad_multiplier, source.calibration.mad_multiplier)
           && approximately_equal(restored.calibration.minimum_outlier_threshold_mm,
                                  source.calibration.minimum_outlier_threshold_mm);
}

bool short_tracking_loss_cannot_trigger_on_reacquisition() {
    aerial_touch::TouchStateMachine touch({ 10.0F, 20.0F, 0.0F, 300 });
    touch.update({ 0, 30.0F, "5" });
    touch.update({ 10, 15.0F, "5" });
    touch.mark_tracking_lost(20);
    const auto ghost = touch.update({ 30, 5.0F, "5" });
    const auto press = touch.update({ 40, 4.0F, "5" });
    return !ghost.has_value() && press.has_value();
}

bool unsupported_camera_settings_are_not_selected() {
    const std::vector<std::string> work_modes{ "Default", "Far" };
    const std::vector<std::string> precisions{ "1mm", "0.8mm" };
    const std::vector<int> frequencies{ 50 };
    return !aerial_touch::select_supported_setting(std::string("Near"), work_modes).has_value()
           && aerial_touch::select_supported_setting(std::string("Far"), work_modes).value_or("") == "Far"
           && !aerial_touch::select_supported_setting(std::string("0.4mm"), precisions).has_value()
           && !aerial_touch::select_supported_setting(60, frequencies).has_value()
           && aerial_touch::select_supported_setting(50, frequencies).value_or(0) == 50;
}

bool camera_fallback_and_filter_order_are_explicit() {
    const auto fps = aerial_touch::fps_fallback_order(60);
    aerial_touch::CameraConfig config;
    config.sdk_temporal_filter = true;
    config.sdk_spatial_filter = true;
    config.hole_filling_filter = true;
    const auto stages = aerial_touch::depth_filter_plan(config, true, true, true);
    return fps == std::vector<int>({ 60, 30, 0 })
           && stages == std::vector<aerial_touch::DepthFilterKind>({
               aerial_touch::DepthFilterKind::Temporal,
               aerial_touch::DepthFilterKind::Spatial,
               aerial_touch::DepthFilterKind::HoleFilling,
           });
}

bool camera_profile_resolver_requires_matching_rgb_and_depth_fps() {
    const std::vector<aerial_touch::CameraProfileOption> options{
        { 0U, 60, 30, true },
        { 1U, 30, 30, true },
        { 2U, 60, 60, false },
    };
    return aerial_touch::select_camera_profile_option(60, options).value_or(99U) == 1U;
}

bool camera_capabilities_only_offer_usable_matching_fps() {
    const std::vector<aerial_touch::CameraProfileOption> options{
        { 0U, 60, 30, true },
        { 1U, 30, 30, true },
        { 2U, 60, 60, false },
        { 3U, 15, 15, true },
        { 4U, 30, 30, true },
    };
    return aerial_touch::supported_camera_fps(options) == std::vector<int>({ 15, 30 });
}

bool failed_depth_filter_is_skipped_without_losing_the_frame() {
    const auto stages = std::vector<aerial_touch::DepthFilterKind>{
        aerial_touch::DepthFilterKind::Temporal,
        aerial_touch::DepthFilterKind::Spatial,
    };
    const auto input = std::make_shared<int>(10);
    const auto result = aerial_touch::process_resilient_filter_chain(
        input, stages, [](const aerial_touch::DepthFilterKind stage, const std::shared_ptr<int>& frame) {
            if(stage == aerial_touch::DepthFilterKind::Temporal) {
                throw std::runtime_error("unsupported");
            }
            return std::make_shared<int>(*frame + 5);
        });
    return result.frame && *result.frame == 15
           && result.failed_stages == std::vector<aerial_touch::DepthFilterKind>{
               aerial_touch::DepthFilterKind::Temporal,
           };
}

bool yaml_config_rejects_nonfinite_values() {
    const auto path = std::filesystem::temp_directory_path() / "aerial_touch_config_nonfinite.yaml";
    std::ofstream output(path);
    output << "depth:\n"
              "  sample_radius: 2\n"
              "touch:\n"
              "  touch_threshold_mm: .nan\n"
              "  release_threshold_mm: 20.0\n"
              "  min_approach_velocity_mm_s: 0.0\n"
              "  tracking_timeout_ms: 300\n"
              "keypad:\n"
              "  key_width_mm: 30.0\n"
              "  key_height_mm: 30.0\n"
              "  horizontal_gap_mm: 5.0\n"
              "  vertical_gap_mm: 5.0\n"
              "calibration:\n"
              "  minimum_point_distance_mm: 80.0\n";
    output.close();
    bool rejected = false;
    try {
        static_cast<void>(aerial_touch::load_app_config(path));
    }
    catch(const std::exception&) {
        rejected = true;
    }
    std::filesystem::remove(path);
    return rejected;
}

bool legacy_yaml_uses_defaults_for_new_stabilization_fields() {
    const auto path = std::filesystem::temp_directory_path() / "aerial_touch_legacy_config.yaml";
    std::ofstream output(path);
    output << "depth:\n"
              "  sample_radius: 2\n"
              "touch:\n"
              "  touch_threshold_mm: 10.0\n"
              "  release_threshold_mm: 20.0\n"
              "  min_approach_velocity_mm_s: 0.0\n"
              "  tracking_timeout_ms: 300\n"
              "keypad:\n"
              "  key_width_mm: 30.0\n"
              "  key_height_mm: 30.0\n"
              "  horizontal_gap_mm: 5.0\n"
              "  vertical_gap_mm: 5.0\n"
              "calibration:\n"
              "  minimum_point_distance_mm: 80.0\n";
    output.close();
    bool uses_defaults = false;
    try {
        const auto config = aerial_touch::load_app_config(path);
        uses_defaults = config.camera.preferred_fps == 30 && config.camera.sdk_temporal_filter
                        && config.depth.median_window_size == 5U
                        && approximately_equal(config.fingertip.beta, 0.12F)
                        && approximately_equal(config.keypad.boundary_hysteresis_mm, 2.0F)
                        && config.calibration.required_samples == 18U
                        && config.touch.dwell_ms == 350
                        && approximately_equal(config.depth.finger_clearance_mm, 3.0F)
                        && approximately_equal(config.fingertip.depth_probe_near_ratio, 0.35F);
    }
    catch(const std::exception&) {
    }
    std::filesystem::remove(path);
    return uses_defaults;
}

bool app_config_validation_rejects_invalid_values() {
    auto config = aerial_touch::AppConfig{};
    config.touch.release_threshold_mm = config.touch.touch_threshold_mm;
    bool rejects_equal_thresholds = false;
    try {
        aerial_touch::validate_app_config(config);
    }
    catch(const std::exception&) {
        rejects_equal_thresholds = true;
    }

    config = aerial_touch::AppConfig{};
    config.keypad.boundary_hysteresis_mm = -1.0F;
    bool rejects_negative_gap = false;
    try {
        aerial_touch::validate_app_config(config);
    }
    catch(const std::exception&) {
        rejects_negative_gap = true;
    }
    config = aerial_touch::AppConfig{};
    config.calibration.required_samples = 14U;
    bool rejects_too_few_samples = false;
    try {
        aerial_touch::validate_app_config(config);
    }
    catch(const std::exception&) {
        rejects_too_few_samples = true;
    }

    return rejects_equal_thresholds && rejects_negative_gap && rejects_too_few_samples
           && approximately_equal(aerial_touch::AppConfig{}.touch.min_approach_velocity_mm_s, 40.0F);
}

bool preview_classifies_touch_zones() {
    using aerial_touch::PreviewZone;
    return aerial_touch::classify_preview_zone(std::nullopt, 10.0F, 20.0F) == PreviewZone::Unknown
           && aerial_touch::classify_preview_zone(5.0F, 10.0F, 20.0F) == PreviewZone::Touch
           && aerial_touch::classify_preview_zone(15.0F, 10.0F, 20.0F) == PreviewZone::Hold
           && aerial_touch::classify_preview_zone(20.0F, 10.0F, 20.0F) == PreviewZone::Release;
}

bool settings_layout_adapts_to_window_size() {
    const auto compact = aerial_touch::calculate_settings_layout(920, 780);
    const auto expanded = aerial_touch::calculate_settings_layout(1200, 960);
    return expanded.preview_group.right > compact.preview_group.right
           && expanded.preview_group.bottom > compact.preview_group.bottom
           && expanded.preview_rect.right > compact.preview_rect.right
           && expanded.buttons_y > compact.buttons_y
           && expanded.touch_group.right > compact.touch_group.right;
}

bool rgbd_frame_validates_alignment_and_buffer_sizes() {
    aerial_touch::RgbdFrame frame;
    frame.color_width = 2;
    frame.color_height = 2;
    frame.depth_width = 2;
    frame.depth_height = 2;
    frame.rgb.assign(12, 0U);
    frame.depth.assign(4, 1000U);
    frame.depth_unit_mm = 1.0F;
    frame.profiles_valid = true;
    if(frame.valid()) {
        return false;
    }
    frame.raw_depth.assign(3, 1000U);
    if(frame.valid()) {
        return false;
    }
    frame.raw_depth.assign(4, 1000U);
    if(!frame.valid()) {
        return false;
    }
    frame.depth_width = 1;
    return !frame.valid();
}

bool unavailable_hardware_d2c_uses_software_alignment() {
    return aerial_touch::choose_alignment_mode(false) == aerial_touch::AlignmentMode::Software;
}

}  // namespace

// --- camera model -----------------------------------------------------------------------------

aerial_touch::CameraIntrinsics test_intrinsics() {
    return { 520.0F, 519.0F, 319.5F, 239.5F, 640, 480 };
}

aerial_touch::CameraExtrinsics identity_extrinsics() {
    return { { 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F }, { 0.0F, 0.0F, 0.0F } };
}

// The on-table keypad overlay is only trustworthy if projecting is the exact inverse of
// deprojecting -- otherwise the drawn keys would sit somewhere the touch maths never looks.
bool camera_projection_round_trips_pixels() {
    const auto intrinsics = test_intrinsics();
    const auto extrinsics = identity_extrinsics();
    for(const auto pixel : { aerial_touch::Vec2{ 0.0F, 0.0F }, aerial_touch::Vec2{ 319.5F, 239.5F },
                             aerial_touch::Vec2{ 639.0F, 479.0F }, aerial_touch::Vec2{ 123.25F, 400.75F } }) {
        for(const float depth : { 400.0F, 812.5F, 1600.0F }) {
            const auto point = aerial_touch::deproject_pixel(intrinsics, extrinsics, pixel, depth);
            if(!point.has_value()) {
                return false;
            }
            const auto back = aerial_touch::project_point(intrinsics, extrinsics, *point);
            if(!back.has_value() || std::fabs(back->x - pixel.x) > 0.01F
               || std::fabs(back->y - pixel.y) > 0.01F) {
                return false;
            }
        }
    }
    return true;
}

bool camera_projection_round_trips_through_a_rotated_extrinsic() {
    const auto intrinsics = test_intrinsics();
    // 90 degrees about z, plus a translation, so a wrong transpose cannot pass by accident.
    const aerial_touch::CameraExtrinsics extrinsics{
        { 0.0F, -1.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F }, { 12.0F, -8.0F, 5.0F }
    };
    const aerial_touch::Vec2 pixel{ 200.0F, 300.0F };
    const auto point = aerial_touch::deproject_pixel(intrinsics, extrinsics, pixel, 900.0F);
    if(!point.has_value()) {
        return false;
    }
    const auto back = aerial_touch::project_point(intrinsics, extrinsics, *point);
    return back.has_value() && std::fabs(back->x - pixel.x) < 0.01F && std::fabs(back->y - pixel.y) < 0.01F;
}

bool camera_projection_rejects_points_behind_the_camera() {
    const auto intrinsics = test_intrinsics();
    const auto extrinsics = identity_extrinsics();
    return !aerial_touch::project_point(intrinsics, extrinsics, { 10.0F, 10.0F, -500.0F }).has_value()
           && !aerial_touch::project_point(intrinsics, extrinsics, { 10.0F, 10.0F, 0.0F }).has_value()
           && !aerial_touch::deproject_pixel(intrinsics, extrinsics, { 10.0F, 10.0F }, -1.0F).has_value();
}

bool plane_unproject_inverts_project() {
    const auto plane = aerial_touch::Plane::from_calibration_points(
        { 100.0F, 100.0F, 1000.0F }, { 340.0F, 100.0F, 1000.0F }, { 220.0F, 420.0F, 1000.0F });
    if(!plane.has_value()) {
        return false;
    }
    for(const auto uv : { aerial_touch::Vec2{ 0.0F, 0.0F }, aerial_touch::Vec2{ 240.0F, 320.0F },
                          aerial_touch::Vec2{ 80.0F, 160.0F } }) {
        for(const float height : { 0.0F, 25.0F }) {
            const auto projected = plane->project(plane->unproject(uv, height));
            if(!approximately_equal(projected.u_mm, uv.x) || !approximately_equal(projected.v_mm, uv.y)
               || !approximately_equal(projected.signed_distance_mm, height)) {
                return false;
            }
        }
    }
    return true;
}

// --- fingertip depth probe --------------------------------------------------------------------

bool fingertip_probe_extrapolates_tip_depth_from_the_joint_gradient() {
    // Synthetic finger: depth grows linearly from 900 mm at the tip to 940 mm at the DIP joint.
    const aerial_touch::Vec2 tip{ 100.0F, 100.0F };
    const aerial_touch::Vec2 joint{ 100.0F, 140.0F };
    const auto sample = [&](const aerial_touch::Vec2 pixel) -> std::optional<float> {
        const float ratio = (pixel.y - tip.y) / (joint.y - tip.y);
        return 900.0F + 40.0F * ratio;
    };
    const auto estimate = aerial_touch::estimate_fingertip_depth_mm(tip, joint, {}, sample);
    return estimate.has_value() && estimate->extrapolated
           && std::fabs(estimate->depth_mm - 900.0F) < 0.1F;
}

bool fingertip_probe_falls_back_when_the_finger_points_at_the_camera() {
    // Tip and joint land on the same pixel, so there is no gradient to extrapolate along.
    const aerial_touch::Vec2 tip{ 100.0F, 100.0F };
    const auto sample = [](const aerial_touch::Vec2) -> std::optional<float> { return 870.0F; };
    const auto estimate = aerial_touch::estimate_fingertip_depth_mm(tip, tip, {}, sample);
    return estimate.has_value() && !estimate->extrapolated
           && approximately_equal(estimate->depth_mm, 870.0F);
}

bool fingertip_probe_clamps_a_runaway_gradient() {
    const aerial_touch::Vec2 tip{ 100.0F, 100.0F };
    const aerial_touch::Vec2 joint{ 100.0F, 160.0F };
    // A depth cliff between the two probes would otherwise fling the tip hundreds of mm away.
    const auto sample = [&](const aerial_touch::Vec2 pixel) -> std::optional<float> {
        return pixel.y < 130.0F ? 900.0F : 1400.0F;
    };
    aerial_touch::FingertipDepthProbeConfig config;
    const auto estimate = aerial_touch::estimate_fingertip_depth_mm(tip, joint, config, sample);
    return estimate.has_value()
           && std::fabs(estimate->depth_mm - 900.0F) <= config.maximum_extrapolation_mm + 0.01F;
}

bool fingertip_probe_reports_nothing_without_a_usable_near_sample() {
    const auto sample = [](const aerial_touch::Vec2) -> std::optional<float> { return std::nullopt; };
    return !aerial_touch::estimate_fingertip_depth_mm({ 10.0F, 10.0F }, { 10.0F, 40.0F }, {}, sample)
                .has_value();
}

// --- surface scan sampling --------------------------------------------------------------------

// The ring around the fingertip lands partly on the finger; only pixels behind it are table.
bool annulus_sampling_rejects_samples_in_front_of_the_fingertip() {
    const int width = 41;
    const int height = 41;
    std::vector<std::uint16_t> depth(static_cast<std::size_t>(width * height), 1000U);
    // Left half of the ring sits on the finger, 80 mm nearer the camera than the table.
    for(int y = 0; y < height; ++y) {
        for(int x = 0; x < 20; ++x) {
            depth[static_cast<std::size_t>(y * width + x)] = 920U;
        }
    }
    const auto unfiltered =
        aerial_touch::sample_depth_annulus_mm(depth, width, height, 20, 20, 3, 10, 1.0F);
    const auto filtered =
        aerial_touch::sample_depth_annulus_mm(depth, width, height, 20, 20, 3, 10, 1.0F, 926.0F);
    if(unfiltered.size() <= filtered.size() || filtered.empty()) {
        return false;
    }
    for(const auto& sample : filtered) {
        if(sample.depth_mm < 926.0F) {
            return false;
        }
    }
    return true;
}

// --- touch triggering -------------------------------------------------------------------------

// Regression test for the missed press. People decelerate as they approach a target, so the
// instantaneous speed on the frame that crosses touch_threshold_mm is far below the speed of the
// approach itself. The velocity gate is therefore latched over the whole approach, not sampled on
// the crossing frame. Before the latch this exact trajectory produced no event at all.
bool decelerating_approach_still_presses() {
    aerial_touch::TouchStateMachine touch({ 10.0F, 20.0F, 40.0F, 300, 0 });
    float distance = 60.0F;
    std::int64_t timestamp = 0;
    for(int frame = 0; frame < 120; ++frame) {
        distance -= 150.0F * (distance / 60.0F) / 30.0F;  // 150 mm/s peak, decaying towards the table
        timestamp += 33;
        const auto event = touch.update({ timestamp, distance, "5" });
        if(event.has_value()) {
            return event->trigger == aerial_touch::PressTrigger::Approach;
        }
    }
    return false;
}

bool slow_approach_presses_by_dwelling() {
    aerial_touch::TouchStateMachine touch({ 10.0F, 20.0F, 40.0F, 300, 350 });
    float distance = 60.0F;
    std::int64_t timestamp = 0;
    for(int frame = 0; frame < 400; ++frame) {
        distance -= 50.0F * (distance / 60.0F) / 30.0F;  // far too slow for the velocity gate
        timestamp += 33;
        const auto event = touch.update({ timestamp, distance, "5" });
        if(event.has_value()) {
            return event->trigger == aerial_touch::PressTrigger::Dwell;
        }
    }
    return false;
}

bool dwell_does_not_fire_before_the_configured_hold() {
    aerial_touch::TouchStateMachine touch({ 10.0F, 20.0F, 1000.0F, 300, 350 });
    touch.update({ 0, 30.0F, "5" });
    if(touch.update({ 100, 8.0F, "5" }).has_value()) {
        return false;
    }
    if(touch.update({ 300, 8.0F, "5" }).has_value()) {
        return false;
    }
    const auto press = touch.update({ 460, 8.0F, "5" });
    return press.has_value() && press->trigger == aerial_touch::PressTrigger::Dwell;
}

// Frame gaps stay under tracking_timeout_ms throughout: a longer gap would legitimately disarm the
// machine and would be testing the timeout rather than the dwell restart.
bool dwell_restarts_when_sliding_onto_another_key() {
    aerial_touch::TouchStateMachine touch({ 10.0F, 20.0F, 1000.0F, 300, 350 });
    touch.update({ 0, 30.0F, "5" });
    touch.update({ 100, 8.0F, "5" });  // hold on "5" starts here
    touch.update({ 250, 8.0F, "5" });
    // Slide onto "6" at 430 ms: "5" had accumulated 330 ms and would have fired on the next frame.
    if(touch.update({ 430, 8.0F, "6" }).has_value()) {
        return false;
    }
    if(touch.update({ 480, 8.0F, "6" }).has_value()) {
        return false;
    }
    if(touch.update({ 700, 8.0F, "6" }).has_value()) {
        return false;
    }
    const auto press = touch.update({ 790, 8.0F, "6" });
    return press.has_value() && press->key == "6"
           && press->trigger == aerial_touch::PressTrigger::Dwell;
}

bool dwell_cannot_repeat_without_releasing() {
    aerial_touch::TouchStateMachine touch({ 10.0F, 20.0F, 1000.0F, 300, 350 });
    touch.update({ 0, 30.0F, "5" });
    touch.update({ 100, 8.0F, "5" });
    touch.update({ 300, 8.0F, "5" });
    if(!touch.update({ 460, 8.0F, "5" }).has_value()) {
        return false;
    }
    // Resting on the key must not auto-repeat; only retreating past the release threshold re-arms.
    for(std::int64_t timestamp = 560; timestamp <= 2000; timestamp += 100) {
        if(touch.update({ timestamp, 8.0F, "5" }).has_value()) {
            return false;
        }
    }
    touch.update({ 2100, 25.0F, "5" });  // retreat: re-arms and clears the hold
    if(touch.update({ 2200, 8.0F, "5" }).has_value()) {
        return false;
    }
    if(touch.update({ 2400, 8.0F, "5" }).has_value()) {
        return false;
    }
    return touch.update({ 2570, 8.0F, "5" }).has_value();
}

bool tracking_loss_clears_the_latched_approach() {
    aerial_touch::TouchStateMachine touch({ 10.0F, 20.0F, 40.0F, 300, 0 });
    touch.update({ 0, 30.0F, "5" });
    touch.update({ 100, 15.0F, "5" });  // fast enough to latch
    if(!touch.approach_latched()) {
        return false;
    }
    touch.mark_tracking_lost(150);
    return !touch.approach_latched();
}

// The latched approach gate must not outlive the approach that set it. These two cases are the
// regressions the latch introduced when it was first written, and both fire with dwell_ms = 0,
// i.e. in the configuration documented as requiring a genuine approach.

// Descending outside the keypad and then sliding sideways onto a key is not a press: the descent
// never had a key under it, so its speed cannot authorise the key the finger later slides onto.
bool lateral_slide_onto_a_key_is_not_a_press() {
    aerial_touch::TouchStateMachine touch({ 10.0F, 20.0F, 40.0F, 300, 0 });
    touch.update({ 0, 40.0F, std::nullopt });
    touch.update({ 33, 25.0F, std::nullopt });
    touch.update({ 66, 12.0F, std::nullopt });   // fast enough to latch
    touch.update({ 99, 3.0F, std::nullopt });    // now resting at the surface, still no key
    // Hand slides sideways into the keypad without any downward motion at all.
    return !touch.update({ 132, 3.0F, "7" }).has_value()
           && !touch.update({ 165, 3.0F, "7" }).has_value();
}

// Backing away and then crawling down is judged on the crawl, not on the earlier fast approach.
bool retreat_cancels_the_latched_approach() {
    aerial_touch::TouchStateMachine touch({ 10.0F, 20.0F, 40.0F, 300, 0 });
    touch.update({ 0, 40.0F, "5" });
    touch.update({ 33, 25.0F, "5" });
    touch.update({ 66, 12.0F, "5" });  // latched here
    if(!touch.approach_latched()) {
        return false;
    }
    // Retreat, but stop short of release_threshold_mm so the machine never re-arms.
    touch.update({ 99, 15.0F, "5" });
    touch.update({ 132, 18.0F, "5" });
    touch.update({ 165, 19.0F, "5" });
    if(touch.approach_latched()) {
        return false;
    }
    // A 3 mm/s crawl back down is exactly what min_approach_velocity_mm_s is meant to reject.
    float distance = 19.0F;
    for(std::int64_t timestamp = 198; distance > 4.0F; timestamp += 33) {
        distance -= 0.1F;
        if(touch.update({ timestamp, distance, "5" }).has_value()) {
            return false;
        }
    }
    return true;
}

// Changing thresholds must not let evidence gathered under the old ones fire under the new ones --
// otherwise dragging a slider in the settings window emits a keystroke.
bool config_change_discards_stale_approach_evidence() {
    aerial_touch::TouchStateMachine touch({ 10.0F, 20.0F, 40.0F, 300, 0 });
    touch.update({ 0, 40.0F, "5" });
    touch.update({ 33, 25.0F, "5" });
    touch.update({ 66, 14.0F, "5" });  // latched, but 14 mm is still outside the touch zone
    if(!touch.approach_latched()) {
        return false;
    }
    // Widen the touch band while the finger sits still at 14 mm.
    touch.set_config({ 18.0F, 30.0F, 40.0F, 300, 0 });
    return !touch.approach_latched() && !touch.update({ 99, 14.0F, "5" }).has_value();
}

bool run_interaction_core_tests() {
    // Every case runs and names itself on failure. The old form was one long && chain, so the first
    // failure short-circuited the rest and the suite reported nothing but a non-zero exit code.
    const std::array<std::pair<const char*, bool (*)()>, 61> tests{ {
        { "plane_projection_uses_camera_facing_normal", &plane_projection_uses_camera_facing_normal },
        { "plane_uses_configured_minimum_point_distance", &plane_uses_configured_minimum_point_distance },
        { "plane_rejects_nearly_collinear_points", &plane_rejects_nearly_collinear_points },
        { "plane_unproject_inverts_project", &plane_unproject_inverts_project },
        { "camera_projection_round_trips_pixels", &camera_projection_round_trips_pixels },
        { "camera_projection_round_trips_through_a_rotated_extrinsic", &camera_projection_round_trips_through_a_rotated_extrinsic },
        { "camera_projection_rejects_points_behind_the_camera", &camera_projection_rejects_points_behind_the_camera },
        { "surface_plane_fit_removes_fingertip_height_offsets", &surface_plane_fit_removes_fingertip_height_offsets },
        { "surface_projection_keeps_keypad_geometry_with_varying_fingertip_heights", &surface_projection_keeps_keypad_geometry_with_varying_fingertip_heights },
        { "surface_plane_fit_rejects_competing_depth_planes", &surface_plane_fit_rejects_competing_depth_planes },
        { "surface_plane_fit_supports_rotated_target_and_ray_intersection", &surface_plane_fit_supports_rotated_target_and_ray_intersection },
        { "surface_scan_completes_only_after_minimum_duration_and_valid_plane", &surface_scan_completes_only_after_minimum_duration_and_valid_plane },
        { "surface_scan_fails_after_timeout_without_a_valid_plane", &surface_scan_fails_after_timeout_without_a_valid_plane },
        { "surface_scan_throttles_failed_plane_fit_attempts", &surface_scan_throttles_failed_plane_fit_attempts },
        { "surface_scan_retries_after_camera_timestamp_moves_backward", &surface_scan_retries_after_camera_timestamp_moves_backward },
        { "annulus_sampling_rejects_samples_in_front_of_the_fingertip", &annulus_sampling_rejects_samples_in_front_of_the_fingertip },
        { "fingertip_probe_extrapolates_tip_depth_from_the_joint_gradient", &fingertip_probe_extrapolates_tip_depth_from_the_joint_gradient },
        { "fingertip_probe_falls_back_when_the_finger_points_at_the_camera", &fingertip_probe_falls_back_when_the_finger_points_at_the_camera },
        { "fingertip_probe_clamps_a_runaway_gradient", &fingertip_probe_clamps_a_runaway_gradient },
        { "fingertip_probe_reports_nothing_without_a_usable_near_sample", &fingertip_probe_reports_nothing_without_a_usable_near_sample },
        { "keypad_calibration_derives_contiguous_cells", &keypad_calibration_derives_contiguous_cells },
        { "keypad_calibration_leaves_no_dead_band_between_keys", &keypad_calibration_leaves_no_dead_band_between_keys },
        { "keypad_calibration_supports_rotated_3d_plane", &keypad_calibration_supports_rotated_3d_plane },
        { "keypad_calibration_ignores_lateral_error_in_bottom_point", &keypad_calibration_ignores_lateral_error_in_bottom_point },
        { "keypad_calibration_rejects_invalid_geometry", &keypad_calibration_rejects_invalid_geometry },
        { "keypad_calibration_reports_bottom_point_outside_keypad", &keypad_calibration_reports_bottom_point_outside_keypad },
        { "keypad_maps_uv_to_expected_number", &keypad_maps_uv_to_expected_number },
        { "keypad_overlay_prioritizes_pressed_key", &keypad_overlay_prioritizes_pressed_key },
        { "keypad_overlay_clears_pressed_key_when_not_currently_held", &keypad_overlay_clears_pressed_key_when_not_currently_held },
        { "fixed_keypad_overlay_layout_has_default_size", &fixed_keypad_overlay_layout_has_default_size },
        { "touch_requires_release_before_repeat_press", &touch_requires_release_before_repeat_press },
        { "touch_does_not_press_outside_keypad", &touch_does_not_press_outside_keypad },
        { "touch_uses_elapsed_time_for_approach_velocity", &touch_uses_elapsed_time_for_approach_velocity },
        { "touch_config_update_preserves_armed_state", &touch_config_update_preserves_armed_state },
        { "tracking_loss_requires_release_before_pressing_again", &tracking_loss_requires_release_before_pressing_again },
        { "short_tracking_loss_cannot_trigger_on_reacquisition", &short_tracking_loss_cannot_trigger_on_reacquisition },
        { "tracking_loss_clears_the_latched_approach", &tracking_loss_clears_the_latched_approach },
        { "lateral_slide_onto_a_key_is_not_a_press", &lateral_slide_onto_a_key_is_not_a_press },
        { "retreat_cancels_the_latched_approach", &retreat_cancels_the_latched_approach },
        { "config_change_discards_stale_approach_evidence", &config_change_discards_stale_approach_evidence },
        { "decelerating_approach_still_presses", &decelerating_approach_still_presses },
        { "slow_approach_presses_by_dwelling", &slow_approach_presses_by_dwelling },
        { "dwell_does_not_fire_before_the_configured_hold", &dwell_does_not_fire_before_the_configured_hold },
        { "dwell_restarts_when_sliding_onto_another_key", &dwell_restarts_when_sliding_onto_another_key },
        { "dwell_cannot_repeat_without_releasing", &dwell_cannot_repeat_without_releasing },
        { "press_event_keeps_fingertip_and_plane_coordinates", &press_event_keeps_fingertip_and_plane_coordinates },
        { "missing_hand_tracker_dll_is_safe", &missing_hand_tracker_dll_is_safe },
        { "yaml_config_loads_all_runtime_thresholds", &yaml_config_loads_all_runtime_thresholds },
        { "yaml_config_round_trips_through_save", &yaml_config_round_trips_through_save },
        { "unsupported_camera_settings_are_not_selected", &unsupported_camera_settings_are_not_selected },
        { "camera_fallback_and_filter_order_are_explicit", &camera_fallback_and_filter_order_are_explicit },
        { "camera_profile_resolver_requires_matching_rgb_and_depth_fps", &camera_profile_resolver_requires_matching_rgb_and_depth_fps },
        { "camera_capabilities_only_offer_usable_matching_fps", &camera_capabilities_only_offer_usable_matching_fps },
        { "failed_depth_filter_is_skipped_without_losing_the_frame", &failed_depth_filter_is_skipped_without_losing_the_frame },
        { "yaml_config_rejects_nonfinite_values", &yaml_config_rejects_nonfinite_values },
        { "app_config_validation_rejects_invalid_values", &app_config_validation_rejects_invalid_values },
        { "legacy_yaml_uses_defaults_for_new_stabilization_fields", &legacy_yaml_uses_defaults_for_new_stabilization_fields },
        { "preview_classifies_touch_zones", &preview_classifies_touch_zones },
        { "settings_layout_adapts_to_window_size", &settings_layout_adapts_to_window_size },
        { "rgbd_frame_validates_alignment_and_buffer_sizes", &rgbd_frame_validates_alignment_and_buffer_sizes },
        { "unavailable_hardware_d2c_uses_software_alignment", &unavailable_hardware_d2c_uses_software_alignment },
    } };

    bool all_passed = true;
    for(const auto& test : tests) {
        if(!test.second()) {
            std::cerr << "FAILED: " << test.first << "\n";
            all_passed = false;
        }
    }
    return all_passed;
}

