#include "aerial_touch/app_config.hpp"
#include "aerial_touch/alignment_mode.hpp"
#include "aerial_touch/keypad.hpp"
#include "aerial_touch/hand_tracker.hpp"
#include "aerial_touch/plane.hpp"
#include "aerial_touch/keypad_overlay.hpp"
#include "aerial_touch/rgbd_frame.hpp"
#include "aerial_touch/settings_window.hpp"
#include "aerial_touch/touch_state_machine.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>

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

bool keypad_maps_uv_to_expected_number() {
    const aerial_touch::Keypad keypad({ 30.0F, 30.0F, 5.0F, 5.0F });
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
    return aerial_touch::currently_pressed_key(last_pressed_key, false, true, false) == last_pressed_key
           && !aerial_touch::currently_pressed_key(last_pressed_key, true, true, false).has_value()
           && !aerial_touch::currently_pressed_key(last_pressed_key, false, false, false).has_value()
           && !aerial_touch::currently_pressed_key(last_pressed_key, false, true, true).has_value();
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
    return config.depth.sample_radius == 3 && approximately_equal(config.touch.touch_threshold_mm, 9.0F)
           && approximately_equal(config.touch.release_threshold_mm, 22.0F)
           && approximately_equal(config.touch.min_approach_velocity_mm_s, 45.0F)
           && config.touch.tracking_timeout_ms == 275 && approximately_equal(config.keypad.key_width_mm, 31.0F)
           && approximately_equal(config.keypad.key_height_mm, 32.0F)
           && approximately_equal(config.keypad.horizontal_gap_mm, 6.0F)
           && approximately_equal(config.keypad.vertical_gap_mm, 7.0F)
           && approximately_equal(config.calibration.minimum_point_distance_mm, 85.0F);
}

bool yaml_config_round_trips_through_save() {
    const auto source = aerial_touch::load_app_config(
        std::filesystem::path(TEST_SOURCE_DIR) / "tests" / "data" / "config.yaml");
    const auto path = std::filesystem::temp_directory_path() / "aerial_touch_config_roundtrip.yaml";
    aerial_touch::save_app_config(source, path);
    const auto restored = aerial_touch::load_app_config(path);
    std::filesystem::remove(path);
    return restored.depth.sample_radius == source.depth.sample_radius
           && approximately_equal(restored.touch.touch_threshold_mm, source.touch.touch_threshold_mm)
           && approximately_equal(restored.touch.release_threshold_mm, source.touch.release_threshold_mm)
           && approximately_equal(restored.touch.min_approach_velocity_mm_s, source.touch.min_approach_velocity_mm_s)
           && restored.touch.tracking_timeout_ms == source.touch.tracking_timeout_ms
           && approximately_equal(restored.keypad.key_width_mm, source.keypad.key_width_mm)
           && approximately_equal(restored.keypad.key_height_mm, source.keypad.key_height_mm)
           && approximately_equal(restored.keypad.horizontal_gap_mm, source.keypad.horizontal_gap_mm)
           && approximately_equal(restored.keypad.vertical_gap_mm, source.keypad.vertical_gap_mm)
           && approximately_equal(restored.calibration.minimum_point_distance_mm,
                                  source.calibration.minimum_point_distance_mm);
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
    config.keypad.horizontal_gap_mm = -1.0F;
    bool rejects_negative_gap = false;
    try {
        aerial_touch::validate_app_config(config);
    }
    catch(const std::exception&) {
        rejects_negative_gap = true;
    }
    return rejects_equal_thresholds && rejects_negative_gap;
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

bool run_interaction_core_tests() {
    return plane_projection_uses_camera_facing_normal() && plane_uses_configured_minimum_point_distance()
           && plane_rejects_nearly_collinear_points()
           && keypad_maps_uv_to_expected_number() && keypad_overlay_prioritizes_pressed_key()
           && keypad_overlay_clears_pressed_key_when_not_currently_held()
           && touch_requires_release_before_repeat_press() && touch_does_not_press_outside_keypad()
           && touch_uses_elapsed_time_for_approach_velocity()
           && touch_config_update_preserves_armed_state()
           && tracking_loss_requires_release_before_pressing_again()
           && press_event_keeps_fingertip_and_plane_coordinates() && missing_hand_tracker_dll_is_safe()
           && yaml_config_loads_all_runtime_thresholds() && yaml_config_round_trips_through_save()
           && yaml_config_rejects_nonfinite_values() && app_config_validation_rejects_invalid_values()
           && preview_classifies_touch_zones() && settings_layout_adapts_to_window_size()
           && rgbd_frame_validates_alignment_and_buffer_sizes()
           && unavailable_hardware_d2c_uses_software_alignment();
}
