#include "aerial_touch/app_config.hpp"
#include "aerial_touch/calibration_geometry.hpp"
#include "aerial_touch/calibration_sampler.hpp"
#include "aerial_touch/depth_sampler.hpp"
#include "aerial_touch/fingertip_probe.hpp"
#include "aerial_touch/hand_tracker.hpp"
#include "aerial_touch/keypad.hpp"
#include "aerial_touch/keypad_overlay.hpp"
#include "aerial_touch/orbbec_camera.hpp"
#include "aerial_touch/plane.hpp"
#include "aerial_touch/settings_window.hpp"
#include "aerial_touch/signal_stabilizer.hpp"
#include "aerial_touch/surface_plane.hpp"
#include "aerial_touch/surface_scan.hpp"
#include "aerial_touch/touch_state_machine.hpp"
#include "aerial_touch/utf8_text.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr int kIndexFingerTip = 8;
constexpr int kIndexFingerDip = 7;
constexpr const char* kWindowName = "aerial_touch_window";
constexpr float kRayNearDepthMm = 400.0F;
constexpr float kRayFarDepthMm = 1600.0F;
// Rejecting the ring pixels that land on the finger removes roughly the near half of each annulus,
// so the per-frame yield needed to be lowered (and a wider ring added) to keep the scan converging.
constexpr std::size_t kMinimumSurfacePatchSamples = 8U;

struct CliOptions {
    std::filesystem::path config{ "config/default.yaml" };
    std::filesystem::path bridge{ "mediapipe_hand_bridge.dll" };
    std::filesystem::path model{ "assets/models/hand_landmarker.task" };
};

CliOptions parse_options(const int argc, char** argv) {
    CliOptions options;
    for(int index = 1; index + 1 < argc; index += 2) {
        const std::string key = argv[index];
        if(key == "--config") {
            options.config = argv[index + 1];
        }
        else if(key == "--bridge") {
            options.bridge = argv[index + 1];
        }
        else if(key == "--model") {
            options.model = argv[index + 1];
        }
        else {
            throw std::runtime_error(std::string(u8"無法識別的選項：") + key);
        }
    }
    if((argc - 1) % 2 != 0) {
        throw std::runtime_error(u8"每個選項都必須提供值");
    }
    return options;
}

std::string vec3_text(const aerial_touch::Vec3 value) {
    std::ostringstream output;
    output << std::fixed << std::setprecision(1) << value.x << ", " << value.y << ", " << value.z << " mm";
    return output.str();
}

std::string vec2_text(const aerial_touch::Vec2 value) {
    std::ostringstream output;
    output << std::fixed << std::setprecision(1) << value.x << ", " << value.y << " mm";
    return output.str();
}

void text_line(aerial_touch::Utf8TextCanvas& canvas,
               const std::string& text,
               const int row,
               const cv::Scalar color = { 255, 255, 255 }) {
    canvas.draw(text, { 12, 10 + row * 34 }, color);
}

// Collect target-surface points around where the operator is pointing.
//
// The region has to be a large fraction of the keypad. Sampling a ring a few pixels wide -- which is
// what this did -- yields thousands of points strung along a ribbon millimetres across; a plane fits
// such a ribbon to a fraction of a millimetre while its rotation about the ribbon's axis stays
// essentially free, so the scan reported a perfect RMS and handed back a normal tilted by degrees.
// Across a 200 mm keypad that is centimetres of error, and it is exactly what put the projected
// keypad off the physical target.
//
// The hand is excluded by landmark proximity, not by depth: a hand laid flat against the target is
// nearly coplanar with it, so no depth threshold can separate the two. The depth window keeps the
// samples on the surface being pointed at instead of the desk or the far wall.
std::vector<aerial_touch::Vec3> surface_patch_samples(const aerial_touch::OrbbecCamera& camera,
                                                       const aerial_touch::RgbdFrame& frame,
                                                       const aerial_touch::HandObservation& hand,
                                                       const aerial_touch::Vec2 fingertip_pixel,
                                                       const float fingertip_depth_mm,
                                                       const aerial_touch::DepthSamplingConfig& depth_config) {
    std::vector<aerial_touch::Vec2> hand_points;
    if(hand.detected) {
        const int count = std::min(hand.landmark_count, static_cast<int>(hand.landmarks.size()));
        hand_points.reserve(static_cast<std::size_t>(std::max(0, count)));
        for(int index = 0; index < count; ++index) {
            hand_points.push_back({ hand.landmarks[index].x * static_cast<float>(frame.color_width),
                                    hand.landmarks[index].y * static_cast<float>(frame.color_height) });
        }
    }

    aerial_touch::SurfaceSampleRegion region;
    region.center_x = static_cast<int>(std::lround(fingertip_pixel.x));
    region.center_y = static_cast<int>(std::lround(fingertip_pixel.y));
    region.radius_px = depth_config.surface_scan_radius_px;
    region.stride_px = depth_config.surface_scan_stride_px;
    region.minimum_depth_mm = std::max(0.0F, fingertip_depth_mm - depth_config.finger_clearance_mm);
    region.maximum_depth_mm = fingertip_depth_mm + depth_config.surface_depth_window_mm;

    const auto depth_samples = aerial_touch::sample_depth_surface_grid_mm(
        frame.depth, frame.depth_width, frame.depth_height, region, frame.depth_unit_mm, hand_points,
        depth_config.hand_exclusion_px);
    if(depth_samples.size() < kMinimumSurfacePatchSamples) {
        return {};
    }

    std::vector<aerial_touch::Vec3> samples;
    samples.reserve(depth_samples.size());
    for(const auto& sample : depth_samples) {
        const auto point = camera.deproject(frame, { static_cast<float>(sample.x), static_cast<float>(sample.y) },
                                            sample.depth_mm);
        if(point.has_value()) {
            samples.push_back(*point);
        }
    }
    return samples;
}

std::optional<aerial_touch::Vec3> surface_point_at_fingertip_pixel(
    const aerial_touch::OrbbecCamera& camera,
    const aerial_touch::RgbdFrame& frame,
    const aerial_touch::Vec2 fingertip_pixel,
    const aerial_touch::SurfacePlane& surface) {
    const auto near_point = camera.deproject(frame, fingertip_pixel, kRayNearDepthMm);
    const auto far_point = camera.deproject(frame, fingertip_pixel, kRayFarDepthMm);
    if(!near_point.has_value() || !far_point.has_value()) {
        return std::nullopt;
    }
    return surface.intersect_ray(*near_point,
                                 { far_point->x - near_point->x, far_point->y - near_point->y,
                                   far_point->z - near_point->z });
}

std::string calibration_failure_text(const aerial_touch::KeypadCalibrationFailure failure) {
    switch(failure) {
    case aerial_touch::KeypadCalibrationFailure::InvalidPlane:
        return u8"校正失敗：鍵盤表面方向無法建立";
    case aerial_touch::KeypadCalibrationFailure::InvalidDimensions:
        return u8"校正失敗：鍵盤或按鍵尺寸無效";
    case aerial_touch::KeypadCalibrationFailure::BottomPointOutsideKeypad:
        return u8"校正失敗：P3 未落在鍵盤下緣範圍內";
    case aerial_touch::KeypadCalibrationFailure::None:
        return u8"校正失敗：未知的鍵盤幾何錯誤";
    }
    return u8"校正失敗：未知的鍵盤幾何錯誤";
}

void draw_hand(cv::Mat& image, const aerial_touch::HandObservation& hand) {
    static constexpr std::array<std::pair<int, int>, 20> connections{ {
        { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 4 }, { 0, 5 }, { 5, 6 }, { 6, 7 }, { 7, 8 },
        { 5, 9 }, { 9, 10 }, { 10, 11 }, { 11, 12 }, { 9, 13 }, { 13, 14 }, { 14, 15 }, { 15, 16 },
        { 13, 17 }, { 0, 17 }, { 17, 18 }, { 18, 19 },
    } };
    if(!hand.detected || hand.landmark_count < 21) {
        return;
    }
    const auto point = [&image, &hand](const int index) {
        return cv::Point{ static_cast<int>(std::lround(hand.landmarks[index].x * image.cols)),
                          static_cast<int>(std::lround(hand.landmarks[index].y * image.rows)) };
    };
    for(const auto& connection : connections) {
        cv::line(image, point(connection.first), point(connection.second), { 70, 220, 70 }, 2, cv::LINE_AA);
    }
    for(int index = 0; index < 21; ++index) {
        cv::circle(image, point(index), index == kIndexFingerTip ? 7 : 3,
                   index == kIndexFingerTip ? cv::Scalar{ 20, 30, 240 } : cv::Scalar{ 30, 230, 230 }, -1, cv::LINE_AA);
    }
}

void draw_keypad(cv::Mat& image,
                 const bool keypad_available,
                 const std::optional<std::string>& hovered_key,
                 const std::optional<std::string>& pressed_key) {
    if(!keypad_available) {
        return;
    }
    static const auto layout = aerial_touch::fixed_keypad_overlay_layout();
    const int origin_x = std::max(layout.margin_px, image.cols - layout.width_px - layout.margin_px);
    const int origin_y = std::max(layout.margin_px, image.rows - layout.height_px - layout.margin_px);
    for(const auto& region : layout.regions) {
        const cv::Rect rect{ origin_x + region.x_px, origin_y + region.y_px, region.width_px, region.height_px };
        const auto state = aerial_touch::keypad_key_visual_state(region.key, hovered_key, pressed_key);
        const bool filled = state != aerial_touch::KeypadKeyVisualState::Idle;
        const cv::Scalar color = state == aerial_touch::KeypadKeyVisualState::Pressed
                                     ? cv::Scalar{ 80, 210, 80 }
                                     : state == aerial_touch::KeypadKeyVisualState::Hover
                                         ? cv::Scalar{ 20, 220, 255 }
                                         : cv::Scalar{ 230, 230, 230 };
        const cv::Scalar text_color = filled ? cv::Scalar{ 0, 0, 0 } : cv::Scalar{ 255, 255, 255 };
        cv::rectangle(image, rect, color, filled ? -1 : 2);
        cv::putText(image, region.key, { rect.x + rect.width / 3, rect.y + 2 * rect.height / 3 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, text_color, 2, cv::LINE_AA);
    }
}

// Draw the calibrated keypad where it physically is, by pushing each key's plane-space corners back
// through the same camera model the touch distances use. Without this the keypad is invisible in the
// world *and* absent from the video, so the operator is aiming at a remembered rectangle; with it, a
// wrong calibration or wrong intrinsics show up immediately as an overlay that does not sit on the
// target.
void draw_table_keypad(cv::Mat& image,
                       const aerial_touch::OrbbecCamera& camera,
                       const aerial_touch::RgbdFrame& frame,
                       const aerial_touch::Plane& plane,
                       const aerial_touch::Keypad& keypad,
                       const std::optional<std::string>& hovered_key,
                       const std::optional<std::string>& pressed_key) {
    // A plane seen almost edge-on projects its far corners to enormous coordinates; clamping the
    // magnitude keeps the float -> int conversion defined instead of relying on the calibration
    // always being sane. OpenCV clips the drawing itself.
    constexpr float kMaximumDrawablePixel = 20000.0F;

    struct DrawableKey {
        std::string key;
        std::vector<cv::Point> polygon;
        cv::Point centre;
        cv::Scalar color;
        aerial_touch::KeypadKeyVisualState state;
    };

    std::vector<DrawableKey> drawable;
    drawable.reserve(keypad.regions().size());
    for(const auto& region : keypad.regions()) {
        const std::array<aerial_touch::Vec2, 4> corners{ {
            { region.u_min_mm, region.v_min_mm },
            { region.u_max_mm, region.v_min_mm },
            { region.u_max_mm, region.v_max_mm },
            { region.u_min_mm, region.v_max_mm },
        } };

        std::vector<cv::Point> polygon;
        polygon.reserve(corners.size());
        for(const auto corner : corners) {
            const auto pixel = camera.project(frame, plane.unproject(corner));
            if(!pixel.has_value() || std::fabs(pixel->x) > kMaximumDrawablePixel
               || std::fabs(pixel->y) > kMaximumDrawablePixel) {
                break;
            }
            polygon.push_back({ static_cast<int>(std::lround(pixel->x)),
                                static_cast<int>(std::lround(pixel->y)) });
        }
        if(polygon.size() != corners.size()) {
            continue;
        }

        cv::Point centre{ 0, 0 };
        for(const auto& point : polygon) {
            centre.x += point.x;
            centre.y += point.y;
        }
        centre.x /= static_cast<int>(polygon.size());
        centre.y /= static_cast<int>(polygon.size());

        const auto state = aerial_touch::keypad_key_visual_state(region.key, hovered_key, pressed_key);
        const cv::Scalar color = state == aerial_touch::KeypadKeyVisualState::Pressed
                                     ? cv::Scalar{ 80, 210, 80 }
                                     : state == aerial_touch::KeypadKeyVisualState::Hover
                                           ? cv::Scalar{ 20, 220, 255 }
                                           : cv::Scalar{ 210, 190, 90 };
        drawable.push_back({ region.key, std::move(polygon), centre, color, state });
    }

    // One clone and one blend for the whole overlay: cloning per key would copy the full frame up to
    // ten times every frame.
    const bool any_highlight = std::any_of(drawable.begin(), drawable.end(), [](const DrawableKey& key) {
        return key.state != aerial_touch::KeypadKeyVisualState::Idle;
    });
    if(any_highlight) {
        cv::Mat highlight = image.clone();
        for(const auto& key : drawable) {
            if(key.state != aerial_touch::KeypadKeyVisualState::Idle) {
                cv::fillConvexPoly(highlight, key.polygon.data(), static_cast<int>(key.polygon.size()),
                                   key.color, cv::LINE_AA);
            }
        }
        cv::addWeighted(highlight, 0.35, image, 0.65, 0.0, image);
    }

    for(const auto& key : drawable) {
        cv::polylines(image, key.polygon, true, key.color,
                      key.state == aerial_touch::KeypadKeyVisualState::Idle ? 1 : 2, cv::LINE_AA);
        const cv::Point label{ key.centre.x - 7, key.centre.y + 7 };
        cv::putText(image, key.key, label, cv::FONT_HERSHEY_SIMPLEX, 0.6, { 0, 0, 0 }, 3, cv::LINE_AA);
        cv::putText(image, key.key, label, cv::FONT_HERSHEY_SIMPLEX, 0.6, key.color, 1, cv::LINE_AA);
    }
}

}  // namespace

int main(int argc, char** argv) {
    aerial_touch::enable_utf8_console();
    try {
        const CliOptions options = parse_options(argc, argv);
        auto config = aerial_touch::load_app_config(options.config);
        std::optional<aerial_touch::Keypad> keypad;
        aerial_touch::TouchStateMachine touch(config.touch);
        aerial_touch::HandSignalStabilizer fingertip_stabilizer(
            { { config.fingertip.min_cutoff_hz, config.fingertip.beta, config.fingertip.derivative_cutoff_hz },
              config.fingertip.display_hold_ms });
        aerial_touch::DepthSignalStabilizer depth_stabilizer(
            { config.depth.median_window_size, config.depth.max_jump_mm, config.depth.invalid_reset_frames });
        aerial_touch::DepthFreshnessGate raw_depth_freshness(config.depth.invalid_reset_frames);
        aerial_touch::CalibrationSampleCollector calibration_collector(
            { config.calibration.required_samples, config.calibration.mad_multiplier,
              config.calibration.minimum_outlier_threshold_mm });
        aerial_touch::SettingsWindow settings_window;
        bool settings_window_created = false;
        std::optional<aerial_touch::Plane> plane;
        aerial_touch::SurfaceScanCollector surface_scan;
        std::optional<aerial_touch::SurfacePlane> calibration_surface;
        std::vector<aerial_touch::Vec3> calibration_points;
        std::vector<aerial_touch::Vec3> calibration_spreads;
        std::optional<aerial_touch::Vec3> current_xyz;
        std::optional<aerial_touch::Vec2> current_uv;
        std::optional<float> current_distance;
        std::optional<float> raw_distance;
        std::optional<std::string> current_key;
        std::optional<std::string> sticky_key;
        std::optional<std::string> active_pressed_key;
        std::optional<aerial_touch::PressEvent> last_event;
        std::optional<std::int64_t> last_confirmed_timestamp_ms;
        bool calibrating = false;
        bool scanning_surface = false;
        bool collecting_calibration_samples = false;

        const auto apply_runtime_config = [&](const aerial_touch::AppConfig& candidate, std::string& error) {
            try {
                aerial_touch::save_app_config(candidate, options.config);
                config = candidate;
                touch.set_config(config.touch);
                sticky_key.reset();
                active_pressed_key.reset();
                return true;
            }
            catch(const std::exception& exception) {
                error = exception.what();
                return false;
            }
        };

        aerial_touch::HandTracker hand_tracker(options.bridge, options.model);
        aerial_touch::OrbbecCamera camera;

        if(!camera.start(config.camera)) {
            std::cerr << camera.error() << '\n';
            return 2;
        }
        const auto& camera_info = camera.runtime_info();
        std::cout << u8"Orbbec 對齊模式：" << (camera.hardware_alignment() ? u8"硬體 D2C" : u8"軟體 D2C")
                  << u8"；深度模式=" << camera_info.depth_work_mode
                  << u8"；深度精度=" << camera_info.depth_precision
                  << u8"；FPS=" << camera_info.fps << '\n';
        std::cout << u8"Gemini 2 可用設定：模式=";
        for(const auto& mode : camera_info.capabilities.depth_work_modes) {
            std::cout << '[' << mode << "] ";
        }
        std::cout << u8"；精度=";
        for(const auto& precision : camera_info.capabilities.depth_precisions) {
            std::cout << '[' << precision << "] ";
        }
        std::cout << u8"；FPS=";
        for(const int value : camera_info.capabilities.fps_values) {
            std::cout << '[' << value << "] ";
        }
        std::cout << u8"；RGB 防閃爍=";
        for(const int value : camera_info.capabilities.rgb_power_line_frequencies_hz) {
            std::cout << '[' << value << " Hz] ";
        }
        std::cout << u8"；SDK 濾波器="
                  << (camera_info.capabilities.temporal_filter_available ? u8"Temporal " : "")
                  << (camera_info.capabilities.spatial_filter_available ? u8"Spatial " : "")
                  << (camera_info.capabilities.hole_filling_filter_available ? u8"HoleFilling" : "") << '\n';
        for(const auto& warning : camera_info.warnings) {
            std::cerr << u8"相機設定警告：" << warning << '\n';
        }
        if(!hand_tracker.available()) {
            std::cerr << hand_tracker.error() << '\n';
        }

        std::string status = hand_tracker.available() ? u8"按 C 開始設定數字鍵盤範圍" : hand_tracker.error();
        auto fps_start = std::chrono::steady_clock::now();
        int fps_frames = 0;
        float fps = 0.0F;

        const auto toggle_settings_window = [&]() {
            if(!settings_window_created) {
                settings_window_created = settings_window.create(config, camera_info.capabilities,
                                                                 options.config, apply_runtime_config);
                if(!settings_window_created) {
                    status = u8"參數設定視窗建立失敗";
                }
            }
            if(settings_window_created) {
                if(settings_window.visible()) {
                    settings_window.hide();
                }
                else {
                    settings_window.show();
                }
            }
        };

        cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
        aerial_touch::set_utf8_window_title(kWindowName, u8"Gemini 2 空中鍵盤");
        for(;;) {
            settings_window.process_messages();
            const auto frame = camera.capture(100);
            if(!frame.has_value()) {
                touch.mark_tracking_lost(last_confirmed_timestamp_ms.value_or(0) + 1);
                fingertip_stabilizer.reset();
                depth_stabilizer.reset();
                raw_depth_freshness.reset();
                camera.reset_depth_filters();
                sticky_key.reset();
                active_pressed_key.reset();
                if(collecting_calibration_samples) {
                    calibration_collector.clear();
                    collecting_calibration_samples = false;
                    status = u8"校正取樣中斷：相機影像暫時無效，請按空白鍵重試";
                }
                const int key = cv::waitKey(1);
                if(key == 'q' || key == 'Q' || key == 27) {
                    break;
                }
                if(key == 's' || key == 'S') {
                    toggle_settings_window();
                }
                continue;
            }

            cv::Mat rgb(frame->color_height, frame->color_width, CV_8UC3, const_cast<std::uint8_t*>(frame->rgb.data()));
            cv::Mat display;
            cv::cvtColor(rgb, display, cv::COLOR_RGB2BGR);
            const auto hand = hand_tracker.detect_rgb(frame->rgb.data(), frame->color_width, frame->color_height,
                                                      frame->color_width * 3, frame->timestamp_ms);
            draw_hand(display, hand);

            current_xyz.reset();
            current_uv.reset();
            current_distance.reset();
            raw_distance.reset();
            current_key.reset();
            std::optional<std::string> fingertip_pixel_text;
            std::optional<aerial_touch::Vec2> raw_pixel;
            std::optional<aerial_touch::Vec2> filtered_tip_pixel;
            if(hand.detected && hand.landmark_count > kIndexFingerTip
               && std::isfinite(hand.landmarks[kIndexFingerTip].x)
               && std::isfinite(hand.landmarks[kIndexFingerTip].y)) {
                raw_pixel = aerial_touch::Vec2{
                    hand.landmarks[kIndexFingerTip].x * static_cast<float>(frame->color_width),
                    hand.landmarks[kIndexFingerTip].y * static_cast<float>(frame->color_height),
                };
            }
            const auto stabilized_tip = fingertip_stabilizer.update(raw_pixel, frame->timestamp_ms);
            if(stabilized_tip.has_value()) {
                const int filtered_x = std::clamp(static_cast<int>(std::lround(stabilized_tip->filtered.x)),
                                                  0, frame->color_width - 1);
                const int filtered_y = std::clamp(static_cast<int>(std::lround(stabilized_tip->filtered.y)),
                                                  0, frame->color_height - 1);
                filtered_tip_pixel = { static_cast<float>(filtered_x), static_cast<float>(filtered_y) };
                cv::circle(display, { filtered_x, filtered_y }, 5,
                           stabilized_tip->confirmed_this_frame ? cv::Scalar{ 255, 120, 20 }
                                                                  : cv::Scalar{ 160, 160, 160 },
                           2, cv::LINE_AA);
                fingertip_pixel_text = std::string(u8"指尖像素（濾波）：") + std::to_string(filtered_x) + ", "
                                        + std::to_string(filtered_y)
                                        + (stabilized_tip->confirmed_this_frame ? "" : u8"（僅保留顯示）");
            }

            std::optional<aerial_touch::Vec3> raw_xyz;
            std::optional<float> tip_depth_mm;
            bool tip_depth_extrapolated = false;
            bool tip_depth_probed = false;
            const bool confirmed_tip = stabilized_tip.has_value() && stabilized_tip->confirmed_this_frame;
            if(!confirmed_tip) {
                filtered_tip_pixel.reset();
            }
            if(confirmed_tip) {
                last_confirmed_timestamp_ms = frame->timestamp_ms;
                const int raw_x = std::clamp(static_cast<int>(std::lround(stabilized_tip->raw.x)),
                                             0, frame->color_width - 1);
                const int raw_y = std::clamp(static_cast<int>(std::lround(stabilized_tip->raw.y)),
                                             0, frame->color_height - 1);
                const int filtered_x = std::clamp(static_cast<int>(std::lround(stabilized_tip->filtered.x)),
                                                  0, frame->color_width - 1);
                const int filtered_y = std::clamp(static_cast<int>(std::lround(stabilized_tip->filtered.y)),
                                                  0, frame->color_height - 1);
                const auto raw_tip_depth_mm = aerial_touch::sample_depth_median_mm(
                    frame->raw_depth, frame->depth_width, frame->depth_height, raw_x, raw_y,
                    config.depth.sample_radius, frame->depth_unit_mm);
                // Landmark 8 sits on the finger's silhouette edge, where the depth ROI straddles
                // finger and table. Probe back along the finger towards the DIP joint instead,
                // where the finger is solid, and extrapolate the depth gradient forward to the tip.
                // The tip *pixel* still decides the key; only the range comes from the probes.
                const aerial_touch::Vec2 filtered_tip{ static_cast<float>(filtered_x),
                                                       static_cast<float>(filtered_y) };
                aerial_touch::Vec2 joint_pixel = filtered_tip;
                if(hand.landmark_count > kIndexFingerDip && std::isfinite(hand.landmarks[kIndexFingerDip].x)
                   && std::isfinite(hand.landmarks[kIndexFingerDip].y)) {
                    // Take the tip->joint offset from the raw landmarks but anchor it on the
                    // filtered tip, so the probe is as steady as the point it is derived from.
                    const float joint_x = hand.landmarks[kIndexFingerDip].x
                                          * static_cast<float>(frame->color_width);
                    const float joint_y = hand.landmarks[kIndexFingerDip].y
                                          * static_cast<float>(frame->color_height);
                    joint_pixel = { filtered_tip.x + (joint_x - stabilized_tip->raw.x),
                                    filtered_tip.y + (joint_y - stabilized_tip->raw.y) };
                }

                const aerial_touch::FingertipDepthProbeConfig probe_config{
                    config.fingertip.depth_probe_near_ratio, config.fingertip.depth_probe_far_ratio, 40.0F, 4.0F
                };
                const auto probe = aerial_touch::estimate_fingertip_depth_mm(
                    filtered_tip, joint_pixel, probe_config,
                    [&](const aerial_touch::Vec2 pixel) -> std::optional<float> {
                        const int probe_x = static_cast<int>(std::lround(pixel.x));
                        const int probe_y = static_cast<int>(std::lround(pixel.y));
                        // Reject rather than clamp: a hand at the edge of the frame can push the
                        // joint probe off-image, and clamping would silently substitute a border
                        // pixel's depth -- some unrelated part of the scene -- for the finger's.
                        if(probe_x < 0 || probe_y < 0 || probe_x >= frame->depth_width
                           || probe_y >= frame->depth_height) {
                            return std::nullopt;
                        }
                        return aerial_touch::sample_depth_median_mm(
                            frame->depth, frame->depth_width, frame->depth_height, probe_x, probe_y,
                            config.depth.sample_radius, frame->depth_unit_mm);
                    });
                const std::optional<float> sdk_depth_mm =
                    probe.has_value() ? std::optional<float>{ probe->depth_mm } : std::nullopt;
                tip_depth_probed = probe.has_value();
                tip_depth_extrapolated = probe.has_value() && probe->extrapolated;

                // Gate liveness on the unfiltered depth at the probe location, not at the tip. The
                // tip pixel is exactly where the sensor drops out (silhouette edge), so gating there
                // would keep resetting the stabiliser on frames whose probe depth was perfectly good.
                std::optional<float> raw_depth_mm = raw_tip_depth_mm;
                if(probe.has_value()) {
                    const int gate_x = std::clamp(static_cast<int>(std::lround(probe->near_pixel.x)), 0,
                                                  frame->depth_width - 1);
                    const int gate_y = std::clamp(static_cast<int>(std::lround(probe->near_pixel.y)), 0,
                                                  frame->depth_height - 1);
                    raw_depth_mm = aerial_touch::sample_depth_median_mm(
                        frame->raw_depth, frame->depth_width, frame->depth_height, gate_x, gate_y,
                        config.depth.sample_radius, frame->depth_unit_mm);
                }
                if(raw_depth_freshness.update(raw_depth_mm)) {
                    const auto stable_depth_mm = depth_stabilizer.update(sdk_depth_mm);
                    if(stable_depth_mm.has_value()) {
                        tip_depth_mm = stable_depth_mm;
                        current_xyz = camera.deproject(*frame, filtered_tip, *stable_depth_mm);
                    }
                }
                else {
                    depth_stabilizer.reset();
                    camera.reset_depth_filters();
                }
                // The HUD's "原始" distance stays a genuine unfiltered reading at the tip, so it is
                // still a useful comparison against the probed and filtered value beside it.
                if(raw_tip_depth_mm.has_value()) {
                    raw_xyz = camera.deproject(*frame, { static_cast<float>(raw_x), static_cast<float>(raw_y) },
                                               *raw_tip_depth_mm);
                }
            }
            else {
                static_cast<void>(depth_stabilizer.update(std::nullopt));
                if(last_confirmed_timestamp_ms.has_value()
                   && frame->timestamp_ms - *last_confirmed_timestamp_ms > config.touch.tracking_timeout_ms) {
                    fingertip_stabilizer.reset();
                    depth_stabilizer.reset();
                    raw_depth_freshness.reset();
                    camera.reset_depth_filters();
                    last_confirmed_timestamp_ms.reset();
                    if(collecting_calibration_samples) {
                        calibration_collector.clear();
                        collecting_calibration_samples = false;
                        status = u8"校正取樣中斷：追蹤逾時，請按空白鍵重試";
                    }
                }
            }

            if(scanning_surface) {
                std::vector<aerial_touch::Vec3> samples;
                if(confirmed_tip && filtered_tip_pixel.has_value() && tip_depth_mm.has_value()) {
                    samples = surface_patch_samples(camera, *frame, hand, *filtered_tip_pixel, *tip_depth_mm,
                                                    config.depth);
                }
                const auto progress = surface_scan.add(samples, frame->timestamp_ms);
                if(progress.state == aerial_touch::SurfaceScanState::Complete) {
                    calibration_surface = surface_scan.surface_plane();
                    scanning_surface = false;
                    status = u8"表面掃描完成；將手指移到 1 鍵左上角並按空白鍵開始取樣";
                }
                else if(progress.state == aerial_touch::SurfaceScanState::Failed) {
                    calibration_surface.reset();
                    scanning_surface = false;
                    status = u8"表面掃描失敗：深度不足、混入不同平面，或掃過的範圍太窄；"
                             u8"請把整個鍵盤區域都掃到，尤其是四個角落";
                }
                else {
                    std::ostringstream scan_text;
                    scan_text << std::fixed << std::setprecision(0) << u8"表面掃描中：已收集 "
                              << progress.sample_count << u8" 個深度樣本，涵蓋範圍 "
                              << progress.quality.minor_extent_mm << " / "
                              << aerial_touch::SurfacePlaneFitConfig{}.minimum_extent_mm
                              << u8" mm；請沿鍵盤區域移動食指";
                    status = scan_text.str();
                }
            }

            if(collecting_calibration_samples && calibration_surface.has_value() && confirmed_tip
               && filtered_tip_pixel.has_value()) {
                const auto surface_point = surface_point_at_fingertip_pixel(
                    camera, *frame, *filtered_tip_pixel, *calibration_surface);
                if(surface_point.has_value()) {
                    calibration_collector.add(*surface_point);
                }
                status = std::string(u8"校正點取樣：") + std::to_string(calibration_collector.sample_count()) + "/"
                         + std::to_string(config.calibration.required_samples) + u8"，請保持不動";
                const auto sample_result = calibration_collector.result();
                if(sample_result.has_value()) {
                    calibration_points.push_back(sample_result->point);
                    calibration_spreads.push_back(sample_result->spread);
                    calibration_collector.clear();
                    collecting_calibration_samples = false;
                    static constexpr std::array<const char*, aerial_touch::kKeypadCalibrationPointCount> names{
                        u8"1 鍵左上角", u8"3 鍵右上角", u8"0 鍵正下方",
                    };
                    static constexpr std::array<const char*, aerial_touch::kKeypadCalibrationPointCount - 1U>
                        next_instructions{
                            u8"請移到 3 鍵右上角並按空白鍵開始取樣",
                            u8"請移到 0 鍵正下方並按空白鍵開始取樣",
                        };
                    status = std::string(u8"已記錄 ") + names[calibration_points.size() - 1U];
                    if(calibration_points.size() < aerial_touch::kKeypadCalibrationPointCount) {
                        status += std::string(u8"；") + next_instructions[calibration_points.size() - 1U];
                    }
                    else {
                        status += u8"；請按 Enter 計算鍵盤尺寸並完成校正";
                    }
                }
                else if(calibration_collector.sample_count() >= 20U) {
                    calibration_collector.clear();
                    collecting_calibration_samples = false;
                    status = u8"校正點散布過大或有效樣本不足，請保持手指不動後按空白鍵重試";
                }
            }
            else if(collecting_calibration_samples && calibration_surface.has_value()) {
                status = u8"等待有效指尖位置與表面射線後開始收集，請保持手指不動";
            }

            if(current_xyz.has_value() && plane.has_value() && keypad.has_value() && !calibrating && confirmed_tip) {
                const auto projected = plane->project(*current_xyz);
                if(std::isfinite(projected.u_mm) && std::isfinite(projected.v_mm)
                   && std::isfinite(projected.signed_distance_mm)) {
                    current_uv = aerial_touch::Vec2{ projected.u_mm, projected.v_mm };
                    current_distance = projected.signed_distance_mm;
                    sticky_key = keypad->key_at(*current_uv, sticky_key, config.keypad.boundary_hysteresis_mm);
                    current_key = sticky_key;
                }
                if(raw_xyz.has_value()) {
                    const auto raw_projected = plane->project(*raw_xyz);
                    if(std::isfinite(raw_projected.signed_distance_mm)) {
                        raw_distance = raw_projected.signed_distance_mm;
                    }
                }
            }
            if(current_xyz.has_value() && keypad.has_value() && current_uv.has_value() && current_distance.has_value()
               && !calibrating && confirmed_tip) {
                const auto event = touch.update({ frame->timestamp_ms, *current_distance, current_key,
                                                  *current_xyz, *current_uv });
                if(event.has_value()) {
                    last_event = event;
                    active_pressed_key = event->key;
                    std::cout << u8"按鍵事件 按鍵=" << event->key << u8" 時間戳記毫秒=" << event->timestamp_ms
                              << u8" 指尖XYZ毫米=(" << vec3_text(event->fingertip_xyz_mm) << u8") 平面UV毫米=("
                              << vec2_text(event->plane_uv_mm) << u8") 原始距離毫米="
                              << (raw_distance.has_value() ? std::to_string(*raw_distance) : "N/A")
                              << u8" 濾波距離毫米=" << *current_distance << "\n";
                }
                if(touch.armed()) {
                    active_pressed_key.reset();
                }
            }
            else {
                touch.mark_tracking_lost(frame->timestamp_ms);
                sticky_key.reset();
                active_pressed_key.reset();
            }

            ++fps_frames;
            const auto now = std::chrono::steady_clock::now();
            const float fps_elapsed = std::chrono::duration<float>(now - fps_start).count();
            if(fps_elapsed >= 0.5F) {
                fps = static_cast<float>(fps_frames) / fps_elapsed;
                fps_frames = 0;
                fps_start = now;
            }

            active_pressed_key = aerial_touch::currently_pressed_key(
                active_pressed_key, touch.armed(), current_uv.has_value() && confirmed_tip, calibrating, current_key);
            const std::optional<std::string> pressed_key = active_pressed_key;
            if(plane.has_value() && keypad.has_value() && !calibrating) {
                draw_table_keypad(display, camera, *frame, *plane, *keypad, current_key, pressed_key);
            }
            draw_keypad(display, keypad.has_value(), current_key, pressed_key);
            {
                aerial_touch::Utf8TextCanvas canvas(display);
                text_line(canvas, std::string(u8"主程式 FPS：") + std::to_string(static_cast<int>(std::lround(fps)))
                                      + u8" | 相機：" + std::to_string(camera_info.fps) + " FPS "
                                      + camera_info.depth_work_mode + " " + camera_info.depth_precision
                                      + u8" | 對齊：" + (camera.hardware_alignment() ? u8"硬體 D2C" : u8"軟體 D2C"),
                          0);
                text_line(canvas,
                          std::string(u8"追蹤：")
                              + (hand_tracker.available() ? (confirmed_tip ? u8"本幀有效" : u8"未取得本幀觀測")
                                                          : u8"無法使用"),
                          1, hand_tracker.available() ? cv::Scalar{ 100, 255, 100 } : cv::Scalar{ 80, 80, 255 });
                if(fingertip_pixel_text.has_value()) {
                    text_line(canvas, *fingertip_pixel_text, 2);
                }
                if(current_xyz.has_value()) {
                    text_line(canvas, std::string("XYZ: ") + vec3_text(*current_xyz), 3);
                }
                if(current_uv.has_value() && current_distance.has_value()) {
                    text_line(canvas, std::string(u8"平面 UV：") + vec2_text(*current_uv), 4);
                    text_line(canvas, std::string(u8"距離：")
                                          + std::to_string(static_cast<int>(std::lround(*current_distance)))
                                          + u8" mm（濾波） | 原始："
                                          + (raw_distance.has_value()
                                                 ? std::to_string(static_cast<int>(std::lround(*raw_distance))) + " mm"
                                                 : "-")
                                          + u8" | 按鍵：" + current_key.value_or("-"),
                              5);
                }
                std::string touch_text = std::string(u8"觸控：")
                                         + (touch.armed() ? u8"可觸發" : u8"等待手指離開");
                if(touch.approach_latched()) {
                    touch_text += u8" | 接近速度已達標";
                }
                else if(config.touch.dwell_ms > 0) {
                    const auto dwell = touch.dwell_elapsed_ms(frame->timestamp_ms);
                    if(dwell.has_value()) {
                        touch_text += u8" | 停留 " + std::to_string(*dwell) + "/"
                                      + std::to_string(config.touch.dwell_ms) + " ms";
                    }
                }
                // Three states, not two: saying "single-point probe" when no probe succeeded would
                // assert a measurement that never happened.
                if(confirmed_tip && tip_depth_probed) {
                    touch_text += tip_depth_extrapolated ? u8" | 深度：指節外推" : u8" | 深度：單點探測";
                }
                else if(confirmed_tip) {
                    touch_text += u8" | 深度：探測失敗";
                }
                text_line(canvas, touch_text, 6);
                const std::string calibration_state = !calibrating
                                                          ? (plane ? u8"完成 " : u8"尚未設定 ")
                                                          : scanning_surface ? u8"表面掃描中 "
                                                                             : calibration_surface.has_value()
                                                                                   ? u8"進行中 "
                                                                                   : u8"等待表面掃描 ";
                text_line(canvas, std::string(u8"校正：") + calibration_state
                                      + std::to_string(calibration_points.size()) + "/"
                                      + std::to_string(aerial_touch::kKeypadCalibrationPointCount),
                          7);
                text_line(canvas, std::string(u8"狀態：") + status, 8, { 80, 230, 255 });
                const auto& surface_progress = surface_scan.progress();
                if(calibrating && (scanning_surface || calibration_surface.has_value())) {
                    std::ostringstream surface_text;
                    // The spread matters as much as the residual: a sweep confined to a narrow band
                    // fits some plane perfectly while leaving its tilt undetermined, so the operator
                    // needs to watch this number grow, not just the RMS.
                    surface_text << std::fixed << std::setprecision(1) << u8"表面："
                                 << (scanning_surface ? u8"掃描中" : u8"已建立") << u8"；樣本 "
                                 << surface_progress.sample_count << u8"；內點 "
                                 << surface_progress.quality.inlier_samples << u8"；RMS "
                                 << surface_progress.quality.rms_residual_mm << u8" mm；範圍 "
                                 << surface_progress.quality.minor_extent_mm << " / "
                                 << aerial_touch::SurfacePlaneFitConfig{}.minimum_extent_mm << " mm";
                    text_line(canvas, surface_text.str(), 9, { 180, 220, 255 });
                }
                text_line(canvas, u8"C：校正 | S：參數 | 空白鍵：開始掃描或取樣 | Enter：完成校正 | R：重設 | Q/Esc：離開", 10);
                if(!camera_info.warnings.empty()) {
                    text_line(canvas, std::string(u8"相機警告：") + camera_info.warnings.back(), 11,
                              { 80, 180, 255 });
                }
                if(last_event.has_value()) {
                    text_line(canvas, std::string(u8"最近按鍵：") + last_event->key, 12, { 50, 255, 255 });
                }
                // One compact line instead of one row per point: at 480p the old layout started at
                // row 13 (y = 452) and every spread readout fell off the bottom of the frame, so the
                // operator never saw the numbers that tell them whether a point was steady enough.
                if(!calibration_spreads.empty()) {
                    std::ostringstream spread_text;
                    spread_text << std::fixed << std::setprecision(1) << u8"校正散布（最大軸）：";
                    for(std::size_t index = 0; index < calibration_spreads.size(); ++index) {
                        const auto& spread = calibration_spreads[index];
                        const float widest = std::max({ spread.x, spread.y, spread.z });
                        spread_text << (index == 0U ? "" : " / ") << "P" << (index + 1U) << " " << widest;
                    }
                    spread_text << " mm";
                    text_line(canvas, spread_text.str(), 13, { 180, 220, 255 });
                }
            }
            settings_window.update_preview({ current_distance, current_uv.has_value() && confirmed_tip,
                                             current_key, touch.armed() });
            cv::imshow(kWindowName, display);

            const int key = cv::waitKey(1);
            if(key == 'q' || key == 'Q' || key == 27) {
                break;
            }
            if(key == 's' || key == 'S') {
                toggle_settings_window();
            }
            else if(key == 'c' || key == 'C') {
                calibrating = true;
                calibration_points.clear();
                calibration_spreads.clear();
                calibration_collector.clear();
                collecting_calibration_samples = false;
                scanning_surface = false;
                surface_scan.clear();
                calibration_surface.reset();
                plane.reset();
                keypad.reset();
                touch = aerial_touch::TouchStateMachine(config.touch);
                sticky_key.reset();
                active_pressed_key.reset();
                status = u8"先沿預計的鍵盤區域移動食指，按空白鍵開始表面掃描";
            }
            else if(key == ' ' && calibrating) {
                if(scanning_surface) {
                    status = u8"表面掃描正在進行，請沿鍵盤區域移動食指";
                }
                else if(!calibration_surface.has_value()) {
                    surface_scan.begin(frame->timestamp_ms);
                    scanning_surface = true;
                    status = u8"表面掃描中：請沿鍵盤區域移動食指，完成後會自動開始 P1 取樣";
                }
                else if(collecting_calibration_samples) {
                    status = u8"校正點正在取樣，請保持手指不動";
                }
                else if(calibration_points.size() < aerial_touch::kKeypadCalibrationPointCount) {
                    calibration_collector.clear();
                    collecting_calibration_samples = true;
                    status = confirmed_tip ? u8"開始收集校正樣本，請保持手指不動"
                                           : u8"等待有效指尖位置後開始收集，請保持手指不動";
                }
            }
            else if(key == 13 && calibrating) {
                if(scanning_surface) {
                    status = u8"表面掃描仍在進行，請等待自動完成";
                }
                else if(collecting_calibration_samples) {
                    status = u8"校正點仍在取樣，請保持手指不動";
                }
                else if(calibration_points.size() != aerial_touch::kKeypadCalibrationPointCount
                        || !calibration_surface.has_value()) {
                    status = u8"請先完成表面掃描，再依序記錄 3 個鍵盤邊界校正點";
                }
                else {
                    std::array<aerial_touch::Vec3, aerial_touch::kKeypadCalibrationPointCount> points{};
                    std::copy(calibration_points.begin(), calibration_points.end(), points.begin());
                    const auto attempt = aerial_touch::calibrate_keypad_detailed(
                        points, config.calibration.minimum_point_distance_mm);
                    if(attempt.result.has_value()) {
                        plane = attempt.result->plane;
                        keypad.emplace(attempt.result->geometry);
                        calibrating = false;
                        std::ostringstream geometry_status;
                        geometry_status << std::fixed << std::setprecision(1)
                                        << u8"校正完成：鍵盤 " << attempt.result->geometry.total_width_mm << u8" × "
                                        << attempt.result->geometry.total_height_mm << u8" mm；按鍵 "
                                        << attempt.result->geometry.key_width_mm << u8" × "
                                        << attempt.result->geometry.key_height_mm << u8" mm（按鍵相連無間隙）";
                        status = geometry_status.str();
                    }
                    else {
                        calibration_points.clear();
                        calibration_spreads.clear();
                        calibration_collector.clear();
                        keypad.reset();
                        status = calibration_failure_text(attempt.failure)
                                 + u8"；表面基準已保留，請從 1 鍵左上角重新取樣";
                    }
                }
            }
            else if(key == 'r' || key == 'R') {
                calibrating = false;
                calibration_points.clear();
                calibration_spreads.clear();
                calibration_collector.clear();
                collecting_calibration_samples = false;
                scanning_surface = false;
                surface_scan.clear();
                calibration_surface.reset();
                plane.reset();
                keypad.reset();
                touch = aerial_touch::TouchStateMachine(config.touch);
                fingertip_stabilizer.reset();
                depth_stabilizer.reset();
                raw_depth_freshness.reset();
                camera.reset_depth_filters();
                sticky_key.reset();
                active_pressed_key.reset();
                status = u8"重設完成；按 C 重新設定數字鍵盤範圍";
            }
        }
        camera.stop();
        cv::destroyAllWindows();
        return 0;
    }
    catch(const std::exception& error) {
        std::cerr << u8"嚴重錯誤：" << error.what() << '\n';
        return 1;
    }
}
