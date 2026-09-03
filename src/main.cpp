#include "aerial_touch/app_config.hpp"
#include "aerial_touch/calibration_geometry.hpp"
#include "aerial_touch/calibration_sampler.hpp"
#include "aerial_touch/depth_sampler.hpp"
#include "aerial_touch/hand_tracker.hpp"
#include "aerial_touch/keypad.hpp"
#include "aerial_touch/keypad_overlay.hpp"
#include "aerial_touch/orbbec_camera.hpp"
#include "aerial_touch/plane.hpp"
#include "aerial_touch/settings_window.hpp"
#include "aerial_touch/signal_stabilizer.hpp"
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
constexpr const char* kWindowName = "aerial_touch_window";

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
        std::optional<aerial_touch::Plane> calibration_plane;
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
                cv::circle(display, { filtered_x, filtered_y }, 5,
                           stabilized_tip->confirmed_this_frame ? cv::Scalar{ 255, 120, 20 }
                                                                  : cv::Scalar{ 160, 160, 160 },
                           2, cv::LINE_AA);
                fingertip_pixel_text = std::string(u8"指尖像素（濾波）：") + std::to_string(filtered_x) + ", "
                                        + std::to_string(filtered_y)
                                        + (stabilized_tip->confirmed_this_frame ? "" : u8"（僅保留顯示）");
            }

            std::optional<aerial_touch::Vec3> raw_xyz;
            const bool confirmed_tip = stabilized_tip.has_value() && stabilized_tip->confirmed_this_frame;
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
                const auto raw_depth_mm = aerial_touch::sample_depth_median_mm(
                    frame->raw_depth, frame->depth_width, frame->depth_height, raw_x, raw_y,
                    config.depth.sample_radius, frame->depth_unit_mm);
                const auto sdk_depth_mm = aerial_touch::sample_depth_median_mm(
                    frame->depth, frame->depth_width, frame->depth_height, filtered_x, filtered_y,
                    config.depth.sample_radius, frame->depth_unit_mm);
                if(raw_depth_freshness.update(raw_depth_mm)) {
                    const auto stable_depth_mm = depth_stabilizer.update(sdk_depth_mm);
                    if(stable_depth_mm.has_value()) {
                        current_xyz = camera.deproject(
                            *frame, { static_cast<float>(filtered_x), static_cast<float>(filtered_y) },
                            *stable_depth_mm);
                    }
                }
                else {
                    depth_stabilizer.reset();
                    camera.reset_depth_filters();
                }
                if(raw_depth_mm.has_value()) {
                    raw_xyz = camera.deproject(*frame, { static_cast<float>(raw_x), static_cast<float>(raw_y) },
                                               *raw_depth_mm);
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

            if(collecting_calibration_samples && current_xyz.has_value()) {
                calibration_collector.add(*current_xyz);
                status = std::string(u8"校正點取樣：") + std::to_string(calibration_collector.sample_count()) + "/"
                         + std::to_string(config.calibration.required_samples) + u8"，請保持不動";
                const auto sample_result = calibration_collector.result();
                if(sample_result.has_value()) {
                    calibration_points.push_back(sample_result->point);
                    calibration_spreads.push_back(sample_result->spread);
                    calibration_collector.clear();
                    collecting_calibration_samples = false;
                    static constexpr std::array<const char*, 7> names{
                        u8"1 鍵左上角", u8"3 鍵右上角", u8"0 鍵正下方",
                        u8"1 鍵右上角", u8"2 鍵右上角", u8"1 鍵左下角", u8"4 鍵左下角",
                    };
                    static constexpr std::array<const char*, 6> next_instructions{
                        u8"請移到 3 鍵右上角並按空白鍵開始取樣",
                        u8"請移到 0 鍵正下方並按空白鍵開始取樣",
                        u8"請移到 1 鍵右上角並按空白鍵開始取樣",
                        u8"請移到 2 鍵右上角並按空白鍵開始取樣",
                        u8"請移到 1 鍵左下角並按空白鍵開始取樣",
                        u8"請移到 4 鍵左下角並按空白鍵開始取樣",
                    };
                    status = std::string(u8"已記錄 ") + names[calibration_points.size() - 1U];
                    if(calibration_points.size() == 3U) {
                        calibration_plane = aerial_touch::Plane::from_calibration_points(
                            calibration_points[0], calibration_points[1], calibration_points[2],
                            config.calibration.minimum_point_distance_mm);
                        if(!calibration_plane.has_value()) {
                            calibration_points.clear();
                            calibration_spreads.clear();
                            status = u8"校正失敗：前三個位置太近或接近直線；請從 1 鍵左上角重新取樣";
                        }
                        else {
                            status += std::string(u8"；平面基準已建立；") + next_instructions[2];
                        }
                    }
                    else if(calibration_points.size() < 7U) {
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
                text_line(canvas, std::string(u8"觸控：") + (touch.armed() ? u8"可觸發" : u8"等待手指離開"), 6);
                text_line(canvas,
                          std::string(u8"校正：") + (calibrating ? u8"進行中 " : (plane ? u8"完成 " : u8"尚未設定 "))
                              + std::to_string(calibration_points.size()) + "/7",
                          7);
                text_line(canvas, std::string(u8"狀態：") + status, 8, { 80, 230, 255 });
                text_line(canvas, u8"C：校正 | S：參數 | 空白鍵：開始取樣 | Enter：完成校正 | R：重設 | Q/Esc：離開", 9);
                if(!camera_info.warnings.empty()) {
                    text_line(canvas, std::string(u8"相機警告：") + camera_info.warnings.back(), 10,
                              { 80, 180, 255 });
                }
                if(last_event.has_value()) {
                    text_line(canvas, std::string(u8"最近按鍵：") + last_event->key, 11, { 50, 255, 255 });
                }
                for(std::size_t index = 0; index < calibration_spreads.size(); ++index) {
                    text_line(canvas,
                              std::string(u8"校正點 ") + std::to_string(index + 1U) + u8" 散布 XYZ："
                                  + vec3_text(calibration_spreads[index]),
                              12 + static_cast<int>(index), { 180, 220, 255 });
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
                plane.reset();
                calibration_plane.reset();
                keypad.reset();
                touch = aerial_touch::TouchStateMachine(config.touch);
                sticky_key.reset();
                active_pressed_key.reset();
                status = u8"將手指移到 1 鍵左上角，保持不動後按空白鍵開始取樣";
            }
            else if(key == ' ' && calibrating) {
                if(collecting_calibration_samples) {
                    status = u8"校正點正在取樣，請保持手指不動";
                }
                else if(calibration_points.size() < 7U) {
                    calibration_collector.clear();
                    collecting_calibration_samples = true;
                    status = current_xyz.has_value() ? u8"開始收集校正樣本，請保持手指不動"
                                                     : u8"等待有效指尖深度後開始收集，請保持手指不動";
                }
            }
            else if(key == 13 && calibrating) {
                if(collecting_calibration_samples) {
                    status = u8"校正點仍在取樣，請保持手指不動";
                }
                else if(calibration_points.size() != 7U || !calibration_plane.has_value()) {
                    status = u8"請先依序記錄 7 個鍵盤邊界校正點";
                }
                else {
                    std::array<aerial_touch::Vec3, 7> points{};
                    std::copy(calibration_points.begin(), calibration_points.end(), points.begin());
                    const auto result = aerial_touch::calibrate_keypad(
                        points, config.calibration.minimum_point_distance_mm);
                    if(result.has_value()) {
                        plane = result->plane;
                        keypad.emplace(result->geometry);
                        calibration_plane.reset();
                        calibrating = false;
                        std::ostringstream geometry_status;
                        geometry_status << std::fixed << std::setprecision(1)
                                        << u8"校正完成：鍵盤 " << result->geometry.total_width_mm << u8" × "
                                        << result->geometry.total_height_mm << u8" mm；按鍵 "
                                        << result->geometry.key_width_mm << u8" × "
                                        << result->geometry.key_height_mm << u8" mm；水平間距 "
                                        << result->geometry.horizontal_gap_mm << u8" mm；垂直間距 "
                                        << result->geometry.vertical_gap_mm << u8" mm";
                        status = geometry_status.str();
                    }
                    else {
                        calibration_points.clear();
                        calibration_spreads.clear();
                        calibration_collector.clear();
                        calibration_plane.reset();
                        keypad.reset();
                        status = u8"校正失敗：請確認第 3 點在 0 鍵正下方而非右下角，且第 4～7 點都是格線角點；請從 1 鍵左上角重新取樣";
                    }
                }
            }
            else if(key == 'r' || key == 'R') {
                calibrating = false;
                calibration_points.clear();
                calibration_spreads.clear();
                calibration_collector.clear();
                collecting_calibration_samples = false;
                plane.reset();
                calibration_plane.reset();
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
