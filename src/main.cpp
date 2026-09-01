#include "aerial_touch/app_config.hpp"
#include "aerial_touch/depth_sampler.hpp"
#include "aerial_touch/hand_tracker.hpp"
#include "aerial_touch/keypad.hpp"
#include "aerial_touch/orbbec_camera.hpp"
#include "aerial_touch/plane.hpp"
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
    canvas.draw(text, { 12, 10 + row * 24 }, color);
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

void draw_keypad(cv::Mat& image, const aerial_touch::Keypad& keypad, const std::optional<std::string>& hit) {
    constexpr float scale = 1.1F;
    const int origin_x = std::max(0, image.cols - 150);
    const int origin_y = std::max(0, image.rows - 180);
    for(const auto& region : keypad.regions()) {
        const cv::Rect rect{ origin_x + static_cast<int>(region.u_min_mm * scale),
                             origin_y + static_cast<int>(region.v_min_mm * scale),
                             std::max(1, static_cast<int>((region.u_max_mm - region.u_min_mm) * scale)),
                             std::max(1, static_cast<int>((region.v_max_mm - region.v_min_mm) * scale)) };
        const bool active = hit.has_value() && *hit == region.key;
        cv::rectangle(image, rect, active ? cv::Scalar{ 20, 220, 255 } : cv::Scalar{ 230, 230, 230 }, active ? -1 : 2);
        cv::putText(image, region.key, { rect.x + rect.width / 3, rect.y + 2 * rect.height / 3 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, active ? cv::Scalar{ 0, 0, 0 } : cv::Scalar{ 255, 255, 255 }, 2,
                    cv::LINE_AA);
    }
}

}  // namespace

int main(int argc, char** argv) {
    aerial_touch::enable_utf8_console();
    try {
        const CliOptions options = parse_options(argc, argv);
        const auto config = aerial_touch::load_app_config(options.config);
        aerial_touch::Keypad keypad(config.keypad);
        aerial_touch::TouchStateMachine touch(config.touch);
        aerial_touch::HandTracker hand_tracker(options.bridge, options.model);
        aerial_touch::OrbbecCamera camera;

        if(!camera.start()) {
            std::cerr << camera.error() << '\n';
            return 2;
        }
        std::cout << u8"Orbbec 對齊模式：" << (camera.hardware_alignment() ? u8"硬體 D2C" : u8"軟體 D2C") << '\n';
        if(!hand_tracker.available()) {
            std::cerr << hand_tracker.error() << '\n';
        }

        std::optional<aerial_touch::Plane> plane;
        std::vector<aerial_touch::Vec3> calibration_points;
        std::optional<aerial_touch::Vec3> current_xyz;
        std::optional<aerial_touch::Vec2> current_uv;
        std::optional<float> current_distance;
        std::optional<std::string> current_key;
        std::optional<aerial_touch::PressEvent> last_event;
        bool calibrating = false;
        std::string status = hand_tracker.available() ? u8"按 C 開始設定數字鍵盤範圍" : hand_tracker.error();
        auto fps_start = std::chrono::steady_clock::now();
        int fps_frames = 0;
        float fps = 0.0F;

        cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
        aerial_touch::set_utf8_window_title(kWindowName, u8"Gemini 2 空中鍵盤");
        for(;;) {
            const auto frame = camera.capture(100);
            if(!frame.has_value()) {
                const int key = cv::waitKey(1);
                if(key == 'q' || key == 'Q' || key == 27) {
                    break;
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
            current_key.reset();
            std::optional<std::string> fingertip_pixel_text;
            if(hand.detected && hand.landmark_count > kIndexFingerTip) {
                const int pixel_x = std::clamp(static_cast<int>(std::lround(hand.landmarks[kIndexFingerTip].x * frame->color_width)),
                                               0, frame->color_width - 1);
                const int pixel_y = std::clamp(static_cast<int>(std::lround(hand.landmarks[kIndexFingerTip].y * frame->color_height)),
                                               0, frame->color_height - 1);
                const auto depth_mm = aerial_touch::sample_depth_median_mm(frame->depth, frame->depth_width, frame->depth_height,
                                                                           pixel_x, pixel_y, config.depth.sample_radius,
                                                                           frame->depth_unit_mm);
                if(depth_mm.has_value()) {
                    current_xyz = camera.deproject(*frame, { static_cast<float>(pixel_x), static_cast<float>(pixel_y) }, *depth_mm);
                }
                fingertip_pixel_text = std::string(u8"指尖像素：") + std::to_string(pixel_x) + ", "
                                        + std::to_string(pixel_y);
            }

            if(current_xyz.has_value() && plane.has_value() && !calibrating) {
                const auto projected = plane->project(*current_xyz);
                current_uv = aerial_touch::Vec2{ projected.u_mm, projected.v_mm };
                current_distance = projected.signed_distance_mm;
                current_key = keypad.key_at(*current_uv);
                const auto event = touch.update({ frame->timestamp_ms, projected.signed_distance_mm, current_key,
                                                  *current_xyz, *current_uv });
                if(event.has_value()) {
                    last_event = event;
                    std::cout << u8"按鍵事件 按鍵=" << event->key << u8" 時間戳記毫秒=" << event->timestamp_ms
                              << u8" 指尖XYZ毫米=(" << vec3_text(event->fingertip_xyz_mm) << u8") 平面UV毫米=("
                              << vec2_text(event->plane_uv_mm) << ")\n";
                }
            }
            else {
                touch.mark_tracking_lost(frame->timestamp_ms);
            }

            ++fps_frames;
            const auto now = std::chrono::steady_clock::now();
            const float fps_elapsed = std::chrono::duration<float>(now - fps_start).count();
            if(fps_elapsed >= 0.5F) {
                fps = static_cast<float>(fps_frames) / fps_elapsed;
                fps_frames = 0;
                fps_start = now;
            }

            draw_keypad(display, keypad, current_key);
            {
                aerial_touch::Utf8TextCanvas canvas(display);
                text_line(canvas, std::string("FPS: ") + std::to_string(static_cast<int>(std::lround(fps)))
                                      + u8" | 對齊：" + (camera.hardware_alignment() ? u8"硬體 D2C" : u8"軟體 D2C"),
                          0);
                text_line(canvas,
                          std::string(u8"追蹤：")
                              + (hand_tracker.available() ? (hand.detected ? u8"偵測到手部" : u8"未偵測到手部")
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
                                          + u8" mm | 按鍵：" + current_key.value_or("-"),
                              5);
                }
                text_line(canvas, std::string(u8"觸控：") + (touch.armed() ? u8"可觸發" : u8"等待手指離開"), 6);
                text_line(canvas,
                          std::string(u8"校正：") + (calibrating ? u8"進行中 " : (plane ? u8"完成 " : u8"尚未設定 "))
                              + std::to_string(calibration_points.size()) + "/3",
                          7);
                text_line(canvas, std::string(u8"狀態：") + status, 8, { 80, 230, 255 });
                text_line(canvas, u8"C：校正 | 空白鍵：擷取 | Enter：完成校正 | R：重設 | Q/Esc：離開", 9);
                if(last_event.has_value()) {
                    text_line(canvas, std::string(u8"最近按鍵：") + last_event->key, 10, { 50, 255, 255 });
                }
            }
            cv::imshow(kWindowName, display);

            const int key = cv::waitKey(1);
            if(key == 'q' || key == 'Q' || key == 27) {
                break;
            }
            if(key == 'c' || key == 'C') {
                calibrating = true;
                calibration_points.clear();
                plane.reset();
                touch = aerial_touch::TouchStateMachine(config.touch);
                status = u8"將手指移到 1 鍵左上角，然後按空白鍵";
            }
            else if(key == ' ' && calibrating) {
                if(!current_xyz.has_value()) {
                    status = u8"無法擷取：指尖深度資料無效";
                }
                else if(calibration_points.size() < 3U) {
                    calibration_points.push_back(*current_xyz);
                    static constexpr std::array<const char*, 3> names{
                        u8"1 鍵左上角",
                        u8"3 鍵右側位置",
                        u8"0 鍵下方位置",
                    };
                    static constexpr std::array<const char*, 2> next_instructions{
                        u8"請移到 3 鍵右側位置並按空白鍵",
                        u8"請移到 0 鍵下方位置並按空白鍵",
                    };
                    status = std::string(u8"已記錄 ") + names[calibration_points.size() - 1U];
                    if(calibration_points.size() < 3U) {
                        status += std::string(u8"；接著") + next_instructions[calibration_points.size() - 1U];
                    }
                    else {
                        status += u8"；請按 Enter 完成校正";
                    }
                }
            }
            else if(key == 13 && calibrating) {
                if(calibration_points.size() != 3U) {
                    status = u8"請先依序記錄 1 鍵左上角、3 鍵右側位置與 0 鍵下方位置";
                }
                else {
                    plane = aerial_touch::Plane::from_calibration_points(calibration_points[0], calibration_points[1],
                                                                          calibration_points[2],
                                                                          config.calibration.minimum_point_distance_mm);
                    if(plane.has_value()) {
                        calibrating = false;
                        status = u8"校正完成";
                    }
                    else {
                        status = u8"校正失敗：三個位置距離太近或幾乎位於同一直線";
                    }
                }
            }
            else if(key == 'r' || key == 'R') {
                calibrating = false;
                calibration_points.clear();
                plane.reset();
                touch = aerial_touch::TouchStateMachine(config.touch);
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
