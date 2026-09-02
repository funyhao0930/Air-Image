#pragma once

#include <array>
#include <optional>
#include <string>

namespace aerial_touch {

struct KeypadOverlayRegion {
    std::string key;
    int x_px{};
    int y_px{};
    int width_px{};
    int height_px{};
};

struct KeypadOverlayLayout {
    int width_px{};
    int height_px{};
    int margin_px{};
    std::array<KeypadOverlayRegion, 10> regions{};
};

KeypadOverlayLayout fixed_keypad_overlay_layout();

enum class KeypadKeyVisualState {
    Idle,
    Hover,
    Pressed,
};

KeypadKeyVisualState keypad_key_visual_state(const std::string& key,
                                             const std::optional<std::string>& hovered_key,
                                             const std::optional<std::string>& pressed_key);

std::optional<std::string> currently_pressed_key(const std::optional<std::string>& last_pressed_key,
                                                 bool touch_armed,
                                                 bool tracking_detected,
                                                 bool calibrating,
                                                 const std::optional<std::string>& hovered_key);

}  // namespace aerial_touch
