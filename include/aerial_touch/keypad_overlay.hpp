#pragma once

#include <optional>
#include <string>

namespace aerial_touch {

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
