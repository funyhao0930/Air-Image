#include "aerial_touch/keypad_overlay.hpp"

namespace aerial_touch {

KeypadKeyVisualState keypad_key_visual_state(const std::string& key,
                                             const std::optional<std::string>& hovered_key,
                                             const std::optional<std::string>& pressed_key) {
    if(pressed_key.has_value() && *pressed_key == key) {
        return KeypadKeyVisualState::Pressed;
    }
    if(hovered_key.has_value() && *hovered_key == key) {
        return KeypadKeyVisualState::Hover;
    }
    return KeypadKeyVisualState::Idle;
}

std::optional<std::string> currently_pressed_key(const std::optional<std::string>& last_pressed_key,
                                                 const bool touch_armed,
                                                 const bool tracking_detected,
                                                 const bool calibrating) {
    if(!last_pressed_key.has_value() || touch_armed || !tracking_detected || calibrating) {
        return std::nullopt;
    }
    return last_pressed_key;
}

}  // namespace aerial_touch
