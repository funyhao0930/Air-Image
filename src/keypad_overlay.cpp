#include "aerial_touch/keypad_overlay.hpp"

namespace aerial_touch {

KeypadOverlayLayout fixed_keypad_overlay_layout() {
    constexpr int key_width = 30;
    constexpr int key_height = 30;
    constexpr int horizontal_gap = 5;
    constexpr int vertical_gap = 5;

    KeypadOverlayLayout layout;
    layout.width_px = 3 * key_width + 2 * horizontal_gap;
    layout.height_px = 4 * key_height + 3 * vertical_gap;
    layout.margin_px = 12;

    static constexpr std::array<const char*, 9> rows{ "1", "2", "3", "4", "5", "6", "7", "8", "9" };
    for(int row = 0; row < 3; ++row) {
        for(int column = 0; column < 3; ++column) {
            const int index = row * 3 + column;
            layout.regions[static_cast<std::size_t>(index)] = {
                rows[static_cast<std::size_t>(index)],
                column * (key_width + horizontal_gap),
                row * (key_height + vertical_gap),
                key_width,
                key_height,
            };
        }
    }
    layout.regions.back() = { "0", key_width + horizontal_gap, 3 * (key_height + vertical_gap), key_width, key_height };
    return layout;
}

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
                                                 const bool calibrating,
                                                 const std::optional<std::string>& hovered_key) {
    if(!last_pressed_key.has_value() || touch_armed || !tracking_detected || calibrating
       || hovered_key != last_pressed_key) {
        return std::nullopt;
    }
    return last_pressed_key;
}

}  // namespace aerial_touch
