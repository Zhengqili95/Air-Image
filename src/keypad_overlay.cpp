#include "aerial_touch/keypad_overlay.hpp"
#include "aerial_touch/input_method.hpp"

namespace aerial_touch {

KeypadOverlayLayout fixed_keypad_overlay_layout() {
    // Large enough for a two-line label such as "7" over "pqrs" once letter mode is active.
    constexpr int key_width = 44;
    constexpr int key_height = 44;
    constexpr int horizontal_gap = 6;
    constexpr int vertical_gap = 6;

    KeypadOverlayLayout layout;
    layout.width_px = 3 * key_width + 2 * horizontal_gap;
    layout.height_px = 4 * key_height + 3 * vertical_gap;
    layout.margin_px = 12;

    // Row-major, phone order: the fourth row is mode key, "0", delete key.
    static constexpr std::array<const char*, 12> keys{ "1", "2", "3", "4", "5", "6",
                                                       "7", "8", "9", kModeKey, "0", kDeleteKey };
    for(int row = 0; row < 4; ++row) {
        for(int column = 0; column < 3; ++column) {
            const auto index = static_cast<std::size_t>(row * 3 + column);
            layout.regions[index] = {
                keys[index],
                column * (key_width + horizontal_gap),
                row * (key_height + vertical_gap),
                key_width,
                key_height,
            };
        }
    }
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
