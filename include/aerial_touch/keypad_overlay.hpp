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
    std::array<KeypadOverlayRegion, 12> regions{};
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

// Vertical "distance to the keypad plane" gauge, used by the overhead mounting where the keypad
// plane is seen edge-on and drawing it on the video says nothing about how close the finger is.
// The top end is far from the plane, the bottom end is past it (a negative distance: the finger has
// gone through the aerial image, which nothing physically stops).
struct DistanceGaugeScale {
    float far_mm{};
    float near_mm{};
};

// Spans from twice the release threshold down to as far past the plane as the release threshold
// is in front of it, so both thresholds and the plane itself always sit inside the gauge.
DistanceGaugeScale distance_gauge_scale(float touch_threshold_mm, float release_threshold_mm);

// Pixel offset from the gauge's top for `distance_mm`, clamped to [0, height_px].
int distance_gauge_offset_px(float distance_mm, DistanceGaugeScale scale, int height_px);

std::optional<std::string> currently_pressed_key(const std::optional<std::string>& last_pressed_key,
                                                 bool touch_armed,
                                                 bool tracking_detected,
                                                 bool calibrating,
                                                 const std::optional<std::string>& hovered_key);

}  // namespace aerial_touch
