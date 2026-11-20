#include "aerial_touch/fingertip_extremity.hpp"

#include <algorithm>
#include <cmath>

namespace aerial_touch {
namespace {

// Below this the finger is pointing at the camera and has no usable direction in the image.
constexpr float kMinimumSpanPx = 2.0F;

bool valid_config(const FingertipExtremityConfig& config) {
    return std::isfinite(config.depth_window_mm) && config.depth_window_mm > 0.0F
           && std::isfinite(config.edge_width_ratio) && config.edge_width_ratio > 0.0F && config.edge_width_ratio <= 1.0F
           && std::isfinite(config.roi_radius_ratio) && config.roi_radius_ratio > 0.0F
           && config.minimum_roi_radius_px > 0 && std::isfinite(config.lateral_limit_ratio)
           && config.lateral_limit_ratio > 0.0F && config.minimum_finger_pixels > 0U
           && std::isfinite(config.maximum_offset_ratio) && config.maximum_offset_ratio > 0.0F
           && std::isfinite(config.depth_band_ratio) && config.depth_band_ratio > 0.0F;
}

struct FingerPixel {
    float along{};    // position along the finger, from the DIP joint towards the tip
    float lateral{};  // signed offset across the finger
    float depth_mm{};
};

float median_of(std::vector<float>& values) {
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2U);
    std::nth_element(values.begin(), middle, values.end());
    if(values.size() % 2U == 1U) {
        return *middle;
    }
    const float upper = *middle;
    return (*std::max_element(values.begin(), middle) + upper) / 2.0F;
}

}  // namespace

std::optional<FingertipExtremity> locate_fingertip_extremity(
    const std::vector<std::uint16_t>& depth_values,
    const int width,
    const int height,
    const float depth_unit_mm,
    const Vec2 tip_pixel,
    const Vec2 joint_pixel,
    const float reference_depth_mm,
    const FingertipExtremityConfig& config) {
    if(!valid_config(config) || width <= 0 || height <= 0 || !std::isfinite(depth_unit_mm)
       || depth_unit_mm <= 0.0F || !std::isfinite(reference_depth_mm) || reference_depth_mm <= 0.0F
       || depth_values.size() < static_cast<std::size_t>(width) * static_cast<std::size_t>(height)
       || !std::isfinite(tip_pixel.x) || !std::isfinite(tip_pixel.y) || !std::isfinite(joint_pixel.x)
       || !std::isfinite(joint_pixel.y)) {
        return std::nullopt;
    }

    const float span = std::hypot(tip_pixel.x - joint_pixel.x, tip_pixel.y - joint_pixel.y);
    if(!std::isfinite(span) || span < kMinimumSpanPx) {
        return std::nullopt;
    }
    const Vec2 along_axis{ (tip_pixel.x - joint_pixel.x) / span, (tip_pixel.y - joint_pixel.y) / span };
    const Vec2 lateral_axis{ -along_axis.y, along_axis.x };

    const float radius = std::max(static_cast<float>(config.minimum_roi_radius_px), config.roi_radius_ratio * span);
    const float lateral_limit = config.lateral_limit_ratio * span;
    const int left = std::max(0, static_cast<int>(std::floor(tip_pixel.x - radius)));
    const int right = std::min(width - 1, static_cast<int>(std::ceil(tip_pixel.x + radius)));
    const int top = std::max(0, static_cast<int>(std::floor(tip_pixel.y - radius)));
    const int bottom = std::min(height - 1, static_cast<int>(std::ceil(tip_pixel.y + radius)));
    if(left > right || top > bottom) {
        return std::nullopt;
    }

    std::vector<FingerPixel> finger;
    finger.reserve(static_cast<std::size_t>((right - left + 1) * (bottom - top + 1)));
    for(int y = top; y <= bottom; ++y) {
        for(int x = left; x <= right; ++x) {
            const float dx = static_cast<float>(x) - tip_pixel.x;
            const float dy = static_cast<float>(y) - tip_pixel.y;
            if(dx * dx + dy * dy > radius * radius) {
                continue;
            }
            const std::uint16_t raw = depth_values[static_cast<std::size_t>(y) * static_cast<std::size_t>(width)
                                                   + static_cast<std::size_t>(x)];
            if(raw == 0U) {
                continue;
            }
            const float depth_mm = static_cast<float>(raw) * depth_unit_mm;
            if(std::fabs(depth_mm - reference_depth_mm) > config.depth_window_mm) {
                continue;
            }
            const float rel_x = static_cast<float>(x) - joint_pixel.x;
            const float rel_y = static_cast<float>(y) - joint_pixel.y;
            const float lateral = rel_x * lateral_axis.x + rel_y * lateral_axis.y;
            if(std::fabs(lateral) > lateral_limit) {
                continue;
            }
            finger.push_back({ rel_x * along_axis.x + rel_y * along_axis.y, lateral, depth_mm });
        }
    }
    if(finger.size() < config.minimum_finger_pixels) {
        return std::nullopt;
    }

    // Slice the finger into one-pixel steps along its direction. A slice through the finger body is
    // about one finger-width of pixels; stray pixels past the silhouette make slices of one or two.
    // The leading edge is the frontmost slice still at least `edge_width_ratio` of the typical width.
    // Unlike a quantile of all finger pixels, this does not move with how much finger the search disc
    // happens to contain.
    float minimum_along = finger.front().along;
    float maximum_along = finger.front().along;
    for(const auto& pixel : finger) {
        minimum_along = std::min(minimum_along, pixel.along);
        maximum_along = std::max(maximum_along, pixel.along);
    }
    const int first_slice = static_cast<int>(std::floor(minimum_along));
    std::vector<std::size_t> slice_counts(static_cast<std::size_t>(static_cast<int>(std::floor(maximum_along)) - first_slice + 1), 0U);
    for(const auto& pixel : finger) {
        ++slice_counts[static_cast<std::size_t>(static_cast<int>(std::floor(pixel.along)) - first_slice)];
    }
    std::vector<float> occupied;
    for(const auto count : slice_counts) {
        if(count > 0U) {
            occupied.push_back(static_cast<float>(count));
        }
    }
    const float typical_width = median_of(occupied);
    const float minimum_width = std::max(2.0F, config.edge_width_ratio * typical_width);
    std::optional<int> leading_slice;
    for(int slice = static_cast<int>(slice_counts.size()) - 1; slice >= 0; --slice) {
        if(static_cast<float>(slice_counts[static_cast<std::size_t>(slice)]) >= minimum_width) {
            leading_slice = slice;
            break;
        }
    }
    if(!leading_slice.has_value()) {
        return std::nullopt;
    }
    const float leading_along = static_cast<float>(first_slice + *leading_slice) + 0.5F;

    // Across the finger, centre on the pixels at the leading edge rather than on the whole ROI, so
    // a finger that curves or is seen slightly askew still puts the tip on its own centreline.
    const float band = std::max(1.0F, 0.1F * span);
    const float depth_band = std::max(2.0F, config.depth_band_ratio * span);
    float lateral_sum = 0.0F;
    std::size_t lateral_count = 0U;
    std::vector<float> depths;
    depths.reserve(finger.size());
    for(const auto& pixel : finger) {
        if(pixel.along >= leading_along - band && pixel.along <= leading_along + band) {
            lateral_sum += pixel.lateral;
            ++lateral_count;
        }
        if(pixel.along >= leading_along - depth_band && pixel.along <= leading_along) {
            depths.push_back(pixel.depth_mm);
        }
    }
    const float lateral = lateral_count > 0U ? lateral_sum / static_cast<float>(lateral_count) : 0.0F;
    if(depths.empty()) {
        return std::nullopt;
    }
    const float depth_mm = median_of(depths);

    const Vec2 edge{ joint_pixel.x + along_axis.x * leading_along + lateral_axis.x * lateral,
                     joint_pixel.y + along_axis.y * leading_along + lateral_axis.y * lateral };
    if(!std::isfinite(edge.x) || !std::isfinite(edge.y) || !std::isfinite(depth_mm)
       || std::hypot(edge.x - tip_pixel.x, edge.y - tip_pixel.y) > config.maximum_offset_ratio * span) {
        return std::nullopt;
    }
    return FingertipExtremity{ edge, finger.size(), depth_mm };
}

std::optional<DepthFingerTrack> follow_finger_by_depth(
    const std::vector<std::uint16_t>& depth_values,
    const int width,
    const int height,
    const float depth_unit_mm,
    const DepthFingerTrack& previous,
    FingertipExtremityConfig config) {
    const float direction_length = std::hypot(previous.direction.x, previous.direction.y);
    if(!std::isfinite(previous.span_px) || previous.span_px < kMinimumSpanPx || !std::isfinite(direction_length)
       || direction_length < 0.5F) {
        return std::nullopt;
    }
    const Vec2 direction{ previous.direction.x / direction_length, previous.direction.y / direction_length };
    // Between frames the finger moves; widen the search so a press or a sideways move between keys
    // stays inside it. The depth window is what keeps this from wandering onto something else.
    config.roi_radius_ratio = std::max(config.roi_radius_ratio, 1.5F);
    config.maximum_offset_ratio = std::max(config.maximum_offset_ratio, 1.5F);
    config.lateral_limit_ratio = std::max(config.lateral_limit_ratio, 0.8F);
    const Vec2 joint{ previous.tip_pixel.x - direction.x * previous.span_px,
                      previous.tip_pixel.y - direction.y * previous.span_px };
    const auto edge = locate_fingertip_extremity(depth_values, width, height, depth_unit_mm, previous.tip_pixel,
                                                 joint, previous.depth_mm, config);
    if(!edge.has_value()) {
        return std::nullopt;
    }
    return DepthFingerTrack{ edge->pixel, direction, previous.span_px, edge->depth_mm };
}

}  // namespace aerial_touch
