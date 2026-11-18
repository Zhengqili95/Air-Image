#pragma once

#include "aerial_touch/types.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace aerial_touch {

// Locates the leading edge of the index finger in the depth image, for the overhead mounting.
//
// Seen from above, a camera looking down onto a vertical aerial image decides "has the finger
// reached the plane" almost entirely from where the fingertip is along the finger's own direction.
// MediaPipe's landmark 8 is an estimate of that point, off by a few pixels along the finger and by
// an amount that changes with hand pose -- which lands squarely on the touch axis. From above,
// though, the background (the imaging plate, the desk) is far below the finger, so the finger
// separates cleanly from it by depth. MediaPipe is then only used to say *which* finger and roughly
// where; the leading edge comes from the finger pixels themselves.
struct FingertipExtremityConfig {
    // Pixels whose depth is within this of the finger's reference depth count as finger.
    float depth_window_mm{ 25.0F };
    // The leading edge is the frontmost one-pixel slice across the finger that still holds at least
    // this fraction of the finger's typical width in pixels. A few flying pixels past the silhouette
    // make slices of one or two and are skipped. The rounded fingertip leaves the edge a pixel or so
    // behind the very tip; that offset is the same during calibration and use, so the calibrated
    // plane absorbs it.
    float edge_width_ratio{ 0.3F };
    // Search disc around the MediaPipe tip, as a fraction of the tip -> DIP span.
    float roi_radius_ratio{ 0.8F };
    int minimum_roi_radius_px{ 10 };
    // Pixels farther than this fraction of the span to either side of the finger's axis belong to
    // some other finger (or to the knuckles), not to the index finger.
    float lateral_limit_ratio{ 0.5F };
    // Fewer finger pixels than this and the segmentation is not trusted.
    std::size_t minimum_finger_pixels{ 12U };
    // The edge found may lie at most this many spans from the tip passed in.
    float maximum_offset_ratio{ 1.0F };
    // The finger's depth is the median over the finger pixels this many spans behind the edge.
    float depth_band_ratio{ 0.6F };
};

struct FingertipExtremity {
    Vec2 pixel{};
    std::size_t finger_pixels{};
    // Median depth of the finger just behind its leading edge. Far from the camera a 5 x 5 window
    // holds only a few finger pixels and some of the silhouette; this uses every finger pixel near
    // the tip, which is what keeps the row (decided by depth) steady on the lower keys.
    float depth_mm{};
};

// `tip_pixel` and `joint_pixel` are MediaPipe's landmark 8 (tip) and 7 (DIP), in depth-image
// pixels. `reference_depth_mm` is the finger's depth just behind the tip (the depth probe).
//
// Returns nullopt -- and the caller keeps MediaPipe's tip -- when the finger points at the camera
// (no usable direction), too few pixels segment as finger, or the edge found is implausibly far
// from MediaPipe's tip.
std::optional<FingertipExtremity> locate_fingertip_extremity(
    const std::vector<std::uint16_t>& depth_values,
    int width,
    int height,
    float depth_unit_mm,
    Vec2 tip_pixel,
    Vec2 joint_pixel,
    float reference_depth_mm,
    const FingertipExtremityConfig& config = {});

// What is needed to keep following the finger in the depth image alone.
struct DepthFingerTrack {
    Vec2 tip_pixel{};
    Vec2 direction{};   // unit vector, DIP joint -> tip, in the image
    float span_px{};    // tip -> DIP distance in the image
    float depth_mm{};
};

// MediaPipe drops the back of a hand seen from above far more often than it loses a palm, and it
// only recovers by re-running palm detection. Seen from above, though, the finger stays easy to
// pick out by depth, so between MediaPipe observations it is followed from where it last was: a
// wider search disc around the previous tip, the previous pointing direction, and a depth that
// must stay within the segmentation window of the previous one. Returns nullopt once the finger
// cannot be found that way (withdrawn, moved too far, or something at another depth).
std::optional<DepthFingerTrack> follow_finger_by_depth(
    const std::vector<std::uint16_t>& depth_values,
    int width,
    int height,
    float depth_unit_mm,
    const DepthFingerTrack& previous,
    FingertipExtremityConfig config = {});

}  // namespace aerial_touch
