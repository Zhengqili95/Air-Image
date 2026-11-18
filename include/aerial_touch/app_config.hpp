#pragma once

#include "aerial_touch/camera_settings.hpp"
#include "aerial_touch/hand_tracker.hpp"
#include "aerial_touch/input_method.hpp"
#include "aerial_touch/keypad.hpp"
#include "aerial_touch/touch_state_machine.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

namespace aerial_touch {

// Where the camera sits relative to the keypad plane.
//
// Front: the camera faces the keypad surface (the desk / paper prototype). The plane is found by
// scanning the surface's depth, and the touch distance is mostly a depth reading.
//
// Overhead: the camera looks straight down onto a vertical aerial image. The plane is seen
// edge-on, so it cannot be scanned and contains the camera, which leaves "towards the camera"
// useless for orienting its normal. Calibration traces the plane with the fingertip instead, the
// normal points to the side the hand is on, and the touch distance comes mostly from the image.
enum class CameraMounting {
    Front,
    Overhead,
};

const char* camera_mounting_name(CameraMounting mounting);
std::optional<CameraMounting> parse_camera_mounting(const std::string& text);

struct DepthSamplingConfig {
    int sample_radius{ 2 };
    std::size_t median_window_size{ 5U };
    float max_jump_mm{ 80.0F };
    std::size_t invalid_reset_frames{ 3U };
    // Surface-scan samples more than this far *in front of* the fingertip are treated as finger,
    // not table. The finger rises away from the surface behind the tip, so its pixels read nearer
    // to the camera; table pixels read at the tip's depth or farther, including when the fingertip
    // is resting on the surface.
    float finger_clearance_mm{ 3.0F };
    // Radius of the surface-scan sampling disc, in pixels. This must cover a large fraction of the
    // keypad: a plane fitted to a narrow strip of points leaves its normal free to rotate about the
    // strip, which no residual or inlier count can detect.
    int surface_scan_radius_px{ 150 };
    int surface_scan_stride_px{ 5 };
    // Pixels within this distance of any hand landmark are never treated as surface. A hand held
    // against the target is nearly coplanar with it, so depth alone cannot separate them.
    float hand_exclusion_px{ 40.0F };
    // Samples farther behind the fingertip than this are background, not the target surface.
    float surface_depth_window_mm{ 150.0F };
};

struct FingertipConfig {
    float min_cutoff_hz{ 1.0F };
    float beta{ 0.12F };
    float derivative_cutoff_hz{ 1.0F };
    std::int64_t display_hold_ms{ 100 };
    // Where along the tip -> DIP-joint span the depth probes sit. The tip pixel itself is on the
    // finger's silhouette edge, so its depth mixes finger and background.
    float depth_probe_near_ratio{ 0.35F };
    float depth_probe_far_ratio{ 0.70F };
    // Overhead mounting only. Pixels within this depth of the finger's probe depth count as finger
    // when locating the fingertip's leading edge; seen from above, the background lies far below.
    float depth_segment_window_mm{ 25.0F };
    // The fingertip's leading edge is the frontmost slice across the finger still at least this
    // fraction of the finger's width, so a few flying pixels at the silhouette cannot drag it forward.
    float edge_width_ratio{ 0.3F };
    // Overhead mounting only: One Euro smoothing of the fingertip's 3D position in millimetres
    // (beta is per mm/s), so the top and bottom keys are smoothed alike.
    float position_min_cutoff_hz{ 1.0F };
    float position_beta{ 0.05F };
    // Overhead mounting only: how long the finger may be followed in the depth image alone after
    // MediaPipe last saw the hand. 0 turns depth following off.
    std::int64_t depth_coast_ms{ 1000 };
};

struct CalibrationConfig {
    float minimum_point_distance_mm{ 80.0F };
    std::size_t required_samples{ 18U };
    float mad_multiplier{ 3.5F };
    float minimum_outlier_threshold_mm{ 2.0F };
};

struct AppConfig {
    CameraMounting mounting{ CameraMounting::Front };
    CameraConfig camera;
    DepthSamplingConfig depth;
    FingertipConfig fingertip;
    HandTrackingConfig hand_tracking;
    TouchConfig touch;
    KeypadConfig keypad;
    CalibrationConfig calibration;
    InputMethodConfig input;
};

AppConfig load_app_config(const std::filesystem::path& path);
void save_app_config(const AppConfig& config, const std::filesystem::path& path);
void validate_app_config(const AppConfig& config);

}  // namespace aerial_touch
