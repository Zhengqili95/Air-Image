// Overhead mounting: a camera looking straight down onto a vertical aerial keypad image.
//
// The cases here pin down the three things that break when the camera sits in the keypad plane
// instead of facing it: which side of the plane the user is on, how the plane is found without a
// surface to scan, and where the fingertip's leading edge is.

#include "aerial_touch/app_config.hpp"
#include "aerial_touch/calibration_geometry.hpp"
#include "aerial_touch/fingertip_extremity.hpp"
#include "aerial_touch/keypad_overlay.hpp"
#include "aerial_touch/hand_tracker.hpp"
#include "aerial_touch/plane.hpp"
#include "aerial_touch/signal_stabilizer.hpp"
#include "aerial_touch/surface_plane.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {

bool within(const float actual, const float expected, const float tolerance) {
    return std::fabs(actual - expected) <= tolerance;
}

// A keypad plane that contains the camera's optical axis: y = 0, spanning x across and z (depth,
// i.e. height below the camera) down. "Towards the camera" is perpendicular to its normal here.
constexpr aerial_touch::Vec3 kTopLeft{ 0.0F, 0.0F, 300.0F };
constexpr aerial_touch::Vec3 kTopRight{ 90.0F, 0.0F, 300.0F };
constexpr aerial_touch::Vec3 kBottom{ 45.0F, 0.0F, 420.0F };

bool user_side_hint_orients_the_normal_when_the_camera_is_in_the_plane() {
    const auto towards_user = aerial_touch::Plane::from_calibration_points(
        kTopLeft, kTopRight, kBottom, 80.0F, aerial_touch::Vec3{ 0.0F, 1.0F, 0.0F });
    const auto away_from_user = aerial_touch::Plane::from_calibration_points(
        kTopLeft, kTopRight, kBottom, 80.0F, aerial_touch::Vec3{ 0.0F, -1.0F, 0.0F });
    if(!towards_user.has_value() || !away_from_user.has_value()) {
        return false;
    }
    const auto a = towards_user->project({ 50.0F, 10.0F, 350.0F });
    const auto b = away_from_user->project({ 50.0F, 10.0F, 350.0F });
    return within(a.u_mm, 50.0F, 0.01F) && within(a.v_mm, 50.0F, 0.01F) && within(a.signed_distance_mm, 10.0F, 0.01F)
           && within(b.signed_distance_mm, -10.0F, 0.01F);
}

// The joint direction is never exactly along the normal: a finger pointing 50 degrees off it still
// says clearly which side the hand is on.
bool an_oblique_user_side_hint_still_decides_the_side() {
    const float angle = 50.0F * 3.14159265F / 180.0F;
    const auto plane = aerial_touch::Plane::from_calibration_points(
        kTopLeft, kTopRight, kBottom, 80.0F, aerial_touch::Vec3{ std::sin(angle), std::cos(angle), 0.0F });
    return plane.has_value() && plane->project({ 50.0F, 10.0F, 350.0F }).signed_distance_mm > 0.0F;
}

bool user_side_hint_lying_in_the_plane_is_rejected() {
    return !aerial_touch::Plane::from_calibration_points(kTopLeft, kTopRight, kBottom, 80.0F,
                                                         aerial_touch::Vec3{ 1.0F, 0.0F, 0.0F })
                .has_value()
           && !aerial_touch::Plane::from_calibration_points(kTopLeft, kTopRight, kBottom, 80.0F,
                                                            aerial_touch::Vec3{ 0.0F, 0.0F, 0.0F })
                   .has_value();
}

bool keypad_calibration_reports_an_ambiguous_user_side() {
    const std::array<aerial_touch::Vec3, aerial_touch::kKeypadCalibrationPointCount> points{ { kTopLeft, kTopRight,
                                                                                              kBottom } };
    const auto ambiguous =
        aerial_touch::calibrate_keypad_detailed(points, 80.0F, aerial_touch::Vec3{ 0.0F, 0.0F, 1.0F });
    const auto decided =
        aerial_touch::calibrate_keypad_detailed(points, 80.0F, aerial_touch::Vec3{ 0.0F, 1.0F, 0.0F });
    return !ambiguous.result.has_value()
           && ambiguous.failure == aerial_touch::KeypadCalibrationFailure::AmbiguousUserSide
           && decided.result.has_value() && within(decided.result->geometry.total_width_mm, 90.0F, 0.01F)
           && within(decided.result->geometry.total_height_mm, 120.0F, 0.01F)
           && decided.result->plane.project({ 45.0F, 8.0F, 360.0F }).signed_distance_mm > 0.0F;
}

// Bad corner points are still reported as such when a hint is given, not as an ambiguous side.
bool collinear_points_with_a_hint_are_still_an_invalid_plane() {
    const std::array<aerial_touch::Vec3, aerial_touch::kKeypadCalibrationPointCount> points{
        { kTopLeft, kTopRight, { 180.0F, 0.0F, 300.0F } }
    };
    const auto attempt =
        aerial_touch::calibrate_keypad_detailed(points, 80.0F, aerial_touch::Vec3{ 0.0F, 1.0F, 0.0F });
    return attempt.failure == aerial_touch::KeypadCalibrationFailure::InvalidPlane;
}

// The fingertip sliding over a card held in the image plane traces it well enough to fit.
bool fingertip_trace_over_a_vertical_card_fits_the_plane() {
    std::vector<aerial_touch::Vec3> trace;
    int step = 0;
    for(float z = 300.0F; z <= 420.0F; z += 8.0F) {
        for(float x = 0.0F; x <= 90.0F; x += 6.0F) {
            // Pressing a little harder or softer against the card: +-1.5 mm across the plane.
            const float wobble = static_cast<float>((step * 5) % 7 - 3) * 0.5F;
            trace.push_back({ x, wobble, z });
            ++step;
        }
    }
    aerial_touch::SurfacePlaneFitQuality quality;
    const auto plane = aerial_touch::SurfacePlane::fit(trace, {}, &quality);
    if(!plane.has_value()) {
        return false;
    }
    const auto dropped = plane->project_to_surface({ 40.0F, 12.0F, 360.0F });
    return within(std::fabs(plane->signed_distance({ 40.0F, 12.0F, 360.0F })), 12.0F, 1.0F) && dropped.has_value()
           && within(dropped->x, 40.0F, 1.0F) && within(dropped->y, 0.0F, 1.0F) && within(dropped->z, 360.0F, 1.0F);
}

// Sliding along a single line leaves the plane free to rotate about it, so it must not be accepted.
bool fingertip_trace_along_one_line_is_rejected() {
    std::vector<aerial_touch::Vec3> trace;
    for(int index = 0; index < 120; ++index) {
        trace.push_back({ static_cast<float>(index) * 0.75F, 0.0F, 360.0F + static_cast<float>(index % 3) * 0.3F });
    }
    return !aerial_touch::SurfacePlane::fit(trace).has_value();
}

// --- Depth-located fingertip ---------------------------------------------------------------------

constexpr int kDepthWidth = 200;
constexpr int kDepthHeight = 200;
constexpr std::uint16_t kBackgroundMm = 600U;
constexpr std::uint16_t kFingerMm = 300U;

// A finger seen from above: a strip pointing along +x whose leading edge is at x = 120.
std::vector<std::uint16_t> finger_depth_image() {
    std::vector<std::uint16_t> depth(static_cast<std::size_t>(kDepthWidth * kDepthHeight), kBackgroundMm);
    for(int y = 95; y <= 105; ++y) {
        for(int x = 40; x <= 120; ++x) {
            depth[static_cast<std::size_t>(y * kDepthWidth + x)] = kFingerMm;
        }
    }
    return depth;
}

// MediaPipe's tip estimate sits 6 px behind the real edge; the depth image puts it back.
bool fingertip_extremity_finds_the_leading_edge_behind_the_landmark() {
    const auto depth = finger_depth_image();
    const auto edge = aerial_touch::locate_fingertip_extremity(depth, kDepthWidth, kDepthHeight, 1.0F,
                                                               { 114.0F, 100.0F }, { 90.0F, 100.0F }, 300.0F);
    return edge.has_value() && within(edge->pixel.x, 120.0F, 1.5F) && within(edge->pixel.y, 100.0F, 0.5F);
}

// The landmark can also overshoot the finger; the edge is still where the finger ends.
bool fingertip_extremity_pulls_back_an_overshooting_landmark() {
    const auto depth = finger_depth_image();
    const auto edge = aerial_touch::locate_fingertip_extremity(depth, kDepthWidth, kDepthHeight, 1.0F,
                                                               { 125.0F, 101.0F }, { 101.0F, 101.0F }, 300.0F);
    return edge.has_value() && within(edge->pixel.x, 120.0F, 1.5F) && within(edge->pixel.y, 100.0F, 1.0F);
}

bool fingertip_extremity_ignores_flying_pixels_past_the_edge() {
    auto depth = finger_depth_image();
    for(const auto& [x, y] : std::array<std::pair<int, int>, 3>{ { { 126, 100 }, { 127, 100 }, { 128, 101 } } }) {
        depth[static_cast<std::size_t>(y * kDepthWidth + x)] = kFingerMm;
    }
    const auto edge = aerial_touch::locate_fingertip_extremity(depth, kDepthWidth, kDepthHeight, 1.0F,
                                                               { 114.0F, 100.0F }, { 90.0F, 100.0F }, 300.0F);
    return edge.has_value() && edge->pixel.x <= 121.5F;
}

// A curled finger lying alongside, but well below, the pointing one is not part of it.
bool fingertip_extremity_ignores_a_finger_at_another_depth() {
    auto depth = finger_depth_image();
    for(int y = 106; y <= 112; ++y) {
        for(int x = 100; x <= 135; ++x) {
            depth[static_cast<std::size_t>(y * kDepthWidth + x)] = 340U;
        }
    }
    const auto edge = aerial_touch::locate_fingertip_extremity(depth, kDepthWidth, kDepthHeight, 1.0F,
                                                               { 114.0F, 100.0F }, { 90.0F, 100.0F }, 300.0F);
    return edge.has_value() && within(edge->pixel.x, 120.0F, 1.5F) && within(edge->pixel.y, 100.0F, 0.5F);
}

bool fingertip_extremity_gives_up_without_a_finger_or_a_direction() {
    const std::vector<std::uint16_t> empty(static_cast<std::size_t>(kDepthWidth * kDepthHeight), kBackgroundMm);
    const auto depth = finger_depth_image();
    return !aerial_touch::locate_fingertip_extremity(empty, kDepthWidth, kDepthHeight, 1.0F, { 114.0F, 100.0F },
                                                     { 90.0F, 100.0F }, 300.0F)
                .has_value()
           // Tip and joint on the same pixel: the finger points at the camera.
           && !aerial_touch::locate_fingertip_extremity(depth, kDepthWidth, kDepthHeight, 1.0F, { 114.0F, 100.0F },
                                                        { 114.0F, 100.0F }, 300.0F)
                   .has_value();
}

// Far from the camera the depth is noisy pixel to pixel; the finger's depth is the median over the
// finger pixels behind the edge, not whatever a few pixels at the tip happen to read.
bool fingertip_extremity_takes_the_finger_depth_from_many_pixels() {
    auto depth = finger_depth_image();
    int index = 0;
    for(int y = 95; y <= 105; ++y) {
        for(int x = 40; x <= 120; ++x) {
            // +-8 mm of pixel noise, and the two front columns (the silhouette) read 15 mm deep.
            const int noise = ((index++ * 7) % 17) - 8;
            depth[static_cast<std::size_t>(y * kDepthWidth + x)] =
                static_cast<std::uint16_t>(static_cast<int>(kFingerMm) + noise + (x >= 119 ? 15 : 0));
        }
    }
    const auto edge = aerial_touch::locate_fingertip_extremity(depth, kDepthWidth, kDepthHeight, 1.0F,
                                                               { 114.0F, 100.0F }, { 90.0F, 100.0F }, 300.0F);
    return edge.has_value() && within(edge->depth_mm, 300.0F, 2.0F);
}

// --- Following the finger by depth while MediaPipe has lost the hand ------------------------------

std::vector<std::uint16_t> finger_at(const int front_x, const int centre_y, const std::uint16_t finger_mm) {
    std::vector<std::uint16_t> depth(static_cast<std::size_t>(kDepthWidth * kDepthHeight), kBackgroundMm);
    for(int y = centre_y - 5; y <= centre_y + 5; ++y) {
        for(int x = front_x - 80; x <= front_x; ++x) {
            depth[static_cast<std::size_t>(y * kDepthWidth + x)] = finger_mm;
        }
    }
    return depth;
}

constexpr aerial_touch::DepthFingerTrack kLastSeen{ { 120.0F, 100.0F }, { 1.0F, 0.0F }, 24.0F, 300.0F };

// Between two frames the finger pushes forward and drifts sideways onto the next key.
bool depth_following_keeps_up_with_a_moving_finger() {
    const auto depth = finger_at(130, 106, 304U);
    const auto followed =
        aerial_touch::follow_finger_by_depth(depth, kDepthWidth, kDepthHeight, 1.0F, kLastSeen);
    return followed.has_value() && within(followed->tip_pixel.x, 130.0F, 1.5F)
           && within(followed->tip_pixel.y, 106.0F, 1.0F) && within(followed->depth_mm, 304.0F, 0.5F)
           && within(followed->span_px, 24.0F, 0.01F);
}

bool depth_following_stops_when_the_finger_is_withdrawn() {
    const std::vector<std::uint16_t> empty(static_cast<std::size_t>(kDepthWidth * kDepthHeight), kBackgroundMm);
    return !aerial_touch::follow_finger_by_depth(empty, kDepthWidth, kDepthHeight, 1.0F, kLastSeen).has_value();
}

// Something at a different height where the finger was -- the knuckles of another finger, the
// imaging plate -- is not the finger.
bool depth_following_will_not_jump_to_another_depth() {
    const auto depth = finger_at(122, 100, 360U);
    return !aerial_touch::follow_finger_by_depth(depth, kDepthWidth, kDepthHeight, 1.0F, kLastSeen).has_value();
}

// --- Millimetre smoothing and the jitter meter ---------------------------------------------------

bool position_filter_smooths_a_still_fingertip_in_millimetres() {
    aerial_touch::OneEuroFilter3D filter({ 1.0F, 0.05F, 1.0F });
    aerial_touch::RollingSpread raw(60U);
    aerial_touch::RollingSpread filtered(60U);
    for(int frame = 0; frame < 120; ++frame) {
        // A fingertip held still at the bottom row: +-3 mm of depth noise, +-1 mm across.
        const float noise = static_cast<float>((frame * 7) % 13 - 6) * 0.5F;
        const aerial_touch::Vec3 sample{ 10.0F + noise / 3.0F, -40.0F, 450.0F + noise };
        const auto smoothed = filter.update(sample, frame * 33);
        if(frame >= 60) {
            raw.add(sample);
            filtered.add(smoothed);
        }
    }
    const auto raw_spread = raw.standard_deviation();
    const auto filtered_spread = filtered.standard_deviation();
    return raw_spread.has_value() && filtered_spread.has_value() && filtered_spread->z < raw_spread->z * 0.5F
           && filtered_spread->x < raw_spread->x * 0.5F;
}

// A real press (about 200 mm/s towards the plane) must not lag far behind, or the approach-speed
// gate and the touch threshold would see it late.
bool position_filter_follows_a_press_closely() {
    aerial_touch::OneEuroFilter3D filter({ 1.0F, 0.05F, 1.0F });
    aerial_touch::Vec3 smoothed{};
    float y = 0.0F;
    for(int frame = 0; frame < 15; ++frame) {
        y = 40.0F - static_cast<float>(frame) * (200.0F / 30.0F);
        smoothed = filter.update({ 0.0F, y, 400.0F }, frame * 33);
    }
    return std::fabs(smoothed.y - y) < 12.0F;
}

bool rolling_spread_reports_the_standard_deviation_of_recent_samples() {
    aerial_touch::RollingSpread spread(4U);
    if(spread.standard_deviation().has_value()) {
        return false;
    }
    for(const float value : { 100.0F, 100.0F, 0.0F, 2.0F, 0.0F, 2.0F }) {
        spread.add({ value, 5.0F, value * 2.0F });
    }
    const auto deviation = spread.standard_deviation();
    return spread.size() == 4U && deviation.has_value() && within(deviation->x, 1.0F, 0.001F)
           && within(deviation->y, 0.0F, 0.001F) && within(deviation->z, 2.0F, 0.001F);
}

// --- Distance gauge ------------------------------------------------------------------------------

bool distance_gauge_keeps_thresholds_and_the_plane_inside() {
    const auto scale = aerial_touch::distance_gauge_scale(10.0F, 20.0F);
    const int height = 200;
    const int release = aerial_touch::distance_gauge_offset_px(20.0F, scale, height);
    const int touch = aerial_touch::distance_gauge_offset_px(10.0F, scale, height);
    const int plane = aerial_touch::distance_gauge_offset_px(0.0F, scale, height);
    const int through = aerial_touch::distance_gauge_offset_px(-10.0F, scale, height);
    return within(scale.far_mm, 40.0F, 0.01F) && within(scale.near_mm, -20.0F, 0.01F) && 0 < release
           && release < touch && touch < plane && plane < through && through < height
           && aerial_touch::distance_gauge_offset_px(500.0F, scale, height) == 0
           && aerial_touch::distance_gauge_offset_px(-500.0F, scale, height) == height;
}

// --- Configuration -------------------------------------------------------------------------------

std::filesystem::path write_minimal_config(const char* file_name, const std::string& camera_section) {
    const auto path = std::filesystem::temp_directory_path() / file_name;
    std::ofstream output(path);
    output << camera_section
           << "depth:\n"
              "  sample_radius: 2\n"
              "touch:\n"
              "  touch_threshold_mm: 10.0\n"
              "  release_threshold_mm: 20.0\n"
              "  min_approach_velocity_mm_s: 0.0\n"
              "  tracking_timeout_ms: 300\n"
              "calibration:\n"
              "  minimum_point_distance_mm: 80.0\n";
    return path;
}

bool mounting_defaults_to_front_and_reads_overhead() {
    const auto legacy = write_minimal_config("aerial_touch_mounting_legacy.yaml", "");
    const auto overhead =
        write_minimal_config("aerial_touch_mounting_overhead.yaml", "camera:\n  mounting: overhead\n");
    const auto legacy_config = aerial_touch::load_app_config(legacy);
    const auto overhead_config = aerial_touch::load_app_config(overhead);
    std::filesystem::remove(legacy);
    std::filesystem::remove(overhead);
    return legacy_config.mounting == aerial_touch::CameraMounting::Front
           && overhead_config.mounting == aerial_touch::CameraMounting::Overhead
           && within(legacy_config.fingertip.depth_segment_window_mm, 25.0F, 0.001F)
           && within(legacy_config.fingertip.edge_width_ratio, 0.3F, 0.001F);
}

bool misspelled_mounting_is_rejected() {
    const auto path = write_minimal_config("aerial_touch_mounting_typo.yaml", "camera:\n  mounting: overheard\n");
    bool rejected = false;
    try {
        static_cast<void>(aerial_touch::load_app_config(path));
    }
    catch(const std::exception&) {
        rejected = true;
    }
    std::filesystem::remove(path);
    return rejected;
}

// The settings window saves the whole config; a mounting it does not show must survive that.
bool mounting_and_extremity_settings_round_trip_through_save() {
    aerial_touch::AppConfig source;
    source.mounting = aerial_touch::CameraMounting::Overhead;
    source.fingertip.depth_segment_window_mm = 18.0F;
    source.fingertip.edge_width_ratio = 0.4F;
    const auto path = std::filesystem::temp_directory_path() / "aerial_touch_mounting_roundtrip.yaml";
    aerial_touch::save_app_config(source, path);
    const auto restored = aerial_touch::load_app_config(path);
    std::filesystem::remove(path);
    return restored.mounting == aerial_touch::CameraMounting::Overhead
           && within(restored.fingertip.depth_segment_window_mm, 18.0F, 0.001F)
           && within(restored.fingertip.edge_width_ratio, 0.4F, 0.001F);
}

bool tracking_settings_round_trip_through_save() {
    aerial_touch::AppConfig source;
    source.fingertip.position_min_cutoff_hz = 0.7F;
    source.fingertip.position_beta = 0.08F;
    source.fingertip.depth_coast_ms = 600;
    source.hand_tracking = { 0.45F, 0.3F, 0.25F };
    const auto path = std::filesystem::temp_directory_path() / "aerial_touch_tracking_roundtrip.yaml";
    aerial_touch::save_app_config(source, path);
    const auto restored = aerial_touch::load_app_config(path);
    std::filesystem::remove(path);
    return within(restored.fingertip.position_min_cutoff_hz, 0.7F, 0.001F)
           && within(restored.fingertip.position_beta, 0.08F, 0.001F) && restored.fingertip.depth_coast_ms == 600
           && within(restored.hand_tracking.min_detection_confidence, 0.45F, 0.001F)
           && within(restored.hand_tracking.min_presence_confidence, 0.3F, 0.001F)
           && within(restored.hand_tracking.min_tracking_confidence, 0.25F, 0.001F);
}

bool invalid_tracking_settings_are_rejected() {
    aerial_touch::AppConfig zero_confidence;
    zero_confidence.hand_tracking.min_presence_confidence = 0.0F;
    aerial_touch::AppConfig confidence_above_one;
    confidence_above_one.hand_tracking.min_detection_confidence = 1.2F;
    aerial_touch::AppConfig negative_coast;
    negative_coast.fingertip.depth_coast_ms = -1;
    aerial_touch::AppConfig zero_cutoff;
    zero_cutoff.fingertip.position_min_cutoff_hz = 0.0F;
    int rejected = 0;
    for(const auto& config : { zero_confidence, confidence_above_one, negative_coast, zero_cutoff }) {
        try {
            aerial_touch::validate_app_config(config);
        }
        catch(const std::exception&) {
            ++rejected;
        }
    }
    return rejected == 4;
}

bool missing_bridge_with_tracking_thresholds_is_safe() {
    aerial_touch::HandTracker tracker("definitely-missing-hand-bridge.dll", "missing-model.task", { 0.4F, 0.3F, 0.3F });
    return !tracker.available() && !tracker.error().empty() && tracker.warning().empty();
}

bool invalid_extremity_settings_are_rejected() {
    aerial_touch::AppConfig zero_window;
    zero_window.fingertip.depth_segment_window_mm = 0.0F;
    aerial_touch::AppConfig percentile_above_one;
    percentile_above_one.fingertip.edge_width_ratio = 1.5F;
    int rejected = 0;
    for(const auto& config : { zero_window, percentile_above_one }) {
        try {
            aerial_touch::validate_app_config(config);
        }
        catch(const std::exception&) {
            ++rejected;
        }
    }
    return rejected == 2;
}

}  // namespace

bool run_overhead_mounting_tests() {
    const std::array<std::pair<const char*, bool (*)()>, 29> tests{ {
        { "fingertip_extremity_takes_the_finger_depth_from_many_pixels", &fingertip_extremity_takes_the_finger_depth_from_many_pixels },
        { "depth_following_keeps_up_with_a_moving_finger", &depth_following_keeps_up_with_a_moving_finger },
        { "depth_following_stops_when_the_finger_is_withdrawn", &depth_following_stops_when_the_finger_is_withdrawn },
        { "depth_following_will_not_jump_to_another_depth", &depth_following_will_not_jump_to_another_depth },
        { "position_filter_smooths_a_still_fingertip_in_millimetres", &position_filter_smooths_a_still_fingertip_in_millimetres },
        { "position_filter_follows_a_press_closely", &position_filter_follows_a_press_closely },
        { "rolling_spread_reports_the_standard_deviation_of_recent_samples", &rolling_spread_reports_the_standard_deviation_of_recent_samples },
        { "tracking_settings_round_trip_through_save", &tracking_settings_round_trip_through_save },
        { "invalid_tracking_settings_are_rejected", &invalid_tracking_settings_are_rejected },
        { "missing_bridge_with_tracking_thresholds_is_safe", &missing_bridge_with_tracking_thresholds_is_safe },
        { "hand_tracking_defaults_to_mediapipe_thresholds", +[] {
              const aerial_touch::AppConfig config;
              return config.hand_tracking.min_detection_confidence == 0.5F
                     && config.hand_tracking.min_presence_confidence == 0.5F
                     && config.hand_tracking.min_tracking_confidence == 0.5F;
          } },
        { "user_side_hint_orients_the_normal_when_the_camera_is_in_the_plane", &user_side_hint_orients_the_normal_when_the_camera_is_in_the_plane },
        { "an_oblique_user_side_hint_still_decides_the_side", &an_oblique_user_side_hint_still_decides_the_side },
        { "user_side_hint_lying_in_the_plane_is_rejected", &user_side_hint_lying_in_the_plane_is_rejected },
        { "keypad_calibration_reports_an_ambiguous_user_side", &keypad_calibration_reports_an_ambiguous_user_side },
        { "collinear_points_with_a_hint_are_still_an_invalid_plane", &collinear_points_with_a_hint_are_still_an_invalid_plane },
        { "fingertip_trace_over_a_vertical_card_fits_the_plane", &fingertip_trace_over_a_vertical_card_fits_the_plane },
        { "fingertip_trace_along_one_line_is_rejected", &fingertip_trace_along_one_line_is_rejected },
        { "fingertip_extremity_finds_the_leading_edge_behind_the_landmark", &fingertip_extremity_finds_the_leading_edge_behind_the_landmark },
        { "fingertip_extremity_pulls_back_an_overshooting_landmark", &fingertip_extremity_pulls_back_an_overshooting_landmark },
        { "fingertip_extremity_ignores_flying_pixels_past_the_edge", &fingertip_extremity_ignores_flying_pixels_past_the_edge },
        { "fingertip_extremity_ignores_a_finger_at_another_depth", &fingertip_extremity_ignores_a_finger_at_another_depth },
        { "fingertip_extremity_gives_up_without_a_finger_or_a_direction", &fingertip_extremity_gives_up_without_a_finger_or_a_direction },
        { "distance_gauge_keeps_thresholds_and_the_plane_inside", &distance_gauge_keeps_thresholds_and_the_plane_inside },
        { "mounting_defaults_to_front_and_reads_overhead", &mounting_defaults_to_front_and_reads_overhead },
        { "misspelled_mounting_is_rejected", &misspelled_mounting_is_rejected },
        { "mounting_and_extremity_settings_round_trip_through_save", &mounting_and_extremity_settings_round_trip_through_save },
        { "invalid_extremity_settings_are_rejected", &invalid_extremity_settings_are_rejected },
        { "legacy_test_config_still_loads_as_front", +[] {
              return aerial_touch::load_app_config(std::filesystem::path(TEST_SOURCE_DIR) / "tests" / "data"
                                                   / "config.yaml")
                         .mounting
                     == aerial_touch::CameraMounting::Front;
          } },
    } };

    bool all_passed = true;
    for(const auto& test : tests) {
        if(!test.second()) {
            std::cerr << "FAILED: " << test.first << "\n";
            all_passed = false;
        }
    }
    return all_passed;
}
