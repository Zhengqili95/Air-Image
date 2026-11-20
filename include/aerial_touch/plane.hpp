#pragma once

#include "aerial_touch/types.hpp"

#include <optional>

namespace aerial_touch {

struct PlanePoint {
    float u_mm{};
    float v_mm{};
    float signed_distance_mm{};
};

// How far the user-side hint must lean out of the plane, as |cos| against the normal, before it is
// trusted to say which side the hand is on.
constexpr float kMinimumUserSideCosine = 0.3F;

class Plane {
public:
    // The normal is oriented so a positive signed distance means "on the user's side".
    //
    // Without `user_side_hint` that side is taken to be the one facing the camera, which is right
    // when the camera looks at the surface. When the camera sits in (or near) the plane, as with an
    // overhead camera above a vertical aerial image, the origin is almost perpendicular to the
    // normal and that test flips at random; pass a direction pointing towards the user instead.
    // A hint that lies nearly in the plane cannot decide the side and makes this return nullopt.
    static std::optional<Plane> from_calibration_points(
        Vec3 origin,
        Vec3 u_reference,
        Vec3 v_reference,
        float minimum_point_distance_mm = 80.0F,
        std::optional<Vec3> user_side_hint = std::nullopt);

    PlanePoint project(Vec3 point) const;

    // Inverse of project(): rebuild the camera-space point that sits at (u, v) on the plane,
    // offset by signed_distance_mm along the camera-facing normal. Used to draw the calibrated
    // keypad back onto the physical surface in the video frame.
    Vec3 unproject(Vec2 uv_mm, float signed_distance_mm = 0.0F) const;

private:
    Plane(Vec3 origin, Vec3 u_axis, Vec3 v_axis, Vec3 normal);

    Vec3 origin_{};
    Vec3 u_axis_{};
    Vec3 v_axis_{};
    Vec3 normal_{};
};

}  // namespace aerial_touch
