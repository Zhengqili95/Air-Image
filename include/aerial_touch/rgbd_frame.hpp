#pragma once

#include "aerial_touch/types.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace aerial_touch {

struct CameraIntrinsics {
    float fx{};
    float fy{};
    float cx{};
    float cy{};
    int width{};
    int height{};
};

struct CameraExtrinsics {
    std::array<float, 9> rotation{};
    std::array<float, 3> translation_mm{};
};

// Where capture() spent its time, in milliseconds, for the HUD's timing breakdown.
struct CaptureTimings {
    float wait_ms{};    // blocked until the SDK had a frameset
    float drain_ms{};   // skipping older framesets queued behind it
    float filter_ms{};  // alignment fallback and the SDK depth filters
    float copy_ms{};    // copying pixels and reading calibration into RgbdFrame
};

struct RgbdFrame {
    std::vector<std::uint8_t> rgb;
    std::vector<std::uint16_t> raw_depth;
    std::vector<std::uint16_t> depth;
    int color_width{};
    int color_height{};
    int depth_width{};
    int depth_height{};
    std::int64_t timestamp_ms{};
    // Host-side lag before capture() handed the frame over: how long it had been waiting since the
    // SDK received it (-1 when the SDK gave no usable host time), and how many older framesets were
    // discarded so that the newest one is processed.
    std::int64_t host_age_ms{ -1 };
    std::uint32_t dropped_framesets{};
    CaptureTimings capture_timings;
    float depth_unit_mm{};
    CameraIntrinsics depth_intrinsics;
    CameraExtrinsics depth_to_color;
    bool profiles_valid{ false };

    bool valid() const {
        return color_width > 0 && color_height > 0 && color_width == depth_width && color_height == depth_height
               && rgb.size() == static_cast<std::size_t>(color_width) * static_cast<std::size_t>(color_height) * 3U
               && raw_depth.size() == static_cast<std::size_t>(depth_width) * static_cast<std::size_t>(depth_height)
               && depth.size() == static_cast<std::size_t>(depth_width) * static_cast<std::size_t>(depth_height)
               && depth_unit_mm > 0.0F && profiles_valid;
    }
};

}  // namespace aerial_touch
