#pragma once

#include "aerial_touch/types.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace aerial_touch {

// MediaPipe Hand Landmarker thresholds. Its defaults are 0.5 for all three. The back of a hand seen
// from above scores lower than a palm, and in VIDEO mode a hand whose presence score falls below
// `min_presence_confidence` is dropped and can only come back through palm detection -- which is
// exactly what struggles with the back of the hand. Lowering presence and tracking keeps a hand
// that has been found; detection stays higher so that nothing is invented from nothing.
struct HandTrackingConfig {
    float min_detection_confidence{ 0.5F };
    float min_presence_confidence{ 0.5F };
    float min_tracking_confidence{ 0.5F };
};

struct HandObservation {
    bool detected{ false };
    int landmark_count{ 0 };
    std::array<Vec3, 21> landmarks{};
};

class HandTracker {
public:
    // A bridge DLL built before the thresholds became configurable still loads; it then runs with
    // MediaPipe's defaults and `warning()` says so.
    HandTracker(const std::filesystem::path& dll_path,
                const std::filesystem::path& model_path,
                HandTrackingConfig tracking = {});
    ~HandTracker();

    HandTracker(const HandTracker&) = delete;
    HandTracker& operator=(const HandTracker&) = delete;
    HandTracker(HandTracker&&) noexcept;
    HandTracker& operator=(HandTracker&&) noexcept;

    bool available() const;
    const std::string& error() const;
    const std::string& warning() const;
    HandObservation detect_rgb(const unsigned char* rgb,
                               int width,
                               int height,
                               int stride_bytes,
                               std::int64_t timestamp_ms);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace aerial_touch
