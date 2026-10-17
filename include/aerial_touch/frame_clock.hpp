#pragma once

#include <cstdint>
#include <optional>

namespace aerial_touch {

// Turns per-frame camera timestamps into the timeline everything downstream depends on. MediaPipe's
// VIDEO running mode rejects any frame whose timestamp does not strictly increase, and the One Euro
// filter, approach velocity, dwell and surface-scan timeouts all measure elapsed time from it.
//
// The device clock is preferred because it marks when the frame was captured. On Windows it only
// advances once the UVC metadata has been registered (OrbbecSDK shared/obsensor_metadata_win10.ps1);
// without that it stays put, which froze every timer and made MediaPipe reject every frame. The first
// time the device clock fails to advance, the clock switches to the host clock for the rest of the
// session and never switches back: alternating between two clock domains would make time jump.
class FrameClock {
public:
    // Returns a timestamp in milliseconds that is strictly greater than the previous one.
    std::int64_t next(std::uint64_t device_us, std::uint64_t host_us);
    bool using_host_clock() const;

private:
    bool using_host_clock_{ false };
    std::optional<std::uint64_t> last_device_us_;
    std::optional<std::int64_t> last_ms_;
    std::int64_t host_offset_ms_{ 0 };
};

}  // namespace aerial_touch
