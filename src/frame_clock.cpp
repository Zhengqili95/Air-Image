#include "aerial_touch/frame_clock.hpp"

namespace aerial_touch {

std::int64_t FrameClock::next(const std::uint64_t device_us, const std::uint64_t host_us) {
    if(!using_host_clock_) {
        const bool device_advanced =
            device_us != 0U && (!last_device_us_.has_value() || device_us > *last_device_us_);
        if(device_advanced) {
            last_device_us_ = device_us;
        }
        else {
            using_host_clock_ = true;
            // Carry on from where the device timeline stopped. The host clock has its own epoch and
            // may read smaller, which would otherwise stall time at one millisecond per frame.
            if(last_ms_.has_value()) {
                host_offset_ms_ = *last_ms_ + 1 - static_cast<std::int64_t>(host_us / 1000U);
            }
        }
    }

    std::int64_t timestamp_ms = using_host_clock_
                                    ? static_cast<std::int64_t>(host_us / 1000U) + host_offset_ms_
                                    : static_cast<std::int64_t>(device_us / 1000U);
    if(last_ms_.has_value() && timestamp_ms <= *last_ms_) {
        timestamp_ms = *last_ms_ + 1;
    }
    last_ms_ = timestamp_ms;
    return timestamp_ms;
}

bool FrameClock::using_host_clock() const {
    return using_host_clock_;
}

}  // namespace aerial_touch
