/* DistanceBank.cpp -- sample Ports::D1..D4 into DistanceReading slots. */
#include "hardware/DistanceBank.h"
#include "config/Ports.h"
#include "core/Specs.h"
#include "pros/distance.hpp"

DistanceReading distanceFromRaw(std::int32_t mm, std::int32_t confidence) {
    DistanceReading r;
    // VEX reports 9999 mm when the beam does not return, and PROS_ERR (INT32_MAX)
    // when the port read fails. Both fall outside the usable window.
    const auto min_mm = static_cast<std::int32_t>(Robot::Range::MIN_M * 1000.0 + 0.5);
    const auto max_mm = static_cast<std::int32_t>(Robot::Range::MAX_M * 1000.0 + 0.5);
    if (mm < min_mm || mm > max_mm) return r;
    int conf = static_cast<int>(confidence);
    if (conf < 0 || conf > 63) conf = 0;
    r.range_m = static_cast<double>(mm) / 1000.0;
    r.confidence = conf;
    r.valid = true;
    return r;
}

DistanceBank::DistanceBank()
    : d1_(static_cast<std::uint8_t>(Ports::D1)),
      d2_(static_cast<std::uint8_t>(Ports::D2)),
      d3_(static_cast<std::uint8_t>(Ports::D3)),
      d4_(static_cast<std::uint8_t>(Ports::D4)) {}

void DistanceBank::read(DistanceReading (&out)[kDistanceChannels]) {
    const std::int32_t mm[kDistanceChannels] = {
        d1_.get(), d2_.get(), d3_.get(), d4_.get()
    };
    const std::int32_t conf[kDistanceChannels] = {
        d1_.get_confidence(), d2_.get_confidence(), d3_.get_confidence(), d4_.get_confidence()
    };
    for (int i = 0; i < kDistanceChannels; ++i) out[i] = distanceFromRaw(mm[i], conf[i]);
}

#if defined(_MSC_VER)
// Host link: the PROS firmware library is not on this target.
pros::v5::Distance::Distance(const std::uint8_t port)
    : Device(port, DeviceType::distance) {}
std::int32_t pros::v5::Distance::get() { return 9999; }
std::int32_t pros::v5::Distance::get_distance() { return 9999; }
std::int32_t pros::v5::Distance::get_confidence() { return 0; }
std::int32_t pros::v5::Distance::get_object_size() { return 0; }
double pros::v5::Distance::get_object_velocity() { return 0.0; }
#endif
