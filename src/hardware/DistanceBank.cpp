/* DistanceBank.cpp -- sample Ports::D1..D4 into DistanceReading slots. */
#include "hardware/DistanceBank.h"
#include "config/Ports.h"
#include "core/Specs.h"
#include "pros/distance.hpp"
#include "pros/error.h"

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

namespace {
// Ticks between retries of a port that failed. The control period is 10 ms,
// so this is about half a second, and a dead port is not touched every tick.
constexpr int kRetryTicks = 50;

const std::int32_t kMinMm = static_cast<std::int32_t>(Robot::Range::MIN_M * 1000.0 + 0.5);
const std::int32_t kMaxMm = static_cast<std::int32_t>(Robot::Range::MAX_M * 1000.0 + 0.5);
}

void DistanceBank::probe() {
    pros::Distance* const sensor[kDistanceChannels] = { &d1_, &d2_, &d3_, &d4_ };
    for (int i = 0; i < kDistanceChannels; ++i) {
        online_[i] = sensor[i]->is_installed();
        retry_[i] = 0;
    }
}

void DistanceBank::read(DistanceReading (&out)[kDistanceChannels]) {
    pros::Distance* const sensor[kDistanceChannels] = { &d1_, &d2_, &d3_, &d4_ };
    for (int i = 0; i < kDistanceChannels; ++i) {
        out[i] = {};
        if (!online_[i]) {
            if (++retry_[i] < kRetryTicks) continue;
            retry_[i] = 0;
            online_[i] = sensor[i]->is_installed();
            if (!online_[i]) continue;
        }
        if (!sensor[i]->is_installed()) {
            online_[i] = false;
            retry_[i] = 0;
            continue;
        }
        const std::int32_t mm = sensor[i]->get();
        if (mm == PROS_ERR) {
            online_[i] = false;
            retry_[i] = 0;
            continue;
        }
        // Confidence is a second port transaction. Skip it when the range
        // is already outside the window, including a 9999 mm no-return.
        const std::int32_t conf = (mm >= kMinMm && mm <= kMaxMm)
            ? sensor[i]->get_confidence() : 0;
        out[i] = distanceFromRaw(mm, conf);
    }
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
