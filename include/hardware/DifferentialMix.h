#pragma once
/* ---------------------------------------------------------------------------
 * DifferentialMix  --  MAIN1/MAIN2 bevel differential.
 *
 * The same voltage on both motors turns the cascade. Opposite voltages turn
 * the intake. Asking for both at once shares the 12 V between them.
 *
 * kCascadeDirection +1 means positive cascade percent raises. Flip it if that
 * pair drives the cascade down. kIntakeDirection flips which way "in" spins.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "pros/error.h"
#include "pros/motors.hpp"
#include <cmath>
#include <cstdint>
#include <cstdlib>

constexpr int kCascadeDirection = 1;
constexpr int kIntakeDirection = 1;

struct MainPairPct {
    int main1_pct = 0;
    int main2_pct = 0;
};

inline MainPairPct mixMainPair(int cascade_pct, int intake_pct) {
    const int cascade = cascade_pct * kCascadeDirection;
    const int intake = intake_pct * kIntakeDirection;
    int main1 = cascade + intake;
    int main2 = cascade - intake;
    const int peak = std::abs(main1) > std::abs(main2) ? std::abs(main1) : std::abs(main2);
    if (peak > 100) {
        main1 = main1 * 100 / peak;
        main2 = main2 * 100 / peak;
    }
    return MainPairPct{main1, main2};
}

inline CascadeReading readMainPair(const pros::Motor& main1, const pros::Motor& main2) {
    auto finite = [](double value) { return std::isfinite(value) ? value : 0.0; };
    auto amps = [](std::int32_t milliamps) {
        if (milliamps == PROS_ERR || milliamps < 0) return 0.0;
        return static_cast<double>(milliamps) / 1000.0;
    };
    const double sign = kCascadeDirection < 0 ? -1.0 : 1.0;
    CascadeReading reading;
    reading.position_deg = sign * (finite(main1.get_position()) + finite(main2.get_position())) * 0.5;
    reading.velocity_rpm = sign * (finite(main1.get_actual_velocity()) + finite(main2.get_actual_velocity())) * 0.5;
    reading.current_amps = (amps(main1.get_current_draw()) + amps(main2.get_current_draw())) * 0.5;
    reading.torque_nm = sign * (finite(main1.get_torque()) + finite(main2.get_torque())) * 0.5;
    return reading;
}
