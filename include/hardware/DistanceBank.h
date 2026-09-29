#pragma once
/* ---------------------------------------------------------------------------
 * DistanceBank  --  the four VEX distance sensors on Ports::D1..D4.
 *
 * D1 front, D2 right, D3 rear, D4 left. read() writes the sensor dict slots
 * in that order. A sample is valid only inside Robot::Range::MIN_M..MAX_M;
 * no-return (9999 mm) and PROS_ERR are left valid=false.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "pros/distance.hpp"
#include <cstdint>

// mm is the raw pros::Distance reading. confidence is 0..63 (10 below 200 mm).
DistanceReading distanceFromRaw(std::int32_t mm, std::int32_t confidence);

class DistanceBank {
public:
    DistanceBank();

    void read(DistanceReading (&out)[kDistanceChannels]);

private:
    pros::Distance d1_;
    pros::Distance d2_;
    pros::Distance d3_;
    pros::Distance d4_;
};
