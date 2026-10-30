#pragma once
/* ---------------------------------------------------------------------------
 * DistanceBank  --  the four VEX distance sensors on Ports::D1..D4.
 *
 * D1 front, D2 right, D3 rear, D4 left. Construction claims those ports.
 * probe() records which ports actually have a distance sensor. A port that
 * does not is left unread, so a failed bring-up cannot stall the control
 * tick. read() writes the sensor dict slots in that order. A sample is
 * valid only inside Robot::Range::MIN_M..MAX_M; no-return (9999 mm) and
 * PROS_ERR are left valid=false.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "pros/distance.hpp"
#include <cstdint>

// mm is the raw pros::Distance reading. confidence is 0..63 (10 below 200 mm).
DistanceReading distanceFromRaw(std::int32_t mm, std::int32_t confidence);

class DistanceBank {
public:
    DistanceBank();

    // One pass over the port registry. Does not read a beam and does not
    // wait. A port that is not a distance sensor stays offline.
    void probe();

    void read(DistanceReading (&out)[kDistanceChannels]);

private:
    pros::Distance d1_;
    pros::Distance d2_;
    pros::Distance d3_;
    pros::Distance d4_;
    // False after probe() or after a read that returns PROS_ERR. read()
    // does not call into an offline port on the control tick.
    bool online_[kDistanceChannels] = { true, true, true, true };
    int retry_[kDistanceChannels] = {};
};
