#pragma once
/* ---------------------------------------------------------------------------
 * StaticMotionLimiter  --  the default command clamp: fixed, tuned ceilings
 * from the constants (Robot::MAX_VEL_*, Physics::MAX_ACCEL_*). Ignoring the
 * operating point, this reproduces v1's behavior exactly.
 *
 * Also provides defaultMotionLimiter(): a process-wide Static instance used as
 * the default injection for the controller and teleop generator, so the
 * out-of-the-box build is byte-for-byte identical to the hardcoded limits.
 * ------------------------------------------------------------------------- */

#include "interfaces/IMotionLimiter.h"
#include "core/Specs.h"
#include "core/Limits.h"

class StaticMotionLimiter : public IMotionLimiter {
public:
    double maxLinearVelocity(const LimiterContext&) const override  { return Robot::MAX_VEL_LIN; }
    double maxLinearAccel(const LimiterContext&) const override     { return Physics::MAX_ACCEL_LIN; }
    double maxAngularVelocity(const LimiterContext&) const override { return Robot::MAX_VEL_ANG; }
    double maxAngularAccel(const LimiterContext&) const override    { return Physics::MAX_ACCEL_ANG; }
};

// Process-wide default (Meyers singleton): the hardcoded-limit clamp.
inline IMotionLimiter& defaultMotionLimiter() {
    static StaticMotionLimiter instance;
    return instance;
}
