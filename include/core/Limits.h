#pragma once
/* ---------------------------------------------------------------------------
 * Limits.h  --  ML-identified dynamics limits (accel ceilings, slew steps,
 * turn-radius clamp). Wraps the auto-generated data/generated_limits.h and
 * provides fallback defaults if it is ever absent. Do not edit the generated
 * file directly -- it comes from utils/train_dynamics.py.
 * ------------------------------------------------------------------------- */

#if __has_include("data/generated_limits.h")
    #include "data/generated_limits.h"
#endif

namespace Physics {
#if __has_include("data/generated_limits.h")
    constexpr double MAX_ACCEL_LIN    = Limits::MAX_ACCEL_LIN;
    constexpr double MAX_ACCEL_ANG    = Limits::MAX_ACCEL_ANG;
    constexpr double SLEW_STEP_LIN    = Limits::SLEW_STEP_LIN;
    constexpr double SLEW_STEP_ANG    = Limits::SLEW_STEP_ANG;
    constexpr double TURN_RADIUS_MAX_M = Limits::TURN_RADIUS_MAX_M;
#else
    #warning "data/generated_limits.h not found! Using fallback defaults."
    constexpr double MAX_ACCEL_LIN    = 2.0;
    constexpr double MAX_ACCEL_ANG    = 8.0;
    constexpr double SLEW_STEP_LIN    = 0.02;
    constexpr double SLEW_STEP_ANG    = 0.08;
    constexpr double TURN_RADIUS_MAX_M = 10.0;
#endif
}
