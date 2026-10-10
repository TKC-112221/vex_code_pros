#pragma once
/* ---------------------------------------------------------------------------
 * MockDriveParams.h  --  the simulated plant. Robot tunables live in
 * core/Tuning.h. These numbers exist only in the host simulator.
 * ------------------------------------------------------------------------- */

namespace MockPlant {
    // First-order lag from commanded voltage to wheel speed.
    constexpr double VELOCITY_TAU_SEC = 0.15;
    // How fast roller friction bleeds off sideways speed.
    constexpr double SKID_TAU_SEC = 0.3;
    // Three V5 motors in parallel per side.
    constexpr double SIDE_RESISTANCE_OHM = 1.6;
    // Multiplier on Robot::MAX_VEL_LIN. 1 keeps the spec sheet speed.
    constexpr double SPEED_SCALE = 1.0;
    // 10 ms loop * 3 = 30 ms, next to the distance sensor's 33 ms budget.
    constexpr int RANGE_PERIOD_TICKS = 3;

    // Stand-in for VEX distance confidence (0–63). Not a published curve.
    constexpr double CONF_FULL = 62.0;
    constexpr double CONF_DROP = 40.0;
    constexpr double CONF_SIGMA = 2.0;
    constexpr int CONF_MIN = 1;
    constexpr int CONF_MAX = 63;

    // Reported motor temperature. The plant does not model heating.
    constexpr double REPORTED_TEMP_C = 35.0;
    // Voltages at or below this are not a usable full-scale reference.
    constexpr double MIN_FULL_SCALE_V = 1.0;
}
