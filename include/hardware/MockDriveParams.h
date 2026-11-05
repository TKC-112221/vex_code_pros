#pragma once
/* ---------------------------------------------------------------------------
 * MockDriveParams.h  --  the simulated plant. Robot tunables live in
 * core/Tuning.h. These numbers exist only in the host simulator.
 *
 * The plant is a DC motor on a rigid chassis, not a voltage lag. Mass,
 * top speed, stall acceleration, and friction are the values a log fit
 * moves. USE_TUNING_PASTE replaces them with python/tuning_paste.txt.
 * The fallback is the same chassis the feedforward was written for:
 * steady speed LIN_SPEED_AT_SAT, launch accel (V - kS) / kA, and the
 * Coulomb force kS * m / kA.
 * ------------------------------------------------------------------------- */

#include "config/Settings.h"
#include "core/Specs.h"
#include "core/Tuning.h"

namespace MockPlant {
#if USE_TUNING_PASTE
#define TUNING_PASTE_PART 1
#include "data/tuning_paste.inc"
#else
    // Dressed V5 robot. Wall impulses and the Newton step both use it.
    constexpr double MASS_KG = 6.0;
    // Steady forward speed at System::MAX_VOLTS, both sides, after friction.
    constexpr double MAX_VEL_MPS = Feedforward::LIN_SPEED_AT_SAT;
    // Forward accel from a stop at that voltage, after static friction.
    constexpr double DRIVE_ACCEL_MPS2 =
        (System::MAX_VOLTS - Feedforward::KS_LIN) / Feedforward::KA_LIN;
    // Total Coulomb force opposing forward motion, both sides together.
    constexpr double STATIC_FRICTION_N =
        Feedforward::KS_LIN * MASS_KG / Feedforward::KA_LIN;
    // 1 is a uniform rectangle of BODY_LENGTH_M by TRACK_WIDTH_M.
    constexpr double YAW_INERTIA_SCALE = 1.0;
    // Torque to start a spin. The default is the same friction, at the
    // wheels: each side carries half the force, one track-half out.
    constexpr double YAW_FRICTION_NM =
        STATIC_FRICTION_N * Robot::Chassis::TRACK_WIDTH_M * 0.5;
    // Roller drag. Lateral speed bleeds in about this many seconds.
    constexpr double SKID_TAU_SEC = 0.30;
    // Omni rollers. A small breakaway so a pure slide does stop.
    constexpr double LATERAL_FRICTION_N = 0.8;
    // Longitudinal grip, friction coefficient. Stall force of this
    // chassis sits just under μ = 1, so the nominal robot does not slip.
    constexpr double TRACTION_MU = 1.0;
    // Three V5 motors in parallel per side, plus harness. Current uses it.
    // Torque uses it only when that current hits CURRENT_LIMIT_A.
    constexpr double SIDE_RESISTANCE_OHM = 1.6;
#endif
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

    // V5 firmware current limit, 2.5 A, three motors in parallel per side.
    constexpr double CURRENT_LIMIT_A = 7.5;
    // Gearbox and wheel, reflected to a linear mass. Slip spins this,
    // not the whole robot.
    constexpr double WHEEL_MASS_KG = 0.25;
    // Identification voltage for MAX_VEL_MPS and DRIVE_ACCEL_MPS2.
    constexpr double NOMINAL_VOLTS = System::MAX_VOLTS;

    constexpr int CONTACT_SUBSTEPS = 4;
}
