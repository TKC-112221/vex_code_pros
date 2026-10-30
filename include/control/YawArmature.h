#pragma once
/* ---------------------------------------------------------------------------
 * YawArmature  --  a pivot from the linear kS / kV / kA model.
 *
 * The chassis yaw command is the voltage of one wheel. TankDrive sets
 * the right side to surge + yaw, so a pure spin is that wheel voltage:
 *
 *   v = ω * (track / 2)
 *   a = α * (track / 2)
 *   yaw = kS sgn(v) + kV v + kA a
 *
 * kS, kV, and kA are the linear feedforward. The angular fit is a second
 * copy of those terms, and curve tracking was free to shrink it because
 * the heading trim was already turning the robot.
 *
 * ω is the brake curve itself:
 *
 *   sign(error) * min(max vel, sqrt(2 * accel * |error|))
 *
 * It is not slewed from the measured rate. Slewing from the measurement
 * keeps the velocity term at the current speed, and kA at this accel is
 * smaller than kS + kV v, so the command never asks the wheels to slow
 * down. α is how far the measured rate sits from that setpoint, capped at
 * the same accel, and it is left off for a small trim: a few degrees would
 * otherwise take the full kA step. kS is the standing start. It fades out
 * as the measured rate climbs to the stiction speed, so the breakaway
 * voltage does not drop off in one tick, and it is gone once the pivot is
 * moving. Leaving it on would hold a drive through the stop. A heading
 * trim while the chassis is also driving passes
 * pivot = false, which keeps kS and kA off. Those two terms are a pivot,
 * and on a few degrees of error they are a volt of yaw.
 * ------------------------------------------------------------------------- */

#include "core/Specs.h"
#include "core/Tuning.h"
#include "interfaces/IFeedForward.h"
#include "math/MathUtils.h"
#include <cmath>

struct YawArmature {
    double omega = 0.0; // commanded rate, rad/s
    double alpha = 0.0; // rad/s^2, capped
};

// Below this rate a heading correction is a trim, and the static term is
// what gets a stopped pivot moving. Above it the velocity term carries the spin.
constexpr double kYawStictionRadS = 0.5;

inline YawArmature planYaw(double error_rad, double omega_now, double max_vel, double accel,
                         bool pivot) {
    if (!std::isfinite(error_rad)) error_rad = 0.0;
    if (!std::isfinite(omega_now)) omega_now = 0.0;
    if (!(accel > Turn::MIN_ACCEL)) accel = Turn::MIN_ACCEL;
    if (!(max_vel > Turn::MIN_VEL)) max_vel = Turn::MIN_VEL;

    double w_cap = max_vel;
    const double w_stop = std::sqrt(2.0 * accel * std::fabs(error_rad));
    if (w_stop < w_cap) w_cap = w_stop;
    const double omega = mathutil::sign(error_rad) * w_cap;

    double alpha = (System::DT_SEC > 1e-6) ? (omega - omega_now) / System::DT_SEC : 0.0;
    if (alpha > accel) alpha = accel;
    if (alpha < -accel) alpha = -accel;
    // A trim is the velocity term. Full accel on a fraction of a radian
    // is a volt of yaw for a degree of error.
    const bool trim = !pivot
        || (std::fabs(omega) < kYawStictionRadS && std::fabs(omega_now) < kYawStictionRadS);
    if (trim) alpha = 0.0;
    return YawArmature{omega, alpha};
}

// Same accel limit as planYaw, for a rate the caller already chose. A pivot
// that ramps its rate up to the brake curve uses this so the velocity term
// does not appear all at once.
inline YawArmature yawArmatureFromRate(double omega, double omega_now, double accel, bool pivot) {
    if (!std::isfinite(omega)) omega = 0.0;
    if (!std::isfinite(omega_now)) omega_now = 0.0;
    if (!(accel > Turn::MIN_ACCEL)) accel = Turn::MIN_ACCEL;
    double alpha = (System::DT_SEC > 1e-6) ? (omega - omega_now) / System::DT_SEC : 0.0;
    if (alpha > accel) alpha = accel;
    if (alpha < -accel) alpha = -accel;
    // A driving trim stays on the velocity term. A pivot keeps the accel
    // term through the end of the turn: cutting it off once both rates are
    // under the stiction speed drops the braking voltage in one tick.
    if (!pivot) alpha = 0.0;
    return YawArmature{omega, alpha};
}

// Chassis yaw volts for a pure spin. linearVolts is the wheel armature.
// The static term is the standing start. It fades out as the measured
// rate climbs to the stiction speed, and a trim never has it.
inline double wheelYawVolts(const IFeedForward& model, double omega, double alpha,
                            double omega_now, bool pivot) {
    const double half = Robot::Chassis::TRACK_WIDTH_M * 0.5;
    const double v = omega * half;
    const double a = alpha * half;
    const double volts = model.linearVolts(v, a, std::fabs(v), std::fabs(a), omega, half);
    const double s = mathutil::sign(v);
    if (s == 0.0) return volts;
    const double ks = model.linearVolts(s * 1e-6, 0.0, 0.0, 0.0, 0.0, half);
    double shed = 1.0;
    if (pivot && std::fabs(omega) >= kYawStictionRadS && kYawStictionRadS > 1e-6) {
        const double rate = std::fabs(omega_now) / kYawStictionRadS;
        shed = (rate < 1.0) ? rate : 1.0;
        if (shed < 0.0) shed = 0.0;
    }
    return volts - ks * shed;
}
