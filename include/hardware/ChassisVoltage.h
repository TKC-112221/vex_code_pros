#pragma once
/* ---------------------------------------------------------------------------
 * ChassisVoltage  --  the only chassis-voltage calculation.
 *
 * The control input is a velocity: forward and lateral in m/s, yaw in
 * rad/s. Acceleration is the change in that velocity, either the one the
 * planner already computed or the step from the previous command. This
 * turns that velocity into volts with the drive's VAS model,
 * kS sgn(v) + kV v + kA a. A recorded Volts command is motor voltage
 * already and is not run through the model. TankDrive, MockDrive, XDrive,
 * and ParenthesisDrive are the only callers.
 * ------------------------------------------------------------------------- */

#include "core/Tuning.h"
#include "core/Types.h"
#include "interfaces/IFeedForward.h"
#include "math/MathUtils.h"
#include <cmath>

struct ChassisVoltageState {
    double v = 0.0;
    double w = 0.0;
    bool primed = false;
};

// Keep every wheel at or under the gearbox free speed. The scale is the
// chassis geometry, not a gain.
inline void limitWheelSpeed(double& v, double& vy, double& w) {
    const double half = Robot::Chassis::TRACK_WIDTH_M * 0.5;
    const double cap = Robot::MAX_VEL_LIN;
    const double wheels[4] = {
        v - vy - w * half,
        v + vy + w * half,
        v + vy - w * half,
        v - vy + w * half,
    };
    double peak = 0.0;
    for (double speed : wheels) {
        const double mag = std::fabs(speed);
        if (mag > peak) peak = mag;
    }
    if (peak > cap && peak > 1e-9) {
        const double scale = cap / peak;
        v *= scale;
        vy *= scale;
        w *= scale;
    }
}

inline ChassisCommand chassisVoltage(const ChassisCommand& cmd, const ChassisObservation& /*obs*/,
                                     const IFeedForward* model, ChassisVoltageState& held) {
    if (cmd.request == DriveRequest::Volts) {
        ChassisCommand out = cmd;
        out.accel = 0.0;
        out.alpha = 0.0;
        out.pivot = false;
        out.request = DriveRequest::Volts;
        return out;
    }

    double v = std::isfinite(cmd.linear) ? cmd.linear : 0.0;
    double vy = std::isfinite(cmd.lateral) ? cmd.lateral : 0.0;
    double w = std::isfinite(cmd.angular) ? cmd.angular : 0.0;
    limitWheelSpeed(v, vy, w);

    double a = std::isfinite(cmd.accel) ? cmd.accel : 0.0;
    double alpha = std::isfinite(cmd.alpha) ? cmd.alpha : 0.0;
    // A stick or any other caller that only set a velocity. The planner's
    // own accel is kept when it sent one.
    if (a == 0.0 && alpha == 0.0 && held.primed && System::DT_SEC > 1e-6) {
        a = (v - held.v) / System::DT_SEC;
        alpha = (w - held.w) / System::DT_SEC;
    }
    if (!std::isfinite(a)) a = 0.0;
    if (!std::isfinite(alpha)) alpha = 0.0;
    held.v = v;
    held.w = w;
    held.primed = true;

    double surge = 0.0;
    double sway = 0.0;
    double yaw = 0.0;
    if (model != nullptr) {
        surge = model->linearVolts(v, a, std::fabs(v), std::fabs(a), w, Physics::TURN_RADIUS_MAX_M);
        sway = model->linearVolts(vy, 0.0, std::fabs(vy), 0.0, 0.0, Physics::TURN_RADIUS_MAX_M);
        yaw = 0.5 * model->angularVolts(w, alpha, std::fabs(v), std::fabs(a), Physics::TURN_RADIUS_MAX_M);
    } else {
        surge = Feedforward::KS_LIN * mathutil::sign(v) + Feedforward::KV_LIN * v
              + Feedforward::KA_LIN * a;
        sway = Feedforward::KS_LIN * mathutil::sign(vy) + Feedforward::KV_LIN * vy;
        yaw = 0.5 * (Feedforward::KS_ANG * mathutil::sign(w) + Feedforward::KV_ANG * w
                   + Feedforward::KA_ANG * alpha);
    }
    if (!std::isfinite(surge)) surge = 0.0;
    if (!std::isfinite(sway)) sway = 0.0;
    if (!std::isfinite(yaw)) yaw = 0.0;

    const double max_v = System::MAX_VOLTS;
    surge = mathutil::clamp_val(surge, -max_v, max_v);
    sway = mathutil::clamp_val(sway, -max_v, max_v);
    yaw = mathutil::clamp_val(yaw, -max_v, max_v);
    // Yaw keeps the rail. Leftover voltage is what remains for translation.
    const double overturn = std::fabs(yaw) + std::fabs(surge) + std::fabs(sway) - max_v;
    if (overturn > 0.0) {
        const double trans = std::fabs(surge) + std::fabs(sway);
        if (trans > 1e-9) {
            const double scale = std::max(0.0, trans - overturn) / trans;
            surge *= scale;
            sway *= scale;
        }
    }

    ChassisCommand out;
    out.linear = surge;
    out.lateral = sway;
    out.angular = yaw;
    out.brake = cmd.brake;
    out.request = DriveRequest::Volts;
    return out;
}
