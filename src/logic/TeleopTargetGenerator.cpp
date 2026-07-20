/* ---------------------------------------------------------------------------
 * TeleopTargetGenerator.cpp  --  ghost-robot integration + face/pivot assists,
 * ported from v1's usercontrol teleop/sysid branches. Behavior-preserving; the
 * pivot output is expressed as a chassis command (linear/angular) that the
 * DriveBase mixes back to the exact asymmetric wheel voltages v1 produced.
 * ------------------------------------------------------------------------- */
#include "logic/TeleopTargetGenerator.h"
#include "core/Specs.h"
#include "core/Gains.h"
#include "core/Limits.h"
#include "config/Settings.h"
#include "math/MathUtils.h"
#include <algorithm>
#include <cmath>

using mathutil::clamp_val;
using mathutil::normalize_angle;

TeleopTargetGenerator::TeleopTargetGenerator(const IDriveCurve& curve, const IMotionLimiter& limiter)
    : curve_(curve), limiter_(limiter),
      slew_linear_(Physics::SLEW_STEP_LIN),
      slew_angular_(Physics::SLEW_STEP_ANG) {}

void TeleopTargetGenerator::resetToPose(const Pose2d& pose) {
    vt_x_ = pose.x; vt_y_ = pose.y; vt_theta_ = pose.theta;
    prev_target_v_ = prev_target_omega_ = 0;
    lpf_linear_.reset(0); lpf_angular_.reset(0);
    slew_linear_.reset(0); slew_angular_.reset(0);
    pivot_active_ = false; prev_a_ = prev_b_ = false;
}

TeleopDrive TeleopTargetGenerator::generate(RobotMode mode, const JoystickData& joy, const ButtonState& btn,
                                            const Pose2d& pose, const ChassisObservation& obs) {
    namespace CT = Settings::ControlToggles;
    const double DT = System::DT_SEC;

    // 1. Condition the sticks: curve -> scale -> low-pass -> slew. The velocity
    //    ceilings and slew (accel) caps come from the injected limiter, so they
    //    can be fixed constants or physics-predicted for the current state.
    LimiterContext lim{ obs.v_lin, obs.v_ang, obs.battery_voltage };
    double a3_cmd = curve_.apply(joy.forward);
    double a1_cmd = curve_.apply(joy.turn);
    double raw_v = lpf_linear_.apply(a3_cmd * limiter_.maxLinearVelocity(lim));
    double raw_w = lpf_angular_.apply(a1_cmd * limiter_.maxAngularVelocity(lim) * Tuning::TURN_INTENSITY);
    slew_linear_.setMaxStep(limiter_.maxLinearAccel(lim) * DT);
    slew_angular_.setMaxStep(limiter_.maxAngularAccel(lim) * DT);
    double target_v = slew_linear_.apply(raw_v);
    double target_w = slew_angular_.apply(raw_w);
    double target_a = (target_v - prev_target_v_) / DT;
    double target_alpha = (target_w - prev_target_omega_) / DT;
    prev_target_v_ = target_v;
    prev_target_omega_ = target_w;

    TeleopDrive result;
    result.curved_forward = a3_cmd;
    result.curved_turn = a1_cmd;

    const bool sysid = (mode == RobotMode::SYSID_DRIVE);

    // 2. Ghost pose integration (teleop/record) or pass-through (sysid).
    double tether_lin = 1.0, tether_ang = 1.0;
    if (sysid) {
        // SysID drives from the true pose (no virtual drift).
        vt_x_ = pose.x; vt_y_ = pose.y; vt_theta_ = pose.theta;
    } else {
        bool pivot_will_be_active = btn.b && (pivot_active_ || !prev_b_);
        if (pivot_will_be_active) {
            vt_x_ = pose.x; vt_y_ = pose.y; vt_theta_ = pose.theta;
        } else {
            double dist_err = std::hypot(vt_x_ - pose.x, vt_y_ - pose.y);
            double ang_err_val = std::fabs(vt_theta_ - pose.theta);
            if (CT::TETHERING) {
                tether_lin = (dist_err < 0.10) ? 1.0 : (0.10 / dist_err);
                tether_ang = (ang_err_val < 0.17) ? 1.0 : (0.17 / ang_err_val);
            }
            vt_x_ += target_v * std::cos(vt_theta_) * DT * tether_lin * tether_ang;
            vt_y_ += target_v * std::sin(vt_theta_) * DT * tether_lin * tether_ang;
            vt_theta_ += target_w * DT * tether_lin * tether_ang;
        }
    }

    // 3. Assemble the base target.
    MotionTarget target;
    target.pose = Pose2d{vt_x_, vt_y_, vt_theta_};
    target.velocity.vx = target_v * tether_lin * tether_ang;
    target.velocity.omega = target_w * tether_lin * tether_ang;
    target.accel_lin = target_a * tether_lin * tether_ang;
    target.accel_ang = target_alpha * tether_lin * tether_ang;

    // 4. Face assist (A): steer toward the nearer of heading 0 or pi.
    double err_0 = std::fabs(normalize_angle(pose.theta));
    double err_pi = std::fabs(normalize_angle(pose.theta - M_PI));
    bool at_target = (err_0 < Face::TOLERANCE_RAD) || (err_pi < Face::TOLERANCE_RAD);
    if (btn.a) {
        if (!at_target) {
            double face_theta = (err_0 < err_pi) ? 0.0 : M_PI;
            double ang_err = normalize_angle(face_theta - pose.theta);
            double omega_cmd = Face::ANG_KP * ang_err - Face::ANG_KD * obs.v_ang;
            double w_max = limiter_.maxAngularVelocity(lim);
            omega_cmd = clamp_val(omega_cmd, -w_max, w_max);
            target.velocity.vx = 0; target.velocity.omega = omega_cmd;
            target.accel_lin = 0; target.accel_ang = 0;
        } else {
            target.velocity.vx = 0; target.velocity.omega = 0;
            target.accel_lin = 0; target.accel_ang = 0;
        }
    }

    // 5. Pivot (B): direct asymmetric spin, bypassing feedback.
    ChassisCommand pivot_cmd;
    if (btn.b) {
        if (!prev_b_) pivot_active_ = true;
        if (at_target) pivot_active_ = false;
        if (pivot_active_) {
            double scale = a3_cmd * Pivot::MAX_VOLTAGE;
            double sum = Pivot::RATIO_LEFT + Pivot::RATIO_RIGHT;
            double left_v  = scale * (Pivot::RATIO_LEFT / sum);
            double right_v = scale * (Pivot::RATIO_RIGHT / sum);
            pivot_cmd.linear  = (left_v + right_v) / 2.0;
            pivot_cmd.angular = (right_v - left_v) / 2.0;
        }
    } else {
        pivot_active_ = false;
        prev_b_ = false;
    }

    // SysID collision snap: if commanded but stalled, re-anchor the ghost.
    if (sysid && pivot_active_) {
        bool collision = (std::fabs(pivot_cmd.linear - pivot_cmd.angular) > 3.0 && std::fabs(obs.left_vel_rpm) < 5.0) ||
                         (std::fabs(pivot_cmd.linear + pivot_cmd.angular) > 3.0 && std::fabs(obs.right_vel_rpm) < 5.0);
        if (collision) { vt_x_ = pose.x; vt_y_ = pose.y; vt_theta_ = pose.theta; }
    }

    prev_a_ = btn.a;
    prev_b_ = btn.b;

    // Always expose the ghost target (used for recording); pivot additionally
    // supplies a direct command that bypasses feedback for actuation.
    result.target = target;
    result.direct = pivot_active_;
    result.command = pivot_cmd;
    return result;
}
