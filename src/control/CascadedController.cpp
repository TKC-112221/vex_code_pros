/* ---------------------------------------------------------------------------
 * CascadedController.cpp  --  control math ported from v1 lib_control.cpp.
 * Behavior-preserving; the only structural change is that the final per-wheel
 * mix and per-actuator safety now live in the DriveBase, so this returns a
 * chassis-frame ChassisCommand{linear, angular} instead of {left, right}.
 * ------------------------------------------------------------------------- */
#include "control/CascadedController.h"
#include "math/MathUtils.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include "config/Settings.h"
#include <algorithm>
#include <cmath>

using mathutil::clamp_val;
using mathutil::normalize_angle;
using mathutil::sinc;

void CascadedController::init() {
    double dt = System::DT_SEC;
    // EKF process noise (higher = trust model less) / measurement noise.
    double q_pos = 1e-5, q_vel = 1e-4, q_acc = 0.1;
    double r_pos = 0.002 * 0.002, r_vel = 0.04 * 0.04, r_acc_cmd = 2.0 * 2.0;
    ekf_.init(dt, q_pos, q_vel, q_acc, r_pos, r_vel, r_acc_cmd);
}

void CascadedController::reset() {
    integral_error_lin = prev_error_lin = 0;
    integral_error_ang = prev_error_ang = 0;
    integral_error_lin_pos = prev_error_lin_pos = 0;
    integral_error_ang_pos = prev_error_ang_pos = 0;
    prev_v_cmd = prev_omega_cmd = 0;
    ekf_.reset();
}

ChassisCommand CascadedController::computeCommand(const Pose2d& pose_in,
                                                  const MotionTarget& target_in,
                                                  const ChassisObservation& obs) {
    const Pose2d pose = poseInchesToMeters(pose_in);
    MotionTarget target = target_in;
    target.pose = poseInchesToMeters(target_in.pose);
    namespace CT = Settings::ControlToggles;

    // REPLAY_VOLTAGE_FALLBACK: emit the recorded chassis effort directly.
    if (target.passthrough_effort) {
        spatial_error_ = std::sqrt((target.pose.x - pose.x) * (target.pose.x - pose.x) +
                                   (target.pose.y - pose.y) * (target.pose.y - pose.y));
        angular_error_ = normalize_angle(target.pose.theta - pose.theta);
        time_warp_ = 1.0;
        ChassisCommand c;
        c.linear  = target.ref_linear_effort;
        c.lateral = target.ref_lateral_effort;
        c.angular = target.ref_angular_effort;
        return c;
    }

    const double dt = System::DT_SEC;
    const double omega_ref = target.velocity.omega;
    const double current_lin_vel = obs.v_lin;
    const double current_ang_vel = obs.v_ang;

    // Command-clamp ceilings evaluated at the current operating point (static
    // constants by default, or physics-predicted if a PhysicsMotionLimiter is
    // wired in).
    const LimiterContext lim{ current_lin_vel, current_ang_vel, obs.battery_voltage };
    const double max_vel_lin = limiter_->maxLinearVelocity(lim);
    const double max_vel_ang = limiter_->maxAngularVelocity(lim);
    const double max_acc_lin = limiter_->maxLinearAccel(lim);
    const double max_acc_ang = limiter_->maxAngularAccel(lim);

    // 1. EKF: predict with previous command accel, correct with measurements.
    double prev_a = ekf_.get_a_cmd_est(), prev_alpha = ekf_.get_alpha_cmd_est();
    ekf_.predict(dt, prev_a, prev_alpha);
    ekf_.update_pose(pose.x, pose.y, pose.theta);
    ekf_.update_velocity(current_lin_vel, current_ang_vel);

    // 2. Pose error, transformed into the robot frame.
    double ex = target.pose.x - pose.x;
    double ey = target.pose.y - pose.y;
    double e_theta = normalize_angle(target.pose.theta - pose.theta);
    double cos_t = cos(pose.theta), sin_t = sin(pose.theta);
    double e_x_robot =  cos_t * ex + sin_t * ey;
    double e_y_robot = -sin_t * ex + cos_t * ey;

    // 3. Spatial warp (slow down when far from target; off in teleop).
    double spatial_error = std::sqrt(ex * ex + ey * ey);
    double s_factor = (CT::ENABLE_SPATIAL_WARP && spatial_error >= Gains::SPATIAL_WARP_M)
        ? (Gains::SPATIAL_WARP_M / spatial_error) : 1.0;
    double v_ref_warped = target.velocity.vx * s_factor;
    double omega_ref_warped = omega_ref * s_factor;
    double s_sq = s_factor * s_factor;

    // 4. Velocity setpoints: Ramsete OR pure reference + position PID.
    double v_cmd, omega_cmd;
    if (CT::RAMSETE) {
        integral_error_lin_pos = prev_error_lin_pos = 0;
        integral_error_ang_pos = prev_error_ang_pos = 0;
        double v_ref_for_gain = (std::fabs(target.velocity.vx) < Gains::Ramsete::SPEED_FLOOR)
            ? Gains::Ramsete::SPEED_FLOOR : target.velocity.vx;
        double omega_ref_for_gain = (std::fabs(omega_ref) < Gains::Ramsete::SPEED_FLOOR)
            ? Gains::Ramsete::SPEED_FLOOR : omega_ref;
        double k = (2.0 * Gains::Ramsete::ZETA) *
                   std::sqrt(Gains::Ramsete::B * std::pow(v_ref_for_gain, 2) + std::pow(omega_ref_for_gain, 2));
        v_cmd = v_ref_warped * cos(e_theta) + k * e_x_robot;
        omega_cmd = omega_ref_warped + k * e_theta + Gains::Ramsete::B * v_ref_warped * sinc(e_theta) * e_y_robot;
    } else {
        v_cmd = v_ref_warped;
        omega_cmd = omega_ref_warped;
        if (CT::POSITION_PID_LIN || CT::POSITION_PID_ANG) {
            double ekf_x, ekf_y, ekf_theta;
            ekf_.get_pose_est(ekf_x, ekf_y, ekf_theta);
            double ex_ekf = target.pose.x - ekf_x, ey_ekf = target.pose.y - ekf_y;
            double e_theta_ekf = normalize_angle(target.pose.theta - ekf_theta);
            double e_x_robot_pos = cos(ekf_theta) * ex_ekf + sin(ekf_theta) * ey_ekf;
            double e_theta_pos = e_theta_ekf;
            if (CT::POSITION_PID_LIN) {
                integral_error_lin_pos = integral_error_lin_pos * PidGains::Position::Linear.iDecay + e_x_robot_pos * dt;
                integral_error_lin_pos = clamp_val(integral_error_lin_pos,
                    -PidGains::Position::INTEGRAL_LIMIT, PidGains::Position::INTEGRAL_LIMIT);
                double deriv = (e_x_robot_pos - prev_error_lin_pos) / dt;
                prev_error_lin_pos = e_x_robot_pos;
                double corr = PidGains::Position::Linear.kP * e_x_robot_pos
                            + PidGains::Position::Linear.kI * integral_error_lin_pos
                            + PidGains::Position::Linear.kD * deriv;
                v_cmd += clamp_val(corr, -max_vel_lin, max_vel_lin);
            }
            if (CT::POSITION_PID_ANG) {
                integral_error_ang_pos = integral_error_ang_pos * PidGains::Position::Angular.iDecay + e_theta_pos * dt;
                integral_error_ang_pos = clamp_val(integral_error_ang_pos,
                    -PidGains::Position::INTEGRAL_LIMIT, PidGains::Position::INTEGRAL_LIMIT);
                double deriv = (e_theta_pos - prev_error_ang_pos) / dt;
                prev_error_ang_pos = e_theta_pos;
                double corr = PidGains::Position::Angular.kP * e_theta_pos
                            + PidGains::Position::Angular.kI * integral_error_ang_pos
                            + PidGains::Position::Angular.kD * deriv;
                omega_cmd += clamp_val(corr, -max_vel_ang, max_vel_ang);
            }
        }
        if (!CT::POSITION_PID_LIN) { integral_error_lin_pos = prev_error_lin_pos = 0; }
        if (!CT::POSITION_PID_ANG) { integral_error_ang_pos = prev_error_ang_pos = 0; }
    }

    // 5. Feedforward: EKF-smoothed command accelerations.
    double raw_a_cmd = clamp_val((v_cmd - prev_v_cmd) / dt, -max_acc_lin, max_acc_lin);
    double raw_alpha_cmd = clamp_val((omega_cmd - prev_omega_cmd) / dt, -max_acc_ang, max_acc_ang);
    prev_v_cmd = v_cmd;
    prev_omega_cmd = omega_cmd;
    ekf_.update_accel_cmd(raw_a_cmd, raw_alpha_cmd);
    double a_cmd = ekf_.get_a_cmd_est();
    double alpha_cmd = ekf_.get_alpha_cmd_est();

    double vol_lin_ff, vol_lat_ff, vol_ang_ff;
    if (CT::USE_RECORDED_VOLTS) {
        // Recorded chassis effort as feedforward base (0 in teleop -> PID-only).
        vol_lin_ff = target.ref_linear_effort;
        vol_lat_ff = target.ref_lateral_effort;
        vol_ang_ff = target.ref_angular_effort;
    } else {
        double turn_radius = (std::fabs(omega_cmd) < Physics::OMEGA_EPS) ? Physics::TURN_RADIUS_MAX_M
                           : std::min(std::fabs(v_cmd) / (std::fabs(omega_cmd) + Physics::OMEGA_EPS), Physics::TURN_RADIUS_MAX_M);
        vol_lin_ff = feedforward_->linearVolts(v_cmd, a_cmd * s_sq, std::fabs(v_cmd),
                                               std::fabs(a_cmd * s_sq), omega_cmd, turn_radius);
        vol_lat_ff = 0.0;  // no lateral dynamics model yet
        vol_ang_ff = feedforward_->angularVolts(omega_cmd, alpha_cmd * s_sq, std::fabs(v_cmd),
                                                std::fabs(a_cmd * s_sq), turn_radius);
    }

    // 6. Velocity PID feedback on EKF-smoothed velocity/derivatives.
    double v_est = ekf_.get_v_est();
    double omega_est = ekf_.get_omega_est();
    double v_error = v_cmd - v_est;
    integral_error_lin = integral_error_lin * PidGains::Velocity::Linear.iDecay + v_error * dt;
    integral_error_lin = clamp_val(integral_error_lin, -5.0, 5.0);
    double deriv_lin = ekf_.get_deriv_lin();
    prev_error_lin = v_error;

    double omega_error = omega_cmd - omega_est;
    integral_error_ang = integral_error_ang * PidGains::Velocity::Angular.iDecay + omega_error * dt;
    integral_error_ang = clamp_val(integral_error_ang, -5.0, 5.0);
    double deriv_ang = ekf_.get_deriv_ang();
    prev_error_ang = omega_error;

    double vol_pid_lin = PidGains::Velocity::Linear.kP * v_error
                       + PidGains::Velocity::Linear.kI * integral_error_lin
                       + PidGains::Velocity::Linear.kD * deriv_lin;
    double vol_pid_ang = PidGains::Velocity::Angular.kP * omega_error
                       + PidGains::Velocity::Angular.kI * integral_error_ang
                       + PidGains::Velocity::Angular.kD * deriv_ang;

    // 7. Combine feedforward + feedback into a chassis-frame effort.
    //    (Angular effort predicts V_R - V_L, halved to apply per side downstream.)
    ChassisCommand cmd;
    cmd.linear  = vol_lin_ff * Gains::FF_BOOST_LIN + vol_pid_lin;
    cmd.lateral = vol_lat_ff;  // passthrough recorded lateral; no lateral PID yet
    cmd.angular = (vol_ang_ff * Gains::FF_BOOST_ANG + vol_pid_ang) / 2.0;
    // Stick-zero brake is a per-actuator hardware behavior; the DriveBase
    // decides it from joystick idle + wheel velocity (see TankDrive).

    spatial_error_ = spatial_error;
    angular_error_ = e_theta;
    time_warp_ = s_factor;
    return cmd;
}
