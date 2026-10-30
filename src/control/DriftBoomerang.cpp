/* ---------------------------------------------------------------------------
 * DriftBoomerang.cpp  --  boomerang carrot, centripetal sideslip, velocity command.
 * ------------------------------------------------------------------------- */
#include "control/DriftBoomerang.h"
#include "core/Tuning.h"
#include "math/MathUtils.h"

#include <cmath>

namespace {

// LemLib PID::update, derivative per tick. Windup is in the error's units.
double pidTick(double error, double kP, double kI, double kD, double windup,
               double& integral, double& prev, bool primed) {
    integral += error;
    if (primed && mathutil::sign(error) != mathutil::sign(prev)) integral = 0.0;
    if (windup != 0.0 && std::fabs(error) > windup) integral = 0.0;
    const double derivative = primed ? (error - prev) : 0.0;
    prev = error;
    return error * kP + integral * kI + derivative * kD;
}

double slewToward(double target, double current, double max_step) {
    if (max_step <= 0.0) return target;
    const double change = target - current;
    if (change > max_step) return current + max_step;
    if (change < -max_step) return current - max_step;
    return target;
}

// Carrot `lead * dist` behind the target, opposite the approach heading.
// Heading 0 is +Y and CCW is positive. There is no VEX-heading conversion.
void carrotOf(double rx, double ry, double tx, double ty, double approach,
              double lead, double& cx, double& cy) {
    const double dist = std::hypot(tx - rx, ty - ry);
    const double back = lead * dist;
    cx = tx - back * mathutil::heading_cos(approach);
    cy = ty - back * mathutil::heading_sin(approach);
}

} // namespace

void DriftBoomerang::reset() {
    i_lin_ = prev_lin_ = 0.0;
    i_ang_ = prev_ang_ = 0.0;
    prev_surge_ = 0.0;
    goal_heading_ = 0.0;
    primed_ = false;
    settled_ = false;
}

ChassisCommand DriftBoomerang::compute(const StateEstimate& state, const Pose2d& target) {
    const Pose2d pose = poseInchesToMeters(state.pose);
    const Pose2d goal = poseInchesToMeters(target);
    const double rx = pose.x;
    const double ry = pose.y;
    const double theta = pose.theta;
    const double tx = goal.x;
    const double ty = goal.y;

    // Travel direction on arrival. Reversing flips it; the front still
    // finishes on target.theta (LemLib's adjusted-heading settle).
    const double approach = config.forwards
        ? goal.theta
        : mathutil::normalize_angle(goal.theta + Robot::PI);

    const double dist = std::hypot(tx - rx, ty - ry);
    const bool close = dist < config.close_m;

    double cx, cy;
    if (close || dist < 1e-4) {
        cx = tx;
        cy = ty;
    } else {
        carrotOf(rx, ry, tx, ty, approach, config.lead, cx, cy);
    }

    const double cdx = cx - rx;
    const double cdy = cy - ry;
    const double carrot_dist = std::hypot(cdx, cdy);
    // Bearing the velocity should travel. At the point itself, hold the approach.
    const double phi = (carrot_dist < 1e-4) ? approach : mathutil::direction_heading(cdx, cdy);

    // κ = dφ/ds along the carrot bearing. Positive is a left (CCW) bend.
    double kappa = 0.0;
    if (!close && carrot_dist > Drift::CARROT_FLOOR_M && dist > Drift::CARROT_FLOOR_M) {
        const double rx2 = rx + Drift::PROBE_M * mathutil::heading_cos(phi);
        const double ry2 = ry + Drift::PROBE_M * mathutil::heading_sin(phi);
        double cx2, cy2;
        carrotOf(rx2, ry2, tx, ty, approach, config.lead, cx2, cy2);
        const double phi2 = mathutil::direction_heading(cx2 - rx2, cy2 - ry2);
        kappa = mathutil::normalize_angle(phi2 - phi) / Drift::PROBE_M;
    }

    // tan(σ) = v² κ / a_ref. Nose leads a left bend by a positive σ.
    const double v = std::fabs(state.v_lin);
    double sigma = 0.0;
    if (!close && config.drift_accel > 1e-6) {
        sigma = std::atan2(v * v * kappa, config.drift_accel);
        sigma = mathutil::clamp_val(sigma, -config.max_slip_rad, config.max_slip_rad);
    }

    double psi = mathutil::normalize_angle(phi + sigma);
    if (!config.forwards) psi = mathutil::normalize_angle(psi + Robot::PI);
    if (close) psi = goal.theta;
    goal_heading_ = psi;

    const double along_in = ((cx - rx) * mathutil::heading_cos(theta) + (cy - ry) * mathutil::heading_sin(theta))
                            / kMetersPerInch;
    const double heading_deg = mathutil::normalize_angle(psi - theta) * (180.0 / Robot::PI);

    const double small_in = Boomerang::Lateral.small_error;
    const double small_deg = Boomerang::Angular.small_error;
    const double heading_to_goal_deg =
        mathutil::normalize_angle(goal.theta - theta) * (180.0 / Robot::PI);
    settled_ = (dist / kMetersPerInch) < small_in
            && std::fabs(heading_to_goal_deg) < small_deg;

    const double lin_per_power = Robot::MAX_VEL_LIN / Controller::FULL_SCALE;
    const double yaw_per_power = Robot::MAX_VEL_ANG / Controller::FULL_SCALE;
    double surge = pidTick(along_in,
                           Boomerang::Lateral.kP, Boomerang::Lateral.kI,
                           Boomerang::Lateral.kD, Boomerang::Lateral.windup,
                           i_lin_, prev_lin_, primed_) * lin_per_power;
    double yaw = pidTick(heading_deg,
                         Boomerang::Angular.kP, Boomerang::Angular.kI,
                         Boomerang::Angular.kD, Boomerang::Angular.windup,
                         i_ang_, prev_ang_, primed_) * yaw_per_power;
    primed_ = true;

    surge = mathutil::clamp_val(surge, -Robot::MAX_VEL_LIN, Robot::MAX_VEL_LIN);
    yaw = mathutil::clamp_val(yaw, -Robot::MAX_VEL_ANG, Robot::MAX_VEL_ANG);

    const double slew = Boomerang::Lateral.slew * lin_per_power;
    surge = slewToward(surge, prev_surge_, slew);

    if (config.forwards && surge < 0.0) surge = 0.0;
    if (!config.forwards && surge > 0.0) surge = 0.0;
    prev_surge_ = surge;

    ChassisCommand cmd;
    cmd.request = DriveRequest::Motion;
    cmd.linear = surge;
    cmd.lateral = 0.0;
    cmd.angular = yaw;
    cmd.brake = false;
    return cmd;
}
