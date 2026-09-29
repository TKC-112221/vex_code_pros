/* ---------------------------------------------------------------------------
 * DriftBoomerang.cpp  --  boomerang carrot, centripetal sideslip, volt command.
 * ------------------------------------------------------------------------- */
#include "control/DriftBoomerang.h"
#include "core/Gains.h"
#include "math/MathUtils.h"

#include <cmath>

namespace {

constexpr double kMetersPerInch = 0.0254;
constexpr double kLemlibFullPower = 127.0;
constexpr double kProbeM = 0.01;          // curvature finite-difference step
constexpr double kCarrotFloorM = 0.05;    // below this, bearing curvature is noise

double voltsPerPower() {
    return System::MAX_VOLTS / kLemlibFullPower;
}

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
// In this codebase heading 0 is +X and CCW is positive, which is already
// LemLib's "standard form", so there is no VEX-heading conversion here.
void carrotOf(double rx, double ry, double tx, double ty, double approach,
              double lead, double& cx, double& cy) {
    const double dist = std::hypot(tx - rx, ty - ry);
    const double back = lead * dist;
    cx = tx - back * std::cos(approach);
    cy = ty - back * std::sin(approach);
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
    const double rx = state.pose.x;
    const double ry = state.pose.y;
    const double theta = state.pose.theta;
    const double tx = target.x;
    const double ty = target.y;

    // Travel direction on arrival. Reversing flips it; the front still
    // finishes on target.theta (LemLib's adjusted-heading settle).
    const double approach = config.forwards
        ? target.theta
        : mathutil::normalize_angle(target.theta + Robot::PI);

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
    const double phi = (carrot_dist < 1e-4) ? approach : std::atan2(cdy, cdx);

    // κ = dφ/ds along the carrot bearing. Positive is a left (CCW) bend.
    double kappa = 0.0;
    if (!close && carrot_dist > kCarrotFloorM && dist > kCarrotFloorM) {
        const double rx2 = rx + kProbeM * std::cos(phi);
        const double ry2 = ry + kProbeM * std::sin(phi);
        double cx2, cy2;
        carrotOf(rx2, ry2, tx, ty, approach, config.lead, cx2, cy2);
        const double phi2 = std::atan2(cy2 - ry2, cx2 - rx2);
        kappa = mathutil::normalize_angle(phi2 - phi) / kProbeM;
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
    if (close) psi = target.theta;
    goal_heading_ = psi;

    const double along_in = ((cx - rx) * std::cos(theta) + (cy - ry) * std::sin(theta))
                            / kMetersPerInch;
    const double heading_deg = mathutil::normalize_angle(psi - theta) * (180.0 / Robot::PI);

    const double small_in = Boomerang::Lateral.small_error;
    const double small_deg = Boomerang::Angular.small_error;
    const double heading_to_goal_deg =
        mathutil::normalize_angle(target.theta - theta) * (180.0 / Robot::PI);
    settled_ = (dist / kMetersPerInch) < small_in
            && std::fabs(heading_to_goal_deg) < small_deg;

    const double scale = voltsPerPower();
    double surge = pidTick(along_in,
                           Boomerang::Lateral.kP, Boomerang::Lateral.kI,
                           Boomerang::Lateral.kD, Boomerang::Lateral.windup,
                           i_lin_, prev_lin_, primed_) * scale;
    double yaw = pidTick(heading_deg,
                         Boomerang::Angular.kP, Boomerang::Angular.kI,
                         Boomerang::Angular.kD, Boomerang::Angular.windup,
                         i_ang_, prev_ang_, primed_) * scale;
    primed_ = true;

    const double max_v = config.max_volts;
    surge = mathutil::clamp_val(surge, -max_v, max_v);
    yaw   = mathutil::clamp_val(yaw, -max_v, max_v);

    const double slew = Boomerang::Lateral.slew * scale;
    surge = slewToward(surge, prev_surge_, slew);

    // Angular effort wins the voltage budget, same priority as LemLib's overturn.
    const double overturn = std::fabs(yaw) + std::fabs(surge) - max_v;
    if (overturn > 0.0) surge -= std::copysign(overturn, surge);

    if (config.forwards && surge < 0.0) surge = 0.0;
    if (!config.forwards && surge > 0.0) surge = 0.0;
    prev_surge_ = surge;

    ChassisCommand cmd;
    cmd.linear  = surge;
    cmd.lateral = 0.0;
    cmd.angular = yaw;
    cmd.brake   = false;
    return cmd;
}
