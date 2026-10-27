/* ---------------------------------------------------------------------------
 * DriftCurve.cpp  --  curve tangent at the target, centripetal sideslip, volts.
 * ------------------------------------------------------------------------- */
#include "control/DriftCurve.h"
#include "control/DriftPathFit.h"
#include "core/Tuning.h"
#include "math/MathUtils.h"

#include <cmath>

using namespace driftpath;

namespace {

double voltsPerPower() {
    return System::MAX_VOLTS / Controller::FULL_SCALE;
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

} // namespace

int DriftCurve::samplePath(const Pose2d& from_in, Pose2d* out, int cap,
                           double v_lin, double v_ang) const {
    if (out == nullptr || cap < 2) return 0;
    const DriftCurveTarget target = targetInMeters(this->target);
    const Pose2d from = poseInchesToMeters(from_in);
    auto publish = [&](int n) {
        for (int i = 0; i < n; ++i) {
            out[i].x = metersToInches(out[i].x);
            out[i].y = metersToInches(out[i].y);
        }
        return n;
    };
    const double dist = std::hypot(target.x - from.x, target.y - from.y);
    double approach = target.exit_direction;
    if (target.ignore_direction && dist > 1e-4) {
        approach = std::atan2(target.y - from.y, target.x - from.x);
    }
    const Pose2d end{target.x, target.y, approach};
    const Finish fin = finishOf(from.x, from.y, target.x, target.y,
                                approach, target.line_length);
    if (fin.on_line) {
        out[0] = from;
        out[1] = end;
        return publish(2);
    }

    const bool to_join = fin.length > 1e-4 && !fin.past;
    const double fit_x = to_join ? fin.jx : target.x;
    const double fit_y = to_join ? fin.jy : target.y;
    const Pose2d fit{fit_x, fit_y, approach};
    int curve_cap = cap;
    if (to_join && cap > 2) curve_cap = cap - 1;

    auto append_finish = [&](int n) {
        if (!to_join || n < 1 || n >= cap) return n;
        out[n] = end;
        return n + 1;
    };

    if (config.shape == DriftCurveShape::Quintic
        || config.shape == DriftCurveShape::Hermite) {
        const bool forwards = config.forwards && !target.backwards;
        const Momentum momentum = momentumOf(from.theta, forwards, v_lin, v_ang);
        const Quintic path = (config.shape == DriftCurveShape::Hermite)
            ? hermiteThrough(from.x, from.y, fit_x, fit_y, approach,
                             momentum.x, momentum.y, momentum.kappa)
            : quinticThrough(from.x, from.y, fit_x, fit_y, approach);
        if (path.length_m < 1e-3) return 0;
        if (!path.curved) {
            out[0] = from;
            out[1] = fit;
            return publish(append_finish(2));
        }
        int steps = curve_cap - 1;
        if (steps > 48) steps = 48;
        const int n = steps + 1;
        for (int i = 0; i < n; ++i) {
            const double u = static_cast<double>(i) / static_cast<double>(steps);
            quinticAt(path, u, out[i].x, out[i].y);
            out[i].theta = 0.0;
        }
        out[0].x = from.x;
        out[0].y = from.y;
        out[n - 1].x = fit.x;
        out[n - 1].y = fit.y;
        return publish(append_finish(n));
    }

    if (config.shape == DriftCurveShape::Exponential
        || config.shape == DriftCurveShape::Quartic
        || config.shape == DriftCurveShape::QuarticHermite) {
        const bool forwards = config.forwards && !target.backwards;
        const Momentum momentum = momentumOf(from.theta, forwards, v_lin, v_ang);
        const bool quartic = config.shape == DriftCurveShape::Quartic;
        const GraphPath path = (config.shape == DriftCurveShape::QuarticHermite)
            ? quarticHermiteThrough(from.x, from.y, fit_x, fit_y, approach,
                                    momentum.x, momentum.y, momentum.kappa)
            : graphThrough(from.x, from.y, fit_x, fit_y,
                           approach, quartic, config.exp_rate);
        if (path.length_m < 1e-3) return 0;
        if (!path.graph) {
            out[0] = from;
            out[1] = fit;
            return publish(append_finish(2));
        }
        int steps = curve_cap - 1;
        if (steps > 48) steps = 48;
        const int n = steps + 1;
        for (int i = 0; i < n; ++i) {
            const double u = static_cast<double>(i) / static_cast<double>(steps);
            const double xi = path.xi0 * (1.0 - u);
            const double eta = graphEta(path, xi);
            out[i].x = path.tx + xi * path.ux + eta * path.nx;
            out[i].y = path.ty + xi * path.uy + eta * path.ny;
            out[i].theta = 0.0;
        }
        out[0].x = from.x;
        out[0].y = from.y;
        out[n - 1].x = fit.x;
        out[n - 1].y = fit.y;
        return publish(append_finish(n));
    }

    const Arc arc = arcThrough(from.x, from.y, fit_x, fit_y, approach);
    if (arc.length_m < 1e-3) return 0;

    if (!arc.curved) {
        out[0] = from;
        out[1] = fit;
        return publish(append_finish(2));
    }

    int steps = curve_cap - 1;
    if (steps > 48) steps = 48;
    const double a0 = std::atan2(from.y - arc.cy, from.x - arc.cx);
    const double dir = (arc.r > 0.0) ? 1.0 : -1.0;
    const double radius = std::fabs(arc.r);
    const int n = steps + 1;
    for (int i = 0; i < n; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(steps);
        const double a = a0 + dir * arc.sweep * u;
        out[i].x = arc.cx + radius * std::cos(a);
        out[i].y = arc.cy + radius * std::sin(a);
        out[i].theta = 0.0;
    }
    out[0].x = from.x;
    out[0].y = from.y;
    out[n - 1].x = fit.x;
    out[n - 1].y = fit.y;
    return publish(append_finish(n));
}

void DriftCurve::reset() {
    i_lin_ = prev_lin_ = 0.0;
    i_ang_ = prev_ang_ = 0.0;
    goal_heading_ = 0.0;
    primed_ = false;
    settled_ = false;
}

ChassisCommand DriftCurve::compute(const StateEstimate& state) {
    const DriftCurveTarget target = targetInMeters(this->target);
    const Pose2d pose = poseInchesToMeters(state.pose);
    const double rx = pose.x;
    const double ry = pose.y;
    const double theta = pose.theta;
    const double tx = target.x;
    const double ty = target.y;

    // Exit velocity direction is the path tangent on arrival. Reversing
    // leaves that tangent alone and turns the nose to face the other way.
    // ignore_direction uses the bearing to the target instead, so only the
    // exit speed is imposed.
    const double dist = std::hypot(tx - rx, ty - ry);
    double approach = target.exit_direction;
    if (target.ignore_direction && dist > 1e-4) {
        approach = std::atan2(ty - ry, tx - rx);
    }
    const bool forwards = config.forwards && !target.backwards;
    const double nose = forwards
        ? approach
        : mathutil::normalize_angle(approach + Robot::PI);
    const double path_drive = forwards ? 1.0 : -1.0;
    const bool close = dist < config.close_m;
    const Finish fin = finishOf(rx, ry, tx, ty, approach, target.line_length);

    double phi = approach;
    double kappa = 0.0;
    double length_m = dist;
    SpeedKnot knots[kSpeedKnots];
    int kn = 0;
    if (fin.on_line) {
        // Already on the straight finish. Hold the exit heading to the target.
        phi = approach;
        kappa = 0.0;
        length_m = -fin.along;
    } else if (dist > 1e-4) {
        const bool to_join = fin.length > 1e-4 && !fin.past;
        const double fit_x = to_join ? fin.jx : tx;
        const double fit_y = to_join ? fin.jy : ty;
        if (config.shape == DriftCurveShape::Quintic
            || config.shape == DriftCurveShape::Hermite) {
            Quintic path;
            if (config.shape == DriftCurveShape::Hermite) {
                const Momentum momentum = momentumOf(theta, forwards, state.v_lin, state.v_ang);
                path = hermiteThrough(rx, ry, fit_x, fit_y, approach,
                                      momentum.x, momentum.y, momentum.kappa);
            } else {
                path = quinticThrough(rx, ry, fit_x, fit_y, approach);
            }
            phi = path.phi;
            kappa = path.kappa;
            length_m = path.length_m;
            if (config.shape == DriftCurveShape::Hermite && path.curved) {
                double look = std::fabs(state.v_lin) * kHermiteLookSec;
                if (look < kHermiteLookMin) look = kHermiteLookMin;
                const double look_cap = kHermiteLookFrac * path.length_m;
                if (look > look_cap) look = look_cap;
                quinticLook(path, look, phi, kappa);
            }
            kn = quinticKnots(path, knots, kSpeedKnots);
        } else if (config.shape == DriftCurveShape::Exponential
            || config.shape == DriftCurveShape::Quartic
            || config.shape == DriftCurveShape::QuarticHermite) {
            GraphPath path;
            if (config.shape == DriftCurveShape::QuarticHermite) {
                const Momentum momentum = momentumOf(theta, forwards, state.v_lin, state.v_ang);
                path = quarticHermiteThrough(rx, ry, fit_x, fit_y, approach,
                                             momentum.x, momentum.y, momentum.kappa);
            } else {
                const bool quartic = config.shape == DriftCurveShape::Quartic;
                path = graphThrough(rx, ry, fit_x, fit_y, approach,
                                    quartic, config.exp_rate);
            }
            phi = path.phi;
            kappa = path.kappa;
            length_m = path.length_m;
            if (config.shape == DriftCurveShape::QuarticHermite && path.graph) {
                double look = std::fabs(state.v_lin) * kHermiteLookSec;
                if (look < kHermiteLookMin) look = kHermiteLookMin;
                const double look_cap = kHermiteLookFrac * path.length_m;
                if (look > look_cap) look = look_cap;
                graphLook(path, look, phi, kappa);
            }
            kn = graphKnots(path, knots, kSpeedKnots);
        } else {
            const Arc arc = arcThrough(rx, ry, fit_x, fit_y, approach);
            phi = arc.phi;
            kappa = arc.kappa;
            length_m = arc.length_m;
            kn = arcKnots(arc, knots, kSpeedKnots);
        }
        if (to_join) {
            length_m += fin.length;
            kn = appendSpan(knots, kn, kSpeedKnots, fin.length);
        }
    }
    if (length_m < 0.0) length_m = 0.0;

    // Ceiling from the timed profile, then stepped from the measured speed
    // so this tick stays inside the longitudinal accel.
    const double v_end = std::fabs(target.exit_speed);
    double accel = target.max_accel.value_or(config.path_accel);
    if (accel < Motion::MIN_ACCEL_MPS2) accel = Motion::MIN_ACCEL_MPS2;
    const SpeedScale pace = scaleSpeed(config.speed_reward, accel, config.centripetal_accel);
    accel = pace.accel;
    if (kn < 2) {
        knots[0].s = 0.0;
        knots[0].kappa = kappa;
        knots[0].dk = 0.0;
        knots[1].s = length_m;
        knots[1].kappa = 0.0;
        knots[1].dk = 0.0;
        kn = 2;
    }
    fillSpeedSlope(knots, kn);
    double v_ideal = feasibleSpeedAt(knots, kn, 0.0, v_end, config.cruise_speed, pace.accel,
                                      pace.centripetal, Robot::MAX_VEL_ANG, pace.yaw_accel);
    if (!std::isfinite(v_ideal) || v_ideal < 0.0) v_ideal = 0.0;
    const double v_now = path_drive * state.v_lin;
    // Slew the nose speed. A travel-frame slew multiplied by the drive sign
    // copies a backward measurement into the command.
    const double step = accel * System::DT_SEC;
    const double v_des = path_drive * v_ideal;
    double v_cmd_slew = v_des;
    if (v_cmd_slew > state.v_lin + step) v_cmd_slew = state.v_lin + step;
    if (v_cmd_slew < state.v_lin - step) v_cmd_slew = state.v_lin - step;
    const double a_cmd_slew = (v_cmd_slew - state.v_lin) / System::DT_SEC;
    const double v_travel = path_drive * v_cmd_slew;
    const double v_ref = (v_travel > 0.0) ? v_travel : 0.0;

    // tan(σ) = v_ref² κ / a_ref. A full stop inside close_m faces the heading.
    const bool stop_settle = close && v_end < 1e-3;
    double sigma = 0.0;
    if (!stop_settle && config.drift_accel > 1e-6) {
        sigma = std::atan2(v_ref * v_ref * kappa, config.drift_accel);
        sigma = mathutil::clamp_val(sigma, -config.max_slip_rad, config.max_slip_rad);
    }

    double psi = mathutil::normalize_angle(phi + sigma);
    if (path_drive < 0.0) psi = mathutil::normalize_angle(psi + Robot::PI);
    // A prescribed finish heading. With ignore_direction the nose stays
    // where it is, and only the speed is driven to exit_speed.
    if (stop_settle && !target.ignore_direction) psi = nose;
    if (stop_settle && target.ignore_direction) psi = theta;
    goal_heading_ = psi;

    const double heading_deg = mathutil::normalize_angle(psi - theta) * (180.0 / Robot::PI);

    const double small_in = Boomerang::Lateral.small_error;
    const double small_deg = Boomerang::Angular.small_error;
    const double heading_to_goal_deg =
        mathutil::normalize_angle(nose - theta) * (180.0 / Robot::PI);
    const bool heading_ok = target.ignore_direction
        || std::fabs(heading_to_goal_deg) < small_deg;
    settled_ = (dist / kMetersPerInch) < small_in
            && heading_ok
            && std::fabs(v_now - v_end) < Motion::SETTLE_SPEED_MPS;

    const double v_cmd = v_cmd_slew;
    const double a_cmd = a_cmd_slew;
    double omega = v_ref * kappa;
    double radius = Physics::TURN_RADIUS_MAX_M;
    if (std::fabs(kappa) > Motion::KAPPA_EPS) radius = 1.0 / std::fabs(kappa);
    if (radius > Physics::TURN_RADIUS_MAX_M) radius = Physics::TURN_RADIUS_MAX_M;

    double surge = 0.0;
    if (model_ != nullptr) {
        surge = model_->linearVolts(v_cmd, a_cmd, std::fabs(v_cmd), std::fabs(a_cmd),
                                    omega, radius);
    }

    const double scale = voltsPerPower();
    double yaw = pidTick(heading_deg,
                         Boomerang::Angular.kP, Boomerang::Angular.kI,
                         Boomerang::Angular.kD, Boomerang::Angular.windup,
                         i_ang_, prev_ang_, primed_) * scale;
    primed_ = true;

    const double max_v = config.max_volts;
    surge = mathutil::clamp_val(surge, -max_v, max_v);
    yaw   = mathutil::clamp_val(yaw, -max_v, max_v);

    // Angular effort wins the voltage budget, same priority as LemLib's overturn.
    const double overturn = std::fabs(yaw) + std::fabs(surge) - max_v;
    if (overturn > 0.0) surge -= std::copysign(overturn, surge);

    ChassisCommand cmd;
    cmd.linear  = surge;
    cmd.lateral = 0.0;
    cmd.angular = yaw;
    cmd.brake   = false;
    return cmd;
}
