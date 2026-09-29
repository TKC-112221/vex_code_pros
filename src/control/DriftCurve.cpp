/* ---------------------------------------------------------------------------
 * DriftCurve.cpp  --  arc tangent at the target, centripetal sideslip, volts.
 * ------------------------------------------------------------------------- */
#include "control/DriftCurve.h"
#include "core/Gains.h"
#include "math/MathUtils.h"

#include <cmath>

namespace {

constexpr double kMetersPerInch = 0.0254;
constexpr double kLemlibFullPower = 127.0;
constexpr double kOnAxisM = 1e-3; // lateral offset below this is the heading line

// The curve is fitted to the far end of a straight finish of `line_length`
// meters. The finish runs into the target along `approach`. on_line means
// the robot is already on that segment. past means it has crossed the target.
struct Finish {
    double jx = 0, jy = 0;
    double length = 0;
    double along = 0;
    bool on_line = false;
    bool past = false;
};

Finish finishOf(double rx, double ry, double tx, double ty,
                double approach, double line_length) {
    Finish fin;
    fin.length = line_length > 0.0 ? line_length : 0.0;
    const double ux = std::cos(approach);
    const double uy = std::sin(approach);
    fin.jx = tx - fin.length * ux;
    fin.jy = ty - fin.length * uy;
    fin.along = (rx - tx) * ux + (ry - ty) * uy;
    fin.past = fin.along > 1e-3;
    fin.on_line = fin.length > 1e-4 && !fin.past && fin.along >= -fin.length - 1e-4;
    return fin;
}

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

// Circle through the robot, tangent to `approach` at the target.
// phi is the travel bearing, kappa is signed CCW curvature, length_m is the
// arc length still to drive in that direction.
struct Arc {
    double phi = 0;
    double kappa = 0;
    double length_m = 0;
    double cx = 0, cy = 0, r = 0, sweep = 0;
    bool curved = false;
};

Arc arcThrough(double rx, double ry, double tx, double ty, double approach) {
    const double dx = rx - tx;
    const double dy = ry - ty;
    const double ux = std::cos(approach);
    const double uy = std::sin(approach);
    const double nx = -uy; // left of the arrival heading
    const double ny = ux;
    const double along = dx * ux + dy * uy;
    const double lat = dx * nx + dy * ny;
    const double dist2 = dx * dx + dy * dy;
    const double dist = std::sqrt(dist2);

    Arc arc;
    arc.phi = approach;
    if (dist < 1e-4) return arc;

    // On the heading axis the circle is the line itself.
    if (std::fabs(lat) < kOnAxisM) {
        arc.length_m = dist;
        if (along > 0.0) arc.phi = std::atan2(ty - ry, tx - rx);
        return arc;
    }

    const double r = dist2 / (2.0 * lat);
    const double cx = tx + r * nx;
    const double cy = ty + r * ny;
    const double ox = rx - cx;
    const double oy = ry - cy;
    const double tcx = tx - cx;
    const double tcy = ty - cy;

    // CCW tangent of (robot - center) is (-oy, ox). Positive r travels CCW.
    double dir_x = -oy;
    double dir_y = ox;
    if (r < 0.0) {
        dir_x = -dir_x;
        dir_y = -dir_y;
    }
    arc.phi = std::atan2(dir_y, dir_x);
    arc.kappa = 1.0 / r;
    arc.cx = cx;
    arc.cy = cy;
    arc.r = r;
    arc.curved = true;

    // Sweep from the robot to the target in the travel direction, in [0, 2π).
    const double dot = ox * tcx + oy * tcy;
    const double cross = ox * tcy - oy * tcx;
    const double delta = std::atan2(cross, dot);
    double sweep;
    if (r > 0.0) sweep = (delta >= 0.0) ? delta : (delta + 2.0 * Robot::PI);
    else         sweep = (delta <= 0.0) ? -delta : (2.0 * Robot::PI - delta);
    arc.sweep = sweep;
    arc.length_m = std::fabs(r) * sweep;
    return arc;
}

// e^{z} − 1 − z, and its derivatives with respect to z. Series near 0.
void expBasis(double z, double& f, double& fp, double& fpp) {
    if (z > 40.0) z = 40.0;
    if (z < -40.0) z = -40.0;
    if (std::fabs(z) < 1e-2) {
        f = (z * z) * (0.5 + z * (1.0 / 6.0 + z / 24.0));
        fp = z * (1.0 + z * (0.5 + z / 6.0));
        fpp = 1.0 + z * (1.0 + z * 0.5);
        return;
    }
    const double e = std::exp(z);
    f = e - 1.0 - z;
    fp = e - 1.0;
    fpp = e;
}

// Graph over the heading axis, flat at the target, through the robot.
// rate is k in lateral ∝ e^{k ξ} − 1 − k ξ. Zero rate is the parabola ξ².
struct ExpPath {
    double phi = 0;
    double kappa = 0;
    double length_m = 0;
    bool graph = false;
    double ux = 1, uy = 0, nx = 0, ny = 1;
    double tx = 0, ty = 0;
    double xi0 = 0, eta0 = 0, rate = 0;
};

double etaOf(double xi, double xi0, double eta0, double rate) {
    if (std::fabs(xi0) < 1e-6) return 0.0;
    if (std::fabs(rate) < 1e-6) return eta0 * (xi * xi) / (xi0 * xi0);
    double f0, fp0, fpp0, f, fp, fpp;
    expBasis(rate * xi0, f0, fp0, fpp0);
    expBasis(rate * xi, f, fp, fpp);
    if (std::fabs(f0) < 1e-9) return 0.0;
    return eta0 * (f / f0);
}

ExpPath expThrough(double rx, double ry, double tx, double ty,
                   double approach, double rate) {
    const double dx = rx - tx;
    const double dy = ry - ty;
    const double ux = std::cos(approach);
    const double uy = std::sin(approach);
    const double nx = -uy;
    const double ny = ux;
    const double xi0 = dx * ux + dy * uy;
    const double eta0 = dx * nx + dy * ny;
    const double dist = std::hypot(dx, dy);

    ExpPath path;
    path.phi = approach;
    path.ux = ux;
    path.uy = uy;
    path.nx = nx;
    path.ny = ny;
    path.tx = tx;
    path.ty = ty;
    path.xi0 = xi0;
    path.eta0 = eta0;
    path.rate = rate;
    if (dist < 1e-4) return path;

    // Beside the target, or already on the heading line: no graph to fit.
    if (std::fabs(xi0) < kOnAxisM || std::fabs(eta0) < kOnAxisM) {
        path.length_m = dist;
        if (xi0 > 0.0) path.phi = std::atan2(ty - ry, tx - rx);
        return path;
    }

    // Slope and curvature of η(ξ) at the robot. Travel toward ξ = 0.
    // Parabola is the k → 0 limit: η ∝ ξ², so m = 2 η0/ξ0 and η'' = 2 η0/ξ0².
    double m = 0.0;
    double eta_pp = 0.0;
    if (std::fabs(rate) < 1e-6) {
        m = 2.0 * eta0 / xi0;
        eta_pp = 2.0 * eta0 / (xi0 * xi0);
    } else {
        double f0, fp0, fpp0;
        expBasis(rate * xi0, f0, fp0, fpp0);
        if (std::fabs(f0) < 1e-9) {
            path.length_m = dist;
            return path;
        }
        m = eta0 * (rate * fp0) / f0;
        eta_pp = eta0 * (rate * rate * fpp0) / f0;
    }
    const double speed = std::sqrt(1.0 + m * m);
    const double toward = (xi0 > 0.0) ? -1.0 : 1.0;
    const double dir_u = toward;
    const double dir_n = toward * m;
    path.phi = std::atan2(dir_u * uy + dir_n * ny, dir_u * ux + dir_n * nx);
    path.kappa = toward * eta_pp / (speed * speed * speed);
    path.graph = true;

    double px = rx, py = ry;
    double length = 0.0;
    constexpr int kSteps = 32;
    for (int i = 1; i <= kSteps; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(kSteps);
        const double xi = xi0 * (1.0 - u);
        const double eta = etaOf(xi, xi0, eta0, rate);
        const double x = tx + xi * ux + eta * nx;
        const double y = ty + xi * uy + eta * ny;
        length += std::hypot(x - px, y - py);
        px = x;
        py = y;
    }
    path.length_m = length;
    return path;
}

} // namespace

int DriftCurve::samplePath(const Pose2d& from, Pose2d* out, int cap) const {
    if (out == nullptr || cap < 2) return 0;
    const DriftCurveTarget& target = this->target;
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
        return 2;
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

    if (config.shape == DriftCurveShape::Exponential) {
        const ExpPath path = expThrough(from.x, from.y, fit_x, fit_y,
                                        approach, config.exp_rate);
        if (path.length_m < 1e-3) return 0;
        if (!path.graph) {
            out[0] = from;
            out[1] = fit;
            return append_finish(2);
        }
        int steps = curve_cap - 1;
        if (steps > 48) steps = 48;
        const int n = steps + 1;
        for (int i = 0; i < n; ++i) {
            const double u = static_cast<double>(i) / static_cast<double>(steps);
            const double xi = path.xi0 * (1.0 - u);
            const double eta = etaOf(xi, path.xi0, path.eta0, path.rate);
            out[i].x = path.tx + xi * path.ux + eta * path.nx;
            out[i].y = path.ty + xi * path.uy + eta * path.ny;
            out[i].theta = 0.0;
        }
        out[0].x = from.x;
        out[0].y = from.y;
        out[n - 1].x = fit.x;
        out[n - 1].y = fit.y;
        return append_finish(n);
    }

    const Arc arc = arcThrough(from.x, from.y, fit_x, fit_y, approach);
    if (arc.length_m < 1e-3) return 0;

    if (!arc.curved) {
        out[0] = from;
        out[1] = fit;
        return append_finish(2);
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
    return append_finish(n);
}

void DriftCurve::reset() {
    i_lin_ = prev_lin_ = 0.0;
    i_ang_ = prev_ang_ = 0.0;
    goal_heading_ = 0.0;
    primed_ = false;
    settled_ = false;
}

ChassisCommand DriftCurve::compute(const StateEstimate& state) {
    const DriftCurveTarget& target = this->target;
    const double rx = state.pose.x;
    const double ry = state.pose.y;
    const double theta = state.pose.theta;
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
    const bool close = dist < config.close_m;
    const Finish fin = finishOf(rx, ry, tx, ty, approach, target.line_length);

    double phi = approach;
    double kappa = 0.0;
    double length_m = dist;
    if (fin.on_line) {
        // Already on the straight finish. Hold the exit heading to the target.
        phi = approach;
        kappa = 0.0;
        length_m = -fin.along;
    } else if (dist > 1e-4) {
        const bool to_join = fin.length > 1e-4 && !fin.past;
        const double fit_x = to_join ? fin.jx : tx;
        const double fit_y = to_join ? fin.jy : ty;
        if (config.shape == DriftCurveShape::Exponential) {
            const ExpPath path = expThrough(rx, ry, fit_x, fit_y, approach, config.exp_rate);
            phi = path.phi;
            kappa = path.kappa;
            length_m = path.length_m;
        } else {
            const Arc arc = arcThrough(rx, ry, fit_x, fit_y, approach);
            phi = arc.phi;
            kappa = arc.kappa;
            length_m = arc.length_m;
        }
        if (to_join) length_m += fin.length;
    }
    if (length_m < 0.0) length_m = 0.0;

    // Fastest travel speed that still reaches v_end under the accel limit,
    // then stepped from the measured speed so this tick stays inside it.
    const double v_end = std::fabs(target.exit_speed);
    double v_max = config.cruise_speed;
    if (v_max < v_end) v_max = v_end;
    double accel = target.max_accel.value_or(config.path_accel);
    if (accel < 0.05) accel = 0.05;
    const double v_brake = std::sqrt(v_end * v_end + 2.0 * accel * length_m);
    const double v_ideal = (v_brake < v_max) ? v_brake : v_max;
    const double v_now = (forwards ? 1.0 : -1.0) * state.v_lin;
    const double step = accel * System::DT_SEC;
    double v_plan = v_ideal;
    if (v_plan > v_now + step) v_plan = v_now + step;
    if (v_plan < v_now - step) v_plan = v_now - step;
    const double a_travel = (v_plan - v_now) / System::DT_SEC;
    const double v_ref = (v_plan > 0.0) ? v_plan : 0.0;

    // tan(σ) = v_ref² κ / a_ref. A full stop inside close_m faces the heading.
    const bool stop_settle = close && v_end < 1e-3;
    double sigma = 0.0;
    if (!stop_settle && config.drift_accel > 1e-6) {
        sigma = std::atan2(v_ref * v_ref * kappa, config.drift_accel);
        sigma = mathutil::clamp_val(sigma, -config.max_slip_rad, config.max_slip_rad);
    }

    double psi = mathutil::normalize_angle(phi + sigma);
    if (!forwards) psi = mathutil::normalize_angle(psi + Robot::PI);
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
    const double direction = forwards ? 1.0 : -1.0;
    const bool heading_ok = target.ignore_direction
        || std::fabs(heading_to_goal_deg) < small_deg;
    settled_ = (dist / kMetersPerInch) < small_in
            && heading_ok
            && std::fabs(v_now - v_end) < 0.2;

    const double v_cmd = direction * v_plan;
    const double a_cmd = direction * a_travel;
    double omega = v_ref * kappa;
    double radius = 10.0;
    if (std::fabs(kappa) > 1e-3) radius = 1.0 / std::fabs(kappa);
    if (radius > 10.0) radius = 10.0;

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
