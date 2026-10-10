/* ---------------------------------------------------------------------------
 * DriftFollower.cpp  --  plan a DriftCurve once, then track it.
 * ------------------------------------------------------------------------- */
#include "control/DriftFollower.h"
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

void pushPoint(DriftFollower::Sample* out, int& n, int cap,
               double x, double y, double phi, double kappa) {
    if (out == nullptr || n >= cap) return;
    if (n > 0) {
        const double d = mathutil::normalize_angle(phi - out[n - 1].phi);
        phi = out[n - 1].phi + d;
    }
    DriftFollower::Sample s;
    s.x = x;
    s.y = y;
    s.phi = phi;
    s.kappa = kappa;
    if (n == 0) s.s = 0.0;
    else s.s = out[n - 1].s + std::hypot(x - out[n - 1].x, y - out[n - 1].y);
    out[n++] = s;
}

void slopeOf(DriftFollower::Sample* samp, int n) {
    if (samp == nullptr || n < 2) return;
    for (int i = 0; i < n; ++i) {
        const int i0 = (i == 0) ? 0 : (i - 1);
        const int i1 = (i + 1 >= n) ? (n - 1) : (i + 1);
        const double ds = samp[i1].s - samp[i0].s;
        samp[i].dk_ds = (ds > 1e-4) ? (samp[i1].kappa - samp[i0].kappa) / ds : 0.0;
    }
    for (int i = 0; i < n; ++i) {
        const int i0 = (i == 0) ? 0 : (i - 1);
        const int i1 = (i + 1 >= n) ? (n - 1) : (i + 1);
        const double ds = samp[i1].s - samp[i0].s;
        samp[i].d2k_ds2 = (ds > 1e-4) ? (samp[i1].dk_ds - samp[i0].dk_ds) / ds : 0.0;
    }
}

void snapEnd(DriftFollower::Sample* out, int n, double x, double y) {
    if (n < 1) return;
    out[n - 1].x = x;
    out[n - 1].y = y;
    if (n >= 2) {
        out[n - 1].s = out[n - 2].s + std::hypot(x - out[n - 2].x, y - out[n - 2].y);
    }
}

void appendFinish(DriftFollower::Sample* out, int& n, int cap,
                  double tx, double ty, double phi) {
    if (n <= 0) return;
    if (n < cap) {
        DriftFollower::Sample join = out[n - 1];
        join.kappa = 0.0;
        join.dk_ds = 0.0;
        join.d2k_ds2 = 0.0;
        out[n++] = join;
    }
    pushPoint(out, n, cap, tx, ty, phi, 0.0);
}

// Exponential and parabola. graphFrame only differentiates the polynomial
// family (quartic and quartic Hermite).
void graphPose(const GraphPath& path, double xi, double& phi, double& kappa) {
    if (path.momentum || path.quartic) {
        graphFrame(path, xi, phi, kappa);
        return;
    }
    double m = 0.0;
    double eta_pp = 0.0;
    if (std::fabs(path.rate) < 1e-6) {
        if (std::fabs(path.xi0) > 1e-9) {
            m = 2.0 * path.eta0 * xi / (path.xi0 * path.xi0);
            eta_pp = 2.0 * path.eta0 / (path.xi0 * path.xi0);
        }
    } else {
        double f0 = 0.0, fp0 = 0.0, fpp0 = 0.0, f = 0.0, fp = 0.0, fpp = 0.0;
        expBasis(path.rate * path.xi0, f0, fp0, fpp0);
        expBasis(path.rate * xi, f, fp, fpp);
        if (std::fabs(f0) > 1e-9) {
            m = path.eta0 * (path.rate * fp) / f0;
            eta_pp = path.eta0 * (path.rate * path.rate * fpp) / f0;
        }
    }
    const double speed = std::sqrt(1.0 + m * m);
    const double toward = (path.xi0 > 0.0) ? -1.0 : 1.0;
    phi = std::atan2(toward * path.uy + toward * m * path.ny,
                     toward * path.ux + toward * m * path.nx);
    const double sp3 = speed * speed * speed;
    kappa = (sp3 > 1e-9) ? toward * eta_pp / sp3 : 0.0;
}

void sampleStraight(DriftFollower::Sample* out, int& n, int cap,
                    double x0, double y0, double x1, double y1, double phi) {
    pushPoint(out, n, cap, x0, y0, phi, 0.0);
    pushPoint(out, n, cap, x1, y1, phi, 0.0);
}

void sampleGraph(DriftFollower::Sample* out, int& n, int cap, const GraphPath& path,
                 double x0, double y0, double x1, double y1) {
    if (!path.graph) {
        sampleStraight(out, n, cap, x0, y0, x1, y1, path.phi);
        return;
    }
    for (int i = 0; i <= Drift::CURVE_STEPS; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(Drift::CURVE_STEPS);
        const double xi = path.xi0 * (1.0 - u);
        const double eta = graphEta(path, xi);
        double phi = 0.0, kappa = 0.0;
        graphPose(path, xi, phi, kappa);
        pushPoint(out, n, cap,
                  path.tx + xi * path.ux + eta * path.nx,
                  path.ty + xi * path.uy + eta * path.ny,
                  phi, kappa);
    }
    snapEnd(out, n, x1, y1);
    if (n > 0) {
        out[0].x = x0;
        out[0].y = y0;
    }
}

void sampleQuintic(DriftFollower::Sample* out, int& n, int cap, const Quintic& path,
                   double x0, double y0, double x1, double y1) {
    if (!path.curved) {
        sampleStraight(out, n, cap, x0, y0, x1, y1, path.phi);
        return;
    }
    for (int i = 0; i <= Drift::CURVE_STEPS; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(Drift::CURVE_STEPS);
        double x = 0.0, y = 0.0, phi = 0.0, kappa = 0.0;
        quinticAt(path, t, x, y);
        quinticFrame(path, t, phi, kappa);
        pushPoint(out, n, cap, x, y, phi, kappa);
    }
    snapEnd(out, n, x1, y1);
    if (n > 0) {
        out[0].x = x0;
        out[0].y = y0;
    }
}

void sampleArc(DriftFollower::Sample* out, int& n, int cap, const Arc& arc,
               double x0, double y0, double x1, double y1) {
    if (!arc.curved) {
        sampleStraight(out, n, cap, x0, y0, x1, y1, arc.phi);
        return;
    }
    const double a0 = std::atan2(y0 - arc.cy, x0 - arc.cx);
    const double dir = (arc.r > 0.0) ? 1.0 : -1.0;
    const double radius = std::fabs(arc.r);
    for (int i = 0; i <= Drift::CURVE_STEPS; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(Drift::CURVE_STEPS);
        const double a = a0 + dir * arc.sweep * u;
        const double x = arc.cx + radius * std::cos(a);
        const double y = arc.cy + radius * std::sin(a);
        double tx = -(y - arc.cy);
        double ty = x - arc.cx;
        if (arc.r < 0.0) {
            tx = -tx;
            ty = -ty;
        }
        const double phi = std::atan2(ty, tx);
        const double kappa = (std::fabs(arc.r) > 1e-9) ? 1.0 / arc.r : 0.0;
        pushPoint(out, n, cap, x, y, phi, kappa);
    }
    snapEnd(out, n, x1, y1);
    if (n > 0) {
        out[0].x = x0;
        out[0].y = y0;
    }
}

} // namespace

void DriftFollower::reset() {
    count_ = 0;
    planned_ = false;
    i_ang_ = prev_ang_ = 0.0;
    i_lat_ = prev_lat_ = prev_dlat_ = 0.0;
    head_primed_ = false;
    lat_primed_ = false;
    goal_heading_ = 0.0;
    settled_ = false;
    a_actual_lat_ = 0.0;
}

int DriftFollower::buildPlan(const DriftFollowerConfig& config, const DriftCurveTarget& target,
                             const Pose2d& from, double v_lin, double v_ang, Sample* out) {
    if (out == nullptr) return 0;
    const double tx = target.x;
    const double ty = target.y;
    const double dist = std::hypot(tx - from.x, ty - from.y);
    double approach = target.exit_direction;
    if (target.ignore_direction && dist > 1e-4) {
        approach = std::atan2(ty - from.y, tx - from.x);
    }
    const bool forwards = config.forwards && !target.backwards;
    const Finish fin = finishOf(from.x, from.y, tx, ty, approach, target.line_length);

    int n = 0;
    if (fin.past || dist < 1e-4) {
        const double ux = std::cos(approach);
        const double uy = std::sin(approach);
        pushPoint(out, n, kCap, tx - 0.01 * ux, ty - 0.01 * uy, approach, 0.0);
        pushPoint(out, n, kCap, tx, ty, approach, 0.0);
        return n;
    }
    if (fin.on_line) {
        sampleStraight(out, n, kCap, fin.jx, fin.jy, tx, ty, approach);
        return n;
    }

    const bool to_join = fin.length > 1e-4 && !fin.past;
    const double fit_x = to_join ? fin.jx : tx;
    const double fit_y = to_join ? fin.jy : ty;

    switch (config.shape) {
    case DriftCurveShape::Quintic:
    case DriftCurveShape::Hermite: {
        const Momentum momentum = momentumOf(from.theta, forwards, v_lin, v_ang);
        const Quintic path = (config.shape == DriftCurveShape::Hermite)
            ? hermiteThrough(from.x, from.y, fit_x, fit_y, approach,
                             momentum.x, momentum.y, momentum.kappa)
            : quinticThrough(from.x, from.y, fit_x, fit_y, approach);
        if (path.length_m < 1e-3) {
            pushPoint(out, n, kCap, from.x, from.y, approach, 0.0);
            pushPoint(out, n, kCap, fit_x, fit_y, approach, 0.0);
        } else {
            sampleQuintic(out, n, kCap, path, from.x, from.y, fit_x, fit_y);
        }
        break;
    }
    case DriftCurveShape::Exponential:
    case DriftCurveShape::Quartic:
    case DriftCurveShape::QuarticHermite: {
        const Momentum momentum = momentumOf(from.theta, forwards, v_lin, v_ang);
        GraphPath path;
        if (config.shape == DriftCurveShape::QuarticHermite) {
            path = quarticHermiteThrough(from.x, from.y, fit_x, fit_y, approach,
                                        momentum.x, momentum.y, momentum.kappa);
        } else {
            const bool quartic = config.shape == DriftCurveShape::Quartic;
            path = graphThrough(from.x, from.y, fit_x, fit_y, approach,
                                quartic, config.exp_rate);
        }
        if (path.length_m < 1e-3) {
            pushPoint(out, n, kCap, from.x, from.y, approach, 0.0);
            pushPoint(out, n, kCap, fit_x, fit_y, approach, 0.0);
        } else {
            sampleGraph(out, n, kCap, path, from.x, from.y, fit_x, fit_y);
        }
        break;
    }
    case DriftCurveShape::Arc: {
        const Arc arc = arcThrough(from.x, from.y, fit_x, fit_y, approach);
        if (arc.length_m < 1e-3) {
            pushPoint(out, n, kCap, from.x, from.y, approach, 0.0);
            pushPoint(out, n, kCap, fit_x, fit_y, approach, 0.0);
        } else {
            sampleArc(out, n, kCap, arc, from.x, from.y, fit_x, fit_y);
        }
        break;
    }
    }

    const int curve_n = n;
    slopeOf(out, curve_n);
    if (to_join) appendFinish(out, n, kCap, tx, ty, approach);
    return n;
}

DriftFollower::Proj DriftFollower::projectAt(const Sample* samp, int n, double rx, double ry) {
    Proj best;
    if (samp == nullptr || n < 2) return best;
    double best_d2 = 1e300;
    bool found = false;
    for (int i = 0; i < n - 1; ++i) {
        const Sample& a = samp[i];
        const Sample& b = samp[i + 1];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double len2 = dx * dx + dy * dy;
        const double seg = std::sqrt(len2);
        double t = 0.0;
        if (len2 > 1e-12) {
            t = ((rx - a.x) * dx + (ry - a.y) * dy) / len2;
        }
        const bool first = i == 0;
        const bool last = i == n - 2;
        if (!first && t < 0.0) t = 0.0;
        if (!last && t > 1.0) t = 1.0;

        double px = 0.0, py = 0.0, phi = 0.0, kappa = 0.0, dk = 0.0, d2 = 0.0, s = 0.0;
        if (t >= 0.0 && t <= 1.0) {
            px = a.x + t * dx;
            py = a.y + t * dy;
            s = a.s + t * (b.s - a.s);
            phi = a.phi + t * (b.phi - a.phi);
            kappa = a.kappa + t * (b.kappa - a.kappa);
            dk = a.dk_ds + t * (b.dk_ds - a.dk_ds);
            d2 = a.d2k_ds2 + t * (b.d2k_ds2 - a.d2k_ds2);
        } else if (t > 1.0) {
            const double over = (t - 1.0) * seg;
            phi = b.phi;
            px = b.x + std::cos(phi) * over;
            py = b.y + std::sin(phi) * over;
            s = b.s + over;
            kappa = b.kappa;
            dk = b.dk_ds;
            d2 = b.d2k_ds2;
        } else {
            const double before = t * seg;
            phi = a.phi;
            px = a.x + std::cos(phi) * before;
            py = a.y + std::sin(phi) * before;
            s = a.s + before;
            kappa = a.kappa;
            dk = a.dk_ds;
            d2 = a.d2k_ds2;
        }

        const double ex = rx - px;
        const double ey = ry - py;
        const double d2err = ex * ex + ey * ey;
        const bool closer = d2err < best_d2 - 1e-12;
        const bool tie_further = !closer && d2err <= best_d2 + 1e-12 && s > best.remaining;
        // `remaining` is reused as the candidate arc length until the winner is known.
        if (!found || closer || tie_further) {
            found = true;
            best_d2 = d2err;
            best.phi = phi;
            best.kappa = kappa;
            best.dk_ds = dk;
            best.d2k_ds2 = d2;
            best.remaining = s;
            const double nx = -std::sin(phi);
            const double ny = std::cos(phi);
            best.lat = ex * nx + ey * ny;
        }
    }
    const double length = samp[n - 1].s;
    const double s = best.remaining;
    best.remaining = length - s;
    if (best.remaining < 0.0) best.remaining = 0.0;
    return best;
}

ChassisCommand DriftFollower::compute(const StateEstimate& state) {
    const DriftCurveTarget target = targetInMeters(this->target);
    const Pose2d pose = poseInchesToMeters(state.pose);
    if (!planned_) {
        count_ = buildPlan(config, target, pose, state.v_lin, state.v_ang, samples_);
        planned_ = true;
        head_primed_ = false;
        lat_primed_ = false;
        i_ang_ = prev_ang_ = 0.0;
        i_lat_ = prev_lat_ = prev_dlat_ = 0.0;
    }

    const double rx = pose.x;
    const double ry = pose.y;
    const double theta = pose.theta;
    const double tx = target.x;
    const double ty = target.y;
    const double dist = std::hypot(tx - rx, ty - ry);
    const bool forwards = config.forwards && !target.backwards;
    const double nose = forwards
        ? target.exit_direction
        : mathutil::normalize_angle(target.exit_direction + Robot::PI);

    double phi = target.exit_direction;
    double kappa = 0.0;
    double dk = 0.0;
    double d2k = 0.0;
    double lat = 0.0;
    double length_m = dist;
    if (count_ >= 2) {
        const Proj proj = projectAt(samples_, count_, rx, ry);
        phi = proj.phi;
        kappa = proj.kappa;
        dk = proj.dk_ds;
        d2k = proj.d2k_ds2;
        lat = proj.lat;
        length_m = proj.remaining;
    }
    if (length_m < 0.0) length_m = 0.0;

    const double v_end = std::fabs(target.exit_speed);
    double v_max = config.cruise_speed;
    if (v_max < v_end) v_max = v_end;
    double accel = target.max_accel.value_or(config.path_accel);
    if (accel < config.min_accel) accel = config.min_accel;
    const double v_brake = std::sqrt(v_end * v_end + 2.0 * accel * length_m);
    const double v_ideal = (v_brake < v_max) ? v_brake : v_max;
    const double v_now = (forwards ? 1.0 : -1.0) * state.v_lin;
    const double step = accel * System::DT_SEC;
    double v_plan = v_ideal;
    if (v_plan > v_now + step) v_plan = v_now + step;
    if (v_plan < v_now - step) v_plan = v_now - step;
    const double a_travel = (v_plan - v_now) / System::DT_SEC;
    const double v_ref = (v_plan > 0.0) ? v_plan : 0.0;

    const bool close = dist < config.close_m;
    const bool stop_settle = close && v_end < 1e-3;

    const double vx = state.v_lin * std::cos(theta);
    const double vy = state.v_lin * std::sin(theta);
    const double v_lat = vx * (-std::sin(phi)) + vy * std::cos(phi);

    double sigma = 0.0;
    double sigma_dot = 0.0;
    double sigma_ddot = 0.0;
    a_actual_lat_ = 0.0;
    if (!stop_settle) {
        const double a_lat = v_ref * v_ref * kappa - config.k_slip * v_lat;
        const double da = 2.0 * v_ref * a_travel * kappa
                        + v_ref * v_ref * v_ref * dk;
        const double d2a = 2.0 * a_travel * a_travel * kappa
                         + 5.0 * v_ref * v_ref * a_travel * dk
                         + v_ref * v_ref * v_ref * v_ref * d2k;
        double sigma_raw = 0.0;
        bool have_sigma = false;
        if (config.slip == DriftSlipModel::Arcsin) {
            const double a_max = std::fabs(config.a_max);
            if (a_max > 1e-6) {
                const double u_raw = a_lat / a_max;
                const bool saturated = u_raw >= 1.0 || u_raw <= -1.0;
                const double u = mathutil::clamp_val(u_raw, -1.0, 1.0);
                sigma_raw = std::asin(u);
                a_actual_lat_ = a_max * u;
                if (!saturated) {
                    const double du = da / a_max;
                    const double d2u = d2a / a_max;
                    const double one_m = 1.0 - u * u;
                    const double root = std::sqrt(one_m);
                    if (root > 1e-8) {
                        sigma_dot = du / root;
                        sigma_ddot = (d2u * one_m + u * du * du) / (one_m * root);
                    }
                }
                have_sigma = true;
            }
        } else if (config.drift_accel > 1e-6) {
            const double a_d = config.drift_accel;
            const double u = a_lat / a_d;
            sigma_raw = std::atan(u);
            const double hypot_a = std::sqrt(a_d * a_d + a_lat * a_lat);
            if (hypot_a > 1e-9) {
                a_actual_lat_ = std::fabs(config.a_max) * a_lat / hypot_a;
            }
            const double du = da / a_d;
            const double d2u = d2a / a_d;
            const double den = 1.0 + u * u;
            sigma_dot = du / den;
            sigma_ddot = (d2u * den - 2.0 * u * du * du) / (den * den);
            have_sigma = true;
        }
        if (have_sigma) {
            if (sigma_raw > config.max_slip_rad || sigma_raw < -config.max_slip_rad) {
                sigma_dot = 0.0;
                sigma_ddot = 0.0;
            }
            sigma = mathutil::clamp_val(sigma_raw, -config.max_slip_rad, config.max_slip_rad);
        }
    }
    if (!std::isfinite(sigma_dot)) sigma_dot = 0.0;
    if (!std::isfinite(sigma_ddot)) sigma_ddot = 0.0;

    double psi = mathutil::normalize_angle(phi + sigma);
    if (!forwards) psi = mathutil::normalize_angle(psi + Robot::PI);
    if (stop_settle && !target.ignore_direction) psi = nose;
    if (stop_settle && target.ignore_direction) psi = theta;
    goal_heading_ = psi;

    double omega_ff = v_ref * kappa + sigma_dot;
    double alpha_ff = a_travel * kappa + v_ref * v_ref * dk + sigma_ddot;
    if (stop_settle) {
        omega_ff = 0.0;
        alpha_ff = 0.0;
    }
    if (!std::isfinite(omega_ff)) omega_ff = 0.0;
    if (!std::isfinite(alpha_ff)) alpha_ff = 0.0;

    const double direction = forwards ? 1.0 : -1.0;
    const double v_cmd = direction * v_plan;
    const double a_cmd = direction * a_travel;
    double radius = config.turn_radius_max;
    if (std::fabs(kappa) > config.kappa_eps) radius = 1.0 / std::fabs(kappa);
    if (radius > config.turn_radius_max) radius = config.turn_radius_max;

    double surge = 0.0;
    double yaw = 0.0;
    if (model_ != nullptr) {
        surge = model_->linearVolts(v_cmd, a_cmd, std::fabs(v_cmd), std::fabs(a_cmd),
                                    omega_ff, radius);
        // angularVolts is V_R − V_L. The chassis command is half of that.
        yaw = 0.5 * model_->angularVolts(omega_ff, alpha_ff, std::fabs(v_cmd),
                                         std::fabs(a_cmd), radius);
    }

    const double heading_deg = mathutil::normalize_angle(psi - theta) * (180.0 / Robot::PI);
    const double scale = voltsPerPower();
    yaw += pidTick(heading_deg,
                   config.head_kP, config.head_kI,
                   config.head_kD, config.head_windup,
                   i_ang_, prev_ang_, head_primed_) * scale;
    head_primed_ = true;

    // Positive cross-track is left of travel. Positive yaw is CCW, so the
    // error is negated and a positive gain steers back onto the path.
    const double e_lat = -lat;
    const double dt = System::DT_SEC;
    double de = 0.0;
    double d2e = 0.0;
    if (lat_primed_ && dt > 1e-6) {
        if (mathutil::sign(e_lat) != mathutil::sign(prev_lat_)) i_lat_ = 0.0;
        de = (e_lat - prev_lat_) / dt;
        d2e = (de - prev_dlat_) / dt;
    }
    i_lat_ += e_lat * dt;
    if (std::fabs(config.lat_kI) > 1e-9) {
        const double i_cap = config.lat_i_limit / std::fabs(config.lat_kI);
        i_lat_ = mathutil::clamp_val(i_lat_, -i_cap, i_cap);
    } else {
        i_lat_ = 0.0;
    }
    prev_dlat_ = de;
    prev_lat_ = e_lat;
    lat_primed_ = true;
    if (!std::isfinite(de)) de = 0.0;
    if (!std::isfinite(d2e)) d2e = 0.0;
    yaw += config.lat_kP * e_lat + config.lat_kI * i_lat_
         + config.lat_kD * de + config.lat_kD2 * d2e;

    const double small_in = config.settle_in;
    const double small_deg = config.settle_deg;
    const double heading_to_goal_deg =
        mathutil::normalize_angle(nose - theta) * (180.0 / Robot::PI);
    const bool heading_ok = target.ignore_direction
        || std::fabs(heading_to_goal_deg) < small_deg;
    settled_ = (dist / kMetersPerInch) < small_in
            && heading_ok
            && std::fabs(v_now - v_end) < config.settle_speed;

    const double max_v = config.max_volts;
    surge = mathutil::clamp_val(surge, -max_v, max_v);
    yaw = mathutil::clamp_val(yaw, -max_v, max_v);
    const double overturn = std::fabs(yaw) + std::fabs(surge) - max_v;
    if (overturn > 0.0) surge -= std::copysign(overturn, surge);

    ChassisCommand cmd;
    cmd.linear = surge;
    cmd.lateral = 0.0;
    cmd.angular = yaw;
    cmd.brake = false;
    return cmd;
}

int DriftFollower::samplePath(const Pose2d& from, Pose2d* out, int cap,
                              double v_lin, double v_ang) const {
    if (out == nullptr || cap < 2) return 0;
    const DriftCurveTarget target = targetInMeters(this->target);
    const Pose2d from_m = poseInchesToMeters(from);
    Sample local[kCap];
    const Sample* src = samples_;
    int n = count_;
    if (!planned_) {
        n = buildPlan(config, target, from_m, v_lin, v_ang, local);
        src = local;
    }
    if (n < 2 || src == nullptr) return 0;
    if (n > cap) {
        const int last = n - 1;
        for (int i = 0; i < cap; ++i) {
            const int idx = (i * last) / (cap - 1);
            out[i].x = metersToInches(src[idx].x);
            out[i].y = metersToInches(src[idx].y);
            out[i].theta = mathutil::normalize_angle(src[idx].phi);
        }
        return cap;
    }
    for (int i = 0; i < n; ++i) {
        out[i].x = metersToInches(src[i].x);
        out[i].y = metersToInches(src[i].y);
        out[i].theta = mathutil::normalize_angle(src[i].phi);
    }
    return n;
}
