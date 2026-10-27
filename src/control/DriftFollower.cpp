/* ---------------------------------------------------------------------------
 * DriftFollower.cpp  --  plan a DriftCurve once, then track it.
 * ------------------------------------------------------------------------- */
#include "control/DriftFollower.h"
#include "control/DriftPathFit.h"
#include "core/Tuning.h"
#include "logic/GoalAligner.h"
#include "math/FrontEdge.h"
#include "math/MathUtils.h"

#include <algorithm>
#include <cmath>

using namespace driftpath;

namespace {

double voltsPerPower() {
    return System::MAX_VOLTS / Controller::FULL_SCALE;
}

// asin(u) derivatives divide by (1−u²)^(3/2). Past this, that term is
// treated as flat so a max-drift u near 1 cannot blow up yaw acceleration.
constexpr double kSlipUMax = 0.95;

// Lateral integrator leak. A sign reset chatters when noise flips a
// straight path between a small positive and a small negative error.
constexpr double kLatITauSec = 0.50;

// Growth past the closest point that ends the leg, inches. One step of
// odometry noise stays under this. It is not a pose window around the target.
constexpr double kDepartIn = 0.25;

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
               double x, double y, double phi, double kappa, double drive = 1.0,
               double v = -1.0) {
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
    s.drive = (drive < 0.0) ? -1.0 : 1.0;
    s.v = v;
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
                  double tx, double ty, double phi, double v_end = -1.0) {
    if (n <= 0) return;
    if (n < cap) {
        DriftFollower::Sample join = out[n - 1];
        join.kappa = 0.0;
        join.dk_ds = 0.0;
        join.d2k_ds2 = 0.0;
        out[n++] = join;
    }
    const int before = n;
    pushPoint(out, n, cap, tx, ty, phi, 0.0);
    if (n > before && n >= 2 && out[n - 2].v >= 0.0 && v_end >= 0.0)
        out[n - 1].v = v_end;
}

void graphPose(const GraphPath& path, double xi, double& phi, double& kappa) {
    graphTangent(path, xi, phi, kappa);
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

void sampleBand(DriftFollower::Sample* out, int& n, int cap, const TimedBand& band,
               double x0, double y0, double x1, double y1) {
    if (!band.curved || band.count < 2) {
        const double phi = (band.count > 0) ? band.phi : 0.0;
        const int n0 = n;
        sampleStraight(out, n, cap, x0, y0, x1, y1, phi);
        const double drive = (band.count > 0 && band.drive[0] < 0.0) ? -1.0 : 1.0;
        const int last = (band.count > 1) ? (band.count - 1) : 0;
        const int written = n - n0;
        for (int i = n0; i < n; ++i) {
            out[i].drive = drive;
            const double u = (written <= 1)
                ? 0.0
                : static_cast<double>(i - n0) / static_cast<double>(written - 1);
            out[i].v = band.v[0] + u * (band.v[last] - band.v[0]);
        }
        return;
    }
    for (int i = 0; i < band.count; ++i) {
        const double drive = (band.drive[i] < 0.0) ? -1.0 : 1.0;
        pushPoint(out, n, cap, band.x[i], band.y[i], band.phi_s[i], band.kappa_s[i],
                  drive, band.v[i]);
    }
    snapEnd(out, n, x1, y1);
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

// Tangent and curvature a short distance ahead of `s_here`. The chord that
// joins the robot to the first band pose flips when the measured nose does.
// Stopping at a drive reversal keeps a cusp's far leg out of the command.
void geometryAhead(const DriftFollower::Sample* samp, int n, double s_here,
                   double& phi, double& kappa, double& dk, double& d2) {
    if (samp == nullptr || n < 1) return;
    double look = kTebLookMin;
    if (n >= 3) {
        const double past_join = samp[2].s - samp[0].s;
        if (look < past_join) look = past_join;
        if (look > kHermiteLookMin) look = kHermiteLookMin;
    }
    int seg = 0;
    for (int i = 1; i < n; ++i) {
        if (samp[i].s + 1e-9 < s_here && i != n - 1) continue;
        seg = i - 1;
        break;
    }
    if (seg < 0) seg = 0;
    const bool back = samp[seg].drive < 0.0;
    double s_cap = samp[n - 1].s;
    int seam = -1;
    for (int i = seg + 1; i < n; ++i) {
        if ((samp[i].drive < 0.0) == back) continue;
        s_cap = samp[i].s;
        seam = i;
        break;
    }
    double s = s_here + look;
    if (s < samp[0].s) s = samp[0].s;
    if (s > s_cap) s = s_cap;
    // The sample on the seam stores the outgoing chord. Reading it turns
    // the nose onto the next leg while the incoming speed is still there.
    // Hold the incoming chord until the robot is actually on that seam.
    if (seam > 0 && s + 1e-9 >= s_cap) {
        phi = samp[seam - 1].phi;
        kappa = 0.0;
        dk = 0.0;
        d2 = 0.0;
        return;
    }
    if (s <= samp[0].s) {
        phi = samp[0].phi;
        kappa = samp[0].kappa;
        dk = samp[0].dk_ds;
        d2 = samp[0].d2k_ds2;
        return;
    }
    for (int i = 1; i < n; ++i) {
        if (samp[i].s + 1e-9 < s && i != n - 1) continue;
        const double ds = samp[i].s - samp[i - 1].s;
        double t = (ds > 1e-9) ? (s - samp[i - 1].s) / ds : 1.0;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        phi = samp[i - 1].phi + t * (samp[i].phi - samp[i - 1].phi);
        kappa = samp[i - 1].kappa + t * (samp[i].kappa - samp[i - 1].kappa);
        dk = samp[i - 1].dk_ds + t * (samp[i].dk_ds - samp[i - 1].dk_ds);
        d2 = samp[i - 1].d2k_ds2 + t * (samp[i].d2k_ds2 - samp[i - 1].d2k_ds2);
        return;
    }
}

// Arc length where the drive direction at `s_here` ends. The far side of a
// reversal stays out of the heading samples.
double driveCap(const DriftFollower::Sample* samp, int n, double s_here) {
    double s_cap = samp[n - 1].s;
    if (n < 2) return s_cap;
    int seg = 0;
    for (int i = 1; i < n; ++i) {
        if (samp[i].s + 1e-9 < s_here && i != n - 1) continue;
        seg = i - 1;
        break;
    }
    if (seg < 0) seg = 0;
    const bool back = samp[seg].drive < 0.0;
    for (int i = seg + 1; i < n; ++i) {
        if ((samp[i].drive < 0.0) == back) continue;
        s_cap = samp[i].s;
        break;
    }
    return s_cap;
}

// Unwrapped travel tangent at arc length s.
bool tangentAt(const DriftFollower::Sample* samp, int n, double s, double& phi) {
    if (samp == nullptr || n < 1) return false;
    if (s <= samp[0].s) {
        phi = samp[0].phi;
        return true;
    }
    if (s >= samp[n - 1].s) {
        phi = samp[n - 1].phi;
        return true;
    }
    for (int i = 1; i < n; ++i) {
        if (samp[i].s + 1e-9 < s && i != n - 1) continue;
        const double ds = samp[i].s - samp[i - 1].s;
        double t = (ds > 1e-9) ? (s - samp[i - 1].s) / ds : 1.0;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        phi = samp[i - 1].phi + t * (samp[i].phi - samp[i - 1].phi);
        return true;
    }
    return false;
}

// FollowerInput::Heading. Stations are kHermiteLookMin apart. The nose
// target is the tangent one station ahead. κ is the heading change over
// the first station, dκ/ds is the change in that rate over the next, and
// d²κ/ds² stays zero. A remainder shorter than kTebLookMin keeps the
// upcoming tangent and drops the curvature.
void headingInput(const DriftFollower::Sample* samp, int n, double s_here,
                  double& phi, double& kappa, double& dk, double& d2) {
    if (samp == nullptr || n < 2) return;
    if (s_here < samp[0].s) s_here = samp[0].s;
    const double s_cap = driveCap(samp, n, s_here);
    if (s_here > s_cap) s_here = s_cap;

    const double step = kHermiteLookMin;
    const double s0 = s_here;
    double s1 = s0 + step;
    if (s1 > s_cap) s1 = s_cap;
    double s2 = s1 + step;
    if (s2 > s_cap) s2 = s_cap;

    double p0 = phi;
    double p1 = phi;
    double p2 = phi;
    if (!tangentAt(samp, n, s0, p0)) return;
    tangentAt(samp, n, s1, p1);
    tangentAt(samp, n, s2, p2);
    // Same seam rule as geometryAhead. The knot at the reversal stores
    // the outgoing chord, and a look that lands on it yaws early.
    if (s1 + 1e-9 >= s_cap && s_cap + 1e-4 < samp[n - 1].s) {
        int incoming = 0;
        while (incoming + 1 < n && samp[incoming + 1].s + 1e-9 < s_cap) ++incoming;
        phi = samp[incoming].phi;
        kappa = 0.0;
        dk = 0.0;
        d2 = 0.0;
        return;
    }

    d2 = 0.0;
    phi = p1;
    const double ds01 = s1 - s0;
    if (ds01 < kTebLookMin) {
        kappa = 0.0;
        dk = 0.0;
        return;
    }
    kappa = (p1 - p0) / ds01;
    const double ds12 = s2 - s1;
    if (ds12 < kTebLookMin) {
        dk = 0.0;
        return;
    }
    const double kappa2 = (p2 - p1) / ds12;
    const double mid = 0.5 * (ds01 + ds12);
    dk = (kappa2 - kappa) / mid;
}

} // namespace

void DriftFollower::reset() {
    count_ = 0;
    planned_ = false;
    track_count_ = 0;
    finish_m_ = 0.0;
    i_ang_ = prev_ang_ = 0.0;
    i_lat_ = prev_lat_ = prev_dlat_ = 0.0;
    head_primed_ = false;
    lat_primed_ = false;
    omega_primed_ = false;
    omega_hold_ = 0.0;
    prev_v_cmd_ = 0.0;
    v_cmd_primed_ = false;
    goal_heading_ = 0.0;
    settled_ = false;
    small_held_ = 0.0;
    large_held_ = 0.0;
    dist_armed_ = false;
    saw_motion_ = false;
    start_dist_m_ = 0.0;
    best_dist_m_ = 0.0;
    drive_hint_ = 0.0;
    a_actual_lat_ = 0.0;
}

// Orbit of a seated nose. Heading sweeps from the measured pose to `nose`,
// and every sample sits half a body length from the pivot with the nose
// pointed at it. phi is that heading, so the field window faces the goal
// instead of the tangent of the arc.
int planGoalOrbit(const Pose2d& from, double nose, DriftFollower::Sample* out, int cap) {
    if (out == nullptr || cap < 2) return 0;
    double gx = 0.0;
    double gy = 0.0;
    goalAlignPivot(poseMetersToInches(from), gx, gy);
    const double error = mathutil::normalize_angle(nose - from.theta);
    const double half_l = Robot::Chassis::BODY_LENGTH_M * 0.5;
    const double kappa = (std::fabs(error) < 1e-6 || half_l < 1e-6)
        ? 0.0
        : ((error > 0.0) ? 1.0 : -1.0) / half_l;
    const int steps = (cap < 17) ? cap : 17;
    int n = 0;
    for (int i = 0; i < steps; ++i) {
        const double u = (steps == 1) ? 1.0 : static_cast<double>(i) / static_cast<double>(steps - 1);
        const double th = from.theta + u * error;
        double x = 0.0;
        double y = 0.0;
        frontedge::placeCenter(x, y, th, gx, gy, 0.0);
        pushPoint(out, n, cap, x, y, th, kappa);
    }
    if (n == 1) pushPoint(out, n, cap, out[0].x, out[0].y, out[0].phi, kappa);
    return n;
}

int DriftFollower::buildPlan(const DriftFollowerConfig& config, const DriftCurveTarget& target,
                             const Pose2d& from, double v_lin, double v_ang, Sample* out,
                             double* finish_m) {
    if (finish_m != nullptr) *finish_m = 0.0;
    if (out == nullptr) return 0;
    if (target.turn_on_goal) {
        const bool forwards = config.forwards && !target.backwards;
        const double nose = forwards
            ? target.exit_direction
            : mathutil::normalize_angle(target.exit_direction + Robot::PI);
        return planGoalOrbit(from, nose, out, kCap);
    }
    const double tx = target.x;
    const double ty = target.y;
    const double dist = std::hypot(tx - from.x, ty - from.y);
    double approach = target.exit_direction;
    if (target.ignore_direction && dist > 1e-4) {
        approach = std::atan2(ty - from.y, tx - from.x);
    }
    const bool forwards = config.forwards && !target.backwards;
    const double nose = forwards
        ? approach
        : mathutil::normalize_angle(approach + Robot::PI);
    const Finish fin = finishOf(from.x, from.y, tx, ty, approach, target.line_length);

    int n = 0;
    // A stop whose miss is sideways (or back past the point) cannot be
    // closed by driving along the exit heading. Chasing that chord swings
    // the nose around and the leg never finishes. Hold, and let the wide
    // settle window hand off.
    const double large_deg = std::max(Boomerang::Angular.large_error, config.settle_deg);
    const double head_err_deg =
        std::fabs(mathutil::normalize_angle(nose - from.theta)) * (180.0 / Robot::PI);
    const bool heading_near = target.ignore_direction || head_err_deg < large_deg;
    const double along_exit =
        (tx - from.x) * std::cos(approach) + (ty - from.y) * std::sin(approach);
    const bool sideways = along_exit < 0.5 * dist;
    if (std::fabs(target.exit_speed) < 1e-3 && heading_near && sideways
        && dist < config.close_m) {
        const double ux = std::cos(approach);
        const double uy = std::sin(approach);
        pushPoint(out, n, kCap, tx - 0.01 * ux, ty - 0.01 * uy, approach, 0.0);
        pushPoint(out, n, kCap, tx, ty, approach, 0.0);
        return n;
    }
    if (dist < 1e-4) {
        const double ux = std::cos(approach);
        const double uy = std::sin(approach);
        pushPoint(out, n, kCap, tx - 0.01 * ux, ty - 0.01 * uy, approach, 0.0);
        pushPoint(out, n, kCap, tx, ty, approach, 0.0);
        return n;
    }

    const bool to_join = fin.length > 1e-4 && !fin.past;
    const double fit_x = to_join ? fin.jx : tx;
    const double fit_y = to_join ? fin.jy : ty;
    const double fit_dist = std::hypot(fit_x - from.x, fit_y - from.y);
    const double ux = std::cos(approach);
    const double uy = std::sin(approach);
    const double lat = (from.x - tx) * (-uy) + (from.y - ty) * ux;
    // A curve shorter than the lookahead is capped near 10/L and the yaw
    // command runs away. Being past the target, or beside the finish, has
    // the same result: the old segment did not start at the robot, so the
    // remaining length collapsed and the move never continued. Drive the
    // chord to the point instead.
    const bool off_finish = fin.on_line && std::fabs(lat) > kOnAxisM;
    const bool short_curve = fit_dist < kHermiteLookMin;
    if (fin.past || off_finish || short_curve) {
        const double phi = std::atan2(ty - from.y, tx - from.x);
        sampleStraight(out, n, kCap, from.x, from.y, tx, ty, phi);
        return n;
    }
    if (fin.on_line) {
        sampleStraight(out, n, kCap, from.x, from.y, tx, ty, approach);
        return n;
    }

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
    if (to_join) {
        if (finish_m != nullptr) *finish_m = fin.length;
        appendFinish(out, n, kCap, tx, ty, approach, std::fabs(target.exit_speed));
    }
    return n;
}

int DriftFollower::adjustElastic(const DriftFollowerConfig& config, const DriftCurveTarget& target,
                                 const Sample* base, int n, double finish_m,
                                 const Pose2d& pose, double v_lin, double v_ang,
                                 double drive_hint, Sample* out) {
    if (base == nullptr || out == nullptr || n < 2) return 0;
    if (base[n - 1].s < 2.0 * kTebMinSegM) return 0;

    int curve_n = n;
    double join_x = base[n - 1].x;
    double join_y = base[n - 1].y;
    double end_x = join_x;
    double end_y = join_y;
    if (finish_m > 1e-4 && n >= 3) {
        curve_n = n - 1;
        join_x = base[n - 2].x;
        join_y = base[n - 2].y;
    }
    if (curve_n < 2) return 0;

    const Proj proj = projectAt(base, curve_n, pose.x, pose.y);
    if (proj.remaining < 2.0 * kTebMinSegM) return 0;
    double s_here = base[curve_n - 1].s - proj.remaining;
    if (s_here < 0.0) s_here = 0.0;

    double xs[kCap];
    double ys[kCap];
    int m = 0;
    // On the stored curve, at the robot's arc length. The measured point
    // sits off that curve by the cross-track, and a band that starts there
    // aims the nose at the error.
    double x_here = base[0].x;
    double y_here = base[0].y;
    if (s_here <= base[0].s) {
        x_here = base[0].x;
        y_here = base[0].y;
    } else if (s_here >= base[curve_n - 1].s) {
        x_here = base[curve_n - 1].x;
        y_here = base[curve_n - 1].y;
    } else {
        for (int i = 1; i < curve_n; ++i) {
            if (base[i].s + 1e-9 < s_here && i != curve_n - 1) continue;
            const double ds = base[i].s - base[i - 1].s;
            double t = (ds > 1e-9) ? (s_here - base[i - 1].s) / ds : 1.0;
            if (t < 0.0) t = 0.0;
            if (t > 1.0) t = 1.0;
            x_here = base[i - 1].x + t * (base[i].x - base[i - 1].x);
            y_here = base[i - 1].y + t * (base[i].y - base[i - 1].y);
            break;
        }
    }
    xs[m] = x_here;
    ys[m] = y_here;
    ++m;
    for (int i = 0; i < curve_n && m < kCap; ++i) {
        if (base[i].s < s_here - 1e-4) continue;
        if (std::hypot(base[i].x - xs[m - 1], base[i].y - ys[m - 1]) < 1e-3) continue;
        xs[m] = base[i].x;
        ys[m] = base[i].y;
        ++m;
    }
    if (m < kCap && std::hypot(join_x - xs[m - 1], join_y - ys[m - 1]) > 1e-3) {
        xs[m] = join_x;
        ys[m] = join_y;
        ++m;
    }
    if (m < 2) return 0;

    const bool forwards = config.forwards && !target.backwards;
    const Momentum momentum = momentumOf(pose.theta, forwards, v_lin, v_ang);
    double accel = target.max_accel.value_or(config.path_accel);
    if (accel < config.min_accel) accel = config.min_accel;
    const SpeedScale pace = scaleSpeed(config.speed_reward, accel, config.centripetal_accel);
    const double approach = base[curve_n - 1].phi;
    TebLimits lim = tebLimits(std::fabs(v_lin), std::fabs(target.exit_speed),
                              config.cruise_speed, pace.accel, pace.centripetal,
                              finish_m, pose.theta, forwards ? 1.0 : -1.0, drive_hint);
    lim.motion = travelHeading(pose.theta, v_lin);
    lim.slip_arcsin = config.slip == DriftSlipModel::Arcsin;
    lim.slip_accel = lim.slip_arcsin ? std::fabs(config.a_max) : config.drift_accel;
    if (lim.slip_accel < 0.0) lim.slip_accel = 0.0;
    lim.slip_limit = config.max_slip_rad;
    copyObstacles(lim, config);
    const TimedBand band = timedBandAlong(xs, ys, m, approach,
                                          momentum.x, momentum.y, momentum.kappa, lim);
    if (band.count < 2) return 0;

    int written = 0;
    const double band_x = (band.count > 0) ? band.x[0] : pose.x;
    const double band_y = (band.count > 0) ? band.y[0] : pose.y;
    sampleBand(out, written, kCap, band, band_x, band_y, join_x, join_y);
    slopeOf(out, written);
    if (finish_m > 1e-4) {
        appendFinish(out, written, kCap, end_x, end_y, approach, std::fabs(target.exit_speed));
    }
    return written;
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
        double v = -1.0;
        if (a.v >= 0.0 && b.v >= 0.0) {
            double tv = t;
            if (tv < 0.0) tv = 0.0;
            if (tv > 1.0) tv = 1.0;
            v = a.v + tv * (b.v - a.v);
        }
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
            best.drive = (a.drive < 0.0) ? -1.0 : 1.0;
            best.v = v;
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

ChassisCommand DriftFollower::commandGoalTurn(const StateEstimate& state,
                                             const DriftCurveTarget& target) {
    const Pose2d pose = poseInchesToMeters(state.pose);
    const bool forwards = config.forwards && !target.backwards;
    const double nose = forwards
        ? target.exit_direction
        : mathutil::normalize_angle(target.exit_direction + Robot::PI);
    goal_heading_ = nose;

    const double theta = std::isfinite(pose.theta) ? pose.theta : 0.0;
    const double error = mathutil::normalize_angle(nose - theta);
    const double w_now = std::isfinite(state.v_ang) ? state.v_ang : 0.0;
    const double tol = config.settle_deg * (Robot::PI / 180.0);
    const double accel = GoalAlign::TURN_YAW_ACCEL;
    const double w_cap = GoalAlign::TURN_YAW_RAD_S;
    const double w_stop = std::sqrt(2.0 * accel * std::fabs(error));
    const double w_mag = (w_stop < w_cap) ? w_stop : w_cap;
    const double w_ideal = mathutil::sign(error) * w_mag;
    const bool in_window = std::fabs(error) <= tol
        && std::fabs(w_now) <= GoalAlign::TURN_SETTLE_RAD_S;

    // Press stays on through the settle window so the bumper is still loaded
    // when the next leg takes the chassis. Yaw follows the brake curve, and
    // a spin that is already faster than that curve is braked the other way.
    double surge = GoalAlign::TURN_SURGE_V;
    double yaw = 0.0;
    if (!in_window) {
        const bool overspeed = std::fabs(w_now) > std::fabs(w_ideal) + GoalAlign::VEL_DEADBAND
            && mathutil::sign(w_now) != 0.0
            && (mathutil::sign(w_ideal) == 0.0 || mathutil::sign(w_now) == mathutil::sign(w_ideal));
        if (overspeed) {
            const double excess = std::fabs(w_now) - std::fabs(w_ideal);
            const double brake = std::min(1.0, excess / w_cap);
            yaw = -mathutil::sign(w_now) * brake * GoalAlign::TURN_YAW_V;
        } else if (w_cap > 1e-6) {
            yaw = mathutil::sign(error) * (w_mag / w_cap) * GoalAlign::TURN_YAW_V;
        }
    }

    if (in_window) small_held_ += System::DT_SEC;
    else small_held_ = 0.0;
    const double small_s = Boomerang::Lateral.small_ms / 1000.0;
    settled_ = small_held_ >= small_s;
    a_actual_lat_ = 0.0;

    const double max_v = config.max_volts;
    const double used = std::fabs(surge) + std::fabs(yaw);
    if (used > max_v && used > 1e-9) {
        const double scale = max_v / used;
        surge *= scale;
        yaw *= scale;
    }

    ChassisCommand cmd;
    cmd.linear = surge;
    cmd.lateral = 0.0;
    cmd.angular = yaw;
    cmd.brake = false;
    return cmd;
}

ChassisCommand DriftFollower::compute(const StateEstimate& state) {
    const DriftCurveTarget target = targetInMeters(this->target);
    const Pose2d pose = poseInchesToMeters(state.pose);
    if (target.turn_on_goal) {
        if (!planned_) {
            count_ = buildPlan(config, target, pose, state.v_lin, state.v_ang, samples_,
                               &finish_m_);
            planned_ = true;
            track_count_ = 0;
            small_held_ = 0.0;
            large_held_ = 0.0;
            dist_armed_ = false;
            saw_motion_ = false;
            settled_ = false;
            head_primed_ = false;
            lat_primed_ = false;
            omega_primed_ = false;
        }
        return commandGoalTurn(state, target);
    }
    if (!planned_) {
        count_ = buildPlan(config, target, pose, state.v_lin, state.v_ang, samples_,
                           &finish_m_);
        planned_ = true;
        head_primed_ = false;
        lat_primed_ = false;
        omega_primed_ = false;
        omega_hold_ = 0.0;
        prev_v_cmd_ = 0.0;
        v_cmd_primed_ = false;
        i_ang_ = prev_ang_ = 0.0;
        i_lat_ = prev_lat_ = prev_dlat_ = 0.0;
        small_held_ = 0.0;
        large_held_ = 0.0;
        dist_armed_ = false;
        saw_motion_ = false;
        track_count_ = 0;
        if (config.timed_elastic && count_ >= 2) {
            track_count_ = adjustElastic(config, target, samples_, count_, finish_m_,
                                         pose, state.v_lin, state.v_ang, drive_hint_, track_);
        }
    }
    const bool elastic = track_count_ >= 2;
    const Sample* track = elastic ? track_ : samples_;
    const int track_n = elastic ? track_count_ : count_;

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
    double path_drive = forwards ? 1.0 : -1.0;
    double v_path = -1.0;
    if (track_n >= 2) {
        const Proj proj = projectAt(track, track_n, rx, ry);
        phi = proj.phi;
        kappa = proj.kappa;
        dk = proj.dk_ds;
        d2k = proj.d2k_ds2;
        lat = proj.lat;
        length_m = proj.remaining;
        const double s_here = track[track_n - 1].s - proj.remaining;
        if (config.input == FollowerInput::Heading) {
            headingInput(track, track_n, s_here, phi, kappa, dk, d2k);
        } else if (elastic) {
            geometryAhead(track, track_n, s_here, phi, kappa, dk, d2k);
        }
        if (elastic) {
            path_drive = proj.drive;
            v_path = proj.v;
            drive_hint_ = track[0].drive;
        }
    }
    if (length_m < 0.0) length_m = 0.0;

    const double v_end = std::fabs(target.exit_speed);
    double accel = target.max_accel.value_or(config.path_accel);
    if (accel < config.min_accel) accel = config.min_accel;
    const SpeedScale pace = scaleSpeed(config.speed_reward, accel, config.centripetal_accel);
    accel = pace.accel;
    SpeedKnot knots[kCap];
    int kn = track_n;
    if (kn > kCap) kn = kCap;
    for (int i = 0; i < kn; ++i) {
        knots[i].s = track[i].s;
        knots[i].kappa = track[i].kappa;
        knots[i].dk = track[i].dk_ds;
    }
    double v_ideal = 0.0;
    if (v_path >= 0.0) {
        // The band publishes a speed on each pose. That magnitude is the
        // target. A tick without one falls through to the kinodynamic cap.
        v_ideal = std::fabs(v_path);
    } else if (kn >= 2) {
        const double s_here = samples_[kn - 1].s - length_m;
        v_ideal = feasibleSpeedAt(knots, kn, s_here, v_end, config.cruise_speed, pace.accel,
                                   pace.centripetal, Robot::MAX_VEL_ANG, pace.yaw_accel);
    } else {
        const double v_brake = std::sqrt(v_end * v_end + 2.0 * accel * length_m);
        v_ideal = (v_brake < config.cruise_speed) ? v_brake : config.cruise_speed;
        if (v_end > config.cruise_speed && v_ideal < v_end) v_ideal = v_end;
    }
    const double drive = path_drive;
    // A large angle between the nose and the travel direction on a short
    // correction is a pivot. speed_reward widens that window so a mild
    // misalignment is driven through instead of stopping to square up.
    const double pivot_len = kHermiteLookMin / pace.reward;
    const double pivot_aim = 0.50 * pace.reward;
    // A published path speed already chose where to slow down, including a
    // zero at a reverse. The pivot stop is only for shapes that have no speed.
    if (v_path < 0.0 && v_end < 1e-3 && length_m < pivot_len && pivot_aim < 2.4) {
        const double face = (drive > 0.0) ? theta : mathutil::normalize_angle(theta + Robot::PI);
        const double aim = std::fabs(mathutil::normalize_angle(phi - face));
        if (aim > pivot_aim) v_ideal = 0.0;
    }
    // Slew the nose speed toward drive * v_ideal from the previous command.
    // Stepping off the measured speed drops the volts when a tile seam slows
    // the chassis, which is when the motor has to keep pushing. The first
    // tick seeds from the measurement so a move does not start at zero.
    const double step = accel * System::DT_SEC;
    const double v_des = drive * v_ideal;
    if (!v_cmd_primed_) {
        prev_v_cmd_ = state.v_lin;
        v_cmd_primed_ = true;
    }
    double v_cmd_slew = v_des;
    if (v_cmd_slew > prev_v_cmd_ + step) v_cmd_slew = prev_v_cmd_ + step;
    if (v_cmd_slew < prev_v_cmd_ - step) v_cmd_slew = prev_v_cmd_ - step;
    const double a_cmd_slew = (System::DT_SEC > 1e-6)
        ? (v_cmd_slew - prev_v_cmd_) / System::DT_SEC
        : 0.0;
    prev_v_cmd_ = v_cmd_slew;
    const double v_travel = drive * v_cmd_slew;
    const double a_travel = drive * a_cmd_slew;
    const double v_ref = (v_travel > 0.0) ? v_travel : 0.0;

    const bool close = dist < config.close_m;
    const double settle_m = config.settle_in * kMetersPerInch;
    const double along_exit =
        (tx - rx) * std::cos(target.exit_direction) +
        (ty - ry) * std::sin(target.exit_direction);
    // Square up only when the exit heading still points at the target.
    // A sideways miss is driven along the chord first; aiming the nose at
    // the exit heading there commands speed away from the point.
    const bool exit_closes = dist < settle_m || along_exit > 0.5 * dist;
    const bool stop_settle = close && v_end < 1e-3 && exit_closes;

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
                // |u| near 1 makes (1−u²)^(3/2) vanish and σ̈ run away.
                const bool flat = u_raw >= kSlipUMax || u_raw <= -kSlipUMax;
                const double u = mathutil::clamp_val(u_raw, -1.0, 1.0);
                sigma_raw = std::asin(u);
                a_actual_lat_ = a_max * u;
                if (!flat) {
                    const double du = da / a_max;
                    const double d2u = d2a / a_max;
                    const double u_d = mathutil::clamp_val(u, -kSlipUMax, kSlipUMax);
                    const double one_m = 1.0 - u_d * u_d;
                    const double root = std::sqrt(one_m);
                    if (root > 1e-8) {
                        sigma_dot = du / root;
                        sigma_ddot = (d2u * one_m + u_d * du * du) / (one_m * root);
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
    sigma_dot = mathutil::clamp_val(sigma_dot, -Robot::MAX_VEL_ANG, Robot::MAX_VEL_ANG);
    sigma_ddot = mathutil::clamp_val(sigma_ddot, -Physics::MAX_ACCEL_ANG, Physics::MAX_ACCEL_ANG);

    double psi = mathutil::normalize_angle(phi + sigma);
    if (drive < 0.0) psi = mathutil::normalize_angle(psi + Robot::PI);
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
    // ROS TEB's angular-acceleration edge. The lookahead tangent is the
    // heading target; this only stops a one-tick curvature spike from
    // reversing the yaw feedforward and spending the whole voltage rail.
    const double w_cap = Robot::MAX_VEL_ANG;
    const double alpha_cap = Physics::MAX_ACCEL_ANG;
    const double dt_slew = System::DT_SEC;
    if (!omega_primed_) {
        omega_hold_ = mathutil::clamp_val(state.v_ang, -w_cap, w_cap);
        omega_primed_ = true;
    }
    if (dt_slew > 1e-6 && alpha_cap > 1e-6) {
        const double dw = alpha_cap * dt_slew;
        const double stepped = mathutil::clamp_val(omega_ff, omega_hold_ - dw, omega_hold_ + dw);
        if (stepped != omega_ff) alpha_ff = (stepped - omega_hold_) / dt_slew;
        omega_ff = mathutil::clamp_val(stepped, -w_cap, w_cap);
    }
    omega_hold_ = omega_ff;

    const double v_cmd = v_cmd_slew;
    const double a_cmd = a_cmd_slew;
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
        de = (e_lat - prev_lat_) / dt;
        d2e = (de - prev_dlat_) / dt;
    }
    // Leaky integral. Clearing it on a sign change kicks the yaw every time
    // noise crosses zero on a straight path.
    if (kLatITauSec > 1e-6) i_lat_ += (e_lat - i_lat_ / kLatITauSec) * dt;
    else i_lat_ += e_lat * dt;
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

    // End the leg when the chassis has stopped, or when the range to the
    // target leaves the closest point of this leg. Neither waits for the
    // pose to enter a coordinate window.
    const double depart_m = kDepartIn * kMetersPerInch;
    if (!dist_armed_) {
        start_dist_m_ = dist;
        best_dist_m_ = dist;
        dist_armed_ = true;
    } else if (dist < best_dist_m_) {
        best_dist_m_ = dist;
    }
    if (std::fabs(state.v_lin) > config.settle_speed) saw_motion_ = true;

    // A reversal is a stop in the middle of the path. Later stations speed
    // up again, so that zero is not the end of the leg.
    bool path_resumes = false;
    if (v_path >= 0.0 && v_path <= config.settle_speed && track_n >= 2) {
        const double s_now = track[track_n - 1].s - length_m;
        for (int i = 0; i < track_n; ++i) {
            if (track[i].s <= s_now + 1e-3) continue;
            if (track[i].v > config.settle_speed) {
                path_resumes = true;
                break;
            }
        }
    }

    const bool closed_in = best_dist_m_ < start_dist_m_ - depart_m;
    const bool departing = closed_in && dist > best_dist_m_ + depart_m;
    const bool measured_stop = std::fabs(state.v_lin) <= config.settle_speed;
    const bool profile_stop = (v_path >= 0.0)
        ? (v_path <= config.settle_speed && !path_resumes)
        : (v_ideal <= config.settle_speed);
    // Sitting still on the first ticks is the start of the move. It counts
    // once the chassis has been moving, and also when the profile is already
    // asking for a stop on top of the target.
    const bool stopped = measured_stop && !path_resumes
        && (saw_motion_ || (profile_stop && length_m <= depart_m));
    settled_ = stopped || departing;

    const double max_v = config.max_volts;
    surge = mathutil::clamp_val(surge, -max_v, max_v);
    yaw = mathutil::clamp_val(yaw, -max_v, max_v);
    // Yaw wins the rail. Shedding it to keep reverse thrust leaves a slide
    // with nothing to point the driven wheels into, and the chassis spins out.
    const double overturn = std::fabs(yaw) + std::fabs(surge) - max_v;
    if (overturn > 0.0) surge -= std::copysign(overturn, surge);

    ChassisCommand cmd;
    cmd.linear = surge;
    cmd.lateral = 0.0;
    cmd.angular = yaw;
    cmd.brake = false;
    return cmd;
}

static int writeSamples(const DriftFollower::Sample* src, int n, Pose2d* out, int cap) {
    if (src == nullptr || out == nullptr || n < 2 || cap < 2) return 0;
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

int DriftFollower::sampleBase(const Pose2d& from, Pose2d* out, int cap,
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
    return writeSamples(src, n, out, cap);
}

int DriftFollower::samplePath(const Pose2d& from, Pose2d* out, int cap,
                              double v_lin, double v_ang) const {
    if (out == nullptr || cap < 2) return 0;
    const DriftCurveTarget target = targetInMeters(this->target);
    const Pose2d from_m = poseInchesToMeters(from);
    Sample local[kCap];
    Sample adjusted[kCap];
    const Sample* src = samples_;
    int n = count_;
    if (!planned_) {
        double finish = 0.0;
        n = buildPlan(config, target, from_m, v_lin, v_ang, local, &finish);
        src = local;
        if (config.timed_elastic && !target.turn_on_goal) {
            const int an = adjustElastic(config, target, local, n, finish,
                                         from_m, v_lin, v_ang, 0.0, adjusted);
            if (an >= 2) {
                src = adjusted;
                n = an;
            }
        }
    } else if (config.timed_elastic && track_count_ >= 2) {
        src = track_;
        n = track_count_;
    }
    return writeSamples(src, n, out, cap);
}
