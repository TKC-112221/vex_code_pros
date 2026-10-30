/* ---------------------------------------------------------------------------
 * SegmentFollower.cpp  --  plan a polyline once, then track it.
 * ------------------------------------------------------------------------- */
#include "control/SegmentFollower.h"
#include "control/YawArmature.h"
#include "math/MathUtils.h"
#include "math/TrackingCenter.h"

#include <cmath>

namespace {

constexpr int kObstacles = kPathObstacles;
constexpr int kSides = 8;
// Two extra nodes per obstacle: a gate on the ray out of a circle the
// robot starts in, and a gate on the ray into a circle the goal sits in.
constexpr int kNodes = 2 + kObstacles * kSides + 2 * kObstacles;
// Cross-track to the next segment, or to the line it lies on. The corner
// pivot starts inside this band. The final stop still uses it as the ball
// around the end point.
constexpr double kArriveM = 0.01;
// How far short of the vertex, along that line, still counts as the
// corner. Further back is the approach, and the chassis drives it at the
// brake curve instead of pivoting early. A miss of this size along the
// segment is left where the pivot finishes.
constexpr double kAlongM = 0.02;
// Body speed of the arc that walks the centre onto the next segment.
constexpr double kSlideMax = 0.20;
// The arc's centre stays this close to the chassis. Farther than that
// the bumper leaves the corner the path was built around.
constexpr double kRhoMax = 0.08;
// A heading change above this is a turn. The chassis stops and yaws there.
constexpr double kTurnRad = 1.0e-2;
// A stop inside this distance of a vertex still belongs to that corner.
// Farther away is the approach, and the brake curve is still driving it.
constexpr double kCornerNearM = 0.15;
// Along-track distance that counts as arrived at a stop vertex. The pivot
// starts from a stop inside this, including when the centre is still off
// the next line. The slide closes that miss during the yaw.
constexpr double kVertexArriveM = 0.05;
// Projection that has reached the far vertex of a segment that does not stop.
constexpr double kPassT = 0.999;
// Still yawing. Settled and the pre-pivot bleed use this. The pivot
// itself stays up until the rate is quieter, so the armature is not cut
// off while the chassis is still turning.
constexpr double kPivotRate = 0.4;
constexpr double kPivotQuiet = 0.15;
// A vertex, and the end of the path, stay stopped until forward speed is
// actually gone. settle_speed still means the leg is slow, and 0.2 m/s
// rolls through the shallow join into the straight finish.
constexpr double kStopMps = 0.05;
// The brake curve is the fastest speed that still stops on the vertex. The
// command is then held for the tick, so that speed is still moving when the
// point arrives. Dropping this much of the current travel out of the
// distance leaves the stop on the vertex instead of past it.
constexpr double kBrakeLeadTicks = 2.0;
// Share of the configured path accel the profile does not spend. The brake
// curve and the speedup slew use the rest. A chassis that is already faster
// than the curve can brake with the whole armature accel, so that
// correction is not capped at the path accel.
constexpr double kAccelOverhead = 0.30;
// Measured speed above the curve is shed on this horizon. The decel that
// comes out of it may exceed the path accel, up to the armature accel.
constexpr double kSpeedCatchSec = 0.15;
double profileAccel(double configured, double floor) {
    double a = configured * (1.0 - kAccelOverhead);
    if (a < floor) a = floor;
    if (a > configured) a = configured;
    return a;
}

double noseOf(double travel, double drive) {
    if (drive < 0.0) travel += Robot::PI;
    return mathutil::normalize_angle(travel);
}

// Seconds to yaw through `err` radians on the turn trapezoid.
double yawSeconds(double err) {
    err = std::fabs(err);
    const double w = Turn::MAX_VEL_RAD_S;
    const double a = Turn::MAX_ACCEL_RAD_S2;
    if (err < 1e-3 || !(w > 1e-3) || !(a > 1e-3)) return 0.0;
    const double tri = (w * w) / a;
    if (err <= tri) return 2.0 * std::sqrt(err / a);
    return (2.0 * w / a) + (err - tri) / w;
}

// Seconds to cover `dist` meters from v0 to v1, accel-limited, at or under cruise.
double trapSeconds(double dist, double v0, double v1, double accel, double cruise) {
    if (dist <= 1e-4) return 0.0;
    if (!(accel > 1e-3)) return 0.0;
    if (cruise < 1e-3) return 0.0;
    if (v0 < 0.0) v0 = 0.0;
    if (v1 < 0.0) v1 = 0.0;
    if (v0 > cruise) v0 = cruise;
    if (v1 > cruise) v1 = cruise;
    const double direct = std::fabs(v1 * v1 - v0 * v0) / (2.0 * accel);
    if (direct >= dist - 1e-6) {
        const double sign = (v1 >= v0) ? 1.0 : -1.0;
        const double v_sq = v0 * v0 + sign * 2.0 * accel * dist;
        const double v_end = std::sqrt(v_sq > 0.0 ? v_sq : 0.0);
        return std::fabs(v_end - v0) / accel;
    }
    const double peak_sq = accel * dist + 0.5 * (v0 * v0 + v1 * v1);
    double peak = std::sqrt(peak_sq > 0.0 ? peak_sq : 0.0);
    if (peak > cruise) peak = cruise;
    if (peak < v0) peak = v0;
    if (peak < v1) peak = v1;
    const double up = (peak * peak - v0 * v0) / (2.0 * accel);
    const double down = (peak * peak - v1 * v1) / (2.0 * accel);
    double cruise_d = dist - up - down;
    if (cruise_d < 0.0) cruise_d = 0.0;
    double t = 0.0;
    if (peak > v0 + 1e-6) t += (peak - v0) / accel;
    if (cruise_d > 1e-6 && peak > 1e-3) t += cruise_d / peak;
    if (peak > v1 + 1e-6) t += (peak - v1) / accel;
    return t;
}

struct Pace {
    double accel = 0;
    double cruise = 0;
    double v_end = 0;
};

Pace paceOf(const SegmentFollowerConfig& config, const DriftCurveTarget& target) {
    Pace pace;
    pace.accel = target.max_accel.value_or(config.path_accel);
    if (pace.accel < config.min_accel) pace.accel = config.min_accel;
    pace.v_end = target.exit_speed;
    if (pace.v_end < 0.0) pace.v_end = 0.0;
    pace.cruise = config.cruise_speed;
    if (pace.cruise < pace.v_end) pace.cruise = pace.v_end;
    if (pace.cruise < 0.0) pace.cruise = 0.0;
    return pace;
}

struct Circle {
    double x = 0;
    double y = 0;
    double r = 0;
};

bool inField(double x, double y) {
    const double m = WallSolve::FIELD_MARGIN_M;
    return x >= Robot::Range::FIELD_X_MIN_M + m
        && x <= Robot::Range::FIELD_X_MAX_M - m
        && y >= Robot::Range::FIELD_Y_MIN_M + m
        && y <= Robot::Range::FIELD_Y_MAX_M - m;
}

double segmentGap(double ax, double ay, double bx, double by, double cx, double cy) {
    const double abx = bx - ax;
    const double aby = by - ay;
    const double len2 = abx * abx + aby * aby;
    double t = 0.0;
    if (len2 > 1e-12) {
        t = ((cx - ax) * abx + (cy - ay) * aby) / len2;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
    }
    const double px = ax + t * abx;
    const double py = ay + t * aby;
    return std::hypot(cx - px, cy - py);
}

bool holds(const Circle& c, double x, double y) {
    return std::hypot(x - c.x, y - c.y) < c.r - 1e-4;
}

// True when (x, y) lies on the line through the obstacle and the goal.
// A few degrees of slack covers the gate, which is built on that ray.
bool colinearWithGoal(const Circle& c, double gx, double gy, double x, double y) {
    const double ox = gx - c.x;
    const double oy = gy - c.y;
    const double px = x - c.x;
    const double py = y - c.y;
    const double glen = std::hypot(ox, oy);
    const double plen = std::hypot(px, py);
    if (!(glen > 1e-6) || !(plen > 1e-6)) return false;
    const double cross = px * oy - py * ox;
    return std::fabs(cross) <= 0.02 * plen * glen;
}

// touch_start / touch_goal: this segment leaves the robot or arrives at the
// goal. A circle the robot is already inside can be crossed, but not closer
// to the obstacle than the robot already is. A circle that holds the goal
// can be entered only along the ray through that goal, and not closer than
// the nearer of the robot and the goal. Every other segment stays outside.
bool lineClear(double ax, double ay, double bx, double by,
               const Circle* circ, int nc,
               bool touch_start, bool touch_goal,
               double sx, double sy, double gx, double gy) {
    for (int i = 0; i < nc; ++i) {
        const Circle& c = circ[i];
        const double gap = segmentGap(ax, ay, bx, by, c.x, c.y);
        const bool start_in = touch_start && holds(c, sx, sy);
        const bool goal_in = touch_goal && holds(c, gx, gy);
        if (goal_in) {
            const bool a_goal = std::hypot(ax - gx, ay - gy) < 1e-3;
            const bool b_goal = std::hypot(bx - gx, by - gy) < 1e-3;
            if (!a_goal && !b_goal) return false;
            const double fx = a_goal ? bx : ax;
            const double fy = a_goal ? by : ay;
            if (!colinearWithGoal(c, gx, gy, fx, fy)) return false;
            double limit = std::hypot(gx - c.x, gy - c.y);
            if (start_in) {
                const double start_dist = std::hypot(sx - c.x, sy - c.y);
                if (start_dist < limit) limit = start_dist;
            }
            if (gap + 1e-4 < limit) return false;
            continue;
        }
        if (start_in) {
            const double start_dist = std::hypot(sx - c.x, sy - c.y);
            if (gap + 1e-4 < start_dist) return false;
            continue;
        }
        if (gap < c.r - 1e-4) return false;
    }
    return true;
}

// Smallest gap from the bumper to (ox, oy) while the nose swings from h0 to h1
// the short way. The bumper orbits the rotation centre at the tracking offset.
double bumperArcGap(double cx, double cy, double h0, double h1, double ox, double oy) {
    const double lever = Robot::Chassis::TRACKING_FORWARD_M;
    const double sweep = mathutil::normalize_angle(h1 - h0);
    auto gapAt = [&](double h) {
        const double bx = cx + lever * mathutil::heading_cos(h);
        const double by = cy + lever * mathutil::heading_sin(h);
        return std::hypot(ox - bx, oy - by);
    };
    double best = gapAt(h0);
    const double end = gapAt(h0 + sweep);
    if (end < best) best = end;
    if (std::hypot(ox - cx, oy - cy) < 1e-6) return best;
    const double toward = mathutil::direction_heading(ox - cx, oy - cy);
    const double along = mathutil::normalize_angle(toward - h0);
    const bool inside = (sweep >= 0.0) ? (along >= -1e-8 && along <= sweep + 1e-8)
                                        : (along <= 1e-8 && along >= sweep - 1e-8);
    if (inside) {
        const double mid = gapAt(toward);
        if (mid < best) best = mid;
    }
    return best;
}

// A pivot already inside a circle may yaw, but the bumper may not swing
// closer to that obstacle than it is at either end of the turn. Every
// other circle must clear the bumper arc.
bool pivotArcClear(double cx, double cy, double h0, double h1,
                   const Circle* circ, int nc) {
    const double lever = Robot::Chassis::TRACKING_FORWARD_M;
    for (int i = 0; i < nc; ++i) {
        const Circle& c = circ[i];
        const double gap = bumperArcGap(cx, cy, h0, h1, c.x, c.y);
        double limit = c.r;
        if (holds(c, cx, cy)) {
            auto bumperGap = [&](double h) {
                const double bx = cx + lever * mathutil::heading_cos(h);
                const double by = cy + lever * mathutil::heading_sin(h);
                return std::hypot(c.x - bx, c.y - by);
            };
            limit = bumperGap(h0);
            const double end = bumperGap(h1);
            if (end < limit) limit = end;
        }
        if (gap < limit - 1e-4) return false;
    }
    return true;
}

// Where (x, y) sits on the infinite line through the segment. t is 0 at
// the start and 1 at the end, and it is not clamped. lat is metres to
// the left of travel.
void segmentFrame(double ax, double ay, double bx, double by, double x, double y,
                  double& t, double& lat) {
    const double abx = bx - ax;
    const double aby = by - ay;
    const double len2 = abx * abx + aby * aby;
    const double len = std::sqrt(len2);
    t = 0.0;
    lat = 0.0;
    if (len2 < 1e-12) {
        lat = std::hypot(x - ax, y - ay);
        return;
    }
    t = ((x - ax) * abx + (y - ay) * aby) / len2;
    const double inv = 1.0 / len;
    const double ux = abx * inv;
    const double uy = aby * inv;
    const double px = ax + t * abx;
    const double py = ay + t * aby;
    lat = (x - px) * (-uy) + (y - py) * ux;
}

// The corner is the neighbourhood of the vertex on the next segment's
// line: within 1 cm of the line, and within kAlongM of the vertex along
// it, including the extension before the vertex. Further down the segment
// is the next leg, not this corner.
bool onNextLine(double ax, double ay, double bx, double by, double x, double y) {
    double t = 0.0;
    double lat = 0.0;
    segmentFrame(ax, ay, bx, by, x, y, t, lat);
    // A millimetre past the band is still the corner. Stopping on the
    // boundary and treating it as a miss leaves the pivot unstarted.
    if (std::fabs(lat) > kArriveM + 1e-3) return false;
    const double along = t * std::hypot(bx - ax, by - ay);
    return std::fabs(along) <= kAlongM + 1e-3;
}

// Distance from (x, y) to that neighbourhood. Zero once the pivot may start.
double lineStripGap(double ax, double ay, double bx, double by, double x, double y) {
    double t = 0.0;
    double lat = 0.0;
    segmentFrame(ax, ay, bx, by, x, y, t, lat);
    const double along = t * std::hypot(bx - ax, by - ay);
    const double cross = std::fabs(lat);
    if (cross <= kArriveM && std::fabs(along) <= kAlongM) return 0.0;
    double target_along = along;
    if (target_along < -kAlongM) target_along = -kAlongM;
    if (target_along > kAlongM) target_along = kAlongM;
    const double target_cross = (cross > kArriveM) ? kArriveM : cross;
    return std::hypot(along - target_along, cross - target_cross);
}

// Body-forward speed for a corner pivot. While the nose is still angled
// off the next segment, forward speed changes cross-track at
// -drive * v * sin(err). Choosing v so that rate is -lat * |ω| walks the
// centre onto the line as the nose arrives, and the miss along the line
// stays. The soft denominator keeps the speed finite and at zero once the
// nose is on the line, so the command does not flip sign at the end.
double pivotSlideMps(double cx, double cy, double theta,
                     double ax, double ay, double bx, double by,
                     double /*travel*/, double nose, double omega_cmd, double drive) {
    double t = 0.0;
    double lat = 0.0;
    segmentFrame(ax, ay, bx, by, cx, cy, t, lat);
    const double err = mathutil::normalize_angle(nose - theta);
    // Inside the arrival band the centre is close enough. A slide there
    // reverses every time the line is crossed, and each reversal pays the
    // static voltage.
    if (std::fabs(lat) <= kArriveM) return 0.0;
    const double s = std::sin(err);
    if (!std::isfinite(omega_cmd) || std::fabs(omega_cmd) < 1e-3) return 0.0;
    double v = drive * lat * std::fabs(omega_cmd) * s / (s * s + 0.05);
    const double rho_cap = std::fabs(omega_cmd) * kRhoMax;
    if (v > rho_cap) v = rho_cap;
    if (v < -rho_cap) v = -rho_cap;
    if (v > kSlideMax) v = kSlideMax;
    if (v < -kSlideMax) v = -kSlideMax;
    return v;
}

// Yaw rate for this tick. The stopping curve is the fastest rate that
// still finishes on the heading. The command ramps up to it, and from a
// stop it steps only as far as the stiction rate so the standing voltage
// can break the pivot loose. A command already faster than the curve
// drops to the curve: that is what lets the velocity term brake.
double slewYawRate(double held, double error, bool pivot, double omega_now) {
    const double accel = Turn::MAX_ACCEL_RAD_S2;
    const double w_max = Turn::MAX_VEL_RAD_S;
    double w_stop = std::sqrt(2.0 * accel * std::fabs(error));
    if (!std::isfinite(w_stop) || w_stop < 0.0) w_stop = 0.0;
    if (w_stop > w_max) w_stop = w_max;
    const double target = mathutil::sign(error) * w_stop;
    double w = std::isfinite(held) ? held : 0.0;
    const bool wrong_way = (w * target < -1e-6);
    const bool too_fast = std::fabs(w) > std::fabs(target) + 1e-4;
    if (wrong_way || too_fast) {
        w = target;
    } else if (pivot
               && std::fabs(omega_now) < kYawStictionRadS
               && std::fabs(w) < kYawStictionRadS
               && std::fabs(target) >= kYawStictionRadS) {
        w = mathutil::sign(target) * kYawStictionRadS;
    } else {
        const double step = accel * System::DT_SEC;
        const double delta = target - w;
        if (delta > step) w += step;
        else if (delta < -step) w -= step;
        else w = target;
    }
    return w;
}

bool fieldOk(double ax, double ay, double bx, double by, bool bound) {
    if (!bound) return true;
    const bool ain = inField(ax, ay);
    const bool bin = inField(bx, by);
    // Both ends inside a convex field keeps the segment inside. One end
    // outside is the robot or the goal crossing the margin. Both outside
    // would route through the wall.
    if (ain && bin) return true;
    if (!ain && !bin) return false;
    return true;
}

} // namespace

void SegmentFollower::project(const Vertex* pts, int seg, double rx, double ry,
             double& t, double& lat, double& remain) {
    const double ax = pts[seg].x;
    const double ay = pts[seg].y;
    const double bx = pts[seg + 1].x;
    const double by = pts[seg + 1].y;
    const double abx = bx - ax;
    const double aby = by - ay;
    const double len2 = abx * abx + aby * aby;
    const double len = std::sqrt(len2);
    t = 0.0;
    if (len2 > 1e-12) {
        t = ((rx - ax) * abx + (ry - ay) * aby) / len2;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
    }
    const double px = ax + t * abx;
    const double py = ay + t * aby;
    const double inv = (len > 1e-9) ? (1.0 / len) : 0.0;
    const double ux = abx * inv;
    const double uy = aby * inv;
    // Left of the direction of travel.
    lat = (rx - px) * (-uy) + (ry - py) * ux;
    remain = (1.0 - t) * len;
}

void SegmentFollower::stamp(Vertex* pts, int n, const SegmentFollowerConfig& config,
                           const DriftCurveTarget& target, double accel, double cruise_in,
                           double v_end) {
    if (pts == nullptr || n < 2) return;
    pts[0].s = 0.0;
    for (int i = 1; i < n; ++i) {
        pts[i].s = pts[i - 1].s
                 + std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    }
    for (int i = 0; i < n - 1; ++i) {
        pts[i].phi = mathutil::direction_heading(pts[i + 1].x - pts[i].x,
                                                pts[i + 1].y - pts[i].y);
        pts[i].stop = false;
    }
    pts[n - 1].phi = pts[n - 2].phi;
    pts[n - 1].stop = false;

    const int nseg = n - 1;
    const bool forwards = config.forwards && !target.backwards;
    pts[nseg - 1].drive = forwards ? 1.0 : -1.0;
    for (int i = nseg - 2; i >= 0; --i) {
        const double nose_next = noseOf(pts[i + 1].phi, pts[i + 1].drive);
        const double turn_pos = std::fabs(mathutil::normalize_angle(nose_next - pts[i].phi));
        const double turn_neg = std::fabs(mathutil::normalize_angle(nose_next - (pts[i].phi + Robot::PI)));
        if (turn_neg + 1e-3 < turn_pos) pts[i].drive = -1.0;
        else if (turn_pos + 1e-3 < turn_neg) pts[i].drive = 1.0;
        else pts[i].drive = pts[i + 1].drive;
    }
    pts[n - 1].drive = pts[n - 2].drive;

    for (int i = 1; i <= n - 2; ++i) {
        const double nose0 = noseOf(pts[i - 1].phi, pts[i - 1].drive);
        const double nose1 = noseOf(pts[i].phi, pts[i].drive);
        const double turn = std::fabs(mathutil::normalize_angle(nose1 - nose0));
        const bool flip = (pts[i].drive < 0.0) != (pts[i - 1].drive < 0.0);
        pts[i].stop = flip || turn > kTurnRad;
    }

    double cruise = cruise_in;
    if (cruise < 0.0) cruise = 0.0;
    pts[n - 1].v = v_end;
    if (pts[n - 1].v < 0.0) pts[n - 1].v = 0.0;
    if (pts[n - 1].v > cruise) pts[n - 1].v = cruise;
    for (int i = n - 2; i >= 0; --i) {
        double ds = pts[i + 1].s - pts[i].s;
        if (ds < 1e-6) ds = 1e-6;
        double cap = cruise;
        if (pts[i].stop) cap = 0.0;
        const double vb = std::sqrt(pts[i + 1].v * pts[i + 1].v + 2.0 * accel * ds);
        pts[i].v = (vb < cap) ? vb : cap;
    }
}

int SegmentFollower::dropCollinear(Vertex* pts, int n) {
    if (n < 3) return n;
    int w = 1;
    for (int i = 1; i < n - 1; ++i) {
        const double ph0 = std::atan2(pts[i].y - pts[w - 1].y, pts[i].x - pts[w - 1].x);
        const double ph1 = std::atan2(pts[i + 1].y - pts[i].y, pts[i + 1].x - pts[i].x);
        if (std::fabs(mathutil::normalize_angle(ph1 - ph0)) < 1e-3) continue;
        pts[w++] = pts[i];
    }
    pts[w++] = pts[n - 1];
    return w;
}

void SegmentFollower::reset() {
    count_ = 0;
    planned_ = false;
    seg_ = 0;
    started_ = false;
    held_turn_ = false;
    corner_hold_ = false;
    spin_latched_ = false;
    yaw_rate_ = 0.0;
    accel_ = 0.0;
    cruise_ = 0.0;
    prev_v_cmd_ = 0.0;
    v_cmd_primed_ = false;
    goal_heading_ = 0.0;
    a_actual_lat_ = 0.0;
    settled_ = false;
    stuck_watch_.reset();
    seconds_left_ = 0.0;
}

double SegmentFollower::etaSeconds(int seg, double along, double v_now, bool yawing,
                                  bool corner, double yaw_err) const {
    if (count_ < 2 || seg < 0 || seg > count_ - 2) return 0.0;
    double t = 0.0;
    if (yawing) t += yawSeconds(yaw_err);
    double v0 = v_now;
    if (v0 < 0.0) v0 = 0.0;
    if (yawing) v0 = 0.0;
    if (v0 > cruise_) v0 = cruise_;
    for (int i = seg; i < count_ - 1; ++i) {
        double d;
        double start_v;
        if (i == seg) {
            d = (pts_[i + 1].s - pts_[i].s) * (1.0 - along);
            start_v = v0;
        } else {
            d = pts_[i + 1].s - pts_[i].s;
            start_v = pts_[i].v;
            if (pts_[i].stop) {
                const bool already = corner && i == seg + 1;
                if (!already && i > 0) {
                    const double nose0 = noseOf(pts_[i - 1].phi, pts_[i - 1].drive);
                    const double nose1 = noseOf(pts_[i].phi, pts_[i].drive);
                    t += yawSeconds(mathutil::normalize_angle(nose1 - nose0));
                }
                start_v = 0.0;
            }
        }
        if (d < 0.0) d = 0.0;
        const double end_v = pts_[i + 1].v;
        t += trapSeconds(d, start_v, end_v, accel_, cruise_);
    }
    if (!std::isfinite(t) || t < 0.0) return 0.0;
    return t;
}

int SegmentFollower::build(const SegmentFollowerConfig& config, const DriftCurveTarget& target_m,
                           const Pose2d& from_m, Vertex* out) {
    if (out == nullptr) return 0;
    const Pace pace = paceOf(config, target_m);
    const double a_plan = profileAccel(pace.accel, config.min_accel);
    const double tx = target_m.x;
    const double ty = target_m.y;
    const double approach = target_m.exit_direction;
    double jx = tx;
    double jy = ty;
    bool have_finish = false;
    if (!target_m.ignore_direction && target_m.line_length > 1e-4) {
        jx = tx - target_m.line_length * mathutil::heading_cos(approach);
        jy = ty - target_m.line_length * mathutil::heading_sin(approach);
        have_finish = std::hypot(tx - jx, ty - jy) > 1e-4;
    }

    Circle circ[kObstacles];
    int nc = 0;
    int listed = config.obstacle_n;
    if (listed < 0) listed = 0;
    if (listed > kObstacles) listed = kObstacles;
    for (int i = 0; i < listed; ++i) {
        if (!(config.obstacles[i].min_dist > 0.0)) continue;
        Circle& c = circ[nc];
        c.x = inchesToMeters(config.obstacles[i].x);
        c.y = inchesToMeters(config.obstacles[i].y);
        // min_dist is the obstacle. The grown radius is the bumper's
        // keep-out: the far corner, half a track out to the side, and
        // kSegmentClearMarginM. A fixed obstacle is already that circle.
        // Corner vertices sit a bumper lever further out so a pivot cannot
        // swing the bumper into it.
        const double obstacle_m = inchesToMeters(config.obstacles[i].min_dist);
        c.r = config.obstacles[i].fixed ? obstacle_m : segmentKeepRadiusM(obstacle_m);
        if (!(c.r > 1e-3) || !std::isfinite(c.x) || !std::isfinite(c.y)) continue;
        ++nc;
    }

    // The straight finish is not searched. If it would cut a keep-out,
    // plan all the way to the target instead.
    if (have_finish && !lineClear(jx, jy, tx, ty, circ, nc, false, true, tx, ty, tx, ty)) {
        jx = tx;
        jy = ty;
        have_finish = false;
    }

    int n = 0;
    auto push = [&](double x, double y) {
        if (n >= kCap) return;
        if (have_finish && n >= kCap - 1) return;
        if (n > 0 && std::hypot(x - out[n - 1].x, y - out[n - 1].y) < 1e-3) return;
        out[n].x = x;
        out[n].y = y;
        ++n;
    };

    const double rx = from_m.x;
    const double ry = from_m.y;
    if (std::hypot(rx - jx, ry - jy) < 1e-3) {
        push(jx, jy);
    } else {
        double nx[kNodes];
        double ny[kNodes];
        int nodes = 0;
        nx[nodes] = rx;
        ny[nodes] = ry;
        ++nodes;
        nx[nodes] = jx;
        ny[nodes] = jy;
        ++nodes;
        const double lever = Robot::Chassis::TRACKING_FORWARD_M;
        const double half = Robot::PI / static_cast<double>(kSides);
        const double outward = 1.0 / std::cos(half);
        for (int c = 0; c < nc; ++c) {
            for (int k = 0; k < kSides; ++k) {
                if (nodes >= kNodes) break;
                const double ang = (2.0 * Robot::PI * static_cast<double>(k))
                                 / static_cast<double>(kSides);
                // A full pivot at this vertex swings the bumper inward by
                // the lever. The vertex sits that far outside the keep-out.
                const double rad = (circ[c].r + lever) * outward;
                const double x = circ[c].x + rad * std::cos(ang);
                const double y = circ[c].y + rad * std::sin(ang);
                if (config.field_bound && !inField(x, y)) continue;
                bool buried = false;
                for (int o = 0; o < nc; ++o) {
                    if (o == c) continue;
                    if (std::hypot(x - circ[o].x, y - circ[o].y) < circ[o].r + lever)
                        buried = true;
                }
                if (buried) continue;
                nx[nodes] = x;
                ny[nodes] = y;
                ++nodes;
            }
        }
        // A gate on the ray through an interior start or goal. The arrival
        // into that circle has to be radial, and an octagon vertex is not
        // on that ray. The gate sits as far out as a corner vertex, so a
        // pivot there still keeps the bumper outside the circle.
        for (int c = 0; c < nc; ++c) {
            auto addGate = [&](double px, double py) {
                if (nodes >= kNodes) return;
                const double dx = px - circ[c].x;
                const double dy = py - circ[c].y;
                const double dist = std::hypot(dx, dy);
                if (!(dist > 1e-4)) return;
                // Just far enough that a pivot here keeps the bumper on the
                // circle. The octagon sits further out; this ray often runs
                // toward another goal, and that extra offset lands inside it.
                const double rad = circ[c].r + lever + 1e-3;
                const double x = circ[c].x + dx / dist * rad;
                const double y = circ[c].y + dy / dist * rad;
                if (config.field_bound && !inField(x, y)) return;
                for (int o = 0; o < nc; ++o) {
                    if (o == c) continue;
                    if (std::hypot(x - circ[o].x, y - circ[o].y) < circ[o].r) return;
                }
                for (int i = 0; i < nodes; ++i) {
                    if (std::hypot(nx[i] - x, ny[i] - y) < 1e-3) return;
                }
                nx[nodes] = x;
                ny[nodes] = y;
                ++nodes;
            };
            if (holds(circ[c], rx, ry)) addGate(rx, ry);
            if (holds(circ[c], jx, jy)) addGate(jx, jy);
        }

        auto nodeAt = [&](double x, double y) {
            for (int i = 0; i < nodes; ++i) {
                if (std::hypot(nx[i] - x, ny[i] - y) < 1e-3) return i;
            }
            return -1;
        };
        // Index in the stamped polyline of a pivot the bumper arc fails.
        // -1 means every pivot stays outside the keep-outs.
        auto blamePivot = [&](const Vertex* pts, int pn) {
            if (pn < 2) return -1;
            const double first_nose = noseOf(pts[0].phi, pts[0].drive);
            if (!pivotArcClear(pts[0].x, pts[0].y, from_m.theta, first_nose, circ, nc))
                return 0;
            for (int i = 1; i <= pn - 2; ++i) {
                if (!pts[i].stop) continue;
                const double n0 = noseOf(pts[i - 1].phi, pts[i - 1].drive);
                const double n1 = noseOf(pts[i].phi, pts[i].drive);
                if (!pivotArcClear(pts[i].x, pts[i].y, n0, n1, circ, nc)) return i;
            }
            if (!target_m.ignore_direction) {
                const bool forwards = config.forwards && !target_m.backwards;
                const double exit_nose = forwards
                    ? target_m.exit_direction
                    : mathutil::normalize_angle(target_m.exit_direction + Robot::PI);
                const double arrival = noseOf(pts[pn - 2].phi, pts[pn - 2].drive);
                if (!pivotArcClear(pts[pn - 1].x, pts[pn - 1].y, arrival, exit_nose, circ, nc))
                    return pn - 1;
            }
            return -1;
        };

        bool banned[kNodes] = {};
        Vertex chosen[kCap];
        int chosen_n = 0;
        for (int attempt = 0; attempt < 16; ++attempt) {
            double dist[kNodes];
            int prev[kNodes];
            bool used[kNodes];
            for (int i = 0; i < nodes; ++i) {
                dist[i] = 1e12;
                prev[i] = -1;
                used[i] = false;
            }
            dist[0] = 0.0;
            for (int iter = 0; iter < nodes; ++iter) {
                int u = -1;
                double best = 1e11;
                for (int i = 0; i < nodes; ++i) {
                    if (banned[i] || used[i] || !(dist[i] < best)) continue;
                    best = dist[i];
                    u = i;
                }
                if (u < 0 || u == 1) break;
                used[u] = true;
                for (int v = 0; v < nodes; ++v) {
                    if (v == u || used[v] || (banned[v] && v != 1)) continue;
                    const bool touch_start = (u == 0 || v == 0);
                    const bool touch_goal = (u == 1 || v == 1);
                    if (!lineClear(nx[u], ny[u], nx[v], ny[v], circ, nc,
                                   touch_start, touch_goal, rx, ry, jx, jy)) continue;
                    if (!fieldOk(nx[u], ny[u], nx[v], ny[v], config.field_bound)) continue;
                    const double w = std::hypot(nx[v] - nx[u], ny[v] - ny[u]);
                    if (w < 1e-4) continue;
                    if (dist[u] + w < dist[v]) {
                        dist[v] = dist[u] + w;
                        prev[v] = u;
                    }
                }
            }

            int order[kNodes];
            int len = 0;
            bool saw_start = false;
            for (int i = 1, guard = 0; i != -1 && guard < nodes; i = prev[i], ++guard) {
                order[len++] = i;
                if (i == 0) saw_start = true;
            }
            if (!saw_start || !(dist[1] < 1e11)) break;

            Vertex trial[kCap];
            int pn = 0;
            for (int i = len - 1; i >= 0 && pn < kCap; --i) {
                trial[pn].x = nx[order[i]];
                trial[pn].y = ny[order[i]];
                ++pn;
            }
            if (have_finish && pn < kCap) {
                trial[pn].x = tx;
                trial[pn].y = ty;
                ++pn;
            }
            if (pn < 2) break;
            pn = dropCollinear(trial, pn);
            stamp(trial, pn, config, target_m, a_plan, pace.cruise, pace.v_end);
            const int blame = blamePivot(trial, pn);
            if (chosen_n == 0 || blame < 0) {
                for (int i = 0; i < pn; ++i) chosen[i] = trial[i];
                chosen_n = pn;
            }
            if (blame < 0) break;
            int ban_at = (blame == 0) ? 1 : blame;
            int ban = -1;
            while (ban_at > 0) {
                ban = nodeAt(trial[ban_at].x, trial[ban_at].y);
                if (ban > 1) break;
                ban = -1;
                --ban_at;
            }
            if (ban < 0 || banned[ban]) break;
            banned[ban] = true;
        }
        if (chosen_n >= 2) {
            for (int i = 0; i < chosen_n; ++i) out[i] = chosen[i];
            return chosen_n;
        }
        push(rx, ry);
        push(jx, jy);
    }
    if (have_finish) push(tx, ty);
    if (n < 2) return n;
    n = dropCollinear(out, n);
    stamp(out, n, config, target_m, a_plan, pace.cruise, pace.v_end);
    return n;
}

int SegmentFollower::samplePath(const Pose2d& from, Pose2d* out, int cap,
                                double v_lin, double v_ang) const {
    (void)v_lin;
    (void)v_ang;
    if (out == nullptr || cap < 2) return 0;
    Vertex local[kCap];
    const Vertex* src = pts_;
    int n = count_;
    const DriftCurveTarget tgt = rotationTarget(targetInMeters(target));
    Pose2d from_m = poseInchesToMeters(from);
    if (tgt.point == CommandPoint::Tracking)
        from_m = tracking::centerFromTracking(from_m);
    if (!planned_) {
        n = build(config, tgt, from_m, local);
        src = local;
    }
    if (n < 2) return 0;
    // Vertices are the rotation centre, which is the point compute() tracks.
    // The field line is this polyline, not the bumper.
    const int count = (n > cap) ? cap : n;
    for (int i = 0; i < count; ++i) {
        const int idx = (n > cap) ? (i * (n - 1)) / (cap - 1) : i;
        out[i].x = metersToInches(src[idx].x);
        out[i].y = metersToInches(src[idx].y);
        out[i].theta = mathutil::normalize_angle(src[idx].phi);
    }
    return count;
}

int SegmentFollower::sampleBase(const Pose2d& from, Pose2d* out, int cap,
                                double v_lin, double v_ang) const {
    return samplePath(from, out, cap, v_lin, v_ang);
}

ChassisCommand SegmentFollower::compute(const StateEstimate& state) {
    const DriftCurveTarget tgt = rotationTarget(targetInMeters(target));
    // Localization publishes the tracking centre. The plan is the yaw centre.
    const Pose2d pose = tracking::centerFromTracking(poseInchesToMeters(state.pose));
    const double theta = pose.theta;
    if (!planned_) {
        const Pace pace = paceOf(config, tgt);
        accel_ = profileAccel(pace.accel, config.min_accel);
        cruise_ = pace.cruise;
        count_ = build(config, tgt, pose, pts_);
        planned_ = true;
        seg_ = 0;
        started_ = false;
        held_turn_ = false;
        corner_hold_ = false;
        spin_latched_ = false;
        yaw_rate_ = 0.0;
    }
    if (count_ < 2) {
        goal_heading_ = theta;
        settled_ = true;
        stuck_watch_.reset();
        seconds_left_ = 0.0;
        ChassisCommand done;
        done.request = DriveRequest::Motion;
        return done;
    }
    if (seg_ < 0) seg_ = 0;
    if (seg_ > count_ - 2) seg_ = count_ - 2;

    bool hold_speed = false;
    bool yaw_only = false;
    bool corner_yaw = false;
    double use_phi = pts_[seg_].phi;
    double use_drive = pts_[seg_].drive;
    double use_lat = 0.0;
    double use_t = 0.0;

    for (int pass = 0; pass < count_; ++pass) {
        double t = 0.0;
        double lat = 0.0;
        double remain = 0.0;
        project(pts_, seg_, pose.x, pose.y, t, lat, remain);
        use_t = t;
        use_lat = lat;
        use_phi = pts_[seg_].phi;
        use_drive = pts_[seg_].drive;

        if (!started_) {
            const double err = std::fabs(mathutil::normalize_angle(
                noseOf(use_phi, use_drive) - theta));
            if (err > config.pivot_done_rad) held_turn_ = true;
            const bool spinning = std::fabs(state.v_ang) > kPivotQuiet;
            const bool moving = std::fabs(state.v_lin) > config.settle_speed;
            // A departure that changes the nose is a turn: stop, yaw, then leave.
            if (held_turn_ && (err > config.pivot_done_rad || moving || spinning)) {
                hold_speed = true;
                yaw_only = true;
                use_lat = 0.0;
                break;
            }
            started_ = true;
        }

        const bool upcoming = seg_ + 1 <= count_ - 2;
        const double nx = upcoming ? pts_[seg_ + 1].x : 0.0;
        const double ny = upcoming ? pts_[seg_ + 1].y : 0.0;
        const double n2x = upcoming ? pts_[seg_ + 2].x : nx;
        const double n2y = upcoming ? pts_[seg_ + 2].y : ny;
        // On the next line, the pivot can start while the last of the
        // speed is still bleeding off. Off that line, the incoming segment
        // is done once the centre has stopped at the vertex. The slide
        // walks onto the line during the yaw.
        const bool on_tight = upcoming && onNextLine(nx, ny, n2x, n2y, pose.x, pose.y);
        if (upcoming && pts_[seg_ + 1].stop) {
            const double away = std::hypot(pose.x - nx, pose.y - ny);
            const double seg_len = pts_[seg_ + 1].s - pts_[seg_].s;
            const double traveled = seg_len - remain;
            const bool stopped = std::fabs(state.v_lin) <= kStopMps;
            const bool at_vertex = stopped && away <= kCornerNearM
                && remain <= kVertexArriveM && traveled > kArriveM;
            if (away > kCornerNearM) corner_hold_ = false;
            else if (on_tight || at_vertex) corner_hold_ = true;
        }
        if (upcoming && pts_[seg_ + 1].stop && corner_hold_) {
            const double dep_phi = pts_[seg_ + 1].phi;
            const double dep_drive = pts_[seg_ + 1].drive;
            const double err = std::fabs(mathutil::normalize_angle(
                noseOf(dep_phi, dep_drive) - theta));
            const bool spinning = std::fabs(state.v_ang) > kPivotQuiet;
            const bool moving = std::fabs(state.v_lin) > kStopMps;
            // The slide runs while the nose is still outside the release
            // window. Inside that window the brake curve is below the rate
            // a stopped chassis will start, so waiting there never finishes
            // the line. The next segment starts instead.
            if (err > config.pivot_done_rad || moving || spinning) {
                hold_speed = true;
                // Stay on the pivot armature until the nose is actually
                // there. Dropping to a trim at the release window still
                // commands about 1 rad/s and then cuts it off.
                yaw_only = true;
                corner_yaw = true;
                use_phi = dep_phi;
                use_drive = dep_drive;
                use_lat = 0.0;
                break;
            }
            corner_hold_ = false;
            ++seg_;
            continue;
        }

        if (t >= kPassT && seg_ < count_ - 2 && !pts_[seg_ + 1].stop) {
            ++seg_;
            continue;
        }
        break;
    }

    // The advance loop may have stepped seg_ on its last pass. Project again
    // so the foot, the cross-track, and the segment heading are one sample.
    // A pivot keeps the departure heading it already chose.
    if (!hold_speed) {
        double t = 0.0;
        double lat = 0.0;
        double remain = 0.0;
        project(pts_, seg_, pose.x, pose.y, t, lat, remain);
        use_t = t;
        use_lat = lat;
        use_phi = pts_[seg_].phi;
        use_drive = pts_[seg_].drive;
        (void)remain;

        const double face = std::fabs(mathutil::normalize_angle(
            noseOf(use_phi, use_drive) - theta));
        if (face > config.pivot_rad) {
            hold_speed = true;
            yaw_only = true;
            use_lat = 0.0;
        }
    }

    // A stop finishes on the exit heading, not on the last segment's nose.
    // Once the centre is on the end point, yaw there in place. Arriving
    // already on that heading releases a departure hold so the leg can end.
    const bool stop_leg = !tgt.ignore_direction && std::fabs(tgt.exit_speed) <= kStopMps;
    if (stop_leg && seg_ >= count_ - 2) {
        const bool nose_forward = config.forwards && !tgt.backwards;
        const double exit_nose = nose_forward
            ? tgt.exit_direction
            : mathutil::normalize_angle(tgt.exit_direction + Robot::PI);
        double t_end = 0.0;
        double lat_end = 0.0;
        double remain_end = 0.0;
        project(pts_, seg_, pose.x, pose.y, t_end, lat_end, remain_end);
        (void)lat_end;
        (void)remain_end;
        const double face = std::fabs(mathutil::normalize_angle(exit_nose - theta));
        const bool spinning = std::fabs(state.v_ang) > kPivotQuiet;
        // Beside the endpoint the along-track distance is already zero.
        // Arrival is the centre in the ball, so a miss does not count as done.
        const bool on_end = std::hypot(pose.x - pts_[count_ - 1].x,
                                       pose.y - pts_[count_ - 1].y) <= kArriveM + 1e-3;
        if (on_end && (face > config.pivot_done_rad || spinning)) {
            hold_speed = true;
            yaw_only = true;
            use_lat = 0.0;
            use_phi = exit_nose;
            use_drive = 1.0;
            use_t = t_end;
        } else if (on_end) {
            hold_speed = false;
            yaw_only = false;
            use_lat = 0.0;
            use_phi = exit_nose;
            use_drive = 1.0;
            use_t = t_end;
        }
    }

    const double speed = std::fabs(state.v_lin);
    double corr = 0.0;
    if (!yaw_only) {
        const double denom = config.stanley_soft + speed;
        if (denom > 1e-6) corr = std::atan2(config.stanley_k * use_lat, denom);
    }
    const double psi = noseOf(use_phi - corr, use_drive);
    goal_heading_ = psi;
    a_actual_lat_ = 0.0;

    const double heading_err = mathutil::normalize_angle(psi - theta);
    const double omega_now = std::isfinite(state.v_ang) ? state.v_ang : 0.0;
    // A pivot waits until forward speed is gone. Yawing while the brake is
    // still taking off half a metre per second is the jerk at the corner.
    bool pivoting = false;
    if (yaw_only && std::fabs(state.v_lin) > kStopMps
        && std::fabs(omega_now) < kPivotRate && !spin_latched_) {
        pivoting = false;
    } else if (yaw_only) {
        pivoting = true;
        spin_latched_ = true;
    } else {
        spin_latched_ = false;
    }
    const double yaw_aim = (yaw_only && !pivoting) ? 0.0 : heading_err;
    yaw_rate_ = slewYawRate(yaw_rate_, yaw_aim, pivoting, omega_now);

    double ds = (pts_[seg_ + 1].s - pts_[seg_].s) * (1.0 - use_t);
    if (ds < 0.0) ds = 0.0;
    // Along-track remaining is already the brake distance. A projection
    // that has reached the vertex while the centre is still just outside
    // the strip would command a stop. Keep at most the along tolerance of
    // travel, so the centre enters the strip and the pivot starts. A miss
    // far from the line does not hold cruise speed.
    if (!hold_speed && seg_ + 1 <= count_ - 2 && pts_[seg_ + 1].stop) {
        const double vx = pts_[seg_ + 1].x - pose.x;
        const double vy = pts_[seg_ + 1].y - pose.y;
        const double toward = vx * mathutil::heading_cos(pts_[seg_].phi)
                            + vy * mathutil::heading_sin(pts_[seg_].phi);
        // Past the vertex, more speed along this segment drives away from it.
        if (toward > 0.0) {
            double gap = lineStripGap(pts_[seg_ + 1].x, pts_[seg_ + 1].y,
                                      pts_[seg_ + 2].x, pts_[seg_ + 2].y,
                                      pose.x, pose.y);
            if (gap > kAlongM) gap = kAlongM;
            if (gap > ds) ds = gap;
        }
    }
    // How far the end point still lies along travel. Negative once the
    // centre has passed it. Growing that distance back into the brake
    // curve speeds the chassis up as it drives away.
    const double end_ahead = (pts_[count_ - 1].x - pose.x) * mathutil::heading_cos(pts_[seg_].phi)
                           + (pts_[count_ - 1].y - pose.y) * mathutil::heading_sin(pts_[seg_].phi);
    if (!hold_speed && stop_leg && seg_ >= count_ - 2 && end_ahead > 0.0) {
        const double gap = std::hypot(pose.x - pts_[count_ - 1].x,
                                      pose.y - pts_[count_ - 1].y);
        const double close = gap - kArriveM;
        if (gap > kArriveM && close > ds) ds = close;
    }
    // Current travel over the lead ticks. Measured speed, so a chassis that
    // is already faster than the command gives the distance up sooner.
    const double lead = std::fabs(state.v_lin) * kBrakeLeadTicks * System::DT_SEC;
    double ds_brake = ds - lead;
    if (ds_brake < 0.0) ds_brake = 0.0;
    // The lead can eat the last centimetre and leave a stop command while
    // the centre is still outside the strip. The distance left is the gap
    // into it, so the segment does not sit short of the pivot.
    if (!hold_speed && ds_brake < 1e-4) {
        double guard = 0.0;
        if (seg_ + 1 <= count_ - 2 && pts_[seg_ + 1].stop) {
            const double vx = pts_[seg_ + 1].x - pose.x;
            const double vy = pts_[seg_ + 1].y - pose.y;
            const double toward = vx * mathutil::heading_cos(pts_[seg_].phi)
                                + vy * mathutil::heading_sin(pts_[seg_].phi);
            if (toward > 0.0) {
                guard = lineStripGap(pts_[seg_ + 1].x, pts_[seg_ + 1].y,
                                     pts_[seg_ + 2].x, pts_[seg_ + 2].y,
                                     pose.x, pose.y);
                if (guard > kAlongM) guard = kAlongM;
            }
        }
        if (stop_leg && seg_ >= count_ - 2 && end_ahead > 0.0) {
            const double gap = std::hypot(pose.x - pts_[count_ - 1].x,
                                          pose.y - pts_[count_ - 1].y);
            const double into = gap - kArriveM;
            if (gap > kArriveM && into > guard) guard = into;
        }
        if (guard > ds_brake) ds_brake = guard;
    }
    const double v_next = pts_[seg_ + 1].v;
    double v_un = std::sqrt(v_next * v_next + 2.0 * accel_ * ds_brake);
    if (v_un > cruise_) v_un = cruise_;
    if (hold_speed) v_un = 0.0;
    // The corner pivot is not a pure spin. The body speed is the
    // wheel-speed ratio that puts the centre on the next line as the nose
    // arrives. Along the line, wherever the centre already sits is close
    // enough. v_un is along travel; body forward is use_drive times that.
    if (corner_yaw && pivoting && seg_ + 2 < count_) {
        const double body = pivotSlideMps(
            pose.x, pose.y, theta,
            pts_[seg_ + 1].x, pts_[seg_ + 1].y, pts_[seg_ + 2].x, pts_[seg_ + 2].y,
            use_phi, noseOf(use_phi, use_drive), yaw_rate_, use_drive);
        v_un = body * use_drive;
    }
    if (!std::isfinite(v_un)) v_un = 0.0;
    if (!corner_yaw && v_un < 0.0) v_un = 0.0;
    // The last segment is not a corner stop. When the distance left cannot
    // shed the measured speed at full armature accel, the command is a stop
    // so the chassis is done on the point.
    double a_full = Feedforward::LIN_ACCEL_AT_FULL_V;
    if (!(a_full > accel_)) a_full = accel_;
    const double v_abs = std::fabs(state.v_lin);
    double stop_at = (a_full > 1e-6) ? (v_abs * v_abs) / (2.0 * a_full) : 0.0;
    stop_at += v_abs * kBrakeLeadTicks * System::DT_SEC;
    if (seg_ >= count_ - 2 && !hold_speed && ds <= stop_at) {
        const double end_gap = std::hypot(pose.x - pts_[count_ - 1].x,
                                          pose.y - pts_[count_ - 1].y);
        // Clearly past the end, more speed in this direction drives away.
        // Still short of the ball, the command has to keep closing: a zero
        // here is a stop outside the arrival, and the leg never settles.
        const bool closing = stop_leg && end_gap > kArriveM && end_ahead > -kArriveM;
        if (!closing) v_un = 0.0;
    }
    if (stop_leg && seg_ >= count_ - 2 && !hold_speed && !corner_yaw) {
        const double end_gap = std::hypot(pose.x - pts_[count_ - 1].x,
                                          pose.y - pts_[count_ - 1].y);
        if (end_ahead <= -kArriveM) {
            v_un = 0.0;
        } else if (end_ahead > 0.0 && end_gap > kArriveM
                   && std::fabs(state.v_lin) <= kStopMps) {
            const double close = end_gap - kArriveM;
            double v_close = std::sqrt(2.0 * accel_ * close);
            // Slower than this, a stopped chassis does not move, and the
            // last centimetre never gets inside the ball.
            if (v_close < kStopMps) v_close = kStopMps;
            if (v_close > cruise_) v_close = cruise_;
            if (v_un < v_close) v_un = v_close;
        }
    }

    const double dt = System::DT_SEC;
    if (!v_cmd_primed_) {
        prev_v_cmd_ = state.v_lin;
        v_cmd_primed_ = true;
    }
    // A hold means stop. If the chassis is already stopped, the previous
    // command is stale: slewing from a cruise command drives off the
    // corner before the nose turns. A slide that is already under its cap
    // is kept, so it can ramp instead of being cleared every tick.
    if (hold_speed && std::fabs(state.v_lin) <= kStopMps) {
        const bool slide = corner_yaw && pivoting && std::fabs(prev_v_cmd_) <= kSlideMax;
        if (!slide) prev_v_cmd_ = 0.0;
    }
    const double prev_along = use_drive * prev_v_cmd_;
    double travel = use_drive * state.v_lin;
    if (!std::isfinite(travel)) travel = 0.0;

    // Speeding up stays on the profile accel, which left overhead unused.
    // Slowing down may use the full armature accel. The path accel is not
    // a cap on that correction.
    double a_lim = Feedforward::LIN_ACCEL_AT_FULL_V;
    if (!(a_lim > accel_)) a_lim = accel_;
    double a_along = (dt > 1e-6) ? (v_un - prev_along) / dt : 0.0;
    if (a_along > accel_) a_along = accel_;
    if (a_along < -a_lim) a_along = -a_lim;
    // The slide is a few centimetres per second. Chasing it at the path
    // accel spends the static term and a full kA step on a nudge.
    if (corner_yaw && pivoting) {
        if (a_along > 0.8) a_along = 0.8;
        if (a_along < -0.8) a_along = -0.8;
    }

    double along = prev_along + a_along * dt;
    // A closing reverse on the last centimetre is the only backward command
    // outside a corner slide. The floor keeps it, and still stops a runaway.
    const double along_floor = corner_yaw ? -kSlideMax : 0.0;
    if (along < along_floor) along = along_floor;
    if (dt > 1e-6) a_along = (along - prev_along) / dt;

    const double over = travel - v_un;
    if (over > 0.0) {
        a_along -= over / kSpeedCatchSec;
        if (a_along < -a_lim) a_along = -a_lim;
    }

    const double v_cmd = use_drive * along;
    const double a_cmd = use_drive * a_along;
    prev_v_cmd_ = v_cmd;

    // Stopped, not being asked to drive, and still off the heading. The
    // 1 cm ball is what usually starts this pivot. Short of it, the same
    // error is a trim, and the trim drops kS. That voltage sits under
    // breakaway, the nose never moves, and a score that waits on settled()
    // never ends. Square up in place. If the chassis still does not yaw,
    // the stuck watch below gives the leg up.
    if (!pivoting
        && std::fabs(state.v_lin) <= kStopMps
        && std::fabs(v_cmd) <= kStopMps
        && std::fabs(heading_err) > config.pivot_done_rad) {
        pivoting = true;
        yaw_rate_ = slewYawRate(yaw_rate_, heading_err, true, omega_now);
    }

    const YawArmature yaw_plan = yawArmatureFromRate(yaw_rate_, omega_now,
                                                     Turn::MAX_ACCEL_RAD_S2, pivoting);

    const bool tracking_last = seg_ >= count_ - 2 && !hold_speed;
    const double end_gap = std::hypot(pose.x - pts_[count_ - 1].x,
                                      pose.y - pts_[count_ - 1].y);
    // A millimetre past the 1 cm ball is still the stop. A chassis that has
    // already stopped within the along tolerance is done too: commanding
    // nothing there never enters the tighter ball.
    const bool stopped_near = std::fabs(state.v_lin) <= kStopMps && end_gap <= kAlongM;
    const bool at_end = tracking_last && (end_gap <= kArriveM + 1e-3 || stopped_near);
    const bool measured_stop = std::fabs(state.v_lin) <= kStopMps;
    const bool nose_forward = config.forwards && !tgt.backwards;
    const double exit_nose = nose_forward
        ? tgt.exit_direction
        : mathutil::normalize_angle(tgt.exit_direction + Robot::PI);
    const bool heading_ok = !stop_leg
        || std::fabs(mathutil::normalize_angle(exit_nose - theta)) <= config.pivot_done_rad;
    const bool spin_ok = !stop_leg || std::fabs(state.v_ang) <= kPivotRate;
    settled_ = at_end && measured_stop && heading_ok && spin_ok;
    if (settled_) {
        stuck_watch_.reset();
    } else {
        // Stopped short of the end, and the command has already given up
        // on driving there. That is not a hold for the lift: the leg is
        // still open, and leaving it alone is how a score sits until the
        // routine ends. A real drive slews past kStopMps, so the start of
        // a leg does not count.
        const bool parked_short = std::fabs(state.v_lin) <= kStopMps
            && std::fabs(v_cmd) <= kStopMps
            && !pivoting
            && end_gap > kAlongM;
        const bool want_lin = std::fabs(v_cmd) > config.settle_speed || parked_short;
        // The bleed before a pivot is a hold. It is not an unanswered yaw.
        const bool want_yaw = pivoting
            && std::fabs(heading_err) > config.pivot_done_rad;
        stuck_watch_.update(want_lin, want_yaw, state.v_lin, state.v_ang);
    }

    const double v_now = hold_speed ? 0.0 : std::fabs(state.v_lin);
    const double yaw_err = std::fabs(mathutil::normalize_angle(goal_heading_ - theta));
    seconds_left_ = settled_ ? 0.0
        : etaSeconds(seg_, use_t, v_now, yaw_only, corner_yaw, yaw_err);

    ChassisCommand cmd;
    cmd.request = DriveRequest::Motion;
    cmd.linear = std::isfinite(v_cmd) ? v_cmd : 0.0;
    cmd.accel = std::isfinite(a_cmd) ? a_cmd : 0.0;
    cmd.angular = std::isfinite(yaw_plan.omega) ? yaw_plan.omega : 0.0;
    cmd.alpha = std::isfinite(yaw_plan.alpha) ? yaw_plan.alpha : 0.0;
    cmd.pivot = pivoting;
    cmd.lateral = 0.0;
    cmd.brake = false;
    return cmd;
}
