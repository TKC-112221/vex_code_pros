/* ---------------------------------------------------------------------------
 * SegmentFollower.cpp  --  plan a polyline once, then track it.
 * ------------------------------------------------------------------------- */
#include "control/SegmentFollower.h"
#include "math/MathUtils.h"

#include <cmath>

namespace {

constexpr int kObstacles = kPathObstacles;
constexpr int kSides = 8;
constexpr int kNodes = 2 + kObstacles * kSides;
constexpr double kArriveM = 0.04;
constexpr double kPassT = 0.999;
constexpr double kPivotRate = 1.0;
constexpr double kEndWindowM = 0.15;
constexpr double kDepartIn = 0.25;

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

double noseOf(double travel, double drive) {
    if (drive < 0.0) travel += Robot::PI;
    return mathutil::normalize_angle(travel);
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

bool lineClear(double ax, double ay, double bx, double by,
               const Circle* circ, int nc) {
    for (int i = 0; i < nc; ++i) {
        if (segmentGap(ax, ay, bx, by, circ[i].x, circ[i].y) < circ[i].r - 1e-4) return false;
    }
    return true;
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
        pts[i].phi = std::atan2(pts[i + 1].y - pts[i].y, pts[i + 1].x - pts[i].x);
        pts[i].stop = false;
    }
    pts[n - 1].phi = pts[n - 2].phi;
    pts[n - 1].stop = false;

    const int nseg = n - 1;
    pts[nseg - 1].drive = target.backwards ? -1.0 : 1.0;
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
        pts[i].stop = flip || turn >= config.pivot_rad;
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
        if (pts[i].stop) {
            cap = 0.0;
        } else if (i > 0) {
            const double nose0 = noseOf(pts[i - 1].phi, pts[i - 1].drive);
            const double nose1 = noseOf(pts[i].phi, pts[i].drive);
            const double turn = std::fabs(mathutil::normalize_angle(nose1 - nose0));
            if (turn > 0.05 && config.corner_yaw > 1e-3 && config.corner_len_m > 1e-4) {
                const double c = config.corner_yaw * config.corner_len_m / turn;
                if (c < cap) cap = c;
            }
        }
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
    yaw_only_ = false;
    accel_ = 0.0;
    cruise_ = 0.0;
    i_ang_ = 0.0;
    prev_ang_ = 0.0;
    head_primed_ = false;
    prev_v_cmd_ = 0.0;
    v_cmd_primed_ = false;
    goal_heading_ = 0.0;
    settled_ = false;
    dist_armed_ = false;
    saw_motion_ = false;
    start_dist_m_ = 0.0;
    best_dist_m_ = 0.0;
}

int SegmentFollower::build(const SegmentFollowerConfig& config, const DriftCurveTarget& target_m,
                           const Pose2d& from_m, Vertex* out) {
    if (out == nullptr) return 0;
    const Pace pace = paceOf(config, target_m);
    const double tx = target_m.x;
    const double ty = target_m.y;
    const double approach = target_m.exit_direction;
    double jx = tx;
    double jy = ty;
    bool have_finish = false;
    if (!target_m.ignore_direction && target_m.line_length > 1e-4) {
        jx = tx - target_m.line_length * std::cos(approach);
        jy = ty - target_m.line_length * std::sin(approach);
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
        c.r = inchesToMeters(config.obstacles[i].min_dist);
        if (!(c.r > 1e-3) || !std::isfinite(c.x) || !std::isfinite(c.y)) continue;
        ++nc;
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
        const double half = Robot::PI / static_cast<double>(kSides);
        const double outward = 1.0 / std::cos(half);
        for (int c = 0; c < nc; ++c) {
            for (int k = 0; k < kSides; ++k) {
                if (nodes >= kNodes) break;
                const double ang = (2.0 * Robot::PI * static_cast<double>(k))
                                 / static_cast<double>(kSides);
                const double rad = circ[c].r * outward;
                const double x = circ[c].x + rad * std::cos(ang);
                const double y = circ[c].y + rad * std::sin(ang);
                if (config.field_bound && !inField(x, y)) continue;
                bool buried = false;
                for (int o = 0; o < nc; ++o) {
                    if (o == c) continue;
                    if (std::hypot(x - circ[o].x, y - circ[o].y) < circ[o].r) buried = true;
                }
                if (buried) continue;
                nx[nodes] = x;
                ny[nodes] = y;
                ++nodes;
            }
        }

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
                if (used[i] || !(dist[i] < best)) continue;
                best = dist[i];
                u = i;
            }
            if (u < 0 || u == 1) break;
            used[u] = true;
            for (int v = 0; v < nodes; ++v) {
                if (v == u || used[v]) continue;
                if (!lineClear(nx[u], ny[u], nx[v], ny[v], circ, nc)) continue;
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
        if (!saw_start || !(dist[1] < 1e11)) {
            push(rx, ry);
            push(jx, jy);
        } else {
            for (int i = len - 1; i >= 0; --i) push(nx[order[i]], ny[order[i]]);
        }
    }
    if (have_finish) push(tx, ty);
    if (n < 2) return n;
    n = dropCollinear(out, n);
    stamp(out, n, config, target_m, pace.accel, pace.cruise, pace.v_end);
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
    if (!planned_) {
        n = build(config, targetInMeters(target), poseInchesToMeters(from), local);
        src = local;
    }
    if (n < 2) return 0;
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

ChassisCommand SegmentFollower::compute(const StateEstimate& state) {
    const DriftCurveTarget tgt = targetInMeters(target);
    const Pose2d pose = poseInchesToMeters(state.pose);
    const double theta = pose.theta;
    if (!planned_) {
        const Pace pace = paceOf(config, tgt);
        accel_ = pace.accel;
        cruise_ = pace.cruise;
        count_ = build(config, tgt, pose, pts_);
        planned_ = true;
        seg_ = 0;
        started_ = false;
        yaw_only_ = false;
    }
    if (count_ < 2) {
        goal_heading_ = theta;
        settled_ = true;
        ChassisCommand done;
        return done;
    }
    if (seg_ < 0) seg_ = 0;
    if (seg_ > count_ - 2) seg_ = count_ - 2;

    bool hold_speed = false;
    bool yaw_only = false;
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
            if (err > config.pivot_rad) {
                hold_speed = true;
                yaw_only = true;
                use_lat = 0.0;
                break;
            }
            started_ = true;
        }

        const bool upcoming = seg_ + 1 <= count_ - 2;
        if (upcoming && pts_[seg_ + 1].stop && (t >= kPassT || remain < kArriveM)) {
            const double dep_phi = pts_[seg_ + 1].phi;
            const double dep_drive = pts_[seg_ + 1].drive;
            const double err = std::fabs(mathutil::normalize_angle(
                noseOf(dep_phi, dep_drive) - theta));
            const bool spinning = std::fabs(state.v_ang) > kPivotRate;
            const bool moving = std::fabs(state.v_lin) > config.settle_speed;
            if (err > config.pivot_done_rad || moving || spinning) {
                hold_speed = true;
                yaw_only = err > config.pivot_rad;
                use_phi = dep_phi;
                use_drive = dep_drive;
                use_lat = yaw_only ? 0.0 : lat;
                break;
            }
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
        const double face = std::fabs(mathutil::normalize_angle(
            noseOf(use_phi, use_drive) - theta));
        if (face > config.pivot_rad) {
            hold_speed = true;
            yaw_only = true;
            use_lat = 0.0;
        }
        (void)remain;
    }

    const double speed = std::fabs(state.v_lin);
    double corr = 0.0;
    if (!yaw_only) {
        const double denom = config.stanley_soft + speed;
        if (denom > 1e-6) corr = std::atan2(config.stanley_k * use_lat, denom);
    }
    const double psi = noseOf(use_phi - corr, use_drive);
    goal_heading_ = psi;

    if (yaw_only != yaw_only_) {
        i_ang_ = 0.0;
        head_primed_ = false;
        yaw_only_ = yaw_only;
    }

    double ds = (pts_[seg_ + 1].s - pts_[seg_].s) * (1.0 - use_t);
    if (ds < 0.0) ds = 0.0;
    const double v_next = pts_[seg_ + 1].v;
    double v_un = std::sqrt(v_next * v_next + 2.0 * accel_ * ds);
    if (v_un > cruise_) v_un = cruise_;
    if (hold_speed) {
        v_un = 0.0;
    } else {
        const double face = std::fabs(mathutil::normalize_angle(
            noseOf(use_phi, use_drive) - theta));
        if (face > 0.05 && config.corner_yaw > 1e-3 && config.corner_len_m > 1e-4) {
            const double cap = config.corner_yaw * config.corner_len_m / face;
            if (cap < v_un) v_un = cap;
        }
    }
    if (!std::isfinite(v_un) || v_un < 0.0) v_un = 0.0;

    const double step = accel_ * System::DT_SEC;
    const double v_des = use_drive * v_un;
    if (!v_cmd_primed_) {
        prev_v_cmd_ = state.v_lin;
        v_cmd_primed_ = true;
    }
    double v_cmd = v_des;
    if (v_cmd > prev_v_cmd_ + step) v_cmd = prev_v_cmd_ + step;
    if (v_cmd < prev_v_cmd_ - step) v_cmd = prev_v_cmd_ - step;
    const double a_cmd = (System::DT_SEC > 1e-6) ? (v_cmd - prev_v_cmd_) / System::DT_SEC : 0.0;
    prev_v_cmd_ = v_cmd;

    double surge = 0.0;
    if (model_ != nullptr) {
        surge = model_->linearVolts(v_cmd, a_cmd, std::fabs(v_cmd), std::fabs(a_cmd),
                                    0.0, Physics::TURN_RADIUS_MAX_M);
    }

    const double heading_deg = mathutil::normalize_angle(psi - theta) * (180.0 / Robot::PI);
    double yaw = pidTick(heading_deg,
                         config.head_kP, config.head_kI,
                         config.head_kD, config.head_windup,
                         i_ang_, prev_ang_, head_primed_) * voltsPerPower();
    head_primed_ = true;

    if (!std::isfinite(surge)) surge = 0.0;
    if (!std::isfinite(yaw)) yaw = 0.0;
    const double max_v = config.max_volts;
    surge = mathutil::clamp_val(surge, -max_v, max_v);
    yaw = mathutil::clamp_val(yaw, -max_v, max_v);
    // Yaw keeps the rail. A slide is caught by pointing the wheels, and the
    // leftover voltage is what remains for surge.
    const double overturn = std::fabs(yaw) + std::fabs(surge) - max_v;
    if (overturn > 0.0) surge -= std::copysign(overturn, surge);

    const double dist = std::hypot(pose.x - tgt.x, pose.y - tgt.y);
    const double depart_m = kDepartIn * kMetersPerInch;
    if (!dist_armed_) {
        start_dist_m_ = dist;
        best_dist_m_ = dist;
        dist_armed_ = true;
    } else if (dist < best_dist_m_) {
        best_dist_m_ = dist;
    }
    if (std::fabs(state.v_lin) > config.settle_speed) saw_motion_ = true;

    double s_foot = pts_[seg_].s + (pts_[seg_ + 1].s - pts_[seg_].s) * use_t;
    double remaining = pts_[count_ - 1].s - s_foot;
    if (remaining < 0.0) remaining = 0.0;
    const bool tracking_last = seg_ >= count_ - 2 && !hold_speed;
    const bool near_end = tracking_last && remaining < kEndWindowM;
    const bool measured_stop = std::fabs(state.v_lin) <= config.settle_speed;
    const bool profile_stop = v_un <= config.settle_speed;
    const bool stopped = near_end && measured_stop && profile_stop
                      && (saw_motion_ || remaining < depart_m);
    const bool closed_in = best_dist_m_ < start_dist_m_ - depart_m;
    const bool departing = near_end && closed_in && dist > best_dist_m_ + depart_m;
    settled_ = stopped || departing;

    ChassisCommand cmd;
    cmd.linear = surge;
    cmd.lateral = 0.0;
    cmd.angular = yaw;
    cmd.brake = false;
    return cmd;
}
