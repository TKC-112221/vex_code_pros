#include "logic/GoalScore.h"
#include "logic/CascadeLift.h"
#include "logic/GoalAligner.h"
#include "field/FieldPoints.h"
#include "math/MathUtils.h"
#include "math/TrackingCenter.h"
#include "interfaces/IFeedForward.h"
#include "interfaces/IMacroController.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include <algorithm>
#include <cmath>

namespace {

GoalScoreOutput driverOnly(const SubsystemCommand& driver) {
    GoalScoreOutput out;
    out.command = driver;
    return out;
}

// `bias_heading` halves the distance of a goal within the nose cone.
int nearestGoal(const Pose2d& from, double heading, bool bias_heading) {
    int best = -1;
    double best_d = 0.0;
    const double cone = GoalScoreTune::kHeadingConeDeg * Robot::PI / 180.0;
    const int n = static_cast<int>(Field::Point::Count);
    for (int i = 0; i < n; ++i) {
        const Field::Spot& spot = Field::kSpots[i];
        if (!Field::isAlignGoal(spot)) continue;
        double d = std::hypot(from.x - spot.x, from.y - spot.y);
        if (bias_heading && d > 1e-3) {
            const double bearing = mathutil::direction_heading(spot.x - from.x, spot.y - from.y);
            const double err = std::fabs(mathutil::normalize_angle(bearing - heading));
            if (err <= cone) d *= GoalScoreTune::kHeadingDistScale;
        }
        if (best < 0 || d < best_d) {
            best = i;
            best_d = d;
        }
    }
    return best;
}

// A return closer than the sense window is the goal in front of the lift.
bool pinSeen(const DistanceReading& front) {
    if (!front.valid || !std::isfinite(front.range_m)) return false;
    const double inches = metersToInches(front.range_m);
    return inches > 0.0 && inches < GoalScoreTune::kSenseIn;
}

// Front bumper `gap_in` short of the goal, nose along `heading` toward it.
// The follower tracks the bumper, so this is the pose it holds.
Pose2d alongApproach(const Field::Spot& goal, double heading, double gap_in) {
    const double c = mathutil::heading_cos(heading);
    const double s = mathutil::heading_sin(heading);
    return Pose2d{goal.x - gap_in * c, goal.y - gap_in * s, heading};
}

// Bumper `gap_in` short of the goal, nose toward it, on the straight line
// from the rotation centre through the goal. The follower tracks that
// centre. A stop on the bumper's line leaves the centre off to the side
// after a turn, and the path into it is not aimed at the goal.
// The centre sits half a body behind the bumper, so it can already be past
// the goal while the bumper is still short. That stop stays on the bumper's
// side, or the bumper is driven through the goal.
Pose2d behindGoal(const Pose2d& bumper, const Pose2d& center, const StateEstimate& state,
                  const Field::Spot& goal, double gap_in) {
    double dx = center.x - goal.x;
    double dy = center.y - goal.y;
    double dist = std::hypot(dx, dy);
    const double bx = bumper.x - goal.x;
    const double by = bumper.y - goal.y;
    const double bdist = std::hypot(bx, by);
    const bool crossed = dist > 1e-3 && bdist > 1e-3 && (dx * bx + dy * by) < 0.0;
    if (crossed || dist < 1e-3) {
        dx = bx;
        dy = by;
        dist = bdist;
    }
    if (dist < 1e-3) {
        dx = -mathutil::heading_cos(state.pose.theta);
        dy = -mathutil::heading_sin(state.pose.theta);
        dist = 1.0;
    }
    const double scale = gap_in / dist;
    return Pose2d{goal.x + dx * scale, goal.y + dy * scale,
                  mathutil::direction_heading(-dx, -dy)};
}

// Seconds for the height loop to cover `inches`. The carriage rides the
// brake envelope, so the time is that curve. The scale and lead start the
// pre-raise early enough that it is done before the standoff.
double liftSeconds(double inches) {
    const double d = std::fabs(inches);
    if (d <= Tuning::CASCADE_HOLD_IN) return 0.0;
    const double a = Tuning::CASCADE_BRAKE_IN_S2;
    if (!(a > 1e-3)) return 0.0;
    return std::sqrt(2.0 * d / a) * GoalScoreTune::kLiftTimeScale
        + GoalScoreTune::kLiftLeadSec;
}

// The bumper is on the standoff pose. True during the final in-place yaw,
// which has not settled yet.
bool atMark(const SegmentFollower& nav, const Pose2d& pose) {
    return std::hypot(pose.x - nav.target.x, pose.y - nav.target.y) <= GoalScoreTune::kMarkIn;
}

bool atBottom(const CascadeReading& sense) {
    const double rpm = std::isfinite(sense.velocity_rpm) ? sense.velocity_rpm : 0.0;
    return cascadeLift().heightIn(sense) <= Tuning::CASCADE_HOLD_IN
        && std::fabs(rpm) <= Tuning::CASCADE_HOLD_RPM;
}

void holdHeight(SubsystemCommand& cmd, double inches) {
    cmd.cascade_pct = 0;
    cmd.cascade_height = true;
    cmd.cascade_target_in = inches;
}

// Press the bumper into the latched goal and yaw onto `heading`. Surge stays
// the larger of the two so the inside wheel does not back the bumper off.
ChassisCommand scoreOrbit(double heading, const StateEstimate& state) {
    const double error = mathutil::normalize_angle(heading - state.pose.theta);
    const double w_now = std::isfinite(state.v_ang) ? state.v_ang : 0.0;
    const double accel = GoalAlign::TURN_YAW_ACCEL;
    const double w_cap = GoalAlign::TURN_YAW_RAD_S;
    const double w_stop = std::sqrt(2.0 * accel * std::fabs(error));
    const double w_mag = (w_stop < w_cap) ? w_stop : w_cap;
    const double w_ideal = mathutil::sign(error) * w_mag;
    const bool in_window = std::fabs(error) <= GoalScoreTune::kScoreFaceRad
        && std::fabs(w_now) <= GoalAlign::TURN_SETTLE_RAD_S;

    ChassisCommand cmd;
    cmd.request = DriveRequest::Motion;
    cmd.lateral = 0.0;
    cmd.brake = false;
    if (!in_window) {
        // Forward speed keeps the bumper seated. angular is the yaw rate.
        cmd.linear = GoalAlign::TURN_SURGE_MPS;
        const bool overspeed = std::fabs(w_now) > std::fabs(w_ideal) + GoalAlign::VEL_DEADBAND
            && mathutil::sign(w_now) != 0.0
            && (mathutil::sign(w_ideal) == 0.0 || mathutil::sign(w_now) == mathutil::sign(w_ideal));
        if (overspeed) {
            const double excess = std::fabs(w_now) - std::fabs(w_ideal);
            const double brake = std::min(1.0, excess / w_cap);
            cmd.angular = -mathutil::sign(w_now) * brake * GoalAlign::TURN_YAW_RAD_S;
        } else {
            cmd.angular = mathutil::sign(error) * w_mag;
        }
    }
    return cmd;
}

// Rotation centre with the bumper seated on the goal, nose at `heading`.
Pose2d seatedCenter(const Field::Spot& spot, double heading) {
    const Pose2d bumper{spot.x, spot.y, heading};
    return poseMetersToInches(tracking::centerFromTracking(poseInchesToMeters(bumper)));
}

} // namespace

int previewGoalScorePath(const Pose2d& from, bool has_aim, double aim_x, double aim_y,
                         bool has_approach, double approach_rad,
                         bool has_detach, double detach_rad,
                         Pose2d* out, int cap, Pose2d* leave) {
    if (leave) *leave = from;
    if (out == nullptr || cap < 2) return 0;
    const Pose2d center = poseMetersToInches(
        tracking::centerFromTracking(poseInchesToMeters(from)));
    const int goal = has_aim
        ? nearestGoal(Pose2d{aim_x, aim_y, 0.0}, 0.0, false)
        : nearestGoal(center, from.theta, true);
    if (goal < 0) return 0;

    const Field::Spot& spot = Field::kSpots[goal];
    StateEstimate state;
    state.pose = from;
    const Pose2d parked = has_approach
        ? alongApproach(spot, approach_rad, GoalScoreTune::kStandoffIn)
        : behindGoal(from, center, state, spot, GoalScoreTune::kStandoffIn);
    const bool back_out =
        std::hypot(from.x - spot.x, from.y - spot.y) < GoalScoreTune::kStandoffIn;
    const double final_h = has_detach ? detach_rad : parked.theta;
    const double sweep = mathutil::normalize_angle(final_h - parked.theta);
    const bool spin = has_detach && std::fabs(sweep) > 1e-3;
    const int arc_steps = spin ? 8 : 0;

    SegmentFollower nav;
    MotionPose motion;
    motion.x = parked.x;
    motion.y = parked.y;
    motion.heading = parked.theta;
    motion.v = back_out ? -1e-4 : 0.0;
    PathConstraints path;
    path.point = CommandPoint::Tracking;
    path.line_length_in = 0.0;
    path.max_accel_mps2 = 3.0;
    nav.config.forwards = !back_out;
    nav.config.cruise_speed = back_out ? 2.0 : 2.0;
    nav.config.path_accel = 1.5;
    nav.config.max_volts = System::MAX_VOLTS;
    PathObstacle circles[kPathObstacles];
    const int circle_n = Field::goalAvoidObstacles(circles, kPathObstacles);
    int kept = 0;
    for (int i = 0; i < circle_n && kept < kPathObstacles; ++i) {
        if (std::hypot(circles[i].x - spot.x, circles[i].y - spot.y) < 0.5) continue;
        nav.config.obstacles[kept++] = circles[i];
    }
    nav.config.obstacle_n = kept;
    nav.target = DriftCurveTarget::fromMotion(motion, path);

    const int tail = 2 + arc_steps;
    const int room = cap > tail ? cap - tail : cap;
    int n = nav.samplePath(from, out, room);
    const Pose2d faced{spot.x, spot.y, final_h};
    const double c = mathutil::heading_cos(final_h);
    const double s = mathutil::heading_sin(final_h);
    const Pose2d backed{faced.x - GoalScoreTune::kBackupIn * c,
                        faced.y - GoalScoreTune::kBackupIn * s,
                        final_h};
    if (leave) *leave = backed;
    if (n < 2) return 0;
    if (n < cap) out[n++] = seatedCenter(spot, parked.theta);
    if (spin) {
        for (int i = 1; i <= arc_steps && n < cap; ++i) {
            const double th = parked.theta + sweep * (static_cast<double>(i) / arc_steps);
            out[n++] = seatedCenter(spot, th);
        }
    }
    if (n < cap) out[n++] = poseMetersToInches(
        tracking::centerFromTracking(poseInchesToMeters(backed)));
    return n;
}

GoalScore& liveGoalScore() {
    static GoalScore score;
    return score;
}

const char* GoalScore::phaseName() const {
    switch (phase_) {
        case Phase::Approach: return "Approach";
        case Phase::Probe: return "Probe";
        case Phase::Latch: return "Latch";
        case Phase::Drop: return "Drop";
        case Phase::Retreat: return "Retreat";
        case Phase::Idle: break;
    }
    return "Idle";
}

void GoalScore::cancel() {
    phase_ = Phase::Idle;
    captured_ = false;
    loss_armed_ = false;
    hit_armed_ = false;
    backup_done_ = false;
    lift_armed_ = false;
    goal_ = -1;
    aim_set_ = false;
    approach_set_ = false;
    approach_live_ = false;
    detach_set_ = false;
    detach_live_ = false;
    goalAlignClearHoldOff();
}

void GoalScore::aimAt(double x_in, double y_in) {
    aim_set_ = true;
    aim_x_ = x_in;
    aim_y_ = y_in;
}

void GoalScore::clearAim() {
    aim_set_ = false;
}

void GoalScore::approachFrom(double heading_rad) {
    approach_set_ = true;
    approach_rad_ = mathutil::normalize_angle(heading_rad);
}

void GoalScore::clearApproach() {
    approach_set_ = false;
}

void GoalScore::detachAt(double heading_rad) {
    detach_set_ = true;
    detach_rad_ = mathutil::normalize_angle(heading_rad);
}

void GoalScore::clearDetach() {
    detach_set_ = false;
}

bool GoalScore::timedOut(uint32_t now_ms, uint32_t limit_ms) const {
    return now_ms - phase_ms_ >= limit_ms;
}

bool GoalScore::closeToGoal(const Pose2d& pose, double inches) const {
    if (goal_ < 0 || !std::isfinite(pose.x) || !std::isfinite(pose.y)) return false;
    const Field::Spot& spot = Field::kSpots[goal_];
    return std::hypot(pose.x - spot.x, pose.y - spot.y) <= inches;
}

void GoalScore::abandon(GoalScoreOutput& out) {
    block_abort_ = true;
    cancel();
    out.drive_active = false;
    out.drive = ChassisCommand{};
    out.command.score = false;
    out.command.cascade_height = false;
    out.command.cascade_pct = 0;
}

bool GoalScore::latched() const {
    return goal_ >= 0 && goalAlignIndex() == goal_;
}

bool GoalScore::scoreFaced(const StateEstimate& state) const {
    if (!detach_live_) return true;
    const double err = std::fabs(mathutil::normalize_angle(detach_live_rad_ - state.pose.theta));
    const double w = std::isfinite(state.v_ang) ? state.v_ang : 0.0;
    return err <= GoalScoreTune::kScoreFaceRad
        && std::fabs(w) <= GoalAlign::TURN_SETTLE_RAD_S;
}

bool GoalScore::raiseReady(const CascadeReading& sense) const {
    if (!captured_) return false;
    const double rpm = std::isfinite(sense.velocity_rpm) ? sense.velocity_rpm : 0.0;
    return cascadeLift().atTarget(lift_target_, sense)
        || (cascadeLift().atMax(sense) && std::fabs(rpm) <= Tuning::CASCADE_HOLD_RPM);
}

void GoalScore::searchStack(SubsystemCommand& cmd, const CascadeReading& sense,
                            const DistanceReading& front, uint32_t now_ms) {
    const Field::Spot& goal = Field::kSpots[goal_];
    if (scan_ == Scan::Down) {
        holdHeight(cmd, 0.0);
        if (pinSeen(front)) {
            if (!hit_armed_) {
                hit_armed_ = true;
                hit_ms_ = now_ms;
            } else if (now_ms - hit_ms_ >= GoalScoreTune::kLossMs) {
                scan_ = Scan::Up;
                loss_armed_ = false;
                hit_armed_ = false;
            }
        } else {
            hit_armed_ = false;
        }
        if (scan_ == Scan::Down && !hit_armed_ && atBottom(sense)) {
            scan_ = Scan::Hold;
            captured_ = true;
            lift_target_ = CascadeLift::clampHeight(goal.base_height_in);
        }
    }
    if (scan_ == Scan::Up) {
        cmd.cascade_height = false;
        cmd.cascade_pct = GoalScoreTune::kRaisePct;
        if (!pinSeen(front)) {
            if (!loss_armed_) {
                loss_armed_ = true;
                loss_ms_ = now_ms;
            } else if (now_ms - loss_ms_ >= GoalScoreTune::kLossMs) {
                scan_ = Scan::Hold;
                captured_ = true;
                lift_target_ = CascadeLift::clampHeight(
                    cascadeLift().heightIn(sense) + GoalScoreTune::kClearanceIn);
            }
        } else {
            loss_armed_ = false;
        }
        if (scan_ == Scan::Up && cascadeLift().atMax(sense)) {
            scan_ = Scan::Hold;
            captured_ = true;
            lift_target_ = CascadeLift::kMaxIn;
        }
    }
    if (scan_ == Scan::Hold)
        holdHeight(cmd, lift_target_);
    if (!captured_ && now_ms - search_ms_ >= GoalScoreTune::kProbeTimeoutMs) {
        scan_ = Scan::Hold;
        captured_ = true;
        lift_target_ = CascadeLift::clampHeight(cascadeLift().heightIn(sense));
        holdHeight(cmd, lift_target_);
    }
}

void GoalScore::armDrive(const Pose2d& pose, bool reverse, double line_in, bool avoid) {
    nav_.reset();
    nav_.bindFeedforward(model_);
    MotionPose goal;
    goal.x = pose.x;
    goal.y = pose.y;
    goal.heading = pose.theta;
    goal.v = reverse ? -1e-4 : 0.0;
    PathConstraints path;
    path.point = CommandPoint::Tracking;
    path.line_length_in = line_in > 0.0 ? line_in : 0.0;
    path.max_accel_mps2 = 1.5;
    nav_.config.forwards = !reverse;
    nav_.config.cruise_speed = reverse ? 1.4 : 1.4;
    nav_.config.path_accel = 2.0;
    nav_.config.max_volts = System::MAX_VOLTS;
    nav_.config.obstacle_n = 0;
    // The 4 inch stop is inside this goal's keep-out. Leaving the circle in
    // the plan sends the robot back to a gate, then forward again to latch.
    if (avoid && goal_ >= 0) {
        PathObstacle circles[kPathObstacles];
        const int circle_n = Field::goalAvoidObstacles(circles, kPathObstacles);
        const Field::Spot& spot = Field::kSpots[goal_];
        int kept = 0;
        for (int i = 0; i < circle_n && kept < kPathObstacles; ++i) {
            if (std::hypot(circles[i].x - spot.x, circles[i].y - spot.y) < 0.5) continue;
            nav_.config.obstacles[kept++] = circles[i];
        }
        nav_.config.obstacle_n = kept;
    }
    nav_.target = DriftCurveTarget::fromMotion(goal, path);
}

void GoalScore::begin(const Pose2d& center, const StateEstimate& state, uint32_t now_ms) {
    block_abort_ = false;
    if (!std::isfinite(center.x) || !std::isfinite(center.y)
        || !std::isfinite(state.pose.x) || !std::isfinite(state.pose.y)
        || !std::isfinite(state.pose.theta)) {
        phase_ = Phase::Idle;
        goal_ = -1;
        return;
    }
    released_claw_ = false;
    captured_ = false;
    loss_armed_ = false;
    hit_armed_ = false;
    backup_done_ = false;
    lift_armed_ = false;
    approach_live_ = approach_set_;
    approach_live_rad_ = approach_rad_;
    approach_set_ = false;
    detach_live_ = detach_set_;
    detach_live_rad_ = detach_rad_;
    detach_set_ = false;
    if (aim_set_) {
        goal_ = nearestGoal(Pose2d{aim_x_, aim_y_, 0.0}, 0.0, false);
        aim_set_ = false;
    } else {
        goal_ = nearestGoal(center, state.pose.theta, true);
    }
    if (goal_ < 0) {
        phase_ = Phase::Idle;
        return;
    }
    enterApproach(center, state, now_ms);
}

void GoalScore::enterApproach(const Pose2d& center, const StateEstimate& state, uint32_t now_ms) {
    phase_ = Phase::Approach;
    phase_ms_ = now_ms;
    lift_armed_ = false;
    const double remembered = Field::goalHeightIn(goal_);
    approach_in_ = CascadeLift::clampHeight(remembered - GoalScoreTune::kBelowIn);
    const Field::Spot& spot = Field::kSpots[goal_];
    const Pose2d parked = approach_live_
        ? alongApproach(spot, approach_live_rad_, GoalScoreTune::kStandoffIn)
        : behindGoal(state.pose, center, state, spot, GoalScoreTune::kStandoffIn);
    // Inside the 4 inch mark the point is behind the bumper. Back out to it
    // instead of driving through the goal. The bumper is the tracked point,
    // so the mark is its gap. The center is half a body closer when the nose
    // points away, and that distance backs the bumper through the goal.
    const double bumper_dist = std::hypot(state.pose.x - spot.x, state.pose.y - spot.y);
    const bool back_out = bumper_dist < GoalScoreTune::kStandoffIn;
    armDrive(parked, back_out, 0.0, true);
}

void GoalScore::enterProbe(const CascadeReading& sense, const DistanceReading& front,
                           uint32_t now_ms) {
    phase_ = Phase::Probe;
    phase_ms_ = now_ms;
    search_ms_ = now_ms;
    captured_ = false;
    loss_armed_ = false;
    hit_armed_ = false;
    if (pinSeen(front)) {
        scan_ = Scan::Up;
        return;
    }
    if (atBottom(sense)) {
        scan_ = Scan::Hold;
        captured_ = true;
        lift_target_ = CascadeLift::clampHeight(Field::kSpots[goal_].base_height_in);
        return;
    }
    scan_ = Scan::Down;
}

void GoalScore::enterLatch(const Pose2d& center, const Field::Spot& goal, uint32_t now_ms) {
    phase_ = Phase::Latch;
    phase_ms_ = now_ms;
    double heading = mathutil::direction_heading(goal.x - center.x, goal.y - center.y);
    if (approach_live_) heading = approach_live_rad_;
    armDrive(Pose2d{goal.x, goal.y, heading}, false, 0.0, false);
}

void GoalScore::enterDrop(const CascadeReading& sense, uint32_t now_ms) {
    phase_ = Phase::Drop;
    phase_ms_ = now_ms;
    const double height = cascadeLift().heightIn(sense);
    drop_target_ = height - GoalScoreTune::kDropIn;
    if (drop_target_ < 0.0) drop_target_ = 0.0;
}

void GoalScore::armRetreat(const StateEstimate& state) {
    // Leave on the detach heading when one was given, even if the drop timed
    // out before the nose finished the swing. A short reverse does not need
    // other-goal keep-outs; those circles can invent a path that never leaves.
    const double leave = detach_live_ ? detach_live_rad_ : state.pose.theta;
    const double c = mathutil::heading_cos(leave);
    const double s = mathutil::heading_sin(leave);
    Pose2d back = state.pose;
    back.theta = leave;
    back.x -= GoalScoreTune::kBackupIn * c;
    back.y -= GoalScoreTune::kBackupIn * s;
    armDrive(back, true, 0.0, false);
}

void GoalScore::enterRetreat(const StateEstimate& state, uint32_t now_ms,
                             IMacroController& macros, SubsystemCommand& out) {
    phase_ = Phase::Retreat;
    phase_ms_ = now_ms;
    backup_done_ = false;
    const double grown = CascadeLift::clampHeight(
        Field::goalHeightIn(goal_) + GoalScoreTune::kPieceIn);
    Field::setGoalHeightIn(goal_, grown);
    // Drop kept the bumper seated (and the plant snap latched). Hold the grab
    // off for the whole reverse so a still-overlapping nose cannot reattach.
    goalAlignHoldOff();
    armRetreat(state);
    // The latch drive and the drop are done. The backup has not moved yet.
    released_claw_ = true;
    out.pneumatics_b_state = GoalScoreTune::kClawReleased;
    macros.setPneumaticB(GoalScoreTune::kClawReleased);
}

GoalScoreOutput GoalScore::apply(const SubsystemCommand& driver,
                                 const ButtonState& buttons,
                                 const CascadeReading& sense,
                                 const DistanceReading& front,
                                 const Pose2d& center_in,
                                 const StateEstimate& state,
                                 uint32_t now_ms,
                                 IMacroController& macros,
                                 const IFeedForward* model) {
    model_ = model;
    const bool right_edge = buttons.right && !prev_right_;
    prev_right_ = buttons.right;

    if (phase_ != Phase::Idle && (right_edge || buttons.l1 || buttons.l2)) {
        cancel();
        return driverOnly(driver);
    }
    // driver.score stays true for the whole auton command, so a second score
    // chained on the next tick still starts after the first one goes idle.
    if (phase_ == Phase::Idle && (right_edge || driver.score))
        begin(center_in, state, now_ms);
    if (phase_ == Phase::Idle) return driverOnly(driver);

    GoalScoreOutput out;
    out.command = driver;
    out.command.intake_pct = 0;
    out.command.cascade_pct = 0;
    out.command.cascade_height = false;
    out.command.cascade_manual = false;
    out.command.score = true;
    out.drive_active = true;

    const Field::Spot& goal = Field::kSpots[goal_];

    switch (phase_) {
        case Phase::Idle:
            break;

        case Phase::Approach: {
            out.drive = nav_.compute(state);
            const double rise = std::fabs(approach_in_ - cascadeLift().heightIn(sense));
            if (!lift_armed_ && nav_.secondsLeft() <= liftSeconds(rise))
                lift_armed_ = true;
            if (lift_armed_) holdHeight(out.command, approach_in_);
            // The search starts when the bumper reaches the mark. The final
            // yaw keeps running on this same path. A chassis that is stuck
            // nearby starts the search too. One that never gets close ends
            // the macro, so the rest of the routine still runs.
            const bool near = closeToGoal(state.pose, GoalScoreTune::kRecoverIn);
            const bool gave_up = nav_.stuck()
                || timedOut(now_ms, GoalScoreTune::kStandoffTimeoutMs);
            if (atMark(nav_, state.pose) || (near && gave_up))
                enterProbe(sense, front, now_ms);
            else if (gave_up)
                abandon(out);
            break;
        }

        case Phase::Probe: {
            out.drive = nav_.compute(state);
            searchStack(out.command, sense, front, now_ms);
            // Settled is the planned stop. Stuck inside the standoff is the
            // bumper already on the goal: the 1 cm ball is not required.
            const bool on_goal = closeToGoal(
                state.pose, GoalScoreTune::kStandoffIn + GoalScoreTune::kMarkIn);
            const bool near = closeToGoal(state.pose, GoalScoreTune::kRecoverIn);
            const bool arrived = nav_.settled() || (nav_.stuck() && on_goal);
            const bool gave_up = nav_.stuck()
                || timedOut(now_ms, GoalScoreTune::kProbeTimeoutMs);
            if (arrived || (gave_up && near)) {
                enterLatch(center_in, goal, now_ms);
                out.drive = nav_.compute(state);
            } else if (gave_up) {
                abandon(out);
            }
            break;
        }

        case Phase::Latch:
            if (!captured_) searchStack(out.command, sense, front, now_ms);
            if (captured_) holdHeight(out.command, lift_target_);
            // The drop waits for the height. A grab before that is the bumper
            // pinning the goal while the cascade is still moving. A lift that
            // never arrives still drops, so the backup can leave.
            {
                const bool seated = latched()
                    || timedOut(now_ms, GoalScoreTune::kLatchTimeoutMs);
                const bool lift_gave_up = captured_
                    && timedOut(now_ms, GoalScoreTune::kRaiseTimeoutMs);
                if ((raiseReady(sense) && seated) || lift_gave_up) {
                    enterDrop(sense, now_ms);
                    break;
                }
            }
            out.drive = nav_.compute(state);
            if (nav_.stuck() && !latched()
                && !closeToGoal(state.pose, GoalScoreTune::kRecoverIn))
                abandon(out);
            break;

        case Phase::Drop: {
            holdHeight(out.command, drop_target_);
            const bool dropped = cascadeLift().atTarget(drop_target_, sense)
                || timedOut(now_ms, GoalScoreTune::kDropTimeoutMs);
            const bool turned = scoreFaced(state)
                || timedOut(now_ms, GoalScoreTune::kDropTimeoutMs);
            if (dropped && turned) {
                enterRetreat(state, now_ms, macros, out.command);
                break;
            }
            // The bumper is on the goal. Yaw swings it around to the detach
            // heading while the cascade drops.
            if (detach_live_) out.drive = scoreOrbit(detach_live_rad_, state);
            break;
        }

        case Phase::Retreat:
            holdHeight(out.command, 0.0);
            out.command.pneumatics_b_state = GoalScoreTune::kClawReleased;
            if (!backup_done_) {
                out.drive = nav_.compute(state);
                // Clear of the standoff, or settled on the reverse target, is a
                // real leave. Stuck on the post is not: re-plan reverse and keep
                // trying until the retreat timer.
                const bool clear = !closeToGoal(state.pose, GoalScoreTune::kStandoffIn);
                if (nav_.settled() || clear) {
                    backup_done_ = true;
                } else if (nav_.stuck()) {
                    goalAlignHoldOff();
                    armRetreat(state);
                    out.drive = nav_.compute(state);
                }
            }
            if (backup_done_) {
                out.drive_active = false;
                out.drive = ChassisCommand{};
            }
            if (timedOut(now_ms, GoalScoreTune::kRetreatTimeoutMs)
                || (backup_done_ && cascadeLift().atTarget(0.0, sense))) {
                phase_ = Phase::Idle;
                goalAlignClearHoldOff();
                out.drive_active = false;
                out.drive = ChassisCommand{};
            }
            break;
    }

    // Off through the approach, the search, and the drive in, until the
    // cascade is holding the height the search found.
    if (phase_ == Phase::Approach || phase_ == Phase::Probe
        || (phase_ == Phase::Latch && !raiseReady(sense)))
        goalAlignRelease();
    return out;
}
