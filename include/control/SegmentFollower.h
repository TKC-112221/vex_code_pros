#pragma once
/* ---------------------------------------------------------------------------
 * SegmentFollower  --  one planned polyline, then a Stanley follower.
 *
 * Same calls as DriftFollower. reset() drops the plan. The next compute()
 * builds it from the pose it is given and keeps that polyline. Later ticks
 * follow it. They do not rebuild it.
 *
 * The path is straight segments from the robot to one target. A straight
 * finish of target.line_length runs into the target along exit_direction,
 * and the search ends at the far end of that line. ignore_direction drops
 * the finish and aims at the point. exit_speed is the speed at the target.
 *
 * Keep-outs are circles, inches in config, the same frame as the target.
 * min_dist is the obstacle's own radius, unless the obstacle is fixed,
 * in which case min_dist is the circle itself. A Tracking command names
 * the front bumper. The plan is shifted to the centre of rotation by the
 * offset in TrackingCenter.h, and the field line is that centre path.
 * The bumper must stay outside the circle. A grown circle is the obstacle
 * plus the far corner (a body length back and half a track to the side)
 * plus a clearance margin. On a straight segment the bumper rides the centre
 * line, so that line stays outside the circle. A pivot swings the bumper
 * around the centre by the tracking offset, so every corner vertex sits
 * that offset further out. The planner puts an 8-sided polygon just
 * outside that larger circle, then searches the straight lines between
 * those vertices, the robot, and the goal. A line that would bring the
 * bumper into a circle is thrown out, and so is a departure whose pivot
 * would. A circle the robot is already inside can be left, but the
 * leaving segment may not get closer to that obstacle than the robot
 * already is. A circle that holds the goal can be entered only along the
 * ray from the obstacle through the goal, and the path may not pass
 * closer than the nearer of the robot and the goal. Later segments still
 * stay outside both.
 * field_bound drops vertices that sit outside the field. If no line
 * reaches the goal, the plan is the straight line anyway.
 *
 * Each segment has a drive sign. The last one follows
 * config.forwards && !target.backwards, the same rule DriftFollower uses:
 * the nose points opposite travel when that is false. Earlier signs are
 * chosen so the nose turns as little as it can at each vertex. Any turn
 * stops the chassis. A corner is reached once the rotation centre is
 * within 1 cm of the next segment's line, including a short stretch of
 * that line extended back past the vertex. The pivot starts there. A
 * small left/right speed difference arcs the centre onto the line as the
 * nose arrives, and a miss of a couple of centimetres along the line is
 * left where the turn finishes. The end of the path is still the 1 cm
 * ball around the last point.
 * The heading target does not look past the vertex.
 * A stop does not finish on the last segment's nose: once the centre is
 * on the end point it yaws in place onto the exit heading, and the leg
 * stays open until that turn is done.
 *
 * Tracking is Stanley. The heading target is the segment nose plus
 *
 *   atan(stanley_k * cross_track / (stanley_soft + speed))
 *
 * Positive cross-track is left of the direction of travel, and the
 * correction turns the travel heading back toward the segment. There is
 * no slip angle. compute() returns a Motion control vector: forward speed,
 * forward accel, yaw rate, and yaw accel. It does not compute voltage.
 * The drive base turns that vector into volts. The commanded rate ramps
 * up to the brake curve, and it drops to the curve when the nose is close
 * enough that the old rate would run past the heading. Speeding up takes
 * one profile-accel step per tick, and that profile is the path accel
 * with overhead left unused. Slowing down is not capped at the path
 * accel: measured speed above the curve may brake at the full armature
 * accel, and that accel is part of the control vector.
 *
 * A vertex stop, including the join in front of the straight finish, is
 * done when the centre has stopped at that vertex. That tick starts the
 * pivot. The slide may walk the centre onto the next line while the nose
 * turns. It does not decide whether the pivot runs. The end of the path
 * stops in the 1 cm ball around the last point, and settled means the
 * centre is at rest there. A stop at an earlier vertex is not the end of
 * the leg.
 *
 * stuck() is a separate give-up. The leg asked for translation, or for a
 * pivot, and the chassis answered with neither for Motion::STUCK_SEC.
 * A hold that asks for neither leaves it clear, and settled() stays false
 * so the caller can move on without treating the stop as an arrival. A
 * stop at a vertex is not that hold: it commands the pivot. A chassis
 * that has already stopped short of the end, with the nose still outside
 * the done window and no drive command, is that same pivot. The drive
 * base treats it as an in-place yaw, so the standing voltage is applied.
 * ------------------------------------------------------------------------- */

#include "control/DriftCurve.h"
#include "control/MotionStuck.h"

class IFeedForward;

// Gap past the bumper corner, meters. The corner already includes half
// the track, so this is clearance beyond the robot's width.
constexpr double kSegmentClearMarginM = 0.05;

// Radius the bumper keeps from an obstacle of radius obstacle_m, including
// while the chassis pivots. The far corner is a body length back from the
// bumper and half a track to the side.
inline double segmentKeepRadiusM(double obstacle_m) {
    if (!(obstacle_m > 0.0)) obstacle_m = 0.0;
    const double side = Robot::Chassis::TRACK_WIDTH_M * 0.5;
    const double back = Robot::Chassis::BODY_LENGTH_M;
    return obstacle_m + std::hypot(back, side) + kSegmentClearMarginM;
}

// DriftCurveConfig supplies forwards, path_accel, cruise_speed, max_volts,
// and close_m. A second copy of those members here would hide them.
struct SegmentFollowerConfig : DriftCurveConfig {
    // Stanley: atan(k * cross_track / (soft + speed)). k is m/s, soft keeps
    // the correction finite while the chassis is stopped.
    double stanley_k = Segment::STANLEY_K;
    double stanley_soft = Segment::STANLEY_SOFT;
    // Mid-segment nose error that stops the chassis to square up, radians.
    // A corner in the polyline always stops, whatever this value is.
    double pivot_rad = Segment::PIVOT_RAD;
    // Heading window that releases a stop, radians.
    double pivot_done_rad = 0.12;

    double min_accel = Motion::MIN_ACCEL_MPS2;
    double settle_speed = Motion::SETTLE_SPEED_MPS;
    double settle_in = Boomerang::Lateral.small_error;
    double settle_deg = Boomerang::Angular.small_error;

    // Written by the same configure call as the curve follower. The pivot
    // does not apply them; yaw is the wheel armature on the trapezoid.
    double head_kP = Boomerang::Angular.kP;
    double head_kI = Boomerang::Angular.kI;
    double head_kD = Boomerang::Angular.kD;
    double head_windup = Boomerang::Angular.windup;

    bool field_bound = false;
    int obstacle_n = 0;
    PathObstacle obstacles[kPathObstacles] {};
};

class SegmentFollower {
public:
    SegmentFollowerConfig config;
    DriftCurveTarget target;

    void reset();

    // Kept so a caller can still hand over a model. The drive base owns
    // the voltage, so this follower does not read it.
    void bindFeedforward(const IFeedForward*) {}

    // state : field pose, forward speed (v_lin, m/s), yaw rate (v_ang, rad/s).
    // The first call after reset() plans from this state. Later calls track.
    ChassisCommand compute(const StateEstimate& state);

    // Heading the chassis was told to face on the last compute(), radians.
    double goalHeading() const { return goal_heading_; }

    // True once the last segment is done: slow, and close to the end of the
    // polyline. A stop also waits until the nose is on the exit heading.
    // A stop in the middle of the path is not the end.
    bool settled() const { return settled_; }

    // True once a translate or pivot request has gone unanswered. A hold
    // does not set it, and a settled leg clears it.
    bool stuck() const { return stuck_watch_.stuck(); }

    // Seconds the speed profile still needs, from the last compute().
    // In-place yaws at corners are included. 0 before the first compute()
    // and once the leg has settled.
    double secondsLeft() const { return seconds_left_; }

    // Polyline the follower is tracking, the centre of rotation, in field
    // inches. A command pose is the front bumper; this line is the axle.
    // Before the first compute() this is a preview from `from` and does not
    // commit the plan. A later pose does not rebuild a plan compute() kept.
    int samplePath(const Pose2d& from, Pose2d* out, int cap,
                   double v_lin = 0, double v_ang = 0) const;

    // The polyline planned once. There is no second fit, so this is the
    // same path samplePath returns. Before the first compute() it previews
    // and does not commit.
    int sampleBase(const Pose2d& from, Pose2d* out, int cap,
                   double v_lin = 0, double v_ang = 0) const;

    // Achievable lateral acceleration from the last compute(), m/s^2.
    // A straight segment has no curvature, so this stays zero.
    double achievableLateral() const { return a_actual_lat_; }

private:
    struct Vertex {
        double x = 0, y = 0, s = 0;
        double phi = 0;   // travel heading of the segment that leaves this vertex
        double drive = 1; // +1 nose leads travel, −1 the segment is driven in reverse
        double v = 0;     // speed ceiling at this vertex, m/s along travel
        bool stop = false;
    };

    // Room for the start, the goal, an 8-gon around each keep-out, and the finish.
    static constexpr int kCap = 40;

    static int build(const SegmentFollowerConfig& config, const DriftCurveTarget& target_m,
                     const Pose2d& from_m, Vertex* out);
    static void project(const Vertex* pts, int seg, double rx, double ry,
                        double& t, double& lat, double& remain);
    static void stamp(Vertex* pts, int n, const SegmentFollowerConfig& config,
                      const DriftCurveTarget& target_m, double accel, double cruise, double v_end);
    static int dropCollinear(Vertex* pts, int n);

    // Drive time from `along` (0..1 on segment `seg`) plus any yaw still
    // ahead. `corner` means the live yaw is the turn onto the next segment,
    // so that vertex is not counted again.
    double etaSeconds(int seg, double along, double v_now, bool yawing, bool corner,
                      double yaw_err) const;

    Vertex pts_[kCap] {};
    int count_ = 0;
    bool planned_ = false;
    int seg_ = 0;
    bool started_ = false;
    bool held_turn_ = false;
    // The corner pivot stays up from the stop at the vertex until the nose
    // is on the next segment. Braking can nudge the centre out of the
    // strip for a tick; dropping the pivot there leaves it short.
    bool corner_hold_ = false;
    // Set once a stop has begun its yaw, so the brake that comes first is
    // not asked to stop again when the slide adds a little forward speed.
    bool spin_latched_ = false;
    // Slewed yaw rate, rad/s. Reset with the plan.
    double yaw_rate_ = 0;
    double accel_ = 0;
    double cruise_ = 0;

    double prev_v_cmd_ = 0;
    bool v_cmd_primed_ = false;
    double goal_heading_ = 0;
    double a_actual_lat_ = 0;
    bool settled_ = false;
    MotionStuck stuck_watch_;
    double seconds_left_ = 0;
};
