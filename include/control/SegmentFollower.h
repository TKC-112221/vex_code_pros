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
 * The planner puts an 8-sided polygon just outside each circle so every
 * side is tangent to it, then searches the straight lines between those
 * vertices, the robot, and the goal. A line that enters a circle is thrown
 * out. field_bound drops vertices that sit outside the field. If no line
 * reaches the goal, the plan is the straight line anyway.
 *
 * Each segment has a drive sign. The last one follows target.backwards:
 * the nose points opposite travel when that is set. Earlier signs are
 * chosen so the nose turns as little as it can at each vertex. A sign
 * change stops the chassis. It waits until it is slow, then leaves on the
 * new segment. A nose change larger than pivot_rad stops too, and the
 * chassis yaws in place onto the next nose. A smaller change stays on the
 * segments. Speed falls as that change grows. The heading target does not
 * look past the vertex, so the path is not cut short of the corner.
 *
 * Tracking is Stanley. The heading target is the segment nose plus
 *
 *   atan(stanley_k * cross_track / (stanley_soft + speed))
 *
 * Positive cross-track is left of the direction of travel, and the
 * correction turns the travel heading back toward the segment. There is
 * no slip angle. A heading PID turns the nose onto that target.
 * Longitudinal volts are the feedforward of the slewed speed. The slew
 * steps from the previous command, one acceleration step per tick.
 *
 * Settled means the chassis is slow on the last segment, close to the end
 * of the polyline, or it has passed the target. A stop at an earlier
 * vertex is not the end of the leg. A detour that walks away from the
 * target before coming back is not the end either.
 * ------------------------------------------------------------------------- */

#include "control/DriftCurve.h"

struct SegmentFollowerConfig {
    // Stanley: atan(k * cross_track / (soft + speed)). k is m/s, soft keeps
    // the correction finite while the chassis is stopped.
    double stanley_k = 1.5;
    double stanley_soft = 0.30;
    // Nose change that stops the chassis for a pivot, radians. Smaller
    // corners are driven, with the speed capped by corner_yaw.
    double pivot_rad = 50.0 * Robot::PI / 180.0;
    // Heading window that releases a pivot, radians.
    double pivot_done_rad = 0.12;
    // A corner the chassis does not stop for is taken no faster than
    // corner_yaw * corner_len / |heading change|.
    double corner_yaw = 2.5;
    double corner_len_m = 0.15;

    double path_accel = Drift::PATH_ACCEL;
    double cruise_speed = Robot::MAX_VEL_LIN;
    double min_accel = Motion::MIN_ACCEL_MPS2;
    double max_volts = System::MAX_VOLTS;
    double settle_speed = Motion::SETTLE_SPEED_MPS;

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

    // Chassis model for the linear volts. Null leaves them at zero.
    // The pointer is not owned. Yaw volts are the heading PID.
    void bindFeedforward(const IFeedForward* model) { model_ = model; }

    // state : field pose, forward speed (v_lin, m/s), yaw rate (v_ang, rad/s).
    // The first call after reset() plans from this pose. Later calls track.
    ChassisCommand compute(const StateEstimate& state);

    // Heading the chassis was told to face on the last compute(), radians.
    double goalHeading() const { return goal_heading_; }

    // True once the last segment is done: slow, and close to the end of the
    // polyline, or past the target after having closed on it. A stop in the
    // middle of the path is not the end.
    bool settled() const { return settled_; }

    // Polyline the follower is tracking, inches. Before the first compute()
    // this is a preview from `from` and does not commit the plan. A later
    // pose does not rebuild a plan that compute() has already kept.
    // v_lin and v_ang are unused; the shape does not open on momentum.
    int samplePath(const Pose2d& from, Pose2d* out, int cap,
                   double v_lin = 0, double v_ang = 0) const;

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

    Vertex pts_[kCap] {};
    int count_ = 0;
    bool planned_ = false;
    int seg_ = 0;
    bool started_ = false;
    bool yaw_only_ = false;
    double accel_ = 0;
    double cruise_ = 0;

    double i_ang_ = 0, prev_ang_ = 0;
    bool head_primed_ = false;
    double prev_v_cmd_ = 0;
    bool v_cmd_primed_ = false;
    double goal_heading_ = 0;
    bool settled_ = false;
    bool dist_armed_ = false;
    bool saw_motion_ = false;
    double start_dist_m_ = 0;
    double best_dist_m_ = 0;
    const IFeedForward* model_ = nullptr;
};
