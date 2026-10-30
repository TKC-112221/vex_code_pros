#pragma once
/* ---------------------------------------------------------------------------
 * DriveSegments  --  one move along a planned polyline.
 *
 * The follower plans straight segments on the first tick from the measured
 * pose, routing around the keep-outs on the constraints and around every
 * align goal, then tracks them.
 * The target is a MotionPose. Its v is the speed at the pose (0 stops).
 * Negative v arrives in reverse and may flip an earlier segment so the
 * nose does not spin through the corner. Completion is
 * SegmentFollower::settled(). stuck() also ends the leg, without counting
 * as an arrival, so the routine starts the next command. So does a pose
 * that is not a number, and a leg that outlasts the follower's own time
 * estimate. path.handoff_in > 0 finishes earlier, once the robot is that
 * close, while v is still non-zero.
 * ------------------------------------------------------------------------- */

#include "auton/Command.h"
#include "auton/MotionLegGuard.h"
#include "control/SegmentFollower.h"
#include "field/FieldPoints.h"
#include "interfaces/IFeedForward.h"
#include <cmath>

namespace auton {

class DriveSegments : public Command {
public:
    DriveSegments(MotionPose target, PathConstraints path, const IFeedForward* model)
        : target_(target), path_(path) {
        if (path_.line_length_in < 0.0) path_.line_length_in = 0.0;
        if (path_.handoff_in < 0.0) path_.handoff_in = 0.0;

        follower_.config.forwards = target_.v >= 0.0;
        follower_.config.field_bound = path_.field_bound;
        int n = path_.obstacle_n;
        if (n < 0) n = 0;
        if (n > kPathObstacles) n = kPathObstacles;
        follower_.config.obstacle_n = 0;
        for (int i = 0; i < n; ++i) {
            if (!(path_.obstacles[i].min_dist > 0.0)) continue;
            PathObstacle& dst = follower_.config.obstacles[follower_.config.obstacle_n];
            dst = path_.obstacles[i];
            ++follower_.config.obstacle_n;
        }
        PathObstacle goals[kPathObstacles];
        const int goal_n = Field::goalAvoidObstacles(goals, kPathObstacles);
        for (int i = 0; i < goal_n && follower_.config.obstacle_n < kPathObstacles; ++i) {
            bool listed = false;
            for (int j = 0; j < follower_.config.obstacle_n; ++j) {
                const PathObstacle& have = follower_.config.obstacles[j];
                if (std::hypot(have.x - goals[i].x, have.y - goals[i].y) < 0.5 &&
                    std::fabs(have.min_dist - goals[i].min_dist) < 0.1) {
                    listed = true;
                    break;
                }
            }
            if (listed) continue;
            follower_.config.obstacles[follower_.config.obstacle_n] = goals[i];
            ++follower_.config.obstacle_n;
        }
        follower_.target = DriftCurveTarget::fromMotion(target_, path_);
        follower_.bindFeedforward(model);
    }

    void initialize(const CommandTick& tick) override {
        (void)tick;
        follower_.reset();
        guard_.reset();
        paused_ = false;
    }

    bool execute(CommandTick& tick) override {
        if (tick.pause_motion) {
            paused_ = true;
            return false;
        }
        if (paused_) {
            follower_.reset();
            guard_.reset();
            paused_ = false;
        }
        if (!MotionLegGuard::usable(tick.state)) {
            yieldMotion(tick);
            return true;
        }
        tick.setDrive(follower_.compute(tick.state));
        if (follower_.settled()) return true;
        if (follower_.stuck() || guard_.overran(follower_.secondsLeft(), 2.0, 10.0)) {
            yieldMotion(tick);
            return true;
        }
        if (path_.handoff_in > 0.0 && std::fabs(target_.v) > 0.05) {
            if (distanceInches(tick.state.pose, target_.geometry()) <= path_.handoff_in) return true;
        }
        return false;
    }

    int samplePath(const Pose2d& from, Pose2d* out, int cap,
                   double v_lin, double v_ang) const override {
        return follower_.samplePath(from, out, cap, v_lin, v_ang);
    }

private:
    SegmentFollower follower_;
    MotionLegGuard guard_;
    MotionPose target_;
    PathConstraints path_;
    bool paused_ = false;
};

} // namespace auton
