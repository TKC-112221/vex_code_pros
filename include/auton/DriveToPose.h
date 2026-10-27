#pragma once
/* ---------------------------------------------------------------------------
 * DriveToPose  --  one continuous move, driven by DriftFollower.
 *
 * The follower plans a curve on the first tick from the measured pose and
 * momentum, then tracks it. The target is a MotionPose: its v is the speed
 * at the pose (0 stops, anything else is a fly-by). Negative v arrives in
 * reverse. Completion is DriftFollower::settled(): forward speed near zero,
 * or the distance to the target starting to grow. It does not wait to enter
 * a coordinate window. path.handoff_in > 0 finishes earlier, once the robot
 * is that close, while v is still non-zero.
 * path.turn_on_goal does not use that distance. The leg finishes when the
 * heading has settled on the orbit. path.follower_input selects curvature
 * or heading stations for the slip and yaw feedforward.
 * ------------------------------------------------------------------------- */

#include "auton/Command.h"
#include "control/DriftFollower.h"
#include "interfaces/IFeedForward.h"
#include <cmath>

namespace auton {

class DriveToPose : public Command {
public:
    DriveToPose(MotionPose target, PathConstraints path, const IFeedForward* model)
        : target_(target), path_(path) {
        if (path_.line_length_in < 0.0) path_.line_length_in = 0.0;
        if (path_.handoff_in < 0.0) path_.handoff_in = 0.0;

        follower_.config.forwards = target_.v >= 0.0;
        follower_.config.input = path_.follower_input;
        follower_.target = DriftCurveTarget::fromMotion(target_, path_);
        follower_.bindFeedforward(model);
    }

    void initialize(const CommandTick& tick) override {
        (void)tick;
        follower_.reset();
    }

    bool execute(CommandTick& tick) override {
        tick.setDrive(follower_.compute(tick.state));
        if (follower_.settled()) return true;
        if (path_.handoff_in > 0.0 && !path_.turn_on_goal && std::fabs(target_.v) > 0.05) {
            if (distanceInches(tick.state.pose, target_.geometry()) <= path_.handoff_in) return true;
        }
        return false;
    }

    int samplePath(const Pose2d& from, Pose2d* out, int cap,
                   double v_lin, double v_ang) const override {
        return follower_.samplePath(from, out, cap, v_lin, v_ang);
    }

private:
    DriftFollower follower_;
    MotionPose target_;
    PathConstraints path_;
};
} // namespace auton
