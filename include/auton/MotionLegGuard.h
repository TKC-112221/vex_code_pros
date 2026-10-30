#pragma once
/* ---------------------------------------------------------------------------
 * MotionLegGuard  --  a motion leg that has stopped being worth waiting for.
 *
 * Stuck detection covers a chassis that was asked to move and did not.
 * This covers the other hang: the pose is not a number, or the leg has
 * outlasted its own time estimate because the wheels are slipping or the
 * path is blocked. The routine then starts the next command.
 * ------------------------------------------------------------------------- */

#include "auton/Command.h"
#include "core/Specs.h"
#include <cmath>

namespace auton {

class MotionLegGuard {
public:
    void reset() {
        elapsed_sec_ = 0.0;
        budget_sec_ = 0.0;
        armed_ = false;
    }

    // Pose and speed the follower can plan from. A bad sample ends the leg
    // before it is allowed to command the chassis.
    static bool usable(const StateEstimate& state) {
        const Pose2d& pose = state.pose;
        return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.theta)
            && std::isfinite(state.v_lin) && std::isfinite(state.v_ang);
    }

    // eta_sec is how long the follower still expects this leg to take.
    // The first finite estimate sets the budget. Later calls only count time.
    // min_sec and max_sec keep a short leg from ending on the first hiccup
    // and a long estimate from consuming the rest of the period.
    bool overran(double eta_sec, double min_sec, double max_sec) {
        elapsed_sec_ += System::DT_SEC;
        if (!armed_ && std::isfinite(eta_sec) && eta_sec > 0.0) {
            double budget = eta_sec * 2.5 + 1.0;
            if (budget < min_sec) budget = min_sec;
            if (budget > max_sec) budget = max_sec;
            budget_sec_ = budget;
            armed_ = true;
        }
        const double limit = armed_ ? budget_sec_ : max_sec;
        return elapsed_sec_ >= limit;
    }

private:
    double elapsed_sec_ = 0.0;
    double budget_sec_ = 0.0;
    bool armed_ = false;
};

inline void yieldMotion(CommandTick& tick) {
    tick.yielded = true;
    tick.drive_active = false;
    tick.drive = ChassisCommand{};
}

} // namespace auton
