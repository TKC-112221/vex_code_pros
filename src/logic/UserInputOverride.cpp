#include "logic/UserInputOverride.h"
#include "control/DriftFollower.h"
#include "control/DrivePolicy.h"
#include "config/Settings.h"
#include "logic/DriftCurveOverride.h"
#include "math/TargetPlane.h"
#include "core/Tuning.h"
#include <cmath>

namespace {

    // DriftCurve g_nav;
    DriftFollower g_nav;
    bool g_prev_a = false;
    bool g_have_prev = false;
    bool g_seen_behind = false;
    bool g_past_plane = false;
    bool g_plane_frozen = false;
    double g_plane_heading = 0.0;
    Pose2d g_prev_pose{};

    void clearPlaneWatch() {
        g_have_prev = false;
        g_seen_behind = false;
        g_past_plane = false;
        g_plane_frozen = false;
        g_plane_heading = 0.0;
    }

    bool sticksActive(const JoystickData& joy) {
        const double db = Controls::STICK_ZERO_DEADBAND;
        return std::fabs(joy.left_x)  > db || std::fabs(joy.left_y)  > db
            || std::fabs(joy.right_x) > db || std::fabs(joy.right_y) > db;
    }

} // namespace

bool g_drift_curve_on = false;
bool g_drift_curve_abort = false;

DriftFollower& driftCurve() { return g_nav; }

void driftCurveOverridePoll(const ButtonState& buttons,
                            const JoystickData& raw_sticks,
                            RobotMode mode) {
    // A alone. A+B reloads a recording and must not start a drive.
    const bool start = mode == RobotMode::IDLE && buttons.a && !buttons.b && !g_prev_a;
    g_prev_a = buttons.a;

    if (start) {
        g_nav.reset();
        MotionPose goal{metersToInches(Practice::GOAL_X_M), metersToInches(Practice::GOAL_Y_M),
                        Practice::GOAL_HEADING_RAD, Practice::EXIT_SPEED_MPS, 0.0};
        PathConstraints limits;
        limits.max_accel_mps2 = Practice::MAX_ACCEL_MPS2;
        limits.line_length_in = metersToInches(Practice::LINE_LENGTH_M);
        driftCurve().config.forwards = goal.v >= 0.0;
        driftCurve().target = DriftCurveTarget::fromMotion(goal, limits);
        clearPlaneWatch();
        g_drift_curve_on = true;
    }
    if (g_drift_curve_on && mode != RobotMode::IDLE) g_drift_curve_abort = true;
    // Holding A keeps a drive that was already in progress from cancelling
    // the curve. Sticks abort only after A is released.
    if (g_drift_curve_on && sticksActive(raw_sticks) && !buttons.a) {
        g_drift_curve_abort = true;
    }
}

CommandVector driftCurveOverride(const StateEstimate& estimate,
                                 const IFeedForward* model) {
    if (Settings::ControlToggles::USE_DRIVE_POLICY) {
        return DrivePolicy::command(estimate, g_nav.target);
    }
    g_nav.bindFeedforward(model);
    return CommandVector::fromChassis(g_nav.compute(estimate));
}

void driftCurveOverrideStop() {
    g_drift_curve_on = false;
    g_drift_curve_abort = false;
    clearPlaneWatch();
    g_nav.reset();
}

namespace {

// Plane normal is the exit heading, or the bearing into the target when
// the drive is not using a heading.
double planeNormal(const DriftCurveTarget& target, const Pose2d& from) {
    if (!target.ignore_direction) return target.exit_direction;
    const double dx = target.x - from.x;
    const double dy = target.y - from.y;
    if (std::hypot(dx, dy) <= 1e-4) return target.exit_direction;
    return std::atan2(dy, dx);
}

} // namespace

int driftCurveOverridePath(const Pose2d& from, Pose2d* out, int cap,
                           double v_lin, double v_ang) {
    return g_nav.samplePath(from, out, cap, v_lin, v_ang);
}


OverrideResult userInputOverride(const ChassisObservation& /*sensor*/,
                                 const CommandVector& commands,
                                 const ButtonState& buttons,
                                 const StateEstimate& estimate,
                                 const JoystickData& raw_sticks,
                                 RobotMode mode,
                                 const IFeedForward* model) {
    OverrideResult out;
    out.commands = commands;
    out.buttons  = buttons;

    driftCurveOverridePoll(buttons, raw_sticks, mode);
    if (g_drift_curve_on && g_have_prev) {
        // Keep the normal taken while still approaching. Rebuilding it from a
        // pose that is already past the target flips the plane, so a missed
        // crossing can never be seen again.
        if (!g_plane_frozen) {
            g_plane_heading = planeNormal(g_nav.target, g_prev_pose);
        }
        const Pose2d plane{g_nav.target.x, g_nav.target.y, g_plane_heading};
        const double prev_x = mathutil::targetPlaneAlong(g_prev_pose, plane);
        const double cur_x = mathutil::targetPlaneAlong(estimate.pose, plane);
        if (prev_x < 0.0) g_seen_behind = true;
        if (g_seen_behind && cur_x >= 0.0) {
            g_past_plane = true;
            g_plane_frozen = true;
        }
        // A stop arrives with v_lin about 0, and the nose may no longer lie
        // along the plane, so the forward-velocity gate would reject it.
        const bool stopping = std::fabs(g_nav.target.exit_speed) < 1e-3;
        const bool moving_through =
            mathutil::targetPlaneNoseSpeed(estimate.pose, estimate.v_lin, plane) > 0.0;
        if (g_past_plane && (stopping || moving_through)) {
            g_drift_curve_abort = true;
        }
    }
    if (g_drift_curve_abort) {
        driftCurveOverrideStop();
    } else if (g_drift_curve_on) {
        out.commands = driftCurveOverride(estimate, model);
        g_prev_pose = estimate.pose;
        g_have_prev = true;
    }
    return out;
}

int userInputOverridePath(const Pose2d& from, Pose2d* out, int cap,
                          double v_lin, double v_ang) {
    if (g_drift_curve_abort || !g_drift_curve_on || out == nullptr || cap < 2) return 0;
    return driftCurveOverridePath(from, out, cap, v_lin, v_ang);
}