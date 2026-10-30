#include "logic/UserInputOverride.h"
#include "control/DriftFollower.h"
#include "control/DrivePolicy.h"
#include "control/SegmentFollower.h"
#include "config/Settings.h"
#include "logic/DriftCurveOverride.h"
#include "field/FieldPoints.h"
#include "logic/GoalAligner.h"
#include "math/TargetPlane.h"
#include "math/MathUtils.h"
#include "core/Tuning.h"
#include <cmath>

namespace {

SegmentFollower g_nav;
bool g_prev_a = false;

// Crossing watch. The plane normal is frozen on the approach: rebuilding
// it from a pose already past the target flips the plane, and a missed
// crossing can never be seen again.
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

void noteApproachPose(const Pose2d& pose) {
    g_prev_pose = pose;
    g_have_prev = true;
}

bool sticksActive(const JoystickData& joy) {
    const double db = Controls::STICK_ZERO_DEADBAND;
    return std::fabs(joy.left_x)  > db || std::fabs(joy.left_y)  > db
        || std::fabs(joy.right_x) > db || std::fabs(joy.right_y) > db;
}

// Recording uses the same stick path as idle, so a curve logs in either.
// Skills replaces that path. Replay does not.
bool inDriverControl(RobotMode mode) {
    return mode == RobotMode::IDLE || mode == RobotMode::RECORDING;
}

// Exit heading, or the bearing into the target when heading is ignored.
double planeNormal(const DriftCurveTarget& target, const Pose2d& from) {
    if (!target.ignore_direction) return target.exit_direction;
    const double dx = target.x - from.x;
    const double dy = target.y - from.y;
    if (std::hypot(dx, dy) <= 1e-4) return target.exit_direction;
    return mathutil::direction_heading(dx, dy);
}

void armPracticeDrive() {
    const MotionPose goal{Practice::GOAL_X, Practice::GOAL_Y,
                          Practice::GOAL_HEADING_RAD, Practice::EXIT_SPEED_MPS, 0.0};
    PathConstraints limits;
    limits.point = CommandPoint::Tracking;
    limits.max_accel_mps2 = Practice::MAX_ACCEL_MPS2;
    limits.line_length_in = Practice::LINE_LENGTH * 12.0;

    g_nav.reset();
    DrivePolicy::reset();
    g_nav.config.forwards = goal.v >= 0.0;
    g_nav.config.obstacle_n = Field::goalAvoidObstacles(g_nav.config.obstacles, kPathObstacles);
    g_nav.target = DriftCurveTarget::fromMotion(goal, limits);
    clearPlaneWatch();
    g_drift_curve_on = true;
}

// A stop arrives with almost no nose speed, so the velocity gate would
// reject it. Count the crossing once the pose is through the plane.
void abortIfPastExitPlane(const StateEstimate& estimate) {
    if (!g_drift_curve_on || !g_have_prev) return;

    if (!g_plane_frozen)
        g_plane_heading = planeNormal(g_nav.target, g_prev_pose);

    const Pose2d plane{g_nav.target.x, g_nav.target.y, g_plane_heading};
    if (mathutil::targetPlaneAlong(g_prev_pose, plane) < 0.0) g_seen_behind = true;
    if (g_seen_behind && mathutil::targetPlaneAlong(estimate.pose, plane) >= 0.0) {
        g_past_plane = true;
        g_plane_frozen = true;
    }

    const bool stopping = std::fabs(g_nav.target.exit_speed) < 1e-3;
    const bool moving_through =
        mathutil::targetPlaneNoseSpeed(estimate.pose, estimate.v_lin, plane) > 0.0;
    if (g_past_plane && (stopping || moving_through)) g_drift_curve_abort = true;
}

} // namespace

bool g_drift_curve_on = false;
bool g_drift_curve_abort = false;

SegmentFollower& driftCurve() { return g_nav; }

void driftCurveOverridePoll(const ButtonState& buttons,
                            const JoystickData& raw_sticks,
                            RobotMode mode) {
    // A alone, rising edge. A+B reloads a recording and must not start a drive.
    const bool start = inDriverControl(mode) && buttons.a && !buttons.b && !g_prev_a;
    g_prev_a = buttons.a;
    if (start) armPracticeDrive();

    // Holding A keeps a drive already in progress. Sticks abort after release.
    if (!g_drift_curve_on) return;
    if (!inDriverControl(mode) || (sticksActive(raw_sticks) && !buttons.a))
        g_drift_curve_abort = true;
}

CommandVector driftCurveOverride(const StateEstimate& estimate,
                                 const IFeedForward* model) {
    if (Settings::ControlToggles::USE_DRIVE_POLICY)
        return DrivePolicy::command(estimate, g_nav.target);
    g_nav.bindFeedforward(model);
    return CommandVector::fromChassis(g_nav.compute(estimate));
}

void driftCurveOverrideStop() {
    g_drift_curve_on = false;
    g_drift_curve_abort = false;
    clearPlaneWatch();
    g_nav.reset();
    DrivePolicy::reset();
}

int driftCurveOverridePath(const Pose2d& from, Pose2d* out, int cap,
                           double v_lin, double v_ang) {
    return g_nav.samplePath(from, out, cap, v_lin, v_ang);
}

OverrideResult userInputOverride(const ChassisObservation& /*sensor*/,
                                 const CommandVector& commands,
                                 const ButtonState& buttons,
                                 const StateEstimate& estimate,
                                 const Pose2d& body,
                                 const JoystickData& raw_sticks,
                                 RobotMode mode,
                                 const IFeedForward* model,
                                 const CommandVector* motion) {
    OverrideResult out;
    out.commands = commands;
    out.buttons  = buttons;

    driftCurveOverridePoll(buttons, raw_sticks, mode);
    abortIfPastExitPlane(estimate);

    if (g_drift_curve_abort) driftCurveOverrideStop();

    if (motion != nullptr) {
        // A routine's control vector replaces the sticks. The drive base
        // is what turns it into voltage.
        out.commands = *motion;
    } else if (g_drift_curve_on) {
        // The curve owns the command. The plant snap still seats the chassis
        // when the front edge reaches a goal.
        out.commands = driftCurveOverride(estimate, model);
        noteApproachPose(estimate.pose);
    } else if (inDriverControl(mode)) {
        out.commands = goalAlignOverride(out.commands, body, raw_sticks, model);
    }
    return out;
}

int userInputOverridePath(const Pose2d& from, const Pose2d& body, Pose2d* out, int cap,
                          double v_lin, double v_ang) {
    if (out == nullptr || cap < 2) return 0;
    if (g_drift_curve_on && !g_drift_curve_abort)
        return driftCurveOverridePath(from, out, cap, v_lin, v_ang);
    return goalAlignPath(body, out, cap);
}
