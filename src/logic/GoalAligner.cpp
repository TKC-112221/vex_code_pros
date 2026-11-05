/* ---------------------------------------------------------------------------
 * GoalAligner.cpp  --  attach when the front edge touches a goal point.
 * The yaw stick orbits that point in driver control. The plant grab seats
 * the nose in every mode, after the rest of the contact step.
 * ------------------------------------------------------------------------- */
#include "logic/GoalAligner.h"
#include "field/FieldPoints.h"
#include "core/Tuning.h"
#include "core/Specs.h"
#include "math/FrontEdge.h"
#include "math/MathUtils.h"
#include "interfaces/IFeedForward.h"
#include <cmath>

namespace {

int g_latched = -1;
// Reverse, or a negative wheel speed, sets this for the rest of the physics
// step. goalAlignPhysicsDone clears it once the grab has finished, unless
// a hold-off is keeping the grab suppressed across steps.
bool g_suppress = false;
bool g_hold_off = false;
// The firmware keeps the grab. A scoring rollout turns it off.
bool g_enabled = true;

CommandVector voltsFor(double v_cmd, double omega, const IFeedForward* model) {
    (void)model;
    if (std::fabs(v_cmd) < GoalAlign::VEL_DEADBAND) v_cmd = 0.0;
    if (std::fabs(omega) < GoalAlign::VEL_DEADBAND) omega = 0.0;

    // The drive base turns this speed into voltage, and it keeps yaw on the rail.
    CommandVector out;
    out.request = DriveRequest::Motion;
    out.surge = v_cmd;
    out.sway = 0.0;
    out.yaw = omega;
    out.pivot = std::fabs(v_cmd) <= GoalAlign::VEL_DEADBAND
             && std::fabs(omega) > GoalAlign::VEL_DEADBAND;
    return out;
}

// Nearest goal whose point lies on the front edge. Pose is meters.
int frontTouchMeters(double x_m, double y_m, double theta) {
    const int n = static_cast<int>(Field::Point::Count);
    int best = -1;
    double best_gap = 0.0;
    for (int i = 0; i < n; ++i) {
        const Field::Spot& spot = Field::kSpots[i];
        if (!Field::isAlignGoal(spot)) continue;
        double contact = 0.0;
        const double gx = inchesToMeters(spot.x);
        const double gy = inchesToMeters(spot.y);
        if (!frontedge::touches(x_m, y_m, theta, gx, gy, contact)) continue;
        const double gap = std::hypot(gx - x_m, gy - y_m);
        if (best >= 0 && gap >= best_gap) continue;
        best = i;
        best_gap = gap;
    }
    return best;
}

// Nearest goal whose point lies on the front edge. Pose is field inches.
int frontTouch(const Pose2d& pose_in) {
    const Pose2d pose = poseInchesToMeters(pose_in);
    return frontTouchMeters(pose.x, pose.y, pose.theta);
}

// The yaw stick is the orbit rate about the goal. Square-up lives in the
// plant, so a pose the localizer has not caught up with cannot hold the stick.
// The mapper has already applied the free-drive turn scale. That scale is
// undone here so full deflection is the orbit cap, not a fraction of it.
CommandVector holdCommand(const CommandVector& commands, const IFeedForward* model) {
    // The mapper already scaled the stick onto free yaw speed, including
    // TURN_STICK_SCALE. Undo that scale so full deflection is the orbit cap.
    double yaw = commands.yaw;
    if (Tuning::TURN_STICK_SCALE > 1e-3) yaw /= Tuning::TURN_STICK_SCALE;
    yaw = mathutil::clamp_val(yaw, -GoalAlign::MAX_YAW_RAD_S, GoalAlign::MAX_YAW_RAD_S);
    return voltsFor(0.0, yaw, model);
}

} // namespace

void goalAlignRelease() {
    g_latched = -1;
    g_suppress = true;
}

void goalAlignHoldOff() {
    g_latched = -1;
    g_suppress = true;
    g_hold_off = true;
}

void goalAlignClearHoldOff() {
    g_hold_off = false;
    g_suppress = false;
}

void goalAlignSetEnabled(bool enabled) {
    g_enabled = enabled;
    if (!enabled) {
        g_latched = -1;
        g_suppress = false;
        g_hold_off = false;
    }
}

void goalAlignPhysicsDone() {
    if (!g_hold_off) g_suppress = false;
}

int goalAlignIndex() {
    return g_latched;
}

void goalAlignPivot(const Pose2d& pose_in, double& gx, double& gy) {
    const int n = static_cast<int>(Field::Point::Count);
    if (g_latched >= 0 && g_latched < n && Field::isAlignGoal(Field::kSpots[g_latched])) {
        gx = inchesToMeters(Field::kSpots[g_latched].x);
        gy = inchesToMeters(Field::kSpots[g_latched].y);
        return;
    }

    const Pose2d pose = poseInchesToMeters(pose_in);
    const double half_l = Robot::Chassis::BODY_LENGTH_M * 0.5;
    const double fx = pose.x + half_l * mathutil::heading_cos(pose.theta);
    const double fy = pose.y + half_l * mathutil::heading_sin(pose.theta);
    int best = -1;
    double best_d = 0.0;
    for (int i = 0; i < n; ++i) {
        const Field::Spot& spot = Field::kSpots[i];
        if (!Field::isAlignGoal(spot)) continue;
        const double sx = inchesToMeters(spot.x);
        const double sy = inchesToMeters(spot.y);
        const double d = std::hypot(sx - fx, sy - fy);
        if (best >= 0 && d >= best_d) continue;
        best = i;
        best_d = d;
    }
    if (best >= 0 && best_d <= GoalBump::LOOKAHEAD_M) {
        gx = inchesToMeters(Field::kSpots[best].x);
        gy = inchesToMeters(Field::kSpots[best].y);
        return;
    }
    gx = fx;
    gy = fy;
}

bool goalAlignGrab(double x_m, double y_m, double theta, double forward_mps,
                   double& gx, double& gy) {
    if (!g_enabled || g_suppress) return false;
    if (forward_mps < -GoalAlign::VEL_DEADBAND) {
        goalAlignRelease();
        return false;
    }

    const int n = static_cast<int>(Field::Point::Count);
    if (g_latched >= 0 && (g_latched >= n || !Field::isAlignGoal(Field::kSpots[g_latched]))) {
        g_latched = -1;
    }
    if (g_latched < 0) g_latched = frontTouchMeters(x_m, y_m, theta);
    if (g_latched < 0) return false;

    const Field::Spot& spot = Field::kSpots[g_latched];
    gx = inchesToMeters(spot.x);
    gy = inchesToMeters(spot.y);
    return true;
}

CommandVector goalAlignOverride(const CommandVector& commands,
                                const Pose2d& body,
                                const JoystickData& raw_sticks,
                                const IFeedForward* model) {
    if (!g_enabled) return commands;
    // Reverse backs off the point and gives the sticks back.
    if (raw_sticks.left_y < -Controls::STICK_ZERO_DEADBAND) {
        goalAlignRelease();
        return commands;
    }

    const int n = static_cast<int>(Field::Point::Count);
    if (g_latched >= 0 && (g_latched >= n || !Field::isAlignGoal(Field::kSpots[g_latched]))) {
        g_latched = -1;
    }

    if (g_latched < 0) {
        g_latched = frontTouch(body);
        if (g_latched < 0) return commands;
    }
    return holdCommand(commands, model);
}

int goalAlignPath(const Pose2d& from, Pose2d* out, int cap) {
    if (out == nullptr || cap < 2 || g_latched < 0) return 0;
    const int nspots = static_cast<int>(Field::Point::Count);
    if (g_latched >= nspots) return 0;
    const Field::Spot& spot = Field::kSpots[g_latched];
    if (!Field::isAlignGoal(spot)) return 0;

    const double gx = inchesToMeters(spot.x);
    const double gy = inchesToMeters(spot.y);
    const Pose2d pose = poseInchesToMeters(from);
    const double dist = std::hypot(gx - pose.x, gy - pose.y);
    const double theta_face = (dist > 1e-6)
        ? mathutil::direction_heading(gx - pose.x, gy - pose.y) : pose.theta;
    const int n = (cap < 17) ? cap : 17;
    for (int i = 0; i < n; ++i) {
        const double u = (n == 1) ? 0.0 : static_cast<double>(i) / static_cast<double>(n - 1);
        const double th = theta_face + (u - 0.5) * Robot::PI;
        double x = 0.0;
        double y = 0.0;
        frontedge::placeCenter(x, y, th, gx, gy, 0.0);
        out[i] = Pose2d{metersToInches(x), metersToInches(y), th};
    }
    return n;
}
