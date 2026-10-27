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
// step. goalAlignPhysicsDone clears it once the grab has finished.
bool g_suppress = false;
// The firmware keeps the grab. A scoring rollout turns it off.
bool g_enabled = true;

CommandVector voltsFor(double v_cmd, double omega, const IFeedForward* model) {
    if (std::fabs(v_cmd) < GoalAlign::VEL_DEADBAND) v_cmd = 0.0;
    if (std::fabs(omega) < GoalAlign::VEL_DEADBAND) omega = 0.0;

    double turn_r = Physics::TURN_RADIUS_MAX_M;
    if (std::fabs(omega) > Physics::OMEGA_EPS) {
        turn_r = std::fabs(v_cmd) / std::fabs(omega);
        if (turn_r > Physics::TURN_RADIUS_MAX_M) turn_r = Physics::TURN_RADIUS_MAX_M;
    }

    double surge;
    double yaw;
    if (model != nullptr) {
        surge = model->linearVolts(v_cmd, 0.0, std::fabs(v_cmd), 0.0, omega, turn_r);
        // angularVolts is V_R - V_L. The chassis command is half of that.
        yaw = model->angularVolts(omega, 0.0, std::fabs(v_cmd), 0.0, turn_r) / 2.0;
    } else {
        surge = Feedforward::KV_LIN * v_cmd;
        yaw = Feedforward::KV_ANG * omega / 2.0;
    }

    // Keep the turn when surge and yaw together would clip a side.
    const double room = System::MAX_VOLTS - std::fabs(yaw);
    if (room <= 0.0) {
        surge = 0.0;
        yaw = std::copysign(System::MAX_VOLTS, yaw);
    } else if (std::fabs(surge) > room) {
        surge = std::copysign(room, surge);
    }

    CommandVector out;
    out.surge = surge;
    out.sway = 0.0;
    out.yaw = yaw;
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
CommandVector holdCommand(const CommandVector& commands, const IFeedForward* model) {
    double yaw_frac = commands.yaw / System::MAX_VOLTS;
    yaw_frac = mathutil::clamp_val(yaw_frac, -1.0, 1.0);
    return voltsFor(0.0, yaw_frac * GoalAlign::MAX_YAW_RAD_S, model);
}

} // namespace

void goalAlignRelease() {
    g_latched = -1;
    g_suppress = true;
}

void goalAlignSetEnabled(bool enabled) {
    g_enabled = enabled;
    if (!enabled) {
        g_latched = -1;
        g_suppress = false;
    }
}

void goalAlignPhysicsDone() {
    g_suppress = false;
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
    const double fx = pose.x + half_l * std::cos(pose.theta);
    const double fy = pose.y + half_l * std::sin(pose.theta);
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
    const double theta_face = (dist > 1e-6) ? std::atan2(gy - pose.y, gx - pose.x) : pose.theta;
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
