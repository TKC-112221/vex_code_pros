/* ---------------------------------------------------------------------------
 * LocalizationOverride.cpp  --  wall, bumper snap, and the goal orbit.
 * ------------------------------------------------------------------------- */
#include "estimators/LocalizationOverride.h"

#include "core/Specs.h"
#include "core/Tuning.h"
#include "field/FieldPoints.h"
#include "logic/GoalAligner.h"
#include "math/ContactPhysics.h"
#include "math/FrontEdge.h"
#include "math/MathUtils.h"
#include "math/TrackingCenter.h"

#include <cmath>
#include <cstdint>

namespace {

std::uint32_t g_goal_bump_ms = 0;

// Body-frame travel that carries pose_m onto (x, y) at chassis.heading_rad.
void travelOnto(ChassisObservation& chassis, const Pose2d& pose_m, double x, double y) {
    const double d_theta = mathutil::normalize_angle(chassis.heading_rad - pose_m.theta);
    const double mid = pose_m.theta + d_theta * 0.5;
    const double c = mathutil::heading_cos(mid);
    const double s = mathutil::heading_sin(mid);
    const double dx = x - pose_m.x;
    const double dy = y - pose_m.y;
    chassis.forward_delta = dx * c + dy * s;
    chassis.strafe_delta = -dx * s + dy * c;
}

void holdWall(ImuLinearDelta& imu, ChassisObservation& chassis, const Pose2d& pose_m) {
    if (!std::isfinite(pose_m.x) || !std::isfinite(pose_m.y) || !std::isfinite(pose_m.theta))
        return;
    if (!std::isfinite(chassis.heading_rad) || !std::isfinite(chassis.forward_delta)
        || !std::isfinite(chassis.strafe_delta))
        return;

    const Pose2d center = tracking::centerFromTracking(pose_m);
    const double d_theta = mathutil::normalize_angle(chassis.heading_rad - center.theta);
    const double mid = center.theta + d_theta * 0.5;
    const double c = mathutil::heading_cos(mid);
    const double s = mathutil::heading_sin(mid);

    ContactBody body;
    body.x_m = center.x + chassis.forward_delta * c - chassis.strafe_delta * s;
    body.y_m = center.y + chassis.forward_delta * s + chassis.strafe_delta * c;
    body.theta = chassis.heading_rad;
    body.v_fwd = imu.forwardVelocity();
    body.v_lat = imu.lateralVelocity();
    body.omega = std::isfinite(chassis.v_ang) ? chassis.v_ang : 0.0;
    holdWallContact(body);

    travelOnto(chassis, center, body.x_m, body.y_m);
    imu.setLinear(body.v_fwd, body.v_lat, chassis);
}

bool seatFrontOnGoal(double x_m, double y_m, double heading, Pose2d& seated_in) {
    if (!std::isfinite(x_m) || !std::isfinite(y_m) || !std::isfinite(heading)) return false;

    // x_m, y_m is the front middle. A goal ahead of that edge has positive
    // forward; one behind the bumper by more than BEHIND_M is ignored.
    const double half_w = Robot::Chassis::TRACK_WIDTH_M * 0.5;
    const double c = mathutil::heading_cos(heading);
    const double s = mathutil::heading_sin(heading);
    const double fwd_min = -GoalBump::BEHIND_M;

    int best = -1;
    double best_gap = 0.0;
    const int n = static_cast<int>(Field::Point::Count);
    for (int i = 0; i < n; ++i) {
        const Field::Spot& spot = Field::kSpots[i];
        if (!Field::isAlignGoal(spot)) continue;
        const double gx = inchesToMeters(spot.x);
        const double gy = inchesToMeters(spot.y);
        const double dx = gx - x_m;
        const double dy = gy - y_m;
        const double forward = dx * c + dy * s;
        const double left = -dx * s + dy * c;
        if (forward < fwd_min) continue;
        const double clamped = mathutil::clamp_val(left, -half_w, half_w);
        const double gap = std::hypot(forward, left - clamped);
        if (gap > GoalBump::LOOKAHEAD_M) continue;
        if (best >= 0 && gap >= best_gap) continue;
        best = i;
        best_gap = gap;
    }
    if (best < 0) return false;

    const Field::Spot& spot = Field::kSpots[best];
    const double gx = inchesToMeters(spot.x);
    const double gy = inchesToMeters(spot.y);
    if (std::hypot(gx - x_m, gy - y_m) <= GoalBump::SEATED_M) return false;

    seated_in = poseMetersToInches(Pose2d{gx, gy, heading});
    return true;
}

// Latched goal. The front stays on the point. The center rides the circle
// of radius half a body length, and the only free motion is yaw. Returns
// false when nothing is latched.
bool holdGoalOrbit(ImuLinearDelta& imu, ChassisObservation& chassis, const Pose2d& pose_m) {
    const int idx = goalAlignIndex();
    const int n = static_cast<int>(Field::Point::Count);
    if (idx < 0 || idx >= n) return false;
    const Field::Spot& spot = Field::kSpots[idx];
    if (!Field::isAlignGoal(spot)) return false;
    if (!std::isfinite(chassis.heading_rad) || !std::isfinite(pose_m.x) || !std::isfinite(pose_m.y))
        return false;

    const double radius = Robot::Chassis::TRACKING_FORWARD_M;
    const double gx = inchesToMeters(spot.x);
    const double gy = inchesToMeters(spot.y);
    const Pose2d center = tracking::centerFromTracking(pose_m);
    double x = 0.0;
    double y = 0.0;
    frontedge::placeCenter(x, y, chassis.heading_rad, gx, gy, 0.0);
    travelOnto(chassis, center, x, y);

    const double yaw = std::isfinite(chassis.v_ang) ? chassis.v_ang : 0.0;
    // Rigid orbit: no speed through the goal, lateral speed is yaw times
    // the radius. Positive yaw swings the center to the robot's right.
    imu.setLinear(0.0, -yaw * radius, chassis);
    return true;
}

} // namespace

LocalizationOverride applyLocalizationOverride(ImuLinearDelta& imu,
                                          ChassisObservation& chassis,
                                          const Pose2d& pose_m,
                                          const Pose2d& carried_m,
                                          const LocalizationCue& cue,
                                          std::uint32_t now_ms) {
    LocalizationOverride out;
    if (holdGoalOrbit(imu, chassis, pose_m)) {
        out.replace = true;
        return out;
    }

    holdWall(imu, chassis, carried_m);

    const bool cooled = g_goal_bump_ms == 0
        || (now_ms - g_goal_bump_ms) >= GoalBump::REFRACTORY_MS;
    Pose2d seated;
    if (cooled
        && cue.front_speed > GoalBump::APPROACH_MPS
        && std::isfinite(cue.accel_g) && cue.accel_g >= GoalBump::DECEL_G
        && std::isfinite(cue.ax_g) && cue.ax_g < GoalBump::LAUNCH_G
        && std::isfinite(cue.surprise_g) && cue.surprise_g >= GoalBump::SURPRISE_G
        && seatFrontOnGoal(carried_m.x, carried_m.y, chassis.heading_rad, seated)) {
        imu.setLinear(0.0, 0.0, chassis);
        chassis.forward_delta = 0.0;
        chassis.strafe_delta = 0.0;
        out.snap = true;
        out.seated_in = seated;
        g_goal_bump_ms = now_ms;
    }
    return out;
}
