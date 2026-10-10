#pragma once
/* ---------------------------------------------------------------------------
 * DrivePolicy  --  a small actor that follows a path's thrust and heading.
 *
 * Output is CommandVector, the universal chassis command in volts:
 *   surge = vx,  sway = vy,  yaw = omega.
 * applyDrive consumes that vector unchanged. On this differential chassis
 * sway is unused by the plant; the action is still there so a holonomic
 * mapper can take the same network.
 *
 * The actor is features -> 32 -> 32 -> 3, ReLU hidden, linear output.
 * command() applies tanh and scales to ±System::MAX_VOLTS, which is the
 * deterministic soft-actor-critic action. init() copies
 * include/control/DrivePolicyWeights.h. The pose-task table does not match
 * this input; re-export after training.
 *
 * Features, in order, all dimensionless. The direction is the path tangent
 * a short distance ahead, minus the direction the robot is actually moving
 * (its nose, when it is stopped). The two derivatives are of that path
 * tangent with respect to arc length, so a straight reads as zero, zero.
 *   0  previous surge command, in [-1, 1]
 *   1  previous sway command
 *   2  previous yaw command
 *   3  sin(path tangent − velocity heading)
 *   4  cos(path tangent − velocity heading)
 *   5  dφ/ds / kKappaUnitPerIn. φ is the path tangent, s is inches
 *   6  d²φ/ds² / kKappaRateUnit
 *   7  target thrust in [0, 1]. 1 is full throttle
 *   8  signed cross-track / kCrossUnitIn. Left of travel is positive
 * Target thrust and cross-track are not derivatives of the direction. The
 * thrust schedule is independent of the shape, and the cross-track is the
 * only signal that says which way to steer back after a drift.
 * ------------------------------------------------------------------------- */

#include "control/DriftCurve.h"
#include "core/Types.h"
#include "core/Tuning.h"

namespace DrivePolicy {

constexpr int kPrevSurge = 0;
constexpr int kPrevSway = 1;
constexpr int kPrevYaw = 2;
constexpr int kDirSin = 3;
constexpr int kDirCos = 4;
constexpr int kDPhiDs = 5;
constexpr int kD2PhiDs2 = 6;
constexpr int kTargetThrust = 7;
constexpr int kCrossTrack = 8;
constexpr int kFeatures = 9;
constexpr int kHidden = 32;
constexpr int kOutputs = 3;
constexpr int kLayerCount = 4;
constexpr int kLayerSizes[kLayerCount] = {kFeatures, kHidden, kHidden, kOutputs};

// Scales and the settle window are DrivePolicy:: in core/Tuning.h.

// Everything read() needs for one tick. Angles are radians. dphi_ds is
// rad per inch and d2phi_ds2 is rad per inch². Commands and target_thrust
// are normalized, not volts.
struct FollowInput {
    double prev_surge = 0;
    double prev_sway = 0;
    double prev_yaw = 0;
    double path_phi = 0;
    double vel_heading = 0;
    double dphi_ds = 0;
    double d2phi_ds2 = 0;
    double target_thrust = 1;
    double cross_track_in = 0;
};

struct Reading {
    float features[kFeatures] {};
    double distance_in = 0;
    double travel_mps = 0;
    double heading_error_rad = 0;
    bool at_target = false;
};

// Copy the compiled weight table. Safe to call more than once.
void init();

// True after a full weight table has been assigned.
bool ready();

// count must equal the actor's weight count. False leaves the old table.
bool assignWeights(const float* weights, int count);

Reading read(const FollowInput& in);

// Zero command until a weight table is assigned.
CommandVector command(const FollowInput& in);

// The pose override still calls this. It has no path, so the curvature
// features are zero and the thrust target is full throttle. That is not
// the observation the path policy is trained on.
CommandVector command(const StateEstimate& state, const DriftCurveTarget& target);

}
