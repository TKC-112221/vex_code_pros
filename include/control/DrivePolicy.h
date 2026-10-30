#pragma once
/* ---------------------------------------------------------------------------
 * DrivePolicy  --  a small actor that follows a path's thrust and heading.
 *
 * Output is CommandVector. This chassis is differential: the actor
 * commands surge and yaw only, and sway is left at zero. command()
 * applies tanh, then scales that fraction onto the chassis free speed.
 * The drive base turns the velocity into volts with its VAS model.
 *
 * The actor is features -> 64 -> 64 -> 2, SiLU hidden, linear output.
 * init() copies include/control/DrivePolicyWeights.h. Re-export after
 * training; a table from the 32-wide ReLU net does not load.
 *
 * Features, all dimensionless and clamped to [-1, 1]. Six history frames,
 * newest first, at the lags in kHistoryLag (10 ms ticks). Each frame holds:
 *   0  forward speed / MAX_VEL_LIN
 *   1  lateral speed / MAX_VEL_LIN. Left is positive
 *   2  yaw rate / MAX_VEL_ANG
 *   3  previous surge command, in [-1, 1]
 *   4  previous yaw command, in [-1, 1]
 * Speeds are m/s and rad/s on the way in. The commands are the normalized
 * actions, not volts. The commands are already their own channels, so the
 * speeds are not pre-subtracted from them.
 *
 * After the six frames:
 *   sin and cos of (path tangent at 0.2 m − chassis heading). The heading
 *   is the body, not the velocity direction, so it stays defined at rest.
 *   body-frame (x, y) of the path at 0.2, 0.5, 1.0, and 1.5 m ahead,
 *   each divided by its own station distance. +x is forward, +y is left
 *   target thrust in [0, 1]
 *   tanh(signed cross-track / kCrossUnitIn). Left of travel is positive
 * ------------------------------------------------------------------------- */

#include "control/DriftCurve.h"
#include "core/Types.h"
#include "core/Tuning.h"

namespace DrivePolicy {

constexpr int kHistory = 6;
constexpr int kHistoryChannels = 5;
constexpr int kHistSurge = 0;
constexpr int kHistSway = 1;
constexpr int kHistYaw = 2;
constexpr int kHistSurgeCmd = 3;
constexpr int kHistYawCmd = 4;
// Ticks of age for the six frames. The loop is 10 ms, so the last tap is
// 300 ms: one velocity lag and the nominal skid lag, with the newest
// three samples still a tick apart.
constexpr int kHistoryLag[kHistory] = {0, 1, 2, 5, 12, 30};
constexpr int kHistoryDepth = kHistoryLag[kHistory - 1] + 1;

constexpr int kLookahead = 4;
constexpr double kLookaheadM[kLookahead] = {0.2, 0.5, 1.0, 1.5};
// Farthest station. Each point is scaled by its own kLookaheadM entry.
constexpr double kLookNormM = 1.5;

constexpr int kHistoryFeatures = kHistory * kHistoryChannels;
constexpr int kDirSin = kHistoryFeatures;
constexpr int kDirCos = kDirSin + 1;
constexpr int kLookStart = kDirCos + 1;
constexpr int kTargetThrust = kLookStart + kLookahead * 2;
constexpr int kCrossTrack = kTargetThrust + 1;
constexpr int kFeatures = kCrossTrack + 1;
constexpr int kHidden = 64;
constexpr int kOutputs = 2;
constexpr int kLayerCount = 4;
constexpr int kLayerSizes[kLayerCount] = {kFeatures, kHidden, kHidden, kOutputs};

// Scales and the settle window are DrivePolicy:: in core/Tuning.h.

// One tick of read(). History index 0 is the current tick, and index i is
// kHistoryLag[i] ticks old. v_* are body frame: surge and sway in m/s
// (forward, left), yaw in rad/s. a_* are the normalized surge and yaw
// commands in [-1, 1] that led into that tick. look_* are meters, body
// frame, at kLookaheadM. path_phi is the travel tangent at the first
// station. body_heading is the chassis nose, not the velocity direction.
struct FollowInput {
    double v_surge[kHistory] {};
    double v_sway[kHistory] {};
    double v_yaw[kHistory] {};
    double a_surge[kHistory] {};
    double a_yaw[kHistory] {};
    double path_phi = 0;
    double body_heading = 0;
    double look_x_m[kLookahead] {};
    double look_y_m[kLookahead] {};
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

// Drop the pose-override history. The next command() starts from rest.
void reset();

Reading read(const FollowInput& in);

// Zero command until a weight table is assigned. Does not touch the
// pose-override history.
CommandVector command(const FollowInput& in);

// Pose override. Records this tick in the history, samples the Hermite
// fit of `target` at kLookaheadM, and commands surge and yaw. Thrust
// stays at full scale: this target has no clock schedule. Lateral speed
// is zero; StateEstimate does not carry it.
CommandVector command(const StateEstimate& state, const DriftCurveTarget& target);

}
