#pragma once
/* ---------------------------------------------------------------------------
 * WallRange  --  expected VEX distance-sensor readings against the field wall.
 *
 * The field is an axis-aligned square centered on the origin (Robot::Range).
 * Each beam is Robot::Range::MOUNT in the chassis frame. predictWallRange is
 * the measurement model shared by the particle filter, the EKF, and MockDrive.
 * solveRanges recovers x/y from ranges when heading is already known.
 * nudgePosition moves an existing x/y a fraction of the way toward those
 * ranges and leaves heading alone.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"

struct WallHit {
    bool hit = false;
    double range_m = 0;   // predicted beam length to the wall
    double hx = 0;        // d(range) / d(x)
    double hy = 0;        // d(range) / d(y)
    double htheta = 0;    // d(range) / d(theta)
};

struct RangeSolve {
    bool ok = false;
    double x = 0;
    double y = 0;
    double mae_m = 0;     // mean absolute range residual of the beams used
};

namespace wallrange {

WallHit predictWallRange(const Pose2d& pose, int sensor_index);

// Beam length only. cos_th/sin_th are the chassis heading; cos_yaw/sin_yaw are
// the mount yaw, so a caller can rotate every particle with one sin/cos pair.
// Returns a negative number when the ray misses the field perimeter.
double beamRange(double x, double y, double cos_th, double sin_th,
                 double mount_x, double mount_y, double cos_yaw, double sin_yaw);
// Single-precision ray used by the particle filter. Same geometry as the
// double overload; the cloud's inner loop stays in float.
float beamRange(float x, float y, float cos_th, float sin_th,
                float mount_x, float mount_y, float cos_yaw, float sin_yaw);

RangeSolve solveRanges(double theta, const DistanceReading* distance, int count);

// One damped x/y step toward the wall ranges at a fixed heading. gain is the
// fraction of the least-squares step to apply; step_m caps it. A beam that
// disagrees with the others is left out. Returns false when no step is taken.
bool nudgePosition(double& x, double& y, double theta,
                   const DistanceReading* distance, int count,
                   double gain, double step_m, double gate_m);

}
