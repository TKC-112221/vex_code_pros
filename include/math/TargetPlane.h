#pragma once
/* ---------------------------------------------------------------------------
 * TargetPlane  --  has the robot crossed the line through a pose.
 *
 * The plane is the line through `plane` perpendicular to plane.theta.
 * Positive x in that frame is the direction of travel through the plane.
 * A crossing is behind-to-ahead, with world velocity pointing the same way.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "math/MathUtils.h"
#include <cmath>

namespace mathutil {

struct Vec2 {
    double x = 0;
    double y = 0;
};

// Row-major 2x2. v' = M v.
struct Mat2 {
    double m00 = 1, m01 = 0;
    double m10 = 0, m11 = 1;
};

inline Vec2 mul(const Mat2& m, const Vec2& v) {
    return { m.m00 * v.x + m.m01 * v.y,
             m.m10 * v.x + m.m11 * v.y };
}

// World vector -> frame whose x-axis points along `heading`.
inline Mat2 worldToFrame(double heading) {
    const double c = heading_cos(heading);
    const double s = heading_sin(heading);
    return { c, s, -s, c };
}

// Signed distance of `pose` ahead of the plane. Negative is still behind it.
inline double targetPlaneAlong(const Pose2d& pose, const Pose2d& plane) {
    const Mat2 frame = worldToFrame(plane.theta);
    return mul(frame, Vec2{pose.x - plane.x, pose.y - plane.y}).x;
}

// Nose speed (m/s) projected onto the plane normal. Positive moves through it.
inline double targetPlaneNoseSpeed(const Pose2d& pose, double v_lin, const Pose2d& plane) {
    const Mat2 frame = worldToFrame(plane.theta);
    const Vec2 velocity = mul(frame, Vec2{v_lin * heading_cos(pose.theta),
                                          v_lin * heading_sin(pose.theta)});
    return velocity.x;
}

// True when `current` has passed through the plane and `v_lin` (m/s along
// the robot's nose, at `current.theta`) points along the plane normal.
bool crossedTargetPlane(const Pose2d& previous, const Pose2d& current,
                        double v_lin, const Pose2d& plane);

} // namespace mathutil
