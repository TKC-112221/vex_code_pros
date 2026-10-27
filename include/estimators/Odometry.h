#pragma once
/* ---------------------------------------------------------------------------
 * Odometry  --  trapezoidal dead-reckoning integrator (pure math helper).
 * Advances a Pose2d given a forward distance increment and an absolute heading,
 * using the midpoint heading to reduce integration drift. Ported from v1's
 * Odometry::update. Shared by OdometryEstimator and KalmanEstimator.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "core/Specs.h"
#include "math/MathUtils.h"
#include <cmath>

class Odometry {
public:
    // Meters. OdometryEstimator converts to inches when it publishes.
    // The default pose is the geometric center of the field.
    Pose2d pose{Robot::Range::CENTER_X_M, Robot::Range::CENTER_Y_M, 0.0};

    // forward_delta: meters travelled along heading since last call
    // strafe_delta : meters travelled sideways (left +); 0 for a differential drive
    // heading_rad  : new absolute heading
    void update(double forward_delta, double strafe_delta, double heading_rad) {
        double d_theta = mathutil::normalize_angle(heading_rad - pose.theta);
        double theta_mid = pose.theta + d_theta / 2.0;
        pose.x += forward_delta * std::cos(theta_mid) - strafe_delta * std::sin(theta_mid);
        pose.y += forward_delta * std::sin(theta_mid) + strafe_delta * std::cos(theta_mid);
        pose.theta = heading_rad;
    }

    // Differential-drive convenience (no lateral motion).
    void update(double forward_delta, double heading_rad) {
        update(forward_delta, 0.0, heading_rad);
    }

    void setPose(const Pose2d& p) { pose = p; }
};
