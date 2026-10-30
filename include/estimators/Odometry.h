#pragma once
/* ---------------------------------------------------------------------------
 * Odometry  --  trapezoidal dead-reckoning integrator (pure math helper).
 * Advances the tracking point (the front middle) from the geometric
 * center's travel and an absolute heading. The midpoint heading reduces
 * integration drift. Shared by OdometryEstimator and KalmanEstimator.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "core/Specs.h"
#include "math/MathUtils.h"
#include "math/TrackingCenter.h"

class Odometry {
public:
    // Meters. OdometryEstimator converts to inches when it publishes.
    // The default is the front of a robot whose center is the field center.
    Pose2d pose = tracking::origin();

    // forward_delta: meters the geometric center travelled along heading
    // strafe_delta : meters the center travelled sideways (left +)
    // heading_rad  : new absolute heading
    void update(double forward_delta, double strafe_delta, double heading_rad) {
        const double theta0 = pose.theta;
        const double d_theta = mathutil::normalize_angle(heading_rad - theta0);
        const double theta_mid = theta0 + d_theta / 2.0;
        tracking::integrate(pose.x, pose.y, theta0, heading_rad,
                            forward_delta, strafe_delta, theta_mid);
        pose.theta = heading_rad;
    }

    // Differential-drive convenience (no lateral motion).
    void update(double forward_delta, double heading_rad) {
        update(forward_delta, 0.0, heading_rad);
    }

    void setPose(const Pose2d& p) { pose = p; }
};
