#pragma once
/* ---------------------------------------------------------------------------
 * OdometryEstimator  --  the default state estimator: pure trapezoidal
 * dead-reckoning, exactly matching v1's odometry behavior. Pose comes from
 * integrating wheel travel along the fused heading; velocity is passed through
 * from the drivebase's measured chassis velocities.
 * ------------------------------------------------------------------------- */

#include "interfaces/IEstimator.h"
#include "estimators/Odometry.h"
#include "math/MathUtils.h"

class OdometryEstimator : public IEstimator {
public:
    void reset() override {
        odom_.setPose(Pose2d{});
        total_heading_ = 0.0;
        prev_theta_ = 0.0;
        v_lin_ = v_ang_ = 0.0;
    }

    void overridePose(const Pose2d& pose) override {
        odom_.setPose(pose);
        prev_theta_ = pose.theta;   // keep display delta continuous
    }

    void update(const ChassisObservation& obs) override {
        odom_.update(obs.forward_delta, obs.strafe_delta, obs.heading_rad);
        // Unwrapped heading accumulator (display only).
        double d = mathutil::normalize_angle(odom_.pose.theta - prev_theta_);
        total_heading_ += d;
        prev_theta_ = odom_.pose.theta;
        v_lin_ = obs.v_lin;
        v_ang_ = obs.v_ang;
    }

    StateEstimate getState() const override {
        StateEstimate s;
        s.pose = odom_.pose;
        s.v_lin = v_lin_;
        s.v_ang = v_ang_;
        return s;
    }

    double getTotalHeading() const override { return total_heading_; }

private:
    Odometry odom_;
    double total_heading_ = 0.0;
    double prev_theta_ = 0.0;
    double v_lin_ = 0.0, v_ang_ = 0.0;
};
