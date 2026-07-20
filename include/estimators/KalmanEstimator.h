#pragma once
/* ---------------------------------------------------------------------------
 * KalmanEstimator  --  alternative estimator that smooths the dead-reckoned
 * pose and velocity through the 9-state EKF. Provided to demonstrate that the
 * estimation stage is a swappable strategy (just construct this instead of
 * OdometryEstimator in main). Not the v1 default.
 * ------------------------------------------------------------------------- */

#include "interfaces/IEstimator.h"
#include "estimators/Odometry.h"
#include "math/ExtendedKalmanFilter.h"
#include "core/Specs.h"

class KalmanEstimator : public IEstimator {
public:
    void initialize() override {
        ekf_.init(System::DT_SEC,
                  /*q_pos*/1e-5, /*q_vel*/1e-4, /*q_acc*/0.1,
                  /*r_pos*/0.002 * 0.002, /*r_vel*/0.04 * 0.04, /*r_acc_cmd*/2.0 * 2.0);
    }

    void reset() override {
        odom_.setPose(Pose2d{});
        ekf_.reset();
        total_heading_ = 0.0;
        prev_theta_ = 0.0;
    }

    void overridePose(const Pose2d& pose) override {
        odom_.setPose(pose);
        ekf_.reset(pose.x, pose.y, pose.theta, 0.0, 0.0);
        prev_theta_ = pose.theta;
    }

    void update(const ChassisObservation& obs) override {
        odom_.update(obs.forward_delta, obs.strafe_delta, obs.heading_rad);
        ekf_.predict(System::DT_SEC, 0.0, 0.0);
        ekf_.update_pose(odom_.pose.x, odom_.pose.y, odom_.pose.theta);
        ekf_.update_velocity(obs.v_lin, obs.v_ang);
        double d = mathutil::normalize_angle(odom_.pose.theta - prev_theta_);
        total_heading_ += d;
        prev_theta_ = odom_.pose.theta;
    }

    StateEstimate getState() const override {
        StateEstimate s;
        double px, py, pth; ekf_.get_pose_est(px, py, pth);
        s.pose = Pose2d{px, py, pth};
        s.v_lin = ekf_.get_v_est();
        s.v_ang = ekf_.get_omega_est();
        return s;
    }

    double getTotalHeading() const override { return total_heading_; }

private:
    Odometry odom_;
    ExtendedKalmanFilter ekf_;
    double total_heading_ = 0.0;
    double prev_theta_ = 0.0;
};
