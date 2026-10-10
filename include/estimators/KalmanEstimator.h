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
#include "math/WallRange.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include <cmath>

class KalmanEstimator : public IEstimator {
public:
    void initialize() override {
        ekf_.init(System::DT_SEC,
                  Ekf::Q_POS, Ekf::Q_VEL, Ekf::Q_ACC,
                  Ekf::R_POS, Ekf::R_VEL, Ekf::R_ACC_CMD);
    }

    void reset() override {
        odom_.setPose(Pose2d{});
        ekf_.reset();
        total_heading_ = 0.0;
        prev_theta_ = 0.0;
        range_phase_ = 0;
    }

    void overridePose(const Pose2d& pose) override {
        const Pose2d stored = poseInchesToMeters(pose);
        odom_.setPose(stored);
        ekf_.reset(stored.x, stored.y, stored.theta, 0.0, 0.0);
        prev_theta_ = stored.theta;
        range_phase_ = 0;
    }

    void update(const SensorDict& sensors) override {
        const ChassisObservation& obs = sensors.chassis;
        odom_.update(obs.forward_delta, obs.strafe_delta, obs.heading_rad);
        ekf_.predict(System::DT_SEC, 0.0, 0.0);
        ekf_.update_pose(odom_.pose.x, odom_.pose.y, odom_.pose.theta);
        ekf_.update_velocity(obs.v_lin, obs.v_ang);
        // Once per distance-sensor period. A 100 Hz update would triple-count
        // a 33 ms range and spend the wall solve on repeated samples.
        bool ranges_moved = false;
        if (range_phase_ == 0) ranges_moved = fuseRanges(sensors.distance);
        range_phase_ = (range_phase_ + 1) % System::RANGE_STRIDE;
        if (ranges_moved) {
            double px, py, pth;
            ekf_.get_pose_est(px, py, pth);
            odom_.setPose(Pose2d{px, py, pth});
        }
        double d = mathutil::normalize_angle(odom_.pose.theta - prev_theta_);
        total_heading_ += d;
        prev_theta_ = odom_.pose.theta;
    }

    StateEstimate getState() const override {
        StateEstimate s;
        double px, py, pth; ekf_.get_pose_est(px, py, pth);
        s.pose = poseMetersToInches(Pose2d{px, py, pth});
        s.v_lin = ekf_.get_v_est();
        s.v_ang = ekf_.get_omega_est();
        return s;
    }

    double getTotalHeading() const override { return total_heading_; }

private:
    // Wall ranges. A self-consistent fix far from the prediction is applied as
    // an x/y update. Otherwise each beam is a gated scalar measurement.
    // Returns true when the pose was moved, so the caller can keep odometry
    // from pulling the correction back on the next tick.
    bool fuseRanges(const DistanceReading* distance) {
        double px, py, pth;
        ekf_.get_pose_est(px, py, pth);
        double var_acc = 0.0;
        int nvar = 0;
        for (int i = 0; i < Robot::Range::COUNT; ++i) {
            if (!distance[i].valid) continue;
            const double s = Robot::rangeSigmaM(distance[i].range_m, distance[i].confidence);
            var_acc += s * s;
            ++nvar;
        }
        const double fallback = Robot::rangeAccuracyM(1.0);
        const double r = (nvar > 0) ? (var_acc / static_cast<double>(nvar))
                                    : (fallback * fallback);
        const RangeSolve sol = wallrange::solveRanges(pth, distance, Robot::Range::COUNT);
        if (sol.ok && sol.mae_m < Ekf::WALL_FIX_MAE_M) {
            const double dx = sol.x - px;
            const double dy = sol.y - py;
            if (dx * dx + dy * dy > Ekf::WALL_FIX_STEP_M * Ekf::WALL_FIX_STEP_M) {
                double H[ExtendedKalmanFilter::N] = {};
                H[0] = 1.0;
                ekf_.update_linear(H, dx, r);
                H[0] = 0.0;
                H[1] = 1.0;
                ekf_.update_linear(H, dy, r);
                return true;
            }
        }

        bool moved = false;
        for (int i = 0; i < Robot::Range::COUNT; ++i) {
            if (!distance[i].valid) continue;
            ekf_.get_pose_est(px, py, pth);
            const WallHit hit = wallrange::predictWallRange(Pose2d{px, py, pth}, i);
            if (!hit.hit) continue;
            const double innov = distance[i].range_m - hit.range_m;
            const double sigma = Robot::rangeSigmaM(distance[i].range_m, distance[i].confidence);
            if (std::fabs(innov) > Ekf::WALL_GATE_SIGMAS * sigma) continue;
            double H[ExtendedKalmanFilter::N] = {};
            H[0] = hit.hx;
            H[1] = hit.hy;
            H[2] = hit.htheta;
            ekf_.update_linear(H, innov, sigma * sigma);
            moved = true;
        }
        return moved;
    }

    Odometry odom_;
    ExtendedKalmanFilter ekf_;
    double total_heading_ = 0.0;
    double prev_theta_ = 0.0;
    int range_phase_ = 0;
};
