#pragma once
/* ---------------------------------------------------------------------------
 * OdometryEstimator  --  trapezoidal wheel odometry, plus a wall nudge.
 *
 * Pose comes from integrating wheel travel along the IMU heading, matching
 * v1. Heading is the IMU measurement, so the distance sensors do not turn
 * the pose: once per distance-sensor period they take one damped step in
 * x/y toward the field walls. Velocity is passed through from the drivebase.
 * ------------------------------------------------------------------------- */

#include "interfaces/IEstimator.h"
#include "estimators/Odometry.h"
#include "math/MathUtils.h"
#include "math/WallRange.h"
#include "core/Specs.h"
#include "core/Tuning.h"

class OdometryEstimator : public IEstimator {
public:
    void reset() override {
        odom_.setPose(Pose2d{});
        total_heading_ = 0.0;
        prev_theta_ = 0.0;
        v_lin_ = v_ang_ = 0.0;
        range_phase_ = 0;
    }

    void overridePose(const Pose2d& pose) override {
        const Pose2d stored = poseInchesToMeters(pose);
        odom_.setPose(stored);
        prev_theta_ = stored.theta;   // keep display delta continuous
        range_phase_ = 0;
    }

    void update(const SensorDict& sensors) override {
        const ChassisObservation& obs = sensors.chassis;
        odom_.update(obs.forward_delta, obs.strafe_delta, obs.heading_rad);
        // Once per distance-sensor period. The walls do not move, and the
        // sensor does not publish a new range on the ticks in between.
        if (range_phase_ == 0) {
            wallrange::nudgePosition(odom_.pose.x, odom_.pose.y, odom_.pose.theta,
                                     sensors.distance, kDistanceChannels,
                                     WallNudge::ODOM_GAIN, WallNudge::STEP_M, WallNudge::GATE_M);
        }
        range_phase_ = (range_phase_ + 1) % System::RANGE_STRIDE;
        // Unwrapped heading accumulator (display only).
        double d = mathutil::normalize_angle(odom_.pose.theta - prev_theta_);
        total_heading_ += d;
        prev_theta_ = odom_.pose.theta;
        v_lin_ = obs.v_lin;
        v_ang_ = obs.v_ang;
    }

    StateEstimate getState() const override {
        StateEstimate s;
        s.pose = poseMetersToInches(odom_.pose);
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
    int range_phase_ = 0;
};
