#pragma once
/* ---------------------------------------------------------------------------
 * ImuLinearDelta  --  chassis travel from the inertial accelerometer.
 *
 * The V5 inertial sensor reports specific force in G, in the chassis frame
 * (x forward along the sensor arrow, y to the left) when it is mounted flat.
 * apply() turns one sample into forward_delta and strafe_delta, in meters,
 * and leaves the wheel-derived velocity channels alone. Every estimator reads
 * those two deltas, so this is the linear step for all of them.
 *
 * Velocity is kept in the chassis frame. Yaw rate is crossed in, so a robot
 * that is moving forward and then turns carries that speed into strafe
 * instead of forgetting it. Bias is learned only while the wheels, the gyro,
 * and the accelerometer all say the robot is sitting still, so a push the
 * wheels miss is still integrated.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include <cmath>

class ImuLinearDelta {
public:
    void reset() {
        v_fwd_ = 0.0;
        v_lat_ = 0.0;
        bias_x_ = 0.0;
        bias_y_ = 0.0;
    }

    // use_accel is false in IMU fallback: the wheel deltas already on obs stay.
    void apply(ChassisObservation& obs, double dt_sec, bool use_accel) {
        if (!use_accel || !(dt_sec > 0.0)) return;
        if (!std::isfinite(obs.imu_accel_x) || !std::isfinite(obs.imu_accel_y)) return;
        if (std::fabs(obs.imu_accel_x) > 8.0 || std::fabs(obs.imu_accel_y) > 8.0) return;

        const double ax_g = obs.imu_accel_x - bias_x_;
        const double ay_g = obs.imu_accel_y - bias_y_;
        const bool wheels_still = std::fabs(obs.v_lin) < 0.05 && std::fabs(obs.v_ang) < 0.15;
        const bool accel_still = (ax_g * ax_g + ay_g * ay_g) < (0.04 * 0.04);
        if (wheels_still && accel_still) {
            const double k = dt_sec / 0.4;
            bias_x_ += (obs.imu_accel_x - bias_x_) * k;
            bias_y_ += (obs.imu_accel_y - bias_y_) * k;
            const double bleed = std::exp(-dt_sec / 0.05);
            v_fwd_ *= bleed;
            v_lat_ *= bleed;
            obs.forward_delta = 0.0;
            obs.strafe_delta = 0.0;
            return;
        }

        const double ax = (obs.imu_accel_x - bias_x_) * kG;
        const double ay = (obs.imu_accel_y - bias_y_) * kG;
        const double w = obs.v_ang;
        const double vf = v_fwd_ + (ax + w * v_lat_) * dt_sec;
        const double vl = v_lat_ + (ay - w * v_fwd_) * dt_sec;
        obs.forward_delta = 0.5 * (v_fwd_ + vf) * dt_sec;
        obs.strafe_delta = 0.5 * (v_lat_ + vl) * dt_sec;
        v_fwd_ = vf;
        v_lat_ = vl;
    }

private:
    static constexpr double kG = 9.80665;

    double v_fwd_ = 0.0;
    double v_lat_ = 0.0;
    double bias_x_ = 0.0;   // G
    double bias_y_ = 0.0;
};
