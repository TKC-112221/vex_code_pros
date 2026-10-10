/* ---------------------------------------------------------------------------
 * SensorKalman.cpp  --  per-channel random-walk Kalman for sensor noise.
 *
 * Window is 3 samples. Steady-state gain is 1/3, the same per-sample weight
 * as a 3-point average, so white noise shrinks by about sqrt(3). Heading and
 * range are predicted with the measured rate first, so that gain no longer
 * lags a constant-rate signal: the innovation is the noise, not the motion.
 * The first three samples of a channel return the running mean and seed the
 * covariance; prediction starts once that seed exists.
 * ------------------------------------------------------------------------- */
#include "math/SensorKalman.h"

#include "core/Specs.h"
#include "core/Tuning.h"

#include <cmath>

namespace {

class ScalarKalman {
public:
    static constexpr int kWindow = SensorFilter::KALMAN_WINDOW;

    void reset() {
        x_ = 0.0;
        p_ = 1.0;
        sum_ = 0.0;
        n_ = 0;
        primed_ = false;
    }

    // predicted_delta is the change expected from a measured rate over this
    // sample. It is applied only after the channel is primed: the warmup mean
    // has no rate to subtract, and shifting it would bias the seed.
    double filter(double measurement, double predicted_delta = 0.0) {
        if (!primed_) {
            sum_ += measurement;
            ++n_;
            x_ = sum_ / static_cast<double>(n_);
            if (n_ >= kWindow) {
                primed_ = true;
                p_ = 1.0 / static_cast<double>(kWindow);
            }
            return x_;
        }

        // R = 1. Q = R / (W * (W - 1)) yields K_ss = 1/W.
        x_ += predicted_delta;
        constexpr double q = 1.0 / (static_cast<double>(kWindow) * (kWindow - 1));
        p_ += q;
        const double k = p_ / (p_ + 1.0);
        x_ += k * (measurement - x_);
        p_ *= (1.0 - k);
        return x_;
    }

private:
    double x_ = 0.0;
    double p_ = 1.0;
    double sum_ = 0.0;
    int n_ = 0;
    bool primed_ = false;
};

struct SensorFilters {
    ScalarKalman heading;
    ScalarKalman v_lin;
    ScalarKalman v_ang;
    ScalarKalman left_vel_rpm;
    ScalarKalman right_vel_rpm;
    ScalarKalman range[kDistanceChannels];
    int range_miss[kDistanceChannels] = {};

    void reset() {
        heading.reset();
        v_lin.reset();
        v_ang.reset();
        left_vel_rpm.reset();
        right_vel_rpm.reset();
        for (int i = 0; i < kDistanceChannels; ++i) {
            range[i].reset();
            range_miss[i] = 0;
        }
    }
};

SensorFilters g_filters;

double filterChannel(ScalarKalman& channel, double sample, double predicted_delta = 0.0) {
    if (!std::isfinite(sample)) return sample;
    if (!std::isfinite(predicted_delta)) predicted_delta = 0.0;
    return channel.filter(sample, predicted_delta);
}

// Closing of one beam over this tick, in meters. Chassis frame is x forward,
// y left; a beam's range falls as its mount moves along the beam. The mount
// lever arm is included so an offset perpendicular to the beam moves with yaw.
double rangeDelta(int index, double forward_delta, double strafe_delta, double v_ang) {
    const Robot::Range::Mount& mount = Robot::Range::MOUNT[index];
    const double dth = v_ang * System::DT_SEC;
    const double dx = forward_delta - dth * mount.y;
    const double dy = strafe_delta + dth * mount.x;
    return -(dx * std::cos(mount.yaw) + dy * std::sin(mount.yaw));
}

void filterChassis(ChassisObservation& obs) {
    // Raw gyro rate, captured before v_ang itself is smoothed. heading_rad and
    // v_ang share a sign (both negated from the IMU), so this is d(heading)/dt.
    // IMU accel / rate stay raw for recording bandwidth; ImuLinearDelta owns
    // their bias model.
    const double heading_delta = obs.v_ang * System::DT_SEC;
    obs.heading_rad = filterChannel(g_filters.heading, obs.heading_rad, heading_delta);
    obs.v_lin = filterChannel(g_filters.v_lin, obs.v_lin);
    obs.v_ang = filterChannel(g_filters.v_ang, obs.v_ang);
    obs.left_vel_rpm = filterChannel(g_filters.left_vel_rpm, obs.left_vel_rpm);
    obs.right_vel_rpm = filterChannel(g_filters.right_vel_rpm, obs.right_vel_rpm);
}

void filterDistances(DistanceReading (&distance)[kDistanceChannels],
                     double forward_delta, double strafe_delta, double v_ang) {
    for (int i = 0; i < kDistanceChannels; ++i) {
        if (!distance[i].valid || !std::isfinite(distance[i].range_m)) {
            if (++g_filters.range_miss[i] >= ScalarKalman::kWindow) {
                g_filters.range[i].reset();
            }
            continue;
        }
        g_filters.range_miss[i] = 0;
        distance[i].range_m = g_filters.range[i].filter(
            distance[i].range_m, rangeDelta(i, forward_delta, strafe_delta, v_ang));
    }
}

}

void applySensorKalman(SensorDict& sensors) {
    // Travel and yaw rate before filterChassis replaces v_ang with the smoothed
    // value. The deltas themselves are never filtered.
    const double forward = sensors.chassis.forward_delta;
    const double strafe = sensors.chassis.strafe_delta;
    const double v_ang = sensors.chassis.v_ang;
    filterChassis(sensors.chassis);
    filterDistances(sensors.distance, forward, strafe, v_ang);
}

void resetSensorKalman() {
    g_filters.reset();
}
