#pragma once
/* ---------------------------------------------------------------------------
 * SlipEnsembleEstimator  --  slip-hypothesis localization.
 *
 * Same IEstimator slot as ParticleFilterEstimator. Wheel travel and the IMU
 * heading change are integrated by each particle's own slip model. Every 5th
 * sample, each pose takes a damped step toward the distance-sensor readings.
 * Velocity is passed through from the drivebase.
 *
 * Absolute fixes are optional. observePosition / observePose pull the cloud
 * without replacing any particle's slip model.
 * ------------------------------------------------------------------------- */

#include "interfaces/IEstimator.h"
#include "estimators/SlipEnsemble.h"
#include "math/MathUtils.h"
#include <cstdint>

class SlipEnsembleEstimator : public IEstimator {
public:
    explicit SlipEnsembleEstimator(const SlipEnsembleConfig& cfg = {})
        : filter_(cfg) {}

    void reset() override {
        filter_.reset();
        last_heading_ = 0.0;
        total_heading_ = 0.0;
        prev_theta_ = 0.0;
        v_lin_ = v_ang_ = 0.0;
        range_phase_ = 0;
    }

    void overridePose(const Pose2d& pose) override {
        filter_.reset(poseInchesToMeters(pose));
        last_heading_ = pose.theta;
        prev_theta_ = filter_.pose().theta;
        range_phase_ = 0;
    }

    void clearVelocity() override { v_lin_ = v_ang_ = 0.0; }

    void update(const SensorDict& sensors) override {
        const ChassisObservation& obs = sensors.chassis;
        const double d_theta = mathutil::normalize_angle(obs.heading_rad - last_heading_);
        last_heading_ = obs.heading_rad;
        filter_.predict(obs.forward_delta, obs.strafe_delta, d_theta);
        // Every 5th 10 ms sample. The walls do not move between those ticks.
        if (range_phase_ == 0) {
            filter_.nudge(sensors.distance, kDistanceChannels, obs.heading_rad);
        }
        range_phase_ = (range_phase_ + 1) % Slip::RANGE_STRIDE;
        v_lin_ = obs.v_lin;
        v_ang_ = obs.v_ang;
        noteHeading();
    }

    void observePosition(double x_in, double y_in, double sigma_in) override {
        filter_.correctPosition(inchesToMeters(x_in), inchesToMeters(y_in),
                                inchesToMeters(sigma_in));
        noteHeading();
    }

    void observePose(const Pose2d& pose, double sigma_xy_in, double sigma_theta_rad) override {
        filter_.correctPose(poseInchesToMeters(pose), inchesToMeters(sigma_xy_in),
                            sigma_theta_rad);
        noteHeading();
    }

    StateEstimate getState() const override {
        StateEstimate s;
        s.pose = poseMetersToInches(filter_.pose());
        s.v_lin = v_lin_;
        s.v_ang = v_ang_;
        return s;
    }

    double getTotalHeading() const override { return total_heading_; }

    // All particles when count <= cap, otherwise cap of them chosen uniformly
    // without replacement. Does not touch the filter's own random stream.
    int sampleParticles(Pose2d* out, int cap) const override {
        const int n = filter_.count();
        if (out == nullptr || cap <= 0 || n <= 0) return 0;
        int take = n < cap ? n : cap;
        if (take == n) {
            for (int i = 0; i < n; ++i) out[i] = poseMetersToInches(filter_.particlePose(i));
            return n;
        }
        if (take > 64) take = 64;
        int chosen[64];
        int got = 0;
        for (int j = n - take; j < n; ++j) {
            int t = static_cast<int>(displayUniform_() * static_cast<double>(j + 1));
            if (t > j) t = j;
            bool seen = false;
            for (int k = 0; k < got; ++k) {
                if (chosen[k] == t) seen = true;
            }
            chosen[got++] = seen ? j : t;
        }
        for (int i = 0; i < got; ++i)
            out[i] = poseMetersToInches(filter_.particlePose(chosen[i]));
        return got;
    }

    double effectiveSampleSize() const { return filter_.effectiveSampleSize(); }
    double positionSpreadM() const { return filter_.positionSpreadM(); }

private:
    void noteHeading() {
        const double th = filter_.pose().theta;
        total_heading_ += mathutil::normalize_angle(th - prev_theta_);
        prev_theta_ = th;
    }

    double displayUniform_() const {
        display_rng_ ^= display_rng_ << 13;
        display_rng_ ^= display_rng_ >> 7;
        display_rng_ ^= display_rng_ << 17;
        return static_cast<double>(display_rng_ >> 11) * (1.0 / 9007199254740992.0);
    }

    SlipEnsemble filter_;
    mutable std::uint64_t display_rng_ = 0x51F17Eull;
    double last_heading_ = 0.0;
    double total_heading_ = 0.0;
    double prev_theta_ = 0.0;
    double v_lin_ = 0.0;
    double v_ang_ = 0.0;
    int range_phase_ = 0;
};
