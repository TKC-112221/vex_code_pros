#pragma once
/* ---------------------------------------------------------------------------
 * ParticleFilter  --  Monte Carlo localization over a fixed particle set.
 *
 * Each particle is a field-frame pose hypothesis. predict() samples a noisy
 * chassis-frame motion model (the same midpoint integration as Odometry).
 * correctHeading(), correctRanges(), correctPosition(), and correctPose()
 * reweight by a Gaussian likelihood and resample when the effective sample
 * size collapses. Range updates use one heading for the whole cloud: the
 * sine and cosine of that heading are computed once per tick, and each
 * beam's wall range is that slope times the particle's x or y. They do not
 * reseed the cloud, so two wall-consistent poses can both survive.
 *
 * With no wall return and no absolute fix, the cloud tracks odometry and the
 * IMU, and its spread is the accumulated motion uncertainty. A position or
 * pose fix (GPS, vision) pulls the cloud, including when the fix disagrees
 * with every current hypothesis.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "core/Specs.h"
#include <cmath>
#include <cstdint>

struct ParticleFilterConfig {
    // Heavy tick is predict + heading + four wall beams + a resample.
    // The cloud is stored and updated in single precision: the V5 user
    // core (one Cortex-A9, 1333 MIPS) pipelines float and issues double
    // at about half that rate. 400 particles therefore cost about what
    // 200 doubles did, around one to two milliseconds, inside the 10 ms
    // brain tick. The sensor sigmas below keep the cloud a centimeter or
    // two wide between wall updates, and 400 samples cover that.
    int count = 400;                 // clamped to [8, ParticleFilter::kMaxParticles]

    // Translation stddev (m) on this step =
    //   alpha_trans * |along-track| + alpha_trans_rot * |dtheta|
    //   + beta_trans when the robot is moving.
    // alpha_trans is the accelerometer scale. Travel comes from the IMU
    // (ImuLinearDelta), and Imu::ACCEL_SCALE_SIGMA is that fraction.
    // alpha_trans_rot is half the track times the gyro scale: a scale error
    // on a turn shifts the step sideways.
    // beta_trans spreads the cloud by the near-field range accuracy over
    // one distance-sensor period (ACCURACY_NEAR_M / sqrt(RANGE_STRIDE)).
    // One blue-cartridge tick is 0.65 mm and the accel white noise is
    // smaller still. A cloud tighter than the range sigma gives every
    // particle the same wall likelihood, so the beams never move the pose.
    double alpha_trans = Robot::Imu::ACCEL_SCALE_SIGMA;
    double alpha_trans_rot =
        (Robot::Chassis::TRACK_WIDTH_M * 0.5) * Robot::Imu::SCALE_SIGMA;
    double beta_trans = Robot::Range::ACCURACY_NEAR_M /
        std::sqrt(static_cast<double>(System::RANGE_STRIDE));
    // Extra lateral stddev (m) = alpha_lateral * |forward| this step.
    // Same accelerometer scale on the lateral axis. The previous 0.30 was
    // about 60x Imu::ACCEL_SCALE_SIGMA and opened the cloud far past both
    // the IMU and the distance sensors, which is what forced the larger count.
    double alpha_lateral = Robot::Imu::ACCEL_SCALE_SIGMA;

    // Rotation stddev (rad) = alpha_rot * |dtheta| + alpha_rot_trans * distance
    //                       + beta_rot when the robot is moving.
    // alpha_rot is Imu::SCALE_SIGMA (half of the ±1% gyro tolerance).
    // alpha_rot_trans is Imu::BIAS_SIGMA_DPS in rad per meter at 1 m/s.
    // beta_rot is one reported heading step (Imu::HEADING_LSB_DEG).
    double alpha_rot = Robot::Imu::SCALE_SIGMA;
    double alpha_rot_trans = Robot::Imu::BIAS_SIGMA_DPS * (Robot::PI / 180.0);
    double beta_rot = Robot::Imu::HEADING_LSB_DEG * (Robot::PI / 180.0);

    // IMU heading measurement stddev (rad). 0 disables the heading update.
    // The heading word is quantized to HEADING_LSB_DEG. This sigma is one
    // revolution of Imu::SCALE_SIGMA, because the filter does not estimate
    // scale. Using the 0.01 deg step would drop every particle that is not
    // identical to the gyro.
    double sigma_heading = Robot::Imu::SCALE_SIGMA * (2.0 * Robot::PI);

    // Resample when Neff < resample_ratio * count. 0 disables resampling.
    double resample_ratio = 0.2;

    uint32_t seed = 1;
};

class ParticleFilter {
public:
    static constexpr int kMaxParticles = 3000;

    ParticleFilter();
    explicit ParticleFilter(const ParticleFilterConfig& cfg);

    void reset();
    void reset(const Pose2d& pose);

    // Sample the motion model. d_theta_rad is the measured heading change.
    void predict(double forward_m, double strafe_m, double d_theta_rad);

    // Gaussian updates. sigma <= 0 is ignored.
    // Heading uses cfg.sigma_heading against an absolute IMU measurement.
    void correctHeading(double heading_rad);
    // Distance beams in sensor-index order (D1..D4). Invalid samples are skipped.
    void correctRanges(const DistanceReading* distance, int count);
    void correctPosition(double x_m, double y_m, double sigma_m);
    void correctPose(const Pose2d& pose, double sigma_xy_m, double sigma_theta_rad);

    const Pose2d& pose() const { return pose_; }
    // Pose of particle index, or the origin if index is out of range.
    Pose2d particlePose(int index) const {
        if (index < 0 || index >= cfg_.count) return Pose2d{};
        return Pose2d{static_cast<double>(particles_[index].x),
                      static_cast<double>(particles_[index].y),
                      static_cast<double>(particles_[index].theta)};
    }
    // 1 / sum(w^2). Equals count when weights are uniform.
    double effectiveSampleSize() const { return n_eff_; }
    // RMS distance (m) of the weighted cloud from the mean position.
    double positionSpreadM() const { return pos_spread_; }
    int count() const { return cfg_.count; }

private:
    struct Particle {
        float x = 0;
        float y = 0;
        float theta = 0;
        float log_w = 0;
    };

    void setUniformWeights();
    void normalize();
    void recomputeEstimate();
    void resample();
    void finishMeasurement();
    void injectPosition(double x_m, double y_m, double sigma_m, double theta);

    float uniform();
    float normal();

    ParticleFilterConfig cfg_{};
    Particle particles_[kMaxParticles]{};
    Particle scratch_[kMaxParticles]{};
    Pose2d pose_{};
    double n_eff_ = 0;
    double pos_spread_ = 0;
    uint64_t rng_ = 1;
};
