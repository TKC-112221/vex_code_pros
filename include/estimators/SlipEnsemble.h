#pragma once
/* ---------------------------------------------------------------------------
 * SlipEnsemble  --  a fixed set of slip hypotheses, nudged by the walls.
 *
 * Each particle keeps its own motion model for the whole run: longitudinal
 * scale, lateral scale, sideways skid, turning skid, and heading scale, plus
 * the width of the noise it adds on top. predict() integrates the measured
 * forward, strafe, and heading deltas through that model. Nothing is copied
 * or deleted.
 *
 * nudge() then takes one damped least-squares step on x/y/theta that reduces
 * the gap between the distance sensors and the ranges that pose would see on
 * the field wall. The reported pose is the weighted mean after that step.
 * Weights come from the fit before the step, so a hypothesis that was already
 * close keeps the estimate, and a gain below 1 leaves the cloud spread out
 * across the slip models instead of pinning every particle to one pose.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "core/Tuning.h"
#include <cstdint>

struct SlipEnsembleConfig {
    int count = Slip::COUNT;         // clamped to [Slip::MIN, SlipEnsemble::kMaxParticles]

    // Half-width of each persistent characteristic. Particle i sits on a
    // low-discrepancy point in this box, centered on "encoder and IMU are
    // right, and the robot does not skid."
    double forward_scale_span = Slip::FORWARD_SCALE_SPAN;
    double strafe_scale_span = Slip::STRAFE_SCALE_SPAN;
    double skid_per_m_span = Slip::SKID_PER_M_SPAN;
    double skid_per_rad_span = Slip::SKID_PER_RAD_SPAN;
    double heading_scale_span = Slip::HEADING_SCALE_SPAN;

    // Per-step noise is the shared motion stddev below, times this particle's
    // own scale. The scales are spread across [min, max].
    double noise_scale_min = Slip::NOISE_SCALE_MIN;
    double noise_scale_max = Slip::NOISE_SCALE_MAX;

    // Shared motion stddev. Systematic skid lives in the spans above, so these
    // stay near sensor noise: a blue-cartridge tick is about 0.65 mm, and the
    // inertial rate noise is a few hundredths of a degree per second.
    double alpha_trans = Slip::ALPHA_TRANS;
    double alpha_trans_rot = Slip::ALPHA_TRANS_ROT;
    double alpha_lateral = Slip::ALPHA_LATERAL;
    double alpha_rot = Slip::ALPHA_ROT;
    double alpha_rot_trans = Slip::ALPHA_ROT_TRANS;
    double beta_trans = Slip::BETA_TRANS;
    double beta_rot = Slip::BETA_ROT;

    // IMU heading stddev (rad) inside the nudge. 0 disables it.
    double sigma_heading = Slip::SIGMA_HEADING;

    // Fraction of the least-squares pose step to apply. 1 snaps every particle
    // onto the walls and erases which slip model was right.
    double nudge_gain = Slip::NUDGE_GAIN;
    double nudge_step_m = Slip::NUDGE_STEP_M;
    double nudge_step_rad = Slip::NUDGE_STEP_RAD;
    // A beam that misses the pose agreed by the other beams by more than this
    // is treated as blocked and left out of the step. Particles may sit
    // farther than this from the walls; that distance is what the step closes.
    double range_gate_m = Slip::RANGE_GATE_M;

    uint32_t seed = 1;
};

class SlipEnsemble {
public:
    static constexpr int kMaxParticles = Slip::MAX;

    SlipEnsemble();
    explicit SlipEnsemble(const SlipEnsembleConfig& cfg);

    void reset();
    void reset(const Pose2d& pose);

    // Integrate one chassis step through each particle's slip model.
    void predict(double forward_m, double strafe_m, double d_theta_rad);

    // Damped range (and heading) step. distance is sensor-index order D1..D4.
    void nudge(const DistanceReading* distance, int count, double heading_rad);

    // Absolute fixes. A fix no particle is near teleports the cloud onto it
    // and leaves each particle's slip model in place.
    void correctPosition(double x_m, double y_m, double sigma_m);
    void correctPose(const Pose2d& pose, double sigma_xy_m, double sigma_theta_rad);

    const Pose2d& pose() const { return pose_; }
    Pose2d particlePose(int index) const {
        if (index < 0 || index >= cfg_.count) return Pose2d{};
        return Pose2d{particles_[index].x, particles_[index].y, particles_[index].theta};
    }
    // 1 / sum(w^2). Equals count when weights are uniform.
    double effectiveSampleSize() const { return n_eff_; }
    // RMS distance (m) of the weighted cloud from the mean position.
    double positionSpreadM() const { return pos_spread_; }
    int count() const { return cfg_.count; }

private:
    struct Particle {
        double x = 0;
        double y = 0;
        double theta = 0;
        double log_w = 0;
        double k_forward = 1;
        double k_strafe = 1;
        double skid_per_m = 0;
        double skid_per_rad = 0;
        double k_heading = 1;
        double noise_scale = 1;
    };

    void sampleCharacteristics();
    void setUniformWeights();
    void normalize();
    void recomputeEstimate();
    void clampParticle(Particle& p) const;

    double uniform();
    double normal();

    SlipEnsembleConfig cfg_{};
    Particle particles_[kMaxParticles]{};
    Pose2d pose_{};
    double n_eff_ = 0;
    double pos_spread_ = 0;
    uint64_t rng_ = 1;
};
