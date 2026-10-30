/* ParticleFilter.cpp -- Monte Carlo localization.
 * The per-particle loops are single precision. Pose2d and the sensor
 * readings stay double; each call converts them once, then stays in float.
 */
#include "estimators/ParticleFilter.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include "math/MathUtils.h"
#include "math/TrackingCenter.h"
#include <cmath>

namespace {

// -0.5 * (4 sigma)^2. A fix farther than this from every particle is treated
// as a kidnapped-robot measurement and half the cloud is reseeded on it.
constexpr float kKidnapLogLik = static_cast<float>(RangeLik::KIDNAP_LOG_LIK);
constexpr float kTwoPi = 6.283185307179586f;
constexpr float kPi = 3.141592653589793f;

float wrapAngle(float angle) {
    while (angle > kPi) angle -= kTwoPi;
    while (angle < -kPi) angle += kTwoPi;
    return angle;
}

}

ParticleFilter::ParticleFilter() : ParticleFilter(ParticleFilterConfig{}) {}

ParticleFilter::ParticleFilter(const ParticleFilterConfig& cfg) : cfg_(cfg) {
    if (cfg_.count < Particles::MIN) cfg_.count = Particles::MIN;
    if (cfg_.count > kMaxParticles) cfg_.count = kMaxParticles;
    if (!(cfg_.resample_ratio >= 0.0)) cfg_.resample_ratio = 0.0;
    rng_ = cfg_.seed ? static_cast<uint64_t>(cfg_.seed) : 1ull;
    rng_ ^= 0x9E3779B97F4A7C15ull;
    uniform();
    uniform();
    uniform();
    reset();
}

void ParticleFilter::reset() {
    reset(tracking::origin());
}

void ParticleFilter::reset(const Pose2d& pose) {
    pose_ = pose;
    pose_.theta = mathutil::normalize_angle(pose.theta);
    n_eff_ = static_cast<double>(cfg_.count);
    pos_spread_ = 0.0;
    const float x = static_cast<float>(pose_.x);
    const float y = static_cast<float>(pose_.y);
    const float th = static_cast<float>(pose_.theta);
    for (int i = 0; i < cfg_.count; ++i) {
        particles_[i].x = x;
        particles_[i].y = y;
        particles_[i].theta = th;
    }
    setUniformWeights();
}

void ParticleFilter::setUniformWeights() {
    const float lw = -std::log(static_cast<float>(cfg_.count));
    for (int i = 0; i < cfg_.count; ++i) particles_[i].log_w = lw;
}

float ParticleFilter::uniform() {
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 7;
    rng_ ^= rng_ << 17;
    // 24 bits, the float mantissa. Top bits of the xorshift.
    return static_cast<float>(rng_ >> 40) * (1.0f / 16777216.0f);
}

float ParticleFilter::normal() {
    float u1 = uniform();
    if (u1 < 1.0e-12f) u1 = 1.0e-12f;
    const float u2 = uniform();
    return std::sqrt(-2.0f * std::log(u1)) * std::cos(kTwoPi * u2);
}

void ParticleFilter::predict(double forward_m, double strafe_m, double d_theta_rad) {
    const float forward = static_cast<float>(forward_m);
    const float strafe = static_cast<float>(strafe_m);
    const float d_theta = static_cast<float>(d_theta_rad);
    const float travel = std::fabs(forward) + std::fabs(strafe);
    const float turning = std::fabs(d_theta);
    if (travel < 1.0e-12f && turning < 1.0e-12f) return;

    const float distance = std::sqrt(forward * forward + strafe * strafe);
    const float a_trans = static_cast<float>(cfg_.alpha_trans);
    const float a_trans_rot = static_cast<float>(cfg_.alpha_trans_rot);
    const float a_lat = static_cast<float>(cfg_.alpha_lateral);
    const float a_rot = static_cast<float>(cfg_.alpha_rot);
    const float a_rot_trans = static_cast<float>(cfg_.alpha_rot_trans);
    float sig_f = a_trans * std::fabs(forward) + a_trans_rot * turning;
    float sig_s = a_trans * std::fabs(strafe)
                + a_lat * std::fabs(forward)
                + a_trans_rot * turning;
    float sig_r = a_rot * turning + a_rot_trans * distance;
    sig_f += static_cast<float>(cfg_.beta_trans);
    sig_s += static_cast<float>(cfg_.beta_trans);
    sig_r += static_cast<float>(cfg_.beta_rot);
    if (sig_f < 0.0f) sig_f = 0.0f;
    if (sig_s < 0.0f) sig_s = 0.0f;
    if (sig_r < 0.0f) sig_r = 0.0f;

    // One rotation for the whole cloud. Heading spread is the gyro noise,
    // so each particle's own midpoint angle would change this step by a
    // fraction of a millimeter.
    const float mid = static_cast<float>(pose_.theta) + d_theta * 0.5f;

    for (int i = 0; i < cfg_.count; ++i) {
        Particle& p = particles_[i];
        const float nf = forward + ((sig_f > 0.0f) ? normal() * sig_f : 0.0f);
        const float ns = strafe + ((sig_s > 0.0f) ? normal() * sig_s : 0.0f);
        const float nd = d_theta + ((sig_r > 0.0f) ? normal() * sig_r : 0.0f);
        // Deltas are the geometric center. The particle is the front middle,
        // so a turn swings it about that center.
        double x = static_cast<double>(p.x);
        double y = static_cast<double>(p.y);
        const double th0 = static_cast<double>(p.theta);
        const double ndd = static_cast<double>(nd);
        tracking::integrate(x, y, th0, th0 + ndd,
                            static_cast<double>(nf), static_cast<double>(ns),
                            static_cast<double>(mid));
        p.x = static_cast<float>(x);
        p.y = static_cast<float>(y);
        p.theta = wrapAngle(static_cast<float>(th0 + ndd));
    }
    recomputeEstimate();
}

void ParticleFilter::correctHeading(double heading_rad) {
    const float sigma = static_cast<float>(cfg_.sigma_heading);
    if (!(sigma > 0.0f)) return;
    const float inv_var = 1.0f / (sigma * sigma);
    const float heading = static_cast<float>(heading_rad);
    for (int i = 0; i < cfg_.count; ++i) {
        const float err = wrapAngle(heading - particles_[i].theta);
        particles_[i].log_w += -0.5f * err * err * inv_var;
    }
    finishMeasurement();
}

void ParticleFilter::correctRanges(const DistanceReading* distance, int count) {
    if (!distance || count <= 0) return;
    if (count > Robot::Range::COUNT) count = Robot::Range::COUNT;

    // Log-likelihood floor, -0.5 * (5 sigma)^2. A residual past that is one
    // outlier (a blocked beam or a miss) and cannot wipe the particle.
    constexpr float kMaxPenalty = static_cast<float>(RangeLik::MAX_PENALTY);

    // Heading is the IMU, shared by the cloud. One sine/cosine, then each
    // beam is a wall plane: range = cx * x + cy * y + c0. That is the ray
    // to an axis-aligned wall with the direction fixed, so the per-particle
    // loop does not cast.
    const float c = mathutil::heading_cos(static_cast<float>(pose_.theta));
    const float s = mathutil::heading_sin(static_cast<float>(pose_.theta));
    const float x_lo = static_cast<float>(Robot::Range::FIELD_X_MIN_M);
    const float x_hi = static_cast<float>(Robot::Range::FIELD_X_MAX_M);
    const float y_lo = static_cast<float>(Robot::Range::FIELD_Y_MIN_M);
    const float y_hi = static_cast<float>(Robot::Range::FIELD_Y_MAX_M);
    const float ox0 = static_cast<float>(pose_.x);
    const float oy0 = static_cast<float>(pose_.y);
    constexpr float kDirMin = 1.0e-3f;
    constexpr float kMiss = 1.0e8f;

    struct BeamPrep {
        bool use = false;
        float z = 0;
        float inv_var = 0;
        float cx = 0;
        float cy = 0;
        float c0 = 0;
    };
    BeamPrep beams[Robot::Range::COUNT]{};
    int nvalid = 0;
    for (int i = 0; i < count; ++i) {
        if (!distance[i].valid) continue;
        const Robot::Range::Mount mount = Robot::Range::TRACKING_OFFSET[i];
        const float z = static_cast<float>(distance[i].range_m);
        const float sigma = static_cast<float>(Robot::rangeSigmaM(distance[i].range_m, distance[i].confidence));
        // Particles store the tracking centre. The offset is already from there.
        const float mx = static_cast<float>(mount.x);
        const float my = static_cast<float>(mount.y);
        const float yaw_c = std::cos(static_cast<float>(mount.yaw));
        const float yaw_s = std::sin(static_cast<float>(mount.yaw));
        const float rx = mx * c - my * s;
        const float ry = mx * s + my * c;
        const float dx = c * yaw_c - s * yaw_s;
        const float dy = s * yaw_c + c * yaw_s;

        float tx = kMiss;
        float ty = kMiss;
        float wall_x = 0.0f;
        float wall_y = 0.0f;
        if (dx > kDirMin) { wall_x = x_hi; tx = (x_hi - (ox0 + rx)) / dx; }
        else if (dx < -kDirMin) { wall_x = x_lo; tx = (x_lo - (ox0 + rx)) / dx; }
        if (dy > kDirMin) { wall_y = y_hi; ty = (y_hi - (oy0 + ry)) / dy; }
        else if (dy < -kDirMin) { wall_y = y_lo; ty = (y_lo - (oy0 + ry)) / dy; }
        const bool x_ok = tx > 0.0f && tx < kMiss;
        const bool y_ok = ty > 0.0f && ty < kMiss;
        if (!x_ok && !y_ok) continue;

        BeamPrep& b = beams[i];
        b.use = true;
        b.z = z;
        b.inv_var = 1.0f / (sigma * sigma);
        if (x_ok && (!y_ok || tx <= ty)) {
            b.cx = -1.0f / dx;
            b.cy = 0.0f;
            b.c0 = (wall_x - rx) / dx;
        } else {
            b.cx = 0.0f;
            b.cy = -1.0f / dy;
            b.c0 = (wall_y - ry) / dy;
        }
        ++nvalid;
    }
    if (nvalid == 0) return;

    for (int p = 0; p < cfg_.count; ++p) {
        Particle& part = particles_[p];
        float log_likelihood = 0.0f;
        for (int i = 0; i < count; ++i) {
            const BeamPrep& b = beams[i];
            if (!b.use) continue;
            const float predicted = b.cx * part.x + b.cy * part.y + b.c0;
            float term = kMaxPenalty;
            if (predicted > 0.0f) {
                const float err = b.z - predicted;
                term = -0.5f * err * err * b.inv_var;
                if (term < kMaxPenalty) term = kMaxPenalty;
            }
            log_likelihood += term;
        }
        part.log_w += log_likelihood;
    }

    finishMeasurement();
}

void ParticleFilter::correctPosition(double x_m, double y_m, double sigma_m) {
    if (!(sigma_m > 0.0)) return;
    const float x = static_cast<float>(x_m);
    const float y = static_cast<float>(y_m);
    const float inv_var = 1.0f / static_cast<float>(sigma_m * sigma_m);

    float best_ll = -1.0e30f;
    for (int i = 0; i < cfg_.count; ++i) {
        const float dx = particles_[i].x - x;
        const float dy = particles_[i].y - y;
        const float ll = -0.5f * (dx * dx + dy * dy) * inv_var;
        if (ll > best_ll) best_ll = ll;
    }
    if (best_ll < kKidnapLogLik) injectPosition(x_m, y_m, sigma_m, pose_.theta);

    for (int i = 0; i < cfg_.count; ++i) {
        const float dx = particles_[i].x - x;
        const float dy = particles_[i].y - y;
        particles_[i].log_w += -0.5f * (dx * dx + dy * dy) * inv_var;
    }
    finishMeasurement();
}

void ParticleFilter::correctPose(const Pose2d& pose, double sigma_xy_m, double sigma_theta_rad) {
    if (!(sigma_xy_m > 0.0) && !(sigma_theta_rad > 0.0)) return;
    const float inv_xy = (sigma_xy_m > 0.0) ? (1.0f / static_cast<float>(sigma_xy_m * sigma_xy_m)) : 0.0f;
    const float inv_th = (sigma_theta_rad > 0.0) ? (1.0f / static_cast<float>(sigma_theta_rad * sigma_theta_rad)) : 0.0f;
    const float meas_x = static_cast<float>(pose.x);
    const float meas_y = static_cast<float>(pose.y);
    const float meas_theta = static_cast<float>(mathutil::normalize_angle(pose.theta));

    if (inv_xy > 0.0f) {
        float best_ll = -1.0e30f;
        for (int i = 0; i < cfg_.count; ++i) {
            const float dx = particles_[i].x - meas_x;
            const float dy = particles_[i].y - meas_y;
            const float ll = -0.5f * (dx * dx + dy * dy) * inv_xy;
            if (ll > best_ll) best_ll = ll;
        }
        if (best_ll < kKidnapLogLik) {
            const double theta = (inv_th > 0.0f) ? static_cast<double>(meas_theta) : pose_.theta;
            injectPosition(pose.x, pose.y, sigma_xy_m, theta);
        }
    }

    for (int i = 0; i < cfg_.count; ++i) {
        if (inv_xy > 0.0f) {
            const float dx = particles_[i].x - meas_x;
            const float dy = particles_[i].y - meas_y;
            particles_[i].log_w += -0.5f * (dx * dx + dy * dy) * inv_xy;
        }
        if (inv_th > 0.0f) {
            const float err = wrapAngle(meas_theta - particles_[i].theta);
            particles_[i].log_w += -0.5f * err * err * inv_th;
        }
    }
    finishMeasurement();
}

void ParticleFilter::finishMeasurement() {
    normalize();
    recomputeEstimate();
    const double limit = cfg_.resample_ratio * static_cast<double>(cfg_.count);
    if (cfg_.resample_ratio > 0.0 && n_eff_ < limit) resample();
}

void ParticleFilter::normalize() {
    const int n = cfg_.count;
    float max_lw = particles_[0].log_w;
    for (int i = 1; i < n; ++i) {
        if (particles_[i].log_w > max_lw) max_lw = particles_[i].log_w;
    }
    if (!std::isfinite(max_lw)) {
        setUniformWeights();
        return;
    }

    float sum = 0.0f;
    for (int i = 0; i < n; ++i) {
        particles_[i].log_w -= max_lw;
        sum += std::exp(particles_[i].log_w);
    }
    if (!(sum > 0.0f) || !std::isfinite(sum)) {
        setUniformWeights();
        return;
    }
    const float log_sum = std::log(sum);
    for (int i = 0; i < n; ++i) particles_[i].log_w -= log_sum;
}

void ParticleFilter::recomputeEstimate() {
    const int n = cfg_.count;
    // Headings sit on the IMU, so the mean is the weighted offset from the
    // current heading. That avoids a sine and cosine per particle.
    const float ref = static_cast<float>(pose_.theta);
    float mx = 0.0f, my = 0.0f, dth = 0.0f, w2 = 0.0f;
    for (int i = 0; i < n; ++i) {
        const float w = std::exp(particles_[i].log_w);
        mx += w * particles_[i].x;
        my += w * particles_[i].y;
        dth += w * wrapAngle(particles_[i].theta - ref);
        w2 += w * w;
    }
    pose_.x = static_cast<double>(mx);
    pose_.y = static_cast<double>(my);
    pose_.theta = static_cast<double>(wrapAngle(ref + dth));
    n_eff_ = (w2 > 0.0f) ? static_cast<double>(1.0f / w2) : static_cast<double>(n);

    float var = 0.0f;
    for (int i = 0; i < n; ++i) {
        const float w = std::exp(particles_[i].log_w);
        const float dx = particles_[i].x - mx;
        const float dy = particles_[i].y - my;
        var += w * (dx * dx + dy * dy);
    }
    pos_spread_ = (var > 0.0f) ? static_cast<double>(std::sqrt(var)) : 0.0;
}

void ParticleFilter::resample() {
    const int n = cfg_.count;
    float cdf = 0.0f;
    for (int i = 0; i < n; ++i) {
        cdf += std::exp(particles_[i].log_w);
        scratch_[i].log_w = cdf;
    }
    if (!(cdf > 0.0f) || !std::isfinite(cdf)) {
        setUniformWeights();
        return;
    }

    const float step = cdf / static_cast<float>(n);
    float u = uniform() * step;
    int i = 0;
    const float lw = -std::log(static_cast<float>(n));
    for (int m = 0; m < n; ++m) {
        const float target = u + static_cast<float>(m) * step;
        while (i < n - 1 && scratch_[i].log_w < target) ++i;
        scratch_[m].x = particles_[i].x;
        scratch_[m].y = particles_[i].y;
        scratch_[m].theta = particles_[i].theta;
    }
    for (int m = 0; m < n; ++m) {
        particles_[m].x = scratch_[m].x;
        particles_[m].y = scratch_[m].y;
        particles_[m].theta = scratch_[m].theta;
        particles_[m].log_w = lw;
    }
    n_eff_ = static_cast<double>(n);
}

void ParticleFilter::injectPosition(double x_m, double y_m, double sigma_m, double theta) {
    const int n_new = cfg_.count / 2;
    const float x = static_cast<float>(x_m);
    const float y = static_cast<float>(y_m);
    const float sigma = static_cast<float>(sigma_m);
    const float th = wrapAngle(static_cast<float>(theta));
    for (int i = 0; i < n_new; ++i) {
        particles_[i].x = x + normal() * sigma;
        particles_[i].y = y + normal() * sigma;
        particles_[i].theta = th;
        particles_[i].log_w = 0.0f;
    }
}
