/* SlipEnsemble.cpp -- fixed slip hypotheses, nudged by wall ranges. */
#include "estimators/SlipEnsemble.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include "math/MathUtils.h"
#include "math/TrackingCenter.h"
#include "math/WallRange.h"
#include <cmath>

namespace {

constexpr double kTwoPi = 6.283185307179586;
// -0.5 * (5 sigma)^2. Matches the particle filter's per-beam floor so a
// blocked return cannot wipe a particle out of the weighted mean.
constexpr double kMaxPenalty = RangeLik::MAX_PENALTY;
constexpr double kKidnapSigmas = RangeLik::KIDNAP_SIGMAS;
constexpr double kMaxGradSq = Slip::MAX_GRAD_SQ;
constexpr double kMaxHTheta = Slip::MAX_H_THETA;

// Kronecker bases so `count` particles fill the slip box instead of clumping.
constexpr double kAlpha[6] = {
    0.6180339887498949,
    0.4142135623730951,
    0.7320508075688773,
    0.2360679774997897,
    0.6457513110645907,
    0.3166247903553998
};

struct BeamPrep {
    bool use = false;
    double z = 0;
    double inv_var = 0;
};

bool solve3(double A[3][3], const double b[3], double step[3]) {
    double m[3][4];
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) m[r][c] = A[r][c];
        m[r][3] = b[r];
    }
    for (int col = 0; col < 3; ++col) {
        int piv = col;
        double best = std::fabs(m[col][col]);
        for (int r = col + 1; r < 3; ++r) {
            const double v = std::fabs(m[r][col]);
            if (v > best) {
                best = v;
                piv = r;
            }
        }
        if (!(best > 1e-14)) return false;
        if (piv != col) {
            for (int c = col; c < 4; ++c) {
                const double tmp = m[col][c];
                m[col][c] = m[piv][c];
                m[piv][c] = tmp;
            }
        }
        const double div = m[col][col];
        for (int c = col; c < 4; ++c) m[col][c] /= div;
        for (int r = 0; r < 3; ++r) {
            if (r == col) continue;
            const double f = m[r][col];
            for (int c = col; c < 4; ++c) m[r][c] -= f * m[col][c];
        }
    }
    step[0] = m[0][3];
    step[1] = m[1][3];
    step[2] = m[2][3];
    return std::isfinite(step[0]) && std::isfinite(step[1]) && std::isfinite(step[2]);
}

void addMeas(double A[3][3], double b[3],
             double hx, double hy, double ht, double innov, double w) {
    A[0][0] += w * hx * hx;
    A[0][1] += w * hx * hy;
    A[0][2] += w * hx * ht;
    A[1][1] += w * hy * hy;
    A[1][2] += w * hy * ht;
    A[2][2] += w * ht * ht;
    b[0] += w * hx * innov;
    b[1] += w * hy * innov;
    b[2] += w * ht * innov;
}

double unitInterval(int index, int axis, double shift) {
    const double x = (static_cast<double>(index) + 0.5 + shift) * kAlpha[axis];
    return x - std::floor(x);
}

}  // namespace

SlipEnsemble::SlipEnsemble() : SlipEnsemble(SlipEnsembleConfig{}) {}

SlipEnsemble::SlipEnsemble(const SlipEnsembleConfig& cfg) : cfg_(cfg) {
    if (cfg_.count < Slip::MIN) cfg_.count = Slip::MIN;
    if (cfg_.count > kMaxParticles) cfg_.count = kMaxParticles;
    if (cfg_.noise_scale_min < 0.0) cfg_.noise_scale_min = 0.0;
    if (cfg_.noise_scale_max < cfg_.noise_scale_min) cfg_.noise_scale_max = cfg_.noise_scale_min;
    if (cfg_.nudge_gain < 0.0) cfg_.nudge_gain = 0.0;
    if (cfg_.nudge_gain > 1.0) cfg_.nudge_gain = 1.0;
    if (cfg_.nudge_step_m < 0.0) cfg_.nudge_step_m = 0.0;
    if (cfg_.nudge_step_rad < 0.0) cfg_.nudge_step_rad = 0.0;
    if (cfg_.range_gate_m < 0.0) cfg_.range_gate_m = 0.0;
    rng_ = cfg_.seed ? static_cast<uint64_t>(cfg_.seed) : 1ull;
    rng_ ^= 0x9E3779B97F4A7C15ull;
    uniform();
    uniform();
    uniform();
    sampleCharacteristics();
    reset();
}

void SlipEnsemble::sampleCharacteristics() {
    const double shift = static_cast<double>(cfg_.seed ? cfg_.seed : 1u);
    const double nmin = cfg_.noise_scale_min;
    const double nspan = cfg_.noise_scale_max - nmin;
    for (int i = 0; i < cfg_.count; ++i) {
        Particle& p = particles_[i];
        const double c0 = unitInterval(i, 0, shift) * 2.0 - 1.0;
        const double c1 = unitInterval(i, 1, shift) * 2.0 - 1.0;
        const double c2 = unitInterval(i, 2, shift) * 2.0 - 1.0;
        const double c3 = unitInterval(i, 3, shift) * 2.0 - 1.0;
        const double c4 = unitInterval(i, 4, shift) * 2.0 - 1.0;
        p.k_forward = 1.0 + c0 * cfg_.forward_scale_span;
        p.k_strafe = 1.0 + c1 * cfg_.strafe_scale_span;
        p.skid_per_m = c2 * cfg_.skid_per_m_span;
        p.skid_per_rad = c3 * cfg_.skid_per_rad_span;
        p.k_heading = 1.0 + c4 * cfg_.heading_scale_span;
        p.noise_scale = nmin + unitInterval(i, 5, shift) * nspan;
    }
}

void SlipEnsemble::reset() {
    reset(tracking::origin());
}

void SlipEnsemble::reset(const Pose2d& pose) {
    pose_ = pose;
    pose_.theta = mathutil::normalize_angle(pose.theta);
    for (int i = 0; i < cfg_.count; ++i) {
        particles_[i].x = pose_.x;
        particles_[i].y = pose_.y;
        particles_[i].theta = pose_.theta;
    }
    setUniformWeights();
    n_eff_ = static_cast<double>(cfg_.count);
    pos_spread_ = 0.0;
}

void SlipEnsemble::setUniformWeights() {
    const double lw = -std::log(static_cast<double>(cfg_.count));
    for (int i = 0; i < cfg_.count; ++i) particles_[i].log_w = lw;
}

void SlipEnsemble::clampParticle(Particle& p) const {
    Pose2d center = tracking::centerFromTracking(Pose2d{p.x, p.y, p.theta});
    Robot::clampFieldMeters(center.x, center.y, 0.18);
    center.theta = mathutil::normalize_angle(center.theta);
    const Pose2d tracked = tracking::trackingFromCenter(center);
    p.x = tracked.x;
    p.y = tracked.y;
    p.theta = tracked.theta;
}

double SlipEnsemble::uniform() {
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 7;
    rng_ ^= rng_ << 17;
    return static_cast<double>(rng_ >> 11) * (1.0 / 9007199254740992.0);
}

double SlipEnsemble::normal() {
    double u1 = uniform();
    if (u1 < 1e-12) u1 = 1e-12;
    const double u2 = uniform();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(kTwoPi * u2);
}

void SlipEnsemble::predict(double forward_m, double strafe_m, double d_theta_rad) {
    const double travel = std::fabs(forward_m) + std::fabs(strafe_m);
    const double turning = std::fabs(d_theta_rad);
    if (travel < 1e-12 && turning < 1e-12) return;

    const double distance = std::sqrt(forward_m * forward_m + strafe_m * strafe_m);
    double sig_f = cfg_.alpha_trans * std::fabs(forward_m) + cfg_.alpha_trans_rot * turning;
    double sig_s = cfg_.alpha_trans * std::fabs(strafe_m)
                 + cfg_.alpha_lateral * std::fabs(forward_m)
                 + cfg_.alpha_trans_rot * turning;
    double sig_r = cfg_.alpha_rot * turning + cfg_.alpha_rot_trans * distance;
    sig_f += cfg_.beta_trans;
    sig_s += cfg_.beta_trans;
    sig_r += cfg_.beta_rot;
    if (sig_f < 0.0) sig_f = 0.0;
    if (sig_s < 0.0) sig_s = 0.0;
    if (sig_r < 0.0) sig_r = 0.0;

    for (int i = 0; i < cfg_.count; ++i) {
        Particle& p = particles_[i];
        const double fwd = forward_m * p.k_forward;
        const double str = strafe_m * p.k_strafe;
        const double yaw = d_theta_rad * p.k_heading;
        const double ns_scale = p.noise_scale;
        const double nf = fwd + ((sig_f > 0.0) ? normal() * sig_f * ns_scale : 0.0);
        const double ns = str + p.skid_per_m * fwd + p.skid_per_rad * yaw
                        + ((sig_s > 0.0) ? normal() * sig_s * ns_scale : 0.0);
        const double nd = yaw + ((sig_r > 0.0) ? normal() * sig_r * ns_scale : 0.0);
        const double th0 = p.theta;
        const double mid = th0 + nd * 0.5;
        tracking::integrate(p.x, p.y, th0, th0 + nd, nf, ns, mid);
        p.theta = th0 + nd;
        clampParticle(p);
    }
    recomputeEstimate();
}

void SlipEnsemble::nudge(const DistanceReading* distance, int count, double heading_rad) {
    if (count < 0) count = 0;
    if (count > Robot::Range::COUNT) count = Robot::Range::COUNT;

    BeamPrep beams[Robot::Range::COUNT]{};
    for (int i = 0; i < count; ++i) {
        if (!distance || !distance[i].valid) continue;
        const double z = distance[i].range_m;
        const double sigma = Robot::rangeSigmaM(z, distance[i].confidence);
        if (!(sigma > 0.0)) continue;
        BeamPrep& b = beams[i];
        b.use = true;
        b.z = z;
        b.inv_var = 1.0 / (sigma * sigma);
    }

    const double inv_th = (cfg_.sigma_heading > 0.0)
                              ? (1.0 / (cfg_.sigma_heading * cfg_.sigma_heading))
                              : 0.0;
    const double gate = cfg_.range_gate_m;

    // Which beams agree with each other, given the IMU heading. One blocked
    // return makes the joint solve fail; dropping that beam recovers it.
    // A particle that is simply in the wrong place still uses the beams that
    // agree, so the step can be larger than the gate.
    bool trust[Robot::Range::COUNT] = {};
    bool have_trust = false;
    int nvalid = 0;
    for (int i = 0; i < count; ++i) if (beams[i].use) ++nvalid;
    if (distance && nvalid >= 2) {
        DistanceReading trial[Robot::Range::COUNT]{};
        for (int i = 0; i < count; ++i) trial[i] = distance[i];
        RangeSolve sol = wallrange::solveRanges(heading_rad, trial, count);
        for (int attempt = 0; attempt < 2 && !sol.ok && nvalid >= 3; ++attempt) {
            RangeSolve best{};
            int drop = -1;
            for (int i = 0; i < count; ++i) {
                if (!trial[i].valid) continue;
                trial[i].valid = false;
                const RangeSolve alt = wallrange::solveRanges(heading_rad, trial, count);
                trial[i].valid = true;
                if (alt.ok && (drop < 0 || alt.mae_m < best.mae_m)) {
                    best = alt;
                    drop = i;
                }
            }
            if (drop < 0) break;
            trial[drop].valid = false;
            --nvalid;
            sol = best;
        }
        if (sol.ok) {
            int ntrust = 0;
            for (int i = 0; i < count; ++i) {
                if (!beams[i].use) continue;
                const WallHit hit = wallrange::predictWallRange(
                    Pose2d{sol.x, sol.y, heading_rad}, i);
                if (hit.hit && std::fabs(beams[i].z - hit.range_m) <= gate) {
                    trust[i] = true;
                    ++ntrust;
                }
            }
            have_trust = ntrust > 0;
        }
    }

    for (int pi = 0; pi < cfg_.count; ++pi) {
        Particle& p = particles_[pi];
        // The particle is the tracking centre, and the beams are offset from it.
        const Pose2d here{p.x, p.y, p.theta};
        WallHit hits[Robot::Range::COUNT]{};
        double log_lik = 0.0;
        for (int i = 0; i < count; ++i) {
            if (!beams[i].use) continue;
            if (have_trust && !trust[i]) continue;
            hits[i] = wallrange::predictWallRange(here, i);
            double term = kMaxPenalty;
            if (hits[i].hit) {
                const double err = beams[i].z - hits[i].range_m;
                term = -0.5 * err * err * beams[i].inv_var;
                if (term < kMaxPenalty) term = kMaxPenalty;
            }
            log_lik += term;
        }
        if (inv_th > 0.0) {
            const double eth = mathutil::normalize_angle(heading_rad - p.theta);
            log_lik += -0.5 * eth * eth * inv_th;
        }
        p.log_w = log_lik;

        double A[3][3] = {};
        double b[3] = {};
        for (int i = 0; i < count; ++i) {
            if (!beams[i].use || !hits[i].hit) continue;
            if (have_trust && !trust[i]) continue;
            const double hx = hits[i].hx;
            const double hy = hits[i].hy;
            const double ht = hits[i].htheta;
            const double g2 = hx * hx + hy * hy;
            if (!(g2 <= kMaxGradSq) || !(std::fabs(ht) <= kMaxHTheta)) continue;
            double innov = beams[i].z - hits[i].range_m;
            // No cross-check: clamp the residual so one bad beam cannot
            // demand a step of its full error. The gain and the step cap
            // still turn that into a nudge.
            if (!have_trust) innov = mathutil::clamp_val(innov, -gate, gate);
            addMeas(A, b, hx, hy, ht, innov, beams[i].inv_var);
        }
        if (inv_th > 0.0) {
            const double eth = mathutil::normalize_angle(heading_rad - p.theta);
            A[2][2] += inv_th;
            b[2] += inv_th * eth;
        }
        A[1][0] = A[0][1];
        A[2][0] = A[0][2];
        A[2][1] = A[1][2];
        A[0][0] += 1e-8;
        A[1][1] += 1e-8;
        A[2][2] += 1e-8;

        double step[3];
        if (!solve3(A, b, step)) continue;
        const double dx = mathutil::clamp_val(step[0] * cfg_.nudge_gain,
                                              -cfg_.nudge_step_m, cfg_.nudge_step_m);
        const double dy = mathutil::clamp_val(step[1] * cfg_.nudge_gain,
                                              -cfg_.nudge_step_m, cfg_.nudge_step_m);
        const double dth = mathutil::clamp_val(step[2] * cfg_.nudge_gain,
                                               -cfg_.nudge_step_rad, cfg_.nudge_step_rad);
        p.x += dx;
        p.y += dy;
        p.theta += dth;
        clampParticle(p);
    }

    normalize();
    recomputeEstimate();
}

void SlipEnsemble::correctPosition(double x_m, double y_m, double sigma_m) {
    correctPose(Pose2d{x_m, y_m, pose_.theta}, sigma_m, 0.0);
}

void SlipEnsemble::correctPose(const Pose2d& pose, double sigma_xy_m, double sigma_theta_rad) {
    if (!(sigma_xy_m > 0.0) && !(sigma_theta_rad > 0.0)) return;
    const double inv_xy = (sigma_xy_m > 0.0) ? (1.0 / (sigma_xy_m * sigma_xy_m)) : 0.0;
    const double inv_th = (sigma_theta_rad > 0.0) ? (1.0 / (sigma_theta_rad * sigma_theta_rad)) : 0.0;
    const double meas_theta = mathutil::normalize_angle(pose.theta);

    double best_d2 = 1.0e300;
    for (int i = 0; i < cfg_.count; ++i) {
        const double dx = particles_[i].x - pose.x;
        const double dy = particles_[i].y - pose.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) best_d2 = d2;
    }
    const bool kidnap = (sigma_xy_m > 0.0)
                        && best_d2 > (kKidnapSigmas * sigma_xy_m) * (kKidnapSigmas * sigma_xy_m);

    double gain = 1.0;
    if (!kidnap && sigma_xy_m > 0.0) {
        gain = mathutil::clamp_val(Slip::FIX_GAIN_NUM / sigma_xy_m, Slip::FIX_GAIN_MIN, 1.0);
    }

    for (int i = 0; i < cfg_.count; ++i) {
        Particle& p = particles_[i];
        double log_lik = 0.0;
        if (inv_xy > 0.0) {
            const double dx = p.x - pose.x;
            const double dy = p.y - pose.y;
            log_lik += -0.5 * (dx * dx + dy * dy) * inv_xy;
        }
        if (inv_th > 0.0) {
            const double eth = mathutil::normalize_angle(meas_theta - p.theta);
            log_lik += -0.5 * eth * eth * inv_th;
        }
        p.log_w = log_lik;

        if (inv_xy > 0.0) {
            if (kidnap) {
                p.x = pose.x;
                p.y = pose.y;
            } else {
                p.x += gain * (pose.x - p.x);
                p.y += gain * (pose.y - p.y);
            }
        }
        if (inv_th > 0.0) {
            const double eth = mathutil::normalize_angle(meas_theta - p.theta);
            if (kidnap) p.theta = meas_theta;
            else p.theta = p.theta + gain * eth;
        }
        clampParticle(p);
    }
    normalize();
    recomputeEstimate();
}

void SlipEnsemble::normalize() {
    const int n = cfg_.count;
    double max_lw = particles_[0].log_w;
    for (int i = 1; i < n; ++i) {
        if (particles_[i].log_w > max_lw) max_lw = particles_[i].log_w;
    }
    if (!std::isfinite(max_lw)) {
        setUniformWeights();
        return;
    }

    double sum = 0.0;
    for (int i = 0; i < n; ++i) {
        particles_[i].log_w -= max_lw;
        sum += std::exp(particles_[i].log_w);
    }
    if (!(sum > 0.0) || !std::isfinite(sum)) {
        setUniformWeights();
        return;
    }
    const double log_sum = std::log(sum);
    for (int i = 0; i < n; ++i) particles_[i].log_w -= log_sum;
}

void SlipEnsemble::recomputeEstimate() {
    const int n = cfg_.count;
    double mx = 0.0, my = 0.0, sc = 0.0, ss = 0.0, w2 = 0.0;
    for (int i = 0; i < n; ++i) {
        const double w = std::exp(particles_[i].log_w);
        mx += w * particles_[i].x;
        my += w * particles_[i].y;
        sc += w * std::cos(particles_[i].theta);
        ss += w * std::sin(particles_[i].theta);
        w2 += w * w;
    }
    pose_.x = mx;
    pose_.y = my;
    pose_.theta = std::atan2(ss, sc);
    n_eff_ = (w2 > 0.0) ? (1.0 / w2) : static_cast<double>(n);

    double var = 0.0;
    for (int i = 0; i < n; ++i) {
        const double w = std::exp(particles_[i].log_w);
        const double dx = particles_[i].x - mx;
        const double dy = particles_[i].y - my;
        var += w * (dx * dx + dy * dy);
    }
    pos_spread_ = (var > 0.0) ? std::sqrt(var) : 0.0;
}
