/* ---------------------------------------------------------------------------
 * SensorSavitzkyGolay.cpp  --  causal Savitzky-Golay for sensor noise.
 *
 * Weights solve the normal equations of a polynomial fit and are evaluated
 * at the newest sample, so the filter does not wait for a centered window.
 * Positions inside a window are 0, -1, -2, ... in samples; the constant
 * basis column of the inverse Gram matrix is the weight vector. A shorter
 * window, used until the buffer fills, drops the degree so the fit stays
 * determined. Heading and range samples are transported by the measured
 * motion first: a perfect prediction makes every sample in the window equal
 * the current reading, and the fit only has to reject noise.
 * ------------------------------------------------------------------------- */
#include "math/SensorSavitzkyGolay.h"

#include "core/Specs.h"

#include <cmath>

namespace {

constexpr double absd(double v) { return v < 0.0 ? -v : v; }

constexpr double ipow(double base, int exp) {
    double r = 1.0;
    for (int i = 0; i < exp; ++i) r *= base;
    return r;
}

struct SgWeights {
    // w[n][i] multiplies sample i in a window of length n. i = 0 is the
    // oldest sample; i = n - 1 is the newest, where the polynomial is read.
    double w[kSavitzkyGolayWindow + 1][kSavitzkyGolayWindow]{};
};

constexpr SgWeights makeWeights() {
    SgWeights out{};
    for (int n = 1; n <= kSavitzkyGolayWindow; ++n) {
        const int deg = (n - 1 < kSavitzkyGolayDegree) ? (n - 1) : kSavitzkyGolayDegree;
        const int m = deg + 1;

        double gram[kSavitzkyGolayDegree + 1][kSavitzkyGolayDegree + 1] = {};
        for (int i = 0; i < n; ++i) {
            const double t = static_cast<double>(i - (n - 1));
            for (int a = 0; a < m; ++a) {
                for (int b = 0; b < m; ++b) gram[a][b] += ipow(t, a + b);
            }
        }

        // Augment with e_0 and solve. Only that column of the inverse is
        // needed: the smoothed value is the constant term of the polynomial
        // written about the newest sample.
        double aug[kSavitzkyGolayDegree + 1][kSavitzkyGolayDegree + 2] = {};
        for (int r = 0; r < m; ++r) {
            for (int c = 0; c < m; ++c) aug[r][c] = gram[r][c];
            aug[r][m] = (r == 0) ? 1.0 : 0.0;
        }
        for (int col = 0; col < m; ++col) {
            int piv = col;
            double best = absd(aug[col][col]);
            for (int r = col + 1; r < m; ++r) {
                const double v = absd(aug[r][col]);
                if (v > best) {
                    best = v;
                    piv = r;
                }
            }
            if (piv != col) {
                for (int c = col; c <= m; ++c) {
                    const double tmp = aug[col][c];
                    aug[col][c] = aug[piv][c];
                    aug[piv][c] = tmp;
                }
            }
            const double div = aug[col][col];
            for (int c = col; c <= m; ++c) aug[col][c] /= div;
            for (int r = 0; r < m; ++r) {
                if (r == col) continue;
                const double f = aug[r][col];
                for (int c = col; c <= m; ++c) aug[r][c] -= f * aug[col][c];
            }
        }

        for (int i = 0; i < n; ++i) {
            const double t = static_cast<double>(i - (n - 1));
            double weight = 0.0;
            for (int a = 0; a < m; ++a) weight += ipow(t, a) * aug[a][m];
            out.w[n][i] = weight;
        }
    }
    return out;
}

constexpr SgWeights kWeights = makeWeights();

constexpr bool momentsHold(int n) {
    const int deg = (n - 1 < kSavitzkyGolayDegree) ? (n - 1) : kSavitzkyGolayDegree;
    for (int k = 0; k <= deg; ++k) {
        double moment = 0.0;
        for (int i = 0; i < n; ++i) {
            const double t = static_cast<double>(i - (n - 1));
            moment += kWeights.w[n][i] * ipow(t, k);
        }
        const double want = (k == 0) ? 1.0 : 0.0;
        if (absd(moment - want) > 1e-8) return false;
    }
    return true;
}

constexpr double noiseVariance() {
    double s = 0.0;
    for (int i = 0; i < kSavitzkyGolayWindow; ++i) {
        const double c = kWeights.w[kSavitzkyGolayWindow][i];
        s += c * c;
    }
    return s;
}

static_assert(kSavitzkyGolayWindow >= 1);
static_assert(kSavitzkyGolayDegree >= 0);
static_assert(kSavitzkyGolayDegree < kSavitzkyGolayWindow);
static_assert(momentsHold(1));
static_assert(momentsHold(kSavitzkyGolayDegree + 1));
static_assert(momentsHold(kSavitzkyGolayWindow));
static_assert(noiseVariance() < 1.0);

class ScalarSavitzkyGolay {
public:
    void reset() { n_ = 0; }

    // predicted_delta is the change expected from a measured rate over this
    // sample. It is applied only after the first sample: the opening point
    // has no previous sample to transport.
    double filter(double measurement, double predicted_delta = 0.0) {
        if (n_ < kSavitzkyGolayWindow) {
            delta_[n_] = (n_ == 0) ? 0.0 : predicted_delta;
            sample_[n_] = measurement;
            ++n_;
        } else {
            for (int i = 1; i < kSavitzkyGolayWindow; ++i) {
                sample_[i - 1] = sample_[i];
                delta_[i - 1] = delta_[i];
            }
            sample_[kSavitzkyGolayWindow - 1] = measurement;
            delta_[kSavitzkyGolayWindow - 1] = predicted_delta;
        }

        double aligned[kSavitzkyGolayWindow];
        double transport = 0.0;
        for (int i = n_ - 1; i >= 0; --i) {
            aligned[i] = sample_[i] + transport;
            transport += delta_[i];
        }

        double y = 0.0;
        for (int i = 0; i < n_; ++i) y += kWeights.w[n_][i] * aligned[i];
        return y;
    }

private:
    double sample_[kSavitzkyGolayWindow]{};
    double delta_[kSavitzkyGolayWindow]{};
    int n_ = 0;
};

struct SensorFilters {
    ScalarSavitzkyGolay heading;
    ScalarSavitzkyGolay v_lin;
    ScalarSavitzkyGolay v_ang;
    ScalarSavitzkyGolay left_vel_rpm;
    ScalarSavitzkyGolay right_vel_rpm;
    ScalarSavitzkyGolay range[kDistanceChannels];
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

double filterChannel(ScalarSavitzkyGolay& channel, double sample, double predicted_delta = 0.0) {
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
            if (++g_filters.range_miss[i] >= kSavitzkyGolayWindow) {
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

void applySensorSavitzkyGolay(SensorDict& sensors) {
    // Travel and yaw rate before filterChassis replaces v_ang with the smoothed
    // value. The deltas themselves are never filtered.
    const double forward = sensors.chassis.forward_delta;
    const double strafe = sensors.chassis.strafe_delta;
    const double v_ang = sensors.chassis.v_ang;
    filterChassis(sensors.chassis);
    filterDistances(sensors.distance, forward, strafe, v_ang);
}

void resetSensorSavitzkyGolay() {
    g_filters.reset();
}
