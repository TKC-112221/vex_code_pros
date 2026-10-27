/* WallRange.cpp -- ray cast from a chassis-mounted distance sensor to the field wall. */
#include "math/WallRange.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include "math/MathUtils.h"
#include <cmath>

namespace {

struct Beam {
    double ox = 0, oy = 0;
    double dx = 0, dy = 0;
    double dsx_dth = 0, dsy_dth = 0;
};

Beam beamAt(const Pose2d& pose, int sensor_index) {
    const Robot::Range::Mount& mount = Robot::Range::MOUNT[sensor_index];
    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    const double a = pose.theta + mount.yaw;
    Beam b;
    b.ox = pose.x + mount.x * c - mount.y * s;
    b.oy = pose.y + mount.x * s + mount.y * c;
    b.dx = std::cos(a);
    b.dy = std::sin(a);
    b.dsx_dth = -mount.x * s - mount.y * c;
    b.dsy_dth =  mount.x * c - mount.y * s;
    return b;
}

// axis 0: the hit is on an x = const wall. axis 1: y = const.
template <typename T>
struct RayPick {
    bool hit = false;
    T t = static_cast<T>(1.0e9);
    int axis = 0;
};

template <typename T>
RayPick<T> closestWall(T ox, T oy, T dx, T dy) {
    const T x_lo = static_cast<T>(Robot::Range::FIELD_X_MIN_M);
    const T x_hi = static_cast<T>(Robot::Range::FIELD_X_MAX_M);
    const T y_lo = static_cast<T>(Robot::Range::FIELD_Y_MIN_M);
    const T y_hi = static_cast<T>(Robot::Range::FIELD_Y_MAX_M);
    const T eps = static_cast<T>(1.0e-6);
    const T t_min = static_cast<T>(1.0e-4);
    const T dir_min = static_cast<T>(1.0e-8);
    RayPick<T> best;
    auto take = [&](T t, T along, T along_lo, T along_hi, int axis) {
        if (!(t > t_min) || t >= best.t) return;
        if (along < along_lo - eps || along > along_hi + eps) return;
        if (!std::isfinite(t)) return;
        best.hit = true;
        best.t = t;
        best.axis = axis;
    };
    if (std::fabs(dx) > dir_min) {
        const T walls[2] = { x_hi, x_lo };
        for (T W : walls) take((W - ox) / dx, oy + ((W - ox) / dx) * dy, y_lo, y_hi, 0);
    }
    if (std::fabs(dy) > dir_min) {
        const T walls[2] = { y_hi, y_lo };
        for (T W : walls) take((W - oy) / dy, ox + ((W - oy) / dy) * dx, x_lo, x_hi, 1);
    }
    return best;
}

template <typename T>
T beamLength(T x, T y, T cos_th, T sin_th, T mount_x, T mount_y, T cos_yaw, T sin_yaw) {
    const T ox = x + mount_x * cos_th - mount_y * sin_th;
    const T oy = y + mount_x * sin_th + mount_y * cos_th;
    const T dx = cos_th * cos_yaw - sin_th * sin_yaw;
    const T dy = sin_th * cos_yaw + cos_th * sin_yaw;
    const RayPick<T> pick = closestWall(ox, oy, dx, dy);
    return pick.hit ? pick.t : static_cast<T>(-1);
}

RangeSolve refine(double theta, const DistanceReading* distance, int count,
                  double x, double y) {
    const double margin = WallSolve::FIELD_MARGIN_M;
    RangeSolve sol;
    sol.x = x;
    sol.y = y;

    for (int iter = 0; iter < WallSolve::ITERATIONS; ++iter) {
        double a00 = 0, a01 = 0, a11 = 0, b0 = 0, b1 = 0;
        for (int i = 0; i < count; ++i) {
            if (!distance[i].valid) continue;
            const WallHit hit = wallrange::predictWallRange(Pose2d{sol.x, sol.y, theta}, i);
            if (!hit.hit) continue;
            const double innov = distance[i].range_m - hit.range_m;
            if (iter > 1 && std::fabs(innov) > WallSolve::INNOV_GATE_M) continue;
            const double jx = hit.hx;
            const double jy = hit.hy;
            if (jx * jx + jy * jy < 1e-8) continue;
            a00 += jx * jx;
            a01 += jx * jy;
            a11 += jy * jy;
            b0 += jx * innov;
            b1 += jy * innov;
        }
        const double det = a00 * a11 - a01 * a01;
        if (!(std::fabs(det) > 1e-12)) break;
        double step_x = (a11 * b0 - a01 * b1) / det;
        double step_y = (a00 * b1 - a01 * b0) / det;
        if (!std::isfinite(step_x) || !std::isfinite(step_y)) break;
        if (step_x > 0.50) step_x = 0.50;
        if (step_x < -0.50) step_x = -0.50;
        if (step_y > 0.50) step_y = 0.50;
        if (step_y < -0.50) step_y = -0.50;
        sol.x += step_x;
        sol.y += step_y;
        Robot::clampFieldMeters(sol.x, sol.y, margin);
        if (step_x * step_x + step_y * step_y < 1e-8) break;
    }

    double abs_err = 0;
    int used = 0;
    for (int i = 0; i < count; ++i) {
        if (!distance[i].valid) continue;
        const WallHit hit = wallrange::predictWallRange(Pose2d{sol.x, sol.y, theta}, i);
        if (!hit.hit) continue;
        abs_err += std::fabs(distance[i].range_m - hit.range_m);
        ++used;
    }
    if (used >= 2) {
        sol.mae_m = abs_err / static_cast<double>(used);
        sol.ok = sol.mae_m < 0.15 && std::isfinite(sol.x) && std::isfinite(sol.y);
    }
    return sol;
}

}  // namespace

namespace wallrange {

WallHit predictWallRange(const Pose2d& pose, int sensor_index) {
    if (sensor_index < 0 || sensor_index >= Robot::Range::COUNT) return WallHit{};

    const Beam b = beamAt(pose, sensor_index);
    const RayPick<double> pick = closestWall(b.ox, b.oy, b.dx, b.dy);
    if (!pick.hit) return WallHit{};

    WallHit best;
    best.hit = true;
    best.range_m = pick.t;
    if (pick.axis == 0) {
        best.hx = -1.0 / b.dx;
        best.hy = 0.0;
        best.htheta = -b.dsx_dth / b.dx + pick.t * b.dy / b.dx;
    } else {
        best.hx = 0.0;
        best.hy = -1.0 / b.dy;
        best.htheta = -b.dsy_dth / b.dy - pick.t * b.dx / b.dy;
    }
    if (!std::isfinite(best.hx) || !std::isfinite(best.hy) || !std::isfinite(best.htheta)) {
        return WallHit{};
    }
    return best;
}

double beamRange(double x, double y, double cos_th, double sin_th,
                 double mount_x, double mount_y, double cos_yaw, double sin_yaw) {
    return beamLength(x, y, cos_th, sin_th, mount_x, mount_y, cos_yaw, sin_yaw);
}

float beamRange(float x, float y, float cos_th, float sin_th,
                float mount_x, float mount_y, float cos_yaw, float sin_yaw) {
    return beamLength(x, y, cos_th, sin_th, mount_x, mount_y, cos_yaw, sin_yaw);
}

RangeSolve solveRanges(double theta, const DistanceReading* distance, int count) {
    RangeSolve none;
    if (!distance || count <= 0) return none;
    if (count > Robot::Range::COUNT) count = Robot::Range::COUNT;

    int nvalid = 0;
    for (int i = 0; i < count; ++i) if (distance[i].valid) ++nvalid;
    if (nvalid < 2) return none;

    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double x_lo = Robot::Range::FIELD_X_MIN_M;
    const double x_hi = Robot::Range::FIELD_X_MAX_M;
    const double y_lo = Robot::Range::FIELD_Y_MIN_M;
    const double y_hi = Robot::Range::FIELD_Y_MAX_M;
    double gx = 0, gy = 0;
    int nx = 0, ny = 0;
    for (int i = 0; i < count; ++i) {
        if (!distance[i].valid) continue;
        const Robot::Range::Mount& mount = Robot::Range::MOUNT[i];
        const double a = theta + mount.yaw;
        const double dx = std::cos(a);
        const double dy = std::sin(a);
        const double rx = mount.x * c - mount.y * s;
        const double ry = mount.x * s + mount.y * c;
        if (std::fabs(dx) >= std::fabs(dy)) {
            const double W = (dx >= 0.0) ? x_hi : x_lo;
            gx += W - distance[i].range_m * dx - rx;
            ++nx;
        } else {
            const double W = (dy >= 0.0) ? y_hi : y_lo;
            gy += W - distance[i].range_m * dy - ry;
            ++ny;
        }
    }
    const double x0 = (nx > 0) ? gx / static_cast<double>(nx) : Robot::Range::CENTER_X_M;
    const double y0 = (ny > 0) ? gy / static_cast<double>(ny) : Robot::Range::CENTER_Y_M;

    const RangeSolve from_beams = refine(theta, distance, count, x0, y0);
    const RangeSolve from_center = refine(theta, distance, count,
                                          Robot::Range::CENTER_X_M, Robot::Range::CENTER_Y_M);
    if (from_beams.ok && from_center.ok) {
        return (from_beams.mae_m <= from_center.mae_m) ? from_beams : from_center;
    }
    if (from_beams.ok) return from_beams;
    if (from_center.ok) return from_center;
    return (from_beams.mae_m <= from_center.mae_m) ? from_beams : from_center;
}

bool nudgePosition(double& x, double& y, double theta,
                   const DistanceReading* distance, int count,
                   double gain, double step_m, double gate_m) {
    if (!distance || count <= 0) return false;
    if (!(gain > 0.0) || !(step_m > 0.0)) return false;
    if (count > Robot::Range::COUNT) count = Robot::Range::COUNT;
    if (gate_m < 0.0) gate_m = 0.0;

    struct Beam {
        bool use = false;
        double z = 0;
        double inv_var = 0;
    };
    Beam beams[Robot::Range::COUNT]{};
    int nvalid = 0;
    for (int i = 0; i < count; ++i) {
        if (!distance[i].valid) continue;
        const double z = distance[i].range_m;
        const double sigma = Robot::rangeSigmaM(z, distance[i].confidence);
        if (!(sigma > 0.0)) continue;
        beams[i].use = true;
        beams[i].z = z;
        beams[i].inv_var = 1.0 / (sigma * sigma);
        ++nvalid;
    }
    if (nvalid == 0) return false;

    // Beams that agree with each other. Dropping one blocked return lets the
    // rest name a pose; the odometry state can still sit far from that pose.
    bool trust[Robot::Range::COUNT] = {};
    bool have_trust = false;
    if (nvalid >= 2) {
        DistanceReading trial[Robot::Range::COUNT]{};
        for (int i = 0; i < count; ++i) trial[i] = distance[i];
        RangeSolve sol = solveRanges(theta, trial, count);
        for (int attempt = 0; attempt < 2 && !sol.ok && nvalid >= 3; ++attempt) {
            RangeSolve best{};
            int drop = -1;
            for (int i = 0; i < count; ++i) {
                if (!trial[i].valid) continue;
                trial[i].valid = false;
                const RangeSolve alt = solveRanges(theta, trial, count);
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
                const WallHit hit = predictWallRange(Pose2d{sol.x, sol.y, theta}, i);
                if (hit.hit && std::fabs(beams[i].z - hit.range_m) <= gate_m) {
                    trust[i] = true;
                    ++ntrust;
                }
            }
            have_trust = ntrust > 0;
        }
    }

    double a00 = 0, a01 = 0, a11 = 0, b0 = 0, b1 = 0;
    const Pose2d here{x, y, theta};
    for (int i = 0; i < count; ++i) {
        if (!beams[i].use) continue;
        if (have_trust && !trust[i]) continue;
        const WallHit hit = predictWallRange(here, i);
        if (!hit.hit) continue;
        const double g2 = hit.hx * hit.hx + hit.hy * hit.hy;
        if (!(g2 <= 16.0)) continue;
        double innov = beams[i].z - hit.range_m;
        if (!have_trust) innov = mathutil::clamp_val(innov, -gate_m, gate_m);
        const double w = beams[i].inv_var;
        a00 += w * hit.hx * hit.hx;
        a01 += w * hit.hx * hit.hy;
        a11 += w * hit.hy * hit.hy;
        b0 += w * hit.hx * innov;
        b1 += w * hit.hy * innov;
    }
    a00 += 1e-8;
    a11 += 1e-8;
    const double det = a00 * a11 - a01 * a01;
    if (!(std::fabs(det) > 1e-12)) return false;
    double dx = (a11 * b0 - a01 * b1) / det;
    double dy = (a00 * b1 - a01 * b0) / det;
    if (!std::isfinite(dx) || !std::isfinite(dy)) return false;
    dx = mathutil::clamp_val(dx * gain, -step_m, step_m);
    dy = mathutil::clamp_val(dy * gain, -step_m, step_m);
    x += dx;
    y += dy;

    Robot::clampFieldMeters(x, y, 0.18);
    return true;
}

}  // namespace wallrange
