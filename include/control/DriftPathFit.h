#pragma once
/* ---------------------------------------------------------------------------
 * DriftPathFit  --  the DriftCurve shapes and the straight finish.
 *
 * Shared by DriftCurve, which refits every tick, and DriftFollower, which
 * samples a path once. The geometry matches the notes in DriftCurve.h.
 * ------------------------------------------------------------------------- */

#include "core/Specs.h"
#include "core/Tuning.h"
#include "math/MathUtils.h"
#include <cmath>

namespace driftpath {

// kOnAxisM, the Hermite lookahead, and kLengthSteps live in core/Tuning.h.

// The curve is fitted to the far end of a straight finish of `line_length`
// meters. The finish runs into the target along `approach`. on_line means
// the robot is already on that segment. past means it has crossed the target.
struct Finish {
    double jx = 0, jy = 0;
    double length = 0;
    double along = 0;
    bool on_line = false;
    bool past = false;
};

inline Finish finishOf(double rx, double ry, double tx, double ty,
                double approach, double line_length) {
    Finish fin;
    fin.length = line_length > 0.0 ? line_length : 0.0;
    const double ux = std::cos(approach);
    const double uy = std::sin(approach);
    fin.jx = tx - fin.length * ux;
    fin.jy = ty - fin.length * uy;
    fin.along = (rx - tx) * ux + (ry - ty) * uy;
    fin.past = fin.along > 1e-3;
    fin.on_line = fin.length > 1e-4 && !fin.past && fin.along >= -fin.length - 1e-4;
    return fin;
}

// Circle through the robot, tangent to `approach` at the target.
// phi is the travel bearing, kappa is signed CCW curvature, length_m is the
// arc length still to drive in that direction.
struct Arc {
    double phi = 0;
    double kappa = 0;
    double length_m = 0;
    double cx = 0, cy = 0, r = 0, sweep = 0;
    bool curved = false;
};

inline Arc arcThrough(double rx, double ry, double tx, double ty, double approach) {
    const double dx = rx - tx;
    const double dy = ry - ty;
    const double ux = std::cos(approach);
    const double uy = std::sin(approach);
    const double nx = -uy; // left of the arrival heading
    const double ny = ux;
    const double along = dx * ux + dy * uy;
    const double lat = dx * nx + dy * ny;
    const double dist2 = dx * dx + dy * dy;
    const double dist = std::sqrt(dist2);

    Arc arc;
    arc.phi = approach;
    if (dist < 1e-4) return arc;

    // On the heading axis the circle is the line itself.
    if (std::fabs(lat) < kOnAxisM) {
        arc.length_m = dist;
        if (along > 0.0) arc.phi = std::atan2(ty - ry, tx - rx);
        return arc;
    }

    const double r = dist2 / (2.0 * lat);
    const double cx = tx + r * nx;
    const double cy = ty + r * ny;
    const double ox = rx - cx;
    const double oy = ry - cy;
    const double tcx = tx - cx;
    const double tcy = ty - cy;

    // CCW tangent of (robot - center) is (-oy, ox). Positive r travels CCW.
    double dir_x = -oy;
    double dir_y = ox;
    if (r < 0.0) {
        dir_x = -dir_x;
        dir_y = -dir_y;
    }
    arc.phi = std::atan2(dir_y, dir_x);
    arc.kappa = 1.0 / r;
    arc.cx = cx;
    arc.cy = cy;
    arc.r = r;
    arc.curved = true;

    // Sweep from the robot to the target in the travel direction, in [0, 2π).
    const double dot = ox * tcx + oy * tcy;
    const double cross = ox * tcy - oy * tcx;
    const double delta = std::atan2(cross, dot);
    double sweep;
    if (r > 0.0) sweep = (delta >= 0.0) ? delta : (delta + 2.0 * Robot::PI);
    else         sweep = (delta <= 0.0) ? -delta : (2.0 * Robot::PI - delta);
    arc.sweep = sweep;
    arc.length_m = std::fabs(r) * sweep;
    return arc;
}

// e^{z} − 1 − z, and its derivatives with respect to z. Series near 0.
inline void expBasis(double z, double& f, double& fp, double& fpp) {
    if (z > 40.0) z = 40.0;
    if (z < -40.0) z = -40.0;
    if (std::fabs(z) < 1e-2) {
        f = (z * z) * (0.5 + z * (1.0 / 6.0 + z / 24.0));
        fp = z * (1.0 + z * (0.5 + z / 6.0));
        fpp = 1.0 + z * (1.0 + z * 0.5);
        return;
    }
    const double e = std::exp(z);
    f = e - 1.0 - z;
    fp = e - 1.0;
    fpp = e;
}

// Graph over the heading axis, flat at the target, through the robot.
// Exponential: lateral ∝ e^{k ξ} − 1 − k ξ. Zero rate is the parabola ξ².
// Quartic: lateral ∝ ξ⁴, so curvature and its slope vanish at the join.
struct GraphPath {
    double phi = 0;
    double kappa = 0;
    double length_m = 0;
    bool graph = false;
    bool quartic = false;
    bool momentum = false;       // QuarticHermite: η/η0 = c6 u^6 + c5 u^5 + c4 u^4
    double c6 = 0, c5 = 0, c4 = 1;
    double ux = 1, uy = 0, nx = 0, ny = 1;
    double tx = 0, ty = 0;
    double xi0 = 0, eta0 = 0, rate = 0;
};

inline double etaOf(double xi, double xi0, double eta0, double rate) {
    if (std::fabs(xi0) < 1e-6) return 0.0;
    if (std::fabs(rate) < 1e-6) return eta0 * (xi * xi) / (xi0 * xi0);
    double f0, fp0, fpp0, f, fp, fpp;
    expBasis(rate * xi0, f0, fp0, fpp0);
    expBasis(rate * xi, f, fp, fpp);
    if (std::fabs(f0) < 1e-9) return 0.0;
    return eta0 * (f / f0);
}

// ξ⁴ through the robot. Fourth-order contact with the join: η, η', η'', η'''
// are all zero there, so κ and dκ/ds match the straight finish.
inline double etaQuartic(double xi, double xi0, double eta0) {
    if (std::fabs(xi0) < 1e-6) return 0.0;
    const double u = xi / xi0;
    const double u2 = u * u;
    return eta0 * u2 * u2;
}

inline double graphEta(const GraphPath& path, double xi) {
    if (path.momentum) {
        if (std::fabs(path.xi0) < 1e-6) return 0.0;
        const double u = xi / path.xi0;
        const double u2 = u * u;
        const double u4 = u2 * u2;
        return path.eta0 * u4 * (path.c6 * u2 + path.c5 * u + path.c4);
    }
    if (path.quartic) return etaQuartic(xi, path.xi0, path.eta0);
    return etaOf(xi, path.xi0, path.eta0, path.rate);
}

inline double graphLength(const GraphPath& path, double rx, double ry) {
    double px = rx, py = ry;
    double length = 0.0;
    for (int i = 1; i <= kLengthSteps; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(kLengthSteps);
        const double xi = path.xi0 * (1.0 - u);
        const double eta = graphEta(path, xi);
        const double x = path.tx + xi * path.ux + eta * path.nx;
        const double y = path.ty + xi * path.uy + eta * path.ny;
        length += std::hypot(x - px, y - py);
        px = x;
        py = y;
    }
    return length;
}

// Tangent and curvature of a QuarticHermite graph at along-track ξ.
inline void graphFrame(const GraphPath& path, double xi, double& phi, double& kappa) {
    const double u = (std::fabs(path.xi0) < 1e-9) ? 0.0 : xi / path.xi0;
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u2 * u2;
    const double dp = 6.0 * path.c6 * u4 * u + 5.0 * path.c5 * u4 + 4.0 * path.c4 * u3;
    const double ddp = 30.0 * path.c6 * u4 + 20.0 * path.c5 * u3 + 12.0 * path.c4 * u2;
    const double m = (std::fabs(path.xi0) < 1e-9) ? 0.0 : (path.eta0 / path.xi0) * dp;
    const double eta_pp = (std::fabs(path.xi0) < 1e-9) ? 0.0
                         : (path.eta0 / (path.xi0 * path.xi0)) * ddp;
    const double speed = std::sqrt(1.0 + m * m);
    const double toward = (path.xi0 > 0.0) ? -1.0 : 1.0;
    const double dir_u = toward;
    const double dir_n = toward * m;
    phi = std::atan2(dir_u * path.uy + dir_n * path.ny,
                     dir_u * path.ux + dir_n * path.nx);
    kappa = toward * eta_pp / (speed * speed * speed);
}

// Tangent and curvature at arc length `distance` from the robot.
inline void graphLook(const GraphPath& path, double distance, double& phi, double& kappa) {
    if (distance <= 1e-4) {
        graphFrame(path, path.xi0, phi, kappa);
        return;
    }
    double px = path.tx + path.xi0 * path.ux + path.eta0 * path.nx;
    double py = path.ty + path.xi0 * path.uy + path.eta0 * path.ny;
    double length = 0.0;
    for (int i = 1; i <= kLengthSteps; ++i) {
        const double frac_i = static_cast<double>(i) / static_cast<double>(kLengthSteps);
        const double xi = path.xi0 * (1.0 - frac_i);
        const double eta = graphEta(path, xi);
        const double x = path.tx + xi * path.ux + eta * path.nx;
        const double y = path.ty + xi * path.uy + eta * path.ny;
        const double step = std::hypot(x - px, y - py);
        if (length + step >= distance) {
            const double blend = (step > 1e-9) ? (distance - length) / step : 1.0;
            const double prev = static_cast<double>(i - 1) / static_cast<double>(kLengthSteps);
            const double xi0 = path.xi0 * (1.0 - prev);
            graphFrame(path, xi0 + (xi - xi0) * blend, phi, kappa);
            return;
        }
        length += step;
        px = x;
        py = y;
    }
    graphFrame(path, 0.0, phi, kappa);
}

inline GraphPath graphThrough(double rx, double ry, double tx, double ty,
                       double approach, bool quartic, double rate) {
    const double dx = rx - tx;
    const double dy = ry - ty;
    const double ux = std::cos(approach);
    const double uy = std::sin(approach);
    const double nx = -uy;
    const double ny = ux;
    const double xi0 = dx * ux + dy * uy;
    const double eta0 = dx * nx + dy * ny;
    const double dist = std::hypot(dx, dy);

    GraphPath path;
    path.phi = approach;
    path.quartic = quartic;
    path.ux = ux;
    path.uy = uy;
    path.nx = nx;
    path.ny = ny;
    path.tx = tx;
    path.ty = ty;
    path.xi0 = xi0;
    path.eta0 = eta0;
    path.rate = rate;
    if (dist < 1e-4) return path;

    // Beside the target, or already on the heading line: no graph to fit.
    if (std::fabs(xi0) < kOnAxisM || std::fabs(eta0) < kOnAxisM) {
        path.length_m = dist;
        if (xi0 > 0.0) path.phi = std::atan2(ty - ry, tx - rx);
        return path;
    }

    // Slope and curvature of η(ξ) at the robot. Travel toward ξ = 0.
    // Parabola is the k → 0 limit: η ∝ ξ², so m = 2 η0/ξ0 and η'' = 2 η0/ξ0².
    // Quartic is η ∝ ξ⁴, so m = 4 η0/ξ0 and η'' = 12 η0/ξ0². η'' and η''' are
    // zero at ξ = 0, so κ and dκ/ds match the straight and α does too.
    double m = 0.0;
    double eta_pp = 0.0;
    if (quartic) {
        m = 4.0 * eta0 / xi0;
        eta_pp = 12.0 * eta0 / (xi0 * xi0);
    } else if (std::fabs(rate) < 1e-6) {
        m = 2.0 * eta0 / xi0;
        eta_pp = 2.0 * eta0 / (xi0 * xi0);
    } else {
        double f0, fp0, fpp0;
        expBasis(rate * xi0, f0, fp0, fpp0);
        if (std::fabs(f0) < 1e-9) {
            path.length_m = dist;
            return path;
        }
        m = eta0 * (rate * fp0) / f0;
        eta_pp = eta0 * (rate * rate * fpp0) / f0;
    }
    const double speed = std::sqrt(1.0 + m * m);
    const double toward = (xi0 > 0.0) ? -1.0 : 1.0;
    const double dir_u = toward;
    const double dir_n = toward * m;
    path.phi = std::atan2(dir_u * uy + dir_n * ny, dir_u * ux + dir_n * nx);
    path.kappa = toward * eta_pp / (speed * speed * speed);
    path.graph = true;
    path.length_m = graphLength(path, rx, ry);
    return path;
}

// Quintic Bezier in the heading frame. Control ξ is uniform in the
// parameter, so the image is a graph and cannot loop. See DriftCurve.h
// for the handle scaling.
struct Quintic {
    double phi = 0;
    double kappa = 0;
    double length_m = 0;
    bool curved = false;
    double px[6] {};
    double py[6] {};
};

inline void quinticAt(const Quintic& path, double t, double& x, double& y) {
    const double u = 1.0 - t;
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double b0 = u3 * u2;
    const double b1 = 5.0 * u2 * u2 * t;
    const double b2 = 10.0 * u3 * t2;
    const double b3 = 10.0 * u2 * t3;
    const double b4 = 5.0 * u * t2 * t2;
    const double b5 = t3 * t2;
    x = b0 * path.px[0] + b1 * path.px[1] + b2 * path.px[2]
      + b3 * path.px[3] + b4 * path.px[4] + b5 * path.px[5];
    y = b0 * path.py[0] + b1 * path.py[1] + b2 * path.py[2]
      + b3 * path.py[3] + b4 * path.py[4] + b5 * path.py[5];
}

inline void quinticDeriv(const Quintic& path, double t,
                  double& dx, double& dy, double& ddx, double& ddy) {
    const double u = 1.0 - t;
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double b0 = u2 * u2;
    const double b1 = 4.0 * u3 * t;
    const double b2 = 6.0 * u2 * t2;
    const double b3 = 4.0 * u * t3;
    const double b4 = t2 * t2;
    double dpx[5];
    double dpy[5];
    for (int i = 0; i < 5; ++i) {
        dpx[i] = path.px[i + 1] - path.px[i];
        dpy[i] = path.py[i + 1] - path.py[i];
    }
    dx = 5.0 * (b0 * dpx[0] + b1 * dpx[1] + b2 * dpx[2] + b3 * dpx[3] + b4 * dpx[4]);
    dy = 5.0 * (b0 * dpy[0] + b1 * dpy[1] + b2 * dpy[2] + b3 * dpy[3] + b4 * dpy[4]);
    double ddxp[4];
    double ddyp[4];
    for (int i = 0; i < 4; ++i) {
        ddxp[i] = dpx[i + 1] - dpx[i];
        ddyp[i] = dpy[i + 1] - dpy[i];
    }
    const double c0 = u3;
    const double c1 = 3.0 * u2 * t;
    const double c2 = 3.0 * u * t2;
    const double c3 = t3;
    ddx = 20.0 * (c0 * ddxp[0] + c1 * ddxp[1] + c2 * ddxp[2] + c3 * ddxp[3]);
    ddy = 20.0 * (c0 * ddyp[0] + c1 * ddyp[1] + c2 * ddyp[2] + c3 * ddyp[3]);
}

inline void quinticFrame(const Quintic& path, double t, double& phi, double& kappa) {
    double dx = 0.0, dy = 0.0, ddx = 0.0, ddy = 0.0;
    quinticDeriv(path, t, dx, dy, ddx, ddy);
    phi = std::atan2(dy, dx);
    kappa = 0.0;
    const double speed2 = dx * dx + dy * dy;
    if (speed2 > 1e-12) {
        const double speed = std::sqrt(speed2);
        kappa = (dx * ddy - dy * ddx) / (speed2 * speed);
    }
}

// Tangent and curvature at arc length `distance` from the robot.
inline void quinticLook(const Quintic& path, double distance, double& phi, double& kappa) {
    if (distance <= 1e-4) {
        quinticFrame(path, 0.0, phi, kappa);
        return;
    }
    double px = 0.0, py = 0.0;
    quinticAt(path, 0.0, px, py);
    double length = 0.0;
    for (int i = 1; i <= kLengthSteps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(kLengthSteps);
        double x = 0.0, y = 0.0;
        quinticAt(path, t, x, y);
        const double step = std::hypot(x - px, y - py);
        if (length + step >= distance) {
            const double frac = (step > 1e-9) ? (distance - length) / step : 1.0;
            const double t0 = static_cast<double>(i - 1) / static_cast<double>(kLengthSteps);
            quinticFrame(path, t0 + (t - t0) * frac, phi, kappa);
            return;
        }
        length += step;
        px = x;
        py = y;
    }
    quinticFrame(path, 1.0, phi, kappa);
}

inline double quinticLength(const Quintic& path) {
    double px = 0.0, py = 0.0;
    quinticAt(path, 0.0, px, py);
    double length = 0.0;
    for (int i = 1; i <= kLengthSteps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(kLengthSteps);
        double x = 0.0, y = 0.0;
        quinticAt(path, t, x, y);
        length += std::hypot(x - px, y - py);
        px = x;
        py = y;
    }
    return length;
}

inline Quintic quinticThrough(double rx, double ry, double tx, double ty, double approach) {
    const double dx = rx - tx;
    const double dy = ry - ty;
    const double ux = std::cos(approach);
    const double uy = std::sin(approach);
    const double nx = -uy;
    const double ny = ux;
    const double xi0 = dx * ux + dy * uy;
    const double eta0 = dx * nx + dy * ny;
    const double dist = std::hypot(dx, dy);

    Quintic path;
    path.phi = approach;
    if (dist < 1e-4) return path;

    // Beside the join, or already on the heading line: no curve to fit.
    if (std::fabs(xi0) < kOnAxisM || std::fabs(eta0) < kOnAxisM) {
        path.length_m = dist;
        if (xi0 > 0.0) path.phi = std::atan2(ty - ry, tx - rx);
        return path;
    }

    // μ = |η| / |ξ|. k is the slope factor dη/dξ = k η/ξ. α = k/5 is the
    // fraction of the offset the first handle drops; β is the fraction of
    // what remains that the second handle keeps. The last three handles
    // sit on the axis.
    const double mu = std::fabs(eta0) / std::fabs(xi0);
    const double slope_k = 2.5 + 1.5 / (1.0 + mu);
    const double alpha = slope_k / 5.0;
    const double beta = 0.25 + 0.35 * mu / (1.0 + mu);
    const double cxi[6] = {
        xi0,
        xi0 * (4.0 / 5.0),
        xi0 * (3.0 / 5.0),
        xi0 * (2.0 / 5.0),
        xi0 * (1.0 / 5.0),
        0.0
    };
    const double ceta[6] = {
        eta0,
        eta0 * (1.0 - alpha),
        eta0 * (1.0 - alpha) * beta,
        0.0,
        0.0,
        0.0
    };
    for (int i = 0; i < 6; ++i) {
        path.px[i] = tx + cxi[i] * ux + ceta[i] * nx;
        path.py[i] = ty + cxi[i] * uy + ceta[i] * ny;
    }

    // B'(0) = 5 (P1 − P0), B''(0) = 20 (P2 − 2 P1 + P0), heading frame.
    const double dxi = 5.0 * (cxi[1] - cxi[0]);
    const double deta = 5.0 * (ceta[1] - ceta[0]);
    const double ddxi = 20.0 * (cxi[2] - 2.0 * cxi[1] + cxi[0]);
    const double ddeta = 20.0 * (ceta[2] - 2.0 * ceta[1] + ceta[0]);
    const double speed2 = dxi * dxi + deta * deta;
    const double dir_x = dxi * ux + deta * nx;
    const double dir_y = dxi * uy + deta * ny;
    path.phi = std::atan2(dir_y, dir_x);
    if (speed2 > 1e-12) {
        const double speed = std::sqrt(speed2);
        path.kappa = (dxi * ddeta - deta * ddxi) / (speed2 * speed);
    }
    path.curved = true;
    path.length_m = quinticLength(path);
    return path;
}

// Wheel speed along the nose, as a field direction and a signed curvature.
// Below kMomentumSpeed the travel heading stands in and curvature is zero:
// a near-stop has no direction to absorb.
struct Momentum {
    double x = 1;
    double y = 0;
    double kappa = 0;
};

inline Momentum momentumOf(double theta, bool forwards, double v_lin, double v_ang) {
    Momentum m;
    const double speed = std::fabs(v_lin);
    if (speed < kMomentumSpeed) {
        const double face = forwards ? theta : (theta + Robot::PI);
        m.x = std::cos(face);
        m.y = std::sin(face);
        return m;
    }
    const double sense = (v_lin > 0.0) ? 1.0 : -1.0;
    m.x = sense * std::cos(theta);
    m.y = sense * std::sin(theta);
    m.kappa = v_ang / speed;
    return m;
}

// Quartic contact at the join, measured slope and curvature at the robot.
// See DriftCurve.h. A momentum aimed across the axis, or back out along it,
// is pulled to 80° off the axis so the result stays a graph.
inline GraphPath quarticHermiteThrough(double rx, double ry, double tx, double ty,
                                double approach, double vx, double vy,
                                double kappa0) {
    const double dx = rx - tx;
    const double dy = ry - ty;
    const double ux = std::cos(approach);
    const double uy = std::sin(approach);
    const double nx = -uy;
    const double ny = ux;
    const double xi0 = dx * ux + dy * uy;
    const double eta0 = dx * nx + dy * ny;
    const double dist = std::hypot(dx, dy);

    GraphPath path;
    path.phi = approach;
    path.momentum = true;
    path.ux = ux;
    path.uy = uy;
    path.nx = nx;
    path.ny = ny;
    path.tx = tx;
    path.ty = ty;
    path.xi0 = xi0;
    path.eta0 = eta0;
    if (dist < 1e-4) return path;

    if (std::fabs(xi0) < kOnAxisM || std::fabs(eta0) < kOnAxisM) {
        path.length_m = dist;
        if (xi0 > 0.0) path.phi = std::atan2(ty - ry, tx - rx);
        return path;
    }

    const double t_xi = vx * ux + vy * uy;
    const double t_eta = vx * nx + vy * ny;
    const double toward = (xi0 > 0.0) ? -1.0 : 1.0;
    const double along = t_xi * toward;
    double m;
    if (along < kGraphAlongMin) {
        const double side = (std::fabs(t_eta) > 1e-6) ? t_eta : eta0;
        m = std::copysign(kGraphSlopeMax, side * toward);
    } else {
        m = t_eta / t_xi;
    }

    const double kappa_cap = 10.0 / dist;
    const double kappa = mathutil::clamp_val(kappa0, -kappa_cap, kappa_cap);
    const double speed = std::sqrt(1.0 + m * m);
    const double eta_pp = kappa * speed * speed * speed * toward;
    const double s = m * xi0 / eta0;
    const double q = eta_pp * (xi0 * xi0) / eta0;
    const double a = 0.5 * q - 4.0 * s + 10.0;
    const double b = 9.0 * s - q - 24.0;
    path.c6 = a;
    path.c5 = b;
    path.c4 = 1.0 - a - b;
    path.graph = true;
    graphFrame(path, xi0, path.phi, path.kappa);
    path.length_m = graphLength(path, rx, ry);
    return path;
}

// Quintic Hermite spline in the field frame. Tangent scale is the chord L,
// so B'(0) = L T0 and B'(1) = L T1. See DriftCurve.h for the handles.
inline Quintic hermiteThrough(double rx, double ry, double tx, double ty, double approach,
                       double vx, double vy, double kappa0) {
    const double dx = tx - rx;
    const double dy = ty - ry;
    const double dist = std::hypot(dx, dy);

    Quintic path;
    path.phi = approach;
    if (dist < 1e-4) return path;

    double dir = std::hypot(vx, vy);
    double tx0 = 0.0;
    double ty0 = 0.0;
    if (dir < 1e-9) {
        tx0 = std::cos(approach);
        ty0 = std::sin(approach);
        kappa0 = 0.0;
    } else {
        tx0 = vx / dir;
        ty0 = vy / dir;
    }
    const double ux = std::cos(approach);
    const double uy = std::sin(approach);
    const double nx = -ty0;
    const double ny = tx0;

    // P2's normal offset is L² κ / 20. |κ| <= 10/L holds it to L/2.
    const double sigma = dist;
    const double kappa_cap = 10.0 / dist;
    const double kappa = mathutil::clamp_val(kappa0, -kappa_cap, kappa_cap);

    const double v0x = sigma * tx0;
    const double v0y = sigma * ty0;
    const double a0x = sigma * sigma * kappa * nx;
    const double a0y = sigma * sigma * kappa * ny;
    const double v1x = sigma * ux;
    const double v1y = sigma * uy;

    path.px[0] = rx;
    path.py[0] = ry;
    path.px[1] = rx + v0x / 5.0;
    path.py[1] = ry + v0y / 5.0;
    path.px[2] = rx + 2.0 * v0x / 5.0 + a0x / 20.0;
    path.py[2] = ry + 2.0 * v0y / 5.0 + a0y / 20.0;
    path.px[5] = tx;
    path.py[5] = ty;
    path.px[4] = tx - v1x / 5.0;
    path.py[4] = ty - v1y / 5.0;
    path.px[3] = tx - 2.0 * v1x / 5.0;
    path.py[3] = ty - 2.0 * v1y / 5.0;

    // B'(0) = 5 (P1 − P0), B''(0) = 20 (P2 − 2 P1 + P0).
    const double dpx = 5.0 * (path.px[1] - path.px[0]);
    const double dpy = 5.0 * (path.py[1] - path.py[0]);
    const double ddpx = 20.0 * (path.px[2] - 2.0 * path.px[1] + path.px[0]);
    const double ddpy = 20.0 * (path.py[2] - 2.0 * path.py[1] + path.py[0]);
    const double speed2 = dpx * dpx + dpy * dpy;
    path.phi = std::atan2(dpy, dpx);
    if (speed2 > 1e-12) {
        const double speed = std::sqrt(speed2);
        path.kappa = (dpx * ddpy - dpy * ddpx) / (speed2 * speed);
    }
    path.curved = true;
    path.length_m = quinticLength(path);
    return path;
}

} // namespace driftpath
