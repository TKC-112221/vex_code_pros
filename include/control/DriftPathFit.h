#pragma once
/* ---------------------------------------------------------------------------
 * DriftPathFit  --  the DriftCurve shapes and the straight finish.
 *
 * Shared by DriftCurve, which refits every tick, and DriftFollower, which
 * samples a path once. The geometry matches the notes in DriftCurve.h.
 * feasibleSpeedAt is the timed profile those shapes are driven with.
 * DriftFollower lays a timed elastic band on that polyline once, when
 * the plan is committed. The band carries its own time steps, and the
 * follower tracks the speed stored there. Its heading-jerk loss is the
 * third derivative of the drifted chassis heading.
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
    const double ux = mathutil::heading_cos(approach);
    const double uy = mathutil::heading_sin(approach);
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
    const double ux = mathutil::heading_cos(approach);
    const double uy = mathutil::heading_sin(approach);
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
        if (along > 0.0) arc.phi = mathutil::direction_heading(tx - rx, ty - ry);
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
    arc.phi = mathutil::direction_heading(dir_x, dir_y);
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
    phi = mathutil::direction_heading(dir_u * path.ux + dir_n * path.nx,
                                      dir_u * path.uy + dir_n * path.ny);
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

// Tangent and curvature of any graph, including the exponential and the
// parabola. graphFrame only differentiates the polynomial family.
inline void graphTangent(const GraphPath& path, double xi, double& phi, double& kappa) {
    if (path.momentum || path.quartic) {
        graphFrame(path, xi, phi, kappa);
        return;
    }
    double m = 0.0;
    double eta_pp = 0.0;
    if (std::fabs(path.rate) < 1e-6) {
        if (std::fabs(path.xi0) > 1e-9) {
            m = 2.0 * path.eta0 * xi / (path.xi0 * path.xi0);
            eta_pp = 2.0 * path.eta0 / (path.xi0 * path.xi0);
        }
    } else {
        double f0 = 0.0, fp0 = 0.0, fpp0 = 0.0, f = 0.0, fp = 0.0, fpp = 0.0;
        expBasis(path.rate * path.xi0, f0, fp0, fpp0);
        expBasis(path.rate * xi, f, fp, fpp);
        if (std::fabs(f0) > 1e-9) {
            m = path.eta0 * (path.rate * fp) / f0;
            eta_pp = path.eta0 * (path.rate * path.rate * fpp) / f0;
        }
    }
    const double speed = std::sqrt(1.0 + m * m);
    const double toward = (path.xi0 > 0.0) ? -1.0 : 1.0;
    phi = mathutil::direction_heading(toward * path.ux + toward * m * path.nx,
                                      toward * path.uy + toward * m * path.ny);
    const double sp3 = speed * speed * speed;
    kappa = (sp3 > 1e-9) ? toward * eta_pp / sp3 : 0.0;
}

inline GraphPath graphThrough(double rx, double ry, double tx, double ty,
                       double approach, bool quartic, double rate) {
    const double dx = rx - tx;
    const double dy = ry - ty;
    const double ux = mathutil::heading_cos(approach);
    const double uy = mathutil::heading_sin(approach);
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
        if (xi0 > 0.0) path.phi = mathutil::direction_heading(tx - rx, ty - ry);
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
    path.phi = mathutil::direction_heading(dir_u * ux + dir_n * nx,
                                          dir_u * uy + dir_n * ny);
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
    phi = mathutil::direction_heading(dx, dy);
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
    const double ux = mathutil::heading_cos(approach);
    const double uy = mathutil::heading_sin(approach);
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
        if (xi0 > 0.0) path.phi = mathutil::direction_heading(tx - rx, ty - ry);
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
    path.phi = mathutil::direction_heading(dir_x, dir_y);
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
    double x = 0;
    double y = 1;
    double kappa = 0;
};

// Field direction of travel. A stop has none, so the nose stands in.
// Backing up is the opposite of the heading: v_lin is signed along the nose.
inline double travelHeading(double theta, double v_lin) {
    if (std::fabs(v_lin) < kMomentumSpeed) return theta;
    if (v_lin >= 0.0) return theta;
    return mathutil::normalize_angle(theta + Robot::PI);
}

inline Momentum momentumOf(double theta, bool forwards, double v_lin, double v_ang) {
    Momentum m;
    const double speed = std::fabs(v_lin);
    if (speed < kMomentumSpeed) {
        const double face = forwards ? theta : (theta + Robot::PI);
        m.x = mathutil::heading_cos(face);
        m.y = mathutil::heading_sin(face);
        return m;
    }
    const double sense = (v_lin > 0.0) ? 1.0 : -1.0;
    m.x = sense * mathutil::heading_cos(theta);
    m.y = sense * mathutil::heading_sin(theta);
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
    const double ux = mathutil::heading_cos(approach);
    const double uy = mathutil::heading_sin(approach);
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
        if (xi0 > 0.0) path.phi = mathutil::direction_heading(tx - rx, ty - ry);
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
        tx0 = mathutil::heading_cos(approach);
        ty0 = mathutil::heading_sin(approach);
        kappa0 = 0.0;
    } else {
        tx0 = vx / dir;
        ty0 = vy / dir;
    }
    const double ux = mathutil::heading_cos(approach);
    const double uy = mathutil::heading_sin(approach);
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
    path.phi = mathutil::direction_heading(dpx, dpy);
    if (speed2 > 1e-12) {
        const double speed = std::sqrt(speed2);
        path.kappa = (dpx * ddpy - dpy * ddpx) / (speed2 * speed);
    }
    path.curved = true;
    path.length_m = quinticLength(path);
    return path;
}

// One sample of a timed profile. s is arc length from the start of the
// polyline, kappa is signed CCW, dk is dκ/ds.
struct SpeedKnot {
    double s = 0;
    double kappa = 0;
    double dk = 0;
};

// speed_reward multiplies the soft caps. The yaw-rate limit stays put:
// that one is the drivetrain, not a tracking preference.
struct SpeedScale {
    double reward = 1;
    double accel = 0;
    double centripetal = 0;
    double yaw_accel = 0;
};

inline SpeedScale scaleSpeed(double reward, double accel, double centripetal) {
    SpeedScale s;
    s.reward = reward;
    if (s.reward < 0.25) s.reward = 0.25;
    if (s.reward > 8.0) s.reward = 8.0;
    s.accel = accel * s.reward;
    if (s.accel > Physics::MAX_ACCEL_LIN) s.accel = Physics::MAX_ACCEL_LIN;
    if (s.accel < Motion::MIN_ACCEL_MPS2) s.accel = Motion::MIN_ACCEL_MPS2;
    s.centripetal = centripetal * s.reward;
    if (s.centripetal < 0.0) s.centripetal = 0.0;
    s.yaw_accel = Physics::MAX_ACCEL_ANG * s.reward;
    return s;
}

// Fastest speed the chassis can hold at this curvature. Cruise, the
// centripetal budget, the yaw-rate limit, and the yaw-accel limit.
inline double speedLimitAt(double kappa, double dk, double v_cruise,
                           double a_cent, double w_max, double alpha_max) {
    double v = (v_cruise > 0.0) ? v_cruise : 0.0;
    const double k = std::fabs(kappa);
    if (k > Motion::KAPPA_EPS) {
        if (a_cent > 1e-6) {
            const double v_c = std::sqrt(a_cent / k);
            if (v_c < v) v = v_c;
        }
        if (w_max > 1e-6) {
            const double v_w = w_max / k;
            if (v_w < v) v = v_w;
        }
    }
    const double slope = std::fabs(dk);
    if (slope > 1e-4 && alpha_max > 1e-6) {
        const double v_a = std::sqrt(alpha_max / slope);
        if (v_a < v) v = v_a;
    }
    if (!std::isfinite(v) || v < 0.0) return 0.0;
    return v;
}

inline void fillSpeedSlope(SpeedKnot* knot, int n) {
    if (knot == nullptr || n < 2) return;
    for (int i = 0; i < n; ++i) {
        const int i0 = (i == 0) ? 0 : (i - 1);
        const int i1 = (i + 1 >= n) ? (n - 1) : (i + 1);
        const double ds = knot[i1].s - knot[i0].s;
        knot[i].dk = (ds > 1e-4) ? (knot[i1].kappa - knot[i0].kappa) / ds : 0.0;
    }
}

// Backward pass from the exit speed. The value at s_query is the fastest
// speed that can still reach v_end without breaking a limit on the way.
// v_out, when set, receives the ceiling at each knot. n is clamped to
// kSpeedKnots.
inline double feasibleSpeedAt(const SpeedKnot* knot, int n, double s_query,
                              double v_end, double v_cruise, double a_long,
                              double a_cent, double w_max, double alpha_max,
                              double* v_out = nullptr) {
    if (v_end < 0.0) v_end = 0.0;
    if (v_cruise < v_end) v_cruise = v_end;
    if (a_long < Motion::MIN_ACCEL_MPS2) a_long = Motion::MIN_ACCEL_MPS2;
    if (n > kSpeedKnots) n = kSpeedKnots;
    if (knot == nullptr || n < 1) return (v_end < v_cruise) ? v_end : v_cruise;
    if (n == 1) {
        double v = speedLimitAt(knot[0].kappa, knot[0].dk, v_cruise, a_cent, w_max, alpha_max);
        if (v_end < v) v = v_end;
        if (v_out != nullptr) v_out[0] = v;
        return v;
    }

    double v[kSpeedKnots];
    v[n - 1] = speedLimitAt(knot[n - 1].kappa, knot[n - 1].dk, v_cruise,
                            a_cent, w_max, alpha_max);
    if (v_end < v[n - 1]) v[n - 1] = v_end;
    for (int i = n - 2; i >= 0; --i) {
        double ds = knot[i + 1].s - knot[i].s;
        if (ds < 1e-6) ds = 1e-6;
        const double v_brake = std::sqrt(v[i + 1] * v[i + 1] + 2.0 * a_long * ds);
        double cap = speedLimitAt(knot[i].kappa, knot[i].dk, v_cruise, a_cent, w_max, alpha_max);
        if (v_brake < cap) cap = v_brake;
        v[i] = cap;
    }
    if (v_out != nullptr) {
        for (int i = 0; i < n; ++i) v_out[i] = v[i];
    }
    if (s_query <= knot[0].s) return v[0];
    if (s_query >= knot[n - 1].s) return v[n - 1];
    for (int i = 0; i < n - 1; ++i) {
        if (s_query > knot[i + 1].s) continue;
        const double ds = knot[i + 1].s - knot[i].s;
        const double t = (ds > 1e-9) ? (s_query - knot[i].s) / ds : 0.0;
        return v[i] + t * (v[i + 1] - v[i]);
    }
    return v[n - 1];
}

// Forward pass from the speed the chassis has now, under the backward ceiling.
inline void runningSpeed(const SpeedKnot* knot, const double* v_back, int n,
                         double v_start, double a_long, double* v_run) {
    if (knot == nullptr || v_back == nullptr || v_run == nullptr || n < 1) return;
    if (v_start < 0.0) v_start = 0.0;
    if (v_start > v_back[0]) v_start = v_back[0];
    if (a_long < Motion::MIN_ACCEL_MPS2) a_long = Motion::MIN_ACCEL_MPS2;
    v_run[0] = v_start;
    for (int i = 1; i < n; ++i) {
        double ds = knot[i].s - knot[i - 1].s;
        if (ds < 1e-6) ds = 1e-6;
        double v_acc = std::sqrt(v_run[i - 1] * v_run[i - 1] + 2.0 * a_long * ds);
        if (v_acc > v_back[i]) v_acc = v_back[i];
        v_run[i] = v_acc;
    }
}

inline int appendSpan(SpeedKnot* knot, int n, int cap, double length) {
    if (knot == nullptr || n < 1 || n >= cap || length < 1e-4) return n;
    knot[n].s = knot[n - 1].s + length;
    knot[n].kappa = 0.0;
    knot[n].dk = 0.0;
    return n + 1;
}

inline int graphKnots(const GraphPath& path, SpeedKnot* out, int cap) {
    if (out == nullptr || cap < 2) return 0;
    if (!path.graph) {
        out[0] = {0.0, 0.0, 0.0};
        out[1] = {path.length_m, 0.0, 0.0};
        return 2;
    }
    int steps = cap - 1;
    if (steps > 16) steps = 16;
    double px = path.tx + path.xi0 * path.ux + path.eta0 * path.nx;
    double py = path.ty + path.xi0 * path.uy + path.eta0 * path.ny;
    double s = 0.0;
    for (int i = 0; i <= steps; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(steps);
        const double xi = path.xi0 * (1.0 - u);
        const double eta = graphEta(path, xi);
        const double x = path.tx + xi * path.ux + eta * path.nx;
        const double y = path.ty + xi * path.uy + eta * path.ny;
        if (i > 0) s += std::hypot(x - px, y - py);
        double phi = 0.0, kappa = 0.0;
        graphTangent(path, xi, phi, kappa);
        out[i].s = s;
        out[i].kappa = kappa;
        out[i].dk = 0.0;
        px = x;
        py = y;
    }
    return steps + 1;
}

inline int quinticKnots(const Quintic& path, SpeedKnot* out, int cap) {
    if (out == nullptr || cap < 2) return 0;
    if (!path.curved) {
        out[0] = {0.0, 0.0, 0.0};
        out[1] = {path.length_m, 0.0, 0.0};
        return 2;
    }
    int steps = cap - 1;
    if (steps > 16) steps = 16;
    double px = 0.0, py = 0.0;
    quinticAt(path, 0.0, px, py);
    double s = 0.0;
    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        double x = 0.0, y = 0.0, phi = 0.0, kappa = 0.0;
        quinticAt(path, t, x, y);
        quinticFrame(path, t, phi, kappa);
        if (i > 0) s += std::hypot(x - px, y - py);
        out[i].s = s;
        out[i].kappa = kappa;
        out[i].dk = 0.0;
        px = x;
        py = y;
    }
    return steps + 1;
}

inline int arcKnots(const Arc& arc, SpeedKnot* out, int cap) {
    if (out == nullptr || cap < 2) return 0;
    if (!arc.curved) {
        out[0] = {0.0, 0.0, 0.0};
        out[1] = {arc.length_m, 0.0, 0.0};
        return 2;
    }
    int steps = cap - 1;
    if (steps > 8) steps = 8;
    for (int i = 0; i <= steps; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(steps);
        out[i].s = arc.length_m * u;
        out[i].kappa = arc.kappa;
        out[i].dk = 0.0;
    }
    return steps + 1;
}

// One circular keep-out. min_dist is the radius the pose centers stay
// outside of, meters. A field wall is the same shape with a large radius,
// or TebLimits::field_bound for the rectangular perimeter.
struct TebObstacle {
    double x = 0;
    double y = 0;
    double min_dist = 0;
};

// Timed elastic band. Poses are meters in the field frame. theta[i] is the
// nose heading. dt[i] is the time step from pose i to pose i+1, seconds,
// and is unused at the last pose. v[i] is the speed that time step implies,
// m/s along travel, and zero where the drive reverses. v_plan is that speed
// at the robot. A straight finish is not a pose; its far end is the exit
// speed. DriftFollower tracks v[i].
struct TimedBand {
    double phi = 0;
    double kappa = 0;
    double length_m = 0;
    double v_plan = 0;
    bool curved = false;
    int count = 0;
    double x[kTebPoses] {};
    double y[kTebPoses] {};
    double theta[kTebPoses] {};
    double dt[kTebPoses] {};
    double s[kTebPoses] {};
    double phi_s[kTebPoses] {};
    double kappa_s[kTebPoses] {};
    double v[kTebPoses] {};
    // +1 the nose leads the segment. −1 the segment is driven in reverse
    // and the nose faces the opposite way, the same choice ROS TEB makes
    // per timed edge.
    double drive[kTebPoses] {};
};

struct TebLimits {
    double v_start = 0;
    double v_end = 0;
    double v_cruise = 0;
    double a_long = 0;
    double a_cent = 0;
    double w_max = 0;
    double alpha_max = 0;
    double finish_m = 0;
    double nose = 0;          // chassis heading, for the yaw cost only
    double motion = 0;        // direction of travel; the nose when nearly stopped
    double finish_drive = 1;  // +1 arrive nose-forward along the exit, −1 backing in
    double drive_hint = 0;    // last tick's opening drive, 0 if none
    bool field_bound = false; // keep free poses inside the 12 ft field
    // Drift map for the heading-jerk loss. slip_accel is a_max under arcsin
    // and drift_accel under atan. Zero keeps the loss on the geometric heading.
    bool slip_arcsin = true;
    double slip_accel = 0;
    double slip_limit = 0;
    int obstacle_n = 0;
    TebObstacle obstacles[kTebObstacles] {};
};

inline bool addObstacle(TebLimits& lim, double x, double y, double min_dist) {
    if (lim.obstacle_n < 0) lim.obstacle_n = 0;
    if (lim.obstacle_n >= kTebObstacles) return false;
    if (!(min_dist > 0.0) || !std::isfinite(x) || !std::isfinite(y)) return false;
    TebObstacle& obs = lim.obstacles[lim.obstacle_n];
    obs.x = x;
    obs.y = y;
    obs.min_dist = min_dist;
    ++lim.obstacle_n;
    return true;
}

inline TebLimits tebLimits(double v_start, double v_end, double v_cruise,
                           double a_long, double a_cent, double finish_m,
                           double nose = 0.0, double finish_drive = 1.0,
                           double drive_hint = 0.0) {
    TebLimits lim;
    lim.v_start = v_start;
    lim.v_end = v_end;
    lim.v_cruise = v_cruise;
    lim.a_long = a_long;
    lim.a_cent = a_cent;
    lim.w_max = Robot::MAX_VEL_ANG;
    lim.alpha_max = Physics::MAX_ACCEL_ANG;
    lim.finish_m = finish_m;
    lim.nose = nose;
    lim.motion = nose;
    lim.finish_drive = (finish_drive < 0.0) ? -1.0 : 1.0;
    lim.drive_hint = drive_hint;
    return lim;
}

inline void stampDrive(TimedBand& band, double drive) {
    const double d = (drive < 0.0) ? -1.0 : 1.0;
    const int n = (band.count > kTebPoses) ? kTebPoses : band.count;
    for (int i = 0; i < n; ++i) band.drive[i] = d;
    for (int i = n; i < kTebPoses; ++i) band.drive[i] = d;
}

inline double driveAt(const TimedBand& band, double distance) {
    if (band.count < 1) return 1.0;
    double d = (band.drive[0] < 0.0) ? -1.0 : 1.0;
    if (distance <= 1e-4) return d;
    for (int i = 0; i < band.count - 1; ++i) {
        d = (band.drive[i] < 0.0) ? -1.0 : 1.0;
        if (band.s[i + 1] + 1e-9 >= distance) return d;
    }
    return d;
}

// Travel time from the timed ceiling, plus the yaw from the current nose
// onto the opening segment. A cusp floors the ceiling so the reversal has
// a finite cost instead of a divide by zero.
inline double bandTravelTime(const TimedBand& band, double v_start, double nose) {
    if (band.count < 2) return 1e6;
    double time = 0.0;
    double v = v_start;
    if (v < 0.05) v = 0.05;
    for (int i = 0; i < band.count - 1; ++i) {
        const double ds = band.s[i + 1] - band.s[i];
        if (ds < 1e-4) continue;
        double cap = band.v[i];
        if (cap < 0.12) cap = 0.12;
        double v_avg = 0.5 * (v + cap);
        if (v_avg < 0.12) v_avg = 0.12;
        time += ds / v_avg;
        v = cap;
    }
    const double want = band.phi_s[0] + ((band.drive[0] < 0.0) ? Robot::PI : 0.0);
    const double yaw = std::fabs(mathutil::normalize_angle(want - nose));
    // Spin in place: accelerate, then brake, and respect the yaw-rate cap.
    // Dividing by the rate limit alone makes a half turn look almost free
    // next to a reverse leg, so the band never backs up.
    if (yaw > 1e-3 && Physics::MAX_ACCEL_ANG > 1e-3) {
        const double w_cap = Robot::MAX_VEL_ANG;
        const double w_peak = std::sqrt(Physics::MAX_ACCEL_ANG * yaw);
        if (w_peak <= w_cap) time += 2.0 * w_peak / Physics::MAX_ACCEL_ANG;
        else if (w_cap > 1e-3) time += yaw / w_cap + w_cap / Physics::MAX_ACCEL_ANG;
    }
    return time;
}

inline void bandStraight(TimedBand& band, double rx, double ry, double tx, double ty,
                         double phi) {
    const double dist = std::hypot(tx - rx, ty - ry);
    band.count = 2;
    band.curved = false;
    band.x[0] = rx;
    band.y[0] = ry;
    band.x[1] = tx;
    band.y[1] = ty;
    band.s[0] = 0.0;
    band.s[1] = dist;
    band.phi_s[0] = phi;
    band.phi_s[1] = phi;
    band.kappa_s[0] = 0.0;
    band.kappa_s[1] = 0.0;
    band.v[0] = 0.0;
    band.v[1] = 0.0;
    band.phi = phi;
    band.kappa = 0.0;
    band.length_m = dist;
    band.v_plan = 0.0;
}

inline void measureBand(TimedBand& band) {
    const int n = band.count;
    if (n < 2) return;
    double seg[kTebPoses];
    seg[0] = mathutil::direction_heading(band.x[1] - band.x[0], band.y[1] - band.y[0]);
    for (int i = 1; i < n - 1; ++i) {
        const double raw = mathutil::direction_heading(band.x[i + 1] - band.x[i],
                                                       band.y[i + 1] - band.y[i]);
        seg[i] = seg[i - 1] + mathutil::normalize_angle(raw - seg[i - 1]);
    }
    band.s[0] = 0.0;
    for (int i = 1; i < n; ++i) {
        band.s[i] = band.s[i - 1]
                  + std::hypot(band.x[i] - band.x[i - 1], band.y[i] - band.y[i - 1]);
    }
    for (int i = 0; i < n - 1; ++i) band.phi_s[i] = seg[i];
    band.phi_s[n - 1] = seg[n - 2];
    for (int i = 1; i < n - 1; ++i) {
        const double dphi = seg[i] - seg[i - 1];
        const double ds = 0.5 * ((band.s[i] - band.s[i - 1]) + (band.s[i + 1] - band.s[i]));
        band.kappa_s[i] = (ds > 1e-4) ? dphi / ds : 0.0;
    }
    band.kappa_s[n - 1] = 0.0;
    band.kappa_s[0] = band.kappa_s[1];
    band.phi = band.phi_s[0];
    band.kappa = band.kappa_s[0];
    band.length_m = band.s[n - 1];
}

// v_run receives the speed the chassis will actually be carrying, starting
// from lim.v_start and staying under the backward ceiling. The finish, when
// set, is an extra straight knot so the curve can stay fast into it.
inline void timeBandInto(TimedBand& band, const TebLimits& lim, double* v_run) {
    SpeedKnot knot[kTebPoses + 1];
    int m = band.count;
    if (m > kTebPoses) m = kTebPoses;
    if (m < 1) return;
    for (int i = 0; i < m; ++i) {
        knot[i].s = band.s[i];
        knot[i].kappa = band.kappa_s[i];
        knot[i].dk = 0.0;
    }
    if (lim.finish_m > 1e-4 && m < kTebPoses + 1) {
        knot[m].s = knot[m - 1].s + lim.finish_m;
        knot[m].kappa = 0.0;
        knot[m].dk = 0.0;
        ++m;
    }
    fillSpeedSlope(knot, m);
    double v_back[kSpeedKnots];
    feasibleSpeedAt(knot, m, band.s[0], lim.v_end, lim.v_cruise, lim.a_long,
                    lim.a_cent, lim.w_max, lim.alpha_max, v_back);
    double run[kSpeedKnots];
    runningSpeed(knot, v_back, m, lim.v_start, lim.a_long, run);
    const int n = band.count;
    for (int i = 0; i < n && i < kTebPoses; ++i) {
        band.v[i] = v_back[i];
        if (v_run != nullptr) v_run[i] = run[i];
    }
    band.v_plan = v_back[0];
}

inline bool bandStepOk(const TimedBand& band, int i, double px, double py) {
    const int n = band.count;
    if (i <= 0 || i >= n - 1) return false;
    if (!std::isfinite(px) || !std::isfinite(py)) return false;
    const double back = std::hypot(px - band.x[i - 1], py - band.y[i - 1]);
    const double fore = std::hypot(band.x[i + 1] - px, band.y[i + 1] - py);
    if (back < kTebMinSegM || fore < kTebMinSegM) return false;
    const double inx = px - band.x[i - 1];
    const double iny = py - band.y[i - 1];
    const double outx = band.x[i + 1] - px;
    const double outy = band.y[i + 1] - py;
    const double turn = std::atan2(inx * outy - iny * outx, inx * outx + iny * outy);
    if (std::fabs(turn) > 2.2) return false;
    const double cx = band.x[n - 1] - band.x[0];
    const double cy = band.y[n - 1] - band.y[0];
    const double cl = std::hypot(cx, cy);
    if (cl < 1e-4) return false;
    const double ux = cx / cl;
    const double uy = cy / cl;
    const double a_prev = (band.x[i - 1] - band.x[0]) * ux + (band.y[i - 1] - band.y[0]) * uy;
    const double a_next = (band.x[i + 1] - band.x[0]) * ux + (band.y[i + 1] - band.y[0]) * uy;
    const double a_new = (px - band.x[0]) * ux + (py - band.y[0]) * uy;
    if (a_new < a_prev + 0.5 * kTebMinSegM) return false;
    if (a_new > a_next - 0.5 * kTebMinSegM) return false;
    return true;
}

inline double curvatureBudget(double v_use, double a_cent, double w_max) {
    if (v_use < 0.30) v_use = 0.30;
    double k_allow = kTebKappaHard;
    if (a_cent > 1e-6) {
        const double k_c = a_cent / (v_use * v_use);
        if (k_c < k_allow) k_allow = k_c;
    }
    if (w_max > 1e-6) {
        const double k_w = w_max / v_use;
        if (k_w < k_allow) k_allow = k_w;
    }
    if (k_allow < 1e-3) k_allow = 1e-3;
    return k_allow;
}

// One sweep of the free poses. A pose steps toward the chord of its
// neighbors only while its curvature is tighter than v_ref can hold.
// v_ref ignores curvature on purpose: a kink is opened, not driven slowly.
// A bend already inside the budget is left alone. Pulling every pose to
// the midpoint would straighten the chain and dump the curvature on the
// clamped ends. The fourth-order term matches each second difference to
// its neighbors so the curvature that remains changes smoothly.
inline void bendBand(TimedBand& band, const double* v_ref, double a_cent, double w_max) {
    const int n = band.count;
    if (n < 5 || v_ref == nullptr) return;
    double sx[kTebPoses];
    double sy[kTebPoses];
    for (int i = 0; i < n; ++i) {
        sx[i] = band.x[i];
        sy[i] = band.y[i];
    }
    auto secondX = [&](int k) {
        return sx[k - 1] - 2.0 * sx[k] + sx[k + 1];
    };
    auto secondY = [&](int k) {
        return sy[k - 1] - 2.0 * sy[k] + sy[k + 1];
    };
    for (int i = 2; i <= n - 3; ++i) {
        if ((band.drive[i - 1] < 0.0) != (band.drive[i] < 0.0)) continue;
        if ((band.drive[i] < 0.0) != (band.drive[i + 1] < 0.0)) continue;
        const double k_allow = curvatureBudget(v_ref[i], a_cent, w_max);
        const double k_prev = curvatureBudget(v_ref[i - 1], a_cent, w_max);
        const double worst = std::fmax(std::fabs(band.kappa_s[i]),
                                        std::fabs(band.kappa_s[i - 1]));
        const double allow = (k_prev < k_allow) ? k_prev : k_allow;
        const double ratio = (allow > 1e-6) ? worst / allow : 0.0;
        double px = sx[i];
        double py = sy[i];
        if (ratio > 1.05) {
            double frac = 0.20 + 0.10 * (ratio - 1.0);
            if (frac > 0.40) frac = 0.40;
            const double mx = 0.5 * (sx[i - 1] + sx[i + 1]);
            const double my = 0.5 * (sy[i - 1] + sy[i + 1]);
            px += frac * (mx - sx[i]);
            py += frac * (my - sy[i]);
        }
        const double mismatch_x = 0.5 * (secondX(i - 1) + secondX(i + 1)) - secondX(i);
        const double mismatch_y = 0.5 * (secondY(i - 1) + secondY(i + 1)) - secondY(i);
        px += -0.5 * kTebBiharmonic * mismatch_x;
        py += -0.5 * kTebBiharmonic * mismatch_y;
        if (!bandStepOk(band, i, px, py)) continue;
        band.x[i] = px;
        band.y[i] = py;
    }
}

// Speed along the chain with curvature ignored: cruise, longitudinal accel,
// and the exit speed. The elastic sweep aims at this, then the published
// ceiling puts the centripetal and yaw limits back on.
inline void referenceSpeed(const TimedBand& band, const TebLimits& lim, double* v_ref) {
    if (v_ref == nullptr || band.count < 1) return;
    SpeedKnot knot[kTebPoses + 1];
    int m = band.count;
    if (m > kTebPoses) m = kTebPoses;
    for (int i = 0; i < m; ++i) {
        knot[i].s = band.s[i];
        knot[i].kappa = 0.0;
        knot[i].dk = 0.0;
    }
    if (lim.finish_m > 1e-4 && m < kTebPoses + 1) {
        knot[m].s = knot[m - 1].s + lim.finish_m;
        knot[m].kappa = 0.0;
        knot[m].dk = 0.0;
        ++m;
    }
    double v_back[kSpeedKnots];
    feasibleSpeedAt(knot, m, band.s[0], lim.v_end, lim.v_cruise, lim.a_long,
                    0.0, 0.0, 0.0, v_back);
    double run[kSpeedKnots];
    runningSpeed(knot, v_back, m, lim.v_start, lim.a_long, run);
    for (int i = 0; i < band.count && i < kTebPoses; ++i) v_ref[i] = run[i];
}

// Put each free pose between its neighbors along the chord, keeping the
// lateral offset the seed had. Pins at 1 and n-2 are left where they are.
inline void orderBand(TimedBand& band) {
    const int n = band.count;
    if (n < 5) return;
    const double cx = band.x[n - 1] - band.x[0];
    const double cy = band.y[n - 1] - band.y[0];
    const double cl = std::hypot(cx, cy);
    if (cl < 1e-4) return;
    const double ux = cx / cl;
    const double uy = cy / cl;
    const double lx = -uy;
    const double ly = ux;
    for (int i = 2; i <= n - 3; ++i) {
        const double a_prev = (band.x[i - 1] - band.x[0]) * ux
                            + (band.y[i - 1] - band.y[0]) * uy;
        const double a_here = (band.x[i] - band.x[0]) * ux
                            + (band.y[i] - band.y[0]) * uy;
        const double a_next = (band.x[i + 1] - band.x[0]) * ux
                            + (band.y[i + 1] - band.y[0]) * uy;
        const double lo = a_prev + kTebMinSegM;
        const double hi = a_next - kTebMinSegM;
        if (hi <= lo) continue;
        if (a_here >= lo && a_here <= hi) continue;
        double a = a_here;
        if (a < lo) a = lo;
        if (a > hi) a = hi;
        const double lat = (band.x[i] - band.x[0]) * lx + (band.y[i] - band.y[0]) * ly;
        band.x[i] = band.x[0] + a * ux + lat * lx;
        band.y[i] = band.y[0] + a * uy + lat * ly;
    }
}

inline int bandKnots(const TimedBand& band, SpeedKnot* out, int cap) {
    if (out == nullptr || cap < 1 || band.count < 1) return 0;
    int n = band.count;
    if (n > cap) n = cap;
    for (int i = 0; i < n; ++i) {
        out[i].s = band.s[i];
        out[i].kappa = band.kappa_s[i];
        out[i].dk = 0.0;
    }
    return n;
}

inline void timedBandLook(const TimedBand& band, double distance, double& phi, double& kappa) {
    if (band.count < 2 || distance <= 1e-4) {
        phi = band.phi;
        kappa = band.kappa;
        return;
    }
    for (int i = 1; i < band.count; ++i) {
        if (band.s[i] + 1e-9 < distance && i != band.count - 1) continue;
        const double ds = band.s[i] - band.s[i - 1];
        double t = (ds > 1e-9) ? (distance - band.s[i - 1]) / ds : 1.0;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        const double dphi = mathutil::normalize_angle(band.phi_s[i] - band.phi_s[i - 1]);
        phi = band.phi_s[i - 1] + t * dphi;
        kappa = band.kappa_s[i - 1] + t * (band.kappa_s[i] - band.kappa_s[i - 1]);
        return;
    }
    phi = band.phi_s[band.count - 1];
    kappa = band.kappa_s[band.count - 1];
}

// Target speed along the band, m/s. The same blend timedBandLook uses for
// the tangent: at `distance` along the chain, between the speeds of the
// poses on either side.
inline double bandSpeedAt(const TimedBand& band, double distance) {
    if (band.count < 1) return 0.0;
    if (band.count == 1 || distance <= 1e-4) return band.v[0];
    for (int i = 1; i < band.count; ++i) {
        if (band.s[i] + 1e-9 < distance && i != band.count - 1) continue;
        const double ds = band.s[i] - band.s[i - 1];
        double t = (ds > 1e-9) ? (distance - band.s[i - 1]) / ds : 1.0;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        return band.v[i - 1] + t * (band.v[i] - band.v[i - 1]);
    }
    return band.v[band.count - 1];
}

// A reversal is a stop, not a turn. The chord heading flips there and
// measureBand would report a curvature the chassis never drives.
inline void clearCuspCurvature(TimedBand& band) {
    const int n = band.count;
    for (int i = 1; i < n - 1; ++i) {
        if ((band.drive[i - 1] < 0.0) != (band.drive[i] < 0.0)) band.kappa_s[i] = 0.0;
    }
    if (n > 1) band.kappa_s[0] = band.kappa_s[1];
    if (n > 0) band.kappa = band.kappa_s[0];
}

// Nose heading of the chord from pose i to i+1. Reverse travel faces the
// other way, so the mean of the two nose headings is the chord plus π.
inline double chordNose(const TimedBand& band, int i) {
    const double travel = mathutil::direction_heading(band.x[i + 1] - band.x[i],
                                                      band.y[i + 1] - band.y[i]);
    if (band.drive[i] < 0.0) return mathutil::normalize_angle(travel + Robot::PI);
    return travel;
}

// θ_i + θ_{i+1} = 2 * chordNose(i). The ends are fixed: the nose at the
// robot, and the nose on the last chord (opposite that chord when the
// finish backs in). Interior headings split the residual of the two
// segments they touch.
inline void syncHeadings(TimedBand& band, const TebLimits& lim) {
    const int n = band.count;
    if (n < 2) return;
    double chord[kTebPoses];
    chord[0] = chordNose(band, 0);
    for (int i = 1; i < n - 1; ++i) {
        const double raw = chordNose(band, i);
        chord[i] = chord[i - 1] + mathutil::normalize_angle(raw - chord[i - 1]);
    }
    band.theta[0] = lim.nose;
    const double exit_travel = mathutil::direction_heading(band.x[n - 1] - band.x[n - 2],
                                                          band.y[n - 1] - band.y[n - 2]);
    double exit_nose = exit_travel;
    if (lim.finish_drive < 0.0) exit_nose = mathutil::normalize_angle(exit_travel + Robot::PI);
    const double last = chord[n - 2];
    band.theta[n - 1] = last + mathutil::normalize_angle(exit_nose - last);
    for (int pass = 0; pass < 4; ++pass) {
        for (int i = 1; i < n - 1; ++i) {
            const double from_back = 2.0 * chord[i - 1] - band.theta[i - 1];
            const double from_fore = 2.0 * chord[i] - band.theta[i + 1];
            const double a = band.theta[i] + mathutil::normalize_angle(from_back - band.theta[i]);
            const double b = band.theta[i] + mathutil::normalize_angle(from_fore - band.theta[i]);
            band.theta[i] = 0.5 * (a + b);
        }
    }
    for (int i = 0; i < n; ++i) band.theta[i] = mathutil::normalize_angle(band.theta[i]);
}

// Smallest Δt that meets cruise, centripetal accel, yaw rate, and the
// brake toward the next pose. Time optimality wants Δt smaller than that.
// The two meet at the floor, so a pass steps toward it and stops there.
inline void projectDt(TimedBand& band, const TebLimits& lim) {
    const int n = band.count;
    if (n < 2) return;
    double cruise = lim.v_cruise;
    if (lim.v_end > cruise) cruise = lim.v_end;
    if (cruise < 0.05) cruise = 0.05;
    double a_long = lim.a_long;
    if (a_long < Motion::MIN_ACCEL_MPS2) a_long = Motion::MIN_ACCEL_MPS2;
    // The straight finish is not a pose. Braking to the exit speed starts
    // from the speed the join can still bleed off along that line.
    double v_arrive = (lim.v_end > 0.0) ? lim.v_end : 0.0;
    if (lim.finish_m > 1e-4) {
        const double v_join = std::sqrt(v_arrive * v_arrive + 2.0 * a_long * lim.finish_m);
        v_arrive = (v_join < cruise) ? v_join : cruise;
    }
    double v_next = v_arrive;
    for (int i = n - 2; i >= 0; --i) {
        double ds = band.s[i + 1] - band.s[i];
        if (ds < 1e-4) ds = 1e-4;
        const double adtheta = std::fabs(mathutil::normalize_angle(band.theta[i + 1] - band.theta[i]));
        double dt_min = 1e-3;
        const double dt_vel = ds / cruise;
        if (dt_vel > dt_min) dt_min = dt_vel;
        if (lim.a_cent > 1e-6 && adtheta > 1e-6) {
            const double dt_cent = std::sqrt((ds * adtheta) / lim.a_cent);
            if (dt_cent > dt_min) dt_min = dt_cent;
        }
        if (lim.w_max > 1e-6 && adtheta > 1e-6) {
            const double dt_w = adtheta / lim.w_max;
            if (dt_w > dt_min) dt_min = dt_w;
        }
        const double k = std::fabs(band.kappa_s[i]);
        if (k > 1e-4 && lim.a_cent > 1e-6) {
            const double dt_k = ds / std::sqrt(lim.a_cent / k);
            if (dt_k > dt_min) dt_min = dt_k;
        }
        if (k > 1e-4 && lim.w_max > 1e-6) {
            const double dt_kw = (ds * k) / lim.w_max;
            if (dt_kw > dt_min) dt_min = dt_kw;
        }
        const double v_brake = std::sqrt(v_next * v_next + 2.0 * a_long * ds);
        if (v_brake > 1e-3) {
            const double dt_b = ds / v_brake;
            if (dt_b > dt_min) dt_min = dt_b;
        }
        double dt = band.dt[i];
        if (!(dt > dt_min) || !std::isfinite(dt)) {
            dt = dt_min;
        } else {
            double cut = kTebTimeStep;
            const double gap = dt - dt_min;
            if (cut < 0.65 * gap) cut = 0.65 * gap;
            dt -= cut;
            if (dt < dt_min) dt = dt_min;
        }
        if (dt > 2.0) dt = 2.0;
        band.dt[i] = dt;
        const double v = ds / dt;
        band.v[i] = std::isfinite(v) ? v : 0.0;
        v_next = band.v[i];
    }
    band.v[n - 1] = v_arrive;
    band.v_plan = band.v[0];
}

// Headings and time steps from a speed profile that is already filled in.
// Does not change v[i]. The cusp and the straight finish build that
// profile with the backward pass; this only records what it implies.
inline void stampClock(TimedBand& band, const TebLimits& lim) {
    syncHeadings(band, lim);
    const int n = band.count;
    for (int i = 0; i < n - 1; ++i) {
        double ds = band.s[i + 1] - band.s[i];
        if (ds < 1e-4) ds = 1e-4;
        double v = band.v[i];
        if (v < 0.05) v = 0.05;
        band.dt[i] = ds / v;
    }
}

inline void stopAtReversals(TimedBand& band, const TebLimits& lim) {
    const int n = band.count;
    if (n < 3) return;
    bool any = false;
    double a_long = lim.a_long;
    if (a_long < Motion::MIN_ACCEL_MPS2) a_long = Motion::MIN_ACCEL_MPS2;
    for (int i = 1; i < n - 1; ++i) {
        if ((band.drive[i - 1] < 0.0) == (band.drive[i] < 0.0)) continue;
        any = true;
        band.v[i] = 0.0;
        for (int j = i - 1; j >= 0; --j) {
            double ds = band.s[j + 1] - band.s[j];
            if (ds < 1e-6) ds = 1e-6;
            const double vb = std::sqrt(band.v[j + 1] * band.v[j + 1] + 2.0 * a_long * ds);
            if (vb < band.v[j]) band.v[j] = vb;
        }
        for (int j = i + 1; j < n; ++j) {
            double ds = band.s[j] - band.s[j - 1];
            if (ds < 1e-6) ds = 1e-6;
            const double vf = std::sqrt(band.v[j - 1] * band.v[j - 1] + 2.0 * a_long * ds);
            if (vf < band.v[j]) band.v[j] = vf;
        }
    }
    if (!any) return;
    for (int i = 0; i < n - 1; ++i) {
        double ds = band.s[i + 1] - band.s[i];
        if (ds < 1e-4) ds = 1e-4;
        double v = band.v[i];
        if (v < 0.05) v = 0.05;
        band.dt[i] = ds / v;
    }
    band.v_plan = band.v[0];
}

inline bool oneDrive(const TimedBand& band) {
    if (band.count < 1) return true;
    const bool back = band.drive[0] < 0.0;
    for (int i = 1; i < band.count; ++i) {
        if ((band.drive[i] < 0.0) != back) return false;
    }
    return true;
}

// Push a free pose out of a circular keep-out, and optionally back inside
// the field rectangle. End poses stay put: they are the robot and the goal.
inline void repelObstacles(TimedBand& band, const TebLimits& lim) {
    const int n = band.count;
    if (n < 5) return;
    const int nobs = (lim.obstacle_n > kTebObstacles) ? kTebObstacles
                   : ((lim.obstacle_n < 0) ? 0 : lim.obstacle_n);
    for (int i = 2; i <= n - 3; ++i) {
        double px = band.x[i];
        double py = band.y[i];
        for (int k = 0; k < nobs; ++k) {
            const TebObstacle& obs = lim.obstacles[k];
            const double dx = px - obs.x;
            const double dy = py - obs.y;
            const double d = std::hypot(dx, dy);
            if (d < obs.min_dist && d > 1e-4) {
                const double push = kTebObstacleWeight * (obs.min_dist - d) / d;
                px += push * dx;
                py += push * dy;
            }
        }
        if (lim.field_bound) {
            Robot::clampFieldMeters(px, py, WallSolve::FIELD_MARGIN_M);
        }
        if (!bandStepOk(band, i, px, py)) continue;
        band.x[i] = px;
        band.y[i] = py;
    }
}

// Defined below. Declared here so the retime pass can call it.
inline void spreadYawAccel(TimedBand& band, double alpha_max);

// Third derivative of the chassis heading the drift loop commands.
// ψ = φ + σ, and σ comes from a_lat = v² κ. There is no measured sideslip
// while the path is planned. Longitudinal jerk is zero, matching the
// feedforward:
//
//   φ⃛ = 3 v a κ' + v³ κ''
//   a_lat⃛ = 12 v a² κ' + 9 v³ a κ'' + v⁵ κ'''
//
// Arcsin and atan turn (u, u̇, ü, u⃛) into σ⃛, with u = a_lat / slip_accel.
// On the thrust clamp, and once |σ| is on its limit, those derivatives are
// zero and the loss keeps only φ⃛.
inline double driftHeadingJerk(double v, double a, double kappa, double kp, double kpp,
                               double kppp, const TebLimits& lim) {
    if (v < 0.0) v = 0.0;
    if (!std::isfinite(v) || !std::isfinite(a) || !std::isfinite(kappa)
        || !std::isfinite(kp) || !std::isfinite(kpp) || !std::isfinite(kppp)) {
        return 0.0;
    }
    const double v2 = v * v;
    const double v3 = v2 * v;
    const double geo = 3.0 * v * a * kp + v3 * kpp;
    const double scale = lim.slip_accel;
    if (!(scale > 1e-6)) return geo;

    const double a_lat = v2 * kappa;
    const double da = 2.0 * v * a * kappa + v3 * kp;
    const double d2a = 2.0 * a * a * kappa + 5.0 * v2 * a * kp + v2 * v2 * kpp;
    const double d3a = 12.0 * v * a * a * kp + 9.0 * v3 * a * kpp + v2 * v3 * kppp;
    const double u = a_lat / scale;
    const double du = da / scale;
    const double d2u = d2a / scale;
    const double d3u = d3a / scale;

    if (lim.slip_arcsin) {
        if (u >= 1.0 || u <= -1.0) return geo;
        if (lim.slip_limit > 0.0 && std::fabs(std::asin(u)) >= lim.slip_limit) return geo;
        const double one = 1.0 - u * u;
        if (one < 1e-4) return geo;
        const double root = std::sqrt(one);
        const double num = d3u * one * one
                         + 3.0 * u * d2u * du * one
                         + du * du * du * (1.0 + 2.0 * u * u);
        const double slip = num / (one * one * root);
        return std::isfinite(slip) ? geo + slip : geo;
    }

    if (lim.slip_limit > 0.0 && std::fabs(std::atan(u)) >= lim.slip_limit) return geo;
    const double den = 1.0 + u * u;
    const double num = d3u * den * den
                     - 6.0 * u * d2u * du * den
                     + 2.0 * du * du * du * (3.0 * u * u - 1.0);
    const double slip = num / (den * den * den);
    return std::isfinite(slip) ? geo + slip : geo;
}

inline bool driveSeamAt(const TimedBand& band, int i) {
    if (i <= 0 || i >= band.count) return false;
    return (band.drive[i - 1] < 0.0) != (band.drive[i] < 0.0);
}

inline double derivSpan(const TimedBand& band, int i0, int i1) {
    if (i1 <= i0) return 0.0;
    for (int k = i0 + 1; k <= i1; ++k) {
        if (driveSeamAt(band, k)) return 0.0;
    }
    const double ds = band.s[i1] - band.s[i0];
    return (ds > 1e-4) ? ds : 0.0;
}

// One [1, 2, 1] pass so a 13-pose third difference is not sampling noise.
inline void smoothKappa(const TimedBand& band, double* ks) {
    const int n = band.count;
    if (ks == nullptr || n < 1) return;
    for (int i = 0; i < n; ++i) ks[i] = band.kappa_s[i];
    if (n < 3) return;
    double raw[kTebPoses];
    for (int i = 0; i < n; ++i) raw[i] = band.kappa_s[i];
    for (int i = 1; i < n - 1; ++i) {
        if (driveSeamAt(band, i) || driveSeamAt(band, i + 1)) continue;
        ks[i] = 0.25 * raw[i - 1] + 0.5 * raw[i] + 0.25 * raw[i + 1];
    }
}

inline void fillKappaDerivatives(const TimedBand& band, const double* ks,
                                 double* kp, double* kpp, double* kppp) {
    const int n = band.count;
    if (ks == nullptr || kp == nullptr || kpp == nullptr || kppp == nullptr) return;
    for (int i = 0; i < n; ++i) {
        kp[i] = 0.0;
        kpp[i] = 0.0;
        kppp[i] = 0.0;
    }
    auto slope = [&](const double* y, int i0, int i1) {
        const double ds = derivSpan(band, i0, i1);
        if (!(ds > 0.0)) return 0.0;
        return (y[i1] - y[i0]) / ds;
    };
    if (n >= 2) {
        kp[0] = slope(ks, 0, 1);
        kp[n - 1] = slope(ks, n - 2, n - 1);
    }
    for (int i = 1; i < n - 1; ++i) kp[i] = slope(ks, i - 1, i + 1);
    if (n >= 2) {
        kpp[0] = slope(kp, 0, 1);
        kpp[n - 1] = slope(kp, n - 2, n - 1);
    }
    for (int i = 1; i < n - 1; ++i) kpp[i] = slope(kp, i - 1, i + 1);
    if (n >= 2) {
        kppp[0] = slope(kpp, 0, 1);
        kppp[n - 1] = slope(kpp, n - 2, n - 1);
    }
    for (int i = 1; i < n - 1; ++i) kppp[i] = slope(kpp, i - 1, i + 1);
}

inline double accelAlong(const TimedBand& band, const double* v_ref, int i) {
    const int n = band.count;
    if (v_ref == nullptr || i < 0 || i >= n) return 0.0;
    const int i0 = (i > 0) ? (i - 1) : i;
    const int i1 = (i + 1 < n) ? (i + 1) : i;
    const double ds = derivSpan(band, i0, i1);
    if (!(ds > 0.0)) return 0.0;
    double v = v_ref[i];
    if (v < 0.0) v = 0.0;
    return v * (v_ref[i1] - v_ref[i0]) / ds;
}

struct HeadingJerkTerms {
    double ks[kTebPoses] {};
    double kp[kTebPoses] {};
    double kpp[kTebPoses] {};
    double kppp[kTebPoses] {};
};

inline void fillHeadingJerkTerms(const TimedBand& band, HeadingJerkTerms& terms) {
    smoothKappa(band, terms.ks);
    fillKappaDerivatives(band, terms.ks, terms.kp, terms.kpp, terms.kppp);
}

inline double headingJerkAt(const TimedBand& band, const double* v_ref, const TebLimits& lim,
                            const HeadingJerkTerms& terms, int i) {
    if (i < 0 || i >= band.count || driveSeamAt(band, i)) return 0.0;
    double v = (v_ref != nullptr) ? v_ref[i] : 0.0;
    if (v < 0.0) v = 0.0;
    const double a = (v_ref != nullptr) ? accelAlong(band, v_ref, i) : 0.0;
    return driftHeadingJerk(v, a, terms.ks[i], terms.kp[i], terms.kpp[i], terms.kppp[i], lim);
}

// Sum of ψ⃛² on [lo, hi]. peak_out receives the largest |ψ⃛| there.
inline double windowHeadingEnergy(const TimedBand& band, const double* v_ref, const TebLimits& lim,
                                  int lo, int hi, double* peak_out) {
    if (lo < 0) lo = 0;
    if (hi >= band.count) hi = band.count - 1;
    HeadingJerkTerms terms;
    fillHeadingJerkTerms(band, terms);
    double energy = 0.0;
    double peak = 0.0;
    for (int j = lo; j <= hi; ++j) {
        const double jerk = headingJerkAt(band, v_ref, lim, terms, j);
        const double mag = std::fabs(jerk);
        if (mag > peak) peak = mag;
        energy += jerk * jerk;
    }
    if (peak_out != nullptr) *peak_out = peak;
    return energy;
}

// Largest |ψ⃛| on the band at the curvature-free reference speed, rad/s³.
inline double peakDriftHeadingJerk(const TimedBand& band_in, const TebLimits& lim) {
    TimedBand band = band_in;
    if (band.count < 3) return 0.0;
    measureBand(band);
    clearCuspCurvature(band);
    double v_ref[kTebPoses];
    referenceSpeed(band, lim, v_ref);
    double peak = 0.0;
    windowHeadingEnergy(band, v_ref, lim, 1, band.count - 2, &peak);
    return peak;
}

// One sweep of the free poses down the gradient of ψ⃛². The reference speed
// ignores curvature, so a kink is judged at the speed the leg wants to hold.
// A step that does not lower the local loss is put back.
inline void relaxHeadingJerk(TimedBand& band, const double* v_ref, const TebLimits& lim) {
    const int n = band.count;
    if (n < 6 || v_ref == nullptr) return;
    if (!(kTebYawJerk > 1e-3) || !(kTebJerkStep > 1e-6)) return;
    const double eps = 1.0e-3;
    for (int i = 2; i <= n - 3; ++i) {
        if ((band.drive[i - 1] < 0.0) != (band.drive[i] < 0.0)) continue;
        if ((band.drive[i] < 0.0) != (band.drive[i + 1] < 0.0)) continue;
        measureBand(band);
        clearCuspCurvature(band);
        const int lo = (i > 4) ? (i - 3) : 1;
        const int hi = (i + 3 < n - 2) ? (i + 3) : (n - 2);
        double peak = 0.0;
        const double before = windowHeadingEnergy(band, v_ref, lim, lo, hi, &peak);
        if (!std::isfinite(before)) continue;
        double step = kTebJerkStep * peak / kTebYawJerk;
        if (step > kTebJerkStep) step = kTebJerkStep;
        if (step < 1.0e-4) continue;

        const double x0 = band.x[i];
        const double y0 = band.y[i];
        auto energyAt = [&](double x, double y) {
            band.x[i] = x;
            band.y[i] = y;
            measureBand(band);
            clearCuspCurvature(band);
            return windowHeadingEnergy(band, v_ref, lim, lo, hi, nullptr);
        };
        const double ep = energyAt(x0 + eps, y0);
        const double em = energyAt(x0 - eps, y0);
        const double gp = energyAt(x0, y0 + eps);
        const double gm = energyAt(x0, y0 - eps);
        band.x[i] = x0;
        band.y[i] = y0;
        if (!std::isfinite(ep) || !std::isfinite(em) || !std::isfinite(gp) || !std::isfinite(gm)) {
            measureBand(band);
            clearCuspCurvature(band);
            continue;
        }
        const double gx = (ep - em) / (2.0 * eps);
        const double gy = (gp - gm) / (2.0 * eps);
        const double g = std::hypot(gx, gy);
        if (!(g > 1e-9)) {
            measureBand(band);
            clearCuspCurvature(band);
            continue;
        }
        const double px = x0 - step * gx / g;
        const double py = y0 - step * gy / g;
        if (!bandStepOk(band, i, px, py)) {
            measureBand(band);
            clearCuspCurvature(band);
            continue;
        }
        const double after = energyAt(px, py);
        if (!std::isfinite(after) || !(after < before)) {
            band.x[i] = x0;
            band.y[i] = y0;
        }
        measureBand(band);
        clearCuspCurvature(band);
    }
}

// Retiming and the curvature pass. `iters` is a full cold start or the
// short warm update. The first and last segments are not free.
inline void optimizeBand(TimedBand& band, const TebLimits& lim, int iters) {
    if (band.count < 2) return;
    if (iters < 1) iters = 1;
    const bool uniform = oneDrive(band);
    for (int iter = 0; iter < iters; ++iter) {
        measureBand(band);
        clearCuspCurvature(band);
        syncHeadings(band, lim);
        if (band.count >= 5) {
            if (uniform) {
                double v_ref[kTebPoses];
                referenceSpeed(band, lim, v_ref);
                bendBand(band, v_ref, lim.a_cent, lim.w_max);
                relaxHeadingJerk(band, v_ref, lim);
            }
            repelObstacles(band, lim);
            if (uniform) orderBand(band);
        }
    }
    measureBand(band);
    clearCuspCurvature(band);
    syncHeadings(band, lim);
    projectDt(band, lim);
    stopAtReversals(band, lim);
    spreadYawAccel(band, lim.alpha_max);
    if (band.count > 0) {
        band.phi = band.phi_s[0];
        band.kappa = band.kappa_s[0];
        band.v_plan = band.v[0];
        band.length_m = band.s[band.count - 1];
    }
}

inline void copyPose(TimedBand& band, int dst, int src) {
    band.x[dst] = band.x[src];
    band.y[dst] = band.y[src];
    band.theta[dst] = band.theta[src];
    band.s[dst] = band.s[src];
    band.phi_s[dst] = band.phi_s[src];
    band.kappa_s[dst] = band.kappa_s[src];
    band.v[dst] = band.v[src];
    band.drive[dst] = band.drive[src];
    band.dt[dst] = band.dt[src];
}

inline void shiftBand(TimedBand& band, int drop) {
    const int n = band.count;
    if (drop <= 0 || drop >= n) return;
    const int keep = n - drop;
    for (int i = 0; i < keep; ++i) copyPose(band, i, i + drop);
    band.count = keep;
}

// Hold the last segment on the exit heading. A hold longer than the
// remaining path lands behind the robot and the first heading turns around.
inline void pinExit(TimedBand& band, double tx, double ty, double approach, double hold) {
    const int n = band.count;
    if (n < 3) return;
    const double reach = std::hypot(tx - band.x[0], ty - band.y[0]);
    if (hold > 0.45 * reach) hold = 0.45 * reach;
    if (hold < kTebMinSegM && reach > 2.0 * kTebMinSegM) hold = kTebMinSegM;
    if (hold < 1e-4) hold = 1e-4;
    band.x[n - 1] = tx;
    band.y[n - 1] = ty;
    band.x[n - 2] = tx - hold * mathutil::heading_cos(approach);
    band.y[n - 2] = ty - hold * mathutil::heading_sin(approach);
}

// Pose 1 has to lead the direction of travel. A knot the robot has already
// passed turns the opening segment back down the old band.
inline bool leadTheNose(TimedBand& band, const TebLimits& lim) {
    const int n = band.count;
    if (n < 3) return false;
    const double hx = mathutil::heading_cos(lim.motion);
    const double hy = mathutil::heading_sin(lim.motion);
    auto lead = [&](int i) {
        return (band.x[i] - band.x[0]) * hx + (band.y[i] - band.y[0]) * hy;
    };
    if (lead(1) >= kTebMinSegM) return true;
    int j = 2;
    while (j < n - 1 && lead(j) < kTebMinSegM) ++j;
    if (j >= n - 1 || lead(j) < kTebMinSegM) return false;
    const double dx = band.x[j] - band.x[0];
    const double dy = band.y[j] - band.y[0];
    const double dist = std::hypot(dx, dy);
    if (dist < 1e-4) return false;
    double step = dist / static_cast<double>(j);
    if (step < kTebMinSegM) step = kTebMinSegM;
    if (step > 0.5 * dist) step = 0.5 * dist;
    band.x[1] = band.x[0] + (dx / dist) * step;
    band.y[1] = band.y[0] + (dy / dist) * step;
    if (j > 2) {
        const int tail = n - j;
        for (int i = 0; i < tail; ++i) copyPose(band, 2 + i, j + i);
        band.count = 2 + tail;
    }
    return band.count >= 3;
}

inline bool frontGapLong(const TimedBand& band) {
    if (band.count < 3) return false;
    const double lead = std::hypot(band.x[1] - band.x[0], band.y[1] - band.y[0]);
    double rest = 0.0;
    int m = 0;
    for (int i = 2; i < band.count; ++i) {
        rest += std::hypot(band.x[i] - band.x[i - 1], band.y[i] - band.y[i - 1]);
        ++m;
    }
    if (m < 1) return lead > 2.0 * kTebMinSegM;
    return lead > 1.8 * (rest / static_cast<double>(m));
}

// Spread the polyline back out to a full band so the next knot stays a
// short step ahead. Skipped when that would pack the segments under the
// minimum the step check will accept.
inline void resampleBand(TimedBand& band, int n) {
    if (n < 2 || n > kTebPoses || band.count < 2 || n == band.count) return;
    measureBand(band);
    const double length = band.s[band.count - 1];
    if (!(length > 1e-4)) return;
    if (length / static_cast<double>(n - 1) < kTebMinSegM) return;
    const int old = band.count;
    double xs[kTebPoses];
    double ys[kTebPoses];
    double drv[kTebPoses];
    for (int i = 0; i < n; ++i) {
        const double s = length * (static_cast<double>(i) / static_cast<double>(n - 1));
        int k = 1;
        while (k < old - 1 && band.s[k] + 1e-9 < s) ++k;
        const double ds = band.s[k] - band.s[k - 1];
        double t = (ds > 1e-9) ? (s - band.s[k - 1]) / ds : 0.0;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        xs[i] = band.x[k - 1] + t * (band.x[k] - band.x[k - 1]);
        ys[i] = band.y[k - 1] + t * (band.y[k] - band.y[k - 1]);
        drv[i] = band.drive[k - 1];
    }
    xs[0] = band.x[0];
    ys[0] = band.y[0];
    xs[n - 1] = band.x[old - 1];
    ys[n - 1] = band.y[old - 1];
    for (int i = 0; i < n; ++i) {
        band.x[i] = xs[i];
        band.y[i] = ys[i];
        band.drive[i] = drv[i];
    }
    band.count = n;
    band.curved = true;
}

// Continue a band that already reaches `tx, ty`. Drops poses behind the
// robot, snaps the first pose to the measured state, and runs a few passes.
// False means the caller should seed a new band.
inline bool retainTimedBand(TimedBand& band, double rx, double ry,
                            double tx, double ty, double approach,
                            const TebLimits& lim) {
    const int n0 = band.count;
    if (n0 < 5 || n0 > kTebPoses) return false;
    if (std::hypot(band.x[n0 - 1] - tx, band.y[n0 - 1] - ty) > 0.30) return false;
    if (band.drive[n0 - 1] * lim.finish_drive < 0.0) return false;
    const double exit = mathutil::direction_heading(band.x[n0 - 1] - band.x[n0 - 2],
                                                   band.y[n0 - 1] - band.y[n0 - 2]);
    if (std::fabs(mathutil::normalize_angle(exit - approach)) > 0.80) return false;

    double best = 1e9;
    int seg = 0;
    double best_t = 0.0;
    for (int i = 0; i < n0 - 1; ++i) {
        const double ax = band.x[i + 1] - band.x[i];
        const double ay = band.y[i + 1] - band.y[i];
        const double len2 = ax * ax + ay * ay;
        double t = 0.0;
        if (len2 > 1e-8) {
            t = ((rx - band.x[i]) * ax + (ry - band.y[i]) * ay) / len2;
            if (t < 0.0) t = 0.0;
            if (t > 1.0) t = 1.0;
        }
        const double px = band.x[i] + t * ax;
        const double py = band.y[i] + t * ay;
        const double d = std::hypot(rx - px, ry - py);
        if (d < best) {
            best = d;
            seg = i;
            best_t = t;
        }
    }
    if (best > 0.45) return false;
    // Advance once the robot is on the segment. Waiting for the far end
    // leaves the next knot behind the nose.
    int drop = seg;
    if (best_t > 0.25 && n0 - (drop + 1) >= 3) drop += 1;
    if (n0 - drop < 3) return false;
    shiftBand(band, drop);

    band.x[0] = rx;
    band.y[0] = ry;
    band.theta[0] = lim.nose;
    const int n = band.count;
    double hold = std::hypot(band.x[n - 1] - band.x[n - 2], band.y[n - 1] - band.y[n - 2]);
    pinExit(band, tx, ty, approach, hold);
    if (!leadTheNose(band, lim)) return false;
    if (band.count < kTebPoses || frontGapLong(band)) resampleBand(band, kTebPoses);
    pinExit(band, tx, ty, approach, hold);
    if (band.count < 3) return false;
    if (std::hypot(band.x[1] - band.x[0], band.y[1] - band.y[0]) < 0.5 * kTebMinSegM) return false;
    optimizeBand(band, lim, kTebWarmIters);
    return band.count >= 2 && std::isfinite(band.length_m) && band.length_m > 1e-4;
}

inline void sampleCubic(double p0x, double p0y, double p1x, double p1y,
                        double p2x, double p2y, double p3x, double p3y,
                        double t, double& x, double& y) {
    const double u = 1.0 - t;
    const double uu = u * u;
    const double tt = t * t;
    const double a = uu * u;
    const double b = 3.0 * uu * t;
    const double c = 3.0 * u * tt;
    const double d = tt * t;
    x = a * p0x + b * p1x + c * p2x + d * p3x;
    y = a * p0y + b * p1y + c * p2y + d * p3y;
}

// Brake to a stop where the drive sign flips, then accelerate away.
// The heading jump there is the reverse, not a turn.
inline void timeCusp(TimedBand& band, const TebLimits& lim) {
    measureBand(band);
    const int n = band.count;
    for (int i = 1; i < n - 1; ++i) {
        if ((band.drive[i - 1] < 0.0) != (band.drive[i] < 0.0)) band.kappa_s[i] = 0.0;
    }
    if (n > 1) band.kappa_s[0] = band.kappa_s[1];
    if (n > 0) band.kappa = band.kappa_s[0];
    timeBandInto(band, lim, nullptr);
    double a_long = lim.a_long;
    if (a_long < Motion::MIN_ACCEL_MPS2) a_long = Motion::MIN_ACCEL_MPS2;
    for (int i = 1; i < n - 1; ++i) {
        if ((band.drive[i - 1] < 0.0) == (band.drive[i] < 0.0)) continue;
        band.v[i] = 0.0;
        for (int j = i - 1; j >= 0; --j) {
            double ds = band.s[j + 1] - band.s[j];
            if (ds < 1e-6) ds = 1e-6;
            const double vb = std::sqrt(band.v[j + 1] * band.v[j + 1] + 2.0 * a_long * ds);
            if (vb < band.v[j]) band.v[j] = vb;
        }
        for (int j = i + 1; j < n; ++j) {
            double ds = band.s[j] - band.s[j - 1];
            if (ds < 1e-6) ds = 1e-6;
            const double vf = std::sqrt(band.v[j - 1] * band.v[j - 1] + 2.0 * a_long * ds);
            if (vf < band.v[j]) band.v[j] = vf;
        }
    }
    band.v_plan = (n > 0) ? band.v[0] : 0.0;
    stampClock(band, lim);
}

// Goal behind the nose, exit facing the other way. Reverse off to one
// side, stop, and come in forward. `side` is +1 for left of the reverse
// tangent and −1 for right. Empty when the bow has no room.
inline TimedBand swingCusp(double rx, double ry, double tx, double ty, double approach,
                           const TebLimits& lim, double side) {
    TimedBand band;
    const double gx = tx - rx;
    const double gy = ty - ry;
    const double reach = std::hypot(gx, gy);
    if (reach < 0.08) return band;
    const double rev = mathutil::normalize_angle(lim.motion + Robot::PI);
    const double rxu = mathutil::heading_cos(rev);
    const double ryu = mathutil::heading_sin(rev);
    const double lx = -side * ryu;
    const double ly = side * rxu;
    // A short bow. A wider one loses to the half-turn on the chord, because
    // the yaw accel makes that spin cheap and the extra length does not.
    double h = 0.28 * reach;
    if (h < 0.16) h = 0.16;
    if (h > 0.55) h = 0.55;
    const double cx = rx + 0.60 * gx + h * lx;
    const double cy = ry + 0.60 * gy + h * ly;
    if (std::hypot(cx - rx, cy - ry) < 0.12) return band;
    if (std::hypot(tx - cx, ty - cy) < 0.12) return band;

    const double d1 = 0.40 * std::hypot(cx - rx, cy - ry);
    const double d2 = 0.40 * std::hypot(tx - cx, ty - cy);
    const double ex = mathutil::heading_cos(approach);
    const double ey = mathutil::heading_sin(approach);
    const double a1x = rx + d1 * rxu;
    const double a1y = ry + d1 * ryu;
    const double b1x = cx - d1 * lx;
    const double b1y = cy - d1 * ly;
    const double a2x = cx - d2 * lx;
    const double a2y = cy - d2 * ly;
    const double b2x = tx - d2 * ex;
    const double b2y = ty - d2 * ey;

    const int n = kTebPoses;
    const int split = n / 2;
    band.count = n;
    band.curved = true;
    for (int i = 0; i <= split; ++i) {
        const double t = (split > 0) ? static_cast<double>(i) / static_cast<double>(split) : 0.0;
        sampleCubic(rx, ry, a1x, a1y, b1x, b1y, cx, cy, t, band.x[i], band.y[i]);
    }
    const int tail = n - 1 - split;
    for (int i = split; i < n; ++i) {
        const double t = (tail > 0)
            ? static_cast<double>(i - split) / static_cast<double>(tail) : 1.0;
        sampleCubic(cx, cy, a2x, a2y, b2x, b2y, tx, ty, t, band.x[i], band.y[i]);
    }
    band.x[0] = rx;
    band.y[0] = ry;
    band.x[split] = cx;
    band.y[split] = cy;
    band.x[n - 1] = tx;
    band.y[n - 1] = ty;
    for (int i = 0; i < n; ++i) band.drive[i] = (i < split) ? -1.0 : lim.finish_drive;
    timeCusp(band, lim);
    return band;
}

// One reversal. Leg 1 runs along the direction of travel (forward or
// back). Leg 2 arrives on the exit direction with finish_drive. Empty
// when the lines miss.
inline TimedBand cuspBand(double rx, double ry, double tx, double ty, double approach,
                          const TebLimits& lim) {
    TimedBand band;
    const double nx = mathutil::heading_cos(lim.motion);
    const double ny = mathutil::heading_sin(lim.motion);
    const double ex = mathutil::heading_cos(approach);
    const double ey = mathutil::heading_sin(approach);
    const double gx = tx - rx;
    const double gy = ty - ry;
    const double reach = std::hypot(gx, gy);
    if (reach < 0.08) return band;
    const double behind = gx * nx + gy * ny;
    const double align = ex * nx + ey * ny;
    const double det = nx * ey - ny * ex;

    double cx = 0.0;
    double cy = 0.0;
    double t_nose = 0.0;
    bool have = false;
    if (std::fabs(det) < 0.25) {
        // Goal on the nose line. Back past it, then come in along the exit,
        // when that exit faces the same way as the nose and the goal is behind.
        if (behind < -0.05 && align > 0.5 && lim.finish_drive > 0.0) {
            double handle = 0.22 * reach;
            if (handle < 0.18) handle = 0.18;
            cx = tx - nx * handle;
            cy = ty - ny * handle;
            t_nose = (cx - rx) * nx + (cy - ry) * ny;
            have = true;
        } else if (behind > 0.05 && align < -0.5 && lim.finish_drive < 0.0) {
            // Goal in front, finish backing up: drive past it, then reverse in.
            double handle = 0.22 * reach;
            if (handle < 0.18) handle = 0.18;
            cx = tx + nx * handle;
            cy = ty + ny * handle;
            t_nose = (cx - rx) * nx + (cy - ry) * ny;
            have = true;
        } else if (lim.finish_drive > 0.0) {
            // Travel and exit are parallel and the exit faces the other way.
            // A straight chord would be a half turn. Reverse off to one side
            // and come in forward, the same basin ROS TEB seeds when the
            // goal sits behind the nose.
            const double nose_x = mathutil::heading_cos(lim.nose);
            const double nose_y = mathutil::heading_sin(lim.nose);
            const double along_nose = gx * nose_x + gy * nose_y;
            const double exit_nose = ex * nose_x + ey * nose_y;
            if (along_nose < -0.05 && exit_nose < -0.5) {
                const TimedBand left = swingCusp(rx, ry, tx, ty, approach, lim, 1.0);
                const TimedBand right = swingCusp(rx, ry, tx, ty, approach, lim, -1.0);
                const double tl = (left.count >= 2)
                    ? bandTravelTime(left, lim.v_start, lim.nose) : 1e9;
                const double tr = (right.count >= 2)
                    ? bandTravelTime(right, lim.v_start, lim.nose) : 1e9;
                return (tl <= tr) ? left : right;
            }
        }
    } else {
        const double t = (gx * ey - gy * ex) / det;
        const double s = (nx * gy - ny * gx) / det;
        const double far = reach * 2.5 + 0.45;
        // The lines can meet behind a goal that is still in front of the
        // nose. That reversal is a U-turn the robot does not need; ROS TEB
        // keeps a forward velocity there and only reverses when the goal
        // itself is behind the heading.
        if (behind < -0.05 && s < -0.08 && std::fabs(t) > 0.08
            && std::fabs(t) < far && std::fabs(s) < far) {
            cx = rx + t * nx;
            cy = ry + t * ny;
            t_nose = t;
            have = true;
        }
    }
    if (!have) return band;
    if (std::hypot(cx - rx, cy - ry) < 0.08) return band;
    if (std::hypot(tx - cx, ty - cy) < 0.08) return band;

    const int n = kTebPoses;
    const int split = n / 2;
    band.count = n;
    band.curved = true;
    for (int i = 0; i <= split; ++i) {
        const double u = (split > 0) ? static_cast<double>(i) / static_cast<double>(split) : 0.0;
        band.x[i] = rx + u * (cx - rx);
        band.y[i] = ry + u * (cy - ry);
    }
    const int tail = n - 1 - split;
    for (int i = split; i < n; ++i) {
        const double u = (tail > 0)
            ? static_cast<double>(i - split) / static_cast<double>(tail) : 1.0;
        band.x[i] = cx + u * (tx - cx);
        band.y[i] = cy + u * (ty - cy);
    }
    band.x[0] = rx;
    band.y[0] = ry;
    band.x[split] = cx;
    band.y[split] = cy;
    band.x[n - 1] = tx;
    band.y[n - 1] = ty;
    const double leg1 = (t_nose >= 0.0) ? 1.0 : -1.0;
    for (int i = 0; i < n; ++i) band.drive[i] = (i < split) ? leg1 : lim.finish_drive;
    timeCusp(band, lim);
    return band;
}

// Direction the opening segment is pinned to. Stopped, there is no travel
// direction. A reverse finish with the goal behind the nose backs out.
// Once the robot is moving, the measured direction wins over the heading.
inline void openingRay(double rx, double ry, double tx, double ty,
                       double vx, double vy, const TebLimits& lim,
                       double& t0x, double& t0y) {
    const double dir = std::hypot(vx, vy);
    t0x = mathutil::heading_cos(lim.motion);
    t0y = mathutil::heading_sin(lim.motion);
    if (dir >= 1e-9) {
        t0x = vx / dir;
        t0y = vy / dir;
    }
    const double nose_x = mathutil::heading_cos(lim.nose);
    const double nose_y = mathutil::heading_sin(lim.nose);
    const double goal_along = (tx - rx) * nose_x + (ty - ry) * nose_y;
    if (lim.v_start < kMomentumSpeed && lim.finish_drive < 0.0 && goal_along < 0.05) {
        t0x = -nose_x;
        t0y = -nose_y;
    }
}

// Keep the forward band, or the one-reversal alternative when that reaches
// the exit sooner. drive_hint holds the choice across a tick of noise.
inline TimedBand chooseCusp(TimedBand forward, double rx, double ry, double tx, double ty,
                            double approach, const TebLimits& lim) {
    stampDrive(forward, lim.finish_drive);
    const TimedBand cusp = cuspBand(rx, ry, tx, ty, approach, lim);
    const double tf = bandTravelTime(forward, lim.v_start, lim.nose);
    const double tc = (cusp.count >= 2) ? bandTravelTime(cusp, lim.v_start, lim.nose) : 1e9;
    bool take = cusp.count >= 2 && tc < tf;
    if (lim.drive_hint < -0.5 && cusp.count >= 2 && cusp.drive[0] < 0.0 && tc < tf * 1.15)
        take = true;
    if (lim.drive_hint > 0.5 && forward.drive[0] > 0.0 && tf < tc * 1.15)
        take = false;
    return take ? cusp : forward;
}

// Stretch any Δt whose heading change reverses faster than the chassis
// can yaw. Time optimality keeps the floor; this only raises it. A
// straight band is unchanged.
inline void spreadYawAccel(TimedBand& band, double alpha_max) {
    const int n = band.count;
    if (n < 3 || !(alpha_max > 1e-3)) return;
    for (int pass = 0; pass < 3; ++pass) {
        for (int i = 0; i < n - 2; ++i) {
            double dt0 = band.dt[i];
            double dt1 = band.dt[i + 1];
            if (!(dt0 > 1e-3) || !std::isfinite(dt0)) dt0 = 1e-3;
            if (!(dt1 > 1e-3) || !std::isfinite(dt1)) dt1 = 1e-3;
            const double w0 = mathutil::normalize_angle(band.theta[i + 1] - band.theta[i]) / dt0;
            const double w1 = mathutil::normalize_angle(band.theta[i + 2] - band.theta[i + 1]) / dt1;
            const double mid = 0.5 * (dt0 + dt1);
            const double alpha = (w1 - w0) / mid;
            if (std::fabs(alpha) <= alpha_max) continue;
            const double grow = std::sqrt(std::fabs(alpha) / alpha_max);
            dt0 *= grow;
            dt1 *= grow;
            if (dt0 > 2.0) dt0 = 2.0;
            if (dt1 > 2.0) dt1 = 2.0;
            band.dt[i] = dt0;
            band.dt[i + 1] = dt1;
        }
    }
    for (int i = 0; i < n - 1; ++i) {
        double ds = band.s[i + 1] - band.s[i];
        if (ds < 1e-4) ds = 1e-4;
        double dt = band.dt[i];
        if (!(dt > 1e-3) || !std::isfinite(dt)) dt = 1e-3;
        const double v = ds / dt;
        band.v[i] = std::isfinite(v) ? v : 0.0;
    }
    band.v_plan = band.v[0];
}

// Pin the ends, then run the curvature and time-step passes from scratch.
// The last segment stays on the exit direction. pin_opening also pulls the
// first segment onto the measured momentum; the local planner leaves it on
// the polyline it was seeded from, because a nose that is already hunting
// would become the next heading target. A failed pass keeps the pinned seed.
inline TimedBand relaxSeed(TimedBand band, double approach, double t0x, double t0y,
                           const TebLimits& lim, bool pin_opening = true) {
    const int n = band.count;
    if (n < 2 || n > kTebPoses) return band;
    const double rx = band.x[0];
    const double ry = band.y[0];
    const double tx = band.x[n - 1];
    const double ty = band.y[n - 1];
    band.x[0] = rx;
    band.y[0] = ry;
    band.x[n - 1] = tx;
    band.y[n - 1] = ty;
    if (n >= 4) {
        const double dist = std::hypot(rx - tx, ry - ty);
        const double ux = mathutil::heading_cos(approach);
        const double uy = mathutil::heading_sin(approach);
        double handle = dist / static_cast<double>(n - 1);
        if (handle < kTebMinSegM) handle = kTebMinSegM;
        if (handle > 0.22 * dist) handle = 0.22 * dist;
        if (pin_opening) {
            band.x[1] = rx + handle * t0x;
            band.y[1] = ry + handle * t0y;
        }
        band.x[n - 2] = tx - handle * ux;
        band.y[n - 2] = ty - handle * uy;
        orderBand(band);
    }

    double seed_x[kTebPoses];
    double seed_y[kTebPoses];
    for (int i = 0; i < n; ++i) {
        seed_x[i] = band.x[i];
        seed_y[i] = band.y[i];
    }

    stampDrive(band, lim.finish_drive);
    optimizeBand(band, lim, kTebIters);

    bool ok = std::isfinite(band.length_m) && band.length_m > 1e-4;
    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(band.x[i]) || !std::isfinite(band.y[i])) ok = false;
        if (!std::isfinite(band.theta[i]) || !std::isfinite(band.v[i])) ok = false;
    }
    for (int i = 0; ok && i < n - 1; ++i) {
        if (std::hypot(band.x[i + 1] - band.x[i], band.y[i + 1] - band.y[i]) < 1e-4) ok = false;
        if (!(band.dt[i] > 0.0) || !std::isfinite(band.dt[i])) ok = false;
    }
    if (!ok) {
        for (int i = 0; i < n; ++i) {
            band.x[i] = seed_x[i];
            band.y[i] = seed_y[i];
        }
        stampDrive(band, lim.finish_drive);
        measureBand(band);
        timeBandInto(band, lim, nullptr);
        stampClock(band, lim);
    }
    return chooseCusp(band, rx, ry, tx, ty, approach, lim);
}

// Hermite seed, then the curvature and time-step passes. Used when the
// caller has no polyline yet. kappa0 shapes the seed; a bend the
// reference speed cannot afford is opened on the passes that follow.
inline TimedBand timedBandThrough(double rx, double ry, double tx, double ty,
                                  double approach, double vx, double vy,
                                  double kappa0, const TebLimits& lim) {
    TimedBand band;
    const double dx = rx - tx;
    const double dy = ry - ty;
    const double dist = std::hypot(rx - tx, ry - ty);
    const double ux = mathutil::heading_cos(approach);
    const double uy = mathutil::heading_sin(approach);
    const double nx = -uy;
    const double ny = ux;
    const double xi0 = dx * ux + dy * uy;
    const double eta0 = dx * nx + dy * ny;

    auto finishStraight = [&](double phi) {
        bandStraight(band, rx, ry, tx, ty, phi);
        stampDrive(band, lim.finish_drive);
        timeBandInto(band, lim, nullptr);
        stampClock(band, lim);
        return band;
    };

    if (dist < 1e-3) return chooseCusp(finishStraight(approach), rx, ry, tx, ty, approach, lim);
    if (std::fabs(xi0) < kOnAxisM || std::fabs(eta0) < kOnAxisM) {
        double phi = approach;
        if (xi0 > 0.0) phi = mathutil::direction_heading(tx - rx, ty - ry);
        return chooseCusp(finishStraight(phi), rx, ry, tx, ty, approach, lim);
    }

    double t0x = 0.0;
    double t0y = 0.0;
    openingRay(rx, ry, tx, ty, vx, vy, lim, t0x, t0y);

    const int n = kTebPoses;
    band.count = n;
    band.curved = true;
    double v_seed = lim.v_start;
    if (v_seed < lim.v_cruise) {
        // The entry is timed from the speed this move will reach, not from a
        // standstill, so a stop does not license a kink the cruise cannot hold.
        double a_seed = lim.a_long;
        if (a_seed < 0.05) a_seed = 0.05;
        const double risen = std::sqrt(v_seed * v_seed + 2.0 * a_seed * dist);
        v_seed = (risen < lim.v_cruise) ? risen : lim.v_cruise;
    }
    const double k_cap = curvatureBudget(v_seed, lim.a_cent, lim.w_max);
    const double k_seed = mathutil::clamp_val(kappa0, -k_cap, k_cap);
    const Quintic seed = hermiteThrough(rx, ry, tx, ty, approach, t0x, t0y, k_seed);
    if (seed.curved) {
        for (int i = 0; i < n; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(n - 1);
            quinticAt(seed, t, band.x[i], band.y[i]);
        }
    } else {
        for (int i = 0; i < n; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(n - 1);
            band.x[i] = rx + t * (tx - rx);
            band.y[i] = ry + t * (ty - ry);
        }
    }
    return relaxSeed(band, approach, t0x, t0y, lim);
}

// Local-planner entry. `xs` and `ys` are the base trajectory in meters,
// beginning at the robot and ending at the curve join. The band is
// resampled from that polyline, then the curvature and time-step passes
// run again. A degenerate polyline falls back to the Hermite seed.
inline TimedBand timedBandAlong(const double* xs, const double* ys, int npts,
                                double approach, double vx, double vy,
                                double kappa0, const TebLimits& lim) {
    if (xs == nullptr || ys == nullptr || npts < 2) return {};
    if (npts > kSpeedKnots) npts = kSpeedKnots;
    const double rx = xs[0];
    const double ry = ys[0];
    const double tx = xs[npts - 1];
    const double ty = ys[npts - 1];
    double spt[kSpeedKnots];
    spt[0] = 0.0;
    for (int i = 1; i < npts; ++i) {
        spt[i] = spt[i - 1] + std::hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]);
    }
    const double length = spt[npts - 1];
    if (length < 1e-3 || std::hypot(tx - rx, ty - ry) < 1e-3) {
        return timedBandThrough(rx, ry, tx, ty, approach, vx, vy, kappa0, lim);
    }

    int n = kTebPoses;
    while (n > 2 && length / static_cast<double>(n - 1) < kTebMinSegM) --n;
    TimedBand band;
    band.count = n;
    band.curved = true;
    for (int i = 0; i < n; ++i) {
        const double s = length * (static_cast<double>(i) / static_cast<double>(n - 1));
        int k = 1;
        while (k < npts - 1 && spt[k] + 1e-9 < s) ++k;
        const double ds = spt[k] - spt[k - 1];
        double t = (ds > 1e-9) ? (s - spt[k - 1]) / ds : 0.0;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        band.x[i] = xs[k - 1] + t * (xs[k] - xs[k - 1]);
        band.y[i] = ys[k - 1] + t * (ys[k] - ys[k - 1]);
    }
    band.x[0] = rx;
    band.y[0] = ry;
    band.x[n - 1] = tx;
    band.y[n - 1] = ty;
    double t0x = 0.0;
    double t0y = 0.0;
    openingRay(rx, ry, tx, ty, vx, vy, lim, t0x, t0y);
    // Stopped, the first step can only leave along the wheels, so pose 1
    // is pinned to that ray. Once the robot is moving, pose 1 stays on the
    // polyline: pinning it to a hunting nose makes the tracked tangent wag.
    const bool pin_opening = lim.v_start < kMomentumSpeed;
    return relaxSeed(band, approach, t0x, t0y, lim, pin_opening);
}

} // namespace driftpath
