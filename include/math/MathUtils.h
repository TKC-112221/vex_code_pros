#pragma once
/* ---------------------------------------------------------------------------
 * MathUtils.h  --  Small pure math helpers shared across the pipeline.
 * Header-only/inline; ported from v1 (utils.cpp) plus the controller's sinc.
 * ------------------------------------------------------------------------- */

#include <cmath>

namespace mathutil {

inline double clamp_val(double v, double min_v, double max_v) {
    return (v < min_v) ? min_v : ((v > max_v) ? max_v : v);
}

inline double sign(double x) {
    return (x > 0.0) ? 1.0 : ((x < 0.0) ? -1.0 : 0.0);
}

// Wrap an angle to (-pi, pi].
inline double normalize_angle(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// sin(x)/x with the removable singularity handled.
inline double sinc(double x) {
    return (std::fabs(x) < 1e-6) ? 1.0 : (std::sin(x) / x);
}

} // namespace mathutil
