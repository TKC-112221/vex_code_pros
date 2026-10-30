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

// Field heading: 0 along +y, counterclockwise positive.
// heading_cos / heading_sin are the field components of that unit vector.
inline double heading_cos(double heading) { return -std::sin(heading); }
inline double heading_sin(double heading) { return std::cos(heading); }
inline float heading_cos(float heading) { return -std::sin(heading); }
inline float heading_sin(float heading) { return std::cos(heading); }

// Heading of a field displacement. A zero vector reports 0 (+y).
inline double direction_heading(double dx, double dy) {
    return normalize_angle(std::atan2(dy, dx) - M_PI / 2.0);
}

// sin(x)/x with the removable singularity handled.
inline double sinc(double x) {
    return (std::fabs(x) < 1e-6) ? 1.0 : (std::sin(x) / x);
}

} // namespace mathutil
