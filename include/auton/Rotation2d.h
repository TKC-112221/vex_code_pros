#pragma once
/* ---------------------------------------------------------------------------
 * Rotation2d  --  a field heading for the autonomous layer.
 *
 * Pose2d remains the pipeline's pose (x, y, theta in radians). This type is
 * the heading half of that pose, so a routine can add and subtract directions
 * without touching the estimator. 0 is +Y and CCW is positive, matching Pose2d.
 * ------------------------------------------------------------------------- */

#include "core/Specs.h"
#include "core/Types.h"
#include "math/MathUtils.h"
#include <cmath>

namespace auton {

class Rotation2d {
public:
    Rotation2d() = default;

    static Rotation2d fromRadians(double radians) {
        return Rotation2d(mathutil::normalize_angle(radians));
    }
    static Rotation2d fromDegrees(double degrees) {
        return fromRadians(degrees * (Robot::PI / 180.0));
    }

    double radians() const { return rad_; }
    double degrees() const { return rad_ * (180.0 / Robot::PI); }

    Rotation2d operator+(Rotation2d rhs) const { return fromRadians(rad_ + rhs.rad_); }
    Rotation2d operator-(Rotation2d rhs) const { return fromRadians(rad_ - rhs.rad_); }
    Rotation2d operator-() const { return fromRadians(-rad_); }

private:
    explicit Rotation2d(double radians) : rad_(radians) {}
    double rad_ = 0.0;
};

inline Pose2d pose(double x_in, double y_in, Rotation2d heading) {
    return Pose2d{x_in, y_in, heading.radians()};
}

// Field position is inches, the same unit localization publishes.
// Heading stays radians. degrees() is the conversion a routine uses when it
// writes a sweep in degrees. v_mps is speed along the nose; negative v_mps
// is a reverse arrival.
inline double inches(double value_in) { return value_in; }

inline double degrees(double value_deg) { return value_deg * (Robot::PI / 180.0); }

inline MotionPose poseInches(double x_in, double y_in, double heading_deg, double v_mps = 0.0) {
    return MotionPose{inches(x_in), inches(y_in),
                      Rotation2d::fromDegrees(heading_deg).radians(), v_mps, 0.0};
}

inline double distanceInches(const Pose2d& a, const Pose2d& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

} // namespace auton
