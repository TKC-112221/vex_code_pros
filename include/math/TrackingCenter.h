#pragma once
/* ---------------------------------------------------------------------------
 * TrackingCenter  --  the pose point versus the centre of rotation.
 *
 * The reported pose is the middle of the front bumper, the tracking
 * centre. The chassis yaws about the geometric centre. The offset below
 * is from that rotation centre to the tracking centre, in the robot
 * frame: forward of the rotation centre, and to its left. Turning swings
 * the tracking centre even when the rotation centre does not move, so a
 * path that servos the tracking centre yaws back and forth. Followers
 * track the rotation centre and shift a tracking command by this offset.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "core/Specs.h"
#include "math/MathUtils.h"
#include <cmath>

namespace tracking {

// Rotation centre to tracking centre, robot frame. Forward is +x along
// the nose. Left is +y. The tracking centre is the front-bumper middle.
constexpr double kRotationToTrackingForwardM = Robot::Chassis::TRACKING_FORWARD_M;
constexpr double kRotationToTrackingLeftM = 0.0;

inline void rotationAxes(double heading, double& fx, double& fy, double& lx, double& ly) {
    fx = mathutil::heading_cos(heading);
    fy = mathutil::heading_sin(heading);
    lx = -fy;
    ly = fx;
}

inline Pose2d centerFromTracking(const Pose2d& tracked) {
    double fx, fy, lx, ly;
    rotationAxes(tracked.theta, fx, fy, lx, ly);
    return Pose2d{
        tracked.x - kRotationToTrackingForwardM * fx - kRotationToTrackingLeftM * lx,
        tracked.y - kRotationToTrackingForwardM * fy - kRotationToTrackingLeftM * ly,
        tracked.theta};
}

inline Pose2d trackingFromCenter(const Pose2d& center) {
    double fx, fy, lx, ly;
    rotationAxes(center.theta, fx, fy, lx, ly);
    return Pose2d{
        center.x + kRotationToTrackingForwardM * fx + kRotationToTrackingLeftM * lx,
        center.y + kRotationToTrackingForwardM * fy + kRotationToTrackingLeftM * ly,
        center.theta};
}

// A follower pose back in the frame the command was written in.
// `travel` is the path tangent. Reverse drive points the nose the other way,
// and the offset lies along the nose.
inline Pose2d commandFromRotation(const Pose2d& rotation, double travel, double drive,
                                 CommandPoint point) {
    const double tangent = mathutil::normalize_angle(travel);
    if (point != CommandPoint::Tracking) return Pose2d{rotation.x, rotation.y, tangent};
    const double nose = (drive < 0.0)
        ? mathutil::normalize_angle(tangent + Robot::PI)
        : tangent;
    Pose2d tracked = trackingFromCenter(Pose2d{rotation.x, rotation.y, nose});
    tracked.theta = tangent;
    return tracked;
}

// Geometric center on the field center, nose along +y. The reported pose
// is the front middle of that chassis.
inline Pose2d origin() {
    return trackingFromCenter(Pose2d{Robot::Range::CENTER_X_M, Robot::Range::CENTER_Y_M, 0.0});
}

// Center travel (forward, strafe) integrated at mid_theta, plus the swing
// of the front from theta0 to theta1. x and y are the tracking point.
inline void integrate(double& x, double& y,
                      double theta0, double theta1,
                      double forward, double strafe, double mid_theta) {
    const double c = mathutil::heading_cos(mid_theta);
    const double s = mathutil::heading_sin(mid_theta);
    const double dcos = mathutil::heading_cos(theta1) - mathutil::heading_cos(theta0);
    const double dsin = mathutil::heading_sin(theta1) - mathutil::heading_sin(theta0);
    x += forward * c - strafe * s
       + kRotationToTrackingForwardM * dcos
       - kRotationToTrackingLeftM * dsin;
    y += forward * s + strafe * c
       + kRotationToTrackingForwardM * dsin
       + kRotationToTrackingLeftM * dcos;
}

} // namespace tracking
