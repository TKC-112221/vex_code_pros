#include "math/TargetPlane.h"

namespace mathutil {

bool crossedTargetPlane(const Pose2d& previous, const Pose2d& current,
                        double v_lin, const Pose2d& plane) {
    if (targetPlaneAlong(previous, plane) >= 0.0) return false;
    if (targetPlaneAlong(current, plane) < 0.0) return false;
    return targetPlaneNoseSpeed(current, v_lin, plane) > 0.0;
}

} // namespace mathutil
