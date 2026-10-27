/* ---------------------------------------------------------------------------
 * MatchRoutines.cpp  --  red and blue routes for the selector.
 *
 * Poses are field inches, heading in degrees, nose speed in m/s. The origin
 * is the center of the bottom edge. 0° is +X, counterclockwise positive.
 * Each builder is the motion only. The start pose lives on the catalog row,
 * because the follower plans from the robot's pose when the routine starts.
 * ------------------------------------------------------------------------- */

#include "auton/MatchRoutines.h"
#include "auton/Rotation2d.h"

namespace auton {

// Right side, low: (56, 20) facing up, through (48, 48), then left to (16, 48).
Routine buildRedLow(const IFeedForward* model) {
    return Routine(model, {
        DriveTo(poseInches(48, 48, 90)),
        TurnTo(Rotation2d::fromDegrees(180)),
        DriveTo(poseInches(16, 48, 180)),
    });
}

// Right side, high: (56, 124) facing left, through (24, 120), then down.
Routine buildRedHigh(const IFeedForward* model) {
    return Routine(model, {
        DriveTo(poseInches(24, 120, 180)),
        TurnTo(Rotation2d::fromDegrees(270)),
        DriveTo(poseInches(24, 88, 270)),
    });
}

// Right side, center: (40, 18) facing up, to (12, 72), then right to (40, 72).
Routine buildRedCenter(const IFeedForward* model) {
    return Routine(model, {
        DriveTo(poseInches(12, 72, 90)),
        TurnTo(Rotation2d::fromDegrees(0)),
        DriveTo(poseInches(40, 72, 0)),
    });
}

// Left side, low: (-56, 20) facing up, through (-48, 48), then right.
Routine buildBlueLow(const IFeedForward* model) {
    return Routine(model, {
        DriveTo(poseInches(-48, 48, 90)),
        TurnTo(Rotation2d::fromDegrees(0)),
        DriveTo(poseInches(-16, 48, 0)),
    });
}

// Left side, high: (-56, 124) facing right, through (-24, 120), then down.
Routine buildBlueHigh(const IFeedForward* model) {
    return Routine(model, {
        DriveTo(poseInches(-24, 120, 0)),
        TurnTo(Rotation2d::fromDegrees(270)),
        DriveTo(poseInches(-24, 88, 270)),
    });
}

// Left side, center: (-40, 18) facing up, to (-12, 72), then left.
Routine buildBlueCenter(const IFeedForward* model) {
    return Routine(model, {
        DriveTo(poseInches(-12, 72, 90)),
        TurnTo(Rotation2d::fromDegrees(180)),
        DriveTo(poseInches(-40, 72, 180)),
    });
}

} // namespace auton
