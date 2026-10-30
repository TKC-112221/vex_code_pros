/* ---------------------------------------------------------------------------
 * MatchRoutines.cpp  --  match routes for the selector.
 *
 * Poses are field inches, heading in degrees, nose speed in m/s. The origin
 * is the center of the bottom edge. 0° is +Y, counterclockwise positive.
 * These poses are the red side, positive x. Selecting blue mirrors x and
 * heading across the field centerline. startAt() is where that route begins.
 * The run places the robot there before the first leg, and the field drawing
 * starts there too.
 * ------------------------------------------------------------------------- */

#include "auton/MatchRoutines.h"
#include "auton/Rotation2d.h"

namespace auton {

Routine buildLeftSide2(const IFeedForward* model) {
    return Routine(model, {
        DriveTo(poseInches(0, 16, 0, kReverseStopMps), togglePath),
        DriveTo(poseInches(0, 16, 0, kReverseStopMps), togglePath),
        mech.score(24, 24, {}, -90),
        DriveTo(poseInches(24, 48, -45), segmentPath())
            .then(mech.pneumatics.setBCommand(MatchLoaderTune::kClawClosed)),
        mech.score(24, 24, {}, -180),
        DriveTo(poseInches(48, 24, -135), segmentPath())
            .then(mech.pneumatics.setBCommand(MatchLoaderTune::kClawClosed)),
        mech.score(-24, 24, {}, 90),
        DriveTo(poseInches(-48, 24, 45), segmentPath()),
    }).startAt(poseInches(0, 18, 0));
}

Routine buildRightSide2(const IFeedForward* model) {
    return Routine(model, {
        DriveTo(poseInches(72-20, 72, 90, kReverseStopMps), togglePath),
        DriveTo(poseInches(72-20, 72, 90, kReverseStopMps), togglePath),
        mech.score(48, 48, {}, -180),
        DriveTo(poseInches(24, 48, 135), segmentPath())
            .then(mech.pneumatics.setBCommand(MatchLoaderTune::kClawClosed)),
        mech.score(48, 48, {}, -90),
        DriveTo(poseInches(48, 24, -135), segmentPath())
            .then(mech.pneumatics.setBCommand(MatchLoaderTune::kClawClosed)),
        mech.score(48, 48, {}, 0),
        DriveTo(poseInches(60, 12, 180), segmentPath()),
    }).startAt(poseInches(72-18, 72, 90));
}

} // namespace auton
