/* ---------------------------------------------------------------------------
 * SkillsRoutine.cpp  --  skills route.
 *
 * Poses are field inches. 0° is +Y, counterclockwise positive. The robot
 * starts at (0, 18) facing that way, and every explicit drive finishes
 * facing it too. Each of those drives is a segment path. A negative speed
 * backs up along the nose; the magnitude is the exit speed, so a near-zero
 * value stops.
 *
 *   (0, 16) then (0, 19), and that pair once more
 *   score the goal at (24, 24), drive to (24, 48), close the claw
 *   score (24, 24), drive to (48, 24), close the claw
 *   score (24, 24)
 *   match-load (60, 0), score (24, 24)
 *   score (24, 24) and the (24, 48) grab, two more times
 *   match-load (60, 0), score the goal at (0, 72)
 *
 * score() and matchLoad() are the same macros as the driver buttons.
 * Pneumatic B low closes the claw.
 * ------------------------------------------------------------------------- */

#include "auton/SkillsRoutine.h"
#include "auton/Rotation2d.h"
#include "core/Types.h"
#include "logic/MatchLoader.h"

namespace auton {

Routine buildSkillsRoutine(const IFeedForward* model) {
    // Sign selects reverse. The speed itself is a stop.

    return Routine(model, {
        DriveTo(poseInches(0, 16, 0, kReverseStopMps), togglePath),
        DriveTo(poseInches(0, 16, 0, kReverseStopMps), togglePath),

        mech.score(24, 24, {}, -90),
        DriveTo(poseInches(24, 48, -45), segmentPath())
            .then(mech.pneumatics.setBCommand(MatchLoaderTune::kClawClosed)),

        mech.score(24, 24, {}, -180),
        DriveTo(poseInches(48, 24, -135), segmentPath())
            .then(mech.pneumatics.setBCommand(MatchLoaderTune::kClawClosed)),

        mech.score(24, 24),
        mech.matchLoad(60, 0),
        mech.score(24, 24),
        mech.matchLoad(60, 0),
        mech.score(24, 24),
        mech.matchLoad(60, 0),
        mech.score(24, 24),


        mech.matchLoad(60, 0),
        mech.score(0, 72),
    }).startAt(poseInches(0, 18, 0));
}

} // namespace auton
