/* ---------------------------------------------------------------------------
 * SkillsRoutine.cpp  --  example skills route.
 *
 * Poses are MotionPose values: field inches, heading in degrees, and speed
 * in m/s along the nose as the fourth argument. 0° is +X, counterclockwise
 * positive. The robot does not start from the first pose; DriftFollower
 * plans from wherever localization says it is. This example assumes that
 * pose is about (0, 0) facing +X, and it stays near the middle of a 12 ft field.
 *
 *   (0, 0) --fly-by--> (36, 0) facing 0° --handoff--> (36, 36) facing 90°
 *          --stop--> (12, 36) facing 180°
 *
 * Intake comes on 12 inches into the first leg. The shooter comes on 18
 * inches into the second. Both stay in that state until the last pose.
 * ------------------------------------------------------------------------- */

#include "auton/SkillsRoutine.h"
#include "auton/Mechanisms.h"
#include "auton/Rotation2d.h"
#include "auton/TurnTo.h"

namespace auton {

Routine buildSkillsRoutine(const IFeedForward* model) {
    Mechanisms& mech = robotMechanisms();

    PathConstraints handoff;
    handoff.handoff_in = inches(6);

    return Routine(model, {
        DriveTo(poseInches(30, 0, 0, 0.0))
            .afterTravel(inches(0.5), mech.intake.in()),
        TurnTo(Rotation2d::fromDegrees(90)),
        DriveTo(poseInches(30, 50, 90, 0.0), handoff)
            .afterTravel(inches(1), mech.shooter.spin()),
        TurnTo(Rotation2d::fromDegrees(180)),
        DriveTo(poseInches(6, 50, 180))
            .then(mech.intake.idle())
            .then(mech.shooter.idle())
            .then(mech.pneumatics.setGCommand(true)),
    });
}

} // namespace auton
