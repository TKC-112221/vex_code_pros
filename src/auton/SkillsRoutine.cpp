/* ---------------------------------------------------------------------------
 * SkillsRoutine.cpp  --  example skills route.
 *
 * Poses are MotionPose values: field inches, heading in degrees, and speed
 * in m/s along the nose as the fourth argument. The origin is the center of
 * the bottom edge. 0° is +X, counterclockwise positive. The robot does not
 * start from the first pose; DriftFollower plans from wherever localization
 * says it is. Calibration leaves that pose at the field center, (0, 72),
 * facing +X, and this example stays near the middle of the 12 ft field.
 *
 *   (0, 72) --fly-by--> (30, 72) facing 0° --handoff--> (30, 122) facing 90°
 *            --stop--> (6, 122) facing 180°
 *
 * Intake comes on half an inch into the first leg and stays on until the
 * last pose. Add mech.cascade.up() or mech.score() on a leg to run the lift.
 * ------------------------------------------------------------------------- */

#include "auton/SkillsRoutine.h"
#include "auton/Mechanisms.h"
#include "auton/Rotation2d.h"
#include "auton/TurnTo.h"
#include "core/Specs.h"

namespace auton {

Routine buildSkillsRoutine(const IFeedForward* model) {
    Mechanisms& mech = robotMechanisms();

    PathConstraints handoff;
    handoff.handoff_in = inches(6);

    return Routine(model, {
        DriveTo(poseInches(30, Robot::Range::CENTER_Y_IN, 0, 0.0))
            .afterTravel(inches(0.5), mech.intake.in()),
        TurnTo(Rotation2d::fromDegrees(90)),
        DriveTo(poseInches(30, Robot::Range::CENTER_Y_IN + 50.0, 90, 0.0), handoff),
        TurnTo(Rotation2d::fromDegrees(180)),
        DriveTo(poseInches(6, Robot::Range::CENTER_Y_IN + 50.0, 180))
            .then(mech.intake.idle())
            .then(mech.pneumatics.setGCommand(true)),
    });
}

} // namespace auton
