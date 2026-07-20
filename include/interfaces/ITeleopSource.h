#pragma once
/* ---------------------------------------------------------------------------
 * ITeleopSource  --  produces a driver-controlled motion intent each tick.
 *
 * The mother interface for teleop drive schemes. TeleopTargetGenerator (ghost
 * robot + face/pivot) is the default; an alternative (e.g. plain arcade, or a
 * fully holonomic field-oriented scheme) implements this identical contract and
 * drops into main.cpp's wiring with no pipeline change.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"

// Result of one teleop tick: either a direct command (e.g. pivot) or a target.
struct TeleopDrive {
    bool direct = false;          // true -> use `command`; false -> use `target`
    ChassisCommand command;
    MotionTarget target;
    double curved_forward = 0;    // shaped stick values (for logging)
    double curved_turn = 0;
};

class ITeleopSource {
public:
    virtual ~ITeleopSource() = default;

    // Snap internal state to reality (calibrate, record start, override handoff).
    virtual void resetToPose(const Pose2d& pose) = 0;

    virtual TeleopDrive generate(RobotMode mode, const JoystickData& joy, const ButtonState& btn,
                                 const Pose2d& pose, const ChassisObservation& obs) = 0;
};
