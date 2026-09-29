#pragma once
/* ---------------------------------------------------------------------------
 * IController  --  turns (estimated pose, motion target) into a chassis-frame
 * effort command. The output is a ChassisCommand expressed in generalized
 * chassis DOFs (linear / lateral / angular) -- NEVER per-wheel voltages. The
 * DriveBase performs the final inverse kinematics.
 *
 * Descendants: CascadedController (v1's Ramsete/PID + injected feedforward).
 * ------------------------------------------------------------------------- */

#include "core/Types.h"

class IController {
public:
    virtual ~IController() = default;

    virtual void init() = 0;
    virtual void reset() = 0;

    // pose  : authoritative pose from the estimator (field frame)
    // target: desired pose/velocity/accel (+ optional recorded feedforward)
    // obs   : measured chassis velocities and sensor context for this tick
    virtual ChassisCommand computeCommand(const Pose2d& pose,
                                          const MotionTarget& target,
                                          const ChassisObservation& obs) = 0;

    // Diagnostics from the most recent computeCommand (logging / UI).
    virtual double lastSpatialError() const { return 0.0; }
    virtual double lastAngularError() const { return 0.0; }
    virtual double lastTimeWarp() const { return 1.0; }
};
