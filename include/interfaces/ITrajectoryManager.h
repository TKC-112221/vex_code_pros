#pragma once
/* ---------------------------------------------------------------------------
 * ITrajectoryManager  --  owns recorded motion: capture, persistence, replay.
 *
 * During replay it acts as a "motion source", emitting a MotionTarget (and the
 * matching subsystem command) each tick, advancing its own playback cursor.
 * During recording it appends frames. Descendants: LogFrameReplayer.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"

class ITrajectoryManager {
public:
    virtual ~ITrajectoryManager() = default;

    // --- Persistence ---
    virtual void loadFromHeader() = 0;      // load the compiled-in recording
    virtual void saveToConsole() = 0;       // dump the buffer as CSV over stdout

    // --- Recording ---
    virtual void beginRecording(RobotMode mode, const Pose2d& startPose) = 0;
    // `command` is the chassis-frame intent (surge/sway/yaw); `applied` is the
    // post-safety chassis effort actually sent to the drivebase. Neither is
    // stick- or wheel-specific. *_error are controller diagnostics for this tick.
    virtual void recordFrame(const MotionTarget& target,
                             const Pose2d& actual,
                             const CommandVector& command,
                             const ChassisCommand& applied,
                             const SubsystemCommand& sub,
                             const ChassisObservation& obs,
                             double lin_error,
                             double ang_error) = 0;
    virtual int  getFrameCount() const = 0;

    // --- Replay (motion source) ---
    virtual void resetPlayback() = 0;
    virtual Pose2d getStartPose() const = 0;
    virtual double getStartHeading() const = 0;
    // Emit the interpolated target for this tick and advance the cursor.
    virtual MotionTarget getNextTarget(const Pose2d& pose, const ChassisObservation& obs) = 0;
    virtual SubsystemCommand getTargetSubsystems() const = 0;
    virtual bool isFinished() const = 0;
};
