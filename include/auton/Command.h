#pragma once
/* ---------------------------------------------------------------------------
 * Command  --  one block of an autonomous routine.
 *
 * initialize() runs once when the block becomes active. execute() runs on
 * each control tick and returns true when the block is finished. end() runs
 * once, with interrupted set when a deadline or a cancel cut it short.
 *
 * A command describes an outcome. Drive commands ask DriftFollower for
 * chassis volts. Mechanism commands only change a subsystem state. None of
 * them write motor voltage themselves.
 * ------------------------------------------------------------------------- */

#include "auton/Rotation2d.h"
#include "core/Types.h"
#include <functional>
#include <memory>
#include <utility>

namespace auton {

// One scheduler tick. state is the localization snapshot. A command that
// owns the chassis this tick calls setDrive(); the scheduler applies it.
struct CommandTick {
    const StateEstimate& state;
    ChassisCommand drive{};
    bool drive_active = false;

    void setDrive(const ChassisCommand& cmd) {
        drive = cmd;
        drive_active = true;
    }
};

class Command {
public:
    virtual ~Command() = default;

    virtual void initialize(const CommandTick& tick) { (void)tick; }
    virtual bool execute(CommandTick& tick) = 0;
    virtual void end(bool interrupted) { (void)interrupted; }

    // Polyline of a motion command, for the field window. 0 means none.
    virtual int samplePath(const Pose2d& from, Pose2d* out, int cap,
                           double v_lin, double v_ang) const {
        (void)from;
        (void)out;
        (void)cap;
        (void)v_lin;
        (void)v_ang;
        return 0;
    }
};

// Runs once on the tick it starts, then finishes. Pneumatics, state changes.
class InstantCommand : public Command {
public:
    explicit InstantCommand(std::function<void()> action) : action_(std::move(action)) {}

    bool execute(CommandTick& tick) override {
        (void)tick;
        if (action_) action_();
        return true;
    }

private:
    std::function<void()> action_;
};

// execute() is the predicate. True finishes the command.
class RunCommand : public Command {
public:
    explicit RunCommand(std::function<bool(const CommandTick&)> done)
        : done_(std::move(done)) {}

    bool execute(CommandTick& tick) override { return done_ ? done_(tick) : true; }

private:
    std::function<bool(const CommandTick&)> done_;
};

inline std::unique_ptr<Command> instant(std::function<void()> action) {
    return std::make_unique<InstantCommand>(std::move(action));
}

inline std::unique_ptr<Command> runUntil(std::function<bool(const CommandTick&)> done) {
    return std::make_unique<RunCommand>(std::move(done));
}

// Finishes when the robot is inside radius_in of target. Distance, not time.
inline std::unique_ptr<Command> untilNear(Pose2d target, double radius_in) {
    return runUntil([target, radius_in](const CommandTick& tick) {
        return distanceInches(tick.state.pose, target) <= radius_in;
    });
}

} // namespace auton
