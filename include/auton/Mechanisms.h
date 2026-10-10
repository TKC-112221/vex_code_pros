#pragma once
/* ---------------------------------------------------------------------------
 * Mechanisms  --  intake, shooter rollers, and the two pneumatics.
 *
 * Each one is a state machine. Commands call set(); they never write volts.
 * periodic() turns the states into a SubsystemCommand, and the drivebase
 * (the only motor owner) applies it on the control tick.
 *
 * These run on that same tick, not on a second task. TankDrive already writes
 * every motor from the pipeline; a background task calling move_voltage would
 * race it. The localization task remains the odometry thread.
 *
 * The stored mode is a SubsystemIntent, the same type the driver macros
 * publish. periodic() turns it into volts through SubsystemIntent::toCommand.
 * pctToMv clamps anything past ±100, so 106 and -103 land on the same
 * voltage as teleop.
 * There is no lift on this robot. Intake current is not in the sensor dict,
 * so Jammed is entered only when a command asks for it.
 * ------------------------------------------------------------------------- */

#include "auton/Command.h"
#include "core/Types.h"
#include <memory>

namespace auton {

class IntakeSubsystem {
public:
    void set(IntakeState state) { state_ = state; }
    IntakeState state() const { return state_; }

    std::unique_ptr<Command> in()     { return instant([this] { state_ = IntakeState::In; }); }
    std::unique_ptr<Command> out()    { return instant([this] { state_ = IntakeState::Out; }); }
    std::unique_ptr<Command> idle()   { return instant([this] { state_ = IntakeState::Idle; }); }
    std::unique_ptr<Command> jammed() { return instant([this] { state_ = IntakeState::Jammed; }); }

private:
    IntakeState state_ = IntakeState::Idle;
};

class ShooterSubsystem {
public:
    void set(ShooterState state) { state_ = state; }
    ShooterState state() const { return state_; }

    std::unique_ptr<Command> spin()    { return instant([this] { state_ = ShooterState::Spin; }); }
    std::unique_ptr<Command> reverse() { return instant([this] { state_ = ShooterState::Reverse; }); }
    std::unique_ptr<Command> idle()    { return instant([this] { state_ = ShooterState::Idle; }); }

private:
    ShooterState state_ = ShooterState::Idle;
};

class PneumaticSubsystem {
public:
    void setG(bool engaged) { g_ = engaged; }
    void setB(bool engaged) { b_ = engaged; }
    bool g() const { return g_; }
    bool b() const { return b_; }

    std::unique_ptr<Command> setGCommand(bool engaged) {
        return instant([this, engaged] { g_ = engaged; });
    }
    std::unique_ptr<Command> setBCommand(bool engaged) {
        return instant([this, engaged] { b_ = engaged; });
    }

private:
    bool g_ = false;
    bool b_ = false;
};

class Mechanisms {
public:
    IntakeSubsystem intake;
    ShooterSubsystem shooter;
    PneumaticSubsystem pneumatics;

    // Motors off, solenoids released. Called when a command routine starts
    // or is cancelled, so a stale state cannot outlive the routine.
    void reset() {
        intake.set(IntakeState::Idle);
        shooter.set(ShooterState::Idle);
        pneumatics.setG(false);
        pneumatics.setB(false);
    }

    // The mode autonomous is commanding, in the same shape the macros publish.
    SubsystemIntent intent() const {
        return SubsystemIntent{intake.state(), shooter.state(), pneumatics.g(), pneumatics.b()};
    }

    SubsystemCommand periodic() const { return intent().toCommand(); }
};

inline Mechanisms& robotMechanisms() {
    static Mechanisms mechanisms;
    return mechanisms;
}

} // namespace auton
