#pragma once
/* ---------------------------------------------------------------------------
 * Mechanisms  --  intake, cascade, and the two pneumatics.
 *
 * Each one is a state machine. Commands call set(); they never write volts.
 * periodic() turns the states into a SubsystemCommand, and the drivebase
 * (the only motor owner) applies it on the control tick. The scoring macro
 * reads the cascade current itself and replaces that command while it runs.
 *
 * These run on that same tick, not on a second task. TankDrive already writes
 * every motor from the pipeline; a background task calling move_voltage would
 * race it. The localization task remains the odometry thread.
 *
 * MAIN1 and MAIN2 are one differential. Cascade voltage spins them the same
 * way. Intake voltage spins them opposite ways. Pneumatic B is the claw.
 * ------------------------------------------------------------------------- */

#include "auton/Command.h"
#include "logic/ScoreMacro.h"
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

class CascadeSubsystem {
public:
    void set(CascadeState state) { state_ = state; }
    CascadeState state() const { return state_; }
    void requestScore(bool on) { score_ = on; }
    bool scoreRequested() const { return score_; }

    std::unique_ptr<Command> up()   { return instant([this] { state_ = CascadeState::Up; score_ = false; }); }
    std::unique_ptr<Command> down() { return instant([this] { state_ = CascadeState::Down; score_ = false; }); }
    std::unique_ptr<Command> idle() { return instant([this] { state_ = CascadeState::Idle; score_ = false; }); }

private:
    CascadeState state_ = CascadeState::Idle;
    bool score_ = false;
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
    CascadeSubsystem cascade;
    PneumaticSubsystem pneumatics;

    // Motors off, solenoids released. Called when a command routine starts
    // or is cancelled, so a stale state cannot outlive the routine.
    void reset() {
        intake.set(IntakeState::Idle);
        cascade.set(CascadeState::Idle);
        cascade.requestScore(false);
        pneumatics.setG(false);
        pneumatics.setB(false);
        cancelScoreMacro();
    }

    // The mode autonomous is commanding, in the same shape the macros publish.
    SubsystemIntent intent() const {
        SubsystemIntent intent{intake.state(), cascade.state(), pneumatics.g(), pneumatics.b()};
        intent.score = cascade.scoreRequested();
        return intent;
    }

    SubsystemCommand periodic() const { return intent().toCommand(); }

    // Raise, seat the piece, open the claw, and return the cascade to the bottom.
    std::unique_ptr<Command> score();
};

class ScoreMacroCommand : public Command {
public:
    explicit ScoreMacroCommand(Mechanisms& mech) : mech_(mech) {}

    void initialize(const CommandTick& tick) override {
        (void)tick;
        mech_.intake.set(IntakeState::Idle);
        mech_.cascade.set(CascadeState::Idle);
    }

    bool execute(CommandTick& tick) override {
        (void)tick;
        mech_.cascade.requestScore(true);
        if (scoreMacroReleasedClaw()) mech_.pneumatics.setB(ScoreTune::kClawReleased);
        if (!saw_run_) {
            if (scoreMacroRunning()) saw_run_ = true;
            return false;
        }
        if (!scoreMacroRunning()) {
            mech_.cascade.requestScore(false);
            mech_.cascade.set(CascadeState::Idle);
            if (scoreMacroReleasedClaw()) mech_.pneumatics.setB(ScoreTune::kClawReleased);
            return true;
        }
        return false;
    }

    void end(bool interrupted) override {
        mech_.cascade.requestScore(false);
        if (interrupted) cancelScoreMacro();
    }

private:
    Mechanisms& mech_;
    bool saw_run_ = false;
};

inline std::unique_ptr<Command> Mechanisms::score() {
    return std::make_unique<ScoreMacroCommand>(*this);
}

inline Mechanisms& robotMechanisms() {
    static Mechanisms mechanisms;
    return mechanisms;
}

} // namespace auton
