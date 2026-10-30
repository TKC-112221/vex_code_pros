#pragma once
/* ---------------------------------------------------------------------------
 * Mechanisms  --  intake, cascade, and the two pneumatics.
 *
 * Each one is a state machine. Commands call set(); they never write volts.
 * periodic() turns the states into a SubsystemCommand, and the drivebase
 * (the only motor owner) applies it on the control tick. The scoring macro
 * GoalScore replaces that command, and the chassis command, while it runs.
 *
 * These run on that same tick, not on a second task. TankDrive already writes
 * every motor from the pipeline; a background task calling move_voltage would
 * race it. The localization task remains the odometry thread.
 *
 * score(), matchLoad(), and run() start a chassis macro and wait until it
 * finishes. A score or match load whose drive cannot reach its target
 * gives up: the macro is cancelled and the routine starts the next
 * command. Contact at that target is the macro's own problem; it finishes
 * the action or times the phase out. A request that never starts the
 * macro gives up as well. The pipeline runs that macro on the same list
 * as the driver buttons. While the macro is driving, the routine's
 * current leg pauses.
 *
 * MAIN1 and MAIN2 are one differential. Cascade voltage spins them the same
 * way. Intake voltage spins them opposite ways. Pneumatic B is the claw.
 * ------------------------------------------------------------------------- */

#include "auton/Command.h"
#include "auton/Rotation2d.h"
#include "logic/GoalScore.h"
#include "logic/MatchLoader.h"
#include "logic/RobotMacro.h"
#include "core/Types.h"
#include <cstdint>
#include <optional>
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
        macro_bits_ = 0;
        cancelRobotMacros();
    }

    void holdMacro(uint8_t bit, bool on) {
        if (on) macro_bits_ = static_cast<uint8_t>(macro_bits_ | bit);
        else macro_bits_ = static_cast<uint8_t>(macro_bits_ & static_cast<uint8_t>(~bit));
    }

    // The mode autonomous is commanding, in the same shape the macros publish.
    SubsystemIntent intent() const {
        SubsystemIntent intent{intake.state(), cascade.state(), pneumatics.g(), pneumatics.b()};
        intent.macro_request = macro_bits_;
        intent.score = cascade.scoreRequested() || (macro_bits_ & kMacroScore) != 0;
        return intent;
    }

    SubsystemCommand periodic() const { return intent().toCommand(); }

    // Hold `bit` until that macro has started and then finished. Releasing the
    // bit does not cancel a macro that is already running. Aborting the
    // routine does: reset() cancels every registered macro.
    std::unique_ptr<Command> run(uint8_t bit, bool (*running)());

    // Drive to a goal, raise the cascade, and score. With no point, a goal
    // within 30 degrees of the heading counts as half its distance. A point
    // scores the goal closest to that field position, in inches.
    // approach_deg is the field heading of the drive in, 0 along +Y. Pass
    // {} for no approach and the robot comes from the bumper's side.
    // detach_deg is the heading the nose rotates to while scoring, and the
    // backup leaves on it. With no detach heading, the robot leaves facing
    // the latch.
    std::unique_ptr<Command> score();
    std::unique_ptr<Command> score(double x_in, double y_in);
    std::unique_ptr<Command> score(double x_in, double y_in, double approach_deg);
    std::unique_ptr<Command> score(double x_in, double y_in,
                                   std::optional<double> approach_deg, double detach_deg);
    std::unique_ptr<Command> score(Rotation2d approach);
    std::unique_ptr<Command> score(Rotation2d approach, Rotation2d detach);

    // Drive the front bumper onto a match loader and close the claw. With no
    // point, a loader within 30 degrees of the heading counts as half its
    // distance. A point drives to the loader closest to that field position,
    // in inches. The approach is always 180 degrees, facing the near wall.
    std::unique_ptr<Command> matchLoad();
    std::unique_ptr<Command> matchLoad(double x_in, double y_in);

private:
    uint8_t macro_bits_ = 0;
};

class HoldMacroCommand : public Command {
public:
    HoldMacroCommand(Mechanisms& mech, uint8_t bit, bool (*running)(), bool idle_on_start,
                     bool has_aim = false, double aim_x = 0, double aim_y = 0,
                     bool has_approach = false, double approach_deg = 0,
                     bool has_detach = false, double detach_deg = 0)
        : mech_(mech), bit_(bit), running_(running), idle_on_start_(idle_on_start),
          has_aim_(has_aim), aim_x_(aim_x), aim_y_(aim_y),
          has_approach_(has_approach), approach_deg_(approach_deg),
          has_detach_(has_detach), detach_deg_(detach_deg) {}

    void initialize(const CommandTick& tick) override {
        (void)tick;
        if (bit_ == kMacroScore) {
            // The previous score left the claw released. This one stays as
            // commanded until its own latch has finished and it backs up.
            goalScoreClearReleased();
            if (has_aim_) goalScoreAim(aim_x_, aim_y_);
            else goalScoreClearAim();
            if (has_approach_) goalScoreApproach(degrees(approach_deg_));
            else goalScoreClearApproach();
            if (has_detach_) goalScoreDetach(degrees(detach_deg_));
            else goalScoreClearDetach();
        } else if (bit_ == kMacroMatchLoad) {
            if (has_aim_) matchLoaderAim(aim_x_, aim_y_);
            else matchLoaderClearAim();
        }
        if (!idle_on_start_) return;
        mech_.intake.set(IntakeState::Idle);
        mech_.cascade.set(CascadeState::Idle);
    }

    // The score is not a DriveTo. Draw the drive in, the swing onto the
    // detach heading, and the retreat. The next leg starts at the bumper
    // after the 6 inch backup. The drawn samples are rotation centres;
    // `from` stays the bumper.
    int appendRoute(Pose2d& from, Pose2d* out, int count, int cap) const override {
        if (bit_ != kMacroScore) return Command::appendRoute(from, out, count, cap);
        if (out == nullptr || cap < 2 || count >= cap) return count;
        Pose2d buf[64];
        Pose2d leave = from;
        const int n = previewGoalScorePath(from, has_aim_, aim_x_, aim_y_,
                                           has_approach_, degrees(approach_deg_),
                                           has_detach_, degrees(detach_deg_),
                                           buf, 64, &leave);
        if (n >= 2) {
            const int begin = (count > 0) ? 1 : 0;
            for (int i = begin; i < n && count < cap; ++i) out[count++] = buf[i];
        }
        from = leave;
        return count;
    }

    bool execute(CommandTick& tick) override {
        (void)tick;
        mech_.holdMacro(bit_, true);
        // Set only after this score has released, which is once the latch
        // drive has finished and before the backup starts.
        if (bit_ == kMacroScore && goalScoreReleasedClaw())
            mech_.pneumatics.setB(GoalScoreTune::kClawReleased);
        const bool live = running_ != nullptr && running_();
        if (!saw_run_) {
            if (live) saw_run_ = true;
            else {
                // The request bit is set on this tick. The macro starts in
                // the same pipeline, one step later. If it is still idle
                // after that, no goal or loader was found.
                ++wait_ticks_;
                if (wait_ticks_ < 25) return false;
                mech_.holdMacro(bit_, false);
                if (bit_ == kMacroScore) cancelGoalScore();
                else if (bit_ == kMacroMatchLoad) cancelMatchLoader();
                return true;
            }
        }
        // The macro gave the drive up and is still marked running. Cancel it
        // so the next command can take the drivetrain.
        const bool wedged = live && driveStuck();
        if (wedged) {
            if (bit_ == kMacroScore) cancelGoalScore();
            else if (bit_ == kMacroMatchLoad) cancelMatchLoader();
        }
        if (!live || wedged) {
            mech_.holdMacro(bit_, false);
            if (bit_ == kMacroScore) {
                mech_.cascade.requestScore(false);
                mech_.cascade.set(CascadeState::Idle);
                if (goalScoreReleasedClaw()) mech_.pneumatics.setB(GoalScoreTune::kClawReleased);
            }
            return true;
        }
        return false;
    }

    // The request bit drops with the command. A macro that already started
    // keeps running, including after this drive leg ends. reset() is what
    // cancels it, and that runs when the routine is aborted.
    void end(bool interrupted) override {
        (void)interrupted;
        mech_.holdMacro(bit_, false);
    }

private:
    Mechanisms& mech_;
    uint8_t bit_ = 0;
    bool (*running_)() = nullptr;
    bool idle_on_start_ = false;
    bool has_aim_ = false;
    double aim_x_ = 0;
    double aim_y_ = 0;
    bool has_approach_ = false;
    double approach_deg_ = 0;
    bool has_detach_ = false;
    double detach_deg_ = 0;
    bool saw_run_ = false;
    int wait_ticks_ = 0;

    bool driveStuck() const {
        if (bit_ == kMacroScore) return goalScoreDriveStuck();
        if (bit_ == kMacroMatchLoad) return matchLoaderDriveStuck();
        return false;
    }
};

inline std::unique_ptr<Command> Mechanisms::run(uint8_t bit, bool (*running)()) {
    return std::make_unique<HoldMacroCommand>(*this, bit, running, false);
}

inline std::unique_ptr<Command> Mechanisms::score() {
    return std::make_unique<HoldMacroCommand>(*this, kMacroScore, goalScoreRunning, true);
}

inline std::unique_ptr<Command> Mechanisms::score(double x_in, double y_in) {
    return std::make_unique<HoldMacroCommand>(*this, kMacroScore, goalScoreRunning, true,
                                              true, x_in, y_in);
}

inline std::unique_ptr<Command> Mechanisms::score(double x_in, double y_in, double approach_deg) {
    return std::make_unique<HoldMacroCommand>(*this, kMacroScore, goalScoreRunning, true,
                                              true, x_in, y_in, true, approach_deg);
}

inline std::unique_ptr<Command> Mechanisms::score(double x_in, double y_in,
                                                 std::optional<double> approach_deg,
                                                 double detach_deg) {
    const bool has_approach = approach_deg.has_value();
    return std::make_unique<HoldMacroCommand>(*this, kMacroScore, goalScoreRunning, true,
                                              true, x_in, y_in, has_approach,
                                              has_approach ? *approach_deg : 0.0,
                                              true, detach_deg);
}

inline std::unique_ptr<Command> Mechanisms::score(Rotation2d approach) {
    return std::make_unique<HoldMacroCommand>(*this, kMacroScore, goalScoreRunning, true,
                                              false, 0.0, 0.0, true, approach.degrees());
}

inline std::unique_ptr<Command> Mechanisms::score(Rotation2d approach, Rotation2d detach) {
    return std::make_unique<HoldMacroCommand>(*this, kMacroScore, goalScoreRunning, true,
                                              false, 0.0, 0.0, true, approach.degrees(),
                                              true, detach.degrees());
}

inline std::unique_ptr<Command> Mechanisms::matchLoad() {
    return std::make_unique<HoldMacroCommand>(*this, kMacroMatchLoad, matchLoaderRunning, false);
}

inline std::unique_ptr<Command> Mechanisms::matchLoad(double x_in, double y_in) {
    return std::make_unique<HoldMacroCommand>(*this, kMacroMatchLoad, matchLoaderRunning, false,
                                              true, x_in, y_in);
}

inline Mechanisms& robotMechanisms() {
    static Mechanisms mechanisms;
    return mechanisms;
}

} // namespace auton
