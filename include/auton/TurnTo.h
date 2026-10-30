#pragma once
/* ---------------------------------------------------------------------------
 * TurnTo  --  one autonomous pivot and the decorators attached to it.
 *
 *   TurnTo(Rotation2d::fromDegrees(90))
 *       .afterTurn(degrees(30), robotMechanisms().intake.in())
 *       .then(robotMechanisms().cascade.idle())
 *
 * The leg is a TurnToHeading. The robot yaws where it stands; it does not
 * chase a path. afterTurn() is a subcommand: the pivot keeps going, and the
 * action starts once this leg has swept that many radians of yaw. The pivot
 * is the deadline. The group keeps running while a side command is still
 * going, and once the heading has settled it stops publishing that pivot.
 * A gate that has not fired yet runs when the pivot arrives. A macro that
 * takes the chassis also pauses the pivot. then() runs after that group
 * finishes, before the routine starts the next leg. A pivot that gave up
 * skips then(). Several of either decorator stack, in the order they were
 * added.
 * ------------------------------------------------------------------------- */

#include "auton/CommandGroups.h"
#include "auton/TurnToHeading.h"
#include <memory>
#include <utility>
#include <vector>

namespace auton {

class Routine;

class TurnTo : public Command {
public:
    explicit TurnTo(Rotation2d heading, TurnConstraints turn = {},
                    const IFeedForward* model = nullptr)
        : heading_(heading), turn_(turn), model_(model) {}

    TurnTo(const TurnTo&) = delete;
    TurnTo& operator=(const TurnTo&) = delete;
    TurnTo(TurnTo&&) = default;
    TurnTo& operator=(TurnTo&&) = default;

    TurnTo& afterTurn(double angle_rad, std::unique_ptr<Command> action) & {
        pushDuring(angle_rad, std::move(action));
        return *this;
    }

    TurnTo afterTurn(double angle_rad, std::unique_ptr<Command> action) && {
        pushDuring(angle_rad, std::move(action));
        return std::move(*this);
    }

    TurnTo& then(std::unique_ptr<Command> command) & {
        pushAfter(std::move(command));
        return *this;
    }

    TurnTo then(std::unique_ptr<Command> command) && {
        pushAfter(std::move(command));
        return std::move(*this);
    }

    void initialize(const CommandTick& tick) override {
        compose();
        impl_->initialize(tick);
    }

    bool execute(CommandTick& tick) override {
        compose();
        return impl_->execute(tick);
    }

    void end(bool interrupted) override {
        if (impl_) impl_->end(interrupted);
    }

    int samplePath(const Pose2d& from, Pose2d* out, int cap,
                   double v_lin, double v_ang) const override {
        if (!impl_) return 0;
        return impl_->samplePath(from, out, cap, v_lin, v_ang);
    }

private:
    friend class Routine;

    void bindFeedforward(const IFeedForward* model) {
        if (model) model_ = model;
    }

    // Reflect this pivot across the field centerline. Call before compose().
    // A forced direction reverses with the heading.
    void mirror() {
        heading_ = -heading_;
        if (turn_.direction == TurnDirection::Clockwise)
            turn_.direction = TurnDirection::Counterclockwise;
        else if (turn_.direction == TurnDirection::Counterclockwise)
            turn_.direction = TurnDirection::Clockwise;
    }

    // Fold decorators into one command: yaw gates run beside the pivot,
    // and then() commands run after that group finishes.
    void compose() {
        if (composed_) return;
        std::unique_ptr<Command> motion =
            std::make_unique<TurnToHeading>(heading_.radians(), turn_, model_);
        if (!during_.empty()) {
            motion = std::make_unique<ParallelDeadlineGroup>(
                std::move(motion), std::move(during_));
        }
        if (!after_.empty()) {
            std::vector<std::unique_ptr<Command>> steps;
            steps.reserve(after_.size() + 1);
            steps.push_back(std::move(motion));
            for (auto& command : after_) {
                if (command) steps.push_back(std::move(command));
            }
            motion = std::make_unique<SequentialCommandGroup>(std::move(steps), true);
        }
        impl_ = std::move(motion);
        composed_ = true;
    }

    void pushDuring(double angle_rad, std::unique_ptr<Command> action) {
        if (!action || composed_) return;
        during_.push_back(std::make_unique<AfterTurn>(angle_rad, std::move(action)));
    }

    void pushAfter(std::unique_ptr<Command> command) {
        if (!command || composed_) return;
        after_.push_back(std::move(command));
    }

    Rotation2d heading_;
    TurnConstraints turn_;
    const IFeedForward* model_ = nullptr;
    std::vector<std::unique_ptr<Command>> during_;
    std::vector<std::unique_ptr<Command>> after_;
    std::unique_ptr<Command> impl_;
    bool composed_ = false;
};

} // namespace auton
