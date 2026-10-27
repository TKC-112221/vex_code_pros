#pragma once
/* ---------------------------------------------------------------------------
 * DriveTo  --  one autonomous leg and the decorators attached to it.
 *
 *   DriveTo(poseInches(24, 48, 0, 1.0))
 *       .afterTravel(inches(12), robotMechanisms().intake.in())
 *       .then(robotMechanisms().cascade.idle())
 *
 * The leg is a DriveToPose. segmentPath() uses straight segments instead,
 * and keep-outs on the constraints are circles the polyline stays outside.
 * turnOnGoal() still plans the swing as an orbit about the goal the front
 * is on, even when segmentPath() is also set: the heading in the pose is
 * where the nose finishes, and the command presses into the point while
 * it yaws.
 *
 *   DriveTo(poseInches(24, 48, 90), turnOnGoal())
 *
 * headingInput() feeds the follower travel tangents at regular stations
 * instead of curvature and its arc-length derivatives. The default keeps
 * the curvature input.
 *
 *   DriveTo(poseInches(24, 48, 0, 1.0), headingInput())
 *
 *   DriveTo(poseInches(24, 48, 0), segmentPath())
 *
 * afterTravel() is a subcommand: the move keeps going, and the action starts
 * once this leg has covered that much odometry.
 * The move is the deadline, so a still-running action is cancelled when the
 * pose is reached and cannot hold the chassis. then() is also a subcommand,
 * and it runs after the move finishes, before the routine starts the next
 * DriveTo. Several of either decorator stack, in the order they were added.
 * ------------------------------------------------------------------------- */

#include "auton/CommandGroups.h"
#include "auton/DriveSegments.h"
#include "auton/DriveToPose.h"
#include <memory>
#include <utility>
#include <vector>

namespace auton {

class Routine;

class DriveTo : public Command {
public:
    explicit DriveTo(MotionPose target, PathConstraints path = {},
                     const IFeedForward* model = nullptr)
        : target_(target), path_(path), model_(model) {}

    DriveTo(const DriveTo&) = delete;
    DriveTo& operator=(const DriveTo&) = delete;
    DriveTo(DriveTo&&) = default;
    DriveTo& operator=(DriveTo&&) = default;

    DriveTo& afterTravel(double distance_in, std::unique_ptr<Command> action) & {
        pushDuring(distance_in, std::move(action));
        return *this;
    }

    DriveTo afterTravel(double distance_in, std::unique_ptr<Command> action) && {
        pushDuring(distance_in, std::move(action));
        return std::move(*this);
    }

    DriveTo& then(std::unique_ptr<Command> command) & {
        pushAfter(std::move(command));
        return *this;
    }

    DriveTo then(std::unique_ptr<Command> command) && {
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

    // Fold decorators into one command: travel gates run beside the move,
    // and then() commands run after that group finishes.
    void compose() {
        if (composed_) return;
        // An orbit is not a polyline. turn_on_goal keeps the curve follower.
        std::unique_ptr<Command> motion;
        if (path_.line_segments && !path_.turn_on_goal)
            motion = std::make_unique<DriveSegments>(target_, path_, model_);
        else
            motion = std::make_unique<DriveToPose>(target_, path_, model_);
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
            motion = std::make_unique<SequentialCommandGroup>(std::move(steps));
        }
        impl_ = std::move(motion);
        composed_ = true;
    }

    void pushDuring(double distance_in, std::unique_ptr<Command> action) {
        if (!action || composed_) return;
        during_.push_back(std::make_unique<AfterTravel>(distance_in, std::move(action)));
    }

    void pushAfter(std::unique_ptr<Command> command) {
        if (!command || composed_) return;
        after_.push_back(std::move(command));
    }

    MotionPose target_;
    PathConstraints path_;
    const IFeedForward* model_ = nullptr;
    std::vector<std::unique_ptr<Command>> during_;
    std::vector<std::unique_ptr<Command>> after_;
    std::unique_ptr<Command> impl_;
    bool composed_ = false;
};

} // namespace auton
