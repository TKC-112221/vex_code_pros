#pragma once
/* ---------------------------------------------------------------------------
 * Command groups. Sequential runs one child at a time. Parallel runs every
 * child until all of them finish. ParallelDeadline runs every child and
 * finishes when the deadline finishes and every side command has finished.
 * Once the deadline has finished, it is not executed again, so a macro can
 * keep the chassis. A side command's drive is used only when the deadline
 * has not published one.
 * ------------------------------------------------------------------------- */

#include "auton/Command.h"
#include "math/MathUtils.h"
#include <cmath>
#include <utility>
#include <vector>

namespace auton {

struct CommandSlot {
    std::unique_ptr<Command> command;
    bool started = false;
    bool finished = false;
};

class SequentialCommandGroup : public Command {
public:
    // stop_on_yield drops every command after one that gave the leg up.
    // A drive's then() uses it, so a claw close does not run when the
    // drive never arrived. The routine's own sequence leaves it off: the
    // next leg still runs.
    explicit SequentialCommandGroup(std::vector<std::unique_ptr<Command>> commands,
                                    bool stop_on_yield = false)
        : stop_on_yield_(stop_on_yield) {
        slots_.reserve(commands.size());
        for (auto& command : commands) {
            if (command) slots_.push_back(CommandSlot{std::move(command), false, false});
        }
    }

    bool execute(CommandTick& tick) override {
        while (index_ < slots_.size()) {
            CommandSlot& slot = slots_[index_];
            if (!slot.started) {
                slot.command->initialize(tick);
                slot.started = true;
            }
            if (!slot.command->execute(tick)) return false;
            const bool yielded = tick.yielded;
            slot.command->end(false);
            slot.finished = true;
            // The finished child no longer owns the chassis. The next child
            // writes a new command on this same tick, or the drive stops.
            tick.drive_active = false;
            tick.yielded = false;
            tick.motion_arrived = false;
            ++index_;
            if (stop_on_yield_ && yielded) {
                index_ = slots_.size();
                return true;
            }
        }
        return true;
    }

    void end(bool interrupted) override {
        if (index_ < slots_.size() && slots_[index_].started && !slots_[index_].finished) {
            slots_[index_].command->end(interrupted);
            slots_[index_].finished = true;
        }
    }

    int samplePath(const Pose2d& from, Pose2d* out, int cap,
                   double v_lin, double v_ang) const override {
        if (index_ >= slots_.size()) return 0;
        return slots_[index_].command->samplePath(from, out, cap, v_lin, v_ang);
    }

    // The live follower only needs the active leg. The selector draws every
    // leg, including ones that have not started.
    int appendRoute(Pose2d& from, Pose2d* out, int count, int cap) const override {
        for (const auto& slot : slots_) {
            if (!slot.command || count >= cap) break;
            count = slot.command->appendRoute(from, out, count, cap);
        }
        return count;
    }

private:
    std::vector<CommandSlot> slots_;
    std::size_t index_ = 0;
    bool stop_on_yield_ = false;
};

class ParallelCommandGroup : public Command {
public:
    explicit ParallelCommandGroup(std::vector<std::unique_ptr<Command>> commands) {
        slots_.reserve(commands.size());
        for (auto& command : commands) {
            if (command) slots_.push_back(CommandSlot{std::move(command), false, false});
        }
    }

    void initialize(const CommandTick& tick) override {
        for (auto& slot : slots_) {
            slot.command->initialize(tick);
            slot.started = true;
        }
    }

    bool execute(CommandTick& tick) override {
        if (slots_.empty()) return true;
        bool all_done = true;
        bool any_drive = false;
        ChassisCommand drive{};
        for (auto& slot : slots_) {
            if (slot.finished) continue;
            CommandTick side = tick.branch();
            const bool done = slot.command->execute(side);
            // Two motion commands in one parallel group fight. The later one
            // keeps the chassis for this tick.
            if (side.drive_active) {
                drive = side.drive;
                any_drive = true;
            }
            if (done) {
                slot.command->end(false);
                slot.finished = true;
            } else {
                all_done = false;
            }
        }
        if (any_drive) tick.setDrive(drive);
        return all_done;
    }

    void end(bool interrupted) override {
        for (auto& slot : slots_) {
            if (slot.started && !slot.finished) {
                slot.command->end(interrupted);
                slot.finished = true;
            }
        }
    }

    int samplePath(const Pose2d& from, Pose2d* out, int cap,
                   double v_lin, double v_ang) const override {
        for (const auto& slot : slots_) {
            const int n = slot.command->samplePath(from, out, cap, v_lin, v_ang);
            if (n > 0) return n;
        }
        return 0;
    }

private:
    std::vector<CommandSlot> slots_;
};

class ParallelDeadlineGroup : public Command {
public:
    ParallelDeadlineGroup(std::unique_ptr<Command> deadline,
                          std::vector<std::unique_ptr<Command>> others)
        : deadline_(std::move(deadline)) {
        others_.reserve(others.size());
        for (auto& command : others) {
            if (command) others_.push_back(CommandSlot{std::move(command), false, false});
        }
    }

    ParallelDeadlineGroup(std::unique_ptr<Command> deadline,
                          std::unique_ptr<Command> other)
        : deadline_(std::move(deadline)) {
        if (other) others_.push_back(CommandSlot{std::move(other), false, false});
    }

    void initialize(const CommandTick& tick) override {
        if (deadline_) {
            deadline_->initialize(tick);
            deadline_started_ = true;
        }
        for (auto& slot : others_) {
            slot.command->initialize(tick);
            slot.started = true;
        }
    }

    bool execute(CommandTick& tick) override {
        if (!deadline_finished_) {
            deadline_finished_ = deadline_ ? deadline_->execute(tick) : true;
            // Arrived, rather than gave up. Gates that have not armed yet
            // fire on the side tick below. A yielded drive never sets this.
            if (deadline_finished_ && !tick.yielded) arrived_ = true;
        }
        // The drive gave up. A side command waiting on more travel would
        // never finish, so the leg ends and the routine can continue.
        if (deadline_finished_ && tick.yielded) {
            for (auto& slot : others_) {
                if (slot.finished) continue;
                if (slot.started && slot.command) slot.command->end(true);
                slot.finished = true;
            }
            return true;
        }
        bool sides_running = false;
        for (auto& slot : others_) {
            if (slot.finished) continue;
            // A side command must not replace the deadline's chassis command.
            CommandTick side = tick.branch();
            side.motion_arrived = arrived_;
            const bool done = slot.command->execute(side);
            if (!tick.drive_active && side.drive_active) tick.setDrive(side.drive);
            if (done) {
                slot.command->end(false);
                slot.finished = true;
            } else {
                sides_running = true;
            }
        }
        // The move has arrived. A macro still running publishes its own drive
        // through the pipeline, and this leg stops adding the old one.
        if (deadline_finished_ && sides_running) tick.drive_active = false;
        return deadline_finished_ && !sides_running;
    }

    void end(bool interrupted) override {
        if (deadline_started_ && deadline_ && !deadline_ended_) {
            deadline_->end(interrupted);
            deadline_ended_ = true;
        }
        for (auto& slot : others_) {
            if (slot.started && !slot.finished) {
                slot.command->end(true);
                slot.finished = true;
            }
        }
    }

    int samplePath(const Pose2d& from, Pose2d* out, int cap,
                   double v_lin, double v_ang) const override {
        if (!deadline_) return 0;
        return deadline_->samplePath(from, out, cap, v_lin, v_ang);
    }

    int appendRoute(Pose2d& from, Pose2d* out, int count, int cap) const override {
        if (!deadline_) return count;
        return deadline_->appendRoute(from, out, count, cap);
    }

private:
    std::unique_ptr<Command> deadline_;
    std::vector<CommandSlot> others_;
    bool deadline_started_ = false;
    bool deadline_ended_ = false;
    bool deadline_finished_ = false;
    bool arrived_ = false;
};

// Waits until the robot has traveled distance_in from where this command
// started, then runs action. Travel is the sum of odometry steps, so a
// curved path counts the ground it covered. A voltage sag does not change it.
class AfterTravel : public Command {
public:
    AfterTravel(double distance_in, std::unique_ptr<Command> action)
        : distance_in_(distance_in > 0.0 ? distance_in : 0.0), action_(std::move(action)) {}

    void initialize(const CommandTick& tick) override {
        prev_ = tick.state.pose;
        traveled_ = 0.0;
        armed_ = false;
        action_done_ = false;
        action_ended_ = false;
    }

    bool execute(CommandTick& tick) override {
        const Pose2d& pose = tick.state.pose;
        traveled_ += std::hypot(pose.x - prev_.x, pose.y - prev_.y);
        prev_ = pose;
        if (!armed_) {
            // The move already arrived. More odometry will not show up,
            // because that drive has stopped publishing.
            if (traveled_ + 1e-9 < distance_in_ && !tick.motion_arrived) return false;
            if (action_) action_->initialize(tick);
            armed_ = true;
        }
        if (!action_ || action_done_) return true;
        action_done_ = action_->execute(tick);
        if (action_done_) {
            action_->end(false);
            action_ended_ = true;
        }
        return action_done_;
    }

    void end(bool interrupted) override {
        if (armed_ && action_ && !action_ended_) {
            action_->end(interrupted);
            action_ended_ = true;
        }
    }

private:
    double distance_in_ = 0.0;
    std::unique_ptr<Command> action_;
    Pose2d prev_{};
    double traveled_ = 0.0;
    bool armed_ = false;
    bool action_done_ = false;
    bool action_ended_ = false;
};

// Waits until the robot has swept angle_rad from the heading where this
// command started, then runs action. The sweep is the sum of absolute yaw
// steps, so a reversal counts too. A full turn is larger than pi.
class AfterTurn : public Command {
public:
    AfterTurn(double angle_rad, std::unique_ptr<Command> action)
        : angle_rad_(angle_rad > 0.0 ? angle_rad : 0.0), action_(std::move(action)) {}

    void initialize(const CommandTick& tick) override {
        prev_ = tick.state.pose.theta;
        turned_ = 0.0;
        armed_ = false;
        action_done_ = false;
        action_ended_ = false;
    }

    bool execute(CommandTick& tick) override {
        const double now = tick.state.pose.theta;
        turned_ += std::fabs(mathutil::normalize_angle(now - prev_));
        prev_ = now;
        if (!armed_) {
            // The pivot already finished. The heading will not keep sweeping.
            if (turned_ + 1e-9 < angle_rad_ && !tick.motion_arrived) return false;
            if (action_) action_->initialize(tick);
            armed_ = true;
        }
        if (!action_ || action_done_) return true;
        action_done_ = action_->execute(tick);
        if (action_done_) {
            action_->end(false);
            action_ended_ = true;
        }
        return action_done_;
    }

    void end(bool interrupted) override {
        if (armed_ && action_ && !action_ended_) {
            action_->end(interrupted);
            action_ended_ = true;
        }
    }

private:
    double angle_rad_ = 0.0;
    std::unique_ptr<Command> action_;
    double prev_ = 0.0;
    double turned_ = 0.0;
    bool armed_ = false;
    bool action_done_ = false;
    bool action_ended_ = false;
};

inline std::unique_ptr<Command> sequence(std::vector<std::unique_ptr<Command>> commands) {
    return std::make_unique<SequentialCommandGroup>(std::move(commands));
}

inline std::unique_ptr<Command> parallel(std::unique_ptr<Command> a, std::unique_ptr<Command> b) {
    std::vector<std::unique_ptr<Command>> commands;
    commands.push_back(std::move(a));
    commands.push_back(std::move(b));
    return std::make_unique<ParallelCommandGroup>(std::move(commands));
}

inline std::unique_ptr<Command> deadline(std::unique_ptr<Command> leader,
                                        std::unique_ptr<Command> other) {
    return std::make_unique<ParallelDeadlineGroup>(std::move(leader), std::move(other));
}

} // namespace auton
