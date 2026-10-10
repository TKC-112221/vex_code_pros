#pragma once
/* ---------------------------------------------------------------------------
 * CommandScheduler  --  the autonomous queue.
 *
 * The pipeline calls run() once per 10 ms tick with the localization
 * snapshot. run() executes the front command and pops it when it finishes.
 * Commands that finish on the tick they start (an instant, a settled drive
 * handing off to the next drive) are chained on that same tick, so a
 * mechanism action does not cost a loop and a fly-by does not stop.
 *
 * The scheduler does not own motors. It returns the chassis command the
 * active motion wrote, and the pipeline applies it through IDriveBase.
 * ------------------------------------------------------------------------- */

#include "auton/Command.h"
#include "pros/rtos.hpp"
#include <queue>
#include <utility>

namespace auton {

struct SchedulerOutput {
    bool running = false;
    bool has_drive = false;
    ChassisCommand drive{};
};

class CommandScheduler {
public:
    void add(std::unique_ptr<Command> command) {
        const Lock lock(mutex_);
        if (command) queue_.push(std::move(command));
    }

    bool idle() const {
        const Lock lock(mutex_);
        return queue_.empty();
    }

    SchedulerOutput run(const StateEstimate& state) {
        const Lock lock(mutex_);
        SchedulerOutput out;
        CommandTick tick{state};
        while (!queue_.empty()) {
            Command& command = *queue_.front();
            if (!front_started_) {
                command.initialize(tick);
                front_started_ = true;
            }
            const bool done = command.execute(tick);
            if (tick.drive_active) {
                out.drive = tick.drive;
                out.has_drive = true;
            }
            if (!done) {
                out.running = true;
                return out;
            }
            command.end(false);
            queue_.pop();
            front_started_ = false;
            tick.drive_active = false;
            out.has_drive = false;
        }
        return out;
    }

    void cancel() {
        const Lock lock(mutex_);
        if (front_started_ && !queue_.empty()) queue_.front()->end(true);
        while (!queue_.empty()) queue_.pop();
        front_started_ = false;
    }

    int samplePath(const Pose2d& from, Pose2d* out, int cap,
                   double v_lin, double v_ang) const {
        const Lock lock(mutex_);
        if (queue_.empty()) return 0;
        return queue_.front()->samplePath(from, out, cap, v_lin, v_ang);
    }

private:
    class Lock {
    public:
        explicit Lock(pros::Mutex& mutex) : mutex_(mutex) { mutex_.take(); }
        ~Lock() { mutex_.give(); }
        Lock(const Lock&) = delete;
        Lock& operator=(const Lock&) = delete;
    private:
        pros::Mutex& mutex_;
    };

    mutable pros::Mutex mutex_;
    std::queue<std::unique_ptr<Command>> queue_;
    bool front_started_ = false;
};

} // namespace auton
