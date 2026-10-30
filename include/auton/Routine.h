#pragma once
/* ---------------------------------------------------------------------------
 * Routine  --  a list of DriveTo and TurnTo commands, run in order.
 *
 *   Routine route(feedforward, {
 *       DriveTo(poseInches(24, 48, 0, 1.0))
 *           .afterTravel(inches(12), robotMechanisms().intake.in()),
 *       TurnTo(Rotation2d::fromDegrees(90))
 *           .afterTurn(degrees(30), robotMechanisms().intake.out()),
 *       DriveTo(poseInches(48, 48, 90))
 *           .then(robotMechanisms().cascade.up()),
 *   }).startAt(poseInches(0, 72, 0));
 *   scheduler.add(route.build());
 *
 * Each leg carries its own subcommands. A mech.score(), mech.matchLoad(),
 * or mech.run(bit, running) can be its own leg, or a then() / afterTravel()
 * on a drive. build() sequences the legs, so the next one starts on the
 * same tick the previous one finishes and a non-zero exit speed is still
 * on the robot. The routine binds its feedforward model onto each motion leg.
 *
 *   Routine route(feedforward, {
 *       DriveTo(poseInches(24, 48, 0)).afterTravel(inches(12), robotMechanisms().score()),
 *       robotMechanisms().matchLoad(),
 *   }).startAt(poseInches(0, 72, 0));
 *
 * startAt() is optional. It is the pose the selector draws from and the pose
 * the robot is placed at when the routine runs. A routine that omits it is
 * drawn from the field center and runs from wherever the robot already is.
 * mirror() reflects that pose, and every drive and pivot, across the field
 * centerline. Call it before build(). A macro leg stays where it was written.
 * ------------------------------------------------------------------------- */

#include "auton/DriveTo.h"
#include "auton/Mechanisms.h"
#include "auton/TurnTo.h"
#include "core/Types.h"
#include <memory>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace auton {

class Routine {
public:
    // Brace list of DriveTo and TurnTo prvalues. initializer_list stores
    // const elements, so a move-only leg cannot live in one; this constructor
    // moves each leg out of the brace list instead.
    class List {
    public:
        List() = default;

        template<typename... Legs>
            requires(sizeof...(Legs) > 0)
        List(Legs&&... legs) {
            static_assert((isLeg<Legs> && ...),
                          "a routine list holds DriveTo, TurnTo, and macro commands");
            static_assert((std::is_rvalue_reference_v<Legs&&> && ...),
                          "each list entry is a temporary");
            legs_.reserve(sizeof...(Legs));
            (legs_.emplace_back(std::forward<Legs>(legs)), ...);
        }

    private:
        template<typename T>
        static constexpr bool isLeg =
            std::is_same_v<std::remove_cvref_t<T>, DriveTo> ||
            std::is_same_v<std::remove_cvref_t<T>, TurnTo> ||
            std::is_same_v<std::remove_cvref_t<T>, std::unique_ptr<Command>>;

        friend class Routine;
        std::vector<std::variant<DriveTo, TurnTo, std::unique_ptr<Command>>> legs_;
    };

    explicit Routine(const IFeedForward* model = nullptr, List legs = {})
        : model_(model) {
        legs_.reserve(legs.legs_.size());
        for (auto& leg : legs.legs_) legs_.push_back(std::move(leg));
    }

    Routine(const Routine&) = delete;
    Routine& operator=(const Routine&) = delete;
    Routine(Routine&&) = default;
    Routine& operator=(Routine&&) = default;

    // Field inches and heading in degrees, the same frame as poseInches().
    // Speed on the pose is ignored: the robot is placed at rest.
    Routine& startAt(MotionPose pose) & {
        start_ = pose.geometry();
        has_start_ = true;
        return *this;
    }

    Routine startAt(MotionPose pose) && {
        startAt(pose);
        return std::move(*this);
    }

    bool hasStart() const { return has_start_; }
    Pose2d startPose() const { return start_; }

    void mirror() {
        if (has_start_) {
            start_.x = -start_.x;
            start_.theta = mathutil::normalize_angle(-start_.theta);
        }
        for (auto& leg : legs_) {
            if (auto* drive = std::get_if<DriveTo>(&leg)) drive->mirror();
            else if (auto* turn = std::get_if<TurnTo>(&leg)) turn->mirror();
        }
    }

    std::unique_ptr<Command> build() {
        std::vector<std::unique_ptr<Command>> steps;
        steps.reserve(legs_.size());
        for (auto& leg : legs_) {
            if (auto* drive = std::get_if<DriveTo>(&leg)) {
                prepare(*drive);
                steps.push_back(std::make_unique<DriveTo>(std::move(*drive)));
            } else if (auto* turn = std::get_if<TurnTo>(&leg)) {
                prepare(*turn);
                steps.push_back(std::make_unique<TurnTo>(std::move(*turn)));
            } else if (auto* command = std::get_if<std::unique_ptr<Command>>(&leg)) {
                if (*command) steps.push_back(std::move(*command));
            }
        }
        legs_.clear();
        return std::make_unique<SequentialCommandGroup>(std::move(steps));
    }

private:
    void prepare(DriveTo& drive) {
        drive.bindFeedforward(model_);
        drive.compose();
    }

    void prepare(TurnTo& turn) {
        turn.bindFeedforward(model_);
        turn.compose();
    }

    const IFeedForward* model_ = nullptr;
    std::vector<std::variant<DriveTo, TurnTo, std::unique_ptr<Command>>> legs_;
    bool has_start_ = false;
    Pose2d start_{};
};

// Polyline tracked by SegmentFollower. The last 3 inches into the pose
// are straight, along the heading in that pose.
inline const PathConstraints togglePath = [] {
    PathConstraints path = segmentPath();
    path.line_length_in = inches(3);
    return path;
}();

// Intake, cascade, and pneumatics. Every routine commands this same object.
inline Mechanisms& mech = robotMechanisms();

} // namespace auton

//params
constexpr double kReverseStopMps = -1.0e-3;