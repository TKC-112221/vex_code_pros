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
 *           .then(robotMechanisms().shooter.spin()),
 *   });
 *   scheduler.add(route.build());
 *
 * Each leg carries its own subcommands. build() sequences the legs, so the
 * next one starts on the same tick the previous one finishes and a non-zero
 * exit speed is still on the robot. The routine binds its feedforward model
 * onto each leg.
 * ------------------------------------------------------------------------- */

#include "auton/DriveTo.h"
#include "auton/TurnTo.h"
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
                          "a routine list holds DriveTo and TurnTo commands");
            static_assert((std::is_rvalue_reference_v<Legs&&> && ...),
                          "each list entry is a temporary");
            legs_.reserve(sizeof...(Legs));
            (legs_.emplace_back(std::forward<Legs>(legs)), ...);
        }

    private:
        template<typename T>
        static constexpr bool isLeg =
            std::is_same_v<std::remove_cvref_t<T>, DriveTo> ||
            std::is_same_v<std::remove_cvref_t<T>, TurnTo>;

        friend class Routine;
        std::vector<std::variant<DriveTo, TurnTo>> legs_;
    };

    explicit Routine(const IFeedForward* model = nullptr, List legs = {})
        : model_(model) {
        legs_.reserve(legs.legs_.size());
        for (auto& leg : legs.legs_) {
            if (auto* drive = std::get_if<DriveTo>(&leg)) prepare(*drive);
            else if (auto* turn = std::get_if<TurnTo>(&leg)) prepare(*turn);
            legs_.push_back(std::move(leg));
        }
    }

    Routine(const Routine&) = delete;
    Routine& operator=(const Routine&) = delete;
    Routine(Routine&&) = default;
    Routine& operator=(Routine&&) = default;

    std::unique_ptr<Command> build() {
        std::vector<std::unique_ptr<Command>> steps;
        steps.reserve(legs_.size());
        for (auto& leg : legs_) {
            if (auto* drive = std::get_if<DriveTo>(&leg)) {
                steps.push_back(std::make_unique<DriveTo>(std::move(*drive)));
            } else if (auto* turn = std::get_if<TurnTo>(&leg)) {
                steps.push_back(std::make_unique<TurnTo>(std::move(*turn)));
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
    std::vector<std::variant<DriveTo, TurnTo>> legs_;
};

} // namespace auton
