#pragma once
/* ---------------------------------------------------------------------------
 * RobotMacro  --  every chassis macro, from one list.
 *
 * Teleop and an autonomous routine both call runRobotMacros(). The first
 * registered macro that is running owns the chassis. The others are cancelled
 * on that tick. Register a new one with registerRobotMacro(); the built-in
 * score and match-load slots are inserted at the front on the first call, so
 * they stay ahead of macros registered from other files.
 *
 * A routine starts one by holding its bit on SubsystemCommand::macro_request
 * (robotMechanisms().run). The driver buttons still start the same macros.
 * While the winner's drive is active, the autonomous leg pauses. When the
 * macro gives the drive back, that leg replans from the pose it is on.
 * ------------------------------------------------------------------------- */

#include "logic/GoalScore.h"
#include <cstdint>

class IFeedForward;
class IMacroController;

// Bits a routine holds in SubsystemCommand::macro_request. A new macro takes
// the next free bit and watches it in its own apply().
constexpr uint8_t kMacroScore = 1u << 0;
constexpr uint8_t kMacroMatchLoad = 1u << 1;

class MacroSlot {
public:
    virtual ~MacroSlot() = default;
    virtual bool running() const = 0;
    virtual void cancel() = 0;
    virtual GoalScoreOutput apply(const SubsystemCommand& command,
                                  const ButtonState& buttons,
                                  const CascadeReading& cascade,
                                  const DistanceReading& front,
                                  const Pose2d& center_in,
                                  const StateEstimate& state,
                                  uint32_t now_ms,
                                  IMacroController& macros,
                                  const IFeedForward* model) = 0;
};

// Later slots lose the chassis to earlier ones. Null and duplicates are ignored.
void registerRobotMacro(MacroSlot* slot);

GoalScoreOutput runRobotMacros(const SubsystemCommand& command,
                               const ButtonState& buttons,
                               const CascadeReading& cascade,
                               const DistanceReading& front,
                               const Pose2d& center_in,
                               const StateEstimate& state,
                               uint32_t now_ms,
                               IMacroController& macros,
                               const IFeedForward* model);

void cancelRobotMacros();

// True after the last runRobotMacros() whose winner was driving.
bool macroOwnsDrive();
