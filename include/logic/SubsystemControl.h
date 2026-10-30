#pragma once
/* ---------------------------------------------------------------------------
 * SubsystemControl  --  non-chassis actuation (cascade, intake, claw).
 * Decodes buttons (via IMacroController), runs every registered chassis
 * macro through runRobotMacros(), and applies the resulting command.
 * Autonomous uses the same runner, so a routine can start any of them.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "interfaces/IDriveBase.h"
#include "interfaces/IMacroController.h"
#include "logic/GoalScore.h"
#include "logic/RobotMacro.h"
#include <cstdint>

class IFeedForward;

// Live teleop path: decode reading.buttons into subsystem actuation, run every
// registered macro, and write the winner to hardware. drive_active replaces
// the stick command while that macro owns the chassis.
GoalScoreOutput controlSubsystems(IDriveBase& drive,
                                  IMacroController& macros,
                                  const RobotReading& reading,
                                  const CommandVector& commands,
                                  const Pose2d& center_in,
                                  const StateEstimate& state,
                                  uint32_t now_ms,
                                  const IFeedForward* model);

// Live teleop and autonomous. Runs every registered macro, then writes hardware.
GoalScoreOutput applySubsystemCommand(IDriveBase& drive,
                                      IMacroController& macros,
                                      const SubsystemCommand& commanded,
                                      const ButtonState& buttons,
                                      const Pose2d& center_in,
                                      const StateEstimate& state,
                                      const DistanceReading& front,
                                      uint32_t now_ms,
                                      const IFeedForward* model);

// Replay path: apply an already-decoded subsystem command. No scoring macro.
void actuateSubsystems(IDriveBase& drive, const SubsystemCommand& sub);
