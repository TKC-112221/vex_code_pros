#pragma once
/* ---------------------------------------------------------------------------
 * SubsystemControl  --  non-chassis actuation (cascade, intake, claw).
 * Decodes buttons (via IMacroController), runs the scoring macro, and applies
 * the resulting SubsystemCommand through the drivebase.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "interfaces/IDriveBase.h"
#include "interfaces/IMacroController.h"
#include <cstdint>

// Live teleop/auton path: decode reading.buttons (+ optional commands context)
// into subsystem actuation and write it to hardware.
SubsystemCommand controlSubsystems(IDriveBase& drive,
                                   IMacroController& macros,
                                   const RobotReading& reading,
                                   const CommandVector& commands,
                                   uint32_t now_ms);

// Live teleop and autonomous. Runs the scoring macro, then writes hardware.
SubsystemCommand applySubsystemCommand(IDriveBase& drive,
                                       IMacroController& macros,
                                       const SubsystemCommand& commanded,
                                       const ButtonState& buttons,
                                       uint32_t now_ms);

// Replay path: apply an already-decoded subsystem command. No scoring macro.
void actuateSubsystems(IDriveBase& drive, const SubsystemCommand& sub);
