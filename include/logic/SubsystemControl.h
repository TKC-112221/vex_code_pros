#pragma once
/* ---------------------------------------------------------------------------
 * SubsystemControl  --  non-chassis actuation (intake, rollers, pneumatics,
 * future lift, ...). Decodes buttons (via IMacroController) and applies the
 * resulting SubsystemCommand through the drivebase.
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

// Replay path: apply an already-decoded subsystem command.
void actuateSubsystems(IDriveBase& drive, const SubsystemCommand& sub);
