#include "logic/SubsystemControl.h"
#include "logic/ScoreMacro.h"

SubsystemCommand applySubsystemCommand(IDriveBase& drive,
                                       IMacroController& macros,
                                       const SubsystemCommand& commanded,
                                       const ButtonState& buttons,
                                       uint32_t now_ms) {
    const SubsystemCommand out = liveScoreMacro().apply(
        commanded, buttons, drive.readCascade(), now_ms, macros);
    drive.applySubsystems(out);
    return out;
}

SubsystemCommand controlSubsystems(IDriveBase& drive,
                                   IMacroController& macros,
                                   const RobotReading& reading,
                                   const CommandVector& /*commands*/,
                                   uint32_t now_ms) {
    // commands is part of the planned API so future subsystem logic can react
    // to chassis intent (e.g. auto-intake while surging). Default ignores it.
    const SubsystemCommand sub = macros.processSubsystems(reading.buttons, now_ms);
    return applySubsystemCommand(drive, macros, sub, reading.buttons, now_ms);
}

void actuateSubsystems(IDriveBase& drive, const SubsystemCommand& sub) {
    drive.applySubsystems(sub);
}
