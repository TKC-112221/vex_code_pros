#include "logic/SubsystemControl.h"

SubsystemCommand controlSubsystems(IDriveBase& drive,
                                   IMacroController& macros,
                                   const RobotReading& reading,
                                   const CommandVector& /*commands*/,
                                   uint32_t now_ms) {
    // commands is part of the planned API so future subsystem logic can react
    // to chassis intent (e.g. auto-intake while surging). Default ignores it.
    SubsystemCommand sub = macros.processSubsystems(reading.buttons, now_ms);
    drive.applySubsystems(sub);
    return sub;
}

void actuateSubsystems(IDriveBase& drive, const SubsystemCommand& sub) {
    drive.applySubsystems(sub);
}
