#include "logic/UserInputOverride.h"
#include "logic/DriftCurveOverride.h"

OverrideResult userInputOverride(const ChassisObservation& /*sensor*/,
                                 const CommandVector& commands,
                                 const ButtonState& buttons,
                                 const StateEstimate& estimate,
                                 const JoystickData& raw_sticks,
                                 RobotMode mode,
                                 const IFeedForward* model) {
    OverrideResult out;
    out.commands = commands;
    out.buttons  = buttons;

    driftCurveOverridePoll(buttons, raw_sticks, mode);
    if (g_drift_curve_abort) {
        driftCurveOverrideStop();
    } else if (g_drift_curve_on) {
        out.commands = driftCurveOverride(estimate, model);
    }
    return out;
}

int userInputOverridePath(const Pose2d& from, Pose2d* out, int cap) {
    if (g_drift_curve_abort || !g_drift_curve_on || out == nullptr || cap < 2) return 0;
    return driftCurveOverridePath(from, out, cap);
}