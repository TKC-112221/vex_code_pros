#include "logic/UserInputOverride.h"
#include "logic/DriftCurveOverride.h"
#include "core/Gains.h"
#include <cmath>

namespace {

    DriftCurve g_nav;
    bool g_prev_a = false;

    bool sticksActive(const JoystickData& joy) {
        const double db = Controls::STICK_ZERO_DEADBAND;
        return std::fabs(joy.left_x)  > db || std::fabs(joy.left_y)  > db
            || std::fabs(joy.right_x) > db || std::fabs(joy.right_y) > db;
    }

} // namespace

bool g_drift_curve_on = false;
bool g_drift_curve_abort = false;

DriftCurve& driftCurve() { return g_nav; }

void driftCurveOverridePoll(const ButtonState& buttons,
                            const JoystickData& raw_sticks,
                            RobotMode mode) {
    // A alone. A+B reloads a recording and must not start a drive.
    const bool start = mode == RobotMode::IDLE && buttons.a && !buttons.b && !g_prev_a;
    g_prev_a = buttons.a;

    if (start) {
        g_nav.reset();
        driftCurve().target = {0, 0, 0, 0, 4.0, 0.3, true};
        g_drift_curve_on = true;
    }
    if (g_drift_curve_on && mode != RobotMode::IDLE) g_drift_curve_abort = true;
    if (g_drift_curve_on && sticksActive(raw_sticks)) g_drift_curve_abort = true;
}

CommandVector driftCurveOverride(const StateEstimate& estimate,
                                 const IFeedForward* model) {
    g_nav.bindFeedforward(model);
    return CommandVector::fromChassis(g_nav.compute(estimate));
}

void driftCurveOverrideStop() {
    g_drift_curve_on = false;
    g_drift_curve_abort = false;
    g_nav.reset();
}

int driftCurveOverridePath(const Pose2d& from, Pose2d* out, int cap) {
    return g_nav.samplePath(from, out, cap);
}


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