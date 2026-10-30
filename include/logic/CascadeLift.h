#pragma once
/* ---------------------------------------------------------------------------
 * CascadeLift  --  target height to MAIN1/MAIN2 common-mode percent.
 *
 * Four-stage cascade, three moving stages, rigged so each stage rises the
 * same amount as the one below it. The top therefore moves three times the
 * cable the winch pays out. The motors are 1:1 to a 1 inch diameter winch,
 * so one revolution pays out pi inches of cable and the top rises 3 pi inches.
 *
 * Height is inches above wherever the lift sat when the program started.
 * The drive calls zero() from initialize(), using that encoder reading as
 * height 0. The motor's own encoder count is left alone.
 * ------------------------------------------------------------------------- */

#include "core/Specs.h"
#include "core/Tuning.h"
#include "core/Types.h"
#include "filters/SlewLimiter.h"
#include "math/MathUtils.h"
#include <cmath>

class CascadeLift {
public:
    static constexpr double kWinchDiameterIn = 1.0;
    static constexpr int kMovingStages = 3;
    static constexpr double kInchesPerRev =
        kMovingStages * Robot::PI * kWinchDiameterIn;
    static constexpr double kInchesPerDeg = kInchesPerRev / 360.0;
    // Height-target travel from the startup zero. A driver shoulder
    // command is not cut off here.
    static constexpr double kMaxIn = 50.0;

    static double clampHeight(double inches) {
        if (!std::isfinite(inches) || inches < 0.0) return 0.0;
        if (inches > kMaxIn) return kMaxIn;
        return inches;
    }

    void zero(const CascadeReading& now) {
        zero_deg_ = std::isfinite(now.position_deg) ? now.position_deg : 0.0;
    }

    double heightIn(const CascadeReading& now) const {
        const double deg = std::isfinite(now.position_deg) ? now.position_deg : zero_deg_;
        return (deg - zero_deg_) * kInchesPerDeg;
    }

    bool atTarget(double target_in, const CascadeReading& now) const {
        const double rpm = std::isfinite(now.velocity_rpm) ? now.velocity_rpm : 0.0;
        return std::fabs(heightIn(now) - clampHeight(target_in)) <= Tuning::CASCADE_HOLD_IN
            && std::fabs(rpm) <= Tuning::CASCADE_HOLD_RPM;
    }

    // True once the carriage is on the top stop. A target above kMaxIn ends here.
    bool atMax(const CascadeReading& now) const {
        return heightIn(now) >= kMaxIn - Tuning::CASCADE_HOLD_IN;
    }

    // A height target becomes a percent, then the slew limits the ramp.
    // A driver shoulder command stays open-loop past kMaxIn. Held into a
    // stall, that voltage drops so the hold brake keeps the carriage and
    // the drive keeps the battery.
    void writePercent(SubsystemCommand& cmd, const CascadeReading& now) {
        double requested;
        const bool manual = cmd.cascade_manual && !cmd.cascade_height;
        if (cmd.cascade_height) {
            requested = percentFor(clampHeight(cmd.cascade_target_in), now);
        } else if (!manual && cmd.cascade_pct > 0 && heightIn(now) >= kMaxIn) {
            requested = 0.0;
        } else {
            requested = static_cast<double>(cmd.cascade_pct);
        }
        if (manual) {
            if (relieveStall(requested, now)) {
                requested = 0.0;
                slew_.reset(0.0);
            }
        } else {
            stall_sec_ = 0.0;
            stall_hold_ = false;
        }
        const double slewed = slew_.apply(requested);
        int pct = static_cast<int>(std::lround(slewed));
        if (pct > 100) pct = 100;
        if (pct < -100) pct = -100;
        cmd.cascade_pct = pct;
    }

private:
    double percentFor(double target_in, const CascadeReading& now) const {
        const double error = target_in - heightIn(now);
        const double rpm = std::isfinite(now.velocity_rpm) ? now.velocity_rpm : 0.0;
        const double v = rpm / 60.0 * kInchesPerRev;
        if (std::fabs(error) <= Tuning::CASCADE_HOLD_IN
            && std::fabs(rpm) <= Tuning::CASCADE_HOLD_RPM) {
            return 0.0;
        }
        const double stop = (v * v) / (2.0 * Tuning::CASCADE_BRAKE_IN_S2);
        const bool closing = v * error > 0.0;
        if (std::fabs(rpm) > Tuning::CASCADE_HOLD_RPM
            && ((closing && std::fabs(error) <= stop)
                || std::fabs(error) <= Tuning::CASCADE_HOLD_IN)) {
            return -mathutil::sign(v) * 100.0;
        }
        return mathutil::clamp_val(Tuning::CASCADE_KP_PCT_PER_IN * error, -100.0, 100.0);
    }

    // True once a hard open-loop command has been stopped at the current
    // cap. Stays true until that command is released, so easing off does
    // not immediately reapply full voltage.
    bool relieveStall(double requested, const CascadeReading& now) {
        const bool pushing = std::fabs(requested) >= 50.0;
        if (!pushing) {
            stall_sec_ = 0.0;
            stall_hold_ = false;
            return false;
        }
        if (stall_hold_) return true;
        const double rpm = std::isfinite(now.velocity_rpm) ? now.velocity_rpm : 0.0;
        const bool stopped = std::fabs(rpm) <= Tuning::CASCADE_HOLD_RPM;
        const bool hot = now.current_amps >= Tuning::CASCADE_STALL_AMPS;
        if (stopped && hot) stall_sec_ += System::DT_SEC;
        else stall_sec_ = 0.0;
        if (stall_sec_ >= Tuning::CASCADE_STALL_SEC) stall_hold_ = true;
        return stall_hold_;
    }

    double zero_deg_ = 0.0;
    double stall_sec_ = 0.0;
    bool stall_hold_ = false;
    SlewLimiter slew_{Tuning::CASCADE_SLEW_PCT};
};

inline CascadeLift& cascadeLift() {
    static CascadeLift lift;
    return lift;
}
