#pragma once
/* ---------------------------------------------------------------------------
 * ScoreMacro  --  score one game piece on the cascade.
 *
 * 1. Run the cascade up until it stalls at the top.
 * 2. Sample how much current it takes to hold the piece there.
 * 3. Lower until the goal takes the piece: motor current rises (more torque
 *    to keep pulling down) or falls off the loaded-lower baseline (less power
 *    than holding the piece up).
 * 4. Open the claw (pneumatic B).
 * 5. Run the cascade down until it stalls at the bottom.
 *
 * Driver: d-pad Right starts it. Right again, or L1 / L2, cancels it.
 * Autonomous: SubsystemCommand::score rising edge starts the same machine.
 * The stall and seat currents are the numbers to change on the real lift.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include <cstdint>

class IMacroController;

namespace ScoreTune {
    constexpr int kRaisePct = 100;
    constexpr int kSettlePct = -40;
    constexpr int kHomePct = -100;

    constexpr double kStallRpm = 12.0;
    constexpr double kStallAmps = 2.2;
    constexpr uint32_t kStallMs = 120;
    constexpr uint32_t kRaiseArmMs = 80;
    constexpr uint32_t kRaiseTimeoutMs = 3000;
    constexpr uint32_t kHoldSampleMs = 150;
    constexpr uint32_t kLowerSkipMs = 120;
    constexpr uint32_t kLowerBaselineMs = 100;
    constexpr uint32_t kSettleTimeoutMs = 2500;
    constexpr uint32_t kConfirmMs = 40;
    constexpr uint32_t kClawMs = 300;
    constexpr uint32_t kHomeTimeoutMs = 2500;

    constexpr double kMinSettleDeg = 40.0;
    constexpr double kMinHomeDeg = 40.0;
    constexpr double kMotionRpm = 25.0;
    constexpr double kTorqueRiseRatio = 1.6;
    constexpr double kTorqueRiseAmps = 0.35;
    constexpr double kUnloadRatio = 0.55;
    constexpr double kHoldDropRatio = 0.65;
    constexpr double kTorqueFloorNm = 0.02;
    constexpr double kHoldFloorAmps = 0.15;

    // Pneumatic B high releases the claw.
    constexpr bool kClawReleased = true;
}

class ScoreMacro {
public:
    SubsystemCommand apply(const SubsystemCommand& driver,
                           const ButtonState& buttons,
                           const CascadeReading& sense,
                           uint32_t now_ms,
                           IMacroController& macros);

    bool running() const { return phase_ != Phase::Idle; }
    bool releasedClaw() const { return released_claw_; }
    const char* phaseName() const {
        switch (phase_) {
            case Phase::Raise: return "Raise";
            case Phase::Hold: return "Hold";
            case Phase::Settle: return "Settle";
            case Phase::Release: return "Release";
            case Phase::Home: return "Home";
            case Phase::Idle: break;
        }
        return "Idle";
    }
    void cancel();

private:
    enum class Phase { Idle, Raise, Hold, Settle, Release, Home };

    void begin(uint32_t now_ms);
    void enter(Phase phase, uint32_t now_ms);
    bool stalled(const CascadeReading& sense, uint32_t now_ms);

    Phase phase_ = Phase::Idle;
    uint32_t phase_ms_ = 0;
    bool slow_valid_ = false;
    uint32_t slow_since_ = 0;
    bool seat_valid_ = false;
    uint32_t seat_since_ = 0;
    bool prev_right_ = false;
    bool score_held_ = false;
    bool released_claw_ = false;
    bool baseline_ready_ = false;
    bool saw_motion_ = false;
    int sample_n_ = 0;
    double sample_amps_ = 0;
    double sample_torque_ = 0;
    double hold_amps_ = 0;
    double baseline_amps_ = 0;
    double baseline_torque_ = 0;
    double baseline_deg_ = 0;
    double home_deg_ = 0;
};

ScoreMacro& liveScoreMacro();
inline void cancelScoreMacro() { liveScoreMacro().cancel(); }
inline bool scoreMacroRunning() { return liveScoreMacro().running(); }
inline bool scoreMacroReleasedClaw() { return liveScoreMacro().releasedClaw(); }
