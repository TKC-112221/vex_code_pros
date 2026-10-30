#include "logic/ScoreMacro.h"
#include "interfaces/IMacroController.h"
#include <cmath>

ScoreMacro& liveScoreMacro() {
    static ScoreMacro macro;
    return macro;
}

void ScoreMacro::cancel() {
    phase_ = Phase::Idle;
    slow_valid_ = false;
    seat_valid_ = false;
}

void ScoreMacro::begin(uint32_t now_ms) {
    released_claw_ = false;
    hold_amps_ = 0;
    baseline_ready_ = false;
    enter(Phase::Raise, now_ms);
}

void ScoreMacro::enter(Phase phase, uint32_t now_ms) {
    phase_ = phase;
    phase_ms_ = now_ms;
    slow_valid_ = false;
    seat_valid_ = false;
    saw_motion_ = false;
    sample_n_ = 0;
    sample_amps_ = 0;
    sample_torque_ = 0;
    if (phase != Phase::Settle) baseline_ready_ = false;
}

bool ScoreMacro::stalled(const CascadeReading& sense, uint32_t now_ms) {
    const bool slow = std::fabs(sense.velocity_rpm) <= ScoreTune::kStallRpm
                   && sense.current_amps >= ScoreTune::kStallAmps;
    if (!slow) {
        slow_valid_ = false;
        return false;
    }
    if (!slow_valid_) {
        slow_valid_ = true;
        slow_since_ = now_ms;
        return false;
    }
    return now_ms - slow_since_ >= ScoreTune::kStallMs;
}

SubsystemCommand ScoreMacro::apply(const SubsystemCommand& driver,
                                   const ButtonState& buttons,
                                   const CascadeReading& sense,
                                   uint32_t now_ms,
                                   IMacroController& macros) {
    const bool right_edge = buttons.right && !prev_right_;
    prev_right_ = buttons.right;
    const bool score_edge = driver.score && !score_held_;
    score_held_ = driver.score;

    if (phase_ != Phase::Idle) {
        if (right_edge || buttons.l1 || buttons.l2) {
            cancel();
            return driver;
        }
    } else if (right_edge || score_edge) {
        begin(now_ms);
    }

    if (phase_ == Phase::Idle) return driver;

    SubsystemCommand out = driver;
    out.intake_pct = 0;
    out.score = true;
    out.cascade_manual = false;
    const uint32_t elapsed = now_ms - phase_ms_;

    switch (phase_) {
        case Phase::Idle:
            break;
        case Phase::Raise:
            out.cascade_pct = ScoreTune::kRaisePct;
            if ((elapsed >= ScoreTune::kRaiseArmMs && stalled(sense, now_ms))
                || elapsed >= ScoreTune::kRaiseTimeoutMs) {
                enter(Phase::Hold, now_ms);
            }
            break;
        case Phase::Hold:
            // Current drawn while the brake holds the piece up.
            out.cascade_pct = 0;
            sample_amps_ += sense.current_amps;
            ++sample_n_;
            if (elapsed >= ScoreTune::kHoldSampleMs && sample_n_ > 0) {
                hold_amps_ = sample_amps_ / static_cast<double>(sample_n_);
                enter(Phase::Settle, now_ms);
            }
            break;
        case Phase::Settle: {
            out.cascade_pct = ScoreTune::kSettlePct;
            if (!baseline_ready_) {
                if (elapsed >= ScoreTune::kLowerSkipMs) {
                    sample_amps_ += sense.current_amps;
                    sample_torque_ += sense.torque_nm;
                    ++sample_n_;
                    if (elapsed >= ScoreTune::kLowerSkipMs + ScoreTune::kLowerBaselineMs
                        && sample_n_ > 0) {
                        baseline_amps_ = sample_amps_ / static_cast<double>(sample_n_);
                        baseline_torque_ = sample_torque_ / static_cast<double>(sample_n_);
                        baseline_deg_ = sense.position_deg;
                        baseline_ready_ = true;
                    }
                }
            } else {
                const double dropped = baseline_deg_ - sense.position_deg;
                const bool traveled = dropped >= ScoreTune::kMinSettleDeg;
                const bool more_torque =
                    sense.current_amps > baseline_amps_ * ScoreTune::kTorqueRiseRatio
                    && sense.current_amps > baseline_amps_ + ScoreTune::kTorqueRiseAmps;
                const bool torque_rose =
                    std::fabs(baseline_torque_) > ScoreTune::kTorqueFloorNm
                    && std::isfinite(sense.torque_nm)
                    && std::fabs(sense.torque_nm) > std::fabs(baseline_torque_) * ScoreTune::kTorqueRiseRatio;
                const bool less_power =
                    sense.current_amps < baseline_amps_ * ScoreTune::kUnloadRatio
                    && (hold_amps_ < ScoreTune::kHoldFloorAmps
                        || sense.current_amps < hold_amps_ * ScoreTune::kHoldDropRatio);
                if (traveled && (more_torque || torque_rose || less_power)) {
                    if (!seat_valid_) {
                        seat_valid_ = true;
                        seat_since_ = now_ms;
                    } else if (now_ms - seat_since_ >= ScoreTune::kConfirmMs) {
                        enter(Phase::Release, now_ms);
                    }
                } else {
                    seat_valid_ = false;
                }
            }
            if (phase_ == Phase::Settle && elapsed >= ScoreTune::kSettleTimeoutMs) {
                enter(Phase::Release, now_ms);
            }
            break;
        }
        case Phase::Release:
            out.cascade_pct = 0;
            out.pneumatics_b_state = ScoreTune::kClawReleased;
            if (!released_claw_) {
                released_claw_ = true;
                macros.setPneumaticB(ScoreTune::kClawReleased);
            }
            if (elapsed >= ScoreTune::kClawMs) {
                home_deg_ = sense.position_deg;
                enter(Phase::Home, now_ms);
            }
            break;
        case Phase::Home:
            out.cascade_pct = ScoreTune::kHomePct;
            out.pneumatics_b_state = ScoreTune::kClawReleased;
            if (std::fabs(sense.velocity_rpm) > ScoreTune::kMotionRpm) saw_motion_ = true;
            if (saw_motion_
                && (home_deg_ - sense.position_deg) >= ScoreTune::kMinHomeDeg
                && stalled(sense, now_ms)) {
                phase_ = Phase::Idle;
                out.cascade_pct = 0;
            } else if (elapsed >= ScoreTune::kHomeTimeoutMs) {
                phase_ = Phase::Idle;
                out.cascade_pct = 0;
            }
            break;
    }

    if (released_claw_) out.pneumatics_b_state = ScoreTune::kClawReleased;
    return out;
}
