#pragma once
/* ---------------------------------------------------------------------------
 * MacroController  --  button macros not tied to the drive target: the
 * four-trigger replay abort, and the subsystem behavior. R1/R2 run the intake
 * (MAIN1 and MAIN2 opposite). L1/L2 run the cascade (both the same way).
 * L1+L2 toggles the claw (pneumatic B). R1+R2 toggles pneumatic G. D-pad
 * Right starts the scoring macro. B alone drives to the nearest match
 * loader. Owns the pneumatics latch.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "interfaces/IMacroController.h"
#include <cstdint>

class MacroController : public IMacroController {
public:
    // All four shoulder triggers held -> abort autonomous replay.
    bool isAbortRequested(const ButtonState& btn) const override;

    // Note when a replay abort happened, to debounce pneumatics toggles after.
    void notifyAbort(uint32_t now_ms) override { abort_ts_ = now_ms; }

    // Decode live buttons into a subsystem command (+ toggle pneumatics).
    SubsystemCommand processSubsystems(const ButtonState& btn, uint32_t now_ms) override;

    // Decode a recorded bitmask into a subsystem command (replay path).
    SubsystemCommand decodeRecorded(uint8_t bitmask) const override;

    SubsystemIntent subsystemIntent() const override { return intent_; }
    void setPneumaticB(bool engaged) override {
        b_state_ = engaged;
        intent_.pneumatic_b = engaged;
    }

private:
    void captureIntent(bool r1, bool r2, bool l1, bool l2);

    SubsystemIntent intent_{};
    bool g_state_ = false, b_state_ = false;
    bool prev_up_ = false, prev_dw_ = false;
    uint32_t abort_ts_ = 0;
};
