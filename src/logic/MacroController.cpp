/* ---------------------------------------------------------------------------
 * MacroController.cpp  --  ported from v1's read_subsystems_bitmask,
 * apply_subsystems_from_buttons, and the replay abort test.
 * ------------------------------------------------------------------------- */
#include "logic/MacroController.h"
#include <cstdio>

bool MacroController::isAbortRequested(const ButtonState& btn) const {
    return btn.l1 && btn.l2 && btn.r1 && btn.r2;
}

static SubsystemCommand mixSubsystems(bool r1, bool r2, bool l1, bool l2,
                                      bool g_state, bool b_state, uint8_t bitmask) {
    return mixShoulderButtons(r1, r2, l1, l2, g_state, b_state, bitmask);
}

SubsystemCommand MacroController::processSubsystems(const ButtonState& btn, uint32_t now_ms) {
    bool r1 = btn.r1, r2 = btn.r2, l1 = btn.l1, l2 = btn.l2;

    // Bitmask reflects the pneumatics state active *before* this tick's toggle
    // (matches v1's read-before-apply ordering so recordings stay comparable).
    uint8_t bitmask = (uint8_t)((r1 ? 1 : 0) << 0) | ((r2 ? 1 : 0) << 1)
                    | ((l1 ? 1 : 0) << 2) | ((l2 ? 1 : 0) << 3)
                    | ((g_state_ ? 1 : 0) << 4) | ((b_state_ ? 1 : 0) << 5)
                    | ((btn.a ? 1 : 0) << 6) | ((btn.b ? 1 : 0) << 7);

    // Latching pneumatics with a 500 ms debounce after a replay abort.
    bool debounce_active = (abort_ts_ != 0) && ((now_ms - abort_ts_) < 500);
    if (!debounce_active) {
        if ((l1 && l2) && !prev_dw_) { b_state_ = !b_state_; prev_dw_ = true; }
        if ((r1 && r2) && !prev_up_) { g_state_ = !g_state_; prev_up_ = true; }
    }
    prev_dw_ = (l1 && l2);
    prev_up_ = (r1 && r2);

    captureIntent(r1, r2, l1, l2);
    // Live driver control speaks SubsystemIntent. The bitmask stays on the
    // command so a recording can still be replayed by decodeRecorded.
    SubsystemCommand sub = intent_.toCommand();
    sub.raw_bitmask = bitmask;
    sub.cascade_manual = true;

    static bool prev_l1 = false, prev_l2 = false, prev_r1 = false, prev_r2 = false;
    if (l1 != prev_l1 || l2 != prev_l2 || r1 != prev_r1 || r2 != prev_r2) {
        prev_l1 = l1;
        prev_l2 = l2;
        prev_r1 = r1;
        prev_r2 = r2;
        std::printf("subsys: buttons L1=%d L2=%d R1=%d R2=%d -> cascade %d intake %d\n",
                    l1 ? 1 : 0, l2 ? 1 : 0, r1 ? 1 : 0, r2 ? 1 : 0,
                    sub.cascade_pct, sub.intake_pct);
        std::fflush(stdout);
    }
    return sub;
}

void MacroController::captureIntent(bool r1, bool r2, bool l1, bool l2) {
    if (r1 && !r2) intent_.intake = IntakeState::In;
    else if (r2 && !r1) intent_.intake = IntakeState::Out;
    else intent_.intake = IntakeState::Idle;

    if (l1 && !l2) intent_.cascade = CascadeState::Up;
    else if (l2 && !l1) intent_.cascade = CascadeState::Down;
    else intent_.cascade = CascadeState::Idle;

    intent_.pneumatic_g = g_state_;
    intent_.pneumatic_b = b_state_;
}

SubsystemCommand MacroController::decodeRecorded(uint8_t bitmask) const {
    bool r1 = bitmask & (1 << 0), r2 = bitmask & (1 << 1);
    bool l1 = bitmask & (1 << 2), l2 = bitmask & (1 << 3);
    bool pg = bitmask & (1 << 4), pb = bitmask & (1 << 5);
    return mixSubsystems(r1, r2, l1, l2, pg, pb, bitmask);
}
