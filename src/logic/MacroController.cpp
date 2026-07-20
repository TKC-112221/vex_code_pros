/* ---------------------------------------------------------------------------
 * MacroController.cpp  --  ported from v1's read_subsystems_bitmask,
 * apply_subsystems_from_buttons, and the replay abort test.
 * ------------------------------------------------------------------------- */
#include "logic/MacroController.h"

bool MacroController::isAbortRequested(const ButtonState& btn) const {
    return btn.l1 && btn.l2 && btn.r1 && btn.r2;
}

// Shared intake/roller mixing used by both live and recorded decoding.
static SubsystemCommand mixSubsystems(bool r1, bool r2, bool l1, bool l2,
                                      bool g_state, bool b_state, uint8_t bitmask) {
    int intake = 0, top = 0, bottom = 0, intake_m = 2, score_m = 2;
    if (r1) { intake += 100;  top += -2;  bottom += 1;   intake_m -= 1; }
    if (r2) { intake += -100; top += -2;  bottom += 1;   intake_m -= 1; }
    if (l1) {                 top += 106; bottom += 97;  score_m -= 1;  }
    if (l2) {                 top += -24; bottom += -103; score_m -= 1; }

    SubsystemCommand sub;
    sub.intake_pct       = intake * intake_m;
    sub.roller_top_pct   = top * score_m;
    sub.roller_bottom_pct= bottom * score_m;
    sub.pneumatics_g_state = g_state;
    sub.pneumatics_b_state = b_state;
    sub.raw_bitmask = bitmask;
    return sub;
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

    return mixSubsystems(r1, r2, l1, l2, g_state_, b_state_, bitmask);
}

SubsystemCommand MacroController::decodeRecorded(uint8_t bitmask) const {
    bool r1 = bitmask & (1 << 0), r2 = bitmask & (1 << 1);
    bool l1 = bitmask & (1 << 2), l2 = bitmask & (1 << 3);
    bool pg = bitmask & (1 << 4), pb = bitmask & (1 << 5);
    return mixSubsystems(r1, r2, l1, l2, pg, pb, bitmask);
}
