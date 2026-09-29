#pragma once
/* ---------------------------------------------------------------------------
 * UIManager  --  controller + brain screen, and the mode/utility buttons that
 * are not about driving: record/sysid/idle toggles (Left/Right), odom-debug
 * toggle (Down), calibrate (Up), save (X), reload (A+B), and manual auton
 * test (Y). Owns edge-detection/debounce
 * so the pipeline loop stays declarative.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "interfaces/IUIManager.h"
#include "api.h"
#include <cstdint>

class UIManager : public IUIManager {
public:
    UIManager();

    void displayMessage(const char* msg) override;

    // Advance record/sysid/debug toggles; returns the (possibly new) mode.
    RobotMode processStateToggles(const ButtonState& btn, RobotMode mode) override;

    bool checkCalibrationRequested(const ButtonState& btn) override;          // Up edge
    bool checkSaveRequested(const ButtonState& btn) override;                 // X held
    bool checkLoadRequested(const ButtonState& btn, RobotMode mode) override; // A+B edge, idle
    bool checkAutonTestRequested(const ButtonState& btn) override;            // Y edge

    void updateDisplay(RobotMode mode, const Pose2d& pose, double total_heading,
                       int frame_count, RobotMode last_recording_mode) override;

    bool debugEnabled() const override { return debug_; }

private:
    void writeScreens(const char* l0, const char* l1, const char* l2);

    pros::Controller* ctrl_;
    bool brain_ok_ = false;
    bool debug_ = false;
    bool prev_up_ = false, prev_dw_ = false, prev_right_ = false, prev_left_ = false;
    bool prev_ab_ = false, prev_y_ = false;
    uint32_t last_t_right_ = 0, last_t_left_ = 0;
    uint32_t last_display_ms_ = 0;   // throttle screen writes
};
