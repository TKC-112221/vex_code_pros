#pragma once
/* ---------------------------------------------------------------------------
 * UIManager  --  controller + brain screen, and the mode/utility buttons that
 * are not about driving: record/idle toggle (Left), odom-debug toggle (Down),
 * calibrate (Up), save (X), reload (A+B), and manual auton test (Y). Owns
 * edge-detection/debounce so the pipeline loop stays declarative.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "interfaces/IUIManager.h"
#include "api.h"
#include <atomic>
#include <cstdint>

class UIManager : public IUIManager {
public:
    UIManager();

    void displayMessage(const char* msg) override;

    // While false, status text is not painted on the brain. The startup clip
    // owns that screen; the controller lines still update.
    void setBrainUpdates(bool on);

    // Advance record/debug toggles; returns the (possibly new) mode.
    RobotMode processStateToggles(const ButtonState& btn, RobotMode mode) override;

    bool checkCalibrationRequested(const ButtonState& btn) override;          // Up edge
    bool checkSaveRequested(const ButtonState& btn) override;                 // X held
    bool checkLoadRequested(const ButtonState& btn, RobotMode mode) override; // A+B edge, idle
    bool checkAutonTestRequested(const ButtonState& btn) override;            // Y edge

    void updateDisplay(RobotMode mode, const Pose2d& pose, double total_heading,
                       int frame_count) override;

    bool debugEnabled() const override { return debug_; }

private:
    // Copy the three status lines, paint the brain immediately, and queue the
    // controller. The controller link takes one line at a time.
    void publishLines(const char* l0, const char* l1, const char* l2);
    void writeBrain();
    void flushControllerLine();

    pros::Controller* ctrl_;
    bool brain_ok_ = false;
    std::atomic<bool> brain_updates_{true};
    bool debug_ = false;
    bool prev_up_ = false, prev_dw_ = false, prev_left_ = false;
    bool prev_ab_ = false, prev_y_ = false;
    uint32_t last_t_left_ = 0;
    uint32_t last_display_ms_ = 0;   // rebuild status text
    uint32_t last_ctrl_ms_ = 0;      // last controller line sent
    bool ctrl_paced_ = false;        // true after the first controller line
    uint8_t ctrl_line_ = 0;          // next controller line to consider
    bool ctrl_known_[3] = {};        // false until that line has been sent once
    char lines_[3][24] = {};
    char sent_[3][24] = {};          // last text actually sent to the controller
};
