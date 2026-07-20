/* ---------------------------------------------------------------------------
 * UIManager.cpp  --  controller screen + mode/utility buttons on the PROS API.
 * The V5 controller screen is rate-limited (unlike VEXcode), so periodic status
 * updates are throttled and lines are overwritten with padding instead of a
 * full clear each tick.
 * ------------------------------------------------------------------------- */
#include "logic/UIManager.h"
#include <cstdio>
#include <cmath>

UIManager::UIManager() {
    ctrl_ = new pros::Controller(pros::E_CONTROLLER_MASTER);
}

void UIManager::displayMessage(const char* msg) {
    ctrl_->print(0, 0, "%-19s", msg);
    ctrl_->print(1, 0, "%-19s", "");
    ctrl_->print(2, 0, "%-19s", "");
}

RobotMode UIManager::processStateToggles(const ButtonState& btn, RobotMode mode) {
    uint32_t now = pros::millis();

    // Down: toggle odom debug view.
    if (btn.down && !prev_dw_) debug_ = !debug_;
    prev_dw_ = btn.down;

    // Right: IDLE <-> SYSID_DRIVE (0.3 s debounce).
    if (btn.right && !prev_right_ && (now - last_t_right_ > 300)) {
        mode = (mode == RobotMode::IDLE) ? RobotMode::SYSID_DRIVE : RobotMode::IDLE;
        last_t_right_ = now;
    }
    prev_right_ = btn.right;

    // Left: IDLE <-> RECORDING (0.3 s debounce).
    if (btn.left && !prev_left_ && (now - last_t_left_ > 300)) {
        if (mode == RobotMode::IDLE)            mode = RobotMode::RECORDING;
        else if (mode == RobotMode::RECORDING)  mode = RobotMode::IDLE;
        last_t_left_ = now;
    }
    prev_left_ = btn.left;

    return mode;
}

bool UIManager::checkCalibrationRequested(const ButtonState& btn) {
    bool edge = btn.up && !prev_up_;
    prev_up_ = btn.up;
    return edge;
}

bool UIManager::checkSaveRequested(const ButtonState& btn) {
    return btn.x;   // caller gates on IDLE + non-empty buffer
}

bool UIManager::checkLoadRequested(const ButtonState& btn, RobotMode mode) {
    bool edge = false;
    if (mode == RobotMode::IDLE && btn.a && btn.b && !prev_ab_) {
        edge = true;
        prev_ab_ = true;
    } else if (!btn.a || !btn.b) {
        prev_ab_ = false;
    }
    return edge;
}

bool UIManager::checkAutonTestRequested(const ButtonState& btn) {
    bool edge = btn.y && !prev_y_;
    prev_y_ = btn.y;
    return edge;
}

static const char* modeName(RobotMode m) {
    switch (m) {
        case RobotMode::IDLE:        return "IDLE";
        case RobotMode::RECORDING:   return "REC: TRAJ";
        case RobotMode::REPLAYING:   return "REPLAYING";
        case RobotMode::SYSID_DRIVE: return "REC: SYSID";
    }
    return "UNKNOWN";
}

void UIManager::updateDisplay(RobotMode mode, const Pose2d& pose, double total_heading,
                              int frame_count, RobotMode last_recording_mode) {
    // Throttle: the V5 controller screen cannot keep up with a 10 ms loop.
    uint32_t now = pros::millis();
    if (now - last_display_ms_ < 100) return;
    last_display_ms_ = now;

    char l0[24] = "", l1[24] = "", l2[24] = "";
    if (debug_) {
        snprintf(l0, sizeof(l0), "X:%5.2f Y:%5.2f", pose.x, pose.y);
        snprintf(l1, sizeof(l1), "H:%5.1f deg", total_heading * (180.0 / M_PI));
        snprintf(l2, sizeof(l2), "-- ODOM DEBUG --");
    } else {
        if (mode == RobotMode::RECORDING || mode == RobotMode::SYSID_DRIVE) {
            snprintf(l2, sizeof(l2), "REC: %d", frame_count);
        } else if (mode == RobotMode::IDLE) {
            if (frame_count > 0)
                snprintf(l2, sizeof(l2), "Save Ready: %s",
                         (last_recording_mode == RobotMode::SYSID_DRIVE ? "SYSID" : "TRAJ"));
            else
                snprintf(l2, sizeof(l2), "IDLE: Empty");
        } else {
            snprintf(l2, sizeof(l2), "%s", modeName(mode));
        }
    }

    ctrl_->print(0, 0, "%-19s", l0);
    ctrl_->print(1, 0, "%-19s", l1);
    ctrl_->print(2, 0, "%-19s", l2);
}
