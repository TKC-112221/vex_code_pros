/* ---------------------------------------------------------------------------
 * UIManager.cpp  --  controller screen + brain LLEMU + mode/utility buttons.
 * Brain LLEMU is a label update (LVGL draws on its own task), so all three
 * lines are written on the pipeline. The controller link is the slow call:
 * one line per flush, at least 50 ms apart, and only when that line changed.
 * On the V5 that flush runs from the brain UI task. On the host it stays
 * inline, because the stand-in print does not block.
 * ------------------------------------------------------------------------- */
#include "logic/UIManager.h"
#include <cstdio>
#include <cstring>
#include <cmath>

namespace {
constexpr uint32_t kStatusPeriodMs = 100;
constexpr uint32_t kControllerLineMs = 50;
}

UIManager::UIManager() {
    ctrl_ = new pros::Controller(pros::E_CONTROLLER_MASTER);
    // Host paints LLEMU text on the field window. The V5 brain is the
    // autonomous selector, so LLEMU is not started there.
#if defined(ROBOT_HOST)
    brain_ok_ = pros::lcd::initialize();
#else
    brain_ok_ = false;
#endif
}

void UIManager::publishLines(const char* l0, const char* l1, const char* l2) {
    std::snprintf(lines_[0], sizeof(lines_[0]), "%s", l0 != nullptr ? l0 : "");
    std::snprintf(lines_[1], sizeof(lines_[1]), "%s", l1 != nullptr ? l1 : "");
    std::snprintf(lines_[2], sizeof(lines_[2]), "%s", l2 != nullptr ? l2 : "");
    writeBrain();
#if defined(ROBOT_HOST)
    flushControllerLine();
#endif
}

void UIManager::serviceController() {
    flushControllerLine();
}

void UIManager::setBrainUpdates(bool on) {
    brain_updates_.store(on);
}

void UIManager::writeBrain() {
#if defined(ROBOT_HOST)
    if (!brain_ok_ || !brain_updates_.load()) return;
    pros::lcd::print(0, "%s", lines_[0]);
    pros::lcd::print(1, "%s", lines_[1]);
    pros::lcd::print(2, "%s", lines_[2]);
#endif
}

void UIManager::flushControllerLine() {
    const uint32_t now = pros::millis();
    if (ctrl_paced_ && now - last_ctrl_ms_ < kControllerLineMs) return;

    for (int n = 0; n < 3; ++n) {
        const int line = (ctrl_line_ + n) % 3;
        if (ctrl_known_[line] && std::strcmp(lines_[line], sent_[line]) == 0) continue;

        // Pad so a shorter string overwrites the previous characters.
        ctrl_->print((std::uint8_t)line, 0, "%-19s", lines_[line]);
        std::snprintf(sent_[line], sizeof(sent_[line]), "%s", lines_[line]);
        ctrl_known_[line] = true;
        ctrl_paced_ = true;
        last_ctrl_ms_ = now;
        ctrl_line_ = (std::uint8_t)((line + 1) % 3);
        return;
    }
}

void UIManager::displayMessage(const char* msg) {
    publishLines(msg, "", "");
}

RobotMode UIManager::processStateToggles(const ButtonState& btn, RobotMode mode) {
    uint32_t now = pros::millis();

    // Down: toggle odom debug view.
    if (btn.down && !prev_dw_) debug_ = !debug_;
    prev_dw_ = btn.down;

    // Left: IDLE <-> RECORDING (0.3 s debounce). One unified capture for
    // trajectory replay and system identification.
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
        case RobotMode::IDLE:      return "IDLE";
        case RobotMode::RECORDING: return "RECORDING";
        case RobotMode::REPLAYING: return "REPLAYING";
        case RobotMode::COMMAND:   return "COMMAND";
    }
    return "UNKNOWN";
}

void UIManager::updateDisplay(RobotMode mode, const Pose2d& pose, double total_heading,
                              int frame_count, bool recording) {
    const uint32_t now = pros::millis();
    if (now - last_display_ms_ >= kStatusPeriodMs) {
        last_display_ms_ = now;

        lines_[0][0] = '\0';
        lines_[1][0] = '\0';
        lines_[2][0] = '\0';
        if (debug_) {
            std::snprintf(lines_[0], sizeof(lines_[0]), "X:%5.1f Y:%5.1f in", pose.x, pose.y);
            std::snprintf(lines_[1], sizeof(lines_[1]), "H:%5.1f deg", total_heading * (180.0 / M_PI));
            std::snprintf(lines_[2], sizeof(lines_[2]), "-- ODOM DEBUG --");
        } else if (mode == RobotMode::RECORDING) {
            std::snprintf(lines_[2], sizeof(lines_[2]), "REC: %d", frame_count);
        } else if (recording) {
            std::snprintf(lines_[2], sizeof(lines_[2]), "CMD REC: %d", frame_count);
        } else if (mode == RobotMode::IDLE) {
            if (frame_count > 0)
                std::snprintf(lines_[2], sizeof(lines_[2]), "Save Ready: %d", frame_count);
            else
                std::snprintf(lines_[2], sizeof(lines_[2]), "IDLE: Empty");
        } else {
            std::snprintf(lines_[2], sizeof(lines_[2]), "%s", modeName(mode));
        }
        writeBrain();
    }

    // Host only. The V5 control tick must not block in controller_print;
    // brain_ui_task calls serviceController().
#if defined(ROBOT_HOST)
    flushControllerLine();
#endif
}
