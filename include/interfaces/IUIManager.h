#pragma once
/* ---------------------------------------------------------------------------
 * IUIManager  --  the operator interface: screen output plus the mode/utility
 * buttons (record/sysid/debug/calibrate/save/load/auton-test).
 * Mother interface so an alternative front-end (brain LCD + controller, different
 * button map)
 * shares one contract with the pipeline.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"

class IUIManager {
public:
    virtual ~IUIManager() = default;

    virtual void displayMessage(const char* msg) = 0;

    virtual RobotMode processStateToggles(const ButtonState& btn, RobotMode mode) = 0;

    virtual bool checkCalibrationRequested(const ButtonState& btn) = 0;
    virtual bool checkSaveRequested(const ButtonState& btn) = 0;
    virtual bool checkLoadRequested(const ButtonState& btn, RobotMode mode) = 0;
    virtual bool checkAutonTestRequested(const ButtonState& btn) = 0;

    virtual void updateDisplay(RobotMode mode, const Pose2d& pose, double total_heading,
                               int frame_count, RobotMode last_recording_mode) = 0;

    virtual bool debugEnabled() const = 0;
};
