#pragma once
/* ---------------------------------------------------------------------------
 * LogFrameReplayer  --  concrete ITrajectoryManager backed by a fixed LogFrame
 * buffer. Records live frames, dumps/loads them, and during replay emits an
 * interpolated MotionTarget each tick (with recorded chassis feedforward),
 * pacing playback with a battery-aware time-warp.
 * ------------------------------------------------------------------------- */

#include "interfaces/ITrajectoryManager.h"
#include "core/LogFrame.h"
#include "core/Specs.h"
#include <array>

class LogFrameReplayer : public ITrajectoryManager {
public:
    void loadFromHeader() override;
    void saveToConsole() override;

    void beginRecording(RobotMode mode, const Pose2d& startPose) override;
    void recordFrame(const MotionTarget& target,
                     const Pose2d& actual,
                     const CommandVector& command,
                     const ChassisCommand& applied,
                     const SubsystemCommand& sub,
                     const ChassisObservation& obs,
                     double lin_error,
                     double ang_error) override;
    int getFrameCount() const override { return length_; }

    void resetPlayback() override;
    Pose2d getStartPose() const override;
    double getStartHeading() const override;
    MotionTarget getNextTarget(const Pose2d& pose, const ChassisObservation& obs) override;
    SubsystemCommand getTargetSubsystems() const override { return current_sub_; }
    bool isFinished() const override { return finished_; }

private:
    std::array<LogFrame, System::MAX_RECORD_FRAMES> buffer_{};
    int length_ = 0;
    RobotMode recording_mode_ = RobotMode::RECORDING;

    // Replay cursor + baselines
    float  replay_index_ = 0.0f;
    bool   finished_ = false;
    double l_pos_start_ = 0, r_pos_start_ = 0, ref_l_0_ = 0, ref_r_0_ = 0;
    bool   has_valid_wheel_ref_ = false;
    bool   baseline_captured_ = false;
    SubsystemCommand current_sub_{};
};
