#pragma once
/* ---------------------------------------------------------------------------
 * Settings.h  --  Feature flags and mode selection. Ported verbatim from v1
 * (config/toggles.h). These toggles gate optional control behaviors so the
 * pipeline's active path exactly matches v1's configured behavior.
 * ------------------------------------------------------------------------- */

namespace Settings {

    constexpr bool SKILLS_MODE = true;

    // Competition autonomous(), and controller Y during driver control.
    // True runs buildSkillsRoutine() through the command scheduler. False
    // keeps the recorded-log replay selected by SKILLS_MODE.
    namespace Auton {
        constexpr bool USE_COMMAND_SCHEDULER = true;
    }

    namespace ControlToggles {
        // Odometry source
        constexpr bool USE_TRACKING_WHEELS = false;

        // Feedback & feedforward
        constexpr bool RAMSETE          = false;
        constexpr bool PID_ADJUSTMENT   = true;   // velocity PID always applied
        constexpr bool POSITION_PID_LIN = true;
        constexpr bool POSITION_PID_ANG = true;

        // Per-actuator post-processing
        constexpr bool CURRENT_LIMITING = false;
        constexpr bool STICK_ZERO_BRAKE = false;
        constexpr bool VOLTAGE_CLAMP    = true;

        // Pose override. False keeps DriftFollower. True runs DrivePolicy
        // on the same StateEstimate and DriftCurveTarget, and returns the
        // universal chassis command (surge / sway / yaw volts).
        constexpr bool USE_DRIVE_POLICY = false;

        // Behavior
        constexpr bool TETHERING       = false;
        constexpr bool BATTERY_SCALING = false;

        // Emergency fallback when the IMU fails (wheel-derived heading/omega)
        constexpr bool IMU_FALLBACK = false;

        // Sensor denoiser in readRobotInput. true: causal Savitzky-Golay
        // (math/SensorSavitzkyGolay.h). false: the 3-sample Kalman.
        constexpr bool SAVITZKY_GOLAY_FILTER = true;

        // Autonomous replay
        constexpr bool REPLAY_VOLTAGE_FALLBACK = true;
        constexpr bool PID_ADJUSTMENT_REPLAY   = false;
        constexpr bool USE_RECORDED_VOLTS      = true;
        constexpr bool ENABLE_SPATIAL_WARP     = false;
    }

    // Startup clip on the brain. The control tick is already running; sticks
    // stay locked until the IMU is ready, which does not wait for this clip.
    // A missing first file returns immediately, so an empty card still boots.
    // Export frames with: ffmpeg -i clip.mp4 -vf scale=480:272 splash/frame_%04d.png
    // Copy that folder to the microSD card. On the host, leave it in the
    // working directory (the "/usd/" prefix is stripped).
    // frame_step 1 plays every file; 2 plays every other file.
    // Touch the brain, click its screen in the main window, or press Escape, to skip.
    namespace Splash {
        constexpr bool ENABLED = true;
        //constexpr const char* PATH_FORMAT = "/usd/VEX/SystemStartAnimation/GANDUM_SystemInitiation/frame_%03d.png";
        constexpr const char* PATH_FORMAT = "/usd/VEX/SystemStartAnimation/SquareHud/frame_%03d.png";
        constexpr int FIRST_FRAME = 1;
        //constexpr int FRAME_COUNT = 387;
        constexpr int FRAME_COUNT = 258;
        constexpr int FRAME_STEP = 2;
        constexpr double FPS = 30.0;
    }
}
