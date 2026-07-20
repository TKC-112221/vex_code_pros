#pragma once
/* ---------------------------------------------------------------------------
 * Settings.h  --  Feature flags and mode selection. Ported verbatim from v1
 * (config/toggles.h). These toggles gate optional control behaviors so the
 * pipeline's active path exactly matches v1's configured behavior.
 * ------------------------------------------------------------------------- */

namespace Settings {

    constexpr bool SKILLS_MODE = true;

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

        // Behavior
        constexpr bool TETHERING       = false;
        constexpr bool BATTERY_SCALING = false;

        // Emergency fallback when the IMU fails (wheel-derived heading/omega)
        constexpr bool IMU_FALLBACK = false;

        // Autonomous replay
        constexpr bool REPLAY_VOLTAGE_FALLBACK = true;
        constexpr bool PID_ADJUSTMENT_REPLAY   = false;
        constexpr bool USE_RECORDED_VOLTS      = true;
        constexpr bool ENABLE_SPATIAL_WARP     = false;
    }
}
