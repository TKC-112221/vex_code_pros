#pragma once
/* ---------------------------------------------------------------------------
 * LogFrame.h  --  Unified on-disk / in-memory recording schema.
 *
 * One frame type serves trajectory replay and system identification. Command
 * intent (surge/sway/yaw) and applied effort (linear/lateral/angular) stay in
 * the chassis frame. Drivebase-specific wheel mixes stay inside the concrete
 * IDriveBase. The raw_* block adds the pre-filter sensor sample.
 *
 * Pose is the estimator pose at the tick (replay path + start pose). Velocity
 * channels used for replay are the denoised rates. The raw_* block is the
 * pre-filter sample from the same tick: heading, wheel speed, travel, and
 * the four distance beams. Encoder position, current, temperature, battery,
 * and IMU accel / yaw rate above are already unfiltered.
 * ------------------------------------------------------------------------- */

#include <cstdint>

struct LogFrame {
    LogFrame() = default;

    double time = 0;

    // Estimator pose (field frame, inches). Replay path and start pose.
    double x = 0;
    double y = 0;
    double theta = 0;

    // Denoised chassis rates this tick (replay velocity reference).
    double v_lin = 0;   // m/s
    double v_ang = 0;   // rad/s

    // Inertial chassis velocity from accelerometer integration (slip-aware).
    double v_inertial_fwd = 0;  // m/s, body forward
    double v_inertial_lat = 0;  // m/s, body left

    // Pre-actuation command (chassis frame, before drivebase safety).
    double cmd_surge = 0;
    double cmd_sway  = 0;
    double cmd_yaw   = 0;

    // Post-safety applied effort (plant input; replay feedforward).
    double applied_linear  = 0;
    double applied_lateral = 0;
    double applied_angular = 0;

    // Raw wheel-bank encoder positions (deg). Differentiate offline for speed.
    double l_pos_deg = 0;
    double r_pos_deg = 0;

    // Motor electrical + thermal (bank averages on multi-motor chassis).
    double l_current_amps = 0;
    double r_current_amps = 0;
    double l_temp_c = 0;
    double r_temp_c = 0;

    double battery_voltage = 0;

    // Raw IMU specific force (G) and yaw rate (deg/s), unfiltered.
    double imu_accel_x = 0;
    double imu_accel_y = 0;
    double imu_rate_z  = 0;

    uint8_t subsystems = 0;

    // Pre-denoiser chassis sample. v_lin / v_ang above stay the filtered
    // rates replay follows; these keep the sensor bandwidth.
    double raw_heading_rad = 0;
    double raw_v_lin = 0;          // m/s
    double raw_v_ang = 0;          // rad/s
    double raw_l_vel_rpm = 0;
    double raw_r_vel_rpm = 0;
    double raw_fwd_m = 0;          // wheel travel this tick, meters
    double raw_strafe_m = 0;

    // Distance beams D1..D4 before range smoothing. 0 m when invalid.
    double raw_range_m[4] = {};
    uint8_t raw_range_conf[4] = {};  // 0..63
    uint8_t raw_range_valid = 0;     // bit i set when D(i+1) returned
};
