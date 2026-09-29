#pragma once
/* ---------------------------------------------------------------------------
 * LogFrame.h  --  The on-disk / in-memory recording schema.
 *
 * Chassis-frame storage: command intent (surge/sway/yaw) and applied effort
 * (linear/lateral/angular). Drivebase-specific wheel mixes belong only inside
 * a concrete IDriveBase, never in the log.
 *
 * The positional constructor still accepts the legacy 16-arg tank CSV shape
 * (joy_fwd, joy_turn, req_l, req_r) used by generated headers: it maps sticks
 * to surge/yaw and converts left/right volts into linear/angular (lateral=0).
 * ------------------------------------------------------------------------- */

#include <cstdint>

struct LogFrame {
    LogFrame() = default;
    // Legacy positional constructor (matches v1 / generated data headers).
    // jf/jt were shaped sticks; rlv/rrv were tank left/right volts.
    LogFrame(double t, double x_, double y_, double th, double vr, double or_, double ar, double alr,
             double jf, double jt, double rlv, double rrv, double lpd, double rpd, double bv, uint8_t sub)
        : time(t), x(x_), y(y_), theta(th), v_ref(vr), omega_ref(or_), a_ref(ar), alpha_ref(alr),
          cmd_surge(jf), cmd_sway(0.0), cmd_yaw(jt),
          applied_linear((rlv + rrv) / 2.0),
          applied_lateral(0.0),
          applied_angular((rrv - rlv) / 2.0),
          l_pos_deg(lpd), r_pos_deg(rpd), battery_voltage(bv), subsystems(sub) {}

    double time = 0;
    double x = 0;
    double y = 0;
    double theta = 0;
    double v_ref = 0;
    double omega_ref = 0;
    double a_ref = 0;
    double alpha_ref = 0;
    double lin_error = 0;
    double ang_error = 0;
    double pid_lin_out = 0;
    double pid_ang_out = 0;
    double actual_x = 0;
    double actual_y = 0;
    double actual_theta = 0;

    // Pre-actuation command vector (chassis frame).
    double cmd_surge = 0;
    double cmd_sway  = 0;
    double cmd_yaw   = 0;

    // Post-safety applied chassis effort (what replay should feed forward).
    double applied_linear  = 0;
    double applied_lateral = 0;
    double applied_angular = 0;

    double l_pos_deg = 0;
    double r_pos_deg = 0;
    double l_vel_rpm = 0;
    double r_vel_rpm = 0;
    double l_current_amps = 0;
    double r_current_amps = 0;
    double battery_voltage = 0;
    double imu_accel_x = 0;
    double imu_accel_y = 0;
    double imu_heading = 0;
    double imu_rate_z = 0;
    uint8_t subsystems = 0;
};
