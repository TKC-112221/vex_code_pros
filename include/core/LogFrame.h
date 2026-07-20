#pragma once
/* ---------------------------------------------------------------------------
 * LogFrame.h  --  The on-disk / in-memory recording schema.
 *
 * This is intentionally kept byte-for-byte compatible with v1's LogFrame so
 * the auto-generated data headers (data/recorded_path.h, data/skills_auto_
 * recorded.h) construct without modification and so exported CSVs keep the
 * same columns. It is a *storage* record, not a pipeline currency -- the
 * TrajectoryManager translates between LogFrame and the drivebase-agnostic
 * MotionTarget / ChassisObservation types.
 * ------------------------------------------------------------------------- */

#include <cstdint>

struct LogFrame {
    LogFrame() = default;
    // Positional constructor (C++11/14 friendly; matches v1 exactly).
    LogFrame(double t, double x_, double y_, double th, double vr, double or_, double ar, double alr,
             double jf, double jt, double rlv, double rrv, double lpd, double rpd, double bv, uint8_t sub)
        : time(t), x(x_), y(y_), theta(th), v_ref(vr), omega_ref(or_), a_ref(ar), alpha_ref(alr),
          joy_fwd_cmd(jf), joy_turn_cmd(jt), req_l_volts(rlv), req_r_volts(rrv),
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
    double joy_fwd_cmd = 0;
    double joy_turn_cmd = 0;
    double req_l_volts = 0;
    double req_r_volts = 0;
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
