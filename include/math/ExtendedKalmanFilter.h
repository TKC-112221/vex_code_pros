#pragma once
/* ---------------------------------------------------------------------------
 * ExtendedKalmanFilter  --  9-state diff-drive dynamics filter.
 * state = [x, y, theta, v, omega, a_act, alpha_act, a_cmd, alpha_cmd]
 *
 * Reusable math component (ported verbatim from v1's lib_control EKF). It fuses
 * pose + velocity measurements with commanded acceleration to produce smooth
 * velocity/derivative estimates. Used internally by CascadedController (for
 * feedforward + the PID D-term) and available to KalmanEstimator.
 * ------------------------------------------------------------------------- */

class ExtendedKalmanFilter {
public:
    static constexpr int N = 9;
    double x[9];      // state vector
    double P[9][9];   // covariance

    void init(double dt_sec, double proc_pos, double proc_vel, double proc_acc,
              double meas_pos, double meas_vel, double meas_acc_cmd);
    void reset();
    void reset(double x0, double y0, double theta0, double v0, double omega0);

    void predict(double dt, double a_cmd_input, double alpha_cmd_input);
    void update_pose(double meas_x, double meas_y, double meas_theta);
    void update_velocity(double meas_v, double meas_omega);
    void update_accel_cmd(double raw_a_cmd, double raw_alpha_cmd);
    // Scalar measurement y = H x. innov is (z - h(x)). Used for wall ranges.
    void update_linear(const double H[N], double innov, double r_var);

    double get_v_est() const        { return x[3]; }
    double get_omega_est() const    { return x[4]; }
    double get_a_act_est() const    { return x[5]; }
    double get_alpha_act_est() const{ return x[6]; }
    double get_a_cmd_est() const    { return x[7]; }
    double get_alpha_cmd_est() const{ return x[8]; }
    double get_deriv_lin() const    { return x[7] - x[5]; }
    double get_deriv_ang() const    { return x[8] - x[6]; }
    void get_pose_est(double& out_x, double& out_y, double& out_theta) const {
        out_x = x[0]; out_y = x[1]; out_theta = x[2];
    }

private:
    double dt_ = 0.01;
    double q_pos_ = 0, q_vel_ = 0, q_acc_ = 0;
    double r_pos_ = 0, r_vel_ = 0, r_acc_cmd_ = 0;
};
