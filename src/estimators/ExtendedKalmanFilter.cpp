/* ExtendedKalmanFilter.cpp -- ported verbatim from v1 lib_control.cpp (EKF). */
#include "math/ExtendedKalmanFilter.h"
#include <cmath>

static double normalize_angle_ekf(double a) {
    while (a > 3.141592653589793)  a -= 6.283185307179586;
    while (a < -3.141592653589793) a += 6.283185307179586;
    return a;
}

void ExtendedKalmanFilter::init(double dt_sec, double proc_pos, double proc_vel, double proc_acc,
                                double meas_pos, double meas_vel, double meas_acc_cmd) {
    dt_ = dt_sec;
    q_pos_ = proc_pos; q_vel_ = proc_vel; q_acc_ = proc_acc;
    r_pos_ = meas_pos; r_vel_ = meas_vel; r_acc_cmd_ = meas_acc_cmd;
    reset();
}

void ExtendedKalmanFilter::reset() {
    for (int i = 0; i < N; ++i) x[i] = 0.0;
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            P[i][j] = (i == j) ? 1.0 : 0.0;
}

void ExtendedKalmanFilter::reset(double x0, double y0, double theta0, double v0, double omega0) {
    x[0] = x0; x[1] = y0; x[2] = theta0;
    x[3] = v0; x[4] = omega0;
    x[5] = 0.0; x[6] = 0.0; x[7] = 0.0; x[8] = 0.0;
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            P[i][j] = (i == j) ? 0.1 : 0.0;
    P[0][0] = P[1][1] = P[2][2] = 0.01;
    P[3][3] = P[4][4] = 0.05;
    P[5][5] = P[6][6] = P[7][7] = P[8][8] = 1.0;
}

void ExtendedKalmanFilter::predict(double dt, double a_cmd_input, double alpha_cmd_input) {
    double& px = x[0]; double& py = x[1]; double& ptheta = x[2];
    double& pv = x[3]; double& pomega = x[4];
    double& pa_act = x[5]; double& palpha_act = x[6];
    double& pa_cmd = x[7]; double& palpha_cmd = x[8];

    // Process model: diff-drive kinematics + accel as random walk toward command.
    double ct = cos(ptheta), st = sin(ptheta);
    double new_x = px + pv * ct * dt;
    double new_y = py + pv * st * dt;
    double new_theta = normalize_angle_ekf(ptheta + pomega * dt);
    double new_v = pv + pa_act * dt;
    double new_omega = pomega + palpha_act * dt;
    const double tau = 0.04;
    double alpha_lag = dt / (tau + dt);
    double new_a_act = pa_act + (pa_cmd - pa_act) * 0.2;
    double new_alpha_act = palpha_act + (palpha_cmd - palpha_act) * 0.2;
    double new_a_cmd = pa_cmd + alpha_lag * (a_cmd_input - pa_cmd);
    double new_alpha_cmd = palpha_cmd + alpha_lag * (alpha_cmd_input - palpha_cmd);

    // Jacobian F.
    double F[9][9];
    for (int i = 0; i < 9; ++i) for (int j = 0; j < 9; ++j) F[i][j] = (i == j) ? 1.0 : 0.0;
    F[0][2] = -pv * st * dt; F[0][3] = ct * dt;
    F[1][2] =  pv * ct * dt; F[1][3] = st * dt;
    F[2][4] = dt;
    F[3][5] = dt;
    F[4][6] = dt;
    F[5][7] = 0.2;  F[5][5] = 0.8;
    F[6][8] = 0.2;  F[6][6] = 0.8;
    F[7][7] = 1.0 - alpha_lag;
    F[8][8] = 1.0 - alpha_lag;

    double Q[9] = { q_pos_, q_pos_, q_pos_ * 0.5, q_vel_, q_vel_, q_acc_, q_acc_, q_acc_ * 2.0, q_acc_ * 2.0 };

    // P = F*P*F' + Q
    double PF[9][9];
    for (int i = 0; i < 9; ++i)
        for (int j = 0; j < 9; ++j) {
            PF[i][j] = 0.0;
            for (int k = 0; k < 9; ++k) PF[i][j] += F[i][k] * P[k][j];
        }
    for (int i = 0; i < 9; ++i)
        for (int j = 0; j < 9; ++j) {
            P[i][j] = Q[i] * (i == j ? 1.0 : 0.0);
            for (int k = 0; k < 9; ++k) P[i][j] += F[i][k] * PF[k][j];
        }

    x[0] = new_x; x[1] = new_y; x[2] = new_theta;
    x[3] = new_v; x[4] = new_omega;
    x[5] = new_a_act; x[6] = new_alpha_act;
    x[7] = new_a_cmd; x[8] = new_alpha_cmd;
}

void ExtendedKalmanFilter::update_pose(double meas_x, double meas_y, double meas_theta) {
    double dtheta = normalize_angle_ekf(meas_theta - x[2]);
    double innov[3] = { meas_x - x[0], meas_y - x[1], dtheta };
    for (int m = 0; m < 3; ++m) {
        double S = P[m][m] + ((m < 2) ? r_pos_ : r_pos_ * 0.5);
        if (S < 1e-10) S = 1e-10;
        double K[9], P_col[9];
        for (int j = 0; j < 9; ++j) P_col[j] = P[j][m];
        for (int i = 0; i < 9; ++i) K[i] = P_col[i] / S;
        for (int i = 0; i < 9; ++i) x[i] += K[i] * innov[m];
        double P_row[9];
        for (int j = 0; j < 9; ++j) P_row[j] = P[m][j];
        for (int i = 0; i < 9; ++i)
            for (int j = 0; j < 9; ++j)
                P[i][j] -= K[i] * P_row[j];
    }
}

void ExtendedKalmanFilter::update_velocity(double meas_v, double meas_omega) {
    double innov[2] = { meas_v - x[3], meas_omega - x[4] };
    for (int m = 0; m < 2; ++m) {
        int idx = 3 + m;
        double S = P[idx][idx] + r_vel_;
        if (S < 1e-10) S = 1e-10;
        double K[9], P_col[9], P_row[9];
        for (int j = 0; j < 9; ++j) P_col[j] = P[j][idx];
        for (int i = 0; i < 9; ++i) K[i] = P_col[i] / S;
        for (int i = 0; i < 9; ++i) x[i] += K[i] * innov[m];
        for (int j = 0; j < 9; ++j) P_row[j] = P[idx][j];
        for (int i = 0; i < 9; ++i)
            for (int j = 0; j < 9; ++j)
                P[i][j] -= K[i] * P_row[j];
    }
}

void ExtendedKalmanFilter::update_accel_cmd(double raw_a_cmd, double raw_alpha_cmd) {
    double innov[2] = { raw_a_cmd - x[7], raw_alpha_cmd - x[8] };
    for (int m = 0; m < 2; ++m) {
        int idx = 7 + m;
        double S = P[idx][idx] + r_acc_cmd_;
        if (S < 1e-10) S = 1e-10;
        double K[9], P_col[9], P_row[9];
        for (int j = 0; j < 9; ++j) P_col[j] = P[j][idx];
        for (int i = 0; i < 9; ++i) K[i] = P_col[i] / S;
        for (int i = 0; i < 9; ++i) x[i] += K[i] * innov[m];
        for (int j = 0; j < 9; ++j) P_row[j] = P[idx][j];
        for (int i = 0; i < 9; ++i)
            for (int j = 0; j < 9; ++j)
                P[i][j] -= K[i] * P_row[j];
    }
}
