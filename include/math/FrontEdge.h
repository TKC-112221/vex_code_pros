#pragma once
/* ---------------------------------------------------------------------------
 * FrontEdge  --  the bumper line the goal lock pivots on.
 *
 * Body frame: +x forward, +y left. The front edge sits half a body length
 * ahead of the center and spans the track. A goal point "touches" when it
 * is within GoalAlign::FRONT_TOUCH_M of that segment.
 * ------------------------------------------------------------------------- */

#include "core/Specs.h"
#include "core/Tuning.h"
#include "math/MathUtils.h"

#include <cmath>

namespace frontedge {

// contact_left_m is the body-left coordinate on the front edge, in meters.
inline bool touches(double x_m, double y_m, double theta,
                    double gx, double gy, double& contact_left_m) {
    const double half_l = Robot::Chassis::BODY_LENGTH_M * 0.5;
    const double half_w = Robot::Chassis::TRACK_WIDTH_M * 0.5;
    const double c = mathutil::heading_cos(theta);
    const double s = mathutil::heading_sin(theta);
    const double dx = gx - x_m;
    const double dy = gy - y_m;
    const double forward = dx * c + dy * s;
    const double left = -dx * s + dy * c;
    const double clamped = mathutil::clamp_val(left, -half_w, half_w);
    const double gap = std::hypot(forward - half_l, left - clamped);
    if (gap > GoalAlign::FRONT_TOUCH_M) return false;
    contact_left_m = clamped;
    return true;
}

// Place the center so the locked front-edge point sits on the goal.
inline void placeCenter(double& x_m, double& y_m, double theta,
                        double gx, double gy, double contact_left_m) {
    const double half_l = Robot::Chassis::BODY_LENGTH_M * 0.5;
    const double c = mathutil::heading_cos(theta);
    const double s = mathutil::heading_sin(theta);
    x_m = gx - half_l * c + contact_left_m * s;
    y_m = gy - half_l * s - contact_left_m * c;
}

// Speeds that put the middle of the front edge on the goal and point the
// heading straight at it. Settled means the pose is already there.
struct CenterWish {
    double v_fwd = 0;
    double v_lat = 0;
    double omega = 0;
    bool settled = false;
};

inline CenterWish centerWish(double x_m, double y_m, double theta,
                             double gx, double gy, double speed_limit) {
    CenterWish wish;
    const double dxg = gx - x_m;
    const double dyg = gy - y_m;
    const double dist = std::hypot(dxg, dyg);
    double theta_face = theta;
    if (dist > 1e-6) theta_face = mathutil::direction_heading(dxg, dyg);
    double x_face = 0.0;
    double y_face = 0.0;
    placeCenter(x_face, y_face, theta_face, gx, gy, 0.0);

    const double e_th = mathutil::normalize_angle(theta_face - theta);
    const double ex = x_face - x_m;
    const double ey = y_face - y_m;
    if (std::fabs(e_th) <= GoalAlign::CENTER_HEAD_RAD
        && std::hypot(ex, ey) <= GoalAlign::CENTER_POS_M) {
        wish.settled = true;
        return wish;
    }

    const double c = mathutil::heading_cos(theta);
    const double s = mathutil::heading_sin(theta);
    const double inv = 1.0 / GoalAlign::CENTER_TAU_SEC;
    const double lim = (speed_limit > 1e-3) ? speed_limit : 1.0e9;
    wish.v_fwd = mathutil::clamp_val((ex * c + ey * s) * inv, -lim, lim);
    wish.v_lat = mathutil::clamp_val((-ex * s + ey * c) * inv, -lim, lim);
    wish.omega = mathutil::clamp_val(e_th * inv,
                                     -GoalAlign::MAX_YAW_RAD_S,
                                     GoalAlign::MAX_YAW_RAD_S);
    return wish;
}

} // namespace frontedge
