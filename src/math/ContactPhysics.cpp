/* ---------------------------------------------------------------------------
 * ContactPhysics.cpp  --  wall contact, then a square-up onto a goal.
 * The goal snap is the last write in each slice, in every drive mode.
 * ------------------------------------------------------------------------- */
#include "math/ContactPhysics.h"
#include "core/Specs.h"
#include "core/Types.h"
#include "hardware/MockDriveParams.h"
#include "logic/GoalAligner.h"
#include "math/FrontEdge.h"
#include "math/MathUtils.h"

#include <cmath>

namespace {

double inertiaKgM2() {
    const double length = Robot::Chassis::BODY_LENGTH_M;
    const double width = Robot::Chassis::TRACK_WIDTH_M;
    return MockPlant::MASS_KG * (length * length + width * width) / 12.0;
}

void pointVelocity(const ContactBody& body, double px, double py, double& vpx, double& vpy) {
    const double c = std::cos(body.theta);
    const double s = std::sin(body.theta);
    const double vx = body.v_fwd * c - body.v_lat * s;
    const double vy = body.v_fwd * s + body.v_lat * c;
    const double rx = px - body.x_m;
    const double ry = py - body.y_m;
    vpx = vx - body.omega * ry;
    vpy = vy + body.omega * rx;
}

// Integrated wall force at one corner. j is the impulse along the inward
// normal: it is the force that removes this corner's into-wall velocity
// and nothing more, so the bumper does not throw the chassis back out.
void applyNormalImpulse(ContactBody& body, double px, double py,
                        double nx, double ny, double j) {
    const double rx = px - body.x_m;
    const double ry = py - body.y_m;
    const double ix = j * nx;
    const double iy = j * ny;
    const double dvx = ix / MockPlant::MASS_KG;
    const double dvy = iy / MockPlant::MASS_KG;
    const double c = std::cos(body.theta);
    const double s = std::sin(body.theta);
    body.v_fwd += dvx * c + dvy * s;
    body.v_lat += -dvx * s + dvy * c;
    body.omega += (rx * iy - ry * ix) / inertiaKgM2();
}

// Returns the body-forward part of the separation nudge. The caller adds it
// to the travel already integrated, so encoders match the pose that remains
// inside the wall.
double pushCorner(ContactBody& body, double px, double py, double pen,
                  double nx, double ny) {
    if (pen <= 0.0) return 0.0;

    double vpx = 0.0;
    double vpy = 0.0;
    pointVelocity(body, px, py, vpx, vpy);
    const double v_n = vpx * nx + vpy * ny;
    if (v_n < 0.0) {
        const double rx = px - body.x_m;
        const double ry = py - body.y_m;
        const double rn = rx * ny - ry * nx;
        const double inertia = inertiaKgM2();
        const double denom = 1.0 / MockPlant::MASS_KG + (rn * rn) / inertia;
        if (denom > 1e-9) applyNormalImpulse(body, px, py, nx, ny, -v_n / denom);
    }

    // Bleed the overlap out along the same normal. Most of it, not all, so
    // a corner shared by two walls does not pop through the other one.
    constexpr double kSlop = 0.001;
    if (pen <= kSlop) return 0.0;
    const double corr = 0.65 * (pen - kSlop);
    body.x_m += nx * corr;
    body.y_m += ny * corr;
    const double c = std::cos(body.theta);
    const double s = std::sin(body.theta);
    return (nx * corr) * c + (ny * corr) * s;
}

double wallImpulses(ContactBody& body) {
    const double half_l = Robot::Chassis::BODY_LENGTH_M * 0.5;
    const double half_w = Robot::Chassis::TRACK_WIDTH_M * 0.5;
    const double x_min = Robot::Range::FIELD_X_MIN_M;
    const double x_max = Robot::Range::FIELD_X_MAX_M;
    const double y_min = Robot::Range::FIELD_Y_MIN_M;
    const double y_max = Robot::Range::FIELD_Y_MAX_M;
    const double bx[4] = {half_l, half_l, -half_l, -half_l};
    const double by[4] = {half_w, -half_w, -half_w, half_w};
    double forward_nudge = 0.0;

    // Two passes: the second corner of a flush side sees the velocity the
    // first corner already changed.
    for (int pass = 0; pass < 2; ++pass) {
        const double c = std::cos(body.theta);
        const double s = std::sin(body.theta);
        for (int i = 0; i < 4; ++i) {
            // Recompute after every push. A correction moves the center, and
            // the next wall has to see the corner where it is now.
            auto hit = [&](double nx, double ny) {
                const double px = body.x_m + bx[i] * c - by[i] * s;
                const double py = body.y_m + bx[i] * s + by[i] * c;
                double pen = y_min - py;
                if (nx < 0.0) pen = px - x_max;
                else if (nx > 0.0) pen = x_min - px;
                else if (ny < 0.0) pen = py - y_max;
                forward_nudge += pushCorner(body, px, py, pen, nx, ny);
            };
            hit(-1.0, 0.0);
            hit(1.0, 0.0);
            hit(0.0, -1.0);
            hit(0.0, 1.0);
        }
    }
    return forward_nudge;
}

// Rotate the center about the goal by `yaw`, and turn the heading with it.
void swingAboutGoal(ContactBody& body, double gx, double gy, double yaw) {
    const double dx = body.x_m - gx;
    const double dy = body.y_m - gy;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    body.x_m = gx + dx * c - dy * s;
    body.y_m = gy + dx * s + dy * c;
    body.theta += yaw;
}

// Square the nose onto the goal and orbit it at the commanded yaw rate.
// This replaces the free slice, so the point wins over the drive command
// and over the wall push.
void squareOrOrbit(ContactBody& body, double h, ContactStep& stepped,
                   double speed_limit, double commanded_omega,
                   double gx, double gy) {
    const double half_l = Robot::Chassis::BODY_LENGTH_M * 0.5;
    const frontedge::CenterWish wish = frontedge::centerWish(
        body.x_m, body.y_m, body.theta, gx, gy, speed_limit);

    double pull_fwd = 0.0;
    double pull_yaw = 0.0;
    if (!wish.settled) {
        const double yaw = wish.omega * h;
        const double mid = body.theta + 0.5 * yaw;
        const double c = std::cos(mid);
        const double s = std::sin(mid);
        body.x_m += (wish.v_fwd * c - wish.v_lat * s) * h;
        body.y_m += (wish.v_fwd * s + wish.v_lat * c) * h;
        body.theta += yaw;
        pull_fwd = wish.v_fwd * h;
        pull_yaw = yaw;
    } else {
        const double dist = std::hypot(gx - body.x_m, gy - body.y_m);
        const double theta_face = (dist > 1e-6)
            ? std::atan2(gy - body.y_m, gx - body.x_m) : body.theta;
        double x_face = 0.0;
        double y_face = 0.0;
        frontedge::placeCenter(x_face, y_face, theta_face, gx, gy, 0.0);
        const double c0 = std::cos(body.theta);
        const double s0 = std::sin(body.theta);
        pull_fwd = (x_face - body.x_m) * c0 + (y_face - body.y_m) * s0;
        pull_yaw = mathutil::normalize_angle(theta_face - body.theta);
        body.theta = theta_face;
        body.x_m = x_face;
        body.y_m = y_face;
    }

    const double yaw = commanded_omega * h;
    swingAboutGoal(body, gx, gy, yaw);
    if (wish.settled) {
        frontedge::placeCenter(body.x_m, body.y_m, body.theta, gx, gy, 0.0);
    }

    body.omega = commanded_omega;
    body.v_fwd = 0.0;
    body.v_lat = -commanded_omega * half_l;
    stepped.forward_m += pull_fwd;
    stepped.yaw_rad += pull_yaw + yaw;
}

void clampMotion(ContactBody& body, double limit) {
    if (limit < 1e-6) return;
    const double half = Robot::Chassis::TRACK_WIDTH_M * 0.5;
    body.v_lat = mathutil::clamp_val(body.v_lat, -limit, limit);
    body.v_fwd = mathutil::clamp_val(body.v_fwd, -limit, limit);
    if (half < 1e-6) return;
    const double wmax = limit / half;
    body.omega = mathutil::clamp_val(body.omega, -wmax, wmax);
    const double wheel = std::fabs(body.v_fwd) + std::fabs(body.omega) * half;
    if (wheel > limit) {
        const double scale = limit / wheel;
        body.v_fwd *= scale;
        body.omega *= scale;
    }
}

void bleedSkid(ContactBody& body, double dt, double tau, double limit) {
    if (tau < 1e-4) {
        body.v_lat = 0.0;
        return;
    }
    const double decay = std::exp(-dt / tau);
    const double lat = body.v_lat * decay
        + (-body.omega * body.v_fwd) * tau * (1.0 - decay);
    body.v_lat = mathutil::clamp_val(lat, -limit, limit);
}

// One free slice: roller skid, commanded body motion, then the walls.
void integrateSlice(ContactBody& body, double h, ContactStep& stepped,
                    double skid_tau, double speed_limit) {
    const double lat0 = body.v_lat;
    bleedSkid(body, h, skid_tau, speed_limit);
    const double lat_avg = 0.5 * (lat0 + body.v_lat);

    const double yaw = body.omega * h;
    const double mid = body.theta + 0.5 * yaw;
    const double c = std::cos(mid);
    const double s = std::sin(mid);
    body.x_m += body.v_fwd * h * c - lat_avg * h * s;
    body.y_m += body.v_fwd * h * s + lat_avg * h * c;
    body.theta += yaw;
    stepped.forward_m += body.v_fwd * h;
    stepped.yaw_rad += yaw;
    stepped.forward_m += wallImpulses(body);
}

// Translate the center until every corner is inside. A chassis this size
// fits the field, so a handful of passes settles. Velocity aimed through a
// wall that we had to leave is dropped; the heading is left alone.
void separateWalls(ContactBody& body) {
    const double x0 = body.x_m;
    const double y0 = body.y_m;
    const double half_l = Robot::Chassis::BODY_LENGTH_M * 0.5;
    const double half_w = Robot::Chassis::TRACK_WIDTH_M * 0.5;
    const double x_min = Robot::Range::FIELD_X_MIN_M;
    const double x_max = Robot::Range::FIELD_X_MAX_M;
    const double y_min = Robot::Range::FIELD_Y_MIN_M;
    const double y_max = Robot::Range::FIELD_Y_MAX_M;
    const double bx[4] = {half_l, half_l, -half_l, -half_l};
    const double by[4] = {half_w, -half_w, -half_w, half_w};

    for (int pass = 0; pass < 8; ++pass) {
        bool outside = false;
        const double c = std::cos(body.theta);
        const double s = std::sin(body.theta);
        for (int i = 0; i < 4; ++i) {
            const double px = body.x_m + bx[i] * c - by[i] * s;
            const double py = body.y_m + bx[i] * s + by[i] * c;
            if (px > x_max) { body.x_m += x_max - px; outside = true; }
            else if (px < x_min) { body.x_m += x_min - px; outside = true; }
            if (py > y_max) { body.y_m += y_max - py; outside = true; }
            else if (py < y_min) { body.y_m += y_min - py; outside = true; }
        }
        if (!outside) break;
    }

    const double push_x = body.x_m - x0;
    const double push_y = body.y_m - y0;
    if (push_x == 0.0 && push_y == 0.0) return;

    const double c = std::cos(body.theta);
    const double s = std::sin(body.theta);
    double vx = body.v_fwd * c - body.v_lat * s;
    double vy = body.v_fwd * s + body.v_lat * c;
    if (push_x < 0.0 && vx > 0.0) vx = 0.0;
    if (push_x > 0.0 && vx < 0.0) vx = 0.0;
    if (push_y < 0.0 && vy > 0.0) vy = 0.0;
    if (push_y > 0.0 && vy < 0.0) vy = 0.0;
    body.v_fwd = vx * c + vy * s;
    body.v_lat = -vx * s + vy * c;
}

} // namespace

void holdWallContact(ContactBody& body) {
    wallImpulses(body);
    separateWalls(body);
}

ContactStep stepContacts(ContactBody& body, double dt,
                         double skid_tau_sec, double speed_limit_mps) {
    ContactStep stepped;
    if (!(dt > 0.0)) {
        separateWalls(body);
        goalAlignPhysicsDone();
        return stepped;
    }

    const int substeps = (MockPlant::CONTACT_SUBSTEPS > 1) ? MockPlant::CONTACT_SUBSTEPS : 1;
    const double h = dt / static_cast<double>(substeps);
    const double loose = speed_limit_mps * 2.0;
    // Wheel speed at the start of the step. The snap rewrites body speed,
    // and that rewrite must not look like a reverse release mid-step.
    const double commanded_omega = body.omega;
    const double commanded_fwd = body.v_fwd;

    for (int i = 0; i < substeps; ++i) {
        ContactBody moved = body;
        ContactStep slice;
        integrateSlice(moved, h, slice, skid_tau_sec, speed_limit_mps);

        double gx = 0.0;
        double gy = 0.0;
        if (goalAlignGrab(moved.x_m, moved.y_m, moved.theta, commanded_fwd, gx, gy)) {
            // Free motion already ran, including the walls. The snap is the
            // pose that remains.
            squareOrOrbit(body, h, stepped, loose, commanded_omega, gx, gy);
        } else {
            body = moved;
            stepped.forward_m += slice.forward_m;
            stepped.yaw_rad += slice.yaw_rad;
        }
        clampMotion(body, loose);
    }
    clampMotion(body, speed_limit_mps);
    goalAlignPhysicsDone();
    return stepped;
}
