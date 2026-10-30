#include "auton/DriveTo.h"
#include "control/SegmentFollower.h"
#include "hardware/ChassisVoltage.h"
#include "control/VASController.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include "field/FieldPoints.h"
#include "interfaces/IFeedForward.h"
#include "math/MathUtils.h"
#include "math/TrackingCenter.h"
#include <cmath>
#include <cstdio>

static int g_fails = 0;

// Field heading is 0 along +y. East is +x, west is -x.
constexpr double kEast = -M_PI / 2.0;
constexpr double kWest = M_PI / 2.0;

static void expect(bool ok, const char* msg) {
    if (ok) return;
    std::printf("FAIL %s\n", msg);
    ++g_fails;
}

// The field line is the rotation centre. A command pose is the bumper.
static Pose2d centreOf(const Pose2d& tracked_in) {
    return poseMetersToInches(tracking::centerFromTracking(poseInchesToMeters(tracked_in)));
}

static double gapTo(double ax, double ay, double bx, double by, double cx, double cy) {
    const double abx = bx - ax;
    const double aby = by - ay;
    const double len2 = abx * abx + aby * aby;
    double t = 0.0;
    if (len2 > 1e-12) {
        t = ((cx - ax) * abx + (cy - ay) * aby) / len2;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
    }
    return std::hypot(cx - (ax + t * abx), cy - (ay + t * aby));
}

static void testStraight() {
    SegmentFollower nav;
    nav.target.x = metersToInches(1.5);
    nav.target.y = metersToInches(0.4);
    nav.target.exit_direction = kEast;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 0.0;
    nav.reset();

    const Pose2d from{0.0, 0.0, kEast};
    Pose2d path[16];
    const int n = nav.samplePath(from, path, 16);
    expect(n == 2, "open field is one segment");
    const Pose2d start = centreOf(from);
    const Pose2d end = centreOf(Pose2d{nav.target.x, nav.target.y, kEast});
    expect(std::hypot(path[0].x - start.x, path[0].y - start.y) < 1e-4,
           "preview starts at the rotation centre");
    expect(std::fabs(path[n - 1].x - end.x) < 1e-4, "preview ends on the rotation centre x");
    expect(std::fabs(path[n - 1].y - end.y) < 1e-4, "preview ends on the rotation centre y");

    StateEstimate st;
    st.pose = from;
    nav.compute(st);
    Pose2d again[16];
    const int n2 = nav.samplePath(Pose2d{10.0, 10.0, 0.0}, again, 16);
    expect(n2 == 2, "committed plan stays one segment");
    expect(std::hypot(again[0].x - start.x, again[0].y - start.y) < 1e-4,
           "a later pose does not rebuild the plan");
    const double want = mathutil::direction_heading(nav.target.x, nav.target.y);
    const double heading = mathutil::normalize_angle(nav.goalHeading() - want);
    expect(std::fabs(heading) < 0.05, "on the segment the nose faces along it");
}

static void testKeepOut() {
    SegmentFollower nav;
    nav.target.x = metersToInches(2.0);
    nav.target.y = 0.0;
    nav.target.exit_direction = kEast;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 0.0;
    nav.config.obstacle_n = 1;
    nav.config.obstacles[0] = {metersToInches(1.0), 0.0, metersToInches(0.30)};
    nav.reset();

    Pose2d path[40];
    const int n = nav.samplePath(Pose2d{0.0, 0.0, kEast}, path, 40);
    expect(n > 2, "a blocking circle adds vertices");
    const double cx = metersToInches(1.0);
    const double radius = metersToInches(segmentKeepRadiusM(0.30));
    const double lever = metersToInches(Robot::Chassis::TRACKING_FORWARD_M);
    double side = 0.0;
    for (int i = 0; i < n; ++i) {
        const double d = std::hypot(path[i].x - cx, path[i].y);
        expect(d >= radius - 0.05, "vertex stays outside the keep-out");
        // A corner pivot swings the bumper around the centre by the lever.
        if (i > 0 && i < n - 1)
            expect(d >= radius + lever - 0.05, "a pivot keeps the bumper outside the keep-out");
        if (std::fabs(path[i].y) > side) side = std::fabs(path[i].y);
        if (i == 0) continue;
        const double gap = gapTo(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y, cx, 0.0);
        expect(gap >= radius - 0.05, "segment stays outside the keep-out");
    }
    expect(side > metersToInches(0.15), "the polyline leaves the straight line");
}

static void testFieldBound() {
    SegmentFollower nav;
    nav.config.field_bound = true;
    nav.target.x = metersToInches(1.0);
    nav.target.y = metersToInches(0.25);
    nav.target.exit_direction = kEast;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 0.0;
    nav.config.obstacle_n = 1;
    nav.config.obstacles[0] = {0.0, metersToInches(0.25), metersToInches(0.35)};
    nav.reset();

    const Pose2d from{metersToInches(-1.0), metersToInches(0.25), kEast};
    Pose2d path[40];
    const int n = nav.samplePath(from, path, 40);
    expect(n > 2, "field-bounded detour has vertices");
    double top = from.y;
    for (int i = 0; i < n; ++i) {
        expect(path[i].y > -1.0, "detour stays inside the field");
        if (path[i].y > top) top = path[i].y;
    }
    expect(top > from.y + metersToInches(0.15), "the detour goes above the circle");
}

static void testReverseFinish() {
    SegmentFollower nav;
    nav.target.x = metersToInches(1.0);
    nav.target.y = 0.0;
    nav.target.exit_direction = kWest;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = metersToInches(0.30);
    nav.reset();

    Pose2d path[8];
    const int n = nav.samplePath(Pose2d{0.0, 0.0, kWest}, path, 8);
    expect(n == 3, "a reversed finish keeps the join");
    expect(path[1].x > nav.target.x + metersToInches(0.2), "the join sits past the target");
    const Pose2d finish = centreOf(Pose2d{nav.target.x, nav.target.y, kWest});
    expect(std::fabs(path[n - 1].x - finish.x) < 1e-3, "the finish ends on the rotation centre");

    StateEstimate st;
    st.pose = {0.0, 0.0, kWest};
    nav.compute(st);
    const double err = std::fabs(mathutil::normalize_angle(nav.goalHeading() - kWest));
    expect(err < 0.15, "the nose stays put across the reversal");

    SegmentFollower turned;
    turned.target = nav.target;
    StateEstimate face;
    face.pose = {0.0, 0.0, kEast};
    turned.compute(face);
    const double spin = std::fabs(mathutil::normalize_angle(turned.goalHeading() - kWest));
    expect(spin < 0.15, "a large nose error pivots before driving");
}

static void testForwardsFlag() {
    SegmentFollower nav;
    nav.config.forwards = false;
    nav.target.x = metersToInches(1.0);
    nav.target.y = 0.0;
    nav.target.exit_direction = kEast;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 0.0;
    StateEstimate st;
    st.pose = {0.0, 0.0, kEast};
    nav.compute(st);
    const double err = std::fabs(mathutil::normalize_angle(nav.goalHeading() - kWest));
    expect(err < 0.15, "config.forwards false faces the nose opposite travel");
    expect(nav.achievableLateral() == 0.0, "a straight segment has no lateral accel");

    Pose2d path[8];
    const int n = nav.sampleBase(Pose2d{0.0, 0.0, kEast}, path, 8);
    expect(n == 2, "sampleBase returns the planned polyline");
}

static void testStanleySide() {
    SegmentFollower nav;
    nav.target.x = metersToInches(2.0);
    nav.target.y = 0.0;
    nav.target.exit_direction = kEast;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 0.0;
    StateEstimate on;
    on.pose = {0.0, 0.0, kEast};
    nav.compute(on);

    StateEstimate off;
    off.pose = {metersToInches(0.5), metersToInches(0.20), kEast};
    nav.compute(off);
    expect(nav.goalHeading() < kEast - 0.2, "left of the segment, the nose turns back toward it");
}

static void testGoalAvoid() {
    SegmentFollower nav;
    nav.target.x = 60.0;
    nav.target.y = 24.0;
    nav.target.exit_direction = kEast;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 0.0;
    nav.config.obstacle_n = Field::goalAvoidObstacles(nav.config.obstacles, kPathObstacles);
    expect(nav.config.obstacle_n == 9, "every align goal is a keep-out");

    Pose2d path[40];
    const Pose2d from{-60.0, 24.0, kEast};
    const int n = nav.samplePath(from, path, 40);
    expect(n > 2, "goals on the straight line force a detour");
    const double radius = Field::goalAvoidIn();
    const double lever = metersToInches(Robot::Chassis::TRACKING_FORWARD_M);
    for (int g = 0; g < static_cast<int>(Field::Point::Count); ++g) {
        const Field::Spot& spot = Field::kSpots[g];
        if (!Field::isAlignGoal(spot)) continue;
        for (int i = 1; i < n; ++i) {
            const double gap = gapTo(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y,
                                     spot.x, spot.y);
            expect(gap >= radius - 0.05, "auto drive stays off the goal point");
            if (i < n - 1) {
                const double d = std::hypot(path[i].x - spot.x, path[i].y - spot.y);
                expect(d >= radius + lever - 0.05,
                       "a pivot keeps the bumper off the goal point");
            }
        }
    }
    const Pose2d end = centreOf(Pose2d{60.0, 24.0, kEast});
    expect(std::hypot(path[n - 1].x - end.x, path[n - 1].y - end.y) < 1e-3,
           "the detour still ends on the rotation centre");
}

static void testStuck() {
    SegmentFollower nav;
    nav.target.x = metersToInches(1.5);
    nav.target.y = 0.0;
    nav.target.exit_direction = kEast;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 0.0;
    nav.reset();

    StateEstimate st;
    st.pose = Pose2d{0.0, 0.0, kEast};
    st.v_lin = 1.0;
    for (int i = 0; i < 300; ++i) nav.compute(st);
    expect(!nav.stuck(), "motion answers a drive");
    expect(!nav.settled(), "a moving leg is not done");

    nav.reset();
    st.v_lin = 0.0;
    st.v_ang = 0.0;
    int took = 0;
    for (; took < 300 && !nav.stuck(); ++took) nav.compute(st);
    const double sec = static_cast<double>(took) * System::DT_SEC;
    expect(nav.stuck(), "an unanswered drive gives up");
    expect(!nav.settled(), "giving up is not an arrival");
    expect(sec + 1e-9 >= Motion::STUCK_SEC, "it waits out the window");
    expect(sec < Motion::STUCK_SEC + 0.5, "it gives up once the window has passed");
}

// Records the longitudinal speed the follower asked the model for.
class SpeedTap : public IFeedForward {
public:
    mutable double lin_v = 0.0;
    mutable double lin_a = 0.0;
    mutable double yaw_w = 0.0;

    double linearVolts(double velocity, double accel, double /*abs_lin_vel*/,
                       double /*abs_lin_accel*/, double /*omega*/,
                       double turn_radius) const override {
        // The surge call uses the straight-line radius. The yaw call reuses
        // this with the half-track, and that velocity is a wheel speed.
        if (turn_radius > 1.0) {
            lin_v = velocity;
            lin_a = accel;
            yaw_w = 0.0;
        } else if (turn_radius > 1e-6 && std::fabs(velocity) > 1e-4) {
            // The static-friction probe calls this with a tiny wheel speed.
            // That is not a yaw command.
            yaw_w = velocity / turn_radius;
        }
        return 0.0;
    }

    double angularVolts(double /*omega*/, double /*alpha*/, double /*abs_lin_vel*/,
                        double /*abs_lin_accel*/, double /*turn_radius*/) const override {
        return 0.0;
    }
};

static void testNoOvershoot() {
    // The command is applied on the next tick, which is how far the old
    // brake curve ran past a stop. The lead should leave the stop on the point.
    SegmentFollower nav;
    nav.config.path_accel = 3.0;
    nav.config.cruise_speed = 1.2;
    nav.config.min_accel = 0.15;
    nav.target.x = metersToInches(1.5);
    nav.target.y = 0.0;
    nav.target.exit_direction = kEast;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 0.0;
    SpeedTap tap;
    nav.bindFeedforward(&tap);

    double x = 0.0;
    double v = 0.0;
    double held = 0.0;
    StateEstimate st;
    st.pose = {0.0, 0.0, kEast};
    bool finished = false;
    for (int i = 0; i < 800; ++i) {
        st.pose.x = metersToInches(x);
        st.pose.y = 0.0;
        st.pose.theta = kEast;
        st.v_lin = v;
        st.v_ang = 0.0;
        const ChassisCommand cmd = nav.compute(st);
        v = held;
        held = cmd.linear;
        x += v * System::DT_SEC * mathutil::heading_cos(kEast);
        if (nav.settled() && std::fabs(v) < 0.05) {
            finished = true;
            break;
        }
    }
    expect(finished, "a straight stop settles");
    expect(x < 1.5 + 0.01, "a straight stop does not run past the target");
    expect(x > 1.5 - 0.03, "a straight stop still reaches the target");
}

static void testPastEndStops() {
    // Once the centre is past the end, the gap to the point grows. That
    // distance must not become a faster command in the same direction.
    SegmentFollower nav;
    nav.config.path_accel = 3.0;
    nav.config.cruise_speed = 1.2;
    nav.config.min_accel = 0.15;
    nav.target.x = metersToInches(1.5);
    nav.target.y = 0.0;
    nav.target.exit_direction = kEast;
    nav.target.exit_speed = 0.0;
    SpeedTap tap;
    nav.bindFeedforward(&tap);

    StateEstimate st;
    st.pose = {0.0, 0.0, kEast};
    st.v_lin = 0.0;
    st.v_ang = 0.0;
    nav.compute(st);

    double x = 1.55;
    double v = 0.4;
    double held = 0.4;
    double max_x = x;
    bool stopped = false;
    for (int i = 0; i < 200; ++i) {
        st.pose.x = metersToInches(x);
        st.pose.y = 0.0;
        st.pose.theta = kEast;
        st.v_lin = v;
        st.v_ang = 0.0;
        const ChassisCommand cmd = nav.compute(st);
        v = held;
        held = cmd.linear;
        if (v < 0.0) v = 0.0;
        x += v * System::DT_SEC * mathutil::heading_cos(kEast);
        if (x > max_x) max_x = x;
        if (std::fabs(held) < 0.02 && std::fabs(v) < 0.05) {
            stopped = true;
            break;
        }
    }
    expect(max_x < 1.55 + 0.05, "past the target the chassis stops");
    expect(stopped, "past the target the speed comes off");
}

static void testHeavyStillStops() {
    // Heavier than the armature model, so the same volts brake less, and the
    // voltage is applied a tick late. The profile leaves accel unused, and
    // the correction may brake harder than the path accel.
    SegmentFollower nav;
    nav.config.path_accel = 3.0;
    nav.config.cruise_speed = 1.2;
    nav.config.min_accel = 0.15;
    nav.target.x = metersToInches(1.5);
    nav.target.y = 0.0;
    nav.target.exit_direction = kEast;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 0.0;
    SpeedTap tap;
    nav.bindFeedforward(&tap);

    const double ks = Feedforward::KS_LIN;
    const double kv = Feedforward::KV_LIN;
    const double ka = Feedforward::KA_LIN;
    const double ka_plant = ka * 1.6;
    const double rail = System::MAX_VOLTS;

    double x = 0.0;
    double v = 0.0;
    double v_cmd = 0.0;
    double a_cmd = 0.0;
    double hardest = 0.0;
    StateEstimate st;
    st.pose = {0.0, 0.0, kEast};
    bool finished = false;
    for (int i = 0; i < 800; ++i) {
        st.pose.x = metersToInches(x);
        st.pose.y = 0.0;
        st.pose.theta = kEast;
        st.v_lin = v;
        st.v_ang = 0.0;
        const ChassisCommand cmd = nav.compute(st);
        if (cmd.accel < hardest) hardest = cmd.accel;

        const double sgn = v_cmd > 1e-3 ? 1.0 : (v_cmd < -1e-3 ? -1.0 : 0.0);
        double volts = ks * sgn + kv * v_cmd + ka * a_cmd;
        if (volts > rail) volts = rail;
        if (volts < -rail) volts = -rail;
        const double sgn_v = v > 0.02 ? 1.0 : (v < -0.02 ? -1.0 : 0.0);
        const double a_act = (volts - ks * sgn_v - kv * v) / ka_plant;
        v += a_act * System::DT_SEC;
        x += v * System::DT_SEC * mathutil::heading_cos(kEast);
        v_cmd = cmd.linear;
        a_cmd = cmd.accel;
        if (nav.settled() && std::fabs(v) < 0.05) {
            finished = true;
            break;
        }
    }
    char where[80];
    std::snprintf(where, sizeof(where), "a heavy chassis stops on the point (x=%.3f)", x);
    expect(finished, "a heavy chassis settles");
    expect(x < 1.5 + 0.02, where);
    expect(x > 1.5 - 0.05, where);
    expect(hardest < -(nav.config.path_accel + 0.5),
           "braking is not capped at the path accel");
}

static void testJoinAndFinishStop() {
    // The join in front of the straight finish is a shallow stop. It has to
    // be at rest before that segment starts, and the segment itself has to
    // end at rest on the target.
    SegmentFollower nav;
    nav.config.path_accel = 1.5;
    nav.config.cruise_speed = 1.0;
    nav.config.min_accel = 0.15;
    nav.target.x = 60.0;
    nav.target.y = 0.0;
    nav.target.exit_direction = Robot::PI;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 12.0;
    nav.config.obstacle_n = Field::goalAvoidObstacles(nav.config.obstacles, kPathObstacles);
    SpeedTap tap;
    nav.bindFeedforward(&tap);

    const Pose2d from{24.0, 48.0, Robot::PI};
    Pose2d path[40];
    const int n = nav.samplePath(from, path, 40);
    expect(n >= 3, "the finish is its own segment");
    const double jx = inchesToMeters(path[n - 2].x);
    const double jy = inchesToMeters(path[n - 2].y);
    const double tx = inchesToMeters(path[n - 1].x);
    const double ty = inchesToMeters(path[n - 1].y);
    expect(std::hypot(jx - tx, jy - ty) > 0.2, "the straight finish has length");

    double x = inchesToMeters(from.x);
    double y = inchesToMeters(from.y);
    double th = from.theta;
    double v = 0.0;
    double join_slow = 1e9;
    bool saw_join = false;
    bool finished = false;
    StateEstimate st;
    for (int i = 0; i < 5000; ++i) {
        st.pose = {metersToInches(x), metersToInches(y), th};
        st.v_lin = v;
        st.v_ang = 0.0;
        const ChassisCommand cmd = nav.compute(st);
        const Pose2d body = tracking::centerFromTracking(Pose2d{x, y, th});
        if (std::hypot(body.x - jx, body.y - jy) < 0.06) {
            saw_join = true;
            if (std::fabs(v) < join_slow) join_slow = std::fabs(v);
        }
        const double turn = mathutil::normalize_angle(nav.goalHeading() - th);
        const double dth = mathutil::clamp_val(turn, -4.0 * System::DT_SEC, 4.0 * System::DT_SEC);
        // The bumper orbits the centre while the chassis yaws in place.
        const Pose2d center = tracking::centerFromTracking(Pose2d{x, y, th});
        th = mathutil::normalize_angle(th + dth);
        const Pose2d swung = tracking::trackingFromCenter(Pose2d{center.x, center.y, th});
        x = swung.x;
        y = swung.y;
        x += v * System::DT_SEC * mathutil::heading_cos(th);
        y += v * System::DT_SEC * mathutil::heading_sin(th);
        v = cmd.linear;
        if (nav.settled() && std::fabs(v) <= 0.05) {
            finished = true;
            break;
        }
    }
    const Pose2d body = tracking::centerFromTracking(Pose2d{x, y, th});
    expect(saw_join, "the chassis reaches the straight-segment entry");
    expect(join_slow <= 0.05, "the straight-segment entry is a full stop");
    expect(finished, "the path ends stopped");
    expect(std::hypot(body.x - tx, body.y - ty) < 0.02, "the path ends on the rotation centre");
}

static void testShortCornerStillTurns() {
    // A match load stopped 1.1 cm short of this join. The projection was
    // already on the vertex, so the turn never started. The centre has to
    // yaw onto the finish and end up on that segment.
    SegmentFollower nav;
    nav.config.path_accel = 1.5;
    nav.config.cruise_speed = 1.0;
    nav.config.min_accel = 0.15;
    nav.target.x = 60.0;
    nav.target.y = 0.0;
    nav.target.exit_direction = Robot::PI;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 12.0;
    nav.config.obstacle_n = Field::goalAvoidObstacles(nav.config.obstacles, kPathObstacles);
    SpeedTap tap;
    nav.bindFeedforward(&tap);

    const Pose2d from{24.0, 48.0, Robot::PI};
    Pose2d path[40];
    const int n = nav.samplePath(from, path, 40);
    expect(n >= 3, "the short-corner path has a join");
    const double jx = inchesToMeters(path[n - 2].x);
    const double jy = inchesToMeters(path[n - 2].y);
    const double finish = path[n - 2].theta;
    const double incoming = path[n - 3].theta;

    double x = inchesToMeters(from.x);
    double y = inchesToMeters(from.y);
    double th = from.theta;
    double v = 0.0;
    double w = 0.0;
    double v_hold = 0.0;
    double w_hold = 0.0;
    bool placed = false;
    bool on_finish = false;
    StateEstimate st;
    for (int i = 0; i < 5000; ++i) {
        const Pose2d centre_now = tracking::centerFromTracking(Pose2d{x, y, th});
        if (!placed && i > 20 && std::hypot(centre_now.x - jx, centre_now.y - jy) < 0.08) {
            const double dx = mathutil::heading_cos(finish);
            const double dy = mathutil::heading_sin(finish);
            const Pose2d shifted{jx - 0.011 * dx, jy - 0.011 * dy, incoming};
            const Pose2d bumper = tracking::trackingFromCenter(shifted);
            x = bumper.x;
            y = bumper.y;
            th = incoming;
            v = 0.0;
            w = 0.0;
            v_hold = 0.0;
            w_hold = 0.0;
            placed = true;
        }
        st.pose = {metersToInches(x), metersToInches(y), th};
        st.v_lin = v;
        st.v_ang = w;
        const ChassisCommand cmd = nav.compute(st);
        double v_app = v_hold;
        double w_app = w_hold;
        if (std::fabs(v_app) < 0.02 && std::fabs(v) < 0.02) v_app = 0.0;
        // A stopped chassis does not start turning on a trim. The standing
        // pivot is the command above the yaw stiction rate.
        if (std::fabs(v) < 0.02 && std::fabs(v_app) < 0.02
            && std::fabs(w_app) < 0.5 && std::fabs(w) < 0.08) {
            w_app = 0.0;
        }
        v = v_app;
        w = w_app;
        const Pose2d center = tracking::centerFromTracking(Pose2d{x, y, th});
        th = mathutil::normalize_angle(th + w * System::DT_SEC);
        const Pose2d swung = tracking::trackingFromCenter(Pose2d{center.x, center.y, th});
        x = swung.x;
        y = swung.y;
        x += v * System::DT_SEC * mathutil::heading_cos(th);
        y += v * System::DT_SEC * mathutil::heading_sin(th);
        v_hold = cmd.linear;
        w_hold = cmd.angular;
        if (!placed) continue;
        const double px = inchesToMeters(path[n - 2].x);
        const double py = inchesToMeters(path[n - 2].y);
        const double qx = inchesToMeters(path[n - 1].x);
        const double qy = inchesToMeters(path[n - 1].y);
        const Pose2d body = tracking::centerFromTracking(Pose2d{x, y, th});
        const double abx = qx - px;
        const double aby = qy - py;
        const double len2 = abx * abx + aby * aby;
        double t = 0.0;
        if (len2 > 1e-12) t = ((body.x - px) * abx + (body.y - py) * aby) / len2;
        const double fx = (t < 0.0) ? px : (t > 1.0) ? qx : px + t * abx;
        const double fy = (t < 0.0) ? py : (t > 1.0) ? qy : py + t * aby;
        const double gap = std::hypot(body.x - fx, body.y - fy);
        const bool head_ok = std::fabs(mathutil::normalize_angle(th - finish)) < 0.35;
        if (gap <= 0.01 && head_ok) {
            on_finish = true;
            break;
        }
    }
    expect(placed, "the chassis reaches the corner before the short placement");
    expect(on_finish, "a centre 1.1 cm short of the corner reaches the next segment");
}

static void testStoppedNoseSquaresUp() {
    // The score sat here: stopped a few centimetres past the standoff,
    // nose still tens of degrees off, and the follower asking for neither
    // a drive nor a pivot. The heading trim is under breakaway, so the
    // chassis never yaws and settled() never comes. That correction is a
    // pivot, and if the chassis still does not answer, the leg gives up.
    SegmentFollower nav;
    nav.target.x = metersToInches(1.0);
    nav.target.y = 0.0;
    nav.target.exit_direction = kEast;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 0.0;
    SpeedTap tap;
    nav.bindFeedforward(&tap);

    StateEstimate st;
    st.pose = {0.0, 0.0, kEast};
    st.v_lin = 0.0;
    st.v_ang = 0.0;
    nav.compute(st);

    const Pose2d goal = tracking::centerFromTracking(
        poseInchesToMeters(Pose2d{nav.target.x, nav.target.y, kEast}));
    // On the line, so the cross-track term does not itself exceed the
    // mid-segment pivot angle. Past the end, so the brake command is zero.
    const Pose2d centre{goal.x + 0.03, goal.y,
                        mathutil::normalize_angle(kEast + 0.5)};
    st.pose = poseMetersToInches(tracking::trackingFromCenter(centre));

    bool saw_pivot = false;
    int took = 0;
    for (; took < 300 && !nav.stuck(); ++took) {
        const ChassisCommand cmd = nav.compute(st);
        if (std::fabs(cmd.angular) + 1e-9 >= 0.5) saw_pivot = true;
    }
    const double sec = static_cast<double>(took) * System::DT_SEC;
    expect(saw_pivot, "a stopped nose short of the end squares up as a pivot");
    expect(!nav.settled(), "squaring up short of the end is not an arrival");
    expect(nav.stuck(), "an unanswered square-up gives the leg up");
    // The plan-building tick is already stopped short of the end, so it
    // counts toward the same unanswered clock as the square-up.
    expect(sec + System::DT_SEC + 1e-9 >= Motion::STUCK_SEC,
           "the square-up waits out the stuck window");
    expect(sec < Motion::STUCK_SEC + 0.5, "the square-up gives up once the window has passed");
}

static void testVoltageFollowsThePlant() {
    // The follower publishes a speed. Two VAS models turn that same speed
    // into different voltages, and the speed itself does not change.
    VASController soft(0.2, 1.0, 0.1, 0.2, 1.0, 0.1);
    VASController stiff(1.4, 4.0, 0.5, 1.4, 4.0, 0.5);
    SegmentFollower nav;
    nav.target.x = metersToInches(1.5);
    nav.target.y = 0.0;
    nav.target.exit_direction = kEast;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 0.0;

    StateEstimate st;
    st.pose = {0.0, 0.0, kEast};
    st.v_lin = 0.0;
    st.v_ang = 0.0;
    const ChassisCommand motion = nav.compute(st);
    expect(motion.request == DriveRequest::Motion, "the follower commands a velocity");
    SegmentFollower other;
    other.bindFeedforward(&stiff);
    other.target = nav.target;
    const ChassisCommand again = other.compute(st);
    expect(std::fabs(again.linear - motion.linear) < 1e-9
           && std::fabs(again.angular - motion.angular) < 1e-9,
           "the velocity command does not depend on the VAS gains");

    ChassisObservation obs;
    ChassisVoltageState soft_state;
    ChassisVoltageState stiff_state;
    const ChassisCommand soft_v = chassisVoltage(motion, obs, &soft, soft_state);
    const ChassisCommand stiff_v = chassisVoltage(motion, obs, &stiff, stiff_state);
    expect(std::fabs(stiff_v.linear - soft_v.linear) > 0.2,
           "only the VAS gains change the voltage");
}

static void testExitHeading() {
    // The last segment does not point along the exit heading. The leg stays
    // open on the end point until the nose has turned onto it.
    SegmentFollower nav;
    nav.target.x = metersToInches(1.0);
    nav.target.y = 0.0;
    nav.target.exit_direction = 0.0;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 0.0;

    StateEstimate st;
    st.pose = {0.0, 0.0, kEast};
    st.v_lin = 0.0;
    st.v_ang = 0.0;
    nav.compute(st);

    const Pose2d centre = tracking::centerFromTracking(
        poseInchesToMeters(Pose2d{nav.target.x, nav.target.y, 0.0}));
    st.pose = poseMetersToInches(tracking::trackingFromCenter(Pose2d{centre.x, centre.y, kEast}));
    nav.compute(st);
    expect(!nav.settled(), "the leg waits for the exit heading");
    expect(std::fabs(mathutil::normalize_angle(nav.goalHeading())) < 0.05,
           "the exit heading is the goal");

    st.pose = poseMetersToInches(tracking::trackingFromCenter(Pose2d{centre.x, centre.y, 0.0}));
    nav.compute(st);
    expect(nav.settled(), "the leg finishes on the exit heading");
}

// The score parks on its standoff. The approach stays outside the goal keep-out
// until the last segment, which may enter because the standoff sits inside it.
static void testApproachInsideCircle() {
    const double heading = M_PI / 2.0;
    const Pose2d parked{-20.0, 24.0, heading};
    SegmentFollower nav;
    nav.target.x = parked.x;
    nav.target.y = parked.y;
    nav.target.exit_direction = heading;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 0.0;
    nav.target.point = CommandPoint::Tracking;
    nav.config.obstacle_n = Field::goalAvoidObstacles(nav.config.obstacles, kPathObstacles);

    Pose2d path[40];
    const Pose2d from{48.0, 24.0, -3.0 * M_PI / 4.0};
    const int n = nav.samplePath(from, path, 40);
    expect(n >= 2, "the score approach has a path");
    const Pose2d end = centreOf(parked);
    expect(std::hypot(path[n - 1].x - end.x, path[n - 1].y - end.y) < 0.2,
           "the approach ends on the standoff centre");
    const double radius = Field::goalAvoidIn();
    for (int i = 1; i < n; ++i) {
        // The standoff is inside the keep-out, so the last segment is the
        // allowed entry. Every earlier segment stays 12 inches off the goal.
        if (i < n - 1) {
            const double into = gapTo(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y, -24.0, 24.0);
            expect(into + 0.05 >= radius, "the approach stays off the scored goal");
        }
        const double other = gapTo(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y, 24.0, 24.0);
        expect(other + 0.05 >= radius, "the approach stays off the other goal");
    }
}

// The retreat starts beside a goal and must not drive closer to it on the
// way to the far side.
static void testLeaveScoredGoal() {
    auton::DriveSegments drive(MotionPose{-48.0, 24.0, M_PI / 4.0, 0.0}, segmentPath(), nullptr);
    Pose2d path[40];
    const Pose2d from{-18.0, 24.0, M_PI / 2.0};
    const int n = drive.samplePath(from, path, 40, 0.0, 0.0);
    expect(n >= 2, "the retreat drive has a path");
    double closest = 1e9;
    for (int i = 1; i < n; ++i) {
        const double gap = gapTo(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y, -24.0, 24.0);
        if (gap < closest) closest = gap;
    }
    expect(closest + 0.05 >= Field::goalAvoidIn(), "leaving the goal does not drive back into it");
}

struct Body {
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
    double v = 0.0;
    double w = 0.0;
    double v_hold = 0.0;
    double w_hold = 0.0;
};

static void stepBody(SegmentFollower& nav, SpeedTap& tap, Body& b) {
    (void)tap;
    const Pose2d bumper = tracking::trackingFromCenter(Pose2d{b.x, b.y, b.th});
    StateEstimate st;
    st.pose = poseMetersToInches(bumper);
    st.v_lin = b.v;
    st.v_ang = b.w;
    const ChassisCommand cmd = nav.compute(st);
    double v_app = b.v_hold;
    double w_app = b.w_hold;
    // A command this small is a stop. The chassis does not creep on it.
    if (std::fabs(v_app) < 0.02 && std::fabs(b.v) < 0.02) v_app = 0.0;
    // A stopped chassis does not start a yaw on a trim. Aligning onto a
    // vertex commands a standing pivot, which is above this rate.
    if (std::fabs(b.v) < 0.02 && std::fabs(v_app) < 0.02
        && std::fabs(w_app) < 0.5 && std::fabs(b.w) < 0.08) {
        w_app = 0.0;
    }
    b.v = v_app;
    b.w = w_app;
    b.th = mathutil::normalize_angle(b.th + b.w * System::DT_SEC);
    b.x += b.v * System::DT_SEC * mathutil::heading_cos(b.th);
    b.y += b.v * System::DT_SEC * mathutil::heading_sin(b.th);
    b.v_hold = cmd.linear;
    b.w_hold = cmd.angular;
}

// One drive, from a centre pose, until it settles or gives up. `place`
// teleports onto a corner the first time the centre gets near it.
static int driveCase(SegmentFollower& nav, SpeedTap& tap, Body start,
                     const Pose2d* path, int n, int corner, double ox, double oy,
                     double place_heading, bool do_place, char* why, int why_n,
                     double lock_nose = NAN) {
    Body b = start;
    bool placed = !do_place;
    double ax = 0.0;
    double ay = 0.0;
    double ath = 0.0;
    int idle = 0;
    int since_anchor = 0;
    const int limit = do_place ? 2200 : 1400;
    for (int i = 0; i < limit; ++i) {
        if (!placed && corner > 0 && corner < n) {
            const double cx = inchesToMeters(path[corner].x);
            const double cy = inchesToMeters(path[corner].y);
            if (std::hypot(b.x - cx, b.y - cy) < 0.15 && i > 15) {
                b.x = cx + ox;
                b.y = cy + oy;
                b.th = place_heading;
                b.v = 0.0;
                b.w = 0.0;
                b.v_hold = 0.0;
                b.w_hold = 0.0;
                placed = true;
                idle = 0;
                since_anchor = 0;
                ax = b.x;
                ay = b.y;
                ath = b.th;
            }
        }
        const double x0 = b.x;
        const double y0 = b.y;
        const double th0 = b.th;
        stepBody(nav, tap, b);
        if (!do_place && std::isfinite(lock_nose)
            && std::fabs(mathutil::normalize_angle(b.th - lock_nose)) > 0.5) {
            std::snprintf(why, why_n, "nose swung to %.2f", b.th);
            return 1;
        }
        if (nav.stuck()) {
            std::snprintf(why, why_n, "gave up at (%.3f, %.3f)", b.x, b.y);
            return 1;
        }
        if (nav.settled()) {
            const double ex = inchesToMeters(path[n - 1].x);
            const double ey = inchesToMeters(path[n - 1].y);
            const double dist = std::hypot(b.x - ex, b.y - ey);
            double want = nav.target.exit_direction;
            if (!(nav.config.forwards && !nav.target.backwards))
                want = mathutil::normalize_angle(want + Robot::PI);
            const double herr = std::fabs(mathutil::normalize_angle(b.th - want));
            if (dist > 0.02) {
                std::snprintf(why, why_n, "settled %.1f cm from the end", dist * 100.0);
                return 1;
            }
            if (herr > 0.25 && !nav.target.ignore_direction) {
                std::snprintf(why, why_n, "settled %.0f deg off the exit heading",
                              herr * 180.0 / M_PI);
                return 1;
            }
            if (do_place && !placed) {
                std::snprintf(why, why_n,
                              "finished before the corner, at (%.3f, %.3f) heading %.2f",
                              b.x, b.y, b.th);
                return 1;
            }
            return 0;
        }
        if (since_anchor == 0) {
            ax = x0;
            ay = y0;
            ath = th0;
        }
        ++since_anchor;
        if (since_anchor >= 60) {
            const double moved = std::hypot(b.x - ax, b.y - ay);
            const double turned = std::fabs(mathutil::normalize_angle(b.th - ath));
            if (moved < 0.003 && turned < 0.05) {
                ++idle;
                if (idle > 0) {
                    std::snprintf(why, why_n,
                                  "sat still at (%.3f, %.3f) heading %.2f",
                                  b.x, b.y, b.th);
                    return 1;
                }
            }
            since_anchor = 0;
            idle = 0;
        }
    }
    if (do_place && !placed) {
        std::snprintf(why, why_n, "never reached the corner");
        return 1;
    }
    std::snprintf(why, why_n, "timed out at (%.3f, %.3f) heading %.2f", b.x, b.y, b.th);
    return 1;
}

static void testCornerDoesNotStick() {
    struct Setup {
        const char* name;
        double x;
        double y;
        double th;
        double gx;
        double gy;
        double exit_h;
        double line;
        bool backwards;
        bool obstacles;
        bool lock_nose;
    };
    const Setup setups[] = {
        {"straight", 0.0, 0.2, kEast, 1.4, 0.2, kEast, 0.0, false, false, false},
        {"join", 0.0, 0.75, -0.4, 0.85, 0.05, Robot::PI, 0.28, false, false, false},
        {"sharp", 0.15, 0.55, kEast, 0.15, 0.15, kWest, 0.22, false, false, false},
        {"shallow", 0.0, 0.40, -1.0, 1.1, 0.28, -1.15, 0.35, false, false, false},
        {"reverse", 0.9, 0.30, kWest, 0.15, 0.30, kWest, 0.2, true, false, false},
        {"back-in", 0.0, 0.0, kWest, 1.0, 0.0, kWest, 0.30, false, false, true},
        {"loader", 0.61, 1.22, Robot::PI, 1.52, 0.0, Robot::PI, 0.3048, false, true, false},
    };

    int failed = 0;
    for (const Setup& s : setups) {
        SegmentFollower nav;
        nav.config.path_accel = 1.5;
        nav.config.cruise_speed = 1.0;
        nav.config.min_accel = 0.15;
        nav.config.forwards = !s.backwards;
        nav.target.x = metersToInches(s.gx);
        nav.target.y = metersToInches(s.gy);
        nav.target.exit_direction = s.exit_h;
        nav.target.exit_speed = 0.0;
        nav.target.line_length = metersToInches(s.line);
        nav.target.backwards = s.backwards;
        if (s.obstacles) {
            nav.config.obstacle_n = Field::goalAvoidObstacles(nav.config.obstacles, kPathObstacles);
        }
        SpeedTap tap;
        nav.bindFeedforward(&tap);
        const Pose2d from = poseMetersToInches(
            tracking::trackingFromCenter(Pose2d{s.x, s.y, s.th}));
        Pose2d path[40];
        const int n = nav.samplePath(from, path, 40);
        char why[160];
        Body start{s.x, s.y, s.th, 0, 0, 0, 0};
        if (n < 2) {
            std::printf("FAIL %s has no path\n", s.name);
            ++failed;
            continue;
        }
        const double lock = s.lock_nose ? s.th : NAN;
        if (driveCase(nav, tap, start, path, n, -1, 0, 0, 0, false, why, 160, lock)) {
            std::printf("FAIL %s full run: %s\n", s.name, why);
            ++failed;
        }

        int corner = -1;
        for (int i = 1; i < n; ++i) {
            const double turn = std::fabs(mathutil::normalize_angle(path[i].theta - path[i - 1].theta));
            if (turn > 0.08) corner = i;
        }
        if (corner < 0) continue;
        const double incoming = path[corner - 1].theta;
        const double outgoing = path[corner].theta;
        // Short of the vertex on the incoming segment, off the next line.
        // A stop there is the incoming leg finishing, so the pivot starts.
        const double ix = mathutil::heading_cos(incoming);
        const double iy = mathutil::heading_sin(incoming);
        const double shorts[] = {0.03, 0.04};
        for (double back : shorts) {
            const double sox = -back * ix;
            const double soy = -back * iy;
            for (double heading : {incoming, incoming + 1.2}) {
                SegmentFollower again;
                again.config = nav.config;
                again.target = nav.target;
                SpeedTap tap2;
                again.bindFeedforward(&tap2);
                char detail[160];
                if (driveCase(again, tap2, start, path, n, corner, sox, soy, heading,
                              true, detail, 160)) {
                    if (failed < 16) {
                        std::printf("FAIL %s short %.1fcm h=%.2f: %s\n",
                                    s.name, back * 100.0, heading, detail);
                    }
                    ++failed;
                }
            }
        }
        const double headings[] = {
            incoming, outgoing, incoming + 1.2, incoming - 1.5, incoming + M_PI};
        const double ux = mathutil::heading_cos(outgoing);
        const double uy = mathutil::heading_sin(outgoing);
        const double lx = -uy;
        const double ly = ux;
        const double alongs[] = {-0.020, -0.011, 0.0, 0.010, 0.020};
        const double crosses[] = {-0.010, -0.005, 0.0, 0.005, 0.010};
        for (double along : alongs) {
            for (double cross : crosses) {
                const double ox = along * ux + cross * lx;
                const double oy = along * uy + cross * ly;
                for (double heading : headings) {
                    SegmentFollower again;
                    again.config = nav.config;
                    again.target = nav.target;
                    SpeedTap tap2;
                    again.bindFeedforward(&tap2);
                    char detail[160];
                    if (driveCase(again, tap2, start, path, n, corner, ox, oy, heading,
                                  true, detail, 160)) {
                        if (failed < 16) {
                            std::printf("FAIL %s line along=%.1fcm cross=%.1fcm h=%.2f: %s\n",
                                        s.name, along * 100.0, cross * 100.0, heading, detail);
                        }
                        ++failed;
                    }
                }
            }
        }
    }
    if (failed != 0) std::printf("%d corner cases failed\n", failed);
    expect(failed == 0, "every corner arrival reaches the end");
}

static void testSwitch() {
    const PathConstraints segments = segmentPath();
    expect(segments.line_segments, "segmentPath selects the polyline");
    const PathConstraints orbit = segmentPath(turnOnGoal());
    expect(orbit.line_segments && orbit.turn_on_goal, "an orbit keeps both flags");
}

int main() {
    testStraight();
    testKeepOut();
    testFieldBound();
    testReverseFinish();
    testForwardsFlag();
    testStanleySide();
    testGoalAvoid();
    testApproachInsideCircle();
    testLeaveScoredGoal();
    testStuck();
    testNoOvershoot();
    testPastEndStops();
    testHeavyStillStops();
    testJoinAndFinishStop();
    testShortCornerStillTurns();
    testStoppedNoseSquaresUp();
    testVoltageFollowsThePlant();
    testCornerDoesNotStick();
    testExitHeading();
    testSwitch();
    if (g_fails != 0) {
        std::printf("%d failed\n", g_fails);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
