#include "auton/DriveTo.h"
#include "control/SegmentFollower.h"
#include "math/MathUtils.h"
#include <cmath>
#include <cstdio>

static int g_fails = 0;

static void expect(bool ok, const char* msg) {
    if (ok) return;
    std::printf("FAIL %s\n", msg);
    ++g_fails;
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
    nav.target.exit_direction = 0.0;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 0.0;
    nav.reset();

    const Pose2d from{0.0, 0.0, 0.0};
    Pose2d path[16];
    const int n = nav.samplePath(from, path, 16);
    expect(n == 2, "open field is one segment");
    expect(std::hypot(path[0].x - from.x, path[0].y - from.y) < 1e-6, "preview starts at the pose");
    expect(std::fabs(path[n - 1].x - nav.target.x) < 1e-4, "preview ends on the target x");
    expect(std::fabs(path[n - 1].y - nav.target.y) < 1e-4, "preview ends on the target y");

    StateEstimate st;
    st.pose = from;
    nav.compute(st);
    Pose2d again[16];
    const int n2 = nav.samplePath(Pose2d{10.0, 10.0, 0.0}, again, 16);
    expect(n2 == 2, "committed plan stays one segment");
    expect(std::hypot(again[0].x - from.x, again[0].y - from.y) < 1e-4,
           "a later pose does not rebuild the plan");
    const double want = std::atan2(nav.target.y, nav.target.x);
    const double heading = mathutil::normalize_angle(nav.goalHeading() - want);
    expect(std::fabs(heading) < 0.05, "on the segment the nose faces along it");
}

static void testKeepOut() {
    SegmentFollower nav;
    nav.target.x = metersToInches(2.0);
    nav.target.y = 0.0;
    nav.target.exit_direction = 0.0;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 0.0;
    nav.config.obstacle_n = 1;
    nav.config.obstacles[0] = {metersToInches(1.0), 0.0, metersToInches(0.30)};
    nav.reset();

    Pose2d path[40];
    const int n = nav.samplePath(Pose2d{0.0, 0.0, 0.0}, path, 40);
    expect(n > 2, "a blocking circle adds vertices");
    const double cx = metersToInches(1.0);
    const double radius = metersToInches(0.30);
    double side = 0.0;
    for (int i = 0; i < n; ++i) {
        const double d = std::hypot(path[i].x - cx, path[i].y);
        expect(d >= radius - 0.05, "vertex stays outside the keep-out");
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
    nav.target.exit_direction = 0.0;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 0.0;
    nav.config.obstacle_n = 1;
    nav.config.obstacles[0] = {0.0, metersToInches(0.25), metersToInches(0.35)};
    nav.reset();

    const Pose2d from{metersToInches(-1.0), metersToInches(0.25), 0.0};
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
    nav.target.exit_direction = Robot::PI;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = metersToInches(0.30);
    nav.reset();

    Pose2d path[8];
    const int n = nav.samplePath(Pose2d{0.0, 0.0, Robot::PI}, path, 8);
    expect(n == 3, "a reversed finish keeps the join");
    expect(path[1].x > nav.target.x + metersToInches(0.2), "the join sits past the target");
    expect(std::fabs(path[n - 1].x - nav.target.x) < 1e-3, "the finish ends on the target");

    StateEstimate st;
    st.pose = {0.0, 0.0, Robot::PI};
    nav.compute(st);
    const double err = std::fabs(mathutil::normalize_angle(nav.goalHeading() - Robot::PI));
    expect(err < 0.15, "the nose stays put across the reversal");

    SegmentFollower turned;
    turned.target = nav.target;
    StateEstimate face;
    face.pose = {0.0, 0.0, 0.0};
    turned.compute(face);
    const double spin = std::fabs(mathutil::normalize_angle(turned.goalHeading() - Robot::PI));
    expect(spin < 0.15, "a large nose error pivots before driving");
}

static void testStanleySide() {
    SegmentFollower nav;
    nav.target.x = metersToInches(2.0);
    nav.target.y = 0.0;
    nav.target.exit_direction = 0.0;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 0.0;
    StateEstimate on;
    on.pose = {0.0, 0.0, 0.0};
    nav.compute(on);

    StateEstimate off;
    off.pose = {metersToInches(0.5), metersToInches(0.20), 0.0};
    nav.compute(off);
    expect(nav.goalHeading() < -0.2, "left of the segment, the nose turns back toward it");
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
    testStanleySide();
    testSwitch();
    if (g_fails != 0) {
        std::printf("%d failed\n", g_fails);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
