#include "control/DriftFollower.h"
#include "math/MathUtils.h"
#include <cmath>
#include <cstdio>

static int g_fails = 0;

static void expect(bool ok, const char* msg) {
    if (ok) return;
    std::printf("FAIL %s\n", msg);
    ++g_fails;
}

int main() {
    DriftFollower nav;
    nav.config.shape = DriftCurveShape::Quartic;
    nav.config.drift_accel = 1.0e6;
    nav.config.k_slip = 0.0;
    nav.config.lat_kP = 24.0;
    nav.config.lat_kI = 0.0;
    nav.config.lat_kD = 0.0;
    nav.config.lat_kD2 = 0.0;
    nav.target.x = metersToInches(1.0);
    nav.target.y = 0.0;
    nav.target.exit_direction = 0.0;
    nav.target.exit_speed = 0.0;
    nav.target.line_length = 0.0;
    nav.reset();

    StateEstimate st;
    st.pose = {0.0, metersToInches(0.4), 0.0};
    nav.compute(st);

    Pose2d path[64];
    const int n = nav.samplePath({9.0, 9.0, 0.0}, path, 64, 0.0, 0.0);
    expect(n >= 8, "quartic sample count");
    expect(std::hypot(path[0].x - 0.0, path[0].y - metersToInches(0.4)) < 1e-4,
           "plan starts at the pose");
    expect(std::hypot(path[n - 1].x - metersToInches(1.0), path[n - 1].y - 0.0) < 1e-3,
           "plan ends at the target");
    expect(std::fabs(path[n - 1].theta) < 0.05, "quartic arrives on the exit heading");

    Pose2d again[64];
    const int n2 = nav.samplePath({9.0, 9.0, 0.0}, again, 64);
    expect(n2 == n, "frozen count");
    expect(std::hypot(again[0].x - path[0].x, again[0].y - path[0].y) < 1e-9, "frozen start");

    const int mid = n / 2;
    const double nx = -std::sin(path[mid].theta);
    const double ny = std::cos(path[mid].theta);
    st.pose = {path[mid].x + metersToInches(0.10) * nx,
               path[mid].y + metersToInches(0.10) * ny, path[mid].theta};
    st.v_lin = 0.4;
    // The first tick after the teleport is the heading D-term. The next tick
    // is the cross-track loop with that derivative settled.
    nav.compute(st);
    const ChassisCommand cmd = nav.compute(st);
    std::printf("off-path yaw %.3f goal %.3f\n", cmd.angular, nav.goalHeading());
    expect(cmd.angular < -1.0, "left of the path steers clockwise");

    nav.reset();
    nav.config.shape = DriftCurveShape::Arc;
    st.pose = {0.0, metersToInches(0.5), 0.0};
    nav.compute(st);
    const int na = nav.samplePath(st.pose, path, 64);
    expect(na >= 8, "arc sample count");
    const double chord_y = path[0].y + (path[na - 1].y - path[0].y)
        * ((path[mid].x - path[0].x) / (path[na - 1].x - path[0].x));
    expect(std::fabs(path[na / 2].y - chord_y) > metersToInches(0.02), "arc leaves the chord");

    nav.reset();
    nav.config.shape = DriftCurveShape::Hermite;
    nav.target.line_length = metersToInches(0.3);
    st.pose = {0.0, metersToInches(0.2), 0.4};
    st.v_lin = 1.0;
    st.v_ang = 0.3;
    nav.compute(st);
    const int nh = nav.samplePath(st.pose, path, 64, st.v_lin, st.v_ang);
    expect(nh >= 8, "hermite sample count");
    expect(std::hypot(path[nh - 1].x - metersToInches(1.0), path[nh - 1].y) < 1e-3,
           "hermite ends at the target");

    nav.reset();
    nav.config.shape = DriftCurveShape::Exponential;
    nav.compute(st);
    expect(nav.samplePath(st.pose, path, 64) >= 2, "exponential samples");

    nav.reset();
    nav.config.shape = DriftCurveShape::Quintic;
    nav.compute(st);
    expect(nav.samplePath(st.pose, path, 64) >= 2, "quintic samples");

    nav.reset();
    nav.config.shape = DriftCurveShape::QuarticHermite;
    nav.compute(st);
    expect(nav.samplePath(st.pose, path, 64) >= 2, "quartic hermite samples");

    // A left-bending arc with tiny thrust saturates arcsin at +π/2.
    nav.reset();
    nav.config.shape = DriftCurveShape::Arc;
    nav.config.slip = DriftSlipModel::Arcsin;
    nav.config.a_max = 0.05;
    nav.config.k_slip = 0.0;
    nav.config.max_slip_rad = M_PI / 2.0;
    nav.target.line_length = 0.0;
    nav.target.exit_speed = 0.0;
    st.pose = {0.0, metersToInches(0.5), 0.0};
    st.v_lin = 1.5;
    st.v_ang = 0.0;
    nav.compute(st);
    const int ns = nav.samplePath(st.pose, path, 64);
    expect(ns >= 2, "arcsin arc samples");
    const double lead = mathutil::normalize_angle(nav.goalHeading() - path[0].theta);
    expect(lead > 1.0, "arcsin lead is near +pi/2 on a left bend");

    std::printf("%s (%d)\n", g_fails ? "FAILED" : "ok", g_fails);
    return g_fails ? 1 : 0;
}
