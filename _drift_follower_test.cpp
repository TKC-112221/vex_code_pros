#include "control/DriftFollower.h"
#include "control/DriftPathFit.h"
#include "interfaces/IFeedForward.h"
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

    nav.reset();
    nav.config.shape = DriftCurveShape::Hermite;
    nav.config.timed_elastic = true;
    nav.config.slip = DriftSlipModel::Arcsin;
    nav.config.a_max = Drift::A_MAX;
    nav.config.centripetal_accel = 6.0;
    nav.target.line_length = 0.0;
    nav.target.exit_speed = 0.0;
    nav.target.exit_direction = 0.0;
    nav.target.x = metersToInches(1.6);
    nav.target.y = 0.0;
    st.pose = {0.0, metersToInches(0.45), 0.0};
    st.v_lin = 2.0;
    st.v_ang = 1.5;
    nav.compute(st);
    const int nt = nav.samplePath(st.pose, path, 64, st.v_lin, st.v_ang);
    expect(nt >= 8, "timed elastic sample count");
    expect(std::hypot(path[0].x, path[0].y - metersToInches(0.45)) < 1e-3,
           "timed elastic starts at the pose");
    expect(std::hypot(path[nt - 1].x - metersToInches(1.6), path[nt - 1].y) < 1e-2,
           "timed elastic ends at the target");
    expect(std::fabs(mathutil::normalize_angle(path[nt - 1].theta)) < 0.2,
           "timed elastic arrives on the exit heading");

    {
        using namespace driftpath;
        const double rx = 0.0, ry = 0.45, tx = 1.6, ty = 0.0, approach = 0.0;
        const double v = 2.0;
        const double kappa0 = 4.0;
        const TebLimits lim = tebLimits(v, 0.0, v, 2.0, 3.0, 0.0);
        const TimedBand band = timedBandThrough(rx, ry, tx, ty, approach, 1.0, 0.0, kappa0, lim);
        expect(band.curved, "timed elastic curves");
        expect(band.count >= 8, "timed elastic pose count");
        expect(std::hypot(band.x[0] - rx, band.y[0] - ry) < 1e-6, "band starts at the robot");
        expect(std::hypot(band.x[band.count - 1] - tx, band.y[band.count - 1] - ty) < 1e-6,
               "band ends at the join");
        const double end_dx = band.x[band.count - 1] - band.x[band.count - 2];
        const double end_dy = band.y[band.count - 1] - band.y[band.count - 2];
        expect(std::fabs(std::atan2(end_dy, end_dx) - approach) < 0.05,
               "last segment is the exit direction");
        const double phi0 = std::atan2(band.y[1] - band.y[0], band.x[1] - band.x[0]);
        expect(std::fabs(phi0) < 0.05, "first segment follows the momentum");
        double peak = 0.0;
        double demand = 0.0;
        for (int i = 0; i < band.count; ++i) {
            const double ak = std::fabs(band.kappa_s[i]);
            if (ak > peak) peak = ak;
            const double spent = band.v[i] * band.v[i] * ak;
            if (spent > demand) demand = spent;
        }
        const Quintic herm = hermiteThrough(rx, ry, tx, ty, approach, 1.0, 0.0, kappa0);
        double hpeak = std::fabs(herm.kappa);
        for (int i = 1; i <= 24; ++i) {
            double phi = 0.0, k = 0.0;
            quinticFrame(herm, static_cast<double>(i) / 24.0, phi, k);
            if (std::fabs(k) > hpeak) hpeak = std::fabs(k);
        }
        expect(demand <= lim.a_cent * 1.15 + 0.05, "profile stays inside centripetal accel");
        expect(peak < hpeak * 0.75, "band opens a curvature the speed cannot hold");

        SpeedKnot straight[2];
        straight[0] = {0.0, 0.0, 0.0};
        straight[1] = {2.0, 0.0, 0.0};
        const double brake = std::sqrt(2.0 * 2.0 * 2.0);
        const double v_straight = feasibleSpeedAt(straight, 2, 0.0, 0.0, 3.0, 2.0, 6.0, 20.0, 50.0);
        expect(std::fabs(v_straight - brake) < 1e-3, "straight profile is the brake speed");
        straight[0].kappa = 2.0;
        fillSpeedSlope(straight, 2);
        const double v_curve = feasibleSpeedAt(straight, 2, 0.0, 0.0, 3.0, 2.0, 2.0, 20.0, 50.0);
        expect(v_curve < brake, "curvature is tighter than braking alone");
        expect(v_curve <= std::sqrt(2.0 / 2.0) + 1e-6, "centripetal cap binds");

        SpeedKnot rewarded[2];
        rewarded[0] = {0.0, 1.0, 0.0};
        rewarded[1] = {2.0, 1.0, 0.0};
        fillSpeedSlope(rewarded, 2);
        const double v_slow = feasibleSpeedAt(rewarded, 2, 0.0, 0.0, 3.0, 2.0, 6.0, 20.0, 50.0);
        const double v_fast = feasibleSpeedAt(rewarded, 2, 0.0, 0.0, 3.0, 2.0, 12.0, 20.0, 100.0);
        expect(v_fast > v_slow + 0.2, "a higher centripetal budget holds more speed");

        // Goal behind the nose, arrive still facing forward: back past it, then in.
        const TebLimits ahead = tebLimits(0.0, 0.0, 2.0, 2.0, 6.0, 0.0, 0.0, 1.0, 0.0);
        const TimedBand cusp = timedBandThrough(0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, ahead);
        bool reversed = false;
        bool past = false;
        for (int i = 0; i < cusp.count; ++i) {
            if (cusp.drive[i] < 0.0) reversed = true;
            if (cusp.x[i] < -1.02) past = true;
        }
        expect(reversed, "band backs up when the goal is behind the nose");
        expect(past, "reverse leg passes the goal before driving in");
        double v_at_flip = 1e9;
        for (int i = 1; i < cusp.count; ++i) {
            if ((cusp.drive[i - 1] < 0.0) != (cusp.drive[i] < 0.0)) v_at_flip = cusp.v[i];
        }
        expect(v_at_flip < 0.05, "reversal target speed is a stop");

        // On the heading axis the band is the chord, and the target speed
        // falls from the brake ceiling to the exit speed.
        const TebLimits line_lim = tebLimits(0.0, 0.0, 2.0, 2.0, 6.0, 0.0);
        const TimedBand line = timedBandThrough(0.0, 0.0, 2.0, 0.0, 0.0, 1.0, 0.0, 0.0, line_lim);
        expect(line.count >= 2, "straight band has poses");
        expect(line.v[0] > 1.5, "straight band leaves near the brake speed");
        expect(line.v[line.count - 1] < 0.05, "straight band arrives stopped");

        // Whole move is reverse: leave toward the goal, nose still facing +x.
        const TebLimits backing = tebLimits(0.0, 0.0, 2.0, 2.0, 6.0, 0.0, 0.0, -1.0, 0.0);
        const TimedBand back = timedBandThrough(0.0, 0.0, -1.0, 0.0, M_PI, 1.0, 0.0, 0.0, backing);
        expect(back.count >= 2 && back.drive[0] < 0.0, "reverse finish opens going backward");
        const double leave = std::atan2(back.y[1] - back.y[0], back.x[1] - back.x[0]);
        expect(std::fabs(mathutil::normalize_angle(leave - M_PI)) < 0.25,
               "reverse finish leaves toward the goal");

        // A few degrees of heading error on a forward goal is a curve,
        // not a reversal. The line intersection sits behind the goal, and
        // taking it makes the command flip sign every tick.
        const TebLimits offset = tebLimits(0.4, 0.0, 2.0, 4.0, 12.0, 0.0, 0.3, 1.0, 0.0);
        const TimedBand nudged = timedBandThrough(0.0, 0.05, 1.0, 0.0, 0.0,
                                                 std::cos(0.3), std::sin(0.3), 0.0, offset);
        bool backed = false;
        for (int i = 0; i < nudged.count; ++i) if (nudged.drive[i] < 0.0) backed = true;
        expect(!backed, "forward goal with a heading offset stays forward");
        expect(nudged.x[nudged.count - 1] > nudged.x[0], "forward goal still ends ahead");

        // Heading stays +x while the robot is backing up. The opening
        // follows that travel, not the nose.
        TebLimits moving = tebLimits(0.8, 0.0, 2.0, 2.0, 6.0, 0.0, 0.0, 1.0, 0.0);
        moving.motion = M_PI;
        const TimedBand backing_off = timedBandThrough(0.0, 0.45, 1.2, 0.0, 0.0,
                                                       -1.0, 0.0, 0.0, moving);
        const double open = std::atan2(backing_off.y[1] - backing_off.y[0],
                                       backing_off.x[1] - backing_off.x[0]);
        expect(std::fabs(mathutil::normalize_angle(open - M_PI)) < 0.35,
               "opening follows the movement, not the heading");

        // Nose and travel point the same way, the goal is behind both, and
        // the exit faces the other way. Reverse off to one side, then in.
        TebLimits flip = tebLimits(0.6, 0.6, Robot::MAX_VEL_LIN, 6.0, 12.0, 0.0, M_PI, 1.0, 0.0);
        flip.motion = M_PI;
        const TimedBand swung = timedBandThrough(0.0, 0.0, 1.0, 0.0, 0.0,
                                                 std::cos(M_PI), std::sin(M_PI), 0.0, flip);
        bool swung_back = false;
        bool swung_fwd = false;
        double bulge = 0.0;
        for (int i = 0; i < swung.count; ++i) {
            if (swung.drive[i] < 0.0) swung_back = true;
            if (swung.drive[i] > 0.0) swung_fwd = true;
            if (std::fabs(swung.y[i]) > bulge) bulge = std::fabs(swung.y[i]);
        }
        expect(swung_back && swung_fwd, "opposite exit behind the nose reverses then finishes forward");
        expect(bulge > 0.15, "reverse-then-forward bows off the chord");

        // A one-point kink. The loss is ψ⃛ of φ + σ under the arcsin map.
        TimedBand kink;
        kink.count = kTebPoses;
        kink.curved = true;
        for (int i = 0; i < kink.count; ++i) {
            const double u = static_cast<double>(i) / static_cast<double>(kink.count - 1);
            kink.x[i] = u * 2.0;
            kink.y[i] = (i == kink.count / 2) ? 0.25 : 0.0;
            kink.drive[i] = 1.0;
        }
        TebLimits jerk_lim = tebLimits(2.0, 0.0, 2.0, 3.0, 6.0, 0.0, 0.0, 1.0, 0.0);
        jerk_lim.slip_arcsin = true;
        jerk_lim.slip_accel = Drift::A_MAX;
        jerk_lim.slip_limit = Drift::MAX_SLIP_RAD;
        const double jerk_before = peakDriftHeadingJerk(kink, jerk_lim);
        optimizeBand(kink, jerk_lim, kTebIters);
        const double jerk_after = peakDriftHeadingJerk(kink, jerk_lim);
        expect(std::isfinite(jerk_before) && std::isfinite(jerk_after),
               "drift heading jerk stays finite");
        expect(jerk_before > kTebYawJerk, "a kink exceeds the drift jerk budget");
        expect(jerk_after < jerk_before, "heading jerk loss lowers the third derivative");
    }

    nav.reset();
    nav.config.shape = DriftCurveShape::Hermite;
    nav.config.timed_elastic = true;
    nav.target.x = metersToInches(1.6);
    nav.target.y = 0.0;
    nav.target.exit_direction = 0.0;
    nav.target.exit_speed = 0.0;
    st.pose = {0.0, metersToInches(0.45), 0.0};
    st.v_lin = 0.0;
    st.v_ang = 0.0;
    nav.compute(st);
    st.pose = {metersToInches(0.40), metersToInches(0.20), 0.0};
    nav.compute(st);
    const int nrefit = nav.samplePath({9.0, 9.0, 0.0}, path, 64, 0.0, 0.0);
    expect(nrefit >= 2, "committed plan still has a path");
    expect(std::hypot(path[0].x - 0.0, path[0].y - metersToInches(0.45)) < 1e-2,
           "timed elastic keeps the plan from the first pose");

    // Too fast into a reversal: the band's target speed is the brake to the
    // stop, which is below the speed a single brake to the exit would allow.
    // The follower should shed speed. linear volts are the nose-frame accel.
    {
        struct AccelFF : IFeedForward {
            double linearVolts(double, double accel, double, double, double, double) const override {
                return accel;
            }
            double angularVolts(double, double, double, double, double) const override {
                return 0.0;
            }
        } ff;
        using namespace driftpath;
        nav.reset();
        nav.bindFeedforward(&ff);
        nav.config.shape = DriftCurveShape::Hermite;
        nav.config.timed_elastic = true;
        nav.config.speed_reward = 1.0;
        nav.config.path_accel = 2.0;
        nav.config.cruise_speed = 4.0;
        nav.config.centripetal_accel = 6.0;
        nav.config.head_kP = 0.0;
        nav.config.head_kI = 0.0;
        nav.config.head_kD = 0.0;
        nav.config.lat_kP = 0.0;
        nav.config.lat_kI = 0.0;
        nav.config.lat_kD = 0.0;
        nav.config.lat_kD2 = 0.0;
        nav.target.x = metersToInches(-0.40);
        nav.target.y = 0.0;
        nav.target.exit_direction = 0.0;
        nav.target.exit_speed = 0.0;
        nav.target.line_length = 0.0;
        const TebLimits lim = tebLimits(0.0, 0.0, 4.0, 2.0, 6.0, 0.0, 0.0, 1.0, 0.0);
        const TimedBand band = timedBandThrough(0.0, 0.0, -0.40, 0.0, 0.0, 0.0, 0.0, 0.0, lim);
        expect(band.count >= 2 && band.drive[0] < 0.0, "follower case opens in reverse");
        expect(band.v[0] > 0.4 && band.v[0] < 3.5, "reversal ceiling is a real brake speed");
        st.pose = {0.0, 0.0, 0.0};
        st.v_lin = -(band.v[0] + 0.20);
        st.v_ang = 0.0;
        const ChassisCommand braking = nav.compute(st);
        std::printf("cusp brake linear %.3f band_v0 %.3f v_lin %.3f\n",
                    braking.linear, band.v[0], st.v_lin);
        expect(braking.linear > 0.5, "overspeed into a reversal brakes");
    }

    // Latched on the (24, 24) in goal, nose along +x. A quarter turn to +y
    // rides the orbit and presses forward while yawing CCW.
    {
        nav.reset();
        nav.config.timed_elastic = true;
        nav.config.settle_deg = 1.0;
        const double half_in = metersToInches(Robot::Chassis::BODY_LENGTH_M * 0.5);
        const double gx = 24.0;
        const double gy = 24.0;
        nav.target.turn_on_goal = true;
        nav.target.x = gx;
        nav.target.y = gy;
        nav.target.exit_direction = Robot::PI / 2.0;
        nav.target.exit_speed = 0.0;
        nav.target.line_length = 0.0;
        nav.target.backwards = false;
        st.pose = {gx - half_in, gy, 0.0};
        st.v_lin = 0.0;
        st.v_ang = 0.0;
        const ChassisCommand swing = nav.compute(st);
        std::printf("goal turn surge %.3f yaw %.3f\n", swing.linear, swing.angular);
        expect(swing.linear > 6.0, "goal turn presses into the goal");
        expect(swing.angular > 3.0, "goal turn yaws toward the heading");
        expect(swing.linear > std::fabs(swing.angular), "inside wheel stays forward");
        expect(std::fabs(swing.linear) + std::fabs(swing.angular) <= nav.config.max_volts + 1e-6,
               "goal turn fits the rail");
        const int ng = nav.samplePath(st.pose, path, 64);
        expect(ng >= 8, "goal turn samples");
        expect(std::hypot(path[0].x - st.pose.x, path[0].y - st.pose.y) < 1e-3,
               "goal turn starts seated");
        expect(std::hypot(path[ng - 1].x - gx, path[ng - 1].y - (gy - half_in)) < 1e-2,
               "goal turn ends on the orbit");
        expect(std::fabs(mathutil::normalize_angle(path[ng - 1].theta - Robot::PI / 2.0)) < 1e-3,
               "goal turn faces the target heading");
        bool on_orbit = true;
        for (int i = 0; i < ng; ++i) {
            const double radius = std::hypot(path[i].x - gx, path[i].y - gy);
            if (std::fabs(radius - half_in) > 0.05) on_orbit = false;
        }
        expect(on_orbit, "goal turn keeps the nose on the goal");

        nav.reset();
        nav.target.exit_direction = -Robot::PI / 2.0;
        const ChassisCommand clockwise = nav.compute(st);
        expect(clockwise.linear > 6.0, "clockwise goal turn still presses");
        expect(clockwise.angular < -3.0, "clockwise goal turn yaws the other way");

        nav.reset();
        nav.target.exit_direction = 0.0;
        expect(!nav.settled(), "goal turn is not settled before it runs");
        ChassisCommand held{};
        for (int i = 0; i < 12; ++i) held = nav.compute(st);
        expect(std::fabs(held.angular) < 1e-6, "settled goal turn drops yaw");
        expect(held.linear > 6.0, "settled goal turn keeps the press");
        expect(nav.settled(), "goal turn settles on the heading");
    }

    {
        PathConstraints selected = headingInput(turnOnGoal());
        expect(selected.follower_input == FollowerInput::Heading, "headingInput selects stations");
        expect(selected.turn_on_goal, "headingInput keeps the other limits");

        nav.reset();
        nav.config.input = FollowerInput::Curvature;
        nav.config.timed_elastic = false;
        nav.config.shape = DriftCurveShape::Arc;
        nav.config.slip = DriftSlipModel::Arcsin;
        nav.config.a_max = Drift::A_MAX;
        nav.config.k_slip = 0.0;
        nav.config.max_slip_rad = Drift::MAX_SLIP_RAD;
        nav.config.lat_kP = 0.0;
        nav.config.lat_kI = 0.0;
        nav.config.lat_kD = 0.0;
        nav.config.lat_kD2 = 0.0;
        nav.config.head_kP = 0.0;
        nav.config.head_kI = 0.0;
        nav.config.head_kD = 0.0;
        nav.target.x = metersToInches(1.0);
        nav.target.y = 0.0;
        nav.target.exit_direction = 0.0;
        nav.target.exit_speed = 0.0;
        nav.target.line_length = 0.0;
        nav.target.turn_on_goal = false;
        st.pose = {0.0, metersToInches(0.5), 0.0};
        st.v_lin = 1.0;
        st.v_ang = 0.0;
        const ChassisCommand by_curvature = nav.compute(st);
        const double curvature_goal = nav.goalHeading();
        Pose2d samples[64];
        const int nsamp = nav.samplePath(st.pose, samples, 64);
        expect(nsamp >= 2, "heading-input arc samples");
        double walked = 0.0;
        double phi_ahead = samples[0].theta;
        for (int i = 1; i < nsamp; ++i) {
            walked += std::hypot(samples[i].x - samples[i - 1].x, samples[i].y - samples[i - 1].y);
            phi_ahead = samples[i].theta;
            if (walked >= metersToInches(0.30)) break;
        }

        nav.reset();
        nav.config.input = FollowerInput::Heading;
        const ChassisCommand by_heading = nav.compute(st);
        const double heading_goal = nav.goalHeading();
        const double curvature_lead = mathutil::normalize_angle(curvature_goal - samples[0].theta);
        const double heading_lead = mathutil::normalize_angle(heading_goal - phi_ahead);
        expect(std::isfinite(by_curvature.angular) && std::isfinite(by_heading.angular),
               "heading input command is finite");
        expect(curvature_lead > 0.02, "curvature slip leads the tangent");
        expect(heading_lead > 0.0, "heading slip leads the upcoming tangent");
        expect(std::fabs(mathutil::normalize_angle(heading_goal - curvature_goal)) < 0.25,
               "heading stations stay near the curvature feedforward");
    }

    {
        // A stop leg ends when speed falls near zero, or when the range to
        // the point starts growing. It does not wait for a pose window.
        nav.reset();
        nav.config.timed_elastic = false;
        nav.config.lat_kP = 0.0;
        nav.config.lat_kI = 0.0;
        nav.config.lat_kD = 0.0;
        nav.config.lat_kD2 = 0.0;
        nav.config.head_kP = 0.0;
        nav.config.head_kI = 0.0;
        nav.config.head_kD = 0.0;
        nav.target.turn_on_goal = false;
        nav.target.x = 40.0;
        nav.target.y = 0.0;
        nav.target.exit_direction = 0.0;
        nav.target.exit_speed = 0.0;
        nav.target.line_length = 0.0;
        st.pose = {0.0, 0.0, 0.0};
        st.v_lin = 0.0;
        st.v_ang = 0.0;
        nav.compute(st);
        expect(!nav.settled(), "a resting start does not finish the drive");

        st.pose.x = 10.0;
        st.v_lin = 1.0;
        nav.compute(st);
        expect(!nav.settled(), "closing in at speed does not finish the drive");

        st.pose.x = 16.0;
        st.v_lin = 0.05;
        nav.compute(st);
        expect(nav.settled(), "near-zero speed finishes short of the target");

        nav.reset();
        st.pose = {0.0, 0.0, 0.0};
        st.v_lin = 0.0;
        nav.compute(st);
        st.pose.x = 20.0;
        st.v_lin = 1.0;
        nav.compute(st);
        st.pose.x = 18.0;
        st.v_lin = 1.0;
        nav.compute(st);
        expect(nav.settled(), "growing distance finishes while still moving");

        nav.reset();
        st.pose = {40.0, 0.0, 0.0};
        st.v_lin = 0.0;
        nav.compute(st);
        expect(nav.settled(), "already on the target finishes");
    }

    std::printf("%s (%d)\n", g_fails ? "FAILED" : "ok", g_fails);
    return g_fails ? 1 : 0;
}
