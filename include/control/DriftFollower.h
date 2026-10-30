#pragma once
/* ---------------------------------------------------------------------------
 * DriftFollower  --  one planned curve, then a follower.
 *
 * The plan is the centre of rotation. A Tracking command names the front
 * bumper, and that point is shifted by the offset in TrackingCenter.h.
 * The sampled polyline is shifted back, so the field drawing stays in
 * the command's frame. An orbit already splits the two points itself.
 *
 * Same calls as DriftCurve, so it can stand in for that class. reset()
 * drops the plan. The next compute() fits the curve once from the pose it
 * is given and keeps that polyline. Later ticks project onto it. They do
 * not refit. Call reset() when `target` or the shape changes.
 *
 * The fit is the DriftCurve family (arc, exponential, quartic, quintic,
 * Hermite, quartic Hermite) plus the straight finish. Hermite and
 * quartic Hermite open on the measured momentum of that first tick.
 *
 * config.timed_elastic deforms that stored curve once, on the first
 * compute(). The band opens curvature the reference speed cannot hold,
 * walks free poses down the third derivative of the drifted heading,
 * pushes them out of keep-outs and back inside the field, and may replace
 * the curve with one reversal when that reaches the exit sooner. Later
 * ticks track that polyline. They do not reseed it. The seed starts on
 * the stored curve, so a cross-track error is not baked into the plan.
 * The follower tracks a short lookahead on the committed band, not the
 * chord that joins the robot to the first pose, plus the speed the time
 * steps imply. The yaw-rate feedforward changes no faster than the
 * yaw-acceleration limit.
 * Keep-out circles and the field bound apply only to this adjustment.
 *
 * Heading offset is the slip model. v_lat is the measured forward
 * velocity projected on the path's left normal. v_ref is the planned
 * travel speed. config.input is Curvature by default, and then κ and
 * its arc-length derivatives come from the stored path. Heading reads
 * the travel tangent at stations kHermiteLookMin apart, one and two
 * stations ahead, and rebuilds κ and dκ/ds from those differences.
 * d²κ/ds² stays zero. The stations stop at a drive reversal. A remainder
 * shorter than kTebLookMin keeps the upcoming tangent and drops the
 * curvature. Either way:
 *
 *   a_lat = v_ref² κ − K_slip v_lat
 *
 * config.slip picks the map from a_lat to the nose lead σ.
 * Atan, the default, uses config.drift_accel as a_drift:
 *
 *   σ = atan(a_lat / a_drift)
 *   a_actual_lat = a_max a_lat / sqrt(a_drift² + a_lat²)
 *
 * Arcsin uses the max thrust config.a_max instead:
 *
 *   σ = asin(clamp(a_lat / a_max, −1, 1))
 *   a_actual_lat = a_max sin(σ)
 *
 * σ is then clamped to config.max_slip_rad. The nose target is the
 * path tangent plus σ (plus half a turn when driving in reverse).
 *
 * Yaw rate and yaw acceleration follow the path and the changing slip.
 * v_lat is held fixed while differentiating, and longitudinal jerk is
 * taken as zero:
 *
 *   ȧ_lat = 2 v_ref a_travel κ + v_ref³ dκ/ds
 *   ä_lat = 2 a_travel² κ + 5 v_ref² a_travel dκ/ds + v_ref⁴ d²κ/ds²
 *
 * Atan, with u = a_lat / a_drift:
 *
 *   σ̇ = u̇ / (1 + u²)
 *   σ̈ = (ü (1 + u²) − 2 u u̇²) / (1 + u²)²
 *
 * Arcsin, with u = a_lat / a_max inside (−1, 1). At |u| ≥ 0.95 the
 * offset is treated as flat, so both derivatives are zero. Below that
 * they are capped at the yaw-rate and yaw-acceleration limits. On the
 * ±1 clamp they are zero too:
 *
 *   σ̇ = u̇ / sqrt(1 − u²)
 *   σ̈ = (ü (1 − u²) + u u̇²) / (1 − u²)^(3/2)
 *
 *   ω_FF = v_ref κ + σ̇
 *   α_FF = a_travel κ + v_ref² dκ/ds + σ̈
 *
 * The bound feedforward turns (ω_FF, α_FF) into differential volts
 * (kV ω + kA α, plus kS when the model has it). That voltage is
 * V_R − V_L, so it is halved into the chassis angular command, the
 * same scale TankDrive applies per side. A Boomerang heading PID trims
 * the nose onto the slip heading. A lateral PID, or PIDD2 when
 * lat_kD2 is set, adds chassis volts from the signed cross-track error
 * (left of the travel direction is positive; the loop steers back
 * toward the path). The lateral integrator leaks. It is not cleared when
 * the cross-track error changes sign.
 *
 * Longitudinal volts match DriftCurve. The speed is the kinodynamic
 * ceiling along the polyline being tracked (cruise, braking distance,
 * centripetal accel, yaw rate, yaw accel). config.speed_reward scales
 * the soft caps, so a higher reward holds more speed. With
 * timed_elastic the ceiling is the target speed stored on the committed
 * band. The command slews toward that ceiling from the previous command,
 * one acceleration step per tick, not from the measured speed. When surge
 * and yaw do not both fit on the rail, surge is reduced and the yaw is
 * kept: a slide is arrested by pointing the driven wheels into it.
 * The bound model supplies the linear feedforward. The band may drive
 * some segments backward; the nose then faces opposite that segment's
 * tangent.
 *
 * target.turn_on_goal is the latched swing. The plan is the orbit of the
 * center about the goal the front is on, out to the target heading. The
 * command is a hard forward press plus yaw, and the press stays larger
 * than the yaw so the inside wheel does not reverse. The timed band does
 * not reshape that orbit. Reverse is never asked for, so the latch stays
 * shut. The leg settles on heading, not on a free-space position.
 * ------------------------------------------------------------------------- */

#include "control/DriftCurve.h"
#include "control/MotionStuck.h"

// Atan maps a_lat through drift_accel. Arcsin maps it through a_max.
enum class DriftSlipModel { Atan, Arcsin };

// Keep-out for the timed elastic band. x, y, and min_dist are inches, the
// same frame as DriftCurveTarget. min_dist is the radius pose centers stay
// outside.
struct DriftObstacle {
    double x = 0;
    double y = 0;
    double min_dist = 0;
};

struct DriftFollowerConfig : DriftCurveConfig {
    // DriftSlipModel slip = DriftSlipModel::Atan;
    DriftSlipModel slip = DriftSlipModel::Arcsin;
    DriftCurveShape shape = DriftCurveShape::Hermite;
    // shape is DriftCurveConfig::shape. A second member here would hide it,
    // and the skills follower would keep planning Hermite.
    double k_slip = Drift::K_SLIP;           // 1/s. Subtracted from lateral accel per m/s of v_lat.
    double a_max = Drift::A_MAX;             // m/s^2. Arcsin thrust, and the atan lateral projection.
    double lat_kP = Drift::LAT_KP;           // rad/s per m of cross-track
    double lat_kI = Drift::LAT_KI;           // rad/s per (m·s)
    double lat_kD = Drift::LAT_KD;           // rad/s per (m/s)
    double lat_kD2 = Drift::LAT_KD2;         // rad/s per (m/s^2). Nonzero makes the lateral loop PIDD2.
    double lat_i_limit = Drift::LAT_I_LIMIT; // rad/s clamp on the lateral integrator term

    // Heading trim and settle window. Defaults match Boomerang:: / Motion::.
    // The tuner overwrites these for one rollout; the firmware keeps the defaults.
    double head_kP = Boomerang::Angular.kP;
    double head_kI = Boomerang::Angular.kI;
    double head_kD = Boomerang::Angular.kD;
    double head_windup = Boomerang::Angular.windup;
    double settle_speed = Motion::SETTLE_SPEED_MPS;
    double min_accel = Motion::MIN_ACCEL_MPS2;
    double turn_radius_max = Physics::TURN_RADIUS_MAX_M;
    double kappa_eps = Motion::KAPPA_EPS;
    double settle_in = Boomerang::Lateral.small_error;
    double settle_deg = Boomerang::Angular.small_error;

    // The shape above is planned once. The band deforms that polyline once:
    // curvature, the drifted heading jerk, keep-outs, the field, and the cusp.
    bool timed_elastic = true;
    // Curvature reads κ, dκ/ds, and d²κ/ds². Heading rebuilds the first two
    // from travel tangents at regular stations and leaves d²κ/ds² at zero.
    FollowerInput input = FollowerInput::Curvature;
    // Keep-out circles are inches, the same frame as target. min_dist is
    // the radius pose centers stay outside. field_bound keeps free poses
    // inside the 12 ft field. At most driftpath::kTebObstacles.
    bool field_bound = false;
    int obstacle_n = 0;
    DriftObstacle obstacles[driftpath::kTebObstacles] {};
};

inline void copyObstacles(driftpath::TebLimits& lim, const DriftFollowerConfig& cfg) {
    lim.field_bound = cfg.field_bound;
    int n = cfg.obstacle_n;
    if (n < 0) n = 0;
    if (n > driftpath::kTebObstacles) n = driftpath::kTebObstacles;
    lim.obstacle_n = 0;
    for (int i = 0; i < n; ++i) {
        if (!(cfg.obstacles[i].min_dist > 0.0)) continue;
        driftpath::TebObstacle& obs = lim.obstacles[lim.obstacle_n];
        obs.x = inchesToMeters(cfg.obstacles[i].x);
        obs.y = inchesToMeters(cfg.obstacles[i].y);
        obs.min_dist = inchesToMeters(cfg.obstacles[i].min_dist);
        ++lim.obstacle_n;
    }
}

class DriftFollower {
public:
    DriftFollowerConfig config;
    DriftCurveTarget target;

    void reset();

    // Kept so callers can still pass a model. The drive base owns the VAS
    // conversion. The pointer is not used and is not owned.
    void bindFeedforward(const IFeedForward* model) { model_ = model; }

    // state : field pose, forward speed (v_lin, m/s), yaw rate (v_ang, rad/s).
    // The first call after reset() plans from this state. Later calls track.
    ChassisCommand compute(const StateEstimate& state);

    // Heading the chassis was told to face on the last compute(), radians.
    double goalHeading() const { return goal_heading_; }

    // True once forward speed has dropped to settle_speed after the leg has
    // been moving, or once the distance to the target grows past the closest
    // point reached so far. A coordinate window is not required. A stop also
    // waits until the nose is on the exit heading. A planned reversal, where
    // path speed is zero and later stations speed up, is not the end of the
    // leg. turn_on_goal still settles on heading.
    bool settled() const { return settled_; }

    // True once a translate or pivot request has gone unanswered. A hold
    // does not set it, and a settled leg clears it.
    bool stuck() const { return stuck_watch_.stuck(); }

    // Polyline the follower is tracking, the centre of rotation, in field
    // inches. A command pose is the front bumper; this line is the axle.
    // Before the first compute() this is a preview from `from` and does not
    // commit the plan. v_lin and v_ang are the measured momentum a Hermite
    // preview opens along.
    // With timed_elastic the preview, and the path after compute(), is the
    // band committed with the plan. A later pose does not rebuild it.
    int samplePath(const Pose2d& from, Pose2d* out, int cap,
                   double v_lin = 0, double v_ang = 0) const;

    // The curve planned once. timed_elastic does not change it. Before
    // the first compute() this previews that curve and does not commit.
    int sampleBase(const Pose2d& from, Pose2d* out, int cap,
                   double v_lin = 0, double v_ang = 0) const;

    // Achievable lateral acceleration from the last compute(), m/s^2.
    double achievableLateral() const { return a_actual_lat_; }

    // One sample of the planned path. s is arc length from the start.
    // phi is the travel tangent, unwrapped along the polyline. kappa is
    // signed CCW. dk_ds and d2k_ds2 are arc-length derivatives of kappa.
    struct Sample {
        double x = 0, y = 0, s = 0;
        double phi = 0;
        double kappa = 0;
        double dk_ds = 0;
        double d2k_ds2 = 0;
        double drive = 1; // +1 nose leads travel, −1 this sample is driven in reverse
        double v = -1;    // target speed, m/s along travel. Negative: this shape did not publish one.
    };

private:
    struct Proj {
        double phi = 0;
        double kappa = 0;
        double dk_ds = 0;
        double d2k_ds2 = 0;
        double lat = 0;
        double remaining = 0;
        double drive = 1;
        double v = -1; // target speed at the projection. Negative: the polyline has none.
    };

    static constexpr int kCap = 64;

    static int buildPlan(const DriftFollowerConfig& config, const DriftCurveTarget& target,
                         const Pose2d& from, double v_lin, double v_ang, Sample* out,
                         double* finish_m = nullptr);
    // One timed-elastic adjustment of `base`. The base is not modified.
    // Zero means this tick should track the base itself.
    static int adjustElastic(const DriftFollowerConfig& config, const DriftCurveTarget& target,
                             const Sample* base, int n, double finish_m,
                             const Pose2d& pose, double v_lin, double v_ang,
                             double drive_hint, Sample* out);
    static Proj projectAt(const Sample* samp, int n, double rx, double ry);

    // Latched swing. Plans nothing; the orbit is already in samples_.
    ChassisCommand commandGoalTurn(const StateEstimate& state, const DriftCurveTarget& target);

    Sample samples_[kCap] {};
    int count_ = 0;
    bool planned_ = false;
    // Last timed-elastic adjustment. The base in samples_ stays put.
    Sample track_[kCap] {};
    int track_count_ = 0;
    double finish_m_ = 0;

    double i_ang_ = 0, prev_ang_ = 0;
    double i_lat_ = 0, prev_lat_ = 0, prev_dlat_ = 0;
    // Previous nose-speed command, m/s. The slew steps from this. A tile
    // seam that drops the measured speed does not drop the volts.
    double prev_v_cmd_ = 0.0;
    bool v_cmd_primed_ = false;
    bool head_primed_ = false;
    bool lat_primed_ = false;
    double goal_heading_ = 0;
    bool settled_ = false;
    MotionStuck stuck_watch_;
    double small_held_ = 0;
    double large_held_ = 0;
    // Closest approach to the target this leg, in meters. dist_armed_ is
    // false until the first compute() of the plan. saw_motion_ is set once
    // forward speed exceeds settle_speed, so a start from rest is not a stop.
    bool dist_armed_ = false;
    bool saw_motion_ = false;
    double start_dist_m_ = 0;
    double best_dist_m_ = 0;
    double drive_hint_ = 0;
    double a_actual_lat_ = 0;
    // Yaw-rate feedforward from the previous tick. A reseeded band can
    // step the curvature; the yaw-acceleration limit does not allow the
    // command to follow that step.
    double omega_hold_ = 0;
    bool omega_primed_ = false;
    const IFeedForward* model_ = nullptr;
};
