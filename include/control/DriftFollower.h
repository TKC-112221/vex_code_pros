#pragma once
/* ---------------------------------------------------------------------------
 * DriftFollower  --  one planned curve, then a follower.
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
 * Heading offset is the slip model. v_lat is the measured forward
 * velocity projected on the path's left normal. v_ref is the planned
 * travel speed, and κ and its arc-length derivatives come from the
 * stored path:
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
 * Arcsin, with u = a_lat / a_max inside (−1, 1). On the clamp the
 * offset is flat, so both derivatives are zero:
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
 * toward the path).
 *
 * Longitudinal volts match DriftCurve: remaining length sets the speed,
 * and the bound model supplies the linear feedforward.
 * ------------------------------------------------------------------------- */

#include "control/DriftCurve.h"

// Atan maps a_lat through drift_accel. Arcsin maps it through a_max.
enum class DriftSlipModel { Atan, Arcsin };

struct DriftFollowerConfig : DriftCurveConfig {
    // DriftSlipModel slip = DriftSlipModel::Atan;
    DriftSlipModel slip = DriftSlipModel::Arcsin;
    // DriftCurveShape shape = DriftCurveShape::Quartic;
    DriftCurveShape shape = DriftCurveShape::Hermite;
    double k_slip = Drift::K_SLIP;           // 1/s. Subtracted from lateral accel per m/s of v_lat.
    double a_max = Drift::A_MAX;             // m/s^2. Arcsin thrust, and the atan lateral projection.
    double lat_kP = Drift::LAT_KP;           // V per m of cross-track
    double lat_kI = Drift::LAT_KI;           // V per (m·s)
    double lat_kD = Drift::LAT_KD;           // V per (m/s)
    double lat_kD2 = Drift::LAT_KD2;         // V per (m/s^2). Nonzero makes the lateral loop PIDD2.
    double lat_i_limit = Drift::LAT_I_LIMIT; // volt clamp on the lateral integrator term

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
};

class DriftFollower {
public:
    DriftFollowerConfig config;
    DriftCurveTarget target;

    void reset();

    // Chassis model for the linear volts and the yaw kV / kA. Null leaves
    // both feedforwards at zero. The pointer is not owned.
    void bindFeedforward(const IFeedForward* model) { model_ = model; }

    // state : field pose, forward speed (v_lin, m/s), yaw rate (v_ang, rad/s).
    // The first call after reset() plans from this state. Later calls track.
    ChassisCommand compute(const StateEstimate& state);

    // Heading the chassis was told to face on the last compute(), radians.
    double goalHeading() const { return goal_heading_; }

    // True inside the Boomerang pose window and within 0.2 m/s of
    // target.exit_speed. The caller decides when to drop the override.
    bool settled() const { return settled_; }

    // Polyline of the planned path. Before the first compute() this is a
    // preview from `from` and does not commit the plan. v_lin and v_ang
    // are the measured momentum a Hermite preview opens along.
    int samplePath(const Pose2d& from, Pose2d* out, int cap,
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
    };

private:
    struct Proj {
        double phi = 0;
        double kappa = 0;
        double dk_ds = 0;
        double d2k_ds2 = 0;
        double lat = 0;
        double remaining = 0;
    };

    static constexpr int kCap = 64;

    static int buildPlan(const DriftFollowerConfig& config, const DriftCurveTarget& target,
                         const Pose2d& from, double v_lin, double v_ang, Sample* out);
    static Proj projectAt(const Sample* samp, int n, double rx, double ry);

    Sample samples_[kCap] {};
    int count_ = 0;
    bool planned_ = false;

    double i_ang_ = 0, prev_ang_ = 0;
    double i_lat_ = 0, prev_lat_ = 0, prev_dlat_ = 0;
    bool head_primed_ = false;
    bool lat_primed_ = false;
    double goal_heading_ = 0;
    bool settled_ = false;
    double a_actual_lat_ = 0;
    const IFeedForward* model_ = nullptr;
};
