#pragma once
/* ---------------------------------------------------------------------------
 * ContactPhysics  --  forces after a commanded motion has become body speed.
 *
 * Wheel voltage is turned into a body velocity first. This layer then
 * pushes on that motion for one control period:
 *
 *   A field wall pushes at each bumper corner that has crossed it, along
 *   the inward normal, at that corner. The push cancels that corner's
 *   into-wall speed, so a square hit stops and an off-center hit yaws.
 *
 *   The goal snap is last, after the commanded motion and the wall push,
 *   and it runs in every mode. A front edge on a goal is seated on that
 *   point and the commanded yaw orbits it. The snap replaces the slice,
 *   so a drive command cannot push through the point. Backing up lets go.
 *
 * dt <= 0 does not integrate. It only slides a pose that was placed past a
 * wall back inside and drops the velocity component still aimed through it.
 * ------------------------------------------------------------------------- */

struct ContactBody {
    double x_m = 0;
    double y_m = 0;
    double theta = 0;   // radians, unwrapped. 0 = +y, CCW positive
    double v_fwd = 0;   // m/s, body forward
    double v_lat = 0;   // m/s, body left
    double omega = 0;   // rad/s, CCW
};

struct ContactStep {
    double forward_m = 0; // body-forward distance the pose actually moved
    double yaw_rad = 0;   // unwrapped heading change over dt
};

// Mass and yaw scale are the same rigid body the motor step integrates.
// lateral_friction_n is roller Coulomb. skid_tau_sec is the viscous bleed.
// speed_limit_mps is a safety ceiling, wider than the motor's free speed.
struct ContactParams {
    double skid_tau_sec = 0.3;
    double speed_limit_mps = 4.0;
    double mass_kg = 6.0;
    double yaw_inertia_scale = 1.0;
    double lateral_friction_n = 0.0;
};

ContactStep stepContacts(ContactBody& body, double dt, const ContactParams& params);

// The pose and the body velocity are already the end of a step. A corner
// past a wall loses its into-wall speed, and the center slides back inside.
// Heading is left alone. This does not integrate, bleed skid, or orbit a goal.
// The one-argument form uses the compiled MockPlant mass and friction.
void holdWallContact(ContactBody& body, const ContactParams& params);
void holdWallContact(ContactBody& body);
