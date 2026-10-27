#pragma once
/* ---------------------------------------------------------------------------
 * LocalizationOverride  --  constraints the encoder step does not know.
 *
 * ImuLinearDelta still produces the sample, and the estimator still
 * integrates it. This pass runs after that step and may replace it:
 *
 *   A field wall takes the speed aimed through it.
 *   A bumper spike seats the front center on the goal that was in front.
 *   A latched goal keeps the center on the orbit whose radius is half a
 *   body length, the distance from that point to the center when the
 *   front edge is seated. The chassis can only yaw about the goal.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "estimators/ImuLinearDelta.h"
#include <cstdint>

// Measured before ImuLinearDelta::apply, so the spike test sees the
// motion that ran into the bumper rather than the step after the hit.
struct LocalizationCue {
    double front_speed = 0; // m/s of the front center
    double accel_g = 0;     // magnitude of the specific force
    double ax_g = 0;        // forward specific force
};

struct LocalizationOverride {
    // Install seated_in and drop any travel still waiting to be integrated.
    bool snap = false;
    // Drop that waiting travel, but keep the deltas written on the sample.
    // Used while a goal orbit replaces the free step.
    bool replace = false;
    Pose2d seated_in{}; // inches, the frame overridePose takes. Valid with snap.
};

// pose_m is the estimate the integrator will step from, in meters.
// carried_m is that pose plus travel already waiting to be integrated.
// A goal orbit steps from pose_m, because it discards the waiting travel.
// A wall uses carried_m, because that travel is kept.
LocalizationOverride applyLocalizationOverride(ImuLinearDelta& imu,
                                          ChassisObservation& chassis,
                                          const Pose2d& pose_m,
                                          const Pose2d& carried_m,
                                          const LocalizationCue& cue,
                                          std::uint32_t now_ms);
