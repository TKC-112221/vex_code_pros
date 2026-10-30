#pragma once
/* ---------------------------------------------------------------------------
 * LocalizationOverride  --  constraints the encoder step does not know.
 *
 * ImuLinearDelta still produces the sample, and the estimator still
 * integrates it. This pass runs after that step and may replace it:
 *
 *   A field wall takes the speed aimed through it.
 *   A bumper spike seats the front middle on the goal that was in front.
 *   A latched goal keeps that point on the goal. The geometric center
 *   rides the orbit of half a body length, and the chassis can only yaw.
 *
 * pose_m and carried_m are the tracking point. The deltas written back are
 * still the geometric center's travel, which is what the integrator expects.
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
    // Specific force the wheel step and the skid formula do not explain, in G.
    // A commanded brake stays small. A hit does not.
    double surprise_g = 0;
};

struct LocalizationOverride {
    // Install seated_in and drop any travel still waiting to be integrated.
    bool snap = false;
    // Drop that waiting travel, but keep the deltas written on the sample.
    // Used while a goal orbit replaces the free step.
    bool replace = false;
    Pose2d seated_in{}; // inches, front middle. The frame overridePose takes. Valid with snap.
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
