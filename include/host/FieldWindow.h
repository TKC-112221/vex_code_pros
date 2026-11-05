#pragma once
/* ---------------------------------------------------------------------------
 * FieldWindow  --  desktop field view. The host window stacks the autonomous
 * selector over a preview of the selected route, and puts the live
 * simulation beside that column. The V5 brain, the controller, and the
 * subsystem states stay on the far side of the simulation. The V5 image
 * does not compile this directory; the brain draws its own selector beside
 * the route preview.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include <cstdint>

namespace fieldwindow {

// Sample the keyboard into the joystick and button state MockDrive returns.
void pollInput(JoystickData& joy, ButtonState& buttons);

// Open the field window if needed. truth is the plant; estimate is the
// pose the running localizer currently reports. particles is a sample of
// the filter cloud (ignored when count is 0).
void render(const Pose2d& truth, const Pose2d& estimate,
            const Pose2d* particles = nullptr, int particle_count = 0);

// Curve the controller is tracking, in field meters. While active, the
// window shows this polyline. When active goes false the last curve stays
// for 5 seconds and then clears. count 0 is not a curve.
void setFollowPath(const Pose2d* points, int count, bool active);

// Brain LCD contents, 0x00RRGGBB, row-major from the top. Copied. nullptr
// clears it so the panel shows LLEMU text again.
void setBrainFrame(const std::uint32_t* rgb, int width, int height);

// Domain randomization for the host plant. The dropdown on the field
// starts at the compiled plant, which is what the controller was tuned to.
// Each slider puts back its own compiled value. Reset plant puts them all back.
// Mass, top speed, launch acceleration, and friction are the rigid-body
// motor. Yaw inertia 1 is a uniform rectangle. Traction is the grip
// coefficient before a wheel slips. A disconnected port reports nothing.
// Distance stays invalid. The IMU holds the last heading and reports no
// rate. A wheel bank reports no travel. Reset plant connects every sensor
// again.
struct PlantOverride {
    double mass_kg = 6;
    double max_vel_mps = 3;
    double drive_accel_mps2 = 6;
    double static_friction_n = 0;
    double yaw_inertia_scale = 1;
    double yaw_friction_nm = 0;
    double skid_tau_sec = 0.3;
    double lateral_friction_n = 0;
    double traction_mu = 1;
    int voltage_delay_ticks = 0;
    bool imu_off = false;
    bool enc_left_off = false;
    bool enc_right_off = false;
    bool distance_off[kDistanceChannels] = {};
};
PlantOverride plantOverride();

// Live mechanism state for the side panel. cascade_frac is 0 at the bottom
// stop and 1 at the top. score_phase is copied. nullptr reads as Idle.
void setMechanisms(const SubsystemCommand& cmd, double cascade_frac,
                   const CascadeReading& sense, const char* score_phase);

// Create the window on the calling thread. The pipeline may draw into it
// afterwards, but it must not be the thread that creates it.
void open();

// Open the main window if needed, paint, and pump messages for hold_ms.
// False when Escape is pressed or the brain screen is clicked. Closing
// the window quits. Call only from the thread that will own the window.
bool pump(std::uint32_t hold_ms);

// Message loop for that same thread after startup. Does not return.
// The pipeline must not dispatch window messages itself.
void run();

}
