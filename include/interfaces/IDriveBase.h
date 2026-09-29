#pragma once
/* ---------------------------------------------------------------------------
 * IDriveBase  --  the physical robot chassis + peripherals.
 *
 * This is the ONLY place in the codebase that is allowed to know how many
 * wheels/motors exist and how they are arranged. It converts:
 *   - its own encoders + IMU  --> a chassis-frame ChassisObservation  (fwd kin.)
 *   - a chassis-frame ChassisCommand --> per-actuator voltages         (inv kin.)
 * Everything above this interface (estimator, controller, main) is therefore
 * completely drivebase-agnostic. Descendants: TankDrive, XDrive, ...
 * ------------------------------------------------------------------------- */

#include "core/Types.h"

class IDriveBase {
public:
    virtual ~IDriveBase() = default;

    // --- Lifecycle ---
    virtual void initialize() = 0;
    virtual void calibrateSensors() = 0;

    // --- Human input ---
    virtual JoystickData readJoystick() = 0;
    virtual ButtonState  readButtons() = 0;

    // --- Sensing (forward kinematics) ---
    // Produce a chassis-frame observation. prev_heading_rad is only consulted in
    // IMU-fallback mode, where heading is integrated from wheel deltas.
    virtual ChassisObservation readObservation(double prev_heading_rad) = 0;

    // Ports::D1..D4, written into out[0..3]. Invalid beams are marked valid=false.
    virtual void readDistances(DistanceReading (&out)[kDistanceChannels]) = 0;

    // --- Actuation (inverse kinematics + per-actuator safety) ---
    // Mix the chassis command down to actuators, apply hardware safety
    // (voltage clamp, current limit, stick-zero brake, battery scaling) and spin.
    // Returns the chassis effort actually applied (post-safety, pre battery
    // scaling) so it can be logged/recorded faithfully.
    virtual ChassisCommand applyDrive(const ChassisCommand& cmd, const ChassisObservation& obs) = 0;
    virtual void applySubsystems(const SubsystemCommand& sub) = 0;
    virtual void stop() = 0;

    // --- Heading alignment for autonomous start ---
    // Offset the heading source so the current raw heading reads as target_rad.
    virtual void alignHeadingTo(double target_rad) = 0;

    // End of a control tick. Hardware drives leave this empty. The simulation
    // advances its plant by dt seconds and redraws display_pose (the estimator).
    // particles may be a sample of the localizer's hypotheses; count 0 draws none.
    virtual void service(double dt, const Pose2d& display_pose,
                         const Pose2d* particles = nullptr, int particle_count = 0) {
        (void)dt;
        (void)display_pose;
        (void)particles;
        (void)particle_count;
    }
};
