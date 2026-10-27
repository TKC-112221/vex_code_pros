#pragma once
/* ---------------------------------------------------------------------------
 * Ports.h  --  The Wiring: PROS V5 smart-port + ADI assignments.
 *
 * PROS ports are plain integers (smart ports 1-21, ADI ports 1-8 / 'A'-'H').
 * Motor reversal is expressed at construction by negating the port, so these
 * stay positive and the drivebase applies the sign. Only a concrete DriveBase
 * should read these.
 * ------------------------------------------------------------------------- */

namespace Ports {
    // Drive (smart ports)
    constexpr int L1 = 14;
    constexpr int L2 = 15;
    constexpr int L3 = 16;
    constexpr int R1 = 13;
    constexpr int R2 = 12;
    constexpr int R3 = 11;

    // Sensors
    constexpr int IMU = 10;
    // Distance sensors. Beam geometry lives in Robot::Range::MOUNT
    // (D1 front, D2 right, D3 rear, D4 left).
    constexpr int D1 = 17;
    constexpr int D2 = 18;
    constexpr int D3 = 19;
    constexpr int D4 = 20;

    // Subsystems (smart ports).
    // MAIN1 and MAIN2 share a differential: the same spin powers the cascade,
    // the opposite spin powers the intake. SUB1 and SUB2 are held off.
    constexpr int MAIN1 = 4;
    constexpr int MAIN2 = 5;
    constexpr int SUB1  = 8;
    constexpr int SUB2  = 9;

    // Pneumatics (ADI ports: 'A'=1 ... 'H'=8)
    constexpr int PNEUMATICS_G = 7;   // 'G'
    constexpr int PNEUMATICS_B = 2;   // 'B'

    // Holonomic drives (XDrive / ParenthesisDrive) reuse the six drive ports.
    constexpr int FL     = L1;   // front-left  omni (45 deg)
    constexpr int FR     = R1;   // front-right omni (45 deg)
    constexpr int BL     = L2;   // back-left   omni (45 deg)
    constexpr int BR     = R2;   // back-right  omni (45 deg)
    constexpr int SIDE_L = L3;   // mid-left  forward traction wheel (parenthesis)
    constexpr int SIDE_R = R3;   // mid-right forward traction wheel (parenthesis)
}
