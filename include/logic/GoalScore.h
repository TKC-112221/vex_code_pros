#pragma once
/* ---------------------------------------------------------------------------
 * GoalScore  --  drive to a goal and score on the cascade.
 *
 * With no aim point, a goal within 30 degrees of the robot's heading counts
 * as half its distance, and the smallest of those wins. aimAt() makes the
 * next start use the goal closest to that field point instead.
 * Mechanisms::score(x, y) sets the aim. score() with no point clears it.
 * score(x, y, approach_deg) drives in with the nose on that field heading,
 * 0 along +Y. score(x, y, {}, detach_deg) leaves the approach empty, so the
 * robot comes from whichever side the bumper is already on, then rotates to
 * detach_deg while scoring and backs away on that heading. With no detach
 * heading, it leaves facing the way it latched.
 *
 * Each goal remembers a cascade height. It starts at that spot's
 * base_height_in. Releasing the claw on a goal adds 6.5 inches.
 *
 * Starting inside the 4 inch mark backs out to it. The aligner stays off
 * until the cascade has settled, so the robot cannot grab the goal before
 * the height is finished. That includes the drive into the goal.
 * The goal being scored is not a keep-out: the 4 inch stop and the latch
 * both sit inside that circle, and treating it as one backs the robot out
 * before it can drive in.
 *
 * 1. Plan a path and drive to 4 inches from the closest goal, facing it.
 *    An approach heading parks the bumper on that side. With none, the
 *    path is the straight line from the rotation centre to the goal.
 * 2. Bring the cascade to 11 inches below the remembered height. The move
 *    waits until the drive has just enough time left for the lift to
 *    finish first.
 * 3. When the bumper reaches that point, search the stack, including
 *    while the final yaw is still turning. Lower until the sensor hits,
 *    raise until the sensor loses the stack, then raise another 10
 *    inches. If the sensor never hits, raise to the goal's base height.
 *    The drive into the goal starts on the tick the follower settles.
 *    The height search keeps running, and the drop waits until the
 *    cascade is holding that height.
 * 4. Drive into the goal along the approach. While the cascade drops,
 *    rotate to the detach heading when one was given. Then release the
 *    claw, hold the goal grab off, back up on that heading (re-planning
 *    if the chassis is still stuck on the post), hand the drive to the
 *    driver, and lower the cascade to height 0.
 *
 * A drive that is stuck far from the goal, or an approach that never
 * gets close, cancels the macro so the routine can run the next command.
 * Contact at the goal is not that failure. The latch, the drop, and the
 * backup keep their own timers and finish the score.
 *
 * D-pad Right starts it. Right again, or L1 / L2, cancels it. Autonomous
 * starts it by holding SubsystemCommand::score, which Mechanisms::score()
 * sets until the macro finishes. The numbers below are the ones to change
 * on the real robot.
 * ------------------------------------------------------------------------- */

#include "control/SegmentFollower.h"
#include "core/Types.h"
#include "field/FieldPoints.h"
#include <cstdint>

class IFeedForward;
class IMacroController;

namespace GoalScoreTune {
    constexpr double kStandoffIn = 4.0;
    constexpr double kBelowIn = 11.0;
    constexpr double kSenseIn = 6.0;
    constexpr double kClearanceIn = 10.0;
    constexpr double kDropIn = 2.0;
    constexpr double kBackupIn = 6.0;
    constexpr double kPieceIn = 6.5;
    // A goal this close to the nose, in degrees, is treated as nearer.
    constexpr double kHeadingConeDeg = 30.0;
    constexpr double kHeadingDistScale = 0.5;
    // Bumper distance to the standoff pose that starts the height search.
    // The final yaw can still be turning.
    constexpr double kMarkIn = 3.0;
    // Nose error, radians, that finishes the latched turn onto the score
    // heading. The drop waits for this before backing up.
    constexpr double kScoreFaceRad = 0.12;
    // Pre-raise time is the brake-envelope estimate times this scale, plus
    // the lead. A larger scale starts the lift earlier.
    constexpr double kLiftTimeScale = 1.2;
    constexpr double kLiftLeadSec = 0.05;
    constexpr int kRaisePct = 70;
    constexpr bool kClawReleased = true;

    constexpr uint32_t kLossMs = 40;
    constexpr uint32_t kStandoffTimeoutMs = 8000;
    constexpr uint32_t kProbeTimeoutMs = 8000;
    constexpr uint32_t kLatchTimeoutMs = 2000;
    // The lift never reached the height the search asked for. Drop anyway.
    constexpr uint32_t kRaiseTimeoutMs = 4000;
    constexpr uint32_t kDropTimeoutMs = 2000;
    constexpr uint32_t kRetreatTimeoutMs = 6000;
    // Still worth latching. Farther than this, a stuck or timed-out
    // approach ends the macro instead of driving the rest of the period.
    constexpr double kRecoverIn = 12.0;
}

struct GoalScoreOutput {
    SubsystemCommand command{};
    bool drive_active = false;
    ChassisCommand drive{};
};

class GoalScore {
public:
    GoalScoreOutput apply(const SubsystemCommand& driver,
                          const ButtonState& buttons,
                          const CascadeReading& sense,
                          const DistanceReading& front,
                          const Pose2d& center_in,
                          const StateEstimate& state,
                          uint32_t now_ms,
                          IMacroController& macros,
                          const IFeedForward* model);

    bool running() const { return phase_ != Phase::Idle; }
    bool releasedClaw() const { return released_claw_; }
    // A new score has not latched yet. The previous release must not open
    // the claw during the approach.
    void clearReleasedClaw() { released_claw_ = false; }
    // The drive was given up because it could not reach the goal. Contact
    // while scoring does not set this. An autonomous score ends the leg on it.
    bool driveStuck() const { return block_abort_; }
    const char* phaseName() const;
    void cancel();

    // The next start scores the goal closest to this field point, in inches.
    // Cleared when that start begins.
    void aimAt(double x_in, double y_in);
    void clearAim();

    // The next start drives in with the nose on this field heading, in
    // radians. 0 faces +Y. Cleared when that start begins.
    void approachFrom(double heading_rad);
    void clearApproach();

    // After the bumper latches, the nose rotates to this field heading
    // while the cascade scores, then the backup leaves on it. Cleared when
    // that start begins. With none, the robot leaves facing the latch.
    void detachAt(double heading_rad);
    void clearDetach();

private:
    enum class Phase { Idle, Approach, Probe, Latch, Drop, Retreat };
    // Down searches for the stack. Up climbs off it. Hold is a height target.
    enum class Scan { Down, Up, Hold };

    void begin(const Pose2d& center, const StateEstimate& state, uint32_t now_ms);
    void enterApproach(const Pose2d& center, const StateEstimate& state, uint32_t now_ms);
    void enterProbe(const CascadeReading& sense, const DistanceReading& front, uint32_t now_ms);
    void enterLatch(const Pose2d& center, const Field::Spot& goal, uint32_t now_ms);
    void enterDrop(const CascadeReading& sense, uint32_t now_ms);
    void enterRetreat(const StateEstimate& state, uint32_t now_ms, IMacroController& macros,
                      SubsystemCommand& out);
    // 6 inch reverse on the leave heading. Re-armed when the follower is stuck
    // still on the goal.
    void armRetreat(const StateEstimate& state);
    // `avoid` keeps every other goal's circle. The goal being scored is
    // never one of them. The latch passes false and drives straight in.
    void armDrive(const Pose2d& pose, bool reverse, double line_in, bool avoid);
    bool timedOut(uint32_t now_ms, uint32_t limit_ms) const;
    bool closeToGoal(const Pose2d& pose, double inches) const;
    void abandon(GoalScoreOutput& out);
    bool latched() const;
    // True when no detach heading was given, or the nose is on it and slow.
    bool scoreFaced(const StateEstimate& state) const;
    bool raiseReady(const CascadeReading& sense) const;
    // The stack search. It keeps running after the approach settles, so the
    // latch drive is not waiting on the lift. The probe clock is search_ms_.
    void searchStack(SubsystemCommand& cmd, const CascadeReading& sense,
                     const DistanceReading& front, uint32_t now_ms);

    Phase phase_ = Phase::Idle;
    Scan scan_ = Scan::Down;
    uint32_t phase_ms_ = 0;
    uint32_t search_ms_ = 0;
    bool prev_right_ = false;
    bool released_claw_ = false;
    bool captured_ = false;
    bool loss_armed_ = false;
    bool hit_armed_ = false;
    bool backup_done_ = false;
    bool lift_armed_ = false;
    bool block_abort_ = false;
    uint32_t loss_ms_ = 0;
    uint32_t hit_ms_ = 0;
    int goal_ = -1;
    bool aim_set_ = false;
    double aim_x_ = 0;
    double aim_y_ = 0;
    bool approach_set_ = false;
    double approach_rad_ = 0;
    bool approach_live_ = false;
    double approach_live_rad_ = 0;
    bool detach_set_ = false;
    double detach_rad_ = 0;
    bool detach_live_ = false;
    double detach_live_rad_ = 0;
    double lift_target_ = 0;
    double drop_target_ = 0;
    double approach_in_ = 0;
    const IFeedForward* model_ = nullptr;
    SegmentFollower nav_{};
};

GoalScore& liveGoalScore();
inline void cancelGoalScore() { liveGoalScore().cancel(); }
inline bool goalScoreRunning() { return liveGoalScore().running(); }
inline bool goalScoreDriveStuck() { return liveGoalScore().driveStuck(); }
inline bool goalScoreReleasedClaw() { return liveGoalScore().releasedClaw(); }
inline void goalScoreClearReleased() { liveGoalScore().clearReleasedClaw(); }
inline const char* goalScorePhaseName() { return liveGoalScore().phaseName(); }
inline void goalScoreAim(double x_in, double y_in) { liveGoalScore().aimAt(x_in, y_in); }
inline void goalScoreClearAim() { liveGoalScore().clearAim(); }
inline void goalScoreApproach(double heading_rad) { liveGoalScore().approachFrom(heading_rad); }
inline void goalScoreClearApproach() { liveGoalScore().clearApproach(); }
inline void goalScoreDetach(double heading_rad) { liveGoalScore().detachAt(heading_rad); }
inline void goalScoreClearDetach() { liveGoalScore().clearDetach(); }

// Polyline the score drive would take from `from`, in the same centre frame
// the segment follower draws. The approach stops short of the goal, then
// the line goes onto the goal. A detach heading swings the centre around
// the goal to that nose, then back along the 6 inch retreat. `leave` is
// the bumper pose the next command starts from. Returns 0 when no goal fits.
int previewGoalScorePath(const Pose2d& from, bool has_aim, double aim_x, double aim_y,
                         bool has_approach, double approach_rad,
                         bool has_detach, double detach_rad,
                         Pose2d* out, int cap, Pose2d* leave);
