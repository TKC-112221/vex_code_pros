#pragma once
/* ---------------------------------------------------------------------------
 * MatchLoader  --  drive the front bumper onto a match loader.
 *
 * With no aim point, a loader within 30 degrees of the robot's heading
 * counts as half its distance. aimAt() makes the next start use the loader
 * closest to that field point instead. Mechanisms::matchLoad(x, y) sets
 * the aim. matchLoad() with no point clears it.
 *
 * The loaders sit on the near wall, five feet either side of the origin.
 * The approach is always 180 degrees: the nose faces -Y, into that wall,
 * and the last foot of the path is straight along that heading. An aim
 * point only chooses which loader. Other goal points stay
 * keep-outs, the same circles the score drive uses.
 *
 * B starts it. B again cancels it. A+B is the recording load and does not
 * start a drive. A routine starts the same drive with mech.matchLoad(), which
 * holds kMacroMatchLoad until the drive finishes. The intake and cascade
 * commands pass through. The claw opens when the drive starts. When the
 * drive settles, pneumatic B goes low, which closes the claw, and the
 * robot backs up 6 inches. Stuck against the loader does the same.
 * Stuck far away closes the claw and ends the macro. A backup that
 * cannot move ends the macro too, so the routine can continue.
 * ------------------------------------------------------------------------- */

#include "control/SegmentFollower.h"
#include "core/Types.h"
#include "logic/GoalScore.h"
#include <cstdint>

class IFeedForward;
class IMacroController;

namespace MatchLoaderTune {
    // 180 degrees. 0 is +Y, so this faces the near wall. The approach
    // heading is this value on every start.
    constexpr double kFaceRad = Robot::PI;
    // Straight finish into the loader, inches, along kFaceRad.
    constexpr double kApproachIn = 12.0;
    // Reverse along the nose after the claw closes, inches.
    constexpr double kBackupIn = 6.0;
    // Same nose cone the score drive uses.
    constexpr double kHeadingConeDeg = 30.0;
    constexpr double kHeadingDistScale = 0.5;
    constexpr uint32_t kTimeoutMs = 12000;
    // Bumper gap that still counts as arriving when the chassis is stuck.
    constexpr double kContactIn = 6.0;
    // Pneumatic B high releases the claw. The macro opens it on start
    // and closes it once the bumper is on the loader.
    constexpr bool kClawReleased = true;
    constexpr bool kClawClosed = false;
}

class MatchLoader {
public:
    GoalScoreOutput apply(const SubsystemCommand& driver,
                          const ButtonState& buttons,
                          const StateEstimate& state,
                          uint32_t now_ms,
                          const IFeedForward* model,
                          IMacroController& macros);

    bool running() const { return running_; }
    // The drive asked the chassis to move and it did not. An autonomous
    // match load gives the leg up on it.
    bool driveStuck() const { return nav_.stuck(); }
    void cancel();

    // The next start drives to the loader closest to this field point, in
    // inches. Cleared when that start begins.
    void aimAt(double x_in, double y_in);
    void clearAim();

    int samplePath(const Pose2d& from, Pose2d* out, int cap) const;

private:
    void begin(const Pose2d& pose, uint32_t now_ms);
    void startBackup(const Pose2d& pose);

    bool running_ = false;
    bool backing_ = false;
    bool prev_b_ = false;
    bool aim_set_ = false;
    double aim_x_ = 0;
    double aim_y_ = 0;
    uint32_t start_ms_ = 0;
    const IFeedForward* model_ = nullptr;
    SegmentFollower nav_{};
};

MatchLoader& liveMatchLoader();
inline void cancelMatchLoader() { liveMatchLoader().cancel(); }
inline bool matchLoaderRunning() { return liveMatchLoader().running(); }
inline bool matchLoaderDriveStuck() { return liveMatchLoader().driveStuck(); }
inline void matchLoaderAim(double x_in, double y_in) { liveMatchLoader().aimAt(x_in, y_in); }
inline void matchLoaderClearAim() { liveMatchLoader().clearAim(); }

int matchLoaderPath(const Pose2d& from, Pose2d* out, int cap);
