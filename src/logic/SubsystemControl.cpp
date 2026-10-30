#include "logic/SubsystemControl.h"
#include "logic/MatchLoader.h"
#include "logic/RobotMacro.h"
#include <vector>

namespace {

bool g_owns_drive = false;
std::vector<MacroSlot*> g_slots;
bool g_builtins = false;

class GoalScoreSlot : public MacroSlot {
public:
    bool running() const override { return goalScoreRunning(); }
    void cancel() override { cancelGoalScore(); }
    GoalScoreOutput apply(const SubsystemCommand& command,
                          const ButtonState& buttons,
                          const CascadeReading& cascade,
                          const DistanceReading& front,
                          const Pose2d& center_in,
                          const StateEstimate& state,
                          uint32_t now_ms,
                          IMacroController& macros,
                          const IFeedForward* model) override {
        return liveGoalScore().apply(command, buttons, cascade, front, center_in, state,
                                     now_ms, macros, model);
    }
};

class MatchLoadSlot : public MacroSlot {
public:
    bool running() const override { return matchLoaderRunning(); }
    void cancel() override { cancelMatchLoader(); }
    GoalScoreOutput apply(const SubsystemCommand& command,
                          const ButtonState& buttons,
                          const CascadeReading& cascade,
                          const DistanceReading& front,
                          const Pose2d& center_in,
                          const StateEstimate& state,
                          uint32_t now_ms,
                          IMacroController& macros,
                          const IFeedForward* model) override {
        (void)cascade;
        (void)front;
        (void)center_in;
        return liveMatchLoader().apply(command, buttons, state, now_ms, model, macros);
    }
};

void ensureBuiltins() {
    if (g_builtins) return;
    g_builtins = true;
    static GoalScoreSlot score;
    static MatchLoadSlot load;
    // Score first, so a scoring request takes the chassis from a load.
    g_slots.insert(g_slots.begin(), &load);
    g_slots.insert(g_slots.begin(), &score);
}

GoalScoreOutput passthrough(const SubsystemCommand& command) {
    GoalScoreOutput out;
    out.command = command;
    return out;
}

} // namespace

void registerRobotMacro(MacroSlot* slot) {
    if (slot == nullptr) return;
    for (const MacroSlot* have : g_slots) {
        if (have == slot) return;
    }
    g_slots.push_back(slot);
}

GoalScoreOutput runRobotMacros(const SubsystemCommand& command,
                               const ButtonState& buttons,
                               const CascadeReading& cascade,
                               const DistanceReading& front,
                               const Pose2d& center_in,
                               const StateEstimate& state,
                               uint32_t now_ms,
                               IMacroController& macros,
                               const IFeedForward* model) {
    ensureBuiltins();
    SubsystemCommand current = command;
    GoalScoreOutput chosen = passthrough(command);
    bool owned = false;
    for (MacroSlot* slot : g_slots) {
        const GoalScoreOutput step = slot->apply(
            current, buttons, cascade, front, center_in, state, now_ms, macros, model);
        if (slot->running()) {
            if (!owned) {
                owned = true;
                chosen = step;
                current = step.command;
            } else {
                slot->cancel();
            }
        } else if (!owned) {
            chosen = step;
            current = step.command;
        }
    }
    g_owns_drive = chosen.drive_active;
    return chosen;
}

void cancelRobotMacros() {
    ensureBuiltins();
    for (MacroSlot* slot : g_slots) slot->cancel();
    g_owns_drive = false;
}

bool macroOwnsDrive() { return g_owns_drive; }

GoalScoreOutput applySubsystemCommand(IDriveBase& drive,
                                      IMacroController& macros,
                                      const SubsystemCommand& commanded,
                                      const ButtonState& buttons,
                                      const Pose2d& center_in,
                                      const StateEstimate& state,
                                      const DistanceReading& front,
                                      uint32_t now_ms,
                                      const IFeedForward* model) {
    const GoalScoreOutput out = runRobotMacros(
        commanded, buttons, drive.readCascade(), front, center_in, state, now_ms, macros, model);
    drive.applySubsystems(out.command);
    return out;
}

GoalScoreOutput controlSubsystems(IDriveBase& drive,
                                  IMacroController& macros,
                                  const RobotReading& reading,
                                  const CommandVector& /*commands*/,
                                  const Pose2d& center_in,
                                  const StateEstimate& state,
                                  uint32_t now_ms,
                                  const IFeedForward* model) {
    const SubsystemCommand sub = macros.processSubsystems(reading.buttons, now_ms);
    return applySubsystemCommand(drive, macros, sub, reading.buttons, center_in, state,
                                 reading.sensors.d1(), now_ms, model);
}

void actuateSubsystems(IDriveBase& drive, const SubsystemCommand& sub) {
    drive.applySubsystems(sub);
}
