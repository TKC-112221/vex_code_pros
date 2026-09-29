#pragma once
/* ---------------------------------------------------------------------------
 * ICommandProcessor  --  mid-pipeline command filter between user-input
 * override and motor mapping. Default implementation returns the input
 * unchanged. Swap in a richer processor (ghost pose, face/pivot assists,
 * cascaded feedback) without changing surrounding stages.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"

class ICommandProcessor {
public:
    virtual ~ICommandProcessor() = default;

    virtual void reset() {}

    // reading  : latest unified sense package
    // commands : post-override surge/sway/yaw
    // estimate : latest localization snapshot (may lag slightly; async)
    virtual CommandVector process(const RobotReading& reading,
                                  const CommandVector& commands,
                                  const StateEstimate& estimate) = 0;
};
