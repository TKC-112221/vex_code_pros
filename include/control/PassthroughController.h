#pragma once
/* ---------------------------------------------------------------------------
 * PassthroughController  --  default ICommandProcessor: returns commands
 * unchanged. Replace with a ghost-pose / assist / cascaded processor later
 * without touching the surrounding pipeline stages.
 * ------------------------------------------------------------------------- */

#include "interfaces/ICommandProcessor.h"

class PassthroughController : public ICommandProcessor {
public:
    void reset() override {}

    CommandVector process(const RobotReading& /*reading*/,
                          const CommandVector& commands,
                          const StateEstimate& /*estimate*/) override {
        return commands;
    }
};
