#pragma once
/* ---------------------------------------------------------------------------
 * IMacroController  --  button macros not tied to the drive target: the replay
 * abort test and subsystem (intake/roller/pneumatics) behavior. Mother
 * interface so alternative subsystem mappings share one contract.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include <cstdint>

class IMacroController {
public:
    virtual ~IMacroController() = default;

    virtual bool isAbortRequested(const ButtonState& btn) const = 0;
    virtual void notifyAbort(uint32_t now_ms) = 0;

    // Decode live buttons into a subsystem command (may latch pneumatics).
    virtual SubsystemCommand processSubsystems(const ButtonState& btn, uint32_t now_ms) = 0;
    // Decode a recorded bitmask into a subsystem command (replay path).
    virtual SubsystemCommand decodeRecorded(uint8_t bitmask) const = 0;

    // Last live decode, as the shared mechanism mode. Recorded replay does
    // not update this; it applies the bitmask mix directly.
    virtual SubsystemIntent subsystemIntent() const { return {}; }
};
