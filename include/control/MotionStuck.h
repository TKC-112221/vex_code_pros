#pragma once
/* ---------------------------------------------------------------------------
 * MotionStuck  --  a motion request the chassis is not answering.
 *
 * want_lin asks for translation. want_yaw asks for a pivot and is only used
 * when the leg is not also asking to drive. Forward speed answers the first,
 * yaw rate the second. A hold that asks for neither keeps the timer at zero,
 * so a score sequence can sit still while the lift moves. After STUCK_SEC
 * the caller gives the leg up and the routine starts the next command.
 * ------------------------------------------------------------------------- */

#include "core/Specs.h"
#include "core/Tuning.h"
#include <cmath>

class MotionStuck {
public:
    void reset() {
        held_sec_ = 0.0;
        stuck_ = false;
    }

    bool update(bool want_lin, bool want_yaw, double v_lin, double v_ang) {
        const double lin = std::fabs(v_lin);
        const double yaw = std::fabs(v_ang);
        const bool lin_ok = std::isfinite(lin) && lin > Motion::SETTLE_SPEED_MPS;
        const bool yaw_ok = std::isfinite(yaw) && yaw > Motion::STUCK_YAW_RAD_S;
        const bool moving = want_lin ? lin_ok : yaw_ok;
        if ((!want_lin && !want_yaw) || moving) {
            held_sec_ = 0.0;
            stuck_ = false;
            return false;
        }
        held_sec_ += System::DT_SEC;
        if (held_sec_ >= Motion::STUCK_SEC) stuck_ = true;
        return stuck_;
    }

    bool stuck() const { return stuck_; }

private:
    double held_sec_ = 0.0;
    bool stuck_ = false;
};
