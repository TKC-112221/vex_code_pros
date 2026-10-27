#pragma once
/* ---------------------------------------------------------------------------
 * BrainSelector  --  the V5 brain screen for picking an autonomous.
 *
 * The selector fills the left of the 480x272 screen and the route preview
 * fills the right. The host build draws this pair in the field window
 * instead, so serviceBrainSelector is a no-op there. On the brain it runs
 * on its own task, not on the control tick.
 * ------------------------------------------------------------------------- */

namespace auton {

// Poll the touchscreen and redraw when the choice changes. paint is false
// while the startup clip owns the display.
void serviceBrainSelector(bool paint);

} // namespace auton
