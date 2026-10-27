#pragma once
/* ---------------------------------------------------------------------------
 * SensorSavitzkyGolay  --  causal Savitzky-Golay on one SensorDict.
 *
 * Each channel keeps kSavitzkyGolayWindow samples and replaces the newest
 * with the value of a degree-kSavitzkyGolayDegree least-squares polynomial
 * at that point. A steady value or a steady rate is reproduced with no lag.
 * A change in acceleration is averaged across the window, so the memory of
 * the fit outlasts one new sample. For the constants below, white-noise
 * standard deviation is multiplied by about 0.421. The first (degree + 1) samples of
 * a channel are returned unchanged: a polynomial through that many points
 * passes through the newest sample, so smoothing starts on the next one.
 *
 * Heading and distance beams are shifted by this tick's measured motion
 * before the fit, same as SensorKalman, so the polynomial sees the residual
 * rather than the travel. Wheel-travel increments and encoder positions stay
 * raw. An invalid distance beam is skipped; a full window of misses in a row
 * drops that beam's memory so the next return starts clean.
 *
 * Settings::ControlToggles::SAVITZKY_GOLAY_FILTER selects this over the
 * 3-sample Kalman.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "core/Tuning.h"

// Control ticks per fit (the loop is 10 ms) and polynomial degree.
// Degree 1 tracks a steady rate. The noise gain is the root-sum-square
// of the fit weights: about 0.421 for this pair.
constexpr int kSavitzkyGolayWindow = SensorFilter::SAVITZKY_WINDOW;
constexpr int kSavitzkyGolayDegree = SensorFilter::SAVITZKY_DEGREE;

void applySensorSavitzkyGolay(SensorDict& sensors);
void resetSensorSavitzkyGolay();
