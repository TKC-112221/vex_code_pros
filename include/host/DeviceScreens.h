#pragma once
/* ---------------------------------------------------------------------------
 * DeviceScreens  --  text the host build shows for the V5 brain (LLEMU,
 * 8x40) and the controller LCD (3x20). The field window paints these.
 * The V5 image does not compile this directory.
 * ------------------------------------------------------------------------- */

#include <cstdarg>

namespace devicescreens {

constexpr int kBrainLines = 8;
constexpr int kBrainCols = 40;
constexpr int kCtrlLines = 3;
constexpr int kCtrlCols = 20;

struct BrainView {
    char lines[kBrainLines][kBrainCols + 1];
};

struct ControllerView {
    char lines[kCtrlLines][kCtrlCols + 1];
};

void copyBrain(BrainView& out);
void copyController(ControllerView& out);

// Place text at a column. Characters past the line stay as they were.
void writeController(int line, int col, const char* text);

}

extern "C" void host_lcd_vprint(int line, const char* fmt, va_list args);
