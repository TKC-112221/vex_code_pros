/* ---------------------------------------------------------------------------
 * DeviceScreens.cpp  --  host copies of the brain and controller text.
 * ------------------------------------------------------------------------- */
#include "host/DeviceScreens.h"

#include <cstdio>
#include <cstring>
#include <mutex>

namespace {

char g_brain[devicescreens::kBrainLines][devicescreens::kBrainCols + 1]{};
char g_ctrl[devicescreens::kCtrlLines][devicescreens::kCtrlCols + 1]{};
bool g_ctrl_ready = false;

std::mutex& mu() {
    static std::mutex m;
    return m;
}

void ensureController() {
    if (g_ctrl_ready) return;
    g_ctrl_ready = true;
    for (int line = 0; line < devicescreens::kCtrlLines; ++line) {
        for (int col = 0; col < devicescreens::kCtrlCols; ++col)
            g_ctrl[line][col] = ' ';
        g_ctrl[line][devicescreens::kCtrlCols] = '\0';
    }
}

}  // namespace

namespace devicescreens {

void copyBrain(BrainView& out) {
    std::lock_guard<std::mutex> lock(mu());
    for (int line = 0; line < kBrainLines; ++line) {
        std::memcpy(out.lines[line], g_brain[line], kBrainCols);
        out.lines[line][kBrainCols] = '\0';
    }
}

void copyController(ControllerView& out) {
    std::lock_guard<std::mutex> lock(mu());
    ensureController();
    for (int line = 0; line < kCtrlLines; ++line)
        std::memcpy(out.lines[line], g_ctrl[line], kCtrlCols + 1);
}

void writeController(int line, int col, const char* text) {
    if (text == nullptr) return;
    if (line < 0 || line >= kCtrlLines) return;
    if (col < 0) col = 0;
    std::lock_guard<std::mutex> lock(mu());
    ensureController();
    char* row = g_ctrl[line];
    for (int i = 0; text[i] != '\0' && col + i < kCtrlCols; ++i)
        row[col + i] = text[i];
}

}  // namespace devicescreens

extern "C" void host_lcd_vprint(int line, const char* fmt, va_list args) {
    if (fmt == nullptr) return;
    if (line < 0 || line >= devicescreens::kBrainLines) return;
    char buf[160];
    if (std::vsnprintf(buf, sizeof(buf), fmt, args) < 0) buf[0] = '\0';

    std::lock_guard<std::mutex> lock(mu());
    char* dst = g_brain[line];
    std::memset(dst, 0, devicescreens::kBrainCols + 1);
    for (int i = 0; i < devicescreens::kBrainCols && buf[i] != '\0' && buf[i] != '\n'; ++i)
        dst[i] = buf[i];
}
