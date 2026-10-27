/* ---------------------------------------------------------------------------
 * FramePlayer.cpp  --  walk a numbered PNG sequence and hold each shown
 * frame for one frame period.
 *
 * The V5 image blits onto the brain (480x272). The host build (ROBOT_HOST,
 * `make host`) blits into the brain screen of the main window. src/host is
 * not linked into the V5 image.
 * ------------------------------------------------------------------------- */
#include "platform/FramePlayer.h"
#include "platform/PngImage.h"
#include "api.h"

#include <cstdio>
#include <cstring>
#include <vector>

#if defined(ROBOT_HOST)
#include "host/FieldWindow.h"
#endif

namespace {

constexpr int kScreenW = 480;
constexpr int kScreenH = 272;
constexpr int kMaxSource = 100000;

void blitFit(const std::uint8_t* rgba, int iw, int ih, std::vector<std::uint32_t>& screen) {
    screen.assign((std::size_t)kScreenW * (std::size_t)kScreenH, 0);
    if (rgba == nullptr || iw <= 0 || ih <= 0) return;

    int dw = 0;
    int dh = 0;
    if ((long long)kScreenW * ih <= (long long)kScreenH * iw) {
        dw = kScreenW;
        dh = (int)((long long)ih * kScreenW / iw);
    } else {
        dh = kScreenH;
        dw = (int)((long long)iw * kScreenH / ih);
    }
    if (dw < 1) dw = 1;
    if (dh < 1) dh = 1;
    int ox = (kScreenW - dw) / 2;
    int oy = (kScreenH - dh) / 2;

    for (int y = 0; y < dh; ++y) {
        int sy = (int)((long long)y * ih / dh);
        if (sy >= ih) sy = ih - 1;
        for (int x = 0; x < dw; ++x) {
            int sx = (int)((long long)x * iw / dw);
            if (sx >= iw) sx = iw - 1;
            const std::uint8_t* p = rgba + ((std::size_t)sy * (std::size_t)iw + (std::size_t)sx) * 4u;
            std::uint32_t a = p[3];
            std::uint32_t r = (std::uint32_t)p[0] * a / 255u;
            std::uint32_t g = (std::uint32_t)p[1] * a / 255u;
            std::uint32_t b = (std::uint32_t)p[2] * a / 255u;
            screen[(std::size_t)(oy + y) * (std::size_t)kScreenW + (std::size_t)(ox + x)] =
                (r << 16) | (g << 8) | b;
        }
    }
}

#if defined(ROBOT_HOST)

bool presentFrame(std::uint32_t* pixels, std::uint32_t hold_ms) {
    fieldwindow::setBrainFrame(pixels, kScreenW, kScreenH);
    return fieldwindow::pump(hold_ms);
}

void endPresentation() {
    fieldwindow::setBrainFrame(nullptr, 0, 0);
    fieldwindow::pump(0);
}

#else

std::int32_t g_press0 = 0;
bool g_press_init = false;

// L1+L2+R1+R2, the same chord that aborts a routine. A failed read is not
// a press, so a missing controller does not skip the clip.
bool shoulderHalt() {
    auto down = [](pros::controller_digital_e_t button) {
        return pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER, button) == 1;
    };
    return down(pros::E_CONTROLLER_DIGITAL_L1) && down(pros::E_CONTROLLER_DIGITAL_L2)
        && down(pros::E_CONTROLLER_DIGITAL_R1) && down(pros::E_CONTROLLER_DIGITAL_R2);
}

bool userSkipped() {
    if (shoulderHalt()) return true;
    pros::screen_touch_status_s_t status = pros::c::screen_touch_status();
    if (!g_press_init) {
        g_press0 = status.press_count;
        g_press_init = true;
        return false;
    }
    return status.press_count != g_press0;
}

bool presentFrame(std::uint32_t* pixels, std::uint32_t hold_ms) {
    pros::c::screen_copy_area(0, 0, (std::int16_t)(kScreenW - 1), (std::int16_t)(kScreenH - 1),
                              pixels, (std::int32_t)kScreenW);
    std::uint32_t left = hold_ms;
    while (left > 0) {
        if (userSkipped()) return false;
        std::uint32_t slice = left > 20u ? 20u : left;
        pros::delay(slice);
        left -= slice;
    }
    return !userSkipped();
}

void endPresentation() {
    g_press_init = false;
}

#endif

}  // namespace

int playFrameSequence(const FrameSequence& seq) {
    if (seq.path_format == nullptr || seq.path_format[0] == '\0') return 0;
    int step = seq.frame_step < 1 ? 1 : seq.frame_step;

    std::uint32_t period = 0;
    if (seq.fps > 0.0 && seq.fps < 1000.0)
        period = (std::uint32_t)(1000.0 / seq.fps + 0.5);
    else if (seq.fps >= 1000.0)
        period = 1;

#if defined(ROBOT_HOST)
    std::vector<std::uint32_t> screen;
#else
    // The V5 blit keeps this pointer. A local vector would be freed on
    // return, while the panel can still be scanning it.
    static std::vector<std::uint32_t> screen;
#endif
    char previous[512] = "";
    int shown = 0;
    bool started = false;

    for (int n = 0; seq.frame_count <= 0 || n < seq.frame_count; ++n) {
        if (n >= kMaxSource) break;
        if ((n % step) != 0) continue;
#if !defined(ROBOT_HOST)
        // Stop before decoding the next file. presentFrame also polls, but
        // a shoulder chord held during a slow card read should not wait.
        if (userSkipped()) break;
#endif

        char path[512];
        int wrote = std::snprintf(path, sizeof(path), seq.path_format, seq.first_frame + n);
        if (wrote < 0 || wrote >= (int)sizeof(path)) break;
        if (seq.frame_count <= 0 && previous[0] != '\0' && std::strcmp(path, previous) == 0) break;
        std::snprintf(previous, sizeof(previous), "%s", path);

        std::uint32_t t0 = pros::millis();
        std::vector<std::uint8_t> rgba;
        int iw = 0;
        int ih = 0;
        if (!loadPngRgba(path, rgba, iw, ih)) {
            if (seq.frame_count <= 0) break;
            continue;
        }

        blitFit(rgba.data(), iw, ih, screen);
        std::uint32_t spent = pros::millis() - t0;
        std::uint32_t hold = period > spent ? period - spent : 0;
        started = true;
        if (!presentFrame(screen.data(), hold)) break;
        ++shown;
    }

    if (started) endPresentation();
    return shown;
}
