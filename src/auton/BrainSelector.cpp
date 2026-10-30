/* ---------------------------------------------------------------------------
 * BrainSelector.cpp  --  selector on the left of the V5 screen, route
 * preview on the right. Touch chooses the side, the routine, and the
 * alliance color. The host build does not call the screen API.
 * ------------------------------------------------------------------------- */

#include "auton/BrainSelector.h"

#if !defined(ROBOT_HOST)

#include "auton/AutonCatalog.h"
#include "field/FieldPoints.h"
#include "pros/screen.hpp"
#include "math/MathUtils.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

#define TRACE(...)                                      \
    do {                                                \
        std::printf("trace: " __VA_ARGS__);             \
        std::printf("\n");                              \
        std::fflush(stdout);                            \
    } while (0)

namespace {

constexpr int kW = 480;
constexpr int kH = 272;
constexpr int kSelW = 176;
constexpr int kPad = 6;
constexpr int kDropH = 30;
constexpr int kRowH = 28;
constexpr int kToggleH = 30;
constexpr int kSideCount = 2;
constexpr int kPreviewCap = 160;
constexpr double kFieldIn = 144.0;

struct Rect {
    int left = 0;
    int top = 0;
    int right = 0;
    int bottom = 0;
};

bool hit(const Rect& r, int x, int y) {
    return x >= r.left && y >= r.top && x < r.right && y < r.bottom;
}

struct Layout {
    Rect drop;
    Rect rows[12];
    int rowIndex[12]{};
    int rowN = 0;
    Rect menu[kSideCount];
    Rect red;
    Rect blue;
};

bool g_open = false;
bool g_touch = false;
bool g_shown = false;
unsigned g_key = 0xffffffffu;
Pose2d g_path[kPreviewCap]{};
int g_path_n = 0;

std::uint32_t allianceColor(auton::Alliance color) {
    return color == auton::Alliance::Blue ? 0x006092D6 : 0x00D66860;
}

std::uint32_t sideColor(auton::AutonSide side, auton::Alliance color) {
    if (side == auton::AutonSide::Skills) return 0x0076B896;
    return allianceColor(color);
}

Layout layoutFor(const auton::AutonSelection& sel) {
    Layout lay;
    const int count = auton::autonCount(sel.side);
    lay.drop = Rect{kPad, kPad, kSelW - kPad, kPad + kDropH};
    const int toggleTop = kH - kPad - kToggleH;
    const int mid = kPad + (kSelW - 2 * kPad) / 2;
    lay.red = Rect{kPad, toggleTop, mid, toggleTop + kToggleH};
    lay.blue = Rect{mid, toggleTop, kSelW - kPad, toggleTop + kToggleH};
    const int listTop = lay.drop.bottom + 6;
    const int listBottom = toggleTop - 4;
    const int fit = (listBottom - listTop) / kRowH;
    const int shown = fit > 0 ? fit : 1;
    int start = 0;
    if (count > shown && sel.index >= start + shown) start = sel.index - shown + 1;
    if (start < 0) start = 0;
    lay.rowN = 0;
    for (int i = 0; i < shown && start + i < count && lay.rowN < 12; ++i) {
        const int top = listTop + i * kRowH;
        lay.rows[lay.rowN] = Rect{kPad, top, kSelW - kPad, top + kRowH};
        lay.rowIndex[lay.rowN] = start + i;
        ++lay.rowN;
    }
    for (int i = 0; i < kSideCount; ++i) {
        const int top = lay.drop.bottom + 4 + i * kRowH;
        lay.menu[i] = Rect{lay.drop.left + 2, top, lay.drop.right - 2, top + kRowH};
    }
    return lay;
}

void fill(std::uint32_t color, int x0, int y0, int x1, int y1) {
    pros::screen::set_pen(color);
    pros::screen::fill_rect((std::int16_t)x0, (std::int16_t)y0,
                            (std::int16_t)x1, (std::int16_t)y1);
}

void textAt(int x, int y, std::uint32_t color, const char* s) {
    pros::screen::set_pen(color);
    pros::screen::print(pros::E_TEXT_SMALL, (std::int16_t)x, (std::int16_t)y, "%s",
                        s != nullptr ? s : "");
}

void ensurePath(const auton::AutonSelection& sel) {
    const unsigned key = (static_cast<unsigned>(sel.side) << 16)
        | (static_cast<unsigned>(sel.color) << 8)
        | (static_cast<unsigned>(sel.index) & 0xffu);
    if (key == g_key) {
        TRACE("preview cached");
        return;
    }
    TRACE("preview build");
    g_path_n = auton::previewSelected(g_path, kPreviewCap);
    TRACE("preview build done n=%d", g_path_n);
    g_key = key;
}

void drawField(std::uint32_t color) {
    const int left = kSelW + 4;
    const int top = 4;
    const int right = kW - 4;
    const int bottom = kH - 4;
    const double margin = 8.0;
    const double scale = std::min((right - left - 2.0 * margin) / kFieldIn,
                                  (bottom - top - 2.0 * margin) / kFieldIn);
    const double cx = (left + right) / 2.0;
    const double cy = (top + bottom) / 2.0;
    auto sx = [&](double x) { return (int)std::lround(cx + x * scale); };
    auto sy = [&](double y) { return (int)std::lround(cy - (y - kFieldIn * 0.5) * scale); };

    fill(0x00E8E8E2, sx(-kFieldIn * 0.5), sy(kFieldIn), sx(kFieldIn * 0.5), sy(0.0));
    pros::screen::set_pen(0x00BABAB4);
    for (int i = 1; i < 6; ++i) {
        const double g = -kFieldIn * 0.5 + 24.0 * i;
        pros::screen::draw_line((std::int16_t)sx(g), (std::int16_t)sy(kFieldIn),
                                (std::int16_t)sx(g), (std::int16_t)sy(0.0));
        const double gy = 24.0 * i;
        pros::screen::draw_line((std::int16_t)sx(-kFieldIn * 0.5), (std::int16_t)sy(gy),
                                (std::int16_t)sx(kFieldIn * 0.5), (std::int16_t)sy(gy));
    }
    const int nspot = static_cast<int>(Field::Point::Count);
    for (int i = 0; i < nspot; ++i) {
        const Field::Spot& spot = Field::kSpots[i];
        if (Field::isLoader(spot)) {
            pros::screen::set_pen(0x00C87828);
            pros::screen::fill_circle((std::int16_t)sx(spot.x), (std::int16_t)sy(spot.y), 2);
            continue;
        }
        if (!Field::isAlignGoal(spot)) continue;
        pros::screen::set_pen(0x005A8C96);
        pros::screen::fill_circle((std::int16_t)sx(spot.x), (std::int16_t)sy(spot.y), 2);
    }
    if (g_path_n >= 2) {
        pros::screen::set_pen(color);
        const int step = g_path_n > 80 ? g_path_n / 60 : 1;
        int prev = 0;
        for (int i = step; i < g_path_n; i += step) {
            pros::screen::draw_line((std::int16_t)sx(g_path[prev].x), (std::int16_t)sy(g_path[prev].y),
                                    (std::int16_t)sx(g_path[i].x), (std::int16_t)sy(g_path[i].y));
            prev = i;
        }
        if (prev != g_path_n - 1) {
            pros::screen::draw_line((std::int16_t)sx(g_path[prev].x), (std::int16_t)sy(g_path[prev].y),
                                    (std::int16_t)sx(g_path[g_path_n - 1].x),
                                    (std::int16_t)sy(g_path[g_path_n - 1].y));
        }
        const int x0 = sx(g_path[0].x);
        const int y0 = sy(g_path[0].y);
        pros::screen::draw_circle((std::int16_t)x0, (std::int16_t)y0, 5);
        const double nose = 10.0;
        pros::screen::draw_line((std::int16_t)x0, (std::int16_t)y0,
                                (std::int16_t)sx(g_path[0].x + nose * mathutil::heading_cos(g_path[0].theta)),
                                (std::int16_t)sy(g_path[0].y + nose * mathutil::heading_sin(g_path[0].theta)));
        pros::screen::fill_circle((std::int16_t)sx(g_path[g_path_n - 1].x),
                                  (std::int16_t)sy(g_path[g_path_n - 1].y), 3);
    }
}

void draw(const auton::AutonSelection& sel, const Layout& lay) {
    TRACE("draw fill");
    fill(0x0016181C, 0, 0, kW - 1, kH - 1);
    TRACE("draw fill done");
    const std::uint32_t color = sideColor(sel.side, sel.color);
    fill(0x00282C32, lay.drop.left, lay.drop.top, lay.drop.right, lay.drop.bottom);
    TRACE("draw text");
    textAt(lay.drop.left + 8, lay.drop.top + 8, 0x00ECECEC, auton::autonSideName(sel.side));
    TRACE("draw text done");
    for (int i = 0; i < lay.rowN; ++i) {
        const Rect& row = lay.rows[i];
        const bool on = lay.rowIndex[i] == sel.index;
        if (on) fill(0x00322C28, row.left, row.top + 1, row.right, row.bottom - 1);
        textAt(row.left + 8, row.top + 7, on ? color : 0x00D6DAD0,
               auton::autonName(sel.side, lay.rowIndex[i]));
    }
    const bool redOn = sel.color == auton::Alliance::Red;
    fill(0x00282C32, lay.red.left, lay.red.top, lay.blue.right, lay.blue.bottom);
    if (redOn) {
        fill(allianceColor(auton::Alliance::Red), lay.red.left + 2, lay.red.top + 2,
             lay.red.right - 1, lay.red.bottom - 2);
    } else {
        fill(allianceColor(auton::Alliance::Blue), lay.blue.left + 1, lay.blue.top + 2,
             lay.blue.right - 2, lay.blue.bottom - 2);
    }
    textAt(lay.red.left + 8, lay.red.top + 8,
           redOn ? 0x00ECECEC : allianceColor(auton::Alliance::Red),
           auton::allianceName(auton::Alliance::Red));
    textAt(lay.blue.left + 8, lay.blue.top + 8,
           redOn ? allianceColor(auton::Alliance::Blue) : 0x00ECECEC,
           auton::allianceName(auton::Alliance::Blue));
    if (g_open) {
        fill(0x0024282E, lay.drop.left, lay.menu[0].top - 2, lay.drop.right,
             lay.menu[kSideCount - 1].bottom + 2);
        for (int i = 0; i < kSideCount; ++i) {
            const auto itemSide = static_cast<auton::AutonSide>(i);
            const bool on = itemSide == sel.side;
            if (on) fill(0x00303836, lay.menu[i].left, lay.menu[i].top + 1,
                         lay.menu[i].right, lay.menu[i].bottom - 1);
            textAt(lay.menu[i].left + 6, lay.menu[i].top + 7,
                   on ? sideColor(itemSide, sel.color) : 0x00ECECEC,
                   auton::autonSideName(itemSide));
        }
    }
    TRACE("draw field");
    drawField(color);
    TRACE("draw field done");
    pros::screen::set_pen(0x00363A40);
    pros::screen::draw_line((std::int16_t)kSelW, 0, (std::int16_t)kSelW, (std::int16_t)(kH - 1));
}

void onPress(int x, int y, const auton::AutonSelection&, const Layout& lay) {
    if (g_open) {
        for (int i = 0; i < kSideCount; ++i) {
            if (hit(lay.menu[i], x, y)) auton::selectSide(static_cast<auton::AutonSide>(i));
        }
        g_open = false;
        return;
    }
    if (hit(lay.drop, x, y)) {
        g_open = true;
        return;
    }
    if (hit(lay.red, x, y)) {
        auton::selectAlliance(auton::Alliance::Red);
        return;
    }
    if (hit(lay.blue, x, y)) {
        auton::selectAlliance(auton::Alliance::Blue);
        return;
    }
    for (int i = 0; i < lay.rowN; ++i) {
        if (hit(lay.rows[i], x, y)) {
            auton::selectIndex(lay.rowIndex[i]);
            return;
        }
    }
}

} // namespace

namespace auton {

void serviceBrainSelector(bool paint) {
    if (!paint) {
        g_shown = false;
        g_touch = false;
        return;
    }
    // Two passes cover the handoff. Later passes stay quiet so the terminal
    // does not fill and block this task. A repaint still traces.
    static int passes = 0;
    const bool talk = passes < 2;
    if (talk) TRACE("selector selection");
    const AutonSelection sel = currentSelection();
    if (talk) TRACE("selector layout");
    const Layout lay = layoutFor(sel);
    if (talk) TRACE("selector touch");
    const pros::screen_touch_status_s_t touch = pros::screen::touch_status();
    if (talk) TRACE("selector touch done");
    const bool down = touch.touch_status == pros::E_TOUCH_PRESSED
        || touch.touch_status == pros::E_TOUCH_HELD;
    bool changed = false;
    if (down && !g_touch) {
        onPress(touch.x, touch.y, sel, lay);
        changed = true;
    }
    g_touch = down;
    if (!g_shown || changed) {
        // Paint the menu before the route fit. The fit is the slow part, and
        // the last splash frame would stay up until it returned.
        TRACE("selector draw");
        AutonSelection now = currentSelection();
        draw(now, layoutFor(now));
        TRACE("selector draw done");
        g_shown = true;
        TRACE("selector preview");
        ensurePath(now);
        TRACE("selector preview done");
        now = currentSelection();
        TRACE("selector redraw");
        draw(now, layoutFor(now));
        TRACE("selector redraw done");
    }
    if (talk) ++passes;
}

} // namespace auton

#else

namespace auton {
void serviceBrainSelector(bool) {}
}

#endif
