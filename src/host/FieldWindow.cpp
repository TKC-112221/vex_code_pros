/* ---------------------------------------------------------------------------
 * FieldWindow.cpp  --  Win32 keyboard and field view for the host build.
 * MockDrive calls these; the V5 link does not include this file.
 * ------------------------------------------------------------------------- */
#include "host/FieldWindow.h"
#include "auton/AutonCatalog.h"
#include "core/Specs.h"
#include "field/FieldPoints.h"
#include "hardware/MockDriveParams.h"
#include "logic/GoalAligner.h"
#include "math/MathUtils.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cwchar>
#include <mutex>
#include <vector>
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

namespace {

constexpr int kPaneW = 840;
constexpr int kDropH = 36;
constexpr int kRowH = 34;
constexpr int kWindowH = 980;
constexpr int kMaxRows = 16;
constexpr int kSideCount = 2;
constexpr int kPreviewCap = 512;
constexpr double kFieldIn = 12.0 * 12.0;
constexpr double kBodyLengthIn = Robot::Chassis::BODY_LENGTH_M * kInchesPerMeter;
constexpr double kPi = 3.14159265358979323846;

HWND g_hwnd = nullptr;
constexpr int kMaxParticleDots = 50;
constexpr int kMaxCurvePts = 64;
constexpr ULONGLONG kPathLifeMs = 5000;
Pose2d g_truth{};
Pose2d g_estimate{};
Pose2d g_particles[kMaxParticleDots]{};
int g_particle_count = 0;

struct StampedPose {
    Pose2d pose;
    ULONGLONG t_ms;
};
std::vector<StampedPose> g_truth_trail;
std::vector<StampedPose> g_est_trail;

Pose2d g_curve[kMaxCurvePts]{};
int g_curve_count = 0;
bool g_curve_lingering = false;
ULONGLONG g_curve_expire_ms = 0;

std::vector<std::uint8_t> g_brain_dib;
int g_brain_w = 0;
int g_brain_h = 0;
RECT g_brain_lcd{0, 0, 0, 0};
bool g_skip_brain = false;

RECT g_drop_rect{};
RECT g_list_rect{};
RECT g_row_rect[kMaxRows]{};
int g_row_index[kMaxRows]{};
int g_row_n = 0;
RECT g_menu_rect[kSideCount]{};
RECT g_color_rect[2]{};
bool g_drop_open = false;
int g_list_scroll = 0;
int g_hover_row = -1;
int g_hover_menu = -1;
int g_hover_color = -1;
bool g_hover_drop = false;

Pose2d g_preview[kPreviewCap]{};
int g_preview_n = 0;
COLORREF g_preview_color = RGB(118, 184, 150);
unsigned g_preview_key = 0xffffffffu;

int g_intake_pct = 0;
int g_cascade_pct = 0;
bool g_pneu_g = false;
bool g_pneu_b = false;
double g_cascade_frac = 0;
double g_cascade_deg = 0;
double g_cascade_rpm = 0;
double g_cascade_amps = 0;
char g_score_phase[16] = "Idle";

HFONT makeUiFont(int px, int weight);
void clearAutonHits();
void drawAutonPane(HDC mem, int x, int y, int w, int h);
void drawRoutePreview(HDC mem, int x, int y, int w, int h);

bool keyDown(int vk) {
    return (GetAsyncKeyState(vk) & 0x8000) != 0;
}

double stickAxis(int negative, int positive) {
    return (keyDown(positive) ? 1.0 : 0.0) - (keyDown(negative) ? 1.0 : 0.0);
}

POINT fieldToScreen(double x, double y, double cx, double cy, double scale) {
    POINT p;
    // Screen center is the geometric center of the field. The origin is the
    // center of the bottom edge, one half-field below that.
    p.x = (LONG)std::lround(cx + x * scale);
    p.y = (LONG)std::lround(cy - (y - kFieldIn * 0.5) * scale);
    return p;
}

void pruneTrail(std::vector<StampedPose>& trail, ULONGLONG now) {
    auto cut = trail.begin();
    while (cut != trail.end() && now - cut->t_ms > kPathLifeMs) ++cut;
    if (cut != trail.begin()) trail.erase(trail.begin(), cut);
}

void rememberPose(std::vector<StampedPose>& trail, const Pose2d& pose, ULONGLONG now) {
    pruneTrail(trail, now);
    if (!trail.empty()) {
        double dx = pose.x - trail.back().pose.x;
        double dy = pose.y - trail.back().pose.y;
        double d2 = dx * dx + dy * dy;
        const double jump = 0.5 * kInchesPerMeter;
        const double hold = 0.01 * kInchesPerMeter;
        if (d2 > jump * jump) trail.clear();
        else if (d2 < hold * hold) return;
    }
    trail.push_back(StampedPose{pose, now});
}

void drawTrail(HDC mem, const std::vector<StampedPose>& trail, HPEN pen,
               double cx, double cy, double scale) {
    if (trail.size() < 2) return;
    std::vector<POINT> pts;
    pts.reserve(trail.size());
    for (const StampedPose& p : trail)
        pts.push_back(fieldToScreen(p.pose.x, p.pose.y, cx, cy, scale));
    SelectObject(mem, pen);
    Polyline(mem, pts.data(), (int)pts.size());
}

void drawLabel(HDC mem, int x, int y, const char* text);

void drawRing(HDC mem, double x, double y, double radius_in,
              double cx, double cy, double scale, HPEN pen) {
    if (radius_in <= 0.0) return;
    POINT c = fieldToScreen(x, y, cx, cy, scale);
    int r = static_cast<int>(std::lround(radius_in * scale));
    if (r < 1) r = 1;
    HGDIOBJ oldPen = SelectObject(mem, pen);
    HGDIOBJ oldBrush = SelectObject(mem, GetStockObject(HOLLOW_BRUSH));
    Ellipse(mem, c.x - r, c.y - r, c.x + r, c.y + r);
    SelectObject(mem, oldBrush);
    SelectObject(mem, oldPen);
}

void drawGoals(HDC mem, double cx, double cy, double scale) {
    const int active = goalAlignIndex();
    HPEN hitPen = CreatePen(PS_DOT, 1, RGB(70, 130, 150));
    HPEN standPen = CreatePen(PS_SOLID, 1, RGB(30, 110, 130));
    HPEN hotPen = CreatePen(PS_SOLID, 2, RGB(0, 170, 160));
    HPEN dotPen = CreatePen(PS_SOLID, 1, RGB(20, 80, 100));
    HBRUSH dotBrush = CreateSolidBrush(RGB(20, 90, 110));
    HBRUSH hotBrush = CreateSolidBrush(RGB(0, 150, 140));
    HPEN loaderPen = CreatePen(PS_SOLID, 1, RGB(140, 80, 20));
    HBRUSH loaderBrush = CreateSolidBrush(RGB(170, 100, 30));
    const int n = static_cast<int>(Field::Point::Count);
    for (int i = 0; i < n; ++i) {
        const Field::Spot& spot = Field::kSpots[i];
        if (Field::isLoader(spot)) {
            POINT c = fieldToScreen(spot.x, spot.y, cx, cy, scale);
            HGDIOBJ oldPen = SelectObject(mem, loaderPen);
            HGDIOBJ oldBrush = SelectObject(mem, loaderBrush);
            Ellipse(mem, c.x - 3, c.y - 3, c.x + 4, c.y + 4);
            SelectObject(mem, oldBrush);
            SelectObject(mem, oldPen);
            drawLabel(mem, c.x + 6, c.y - 16, spot.name);
            continue;
        }
        if (!Field::isAlignGoal(spot)) continue;
        const bool hot = (i == active);
        drawRing(mem, spot.x, spot.y, spot.hitbox_in, cx, cy, scale, hot ? hotPen : hitPen);
        drawRing(mem, spot.x, spot.y, spot.standoff_in, cx, cy, scale, hot ? hotPen : standPen);
        POINT c = fieldToScreen(spot.x, spot.y, cx, cy, scale);
        HGDIOBJ oldPen = SelectObject(mem, dotPen);
        HGDIOBJ oldBrush = SelectObject(mem, hot ? hotBrush : dotBrush);
        Ellipse(mem, c.x - 3, c.y - 3, c.x + 4, c.y + 4);
        SelectObject(mem, oldBrush);
        SelectObject(mem, oldPen);
        drawLabel(mem, c.x + 6, c.y - 16, spot.name);
    }
    DeleteObject(loaderBrush);
    DeleteObject(loaderPen);
    DeleteObject(hotBrush);
    DeleteObject(dotBrush);
    DeleteObject(dotPen);
    DeleteObject(hotPen);
    DeleteObject(standPen);
    DeleteObject(hitPen);
}

void drawCurve(HDC mem, HPEN pen, double cx, double cy, double scale) {
    if (g_curve_count < 2) return;
    std::vector<POINT> pts;
    pts.reserve((size_t)g_curve_count);
    for (int i = 0; i < g_curve_count; ++i)
        pts.push_back(fieldToScreen(g_curve[i].x, g_curve[i].y, cx, cy, scale));
    SelectObject(mem, pen);
    Polyline(mem, pts.data(), g_curve_count);
}

void drawChassis(HDC mem, const Pose2d& pose, double cx, double cy, double scale,
                 HBRUSH fill, HPEN edge, HPEN nosePen) {
    // Pose is the middle of the front edge. The rectangle runs back from it.
    const double length = kBodyLengthIn;
    const double hw = Robot::Chassis::TRACK_WIDTH_M * kInchesPerMeter / 2.0;
    const double cth = mathutil::heading_cos(pose.theta);
    const double sth = mathutil::heading_sin(pose.theta);
    const double rx[4] = {0.0, 0.0, -length, -length};
    const double ry[4] = {hw, -hw, -hw, hw};
    POINT body[4];
    for (int i = 0; i < 4; ++i) {
        double fx = pose.x + rx[i] * cth - ry[i] * sth;
        double fy = pose.y + rx[i] * sth + ry[i] * cth;
        body[i] = fieldToScreen(fx, fy, cx, cy, scale);
    }
    SelectObject(mem, fill);
    SelectObject(mem, edge);
    Polygon(mem, body, 4);
    POINT nose0 = fieldToScreen(pose.x, pose.y, cx, cy, scale);
    const double nose_in = 0.06 * kInchesPerMeter;
    POINT nose1 = fieldToScreen(pose.x + nose_in * cth,
                                pose.y + nose_in * sth, cx, cy, scale);
    SelectObject(mem, nosePen);
    MoveToEx(mem, nose0.x, nose0.y, nullptr);
    LineTo(mem, nose1.x, nose1.y);
}

void drawLabel(HDC mem, int x, int y, const char* text) {
    HFONT font = (HFONT)GetStockObject(DEFAULT_GUI_FONT);
    HGDIOBJ old = SelectObject(mem, font);
    SetBkMode(mem, TRANSPARENT);
    SetTextColor(mem, RGB(186, 190, 198));
    TextOutA(mem, x, y, text, (int)std::strlen(text));
    SelectObject(mem, old);
}

void fillRoundRect(HDC mem, int left, int top, int right, int bottom,
                   int radius, COLORREF fill, COLORREF edge) {
    HBRUSH brush = CreateSolidBrush(fill);
    HPEN pen = CreatePen(PS_SOLID, 1, edge);
    HGDIOBJ oldBrush = SelectObject(mem, brush);
    HGDIOBJ oldPen = SelectObject(mem, pen);
    RoundRect(mem, left, top, right, bottom, radius, radius);
    SelectObject(mem, oldPen);
    SelectObject(mem, oldBrush);
    DeleteObject(pen);
    DeleteObject(brush);
}

void blitBrainLcd(HDC mem, int left, int top, int right, int bottom) {
    BITMAPINFO bmi{};
    bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth = g_brain_w;
    bmi.bmiHeader.biHeight = g_brain_h;
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 32;
    bmi.bmiHeader.biCompression = BI_RGB;
    SaveDC(mem);
    HRGN clip = CreateRoundRectRgn(left, top, right + 1, bottom + 1, 16, 16);
    SelectClipRgn(mem, clip);
    StretchDIBits(mem, left, top, right - left, bottom - top,
                  0, 0, g_brain_w, g_brain_h,
                  g_brain_dib.data(), &bmi, DIB_RGB_COLORS, SRCCOPY);
    RestoreDC(mem, -1);
    DeleteObject(clip);
}

void drawPill(HDC mem, int right, int y, const char* text, COLORREF fill, COLORREF ink) {
    HFONT font = makeUiFont(14, FW_SEMIBOLD);
    HGDIOBJ old = SelectObject(mem, font);
    SIZE sz{};
    const int n = (int)std::strlen(text);
    GetTextExtentPoint32A(mem, text, n, &sz);
    const int padX = 10;
    const int boxW = sz.cx + padX * 2;
    const int boxH = 22;
    const int left = right - boxW;
    fillRoundRect(mem, left, y, right, y + boxH, 10, fill, fill);
    SetBkMode(mem, TRANSPARENT);
    SetTextColor(mem, ink);
    TextOutA(mem, left + padX, y + (boxH - sz.cy) / 2, text, n);
    SelectObject(mem, old);
    DeleteObject(font);
}

int drawMechRow(HDC mem, int x, int right, int y, const char* label, const char* value,
                COLORREF fill, COLORREF ink) {
    HFONT font = makeUiFont(15, FW_NORMAL);
    HGDIOBJ old = SelectObject(mem, font);
    SetBkMode(mem, TRANSPARENT);
    SetTextColor(mem, RGB(214, 218, 224));
    SIZE sz{};
    GetTextExtentPoint32A(mem, label, (int)std::strlen(label), &sz);
    TextOutA(mem, x, y + (22 - sz.cy) / 2, label, (int)std::strlen(label));
    SelectObject(mem, old);
    DeleteObject(font);
    drawPill(mem, right, y, value, fill, ink);
    return y + 28;
}

void drawCascadeBar(HDC mem, int x, int y, int w, double frac, int pct) {
    if (frac < 0.0) frac = 0.0;
    if (frac > 1.0) frac = 1.0;
    const int h = 10;
    fillRoundRect(mem, x, y, x + w, y + h, 8, RGB(16, 18, 22), RGB(48, 52, 58));
    const int fillW = (int)std::lround((w - 4) * frac);
    if (fillW <= 0) return;
    const COLORREF c = pct > 0 ? RGB(86, 176, 114)
                      : pct < 0 ? RGB(214, 148, 64)
                                : RGB(118, 158, 198);
    fillRoundRect(mem, x + 2, y + 2, x + 2 + fillW, y + h - 2, 6, c, c);
}

void drawSubsystemPanel(HDC mem, int x, int y, int w, int bottom) {
    if (w < 80 || y + 80 > bottom) return;
    drawLabel(mem, x, y, "Subsystems");
    y += 24;

    const int cardBottom = bottom - 12;
    if (cardBottom - y < 80) return;

    fillRoundRect(mem, x, y, x + w, cardBottom, 14, RGB(36, 38, 44), RGB(54, 58, 64));
    const int inset = 16;
    const int left = x + inset;
    const int right = x + w - inset;
    const int innerTop = y + 18;
    const int innerBot = cardBottom - 18;
    if (right - left < 40 || innerBot <= innerTop) return;

    const COLORREF idleFill = RGB(58, 62, 68);
    const COLORREF idleInk = RGB(196, 200, 206);
    const COLORREF goFill = RGB(36, 104, 68);
    const COLORREF goInk = RGB(214, 240, 220);
    const COLORREF backFill = RGB(128, 82, 32);
    const COLORREF backInk = RGB(255, 224, 184);
    const COLORREF scoreFill = RGB(36, 78, 128);
    const COLORREF scoreInk = RGB(214, 228, 248);

    constexpr int kRow = 22;
    constexpr int kMeter = 36;
    constexpr int kGaps = 5;
    const int fixed = kRow * 5 + kMeter;
    int gap = 8;
    if (innerBot - innerTop > fixed)
        gap = (innerBot - innerTop - fixed) / kGaps;
    if (gap < 8) gap = 8;
    if (gap > 40) gap = 40;

    SaveDC(mem);
    IntersectClipRect(mem, x + 4, y + 4, x + w - 4, cardBottom - 4);

    int row = innerTop;
    const char* intake = "Idle";
    COLORREF intakeFill = idleFill;
    COLORREF intakeInk = idleInk;
    if (g_intake_pct > 0) { intake = "In"; intakeFill = goFill; intakeInk = goInk; }
    else if (g_intake_pct < 0) { intake = "Out"; intakeFill = backFill; intakeInk = backInk; }
    drawMechRow(mem, left, right, row, "Intake", intake, intakeFill, intakeInk);
    row += kRow + gap;

    const char* cascade = "Idle";
    COLORREF cascadeFill = idleFill;
    COLORREF cascadeInk = idleInk;
    if (g_cascade_pct > 0) { cascade = "Up"; cascadeFill = goFill; cascadeInk = goInk; }
    else if (g_cascade_pct < 0) { cascade = "Down"; cascadeFill = backFill; cascadeInk = backInk; }
    drawMechRow(mem, left, right, row, "Cascade", cascade, cascadeFill, cascadeInk);
    row += kRow + gap;

    drawCascadeBar(mem, left, row, right - left, g_cascade_frac, g_cascade_pct);
    char metrics[64];
    std::snprintf(metrics, sizeof(metrics), "%3.0f deg   %4.2f A   %+4.0f rpm",
                  g_cascade_deg, g_cascade_amps, g_cascade_rpm);
    HFONT font = makeUiFont(13, FW_NORMAL);
    HGDIOBJ old = SelectObject(mem, font);
    SetBkMode(mem, TRANSPARENT);
    SetTextColor(mem, RGB(168, 174, 182));
    TextOutA(mem, left, row + 14, metrics, (int)std::strlen(metrics));
    SelectObject(mem, old);
    DeleteObject(font);
    row += kMeter + gap;

    const bool clawOpen = g_pneu_b;
    drawMechRow(mem, left, right, row, "Claw", clawOpen ? "Open" : "Closed",
                clawOpen ? backFill : idleFill, clawOpen ? backInk : idleInk);
    row += kRow + gap;
    drawMechRow(mem, left, right, row, "Pneu G", g_pneu_g ? "On" : "Off",
                g_pneu_g ? goFill : idleFill, g_pneu_g ? goInk : idleInk);
    row += kRow + gap;

    const bool scoring = std::strcmp(g_score_phase, "Idle") != 0;
    drawMechRow(mem, left, right, row, "Score", g_score_phase,
                scoring ? scoreFill : idleFill, scoring ? scoreInk : idleInk);
    RestoreDC(mem, -1);
}

void drawSidePanel(HDC mem, int x, int y, int w, int h) {
    if (w < 40 || h < 40) return;

    RECT column{x, y, x + w, y + h};
    HBRUSH columnBg = CreateSolidBrush(RGB(22, 24, 28));
    FillRect(mem, &column, columnBg);
    DeleteObject(columnBg);

    HPEN sep = CreatePen(PS_SOLID, 1, RGB(54, 58, 64));
    HGDIOBJ oldSep = SelectObject(mem, sep);
    MoveToEx(mem, x, y, nullptr);
    LineTo(mem, x, y + h);
    SelectObject(mem, oldSep);
    DeleteObject(sep);

    const int pad = 16;
    const int innerW = w - pad * 2;
    if (innerW < 80) return;
    int cursor = y + pad;
    const int originX = x + pad;

    drawLabel(mem, originX, cursor, "Brain");
    cursor += 22;

    // The right half is the brain screen over the subsystem states.
    const int mid = y + h / 2;
    int brainSlotB = mid - 12;
    if (brainSlotB < cursor + 96) brainSlotB = cursor + 96;

    int brainW = innerW;
    int brainH = brainW * 272 / 480;
    const int brainSlotH = brainSlotB - cursor;
    if (brainH > brainSlotH && brainSlotH > 0) {
        brainH = brainSlotH;
        brainW = brainH * 480 / 272;
        if (brainW > innerW) brainW = innerW;
    }
    const int brainX = originX + (innerW - brainW) / 2;
    if (brainH < brainSlotH) cursor += (brainSlotH - brainH) / 2;

    fillRoundRect(mem, brainX, cursor, brainX + brainW, cursor + brainH, 16,
                  RGB(52, 56, 64), RGB(30, 32, 38));

    int inset = brainH / 28;
    if (inset < 6) inset = 6;
    const int lcdL = brainX + inset;
    const int lcdT = cursor + inset;
    const int lcdR = brainX + brainW - inset;
    const int lcdB = cursor + brainH - inset;
    fillRoundRect(mem, lcdL, lcdT, lcdR, lcdB, 8, RGB(22, 24, 28), RGB(22, 24, 28));
    g_brain_lcd.left = lcdL;
    g_brain_lcd.top = lcdT;
    g_brain_lcd.right = lcdR;
    g_brain_lcd.bottom = lcdB;

    const bool brain_frame = g_brain_w > 0 && g_brain_h > 0 &&
        g_brain_dib.size() == (std::size_t)g_brain_w * (std::size_t)g_brain_h * 4u;
    if (brain_frame) {
        blitBrainLcd(mem, lcdL, lcdT, lcdR, lcdB);
        clearAutonHits();
    } else {
        // The V5 brain is the selector on the left and the route preview
        // on the right. 176 is the selector width on a 480-wide screen.
        SaveDC(mem);
        HRGN clip = CreateRoundRectRgn(lcdL, lcdT, lcdR + 1, lcdB + 1, 16, 16);
        SelectClipRgn(mem, clip);
        const int lcdW = lcdR - lcdL;
        const int lcdH = lcdB - lcdT;
        int selW = lcdW * 176 / 480;
        if (selW < 110) selW = 110;
        if (selW > lcdW - 80) selW = lcdW / 2;
        drawAutonPane(mem, lcdL, lcdT, selW, lcdH);
        drawRoutePreview(mem, lcdL + selW, lcdT, lcdW - selW, lcdH);
        RestoreDC(mem, -1);
        DeleteObject(clip);
    }

    HPEN rule = CreatePen(PS_SOLID, 1, RGB(54, 58, 64));
    HGDIOBJ oldRule = SelectObject(mem, rule);
    MoveToEx(mem, x + 12, mid, nullptr);
    LineTo(mem, x + w - 12, mid);
    SelectObject(mem, oldRule);
    DeleteObject(rule);

    drawSubsystemPanel(mem, originX, mid + 12, innerW, y + h);
}

COLORREF allianceColor(auton::Alliance color) {
    return color == auton::Alliance::Blue ? RGB(96, 146, 214) : RGB(214, 104, 96);
}

COLORREF allianceFill(auton::Alliance color) {
    return color == auton::Alliance::Blue ? RGB(28, 44, 78) : RGB(72, 34, 32);
}

COLORREF sideColor(auton::AutonSide side, auton::Alliance color) {
    if (side == auton::AutonSide::Skills) return RGB(118, 184, 150);
    return allianceColor(color);
}

COLORREF sideFill(auton::AutonSide side, auton::Alliance color) {
    if (side == auton::AutonSide::Skills) return RGB(30, 52, 44);
    return allianceFill(color);
}

HFONT makeUiFont(int px, int weight) {
    return CreateFontW(-px, 0, 0, 0, weight, FALSE, FALSE, FALSE,
                       ANSI_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                       CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
}

void drawChevron(HDC mem, int x, int y, bool open) {
    HPEN pen = CreatePen(PS_SOLID, 2, RGB(196, 200, 206));
    HGDIOBJ old = SelectObject(mem, pen);
    if (open) {
        MoveToEx(mem, x, y + 5, nullptr);
        LineTo(mem, x + 5, y);
        LineTo(mem, x + 10, y + 5);
    } else {
        MoveToEx(mem, x, y, nullptr);
        LineTo(mem, x + 5, y + 5);
        LineTo(mem, x + 10, y);
    }
    SelectObject(mem, old);
    DeleteObject(pen);
}

void drawUiText(HDC mem, RECT rect, const char* text, COLORREF color, UINT flags) {
    SetTextColor(mem, color);
    DrawTextA(mem, text != nullptr ? text : "", -1, &rect, flags);
}

void clearAutonHits() {
    g_drop_rect = RECT{};
    g_list_rect = RECT{};
    g_row_n = 0;
    for (int i = 0; i < kSideCount; ++i) g_menu_rect[i] = RECT{};
    for (int i = 0; i < 2; ++i) g_color_rect[i] = RECT{};
}

void ensurePreview() {
    const auton::AutonSelection sel = auton::currentSelection();
    const unsigned key = (static_cast<unsigned>(sel.side) << 16)
        | (static_cast<unsigned>(sel.color) << 8)
        | (static_cast<unsigned>(sel.index) & 0xffu);
    if (key == g_preview_key) return;
    g_preview_color = sideColor(sel.side, sel.color);
    g_preview_n = auton::previewSelected(g_preview, kPreviewCap);
    g_preview_key = key;
}

void drawPreview(HDC mem, double cx, double cy, double scale) {
    if (g_preview_n < 2) return;
    const COLORREF color = g_preview_color;
    HPEN pen = CreatePen(PS_SOLID, 2, color);
    HPEN edge = CreatePen(PS_SOLID, 1, color);
    HBRUSH dot = CreateSolidBrush(color);
    HGDIOBJ oldPen = SelectObject(mem, pen);
    HGDIOBJ oldBrush = SelectObject(mem, GetStockObject(HOLLOW_BRUSH));

    std::vector<POINT> pts;
    pts.reserve((size_t)g_preview_n);
    for (int i = 0; i < g_preview_n; ++i)
        pts.push_back(fieldToScreen(g_preview[i].x, g_preview[i].y, cx, cy, scale));
    Polyline(mem, pts.data(), g_preview_n);

    drawChassis(mem, g_preview[0], cx, cy, scale,
                (HBRUSH)GetStockObject(HOLLOW_BRUSH), edge, pen);

    POINT end = pts.back();
    SelectObject(mem, dot);
    SelectObject(mem, pen);
    Ellipse(mem, end.x - 4, end.y - 4, end.x + 5, end.y + 5);

    SelectObject(mem, oldPen);
    SelectObject(mem, oldBrush);
    DeleteObject(pen);
    DeleteObject(edge);
    DeleteObject(dot);
}

void drawAutonPane(HDC mem, int x, int y, int w, int h) {
    clearAutonHits();
    if (w < 80 || h < 80) return;

    RECT pane{x, y, x + w, y + h};
    HBRUSH paneBg = CreateSolidBrush(RGB(22, 24, 28));
    FillRect(mem, &pane, paneBg);
    DeleteObject(paneBg);

    HPEN sep = CreatePen(PS_SOLID, 1, RGB(54, 58, 64));
    HGDIOBJ oldSep = SelectObject(mem, sep);
    MoveToEx(mem, x + w - 1, y, nullptr);
    LineTo(mem, x + w - 1, y + h);
    SelectObject(mem, oldSep);
    DeleteObject(sep);

    const auton::AutonSelection sel = auton::currentSelection();
    const auton::AutonSide side = sel.side;
    const int count = auton::autonCount(side);
    const COLORREF color = sideColor(side, sel.color);

    HFONT font = makeUiFont(15, FW_NORMAL);
    const bool ownFont = font != nullptr;
    if (!ownFont) font = (HFONT)GetStockObject(DEFAULT_GUI_FONT);
    HGDIOBJ oldFont = SelectObject(mem, font);
    SetBkMode(mem, TRANSPARENT);

    const int pad = 14;
    int cursor = y + pad;
    g_drop_rect = RECT{x + pad, cursor, x + w - pad, cursor + kDropH};
    const COLORREF dropFill = g_hover_drop ? RGB(52, 56, 64) : RGB(40, 44, 50);
    fillRoundRect(mem, g_drop_rect.left, g_drop_rect.top, g_drop_rect.right, g_drop_rect.bottom,
                  8, dropFill, color);
    RECT dropText = g_drop_rect;
    dropText.left += 14;
    dropText.right -= 28;
    drawUiText(mem, dropText, auton::autonSideName(side), RGB(236, 236, 236),
               DT_LEFT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS | DT_NOPREFIX);
    drawChevron(mem, g_drop_rect.right - 24, g_drop_rect.top + (kDropH - 6) / 2, g_drop_open);
    cursor += kDropH + 8;

    const int toggleTop = y + h - pad - kDropH;
    const int gap = 6;
    const int half = (w - 2 * pad - gap) / 2;
    g_color_rect[0] = RECT{x + pad, toggleTop, x + pad + half, toggleTop + kDropH};
    g_color_rect[1] = RECT{g_color_rect[0].right + gap, toggleTop, x + w - pad, toggleTop + kDropH};
    int listBottom = toggleTop - 8;
    if (listBottom < cursor + kRowH) listBottom = cursor + kRowH;
    g_list_rect = RECT{x + pad, cursor, x + w - pad, listBottom};

    const int visible = (g_list_rect.bottom - g_list_rect.top) / kRowH;
    const int shown = visible > 0 ? visible : 1;
    int maxScroll = count - shown;
    if (maxScroll < 0) maxScroll = 0;
    if (sel.index < g_list_scroll) g_list_scroll = sel.index;
    if (count > 0 && sel.index >= g_list_scroll + shown)
        g_list_scroll = sel.index - shown + 1;
    if (g_list_scroll < 0) g_list_scroll = 0;
    if (g_list_scroll > maxScroll) g_list_scroll = maxScroll;

    SaveDC(mem);
    IntersectClipRect(mem, g_list_rect.left, g_list_rect.top,
                      g_list_rect.right, g_list_rect.bottom);
    if (count <= 0) {
        RECT empty = g_list_rect;
        empty.left += 8;
        drawUiText(mem, empty, "No routines", RGB(140, 144, 150),
                   DT_LEFT | DT_VCENTER | DT_SINGLELINE | DT_NOPREFIX);
    }
    g_row_n = 0;
    for (int i = 0; i < shown && g_list_scroll + i < count && g_row_n < kMaxRows; ++i) {
        const int index = g_list_scroll + i;
        RECT row{g_list_rect.left, g_list_rect.top + i * kRowH,
                 g_list_rect.right, g_list_rect.top + (i + 1) * kRowH};
        g_row_rect[g_row_n] = row;
        g_row_index[g_row_n] = index;
        ++g_row_n;
        const bool on = index == sel.index;
        const bool hot = index == g_hover_row && !g_drop_open;
        if (on) {
            fillRoundRect(mem, row.left, row.top + 2, row.right, row.bottom - 2,
                          8, sideFill(side, sel.color), color);
        } else if (hot) {
            fillRoundRect(mem, row.left, row.top + 2, row.right, row.bottom - 2,
                          8, RGB(40, 44, 50), RGB(40, 44, 50));
        }
        RECT text = row;
        text.left += 14;
        text.right -= 8;
        drawUiText(mem, text, auton::autonName(side, index),
                   on ? RGB(246, 246, 246) : RGB(214, 218, 224),
                   DT_LEFT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS | DT_NOPREFIX);
    }
    RestoreDC(mem, -1);

    const auton::Alliance colors[2] = {auton::Alliance::Red, auton::Alliance::Blue};
    for (int i = 0; i < 2; ++i) {
        const RECT& chip = g_color_rect[i];
        const bool on = sel.color == colors[i];
        const bool hot = i == g_hover_color && !on;
        const COLORREF fill = on ? allianceFill(colors[i])
                                  : (hot ? RGB(48, 52, 58) : RGB(40, 44, 50));
        const COLORREF edge = on ? allianceColor(colors[i])
                                  : (hot ? RGB(90, 96, 104) : RGB(40, 44, 50));
        fillRoundRect(mem, chip.left, chip.top, chip.right, chip.bottom, 8, fill, edge);
        RECT text = chip;
        text.left += 8;
        text.right -= 8;
        drawUiText(mem, text, auton::allianceName(colors[i]),
                   on ? RGB(246, 246, 246) : allianceColor(colors[i]),
                   DT_CENTER | DT_VCENTER | DT_SINGLELINE | DT_NOPREFIX);
    }

    if (g_drop_open) {
        const int menuTop = g_drop_rect.bottom + 4;
        const int menuH = kSideCount * kRowH;
        fillRoundRect(mem, g_drop_rect.left, menuTop, g_drop_rect.right, menuTop + menuH,
                      8, RGB(36, 40, 46), RGB(78, 84, 94));
        for (int i = 0; i < kSideCount; ++i) {
            RECT item{g_drop_rect.left + 4, menuTop + i * kRowH,
                      g_drop_rect.right - 4, menuTop + (i + 1) * kRowH};
            g_menu_rect[i] = item;
            const auto itemSide = static_cast<auton::AutonSide>(i);
            const bool on = itemSide == side;
            const bool hot = i == g_hover_menu;
            if (on || hot) {
                fillRoundRect(mem, item.left, item.top + 2, item.right, item.bottom - 2,
                              6, on ? sideFill(itemSide, sel.color) : RGB(50, 54, 62),
                              on ? sideColor(itemSide, sel.color) : RGB(50, 54, 62));
            }
            RECT text = item;
            text.left += 12;
            text.right -= 8;
            drawUiText(mem, text, auton::autonSideName(itemSide), RGB(236, 236, 236),
                       DT_LEFT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS | DT_NOPREFIX);
        }
    }

    SelectObject(mem, oldFont);
    if (ownFont) DeleteObject(font);
}

bool handleAutonClick(POINT p) {
    if (g_drop_open) {
        for (int i = 0; i < kSideCount; ++i) {
            if (PtInRect(&g_menu_rect[i], p)) {
                auton::selectSide(static_cast<auton::AutonSide>(i));
                break;
            }
        }
        g_drop_open = false;
        return true;
    }
    if (PtInRect(&g_drop_rect, p)) {
        g_drop_open = true;
        return true;
    }
    for (int i = 0; i < 2; ++i) {
        if (PtInRect(&g_color_rect[i], p)) {
            auton::selectAlliance(static_cast<auton::Alliance>(i));
            return true;
        }
    }
    for (int i = 0; i < g_row_n; ++i) {
        if (PtInRect(&g_row_rect[i], p)) {
            auton::selectIndex(g_row_index[i]);
            return true;
        }
    }
    return false;
}

void drawRoutePreview(HDC mem, int x, int y, int w, int h) {
    if (w < 40 || h < 40) return;
    RECT pane{x, y, x + w, y + h};
    HBRUSH paneBg = CreateSolidBrush(RGB(22, 24, 28));
    FillRect(mem, &pane, paneBg);
    DeleteObject(paneBg);

    const double margin = 12.0;
    if (w <= (int)(2.0 * margin) || h <= (int)(2.0 * margin)) return;
    const double scale = std::min((w - 2.0 * margin) / kFieldIn, (h - 2.0 * margin) / kFieldIn);
    const double cx = x + w / 2.0;
    const double cy = y + h / 2.0;
    const double half = kFieldIn / 2.0;
    POINT field[4] = {
        fieldToScreen(-half, kFieldIn, cx, cy, scale),
        fieldToScreen( half, kFieldIn, cx, cy, scale),
        fieldToScreen( half, 0.0, cx, cy, scale),
        fieldToScreen(-half, 0.0, cx, cy, scale),
    };
    HBRUSH fieldBrush = CreateSolidBrush(RGB(232, 232, 226));
    HPEN border = CreatePen(PS_SOLID, 2, RGB(40, 40, 40));
    HGDIOBJ oldPen = SelectObject(mem, border);
    HGDIOBJ oldBrush = SelectObject(mem, fieldBrush);
    Polygon(mem, field, 4);
    HPEN grid = CreatePen(PS_SOLID, 1, RGB(186, 186, 180));
    SelectObject(mem, grid);
    for (int i = 1; i < 6; ++i) {
        double gx = -half + 24.0 * i;
        POINT a = fieldToScreen(gx, kFieldIn, cx, cy, scale);
        POINT b = fieldToScreen(gx, 0.0, cx, cy, scale);
        MoveToEx(mem, a.x, a.y, nullptr);
        LineTo(mem, b.x, b.y);
        double gy = 24.0 * i;
        POINT c = fieldToScreen(-half, gy, cx, cy, scale);
        POINT d = fieldToScreen( half, gy, cx, cy, scale);
        MoveToEx(mem, c.x, c.y, nullptr);
        LineTo(mem, d.x, d.y);
    }
    SelectObject(mem, oldPen);
    SelectObject(mem, oldBrush);
    DeleteObject(grid);
    DeleteObject(border);
    DeleteObject(fieldBrush);
    drawGoals(mem, cx, cy, scale);
    drawPreview(mem, cx, cy, scale);
}

// Knobs a controller overfits. Off the compiled plant, a short lag or a late
// voltage oscillates, a long lag or a low gain undershoots, and extra speed
// or a long skid runs past the target. Side resistance is omitted: it changes
// reported current and does not move the wheels.
// Friction, velocity, and acceleration are split into forward and yaw. The
// compiled tick is no friction and a scale of 1, which keeps the shared lag.
constexpr int kKnobDelay = 0;
constexpr int kKnobLag = 1;
constexpr int kKnobSpeed = 2;
constexpr int kKnobSkid = 3;
constexpr int kKnobLinFriction = 4;
constexpr int kKnobLinVel = 5;
constexpr int kKnobLinAccel = 6;
constexpr int kKnobAngFriction = 7;
constexpr int kKnobAngVel = 8;
constexpr int kKnobAngAccel = 9;
constexpr int kKnobCount = 10;

// Ports the domain test can unplug. Order matches the chips under the sliders.
constexpr int kSensImu = 0;
constexpr int kSensEncL = 1;
constexpr int kSensEncR = 2;
constexpr int kSensDist = 3;
constexpr int kSensCount = kSensDist + kDistanceChannels;

struct Knob {
    const char* label;
    const char* hint;
    const char* unit;
    double min;
    double max;
    double original;
    double step;
};

constexpr Knob kKnobs[kKnobCount] = {
    {"Voltage delay", "late volts oscillate a tuned loop", "ms",
     0.0, 120.0, 0.0, 10.0},
    {"Velocity lag", "short oscillates, long undershoots", "s",
     0.02, 0.80, MockPlant::VELOCITY_TAU_SEC, 0.0},
    {"Speed scale", "high runs past, low stops short", "x",
     0.40, 1.80, MockPlant::SPEED_SCALE, 0.0},
    {"Skid time", "long slides past the path", "s",
     0.04, 1.50, MockPlant::SKID_TAU_SEC, 0.0},
    {"Linear friction", "forward volts lost before it rolls", "V",
     0.0, 3.0, 0.0, 0.1},
    {"Linear velocity", "scales settled forward speed", "x",
     0.25, 1.75, 1.0, 0.0},
    {"Linear acceleration", "high oscillates, low lags the drive", "x",
     0.25, 1.75, 1.0, 0.0},
    {"Angular friction", "VR-VL volts lost before it yaws", "V",
     0.0, 6.0, 0.0, 0.1},
    {"Angular velocity", "scales settled yaw rate", "x",
     0.25, 1.75, 1.0, 0.0},
    {"Angular acceleration", "high oscillates, low lags the turn", "x",
     0.25, 1.75, 1.0, 0.0},
};

double g_knob[kKnobCount] = {
    kKnobs[kKnobDelay].original,
    kKnobs[kKnobLag].original,
    kKnobs[kKnobSpeed].original,
    kKnobs[kKnobSkid].original,
    kKnobs[kKnobLinFriction].original,
    kKnobs[kKnobLinVel].original,
    kKnobs[kKnobLinAccel].original,
    kKnobs[kKnobAngFriction].original,
    kKnobs[kKnobAngVel].original,
    kKnobs[kKnobAngAccel].original,
};
// True while that port is unplugged. Connected is the compiled robot.
bool g_sensor_off[kSensCount] = {};
std::mutex g_plant_mu;

constexpr int kHitNone = -1;
constexpr int kHitDrop = 1;
constexpr int kHitPanel = 2;
constexpr int kHitAll = 3;
constexpr int kHitTrack = 10;
constexpr int kHitReset = 40;
constexpr int kHitSensor = 60;

RECT g_plant_drop{};
RECT g_plant_panel{};
RECT g_plant_track[kKnobCount]{};
RECT g_plant_reset[kKnobCount]{};
RECT g_plant_sensor[kSensCount]{};
RECT g_plant_all{};
bool g_plant_open = false;
int g_plant_drag = -1;
int g_plant_hover = kHitNone;

double knobLo(const Knob& k) { return std::min(k.min, k.original); }
double knobHi(const Knob& k) { return std::max(k.max, k.original); }

double knobT(const Knob& k, double value) {
    const double lo = knobLo(k);
    const double hi = knobHi(k);
    if (hi <= lo) return 0.0;
    double t = (value - lo) / (hi - lo);
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    return t;
}

double snapKnob(const Knob& k, double value) {
    const double lo = knobLo(k);
    const double hi = knobHi(k);
    if (value < lo) value = lo;
    if (value > hi) value = hi;
    if (k.step > 0.0) {
        value = lo + std::round((value - lo) / k.step) * k.step;
        if (value < lo) value = lo;
        if (value > hi) value = hi;
    }
    return value;
}

bool knobShifted(int index, double value) {
    return std::fabs(value - kKnobs[index].original) > 1e-4;
}

void formatKnob(int index, double value, char* buf, int n) {
    const Knob& k = kKnobs[index];
    if (k.step >= 1.0) std::snprintf(buf, n, "%.0f %s", value, k.unit);
    else if (k.step >= 0.1) std::snprintf(buf, n, "%.1f %s", value, k.unit);
    else std::snprintf(buf, n, "%.3f %s", value, k.unit);
}

bool rectLive(const RECT& r) {
    return r.right > r.left && r.bottom > r.top;
}

void clearPlantHits() {
    g_plant_panel = RECT{};
    g_plant_all = RECT{};
    for (int i = 0; i < kKnobCount; ++i) {
        g_plant_track[i] = RECT{};
        g_plant_reset[i] = RECT{};
    }
    for (int i = 0; i < kSensCount; ++i) g_plant_sensor[i] = RECT{};
}

int plantHit(POINT p) {
    if (rectLive(g_plant_drop) && PtInRect(&g_plant_drop, p)) return kHitDrop;
    if (!g_plant_open) return kHitNone;
    for (int i = 0; i < kKnobCount; ++i) {
        if (rectLive(g_plant_reset[i]) && PtInRect(&g_plant_reset[i], p)) return kHitReset + i;
    }
    if (rectLive(g_plant_all) && PtInRect(&g_plant_all, p)) return kHitAll;
    for (int i = 0; i < kSensCount; ++i) {
        if (rectLive(g_plant_sensor[i]) && PtInRect(&g_plant_sensor[i], p)) return kHitSensor + i;
    }
    for (int i = 0; i < kKnobCount; ++i) {
        if (!rectLive(g_plant_track[i])) continue;
        RECT hit = g_plant_track[i];
        InflateRect(&hit, 0, 6);
        if (PtInRect(&hit, p)) return kHitTrack + i;
    }
    if (rectLive(g_plant_panel) && PtInRect(&g_plant_panel, p)) return kHitPanel;
    return kHitNone;
}

void setKnobFromX(int index, int x) {
    const Knob& k = kKnobs[index];
    const RECT& track = g_plant_track[index];
    const int span = track.right - track.left;
    if (span <= 1) return;
    double t = (double)(x - track.left) / (double)span;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    const double lo = knobLo(k);
    const double hi = knobHi(k);
    double value = lo + t * (hi - lo);
    if (std::fabs(value - k.original) <= 0.012 * (hi - lo)) value = k.original;
    else value = snapKnob(k, value);
    std::lock_guard<std::mutex> guard(g_plant_mu);
    g_knob[index] = value;
}

void resetKnob(int index) {
    std::lock_guard<std::mutex> guard(g_plant_mu);
    g_knob[index] = kKnobs[index].original;
}

void resetPlantKnobs() {
    std::lock_guard<std::mutex> guard(g_plant_mu);
    for (int i = 0; i < kKnobCount; ++i) g_knob[i] = kKnobs[i].original;
    for (int i = 0; i < kSensCount; ++i) g_sensor_off[i] = false;
}

static_assert(kSensCount == 7, "sensor chips");
const char* kSensLabel[kSensCount] = {
    "IMU", "L enc", "R enc", "Front", "Right", "Rear", "Left",
};
const int kSensWidth[kSensCount] = { 52, 62, 62, 58, 58, 54, 52 };

void drawSensorChip(HDC mem, const RECT& chip, const char* text, bool off, bool hot) {
    const COLORREF fill = off ? (hot ? RGB(120, 72, 32) : RGB(92, 58, 28))
                              : (hot ? RGB(48, 62, 72) : RGB(36, 40, 46));
    const COLORREF edge = off ? RGB(214, 148, 64) : RGB(90, 148, 168);
    const COLORREF ink = off ? RGB(255, 220, 180) : RGB(220, 226, 232);
    fillRoundRect(mem, chip.left, chip.top, chip.right, chip.bottom, 8, fill, edge);
    drawUiText(mem, chip, text, ink, DT_CENTER | DT_VCENTER | DT_SINGLELINE | DT_NOPREFIX);
}

void drawResetChip(HDC mem, const RECT& chip, const char* text, bool hot, bool idle) {
    const COLORREF fill = idle ? RGB(40, 44, 50) : (hot ? RGB(92, 64, 36) : RGB(64, 48, 32));
    const COLORREF edge = idle ? RGB(70, 74, 80) : RGB(214, 148, 64);
    const COLORREF ink = idle ? RGB(160, 164, 170) : RGB(255, 228, 196);
    fillRoundRect(mem, chip.left, chip.top, chip.right, chip.bottom, 8, fill, edge);
    drawUiText(mem, chip, text, ink, DT_CENTER | DT_VCENTER | DT_SINGLELINE | DT_NOPREFIX);
}

void drawPlantPanel(HDC mem, int fieldX, int fieldW) {
    clearPlantHits();
    if (fieldW < 160) {
        g_plant_drop = RECT{};
        return;
    }

    double shown[kKnobCount];
    bool sensor_off[kSensCount];
    {
        std::lock_guard<std::mutex> guard(g_plant_mu);
        for (int i = 0; i < kKnobCount; ++i) shown[i] = g_knob[i];
        for (int i = 0; i < kSensCount; ++i) sensor_off[i] = g_sensor_off[i];
    }
    bool any = false;
    for (int i = 0; i < kKnobCount; ++i) any = any || knobShifted(i, shown[i]);
    for (int i = 0; i < kSensCount; ++i) any = any || sensor_off[i];

    HFONT font = makeUiFont(14, FW_NORMAL);
    HFONT hintFont = makeUiFont(12, FW_NORMAL);
    const bool ownFont = font != nullptr;
    const bool ownHint = hintFont != nullptr;
    if (!ownFont) font = (HFONT)GetStockObject(DEFAULT_GUI_FONT);
    if (!ownHint) hintFont = font;
    HGDIOBJ oldFont = SelectObject(mem, font);
    SetBkMode(mem, TRANSPARENT);

    const int btnW = 168;
    const int btnH = 28;
    int btnRight = fieldX + fieldW - 12;
    int btnLeft = btnRight - btnW;
    if (btnLeft < fieldX + 8) btnLeft = fieldX + 8;
    const int btnTop = 70;
    g_plant_drop = RECT{btnLeft, btnTop, btnLeft + btnW, btnTop + btnH};
    if (g_plant_drop.right > fieldX + fieldW - 8) g_plant_drop.right = fieldX + fieldW - 8;

    if (g_plant_open) {
        const int panelW = std::min(460, fieldW - 16);
        int panelRight = g_plant_drop.right;
        int panelLeft = panelRight - panelW;
        if (panelLeft < fieldX + 8) panelLeft = fieldX + 8;
        const int pad = 12;
        const int left = panelLeft + pad;
        const int right = panelRight - pad;
        int cursor = g_plant_drop.bottom + 6 + pad;

        RECT caption{left, cursor, right, cursor + 36};
        cursor = caption.bottom + 8;

        // Shared plant on the left. Forward and yaw friction, speed, and
        // rise time on the right, so all ten stay above the robot.
        RECT resets[kKnobCount]{};
        RECT tracks[kKnobCount]{};
        const int colGap = 14;
        const int colW = (right - left - colGap) / 2;
        const int stride = 46;
        const int axisRows = 6;
        auto place = [&](int index, int col, int row) {
            const int x0 = left + col * (colW + colGap);
            const int x1 = x0 + colW;
            const int y = cursor + row * stride;
            resets[index] = RECT{x1 - 52, y, x1, y + 18};
            tracks[index] = RECT{x0, y + 34, x1, y + 42};
        };
        for (int i = 0; i < kKnobLinFriction; ++i) place(i, 0, i);
        for (int i = 0; i < axisRows; ++i) place(kKnobLinFriction + i, 1, i);
        cursor += axisRows * stride + 4;
        RECT sensCaption{left, cursor, right, cursor + 46};
        cursor = sensCaption.bottom + 6;
        const int chipH = 26;
        const int chipGap = 6;
        int chipX = left;
        int chipY = cursor;
        int rowBottom = chipY + chipH;
        for (int i = 0; i < kSensCount; ++i) {
            const int chipW = kSensWidth[i];
            if (chipX > left && chipX + chipW > right) {
                chipX = left;
                chipY += chipH + chipGap;
                rowBottom = chipY + chipH;
            }
            g_plant_sensor[i] = RECT{chipX, chipY, chipX + chipW, chipY + chipH};
            chipX += chipW + chipGap;
        }
        cursor = rowBottom + 10;
        RECT all{left, cursor, right, cursor + 28};
        const int panelBottom = all.bottom + pad;
        g_plant_panel = RECT{panelLeft, g_plant_drop.bottom + 6, panelRight, panelBottom};
        for (int i = 0; i < kKnobCount; ++i) {
            g_plant_reset[i] = resets[i];
            g_plant_track[i] = tracks[i];
        }
        g_plant_all = all;

        fillRoundRect(mem, g_plant_panel.left, g_plant_panel.top, g_plant_panel.right, g_plant_panel.bottom,
                      10, RGB(28, 30, 34), RGB(78, 84, 94));
        SelectObject(mem, hintFont);
        drawUiText(mem, caption,
                   "Domain randomization of the compiled plant. A tuned loop oscillates or misses when these move.",
                   RGB(168, 174, 182), DT_LEFT | DT_WORDBREAK | DT_NOPREFIX);

        for (int i = 0; i < kKnobCount; ++i) {
            const bool shifted = knobShifted(i, shown[i]);
            const bool trackHot = g_plant_hover == kHitTrack + i || g_plant_drag == i;
            const int rowLeft = tracks[i].left;
            const int rowRight = tracks[i].right;
            const int row = resets[i].top;
            RECT label{rowLeft, row, resets[i].left - 4, row + 16};
            char value[32];
            formatKnob(i, shown[i], value, (int)sizeof(value));
            SelectObject(mem, font);
            drawUiText(mem, label, kKnobs[i].label, RGB(230, 232, 236),
                       DT_LEFT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS | DT_NOPREFIX);
            const int valueW = 64;
            RECT hint{rowLeft, row + 18, rowRight - valueW - 4, row + 32};
            RECT valueRect{rowRight - valueW, row + 18, rowRight, row + 32};
            SelectObject(mem, hintFont);
            drawUiText(mem, hint, kKnobs[i].hint, RGB(140, 146, 154),
                       DT_LEFT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS | DT_NOPREFIX);
            SelectObject(mem, font);
            drawUiText(mem, valueRect, value,
                       shifted ? RGB(255, 196, 120) : RGB(186, 196, 206),
                       DT_RIGHT | DT_VCENTER | DT_SINGLELINE | DT_NOPREFIX);
            drawResetChip(mem, resets[i], "Reset", g_plant_hover == kHitReset + i, !shifted);

            const RECT& track = tracks[i];
            const COLORREF accent = shifted ? RGB(214, 148, 64) : RGB(90, 148, 168);
            fillRoundRect(mem, track.left, track.top, track.right, track.bottom, 8,
                          RGB(16, 18, 22), trackHot ? RGB(180, 186, 194) : RGB(48, 52, 58));
            const int width = track.right - track.left;
            const int hx = track.left + (int)std::lround(knobT(kKnobs[i], shown[i]) * width);
            if (hx > track.left + 2)
                fillRoundRect(mem, track.left, track.top, hx, track.bottom, 8, accent, accent);
            const int nx = track.left + (int)std::lround(knobT(kKnobs[i], kKnobs[i].original) * width);
            HPEN tickPen = CreatePen(PS_SOLID, 2, RGB(226, 228, 232));
            HGDIOBJ oldPen = SelectObject(mem, tickPen);
            MoveToEx(mem, nx, track.top + 1, nullptr);
            LineTo(mem, nx, track.bottom - 1);
            SelectObject(mem, oldPen);
            DeleteObject(tickPen);

            const int mid = (track.top + track.bottom) / 2;
            const int hr = 6;
            HBRUSH handleBrush = CreateSolidBrush(RGB(236, 236, 236));
            HPEN handlePen = CreatePen(PS_SOLID, 1, RGB(20, 22, 26));
            HGDIOBJ oldBrush = SelectObject(mem, handleBrush);
            oldPen = SelectObject(mem, handlePen);
            Ellipse(mem, hx - hr, mid - hr, hx + hr, mid + hr);
            SelectObject(mem, oldBrush);
            SelectObject(mem, oldPen);
            DeleteObject(handleBrush);
            DeleteObject(handlePen);
        }

        SelectObject(mem, hintFont);
        drawUiText(mem, sensCaption,
                   "IMU holds its last heading. L enc and R enc report no travel. Front, Right, Rear, and Left drop that distance beam.",
                   RGB(168, 174, 182), DT_LEFT | DT_WORDBREAK | DT_NOPREFIX);
        for (int i = 0; i < kSensCount; ++i) {
            drawSensorChip(mem, g_plant_sensor[i], kSensLabel[i], sensor_off[i],
                           g_plant_hover == kHitSensor + i);
        }

        drawResetChip(mem, all, "Reset plant", g_plant_hover == kHitAll, !any);
    }

    const bool dropHot = g_plant_hover == kHitDrop;
    const COLORREF dropEdge = any ? RGB(214, 148, 64) : RGB(90, 148, 168);
    const COLORREF dropFill = dropHot ? RGB(52, 56, 64) : RGB(40, 44, 50);
    fillRoundRect(mem, g_plant_drop.left, g_plant_drop.top, g_plant_drop.right, g_plant_drop.bottom,
                  8, dropFill, dropEdge);
    RECT dropText = g_plant_drop;
    dropText.left += 12;
    dropText.right -= 26;
    SelectObject(mem, font);
    drawUiText(mem, dropText, any ? "Plant *" : "Plant", RGB(236, 236, 236),
               DT_LEFT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS | DT_NOPREFIX);
    drawChevron(mem, g_plant_drop.right - 22, g_plant_drop.top + (btnH - 6) / 2, g_plant_open);

    SelectObject(mem, oldFont);
    if (ownHint && hintFont != font) DeleteObject(hintFont);
    if (ownFont) DeleteObject(font);
}

bool handlePlantClick(HWND hwnd, POINT p) {
    const int hit = plantHit(p);
    if (hit == kHitNone) return false;
    if (hit == kHitDrop) {
        g_plant_open = !g_plant_open;
        g_plant_drag = -1;
        ReleaseCapture();
        return true;
    }
    if (hit == kHitPanel) return true;
    if (hit >= kHitTrack && hit < kHitTrack + kKnobCount) {
        g_plant_drag = hit - kHitTrack;
        setKnobFromX(g_plant_drag, p.x);
        SetCapture(hwnd);
        return true;
    }
    if (hit >= kHitReset && hit < kHitReset + kKnobCount) {
        resetKnob(hit - kHitReset);
        return true;
    }
    if (hit >= kHitSensor && hit < kHitSensor + kSensCount) {
        const int index = hit - kHitSensor;
        std::lock_guard<std::mutex> guard(g_plant_mu);
        g_sensor_off[index] = !g_sensor_off[index];
        return true;
    }
    if (hit == kHitAll) {
        resetPlantKnobs();
        return true;
    }
    return true;
}

void paintField(HWND hwnd) {
    PAINTSTRUCT ps;
    HDC dc = BeginPaint(hwnd, &ps);
    RECT client{};
    GetClientRect(hwnd, &client);
    const int w = client.right - client.left;
    const int h = client.bottom - client.top;
    if (w <= 0 || h <= 0) {
        EndPaint(hwnd, &ps);
        return;
    }
    const int fieldW = w / 2 > 0 ? w / 2 : 1;
    const int fieldH = h;
    const int fieldX = 0;
    const int rightX = fieldW;
    const int rightW = w - fieldW > 0 ? w - fieldW : 1;

    HDC mem = CreateCompatibleDC(dc);
    HBITMAP bmp = CreateCompatibleBitmap(dc, w, h);
    HGDIOBJ oldBmp = SelectObject(mem, bmp);

    HBRUSH bg = CreateSolidBrush(RGB(28, 30, 34));
    FillRect(mem, &client, bg);
    DeleteObject(bg);

    ensurePreview();

    SaveDC(mem);
    IntersectClipRect(mem, fieldX, 0, fieldX + fieldW, fieldH);

    const double margin = 48.0;
    const bool room = (fieldW > (int)(2.0 * margin)) && (fieldH > (int)(2.0 * margin));
    double scale = 1.0;
    double cx = fieldX + fieldW / 2.0;
    // Keep the tile below the pose readout and above the key line.
    const int viewTop = 68;
    const int viewBottom = fieldH - 32;
    const double viewH = (double)(viewBottom > viewTop ? viewBottom - viewTop : 1);
    double cy = viewTop + viewH / 2.0;
    HPEN border = nullptr;
    HPEN grid = nullptr;
    HPEN origin = nullptr;
    HPEN trailPen = nullptr;
    HPEN estTrailPen = nullptr;
    HPEN curvePen = nullptr;
    HPEN robotPen = nullptr;
    HPEN estPen = nullptr;
    HPEN nose = nullptr;
    HPEN estNose = nullptr;
    HBRUSH fieldBrush = nullptr;
    HBRUSH robotBrush = nullptr;
    HBRUSH estBrush = nullptr;
    HGDIOBJ oldPen = nullptr;
    HGDIOBJ oldBrush = nullptr;

    if (room) {
        scale = std::min((fieldW - 2.0 * margin) / kFieldIn, (viewH - 8.0) / kFieldIn);
        const double half = kFieldIn / 2.0;
        POINT field[4] = {
            fieldToScreen(-half, kFieldIn, cx, cy, scale),
            fieldToScreen( half, kFieldIn, cx, cy, scale),
            fieldToScreen( half, 0.0, cx, cy, scale),
            fieldToScreen(-half, 0.0, cx, cy, scale),
        };
        fieldBrush = CreateSolidBrush(RGB(232, 232, 226));
        border = CreatePen(PS_SOLID, 2, RGB(40, 40, 40));
        oldPen = SelectObject(mem, border);
        oldBrush = SelectObject(mem, fieldBrush);
        Polygon(mem, field, 4);

        grid = CreatePen(PS_SOLID, 1, RGB(186, 186, 180));
        SelectObject(mem, grid);
        const double tile = 24.0;
        for (int i = 1; i < 6; ++i) {
            double x = -half + tile * i;
            POINT a = fieldToScreen(x, kFieldIn, cx, cy, scale);
            POINT b = fieldToScreen(x, 0.0, cx, cy, scale);
            MoveToEx(mem, a.x, a.y, nullptr);
            LineTo(mem, b.x, b.y);
            double y = tile * i;
            POINT c = fieldToScreen(-half, y, cx, cy, scale);
            POINT d = fieldToScreen( half, y, cx, cy, scale);
            MoveToEx(mem, c.x, c.y, nullptr);
            LineTo(mem, d.x, d.y);
        }

        origin = CreatePen(PS_SOLID, 1, RGB(140, 140, 140));
        SelectObject(mem, origin);
        const double mark = 0.15 * kInchesPerMeter;
        POINT o0 = fieldToScreen(-mark, 0.0, cx, cy, scale);
        POINT o1 = fieldToScreen( mark, 0.0, cx, cy, scale);
        POINT o2 = fieldToScreen(0.0, -mark, cx, cy, scale);
        POINT o3 = fieldToScreen(0.0,  mark, cx, cy, scale);
        MoveToEx(mem, o0.x, o0.y, nullptr);
        LineTo(mem, o1.x, o1.y);
        MoveToEx(mem, o2.x, o2.y, nullptr);
        LineTo(mem, o3.x, o3.y);

        drawGoals(mem, cx, cy, scale);

        trailPen = CreatePen(PS_SOLID, 2, RGB(70, 130, 210));
        estTrailPen = CreatePen(PS_SOLID, 2, RGB(230, 150, 40));
        curvePen = CreatePen(PS_SOLID, 3, RGB(36, 150, 72));
        drawTrail(mem, g_truth_trail, trailPen, cx, cy, scale);
        drawTrail(mem, g_est_trail, estTrailPen, cx, cy, scale);
        drawCurve(mem, curvePen, cx, cy, scale);

        HBRUSH dotBrush = CreateSolidBrush(RGB(120, 50, 170));
        HPEN dotPen = CreatePen(PS_SOLID, 1, RGB(70, 20, 110));
        SelectObject(mem, dotBrush);
        SelectObject(mem, dotPen);
        for (int i = 0; i < g_particle_count; ++i) {
            POINT c = fieldToScreen(g_particles[i].x, g_particles[i].y, cx, cy, scale);
            Ellipse(mem, c.x - 3, c.y - 3, c.x + 4, c.y + 4);
        }
        SelectObject(mem, oldPen);
        SelectObject(mem, oldBrush);
        DeleteObject(dotBrush);
        DeleteObject(dotPen);

        robotBrush = CreateSolidBrush(RGB(28, 86, 196));
        robotPen = CreatePen(PS_SOLID, 2, RGB(12, 32, 80));
        nose = CreatePen(PS_SOLID, 3, RGB(210, 48, 40));
        drawChassis(mem, g_truth, cx, cy, scale, robotBrush, robotPen, nose);

        estBrush = (HBRUSH)GetStockObject(HOLLOW_BRUSH);
        estPen = CreatePen(PS_SOLID, 2, RGB(230, 150, 40));
        estNose = CreatePen(PS_SOLID, 3, RGB(255, 196, 70));
        drawChassis(mem, g_estimate, cx, cy, scale, estBrush, estPen, estNose);

        SelectObject(mem, oldPen);
        SelectObject(mem, oldBrush);
    }

    RECT banner{fieldX, 0, fieldX + fieldW, 64};
    HBRUSH bannerBrush = CreateSolidBrush(RGB(28, 30, 34));
    FillRect(mem, &banner, bannerBrush);
    DeleteObject(bannerBrush);

    SetBkMode(mem, TRANSPARENT);
    SetTextColor(mem, RGB(236, 236, 236));
    HFONT font = (HFONT)GetStockObject(DEFAULT_GUI_FONT);
    HGDIOBJ oldFont = SelectObject(mem, font);
    wchar_t line[180];
    int n = std::swprintf(line, 180, L"true  x %+7.1f   y %+7.1f in   h %+6.1f deg",
                          g_truth.x, g_truth.y, g_truth.theta * (180.0 / kPi));
    if (n > 0) TextOutW(mem, fieldX + 12, 8, line, n);
    n = std::swprintf(line, 180, L"est   x %+7.1f   y %+7.1f in   h %+6.1f deg",
                      g_estimate.x, g_estimate.y, g_estimate.theta * (180.0 / kPi));
    if (n > 0) TextOutW(mem, fieldX + 12, 26, line, n);
    double dh = g_estimate.theta - g_truth.theta;
    while (dh > kPi) dh -= 2.0 * kPi;
    while (dh < -kPi) dh += 2.0 * kPi;
    n = std::swprintf(line, 180, L"err   dx %+5.2f   dy %+5.2f   dh %+5.1f    blue true  orange est  green curve",
                      g_estimate.x - g_truth.x, g_estimate.y - g_truth.y, dh * (180.0 / kPi));
    if (n > 0) TextOutW(mem, fieldX + 12, 44, line, n);
    const wchar_t* help =
        L"WASD left stick    arrows right stick    TFGH d-pad    IJKL A/B/X/Y    ZXCV L1/L2/R1/R2";
    TextOutW(mem, fieldX + 12, fieldH - 24, help, (int)std::wcslen(help));
    SelectObject(mem, oldFont);
    RestoreDC(mem, -1);

    drawSidePanel(mem, rightX, 0, rightW, h);
    drawPlantPanel(mem, fieldX, fieldW);

    BitBlt(dc, 0, 0, w, h, mem, 0, 0, SRCCOPY);

    SelectObject(mem, oldBmp);
    DeleteObject(bmp);
    DeleteDC(mem);
    DeleteObject(fieldBrush);
    DeleteObject(border);
    DeleteObject(grid);
    DeleteObject(origin);
    DeleteObject(trailPen);
    DeleteObject(estTrailPen);
    DeleteObject(curvePen);
    DeleteObject(robotBrush);
    DeleteObject(robotPen);
    DeleteObject(estPen);
    DeleteObject(nose);
    DeleteObject(estNose);
    EndPaint(hwnd, &ps);
}

LRESULT CALLBACK fieldWndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
        case WM_PAINT:
            paintField(hwnd);
            return 0;
        case WM_LBUTTONDOWN: {
            POINT p{ (LONG)(short)LOWORD(lp), (LONG)(short)HIWORD(lp) };
            if (handlePlantClick(hwnd, p)) {
                InvalidateRect(hwnd, nullptr, FALSE);
                return 0;
            }
            if (handleAutonClick(p)) {
                InvalidateRect(hwnd, nullptr, FALSE);
                return 0;
            }
            if (g_brain_w > 0 && PtInRect(&g_brain_lcd, p)) g_skip_brain = true;
            return 0;
        }
        case WM_LBUTTONUP: {
            if (g_plant_drag >= 0) {
                g_plant_drag = -1;
                ReleaseCapture();
                InvalidateRect(hwnd, nullptr, FALSE);
            }
            return 0;
        }
        case WM_MOUSEMOVE: {
            POINT p{ (LONG)(short)LOWORD(lp), (LONG)(short)HIWORD(lp) };
            if (g_plant_drag >= 0) setKnobFromX(g_plant_drag, p.x);
            const int plantHover = plantHit(p);
            int hoverRow = -1;
            int hoverMenu = -1;
            int hoverColor = -1;
            const bool hoverDrop = PtInRect(&g_drop_rect, p) != FALSE;
            if (g_drop_open) {
                for (int i = 0; i < kSideCount; ++i) {
                    if (PtInRect(&g_menu_rect[i], p)) hoverMenu = i;
                }
            } else {
                for (int i = 0; i < g_row_n; ++i) {
                    if (PtInRect(&g_row_rect[i], p)) hoverRow = g_row_index[i];
                }
                for (int i = 0; i < 2; ++i) {
                    if (PtInRect(&g_color_rect[i], p)) hoverColor = i;
                }
            }
            if (hoverRow != g_hover_row || hoverMenu != g_hover_menu || hoverColor != g_hover_color
                || hoverDrop != g_hover_drop || plantHover != g_plant_hover || g_plant_drag >= 0) {
                g_hover_row = hoverRow;
                g_hover_menu = hoverMenu;
                g_hover_color = hoverColor;
                g_hover_drop = hoverDrop;
                g_plant_hover = plantHover;
                InvalidateRect(hwnd, nullptr, FALSE);
            }
            TRACKMOUSEEVENT ev{};
            ev.cbSize = sizeof(ev);
            ev.dwFlags = TME_LEAVE;
            ev.hwndTrack = hwnd;
            TrackMouseEvent(&ev);
            return 0;
        }
        case WM_MOUSELEAVE:
            g_hover_row = -1;
            g_hover_menu = -1;
            g_hover_color = -1;
            g_hover_drop = false;
            g_plant_hover = kHitNone;
            InvalidateRect(hwnd, nullptr, FALSE);
            return 0;
        case WM_MOUSEWHEEL: {
            POINT p{ (LONG)(short)LOWORD(lp), (LONG)(short)HIWORD(lp) };
            ScreenToClient(hwnd, &p);
            if (PtInRect(&g_list_rect, p)) {
                const int delta = (int)(short)HIWORD(wp);
                if (delta > 0) --g_list_scroll;
                else if (delta < 0) ++g_list_scroll;
                if (g_list_scroll < 0) g_list_scroll = 0;
                InvalidateRect(hwnd, nullptr, FALSE);
            }
            return 0;
        }
        case WM_DESTROY:
            g_hwnd = nullptr;
            PostQuitMessage(0);
            return 0;
        case WM_CLOSE:
            DestroyWindow(hwnd);
            return 0;
        default:
            return DefWindowProcW(hwnd, msg, wp, lp);
    }
}

void ensureFieldWindow() {
    if (g_hwnd != nullptr) return;
    HINSTANCE inst = GetModuleHandleW(nullptr);
    WNDCLASSW wc{};
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = fieldWndProc;
    wc.hInstance = inst;
    wc.lpszClassName = L"MockDriveField";
    wc.hCursor = LoadCursorA(nullptr, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    RegisterClassW(&wc);

    RECT r{0, 0, kPaneW * 2, kWindowH};
    DWORD style = WS_OVERLAPPEDWINDOW;
    AdjustWindowRect(&r, style, FALSE);
    g_hwnd = CreateWindowW(
        L"MockDriveField", L"MockDrive",
        style | WS_VISIBLE,
        CW_USEDEFAULT, CW_USEDEFAULT,
        r.right - r.left, r.bottom - r.top,
        nullptr, nullptr, inst, nullptr);
}

}  // namespace

namespace fieldwindow {

PlantOverride plantOverride() {
    std::lock_guard<std::mutex> guard(g_plant_mu);
    PlantOverride over;
    over.velocity_tau_sec = g_knob[kKnobLag];
    over.skid_tau_sec = g_knob[kKnobSkid];
    over.speed_scale = g_knob[kKnobSpeed];
    over.lin_friction_v = g_knob[kKnobLinFriction];
    over.lin_vel_scale = g_knob[kKnobLinVel];
    over.lin_accel_scale = g_knob[kKnobLinAccel];
    over.ang_friction_v = g_knob[kKnobAngFriction];
    over.ang_vel_scale = g_knob[kKnobAngVel];
    over.ang_accel_scale = g_knob[kKnobAngAccel];
    const double tick_ms = System::DT_SEC * 1000.0;
    int ticks = 0;
    if (tick_ms > 1e-6) ticks = (int)std::lround(g_knob[kKnobDelay] / tick_ms);
    if (ticks < 0) ticks = 0;
    over.voltage_delay_ticks = ticks;
    over.imu_off = g_sensor_off[kSensImu];
    over.enc_left_off = g_sensor_off[kSensEncL];
    over.enc_right_off = g_sensor_off[kSensEncR];
    for (int i = 0; i < kDistanceChannels; ++i)
        over.distance_off[i] = g_sensor_off[kSensDist + i];
    return over;
}

void pollInput(JoystickData& joy, ButtonState& buttons) {
    joy.left_y  = stickAxis('S', 'W');
    joy.left_x  = stickAxis('A', 'D');
    joy.right_y = stickAxis(VK_DOWN, VK_UP);
    joy.right_x = stickAxis(VK_LEFT, VK_RIGHT);

    ButtonState b;
    b.up    = keyDown('T');
    b.right = keyDown('H');
    b.down  = keyDown('G');
    b.left  = keyDown('F');
    b.a  = keyDown('L');
    b.b  = keyDown('K');
    b.x  = keyDown('I');
    b.y  = keyDown('J');
    b.l1 = keyDown('Z');
    b.l2 = keyDown('X');
    b.r1 = keyDown('V');
    b.r2 = keyDown('C');
    buttons = b;
}

void open() {
    ensureFieldWindow();
}

void render(const Pose2d& truth, const Pose2d& estimate,
            const Pose2d* particles, int particle_count) {
    // The window belongs to the main thread. Creating it here would leave
    // its messages on this task, which never dispatches them.
    if (g_hwnd == nullptr) return;
    g_truth = truth;
    g_estimate = estimate;
    g_particle_count = 0;
    if (particles != nullptr && particle_count > 0) {
        if (particle_count > kMaxParticleDots) particle_count = kMaxParticleDots;
        for (int i = 0; i < particle_count; ++i) g_particles[i] = particles[i];
        g_particle_count = particle_count;
    }
    const ULONGLONG now = GetTickCount64();
    rememberPose(g_truth_trail, truth, now);
    rememberPose(g_est_trail, estimate, now);
    if (g_curve_lingering && now >= g_curve_expire_ms) g_curve_count = 0;
    // Posted from the pipeline thread. The window's thread dispatches it.
    InvalidateRect(g_hwnd, nullptr, FALSE);
}

void run() {
    ensureFieldWindow();
    for (;;) {
        MSG msg;
        while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) std::exit(0);
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
        if (g_hwnd == nullptr) std::exit(0);
        MsgWaitForMultipleObjects(0, nullptr, FALSE, 16, QS_ALLINPUT);
    }
}

void setMechanisms(const SubsystemCommand& cmd, double cascade_frac,
                   const CascadeReading& sense, const char* score_phase) {
    g_intake_pct = cmd.intake_pct;
    g_cascade_pct = cmd.cascade_pct;
    g_pneu_g = cmd.pneumatics_g_state;
    g_pneu_b = cmd.pneumatics_b_state;
    if (cascade_frac < 0.0) cascade_frac = 0.0;
    if (cascade_frac > 1.0) cascade_frac = 1.0;
    g_cascade_frac = cascade_frac;
    g_cascade_deg = sense.position_deg;
    g_cascade_rpm = sense.velocity_rpm;
    g_cascade_amps = sense.current_amps;
    if (score_phase == nullptr || score_phase[0] == '\0') score_phase = "Idle";
    std::snprintf(g_score_phase, sizeof(g_score_phase), "%s", score_phase);
}

void setBrainFrame(const std::uint32_t* rgb, int width, int height) {
    if (rgb == nullptr || width <= 0 || height <= 0) {
        g_brain_w = 0;
        g_brain_h = 0;
        g_brain_dib.clear();
        return;
    }
    g_brain_w = width;
    g_brain_h = height;
    g_brain_dib.resize((std::size_t)width * (std::size_t)height * 4u);
    for (int y = 0; y < height; ++y) {
        std::uint8_t* dst = g_brain_dib.data() +
            (std::size_t)(height - 1 - y) * (std::size_t)width * 4u;
        const std::uint32_t* src = rgb + (std::size_t)y * (std::size_t)width;
        for (int x = 0; x < width; ++x) {
            std::uint32_t c = src[x];
            dst[0] = (std::uint8_t)(c & 0xffu);
            dst[1] = (std::uint8_t)((c >> 8) & 0xffu);
            dst[2] = (std::uint8_t)((c >> 16) & 0xffu);
            dst[3] = 0;
            dst += 4;
        }
    }
}

bool pump(std::uint32_t hold_ms) {
    ensureFieldWindow();
    if (g_hwnd == nullptr) return false;
    g_skip_brain = false;
    InvalidateRect(g_hwnd, nullptr, FALSE);
    UpdateWindow(g_hwnd);

    const ULONGLONG deadline = GetTickCount64() + hold_ms;
    for (;;) {
        MSG msg;
        while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) std::exit(0);
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
        if (g_hwnd == nullptr) std::exit(0);
        // Z X C V are L1 L2 R2 R1. All four skip the clip, same as the robot.
        const bool shoulders = keyDown('Z') && keyDown('X') && keyDown('C') && keyDown('V');
        if (g_skip_brain || shoulders || (GetAsyncKeyState(VK_ESCAPE) & 0x8000) != 0) return false;
        const ULONGLONG now = GetTickCount64();
        if (now >= deadline) return true;
        const ULONGLONG left = deadline - now;
        const DWORD slice = left > 15ull ? 15u : (DWORD)left;
        MsgWaitForMultipleObjects(0, nullptr, FALSE, slice, QS_ALLINPUT);
    }
}

void setFollowPath(const Pose2d* points, int count, bool active) {
    const ULONGLONG now = GetTickCount64();
    if (active && points != nullptr && count >= 2) {
        if (count > kMaxCurvePts) count = kMaxCurvePts;
        for (int i = 0; i < count; ++i) g_curve[i] = points[i];
        g_curve_count = count;
        g_curve_lingering = false;
        return;
    }
    if (g_curve_count >= 2 && !g_curve_lingering) {
        g_curve_lingering = true;
        g_curve_expire_ms = now + kPathLifeMs;
    }
    if (g_curve_lingering && now >= g_curve_expire_ms) g_curve_count = 0;
}

}  // namespace fieldwindow
