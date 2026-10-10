/* ---------------------------------------------------------------------------
 * FieldWindow.cpp  --  Win32 keyboard and field view for the host build.
 * MockDrive calls these; the V5 link does not include this file.
 * ------------------------------------------------------------------------- */
#include "host/FieldWindow.h"
#include "core/Specs.h"
#include "host/DeviceScreens.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cwchar>
#include <vector>
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

namespace {

constexpr int kFieldPx = 760;
constexpr int kPanelW = 520;
constexpr double kFieldIn = 12.0 * 12.0;
constexpr double kBodyLengthIn = 0.38 * kInchesPerMeter;
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

bool keyDown(int vk) {
    return (GetAsyncKeyState(vk) & 0x8000) != 0;
}

double stickAxis(int negative, int positive) {
    return (keyDown(positive) ? 1.0 : 0.0) - (keyDown(negative) ? 1.0 : 0.0);
}

POINT fieldToScreen(double x, double y, double cx, double cy, double scale) {
    POINT p;
    p.x = (LONG)std::lround(cx + x * scale);
    p.y = (LONG)std::lround(cy - y * scale);
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
    const double hl = kBodyLengthIn / 2.0;
    const double hw = Robot::Chassis::TRACK_WIDTH_M * kInchesPerMeter / 2.0;
    const double cth = std::cos(pose.theta);
    const double sth = std::sin(pose.theta);
    const double rx[4] = {hl, hl, -hl, -hl};
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
    POINT nose1 = fieldToScreen(pose.x + (hl + nose_in) * cth,
                                pose.y + (hl + nose_in) * sth, cx, cy, scale);
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

void drawCentered(HDC mem, int left, int top, int right, int bottom, const char* text) {
    SIZE sz{};
    int n = (int)std::strlen(text);
    GetTextExtentPoint32A(mem, text, n, &sz);
    int x = left + ((right - left) - sz.cx) / 2;
    int y = top + ((bottom - top) - sz.cy) / 2;
    TextOutA(mem, x, y, text, n);
}

int fitFontPx(int lineH, int width, int cols) {
    int byLine = lineH - 4;
    int byCols = cols > 0 ? (width * 10) / (6 * cols) : byLine;
    int px = byLine < byCols ? byLine : byCols;
    if (px < 8) px = 8;
    if (px > 22) px = 22;
    return px;
}

HFONT makeMono(int px) {
    return CreateFontW(-px, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
                       ANSI_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                       CLEARTYPE_QUALITY, FIXED_PITCH | FF_MODERN, L"Consolas");
}

void drawLcdLines(HDC mem, int left, int top, int right, int bottom,
                  int rows, int cols, COLORREF color, const char* const* lines) {
    if (rows < 1 || bottom <= top || right <= left) return;
    int lineH = (bottom - top) / rows;
    if (lineH < 1) lineH = 1;
    HFONT font = makeMono(fitFontPx(lineH, right - left, cols));
    if (font == nullptr) return;
    HGDIOBJ old = SelectObject(mem, font);
    SetBkMode(mem, TRANSPARENT);
    SetTextColor(mem, color);
    TEXTMETRIC tm{};
    GetTextMetrics(mem, &tm);
    int extra = lineH - (int)tm.tmHeight;
    if (extra < 0) extra = 0;
    SaveDC(mem);
    IntersectClipRect(mem, left, top, right, bottom);
    for (int i = 0; i < rows; ++i) {
        const char* s = lines[i] != nullptr ? lines[i] : "";
        TextOutA(mem, left, top + i * lineH + extra / 2, s, (int)std::strlen(s));
    }
    RestoreDC(mem, -1);
    SelectObject(mem, old);
    DeleteObject(font);
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

void drawSidePanel(HDC mem, int x, int y, int w, int h) {
    if (w < 40 || h < 40) return;

    devicescreens::ControllerView ctrl{};
    devicescreens::copyController(ctrl);

    HPEN sep = CreatePen(PS_SOLID, 1, RGB(54, 58, 64));
    HGDIOBJ oldPen = SelectObject(mem, sep);
    MoveToEx(mem, x, y, nullptr);
    LineTo(mem, x, y + h);
    SelectObject(mem, oldPen);
    DeleteObject(sep);

    const int pad = 16;
    const int innerW = w - pad * 2;
    if (innerW < 80) return;
    int cursor = y + pad;
    const int originX = x + pad;

    drawLabel(mem, originX, cursor, "Brain");
    cursor += 20;

    int brainW = innerW;
    int brainH = brainW * 272 / 480;
    const int below = 18 + 20 + 108 + pad;
    int maxBrainH = h - cursor - below;
    if (maxBrainH < 96) maxBrainH = 96;
    if (brainH > maxBrainH) {
        brainH = maxBrainH;
        brainW = brainH * 480 / 272;
        if (brainW > innerW) brainW = innerW;
    }
    const int brainX = originX + (innerW - brainW) / 2;

    fillRoundRect(mem, brainX, cursor, brainX + brainW, cursor + brainH, 16,
                  RGB(52, 56, 64), RGB(30, 32, 38));

    int inset = brainH / 28;
    if (inset < 6) inset = 6;
    const int lcdL = brainX + inset;
    const int lcdT = cursor + inset;
    const int lcdR = brainX + brainW - inset;
    const int lcdB = cursor + brainH - inset;
    fillRoundRect(mem, lcdL, lcdT, lcdR, lcdB, 8, RGB(246, 246, 244), RGB(210, 210, 206));
    g_brain_lcd.left = lcdL;
    g_brain_lcd.top = lcdT;
    g_brain_lcd.right = lcdR;
    g_brain_lcd.bottom = lcdB;

    const bool brain_frame = g_brain_w > 0 && g_brain_h > 0 &&
        g_brain_dib.size() == (std::size_t)g_brain_w * (std::size_t)g_brain_h * 4u;
    if (brain_frame) {
        blitBrainLcd(mem, lcdL, lcdT, lcdR, lcdB);
    } else {
        int btnH = (lcdB - lcdT) * 40 / 272;
        if (btnH < 18) btnH = 18;
        const int textL = lcdL + 10;
        const int textR = lcdR - 8;
        const int textT = lcdT + 8;
        const int textB = lcdB - btnH - 10;
        devicescreens::BrainView brain{};
        devicescreens::copyBrain(brain);
        const char* brainLines[devicescreens::kBrainLines];
        for (int i = 0; i < devicescreens::kBrainLines; ++i) brainLines[i] = brain.lines[i];
        drawLcdLines(mem, textL, textT, textR, textB, devicescreens::kBrainLines,
                     devicescreens::kBrainCols, RGB(24, 26, 30), brainLines);

        const int btnT = lcdB - btnH - 6;
        const int btnBtm = lcdB - 6;
        const int gap = 8;
        const int btnW = (textR - textL - gap * 2) / 3;
        const char* btnName[3] = {"Left", "Center", "Right"};
        if (btnW > 8 && btnBtm > btnT) {
            HFONT font = (HFONT)GetStockObject(DEFAULT_GUI_FONT);
            HGDIOBJ oldFont = SelectObject(mem, font);
            SetBkMode(mem, TRANSPARENT);
            SetTextColor(mem, RGB(36, 40, 46));
            for (int i = 0; i < 3; ++i) {
                int left = textL + i * (btnW + gap);
                fillRoundRect(mem, left, btnT, left + btnW, btnBtm, 8,
                              RGB(214, 216, 220), RGB(150, 154, 160));
                drawCentered(mem, left, btnT, left + btnW, btnBtm, btnName[i]);
            }
            SelectObject(mem, oldFont);
        }
    }

    cursor += brainH + 18;
    if (cursor + 48 >= y + h) return;

    drawLabel(mem, originX, cursor, "Controller");
    cursor += 20;
    int ctrlH = 108;
    if (cursor + ctrlH > y + h - 8) ctrlH = y + h - 8 - cursor;
    if (ctrlH < 36) return;

    fillRoundRect(mem, originX, cursor, originX + innerW, cursor + ctrlH, 14,
                  RGB(46, 48, 54), RGB(24, 26, 30));
    const int cInset = 8;
    const int cL = originX + cInset;
    const int cT = cursor + cInset;
    const int cR = originX + innerW - cInset;
    const int cB = cursor + ctrlH - cInset;
    fillRoundRect(mem, cL, cT, cR, cB, 6, RGB(8, 12, 8), RGB(20, 28, 20));

    const char* ctrlLines[devicescreens::kCtrlLines];
    for (int i = 0; i < devicescreens::kCtrlLines; ++i) ctrlLines[i] = ctrl.lines[i];
    drawLcdLines(mem, cL + 8, cT + 4, cR - 6, cB - 4, devicescreens::kCtrlLines,
                 devicescreens::kCtrlCols, RGB(176, 230, 168), ctrlLines);
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
    int panelW = kPanelW;
    if (panelW > w / 2) panelW = w / 2;
    if (panelW < 1) panelW = 1;
    const int fieldW = w - panelW;

    HDC mem = CreateCompatibleDC(dc);
    HBITMAP bmp = CreateCompatibleBitmap(dc, w, h);
    HGDIOBJ oldBmp = SelectObject(mem, bmp);

    HBRUSH bg = CreateSolidBrush(RGB(28, 30, 34));
    FillRect(mem, &client, bg);
    DeleteObject(bg);

    SaveDC(mem);
    IntersectClipRect(mem, 0, 0, fieldW, h);

    const double margin = 48.0;
    const bool room = (fieldW > (int)(2.0 * margin)) && (h > (int)(2.0 * margin));
    double scale = 1.0;
    double cx = fieldW / 2.0;
    double cy = h / 2.0;
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
        scale = std::min((fieldW - 2.0 * margin) / kFieldIn, (h - 2.0 * margin) / kFieldIn);
        const double half = kFieldIn / 2.0;
        POINT field[4] = {
            fieldToScreen(-half,  half, cx, cy, scale),
            fieldToScreen( half,  half, cx, cy, scale),
            fieldToScreen( half, -half, cx, cy, scale),
            fieldToScreen(-half, -half, cx, cy, scale),
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
            double t = -half + tile * i;
            POINT a = fieldToScreen(t,  half, cx, cy, scale);
            POINT b = fieldToScreen(t, -half, cx, cy, scale);
            MoveToEx(mem, a.x, a.y, nullptr);
            LineTo(mem, b.x, b.y);
            POINT c = fieldToScreen(-half, t, cx, cy, scale);
            POINT d = fieldToScreen( half, t, cx, cy, scale);
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

    RECT banner{0, 0, fieldW, 64};
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
    if (n > 0) TextOutW(mem, 12, 8, line, n);
    n = std::swprintf(line, 180, L"est   x %+7.1f   y %+7.1f in   h %+6.1f deg",
                      g_estimate.x, g_estimate.y, g_estimate.theta * (180.0 / kPi));
    if (n > 0) TextOutW(mem, 12, 26, line, n);
    double dh = g_estimate.theta - g_truth.theta;
    while (dh > kPi) dh -= 2.0 * kPi;
    while (dh < -kPi) dh += 2.0 * kPi;
    n = std::swprintf(line, 180, L"err   dx %+5.2f   dy %+5.2f   dh %+5.1f    blue true  orange est  green curve",
                      g_estimate.x - g_truth.x, g_estimate.y - g_truth.y, dh * (180.0 / kPi));
    if (n > 0) TextOutW(mem, 12, 44, line, n);
    const wchar_t* help =
        L"WASD left stick    arrows right stick    TFGH d-pad    IJKL A/B/X/Y    ZXCV L1/L2/R1/R2";
    TextOutW(mem, 12, h - 24, help, (int)std::wcslen(help));
    SelectObject(mem, oldFont);
    RestoreDC(mem, -1);

    drawSidePanel(mem, fieldW, 0, panelW, h);

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
        case WM_LBUTTONDOWN:
            if (g_brain_w > 0) {
                POINT p{ (LONG)(short)LOWORD(lp), (LONG)(short)HIWORD(lp) };
                if (PtInRect(&g_brain_lcd, p)) g_skip_brain = true;
            }
            return 0;
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

    RECT r{0, 0, kFieldPx + kPanelW, kFieldPx};
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
        if (g_skip_brain || (GetAsyncKeyState(VK_ESCAPE) & 0x8000) != 0) return false;
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
