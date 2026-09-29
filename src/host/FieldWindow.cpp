/* ---------------------------------------------------------------------------
 * FieldWindow.cpp  --  Win32 keyboard and field view for the host build.
 * MockDrive calls these; the V5 link does not include this file.
 * ------------------------------------------------------------------------- */
#include "host/FieldWindow.h"
#include "core/Specs.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cwchar>
#include <vector>
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

namespace {

constexpr int kWindowPx = 760;
constexpr double kFieldM = 12.0 * 0.3048;
constexpr double kBodyLengthM = 0.38;
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
        if (d2 > 0.25) trail.clear();
        else if (d2 < 0.0001) return;
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
    const double hl = kBodyLengthM / 2.0;
    const double hw = Robot::Chassis::TRACK_WIDTH_M / 2.0;
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
    POINT nose1 = fieldToScreen(pose.x + (hl + 0.06) * cth,
                                pose.y + (hl + 0.06) * sth, cx, cy, scale);
    SelectObject(mem, nosePen);
    MoveToEx(mem, nose0.x, nose0.y, nullptr);
    LineTo(mem, nose1.x, nose1.y);
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

    HDC mem = CreateCompatibleDC(dc);
    HBITMAP bmp = CreateCompatibleBitmap(dc, w, h);
    HGDIOBJ oldBmp = SelectObject(mem, bmp);

    HBRUSH bg = CreateSolidBrush(RGB(28, 30, 34));
    FillRect(mem, &client, bg);
    DeleteObject(bg);

    const double margin = 48.0;
    const bool room = (w > (int)(2.0 * margin)) && (h > (int)(2.0 * margin));
    double scale = 1.0;
    double cx = w / 2.0;
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
        scale = std::min((w - 2.0 * margin) / kFieldM, (h - 2.0 * margin) / kFieldM);
        const double half = kFieldM / 2.0;
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
        const double tile = 2.0 * 0.3048;
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
        POINT o0 = fieldToScreen(-0.15, 0.0, cx, cy, scale);
        POINT o1 = fieldToScreen( 0.15, 0.0, cx, cy, scale);
        POINT o2 = fieldToScreen(0.0, -0.15, cx, cy, scale);
        POINT o3 = fieldToScreen(0.0,  0.15, cx, cy, scale);
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

    RECT banner{0, 0, w, 64};
    HBRUSH bannerBrush = CreateSolidBrush(RGB(28, 30, 34));
    FillRect(mem, &banner, bannerBrush);
    DeleteObject(bannerBrush);

    SetBkMode(mem, TRANSPARENT);
    SetTextColor(mem, RGB(236, 236, 236));
    HFONT font = (HFONT)GetStockObject(DEFAULT_GUI_FONT);
    HGDIOBJ oldFont = SelectObject(mem, font);
    wchar_t line[180];
    int n = std::swprintf(line, 180, L"true  x %+6.2f   y %+6.2f   h %+6.1f deg",
                          g_truth.x, g_truth.y, g_truth.theta * (180.0 / kPi));
    if (n > 0) TextOutW(mem, 12, 8, line, n);
    n = std::swprintf(line, 180, L"est   x %+6.2f   y %+6.2f   h %+6.1f deg",
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

    RECT r{0, 0, kWindowPx, kWindowPx};
    DWORD style = WS_OVERLAPPEDWINDOW;
    AdjustWindowRect(&r, style, FALSE);
    g_hwnd = CreateWindowW(
        L"MockDriveField", L"MockDrive field",
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

void render(const Pose2d& truth, const Pose2d& estimate,
            const Pose2d* particles, int particle_count) {
    ensureFieldWindow();
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
    InvalidateRect(g_hwnd, nullptr, FALSE);
    MSG msg;
    while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
        if (msg.message == WM_QUIT) std::exit(0);
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
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
