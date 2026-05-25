// settings_ui.cpp -- the classic Win32 settings dialog (the second part of the
// module that touches the Windows API, alongside preview.cpp). A DLL-owned thread
// registers a global hotkey (Ctrl+Alt+M) and serves a modeless dialog with
// trackbars for the detection knobs (fiducial diameter bracket, Round accept
// threshold) and a grayscale image-adjustment set (gamma, brightness, contrast,
// black/white levels, sharpen), plus a LIVE readout (active mode, LOCK/NO-LOCK,
// score, radius, offset) fed by the detector. The dialog writes vis::Settings (which
// the detectors read) and persists to a text file.
//
// Build-only on Windows; not part of the off-target test harness.

#include "settings_ui.h"

#include "controller.h"
#include "settings.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <commctrl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <fstream>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>

#ifndef MVISION_VERSION        // injected by CMake from `git describe`; fallback for ad-hoc builds
#define MVISION_VERSION "dev"  // NOLINT(cppcoreguidelines-macro-usage) -- build-injected via -D
#endif

namespace vis {
namespace {

const char* const kClassName = "MVisionSettingsWnd";
// MVision.ini in the working directory. Resolved to an ABSOLUTE path once at startup
// (the host changes CWD via its file dialogs, so a relative path would drift between
// load and save).
std::string g_iniPath;

enum : int { ID_TIMER = 1,
             HOTKEY_ID = 1,
             ID_BTN_SAVE = 1100,
             ID_BTN_RESET = 1101,
             ID_BTN_CLOSE = 1102,
             ID_CHK_MEDIAN = 1103,
             // Detector master switches -- consecutive, in METHOD_* order, so
             // method = id - ID_CHK_ROUND.
             ID_CHK_ROUND = 1104,
             ID_CHK_CIRCULAR = 1105,
             ID_CHK_TEMPLATE = 1106,
             ID_CHK_COMP = 1107,
             ID_CBO_MODE = 1108 };  // "Edit:" detector dropdown

// One slider per tunable. `kind` drives both the value-label format and the
// Settings <-> trackbar mapping.
enum Kind { K_PX,
            K_THR,
            K_SENS,  // Round accept threshold: quadratic curve (fine low end), 0 = "Auto"
            K_GAMMA,
            K_SINT,
            K_INT,
            K_STENTHS,
            K_AUTOINT,  // 0 = "Auto", else integer
            K_BLUR };   // 0 = "Auto", else odd kernel "NN px"
enum Idx { S_RMIN,
           S_RMAX,
           S_SYM,
           S_GAMMA,
           S_BRI,
           S_CON,
           S_BLK,
           S_WHT,
           S_SHP,
           S_EXPLO,
           S_EXPHI,
           S_BLUR,
           S_COUNT };

struct SliderDef {
    const char* label;
    int lo, hi;
    Kind kind;
    int y;
};

// Layout (client 420 x 482). Group headers + the median checkbox are separate
// statics/buttons. Entries are in Idx order; the y values place them in two groups.
const std::array<SliderDef, S_COUNT> kDefs = {{
    {.label = "Diameter min", .lo = 0, .hi = 120, .kind = K_PX, .y = 46},        // S_RMIN (slider is RADIUS px; shown as diameter)
    {.label = "Diameter max", .lo = 0, .hi = 120, .kind = K_PX, .y = 71},        // S_RMAX (slider is RADIUS px; shown as diameter)
    {.label = "Sensitivity", .lo = 0, .hi = 200, .kind = K_SENS, .y = 96},       // S_SYM   (accept threshold, quadratic map)
    {.label = "Gamma", .lo = 0, .hi = 40, .kind = K_GAMMA, .y = 214},            // S_GAMMA (x10)
    {.label = "Brightness", .lo = -128, .hi = 128, .kind = K_SINT, .y = 239},    // S_BRI
    {.label = "Contrast", .lo = 0, .hi = 30, .kind = K_THR, .y = 264},           // S_CON   (x10)
    {.label = "Black point", .lo = 0, .hi = 128, .kind = K_INT, .y = 289},       // S_BLK
    {.label = "White point", .lo = 0, .hi = 128, .kind = K_INT, .y = 314},       // S_WHT
    {.label = "Sharpen", .lo = -10, .hi = 10, .kind = K_STENTHS, .y = 339},      // S_SHP   (x10)
    {.label = "Exposure min", .lo = 0, .hi = 128, .kind = K_AUTOINT, .y = 121},  // S_EXPLO (frame-mean gate low)
    {.label = "Exposure max", .lo = 0, .hi = 255, .kind = K_AUTOINT, .y = 146},  // S_EXPHI (frame-mean gate high)
    {.label = "Blur", .lo = 0, .hi = 25, .kind = K_BLUR, .y = 364},              // S_BLUR  (pre-blur kernel px)
}};

HINSTANCE g_inst = nullptr;
HWND g_win = nullptr;
HWND g_lblMode = nullptr, g_lblStatus = nullptr;
std::array<HWND, S_COUNT> g_tb{};
std::array<HWND, S_COUNT> g_lblName{};
std::array<HWND, S_COUNT> g_lblVal{};
HWND g_chkMedian = nullptr;
std::array<HWND, METHOD_COUNT> g_chkMethod{};  // Detector on/off boxes, METHOD_* order
HBRUSH g_brGreen = nullptr, g_brRed = nullptr;
bool g_lastFound = false;
HWND g_cboMode = nullptr;                // "Edit:" detector dropdown (Round/Circular/Template/Component)
int g_curMode = MODE_ROUND;              // which mode's settings the sliders currently edit
bool g_modeManual = false;               // operator picked a mode -> stop auto-following until reopen
HFONT g_font = nullptr;                  // host-matching UI font (system dialog font)
HANDLE g_actctx = INVALID_HANDLE_VALUE;  // comctl32 v6 activation context -> themed controls

// The system dialog font -- what native dialogs and the (themed) host use, so our
// controls match instead of the legacy bitmap "System" font.
HFONT create_ui_font() {
    NONCLIENTMETRICSA ncm{};
    ncm.cbSize = sizeof ncm;
    if (SystemParametersInfoA(SPI_GETNONCLIENTMETRICS, sizeof ncm, &ncm, 0) != 0) {
        return CreateFontIndirectA(&ncm.lfMessageFont);
    }
    return nullptr;
}

BOOL CALLBACK set_font_cb(HWND child, LPARAM f) {  // __stdcall (CALLBACK) for x86 safety
    SendMessageA(child, WM_SETFONT, reinterpret_cast<WPARAM>(reinterpret_cast<HFONT>(f)), TRUE);
    return TRUE;
}

// Build a comctl32 v6 activation context from a throwaway manifest so the trackbars
// and buttons render with visual styles (themed), like the host. The host enables
// visual styles only on its own UI thread, so our separate thread needs its own
// context; we activate it around control creation. Returns INVALID_HANDLE_VALUE on
// failure (controls then fall back to the classic look -- no crash).
HANDLE make_v6_context() {
    std::array<char, MAX_PATH> dir{};
    if (GetTempPathA(MAX_PATH, dir.data()) == 0) {
        return INVALID_HANDLE_VALUE;
    }
    std::array<char, MAX_PATH> path{};
    if (GetTempFileNameA(dir.data(), "mvm", 0, path.data()) == 0) {
        return INVALID_HANDLE_VALUE;
    }
    static constexpr std::string_view kManifest =
        "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\r\n"
        "<assembly xmlns=\"urn:schemas-microsoft-com:asm.v1\" manifestVersion=\"1.0\">\r\n"
        "<dependency><dependentAssembly><assemblyIdentity type=\"win32\" "
        "name=\"Microsoft.Windows.Common-Controls\" version=\"6.0.0.0\" "
        "processorArchitecture=\"*\" publicKeyToken=\"6595b64144ccf1df\" language=\"*\"/>"
        "</dependentAssembly></dependency></assembly>\r\n";
    {
        std::ofstream f{path.data(), std::ios::binary};
        if (!f) {
            return INVALID_HANDLE_VALUE;
        }
        f.write(kManifest.data(), static_cast<std::streamsize>(kManifest.size()));
    }
    ACTCTXA actx{};
    actx.cbSize = sizeof actx;
    actx.lpSource = path.data();
    HANDLE h = CreateActCtxA(&actx);
    DeleteFileA(path.data());  // CreateActCtx has parsed it; the context lives independently
    return h;
}

HWND make_static(HWND parent, const char* text, int x, int y, int w, int h) {
    return CreateWindowExA(0, "STATIC", text, WS_CHILD | WS_VISIBLE, x, y, w, h, parent,
                           nullptr, g_inst, nullptr);
}

HWND make_button(HWND parent, int id, const char* text, int x, int y, int w, int h) {
    return CreateWindowExA(0, "BUTTON", text, WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                           x, y, w, h, parent,
                           reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)), g_inst, nullptr);
}

// The Sensitivity slider maps to minSymmetry on a QUADRATIC curve, not linearly:
// the useful decision band is tiny and low (real fiducials score ~2.6-4.6,
// distractors ~1.2-1.7), so a linear 0-10 crammed all the tuning into the lower
// half with 0.1 steps. The square curve gives the low end ~71% of the travel and
// ~0.05 resolution while still reaching 10. pos 0 = Auto (detector default 3.5).
constexpr int kSensTicks = 200;
constexpr double kSensMax = 10.0;

double sens_pos_to_val(int pos) {
    if (pos <= 0) {
        return 0.0;
    }
    const double f = static_cast<double>(pos) / kSensTicks;
    return kSensMax * f * f;
}

int sens_val_to_pos(double val) {
    if (val <= 0.0) {
        return 0;
    }
    int p = static_cast<int>(std::lround(std::sqrt(val / kSensMax) * kSensTicks));
    p = std::max(p, 1);
    p = std::min(p, kSensTicks);
    return p;
}

// Down-camera px->mm scale from the controller (0 if not yet available). Mean of the
// anisotropic X/Y, for length/radius display; the live readout uses per-axis for offset.
double down_scale_mean() {
    const CamScale sc = down_cam_scale();
    return sc.valid ? (sc.xMmPerPx + sc.yMmPerPx) * 0.5 : 0.0;
}

std::string format_value(Kind kind, int pos) {
    const double t = static_cast<double>(pos) / 10.0;
    switch (kind) {
        case K_PX:  // radius slider, shown as DIAMETER (2x) to match the fiducial/host convention
            if (pos <= 0) {
                return "Auto";
            }
            if (const double mm = down_scale_mean(); mm > 0.0) {
                return std::format("{} px ({:.2f} mm)", pos * 2, pos * 2 * mm);
            }
            return std::format("{} px", pos * 2);
        case K_THR: return pos <= 0 ? "Auto" : std::format("{:.1f}", t);
        case K_SENS: return pos <= 0 ? "Auto" : std::format("{:.2f}", sens_pos_to_val(pos));
        case K_GAMMA: return pos <= 0 ? "off" : std::format("{:.1f}", t);
        case K_SINT: return pos == 0 ? "0" : std::format("{:+d}", pos);
        case K_INT: return pos <= 0 ? "0" : std::format("{}", pos);
        case K_STENTHS: return pos == 0 ? "0" : std::format("{:+.1f}", t);
        case K_AUTOINT: return pos <= 0 ? "Auto" : std::format("{}", pos);
        case K_BLUR: {
            if (pos <= 0) {
                return "Auto";
            }
            int k = std::clamp(pos, 3, 25);
            if ((k & 1) == 0) {
                ++k;  // shown as the odd kernel the detector will use
            }
            return std::format("{} px", k);
        }
    }
    return {};  // unreachable (all Kind values handled); satisfies non-void return analysis
}

int tb_pos(std::size_t i) {
    return static_cast<int>(SendMessageA(g_tb[i], TBM_GETPOS, 0, 0));
}

void refresh_value_labels() {
    const bool comp = (g_curMode == MODE_COMP);
    for (std::size_t i = 0; i < kDefs.size(); ++i) {
        std::string v;
        if (comp && (i == S_RMIN || i == S_RMAX)) {
            // Component's repurposed guard sliders read as DIRECT px (not the fiducial
            // 2x diameter) with the UP-camera mm scale.
            const int pos = tb_pos(i);
            if (pos <= 0) {
                v = "Auto";
            } else if (const CamScale sc = up_cam_scale(); sc.valid && sc.xMmPerPx > 0.0) {
                v = std::format("{} px ({:.1f} mm)", pos, pos * (sc.xMmPerPx + sc.yMmPerPx) * 0.5);
            } else {
                v = std::format("{} px", pos);
            }
        } else {
            v = format_value(kDefs[i].kind, tb_pos(i));
        }
        SetWindowTextA(g_lblVal[i], v.c_str());
    }
}

// pos>0 (or !=0 for signed) overrides; 0 = Auto/off (detector default).
double map_to_setting(Idx i, int pos) {
    switch (i) {
        case S_RMIN:
        case S_RMAX:
        case S_BLK:
        case S_WHT:
        case S_EXPLO:
        case S_EXPHI:
        case S_BLUR: return pos > 0 ? static_cast<double>(pos) : 0.0;
        case S_BRI: return static_cast<double>(pos);  // signed, 0 = off
        case S_SYM: return sens_pos_to_val(pos);      // quadratic curve, 0 = Auto
        case S_GAMMA:
        case S_CON: return pos > 0 ? static_cast<double>(pos) / 10.0 : 0.0;
        case S_SHP: return pos != 0 ? static_cast<double>(pos) / 10.0 : 0.0;
        default: return 0.0;
    }
}

void apply_from_controls() {
    Settings s;
    s.radiusMinPx = map_to_setting(S_RMIN, tb_pos(S_RMIN));
    s.radiusMaxPx = map_to_setting(S_RMAX, tb_pos(S_RMAX));
    s.minSymmetry = map_to_setting(S_SYM, tb_pos(S_SYM));
    s.gamma = map_to_setting(S_GAMMA, tb_pos(S_GAMMA));
    s.brightness = map_to_setting(S_BRI, tb_pos(S_BRI));
    s.contrast = map_to_setting(S_CON, tb_pos(S_CON));
    s.blackPoint = map_to_setting(S_BLK, tb_pos(S_BLK));
    s.whitePoint = map_to_setting(S_WHT, tb_pos(S_WHT));
    s.sharpen = map_to_setting(S_SHP, tb_pos(S_SHP));
    s.meanLo = map_to_setting(S_EXPLO, tb_pos(S_EXPLO));
    s.meanHi = map_to_setting(S_EXPHI, tb_pos(S_EXPHI));
    s.blur = map_to_setting(S_BLUR, tb_pos(S_BLUR));
    s.medianRings = (SendMessageA(g_chkMedian, BM_GETCHECK, 0, 0) == BST_CHECKED) ? 1.0 : 0.0;
    set_settings(g_curMode, s);
    refresh_value_labels();
}

// Which controls actually affect a given mode (the rest are greyed out).
bool slider_applies(int mode, std::size_t idx) {
    switch (idx) {
        case S_SYM:
            return mode == MODE_ROUND || mode == MODE_COMP;  // symmetry accept: Round + Component
        case S_RMIN:
        case S_RMAX:
            // Fiducial diameter bracket (Round/Circular); repurposed as the stray-guard
            // search radius / max part size for Component.
            return mode == MODE_ROUND || mode == MODE_CIRCULAR || mode == MODE_COMP;
        default:
            return true;  // exposure gate + all image adjustments: every mode
    }
}

// Enable/disable each control (and its name + value labels) for the active mode, so
// settings that have no effect read as greyed out.
void update_enabled(int mode) {
    for (std::size_t i = 0; i < kDefs.size(); ++i) {
        const BOOL en = slider_applies(mode, i) ? TRUE : FALSE;
        EnableWindow(g_tb[i], en);
        EnableWindow(g_lblName[i], en);
        EnableWindow(g_lblVal[i], en);
    }
    EnableWindow(g_chkMedian, mode == MODE_ROUND ? TRUE : FALSE);  // median ring scoring: Round only
}

void controls_from_settings() {
    const Settings s = get_settings(g_curMode);
    const bool comp = (g_curMode == MODE_COMP);
    // Component repurposes the two diameter sliders as stray-guard knobs in DIRECT
    // pixels that span the up-camera frame -- a large LQFP with pins is hundreds of px
    // (e.g. LQFP-176 ~26mm ~585px at ~22.5 px/mm), far past the fiducial 0-120 radius.
    // Set the range BEFORE TBM_SETPOS so the loaded value isn't clamped to the old max.
    SendMessageA(g_tb[S_RMIN], TBM_SETRANGEMAX, TRUE, comp ? 320 : kDefs[S_RMIN].hi);  // search radius px
    SendMessageA(g_tb[S_RMAX], TBM_SETRANGEMAX, TRUE, comp ? 640 : kDefs[S_RMAX].hi);  // max part size px
    std::array<int, S_COUNT> pos{};
    pos[S_RMIN] = static_cast<int>(s.radiusMinPx);
    pos[S_RMAX] = static_cast<int>(s.radiusMaxPx);
    pos[S_SYM] = sens_val_to_pos(s.minSymmetry);
    pos[S_GAMMA] = static_cast<int>(std::lround(s.gamma * 10.0));
    pos[S_BRI] = static_cast<int>(std::lround(s.brightness));
    pos[S_CON] = static_cast<int>(std::lround(s.contrast * 10.0));
    pos[S_BLK] = static_cast<int>(std::lround(s.blackPoint));
    pos[S_WHT] = static_cast<int>(std::lround(s.whitePoint));
    pos[S_SHP] = static_cast<int>(std::lround(s.sharpen * 10.0));
    pos[S_EXPLO] = static_cast<int>(std::lround(s.meanLo));
    pos[S_EXPHI] = static_cast<int>(std::lround(s.meanHi));
    pos[S_BLUR] = static_cast<int>(std::lround(s.blur));
    for (std::size_t i = 0; i < pos.size(); ++i) {
        SendMessageA(g_tb[i], TBM_SETPOS, TRUE, static_cast<LPARAM>(pos[i]));
    }
    SendMessageA(g_chkMedian, BM_SETCHECK, s.medianRings > 0.5 ? BST_CHECKED : BST_UNCHECKED, 0);
    // Component has no fiducial ring: relabel the two repurposed sliders to the
    // stray-guard knobs. Down modes keep the diameter-bracket labels.
    SetWindowTextA(g_lblName[S_RMIN], comp ? "Search radius" : "Diameter min");
    SetWindowTextA(g_lblName[S_RMAX], comp ? "Max part size" : "Diameter max");
    if (g_cboMode != nullptr) {
        SendMessageA(g_cboMode, CB_SETCURSEL, static_cast<WPARAM>(g_curMode - 1), 0);  // sync dropdown
    }
    update_enabled(g_curMode);
    refresh_value_labels();
}

void refresh_status() {
    const LiveStatus st = get_status();
    // Follow the active mark mode: when it changes, load THAT mode's settings into the
    // sliders (each mode has its own tuning). Suspended once the operator picks a mode
    // from the dropdown (e.g. Component, which never auto-activates) until the dialog
    // is reopened.
    if (!g_modeManual && st.mode >= MODE_ROUND && st.mode <= MODE_TEMPLATE && st.mode != g_curMode) {
        g_curMode = st.mode;
        controls_from_settings();
    }
    const char* mode = "(none yet)";
    if (st.mode == 1) {
        mode = "Round (CheckMark)";
    } else if (st.mode == 2) {
        mode = "Circular (CheckMark2)";
    } else if (st.mode == 3) {
        mode = "Template (CheckTemplate)";
    }
    SetWindowTextA(g_lblMode, std::format("Active mode:  {}", mode).c_str());

    std::string status;
    if (st.found) {
        const CamScale sc = down_cam_scale();
        const double dpx = st.radiusPx * 2.0;  // detected diameter (radius x2)
        std::string r;
        std::string o;
        if (sc.valid) {
            const double mean = (sc.xMmPerPx + sc.yMmPerPx) * 0.5;
            r = std::format("dia {:.1f} px ({:.2f} mm)", dpx, dpx * mean);
            o = std::format("off {:.1f}, {:.1f} px ({:.2f}, {:.2f} mm)", st.offXpx, st.offYpx,
                            st.offXpx * sc.xMmPerPx, st.offYpx * sc.yMmPerPx);
        } else {
            r = std::format("dia {:.1f} px", dpx);
            o = std::format("off {:.1f}, {:.1f} px", st.offXpx, st.offYpx);
        }
        status = std::format(" LOCKED    score {:.2f}    {}    {}", st.score, r, o);
    } else {
        status = std::format(" NO LOCK    score {:.2f}", st.score);
    }
    SetWindowTextA(g_lblStatus, status.c_str());

    g_lastFound = st.found;
    InvalidateRect(g_lblStatus, nullptr, TRUE);
}

LRESULT CALLBACK wnd_proc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    // A window procedure must never let a C++ exception unwind into the Win32
    // dispatcher (UB). The handlers use std::format/std::string, which can allocate,
    // so contain everything and fall through to the default handler on failure.
    try {
        switch (msg) {
            case WM_CREATE: {
                INITCOMMONCONTROLSEX icc{};
                icc.dwSize = sizeof icc;
                icc.dwICC = ICC_BAR_CLASSES;
                InitCommonControlsEx(&icc);

                g_lblMode = make_static(hwnd, "Active mode:  (none yet)", 12, 8, 206, 18);
                make_static(hwnd, "Edit:", 224, 9, 30, 16);
                g_cboMode = CreateWindowExA(0, "COMBOBOX", "",
                                            WS_CHILD | WS_VISIBLE | WS_VSCROLL | CBS_DROPDOWNLIST,
                                            256, 6, 152, 220, hwnd,
                                            reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_CBO_MODE)),
                                            g_inst, nullptr);
                for (const char* name : {"Round", "Circular", "ImageTemplate", "Component"}) {
                    SendMessageA(g_cboMode, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(name));
                }
                make_static(hwnd, "- Detection -", 12, 28, 200, 16);
                make_static(hwnd, "- Image adjustments -", 12, 196, 240, 16);

                for (std::size_t i = 0; i < kDefs.size(); ++i) {
                    g_lblName[i] = make_static(hwnd, kDefs[i].label, 12, kDefs[i].y + 2, 90, 18);  // wide enough for "Exposure min"
                    g_tb[i] = CreateWindowExA(0, "msctls_trackbar32", "",
                                              WS_CHILD | WS_VISIBLE | TBS_HORZ | TBS_NOTICKS,
                                              106, kDefs[i].y, 176, 24, hwnd, nullptr, g_inst, nullptr);
                    SendMessageA(g_tb[i], TBM_SETRANGEMIN, FALSE, static_cast<LPARAM>(kDefs[i].lo));
                    SendMessageA(g_tb[i], TBM_SETRANGEMAX, TRUE, static_cast<LPARAM>(kDefs[i].hi));
                    g_lblVal[i] = make_static(hwnd, "Auto", 288, kDefs[i].y + 2, 124, 18);  // wide for "NN px (M.MM mm)"
                }

                g_chkMedian = CreateWindowExA(0, "BUTTON", "Median ring scoring (robust to glare)",
                                              WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
                                              12, 172, 340, 20, hwnd,
                                              reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_CHK_MEDIAN)),
                                              g_inst, nullptr);

                g_lblStatus = make_static(hwnd, " NO LOCK", 12, 394, 396, 42);

                // Detector master switches (GLOBAL, not per-mode): uncheck to fall back
                // to the stock vision for that method. State persists in the INI [enable]
                // section on Save. Applied live -- the dispatch checks method_enabled().
                make_static(hwnd, "- Detectors (uncheck = use stock) -", 12, 442, 320, 16);
                struct ChkDef {
                    int id;
                    int method;
                    const char* label;
                };
                const std::array<ChkDef, METHOD_COUNT> chk = {{
                    {.id = ID_CHK_ROUND, .method = METHOD_ROUND, .label = "Round  (CheckMark)"},
                    {.id = ID_CHK_CIRCULAR, .method = METHOD_CIRCULAR, .label = "Circular  (CheckMark2)"},
                    {.id = ID_CHK_TEMPLATE, .method = METHOD_TEMPLATE, .label = "ImageTemplate  (CheckTemplate)"},
                    {.id = ID_CHK_COMP, .method = METHOD_COMP, .label = "Component up-vision  (CheckComp)"},
                }};
                for (std::size_t i = 0; i < chk.size(); ++i) {
                    g_chkMethod[i] = CreateWindowExA(
                        0, "BUTTON", chk[i].label, WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX, 12,
                        462 + static_cast<int>(i) * 22, 380, 20, hwnd,
                        reinterpret_cast<HMENU>(static_cast<INT_PTR>(chk[i].id)), g_inst, nullptr);
                    SendMessageA(g_chkMethod[i], BM_SETCHECK,
                                 method_enabled(chk[i].method) ? BST_CHECKED : BST_UNCHECKED, 0);
                }

                // Center the bottom button row on the ACTUAL client width (computed, not
                // hard-coded x positions) so it stays centered regardless of width / DPI.
                constexpr int kBtnW = 80;
                constexpr int kBtnH = 26;
                constexpr int kBtnGap = 10;
                constexpr int kBtnY = 558;
                constexpr int kNBtn = 3;
                RECT cr{};
                GetClientRect(hwnd, &cr);
                const int bx0 =
                    static_cast<int>((cr.right - (kNBtn * kBtnW + (kNBtn - 1) * kBtnGap)) / 2);
                make_button(hwnd, ID_BTN_SAVE, "Save", bx0, kBtnY, kBtnW, kBtnH);
                make_button(hwnd, ID_BTN_RESET, "Reset", bx0 + (kBtnW + kBtnGap), kBtnY, kBtnW, kBtnH);
                make_button(hwnd, ID_BTN_CLOSE, "Close", bx0 + 2 * (kBtnW + kBtnGap), kBtnY, kBtnW, kBtnH);

                {  // open showing the currently-active mode's settings
                    const int m0 = get_status().mode;
                    if (m0 >= MODE_ROUND && m0 <= MODE_TEMPLATE) {
                        g_curMode = m0;
                    }
                }
                controls_from_settings();
                if (g_font != nullptr) {  // host-matching font on every control
                    EnumChildWindows(hwnd, set_font_cb, reinterpret_cast<LPARAM>(g_font));
                    SendMessageA(hwnd, WM_SETFONT, reinterpret_cast<WPARAM>(g_font), TRUE);
                }
                SetTimer(hwnd, static_cast<UINT_PTR>(ID_TIMER), 120, nullptr);
                return 0;
            }
            case WM_HSCROLL:
                apply_from_controls();
                return 0;
            case WM_TIMER:
                refresh_status();
                return 0;
            case WM_CTLCOLORSTATIC: {
                HWND ctl = reinterpret_cast<HWND>(lParam);
                if (ctl == g_lblStatus) {
                    HDC hdc = reinterpret_cast<HDC>(wParam);
                    SetTextColor(hdc, RGB(255, 255, 255));
                    SetBkColor(hdc, g_lastFound ? RGB(40, 150, 60) : RGB(170, 50, 50));
                    return reinterpret_cast<LRESULT>(g_lastFound ? g_brGreen : g_brRed);
                }
                break;
            }
            case WM_COMMAND: {
                const int id = LOWORD(wParam);
                if (id == ID_BTN_SAVE) {
                    apply_from_controls();
                    save_settings(g_iniPath.c_str());
                } else if (id == ID_BTN_RESET) {
                    set_settings(g_curMode, Settings{});  // reset only the current mode
                    controls_from_settings();
                } else if (id == ID_BTN_CLOSE) {
                    ShowWindow(hwnd, SW_HIDE);
                } else if (id == ID_CHK_MEDIAN) {
                    apply_from_controls();  // checkbox toggled
                } else if (id >= ID_CHK_ROUND && id <= ID_CHK_COMP) {
                    const int method = id - ID_CHK_ROUND;  // METHOD_* order matches the ID order
                    const bool on = SendMessageA(g_chkMethod[static_cast<std::size_t>(method)],
                                                 BM_GETCHECK, 0, 0) == BST_CHECKED;
                    set_method_enabled(method, on);  // live; persists in the INI on Save
                } else if (id == ID_CBO_MODE && HIWORD(wParam) == CBN_SELCHANGE) {
                    const auto sel = static_cast<int>(SendMessageA(g_cboMode, CB_GETCURSEL, 0, 0));
                    if (sel >= 0) {
                        g_curMode = sel + 1;  // dropdown index 0..3 -> MODE_ROUND..MODE_COMP
                        g_modeManual = true;  // operator chose a mode -> stop auto-following
                        controls_from_settings();
                    }
                }
                return 0;
            }
            case WM_CLOSE:
                ShowWindow(hwnd, SW_HIDE);  // hide, don't destroy -> reopen via the hotkey
                return 0;
            case WM_DESTROY:
                KillTimer(hwnd, static_cast<UINT_PTR>(ID_TIMER));
                g_win = nullptr;
                return 0;
            default:
                break;
        }
    } catch (...) {  // NOLINT(bugprone-empty-catch) -- fall through to the default handler
    }
    return DefWindowProcA(hwnd, msg, wParam, lParam);
}

void create_window() {
    const DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
    const DWORD ex = WS_EX_TOPMOST | WS_EX_TOOLWINDOW;
    RECT wr{0, 0, 420, 600};
    AdjustWindowRectEx(&wr, style, FALSE, ex);
    // Activate the v6 context around creation so this window AND the child controls
    // it makes in WM_CREATE render themed.
    ULONG_PTR cookie = 0;
    const bool act = (g_actctx != INVALID_HANDLE_VALUE) && (ActivateActCtx(g_actctx, &cookie) != 0);
    g_win = CreateWindowExA(ex, kClassName, "MVision Settings  " MVISION_VERSION, style,
                            CW_USEDEFAULT, CW_USEDEFAULT,
                            wr.right - wr.left, wr.bottom - wr.top,
                            nullptr, nullptr, g_inst, nullptr);
    if (act) {
        DeactivateActCtx(0, cookie);
    }
}

void toggle_window() {
    if (g_win == nullptr) {
        create_window();
    }
    if (g_win == nullptr) {
        return;
    }
    if (IsWindowVisible(g_win) != 0) {
        ShowWindow(g_win, SW_HIDE);
    } else {
        g_modeManual = false;  // each open resumes auto-following the active mode
        ShowWindow(g_win, SW_SHOW);
        SetForegroundWindow(g_win);
    }
}

void ui_thread() {
    g_inst = GetModuleHandleA(nullptr);
    g_brGreen = CreateSolidBrush(RGB(40, 150, 60));
    g_brRed = CreateSolidBrush(RGB(170, 50, 50));
    g_actctx = make_v6_context();  // themed controls
    g_font = create_ui_font();     // host-matching font

    WNDCLASSEXA wc{};
    wc.cbSize = sizeof wc;
    wc.lpfnWndProc = wnd_proc;
    wc.hInstance = g_inst;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(static_cast<INT_PTR>(COLOR_BTNFACE + 1));
    wc.lpszClassName = kClassName;
    RegisterClassExA(&wc);

    // Global hotkey -> WM_HOTKEY posted to THIS thread's queue (NULL hwnd).
    RegisterHotKey(nullptr, HOTKEY_ID, MOD_CONTROL | MOD_ALT, static_cast<UINT>('M'));

    MSG msg;
    while (GetMessageA(&msg, nullptr, 0, 0) > 0) {
        if (msg.message == WM_HOTKEY && msg.wParam == HOTKEY_ID) {
            toggle_window();
            continue;
        }
        TranslateMessage(&msg);
        DispatchMessageA(&msg);
    }
}

}  // namespace

void start_settings_ui() {
    static std::once_flag once;
    std::call_once(once, [] {
        // Runs inside QueryFrame's call_once on the HOST thread, so it must not throw
        // across the C ABI: load_settings / std::string can allocate. Contain it.
        try {
            std::array<char, MAX_PATH> cwd{};
            const DWORD n = GetCurrentDirectoryA(MAX_PATH, cwd.data());
            g_iniPath =
                (n > 0 && n < MAX_PATH) ? std::string(cwd.data()) + "\\MVision.ini" : "MVision.ini";
            load_settings(g_iniPath.c_str());  // apply persisted settings before the first detect
            std::thread(ui_thread).detach();
        } catch (...) {  // NOLINT(bugprone-empty-catch) -- host thread; never throw across the C ABI
        }
    });
}

}  // namespace vis
