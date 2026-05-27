// settings_ui.cpp -- the classic Win32 settings dialog (the second part of the
// module that touches the Windows API, alongside preview.cpp). A DLL-owned thread
// registers a global hotkey (Ctrl+Alt+M) and serves a modeless dialog:
//
//   Header     : active-mode label + "Edit:" dropdown to pick which mode the
//                Detection tab targets (Round / Circular / Template / Component).
//   Status     : LOCKED / NO-LOCK banner with score / radius / offset, pinned
//                at the top so it stays visible regardless of tab.
//   Tabs       : Detection -- per-mode knobs (diameter / sensitivity / exposure
//                             / median-ring; Component-only CompThre help line)
//                Image     -- pre-detection adjustments (gamma / brightness /
//                             contrast / black-point / white-point / sharpen / blur)
//   Detectors  : persistent strip BELOW the tab control with the 4 master
//                on/off switches (uncheck = fall back to stock vision for that
//                mode). Global setting; not tab-specific.
//   Footer     : Save / Reset / Close.
//
// The dialog writes vis::Settings (which the detectors read) and persists to
// MVision.ini in the host working directory.
//
// Tab implementation: all controls remain children of the main dialog; on
// TCN_SELCHANGE we ShowWindow each to match the active tab (no reparenting).
//
// Build-only on Windows; not part of the off-target test harness.

#include "settings_ui.h"

#include "capture.h"
#include "controller.h"
#include "settings.h"
#include "vision.h"  // METHOD_* + comp_profile_name/comp_profile_count (latter held for future Profiles tab)

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <commctrl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
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
             SNAP_HOTKEY_ID = 2,  // Ctrl+Alt+U one-shot up-cam snapshot
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
             ID_CBO_MODE = 1108,  // "Edit:" detector dropdown
             ID_TAB = 1109 };     // SysTabControl32

// Tab pages. Each existing control (slider, checkbox, header static, info
// label) is assigned a Tab via kSliderTab[] / kHeaderTab below. On
// TCN_SELCHANGE we ShowWindow each control to match the active tab -- no
// reparenting, controls stay children of the main dialog.
//
// Only two tabs are shown today: per-mode Detection knobs, and the universal
// pre-detection Image adjustments. The global detector master-switches (Round /
// Circular / Template / Component) sit in a persistent strip below the tab
// control -- they're global, not mode-specific, so a tab page for them was
// awkward. The Profiles slot listing (CompThre 0-9 reservation) is hidden until
// any of the 1-9 slots actually carry a profile; today they're all empty so the
// page has no signal value.
enum Tab : int { TAB_DETECTION,
                 TAB_IMAGE,
                 TAB_COUNT };

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
    Tab tab;  // which tab the slider lives on (show/hide on TCN_SELCHANGE)
};

// Layout (client 420 x 460). Sliders placed inside the tab control's content
// area (y starts ~125, 28-px row spacing). Detection-tab sliders y=125..237;
// Image-tab sliders y=125..293.
const std::array<SliderDef, S_COUNT> kDefs = {{
    {.label = "Diameter min", .lo = 0, .hi = 120, .kind = K_PX, .y = 125, .tab = TAB_DETECTION},       // S_RMIN
    {.label = "Diameter max", .lo = 0, .hi = 120, .kind = K_PX, .y = 153, .tab = TAB_DETECTION},       // S_RMAX
    {.label = "Sensitivity", .lo = 0, .hi = 200, .kind = K_SENS, .y = 181, .tab = TAB_DETECTION},      // S_SYM
    {.label = "Gamma", .lo = 0, .hi = 40, .kind = K_GAMMA, .y = 125, .tab = TAB_IMAGE},                // S_GAMMA
    {.label = "Brightness", .lo = -128, .hi = 128, .kind = K_SINT, .y = 153, .tab = TAB_IMAGE},        // S_BRI
    {.label = "Contrast", .lo = 0, .hi = 30, .kind = K_THR, .y = 181, .tab = TAB_IMAGE},               // S_CON
    {.label = "Black point", .lo = 0, .hi = 128, .kind = K_INT, .y = 209, .tab = TAB_IMAGE},           // S_BLK
    {.label = "White point", .lo = 0, .hi = 128, .kind = K_INT, .y = 237, .tab = TAB_IMAGE},           // S_WHT
    {.label = "Sharpen", .lo = -10, .hi = 10, .kind = K_STENTHS, .y = 265, .tab = TAB_IMAGE},          // S_SHP
    {.label = "Exposure min", .lo = 0, .hi = 128, .kind = K_AUTOINT, .y = 209, .tab = TAB_DETECTION},  // S_EXPLO
    {.label = "Exposure max", .lo = 0, .hi = 255, .kind = K_AUTOINT, .y = 237, .tab = TAB_DETECTION},  // S_EXPHI
    {.label = "Blur", .lo = 0, .hi = 25, .kind = K_BLUR, .y = 293, .tab = TAB_IMAGE},                  // S_BLUR
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

// Tab control + per-tab control sets. Each control is born on whatever tab
// it belongs to and is shown/hidden on TCN_SELCHANGE; no reparenting.
HWND g_tabs = nullptr;
int g_curTab = TAB_DETECTION;
HWND g_lblHelpComp = nullptr;  // "CompThre = ..." help on Detection tab when Component is selected

// Modern Segoe UI 10pt -- bigger than the system default (~9pt) for a more
// modern, less-Win2K feel. Explicitly pick Segoe UI (Vista+; universal on any
// Win10/11 ShopPC) with CLEARTYPE_QUALITY for crisp subpixel rendering.
// Falls back to whatever the system gives if the font is somehow missing.
HFONT create_ui_font() {
    LOGFONTA lf{};
    lf.lfHeight = -13;  // ~10pt at 96 dpi (-MulDiv(10, 96, 72) = -13)
    lf.lfWeight = FW_NORMAL;
    lf.lfCharSet = DEFAULT_CHARSET;
    lf.lfOutPrecision = OUT_DEFAULT_PRECIS;
    lf.lfClipPrecision = CLIP_DEFAULT_PRECIS;
    lf.lfQuality = CLEARTYPE_QUALITY;
    lf.lfPitchAndFamily = DEFAULT_PITCH | FF_DONTCARE;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay) -- lfFaceName is a fixed-size char array; strncpy with the array length is the canonical fill.
    std::strncpy(lf.lfFaceName, "Segoe UI", LF_FACESIZE - 1);
    if (HFONT h = CreateFontIndirectA(&lf); h != nullptr) {
        return h;
    }
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

// Pick the right bg brush for a child control based on what's painted behind it:
//   inside the tab control's content area -> white (the tab interior is white)
//   anywhere else                          -> COLOR_BTNFACE (the dialog surface)
// Returned by the WM_CTLCOLOR* handlers so each control erases its rect with
// the matching colour BEFORE painting its content. Without this, a frequently-
// updated label (e.g. the slider value strings, refreshed on every
// WM_HSCROLL) would overdraw old text on top of new -- garbled glyphs in the
// screenshot. The text itself stays SetBkMode(TRANSPARENT) so it doesn't
// re-paint a different-colour text bg over the matching erase.
HBRUSH brush_for_control(HWND ctl) {
    if (g_tabs == nullptr || ctl == nullptr) {
        return reinterpret_cast<HBRUSH>(static_cast<INT_PTR>(COLOR_BTNFACE + 1));
    }
    RECT cr{};
    RECT tr{};
    GetWindowRect(ctl, &cr);
    GetWindowRect(g_tabs, &tr);
    // ~22 px past the tab control's top edge skips the tab-strip bar; the rest
    // of the tab area is the white content surface.
    if (cr.left >= tr.left && cr.right <= tr.right && cr.top >= tr.top + 22 && cr.bottom <= tr.bottom) {
        return static_cast<HBRUSH>(GetStockObject(WHITE_BRUSH));
    }
    return reinterpret_cast<HBRUSH>(static_cast<INT_PTR>(COLOR_BTNFACE + 1));
}

// Show the per-tab control set; hide everything else. Cheap (just SW_HIDE /
// SW_SHOW per control) and runs only on tab-change.
void show_tab(int tab) {
    g_curTab = tab;
    for (std::size_t i = 0; i < kDefs.size(); ++i) {
        const int cmd = (kDefs[i].tab == tab) ? SW_SHOWNA : SW_HIDE;
        ShowWindow(g_tb[i], cmd);
        ShowWindow(g_lblName[i], cmd);
        ShowWindow(g_lblVal[i], cmd);
    }
    ShowWindow(g_chkMedian, (tab == TAB_DETECTION) ? SW_SHOWNA : SW_HIDE);
    if (g_lblHelpComp != nullptr) {
        ShowWindow(g_lblHelpComp,
                   (tab == TAB_DETECTION && g_curMode == MODE_COMP) ? SW_SHOWNA : SW_HIDE);
    }
    // The 4 detector master switches + their header are persistent (global
    // settings strip below the tab control), shown regardless of tab. The
    // Profiles labels are not created today (slot registry is empty until a
    // profile is authored); the hide loops are gone with them.
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
    // The Component-help label on the Detection tab is only meaningful when
    // editing Component settings; keep it visible/hidden in step with both
    // the active tab AND the active mode.
    if (g_lblHelpComp != nullptr) {
        ShowWindow(g_lblHelpComp,
                   (g_curTab == TAB_DETECTION && mode == MODE_COMP) ? SW_SHOWNA : SW_HIDE);
    }
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
                icc.dwICC = ICC_BAR_CLASSES | ICC_TAB_CLASSES;
                InitCommonControlsEx(&icc);

                // ---- Header: active-mode label + "Edit:" dropdown -------------
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

                // ---- Status banner: pinned at the top, always visible ---------
                g_lblStatus = make_static(hwnd, " NO LOCK", 12, 34, 396, 42);

                // ---- Tab control covering the middle band ---------------------
                // Two tabs only: per-mode Detection knobs + universal Image
                // adjustments. Detector master-switches sit in their own strip
                // below the tab control (global, not mode-specific); the
                // Profiles slot listing is omitted today (all CompThre 0-9
                // slots are empty -- nothing to show).
                g_tabs = CreateWindowExA(0, WC_TABCONTROLA, "",
                                         WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS,
                                         8, 84, 404, 268, hwnd,
                                         reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_TAB)),
                                         g_inst, nullptr);
                {
                    // Tab labels: TCITEMA.pszText is non-const, so use modifiable
                    // char arrays instead of a string-literal table + const_cast.
                    std::array<std::array<char, 16>, TAB_COUNT> tabLabels{};
                    static constexpr std::array<const char*, TAB_COUNT> kTabLabels = {
                        "Detection", "Image"};
                    for (int i = 0; i < TAB_COUNT; ++i) {
                        std::strncpy(tabLabels[i].data(), kTabLabels[i], tabLabels[i].size() - 1);
                        TCITEMA ti{};
                        ti.mask = TCIF_TEXT;
                        ti.pszText = tabLabels[i].data();
                        SendMessageA(g_tabs, TCM_INSERTITEMA, static_cast<WPARAM>(i),
                                     reinterpret_cast<LPARAM>(&ti));
                    }
                }

                // ---- Sliders + their name/value labels ------------------------
                // y values come from kDefs (per-tab). Tab assignment too -- the
                // initial show_tab() at the bottom hides the off-tab ones.
                // Slider rows: name (left, 100 wide) | trackbar (160 wide) |
                // value (right, 108 wide). Sized for 10pt Segoe UI so "Exposure
                // min" labels + "NN px (M.MM mm)" value strings don't truncate.
                for (std::size_t i = 0; i < kDefs.size(); ++i) {
                    g_lblName[i] = make_static(hwnd, kDefs[i].label, 24, kDefs[i].y + 4, 100, 20);
                    g_tb[i] = CreateWindowExA(0, "msctls_trackbar32", "",
                                              WS_CHILD | WS_VISIBLE | TBS_HORZ | TBS_NOTICKS,
                                              130, kDefs[i].y, 160, 24, hwnd, nullptr, g_inst, nullptr);
                    SendMessageA(g_tb[i], TBM_SETRANGEMIN, FALSE, static_cast<LPARAM>(kDefs[i].lo));
                    SendMessageA(g_tb[i], TBM_SETRANGEMAX, TRUE, static_cast<LPARAM>(kDefs[i].hi));
                    g_lblVal[i] = make_static(hwnd, "Auto", 296, kDefs[i].y + 4, 108, 20);
                }

                // ---- Detection tab: median checkbox + Component help line -----
                g_chkMedian = CreateWindowExA(0, "BUTTON", "Median ring scoring (robust to glare)",
                                              WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
                                              24, 268, 360, 22, hwnd,
                                              reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_CHK_MEDIAN)),
                                              g_inst, nullptr);
                g_lblHelpComp =
                    make_static(hwnd,
                                "CompThre 10-100 = manual % of ROI max brightness.\n"
                                "(0-9 reserved for future per-board profile slots.)",
                                24, 300, 380, 40);

                // ---- Detector master switches: persistent strip below the tabs.
                // Global setting (uncheck = fall back to stock vision for that
                // mode); shown regardless of which tab is selected.
                make_static(hwnd, "Detectors  (uncheck = use stock vision):",
                            12, 360, 320, 18);
                struct ChkDef {
                    int id;
                    int method;
                    const char* label;
                };
                const std::array<ChkDef, METHOD_COUNT> chk = {{
                    {.id = ID_CHK_ROUND, .method = METHOD_ROUND, .label = "Round"},
                    {.id = ID_CHK_CIRCULAR, .method = METHOD_CIRCULAR, .label = "Circular"},
                    {.id = ID_CHK_TEMPLATE, .method = METHOD_TEMPLATE, .label = "Template"},
                    {.id = ID_CHK_COMP, .method = METHOD_COMP, .label = "Component"},
                }};
                constexpr int kChkY = 382;
                constexpr int kChkW = 96;
                constexpr int kChkH = 22;
                for (std::size_t i = 0; i < chk.size(); ++i) {
                    const int x = 12 + static_cast<int>(i) * (kChkW + 8);
                    g_chkMethod[i] = CreateWindowExA(
                        0, "BUTTON", chk[i].label, WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX, x,
                        kChkY, kChkW, kChkH, hwnd,
                        reinterpret_cast<HMENU>(static_cast<INT_PTR>(chk[i].id)), g_inst, nullptr);
                    SendMessageA(g_chkMethod[i], BM_SETCHECK,
                                 method_enabled(chk[i].method) ? BST_CHECKED : BST_UNCHECKED, 0);
                }

                // ---- Footer buttons: centered on the actual client width ------
                constexpr int kBtnW = 80;
                constexpr int kBtnH = 26;
                constexpr int kBtnGap = 10;
                constexpr int kBtnY = 416;
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
                show_tab(TAB_DETECTION);  // initial tab visibility
                SetTimer(hwnd, static_cast<UINT_PTR>(ID_TIMER), 120, nullptr);
                return 0;
            }
            case WM_HSCROLL:
                apply_from_controls();
                return 0;
            case WM_TIMER:
                refresh_status();
                return 0;
            case WM_NOTIFY: {
                const auto* nm = reinterpret_cast<NMHDR*>(lParam);
                if (nm != nullptr && nm->hwndFrom == g_tabs && nm->code == TCN_SELCHANGE) {
                    const int t = static_cast<int>(SendMessageA(g_tabs, TCM_GETCURSEL, 0, 0));
                    if (t >= 0 && t < TAB_COUNT) {
                        show_tab(t);
                    }
                }
                return 0;
            }
            case WM_CTLCOLORSTATIC: {
                HWND ctl = reinterpret_cast<HWND>(lParam);
                // Status banner keeps its own coloured bg (green = locked, red = no lock).
                if (ctl == g_lblStatus) {
                    HDC hdc = reinterpret_cast<HDC>(wParam);
                    SetTextColor(hdc, RGB(255, 255, 255));
                    SetBkColor(hdc, g_lastFound ? RGB(40, 150, 60) : RGB(170, 50, 50));
                    return reinterpret_cast<LRESULT>(g_lastFound ? g_brGreen : g_brRed);
                }
                HDC hdc = reinterpret_cast<HDC>(wParam);
                SetBkMode(hdc, TRANSPARENT);
                return reinterpret_cast<LRESULT>(brush_for_control(ctl));
            }
            // BUTTON-class controls (median + 4 detector master switches) and
            // trackbars share the same routing: match the bg behind them so
            // the control's text/box overdraw doesn't leave artefacts.
            case WM_CTLCOLORBTN: {
                HDC hdc = reinterpret_cast<HDC>(wParam);
                SetBkMode(hdc, TRANSPARENT);
                return reinterpret_cast<LRESULT>(brush_for_control(reinterpret_cast<HWND>(lParam)));
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
    RECT wr{0, 0, 420, 460};
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
    // Standard themed dialog background (COLOR_BTNFACE = host's panel grey).
    // The tab control's content area paints its own bg; the static labels and
    // checkboxes are made transparent in WM_CTLCOLORSTATIC / WM_CTLCOLORBTN so
    // they pick up whichever bg is behind them (panel grey OR tab interior),
    // instead of each control adding its own opaque rectangle.
    wc.hbrBackground = reinterpret_cast<HBRUSH>(static_cast<INT_PTR>(COLOR_BTNFACE + 1));
    wc.lpszClassName = kClassName;
    RegisterClassExA(&wc);

    // Global hotkeys -> WM_HOTKEY posted to THIS thread's queue (NULL hwnd).
    // Ctrl+Alt+M toggles the settings dialog; Ctrl+Alt+U arms a one-shot
    // up-cam snapshot consumed by the next CheckComp (raw frame + overlay).
    RegisterHotKey(nullptr, HOTKEY_ID, MOD_CONTROL | MOD_ALT, static_cast<UINT>('M'));
    RegisterHotKey(nullptr, SNAP_HOTKEY_ID, MOD_CONTROL | MOD_ALT, static_cast<UINT>('U'));

    MSG msg;
    while (GetMessageA(&msg, nullptr, 0, 0) > 0) {
        if (msg.message == WM_HOTKEY && msg.wParam == HOTKEY_ID) {
            toggle_window();
            continue;
        }
        if (msg.message == WM_HOTKEY && msg.wParam == SNAP_HOTKEY_ID) {
            cap::arm_snap();
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
