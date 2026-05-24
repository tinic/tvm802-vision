// settings_ui.cpp -- the classic Win32 settings dialog (the second part of the
// module that touches the Windows API, alongside preview.cpp). A DLL-owned thread
// registers a global hotkey (Ctrl+Alt+M) and serves a modeless dialog with
// trackbars for the detection knobs (fiducial radius bracket, Round accept
// threshold) and a grayscale image-adjustment set (gamma, brightness, contrast,
// black/white levels, sharpen), plus a LIVE readout (active mode, LOCK/NO-LOCK,
// score, radius, offset) fed by the detector. The dialog writes vis::Settings (which
// the detectors read) and persists to a text file.
//
// Build-only on Windows; not part of the off-target test harness.

#include "settings_ui.h"

#include "settings.h"

#define NOMINMAX
#include <windows.h>
#include <commctrl.h>

#include <cmath>
#include <cstdio>
#include <mutex>
#include <thread>

namespace vis {
namespace {

const char* const kClassName = "MVisionSettingsWnd";
const char* const kIniPath = "C:\\mvision_capture\\MVision_settings.txt";

enum : int { ID_TIMER = 1,
             HOTKEY_ID = 1,
             ID_BTN_SAVE = 1100,
             ID_BTN_RESET = 1101,
             ID_BTN_CLOSE = 1102 };

// One slider per tunable. `kind` drives both the value-label format and the
// Settings <-> trackbar mapping.
enum Kind { K_PX,
            K_THR,
            K_GAMMA,
            K_SINT,
            K_INT,
            K_STENTHS };
enum Idx { S_RMIN,
           S_RMAX,
           S_SYM,
           S_GAMMA,
           S_BRI,
           S_CON,
           S_BLK,
           S_WHT,
           S_SHP,
           S_COUNT };

struct SliderDef {
    const char* label;
    int lo, hi;
    Kind kind;
    int y;
};

// Layout (client 364 x 396). Group headers are drawn as separate statics.
const SliderDef kDefs[S_COUNT] = {
    {"Radius min", 0, 120, K_PX, 48},        // S_RMIN
    {"Radius max", 0, 120, K_PX, 74},        // S_RMAX
    {"Sensitivity", 0, 100, K_THR, 100},     // S_SYM  (accept threshold x10)
    {"Gamma", 0, 40, K_GAMMA, 146},          // S_GAMMA (x10)
    {"Brightness", -128, 128, K_SINT, 172},  // S_BRI
    {"Contrast", 0, 30, K_THR, 198},         // S_CON  (x10)
    {"Black point", 0, 128, K_INT, 224},     // S_BLK
    {"White point", 0, 128, K_INT, 250},     // S_WHT
    {"Sharpen", -10, 10, K_STENTHS, 276},    // S_SHP  (x10)
};

HINSTANCE g_inst = nullptr;
HWND g_win = nullptr;
HWND g_lblMode = nullptr, g_lblStatus = nullptr;
HWND g_tb[S_COUNT] = {};
HWND g_lblVal[S_COUNT] = {};
HBRUSH g_brGreen = nullptr, g_brRed = nullptr;
bool g_lastFound = false;
HFONT g_font = nullptr;                  // host-matching UI font (system dialog font)
HANDLE g_actctx = INVALID_HANDLE_VALUE;  // comctl32 v6 activation context -> themed controls

// The system dialog font -- what native dialogs and the (themed) host use, so our
// controls match instead of the legacy bitmap "System" font.
HFONT create_ui_font() {
    NONCLIENTMETRICSA ncm{};
    ncm.cbSize = sizeof ncm;
    if (SystemParametersInfoA(SPI_GETNONCLIENTMETRICS, sizeof ncm, &ncm, 0))
        return CreateFontIndirectA(&ncm.lfMessageFont);
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
    char dir[MAX_PATH] = {};
    if (GetTempPathA(MAX_PATH, dir) == 0) return INVALID_HANDLE_VALUE;
    char path[MAX_PATH] = {};
    if (GetTempFileNameA(dir, "mvm", 0, path) == 0) return INVALID_HANDLE_VALUE;
    FILE* f = std::fopen(path, "wb");
    if (!f) return INVALID_HANDLE_VALUE;
    static const char kManifest[] =
        "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\r\n"
        "<assembly xmlns=\"urn:schemas-microsoft-com:asm.v1\" manifestVersion=\"1.0\">\r\n"
        "<dependency><dependentAssembly><assemblyIdentity type=\"win32\" "
        "name=\"Microsoft.Windows.Common-Controls\" version=\"6.0.0.0\" "
        "processorArchitecture=\"*\" publicKeyToken=\"6595b64144ccf1df\" language=\"*\"/>"
        "</dependentAssembly></dependency></assembly>\r\n";
    std::fwrite(kManifest, 1, sizeof(kManifest) - 1, f);
    std::fclose(f);
    ACTCTXA actx{};
    actx.cbSize = sizeof actx;
    actx.lpSource = path;
    HANDLE h = CreateActCtxA(&actx);
    DeleteFileA(path);  // CreateActCtx has parsed it; the context lives independently
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

void format_value(Kind kind, int pos, char* buf, size_t n) {
    const double t = static_cast<double>(pos) / 10.0;
    switch (kind) {
        case K_PX: pos <= 0 ? std::snprintf(buf, n, "Auto") : std::snprintf(buf, n, "%d px", pos); break;
        case K_THR: pos <= 0 ? std::snprintf(buf, n, "Auto") : std::snprintf(buf, n, "%.1f", t); break;
        case K_GAMMA: pos <= 0 ? std::snprintf(buf, n, "off") : std::snprintf(buf, n, "%.1f", t); break;
        case K_SINT: pos == 0 ? std::snprintf(buf, n, "0") : std::snprintf(buf, n, "%+d", pos); break;
        case K_INT: pos <= 0 ? std::snprintf(buf, n, "0") : std::snprintf(buf, n, "%d", pos); break;
        case K_STENTHS: pos == 0 ? std::snprintf(buf, n, "0") : std::snprintf(buf, n, "%+.1f", t); break;
    }
}

int tb_pos(int i) {
    return static_cast<int>(SendMessageA(g_tb[i], TBM_GETPOS, 0, 0));
}

void refresh_value_labels() {
    for (int i = 0; i < S_COUNT; ++i) {
        char buf[32];
        format_value(kDefs[i].kind, tb_pos(i), buf, sizeof buf);
        SetWindowTextA(g_lblVal[i], buf);
    }
}

// pos>0 (or !=0 for signed) overrides; 0 = Auto/off (detector default).
double map_to_setting(Idx i, int pos) {
    switch (i) {
        case S_RMIN:
        case S_RMAX:
        case S_BLK:
        case S_WHT: return pos > 0 ? static_cast<double>(pos) : 0.0;
        case S_BRI: return static_cast<double>(pos);  // signed, 0 = off
        case S_SYM:
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
    set_settings(s);
    refresh_value_labels();
}

void controls_from_settings() {
    const Settings s = get_settings();
    int pos[S_COUNT];
    pos[S_RMIN] = static_cast<int>(s.radiusMinPx);
    pos[S_RMAX] = static_cast<int>(s.radiusMaxPx);
    pos[S_SYM] = static_cast<int>(std::lround(s.minSymmetry * 10.0));
    pos[S_GAMMA] = static_cast<int>(std::lround(s.gamma * 10.0));
    pos[S_BRI] = static_cast<int>(std::lround(s.brightness));
    pos[S_CON] = static_cast<int>(std::lround(s.contrast * 10.0));
    pos[S_BLK] = static_cast<int>(std::lround(s.blackPoint));
    pos[S_WHT] = static_cast<int>(std::lround(s.whitePoint));
    pos[S_SHP] = static_cast<int>(std::lround(s.sharpen * 10.0));
    for (int i = 0; i < S_COUNT; ++i)
        SendMessageA(g_tb[i], TBM_SETPOS, TRUE, static_cast<LPARAM>(pos[i]));
    refresh_value_labels();
}

void refresh_status() {
    const LiveStatus st = get_status();
    const char* mode = st.mode == 1   ? "Round (CheckMark)"
                       : st.mode == 2 ? "Circular (CheckMark2)"
                       : st.mode == 3 ? "Template (CheckTemplate)"
                                      : "(none yet)";
    char m[96];
    std::snprintf(m, sizeof m, "Active mode:  %s", mode);
    SetWindowTextA(g_lblMode, m);

    char buf[160];
    if (st.found)
        std::snprintf(buf, sizeof buf, " LOCKED    score %.2f    r %.1f px    off %.2f, %.2f mm",
                      st.score, st.radiusPx, st.offXmm, st.offYmm);
    else
        std::snprintf(buf, sizeof buf, " NO LOCK    score %.2f", st.score);
    SetWindowTextA(g_lblStatus, buf);

    g_lastFound = st.found;
    InvalidateRect(g_lblStatus, nullptr, TRUE);
}

LRESULT CALLBACK wnd_proc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
        case WM_CREATE: {
            INITCOMMONCONTROLSEX icc{};
            icc.dwSize = sizeof icc;
            icc.dwICC = ICC_BAR_CLASSES;
            InitCommonControlsEx(&icc);

            g_lblMode = make_static(hwnd, "Active mode:  (none yet)", 12, 8, 340, 18);
            make_static(hwnd, "- Detection -", 12, 30, 200, 16);
            make_static(hwnd, "- Image adjustments -", 12, 128, 240, 16);

            for (int i = 0; i < S_COUNT; ++i) {
                make_static(hwnd, kDefs[i].label, 12, kDefs[i].y + 2, 70, 18);
                g_tb[i] = CreateWindowExA(0, "msctls_trackbar32", "",
                                          WS_CHILD | WS_VISIBLE | TBS_HORZ | TBS_NOTICKS,
                                          86, kDefs[i].y, 200, 24, hwnd, nullptr, g_inst, nullptr);
                SendMessageA(g_tb[i], TBM_SETRANGEMIN, FALSE, static_cast<LPARAM>(kDefs[i].lo));
                SendMessageA(g_tb[i], TBM_SETRANGEMAX, TRUE, static_cast<LPARAM>(kDefs[i].hi));
                g_lblVal[i] = make_static(hwnd, "Auto", 290, kDefs[i].y + 2, 66, 18);
            }

            g_lblStatus = make_static(hwnd, " NO LOCK", 12, 308, 340, 42);
            make_button(hwnd, ID_BTN_SAVE, "Save", 60, 360, 80, 26);
            make_button(hwnd, ID_BTN_RESET, "Reset", 150, 360, 80, 26);
            make_button(hwnd, ID_BTN_CLOSE, "Close", 240, 360, 80, 26);

            controls_from_settings();
            if (g_font) {  // host-matching font on every control
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
                save_settings(kIniPath);
            } else if (id == ID_BTN_RESET) {
                Settings def;  // all Auto / neutral
                set_settings(def);
                controls_from_settings();
            } else if (id == ID_BTN_CLOSE) {
                ShowWindow(hwnd, SW_HIDE);
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
    return DefWindowProcA(hwnd, msg, wParam, lParam);
}

void create_window() {
    const DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
    const DWORD ex = WS_EX_TOPMOST | WS_EX_TOOLWINDOW;
    RECT wr{0, 0, 364, 396};
    AdjustWindowRectEx(&wr, style, FALSE, ex);
    // Activate the v6 context around creation so this window AND the child controls
    // it makes in WM_CREATE render themed.
    ULONG_PTR cookie = 0;
    const bool act = (g_actctx != INVALID_HANDLE_VALUE) && ActivateActCtx(g_actctx, &cookie);
    g_win = CreateWindowExA(ex, kClassName, "MVision Settings", style,
                            CW_USEDEFAULT, CW_USEDEFAULT,
                            wr.right - wr.left, wr.bottom - wr.top,
                            nullptr, nullptr, g_inst, nullptr);
    if (act) DeactivateActCtx(0, cookie);
}

void toggle_window() {
    if (!g_win) create_window();
    if (!g_win) return;
    if (IsWindowVisible(g_win)) {
        ShowWindow(g_win, SW_HIDE);
    } else {
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
        load_settings(kIniPath);  // apply persisted settings before the first detect
        std::thread(ui_thread).detach();
    });
}

}  // namespace vis
