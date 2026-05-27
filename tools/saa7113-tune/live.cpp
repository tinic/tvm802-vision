// Live-preview mode for saa7113-tune: opens a Win32 window streaming UYVY
// frames over the STK1150's isoc IN endpoint while a stdin REPL accepts
// register-tweak commands on the same WinUSB session. No driver swap
// between tweaks.
//
// Pipeline (per Linux drivers/media/usb/stk1160/, GPL-2.0):
//
//   1. Reset STK1160 housekeeping registers (stk1160_reg_reset sequence).
//   2. SAA7113 cold init via I2C (saa7113_init sequence from saa7115.c).
//   3. Set capture window for NTSC 720x480 (CFSPO/CFEPO).
//   4. Pick the alt setting whose isoc IN endpoint 0x82 has the largest
//      wMaxPacketSize >= STK1160_MIN_PKT_SIZE (3072).
//   5. Kick streaming: STK1160_DCTRL <- 0xb3, DCTRL+3 <- 0x00.
//   6. Submit STK1160_NUM_BUFS=16 isoc URBs, each STK1160_NUM_PACKETS=64
//      packets; reassemble frames in the completion callback.
//   7. On each completed frame, copy UYVY into a window-side buffer; the
//      WM_PAINT handler converts UYVY -> BGR and StretchDIBits to the HDC.
//
// Frame parser (verbatim from stk1160-video.c):
//   p[0] == 0xc0  -> end-of-frame; finish current buf, fetch next.
//   p[0] == 0x80  -> field marker; buf->odd = p[0] & 0x40; reset pos.
//   else          -> raster data after a 4-byte header; copy interleaved.
//
// Threading model:
//   - Main thread: Win32 message loop + GDI repaint.
//   - libusb thread: spins libusb_handle_events; isoc callbacks fire here.
//   - REPL thread: blocks on std::getline; saa_write/saa_read from here.
//   The frame buffer is double-buffered: the libusb thread fills the
//   "back" buffer, swaps under a mutex when a frame completes, and the
//   main thread copies from the "front" buffer at paint time.
//   libusb_control_transfer is thread-safe.

#include "live.h"
#include "common.h"

#include <libusb.h>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <mutex>
#include <print>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace saa {

namespace {

// ---- Capture geometry ---------------------------------------------------
// NTSC 720x480 UYVY interlaced. The chip emits BOTH 0xc0 (odd field +
// frame start) and 0x80 (even field) markers; we accumulate two field
// halves into a 720x480 interleaved buffer. (An earlier build of this
// tool didn't classify 0x80 correctly and we incorrectly concluded the
// chip emitted only one field -- false alarm.)
constexpr int kFrameW = 720;
constexpr int kFrameH = 480;
constexpr int kBytesPerLine = kFrameW * 2;
constexpr int kFrameBytes = kBytesPerLine * kFrameH;

// ---- USB streaming knobs (from stk1160.h) ------------------------------
constexpr int kNumBufs = 8;       // 16 is overkill for preview; cut latency
constexpr int kNumPackets = 64;
constexpr int kMaxPktSize = 3072;

// ---- STK1160 register reset sequence (datasheet + Linux stk1160_reg_reset) -
struct RegVal { std::uint16_t reg; std::uint8_t val; };
constexpr std::array kStkResetSeq = std::to_array<RegVal>({
    {0x002, 0x78},  // GCTRL+2
    {0x00d, 0x00},  // RMCTL+1
    {0x00f, 0x02},  // RMCTL+3
    {0x018, 0x10},  // PLLSO
    {0x019, 0x00},  // PLLSO+1
    {0x01a, 0x14},  // PLLSO+2
    {0x01b, 0x0e},  // PLLSO+3
    {0x01c, 0x46},  // PLLFD
    {0x300, 0x12},  // TIGEN
    {0x350, 0x2d},  // TICTL
    {0x351, 0x01},  // TICTL+1
    {0x352, 0x00},  // TICTL+2
    {0x353, 0x00},  // TICTL+3
    {0x300, 0x80},  // TIGEN -- enable
});

// NTSC capture window: CFSPO=(0,3) CFEPO=(0x5a0,0xf3). Datasheet section
// 8.5 + Linux stk1160_set_std (525-line branch).
constexpr std::array kStkCaptureNtsc = std::to_array<RegVal>({
    {0x110, 0x00},  // CFSPO_STX_L
    {0x111, 0x00},  // CFSPO_STX_H
    {0x112, 0x03},  // CFSPO_STY_L
    {0x113, 0x00},  // CFSPO_STY_H
    {0x114, 0xa0},  // CFEPO_ENX_L
    {0x115, 0x05},  // CFEPO_ENX_H
    {0x116, 0xf3},  // CFEPO_ENY_L
    {0x117, 0x00},  // CFEPO_ENY_H
});

// SAA7113 cold init (saa7115.c saa7113_init[]). Configures NTSC, CVBS on
// AI21, sane brightness/contrast/saturation defaults.
constexpr std::array kSaa7113Init = std::to_array<RegVal>({
    {0x01, 0x08}, {0x02, 0xc2}, {0x03, 0x30}, {0x04, 0x00}, {0x05, 0x00},
    {0x06, 0x89}, {0x07, 0x0d}, {0x08, 0x88}, {0x09, 0x01}, {0x0a, 0x80},
    {0x0b, 0x47}, {0x0c, 0x40}, {0x0d, 0x00}, {0x0e, 0x01}, {0x0f, 0x2a},
    {0x10, 0x08}, {0x11, 0x0c}, {0x12, 0x07}, {0x13, 0x00}, {0x14, 0x00},
    {0x15, 0x00}, {0x16, 0x00}, {0x17, 0x00},
});

// ---- Shared frame state -------------------------------------------------

struct Frame {
    std::vector<std::uint8_t> uyvy;   // kFrameBytes
    bool odd = false;                  // current field parity
    int pos = 0;                       // bytes written into this field
    int bytesused = 0;
};

struct State {
    libusb_device_handle* h = nullptr;
    std::uint8_t addr = kSaaDefaultAddr;
    std::atomic<bool> running{true};
    bool debug = false;

    // Double-buffered frame: ISOC fills `back`, swaps under mutex on
    // end-of-frame, WM_PAINT reads `front`.
    std::mutex frameMu;
    Frame back;
    Frame front;
    std::atomic<bool> frontDirty{false};

    HWND hwnd = nullptr;

    // Diagnostic counters (atomic so the libusb thread can update freely);
    // only printed when `debug == true`. Inert otherwise.
    std::atomic<std::uint64_t> pktTotal{0};
    std::atomic<std::uint64_t> pktEmpty{0};   // actual_length == 0
    std::atomic<std::uint64_t> pktShort{0};   // 0 < actual_length <= 4
    std::atomic<std::uint64_t> pktData{0};    // non-header packets with data
    std::atomic<std::uint64_t> pktSofC0{0};   // 0xc0 start-of-frame markers
    std::atomic<std::uint64_t> pktSof80{0};   // 0x80 field markers
    std::atomic<std::uint64_t> framesDone{0};
    std::atomic<std::uint64_t> maxLen{0};
};

// ---- Isoc callback ------------------------------------------------------

void process_packet(State& s, const std::uint8_t* p, int len) {
    s.pktTotal.fetch_add(1, std::memory_order_relaxed);
    auto mx = s.maxLen.load(std::memory_order_relaxed);
    while (std::uint64_t(len) > mx &&
           !s.maxLen.compare_exchange_weak(mx, std::uint64_t(len))) {}
    if (len == 0) { s.pktEmpty.fetch_add(1, std::memory_order_relaxed); return; }
    if (len <= 4) { s.pktShort.fetch_add(1, std::memory_order_relaxed); return; }
    if (p[0] == 0xc0) {
        s.pktSofC0.fetch_add(1, std::memory_order_relaxed);
        // End-of-frame: swap back -> front under lock, signal repaint via
        // PostMessage (documented way to nudge the GUI thread from a worker).
        {
            std::lock_guard lk(s.frameMu);
            std::swap(s.back, s.front);
            s.back.pos = 0;
            s.back.bytesused = 0;
        }
        s.framesDone.fetch_add(1, std::memory_order_relaxed);
        s.frontDirty.store(true, std::memory_order_release);
        if (s.hwnd != nullptr) {
            PostMessage(s.hwnd, WM_APP + 1, 0, 0);
        }
    } else if (p[0] == 0x80) {
        s.pktSof80.fetch_add(1, std::memory_order_relaxed);
    } else {
        s.pktData.fetch_add(1, std::memory_order_relaxed);
    }
    if (p[0] == 0xc0 || p[0] == 0x80) {
        s.back.odd = (p[0] & 0x40) != 0;
        s.back.pos = 0;
        return;
    }
    // Continuation packet: skip 4-byte header, copy raster data into back
    // buffer at interleaved field offset.
    const int hdr = 4;
    int remain = len - hdr;
    const std::uint8_t* src = p + hdr;

    // Interleaved storage: odd-field lines at indices 0, 2, 4... and
    // even-field lines at 1, 3, 5... per Linux stk1160-video.c.
    int linesdone = s.back.pos / kBytesPerLine;
    int lineoff = s.back.pos % kBytesPerLine;
    int dstLine = linesdone * 2 + (s.back.odd ? 0 : 1);
    int dstOff = dstLine * kBytesPerLine + lineoff;

    while (remain > 0) {
        if (dstOff >= kFrameBytes) return;
        int lencopy = std::min(remain, kBytesPerLine - lineoff);
        if (dstOff + lencopy > kFrameBytes) {
            lencopy = kFrameBytes - dstOff;
        }
        std::memcpy(s.back.uyvy.data() + dstOff, src, std::size_t(lencopy));
        src += lencopy;
        remain -= lencopy;
        s.back.pos += lencopy;
        s.back.bytesused += lencopy;
        lineoff = 0;
        ++linesdone;
        dstLine = linesdone * 2 + (s.back.odd ? 0 : 1);
        dstOff = dstLine * kBytesPerLine;
    }
}

void LIBUSB_CALL on_isoc_complete(libusb_transfer* xfer) {
    auto* s = static_cast<State*>(xfer->user_data);
    if (xfer->status == LIBUSB_TRANSFER_CANCELLED ||
        xfer->status == LIBUSB_TRANSFER_NO_DEVICE) {
        return;
    }
    for (int i = 0; i < xfer->num_iso_packets; ++i) {
        auto& d = xfer->iso_packet_desc[i];
        if (d.status != LIBUSB_TRANSFER_COMPLETED) continue;
        const std::uint8_t* p = libusb_get_iso_packet_buffer_simple(xfer, std::uint32_t(i));
        process_packet(*s, p, int(d.actual_length));
    }
    if (s->running.load(std::memory_order_acquire)) {
        libusb_submit_transfer(xfer);
    }
}

// ---- Alt-setting picker -------------------------------------------------

// Pick the alt setting on interface 0 whose endpoint 0x82 has the largest
// isoc wMaxPacketSize that's at least kMaxPktSize. Returns -1 if none works.
int pick_alt_setting(libusb_device_handle* h, int& outMaxPkt) {
    libusb_device* dev = libusb_get_device(h);
    libusb_config_descriptor* cfg = nullptr;
    if (libusb_get_active_config_descriptor(dev, &cfg) < 0) return -1;
    int bestAlt = -1;
    int bestSize = 0;
    if (cfg->bNumInterfaces > 0) {
        const auto& intf = cfg->interface[0];
        for (int a = 0; a < intf.num_altsetting; ++a) {
            const auto& alt = intf.altsetting[a];
            for (int e = 0; e < alt.bNumEndpoints; ++e) {
                const auto& ep = alt.endpoint[e];
                if (ep.bEndpointAddress != kStkEpVideo) continue;
                int size = ep.wMaxPacketSize & 0x07ff;
                int xtra = ((ep.wMaxPacketSize >> 11) & 0x3) + 1;
                size *= xtra;
                if (size >= kMaxPktSize && size > bestSize) {
                    bestSize = size;
                    bestAlt = alt.bAlternateSetting;
                }
            }
        }
    }
    libusb_free_config_descriptor(cfg);
    outMaxPkt = bestSize;
    return bestAlt;
}

// ---- UYVY -> BGR conversion + repaint ----------------------------------

// UYVY: each 4 bytes = U Y0 V Y1 -> two BGR pixels.
//
// The TVM802B's cameras are monochrome (no chroma signal carried), so the
// chroma calculation is wasted work and the standard BT.601 limited-range
// formula (`B = (298*(Y-16) + ...) >> 8`) over-stretches the image -- it
// expects Y in [16..235] but the SAA7113 here outputs essentially full-
// range Y. The stretch clamps both ends and looks "very contrasty". So:
// take Y straight through (B = G = R = Y), drop the chroma math entirely.
void uyvy_to_bgr(const std::uint8_t* uyvy, std::uint8_t* bgr, int w, int h) {
    for (int y = 0; y < h; ++y) {
        const std::uint8_t* sp = uyvy + y * w * 2;
        std::uint8_t* dp = bgr + y * w * 3;
        for (int x = 0; x < w; x += 2) {
            std::uint8_t y0 = sp[1];
            std::uint8_t y1 = sp[3];
            dp[0] = dp[1] = dp[2] = y0;
            dp[3] = dp[4] = dp[5] = y1;
            sp += 4;
            dp += 6;
        }
    }
}

// ---- Win32 window -------------------------------------------------------

LRESULT CALLBACK wnd_proc(HWND h, UINT msg, WPARAM w, LPARAM l) {
    auto* s = reinterpret_cast<State*>(GetWindowLongPtr(h, GWLP_USERDATA));
    switch (msg) {
        case WM_APP + 1:
            InvalidateRect(h, nullptr, FALSE);
            return 0;
        case WM_PAINT: {
            PAINTSTRUCT ps{};
            HDC hdc = BeginPaint(h, &ps);
            if (s != nullptr && s->frontDirty.exchange(false)) {
                static std::vector<std::uint8_t> bgr(std::size_t(kFrameW * kFrameH * 3));
                {
                    std::lock_guard lk(s->frameMu);
                    uyvy_to_bgr(s->front.uyvy.data(), bgr.data(), kFrameW, kFrameH);
                }
                BITMAPINFO bmi{};
                bmi.bmiHeader.biSize = sizeof(bmi.bmiHeader);
                bmi.bmiHeader.biWidth = kFrameW;
                // negative height = top-down rows (no Y-flip)
                bmi.bmiHeader.biHeight = -kFrameH;
                bmi.bmiHeader.biPlanes = 1;
                bmi.bmiHeader.biBitCount = 24;
                bmi.bmiHeader.biCompression = BI_RGB;
                RECT rc;
                GetClientRect(h, &rc);
                SetStretchBltMode(hdc, HALFTONE);
                StretchDIBits(hdc, 0, 0, rc.right, rc.bottom,
                              0, 0, kFrameW, kFrameH,
                              bgr.data(), &bmi, DIB_RGB_COLORS, SRCCOPY);
            }
            EndPaint(h, &ps);
            return 0;
        }
        case WM_CLOSE:
            if (s != nullptr) s->running.store(false, std::memory_order_release);
            DestroyWindow(h);
            return 0;
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
    }
    return DefWindowProc(h, msg, w, l);
}

HWND create_window(State& s) {
    WNDCLASSA wc{};
    wc.lpfnWndProc = wnd_proc;
    wc.hInstance = GetModuleHandleA(nullptr);
    wc.lpszClassName = "Saa7113Preview";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    RegisterClassA(&wc);
    // Window at native 720x480.
    RECT rc{0, 0, kFrameW, kFrameH};
    AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, FALSE);
    HWND h = CreateWindowA("Saa7113Preview", "saa7113-tune live preview",
                           WS_OVERLAPPEDWINDOW | WS_VISIBLE,
                           CW_USEDEFAULT, CW_USEDEFAULT,
                           rc.right - rc.left, rc.bottom - rc.top,
                           nullptr, nullptr, GetModuleHandleA(nullptr), nullptr);
    SetWindowLongPtr(h, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(&s));
    return h;
}

// ---- REPL ---------------------------------------------------------------

void repl_help() {
    std::println(
        "Commands (one per line):\n"
        "  b NNN  / brightness NNN   reg 0x0A, 0..255\n"
        "  c NNN  / contrast NNN     reg 0x0B\n"
        "  s NNN  / saturation NNN   reg 0x0C\n"
        "  h NNN  / hue NNN          reg 0x0D\n"
        "  g NNN  / gain NNN         9-bit manual gain 0..511 (~-3..+6 dB; sets GAFIX)\n"
        "  agc on|off                GAFIX bit in reg 0x03 (1 = manual gain, 0 = AGC)\n"
        "  hold on|off               HOLDG bit in reg 0x03 (1 = freeze AGC integration)\n"
        "  i 0..3 / input 0..3       AI11/AI12/AI21/AI22 select in reg 0x02\n"
        "  w REG VAL / write REG VAL raw SAA7113 register write (hex or dec)\n"
        "  r REG     / read REG      raw read\n"
        "  d         / dump          all 128 SAA7113 registers\n"
        "  hist                      Y-channel min/max/mean + 16-bin histogram\n"
        "  debug on|off              toggle 1Hz isoc stats line\n"
        "  ?         / help\n"
        "  q         / quit");
}

void repl_thread(State& s) {
    repl_help();
    std::string line;
    while (s.running.load(std::memory_order_acquire) && std::getline(std::cin, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string cmd;
        ss >> cmd;
        auto write_named = [&](std::uint8_t reg, std::string_view name, std::string_view val_s) {
            std::uint8_t v = 0;
            if (parse_u8(val_s, v) < 0) { std::println("  bad value"); return; }
            int rc = saa_write(s.h, s.addr, reg, v);
            if (rc < 0) std::println("  {} write failed: {}", name, libusb_error_name(rc));
            else        std::println("  {} (reg 0x{:02x}) <- 0x{:02x}", name, reg, v);
        };
        std::string a1, a2;
        ss >> a1;
        ss >> a2;

        if (cmd == "q" || cmd == "quit") {
            s.running.store(false, std::memory_order_release);
            if (s.hwnd) PostMessage(s.hwnd, WM_CLOSE, 0, 0);
            return;
        }
        if (cmd == "?" || cmd == "help") { repl_help(); continue; }
        if (cmd == "b" || cmd == "brightness") { write_named(kSaaBrightness, "brightness", a1); continue; }
        if (cmd == "c" || cmd == "contrast")   { write_named(kSaaContrast,   "contrast",   a1); continue; }
        if (cmd == "s" || cmd == "saturation") { write_named(kSaaSaturation, "saturation", a1); continue; }
        if (cmd == "h" || cmd == "hue")        { write_named(kSaaHue,        "hue",        a1); continue; }
        if (cmd == "g" || cmd == "gain") {
            // Manual gain: set GAFIX=1 in reg 0x03 (selects manual gain over
            // AGC), then write the low 8 bits of the 9-bit gain to 0x04/0x05
            // and the MSB (GAI18/GAI28) into reg 0x03 bits 0/1.
            // Per datasheet Table 31 (SA 04): 9-bit decimal 0 ≈ -3dB,
            // 117 ≈ 0dB (unity), 511 ≈ +6dB. Argument is the FULL 9-bit
            // value 0..511. Accept "g NNN" decimal or "g 0xHHH" hex.
            char* end = nullptr;
            unsigned long g9 = std::strtoul(a1.c_str(), &end, 0);
            if (end == a1.c_str() || g9 > 0x1ff) {
                std::println("  bad gain (need 0..511)"); continue;
            }
            std::uint8_t lo = std::uint8_t(g9 & 0xff);
            std::uint8_t msb = std::uint8_t((g9 >> 8) & 0x01);
            std::uint8_t cur = 0;
            saa_read(s.h, s.addr, kSaaInputCntl2, cur);
            std::uint8_t nv = std::uint8_t(
                (cur & ~(kSaaGafix | kSaaGai18 | kSaaGai28))
                | kSaaGafix
                | (msb ? kSaaGai18 : 0)
                | (msb ? kSaaGai28 : 0));
            saa_write(s.h, s.addr, kSaaInputCntl2, nv);
            saa_write(s.h, s.addr, kSaaGainCh1Lo, lo);
            saa_write(s.h, s.addr, kSaaGainCh2Lo, lo);
            std::println("  gain {} -> reg0x03=0x{:02x} GAI1=GAI2=0x{:02x} (GAFIX set)",
                         g9, nv, lo);
            continue;
        }
        if (cmd == "agc") {
            // AGC control is GAFIX (reg 0x03 bit 2): 0 = AGC drives gain,
            // 1 = manual gain in 0x04/0x05 used. (FUSE bits in reg 0x02
            // are analog filter bypass per datasheet Table 29, NOT AGC.)
            std::uint8_t cur = 0;
            saa_read(s.h, s.addr, kSaaInputCntl2, cur);
            std::uint8_t nv = (a1 == "on")
                                  ? std::uint8_t(cur & ~kSaaGafix)
                                  : std::uint8_t(cur |  kSaaGafix);
            saa_write(s.h, s.addr, kSaaInputCntl2, nv);
            std::println("  reg 0x03 <- 0x{:02x}  (GAFIX {} -> AGC {})",
                         nv, (nv & kSaaGafix) ? "1" : "0", a1);
            continue;
        }
        if (cmd == "hold") {
            // HOLDG (reg 0x03 bit 3): 1 = freeze AGC integration at current
            // value, 0 = AGC continues updating. Useful for fixing the
            // working AGC value without switching to manual.
            std::uint8_t cur = 0;
            saa_read(s.h, s.addr, kSaaInputCntl2, cur);
            std::uint8_t nv = (a1 == "on")
                                  ? std::uint8_t(cur |  kSaaHoldg)
                                  : std::uint8_t(cur & ~kSaaHoldg);
            saa_write(s.h, s.addr, kSaaInputCntl2, nv);
            std::println("  reg 0x03 <- 0x{:02x}  (HOLDG {})",
                         nv, (nv & kSaaHoldg) ? "on" : "off");
            continue;
        }
        if (cmd == "i" || cmd == "input") {
            std::uint8_t inp = 0;
            saa_read(s.h, s.addr, kSaaInputCntl1, inp);
            std::uint8_t mode = 0;
            if (parse_u8(a1, mode) < 0 || mode > 0x0f) { std::println("  bad input"); continue; }
            std::uint8_t nv = std::uint8_t((inp & 0xf0) | (mode & 0x0f));
            saa_write(s.h, s.addr, kSaaInputCntl1, nv);
            std::println("  reg 0x02 <- 0x{:02x}  (input MODE={})", nv, int(mode));
            continue;
        }
        if (cmd == "w" || cmd == "write") {
            std::uint8_t reg = 0, val = 0;
            if (parse_u8(a1, reg) < 0 || parse_u8(a2, val) < 0) { std::println("  bad arg"); continue; }
            int rc = saa_write(s.h, s.addr, reg, val);
            if (rc < 0) std::println("  write failed: {}", libusb_error_name(rc));
            else        std::println("  0x{:02x} <- 0x{:02x}", reg, val);
            continue;
        }
        if (cmd == "r" || cmd == "read") {
            std::uint8_t reg = 0, v = 0;
            if (parse_u8(a1, reg) < 0) { std::println("  bad reg"); continue; }
            int rc = saa_read(s.h, s.addr, reg, v);
            if (rc < 0) std::println("  read failed: {}", libusb_error_name(rc));
            else        std::println("  0x{:02x} = 0x{:02x} ({})", reg, v, v);
            continue;
        }
        if (cmd == "sw" || cmd == "stkwrite") {
            // Raw STK1160 register write: sw REG VAL  (REG is hex, e.g. 0x000).
            std::uint8_t val = 0;
            char* end = nullptr;
            unsigned long reg = std::strtoul(a1.c_str(), &end, 0);
            if (end == a1.c_str() || reg > 0xffff || parse_u8(a2, val) < 0) {
                std::println("  bad arg (need REG VAL)"); continue;
            }
            int rc = stk_write_reg(s.h, std::uint16_t(reg), val);
            std::println("  stk {:03x} <- {:02x} ({})", reg, val,
                         rc < 0 ? libusb_error_name(rc) : "ok");
            continue;
        }
        if (cmd == "sr" || cmd == "stkread") {
            std::uint8_t val = 0;
            char* end = nullptr;
            unsigned long reg = std::strtoul(a1.c_str(), &end, 0);
            if (end == a1.c_str() || reg > 0xffff) { std::println("  bad reg"); continue; }
            int rc = stk_read_reg(s.h, std::uint16_t(reg), val);
            if (rc < 0) std::println("  read failed: {}", libusb_error_name(rc));
            else        std::println("  stk {:03x} = {:02x}", reg, val);
            continue;
        }
        if (cmd == "gpio_scan" || cmd == "gscan") {
            // For each of the 10 GPIO pins, try driving HIGH then LOW;
            // after a settle, read SAA7113 status reg 0x1f. Print any
            // combo where HLCK transitions from 1 (unlocked) to 0 (locked).
            // Restores original GCTRL state on exit.
            std::uint8_t gv0=0, gv1=0, gd0=0, gd1=0;
            stk_read_reg(s.h, 0x000, gv0); stk_read_reg(s.h, 0x001, gv1);
            stk_read_reg(s.h, 0x002, gd0); stk_read_reg(s.h, 0x003, gd1);
            std::println("  saved GV={:02x}{:02x} GDIR={:02x}{:02x}", gv1, gv0, gd1, gd0);
            auto check = [&](const char* tag) {
                std::this_thread::sleep_for(std::chrono::milliseconds(150));
                std::uint8_t st = 0;
                if (saa_read(s.h, s.addr, 0x1f, st) < 0) return;
                int hlck = (st >> 7) & 1;
                int hlvln = (st >> 6) & 1;
                std::println("    {} -> reg1f=0x{:02x} HLCK={} HLVLN={}{}",
                             tag, st, hlck, hlvln, (hlck == 0 && hlvln == 0) ? "  *** LOCKED ***" : "");
            };
            // Pin-by-pin scan, GPIO 0..7 only (8/9 are typically straps).
            for (int pin = 0; pin < 8; ++pin) {
                std::println("  pin {}:", pin);
                // Force pin to output.
                stk_write_reg(s.h, 0x002, std::uint8_t(0x01 << pin));
                stk_write_reg(s.h, 0x000, std::uint8_t(0x01 << pin)); check("  HIGH");
                stk_write_reg(s.h, 0x000, 0x00);                       check("  LOW ");
            }
            // Also try ALL HIGH + ALL LOW.
            stk_write_reg(s.h, 0x002, 0xff);
            stk_write_reg(s.h, 0x000, 0xff); check("ALL 8 HIGH");
            stk_write_reg(s.h, 0x000, 0x00); check("ALL 8 LOW ");
            // Restore.
            stk_write_reg(s.h, 0x000, gv0); stk_write_reg(s.h, 0x001, gv1);
            stk_write_reg(s.h, 0x002, gd0); stk_write_reg(s.h, 0x003, gd1);
            std::println("  restored.");
            continue;
        }
        if (cmd == "debug") {
            s.debug = (a1 == "on");
            std::println("  debug {}", s.debug ? "on" : "off");
            continue;
        }
        if (cmd == "sleep") {
            // Pause the REPL for N ms (mostly for scripted runs piping in
            // commands). NB: doesn't pause isoc/window threads.
            char* end = nullptr;
            unsigned long ms = std::strtoul(a1.c_str(), &end, 0);
            if (end != a1.c_str() && ms <= 60000) {
                std::this_thread::sleep_for(std::chrono::milliseconds(ms));
            }
            continue;
        }
        if (cmd == "hist") {
            // Per-channel min/max/mean + a Y-channel 16-bin histogram. UYVY
            // layout: U Y0 V Y1, so byte offsets 0/2 are chroma, 1/3 are
            // luma. For a monochrome camera U,V should sit at ~128 (chroma
            // null); deviation suggests they carry extra info (or the chip
            // is decoding stray chroma off composite noise).
            struct ChanStat {
                int min = 255, max = 0;
                std::uint64_t sum = 0;
                std::uint64_t count = 0;
                void add(int v) {
                    if (v < min) min = v;
                    if (v > max) max = v;
                    sum += std::uint64_t(v);
                    ++count;
                }
                double mean() const { return count ? double(sum) / double(count) : 0.0; }
            };
            ChanStat U, Y, V;
            std::array<std::uint64_t, 16> ybins{};
            {
                std::lock_guard lk(s.frameMu);
                const std::uint8_t* p = s.front.uyvy.data();
                const int n = int(s.front.uyvy.size());
                for (int i = 0; i + 3 < n; i += 4) {
                    U.add(p[i + 0]);
                    Y.add(p[i + 1]);
                    V.add(p[i + 2]);
                    Y.add(p[i + 3]);
                    ybins[std::size_t(p[i + 1] >> 4)]++;
                    ybins[std::size_t(p[i + 3] >> 4)]++;
                }
            }
            if (Y.count == 0) { std::println("  no frame yet"); continue; }
            std::println("  Y: min={:3} max={:3} mean={:6.1f}  ({} samples)", Y.min, Y.max, Y.mean(), Y.count);
            std::println("  U: min={:3} max={:3} mean={:6.1f}  ({} samples)  [128 = neutral]",
                         U.min, U.max, U.mean(), U.count);
            std::println("  V: min={:3} max={:3} mean={:6.1f}  ({} samples)  [128 = neutral]",
                         V.min, V.max, V.mean(), V.count);
            std::println("  Y histogram (16 bins of 16):");
            std::uint64_t maxbin = 0;
            for (auto v : ybins) if (v > maxbin) maxbin = v;
            for (std::size_t b = 0; b < ybins.size(); ++b) {
                int barlen = maxbin ? int((ybins[b] * 40) / maxbin) : 0;
                std::string bar(std::size_t(barlen), '#');
                std::println("    {:3}-{:3}: {:>7} {}",
                             int(b * 16), int(b * 16 + 15), ybins[b], bar);
            }
            continue;
        }
        if (cmd == "lock") {
            std::uint8_t v = 0;
            if (saa_read(s.h, s.addr, 0x1f, v) >= 0) {
                std::println("  SAA7113 reg1f=0x{:02x}  HLCK={} HLVLN={} RDCAP={}",
                             v, (v >> 7) & 1, (v >> 6) & 1, v & 1);
            }
            continue;
        }
        if (cmd == "d" || cmd == "dump") {
            for (int row = 0; row < 8; ++row) {
                std::print("  {:02x}:", row * 16);
                for (int col = 0; col < 16; ++col) {
                    std::uint8_t v = 0;
                    if (saa_read(s.h, s.addr, std::uint8_t(row * 16 + col), v) < 0) std::print(" --");
                    else std::print(" {:02x}", v);
                }
                std::println("");
            }
            continue;
        }
        std::println("  unknown -- type 'help'");
    }
}

int write_seq(libusb_device_handle* h, std::span<const RegVal> seq) {
    for (auto rv : seq) {
        int rc = stk_write_reg(h, rv.reg, rv.val);
        if (rc < 0) return rc;
    }
    return 0;
}

int write_i2c_seq(libusb_device_handle* h, std::uint8_t addr, std::span<const RegVal> seq) {
    for (auto rv : seq) {
        int rc = saa_write(h, addr, std::uint8_t(rv.reg), rv.val);
        if (rc < 0) return rc;
    }
    return 0;
}

}  // namespace

int run_live(libusb_device_handle* h, std::uint8_t addr, bool debug) {
    State s;
    s.h = h;
    s.addr = addr;
    s.debug = debug;
    s.back.uyvy.assign(std::size_t(kFrameBytes), 0);
    s.front.uyvy.assign(std::size_t(kFrameBytes), 0);

    if (debug) {
        std::uint8_t st = 0;
        if (saa_read(h, addr, 0x1f, st) >= 0) {
            std::println("SAA7113 reg1f=0x{:02x}  HLCK={} HLVLN={} RDCAP={}",
                         st, (st >> 7) & 1, (st >> 6) & 1, st & 1);
        }
        std::uint8_t gv0=0, gv1=0, gd0=0, gd1=0;
        stk_read_reg(h, 0x000, gv0); stk_read_reg(h, 0x001, gv1);
        stk_read_reg(h, 0x002, gd0); stk_read_reg(h, 0x003, gd1);
        std::println("GCTRL: GV={:02x}{:02x} GDIR={:02x}{:02x}", gv1, gv0, gd1, gd0);
    }

    // Skip the STK1160 reset and SAA7113 cold init -- the chip stays powered
    // across the Zadig WinUSB swap, so previous (working) state from Syntek
    // is preserved. Our cold-init sequence (Linux saa7115_init) forces
    // MODE=AI21 + AUFD=1, which on the TVM802B (camera on AI11, no auto-
    // detectable signal) silently breaks sync and produces empty USB packets.
    // We only program the streaming-side bits.
    if (write_seq(h, kStkCaptureNtsc) < 0) {
        std::println(stderr, "capture-window sequence failed");
        return 1;
    }
    // 4. Pick alt setting with isoc bandwidth.
    int maxPkt = 0;
    int alt = pick_alt_setting(h, maxPkt);
    if (alt < 0) {
        std::println(stderr, "no suitable isoc alt setting found on EP 0x{:02x}", kStkEpVideo);
        return 1;
    }
    if (debug) {
        std::println("alt setting {} -- isoc EP 0x{:02x} maxPkt={}", alt, kStkEpVideo, maxPkt);
    }

    int cl = libusb_claim_interface(h, 0);
    if (cl < 0) {
        std::println(stderr, "claim_interface(0) failed: {}", libusb_error_name(cl));
        return 1;
    }
    int sa = libusb_set_interface_alt_setting(h, 0, alt);
    if (sa < 0) {
        if (debug) {
            std::println(stderr, "set_interface_alt_setting(0, {}) failed: {} -- "
                                 "falling back to raw SET_INTERFACE ctrl-xfer",
                                 alt, libusb_error_name(sa));
        }
        sa = libusb_control_transfer(h, 0x01, 0x0B, std::uint16_t(alt), 0, nullptr, 0, 1000);
        if (sa < 0) {
            std::println(stderr, "raw SET_INTERFACE also failed: {}", libusb_error_name(sa));
            return 1;
        }
    }

    // 5. Submit isoc URBs FIRST (URBs pending, ready to receive)...
    std::vector<libusb_transfer*> xfers(kNumBufs, nullptr);
    std::vector<std::vector<std::uint8_t>> bufs(kNumBufs);
    for (int i = 0; i < kNumBufs; ++i) {
        xfers[i] = libusb_alloc_transfer(kNumPackets);
        bufs[i].assign(std::size_t(kNumPackets * maxPkt), 0);
        libusb_fill_iso_transfer(xfers[i], h, kStkEpVideo,
                                 bufs[i].data(), int(bufs[i].size()),
                                 kNumPackets, on_isoc_complete, &s, 1000);
        libusb_set_iso_packet_lengths(xfers[i], std::uint32_t(maxPkt));
        int rc = libusb_submit_transfer(xfers[i]);
        if (rc < 0) {
            std::println(stderr, "submit_transfer[{}] failed: {}", i, libusb_error_name(rc));
        }
    }

    // 6. ... THEN kick streaming. Matches Linux stk1160_start_streaming.
    stk_write_reg(h, 0x100, 0xb3);  // DCTRL
    stk_write_reg(h, 0x103, 0x00);  // DCTRL+3

    // 7. Window + threads.
    s.hwnd = create_window(s);
    std::thread tEvt([&]{
        while (s.running.load(std::memory_order_acquire)) {
            timeval tv{0, 50000};
            libusb_handle_events_timeout_completed(nullptr, &tv, nullptr);
        }
    });
    // 1Hz isoc-stats line so it's obvious whether data is flowing. Off by
    // default; -d/--debug or `debug on` at the REPL toggles it.
    std::thread tStats([&]{
        std::uint64_t lastTotal = 0, lastFrames = 0;
        while (s.running.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (!s.debug) {
                lastTotal = s.pktTotal.load();
                lastFrames = s.framesDone.load();
                continue;
            }
            auto tot = s.pktTotal.load();
            auto fr  = s.framesDone.load();
            std::println("isoc: pkts/s={} maxLen={} (empty={} short={} data={} sof_c0={} sof_80={}) frames/s={}",
                         tot - lastTotal, s.maxLen.load(),
                         s.pktEmpty.load(), s.pktShort.load(), s.pktData.load(),
                         s.pktSofC0.load(), s.pktSof80.load(),
                         fr - lastFrames);
            lastTotal = tot;
            lastFrames = fr;
        }
    });
    std::thread tRepl([&]{ repl_thread(s); });

    MSG msg;
    while (s.running.load(std::memory_order_acquire) && GetMessage(&msg, nullptr, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
    s.running.store(false, std::memory_order_release);

    // Cancel + drain.
    for (auto* x : xfers) if (x) libusb_cancel_transfer(x);
    timeval tv{0, 500000};
    libusb_handle_events_timeout_completed(nullptr, &tv, nullptr);
    for (auto* x : xfers) if (x) libusb_free_transfer(x);

    libusb_release_interface(h, 0);
    if (tEvt.joinable()) tEvt.join();
    if (tStats.joinable()) tStats.join();
    if (tRepl.joinable()) tRepl.detach();  // blocked on stdin; OS will tear down
    return 0;
}

}  // namespace saa
