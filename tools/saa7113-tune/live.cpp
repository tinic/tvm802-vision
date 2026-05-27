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
// NTSC 720x480 UYVY. PAL would be 720x576 + a different CFEPO; only NTSC
// here because the TVM802B is configured for NTSC on the analog side.
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
    libusb_context* ctx = nullptr;       // for handle_events_timeout_completed
    std::uint8_t addr = kSaaDefaultAddr;
    std::atomic<bool> running{true};

    // Double-buffered frame: ISOC fills `back`, swaps under mutex on
    // end-of-frame, WM_PAINT reads `front`.
    std::mutex frameMu;
    Frame back;
    Frame front;
    std::atomic<bool> frontDirty{false};

    HWND hwnd = nullptr;
};

// ---- Isoc callback ------------------------------------------------------

void process_packet(State& s, const std::uint8_t* p, int len) {
    if (len <= 4) return;
    if (p[0] == 0xc0) {
        // End-of-frame: swap back -> front under lock, signal repaint.
        {
            std::lock_guard lk(s.frameMu);
            std::swap(s.back, s.front);
            s.back.pos = 0;
            s.back.bytesused = 0;
        }
        s.frontDirty.store(true, std::memory_order_release);
        if (s.hwnd != nullptr) {
            // PostMessage is the documented way to nudge the GUI thread
            // from a worker.
            PostMessage(s.hwnd, WM_APP + 1, 0, 0);
        }
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

    int linesdone = s.back.pos / kBytesPerLine;
    int lineoff = s.back.pos % kBytesPerLine;

    // odd field on top (even line numbers 0,2,...), even field below
    // (odd line numbers 1,3,...): the same alternation Linux uses.
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
void uyvy_to_bgr(const std::uint8_t* uyvy, std::uint8_t* bgr, int w, int h) {
    for (int y = 0; y < h; ++y) {
        const std::uint8_t* sp = uyvy + y * w * 2;
        std::uint8_t* dp = bgr + y * w * 3;
        for (int x = 0; x < w; x += 2) {
            int u  = sp[0] - 128;
            int y0 = sp[1];
            int v  = sp[2] - 128;
            int y1 = sp[3];
            auto cvt = [&](int Y, int& B, int& G, int& R) {
                int c = Y - 16;
                B = (298 * c + 516 * u + 128) >> 8;
                G = (298 * c - 100 * u - 208 * v + 128) >> 8;
                R = (298 * c + 409 * v + 128) >> 8;
                B = std::clamp(B, 0, 255);
                G = std::clamp(G, 0, 255);
                R = std::clamp(R, 0, 255);
            };
            int B, G, R;
            cvt(y0, B, G, R);
            dp[0] = std::uint8_t(B); dp[1] = std::uint8_t(G); dp[2] = std::uint8_t(R);
            cvt(y1, B, G, R);
            dp[3] = std::uint8_t(B); dp[4] = std::uint8_t(G); dp[5] = std::uint8_t(R);
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
        "  g NNN  / gain NNN         reg 0x04+0x05 manual gain, disables AGC\n"
        "  agc on|off                FUSE bits in reg 0x02\n"
        "  i 0..3 / input 0..3       AI11/AI12/AI21/AI22 select in reg 0x02\n"
        "  w REG VAL / write REG VAL raw SAA7113 register write (hex or dec)\n"
        "  r REG     / read REG      raw read\n"
        "  d         / dump          all 128 SAA7113 registers\n"
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
            // Manual gain: disable AGC (FUSE=10 in reg 0x02) + write same gain
            // to both channels. NB: this also forces MODE bits -- read+modify+
            // write reg 0x02 to preserve input selection.
            std::uint8_t v = 0;
            if (parse_u8(a1, v) < 0) { std::println("  bad gain"); continue; }
            std::uint8_t inp = 0;
            saa_read(s.h, s.addr, kSaaInputCntl1, inp);
            std::uint8_t fix = std::uint8_t((inp & 0x3f) | 0x80);  // FUSE=10
            saa_write(s.h, s.addr, kSaaInputCntl1, fix);
            saa_write(s.h, s.addr, kSaaGainCh1Lo, v);
            saa_write(s.h, s.addr, kSaaGainCh2Lo, v);
            std::println("  gain {} written to ch1+ch2 (AGC fixed)", int(v));
            continue;
        }
        if (cmd == "agc") {
            std::uint8_t inp = 0;
            saa_read(s.h, s.addr, kSaaInputCntl1, inp);
            std::uint8_t fuse = (a1 == "on") ? 0xc0 : 0x80;  // 11=AGC, 10=fixed
            std::uint8_t nv = std::uint8_t((inp & 0x3f) | fuse);
            saa_write(s.h, s.addr, kSaaInputCntl1, nv);
            std::println("  reg 0x02 <- 0x{:02x}  (AGC {})", nv, a1);
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

int run_live(libusb_device_handle* h, std::uint8_t addr) {
    State s;
    s.h = h;
    s.ctx = libusb_get_context(libusb_get_device(h));
    s.addr = addr;
    s.back.uyvy.assign(std::size_t(kFrameBytes), 0);
    s.front.uyvy.assign(std::size_t(kFrameBytes), 0);

    // 1. Reset STK1160 housekeeping.
    if (write_seq(h, kStkResetSeq) < 0) {
        std::println(stderr, "STK1160 reset register sequence failed");
        return 1;
    }
    // 2. SAA7113 cold init.
    if (write_i2c_seq(h, addr, kSaa7113Init) < 0) {
        std::println(stderr, "SAA7113 init sequence failed -- wrong I2C addr? Try --addr 0x24");
        return 1;
    }
    // 3. NTSC capture window.
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
    std::println("alt setting {} -- isoc EP 0x{:02x} maxPkt={}", alt, kStkEpVideo, maxPkt);

    if (libusb_claim_interface(h, 0) < 0) {
        std::println(stderr, "claim_interface(0) failed");
        return 1;
    }
    if (libusb_set_interface_alt_setting(h, 0, alt) < 0) {
        std::println(stderr, "set_alt({}) failed", alt);
        return 1;
    }

    // 5. Kick streaming.
    stk_write_reg(h, 0x100, 0xb3);  // DCTRL
    stk_write_reg(h, 0x103, 0x00);  // DCTRL+3

    // 6. Submit isoc URBs.
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

    // 7. Window + threads.
    s.hwnd = create_window(s);
    std::thread tEvt([&]{
        while (s.running.load(std::memory_order_acquire)) {
            timeval tv{0, 50000};
            libusb_handle_events_timeout_completed(s.ctx, &tv, nullptr);
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
    libusb_handle_events_timeout_completed(s.ctx, &tv, nullptr);
    for (auto* x : xfers) if (x) libusb_free_transfer(x);

    libusb_release_interface(h, 0);
    if (tEvt.joinable()) tEvt.join();
    if (tRepl.joinable()) tRepl.detach();  // blocked on stdin; OS will tear down
    return 0;
}

}  // namespace saa
