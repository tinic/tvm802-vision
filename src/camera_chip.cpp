#include "camera_chip.h"

#include "../tools/saa7113-tune/common.h"

#include <algorithm>
#include <mutex>

namespace camchip {

namespace {

// HEAP-ALLOCATE + NEVER DESTROY. SurfaceMount.exe unloads MVision.dll on
// exit; a static UsbCtx's dtor then runs libusb_close + libusb_exit, which
// destroys libusb's process-wide default context. mvision_grabber.ax
// (loaded into the same process) holds its own libusb refs and crashes
// the moment something accesses them after we've torn the context down.
//
// Same fix as the grabber's "leak Filter object on Release" -- never let
// our state's dtor run. Heap-allocate, store the pointer in a function-
// local-static (or static raw ptr), let process exit reclaim everything.
// Costs a few hundred bytes of leak per process; saves the AV.
struct State {
    saa::UsbCtx usb;
    std::mutex mu;
    bool ok = false;
    int brightness = 128;
    int contrast = 71;
    int saturation = 64;
    int hue = 0;
    int gain = 117;
    bool agc = true;
    int sharpness = 0;       // APER bits in reg 0x09: 0=off, 1=0.25x, 2=0.5x, 3=1.0x
    bool prefilter = false;  // PREF bit in reg 0x09
};

// SAA7113 Luminance Control (reg 0x09) bit positions, per datasheet Table 36.
constexpr std::uint8_t kSaaApeMask = 0x03;   // D0-D1: aperture factor
constexpr std::uint8_t kSaaPrefMask = 0x40;  // D6: prefilter on/off (1 = on)
State& state() {
    static State* s = nullptr;
    static std::once_flag once;
    std::call_once(once, [] {
        s = new State{};
        s->ok = saa::open_device(s->usb) >= 0;
    });
    return *s;
}

bool ensure() {
    return state().ok;
}

}  // namespace

bool available() {
    State& s = state();
    std::lock_guard lk(s.mu);
    return ensure();
}

bool set_brightness(int v) {
    State& s = state();
    std::lock_guard lk(s.mu);
    s.brightness = std::clamp(v, 0, 255);
    if (!ensure())
        return false;
    return saa::saa_write(s.usb.h, saa::kSaaDefaultAddr, saa::kSaaBrightness,
                          static_cast<std::uint8_t>(s.brightness)) >= 0;
}

bool set_contrast(int v) {
    State& s = state();
    std::lock_guard lk(s.mu);
    s.contrast = std::clamp(v, 0, 127);
    if (!ensure())
        return false;
    return saa::saa_write(s.usb.h, saa::kSaaDefaultAddr, saa::kSaaContrast,
                          static_cast<std::uint8_t>(s.contrast)) >= 0;
}

bool set_saturation(int v) {
    State& s = state();
    std::lock_guard lk(s.mu);
    s.saturation = std::clamp(v, 0, 127);
    if (!ensure())
        return false;
    return saa::saa_write(s.usb.h, saa::kSaaDefaultAddr, saa::kSaaSaturation,
                          static_cast<std::uint8_t>(s.saturation)) >= 0;
}

bool set_hue(int v) {
    State& s = state();
    std::lock_guard lk(s.mu);
    s.hue = std::clamp(v, -128, 127);
    if (!ensure())
        return false;
    return saa::saa_write(s.usb.h, saa::kSaaDefaultAddr, saa::kSaaHue,
                          static_cast<std::uint8_t>(s.hue)) >= 0;
}

bool set_gain(int v) {
    State& s = state();
    std::lock_guard lk(s.mu);
    s.gain = std::clamp(v, 0, 511);
    s.agc = false;  // manual gain auto-sets GAFIX
    if (!ensure())
        return false;
    auto h = s.usb.h;
    const auto addr = saa::kSaaDefaultAddr;
    std::uint8_t cur = 0;
    if (saa::saa_read(h, addr, saa::kSaaInputCntl2, cur) < 0)
        return false;
    const std::uint8_t nv = static_cast<std::uint8_t>(
        (cur & ~(saa::kSaaGafix | saa::kSaaGai18 | saa::kSaaGai28)) | saa::kSaaGafix |
        ((s.gain & 0x100) ? saa::kSaaGai18 : 0) |
        ((s.gain & 0x100) ? saa::kSaaGai28 : 0));
    if (saa::saa_write(h, addr, saa::kSaaInputCntl2, nv) < 0)
        return false;
    const std::uint8_t lo = static_cast<std::uint8_t>(s.gain & 0xff);
    if (saa::saa_write(h, addr, saa::kSaaGainCh1Lo, lo) < 0)
        return false;
    if (saa::saa_write(h, addr, saa::kSaaGainCh2Lo, lo) < 0)
        return false;
    return true;
}

bool set_agc(bool on) {
    State& s = state();
    std::lock_guard lk(s.mu);
    s.agc = on;
    if (!ensure())
        return false;
    auto h = s.usb.h;
    const auto addr = saa::kSaaDefaultAddr;
    std::uint8_t cur = 0;
    if (saa::saa_read(h, addr, saa::kSaaInputCntl2, cur) < 0)
        return false;
    const std::uint8_t nv = on
                                ? static_cast<std::uint8_t>(cur & ~saa::kSaaGafix)
                                : static_cast<std::uint8_t>(cur | saa::kSaaGafix);
    return saa::saa_write(h, addr, saa::kSaaInputCntl2, nv) >= 0;
}

int get_brightness() {
    return state().brightness;
}
int get_contrast() {
    return state().contrast;
}
int get_saturation() {
    return state().saturation;
}
int get_hue() {
    return state().hue;
}
int get_gain() {
    return state().gain;
}
bool get_agc() {
    return state().agc;
}
int get_sharpness() {
    return state().sharpness;
}
bool get_prefilter() {
    return state().prefilter;
}

// Both sharpness (APER) and prefilter (PREF) live in reg 0x09 (LCR), so we
// read-modify-write to preserve the other bits (BYPS / BPSS / VBLB / UPTCV).
bool set_sharpness(int v) {
    State& s = state();
    std::lock_guard lk(s.mu);
    s.sharpness = std::clamp(v, 0, 3);
    if (!ensure())
        return false;
    auto h = s.usb.h;
    const auto addr = saa::kSaaDefaultAddr;
    std::uint8_t cur = 0;
    if (saa::saa_read(h, addr, saa::kSaaLumaCntl, cur) < 0)
        return false;
    const std::uint8_t nv = static_cast<std::uint8_t>(
        (cur & ~kSaaApeMask) | static_cast<std::uint8_t>(s.sharpness & kSaaApeMask));
    return saa::saa_write(h, addr, saa::kSaaLumaCntl, nv) >= 0;
}

bool set_prefilter(bool on) {
    State& s = state();
    std::lock_guard lk(s.mu);
    s.prefilter = on;
    if (!ensure())
        return false;
    auto h = s.usb.h;
    const auto addr = saa::kSaaDefaultAddr;
    std::uint8_t cur = 0;
    if (saa::saa_read(h, addr, saa::kSaaLumaCntl, cur) < 0)
        return false;
    const std::uint8_t nv = on ? static_cast<std::uint8_t>(cur | kSaaPrefMask)
                               : static_cast<std::uint8_t>(cur & ~kSaaPrefMask);
    return saa::saa_write(h, addr, saa::kSaaLumaCntl, nv) >= 0;
}

}  // namespace camchip
