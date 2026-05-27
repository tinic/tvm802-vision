#include "camera_chip.h"

#include "../tools/saa7113-tune/common.h"

#include <algorithm>
#include <mutex>

namespace camchip {

namespace {

std::once_flag g_initOnce;
std::mutex g_mu;
saa::UsbCtx g_usb;
bool g_ok = false;

int g_brightness = 128;
int g_contrast = 71;
int g_saturation = 64;
int g_hue = 0;
int g_gain = 117;
bool g_agc = true;

// Open libusb the first time anyone asks. Subsequent calls cheap.
bool ensure() {
    std::call_once(g_initOnce, [] {
        g_ok = saa::open_device(g_usb) >= 0;
    });
    return g_ok;
}

}  // namespace

bool available() {
    std::lock_guard lk(g_mu);
    return ensure();
}

bool set_brightness(int v) {
    std::lock_guard lk(g_mu);
    g_brightness = std::clamp(v, 0, 255);
    if (!ensure())
        return false;
    return saa::saa_write(g_usb.h, saa::kSaaDefaultAddr, saa::kSaaBrightness,
                          static_cast<std::uint8_t>(g_brightness)) >= 0;
}

bool set_contrast(int v) {
    std::lock_guard lk(g_mu);
    g_contrast = std::clamp(v, 0, 127);
    if (!ensure())
        return false;
    return saa::saa_write(g_usb.h, saa::kSaaDefaultAddr, saa::kSaaContrast,
                          static_cast<std::uint8_t>(g_contrast)) >= 0;
}

bool set_saturation(int v) {
    std::lock_guard lk(g_mu);
    g_saturation = std::clamp(v, 0, 127);
    if (!ensure())
        return false;
    return saa::saa_write(g_usb.h, saa::kSaaDefaultAddr, saa::kSaaSaturation,
                          static_cast<std::uint8_t>(g_saturation)) >= 0;
}

bool set_hue(int v) {
    std::lock_guard lk(g_mu);
    g_hue = std::clamp(v, -128, 127);
    if (!ensure())
        return false;
    return saa::saa_write(g_usb.h, saa::kSaaDefaultAddr, saa::kSaaHue,
                          static_cast<std::uint8_t>(g_hue)) >= 0;
}

bool set_gain(int v) {
    std::lock_guard lk(g_mu);
    g_gain = std::clamp(v, 0, 511);
    g_agc = false;  // manual gain auto-sets GAFIX
    if (!ensure())
        return false;
    auto h = g_usb.h;
    const auto addr = saa::kSaaDefaultAddr;
    std::uint8_t cur = 0;
    if (saa::saa_read(h, addr, saa::kSaaInputCntl2, cur) < 0)
        return false;
    const std::uint8_t nv = static_cast<std::uint8_t>(
        (cur & ~(saa::kSaaGafix | saa::kSaaGai18 | saa::kSaaGai28)) | saa::kSaaGafix | ((g_gain & 0x100) ? saa::kSaaGai18 : 0) | ((g_gain & 0x100) ? saa::kSaaGai28 : 0));
    if (saa::saa_write(h, addr, saa::kSaaInputCntl2, nv) < 0)
        return false;
    const std::uint8_t lo = static_cast<std::uint8_t>(g_gain & 0xff);
    if (saa::saa_write(h, addr, saa::kSaaGainCh1Lo, lo) < 0)
        return false;
    if (saa::saa_write(h, addr, saa::kSaaGainCh2Lo, lo) < 0)
        return false;
    return true;
}

bool set_agc(bool on) {
    std::lock_guard lk(g_mu);
    g_agc = on;
    if (!ensure())
        return false;
    auto h = g_usb.h;
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
    return g_brightness;
}
int get_contrast() {
    return g_contrast;
}
int get_saturation() {
    return g_saturation;
}
int get_hue() {
    return g_hue;
}
int get_gain() {
    return g_gain;
}
bool get_agc() {
    return g_agc;
}

}  // namespace camchip
