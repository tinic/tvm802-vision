// SAA7113 hardware control surface, exposed to the settings UI.
//
// Lazy-opens a WinUSB / libusbK handle to the STK1150 the first time any
// setter is called. All subsequent calls go straight through to I2C-master
// writes (single ~1 ms round trip per call). Coexists with the
// mvision_grabber.ax filter's open + isoc stream: control transfers
// (vendor requests on EP0) are non-exclusive in libusbK, so concurrent
// access works as long as we never libusb_claim_interface here.
//
// Implementation lives in src/camera_chip.cpp; only compiled into
// MVision.dll when BUILD_CAMERA_CONTROL=ON (adds a libusb-1.0 link dep).
// When the build flag is off, the header still exists but the .cpp isn't
// in MVISION_SOURCES, so any caller must guard with #ifdef
// MVISION_CAMERA_CONTROL or take a link error.

#pragma once

namespace camchip {

// Returns true if the chip is reachable. Subsequent calls cheap (cached).
bool available();

// Set chip controls. Values clamped to the chip's range. Return true on
// successful I2C write; false if the chip is unreachable.
bool set_brightness(int v);   // 0..255   (SAA7113 reg 0x0A, default 128)
bool set_contrast(int v);     // 0..127   (reg 0x0B, default 71)
bool set_saturation(int v);   // 0..127   (reg 0x0C, default 64)
bool set_hue(int v);          // -128..127 (reg 0x0D, default 0)
bool set_gain(int v);         // 0..511   (9-bit GAI18:GAI10/GAI28:GAI20 split,
                              //           also sets GAFIX=1 = manual gain)
bool set_agc(bool on);        // true  = GAFIX 0 = AGC drives gain
                              // false = GAFIX 1 = manual gain in reg 0x04/0x05
bool set_sharpness(int v);    // 0..3 (reg 0x09 APER bits): factor 0 / 0.25 / 0.5 / 1.0
bool set_prefilter(bool on);  // reg 0x09 PREF bit: true = luma low-pass on, false = off
bool set_wpoff(bool on);      // reg 0x03 WPOFF bit: true = AGC ignores white peaks (stabler exposure on glossy SMT)
bool set_hlnrs(bool on);      // reg 0x03 HLNRS bit: true = "reference select" clamping when H-sync briefly unlocks

// Last-set value (or chip default if never set this session). The UI
// uses these to initialise its sliders without doing a chip read at
// dialog-open time.
int get_brightness();
int get_contrast();
int get_saturation();
int get_hue();
int get_gain();
bool get_agc();
int get_sharpness();
bool get_prefilter();
bool get_wpoff();
bool get_hlnrs();

}  // namespace camchip
