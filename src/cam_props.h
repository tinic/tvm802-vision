#pragma once
// SAA7113 hardware brightness / contrast via DirectShow IAMVideoProcAmp on the
// STK1150 "USB2.0 Grabber" capture filter.
//
// These are PRE-capture adjustments applied in the SAA7113 decoder's video-
// processor block BEFORE the chip pushes its 8-bit YUYV byte to the STK1160
// USB bridge -- they affect the analog-to-digital pipeline, not our OpenCV
// post-capture buffer. Distinct from the Image-tab Gamma / Brightness / Contrast
// sliders which operate on the already-captured 8-bit gray frame.
//
// Probe (commit TBD) confirmed the DirectShow ranges match the SAA7113 register
// widths exactly:
//   Brightness  0-255  -> SAA7113 reg 0x0A BRIGHT  (8 bits)
//   Contrast    0-127  -> SAA7113 reg 0x0B CONTRAST (7 bits)
//   Saturation  0-127  -> SAA7113 reg 0x0C SATN     (7 bits)
//   Hue         0-127  -> SAA7113 reg 0x0D HUEC     (7 bits)
// Strong signal the Windows driver is forwarding straight to the I2C-over-USB
// register write. We only expose the two useful-for-mono ones (Brightness =
// luminance offset, Contrast = luminance gain / scaling around midpoint).
//
// All functions return false / fallback on any failure -- no-throw barrier
// (DirectShow calls inside the shim must never let an exception cross the
// C ABI). Safe to call from any thread; lazily initialises COM + the IAMVideoProcAmp
// interface on first use, holds the interface for the DLL's lifetime.

namespace vis {

// Push a value to the SAA7113 via the STK1150 driver's IAMVideoProcAmp interface.
// Range 0-255 for brightness, 0-127 for contrast. Returns true on success.
bool set_cam_brightness(int v);
bool set_cam_contrast(int v);

// Read the current value. Returns the fallback default if the device or
// interface isn't reachable. Used to initialise the UI sliders to whatever
// the SAA7113 currently has (it retains values across DLL re-loads).
int get_cam_brightness(int fallback = 128);
int get_cam_contrast(int fallback = 64);

}  // namespace vis
