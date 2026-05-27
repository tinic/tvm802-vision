#pragma once

#include <libusb.h>

#include <cstdint>

namespace saa {

// Interactive live-preview mode: open a Win32 window streaming UYVY frames
// from the STK1150/STK1160 via isoc, run a stdin REPL for SAA7113 register
// tweaks alongside. Blocks until the user enters 'q' or closes the window.
// `addr` is the SAA7113 7-bit I2C address.
int run_live(libusb_device_handle* h, std::uint8_t addr);

}  // namespace saa
