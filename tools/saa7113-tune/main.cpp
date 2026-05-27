// saa7113-tune -- one-shot SAA7113 register tuner over the STK1160 I2C master.
//
// See README.md for the operational story. The shared USB / I2C primitives
// live in common.{h,cpp}; the live-preview Win32 window + REPL is in
// live.{h,cpp}. This file is argument parsing + one-shot subcommands.

#include "common.h"
#ifndef SAA_NO_LIVE
#include "live.h"
#endif

#include <libusb.h>

#include <cstdint>
#include <cstdlib>
#include <print>
#include <string_view>

using namespace saa;

namespace {

int cmd_dump(libusb_device_handle* h, std::uint8_t addr) {
    std::println("SAA7113 register dump (I2C addr 0x{:02x}):", addr);
    std::print("     ");
    for (int c = 0; c < 16; ++c) std::print(" {:02x}", c);
    std::println("");
    for (int row = 0; row < 8; ++row) {
        std::print("  {:02x}:", row * 16);
        for (int col = 0; col < 16; ++col) {
            std::uint8_t v = 0;
            const int rc = saa_read(h, addr, std::uint8_t(row * 16 + col), v);
            if (rc < 0) std::print(" --");
            else        std::print(" {:02x}", v);
        }
        std::println("");
    }
    return 0;
}

int cmd_scan(libusb_device_handle* h) {
    // Probe 7-bit I2C addresses 0x08..0x77 by attempting a read of subaddr 0
    // and checking whether the transaction completes. A device that ACKs
    // completes; one that NAKs times out.
    std::println("I2C bus scan (7-bit addresses that ACK):");
    for (std::uint8_t a = 0x08; a <= 0x77; ++a) {
        std::uint8_t v = 0;
        if (saa_read(h, a, 0x00, v) == 0) {
            std::println("  0x{:02x}  (reg00=0x{:02x})", a, v);
        }
    }
    return 0;
}

int cmd_read(libusb_device_handle* h, std::uint8_t addr, std::string_view reg_s) {
    std::uint8_t reg = 0;
    if (parse_u8(reg_s, reg) < 0) { std::println(stderr, "bad register: {}", reg_s); return 1; }
    std::uint8_t v = 0;
    const int rc = saa_read(h, addr, reg, v);
    if (rc < 0) { std::println(stderr, "read failed: {}", libusb_error_name(rc)); return 1; }
    std::println("0x{:02x} = 0x{:02x} ({})", reg, v, v);
    return 0;
}

int cmd_write(libusb_device_handle* h, std::uint8_t addr, std::string_view reg_s,
              std::string_view val_s) {
    std::uint8_t reg = 0;
    std::uint8_t val = 0;
    if (parse_u8(reg_s, reg) < 0 || parse_u8(val_s, val) < 0) {
        std::println(stderr, "bad arg (need REG VAL, decimal or 0x..)");
        return 1;
    }
    const int rc = saa_write(h, addr, reg, val);
    if (rc < 0) { std::println(stderr, "write failed: {}", libusb_error_name(rc)); return 1; }
    std::uint8_t v = 0;
    if (saa_read(h, addr, reg, v) == 0)
        std::println("0x{:02x} <- 0x{:02x}  (read-back 0x{:02x})", reg, val, v);
    else
        std::println("0x{:02x} <- 0x{:02x}  (read-back failed)", reg, val);
    return 0;
}

int cmd_named(libusb_device_handle* h, std::uint8_t addr, std::uint8_t reg,
              std::string_view name, std::string_view val_s) {
    std::uint8_t val = 0;
    if (parse_u8(val_s, val) < 0) { std::println(stderr, "bad {} value: {}", name, val_s); return 1; }
    const int rc = saa_write(h, addr, reg, val);
    if (rc < 0) { std::println(stderr, "write failed: {}", libusb_error_name(rc)); return 1; }
    std::println("{} (reg 0x{:02x}) <- 0x{:02x} ({})", name, reg, val, val);
    return 0;
}

void usage(std::string_view argv0) {
    std::println(
        "saa7113-tune -- SAA7113 register tuner via STK1160 I2C master\n"
        "\n"
        "One-shot subcommands:\n"
        "  {0} [--addr 0x25] dump\n"
        "  {0} [--addr 0x25] scan\n"
        "  {0} [--addr 0x25] read  REG\n"
        "  {0} [--addr 0x25] write REG VAL\n"
        "  {0} [--addr 0x25] brightness 0..255   (reg 0x0A)\n"
        "  {0} [--addr 0x25] contrast   0..127   (reg 0x0B)\n"
        "  {0} [--addr 0x25] saturation 0..127   (reg 0x0C)\n"
        "  {0} [--addr 0x25] hue        0..255   (reg 0x0D)\n"
        "\n"
        "Live preview + interactive REPL:\n"
        "  {0} [--addr 0x25] live\n"
        "    Opens a window streaming UYVY frames over WinUSB. Type tweaks at\n"
        "    the console; effect appears live (no driver swap per tweak).\n"
        "\n"
        "REG, VAL accept decimal or 0xHEX. --addr defaults to 0x25 (= 8-bit\n"
        "0x4A, SAA7113 RTS0 strapped HIGH); 0x24 is the strap-LOW alternate.\n"
        "Run with SurfaceMount.exe CLOSED, after Zadig has bound WinUSB to\n"
        "the STK1150 'USB2.0 Grabber'. See README.md.",
        argv0);
}

}  // namespace

int main(int argc, char* argv[]) {
    std::uint8_t addr = kSaaDefaultAddr;

    int argi = 1;
    if (argi + 1 < argc && std::string_view(argv[argi]) == "--addr") {
        if (parse_u8(argv[argi + 1], addr) < 0 || addr > 0x7F) {
            std::println(stderr, "bad --addr value: {}", argv[argi + 1]);
            return 1;
        }
        argi += 2;
    }
    if (argi >= argc) { usage(argv[0]); return 1; }
    const std::string_view cmd = argv[argi];
    const int rest = argc - argi - 1;
    char** restv = argv + argi + 1;

    UsbCtx u;
    if (open_device(u) < 0) return 1;

    if (cmd == "dump" && rest == 0)               return cmd_dump(u.h, addr);
    if (cmd == "scan" && rest == 0)               return cmd_scan(u.h);
    if (cmd == "read" && rest == 1)               return cmd_read(u.h, addr, restv[0]);
    if (cmd == "write" && rest == 2)              return cmd_write(u.h, addr, restv[0], restv[1]);
    if (cmd == "brightness" && rest == 1)         return cmd_named(u.h, addr, kSaaBrightness, "Brightness", restv[0]);
    if (cmd == "contrast" && rest == 1)           return cmd_named(u.h, addr, kSaaContrast,   "Contrast",   restv[0]);
    if (cmd == "saturation" && rest == 1)         return cmd_named(u.h, addr, kSaaSaturation, "Saturation", restv[0]);
    if (cmd == "hue" && rest == 1)                return cmd_named(u.h, addr, kSaaHue,        "Hue",        restv[0]);
    if (cmd == "live" && rest == 0) {
#ifndef SAA_NO_LIVE
        return run_live(u.h, addr);
#else
        std::println(stderr, "'live' requires the Win32 build (Windows-only).");
        return 1;
#endif
    }

    usage(argv[0]);
    return 1;
}
