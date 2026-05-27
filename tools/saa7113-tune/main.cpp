// saa7113-tune -- one-shot SAA7113 register tuner over the STK1160 I2C master.
//
// Why this exists
// ---------------
// The stock Syntek Windows driver for the "USB2.0 Grabber" (STK1150/STK1160)
// targets a CMOS-Bayer sensor family (SyntekM811) -- not the SAA7113 analog
// decoder that's on the QiHe TVM802B board. Every IAMVideoProcAmp Brightness/
// Contrast/Gain call therefore drives a pipeline that isn't there, and the
// driver silently substitutes software post-processing on the YUYV buffer.
//
// To reach the real analog gain on the SAA7113 we bypass the Syntek driver
// entirely and drive the STK1160's I2C master directly over USB, writing
// SAA7113 registers (Brightness 0x0A, Contrast 0x0B, Saturation 0x0C, ...).
//
// One-shot workflow (per machine)
// -------------------------------
//  1. Close SurfaceMount.exe (it holds the device open via the Syntek driver).
//  2. Run Zadig and replace the "USB2.0 Grabber" driver with WinUSB. (One
//     time per machine; see README.md.)
//  3. saa7113-tune dump            -- verify communication.
//  4. saa7113-tune brightness 144  -- dial each control, previewing in
//     saa7113-tune contrast   80      SurfaceMount after swapping back.
//  5. Run Zadig once more and restore the original Syntek driver.
//  6. Relaunch SurfaceMount. The SAA7113 retains the values across the
//     driver swap because the chip stays powered via USB Vbus.
//
// USB protocol (from drivers/media/usb/stk1160/, GPL-2.0)
// -------------------------------------------------------
//   STK1160 register write: bmRequestType=0x40, bRequest=0x01,
//                           wValue=value, wIndex=reg, no data, 1000 ms.
//   STK1160 register read:  bmRequestType=0xC0, bRequest=0x00,
//                           wValue=0,     wIndex=reg, 1-byte IN, 1000 ms.
//
// SAA7113 I2C transaction over the STK1160's I2C master
// -----------------------------------------------------
//   write(addr, reg, val):
//     stk_write_reg(0x203, addr << 1);  // SICTL_SDA -- 8-bit slave address
//     stk_write_reg(0x204, reg);        // SBUSW_WA  -- subaddress
//     stk_write_reg(0x205, val);        // SBUSW_WD  -- data
//     stk_write_reg(0x200, 0x01);       // SICTL     -- kick write
//     poll  0x201 until bit 0x04 set.
//
//   read(addr, reg) -> val:
//     stk_write_reg(0x203, addr << 1);  // SICTL_SDA
//     stk_write_reg(0x208, reg);        // SBUSR_RA  -- subaddress (read path)
//     stk_write_reg(0x200, 0x20);       // SICTL     -- kick read
//     poll  0x201 until bit 0x01 set.
//     stk_read_reg(0x209) -> val.       // SBUSR_RD

#include <libusb.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <print>
#include <string_view>
#include <thread>

namespace {

// Known STK1150/STK1160 USB IDs. The TVM802B "USB2.0 Grabber" is 05e1:0408;
// the other entries are the rest of the family the same INF binds to, so
// this tool works unchanged on any of them.
struct UsbId {
    std::uint16_t vid;
    std::uint16_t pid;
};
constexpr std::array<UsbId, 4> kKnownIds{{
    {0x05e1, 0x0408},  // TVM802B
    {0x05e1, 0x0400},
    {0x05e1, 0x04f3},
    {0x05e1, 0x04f4},
}};

// SAA7113H 7-bit I2C slave address (SA pin strapped LOW; the alternate is
// 0x24 for SA HIGH). --addr overrides if a future board straps differently.
constexpr std::uint8_t kSaaDefaultAddr = 0x25;

// SAA7113 control-register subset that has well-known semantics and is what
// you actually want to dial. Everything else is reachable via `read`/`write`.
constexpr std::uint8_t kSaaBrightness = 0x0A;  // 0..255, default 0x80=128
constexpr std::uint8_t kSaaContrast   = 0x0B;  // -128..127 s8, default 0x44
constexpr std::uint8_t kSaaSaturation = 0x0C;  // -128..127 s8, default 0x40
constexpr std::uint8_t kSaaHue        = 0x0D;  // -128..127 s8, default 0x00

// STK1160 I2C-master registers (drivers/media/usb/stk1160/stk1160-reg.h).
constexpr std::uint16_t kStkSictl    = 0x200;  // I2C control / kick
constexpr std::uint16_t kStkSictlSt  = 0x201;  // status (polled)
constexpr std::uint16_t kStkSictlSda = 0x203;  // slave address (8-bit form)
constexpr std::uint16_t kStkSbuswWa  = 0x204;  // write-path subaddress
constexpr std::uint16_t kStkSbuswWd  = 0x205;  // write-path data
constexpr std::uint16_t kStkSbusrRa  = 0x208;  // read-path subaddress
constexpr std::uint16_t kStkSbusrRd  = 0x209;  // read-path data

constexpr std::uint8_t kStkKickWrite = 0x01;
constexpr std::uint8_t kStkKickRead  = 0x20;
constexpr std::uint8_t kStkDoneWrite = 0x04;
constexpr std::uint8_t kStkDoneRead  = 0x01;

constexpr unsigned int kPollTimeoutMs = 100;
constexpr unsigned int kCtrlTimeoutMs = 1000;

// ---- USB control transfers ----------------------------------------------

int stk_write_reg(libusb_device_handle* h, std::uint16_t reg, std::uint8_t val) {
    return libusb_control_transfer(h, 0x40, 0x01, val, reg, nullptr, 0, kCtrlTimeoutMs);
}

int stk_read_reg(libusb_device_handle* h, std::uint16_t reg, std::uint8_t& out) {
    out = 0;
    return libusb_control_transfer(h, 0xC0, 0x00, 0, reg, &out, 1, kCtrlTimeoutMs);
}

// ---- I2C transaction poll ----------------------------------------------

bool i2c_busy_wait(libusb_device_handle* h, std::uint8_t done_bit) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(kPollTimeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        std::uint8_t flag = 0;
        if (stk_read_reg(h, kStkSictlSt, flag) < 0) {
            return false;
        }
        if ((flag & done_bit) != 0) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}

// ---- SAA7113 register access ------------------------------------------

int saa_write(libusb_device_handle* h, std::uint8_t addr, std::uint8_t reg, std::uint8_t val) {
    int rc = 0;
    if ((rc = stk_write_reg(h, kStkSictlSda, std::uint8_t(addr << 1))) < 0) {
        return rc;
    }
    if ((rc = stk_write_reg(h, kStkSbuswWa, reg)) < 0) {
        return rc;
    }
    if ((rc = stk_write_reg(h, kStkSbuswWd, val)) < 0) {
        return rc;
    }
    if ((rc = stk_write_reg(h, kStkSictl, kStkKickWrite)) < 0) {
        return rc;
    }
    if (!i2c_busy_wait(h, kStkDoneWrite)) {
        return LIBUSB_ERROR_TIMEOUT;
    }
    return 0;
}

int saa_read(libusb_device_handle* h, std::uint8_t addr, std::uint8_t reg, std::uint8_t& out) {
    int rc = 0;
    out = 0;
    if ((rc = stk_write_reg(h, kStkSictlSda, std::uint8_t(addr << 1))) < 0) {
        return rc;
    }
    if ((rc = stk_write_reg(h, kStkSbusrRa, reg)) < 0) {
        return rc;
    }
    if ((rc = stk_write_reg(h, kStkSictl, kStkKickRead)) < 0) {
        return rc;
    }
    if (!i2c_busy_wait(h, kStkDoneRead)) {
        return LIBUSB_ERROR_TIMEOUT;
    }
    return stk_read_reg(h, kStkSbusrRd, out);
}

// ---- Device open --------------------------------------------------------

struct UsbCtx {
    libusb_context* ctx = nullptr;
    libusb_device_handle* h = nullptr;
    ~UsbCtx() {
        if (h != nullptr) {
            libusb_close(h);
        }
        if (ctx != nullptr) {
            libusb_exit(ctx);
        }
    }
};

bool is_known_id(std::uint16_t vid, std::uint16_t pid) {
    for (const auto& id : kKnownIds) {
        if (id.vid == vid && id.pid == pid) {
            return true;
        }
    }
    return false;
}

int open_device(UsbCtx& u) {
    int rc = libusb_init(&u.ctx);
    if (rc < 0) {
        std::println(stderr, "libusb_init: {}", libusb_error_name(rc));
        return rc;
    }
    libusb_device** list = nullptr;
    const ssize_t n = libusb_get_device_list(u.ctx, &list);
    if (n < 0) {
        std::println(stderr, "libusb_get_device_list: {}", libusb_error_name(int(n)));
        return int(n);
    }
    for (ssize_t i = 0; i < n; ++i) {
        libusb_device_descriptor d{};
        if (libusb_get_device_descriptor(list[i], &d) < 0) {
            continue;
        }
        if (!is_known_id(d.idVendor, d.idProduct)) {
            continue;
        }
        rc = libusb_open(list[i], &u.h);
        if (rc < 0) {
            std::println(stderr,
                         "libusb_open({:04x}:{:04x}): {} -- is WinUSB bound? "
                         "Close SurfaceMount.exe and use Zadig.",
                         d.idVendor, d.idProduct, libusb_error_name(rc));
            libusb_free_device_list(list, 1);
            return rc;
        }
        std::println("Opened {:04x}:{:04x}", d.idVendor, d.idProduct);
        break;
    }
    libusb_free_device_list(list, 1);
    if (u.h == nullptr) {
        std::println(stderr, "No STK1150/STK1160 device found on USB.");
        return LIBUSB_ERROR_NO_DEVICE;
    }
    // Vendor control transfers on endpoint 0 don't require claim_interface;
    // skip it so we don't trip over Windows handing us interface 1/2 (AC97
    // audio etc.) that we don't actually touch.
    return 0;
}

// ---- Subcommand dispatch -----------------------------------------------

int parse_u8(std::string_view s, std::uint8_t& out) {
    if (s.empty()) {
        return -1;
    }
    int base = 10;
    if (s.starts_with("0x") || s.starts_with("0X")) {
        s.remove_prefix(2);
        base = 16;
    }
    char* end = nullptr;
    const long v = std::strtol(s.data(), &end, base);
    if (end == s.data() || v < 0 || v > 0xFF) {
        return -1;
    }
    out = std::uint8_t(v);
    return 0;
}

int cmd_dump(libusb_device_handle* h, std::uint8_t addr) {
    std::println("SAA7113 register dump (I2C addr 0x{:02x}):", addr);
    std::print("     ");
    for (int c = 0; c < 16; ++c) {
        std::print(" {:02x}", c);
    }
    std::println("");
    for (int row = 0; row < 8; ++row) {
        std::print("  {:02x}:", row * 16);
        for (int col = 0; col < 16; ++col) {
            std::uint8_t v = 0;
            const int rc = saa_read(h, addr, std::uint8_t(row * 16 + col), v);
            if (rc < 0) {
                std::print(" --");
            } else {
                std::print(" {:02x}", v);
            }
        }
        std::println("");
    }
    return 0;
}

int cmd_scan(libusb_device_handle* h) {
    // Probe 7-bit I2C addresses 0x08..0x77 by attempting a read of subaddr 0
    // and checking whether the transaction completes. A device that ACKs the
    // address completes; one that NAKs times out.
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
    if (parse_u8(reg_s, reg) < 0) {
        std::println(stderr, "bad register: {}", reg_s);
        return 1;
    }
    std::uint8_t v = 0;
    const int rc = saa_read(h, addr, reg, v);
    if (rc < 0) {
        std::println(stderr, "read failed: {}", libusb_error_name(rc));
        return 1;
    }
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
    if (rc < 0) {
        std::println(stderr, "write failed: {}", libusb_error_name(rc));
        return 1;
    }
    std::uint8_t v = 0;
    if (saa_read(h, addr, reg, v) == 0) {
        std::println("0x{:02x} <- 0x{:02x}  (read-back 0x{:02x})", reg, val, v);
    } else {
        std::println("0x{:02x} <- 0x{:02x}  (read-back failed)", reg, val);
    }
    return 0;
}

int cmd_named(libusb_device_handle* h, std::uint8_t addr, std::uint8_t reg,
              std::string_view name, std::string_view val_s) {
    std::uint8_t val = 0;
    if (parse_u8(val_s, val) < 0) {
        std::println(stderr, "bad {} value: {}", name, val_s);
        return 1;
    }
    const int rc = saa_write(h, addr, reg, val);
    if (rc < 0) {
        std::println(stderr, "write failed: {}", libusb_error_name(rc));
        return 1;
    }
    std::println("{} (reg 0x{:02x}) <- 0x{:02x} ({})", name, reg, val, val);
    return 0;
}

void usage(std::string_view argv0) {
    std::println(
        "saa7113-tune -- SAA7113 register tuner via STK1160 I2C master\n"
        "\n"
        "Usage:\n"
        "  {0} [--addr 0x25] dump\n"
        "  {0} [--addr 0x25] scan\n"
        "  {0} [--addr 0x25] read  REG\n"
        "  {0} [--addr 0x25] write REG VAL\n"
        "  {0} [--addr 0x25] brightness 0..255    (reg 0x0A, default 128)\n"
        "  {0} [--addr 0x25] contrast   0..127    (reg 0x0B, default 68)\n"
        "  {0} [--addr 0x25] saturation 0..127    (reg 0x0C, default 64)\n"
        "  {0} [--addr 0x25] hue        0..255    (reg 0x0D, default 0)\n"
        "\n"
        "REG, VAL accept decimal or 0xHEX. --addr is the SAA7113 7-bit I2C\n"
        "slave address (default 0x25 / 8-bit 0x4A; SA-strapped LOW).\n"
        "\n"
        "Run with SurfaceMount.exe CLOSED, after Zadig has bound WinUSB to\n"
        "the STK1150 'USB2.0 Grabber'. See README.md for the swap procedure.",
        argv0);
}

}  // namespace

int main(int argc, char* argv[]) {
    std::uint8_t addr = kSaaDefaultAddr;

    // --addr 0xNN may appear before the subcommand.
    int argi = 1;
    if (argi + 1 < argc && std::string_view(argv[argi]) == "--addr") {
        if (parse_u8(argv[argi + 1], addr) < 0 || addr > 0x7F) {
            std::println(stderr, "bad --addr value: {}", argv[argi + 1]);
            return 1;
        }
        argi += 2;
    }
    if (argi >= argc) {
        usage(argv[0]);
        return 1;
    }
    const std::string_view cmd = argv[argi];
    const int rest = argc - argi - 1;
    char** restv = argv + argi + 1;

    UsbCtx u;
    if (open_device(u) < 0) {
        return 1;
    }

    if (cmd == "dump" && rest == 0) {
        return cmd_dump(u.h, addr);
    }
    if (cmd == "scan" && rest == 0) {
        return cmd_scan(u.h);
    }
    if (cmd == "read" && rest == 1) {
        return cmd_read(u.h, addr, restv[0]);
    }
    if (cmd == "write" && rest == 2) {
        return cmd_write(u.h, addr, restv[0], restv[1]);
    }
    if (cmd == "brightness" && rest == 1) {
        return cmd_named(u.h, addr, kSaaBrightness, "Brightness", restv[0]);
    }
    if (cmd == "contrast" && rest == 1) {
        return cmd_named(u.h, addr, kSaaContrast, "Contrast", restv[0]);
    }
    if (cmd == "saturation" && rest == 1) {
        return cmd_named(u.h, addr, kSaaSaturation, "Saturation", restv[0]);
    }
    if (cmd == "hue" && rest == 1) {
        return cmd_named(u.h, addr, kSaaHue, "Hue", restv[0]);
    }

    usage(argv[0]);
    return 1;
}
