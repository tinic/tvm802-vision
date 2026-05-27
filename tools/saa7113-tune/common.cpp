#include "common.h"

#include <array>
#include <chrono>
#include <cstdlib>
#include <print>
#include <thread>

namespace saa {

namespace {

// STK1160 I2C-master registers (datasheet §8.4.1; offsets within the 32-bit
// SICTL/SBUSW/SBUSR registers at 0x200/0x204/0x208).
constexpr std::uint16_t kStkSictl    = 0x200;
constexpr std::uint16_t kStkSictlSt  = 0x201;  // status byte (poll)
constexpr std::uint16_t kStkSictlSda = 0x203;  // slave address (addr<<1)
constexpr std::uint16_t kStkSbuswWa  = 0x204;  // write-path subaddress
constexpr std::uint16_t kStkSbuswWd  = 0x205;  // write-path data
constexpr std::uint16_t kStkSbusrRa  = 0x208;  // read-path subaddress
constexpr std::uint16_t kStkSbusrRd  = 0x209;  // read-path data

constexpr std::uint8_t  kStkKickWrite = 0x01;
constexpr std::uint8_t  kStkKickRead  = 0x20;
constexpr std::uint8_t  kStkDoneWrite = 0x04;  // WF -- bit 10 of SICTL
constexpr std::uint8_t  kStkDoneRead  = 0x01;  // RF -- bit 8 of SICTL
constexpr std::uint8_t  kStkFailWrite = 0x08;  // FWDA -- bit 11 of SICTL
constexpr std::uint8_t  kStkFailRead  = 0x02;  // FRDA -- bit 9 of SICTL

constexpr unsigned int  kPollTimeoutMs = 100;
constexpr unsigned int  kCtrlTimeoutMs = 1000;

struct UsbId { std::uint16_t vid; std::uint16_t pid; };
constexpr std::array kKnownIds = std::to_array<UsbId>({
    {0x05e1, 0x0408},  // TVM802B
    {0x05e1, 0x0400},
    {0x05e1, 0x04f3},
    {0x05e1, 0x04f4},
});

bool is_known_id(std::uint16_t vid, std::uint16_t pid) {
    for (const auto& id : kKnownIds) {
        if (id.vid == vid && id.pid == pid) {
            return true;
        }
    }
    return false;
}

// Returns 0 on clean success, 1 on NAK (no ACK from the slave), -1 on
// timeout/USB error. Critically: the kick command (writing byte 0x200)
// clears status bits, but NOT synchronously -- there's a window where the
// chip is starting the transaction and old RF/WF may still read as set.
// Without an initial delay, polling sees the leftover bit, returns "done"
// instantly, and we end up reading garbage (the previous transaction's
// data still parked in 0x209). 5ms is well above I2C @ 93kHz worst-case
// transaction time (~0.3ms), so we never time out a real transaction.
int i2c_busy_wait(libusb_device_handle* h, std::uint8_t done_bit, std::uint8_t fail_bit) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(kPollTimeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        std::uint8_t flag = 0;
        if (stk_read_reg(h, kStkSictlSt, flag) < 0) {
            return -1;
        }
        if ((flag & fail_bit) != 0) {
            return 1;  // NAK
        }
        if ((flag & done_bit) != 0) {
            return 0;  // success
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return -1;
}

}  // namespace

UsbCtx::~UsbCtx() {
    if (h != nullptr) {
        libusb_close(h);
    }
    if (inited) {
        libusb_exit(nullptr);
    }
}

int stk_write_reg(libusb_device_handle* h, std::uint16_t reg, std::uint8_t val) {
    return libusb_control_transfer(h, 0x40, 0x01, val, reg, nullptr, 0, kCtrlTimeoutMs);
}

int stk_read_reg(libusb_device_handle* h, std::uint16_t reg, std::uint8_t& out) {
    out = 0;
    return libusb_control_transfer(h, 0xC0, 0x00, 0, reg, &out, 1, kCtrlTimeoutMs);
}

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
    int r = i2c_busy_wait(h, kStkDoneWrite, kStkFailWrite);
    if (r < 0) return LIBUSB_ERROR_TIMEOUT;
    if (r == 1) return LIBUSB_ERROR_IO;  // NAK
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
    int r = i2c_busy_wait(h, kStkDoneRead, kStkFailRead);
    if (r < 0) return LIBUSB_ERROR_TIMEOUT;
    if (r == 1) return LIBUSB_ERROR_IO;  // NAK
    return stk_read_reg(h, kStkSbusrRd, out);
}

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

int open_device(UsbCtx& u) {
    int rc = libusb_init(nullptr);
    if (rc < 0) {
        std::println(stderr, "libusb_init: {}", libusb_error_name(rc));
        return rc;
    }
    u.inited = true;
    libusb_device** list = nullptr;
    const ssize_t n = libusb_get_device_list(nullptr, &list);
    if (n < 0) {
        std::println(stderr, "libusb_get_device_list: {}", libusb_error_name(int(n)));
        return int(n);
    }
    // On Windows with a composite WinUSB binding (one INF per interface),
    // libusb sees TWO devices with the same VID:PID -- one for interface 0
    // (video, isoc EP 0x82) and one for interface 1 (audio). For our work
    // we want the video handle; identify it by the presence of EP 0x82 in
    // the config descriptor. First handle that has it wins. Vendor control
    // transfers work on either, but live preview needs isoc, which is on
    // the video side only.
    int firstError = 0;
    for (ssize_t i = 0; i < n; ++i) {
        libusb_device_descriptor d{};
        if (libusb_get_device_descriptor(list[i], &d) < 0) {
            continue;
        }
        if (!is_known_id(d.idVendor, d.idProduct)) {
            continue;
        }
        libusb_device_handle* h = nullptr;
        rc = libusb_open(list[i], &h);
        if (rc < 0) {
            if (firstError == 0) firstError = rc;
            continue;
        }
        // Look for the video EP in this handle's config descriptor.
        libusb_config_descriptor* cfg = nullptr;
        bool isVideo = false;
        if (libusb_get_active_config_descriptor(list[i], &cfg) == 0 && cfg != nullptr) {
            for (int ii = 0; ii < cfg->bNumInterfaces && !isVideo; ++ii) {
                const auto& intf = cfg->interface[ii];
                for (int a = 0; a < intf.num_altsetting && !isVideo; ++a) {
                    const auto& alt = intf.altsetting[a];
                    for (int e = 0; e < alt.bNumEndpoints; ++e) {
                        if (alt.endpoint[e].bEndpointAddress == kStkEpVideo) {
                            isVideo = true;
                            break;
                        }
                    }
                }
            }
            libusb_free_config_descriptor(cfg);
        }
        if (isVideo) {
            u.h = h;
            std::println("Opened {:04x}:{:04x}  (video interface)", d.idVendor, d.idProduct);
            break;
        }
        // Wrong handle (audio); release and try next.
        libusb_close(h);
    }
    if (u.h == nullptr && firstError != 0) {
        std::println(stderr,
                     "libusb_open: {} -- is WinUSB bound? Close SurfaceMount.exe and use Zadig.",
                     libusb_error_name(firstError));
        libusb_free_device_list(list, 1);
        return firstError;
    }
    libusb_free_device_list(list, 1);
    if (u.h == nullptr) {
        std::println(stderr, "No STK1150/STK1160 device found on USB.");
        return LIBUSB_ERROR_NO_DEVICE;
    }
    // Minimum prep so I2C transactions actually work after a cold USB
    // enumerate: write the SICTL clock divider (byte 0x202) so the I2C
    // bus runs at ~93 kHz instead of whatever CD=0 produces (15 MHz, no
    // chip will ACK). Datasheet §8.4.1: clock = 30 MHz / (CD * 16 + 2).
    // 0x14 is the documented default when an EEPROM is strapped.
    stk_write_reg(u.h, 0x202, 0x14);
    return 0;
}

}  // namespace saa
