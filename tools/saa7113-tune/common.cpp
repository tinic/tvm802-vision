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
constexpr std::uint8_t  kStkDoneWrite = 0x04;
constexpr std::uint8_t  kStkDoneRead  = 0x01;

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

}  // namespace

UsbCtx::~UsbCtx() {
    if (h != nullptr) {
        libusb_close(h);
    }
    if (ctx != nullptr) {
        libusb_exit(ctx);
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
    return 0;
}

}  // namespace saa
