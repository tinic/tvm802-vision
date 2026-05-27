// Shared primitives: USB control transfers, I2C-over-STK1160, device open.
// Used by main.cpp (one-shot subcommands) and live.cpp (interactive preview).

#pragma once

#include <libusb.h>

#include <cstdint>
#include <string_view>

namespace saa {

// SAA7113H 7-bit I2C slave address: 0x25 default (RTS0 internal pull-up),
// 0x24 if RTS0 strapped LOW. The QiHe TVM802B uses default unless verified.
inline constexpr std::uint8_t kSaaDefaultAddr = 0x25;

// SAA7113 control registers we name explicitly. Everything else is reachable
// via raw write/read.
inline constexpr std::uint8_t kSaaInputCntl1 = 0x02;  // FUSE+GUDL+MODE
inline constexpr std::uint8_t kSaaInputCntl2 = 0x03;  // HOLDG+GAFIX+GAI{2,1}8
inline constexpr std::uint8_t kSaaGainCh1Lo  = 0x04;  // GAI17..GAI10
inline constexpr std::uint8_t kSaaGainCh2Lo  = 0x05;  // GAI27..GAI20
inline constexpr std::uint8_t kSaaLumaCntl   = 0x09;
inline constexpr std::uint8_t kSaaBrightness = 0x0A;  // BRIG7..0, default 0x80
inline constexpr std::uint8_t kSaaContrast   = 0x0B;  // CONT7..0, default 0x47
inline constexpr std::uint8_t kSaaSaturation = 0x0C;  // SATN7..0, default 0x40
inline constexpr std::uint8_t kSaaHue        = 0x0D;  // HUEC7..0, default 0x00

// STK1160 USB / framing constants (datasheet Rev 1.2 + Linux stk1160 driver).
inline constexpr std::uint8_t  kStkEpVideo    = 0x82;
inline constexpr std::uint16_t kStkMinPktSize = 3072;

struct UsbCtx {
    libusb_context* ctx = nullptr;
    libusb_device_handle* h = nullptr;
    ~UsbCtx();
    UsbCtx() = default;
    UsbCtx(const UsbCtx&) = delete;
    UsbCtx& operator=(const UsbCtx&) = delete;
};

int open_device(UsbCtx& u);

// Generic STK1160 register byte read/write (vendor control transfers).
int stk_write_reg(libusb_device_handle* h, std::uint16_t reg, std::uint8_t val);
int stk_read_reg(libusb_device_handle* h, std::uint16_t reg, std::uint8_t& out);

// I2C transaction over the STK1160 I2C master. addr is 7-bit.
int saa_write(libusb_device_handle* h, std::uint8_t addr, std::uint8_t reg, std::uint8_t val);
int saa_read(libusb_device_handle* h, std::uint8_t addr, std::uint8_t reg, std::uint8_t& out);

// Parse "0xNN" / "NNN" as an 8-bit unsigned. Returns 0 on success, -1 on bad input.
int parse_u8(std::string_view s, std::uint8_t& out);

}  // namespace saa
