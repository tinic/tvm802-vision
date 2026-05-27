# saa7113-tune

One-shot tuning utility for the SAA7113 analog video decoder on the QiHe
TVM802B's USB capture board. Writes the SAA7113's Brightness / Contrast /
Saturation / Hue / gain registers **in hardware**, bypassing the stock
Syntek Windows driver entirely.

## Why this exists

The stock Syntek driver (`stk1160.inf`, family `SyntekM811`) was built for
**CMOS-Bayer sensor boards**, not the SAA7113 analog decoder on this
hardware. It still binds to the same USB ID and forwards a YUYV stream, but
every IAMVideoProcAmp call (Brightness, Contrast, Gain, ...) silently
substitutes software post-processing on the 8-bit buffer -- the SAA7113
itself never sees the values.

This utility talks to the STK1160's built-in I²C master via vendor USB
control transfers and writes SAA7113 registers directly. Values persist in
the SAA7113 across the driver swap (the chip stays powered via USB Vbus),
so tune-once-and-forget is the intended workflow.

## One-time setup per machine

1. Install [Zadig](https://zadig.akeo.ie/).
2. Close `SurfaceMount.exe`.
3. In Zadig: *Options → List All Devices*, pick **"USB2.0 Grabber"**
   (interface 0; VID `05E1` PID `0408`), pick **WinUSB** in the target
   driver dropdown, click *Replace Driver*.

## Tuning loop (recommended: live mode)

```
saa7113-tune live
```

Opens a Win32 window showing the live UYVY stream from the SAA7113/STK1160
over the WinUSB binding. A console REPL accepts tweaks; the effect is
visible immediately in the preview window. No driver-swap-per-tweak needed:

```
saa7113-tune live
> b 144         brightness
> c 80          contrast
> g 32          manual gain ch1+ch2 (also disables AGC)
> agc on        AGC back on
> w 0x09 0x40   raw LCR write
> d             dump all registers
> q             quit
```

When you have values you like, **write them down** -- they're volatile and
the chip resets to its 24Cxx EEPROM-loaded defaults on USB plug-cycle (and
the Syntek driver doesn't re-apply our values when you swap back). One
practical workflow: save the working register set as a small `.bat`:

```
saa7113-tune brightness 144
saa7113-tune contrast 80
saa7113-tune gain 32
```

and run that batch after each cold boot / plug event.

## One-shot subcommands (no preview)

```
saa7113-tune dump
saa7113-tune brightness 144
saa7113-tune contrast 80
saa7113-tune write 0x09 0x40
```

To verify the effect of one-shot writes without `live`: swap Zadig back to
the Syntek driver and relaunch SurfaceMount.

Once you've found values you like, write them down -- they're in the chip's
volatile registers and reset when USB Vbus drops. (Plug-cycle or full host
power cycle will erase them; the Linux stk1160 driver re-applies its own
defaults on every open, which is what happens under the Syntek driver too,
but at least we now know exactly what defaults the chip has.)

## Subcommands

| Command | Meaning |
|---|---|
| `live` | **Open live preview window + REPL** (Windows only). Type `help` inside it for REPL commands. |
| `dump` | Read all 128 SAA7113 registers, print as a hex matrix |
| `scan` | I²C bus scan (7-bit addresses 0x08..0x77) -- finds the chip |
| `read REG` | Read one SAA7113 register (REG is decimal or `0xNN`) |
| `write REG VAL` | Write one SAA7113 register, with read-back confirmation |
| `brightness 0..255` | Write SAA7113 register 0x0A |
| `contrast 0..127` | Write SAA7113 register 0x0B |
| `saturation 0..127` | Write SAA7113 register 0x0C |
| `hue 0..255` | Write SAA7113 register 0x0D |

The `--addr 0xNN` flag before any subcommand overrides the I²C slave
address. Default is `0x25` (7-bit, = 8-bit `0x4A`, SA pin LOW). The other
documented strap value is `0x24` (SA HIGH).

For gain, AGC and any other control, consult the SAA7113H datasheet and use
`write`. The pertinent registers are:

| Reg | Name | Notes |
|---|---|---|
| 0x02 | AICO1 | analog input select (CVBS source) + AGC fixed-gain bits |
| 0x05 | GAI28..21 | manual analog gain channel 1 |
| 0x06 | GAI18..11 | manual analog gain channel 2 |
| 0x09 | LCR | luminance control; prefilter, anti-alias, AGC hold |
| 0x0E | CHCV | chroma control 1 |
| 0x0F | CGC | chroma gain control |

## Build

From the repo root:

```
cmake -S . -B build-tools -DBUILD_SAA7113_TUNE=ON \
    -DCMAKE_TOOLCHAIN_FILE=/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build-tools --target saa7113-tune --config Release
```

On a Linux dev box (for protocol experimentation; the TVM802B's actual
board is on Windows):

```
sudo apt install libusb-1.0-0-dev
cmake -S . -B build-tools -DBUILD_SAA7113_TUNE=ON
cmake --build build-tools --target saa7113-tune
```

## Protocol reference

The full STK1160 + SAA7113 I²C transaction sequences are documented inline
in `main.cpp`. They are derived from `drivers/media/usb/stk1160/` in the
mainline Linux kernel (GPL-2.0) and the SAA7113H datasheet.
