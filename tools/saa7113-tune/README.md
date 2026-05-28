# saa7113-tune

A standalone WinUSB tuner for the SAA7113 analog video decoder on the QiHe TVM802B's USB capture board. It writes the chip's brightness, contrast, gain, AGC, sharpness, and similar controls directly through STK1160 I²C-master vendor USB control transfers. This bypasses the stock Syntek driver, which fakes these controls in software and never actually reaches the chip on this hardware family.

The tool is useful for diagnostic work outside SurfaceMount, for scripting iteration on chip registers, and for visual tuning through a live preview window. The same primitives are linked into MVision.dll's **Camera tab** (Ctrl+Alt+M) and into `mvision_grabber.ax`.

## Setup (once per machine)

1. Install [Zadig](https://zadig.akeo.ie/).
2. Close `SurfaceMount.exe`.
3. In Zadig, open *Options → List All Devices*. For **both** `USB2.0 Grabber (Interface 0)` and `(Interface 1)`, select the entry, set the target driver to **libusbK**, and click **Replace Driver**.

You can also just run `install.bat` from the release bundle, which prints the same steps.

## Usage

```
saa7113-tune live              live preview window + REPL (type 'help' inside)
saa7113-tune dump              full register dump
saa7113-tune scan              I²C bus scan
saa7113-tune brightness 144    one-shot setter
saa7113-tune write REG VAL     raw write (REG and VAL accept hex or decimal)
saa7113-tune --addr 0xNN ...   override the default 7-bit slave address (0x25)
```

Inside `live` mode you have a small REPL: `b`, `c`, `s`, `h`, `g` set brightness, contrast, saturation, hue, and gain; `agc on|off` toggles AGC; `i 0..3` selects the input; `w` and `r` do raw register writes and reads; `d` dumps all registers; `hist` prints a Y-channel histogram; `q` quits; and `help` lists everything. Pass `--debug` to print a one-line isoc-transfer statistics report every second.

## Build

The tool builds as part of the main CMake configuration when you pass `-DBUILD_SAA7113_TUNE=ON`. See `CLAUDE.md` for the full build instructions and the shared-code architecture.
