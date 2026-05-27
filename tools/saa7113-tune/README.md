# saa7113-tune

Standalone WinUSB tuner for the SAA7113 analog video decoder on the QiHe TVM802B's USB capture board. Writes the chip's Brightness / Contrast / Gain / AGC / sharpness / etc. directly via STK1160 I²C-master vendor USB control transfers — bypasses the stock Syntek driver, which software-fakes these controls and never actually reaches the chip on this hardware family.

Use cases: diagnostic work outside SurfaceMount, scripted iteration on chip registers, live-preview (Win32 window) for visual tuning. The same primitives are linked into MVision.dll's **Camera tab** (Ctrl+Alt+M) and `mvision_grabber.ax`.

## Setup (once per machine)

1. Install [Zadig](https://zadig.akeo.ie/).
2. Close `SurfaceMount.exe`.
3. Zadig → *Options → List All Devices*; for BOTH `USB2.0 Grabber (Interface 0)` and `(Interface 1)`: pick → target driver = **libusbK** → **Replace Driver**.

(Or just run `install.bat` from the release bundle, which prints the same steps.)

## Usage

```
saa7113-tune live              live preview window + REPL (type 'help' inside)
saa7113-tune dump              full register dump
saa7113-tune scan              I²C bus scan
saa7113-tune brightness 144    one-shot setter
saa7113-tune write REG VAL     raw write (REG/VAL hex or decimal)
saa7113-tune --addr 0xNN ...   override default 7-bit slave address (0x25)
```

REPL inside `live` mode: `b/c/s/h/g` (brightness/contrast/sat/hue/gain), `agc on|off`, `i 0..3` (input select), `w/r REG VAL` (raw), `d` (dump), `hist` (Y-channel histogram), `q` (quit), `help`. See `--debug` for the 1 Hz isoc stats line.

## Build

Built as part of the main cmake config when `-DBUILD_SAA7113_TUNE=ON`. See `CLAUDE.md` for the full build instructions and shared-code architecture.
