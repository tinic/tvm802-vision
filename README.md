# tvm802-vision — OpenCV down-vision fiducial detector for the QiHe TVM802A / TVM802B

A drop-in replacement for the QiHe **TVM802A / TVM802B** pick-and-place's
`MVision.dll` that replaces the stock **down-vision (board fiducial) detection**
with a modern OpenCV pipeline. The stock vision is jittery, drops lock, and is
slow; this gives a sub-pixel-stable, motion-robust fiducial lock — reliable even
at high head speed.

> Developed and tested on a **TVM802B**. The TVM802A shares the same vision DLL
> and analog capture path, so this should apply to it as well (the 802A is the
> more common model) — but it is currently **untested on the 802A**; reports
> welcome. You may need to retune the radius bracket / mark size for your board's
> fiducials (see *Tuning*).

> **Not affiliated with QiHe.** This is an independent, interoperability-focused
> reimplementation. **You must supply your own original `MVision.dll`** — no
> vendor binaries are included or redistributed here.

## What it does

- Locks the **1 mm copper fiducial pad** with a tight Hough radius bracket, so it
  ignores the larger concentric solder-mask ring that made the stock detection
  "jitter" (the detector was flip-flopping between the two circles).
- **Deinterlaces** the analog capture and detects on *both* fields independently
  — each field is a single instant in time, so a moving target no longer combs
  into a "double image". Robust at high job speed.
- **Bilateral denoise before CLAHE** → cleans analog sensor noise without
  softening the copper edge, for a sub-pixel center.
- **Averages both fields when settled** → recovers full vertical resolution and
  cancels the per-field bias, for sub-pixel placement accuracy.
- Renders its **own 1:1 preview overlay** (crisp reference crosshair, search
  circle, and green detection marker).

## How it works

It's a **passthrough shim**. It exports the same ABI as the original
`MVision.dll`, forwards every call it doesn't change to the original (which you
rename to `MVision-orig.dll`), and overrides only the down-vision fiducial path
(`CheckMark2` + its preview). `GetOffset`/`GetMin_val` return our result for that
mode; everything else (up-vision, nozzle, component checks, etc.) passes straight
through to the original, unchanged.

## Requirements

- **32-bit (x86)** build — it must match the host process.
- MSVC (Visual Studio Build Tools) and CMake.
- OpenCV 4.x, x86 **static**, via vcpkg:
  `vcpkg install opencv4:x86-windows-static`
- Your **original `MVision.dll`** from your machine.

## Build

1. Install OpenCV: `vcpkg install opencv4:x86-windows-static`
2. Edit `build-cmake.bat` so `OpenCV_DIR` points at your vcpkg install
   (e.g. `C:/vcpkg/installed/x86-windows-static/share/opencv4`).
3. Run `build-cmake.bat` from an x86 toolchain environment.
   Output: `build\MVision.dll`.

(Or configure CMake directly with the x86 MSVC toolchain and `-DOpenCV_DIR=...`.)

## Install

With the SurfaceMount application **closed**:

1. Back up your original `MVision.dll`.
2. Rename the original to **`MVision-orig.dll`** (same folder).
3. Drop in the freshly built `MVision.dll`.
4. Launch the app — down-vision fiducial detection now uses this pipeline.

## Tuning (`src/vision.cpp`)

- **Radius bracket** `0.22–0.42 × size` (the host's mark "size" argument). Keep
  the app's mark-size setting near its default so the bracket tracks the copper
  pad's pixel radius.
- **Gates**: circularity, contrast, and search-area constants live near the top
  of `detect_one_field`.
- **Deinterlace / averaging**: field selection and the settled-averaging
  threshold are in `detect_circle_mark`.

## Caveats

- Developed on a **Gen-2 802B**: two analog cameras → CD4052 mux → Syntek
  STK1150 USB capture, 640×480 grayscale. Other capture hardware (e.g. some
  802A revisions) may behave differently.
- The fiducial target is the **1 mm copper pad**; the detector is tuned for that.
  A different fiducial size needs a radius-bracket / mark-size adjustment.
- The coordinate convention (180° down-mirror, sub-pixel crop, offset signs) was
  recovered by reverse engineering and matched to the original DLL's behavior.

## License

MIT — see [LICENSE](LICENSE).
