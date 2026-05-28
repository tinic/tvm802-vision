# CLAUDE.md — developer and architecture notes

This file holds the developer-facing material that used to clutter `README.md`. It is aimed at contributors and at AI agents touching the code. If you are an end user, you can safely ignore it.

## Architecture

The DLL is a **passthrough shim**. `MVision.dll` exports the original DLL's full ABI, but it only overrides the down-vision mark paths (`CheckMark2`, `CheckTemplate`, `CheckMark`), their preview, and `CheckComp` (up-vision component detection). Every other call is forwarded to the original DLL, which the installer renames to `MVision-orig.dll`.

`mvision_grabber.ax` is a usermode DirectShow capture filter. It registers under the same friendly name (`"USB2.0 Grabber"`) that SurfaceMount looks for through `DsDevice.GetDevicesOfCat`, so the host opens it transparently in place of the stock driver. The filter streams UYVY 640×480 frames from our own WinUSB plus STK1160 I²C-master code, and it exposes an `IAMVideoProcAmp` interface backed by real SAA7113 register writes. By contrast, the stock Syntek driver fakes `IAMVideoProcAmp` in software, and on this hardware family those calls never reach the chip.

`saa7113-tune.exe` is the standalone twin of the filter. It uses the same WinUSB primitives, adds a live-preview Win32 window and a small REPL, and is meant for diagnostic work outside SurfaceMount.

## Status

- **Down-vision**: all three mark modes (Round, Circular, ImageTemplate) work, are field-aware, and have been validated on a Gen-2 TVM802B in real production.
- **Up-vision component detection** (`CheckComp`): drives placement and has been validated on hardware. It uses rectilinear-symmetry with a minimum-area-rect fallback. The pose-to-host transform has been calibrated over hundreds of real reads. The detector is field-aware: it uses the newest single field while the head is moving, and a woven full-resolution frame once motion settles. It renders its own oriented overlay. Per-detector on/off checkboxes can fall any one method back to the stock vision.
- **Missing-part reject**: the detector signals not-found by reporting a zero component size. The host's existing retry and controlled-stop path takes care of the rest.
- **Camera tab and chip control**: per-mode SAA7113 brightness, contrast, gain, AGC, sharpness, and prefilter over WinUSB I²C, persisted to `MVision.ini`. This feature requires the `.ax` filter and a one-time Zadig WinUSB binding per machine.

## Build (from source)

The DLL is an x86 build under MSVC, because it has to match the i386 host process.

```
cmake -S . -B build -A Win32 ^
    -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake ^
    -DVCPKG_TARGET_TRIPLET=x86-windows-static ^
    -DBUILD_SAA7113_TUNE=ON ^
    -DBUILD_MVISION_GRABBER=ON
cmake --build build --config Release
```

OpenCV and libusb come from `vcpkg.json` in manifest mode. The release zip is built by `.github/workflows/build.yml` and contains all four artifacts along with the install scripts.

On a Linux development box you can do host-architecture validation only. `mvision-tune` builds and runs, but the DLL is skipped.

```
sudo apt install libusb-1.0-0-dev libopencv-dev
git submodule update --init --recursive   # constixel, for mvision-tune sixel
cmake -S . -B build -DBUILD_SAA7113_TUNE=ON
cmake --build build -j
```

`tools/check.sh` runs the full lint gate: clang-format, cppcheck, clang-tidy, host build, and ctest. The normal iteration loop on the live machine is: edit, scp the changed files to winbuilder, rebuild with NMake through `build-winbuilder.bat`, then scp the resulting DLL to the ShopPC. Do not run `tools/check.sh` in the inner loop; it is slow and is only needed at commit time.

## TVM802BX (built-in Atom N2800)

The BX's embedded computer uses an Atom N2800 (Cedar Trail / Saltwell, 2012). It supports SSSE3 only, with no AVX2, no SSE4, and no FMA. You must use `MVision-noavx2.dll`, which is the SSE2 baseline build; rename it to `MVision.dll` after install. If you accidentally drop the AVX2 build onto a BX, the DLL self-checks at startup and pops up a modal warning that points you at the no-AVX2 build, rather than faulting with an illegal-instruction exception.

The N2800 is also slow and memory-bandwidth-limited. On the BX, keep the host's **Range** setting small, and prefer **Circular** over **Round** to keep the detector light. SSE4 gave no measurable speedup for this gather-bound detector; only AVX2's FMA helped, and the Atom does not have it.

## CompThre tuning table (host-side, ParamEdit)

The per-stack **视觉阈值 (CompThre)** is the operator's main up-vision tuning knob. It is a **percentage (0–100) of the ROI's maximum brightness**, not an absolute gray level. Set it per feeder stack in the host's ParamEdit dialog, and **save the placement CSV** afterwards. The CSV carries the per-slot values, and loading a CSV overwrites whatever was in memory.

| Part class | CompThre | Why |
|---|---|---|
| 0402, 0603, 0805 chip caps and resistors | **50** | Symmetry catches the body cleanly, and the exact value is not critical. |
| Self-emitting LEDs (WS28xx-style, with a halo around the body) | **60–70** | A higher value keeps the fallback's catch small (the body core only), so the symmetry path's clean body-edge result wins and stays stable to sub-pixel precision. |
| SOT-23-3 and SOT-23-6 leaded ICs | **25–30** | A low value catches every dim pin tip; the fallback then unions them into the full body bounding box. |
| SOIC, QFN, and larger leaded ICs | **30–40** | Catches the pin tips plus the partially visible body. |
| Tantalum caps and electrolytics (bright tops) | **45–55** | The body is bright, so symmetry handles it cleanly. |
| Shielded inductors (with reflective sides) | **40–60** | Start in the middle. Lower the value if the body reads dim, raise it if the sides glare. |
| Default when unsure | **50** | A sensible middle ground. |

The detector itself does not branch on part class; there is a single generic algorithm. Values 0 through 9 are reserved for future "special profile" overrides.

## Ctrl+Alt+U — one-shot up-cam snapshot

A second global hotkey arms a single up-camera capture for the next `CheckComp` read. When you press it, the next time the host triggers an up-vision read the DLL writes:

- `C:\mvision_capture\snap_NNNN.png`: the raw full-frame capture (written every time).
- `C:\mvision_capture\snap_overlay_NNNN.png`: the rendered overlay, written whenever the Component detector checkbox is on (the default).

This is independent of the bulk-capture triggers (`on` and `frames`). One press produces one capture. It pairs naturally with `mvision-tune` below.

## `mvision-tune` — offline component tuner

A standalone host-side tool that lets you iterate on a part class against a captured snapshot, without having to redeploy the DLL. It reads a PNG, runs the production `detect_component`, renders the green-box, arrow, and cross overlay, and prints it inline in your terminal using sixel, kitty, or iTerm2 graphics (auto-detected).

The tool is part of the standard CMake configuration. It is built by default whenever `third_party/constixel` is checked out:

```
git submodule update --init --recursive
cmake -S . -B build && cmake --build build -j
```

Single-frame usage:

```
mvision-tune --thr 50 captures/comp_0042.png
mvision-tune --thr 30 --w 50 --h 80 frame.png      # size prior
mvision-tune --proto none frame.png                # write the PNG without printing
```

For corpus stability across many frames:

```
mvision-tune --thr 50 --corpus captures/0603/
```

This prints per-frame CSV to stdout and a summary with σ to stderr. The `angle` column uses circular statistics, because the chip normalises angle to (-45°, 45]: readings at +44° and -44° are 2° apart modulo 90°, not 88°.

A sample corpus lives in `tests/corpus/` (see `tests/corpus/README.md` for the class layout). The same corpus is exercised by `tests/smoke_comp.sh`.

## Code layout

- `src/dllmain.cpp` — DLL entry point and ABI exports.
- `src/passthrough.cpp` — forwarding shims for everything we do not override.
- `src/capture.cpp` — host-side frame plumbing and the capture sentinels.
- `src/detect_*.cpp` — per-mode detectors. Their tunable constants are at the top of each file.
- `src/detect_common.cpp` — the shared field/frame deinterlace and the image-adjustment LUT.
- `src/preview.cpp` — overlay rendering through GDI.
- `src/controller.cpp` — the pixel-to-millimeter scale read from the motion controller over TCP.
- `src/settings.cpp` and `settings.h` — per-mode `Settings` and INI persistence.
- `src/settings_ui.cpp` — the Ctrl+Alt+M dialog.
- `src/camera_chip.cpp` — the SAA7113 hardware control surface used by the Camera tab.
- `tools/saa7113-tune/` — the standalone diagnostic tuner. `common.cpp` houses the shared WinUSB and STK1160 I²C-master primitives, which are linked into both the DLL (Camera tab) and the grabber.
- `tools/mvision-grabber/` — the DirectShow source filter (`.ax`).
- `tests/` — host-architecture test harness, corpus, and `mvision-tune`.

## Caveats

- The code was developed on a Gen-2 802B. The capture chain is analog cameras into a CD4052 mux into a Syntek STK1150 USB capture board, delivering 640×480 grayscale. Different capture hardware may behave differently.
- The fiducial target is the standard 1 mm copper pad. A very different fiducial *shape* (as opposed to size) would need a code-level change.
- The coordinate conventions (down-mirror, sub-pixel crop, offset signs) were recovered by reverse-engineering against the original DLL.

## Pointers

- `tools/saa7113-tune/SYNTEK_RE.md` — vendor-driver reverse-engineering notes. Gitignored and private.
- `~/.claude/projects/-home-turo/memory/` — long-form context for AI agents touching this repo. `MEMORY.md` is the index.
