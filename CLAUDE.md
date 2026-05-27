# CLAUDE.md — developer + architecture notes

Everything that used to clutter `README.md` lives here. Aimed at contributors and AI agents touching the code; ignore if you're an end-user.

## Architecture

A **passthrough shim**: `MVision.dll` exports the original DLL's ABI, overrides only the down-vision mark paths (`CheckMark2`, `CheckTemplate`, `CheckMark`) + their preview + `CheckComp` (up-vision component), and forwards everything else to the original (renamed to `MVision-orig.dll`).

`mvision_grabber.ax` is a usermode DirectShow capture filter that drops in transparently as `"USB2.0 Grabber"` (the friendly name SurfaceMount string-matches on via `DsDevice.GetDevicesOfCat`). Streams UYVY 640×480 sourced from our own WinUSB + STK1160 I²C-master code; exposes `IAMVideoProcAmp` wired to real SAA7113 register writes (vs the stock Syntek driver where IAMVideoProcAmp is software-faked and never reaches the chip on this hardware family).

`saa7113-tune.exe` is the standalone twin: same WinUSB primitives, live-preview Win32 window + REPL for diagnostic work outside SurfaceMount.

## Status

- **Down-vision** — all three mark modes (Round / Circular / ImageTemplate), field-aware, hardware-validated on Gen-2 TVM802B.
- **Up-vision component detection** (`CheckComp`) — drives placement, validated on hardware. Rectilinear-symmetry with min-area-rect fallback; pose-to-host transform calibrated over hundreds of real reads. Field-aware (newest single field under motion; woven full-res when settled). Renders its own oriented overlay. Per-detector on/off checkboxes fall any one method back to stock vision.
- **Missing-part reject** — detector signals not-found via zero component-size; host's own retry / controlled-stop path handles it.
- **Camera tab + chip control** — per-mode SAA7113 Brightness / Contrast / Gain / AGC / Sharpness / Prefilter via WinUSB I²C, persisted to `MVision.ini`. Requires the `.ax` + a one-time Zadig WinUSB binding per machine.

## Build (from source)

x86 MSVC; must match the i386 host process.

```
cmake -S . -B build -A Win32 ^
    -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake ^
    -DVCPKG_TARGET_TRIPLET=x86-windows-static ^
    -DBUILD_SAA7113_TUNE=ON ^
    -DBUILD_MVISION_GRABBER=ON
cmake --build build --config Release
```

OpenCV + libusb come from `vcpkg.json` (manifest mode). The release zip is built by `.github/workflows/build.yml` and contains all four artifacts + the install scripts.

On a Linux dev box (host-arch validation only, `mvision-tune` builds but the DLL is skipped):

```
sudo apt install libusb-1.0-0-dev libopencv-dev
git submodule update --init --recursive   # constixel, for mvision-tune sixel
cmake -S . -B build -DBUILD_SAA7113_TUNE=ON
cmake --build build -j
```

`tools/check.sh` runs the lint gate (clang-format + cppcheck + clang-tidy + host build + ctest). Iteration loop on the live machine: edit, scp the changed file(s) to winbuilder, NMake-rebuild via `build-winbuilder.bat`, scp the `.dll` to the ShopPC. Don't run `tools/check.sh` in the inner loop — it's slow and only gates commits.

## TVM802BX (built-in Atom N2800)

The BX's embedded computer (Atom N2800 — Cedar Trail / Saltwell, 2012) has **no AVX2 / SSE4 / FMA** — only SSSE3. Must use `MVision-noavx2.dll` (SSE2 baseline; rename to `MVision.dll`). The AVX2 build self-checks at startup on a non-AVX2 CPU and pops a modal warning pointing to the no-AVX2 build instead of faulting.

That N2800 is also slow + memory-bandwidth-limited. On the BX keep the host **Range** small and prefer **Circular** over **Round** to keep the detector light. SSE4 measured *no* speedup for this gather-bound detector anyway — only AVX2's FMA helped, which the Atom lacks.

## CompThre tuning table (host-side, ParamEdit)

The per-stack **视觉阈值 (CompThre)** is the operator's main up-vision tuning knob — a **percentage (0–100) of the ROI's max brightness**, not an absolute gray level. Set per-feeder-stack in the host's ParamEdit dialog and **save the placement CSV** afterwards (the CSV carries the per-slot values; loading a CSV overwrites them).

| Part class | CompThre | Why |
|---|---|---|
| 0402 / 0603 / 0805 chip caps & resistors | **50** | Symmetry catches the body cleanly; value isn't critical. |
| Self-emitting LEDs (WS28xx-style, halo around body) | **60–70** | Higher value keeps the fallback's catch small (body core only) so symmetry's clean body-edge result wins and stays sub-pixel stable. |
| SOT-23-3 / SOT-23-6 leaded ICs | **25–30** | Low value catches every dim pin tip; fallback's union spans the full body bounding box. |
| SOIC / QFN / larger leaded ICs | **30–40** | Pin tips plus partial body visibility. |
| Tantalum caps / electrolytics (bright top) | **45–55** | Body is bright; symmetry handles cleanly. |
| Shielded inductors (reflective sides) | **40–60** | Try mid-range; lower if the body reads dim, higher if the sides glare. |
| Default when unsure | **50** | Sensible middle ground. |

The detector itself has **no per-part-class branching** — there's a single generic algorithm. Values 0–9 are reserved for future "special profile" overrides.

## Ctrl+Alt+U — one-shot up-cam snapshot

Second global hotkey arms a single up-cam capture for the next `CheckComp` read. On press, the next time the host triggers an up-vision read the DLL writes:

- `C:\mvision_capture\snap_NNNN.png` — raw full-frame capture (always).
- `C:\mvision_capture\snap_overlay_NNNN.png` — rendered overlay, when the Component detector checkbox is on (default).

Independent of the bulk-capture triggers (`on` / `frames`); one press = one capture. Pair with `mvision-tune` below.

## `mvision-tune` — offline component tuner

Standalone host-side tool for iterating on a part class against a captured snapshot, without redeploying the DLL. Reads a PNG, runs the production `detect_component`, renders the green-box / arrow / cross overlay, prints it inline in your terminal via sixel / kitty / iTerm2 (auto-detected).

Build (already part of the standard cmake config; built by default if `third_party/constixel` is checked out):

```
git submodule update --init --recursive
cmake -S . -B build && cmake --build build -j
```

Single-frame:

```
mvision-tune --thr 50 captures/comp_0042.png
mvision-tune --thr 30 --w 50 --h 80 frame.png      # size prior
mvision-tune --proto none frame.png                 # write PNG, no terminal
```

Corpus stability across many frames:

```
mvision-tune --thr 50 --corpus captures/0603/
```

Prints per-frame CSV to stdout, summary with σ to stderr. `angle` uses circular statistics (chip normalises to (-45°, 45]; readings at +44° and -44° are 2° apart mod 90, not 88).

Sample corpus in `tests/corpus/` (see `tests/corpus/README.md` for class layout); covered by `tests/smoke_comp.sh`.

## Code layout

- `src/dllmain.cpp` — DLL entry, ABI exports.
- `src/passthrough.cpp` — forwards-to-original for everything we don't override.
- `src/capture.cpp` — host-side frame plumbing + capture sentinels.
- `src/detect_*.cpp` — per-mode detectors; constants near the top of each file.
- `src/detect_common.cpp` — shared field/frame deinterlace + image-adjustment LUT.
- `src/preview.cpp` — overlay rendering (GDI).
- `src/controller.cpp` — TCP px↔mm scale read from the motion controller.
- `src/settings.cpp` / `settings.h` — per-mode Settings + INI persistence.
- `src/settings_ui.cpp` — Ctrl+Alt+M dialog.
- `src/camera_chip.cpp` — SAA7113 hardware control surface (used by Camera tab).
- `tools/saa7113-tune/` — standalone diagnostic tuner; `common.cpp` houses the
  shared WinUSB + STK1160 I²C-master primitives consumed by both the DLL
  (Camera tab) and the grabber.
- `tools/mvision-grabber/` — DirectShow source filter (.ax).
- `tests/` — host-arch test harness, corpus, mvision-tune.

## Caveats

- Developed on a Gen-2 802B. Analog cameras → CD4052 mux → Syntek STK1150 USB capture, 640×480 grayscale. Different capture hardware may behave differently.
- Fiducial target is the 1 mm copper pad; a very different fiducial *shape* (not size) needs a code-level tweak.
- Coordinate conventions (down-mirror, sub-pixel crop, offset signs) were recovered by reverse engineering against the original DLL.

## Pointers

- `tools/saa7113-tune/SYNTEK_RE.md` (gitignored / private) — vendor-driver RE notes.
- `~/.claude/projects/-home-turo/memory/` — long-form context for AI agents touching this repo. `MEMORY.md` is the index.
