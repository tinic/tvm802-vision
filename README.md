# tvm802-vision — OpenCV down-vision fiducial detector for the QiHe TVM802A / TVM802B

A drop-in replacement for the QiHe **TVM802A / TVM802B** pick-and-place's
`MVision.dll` that replaces the stock **down-vision (board fiducial) detection**
with a modern OpenCV pipeline. The stock vision is jittery, drops lock, and is
slow; this gives a sub-pixel-stable, motion-robust fiducial lock — reliable even
at high head speed.

> Developed and tested on a **TVM802B**. The TVM802A shares the same vision DLL
> and analog capture path, so this should apply to it as well (the 802A is the
> more common model) — but it is currently **untested on the 802A**; reports
> welcome. For a different fiducial size, set the app's **mark-size setting** to
> match (the detector scales with it — see *Tuning*).

> **Not affiliated with QiHe.** This is an independent, interoperability-focused
> reimplementation. **You must supply your own original `MVision.dll`** — no
> vendor binaries are included or redistributed here.

## What it does

- Handles **all three down-vision mark modes**: **Circular** fiducial detection
  (Hough), **ImageTemplate** template-match (for square / cross / text / arbitrary
  mark shapes), and **Round** (`CheckMark`) **circular-symmetry** detection
  (threshold- and edge-free — it finds the center that maximizes concentric-ring
  symmetry, robust to soft/low-contrast/specular rings). All are field-aware and
  share the same sub-pixel offset plumbing.
- Locks the **1 mm copper fiducial pad** with a tight Hough radius bracket, so it
  ignores the larger concentric solder-mask ring that made the stock detection
  "jitter" (the detector was flip-flopping between the two circles).
- **Deinterlaces** the analog capture and detects on *both* fields independently
  — each field is a single instant in time, so a moving target no longer combs
  into a "double image". Robust at high job speed (the template-match path is
  field-aware too — it matches each field, not the combed frame).
- **Bilateral denoise before CLAHE** → cleans analog sensor noise without
  softening the copper edge, for a sub-pixel center.
- **Averages both fields when settled** → recovers full vertical resolution and
  cancels the per-field bias, for sub-pixel placement accuracy.
- **Fresh-frame / settle guard** → discards the stale duplicate frames the USB
  capture hands back when polled faster than it delivers, and lets the head ring
  down before the read, for reliable, repeatable lock-in.
- Renders its **own 1:1 preview overlay** (crisp reference crosshair, search
  circle, and green detection marker).

## How it works

It's a **passthrough shim**. It exports the same ABI as the original
`MVision.dll`, forwards every call it doesn't change to the original (which you
rename to `MVision-orig.dll`), and overrides only the down-vision mark paths
(`CheckMark2` circular, `CheckTemplate` template-match, and `CheckMark` Round,
plus their preview). `GetOffset`/`GetMin_val` return our result for those modes;
everything else (up-vision component checks, calibration, etc.) passes straight
through to the original, unchanged.

## Status & roadmap

This project is replacing the TVM802's weak stock vision one camera path at a
time, all through the same passthrough shim:

- [x] **Down-vision fiducial detection** (`CheckMark2`, circular) — done.
- [x] **Down-vision template-match** (`CheckTemplate`, ImageTemplate mode) — done;
  field-aware multi-scale SQDIFF for non-circular / arbitrary mark shapes.
- [x] **Down-vision Round mark** (`CheckMark`) — done; **circular-symmetry**
  detection (threshold- and edge-free, sub-pixel) within a fixed physical size
  bracket (0.5–3.5 mm). **All three down-vision mark modes are now covered — the
  down-vision camera path is complete.**
- [ ] **Up-vision component detection** (`CheckComp`) — possible future work, not
  currently planned. Components are varied shapes and placement needs an accurate
  rotation angle, so this is a larger effort (minimum-area-rectangle /
  rectilinear-symmetry based, returning center + θ).

Down-vision paths share the shim, the `GetOffset` plumbing, and the
capture-for-offline-tuning workflow. Contributions and test reports (especially on
the 802A) are welcome.

## Requirements

- **32-bit (x86)** build — it must match the host process.
- MSVC (Visual Studio Build Tools), CMake, and **vcpkg**.
- OpenCV is declared in `vcpkg.json` (minimal: core / imgproc / imgcodecs, x86
  **static**) and installed automatically by the vcpkg toolchain — no manual
  `vcpkg install` needed, and no `dnn`/protobuf pulled in.
- Your **original `MVision.dll`** from your machine.

## Prebuilt binary

Don't want to build it yourself? CI builds `MVision.dll` (x86, self-contained —
static OpenCV) on every push:

- **Latest:** the **Actions** tab → newest `build` run → `MVision-x86` artifact.
- **Tagged releases:** the **Releases** page (the DLL is attached to each `v*` tag).

You still need to supply your own original `MVision.dll` (renamed
`MVision-orig.dll`) — see *Install*.

## Build

Set `VCPKG_ROOT` to your vcpkg checkout, then run **`build-cmake.bat`** — it pins
the vcpkg baseline, installs the minimal OpenCV from `vcpkg.json`, and builds.
Output: `build\Release\MVision.dll`.

Or directly:

    vcpkg x-update-baseline --add-initial-baseline
    cmake -S . -B build -A Win32 ^
      -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake ^
      -DVCPKG_TARGET_TRIPLET=x86-windows-static
    cmake --build build --config Release

## Install

With the SurfaceMount application **closed**:

1. Back up your original `MVision.dll`.
2. Rename the original to **`MVision-orig.dll`** (same folder).
3. Drop in the freshly built `MVision.dll`.
4. Launch the app — down-vision fiducial detection now uses this pipeline.

## Tuning

The detectors are split one-per-file under `src/` (`detect_circle.cpp`,
`detect_template.cpp`, `detect_symmetry.cpp`), with the shared field/frame plumbing
in `detect_common.cpp`:

- **Mark size** is set in the app's UI, *not* in code: the detection radius
  bracket is computed as `0.22–0.42 × size` where `size` is the host's mark
  "size" argument. So a different fiducial *size* is handled by the UI setting —
  set it to match your fiducial and the bracket tracks it. The `0.22–0.42`
  fractions only need editing if the fiducial's *shape/proportion* differs a lot
  from this copper-pad-in-mask-ring (e.g. a solid disc). Constants live near the
  top of `detect_circle.cpp` (circularity, contrast, search-area gates).
- **Round (`CheckMark`)**: this mode has no host mark-size argument, so instead of
  scaling with a size it accepts any round feature of **0.5–3.5 mm diameter** (a
  fixed physical radius bracket, `kCsym.minRadiusPx`/`maxRadiusPx`), and uses the
  app's **Range** value as the search radius. It's **circular-symmetry** detection:
  at each candidate center it scores the variance ratio across concentric rings
  (uniform rings about a true center → high score), which is threshold- and
  edge-free, so it stays locked where Hough or contour fits break up on
  soft/low-contrast/specular rings. Coarse → fine → parabolic sub-pixel; the
  `CircularSymmetry` scorer and `kCsym` constants are in `detect_symmetry.cpp`.
- **Template-match (ImageTemplate)**: the SQDIFF accept threshold, multi-scale
  sweep, and parabolic sub-pixel refine are in `detect_template_one_field`;
  `detect_template_mark` preps the template and runs it field-aware
  (`detect_template.cpp`).
- **Deinterlace / averaging**: field split, settled-averaging threshold, and the
  moving-frame field choice live in `detect_with_fields` (shared by both modes).

## Caveats

- Developed on a **Gen-2 802B**: two analog cameras → CD4052 mux → Syntek
  STK1150 USB capture, 640×480 grayscale. Other capture hardware (e.g. some
  802A revisions) may behave differently.
- The fiducial target is the **1 mm copper pad**. Different fiducial *sizes* are
  handled by the app's mark-size setting (the radius bracket scales with it); see
  *Tuning*. Only a very different fiducial *shape* needs a code-level tweak.
- The coordinate convention (180° down-mirror, sub-pixel crop, offset signs) was
  recovered by reverse engineering and matched to the original DLL's behavior.

## License

MIT — see [LICENSE](LICENSE).
