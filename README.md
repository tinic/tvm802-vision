# tvm802-vision — OpenCV down-vision fiducial detector for the QiHe TVM802A / TVM802B

A drop-in `MVision.dll` for the QiHe **TVM802A / TVM802B** pick-and-place that
replaces the stock **down-vision (board fiducial) detection** with a modern OpenCV
pipeline. The stock vision is jittery, drops lock, and is slow; this gives a
sub-pixel-stable, motion-robust lock — even at high head speed.

> **Not affiliated with QiHe.** Independent, interoperability-focused
> reimplementation. **You supply your own original `MVision.dll`** — no vendor
> binaries are included or redistributed.
>
> Developed and tested on a **TVM802B**. The 802A shares the same vision DLL and
> analog capture path, so it should apply there too — but is **untested on the
> 802A**; reports welcome.

## What it does

- Handles **all three down-vision mark modes** — Circular (Hough), ImageTemplate
  (template-match for square / cross / text / arbitrary shapes), and Round
  (circular-symmetry). All are field-aware with sub-pixel offsets.
- **Deinterlaces** the analog capture and detects each field independently, so a
  moving fiducial no longer combs into a "double image" — robust at high job speed.
- **Settle / fresh-frame guard** discards the stale duplicate frames the USB
  capture repeats, lets the head ring down before the read, then averages both
  fields for a stable, repeatable sub-pixel lock.
- Renders its **own 1:1 preview overlay** (reference crosshair, search circle,
  detection marker).

## How it works

A **passthrough shim**: it exports the original DLL's ABI, overrides only the
down-vision mark paths (`CheckMark2`, `CheckTemplate`, `CheckMark`) plus their
preview, and forwards everything else to the original — which you rename to
`MVision-orig.dll`.

## Status

- [x] **Down-vision complete** — all three mark modes implemented, field-aware,
  and hardware-validated.
- [ ] **Up-vision component detection** (`CheckComp`) — possible future work, not
  currently planned.

Contributions and test reports (especially on the 802A) are welcome.

## Prebuilt binary

CI builds a self-contained `MVision.dll` (x86, static OpenCV) on every push:

- **Latest:** the **Actions** tab → newest `build` run → `MVision-x86` artifact.
- **Releases:** the DLL is attached to each `v*` tag.

## Build

A 32-bit (x86) MSVC build — it must match the host process. Install Visual Studio
Build Tools, CMake, and **vcpkg**; OpenCV is pulled in automatically (minimal,
static) from `vcpkg.json`. Set `VCPKG_ROOT`, then run **`build-cmake.bat`** →
`build\Release\MVision.dll`.

## Install

With SurfaceMount **closed**: back up your `MVision.dll`, rename the original to
**`MVision-orig.dll`** (same folder), drop in the new `MVision.dll`, and relaunch.

## Tuning

**Fiducial size** is set in the app UI, *not* in code — the detector scales its
radius bracket to the host's mark-size setting, so set that to match your fiducial.
Detector logic is one-per-file under `src/` (`detect_circle.cpp`,
`detect_template.cpp`, `detect_symmetry.cpp`), with the shared field/frame plumbing
in `detect_common.cpp`; tuning constants live at the top of each.

## Caveats

- Developed on a **Gen-2 802B**: analog cameras → CD4052 mux → Syntek STK1150 USB
  capture, 640×480 grayscale. Other capture hardware may behave differently.
- The fiducial target is the **1 mm copper pad**; a very different fiducial
  *shape* (not size) needs a code-level tweak.
- Coordinate conventions (down-mirror, sub-pixel crop, offset signs) were
  recovered by reverse engineering and matched to the original DLL's behavior.

## License

MIT — see [LICENSE](LICENSE).
