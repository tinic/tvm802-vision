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
> 802A**; reports welcome. The **TVM802BX** is the same machine with a built-in PC +
> touchscreen (a low-power Intel Atom — see *Prebuilt binary* for the build it needs
> and why).

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
- **Live, on-machine tuning** via an on-screen settings dialog (**`Ctrl + Alt + M`**)
  with a real-time lock/score readout — per-mode detection knobs and image
  adjustments, no recompile (see *Tuning*).

## How it works

A **passthrough shim**: it exports the original DLL's ABI, overrides only the
down-vision mark paths (`CheckMark2`, `CheckTemplate`, `CheckMark`) plus their
preview, and forwards everything else to the original — which you rename to
`MVision-orig.dll`.

## Status

- [x] **Down-vision complete** — all three mark modes implemented, field-aware,
  and hardware-validated.
- [x] **Up-vision component detection** (`CheckComp`) — **drives placement, validated
  on hardware** (still opt-in via a sentinel file, so a shipped DLL is inert until an
  operator enables it). A rectilinear-symmetry detector (center + size + angle) with a
  minimum-area-rectangle fallback for asymmetric parts; the pose-to-host transform is
  calibrated against the original DLL over hundreds of real reads. It tracks parts to
  sub-pixel, ignores the off-nozzle distractors the stock vision locks onto, and is
  **field-aware** — it detects on the newest single field when the part is rotating in
  flight (no interlace comb), woven full-res when settled. It renders its own oriented
  overlay (green body box + direction arrow, mm tick marks, settings hint) matching the
  down view. Live tuning shares the `Ctrl+Alt+M` dialog (pick **Component** from the mode
  dropdown). Per-detector on/off checkboxes let an operator fall any method back to the
  stock vision.
  - [ ] **Known gap:** when no part is on the nozzle (e.g. dropped in flight), it does
    not yet *reject* the placement — it falls back to the stock detector. A clean reject
    needs the host's internal nozzle-calibration, which isn't exposed; tracked for future
    work.

Contributions and test reports (especially on the 802A) are welcome.

## Prebuilt binary

CI builds a self-contained `MVision.dll` (x86, static OpenCV) on every push:

- **Latest:** the **Actions** tab → newest `build` run → `MVision-x86` artifact.
- **Releases:** the DLLs are attached to each `v*` tag.

Two builds are produced (same code, same ABI — pick one and rename it to
`MVision.dll`):

- **`MVision.dll`** — AVX2, recommended for any CPU from ~2015 on (Haswell / Zen+).
  It self-checks at startup: on a CPU without AVX2 it pops a modal warning pointing
  you to the no-AVX2 build, instead of faulting with a cryptic illegal instruction.
- **`MVision-noavx2.dll`** — SSE2 baseline, for older CPUs. (OpenCV does its own
  runtime CPU dispatch, so only our own emitted code carries the AVX2 requirement.)

### Which build for which machine — mind the CPU

The plain **802A / 802B** drive from *your* external PC: use the AVX2 build on
anything ~2015 or newer, the no-AVX2 build on older silicon.

The **TVM802BX** (built-in PC + touchscreen) is the one to watch. Its embedded
computer has been reported (EEVblog teardown) as an **Intel Atom N2800** — *Cedar
Trail / Saltwell, 2012; in-order dual-core + Hyper-Threading @ 1.86 GHz; 2 GB DDR3*.
Its SIMD ceiling is **SSSE3 — no SSE4, no AVX, no AVX2, no FMA.** So on the BX you
**must** use **`MVision-noavx2.dll`**; the AVX2 build cannot run there. (Other QiHe
built-in boards may vary by batch, but expect a similar low-power Atom — check it
with Task Manager / CPU-Z and use the no-AVX2 build unless it reports AVX2.)

That N2800 is also slow and memory-bandwidth-limited, so on the BX keep the host
**Range** small and prefer **Circular** over **Round** to keep the detector light.
(The no-AVX2 baseline is plain **SSE2** on purpose: the N2800 predates SSE4, and SSE4
measured *no* speedup for this gather-bound detector anyway — only AVX2's FMA helped,
which that Atom lacks.)

## Build

A 32-bit (x86) MSVC build — it must match the host process. Install Visual Studio
Build Tools, CMake, and **vcpkg**; OpenCV is pulled in automatically (minimal,
static) from `vcpkg.json`. Set `VCPKG_ROOT`, then run **`build-cmake.bat`**. This
produces both `build\Release\MVision.dll` (AVX2) and `build\Release\MVision-noavx2.dll`
(SSE2 baseline).

## Install

With SurfaceMount **closed**: back up your `MVision.dll`, rename the original to
**`MVision-orig.dll`** (same folder), drop in the new `MVision.dll`, and relaunch.

## Tuning

### Live settings dialog — press **Ctrl + Alt + M**

While SurfaceMount is running (with a camera view active), the global hotkey
**`Ctrl + Alt + M`** opens an on-screen settings panel — there's also a small
`Ctrl+Alt+M: settings` hint in the top-right of the preview. It lets you tune
detection **on the machine, with no recompile and no file editing**, watching a
**live readout** (active mode, `LOCKED` / `NO LOCK`, score, detected radius,
offset) update as you drag — so you adjust until the marker turns green. The
preview also shows the image adjustments live.

- **Per mode.** Round, Circular, ImageTemplate, and **Component** (up-vision) each
  have independent settings. The panel follows whichever down mode is running; use the
  **Edit:** dropdown to pick a mode manually (the only way to reach Component, since
  up-vision publishes no mode). Controls that don't affect the selected mode are greyed
  out. **Save** writes `MVision.ini` (reloaded on next launch); **Reset** reverts the
  current mode. Everything starts at **Auto / neutral**.
- **Detectors on/off.** Four checkboxes (Round / Circular / ImageTemplate / Component)
  fall any one method back to the **stock** vision without reinstalling; default on.
- **Detection:** fiducial **radius** bracket (min/max px), accept **sensitivity**
  (lower = more lenient), **exposure** gate (min/max frame brightness), and — for
  Round — **median ring scoring** (more robust to glare). In **Component** mode the two
  radius sliders become the stray-guard **search radius** and **max part size** (in px,
  spanning the up-camera frame — large LQFP MCUs are hundreds of px).
- **Image adjustments** (applied before detection): **gamma, brightness, contrast,
  black/white levels, sharpen,** and a Gaussian **blur** (smooths a speckled or
  specular pad so it reads cleanly).

> **Stubborn / specular pad not locking?** Raise **Blur** to average out the glare
> speckle, turn on **median ring scoring**, set the **radius** bracket around the
> pad, and use **gamma > 1** / **white point** to tame a blown highlight — watch the
> live score climb past the threshold. (A genuinely domed/specular copper pad may
> still need light sanding to a matte finish.)

### Host (SurfaceMount) settings

Some behavior comes from the app itself, not this dialog:

- **Circular** scales its detection radius to the app's **mark-size** setting.
- **ImageTemplate** uses the template image you teach in the UI.
- **Round** uses the app's **Range** value as the search radius.

### Code-level

Built-in defaults / constants live near the top of the per-mode files under `src/`
(`detect_circle.cpp`, `detect_template.cpp`, `detect_symmetry.cpp`; shared
field/frame plumbing and the image-adjustment chain in `detect_common.cpp`).

## Caveats

- Developed on a **Gen-2 802B**: analog cameras → CD4052 mux → Syntek STK1150 USB
  capture, 640×480 grayscale. Other capture hardware may behave differently.
- The fiducial target is the **1 mm copper pad**; a very different fiducial
  *shape* (not size) needs a code-level tweak.
- Coordinate conventions (down-mirror, sub-pixel crop, offset signs) were
  recovered by reverse engineering and matched to the original DLL's behavior.

## License

MIT — see [LICENSE](LICENSE).
