# tvm802-vision

Drop-in `MVision.dll` for the QiHe **TVM802A / TVM802B / TVM802BX** pick-and-place — replaces the stock vision (jittery, drops lock, slow) with a sub-pixel-stable, motion-robust OpenCV pipeline. Plus a chip-level **DirectShow source filter** and a **Ctrl+Alt+M settings dialog** that drive the analog capture board's brightness / contrast / gain directly in hardware.

![Settings dialog beside the SurfaceMount preview](docs/images/capture.png)

> Not affiliated with QiHe. You supply your own original `MVision.dll` (renamed to `MVision-orig.dll`) — no vendor binaries are bundled.

## Install (5 minutes)

1. Download **`tvm802-vision-vX.Y.Z.zip`** from the [latest release](https://github.com/tinic/tvm802-vision/releases/latest).
2. Unzip it, close `SurfaceMount.exe`, right-click **`install.bat` → Run as administrator**.
3. (Optional, only for chip-level Camera tab) Run **Zadig** once per machine — `INSTALL.md` in the zip has the per-step walkthrough.

**On the TVM802BX** (built-in Atom): after `install.bat`, rename `MVision-noavx2.dll` → `MVision.dll` in the SurfaceMount folder. (The Atom N2800 has no AVX2; `INSTALL.md` covers this.)

## What you get

- **All three down-vision mark modes** (Round / Circular / ImageTemplate) — field-aware deinterlace, sub-pixel offsets, robust at high head speed.
- **Up-vision component detection** (`CheckComp`) — rectilinear-symmetry + minimum-area-rect fallback. Drives placement; missing-part reject; per-stack `视觉阈值` (CompThre) is the operator's main tuning knob.
- **Live settings dialog** (`Ctrl + Alt + M`) — per-mode tuning with live LOCKED / NO LOCK readout, no recompile.
- **Camera tab** (with the DirectShow filter installed) — hardware Brightness / Contrast / Gain / AGC / Sharpness / Prefilter directly on the SAA7113 chip, per-mode, persisted to `MVision.ini`.

## Tuning

Press **`Ctrl + Alt + M`** while SurfaceMount is running. Three tabs:

- **Detection** — per-mode fiducial knobs (radius bracket, accept sensitivity, exposure gate, Round median-ring).
- **Image** — software pre-detect adjustments (gamma, brightness, contrast, black/white, sharpen, blur).
- **Camera** — chip-level hardware adjustments (only if the DirectShow filter is installed; see Install step 3).

The 4-method *Detector* strip below the tabs falls any mode back to the stock vision (uncheck to disable our path).

The host's per-stack **视觉阈值 (CompThre)** in ParamEdit is the main up-vision knob (it's a 0–100 % of the ROI's max brightness, not a gray level). A starting guide for common parts lives in `CLAUDE.md` under *CompThre table*.

## License

MIT — see [LICENSE](LICENSE).

---

Developer / architecture / build / RE notes: see **[CLAUDE.md](CLAUDE.md)**.
