# tvm802-vision — install

You've unzipped this on a machine that runs **SurfaceMount.exe** for a QiHe
**TVM802A / TVM802B** pick-and-place. Three pieces inside:

| File | What it does |
|---|---|
| `MVision.dll` (or `MVision-noavx2.dll`) | Drop-in detector replacement (down-vision fiducial + up-vision component) |
| `mvision_grabber.ax` | DirectShow source filter — SurfaceMount uses this transparently instead of the stock Syntek driver |
| `saa7113-tune.exe` | Standalone chip-side tuning tool (Brightness / Contrast / Gain / live preview) |

## Quick install (recommended)

1. **Close `SurfaceMount.exe`** if it's running.
2. **Run `install.bat`** as Administrator (right-click → *Run as administrator*).
   - It auto-targets `%USERPROFILE%\qihetvm802b\` (the standard QiHe install path).
   - If yours is elsewhere, pass the path:
     `install.bat "D:\My Path\SurfaceMount folder"`
   - The script backs up your stock `MVision.dll` to `MVision-orig.dll` on first install, copies the three new files, and registers the DirectShow filter via `regsvr32`.
3. **One-time Zadig WinUSB binding** (only needed if you want chip-level camera tuning — the Camera tab in `Ctrl+Alt+M`). The detector improvements work without this.
   - Download **Zadig** (single 5 MB .exe, no installer): <https://zadig.akeo.ie/>
   - Run Zadig → **Options → List All Devices**.
   - You'll see TWO entries for the analog capture board:
     - `USB2.0 Grabber (Interface 0)`
     - `USB2.0 Grabber (Interface 1)`
   - For **BOTH**: pick it in the dropdown → target driver = **libusbK** → click **Replace Driver**.
   - Done. This binding persists across reboots and Windows updates — only needs to be done once per machine.
4. **Launch SurfaceMount.exe**. It now uses our detector + grabber transparently.

## Verify

- Press **`Ctrl + Alt + M`** while SurfaceMount is running. The settings dialog opens.
- Click the **Camera** tab — Brightness / Contrast / Gain / Sharpness sliders + AGC and Prefilter checkboxes. Drag a slider — the live picture in SurfaceMount should change immediately.
- (If the Camera-tab sliders do nothing, the Zadig binding step was missed or hit only one interface — repeat step 3.)

## CPU choice (TVM802BX only)

Two MVision DLLs in the bundle:

| Build | Use when |
|---|---|
| `MVision.dll` | Any modern PC (Haswell / Zen or newer ~ 2015+). AVX2 build. |
| `MVision-noavx2.dll` | **TVM802BX** (built-in Atom N2800), and any older PC that crashes the AVX2 build with an illegal-instruction fault. SSE2 baseline. |

For the BX, **rename `MVision-noavx2.dll` → `MVision.dll`** AFTER running `install.bat` (or modify the script — it copies the AVX2 build by default).

## Uninstall

Run **`uninstall.bat`** as Administrator. It:

- Unregisters `mvision_grabber.ax`
- Restores `MVision-orig.dll` → `MVision.dll`
- Deletes the extra files

It does **not** touch the Zadig binding (that's per USB device; if you want to revert, use Device Manager → USB2.0 Grabber → Uninstall device → tick "Delete the driver software" → unplug + replug).

## Troubleshooting

- **`regsvr32 failed`** — run `install.bat` as Administrator (right-click → Run as administrator).
- **`SurfaceMount.exe is running`** — close it first; the DLL is locked while loaded.
- **AVX2 illegal instruction on the BX** — rename `MVision-noavx2.dll` → `MVision.dll`.
- **Camera tab sliders do nothing** — Zadig step missed; do it for both interfaces.
- **Live picture has wrong aspect** — should not happen; report a bug with a screenshot.
- **Crash on SurfaceMount exit** — known historical issue, fixed in the current bundle. If you see it on this version, file a bug with the .NET exception trace.

## What changed vs the stock vision

See `README.md` in the source repo: <https://github.com/tinic/tvm802-vision>
