# tvm802-vision — install

You have unzipped this on a machine that runs **SurfaceMount.exe** for a QiHe **TVM802A / TVM802B** pick-and-place. The bundle contains three pieces:

| File | What it does |
|---|---|
| `MVision.dll` (or `MVision-noavx2.dll`) | The drop-in detector replacement, for both down-vision fiducials and up-vision components. |
| `mvision_grabber.ax` | A DirectShow source filter. SurfaceMount uses it in place of the stock Syntek driver, with no changes to the host. |
| `saa7113-tune.exe` | A standalone tuning tool for the capture chip (brightness, contrast, gain, live preview). |

## Quick install (recommended)

1. **Close `SurfaceMount.exe`** if it is running.
2. **Run `install.bat` as Administrator** (right-click → *Run as administrator*).
   - The script targets `%USERPROFILE%\qihetvm802b\` by default, which is the standard QiHe install path.
   - If your install lives somewhere else, pass the path as an argument: `install.bat "D:\My Path\SurfaceMount folder"`.
   - On a first install the script backs up your stock `MVision.dll` to `MVision-orig.dll`, copies the three new files into place, and registers the DirectShow filter with `regsvr32`.
3. **Run Zadig once to bind the WinUSB driver**. This step is only needed if you want chip-level camera tuning (the Camera tab inside the `Ctrl+Alt+M` dialog). The detector improvements work without it. See the section below for the full walkthrough.
4. **Launch SurfaceMount.exe**. From this point on it uses our detector and grabber transparently.

## Zadig walkthrough

The Camera tab and the standalone `saa7113-tune.exe` talk to the SAA7113 capture chip over WinUSB. To do that, the USB capture board's two interfaces have to be bound to the **libusbK** driver instead of the stock Syntek driver. Zadig is the standard tool for this; the binding is a one-time operation per machine, and it persists across reboots and Windows updates.

1. **Download Zadig** from <https://zadig.akeo.ie/>. It is a single ~5 MB executable with no installer.
2. **Close SurfaceMount.exe** before changing the driver. If SurfaceMount has the device open, the rebind step will fail.
3. **Plug in the USB capture cable** if it is not already connected. The TVM802B's analog cameras feed into a USB capture board labelled `USB2.0 Grabber`.
4. **Launch Zadig as Administrator**.
5. In the menu bar, click **Options → List All Devices**, and uncheck **Ignore Hubs or Composite Parents**. Without these settings the two interfaces below do not show up.
6. In the device dropdown you should now see two entries:
   - `USB2.0 Grabber (Interface 0)`
   - `USB2.0 Grabber (Interface 1)`
7. **For each of the two entries**, do the following:
   - Select it in the dropdown.
   - Set the target driver (the box on the right) to **libusbK**. Use the up/down arrows next to the box to cycle through driver options if libusbK is not shown by default.
   - Click **Replace Driver** and wait for the success dialog.
8. **Done.** Both interfaces should now show `libusbK` as the current driver. The binding survives reboots, sleep, and the cable being unplugged. You only do this once per machine.

If you accidentally bind only Interface 0, the Camera tab sliders will appear to work but no actual data will flow to the chip, because libusb needs both interfaces of this composite device. Repeat the step for Interface 1 to fix it.

## Verify

- Press **`Ctrl + Alt + M`** while SurfaceMount is running. The settings dialog opens.
- Click the **Camera** tab. You should see sliders for brightness, contrast, gain, and sharpness, along with checkboxes for AGC and prefilter. Drag a slider and the live picture in SurfaceMount should change immediately.
- If the Camera-tab sliders do nothing, the Zadig binding was either skipped or only applied to one interface. Repeat the Zadig walkthrough above.

## CPU choice (TVM802BX only)

The bundle ships two MVision DLLs:

| Build | When to use it |
|---|---|
| `MVision.dll` | Any modern PC (Haswell or Zen, roughly 2015 or newer). This is the AVX2 build. |
| `MVision-noavx2.dll` | The **TVM802BX**, which has a built-in Atom N2800, and any older PC that crashes the AVX2 build with an illegal-instruction fault. This is the SSE2 baseline. |

On the BX, **rename `MVision-noavx2.dll` to `MVision.dll`** after running `install.bat`. The installer copies the AVX2 build by default; the rename swaps in the no-AVX2 build.

## Uninstall

Run **`uninstall.bat`** as Administrator. It:

- Unregisters `mvision_grabber.ax`.
- Restores `MVision-orig.dll` back to `MVision.dll`.
- Deletes the extra files.

It does **not** undo the Zadig binding, because that is a per-USB-device setting. If you want to revert it, open Device Manager, find the `USB2.0 Grabber` entries, choose **Uninstall device**, check **Delete the driver software**, then unplug and reconnect the USB cable. Windows will reinstall the stock driver.

## Troubleshooting

- **`regsvr32 failed`**: run `install.bat` as Administrator (right-click → Run as administrator).
- **`SurfaceMount.exe is running`**: close it before installing. The DLL is locked while the host has it loaded.
- **AVX2 illegal-instruction crash on the BX**: rename `MVision-noavx2.dll` to `MVision.dll`.
- **The Camera tab sliders do nothing**: the Zadig step was skipped, or only one of the two interfaces was rebound. Repeat the Zadig walkthrough for both interfaces.
- **The live picture has the wrong aspect ratio**: this should not happen on a supported board. Please file a bug with a screenshot.
- **SurfaceMount crashes on exit**: a historical issue that was fixed in the current bundle. If you still see it on this version, please file a bug with the .NET exception trace.

## What changed compared to the stock vision

See `README.md` in the source repository: <https://github.com/tinic/tvm802-vision>.
