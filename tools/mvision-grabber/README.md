# mvision_grabber.ax

Usermode DirectShow capture filter, drop-in for the Syntek **"USB2.0 Grabber"** on the QiHe TVM802B. Registers under `CLSID_VideoInputDeviceCategory` with the same friendly name SurfaceMount string-matches on (`DsDevice.GetDevicesOfCat`), so the host opens it via its existing DirectShow code path with no changes. Streams UYVY 640×480 via WinUSB + STK1160 isoc; exposes `IAMVideoProcAmp` wired to real SAA7113 register writes.

Why: stock Syntek's `IAMVideoProcAmp` is software-faked on this hardware family (writes target a CMOS-sensor pipeline that isn't there) — brightness / contrast / gain through SurfaceMount never actually reach the chip. Our filter does.

## Install

Use the release bundle's `install.bat` (right-click → Run as administrator). It copies `mvision_grabber.ax` to `%USERPROFILE%\qihetvm802b\` and registers it via 32-bit `regsvr32` (`SysWOW64\regsvr32.exe` on 64-bit Windows). One-time Zadig WinUSB binding is also required — `INSTALL.md` in the bundle has the walkthrough.

To register manually (e.g. when building from source):

```
C:\Windows\SysWOW64\regsvr32.exe path\to\mvision_grabber.ax
```

Uninstall: `regsvr32 /u` then delete the file. Or run the bundle's `uninstall.bat`.

## Build

Part of the main cmake config when `-DBUILD_MVISION_GRABBER=ON`. See `CLAUDE.md` for the full build instructions, the SurfaceMount transparency contract, and the COM / threading / leak-on-Release rationale.
