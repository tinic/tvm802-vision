# mvision_grabber.ax

A usermode DirectShow capture filter that acts as a drop-in replacement for the Syntek **"USB2.0 Grabber"** on the QiHe TVM802B. It registers under `CLSID_VideoInputDeviceCategory` with the same friendly name SurfaceMount searches for through `DsDevice.GetDevicesOfCat`, so the host opens it through its existing DirectShow code path with no changes. The filter streams UYVY 640×480 over WinUSB using STK1160 isochronous transfers, and exposes `IAMVideoProcAmp` connected to real SAA7113 register writes.

Why we need it: on this hardware family, the stock Syntek driver's `IAMVideoProcAmp` is faked in software. Its writes target a CMOS-sensor pipeline that does not exist on these boards, so changing brightness, contrast, or gain through SurfaceMount never reaches the chip. Our filter routes those calls to the real registers.

## Install

Use `install.bat` from the release bundle (right-click → Run as administrator). It copies `mvision_grabber.ax` into `%USERPROFILE%\qihetvm802b\` and registers it with the 32-bit `regsvr32` (`SysWOW64\regsvr32.exe` on 64-bit Windows). You also need to run Zadig once to bind the WinUSB driver; the bundle's `INSTALL.md` has the walkthrough.

To register the filter manually (for example when building from source):

```
C:\Windows\SysWOW64\regsvr32.exe path\to\mvision_grabber.ax
```

To uninstall, run `regsvr32 /u` on the filter and then delete the file, or run the bundle's `uninstall.bat`.

## Build

The filter builds as part of the main CMake configuration when you pass `-DBUILD_MVISION_GRABBER=ON`. See `CLAUDE.md` for the full build instructions, the SurfaceMount transparency contract, and the rationale behind the COM, threading, and leak-on-Release design choices.
