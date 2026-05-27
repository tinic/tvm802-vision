# mvision_grabber.ax

**Drop-in DirectShow capture-filter replacement for the Syntek STK1150
"USB2.0 Grabber" driver on the QiHe TVM802B.** Usermode COM in-proc server
(.ax = renamed .dll); built with MSVC, installed via custom INF that binds
WinUSB to the device at first plug (no Zadig dance per machine).

## Why

The stock Syntek driver:

- Software-fakes `IAMVideoProcAmp` (brightness/contrast/gain are post-process
  on the YUYV buffer; the SAA7113 never sees the values; see
  `~/.claude/projects/-home-turo/memory/reference_stk1150_capture_surface.md`).
- Conflicts with `saa7113-tune` (have to Zadig-swap drivers per tuning
  session — destroys the iteration loop).

This filter:

- Talks WinUSB → STK1160 I²C master → SAA7113 *directly* for every control.
  Brightness/contrast/gain actually affect the chip.
- Is enumerated by `ICreateDevEnum.CreateClassEnumerator(CLSID_VideoInputDeviceCategory)`
  under the friendly name **"USB2.0 Grabber"** — SurfaceMount.exe finds and
  opens it via its existing DirectShow code path with no changes.
- Exposes `IKsPropertySet` with a vendor property set so a co-process
  tuning UI can drive the chip without needing its own WinUSB binding.

## SurfaceMount transparency contract

SurfaceMount uses (per memory `reference_stk1150_capture_surface`, section
"What's actually in the host today"):

```
DsDevice.GetDevicesOfCat(VideoInputDevice)  →  match on "USB2.0 Grabber"
ifilterGraph2_0.AddSourceFilterForMoniker(...)
IAMStreamConfig.SetFormat(640x480 YUY2)
SampleGrabber + NULL renderer + Graph.Run()
```

So our filter MUST:

1. Register as `CLSID_VideoInputDeviceCategory` with friendly name
   `USB2.0 Grabber` (string match exact).
2. Advertise a 640×480 YUY2 / UYVY format on its capture output pin
   (`IAMStreamConfig`).
3. Stream samples in that format.

Anything else SurfaceMount *also* exercises (still pin, etc.) — TBD as we
finish the Syntek RE. See `SYNTEK_RE.md` (private; gitignored).

## Architecture (one file at first; will split when it grows)

```
grabber.cpp
  ├─ COM plumbing: DllMain, DllGetClassObject, DllCanUnloadNow,
  │   DllRegisterServer (IFilterMapper2 + FriendlyName + AM_KSCATEGORY_*),
  │   DllUnregisterServer, class factory.
  ├─ Filter:  IBaseFilter / IMediaFilter / IPersist / IAMFilterMiscFlags
  │           IAMVideoProcAmp  (real-hardware via WinUSB → SAA7113 I²C)
  │           IKsPropertySet   (vendor-private surface for tuning UI)
  ├─ Pin:     IPin output (capture category), IAMStreamConfig,
  │           IQualityControl, IKsPropertySet (advertises being a
  │           CAPTURE pin via PINNAME_VIDEO_CAPTURE)
  ├─ Worker:  WinUSB isoc thread; pulls frames; calls downstream
  │           IMemAllocator::GetBuffer + IMemInputPin::Receive.
  └─ Capture: shared with tools/saa7113-tune common.cpp (same I²C-master
              register code).
```

## Build

```
cmake -S . -B build-grabber -DBUILD_MVISION_GRABBER=ON
cmake --build build-grabber --target mvision_grabber --config Release
```

Produces `mvision_grabber.ax`. Install + register:

```
copy mvision_grabber.ax %WINDIR%\System32\
regsvr32 mvision_grabber.ax
```

(Once we have the INF, distribution is single-step via `pnputil`.)

## Status

Shipping since v1.11.0 — full DirectShow pipeline (enum / connect /
allocator / `IAMVideoProcAmp` real-hardware / streaming) validated
end-to-end against SurfaceMount. The remaining items would just polish
distribution (custom INF that binds WinUSB at first plug = no Zadig
dance, signed binary = no SmartScreen warning); both are optional and
the project owner is comfortable shipping with the one-time Zadig step.
