// mvision_grabber.ax -- usermode DirectShow capture filter, drop-in
// replacement for the Syntek STK1150 "USB2.0 Grabber" capture surface.
//
// SCAFFOLD (2026-05-27): COM plumbing + filter/pin skeleton with stubs for
// the streaming path. Compiles, registers via DllRegisterServer + appears
// in DirectShow's CLSID_VideoInputDeviceCategory enumeration under the
// friendly name "USB2.0 Grabber". Streaming pipeline is TODO -- worker
// thread for WinUSB isoc + IMemInputPin::Receive flow comes next.
//
// Why hand-rolled IBaseFilter/IPin instead of the DirectShow Base Classes
// (CSource/CSourceStream etc.): the baseclasses live in the Windows Driver
// Kit samples (~50 files of legacy C++), and most of our value-add is the
// hardware-real IAMVideoProcAmp + IKsPropertySet plumbing -- the base
// boilerplate is verbose but tractable, and avoiding the vendored library
// means a single .cpp/.def we can scp to winbuilder for the iteration loop.
// If this grows past ~1500 lines we revisit.
//
// References (don't re-derive):
//   - tools/mvision-grabber/README.md  (SurfaceMount transparency contract)
//   - tools/mvision-grabber/SYNTEK_RE.md (private; vendor RE findings)
//   - memory: reference_stk1150_capture_surface

#define WIN32_LEAN_AND_MEAN
#define INITGUID
#include <windows.h>
#include <objbase.h>
#include <olectl.h>      // SELFREG_E_CLASS
#include <strmif.h>
#include <uuids.h>
#include <vfwmsgs.h>
#include <amvideo.h>
#include <control.h>
#include <ks.h>
#include <ksmedia.h>
#include <ksproxy.h>
#include <dvdmedia.h>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>

// ---------------------------------------------------------------------------
// CLSID for our filter. Generated 2026-05-27, dedicated to this project --
// must stay stable across releases (changing it breaks any tool that has it
// cached). Format: a unique 128-bit GUID we own.
//   {6F4E7B12-9C8E-4D3A-B5F6-1A2C3D4E5F60}
// ---------------------------------------------------------------------------
DEFINE_GUID(CLSID_MVisionGrabber,
    0x6f4e7b12, 0x9c8e, 0x4d3a, 0xb5, 0xf6, 0x1a, 0x2c, 0x3d, 0x4e, 0x5f, 0x60);

// Vendor-private KS property set for the tuning UI (input select, AGC, hist,
// any other SAA7113-specific knobs that don't map cleanly to IAMVideoProcAmp).
//   {6F4E7B12-9C8E-4D3A-B5F6-1A2C3D4E5F61}
DEFINE_GUID(MVISIONPROPSETID_Saa7113,
    0x6f4e7b12, 0x9c8e, 0x4d3a, 0xb5, 0xf6, 0x1a, 0x2c, 0x3d, 0x4e, 0x5f, 0x61);

namespace {

// SurfaceMount enumerates by friendly name; this string MUST stay
// "USB2.0 Grabber" exactly -- see SurfaceMount string-match per memory.
constexpr const wchar_t* kFriendlyName = L"USB2.0 Grabber";
constexpr const wchar_t* kPinNameCapture = L"Capture";

// Module-global ref/lock counts for the COM in-proc server lifecycle.
std::atomic<LONG> g_serverLockCount{0};
HINSTANCE g_hInst = nullptr;

// =========================================================================
// Class factory -- one filter class for now, so a single CClassFactory.
// =========================================================================

class CMVisionGrabberFilter;
HRESULT CreateInstance(IUnknown* pUnkOuter, REFIID riid, void** ppv);

class CClassFactory final : public IClassFactory {
    std::atomic<LONG> m_ref{1};
   public:
    // IUnknown ------------------------------------------------------------
    STDMETHODIMP QueryInterface(REFIID riid, void** ppv) override {
        if (!ppv) return E_POINTER;
        if (riid == IID_IUnknown || riid == IID_IClassFactory) {
            *ppv = static_cast<IClassFactory*>(this);
            AddRef();
            return S_OK;
        }
        *ppv = nullptr;
        return E_NOINTERFACE;
    }
    STDMETHODIMP_(ULONG) AddRef() override { return ULONG(++m_ref); }
    STDMETHODIMP_(ULONG) Release() override {
        LONG r = --m_ref;
        if (r == 0) delete this;
        return ULONG(r);
    }
    // IClassFactory -------------------------------------------------------
    STDMETHODIMP CreateInstance(IUnknown* pUnkOuter, REFIID riid, void** ppv) override {
        if (!ppv) return E_POINTER;
        *ppv = nullptr;
        if (pUnkOuter) return CLASS_E_NOAGGREGATION;
        return ::CreateInstance(pUnkOuter, riid, ppv);
    }
    STDMETHODIMP LockServer(BOOL fLock) override {
        if (fLock) ++g_serverLockCount;
        else       --g_serverLockCount;
        return S_OK;
    }
};

// =========================================================================
// IPin output stub -- enough to enumerate the pin, advertise UYVY 640x480,
// and be a connectable capture pin. Streaming Receive() flow is TODO.
// =========================================================================

class CCaptureOutputPin final : public IPin,
                                public IAMStreamConfig,
                                public IKsPropertySet,
                                public IQualityControl {
    std::atomic<LONG> m_ref{1};
    IBaseFilter* m_pFilter = nullptr;          // owner; weak ref
    IPin* m_pConnected = nullptr;              // downstream; strong ref
    AM_MEDIA_TYPE m_mtCurrent{};               // currently negotiated format
    BOOL m_bFormatSet = FALSE;

   public:
    explicit CCaptureOutputPin(IBaseFilter* pFilter) : m_pFilter(pFilter) {
        set_default_format(&m_mtCurrent);
    }
    ~CCaptureOutputPin() {
        if (m_pConnected) m_pConnected->Release();
        free_media_type(&m_mtCurrent);
    }

    // Format default: UYVY 640x480 30 fps. SurfaceMount sets w/h via
    // IAMStreamConfig::SetFormat to 640x480 (per memory reference_stk1150_*).
    static void set_default_format(AM_MEDIA_TYPE* mt) {
        std::memset(mt, 0, sizeof(*mt));
        mt->majortype = MEDIATYPE_Video;
        mt->subtype = MEDIASUBTYPE_UYVY;
        mt->bFixedSizeSamples = TRUE;
        mt->bTemporalCompression = FALSE;
        mt->lSampleSize = 640 * 480 * 2;
        mt->formattype = FORMAT_VideoInfo;
        mt->cbFormat = sizeof(VIDEOINFOHEADER);
        auto* vih = static_cast<VIDEOINFOHEADER*>(CoTaskMemAlloc(sizeof(VIDEOINFOHEADER)));
        std::memset(vih, 0, sizeof(*vih));
        vih->AvgTimePerFrame = 333667;  // 100ns units, ~30 fps
        vih->bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        vih->bmiHeader.biWidth = 640;
        vih->bmiHeader.biHeight = 480;
        vih->bmiHeader.biPlanes = 1;
        vih->bmiHeader.biBitCount = 16;
        vih->bmiHeader.biCompression = MAKEFOURCC('U', 'Y', 'V', 'Y');
        vih->bmiHeader.biSizeImage = 640 * 480 * 2;
        mt->pbFormat = reinterpret_cast<BYTE*>(vih);
    }
    static void free_media_type(AM_MEDIA_TYPE* mt) {
        if (mt->pbFormat) { CoTaskMemFree(mt->pbFormat); mt->pbFormat = nullptr; }
        if (mt->pUnk)     { mt->pUnk->Release();          mt->pUnk = nullptr; }
        mt->cbFormat = 0;
    }

    // IUnknown ------------------------------------------------------------
    STDMETHODIMP QueryInterface(REFIID riid, void** ppv) override {
        if (!ppv) return E_POINTER;
        if (riid == IID_IUnknown || riid == IID_IPin)
            *ppv = static_cast<IPin*>(this);
        else if (riid == IID_IAMStreamConfig)
            *ppv = static_cast<IAMStreamConfig*>(this);
        else if (riid == IID_IKsPropertySet)
            *ppv = static_cast<IKsPropertySet*>(this);
        else if (riid == IID_IQualityControl)
            *ppv = static_cast<IQualityControl*>(this);
        else {
            *ppv = nullptr;
            return E_NOINTERFACE;
        }
        AddRef();
        return S_OK;
    }
    STDMETHODIMP_(ULONG) AddRef() override { return ULONG(++m_ref); }
    STDMETHODIMP_(ULONG) Release() override {
        LONG r = --m_ref;
        if (r == 0) delete this;
        return ULONG(r);
    }
    // IPin ----------------------------------------------------------------
    STDMETHODIMP Connect(IPin* pReceivePin, const AM_MEDIA_TYPE* pmt) override {
        if (!pReceivePin) return E_POINTER;
        if (m_pConnected) return VFW_E_ALREADY_CONNECTED;
        AM_MEDIA_TYPE mt = pmt ? *pmt : m_mtCurrent;
        HRESULT hr = pReceivePin->ReceiveConnection(this, &mt);
        if (FAILED(hr)) return hr;
        m_pConnected = pReceivePin;
        m_pConnected->AddRef();
        return S_OK;
    }
    STDMETHODIMP ReceiveConnection(IPin*, const AM_MEDIA_TYPE*) override {
        return E_UNEXPECTED;  // we're an output pin
    }
    STDMETHODIMP Disconnect() override {
        if (m_pConnected) { m_pConnected->Release(); m_pConnected = nullptr; }
        return S_OK;
    }
    STDMETHODIMP ConnectedTo(IPin** ppPin) override {
        if (!ppPin) return E_POINTER;
        *ppPin = m_pConnected;
        if (m_pConnected) m_pConnected->AddRef();
        return m_pConnected ? S_OK : VFW_E_NOT_CONNECTED;
    }
    STDMETHODIMP ConnectionMediaType(AM_MEDIA_TYPE* pmt) override {
        if (!pmt) return E_POINTER;
        if (!m_pConnected) return VFW_E_NOT_CONNECTED;
        *pmt = m_mtCurrent;
        if (m_mtCurrent.pbFormat) {
            pmt->pbFormat = static_cast<BYTE*>(CoTaskMemAlloc(m_mtCurrent.cbFormat));
            std::memcpy(pmt->pbFormat, m_mtCurrent.pbFormat, m_mtCurrent.cbFormat);
        }
        return S_OK;
    }
    STDMETHODIMP QueryPinInfo(PIN_INFO* pInfo) override {
        if (!pInfo) return E_POINTER;
        pInfo->pFilter = m_pFilter;
        if (m_pFilter) m_pFilter->AddRef();
        pInfo->dir = PINDIR_OUTPUT;
        wcscpy_s(pInfo->achName, kPinNameCapture);
        return S_OK;
    }
    STDMETHODIMP QueryDirection(PIN_DIRECTION* pPinDir) override {
        if (!pPinDir) return E_POINTER;
        *pPinDir = PINDIR_OUTPUT;
        return S_OK;
    }
    STDMETHODIMP QueryId(LPWSTR* Id) override {
        if (!Id) return E_POINTER;
        size_t cb = (wcslen(kPinNameCapture) + 1) * sizeof(WCHAR);
        *Id = static_cast<LPWSTR>(CoTaskMemAlloc(cb));
        wcscpy_s(*Id, cb / sizeof(WCHAR), kPinNameCapture);
        return S_OK;
    }
    STDMETHODIMP QueryAccept(const AM_MEDIA_TYPE* pmt) override {
        if (!pmt) return E_POINTER;
        return (pmt->majortype == MEDIATYPE_Video && pmt->subtype == MEDIASUBTYPE_UYVY)
                   ? S_OK : S_FALSE;
    }
    STDMETHODIMP EnumMediaTypes(IEnumMediaTypes** /*ppEnum*/) override {
        // TODO: enumerate UYVY 640x480 + any other formats we advertise.
        return E_NOTIMPL;
    }
    STDMETHODIMP QueryInternalConnections(IPin**, ULONG*) override { return E_NOTIMPL; }
    STDMETHODIMP EndOfStream() override { return S_OK; }
    STDMETHODIMP BeginFlush() override { return S_OK; }
    STDMETHODIMP EndFlush() override { return S_OK; }
    STDMETHODIMP NewSegment(REFERENCE_TIME, REFERENCE_TIME, double) override { return S_OK; }

    // IAMStreamConfig -----------------------------------------------------
    STDMETHODIMP SetFormat(AM_MEDIA_TYPE* pmt) override {
        if (!pmt) return E_POINTER;
        // SurfaceMount calls SetFormat(640x480 UYVY). Accept any UYVY format;
        // store it for ConnectionMediaType.
        if (pmt->majortype != MEDIATYPE_Video || pmt->subtype != MEDIASUBTYPE_UYVY)
            return VFW_E_INVALIDMEDIATYPE;
        free_media_type(&m_mtCurrent);
        m_mtCurrent = *pmt;
        if (pmt->pbFormat && pmt->cbFormat) {
            m_mtCurrent.pbFormat = static_cast<BYTE*>(CoTaskMemAlloc(pmt->cbFormat));
            std::memcpy(m_mtCurrent.pbFormat, pmt->pbFormat, pmt->cbFormat);
        }
        m_bFormatSet = TRUE;
        return S_OK;
    }
    STDMETHODIMP GetFormat(AM_MEDIA_TYPE** ppmt) override {
        if (!ppmt) return E_POINTER;
        *ppmt = static_cast<AM_MEDIA_TYPE*>(CoTaskMemAlloc(sizeof(AM_MEDIA_TYPE)));
        **ppmt = m_mtCurrent;
        if (m_mtCurrent.pbFormat) {
            (*ppmt)->pbFormat = static_cast<BYTE*>(CoTaskMemAlloc(m_mtCurrent.cbFormat));
            std::memcpy((*ppmt)->pbFormat, m_mtCurrent.pbFormat, m_mtCurrent.cbFormat);
        }
        return S_OK;
    }
    STDMETHODIMP GetNumberOfCapabilities(int* piCount, int* piSize) override {
        if (!piCount || !piSize) return E_POINTER;
        *piCount = 1;
        *piSize = sizeof(VIDEO_STREAM_CONFIG_CAPS);
        return S_OK;
    }
    STDMETHODIMP GetStreamCaps(int iIndex, AM_MEDIA_TYPE** ppmt, BYTE* pSCC) override {
        if (!ppmt || !pSCC) return E_POINTER;
        if (iIndex != 0) return S_FALSE;
        *ppmt = static_cast<AM_MEDIA_TYPE*>(CoTaskMemAlloc(sizeof(AM_MEDIA_TYPE)));
        set_default_format(*ppmt);
        auto* scc = reinterpret_cast<VIDEO_STREAM_CONFIG_CAPS*>(pSCC);
        std::memset(scc, 0, sizeof(*scc));
        scc->guid = FORMAT_VideoInfo;
        scc->VideoStandard = AnalogVideo_NTSC_M;
        scc->InputSize.cx = scc->MinOutputSize.cx = scc->MaxOutputSize.cx = 640;
        scc->InputSize.cy = scc->MinOutputSize.cy = scc->MaxOutputSize.cy = 480;
        scc->MinFrameInterval = scc->MaxFrameInterval = 333667;
        scc->MinBitsPerSecond = scc->MaxBitsPerSecond = 640 * 480 * 16 * 30;
        return S_OK;
    }

    // IKsPropertySet -- minimum: claim PIN_CATEGORY_CAPTURE so the pin is
    // recognized as a capture pin by DirectShow.
    STDMETHODIMP Set(REFGUID, DWORD, LPVOID, DWORD, LPVOID, DWORD) override {
        return E_NOTIMPL;
    }
    STDMETHODIMP Get(REFGUID PropSet, DWORD Id, LPVOID, DWORD, LPVOID pBuf,
                     DWORD cbBuf, DWORD* pcbReturned) override {
        if (PropSet != AMPROPSETID_Pin) return E_PROP_SET_UNSUPPORTED;
        if (Id != AMPROPERTY_PIN_CATEGORY) return E_PROP_ID_UNSUPPORTED;
        if (pcbReturned) *pcbReturned = sizeof(GUID);
        if (pBuf && cbBuf >= sizeof(GUID)) {
            *static_cast<GUID*>(pBuf) = PIN_CATEGORY_CAPTURE;
            return S_OK;
        }
        return cbBuf >= sizeof(GUID) ? S_OK : E_PROP_ID_UNSUPPORTED;
    }
    STDMETHODIMP QuerySupported(REFGUID PropSet, DWORD Id, DWORD* pTypeSupport) override {
        if (PropSet != AMPROPSETID_Pin) return E_PROP_SET_UNSUPPORTED;
        if (Id != AMPROPERTY_PIN_CATEGORY) return E_PROP_ID_UNSUPPORTED;
        if (pTypeSupport) *pTypeSupport = KSPROPERTY_SUPPORT_GET;
        return S_OK;
    }

    // IQualityControl -- accept all upstream feedback as no-op for now.
    STDMETHODIMP Notify(IBaseFilter*, Quality) override { return S_OK; }
    STDMETHODIMP SetSink(IQualityControl*) override { return S_OK; }
};

// =========================================================================
// Filter -- IBaseFilter + the IAMVideoProcAmp / IKsPropertySet hardware
// control surface. Most state lives here; the pin just forwards format
// negotiation and downstream connection.
// =========================================================================

class CMVisionGrabberFilter final : public IBaseFilter, public IAMVideoProcAmp {
    std::atomic<LONG> m_ref{1};
    CCaptureOutputPin* m_pPin = nullptr;
    IFilterGraph* m_pGraph = nullptr;          // weak ref per DirectShow rules
    IReferenceClock* m_pClock = nullptr;
    FILTER_STATE m_state = State_Stopped;
    REFERENCE_TIME m_tStart = 0;
    WCHAR m_wszName[MAX_FILTER_NAME] = {0};

   public:
    CMVisionGrabberFilter() {
        m_pPin = new CCaptureOutputPin(this);
    }
    ~CMVisionGrabberFilter() {
        if (m_pPin) m_pPin->Release();
        if (m_pClock) m_pClock->Release();
    }

    // IUnknown ------------------------------------------------------------
    STDMETHODIMP QueryInterface(REFIID riid, void** ppv) override {
        if (!ppv) return E_POINTER;
        if (riid == IID_IUnknown || riid == IID_IPersist || riid == IID_IMediaFilter ||
            riid == IID_IBaseFilter)
            *ppv = static_cast<IBaseFilter*>(this);
        else if (riid == IID_IAMVideoProcAmp)
            *ppv = static_cast<IAMVideoProcAmp*>(this);
        else {
            *ppv = nullptr;
            return E_NOINTERFACE;
        }
        AddRef();
        return S_OK;
    }
    STDMETHODIMP_(ULONG) AddRef() override { return ULONG(++m_ref); }
    STDMETHODIMP_(ULONG) Release() override {
        LONG r = --m_ref;
        if (r == 0) delete this;
        return ULONG(r);
    }
    // IPersist ------------------------------------------------------------
    STDMETHODIMP GetClassID(CLSID* pClsID) override {
        if (!pClsID) return E_POINTER;
        *pClsID = CLSID_MVisionGrabber;
        return S_OK;
    }
    // IMediaFilter --------------------------------------------------------
    STDMETHODIMP Stop() override {
        m_state = State_Stopped;
        // TODO: stop worker thread, cancel isoc URBs.
        return S_OK;
    }
    STDMETHODIMP Pause() override {
        m_state = State_Paused;
        return S_OK;
    }
    STDMETHODIMP Run(REFERENCE_TIME tStart) override {
        m_state = State_Running;
        m_tStart = tStart;
        // TODO: kick worker thread; samples start flowing via the pin.
        return S_OK;
    }
    STDMETHODIMP GetState(DWORD, FILTER_STATE* State) override {
        if (!State) return E_POINTER;
        *State = m_state;
        return S_OK;
    }
    STDMETHODIMP SetSyncSource(IReferenceClock* pClock) override {
        if (m_pClock) m_pClock->Release();
        m_pClock = pClock;
        if (m_pClock) m_pClock->AddRef();
        return S_OK;
    }
    STDMETHODIMP GetSyncSource(IReferenceClock** ppClock) override {
        if (!ppClock) return E_POINTER;
        *ppClock = m_pClock;
        if (m_pClock) m_pClock->AddRef();
        return S_OK;
    }
    // IBaseFilter ---------------------------------------------------------
    STDMETHODIMP EnumPins(IEnumPins** /*ppEnum*/) override {
        // TODO: return an IEnumPins enumerator over [m_pPin].
        return E_NOTIMPL;
    }
    STDMETHODIMP FindPin(LPCWSTR Id, IPin** ppPin) override {
        if (!Id || !ppPin) return E_POINTER;
        if (wcscmp(Id, kPinNameCapture) == 0) {
            *ppPin = m_pPin;
            m_pPin->AddRef();
            return S_OK;
        }
        *ppPin = nullptr;
        return VFW_E_NOT_FOUND;
    }
    STDMETHODIMP QueryFilterInfo(FILTER_INFO* pInfo) override {
        if (!pInfo) return E_POINTER;
        wcscpy_s(pInfo->achName, m_wszName);
        pInfo->pGraph = m_pGraph;
        if (m_pGraph) m_pGraph->AddRef();
        return S_OK;
    }
    STDMETHODIMP JoinFilterGraph(IFilterGraph* pGraph, LPCWSTR pName) override {
        m_pGraph = pGraph;  // weak ref per DirectShow contract
        if (pName) wcscpy_s(m_wszName, pName);
        return S_OK;
    }
    STDMETHODIMP QueryVendorInfo(LPWSTR* pVendorInfo) override {
        if (!pVendorInfo) return E_POINTER;
        const wchar_t* s = L"mvision_grabber (clean-room STK1150/SAA7113)";
        size_t cb = (wcslen(s) + 1) * sizeof(WCHAR);
        *pVendorInfo = static_cast<LPWSTR>(CoTaskMemAlloc(cb));
        wcscpy_s(*pVendorInfo, cb / sizeof(WCHAR), s);
        return S_OK;
    }

    // IAMVideoProcAmp -----------------------------------------------------
    // TODO: each Set() writes the corresponding SAA7113 register via WinUSB
    // → STK1160 I²C master. Get() reads back. Range/default values per
    // SAA7113 datasheet (already in memory reference_stk1160_saa7113_datasheets).
    STDMETHODIMP GetRange(LONG Property, LONG* pMin, LONG* pMax, LONG* pSteppingDelta,
                          LONG* pDefault, LONG* pCapsFlags) override {
        if (!pMin || !pMax || !pSteppingDelta || !pDefault || !pCapsFlags)
            return E_POINTER;
        *pSteppingDelta = 1;
        *pCapsFlags = VideoProcAmp_Flags_Manual | VideoProcAmp_Flags_Auto;
        switch (Property) {
            case VideoProcAmp_Brightness:
                *pMin = 0; *pMax = 255; *pDefault = 128; return S_OK;
            case VideoProcAmp_Contrast:
                *pMin = 0; *pMax = 127; *pDefault = 71;  return S_OK;
            case VideoProcAmp_Saturation:
                *pMin = 0; *pMax = 127; *pDefault = 64;  return S_OK;
            case VideoProcAmp_Hue:
                *pMin = -128; *pMax = 127; *pDefault = 0; return S_OK;
            case VideoProcAmp_Gain:
                *pMin = 0; *pMax = 511; *pDefault = 117; return S_OK;
            default: return E_PROP_ID_UNSUPPORTED;
        }
    }
    STDMETHODIMP Set(LONG /*Property*/, LONG /*lValue*/, LONG /*Flags*/) override {
        // TODO: route to SAA7113 register write via the shared common.cpp.
        return E_NOTIMPL;
    }
    STDMETHODIMP Get(LONG /*Property*/, LONG* /*lValue*/, LONG* /*Flags*/) override {
        // TODO: route to SAA7113 register read.
        return E_NOTIMPL;
    }
};

HRESULT CreateInstance(IUnknown* /*pUnkOuter*/, REFIID riid, void** ppv) {
    auto* p = new CMVisionGrabberFilter();
    if (!p) return E_OUTOFMEMORY;
    HRESULT hr = p->QueryInterface(riid, ppv);
    p->Release();
    return hr;
}

// =========================================================================
// Registration via IFilterMapper2 -- this is what makes our filter appear
// in `ICreateDevEnum.CreateClassEnumerator(CLSID_VideoInputDeviceCategory)`
// under the friendly name SurfaceMount string-matches on.
// =========================================================================

HRESULT RegisterServer(BOOL bRegister) {
    HRESULT hr = CoInitialize(nullptr);
    bool needUninit = SUCCEEDED(hr);

    IFilterMapper2* pFM = nullptr;
    hr = CoCreateInstance(CLSID_FilterMapper2, nullptr, CLSCTX_INPROC_SERVER,
                          IID_IFilterMapper2, reinterpret_cast<void**>(&pFM));
    if (FAILED(hr)) {
        if (needUninit) CoUninitialize();
        return hr;
    }

    if (bRegister) {
        REGFILTERPINS pinReg[1] = {};
        REGPINTYPES pinTypes[1] = { &MEDIATYPE_Video, &MEDIASUBTYPE_UYVY };
        pinReg[0].strName = const_cast<LPWSTR>(kPinNameCapture);
        pinReg[0].bRendered = FALSE;
        pinReg[0].bOutput = TRUE;
        pinReg[0].bZero = FALSE;
        pinReg[0].bMany = FALSE;
        pinReg[0].clsConnectsToFilter = nullptr;
        pinReg[0].strConnectsToPin = nullptr;
        pinReg[0].nMediaTypes = 1;
        pinReg[0].lpMediaType = pinTypes;

        REGFILTER2 regFilter = {};
        regFilter.dwVersion = 1;
        regFilter.dwMerit = MERIT_DO_NOT_USE;  // SurfaceMount finds us by
                                                // explicit moniker enum, so
                                                // merit doesn't matter.
        regFilter.cPins = 1;
        regFilter.rgPins = pinReg;

        hr = pFM->RegisterFilter(CLSID_MVisionGrabber, kFriendlyName, nullptr,
                                 &CLSID_VideoInputDeviceCategory, kFriendlyName,
                                 &regFilter);
    } else {
        hr = pFM->UnregisterFilter(&CLSID_VideoInputDeviceCategory, kFriendlyName,
                                   CLSID_MVisionGrabber);
    }

    pFM->Release();
    if (needUninit) CoUninitialize();
    return hr;
}

HRESULT WriteRegistryClsid(BOOL bRegister) {
    // Standard COM in-proc server registry layout.
    const wchar_t* clsidStr = L"{6F4E7B12-9C8E-4D3A-B5F6-1A2C3D4E5F60}";
    wchar_t keyPath[256];
    swprintf_s(keyPath, L"CLSID\\%s", clsidStr);

    if (bRegister) {
        HKEY hKey;
        if (RegCreateKeyExW(HKEY_CLASSES_ROOT, keyPath, 0, nullptr, 0, KEY_WRITE,
                            nullptr, &hKey, nullptr) != ERROR_SUCCESS)
            return SELFREG_E_CLASS;
        RegSetValueExW(hKey, nullptr, 0, REG_SZ,
                       reinterpret_cast<const BYTE*>(kFriendlyName),
                       DWORD((wcslen(kFriendlyName) + 1) * sizeof(WCHAR)));
        HKEY hSub;
        RegCreateKeyExW(hKey, L"InprocServer32", 0, nullptr, 0, KEY_WRITE, nullptr,
                        &hSub, nullptr);
        wchar_t modulePath[MAX_PATH];
        GetModuleFileNameW(g_hInst, modulePath, MAX_PATH);
        RegSetValueExW(hSub, nullptr, 0, REG_SZ,
                       reinterpret_cast<const BYTE*>(modulePath),
                       DWORD((wcslen(modulePath) + 1) * sizeof(WCHAR)));
        RegSetValueExW(hSub, L"ThreadingModel", 0, REG_SZ,
                       reinterpret_cast<const BYTE*>(L"Both"), sizeof(L"Both"));
        RegCloseKey(hSub);
        RegCloseKey(hKey);
    } else {
        wchar_t sub[256];
        swprintf_s(sub, L"%s\\InprocServer32", keyPath);
        RegDeleteKeyW(HKEY_CLASSES_ROOT, sub);
        RegDeleteKeyW(HKEY_CLASSES_ROOT, keyPath);
    }
    return S_OK;
}

}  // namespace

// =========================================================================
// DLL exports.
// =========================================================================

extern "C" {

BOOL APIENTRY DllMain(HINSTANCE hInst, DWORD reason, LPVOID) {
    if (reason == DLL_PROCESS_ATTACH) {
        g_hInst = hInst;
        DisableThreadLibraryCalls(hInst);
    }
    return TRUE;
}

STDAPI DllGetClassObject(REFCLSID rclsid, REFIID riid, LPVOID* ppv) {
    if (!ppv) return E_POINTER;
    if (rclsid != CLSID_MVisionGrabber) return CLASS_E_CLASSNOTAVAILABLE;
    auto* p = new CClassFactory();
    HRESULT hr = p->QueryInterface(riid, ppv);
    p->Release();
    return hr;
}

STDAPI DllCanUnloadNow() {
    return g_serverLockCount == 0 ? S_OK : S_FALSE;
}

STDAPI DllRegisterServer() {
    HRESULT hr = WriteRegistryClsid(TRUE);
    if (FAILED(hr)) return hr;
    return RegisterServer(TRUE);
}

STDAPI DllUnregisterServer() {
    HRESULT hr1 = RegisterServer(FALSE);
    HRESULT hr2 = WriteRegistryClsid(FALSE);
    return SUCCEEDED(hr1) ? hr2 : hr1;
}

}  // extern "C"
