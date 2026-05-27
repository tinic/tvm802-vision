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
#define NOMINMAX  // keep windows.h from defining min/max macros
#define INITGUID
// winsock2.h must come BEFORE windows.h. We don't actually use winsock, but
// libusb.h declares libusb_handle_events_timeout_completed with `struct
// timeval *` and expects the caller to have the type already; on Windows
// that lives in winsock2.h, not windows.h (WIN32_LEAN_AND_MEAN excludes
// the older winsock.h). mingw's sys/time.h compat header includes it
// transitively, which is why the syntax-check passed there.
#include <winsock2.h>
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

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

// Shared WinUSB + STK1160 I2C-master code from the saa7113-tune tool.
// The build script flattens both source trees into the build directory.
#include "common.h"

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

// =========================================================================
// Tiny enumerator helpers -- IEnumPins (one entry) and IEnumMediaTypes
// (one entry). DirectShow contract requires these to be Reset()/Skip()/
// Clone()-able; we implement the minimum.
// =========================================================================

template <typename TIface, typename TItem>
class CSingleEnum final : public TIface {
    std::atomic<LONG> m_ref{1};
    TItem m_item;
    ULONG m_pos = 0;  // 0 = not yet emitted, 1 = emitted
    void item_addref() {
        if constexpr (std::is_same_v<TItem, IPin*>) {
            if (m_item) m_item->AddRef();
        }
    }
    void item_release() {
        if constexpr (std::is_same_v<TItem, IPin*>) {
            if (m_item) m_item->Release();
        }
    }

   public:
    explicit CSingleEnum(TItem item) : m_item(item) { item_addref(); }
    ~CSingleEnum() { item_release(); }

    STDMETHODIMP QueryInterface(REFIID riid, void** ppv) override {
        if (!ppv) return E_POINTER;
        if (riid == IID_IUnknown || riid == __uuidof(TIface)) {
            *ppv = static_cast<TIface*>(this);
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
    STDMETHODIMP Reset() override { m_pos = 0; return S_OK; }
    STDMETHODIMP Skip(ULONG cElt) override {
        if (m_pos + cElt > 1) { m_pos = 1; return S_FALSE; }
        m_pos += cElt;
        return S_OK;
    }
    STDMETHODIMP Clone(TIface** ppEnum) override {
        if (!ppEnum) return E_POINTER;
        auto* c = new CSingleEnum(m_item);
        c->m_pos = m_pos;
        *ppEnum = c;
        return S_OK;
    }
};

// CEnumPins: holds a strong ref to the single IPin, emits it once.
class CEnumPins final : public IEnumPins {
    std::atomic<LONG> m_ref{1};
    IPin* m_pPin = nullptr;
    ULONG m_pos = 0;
   public:
    explicit CEnumPins(IPin* p) : m_pPin(p) { if (m_pPin) m_pPin->AddRef(); }
    ~CEnumPins() { if (m_pPin) m_pPin->Release(); }
    STDMETHODIMP QueryInterface(REFIID riid, void** ppv) override {
        if (!ppv) return E_POINTER;
        if (riid == IID_IUnknown || riid == IID_IEnumPins) {
            *ppv = static_cast<IEnumPins*>(this);
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
    STDMETHODIMP Next(ULONG cPins, IPin** ppPins, ULONG* pcFetched) override {
        if (!ppPins) return E_POINTER;
        ULONG fetched = 0;
        if (m_pos == 0 && cPins >= 1 && m_pPin) {
            m_pPin->AddRef();
            ppPins[0] = m_pPin;
            m_pos = 1;
            fetched = 1;
        }
        if (pcFetched) *pcFetched = fetched;
        return fetched == cPins ? S_OK : S_FALSE;
    }
    STDMETHODIMP Skip(ULONG cPins) override {
        if (m_pos + cPins > 1) { m_pos = 1; return S_FALSE; }
        m_pos += cPins;
        return S_OK;
    }
    STDMETHODIMP Reset() override { m_pos = 0; return S_OK; }
    STDMETHODIMP Clone(IEnumPins** ppEnum) override {
        if (!ppEnum) return E_POINTER;
        auto* c = new CEnumPins(m_pPin);
        c->m_pos = m_pos;
        *ppEnum = c;
        return S_OK;
    }
};

// Forward decl for set_default_format -- used by CEnumMediaTypes.
void make_default_media_type(AM_MEDIA_TYPE* mt);

class CEnumMediaTypes final : public IEnumMediaTypes {
    std::atomic<LONG> m_ref{1};
    ULONG m_pos = 0;
   public:
    STDMETHODIMP QueryInterface(REFIID riid, void** ppv) override {
        if (!ppv) return E_POINTER;
        if (riid == IID_IUnknown || riid == IID_IEnumMediaTypes) {
            *ppv = static_cast<IEnumMediaTypes*>(this);
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
    STDMETHODIMP Next(ULONG cMediaTypes, AM_MEDIA_TYPE** ppMediaTypes,
                      ULONG* pcFetched) override {
        if (!ppMediaTypes) return E_POINTER;
        ULONG fetched = 0;
        if (m_pos == 0 && cMediaTypes >= 1) {
            auto* mt = static_cast<AM_MEDIA_TYPE*>(CoTaskMemAlloc(sizeof(AM_MEDIA_TYPE)));
            make_default_media_type(mt);
            ppMediaTypes[0] = mt;
            m_pos = 1;
            fetched = 1;
        }
        if (pcFetched) *pcFetched = fetched;
        return fetched == cMediaTypes ? S_OK : S_FALSE;
    }
    STDMETHODIMP Skip(ULONG cMediaTypes) override {
        if (m_pos + cMediaTypes > 1) { m_pos = 1; return S_FALSE; }
        m_pos += cMediaTypes;
        return S_OK;
    }
    STDMETHODIMP Reset() override { m_pos = 0; return S_OK; }
    STDMETHODIMP Clone(IEnumMediaTypes** ppEnum) override {
        if (!ppEnum) return E_POINTER;
        auto* c = new CEnumMediaTypes();
        c->m_pos = m_pos;
        *ppEnum = c;
        return S_OK;
    }
};

void make_default_media_type(AM_MEDIA_TYPE* mt) {
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
    vih->AvgTimePerFrame = 333667;  // ~30 fps
    vih->bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    vih->bmiHeader.biWidth = 640;
    vih->bmiHeader.biHeight = 480;
    vih->bmiHeader.biPlanes = 1;
    vih->bmiHeader.biBitCount = 16;
    vih->bmiHeader.biCompression = MAKEFOURCC('U', 'Y', 'V', 'Y');
    vih->bmiHeader.biSizeImage = 640 * 480 * 2;
    mt->pbFormat = reinterpret_cast<BYTE*>(vih);
}

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

// SEH-safe COM Release. SurfaceMount + DirectShowLib-2005 sometimes release
// downstream filters before our dtor runs (against the documented
// "both pins hold refs" contract). Releasing a dangling COM pointer faults
// the whole process on form-close. This helper catches the AV so we can
// no-op when downstream is dead and release cleanly when it isn't.
// Must live in a function with no C++ object unwinding (no destructors)
// hence the standalone helper.
static void safe_release(IUnknown* p) {
    if (!p) return;
    __try { p->Release(); }
    __except (EXCEPTION_EXECUTE_HANDLER) {
        OutputDebugStringA("[mvision_grabber] safe_release: Release faulted; ignored\n");
    }
}

// Free a media type's allocated pointers in place (does NOT free the struct).
inline void free_media_type(AM_MEDIA_TYPE* mt) {
    if (mt->pbFormat) { CoTaskMemFree(mt->pbFormat); mt->pbFormat = nullptr; }
    if (mt->pUnk)     { mt->pUnk->Release();          mt->pUnk = nullptr; }
    mt->cbFormat = 0;
}

class CCaptureOutputPin final : public IPin,
                                public IAMStreamConfig,
                                public IKsPropertySet,
                                public IQualityControl {
    std::atomic<LONG> m_ref{1};
    std::atomic<bool> m_cleanedUp{false};
    IBaseFilter* m_pFilter = nullptr;          // owner; weak ref
    // The three "connection" pointers below are read by the worker thread
    // (deliver_frame) and reset by Disconnect / ~CCaptureOutputPin on the
    // main thread. Without a lock, the worker can dereference a pointer
    // that's about to be released -> crash on graph teardown. The mutex
    // is mutable so const accessors (used by snapshot helpers) can lock.
    mutable std::mutex m_connMu;
    IPin* m_pConnected = nullptr;              // downstream; strong ref
    IMemInputPin* m_pInput = nullptr;          // downstream's IMemInputPin
    IMemAllocator* m_pAllocator = nullptr;     // negotiated allocator
    AM_MEDIA_TYPE m_mtCurrent{};               // currently negotiated format
    BOOL m_bFormatSet = FALSE;

   public:
    explicit CCaptureOutputPin(IBaseFilter* pFilter) : m_pFilter(pFilter) {
        make_default_media_type(&m_mtCurrent);
    }
    // Dtor never runs in practice -- see Release(). Keep it correct for
    // any explicit `delete` path (we don't take one).
    ~CCaptureOutputPin() {
        if (!m_cleanedUp.exchange(true)) pin_cleanup();
        free_media_type(&m_mtCurrent);
    }

    void pin_cleanup() {
        OutputDebugStringA("[mvision_grabber] Pin cleanup\n");
        IMemAllocator* a = nullptr;
        IMemInputPin*  i = nullptr;
        IPin*          c = nullptr;
        {
            std::lock_guard lk(m_connMu);
            a = m_pAllocator; m_pAllocator = nullptr;
            i = m_pInput;     m_pInput = nullptr;
            c = m_pConnected; m_pConnected = nullptr;
        }
        // See safe_release() comment: SurfaceMount/DSLib teardown order
        // can dangle downstream by this point.
        safe_release(a);
        safe_release(i);
        safe_release(c);
    }

    // Atomically snapshot+AddRef the two pointers the worker needs.
    // Returns false if the pin is currently disconnected. Caller releases.
    bool snapshot_input_alloc(IMemInputPin** ppIn, IMemAllocator** ppAlloc) {
        std::lock_guard lk(m_connMu);
        if (!m_pInput || !m_pAllocator) return false;
        *ppIn = m_pInput;     (*ppIn)->AddRef();
        *ppAlloc = m_pAllocator; (*ppAlloc)->AddRef();
        return true;
    }
    const AM_MEDIA_TYPE& current_format() const { return m_mtCurrent; }

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
        // LEAK ON RELEASE -- see the long comment on the filter's Release.
        // Run cleanup once when refcount first hits 0, but never `delete
        // this`, so a subsequent over-Release just decrements a still-valid
        // atomic instead of touching freed memory.
        LONG r = --m_ref;
        if (r == 0 && !m_cleanedUp.exchange(true)) pin_cleanup();
        if (r < 0) m_ref = 0;
        return r > 0 ? ULONG(r) : 0;
    }
    // IPin ----------------------------------------------------------------
    STDMETHODIMP Connect(IPin* pReceivePin, const AM_MEDIA_TYPE* pmt) override {
        if (!pReceivePin) return E_POINTER;
        if (m_pConnected) return VFW_E_ALREADY_CONNECTED;
        AM_MEDIA_TYPE mt = pmt ? *pmt : m_mtCurrent;
        HRESULT hr = pReceivePin->ReceiveConnection(this, &mt);
        if (FAILED(hr)) return hr;

        // Get downstream IMemInputPin for sample delivery.
        IMemInputPin* pIn = nullptr;
        hr = pReceivePin->QueryInterface(IID_IMemInputPin,
                                         reinterpret_cast<void**>(&pIn));
        if (FAILED(hr)) {
            pReceivePin->Disconnect();
            return hr;
        }

        // Negotiate the allocator. Try downstream's first; if it refuses to
        // provide one, create the system memory allocator ourselves.
        IMemAllocator* pAlloc = nullptr;
        if (FAILED(pIn->GetAllocator(&pAlloc))) {
            CoCreateInstance(CLSID_MemoryAllocator, nullptr, CLSCTX_INPROC_SERVER,
                             IID_IMemAllocator, reinterpret_cast<void**>(&pAlloc));
        }
        if (!pAlloc) {
            pIn->Release();
            pReceivePin->Disconnect();
            return E_FAIL;
        }
        ALLOCATOR_PROPERTIES req{};
        req.cBuffers = 4;
        req.cbBuffer = 640 * 480 * 2;
        req.cbAlign = 1;
        req.cbPrefix = 0;
        ALLOCATOR_PROPERTIES got{};
        hr = pAlloc->SetProperties(&req, &got);
        if (SUCCEEDED(hr)) hr = pIn->NotifyAllocator(pAlloc, FALSE);
        if (SUCCEEDED(hr)) hr = pAlloc->Commit();
        if (FAILED(hr)) {
            pAlloc->Release();
            pIn->Release();
            pReceivePin->Disconnect();
            return hr;
        }

        {
            std::lock_guard lk(m_connMu);
            m_pConnected = pReceivePin;
            m_pConnected->AddRef();
            m_pInput = pIn;            // ownership transferred
            m_pAllocator = pAlloc;     // ownership transferred
        }
        return S_OK;
    }
    STDMETHODIMP ReceiveConnection(IPin*, const AM_MEDIA_TYPE*) override {
        return E_UNEXPECTED;  // we're an output pin
    }
    STDMETHODIMP Disconnect() override {
        // Locally extract then null out under the lock; do the releases
        // after the lock to avoid holding it across Decommit (which can
        // synchronize with the downstream allocator).
        IMemAllocator* a = nullptr;
        IMemInputPin* i = nullptr;
        IPin* c = nullptr;
        {
            std::lock_guard lk(m_connMu);
            a = m_pAllocator; m_pAllocator = nullptr;
            i = m_pInput;     m_pInput = nullptr;
            c = m_pConnected; m_pConnected = nullptr;
        }
        if (a) { a->Decommit(); a->Release(); }
        if (i) i->Release();
        if (c) c->Release();
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
    STDMETHODIMP EnumMediaTypes(IEnumMediaTypes** ppEnum) override {
        if (!ppEnum) return E_POINTER;
        *ppEnum = new CEnumMediaTypes();
        return S_OK;
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
        make_default_media_type(*ppmt);
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

// =========================================================================
// Filter: owns the worker thread + the WinUSB capture state. The pin is
// just for graph negotiation; samples flow from the worker through the
// pin's allocator/IMemInputPin to downstream.
// =========================================================================

class CMVisionGrabberFilter final : public IBaseFilter, public IAMVideoProcAmp {
    std::atomic<LONG> m_ref{1};
    std::atomic<bool> m_cleanedUp{false};
    CCaptureOutputPin* m_pPin = nullptr;
    IFilterGraph* m_pGraph = nullptr;          // weak ref per DirectShow rules
    IReferenceClock* m_pClock = nullptr;
    FILTER_STATE m_state = State_Stopped;
    REFERENCE_TIME m_tStart = 0;
    WCHAR m_wszName[MAX_FILTER_NAME] = {0};

    // ---- WinUSB capture state (worker thread) ---------------------------
    std::thread m_capThread;
    std::atomic<bool> m_capRun{false};
    std::mutex m_usbMu;                // guards usb access from
                                        // IAMVideoProcAmp + worker
    saa::UsbCtx m_usb;                 // opened on first Run(), released
                                        // in Stop()/dtor
    bool m_usbOpen = false;
    REFERENCE_TIME m_streamStart = 0;  // sample time origin (from Run())

    // Frame-assembly state. Chip emits BOTH 0xc0 (odd field + frame start)
    // and 0x80 (even field) markers; we accumulate two interleaved fields
    // into a 720x480 buffer per the Linux stk1160-video.c convention.
    static constexpr int kSrcW = 720;
    static constexpr int kSrcH = 480;          // interleaved frame
    static constexpr int kSrcBpl = kSrcW * 2;  // UYVY
    std::vector<std::uint8_t> m_assembly;      // 720x480 UYVY accumulator
    int m_asmPos = 0;
    bool m_asmOdd = true;                      // current field parity
    REFERENCE_TIME m_frameInterval = 333667;   // 100ns units; ~30 fps

    // libusb isoc context.
    static constexpr std::uint8_t kEpVideo = 0x82;
    static constexpr int kNumXfers = 8;
    static constexpr int kNumPkts  = 64;
    int m_isocMaxPkt = 0;

   public:
    CMVisionGrabberFilter() {
        m_pPin = new CCaptureOutputPin(this);
        m_assembly.assign(std::size_t(kSrcBpl * kSrcH), 0);
    }
    // Dtor never runs in practice -- see Release(). Keep it correct for
    // any explicit `delete` path (we don't take one).
    ~CMVisionGrabberFilter() {
        if (!m_cleanedUp.exchange(true)) filter_cleanup();
    }

    void filter_cleanup() {
        OutputDebugStringA("[mvision_grabber] Filter cleanup: stop_worker\n");
        stop_worker();
        OutputDebugStringA("[mvision_grabber] Filter cleanup: pin release\n");
        if (m_pPin) { m_pPin->Release(); m_pPin = nullptr; }
        OutputDebugStringA("[mvision_grabber] Filter cleanup: clock release\n");
        safe_release(m_pClock);
        m_pClock = nullptr;
        // m_usb closes when filter_cleanup ends only if we'd actually destroy
        // the object -- since we leak, we close USB explicitly here.
        if (m_usbOpen) {
            libusb_release_interface(m_usb.h, 0);
            // m_usb's UsbCtx dtor handles libusb_close + libusb_exit when
            // the object actually gets freed. Since we leak the filter,
            // we'd never close USB; do it now manually.
            if (m_usb.h) { libusb_close(m_usb.h); m_usb.h = nullptr; }
            if (m_usb.inited) { libusb_exit(nullptr); m_usb.inited = false; }
            m_usbOpen = false;
        }
        OutputDebugStringA("[mvision_grabber] Filter cleanup: done\n");
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
        // LEAK ON RELEASE. Long story: DirectShowLib-2005 + SurfaceMount's
        // form-close path Releases this filter one too many times -- some
        // .NET wrapper held a native pointer past its true lifetime. With
        // the textbook `if (r == 0) delete this`, the over-release crashed
        // at `--m_ref` on freed memory. We can't fix DSLib, so instead we
        // never free the object. On the FIRST hit to zero we run cleanup
        // (stop worker, close USB, release downstream refs); subsequent
        // Release calls just decrement a still-valid std::atomic and
        // return 0. The leak is one filter object per SurfaceMount session
        // (a few hundred bytes plus a closed UsbCtx), which is negligible.
        LONG r = --m_ref;
        if (r == 0 && !m_cleanedUp.exchange(true)) filter_cleanup();
        if (r < 0) m_ref = 0;
        return r > 0 ? ULONG(r) : 0;
    }
    // IPersist ------------------------------------------------------------
    STDMETHODIMP GetClassID(CLSID* pClsID) override {
        if (!pClsID) return E_POINTER;
        *pClsID = CLSID_MVisionGrabber;
        return S_OK;
    }
    // IMediaFilter --------------------------------------------------------
    STDMETHODIMP Stop() override {
        // CRITICAL ORDER: flip state to Stopped BEFORE joining the worker.
        // deliver_frame() checks m_state and early-returns if Stopped, so
        // setting it first guarantees no further Receive() calls after
        // this point (which could otherwise deadlock against the graph
        // mutex while we wait on join).
        m_state = State_Stopped;
        stop_worker();
        return S_OK;
    }
    STDMETHODIMP Pause() override {
        // DirectShow contract: Pause = "ready to deliver"; for a live
        // source we typically start running here so the first frame is
        // available when Run() flips the play state.
        if (m_state == State_Stopped) start_worker();
        m_state = State_Paused;
        return S_OK;
    }
    STDMETHODIMP Run(REFERENCE_TIME tStart) override {
        if (m_state == State_Stopped) start_worker();
        m_streamStart = tStart;
        m_state = State_Running;
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
    STDMETHODIMP EnumPins(IEnumPins** ppEnum) override {
        if (!ppEnum) return E_POINTER;
        *ppEnum = new CEnumPins(static_cast<IPin*>(m_pPin));
        return S_OK;
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
    STDMETHODIMP Set(LONG Property, LONG lValue, LONG /*Flags*/) override {
        // Hardware-real: every standard IAMVideoProcAmp property maps to a
        // SAA7113 register (Brightness=0x0A, Contrast=0x0B, Saturation=0x0C,
        // Hue=0x0D). Gain is the 9-bit GAI18:GAI10/GAI28:GAI20 split with
        // GAFIX in reg 0x03 bit 2.
        std::lock_guard lk(m_usbMu);
        if (!ensure_usb_open()) return E_FAIL;
        const std::uint8_t addr = saa::kSaaDefaultAddr;
        auto h = m_usb.h;
        switch (Property) {
            case VideoProcAmp_Brightness:
                return saa::saa_write(h, addr, saa::kSaaBrightness,
                                      std::uint8_t(std::clamp<LONG>(lValue, 0, 255))) >= 0
                           ? S_OK : E_FAIL;
            case VideoProcAmp_Contrast:
                return saa::saa_write(h, addr, saa::kSaaContrast,
                                      std::uint8_t(std::clamp<LONG>(lValue, 0, 127))) >= 0
                           ? S_OK : E_FAIL;
            case VideoProcAmp_Saturation:
                return saa::saa_write(h, addr, saa::kSaaSaturation,
                                      std::uint8_t(std::clamp<LONG>(lValue, 0, 127))) >= 0
                           ? S_OK : E_FAIL;
            case VideoProcAmp_Hue:
                return saa::saa_write(h, addr, saa::kSaaHue,
                                      std::uint8_t(std::clamp<LONG>(lValue, -128, 127))) >= 0
                           ? S_OK : E_FAIL;
            case VideoProcAmp_Gain: {
                LONG g = std::clamp<LONG>(lValue, 0, 511);
                std::uint8_t cur = 0;
                if (saa::saa_read(h, addr, saa::kSaaInputCntl2, cur) < 0) return E_FAIL;
                std::uint8_t nv = std::uint8_t(
                    (cur & ~(saa::kSaaGafix | saa::kSaaGai18 | saa::kSaaGai28))
                    | saa::kSaaGafix
                    | ((g & 0x100) ? saa::kSaaGai18 : 0)
                    | ((g & 0x100) ? saa::kSaaGai28 : 0));
                if (saa::saa_write(h, addr, saa::kSaaInputCntl2, nv) < 0) return E_FAIL;
                if (saa::saa_write(h, addr, saa::kSaaGainCh1Lo, std::uint8_t(g & 0xff)) < 0) return E_FAIL;
                if (saa::saa_write(h, addr, saa::kSaaGainCh2Lo, std::uint8_t(g & 0xff)) < 0) return E_FAIL;
                return S_OK;
            }
            default:
                return E_PROP_ID_UNSUPPORTED;
        }
    }
    STDMETHODIMP Get(LONG Property, LONG* lValue, LONG* Flags) override {
        if (!lValue) return E_POINTER;
        std::lock_guard lk(m_usbMu);
        if (!ensure_usb_open()) return E_FAIL;
        const std::uint8_t addr = saa::kSaaDefaultAddr;
        auto h = m_usb.h;
        std::uint8_t v = 0;
        std::uint8_t reg = 0;
        switch (Property) {
            case VideoProcAmp_Brightness: reg = saa::kSaaBrightness; break;
            case VideoProcAmp_Contrast:   reg = saa::kSaaContrast;   break;
            case VideoProcAmp_Saturation: reg = saa::kSaaSaturation; break;
            case VideoProcAmp_Hue:        reg = saa::kSaaHue;        break;
            case VideoProcAmp_Gain: {
                std::uint8_t inp2 = 0;
                std::uint8_t lo = 0;
                if (saa::saa_read(h, addr, saa::kSaaInputCntl2, inp2) < 0) return E_FAIL;
                if (saa::saa_read(h, addr, saa::kSaaGainCh1Lo, lo)    < 0) return E_FAIL;
                *lValue = lo | ((inp2 & saa::kSaaGai18) ? 0x100 : 0);
                if (Flags) *Flags = (inp2 & saa::kSaaGafix) ? VideoProcAmp_Flags_Manual
                                                            : VideoProcAmp_Flags_Auto;
                return S_OK;
            }
            default: return E_PROP_ID_UNSUPPORTED;
        }
        if (saa::saa_read(h, addr, reg, v) < 0) return E_FAIL;
        *lValue = (Property == VideoProcAmp_Hue) ? LONG(int8_t(v)) : LONG(v);
        if (Flags) *Flags = VideoProcAmp_Flags_Manual;
        return S_OK;
    }

   private:
    // ---- worker thread + USB capture ------------------------------------
    bool ensure_usb_open() {
        if (m_usbOpen) return true;
        if (saa::open_device(m_usb) < 0) return false;
        // Set the alt setting on interface 0 to the isoc-bandwidth one
        // (alt 5 on this chip; same as saa7113-tune live mode). Use the
        // raw SET_INTERFACE control transfer -- libusb's
        // set_interface_alt_setting is fussy on per-interface composite
        // WinUSB bindings; raw works everywhere.
        // Pick alt setting by walking the config descriptor.
        int alt = -1;
        libusb_device* dev = libusb_get_device(m_usb.h);
        libusb_config_descriptor* cfg = nullptr;
        if (libusb_get_active_config_descriptor(dev, &cfg) == 0 && cfg) {
            for (int ii = 0; ii < cfg->bNumInterfaces && alt < 0; ++ii) {
                const auto& intf = cfg->interface[ii];
                for (int a = 0; a < intf.num_altsetting; ++a) {
                    const auto& as = intf.altsetting[a];
                    for (int e = 0; e < as.bNumEndpoints; ++e) {
                        const auto& ep = as.endpoint[e];
                        if (ep.bEndpointAddress != kEpVideo) continue;
                        int sz = (ep.wMaxPacketSize & 0x07ff) *
                                 (((ep.wMaxPacketSize >> 11) & 0x3) + 1);
                        if (sz > m_isocMaxPkt) {
                            m_isocMaxPkt = sz;
                            alt = as.bAlternateSetting;
                        }
                    }
                }
            }
            libusb_free_config_descriptor(cfg);
        }
        if (alt < 0) return false;
        libusb_claim_interface(m_usb.h, 0);
        libusb_control_transfer(m_usb.h, 0x01, 0x0B, std::uint16_t(alt), 0,
                                nullptr, 0, 1000);
        // Capture-window registers (NTSC 720x480, same as saa7113-tune).
        struct RV { std::uint16_t r; std::uint8_t v; };
        const RV cap[] = {
            {0x110, 0x00}, {0x111, 0x00}, {0x112, 0x03}, {0x113, 0x00},
            {0x114, 0xa0}, {0x115, 0x05}, {0x116, 0xf3}, {0x117, 0x00},
        };
        for (auto& rv : cap) saa::stk_write_reg(m_usb.h, rv.r, rv.v);
        m_usbOpen = true;
        return true;
    }

    void start_worker() {
        {
            std::lock_guard lk(m_usbMu);
            if (!ensure_usb_open()) return;
        }
        m_capRun.store(true);
        m_capThread = std::thread([this] { capture_loop(); });
    }
    void stop_worker() {
        m_capRun.store(false);
        if (m_capThread.joinable()) m_capThread.join();
        // Don't close USB here -- IAMVideoProcAmp may still use it.
    }

    // Isoc callback: receives packets, calls into process_packet via
    // user_data = this.
    static void LIBUSB_CALL on_isoc(libusb_transfer* xfer);
    void process_packet(const std::uint8_t* p, int len);
    void deliver_frame();

    // Capture loop: submits isoc URBs, kicks streaming, pumps events.
    void capture_loop() {
        std::vector<libusb_transfer*> xfers(kNumXfers, nullptr);
        std::vector<std::vector<std::uint8_t>> bufs(kNumXfers);
        for (int i = 0; i < kNumXfers; ++i) {
            xfers[i] = libusb_alloc_transfer(kNumPkts);
            bufs[i].assign(std::size_t(kNumPkts * m_isocMaxPkt), 0);
            libusb_fill_iso_transfer(xfers[i], m_usb.h, kEpVideo,
                                     bufs[i].data(), int(bufs[i].size()),
                                     kNumPkts, &CMVisionGrabberFilter::on_isoc,
                                     this, 1000);
            libusb_set_iso_packet_lengths(xfers[i], std::uint32_t(m_isocMaxPkt));
            libusb_submit_transfer(xfers[i]);
        }
        // Kick streaming AFTER URBs are pending.
        saa::stk_write_reg(m_usb.h, 0x100, 0xb3);
        saa::stk_write_reg(m_usb.h, 0x103, 0x00);

        while (m_capRun.load()) {
            timeval tv{0, 50000};
            libusb_handle_events_timeout_completed(nullptr, &tv, nullptr);
        }

        for (auto* x : xfers) if (x) libusb_cancel_transfer(x);
        timeval tv{0, 500000};
        libusb_handle_events_timeout_completed(nullptr, &tv, nullptr);
        for (auto* x : xfers) if (x) libusb_free_transfer(x);
    }
};

void CMVisionGrabberFilter::on_isoc(libusb_transfer* xfer) {
    auto* self = static_cast<CMVisionGrabberFilter*>(xfer->user_data);
    if (xfer->status == LIBUSB_TRANSFER_CANCELLED ||
        xfer->status == LIBUSB_TRANSFER_NO_DEVICE) return;
    for (int i = 0; i < xfer->num_iso_packets; ++i) {
        auto& d = xfer->iso_packet_desc[i];
        if (d.status != LIBUSB_TRANSFER_COMPLETED) continue;
        const std::uint8_t* p =
            libusb_get_iso_packet_buffer_simple(xfer, std::uint32_t(i));
        self->process_packet(p, int(d.actual_length));
    }
    if (self->m_capRun.load()) libusb_submit_transfer(xfer);
}

void CMVisionGrabberFilter::process_packet(const std::uint8_t* p, int len) {
    if (len <= 4) return;
    if (p[0] == 0xc0) {
        // 0xc0 = end-of-frame + start of odd field for the next one.
        deliver_frame();
        m_asmPos = 0;
        m_asmOdd = (p[0] & 0x40) != 0;
        return;
    }
    if (p[0] == 0x80) {
        // 0x80 = start of even field within the same frame.
        m_asmPos = 0;
        m_asmOdd = (p[0] & 0x40) != 0;
        return;
    }
    // Continuation -- skip 4-byte header, interleave by field parity:
    // odd field -> destination lines 0, 2, 4, ...; even -> 1, 3, 5, ...
    int remain = len - 4;
    const std::uint8_t* src = p + 4;
    int linesdone = m_asmPos / kSrcBpl;
    int lineoff = m_asmPos % kSrcBpl;
    int dstLine = linesdone * 2 + (m_asmOdd ? 0 : 1);
    int dstOff = dstLine * kSrcBpl + lineoff;
    while (remain > 0) {
        if (dstOff >= int(m_assembly.size())) return;
        int lencopy = std::min(remain, kSrcBpl - lineoff);
        if (dstOff + lencopy > int(m_assembly.size())) {
            lencopy = int(m_assembly.size()) - dstOff;
        }
        if (lencopy <= 0) return;
        std::memcpy(m_assembly.data() + dstOff, src, std::size_t(lencopy));
        src += lencopy;
        remain -= lencopy;
        m_asmPos += lencopy;
        lineoff = 0;
        ++linesdone;
        dstLine = linesdone * 2 + (m_asmOdd ? 0 : 1);
        dstOff = dstLine * kSrcBpl;
    }
}

void CMVisionGrabberFilter::deliver_frame() {
    if (m_state == State_Stopped) return;
    auto* pin = m_pPin;
    if (!pin) return;

    // Atomically AddRef the pin's connection state -- otherwise Disconnect
    // on the main thread can Release them while we're using them. Cleared
    // up before this function returns.
    IMemInputPin* in = nullptr;
    IMemAllocator* alloc = nullptr;
    if (!pin->snapshot_input_alloc(&in, &alloc)) return;

    IMediaSample* sample = nullptr;
    if (FAILED(alloc->GetBuffer(&sample, nullptr, nullptr, 0))) {
        in->Release(); alloc->Release(); return;
    }
    BYTE* dst = nullptr;
    if (FAILED(sample->GetPointer(&dst))) {
        sample->Release(); in->Release(); alloc->Release(); return;
    }
    const LONG cap = sample->GetSize();
    const int dstW = 640, dstH = 480, dstBpl = dstW * 2;
    if (cap < dstBpl * dstH) {
        sample->Release(); in->Release(); alloc->Release(); return;
    }

    // 720 source -> 640 destination by center-CROP, not squeeze: drop 40
    // pixels (= 80 bytes UYVY) from each side. The dropped pixels are
    // ITU-656 active-blank margins, not real image, so cropping preserves
    // the 4:3 aspect SurfaceMount expects. Vertical is 1:1.
    constexpr int kXOffset = ((kSrcW - 640) / 2) * 2;  // 80 bytes
    for (int dy = 0; dy < dstH; ++dy) {
        const std::uint8_t* sline = m_assembly.data() + dy * kSrcBpl + kXOffset;
        std::uint8_t* dline = dst + dy * dstBpl;
        std::memcpy(dline, sline, std::size_t(dstBpl));
    }
    sample->SetActualDataLength(dstBpl * dstH);
    sample->SetSyncPoint(TRUE);
    sample->SetDiscontinuity(FALSE);
    REFERENCE_TIME tNow = 0;
    if (m_pClock) m_pClock->GetTime(&tNow);
    REFERENCE_TIME tStart = tNow - m_streamStart;
    REFERENCE_TIME tStop = tStart + m_frameInterval;
    sample->SetTime(&tStart, &tStop);
    in->Receive(sample);

    sample->Release();
    in->Release();
    alloc->Release();
}

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
    // NEVER allow unload while the process is alive. Standard DllCanUnloadNow
    // returns S_OK when serverLock + alive-object counts are zero, expecting
    // COM clients to LockServer or AddRef properly to keep the DLL pinned.
    // SurfaceMount + DirectShowLib-2005 don't: they call CoFreeUnusedLibraries
    // implicitly via Marshal.CleanupUnusedObjectsInCurrentContext after form
    // teardown, our DLL gets unmapped, and any leftover .NET RCW that still
    // holds a pointer to our (leaked) COM object then dereferences a vtable
    // that's no longer mapped -> AccessViolationException on Release().
    // Refusing to unload makes the leaked objects' vtables stay valid for
    // the rest of the process lifetime, which is what we need.
    return S_FALSE;
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
