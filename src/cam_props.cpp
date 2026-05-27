// SAA7113 hardware brightness / contrast via DirectShow IAMVideoProcAmp.
// See cam_props.h for the design rationale.
//
// Approach: enumerate VideoInputDevice monikers, find the "USB2.0 Grabber"
// (STK1150), BindToObject for an IBaseFilter, QueryInterface(IAMVideoProcAmp),
// cache it for the DLL lifetime. Set / Get call straight into the proc-amp
// interface. The interface is bound to the SAA7113 chip via the STK1150
// driver's I2C-over-USB path; values applied here persist on the SAA7113 chip
// across DLL re-loads (the chip stays powered while the USB device is attached).
//
// COM is initialised the first call on each thread that hits us; the proc-amp
// interface is process-wide. The CoInitializeEx call is idempotent + ref-
// counted by Windows so we don't unbalance any other COM users in SurfaceMount.

#include "cam_props.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <dshow.h>

#include <atomic>
#include <cstring>
#include <mutex>

namespace vis {
namespace {

std::mutex g_initMx;
bool g_inited = false;
IAMVideoProcAmp* g_proc = nullptr;  // owned; released at DLL unload (or leaked, ok)

// Find the STK1150 source filter ("USB2.0 Grabber"). Caller releases.
IBaseFilter* find_grabber_filter() {
    ICreateDevEnum* devEnum = nullptr;
    if (FAILED(CoCreateInstance(CLSID_SystemDeviceEnum, nullptr, CLSCTX_INPROC_SERVER,
                                IID_ICreateDevEnum, reinterpret_cast<void**>(&devEnum))) ||
        devEnum == nullptr) {
        return nullptr;
    }
    IEnumMoniker* en = nullptr;
    HRESULT hr = devEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &en, 0);
    devEnum->Release();
    if (hr != S_OK || en == nullptr) {
        return nullptr;
    }
    IBaseFilter* found = nullptr;
    IMoniker* mon = nullptr;
    while (en->Next(1, &mon, nullptr) == S_OK) {
        IPropertyBag* pb = nullptr;
        if (SUCCEEDED(mon->BindToStorage(0, nullptr, IID_IPropertyBag,
                                         reinterpret_cast<void**>(&pb))) &&
            pb != nullptr) {
            VARIANT var;
            VariantInit(&var);
            if (SUCCEEDED(pb->Read(L"FriendlyName", &var, nullptr)) && var.bstrVal != nullptr) {
                // Match "USB2.0 Grabber" (Syntek STK1150 friendly name) or any
                // STK11xx variant.
                if (wcsstr(var.bstrVal, L"USB2.0 Grabber") != nullptr || wcsstr(var.bstrVal, L"STK11") != nullptr) {
                    mon->BindToObject(nullptr, nullptr, IID_IBaseFilter,
                                      reinterpret_cast<void**>(&found));
                }
            }
            VariantClear(&var);
            pb->Release();
        }
        mon->Release();
        if (found != nullptr) {
            break;
        }
    }
    en->Release();
    return found;
}

void ensure_init() {
    std::scoped_lock lk{g_initMx};
    if (g_inited) {
        return;
    }
    g_inited = true;
    // COM init: idempotent + ref-counted on this thread. STA vs MTA: pick
    // multi-threaded so any subsequent caller from another thread doesn't
    // collide with our apartment. (DirectShow filter graphs typically run
    // single-threaded but the IAMVideoProcAmp interface is apartment-neutral
    // for our minimal set/get usage.)
    CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    IBaseFilter* f = find_grabber_filter();
    if (f == nullptr) {
        return;
    }
    // Holding the proc-amp ref keeps the source filter instance alive
    // implicitly (via COM aggregation). We can release the filter ref.
    f->QueryInterface(IID_IAMVideoProcAmp, reinterpret_cast<void**>(&g_proc));
    f->Release();
}

bool set_prop(long propId, int value) {
    try {
        ensure_init();
        if (g_proc == nullptr) {
            return false;
        }
        // Pin to Manual mode -- we don't want AGC stealing the value back.
        return SUCCEEDED(g_proc->Set(propId, value, VideoProcAmp_Flags_Manual));
    } catch (...) {  // NOLINT(bugprone-empty-catch) -- never throw across the C ABI
        return false;
    }
}

int get_prop(long propId, int fallback) {
    try {
        ensure_init();
        if (g_proc == nullptr) {
            return fallback;
        }
        long val = 0;
        long flags = 0;
        if (FAILED(g_proc->Get(propId, &val, &flags))) {
            return fallback;
        }
        return static_cast<int>(val);
    } catch (...) {  // NOLINT(bugprone-empty-catch) -- never throw across the C ABI
        return fallback;
    }
}

}  // namespace

bool set_cam_brightness(int v) {
    return set_prop(VideoProcAmp_Brightness, v);
}
bool set_cam_contrast(int v) {
    return set_prop(VideoProcAmp_Contrast, v);
}
int get_cam_brightness(int fallback) {
    return get_prop(VideoProcAmp_Brightness, fallback);
}
int get_cam_contrast(int fallback) {
    return get_prop(VideoProcAmp_Contrast, fallback);
}

}  // namespace vis
