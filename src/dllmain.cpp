// dllmain.cpp — load the renamed original DLL once at process attach, expose
// the function pointers our passthrough.cpp uses.
//
// Deployment expects the original native binary to be renamed alongside us:
//   MVision.dll      <-- our replacement (this one)
//   MVision-orig.dll <-- the original native binary, renamed
//
// On DLL_PROCESS_ATTACH we LoadLibrary("MVision-orig.dll") and bind every
// function pointer. If the original isn't found we deliberately fail to load
// rather than silently degrade, so a misdeployed install is loud.

#include <windows.h>
#include "originals.h"

namespace mv::orig {
HMODULE module = nullptr;

#define X(name, sig, sym) sig name = nullptr;
MVISION_FUNCTIONS(X)
#undef X
}  // namespace mv::orig

// Bind every original export by its decorated stdcall name (the third column of
// MVISION_FUNCTIONS, e.g. "_CheckMark2@24"). Returns false if ANY lookup failed,
// which means the original is incomplete or the wrong version — the caller fails
// the DLL load rather than silently degrade.
static bool bind_originals(HMODULE m) {
    using namespace mv::orig;
    bool ok = true;
#define X(name, sig, sym)                                 \
    name = reinterpret_cast<sig>(GetProcAddress(m, sym)); \
    ok = ok && (name != nullptr);
    MVISION_FUNCTIONS(X)
#undef X
    return ok;
}

// cppcheck-suppress unusedFunction  // DLL entry point, called by the OS loader.
BOOL APIENTRY DllMain(HMODULE /*self*/, DWORD reason, LPVOID /*reserved*/) {
    using namespace mv::orig;
    switch (reason) {
        case DLL_PROCESS_ATTACH: {
            module = LoadLibraryW(L"MVision-orig.dll");
            if (module == nullptr) {
                return FALSE;
            }
            if (!bind_originals(module)) {
                FreeLibrary(module);
                module = nullptr;
                return FALSE;
            }
            break;
        }
        case DLL_PROCESS_DETACH:
            if (module != nullptr) {
                FreeLibrary(module);
                module = nullptr;
            }
            break;
    }
    return TRUE;
}
