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

#define X(name, sig) sig name = nullptr;
    MVISION_FUNCTIONS(X)
#undef X
}

static bool bind_originals(HMODULE m) {
    using namespace mv::orig;
#define X(name, sig)                                                            \
    name = reinterpret_cast<sig>(GetProcAddress(m, "_" #name "@" /* concat */));
    // The macro above is intentionally broken — we need the per-function
    // @N suffix, which the macro can't synthesize. We unroll explicitly
    // below instead.
#undef X

    // Decorated stdcall export names — must match dumpbin /exports output
    // of the original MVision.dll exactly. @N is the total bytes of args.
    Initialize                    = reinterpret_cast<fn_Initialize>                  (GetProcAddress(m, "_Initialize@8"));
    Release                       = reinterpret_cast<fn_Release>                     (GetProcAddress(m, "_Release@0"));
    GetCameraHandle               = reinterpret_cast<fn_GetCameraHandle>             (GetProcAddress(m, "_GetCameraHandle@4"));
    QueryFrame                    = reinterpret_cast<fn_QueryFrame>                  (GetProcAddress(m, "_QueryFrame@8"));

    CheckComp                     = reinterpret_cast<fn_CheckComp>                   (GetProcAddress(m, "_CheckComp@16"));
    CheckNozzle                   = reinterpret_cast<fn_CheckNozzle>                 (GetProcAddress(m, "_CheckNozzle@16"));
    CheckMark                     = reinterpret_cast<fn_CheckMark>                   (GetProcAddress(m, "_CheckMark@24"));
    CheckMark2                    = reinterpret_cast<fn_CheckMark2>                  (GetProcAddress(m, "_CheckMark2@24"));
    DownCheckComp                 = reinterpret_cast<fn_DownCheckComp>               (GetProcAddress(m, "_DownCheckComp@20"));
    DownShow                      = reinterpret_cast<fn_DownShow>                    (GetProcAddress(m, "_DownShow@4"));

    GetTemplate                   = reinterpret_cast<fn_GetTemplate>                 (GetProcAddress(m, "_GetTemplate@20"));
    CheckTemplate                 = reinterpret_cast<fn_CheckTemplate>               (GetProcAddress(m, "_CheckTemplate@36"));
    TemplateVision                = reinterpret_cast<fn_TemplateVision>              (GetProcAddress(m, "_TemplateVision@36"));

    OpenPerspectiveTransform      = reinterpret_cast<fn_OpenPerspectiveTransform>    (GetProcAddress(m, "_OpenPerspectiveTransform@0"));
    ClosePerspectiveTransform     = reinterpret_cast<fn_ClosePerspectiveTransform>   (GetProcAddress(m, "_ClosePerspectiveTransform@0"));
    SetPerspectiveMatrix          = reinterpret_cast<fn_SetPerspectiveMatrix>        (GetProcAddress(m, "_SetPerspectiveMatrix@4"));
    GetLowResTransformParam       = reinterpret_cast<fn_GetLowResTransformParam>     (GetProcAddress(m, "_GetLowResTransformParam@36"));

    OpenPerspectiveTransform7     = reinterpret_cast<fn_OpenPerspectiveTransform7>   (GetProcAddress(m, "_OpenPerspectiveTransform7@0"));
    ClosePerspectiveTransform7    = reinterpret_cast<fn_ClosePerspectiveTransform7>  (GetProcAddress(m, "_ClosePerspectiveTransform7@0"));
    SetPerspectiveMatrix7         = reinterpret_cast<fn_SetPerspectiveMatrix7>       (GetProcAddress(m, "_SetPerspectiveMatrix7@4"));
    GetLowResTransformParam7      = reinterpret_cast<fn_GetLowResTransformParam7>    (GetProcAddress(m, "_GetLowResTransformParam7@24"));

    Draw                          = reinterpret_cast<fn_Draw>                        (GetProcAddress(m, "_Draw@16"));
    Draw2                         = reinterpret_cast<fn_Draw2>                       (GetProcAddress(m, "_Draw2@20"));
    myLoadImage                   = reinterpret_cast<fn_myLoadImage>                 (GetProcAddress(m, "_myLoadImage@4"));
    mySaveImage                   = reinterpret_cast<fn_mySaveImage>                 (GetProcAddress(m, "_mySaveImage@8"));

    GetOffset                     = reinterpret_cast<fn_GetOffset>                   (GetProcAddress(m, "_GetOffset@20"));
    GetMin_val                    = reinterpret_cast<fn_GetMin_val>                  (GetProcAddress(m, "_GetMin_val@4"));

    SetThreshold                  = reinterpret_cast<fn_SetThreshold>                (GetProcAddress(m, "_SetThreshold@4"));
    SetCompAngle                  = reinterpret_cast<fn_SetCompAngle>                (GetProcAddress(m, "_SetCompAngle@8"));
    SetCompSizeWHA                = reinterpret_cast<fn_SetCompSizeWHA>              (GetProcAddress(m, "_SetCompSizeWHA@24"));
    SetMarkVisionOffsetXY         = reinterpret_cast<fn_SetMarkVisionOffsetXY>       (GetProcAddress(m, "_SetMarkVisionOffsetXY@16"));
    SetUpVisionOffsetXY           = reinterpret_cast<fn_SetUpVisionOffsetXY>         (GetProcAddress(m, "_SetUpVisionOffsetXY@16"));
    SetMarkVisionAreaMinMax       = reinterpret_cast<fn_SetMarkVisionAreaMinMax>     (GetProcAddress(m, "_SetMarkVisionAreaMinMax@8"));
    SetIsCheckTemplate            = reinterpret_cast<fn_SetIsCheckTemplate>          (GetProcAddress(m, "_SetIsCheckTemplate@4"));
    SetUpVisionCameraMirrorMode   = reinterpret_cast<fn_SetUpVisionCameraMirrorMode> (GetProcAddress(m, "_SetUpVisionCameraMirrorMode@4"));
    SetDownVisionCameraMirrorMode = reinterpret_cast<fn_SetDownVisionCameraMirrorMode>(GetProcAddress(m, "_SetDownVisionCameraMirrorMode@4"));

    // If any required binding failed, the original was incomplete or wrong
    // version. Fail the DLL load.
    return Initialize && Release && GetCameraHandle && QueryFrame &&
           CheckComp && CheckMark && CheckMark2 && CheckNozzle &&
           DownCheckComp && DownShow &&
           GetTemplate && CheckTemplate && TemplateVision &&
           OpenPerspectiveTransform  && ClosePerspectiveTransform  && SetPerspectiveMatrix  && GetLowResTransformParam &&
           OpenPerspectiveTransform7 && ClosePerspectiveTransform7 && SetPerspectiveMatrix7 && GetLowResTransformParam7 &&
           Draw && Draw2 && myLoadImage && mySaveImage &&
           GetOffset && GetMin_val &&
           SetThreshold && SetCompAngle && SetCompSizeWHA &&
           SetMarkVisionOffsetXY && SetUpVisionOffsetXY && SetMarkVisionAreaMinMax &&
           SetIsCheckTemplate && SetUpVisionCameraMirrorMode && SetDownVisionCameraMirrorMode;
}

BOOL APIENTRY DllMain(HMODULE /*self*/, DWORD reason, LPVOID /*reserved*/) {
    using namespace mv::orig;
    switch (reason) {
    case DLL_PROCESS_ATTACH: {
        module = LoadLibraryW(L"MVision-orig.dll");
        if (!module) return FALSE;
        if (!bind_originals(module)) {
            FreeLibrary(module);
            module = nullptr;
            return FALSE;
        }
        break;
    }
    case DLL_PROCESS_DETACH:
        if (module) { FreeLibrary(module); module = nullptr; }
        break;
    }
    return TRUE;
}
