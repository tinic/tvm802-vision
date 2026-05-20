#pragma once
// Function-pointer types for each of MVision-orig.dll's exports, plus the
// globals that hold the bound pointers. dllmain.cpp owns binding; the
// dispatcher functions in passthrough.cpp read them.

#include <windows.h>

extern "C" {
    typedef int   (__stdcall *fn_Initialize)(int, int);
    typedef void  (__stdcall *fn_Release)(void);
    typedef HANDLE(__stdcall *fn_GetCameraHandle)(int);
    typedef void* (__stdcall *fn_QueryFrame)(void*, int);

    typedef int   (__stdcall *fn_CheckComp)(void*, int, int, int);
    typedef int   (__stdcall *fn_CheckNozzle)(void*, int, int, int);
    typedef int   (__stdcall *fn_CheckMark)(void*, int, int, int, int, int);
    typedef int   (__stdcall *fn_CheckMark2)(void*, int, int, int, int, int);
    typedef int   (__stdcall *fn_DownCheckComp)(void*, int, int, int, int);
    typedef void* (__stdcall *fn_DownShow)(void*);

    typedef int   (__stdcall *fn_GetTemplate)(void*, unsigned char*, int, double);
    typedef int   (__stdcall *fn_CheckTemplate)(void*, int, int, int, unsigned char*, int, double, int);
    typedef int   (__stdcall *fn_TemplateVision)(void*, int, int, int, unsigned char*, int, double, int);

    typedef int   (__stdcall *fn_OpenPerspectiveTransform)(void);
    typedef int   (__stdcall *fn_ClosePerspectiveTransform)(void);
    typedef int   (__stdcall *fn_SetPerspectiveMatrix)(double*);
    typedef void* (__stdcall *fn_GetLowResTransformParam)(void*, int, int, int, int, int, int*, double*, double*);

    typedef int   (__stdcall *fn_OpenPerspectiveTransform7)(void);
    typedef int   (__stdcall *fn_ClosePerspectiveTransform7)(void);
    typedef int   (__stdcall *fn_SetPerspectiveMatrix7)(double*);
    typedef void* (__stdcall *fn_GetLowResTransformParam7)(void*, int, int, int*, double*, double*);

    typedef void  (__stdcall *fn_Draw)(void*, int, int, int);
    typedef void  (__stdcall *fn_Draw2)(unsigned char*, int, void*, int, int);
    typedef int   (__stdcall *fn_myLoadImage)(const char*);
    typedef int   (__stdcall *fn_mySaveImage)(void*, const char*);

    typedef void  (__stdcall *fn_GetOffset)(double*, double*, double*, double*, double*);
    typedef void  (__stdcall *fn_GetMin_val)(double*);

    typedef void  (__stdcall *fn_SetThreshold)(int);
    typedef void  (__stdcall *fn_SetCompAngle)(double);
    typedef void  (__stdcall *fn_SetCompSizeWHA)(double, double, double);
    typedef void  (__stdcall *fn_SetMarkVisionOffsetXY)(double, double);
    typedef void  (__stdcall *fn_SetUpVisionOffsetXY)(double, double);
    typedef void  (__stdcall *fn_SetMarkVisionAreaMinMax)(int, int);
    typedef void  (__stdcall *fn_SetIsCheckTemplate)(int);
    typedef void  (__stdcall *fn_SetUpVisionCameraMirrorMode)(int);
    typedef void  (__stdcall *fn_SetDownVisionCameraMirrorMode)(int);
}

// We use an X-macro elsewhere (commented out in dllmain.cpp because @N
// suffixes can't be synthesized) — keeping this here in case a future
// codegen pass wants it.
#define MVISION_FUNCTIONS(X)                                                    \
    X(Initialize,                    fn_Initialize)                             \
    X(Release,                       fn_Release)                                \
    X(GetCameraHandle,               fn_GetCameraHandle)                        \
    X(QueryFrame,                    fn_QueryFrame)                             \
    X(CheckComp,                     fn_CheckComp)                              \
    X(CheckNozzle,                   fn_CheckNozzle)                            \
    X(CheckMark,                     fn_CheckMark)                              \
    X(CheckMark2,                    fn_CheckMark2)                             \
    X(DownCheckComp,                 fn_DownCheckComp)                          \
    X(DownShow,                      fn_DownShow)                               \
    X(GetTemplate,                   fn_GetTemplate)                            \
    X(CheckTemplate,                 fn_CheckTemplate)                          \
    X(TemplateVision,                fn_TemplateVision)                         \
    X(OpenPerspectiveTransform,      fn_OpenPerspectiveTransform)               \
    X(ClosePerspectiveTransform,     fn_ClosePerspectiveTransform)              \
    X(SetPerspectiveMatrix,          fn_SetPerspectiveMatrix)                   \
    X(GetLowResTransformParam,       fn_GetLowResTransformParam)                \
    X(OpenPerspectiveTransform7,     fn_OpenPerspectiveTransform7)              \
    X(ClosePerspectiveTransform7,    fn_ClosePerspectiveTransform7)             \
    X(SetPerspectiveMatrix7,         fn_SetPerspectiveMatrix7)                  \
    X(GetLowResTransformParam7,      fn_GetLowResTransformParam7)               \
    X(Draw,                          fn_Draw)                                   \
    X(Draw2,                         fn_Draw2)                                  \
    X(myLoadImage,                   fn_myLoadImage)                            \
    X(mySaveImage,                   fn_mySaveImage)                            \
    X(GetOffset,                     fn_GetOffset)                              \
    X(GetMin_val,                    fn_GetMin_val)                             \
    X(SetThreshold,                  fn_SetThreshold)                           \
    X(SetCompAngle,                  fn_SetCompAngle)                           \
    X(SetCompSizeWHA,                fn_SetCompSizeWHA)                         \
    X(SetMarkVisionOffsetXY,         fn_SetMarkVisionOffsetXY)                  \
    X(SetUpVisionOffsetXY,           fn_SetUpVisionOffsetXY)                    \
    X(SetMarkVisionAreaMinMax,       fn_SetMarkVisionAreaMinMax)                \
    X(SetIsCheckTemplate,            fn_SetIsCheckTemplate)                     \
    X(SetUpVisionCameraMirrorMode,   fn_SetUpVisionCameraMirrorMode)            \
    X(SetDownVisionCameraMirrorMode, fn_SetDownVisionCameraMirrorMode)

namespace mv::orig {
    extern HMODULE module;
#define X(name, sig) extern sig name;
    MVISION_FUNCTIONS(X)
#undef X
}
