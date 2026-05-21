#pragma once
// Function-pointer types for each of MVision-orig.dll's exports, plus the
// globals that hold the bound pointers. dllmain.cpp owns binding; the
// dispatcher functions in passthrough.cpp read them.

#include <windows.h>

extern "C" {
typedef int(__stdcall* fn_Initialize)(int, int);
typedef void(__stdcall* fn_Release)(void);
typedef HANDLE(__stdcall* fn_GetCameraHandle)(int);
typedef void*(__stdcall* fn_QueryFrame)(void*, int);

typedef int(__stdcall* fn_CheckComp)(void*, int, int, int);
typedef int(__stdcall* fn_CheckNozzle)(void*, int, int, int);
typedef int(__stdcall* fn_CheckMark)(void*, int, int, int, int, int);
typedef int(__stdcall* fn_CheckMark2)(void*, int, int, int, int, int);
typedef int(__stdcall* fn_DownCheckComp)(void*, int, int, int, int);
typedef void*(__stdcall* fn_DownShow)(void*);

typedef int(__stdcall* fn_GetTemplate)(void*, unsigned char*, int, double);
typedef int(__stdcall* fn_CheckTemplate)(void*, int, int, int, unsigned char*, int, double, int);
typedef int(__stdcall* fn_TemplateVision)(void*, int, int, int, unsigned char*, int, double, int);

typedef int(__stdcall* fn_OpenPerspectiveTransform)(void);
typedef int(__stdcall* fn_ClosePerspectiveTransform)(void);
typedef int(__stdcall* fn_SetPerspectiveMatrix)(double*);
typedef void*(__stdcall* fn_GetLowResTransformParam)(void*, int, int, int, int, int, int*, double*, double*);

typedef int(__stdcall* fn_OpenPerspectiveTransform7)(void);
typedef int(__stdcall* fn_ClosePerspectiveTransform7)(void);
typedef int(__stdcall* fn_SetPerspectiveMatrix7)(double*);
typedef void*(__stdcall* fn_GetLowResTransformParam7)(void*, int, int, int*, double*, double*);

typedef void(__stdcall* fn_Draw)(void*, int, int, int);
typedef void(__stdcall* fn_Draw2)(unsigned char*, int, void*, int, int);
typedef int(__stdcall* fn_myLoadImage)(const char*);
typedef int(__stdcall* fn_mySaveImage)(void*, const char*);

typedef void(__stdcall* fn_GetOffset)(double*, double*, double*, double*, double*);
typedef void(__stdcall* fn_GetMin_val)(double*);

typedef void(__stdcall* fn_SetThreshold)(int);
typedef void(__stdcall* fn_SetCompAngle)(double);
typedef void(__stdcall* fn_SetCompSizeWHA)(double, double, double);
typedef void(__stdcall* fn_SetMarkVisionOffsetXY)(double, double);
typedef void(__stdcall* fn_SetUpVisionOffsetXY)(double, double);
typedef void(__stdcall* fn_SetMarkVisionAreaMinMax)(int, int);
typedef void(__stdcall* fn_SetIsCheckTemplate)(int);
typedef void(__stdcall* fn_SetUpVisionCameraMirrorMode)(int);
typedef void(__stdcall* fn_SetDownVisionCameraMirrorMode)(int);
}

// X-macro table of every original export. The third column is the DECORATED
// stdcall export name (leading underscore + @N, where N is the total bytes of
// args) exactly as `dumpbin /exports MVision-orig.dll` lists it. dllmain.cpp
// expands this once to declare the pointers and once to GetProcAddress-bind them,
// so the name list and the @N suffixes are maintained in exactly one place.
#define MVISION_FUNCTIONS(X)                                                                         \
    X(Initialize, fn_Initialize, "_Initialize@8")                                                    \
    X(Release, fn_Release, "_Release@0")                                                             \
    X(GetCameraHandle, fn_GetCameraHandle, "_GetCameraHandle@4")                                     \
    X(QueryFrame, fn_QueryFrame, "_QueryFrame@8")                                                    \
    X(CheckComp, fn_CheckComp, "_CheckComp@16")                                                      \
    X(CheckNozzle, fn_CheckNozzle, "_CheckNozzle@16")                                                \
    X(CheckMark, fn_CheckMark, "_CheckMark@24")                                                      \
    X(CheckMark2, fn_CheckMark2, "_CheckMark2@24")                                                   \
    X(DownCheckComp, fn_DownCheckComp, "_DownCheckComp@20")                                          \
    X(DownShow, fn_DownShow, "_DownShow@4")                                                          \
    X(GetTemplate, fn_GetTemplate, "_GetTemplate@20")                                                \
    X(CheckTemplate, fn_CheckTemplate, "_CheckTemplate@36")                                          \
    X(TemplateVision, fn_TemplateVision, "_TemplateVision@36")                                       \
    X(OpenPerspectiveTransform, fn_OpenPerspectiveTransform, "_OpenPerspectiveTransform@0")          \
    X(ClosePerspectiveTransform, fn_ClosePerspectiveTransform, "_ClosePerspectiveTransform@0")       \
    X(SetPerspectiveMatrix, fn_SetPerspectiveMatrix, "_SetPerspectiveMatrix@4")                      \
    X(GetLowResTransformParam, fn_GetLowResTransformParam, "_GetLowResTransformParam@36")            \
    X(OpenPerspectiveTransform7, fn_OpenPerspectiveTransform7, "_OpenPerspectiveTransform7@0")       \
    X(ClosePerspectiveTransform7, fn_ClosePerspectiveTransform7, "_ClosePerspectiveTransform7@0")    \
    X(SetPerspectiveMatrix7, fn_SetPerspectiveMatrix7, "_SetPerspectiveMatrix7@4")                   \
    X(GetLowResTransformParam7, fn_GetLowResTransformParam7, "_GetLowResTransformParam7@24")         \
    X(Draw, fn_Draw, "_Draw@16")                                                                     \
    X(Draw2, fn_Draw2, "_Draw2@20")                                                                  \
    X(myLoadImage, fn_myLoadImage, "_myLoadImage@4")                                                 \
    X(mySaveImage, fn_mySaveImage, "_mySaveImage@8")                                                 \
    X(GetOffset, fn_GetOffset, "_GetOffset@20")                                                      \
    X(GetMin_val, fn_GetMin_val, "_GetMin_val@4")                                                    \
    X(SetThreshold, fn_SetThreshold, "_SetThreshold@4")                                              \
    X(SetCompAngle, fn_SetCompAngle, "_SetCompAngle@8")                                              \
    X(SetCompSizeWHA, fn_SetCompSizeWHA, "_SetCompSizeWHA@24")                                       \
    X(SetMarkVisionOffsetXY, fn_SetMarkVisionOffsetXY, "_SetMarkVisionOffsetXY@16")                  \
    X(SetUpVisionOffsetXY, fn_SetUpVisionOffsetXY, "_SetUpVisionOffsetXY@16")                        \
    X(SetMarkVisionAreaMinMax, fn_SetMarkVisionAreaMinMax, "_SetMarkVisionAreaMinMax@8")             \
    X(SetIsCheckTemplate, fn_SetIsCheckTemplate, "_SetIsCheckTemplate@4")                            \
    X(SetUpVisionCameraMirrorMode, fn_SetUpVisionCameraMirrorMode, "_SetUpVisionCameraMirrorMode@4") \
    X(SetDownVisionCameraMirrorMode, fn_SetDownVisionCameraMirrorMode, "_SetDownVisionCameraMirrorMode@4")

namespace mv::orig {
extern HMODULE module;
#define X(name, sig, sym) extern sig name;
MVISION_FUNCTIONS(X)
#undef X
}  // namespace mv::orig
