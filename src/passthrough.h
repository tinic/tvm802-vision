#pragma once
// All native exports we need to provide, with the exact stdcall signatures
// that MVisionControl.dll's P/Invokes expect. Argument sizes were verified
// against (a) the @N suffix on the original DLL's decorated exports and
// (b) the P/Invoke declarations in MVisionControl.dll.
//
// On i386 Windows, __stdcall callee cleans the stack and the linker prepends
// `_` and appends `@<argbytes>` to the export name.

#include <windows.h>

extern "C" {

// ---- camera / capture --------------------------------------------------
__declspec(dllexport) int   __stdcall Initialize(int width, int height);
__declspec(dllexport) void  __stdcall Release(void);
__declspec(dllexport) HANDLE __stdcall GetCameraHandle(int deviceIndex);
__declspec(dllexport) void* __stdcall QueryFrame(void* cameraHandle, int timeout);

// ---- vision modes ------------------------------------------------------
__declspec(dllexport) int   __stdcall CheckComp(void* frame, int hwnd, int width, int height);
__declspec(dllexport) int   __stdcall CheckNozzle(void* frame, int hwnd, int width, int height);
__declspec(dllexport) int   __stdcall CheckMark(void* frame, int hwnd, int width, int height, int algo, int range);
__declspec(dllexport) int   __stdcall CheckMark2(void* frame, int hwnd, int width, int height, int algo, int range);
__declspec(dllexport) int   __stdcall DownCheckComp(void* frame, int hwnd, int width, int height, int param);
__declspec(dllexport) void* __stdcall DownShow(void* frame);

// ---- template matching -------------------------------------------------
__declspec(dllexport) int   __stdcall GetTemplate(void* frame, unsigned char* outRgb, int size, double param);
__declspec(dllexport) int   __stdcall CheckTemplate(void* frame, int hwnd, int width, int height,
                                                    unsigned char* templateRgb, int size, double threshold, int mode);
__declspec(dllexport) int   __stdcall TemplateVision(void* frame, int hwnd, int width, int height,
                                                     unsigned char* templateRgb, int size, double threshold, int mode);

// ---- perspective transforms -------------------------------------------
__declspec(dllexport) int   __stdcall OpenPerspectiveTransform(void);
__declspec(dllexport) int   __stdcall ClosePerspectiveTransform(void);
__declspec(dllexport) int   __stdcall SetPerspectiveMatrix(double* matrix9);
__declspec(dllexport) void* __stdcall GetLowResTransformParam(void* frame, int hwnd, int width, int height,
                                                              int a, int b, int* outA, double* outB, double* outMatrix9);

__declspec(dllexport) int   __stdcall OpenPerspectiveTransform7(void);
__declspec(dllexport) int   __stdcall ClosePerspectiveTransform7(void);
__declspec(dllexport) int   __stdcall SetPerspectiveMatrix7(double* matrix9);
__declspec(dllexport) void* __stdcall GetLowResTransformParam7(void* frame, int width, int height,
                                                               int* outA, double* outB, double* outMatrix9);

// ---- drawing / I/O -----------------------------------------------------
__declspec(dllexport) void  __stdcall Draw(void* frame, int hwnd, int width, int height);
__declspec(dllexport) void  __stdcall Draw2(unsigned char* rgb, int size, void* frame, int width, int height);
// Path args are ANSI char* — the shim P/Invokes these with char[] under the
// default CharSet.Ansi, so the runtime marshals to a byte string.
__declspec(dllexport) int   __stdcall myLoadImage(const char* path);
__declspec(dllexport) int   __stdcall mySaveImage(void* frame, const char* path);

// ---- result accessors --------------------------------------------------
__declspec(dllexport) void  __stdcall GetOffset(double* outX, double* outY, double* outW, double* outH, double* outAngle);
__declspec(dllexport) void  __stdcall GetMin_val(double* outVal);

// ---- tuning setters ----------------------------------------------------
__declspec(dllexport) void  __stdcall SetThreshold(int v);
__declspec(dllexport) void  __stdcall SetCompAngle(double v);
__declspec(dllexport) void  __stdcall SetCompSizeWHA(double w, double h, double angle);
__declspec(dllexport) void  __stdcall SetMarkVisionOffsetXY(double x, double y);
__declspec(dllexport) void  __stdcall SetUpVisionOffsetXY(double x, double y);
__declspec(dllexport) void  __stdcall SetMarkVisionAreaMinMax(int minA, int maxA);
__declspec(dllexport) void  __stdcall SetIsCheckTemplate(int b);            // BOOL -> int in stdcall
__declspec(dllexport) void  __stdcall SetUpVisionCameraMirrorMode(int m);
__declspec(dllexport) void  __stdcall SetDownVisionCameraMirrorMode(int m);

// Note: SetUpisionOffsetXY (typo) is provided as a .def alias of
// SetUpVisionOffsetXY — no separate symbol needed in code.

} // extern "C"
