// MVision.dll replacement.
//
// Most exports forward to the renamed original (mv::orig::, bound in
// dllmain.cpp). CheckMark2 (down-vision fiducial) is replaced with our OpenCV
// detector; its result is returned to the host via GetOffset/GetMin_val using
// a flag-based dispatch so all other modes keep using the original.
//
// Coordinate convention (recovered by matching the original's behavior): for a
// mark at frame px (cx,cy) in the WxH IplImage,
//   W = (cx - W/2 - markVisionOffsetX) / 1.5      -> GetOffset W field
//   H = -(cy - H/2 - markVisionOffsetY) / 1.5     -> GetOffset H field (negated)
//   X = Y = A = 0 ; min_val = match quality (0=best,1=worst)
// The shim multiplies W/H by the perspective scale to get mm.

#include "passthrough.h"
#include "originals.h"
#include "capture.h"
#include "vision.h"

#include <cstdio>
#include <chrono>
#include <thread>

// Lightweight timing to locate the frame-rate bottleneck (QueryFrame vs our
// detector vs the original CheckMark2). Logged per frame when capture is armed.
namespace {
using clk = std::chrono::high_resolution_clock;
inline double ms_since(clk::time_point t) {
    return std::chrono::duration<double, std::milli>(clk::now() - t).count();
}
double g_qfMs = 0.0;     // last QueryFrame duration
double g_detMs = 0.0;    // last our-detector duration
double g_origMs = 0.0;   // last original-CheckMark2 duration
unsigned int g_lastFrameHash = 0;  // freshness guard: hash of the last frame served
}

// ---- shared result state (matches the original's shared-globals design) -----
namespace {
constexpr double kScale = 1.5;   // original's offset divisor

double g_mvoX = 0.0, g_mvoY = 0.0;   // markVisionOffset (px), from setter
int    g_areaMin = 0, g_areaMax = 0; // SetMarkVisionAreaMinMax ("Range") — search-area radius

bool   g_ours   = false;             // is the last result ours (vs original)?
double g_ourW   = 0.0;               // GetOffset W field (offsetX)
double g_ourH   = 0.0;               // GetOffset H field (already negated)
double g_ourA   = 0.0;               // angle
double g_ourMin = 1.0;               // match quality (0=best)

// Compute our result in the native convention from a detection.
// The original mirrors the frame 180 deg (down-vision camera mirror mode) before
// cvGetRectSubPix at center (imgW/2+mvoX, imgH/2+mvoY). Our detector reads the
// UNmirrored frame, so a mark at (cx,cy) is at (imgW-cx, imgH-cy) in the mirrored
// frame the original uses. Net (verified against the original's aligned output):
//   W =  (imgW/2 - cx - mvoX) / 1.5
//   H = -(imgH/2 - cy - mvoY) / 1.5
void set_our_result(const vis::MarkResult& mr) {
    // Always own the result for CheckMark2 (we no longer call the original, so
    // the original's offset globals are stale). On detection failure, replicate
    // the original's "not found" sentinel so the host reacts exactly as before.
    if (!mr.found || !mr.headerOk) {
        g_ourW = -159.333; g_ourH = 132.667; g_ourA = 0.0; g_ourMin = 1.0;
        g_ours = true;
        return;
    }
    const double cxC = mr.imgW / 2.0;
    const double cyC = mr.imgH / 2.0;
    g_ourW =  (cxC - mr.cx - g_mvoX) / kScale;
    g_ourH = -(cyC - mr.cy - g_mvoY) / kScale;   // GetOffset returns H negated
    g_ourA = 0.0;
    g_ourMin = 0.05;                              // low = good match
    g_ours = true;
}

// Optional capture/logging (opt-in via trigger file).
void maybe_log(void* frame, const char* func, int algo, int range, const vis::MarkResult& mr) {
    if (!cap::armed()) return;
    int idx = cap::next_index();
    if (idx < 0) return;
    // PNG save is opt-in (separate 'frames' trigger): imwrite is the heavy part
    // and adds latency to the detection path, so log-only runs stay responsive.
    if (cap::frames_enabled()) {
        char png[MAX_PATH];
        std::snprintf(png, sizeof png, "%s\\frame_%04d.png", cap::dir(), idx);
        vis::save_frame(frame, png);
    }

    double ox = 0, oy = 0, ow = 0, oh = 0, oa = 0, omin = 0;
    if (mv::orig::GetOffset)  mv::orig::GetOffset(&ox, &oy, &ow, &oh, &oa);
    if (mv::orig::GetMin_val) mv::orig::GetMin_val(&omin);

    char line[760];
    std::snprintf(line, sizeof line,
        "%04d,%s,algo=%d,range=%d,mvo=%.1f/%.1f,area=%d/%d,"
        "ours,found=%d,cx=%.2f,cy=%.2f,W=%.4f,H=%.4f,min=%.3f,"
        "orig,W=%.4f,H=%.4f,min=%.4f,"
        "ms,queryFrame=%.1f,ourDetect=%.1f,origCheck=%.1f,"
        "ipl,ok=%d,w=%d,h=%d,origin=%d,fhash=%u",
        idx, func, algo, range, g_mvoX, g_mvoY, g_areaMin, g_areaMax,
        mr.found ? 1 : 0, mr.cx, mr.cy, g_ourW, g_ourH, g_ourMin,
        ow, oh, omin,
        g_qfMs, g_detMs, g_origMs,
        mr.headerOk ? 1 : 0, mr.imgW, mr.imgH, mr.imgOrigin, mr.frameHash);
    cap::log_line(line);
}
} // namespace

// Alias for the shim's typo'd P/Invoke name (SetUpisionOffsetXY, missing 'V').
#pragma comment(linker, "/EXPORT:_SetUpisionOffsetXY@16=_SetUpVisionOffsetXY@16")

extern "C" {

int   __stdcall Initialize(int w, int h)                                    { return mv::orig::Initialize(w, h); }
void  __stdcall Release(void)                                               { mv::orig::Release(); }
HANDLE __stdcall GetCameraHandle(int idx)                                   { return mv::orig::GetCameraHandle(idx); }
void* __stdcall QueryFrame(void* cam, int timeout) {
    auto t0 = clk::now();
    void* r = mv::orig::QueryFrame(cam, timeout);
    // Freshness + settle guard. Two problems:
    //  (1) The STK1150 is polled faster than it delivers (~30 fps) -> QueryFrame
    //      returns a STALE duplicate ~30% of the time, corrupting the host's
    //      closed-loop correction.
    //  (2) Right after move-done the head is still ringing down from the
    //      correction-move overshoot, so an immediate read catches an unsettled
    //      position -> the host can't reach a tight window -> times out -> places
    //      at the overshoot peak (the intermittent ~0.5 mm miss).
    // Fix: wait until we've seen (1 + kSettleFrames) DISTINCT fresh frames past
    // the one we last served. This discards stale duplicates AND gives the head
    // ~kSettleFrames camera periods (~33 ms each) to settle before the read.
    // Bounded by a timeout; returns the freshest frame seen. Never withholds a
    // detection, so it can't starve the host's loop. Invalid frames hash to 0
    // and pass through. kSettleFrames trades loop speed for settle margin.
    constexpr int kSettleFrames = 2;                 // extra camera frames (~66 ms) to ring down
    const int  kTarget  = 1 + kSettleFrames;
    const auto deadline = t0 + std::chrono::milliseconds(160);
    unsigned int h = vis::frame_hash(r);
    unsigned int cur = h;
    int newCount = (h != 0 && h != g_lastFrameHash) ? 1 : 0;
    while (h != 0 && newCount < kTarget && clk::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        void* r2 = mv::orig::QueryFrame(cam, timeout);
        unsigned int h2 = vis::frame_hash(r2);
        if (r2 && h2 != 0) { r = r2; if (h2 != cur) { ++newCount; cur = h2; } h = h2; }
    }
    if (h != 0) g_lastFrameHash = h;
    g_qfMs = ms_since(t0);
    return r;
}

// Up-vision component / nozzle / down-component: keep original; mark our result
// invalid so GetOffset passes through to the original for these modes.
int   __stdcall CheckComp(void* f, int hwnd, int w, int h)                  { g_ours = false; return mv::orig::CheckComp(f, hwnd, w, h); }
int   __stdcall CheckNozzle(void* f, int hwnd, int w, int h)                { g_ours = false; return mv::orig::CheckNozzle(f, hwnd, w, h); }
int   __stdcall DownCheckComp(void* f, int hwnd, int w, int h, int p)       { g_ours = false; return mv::orig::DownCheckComp(f, hwnd, w, h, p); }
void* __stdcall DownShow(void* f)                                           { g_ours = false; return mv::orig::DownShow(f); }

// CheckMark: algo == -1 is preview-only in the original (no detection), so just
// pass through. (Algorithmic mark modes could be handled later.)
int   __stdcall CheckMark(void* f, int hwnd, int w, int h, int algo, int r) {
    g_ours = false;
    return mv::orig::CheckMark(f, hwnd, w, h, algo, r);
}

// CheckMark2: the down-vision fiducial mode (what the machine uses). Detect with
// our OpenCV pipeline; the original still runs to render the live preview, but
// GetOffset/GetMin_val return OUR result.
int   __stdcall CheckMark2(void* f, int hwnd, int w, int h, int algo, int r) {
    auto t0 = clk::now();
    // Reference point (where the mark sits when aligned) in unflipped frame
    // coords, and the search radius from the "Range" setting (areaMin). Reject
    // detections outside it to suppress stray circles.
    const double refX = 320.0 - g_mvoX;   // imgW/2 - mvoX  (640-wide frame)
    const double refY = 240.0 - g_mvoY;   // imgH/2 - mvoY
    const int searchR = (g_areaMin > 8 && g_areaMin < 400) ? g_areaMin : 150;
    // CheckMark2 args: algo = template size px (from the 1.2mm setting), r = strength.
    vis::MarkResult mr = vis::detect_circle_mark(f, algo, refX, refY, searchR);  // read-only
    g_detMs = ms_since(t0);

    // No temporal smoothing: report the detected center as-is. An EMA smoother
    // here lagged the reported position by a few frames, so the host would settle
    // on the lagged value (green appearing on the red target), then the smoother
    // caught up and the green cross drifted to its true position OFF the red
    // target ("settles too quickly"). With the tight radius bracket + both-field
    // detection the raw center is already sub-pixel-stable, so smoothing is
    // unneeded and only hurt alignment accuracy.
    set_our_result(mr);                                  // our offset drives placement
    auto t1 = clk::now();
    // Render the preview ourselves (skips the original's redundant template
    // match). Fall back to the original only if our render fails.
    if (!vis::render_preview(f, reinterpret_cast<void*>(static_cast<intptr_t>(hwnd)), g_mvoX, g_mvoY, searchR, mr))
        mv::orig::CheckMark2(f, hwnd, w, h, algo, r);
    g_origMs = ms_since(t1);
    maybe_log(f, "CheckMark2", algo, r, mr);
    return 0;
}

int   __stdcall GetTemplate(void* f, unsigned char* out, int sz, double p)  { return mv::orig::GetTemplate(f, out, sz, p); }
int   __stdcall CheckTemplate(void* f, int hwnd, int w, int h, unsigned char* t, int sz, double th, int m) {
    g_ours = false;
    return mv::orig::CheckTemplate(f, hwnd, w, h, t, sz, th, m);
}
int   __stdcall TemplateVision(void* f, int hwnd, int w, int h, unsigned char* t, int sz, double th, int m) {
    return mv::orig::TemplateVision(f, hwnd, w, h, t, sz, th, m);
}

int   __stdcall OpenPerspectiveTransform(void)                              { return mv::orig::OpenPerspectiveTransform(); }
int   __stdcall ClosePerspectiveTransform(void)                            { return mv::orig::ClosePerspectiveTransform(); }
int   __stdcall SetPerspectiveMatrix(double* m9)                            { return mv::orig::SetPerspectiveMatrix(m9); }
void* __stdcall GetLowResTransformParam(void* f, int hwnd, int w, int h, int a, int b, int* oa, double* ob, double* om) {
    return mv::orig::GetLowResTransformParam(f, hwnd, w, h, a, b, oa, ob, om);
}

int   __stdcall OpenPerspectiveTransform7(void)                             { return mv::orig::OpenPerspectiveTransform7(); }
int   __stdcall ClosePerspectiveTransform7(void)                           { return mv::orig::ClosePerspectiveTransform7(); }
int   __stdcall SetPerspectiveMatrix7(double* m9)                          { return mv::orig::SetPerspectiveMatrix7(m9); }
void* __stdcall GetLowResTransformParam7(void* f, int w, int h, int* oa, double* ob, double* om) {
    return mv::orig::GetLowResTransformParam7(f, w, h, oa, ob, om);
}

void  __stdcall Draw(void* f, int hwnd, int w, int h)                       { mv::orig::Draw(f, hwnd, w, h); }
void  __stdcall Draw2(unsigned char* rgb, int sz, void* f, int w, int h)    { mv::orig::Draw2(rgb, sz, f, w, h); }
int   __stdcall myLoadImage(const char* p)                                  { return mv::orig::myLoadImage(p); }
int   __stdcall mySaveImage(void* f, const char* p)                         { return mv::orig::mySaveImage(f, p); }

// Result accessors: return OUR result when the last check was ours, else the
// original's (so up-vision / component modes are unaffected).
void  __stdcall GetOffset(double* x, double* y, double* w, double* h, double* a) {
    if (g_ours) { *x = 0.0; *y = 0.0; *w = g_ourW; *h = g_ourH; *a = g_ourA; }
    else        { mv::orig::GetOffset(x, y, w, h, a); }
}
void  __stdcall GetMin_val(double* v) {
    if (g_ours) { *v = g_ourMin; }
    else        { mv::orig::GetMin_val(v); }
}

void  __stdcall SetThreshold(int v)                                         { mv::orig::SetThreshold(v); }
void  __stdcall SetCompAngle(double v)                                      { mv::orig::SetCompAngle(v); }
void  __stdcall SetCompSizeWHA(double w, double h, double a)                { mv::orig::SetCompSizeWHA(w, h, a); }
void  __stdcall SetMarkVisionOffsetXY(double x, double y)                   { g_mvoX = x; g_mvoY = y; mv::orig::SetMarkVisionOffsetXY(x, y); }
void  __stdcall SetUpVisionOffsetXY(double x, double y)                     { mv::orig::SetUpVisionOffsetXY(x, y); }
void  __stdcall SetMarkVisionAreaMinMax(int mn, int mx)                     { g_areaMin = mn; g_areaMax = mx; mv::orig::SetMarkVisionAreaMinMax(mn, mx); }
void  __stdcall SetIsCheckTemplate(int b)                                   { mv::orig::SetIsCheckTemplate(b); }
void  __stdcall SetUpVisionCameraMirrorMode(int m)                          { mv::orig::SetUpVisionCameraMirrorMode(m); }
void  __stdcall SetDownVisionCameraMirrorMode(int m)                        { mv::orig::SetDownVisionCameraMirrorMode(m); }

} // extern "C"
