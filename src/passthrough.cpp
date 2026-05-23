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
#include <cstring>
#include <chrono>
#include <thread>

// Lightweight timing to locate the frame-rate bottleneck (QueryFrame vs our
// detector vs our preview render). Logged per frame when capture is armed.
namespace {
using clk = std::chrono::high_resolution_clock;
inline double ms_since(clk::time_point t) {
    return std::chrono::duration<double, std::milli>(clk::now() - t).count();
}
double g_qfMs = 0.0;               // last QueryFrame duration
double g_detMs = 0.0;              // last our-detector duration
double g_renderMs = 0.0;           // last preview-render duration
unsigned int g_lastFrameHash = 0;  // freshness guard: hash of the last frame served

// ImageTemplate cache. The host pushes its single mark template to CheckTemplate
// occasionally and passes null on most frames, so we hold the latest one and
// match it across the null frames. `scale` is the locked match scale; it is reset
// only when the template actually changes (hash compare), so the lock survives the
// host re-pushing the same template. The plane is a size x size 8-bit gray image,
// widthStep aligned to 4.
struct Template {
    unsigned char buf[16384];
    int sz = 0;  // 0 = nothing cached yet
    unsigned int hash = 0;
    double scale = 0.0;  // locked match scale (0 = re-sweep next detect)
};
Template g_tmpl;
unsigned int tmpl_hash(const unsigned char* p, size_t n) {
    unsigned int h = 2166136261u;
    for (size_t i = 0; i < n; i += 7) h = (h ^ p[i]) * 16777619u;
    return h ? h : 1u;
}
// Cache the host's template. No-op on a null / too-small / oversized buffer, or
// when the same template is pushed again (which preserves the locked scale).
void cache_template(const unsigned char* t, int sz) {
    const int step = (sz + 3) & ~3;
    const size_t n = static_cast<size_t>(step) * sz;
    if (!t || sz < 4 || n > sizeof g_tmpl.buf) return;
    const unsigned int h = tmpl_hash(t, n);
    if (h == g_tmpl.hash && sz == g_tmpl.sz) return;  // unchanged -> keep scale lock
    std::memcpy(g_tmpl.buf, t, n);
    g_tmpl.sz = sz;
    g_tmpl.hash = h;
    g_tmpl.scale = 0.0;  // new template -> re-sweep scale next detect
}
}  // namespace

// ---- shared result state (matches the original's shared-globals design) -----
namespace {
constexpr double kScale = 1.5;  // original's offset divisor

// Default frame geometry (Gen-2 802B: 640x480 grayscale). The live reference is
// derived from each frame's actual dimensions; these are the fallback used when
// a frame header can't be read.
constexpr int kDefaultFrameW = 640, kDefaultFrameH = 480;

// "Range" (search-area radius): honor the host's value within these bounds, else
// fall back to a sane default.
constexpr int kAreaMinPx = 8, kAreaMaxPx = 400, kDefaultSearchPx = 150;

double g_mvoX = 0.0, g_mvoY = 0.0;  // markVisionOffset (px), from setter
int g_areaMin = 0, g_areaMax = 0;   // SetMarkVisionAreaMinMax ("Range") — search-area radius

bool g_ours = false;    // is the last result ours (vs original)?
double g_ourW = 0.0;    // GetOffset W field (offsetX)
double g_ourH = 0.0;    // GetOffset H field (already negated)
double g_ourMin = 1.0;  // match quality (0=best)

// Compute our result in the native convention from a detection.
// The original mirrors the frame 180 deg (down-vision camera mirror mode) before
// cvGetRectSubPix at center (imgW/2+mvoX, imgH/2+mvoY). Our detector reads the
// UNmirrored frame, so a mark at (cx,cy) is at (imgW-cx, imgH-cy) in the mirrored
// frame the original uses. Net (verified against the original's aligned output):
//   W =  (imgW/2 - cx - mvoX) / 1.5
//   H = -(imgH/2 - cy - mvoY) / 1.5
void set_our_result(const vis::MarkResult& mr) {
    // Always own the result for the down-vision mark modes (we no longer call the
    // original, so its offset globals are stale). On a detection MISS, return a
    // ZERO offset ("no correction") -- NOT a sentinel. The host applies GetOffset
    // to the head setpoint every frame, unconditionally (it checks neither a
    // found-flag nor GetMin_val), so any non-zero "not found" value is applied as a
    // real move and the head WANDERS on transient dropouts; zero = "stay put" holds
    // through a brief miss. (The original writes a huge 999999 sentinel here and the
    // host doesn't guard that either -- it just rarely drops out, locking the larger
    // solder-mask disc instead of the 1mm copper.)
    if (!mr.found || !mr.headerOk) {
        g_ourW = 0.0;
        g_ourH = 0.0;
        g_ourMin = 1.0;
        g_ours = true;
        return;
    }
    const double cxC = mr.imgW / 2.0;
    const double cyC = mr.imgH / 2.0;
    g_ourW = (cxC - mr.cx - g_mvoX) / kScale;
    g_ourH = -(cyC - mr.cy - g_mvoY) / kScale;  // GetOffset returns H negated
    g_ourMin = 0.05;                            // low = good match
    g_ours = true;
}

// Optional capture/logging (opt-in via trigger file).
void maybe_log(const void* frame, const char* func, int algo, int range, const vis::MarkResult& mr) {
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

    char line[640];
    std::snprintf(line, sizeof line,
                  "%04d,%s,algo=%d,range=%d,mvo=%.1f/%.1f,area=%d/%d,"
                  "ours,found=%d,cx=%.2f,cy=%.2f,W=%.4f,H=%.4f,min=%.3f,"
                  "ms,queryFrame=%.1f,detect=%.1f,render=%.1f,"
                  "ipl,ok=%d,w=%d,h=%d,origin=%d,fhash=%u",
                  idx, func, algo, range, g_mvoX, g_mvoY, g_areaMin, g_areaMax,
                  mr.found ? 1 : 0, mr.cx, mr.cy, g_ourW, g_ourH, g_ourMin,
                  g_qfMs, g_detMs, g_renderMs,
                  mr.headerOk ? 1 : 0, mr.imgW, mr.imgH, mr.imgOrigin, mr.frameHash);
    cap::log_line(line);
}

// Reference point (where the mark sits when aligned), in unflipped frame coords:
// frame center minus the markVisionOffset. Derived from the frame's actual
// dimensions, falling back to the default geometry if the header can't be read.
void reference_point(const void* frame, double* refX, double* refY) {
    int w = kDefaultFrameW, h = kDefaultFrameH;
    vis::frame_size(frame, &w, &h);
    *refX = w / 2.0 - g_mvoX;
    *refY = h / 2.0 - g_mvoY;
}

// Shared scaffolding for the down-vision mark checks we own (CheckMark2,
// CheckTemplate): time the detector, publish OUR result via the GetOffset
// plumbing, render our own preview (falling back to the original's render only if
// ours fails), and log. `detect(refX, refY, searchR)` runs the mode-specific
// detector; `origRender()` calls the original check purely for its preview side
// effect. No temporal smoothing — an EMA here lagged the reported center so the
// host settled on the lagged value and the green cross then drifted off target;
// the tight radius bracket + both-field detection already give a stable center.
template <class Detect, class OrigRender>
int run_mark_check(const void* f, int hwnd, const char* name, int logAlgo, int logRange,
                   Detect&& detect, OrigRender&& origRender) {
    double refX = 0.0, refY = 0.0;
    reference_point(f, &refX, &refY);
    const int searchR = (g_areaMin > kAreaMinPx && g_areaMin < kAreaMaxPx)
                            ? g_areaMin
                            : kDefaultSearchPx;

    auto t0 = clk::now();
    vis::MarkResult mr = detect(refX, refY, searchR);  // read-only
    g_detMs = ms_since(t0);

    set_our_result(mr);  // our offset drives placement

    auto t1 = clk::now();
    if (!vis::render_preview(f, reinterpret_cast<void*>(static_cast<intptr_t>(hwnd)),
                             g_mvoX, g_mvoY, searchR, mr))
        origRender();  // fall back to the original's render
    g_renderMs = ms_since(t1);

    maybe_log(f, name, logAlgo, logRange, mr);
    return 0;
}
}  // namespace

// Alias for the shim's typo'd P/Invoke name (SetUpisionOffsetXY, missing 'V').
#pragma comment(linker, "/EXPORT:_SetUpisionOffsetXY@16=_SetUpVisionOffsetXY@16")

extern "C" {

int __stdcall Initialize(int w, int h) {
    return mv::orig::Initialize(w, h);
}
void __stdcall Release(void) {
    mv::orig::Release();
}
HANDLE __stdcall GetCameraHandle(int idx) {
    return mv::orig::GetCameraHandle(idx);
}
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
    constexpr int kSettleFrames = 2;  // extra camera frames (~66 ms) to ring down
    const int kTarget = 1 + kSettleFrames;
    const auto deadline = t0 + std::chrono::milliseconds(160);
    unsigned int h = vis::frame_hash(r);
    unsigned int cur = h;
    int newCount = (h != 0 && h != g_lastFrameHash) ? 1 : 0;
    while (h != 0 && newCount < kTarget && clk::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        void* r2 = mv::orig::QueryFrame(cam, timeout);
        unsigned int h2 = vis::frame_hash(r2);
        if (r2 && h2 != 0) {
            r = r2;
            if (h2 != cur) {
                ++newCount;
                cur = h2;
            }
            h = h2;
        }
    }
    if (h != 0) g_lastFrameHash = h;
    g_qfMs = ms_since(t0);
    return r;
}

// Up-vision component / nozzle / down-component: keep original; mark our result
// invalid so GetOffset passes through to the original for these modes.
int __stdcall CheckComp(void* f, int hwnd, int w, int h) {
    g_ours = false;
    return mv::orig::CheckComp(f, hwnd, w, h);
}
int __stdcall CheckNozzle(void* f, int hwnd, int w, int h) {
    g_ours = false;
    return mv::orig::CheckNozzle(f, hwnd, w, h);
}
int __stdcall DownCheckComp(void* f, int hwnd, int w, int h, int p) {
    g_ours = false;
    return mv::orig::DownCheckComp(f, hwnd, w, h, p);
}
void* __stdcall DownShow(void* f) {
    g_ours = false;
    return mv::orig::DownShow(f);
}

// CheckMark: algo==0 is the down-vision "Round" mark mode -> our circle detector
// sized by the operator's Range (drives placement via GetOffset, same dispatch as
// CheckMark2). Other algo values (-1 preview-only, legacy shape modes) pass
// through. r = strength (unused by the Round detector).
int __stdcall CheckMark(void* f, int hwnd, int w, int h, int algo, int r) {
    if (algo != 0) {
        g_ours = false;
        return mv::orig::CheckMark(f, hwnd, w, h, algo, r);
    }
    return run_mark_check(f, hwnd, "CheckMark", algo, r, [&](double refX, double refY, int searchR) { return vis::detect_round_mark(f, refX, refY, searchR, r); }, [&] { mv::orig::CheckMark(f, hwnd, w, h, algo, r); });
}

// CheckMark2: the down-vision fiducial mode (what the machine uses). Detect with
// our OpenCV pipeline; GetOffset/GetMin_val return OUR result. The original is
// invoked only as a preview fallback (run_mark_check). CheckMark2 args:
// algo = template size px (from the 1.2mm setting), r = strength.
int __stdcall CheckMark2(void* f, int hwnd, int w, int h, int algo, int r) {
    return run_mark_check(f, hwnd, "CheckMark2", algo, r, [&](double refX, double refY, int searchR) { return vis::detect_circle_mark(f, algo, refX, refY, searchR); }, [&] { mv::orig::CheckMark2(f, hwnd, w, h, algo, r); });
}

int __stdcall GetTemplate(void* f, unsigned char* out, int sz, double p) {
    return mv::orig::GetTemplate(f, out, sz, p);
}

// CheckTemplate (down-vision ImageTemplate mode): field-aware template match.
// The host supplies its mark template here (and null on most frames). Cache the
// latest one, match it every frame, and drive placement via GetOffset — same
// dispatch as CheckMark2. Falls back to the original render only if our preview
// render fails.
int __stdcall CheckTemplate(void* f, int hwnd, int w, int h, unsigned char* t, int sz, double th, int m) {
    cache_template(t, sz);  // store the latest template (no-op on null/unchanged)
    return run_mark_check(f, hwnd, "CheckTemplate", sz, m, [&](double refX, double refY, int searchR) {
            vis::MarkResult mr;
            if (g_tmpl.sz > 0)
                mr = vis::detect_template_mark(f, g_tmpl.buf, g_tmpl.sz, th, m,
                                               refX, refY, searchR, &g_tmpl.scale);
            return mr; }, [&] { mv::orig::CheckTemplate(f, hwnd, w, h, t, sz, th, m); });
}
int __stdcall TemplateVision(void* f, int hwnd, int w, int h, unsigned char* t, int sz, double th, int m) {
    return mv::orig::TemplateVision(f, hwnd, w, h, t, sz, th, m);
}

int __stdcall OpenPerspectiveTransform(void) {
    return mv::orig::OpenPerspectiveTransform();
}
int __stdcall ClosePerspectiveTransform(void) {
    return mv::orig::ClosePerspectiveTransform();
}
int __stdcall SetPerspectiveMatrix(double* m9) {
    return mv::orig::SetPerspectiveMatrix(m9);
}
void* __stdcall GetLowResTransformParam(void* f, int hwnd, int w, int h, int a, int b, int* oa, double* ob, double* om) {
    return mv::orig::GetLowResTransformParam(f, hwnd, w, h, a, b, oa, ob, om);
}

int __stdcall OpenPerspectiveTransform7(void) {
    return mv::orig::OpenPerspectiveTransform7();
}
int __stdcall ClosePerspectiveTransform7(void) {
    return mv::orig::ClosePerspectiveTransform7();
}
int __stdcall SetPerspectiveMatrix7(double* m9) {
    return mv::orig::SetPerspectiveMatrix7(m9);
}
void* __stdcall GetLowResTransformParam7(void* f, int w, int h, int* oa, double* ob, double* om) {
    return mv::orig::GetLowResTransformParam7(f, w, h, oa, ob, om);
}

void __stdcall Draw(void* f, int hwnd, int w, int h) {
    mv::orig::Draw(f, hwnd, w, h);
}
void __stdcall Draw2(unsigned char* rgb, int sz, void* f, int w, int h) {
    mv::orig::Draw2(rgb, sz, f, w, h);
}
int __stdcall myLoadImage(const char* p) {
    return mv::orig::myLoadImage(p);
}
int __stdcall mySaveImage(void* f, const char* p) {
    return mv::orig::mySaveImage(f, p);
}

// Result accessors: return OUR result when the last check was ours, else the
// original's (so up-vision / component modes are unaffected).
void __stdcall GetOffset(double* x, double* y, double* w, double* h, double* a) {
    if (g_ours) {
        *x = 0.0;
        *y = 0.0;
        *w = g_ourW;
        *h = g_ourH;
        *a = 0.0;  // our down-vision modes never report an angle
    } else {
        mv::orig::GetOffset(x, y, w, h, a);
    }
}
void __stdcall GetMin_val(double* v) {
    if (g_ours) {
        *v = g_ourMin;
    } else {
        mv::orig::GetMin_val(v);
    }
}

void __stdcall SetThreshold(int v) {
    mv::orig::SetThreshold(v);
}
void __stdcall SetCompAngle(double v) {
    mv::orig::SetCompAngle(v);
}
void __stdcall SetCompSizeWHA(double w, double h, double a) {
    mv::orig::SetCompSizeWHA(w, h, a);
}
void __stdcall SetMarkVisionOffsetXY(double x, double y) {
    g_mvoX = x;
    g_mvoY = y;
    mv::orig::SetMarkVisionOffsetXY(x, y);
}
void __stdcall SetUpVisionOffsetXY(double x, double y) {
    mv::orig::SetUpVisionOffsetXY(x, y);
}
void __stdcall SetMarkVisionAreaMinMax(int mn, int mx) {
    g_areaMin = mn;
    g_areaMax = mx;
    mv::orig::SetMarkVisionAreaMinMax(mn, mx);
}
void __stdcall SetIsCheckTemplate(int b) {
    mv::orig::SetIsCheckTemplate(b);
}
void __stdcall SetUpVisionCameraMirrorMode(int m) {
    mv::orig::SetUpVisionCameraMirrorMode(m);
}
void __stdcall SetDownVisionCameraMirrorMode(int m) {
    mv::orig::SetDownVisionCameraMirrorMode(m);
}

}  // extern "C"
