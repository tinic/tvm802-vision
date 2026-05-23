#include "detect_common.h"

#include "iplframe.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

namespace vis {
namespace {
using detail::as_valid_ipl;
using detail::wrap_ipl;

// Two deinterlaced fields whose detected centers are within this many pixels are
// treated as settled and averaged; otherwise only the newest field is reported.
constexpr double kSettleAgreePx = 4.0;

// Interlace comb / motion metric (detect_with_fields). A central pixel whose
// deviations from its two vertical neighbours have the same sign and a product
// over kCombPixThresh is a "comb" pixel -- the zigzag of two misaligned fields.
// When the comb fraction exceeds kCombMovingFrac the head is moving. Settled
// board detail sits ~0.03-0.07; real motion is the 0.10-0.39 tail (calibrated).
constexpr int kCombPixThresh = 300;
constexpr double kCombMovingFrac = 0.10;

// Copy a per-field detection into the combined result (the fields shared by both
// detectors). Used by detect_with_fields' combine branches.
void assign_result(MarkResult& dst, const MarkResult& src) {
    dst.found = true;
    dst.cx = src.cx;
    dst.cy = src.cy;
    dst.radius = src.radius;
    dst.quality = src.quality;
}
}  // namespace

unsigned int frame_hash(const void* frame) {
    const IplImage* ipl = as_valid_ipl(frame);
    if (!ipl) return 0;
    unsigned int h = 2166136261u;  // FNV-1a offset basis
    const unsigned char* d = reinterpret_cast<const unsigned char*>(ipl->imageData);
    const int ws = ipl->widthStep, rowbytes = ipl->width * ipl->nChannels;
    for (int y = 0; y < ipl->height; y += 37)
        for (int x = 0; x < rowbytes; x += 17)
            h = (h ^ d[y * ws + x]) * 16777619u;
    return h ? h : 1u;  // 0 reserved for invalid
}

bool frame_size(const void* frame, int* w, int* h) {
    const IplImage* ipl = as_valid_ipl(frame);
    if (!ipl) return false;
    if (w) *w = ipl->width;
    if (h) *h = ipl->height;
    return true;
}

bool save_frame(const void* frame, const char* path) {
    if (!path) return false;
    try {  // no-throw barrier: never let an OpenCV failure reach the host
        int origin = -1;
        cv::Mat wrapped = wrap_ipl(frame, origin);
        if (wrapped.empty()) return false;
        // ORIGIN: ignore the flag (treat top-down) so the captured PNG matches what
        // the detector sees -- offline tuning runs on these frames. TODO(origin):
        // not validated on this hardware -- revisit after the merge (see work item).
        (void)origin;
        cv::Mat out = wrapped.clone();  // copy; never touch live buffer
        return cv::imwrite(path, out);
    } catch (...) {
        return false;
    }
}

// Field-aware wrapper: deinterlace the analog frame into its two time-separated
// fields (even/odd scanlines, ~1/60s apart), upscale each back to full height,
// run `perField` on BOTH, and combine. The STK1150 weaves the fields into one
// frame, so under head motion they comb ("double image"); detecting on each
// single-instant field and taking the best (or averaging when they agree)
// removes the combing and ~doubles temporal sampling. The down-vision read has
// no settle-delay (unlike the up camera), so it fires mid-motion at high speed.
//
// `perField` runs on one upscaled full-height field and returns a MarkResult
// with cx in full-frame x and cy in field-row space; this wrapper applies the
// +-0.5 row correction and combines. Reused by every field-aware detector
// (circular, template, contour). NON-DESTRUCTIVE: copies out of the frame buffer.
//
// Motion-adaptive (see the comb metric below): under MOTION it reports only the
// most-recent single-instant field. When SETTLED it uses the two-field AVERAGE,
// EXCEPT when wovenWhenSettled is set (Round contour) -- then it detects on the
// sharp full-res woven frame, whose edges an upscaled field would soften.
MarkResult detect_with_fields(
    const void* frame,
    const std::function<MarkResult(const cv::Mat&)>& perField,
    bool wovenWhenSettled) {
    MarkResult r;

    const IplImage* ipl = as_valid_ipl(frame);
    if (!ipl) return r;

    r.headerOk = true;
    r.imgW = ipl->width;
    r.imgH = ipl->height;
    r.imgOrigin = ipl->origin;

    r.frameHash = frame_hash(frame);  // stale-frame diagnostic (see compare.log)

    // A frame too small to split into two fields would make the deinterlace
    // resize throw on a 0-row field. Real captures are 640x480; anything below a
    // few pixels is a malformed header -> report not-found rather than risk it.
    if (ipl->height < 4 || ipl->width < 4) return r;

    // EXCEPTION BARRIER: this module is called across a plain C ABI from the host
    // (SurfaceMount.exe). An OpenCV/STL exception (corrupt frame, out-of-memory,
    // failed assertion) unwinding past that boundary is undefined behavior. Catch
    // everything here and report "not found", which the host already handles.
    try {
        cv::Mat wrapped(ipl->height, ipl->width,
                        CV_MAKETYPE(CV_8U, ipl->nChannels),
                        ipl->imageData, static_cast<size_t>(ipl->widthStep));

        // ORIGIN: we IGNORE the IplImage origin flag and treat the frame as
        // top-down. The down-vision camera toggles origin on camera-switch without
        // changing the pixel layout, so honoring it flips detection (and the
        // preview) upside-down. TODO(origin): neither ignore nor respect was
        // validated on this hardware -- revisit after the merge (see work item).
        const cv::Mat& img = wrapped;

        cv::Mat gray;
        if (img.channels() == 3)
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        else
            gray = img;

        const size_t step = gray.step[0];
        cv::Mat evenF, oddF, evenU, oddU;
        cv::Mat(gray.rows / 2, gray.cols, gray.type(), gray.data, step * 2).copyTo(evenF);
        cv::Mat(gray.rows / 2, gray.cols, gray.type(), gray.data + step, step * 2).copyTo(oddF);
        // Bob-deinterlace each half-height field back to full height. INTER_CUBIC
        // (not LINEAR) preserves the copper edge sharpness that HoughCircles'
        // param2 accumulator and the circularity ring-sample at fr+-3 both rely
        // on; the ~1px vertical softening from bilinear blunts both. Two small
        // resizes/frame -> negligible cost.
        cv::resize(evenF, evenU, cv::Size(gray.cols, gray.rows), 0, 0, cv::INTER_CUBIC);
        cv::resize(oddF, oddU, cv::Size(gray.cols, gray.rows), 0, 0, cv::INTER_CUBIC);

        // Interlace comb / motion metric on the woven frame, central region: the
        // fraction of "zigzag" pixels (deviating from both vertical neighbours the
        // same way). High => the two ~1/60s-apart fields are misaligned => moving.
        const int cx0 = std::max(1, gray.cols / 2 - 150), cx1 = std::min(gray.cols - 1, gray.cols / 2 + 150);
        const int cy0 = std::max(1, gray.rows / 2 - 120), cy1 = std::min(gray.rows - 1, gray.rows / 2 + 120);
        long comb = 0, total = 0;
        for (int y = cy0; y < cy1; ++y) {
            const uchar* rm = gray.ptr<uchar>(y);
            const uchar* ra = gray.ptr<uchar>(y - 1);
            const uchar* rb = gray.ptr<uchar>(y + 1);
            for (int x = cx0; x < cx1; ++x) {
                const int d1 = static_cast<int>(rm[x]) - static_cast<int>(ra[x]);
                const int d2 = static_cast<int>(rm[x]) - static_cast<int>(rb[x]);
                if (d1 * d2 > kCombPixThresh) ++comb;
                ++total;
            }
        }
        r.combFrac = total > 0 ? static_cast<double>(comb) / static_cast<double>(total) : 0.0;

        // Pick the detection source by motion state. BFF: the even rows (0,2,4) are
        // the most-recent field; even samples rows 2i, so its y needs the -0.5.
        if (r.combFrac > kCombMovingFrac) {
            // MOVING: only the most-recent single-instant field (no comb).
            MarkResult re = perField(evenU);
            if (re.found) {
                re.cy -= 0.5;
                assign_result(r, re);
            }
        } else if (wovenWhenSettled) {
            // SETTLED, contour (Round): the sharp full-res woven frame.
            MarkResult rw = perField(gray);
            if (rw.found) assign_result(r, rw);
        } else {
            // SETTLED, Circular/ImageTemplate: the validated two-field AVERAGE --
            // robust to residual ring-down and halves the sub-pixel scatter.
            MarkResult re = perField(evenU);
            MarkResult ro = perField(oddU);
            if (re.found) re.cy -= 0.5;  // even field samples rows 2i   -> full-frame y
            if (ro.found) ro.cy += 0.5;  // odd  field samples rows 2i+1 -> full-frame y
            if (re.found && ro.found) {
                const double dx = re.cx - ro.cx, dy = re.cy - ro.cy;
                if (dx * dx + dy * dy <= kSettleAgreePx * kSettleAgreePx) {
                    r.found = true;
                    r.cx = 0.5 * (re.cx + ro.cx);
                    r.cy = 0.5 * (re.cy + ro.cy);
                    r.radius = 0.5 * (re.radius + ro.radius);
                    r.quality = std::max(re.quality, ro.quality);
                } else {
                    assign_result(r, re);  // BFF: even is newest
                }
            } else if (re.found) {
                assign_result(r, re);
            } else if (ro.found) {
                assign_result(r, ro);
            }
        }
    } catch (...) {
        r.found = false;  // never let an exception reach the host
    }
    return r;
}

}  // namespace vis
