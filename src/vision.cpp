#include "vision.h"

#include "iplframe.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <vector>

namespace vis {

namespace {
using detail::as_valid_ipl;
using detail::wrap_ipl;

// ---- Tuning constants -------------------------------------------------------
// The values the README's "Tuning" section refers to live here, in one place,
// instead of being scattered through the detector bodies. Changing detection
// behavior should mean editing this block — not hunting for literals inline.
// (Fixed DSP kernel sizes — median 3, bilateral 5/25/25, CLAHE 3.0/8x8, the 3x3
// blur — stay inline below: they are filter choices, not detection knobs.)

struct CircleParams {
    // Radius bracket as a fraction of the host mark "size" arg. Locks the inner
    // 1mm copper (~0.32*size); excludes the larger solder-mask ring (~0.63*size)
    // that otherwise causes flip-flop jitter.
    double radiusMinFrac = 0.22, radiusMaxFrac = 0.42;
    int radiusMinPx = 8;                    // floor on the min radius
    int markSizeLo = 16, markSizeHi = 400;  // valid host "size" range
    int radiusFallbackMin = 12, radiusFallbackMax = 28;
    // Frame / search gates.
    double meanLo = 6.0, meanHi = 250.0;  // dropped (black) / blown (white)
    int searchFallbackPx = 150;           // when the host "Range" is unset
    int roiPad = 6;                       // ROI = search + maxR + pad
    int roiMinSlackPx = 8;                // ROI must exceed 2*minR by this
    // HoughCircles.
    double houghDp = 1.0;
    int houghParam1 = 80, houghParam2 = 25;
    // Circularity gate: edge support sampled around the circumference.
    int circSamples = 24, circRingDelta = 3, circEdgeDelta = 18;
    double circEdgeFrac = 0.70;
    // Contrast gate: a solid copper blob vs. its surrounding annulus.
    int contrastPad = 4;
    double contrastInnerFrac = 0.36, contrastOuterLo = 1.32, contrastOuterHi = 2.25;
    double contrastMinDelta = 28.0;
    // Sub-pixel intensity-weighted centroid refine.
    double refineWindowFrac = 1.3, refineMaxShiftFrac = 0.7;
    int refineWindowPad = 4;
};
constexpr CircleParams kCircle;

struct TemplateParams {
    double meanLo = 6.0, meanHi = 250.0;  // dropped / blown
    int blurKernel = 5;                   // pre-match smoothing
    int searchMinPx = 8;                  // host "Range" honored above this
    int searchFallbackPx = 150;
    int roiPad = 6;  // ROI = search + size/2 + pad
    // Multi-scale SQDIFF sweep (matchTemplate is not scale-invariant).
    double scaleLockLo = 0.4, scaleLockHi = 1.4;  // a cached scale in range = locked
    double scaleSweepLo = 0.5, scaleSweepHi = 1.3001, scaleStep = 0.1;
    double scaleResweepThresh = 0.42;  // locked scale this bad -> re-sweep
    int minTemplateDim = 10;
    // Accept bar: SQDIFF must be <= acceptBase - acceptPerStrength * strength
    // (strength 1 -> 0.545 loose, strength 10 -> 0.23 strict).
    double acceptBase = 0.58, acceptPerStrength = 0.035;
    int strengthLo = 1, strengthHi = 10, strengthDefault = 5;
};
constexpr TemplateParams kTmpl;

// Round (CheckMark algo==0) contour-circularity params (the native Round algo --
// HoughCircles is forbidden in this mode).
struct ContourParams {
    double meanLo = 6.0, meanHi = 250.0;  // dropped (black) / blown (white)
    int searchFallbackPx = 150;           // search radius when the host Range is unset
    int roiPad = 50;                      // ROI = Range + pad around the reference
    int roiMinDimPx = 40;                 // skip the ROI crop if it would be smaller
    int gaussKernel = 13;                 // heavy blur -> cleaner, more complete rings
    // Fixed physical feature-size gate: 0.5mm..3.5mm diameter (covers all real
    // fiducials), as a RADIUS bracket at ~40 px/mm (1mm copper ~ d40px here).
    // Independent of Range; Range stays the search radius. min=0.5/2*40, max=3.5/2*40.
    int minRadiusPx = 10, maxRadiusPx = 70;
    // Canny: low = cannyStep*strength + cannyBase, high = low + cannyHighDelta.
    int cannyBase = 5, cannyStep = 15, cannyHighDelta = 20;
    int maskMorphKernel = 5;  // close broken Canny rings (the deinterlace softens edges)
    double circTol = 0.45;    // |pi*r^2 - area| < pi*r^2*tol  (the native test)
    double qTieBand = 0.02;   // quality band within which distance breaks ties
};
constexpr ContourParams kContour;

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

// Core HoughCircles detector over ONE deinterlaced field, given an explicit radius
// bracket [minRfull, maxRfull]. Used by the Circular mode ONLY; Round (CheckMark)
// uses the contour detector, never Hough. cx in full-frame x; cy in field-row space
// (caller applies the +-0.5 field correction). Returns found=false on no mark.
static MarkResult detect_one_field_circle(const cv::Mat& gFull, int minRfull, int maxRfull,
                                          double refX, double refY, int searchRadiusPx) {
    MarkResult r;

    // Restrict the WHOLE pipeline to a ROI around the reference. The mark is
    // always within the search radius of it, so this cuts detection latency ~2x
    // (HoughCircles + filters over a small window) with identical results --
    // lower latency tightens the host's closed-loop lock-in. No reference (or a
    // ROI too small) -> use the full field.
    const double rxF = (refX >= 0.0) ? refX : gFull.cols / 2.0;
    const double ryF = (refY >= 0.0) ? refY : gFull.rows / 2.0;
    const int sr = (searchRadiusPx > 0) ? searchRadiusPx : kCircle.searchFallbackPx;
    cv::Rect crop(0, 0, gFull.cols, gFull.rows);
    if (refX >= 0.0 && refY >= 0.0) {
        const int R = sr + maxRfull + kCircle.roiPad;
        cv::Rect c = cv::Rect(cvRound(rxF) - R, cvRound(ryF) - R, 2 * R, 2 * R) & cv::Rect(0, 0, gFull.cols, gFull.rows);
        const int slack = 2 * minRfull + kCircle.roiMinSlackPx;
        if (c.width >= slack && c.height >= slack) crop = c;
    }
    const cv::Mat g = gFull(crop);
    const double cxRef = rxF - crop.x, cyRef = ryF - crop.y;  // reference in crop coords

    const double imgMean = cv::mean(g)[0];
    if (imgMean < kCircle.meanLo || imgMean > kCircle.meanHi) return r;  // dropped/blown

    // Denoise -> illumination normalize -> light smooth. Denoise BEFORE CLAHE
    // (CLAHE amplifies noise). medianBlur kills analog impulse pixels; the
    // edge-preserving bilateral removes sensor noise without softening the
    // copper edge. bilateralFilter cannot run in-place: src (med) != dst (den).
    cv::Mat med, den, norm, det;
    cv::medianBlur(g, med, 3);
    cv::bilateralFilter(med, den, /*d*/ 5, /*sigmaColor*/ 25, /*sigmaSpace*/ 25);
    cv::createCLAHE(3.0, cv::Size(8, 8))->apply(den, norm);
    cv::GaussianBlur(norm, det, cv::Size(3, 3), 1.0);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(det, circles, cv::HOUGH_GRADIENT, kCircle.houghDp,
                     /*minDist*/ std::max(8.0, det.rows / 4.0), kCircle.houghParam1,
                     kCircle.houghParam2, minRfull, maxRfull);
    if (circles.empty()) return r;

    int best = 0;
    double bestD = 1e18;
    for (size_t i = 0; i < circles.size(); ++i) {
        const double dx = circles[i][0] - cxRef, dy = circles[i][1] - cyRef;
        const double d = dx * dx + dy * dy;
        if (d < bestD) {
            bestD = d;
            best = static_cast<int>(i);
        }
    }
    const size_t bi = static_cast<size_t>(best);
    double fx = circles[bi][0], fy = circles[bi][1], fr = circles[bi][2];

    // Search-area ("Range") constraint: reject detections too far from the ref.
    if (searchRadiusPx > 0) {
        const double ddx = fx - cxRef, ddy = fy - cyRef;
        if (ddx * ddx + ddy * ddy > double(searchRadiusPx) * searchRadiusPx)
            return r;
    }

    // Circularity: reject HoughCircles votes on rectangular pads (need a real
    // edge around most of the circumference). The edge-support fraction is also
    // surfaced as MarkResult::quality (diagnostic only — see the test harness).
    double circQuality = 0.0;
    {
        const int N = kCircle.circSamples;
        int hits = 0, valid = 0;
        for (int k = 0; k < N; ++k) {
            const double a = 2.0 * CV_PI * k / N, ca = std::cos(a), sa = std::sin(a);
            const int xin = cvRound(fx + (fr - kCircle.circRingDelta) * ca);
            const int yin = cvRound(fy + (fr - kCircle.circRingDelta) * sa);
            const int xou = cvRound(fx + (fr + kCircle.circRingDelta) * ca);
            const int you = cvRound(fy + (fr + kCircle.circRingDelta) * sa);
            if (xin < 0 || yin < 0 || xin >= det.cols || yin >= det.rows) continue;
            if (xou < 0 || you < 0 || xou >= det.cols || you >= det.rows) continue;
            ++valid;
            if (std::abs(int(det.at<uchar>(yin, xin)) - int(det.at<uchar>(you, xou))) > kCircle.circEdgeDelta)
                ++hits;
        }
        if (valid < N / 2 || hits < kCircle.circEdgeFrac * valid) return r;
        circQuality = double(hits) / valid;
    }

    // Contrast gate: a real copper dot is a solid bright/dark blob vs annulus.
    {
        const int rr = std::max(2, int(fr));
        double sin_ = 0, sout = 0;
        int nin = 0, nout = 0;
        for (int dy = -rr - kCircle.contrastPad; dy <= rr + kCircle.contrastPad; ++dy)
            for (int dx = -rr - kCircle.contrastPad; dx <= rr + kCircle.contrastPad; ++dx) {
                const int xx = cvRound(fx) + dx, yy = cvRound(fy) + dy;
                if (xx < 0 || yy < 0 || xx >= det.cols || yy >= det.rows) continue;
                const double d2 = double(dx) * dx + double(dy) * dy;
                if (d2 < kCircle.contrastInnerFrac * rr * rr) {
                    sin_ += det.at<uchar>(yy, xx);
                    ++nin;
                } else if (d2 > kCircle.contrastOuterLo * rr * rr &&
                           d2 < kCircle.contrastOuterHi * rr * rr) {
                    sout += det.at<uchar>(yy, xx);
                    ++nout;
                }
            }
        if (nin > 0 && nout > 0 &&
            std::abs(sin_ / nin - sout / nout) < kCircle.contrastMinDelta)
            return r;
    }

    // Sub-pixel refine: intensity-weighted centroid around the Hough center.
    const int m = static_cast<int>(fr * kCircle.refineWindowFrac) + kCircle.refineWindowPad;
    cv::Rect roi(cvRound(fx) - m, cvRound(fy) - m, 2 * m, 2 * m);
    roi &= cv::Rect(0, 0, norm.cols, norm.rows);
    if (roi.width > 6 && roi.height > 6) {
        cv::Mat patch;
        norm(roi).convertTo(patch, CV_32F);
        const double mean = cv::mean(patch)[0];
        cv::Mat w = patch - mean;
        cv::threshold(w, w, 0.0, 0.0, cv::THRESH_TOZERO);
        const double sw = cv::sum(w)[0];
        if (sw > 1.0) {
            cv::Mat colW, rowW;
            cv::reduce(w, colW, 0, cv::REDUCE_SUM, CV_64F);
            cv::reduce(w, rowW, 1, cv::REDUCE_SUM, CV_64F);
            double sx = 0, sy = 0;
            for (int x = 0; x < colW.cols; ++x) sx += colW.at<double>(0, x) * x;
            for (int y = 0; y < rowW.rows; ++y) sy += rowW.at<double>(y, 0) * y;
            const double rx = roi.x + sx / sw, ry = roi.y + sy / sw;
            if (std::abs(rx - fx) <= fr * kCircle.refineMaxShiftFrac &&
                std::abs(ry - fy) <= fr * kCircle.refineMaxShiftFrac) {
                fx = rx;
                fy = ry;
            }
        }
    }

    r.found = true;
    r.cx = fx + crop.x;  // map crop coords -> field coords
    r.cy = fy + crop.y;
    r.radius = fr;
    r.quality = circQuality;
    return r;
}

// Circular (CheckMark2): radius bracket from the host "size" arg, locked to the
// inner 1mm copper (excludes the larger solder-mask ring -> no flip-flop jitter).
static MarkResult detect_one_field(const cv::Mat& gFull, int markSizePx,
                                   double refX, double refY, int searchRadiusPx) {
    int minRfull, maxRfull;
    if (markSizePx >= kCircle.markSizeLo && markSizePx <= kCircle.markSizeHi) {
        minRfull = std::max(kCircle.radiusMinPx,
                            static_cast<int>(markSizePx * kCircle.radiusMinFrac));
        maxRfull = static_cast<int>(markSizePx * kCircle.radiusMaxFrac);
    } else {
        minRfull = kCircle.radiusFallbackMin;
        maxRfull = kCircle.radiusFallbackMax;
    }
    return detect_one_field_circle(gFull, minRfull, maxRfull, refX, refY, searchRadiusPx);
}

// Round (CheckMark algo==0): contour-circularity, faithful to the native Round.
// HoughCircles is FORBIDDEN in this mode (it stays in CheckMark2/Circular only).
// 13x13 Gaussian -> strength-scaled Canny -> Range-circle mask + morph-close the
// broken ring -> findContours -> size + circularity + search gate -> the winning
// contour's moment centroid. The size gate is a FIXED physical bracket (0.5..3.5mm
// feature diameter, kContour.min/maxRadiusPx). The operator's Range (searchRadiusPx)
// is used AS-IS for the search-area gate, NOT for sizing. cx in full-frame x; cy in
// field-row space (caller applies the +-0.5 correction).
static MarkResult detect_one_field_contour(const cv::Mat& gFull, double refX, double refY,
                                           int searchRadiusPx, int strength) {
    MarkResult r;
    const int sN = std::clamp(strength, 1, 10);

    const double rxF = (refX >= 0.0) ? refX : gFull.cols / 2.0;
    const double ryF = (refY >= 0.0) ? refY : gFull.rows / 2.0;
    const int sr = (searchRadiusPx > 0) ? searchRadiusPx : kContour.searchFallbackPx;
    cv::Rect crop(0, 0, gFull.cols, gFull.rows);
    if (refX >= 0.0 && refY >= 0.0) {
        const int R = sr + kContour.roiPad;
        cv::Rect c = cv::Rect(cvRound(rxF) - R, cvRound(ryF) - R, 2 * R, 2 * R) & cv::Rect(0, 0, gFull.cols, gFull.rows);
        if (c.width > kContour.roiMinDimPx && c.height > kContour.roiMinDimPx) crop = c;
    }
    const cv::Mat g = gFull(crop);
    const double cxRef = rxF - crop.x, cyRef = ryF - crop.y;

    const double imgMean = cv::mean(g)[0];
    if (imgMean < kContour.meanLo || imgMean > kContour.meanHi) return r;  // dropped/blown

    cv::Mat sm, edges;
    cv::GaussianBlur(g, sm, cv::Size(kContour.gaussKernel, kContour.gaussKernel), 0);
    const int cannyLo = kContour.cannyStep * sN + kContour.cannyBase;
    cv::Canny(sm, edges, cannyLo, cannyLo + kContour.cannyHighDelta, 3);

    // Restrict to the Range circle, then close the broken ring (the deinterlace
    // softens edges -> Canny rings come out broken -> fail circularity = dropouts).
    cv::Mat mask = cv::Mat::zeros(edges.size(), CV_8U);
    cv::circle(mask, cv::Point(cvRound(cxRef), cvRound(cyRef)), sr, cv::Scalar(255), -1);
    cv::bitwise_and(edges, mask, edges);
    cv::morphologyEx(edges, edges, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kContour.maskMorphKernel, kContour.maskMorphKernel)));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    if (contours.empty()) return r;

    const double minArea = CV_PI * kContour.minRadiusPx * kContour.minRadiusPx;
    int best = -1;
    double bestQ = -1.0, bestD = 1e18, fr = 0.0;
    for (size_t i = 0; i < contours.size(); ++i) {
        const std::vector<cv::Point>& c = contours[i];
        const double area = std::abs(cv::contourArea(c));
        if (area < minArea) continue;
        cv::Point2f ec;
        float encR = 0.0f;
        cv::minEnclosingCircle(c, ec, encR);
        if (encR < static_cast<float>(kContour.minRadiusPx) || encR > static_cast<float>(kContour.maxRadiusPx)) continue;
        const double pr2 = CV_PI * encR * encR;
        if (std::abs(pr2 - area) >= pr2 * kContour.circTol) continue;  // circularity
        const double dx = ec.x - cxRef, dy = ec.y - cyRef, dist2 = dx * dx + dy * dy;
        const double srd = static_cast<double>(searchRadiusPx);
        if (searchRadiusPx > 0 && dist2 > srd * srd) continue;  // search-area gate
        const double q = area / pr2;                            // ~1 = perfect circle
        if (q > bestQ + kContour.qTieBand || (q > bestQ - kContour.qTieBand && dist2 < bestD)) {
            bestQ = q;
            bestD = dist2;
            best = static_cast<int>(i);
            fr = encR;
        }
    }
    if (best < 0) return r;  // nothing circular passed

    const std::vector<cv::Point>& win = contours[static_cast<size_t>(best)];
    const cv::Moments mm = cv::moments(win);
    if (mm.m00 <= 0.0) return r;

    r.found = true;
    r.cx = mm.m10 / mm.m00 + crop.x;
    r.cy = mm.m01 / mm.m00 + crop.y;
    r.radius = fr;
    r.quality = std::min(1.0, bestQ);
    return r;
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
static MarkResult detect_with_fields(
    const void* frame,
    const std::function<MarkResult(const cv::Mat&)>& perField,
    bool wovenWhenSettled = false) {
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

MarkResult detect_circle_mark(const void* frame, int markSizePx,
                              double refX, double refY, int searchRadiusPx) {
    return detect_with_fields(frame, [&](const cv::Mat& g) {
        return detect_one_field(g, markSizePx, refX, refY, searchRadiusPx);
    });
}

// ImageTemplate per-field core: SQDIFF_NORMED multi-scale match of the
// already-prepped (180-flipped, 5x5-blurred) template `tb` against ONE
// full-height, deinterlaced gray field. Returns the matched template CENTER in
// full-frame x and field-row y; detect_with_fields applies the +-0.5 row
// correction and combines the two fields. Per-field counterpart of
// detect_one_field (Circular). score/quality carry the SQDIFF (lower = better).
static MarkResult detect_template_one_field(const cv::Mat& g, const cv::Mat& tb,
                                            int size, int strength,
                                            double refX, double refY,
                                            int searchRadiusPx, double* ioScale) {
    MarkResult r;
    const double imgMean = cv::mean(g)[0];
    if (imgMean < kTmpl.meanLo || imgMean > kTmpl.meanHi) return r;  // dropped/blown
    if (tb.cols >= g.cols || tb.rows >= g.rows) return r;

    cv::Mat gb;
    cv::GaussianBlur(g, gb, cv::Size(kTmpl.blurKernel, kTmpl.blurKernel), 0);  // smooth; no brightness-norm

    // Search ROI = the host's Range, honored directly (it's the red preview circle).
    const double rx = (refX >= 0.0) ? refX : g.cols / 2.0;
    const double ry = (refY >= 0.0) ? refY : g.rows / 2.0;
    const int sr = (searchRadiusPx > kTmpl.searchMinPx) ? searchRadiusPx : kTmpl.searchFallbackPx;
    const int R = sr + size / 2 + kTmpl.roiPad;
    cv::Rect crop = cv::Rect(cvRound(rx) - R, cvRound(ry) - R, 2 * R, 2 * R) & cv::Rect(0, 0, g.cols, g.rows);
    if (crop.width < size + 2 || crop.height < size + 2) return r;

    // Multi-scale SQDIFF: the red ring light blooms the mark differently between
    // template capture and runtime, and matchTemplate is NOT scale-invariant.
    // Sweep template scales, keep the lowest SQDIFF. LOCK the scale after the
    // first good match (ioScale) and only re-sweep if it degrades -- across the
    // two fields this means the even field sweeps and the odd reuses the lock.
    const cv::Mat cropImg = gb(crop);
    double bestVal = 2.0, bestScl = 1.0;
    cv::Point bestLoc;
    int bestW = tb.cols, bestH = tb.rows;
    cv::Mat bestResult;
    const bool locked = (ioScale && *ioScale >= kTmpl.scaleLockLo && *ioScale <= kTmpl.scaleLockHi);
    for (int pass = 0; pass < 2; ++pass) {
        const double a = (locked && pass == 0) ? *ioScale : kTmpl.scaleSweepLo;
        const double b = (locked && pass == 0) ? *ioScale : kTmpl.scaleSweepHi;
        const int nScl = static_cast<int>(std::lround((b - a) / kTmpl.scaleStep)) + 1;
        for (int si = 0; si < nScl; ++si) {
            const double scl = a + si * kTmpl.scaleStep;
            cv::Mat ts;
            if (std::fabs(scl - 1.0) < 1e-6)
                ts = tb;
            else
                cv::resize(tb, ts, cv::Size(), scl, scl, scl < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR);
            if (ts.cols < kTmpl.minTemplateDim || ts.rows < kTmpl.minTemplateDim ||
                ts.cols >= cropImg.cols || ts.rows >= cropImg.rows) continue;
            cv::Mat res;
            cv::matchTemplate(cropImg, ts, res, cv::TM_SQDIFF_NORMED);
            // Honor the Range as a CIRCLE: accept only peaks whose matched
            // template CENTER lies within sr of the reference.
            cv::Mat mask(res.size(), CV_8U, cv::Scalar(0));
            const cv::Point mc(cvRound(rx) - crop.x - ts.cols / 2,
                               cvRound(ry) - crop.y - ts.rows / 2);
            cv::circle(mask, mc, sr, cv::Scalar(255), -1);
            if (cv::countNonZero(mask) == 0) continue;
            double mn;
            cv::Point ml;
            cv::minMaxLoc(res, &mn, nullptr, &ml, nullptr, mask);
            if (mn < bestVal) {
                bestVal = mn;
                bestLoc = ml;
                bestScl = scl;
                bestW = ts.cols;
                bestH = ts.rows;
                bestResult = res;
            }
        }
        if (!(locked && pass == 0 && bestVal > kTmpl.scaleResweepThresh)) break;  // locked scale OK, or full sweep done
        bestVal = 2.0;
        bestResult.release();  // stale lock -> full re-sweep
    }
    r.quality = bestVal;  // SQDIFF: lower = better

    const int s = (strength >= kTmpl.strengthLo && strength <= kTmpl.strengthHi)
                      ? strength
                      : kTmpl.strengthDefault;
    if (bestResult.empty() || bestVal > kTmpl.acceptBase - kTmpl.acceptPerStrength * s) return r;
    if (ioScale) *ioScale = bestScl;  // remember the working scale

    // Parabolic sub-pixel refine on the best-scale SQDIFF surface (shape-agnostic).
    const cv::Point ml = bestLoc;
    double dx = 0.0, dy = 0.0;
    if (ml.x > 0 && ml.x < bestResult.cols - 1) {
        const double a = bestResult.at<float>(ml.y, ml.x - 1), b = bestResult.at<float>(ml.y, ml.x),
                     c = bestResult.at<float>(ml.y, ml.x + 1), den = a - 2 * b + c;
        if (std::fabs(den) > 1e-6) dx = 0.5 * (a - c) / den;
    }
    if (ml.y > 0 && ml.y < bestResult.rows - 1) {
        const double a = bestResult.at<float>(ml.y - 1, ml.x), b = bestResult.at<float>(ml.y, ml.x),
                     c = bestResult.at<float>(ml.y + 1, ml.x), den = a - 2 * b + c;
        if (std::fabs(den) > 1e-6) dy = 0.5 * (a - c) / den;
    }

    r.found = true;
    r.cx = ml.x + dx + bestW / 2.0 + crop.x;  // full-frame x
    r.cy = ml.y + dy + bestH / 2.0 + crop.y;  // field-row y (caller adds +-0.5)
    r.radius = std::max(bestW, bestH) / 2.0;
    return r;
}

// ImageTemplate (down-vision) matcher — FIELD-AWARE. Shape-agnostic (squares,
// crosses, text — not just round marks), so the center comes from the template
// match itself, never a circle fit. CV_TM_SQDIFF_NORMED locks the specular dome
// where CCOEFF / gradient / CLAHE / masked variants latch onto bright solder
// blobs. 5x5 blur, NO brightness-normalization (it saturated bright pixels and
// shifted the SQDIFF peak). strength (1-10) sets the accept bar.
//
// Field handling: like detect_circle_mark, runs through detect_with_fields —
// deinterlace into even/odd fields, match EACH, and combine (settled: average;
// moving: newest field, BFF). Matching the woven frame combs under head motion.
// The template is prepped once (the per-field cost is just blur + match).
// NOTE: template-matching a round specular dome is inherently ~10px imprecise
// (the highlight moves) — round marks belong on Circular mode; this path is for
// arbitrary template shapes. NON-DESTRUCTIVE.
MarkResult detect_template_mark(const void* frame, const unsigned char* templateBytes,
                                int size, double /*threshold*/, int strength,
                                double refX, double refY, int searchRadiusPx,
                                double* ioScale) {
    // No template yet: still parse the header / hash via detect_with_fields so the
    // frame is logged, but report not-found.
    if (!templateBytes || size < 4)
        return detect_with_fields(frame, [](const cv::Mat&) { return MarkResult(); });

    // Prep the template ONCE (outside the per-field loop): plane0 (gray, aligned
    // stride), 180-deg flipped (the host stores it rotated 180° relative to our
    // frame — required for asymmetric marks), then pre-blurred to match the
    // per-field 5x5 smoothing.
    cv::Mat tb;
    try {
        const int step = (size + 3) & ~3;
        cv::Mat templ;
        cv::Mat(size, size, CV_8UC1, const_cast<unsigned char*>(templateBytes),
                static_cast<size_t>(step))
            .copyTo(templ);
        cv::flip(templ, templ, -1);
        cv::GaussianBlur(templ, tb, cv::Size(kTmpl.blurKernel, kTmpl.blurKernel), 0);
    } catch (...) {
        tb.release();  // malformed template -> fall through to the not-found path
    }
    // Prep failed (or produced nothing): still parse/log the frame, report not-found.
    if (tb.empty())
        return detect_with_fields(frame, [](const cv::Mat&) { return MarkResult(); });

    MarkResult r = detect_with_fields(frame, [&](const cv::Mat& g) {
        return detect_template_one_field(g, tb, size, strength, refX, refY,
                                         searchRadiusPx, ioScale);
    });
    r.shape = MarkShape::Square;  // ImageTemplate: the preview draws the matched square
    return r;
}

// Round (CheckMark algo==0): contour-circularity detector (NOT Hough -- Hough is
// banned in CheckMark mode). Feature size gated by a fixed 0.5-3.5mm bracket;
// searchRadiusPx (the operator's Range) is used as-is for the search-area gate.
// Woven-when-settled: an upscaled field's soft edges break the ring up, so when
// settled it runs on the sharp full-res frame. The preview draws a circle at the
// detected center/radius. NON-DESTRUCTIVE.
MarkResult detect_round_mark(const void* frame, double refX, double refY,
                             int searchRadiusPx, int strength) {
    MarkResult r = detect_with_fields(
        frame,
        [&](const cv::Mat& g) { return detect_one_field_contour(g, refX, refY, searchRadiusPx, strength); },
        /*wovenWhenSettled=*/true);
    r.shape = MarkShape::Circle;
    return r;
}

}  // namespace vis
