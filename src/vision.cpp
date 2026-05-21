#include "vision.h"

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

// Two deinterlaced fields whose detected centers are within this many pixels are
// treated as settled and averaged; otherwise only the newest field is reported.
constexpr double kSettleAgreePx = 4.0;

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
        cv::Mat out;
        if (origin == IPL_ORIGIN_BL)
            cv::flip(wrapped, out, 0);  // into a NEW Mat
        else
            out = wrapped.clone();  // copy; never touch live buffer
        return cv::imwrite(path, out);
    } catch (...) {
        return false;
    }
}

// Detect the copper mark in ONE already-deinterlaced, full-height grayscale
// field image. cx comes out in full-frame x; cy is in this image's row space and
// needs the caller's +-0.5 field correction. Returns found=false on no mark.
// This is the per-field core; detect_circle_mark runs it on both fields.
static MarkResult detect_one_field(const cv::Mat& gFull, int markSizePx,
                                   double refX, double refY,
                                   int searchRadiusPx) {
    MarkResult r;

    // Tight radius bracket locked to the inner 1mm copper; excludes the larger
    // solder-mask ring that otherwise causes flip-flop jitter. markSizePx is the
    // host "size" arg (algo=63 ~ 1.2mm). Computed first so it can size the ROI.
    int minRfull, maxRfull;
    if (markSizePx >= kCircle.markSizeLo && markSizePx <= kCircle.markSizeHi) {
        minRfull = std::max(kCircle.radiusMinPx,
                            static_cast<int>(markSizePx * kCircle.radiusMinFrac));
        maxRfull = static_cast<int>(markSizePx * kCircle.radiusMaxFrac);
    } else {
        minRfull = kCircle.radiusFallbackMin;
        maxRfull = kCircle.radiusFallbackMax;
    }

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
    double fx = circles[best][0], fy = circles[best][1], fr = circles[best][2];

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
// (circular, template, ...). NON-DESTRUCTIVE: copies out of the frame buffer.
static MarkResult detect_with_fields(
    const void* frame,
    const std::function<MarkResult(const cv::Mat&)>& perField) {
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

        // IplImage origin: 0 = top-left (top-down), 1 = bottom-left (bottom-up).
        cv::Mat img;
        if (ipl->origin == IPL_ORIGIN_BL)
            cv::flip(wrapped, img, 0);
        else
            img = wrapped;

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

        MarkResult re = perField(evenU);
        MarkResult ro = perField(oddU);
        if (re.found) re.cy -= 0.5;  // even field samples rows 2i   -> full-frame y
        if (ro.found) ro.cy += 0.5;  // odd  field samples rows 2i+1 -> full-frame y

        // The two fields are captured ~1/60 s apart. Combine them by motion state:
        //  - settled (fields agree): AVERAGE -> full vertical resolution, cancels
        //    the opposite +-0.5 row bias.
        //  - moving (fields disagree): report ONLY the most-recent field; the
        //    older field carries a stale head position that misdirects the host's
        //    correction. These cameras are bottom-field-first (BFF): the bottom
        //    field = odd rows (1,3,5) is captured FIRST (older); the top field =
        //    even rows (0,2,4) is the most recent. So under motion we report the
        //    EVEN field. (In normal operation this branch is rare -- the host only
        //    reads vision at a move endpoint, so frames are almost always settled.)
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
        for (double scl = a; scl <= b + 1e-9; scl += kTmpl.scaleStep) {
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

    return detect_with_fields(frame, [&](const cv::Mat& g) {
        return detect_template_one_field(g, tb, size, strength, refX, refY,
                                         searchRadiusPx, ioScale);
    });
}

}  // namespace vis
