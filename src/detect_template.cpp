#include "vision.h"

#include "detect_common.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace vis {
namespace {

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

}  // namespace

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

}  // namespace vis
