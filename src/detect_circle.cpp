#include "vision.h"

#include "detect_common.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace vis {
namespace {

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

}  // namespace

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

MarkResult detect_circle_mark(const void* frame, int markSizePx,
                              double refX, double refY, int searchRadiusPx) {
    return detect_with_fields(frame, [&](const cv::Mat& g) {
        return detect_one_field(g, markSizePx, refX, refY, searchRadiusPx);
    });
}

}  // namespace vis
