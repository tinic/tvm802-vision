#include "vision.h"

#include "detect_common.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace vis {
namespace {

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

}  // namespace

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
        const cv::Rect c = cv::Rect(cvRound(rxF) - R, cvRound(ryF) - R, 2 * R, 2 * R) & cv::Rect(0, 0, gFull.cols, gFull.rows);
        if (c.width > kContour.roiMinDimPx && c.height > kContour.roiMinDimPx) {
            crop = c;
        }
    }
    const cv::Mat g = gFull(crop);
    const double cxRef = rxF - crop.x;
    const double cyRef = ryF - crop.y;

    const double imgMean = cv::mean(g)[0];
    if (imgMean < kContour.meanLo || imgMean > kContour.meanHi) {
        return r;  // dropped/blown
    }

    cv::Mat sm;
    cv::Mat edges;
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
    if (contours.empty()) {
        return r;
    }

    const double minArea = CV_PI * kContour.minRadiusPx * kContour.minRadiusPx;
    int best = -1;
    double bestQ = -1.0;
    double bestD = 1e18;
    double fr = 0.0;
    for (size_t i = 0; i < contours.size(); ++i) {
        const std::vector<cv::Point>& c = contours[i];
        const double area = std::abs(cv::contourArea(c));
        if (area < minArea) {
            continue;
        }
        cv::Point2f ec;
        float encR = 0.0f;
        cv::minEnclosingCircle(c, ec, encR);
        if (encR < static_cast<float>(kContour.minRadiusPx) || encR > static_cast<float>(kContour.maxRadiusPx)) {
            continue;
        }
        const double pr2 = CV_PI * encR * encR;
        if (std::abs(pr2 - area) >= pr2 * kContour.circTol) {
            continue;  // circularity
        }
        const double dx = ec.x - cxRef;
        const double dy = ec.y - cyRef;
        const double dist2 = std::fma(dx, dx, dy * dy);
        const auto srd = static_cast<double>(searchRadiusPx);
        if (searchRadiusPx > 0 && dist2 > srd * srd) {
            continue;  // search-area gate
        }
        const double q = area / pr2;  // ~1 = perfect circle
        if (q > bestQ + kContour.qTieBand || (q > bestQ - kContour.qTieBand && dist2 < bestD)) {
            bestQ = q;
            bestD = dist2;
            best = static_cast<int>(i);
            fr = encR;
        }
    }
    if (best < 0) {
        return r;  // nothing circular passed
    }

    const std::vector<cv::Point>& win = contours[static_cast<size_t>(best)];
    const cv::Moments mm = cv::moments(win);
    if (mm.m00 <= 0.0) {
        return r;
    }

    r.found = true;
    r.cx = mm.m10 / mm.m00 + crop.x;
    r.cy = mm.m01 / mm.m00 + crop.y;
    r.radius = fr;
    r.quality = std::min(1.0, bestQ);
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
