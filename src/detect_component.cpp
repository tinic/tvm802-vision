#include "vision.h"

#include "iplframe.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

// Up-vision component detector (CheckComp). See vision.h for the contract and the
// rationale. Two cooperating paths:
//   1. RECTLINEAR SYMMETRY (primary, threshold-free) -- the angle is the rotated
//      orientation whose silhouette projects to the sharpest-edged pulse; the
//      center and body size come from the symmetric rising/falling edge pair of the
//      axis-aligned projections. Robust: a bent pin or glare perturbs the pulse but
//      cannot stretch the box the way an extremal minAreaRect fit does.
//   2. MINAREARECT FALLBACK -- thresholded silhouette -> largest central contour ->
//      cv::minAreaRect, when the symmetry confidence is low (genuinely asymmetric
//      parts). This is the vendor-equivalent baseline.
// OpenCV-only (no Windows headers) so the test harness can exercise it off-target.

namespace vis {
namespace {
using detail::as_valid_ipl;

struct CompParams {
    double meanLo = 3.0, meanHi = 252.0;  // dropped (black) / blown (white) frame
    int searchFallbackPx = 110;           // center-search radius when the host gives none
    int roiPad = 28;                      // ROI margin beyond search + expected size
    int gaussKernel = 5;                  // anti-noise pre-blur (native cvSmooth 5x5)
    int defaultThreshold = 50;            // native Comp Threshold default (0x32) for fallback
    int minPartPx = 12;                   // ignore features/contours smaller than this
    // Orientation search: a rectangle repeats every 90 deg, so search (-45, 45].
    double angleSpanDeg = 45.0;
    double angleCoarseStepDeg = 3.0;
    double angleFineStepDeg = 0.3;
    double minSymQuality = 0.30;    // accept the symmetry result above this; else fall back
    double minFillFallback = 0.45;  // minAreaRect rectangularity floor to call it found
};
constexpr CompParams kComp;

// Subpixel location of an extremum in v[] near integer index i (parabola through
// i-1, i, i+1). Returns i + delta, delta clamped to [-1, 1].
double parabolic(const std::vector<float>& v, int i) {
    const int n = static_cast<int>(v.size());
    if (i <= 0 || i + 1 >= n) {
        return static_cast<double>(i);
    }
    const auto a = static_cast<double>(v[static_cast<size_t>(i) - 1]);
    const auto b = static_cast<double>(v[static_cast<size_t>(i)]);
    const auto c = static_cast<double>(v[static_cast<size_t>(i) + 1]);
    const double den = a - 2.0 * b + c;
    if (std::abs(den) < 1e-9) {
        return static_cast<double>(i);
    }
    double d = 0.5 * (a - c) / den;
    if (d > 1.0) {
        d = 1.0;
    } else if (d < -1.0) {
        d = -1.0;
    }
    return static_cast<double>(i) + d;
}

// Project a single-channel image onto one axis (column-averaged), then subtract the
// minimum so a bright part reads as a positive pulse over ~0 background. dim per
// cv::reduce: 0 -> average rows (project onto X, length = cols); 1 -> average cols
// (project onto Y, length = rows). REDUCE_AVG keeps the scale independent of extent.
void projection(const cv::Mat& g, int dim, std::vector<float>& out) {
    cv::Mat p;
    cv::reduce(g, p, dim, cv::REDUCE_AVG, CV_32F);
    if (dim == 1) {
        p = p.reshape(1, 1);  // column vector -> row vector, uniform handling
    }
    const int n = p.cols;
    out.assign(static_cast<size_t>(n), 0.0f);
    float lo = p.at<float>(0, 0);
    for (int i = 0; i < n; ++i) {
        lo = std::min(lo, p.at<float>(0, i));
    }
    for (int i = 0; i < n; ++i) {
        out[static_cast<size_t>(i)] = p.at<float>(0, i) - lo;
    }
}

// Sum of squared central differences of a projection -- the silhouette "edge
// energy". A part whose sides are aligned with the projection axis yields steep
// edges (high energy); an off-axis part smears them (low). Used to score angle.
double edge_energy(const std::vector<float>& proj) {
    const int n = static_cast<int>(proj.size());
    double e = 0.0;
    for (int i = 1; i < n - 1; ++i) {
        const double d = 0.5 * (static_cast<double>(proj[static_cast<size_t>(i) + 1]) -
                                static_cast<double>(proj[static_cast<size_t>(i) - 1]));
        e += d * d;
    }
    return e;
}

struct AxisFit {
    bool ok = false;
    double center = 0.0;  // subpixel center along the axis
    double size = 0.0;    // subpixel extent (edge-pair distance)
    double sym = 0.0;     // mirror-symmetry quality of the pulse about center, [0,1]
};

// Locate the part along one projection: the strongest rising edge (max +gradient)
// and strongest falling edge (max -gradient), their subpixel positions, the center
// and width between them, and a normalized mirror-symmetry quality of the pulse.
AxisFit fit_axis(const std::vector<float>& proj, int minSizePx) {
    AxisFit f;
    const int n = static_cast<int>(proj.size());
    if (n < 2 * minSizePx + 3) {
        return f;
    }

    std::vector<float> d(static_cast<size_t>(n), 0.0f);
    for (int i = 1; i < n - 1; ++i) {
        d[static_cast<size_t>(i)] = 0.5f * (proj[static_cast<size_t>(i) + 1] -
                                            proj[static_cast<size_t>(i) - 1]);
    }

    int li = 1;
    int ri = n - 2;
    float dmax = -1e30f;
    float dmin = 1e30f;
    for (int i = 1; i < n - 1; ++i) {
        if (d[static_cast<size_t>(i)] > dmax) {
            dmax = d[static_cast<size_t>(i)];
            li = i;
        }
        if (d[static_cast<size_t>(i)] < dmin) {
            dmin = d[static_cast<size_t>(i)];
            ri = i;
        }
    }
    if (ri <= li) {
        return f;  // a real pulse rises (left) before it falls (right)
    }

    std::vector<float> dabs(static_cast<size_t>(n), 0.0f);
    for (int i = 0; i < n; ++i) {
        dabs[static_cast<size_t>(i)] = std::abs(d[static_cast<size_t>(i)]);
    }
    const double lpos = parabolic(dabs, li);
    const double rpos = parabolic(dabs, ri);
    const double size = rpos - lpos;
    if (size < static_cast<double>(minSizePx)) {
        return f;
    }

    const double center = 0.5 * (lpos + rpos);
    const int c = static_cast<int>(std::lround(center));
    const int half = static_cast<int>(size * 0.5);
    double sse = 0.0;
    double ssr = 0.0;
    for (int t = 1; t <= half; ++t) {
        const int a = c - t;
        const int b = c + t;
        if (a < 0 || b >= n) {
            break;
        }
        const auto pa = static_cast<double>(proj[static_cast<size_t>(a)]);
        const auto pb = static_cast<double>(proj[static_cast<size_t>(b)]);
        const double diff = pa - pb;
        const double sum = pa + pb;
        sse += diff * diff;
        ssr += sum * sum;
    }
    f.sym = (ssr > 1e-9) ? std::max(0.0, 1.0 - sse / ssr) : 0.0;
    f.center = center;
    f.size = size;
    f.ok = true;
    return f;
}

// Rotate the ROI by `deg` (CCW) about its center; out is the axis-aligned view at
// that trial angle. M is returned so the found center can be mapped back to the ROI.
void rotate_roi(const cv::Mat& roi, double deg, cv::Mat& out, cv::Mat& M) {
    const cv::Point2f c(static_cast<float>(roi.cols) * 0.5f, static_cast<float>(roi.rows) * 0.5f);
    M = cv::getRotationMatrix2D(c, deg, 1.0);
    cv::warpAffine(roi, out, M, roi.size(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
}

// Best orientation: the trial angle whose axis-aligned projections carry the most
// silhouette edge energy. Coarse sweep over (-span, span], then a fine refine.
double best_angle(const cv::Mat& roi) {
    cv::Mat rot;
    cv::Mat M;
    std::vector<float> px;
    std::vector<float> py;
    auto score_at = [&](double deg) {
        rotate_roi(roi, deg, rot, M);
        projection(rot, 0, px);
        projection(rot, 1, py);
        return edge_energy(px) + edge_energy(py);
    };

    // Integer-indexed sweeps (no floating-point loop counters): coarse over the full
    // (-span, span], then fine around the coarse winner.
    double bestDeg = 0.0;
    double bestScore = -1.0;
    const int nCoarse = static_cast<int>(std::lround(2.0 * kComp.angleSpanDeg / kComp.angleCoarseStepDeg));
    for (int i = 0; i <= nCoarse; ++i) {
        const double deg = -kComp.angleSpanDeg + static_cast<double>(i) * kComp.angleCoarseStepDeg;
        const double s = score_at(deg);
        if (s > bestScore) {
            bestScore = s;
            bestDeg = deg;
        }
    }
    double refineBest = bestDeg;
    double refineScore = bestScore;
    const int nFine = static_cast<int>(std::lround(2.0 * kComp.angleCoarseStepDeg / kComp.angleFineStepDeg));
    for (int i = 0; i <= nFine; ++i) {
        const double deg = bestDeg - kComp.angleCoarseStepDeg + static_cast<double>(i) * kComp.angleFineStepDeg;
        const double s = score_at(deg);
        if (s > refineScore) {
            refineScore = s;
            refineBest = deg;
        }
    }
    return refineBest;
}

// Bring angle into (-45, 45], swapping w/h across each 90-deg boundary (standard
// rotated-rect normalization; matches the native negate+swap-90 behavior).
void normalize_pose(double& w, double& h, double& angle) {
    while (angle > 45.0) {
        angle -= 90.0;
        std::swap(w, h);
    }
    while (angle <= -45.0) {
        angle += 90.0;
        std::swap(w, h);
    }
}

// If a non-square size prior is available, choose between the as-is labeling and the
// +90/swap labeling whichever (w,h) better matches (expW,expH). Resolves the
// 90-deg W<->H ambiguity using the host's expected geometry.
void disambiguate_with_prior(double& w, double& h, double& angle,
                             double expW, double expH) {
    if (expW <= 0.0 || expH <= 0.0) {
        return;
    }
    const double err0 = std::abs(w - expW) + std::abs(h - expH);
    const double err1 = std::abs(h - expW) + std::abs(w - expH);  // after a swap
    if (err1 + 1e-6 < err0) {
        std::swap(w, h);
        angle += (angle <= 0.0) ? 90.0 : -90.0;
        normalize_pose(w, h, angle);
    }
}

// Thresholded-silhouette minAreaRect fallback (vendor-equivalent baseline). Returns
// found=false if no plausible central contour is rectangular enough.
CompResult minarea_fallback(const cv::Mat& g, const cv::Rect& crop,
                            double refXroi, double refYroi, int threshold) {
    CompResult r;
    cv::Mat bin;
    cv::threshold(g, bin, static_cast<double>(threshold), 255.0, cv::THRESH_BINARY);
    const cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(bin, bin, cv::MORPH_OPEN, k);
    cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, k);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        return r;
    }

    // Prefer the largest contour whose centroid is near the reference (the part sits
    // over the nozzle, ~centered); reject specks.
    int best = -1;
    double bestScore = -1.0;
    const auto minArea = static_cast<double>(kComp.minPartPx * kComp.minPartPx);
    for (size_t i = 0; i < contours.size(); ++i) {
        const double area = cv::contourArea(contours[i]);
        if (area < minArea) {
            continue;
        }
        const cv::Moments m = cv::moments(contours[i]);
        if (m.m00 <= 0.0) {
            continue;
        }
        const double cxr = m.m10 / m.m00;
        const double cyr = m.m01 / m.m00;
        const double dx = cxr - refXroi;
        const double dy = cyr - refYroi;
        const double score = area / (1.0 + std::sqrt(dx * dx + dy * dy));
        if (score > bestScore) {
            bestScore = score;
            best = static_cast<int>(i);
        }
    }
    if (best < 0) {
        return r;
    }

    const cv::RotatedRect rr = cv::minAreaRect(contours[static_cast<size_t>(best)]);
    double w = rr.size.width;
    double h = rr.size.height;
    double angle = rr.angle;
    const double boxArea = w * h;
    const double fill = (boxArea > 1.0) ? cv::contourArea(contours[static_cast<size_t>(best)]) / boxArea : 0.0;
    if (fill < kComp.minFillFallback) {
        return r;
    }

    normalize_pose(w, h, angle);
    r.found = true;
    r.cx = static_cast<double>(rr.center.x) + crop.x;
    r.cy = static_cast<double>(rr.center.y) + crop.y;
    r.w = w;
    r.h = h;
    r.angle = angle;
    r.quality = fill;  // rectangularity as the fallback's confidence
    r.method = CompResult::Method::MinAreaRect;
    return r;
}

}  // namespace

CompResult detect_component(const void* frame,
                            double expectedWpx, double expectedHpx,
                            double expectedAngleDeg,
                            int threshold,
                            double refX, double refY,
                            int searchRadiusPx) {
    CompResult r;
    const IplImage* ipl = as_valid_ipl(frame);
    if (ipl == nullptr) {
        return r;
    }
    r.headerOk = true;
    r.imgW = ipl->width;
    r.imgH = ipl->height;
    r.imgOrigin = ipl->origin;
    r.frameHash = frame_hash(frame);

    const int thr = (threshold > 0) ? threshold : kComp.defaultThreshold;

    // EXCEPTION BARRIER: called across a plain C ABI from the host. Never let an
    // OpenCV/STL exception unwind past this boundary (UB) -- report not-found.
    try {
        const cv::Mat wrapped(ipl->height, ipl->width, CV_MAKETYPE(CV_8U, ipl->nChannels),
                              ipl->imageData, static_cast<size_t>(ipl->widthStep));
        cv::Mat gray;
        if (wrapped.channels() == 3) {
            cv::extractChannel(wrapped, gray, 0);  // monochrome capture: plane 0 == gray
        } else {
            gray = wrapped;
        }

        const double rxF = (refX >= 0.0) ? refX : gray.cols / 2.0;
        const double ryF = (refY >= 0.0) ? refY : gray.rows / 2.0;
        const int sr = (searchRadiusPx > 0) ? searchRadiusPx : kComp.searchFallbackPx;

        // ROI around the reference: search radius + half the expected body + pad.
        const double halfBody = 0.5 * std::max({expectedWpx, expectedHpx, 0.0});
        const int rad = sr + static_cast<int>(std::ceil(halfBody)) + kComp.roiPad;
        const cv::Rect crop = cv::Rect(cvRound(rxF) - rad, cvRound(ryF) - rad, 2 * rad, 2 * rad) &
                              cv::Rect(0, 0, gray.cols, gray.rows);
        if (crop.width < 2 * kComp.minPartPx || crop.height < 2 * kComp.minPartPx) {
            return r;
        }

        const cv::Mat sub = gray(crop);
        const double meanv = cv::mean(sub)[0];
        if (meanv < kComp.meanLo || meanv > kComp.meanHi) {
            return r;  // dropped / blown
        }

        cv::Mat g;
        cv::GaussianBlur(sub, g, cv::Size(kComp.gaussKernel, kComp.gaussKernel), 1.0);

        // --- Primary: rectlinear symmetry ---
        const double aDeg = best_angle(g);
        cv::Mat rot;
        cv::Mat M;
        rotate_roi(g, aDeg, rot, M);
        std::vector<float> px;
        std::vector<float> py;
        projection(rot, 0, px);
        projection(rot, 1, py);
        const AxisFit fx = fit_axis(px, kComp.minPartPx);
        const AxisFit fy = fit_axis(py, kComp.minPartPx);

        if (fx.ok && fy.ok) {
            // Center is in the rotated frame; map back through the inverse rotation.
            cv::Mat Minv;
            cv::invertAffineTransform(M, Minv);
            const double xr = fx.center;
            const double yr = fy.center;
            const double xo = Minv.at<double>(0, 0) * xr + Minv.at<double>(0, 1) * yr + Minv.at<double>(0, 2);
            const double yo = Minv.at<double>(1, 0) * xr + Minv.at<double>(1, 1) * yr + Minv.at<double>(1, 2);

            double w = fx.size;
            double h = fy.size;
            // The trial rotation that axis-aligns the part equals the part's angle in
            // OpenCV's cv::RotatedRect/minAreaRect convention (y-down image coords),
            // so both detector paths report angle the same way. The ABSOLUTE sign the
            // host expects is a separate calibration against the shadow log -- applied
            // in the shim's comp packing, not here.
            double angle = aDeg;
            normalize_pose(w, h, angle);
            disambiguate_with_prior(w, h, angle, expectedWpx, expectedHpx);

            r.found = true;
            r.cx = xo + crop.x;
            r.cy = yo + crop.y;
            r.w = w;
            r.h = h;
            r.angle = angle;
            r.quality = std::min(fx.sym, fy.sym);  // weakest axis governs confidence
            r.method = CompResult::Method::Symmetry;
            (void)expectedAngleDeg;  // reserved: narrow the angle search once data exists
            if (r.quality >= kComp.minSymQuality) {
                return r;
            }
        }

        // --- Fallback: thresholded-silhouette minAreaRect ---
        const CompResult fb = minarea_fallback(g, crop, rxF - crop.x, ryF - crop.y, thr);
        if (fb.found) {
            CompResult out = fb;
            out.imgW = r.imgW;
            out.imgH = r.imgH;
            out.imgOrigin = r.imgOrigin;
            out.headerOk = r.headerOk;
            out.frameHash = r.frameHash;
            disambiguate_with_prior(out.w, out.h, out.angle, expectedWpx, expectedHpx);
            return out;
        }
        // Neither path confident: return the low-confidence symmetry result if we had
        // one (better than nothing for offline inspection), else not-found.
        return r;
    } catch (...) {
        r.found = false;
        return r;
    }
}

}  // namespace vis
