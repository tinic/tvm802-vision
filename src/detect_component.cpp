#include "vision.h"

#include "detect_common.h"
#include "iplframe.h"
#include "settings.h"
#include "thread_pool.h"

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
    double meanLo = 0.5, meanHi = 254.5;  // catches truly dropped/blown frames only
    int searchFallbackPx = 110;           // center-search radius when the host gives none
    int roiPad = 28;                      // ROI margin beyond search + expected size
    int gaussKernel = 3;                  // light pre-blur (5x5 smears 0402 short axis)
    int defaultThreshold = 50;            // native Comp Threshold default (0x32) for fallback
    int minPartPx = 6;                    // 0402 short axis is ~20 px on the up-cam
    // Orientation search: a rectangle repeats every 90 deg, so search (-45, 45].
    double angleSpanDeg = 45.0;
    double angleCoarseStepDeg = 3.0;
    double angleFineStepDeg = 0.3;
    double minSymQuality = 0.30;    // accept the symmetry result above this; else fall back
    double minFillFallback = 0.45;  // minAreaRect rectangularity floor to call it found
    double fillCap = 0.92;          // reject a box that spans >=this fraction of the ROI
};
constexpr CompParams kComp;

// CompThre profile slots. The operator-typed 视觉阈值 column accepts 0-100;
// the upper 90% of that range (10-100) is the existing manual %-of-max-
// brightness threshold. Values 0-9 are RESERVED for "special profiles" --
// detector presets we'll fill in per-board as real PCBs demand custom
// tuning (chip / LED / SOT / IC families, etc.). Today all 1-9 slots are
// EMPTY (name == nullptr) and fall through to the AUTO path -- identical
// to threshold == 0 -- so reserving them is invisible until a profile is
// actually written. 0 is the AUTO sentinel.
struct CompProfile {
    int id;            // 0..9
    const char* name;  // nullptr if slot empty (reserved-but-unfilled)
    // FUTURE FIELDS (none yet -- filled in as profiles are authored):
    //   int  thresholdPct;     // override the %-of-max threshold
    //   int  blurKernel;       // override the pre-blur kernel (odd, 3..9)
    //   bool maskNozzle;       // mask the known nozzle circle out of the ROI
    //   bool fallbackOnly;     // skip symmetry, use minAreaRect path only
};
constexpr std::array<CompProfile, 10> kProfiles = {{
    {.id = 0, .name = "AUTO"},
    {.id = 1, .name = nullptr},
    {.id = 2, .name = nullptr},
    {.id = 3, .name = nullptr},
    {.id = 4, .name = nullptr},
    {.id = 5, .name = nullptr},
    {.id = 6, .name = nullptr},
    {.id = 7, .name = nullptr},
    {.id = 8, .name = nullptr},
    {.id = 9, .name = nullptr},
}};

// Subpixel location of an extremum at integer index i, given three sample values
// at i-1, i, i+1. Returns i + delta with delta clamped to [-1, 1]. Three-value
// form (vs taking a vector + index) so callers can supply the values directly
// without materialising an intermediate buffer (see fit_axis below).
double parabolic3(double a, double b, double c, int i) {
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
    out.resize(static_cast<size_t>(n));  // no-init resize (vs assign which zeros first)
    // Single base pointer + raw indexing: cv::Mat::at<float>() has per-access
    // overhead the optimizer can't elide; the row is guaranteed contiguous after
    // the reshape above so the float* hop is sound.
    const float* src = p.ptr<float>(0);
    float lo = src[0];
    for (int i = 1; i < n; ++i) {
        lo = std::min(lo, src[i]);
    }
    float* dst = out.data();
    for (int i = 0; i < n; ++i) {
        dst[i] = src[i] - lo;
    }
}

// Sum of squared central differences of a projection -- the silhouette "edge
// energy". A part whose sides are aligned with the projection axis yields steep
// edges (high energy); an off-axis part smears them (low). Used to score angle.
double edge_energy(const std::vector<float>& proj) {
    const int n = static_cast<int>(proj.size());
    const float* p = proj.data();
    double e = 0.0;
    for (int i = 1; i < n - 1; ++i) {
        const double d = 0.5 * (static_cast<double>(p[i + 1]) - static_cast<double>(p[i - 1]));
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

    // Central-difference derivative. thread_local so the buffer is reused across
    // calls (this function fires ~100 times per detect_component); plain
    // resize-without-shrink avoids the per-call heap allocation.
    static thread_local std::vector<float> d;
    d.resize(static_cast<size_t>(n));
    const float* psrc = proj.data();
    float* dptr = d.data();
    dptr[0] = 0.0f;
    dptr[n - 1] = 0.0f;
    float dmax = -1e30f;
    float dmin = 1e30f;
    int li = 1;
    int ri = n - 2;
    for (int i = 1; i < n - 1; ++i) {
        const float di = 0.5f * (psrc[i + 1] - psrc[i - 1]);
        dptr[i] = di;
        if (di > dmax) {
            dmax = di;
            li = i;
        }
        if (di < dmin) {
            dmin = di;
            ri = i;
        }
    }
    if (ri <= li) {
        return f;  // a real pulse rises (left) before it falls (right)
    }

    // Sub-pixel refine via parabolic interpolation around li / ri. We only need
    // |d| at 6 indices (li-1, li, li+1 and the same around ri); fetch on the fly
    // instead of materialising a full |d| buffer over n samples.
    const double lpos =
        parabolic3(std::abs(dptr[li - 1]), std::abs(dptr[li]), std::abs(dptr[li + 1]), li);
    const double rpos =
        parabolic3(std::abs(dptr[ri - 1]), std::abs(dptr[ri]), std::abs(dptr[ri + 1]), ri);
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
//
// Writes the 2x3 rotation matrix coefficients in-place into M instead of going
// through cv::getRotationMatrix2D, which allocates a fresh 2x3 Mat per call.
// Bit-identical to OpenCV's implementation: alpha = cos(angle), beta = sin(angle)
// after angle * PI / 180 -- same constants, same order of ops, same float math.
void rotate_roi(const cv::Mat& roi, double deg, cv::Mat& out, cv::Mat& M) {
    if (M.empty() || M.rows != 2 || M.cols != 3 || M.type() != CV_64FC1) {
        M.create(2, 3, CV_64FC1);
    }
    const double rad = deg * CV_PI / 180.0;
    const double a = std::cos(rad);
    const double b = std::sin(rad);
    const double cx = static_cast<double>(roi.cols) * 0.5;
    const double cy = static_cast<double>(roi.rows) * 0.5;
    auto* m = M.ptr<double>();
    m[0] = a;
    m[1] = b;
    m[2] = (1.0 - a) * cx - b * cy;
    m[3] = -b;
    m[4] = a;
    m[5] = b * cx + (1.0 - a) * cy;
    cv::warpAffine(roi, out, M, roi.size(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
}

// Best orientation: the trial angle whose axis-aligned projections carry the most
// silhouette edge energy. Coarse sweep over a range, then a fine refine.
//
// When the host passes a meaningful expected angle (via SetCompAngle / SetCompSizeWHA)
// we narrow the coarse sweep to prior +/- kPriorHalfSpan instead of the full
// (-span, span]. The placement loop's per-part CSV always carries the expected
// orientation, so the prior is reliable in practice. If the narrow sweep's best
// lands ON the boundary (i.e. truth is outside the window -- e.g. operator put a
// part in upside-down), we expand to the full sweep so accuracy is preserved.
double best_angle(const cv::Mat& roi, double priorDeg = std::numeric_limits<double>::quiet_NaN()) {
    // Per-trial scoring helper. rot/M/px/py are thread_local so each worker
    // thread in the pool keeps its own buffers across the parallel coarse
    // sweep AND the calling thread reuses them in the serial fine refine.
    // No per-trial allocation, no shared mutable state.
    auto score_at = [&](double deg) {
        static thread_local cv::Mat rot;
        static thread_local cv::Mat M;
        static thread_local std::vector<float> px;
        static thread_local std::vector<float> py;
        rotate_roi(roi, deg, rot, M);
        projection(rot, 0, px);
        projection(rot, 1, py);
        return edge_energy(px) + edge_energy(py);
    };

    // Choose coarse sweep range. With a finite prior, narrow to prior +/- half-span;
    // otherwise full (-span, span]. Half-span generously covers expected drift
    // (parts in tape are at fixed orientation, drift due to pickup is ~few degrees).
    constexpr double kPriorHalfSpan = 15.0;
    double sweepLo = -kComp.angleSpanDeg;
    double sweepHi = kComp.angleSpanDeg;
    bool narrowed = false;
    if (std::isfinite(priorDeg)) {
        double p = priorDeg;
        while (p > 45.0) {
            p -= 90.0;
        }
        while (p <= -45.0) {
            p += 90.0;
        }
        sweepLo = p - kPriorHalfSpan;
        sweepHi = p + kPriorHalfSpan;
        narrowed = true;
    }
    // (else: sweepLo/Hi already initialised to the full (-span, span] above)

    // Coarse sweep -- parallel across the persistent pool (<=4 cores). Each
    // worker writes its OWN trial's score; the serial reduction below picks
    // the max in trial-index order with the same strict-> tie-break as the
    // old serial loop. BIT-IDENTICAL to serial. (Same pattern as
    // detect_circular_symmetry's coarse grid; see that file.)
    const int nCoarse =
        static_cast<int>(std::lround((sweepHi - sweepLo) / kComp.angleCoarseStepDeg));
    std::vector<double> scores(static_cast<size_t>(nCoarse) + 1);
    parallel_for(nCoarse + 1, [&](int i) {
        const double deg = sweepLo + static_cast<double>(i) * kComp.angleCoarseStepDeg;
        scores[static_cast<size_t>(i)] = score_at(deg);
    });
    double bestDeg = 0.0;
    double bestScore = -1.0;
    int bestI = -1;
    for (int i = 0; i <= nCoarse; ++i) {
        const double s = scores[static_cast<size_t>(i)];
        if (s > bestScore) {
            bestScore = s;
            bestDeg = sweepLo + static_cast<double>(i) * kComp.angleCoarseStepDeg;
            bestI = i;
        }
    }
    // Boundary check: if the narrow sweep's best is at i=0 or i=nCoarse, the true
    // best may lie outside the window. Expand to the full sweep and rescore
    // everything (the savings disappear in this rare edge case but accuracy is
    // preserved -- the prior was wrong). Parallel + same reduction shape.
    if (narrowed && (bestI == 0 || bestI == nCoarse)) {
        const int nCoarseFull = static_cast<int>(
            std::lround(2.0 * kComp.angleSpanDeg / kComp.angleCoarseStepDeg));
        std::vector<double> scoresFull(static_cast<size_t>(nCoarseFull) + 1);
        parallel_for(nCoarseFull + 1, [&](int i) {
            const double deg =
                -kComp.angleSpanDeg + static_cast<double>(i) * kComp.angleCoarseStepDeg;
            scoresFull[static_cast<size_t>(i)] = score_at(deg);
        });
        bestScore = -1.0;
        for (int i = 0; i <= nCoarseFull; ++i) {
            const double s = scoresFull[static_cast<size_t>(i)];
            if (s > bestScore) {
                bestScore = s;
                bestDeg =
                    -kComp.angleSpanDeg + static_cast<double>(i) * kComp.angleCoarseStepDeg;
            }
        }
    }
    // Fine refine: +/- one coarse step around the winner. Kept serial -- only
    // ~21 trials, all on the calling thread, so dispatch overhead would dwarf
    // the work. The thread_local buffers above carry across from coarse.
    double refineBest = bestDeg;
    double refineScore = bestScore;
    const int nFine =
        static_cast<int>(std::lround(2.0 * kComp.angleCoarseStepDeg / kComp.angleFineStepDeg));
    for (int i = 0; i <= nFine; ++i) {
        const double deg =
            bestDeg - kComp.angleCoarseStepDeg + static_cast<double>(i) * kComp.angleFineStepDeg;
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
// 90-deg W<->H ambiguity using the host's expected geometry. No-prior reports
// the rectangle exactly as the detector measured it -- mvision's job is to
// REPORT the pose, not impose a long-axis convention; the host is the source
// of truth for which dimension is "W" vs "H".
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
    // CompThre as PERCENTAGE of the ROI's max brightness (0-100). The operator
    // is the only person who knows how their part presents on the up-cam
    // (chip cap with dim body between bright terminals, SOT pins, LED with
    // halo, etc.); CompThre is their knob. Lower = more permissive (catches
    // dim features); higher = stricter (rejects halos / glare). Default 30
    // when unset.
    double maxv = 0.0;
    cv::minMaxLoc(g, nullptr, &maxv);
    if (maxv < 8.0) {
        return r;
    }
    // 10-100 = manual %-of-max. 0-9 = profile slot (all empty today; use
    // the default %). See detect_component() for the full routing.
    const double pct = (threshold >= 10 && threshold <= 100) ? threshold : 30.0;
    const double thrUse = (pct / 100.0) * maxv;
    cv::Mat bin;
    cv::threshold(g, bin, thrUse, 255.0, cv::THRESH_BINARY);
    const cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(bin, bin, cv::MORPH_OPEN, k);
    cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, k);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        return r;
    }

    // Union of every near-reference contour, fit a minAreaRect to the merged
    // point cloud. Handles parts where the body is INVISIBLE between
    // distinct bright features (a leaded IC like SOT-23 shows only its pin
    // tips against a dark body; a chip cap shows only the two metallic
    // terminals). Smaller per-contour minArea than kComp.minPartPx^2 so
    // individual pins / terminals (which are tiny) still pass.
    const auto dxLim = static_cast<double>(kComp.searchFallbackPx);
    const double minBlobArea = std::max(4.0,
                                        0.25 * static_cast<double>(kComp.minPartPx * kComp.minPartPx));
    std::vector<cv::Point> allPts;
    allPts.reserve(2048);
    double totalArea = 0.0;
    for (const auto& c : contours) {
        const double area = cv::contourArea(c);
        if (area < minBlobArea) {
            continue;
        }
        const cv::Moments m = cv::moments(c);
        if (m.m00 <= 0.0) {
            continue;
        }
        const double dx = m.m10 / m.m00 - refXroi;
        const double dy = m.m01 / m.m00 - refYroi;
        if (dx * dx + dy * dy > dxLim * dxLim) {
            continue;
        }
        allPts.insert(allPts.end(), c.begin(), c.end());
        totalArea += area;
    }
    if (allPts.size() < 6) {
        return r;
    }

    const cv::RotatedRect rr = cv::minAreaRect(allPts);
    double w = rr.size.width;
    double h = rr.size.height;
    double angle = rr.angle;
    const double boxArea = w * h;
    const double fill = (boxArea > 1.0) ? totalArea / boxArea : 0.0;

    normalize_pose(w, h, angle);
    r.found = true;
    r.cx = static_cast<double>(rr.center.x) + crop.x;
    r.cy = static_cast<double>(rr.center.y) + crop.y;
    r.w = w;
    r.h = h;
    r.angle = angle;
    r.quality = fill;
    r.method = CompResult::Method::MinAreaRect;
    return r;
}

// Stray-detection guard (purely geometric -- the configured part size never reaches
// the DLL, so we can't size-match; see notes). A real part rides the nozzle, so its
// center must lie within the search radius of the reference, and a box that spans
// almost the whole ROI is the detector latching the background/border rather than a
// discrete part. Rejects the off-nozzle distractors and frame-filling blobs the stock
// vision locks onto. Angle-agnostic, so it never interferes with the pick-angle /
// rotation handling.
bool plausible_part(double cx, double cy, double w, double h, double refXf, double refYf, int sr,
                    const cv::Rect& crop, double maxSizePx) {
    const double dx = cx - refXf;
    const double dy = cy - refYf;
    if (dx * dx + dy * dy > static_cast<double>(sr) * static_cast<double>(sr)) {
        return false;  // off the nozzle -> not the picked part
    }
    if (maxSizePx > 0.0) {
        if (w > maxSizePx || h > maxSizePx) {
            return false;  // exceeds the operator's configured max part size
        }
    } else if (w > kComp.fillCap * crop.width || h > kComp.fillCap * crop.height) {
        return false;  // box ~= ROI -> background/border latch, not a part
    }
    return true;
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

    // Resolve the operator-typed CompThre value into the detector's working
    // threshold. 0-9 are profile slots (today all empty -> AUTO defaults);
    // 10-100 are the manual %-of-max-brightness path.
    int thr = kComp.defaultThreshold;  // AUTO default
    if (threshold >= 10 && threshold <= 100) {
        thr = threshold;
    } else if (threshold >= 1 && threshold <= 9) {
        // Profile slot lookup. All slots empty today -> thr stays at the AUTO
        // default. A filled-in profile (future) will override thr (and other
        // detector params) here. The slot is still RESERVED so the operator's
        // typed value is recognized as "this is a profile, not a percentage".
        // (We deliberately do NOT treat low percentage values 1-9 as
        // thresholds -- they'd binarise far too aggressively. If an operator
        // really wants a 9% threshold they can type 10 instead.)
        const auto& p = kProfiles[static_cast<std::size_t>(threshold)];
        (void)p;  // no-op until a profile is authored
    }

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

        // Interlace handling: detect on the sharp woven frame when settled, but on the
        // newest single-instant field when the part is moving (e.g. rotating during
        // Accurate convergence) -- a combed frame otherwise corrupts the angle and edges.
        // cyCorrection maps a field-row cy back to full-frame y (added to the result).
        double cyCorrection = 0.0;
        gray = motion_aware_gray(gray, &cyCorrection);

        // Live tuning from the settings UI ("Component" mode). Every field is a no-op
        // at its neutral value, so an unconfigured machine runs exactly on the defaults.
        const Settings cfg = get_settings(MODE_COMP);
        const double minSym = (cfg.minSymmetry > 0.0) ? cfg.minSymmetry : kComp.minSymQuality;

        const double rxF = (refX >= 0.0) ? refX : gray.cols / 2.0;
        const double ryF = (refY >= 0.0) ? refY : gray.rows / 2.0;
        // Search radius (also the stray-guard center tolerance): UI override > host arg > default.
        int sr = kComp.searchFallbackPx;
        if (cfg.radiusMinPx > 0.0) {
            sr = static_cast<int>(cfg.radiusMinPx);
        } else if (searchRadiusPx > 0) {
            sr = searchRadiusPx;
        }

        // ROI around the reference: center tolerance (sr) + half the part body + pad.
        // The "Max part size" knob sizes the body so a large part (e.g. an LQFP MCU,
        // hundreds of px) is fully contained; else the host's expected-size prior (usually
        // absent). So sr stays a small center tolerance while big parts still fit the ROI.
        const double halfBody = (cfg.radiusMaxPx > 0.0)
                                    ? 0.5 * cfg.radiusMaxPx
                                    : 0.5 * std::max({expectedWpx, expectedHpx, 0.0});
        const int rad = sr + static_cast<int>(std::ceil(halfBody)) + kComp.roiPad;
        const cv::Rect crop = cv::Rect(cvRound(rxF) - rad, cvRound(ryF) - rad, 2 * rad, 2 * rad) &
                              cv::Rect(0, 0, gray.cols, gray.rows);
        if (crop.width < 2 * kComp.minPartPx || crop.height < 2 * kComp.minPartPx) {
            return r;
        }

        // OWN copy of the ROI: image adjustments modify in place, and `gray` may alias
        // the live frame buffer (channels==1), which must stay untouched.
        cv::Mat sub = gray(crop).clone();
        const double meanLo = (cfg.meanLo > 0.0) ? cfg.meanLo : kComp.meanLo;
        const double meanHi = (cfg.meanHi > 0.0) ? cfg.meanHi : kComp.meanHi;
        const double meanv = cv::mean(sub)[0];
        if (meanv < meanLo || meanv > meanHi) {
            return r;  // dropped / blown
        }
        apply_image_adjustments(sub, cfg);  // gamma/brightness/contrast/levels/sharpen

        cv::Mat g;
        const int gk = (cfg.blur > 0.0) ? blur_kernel(cfg.blur, kComp.gaussKernel) : kComp.gaussKernel;
        cv::GaussianBlur(sub, g, cv::Size(gk, gk), 1.0);

        // --- Primary: rectlinear symmetry ---
        // Pass the host's expected-angle prior (from SetCompAngle / SetCompSizeWHA)
        // to narrow the coarse search ~3x. NaN = no prior -> full sweep.
        const double aDeg = best_angle(g, expectedAngleDeg);
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
            r.cy = yo + crop.y + cyCorrection;
            r.w = w;
            r.h = h;
            r.angle = angle;
            r.quality = std::min(fx.sym, fy.sym);  // weakest axis governs confidence
            r.method = CompResult::Method::Symmetry;
            // (expectedAngleDeg is consumed by best_angle() above to narrow the coarse search)
            if (!plausible_part(r.cx, r.cy, r.w, r.h, rxF, ryF, sr, crop, cfg.radiusMaxPx)) {
                r.found = false;
            }
            (void)minSym;
        }

        // Always also run the union-of-contours fallback. CompThre (the
        // operator's per-stack 视觉阈值, % of ROI max) tunes which features
        // the binarization catches; the LARGER plausible-pose result wins.
        // The two paths are complementary -- symmetry locks cleanly on
        // single-bright-body parts (LEDs); union spans multi-feature parts
        // (chip caps, SOT pin layouts) that symmetry would only catch
        // a sub-feature of. No part-class branching in the code; the
        // operator's CompThre value steers which path's result dominates.
        const CompResult fb = minarea_fallback(g, crop, rxF - crop.x, ryF - crop.y, thr);
        const bool symOk = r.found;
        const bool fbOk = fb.found &&
                          plausible_part(fb.cx, fb.cy, fb.w, fb.h, rxF, ryF, sr, crop, cfg.radiusMaxPx);
        if (symOk && fbOk) {
            const double symArea = r.w * r.h;
            const double fbArea = fb.w * fb.h;
            // Fallback only wins when meaningfully larger -- a 2x area threshold
            // distinguishes "symmetry caught a sub-feature of a multi-feature
            // part" (SOT-23 single pin vs union of all pins) from "both paths
            // see the same body" (chip cap), avoiding frame-to-frame flip
            // wobble when sizes are similar.
            if (fbArea > 2.0 * symArea) {
                CompResult out = fb;
                out.cy += cyCorrection;
                out.imgW = r.imgW;
                out.imgH = r.imgH;
                out.imgOrigin = r.imgOrigin;
                out.headerOk = r.headerOk;
                out.frameHash = r.frameHash;
                disambiguate_with_prior(out.w, out.h, out.angle, expectedWpx, expectedHpx);
                return out;
            }
            return r;
        }
        if (fbOk) {
            CompResult out = fb;
            out.cy += cyCorrection;
            out.imgW = r.imgW;
            out.imgH = r.imgH;
            out.imgOrigin = r.imgOrigin;
            out.headerOk = r.headerOk;
            out.frameHash = r.frameHash;
            disambiguate_with_prior(out.w, out.h, out.angle, expectedWpx, expectedHpx);
            return out;
        }
        return r;
    } catch (...) {
        r.found = false;
        return r;
    }
}

// Profile-registry introspection (see vision.h). Returns the slot's name
// when authored, nullptr when the slot is empty/reserved, and nullptr for
// any out-of-range index. Callers (e.g. the settings UI's profile dropdown)
// use a nullptr return to render the slot as "(reserved)".
const char* comp_profile_name(int slot) {
    if (slot < 0 || slot >= static_cast<int>(kProfiles.size())) {
        return nullptr;
    }
    return kProfiles[static_cast<std::size_t>(slot)].name;
}

int comp_profile_count() {
    return static_cast<int>(kProfiles.size());
}

}  // namespace vis
