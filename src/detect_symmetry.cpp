#include "detect_symmetry.h"

#include "detect_common.h"
#include "settings.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace vis {
namespace {

// Circular-symmetry tuning constants. The fiducial copper ring sits at r~19px on
// this rig; the bracket is generous (the score is scale-adaptive within it -- the
// center is robust even when both the copper and the larger solder-mask ring fall
// inside, since both are concentric about the SAME center). minSymmetry separates
// the real fiducial (score 2.6-4.2 measured) from glare/other-pad distractors
// (~1.2-1.7). Center search is coarse -> fine -> sub-pixel around the reference.
struct CsymParams {
    double meanLo = 6.0, meanHi = 250.0;  // dropped (black) / blown (white) frame
    int searchFallbackPx = 60;            // search radius when the host Range is unset
    int roiPad = 40;                      // ROI = search + maxR + pad around the reference
    int gaussKernel = 5;                  // anti-moire pre-blur
    // Physical ring bracket (RADIUS px) when the caller passes no diameter. ~0.4..1.7mm
    // dia at ~40 px/mm; covers all real fiducials. CheckMark2 passes a tight bracket.
    int minRadiusPx = 8, maxRadiusPx = 34;
    int ringStep = 1;          // radius sampling step
    double minSymmetry = 3.5;  // accept score (overallVar / area-weighted mean ring var);
                               // live data: real fiducial >=4.6, blown-frame false +ve 2.85
    int coarseStep = 4;        // center grid: coarse pass (fine pass refines +-fineSpan)
    int fineSpan = 3;          // +-span around the coarse winner at 1px (>= coarseStep/2)
    int coarseSampleStep = 2;  // ring/point subsample for the coarse pass (4x cheaper)
};
constexpr CsymParams kCsym;

// Bilinear sample from a raw row-major buffer. Caller guarantees 1 <= x < cols-1
// and 1 <= y < rows-1, so x,y are strictly positive -> a plain truncation equals
// floor (no std::floor: MSVC emits an x87 fld/fstp dance around it). The row
// pointer is derived from data+step here, NOT via cv::Mat::ptr() per call.
inline double bilin(const uchar* data, ptrdiff_t step, double x, double y) {
    const int x0 = static_cast<int>(x);
    const int y0 = static_cast<int>(y);
    const double fx = x - x0, fy = y - y0;
    const uchar* row0 = data + static_cast<ptrdiff_t>(y0) * step;
    const uchar* row1 = row0 + step;
    const double a = row0[x0] * (1.0 - fx) + row0[x0 + 1] * fx;
    const double b = row1[x0] * (1.0 - fx) + row1[x0 + 1] * fx;
    return a * (1.0 - fy) + b * fy;
}

// One deinterlaced field: search the center of strongest circular symmetry within
// the Range gate around the reference. Returns cx in full-frame x, cy in field-row
// space (detect_with_fields applies the +-0.5 correction).
MarkResult detect_one_field_csym(const cv::Mat& gFull, const CircularSymmetry& sym,
                                 double refX, double refY, int searchRadiusPx,
                                 double maxR, double minSym, const Settings& adj) {
    MarkResult r;
    const double rxF = (refX >= 0.0) ? refX : gFull.cols / 2.0;
    const double ryF = (refY >= 0.0) ? refY : gFull.rows / 2.0;
    const int sr = (searchRadiusPx > 0) ? searchRadiusPx : kCsym.searchFallbackPx;

    // ROI around the reference (the mark is always within Range of it).
    cv::Rect crop(0, 0, gFull.cols, gFull.rows);
    if (refX >= 0.0 && refY >= 0.0) {
        const int rad = sr + static_cast<int>(std::ceil(maxR)) + kCsym.roiPad;
        const cv::Rect c = cv::Rect(cvRound(rxF) - rad, cvRound(ryF) - rad, 2 * rad, 2 * rad) &
                           cv::Rect(0, 0, gFull.cols, gFull.rows);
        const int need = 2 * static_cast<int>(maxR) + 4;
        if (c.width > need && c.height > need) crop = c;
    }
    const cv::Mat sub = gFull(crop);
    const double meanv = cv::mean(sub)[0];
    if (meanv < kCsym.meanLo || meanv > kCsym.meanHi) return r;  // dropped / blown

    cv::Mat g;
    cv::GaussianBlur(sub, g, cv::Size(kCsym.gaussKernel, kCsym.gaussKernel), 1.0);
    apply_image_adjustments(g, adj);  // optional UI image adjustments (no-op when neutral)
    const double cxRef = rxF - crop.x, cyRef = ryF - crop.y;
    const double sr2 = static_cast<double>(sr) * sr;

    struct Cand {
        double x = 0.0, y = 0.0, s = -1.0, e = 0.0;
    };

    // --- Coarse localization: grid over the Range disc, with subsampled
    //     rings/points (full density only in the fine refine below). SERIAL on
    //     purpose: profiling showed parallelizing this sub-millisecond per-field
    //     search across the MSVC OpenCV ConcRT pool cost far more in scheduler
    //     overhead (VirtualProcessor / SchedulingNode / KiSwapThread /
    //     KeYieldExecution) than the work itself -- net-negative here. ---
    const double gstep = static_cast<double>(kCsym.coarseStep);
    const int nrows = static_cast<int>(std::lround(2.0 * static_cast<double>(sr) / gstep));
    const int sstep = kCsym.coarseSampleStep;
    Cand c0;
    for (int iy = 0; iy <= nrows; ++iy) {
        const double y = cyRef - sr + static_cast<double>(iy) * gstep;
        for (int ix = 0; ix <= nrows; ++ix) {
            const double x = cxRef - sr + static_cast<double>(ix) * gstep;
            const double dx = x - cxRef, dy = y - cyRef;
            if (dx * dx + dy * dy > sr2) continue;
            double e = 0.0;
            const double s = sym.score(g, x, y, e, sstep);
            if (s > c0.s) {
                c0.s = s;
                c0.x = x;
                c0.y = y;
                c0.e = e;
            }
        }
    }

    // --- Fine refine: small full-density grid around the coarse winner (serial;
    //     includes the coarse point itself, so c1 ends up full-density-scored). ---
    Cand c1;
    const int span = kCsym.fineSpan;
    for (int iy = -span; iy <= span; ++iy) {
        const double y = c0.y + iy;
        for (int ix = -span; ix <= span; ++ix) {
            const double x = c0.x + ix;
            const double dx = x - cxRef, dy = y - cyRef;
            if (dx * dx + dy * dy > sr2) continue;
            double e = 0.0;
            const double s = sym.score(g, x, y, e, 1);
            if (s > c1.s) {
                c1.s = s;
                c1.x = x;
                c1.y = y;
                c1.e = e;
            }
        }
    }
    if (c1.s < minSym) return r;

    // Sub-pixel: parabolic fit on the score surface around the 1px winner (4 extra
    // full-density evals, not an N x N sub-grid scan).
    double et = 0.0;
    const double sL = sym.score(g, c1.x - 1.0, c1.y, et, 1);
    const double sR = sym.score(g, c1.x + 1.0, c1.y, et, 1);
    const double sU = sym.score(g, c1.x, c1.y - 1.0, et, 1);
    const double sD = sym.score(g, c1.x, c1.y + 1.0, et, 1);
    double dx = 0.0, dy = 0.0;
    const double denx = sL - 2.0 * c1.s + sR;
    const double deny = sU - 2.0 * c1.s + sD;
    if (denx < 0.0) dx = 0.5 * (sL - sR) / denx;
    if (deny < 0.0) dy = 0.5 * (sU - sD) / deny;
    if (std::abs(dx) > 1.0) dx = 0.0;
    if (std::abs(dy) > 1.0) dy = 0.0;

    r.found = true;
    r.cx = c1.x + dx + crop.x;
    r.cy = c1.y + dy + crop.y;
    r.radius = c1.e;
    r.quality = c1.s;  // symmetry score (higher = better)
    return r;
}

}  // namespace

CircularSymmetry::CircularSymmetry(double minRadiusPx, double maxRadiusPx, int ringStep) {
    if (ringStep < 1) ringStep = 1;
    int nRings = static_cast<int>((maxRadiusPx - minRadiusPx) / ringStep) + 1;
    if (nRings < 1) nRings = 1;  // guard: an inverted bracket must NOT reserve(negative -> huge)
    ringStart_.reserve(static_cast<size_t>(nRings) + 1);
    ringR_.reserve(static_cast<size_t>(nRings));
    for (int ri = 0; ri < nRings; ++ri) {
        const double rr = minRadiusPx + static_cast<double>(ri) * ringStep;
        const int m = std::max(8, static_cast<int>(std::lround(2.0 * CV_PI * rr)));
        ringStart_.push_back(static_cast<int>(ox_.size()));
        ringR_.push_back(static_cast<float>(rr));
        for (int k = 0; k < m; ++k) {
            const double a = 2.0 * CV_PI * static_cast<double>(k) / m;
            ox_.push_back(static_cast<float>(rr * std::cos(a)));
            oy_.push_back(static_cast<float>(rr * std::sin(a)));
        }
    }
    ringStart_.push_back(static_cast<int>(ox_.size()));
}

// Trig-free: walks the precomputed ring offsets (built once in the constructor).
// step>1 subsamples rings and points (coarse pass); step=1 is full (fine pass).
double CircularSymmetry::score(const cv::Mat& g, double cx, double cy, double& edgeR, int step) const {
    double sumAll = 0.0, sumAll2 = 0.0;
    int nAll = 0;
    double wRingVar = 0.0, wSum = 0.0, maxRingVar = -1.0;
    edgeR = 0.0;
    const double wlim = g.cols - 1.0, hlim = g.rows - 1.0;
    const uchar* data = g.ptr<uchar>(0);                        // row pointer/stride computed ONCE here,
    const ptrdiff_t gstep = static_cast<ptrdiff_t>(g.step[0]);  // not cv::Mat::ptr() per sample
    const int nRings = static_cast<int>(ringR_.size());
    for (int ri = 0; ri < nRings; ri += step) {
        const int a = ringStart_[static_cast<size_t>(ri)];
        const int b = ringStart_[static_cast<size_t>(ri) + 1];
        double s = 0.0, s2 = 0.0;
        int cnt = 0;
        for (int i = a; i < b; i += step) {
            const double x = cx + ox_[static_cast<size_t>(i)], y = cy + oy_[static_cast<size_t>(i)];
            if (x < 1.0 || y < 1.0 || x >= wlim || y >= hlim) continue;
            const double v = bilin(data, gstep, x, y);
            s += v;
            s2 += v * v;
            ++cnt;
            sumAll += v;
            sumAll2 += v * v;
            ++nAll;
        }
        if (cnt < 4) continue;
        const double mean = s / cnt;
        double var = s2 / cnt - mean * mean;
        if (var < 0.0) var = 0.0;
        wRingVar += cnt * var;
        wSum += cnt;
        if (var > maxRingVar) {
            maxRingVar = var;
            edgeR = ringR_[static_cast<size_t>(ri)];
        }
    }
    if (nAll < 16 || wSum < 1.0) return 0.0;
    const double meanAll = sumAll / static_cast<double>(nAll);
    double overallVar = sumAll2 / static_cast<double>(nAll) - meanAll * meanAll;
    if (overallVar < 0.0) overallVar = 0.0;
    double avgRingVar = wRingVar / wSum;
    if (avgRingVar < 1e-6) avgRingVar = 1e-6;
    return overallVar / avgRingVar;
}

// Public: circular-symmetry fiducial detector, field-aware. Diameter bracket
// [minDiaPx,maxDiaPx] (<=0 -> physical default); two-field average when settled
// (no wovenWhenSettled -- the score integrates over rings, so it does not need the
// un-softened woven edges the contour detector relies on).
MarkResult detect_circular_symmetry(const void* frame, double refX, double refY,
                                    int searchRadiusPx, int minDiaPx, int maxDiaPx) {
    // EXCEPTION BARRIER: called across a plain C ABI from the host. The ring builder
    // and OpenCV can throw (e.g. on a bad UI radius bracket / OOM); never let that
    // unwind into the .NET host (undefined behavior, crash). Report not-found instead.
    try {
        // Precedence: UI/settings override > caller's diameter hint > built-in default.
        const Settings cfg = get_settings(MODE_ROUND);
        double minR = cfg.radiusMinPx > 0.0 ? cfg.radiusMinPx
                      : (minDiaPx > 0)      ? minDiaPx / 2.0
                                            : static_cast<double>(kCsym.minRadiusPx);
        double maxR = cfg.radiusMaxPx > 0.0 ? cfg.radiusMaxPx
                      : (maxDiaPx > 0)      ? maxDiaPx / 2.0
                                            : static_cast<double>(kCsym.maxRadiusPx);
        // Guard the bracket: an inverted/degenerate one (e.g. the user raised the min
        // slider above the still-Auto max) would otherwise make the ring builder
        // reserve a negative count -> throw. Widen to a sane bracket instead.
        if (minR < 1.0) minR = 1.0;
        if (maxR <= minR) maxR = minR + static_cast<double>(kCsym.maxRadiusPx - kCsym.minRadiusPx);
        const double minSym = cfg.minSymmetry > 0.0 ? cfg.minSymmetry : kCsym.minSymmetry;
        const CircularSymmetry sym(minR, maxR, kCsym.ringStep);
        MarkResult r = detect_with_fields(frame, [&](const cv::Mat& g) {
            return detect_one_field_csym(g, sym, refX, refY, searchRadiusPx, maxR, minSym, cfg);
        });
        r.shape = MarkShape::Circle;
        return r;
    } catch (...) {
        return MarkResult{};
    }
}

}  // namespace vis
