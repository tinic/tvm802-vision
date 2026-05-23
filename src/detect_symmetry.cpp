#include "detect_symmetry.h"

#include "detect_common.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

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
    int coarseStep = 2;        // center grid: coarse pass
    int fineSpan = 3;          // +-span around the coarse winner at 1px
    double subStep = 0.25;     // sub-pixel grid (superSampling = 4)
};
constexpr CsymParams kCsym;

// Bilinear sample; caller guarantees 1 <= x < cols-1 and 1 <= y < rows-1.
inline double bilin(const cv::Mat& g, double x, double y) {
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const double fx = x - x0, fy = y - y0;
    const uchar* row0 = g.ptr<uchar>(y0);
    const uchar* row1 = g.ptr<uchar>(y0 + 1);
    const double a = row0[x0] * (1.0 - fx) + row0[x0 + 1] * fx;
    const double b = row1[x0] * (1.0 - fx) + row1[x0 + 1] * fx;
    return a * (1.0 - fy) + b * fy;
}

// One deinterlaced field: search the center of strongest circular symmetry within
// the Range gate around the reference. Returns cx in full-frame x, cy in field-row
// space (detect_with_fields applies the +-0.5 correction).
MarkResult detect_one_field_csym(const cv::Mat& gFull, const CircularSymmetry& sym,
                                 double refX, double refY, int searchRadiusPx,
                                 double maxR, double minSym) {
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
    const double cxRef = rxF - crop.x, cyRef = ryF - crop.y;
    const double sr2 = static_cast<double>(sr) * sr;

    struct Cand {
        double x = 0.0, y = 0.0, s = -1.0, e = 0.0;
    };
    // Max-score center over a grid, gated to the Range circle around the reference.
    auto scan = [&](double x0, double y0, double rad, double step) {
        Cand best;
        best.x = x0;
        best.y = y0;
        const int steps = static_cast<int>(std::lround(2.0 * rad / step));
        for (int iy = 0; iy <= steps; ++iy) {
            const double y = y0 - rad + static_cast<double>(iy) * step;
            for (int ix = 0; ix <= steps; ++ix) {
                const double x = x0 - rad + static_cast<double>(ix) * step;
                const double dx = x - cxRef, dy = y - cyRef;
                if (dx * dx + dy * dy > sr2) continue;
                double e = 0.0;
                const double s = sym.score(g, x, y, e);
                if (s > best.s) {
                    best.s = s;
                    best.x = x;
                    best.y = y;
                    best.e = e;
                }
            }
        }
        return best;
    };

    const Cand c0 = scan(cxRef, cyRef, static_cast<double>(sr), static_cast<double>(kCsym.coarseStep));
    const Cand c1 = scan(c0.x, c0.y, static_cast<double>(kCsym.fineSpan), 1.0);
    const Cand c2 = scan(c1.x, c1.y, 1.0, kCsym.subStep);
    if (c2.s < minSym) return r;

    r.found = true;
    r.cx = c2.x + crop.x;
    r.cy = c2.y + crop.y;
    r.radius = c2.e;
    r.quality = c2.s;  // symmetry score (higher = better)
    return r;
}

}  // namespace

double CircularSymmetry::score(const cv::Mat& g, double cx, double cy, double& edgeR) const {
    double sumAll = 0.0, sumAll2 = 0.0;
    int nAll = 0;
    double wRingVar = 0.0, wSum = 0.0, maxRingVar = -1.0;
    edgeR = 0.0;
    const int nRings = static_cast<int>((maxR_ - minR_) / ringStep_) + 1;
    for (int ri = 0; ri < nRings; ++ri) {
        const double rr = minR_ + static_cast<double>(ri) * ringStep_;
        const int m = std::max(8, static_cast<int>(std::lround(2.0 * CV_PI * rr)));
        double s = 0.0, s2 = 0.0;
        int cnt = 0;
        for (int k = 0; k < m; ++k) {
            const double a = 2.0 * CV_PI * k / m;
            const double x = cx + rr * std::cos(a), y = cy + rr * std::sin(a);
            if (x < 1.0 || y < 1.0 || x >= g.cols - 1 || y >= g.rows - 1) continue;
            const double v = bilin(g, x, y);
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
            edgeR = rr;
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
    const double minR = (minDiaPx > 0) ? minDiaPx / 2.0 : static_cast<double>(kCsym.minRadiusPx);
    const double maxR = (maxDiaPx > 0) ? maxDiaPx / 2.0 : static_cast<double>(kCsym.maxRadiusPx);
    const CircularSymmetry sym(minR, maxR, kCsym.ringStep);
    MarkResult r = detect_with_fields(frame, [&](const cv::Mat& g) {
        return detect_one_field_csym(g, sym, refX, refY, searchRadiusPx, maxR, kCsym.minSymmetry);
    });
    r.shape = MarkShape::Circle;
    return r;
}

}  // namespace vis
