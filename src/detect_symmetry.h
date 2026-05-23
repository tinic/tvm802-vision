#pragma once

#include "vision.h"

#include <opencv2/core.hpp>

#include <vector>

namespace vis {

// Reusable concentric-ring symmetry scorer (OpenPnP DetectCircularSymmetry,
// reimplemented from the published algorithm -- no third-party code). At a
// candidate center it samples the image on concentric rings over a radius
// bracket and returns the variance over ALL ring samples divided by the
// area-weighted mean of the per-ring variances. At a true concentric center
// every ring is internally uniform (low per-ring variance) while the rings
// differ radially (high overall variance), so the ratio peaks -- threshold- and
// edge-free, robust to soft/low-contrast/specular rings. Stateless w.r.t. the
// image; holds only the ring geometry, so one instance is reused across a frame's
// two fields. (A rectlinear sibling for up-vision components will live here too.)
class CircularSymmetry {
public:
    // Precomputes the ring sample offsets once (the costly cos/sin are done here,
    // NOT per candidate center) so score() is a trig-free gather.
    CircularSymmetry(double minRadiusPx, double maxRadiusPx, int ringStep);

    // g: single-channel 8U. (cx,cy): candidate center in g's pixels. edgeR (out):
    // radius of the strongest ring (the dominant circular edge). Returns the
    // symmetry score (higher = stronger concentric symmetry; ~1 = none).
    // step>1 subsamples rings AND points (step*step fewer samples) -- used for the
    // cheap coarse localization pass; the fine/sub-pixel pass uses step=1 for the
    // accurate score and center.
    double score(const cv::Mat& g, double cx, double cy, double& edgeR, int step = 1) const;

private:
    // Sample-point offsets from the center, grouped by ring. ring i occupies
    // [ringStart_[i], ringStart_[i+1]); ringR_[i] is its radius.
    std::vector<float> ox_;
    std::vector<float> oy_;
    std::vector<int> ringStart_;
    std::vector<float> ringR_;
};

}  // namespace vis
