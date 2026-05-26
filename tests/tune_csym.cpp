// Offline circular-symmetry rig. Runs the real vis::detect_circular_symmetry over
// captured PNG fiducial frames with the production config and reports per-frame
// results + aggregate hit-rate and centroid stability. Used by the public smoke
// (tests/smoke_comp.sh) and for ad-hoc tuning. Build standalone:
//   g++ -std=c++23 -O2 -I src tests/tune_csym.cpp \
//       src/detect_common.cpp src/detect_circle.cpp src/detect_contour.cpp \
//       src/detect_template.cpp src/detect_symmetry.cpp \
//       $(pkg-config --cflags --libs opencv4) -o /tmp/tune_csym
//
// CLI:  tune_csym [--refx X] [--refy Y] [--r SEARCH_RADIUS] frame1.png frame2.png ...
// Defaults are 370 / 270 / 45 -- the historical mvo -50/-30 / areaMin=45 case
// the public corpus was captured under; flags exist so private corpora captured
// at other mark-search positions can be swept without recompiling.
#include "vision.h"

#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/imgcodecs.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

static IplImage make_ipl(cv::Mat& m) {
    IplImage ipl{};
    ipl.nSize = sizeof(IplImage);
    ipl.nChannels = m.channels();
    ipl.depth = IPL_DEPTH_8U;
    ipl.dataOrder = 0;
    ipl.origin = 0;
    ipl.width = m.cols;
    ipl.height = m.rows;
    ipl.widthStep = static_cast<int>(m.step);
    ipl.imageSize = ipl.widthStep * m.rows;
    ipl.imageData = reinterpret_cast<char*>(m.data);
    ipl.imageDataOrigin = ipl.imageData;
    return ipl;
}

int main(int argc, char** argv) {
    double refX = 370.0;   // mvo -50/-30 -> ref 370/270 (legacy)
    double refY = 270.0;
    int searchR = 45;      // areaMin=45 -> Range search radius (legacy)

    int n = 0, hit = 0;
    double sx = 0, sy = 0, sx2 = 0, sy2 = 0;
    std::printf("file,found,cx,cy,radius,score\n");
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--refx") == 0 && i + 1 < argc) {
            refX = std::atof(argv[++i]);
            continue;
        }
        if (std::strcmp(argv[i], "--refy") == 0 && i + 1 < argc) {
            refY = std::atof(argv[++i]);
            continue;
        }
        if (std::strcmp(argv[i], "--r") == 0 && i + 1 < argc) {
            searchR = std::atoi(argv[++i]);
            continue;
        }
        cv::Mat img = cv::imread(argv[i], cv::IMREAD_COLOR);
        if (img.empty()) {
            std::printf("%s,READ_ERROR\n", argv[i]);
            continue;
        }
        IplImage ipl = make_ipl(img);
        vis::MarkResult r = vis::detect_circular_symmetry(&ipl, refX, refY, searchR);
        std::printf("%s,%d,%.2f,%.2f,%.2f,%.3f\n", argv[i], r.found ? 1 : 0,
                    r.cx, r.cy, r.radius, r.quality);
        ++n;
        if (r.found) {
            ++hit;
            sx += r.cx;
            sy += r.cy;
            sx2 += r.cx * r.cx;
            sy2 += r.cy * r.cy;
        }
    }
    if (hit) {
        double mx = sx / hit, my = sy / hit;
        double vx = sx2 / hit - mx * mx, vy = sy2 / hit - my * my;
        std::fprintf(stderr, "\nCSYM HITRATE %d/%d = %.0f%%  cx=%.2f(sd %.2f)  cy=%.2f(sd %.2f)\n",
                     hit, n, 100.0 * hit / n, mx, std::sqrt(vx < 0 ? 0 : vx),
                     my, std::sqrt(vy < 0 ? 0 : vy));
    } else {
        std::fprintf(stderr, "\nCSYM HITRATE 0/%d\n", n);
    }
    return 0;
}
