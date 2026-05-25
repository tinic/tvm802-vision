// Offline up-vision component (CheckComp) tuning rig. Runs the real
// vis::detect_component over captured PNG frames (the shim writes comp_NNNN.png +
// compare.log when capture is armed) and prints per-frame pose + a method tally, so
// the detector can be tuned against the original's logged baseline before going
// live. Optional priors via flags. Build standalone:
//   g++ -std=c++20 -O2 -pthread -I src tests/tune_comp.cpp \
//       src/detect_common.cpp src/detect_circle.cpp src/detect_contour.cpp \
//       src/detect_template.cpp src/detect_symmetry.cpp src/detect_component.cpp \
//       src/settings.cpp src/thread_pool.cpp \
//       $(pkg-config --cflags --libs opencv4) -o /tmp/tune_comp
// Usage: /tmp/tune_comp [--w EXPW_PX --h EXPH_PX --a EXPA_DEG] frame*.png
#include "vision.h"

#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/imgcodecs.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

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
    double expW = 0.0, expH = 0.0;
    double expA = std::numeric_limits<double>::quiet_NaN();
    int sym = 0, mar = 0, miss = 0, n = 0;

    std::printf("file,found,cx,cy,w,h,angle,quality,method\n");
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--w") == 0 && i + 1 < argc) {
            expW = std::atof(argv[++i]);
            continue;
        }
        if (std::strcmp(argv[i], "--h") == 0 && i + 1 < argc) {
            expH = std::atof(argv[++i]);
            continue;
        }
        if (std::strcmp(argv[i], "--a") == 0 && i + 1 < argc) {
            expA = std::atof(argv[++i]);
            continue;
        }

        cv::Mat img = cv::imread(argv[i], cv::IMREAD_COLOR);
        if (img.empty()) {
            std::printf("%s,READ_ERROR\n", argv[i]);
            continue;
        }
        IplImage ipl = make_ipl(img);
        vis::CompResult r = vis::detect_component(&ipl, expW, expH, expA, /*threshold=*/0,
                                                  /*refX=*/-1.0, /*refY=*/-1.0, /*searchRadiusPx=*/0);
        const char* method = r.method == vis::CompResult::Method::Symmetry      ? "symmetry"
                             : r.method == vis::CompResult::Method::MinAreaRect ? "minarearect"
                                                                                : "none";
        std::printf("%s,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%s\n", argv[i], r.found ? 1 : 0,
                    r.cx, r.cy, r.w, r.h, r.angle, r.quality, method);
        ++n;
        if (!r.found)
            ++miss;
        else if (r.method == vis::CompResult::Method::Symmetry)
            ++sym;
        else
            ++mar;
    }
    if (n > 0)
        std::fprintf(stderr, "\nCOMP %d frames: symmetry=%d minarearect=%d miss=%d\n", n, sym, mar, miss);
    return 0;
}
