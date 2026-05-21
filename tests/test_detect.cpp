// Offline regression harness for the OpenCV detector.
//
// Not built by default — configure with -DBUILD_TESTS=ON. It links only
// src/vision.cpp (which depends on OpenCV alone, no Windows headers), so it
// builds and runs on any host with OpenCV. Two modes:
//   (no args)         synthetic self-check: draw a fiducial, detect it, assert
//                     the recovered center. Returns non-zero on failure.
//   <png> [<png>...]  run detection over real captured frames (e.g. the PNGs the
//                     in-DLL capture writes to C:\mvision_capture) and print CSV.
//
// The synthetic case is wired as a ctest (`detect_synthetic`).

#include "vision.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/types_c.h>

#include <cmath>
#include <cstdio>

namespace {

// Build an IplImage header that points at `m`'s pixel buffer (no copy). The
// detector validates exactly these fields (see iplframe.h), so set them all.
IplImage make_ipl(cv::Mat& m) {
    IplImage ipl{};
    ipl.nSize = sizeof(IplImage);
    ipl.nChannels = m.channels();
    ipl.depth = IPL_DEPTH_8U;
    ipl.dataOrder = 0;
    ipl.origin = 0;  // top-left / top-down
    ipl.width = m.cols;
    ipl.height = m.rows;
    ipl.widthStep = static_cast<int>(m.step);
    ipl.imageSize = ipl.widthStep * m.rows;
    ipl.imageData = reinterpret_cast<char*>(m.data);
    ipl.imageDataOrigin = ipl.imageData;
    return ipl;
}

int synthetic_self_check() {
    // 640x480 gray field with a bright copper-like disc, offset from the exact
    // center so a stuck "report the center" bug would be caught.
    cv::Mat img(480, 640, CV_8UC1, cv::Scalar(45));
    const cv::Point2d center(322.0, 241.0);
    const int radius = 14;  // ~1mm copper; markSize 44 -> bracket [9, 18]
    cv::circle(img, cv::Point(cvRound(center.x), cvRound(center.y)), radius,
               cv::Scalar(225), -1, cv::LINE_AA);
    cv::GaussianBlur(img, img, cv::Size(3, 3), 0);  // soften the synthetic edge

    IplImage ipl = make_ipl(img);
    vis::MarkResult r = vis::detect_circle_mark(&ipl, /*markSizePx=*/44,
                                                /*refX=*/320.0, /*refY=*/240.0,
                                                /*searchRadiusPx=*/120);

    std::printf("synthetic: found=%d cx=%.2f cy=%.2f radius=%.2f quality=%.3f\n",
                r.found ? 1 : 0, r.cx, r.cy, r.radius, r.quality);
    if (!r.found) {
        std::printf("FAIL: detector found no mark\n");
        return 1;
    }
    const double ex = std::abs(r.cx - center.x), ey = std::abs(r.cy - center.y);
    if (ex > 2.0 || ey > 2.0) {
        std::printf("FAIL: center off by (%.2f, %.2f) px, tolerance 2.0\n", ex, ey);
        return 1;
    }
    std::printf("PASS: center within (%.2f, %.2f) px\n", ex, ey);
    return 0;
}

int run_over_files(int n, char** paths) {
    std::printf("file,found,cx,cy,radius,quality\n");
    int rc = 0;
    for (int i = 0; i < n; ++i) {
        cv::Mat g = cv::imread(paths[i], cv::IMREAD_GRAYSCALE);
        if (g.empty()) {
            std::printf("%s,READ_ERROR\n", paths[i]);
            rc = 1;
            continue;
        }
        IplImage ipl = make_ipl(g);
        vis::MarkResult r = vis::detect_circle_mark(&ipl, /*markSizePx=*/44,
                                                    g.cols / 2.0, g.rows / 2.0,
                                                    /*searchRadiusPx=*/0);
        std::printf("%s,%d,%.2f,%.2f,%.2f,%.3f\n", paths[i], r.found ? 1 : 0,
                    r.cx, r.cy, r.radius, r.quality);
    }
    return rc;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1) return run_over_files(argc - 1, argv + 1);
    return synthetic_self_check();
}
