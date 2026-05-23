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
#include <vector>

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

int synthetic_comp_check() {
    // 640x480 dark field with a bright rectangular "component" rotated 20 deg, body
    // 80x40 px, centered off the exact image center. Exercises the full pose: a
    // stuck-at-center, wrong-size, or wrong-angle bug all fail.
    cv::Mat img(480, 640, CV_8UC1, cv::Scalar(20));
    const cv::Point2f center(330.0f, 246.0f);
    const cv::Size2f body(80.0f, 40.0f);
    const float angle = 20.0f;
    cv::Point2f pts[4];
    cv::RotatedRect(center, body, angle).points(pts);
    std::vector<cv::Point> poly{cv::Point(cvRound(pts[0].x), cvRound(pts[0].y)),
                                cv::Point(cvRound(pts[1].x), cvRound(pts[1].y)),
                                cv::Point(cvRound(pts[2].x), cvRound(pts[2].y)),
                                cv::Point(cvRound(pts[3].x), cvRound(pts[3].y))};
    cv::fillConvexPoly(img, poly, cv::Scalar(210), cv::LINE_AA);
    cv::GaussianBlur(img, img, cv::Size(3, 3), 0);

    IplImage ipl = make_ipl(img);
    vis::CompResult r = vis::detect_component(&ipl, /*expectedWpx=*/80.0, /*expectedHpx=*/40.0,
                                              /*expectedAngleDeg=*/20.0, /*threshold=*/50,
                                              /*refX=*/320.0, /*refY=*/240.0,
                                              /*searchRadiusPx=*/60);
    const char* method = r.method == vis::CompResult::Method::Symmetry      ? "symmetry"
                         : r.method == vis::CompResult::Method::MinAreaRect ? "minarearect"
                                                                           : "none";
    std::printf("synthetic-comp: found=%d cx=%.2f cy=%.2f w=%.2f h=%.2f angle=%.2f q=%.3f method=%s\n",
                r.found ? 1 : 0, r.cx, r.cy, r.w, r.h, r.angle, r.quality, method);
    if (!r.found) {
        std::printf("FAIL: detector found no component\n");
        return 1;
    }
    const double ec = std::hypot(r.cx - center.x, r.cy - center.y);
    // Body could come back as (80,40) or the swapped labeling; accept either ordering.
    const double longer = std::max(r.w, r.h), shorter = std::min(r.w, r.h);
    const double ew = std::abs(longer - 80.0), eh = std::abs(shorter - 40.0);
    double ea = std::abs(r.angle - static_cast<double>(angle));
    if (ea > 45.0) ea = std::abs(ea - 90.0);  // 90-deg labeling is acceptable
    int rc = 0;
    if (ec > 3.0) { std::printf("FAIL: center off by %.2f px (tol 3.0)\n", ec); rc = 1; }
    if (ew > 6.0 || eh > 6.0) { std::printf("FAIL: size off (%.2f, %.2f) px (tol 6.0)\n", ew, eh); rc = 1; }
    if (ea > 4.0) { std::printf("FAIL: angle off by %.2f deg (tol 4.0)\n", ea); rc = 1; }
    if (rc == 0) std::printf("PASS: center %.2fpx, size (%.2f,%.2f), angle %.2fdeg\n", ec, ew, eh, ea);
    return rc;
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
    const int rc = synthetic_self_check();
    return rc | synthetic_comp_check();
}
