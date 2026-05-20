#include "vision.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/types_c.h>  // IplImage (frozen ABI, matches OpenCV 2.4)

#define NOMINMAX                   // keep std::min/std::max usable
#include <windows.h>               // GDI for self-render (defines `small`, `min`, `max`)
#include <vector>
#include <algorithm>
#include <cmath>
#include <functional>

namespace {
// Validate an IplImage* and wrap its pixel buffer in a cv::Mat (no copy).
// Returns an empty Mat on failure. Reads only — never modifies the buffer.
cv::Mat wrap_ipl(const void* frame, int& origin) {
    origin = -1;
    if (!frame) return {};
    const IplImage* ipl = reinterpret_cast<const IplImage*>(frame);
    if (ipl->nSize != sizeof(IplImage)) return {};
    if (ipl->width <= 0 || ipl->height <= 0) return {};
    if (ipl->width > 8192 || ipl->height > 8192) return {};
    if (ipl->nChannels != 1 && ipl->nChannels != 3) return {};
    if (ipl->depth != IPL_DEPTH_8U) return {};
    if (!ipl->imageData) return {};
    origin = ipl->origin;
    return cv::Mat(ipl->height, ipl->width, CV_MAKETYPE(CV_8U, ipl->nChannels),
                   ipl->imageData, static_cast<size_t>(ipl->widthStep));
}
} // namespace

namespace vis {

unsigned int frame_hash(const void* frame) {
    if (!frame) return 0;
    const IplImage* ipl = reinterpret_cast<const IplImage*>(frame);
    if (ipl->nSize != sizeof(IplImage)) return 0;
    if (ipl->width <= 0 || ipl->height <= 0) return 0;
    if (ipl->nChannels != 1 && ipl->nChannels != 3) return 0;
    if (!ipl->imageData) return 0;
    unsigned int h = 2166136261u;                       // FNV-1a offset basis
    const unsigned char* d = reinterpret_cast<const unsigned char*>(ipl->imageData);
    const int ws = ipl->widthStep, rowbytes = ipl->width * ipl->nChannels;
    for (int y = 0; y < ipl->height; y += 37)
        for (int x = 0; x < rowbytes; x += 17)
            h = (h ^ d[y * ws + x]) * 16777619u;
    return h ? h : 1u;                                  // 0 reserved for invalid
}

bool save_frame(const void* frame, const char* path) {
    if (!path) return false;
    int origin = -1;
    cv::Mat wrapped = wrap_ipl(frame, origin);
    if (wrapped.empty()) return false;
    cv::Mat out;
    if (origin == IPL_ORIGIN_BL) cv::flip(wrapped, out, 0);  // into a NEW Mat
    else                         out = wrapped.clone();       // copy; never touch live buffer
    try { return cv::imwrite(path, out); } catch (...) { return false; }
}

bool render_preview(const void* frame, void* hwndV, double mvoX, double mvoY,
                    int searchRadiusPx, const MarkResult& mr) {
    if (!hwndV) return false;
    int origin = -1;
    cv::Mat wrapped = wrap_ipl(frame, origin);
    if (wrapped.empty()) return false;

    cv::Mat img;
    if (origin == IPL_ORIGIN_BL) cv::flip(wrapped, img, 0); else img = wrapped;
    cv::Mat bgr;
    if (img.channels() == 1)      cv::cvtColor(img, bgr, cv::COLOR_GRAY2BGR);
    else if (img.channels() == 3) bgr = img;
    else return false;

    const int W = bgr.cols, H = bgr.rows;
    cv::Mat flipped;
    cv::flip(bgr, flipped, -1);                  // 180 deg, matches the original mirror

    HWND hwnd = reinterpret_cast<HWND>(hwndV);
    HDC dc = GetDC(hwnd);
    if (!dc) return false;
    RECT rc{}; GetClientRect(hwnd, &rc);
    int rw = rc.right - rc.left, rh = rc.bottom - rc.top;
    if (rw <= 0) rw = W;
    if (rh <= 0) rh = H;

    // 1:1 NATIVE preview (no upscaling -> crisp overlay; a stretched crop made
    // the thin red lines blocky). Crop a window-sized region at native pixels,
    // centered on the reference. Cap the crop to the source minus the mvo offset
    // so we never run off the frame (no replicated border), and round the width
    // down to a multiple of 4 so the cv::Mat row stride (cw*3) is DIB-aligned.
    const double cx = W / 2.0 + mvoX, cy = H / 2.0 + mvoY;   // reference, flipped coords
    int marginW = W - 2 * static_cast<int>(std::ceil(std::abs(mvoX)));
    int marginH = H - 2 * static_cast<int>(std::ceil(std::abs(mvoY)));
    int cw = std::min(rw, marginW > 16 ? marginW : W);
    int ch = std::min(rh, marginH > 16 ? marginH : H);
    cw &= ~3;                                   // multiple of 4 -> DIB row aligned
    if (cw < 16 || ch < 16) { ReleaseDC(hwnd, dc); return false; }

    cv::Mat work;
    cv::getRectSubPix(flipped, cv::Size(cw, ch),
                      cv::Point2f((float)cx, (float)cy), work);
    if (work.type() != CV_8UC3) { ReleaseDC(hwnd, dc); return false; }

    const int RT = 2;   // red overlay thickness (crisp at 1:1, more visible)
    // Reference crosshair (red) at the crop center == the reference point.
    cv::line(work, cv::Point(cw / 2, 0), cv::Point(cw / 2, ch), cv::Scalar(0, 0, 255), RT);
    cv::line(work, cv::Point(0, ch / 2), cv::Point(cw, ch / 2), cv::Scalar(0, 0, 255), RT);
    // Search-area ("Range") circle (red) — detection is limited to inside this.
    if (searchRadiusPx > 0)
        cv::circle(work, cv::Point(cw / 2, ch / 2), searchRadiusPx, cv::Scalar(0, 0, 255), RT);

    // Our detection (green) mapped from frame coords into crop coords.
    if (mr.found) {
        const double cropLeft = cx - cw / 2.0;
        const double cropTop  = cy - ch / 2.0;
        const int u = cvRound((W - mr.cx) - cropLeft);
        const int v = cvRound((H - mr.cy) - cropTop);
        cv::circle(work, cv::Point(u, v), cvRound(mr.radius), cv::Scalar(0, 255, 0), 2);
        cv::drawMarker(work, cv::Point(u, v), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 14, 2);
    }

    if (!work.isContinuous()) work = work.clone();

    BITMAPINFO bmi; ZeroMemory(&bmi, sizeof(bmi));
    bmi.bmiHeader.biSize        = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth       = cw;
    bmi.bmiHeader.biHeight      = -ch;           // negative = top-down (matches cv::Mat)
    bmi.bmiHeader.biPlanes      = 1;
    bmi.bmiHeader.biBitCount    = 24;
    bmi.bmiHeader.biCompression = BI_RGB;
    // 1:1 blit (dst size == src size -> no scaling). Top-left; black-fill any
    // border strips (only when the window is larger than the native crop).
    SetStretchBltMode(dc, COLORONCOLOR);
    StretchDIBits(dc, 0, 0, cw, ch, 0, 0, cw, ch, work.data, &bmi, DIB_RGB_COLORS, SRCCOPY);
    HBRUSH black = (HBRUSH)GetStockObject(BLACK_BRUSH);
    if (rw > cw) { RECT s{ cw, 0, rw, rh }; FillRect(dc, &s, black); }
    if (rh > ch) { RECT s{ 0, ch, cw, rh }; FillRect(dc, &s, black); }
    ReleaseDC(hwnd, dc);
    return true;
}

// Detect the copper mark in ONE already-deinterlaced, full-height grayscale
// field image. cx comes out in full-frame x; cy is in this image's row space and
// needs the caller's +-0.5 field correction. Returns found=false on no mark.
// This is the per-field core; detect_circle_mark runs it on both fields.
static MarkResult detect_one_field(const cv::Mat& gFull, int markSizePx,
                                   double refX, double refY,
                                   int searchRadiusPx) {
    MarkResult r;

    // Tight radius bracket locked to the inner 1mm copper (radius ~0.32*size);
    // excludes the larger solder-mask ring (~0.63*size) that otherwise causes
    // flip-flop jitter. markSizePx is the host "size" arg (algo=63 ~ 1.2mm).
    // Computed first so it can size the ROI below.
    int minRfull, maxRfull;
    if (markSizePx >= 16 && markSizePx <= 400) {
        minRfull = std::max(8, static_cast<int>(markSizePx * 0.22));
        maxRfull =            static_cast<int>(markSizePx * 0.42);
    } else {
        minRfull = 12; maxRfull = 28;
    }

    // Restrict the WHOLE pipeline to a ROI around the reference. The mark is
    // always within the search radius of it, so this cuts detection latency ~2x
    // (HoughCircles + filters over a small window) with identical results --
    // lower latency tightens the host's closed-loop lock-in. No reference (or a
    // ROI too small) -> use the full field.
    const double rxF = (refX >= 0.0) ? refX : gFull.cols / 2.0;
    const double ryF = (refY >= 0.0) ? refY : gFull.rows / 2.0;
    const int sr = (searchRadiusPx > 0) ? searchRadiusPx : 150;
    cv::Rect crop(0, 0, gFull.cols, gFull.rows);
    if (refX >= 0.0 && refY >= 0.0) {
        const int R = sr + maxRfull + 6;
        cv::Rect c = cv::Rect(cvRound(rxF) - R, cvRound(ryF) - R, 2 * R, 2 * R)
                     & cv::Rect(0, 0, gFull.cols, gFull.rows);
        if (c.width >= 2 * minRfull + 8 && c.height >= 2 * minRfull + 8) crop = c;
    }
    const cv::Mat g = gFull(crop);
    const double cxRef = rxF - crop.x, cyRef = ryF - crop.y;   // reference in crop coords

    r.imgMean = cv::mean(g)[0];
    if (r.imgMean < 6.0 || r.imgMean > 250.0) return r;   // dropped/blown

    // Denoise -> illumination normalize -> light smooth. Denoise BEFORE CLAHE
    // (CLAHE amplifies noise). medianBlur kills analog impulse pixels; the
    // edge-preserving bilateral removes sensor noise without softening the
    // copper edge. bilateralFilter cannot run in-place: src (med) != dst (den).
    cv::Mat med, den, norm, det;
    cv::medianBlur(g, med, 3);
    cv::bilateralFilter(med, den, /*d*/ 5, /*sigmaColor*/ 25, /*sigmaSpace*/ 25);
    cv::createCLAHE(3.0, cv::Size(8, 8))->apply(den, norm);
    cv::GaussianBlur(norm, det, cv::Size(3, 3), 1.0);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(det, circles, cv::HOUGH_GRADIENT, /*dp*/ 1.0,
                     /*minDist*/ std::max(8.0, det.rows / 4.0), /*param1*/ 80,
                     /*param2*/ 25, /*minRadius*/ minRfull, /*maxRadius*/ maxRfull);
    if (circles.empty()) return r;

    int best = 0; double bestD = 1e18;
    for (size_t i = 0; i < circles.size(); ++i) {
        const double dx = circles[i][0] - cxRef, dy = circles[i][1] - cyRef;
        const double d = dx * dx + dy * dy;
        if (d < bestD) { bestD = d; best = static_cast<int>(i); }
    }
    double fx = circles[best][0], fy = circles[best][1], fr = circles[best][2];

    // Search-area ("Range") constraint: reject detections too far from the ref.
    if (searchRadiusPx > 0) {
        const double ddx = fx - cxRef, ddy = fy - cyRef;
        if (ddx * ddx + ddy * ddy > double(searchRadiusPx) * searchRadiusPx)
            return r;
    }

    // Circularity: reject HoughCircles votes on rectangular pads (need a real
    // edge around most of the circumference). The edge-support fraction also
    // doubles as a detection-quality score (used for the even/odd tiebreak).
    double circQuality = 0.0;
    {
        const int N = 24; int hits = 0, valid = 0;
        for (int k = 0; k < N; ++k) {
            const double a = 2.0 * CV_PI * k / N, ca = std::cos(a), sa = std::sin(a);
            const int xin = cvRound(fx + (fr - 3) * ca), yin = cvRound(fy + (fr - 3) * sa);
            const int xou = cvRound(fx + (fr + 3) * ca), you = cvRound(fy + (fr + 3) * sa);
            if (xin < 0 || yin < 0 || xin >= det.cols || yin >= det.rows) continue;
            if (xou < 0 || you < 0 || xou >= det.cols || you >= det.rows) continue;
            ++valid;
            if (std::abs(int(det.at<uchar>(yin, xin)) - int(det.at<uchar>(you, xou))) > 18)
                ++hits;
        }
        if (valid < N / 2 || hits < 0.70 * valid) return r;
        circQuality = double(hits) / valid;
    }

    // Contrast gate: a real copper dot is a solid bright/dark blob vs annulus.
    {
        const int rr = std::max(2, int(fr));
        double sin_ = 0, sout = 0; int nin = 0, nout = 0;
        for (int dy = -rr - 4; dy <= rr + 4; ++dy)
            for (int dx = -rr - 4; dx <= rr + 4; ++dx) {
                const int xx = cvRound(fx) + dx, yy = cvRound(fy) + dy;
                if (xx < 0 || yy < 0 || xx >= det.cols || yy >= det.rows) continue;
                const double d2 = double(dx) * dx + double(dy) * dy;
                if (d2 < 0.36 * rr * rr) { sin_ += det.at<uchar>(yy, xx); ++nin; }
                else if (d2 > 1.32 * rr * rr && d2 < 2.25 * rr * rr) { sout += det.at<uchar>(yy, xx); ++nout; }
            }
        if (nin > 0 && nout > 0 && std::abs(sin_ / nin - sout / nout) < 28.0)
            return r;
    }

    // Sub-pixel refine: intensity-weighted centroid around the Hough center.
    const int m = static_cast<int>(fr * 1.3) + 4;
    cv::Rect roi(cvRound(fx) - m, cvRound(fy) - m, 2 * m, 2 * m);
    roi &= cv::Rect(0, 0, norm.cols, norm.rows);
    if (roi.width > 6 && roi.height > 6) {
        cv::Mat patch;
        norm(roi).convertTo(patch, CV_32F);
        const double mean = cv::mean(patch)[0];
        cv::Mat w = patch - mean;
        cv::threshold(w, w, 0.0, 0.0, cv::THRESH_TOZERO);
        const double sw = cv::sum(w)[0];
        if (sw > 1.0) {
            cv::Mat colW, rowW;
            cv::reduce(w, colW, 0, cv::REDUCE_SUM, CV_64F);
            cv::reduce(w, rowW, 1, cv::REDUCE_SUM, CV_64F);
            double sx = 0, sy = 0;
            for (int x = 0; x < colW.cols; ++x) sx += colW.at<double>(0, x) * x;
            for (int y = 0; y < rowW.rows; ++y) sy += rowW.at<double>(y, 0) * y;
            const double rx = roi.x + sx / sw, ry = roi.y + sy / sw;
            if (std::abs(rx - fx) <= fr * 0.7 && std::abs(ry - fy) <= fr * 0.7) {
                fx = rx; fy = ry;
            }
        }
    }

    r.found = true;
    r.cx = fx + crop.x;          // map crop coords -> field coords
    r.cy = fy + crop.y;
    r.radius = fr;
    r.quality = circQuality;
    const double maxD = std::sqrt(rxF * rxF + ryF * ryF);
    r.score = 1.0 - std::sqrt(bestD) / maxD;   // bestD is crop-relative = field-relative dist
    return r;
}

// Field-aware wrapper: deinterlace the analog frame into its two time-separated
// fields (even/odd scanlines, ~1/60s apart), upscale each back to full height,
// run `perField` on BOTH, and combine. The STK1150 weaves the fields into one
// frame, so under head motion they comb ("double image"); detecting on each
// single-instant field and taking the best (or averaging when they agree)
// removes the combing and ~doubles temporal sampling. The down-vision read has
// no settle-delay (unlike the up camera), so it fires mid-motion at high speed.
//
// `perField` runs on one upscaled full-height field and returns a MarkResult
// with cx in full-frame x and cy in field-row space; this wrapper applies the
// +-0.5 row correction and combines. Reused by every field-aware detector
// (circular, template, ...). NON-DESTRUCTIVE: copies out of the frame buffer.
static MarkResult detect_with_fields(
        const void* frame,
        const std::function<MarkResult(const cv::Mat&)>& perField) {
    MarkResult r;
    if (!frame) return r;

    // The native QueryFrame returns an OpenCV 2.4 IplImage*. Its struct layout
    // is ABI-frozen, so OpenCV 4's IplImage matches it. Validate before trusting.
    const IplImage* ipl = reinterpret_cast<const IplImage*>(frame);
    if (ipl->nSize != sizeof(IplImage)) return r;          // not an IplImage
    if (ipl->width <= 0 || ipl->height <= 0) return r;
    if (ipl->width > 8192 || ipl->height > 8192) return r;
    if (ipl->nChannels != 1 && ipl->nChannels != 3) return r;
    if (ipl->depth != IPL_DEPTH_8U) return r;
    if (!ipl->imageData) return r;

    r.headerOk    = true;
    r.imgW        = ipl->width;
    r.imgH        = ipl->height;
    r.imgChannels = ipl->nChannels;
    r.imgOrigin   = ipl->origin;
    r.imgStep     = ipl->widthStep;

    r.frameHash = frame_hash(frame);   // stale-frame diagnostic (see compare.log)

    cv::Mat wrapped(ipl->height, ipl->width,
                    CV_MAKETYPE(CV_8U, ipl->nChannels),
                    ipl->imageData, static_cast<size_t>(ipl->widthStep));

    // IplImage origin: 0 = top-left (top-down), 1 = bottom-left (bottom-up).
    cv::Mat img;
    if (ipl->origin == IPL_ORIGIN_BL) cv::flip(wrapped, img, 0);
    else                              img = wrapped;

    cv::Mat gray;
    if (img.channels() == 3) cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    else                     gray = img;

    const size_t step = gray.step[0];
    cv::Mat evenF, oddF, evenU, oddU;
    cv::Mat(gray.rows / 2, gray.cols, gray.type(), gray.data,        step * 2).copyTo(evenF);
    cv::Mat(gray.rows / 2, gray.cols, gray.type(), gray.data + step, step * 2).copyTo(oddF);
    // Bob-deinterlace each half-height field back to full height. INTER_CUBIC
    // (not LINEAR) preserves the copper edge sharpness that HoughCircles' param2
    // accumulator and the circularity ring-sample at fr+-3 both rely on; the
    // ~1px vertical softening from bilinear blunts both. Two small resizes/frame
    // -> negligible cost.
    cv::resize(evenF, evenU, cv::Size(gray.cols, gray.rows), 0, 0, cv::INTER_CUBIC);
    cv::resize(oddF,  oddU,  cv::Size(gray.cols, gray.rows), 0, 0, cv::INTER_CUBIC);

    MarkResult re = perField(evenU);
    MarkResult ro = perField(oddU);
    if (re.found) re.cy -= 0.5;   // even field samples rows 2i   -> full-frame y
    if (ro.found) ro.cy += 0.5;   // odd  field samples rows 2i+1 -> full-frame y

    // The two fields are captured ~1/60 s apart. Combine them by motion state:
    //  - settled (fields agree): AVERAGE -> full vertical resolution, cancels
    //    the opposite +-0.5 row bias.
    //  - moving (fields disagree): report ONLY the most-recent field; the older
    //    field carries a stale head position that misdirects the host's
    //    correction. These cameras are bottom-field-first (BFF): the bottom field
    //    = odd rows (1,3,5) is captured FIRST (older); the top field = even rows
    //    (0,2,4) is the most recent. So under motion we report the EVEN field.
    //    (In normal operation this branch is rare -- the host only reads vision
    //    at a move endpoint, so frames are almost always settled.)
    if (re.found && ro.found) {
        const double dx = re.cx - ro.cx, dy = re.cy - ro.cy;
        if (dx * dx + dy * dy <= 4.0 * 4.0) {
            r.found = true;
            r.cx = 0.5 * (re.cx + ro.cx);
            r.cy = 0.5 * (re.cy + ro.cy);
            r.radius = 0.5 * (re.radius + ro.radius);
            r.score = std::max(re.score, ro.score);
            r.quality = std::max(re.quality, ro.quality);
        } else {
            r.found = true; r.cx = re.cx; r.cy = re.cy; r.radius = re.radius;
            r.score = re.score; r.quality = re.quality;     // BFF: even is newest
        }
    } else if (re.found) {
        r.found = true; r.cx = re.cx; r.cy = re.cy; r.radius = re.radius; r.score = re.score; r.quality = re.quality;
    } else if (ro.found) {
        r.found = true; r.cx = ro.cx; r.cy = ro.cy; r.radius = ro.radius; r.score = ro.score; r.quality = ro.quality;
    }
    r.imgMean = re.imgMean;   // even-field mean (~frame brightness); no full-frame recompute
    return r;
}

MarkResult detect_circle_mark(const void* frame, int markSizePx,
                              double refX, double refY, int searchRadiusPx) {
    return detect_with_fields(frame, [&](const cv::Mat& g) {
        return detect_one_field(g, markSizePx, refX, refY, searchRadiusPx);
    });
}

} // namespace vis
