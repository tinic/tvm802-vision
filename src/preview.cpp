// preview.cpp — the down-vision preview overlay we render ourselves (replaces
// the original CheckMark2/CheckTemplate rendering).
//
// This is the ONLY part of the vision module that touches the Windows API (GDI),
// so it lives apart from vision.cpp — that keeps the detector itself dependent on
// OpenCV alone, so it can be unit-tested off-target (see tests/).

#include "vision.h"

#include "detect_common.h"
#include "iplframe.h"
#include "settings.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#define NOMINMAX      // keep std::min/std::max usable
#include <windows.h>  // GDI for self-render (otherwise defines min/max macros)

#include <algorithm>
#include <cmath>

namespace vis {

bool render_preview(const void* frame, void* hwndV, double mvoX, double mvoY,
                    int searchRadiusPx, const MarkResult& mr) {
    if (!hwndV) return false;
    HWND hwnd = reinterpret_cast<HWND>(hwndV);

    // Build the final overlay image FIRST. ALL OpenCV work happens here, before
    // we acquire the device context below, so (a) a throw can never leak the DC
    // and (b) the whole body is a no-throw barrier — the host calls us across a
    // plain C ABI, where an unwinding exception would be undefined behavior.
    cv::Mat work;
    int cw = 0, ch = 0, rw = 0, rh = 0;
    try {
        int origin = -1;
        cv::Mat wrapped = detail::wrap_ipl(frame, origin);
        if (wrapped.empty()) return false;

        // ORIGIN: ignore the flag (the down-vision camera toggles it spuriously) and
        // treat the frame top-down, so the preview matches detection. TODO(origin):
        // not validated on this hardware -- revisit after the merge (see work item).
        (void)origin;
        cv::Mat img = wrapped;
        // Show the SAME image the detector sees: extract gray on an OWNED copy (never
        // the live buffer), apply the UI image adjustments (no-op when neutral), then
        // colorize for the overlay. So dragging gamma/contrast/levels/sharpen is
        // visible in the preview, not just in the lock/no-lock readout.
        cv::Mat gray;
        if (img.channels() == 1)
            gray = img.clone();
        else if (img.channels() == 3)
            cv::extractChannel(img, gray, 0);  // mono capture: plane 0 == gray
        else
            return false;
        const Settings pcfg = get_settings(get_status().mode);  // active mode's adjustments
        apply_image_adjustments(gray, pcfg);
        if (pcfg.blur > 0.0) {  // show an EXPLICIT pre-blur (Auto = detector default, not shown)
            const int gk = blur_kernel(pcfg.blur, 5);
            cv::GaussianBlur(gray, gray, cv::Size(gk, gk), 0);
        }
        cv::Mat bgr;
        cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);

        const int W = bgr.cols, H = bgr.rows;
        cv::Mat flipped;
        cv::flip(bgr, flipped, -1);  // 180 deg, matches the original mirror

        RECT rc{};
        GetClientRect(hwnd, &rc);  // window metrics only; no DC required
        rw = rc.right - rc.left;
        rh = rc.bottom - rc.top;
        if (rw <= 0) rw = W;
        if (rh <= 0) rh = H;

        // 1:1 NATIVE preview (no upscaling -> crisp overlay; a stretched crop
        // made the thin red lines blocky). Crop a window-sized region at native
        // pixels, centered on the reference. Cap the crop to the source minus the
        // mvo offset so we never run off the frame (no replicated border), and
        // round the width down to a multiple of 4 so the cv::Mat row stride
        // (cw*3) is DIB-aligned.
        const double cx = W / 2.0 + mvoX, cy = H / 2.0 + mvoY;  // reference, flipped coords
        int marginW = W - 2 * static_cast<int>(std::ceil(std::abs(mvoX)));
        int marginH = H - 2 * static_cast<int>(std::ceil(std::abs(mvoY)));
        cw = std::min(rw, marginW > 16 ? marginW : W);
        ch = std::min(rh, marginH > 16 ? marginH : H);
        cw &= ~3;  // multiple of 4 -> DIB row aligned
        if (cw < 16 || ch < 16) return false;

        cv::getRectSubPix(flipped, cv::Size(cw, ch),
                          cv::Point2f((float)cx, (float)cy), work);
        if (work.type() != CV_8UC3) return false;

        const int RT = 2;  // red overlay thickness (crisp at 1:1, more visible)
        // Reference crosshair (red) at the crop center == the reference point.
        cv::line(work, cv::Point(cw / 2, 0), cv::Point(cw / 2, ch), cv::Scalar(0, 0, 255), RT);
        cv::line(work, cv::Point(0, ch / 2), cv::Point(cw, ch / 2), cv::Scalar(0, 0, 255), RT);
        // Search-area ("Range") circle (red) — detection is limited to inside this.
        if (searchRadiusPx > 0)
            cv::circle(work, cv::Point(cw / 2, ch / 2), searchRadiusPx, cv::Scalar(0, 0, 255), RT);

        // Our detection (green) mapped from frame coords into crop coords. The
        // overlay style depends on the mode: a square for ImageTemplate (the
        // matched region), else a circle (Circular and Round).
        if (mr.found) {
            const double cropLeft = cx - cw / 2.0;
            const double cropTop = cy - ch / 2.0;
            const int u = cvRound((W - mr.cx) - cropLeft);
            const int v = cvRound((H - mr.cy) - cropTop);
            const int rad = cvRound(mr.radius);
            const cv::Scalar green(0, 255, 0);
            if (mr.shape == MarkShape::Square)
                cv::rectangle(work, cv::Point(u - rad, v - rad), cv::Point(u + rad, v + rad), green, 2);
            else
                cv::circle(work, cv::Point(u, v), rad, green, 2);
            cv::drawMarker(work, cv::Point(u, v), green, cv::MARKER_CROSS, 14, 2);
        }

        // Discoverability hint for the settings dialog (black outline + light text so
        // it reads on any background). Bottom-left of the crop.
        {
            const cv::Point at(6, ch - 8);
            cv::putText(work, "Ctrl+Alt+M: settings", at, cv::FONT_HERSHEY_SIMPLEX, 0.4,
                        cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
            cv::putText(work, "Ctrl+Alt+M: settings", at, cv::FONT_HERSHEY_SIMPLEX, 0.4,
                        cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
        }

        if (!work.isContinuous()) work = work.clone();
    } catch (...) {
        return false;  // never let an OpenCV failure reach the host
    }

    // GDI section: NO OpenCV calls between GetDC and ReleaseDC, so the DC cannot
    // leak. `work` is a continuous CV_8UC3 of size cw x ch (cw is a multiple of
    // 4, so the cw*3 row stride is DIB-aligned).
    HDC dc = GetDC(hwnd);
    if (!dc) return false;
    BITMAPINFO bmi;
    ZeroMemory(&bmi, sizeof(bmi));
    bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth = cw;
    bmi.bmiHeader.biHeight = -ch;  // negative = top-down (matches cv::Mat)
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 24;
    bmi.bmiHeader.biCompression = BI_RGB;
    // 1:1 blit (dst size == src size -> no scaling). Top-left; black-fill any
    // border strips (only when the window is larger than the native crop).
    SetStretchBltMode(dc, COLORONCOLOR);
    StretchDIBits(dc, 0, 0, cw, ch, 0, 0, cw, ch, work.data, &bmi, DIB_RGB_COLORS, SRCCOPY);
    HBRUSH black = (HBRUSH)GetStockObject(BLACK_BRUSH);
    if (rw > cw) {
        RECT s{cw, 0, rw, rh};
        FillRect(dc, &s, black);
    }
    if (rh > ch) {
        RECT s{0, ch, cw, rh};
        FillRect(dc, &s, black);
    }
    ReleaseDC(hwnd, dc);
    return true;
}

}  // namespace vis
