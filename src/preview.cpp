// preview.cpp — the down-vision preview overlay we render ourselves (replaces
// the original CheckMark2/CheckTemplate rendering).
//
// This is the ONLY part of the vision module that touches the Windows API (GDI),
// so it lives apart from vision.cpp — that keeps the detector itself dependent on
// OpenCV alone, so it can be unit-tested off-target (see tests/).

#include "vision.h"

#include "controller.h"
#include "detect_common.h"
#include "iplframe.h"
#include "settings.h"

#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#define NOMINMAX      // keep std::min/std::max usable
#include <windows.h>  // GDI for self-render (otherwise defines min/max macros)

#include <algorithm>
#include <array>
#include <cmath>

namespace vis {

namespace {

// Blit a continuous CV_8UC3 (cw x ch, cw a multiple of 4 so the cw*3 row stride is
// DIB-aligned) 1:1 to the control's window, black-filling any border when the window
// is larger than the native crop. This is the ONLY GDI in the module -- all overlay
// graphics are drawn in OpenCV beforehand. No OpenCV calls happen between GetDC and
// ReleaseDC, so the DC cannot leak; the caller has already finished a no-throw build
// of `work`, so nothing unwinds across the C ABI here either.
bool blit_to_hwnd(HWND hwnd, const cv::Mat& work, int cw, int ch, int rw, int rh) {
    HDC dc = GetDC(hwnd);
    if (dc == nullptr) {
        return false;
    }
    BITMAPINFO bmi;
    ZeroMemory(&bmi, sizeof(bmi));
    bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth = cw;
    bmi.bmiHeader.biHeight = -ch;  // negative = top-down (matches cv::Mat)
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 24;
    bmi.bmiHeader.biCompression = BI_RGB;
    SetStretchBltMode(dc, COLORONCOLOR);
    StretchDIBits(dc, 0, 0, cw, ch, 0, 0, cw, ch, work.data, &bmi, DIB_RGB_COLORS, SRCCOPY);
    auto black = static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH));
    if (rw > cw) {
        const RECT s{cw, 0, rw, rh};
        FillRect(dc, &s, black);
    }
    if (rh > ch) {
        const RECT s{0, ch, cw, rh};
        FillRect(dc, &s, black);
    }
    ReleaseDC(hwnd, dc);
    return true;
}

}  // namespace

bool render_preview(const void* frame, void* hwndV, double mvoX, double mvoY,
                    int searchRadiusPx, const MarkResult& mr) {
    if (hwndV == nullptr) {
        return false;
    }
    HWND hwnd = reinterpret_cast<HWND>(hwndV);

    // Build the final overlay image FIRST. ALL OpenCV work happens here, before
    // we acquire the device context below, so (a) a throw can never leak the DC
    // and (b) the whole body is a no-throw barrier — the host calls us across a
    // plain C ABI, where an unwinding exception would be undefined behavior.
    cv::Mat work;
    int cw = 0;
    int ch = 0;
    int rw = 0;
    int rh = 0;
    try {
        int origin = -1;
        const cv::Mat wrapped = detail::wrap_ipl(frame, origin);
        if (wrapped.empty()) {
            return false;
        }

        // ORIGIN: ignore the flag (the down-vision camera toggles it spuriously) and
        // treat the frame top-down, so the preview matches detection. TODO(origin):
        // not validated on this hardware -- revisit after the merge (see work item).
        (void)origin;
        const cv::Mat& img = wrapped;
        // Show the SAME image the detector sees: extract gray on an OWNED copy (never
        // the live buffer), apply the UI image adjustments (no-op when neutral), then
        // colorize for the overlay. So dragging gamma/contrast/levels/sharpen is
        // visible in the preview, not just in the lock/no-lock readout.
        cv::Mat gray;
        if (img.channels() == 1) {
            gray = img.clone();
        } else if (img.channels() == 3) {
            cv::extractChannel(img, gray, 0);  // mono capture: plane 0 == gray
        } else {
            return false;
        }
        const Settings pcfg = get_settings(get_status().mode);  // active mode's adjustments
        apply_image_adjustments(gray, pcfg);
        if (pcfg.blur > 0.0) {  // show an EXPLICIT pre-blur (Auto = detector default, not shown)
            const int gk = blur_kernel(pcfg.blur, 5);
            cv::GaussianBlur(gray, gray, cv::Size(gk, gk), 0);
        }
        cv::Mat bgr;
        cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);

        const int W = bgr.cols;
        const int H = bgr.rows;
        cv::Mat flipped;
        cv::flip(bgr, flipped, -1);  // 180 deg, matches the original mirror

        RECT rc{};
        GetClientRect(hwnd, &rc);  // window metrics only; no DC required
        rw = rc.right - rc.left;
        rh = rc.bottom - rc.top;
        if (rw <= 0) {
            rw = W;
        }
        if (rh <= 0) {
            rh = H;
        }

        // 1:1 NATIVE preview (no upscaling -> crisp overlay; a stretched crop
        // made the thin red lines blocky). Crop a window-sized region at native
        // pixels, centered on the reference. Cap the crop to the source minus the
        // mvo offset so we never run off the frame (no replicated border), and
        // round the width down to a multiple of 4 so the cv::Mat row stride
        // (cw*3) is DIB-aligned.
        const double cx = W / 2.0 + mvoX;
        const double cy = H / 2.0 + mvoY;  // reference, flipped coords
        const int marginW = W - 2 * static_cast<int>(std::ceil(std::abs(mvoX)));
        const int marginH = H - 2 * static_cast<int>(std::ceil(std::abs(mvoY)));
        cw = std::min(rw, marginW > 16 ? marginW : W);
        ch = std::min(rh, marginH > 16 ? marginH : H);
        cw &= ~3;  // multiple of 4 -> DIB row aligned
        if (cw < 16 || ch < 16) {
            return false;
        }

        cv::getRectSubPix(flipped, cv::Size(cw, ch),
                          cv::Point2f(static_cast<float>(cx), static_cast<float>(cy)), work);
        if (work.type() != CV_8UC3) {
            return false;
        }

        const int RT = 1;  // red overlay thickness -- 1px, antialiased on the 1:1 display
        // Reference crosshair (red) at the crop center == the reference point.
        cv::line(work, cv::Point(cw / 2, 0), cv::Point(cw / 2, ch), cv::Scalar(0, 0, 255), RT, cv::LINE_AA);
        cv::line(work, cv::Point(0, ch / 2), cv::Point(cw, ch / 2), cv::Scalar(0, 0, 255), RT, cv::LINE_AA);
        // Search-area ("Range") circle (red) — detection is limited to inside this.
        if (searchRadiusPx > 0) {
            cv::circle(work, cv::Point(cw / 2, ch / 2), searchRadiusPx, cv::Scalar(0, 0, 255), RT, cv::LINE_AA);
        }

        // Tick marks along the crosshair every 0.25 mm, from the controller's px/mm
        // scale (down camera — this preview is the down-vision mark modes). Anisotropic:
        // the X scale spaces the horizontal axis, Y the vertical. Length hierarchy:
        // 1.0 mm > 0.5 mm > 0.25 mm. Absent until the background controller read lands.
        if (const CamScale sc = down_cam_scale(); sc.valid && sc.xMmPerPx > 0.0 && sc.yMmPerPx > 0.0) {
            const cv::Scalar red(0, 0, 255);
            const int cxp = cw / 2;
            const int cyp = ch / 2;
            const double pxQX = 0.25 / sc.xMmPerPx;  // px per 0.25 mm, horizontal
            const double pxQY = 0.25 / sc.yMmPerPx;  // px per 0.25 mm, vertical
            auto tick_len = [](int n) { return (n % 4 == 0) ? 9 : (n % 2 == 0) ? 6
                                                                               : 3; };
            for (int n = 1; n < 1000; ++n) {
                const int d = static_cast<int>(std::lround(n * pxQX));
                if (cxp - d < 0 && cxp + d >= cw) {
                    break;
                }
                const int len = tick_len(n);  // 1.0 / 0.5 / 0.25 mm
                for (int sgn = -1; sgn <= 1; sgn += 2) {
                    const int x = cxp + sgn * d;
                    if (x >= 0 && x < cw) {
                        cv::line(work, cv::Point(x, cyp - len), cv::Point(x, cyp + len), red, RT, cv::LINE_AA);
                    }
                }
            }
            for (int n = 1; n < 1000; ++n) {
                const int d = static_cast<int>(std::lround(n * pxQY));
                if (cyp - d < 0 && cyp + d >= ch) {
                    break;
                }
                const int len = tick_len(n);
                for (int sgn = -1; sgn <= 1; sgn += 2) {
                    const int y = cyp + sgn * d;
                    if (y >= 0 && y < ch) {
                        cv::line(work, cv::Point(cxp - len, y), cv::Point(cxp + len, y), red, RT, cv::LINE_AA);
                    }
                }
            }
        }

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
            if (mr.shape == MarkShape::Square) {
                cv::rectangle(work, cv::Point(u - rad, v - rad), cv::Point(u + rad, v + rad), green, 1, cv::LINE_AA);
            } else {
                cv::circle(work, cv::Point(u, v), rad, green, 1, cv::LINE_AA);
            }
            cv::drawMarker(work, cv::Point(u, v), green, cv::MARKER_CROSS, 14, 1, cv::LINE_AA);
        }

        // Discoverability hint for the settings dialog (black outline + light text so
        // it reads on any background). TOP-RIGHT, clear of the host's stats overlay
        // (bottom) and the "Switch" camera button (top-left).
        {
            const char* hint = "Ctrl+Alt+M: settings";
            int baseline = 0;
            const cv::Size ts = cv::getTextSize(hint, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
            int hx = cw - ts.width - 6;
            hx = std::max(hx, 6);
            const cv::Point at(hx, ts.height + 6);
            cv::putText(work, hint, at, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
            cv::putText(work, hint, at, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
        }

        if (!work.isContinuous()) {
            work = work.clone();
        }
    } catch (...) {
        return false;  // never let an OpenCV failure reach the host
    }

    return blit_to_hwnd(hwnd, work, cw, ch, rw, rh);
}

bool render_comp_preview(const void* frame, void* hwndV, const CompResult& cr) {
    if (hwndV == nullptr) {
        return false;
    }
    HWND hwnd = reinterpret_cast<HWND>(hwndV);

    // Build the overlay first (all OpenCV, no-throw barrier), then blit. Same shape
    // as render_preview: a throw can't leak the DC, and nothing unwinds across the
    // C ABI the host calls us through.
    cv::Mat work;
    int cw = 0;
    int ch = 0;
    int rw = 0;
    int rh = 0;
    try {
        int origin = -1;
        const cv::Mat wrapped = detail::wrap_ipl(frame, origin);
        if (wrapped.empty()) {
            return false;
        }
        // Up camera: detection/offset use the UNflipped buffer (calibrated -- our center
        // matches the original's reported offset, no mirror). But the host DISPLAYS the
        // up view horizontally mirrored, so mirror the PREVIEW to match it, and map the
        // overlay through the same flip below so the box still lands on the part.
        (void)origin;
        cv::Mat gray;
        if (wrapped.channels() == 1) {
            gray = wrapped.clone();
        } else if (wrapped.channels() == 3) {
            cv::extractChannel(wrapped, gray, 0);  // mono capture: plane 0 == gray
        } else {
            return false;
        }
        cv::Mat bgr;
        cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
        cv::flip(bgr, bgr, 1);  // 1 = horizontal mirror, to match the host's up display

        const int W = bgr.cols;
        const int H = bgr.rows;
        RECT rc{};
        GetClientRect(hwnd, &rc);
        rw = rc.right - rc.left;
        rh = rc.bottom - rc.top;
        if (rw <= 0) {
            rw = W;
        }
        if (rh <= 0) {
            rh = H;
        }

        // 1:1 native crop centered on the frame center == the nozzle reference (the
        // part rides the nozzle near center). cw rounded to a multiple of 4 for DIB.
        const double rx = W / 2.0;
        const double ry = H / 2.0;
        cw = std::min(rw, W) & ~3;
        ch = std::min(rh, H);
        if (cw < 16 || ch < 16) {
            return false;
        }
        cv::getRectSubPix(bgr, cv::Size(cw, ch),
                          cv::Point2f(static_cast<float>(rx), static_cast<float>(ry)), work);
        if (work.type() != CV_8UC3) {
            return false;
        }
        const double cropLeft = rx - cw / 2.0;
        const double cropTop = ry - ch / 2.0;

        const int RT = 1;
        // Reference crosshair (red) at the crop center = the nozzle center. The gap to
        // the green part center is the placement offset the host corrects.
        const cv::Scalar red(0, 0, 255);
        cv::line(work, cv::Point(cw / 2, 0), cv::Point(cw / 2, ch), red, RT, cv::LINE_AA);
        cv::line(work, cv::Point(0, ch / 2), cv::Point(cw, ch / 2), red, RT, cv::LINE_AA);

        // Detected part (green = locked): oriented body box + center cross + a direction
        // arrow poking past one edge so the rotation reads at a glance (OpenPnP's
        // DrawRotatedRects style). The preview is horizontally mirrored above, so map the
        // detection through the same flip: x -> W - cx, and a horizontal mirror negates
        // the angle (the detector reports it in OpenCV's RotatedRect convention on the
        // unflipped buffer; validated against captured frames + detection).
        if (cr.found) {
            const cv::Scalar green(0, 255, 0);
            const auto u = static_cast<float>((static_cast<double>(W) - cr.cx) - cropLeft);
            const auto v = static_cast<float>(cr.cy - cropTop);
            const cv::RotatedRect rr(cv::Point2f(u, v),
                                     cv::Size2f(static_cast<float>(cr.w), static_cast<float>(cr.h)),
                                     static_cast<float>(-cr.angle));
            std::array<cv::Point2f, 4> p{};
            rr.points(p.data());
            for (int i = 0; i < 4; ++i) {
                cv::line(work, p[i], p[(i + 1) % 4], green, RT, cv::LINE_AA);
            }
            // Arrow along the height axis (angle - 90 deg), 1.3x half-height out.
            const double ma = (-cr.angle - 90.0) * CV_PI / 180.0;
            const cv::Point center(cvRound(u), cvRound(v));
            const cv::Point tip(cvRound(u + 1.3 * cr.h / 2.0 * std::cos(ma)),
                                cvRound(v + 1.3 * cr.h / 2.0 * std::sin(ma)));
            cv::arrowedLine(work, center, tip, green, RT, cv::LINE_AA, 0, 0.3);
            cv::drawMarker(work, center, green, cv::MARKER_CROSS, 10, RT, cv::LINE_AA);
        }

        if (!work.isContinuous()) {
            work = work.clone();
        }
    } catch (...) {
        return false;
    }

    return blit_to_hwnd(hwnd, work, cw, ch, rw, rh);
}

}  // namespace vis
