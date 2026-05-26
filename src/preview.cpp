// preview.cpp — the down-vision preview overlay we render ourselves (replaces
// the original CheckMark2/CheckTemplate rendering).
//
// This is the ONLY part of the vision module that touches the Windows API (GDI),
// so it lives apart from vision.cpp — that keeps the detector itself dependent on
// OpenCV alone, so it can be unit-tested off-target (see tests/).

#include "vision.h"

#include "capture.h"
#include "controller.h"
#include "detect_common.h"
#include "iplframe.h"
#include "settings.h"

#include <algorithm>
#include <format>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
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

// Draw 0.25 mm tick marks along the centered crosshair from a camera's px/mm scale
// (anisotropic: X scale spaces the horizontal ticks, Y the vertical). Length tiers
// 1.0 mm > 0.5 mm > 0.25 mm. No-op until the controller scale read has landed. Shared
// by the down (down_cam_scale) and up (up_cam_scale) previews.
void draw_mm_ticks(cv::Mat& work, int cw, int ch, const CamScale& sc, int thickness,
                   int cxOverride = -1, int cyOverride = -1) {
    if (!sc.valid || sc.xMmPerPx <= 0.0 || sc.yMmPerPx <= 0.0) {
        return;
    }
    const cv::Scalar red(0, 0, 255);
    const int cxp = (cxOverride >= 0) ? cxOverride : cw / 2;
    const int cyp = (cyOverride >= 0) ? cyOverride : ch / 2;
    const double pxQX = 0.25 / sc.xMmPerPx;  // px per 0.25 mm, horizontal
    const double pxQY = 0.25 / sc.yMmPerPx;  // px per 0.25 mm, vertical
    const auto tick_len = [](int n) { return (n % 4 == 0) ? 9 : (n % 2 == 0) ? 6
                                                                             : 3; };
    for (int n = 1; n < 1000; ++n) {
        const int d = static_cast<int>(std::lround(n * pxQX));
        if (cxp - d < 0 && cxp + d >= cw) {
            break;
        }
        const int len = tick_len(n);
        for (int sgn = -1; sgn <= 1; sgn += 2) {
            const int x = cxp + sgn * d;
            if (x >= 0 && x < cw) {
                cv::line(work, cv::Point(x, cyp - len), cv::Point(x, cyp + len), red, thickness, cv::LINE_AA);
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
                cv::line(work, cv::Point(cxp - len, y), cv::Point(cxp + len, y), red, thickness, cv::LINE_AA);
            }
        }
    }
}

// "Ctrl+Alt+M: settings" discoverability hint, top-right (black outline + light fill so
// it reads on any background). Clear of the host's bottom stats + top-left Switch button.
void draw_settings_hint(cv::Mat& work, int cw) {
    const char* hint = "Ctrl+Alt+M: settings";
    int baseline = 0;
    const cv::Size ts = cv::getTextSize(hint, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
    int hx = cw - ts.width - 6;
    hx = std::max(hx, 6);
    const cv::Point at(hx, ts.height + 6);
    cv::putText(work, hint, at, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
    cv::putText(work, hint, at, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
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

        // Tick marks along the crosshair every 0.25 mm (down-camera px/mm scale).
        draw_mm_ticks(work, cw, ch, down_cam_scale(), RT);

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

        draw_settings_hint(work, cw);  // "Ctrl+Alt+M: settings" hint, top-right

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
        // Show the frame exactly as the detector sees it (NO flip), so the overlay box at
        // the detected (cx,cy)/angle lands precisely on the part -- this is the only
        // orientation confirmed correct for the box. The display-orientation fix (if the
        // operator wants the image un-mirrored) is applied separately once verified from
        // the debug capture below; it must not move the box off the part.
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
        // Reference crosshair (red) drawn at the CALIBRATED NOZZLE POSITION in the
        // camera frame, not raw frame center. The host stores this offset on the
        // controller (keys 38/39 = Nozzle 1 mm offset from the camera reference, the
        // same value it subtracts from our reported offset to derive the placement
        // correction); we read it in controller.cpp. Without this shift the red
        // cross sat at frame center while the actual nozzle hangs ~5.6 px away,
        // making the green box LOOK like it lock onto the wrong place. With it,
        // green-on-red after the servo converges means "part centered on nozzle"
        // -- the real diagnostic. Falls back to frame center when the controller
        // read hasn't landed or the scale is missing.
        const auto upSc = up_cam_scale();
        const auto noz = nozzle1_up_offset();
        double nozDxPx = 0.0;
        double nozDyPx = 0.0;
        if (noz.valid && upSc.valid && upSc.xMmPerPx > 0 && upSc.yMmPerPx > 0) {
            nozDxPx = noz.xMm / upSc.xMmPerPx;
            nozDyPx = noz.yMm / upSc.yMmPerPx;
        }
        const int crossX = std::clamp(static_cast<int>(std::lround(cw / 2.0 + nozDxPx)), 0, cw - 1);
        const int crossY = std::clamp(static_cast<int>(std::lround(ch / 2.0 + nozDyPx)), 0, ch - 1);
        const cv::Scalar red(0, 0, 255);
        cv::line(work, cv::Point(crossX, 0), cv::Point(crossX, ch), red, RT, cv::LINE_AA);
        cv::line(work, cv::Point(0, crossY), cv::Point(cw, crossY), red, RT, cv::LINE_AA);
        // 0.25 mm tick marks along the crosshair, from the UP-camera px/mm scale.
        draw_mm_ticks(work, cw, ch, upSc, RT, crossX, crossY);

        // Detected part (green = locked): oriented body box + center cross + a direction
        // arrow poking past one edge so the rotation reads at a glance (OpenPnP's
        // DrawRotatedRects style). Center maps straight in (no flip). The box angle is
        // NEGATED: cv::RotatedRect's angle convention is the opposite sign of the
        // detector's (and the host's, which places correctly with cr.angle as-is) -- a
        // sign error is invisible at 0/45 deg (a near-square draws the same box either
        // way) but tilts the box the wrong way at intermediate angles. We negate only the
        // OVERLAY; cr.angle that drives placement is untouched.
        if (cr.found) {
            // Sub-pixel-aware drawing. cv::line/arrowedLine accept a `shift` arg --
            // every point coord is interpreted as `coord / (1 << shift)`. With
            // shift=4 the effective resolution is 1/16 px, killing the
            // pixel-snap shimmer when the detection wiggles sub-pixel across
            // frames. cv::drawMarker doesn't take shift, so the center cross
            // is two cv::line calls instead.
            constexpr int kShift = 4;
            constexpr double kSub = static_cast<double>(1 << kShift);
            const auto sp = [&](double x, double y) {
                return cv::Point(cvRound(x * kSub), cvRound(y * kSub));
            };
            const cv::Scalar green(0, 255, 0);
            const double u = cr.cx - cropLeft;
            const double v = cr.cy - cropTop;
            const cv::RotatedRect rr(cv::Point2f(static_cast<float>(u), static_cast<float>(v)),
                                     cv::Size2f(static_cast<float>(cr.w), static_cast<float>(cr.h)),
                                     static_cast<float>(-cr.angle));
            std::array<cv::Point2f, 4> p{};
            rr.points(p.data());
            for (int i = 0; i < 4; ++i) {
                cv::line(work, sp(p[i].x, p[i].y), sp(p[(i + 1) % 4].x, p[(i + 1) % 4].y),
                         green, RT, cv::LINE_AA, kShift);
            }
            // Arrow along the height axis (box angle - 90 deg), 1.3x half-height out.
            const double ma = (-cr.angle - 90.0) * CV_PI / 180.0;
            const double tipX = u + 1.3 * cr.h / 2.0 * std::cos(ma);
            const double tipY = v + 1.3 * cr.h / 2.0 * std::sin(ma);
            cv::arrowedLine(work, sp(u, v), sp(tipX, tipY), green, RT, cv::LINE_AA, kShift, 0.3);
            // Center cross as two sub-pixel lines (drawMarker takes no shift).
            constexpr double kCrossHalf = 5.0;
            cv::line(work, sp(u - kCrossHalf, v), sp(u + kCrossHalf, v), green, RT, cv::LINE_AA, kShift);
            cv::line(work, sp(u, v - kCrossHalf), sp(u, v + kCrossHalf), green, RT, cv::LINE_AA, kShift);
        }

        draw_settings_hint(work, cw);  // "Ctrl+Alt+M: settings" hint, top-right

        if (!work.isContinuous()) {
            work = work.clone();
        }

        // Save the rendered overlay (red cross + green box + ticks + arrow) as a
        // PNG when capture is armed. Gated by `frames` like the raw-frame saves;
        // gives a per-read visual record of green-on-red alignment for offline
        // calibration review (e.g. confirming that a 0.1 mm cx offset between
        // green and red is consistent across all parts -> nozzle calibration
        // issue, not detector wobble).
        if (cap::armed() && cap::frames_enabled()) {
            try {
                const int idx = cap::next_index();
                if (idx >= 0) {
                    cv::imwrite(std::format("{}\\overlay_{:04d}.png", cap::dir(), idx), work);
                }
            } catch (...) {  // NOLINT(bugprone-empty-catch)
            }
        }
    } catch (...) {
        return false;
    }

    return blit_to_hwnd(hwnd, work, cw, ch, rw, rh);
}

}  // namespace vis
