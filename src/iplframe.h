#pragma once
// Shared helpers for reading the host's OpenCV-2.4 IplImage* frames.
//
// The native QueryFrame hands back an OpenCV 2.4 IplImage*. Its struct layout is
// ABI-frozen, so OpenCV 4's IplImage matches it; we still validate the header
// before trusting any field. This is the SINGLE source of truth for that
// validation — every code path that touches a frame buffer (hash, size, wrap,
// detect, preview) goes through as_valid_ipl so the checks can never drift apart.

#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>  // IplImage (frozen ABI, matches OpenCV 2.4)

namespace vis::detail {

// Validate that `frame` is an 8-bit IplImage we can handle. Returns the typed
// pointer, or nullptr if the header is missing/unsupported. Reads the header
// only; never dereferences the pixel buffer.
inline const IplImage* as_valid_ipl(const void* frame) {
    if (!frame) return nullptr;
    const IplImage* ipl = reinterpret_cast<const IplImage*>(frame);
    if (ipl->nSize != sizeof(IplImage)) return nullptr;
    if (ipl->width <= 0 || ipl->height <= 0) return nullptr;
    if (ipl->width > 8192 || ipl->height > 8192) return nullptr;
    if (ipl->nChannels != 1 && ipl->nChannels != 3) return nullptr;
    if (ipl->depth != IPL_DEPTH_8U) return nullptr;
    if (!ipl->imageData) return nullptr;
    return ipl;
}

// Wrap a validated IplImage's pixel buffer in a cv::Mat (no copy). On failure
// returns an empty Mat and sets origin = -1; on success origin is the IplImage
// origin (0 = top-left/top-down, 1 = bottom-left/bottom-up). Reads only — never
// modifies the buffer.
inline cv::Mat wrap_ipl(const void* frame, int& origin) {
    origin = -1;
    const IplImage* ipl = as_valid_ipl(frame);
    if (!ipl) return {};
    origin = ipl->origin;
    return cv::Mat(ipl->height, ipl->width, CV_MAKETYPE(CV_8U, ipl->nChannels),
                   ipl->imageData, static_cast<size_t>(ipl->widthStep));
}

}  // namespace vis::detail
