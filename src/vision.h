#pragma once
// Modern OpenCV-based fiducial detector. Phase 1B.
//
// For the shadow-comparison build this runs alongside the original CheckMark
// and only logs; once validated it will drive the result. Kept free of the
// rest of the DLL so it can be unit-tested against captured BMP frames offline.

namespace vis {

struct MarkResult {
    bool   found  = false;
    double cx     = 0;   // detected center, image pixels, top-left origin, top-down
    double cy     = 0;
    double radius = 0;
    double score  = 0;   // 0..1 proximity-to-reference (higher = nearer)
    double quality = 0;  // 0..1 detection quality (circ. edge-support / match corr)
    double imgMean = 0;  // frame brightness; near-0 => dropped/black frame
    // Parsed IplImage header fields (for format diagnostics in the log).
    int    imgW = 0, imgH = 0, imgChannels = 0, imgOrigin = -1, imgStep = 0;
    bool   headerOk = false;
};

// frame: an OpenCV 2.4 IplImage* (the native QueryFrame return). We parse the
// header to find the real pixel buffer rather than assuming a raw layout.
// markSizePx: the configured template size in px (host passes this as the
//   CheckMark2 size arg, derived from the "1.2mm" setting). Constrains the
//   detection radius so we lock the dot, not the larger solder-mask ring.
//   <=0 falls back to a default radius bracket.
// refX/refY: preferred center (frame px) for choosing among multiple circles —
//   pass the reference point (where the mark sits when aligned). Negative = use
//   image center.
// searchRadiusPx: reject detections farther than this from (refX,refY) — the
//   "Range" search-area constraint that suppresses stray circles. <=0 = no limit.
// NON-DESTRUCTIVE: only reads the frame buffer.
MarkResult detect_circle_mark(const void* frame, int markSizePx,
                              double refX = -1.0, double refY = -1.0,
                              int searchRadiusPx = 0);

// Save an IplImage* frame to `path` (PNG) without modifying the buffer — we
// must NOT use the original mySaveImage, which flips the buffer in place.
// Returns true on success.
bool save_frame(const void* frame, const char* path);

// Render the down-vision preview ourselves (replaces the original CheckMark2's
// rendering): 180-deg-mirrored 540x460 crop centered at (imgW/2+mvoX,
// imgH/2+mvoY), red reference crosshair, and our green detection overlay,
// StretchDIBits'd to the control's window. Returns false on any failure so the
// caller can fall back to the original. NON-DESTRUCTIVE on the frame buffer.
bool render_preview(const void* frame, void* hwnd, double mvoX, double mvoY,
                    int searchRadiusPx, const MarkResult& mr);

} // namespace vis
