#pragma once
// Modern OpenCV-based down-vision fiducial detector.
//
// Drives the host's placement result (via the shim's GetOffset plumbing) for the
// Circular (CheckMark2) and ImageTemplate (CheckTemplate) down-vision modes. The
// detector depends only on OpenCV — no Windows headers; the GDI preview lives in
// preview.cpp — so it can be unit-tested against captured PNG frames off-target
// (see tests/).

namespace vis {

struct MarkResult {
    bool found = false;
    double cx = 0;  // detected center, image pixels, top-left origin, top-down
    double cy = 0;
    double radius = 0;
    // Detection quality: circularity edge-support fraction (higher = better) for
    // Circular, or the SQDIFF match value (lower = better) for ImageTemplate.
    // Diagnostic only — surfaced by the test harness; not used to drive placement.
    double quality = 0;
    // Parsed IplImage header fields (for format diagnostics in the log).
    int imgW = 0, imgH = 0, imgOrigin = -1;
    bool headerOk = false;
    // Sparse checksum of the raw frame buffer (stale-frame diagnostic in the log).
    unsigned int frameHash = 0;
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

// Cheap sparse FNV-1a hash of an IplImage* frame buffer (samples a grid). Two
// reads with the same hash are (almost certainly) the same camera frame — used
// to detect stale/duplicate frames from QueryFrame. Returns 0 for an invalid or
// null frame; any valid frame hashes to non-zero.
unsigned int frame_hash(const void* frame);

// Read the frame's pixel dimensions from its IplImage header (no buffer access).
// Returns false and leaves *w/*h untouched if the header is missing/unsupported.
bool frame_size(const void* frame, int* w, int* h);

// Template match for the down-vision mark (ImageTemplate mode). templateBytes:
// the host's CheckTemplate buffer; plane0 (size*size, widthStep aligned to 4) is
// the gray template. Field-aware: deinterlaces and matches the (180-deg flipped)
// template against each field, restricted to a tight crop around the reference
// (same center as the Circular path). strength (1-10) sets the acceptance
// threshold. cx/cy = matched template CENTER in full-frame coords (same
// convention as detect_circle_mark). NON-DESTRUCTIVE.
// ioScale (optional): per-template scale cache. On entry, if 0.4..1.4 the matcher
// tries that scale first (single-scale, fast) and only re-sweeps if it degrades;
// on a good match it is set to the working scale. Pass a distinct double per
// cached template (dual cache) so each mark keeps its own locked scale.
MarkResult detect_template_mark(const void* frame, const unsigned char* templateBytes,
                                int size, double threshold, int strength,
                                double refX = -1.0, double refY = -1.0,
                                int searchRadiusPx = 0, double* ioScale = nullptr);

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

}  // namespace vis
