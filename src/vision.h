#pragma once
// Modern OpenCV-based down-vision fiducial detector.
//
// Drives the host's placement result (via the shim's GetOffset plumbing) for the
// Circular (CheckMark2) and ImageTemplate (CheckTemplate) down-vision modes. The
// detector depends only on OpenCV — no Windows headers; the GDI preview lives in
// preview.cpp — so it can be unit-tested against captured PNG frames off-target
// (see tests/).

#include <cstdint>
#include <limits>

namespace vis {

// Which overlay the preview draws for a detection: a circle (Circular and Round)
// or a square (ImageTemplate — the matched region). Kept here (no OpenCV types) so
// the detector header stays Windows/OpenCV-free.
enum class MarkShape : std::uint8_t { Circle,
                                      Square };

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
    // Interlace comb fraction (motion metric: high => fields misaligned => moving).
    double combFrac = 0;
    MarkShape shape = MarkShape::Circle;  // preview overlay style (set per detector)
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

// Round (down-vision CheckMark algo==0): contour-circularity detector (NOT Hough --
// HoughCircles is forbidden in CheckMark mode; it stays in the Circular path only).
// Feature size is gated by a fixed physical bracket (0.5..3.5mm diameter); the
// operator's Range (searchRadiusPx) is used AS-IS for the search-area gate.
// strength (1-10) scales the Canny thresholds. NON-DESTRUCTIVE.
MarkResult detect_round_mark(const void* frame, double refX = -1.0, double refY = -1.0,
                             int searchRadiusPx = 0, int strength = 5);

// Circular-symmetry detector (OpenPnP DetectCircularSymmetry, reimplemented from
// the published algorithm -- no third-party code). Finds the fiducial center by
// maximizing concentric-ring symmetry: the variance over all ring samples divided
// by the area-weighted mean per-ring variance peaks where every ring is uniform
// (a true concentric center). Threshold- and edge-free, so robust to soft/low-
// contrast/specular rings where Hough/contour break up. Diameter bracket
// [minDiaPx,maxDiaPx] (<=0 -> physical default); refX/refY/searchRadiusPx as
// detect_circle_mark. Field-aware via detect_with_fields. NON-DESTRUCTIVE.
MarkResult detect_circular_symmetry(const void* frame, double refX = -1.0, double refY = -1.0,
                                    int searchRadiusPx = 0, int minDiaPx = 0, int maxDiaPx = 0);

// Up-vision component pose (CheckComp). Unlike a fiducial (center only), a placed
// part needs full pose -- center, body size, and rotation -- which the host packs
// into GetOffset as X/Y = offset, W/H = size, A = angle (a DIFFERENT packing from
// the down-mark modes, which put the offset in W/H). The up-vision read is settled
// (the host dwells before the trigger), so motion/interlace is a minor factor; the
// hard part is shape + angle, not lock-under-motion.
struct CompResult {
    bool found = false;
    double cx = 0, cy = 0;  // detected center, image px (top-left origin, top-down, UNflipped)
    double w = 0, h = 0;    // detected body size, image px (w along the reported-angle axis)
    double angle = 0;       // part rotation, degrees, normalized to (-45, 45]
    double quality = 0;     // detector confidence (symmetry/fit score; higher = better)
    int imgW = 0, imgH = 0, imgOrigin = -1;
    bool headerOk = false;
    unsigned int frameHash = 0;
    // Which path produced the result (diagnostic; logged for offline comparison).
    enum class Method : std::uint8_t { None,
                                       Symmetry,
                                       MinAreaRect } method = Method::None;
};

// Up-vision component detector (CheckComp), reimplemented from the OpenPnP
// DetectRectlinearSymmetry idea (no third-party code): find the part angle from the
// rotated cross-section with the sharpest silhouette edges, then the center and body
// size from the symmetric edge pair on the axis-aligned projections -- threshold-
// free, so it does not share the vendor minAreaRect detector's per-package/lighting
// fragility (one bent pin or glare cannot stretch the box). Falls back to a
// thresholded-silhouette minAreaRect fit when the symmetry confidence is low (e.g.
// genuinely asymmetric parts). OpenCV-only (no Windows headers) so it unit-tests
// off-target against captured frames.
//
// Priors from the host (used to seed/disambiguate -- pass <=0 / NaN when absent):
//   expectedWpx, expectedHpx: expected body size in image px (from SetCompSizeWHA),
//   expectedAngleDeg:         expected rotation in degrees (NaN = no prior).
// threshold:      the host Comp Threshold (SetThreshold); <=0 -> built-in default.
// refX/refY:      where the part center is expected, frame px; <0 -> image center.
// searchRadiusPx: limit the center search to this radius about (refX,refY); <=0 ->
//                 default. NON-DESTRUCTIVE: only reads the frame buffer.
CompResult detect_component(const void* frame,
                            double expectedWpx = 0.0, double expectedHpx = 0.0,
                            double expectedAngleDeg = std::numeric_limits<double>::quiet_NaN(),
                            int threshold = 0,
                            double refX = -1.0, double refY = -1.0,
                            int searchRadiusPx = 0);

// CompThre profile-slot introspection. Operator-typed values 0-9 are
// reserved as detector-profile slots (0=AUTO sentinel, 1-9 reserved for
// per-board fills as real PCBs demand them; today all 1-9 are empty
// and fall through to AUTO). 10-100 stay the manual %-of-max threshold.
// profile_name(i) returns "AUTO" for slot 0, nullptr for an empty slot,
// or the profile's name once authored. comp_profile_count is the size
// of the registry (always 10).
const char* comp_profile_name(int slot);
int comp_profile_count();

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

// Render the up-vision component preview ourselves (replaces the original
// CheckComp's rendering): a 1:1 crop centered on the frame center (the nozzle
// reference) with a red reference crosshair and, when the part is found, an
// OpenPnP-style green overlay -- the oriented body box, a center cross, and a
// direction arrow poking out one edge so the rotation is obvious. All graphics
// are drawn in OpenCV; only the final blit touches GDI (shared with
// render_preview). Returns false on any failure so the caller can fall back to
// the original. NON-DESTRUCTIVE on the frame buffer.
bool render_comp_preview(const void* frame, void* hwnd, const CompResult& cr);

// One-shot overlay-PNG snapshot handoff. CheckComp calls request_overlay_snap()
// with a fresh frame index after consuming cap::consume_snap() and writing the
// raw frame; the next render_comp_preview save the cropped overlay as
// snap_overlay_<idx>.png (separate from the bulk overlay_<idx>.png stream).
// Best-effort: if render_comp_preview doesn't run this round (e.g. host hwnd
// is null or comp is disabled by a sentinel race), the staged idx is dropped
// at the end of CheckComp via release_overlay_snap().
void request_overlay_snap(int idx);
void release_overlay_snap();

}  // namespace vis
