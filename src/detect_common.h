#pragma once

#include "vision.h"

#include <opencv2/core.hpp>

#include <functional>

namespace vis {

// Field-aware wrapper shared by every detector. Deinterlaces the analog frame
// into its two ~1/60s-apart fields (even/odd scanlines), bobs each back to full
// height, runs `perField` on each, and combines: settled -> two-field average
// (halves sub-pixel scatter), moving -> only the newest field (BFF, no comb).
// `perField` returns cx in full-frame x and cy in field-row space; this wrapper
// applies the +-0.5 row correction. wovenWhenSettled runs the sharp full-res
// woven frame when settled (the Round contour detector wants un-softened edges).
// Also fills the MarkResult header/hash/comb diagnostics. NON-DESTRUCTIVE.
// Internal API (not part of the public detector surface in vision.h).
MarkResult detect_with_fields(const void* frame,
                              const std::function<MarkResult(const cv::Mat&)>& perField,
                              bool wovenWhenSettled = false);

// Interlace-aware source selection for "woven-when-settled" detectors that own their
// own ROI/crop (e.g. up-vision CheckComp) and so can't use detect_with_fields. Given
// the full woven gray, returns the image to detect on: the sharp woven frame when the
// scene is settled, else the newest single-instant field bobbed to full height (no
// interlace comb under motion -- e.g. a part rotating during placement). *cyCorrection
// is the value to ADD to a detected cy to map field-row space back to full-frame y
// (0 for the woven frame, -0.5 for the newest field). NON-DESTRUCTIVE on `gray`.
cv::Mat motion_aware_gray(const cv::Mat& gray, double* cyCorrection);

struct Settings;  // settings.h

// Apply the image adjustments (gamma / levels / brightness / contrast / sharpen)
// in place to a single-channel 8U ROI before detection. The four point ops compose
// into ONE 256-entry LUT (one pass); sharpen/blur is a 3x3 spatial op after. Every
// field is a no-op at its neutral value, so a default Settings leaves `g` untouched.
// A grayscale subset of the png2amiga preprocess pipeline.
void apply_image_adjustments(cv::Mat& g, const Settings& s);

// Effective Gaussian pre-blur kernel: the settings value (rounded to an odd size,
// clamped to [3,25]) when > 0, else the detector's per-mode default. settings 0 =
// "Auto". Larger smooths a speckled/specular surface so concentric features read
// uniformly to the detector.
int blur_kernel(double setting, int dflt);

}  // namespace vis
