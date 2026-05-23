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

}  // namespace vis
