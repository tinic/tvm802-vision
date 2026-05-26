#pragma once
// Capture / shadow-comparison instrumentation (Phase 1B).
//
// Opt-in via a trigger file so it never fires in normal use:
//   create  C:\mvision_capture\on   to start; delete it (or hit the cap) to stop.
//
// IMPORTANT: frame saving is done by vis::save_frame (OpenCV, non-destructive).
// We must NOT use the original mySaveImage — it flips the live IplImage buffer
// in place, which corrupts the frame the original CheckMark then processes.

#include <string_view>

namespace cap {

// True if capture is armed (trigger file present and frame cap not reached).
bool armed();

// True if PNG frame saving is enabled (separate, heavier trigger:
// C:\mvision_capture\frames). compare.log is always written while armed(); image
// saving is opt-in ON TOP, because imwrite adds latency to the detection path
// and can hurt closed-loop responsiveness. Arm 'on' alone for log-only runs.
bool frames_enabled();

// True if the up-vision component (CheckComp) PROTOTYPE is enabled (trigger:
// C:\mvision_capture\comp). It gates the WHOLE prototype: with the sentinel absent
// (the shipped default) CheckComp is a pure passthrough to the original -- our
// detector never even runs. Create the file to opt in to the shadow detector + its
// logging. Kept separate from 'on' so a released DLL can carry the prototype inert.
bool comp_enabled();

// Reserve the next frame index (atomic, capped). Returns the index or -1 when
// the per-session cap is reached.
int next_index();

// Directory where captures are written (no trailing slash).
const char* dir();

// Append a line to C:\mvision_capture\compare.log.
void log_line(std::string_view line);

// One-shot up-cam snapshot. The settings-UI thread arms it on Ctrl+Alt+U;
// the next CheckComp consumes the flag and writes snap_NNNN.png + (best-
// effort) snap_overlay_NNNN.png. Atomic exchange -- one press, one capture,
// regardless of whether the bulk-capture sentinels (on / comp / frames)
// are present. The comp sentinel is still needed for the OVERLAY save
// (overlay only renders when the prototype detector is active); raw-frame
// snap_NNNN.png fires unconditionally.
void arm_snap();
bool consume_snap();

}  // namespace cap
