// controller.h -- read the per-camera pixel scale (mm per pixel) from the machine's
// motion controller over TCP, for drawing 0.5 mm tick marks on the preview axis.
// Read-only; the read runs on its own connection and does not disturb anything else.
#pragma once

namespace vis {

// Per-camera anisotropic px<->mm scale. valid=false until a controller read lands.
struct CamScale {
    double xMmPerPx = 0.0;
    double yMmPerPx = 0.0;
    bool valid = false;
};

// Cached scales. The first call starts a one-shot background fetch and returns
// {valid=false} until it lands; thereafter the cached value. Safe to call every
// frame from the preview thread -- never blocks, never throws.
CamScale down_cam_scale();
CamScale up_cam_scale();

// Per-nozzle calibrated offset between the up-camera reference and the actual
// nozzle position, in mm. The host uses these as a placement-correction
// subtrahend (placement_correction = our_offset_mm - nozzle_offset_mm), so they
// encode where the nozzle ACTUALLY appears in the up-cam frame relative to its
// nominal reference. Stored on the controller as keys 38/39 (nozzle 1) and
// 40/41 (nozzle 2), x1000 in millimeters. Fetched alongside the camera scales;
// `valid=false` until a controller read lands.
struct NozzleOffset {
    double xMm = 0.0;
    double yMm = 0.0;
    bool valid = false;
};
NozzleOffset nozzle1_up_offset();
NozzleOffset nozzle2_up_offset();

}  // namespace vis
