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

}  // namespace vis
