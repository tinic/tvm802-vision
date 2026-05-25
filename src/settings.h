#pragma once
// Runtime-tunable detector settings + live detection status, shared between the
// detectors (which READ settings and the shim which PUBLISHES status) and the Win32
// settings UI (which writes settings and reads status). Windows-free and OpenCV-free
// so it links into the portable detector core and the off-target test harness.

namespace vis {

// Detector overrides. A field <= 0 means "use the detector's built-in default", so a
// default-constructed Settings leaves behavior EXACTLY as shipped -- the UI only
// deviates when the operator drags a slider off its "Auto" position.
struct Settings {
    // Detection (apply to both down-vision detectors).
    double radiusMinPx = 0.0;  // fiducial ring radius bracket (Round + Circular)
    double radiusMaxPx = 0.0;
    double minSymmetry = 0.0;  // Round (circular-symmetry) accept threshold; lower = more lenient

    // Image adjustments -- a grayscale subset of the png2amiga preprocess set,
    // applied to the ROI before detection. Every field is a no-op at its neutral
    // value, so a default Settings leaves the image untouched. (Saturation/hue are
    // color-only and omitted -- our frames are monochrome.)
    double gamma = 0.0;       // 0 = off (1.0); <1 lifts shadows, >1 boosts highlights
    double brightness = 0.0;  // 8-bit add, 0 = off (-128..+128)
    double contrast = 0.0;    // 0 = off (1.0); multiply about mid-gray
    double blackPoint = 0.0;  // 8-bit levels low clip, 0 = off
    double whitePoint = 0.0;  // 8-bit levels high clip, 0 = off
    double sharpen = 0.0;     // 0 = off; >0 unsharp, <0 blur (-1..+1)
    double blur = 0.0;        // detector pre-blur Gaussian kernel px; 0 = Auto (per-mode default), else odd 3..25

    // Detection gates / scoring.
    double meanLo = 0.0;       // exposure gate: reject frame mean below this; 0 = Auto (default 6)
    double meanHi = 0.0;       // exposure gate: reject frame mean above this; 0 = Auto (default 250)
    double medianRings = 0.0;  // Round: combine rings by MEDIAN (robust to glare) instead of mean; 0 = off, 1 = on
};

// Detector modes. Settings are kept PER MODE, so each has an independent tuning.
// Round / Circular / ImageTemplate are the down-vision marks (match LiveStatus.mode);
// Component is up-vision (CheckComp). The UI's mode dropdown selects which one to edit.
// For MODE_COMP a few Settings fields are reinterpreted (the part has no fiducial
// ring): radiusMinPx = stray-guard search radius px, radiusMaxPx = stray-guard max
// part size px (0 = the detector's defaults). minSymmetry / meanLo / meanHi / blur /
// image adjustments mean the same as elsewhere.
enum { MODE_NONE = 0,
       MODE_ROUND = 1,
       MODE_CIRCULAR = 2,
       MODE_TEMPLATE = 3,
       MODE_COMP = 4,
       MODE_COUNT = 5 };

// Thread-safe per-mode snapshot / update (the UI thread writes; detector threads
// read). A mode outside [1,3] is clamped to MODE_ROUND.
Settings get_settings(int mode);
void set_settings(int mode, const Settings& s);

// Per-detector master switches: run OUR detector (true = default) or pass the call
// straight through to the original DLL (false). Global (not per-mode), exposed as the
// "Detectors" checkboxes in the settings UI and persisted in the INI [enable] section.
// Lets the operator fall back to the stock vision per method without reinstalling.
enum { METHOD_ROUND = 0,     // CheckMark    (Round / circular-symmetry)
       METHOD_CIRCULAR = 1,  // CheckMark2   (Circular / Hough)
       METHOD_TEMPLATE = 2,  // CheckTemplate (ImageTemplate)
       METHOD_COMP = 3,      // CheckComp    (up-vision component)
       METHOD_COUNT = 4 };
bool method_enabled(int method);  // default true; out-of-range -> true
void set_method_enabled(int method, bool on);

// Persistence: an INI with one [round]/[circular]/[template] section per mode. load
// is a no-op if the file is absent/unreadable, so a fresh install runs on defaults.
void load_settings(const char* path);
void save_settings(const char* path);

// Live detection status the shim publishes each frame for the UI's readout.
// mode: 0 none, 1 Round (CheckMark), 2 Circular (CheckMark2), 3 Template.
struct LiveStatus {
    int mode = 0;
    bool found = false;
    double score = 0.0;  // detector quality (symmetry score / hough edge fraction / match)
    double radiusPx = 0.0;
    double offXpx = 0.0, offYpx = 0.0;  // detection offset from the reference, raw pixels
};
LiveStatus get_status();
void publish_status(const LiveStatus& st);

}  // namespace vis
