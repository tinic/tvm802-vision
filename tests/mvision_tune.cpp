// mvision-tune -- offline component-detector tuner with inline-image preview.
// Two modes:
//
//   SINGLE FRAME: pass one PNG. Runs detect_component() over it, renders the
//                 same green-box overlay the live preview draws, and prints the
//                 result inline in your terminal via sixel / kitty / iTerm2
//                 (auto-detected from env, mirroring png2amiga). For tuning a
//                 specific part class against a captured snapshot without
//                 redeploying the DLL.
//
//   CORPUS BATCH: pass --corpus DIR. Scans DIR/*.png, runs detect_component on
//                 each, prints per-frame CSV to stdout plus an aggregate
//                 stats summary on stderr -- found count, method histogram,
//                 and sigma (cx, cy, w, h, angle) across the locked-on set.
//                 This is the "is this part stable?" answer in one command.
//                 No sixel in corpus mode (would flood the terminal).
//
// Cross-platform: needs OpenCV (core/imgproc/imgcodecs) and a C++23 toolchain.
// Built by the standard cmake flow on Linux / macOS / Windows.
//
// Usage:
//   mvision-tune [options] frame.png            (single-frame; sixel/kitty/iterm)
//   mvision-tune [options] --corpus DIR          (batch stats; no preview)
// Options:
//   --thr N         Override CompThre (0-100; 0 = detector default)
//   --w PX --h PX   Expected body size in px (prior; default unknown -> 0)
//   --a DEG         Expected angle in deg (prior; default NaN -> no prior)
//   --scale N       Single-frame mode only: upscale the rendered preview NxN
//                   (default 1; useful on a hidpi terminal to make the 640x480
//                   frame readable inline)
//   --proto P       Force terminal protocol: sixel | kitty | iterm | none
//                   (default: auto-detect from env). `none` writes the overlay
//                   PNG to mvision_tune_overlay.png in CWD and skips emit.

#include "vision.h"

#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "constixel.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <limits>
#include <numbers>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

namespace {

IplImage make_ipl(cv::Mat& m) {
    IplImage ipl{};
    ipl.nSize = sizeof(IplImage);
    ipl.nChannels = m.channels();
    ipl.depth = IPL_DEPTH_8U;
    ipl.dataOrder = 0;
    ipl.origin = 0;
    ipl.width = m.cols;
    ipl.height = m.rows;
    ipl.widthStep = static_cast<int>(m.step);
    ipl.imageSize = ipl.widthStep * m.rows;
    ipl.imageData = reinterpret_cast<char*>(m.data);
    ipl.imageDataOrigin = ipl.imageData;
    return ipl;
}

// Mirror src/preview.cpp's component overlay: oriented body box, direction
// arrow along +height, center cross. All at 1/16 px via cv::line shift=4 so
// the sub-pixel detection lands cleanly. The red nozzle-offset cross from
// the live preview is omitted -- it depends on controller state we don't
// have offline. A frame-center cross is drawn instead as a static reference.
void draw_overlay(cv::Mat& bgr, const vis::CompResult& r) {
    const cv::Scalar red(0, 0, 255);
    const cv::Scalar green(0, 255, 0);
    const int RT = 1;
    const int cw = bgr.cols;
    const int ch = bgr.rows;
    cv::line(bgr, cv::Point(cw / 2, 0), cv::Point(cw / 2, ch), red, RT, cv::LINE_AA);
    cv::line(bgr, cv::Point(0, ch / 2), cv::Point(cw, ch / 2), red, RT, cv::LINE_AA);

    if (!r.found) return;

    constexpr int kShift = 4;
    constexpr double kSub = static_cast<double>(1 << kShift);
    auto sp = [&](double x, double y) {
        return cv::Point(cvRound(x * kSub), cvRound(y * kSub));
    };
    // Same RotatedRect angle convention fix the live preview applies.
    const cv::RotatedRect rr(cv::Point2f(static_cast<float>(r.cx), static_cast<float>(r.cy)),
                             cv::Size2f(static_cast<float>(r.w), static_cast<float>(r.h)),
                             static_cast<float>(-r.angle));
    std::array<cv::Point2f, 4> p{};
    rr.points(p.data());
    for (int i = 0; i < 4; ++i) {
        cv::line(bgr, sp(p[i].x, p[i].y), sp(p[(i + 1) % 4].x, p[(i + 1) % 4].y),
                 green, RT, cv::LINE_AA, kShift);
    }
    const double ma = (-r.angle - 90.0) * CV_PI / 180.0;
    const double tipX = r.cx + 1.3 * r.h / 2.0 * std::cos(ma);
    const double tipY = r.cy + 1.3 * r.h / 2.0 * std::sin(ma);
    cv::arrowedLine(bgr, sp(r.cx, r.cy), sp(tipX, tipY), green, RT, cv::LINE_AA, kShift, 0.3);
    constexpr double kCrossHalf = 5.0;
    cv::line(bgr, sp(r.cx - kCrossHalf, r.cy), sp(r.cx + kCrossHalf, r.cy),
             green, RT, cv::LINE_AA, kShift);
    cv::line(bgr, sp(r.cx, r.cy - kCrossHalf), sp(r.cx, r.cy + kCrossHalf),
             green, RT, cv::LINE_AA, kShift);
}

// ---------------------------------------------------------------------------
// Terminal inline-image protocol detection. Mirror of the env-var detection
// in png2amiga/src/main.cpp::detect_inline_image_protocol().
// ---------------------------------------------------------------------------

enum class Proto { none, iterm, kitty, sixel };

Proto detect_proto() {
    auto eq = [](const char* a, const char* b) {
        return a && std::strcmp(a, b) == 0;
    };
    auto contains = [](const char* hay, const char* needle) {
        return hay && std::strstr(hay, needle) != nullptr;
    };
    auto set = [](const char* name) {
        auto v = std::getenv(name);  // NOLINT(concurrency-mt-unsafe)
        return v && *v;
    };
    auto term = std::getenv("TERM");                  // NOLINT(concurrency-mt-unsafe)
    auto term_program = std::getenv("TERM_PROGRAM");  // NOLINT(concurrency-mt-unsafe)
    // LC_TERMINAL is iTerm2's portable hint -- sshd's default AcceptEnv
    // includes LC_*, so the variable survives an SSH hop where TERM_PROGRAM
    // does not. Set by iTerm2 to "iTerm2" since build 3.4.0+ (~2020).
    auto lc_terminal = std::getenv("LC_TERMINAL");    // NOLINT(concurrency-mt-unsafe)

    if (contains(term, "kitty")) return Proto::kitty;
    if (contains(term, "ghostty")) return Proto::kitty;
    if (set("KITTY_WINDOW_ID")) return Proto::kitty;
    if (eq(term_program, "ghostty")) return Proto::kitty;
    if (set("GHOSTTY_RESOURCES_DIR")) return Proto::kitty;

    if (eq(term_program, "iTerm.app")) return Proto::iterm;
    if (eq(lc_terminal, "iTerm2")) return Proto::iterm;

    // Windows Terminal: sixel since 1.22; WT_SESSION is set on every pane.
    if (set("WT_SESSION")) return Proto::sixel;
    // VS Code integrated terminal: sixel since 1.80.
    if (eq(term_program, "vscode")) return Proto::sixel;

    return Proto::none;
}

const char* proto_name(Proto p) {
    switch (p) {
        case Proto::none:  return "none";
        case Proto::iterm: return "iterm";
        case Proto::kitty: return "kitty";
        case Proto::sixel: return "sixel";
    }
    return "?";
}

Proto parse_proto(const char* s) {
    if (std::strcmp(s, "sixel") == 0) return Proto::sixel;
    if (std::strcmp(s, "kitty") == 0) return Proto::kitty;
    if (std::strcmp(s, "iterm") == 0) return Proto::iterm;
    return Proto::none;
}

// ---------------------------------------------------------------------------
// Base64 (for iTerm OSC 1337 and kitty APC payloads). Standard alphabet,
// padded. Small enough to keep inline.
// ---------------------------------------------------------------------------

std::string base64_encode(const std::vector<std::uint8_t>& bin) {
    static constexpr char kAlpha[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((bin.size() + 2) / 3) * 4);
    std::size_t i = 0;
    for (; i + 3 <= bin.size(); i += 3) {
        std::uint32_t v = (std::uint32_t(bin[i]) << 16) |
                          (std::uint32_t(bin[i + 1]) << 8) |
                          std::uint32_t(bin[i + 2]);
        out.push_back(kAlpha[(v >> 18) & 0x3F]);
        out.push_back(kAlpha[(v >> 12) & 0x3F]);
        out.push_back(kAlpha[(v >> 6) & 0x3F]);
        out.push_back(kAlpha[v & 0x3F]);
    }
    if (i < bin.size()) {
        std::uint32_t v = std::uint32_t(bin[i]) << 16;
        if (i + 1 < bin.size()) v |= std::uint32_t(bin[i + 1]) << 8;
        out.push_back(kAlpha[(v >> 18) & 0x3F]);
        out.push_back(kAlpha[(v >> 12) & 0x3F]);
        out.push_back((i + 1 < bin.size()) ? kAlpha[(v >> 6) & 0x3F] : '=');
        out.push_back('=');
    }
    return out;
}

// ---------------------------------------------------------------------------
// Protocol emitters. All operate on a BGR cv::Mat (origin top-left).
// ---------------------------------------------------------------------------

void emit_sixel(const cv::Mat& bgr) {
    // Build an RGBA buffer, harvest the unique colors as a custom palette
    // (the overlay only adds red+green, so the post-OpenCV count is well
    // under 256 for our captures; if it ever overflows constixel's
    // 256-color built-in quantizer is used instead).
    const int W = bgr.cols;
    const int origH = bgr.rows;
    // sixel emits 6-px-tall bands; pad height up so the trailing band has
    // defined content (otherwise the terminal background bleeds through).
    const int H = ((origH + 5) / 6) * 6;
    std::vector<std::uint8_t> rgba(static_cast<std::size_t>(W) * H * 4);
    std::unordered_set<std::uint32_t> unique_colors;
    unique_colors.reserve(512);
    bool overflow = false;
    for (int y = 0; y < origH; ++y) {
        const auto* row = bgr.ptr<cv::Vec3b>(y);
        for (int x = 0; x < W; ++x) {
            const auto& px = row[x];
            std::size_t base = (static_cast<std::size_t>(y) * W + x) * 4;
            rgba[base + 0] = px[2];
            rgba[base + 1] = px[1];
            rgba[base + 2] = px[0];
            rgba[base + 3] = 255;
            if (!overflow) {
                unique_colors.insert((std::uint32_t(px[2]) << 16) |
                                     (std::uint32_t(px[1]) << 8) | std::uint32_t(px[0]));
                if (unique_colors.size() > 256) overflow = true;
            }
        }
    }
    if (H > origH && origH > 0) {
        const auto* src = &rgba[static_cast<std::size_t>(origH - 1) * W * 4];
        for (int y = origH; y < H; ++y) {
            std::memcpy(&rgba[static_cast<std::size_t>(y) * W * 4], src, static_cast<std::size_t>(W) * 4);
        }
    }
    auto sink = [](char c) { std::fputc(c, stdout); };
    std::vector<std::array<std::uint8_t, 3>> pal;
    if (!overflow) {
        pal.reserve(unique_colors.size());
        for (auto c : unique_colors) {
            pal.push_back({static_cast<std::uint8_t>((c >> 16) & 0xFF),
                           static_cast<std::uint8_t>((c >> 8) & 0xFF),
                           static_cast<std::uint8_t>(c & 0xFF)});
        }
    }
    if (!pal.empty()) {
        constixel::dynamic_image<constixel::format_8bit_dyn> dimg(
            W, H, std::span<const std::array<std::uint8_t, 3>>(pal));
        dimg.blit_RGBA(0, 0, W, H, rgba.data(), W, H, W * 4);
        dimg.sixel(sink);
    } else {
        // >256 unique colors: let constixel use its built-in 256-color
        // quantizer (good enough for a debug overlay).
        constixel::dynamic_image<constixel::format_8bit_dyn> dimg(W, H);
        dimg.blit_RGBA(0, 0, W, H, rgba.data(), W, H, W * 4);
        dimg.sixel(sink);
    }
    std::fputc('\n', stdout);
}

void emit_png_chunked(Proto p, const cv::Mat& bgr) {
    std::vector<std::uint8_t> png;
    if (!cv::imencode(".png", bgr, png)) {
        std::fprintf(stderr, "mvision-tune: cv::imencode .png FAILED\n");
        return;
    }
    auto encoded = base64_encode(png);
    if (p == Proto::iterm) {
        // OSC 1337 File=inline=1. iTerm2 sizes by `size=` and the source
        // dimensions; the cell-pixel ratio is up to the terminal.
        std::printf("\033]1337;File=inline=1;size=%zu;width=%dpx;height=%dpx:%s\a\n",
                    png.size(), bgr.cols, bgr.rows, encoded.c_str());
        return;
    }
    // Kitty / Ghostty: APC _G a=T,f=100,q=2 chunked at 4096 base64 bytes.
    // q=2 suppresses the OK/failure responses (otherwise they land on stdin).
    constexpr std::size_t kMaxChunk = 4096;
    bool first = true;
    std::size_t pos = 0;
    while (pos < encoded.size()) {
        auto bytes = std::min(encoded.size() - pos, kMaxChunk);
        bool last = (pos + bytes == encoded.size());
        std::fputs(first ? "\033_Ga=T,f=100,q=2," : "\033_G", stdout);
        first = false;
        std::fputs(last ? "m=0;" : "m=1;", stdout);
        std::fwrite(encoded.data() + pos, 1, bytes, stdout);
        std::fputs("\033\\", stdout);
        pos += bytes;
    }
    std::fputc('\n', stdout);
}

// ---------------------------------------------------------------------------
// Stats accumulator. Linear σ for the bounded values (cx, cy, w, h); circular
// σ for angle. The detector normalizes angle to (-45, 45], so two adjacent
// reads at +44 / -44 are 2 deg apart (mod 90), not 88 -- circular stats catch
// that. We compute it in the doubled-angle space and halve back at the end.
// ---------------------------------------------------------------------------

struct LinearStat {
    int n = 0;
    double sum = 0.0;
    double sumSq = 0.0;
    void push(double x) {
        ++n;
        sum += x;
        sumSq += x * x;
    }
    [[nodiscard]] double mean() const { return n > 0 ? sum / n : 0.0; }
    [[nodiscard]] double sigma() const {
        if (n < 2) return 0.0;
        const double m = mean();
        const double v = sumSq / static_cast<double>(n) - m * m;
        return std::sqrt(std::max(0.0, v));
    }
};

struct CircularAngleStat {
    int n = 0;
    double sumCos = 0.0;
    double sumSin = 0.0;
    void push(double angleDeg) {
        // The detector's angle is normalized to (-45, 45] (90-deg-symmetric
        // rectangle) -- double it before going to the unit circle so the
        // 90-deg wrap becomes a 360-deg full circle, halve back after.
        const double a2 = 2.0 * angleDeg * std::numbers::pi / 180.0;
        ++n;
        sumCos += std::cos(a2);
        sumSin += std::sin(a2);
    }
    [[nodiscard]] double mean_deg() const {
        if (n == 0) return 0.0;
        return 0.5 * std::atan2(sumSin, sumCos) * 180.0 / std::numbers::pi;
    }
    [[nodiscard]] double sigma_deg() const {
        // Circular standard deviation: sqrt(-2 ln R) in the doubled-angle
        // space, halved back to the original angle space. R is the mean
        // resultant length; R=1 -> perfectly clustered, R~0 -> uniform.
        if (n < 2) return 0.0;
        const double R = std::hypot(sumCos / n, sumSin / n);
        if (R <= 1e-9) return 90.0;  // saturated -- as scattered as possible
        const double sdRad = std::sqrt(-2.0 * std::log(R));
        return 0.5 * sdRad * 180.0 / std::numbers::pi;
    }
};

// Run detect_component on one PNG. Returns false on a read error (file
// missing / not a valid image); the caller handles the "found / not found"
// distinction via the CompResult itself.
bool run_one(const std::string& path, double expW, double expH, double expA, int threshold,
             vis::CompResult& out) {
    cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
    if (img.empty()) {
        return false;
    }
    IplImage ipl = make_ipl(img);
    out = vis::detect_component(&ipl, expW, expH, expA, threshold,
                                /*refX=*/-1.0, /*refY=*/-1.0, /*searchRadiusPx=*/0);
    return true;
}

const char* method_name(vis::CompResult::Method m) {
    return m == vis::CompResult::Method::Symmetry      ? "symmetry"
           : m == vis::CompResult::Method::MinAreaRect ? "minarearect"
                                                       : "none";
}

int run_corpus(const std::string& dir, double expW, double expH, double expA, int threshold) {
    namespace fs = std::filesystem;
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) {
        std::fprintf(stderr, "mvision-tune: --corpus %s is not a directory\n", dir.c_str());
        return 2;
    }
    // Collect + sort the .png entries -- deterministic ordering matters
    // because the per-frame CSV is what an operator scrolls to find a
    // misbehaving frame.
    std::vector<std::string> files;
    for (auto& entry : fs::directory_iterator(dir, ec)) {
        if (!entry.is_regular_file()) continue;
        auto ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (ext == ".png") {
            files.push_back(entry.path().string());
        }
    }
    std::sort(files.begin(), files.end());
    if (files.empty()) {
        std::fprintf(stderr, "mvision-tune: no .png files in %s\n", dir.c_str());
        return 1;
    }

    LinearStat sCx, sCy, sW, sH, sQuality;
    CircularAngleStat sAngle;
    int nFound = 0;
    int nSym = 0;
    int nMar = 0;
    int nNone = 0;
    int nReadError = 0;

    std::printf("file,found,cx,cy,w,h,angle,quality,method\n");
    for (const auto& path : files) {
        vis::CompResult r;
        if (!run_one(path, expW, expH, expA, threshold, r)) {
            std::printf("%s,READ_ERROR\n", path.c_str());
            ++nReadError;
            continue;
        }
        std::printf("%s,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%s\n", path.c_str(),
                    r.found ? 1 : 0, r.cx, r.cy, r.w, r.h, r.angle, r.quality, method_name(r.method));
        if (r.found) {
            ++nFound;
            sCx.push(r.cx);
            sCy.push(r.cy);
            sW.push(r.w);
            sH.push(r.h);
            sAngle.push(r.angle);
            sQuality.push(r.quality);
            switch (r.method) {
                case vis::CompResult::Method::Symmetry:    ++nSym; break;
                case vis::CompResult::Method::MinAreaRect: ++nMar; break;
                case vis::CompResult::Method::None:        ++nNone; break;
            }
        }
    }
    const int nTotal = static_cast<int>(files.size());
    std::fprintf(stderr,
                 "\nmvision-tune corpus: %s  thr=%d\n"
                 "  files=%d  found=%d (%.0f%%)  read_errors=%d\n"
                 "  methods: symmetry=%d  minarearect=%d  none=%d\n",
                 dir.c_str(), threshold, nTotal, nFound,
                 nTotal > 0 ? (100.0 * nFound / nTotal) : 0.0, nReadError, nSym, nMar, nNone);
    if (nFound >= 2) {
        std::fprintf(stderr,
                     "  cx     mean=%.2f  sigma=%.2f  px\n"
                     "  cy     mean=%.2f  sigma=%.2f  px\n"
                     "  w      mean=%.2f  sigma=%.2f  px\n"
                     "  h      mean=%.2f  sigma=%.2f  px\n"
                     "  angle  mean=%.2f  sigma=%.2f  deg  (circular)\n"
                     "  qual   mean=%.3f sigma=%.3f\n",
                     sCx.mean(), sCx.sigma(),
                     sCy.mean(), sCy.sigma(),
                     sW.mean(), sW.sigma(),
                     sH.mean(), sH.sigma(),
                     sAngle.mean_deg(), sAngle.sigma_deg(),
                     sQuality.mean(), sQuality.sigma());
    } else if (nFound == 1) {
        std::fprintf(stderr, "  (only 1 found -- no spread)\n");
    }
    return nFound > 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    double expW = 0.0;
    double expH = 0.0;
    double expA = std::numeric_limits<double>::quiet_NaN();
    int threshold = 0;
    int scale = 1;
    Proto forced = Proto::none;
    bool protoForced = false;
    const char* path = nullptr;
    const char* corpus = nullptr;

    for (int i = 1; i < argc; ++i) {
        auto need = [&](const char* flag) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "mvision-tune: %s needs an argument\n", flag);
                std::exit(2);
            }
            return argv[++i];
        };
        std::string_view a(argv[i]);
        if (a == "--w")               expW = std::atof(need("--w"));
        else if (a == "--h")          expH = std::atof(need("--h"));
        else if (a == "--a")          expA = std::atof(need("--a"));
        else if (a == "--thr")        threshold = std::atoi(need("--thr"));
        else if (a == "--scale")      scale = std::max(1, std::min(8, std::atoi(need("--scale"))));
        else if (a == "--proto")    { forced = parse_proto(need("--proto")); protoForced = true; }
        else if (a == "--corpus")     corpus = need("--corpus");
        else if (a.size() > 0 && a[0] == '-') {
            std::fprintf(stderr, "mvision-tune: unknown flag %s\n", argv[i]);
            return 2;
        } else {
            path = argv[i];
        }
    }
    if (corpus != nullptr) {
        if (path != nullptr) {
            std::fprintf(stderr,
                         "mvision-tune: --corpus and a positional frame are mutually "
                         "exclusive; pick one.\n");
            return 2;
        }
        return run_corpus(corpus, expW, expH, expA, threshold);
    }
    if (path == nullptr) {
        std::fprintf(stderr,
                     "usage: mvision-tune [--thr N] [--w PX] [--h PX] [--a DEG] "
                     "[--scale N] [--proto sixel|kitty|iterm|none] frame.png\n"
                     "       mvision-tune [--thr N] ... --corpus DIR/        (batch stats)\n");
        return 2;
    }

    cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
    if (img.empty()) {
        std::fprintf(stderr, "mvision-tune: cannot read %s\n", path);
        return 1;
    }

    IplImage ipl = make_ipl(img);
    vis::CompResult r = vis::detect_component(&ipl, expW, expH, expA, threshold,
                                              /*refX=*/-1.0, /*refY=*/-1.0,
                                              /*searchRadiusPx=*/0);

    // Render the overlay onto a gray->BGR copy, same order as the live preview.
    cv::Mat gray;
    if (img.channels() == 1) gray = img.clone();
    else                     cv::extractChannel(img, gray, 0);
    cv::Mat overlay;
    cv::cvtColor(gray, overlay, cv::COLOR_GRAY2BGR);
    draw_overlay(overlay, r);

    if (scale > 1) {
        cv::Mat up;
        cv::resize(overlay, up, cv::Size(overlay.cols * scale, overlay.rows * scale),
                   0, 0, cv::INTER_NEAREST);
        overlay = std::move(up);
    }

    Proto proto = protoForced ? forced : detect_proto();
    // Summary on stderr so it survives stdout-redirection.
    std::fprintf(stderr,
                 "mvision-tune: %s  proto=%s  thr=%d  found=%d  "
                 "cx=%.2f cy=%.2f w=%.2f h=%.2f angle=%.2f q=%.3f method=%s\n",
                 path, proto_name(proto), threshold, r.found ? 1 : 0,
                 r.cx, r.cy, r.w, r.h, r.angle, r.quality, method_name(r.method));

    if (proto == Proto::none) {
        const char* outPath = "mvision_tune_overlay.png";  // CWD-relative; cross-platform
        if (!cv::imwrite(outPath, overlay)) {
            std::fprintf(stderr, "mvision-tune: cv::imwrite %s FAILED\n", outPath);
            return 1;
        }
        std::fprintf(stderr, "mvision-tune: terminal has no inline-image protocol; overlay -> %s\n",
                     outPath);
        return r.found ? 0 : 1;
    }
    if (proto == Proto::sixel) emit_sixel(overlay);
    else                       emit_png_chunked(proto, overlay);
    return r.found ? 0 : 1;
}
