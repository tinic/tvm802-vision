#include "capture.h"

#include <windows.h>
#include <atomic>
#include <format>
#include <fstream>
#include <string>
#include <string_view>

namespace {
constexpr const char* kDir = "C:\\mvision_capture";
constexpr const char* kTrigger = "C:\\mvision_capture\\on";
constexpr const char* kFramesTrigger = "C:\\mvision_capture\\frames";
constexpr const char* kCompTrigger = "C:\\mvision_capture\\comp";

std::atomic<int> g_counter{0};

bool file_exists(const char* path) {
    const DWORD a = GetFileAttributesA(path);
    return a != INVALID_FILE_ATTRIBUTES && ((a & FILE_ATTRIBUTE_DIRECTORY) == 0u);
}
}  // namespace

namespace cap {

bool armed() {
    // No frame cap: capture for as long as the trigger file exists, so a full
    // run (including the fast-motion frames we need to diagnose) is recorded.
    // Remove the trigger to stop.
    return file_exists(kTrigger);
}

bool frames_enabled() {
    return file_exists(kFramesTrigger);
}

bool comp_enabled() {
    return file_exists(kCompTrigger);
}

int next_index() {
    // Part-boundary detection (for offline smoke-test slicing). A new placement
    // cycle is hundreds of ms of motion between CheckComp reads, so a > 1 s gap
    // is a reliable "new part starts here" signal. Frame numbering stays
    // monotonic across the session; the boundary is recorded in compare.log
    // as a line "# PART <p> starts at frame <f>" so a curator can slice the
    // saved PNGs into per-part buckets retrospectively.
    static std::atomic<uint64_t> g_lastMs{0};
    static std::atomic<int> g_partCount{0};
    const uint64_t nowMs = ::GetTickCount64();
    const uint64_t lastMs = g_lastMs.exchange(nowMs, std::memory_order_relaxed);
    const bool newPart = (lastMs == 0) || (nowMs > lastMs && nowMs - lastMs > 1000);
    if (newPart) {
        const int p = g_partCount.fetch_add(1, std::memory_order_relaxed) + 1;
        const int f = g_counter.load(std::memory_order_relaxed);
        log_line(std::format("# PART {} starts at frame {}", p, f));
    }

    const int idx = g_counter.fetch_add(1, std::memory_order_relaxed);
    CreateDirectoryA(kDir, nullptr);  // no-op if present
    return idx;
}

const char* dir() {
    return kDir;
}

void log_line(std::string_view line) {
    // Diagnostic sink: must never throw across the C ABI (std::format / ofstream can
    // allocate / fail). Swallow -- a lost log line is harmless.
    try {
        CreateDirectoryA(kDir, nullptr);
        // Binary append so the line ending stays LF-only (matches the original
        // fopen("ab") writer; text mode would translate '\n' -> "\r\n" on Windows).
        if (std::ofstream f{std::format("{}\\compare.log", kDir), std::ios::app | std::ios::binary}) {
            f << line << '\n';
        }
    } catch (...) {  // NOLINT(bugprone-empty-catch) -- best-effort sink; must not cross the C ABI
    }
}

}  // namespace cap
