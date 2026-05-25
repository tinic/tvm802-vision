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
    DWORD a = GetFileAttributesA(path);
    return a != INVALID_FILE_ATTRIBUTES && !(a & FILE_ATTRIBUTE_DIRECTORY);
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
    int idx = g_counter.fetch_add(1, std::memory_order_relaxed);
    CreateDirectoryA(kDir, nullptr);  // no-op if present
    return idx;
}

const char* dir() {
    return kDir;
}

void log_line(std::string_view line) {
    CreateDirectoryA(kDir, nullptr);
    // Binary append so the line ending stays LF-only (matches the original
    // fopen("ab") writer; text mode would translate '\n' -> "\r\n" on Windows).
    if (std::ofstream f{std::format("{}\\compare.log", kDir), std::ios::app | std::ios::binary})
        f << line << '\n';
}

}  // namespace cap
