#include "settings.h"

#include <array>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <string>

// Windows-free: std::mutex / std::fstream only, so this links into the portable
// detector core and the off-target test harness alongside the Win32 settings UI.

namespace vis {
namespace {
std::mutex g_mu;
std::array<Settings, MODE_COUNT> g_settings;  // indexed by mode; [1..3] used
LiveStatus g_status;

std::size_t clamp_mode(int m) {
    return static_cast<std::size_t>((m >= MODE_ROUND && m <= MODE_TEMPLATE) ? m : MODE_ROUND);
}

const char* mode_name(int m) {
    return m == MODE_ROUND ? "round" : m == MODE_CIRCULAR ? "circular"
                                                          : "template";
}

int mode_from_name(const std::string& s) {
    if (s == "round") return MODE_ROUND;
    if (s == "circular") return MODE_CIRCULAR;
    if (s == "template") return MODE_TEMPLATE;
    return MODE_NONE;
}

void assign_kv(Settings& s, const std::string& key, double val) {
    if (key == "radiusMinPx")
        s.radiusMinPx = val;
    else if (key == "radiusMaxPx")
        s.radiusMaxPx = val;
    else if (key == "minSymmetry")
        s.minSymmetry = val;
    else if (key == "gamma")
        s.gamma = val;
    else if (key == "brightness")
        s.brightness = val;
    else if (key == "contrast")
        s.contrast = val;
    else if (key == "blackPoint")
        s.blackPoint = val;
    else if (key == "whitePoint")
        s.whitePoint = val;
    else if (key == "sharpen")
        s.sharpen = val;
    else if (key == "blur")
        s.blur = val;
    else if (key == "meanLo")
        s.meanLo = val;
    else if (key == "meanHi")
        s.meanHi = val;
    else if (key == "medianRings")
        s.medianRings = val;
}

void write_section(std::ofstream& f, int m, const Settings& s) {
    f << "[" << mode_name(m) << "]\n"
      << "radiusMinPx=" << s.radiusMinPx << "\n"
      << "radiusMaxPx=" << s.radiusMaxPx << "\n"
      << "minSymmetry=" << s.minSymmetry << "\n"
      << "gamma=" << s.gamma << "\n"
      << "brightness=" << s.brightness << "\n"
      << "contrast=" << s.contrast << "\n"
      << "blackPoint=" << s.blackPoint << "\n"
      << "whitePoint=" << s.whitePoint << "\n"
      << "sharpen=" << s.sharpen << "\n"
      << "blur=" << s.blur << "\n"
      << "meanLo=" << s.meanLo << "\n"
      << "meanHi=" << s.meanHi << "\n"
      << "medianRings=" << s.medianRings << "\n\n";
}
}  // namespace

Settings get_settings(int mode) {
    std::lock_guard<std::mutex> lk(g_mu);
    return g_settings[clamp_mode(mode)];
}

void set_settings(int mode, const Settings& s) {
    std::lock_guard<std::mutex> lk(g_mu);
    g_settings[clamp_mode(mode)] = s;
}

LiveStatus get_status() {
    std::lock_guard<std::mutex> lk(g_mu);
    return g_status;
}

void publish_status(const LiveStatus& st) {
    std::lock_guard<std::mutex> lk(g_mu);
    g_status = st;
}

void load_settings(const char* path) {
    if (!path) return;
    std::ifstream f(path);
    if (!f) return;
    std::array<Settings, MODE_COUNT> parsed;  // start from defaults; fill the sections present
    int cur = MODE_NONE;
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line.front() == '[') {
            const std::string::size_type rb = line.find(']');
            if (rb != std::string::npos) cur = mode_from_name(line.substr(1, rb - 1));
            continue;
        }
        if (cur == MODE_NONE) continue;  // keys before any [section] are ignored
        const std::string::size_type eq = line.find('=');
        if (eq == std::string::npos) continue;
        assign_kv(parsed[static_cast<std::size_t>(cur)], line.substr(0, eq),
                  std::atof(line.c_str() + eq + 1));
    }
    std::lock_guard<std::mutex> lk(g_mu);
    for (int m = MODE_ROUND; m <= MODE_TEMPLATE; ++m)
        g_settings[static_cast<std::size_t>(m)] = parsed[static_cast<std::size_t>(m)];
}

void save_settings(const char* path) {
    if (!path) return;
    std::array<Settings, MODE_COUNT> snap;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        for (int m = MODE_ROUND; m <= MODE_TEMPLATE; ++m)
            snap[static_cast<std::size_t>(m)] = g_settings[static_cast<std::size_t>(m)];
    }
    std::ofstream f(path, std::ios::trunc);
    if (!f) return;
    for (int m = MODE_ROUND; m <= MODE_TEMPLATE; ++m) write_section(f, m, snap[static_cast<std::size_t>(m)]);
}

}  // namespace vis
