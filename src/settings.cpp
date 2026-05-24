#include "settings.h"

#include <cstdlib>
#include <fstream>
#include <mutex>
#include <string>

// Windows-free: std::mutex / std::fstream only, so this links into the portable
// detector core and the off-target test harness alongside the Win32 settings UI.

namespace vis {
namespace {
std::mutex g_mu;
Settings g_settings;
LiveStatus g_status;
}  // namespace

Settings get_settings() {
    std::lock_guard<std::mutex> lk(g_mu);
    return g_settings;
}

void set_settings(const Settings& s) {
    std::lock_guard<std::mutex> lk(g_mu);
    g_settings = s;
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
    Settings s;
    std::string line;
    while (std::getline(f, line)) {
        const std::string::size_type eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = line.substr(0, eq);
        const double val = std::atof(line.c_str() + eq + 1);
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
    }
    set_settings(s);
}

void save_settings(const char* path) {
    if (!path) return;
    const Settings s = get_settings();
    std::ofstream f(path, std::ios::trunc);
    if (!f) return;
    f << "radiusMinPx=" << s.radiusMinPx << "\n"
      << "radiusMaxPx=" << s.radiusMaxPx << "\n"
      << "minSymmetry=" << s.minSymmetry << "\n"
      << "gamma=" << s.gamma << "\n"
      << "brightness=" << s.brightness << "\n"
      << "contrast=" << s.contrast << "\n"
      << "blackPoint=" << s.blackPoint << "\n"
      << "whitePoint=" << s.whitePoint << "\n"
      << "sharpen=" << s.sharpen << "\n";
}

}  // namespace vis
