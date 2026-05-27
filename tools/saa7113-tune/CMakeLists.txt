# saa7113-tune: standalone USB tuning utility -- NOT part of MVision.dll, NOT
# loaded by SurfaceMount. Standalone .exe that runs while the host is closed.
# Live-preview mode is Windows-only (needs Win32 + GDI); on Linux/macOS only
# the one-shot subcommands build (live.cpp is excluded).
#
# Build is opt-in via -DBUILD_SAA7113_TUNE=ON because it pulls libusb-1.0,
# which the default DLL build doesn't need.

cmake_minimum_required(VERSION 3.21)

# Uses the top-level `mvision_libusb` INTERFACE target (set in CMakeLists.txt).

set(SAA_SOURCES main.cpp common.cpp)
if(WIN32)
    list(APPEND SAA_SOURCES live.cpp)
else()
    # Stub `run_live` on non-Windows so main.cpp links cleanly.
    set_property(SOURCE main.cpp APPEND PROPERTY COMPILE_DEFINITIONS SAA_NO_LIVE)
endif()

add_executable(saa7113-tune ${SAA_SOURCES})
target_link_libraries(saa7113-tune PRIVATE mvision_libusb)
if(WIN32)
    target_link_libraries(saa7113-tune PRIVATE user32 gdi32)
endif()
if(MSVC)
    target_compile_options(saa7113-tune PRIVATE /W4 /Zc:__cplusplus /EHsc /O2)
else()
    target_compile_options(saa7113-tune PRIVATE -Wall -Wextra)
endif()
