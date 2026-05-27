# saa7113-tune: standalone USB tuning utility -- NOT part of MVision.dll, NOT
# loaded by SurfaceMount. Standalone .exe that runs while the host is closed.
#
# Build is opt-in via -DBUILD_SAA7113_TUNE=ON because it pulls libusb-1.0,
# which the default DLL build doesn't need. The top-level CMakeLists.txt
# add_subdirectory's this only when the option is on.

cmake_minimum_required(VERSION 3.21)

# libusb-1.0 ships with a CMake config in modern vcpkg ports; fall back to
# pkg-config (Linux dev box) when find_package can't find it. Either way the
# resulting target is `libusb-1.0` -- ours via an INTERFACE alias if needed.
find_package(libusb CONFIG QUIET)
if(NOT libusb_FOUND)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(LIBUSB IMPORTED_TARGET libusb-1.0)
    endif()
    if(NOT TARGET PkgConfig::LIBUSB)
        message(FATAL_ERROR
            "libusb-1.0 not found. On vcpkg: add \"libusb\" to vcpkg.json. "
            "On Linux: apt install libusb-1.0-0-dev.")
    endif()
    add_library(saa_libusb INTERFACE)
    target_link_libraries(saa_libusb INTERFACE PkgConfig::LIBUSB)
else()
    add_library(saa_libusb INTERFACE)
    target_link_libraries(saa_libusb INTERFACE libusb::libusb)
endif()

add_executable(saa7113-tune main.cpp)
target_link_libraries(saa7113-tune PRIVATE saa_libusb)
# This is a small utility; keep warnings sane but not -Werror.
if(MSVC)
    target_compile_options(saa7113-tune PRIVATE /W4 /Zc:__cplusplus /EHsc /O2)
else()
    target_compile_options(saa7113-tune PRIVATE -Wall -Wextra)
endif()
