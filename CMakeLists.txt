cmake_minimum_required(VERSION 3.21)
project(MVision LANGUAGES CXX)

option(BUILD_TESTS "Build the offline OpenCV detector test harness (host arch)" OFF)

# Default to Release for single-config generators (Unix Makefiles, Ninja) when
# the user hasn't picked one. Without this, an empty CMAKE_BUILD_TYPE means
# CMake passes NO optimization flags to GCC/Clang and the host-side mvision-tune
# binary runs unoptimized -- a 2-5x slowdown vs Release for no reason. Multi-
# config generators (Visual Studio) ignore CMAKE_BUILD_TYPE; their per-config
# flags (e.g. Release /O2) are set in the MSVC block below.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "Build type (Debug / Release / RelWithDebInfo / MinSizeRel)" FORCE)
endif()

# C++23 for std::format-based diagnostics/UI text (no snprintf / fixed char buffers)
# and std::array throughout. The detector core + Win32 TUs only use <= C++20 library
# features, so dropping back to 20 here stays buildable if a toolchain lacks C++23.
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Version string for the UI, from git (self-maintaining; tag build -> "v1.7.2",
# dev build -> "v1.7.1-3-gabc123-dirty"). Falls back to "dev" outside a git tree.
find_package(Git QUIET)
if(Git_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --tags --dirty --always
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE MVISION_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
endif()
if(NOT MVISION_VERSION)
    set(MVISION_VERSION "dev")
endif()
message(STATUS "MVision version: ${MVISION_VERSION}")

# The MVision.dll replacement must be x86 to match SurfaceMount.exe (i386). The
# offline tools (mvision-tune) + test harness (BUILD_TESTS) have no such
# constraint, so on a 64-bit host (incl. Linux/macOS) we just build those and
# skip the DLL rather than failing configuration.
if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(_build_dll TRUE)
else()
    set(_build_dll FALSE)
    message(STATUS "64-bit host: skipping the x86 MVision.dll target; building mvision-tune (+ tests if BUILD_TESTS=ON).")
endif()

# OpenCV from vcpkg (toolchain file passed on the cmake command line).
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# Threads: the circular-symmetry detector parallelizes its coarse search across
# cores (std::thread). On Windows this is a no-op; on POSIX it links pthread.
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# SIMD: the detector's inner sampling is hot, so enable AVX2 + FMA. AVX2 is
# baseline on any x86 CPU made in the last decade (Haswell/Zen+). MSVC uses
# /arch:AVX2 (set per-target below); GCC/Clang/mingw use -mavx2 -mfma.
if(MSVC)
    set(MVISION_SIMD)  # MSVC AVX2 is applied via /arch:AVX2 in the per-target block
else()
    set(MVISION_SIMD -mavx2 -mfma)
endif()

# Strict diagnostics, per cpp-best-practices "Use the Tools Available". Applied to
# OUR translation units only: third-party headers (OpenCV, Windows, the STL) are
# pulled in via angle brackets and set to /external:W0, so /WX never trips on
# warnings we can't fix. /Wall is intentionally avoided (it's noisy in system
# headers). On non-MSVC toolchains (the test harness on a dev box) use a sensible
# GCC/Clang set without -Werror.
if(MSVC)
    set(MVISION_WARNINGS
        /W4                  # all reasonable warnings
        /permissive-         # standards conformance
        /w14242 /w14254 /w14263 /w14265 /w14287 /we4289 /w14296
        /w14311 /w14545 /w14546 /w14547 /w14549 /w14555
        /w14619 /w14640 /w14826 /w14905 /w14906 /w14928
        /WX                  # treat warnings as errors
        /external:anglebrackets /external:W0)  # silence third-party headers
else()
    set(MVISION_WARNINGS -Wall -Wextra -Wshadow -Wnon-virtual-dtor -Wpedantic
                         -Wconversion -Wsign-conversion)
endif()

if(_build_dll)
    # Static CRT to match the x86-windows-static vcpkg triplet and avoid runtime
    # DLL dependencies at install time.
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

    set(MVISION_SOURCES
        src/dllmain.cpp
        src/passthrough.cpp
        src/capture.cpp
        src/detect_common.cpp
        src/detect_circle.cpp
        src/detect_contour.cpp
        src/detect_template.cpp
        src/detect_symmetry.cpp
        src/detect_component.cpp
        src/settings.cpp
        src/settings_ui.cpp
        src/cam_props.cpp
        src/preview.cpp
        src/controller.cpp
        src/thread_pool.cpp
    )

    # Build one MVision.dll. use_avx2=ON emits AVX2+FMA in OUR translation units
    # (vectorizes the inner sampling/ring/contrast loops; needs a ~2015+ Haswell/Zen+
    # CPU at runtime). use_avx2=OFF falls back to the universal x86 SSE2 baseline so
    # the DLL runs on ANY pre-AVX2 CPU -- shipped as a separate MVision-noavx2.dll for
    # the rare old PC where the AVX2 build faults with an illegal instruction. (OpenCV
    # itself is precompiled with its own runtime CPU dispatch, so it is fine either
    # way; the AVX2 requirement comes only from our own emitted code.)
    function(add_mvision_dll target output_name use_avx2)
        add_library(${target} SHARED ${MVISION_SOURCES})
        set_target_properties(${target} PROPERTIES
            OUTPUT_NAME "${output_name}" PREFIX "" SUFFIX ".dll")
        target_include_directories(${target} PRIVATE src)
        # gdi32: preview.cpp uses GDI (StretchDIBits, GetStockObject, ...).
        # ws2_32 + iphlpapi: controller.cpp reads the px/mm scale from the motion
        # controller (TCP), discovering its endpoint via the host's own connection.
        target_link_libraries(${target} PRIVATE ${OpenCV_LIBS} Threads::Threads kernel32 user32 gdi32 comctl32 uxtheme ws2_32 iphlpapi strmiids ole32 oleaut32 uuid)
        target_compile_definitions(${target} PRIVATE _CRT_SECURE_NO_WARNINGS
                                   MVISION_VERSION="${MVISION_VERSION}")
        target_compile_options(${target} PRIVATE ${MVISION_WARNINGS})
        if(MSVC)
            target_compile_options(${target} PRIVATE /Zc:__cplusplus /EHsc /O2 /fp:contract)
            if(use_avx2)
                target_compile_options(${target} PRIVATE /arch:AVX2)
            endif()
            # use_avx2=OFF leaves the MSVC x86 default (/arch:SSE2) -- the baseline.
        else()
            if(use_avx2)
                target_compile_options(${target} PRIVATE -mavx2 -mfma)
            endif()
        endif()
    endfunction()

    # Default DLL (recommended for modern CPUs) + the no-AVX2 baseline variant. Same
    # ABI and behavior; build both so the right one can be dropped in as MVision.dll.
    add_mvision_dll(MVision "MVision" ON)
    add_mvision_dll(MVision_noavx2 "MVision-noavx2" OFF)

    # Exports: __declspec(dllexport) in source auto-emits _Name@N stdcall names.
    # The SetUpisionOffsetXY typo alias is added via #pragma in passthrough.cpp.
endif()

if(BUILD_TESTS)
    # Links only the platform-neutral detector (no preview.cpp / no Windows libs).
    add_executable(mvision_detect_test tests/test_detect.cpp
        src/detect_common.cpp
        src/detect_circle.cpp
        src/detect_contour.cpp
        src/detect_template.cpp
        src/detect_symmetry.cpp
        src/detect_component.cpp
        src/settings.cpp
        src/thread_pool.cpp)
    target_include_directories(mvision_detect_test PRIVATE src)
    target_link_libraries(mvision_detect_test PRIVATE ${OpenCV_LIBS} Threads::Threads)
    target_compile_definitions(mvision_detect_test PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_options(mvision_detect_test PRIVATE ${MVISION_WARNINGS} ${MVISION_SIMD})
    if(MSVC)
        target_compile_options(mvision_detect_test PRIVATE /Zc:__cplusplus /EHsc)
    endif()
    enable_testing()
    add_test(NAME detect_synthetic COMMAND mvision_detect_test)
endif()

# ---------------------------------------------------------------------------
# mvision-tune -- offline component-detector tuner with inline-image preview
# (sixel / kitty / iTerm2). Always built. Cross-platform: needs only OpenCV
# (already required above) and a C++23 toolchain. constixel is vendored as a
# git submodule; configure fails with a clear hint if it isn't checked out.
# ---------------------------------------------------------------------------
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/constixel/constixel.hpp")
    message(WARNING
        "third_party/constixel/constixel.hpp not found. mvision-tune will not be built. "
        "Run: git submodule update --init --recursive")
else()
    add_executable(mvision-tune tests/mvision_tune.cpp
        src/detect_common.cpp
        src/detect_circle.cpp
        src/detect_contour.cpp
        src/detect_template.cpp
        src/detect_symmetry.cpp
        src/detect_component.cpp
        src/settings.cpp
        src/thread_pool.cpp)
    target_include_directories(mvision-tune PRIVATE src third_party/constixel)
    target_link_libraries(mvision-tune PRIVATE ${OpenCV_LIBS} Threads::Threads)
    target_compile_definitions(mvision-tune PRIVATE _CRT_SECURE_NO_WARNINGS)
    # No /WX or -Werror -- this is a developer tool, third-party constixel
    # parsing throws a few deprecated-enum-enum-conversion warnings inside
    # OpenCV's headers that we don't want to gate the build on.
    if(MSVC)
        target_compile_options(mvision-tune PRIVATE /Zc:__cplusplus /EHsc /O2)
    endif()
endif()
