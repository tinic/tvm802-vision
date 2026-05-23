cmake_minimum_required(VERSION 3.21)
project(MVision LANGUAGES CXX)

option(BUILD_TESTS "Build the offline OpenCV detector test harness (host arch)" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The MVision.dll replacement must be x86 to match SurfaceMount.exe (i386). The
# offline test harness (BUILD_TESTS) has no such constraint, so on a 64-bit host
# we build the tests only and skip the DLL rather than failing configuration.
if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(_build_dll TRUE)
else()
    set(_build_dll FALSE)
    if(NOT BUILD_TESTS)
        message(FATAL_ERROR
            "MVision must be built x86 (use an x86 toolchain, e.g. VsDevCmd -arch=x86). "
            "Configure with -DBUILD_TESTS=ON to build the host-arch test harness instead.")
    endif()
    message(STATUS "64-bit host: skipping the x86 MVision.dll target; building tests only.")
endif()

# OpenCV from vcpkg (toolchain file passed on the cmake command line).
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

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

    add_library(MVision SHARED
        src/dllmain.cpp
        src/passthrough.cpp
        src/capture.cpp
        src/detect_common.cpp
        src/detect_circle.cpp
        src/detect_contour.cpp
        src/detect_template.cpp
        src/detect_symmetry.cpp
        src/preview.cpp
    )

    set_target_properties(MVision PROPERTIES
        OUTPUT_NAME "MVision"
        PREFIX ""
        SUFFIX ".dll"
    )

    target_include_directories(MVision PRIVATE src)
    # gdi32: preview.cpp uses GDI (StretchDIBits, GetStockObject, ...).
    target_link_libraries(MVision PRIVATE ${OpenCV_LIBS} kernel32 user32 gdi32)

    target_compile_definitions(MVision PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_options(MVision PRIVATE ${MVISION_WARNINGS})
    if(MSVC)
        # /O2 optimize; /fp:contract allow FMA; /arch:AVX2 vectorize our own inner
        # loops (circularity ring, contrast window, centroid reduce) -- OpenCV is
        # precompiled with its own CPU dispatch. Requires an AVX2-capable CPU at
        # runtime (anything from ~2015 / Haswell+ / Zen+).
        target_compile_options(MVision PRIVATE /Zc:__cplusplus /EHsc
                               /O2 /fp:contract /arch:AVX2)
    endif()

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
        src/detect_symmetry.cpp)
    target_include_directories(mvision_detect_test PRIVATE src)
    target_link_libraries(mvision_detect_test PRIVATE ${OpenCV_LIBS})
    target_compile_definitions(mvision_detect_test PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_options(mvision_detect_test PRIVATE ${MVISION_WARNINGS})
    if(MSVC)
        target_compile_options(mvision_detect_test PRIVATE /Zc:__cplusplus /EHsc)
    endif()
    enable_testing()
    add_test(NAME detect_synthetic COMMAND mvision_detect_test)
endif()
