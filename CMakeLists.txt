cmake_minimum_required(VERSION 3.21)
project(MVision LANGUAGES CXX)

# x86 only — SurfaceMount.exe is i386, the DLL must match.
if(NOT CMAKE_SIZEOF_VOID_P EQUAL 4)
    message(FATAL_ERROR "MVision must be built x86. Use an x86 toolchain (VsDevCmd -arch=x86).")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Static CRT to match the x86-windows-static vcpkg triplet and avoid runtime
# DLL dependencies at install time.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# OpenCV from vcpkg (toolchain file passed on the cmake command line).
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_library(MVision SHARED
    src/dllmain.cpp
    src/passthrough.cpp
    src/capture.cpp
    src/vision.cpp
)

set_target_properties(MVision PROPERTIES
    OUTPUT_NAME "MVision"
    PREFIX ""
    SUFFIX ".dll"
)

target_include_directories(MVision PRIVATE src)
target_link_libraries(MVision PRIVATE ${OpenCV_LIBS} kernel32 user32)

target_compile_definitions(MVision PRIVATE _CRT_SECURE_NO_WARNINGS)
if(MSVC)
    # /O2 optimize; /fp:contract allow FMA; /arch:AVX2 vectorize our own inner
    # loops (circularity ring, contrast window, centroid reduce) -- OpenCV is
    # precompiled with its own CPU dispatch. Requires an AVX2-capable CPU at
    # runtime (anything from ~2015 / Haswell+ / Zen+).
    target_compile_options(MVision PRIVATE /W4 /permissive- /Zc:__cplusplus /EHsc
                           /O2 /fp:contract /arch:AVX2)
endif()

# Exports: __declspec(dllexport) in source auto-emits _Name@N stdcall names.
# The SetUpisionOffsetXY typo alias is added via #pragma in passthrough.cpp.
