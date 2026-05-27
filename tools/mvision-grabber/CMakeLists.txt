# mvision_grabber.ax -- usermode DirectShow capture filter (drop-in for
# Syntek "USB2.0 Grabber"). i386 only, must match SurfaceMount.exe bitness.
# Off by default via -DBUILD_MVISION_GRABBER=ON; CI enables it.

cmake_minimum_required(VERSION 3.21)

if(NOT WIN32)
    message(FATAL_ERROR "mvision_grabber.ax is Windows-only")
endif()
if(NOT CMAKE_SIZEOF_VOID_P EQUAL 4)
    message(FATAL_ERROR "mvision_grabber.ax must be built as x86; current arch is x64")
endif()

# Uses the top-level `mvision_libusb` INTERFACE target (set in CMakeLists.txt).

add_library(mvision_grabber MODULE
    grabber.cpp
    ${CMAKE_SOURCE_DIR}/tools/saa7113-tune/common.cpp
)
target_include_directories(mvision_grabber PRIVATE
    ${CMAKE_SOURCE_DIR}/tools/saa7113-tune
)
target_link_libraries(mvision_grabber PRIVATE
    mvision_libusb
    strmiids uuid ole32 oleaut32 advapi32 user32 setupapi winusb
)
# .ax extension instead of .dll. DirectShow filters traditionally use .ax;
# the underlying file is just a COM in-proc-server DLL.
set_target_properties(mvision_grabber PROPERTIES
    OUTPUT_NAME "mvision_grabber"
    PREFIX ""
    SUFFIX ".ax"
)
if(MSVC)
    target_compile_options(mvision_grabber PRIVATE /W4 /Zc:__cplusplus /EHsc /O2)
    target_compile_definitions(mvision_grabber PRIVATE
        NOMINMAX _WIN32_WINNT=0x0601 UNICODE _UNICODE _CRT_SECURE_NO_WARNINGS)
    # Module-definition file with the four COM entry points.
    set_target_properties(mvision_grabber PROPERTIES
        LINK_OPTIONS "/DEF:${CMAKE_CURRENT_SOURCE_DIR}/mvision_grabber.def")
endif()
