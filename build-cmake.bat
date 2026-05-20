@echo off
setlocal

rem ----------------------------------------------------------------------
rem Build MVision.dll (x86, static OpenCV) via CMake + vcpkg manifest.
rem
rem OpenCV is declared in vcpkg.json (minimal: core/imgproc/imgcodecs+png) and
rem installed automatically by the vcpkg toolchain during configure. Static
rem linkage => the resulting MVision.dll is self-contained.
rem
rem Requires: Visual Studio Build Tools (x86), CMake, and vcpkg.
rem Set VCPKG_ROOT to your vcpkg checkout (defaults to C:\vcpkg).
rem
rem Output: build\Release\MVision.dll
rem ----------------------------------------------------------------------

if "%VCPKG_ROOT%"=="" set "VCPKG_ROOT=C:\vcpkg"

rem Pin the manifest baseline to your vcpkg checkout (one-time; no-op if set).
"%VCPKG_ROOT%\vcpkg.exe" x-update-baseline --add-initial-baseline
if errorlevel 1 ( echo ERROR: vcpkg x-update-baseline failed & exit /b 1 )

rem Configure (manifest install of OpenCV happens here) + build.
cmake -S . -B build -A Win32 ^
    -DCMAKE_TOOLCHAIN_FILE="%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake" ^
    -DVCPKG_TARGET_TRIPLET=x86-windows-static
if errorlevel 1 ( echo ERROR: cmake configure failed & exit /b 1 )

cmake --build build --config Release
if errorlevel 1 ( echo ERROR: cmake build failed & exit /b 1 )

echo.
echo Built build\Release\MVision.dll
dir build\Release\MVision.dll | findstr MVision.dll
