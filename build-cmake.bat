@echo off
setlocal

rem ----------------------------------------------------------------------
rem Build MVision.dll (x86) with OpenCV via CMake + vcpkg toolchain.
rem Uses the NMake Makefiles generator (no ninja needed) and native CMake,
rem with a sanitized PATH so MSYS2's Unix cmake/git can't interfere.
rem
rem Output: build\MVision.dll
rem ----------------------------------------------------------------------

rem 1) MSVC x86 environment.
set "VSDEVCMD=C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat"
call "%VSDEVCMD%" -arch=x86 -host_arch=x64 -no_logo
if errorlevel 1 ( echo ERROR: VsDevCmd failed & exit /b 1 )

rem 2) Sanitize PATH: native CMake first, drop C:\msys64. Keep the MSVC paths
rem    that VsDevCmd just added (everything after our prepends).
set "PATH=C:\Program Files\CMake\bin;%PATH%"
set "PATH=%PATH:C:\msys64\usr\bin;=%"
set "PATH=%PATH:C:\msys64\mingw64\bin;=%"

set "CMAKE=C:\Program Files\CMake\bin\cmake.exe"

rem OpenCV was installed classic-mode by vcpkg; point find_package straight at
rem its config dir. We deliberately do NOT use the vcpkg toolchain file, which
rem would trigger manifest-mode install and pull a bad baseline.
set "OPENCV_DIR=C:/vcpkg/installed/x86-windows-static/share/opencv4"

rem nmake comes from VsDevCmd; surface it explicitly so the generator finds it.
for /f "delims=" %%I in ('where nmake 2^>nul') do set "NMAKE=%%I"
if not defined NMAKE ( echo ERROR: nmake not found after VsDevCmd & exit /b 1 )
echo Using nmake: %NMAKE%

rem 3) Configure (NMake generator = uses nmake from VsDevCmd).
"%CMAKE%" -S . -B build -G "NMake Makefiles" ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DCMAKE_MAKE_PROGRAM="%NMAKE%" ^
    -DOpenCV_DIR="%OPENCV_DIR%" ^
    -DCMAKE_PREFIX_PATH="C:/vcpkg/installed/x86-windows-static"
if errorlevel 1 ( echo ERROR: cmake configure failed & exit /b 1 )

rem 4) Build.
"%CMAKE%" --build build
if errorlevel 1 ( echo ERROR: cmake build failed & exit /b 1 )

echo.
echo Built build\MVision.dll
dir build\MVision.dll | findstr MVision.dll
