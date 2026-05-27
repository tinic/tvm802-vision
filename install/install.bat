@echo off
REM tvm802-vision installer. Run as administrator from the unzipped bundle.
REM
REM What it does:
REM   1. Checks SurfaceMount.exe isn't running (DLL would be locked).
REM   2. Backs up the stock MVision.dll -> MVision-orig.dll on first install.
REM   3. Copies MVision.dll + mvision_grabber.ax + saa7113-tune.exe to the
REM      SurfaceMount folder.
REM   4. Registers mvision_grabber.ax with DirectShow via 32-bit regsvr32.
REM   5. Prints the Zadig step the operator needs to do once per machine to
REM      enable the Camera-tab hardware tuning (chip-level brightness /
REM      contrast / gain).
REM
REM Default install target is %USERPROFILE%\qihetvm802b\ -- override by
REM passing a path:  install.bat "D:\Path\to\SurfaceMount\folder"

setlocal

set "SRC=%~dp0"
if "%~1"=="" (
    set "DST=%USERPROFILE%\qihetvm802b"
) else (
    set "DST=%~1"
)

echo === tvm802-vision installer ===
echo Source : %SRC%
echo Target : %DST%
echo.

if not exist "%DST%" (
    echo ERROR: target folder %DST% not found.
    echo        Pass the SurfaceMount install path:  install.bat "PATH"
    pause
    exit /b 1
)

if not exist "%DST%\SurfaceMount.exe" (
    echo WARNING: SurfaceMount.exe not found in %DST%.
    echo          Continue anyway? Ctrl-C to abort, any key to proceed.
    pause
)

tasklist /fi "imagename eq SurfaceMount.exe" 2>NUL | find /i "SurfaceMount.exe" >NUL
if not errorlevel 1 (
    echo ERROR: SurfaceMount.exe is running. Close it first.
    pause
    exit /b 1
)

REM Back up stock MVision.dll on first install (don't clobber an existing backup).
if exist "%DST%\MVision.dll" if not exist "%DST%\MVision-orig.dll" (
    echo Backing up stock MVision.dll -^> MVision-orig.dll
    move /Y "%DST%\MVision.dll" "%DST%\MVision-orig.dll" >NUL
)

echo Copying binaries...
copy /Y "%SRC%MVision.dll"          "%DST%\" >NUL || goto :copyfail
REM also drop the no-AVX2 variant alongside so a BX/Atom user can rename it
REM to MVision.dll (the AVX2 build can't run there).
if exist "%SRC%MVision-noavx2.dll" (
    copy /Y "%SRC%MVision-noavx2.dll" "%DST%\" >NUL || goto :copyfail
)
copy /Y "%SRC%mvision_grabber.ax"   "%DST%\" >NUL || goto :copyfail
copy /Y "%SRC%saa7113-tune.exe"     "%DST%\" >NUL || goto :copyfail

REM 32-bit COM in-proc server -> use the 32-bit regsvr32 (SysWOW64 on 64-bit Win).
echo Registering mvision_grabber.ax with DirectShow...
if exist "%SystemRoot%\SysWOW64\regsvr32.exe" (
    "%SystemRoot%\SysWOW64\regsvr32.exe" /s "%DST%\mvision_grabber.ax" || goto :regfail
) else (
    "%SystemRoot%\System32\regsvr32.exe" /s "%DST%\mvision_grabber.ax" || goto :regfail
)

echo.
echo === INSTALL COMPLETE ===
echo.
echo MVision.dll        : detector replacement (always active when SurfaceMount runs)
echo mvision_grabber.ax : DirectShow source filter (now registered)
echo saa7113-tune.exe   : standalone chip tuner (optional, for diagnostic work)
echo.
echo === ONE-TIME PER MACHINE: Zadig WinUSB binding ===
echo The Camera tab in Ctrl+Alt+M needs the analog capture board to be bound
echo to libusbK instead of the stock Syntek driver. The detector + grabber
echo work without it, but real-hardware brightness/contrast/gain do not.
echo.
echo   1. Download Zadig from https://zadig.akeo.ie/  (single .exe, no install)
echo   2. Options -^> List All Devices
echo   3. You will see TWO entries for the capture board:
echo          USB2.0 Grabber (Interface 0)
echo          USB2.0 Grabber (Interface 1)
echo      For BOTH:
echo          - Pick the entry in the dropdown
echo          - Target driver = libusbK
echo          - Click Replace Driver
echo   4. Done. Launch SurfaceMount, press Ctrl+Alt+M, switch to Camera tab.
echo.
pause
exit /b 0

:copyfail
echo ERROR: copy failed.
pause
exit /b 1

:regfail
echo ERROR: regsvr32 failed. Run install.bat as Administrator?
pause
exit /b 1
