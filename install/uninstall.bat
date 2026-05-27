@echo off
REM Reverse the install: unregister the grabber, restore the original
REM MVision.dll, delete the extras. Leaves the Zadig binding alone (it's a
REM per-device thing; if you want to undo it, use Zadig to swap the driver
REM back, or Device Manager -> Properties -> Driver -> Uninstall device).

setlocal

if "%~1"=="" (
    set "DST=%USERPROFILE%\qihetvm802b"
) else (
    set "DST=%~1"
)

echo === tvm802-vision uninstaller ===
echo Target : %DST%
echo.

tasklist /fi "imagename eq SurfaceMount.exe" 2>NUL | find /i "SurfaceMount.exe" >NUL
if not errorlevel 1 (
    echo ERROR: SurfaceMount.exe is running. Close it first.
    pause
    exit /b 1
)

if exist "%DST%\mvision_grabber.ax" (
    echo Unregistering mvision_grabber.ax...
    if exist "%SystemRoot%\SysWOW64\regsvr32.exe" (
        "%SystemRoot%\SysWOW64\regsvr32.exe" /s /u "%DST%\mvision_grabber.ax"
    ) else (
        "%SystemRoot%\System32\regsvr32.exe" /s /u "%DST%\mvision_grabber.ax"
    )
    del /f /q "%DST%\mvision_grabber.ax"
)
if exist "%DST%\saa7113-tune.exe" del /f /q "%DST%\saa7113-tune.exe"

if exist "%DST%\MVision-orig.dll" (
    echo Restoring stock MVision.dll from MVision-orig.dll
    del /f /q "%DST%\MVision.dll" 2>NUL
    move /Y "%DST%\MVision-orig.dll" "%DST%\MVision.dll" >NUL
)

echo.
echo === UNINSTALL COMPLETE ===
echo Zadig binding NOT touched. If you want to restore the stock Syntek
echo driver: Device Manager -^> Sound, video... -^> USB2.0 Grabber -^> right-
echo click -^> Uninstall device -^> tick 'Delete the driver software' -^>
echo unplug + replug the capture board. Windows will install the stock
echo driver from its driver store.
echo.
pause
exit /b 0
