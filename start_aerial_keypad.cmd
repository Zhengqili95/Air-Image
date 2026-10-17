@echo off
rem Start the aerial keypad from the folder that holds the program, where it finds the config,
rem model and DLLs. Works both in a source checkout (build\windows-release) and in a release zip
rem (program next to this script). The window stays open after the program exits so any error
rem message can still be read.
set "APP_DIR=%~dp0"
if not exist "%APP_DIR%aerial_touch_app.exe" set "APP_DIR=%~dp0build\windows-release\"
if not exist "%APP_DIR%aerial_touch_app.exe" (
    echo aerial_touch_app.exe was not found next to this script or in build\windows-release. Build the project first.
    pause >nul
    exit /b 1
)
cd /d "%APP_DIR%"
aerial_touch_app.exe
echo.
echo The program has exited. Read any error above, then press any key to close this window.
pause >nul
