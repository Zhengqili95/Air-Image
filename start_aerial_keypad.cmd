@echo off
rem Start the aerial keypad from its output folder, where it finds the config, model and DLLs.
rem %~dp0 is this script's own folder, so the launcher works wherever the project is checked out.
rem The window stays open after the program exits so any error message can still be read.
cd /d "%~dp0build\windows-release"
if not exist aerial_touch_app.exe (
    echo aerial_touch_app.exe was not found in "%~dp0build\windows-release". Build the project first.
    pause >nul
    exit /b 1
)
aerial_touch_app.exe
echo.
echo The program has exited. Read any error above, then press any key to close this window.
pause >nul
