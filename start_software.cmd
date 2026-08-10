@echo off
setlocal
cd /d "%~dp0"

if not exist "%~dp0run_integrated_app.ps1" (
    echo [ERROR] Launcher not found: "%~dp0run_integrated_app.ps1"
    pause
    exit /b 1
)

if /I "%~1"=="--check" (
    echo Launcher found: "%~dp0run_integrated_app.ps1"
    echo Working directory: "%CD%"
    exit /b 0
)

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_integrated_app.ps1"
set "APP_EXIT_CODE=%ERRORLEVEL%"

if not "%APP_EXIT_CODE%"=="0" (
    echo.
    echo [ERROR] Software exited with code %APP_EXIT_CODE%.
    echo Review the message above, then press any key to close this window.
    pause >nul
)

exit /b %APP_EXIT_CODE%
