@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo Run Setup-Windows.cmd first.
    pause
    exit /b 1
)
".venv\Scripts\python.exe" "app.py" %*
set "ROVER_RUN_RESULT=%errorlevel%"
if not "%ROVER_RUN_RESULT%"=="0" pause
exit /b %ROVER_RUN_RESULT%
