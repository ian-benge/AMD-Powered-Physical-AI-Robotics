@echo off
setlocal
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" (
    set "ROVER_PYTHON=.venv\Scripts\python.exe"
    goto setup
)
py -3.12 --version >nul 2>&1
if not errorlevel 1 (
    set "ROVER_PYTHON=py -3.12"
    goto setup
)
py -3.11 --version >nul 2>&1
if not errorlevel 1 (
    set "ROVER_PYTHON=py -3.11"
    goto setup
)
echo Python 3.12 or 3.11 was not found through the Windows Python launcher.
echo Install Python Install Manager from https://www.python.org/downloads/windows/
echo Then open PowerShell and run: pymanager install 3.12
echo Reopen this setup file after Python finishes installing.
echo If Python is installed without the launcher, see README.md for manual setup.
pause
exit /b 1
:setup
%ROVER_PYTHON% "scripts\setup_environment.py"
set "ROVER_SETUP_RESULT=%errorlevel%"
if not "%ROVER_SETUP_RESULT%"=="0" echo Setup stopped. Keep the error text above for troubleshooting.
pause
exit /b %ROVER_SETUP_RESULT%
