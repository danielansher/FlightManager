@echo off
REM ---------------------------------------------------------------------------
REM  Checks everything: Python, SimConnect, which simulator answered, the
REM  navigation data, and the WASM bridge. Run this first, and run it again
REM  whenever something does not work.
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0\.."

where python >nul 2>nul
if errorlevel 1 (
  echo Python was not found. Install it from https://www.python.org/downloads/
  echo and tick "Add Python to PATH" during setup.
  pause
  exit /b 1
)

python -m aipilot doctor --msfs 2020
echo.
pause
