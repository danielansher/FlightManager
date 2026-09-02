@echo off
REM ---------------------------------------------------------------------------
REM  Opens the AI Pilot control panel in your browser.
REM  Start MSFS first, load a flight, and leave it running on a runway.
REM  Then double-click this file.
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0\.."

where python >nul 2>nul
if errorlevel 1 (
  echo.
  echo Python was not found.
  echo Install it from https://www.python.org/downloads/ and tick
  echo "Add Python to PATH" during setup, then run this again.
  echo.
  pause
  exit /b 1
)

echo Starting the AI Pilot control panel...
echo Leave this window open while you fly. Close it to stop.
echo.
python -m aipilot ui --open
pause
