@echo off
REM ---------------------------------------------------------------------------
REM  Flies a complete London to Manchester flight with no simulator involved,
REM  in about fifteen seconds. Nothing here touches MSFS. Use it to see what
REM  the AI Pilot does before connecting it to anything.
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0\.."
python -m aipilot fly EGLL EGCC --aircraft b787-10 --sim mock --speed 200 --quiet
echo.
pause
