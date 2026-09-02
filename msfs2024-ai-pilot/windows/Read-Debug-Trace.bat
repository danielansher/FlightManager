@echo off
REM ---------------------------------------------------------------------------
REM  Summarises the most recent flight trace: where it went, what it sent to
REM  the simulator, and what looks wrong.
REM ---------------------------------------------------------------------------
setlocal enabledelayedexpansion
cd /d "%~dp0\.."

if not exist "logs\*.jsonl" goto :none

set NEWEST=
for /f "delims=" %%F in ('dir /b /o-d "logs\*.jsonl"') do (
  if not defined NEWEST set NEWEST=%%F
)

echo Reading logs\!NEWEST!
echo.
python -m aipilot debug-report "logs\!NEWEST!"
echo.
echo To send this for diagnosis, attach the file itself:
echo   %CD%\logs\!NEWEST!
echo.
pause
exit /b 0

:none
echo.
echo No flight traces yet. Fly once with Fly-With-Debug.bat first.
pause
exit /b 1
