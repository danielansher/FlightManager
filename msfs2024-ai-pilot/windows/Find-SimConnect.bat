@echo off
REM ---------------------------------------------------------------------------
REM  Searches this PC for SimConnect.dll and puts it where the AI Pilot can
REM  find it. Several tools bundle a copy -- Little Navmap and MobiFlight
REM  among them -- so there is a good chance one is already here.
REM
REM  Run this once, if Check-My-Setup said SimConnect.dll was missing.
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0\.."
python -m aipilot find-simconnect --copy
echo.
pause
