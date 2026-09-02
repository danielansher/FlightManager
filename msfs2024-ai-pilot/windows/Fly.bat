@echo off
REM ---------------------------------------------------------------------------
REM  Asks where you want to go, then flies there.
REM  MSFS must already be running, with the 787 on a runway, engines started.
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0\.."

set /p FROM=Departure airport (ICAO, e.g. EGLL):
set /p TO=Destination airport (ICAO, e.g. LFPG):
if "%FROM%"=="" goto :blank
if "%TO%"=="" goto :blank

echo.
echo Flying %FROM% to %TO% in the 787-10.
echo Press Ctrl-C at any time to stop. The autopilot is left as it is.
echo.
python -m aipilot fly %FROM% %TO% --aircraft b787-10 --msfs 2020
echo.
pause
exit /b 0

:blank
echo.
echo Both airports are needed.
pause
exit /b 1
