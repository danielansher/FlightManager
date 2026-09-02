@echo off
REM ---------------------------------------------------------------------------
REM  The same as Fly.bat, but it records everything to a file.
REM
REM  Use this when something has gone wrong and you want it looked at. The
REM  trace holds what the aeroplane was doing, what the AI Pilot commanded, and
REM  every event it sent to the simulator -- which is what actually finds a
REM  fault. It goes in the logs folder. Nothing personal is in it; folder paths
REM  have your user name taken out.
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0\.."

set /p FROM=Departure airport (ICAO, e.g. EGLL):
set /p TO=Destination airport (ICAO, e.g. LFPG):
if "%FROM%"=="" goto :blank
if "%TO%"=="" goto :blank

echo.
echo Flying %FROM% to %TO% in the 787-10, recording as it goes.
echo Press Ctrl-C at any time to stop. The trace is still written.
echo.
python -m aipilot fly %FROM% %TO% --aircraft b787-10 --msfs 2020 --debug
echo.
echo The trace is in the logs folder. Read-Debug-Trace.bat summarises the
echo newest one; send the file itself if you want it looked at properly.
echo.
pause
exit /b 0

:blank
echo.
echo Both airports are needed.
pause
exit /b 1
