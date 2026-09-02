@echo off
REM ---------------------------------------------------------------------------
REM  Flies the flight plan you last made in SimBrief: the same airports, the
REM  same route, and the same runways at both ends.
REM
REM  MSFS must already be running, with the 787 loaded and the engines started.
REM  Your SimBrief username is the one you sign in with -- your numeric pilot
REM  ID from the SimBrief account page works just as well.
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0\.."

set /p SBUSER=Your SimBrief username (or pilot ID):
if "%SBUSER%"=="" goto :blank

echo.
echo Fetching your latest SimBrief plan...
echo Press Ctrl-C at any time to stop. The autopilot is left as it is.
echo.
python -m aipilot fly --simbrief %SBUSER% --aircraft b787-10 --msfs 2020
echo.
pause
exit /b 0

:blank
echo.
echo A SimBrief username is needed. Use Fly.bat instead to type the airports
echo in yourself.
pause
exit /b 1
