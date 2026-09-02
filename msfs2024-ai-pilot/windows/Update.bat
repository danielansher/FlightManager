@echo off
REM ---------------------------------------------------------------------------
REM  Fetches the latest version, if you cloned this with git.
REM  If you downloaded a ZIP instead, this will tell you so.
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0\.."

if not exist ".git" (
  echo.
  echo This folder was not cloned with git, so there is nothing to pull.
  echo Download the ZIP again from GitHub, and remember to copy across:
  echo    SimConnect.dll        ^(if you put one here^)
  echo    airports.csv / runways.csv  ^(if you downloaded them^)
  echo.
  pause
  exit /b 1
)

echo Fetching the latest version...
git pull
echo.
python -m aipilot --version
echo.
pause
