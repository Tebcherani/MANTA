@echo off
REM ===========================================================
REM  Review the most recent .obd file
REM
REM  Searches ..\Data\ (the simulator's default output folder)
REM  for the newest .obd and reviews it. Useful right after
REM  running sim.exe.
REM
REM  Looks in: ..\Data, then .\Data, then current folder.
REM ===========================================================
setlocal enabledelayedexpansion

cd /d "%~dp0"

set "OPTS=2,3,8,9"

if not exist ".venv\Scripts\activate.bat" (
    echo Virtual environment not found.
    echo Please run setup.bat once before using this script.
    pause
    exit /b 1
)

call .venv\Scripts\activate.bat

REM ----- Find newest .obd in candidate directories ------------
set "OBDFILE="
for %%D in ("..\Data" ".\Data" ".") do (
    if exist "%%~D\*.obd" (
        for /f "delims=" %%F in ('dir /b /o-d /a-d "%%~D\*.obd" 2^>nul') do (
            if not defined OBDFILE set "OBDFILE=%%~D\%%F"
        )
    )
)

if "%OBDFILE%"=="" (
    echo No .obd files found in ..\Data, .\Data, or current folder.
    pause
    exit /b 1
)

echo.
echo Newest .obd: %OBDFILE%
echo Options:    %OPTS%
echo.
python -m ody_review.review "%OBDFILE%" --opts %OPTS%

if errorlevel 1 (
    echo.
    echo Review exited with an error.
)

echo.
pause
