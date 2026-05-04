@echo off
REM ===========================================================
REM  ody_review launcher
REM
REM  Two ways to use:
REM    1. Drag an .obd file onto this script's icon.
REM    2. Double-click. A file picker will appear.
REM
REM  Default plot options: 2,3,8,9
REM    (vertical plane, heading, CTD time series, T/S/SOS)
REM  Edit OPTS below to change. Use "all" for every plot.
REM ===========================================================
setlocal

cd /d "%~dp0"

set "OPTS=2,3,8,9"

REM ----- Make sure setup has been run -------------------------
if not exist ".venv\Scripts\activate.bat" (
    echo Virtual environment not found.
    echo Please run setup.bat once before using this script.
    pause
    exit /b 1
)

call .venv\Scripts\activate.bat

REM ----- Get the .obd file path -------------------------------
set "OBDFILE=%~1"

if "%OBDFILE%"=="" (
    REM No file dropped — open a file picker via PowerShell
    echo Opening file picker ...
    for /f "usebackq delims=" %%i in (`powershell -NoProfile -ExecutionPolicy Bypass -Command ^
        "Add-Type -AssemblyName System.Windows.Forms; ^
         $f = New-Object System.Windows.Forms.OpenFileDialog; ^
         $f.Filter = 'Odyssey Binary Data (*.obd)^|*.obd^|All files (*.*)^|*.*'; ^
         $f.Title  = 'Select an .obd file to review'; ^
         $f.InitialDirectory = (Resolve-Path '..\Data' -ErrorAction SilentlyContinue); ^
         if ($f.ShowDialog() -eq 'OK') { Write-Output $f.FileName }"`) do (
        set "OBDFILE=%%i"
    )
)

if "%OBDFILE%"=="" (
    echo No file selected. Exiting.
    pause
    exit /b 0
)

if not exist "%OBDFILE%" (
    echo ERROR: file not found: %OBDFILE%
    pause
    exit /b 1
)

echo.
echo Reviewing: %OBDFILE%
echo Options:   %OPTS%
echo.
python -m ody_review.review "%OBDFILE%" --opts %OPTS%

if errorlevel 1 (
    echo.
    echo Review exited with an error.
)

echo.
pause
