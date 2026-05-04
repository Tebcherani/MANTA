@echo off
REM ===========================================================
REM  ody_review setup
REM  Run this once. Creates a Python virtual environment in
REM  .venv\ and installs the packages ody_review needs.
REM ===========================================================
setlocal

cd /d "%~dp0"

echo.
echo === ody_review setup ===
echo.

REM ----- Pick a Python launcher -------------------------------
where py >nul 2>&1
if %errorlevel%==0 (
    set "PYCMD=py -3"
    goto have_python
)
where python >nul 2>&1
if %errorlevel%==0 (
    set "PYCMD=python"
    goto have_python
)

echo ERROR: Python was not found on PATH.
echo Install Python 3.10 or newer from https://www.python.org/downloads/
echo and tick "Add Python to PATH" during install. Then re-run this script.
pause
exit /b 1

:have_python
echo Using: %PYCMD%
%PYCMD% --version
echo.

REM ----- Create venv if missing -------------------------------
if not exist ".venv\Scripts\activate.bat" (
    echo Creating virtual environment in .venv ...
    %PYCMD% -m venv .venv
    if errorlevel 1 (
        echo ERROR: failed to create virtual environment.
        pause
        exit /b 1
    )
) else (
    echo Virtual environment already exists, skipping creation.
)

REM ----- Install / upgrade packages ---------------------------
echo.
echo Installing dependencies ...
call .venv\Scripts\activate.bat
python -m pip install --upgrade pip
python -m pip install numpy xarray matplotlib pyqt5
python -m pip install gsw
if errorlevel 1 (
    echo.
    echo NOTE: gsw failed to install. The package will still work using
    echo the built-in UNESCO seawater fallback. Continuing.
)

echo.
echo === Setup complete ===
echo.
echo You can now double-click review.bat to review a file,
echo or drag any .obd file onto review.bat.
echo.
pause
