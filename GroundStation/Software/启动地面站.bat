@echo off
setlocal
cd /d "%~dp0"
title LXS1 Ground Station

set "PYTHON_EXE="
set "PYTHON_ARGS="

if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" -c "import tkinter, PIL, serial" >nul 2>nul
    if not errorlevel 1 set "PYTHON_EXE=%CD%\.venv\Scripts\python.exe"
)

if not defined PYTHON_EXE (
    for /f "delims=" %%P in ('where python 2^>nul') do (
        "%%P" -c "import tkinter, PIL, serial" >nul 2>nul
        if not errorlevel 1 if not defined PYTHON_EXE set "PYTHON_EXE=%%P"
    )
)

if not defined PYTHON_EXE (
    where py >nul 2>nul
    if not errorlevel 1 (
        py -3 -c "import tkinter, PIL, serial" >nul 2>nul
        if not errorlevel 1 set "PYTHON_EXE=py"
        if not errorlevel 1 set "PYTHON_ARGS=-3"
    )
)

if not defined PYTHON_EXE goto :install
goto :run

:install
echo No ready Python environment was found.
echo Creating .venv and installing dependencies...
set "BASE_PYTHON="
for /f "delims=" %%P in ('where python 2^>nul') do (
    "%%P" -c "import tkinter" >nul 2>nul
    if not errorlevel 1 "%%P" -m pip --version >nul 2>nul
    if not errorlevel 1 if not defined BASE_PYTHON set "BASE_PYTHON=%%P"
)

if not defined BASE_PYTHON (
    echo.
    echo ERROR: Python with Tk and pip was not found.
    echo Install Python 3.10 or newer from python.org and enable Add Python to PATH.
    pause
    exit /b 1
)

"%BASE_PYTHON%" -m venv .venv
if errorlevel 1 goto :failed
".venv\Scripts\python.exe" -m pip install -r requirements.txt
if errorlevel 1 goto :failed
set "PYTHON_EXE=%CD%\.venv\Scripts\python.exe"

:run
echo Starting LXS1 Ground Station with:
echo %PYTHON_EXE% %PYTHON_ARGS%
echo.
"%PYTHON_EXE%" %PYTHON_ARGS% main.py
if errorlevel 1 goto :failed
exit /b 0

:failed
echo.
echo ERROR: Ground station failed to start. Review the error above.
pause
exit /b 1
