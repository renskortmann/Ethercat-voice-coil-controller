@echo off
rem Start the VCA simulator app and open it in the browser. Double-click this file, or run it from a terminal.
rem Close this window (or press Ctrl+C) to stop the app.
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo No .venv found. Create it first:  python -m venv .venv  then  .venv\Scripts\pip install -r requirements.txt
    pause
    exit /b 1
)
".venv\Scripts\python.exe" -m vca_sim.app %*
if errorlevel 1 pause
