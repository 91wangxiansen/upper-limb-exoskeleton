@echo off
cd /d "%~dp0"
set "PYTHON=py"
py --version >nul 2>&1
if errorlevel 1 set "PYTHON=C:\Users\24971\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if not exist "%~dp0.deps\serial\__init__.py" (
  "%PYTHON%" -m pip install --target "%~dp0.deps" -r requirements.txt
  if errorlevel 1 (
    echo Failed to install the serial communication dependency.
    pause
    exit /b 1
  )
)
set "PYTHONPATH=%~dp0.deps;%PYTHONPATH%"
"%PYTHON%" "%~dp0digital_twin_server.py"
