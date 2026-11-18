@echo off
cd /d "%~dp0"
set "PYTHON=py"
py --version >nul 2>&1
if errorlevel 1 set "PYTHON=C:\Users\24971\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
"%PYTHON%" -m pip install --target "%~dp0.deps" -r requirements.txt
set "PYTHONPATH=%~dp0.deps;%PYTHONPATH%"
"%PYTHON%" upper_computer.py
