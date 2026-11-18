@echo off
cd /d "%~dp0"
echo Starting local ESP32 debug bridge...
python debug_bridge.py
if errorlevel 1 pause
