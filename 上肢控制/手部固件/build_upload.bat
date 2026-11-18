@echo off
cd /d "%~dp0"
echo Starting build and upload for COM14...
echo Project directory: %CD%
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\upload.ps1" -Port COM14
if errorlevel 1 (
  echo Build or upload failed. Keep this window open to read the error.
) else (
  echo Build and upload completed.
)
pause
