$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$arduinoCli = 'D:\Arduino\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'
$configPath = Join-Path $env:USERPROFILE '.arduinoIDE\arduino-cli.yaml'
if (-not (Test-Path -LiteralPath $arduinoCli)) {
    $arduinoCli = (Get-Command arduino-cli -ErrorAction Stop).Source
}
$buildPath = Join-Path $projectRoot 'build'
New-Item -ItemType Directory -Path $buildPath -Force | Out-Null
# The ESP32 linker cannot reliably create output files under Chinese paths.
$stagePath = Join-Path $env:USERPROFILE 'AppData\Local\Temp\HandExoLatestBuild'
if ($stagePath -match '[^\x00-\x7F]') { throw 'An ASCII temporary build path is required by the ESP32 linker.' }
$stageFirmware = Join-Path $stagePath 'firmware'
New-Item -ItemType Directory -Path $stageFirmware -Force | Out-Null
Write-Host 'Starting ESP32 firmware build. Detailed compiler output follows.'
Write-Host ('Project: ' + $projectRoot)
Write-Host ('Build directory: ' + $stagePath)
$buildStarted = Get-Date
$argsList = @('compile', '--verbose', '--fqbn', 'esp32:esp32:esp32:PartitionScheme=huge_app',
              '--build-path', $stagePath,
              '--output-dir', $stageFirmware,
              '--jobs', '4')
if (Test-Path -LiteralPath $configPath) { $argsList += @('--config-file', $configPath) }
$argsList += (Join-Path $projectRoot 'ServoDriver')
& $arduinoCli @argsList 2>&1 | Tee-Object -FilePath (Join-Path $buildPath 'compile-latest.log')
if ($LASTEXITCODE -ne 0) { throw 'Firmware build failed. Inspect the compiler output above.' }
$localFirmware = Join-Path $buildPath 'firmware'
New-Item -ItemType Directory -Path $localFirmware -Force | Out-Null
foreach ($file in Get-ChildItem -LiteralPath $stageFirmware -File) {
    Copy-Item -LiteralPath $file.FullName -Destination $localFirmware -Force
}
Write-Host ('Build completed in {0:N1} seconds.' -f ((Get-Date) - $buildStarted).TotalSeconds)

