param([string]$Port = 'COM14')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
& (Join-Path $PSScriptRoot 'build.ps1')
if (-not $?) { throw 'Build failed. Upload cancelled.' }
$arduinoCli = 'D:\Arduino\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'
if (-not (Test-Path -LiteralPath $arduinoCli)) {
    $arduinoCli = (Get-Command arduino-cli -ErrorAction Stop).Source
}
$configPath = Join-Path $env:USERPROFILE '.arduinoIDE\arduino-cli.yaml'
$stageFirmware = Join-Path $env:USERPROFILE 'AppData\Local\Temp\HandExoLatestBuild\firmware'
$uploadArgs = @('upload', '--fqbn', 'esp32:esp32:esp32:PartitionScheme=huge_app,UploadSpeed=115200',
                '--port', $Port, '--input-dir', $stageFirmware)
if (Test-Path -LiteralPath $configPath) { $uploadArgs += @('--config-file', $configPath) }
& $arduinoCli @uploadArgs
if ($LASTEXITCODE -ne 0) { throw 'Upload failed. Inspect the serial-port error above.' }
Write-Host 'Upload complete. Connect to Hand-Exo Wi-Fi and open http://192.168.4.1'
