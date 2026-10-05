# Compile and upload firmware/braccio_uno_firmware to an Arduino UNO.
#   .\scripts\flash_uno.ps1                       # UNO R3 on COM3
#   .\scripts\flash_uno.ps1 -Port COM5
#   .\scripts\flash_uno.ps1 -Port COM5 -Fqbn arduino:renesas_uno:minima   # UNO R4 Minima
param(
    [string]$Port = "COM3",
    [string]$Fqbn = "arduino:avr:uno"
)

$ErrorActionPreference = "Stop"
$RepoRoot = Resolve-Path "$PSScriptRoot\.."
$SketchDir = Join-Path $RepoRoot "firmware\braccio_uno_firmware"
$Core = $Fqbn.Substring(0, $Fqbn.LastIndexOf(":"))

arduino-cli core update-index
arduino-cli core install $Core
arduino-cli lib install Servo
arduino-cli compile --fqbn $Fqbn $SketchDir
arduino-cli upload -p $Port --fqbn $Fqbn $SketchDir
