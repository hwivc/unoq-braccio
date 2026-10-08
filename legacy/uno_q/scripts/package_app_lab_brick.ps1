param(
  [string]$BrickName = "unoq_braccio_bridge"
)

$ErrorActionPreference = "Stop"

$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$BrickRoot = Join-Path $RepoRoot "app_lab\bricks\$BrickName"
$DistRoot = Join-Path $RepoRoot "dist"

if (-not (Test-Path $BrickRoot)) {
  throw "App Lab brick folder not found: $BrickRoot"
}

New-Item -ItemType Directory -Force -Path $DistRoot | Out-Null

$Stamp = Get-Date -Format "yyyyMMdd"
$ZipPath = Join-Path $DistRoot "$BrickName-brick-$Stamp.zip"

if (Test-Path $ZipPath) {
  Remove-Item $ZipPath
}

$Python = @'
import os
import sys
import zipfile
from pathlib import Path

root = Path(sys.argv[1]).resolve()
zip_path = Path(sys.argv[2]).resolve()

with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as archive:
    for path in sorted(root.rglob("*")):
        if path.is_dir():
            continue
        if "__pycache__" in path.parts or path.suffix == ".pyc":
            continue
        rel = path.relative_to(root.parent).as_posix()
        archive.write(path, rel)
'@

$Python | python - $BrickRoot $ZipPath

Write-Host "Created $ZipPath"
