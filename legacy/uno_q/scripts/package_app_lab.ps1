param(
    [string]$AppName = "braccio_web_agent"
)

$ErrorActionPreference = "Stop"
$RepoRoot = Resolve-Path "$PSScriptRoot\.."
$AppDir = Join-Path $RepoRoot "app_lab\$AppName"
$DistDir = Join-Path $RepoRoot "dist"
$Date = Get-Date -Format yyyyMMdd
$ZipPath = Join-Path $DistDir "$AppName-app_lab-$Date.zip"
$TempParent = Join-Path $env:TEMP "unoq-braccio-applab-package"
$TempDir = Join-Path $TempParent $AppName

if (!(Test-Path $AppDir)) {
    throw "App Lab app not found: $AppDir"
}

Remove-Item -LiteralPath $TempParent -Recurse -Force -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force -Path $TempDir | Out-Null
New-Item -ItemType Directory -Force -Path $DistDir | Out-Null

robocopy $AppDir $TempDir /E /XD __pycache__ /XF *.pyc | Out-Null
if ($LASTEXITCODE -ge 8) {
    throw "robocopy failed with exit code $LASTEXITCODE"
}

@"
import os
import zipfile
from pathlib import Path

root = Path(r"$TempDir")
zip_path = Path(r"$ZipPath")
if zip_path.exists():
    zip_path.unlink()

with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
    for path in sorted(root.rglob("*")):
        if path.is_dir():
            continue
        rel = path.relative_to(root.parent).as_posix()
        archive.write(path, rel)
"@ | python -

Remove-Item -LiteralPath $TempParent -Recurse -Force

Write-Output $ZipPath
