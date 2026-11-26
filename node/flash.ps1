# Build and flash a pot node (ESP32-DevKitC-32E).
#
#   powershell -File node\flash.ps1 -Sketch bench\node_v1_0
#   powershell -File node\flash.ps1 -Sketch bench\node_v1_0 -Port COM75
#
# Writes <sketch>\version.h from git first (commit, date, uncommitted changes in node\), so the
# board's first serial line says which code it runs. Commit before the final flash.
param([Parameter(Mandatory = $true)][string]$Sketch, [string]$Port = "")
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$sketchDir = Join-Path $here $Sketch
$cli = (Get-Command arduino-cli -ErrorAction SilentlyContinue).Source
if (-not $cli) { $cli = "C:\Program Files\Arduino CLI\arduino-cli.exe" }
$fqbn = "esp32:esp32:esp32"

# --- version.h: "commit · 26 Nov 13:20 +2 uncommitted"
Push-Location $here
try {
  $commit = (git rev-parse --short HEAD).Trim()
  $date = [DateTimeOffset]::Parse((git log -1 --format=%cI).Trim()).ToString("dd MMM HH:mm")
  $changes = @(git status --porcelain -- . | Where-Object { $_ -notmatch 'version\.h$' }).Count
} finally { Pop-Location }
$dot = [char]0x00B7                 # middle dot, built here: PowerShell 5 reads this file as ANSI
$label = "$commit $dot $date" + $(if ($changes) { " +$changes uncommitted" } else { "" })
$h = "// written by flash.ps1 - do not edit, do not commit`n#define FW_VERSION `"$label`"`n"
[IO.File]::WriteAllText((Join-Path $sketchDir "version.h"), $h, (New-Object Text.UTF8Encoding $false))
Write-Host "version $label"

# --- the port: the one CP210x (10C4) or CH340 (1A86) USB-serial bridge, unless told
if (-not $Port) {
  $dev = @(Get-CimInstance Win32_PnPEntity | Where-Object {
      ($_.DeviceID -like "USB\VID_10C4*" -or $_.DeviceID -like "USB\VID_1A86*") -and $_.Name -match '\((COM\d+)\)' })
  if ($dev.Count -ne 1) { throw "Found $($dev.Count) USB-serial bridges; say which with -Port COMn" }
  $Port = [regex]::Match($dev[0].Name, 'COM\d+').Value
}
Write-Host "port $Port"

& $cli compile --fqbn $fqbn $sketchDir
if ($LASTEXITCODE) { throw "compile failed" }
& $cli upload -p $Port --fqbn $fqbn $sketchDir
if ($LASTEXITCODE) { throw "upload failed (serial monitor holding ${Port}? or hold BOOT while it says Connecting)" }

# --- show the boot line, so the version that landed is on screen
Start-Sleep -Seconds 1
$sp = New-Object System.IO.Ports.SerialPort $Port, 115200
$sp.ReadTimeout = 4000
try {
  $sp.Open()
  $sp.DtrEnable = $false; $sp.RtsEnable = $true; Start-Sleep -Milliseconds 100; $sp.RtsEnable = $false  # EN reset
  $t = [DateTime]::Now
  while (([DateTime]::Now - $t).TotalSeconds -lt 5) {
    $line = $sp.ReadLine().Trim()
    if ($line -like "node_v*") { Write-Host $line; break }
  }
} catch { Write-Host "(no boot line read: $($_.Exception.Message))" } finally { $sp.Close() }
