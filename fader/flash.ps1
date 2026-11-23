# Build and flash the TL fader (ESP32-S3 SuperMini).
#
#   powershell -File fader\flash.ps1              finds the SuperMini by its USB id (303A)
#   powershell -File fader\flash.ps1 -Port COM4
#
# Writes homeostat_fader\version.h from git first (commit, date, uncommitted changes in fader\),
# so the board can say which code it runs: first serial line, the editor's /fader page, the sim.
param([string]$Port = "")
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$sketch = Join-Path $here "homeostat_fader"
$cli = (Get-Command arduino-cli -ErrorAction SilentlyContinue).Source
if (-not $cli) { $cli = "C:\Program Files\Arduino CLI\arduino-cli.exe" }
$fqbn = "esp32:esp32:esp32s3:CDCOnBoot=cdc"

# --- version.h: same shape as arm/version.py's label: "commit · 23 Nov 13:20 +2 uncommitted"
Push-Location $here
try {
  $commit = (git rev-parse --short HEAD).Trim()
  $date = [DateTimeOffset]::Parse((git log -1 --format=%cI).Trim()).ToString("dd MMM HH:mm")
  $changes = @(git status --porcelain -- . | Where-Object { $_ -notmatch 'version\.h$' }).Count
} finally { Pop-Location }
$dot = [char]0x00B7                 # middle dot, built here: PowerShell 5 reads this file as ANSI
$label = "$commit $dot $date" + $(if ($changes) { " +$changes uncommitted" } else { "" })
$h = "// written by flash.ps1 - do not edit, do not commit`n#define FW_VERSION `"$label`"`n"
[IO.File]::WriteAllText((Join-Path $sketch "version.h"), $h, (New-Object Text.UTF8Encoding $false))
Write-Host "version $label"

# --- the port: the one Espressif native-USB device, unless told
if (-not $Port) {
  $dev = @(Get-CimInstance Win32_PnPEntity | Where-Object { $_.DeviceID -like "USB\VID_303A*" -and $_.Name -match '\((COM\d+)\)' })
  if ($dev.Count -ne 1) { throw "Found $($dev.Count) ESP32-S3 USB ports; say which with -Port COMn" }
  $Port = [regex]::Match($dev[0].Name, 'COM\d+').Value
}
Write-Host "port $Port"

& $cli compile --fqbn $fqbn $sketch
if ($LASTEXITCODE) { throw "compile failed" }
& $cli upload -p $Port --fqbn $fqbn $sketch
if ($LASTEXITCODE) { throw "upload failed (is the editor's bridge or a serial monitor holding $Port?)" }

# --- show the boot line, so the version that landed is on screen
Start-Sleep -Seconds 2
$sp = New-Object System.IO.Ports.SerialPort $Port, 115200
$sp.DtrEnable = $true; $sp.ReadTimeout = 4000
try {
  $sp.Open(); $t = [DateTime]::Now
  while (([DateTime]::Now - $t).TotalSeconds -lt 5) {
    $line = $sp.ReadLine().Trim()
    if ($line -like "homeostat_fader*") { Write-Host $line; break }
  }
} catch { Write-Host "(no boot line read: $($_.Exception.Message))" } finally { $sp.Close() }
