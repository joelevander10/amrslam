# Render a throw-away PNG of one or more sheets for the visual check. Nothing is written to the project.
# Usage:
#   pwsh -File diagrams/agent/scripts/preview.ps1 <path-to-sheet.html> [...]
# Output: $env:TEMP\jis-preview\<name>.png (2480 x 3508). Read that file to check the render.
# Deliverable files (PNG/PDF in the project) come from export.ps1, only when the user asks.
param([Parameter(ValueFromRemainingArguments = $true)][string[]]$Files)

$out = Join-Path $env:TEMP 'jis-preview'
New-Item -ItemType Directory -Force $out | Out-Null

$browser = @(
  "${env:ProgramFiles(x86)}\Microsoft\Edge\Application\msedge.exe",
  "$env:ProgramFiles\Microsoft\Edge\Application\msedge.exe",
  "$env:ProgramFiles\Google\Chrome\Application\chrome.exe"
) | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $browser) { throw 'Edge or Chrome not found.' }

foreach ($f in $Files) {
  $file = Resolve-Path $f -ErrorAction SilentlyContinue
  if (-not $file) { Write-Warning "missing: $f"; continue }
  $name = [IO.Path]::GetFileNameWithoutExtension($file)
  $png = Join-Path $out "$name.png"
  Remove-Item $png -ErrorAction SilentlyContinue

  & $browser --headless=new --disable-gpu --hide-scrollbars --window-size=1240,1754 `
    --force-device-scale-factor=2 --virtual-time-budget=8000 --screenshot="$png" "$(([Uri]$file.Path).AbsoluteUri)#bare" 2>$null

  $t = 0; while (-not (Test-Path $png) -and $t -lt 20) { Start-Sleep -Milliseconds 500; $t++ }
  if (Test-Path $png) { $png } else { Write-Warning "no render: $f" }
}
