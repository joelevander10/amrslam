# Export JIS sheets to PNG (A4, 300 dpi) and vector PDF with headless Edge/Chrome.
# Usage (from anywhere):
#   pwsh -File diagrams/agent/scripts/export.ps1                    # every sheet except templates
#   pwsh -File diagrams/agent/scripts/export.ps1 fw30hs-t-sheet-07   # one sheet (name without .html)
# Output: diagrams/export/<name>.png and <name>.pdf
param([Parameter(ValueFromRemainingArguments = $true)][string[]]$Names)

$diagrams = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$sheets = Join-Path $diagrams 'sheets'
$out = Join-Path $diagrams 'export'
New-Item -ItemType Directory -Force $out | Out-Null

$browser = @(
  "${env:ProgramFiles(x86)}\Microsoft\Edge\Application\msedge.exe",
  "$env:ProgramFiles\Microsoft\Edge\Application\msedge.exe",
  "$env:ProgramFiles\Google\Chrome\Application\chrome.exe"
) | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $browser) { throw 'Edge or Chrome not found.' }

if (-not $Names) {
  $Names = Get-ChildItem $sheets -Filter '*.html' | Where-Object { $_.Name -notlike '_*' } | ForEach-Object { $_.BaseName }
}

foreach ($n in $Names) {
  $file = Join-Path $sheets "$n.html"
  if (-not (Test-Path $file)) { Write-Warning "missing: $file"; continue }
  $url = ([Uri]$file).AbsoluteUri
  $png = Join-Path $out "$n.png"
  $pdf = Join-Path $out "$n.pdf"
  Remove-Item $png, $pdf -ErrorAction SilentlyContinue

  # '#bare' shows the sheet alone at 1240x1754 css px; scale 2 → 2480x3508 px
  & $browser --headless=new --disable-gpu --hide-scrollbars --window-size=1240,1754 `
    --force-device-scale-factor=2 --virtual-time-budget=8000 --screenshot="$png" "$url#bare" 2>$null
  & $browser --headless=new --disable-gpu --no-pdf-header-footer --virtual-time-budget=8000 `
    --print-to-pdf="$pdf" "$url" 2>$null

  # headless Edge can return before the file is flushed
  $t = 0; while (-not ((Test-Path $png) -and (Test-Path $pdf)) -and $t -lt 20) { Start-Sleep -Milliseconds 500; $t++ }
  "{0,-32} png:{1} pdf:{2}" -f $n, (Test-Path $png), (Test-Path $pdf)
}
