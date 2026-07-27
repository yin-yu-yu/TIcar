param(
    [string]$ProbeIP = "192.168.31.56"
)

$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent $PSScriptRoot
$image = Join-Path $projectRoot "Debug\empty_LP_MSPM0G3507_nortos_ticlang.out"
$commandFile = Join-Path $PSScriptRoot "flash_mspm0g3507.jlink"

$candidates = @(
    (Join-Path $env:ProgramFiles "SEGGER\JLink\JLink.exe"),
    "JLink.exe"
)

$jlink = $null
foreach ($candidate in $candidates) {
    if ($candidate -eq "JLink.exe") {
        $resolved = Get-Command $candidate -ErrorAction SilentlyContinue
        if ($resolved) { $jlink = $resolved.Source; break }
    } elseif (Test-Path -LiteralPath $candidate) {
        $jlink = $candidate
        break
    }
}

if (-not $jlink) {
    throw "JLink.exe not found. Install SEGGER J-Link Software or add it to PATH."
}
if (-not (Test-Path -LiteralPath $image)) {
    throw "Build output not found: $image. Build the CCS project first."
}

Push-Location $projectRoot
try {
    & $jlink `
        -IP $ProbeIP `
        -Device MSPM0G3507 `
        -If SWD `
        -Speed 1000 `
        -AutoConnect 1 `
        -ExitOnError 1 `
        -NoGui 1 `
        -CommandFile $commandFile

    if ($LASTEXITCODE -ne 0) {
        throw "J-Link programming failed with exit code $LASTEXITCODE."
    }
} finally {
    Pop-Location
}
