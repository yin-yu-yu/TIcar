# Erase MSPM0G3507 via J-Link @ 192.168.31.56
#
# Prerequisites: JLink.exe must be in PATH, or set $env:JLINK_PATH below.

$JLink = if ($env:JLINK_PATH) { $env:JLINK_PATH } else { "JLink.exe" }
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

Write-Host "Erasing MSPM0G3507 via J-Link @ 192.168.31.56..."
& $JLink -CommandFile "$ScriptDir\erase_mspm0g3507.jlink" -AutoConnect 0
