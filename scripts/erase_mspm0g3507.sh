#!/bin/bash
# Erase MSPM0G3507 via J-Link @ 192.168.31.56
#
# Prerequisites: JLink.exe must be in PATH or set JLINK_PATH below.

JLINK="${JLINK_PATH:-JLink.exe}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Erasing MSPM0G3507 via J-Link @ 192.168.31.56..."
"$JLINK" -CommandFile "$SCRIPT_DIR/erase_mspm0g3507.jlink" -AutoConnect 0
