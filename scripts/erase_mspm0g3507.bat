@echo off
REM Erase MSPM0G3507 via J-Link @ 192.168.31.56
REM
REM Prerequisites: JLink.exe must be in PATH, or set JLINK_PATH below.

if not defined JLINK_PATH set JLINK_PATH=JLink.exe

echo Erasing MSPM0G3507 via J-Link @ 192.168.31.56...
"%JLINK_PATH%" -CommandFile "%~dp0erase_mspm0g3507.jlink" -AutoConnect 0
