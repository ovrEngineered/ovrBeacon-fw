@echo off
bgbuild ovrBeacon.bgproj
mkdir output
move ovrBeacon.hex output
move ovrBeacon.ota output
pause>nul