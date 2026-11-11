@echo off
rem Drag one or more .csv files onto this file: it opens rviz2 with the fixed map and your CSVs.
rem Double-click (no file): shows the map + path.csv.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0drop_view.ps1" %*
if errorlevel 1 pause
