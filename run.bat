@echo off
start "" renode simulate.resc
timeout /t 2 /nobreak >nul
start cmd /k "nc.exe -C localhost 3456"