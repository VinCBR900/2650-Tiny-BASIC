@echo off
SET PATH=C:\Users\VincentCrabtree\Desktop\Utils\bin;%PATH%
tcc -O2 -o asm2650.exe asm2650.c
tcc -O2 -DGAMER -o pipbug_wrap.exe pipbug_wrap.c
rem dir pipbug_wrap.* asm2650.*
powershell -NoProfile -Command ^
  "Get-ChildItem -Path 'asm2650.*','pipbug_wrap.*' -ErrorAction SilentlyContinue | Format-Table Name, LastWriteTime, Length -AutoSize"
copy *.exe ..
pause