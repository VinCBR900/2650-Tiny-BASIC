@echo off
tar cf bundle.tar *.c *.h Makefile
gzip -9 bundle.tar
base64 bundle.tar.gz > bundle.txt
if exist bundle.tar.gz del bundle.tar.gz
if exist bundle.tar del bundle.tar
dir bundle.txt,*.c,*.h,Makefile
