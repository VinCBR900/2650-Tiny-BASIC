## Tools

This folder contasin an assembler and simulator for Signetics 2650.

The assembler is home built but cross referenced with 

https://ztpe.nl/2650/development/as2650-a-2650-assembler/

For the Simulator, we (Claude and I) spent a long time trying to build our own, archived in the 'Old' folder in the root.  However we kept runing into bugs when cross checking in Winarcadia, a **validated** emulator.

Eventually I realized, since we have been corss checking with a validated source, we could use the CPU core from Winarcadia `2650.c` and wrap just enough functionality for the basic emulation we need.

Winarcadia is not on github (yet) but is available below.  I had to copy the `2650.c` source file here so CODEX could access. 

https://amigan.yatho.com/
