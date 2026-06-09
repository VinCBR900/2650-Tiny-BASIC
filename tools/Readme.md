## Tools

This folder contains an assembler and simulator for Signetics 2650 CPU.

The assembler is home built but cross referenced with

https://ztpe.nl/2650/development/as2650-a-2650-assembler/

### Building

```sh
make -C tools
```

This builds:

* `asm2650` / `asm2650.exe` — the cross-assembler.
* `pipbug_wrap` / `pipbug_wrap.exe` — the PIPBUG 1 simulator wrapper around the WinArcadia 2650 CPU core.

### Simulator usage

`pipbug_wrap` accepts either an assembled Intel HEX file or an ASM source file:

```sh
./pipbug_wrap [options] program.hex
./pipbug_wrap [options] program.asm
```

When a `.hex` file is provided, the simulator keeps the legacy behavior and loads it directly.

When a native Linux/Win32 `.asm` file is provided, the simulator invokes the sibling `asm2650` executable, writes the generated `.hex` beside the source, then reads the generated `.LST` listing to auto-detect `CHIN`, `COUT`, and the program entry point. Explicit command-line options such as `--chin`, `--cout`, and `--entry` still take precedence over auto-detected values.

ASM auto-assembly is intentionally unavailable in Emscripten/browser builds because those builds cannot spawn the host `asm2650` executable; assemble to HEX before loading in the browser.

Common options:

```text
-t              Trace every instruction to stderr
-s              Step mode: pause each instruction
-i              Interactive terminal mode; raw I/O, Ctrl-] exits
-b 0xADDR       Breakpoint at address (hex)
-m 0xADDR LEN   Dump LEN bytes from address at halt
-n LIMIT        Instruction limit (default 5000000, 0=unlimited)
--chin 0xADDR   CHIN intercept address (default 0x0286)
--cout 0xADDR   COUT intercept address (default 0x02B4)
--crlf 0xADDR   CRLF intercept address (default 0x008A)
--entry 0xADDR  Program entry address (default 0x0440)
```

For the Simulator, we (Claude and I) spent a long time trying to build our own, archived in the 'Old' folder in the root.  However we kept running into bugs when cross checking in Winarcadia, a **validated** emulator.

Eventually I realized, since we have been cross checking with a validated source, we could use the CPU core from Winarcadia `2650.c` and wrap just enough functionality for the basic CPU & TTY I/O emulation we need. Later we expanded to make it interactive, then made in EMSCRIPTEN compatble for use online in a browser, hopefully without breaking legacy Linux/Win32 Batch/Interactive functionality.

Winarcadia is not on github (yet) but is available below.  I had to copy the `2650.c` source file here so CODEX could access, with very minor changes to paths to build.

https://amigan.yatho.com/
