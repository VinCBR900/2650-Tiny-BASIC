# 2650-Tiny-BASIC - Intended for 2732 EPROM

Signetics 2650 Tiny BASIC - core functionality present

You can play with this Interpreter online at [https://vincbr900.github.io/2650-Tiny-BASIC/](https://vincbr900.github.io/2650-Tiny-BASIC/)

This minimal integer Tiny BASIC interpreter explores what can be achieved on a processor designed before personal computers existed, embracing the constraints of the 2650, particularly its limited hardware stack and memory model, while demonstrating that capable interactive language can still fit within those restrictions.  There is a history article availble [here](/docs/history.md), summarized below.  
No tokeniser — program lines are stored as raw ASCII and re-parsed on every execution using 2 character keyword matching. This costs RAM and speed but keeps the interpreter small. Will fit into a 2732 EPROM (4096 bytes).

If you just want a proper BASIC for your Signetics 2650 system then the vintage [MicroWorld BASIC interpreter](https://binnie.id.au/MicroByte/BASIC%20Manual.pdf) is significantly more capable with floating point and string support. It is scattered around on the internet but I found a version at [https://github.com/jim11662418/Signetics_2650_Single_Board_Computer/tree/main](https://github.com/jim11662418/Signetics_2650_Single_Board_Computer/tree/main).

---
## Functionality

**Statements:** `PRINT [TAB(spaces)] [CHR$(expr)] [;]` `IF`/`THEN` `GOTO` [`LET`] `INPUT` `REM` `END` `RUN` `LIST` `NEW`   
(Parser accepts 2-letter prefixes: `PR` `IF` `GO` `LE` `IN` `RE` `EN` `RU` `LI` `NE` .)

**Expressions:** `+` `-` `*` `/` `% (Mod)` `=` `<` `>` `<=` `>=` `<>` unary `-` `(` `)` variables `A`–`Z`

**Numbers:** signed 16-bit integers, −32768 to 32767

**Errors** (printed as `?N [IN line]`):

| Code | Meaning |
|------|---------|
| ?0 SN | Syntax / bad expression |
| ?1 UL | Undefined line number |
| ?2 OV | Division or modulo by zero |
| ?3 OM | Out of memory |
| ?4 UK | Bad variable assignment |

**Note Multi-statement lines** with `:` separator(e.g. `10 A=1 : B=2 : PRINT A+B`) **Not Supported** Unikley to be be due to Stack limitations as below. 

Type `LIST` to see the embedded BASIC program and `RUN` to execute it - Pressing `CTRL-]` aborts running program. 

##  Notes

So far, this has been much more difficult than writing the [6502 Tiny BASIC](https://github.com/VinCBR900/65c02-Tiny-BASIC). Architectural Challanges are: 

- 8 level hardware Return Address Stack (RAS). Recursion trades speed for code size (e.g. expression parser, printing digits), but here the standard stack too small
  - One of the tricks for reduced size (sometimes called _Code Golf_) is for any code used twice or more, make it a subroutine.  That's Not possible here due to the small RAS, so we have lots of duplicate inline code.
  - I'm experimenting with SW stack, but that has about a dozen bytes overhead for each call, so best be worth it.  Interestingly the return address does not have to be the immediate next instruction...   
- Although it has nice features like auto-increment and decrement, heavy indirection and return on condition code, these are not size optimized
  - e.g. Relative Jumps limited to +/- 63 bytes and still take 2 bytes due to condition codes.  So most jumps take 3 bytes  
- From a programmers perspective, the instruction set was clearly designed by an engineer and instructions are dense
  - e.g. `BCTR,LT addr` **B**ranch, got that - now do we want **C**ontrol for a jump, or **S**ubroutine for a call, and do we want **T**rue or **F**alse, and are we **Relative** short jump or **A**bsolute address, and then do we want **LT**, **GT**,**EQ**,**UN** condition code, and finally do we want _addr_ or _*addr_.  This makes it very easy to overlook one character and make a typo that is hard to spot.
  
The biggest challange is that, unlike MOS 6502, Claude, Gemini and CODEX dont really know the 2650 CPU architecture, probably because training data is limited.  Available period PDF data sheets and App notes are poorely OCR'ed (Probably scanned in the 1990s) so can't just feed into the AI.

---

## Files

| File | Description |
|------|-------------|
| `uBASIC2650.asm` | uBASIC source (~2500 lines, heavily commented) |
| `Instruction_Oracle_2650.md`| Op-code Crib sheet for the AI|
| `tools/ASM2650.c`| Native 2650 Assembler, ported from [https://ztpe.nl/2650/development/as2650-a-2650-assembler/](https://ztpe.nl/2650/development/as2650-a-2650-assembler/)|
| `tools/Pipbug_Wrap.c`| Batch and interactive simulator.  Levearges `2650.c` CPU core from the [Winarcadia project](https://amigan.yatho.com/)|

The Tiny BASIC assembly sources include a pre-loaded **feature showcase program**. Type `RUN` to see it, `NEW` to clear it, `LIST` to read the source.  The showcase exercises `PRINT`, `CHR$`, arithmetic, comparisons, `IF`/`THEN`, `GOTO`-based loops (including nested), and finishes with a fixed point Mandelbrot renderer.

---

# The Signetics 2650 History

The Signetics 2650 was an 8-bit microprocessor designed in 1972 and released in 1975. Unlike many early CPUs that evolved from calculator chips, the 2650 was heavily influenced by minicomputer designs such as the IBM 1130, giving it an unusually sophisticated architecture for its era.

### Key Architectural Features

-   Seven general-purpose registers arranged as two switchable banks
-   Powerful register-to-register instructions
-   Built-in 8-level hardware return-address stack
-   Fully static design allowing single-step operation
-   15-bit address space organised into four 8 KB pages - this was consideredenough for anyone in 1972/3

These features allowed small systems to be built with little more than a CPU, ROM and clock - no RAM.

### Strengths and Weaknesses

The architecture was highly efficient for hand-written assembly language and embedded control applications. However, design decisions that made sense in 1972 became limitations by the late 1970s:

-   Fixed 8-level hardware stack
-   Paged memory model
-   No conventional stack pointer
-   Less suitable for high-level language compilers

Meanwhile newer processors such as the MOS 6502, Zilog Z80 and Motorola 6809 offered larger address spaces and RAM-based stacks, stealing the crown

### Prologue

Although it never achieved the commercial success of its rivals, the 2650 developed a loyal following among hobbyists and industrial users. It appeared in systems published by Elektor, Electronics Australia and numerous arcade and pinball machines.

---

## Technical Notes

TBD - Howie did-it.

---

## Credits & Similar Projects

- **jim11662418** nice modern 2650 SBC - [https://github.com/jim11662418/Signetics_2650_Single_Board_Computer](https://github.com/jim11662418/Signetics_2650_Single_Board_Computer)
- **2650 Restoration Project** for keeping this alive [https://ztpe.nl/2650/](https://ztpe.nl/2650/)
- **Frank's Electron Tube Homepage** and 2650 archive [https://frank.pocnet.net/instruments/FP_DIY/phunsy/phunsy.html](https://frank.pocnet.net/instruments/FP_DIY/phunsy/phunsy.html)
- **[Claude AI](https://claude.ai)** for making it possible for a non-expert to ship something that had been on the back burner since 1989.

---

## Licence

Copyright (c) 2026 Vincent Crabtree

**MIT License**

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
