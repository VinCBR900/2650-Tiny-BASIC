# 2650-Tiny-BASIC - Intended for 2732 EPROM

Signetics 2650 Tiny BASIC **WORK IN PROGRESS**, core functionality present

A minimal integer BASIC for Signetics 2650. No tokeniser — program lines are stored as raw ASCII and re-parsed on every execution using 2 character keyword matching. This costs RAM and speed but keeps the interpreter very small. Will fit into a 2732 EPROM (4096 bytes).

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

**Note Multi-statement lines** with `:` separator: **Not Supported** (yet) (e.g. `10 A=1 : B=2 : PRINT A+B`)

You can play with this online at [https://vincbr900.github.io/2650-Tiny-BASIC/](https://vincbr900.github.io/2650-Tiny-BASIC/)

Type `LIST` to see the embedded BASIC program and `RUN` to execute it - Pressing `CTRL-]` aborts running program. 

##  Notes

So far, this has been much more difficult than writing the [6502 Tiny BASIC](https://github.com/VinCBR900/65c02-Tiny-BASIC). Architectural Challanges are: 

- 8 level hardware stack. Recursion trades speed for code size (e.g. expression parser, printing digits), but here the standard stack too small
  - Option is to use SW stack but that consumes a register and adds 10-20 bytes of overhead each call, so best be worth it...  
- Although it has nice features like auto-increment and decrement, heavy indirection and return on condition code, these are not size optimized
  - e.g. Relative Jumps limited to +/- 63 bytes and still take 2 bytes due to condition codes.  So most jumps take 3 bytes  
- From a programmers perspective, the instruction set was clearly designed by an engineer and instructions are dense
  - e.g. `BCTR,LT addr` **B**ranch, got that - now do we want **C**ontrol for a jump, or **S**ubroutine for a call, and do we want **T**rue or **F**alse, and are we **Relative** short jump or **A**bsolute address, and then do we want **LT**, **GT**,**EQ**,**UN** condition code.  This is all very easy to overlook one character and make a typo which is hard to spot.
  
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

### Things to watch out for

- **ROM size.** uBASIC has less than a dozen bytes free, so pretty full. Always check after a change. Claude will help you find space savings if you're over budget.
- **Fall-through chains.** Several functions share a single `RETC,UN` by falling through into the next function. These are clearly marked in the source. Inserting code between them without understanding the fall-through will break things — tell Claude to watch out for them.
- **Arithmetic Gotchas** Signetics 2650 is intrinisically signed so the compares can be odd to understand
- **HW Stack limitations.** The Achilees heel of this device is the 8 level HW stack.  Stack overflows causes seemingly random errors, so you always have to keep an eye on it.

---

## Technical Notes

#### Why no tokeniser in uBASIC?

A tokeniser saves RAM (shorter stored programs) and speeds execution (no re-parsing). But the tokeniser itself costs ROM. The aim hjere was to get an MVP 2650 Tiny BASIC as I have never seen source for a 2650 version before, which is what this is.  Note the original assumption was 2kBytes would be feasible, like the others in this repo, but the architecture required 4kbyte. I would be **very** interested to hear if anyone can get it under 2kbytes.

---

## Credits & Similar Projects

- **jim11662418** nice modern 2650 SBC - [https://github.com/jim11662418/Signetics_2650_Single_Board_Computer](https://github.com/jim11662418/Signetics_2650_Single_Board_Computer)
- **2650 Restoration Project** for keeping this alive [https://ztpe.nl/2650/](https://ztpe.nl/2650/)
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
