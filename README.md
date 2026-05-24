# 2650-Tiny-BASIC

Signetics 2650 Tiny BASIC **WORK IN PROGRESS**, do not use.

##  Notes

First attempt at writing a Signetics 2650 Tiny BASIC Interpreter in Claude with help from CODEX.

So far, this has been much more difficult than writing the [6502 Tiny BASIC](https://github.com/VinCBR900/65c02-Tiny-BASIC). Archtectural Challanges are: 

- 8 level hardware stack. Recursion trades speed for code size (e.g. expression parser, printing digits), but here the standard stack too small
  - Even worse, this is User Inaccessible - no `push`/`pop`s allowed to clean up the stack
  - Option is to use SW stack but that consumes a register and adds 10-20 bytes of overhead each call, so best be worth it...  
- Although it has nice features like auto-increment and decrement, and branch on condition code, these are not size optimized
  - e.g. Relative Jumps limited to +/- 63 bytes and still take 2 bytes due to condition codes.  So most jumps take 3 bytes  
- 8kbyte memory pages - shouldn't matter here as expect max 4kbyte ROM, 4kbyte RAM
- From a programmers perspective, the instruction set was designed by an engineer and instructions are dense
  - e.g. `BCTR,LT addr` **B**ranch, got that - now do we want **C**ontrol for a jump, or **S**ubroutine for a call, and do we want **T**rue or **F**alse, and are we **Relative** short jump or **A**bsolute address, and then do we want **LT**, **GT**,**EQ**,**UN** condition code.  This is all very easy to overlook one character when debugging.
  
The biggest challange is that, unlike MOS 6502, Claude, Gemini and CODEX dont really know the 2650 CPU architecture, probably because training data is limited.  Available period PDF data sheets and App notes are poorely OCR'ed (Probably scanned in the 1990s) so can't just feed into the AI.

So they all **confidently** write code that is plain wrong.

The original aim was to get a working 2kbyte Tiny BASIC, but  we are currently hopeful for 4kbyte. 
