# 2650-Tiny-BASIC

Signetics 2650 Tiny BASIC **WORK IN PRGRESS**, do not use.

##  Notes

First attempt at writing a Signetics 2650 Tiny BASIC Interpreter in Claude with help from CODEX.

So far, this has been much more difficult than writing the [6502 Tiny BASIC](https://github.com/VinCBR900/65c02-Tiny-BASIC). Archtectural Challanges are: 

- 8 level hardware stack.  Recursion trades speed for code size (e.g. expression parser, printing digits), but here the standard stack too small
  - Option is to use SW stack but that add 10-20 bytes of overhead each call, so best be worth it...  
- Although it has nice features like auto-increment and decrement, and branch on condition code, these are not size optimized
  - e.g. Relative Jumps limited to +/- 63 bytes and still take 2 bytes due to condition codes  
- 8kbyte memory pages - shouldn't matter here as expect max 4kbyte ROM, 4kbyte RAM
  
The biggest challange is that, unlike MOS 6502, Claude, Gemini and CODEX dont really know the 2650 CPU architecture, probably because training data is limited.

So they all **confidently** write code that is plain wrong.

The original aim was to get a working 2kbyte Tiny BASIC, but at this rate 4kbyte is more likely. 
