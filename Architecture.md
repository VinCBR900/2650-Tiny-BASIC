# uBASIC2650 — Architecture Design Document
## Version: v1.50
## Updated: Session 7 — 2026-03-28

## Change history:
 -  v1.0  Initial
 -  v1.1  Shunting-yard, SW call stack
 -  v1.2  INC_TMP strategy, register bank decision
 -  v1.3  ZBSR placement, SW call stack design, ROM/RAM layout
 -  v1.4  WinArcadia platform adopted; PIPBUG 2 I/O addresses (WRONG)
 -  v1.5  Corrected to PIPBUG 1 (WinArcadia uses original PIPBUG, not PIPBUG 2)
     - COUT=$02B4 CHIN=$0286 CRLF=$008A (confirmed working)
     - Clarified EPROM size targets: 2732=4K primary, 2716=2K stretch
     - ZBSR: can only call addresses +/-64 bytes (page-0, or top of 8k memory page)
     - CHIN is blocking 

================================================================================
## SECTION 1 — REFERENCE DOCUMENTS

  2650 User Manual (online):
    https://amigan.yatho.com/2650UM.html

  2650 Addressing Modes:
    https://en.wikibooks.org/wiki/Signetics_2650_%26_2636_programming/2650_processor#Indexed_addressing

  2650 Indexed Branching:
    https://en.wikibooks.org/wiki/Signetics_2650_%26_2636_programming/Indexed_branching

  uBASIC6502 v1.1 reference implementation:
    https://raw.githubusercontent.com/VinCBR900/65c02-Tiny-BASIC/refs/heads/main/uBASIC6502.asm

  Project repository:
    https://github.com/VinCBR900/2650-Tiny-BASIC

  WinArcadia 2650 emulator (validated reference):
    http://amigan.1emu.net/releases/

  Python asm2650.py 2650Assembler (validated reference):
    https://ztpe.nl/2650/development/as2650-a-2650-assembler/

================================================================================
## SECTION 2 — TOOLCHAIN

  WinArcadia usage:
    Ctrl+4                     select PIPBUG machine
    Ctrl+/                     toggle debugger
    Tab                        activate debugger input
    ASM <filename>             assemble (file in Projects/ subdirectory)
    G 0C00                     run from $0C00
    T                          CPU trace
    S                          single step
    DIS <addr1> <addr2>        disassemble
    = <label>                  show label address
    E <addr> <val>             poke memory

  Cross-assembler: asm2650.c
  
    Build: gcc -Wall -O2 -o tools/asm2650 tools/asm2650.c
  
    Produces Intel HEX output.

  Assembler oracle: asm2650.py
  
    Usage: python3 tools/asm2650.py src/foo.asm /tmp/foo_py.hex
  
    Diff against asm2650.c output to find encoding bugs:
    
      diff <(./tools/asm2650 src/foo.asm /dev/stdout) \
           <(python3 tools/asm2650.py src/foo.asm /dev/stdout)

  Simulator: sim2650.c
    Build: gcc -Wall -O2 -o tools/sim2650 tools/sim2650.c
    
    Run:   ./tools/sim2650 --allow-ram-image image.hex
    
    Compare output against WinArcadia to find execution bugs.
  
================================================================================
## SECTION 3 — DEVELOPMENT PLATFORM: PIPBUG 1 (WinArcadia)

  WinArcadia emulates the ORIGINAL PIPBUG (not PIPBUG 2, not BINBUG).
  These are the CONFIRMED correct addresses for WinArcadia's built-in PIPBUG:

  - COUT = $02B4   BSTA,UN $02B4   R0 → terminal     CONFIRMED WORKING    │
  - CHIN = $0286   BSTA,UN $0286   terminal → R0     CONFIRMED BLOCKING    
  -  CRLF = $008A   BSTA,UN $008A   prints CR+LF                            
  
  PIPBUG memory map (WinArcadia PIPBUG machine):
   - $0000–$03FF   PIPBUG ROM (1K, read-only, built-in)
   - $0440–$0fFF   User program RAM — OUR CODE STARTS HERE
    
  User program entry:
   - ORG $0440
   - Run via PIPBUG command:  G 440
  
  Phase 2 — Standalone (after BASIC is working):
   - We replace PIPBUG I/O with our own bit-bang serial routines.
   - ROM starts at $0000. ZBSR jump table placed at base of ROM.
   - COUT/CHIN replaced by bitbang PUTCH/GETCH using FLAG/SENSE pins.

================================================================================
## SECTION 4 — EPROM SIZE TARGETS
  
  -  PRIMARY TARGET:  2732 EPROM = 4096 bytes  (period-correct 4K part)     │
  -  STRETCH TARGET:  2716 EPROM = 2048 bytes  (period-correct 2K part)     │
 
 These are EPROM sizes, not RAM sizes. The code fits in one chip.
 
 Both parts were widely available in 1977–1980, which is the target era.

  Current status:
    - uBASIC2650.asm v1.1 = 4108 bytes  ← 12 bytes OVER the 4K target!
    - Must save ≥ 13 bytes to fit a 2732 at all.
    - Recursive parser rewrite saves ~573 bytes → comfortably fits 2732.
    - Getting to 2716 (2K) requires ~2060 additional bytes of savings.
    - 2716 target is aspirational; focus first on 2732 then evaluate 2716.

  Size history:
    - Baseline v0.4 (buggy port):        ~4100 bytes (estimated)
    - v1.0 fresh arch, all features:      5783 bytes  (over ROM)
    - v1.1 feature strip + INC_*:         4108 bytes  (12 bytes over 2732)
    - Target v1.2 (recursive parser):   <3500 bytes   (fits 2732, good margin)
    - Stretch (2716):                    <2048 bytes   (very aggressive)

  Simulation target ROM was set to 5120 bytes to give initial headroom
  
  during development — larger than EPROM target to avoid premature constraint.

================================================================================
## SECTION 5 — INSTRUCTION SET REFERENCE

  ═══ 7.1 Register Set ═══

    R0  Primary accumulator. Destination for all load instructions.
        Source/destination for all ALU immediate/register instructions.
    R1  Secondary register. WRTD,R1=serial out, REDE,R1=serial in (Phase 2).
        Also used as index in ADDZ,R1 / SUBZ,R1 etc.
    R2  Index register for indexed addressing. Unchanged by plain load.
        Pre-incremented by LODA,R0 BASE,R2+ before the memory access.
    R3  DUAL USE: (a) loop counter for BRNR/BIRR, (b) SW call stack index.
        Never mix these uses in the same code path.

  ═══ 7.2 Addressing Modes (Verified) ═══

    Z     LODZ Rn           1 byte   R0 = Rn
    I     LODI,Rn n         2 bytes  Rn = n (immediate)
    R     LODR,Rn off       2 bytes  Rn = mem[PC+off]  (±63 relative)
    A     LODA,Rn addr      3 bytes  Rn = mem[addr]
    A*    LODA,Rn *addr     3 bytes  Rn = mem[ mem[addr]:mem[addr+1] ]
    A,R2  LODA,R0 addr,R2   3 bytes  R0 = mem[addr+R2], R2 unchanged
    A,R2+ LODA,R0 addr,R2+  3 bytes  R2++, then R0 = mem[addr+R2]
    A,R2- LODA,R0 addr,R2-  3 bytes  R2--, then R0 = mem[addr+R2]

  KEY RULE — INDEXED LODA (confirmed by WinArcadia assembler warning):
    The first register field is ALWAYS R0 (destination).
    The index register is the SECOND field (R2 typically).
    LODA,R2 BASE,R2 is INVALID — WinArcadia rejects it.
    Think of it as: 6502 LDA BASE,Y  where R2=Y, R0=A.
    R2 is UNCHANGED by a plain indexed load.
    R2 is incremented/decremented by R2+/R2- BEFORE the memory access.

  ═══ 7.3 ZBSR / ZBRR ═══

    ZBSR *offset    2 bytes ($BB + offset)
      Pushes current IAR to RAS, jumps to absolute address = offset byte.
      Range: $00–$7F only (7-bit positive offset = first 128 bytes of memory).
      Used in Phase 2 standalone to call short routines at ROM base.
      In Phase 1 (PIPBUG): $0000–$007F is PIPBUG ROM — cannot place stubs there.

  ═══ 7.4 CC Semantics ═══

    After ADD (ADDI/ADDA/ADDZ):
      No carry, result any    → CC=GT
      Carry, result = 0       → CC=EQ
      Carry, result ≠ 0       → CC=LT

    After SUB (SUBI/SUBA/SUBZ):
      No borrow, result ≠ 0   → CC=GT
      No borrow, result = 0   → CC=EQ
      Borrow                  → CC=LT

    16-BIT CARRY PROPAGATION IDIOM:
      LODA,R0 PTR_LO
      ADDI,R0 1
      STRA,R0 PTR_LO
      BCTA,GT skip_hi_inc     ; GT = no carry → skip
      LODA,R0 PTR_HI
      ADDI,R0 1
      STRA,R0 PTR_HI
    skip_hi_inc:

    16-BIT BORROW PROPAGATION IDIOM:
      LODA,R0 PTR_LO
      SUBI,R0 1
      STRA,R0 PTR_LO
      BCFA,LT skip_hi_dec     ; NOT LT = no borrow → skip (covers EQ too)
      LODA,R0 PTR_HI
      SUBI,R0 1
      STRA,R0 PTR_HI
    skip_hi_dec:

    *** NEVER use BCTA,GT for borrow skip — misses the CC=EQ case ***
    *** This was BUG-02 in v0.4 and caused silent arithmetic errors ***
Note: 
  - LODZ,R0 is not supported and the assembler should replace with $60 and warn.
  - ANDZ,R0 is not supported and triggers a HALT encoded as $00.  This too should warn. 

================================================================================
## SECTION 8 — SOFTWARE CALL STACK
  HW stack limited to 8 pushes.  Can still use for shallow branches.
  HW RAS budget: 8 slots. Reserve 2 for interrupts. Max safe depth = 6.
 
  SW stack enables recursion but has Byte size overhead - hopefully
  overall size gain with recursive subroutines like PRINT_S16.
  Uses R3 as index, SW_STKPTR as base, TEMP_RET as workspace.
  All SW-called routines end with BCTA,UN SW_RETURN (not RETC,UN).
  All SW-callers use inline push + BCTA,UN to target (not BSTA).
  Note SW push can be any addres, not necessarily imemdiately after.

 
  ─── SW_JSR inline (14 bytes per call site) ───
    LODI,R0 >RETADDR
    STRA,R0 *SW_STKPTR,R3+
    LODI,R0 <RETADDR
    STRA,R0 *SW_STKPTR,R3+
    BCTA,UN  TARGET
  RETADDR:

  ─── SW_RETURN shared handler ───
  SW_RETURN:
    LODA,R0 *SW_STKPTR,-R3
    STRA,R0 TEMP_RET_L
    LODA,R0 *SW_STKPTR,-R3
    STRA,R0 TEMP_RET_H
    BCTA,UN *TEMP_RET_H

================================================================================
## SECTION 9 — PARSER

  Recursive descent (NOT shunting-yard) with SW call stack for recursion.
  Saves ~67 bytes vs shunting-yard. Avoids operator precedence table.

  Grammar:
    expr   ::= term   { ('+' | '-') term }
    term   ::= unary  { ('*' | '/') unary }
    unary  ::= ['-']  factor
    factor ::= var | number | '(' expr ')'

================================================================================
## SECTION 10 — FEATURE SET

  Included: PRINT  IF..THEN  GOTO  LIST  RUN  NEW  INPUT  REM  END  LET
  TBD:  GOSUB  RETURN  POKE  PEEK()  USR()  CHR$()

  Variables:  A–Z, signed 16-bit (-32768..32767)
  Line numbers: 1–32767 (not checked)

================================================================================
## SECTION 12 — SUBROUTINE HEADER CONVENTION

  Every subroutine must have a header:

  ; ─── NAME ─────────────────────────────────────────────────────────────────
  ; Purpose: one-line description
  ; In:      register/RAM inputs
  ; Out:     register/RAM outputs, CC state if relevant
  ; Clobbers: all modified registers and RAM cells
  ; Depth:   HW RAS depth when called
  ; Size:    byte count (fill after assembly)

