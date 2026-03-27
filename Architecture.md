# uBASIC2650 — Architecture Design Document
# Version: v1.3
# Updated: Session 5 — 2026-03-27
#
# History:
#   v1.0  Draft — initial arch plan before source analysis
#   v1.1  Post session 2 — shunting-yard parser, SW call stack for GOSUB
#   v1.2  Post session 4 — correct auto-inc understanding, INC_TMP strategy
#   v1.3  Post session 5 — ZBSR placement strategy, correct indexed addr,
#         SW call stack design, register bank decision, ROM/RAM layout,
#         reduced feature set, reference links added

================================================================================
## REFERENCE DOCUMENTS

  2650 User Manual (online):
    https://amigan.yatho.com/2650UM.html

  2650 Addressing Modes (wikibooks):
    https://en.wikibooks.org/wiki/Signetics_2650_%26_2636_programming/2650_processor#Indexed_addressing

  2650 Indexed Branching (wikibooks):
    https://en.wikibooks.org/wiki/Signetics_2650_%26_2636_programming/Indexed_branching

  uBASIC6502 v1.1 reference implementation:
    https://raw.githubusercontent.com/VinCBR900/65c02-Tiny-BASIC/refs/heads/main/uBASIC6502.asm

  uBASIC2650 baseline v0.4 (original port, buggy):
    https://raw.githubusercontent.com/VinCBR900/2650-Tiny-BASIC/refs/heads/main/uBASIC2650.asm

  Project repository:
    https://github.com/VinCBR900/2650-Tiny-BASIC

================================================================================
## TOOLCHAIN

  Assembler: tools/asm2650.c  (header v1.2, internal comment says core v1.3)
  Simulator: tools/sim2650.c  (header v1.2, SIM_VER string "1.4")

  Build:
    gcc -Wall -O2 -o tools/asm2650 tools/asm2650.c
    gcc -Wall -O2 -o tools/sim2650 tools/sim2650.c

  Assemble + run:
    ./tools/asm2650 src/uBASIC2650.asm ubasic.hex
    ./tools/sim2650 --allow-ram-image ubasic.hex

  CRITICAL ASSEMBLER RULE: Semicolons are COMMENTS, not statement separators.
  Every instruction must be on its own line.

  NOTE: asm2650.c and sim2650.c have been updated in the GitHub repository.
  Always pull latest before a session and update version numbers here.

================================================================================
## PRIORITY: CODE SIZE OVER EXECUTION SPEED

  The primary optimisation goal is minimum ROM bytes. Speed is irrelevant.
  Every design decision should ask "which uses fewer bytes?" not "which is faster?"

================================================================================
## FEATURE SET

  REMOVED (size reduction): GOSUB, RETURN, POKE, PEEK(), USR(), CHR$()

  Statements: PRINT  IF..THEN  GOTO  LIST  RUN  NEW  INPUT  REM  END  LET
  Operators:  + - * /   = < > <= >= <>   unary- unary+   ( )
  Variables:  A-Z (signed 16-bit each)
  Numbers:    -32768 .. 32767
  Lines:      1 .. 32767 (not range-checked — accepted as-is)

================================================================================
## MEMORY MAP — PAGE 0 LAYOUT (aligned with 2650 page architecture)

  The 2650 addresses 32768 bytes (15-bit) across four 8192-byte pages.
  Non-branch instructions address within the CURRENT PAGE only (13-bit direct).
  Cross-page access for data requires indirect addressing (LODA,R0 *addr).

  DESIGN DECISION: Place both ROM and RAM within page 0 ($0000-$1FFF)
  This allows all non-branch instructions to address everything directly
  without needing indirect addressing or page-crossing complexities.

  $0000-$13FF  ROM  5120 bytes  (code + constant tables)
  $1400-$1BFF  RAM  2048 bytes  (interpreter state + program store)
  $1C00-$1FFF  (unmapped or future expansion)

  NOTE: The first few bytes of ROM ($0000 onward) hold the ZBSR jump table
  (frequently called short subroutines). The first instruction at $0000 is
  an unconditional BCTA to INIT, placed after the last jump table entry.
  INIT is placed after the jump table and frequent subroutines in ROM.

================================================================================
## ROM LAYOUT — ZBSR JUMP TABLE AT BASE

  The 2650 ZBSR instruction calls the subroutine whose address is stored at
  the current RAS top — a 1-byte relative branch to a subroutine within the
  first ~63 bytes of ROM. Frequently called short subroutines (PUTCH, GETCH,
  WSKIP, CHECK_CR) are placed here to benefit from BSTR,UN (2-byte relative
  call) instead of BSTA,UN (3-byte absolute call) when called from within
  ±63 bytes of ROM start.

  ROM structure:
    $0000  BCTA,UN INIT          ; 3 bytes — jump past the table
    $0003  PUTCH:  WRTD,R1       ; 1 byte
                   ZBRR          ; 1 byte — return (ZBRR pops RAS)
    $0005  GETCH:  REDE,R1       ; 1 byte
                   ZBRR          ; 1 byte
    $0007  WSKIP:  [skip-spaces code, ≤63 bytes]
    ...
    $nnnn  INIT:   [cold start and REPL]
    ...
    $13FF  [end of ROM]

  Calls to PUTCH/GETCH from within ±63 bytes: BSTR,UN PUTCH (2 bytes).
  Calls from anywhere in ROM: BSTA,UN PUTCH (3 bytes).
  Conditional calls (e.g. "if error, print"): always BSTA,UN (BSTR is UN only).

================================================================================
## RAM MAP

  All 16-bit values stored big-endian (hi byte at lower address).

  $1400  IPH     Interpreter pointer hi   — walk pointer for IBUF / PROG
  $1401  IPL     Interpreter pointer lo
  $1402  PEH     Program end pointer hi   — one past last stored PROG byte
  $1403  PEL     Program end pointer lo
  $1404  RUNFLG  $01=running $00=immediate
  $1405  GOTOFLG $01=GOTO pending
  $1406  GOTOH   GOTO target line hi
  $1407  GOTOL   GOTO target line lo
  $1408  CURH    Current executing line hi (for error reporting)
  $1409  CURL    Current executing line lo
  $140A  LNUMH   Scratch line number hi
  $140B  LNUML   Scratch line number lo
  $140C  SC0     General scratch byte 0
  $140D  SC1     General scratch byte 1
  $140E  ERRFLG  $00=ok; non-zero=error
  $140F  NEGFLG  Sign flag (number parsing, mul/div sign tracking)
  $1410  EXPH    Expression result hi
  $1411  EXPL    Expression result lo
  $1412  TMPH    Temp/walk pointer hi   (used as indirect base for prog store walk)
  $1413  TMPL    Temp/walk pointer lo
  $1414  RELOP   Relational operator code 1-6

  ; ── Software call stack (see §CALL STRATEGY) ─────────────────────────────
  $1415  SW_STKPTR_H  SW stack pointer cell hi   (points into SW_STK)
  $1416  SW_STKPTR_L  SW stack pointer cell lo
  $1417  TEMP_RET_H   Temp return address hi     (used during SW_RETURN)
  $1418  TEMP_RET_L   Temp return address lo
  $1419  SW_STK       SW call stack: 64 bytes    ($1419-$1458)
                      Supports 32 nested SW calls (2 bytes each)

  $1459-$145F  (free — 7 bytes)
  $1460  IBUF    Input line buffer 64 bytes       ($1460-$149F)
  $14A0  VARS    A-Z variables 52 bytes           ($14A0-$14D3)
                 A=$14A0:$14A1 .. Z=$14D2:$14D3
  $14D4-$14FF  (free — 44 bytes)
  $1500  PROG    Program store base
  $1C00  PROGLIM One past end of program store    (1792 bytes available)

  Program store record format:
    [LINE_HI][LINE_LO][BODY_LEN][body bytes...]
    Sorted ascending by line number.
    Big-endian. Record size = 3 + BODY_LEN.

================================================================================
## REGISTER ALLOCATION

  R0   Primary accumulator. Destination for all ALU operations in Z/I/R/A
       modes (except: A-mode writes to Rn, Z-mode reads from Rn writes to R0).
       Freely clobbered. Used for all arithmetic and comparisons.

  R1   I/O register. WRTD,R1 = putchar. REDE,R1 = getchar.
       Also secondary scratch for character handling.
       Convention: loaded with char immediately before PUTCH/GETCH calls.

  R2   Index register for fixed-base indexed addressing (see §INDEXED ADDRESSING).
       Also general scratch when indexing not active.

  R3   SW call stack pointer. Indexes into SW_STK space.
       Also used as loop counter for BRNR,R3 / BIRR,R3 counted loops.
       NOTE: when R3 is used as a loop counter, SW calls cannot be made
       within that loop. Design code sections to not mix these uses.

  REGISTER BANK (RS bit in PSL): see §REGISTER BANK DECISION.

================================================================================
## REGISTER BANK DECISION

  The 2650 RS bit (PSL bit 4) switches between register banks 0 and 1.
  Bank 1 gives access to R1', R2', R3' (R0 is always shared).
  PPSL $10 = switch to bank 1. CPSL $10 = switch back to bank 0.

  DECISION: Register bank switching is NOT used in v1.x.

  Rationale:
    - PPSL/CPSL each cost 2 bytes. Eliminating one BSTA saves 3 bytes.
      Net saving from bank switch = 3 - (2+2) = -1 byte per elimination.
      It costs MORE bytes to use bank switching than BSTA for most cases.
    - The primary benefit (avoiding RAM saves) is marginal since our
      subroutines are designed to not require register preservation.
    - Adds mental complexity to track which bank is active.
    - Future consideration: if a specific hot path needs 7 registers
      simultaneously, bank switching could help. Document and revisit then.

================================================================================
## INDEXED ADDRESSING — CORRECT UNDERSTANDING (verified by simulation)

  The 2650 has THREE forms of absolute addressing (encoded in idxctl bits):

  1. LODA,R0 addr         — plain absolute (3 bytes). R0 = mem[addr].
  2. LODA,R0 *addr        — indirect absolute (3 bytes). R0 = mem[mem[addr]].
  3. LODA,Rn addr,Rn      — indexed (3 bytes). Rn = mem[addr + Rn].
  4. LODA,Rn addr,Rn+     — pre-increment indexed. Rn++; Rn = mem[addr + Rn].
  5. LODA,Rn addr,Rn-     — pre-decrement indexed. Rn--; Rn = mem[addr + Rn].

  CRITICAL: In indexed mode, Rn is BOTH the index register AND the destination.
  There is no way to use R2 as index with R0 as destination in one instruction.
  (The wikibook example `loda,r0 $1F00,r2+` appears to show this, but the
  confirmed assembler/simulator behaviour is: ,R2 sets idxctl bits but the
  DESTINATION is always Rn from the opcode field, not R0.)
  Write LODA,R2 addr,R2 to use R2 as both index and destination.

  CONFIRMED USEFUL PATTERNS:

  Pattern A — Single indexed load (random access):
    ; Load VARS[R2] hi byte into R2, then copy to R0:
    LODI,R2 offset         ; 2 bytes: R2 = (letter-'A')*2
    LODA,R2 VARS,R2        ; 3 bytes: R2 = VARS[offset]  (hi byte value)
    LODZ,R0                ; 1 byte:  R0 = R2  (copy result to accumulator)

  Pattern B — Sequential byte printing (R2 as combined value+index):
    LODI,R2 0              ; 2 bytes: start index
  LOOP:
    LODA,R2 STRBUF,R2      ; 3 bytes: R2 = STRBUF[R2]
    COMI,R2 NUL            ; 2 bytes: NUL check (COMI Rn uses Rn not R0)
    BCTA,EQ DONE           ; 3 bytes
    WRTD,R2                ; 1 byte:  print R2 (char is in R2)
    ADDI,R2 1              ; 2 bytes: next index (R2 = char+1, only works if
                           ;          chars are sequential — NOT generally useful)
    BCTR,UN LOOP           ; 2 bytes

  Pattern C — Indexed store (STRA,Rn BASE,Rn stores Rn, not R0):
    STRA,R2 OUTBUF,R2      ; stores R2 to OUTBUF+R2

  LIMITATION: Indexed addressing is primarily useful for RANDOM ACCESS
  to fixed-base tables (VARS, DIVTAB) where one load per invocation suffices.
  Sequential byte walking over variable-base pointers (program store) still
  requires indirect addressing via LODA,R0 *TMPH with INC_TMP subroutine.

  INDIRECT ADDRESSING (LODA,R0 *addr):
    Most flexible for walking unknown-base pointers.
    Used for program store traversal where base changes dynamically.
    Cost: LODA,R0 *TMPH (3 bytes) + BSTA,UN INC_TMP (3 bytes) = 6 bytes/byte.

================================================================================
## CC SEMANTICS (from sim2650.c — verified by instruction_microtests.py)

  set_cc(result):
    result=0  → CC=EQ ($80 in PSL)   Branch: BCTA,EQ / BCFA,EQ (inverse)
    result>0  → CC=GT ($40 in PSL)   Branch: BCTA,GT / BCFA,GT
    result<0  → CC=LT ($00 in PSL)   Branch: BCTA,LT / BCFA,LT

  set_cc_add(result) — after ADDI/ADDA/ADDZ:
    C=0                  → CC=GT  (no carry — the common case)
    C=1 and result=0     → CC=EQ  (carry, wrapped exactly to zero)
    C=1 and result≠0     → CC=LT  (carry with non-zero result)

  set_cc_sub(result) — after SUBI/SUBA/SUBZ:
    C=1 (no borrow) and result≠0 → CC=GT
    C=1 (no borrow) and result=0 → CC=EQ
    C=0 (borrow occurred)         → CC=LT

  *** CRITICAL IDIOMS — GET THESE WRONG AND ARITHMETIC IS SILENTLY BROKEN ***

  16-bit ADD with carry propagation to hi byte:
    LODA,R0 PTR_L
    ADDI,R0 1
    STRA,R0 PTR_L
    BCTA,GT skip_hi_inc    ← GT = C=0 = no carry → skip
    LODA,R0 PTR_H
    ADDI,R0 1
    STRA,R0 PTR_H
  skip_hi_inc:

  16-bit SUB with borrow propagation to hi byte:
    LODA,R0 ACC_L
    SUBA,R0 VAL_L
    STRA,R0 ACC_L
    BCFA,LT skip_hi_dec    ← BCFA,LT = branch if NOT LT = C=1 = no borrow
    LODA,R0 ACC_H          ← only reach here if C=0 (borrow)
    SUBI,R0 1
    STRA,R0 ACC_H
  skip_hi_dec:

  *** NEVER use BCTA,GT for borrow skip — it misses CC=EQ (zero result,    ***
  *** no borrow) and subtracts an extra 1 from the hi byte. This was        ***
  *** BUG-02 in the original v0.4 port and cost significant debugging time. ***

================================================================================
## SUBROUTINE CALLING STRATEGY — THREE TIERS

  Priority: smallest code size. Use the cheapest call mechanism that is safe.

  === TIER 1: Hardware RAS — BSTA,UN / BSTR,UN / BSTA,cc ===
  Use for: non-recursive subroutines where call depth stays ≤ 6.
  Cost: BSTA,UN = 3 bytes absolute. BSTR,UN = 2 bytes relative (±63).
  Return: RETC,UN = 1 byte (always unconditional for speed and size).
  Conditional call: BSTA,cc addr (3 bytes) — only for EQ/GT/LT/UN.
                   BSTR,cc offset (2 bytes) — conditional relative, ±63 bytes.

  Reserve 2 RAS slots for interrupts (see §INTERRUPT CONSIDERATION).
  Maximum safe call depth = 6 (8-slot RAS minus 2 interrupt reserve).

  HW RAS subroutines that CALL others must count total depth:
    REPL[1] → STMT_EXEC[2] → DO_xxx[3] → PARSE_EXPR[4] → PARSE_FACTOR[5]
                                                         → INC_TMP[6]  ← safe limit

  === TIER 2: BSTR,UN for nearby utility routines ===
  Use for: PUTCH, GETCH, WSKIP, UPCASE, CHECK_CR when called from within ±63
  bytes of the target. Saves 1 byte vs BSTA,UN.
  Targets at ROM base ($0000-$003F) benefit from this from many call sites.

  === TIER 3: Software Call Stack — BCTA/BCTR only, no BSTA/RETC ===
  Use for: recursive routines (PARSE_EXPR, PARSE_TERM, PARSE_FACTOR, PRT16).
  The HW RAS is NOT used — all branching uses BCTA,UN or BCTR,UN.
  See §SOFTWARE CALL STACK for full design.

  TAIL CALLS: Any subroutine may end with BCTA,UN next_sub instead of
  RETC,UN + call. Avoids one RAS slot and saves 1-2 bytes.
  Example: PARSE_FACTOR tail-calls GET_VARPTR instead of BSTA + RETC.

================================================================================
## SOFTWARE CALL STACK — DESIGN AND USAGE

  PURPOSE: Enable recursion (PARSE_EXPR→PARSE_FACTOR→PARSE_EXPR for parens,
  PRT16 recursive digit extraction) without overflowing the 8-slot HW RAS.

  The SW call stack uses R3 as stack pointer and RAM for storage.
  SW calls use BCTA,UN (branch) NOT BSTA (which would push HW RAS).
  RETC,UN is NEVER used in SW-called routines — return is via SW_RETURN.

  === MEMORY ===
    SW_STKPTR  EQU $1415  ; 2-byte RAM cell holding pointer into SW_STK
    TEMP_RET   EQU $1417  ; 2-byte workspace for return address during SW_RETURN
    SW_STK     EQU $1419  ; 64 bytes = 32 nested calls

  === INITIALISATION (in RESET) ===
    LODI,R3 0             ; R3 = SW stack index = 0 (empty)
    LODI,R0 >SW_STK
    STRA,R0 SW_STKPTR     ; SW_STKPTR_H = >SW_STK
    LODI,R0 <SW_STK
    STRA,R0 SW_STKPTR+1   ; SW_STKPTR_L = <SW_STK

  === SW_JSR — CALLER SNIPPET (can be macro or inline) ===
  Equivalent to JSR TARGET with return to RETADDR.
  The caller chooses RETADDR — it does not have to be the next instruction,
  enabling computed jumps and tail-call optimisation.

    ; Push RETADDR hi byte
    LODI,R0 >RETADDR
    STRA,R0 *SW_STKPTR,R3+     ; store to SW_STK[R3], then R3++

    ; Push RETADDR lo byte
    LODI,R0 <RETADDR
    STRA,R0 *SW_STKPTR,R3+     ; store to SW_STK[R3], then R3++

    BCTA,UN TARGET             ; branch to target (NO BSTA — HW stack untouched)

  RETADDR:                     ; execution resumes here after SW_RETURN

  NOTE: For assembly tail calls, simply jump to the next routine without
  pushing a return address. The last routine in the chain pops and returns
  to the PREVIOUSLY pushed address. This saves 8 bytes per tail call site.

  === SW_RETURN — SHARED RETURN HANDLER ===
  The LAST instruction of any SW-called subroutine is: BCTA,UN SW_RETURN
  (or BCTR,UN SW_RETURN if within 63 bytes — saves 1 byte).

  SW_RETURN:
    LODA,R0 *SW_STKPTR,-R3     ; pre-decrement R3, load SW_STK[R3] → R0 (lo byte)
    STRA,R0 TEMP_RET+1         ; store lo byte to TEMP_RET+1

    LODA,R0 *SW_STKPTR,-R3     ; pre-decrement R3, load SW_STK[R3] → R0 (hi byte)
    STRA,R0 TEMP_RET           ; store hi byte to TEMP_RET

    BCTA,UN *TEMP_RET          ; indirect branch to popped address (NO RETC)

  NOTE: RETC,UN is NEVER used in SW_RETURN. RETC would pop the HW RAS which
  may contain stale addresses. BCTA,UN *TEMP_RET is the correct return.

  === WHAT THE -R3 SYNTAX MEANS ===
  LODA,R0 *SW_STKPTR,-R3: idxctl=2 (pre-decrement Rn then index).
  Since Rn=3: R3--; eff = mem[SW_STKPTR] + R3; R0 = mem[eff].
  CORRECTION: this uses R3 as index, result goes to R0. Verify assembler
  syntax for pre-decrement with separate index/dest — may need SUBI,R3 1 first.

  === HW vs SW STACK SUMMARY ===
  | Situation                          | Mechanism      | Cost per call |
  |------------------------------------|----------------|---------------|
  | Short leaf routine, near           | BSTR,UN (rel)  | 2 bytes       |
  | Short leaf routine, anywhere       | BSTA,UN (abs)  | 3 bytes       |
  | Conditional call                   | BSTA,cc (abs)  | 3 bytes       |
  | Recursive / deep call              | SW_JSR inline  | 14 bytes      |
  | Return from HW routine             | RETC,UN        | 1 byte        |
  | Return from SW routine             | BCTA SW_RETURN | 3 bytes       |

================================================================================
## INTERRUPT CONSIDERATION

  The 2650 interrupt mechanism forces a BSTA to a device-determined address,
  pushing the current PC onto the HW RAS. This uses 1 RAS slot.
  A BREAK-key interrupt handler is planned for a future version.

  DESIGN RULE: Reserve 2 HW RAS slots at all times to accommodate:
    - 1 slot for the interrupt BSTA
    - 1 slot for the interrupt handler to call one subroutine
  Maximum safe HW call depth = 6 (not 8).

================================================================================
## RECURSIVE EXPRESSION PARSER — CALL DEPTH ANALYSIS

  Recursive descent: PARSE_EXPR → PARSE_TERM → PARSE_FACTOR → PARSE_EXPR (parens)

  Using SW call stack for all recursive calls:
    REPL[HW1] → STMT_EXEC[HW2] → DO_xxx[HW3] → PARSE_EXPR[SW]
    PARSE_EXPR[SW] → PARSE_TERM[HW4] → PARSE_FACTOR[HW5]
    PARSE_FACTOR for parens: BCTA,UN PARSE_EXPR (no HW push — SW_JSR inline)

  HW depth stays at 5 max (leaving 3 slots spare for interrupts + 1 extra).
  SW depth is unlimited (bounded only by SW_STK size: 32 levels).

  The 6502 version achieves compactness through recursive descent because
  JSR/RTS is 3+1 = 4 bytes vs SW_JSR inline which costs 14 bytes.
  HOWEVER: recursive descent allows sharing PARSE_EXPR code for parens,
  unary operators, and nested expressions — saving code vs a flat shunting-yard.

  COMPARISON for uBASIC2650:
    Recursive descent + SW stack:
      PARSE_EXPR: ~50 bytes
      PARSE_TERM: ~50 bytes  
      PARSE_FACTOR: ~80 bytes
      SW_JSR inline at 2 sites: ~28 bytes
      SW_RETURN: ~15 bytes
      Total: ~223 bytes

    Shunting-yard (iterative):
      PARSE_EXPR: ~150 bytes
      APPLY_OP: ~120 bytes
      GET_PREC: ~20 bytes
      Total: ~290 bytes

  VERDICT: Recursive descent saves ~67 bytes. USE RECURSIVE DESCENT.

================================================================================
## RECURSIVE PRT16 — DESIGN

  The 6502 PRT16 uses hardware stack to recurse, printing digits in order
  by unwinding the recursion. On the 2650, use SW stack:

  ; PRT16 — print signed 16-bit T0 as decimal
  ; Input: EXPH:EXPL = value
  ; Uses: SW stack for recursion (handles leading zeros by not printing them)
  ;
  PRT16:
    ; Check sign
    LODA,R0 EXPH
    COMI,R0 $80
    BCTA,LT PRT16_POS
    LODI,R1 '-'
    WRTD,R1              ; print minus
    [negate EXPH:EXPL]
  PRT16_POS:
    ; Divide EXPH:EXPL by 10 → quotient in EXPH:EXPL, remainder in SC0
    [divide]
    ; If quotient != 0, recursive call to print higher digits
    LODA,R0 EXPH
    COMI,R0 0
    BCTA,GT PRT16_REC
    LODA,R0 EXPL
    COMI,R0 0
    BCTA,EQ PRT16_DIGIT   ; quotient=0, just print this digit
  PRT16_REC:
    [SW_JSR PRT16 with RETADDR=PRT16_DIGIT]
    BCTA,UN PRT16
  PRT16_DIGIT:
    LODA,R1 SC0           ; remainder = digit
    ADDI,R1 '0'
    WRTD,R1
    BCTA,UN SW_RETURN

================================================================================
## PROGRAM STORE — FIND_LINE + FIND_INS MERGED

  FIND_LINE and FIND_INS are nearly identical (both walk sorted records).
  Merged into one routine with SC0 flag:
    SC0=$00 → find exact match (FIND_LINE behaviour)
    SC0=$01 → find insertion point (FIND_INS behaviour)
  Returns TMPH:TMPL at result. ERRFLG=$00 if exact match found (FIND_LINE mode).
  Saves ~110 bytes vs two separate routines.

================================================================================
## KEYWORD MATCHING — INDEXED BRANCH TABLE

  Current: sequential COMI/BCTA dispatch (~60 bytes for 10 keywords).
  Improved: use BXA (indexed branch) with R2 as token offset.
    Build a branch table: DW HANDLER1, DW HANDLER2, ...
    R2 = (token-1) * 2
    BCTA,UN JMPTAB,R2  ; jumps to mem[JMPTAB + R2]
  OR: use token as index into BIRA/BDRA counted branch series.
  BXA $BF: indexed absolute branch from a table. Saves ~30 bytes.

================================================================================
## SUBROUTINE HEADER CONVENTION (from uBASIC6502 style)

  Every subroutine must have a header comment:
  ; ─── ROUTINE_NAME ─────────────────────────
  ; Purpose: one-line description
  ; In:  register/memory inputs
  ; Out: register/memory outputs
  ; Clobbers: list of modified registers/cells
  ; Depth: HW stack depth when called (for budget tracking)

================================================================================
## SHARED INCREMENT SUBROUTINES (INC_TMP / INC_IP / INC_EXP)

  Each 16-bit pointer increment inline costs 19 bytes (7 instructions).
  Shared subroutine body: 17 bytes. Breakeven at 2 call sites.
  Each call: BSTA,UN INC_xxx = 3 bytes. Saving: 16 bytes per additional site.

  Current implementation (4108 bytes) has:
    INC_TMP: 23 call sites → (23×3 + 17) vs (23×19) = 86 vs 437 = saves 351 bytes
    INC_IP:  23 call sites → saves 351 bytes
    INC_EXP: 10 call sites → saves 113 bytes
  Total saving from INC_ subroutines: ~815 bytes (confirmed in current build).

  Also provide DEC_TMP for backwards memory moves in STORE_LINE / DELETE_LINE.

================================================================================
## SIZE TRACKING (session fingerprint)

  Assembled size after each major session:
    Baseline v0.4 (original buggy port):       ~4000+ bytes (estimated)
    v1.0 fresh arch, full features:             5783 bytes (OVER ROM)
    v1.1 after feature removal + INC_ routines: 4108 bytes (fits ROM)
    Target for v1.2 (recursive parser, merged): < 3500 bytes
    Aspirational:                               < 3072 bytes
    ROM limit:                                    5120 bytes ($0000-$13FF)

  Session continuity check: assemble at session start and confirm byte count
  matches the last recorded value in the trace log.

================================================================================
## COMPARISON: uBASIC6502 v1.1 vs uBASIC2650

  uBASIC6502: 2017 bytes for full-feature BASIC (POKE/PEEK/USR/CHR$/GOSUB).
  uBASIC2650 target: ~3000-3500 bytes for reduced feature set.

  WHY THE 2650 NEEDS MORE BYTES:
  1. Zero-page instructions (6502: 2 bytes vs 2650: 3 bytes for RAM access)
     ~150 ZP accesses × 1 byte extra = ~150 bytes overhead
  2. No (zp),Y equivalent: 2650 has no instruction using R as pointer AND
     separate index register; all indirect goes through RAM pointer cells
  3. Hardware data stack (6502: PHA=1 byte vs 2650: STRA=3 bytes per save)
  4. JSR/RTS = 3+1=4 bytes vs SW_JSR = 14 bytes + SW_RETURN = 3 bytes
  Irreducible structural overhead: ~600-1000 bytes vs 6502.

================================================================================
END OF ARCHITECTURE DOCUMENT v1.3
