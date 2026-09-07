; pBASIC2650.asm       Minimal PoC Tiny BASIC interpreter for Signetics 2650
; Version: v1.0
; By Vincent Crabtree, 2026.  MIT License
;
; Target:  Standalone (no PIPBUG ROM). Code ORG 0. I/O routines embedded.
;          Single 8192-byte address space (2650 bits 15:13 always 0).
;
; Goal:    Minimal Tiny BASIC for Signetics 2650 target <2kbyte
;
; Assembler: asm2650.c v1.16  Simulator: pipbug_wrap.c
; Build:
;   gcc -Wall -O2 -o asm2650 asm2650.c
;   gcc -Wall -O2 -DGAMER -o pipbug_wrap pipbug_wrap.c
;   ./asm2650 pBASIC2650.asm pBASIC2650.hex
;   grep -n "^CHIN \|^COUT " pBASIC2650.LST   -- find their real addresses
;   ./pipbug_wrap --entry 0 --chin 0x<real> --cout 0x<real> --crlf 0x7fff pBASIC2650.hex
;
;        CC SEMANTICS (2650 ALU)
;   ADD/SUB: result<0 -> LT   result>0 -> GT   result=0 -> EQ
;   Carry bit (PSL bit 0): C=1 = carry / no-borrow (independent of CC).
;   Carry test: TPSL $01  ->  CC=EQ if C=1 (carry), CC=LT if C=0 (no carry).
;   Carry skip (no carry): BCTR,LT. Carry taken (carry): fall through or BCTR,EQ.
;   PREFERRED: Use WC idiom (CPSL $08 / lo-add / PPSL $08 / hi-add / CPSL $08) for 16-bit adds.
;   Unsigned compare: PPSL $02 / COMA or SUBA / CPSL $02.
;   Binary flag (0 or 1): after LODA CC=EQ(0) or GT(1); use BCTR,GT not COMI $01.
;   Single page: all addresses $0000-$1FFF; hi-byte carry impossible.
;
;        HI/LO OPERATOR CONVENTION
;   <ADDR = HIGH byte (bits 15:8)   e.g. <$1634 = $16
;   >ADDR = LOW  byte (bits  7:0)   e.g. >$1634 = $34
;
;        RAS DEPTH BUDGET (8-level hardware stack)
;   Every followed BSxx/ZBSR consumes one slot regardless of condition.
;   BCxx/ZBRR are plain branches -- no RAS cost.
;   PARSE_EXPR entry guard: SPSU/ANDI/COMI fires ERR_NEST if SP>=6 at entry.
;
;        SCRATCH REGISTER CONVENTIONS
;   R0  working register, arithmetic, I/O.
;   R1  index register (LODA/STRA BASE,R1); also PRINT_S16 digit buffer index.
;       Clobbered by INC_ET (INC_TMP/INC_EXP shared body). Callers verified safe.
;   R2  long-lived variable letter (DL_STORE/DO_ASK, preserved across PARSE_EXPR)
;       Never written by subroutines except DL_STORE, SE_NOTKW, and (v0.9)
;       PRINT_S16, which saves/restores it internally (see PRINT_S16's own
;       header) rather than being exempt from the rule.
;   R3  loop counter (BDRR/BIRR). SW expr-stack pointer.
;       GETLINE (v0.2, was RDLINE): IBUF offset, $FF=empty sentinel (",Rx+"
;       pre-increments before access -- see SWBASE convention -- so empty
;       must be one below the first valid index, not 0).
;
;        KNOWN LIMITATIONS
; Deliberate size/simplicity tradeoffs from the pBASIC65c02-style minimal
; design goal (see "Goal:" above), not oversights, unless marked BUG.
;
;   PARENTHESES: exactly 1 level of nestingallowed. A paren appearing INSIDE
;     another open paren's operand "((1+2))", "(1+(2+3))" triggers ERR_NEST.
;     SEPARATE parenthesized groups at SAME level allowed "(A+B)-(C+D)+(E+F)"
;
;   PRECEDENCE: flat, left to right - all six operators (+ - * / = <) sit
;     at ONE precedence level, no BODMAS/PEMDAS (v0.5). "1+2*3" evaluates
;     as "(1+2)*3" = 9, not 7. Use explicit parens (subject to the 1-level
;     limit above) to force a different grouping.
;
;   RELATIONAL OPERATORS: only = and < exist. No >, <=, >=, or <>, and no
;     built-in workaround for them (reversing operands mirrors < but
;     doesn't give you <=).
;
;   STATEMENT DISPATCH matches ONLY the first character of a line against
;     a 9-entry table (A/E/G/I/L/N/P/R/W for ASK/END/GOTO/IF/LIST/NEW/
;     PRINT/RUN/WR - see TOK_CHARS, v0.3). It first peeks the 2nd
;     character and requires it to be any LETTER
;
;   LINE STORAGE IS APPEND-ONLY - typed line number is accepted only if 
;     it's greater than every stored line (a plain append) or EXACTLY
;     equal to the current highest stored line - this replaces or deletes 
;     it outright if empty.
;
;   NO INPUT LENGTH VALIDATION: GETLINE writes into IBUF without bounds
;     check, accepting characters until CR/LF/NUL. Long lines will crash.
;
;   VARIABLES: 26 total, single uppercase letter A-Z only, each a 16-bit
;     signed integer. No arrays. No string variables - string LITERALS
;     work for PRINT, but can't be stored in or read back from a
;     variable.
;
;   NO FUNCTIONS: no ABS/SIN/RND/etc.
;
; VERSION HISTORY (pBASIC2650)
;
; V1.0  (Sep 2026) - Code Golf - ROMEND $7FB
;
; v0.11 (Sep 2026) - -32768 overflow bug fixed in both MUL16 and DIV16
;   - Root cause: TMPH:TMPL holds an unsigned MAGNITUDE after ABS_TMP (the
;     sign is extracted into NEGFLG separately), so it can legitimately
;     range 0-65535 - but ABS_TMP's two's-complement negate can't
;     represent +32768 in 16 bits, so a left/dividend operand of exactly
;     -32768 leaves |TMP| as the bit pattern $8000 (TMPH=$80) rather than
;     overflowing to something else. Both MUL16's MU_LP and DIV16's
;     DV_LP tested this magnitude's high byte with a raw SIGNED
;     comparison (BCTR,GT / SUBA+BCTR,LT,GT), which misreads any TMPH in
;     $80-$FF as "not positive" / "less than" regardless of the true
;     unsigned value - so both loops read TMPH=$80 as "there's nothing
;     left to do" and exited before their first pass. PRINT -32768*1 and
;     PRINT -32768/1 both gave 0 instead of -32768. Every other legal
;     operand has |value| <= 32767, so TMPH after ABS_TMP is always <=
;     $7F and this never manifests - the bug is specific to this one
;     magnitude.
;   - MUL16 fix: MU_LP's "BCTR,GT MU_ADD" (branch if TMPH looks positive)
;     replaced with "BCFR,EQ MU_ADD" (branch if TMPH is NOT exactly
;     zero) - covers both the ordinary GT case and the $80-$FF LT-but-
;     actually-nonzero case identically, same 2 bytes either way.
;   - DIV16 fix: DV_LP's high-byte comparison (TMPH vs SC0, the divisor's
;     magnitude high byte) needed a genuine unsigned GT/EQ/LT result, not
;     just a zero test, so the same BCFR trick doesn't apply here. First
;     attempt used PPSL $02 (unsigned compare mode, per this file's own
;     CC SEMANTICS header note - "Unsigned compare: PPSL $02 / COMA or
;     SUBA / CPSL $02") wrapped around the existing SUBA - assembled
;     clean, ran clean, and changed NOTHING (confirmed by testing - still
;     gave 0). Root cause of that: the oracle explicitly documents COM
;     only affects COMA/COMI/COMR/COMZ, not SUBA/ADDA - this file's own
;     header note is imprecise on that point. Fixed by switching to
;     COMA,R0 SC0 (a genuine non-destructive compare, which COM mode does
;     govern) in place of SUBA,R0 SC0; R0 no longer holds the difference
;     afterward, but nothing downstream needed it - both branches on
;     either side reload fresh. This is the pattern CMP_TMP_PE (v0.8)
;     already established elsewhere in the file for the sequential-hi/lo
;     byte-compare idiom generally; DV_LP's loop-continuation test had a
;     narrower version of the same hazard.
;   - Verified via pipbug_wrap throughout, including the failed first
;     DIV16 attempt (a wrong fix that assembles and "runs" is still
;     wrong - re-tested after every change, not just the last one):
;     PRINT -32768*1/-32768*2/-32768*-1/-32768*0 give -32768/0/-32768/0
;     (the *-1 and *2 cases both wrap correctly, since +32768 and -65536
;     both overflow 16-bit signed right back around); PRINT -32768/1,
;     /2, /-1, /3 give -32768/-16384/-32768/-10922, all hand-verified.
;     Re-ran the full v0.10 verification sweep (999*67, 128*129, 255*255,
;     256*300, -300*300, -1*-1, 30000*2, 100*100, plus the equivalent
;     division cases) to confirm neither fix disturbed the already-
;     correct range - all still match hand-computed 16-bit wraparound.
;     Full showcase RUN (incl. Mandelbrot) is byte-for-byte identical to
;     the pre-fix v0.10 output (the Mandelbrot's coordinate range never
;     reaches this specific magnitude); TRY_STORE_LINE, the paren-nesting
;     guard, flat precedence, and the original PAREN-NEST-02 crash-fix
;     expressions all still correct.
;   - ROMEND $0825 -> $082A (2085 -> 2090 bytes), +5: 0 for the MUL16 fix
;     (BCFR,EQ is the same 2 bytes as the BCTR,GT it replaced), +5 for
;     DIV16 (PPSL $02 + CPSL $02, 2 bytes each, plus BCTA replacing a
;     BCTR that fell out of relative range once the routine grew by 4
;     bytes - a genuine absolute-vs-relative cost, not padding). No
;     bugs currently known - see KNOWN BUGS above.
;
; v0.10 (Sep 2026) - MUL16 overflow bug fixed (was in KNOWN BUGS)
;   - Root cause: MU_LP's inline 16-bit decrement of the loop counter
;     TMPH:TMPL tested the wrong condition for "did the low byte borrow".
;     It branched directly off the CC left by "SUBI,R0 1 / STRA,R0 TMPL"
;     (BCFR,LT MU_TNB), but per the CC-after-SUB semantics documented in
;     the Oracle (and in this file's own CC SEMANTICS header note), CC=LT
;     there means only "decremented byte >= 128", not "a borrow actually
;     happened" - those coincide only when TMPL was 0 before the
;     subtract. For any TMPL in $81-$FF (i.e. whenever |left operand| had
;     a nonzero high byte to begin with, e.g. any |TMP| >= 256), nearly
;     every pass through the loop wrongly decremented TMPH too, driving
;     it to 0 and then negative within the first few iterations. Once
;     TMPH stopped being positive, the loop's own top-of-loop guard
;     (BCTR,GT MU_ADD, which only forces continuation while TMPH>0) no
;     longer held it open, so the loop fell back to exiting purely on
;     TMPL==0 - ending after |TMP| mod 256 passes instead of the full
;     |TMP| passes. PRINT 999*67 (TMP=999=$03E7) exited after exactly 231
;     passes (TMPL's own starting value) instead of 999: 231*67=15477,
;     the exact wrong answer reported. Confirmed identically on the
;     unmodified v0.9 file via pipbug_wrap before making any change.
;   - Fix: replaced the CC-off-SUBI test with the carry-flag test this
;     file's own DEC_ET already uses correctly for the same idiom
;     elsewhere (TPSL $01 / BCTR,EQ) - C=1 (no borrow) skips the hi-byte
;     decrement, C=0 (genuine borrow, TMPL wrapped $00->$FF) falls
;     through to it. +2 bytes (TPSL $01 adds one instruction the old
;     single-branch test didn't need); RAS-neutral, since neither TPSL
;     nor BCTR consume hardware-stack depth - preserves the "DEC_TMP
;     omitted, MUL16 call site RAS-depth-unsafe" constraint noted above
;     SETUP_MULDIV.
;   - Verified via pipbug_wrap, not by inspection alone: PRINT 999*67 now
;     gives 1397 (was 15477). Broad operand sweep incl. both-negative,
;     one-negative, zero, TMPL just above/below the $80 boundary (128,
;     129, 255, 256), and TMP >= 256 with varying TMPH (300, 999, 1000,
;     30000) all match hand-computed 16-bit wraparound products. Full
;     built-in showcase RUN (arithmetic/comparison self-checks, GOTO
;     loop, Mandelbrot render - the Mandelbrot body multiplies fixed-
;     point values that routinely exceed 256, i.e. exactly the buggy
;     range) completes without error; self-check lines are byte-for-byte
;     unchanged from the pre-fix run (they only ever multiply small
;     operands, so were never in the buggy range), but the Mandelbrot
;     ASCII-art itself is now visibly different on every row from the
;     pre-fix run, confirming the bug was silently corrupting it too.
;   - New bug surfaced during verification, NOT fixed here (see KNOWN
;     BUGS above): PRINT -32768*1 gives 0 instead of -32768 - a separate,
;     pre-existing |TMP|-overflow edge case unrelated to the above (loop
;     never reaches the decrement code this fix touches; reproduces
;     identically on the unmodified v0.9 file).
;   - ROMEND $0823 -> $0825 (2083 -> 2085 bytes), +2.
;
; v0.9 (Sep 2026]) - PRINT_S16 replaced with a flat power-of-10 loop
;   - Replaced the SW-stack recursive digit printer (PREC/PR_LP/PR_REC/
;     SWRETURN/PS_DONE, plus the MSG_MIN "32768" literal for the -32768
;     special case) with a flat repeated-subtraction loop against a
;     five-entry power-of-10 table - no recursion machinery at all,
;     hardware or software. PRINT_S16 was SWBASE's only consumer besides
;     EXPR, so SWBASE's sizing is now driven by EXPR alone.
;   - RAS safety was the reason PREC used SW-stack recursion in the first
;     place (DO_ERROR calls PRINT_S16 to print "@line" without unwinding
;     the hardware RAS first, so anything it does stacks on top of
;     whatever depth was already live when the error fired). Verified this
;     replacement adds no more depth than before, both by inspection
;     (every call it makes - NEG_EXP_BODY, VCOUT - is itself leaf-bottomed,
;     documented "NO BSTA inside" or the same COUT->DLAY chain every
;     character output already went through) and empirically: a
;     divide-by-zero nested inside the one level of paren nesting the
;     guard allows, from a RUN'd line, still correctly prints "?2@10"
;     matching pre-change behavior.
;   - Also drops the old implementation's -32768 special case: negating
;     $8000 two's-complement-overflows back to $8000, which the old
;     algorithm couldn't handle (hence the MSG_MIN string workaround); this
;     loop doesn't need it since it decomposes the raw bit pattern via
;     unsigned repeated subtraction regardless of how it was produced, and
;     $8000 unsigned IS 32768, the correct magnitude.
;   - Three real bugs surfaced and fixed during implementation, each found
;     by testing rather than inspection:
;     1. IORZ,R3 (meant to test the leading-zero flag) computes r0|=r3, not
;        a pure test of r3 - R0 still held the previous SUBA's leftover
;        value at that point, so the "test" was corrupted by garbage.
;        Fixed with an EORZ,R0 immediately before it.
;     2. CPSL $08 (clear With-Carry) sat AFTER ADDI,R1 1 in the loop
;        instead of before it. WC left on from the previous digit's PPSL
;        $08 (high-byte with-carry subtract) made ADDI silently add the
;        stale carry too, corrupting the digit by +1 on every pass after
;        the first (observed: R1 jumping $30->$32, skipping '1' entirely).
;        Fixed by moving CPSL $08 to the top of SUB_LOOP, before ADDI.
;     3. R2 and R3 were left clobbered on return. R2 is documented
;        elsewhere (register-conventions note) as a long-lived variable-
;        letter register never written outside DL_STORE/SE_NOTKW - the old
;        PREC-based implementation respected this (never in its own
;        Clobbers list); this loop uses R2 as its column index and needed
;        to save/restore it (in SC1). R3 needed the same treatment (in
;        SC0) despite the old implementation's Clobbers list including
;        it - evidently something downstream cared about the specific
;        value it was left with, not just that it changed. On top of
;        both, WC itself also needed clearing before return: the final
;        digit column always exits via BORROW (the ones-column
;        subtraction "overshoots by one" to detect completion, same as
;        every column), leaving WC on from that column's PPSL $08 with
;        nothing looping back through the SUB_LOOP-top fix to clear it -
;        so PRINT_S16 returned with WC still enabled, and the next
;        SUBA/ADDA anywhere downstream (the keyword matcher's own
;        character-compare SUBA, as it happened) silently absorbed a
;        stale carry-in. Symptom: a second PRINT statement's keyword
;        dispatch fell through to variable-assignment parsing (?4 ERR_VAR)
;        instead of recognizing PRINT, even after R2/R3 were confirmed
;        correctly restored by trace - isolated to leftover arithmetic-
;        mode state, not register contents, by swapping in a trivial
;        stub (print one fixed character, touch nothing else) and
;        confirming that did NOT reproduce the failure.
;   - Verified via full regression, not just re-running the showcase:
;     RUN of the full showcase (incl. Mandelbrot) is byte-for-byte
;     identical to the pre-change output; PRINT of 7, 0, -5, 100, 10,
;     12345, -1, -32768, 32767, 1000, 1001 (leading-zero and sign-extremes
;     coverage) as a sequence of separate statements (exercising the
;     R2/R3/WC-across-statement-boundary fixes specifically) all correct;
;     TRY_STORE_LINE append/out-of-order-reject/non-last-reject/delete-
;     last still correct; the original PAREN-NEST-02 crash-fix expressions
;     and the paren-depth guard (?8 on double-nesting) still correct.
;   - ROMEND $08C1 -> $0823 (2241 -> 2083 bytes), saving 158 bytes: -186
;     for the algorithm swap itself, +26 for the three correctness fixes
;     above (registers/WC save-restore) and +2 for MSG_MIN's removal
;     landing slightly differently than estimated. 35 bytes still needed
;     to clear the under-2KB ($0800) target.
;
; v0.8 (Sep 2026) - Size pass: CMP_TMP_PE extraction, PRT_BS removal
;   - Found via a duplicate-byte-sequence scan of the assembled ROM:
;     six near-identical inlined copies of the same 16-bit "is TMP at or
;     past PE" compare (DR_LP, TRY_STORE_LINE x2, FIND_LINE, FIND_INS,
;     DO_LIST) - the exact class of pattern PAREN-NEST-02 lives in
;     (program-end / line-storage bookkeeping). Factored into a single
;     CMP_TMP_PE subroutine returning the same CC (GT/LT/EQ) the inlined
;     code produced at each site, so every call site is a byte-for-byte
;     semantic match, not an approximation - most sites also dropped a
;     redundant duplicate LT-target branch once the compare's two
;     LT-producing paths (hi differs vs. lo borrows) collapsed into one
;     shared return.
;   - PRT_BS (print backspace) removed: BS has been unreachable dead code
;     since v0.2 dropped backspace support in GETLINE - nothing in the
;     build ever printed it. PRT_SPACE now ends the $EC skip-chain
;     directly instead of falling through into it.
;   - Verified via full regression, not just re-running the showcase:
;     RUN of the full showcase (incl. Mandelbrot) is byte-for-byte
;     identical to the pre-golf output; LIST and GOTO-driven execution
;     exercise FIND_LINE/FIND_INS/DLS_LP/DR_LP already, but
;     TRY_STORE_LINE's two sites needed a dedicated test since the
;     preloaded showcase never types a line in at the REPL: confirmed
;     legal append, out-of-order insert correctly rejected (?0), an
;     exact match on a non-last line correctly rejected (?0), and
;     delete-last (replace with empty body) correctly excises.
;   - ROMEND $090D -> $08C1 (2317 -> 2241 bytes), saving 76 bytes: 70
;     from the CMP_TMP_PE extraction, 3 from three branches the smaller
;     code let the assembler shrink to relative form, 3 from PRT_BS.
;     Net vs. the pre-fix v0.6 baseline ($08F3, 2291 bytes): 50 bytes
;     smaller, despite the v0.7 correctness fix's own +26-byte cost -
;     193 bytes still needed to clear the under-2KB ($0800) target.
;
; v0.7 (Sep 2026) - PAREN-NEST-02: fix wrong-operator bug in EXPR
;   - Root cause: OPS_HIT stashes the matched operator's handler address
;     in the flat GOTOH:GOTOL cells, then calls EXPR_ATOM to parse the
;     right operand, then jumps through *GOTOH. If that right operand is
;     itself a parenthesized sub-expression containing its OWN operator
;     (e.g. the "(B*B/64)" in "(A*A/64)-(B*B/64)"), the inner expression
;     recurses through EA_PAREN->EXPR_GUARDED, hits this same OPS_HIT for
;     its own "*"/"/", and overwrites the shared GOTOH:GOTOL before the
;     outer level gets back to use them - so the outer operator silently
;     runs as whatever the inner expression's last operator was instead.
;     This is the same clobbering hazard already identified and fixed
;     for the left operand via the SWBASE LIFO push (see the
;     PAREN-NEST-01-style comment at OPS_HIT) - just never extended to
;     GOTOH:GOTOL itself.
;   - Symptom: with A=-144, B=-64, "(A*A/64)-(B*B/64)" evaluated to 5
;     (i.e. 324/64, the INNER paren's own division) instead of 260
;     (324-64). This is exactly SHOWCASE line 405's
;     "F=(A*A/64)+(B*B/64)" - the corrupted arithmetic let the
;     Mandelbrot iteration's A/B values grow unboundedly across
;     iterations until a real divide-by-zero eventually fired
;     (?2@405), the reported crash.
;   - Fix: push GOTOH:GOTOL onto the same SWBASE stack as the left
;     operand (4 bytes per operator level instead of 2) immediately
;     before the recursive EXPR_ATOM call, and pop them back off right
;     after it returns, before the indirect jump. Cost: +26 bytes
;     (ROMEND $08F3 -> $090D).
;   - Verified: "(A*A/64)-(B*B/64)" now gives 260, matching the manual
;     calc; full showcase runs end-to-end with unlimited instructions,
;     including a complete, correctly-rendered Mandelbrot (classic bulb
;     silhouette, no errors). No regressions: single-paren nesting still
;     works, double-nesting still correctly refused with ?8 (ERR_NEST),
;     other multi-paren-group expressions ("(1+2)*(3+4)-(5+1)",
;     "(A+B)-(A-B)*2") all compute correctly under flat left-to-right
;     precedence.
;
; v0.6 (Sep 2026) - Size pass: dead-code cull, single-caller vector
;   collapse, and RAS-01 (Mandelbrot parenthesis-crash) fix.
;   - Deleted GETCI_UC and TMP_TO_EXP16: both orphaned (zero real call
;     sites; their "N sites" vector comments were stale). CUR_TO_EXP16
;     now falls straight into ET_TO_EXP16, dropping its own db $EC skip.
;   - Deleted RND_SHUFFLE/VRND_SHUFFLE/RNDSEED and MAIN's seed-priming
;     stub: functionally dead - RND_SHUFFLE only mutated RNDSEED, and
;     nothing ever read it back since RND() was cut in v0.1a. Also
;     dropped the ZBSR *VRND_SHUFFLE call in CHIN.
;   - DEC_IP/INC_EXP: sole ZBSR *Vxxx call site converted to direct
;     BSTA,UN; vector DW entry removed (1 byte saved each). Both stay
;     put as anchor points in the shared INC_ET/DEC_ET byte-skip chains.
;   - PUSH_RET (PREC's digit-printing recursion) and IP_TO_TMP (DO_LIST)
;     were each down to one caller: inlined their bodies directly at the
;     call site instead, removing both vectors and both routines
;     entirely (net 5 bytes saved each).
;   - TRY_STORE_LINE's TSL_ERR (BCTA,UN JSYNERR, 3 bytes) converted to
;     ZBRR *VJSYNERR (2 bytes) - reuses the vector PRO_NONE (PARSE_U16's
;     error surrogate) already needs, so no new vector required.
;   - RAS-01: SHOWCASE line 410 needed the outer paren around
;     (A*A/64)+(B*B/64) for correct grouping under flat left-to-right
;     precedence, plus each inner atom's own paren - two simultaneous
;     nesting levels. With MUL/DIV's own internal call chain that peaks
;     at SP=8, the hardware ceiling, past where the SP>=6 PE_RAS_LIMIT
;     guard can catch it (the guard only checks at paren-open, before
;     the inner atom is evaluated). Fixed by precomputing the sum on a
;     new line 405 (single paren level, sequential like line 390) and
;     simplifying line 410 to a bare-atom compare needing no parens.
;   - Corrected two stale comments (top-of-file RAS budget note, REC-01)
;     that still said SP>=5/PE_RAS_LIMIT=5; it's been 6 since v0.5.
;
; v0.5 (Aug 2026) - Stage 5: flatten precedence (Reading A)
;   - Replaced the multi-tier PARSE_EXPR/EAM_ATOM/EAM_HI/EAM_LO_LOOP + SW-
;     stack return trampolining (PUSH_RET/PARSER_RET/SWRETURN dance) with
;     a flat EXPR/EXPR_ATOM/EXPR_LOOP: all six operators (+-*/=<) sit at
;     one precedence, left to right - "1+2*3" evaluates as "(1+2)*3".
;     Return addresses now use ordinary hardware calls throughout
;     (Reading A, agreed in chat) - only genuine recursion (parens,
;     chained unary +/-) costs real RAS depth, not "which precedence
;     tier am I returning to". EXPR_ATOM is one shared routine for both
;     the first atom and every right-operand atom (smallest code, at the
;     cost of paren depth - see below).
;   - Relops (=, <) folded into the same flat operator table as ordinary
;     arithmetic (DO_EQOP/DO_LTOP), dispatched identically to +-*/.
;   - DO_IF collapsed to ~6 lines: evaluate the (now self-contained)
;     condition expression, test nonzero, jump to STMT_EXEC. THEN is
;     gone entirely - "IF a IF b stmt" nests for free since the true-path
;     dispatch is a jump, not a call, so chained IFs cost no extra depth
;     regardless of how many are stacked.
;   - Cut CHECK_POW (^) and its ERR_OV/POWCNTH/POWCNTL support (138 bytes
;     on its own) and % (MOD) - both deferred from Stages 1-4 specifically
;     because this stage replaces their home wholesale.
;   - PARSE_RELOP (the old = / < only parser from Stage 4) is gone too -
;     no longer called now that relops dispatch directly as operators.
;     RELOP RAM cell removed.
;   - RAS guard: PE_RAS_LIMIT set to 6 (of 8 hardware levels), matching
;     the project's declared 6-of-8 effective budget - 2 reserved for
;     CHIN/COUT's own internal delay-loop calls (their internal bit
;     timing costs 2 real stack slots each time they fire). NOT derived
;     from manual depth-counting, which proved unreliable during design
;     (caught myself making the arithmetic wrong twice) - measured via a
;     breakpoint at EXPR's entry during a real RUN, which showed SP=3 for
;     a completely ordinary, non-nested statement. The original
;     PE_RAS_LIMIT=3 placeholder was wrong in the conservative direction
;     - it rejected that ordinary case outright.
;   - Verified paren-nesting ceiling empirically, not asserted: exactly
;     1 level of paren nesting is safe (confirmed identical in both the
;     interactive prompt and mid-RUN contexts); 2 levels correctly
;     triggers ERR_NEST rather than corrupting state. This is fewer than
;     the 2 levels estimated on paper before implementation - the
;     estimate assumed inlining the first-atom parse separately from
;     EXPR_ATOM, which was not implemented (shared EXPR_ATOM costs 1 more
;     real call per level than that alternative would have, in exchange
;     for smaller code - a deliberate size-over-depth tradeoff given the
;     under-2KB target). Not a limitation in practice: the showcase uses
;     zero paren nesting anywhere, and 1 level covers ordinary grouping
;     like "(A+B)".
;   - SW-stack correction caught before it shipped: PRINT_S16's own
;     digit-printing recursion (PREC) independently reuses SWBASE/
;     PUSH_RET/SWRETURN/PS_DONE for its own unrelated purpose (up to 15
;     bytes for a 5-digit number - 3 bytes/digit level + a 2-byte outer
;     wrapper). SWBASE was corrected to 16 bytes, not shrunk to match the
;     expression evaluator's own much smaller need as first planned -
;     PRINT_S16's requirement dominates. This is a RAM-only correction;
;     it does not affect ROM size, since RES reserves address space
;     without emitting any bytes.
;   - Real bug caught by simulator trace, not the assembler: OPS_HIT
;     originally read the matched operator's handler address from the
;     TOK_CHARS scan position *after* parsing the right operand - but
;     EXPR_ATOM's descent into PARSE_FACTOR->PARSE_S16->PARSE_U16
;     clobbers TMPH:TMPL for its own digit-accumulation scratch
;     (EXP16_TO_TMP) along the way, so the "handler address" being read
;     back was garbage from that unrelated computation. Manifested as an
;     infinite loop (jumped to $2C09, well past ROMEND, all zero bytes)
;     on literally the simplest possible expression, "1+2". Found via a
;     breakpoint-driven trace (OPS_HIT reached correctly, DO_ADD never
;     reached even after 50000 instructions), not by inspection. Fixed
;     by extracting the handler address from the table immediately after
;     matching, before calling EXPR_ATOM for the right operand.
;   - Removed ORG $286/$2B4 (see below) - CHIN/COUT no longer pinned to
;     PIPBUG-compatible addresses; the gap this had been forcing had
;     grown to 168 bytes of pure padding as more code was cut over
;     Stages 1-4. Confirmed by measuring the address immediately before
;     the old ORG point ($01DE) against $0286 before removing it.
;   - Showcase: THEN removed from every IF (12 sites, including the
;     Mandelbrot renderer's nested "IF...THEN IF...THEN..." -> "IF...IF...");
;     the one "%5" (MOD) demo item removed with no replacement, matching
;     pBASIC65c02 (which never had MOD either); every relop already
;     narrowed to =/< in Stage 4 needed no further changes here.
;   - Verified: assembles 0 errors; full showcase runs correctly
;     end-to-end, including a complete, correct Mandelbrot render when
;     given enough instructions to finish (the render is visibly correct
;     - the classic bulb silhouette - confirming ADD/SUB/MUL/DIV/EQ/LT
;     and nested IF all work correctly together on real, non-trivial
;     arithmetic, not just the isolated arithmetic-demo cases); the
;     Stage 2 line-editor test and the Stage 3 abbreviation test both
;     pass unchanged; the paren-depth boundary (1 safe, 2 refused) was
;     independently confirmed at both the prompt and mid-RUN.
;   - ROM: ROMEND $0AE6 (2790) -> $093D (2365 bytes) - 257 bytes from
;     flattening precedence + cutting ^/%/PARSE_RELOP, 168 bytes from
;     removing the ORG pin. Running total since baseline: 4048 -> 2365,
;     -1683 bytes (42%). Still above the under-2KB target (317 bytes
;     over) - Stage 6's dedicated golf pass is the next and likely last
;     opportunity to close that gap.
;
; v0.4 (Aug 2026) - Stage 4: narrow relops to = and <, plus a golf pass
;   - PARSE_RELOP: was a loop accumulating '<'/'='/'>' into a 3-bit mask
;     (6 relops: = < > <= >= <>). Now matches only '=' or '<' directly;
;     RELOP is a plain 0/1 flag, no mask or loop needed. Anything else
;     ('>' included) is a syntax error.
;   - DO_IF's final dispatch (DIF_EW) golfed to match: was "map the 3-way
;     compare result to a bitmask, AND against RELOP" (needed for 6
;     possible relops); now just two direct checks exploiting LODA's
;     own CC-from-loaded-value behaviour (RELOP=0/'=' true iff SC1=0;
;     RELOP=1/'<' true iff SC1=$FF/bit7 set) - no bitmask, no AND. The
;     3-way LNUMH-vs-EXPH compare that produces SC1 is otherwise
;     unchanged (still needed to tell LT from EQ from GT) - restructuring
;     that further belongs to Stage 5's bigger precedence rewrite, not
;     this "small trim" stage.
;   - Showcase: every '>'/'<='/'>=' /'<>' in both the COMPARISONS demo and
;     the Mandelbrot renderer rewritten for '='/'<' only. Most were a
;     direct operand swap (X>Y == Y<X: "IF I>56" -> "IF 56<I", etc. - 6
;     sites total, including one in the GOTO-loop demo I initially
;     missed and caught on a second sweep). The one exception was line
;     421's "N<=16" loop-continue test, which isn't a simple swap;
;     restructured as "IF 16<N THEN GOTO 430" + a new line 422
;     ("GOTO 370") to fall through to on the untaken branch - logically
;     identical to the original, just expressed as two lines instead of
;     one relop pBASIC2650 no longer has.
;   - Verified: assembles 0 errors; full showcase (including a complete
;     re-verification that the Mandelbrot renderer produces byte-identical
;     output to every prior stage up to the same instruction-limit
;     cutoff) plus the Stage 2 line-editor and Stage 3 abbreviation tests
;     all pass unchanged.
;   - Golf pass (on request): swept the "BCTA/BSTA can use relative form"
;     warnings that had been accumulating since Stage 1a (deliberately
;     deferred each time rather than mixed into unrelated diffs) - 12
;     sites converted from absolute to relative branch/call form, each a
;     1-byte saving. Re-verified against all three regression tests
;     (showcase, line-editor, abbreviations) since even a purely
;     mechanical encoding change gets tested here, not assumed safe.
;   - ROM: ROMEND $0B00 (2816) -> $0AE6 (2790 bytes), -26 bytes (15 from
;     the relop narrowing itself, 11 from the golf pass). Running total
;     since baseline: 4048 -> 2790, -1258 bytes (31%).
;
; v0.3 (Aug 2026) - Stage 3: single-char + letter statement dispatch
;   - Replaced KW_TAB's 2-3 char match (MATCH_KW, stride 5) with TOK_CHARS
;     (stride 3, 1-char match) - ported from pBASIC65c02.asm's
;     MATCH_DISPATCH/TOK_CHARS/SKIP_KW. Peeks the 2nd character first: if
;     it isn't a letter, it can't be a multi-letter keyword, so it's
;     treated as bare "V=expr" (SE_NOTKW) without even scanning the table -
;     this is also why single-letter statement abbreviations don't work,
;     matching pBASIC65c02's own documented 2-letter minimum. Otherwise
;     the (still-unconsumed) 1st character is matched against TOK_CHARS;
;     on a hit, EATWORD consumes the whole keyword in one pass (it was
;     never actually read off IP, only peeked, so EATWORD picks it up
;     too), then control jumps to the handler.
;   - LET is gone: it collided with LIST on 1-char dispatch (both start
;     with 'L'), and bare "V=expr" (SE_NOTKW) already covered every
;     assignment - pBASIC65c02 itself has no LET keyword either. DO_LET's
;     own prologue (letter-parse + '=' check, reachable only via the
;     keyword) is gone with it; DL_EX/DL_STORE survive as the shared
;     assignment body, reached from SE_NOTKW and DO_ASK.
;   - Consequence worth knowing, not a bug: typing "LET X=5" now silently
;     runs LIST instead (L matches, EATWORD eats "LET" as if it were
;     LIST's own keyword body, "X=5" is then discarded unread) - this is
;     the same accepted tradeoff pBASIC65c02 itself has for any word
;     sharing a first letter with a keyword (e.g. "LOOP" would hit the
;     same way there). 1-char dispatch trades keyword precision for size;
;     this is the cost, not a defect.
;   - Caught a real bug before it shipped by testing, not by the
;     assembler: my 2nd-char peek loaded the character, then called
;     DEC_IP to restore IP - but DEC_IP clobbers R0 as part of its own
;     offset setup (EORZ,R0), destroying the just-peeked value before
;     UPCASE ever saw it. Every single input, including plain "NEW",
;     failed with a syntax error as a result - the assembler had nothing
;     to flag since the code was perfectly valid, just wrong. This is the
;     exact same gotcha GETCI_UC's own comment already documents
;     ("save before INC_IP clobbers R0") - I'd read that comment while
;     researching this stage and still didn't apply it to my own new
;     code until a breakpoint at the match target showed it was never
;     reached. Fixed by saving the peeked character in R1 across the
;     DEC_IP call, exactly as GETCI_UC already does.
;   - Verified: assembles 0 errors; full showcase and the Stage 2
;     line-editor test both pass identically; additionally verified
;     2-char abbreviations ("PR", "GO") dispatch correctly, and that bare
;     assignment ("X=7") still works with LET gone.
;   - ROM: ROMEND $0B17 (2839) -> $0B00 (2816 bytes), -23 bytes. Running
;     total since baseline: 4048 -> 2816, -1232 bytes (30%).
;
; v0.2 (Aug 2026) - Stage 2: minimal line handling
;   - TRY_STORE_LINE rewritten append-only (ported from pBASIC65c02.asm's
;     EDITLN): a line number is accepted only if greater than every stored
;     line (append) or exactly equal to the current LAST line (in-place
;     replace, or delete on an empty body). Anything else is a syntax
;     error. Reuses FIND_LINE directly (already needed by DO_GOTO) instead
;     of a separate scan - one exact-match check plus one "is it the last
;     record" check, no shifting, ever.
;   - Removed STORE_LINE, DELETE_LINE, MEMCPY (superseded), and DEC_LNUM/
;     DEC_GOTO (orphaned - they were STORE_LINE's shift-loop pointer
;     helpers). FIND_LINE/FIND_INS unchanged - DO_GOTO still needs a plain
;     "find this line" scan, which the new TRY_STORE_LINE also reuses.
;   - RDLINE renamed GETLINE (matching pBASIC65c02's naming) and
;     simplified: no backspace, no buffer-full check - matches
;     pBASIC65c02's own documented minimalism ("too many characters will
;     crash"). Echo and the NUL-from-CHIN check are both kept deliberately
;     (see GETLINE's header) - real usability and an inherited PIPBUG
;     hardware quirk respectively, neither is a pure size cut like
;     backspace/bounds-checking were.
;   - Caught a real bug while writing GETLINE, before assembling: an
;     over-simplification (`STRA,R1 IBUF,R3+`) hit the documented 2650
;     gotcha that indexed-autoincrement STRA only works for R0 - it
;     would have silently stored R0's value instead of R1's. Reverted to
;     the proven LODZ,R1/STRA,R0 pattern the original code already used.
;   - Caught two real bugs by assembling/running, not just by inspection:
;     (1) several of the new branches (BCTR) were out of relative-branch
;     range once the routine got long enough - the assembler's own
;     "relative offset out of range" errors caught these; converted to
;     BCTA (absolute). (2) A more serious one: I initially ported
;     pBASIC65c02's EDITLN body-copy logic verbatim, which copies "up to
;     and including CR" - correct there, because that system's GETLINE
;     leaves the CR embedded in IBUF. This system's GETLINE (see below)
;     NUL-terminates IBUF instead and never stores the CR at all, so the
;     ported copy loop had no terminator to find and ran away through
;     memory hunting for a CR byte that would never appear - a genuine
;     infinite loop, caught by testing an actual line-store round-trip
;     (append/replace/delete/illegal-edit) through the interpreter itself,
;     not by the assembler. Fixed by checking for NUL (via LODA's own
;     CC-from-loaded-value semantics, confirmed in the instruction-set
;     oracle) and manufacturing the CR terminator explicitly when writing
;     the record, instead of assuming the source contains one.
;   - Verified with a dedicated interactive test (append, append, replace
;     the last line, append again, delete the last line, attempt an
;     illegal out-of-order insert - correctly rejected with no state
;     change, then a legal replace of the new last line, then RUN) in
;     addition to the full showcase, since the showcase's program is
;     preloaded directly into ROM and never exercises TRY_STORE_LINE at
;     all on its own.
;   - ROM: ROMEND $0BD9 (3033) -> $0B18 (2840 bytes), -193 bytes. Running
;     total since baseline: 4048 -> 2840, -1208 bytes (30%).
;
; v0.1c (Aug 2026) - Stage 1c: narrow PRINT, add WR (resolves one of the
;   two items flagged unassigned at the end of v0.1b)
;   - Removed PRINT's CHR$(n)/TAB(n)/HEX$(n) special-casing (DP_CHAR, the
;     DP_TAB TAB() loop, DP_NOTC/DP_HEXITEM/PRINT_HEX_BYTE). DP_ITEM no
;     longer peeks for 'C'/'H'/'T' prefixes at all - straight from the `"`
;     string check to a plain expression, matching pBASIC65c02's minimal
;     PRINT (no DP_BACKUP undo-a-peek step needed either, since nothing
;     speculatively consumes a character anymore).
;   - Added DO_WR: `WR expr` writes the low byte of expr to COUT with no
;     newline - a statement, not a PRINT item, so it needs no PRINT/`;`
;     wrapper the way chained CHR$() calls did. Replaces CHR$'s role.
;   - TAB()/HEX$ have no replacement (matching pBASIC65c02, which never had
;     either).
;   - Showcase: "--- PRINT / CHR$ ---" demo now uses three WR statements
;     instead of one PRINT with three chained CHR$(); the Mandelbrot
;     renderer's two pixel-plotting lines (`PRINT CHR$(n);`) converted to
;     bare `WR n` - shorter, and no `;` needed since WR already has no
;     newline.
;   - `%` and `^` are still unassigned - deferred to Stage 5, since that
;     stage rewrites the expression evaluator (EAM_HI/CHECK_POW) wholesale
;     anyway; removing them from the current multi-tier code now would be
;     edited then immediately replaced.
;   - Verified: assembles 0 errors; showcase runs correctly end to end,
;     including the WR-based "ABC" demo and a full Mandelbrot render.
;
; v0.1b (Aug 2026) - Stage 1b: cut statements, LIST range; rename INPUT
;   - Removed FOR/NEXT (DO_FOR, DO_NEXT/DN_POP_EMPTY, FORSP/FORBASE,
;     GOTOFLG's "$03 direct NLP" dispatch case), GOSUB/RETURN (DO_GOSUB,
;     DO_RETURN/DRT_GO, SWSP/GSBASE, GOTOFLG's "$02" case), POKE, REM
;     (keyword only - the DO_REM label survives as DO_LET's shared exit),
;     FRE.
;   - LIST is now whole-program only: replaced DO_LIST+DO_POKE+
;     PARSE_2ARGS+every P2A_* handler (the whole shared arg-parsing block,
;     including the AND/OR/XOR bodies left as dead code from Stage 1a)
;     with a single DO_LIST that always prints PROG..PE. Also removed
;     CHECK_LPAREN/CHECK_RPAREN (orphaned - every caller was inside what
;     just got cut).
;   - Simplified DR_EXEC's GOTOFLG dispatch from 3-way to 2-way
;     (sequential/GOTO) - GOTOFLG is never set to $02/$03 anymore.
;   - Simplified DO_END - nothing left to clear but GOTOFLG/RUNFLG.
;   - Retired the error-jump chain entries for cut features (JERR_NXT,
;     JFORERR, DRT_UNDERFLOW) and their ERR_NXT/ERR_FOR/ERR_RET codes;
;     JERROOM is now the chain's last entry and falls through to DO_ERROR
;     directly instead of skip-jumping over a now-absent next entry.
;   - Renamed INPUT -> ASK (DO_INPUT -> DO_ASK) - I is taken by IF under
;     the 1-char statement dispatch coming in Stage 3.
;   - KW_TAB down to 9 statements: ASK, END, GOTO, IF, LET, LIST, NEW,
;     PRINT, RUN.
;   - RAM removed: FORVAR, FORSP/FORBASE (29B), SWSP/GSBASE (17B), ARGAH/
;     ARGAL/FUNCOP, and - a Stage 1a cleanup omission caught while doing
;     this - FT_SP/FT_STK/FT_SAVE_SP/FT_SAVE/FT_N/FT_R2SAVE (72B), which
;     should have gone when the functions that used them were cut.
;   - Showcase: removed the FOR/NEXT and GOSUB/RETURN demo blocks, the
;     now-unreachable line-530 GOSUB target, and simplified "LIST 40,60"
;     to plain "LIST".
;   - Not yet touched: relops (still all 6, still THEN-based), precedence
;     (still multi-tier, SWBASE still present), line store (still
;     sorted-insert), dispatch (still KW_TAB's 2-3 char match), CHR$/TAB/
;     HEX$/WR and the %/^ operators (unassigned - see chat for the open
;     question on which stage these land in).
;   - Verified: assembles 0 errors; showcase runs correctly end to end.
;
; v0.1 (Aug 2026) - Initial fork from uBASIC2650.asm v4.9
;   - Stage 1a: cut functions (ABS, AND, NEG, NOT, OR, PEEK, RND, USR, XOR)
;     and their FUNC_TAB dispatch: DO_ABS_FUNC, DO_AND_FUNC/DO_OR_FUNC/
;     DO_XOR_FUNC (stub entries only -- the P2A_ANDOP/OROP/XOROP bodies
;     inside PARSE_2ARGS are left in place as dead code for now, since
;     PARSE_2ARGS is still load-bearing for POKE/LIST until Stage 1b),
;     DO_NEG_FUNC, DO_NOT_FUNC, DO_PEEK_FUNC/DO_USR_FUNC, DO_RND_FUNC.
;   - Removed FUNC_TAB itself, FUNC_EPILOG/FUNC_CONT, and the VFUNC_CONT
;     vector (no callers left once every function handler is gone).
;   - Simplified PARSE_EXPR's entry (old PE_SAFE/PE_NOFUNC/PE_NOFUNC_TOP)
;     and EAM_ATOM's FUNCATOM-01 mid-expression function scan: both used
;     to scan FUNC_TAB via MATCH_KW before falling back to PARSE_FACTOR on
;     a miss. With no functions left, both now go straight there with no
;     scan at all. This also removes the FT_SP/FT_STK/FT_SAVE_SP/FT_SAVE/
;     FT_N/FT_R2SAVE byte-copy dance entirely -- it existed solely to
;     protect SWBASE around a function argument's own nested PARSE_EXPR
;     call.
;   - KEPT (shared with surviving core arithmetic -- verified by tracing
;     every caller before cutting): NEG_EXP/NEG_EXP_BODY/NEG_SHARED and
;     ABS_TMP/ABS_EXP (used by unary minus, subtraction, and MUL/DIV sign
;     handling); RND_SHUFFLE/RNDSEED (CHIN mixes an entropy bit into
;     RNDSEED on every keypress, independent of RND() -- confirmed in
;     CHIN's own body, left untouched per the CHIN/COUT PIPBUG-verbatim
;     note above).
;   - KEPT FOR NOW (genuinely shared with POKE/LIST, not yet cut):
;     PARSE_2ARGS, ARGAH/ARGAL/FUNCOP, CHECK_LPAREN/CHECK_RPAREN. Fully
;     removable once Stage 1b cuts POKE and simplifies LIST to
;     whole-program only.
;   - Showcase: removed the "--- FUNCTIONS ---" demo block (old lines
;     220-234: ABS/NEG/AND/OR/XOR/NOT demo, the POKE/PEEK pairing, RND
;     demo) since none of it can run without functions. POKE's statement
;     handler itself is untouched here -- only its now-orphaned showcase
;     line (paired with the removed PEEK readback) was dropped.
;   - Not yet touched: statements, LIST range, relops, precedence, line
;     store, dispatch. See plan.md for the remaining stages.
;   - Verified: assembles clean (0 errors); showcase runs end-to-end in
;     pipbug_wrap through the Mandelbrot finale with output unchanged
;     apart from the removed FUNCTIONS section.
;
; =============================================================================

;  ASCII Defines
CR      EQU     $0D
LF      EQU     $0A
BS      EQU     $08
SP      EQU     $20
NUL     EQU     $00
DQ      EQU     $22

;  ERROR Defines
ERR_SYN         EQU '0'
; ERR_UND_LINE    EQU '1'         ; unused
ERR_DIV_ZERO    EQU '2'
ERR_OOM         EQU '3'
ERR_VAR         EQU '4'
ERR_NEST        EQU '8'         ; Expression nesting too deep (RAS guard, v3.2 had '5')

; RAS (hardware Return Address Stack) Defines
RAS_DEPTH       EQU 8           ; 2650 HW RAS depth (SPSU field is 3 bits, 0-7)
; PE_RAS_LIMIT (v0.5): measured empirically, not asserted - a breakpoint at
; EXPR's entry during a normal, non-nested statement (RUN in progress)
; showed SP=3 there, which is a completely ordinary, safe case (the
; original placeholder of 3 was wrong - it rejected this). Effective
; budget is 6 of the 8 hardware RAS levels (2 reserved for CHIN/COUT's
; own internal delay-loop calls, per the project's standing design
; margin). Refuses at SP>=6, the true ceiling for this project.
PE_RAS_LIMIT    EQU 6

; PSW Defines
PSW_RS          EQU     $10
PSW_WC          EQU     $08             ; WC (With Carry) bit in PSL (bit 3)
PSW_FLAG        EQU     $40

; System Defines
PROGLIM         EQU $1FFF   ; top of program store (numeric constant, not address)

;  CODE starts at Zero (No Pipbug)
        ORG 0

; =============================================================================
;  RESET / ENTRY + PAGE-ZERO VECTOR TABLE
; In:  nothing (cold start)
; Out: nothing
;
; Page-zero subroutine vector table.
; Each DW entry holds the absolute address of the subroutine.
; Callers use ZBRR/ZBSR *Vxxx (2 bytes) vs BCTA/BSTA,UN xxx (3 bytes)
;
RESET:
        BCTR,UN MAIN            ; trampoline over vector table ($0000)
VINC_IP:
        DW INC_IP               ; 28 sites
VWSKIP:
        DW WSKIP                ; 23 sites
VINC_TMP:
        DW INC_TMP              ; 23 sites
VCOUT:
        DW COUT                 ; 10 sites
VPARSE_EXPR:
        DW EXPR                 ; 19 sites (v0.5: was PARSE_EXPR)
VEATWORD:
        DW EATWORD              ; 7 sites
VSET_IP_IBUF:
        DW SET_IP_IBUF          ; 4 sites
VPRT_SPACE:
        DW PRT_SPACE            ; 4 sites (v0.1c: was 5 - TAB()'s call went
VCLR_EXP:
        DW CLR_EXP              ; 5 sites
VDO_ERROR:
        DW DO_ERROR             ; 3 sites
VJSYNERR:
        DW JSYNERR              ; multiple sites
VDR_LP:
        DW DR_LP                ; 3 sites
VCLR_RUNFLG:
        DW CLR_RUNFLG           ; 3 sites
VEXP16_TO_LNUM:
        DW EXP16_TO_LNUM        ; 4 sites (was 5; DO_LIST now sets LNUM
VSET_TMP_PROG:
        DW SET_TMP_PROG

MAIN:
       ; Delete for ROM 
        LODI,R0 <SHOWCASE_END
        STRA,R0 PEH
        LODI,R0 >SHOWCASE_END
        STRA,R0 PEL

        ; clear flags - change to DO_NEW for ROM
        BSTA,UN DO_END          

        ; print sign-on banner
        LODI,R0 <BANNER
        STRA,R0 IPH
        LODI,R0 >BANNER
        STRA,R0 IPL
        BSTA,UN PRTSTR
        ; fall through to REPL

; =============================================================================
;  REPL -- Main read-eval-print loop
; In:  nothing
; Out: loops forever
; Clobbers: all
REPL:
        CPSL PSW_RS + 7             ; primary reg bank; clear PSL CC/flag bits
        CPSU $07                    ; clear PSU SP field (bits 2:0 = HW RAS depth)
                                     ; MUST be separate from CPSL: SP is in PSU not PSL
        LODI,R0 '>'                    ; print prompt only used here
        ZBSR *VCOUT  
        ZBSR *VPRT_SPACE  
        BSTA,UN GETLINE
        ZBSR *VSET_IP_IBUF                ; IPH:IPL = IBUF
        BSTA,UN TRY_STORE_LINE           ; CC=GT: line stored/deleted; CC=EQ: not a line
        BSTR,EQ STMT_EXEC               ; If CC=EQ (not a line), execute
        BCTR,UN REPL

; =============================================================================
;  DO_IF -- Conditional execution (v0.5: THEN is gone - relops are now
;  flat operators inside EXPR itself, so the expression's own result IS
;  the whole truth value. No separate relop parse, no THEN keyword.
;  Nests for free: "IF a IF b stmt" - the true-path dispatch is a JUMP to
;  STMT_EXEC, so if "stmt" is itself another IF, it costs no extra depth.)
; Syntax: IF expr stmt
; In:  IP -> first char after IF keyword
; Out: executes stmt if expr is nonzero; otherwise sequential return
; Clobbers: R0, R1, EXPH, EXPL (via EXPR, plus whatever the dispatched
;           statement clobbers on the true path)
DO_IF:
        BSTA,UN EXPR                      ; [+1] condition -> EXPH:EXPL
        LODA,R0 EXPL
        IORA,R0 EXPH
        RETC,EQ                           ; both zero: false, sequential return
        ; drop through
; =============================================================================
;  STMT_EXEC -- Decode and dispatch one BASIC statement from IP.
; In:  IPH:IPL -> first char of statement (after any leading whitespace)
; RAS depth: 1 from REPL, 3 from DO_IF(THEN body).
; Worst inner depth from here: +4 (DO_xxx->PARSE_EXPR->PARSE_FACTOR->UPCASE)
; v0.3: 1-char + letter statement dispatch (was 2-3 char KW_TAB match).
; Peeks the 2nd character first: if it isn't a letter, this can't be a
; multi-letter keyword, so it's treated as a bare "V=expr" assignment
; (SE_NOTKW) without even attempting a keyword match - this is also why
; single-letter statement abbreviations don't work, matching pBASIC65c02's
; own documented limitation (2 letters minimum). Otherwise the
; (still-unconsumed) 1st character is matched against TOK_CHARS; on a
; hit, EATWORD consumes the whole keyword - 1st char included, since it
; was only peeked, never actually read off IP - in one pass, then control
; jumps to the handler. 
STMT_EXEC:
        ZBSR *VWSKIP  
        ; peek 2nd char (IP+1) without consuming
        ZBSR *VINC_IP  
        LODA,R0 *IPH
        STRZ,R1                           ; save peeked char - DEC_IP clobbers R0
        BSTA,UN DEC_IP                    ; v0.6: was ZBSR *VDEC_IP (sole caller now direct)
        LODZ,R1                           ; restore
        BSTA,UN UPCASE                    ; [+1]
        COMI,R0 A'A'
        BCTR,LT SE_NOTKW                  ; not a letter: bare assignment
        COMI,R0 A'Z'+1
        BCTR,GT SE_NOTKW                  ; not a letter: bare assignment

        LODI,R0 <TOK_CHARS
        STRA,R0 TMPH
        LODI,R0 >TOK_CHARS
        STRA,R0 TMPL
        LODA,R0 *IPH                      ; peek 1st char (not consumed)
        BSTA,UN UPCASE                    ; [+1]
        STRA,R0 SC0
MD_SCAN:
        LODA,R0 *TMPH                     ; table char
        BCTR,EQ SE_NOTKW                  ; NUL row: no match -> bare assignment
        SUBA,R0 SC0
        BCTR,EQ MD_HIT
        ZBSR *VINC_TMP  
        ZBSR *VINC_TMP  
        ZBSR *VINC_TMP  
        BCTR,UN MD_SCAN
MD_HIT:
        ZBSR *VEATWORD                    ; [+1] consume the whole keyword
        ZBSR *VINC_TMP  
        LODA,R0 *TMPH
        STRA,R0 GOTOH                     ; handler hi
        ZBSR *VINC_TMP  
        LODA,R0 *TMPH
        STRA,R0 GOTOL                     ; handler lo
        BCTA,UN *GOTOH                    ; indirect jump

SE_NOTKW:
        ; Bare variable assignment ("X=expr" - either the 2nd-char peek
        ; above wasn't a letter, or the 1st char matched no statement).
        ; Neither path consumes anything from IP, unlike the old KW_TAB
        ; scan (which always ate 2 chars first) - no rewind needed here.
        BSTA,UN PARSE_VAR_SAVE            ; validates A-Z, SC0/R2 = letter, IP -> past it
        ZBSR *VWSKIP
        LODA,R0 *IPH
        COMI,R0 A'='
        BCFA,EQ JSYNERR
        ZBSR *VINC_IP
        ; drop through
; =============================================================================
;  DL_EX / DL_STORE -- Variable assignment (v0.3: DO_LET's own prologue,
;  reachable only via the explicit "LET" keyword, is gone along with LET
;  itself - see STMT_EXEC. This is now reached only from SE_NOTKW's bare
;  "V=expr" path and from DO_ASK.)
; In:  IP -> expression (DL_EX) or SC0/R2 = var letter, EXPH:EXPL = value (DL_STORE)
; Out: VARS[V] = EXPH:EXPL
; Clobbers: R0, R1, EXPH, EXPL, TMPH, TMPL (via PARSE_EXPR, DL_EX only)
DL_EX:
        ZBSR *VPARSE_EXPR                 ; [+1]
DL_STORE:
        LODZ,R2          ; R0 = R2 (Variable character letter)
        SUBI,R0 A'A'     ; R0 = R0 - 'A' (0 to 25)
        ADDZ,R0          ; R0 = R0 * 2 (Double for 16-bit word stride)
        STRZ,R1          ; R1 = R0 (Transfer offset to Index Register R1)
        LODA,R0 EXPH     ; R0 = High byte of expression
        STRA,R0 VARS,R1  ; Store directly to VARS array + offset
        LODA,R0 EXPL     ; R0 = Low byte of expression
        STRA,R0 VARS+1,R1; Store directly to VARS array + offset + 1
        RETC,UN

; =============================================================================
;  DO_ASK -- Read signed integer from user into variable (v0.1b: renamed
;  from INPUT - I is taken by IF under 1-char statement dispatch)
; Syntax: ASK V
; In:  IP -> variable letter
; Out: VARS[V] = parsed value
; Clobbers: R0, R2, SC0, SC1, EXPH, EXPL, TMPH, TMPL, IBUF
DO_ASK:
        BSTR,UN PARSE_VAR_SAVE
        BSTA,UN PRT_QUEST
        ZBSR *VPRT_SPACE  
        BSTA,UN GETLINE                   ; [+1]
        ZBSR *VSET_IP_IBUF                ; IPH:IPL = IBUF
        BSTA,UN PARSE_S16                ; [+1]
        BCTR,UN DL_STORE

; =============================================================================
;  DO_GOTO -- Computed GOTO
; Syntax: GOTO expr
; In:  IP -> first char after GOTO keyword
; Out: GOTOH:GOTOL = target line; GOTOFLG=$01
; Clobbers: R0, EXPH, EXPL, GOTOH, GOTOL, GOTOFLG
DO_GOTO:
        ZBSR *VWSKIP  
        ZBSR *VPARSE_EXPR                 ; [+1]
        BSTA,UN EXP16_TO_GOTO             ; GOTOH:GOTOL = EXPH:EXPL
        LODI,R0 1
        STRA,R0 GOTOFLG
        LODA,R0 RUNFLG                   ; OPT-10
        RETC,GT                          ; return if running
        ZBRR *VCLR_RUNFLG 

; =============================================================================
;  SET_TMP_PROG -- Set TMPH:TMPL = PROG base address
; Clobbers: R0
SET_TMP_PROG:
        LODI,R0 <PROG
        STRA,R0 TMPH
        LODI,R0 >PROG
        STRA,R0 TMPL
        RETC,UN

; =============================================================================
; PARSE_VAR_SAVE -- skip whitespace, read+upcase var letter, range-check,
;                   save to SC0 and R2, advance IP.
; Out: SC0=R2=letter (A-Z); IP advanced past letter
; Error: tail-jumps to JERRVAR (no return)
; Clobbers: R0, R1, R2, SC0
PARSE_VAR_SAVE:
        ZBSR *VWSKIP  
        LODA,R0 *IPH
        BSTA,UN UPCASE
        COMI,R0 A'A'
        BCTA,LT JERRVAR       ; out of range low  -- tail jump, no return
        COMI,R0 A'Z'+1
        BCFA,LT JERRVAR       ; out of range high -- tail jump, no return
        STRA,R0 SC0
        STRZ,R2                          ; save in R2 for DL_STORE
        ZBRR *VINC_IP           ; tail call  

; =============================================================================
;  DO_NEW -- Reset PE and IP
; Syntax: NEW
; In:  nothing
DO_NEW:
        ; set both PEH:PEL and IPH:IPL to PROG in one pass
        LODI,R0 <PROG
        STRA,R0 PEH
        STRA,R0 IPH
        LODI,R0 >PROG
        STRA,R0 PEL 
        STRA,R0 IPL
        ; fall through to DO_END

; =============================================================================
;  DO_END -- Stop execution and clear all run state
; Syntax: END  (also called by DO_NEW, DO_ERROR, RESET)
; In:  nothing
; Out: GOTOFLG=0, RUNFLG=0
; Clobbers: R0
DO_END:
        EORZ,R0
        STRA,R0 GOTOFLG
        ZBRR *VCLR_RUNFLG               ; tail call

; =============================================================================
; Character IO
; 110 Baud teletype from PIPBUG V1 as per Signetics M20 application note
; v0.5: ORG $286 pin removed - CHIN/COUT no longer forced to PIPBUG-
; compatible addresses. That pin cost 168 bytes of pure padding once
; enough code was cut that the gap between here and the previous content
; grew that large (confirmed by measuring the address right before this
; point against $0286 before removing the pin). CHIN/COUT now float to
; wherever they land; their real addresses must be read from the .LST
; file and passed to pipbug_wrap's --chin/--cout flags for testing,
; since the old fixed 0x286/0x2B4 no longer apply.
CHIN:
        PPSL PSW_RS
        LODI,R0 $80
;        WRTC,R0        ; make space for shuffle
        LODI,R1 0
        LODI,R2 8
;ACHI:   
        SPSU
        BCTR,LT CHIN
        EORZ,R0
;        WRTC,R0        ; make space for shuffle
        BSTR,UN DLY
BCHI:
        BSTR,UN DLAY
        SPSU
        ANDI,R0 $80
        RRR,R1
        IORZ,R1
        STRZ,R1
        BDRR,R2 BCHI
        BSTR,UN DLAY
        ANDI,R1 $7f
        LODZ,R1
        CPSL PSW_RS + PSW_WC
        RETC,UN
; Delay for 1 bit time
DLAY:
        EORZ,R0
        BDRR,R0 $
        BDRR,R0 $
DLY:
        BDRR,R0 $
        LODI,R0 $e5
        BDRR,R0 $
        RETC,UN

COUT:
        PPSL PSW_RS
        PPSU PSW_FLAG
        STRZ,R2
        LODI,R1 8
        BSTR,UN DLAY
        BSTR,UN DLAY
        CPSU PSW_FLAG
ACOU:
        BSTR,UN DLAY
        RRR,R2
        BCTR,LT ONE
        CPSU PSW_FLAG
ONE:
        PPSU PSW_FLAG
;ZERO:
        BDRR,R1 ACOU
        BSTR,UN DLAY
        PPSU PSW_FLAG
        CPSL PSW_RS
        RETC,UN

; =============================================================================
;  DO_PRINT / PRTSTR -- Print statement and NUL-terminated string helper
; Syntax: PRINT [item {; item}]
;   item = "string" | expr
;   Trailing ; suppresses newline. (v0.1c: TAB(n)/CHR$(n)/HEX$(n) removed -
;   CHR$ is replaced by the WR statement; TAB/HEX$ dropped with no
;   replacement, matching pBASIC65c02's minimal PRINT.)
; In:  IP -> first char after PRINT keyword
; Out: text written to COUT; IP advanced past statement
; Clobbers: R0, R1, EXPH, EXPL, TMPH, TMPL, NEGFLG, LNUMH, LNUML, SC0, SC1
DO_PRINT:
        ZBSR *VWSKIP  
        LODA,R0 *IPH
        BCTR,EQ DP_NL

DP_ITEM:
        ZBSR *VWSKIP  
        LODA,R0 *IPH
        COMI,R0 DQ
        BCTR,EQ DP_STRING
DP_EXPR:
        ZBSR *VPARSE_EXPR  
        BSTA,UN PRINT_S16
        BCTR,UN DP_SEP

DP_SEP:
        ZBSR *VWSKIP  
        LODA,R0 *IPH
        COMI,R0 $3B             ; semicolon
        BCTR,EQ DP_SEMI
        ; fall through to DP_NL
DP_NL:
        BCTA,UN PRT_CRLF          ; tail call: return from DO_PRINT

DP_SEMI:
        ZBSR *VINC_IP  
        ZBSR *VWSKIP  
        LODA,R0 *IPH
        RETC,EQ                 ; bail if NUL
        BCTR,UN DP_ITEM

DP_STRING:
        ZBSR *VINC_IP  
PRTSTR:
        LODA,R0 *IPH
        RETC,EQ                 ; NUL before closing ": bail
        COMI,R0 DQ
        BCTR,EQ DP_SCLS
        ZBSR *VCOUT  
        ZBSR *VINC_IP  
        BCTR,UN PRTSTR

DP_SCLS:
        ZBSR *VINC_IP  
        BCTR,UN DP_SEP

; =============================================================================
;  DO_WR -- Write raw character (v0.1c: replaces CHR$, which is cut. This is
;  a statement, not a PRINT item - consecutive WRs need no PRINT/semicolon
;  wrapper the way "PRINT CHR$(a);CHR$(b);" did.)
; Syntax: WR expr
; In:  IP -> first char after WR keyword
; Out: low byte of expr's value written to COUT (no newline)
; Clobbers: R0, EXPH, EXPL, TMPH, TMPL, NEGFLG, LNUMH, LNUML, SC0, SC1
DO_WR:
        ZBSR *VWSKIP  
        ZBSR *VPARSE_EXPR  
        LODA,R0 EXPL
        ZBRR *VCOUT              ; tail call

; =============================================================================
;  CMP_TMP_PE -- Compare TMPH:TMPL against PEH:PEL (16-bit, byte-serial,
;  proper unsigned semantics via carry rather than SUBA's own CC, which is
;  only reliable over half the 0-255 byte range - see the TPSL $01 note
;  below). Factored out of six near-identical inlined copies (DR_LP,
;  TRY_STORE_LINE x2, FIND_LINE, FIND_INS, DO_LIST) - v0.7 size pass.
; Out: CC=GT if TMPH:TMPL >  PEH:PEL (hi bytes differed)
;      CC=LT if TMPH:TMPL <  PEH:PEL (hi bytes differed, or hi equal and
;            the lo-byte subtraction borrowed)
;      CC=EQ if hi bytes equal and lo bytes did not borrow (TMPL>=PEL -
;            ambiguous between "exactly equal" and "TMPL>PEL"; every call
;            site already only relied on this same ambiguous EQ meaning,
;            since TMP is guaranteed <= PE by construction wherever it's
;            used, so the two coincide in practice)
; Clobbers: R0
CMP_TMP_PE:
        LODA,R0 TMPH
        SUBA,R0 PEH
        BCTR,EQ CTP_LOBYTE                ; hi equal: lo bytes decide
        RETC,UN                           ; hi differed: SUBA's own CC
                                           ; (GT/LT) is already the answer
CTP_LOBYTE:
        LODA,R0 TMPL
        SUBA,R0 PEL
        TPSL $01                          ; carry-based unsigned compare:
        RETC,UN                           ; C=1 (no borrow) -> EQ, C=0 -> LT

; =============================================================================
;  DO_RUN -- Execute stored program
; Syntax: RUN
; In:  PROG=program base, PEH:PEL=program end
; Out: runs until END, error, or exhausted; returns to REPL
; Clobbers: all
; GOTOFLG after STMT_EXEC: $00=sequential, $01=GOTO.
; v0.1b: was a 3-way dispatch ($00/$02=GOSUB/$03=FOR direct NLP); GOSUB and
; FOR/NEXT are cut, so GOTOFLG is never set to $02 or $03 anymore - just a
; plain sequential-vs-GOTO check now.
DO_RUN:
        LODI,R0 1
        STRA,R0 RUNFLG
        EORZ,R0
        STRA,R0 GOTOFLG
        ZBSR *VSET_TMP_PROG
DR_LP:
        LODA,R0 RUNFLG
        RETC,EQ
        ; end of program? TMPH:TMPL >= PEH:PEL
        BSTR,UN CMP_TMP_PE
        BCTA,GT DR_STOP
        RETC,EQ
DR_EXEC:
        ; save current line number for error reporting
        LODA,R0 *TMPH
        STRA,R0 CURH
        ZBSR *VINC_TMP  
        LODA,R0 *TMPH
        STRA,R0 CURL
        ZBSR *VINC_TMP  
        ; copy body to IBUF until CR, NUL-terminate
        ZBSR *VSET_IP_IBUF                ; IPH:IPL = IBUF
DR_CPY:
        LODA,R1 *TMPH
        COMI,R1 CR
        BCTR,EQ DR_CD
        STRA,R1 *IPH
        ZBSR *VINC_TMP  
        ZBSR *VINC_IP  
        BCTR,UN DR_CPY
DR_CD:
        ZBSR *VINC_TMP                    ; skip past CR in store
        EORZ,R0
        STRA,R0 *IPH                     ; NUL-terminate IBUF
        ; Save next-line pointer into SWSTK before STMT_EXEC clobbers SC0/SC1.
        ; SWSTK persists across STMT_EXEC; DO_GOSUB and DO_FOR read from it.
        LODA,R0 TMPH
        STRA,R0 SWSTK
        LODA,R0 TMPL
        STRA,R0 SWSTK+1
        ; execute line
        ZBSR *VSET_IP_IBUF                ; IPH:IPL = IBUF
        BSTA,UN STMT_EXEC                ; [+1]
        ; dispatch on GOTOFLG
        LODA,R0 GOTOFLG
        BCTR,EQ DR_SEQ                   ; $00: sequential
        BCTR,UN DR_GOTO                  ; $01: GOTO (only remaining setter)
DR_SEQ:
        LODA,R0 SWSTK
        STRA,R0 TMPH
        LODA,R0 SWSTK+1
        STRA,R0 TMPL
        ZBRR *VDR_LP 
DR_GOTO:
        ; GOTOFLG=$01 (GOTO).
        EORZ,R0
        STRA,R0 GOTOFLG
        LODA,R0 GOTOH
        STRA,R0 EXPH
        LODA,R0 GOTOL
        STRA,R0 EXPL
        ZBSR *VEXP16_TO_LNUM             ; LNUMH:LNUML = GOTOH:GOTOL (target line)
        BSTA,UN FIND_LINE                ; [+1] sets TMPH:TMPL
        ZBRR *VDR_LP 
DR_STOP:
        ; fall through to CLR_RUNFLG

; =============================================================================
;  CLR_RUNFLG -- Clear run flag
; In:  nothing
; Out: RUNFLG=0
; Clobbers: R0
CLR_RUNFLG:
        EORZ,R0
        STRA,R0 RUNFLG
        RETC,UN

; =============================================================================
;  TRY_STORE_LINE -- Store or delete a numbered line if IP starts with a digit
; In:  IPH:IPL -> input buffer
; Out: CC=GT if line stored/deleted; CC=EQ if not a numbered line
; Clobbers: R0, EXPH, EXPL, LNUMH, LNUML, TMPH, TMPL, CURH, CURL
; v0.2: append-only rewrite (was sorted-insert via STORE_LINE/DELETE_LINE/
; FIND_INS's shift-copy loops). A line number is only accepted if it's
; greater than every stored line (plain append) or exactly matches the
; current LAST stored line (in-place replace, or delete if the typed body
; is CR-only). Anything else - a match on a non-last line, or a number
; that falls before/between existing lines - is a syntax error. Because
; storage order is thereby kept == numeric order, GOTOL/DO_RUN/DO_LIST
; need no changes: they already walk storage order top to bottom and
; never assumed sorted-by-insertion order themselves. Ported from
; pBASIC65c02.asm's EDITLN (see plan.md).
TRY_STORE_LINE:
        LODA,R0 *IPH
        COMI,R0 A'0'
        BCTR,LT TSL_NO                   ; not a digit
        COMI,R0 A'9'+1
        BCTR,LT TSL_NUM
TSL_NO:
        EORZ,R0                          ; CC=EQ: not a numbered line
        RETC,UN
TSL_NUM:
        ZBSR *VWSKIP  
        BSTA,UN PARSE_S16                ; [+1]
        LODA,R0 EXPH
        BCTR,GT TSL_NZ
        LODA,R0 EXPL
        BCTR,EQ TSL_NO                   ; line number zero: not stored
TSL_NZ:
        ZBSR *VEXP16_TO_LNUM             ; LNUMH:LNUML = EXPH:EXPL (parsed line number)
        ZBSR *VWSKIP                      ; [+1] skip space after line number
        BSTA,UN FIND_LINE                ; [+1] TMP=matched record (CC=EQ), or
                                          ; FIND_INS's insertion point (CC=GT)
        BCTR,EQ TSL_MATCH                ; exact match exists somewhere in the store
        ; No exact match: TMP = first record with line > target, or PE if
        ; none. Legal only if TMP == PE (target exceeds every stored line -
        ; a plain append); otherwise some stored line already exceeds
        ; target with no exact match, which is out of order.
        BSTA,UN CMP_TMP_PE
        BCTR,EQ TSL_WRITE                ; TMP == PE exactly: legal append
        BCTA,UN TSL_ERR
TSL_MATCH:
        ; Exact match at TMP. Legal only if it's the LAST stored line:
        ; save its start, advance a check past it, compare to PE.
        LODA,R0 TMPH
        STRA,R0 CURH
        LODA,R0 TMPL
        STRA,R0 CURL
        ZBSR *VINC_TMP  
        ZBSR *VINC_TMP  
TSL_MAS:
        LODA,R0 *TMPH
        COMI,R0 CR
        BCTR,EQ TSL_MADONE
        ZBSR *VINC_TMP  
        BCTR,UN TSL_MAS
TSL_MADONE:
        ZBSR *VINC_TMP                    ; skip the CR itself
        BSTA,UN CMP_TMP_PE
        BCTR,EQ TSL_EXCISE
        BCTA,UN TSL_ERR
TSL_EXCISE:
        ; It's the last line: truncate the store back to where it started -
        ; nothing after it, so no shifting needed.
        LODA,R0 CURH
        STRA,R0 PEH
        STRA,R0 TMPH
        LODA,R0 CURL
        STRA,R0 PEL
        STRA,R0 TMPL
TSL_WRITE:
        ZBSR *VWSKIP  
        LODA,R0 *IPH                      ; LODA sets CC from the loaded value (Load/
        BCTR,EQ TSL_DONE                  ; Arithmetic class) - EQ means NUL (empty
                                          ; body): delete-only (or no-op append).
                                          ; IBUF is NUL-terminated, not CR-terminated
                                          ; like pBASIC65c02's - no CR appears in it.
        LODA,R0 LNUMH
        STRA,R0 *TMPH                     ; write line hi
        ZBSR *VINC_TMP  
        LODA,R0 LNUML
        STRA,R0 *TMPH                     ; write line lo
        ZBSR *VINC_TMP  
TSL_CPY:
        LODA,R0 *IPH
        BCTR,EQ TSL_CPYDONE                ; NUL: end of typed body
        STRA,R0 *TMPH                     ; copy one body byte
        ZBSR *VINC_TMP  
        ZBSR *VINC_IP  
        BCTR,UN TSL_CPY
TSL_CPYDONE:
        LODI,R0 CR                        ; manufacture the CR terminator ourselves -
        STRA,R0 *TMPH                     ; the stored record format needs one, but
        ZBSR *VINC_TMP                     ; IBUF (NUL-terminated) never contains one
        LODA,R0 TMPH
        STRA,R0 PEH
        LODA,R0 TMPL
        STRA,R0 PEL
TSL_DONE:
        LODI,R0 1                        ; CC=GT: line stored/deleted
        RETC,UN
TSL_ERR:
        ZBRR *VJSYNERR                    ; v0.6: was BCTA,UN JSYNERR (3B); vector already exists

; =============================================================================
;  INC16_TMP_TO_EXP -- EXPH:EXPL = TMPH:TMPL + 1
; Factored out of two identical inlined copies (FIND_LINE's FL_CHKLO,
; FIND_INS's FI_CHK hi-equal path) found via the v0.8 duplicate-byte-
; sequence scan - both need "the byte right after TMP" (the record's
; lo line-number byte lives at TMP+1) and computed it inline.
; In:  TMPH:TMPL
; Out: EXPH:EXPL = TMPH:TMPL + 1
; Clobbers: R0
INC16_TMP_TO_EXP:
        LODA,R0 TMPL
        ADDI,R0 1
        STRA,R0 EXPL
        LODA,R0 TMPH
        TPSL $01
        BCTR,LT I16TE_NC
        ADDI,R0 1
I16TE_NC:
        STRA,R0 EXPH
        RETC,UN

; =============================================================================
;  FIND_LINE -- Search for line LNUMH:LNUML in program store
; Out: TMPH:TMPL = record start if found; CC=EQ found, CC=GT not found.
; Clobbers: R0, TMPH, TMPL, EXPH, EXPL
FIND_LINE:
        BSTR,UN FIND_INS                 ; [+1]
        ; check if at end of program
        BSTA,UN CMP_TMP_PE
        BCTR,LT FL_CHK
        BCTR,UN FL_RET_NF
FL_CHK:
        LODA,R0 *TMPH
        SUBA,R0 LNUMH
        BCTR,EQ FL_CHKLO
FL_RET_NF:
        LODI,R0 1                        ; CC=GT: not found
        RETC,UN

FL_CHKLO:
        BSTR,UN INC16_TMP_TO_EXP
        LODA,R0 *EXPH
        SUBA,R0 LNUML
        BCTR,EQ FL_FOUND
        BCTR,UN FL_RET_NF
FL_FOUND:
        EORZ,R0                          ; CC=EQ: found
        RETC,UN

; =============================================================================
;  FIND_INS -- Find sorted insertion point for LNUMH:LNUML
; Returns TMPH:TMPL = address of first record with line >= LNUMH:LNUML,
; or PEH:PEL if all lines are smaller.
; In:  LNUMH:LNUML = target line number
; Out: TMPH:TMPL = insertion point
; Clobbers: R0, TMPH, TMPL, EXPH, EXPL
FIND_INS:
        ZBSR *VSET_TMP_PROG
FI_LP:
        BSTA,UN CMP_TMP_PE
        RETC,GT
        RETC,EQ
FI_CHK:
        LODA,R0 LNUMH
        SUBA,R0 *TMPH                    ; LNUMH - stored.hi
        BCTR,GT FI_ADV
        BCTR,LT FI_RET
        ; hi bytes equal: check lo
        BSTA,UN INC16_TMP_TO_EXP
        LODA,R0 LNUML
        PPSL $02                         ; unsigned compare mode
        COMA,R0 *EXPH
        CPSL $02
        BCTR,GT FI_ADV
FI_RET:
        RETC,UN
FI_ADV:
        ; advance TMPH:TMPL past record: skip hi+lo then scan body until CR
        ZBSR *VINC_TMP
        ZBSR *VINC_TMP
FI_AS:
        LODA,R0 *TMPH
        COMI,R0 CR
        BCTR,EQ FI_DONE
        ZBSR *VINC_TMP  
        BCTR,UN FI_AS
FI_DONE:
        ZBSR *VINC_TMP                    ; skip the CR itself
        BCTR,UN FI_LP

; =============================================================================
;  EXPR -- Flat-precedence expression evaluator (v0.5, replaces the
;  multi-tier PARSE_EXPR/EAM_ATOM/EAM_HI/EAM_LO_LOOP + SW-stack return
;  trampolining/PUSH_RET/PARSER_RET entirely). One left-to-right pass -
;  all six operators (+-*/=<) sit at the same precedence, matching
;  pBASIC65c02's EXPR/EXPR_LOOP exactly. "1+2*3" evaluates as "(1+2)*3" -
;  deliberate, not a bug.
; In:  IPH:IPL -> expression string
; Out: EXPH:EXPL = 16-bit result (relops produce 0=false, 1=true)
; Clobbers: R0, R1, R3, SAVEH, SAVEL, NEGFLG, SC0, TMPH, TMPL
; Reading A (see chat): return addresses now use real HW calls, no
; SW-stack trampolining - only genuine recursion (parens, unary +/-,
; chained atoms) costs real RAS depth, guarded below. EXPR_ATOM is one
; shared routine for both the first atom and every later right-operand
; atom (smallest code); the actual safe nesting depth this yields is
; being measured empirically against the project's declared 6-of-8
; effective RAS budget (2 reserved for CHIN/COUT's own internal delay
; calls) rather than asserted from manual counting - PE_RAS_LIMIT (defined
; near RAS_DEPTH, top of file) is a placeholder pending that measurement.
EXPR:
        LODI,R3 $FF                      ; SW operand-stack empty sentinel -
                                          ; ONLY safe here, at the genuine
                                          ; top-level entry (never reset on
                                          ; the recursive path - EA_PAREN
                                          ; enters at EXPR_GUARDED instead,
                                          ; preserving R3 across the paren
                                          ; so an outer pending operand
                                          ; can't be overwritten by an
                                          ; inner one - PAREN-NEST-01 again
                                          ; otherwise).
EXPR_GUARDED:
        SPSU                             ; R0 = PSU; SP in bits 2:0
        ANDI,R0 $07
        COMI,R0 PE_RAS_LIMIT
        BCTR,LT EXPR_OK
        LODI,R0 ERR_NEST
        ZBRR *VDO_ERROR
EXPR_OK:
        BSTA,UN EXPR_ATOM
EXPR_LOOP:
        ZBSR *VWSKIP  
        LODA,R0 *IPH
        STRA,R0 SC0                      ; SC0 = char to match against operators
        LODI,R0 <TOK_CHARS
        STRA,R0 TMPH
        LODI,R0 >TOK_CHARS
        STRA,R0 TMPL
        LODI,R1 6                        ; bound: exactly the 6 operator rows -
                                          ; unlike statement dispatch, this scan
                                          ; has no letter-gate to stop it from
                                          ; wrongly matching a following
                                          ; statement's keyword letter
OPS_LP:
        LODA,R0 *TMPH
        SUBA,R0 SC0
        BCTR,EQ OPS_HIT
        ZBSR *VINC_TMP  
        ZBSR *VINC_TMP  
        ZBSR *VINC_TMP  
        BDRR,R1 OPS_LP
        RETC,UN                          ; no operator: EXPH:EXPL is the final result
OPS_HIT:
        ; Extract the matched handler's address FIRST, while TMP still
        ; points at the right table row - EXPR_ATOM's own descent into
        ; PARSE_FACTOR->PARSE_S16->PARSE_U16 clobbers TMPH:TMPL for its
        ; own digit-accumulation scratch (EXP16_TO_TMP), so reading the
        ; table through TMP *after* parsing the right operand silently
        ; picks up garbage instead of the handler address - confirmed via
        ; simulator trace: it jumped to $2C09 (well past ROMEND, all
        ; zero bytes) instead of DO_ADD at $058A.
        ZBSR *VINC_TMP  
        LODA,R0 *TMPH
        STRA,R0 GOTOH                    ; matched operator's handler, hi
        ZBSR *VINC_TMP  
        LODA,R0 *TMPH
        STRA,R0 GOTOL                    ; handler lo
        ; PAREN-NEST-01-style protection: push onto SWBASE (LIFO), not a
        ; flat cell - a same-precedence op one recursion level deeper
        ; (e.g. the "+" inside "2*(3+4)") would otherwise clobber it.
        LODA,R0 EXPL
        STRA,R0 SWBASE,R3+
        LODA,R0 EXPH
        STRA,R0 SWBASE,R3+
        ; PAREN-NEST-02 fix: GOTOH:GOTOL themselves are a flat pair, not
        ; part of the LIFO above - a parenthesized right operand (e.g.
        ; the "(B*B/64)" in "(A*A/64)-(B*B/64)") recurses through
        ; EXPR_ATOM->EA_PAREN->EXPR_GUARDED, which hits this same OPS_HIT
        ; for its OWN "*"/"/" operators and overwrites GOTOH:GOTOL before
        ; we get back here to use them. Confirmed via simulator: with
        ; A=-144,B=-64, "(A*A/64)-(B*B/64)" produced 5 (i.e. 324/64, the
        ; INNER divide's handler) instead of 260 (324-64) - the outer "-"
        ; was silently replaced by the inner paren's last operator. Fix:
        ; push GOTOH:GOTOL onto the same SWBASE stack too, so a clobber
        ; during the recursive call is popped back off, not read stale.
        LODA,R0 GOTOH
        STRA,R0 SWBASE,R3+
        LODA,R0 GOTOL
        STRA,R0 SWBASE,R3+
        ZBSR *VINC_IP                    ; consume the operator char
        BSTR,UN EXPR_ATOM                ; parse the right operand
        LODA,R0 SWBASE,R3                ; handler lo (top of stack)
        STRA,R0 GOTOL
        LODA,R0 SWBASE,R3-               ; handler hi; R3 now points at
        STRA,R0 GOTOH                    ; the pushed left operand's EXPH
        SUBI,R3 1                        ; realign R3 for the DO_xxx pop
        BCTA,UN *GOTOH                   ; jump to handler (pops SWBASE,
                                          ; combines, jumps back to EXPR_LOOP)

; =============================================================================
;  EXPR_ATOM -- parse one atom: unary +/-, parens, or a literal/variable.
; In:  IPH:IPL -> atom
; Out: EXPH:EXPL = value
; Clobbers: R0
EXPR_ATOM:
        ZBSR *VWSKIP  
        LODA,R0 *IPH
        COMI,R0 A'-'
        BCTR,EQ EA_NEG
        COMI,R0 A'+'
        BCTR,EQ EA_POS
        COMI,R0 A'('
        BCTR,EQ EA_PAREN
        BCTA,UN PARSE_FACTOR              ; tail call
EA_NEG:
        ZBSR *VINC_IP  
        BSTR,UN EXPR_ATOM                 ; real recursive call (operand)
        BCTA,UN NEG_EXP_BODY              ; tail call: negate, return
EA_POS:
        ZBSR *VINC_IP  
        BCTR,UN EXPR_ATOM                 ; tail call: '+' is a no-op
EA_PAREN:
        ZBSR *VINC_IP  
        BSTA,UN EXPR_GUARDED              ; real recursive call (sub-expr);
                                          ; NOT "EXPR" - must not reset R3
        ZBSR *VWSKIP  
        ZBSR *VINC_IP                     ; consume ')'
        RETC,UN

; =============================================================================
;  DO_ADD / DO_SUB / DO_MUL / DO_DIV / DO_EQOP / DO_LTOP -- flat operator
;  handlers. Each pops the left operand pushed by OPS_HIT, combines with
;  EXPH:EXPL (the just-parsed right operand), and jumps back to
;  EXPR_LOOP to look for more operators at the same precedence.
; In:  EXPH:EXPL = right operand; SWBASE top = pushed left operand (lo,hi)
; Out: EXPH:EXPL = combined result; control resumes at EXPR_LOOP
; Clobbers: R0, R3 (popped by 2), plus per-operator (see each)
DO_SUB:
        BSTA,UN NEG_EXP_BODY              ; EXP = -EXP
DO_ADD:
        BCTR,UN ADD16_SAVE_EXP

; =============================================================================
;  ADD16_SAVE_EXP -- EXP = SAVE + EXP (16-bit, WC carry chain); resumes
;  EXPR_LOOP. Left operand is popped off SWBASE (pushed by OPS_HIT above)
;  into SAVEH:SAVEL just before use - a flat cell could have been
;  clobbered by a nested same-precedence op in the meantime.
; In:  EXPH:EXPL = right operand; SWBASE top = pushed left operand (lo,hi)
; Out: EXPH:EXPL = left + EXPH:EXPL; tail-jumps into EXPR_LOOP
; Clobbers: R0, R3 (popped by 2)
ADD16_SAVE_EXP:
        LODA,R0 SWBASE,R3                ; left hi (top, no dec)
        STRA,R0 SAVEH
        LODA,R0 SWBASE,R3-               ; left lo, then R3--
        STRA,R0 SAVEL
        SUBI,R3 1                        ; drop the hi slot too
        CPSL PSW_WC
        LODA,R0 SAVEL
        ADDA,R0 EXPL
        STRA,R0 EXPL
        PPSL PSW_WC
        LODA,R0 SAVEH
        ADDA,R0 EXPH
        STRA,R0 EXPH
        CPSL PSW_WC
        BCTA,UN EXPR_LOOP

DO_MUL:
        BSTR,UN POP_SAVE_TO_TMP           ; TMPH:TMPL = popped left operand
        BSTA,UN MUL16
        BCTA,UN EXPR_LOOP
DO_DIV:
        BSTR,UN POP_SAVE_TO_TMP
        BSTA,UN DIV16
        BCTA,UN EXPR_LOOP

; =============================================================================
;  POP_SAVE_TO_TMP -- pop a 2-byte value pushed on SWBASE into TMPH:TMPL
; In:  R3 = SW stack pointer; top (R3)=hi, (R3-1)=lo of pushed left operand
; Out: TMPH:TMPL = popped value; R3 -= 2
; Clobbers: R0
POP_SAVE_TO_TMP:
        LODA,R0 SWBASE,R3
        STRA,R0 TMPH
        LODA,R0 SWBASE,R3-
        STRA,R0 TMPL
        SUBI,R3 1
        RETC,UN

; =============================================================================
;  DO_EQOP / DO_LTOP -- relop handlers (v0.5, folded into the flat table -
;  see PORT HISTORY). Pop left, compare against right (EXPH:EXPL), leave
;  0 (false) or 1 (true) in EXPH:EXPL.
DO_EQOP:
        BSTR,UN POP_SAVE_TO_TMP           ; TMPH:TMPL = left
        LODA,R0 TMPH
        SUBA,R0 EXPH
        STRA,R0 SC0
        LODA,R0 TMPL
        SUBA,R0 EXPL
        LODA,R1 SC0
        IORZ R1
        BCTR,EQ DOP_TRUE
        BCTR,UN DOP_FALSE
DO_LTOP:
        BSTR,UN POP_SAVE_TO_TMP           ; TMPH:TMPL = left
        LODA,R0 TMPH
        EORI,R0 $80
        STRA,R0 SC0
        LODA,R0 EXPH
        EORI,R0 $80
        SUBA,R0 SC0                       ; biased(right.hi) - biased(left.hi)
        BCTR,GT DOP_TRUE                  ; right.hi > left.hi -> left<right
        BCTR,LT DOP_FALSE
        LODA,R0 EXPL
        SUBA,R0 TMPL                      ; right.lo - left.lo (hi bytes equal)
        BCTR,GT DOP_TRUE
DOP_FALSE:
        EORZ,R0
        db $EC                            ; COMA,R0 -- consume next 2 bytes
DOP_TRUE:
        LODI,R0 $FF
        STRA,R0 EXPH
        STRA,R0 EXPL
        BCTA,UN EXPR_LOOP

; =============================================================================
;  PARSE_FACTOR -- Parse a single value (variable or literal)
; In:  IPH:IPL -> first char of factor
; Out: EXPH:EXPL = value
; Clobbers: R0, R1, SC0
; Note: UPCASE inlined to avoid consuming an extra RAS slot.
PARSE_FACTOR:
        LODA,R0 *IPH
        ; inline UPCASE: if 'a'..'z' subtract 32
        COMI,R0 A'a'
        BCTR,LT PF_UC_DONE
        COMI,R0 A'z'+1
        BCTR,GT PF_UC_DONE
        SUBI,R0 32
PF_UC_DONE:
        COMI,R0 A'A'
        BCTR,LT PF_NUM
        COMI,R0 A'Z'+1
        BCTR,LT PF_LOADVAR
PF_NUM:
        BCTR,UN PARSE_S16                ; tail call: PARSE_S16's RETC,UN returns to our caller

; =============================================================================
;  PF_LOADVAR -- Load variable value from VARS
; In:  R0 = uppercase variable letter A-Z; IP -> that char
; Out: EXPH:EXPL = variable value
; Clobbers: R0, R1, SC0
PF_LOADVAR:
        STRA,R0 SC0
        ZBSR *VINC_IP  
        LODA,R0 SC0
        SUBI,R0 A'A'
        STRZ,R1                          ; R1 = index (0..25)
        ADDZ,R1                          ; R0 = index*2
        STRZ,R1                          ; R1 = index*2
        LODA,R0 VARS,R1                  ; hi byte
        STRA,R0 EXPH
        LODA,R0 VARS+1,R1               ; lo byte
        STRA,R0 EXPL
        RETC,UN

PRO_NONE:
        ZBRR *VJSYNERR 

; =============================================================================
;  PARSE_S16 -- Parse signed decimal integer
; In:  IPH:IPL -> first char (optional '-' then digits)
; Out: EXPH:EXPL = signed 16-bit value
; Clobbers: R0, NEGFLG, EXPH, EXPL
PARSE_S16:
        EORZ,R0
        STRA,R0 NEGFLG
        LODA,R0 *IPH
        COMI,R0 A'-'
        BCTR,EQ PS16_NEG
        BCTR,UN PS16_UN
PS16_NEG:
        ZBSR *VINC_IP  
        LODI,R0 1
        STRA,R0 NEGFLG
PS16_UN:

; =============================================================================
;  PARSE_U16 -- Parse unsigned decimal digits -> EXPH:EXPL
; Jumps to JSYNERR if no digits found.
; In:  IPH:IPL -> first digit char
; Out: EXPH:EXPL = value
; Clobbers: R0, R3, SC0, EXPH, EXPL, TMPH, TMPL (R3SAVE used to preserve R3)
;PARSE_U16:
        ZBSR *VCLR_EXP
        LODA,R0 *IPH
        COMI,R0 A'0'
        BCTR,LT PRO_NONE; surrogate for JSYNERR
        COMI,R0 A'9'+1
        BCTR,GT PRO_NONE; surrogate for JSYNERR
PU16_LP:
        LODA,R0 *IPH
        COMI,R0 A'0'
        BCTR,LT PU16_RET ; RETC,LT
        COMI,R0 A'9'+1
        BCTR,LT PU16_DIG
        BCTR,UN PU16_RET ; RETC,UN
PU16_DIG:
        SUBI,R0 A'0'
        STRA,R0 SC0
        ZBSR *VINC_IP
PU16_DNC:
        STRA,R3 R3SAVE                   ; save SW stack pointer
        BSTA,UN EXP16_TO_TMP
        ZBSR *VCLR_EXP
        LODI,R3 10
PU16_M10:
        LODA,R0 EXPL
        ADDA,R0 TMPL
        BSTA,UN CARRY_INTO_EXPH
        LODA,R0 EXPH
        ADDA,R0 TMPH
        STRA,R0 EXPH
        BDRR,R3 PU16_M10
        LODA,R3 R3SAVE                   ; restore SW stack pointer
        LODA,R0 EXPL
        ADDA,R0 SC0
        BSTA,UN CARRY_INTO_EXPH
        BCTR,UN PU16_LP

PU16_RET:
        ; drop through

; =============================================================================
;  NEG_EXP -- Negate EXPH:EXPL if NEGFLG set
;  NEG_EXP_BODY -- Unconditional negate EXPH:EXPL
; In:  EXPH:EXPL = value; NEGFLG = flag
; Out: EXPH:EXPL negated (two's complement) if NEGFLG!=0
; Clobbers: R0, R1
NEG_EXP:
        LODA,R0 NEGFLG
        RETC,EQ
NEG_EXP_BODY:
        LODI,R1 EXPH-IPH                 ; EXPH offset from IPH (= 4); R1 variant for NEG_SHARED
        BCTR,UN NEG_SHARED

; =============================================================================
;  ABS_TMP -- Absolute value of TMPH:TMPL; set NEGFLG=1 if was negative
; In:  TMPH:TMPL = signed value; NEGFLG cleared by caller
; Out: TMPH:TMPL = |value|; NEGFLG=1 if was negative
; Clobbers: R0, R1
ABS_TMP:
        LODA,R0 TMPH
        ANDI,R0 $80
        RETC,EQ
        LODI,R0 1
        STRA,R0 NEGFLG
        LODI,R1 TMPH-IPH                 ; TMPH offset from IPH (= 2); R1 variant for NEG_SHARED
        ; fall through to NEG_SHARED

; =============================================================================
;  NEG_SHARED -- Shared negation core (two's complement via 1s complement + INC_ET)
; In:  R1 = offset (EXPH-IPH for EXP, TMPH-IPH for TMP)
; Out: value at IPH+R1:IPL+R1 negated
; Clobbers: R0
NEG_SHARED:
        LODA,R0 IPH,R1
        EORI,R0 $FF
        STRA,R0 IPH,R1
        LODA,R0 IPL,R1
        EORI,R0 $FF
        STRA,R0 IPL,R1
        LODZ R1
        BCTA,UN INC_ET                   ; tail call: adds 1 (INC_ET uses alt bank R1)

; =============================================================================
;  ABS_EXP -- Absolute value of EXPH:EXPL; toggle NEGFLG if was negative
; In:  EXPH:EXPL = signed value; NEGFLG = current flag
; Out: EXPH:EXPL = |value|; NEGFLG toggled if was negative
; Clobbers: R0, R1
ABS_EXP:
        LODA,R0 EXPH
        ANDI,R0 $80
        RETC,EQ
        LODA,R0 NEGFLG
        EORI,R0 $01
        STRA,R0 NEGFLG
        LODI,R1 EXPH-IPH
        BCTR,UN NEG_SHARED


; =============================================================================
;  DO_LIST -- Print stored BASIC lines (v0.1b: whole program only - the
;  range-filtered LIST start,end and DO_POKE both used to share this block
;  via PARSE_2ARGS; both are cut now, so the whole shared mechanism
;  (PARSE_2ARGS/P2A_*/ARGAH/ARGAL/FUNCOP, plus the AND/OR/XOR bodies left
;  over as dead code from Stage 1a) goes in one sweep. What's left is just
;  the print loop, unconditionally from PROG to program end.)
; Syntax: LIST
; In:  PROG=program base, PEH:PEL=program end
; Out: whole program printed
; Clobbers: R0, IPH, IPL, TMPH, TMPL, EXPH, EXPL
DO_LIST:
        ZBSR *VSET_TMP_PROG
DLS_LP:
        ; Check TMP against program end
        BSTA,UN CMP_TMP_PE
        RETC,GT
        RETC,EQ
DLS_BODY:
        ; Copy TMP -> IP, read+print line number, then rest of line verbatim
        LODA,R0 TMPH
        STRA,R0 IPH
        LODA,R0 TMPL
        STRA,R0 IPL
        LODA,R0 *IPH
        STRA,R0 EXPH
        ZBSR *VINC_IP                     ; advance past line hi byte
        LODA,R0 *IPH
        STRA,R0 EXPL
        ZBSR *VINC_IP                     ; advance past line lo byte
        BSTA,UN PRINT_S16
        ZBSR *VPRT_SPACE
DLS_BLPX:
        LODA,R0 *IPH
        COMI,R0 CR
        BCTR,EQ DLS_NL
        ZBSR *VCOUT
        ZBSR *VINC_IP
        BCTR,UN DLS_BLPX
DLS_NL:
        ZBSR *VINC_IP                     ; skip over CR
        BSTA,UN PRT_CRLF
        ; Update TMP from IP for next iteration (v0.6: IP_TO_TMP inlined)
        LODA,R0 IPH
        STRA,R0 TMPH
        LODA,R0 IPL
        STRA,R0 TMPL
        BCTA,UN DLS_LP

; =============================================================================
;  SETUP_MULDIV -- Common preamble for MUL16 and DIV16
; Clears NEGFLG, takes absolute values of TMP and EXP (toggling NEGFLG for
; each negative operand), then saves |EXP| in SC0:SC1 and clears EXP to zero
; ready for the multiply/divide accumulation loop.
; In:  TMPH:TMPL = left operand; EXPH:EXPL = right operand
; Out: NEGFLG = result sign (0=positive, 1=negative); SC0:SC1 = |EXP|; EXP = 0
; Clobbers: R0, R1, NEGFLG, SC0, SC1, TMPH, TMPL, EXPH, EXPL
; RAS: called at depth 6 (MUL16/DIV16 call sites); max depth inside = 8 (at limit).
;   ABS_TMP/ABS_EXP use only BCTR/BCTA internally -- no further RAS consumption.
SETUP_MULDIV:
        EORZ,R0
        STRA,R0 NEGFLG
        BSTA,UN ABS_TMP                  ; [+1] sets NEGFLG=1 if TMP was negative
        BSTA,UN ABS_EXP                  ; [+1] toggles NEGFLG if EXP was negative
        LODI,R0 SC0-IPH                 ; offset to SCO and 1, SC1 = |EXP| lo
        BSTA,UN EXP16_TO_ET             ; SC0 = |EXP| hi
        ZBSR *VCLR_EXP                  ; clear EXP (accumulator starts at 0)
        RETC,UN

; =============================================================================
;  CARRY_INTO_EXPH -- Store R0 into EXPL, propagating carry into EXPH.
; Factored out of three identical inlined copies (PU16_M10, PU16's final-
; digit add, MUL16's MU_ADD) found via a duplicate-byte-sequence scan -
; same v0.8 CMP_TMP_PE technique. Caller does the 8-bit ADDA into R0
; itself (the addend differs per site); this shared tail just stores the
; result and carries into EXPH, exactly as each site did inline.
; In:  R0 = EXPL + <addend>, CC set by that ADDA
; Out: EXPL = R0; EXPH += 1 iff the ADDA carried
; Clobbers: R0
CARRY_INTO_EXPH:
        STRA,R0 EXPL
        TPSL $01
        BCTR,LT CIE_NC
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
CIE_NC:
        RETC,UN

; =============================================================================
;  MUL16 -- Signed 16-bit multiply: TMPH:TMPL * EXPH:EXPL -> EXPH:EXPL
; In:  TMPH:TMPL = left operand; EXPH:EXPL = right operand
; Out: EXPH:EXPL = product (16-bit two's complement wrap)
; Clobbers: R0, NEGFLG, SC0, SC1, TMPH, TMPL
MUL16:
        BSTR,UN SETUP_MULDIV             ; [+1] sign setup, |EXP|->SC0:SC1, EXP=0
MU_LP:
        LODA,R0 TMPH
        BCFR,EQ MU_ADD  ; v0.11 fix: was BCTR,GT MU_ADD - see KNOWN BUGS/
                        ; PORT HISTORY. TMPH here is an UNSIGNED magnitude
                        ; byte (post-ABS_TMP), so "is it zero" must ignore
                        ; the sign bit; BCTR,GT wrongly excluded TMPH in
                        ; $80-$FF (bit 7 set, so CC=LT even though nonzero)
                        ; from continuing the loop. BCFR,EQ branches on
                        ; "not exactly zero" instead, covering GT and LT
                        ; alike, same 2 bytes either way.
        LODA,R0 TMPL
        BCTR,EQ MU_DONE
MU_ADD:
        LODA,R0 EXPL
        ADDA,R0 SC1
        BSTR,UN CARRY_INTO_EXPH
        LODA,R0 EXPH
        ADDA,R0 SC0
        STRA,R0 EXPH
        LODA,R0 TMPL
        SUBI,R0 1
        STRA,R0 TMPL
        TPSL $01                         ; C=1 no borrow(EQ) / C=0 borrow(LT) -- v0.10 fix, was BCFR,LT off raw CC (see KNOWN BUGS/PORT HISTORY)
        BCTR,EQ MU_TNB
        LODA,R0 TMPH
        SUBI,R0 1
        STRA,R0 TMPH
MU_TNB:
        BCTR,UN MU_LP
MU_DONE:
        BSTA,UN NEG_EXP
        EORZ,R0
        STRA,R0 NEGFLG
        RETC,UN

; =============================================================================
;  DIV16 -- Signed 16-bit divide: TMPH:TMPL / EXPH:EXPL -> EXPH:EXPL
; Remainder left in TMPH:TMPL.
; In:  TMPH:TMPL = dividend; EXPH:EXPL = divisor
; Out: EXPH:EXPL = quotient; TMPH:TMPL = remainder
; Clobbers: R0, NEGFLG, SC0, SC1
; Error: divisor=0 -> ERR_DIV_ZERO
DIV16:
        LODA,R0 EXPH
        BCFR,EQ DV_NZ   ; check for zero
        LODA,R0 EXPL
        BCTA,EQ JERRDIVZER
        
        ; not zero
DV_NZ:
        BSTA,UN SETUP_MULDIV             ; [+1] sign setup, |EXP|->SC0:SC1, EXP=0
DV_LP:
        LODA,R0 TMPH
        PPSL $02                 ; v0.11 fix: unsigned compare mode (see
                                  ; CC SEMANTICS header note) - TMPH here
                                  ; is an unsigned magnitude byte (post-
                                  ; ABS_TMP), and a raw signed SUBA CC
                                  ; mishandles it in $80-$FF (same bug
                                  ; class as MU_LP, e.g. TMPH=$80 for a
                                  ; dividend of exactly -32768 read as
                                  ; "already less than the divisor" and
                                  ; exiting the loop before it starts -
                                  ; PRINT -32768/1 gave 0, not -32768).
                                  ; NOTE: the COM mode bit only affects
                                  ; COMA/COMI/COMR/COMZ, NOT SUBA/ADDA (the
                                  ; oracle's own explicit pitfall warning) -
                                  ; the first attempt at this fix used
                                  ; SUBA and, confirmed by testing, changed
                                  ; nothing. COMA is the non-destructive
                                  ; compare that COM mode actually govern;
                                  ; it doesn't store a result, so R0 is
                                  ; still TMPH afterward - fine here, since
                                  ; nothing below depends on the old SUBA
                                  ; difference, only on the branch taken.
        COMA,R0 SC0
        CPSL $02                 ; restore normal (signed) mode
        BCTR,LT MU_DONE ; DV_DONE
        BCTR,GT DV_SUB
        LODA,R0 TMPL
        SUBA,R0 SC1
        TPSL $01
        BCTR,EQ DV_SUB
        BCTR,UN MU_DONE ; DV_DONE
DV_SUB:
        LODA,R0 TMPL
        SUBA,R0 SC1
        STRA,R0 TMPL
        TPSL $01
        BCTR,EQ DV_SNB
        LODA,R0 TMPH
        SUBI,R0 1
        STRA,R0 TMPH
DV_SNB:
        LODA,R0 TMPH
        SUBA,R0 SC0
        STRA,R0 TMPH
        BSTA,UN INC_EXP                   ; v0.6: was ZBSR *VINC_EXP (sole caller now direct)
        BCTR,UN DV_LP

JERRDIVZER:
        LODI,R0 ERR_DIV_ZERO
        ZBRR *VDO_ERROR 

; =============================================================================
;  PRINT_S16 -- Print signed 16-bit value EXPH:EXPL as decimal
; In:  EXPH:EXPL = signed value
; Out: decimal digits written to COUT
; Clobbers: R0, R1, R3, TMPL (R2 saved/restored internally - see v0.9 note)
; v0.9 fix: R2 is documented above (register-conventions note) as a long-
; lived variable-letter register, "never written by subroutines except
; DL_STORE, SE_NOTKW" - the old PREC-based implementation respected this
; (its own Clobbers list never included R2); this loop uses R2 as its P10
; column index and, unlike R3 (which the old implementation also freely
; clobbered), must save and restore it. Confirmed via testing: without
; this, a second PRINT statement's own keyword dispatch (MD_HIT, wholly
; unrelated to printing) read garbage into GOTOH:GOTOL and jumped into
; uninitialized memory - R2 was live across the statement boundary for a
; reason unrelated to printing at all, and clobbering it broke whatever
; used it next, wherever that happened to be. Saved in SC1 (old
; implementation's own Clobbers list already included SC1 freely, so nothing
; else can be relying on it surviving a PRINT_S16 call).
PRINT_S16:
        LODZ,R2                  ; save caller's R2 (see note above)
        STRA,R0 SC1
        STRA,R3 SC0               ; v0.9 fix (round 2): also save/restore R3
                                  ; - clobbering it left keyword dispatch for
                                  ; the NEXT statement broken (fell through
                                  ; to variable-assignment parsing instead of
                                  ; recognizing PRINT), confirmed via testing
                                  ; two prints in a row; R2 alone wasn't
                                  ; sufficient. The old implementation's own
                                  ; Clobbers list did say R3, but evidently
                                  ; something downstream relies on whatever
                                  ; specific state it happened to leave R3 in.
        LODA,R0 EXPH             ; get high byte & establish CC
        BCTR,LT IS_NEG           ; branch if negative (bit 7 set)

        IORA,R0 EXPL             ; Check for ZERO
        BCFR,EQ PS_DIGITS       ; >0, flow into subtract printer

        LODA,R0 SC1               ; restore R2/R3 before this tail-call exit -
        STRZ,R2                  ; execution never returns here afterward
        LODA,R3 SC0
        LODI,R0 A'0'             ; Handle Zero
        ZBRR *VCOUT              ; Print '0' and tail call return
IS_NEG:
        LODI,R0 A'-'
        ZBSR *VCOUT
        BSTA,UN NEG_EXP_BODY     ; Negate, making EXPH:EXPL positive
;   RAS safety (the reason PREC used SW-stack recursion in the first
; place): DO_ERROR calls PRINT_S16 to print "@line" WITHOUT unwinding the
; hardware RAS back to REPL first (errors are delivered via ZBRR, a plain
; branch, from wherever in the call chain they're detected - e.g. mid
; MUL16/DIV16, already documented elsewhere as reaching hardware RAS depth
; 6-8), so anything PRINT_S16 does is stacked on top of whatever depth was
; already live. This loop adds only ONE call level of its own regardless
; of digit count (BSTA,UN NEG_EXP_BODY for the sign, ZBSR *VCOUT per
; character) - NEG_EXP_BODY's own chain (NEG_SHARED->INC_ET) is documented
; leaf-only ("NO BSTA inside"), and VCOUT's COUT->DLAY chain is the same
; leaf-bottoming chain every character output already goes through, old
; implementation included - so this is no deeper than before. Verified
; empirically, not just by inspection: a divide-by-zero nested inside the
; one level of paren nesting the guard allows, from a RUN'd line (the
; realistic worst case - deeper paren nesting than that already hits
; ERR_NEST before reaching this code at all), still correctly prints
; "?2@10" with this replacement, matching the pre-change behavior exactly.
;   Also drops the old implementation's explicit -32768 special case
; (negating $8000 two's-complement-overflows back to $8000, which the old
; PREC's own algorithm couldn't handle, hence the MSG_MIN literal-string
; workaround) - this loop doesn't need it: it decomposes the raw 16-bit
; bit pattern via unsigned repeated subtraction regardless of how it was
; produced, and $8000 as an unsigned magnitude IS 32768, the correct
; digits for -32768's magnitude. Verified: PRINT -32768 (via 0-32768)
; still prints "-32768" correctly.
PS_DIGITS:
        LODI,R2 0                ; R2 = P10 table index (0 to 4)
        LODI,R3 0                ; R3 = leading zero flag (0 = leading, >0 = printing)
DIGIT_LOOP:
        LODI,R1 A'0'-1           ; R1 = ASCII digit character
SUB_LOOP:
        ; v0.9 fix: CPSL $08 must run before ADDI,R1 1, not after it. The
        ; oracle's own canonical WC idiom clears WC both before AND after
        ; a multi-byte with-carry op ("CPSL $08 to restore standard 8-bit
        ; math mode") - this loop only cleared it before the low-byte sub,
        ; never after the high-byte sub's PPSL $08. So on every loop-back
        ; (and on entry to a fresh digit column after a borrow), ADDI,R1 1
        ; ran with WC still enabled from the previous pass's high-byte
        ; subtract - and per the oracle, WC=1 makes ADDI inject the
        ; leftover carry too, silently adding +2 instead of +1 whenever
        ; the previous pass's high-byte subtraction hadn't borrowed (the
        ; normal case). Confirmed via instruction trace: R1 was observed
        ; jumping 0x30->0x32 (skipping the digit '1' entirely) on exactly
        ; this transition. Moving CPSL $08 here, before ADDI, guarantees
        ; WC is clear on every entry to this loop regardless of path.
        CPSL $08                 ; Clear WC bit for standard 8-bit math
        ADDI,R1 1                ; Increment digit
        LODA,R0 EXPL
        SUBA,R0 P10_LO,R2        ; Subtract low byte
        STRA,R0 TMPL             ; Save tentatively
        PPSL $08                 ; Set WC bit (enables Carry-In/Borrow)
        LODA,R0 EXPH
        SUBA,R0 P10_HI,R2        ; Subtract high byte
        BCTR,LT BORROW           ; If borrow occurred (C=0), result is LT. Stop subtracting.

        ; No borrow: commit result and subtract again
        STRA,R0 EXPH
        LODA,R0 TMPL
        STRA,R0 EXPL
        BCTR,UN SUB_LOOP

BORROW:
        ; Check for leading zero suppression
        COMI,R1 A'0'
        BCTR,GT PRINT_IT         ; Not '0', must print
        EORZ,R0                  ; v0.9 fix: R0 still held the borrow
                                  ; subtraction's leftover (nonzero) value
                                  ; here, so IORZ,R3 (r0|=r3) wasn't
                                  ; actually testing R3 alone - clear R0
                                  ; first so the OR reflects R3 only.
        IORZ,R3                  ; Test R3 flag
        BCFR,EQ PRINT_IT         ; Flag set (>0), print the zero
        COMI,R2 4                ; Is it the final column (1s)?
        BCTR,EQ PRINT_IT         ; Always print the final digit
        BCTR,UN NEXT_DIG         ; Skip printing leading zero

PRINT_IT:
        LODI,R3 1                ; Set leading zero flag
        LODZ,R1                  ; R0 = R1 (destination is always R0)
        ZBSR *VCOUT              ; Print the character
NEXT_DIG:
        ADDI,R2 1                ; Advance to next power of 10
        COMI,R2 5                ; Have we processed all 5 powers?
        BCTR,LT DIGIT_LOOP
        CPSL $08                  ; v0.9 fix (round 3): the final digit
                                  ; column always exits via BORROW (the
                                  ; ones-column subtraction "overshoots by
                                  ; one" to detect completion, same as
                                  ; every other column), which leaves WC
                                  ; enabled from that column's PPSL $08 -
                                  ; and nothing loops back through the
                                  ; SUB_LOOP-top CPSL $08 to clear it once
                                  ; the whole loop is done. Without this,
                                  ; PRINT_S16 returned to its caller with
                                  ; WC still on, and the NEXT SUBA/ADDA
                                  ; anywhere downstream (e.g. the keyword
                                  ; matcher's own character-compare SUBA)
                                  ; silently absorbed a stale carry-in -
                                  ; confirmed via testing: two PRINT
                                  ; statements in a row broke the SECOND
                                  ; one's keyword dispatch even after R2/R3
                                  ; were correctly saved/restored, and a
                                  ; trivial print-one-fixed-character stub
                                  ; in place of this whole routine did NOT
                                  ; reproduce the failure - isolating the
                                  ; cause to leftover arithmetic-mode state
                                  ; rather than register contents.
        LODA,R0 SC1               ; restore caller's R2/R3 before returning
        STRZ,R2
        LODA,R3 SC0
        RETC,UN                  ; Return to caller

; Powers of 10 Tables (10000, 1000, 100, 10, 1)
P10_HI:
        db $27, $03, $00, $00, $00
P10_LO:
        db $10, $E8, $64, $0A, $01


; =============================================================================
;  GETLINE -- Minimal read a line from input into IBUF (v0.2, was RDLINE)
; In:  nothing
; Out: IBUF = NUL-terminated input line.
;      R3 is used as an index into IBUF (not IPH:IPL - both callers re-point
;      IP via VSET_IP_IBUF immediately after calling this). R3=$FF means
;      empty (matches the SWBASE convention) since the 2650's ",R3+"
;      addressing mode pre-increments before the access.
; Clobbers: R0, R1, R3
; v0.2: no backspace support, no buffer-full check - matches pBASIC65c02's
; own GETLINE ("no Backspace support or range limits - too many characters
; will crash"). Echo IS kept (ZBSR *VCOUT per char, same as old RDLINE) -
; a real 2650 terminal has no local echo of its own to fall back on, so
; dropping it would be a genuine interactive-usability regression, not
; just a size cut. The NUL check on CHIN's result is also kept (inherited
; from PIPBUG's CHIN - not confident enough in its exact hardware meaning
; to drop it without risk, unlike backspace/bounds-check which are purely
; our own added conveniences).
GETLINE:
        LODI,R3 $FF                      ; R3 = empty-buffer sentinel (pre-inc convention)
GL_LP:
        BSTA,UN CHIN                     ; [+1] blocking read
        STRZ,R1
        COMI,R1 CR
        BCTR,EQ GL_EOL
        COMI,R1 LF
        BCTR,EQ GL_EOL
        LODZ,R1                          ; R0 = char (indexed autoinc store only works for R0)
        STRA,R0 IBUF,R3+                 ; R3++ (pre-inc); IBUF[R3]=char
        ZBSR *VCOUT  
        BCTR,UN GL_LP
GL_EOL:
        EORZ,R0
        STRA,R0 IBUF,R3+                 ; R3++ (pre-inc, one past last char); NUL-terminate
        BCTA,UN PRT_CRLF                ; tail call

; =============================================================================
;  EATWORD -- Consume [A-Za-z$] chars at IP
; In:  IPH:IPL -> current position
; Out: IP advanced past word
; Clobbers: R0
EATWORD:
        LODA,R0 *IPH
        BSTR,UN UPCASE                   ; [+1]
        COMI,R0 A'A'
        BCTR,LT EW_DS
        COMI,R0 A'Z'+1
        BCTR,LT EW_ADV
EW_DS:
        COMI,R0 A'$'
        BCFR,EQ WSKIPRET
EW_ADV:
        ZBSR *VINC_IP 
        BCTR,UN EATWORD

; =============================================================================
;  WSKIP -- Skip spaces at IP
; In:  IPH:IPL -> current position
; Out: IPH:IPL -> first non-space char
; Clobbers: R0
WSKIP:
        LODA,R0 *IPH
        COMI,R0 SP
        BCFR,EQ WSKIPRET
        ZBSR *VINC_IP 
        BCTR,UN WSKIP 

; =============================================================================
;  UPCASE -- Convert R0 to uppercase if 'a'..'z'
; In:  R0 = character
; Out: R0 = uppercase character
; Clobbers: R0
UPCASE:
        COMI,R0 A'a'
        RETC,LT
        COMI,R0 A'z'+1
        BCFR,LT WSKIPRET
        SUBI,R0 32
WSKIPRET:
        RETC,UN

; =============================================================================
;  SHARED 16-BIT POINTER DECREMENT -- DEC_ET family
; DEC_IP:    IPH:IPL    -= 1    (offset  0 from IPH)
; Shares DEC_ET body via register bank switch, mirroring INC_ET.
; v0.2: DEC_LNUM/DEC_GOTO removed (they were STORE_LINE's shift-loop
; pointers, and STORE_LINE's shift loop no longer exists - see
; TRY_STORE_LINE). DEC_IP no longer needs a skip-chain in front of it.
; RAS rule: NO BSTA inside body -- must not consume extra depth.
; DEC_EXP/DEC_TMP omitted: MUL16 call site is at RAS depth 5+1=6 (unsafe).
DEC_IP:
        EORZ,R0                 ; offset = 0 (IPH:IPL)
DEC_ET:
        PPSL PSW_RS                 ; switch to alternate register bank
        STRZ R1                 ; R1 = offset
        LODA,R0 IPL,R1          ; load lo byte
        SUBI,R0 1
        STRA,R0 IPL,R1
        TPSL $01                ; C=1 = no borrow (lo was >=1): CC=EQ -> skip hi--
        BCTR,EQ ET_RET          ; C=0 = borrow (lo was 0): fall through to hi--
        LODA,R0 IPH,R1          ; borrow: decrement hi byte
        SUBI,R0 1
        BCTR,UN ET_STORE        ; borrow tail from INC_xx

; =============================================================================
;  SHARED 16-BIT POINTER INCREMENT  - INC_ET family
; INC_EXP : EXPH:EXPL += 1   (offset EXPH-IPH from IPH)
; INC_TMP : TMPH:TMPL += 1   (offset TMPH-IPH from IPH)
; INC_IP  : IPH:IPL  += 1    (offset 0 from IPH)
; All share INC_ET body using register bank switch.
; Rule: NO BSTA inside these -- must not consume extra RAS depth.
; Offsets are assembly-time expressions (e.g. EXPH-IPH=4) -- sequential
; ordering of the IPH..LNUML block must be preserved or these silently break.
INC_EXP:
        LODI,R0 EXPH-IPH        ; EXP offset from IPH (= 4); assembly-time expression
        db $EC                  ; COMA,R0 -- consume next 2 bytes (skip to INC_IP path)
INC_TMP:
        LODI,R0 TMPH-IPH        ; TMP offset from IPH (= 2); assembly-time expression
        db $C4                  ; COMI,R0 -- consume next 1 byte
INC_IP:
        EORZ,R0                 ; offset = 0 (IPH itself)
; Can jump in here with R0 set for offset
INC_ET:
        PPSL PSW_RS                 ; switch to alternate register bank
        STRZ R1                 ; R1 = offset
        LODA,R0 IPL,R1          ; load lo byte
        ADDI,R0 1
        STRA,R0 IPL,R1
        TPSL $01
        BCTR,LT ET_RET          ; no carry: done
        LODA,R0 IPH,R1          ; carry: increment hi byte
        ADDI,R0 1
ET_STORE:
        STRA,R0 IPH,R1
ET_RET:
        CPSL PSW_RS                 ; switch back to primary bank
        RETC,UN

; =============================================================================
;  EXP16_TO_ET family -- copy EXPH:EXPL to any RAM register pair.
;  ET_TO_EXP16 family -- copy any RAM register pair to EXPH:EXPL.
;
;  Placed immediately after DEC_ET so BCTR,UN ET_STORE / BCTR,UN ET_RET
;  reach the shared tails above within ±63 bytes.
;  Each entry loads its offset (XYZH-IPH) into R0 (always bank-0, unaffected
;  by PSW_RS), falls through to body.  STRZ R1 copies R0 into alt-R1 for
;  indexed addressing.  Primary R1/R2/R3 fully preserved via CPSL PSW_RS.
;  Clobbers R0 only.  NO BSTA inside body.
;  Direct BSTA,UN (no ZP slot): CUR_TO_EXP16 (1 site).
EXP16_TO_TMP:
        LODI,R0 TMPH-IPH      ; TMPH offset from IPH
        db $EC                  ; COMA,R0: skip next 2 bytes
EXP16_TO_GOTO:
        LODI,R0 GOTOH-IPH       ; GOTOH offset from IPH (= 8)
        db $EC                  ; COMA,R0: skip next 2 bytes
EXP16_TO_LNUM:
        LODI,R0 LNUMH-IPH       ; LNUMH offset from IPH (= 12)
EXP16_TO_ET:
        PPSL PSW_RS             ; switch to alternate register bank
        STRZ R1                 ; alt-R1 = R0 = destination offset
        LODA,R0 EXPL
        STRA,R0 IPL,R1          ; store lo byte to dest+1
        LODA,R0 EXPH
        BCTR,UN ET_STORE        ; store hi byte, restore bank, return

; -----------------------------------------------------------------------------
CUR_TO_EXP16:
        LODI,R0 CURH-IPH        ; CURH offset from IPH (= 10)
; v0.6: TMP_TO_EXP16 removed - orphaned (see PORT HISTORY); CUR_TO_EXP16
; now falls straight into ET_TO_EXP16.
ET_TO_EXP16:
        PPSL PSW_RS             ; switch to alternate register bank
        STRZ R1                 ; alt-R1 = R0 = source offset
        LODA,R0 IPH,R1          ; load hi byte from source
        STRA,R0 EXPH
        LODA,R0 IPL,R1          ; load lo byte from source
        STRA,R0 EXPL
        BCTR,UN ET_RET

; v0.6: IP_TO_TMP removed - inlined at its sole caller (DO_LIST's DLS_LP
; loop end). See PORT HISTORY.

; =============================================================================
;  JERRVAR -- Error with variable
;  JSYNERR -- Syntax error jump
; In:  nothing (R0 irrelevant)
; Out: jumps to DO_ERROR
; Clobbers: R0
; v0.1b: JERR_NXT/JFORERR/DRT_UNDERFLOW retired (NEXT/FOR/RETURN are cut,
; nothing raises ERR_NXT/ERR_FOR/ERR_RET anymore). JERROOM is now the last
; entry in this chain, so it drops through to DO_ERROR directly instead of
; skip-jumping over the next entry's LODI (there isn't one anymore).
JERRVAR:
        LODI,R0 ERR_VAR
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to BCTA
JSYNERR:
        LODI,R0 ERR_SYN
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to BCTA
JERROOM:
        LODI,R0 ERR_OOM
        ; drop through

; =============================================================================
;  DO_ERROR -- Print error, clear run state, return to REPL
; Entry: R0 = error code character ('0'..'8').
; Clears RUNFLG, SWSP, FORSP. Prints "?n" or "?n@line" if running.
; Tail-jumps to REPL (clears full hardware RAS).
; In:  R0 = error code
; Out: jumps to REPL
; Clobbers: all (RAS cleared by REPL)
DO_ERROR:
        STRZ,R1                         ; Save ASCII error code
        BSTR,UN PRT_QUEST               ; Print Question mark
        LODZ,R1
        ZBSR *VCOUT                     ; print error code
        LODA,R0 RUNFLG                  ; OPT-10: SC1=RUNFLG, 0->EQ, 1->GT
        BCTR,EQ DE_NL                   ; not running, no line number
        LODI,R0 '@'                     
        ZBSR *VCOUT                     ; Print at line
        BSTR,UN CUR_TO_EXP16             ; EXPH:EXPL = CURH:CURL
        BSTA,UN PRINT_S16                ; [+1]
DE_NL:
        BSTR,UN PRT_CRLF
        BSTA,UN DO_END                   ; [+1] clears SWSP, FORSP, GOTOFLG, RUNFLG
        BCTA,UN REPL                     ; REPL resets RAS (PSU SP bits) on entry

; =============================================================================
;  Shared character print routines -- $EC (COMA) byte-skip chain
; Each entry loads its character then falls through via the skip opcode trick.
PRT_CRLF:
        BSTR,UN PRT_CR                   ; Print CR/LF
        ; drop through
PRT_LF:
        LODI,R0 LF
        db $EC
PRT_QUEST:
        LODI,R0 '?'
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to next LODI
PRT_CR:
        LODI,R0 CR
        db $EC
PRT_SPACE:
        LODI,R0 32
        ZBRR *VCOUT                
        
; =============================================================================
;  SET_IP_IBUF -- Set IPH:IPL = IBUF base address
; In:  nothing
; Out: IPH = <IBUF, IPL = >IBUF
; Clobbers: R0
SET_IP_IBUF:
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
        RETC,UN

; =============================================================================
; CLR_EXP -- Helper Zeroes EXP
; Clobbers R0
CLR_EXP:
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 EXPL
        RETC,UN
        
; =============================================================================
;  TABLES 
BANNER:
        DB CR, LF, "pBASIC2650V1.0", CR, LF, NUL

; -- Combined operator + statement dispatch table (v0.5: extends the v0.3
; statement-only table with 6 operator entries ahead of it, matching
; pBASIC65c02's final TOK_CHARS/TOK_VECS shape - one table, one dispatch
; tail, for both). Format: [char][hi][lo], stride 3, NUL-terminated.
; Statement scan (STMT_EXEC/MD_SCAN) safely runs the WHOLE table because
; its 2nd-char letter-gate guarantees the char being matched is A-Z,
; which none of the 6 operator chars are - no bounding needed there.
; Operator scan (EXPR_LOOP/OP_SCAN) is explicitly bounded to the first 6
; entries (BDRR counter) - unlike the statement scan, it has no gate to
; stop it from wrongly matching a variable-adjacent statement letter
; (e.g. mistaking the 'P' that starts the next statement for an operator).
TOK_CHARS:
        DB "+", <DO_ADD,    >DO_ADD       ; +
        DB "-", <DO_SUB,    >DO_SUB       ; -
        DB "*", <DO_MUL,    >DO_MUL       ; *
        DB "/", <DO_DIV,    >DO_DIV       ; /
        DB "=", <DO_EQOP,   >DO_EQOP      ; = (relop, folded in - v0.5)
        DB "<", <DO_LTOP,   >DO_LTOP      ; < (relop, folded in - v0.5)
        DB "A", <DO_ASK,    >DO_ASK       ; ASK
        DB "E", <DO_END,    >DO_END       ; END
        DB "G", <DO_GOTO,   >DO_GOTO      ; GOTO
        DB "I", <DO_IF,     >DO_IF        ; IF
        DB "L", <DO_LIST,   >DO_LIST      ; LIST
        DB "N", <DO_NEW,    >DO_NEW       ; NEW
        DB "P", <DO_PRINT,  >DO_PRINT     ; PRINT
        DB "R", <DO_RUN,    >DO_RUN       ; RUN
        DB "W", <DO_WR,     >DO_WR        ; WR
        DB NUL, <SE_NOTKW,  >SE_NOTKW     ; No match handler

ROMEND: 

;  RAM variables -- sequential RES block 
 
        ORG     4096    ; half a 2650 8kbyte page

; --- Ordered group: offsets from IPH used by INC_ET/DEC_ET/NEG_SHARED ---
IPH     RES 1       ; interpreter pointer hi       (INC_ET offset 0)
IPL     RES 1       ; interpreter pointer lo
TMPH    RES 1       ; temp 16-bit hi               (INC_ET offset 2 = TMPH-IPH)
TMPL    RES 1       ; temp 16-bit lo
GOTOFLG RES 1       ; $00=sequential $01=GOTO $02=GOSUB $03=FOR direct addr
GOTOH   RES 1       ; pending target hi            (DEC_ET offset 8 = GOTOH-IPH)
GOTOL   RES 1       ; pending target lo
CURH    RES 1       ; current line hi  (error reporting)
CURL    RES 1       ; current line lo

LNUMH   RES 1       ; scratch line number hi       (DEC_ET offset 12 = LNUMH-IPH)
LNUML   RES 1       ; scratch line number lo
EXPH    RES 1       ; expression result hi         (INC_ET offset 4 = EXPH-IPH)
EXPL    RES 1       ; expression result lo
SWSTK   RES 2       ; next-line pointer cache [NLP_H][NLP_L] written by DR_EXEC

; --- Remaining ---
SC0     RES 1       ; Scratch byte 0
SC1     RES 1       ; Scratch byte 1
PEH     RES 1       ; Program end pointer hi
PEL     RES 1       ; Program end pointer lo
SAVEH   RES 1       ; ADD16_SAVE_EXP: popped left operand scratch (hi)
SAVEL   RES 1       ; ADD16_SAVE_EXP: popped left operand scratch (lo)
TEMPRETH RES 1      ; SW return address hi
TEMPRETL RES 1      ; SW return address lo

;  --- Flags & Stuff --- 
RUNFLG  RES 1       ; $01=running $00=immediate
R3SAVE  RES 1       ; Save/restore R3 across PARSE_U16 multiply loop
NEGFLG  RES 1       ; Sign flag

;  SW call stack -- used by PARSE_EXPR / PRINT_S16 only
; R3 = index ($FF=empty, grows up). Each frame = [lo][hi].
; Push: STRA,R0 *SWBASE,R3+ (lo first), STRA,R0 *SWBASE,R3+ (hi).
; Pop:  LODA,R0 *SWBASE,R3- (hi first), LODA,R0 *SWBASE,R3- (lo).
SWBASE  RES 16      ; SW stack base (v0.5, was 32 bytes). Sized for
                     ; PRINT_S16/PREC's OWN independent digit-printing
                     ; recursion (up to 15 bytes for a 5-digit number: each
                     ; digit level pushes 1 byte for the digit + 2 for a
                     ; return address if it recurses further, plus PS_NZ's
                     ; 2-byte outer wrapper - 3*5-2+2=15), NOT by the
                     ; expression evaluator's operand-save need (at most a
                     ; few bytes for 1-2 levels of paren nesting) - these
                     ; two consumers are temporally separate (PRINT_S16
                     ; only ever runs after an expression has fully
                     ; unwound) but share this one array, and PRINT_S16's
                     ; requirement is the larger of the two.

; Buffers
IBUF    RES 64      ; Input buffer 64 bytes
VARS    RES 52      ; A-Z variables 2 bytes each

; =============================================================================
;  Pre-loaded SHOWCASE program
;
;  Line format: <lineno_hi> <lineno_lo> <body_ASCII> <CR>
;  Lines  10-190: feature demos (PRINT, WR, arithmetic, comparisons, GOTO loop)
;  Lines 236-238: LIST demo (v0.1b: FOR/NEXT, GOSUB/RETURN demos and the
;                 line-530 GOSUB target removed; LIST is whole-program only)
;  Lines 300-510: Mandelbrot set renderer (v4.3: widened, C=-144..28 step 4,
;                 44 cols vs v4.2's 32; row range I=-64..56 step 6 unchanged)
;  Line  530:     GOSUB subroutine (PRINT "sub"; / RETURN)
;
;  Format: DB hi,lo,"text",$0D  -- hi-then-lo matches DR_EXEC record format.
;  $22=DQ $3B=semicolon  in-string chars that need escaping.
; =============================================================================
PROG:
        DB 0,20,"PRINT ",$22,"-- pBASIC2650 v0.4 Showcase --",$22,$0D
        DB 0,30,"PRINT ",$22,"--- PRINT / WR ---",$22,$0D                      ; 30  PRINT "--- PRINT / WR ---"
        DB 0,40,"WR 65",$0D                                                    ; 40  WR 65
        DB 0,41,"WR 66",$0D                                                    ; 41  WR 66
        DB 0,42,"WR 67",$0D                                                    ; 42  WR 67
        DB 0,43,"PRINT",$0D                                                    ; 43  PRINT
        DB 0,50,"PRINT ",$22,"--- ARITHMETIC ---",$22,$0D                      ; 50  PRINT "--- ARITHMETIC ---"
        DB 0,60,"PRINT ",$22,"3+4=",$22,$3B,"3+4",$3B,$22,"  10-3=",$22,$3B,"10-3",$3B,$22,"  6*7=",$22,$3B,"6*7",$0D  ; 60  PRINT "3+4=";3+4;"  10-3=";10-3;"  6*7=";6*7
        DB 0,70,"PRINT ",$22,"20/4=",$22,$3B,"20/4",$0D                        ; 70  PRINT "20/4=";20/4
        DB 0,80,"PRINT ",$22,"--- COMPARISONS ---",$22,$0D                     ; 80  PRINT "--- COMPARISONS ---"
        DB 0,90,"IF 2<9 PRINT ",$22,"2<9 ok",$22,$0D                      ; 90  IF 2<9 PRINT "2<9 ok"
        DB 0,100,"IF 3<5 PRINT ",$22,"3<5 ok",$22,$0D                     ; 100 IF 3<5 PRINT "3<5 ok"
        DB 0,110,"IF 7=7 PRINT ",$22,"7=7 ok",$22,$0D                     ; 110 IF 7=7 PRINT "7=7 ok"
        DB 0,120,"IF 1<8 PRINT ",$22,"1<8 ok",$22,$0D                     ; 120 IF 1<8 PRINT "1<8 ok"
        DB 0,130,"IF 3=3 PRINT ",$22,"3=3 ok",$22,$0D                     ; 130 IF 3=3 PRINT "3=3 ok"
        DB 0,140,"PRINT ",$22,"--- LOOP via GOTO ---",$22,$0D                  ; 140 PRINT "--- LOOP via GOTO ---"
        DB 0,150,"I=1",$0D                                                      ; 150 I=1
        DB 0,160,"IF 5<I GOTO 190",$0D                                    ; 160 IF 5<I GOTO 190
        DB 0,170,"PRINT I",$3B,$0D                                              ; 170 PRINT I;
        DB 0,180,"I=I+1",$0D                                                    ; 180 I=I+1
        DB 0,185,"GOTO 160",$0D                                                 ; 185 GOTO 160
        DB 0,190,"PRINT ",$22,"",$22,$0D                                        ; 190 PRINT ""
        DB 0,216,"PRINT ",$22,"",$22,$0D                                        ; 216 PRINT ""
        DB 0,236,"PRINT ",$22,"--- LIST ---",$22,$0D                            ; 236 PRINT "--- LIST ---"
;        DB 0,238,"LIST",$0D                                                     ; 238 LIST
        DB 0,240,"GOTO 300",$0D                                                 ; 240 GOTO 300
        DB 1,44,"PRINT ",$22,"--- MANDELBROT ---",$22,$0D                      ; 300 PRINT "--- MANDELBROT ---"
        DB 1,54,"I=-64",$0D                                                     ; 310 I=-64
        DB 1,64,"IF 56<I GOTO 510",$0D                                    ; 320 IF 56<I GOTO 510
        DB 1,74,"D=I",$0D                                                       ; 330 D=I
        DB 1,84,"C=-144",$0D                                                    ; 340 C=-144 (widened from -120)
        DB 1,94,"IF 28<C GOTO 480",$0D                                    ; 350 IF 28<C GOTO 480 (widened from 4)
        DB 1,104,"A=C",$0D                                                      ; 360 A=C
        DB 1,105,"B=D",$0D                                                      ; 361 B=D
        DB 1,106,"E=0",$0D                                                      ; 362 E=0
        DB 1,107,"N=1",$0D                                                      ; 363 N=1
        DB 1,114,"IF 16<N GOTO 420",$0D                                   ; 370 IF 16<N GOTO 420
        DB 1,124,"IF 0<E GOTO 410",$0D                                    ; 380 IF 0<E GOTO 410
;        DB 1,134,"T=A*A/64-B*B/64+C",$0D                                       ; 390 T=A*A/64-B*B/64+C
        DB 1,134,"T=(A*A/64)-(B*B/64)+C",CR             
        DB 1,144,"B=2*A*B/64+D",$0D                                             ; 400 B=2*A*B/64+D
        DB 1,145,"A=T",$0D                                                      ; 401 A=T
        ; DB 1,154,"IF 256<A*A/64+B*B/64 IF E=0 E=N",$0D               ; 410 IF 256<A*A/64+B*B/64 IF E=0 E=N
        DB 1,149,"F=(A*A/64)+(B*B/64)",CR                                       ; 405 F=(A*A/64)+(B*B/64)  (v0.6 RAS-01 fix: new line, single paren level)
        DB 1,154,"IF 256<F IF E=0 E=N",CR                                       ; 410 IF 256<F IF E=0 E=N  (F now a bare atom, no parens needed)
        DB 1,164,"N=N+1",$0D                                                    ; 420 N=N+1
        DB 1,165,"IF 16<N GOTO 430",$0D                                   ; 421 IF 16<N GOTO 430
        DB 1,166,"GOTO 370",$0D                                                 ; 422 GOTO 370
        DB 1,174,"IF 0<E WR E+32",$0D                                     ; 430 IF 0<E WR E+32
        DB 1,184,"IF E=0 WR 32",$0D                                       ; 440 IF E=0 WR 32
        DB 1,194,"C=C+4",$0D                                                    ; 450 C=C+4
        DB 1,204,"GOTO 350",$0D                                                 ; 460 GOTO 350
        DB 1,224,"PRINT",$0D                                                    ; 480 PRINT
        DB 1,234,"I=I+6",$0D                                                    ; 490 I=I+6
        DB 1,244,"GOTO 320",$0D                                                 ; 500 GOTO 320
        DB 1,254,"END",$0D                                                      ; 510 END
SHOWCASE_END:

        END
