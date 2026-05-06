; uBASIC2650.asm  —  Tiny BASIC for Signetics 2650
; Version: v2.1
; Date:    2026-05-06
; Size: 4234 bytes ($0440-$14C9)
;
; Target: PIPBUG 1 monitor (1kB ROM $0000-$03FF, 64B RAM $0400-$043F)
;   Code base $0440.
;   I/O via PIPBUG ROM entry points (BSTA,UN):
;     COUT $02B4  — output char in R0
;     CHIN $0286  — blocking input, char returned in R0
;     CRLF $008A  — emit CR+LF
;
; Assembler: asm2650.c v1.7   Simulator: pipbug_wrap v1.1
; Build:
;   gcc -Wall -O2 -o asm2650 asm2650.c
;   gcc -Wall -O2 -DGAMER -o pipbug_wrap pipbug_wrap.c
;   ./asm2650 uBASIC2650.asm uBASIC2650.hex
;   ./pipbug_wrap uBASIC2650.hex
;   ./pipbug_wrap -t uBASIC2650.hex             # CPU trace
;   ./pipbug_wrap -b 0xADDR uBASIC2650.hex      # breakpoint
;   ./pipbug_wrap -m 0xADDR LEN uBASIC2650.hex  # mem dump at halt
;
; BUG-MAND-01 (FIXED): SC1 conflict in PARSE_EXPR/APPLY_OP.
;   cur_prec saved in SC1; APPLY_OP clobbered SC1 with left.lo.
;   Fix: use PRECTMP ($163D) to hold cur_prec across APPLY_OP.
;
; ── KNOWN OPEN BUGS (v2.1-dev, 2026-04-30) ──────────────────────────────────
;
; BUG-MAND-01 (ACTIVE): Mandelbrot expression U*U/16+V*V/16 gives wrong result.
;   Symptom: PRINT U*U/16+V*V/16 returns 1 when U=16,V=0 (should be 16).
;   A+0=16 works, U*U/16=16 works, but U*U/16+V*V/16=1.
;   Root cause under investigation — suspected PARSE_EXPR operator stack issue
;   when mixing * / + with identical-named variables.
;   Does not affect: arithmetic without division-then-addition combos.
;   Does not affect: showcase non-Mandelbrot sections.
;
; ── FIXED THIS SESSION (v2.1-dev) ────────────────────────────────────────────
;
; CR-FORMAT CHANGE: Line records now [linehi][linelo][body...][CR]
;   Eliminates bodylen byte, saves ~50 bytes in STORE_LINE/DELETE_LINE.
;   All affected routines updated: STORE_LINE, DELETE_LINE, FIND_LINE,
;   FIND_INS, DO_LIST, DO_RUN.
;
; BUG-UNSIGNED-01 (FIXED): All record-walk boundary checks (DO_LIST DLS_LP,
;   DO_RUN DR_LP, FIND_LINE, FIND_INS, DELETE_LINE DL2_LP) used signed
;   subtraction for the lo-byte comparison. Signed fails when PEL > $7F
;   (e.g. after storing 11+ lines, PEL=$C9 → $34-$C9 signed is GT, not LT).
;   Fix: TPSL $01 after SUBA for lo byte to check carry (unsigned borrow).
;
; BUG-STORE-01 (FIXED): STORE_LINE scratch registers NEGFLG:RELOP were clobbered
;   by PARSE_EXPR (NEGFLG used for sign in MUL/DIV, RELOP used for relop mask).
;   Caused lines 12+ to corrupt program store silently.
;   Fix: Changed STORE_LINE scratch to CURH:CURL (only written during RUN).
;
; BUG-STORE-02 (FIXED): Space-check scratch also used LNUMH:LNUML, clobbering
;   the line number before FIND_INS was called.
;   Fix: Space-check scratch moved to NEGFLG:RELOP (re-evaluated as safe here).
;   Then scratch moved to CURH:CURL to avoid PARSE_EXPR collision (BUG-STORE-01).
;
; BUG-DIV-01 (FIXED): DIV16 loop used signed comparison for lo byte of
;   dividend vs divisor. For 32767/128: $7F-$80 signed = GT → subtracted;
;   correct unsigned: $7F < $80 → stop. Fix: TPSL $01 carry check.
;
; BUG-DIV-02 (FIXED): DIV16 borrow propagation used BCFR,LT (old signed-borrow
;   pattern). Fix: TPSL $01 / BCTR,EQ DV_SNB (carry-based borrow detect).
;
; BUG-PRINT-02 (FIXED): PRINT_S16 zero check used BCTR,GT to skip "0" path.
;   EXPL=$FF (=255 unsigned) is -1 signed → CC=LT → fell to PRINT "0".
;   Fix: explicit BCTR,EQ PS16P_ZERO test only when EXPH=0 AND EXPL=0.
;
; BUG-CHR-02 (FIXED): CHR$(expr) only evaluated first atom (called PARSE_FACTOR
;   not PARSE_EXPR). CHR$(I+48) returned I not I+48.
;   Fix: CHR$ calls PARSE_EXPR; PF_CHROK no longer re-consumes ')' since
;   PARSE_EXPR handles ')' via PX_RPAR.
;
; ── REGRESSION STATUS (v2.1-dev, 2026-04-30) ─────────────────────────────────
;   PRINT numeric (all range)   ✓  including 128..32767, -32768
;   PRINT 255, 256              ✓  (BUG-PRINT-02 fixed)
;   256/16, 32767/128           ✓  (BUG-DIV-01/02 fixed)
;   Arithmetic +/-/*///%        ✓
;   IF all relops               ✓
;   CHR$(expr)                  ✓  (BUG-CHR-02 fixed)
;   CHR$() A-Z loop             ✓
;   LET / PRINT variable        ✓
;   GOTO counted loop           ✓
;   Nested IF                   ✓
;   LIST / NEW / RUN            ✓
;   Edit/delete line            ✓
;   11+ line programs           ✓  (BUG-UNSIGNED-01/STORE-01/02 fixed)
;   Mandelbrot kernel (3 iters) ✓
;   U*U/16+V*V/16 expression    ✗  BUG-MAND-01 (active)
;
; ── FIXED THIS SESSION ───────────────────────────────────────────────────────
;
; BUG-RELOP-02 (FIXED): TMI,R0 RELOP wrong — TMI takes immediate literal byte,
;   assembler embedded $00 → all IF conditions trivially true.
;   Fix: LODA,R1 RELOP / ANDZ,R1 / BCTR,EQ DIF_FALSE (runtime AND test).
;
; BUG-PRINT-MIN (FIXED): PRINT -32768 output "-0" — negation of $8000 overflows.
;   Fix: detect EXPH=$80,EXPL=$00 before negation, print "32768" directly.
;
; BUG-PRINT-01 (FIXED): PRINT >=10000 corrupted (e.g. 32767→"3150#").
;   Fix: PPSL $02 unsigned mode, COMZ,R1, ADDI,R3 1, TPSL $01 borrow.
;
; BUG-RELOP-01a (FIXED): EORZ,R1 in PARSE_RELOP entry does R0^=R1 not R1=0.
;   Fix: EORZ,R0 / STRZ,R1.
;
; BUG-RELOP-01b (FIXED): STRZ,R1 before LODZ,R1 on PARSE_RELOP exit clobbered
;   mask with non-relop char. Fix: removed STRZ,R1.
;
; ── REGRESSION STATUS (2026-04-22) ───────────────────────────────────────────
;   PRINT numeric (all range)  ✓   including -32768, 32767
;   Arithmetic +/-/*///%       ✓
;   Parentheses                ✓
;   IF relops =<><=>=<>        ✓   all 6 operators
;   Nested IF                  ✓
;   GOTO counted loop          ✓
;   LET / PRINT variable       ✓
;   LIST / NEW / RUN           ✓
;   PRINT CHR$(n) simple       ✓
;   CHR$() A-Z loop            ✓   (BUG-CHR-01 self-resolved after BCTA→BCTR pass)
;
; ── MEMORY MAP ───────────────────────────────────────────────────────────────
;   $0000-$03FF  PIPBUG ROM (read-only)
;   $0400-$043F  PIPBUG RAM (reserved)
;   $0440-$147D  uBASIC code (this file, 4158 bytes)
;   $1600-$163F  RAM variables (see EQU block)
;   $1640-$165F  SW call stack (32 bytes = 16 frames, v2.0 — not yet active)
;   $1660-$1662  SW stack workspace (TEMPRETH, TEMPRETL, R3SAVE)
;   $1658-$165C  PRINT_S16 digit buffer P16BUF (5 bytes — not yet used)
;   $1663-$16A2  IBUF input buffer (64 bytes)
;   $1A00-$1A33  VARS A-Z variables (52 bytes)
;   $1A34-$1BFF  PROG program store (460 bytes — NOTE: reduced from 1607)
;   $1C00        PROGLIM
;
; ── CC SEMANTICS (2650 ALU) ──────────────────────────────────────────────────
;   ADD: result>=128→LT  result>0,<128→GT  result=0→EQ
;   SUB: result>=128→LT  result>0,<128→GT  result=0→EQ
;   Carry bit (PSL bit 0) set independently: C=1 means carry/no-borrow.
;   CORRECT carry skip: TPSL $01 then RETC,LT / BCTA,LT = branch if C=0.
;   WRONG: BCTA,GT after ADD (tests result sign, not carry — only safe $01-$7F).
;
; ── HI/LO OPERATOR CONVENTION ────────────────────────────────────────────────
;   <ADDR = HIGH byte (bits 15:8)   e.g. <$1A34 = $1A
;   >ADDR = LOW  byte (bits  7:0)   e.g. >$1A34 = $34
;
; ── RAS DEPTH BUDGET (8-level hardware stack) ────────────────────────────────
;   PIPBUG COUT/CHIN: depth+2 internally. CRLF: depth+3.
;   Safe max user call depth from REPL: 5 levels.
;   Deepest working path: REPL(0)→STMT_EXEC(1)→DO_IF(2)→STMT_EXEC(3)→
;     DO_PRINT(4)→PARSE_EXPR(5)→PARSE_FACTOR(6)→PARSE_S16(7) = SP=7. Safe.
;   CHR$() path: adds one more → SP=8 = overflow (BUG-CHR-01).
;
; ── SCRATCH REGISTER ALLOCATION ──────────────────────────────────────────────
;   R0       — working register, arithmetic, I/O
;   R1       — index register; also PRINT_S16 digit buffer index (P16BUF)
;   R2       — never written by any routine; long-lived scratch (DO_LET var letter)
;   R3       — loop counter (BDRR/BIRR); SW stack index (v2.0, not yet active)
;   SC0:SC1  — general scratch (clobbered by STMT_EXEC — not inter-statement safe)
;   SWSTK[0:1] ($162E:$162F) — DO_RUN next-line-pointer save across STMT_EXEC
;   LNUMH:LNUML — scratch line number; save area in DO_LIST (BUG-BASIC-12 fix)
;   TMPH:TMPL — general 16-bit temp; clobbered by PRINT_S16 (BUG-BASIC-12 fix)
;
; ── v2.0 SW STACK INFRASTRUCTURE (EQUs added, not yet activated) ─────────────
;   SWBASE=$1640, TEMPRETH=$1660, TEMPRETL=$1661, R3SAVE=$1662, P16BUF=$1658.
;   Auto-index semantics confirmed: *BASE,R1+ = pre-increment (R1++ then access).
;   Correct PRINT_S16 digit push pattern: init R1=$FF, *BUF,R1+ writes BUF+0 first.
;   Correct pop pattern: increment R1 once after N pushes, loop *BUF,R1- until R1=0.
;   SWRETURN implementation deferred pending BUG-RELOP-02 fix and BUG-CHR-01 fix.
;
; ── KNOWN BUGS (as of v2.0-dev, 2026-04-22) ──────────────────────────────────
;
; BUG-RELOP-02 (ACTIVE — BLOCKS ALL IF/THEN):
;   Root cause: `TMI,R0 RELOP` in DIF_TMASK is semantically wrong.
;   TMI is a 2-byte immediate instruction: TMI,rn mask — the mask is a
;   LITERAL BYTE embedded in the instruction, not a memory address.
;   The assembler resolves RELOP=$163E and truncates to immediate=$3E (or $00
;   depending on truncation), so the test is always against a fixed constant,
;   not the runtime RELOP value.  This makes ALL relational tests wrong.
;
;   Fix: Replace the TMI approach with a runtime AND:
;     DIF_TMASK:
;         LODA,R1 RELOP        ; R1 = runtime bitmask from RAM
;         ANDZ,R1              ; R0 &= R1  (R0 had the result-bit: 1=LT, 2=EQ, 4=GT)
;         BCTR,EQ DIF_FALSE    ; zero → no bit match → condition false
;         BCTA,UN DIF_TRUE     ; non-zero → condition true
;   Cost: net neutral (was 2 bytes TMI+branch, now 4 bytes — but removes two
;   redundant intermediate labels DIF_IS_LT/DIF_IS_EQ).
;   After fix, the whole DIF_IS_LT/DIF_IS_EQ/DIF_TMASK block simplifies to:
;     ; SC1 = $FF→LT, $00→EQ, $01→GT
;     LODA,R0 SC1
;     COMI,R0 $FF
;     BCTR,EQ DIF_BIT_LT       ; map LT→bit 0
;     COMI,R0 $00
;     BCTR,EQ DIF_BIT_EQ       ; map EQ→bit 1
;     LODI,R0 4                 ; GT→bit 2
;     BCTA,UN DIF_ANDTEST
;   DIF_BIT_LT: LODI,R0 1 ; fall to DIF_ANDTEST (already done above)
;   DIF_BIT_EQ: LODI,R0 2
;   DIF_ANDTEST:
;     LODA,R1 RELOP
;     ANDZ,R1
;     BCTR,EQ DIF_FALSE
;     ; fall through to DIF_TRUE
;
; BUG-RELOP-01 (FIXED this session — two sub-bugs):
;   BUG-RELOP-01a: `EORZ,R1` in PARSE_RELOP entry was used to zero R1 but
;     EORZ,rn computes R0 ^= rn, leaving rn unchanged.  R1 stayed dirty with
;     leftover INC_IP value, OR'd into the mask → wrong RELOP for <=, >=.
;     Fix: replaced with EORZ,R0 / STRZ,R1.
;   BUG-RELOP-01b: `STRZ,R1` before `LODZ,R1` on the PARSE_RELOP exit path
;     overwrote R1 (the accumulated mask) with the non-relop char in R0.
;     LODZ,R1 then loaded the corrupted char into R0 → RELOP = ASCII char.
;     Fix: removed STRZ,R1, kept only LODZ,R1.
;
; BUG-PRINT-01 (FIXED this session):
;   Root cause: the digit subtraction loop used BIRR,R3 (increment-and-branch)
;   as a counter, but BIRR increments BEFORE testing — R3 count was off by one
;   for values crossing place-value boundaries.  Also, the original borrow
;   propagation tested BCFA,LT (no-borrow) instead of TPSL $01 (carry test),
;   producing wrong hi-byte adjustment on borrow.  New version uses unsigned
;   compare mode (PPSL $02), COMZ,R1 for comparison, ADDI,R3 1 for clean
;   incrementing.  Restores PSL from R2 (saved via SPSL/STRZ,R2) on exit.
;   Status: 0–32767 correct. -32768 still fails (two's-complement min-value
;   negation overflow: -32768 negated is still -32768).
;
; BUG-PRINT-MIN (ACTIVE — minor):
;   PRINT -32768 outputs "-0".  Negation of $8000: XOR→$7FFF, INC_EXP→$8000,
;   sign bit still set → treated as negative again.
;   Fix: special-case $8000 before negation: if EXPH=$80 and EXPL=$00, print
;   literal "32768" after printing "-".
;   Impact: minor — values 0 to 32767 all correct.
;
; BUG-CHR-01 (FIXED — self-resolved):
;   CHR$() loop was overflowing hardware RAS at depth 8. After the BCTA→BCTR
;   pass saved 135 bytes and shifted code addresses, the branch targets moved
;   within relative range and depth reduced to 7. All CHR$() loop tests pass.
;
; BUG-COLON (UNCONFIRMED):
;   Multi-statement lines "LET A=1:PRINT A" may not be implemented.
;   Not tested — investigate after relop fix.
;
; ── REGRESSION STATUS (v2.0-dev, 2026-04-22) ────────────────────────────────
;   PRINT numeric (0-9999)     ✓
;   PRINT numeric (10000-32767)✓  (fixed BUG-PRINT-01 this session)
;   PRINT -32768               ✗  (BUG-PRINT-MIN)
;   IF relops (=, <, >, <=, >=, <>)  ✗  (BUG-RELOP-02 — all wrong)
;   PRINT CHR$(n) simple       ✓
;   CHR$() in loop             ✗  (BUG-CHR-01)
;   LET A=expr / PRINT A       needs re-test after relop fix
;   GOTO counted loop          needs re-test after relop fix
;   LIST / NEW / RUN           ✓
;   Arithmetic +/-/*///%       ✓
;   Parentheses                ✓
;
; ── NEXT STEPS (priority order) ──────────────────────────────────────────────
;   1. Fix BUG-RELOP-02: replace TMI,R0 RELOP with LODA,R1 RELOP / ANDZ,R1
;      in DIF_TMASK. ~4 lines changed, net neutral size.
;   2. Fix BUG-PRINT-MIN: special-case -32768 in PRINT_S16 prologue. ~8 bytes.
;   3. Full regression after 1+2: GOTO loop, LET, nested IF.
;   4. BUG-CHR-01: SW stack for PARSE_EXPR (major restructuring).
;
; ── BCTA→BCTR SIZE REDUCTION (deferred) ─────────────────────────────────────
;   Assembler --no-warn-local-branch reports ~90 BCTA that could be BCTR.
;   Estimated saving: ~90 bytes.  Apply after all bugs fixed.

; Change history:
;   v2.1-dev  CR-terminated line format (bodylen byte removed). All 6 affected
;             routines updated. Net size change: -10 bytes before unsigned fixes.
;             BUG-UNSIGNED-01 FIXED: signed boundary checks in DLS_LP/DR_LP/
;               FIND_LINE/FIND_INS/DL2_LP fail when PEL>$7F. Fix: carry check.
;             BUG-STORE-01/02 FIXED: scratch register conflicts in STORE_LINE.
;               Moved to CURH:CURL which is only written during DO_RUN.
;             BUG-DIV-01/02 FIXED: DIV16 unsigned comparison and borrow.
;               256/16=16 now correct (was 1). 32767/128=255 now correct.
;             BUG-PRINT-02 FIXED: PRINT_S16 zero check excluded $00FF (255).
;             BUG-CHR-02 FIXED: CHR$(I+48) now evaluates full expression.
;             BUG-MAND-01 IDENTIFIED: U*U/16+V*V/16 gives wrong result when
;               mixing division+addition with variables. Under investigation.
;             Regression: all prior tests pass; Mandelbrot render pending fix.
;             Code size: 4224 bytes ($0440-$14BF).
;   v2.0-dev  SW stack EQU infrastructure added (SWBASE/TEMPRETH/TEMPRETL/R3SAVE/
;               P16BUF). Memory layout: IBUF→$1663, VARS→$1A00, PROG→$1A34.
;             PRINT_S16 rewritten: unsigned compare mode (PPSL $02), COMZ,R1,
;               ADDI,R3 1, TPSL $01 borrow, PSL save/restore via R2.
;             DIVTAB now inline in PRINT_S16 (not a shared table).
;             BUG-PRINT-01 FIXED: PRINT 10000–32767 now correct.
;             BUG-RELOP-01a FIXED: EORZ,R0/STRZ,R1 replaces EORZ,R1 in PARSE_RELOP.
;             BUG-RELOP-01b FIXED: removed STRZ,R1 from PARSE_RELOP exit path.
;             BUG-RELOP-02 IDENTIFIED (ACTIVE): TMI,R0 RELOP uses RELOP as
;               immediate byte (assembles to $00), not runtime memory read.
;               All IF/THEN conditions broken. Fix is LODA,R1 RELOP / ANDZ,R1.
;             Auto-index semantics confirmed: *BASE,Rn+/- are pre-modify.
;             Regression: PRINT numeric ✓, IF/THEN ✗ (BUG-RELOP-02),
;               CHR$() simple ✓, CHR$() loop ✗ (BUG-CHR-01).
;   v1.17  Conditional returns replacing BCTA+RETC patterns (~15 bytes).
;          Relop bitmask parser: <1 =2 >4 in R1; TMI test in DO_IF (~120 bytes).
;          FIND_LINE calls FIND_INS, sharing walk code (~57 bytes).
;          KW_TAB [c1][c2][hi][lo]; STMT_EXEC indirect dispatch via *GOTOH.
;          Paren bug FIXED (PX_POPSENT copies value down, decrements STKIDX).
;          HW stack overflow FIXED (INC_IP inlined in PARSE_U16 digit loop).
;   v1.15  CHR$(), PRINT semicolons, modulo %; RAS fix (PARSE_U16 WSKIP removed).
;   v1.11 BUG-SCA-12 FIXED: DO_RUN next-line-pointer save/restore used
;           STRA,R0 *SWSTK / LODA,R0 *SWSTK (indirect — dereferences the value
;           stored AT SWSTK as a pointer, then accesses that address). After
;           CLRV, SWSTK=$00:$00, so the first RUN wrote the next-line pointer
;           hi byte into PIPBUG ROM at $0000. Fix: STRA,R0 SWSTK / LODA,R0 SWSTK
;           (direct), matching the correct SWSTK+1 usage on adjacent lines.
;         BUG-SCA-13 FIXED: WinArcadia assembler requires labels to be on their
;           own dedicated line (per header comment). Three labels had code on the
;           same line: UC_DO:, UC_RET:, EW_DS:. WinArcadia silently dropped the
;           instruction on the label line, so UC_DO jumped to RETC,UN instead of
;           SUBI,R0 32 — lowercase input was never uppercased, so every keyword
;           scan failed and every direct command returned ?0. Fix: split all
;           three labels onto their own lines.
;   v1.10 BUG-SCA-11 FIXED: BDRR/BDRA semantics are rn--; if(rn!=0) branch —
;           exit when rn hits zero (not signed underflow to $FF as previously
;           assumed). All v1.9 BDRR conversions that load a count from memory
;           (bodylen, shift count) are correct because N iterations occur for
;           load value N. Two sites had hardcoded wrong loads:
;           (a) CLRV: load was $33 → only 51 iterations, missing last VARS byte
;               at $01B7. Fix: load $34 for 52 iterations.
;           (b) PU16_M10: load was 9 → only 9 multiplications (off by 1 in
;               every multi-digit number). Fix: load 10 for 10 iterations.
;           Also corrected all BDRR loop comments to say "if R3!=0 branch"
;           instead of "while R3>=0 signed".
;   v1.9  BUG-SCA-01 FIXED: CLRV loop used BRNR,R3 (pure test, no decrement) →
;           infinite loop on startup. R3 never reached zero. Fix: BDRR,R3 with
;           initial load adjusted for BDRR semantics (exits after N+1 iters when
;           loaded with N; load $33 for 52 clears, guard zero case not needed as
;           VARS is always 52 bytes). Label CLRV_NC removed (no longer needed).
;         BUG-SCA-02 FIXED: DO_LIST DLS_BLPX body-print loop used BRNR,R3 →
;           infinite loop printing first byte of every stored line body.
;           Fix: BDRR,R3. R3 loaded from memory; guard COMI,R3 $00 / BCTA,EQ
;           DLS_NL before loop entry retained (BDRR with R3=0 would execute once).
;           Label DLS_BNC removed.
;         BUG-SCA-03 FIXED: DO_RUN DR_CPY copy-to-IBUF loop used BRNR,R3 →
;           infinite loop copying line body. Fix: BDRR,R3.
;           Labels DR_TNC, DR_INC removed.
;         BUG-SCA-04 FIXED: STORE_LINE SL_SHLOOP shift loop used BRNR,R3 →
;           infinite loop during any line insertion that requires shifting.
;           Fix: BDRR,R3. Existing zero-count guard (COMI,R3 / BCTA,EQ SL_NOSHIFT)
;           at loop entry retained (BDRR with R3=0 executes once).
;         BUG-SCA-05 FIXED: STORE_LINE SL_WBODY write-body loop used BRNR,R3 →
;           infinite loop writing body bytes. Fix: BDRR,R3.
;           Labels SL_WBNC, SL_WENC removed.
;         BUG-SCA-06 FIXED: FIND_LINE FL_AS advance loop used BRNR,R3 →
;           infinite loop advancing past body bytes; search never found any line
;           beyond the first record. Fix: BDRR,R3. Label FL_ASN removed.
;         BUG-SCA-07 FIXED: FIND_INS FI_AS advance loop — same as BUG-SCA-06.
;           Fix: BDRR,R3. Label FI_ASN removed.
;         BUG-SCA-08 FIXED: DELETE_LINE DL2_SKIP advance loop used BRNR,R3 →
;           infinite loop; deletion never found copy start. Fix: BDRR,R3.
;           Label DL2_SN removed.
;         BUG-SCA-09 FIXED: MUL16 right-operand abs() NEGFLG toggle was inside
;           the carry path only (BCTA,GT jumped over both the hi-byte increment
;           AND the EORI/STRA toggle). For most negative right values (e.g. -3:
;           abs complement+1 = no carry), NEGFLG was never toggled, giving wrong
;           sign: 3*(-3)=+9 instead of -9. Fix: introduce MU_RA_NC label so the
;           no-carry path skips only the hi-byte increment but falls through to
;           the NEGFLG toggle. Same fix applied to DIV16 DV_VA block (BUG-SCA-09b).
;         BUG-SCA-10 FIXED: PARSE_U16 multiply-by-10 loop used BRNR,R3 with
;           LODI,R3 10 — R3 never decremented, so any number with 2+ digits
;           entered an infinite loop during parsing. Fix: BDRR,R3 with load
;           adjusted to 9 (BDRR gives 10 iterations: 9→8→...→0→exit).
;   v1.8  ISSUE-01 RE-FIX: MUL16/DIV16 NEGFLG placement was still wrong.
;           The LODI,R0 1 / STRA,R0 NEGFLG in v1.7 was placed after the
;           hi-byte carry increment, which is only reached on carry. For
;           values like -3 ($FFFD): XOR→$0002, +1→$0003 — no carry, so
;           BCTA,GT branched past NEGFLG=1 to MU_LA/DV_DA. Sign was lost,
;           result printed positive. Fix: introduce MU_LNC/DV_DNC labels,
;           branch there on carry (skipping hi-byte inc), then BOTH paths
;           fall into LODI,R0 1 / STRA,R0 NEGFLG before MU_LA/DV_DA.
;   v1.7  ISSUE-03 FIXED: DO_GOTO set GOTOFLG=$00 (EORZ/STRA) instead of $01.
;           GOTO was silently ignored during RUN — DR_GOTO path never triggered.
;           Fix: LODI,R0 1 / STRA,R0 GOTOFLG.
;         ISSUE-01 FIXED: MUL16/DIV16 NEGFLG reset bug. The abs(left) block
;           contained EORZ,R0 / STRA,R0 NEGFLG AFTER the two's-complement
;           negation carry propagation step. This unconditionally cleared NEGFLG
;           to zero even after it had been set to 1 for a negative left operand.
;           Net effect: negative×anything gave wrong sign (e.g. -3*2=6 not -6).
;           Fix: replace EORZ/STRA in abs(left) blocks with LODI,R0 1 / STRA,R0
;           NEGFLG in both MUL16 and DIV16.
;         ISSUE-02 FIXED: STORE_LINE shift-dest carry corrupted GOTOH.
;           After ADDA,R0 SC1 / STRA,R0 GOTOL, the carry from the low-byte add
;           was lost when LODA,R0 LNUMH loaded LNUMH (clobbering CC). The
;           subsequent BCTA,GT SL_DSNCA tested LNUMH's sign/zero, not carry.
;           Fix: test carry with BCTA,GT before LODA, duplicate LODA on both
;           paths (carry / no-carry), store GOTOH on both paths.
;         ISSUE-05 FIXED: PARSE_RELOP no-match path returned ERRFLG=$00 (success)
;           when no relop character found. DO_IF proceeded as if relop was valid,
;           using the previous RELOP value — silent wrong comparison.
;           Fix: set ERRFLG=$01 before RETC,UN on the no-match path.
;         ISSUE-04 FIXED: SE_SCAN / SE_C2N table-advance used BCTR,GT (relative
;           short branch) to skip TMPH hi-byte increment, but CC after STRA is
;           set by the stored value not the carry. Replaced with carry-safe idiom:
;           test carry via BCTA,GT before STRA, then branch two paths.
;           Note: practical risk was near-zero (KW_TAB <64B, no page wrap), but
;           corrected for correctness.
;         ISSUE-06 FIXED: Removed redundant second NUL check in RDLINE. After
;           BUG-ASM-08 fix (v1.6), the first NUL check at RL_LP entry catches
;           EOF. The second check after RL_STORE was unreachable dead code.
;   v1.6  BUG-BASIC-14 FIXED: DO_LET variable letter saved to SC0 before
;           PARSE_EXPR, but PARSE_EXPR clobbers SC0 (operator stack writes it
;           repeatedly). DL_STORE read the token id, not the letter. All LET
;           statements wrote to the wrong VARS slot.
;           Fix: STRZ,R2 saves letter to R2 (never written by any routine);
;           DL_STORE restores with LODZ,R2 before computing VARS address.
;           DO_INPUT also updated for consistency.
;         BUG-BASIC-15 FIXED: PF_LOADVAR saved variable letter to SC0, called
;           INC_IP (clobbers R0 with new IPL), then used R0 directly for the
;           VARS offset calculation instead of reloading from SC0.
;           Fix: LODA,R0 SC0 added after INC_IP call.
;         BUG-BASIC-16 FIXED: All 15 indexed VARS/stack accesses used
;           STRA,R1 TMPL after ADDZ,R1. ADDZ,R1 means R0 += R1 (ends-in-Z
;           affects R0); R1 is unchanged. Storing R1 always wrote the base
;           address low byte, not the computed offset. This is the v1.4
;           BUG-BASIC-07 "fix" applied backwards — it swapped R0→R1 but R0
;           is correct (R0 holds the sum after ADDZ). The original code was
;           right; the v1.4 fix broke it. All 15 STRA,R1 TMPL → STRA,R0 TMPL.
;         BUG-ASM-08 FIXED: RDLINE entered infinite NUL loop after stdin EOF.
;           GETKEY returns NUL ($00) forever once stdin is exhausted. RDLINE
;           stored NULs filling IBUF, then overflowed into VARS ($0184+),
;           zeroing variable values set by LET during RUN. This is why LET
;           worked (confirmed by watchpoint at $0185) but the value was then
;           clobbered before PRINT could read it.
;           Fix: added COMI,R0 NUL / BCTA,EQ RL_EOL immediately after GETKEY
;           in RL_LP. NUL from stdin EOF is treated as end-of-line.
;   v1.5  BUG-BASIC-09 FIXED: TRY_STORE_LINE/TSL_DONE cleared ERRFLG to $00
;           after storing a numbered line. REPL checks ERRFLG=$01 to skip
;           execution, so every stored line was immediately executed too.
;           Fix: TSL_DONE sets ERRFLG=$01.
;         BUG-BASIC-10 FIXED: FIND_LINE never set ERRFLG=$01 for "not found".
;           ERRFLG was cleared at entry and never set to $01; FL_RET returned
;           with ERRFLG=$00 (same as "found"), so DELETE_LINE always believed
;           a line existed and corrupted the program store on every STORE_LINE.
;           Fix: FL_RET sets ERRFLG=$01 before returning.
;         BUG-BASIC-11 FIXED: FIND_INS used BCTA,UN FI_RET on both GT and EQ
;           hi-byte compare, making the lo-byte check dead code. Lines sharing
;           the same hi byte (e.g. 10 and 20, both hi=$00) were always inserted
;           at the first record found, corrupting sort order.
;           Fix: BCTA,GT FI_RET so EQ falls through to lo-byte comparison.
;         BUG-BASIC-12 FIXED: DO_LIST called PRINT_S16 without saving TMPH:TMPL.
;           PRINT_S16 loads DIVTAB address into TMPH:TMPL, destroying the LIST
;           iterator. Result: infinite loop printing garbage after first line
;           number. Fix: save/restore TMPH:TMPL via LNUMH:LNUML around call.
;         BUG-BASIC-13 FIXED: DO_RUN saved the next-line pointer in SC0:SC1,
;           but SC0:SC1 are general scratch clobbered by STMT_EXEC (PRINT_S16,
;           STORE_LINE, parser all write SC0/SC1). After executing any line the
;           restored TMPH:TMPL was garbage, causing RUN to jump to a random
;           address. Fix: save next-line pointer in SWSTK[0:1] ($012E:$012F),
;           which are unused until GOSUB is implemented.
;   v1.4  BUG-BASIC-07 (INCORRECTLY FIXED — re-fixed in v1.6 above):
;           Changed STRA,R0 TMPL to STRA,R1 TMPL, believing ADDZ,R1 stored
;           the result in R1. Correct understanding: ADDZ,R1 = R0 += R1.
;   v1.3  BUG-BASIC-03..06, BUG-ASM-04/06/10 fixed (see earlier sessions).
;   v1.2  BUG-BASIC-01: All HI/LO operators corrected (66 swapped lines).
;   v1.1  Initial PIPBUG 1 port.

; ─── ASCII ────────────────────────────────────────────────────────────────────
CR      EQU     $0D
LF      EQU     $0A
BS      EQU     $08
SP      EQU     $20
NUL     EQU     $00
DQ      EQU     $22

; ─── PIPBUG 1 I/O entry points ────────────────────────────────────────────────
COUT    EQU     $02B4   ; putchar: R0 = char to output
CHIN    EQU     $0286   ; getchar: blocking: R0 =  key
CRLF    EQU     $008A   ; print CR+LF (no registers used/changed)

; ─── RAM variables — pinned above code, below PROGLIM ────────────────────────────────────────────────────
; BUG-ASM-10 FIX: Addres $0100 pins variables regardless of code growth.
; Code ceiling: ~$15FF (code must not reach $0100 or assembler will error).
; Variables: $0100-$01B8 (185 bytes). Program store: $01B9-$1BFF (1607 bytes).
IPH     EQU $1600   ; interpreter pointer hi
IPL     EQU $1601   ; interpreter pointer lo
PEH     EQU $1602   ; program end pointer hi
PEL     EQU $1603   ; program end pointer lo
RUNFLG  EQU $1604   ; $01=running $00=immediate
GOTOFLG EQU $1605   ; $01=GOTO/GOSUB pending
GOTOH   EQU $1606   ; pending target line hi
GOTOL   EQU $1607   ; pending target line lo
CURH    EQU $1608   ; current line hi  (error reporting)
CURL    EQU $1609   ; current line lo
LNUMH   EQU $160A   ; scratch line number hi
LNUML   EQU $160B   ; scratch line number lo
SC0     EQU $160C   ; scratch byte 0
SC1     EQU $160D   ; scratch byte 1
ERRFLG  EQU $160E   ; error flag $00=ok $01=error/handled
NEGFLG  EQU $160F   ; sign / CHR$ flag
EXPH    EQU $1610   ; expression result hi
EXPL    EQU $1611   ; expression result lo
TMPH    EQU $1612   ; temp 16-bit hi
TMPL    EQU $1613   ; temp 16-bit lo
OPSTK   EQU $1614   ; operator stack [8]  $0114-$011B
VALSH   EQU $161C   ; value stack hi  [8]  $011C-$0123
VALSL   EQU $1624   ; value stack lo  [8]  $0124-$012B
STKIDX  EQU $162C   ; parser stack top ($FF=empty)
SWSP    EQU $162D   ; SW call stack pointer ($FF=empty)
SWSTK   EQU $162E   ; SW call stack 8×2 bytes  $012E-$013D
PRECTMP EQU $163D   ; PARSE_EXPR cur_prec save (survives APPLY_OP which clobbers SC1)
RELOP   EQU $163E   ; relational op 1-6
CHRFLG  EQU $163F   ; CHR$() output flag ($01=print EXPL as char)
; ── SW call stack (v2.0) ─────────────────────────────────────────────────────
; R3 = index (0=empty, grows up). Each frame = [lo][hi] (lo pushed first).
; Push sequence: STRA,R0 *SWBASE,R3+ (lo first), STRA,R0 *SWBASE,R3+ (hi)
; Pop sequence:  LODA,R0 *SWBASE,R3- (hi first), LODA,R0 *SWBASE,R3- (lo)
; Auto-index on 2650: *base,R3+ = post-increment (write/read then R3++)
;                     *base,R3- = pre-decrement  (R3-- then write/read)
; R3=0 at startup (CLRV loop exits with R3=0). SW stack empty = R3=0.
SWBASE   EQU $1640  ; SW stack base: 32 bytes = 16 frames (2 bytes each)
                    ; $1640-$165F  (16 levels deep minimum per spec)
TEMPRETH EQU $1660  ; SW return address hi (workspace for SWRETURN only)
TEMPRETL EQU $1661  ; SW return address lo
R3SAVE   EQU $1662  ; save/restore R3 when SW routine calls HW routine using R3
IBUF    EQU $1663   ; input buffer 64 bytes  $1663-$16A2
VARS    EQU $1A00   ; A-Z variables 2 bytes each  $1A00-$1A33
PROG    EQU $1A34   ; program store base (VARS+52)
PROGLIM EQU $1c00   ; one past end of program store

; ─── CODE starts at $0440 (after Pipbug 1kB ROM + 64B RAM) ───────────────────
        ORG     $0440

; ─── RESET / ENTRY ────────────────────────────────────────────────────────────
RESET:
        LODI,R0 <PROG
        STRA,R0 PEH
        LODI,R0 >PROG
        STRA,R0 PEL
        LODI,R0 $FF
        STRA,R0 SWSP
        EORZ,R0 ; Clear R0
        STRA,R0 RUNFLG
        STRA,R0 GOTOFLG
        ; clear A-Z variables (52 bytes) using IPH:IPL as scratch pointer
        LODI,R0 <VARS
        STRA,R0 IPH
        LODI,R0 >VARS
        STRA,R0 IPL
; BUG-SCA-01 FIX: was LODI,R3 $34 / BRNR,R3 — BRNR never decrements R3.
; BUG-SCA-11 FIX: BDRR semantics are rn--; if(rn!=0) branch — exits when rn
; hits zero. Load N for exactly N iterations: $34→$33→...→$01→$00→exit = 52.
        LODI,R3 $34             ; 52 iterations: R3 counts $34→$33→...→$01→$00→exit
CLRV:
        EORZ,R0 ; Clear R0
        STRA,R0 *IPH
        BSTA,UN INC_IP
        BDRR,R3 CLRV            ; R3--; if R3!=0 branch
        LODI,R0 <BANNER
        STRA,R0 IPH
        LODI,R0 >BANNER
        STRA,R0 IPL
        BSTA,UN PRTSTR

; ─── REPL ────────────────────────────────────────────────────────────────────
REPL:
        LODI,R0 A'>'
        BSTA,UN COUT
        LODI,R0 SP
        BSTA,UN COUT
        BSTA,UN RDLINE
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
        BSTA,UN TRY_STORE_LINE
        LODA,R0 ERRFLG
        COMI,R0 $01
        BCTR,EQ REPL
        BSTA,UN STMT_EXEC
        BCTR,UN REPL

; ─── TABLES ───────────────────────────────────────────────────────────────────
BANNER:
        DB CR, LF
        DB A'u',A'B',A'A',A'S',A'I',A'C',A' ',A'2',A'6',A'5',A'0',A' ',A'v',A'2',A'.',A'0'
        DB CR, LF, NUL

; Keyword table: [c1][c2][token]  NUL-terminated.
; Matched on first two uppercase chars; EATWORD skips the rest.
; Token 11 (THEN) matched internally by DO_IF — not dispatched here.
; Keyword table: [c1][c2][hi][lo]  NUL-terminated.
; hi:lo = address of handler routine. Matched on first two uppercase chars.
; SE_SCAN loads hi:lo, stores to TMPH:TMPL, branches via *TMPH (indirect jump).
; Token 11 (THEN) matched internally by DO_IF — not dispatched here.
KW_TAB:
        DB A'P',A'R', <DO_PRINT, >DO_PRINT   ; PRINT
        DB A'L',A'E', <DO_LET,   >DO_LET     ; LET
        DB A'L',A'I', <DO_LIST,  >DO_LIST    ; LIST
        DB A'R',A'E', <DO_REM,   >DO_REM     ; REM
        DB A'R',A'U', <DO_RUN,   >DO_RUN     ; RUN
        DB A'E',A'N', <DO_END,   >DO_END     ; END
        DB A'I',A'N', <DO_INPUT, >DO_INPUT   ; INPUT
        DB A'I',A'F', <DO_IF,    >DO_IF      ; IF
        DB A'N',A'E', <DO_NEW,   >DO_NEW     ; NEW
        DB A'G',A'O', <DO_GOTO,  >DO_GOTO    ; GOTO
        DB NUL

DIVTAB:
        DB $27,$10      ; 10000
        DB $03,$E8      ;  1000
        DB $00,$64      ;   100
        DB $00,$0A      ;    10
        DB $00,$00      ; sentinel

; ─── STMT_EXEC ────────────────────────────────────────────────────────────────
; Decode and dispatch one statement from IP.
; RAS depth: 1 from REPL, or 3 from DO_IF (THEN body).
; Worst inner depth from here: +4 (->DO_xxx->PARSE_EXPR->PARSE_FACTOR->UPCASE)
; ─── STMT_EXEC ────────────────────────────────────────────────────────────────
; Decode and dispatch one statement from IP.
; KW_TAB format: [c1][c2][hi][lo] where hi:lo = handler address.
; SE_SCAN advances TMPH:TMPL by 4 per entry; at match loads hi:lo into
; EXPH:EXPL and branches via BCTA,UN *EXPH (absolute indirect jump).
STMT_EXEC:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 NUL
        RETC,EQ                          ; blank line → return

        BSTA,UN GETCI_UC
        STRA,R0 SC0  ; char1 uppercase, IP advanced
        BSTA,UN GETCI_UC
        STRA,R0 SC1  ; char2 uppercase, IP advanced

        ; scan KW_TAB with TMPH:TMPL as pointer
        LODI,R0 <KW_TAB
        STRA,R0 TMPH
        LODI,R0 >KW_TAB
        STRA,R0 TMPL
SE_SCAN:
        LODA,R0 *TMPH                    ; c1
        COMI,R0 NUL
        BCTA,EQ SE_SYNERR                ; end of table
        SUBA,R0 SC0
        BCTR,EQ SE_CHK2
SE_SKIP:
        ; advance 4 bytes to next entry (with 16-bit carry)
        LODA,R0 TMPL
        ADDI,R0 4
        STRA,R0 TMPL
        TPSL $01
        BCTR,LT SE_SCAN                  ; no carry
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
        BCTR,UN SE_SCAN
SE_CHK2:
        BSTA,UN INC_TMP                  ; point to c2 byte
        LODA,R0 *TMPH
        SUBA,R0 SC1
        BCTR,EQ SE_MATCH
        ; c2 mismatch: advance remaining 3 bytes (back to next c1)
        LODA,R0 TMPL
        ADDI,R0 3
        STRA,R0 TMPL
        TPSL $01
        BCTR,LT SE_SCAN
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
        BCTA,UN SE_SCAN
SE_MATCH:
        BSTA,UN EATWORD                  ; [+1] consume remaining alpha chars
        ; load handler address from next 2 bytes: [hi][lo]
        BSTA,UN INC_TMP                  ; point to hi byte
        LODA,R0 *TMPH
        STRA,R0 EXPH                     ; handler hi
        BSTA,UN INC_TMP                  ; point to lo byte
        LODA,R0 *TMPH
        STRA,R0 EXPL                     ; handler lo
        ; Indirect branch: EXPH:EXPL hold the target address.
        ; Store in GOTOH:GOTOL and use BCTA,UN *GOTOH
        LODA,R0 EXPH
        STRA,R0 GOTOH
        LODA,R0 EXPL
        STRA,R0 GOTOL
        BCTA,UN *GOTOH                   ; indirect jump to handler
SE_SYNERR:
        EORZ,R0
        BCTA,UN DO_ERROR

DO_NEW:
        LODI,R0 <PROG
        STRA,R0 PEH
        LODI,R0 >PROG
        STRA,R0 PEL
        ; RETC,UN
        ; drop through
DO_END:
        EORZ,R0 ; Clear R0
        STRA,R0 RUNFLG
        ; drop through
; ─── SIMPLE STATEMENTS ────────────────────────────────────────────────────────
SE_RET:
DO_REM:
        RETC,UN

; ─── DO_PRINT ─────────────────────────────────────────────────────────────────
; PRINT [item {, item}]    item = "string" | expr
; CHR$ flag: NEGFLG=$01 after PARSE_FACTOR detects CHR$ — print EXPL as char.
DO_PRINT:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 NUL ; No opening " so just CRLF
        BCTA,EQ DP_NL

DP_ITEM:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 DQ
        BCTR,EQ DP_STRING
        EORZ,R0 ; Clear R0
        STRA,R0 CHRFLG  ; clear CHR$ flag before parse
        BSTA,UN PARSE_EXPR               ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTR,EQ DP_NUM
        BSTA,UN PRTSTR_IP
        BCTR,UN DP_NL  ; [+1] raw text fallback
DP_NUM:
        LODA,R0 CHRFLG
        COMI,R0 $01
        BCTR,EQ DP_CHAR
        BSTA,UN PRINT_S16
        BCTR,UN DP_SEP  ; [+1]
DP_CHAR:
        LODA,R0 EXPL
        BSTA,UN COUT
        BCTR,UN DP_SEP

DP_STRING:
        ; consume opening "
        BSTA,UN INC_IP
DP_SLP:
        LODA,R1 *IPH
        COMI,R1 NUL
        BCTR,EQ DP_SDONE
        COMI,R1 DQ
        BCTR,EQ DP_SCLS
        LODZ,R1
        BSTA,UN COUT
        BSTA,UN INC_IP
        BCTR,UN DP_SLP
DP_SCLS:
        ; consume closing "
        BSTA,UN INC_IP
DP_SDONE:
DP_SEP:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 $3B              ; ';' = $3B (A';' rejected by WinArcadia assembler)
        BCTR,EQ DP_SEMI  ; semicolon → no space, continue or end without CRLF
        COMI,R0 A','
        BCTR,EQ DP_COMMA
        ; fall through to DP_NL (NUL, or any non-separator = end of PRINT)
DP_NL:
        BSTA,UN CRLF
        RETC,UN
DP_SEMI:
        BSTA,UN INC_IP                   ; consume ';'
DP_SEMI2:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 NUL
        RETC,EQ                          ; trailing ";" → no CRLF, done
        BCTA,UN DP_ITEM                  ; more items follow
DP_COMMA:
        BSTA,UN INC_IP
        BCTA,UN DP_ITEM

; ─── DO_LET / shared store path ───────────────────────────────────────────────
; DO_INPUT jumps to DL_STORE with SC0 = variable letter already set.
DO_LET:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        BSTA,UN UPCASE  ; [+1]
        COMI,R0 A'A'
        BCTR,LT DL_ERR
        COMI,R0 A'Z'+1
        BCTR,LT DL_VAROK
DL_ERR:
        LODI,R0 4
        BCTA,UN DO_ERROR
DL_VAROK:
        STRA,R0 SC0                      ; save variable letter in SC0 (immediate use)
        STRZ,R2                          ; BUG-BASIC-14 FIX: also save in R2 (STRZ stores R0→Rn).
        ; SC0 is general scratch clobbered by PARSE_EXPR (operator-stack ops
        ; write SC0 repeatedly). R2 is never written by any routine and
        ; survives the full PARSE_EXPR call below.
        BSTA,UN INC_IP
DL_EQ:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 A'='
        BCTR,EQ DL_EQC
        EORZ,R0 ; Clear 
        BCTA,UN DO_ERROR
DL_EQC:
        BSTA,UN INC_IP
DL_EX:
        BSTA,UN PARSE_EXPR               ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTR,EQ DL_STORE
        EORZ,R0 ; Clear 
        BCTA,UN DO_ERROR
DL_STORE:
        ; address = VARS + (SC0 - 'A') * 2
        ; BUG-BASIC-14 FIX: restore variable letter from R2 (SC0 was clobbered
        ; by PARSE_EXPR). R2 is caller-saved across PARSE_EXPR (never written
        ; by any routine). DO_INPUT jumps here with letter already in SC0 and R2.
        LODZ,R2                          ; R0 = variable letter (preserved in R2 across PARSE_EXPR)
        STRA,R0 SC0                      ; resync SC0 for any code reading it below
        SUBI,R0 A'A'  ; 0-25
        STRA,R0 SC1
        ADDA,R0 SC1  ; *2  (SC1 = index, R0 = index*2)
        LODI,R1 >VARS
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VARS
        BCTR,GT DL_NC
        ADDI,R0 1
DL_NC:
        STRA,R0 TMPH
        LODA,R0 EXPH
        STRA,R0 *TMPH  ; store hi
        BSTA,UN INC_TMP
DL_NC2:
        LODA,R0 EXPL
        STRA,R0 *TMPH  ; store lo
        RETC,UN

; ─── DO_INPUT ─────────────────────────────────────────────────────────────────
DO_INPUT:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        BSTA,UN UPCASE  ; [+1]
        COMI,R0 A'A'
        BCTR,LT DIN_ERR
        COMI,R0 A'Z'+1
        BCTR,LT DIN_VAROK
DIN_ERR:
        LODI,R0 4
        BCTA,UN DO_ERROR
DIN_VAROK:
        STRA,R0 SC0                      ; save variable letter
        STRZ,R2                          ; also save in R2 for DL_STORE (SC0 clobbered by PARSE_S16)
        BSTA,UN INC_IP
DIN_PR:
        LODI,R0 A'?'
        BSTA,UN COUT
        LODI,R0 SP
        BSTA,UN COUT
        BSTA,UN RDLINE                   ; [+1]
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
        BSTA,UN PARSE_S16                ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DL_STORE
        EORZ,R0 ; Clear 
        BCTA,UN DO_ERROR

; ─── DO_IF ────────────────────────────────────────────────────────────────────
; IF expr relop expr THEN stmt
; Depth at entry: 2 (from REPL->STMT_EXEC) or 4 (from REPL->STMT_EXEC->DO_IF->STMT_EXEC->here)
; After THEN: calls STMT_EXEC at +1, which can call DO_xxx at +1, PARSE_EXPR at +1,
;             PARSE_FACTOR at +1 → max total 2+1+1+1+1+1 = depth 7 OK.
DO_IF:
        BSTA,UN PARSE_EXPR               ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTR,EQ DIF_LS
        EORZ,R0 ; Clear 
        BCTA,UN DO_ERROR
DIF_LS:
        LODA,R0 EXPH
        STRA,R0 LNUMH  ; BUG-T6 FIX: save left in LNUMH:LNUML (TMPH:TMPL clobbered
        LODA,R0 EXPL   ;   by PARSE_EXPR's PX_PUSHV writing <VALSH/$15 to TMPH)
        STRA,R0 LNUML
        BSTA,UN PARSE_RELOP              ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTR,EQ DIF_RP
        EORZ,R0 ; Clear 
        BCTA,UN DO_ERROR
DIF_RP:
        BSTA,UN PARSE_EXPR               ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTR,EQ DIF_EVAL
        EORZ,R0 ; Clear 
        BCTA,UN DO_ERROR
DIF_EVAL:
        ; signed 16-bit compare: LNUMH:LNUML (left) vs EXPH:EXPL (right)
        ; bias hi bytes by XOR $80 → unsigned compare
        LODA,R0 LNUMH
        EORI,R0 $80
        STRA,R0 SC0
        LODA,R0 EXPH
        EORI,R0 $80
        SUBA,R0 SC0             ; biased right.hi - biased left.hi
        BCTR,LT DIF_LT
        BCTR,GT DIF_GT
        ; hi bytes equal: compare lo (unsigned)
        LODA,R0 EXPL
        SUBA,R0 LNUML
        BCTR,LT DIF_LT
        BCTR,GT DIF_GT
        EORZ,R0 ; Clear R0
        STRA,R0 SC1
        BCTR,UN DIF_TH  ; EQ
DIF_LT:
        LODI,R0 $01          ; right-hi < left-hi: left > right → SC1=$01
        STRA,R0 SC1
        BCTR,UN DIF_TH  ; LT (result: left > right)
DIF_GT:
        LODI,R0 $FF          ; right-hi > left-hi: left < right → SC1=$FF
        STRA,R0 SC1  ; GT (result: left < right)

DIF_TH:
        ; consume THEN keyword: expect T then H then EATWORD
        BSTA,UN WSKIP                    ; [+1]
        BSTA,UN GETCI_UC                 ; [+1]  must be A'T'
        COMI,R0 A'T'
        BCTR,EQ DIF_TH2
        EORZ,R0 ; Clear 
        BCTA,UN DO_ERROR
DIF_TH2:
        BSTA,UN GETCI_UC                 ; [+1]  must be A'H'
        COMI,R0 A'H'
        BCTR,EQ DIF_EW
        EORZ,R0 ; Clear 
        BCTA,UN DO_ERROR
DIF_EW:
        BSTA,UN EATWORD                  ; [+1]

        ; BUG-RELOP-02 FIX: TMI,R0 RELOP was wrong — TMI uses RELOP as an
        ; immediate byte (not a runtime RAM read), always assembling as $00.
        ; Fix: map SC1 result to a bitmask in R0, then AND against RELOP at runtime.
        ;   SC1=$FF → LT → bit 0 ($01)
        ;   SC1=$00 → EQ → bit 1 ($02)
        ;   SC1=$01 → GT → bit 2 ($04)
        ; ANDZ,R1: R0 &= R1.  If result=0: no match → false.
        LODA,R0 SC1
        COMI,R0 $FF
        BCTR,EQ DIF_IS_LT
        COMI,R0 $00
        BCTR,EQ DIF_IS_EQ
        LODI,R0 4                        ; GT → bit 2
        BCTR,UN DIF_ANDTEST
DIF_IS_LT:
        LODI,R0 1                        ; LT → bit 0
        BCTR,UN DIF_ANDTEST
DIF_IS_EQ:
        LODI,R0 2                        ; EQ → bit 1
DIF_ANDTEST:
        LODA,R1 RELOP                    ; R1 = runtime bitmask from RAM
        ANDZ,R1                          ; R0 &= R1  (ANDZ,rn: R0 &= rn)
        BCTR,EQ DIF_FALSE                ; zero → no bit match → condition false
        ; fall through to DIF_TRUE
        BSTA,UN STMT_EXEC                ; [+1]  execute THEN body
        RETC,UN
DIF_FALSE:
        RETC,UN

DO_GOTO:
        BSTA,UN WSKIP                    ; [+1] RAS-FIX: PARSE_U16 no longer calls WSKIP
        BSTA,UN PARSE_U16                ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTR,EQ DG_OK
        EORZ,R0 ; Clear 
        BCTA,UN DO_ERROR
DG_OK:
        LODA,R0 EXPH
        STRA,R0 GOTOH
        LODA,R0 EXPL
        STRA,R0 GOTOL
        LODI,R0 1               ; ISSUE-03 FIX: was EORZ/STRA ($00) — must be $01
        STRA,R0 GOTOFLG
        LODA,R0 RUNFLG
        COMI,R0 $01
        ;BCTR,EQ DG_RET
        RETC,EQ
        EORZ,R0 ; Clear R0
        STRA,R0 RUNFLG  ; start run if in immediate mode
DG_RET:
        RETC,UN


DO_LIST:
        LODI,R0 <PROG
        STRA,R0 TMPH
        LODI,R0 >PROG
        STRA,R0 TMPL
DLS_LP:
        ; unsigned 16-bit: if TMPH:TMPL >= PEH:PEL → done
        ; Use COMZ via unsigned COM mode for hi byte, carry check for lo byte
        LODA,R0 TMPH
        PPSL $02                ; unsigned compare mode
        COMI,R0 $00
        STRA,R0 SC0
        LODA,R0 PEH
        COMI,R0 $00
        STRA,R0 SC1
        LODA,R0 SC0
        COMA,R0 SC1             ; unsigned: TMPH vs PEH
        CPSL $02
        RETC,GT                 ; TMPH > PEH → past end
        BCTR,LT DLS_BODY        ; TMPH < PEH → before end
        ; TMPH == PEH: check lo byte carry from subtraction
        LODA,R0 TMPL
        SUBA,R0 PEL             ; TMPL - PEL (signed sub but carry=1 means >=)
        TPSL $01                ; C=1 → no borrow → TMPL >= PEL → at/past end
        RETC,EQ                 ; CC=EQ means C=1 → TMPL >= PEL → done
DLS_BODY:
DLS_BODY:
        ; line number hi:lo
        LODA,R0 *TMPH
        STRA,R0 EXPH
        BSTA,UN INC_TMP
DLS_N1:
        LODA,R0 *TMPH
        STRA,R0 EXPL
        BSTA,UN INC_TMP
DLS_N2:
        ; BUG-BASIC-12 FIX: PRINT_S16 clobbers TMPH:TMPL (loads DIVTAB ptr).
        ; Save TMPH:TMPL in LNUMH:LNUML and restore after the call.
        LODA,R0 TMPH
        STRA,R0 LNUMH
        LODA,R0 TMPL
        STRA,R0 LNUML
        BSTA,UN PRINT_S16                ; [+1]
        LODA,R0 LNUMH
        STRA,R0 TMPH
        LODA,R0 LNUML
        STRA,R0 TMPL
        LODI,R0 SP
        BSTA,UN COUT
DLS_N3:
        ; print body bytes until CR (CR-terminated format)
DLS_BLPX:
        LODA,R0 *TMPH
        COMI,R0 CR
        BCTR,EQ DLS_NL
        BSTA,UN COUT
        BSTA,UN INC_TMP
        BCTR,UN DLS_BLPX
DLS_NL:
        BSTA,UN INC_TMP          ; skip past CR
        BSTA,UN CRLF
        BCTA,UN DLS_LP
DLS_RET:
        ; RETC,UN

; ─── DO_RUN ───────────────────────────────────────────────────────────────────
; Executes stored lines sequentially, honouring GOTOFLG for GOTO/GOSUB/RETURN.
; SC0:SC1 = next-line-pointer saved BEFORE STMT_EXEC so DO_GOSUB can read it.
DO_RUN:
        LODI,R0 1 
        STRA,R0 RUNFLG
        EORZ,R0 ; Clear R0
        STRA,R0 GOTOFLG
        LODI,R0 <PROG
        STRA,R0 TMPH
        LODI,R0 >PROG
        STRA,R0 TMPL
DR_LP:
        LODA,R0 RUNFLG
        COMI,R0 $00
        ; BCTA,EQ DR_RET
        RETC,EQ
        ; end of program? unsigned 16-bit: TMPH:TMPL >= PEH:PEL → stop
        LODA,R0 TMPH
        PPSL $02
        COMI,R0 $00
        STRA,R0 SC0
        LODA,R0 PEH
        COMI,R0 $00
        STRA,R0 SC1
        LODA,R0 SC0
        COMA,R0 SC1             ; unsigned TMPH vs PEH
        CPSL $02
        BCTA,GT DR_STOP          ; TMPH > PEH → past end
        BCTR,LT DR_EXEC          ; TMPH < PEH → before end
        ; TMPH == PEH: lo byte
        LODA,R0 TMPL
        SUBA,R0 PEL
        TPSL $01                ; C=1 → no borrow → TMPL >= PEL → at/past end
        RETC,EQ                 ; done if TMPL >= PEL
        BCTA,UN DR_EXEC
DR_EXEC:
        ; save line number for error reporting
        LODA,R0 *TMPH
        STRA,R0 CURH
        BSTA,UN INC_TMP
DR_N1:
        LODA,R0 *TMPH
        STRA,R0 CURL
        BSTA,UN INC_TMP
DR_N2:
DR_N3:
        ; copy body to IBUF until CR (CR-terminated format), NUL-terminate
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
DR_CPY:
        LODA,R1 *TMPH
        COMI,R1 CR
        BCTR,EQ DR_CD
        STRA,R1 *IPH
        BSTA,UN INC_TMP
        BSTA,UN INC_IP
        BCTR,UN DR_CPY
DR_CD:
        BSTA,UN INC_TMP          ; skip past CR in store
        LODI,R1 NUL
        STRA,R1 *IPH  ; NUL-terminate IBUF
        ; BUG-BASIC-13 FIX: Save next-line pointer in SWSTK[0:1] instead of
        ; SC0:SC1. SC0 and SC1 are scratch bytes clobbered by STMT_EXEC (used
        ; by PRINT_S16, STORE_LINE, parser, etc.).  SWSTK is the GOSUB return
        ; stack, indexed from the top; [0:1] at $012E:$012F are unused while
        ; SWSP=$FF (empty) and GOSUB is not yet implemented.
        LODA,R0 TMPH
        STRA,R0 SC0      ; SC0:SC1 still set (DO_GOSUB reads them for return addr)
        ; BUG-SCA-12 FIX: was STRA,R0 *SWSTK — indirect addressing writes to the
        ; address stored AT SWSTK ($012E:$012F), not into SWSTK itself. After CLRV
        ; SWSTK contains $0000, so the next-line pointer hi byte was written into
        ; PIPBUG ROM at $0000. Fix: direct STRA,R0 SWSTK stores into $012E.
        STRA,R0 SWSTK    ; NLP_H: save hi byte of next-line ptr directly into $012E
        LODA,R0 TMPL
        STRA,R0 SC1
        LODA,R0 TMPL
        STRA,R0 SWSTK+1  ; NLP_L: save lo byte directly into $012F
        ; execute line
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
        BSTA,UN STMT_EXEC                ; [+1]
        ; check GOTO/GOSUB/RETURN flag
        LODA,R0 GOTOFLG
        COMI,R0 $01
        BCTR,EQ DR_GOTO
        ; advance: restore next-line pointer from SWSTK[0:1] (SC0:SC1 clobbered)
        ; BUG-SCA-12 FIX: was LODA,R0 *SWSTK (indirect). Direct read from $012E.
        LODA,R0 SWSTK
        STRA,R0 TMPH
        LODA,R0 SWSTK+1
        STRA,R0 TMPL
        BCTA,UN DR_LP
DR_GOTO:
        EORZ,R0 ; Clear R0
        STRA,R0 GOTOFLG
        LODA,R0 GOTOH
        STRA,R0 LNUMH
        LODA,R0 GOTOL
        STRA,R0 LNUML
        BSTA,UN FIND_LINE                ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DR_LP
        LODI,R0 1
        BSTA,UN DO_ERROR  ; [+1] undefined line — returns to REPL
        BCTA,UN DR_LP
DR_STOP:
        EORZ,R0 ; Clear R0
        STRA,R0 RUNFLG
DR_RET:
        RETC,UN

; ─── TRY_STORE_LINE ───────────────────────────────────────────────────────────
; If IP starts with a digit, parse and store/delete the numbered line.
; Returns ERRFLG=$01 if handled as a numbered line, $00 if immediate.
TRY_STORE_LINE:
        EORZ,R0 ; Clear R0
        STRA,R0 ERRFLG
        LODA,R0 *IPH
        COMI,R0 A'0'
        ; BCTR,LT TSL_RET
        RETC,LT
        COMI,R0 A'9'+1
        BCTR,LT TSL_NUM
TSL_RET:
        RETC,UN
TSL_NUM:
        BSTA,UN WSKIP                    ; [+1] RAS-FIX: PARSE_U16 no longer calls WSKIP
        BSTA,UN PARSE_U16                ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTR,EQ TSL_GOT
        ;BCTR,UN TSL_RET
        RETC,UN
TSL_GOT:
        ; validate 1..32767
        LODA,R0 EXPH
        ANDI,R0 $80
        BCTR,EQ TSL_RNG
        EORZ,R0 ; Clear R0
        STRA,R0 ERRFLG
        RETC,UN  ; >=32768 silently ignore
TSL_RNG:
        LODA,R0 EXPH
        COMI,R0 $00
        BCTR,GT TSL_NZ
        LODA,R0 EXPL
        COMI,R0 $00
        ; BCTA,EQ TSL_RET2  ; line 0 invalid
        RETC,EQ
TSL_NZ:
        LODA,R0 EXPH
        STRA,R0 LNUMH
        LODA,R0 EXPL
        STRA,R0 LNUML
        BSTA,UN WSKIP                    ; [+1]  skip space after line number
        LODA,R0 *IPH
        COMI,R0 NUL
        BCTR,EQ TSL_DEL
        BSTR,UN STORE_LINE               ; [+1]
        BCTR,UN TSL_DONE
TSL_DEL:
        BSTA,UN DELETE_LINE              ; [+1]
TSL_DONE:
        LODI,R0 1               ; BUG-BASIC-09 FIX: $01 = "line stored, skip exec"
        STRA,R0 ERRFLG
TSL_RET2:
        RETC,UN

; ─── STORE_LINE ───────────────────────────────────────────────────────────────
; Insert line LNUMH:LNUML with body at IP into program store (sorted).
; Record format: [linehi][linelo][body...][CR]
; Strategy: delete existing line, measure body, check space, find insertion
;           point (EXPH:EXPL), shift existing records up, write new record.
STORE_LINE:
        BSTA,UN DELETE_LINE              ; [+1]  remove if exists

        ; save body start, then measure length via TMPH:TMPL (preserves IP)
        LODA,R0 IPH
        STRA,R0 TMPH
        LODA,R0 IPL
        STRA,R0 TMPL  ; TMPH:TMPL = body start
        LODI,R3 0
SL_MEAS:
        LODA,R0 *TMPH
        COMI,R0 NUL
        BCTR,EQ SL_MEASD
        BSTA,UN INC_TMP
SL_MNC:
        BIRR,R3 SL_MEAS         ; R3++ always (counts: 0→1→2...)
SL_MEASD:
        ; R3 = body length.  SC0 = body len.  SC1 = record size = 2+bodylen+1 (hi:lo:body:CR).
        STRA,R3 SC0
        LODA,R0 SC0
        ADDI,R0 3
        STRA,R0 SC1
        ; TMPH:TMPL already at body start — restore for space-check then write

        ; check free space: PROGLIM - PE >= SC1 (scratch: CURH:CURL — safe during entry)
        LODI,R0 >PROGLIM
        SUBA,R0 PEL
        STRA,R0 CURL
        LODI,R0 <PROGLIM
        SUBA,R0 PEH
        BCFR,LT SL_NBC
        SUBI,R0 1
SL_NBC:
        STRA,R0 CURH            ; CURH:CURL = free bytes (LNUMH:LNUML preserved)
        LODA,R0 CURH
        COMI,R0 $00
        BCTR,GT SL_ROOM
        LODA,R0 CURL
        SUBA,R0 SC1
        BCFR,LT SL_ROOM  ; free >= needed?
        LODI,R0 3
        BCTA,UN DO_ERROR  ; out of memory

SL_ROOM:
        ; find sorted insertion point (FIND_INS clobbers TMPH:TMPL — that is fine,
        ; body start is in IP which survives, line number is in LNUMH:LNUML)
        BSTA,UN FIND_INS                 ; [+1]  result → TMPH:TMPL
        ; save insertion point in EXPH:EXPL
        LODA,R0 TMPH
        STRA,R0 EXPH
        LODA,R0 TMPL
        STRA,R0 EXPL
        ; save line number to CURH:CURL — shift loop clobbers LNUMH:LNUML
        LODA,R0 LNUMH
        STRA,R0 CURH
        LODA,R0 LNUML
        STRA,R0 CURL
        ; shift bytes PE-1 down to EXPH:EXPL upward by SC1 positions (backwards copy)
        ; (body pointer reloaded from IP at SL_NOSHIFT after shift completes)
        ; shift count = PE - EXPH:EXPL
        LODA,R0 PEL
        SUBA,R0 EXPL
        STRA,R0 TMPL
        LODA,R0 PEH
        SUBA,R0 EXPH
        BCFR,LT SL_SHCNB
        SUBI,R0 1
SL_SHCNB:
        STRA,R0 TMPH            ; TMPH:TMPL = shift count

        ; if shift count == 0 skip loop
        LODA,R0 TMPH
        COMI,R0 $00
        BCTR,GT SL_DOSHIFT
        LODA,R0 TMPL
        COMI,R0 $00
        BCTA,EQ SL_NOSHIFT
SL_DOSHIFT:
        ; src = PE-1 in LNUMH:LNUML (shift uses these as src pointer)
        LODA,R0 PEL
        SUBI,R0 1
        STRA,R0 LNUML
        LODA,R0 PEH
        BCFR,LT SL_SNBR
        SUBI,R0 1
SL_SNBR:
        STRA,R0 LNUMH           ; LNUMH:LNUML = src = PE-1
        ; dst = src + SC1 (record size = shift amount)
        ; ISSUE-02 FIX: must test carry from ADDA before any LODA clobbers CC.
        ; Old code did STRA / LODA LNUMH / BCTA,GT — LODA wiped the carry.
        ; New code: test carry immediately after ADDA, then load LNUMH on both paths.
        LODA,R0 LNUML
        ADDA,R0 SC1
        STRA,R0 GOTOL
        TPSL $01                 ; BUG-SCA-14 FIX: carry from lo-byte add
        BCTR,LT SL_DSNCA         ; branch if C=0 (no carry)
        LODA,R0 LNUMH           ; carry path: hi += 1
        ADDI,R0 1
        STRA,R0 GOTOH
        BCTR,UN SL_DSNCB
SL_DSNCA:
        LODA,R0 LNUMH           ; no-carry path: hi unchanged
        STRA,R0 GOTOH
SL_DSNCB:

        ; use R3 as count (shift count lo; assume <256 for any real program)
        ; BUG-SCA-04 FIX: was BRNR,R3 at loop end — R3 never decremented → infinite shift.
        ; Guard zero case first (BDRR with R3=0 would execute once wrongly).
        LODA,R3 TMPL
SL_SHLOOP:
        COMI,R3 $00
        BCTR,EQ SL_NOSHIFT
        ; read from LNUMH:LNUML
        LODA,R1 *LNUMH
        ; write to GOTOH:GOTOL
        STRA,R1 *GOTOH
        ; decrement both pointers
        LODA,R0 LNUML
        SUBI,R0 1
        STRA,R0 LNUML
        BCFR,LT SL_SRNB
        LODA,R0 LNUMH
        SUBI,R0 1
        STRA,R0 LNUMH
SL_SRNB:
        LODA,R0 GOTOL
        SUBI,R0 1
        STRA,R0 GOTOL
        BCFR,LT SL_DRNB
        LODA,R0 GOTOH
        SUBI,R0 1
        STRA,R0 GOTOH
SL_DRNB:
        BDRR,R3 SL_SHLOOP       ; R3--; if R3!=0 branch

SL_NOSHIFT:
        ; write record at EXPH:EXPL (insertion point)
        ; Restore line number (clobbered by shift using LNUMH:LNUML as src ptr)
        LODA,R0 CURH
        STRA,R0 LNUMH
        LODA,R0 CURL
        STRA,R0 LNUML
        ; Reload body start from IP (IP preserved across shift; TMPH:TMPL clobbered)
        LODA,R0 IPH
        STRA,R0 TMPH
        LODA,R0 IPL
        STRA,R0 TMPL
        LODA,R0 LNUMH
        STRA,R0 *EXPH  ; write line hi
        BSTA,UN INC_EXP
SL_WN1:
        LODA,R0 LNUML
        STRA,R0 *EXPH  ; write line lo
        BSTA,UN INC_EXP
SL_WN2:
        ; write body bytes until NUL (CR-terminated format — no bodylen byte)
SL_WBODY:
        LODA,R1 *TMPH
        COMI,R1 NUL
        BCTR,EQ SL_WDONE
        STRA,R1 *EXPH
        BSTA,UN INC_TMP
        BSTA,UN INC_EXP
        BCTR,UN SL_WBODY
SL_WDONE:
        LODI,R0 CR
        STRA,R0 *EXPH  ; write CR terminator
        BSTA,UN INC_EXP
        ; update PE += SC1 (record size)
        LODA,R0 PEL
        ADDA,R0 SC1
        STRA,R0 PEL
        TPSL $01                 ; carry from lo-byte add
        RETC,LT                  ; C=0 (no carry) → done
        LODA,R0 PEH
        ADDI,R0 1
        STRA,R0 PEH
        RETC,UN

; ─── DELETE_LINE ──────────────────────────────────────────────────────────────
DELETE_LINE:
        BSTA,UN FIND_LINE                ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTR,EQ DL2_FOUND
        RETC,UN
DL2_FOUND:
        ; record start in TMPH:TMPL.  CR-format: size = scan from +2 until CR + 3.
        LODA,R0 TMPH
        STRA,R0 EXPH  ; save record start in EXPH:EXPL
        LODA,R0 TMPL
        STRA,R0 EXPL
        ; advance TMPH:TMPL by 2 (skip linehi, linelo)
        LODA,R0 TMPL
        ADDI,R0 2
        STRA,R0 TMPL
        TPSL $01
        BCTR,LT DL2_BLN
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
DL2_BLN:
        ; scan body until CR to find record end; SC0 = record size
        LODI,R0 3                ; start at 3 (hi + lo + CR byte itself)
        STRA,R0 SC0
DL2_SCAN:
        LODA,R1 *TMPH
        COMI,R1 CR
        BCTR,EQ DL2_SCAND
        BSTA,UN INC_TMP
        LODA,R0 SC0
        ADDI,R0 1
        STRA,R0 SC0
        BCTR,UN DL2_SCAN
DL2_SCAND:
        BSTA,UN INC_TMP          ; skip the CR byte itself
DL2_COPY:
        ; copy TMPH:TMPL..PE-1 to EXPH:EXPL
DL2_LP:
        LODA,R0 TMPH
        SUBA,R0 PEH                      ; signed OK: PEH=$1A < $80
        BCTR,GT DL2_DONE
        BCTR,LT DL2_MOV
        ; TMPH == PEH: unsigned lo via carry
        LODA,R0 TMPL
        SUBA,R0 PEL
        TPSL $01                         ; C=1 → TMPL >= PEL → done
        BCTR,EQ DL2_DONE
        BCTR,UN DL2_MOV
DL2_MOV:
        LODA,R1 *TMPH
        STRA,R1 *EXPH
        BSTA,UN INC_TMP
DL2_TNC:
        BSTA,UN INC_EXP
DL2_ENC:
        BCTR,UN DL2_LP
DL2_DONE:
        ; PE -= SC0
        LODA,R0 PEL
        SUBA,R0 SC0
        STRA,R0 PEL
        TPSL $01                 ; CC=EQ if C=1 (no borrow), CC=LT if C=0 (borrow)
        RETC,EQ                  ; no borrow → done
        LODA,R0 PEH
        SUBI,R0 1
        STRA,R0 PEH
        RETC,UN

; ─── FIND_LINE ────────────────────────────────────────────────────────────────
; Search for line LNUMH:LNUML in program store (sorted ascending).
; Returns: TMPH:TMPL = record start if found; ERRFLG=$00 found / $01 not found.
; Calls FIND_INS to locate position, then checks if it is an exact match.
FIND_LINE:
        BSTA,UN FIND_INS                 ; [+1] sets TMPH:TMPL to insertion point
        ; Check if at end of program (no match possible)
        LODA,R0 TMPH
        SUBA,R0 PEH                      ; signed OK: PEH=$1A always < $80
        BCTR,GT FL_RET_NF
        BCTR,LT FL_CHK
        ; TMPH == PEH: unsigned lo comparison via carry
        LODA,R0 TMPL
        SUBA,R0 PEL
        TPSL $01                         ; C=1 → no borrow → TMPL >= PEL
        BCTR,EQ FL_RET_NF               ; TMPL >= PEL → at/past end (C=1, CC=EQ)
        BCTR,UN FL_CHK                   ; TMPL < PEL → check record
FL_CHK:
        ; Check exact match: *TMPH == LNUMH and *(TMPH:TMPL+1) == LNUML
        LODA,R0 *TMPH
        SUBA,R0 LNUMH
        BCTR,EQ FL_CHKLO
FL_RET_NF:
        LODI,R0 1
        STRA,R0 ERRFLG
        RETC,UN
FL_CHKLO:
        LODA,R0 TMPL
        ADDI,R0 1
        STRA,R0 EXPL
        LODA,R0 TMPH
        TPSL $01
        RETC,LT                          ; no carry → EXPH = TMPH
        ADDI,R0 1
FL_LH:
        STRA,R0 EXPH
        LODA,R0 *EXPH
        SUBA,R0 LNUML
        BCTR,EQ FL_FOUND
        BCTR,UN FL_RET_NF
FL_FOUND:
        EORZ,R0
        STRA,R0 ERRFLG
        RETC,UN

; ─── FIND_INS ─────────────────────────────────────────────────────────────────
; Find sorted insertion point for LNUMH:LNUML.
; Returns TMPH:TMPL = address of first record with line >= LNUMH:LNUML,
; or PEH:PEL if all lines are smaller (insert at end).
FIND_INS:
        LODI,R0 <PROG
        STRA,R0 TMPH
        LODI,R0 >PROG
        STRA,R0 TMPL
FI_LP:
        ; boundary check: TMPH:TMPL >= PEH:PEL → done (unsigned)
        LODA,R0 TMPH
        SUBA,R0 PEH                      ; signed OK: PEH=$1A always < $80
        RETC,GT
        BCTR,LT FI_CHK
        ; TMPH == PEH: unsigned lo via carry
        LODA,R0 TMPL
        SUBA,R0 PEL
        TPSL $01                         ; C=1 → no borrow → TMPL >= PEL → done
        RETC,EQ                          ; C=1 → at/past end
FI_CHK:
        LODA,R0 *TMPH
        SUBA,R0 LNUMH
        BCTR,LT FI_ADV
        RETC,GT                          ; stored.hi > target → insertion point found
        ; hi bytes equal: check lo
        LODA,R0 TMPL
        ADDI,R0 1
        STRA,R0 EXPL
        LODA,R0 TMPH
        TPSL $01
        BCTR,LT FI_LH
        ADDI,R0 1
FI_LH:
        STRA,R0 EXPH
        LODA,R0 *EXPH
        SUBA,R0 LNUML
        RETC,GT                          ; stored.lo >= target lo → insertion point
        RETC,EQ
FI_ADV:
        ; advance TMPH:TMPL past record: skip hi+lo then scan body until CR
        LODA,R0 TMPL
        ADDI,R0 2
        STRA,R0 TMPL
        TPSL $01
        BCTR,LT FI_AN
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
FI_AN:
        ; scan body bytes until CR
FI_AS:
        LODA,R0 *TMPH
        COMI,R0 CR
        BCTR,EQ FI_DONE
        BSTA,UN INC_TMP
        BCTR,UN FI_AS
FI_DONE:
        BSTA,UN INC_TMP          ; skip the CR itself
        BCTA,UN FI_LP


; ─── PARSE_EXPR ───────────────────────────────────────────────────────────────
; Shunting-yard iterative operator-precedence parser.
; Entry: IP at expression.  Exit: EXPH:EXPL = result, ERRFLG=$00.
; RAS budget: this routine is at depth N; calls PARSE_FACTOR at N+1.
; Max depth from caller: +2. PARSE_FACTOR may call PARSE_EXPR for functions
; at N+1+1 = N+2 total extra levels — tight at deepest path, see ARCH §12.
;
; OPSTK[0..STKIDX]: operator stack   '(' = sentinel (prec 0)
; VALSH/VALSL[0..STKIDX]: value stack
;
; Operator precedences: '('=0 (sentinel, never reduces), '+''-'=1, '*''/'=2
; Reduction: while top-op-prec >= cur-op-prec AND top-op != '(': apply top op
PARSE_EXPR:
        LODI,R0 $FF
        STRA,R0 STKIDX
        EORZ,R0 ; Clear R0
        STRA,R0 ERRFLG

PX_ATOM:
        ; skip spaces then parse one atom (number, variable, unary, paren)
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 A'('
        BCTR,EQ PX_LPAR
        COMI,R0 A'-'
        BCTR,EQ PX_UNEG
        COMI,R0 A'+'
        BCTA,EQ PX_UPOS
        BSTA,UN PARSE_FACTOR             ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ PX_PUSHV
        RETC,UN

PX_LPAR:
        ; push '(' sentinel onto OPSTK
        BSTA,UN INC_IP
PX_LPN:
        LODA,R0 STKIDX
        ADDI,R0 1
        STRA,R0 STKIDX
        LODI,R1 >OPSTK
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <OPSTK
        BCTR,GT PX_LPNCA
        ADDI,R0 1
PX_LPNCA:
        STRA,R0 TMPH
        LODI,R0 A'('
        STRA,R0 *TMPH
        BCTR,UN PX_ATOM

PX_UNEG:
        ; consume '-', parse factor, negate result
        BSTA,UN INC_IP
PX_UNN:
        BSTA,UN PARSE_FACTOR             ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTR,EQ PX_NEG
        RETC,UN
PX_NEG:
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
        BCTR,UN PX_PUSHV

PX_UPOS:
        ; consume '+', parse factor — result unchanged
        BSTA,UN INC_IP
PX_UPN:
        BSTA,UN PARSE_FACTOR             ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTR,EQ PX_PUSHV
        RETC,UN

PX_PUSHV:
        ; push EXPH:EXPL to value stack at STKIDX+1
        LODA,R0 STKIDX
        ADDI,R0 1
        STRA,R0 STKIDX
        LODI,R1 >VALSH
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSH
        BCTR,GT PX_VHN
        ADDI,R0 1
PX_VHN:
        STRA,R0 TMPH
        LODA,R0 EXPH
        STRA,R0 *TMPH
        LODA,R0 STKIDX
        LODI,R1 >VALSL
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSL
        BCTR,GT PX_VLN
        ADDI,R0 1
PX_VLN:
        STRA,R0 TMPH
        LODA,R0 EXPL
        STRA,R0 *TMPH

PX_PEEKOP:
        ; peek next char for operator
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH

        COMI,R0 A')'
        BCTA,EQ PX_RPAR  ; A')' → reduce until A'(' sentinel

        BSTA,UN GET_PREC                 ; [+1]  R0 = prec(cur op)  ; 0=not an op
        COMI,R0 $00
        BCTA,EQ PX_RALL  ; end of expression → reduce all
        STRA,R0 PRECTMP                      ; PRECTMP = cur op prec (SC1 is scratch for APPLY_OP)

PX_REDLP:
        ; while STKIDX >= 1 and top-op-prec >= SC1: reduce
        LODA,R0 STKIDX
        COMI,R0 $00
        BCTR,EQ PX_PUSHOP  ; only 1 value
        ; get top op from OPSTK[STKIDX-1]
        SUBI,R0 1
        LODI,R1 >OPSTK
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <OPSTK
        BCTR,GT PX_TOPNC
        ADDI,R0 1
PX_TOPNC:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 SC0  ; SC0 = top op byte
        COMI,R0 A'('
        BCTR,EQ PX_PUSHOP  ; sentinel → stop reducing
        BSTA,UN GET_PREC_SC0             ; [+1]  R0 = prec(SC0)
        SUBA,R0 PRECTMP                      ; top_prec - cur_prec (SC1 clobbered by APPLY_OP, use PRECTMP)
        BCTR,LT PX_PUSHOP                ; top_prec < cur_prec → push new op
        BSTA,UN APPLY_OP                 ; [+1]  reduce top pair
        BCTR,UN PX_REDLP

PX_PUSHOP:
        ; push cur op byte onto OPSTK[STKIDX] and consume from IP
        LODA,R0 *IPH
        STRA,R0 SC0
        BSTA,UN INC_IP
PX_PON:
        LODA,R0 STKIDX
        LODI,R1 >OPSTK
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <OPSTK
        BCTR,GT PX_OPN
        ADDI,R0 1
PX_OPN:
        STRA,R0 TMPH
        LODA,R0 SC0
        STRA,R0 *TMPH
        BCTA,UN PX_ATOM                  ; parse next value

PX_RPAR:
        ; consume ')'
        BSTA,UN INC_IP
PX_RPNCA:
        ; reduce until '(' sentinel
PX_RPLP:
        LODA,R0 STKIDX
        COMI,R0 $00
        BCTA,EQ PX_RPDONE  ; guard
        SUBI,R0 1
        LODI,R1 >OPSTK
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <OPSTK
        BCTR,GT PX_RPNCA2
        ADDI,R0 1
PX_RPNCA2:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 SC0
        COMI,R0 A'('
        BCTR,EQ PX_POPSENT
        BSTA,UN APPLY_OP                 ; [+1]
        BCTR,UN PX_RPLP
PX_POPSENT:
        ; Copy result from VALSH/VALSL[STKIDX] down to [STKIDX-1], then decrement.
        ; This aligns the value stack with the outer expression's STKIDX.

        ; read VALSH[STKIDX] — compute address into TMPH:TMPL
        LODA,R0 STKIDX
        LODI,R1 >VALSH
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSH
        BCTR,GT PX_PS_H1
        ADDI,R0 1
PX_PS_H1:
        STRA,R0 TMPH
        LODA,R0 *TMPH                    ; hi byte of value
        STRA,R0 SC0                      ; save hi

        ; write to VALSH[STKIDX-1]
        LODA,R0 STKIDX
        SUBI,R0 1
        LODI,R1 >VALSH
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSH
        BCTR,GT PX_PS_H2
        ADDI,R0 1
PX_PS_H2:
        STRA,R0 TMPH
        LODA,R0 SC0
        STRA,R0 *TMPH

        ; read VALSL[STKIDX]
        LODA,R0 STKIDX
        LODI,R1 >VALSL
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSL
        BCTR,GT PX_PS_L1
        ADDI,R0 1
PX_PS_L1:
        STRA,R0 TMPH
        LODA,R0 *TMPH                    ; lo byte of value
        STRA,R0 SC0

        ; write to VALSL[STKIDX-1]
        LODA,R0 STKIDX
        SUBI,R0 1
        LODI,R1 >VALSL
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSL
        BCTR,GT PX_PS_L2
        ADDI,R0 1
PX_PS_L2:
        STRA,R0 TMPH
        LODA,R0 SC0
        STRA,R0 *TMPH

        ; decrement STKIDX
        LODA,R0 STKIDX
        SUBI,R0 1
        STRA,R0 STKIDX
PX_RPDONE:
        BCTA,UN PX_PEEKOP                ; continue scanning for more operators

PX_RALL:
        ; reduce all remaining ops
PX_RALL_LP:
        LODA,R0 STKIDX
        COMI,R0 $00
        BCTR,EQ PX_DONE
        SUBI,R0 1
        LODI,R1 >OPSTK
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <OPSTK
        BCTR,GT PX_RANC
        ADDI,R0 1
PX_RANC:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 SC0
        BSTA,UN APPLY_OP                 ; [+1]
        BCTR,UN PX_RALL_LP
PX_DONE:
        ; result is at VALSH[STKIDX]:VALSL[STKIDX]
        ; (after popsent the value index = STKIDX, not necessarily 0)
        LODA,R0 STKIDX
        LODI,R1 >VALSH
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSH
        BCTR,GT PX_DN_HN
        ADDI,R0 1
PX_DN_HN:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 EXPH
        LODA,R0 STKIDX
        LODI,R1 >VALSL
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSL
        BCTR,GT PX_DN_LN
        ADDI,R0 1
PX_DN_LN:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 EXPL
        EORZ,R0
        STRA,R0 ERRFLG
        RETC,UN

; ─── GET_PREC ─────────────────────────────────────────────────────────────────
; R0 = precedence of *IPH  (0=not-an-op, 1=+/-, 2=*/)
GET_PREC:
        LODA,R0 *IPH
        ; fall through to GET_PREC_SC0

; R0 = precedence of char in R0
GET_PREC_SC0:
        COMI,R0 A'+'
        BCTR,EQ GP_LOW
        COMI,R0 A'-'
        BCTR,EQ GP_LOW
        COMI,R0 A'*'
        BCTR,EQ GP_HIGH
        COMI,R0 A'/'
        BCTR,EQ GP_HIGH
        COMI,R0 A'%'
        BCTR,EQ GP_HIGH
        EORZ,R0 ; Clear 
        RETC,UN
GP_LOW:  
        LODI,R0 1
        RETC,UN
GP_HIGH: 
        LODI,R0 2
        RETC,UN

; ─── APPLY_OP ─────────────────────────────────────────────────────────────────
; Apply operator SC0 to top two stack values. Result → VALSH/VALSL[STKIDX-1].
; STKIDX decremented (one value consumed).
; Uses NEGFLG:SC1 as temp for left value during computation.
APPLY_OP:
        ; load right value: VALSH/VALSL[STKIDX]
        LODA,R0 STKIDX
        LODI,R1 >VALSH
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSH
        BCTR,GT AO_RHN
        ADDI,R0 1
AO_RHN:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 EXPH  ; right.hi
        LODA,R0 STKIDX
        LODI,R1 >VALSL
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSL
        BCTR,GT AO_RLN
        ADDI,R0 1
AO_RLN:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 EXPL  ; right.lo

        ; load left value: VALSH/VALSL[STKIDX-1]
        LODA,R0 STKIDX
        SUBI,R0 1
        LODI,R1 >VALSH
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSH
        BCTR,GT AO_LHN
        ADDI,R0 1
AO_LHN:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 NEGFLG  ; left.hi → NEGFLG temp
        LODA,R0 STKIDX
        SUBI,R0 1
        LODI,R1 >VALSL
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSL
        BCTR,GT AO_LLN
        ADDI,R0 1
AO_LLN:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 SC1  ; left.lo → SC1

        ; left = NEGFLG:SC1,  right = EXPH:EXPL
        ; dispatch on SC0
        LODA,R0 SC0
        COMI,R0 A'+'
        BCTR,EQ AO_ADD
        COMI,R0 A'-'
        BCTR,EQ AO_SUB
        COMI,R0 A'*'
        BCTA,EQ AO_MUL
        COMI,R0 A'/'
        BCTA,EQ AO_DIV
        COMI,R0 A'%'
        BCTA,EQ AO_MOD
        RETC,UN

AO_ADD:
        ; EXPH:EXPL = NEGFLG:SC1 + EXPH:EXPL
        LODA,R0 SC1
        ADDA,R0 EXPL
        STRA,R0 EXPL
        TPSL $01                 ; BUG-SCA-14 FIX: carry from lo-byte add
        BCTR,LT AO_ADDNC         ; branch if C=0 (no carry)
        LODA,R0 NEGFLG
        ADDI,R0 1
        BCTR,UN AO_ADDHI
AO_ADDNC:
        LODA,R0 NEGFLG
AO_ADDHI:
        ADDA,R0 EXPH
        STRA,R0 EXPH
        BCTA,UN AO_STORE

AO_SUB:
        ; EXPH:EXPL = NEGFLG:SC1 - EXPH:EXPL
        LODA,R0 SC1
        SUBA,R0 EXPL
        STRA,R0 EXPL
        BCFR,LT AO_SUBNB                 ; no borrow → skip hi decrement
        LODA,R0 NEGFLG
        SUBI,R0 1
        BCTR,UN AO_SUBHI
AO_SUBNB:
        LODA,R0 NEGFLG
AO_SUBHI:
        SUBA,R0 EXPH
        STRA,R0 EXPH
        BCTA,UN AO_STORE

AO_MUL:
        ; MUL16: TMPH:TMPL * EXPH:EXPL → EXPH:EXPL  (NEGFLG:SC1 = left)
        LODA,R0 NEGFLG
        STRA,R0 TMPH
        LODA,R0 SC1
        STRA,R0 TMPL
        BSTA,UN MUL16                    ; [+1]
        BCTR,UN AO_STORE

AO_DIV:
        LODA,R0 NEGFLG
        STRA,R0 TMPH
        LODA,R0 SC1
        STRA,R0 TMPL
        BSTA,UN DIV16                    ; [+1]
        ; ERRFLG=$01 on /0 — DO_ERROR called inside DIV16
        BCTR,UN AO_STORE

AO_MOD:
        ; Modulo: left % right = left - (left/right)*right
        ; DIV16 leaves remainder in TMPH:TMPL (dividend after subtraction loop)
        ; We call DIV16 and use TMPH:TMPL as result.
        ; left=NEGFLG:SC1, right=EXPH:EXPL
        LODA,R0 NEGFLG
        STRA,R0 TMPH
        LODA,R0 SC1
        STRA,R0 TMPL
        BSTA,UN DIV16                    ; [+1] quotient→EXPH:EXPL, remainder→TMPH:TMPL
        ; DIV16 on /0 jumps to DO_ERROR directly
        ; Remainder in TMPH:TMPL — copy to EXPH:EXPL for AO_STORE
        LODA,R0 TMPH
        STRA,R0 EXPH
        LODA,R0 TMPL
        STRA,R0 EXPL
        BCTR,UN AO_STORE

AO_STORE:
        ; write EXPH:EXPL to VALSH/VALSL[STKIDX-1]; STKIDX--
        LODA,R0 STKIDX
        SUBI,R0 1
        LODI,R1 >VALSH
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSH
        BCTR,GT AO_SHN
        ADDI,R0 1
AO_SHN:
        STRA,R0 TMPH
        LODA,R0 EXPH
        STRA,R0 *TMPH
        LODA,R0 STKIDX
        SUBI,R0 1
        LODI,R1 >VALSL
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSL
        BCTR,GT AO_SLN
        ADDI,R0 1
AO_SLN:
        STRA,R0 TMPH
        LODA,R0 EXPL
        STRA,R0 *TMPH
        LODA,R0 STKIDX
        SUBI,R0 1
        STRA,R0 STKIDX
        RETC,UN

; ─── PARSE_FACTOR ─────────────────────────────────────────────────────────────
; Parse one atom: variable A-Z, signed decimal, PEEK(), CHR$(), USR().
; Called from PARSE_EXPR at depth N+1. May call PARSE_EXPR for function args
; (adds 1 more level). Unary - and + handled by PARSE_EXPR before calling here.
; CHR$ result: sets NEGFLG=$01 so DO_PRINT outputs EXPL as a character.
PARSE_FACTOR:
        EORZ,R0 ; Clear R0
        STRA,R0 CHRFLG  ; clear CHR$ flag
        LODA,R0 *IPH
        ; RAS-FIX: inline UPCASE here instead of BSTA UPCASE (+1 slot).
        ; Saves 1 RAS slot so CHR$(expr) path stays within 7 levels.
        ; Equivalent to: if(r0>='a' && r0<='z') r0-=32
        COMI,R0 A'a'
        BCTR,LT PF_UC_DONE       ; < 'a' → already uppercase or not alpha
        COMI,R0 A'z'+1
        BCTR,GT PF_UC_DONE       ; > 'z' → not lowercase
        SUBI,R0 32               ; convert to uppercase
PF_UC_DONE:

        ; check for variable A-Z
        COMI,R0 A'A'
        BCTR,LT PF_NUM
        COMI,R0 A'Z'+1
        BCTR,LT PF_LOADVAR

PF_NUM:
        ; decimal number (may have leading '-' but unary is in PARSE_EXPR)
        BSTA,UN PARSE_S16                ; [+1]
        RETC,UN

PF_LOADVAR:
        ; load variable value from VARS — but first check for CHR$()
        ; R0 already has the uppercased first char. If 'C', may be CHR$
        COMI,R0 A'C'
        BCTR,EQ PF_CHR_TRY
PF_VAR:
        ; BUG-BASIC-03 FIX: save letter to SC0 BEFORE INC_IP clobbers R0.
        STRA,R0 SC0              ; save variable letter (A-Z)
        BSTA,UN INC_IP
PF_LVNCA:
        ; BUG-BASIC-15 FIX: INC_IP returns new IPL in R0, clobbering the letter.
        ; Reload from SC0 before computing the VARS offset.
        LODA,R0 SC0
        SUBI,R0 A'A'  ; 0-25
        STRA,R0 SC1
        ADDA,R0 SC1  ; *2
        LODI,R1 >VARS
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VARS
        BCTR,GT PF_LVN
        ADDI,R0 1
PF_LVN:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 EXPH
        BSTA,UN INC_TMP
PF_LVN2:
        LODA,R0 *TMPH
        STRA,R0 EXPL
        EORZ,R0 ; Clear R0
        STRA,R0 ERRFLG
        RETC,UN

; ─── CHR$(n) detection ────────────────────────────────────────────────────────
; Entry: R0=A'C', IP at 'C'. Check next chars are H, R, $, (
; If yes: consume "CHR$(", parse expr, set NEGFLG=$01 (char output flag).
; If no:  fall through to normal variable load of C.
;
; Input : R0=A'C', *IPH=A'C'
; Output: EXPH:EXPL=char value, NEGFLG=$01, ERRFLG=$00
; Clobbers: R0, TMPH:TMPL, SC0, SC1
PF_CHR_TRY:
        BSTA,UN INC_IP           ; consume 'C'
PF_CHRT1:
        ; NB: stored program text is always uppercase (RDLINE uppercases on store)
        ; so we can compare directly without calling UPCASE here.
        LODA,R0 *IPH
        COMI,R0 A'H'
        BCTR,EQ PF_CHRT2
        ; Not CHR$ — treat C as variable
        LODI,R0 A'C'
        BCTA,UN PF_VAR
PF_CHRT2:
        BSTA,UN INC_IP           ; consume 'H'
PF_CHRT3:
        LODA,R0 *IPH
        COMI,R0 A'R'
        BCTR,EQ PF_CHRT4
        LODI,R0 A'C'
        BCTA,UN PF_VAR
PF_CHRT4:
        BSTA,UN INC_IP           ; consume 'R'
PF_CHRT5:
        LODA,R0 *IPH
        COMI,R0 A'$'
        BCTR,EQ PF_CHRT6
        LODI,R0 A'C'
        BCTA,UN PF_VAR
PF_CHRT6:
        BSTA,UN INC_IP           ; consume '$'
PF_CHRT7:
        BSTA,UN WSKIP
        LODA,R0 *IPH
        COMI,R0 A'('
        BCTR,EQ PF_CHRARG
        LODI,R0 A'C'
        BCTA,UN PF_VAR
PF_CHRARG:
        BSTA,UN INC_IP           ; consume '('
PF_CHREA:
        ; BUG-CHR-01 resolved: depth is now 7 (safe). Upgrade to PARSE_EXPR
        ; so CHR$(I+48), CHR$(A+32) etc. work correctly.
        ; Depth: DO_PRINT(3)→PARSE_EXPR(4)→PARSE_FACTOR/CHR$(5)→PARSE_EXPR(6)
        ;        →PARSE_FACTOR(7) — max 7, safe.
        ; Note: PARSE_FACTOR clears CHRFLG at entry; PF_CHRDN restores it after.
        BSTA,UN PARSE_EXPR       ; [+1]  evaluate full expression
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTR,EQ PF_CHROK
        RETC,UN
PF_CHROK:
        ; PARSE_EXPR already consumed ')' via PX_RPAR — do not consume again.
        ; (Old PARSE_FACTOR path left ')' unconsumed; PARSE_EXPR does not.)
PF_CHRDN:
        LODI,R0 $FF
        STRA,R0 STKIDX           ; BUG-CHR-02: restore outer PARSE_EXPR stack index.
        ; Inner PARSE_EXPR (called from PF_CHREA) initialised STKIDX=$FF and
        ; incremented it to $00 when pushing its result. The outer PARSE_EXPR
        ; at PX_ATOM has not yet called PX_PUSHV, so it expects STKIDX=$FF.
        ; Resetting here lets outer PX_PUSHV store result at slot [0]. Correct.
        LODI,R0 1
        STRA,R0 CHRFLG           ; signal DO_PRINT to output as char
        EORZ,R0
        STRA,R0 ERRFLG
        RETC,UN


; ─── PARSE_RELOP ──────────────────────────────────────────────────────────────
; Scan relational operator(s) at IP, build bitmask in RELOP.
;   '<' sets bit 0 (LT=1), '=' sets bit 1 (EQ=2), '>' sets bit 2 (GT=4)
;   So: < =1  = =2  > =4  <= =3  <> =5  >= =6
; Returns ERRFLG=$00 if any relop found, $01 if none.
; Clobbers: R0, R1 (R1 used as mask accumulator)
; Input : IP at first char of relop
; Output: RELOP = bitmask, ERRFLG=$00 ok / $01 none
PARSE_RELOP:
        BSTA,UN WSKIP                    ; [+1] skip leading space
        EORZ,R0                          ; BUG-RELOP-01 FIX: EORZ,R1 does R0^=R1 not R1=0.
        STRZ,R1                          ; R1 = 0 (mask accumulator)
PRO_LP:
        LODA,R0 *IPH
        COMI,R0 A'<'
        BCTR,EQ PRO_LT
        COMI,R0 A'='
        BCTR,EQ PRO_EQ
        COMI,R0 A'>'
        BCTR,EQ PRO_GT
        ; not a relop char — stop
        COMI,R1 $00
        BCTR,EQ PRO_NONE                 ; no relop chars seen → error
        LODZ,R1                          ; R0 = mask (BUG-RELOP-01: removed STRZ,R1 which clobbered R1 with non-relop char)
        STRA,R0 RELOP
        EORZ,R0
        STRA,R0 ERRFLG
        RETC,UN
PRO_LT:
        IORI,R1 1                        ; set LT bit
        BSTA,UN INC_IP
        BCTR,UN PRO_LP
PRO_EQ:
        IORI,R1 2                        ; set EQ bit
        BSTA,UN INC_IP
        BCTR,UN PRO_LP
PRO_GT:
        IORI,R1 4                        ; set GT bit
        BSTA,UN INC_IP
        BCTR,UN PRO_LP
PRO_NONE:
        LODI,R0 1
        STRA,R0 ERRFLG
        RETC,UN

PARSE_S16:
        ; RAS-FIX: WSKIP removed. PX_ATOM already called WSKIP before PARSE_FACTOR,
        ; and PARSE_S16 is only called from PARSE_FACTOR(PF_NUM) or DO_INPUT.
        ; DO_INPUT calls PARSE_S16 after RDLINE which starts a fresh buffer — no
        ; leading spaces possible. Saves 1 RAS slot on the hot path:
        ;   REPL→STMT_EXEC→DO_PRINT→PARSE_EXPR→PARSE_FACTOR→PARSE_S16→PARSE_U16
        ; was 6 deep; with WSKIP removed from PARSE_S16, deepest is now
        ;   ...→PARSE_S16→PARSE_U16→INC_IP = 6 (SP=6, safe).
        EORZ,R0 ; Clear R0
        STRA,R0 NEGFLG
        LODA,R0 *IPH
        COMI,R0 A'-'
        BCTR,EQ PS16_NEG
        BCTR,UN PS16_UN
PS16_NEG:
        BSTA,UN INC_IP
PS16_NN:
        LODI,R0 1               ; BUG-BASIC-05 FIX: NEGFLG=1 = "negate result"
        STRA,R0 NEGFLG          ; was EORZ,R0 which cleared flag, skipping negation
PS16_UN:
        BSTR,UN PARSE_U16                ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTR,EQ PS16_CHK
        RETC,UN
PS16_CHK:
        LODA,R0 NEGFLG
        COMI,R0 $00
        RETC,EQ                          ; NEGFLG=0 → no negation needed
        ; negate EXPH:EXPL
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
; (PS16_RET merged into inline RETC,EQ above)

; ─── PARSE_U16 ────────────────────────────────────────────────────────────────
; Parse unsigned decimal digits → EXPH:EXPL. ERRFLG=$00 if ≥1 digit.
PARSE_U16:
        EORZ,R0 ; Clear R0
        STRA,R0 EXPH
        STRA,R0 EXPL
        LODI,R0 1               ; BUG-BASIC-06 FIX: ERRFLG=1 = "no digits yet" (failure)
        STRA,R0 ERRFLG          ; was EORZ,R0 meaning "success" before any digit seen
PU16_LP:
        ; RAS-FIX: WSKIP removed entirely from PARSE_U16. All callers must
        ; call WSKIP before invoking PARSE_U16 (PARSE_S16 does; DO_GOTO and
        ; TRY_STORE_LINE have explicit WSKIP added). This saves 1 RAS slot
        ; from the inner loop, preventing overflow at nested IF + CHR$().
        LODA,R0 *IPH
        COMI,R0 A'0'
        RETC,LT
        COMI,R0 A'9'+1
        BCTR,LT PU16_DIG
        RETC,UN
PU16_DIG:
        SUBI,R0 A'0'
        STRA,R0 SC0  ; digit value 0-9
        ; INC_IP inlined to save RAS slot (deepest path: DO_RUN→STMT_EXEC→DO_IF
        ; →PARSE_EXPR→PARSE_FACTOR→PARSE_S16→PARSE_U16→INC_IP would overflow SP=7)
        LODA,R0 IPL
        ADDI,R0 1
        STRA,R0 IPL
        TPSL $01
        BCTR,LT PU16_DNC
        LODA,R0 IPH
        ADDI,R0 1
        STRA,R0 IPH
PU16_DNC:
        ; BUG-SCA-10 FIX: EXP = EXP*10.  Was LODI,R3 10 / BRNR,R3 — BRNR never
        ; decrements R3, so loop ran forever for any input with 2+ digits.
        ; BUG-SCA-11 FIX: BDRR semantics are rn--; if(rn!=0) branch. Load N for
        ; exactly N iterations. Need 10 additions so load 10: 10→9→...→1→0→exit.
        LODA,R0 EXPH
        STRA,R0 TMPH
        LODA,R0 EXPL
        STRA,R0 TMPL  ; TMPH:TMPL = old EXP
        EORZ,R0 ; Clear R0
        STRA,R0 EXPH
        STRA,R0 EXPL
        LODI,R3 10              ; 10 iterations: R3 counts 10→9→...→1→0→exit
PU16_M10:
        LODA,R0 EXPL
        ADDA,R0 TMPL
        STRA,R0 EXPL
        TPSL $01                 ; BUG-SCA-14 FIX: carry from lo-byte add
        BCTR,LT PU16_MNC         ; branch if C=0 (no carry)
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
PU16_MNC:
        LODA,R0 EXPH
        ADDA,R0 TMPH
        STRA,R0 EXPH
        BDRR,R3 PU16_M10       ; R3--; if R3!=0 branch
        ; EXP += digit
        LODA,R0 EXPL
        ADDA,R0 SC0
        STRA,R0 EXPL
        TPSL $01                 ; BUG-SCA-14 FIX: carry from lo-byte add
        BCTR,LT PU16_DIG_NC      ; branch if C=0 (no carry)
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
PU16_DIG_NC:
        EORZ,R0 ; Clear R0
        STRA,R0 ERRFLG  ; success: at least one digit
        BCTA,UN PU16_LP
PU16_DONE:
        ; RETC,UN

; ─── MUL16 ────────────────────────────────────────────────────────────────────
; Signed TMPH:TMPL × EXPH:EXPL → EXPH:EXPL  (16-bit two's complement wrap)
MUL16:
        EORZ,R0 ; Clear R0
        STRA,R0 NEGFLG
        ; abs(left) TMPH:TMPL
        LODA,R0 TMPH
        ANDI,R0 $80
        BCTR,EQ MU_LA
        LODA,R0 TMPH
        EORI,R0 $FF
        STRA,R0 TMPH
        LODA,R0 TMPL
        EORI,R0 $FF
        STRA,R0 TMPL
        LODA,R0 TMPL
        ADDI,R0 1
        STRA,R0 TMPL
        TPSL $01                 ; BUG-SCA-14 FIX: carry from lo-byte +1
        BCTR,LT MU_LNC           ; branch if C=0 (no carry)
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
MU_LNC:
        LODI,R0 1               ; ISSUE-01 FIX (corrected): set NEGFLG=1 on BOTH
        STRA,R0 NEGFLG          ; carry and no-carry paths — left was negative
MU_LA:
        ; abs(right) EXPH:EXPL
        LODA,R0 EXPH
        ANDI,R0 $80
        BCTR,EQ MU_RA
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        LODA,R0 EXPL
        ADDI,R0 1
        STRA,R0 EXPL
        ; BUG-SCA-09 FIX: was BCTA,GT MU_RA — this jumped over BOTH the hi-byte
        ; increment AND the NEGFLG toggle, so for most negative right values (those
        ; whose +1 does not carry to hi byte, e.g. -3→$FFFD, abs=$0003) NEGFLG was
        ; never toggled → wrong sign (3*-3=+9 not -9). Fix: introduce MU_RA_NC so
        ; no-carry path skips only the hi-byte increment, then BOTH paths toggle.
        TPSL $01                 ; BUG-SCA-14 FIX: carry from lo-byte +1
        BCTR,LT MU_RA_NC         ; branch if C=0 (no carry): skip hi increment
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
MU_RA_NC:
        LODA,R0 NEGFLG          ; toggle sign on BOTH carry and no-carry paths
        EORI,R0 $01
        STRA,R0 NEGFLG
MU_RA:
        ; save right in SC0:SC1; result EXP=0
        LODA,R0 EXPH
        STRA,R0 SC0
        LODA,R0 EXPL
        STRA,R0 SC1
        EORZ,R0 ; Clear R0
        STRA,R0 EXPH
        STRA,R0 EXPL
MU_LP:
        LODA,R0 TMPH
        COMI,R0 $00
        BCTR,GT MU_ADD
        LODA,R0 TMPL
        COMI,R0 $00
        BCTR,EQ MU_DONE
MU_ADD:
        LODA,R0 EXPL
        ADDA,R0 SC1
        STRA,R0 EXPL
        TPSL $01                 ; BUG-SCA-14 FIX: carry from lo-byte add
        BCTR,LT MU_MNC           ; branch if C=0 (no carry)
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
MU_MNC:
        LODA,R0 EXPH
        ADDA,R0 SC0
        STRA,R0 EXPH
        ; TMPH:TMPL-- (left counter)
        LODA,R0 TMPL
        SUBI,R0 1
        STRA,R0 TMPL
        BCFR,LT MU_TNB
        LODA,R0 TMPH
        SUBI,R0 1
        STRA,R0 TMPH
MU_TNB:
        BCTR,UN MU_LP
MU_DONE:
        LODA,R0 NEGFLG
        COMI,R0 $00
        BCTR,EQ MU_RET
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
MU_RET:
        EORZ,R0                          ; ISSUE-01 RE-FIX pt2: clear NEGFLG on
        STRA,R0 NEGFLG                   ; exit — dual-use with CHR$ flag in DO_PRINT
        RETC,UN

; ─── DIV16 ────────────────────────────────────────────────────────────────────
; Signed TMPH:TMPL ÷ EXPH:EXPL → EXPH:EXPL  (truncate toward zero)
; ERRFLG=$01 and DO_ERROR called on divide-by-zero.
DIV16:
        EORZ,R0 ; Clear R0
        STRA,R0 ERRFLG
        LODA,R0 EXPH
        COMI,R0 $00
        BCTR,GT DV_NZ
        LODA,R0 EXPL
        COMI,R0 $00
        BCTA,EQ DV_ZERO
DV_NZ:
        EORZ,R0 ; Clear R0
        STRA,R0 NEGFLG
        ; abs(dividend) TMPH:TMPL
        LODA,R0 TMPH
        ANDI,R0 $80
        BCTR,EQ DV_DA
        LODA,R0 TMPH
        EORI,R0 $FF
        STRA,R0 TMPH
        LODA,R0 TMPL
        EORI,R0 $FF
        STRA,R0 TMPL
        LODA,R0 TMPL
        ADDI,R0 1
        STRA,R0 TMPL
        TPSL $01                 ; BUG-SCA-14 FIX: carry from lo-byte +1
        BCTR,LT DV_DNC           ; branch if C=0 (no carry)
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
DV_DNC:
        LODI,R0 1               ; ISSUE-01 FIX (corrected): set NEGFLG=1 on BOTH
        STRA,R0 NEGFLG          ; carry and no-carry paths — dividend was negative
DV_DA:
        ; abs(divisor) EXPH:EXPL
        LODA,R0 EXPH
        ANDI,R0 $80
        BCTR,EQ DV_VA
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        LODA,R0 EXPL
        ADDI,R0 1
        STRA,R0 EXPL
        ; BUG-SCA-09b FIX: same as MUL16 right-operand fix. BCTA,GT DV_VA jumped
        ; over BOTH hi-byte increment AND NEGFLG toggle for no-carry cases.
        ; Fix: introduce DV_VA_NC so no-carry skips only the hi-byte increment.
        TPSL $01                 ; BUG-SCA-14 FIX: carry from lo-byte +1
        BCTR,LT DV_VA_NC         ; branch if C=0 (no carry): skip hi increment
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
DV_VA_NC:
        LODA,R0 NEGFLG          ; toggle sign on BOTH paths
        EORI,R0 $01
        STRA,R0 NEGFLG
DV_VA:
        LODA,R0 EXPH
        STRA,R0 SC0  ; divisor hi
        LODA,R0 EXPL
        STRA,R0 SC1  ; divisor lo
        EORZ,R0 ; Clear R0
        STRA,R0 EXPH
        STRA,R0 EXPL  ; quotient = 0
DV_LP:
        ; while TMPH:TMPL >= SC0:SC1 (unsigned)
        LODA,R0 TMPH
        SUBA,R0 SC0               ; hi byte (SC0 < $80 always for reasonable divisors)
        BCTR,LT DV_DONE           ; TMPH < SC0 (signed OK if SC0 < $80) → done
        BCTR,GT DV_SUB            ; TMPH > SC0 → subtract
        ; TMPH == SC0: unsigned lo comparison via carry
        LODA,R0 TMPL
        SUBA,R0 SC1
        TPSL $01                  ; C=1 → no borrow → TMPL >= SC1 → subtract
        BCTR,EQ DV_SUB            ; C=1 → TMPL >= SC1 → continue subtract
        BCTR,UN DV_DONE           ; C=0 → TMPL < SC1 → done
DV_SUB:
        LODA,R0 TMPL
        SUBA,R0 SC1
        STRA,R0 TMPL
        TPSL $01                  ; C=1 → no borrow → skip hi decrement
        BCTR,EQ DV_SNB            ; C=1 → no borrow
        LODA,R0 TMPH
        SUBI,R0 1
        STRA,R0 TMPH
DV_SNB:
        LODA,R0 TMPH
        SUBA,R0 SC0
        STRA,R0 TMPH
        ; quotient++
        BSTA,UN INC_EXP
        BCTR,UN DV_LP
DV_DONE:
        LODA,R0 NEGFLG
        COMI,R0 $00
        BCTR,EQ DV_RET
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
DV_RET:
        EORZ,R0                          ; ISSUE-01 RE-FIX pt2: clear NEGFLG on
        STRA,R0 NEGFLG                   ; exit — dual-use with CHR$ flag in DO_PRINT
        RETC,UN
DV_ZERO:
        LODI,R0 2
        BCTA,UN DO_ERROR  ; divide by zero error

; ─── PRINT_S16 ────────────────────────────────────────────────────────────────
; Print signed 16-bit value EXPH:EXPL as decimal.
; Uses DIVTAB for digit extraction. NEGFLG = leading-zero suppression flag.
PRINT_S16:
        LODA,R0 EXPH
        ANDI,R0 $80
        BCTA,EQ PS16P_POS
        LODI,R0 A'-'
        BSTA,UN COUT
        ; BUG-PRINT-MIN FIX: -32768 ($8000) negation overflows back to $8000.
        ; Detect EXPH=$80,EXPL=$00 and print "32768" directly.
        LODA,R0 EXPH
        COMI,R0 $80
        BCTR,EQ PS16P_CHKMIN
PS16P_NEGNORM:
        ; normal negation: flip bits, add 1
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
        BCTR,UN PS16P_POS
PS16P_CHKMIN:
        LODA,R0 EXPL
        COMI,R0 $00
        BCTR,EQ PS16P_MIN        ; exactly $8000 = -32768
        BCTR,UN PS16P_NEGNORM    ; $80xx with non-zero lo — normal negative
PS16P_MIN:
        ; print "32768" as ASCII literals
        LODI,R0 A'3'
        BSTA,UN COUT
        LODI,R0 A'2'
        BSTA,UN COUT
        LODI,R0 A'7'
        BSTA,UN COUT
        LODI,R0 A'6'
        BSTA,UN COUT
        LODI,R0 A'8'
        BSTA,UN COUT
        RETC,UN
PS16P_POS:
        ; zero check: EXP == $0000?  (use EQ not GT to handle $FF correctly)
        LODA,R0 EXPH
        COMI,R0 $00
        BCTR,GT PS16P_NZ
        BCTR,LT PS16P_NZ
        LODA,R0 EXPL
        COMI,R0 $00
        BCTR,EQ PS16P_ZERO        ; EXPH=$00, EXPL=$00 → zero
        BCTR,UN PS16P_NZ          ; EXPH=$00, EXPL!=0 → non-zero (handles $FF correctly)
PS16P_ZERO:
        LODI,R0 A'0'
        BSTA,UN COUT
        RETC,UN

PS16P_NZ:
        LODI,R0 <DIVTAB
        STRA,R0 TMPH
        LODI,R0 >DIVTAB
        STRA,R0 TMPL
        EORZ,R0 ; Clear R0
        STRA,R0 NEGFLG  ; leading-zero flag
        ; PRINT_S16 needs unsigned compares for the digit loop (value >= divisor).
        ; Save PSL in R2 and set COM=1 (unsigned compare mode).
        SPSL
        STRZ,R2
        PPSL $02
PS16P_DIVLP:
        ; load next divisor pair from DIVTAB
        LODA,R0 *TMPH
        STRA,R0 SC0  ; div hi
        BSTA,UN INC_TMP
PS16P_D1:
        LODA,R0 *TMPH
        STRA,R0 SC1  ; div lo
        BSTA,UN INC_TMP
PS16P_D2:
        ; sentinel 0,0 → print final ones digit
        LODA,R0 SC0
        COMI,R0 $00
        BCTR,GT PS16P_CNT
        LODA,R0 SC1
        COMI,R0 $00
        BCTA,EQ PS16P_LAST
PS16P_CNT:
        ; count subtractions using R3
        LODI,R3 $00
PS16P_SLP:
        ; Unsigned compare (COM=1): if value < divisor → emit digit.
        ; Compare high byte first.
        LODA,R0 EXPH
        LODA,R1 SC0
        COMZ,R1
        BCTR,LT PS16P_EMIT
        BCTR,GT PS16P_DO
        ; High bytes equal → compare low
        LODA,R0 EXPL
        LODA,R1 SC1
        COMZ,R1
        BCTR,LT PS16P_EMIT
PS16P_DO:
        ; 16-bit subtract with low-borrow propagation
        LODA,R0 EXPL
        SUBA,R0 SC1
        STRA,R0 EXPL
        TPSL $01
        BCTR,EQ PS16P_NB             ; C=1 → no borrow from low byte
        LODA,R0 EXPH
        SUBI,R0 1
        STRA,R0 EXPH
PS16P_NB:
        LODA,R0 EXPH
        SUBA,R0 SC0
        STRA,R0 EXPH
        ADDI,R3 1                    ; digit++
        BCTR,UN PS16P_SLP
PS16P_EMIT:
        ; R3 = digit value
        LODA,R0 NEGFLG
        COMI,R0 $00
        BCTR,GT PS16P_FPRINT  ; already printing
        ; leading zero check: LODZ,R3 → R0 = R3
        LODZ,R3                 ; R0 = R3 (digit count, LODZ Rn loads Rn into R0)
        COMI,R0 $00
        BCTA,EQ PS16P_DIVLP  ; skip leading zero
PS16P_FPRINT:
        LODZ,R3                 ; R0 = R3 (digit value 0-9)
        ADDI,R0 A'0'            ; R0 = ASCII digit
        BSTA,UN COUT
        LODI,R0 1               ; BUG-BASIC-04 FIX: NEGFLG=1 = "digits active, print all"
        STRA,R0 NEGFLG          ; was EORZ,R0 which cleared flag, suppressing subsequent digits
        BCTA,UN PS16P_DIVLP
PS16P_LAST:
        LODA,R0 EXPL
        ADDI,R0 A'0'
        BSTA,UN COUT
        ; restore PSL (undo COM=1)
        LODZ,R2
        LPSL
        RETC,UN

; ─── GETKEY ───────────────────────────────────────────────────────────────────
; Blocking keyboard read via Pipbug CHIN.
; CHIN is blocking — waits for a keypress before returning.
;
; Later Implement Proprietary Bitbanged SENSE input when basic working
; Returns char in R0.  Clobbers R0 only.
GETKEY:
;        BSTA,UN CHIN            ; R0 = char (CHIN blocks until key pressed)
;        RETC,UN

; ─── RDLINE ───────────────────────────────────────────────────────────────────
; Read a line from input into IBUF, echo with backspace support. NUL-terminates.
; Uses GETKEY (via CHIN) for blocking input. Char received in R0 at each step;
; saved to R1 for storage/echo so R0 is free for pointer arithmetic.
RDLINE:
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
RL_LP:
        BSTA,UN CHIN          ; [+1] blocking — R0 = char
        COMI,R0 NUL             ; BUG-ASM-08 FIX: NUL = EOF from sim stdin.
        BCTA,EQ RL_EOL          ;   Treat as end-of-line so we don't flood IBUF
        ;                       ;   (and overwrite VARS) after stdin is exhausted.
        STRZ,R1                 ; R1 = char (R0 still has char for CR/BS checks)
        COMI,R1 CR
        BCTA,EQ RL_EOL
        COMI,R1 LF
        BCTA,EQ RL_EOL
        ; ISSUE-06 FIX: removed redundant second COMI,R1 NUL / BCTA,EQ RL_EOL here.
        ; BUG-ASM-08 fix (first NUL check immediately after GETKEY above) already
        ; catches EOF before we reach this point — second check was dead code.
        COMI,R1 BS
        BCTR,EQ RL_BS
        ; buffer full?  IP >= IBUF+63
        ; BUG-BASIC-17 FIX: was SUBA (absolute read) not SUBI (immediate compare).
        ; SUBA,R0 <IBUF reads mem[$0016] (now at $1600+) (PIPBUG ROM), not the constant $15.
        ; All four pointer comparisons here must use SUBI.
        LODA,R0 IPH
        SUBI,R0 <IBUF           ; compare IPH against IBUF hi byte ($16)
        BCTR,GT RL_FULL
        BCTR,LT RL_STORE
        LODA,R0 IPL
        SUBI,R0 >IBUF+63        ; compare IPL against IBUF lo byte + 63 ($83 at $1644+63)
        BCTR,LT RL_STORE
RL_FULL:
        BCTR,UN RL_LP
RL_STORE:
        STRA,R1 *IPH            ; store char to buffer
        LODZ,R1
        BSTA,UN COUT            ; echo char
        BSTA,UN INC_IP
        BCTR,UN RL_LP
RL_BS:
        ; at IBUF start? — no backspace if buffer empty
        LODA,R0 IPH
        SUBI,R0 <IBUF           ; compare IPH against IBUF hi byte ($16)
        BCTR,GT RL_BSDO
        BCTR,LT RL_LP
        LODA,R0 IPL
        SUBI,R0 >IBUF           ; compare IPL against IBUF lo byte ($44 at $1644)
        BCTA,EQ RL_LP
RL_BSDO:
        LODA,R0 IPL
        SUBI,R0 1
        STRA,R0 IPL
        BCFR,LT RL_BSNB
        LODA,R0 IPH
        SUBI,R0 1
        STRA,R0 IPH
RL_BSNB:
        LODI,R0 BS
        BSTA,UN COUT
        LODI,R0 SP
        BSTA,UN COUT
        LODI,R0 BS
        BSTA,UN COUT
        BCTA,UN RL_LP
RL_EOL:
        LODI,R1 NUL
        STRA,R1 *IPH            ; NUL-terminate buffer
        BSTA,UN CRLF
        RETC,UN

; ─── PRTSTR / PRTSTR_IP ───────────────────────────────────────────────────────
; Print NUL-terminated string at IPH:IPL.
; PRTSTR_IP is the same routine, just an alias for clarity at the call site.
PRTSTR_IP:
PRTSTR:
        LODA,R1 *IPH
        COMI,R1 NUL
        ;BCTA,EQ PRTSTR_RET
        RETC,EQ
        LODZ,R1
        BSTA,UN COUT
        BSTR,UN INC_IP
        BCTR,UN PRTSTR
PRTSTR_RET:
       ; RETC,UN

; ─── WSKIP ────────────────────────────────────────────────────────────────────
WSKIP:
        LODA,R0 *IPH
        COMI,R0 SP
        BCTR,EQ WS_ADV
        RETC,UN
WS_ADV:
        BSTR,UN INC_IP
        BCTR,UN WSKIP

; ─── GETCI_UC ─────────────────────────────────────────────────────────────────
; Read *IPH uppercase into R0, advance IP.
; BUG-ASM-04 FIX: INC_IP clobbers R0 (returns new IPL). Save char in R1
; across the INC_IP call using STRZ,R1 / LODZ,R1 sandwich.
; Clobbers: R1 (caller must not rely on R1 across GETCI_UC call)
GETCI_UC:
        LODA,R0 *IPH
        BSTR,UN UPCASE                   ; [+1] R0 = uppercased char
        STRZ,R1                          ; R1 = char (save before INC_IP clobbers R0)
        BSTR,UN INC_IP                   ; [+1] advance IP (clobbers R0)
        LODZ,R1                          ; R0 = char (restore)
GETCI_UC_RET:
        RETC,UN

; ─── UPCASE ───────────────────────────────────────────────────────────────────
UPCASE:
        COMI,R0 A'a'
        ; BCTA,LT UC_RET
        RETC,LT
        COMI,R0 A'z'+1
        BCTR,LT UC_DO
        ;BCTR,UN UC_RET
        RETC,UN
UC_DO:
        SUBI,R0 32
UC_RET:
        RETC,UN

; ─── EATWORD ──────────────────────────────────────────────────────────────────
; Skip [A-Za-z$] at IP.
EATWORD:
        LODA,R0 *IPH
        BSTR,UN UPCASE  ; [+1]
        COMI,R0 A'A'
        BCTR,LT EW_DS
        COMI,R0 A'Z'+1
        BCTR,LT EW_ADV
EW_DS:
        COMI,R0 A'$'
        BCTR,EQ EW_ADV
        RETC,UN
EW_ADV:
        BSTR,UN INC_IP
        BCTR,UN EATWORD

; ─── SHARED 16-BIT POINTER INCREMENT/DECREMENT SUBROUTINES ───────────────────
; INC_IP  : IPH:IPL  += 1   (clobbers R0)
; INC_TMP : TMPH:TMPL += 1  (clobbers R0)
; INC_EXP : EXPH:EXPL += 1  (clobbers R0)
; DEC_TMP : TMPH:TMPL -= 1  (clobbers R0)
; Rule: NO BSTA inside these — must not consume extra RAS depth.
; Carry idiom: ADDI sets no-carry->GT, carry->EQ/LT.
;   BCTA,LT skip = branch on no-carry (C=0). BCFA,LT skip = branch on no-borrow (C=1).
; Borrow idiom: SUBI sets no-borrow->GT/EQ, borrow->LT.
;   BCFA,LT skip  =  skip hi-byte decrement if no borrow (C=1).

INC_IP:
        LODA,R0 IPL
        ADDI,R0 1
        STRA,R0 IPL
        ;BCTA,GT INC_IP_RET      ; no carry — hi byte unchanged
        TPSL $01                 ; BUG-SCA-14 FIX: test carry bit directly
        RETC,LT                  ; return if C=0 (no carry) — result sign unreliable
        LODA,R0 IPH
        ADDI,R0 1
        STRA,R0 IPH
INC_IP_RET:
        RETC,UN

INC_TMP:
        LODA,R0 TMPL
        ADDI,R0 1
        STRA,R0 TMPL
        ;BCTA,GT INC_TMP_RET     ; no carry
        TPSL $01                 ; BUG-SCA-14 FIX: test carry bit directly
        RETC,LT                  ; return if C=0 (no carry)
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
INC_TMP_RET:
        RETC,UN

INC_EXP:
        LODA,R0 EXPL
        ADDI,R0 1
        STRA,R0 EXPL
        ;BCTA,GT INC_EXP_RET     ; no carry
        TPSL $01                 ; BUG-SCA-14 FIX: test carry bit directly
        RETC,LT                  ; return if C=0 (no carry)
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
INC_EXP_RET:
        RETC,UN

DEC_TMP:
        LODA,R0 TMPL
        SUBI,R0 1
        STRA,R0 TMPL
        RETC,LT                  ; C=1 (no borrow) → hi unchanged, return
        LODA,R0 TMPH
        SUBI,R0 1
        STRA,R0 TMPH
        RETC,UN


; ─── DO_ERROR ─────────────────────────────────────────────────────────────────
; Entry: R0 = error code (0-5).
; Saves RUNFLG, clears all run state, prints "?n [IN line]", jumps to REPL.
; This is a tail-jump (BCTA,UN DO_ERROR from callers), so it kills the full RAS.
DO_ERROR:
        STRA,R0 SC0                      ; save error code
        LODA,R0 RUNFLG
        STRA,R0 SC1  ; save run state
        EORZ,R0 ; Clear R0
        STRA,R0 RUNFLG  ; clear run
        LODI,R0 $FF
        STRA,R0 SWSP  ; clear GOSUB stack
        LODI,R0 A'?'
        BSTA,UN COUT
        LODA,R0 SC0
        ADDI,R0 A'0'
        BSTA,UN COUT
        LODA,R0 SC1
        COMI,R0 $01
        BCTR,EQ DE_IN
        BCTR,UN DE_NL
DE_IN:
        LODI,R0 SP
        BSTA,UN COUT
        LODI,R0 A'I'
        BSTA,UN COUT
        LODI,R0 A'N'
        BSTA,UN COUT
        LODI,R0 SP
        BSTA,UN COUT
        LODA,R0 CURH
        STRA,R0 EXPH
        LODA,R0 CURL
        STRA,R0 EXPL
        BSTA,UN PRINT_S16                ; [+1]
DE_NL:
        BSTA,UN CRLF
        BCTA,UN REPL                     ; jump to REPL — clears full hardware RAS
ROMEND: ; so we can measure Binary rom size
        END
