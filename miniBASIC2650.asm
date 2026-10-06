; =============================================================================
; miniBASIC2650 v3.3  --  Tiny BASIC with 32-bit MBF4 floating point for the Signetics 2650
; (derived from uBASIC2650 v2.11; archive/uBASIC2650_v2.11_orig.asm is the integer original; RND/PRE_CHIN ported from uBASIC2650 v2.12)
; Copyright (c) 2026 Vincent Crabtree, licensed under the MIT License, see LICENSE
;
; Note: standalone build - I/O is bit-banged in software, no PIPBUG ROM or
; UART/ACIA hardware required.
;
;   CPU    : Signetics 2650
;   ROM    : 4 KB target, $0000 upward (ROMEND $0E99 = 3737 bytes - see ROMEND; the FP library is the block marked MBF4 near the end)
;   RAM    : ~400 bytes of variables and stacks from $1000 (FA/FB and the FP work area included), program store PROG..$1FFF above
;   I/O    : CHIN/COUT, bit-banged software serial via PSU/PSL flag bits.
;            Addresses float per build - read them from the .LST (see BUILD)
;   NMI/IRQ: Not used
;
; Usage:
;   Power-up prints the banner and the '>' prompt, with the showcase preloaded: type RUN (CR terminates a line) to run it, LIST to read it,
;   FREE for the free program bytes, NEW to clear the store before typing your own program.  Lines: <number 0-32767> <statement>.
;
; Statements:
;   INPUT  END  FREE  GOTO <expr>  GOSUB <expr>  IF <cond> [THEN] <stmt>
;   LIST  NEW  PRINT  RETURN  RUN  [LET] <var>=<expr>  (LET and THEN optional)
;   FOR <var>=<expr> TO <expr> [STEP <expr>]   NEXT [<var>]
;
; PRINT items: "literal", CHR$(n), TAB(n), or an expression; separate with ';'.
;
; Arithmetic: + - * /  (unary -).  True division: 7/2 is 3.5; X/0 is error ?Z; overflow (> about 1.7E38) is ?O; no wraparound.
; Precedence: BODMAS-lite * / binds before + -, which binds before relops (= < >).
; Relops: `=`  `<`  `>`  prefix with ! to invert: `!=`, `!<`, `!>` (<= is !>, >= is !<).
; RND: niladic: the 16-bit signed pseudorandom integer (as in uBASIC2650 v2.12) converted to a float, no parenthesis/arg.  A value in
;      0 < x <= 1 is ABS(RND)/32768 (see the showcase, lines 77-79).
; ABS(expr): absolute value function - expressions allowed
; SIN(expr) COS(expr): sine and cosine, the argument in RADIANS (any expression, as for ABS).  Accurate to about 5E-7 for |x| <= 2*PI;
;      the error grows with |x| (see KNOWN LIMITATIONS) and |x| above about 51471 gives ?R.  There is no PI constant: write 3.14159265, or
;      K=0.0174532925 for degrees as the showcase does.  Only these two functions so far (Stage 5 tier 1).
;
; Numbers : MBF4 floating point: 24-bit mantissa (exact integers to 16777216), exponent range about 1E-38 .. 1.7E38.
;           Literals [digits][.digits] (no E notation).  PRINT shows 6 significant digits in plain notation
;           (1234567 prints 1234570, 1/3 prints 0.333333, trailing zeros trimmed).  Integer uses (GOTO/GOSUB target, line
;           numbers, TAB(, CHR$() truncate toward zero; |x| >= 32768 gives ?R.  INT() is not provided.
; Variables: A-Z (26), MBF4 values (4 bytes each); no arrays or string variables
; Print   : "literals", CHR$(n), TAB(n), `;` separator - no string vars
;
; KNOWN LIMITATIONS
;
; Floating point (v3.0-v3.2):
;   - No E notation: 1E3 reads as 1 (the E3 is silently ignored); write 1000.  No INT(); truncate with CHR$/TAB(/GOTO targets.
;   - FOR/NEXT with a fractional STEP accumulates rounding error: STEP 0.25 is exact, STEP 0.1 is not (FOR X=0 TO 1 STEP 0.1 stops after 0.9
;     because ten additions of 0.1 give 1.0000001).  '=' is exact equality.
;   - RND is the new 16-bit LFSR seed as a float (signed, -32768..32767; never 0, so ABS(RND)/32768 is 0 < x <= 1).
;
; Trigonometry (v3.3):
;   - SIN/COS reduce the argument as y = |x|*2/PI in single precision, so the result carries an angular error of about 6E-8*|x| radians (the
;     reduction, not the polynomial, which is good to 4E-8).  Measured maximum error against double precision (tools/fn_diff.py):
;     1.5E-7 in [0,PI/2], 4.4E-7 in [-2PI,2PI], 7E-6 up to |x|=100, 4E-4 up to 5000, 3.5E-3 up to 51000.  |x|*2/PI >= 32768 (|x| above about
;     51471) gives ?R.  COS(PI/2) prints a tiny non-zero number: the single-precision argument is not exactly PI/2.
;   - Name matching: in an expression S+letter is SIN and C+letter is COS (R RND, A ABS); any further letters are eaten.  A variable directly
;     followed by a letter (SA) reads as SIN.  In PRINT, CHR$ and TAB( are told from COS and TAN( by their THIRD letter (R / B).
;   - The library rounds half away from zero.  Against mbf4.py with rounding RN_AWAY, * and / were bit-exact and + - differed by 1 ULP in 1 of
;     300 random pairs (tools/lib_bitexact.py); the library was not changed.
;
; UPPERCASE only apart from PRINT "String literals"
;
; RELATIONAL OPERATORS
;   =  <  > are native; '!' inverts the one that follows it (!=, !<, !>),
;   so all six are available: A<=B is A!>B; A>=B is A!<B; A<>B is A!=B.
;   A relop yields -1 (true) or 0 (false).
;   Relops have the LOWEST precedence and associate to the RIGHT: a relop's right
;   operand is a whole + - * / expression, so A<B+1 is A<(B+1) and
;   1+A<B*2 is (1+A)<(B*2).  Do not chain relops: 1<2<3 is 1<(2<3.  Keep a '!' 
;   relop's right operand free of other relops (use a variable or separate IFs).
;
; OPERATOR PRECEDENCE
;   BODMAS-lite, tightest first: * /   then   + -   then relops.
;   So "1+2*3" is 7 and "1+2>2*2" is (1+2)>(2*2).  * / and + - are left-to-right
;   among themselves; relops are right-to-left (see above).  Parentheses override.
;
; PARENTHESIS NESTING
;   Bounded by the SW stack (SWBASE, 64 bytes). Limit depends on expression
;   complexity - operators within a level consume additional stack so a
;   deeply-nested expression with many operators uses more stack per level.
;   ERR_EXPR ('?8') fires gracefully when the stack is full. Practical
;   ceiling for typical BASIC expressions is 5-6 levels of mixed
;   paren/operator nesting.
;
; GOSUB/RETURN NESTING
;   Fixed ceiling of 4 levels deep (GSSTKLIM=8 bytes, 2 per level). A 5th
;   nested GOSUB, or a RETURN with nothing pushed, raises ERR_OOM ('?3').
;
; FOR / NEXT
;   FOR <var>=<start> TO <limit> [STEP <step>], then NEXT [<var>].  Step
;   defaults to 1 if omitted; a negative STEP counts down (limit < start).
;   - Test at end so Body always runs at least once
;   - NEXT always closes the innermost loop
;   - STEP 0 loops forever (var never passes the limit) - no check for it.
;   - Leaving a loop early (GOTO / GOSUB target outside it) not permitted.
;
; STATEMENT DISPATCH matches ONLY the first character of a line.
;   GOTO/GOSUB, RUN/RETURN/REM, LIST/LET and NEW/NEXT dispatched on 3rd char.
;   LET and THEN are optional.
;
; STORAGE / INPUT
;   Full random line editing: a numbered line is inserted in order,
;   replaces an existing line of the same number, or (empty body) deletes it.
;   Permitted Line numbers are 0-32767.
;   Input is not bounds-checked; overlong lines will corrupt memory.
;
; SYNTAX VALIDATION
;   Minimal; malformed constructs may produce a generic syntax/runtime
;   error rather than a specialised diagnostic.
;
; =============================================================================
; IMPLEMENTATION NOTES (for maintainers)
; =============================================================================
;
; BUILD
;   gcc -Wall -O2 -o asm2650 asm2650.c
;   gcc -O2 -DGAMER -o pipbug_wrap pipbug_wrap.c
;
;   ./asm2650 --no-warn-inline-label miniBASIC2650.asm miniBASIC2650.hex
;   grep -n "^CHIN \|^COUT \|^ROMEND " miniBASIC2650.LST
;   ./pipbug_wrap --entry 0 --chin 0x<addr> --cout 0x<addr> miniBASIC2650.hex   (--cout at an UNUSED address runs the real bit-banged COUT)
;
; ROMEND is used as ROM tide mark
;
; 2650 FLAGS / COMPARES
;   SUB/ADD: CC = LT, EQ or GT from the signed 8-bit result.
;   Carry (PSL bit 0): C=1 = no borrow / carry, C=0 = borrow.
;   Carry test: TPSL $01 -> EQ if C=1, LT if C=0.
;   Unsigned compare: COMA.  COM mode is set once at boot and left set.
;   All COMI inputs here are 0-127 values.
;
; 16-BIT CONVENTION
;   <$ADDR = high byte
;   >$ADDR = low byte
;   Example: <$1634 = $16, >$1634 = $34
;
; REGISTER CONVENTIONS
;   R0  general working register / arithmetic / I/O
;   R1  index register; also used by PRINT_S16
;   R2  current variable letter; MUST survive the entire RHS expression
;       evaluation of a "V=expr" assignment (DL_STORE reads it back after
;       EXPR returns) - do not use as scratch inside EXPR/EXPR_ATOM or any
;       routine they call. PEEK_C2_ALPHA clobbers R2 by design (its 2-letter
;       keyword check for DO_PRINT) and is therefore only safe to call from
;       statement-level dispatch, never from inside expression parsing.
;   R3  loop counter / Expression SW stack 
;
; RAS / RECURSION
;   The 2650 has an 8-level hardware return-address stack, not user accessible.
;   BSxx/ZBSR consume a RAS entry; BCxx/ZBRR do not.
;   PARSE_EXPR guards against excessive hardware-stack depth.
;   Parentheses use a software depth counter to reduce RAS usage.
;   GOSUB/RETURN do NOT consume RAS for BASIC-level nesting depth - SWSTK
;   (not a real call) carries the redirect, so 4 levels of GOSUB cost the
;   same hardware stack as 1.
;
; =============================================================================
; VERSION HISTORY 
; =============================================================================
;
; v3.3 (Oct 2026) - Stage 5 tier 1: SIN and COS (radians).  ROMEND $0D8D -> $0E99 (3469 -> 3737 bytes, +268); 359 bytes remain below the $1000 end
;   of the 2732 EPROM.  (v3.2's header said $0D8F / 3471; the assembler gives $0D8D / 3469 for the v3.2 source as received.)
;   - New block S_TRIG..E_TRIG after the Stage 3 library, which is unchanged (240 bytes): DO_SIN/DO_COS (atoms), SIN_RET/COS_RET, SC_CORE, HORNER_ODD
;     (table-driven, reusable for ATN/LN/EXP), LD_B_PTR / LD_B_Z / SAVE_Z, and SCT (2/PI, 1.0, five minimax coefficients, 28 bytes).
;   - Method: y = |x|*2/PI, q = TRUNC(y), f = y - q;  n = (q + offset) AND 3  (SIN: offset 0, 2 for x < 0;  COS: 1);  t = f, or 1 - f for odd n;
;     r = t * P(t*t) = sin(PI*t/2);  r = -r when n AND 2.  Horner evaluation, coefficients from tools/gen_coeffs.py.
;   - Hooks, 28 bytes: EXPR_ATOM dispatches 'S' and 'C' (+10); DO_PRINT tells CHR$/TAB( from COS/TAN( by the third letter (+16: it moved DP_SEP out of BCTR
;     range, so three branches became BCTA); EXPR's two PUSH_LOLOOP/PUSH_HILOOP calls became BSTA (+2, BSTR was out of range).
;   - No new RAM: ZV, QF, HPTR, HCNT alias the 8 bytes of PRINT's private digit buffer DIG, which is free while a function is evaluated.
;   - Call depth (tools/depth_trig.py): the continuation starts at depth 2 (prompt) or 3 (RUN, also under nested IF/GOSUB/FOR) and adds 4
;     (SC_CORE -> HORNER_ODD -> FLT_MUL/FLT_ADD -> inner call).  Peak 7, the same as the existing maximum; 8 is never reached.
;   - Showcase: trig section lines 520-590 (297 jumps to 520, 590 jumps back to 300): values, a degrees table, S^2+C^2, a one-period sine wave.
;     Boot FREE 1608 -> 1238.  Mandelbrot plot unchanged (924 cells, 0 differences from the golden model).
;   - Tested: SIN/COS bit-exact against the host model tools/fn_model.py (520 values, 0 mismatches, with the model rounding half away like the
;     library); p1..p8 identical to v3.2 apart from banner and boot FREE; probes p9_trig (values, nesting, FOR/IF/GOSUB, name collisions,
;     errors) and p10_trigdepth (depth).
;   - Found, not changed: the library rounds half away from zero (the golden model's default is half to even); see KNOWN LIMITATIONS.
;
; v3.2 (Oct 2026) - RND improvements ported from uBASIC2650 v2.12 (RND_SHUFFLE, PRE_CHIN, DO_RND).  ROMEND $0D8A -> $0D8F (3466 -> 3471 bytes, +5).
;   - RND_SHUFFLE now runs in register bank 1 (PPSL/CPSL PSW_RS+PSW_WC): the caller's bank-0 R1-R3 survive (R0 is not banked, so it does not).
;     v3.1's version ran in bank 0 and clobbered R1.  That was harmless from DO_RND but not from CHIN, which called it before its own bank
;     switch and so destroyed GETLINE's buffer index on real hardware; the simulator intercepts CHIN, so it never showed there.
;   - New PRE_CHIN (RND_SHUFFLE once, then falls into CHIN); GETLINE calls PRE_CHIN; CHIN no longer shuffles.  The LFSR now steps once per
;     keystroke in the simulator too (CR included), so the first RND depends on the typing before RUN there as well (v3.1: always -7764).
;     Hardware trade-off: v3.1's CHIN stepped the LFSR once per idle-line poll (keypress-timing entropy); that source is gone.
;   - GET_RND removed.  DO_RND does the mix itself: 8 LFSR steps per RND, result = the NEW seed as a float (v3.1: 1 step, the OLD seed).
;     DO_RND grew 4 bytes; EXPR_ATOM's BCTR,EQ DO_RND became BCTA,EQ (DO_RND is now beyond relative range), +1 byte.
;   - Comments: DO_RND given its own header (the EXPR header had been sitting above it, now back above EXPR); EXPR_ATOM no longer cites
;     TRY_RND/GET_RND; header RND text and KNOWN LIMITATIONS RND bullet rewritten.
;   - Showcase lines 78 (five raw RNDs) and 79 (three 0..1 values) added after line 77.  Boot FREE 1724 -> 1608.  Banner 3.1 -> 3.2.
;   - Tested (simulator): showcase output identical to v3.1 apart from banner/FREE and the RND lines; RND_SHUFFLE keeps R1-R3 and
;     matches v3.1's LFSR step; RND values match a host LFSR model (1 step per typed char, 8 per RND); prompt-level RND in
;     expressions, assignment, IF, GOSUB OK.  Not testable in the simulator: the bit-banged CHIN body itself (intercepted).
;
; v3.1 (Oct 2026) - Showcase converted to floating point (Stage 4 of the miniBASIC2650 plan).  NO code change except the banner text
;   ('miniBASIC2650 3.0' -> '3.1', same length): ROMEND $0D8A = 3466 bytes, library and interpreter bytes identical to v3.0.
;   - Mandelbrot: lines 620/625/630 lose the /64 fixed-point scaling (T=A*A-B*B+C, B=2*A*B+D, test 4<A*A+B*B); 320/350 give real coordinates
;     D=(R-10)/8 (-1.25..1.25, step 0.125 = twice the column step: square pixels on 2:1 character cells) and C=Q/16-2.25 (-2.25..0.4375).
;     Line 305 (M=16, the iteration limit) is unitless and unchanged.  Same 44 x 21 plot; the set is now symmetric about row 10 (D=0 is sampled).
;   - New showcase lines: 71-77 true division, decimals, 6-digit printing, no 16-bit wrap, RND and ABS(RND)/32768; 133-134 decimal compares;
;     271-274 FOR with STEP 0.25.  Title line 20 now says miniBASIC2650.  CHR$/TAB( lines 40/44/430/440 work unchanged through FIX (truncation).
;   - KNOWN LIMITATIONS rewritten (E notation, fractional STEP error, RND, ?Z ?O ?R, program store).  Boot FREE 2169 -> 1724.
;   - Open items settled by the owner: RND stays the integer seed as float (no 0<=RND<1 code), E notation stays a documented limitation, no
;     page-zero vectors for PRT_INT / FIX_EXP.
;   - Layout only (no revision change): every 'LABEL: INSTRUCTION' pair now has the label on a line of its own (the Signetics assembler
;     requires it); 90 lines split, instruction and comment columns kept.  Hex byte-identical.  Name/EQU/RES/DB definitions unchanged.
;
; v3.0 (Oct 2026) - 32-bit MBF4 floating point (Stage 3 of the miniBASIC2650 plan).  ROMEND ROMEND $0D8A = 3466 bytes.
;   - Every numeric value is MBF4.  Expression result and right operand: FA; the left operand pops into FB (OPS_HIT_RET); EXP (EXPH:EXPL)
;     stays the 16-bit INTEGER register (FLT_TO_INT out / FLT_FROM_INT in) so the IPH-relative pair table and REG16_TO_REG16 are untouched.
;   - Library mbf4_lib.asm v1.0 (1583 bytes) spliced in verbatim, FP code runs in the alternate register bank (PPSL $10 ... CPSL $10 at every
;     call site; the library itself is entered/left in that bracket).  R2 (variable) and R3 (SW-stack index) survive every expression.
;   - Operator frame on SWBASE 5 bytes (FA + row offset) + continuation; SWBASE 96 bytes, SWCAP_LIMIT 92.  VARS stride 4 (104 bytes).
;     FOR frame 11 bytes [var][limit 4][step 4][body 2], FSTKLIM 44 (4 nested loops), real limit and STEP.  Booleans are -1.0 / 0.0.
;   - New glue: PF_INP/PF_NUM/PF_INT (literals; PF_NUM TAIL-JUMPS into FLT_PARSE: a BSTA would reach hardware depth 8 on a FOR literal),
;     FIX_EXP, PRT_INT, PRT_FA, FA_TO_BLK/BLK_TO_FB.  DO_ERROR starts with CPSU $07 (error print independent of the depth the error came from).
;   - Deleted (now FP): ADD_CORE, CMP_OPS, NEG_EXP/ABS_TMP/NEG_SHARED, ABS_EXP, MULT_LOOP, DIV16/MUL16, PARSE_S16/PARSE_U16, PRINT_S16, CLR_EXP,
;     the power-of-10 tables, NEGFLG, vectors VCLR_EXP and VNEG_EXP_BODY.  Banner 'miniBASIC2650 3.0'.  Depth: peak 7 (tools/depthtrace.py).
;
; v2.11 (Sep 2026) - Code golf: REG16_TO_REG16 generic 16-bit copy.
;   - Replaced EXP16_TO_ET/TMP_TO_ET and DE_LOOP, DR_SWSTK,DN_LIM, GR_LP, OG_SAV
;     copy loops with REG16_TO_REG16 using R0 = packed (SRC_IDX<<4)|DST_IDX pair. 
;   - RAM reorg: PEH/PEL and SC0/SC1 moved contiguous for use with REG16.
;   - ROMEND $07EF (2031) -> $07D0 (2000 bytes), -31.
;
; v2.10 (Sep 2026) - BUG FIX: mid-list line insert corrupted following line.
;   - OPEN_GAP's move loop decrements PE via an inlined former DEC_PE.
;     However still used "RETC,EQ" for fast path so bailed early. Fixed by 
;     jumping to where subroutine end would be.
;     ROMEND $07EF (2031 bytes)
;
; v2.9 (Sep 2026) - Added ABS(x) function to EXPR, and FREE memory statement.
;   -  ROMEND $07C5 (1989) -> $07EE (2030 bytes).
;
; v2.8 (Sep 2026) - Code golf, ported from pBASIC2650.asm v0.33.
;   - New ADD_CORE: shared 16-bit indexed add (EXPH:EXPL += *(IPH+off)) used by
;     DO_ADD and MULT_LOOP, deleting CARRY_INTO_EXPH. 
;   - Minor Refactor ABS_EXP for size.
;   - DV_LP: removed leading CPSL $08 (matches pBASIC's) global invariant.
;   - ROMEND $07E1 (2017) -> $07C7 (1991 bytes).
;
; v2.7 (Sep 2026) - BASIC line editing, LET/THEN/INPUT and FOR/NEXT restored.
;   - TRY_STORE_LINE: FIND_LINE; if the line exists DEL_REC removes it and
;     FIND_LINE is repeated.  New DEL_REC (fwd copy), OPEN_GAP (backward copy)
;     and DEC_PE. Line numbers > 32767 rejected with syntax error. 
;   - PROGLIM (EQU $1FFF, must be $xxFF): OPEN_GAP builds the new PE in EXP,
;     checks its hi byte, and only then moves anything; store full gives ?3
;     (ERR_OOM) with the store untouched (+10). A REPLACE that does not fit has
;     already deleted the old line. 
;   - Relop refactor, added '>': 3-way compare, each handler picks its CC (EQ/LT/GT).
;   - FOR/NEXT: body runs at least once.
;   - FOR/NEXT frame is pushed/popped by loops 
;   - STEP added, frame widened to 7 bytes, holding [var][limit lo/hi]
;     [step lo/hi][body lo/hi]; 
;   - SHOWCASE updated for new keywords amd relops.
;   - Golf pass - duplicate digit test (LODA,R0 *IPH; SUBI,R0 A'0';
;     COMI,R0 9) inlined in TRY_STORE_LINE and PARSE_U16, now DIGIT_CHECK.
;     ROMEND $07FA (2042) -> $07F5 (2037 bytes);
;
; v2.6 (Sep 2026) - Code-golf and Bigfix.
;   - STMT_EXEC/MD_SCAN: dispatch scan now starts at 1st STATEMENT row, skipping
;     operator rows to fix crash bug when a line started with an operator.
;   - BUG FIX: TSL_CPYDONE's "tail call" into TMP_TO_ET used BSTA instead
;     of BCTA.
;   - BUG FIX DR_LP only cleared RUNFLG when TMP>PE (overrun), not equal. 
;   - PARSE_VAR_SAVE/PARSE_FACTOR/TRY_STORE_LINE/PU16/EATWORD: two-comparison 
;     signed replaced with SUBI+COMI+single unsigned branch throughout. 
;   - PARSE_VAR_SAVE now threads the VARS byte-offset (index*2) through R2
;     instead of the raw letter;
;   - FIND_LINE/FIND_INS: INC16_TMP_TO_EXP deleted.
;   - DO_EQOP/DO_LTOP: intermediate compare byte now parked in R1 not SC0.
;   - DIV16: remainder was unused, no MOD operator. DV_LP/DV_SUB/DV_SNB's 
;     refactored into destructive subtract-and-test-borrow pass
;   - PRINT_S16: removed the R2 save/restore 
;   - DR_HDR: refactored to use shared TMP_TO_ET call 
;   - NEGFLG polarity flipped to 0=negate across PARSE_S16,ABS_TMP, ABS_EXP,
;     NEG_EXP, DO_MUL, MU_DONE. 
;   - IBUF moved to $1010 so GETLINE now sets IPH:IPL with one LODI.
;   - MUL16's multiply loop and PU16's x10 shareMULT_LOOP leaf sub. PU16_DIG
;     doesnt need EXP16_TO_TMP so deleted.
;   - ROMEND: $079C -> $06f6 (1948 -> 1782 bytes)
; v2.5 (Sep 2026) - Added niladic RND to the expression parser.
;   - EXPR_ATOM: added 'R'-then-alpha check - cant use PEEK_C2_ALPHA due to R2
;   - Golf pass, all over.
;   - ROMEND: $0800 after RND detection -> $07AF (1976 bytes)
;   - Stored-program end of line terminatorchanged to NUL from CR, matches what
;     STMT_EXEC/EXPR/EATWORD/etc need so DO_RUN executes straight out of PROG. 
;     Updated Showcase's terminator lines to match.
;   - ROMEND: $07AF -> $0797 (1943 bytes)
; v2.4 (Sep 2026) - Added GOSUB/RETURN; golf pass; showcase exercises both.
;   - GOSUB/RETURN: TOK_CHARS only matches 1st ketter, so GOTO/GOSUB share a
;     row (as do RUN/RETURN) 
;   - Showcase: added "--- GOSUB/RETURN ---" section and updated Mandelbrot.
;   - Added RND_Shuffle pseduo random sequnce ready for RND keyword.
;   - ROMEND: $0774 -> $07DA 
; v2.3 (Sep 2026) - Fixed broken Mandelbrot output; golf pass.
;   - Golf pass
;   - ROMEND: $0784 -> $0774 (1925 -> 1908 bytes)
; v2.2 (Sep 2026) - CHR$(n) and TAB(n) added to PRINT; WR statement removed
;   - Showcase updated
;   - ROMEND: $0785 (1925 bytes)
; v2.1 (Sep 2026) - Recursive parens SW stack BODMAS-lite restored on top.
;   - SWBASE grown to 64 bytes; MU_DONE now pushes HI_LOOP continuation so
;     */  chaining (e.g. "2*3*4", "100/5/2") evaluates left-to-right correctly.
;   - ROMEND: $0767 (1895 bytes)
; V2.0 (sep 2026) - 
;   - BODMAS layered on afterward: every "give me a fully-resolved value"
;     site now pushes {LO_LOOP, HI_LOOP} (HI checked first); LO operators'
;     own right-operand ALSO pushes {HI_LOOP} (so "3+4*5" groups as
;     "3+(4*5)"); HI operators' own right-operand does not - chaining
;     ("2*3*4") happens via DO_MUL/DO_DIV looping back to HI_LOOP directly,
;     preserving left-to-right associativity (required for correct "/"
;     chaining, e.g. "100/5/2"). TOK_CHARS reordered (*/  first) so each
;     tier's scan is a contiguous sub-range; DO_MUL's RXSAVE-based MUL-vs-
;     DIV check re-derived from the new table position (offset 2, not 8)
;     by simulator trace, not assumed.
;   - ROMEND: $075D (1885 bytes)
;
; Branched from pBASIC V0.22
; =============================================================================

;  ASCII Defines
CR      EQU     $0D
LF      EQU     $0A
SP      EQU     $20
NUL     EQU     $00
DQ      EQU     $22

;  ERROR Defines
ERR_SYN         EQU '0'
; ERR_UND_LINE    EQU '1'         ; unused
ERR_OOM         EQU '3'         ; GOSUB/FOR stack full, RETURN/NEXT with nothing pushed, program store full
ERR_VAR         EQU '4'
ERR_EXPR        EQU '8'         ; Expression too complex (SW stack full)
GSSTKLIM        EQU 8           ; 4 GOSUB levels x 2 bytes
FSTKLIM         EQU 44          ; 4 FOR levels x 11 bytes
PROGLIM         EQU $1FFF       ; last usable program-store address. MUST be $xxFF (one
                                ; below a page boundary): OPEN_GAP checks the new PE's
                                ; hi byte only. Change for other RAM sizes (1 KB: $13FF)

; Error check to make sure we dont crash due to HW Stack overflow 
PE_RAS_LIMIT    EQU 7

; PSW Defines
PSW_RS          EQU     $10
PSW_WC          EQU     $08             ; WC (With Carry) bit in PSL (bit 3)
PSW_FLAG        EQU     $40

; REG16_TO_REG16 register indices (idx*2 = byte offset from IPH; see the
; "Ordered group" RAM comment). v2.11.
IDX_IP    EQU 0
IDX_TMP   EQU 1
IDX_GOTO  EQU 2
IDX_CUR   EQU 3
IDX_SWSTK EQU 4
IDX_EXP   EQU 5
IDX_LNUM  EQU 6
IDX_PE    EQU 8
IDX_SC0   EQU 9
IDX_RND   EQU 10

;  CODE starts at Zero (No Pipbug)
        ORG 0

; =============================================================================
;  RESET / ENTRY + PAGE-ZERO VECTOR TABLE
;
; Page-zero subroutine vector table.
; Each DW entry holds the absolute address of the subroutine.
; Callers use ZBRR/ZBSR *Vxxx (2 bytes) vs BCTA/BSTA,UN xxx (3 bytes)
;
RESET:
        BCTR,UN MAIN            ; trampoline over vector table ($0000)
;  Page-zero vector table for ZBSR/ZBRR *Vxxx (2 bytes vs 3 for BSTA/BCTA).
;  A vector costs 2 bytes and saves 1 per call, so an entry pays for itself
;  only from 3 callers up: keep every entry >= 3, and re-count after any
;  change that adds or removes calls. Only unconditional BSTA/BCTA convert
;  (there is no conditional ZB form). Reach: ZBSR/ZBRR take a 7-bit signed
;  displacement, so entries must sit at addresses $02..$3E (max 31 entries).
;  Do not point a vector at a routine that a "db $EC" skip trick skips over
;  unless the routine stays 2 bytes.
VINC_IP:
        DW INC_IP
VWSKIP:
        DW WSKIP
VINC_TMP:
        DW INC_TMP
VCOUT:
        DW COUT
VPARSE_EXPR:
        DW EXPR
VPRT_SPACE:
        DW PRT_SPACE
VDO_ERROR:
        DW DO_ERROR
VSET_TMP_PROG:
        DW SET_TMP_PROG
VCMP_TMP_PE:
        DW CMP_TMP_PE
VREG16_TO_REG16:
        DW REG16_TO_REG16    ; v2.11: generic nibble-packed 16-bit copy
VPUSH_RET:
        DW PUSH_RET
VPARSER_RET:
        DW PARSER_RET
VEATWORD:
        DW EATWORD
VEXPR_ATOM:
        DW EXPR_ATOM
VINC_ET:
        DW INC_ET
VDIGIT_CHECK:
        DW DIGIT_CHECK

; =============================================================================
; MAIN - Program init
; =============================================================================
MAIN:

        ; Initialize RND seed
        LODI,R1 $AC
        LODI,R0 $E1
        BSTA,UN RND_SKIP

        PPSL $02                ; COM=1 (unsigned compare mode) for the entire

        ; clear Run flag - change to DO_NEW for ROM
        BSTA,UN CLR_RUNFLG             

        ; print sign-on banner
        LODI,R0 <BANNER
        STRA,R0 IPH
        LODI,R0 >BANNER
        STRA,R0 IPL
        BSTA,UN PRTSTR
        BSTR,UN DO_FREE         ; free memory 
        ; fall through to REPL

; =============================================================================
;  REPL -- Main read-eval-print loop
; =============================================================================
REPL:
        CPSL PSW_RS + 5             ; primary reg bank; clear C/OVF/RS -
        CPSU $07                    ; clear PSU SP field (bits 2:0 = HW RAS depth)
        LODI,R0 '>'                    ; print prompt only used here
        ZBSR *VCOUT  
        ZBSR *VPRT_SPACE  
        BSTA,UN GETLINE                  ; also sets IPH:IPL = IBUF
        BSTA,UN TRY_STORE_LINE           ; CC=GT: line stored/deleted; CC=EQ: not a line
        BSTA,EQ STMT_EXEC               ; If CC=EQ (not a line), execute
        BCTR,UN REPL

; =============================================================================
;  JSYNERR -- Syntax error jump
; In:  nothing (R0 irrelevant)
; Out: jumps to DO_ERROR
; Clobbers: R0
JSYNERR:
        LODI,R0 ERR_SYN
        ; drop through
; =============================================================================
;  DO_ERROR -- Print error, clear run state, return to REPL
; Entry: R0 = error code character ('0'..'8', or the FP letters 'O' overflow / 'Z' divide by zero / 'R' integer range).
; Clears RUNFLG, SWSP, FORSP. Prints "?n" or "?n@line" if running (the line number goes through PRT_INT).
; CPSU $07 first: the hardware RAS depth is reset, so the print path never depends on how deep the error was raised
; (the FP error exits abandon pending return addresses).  Tail-jumps to REPL (clears the RAS again).
; In:  R0 = error code
; Out: jumps to REPL
; Clobbers: all (RAS cleared by REPL)
DO_ERROR:
        CPSU $07                        ; hardware RAS depth := 0: the print below never depends on where the error came from
        STRZ,R1                         ; Save ASCII error code
        BSTR,UN PRT_QUEST               ; Print Question mark
        LODZ,R1
        ZBSR *VCOUT                     ; print error code
        LODA,R0 RUNFLG                  ; OPT-10: SC1=RUNFLG, 0->EQ, 1->GT
        BCTR,EQ DE_NL                   ; not running, no line number
        LODI,R0 '@'                     
        ZBSR *VCOUT                     ; Print at line
        LODI,R0 (IDX_CUR*16)+IDX_EXP
        ZBSR *VREG16_TO_REG16
        BSTA,UN PRT_INT                  ; [+1]
DE_NL:
        BSTR,UN PRT_CRLF
        BSTA,UN CLR_RUNFLG                ; Not running
        BCTR,UN REPL                     ; REPL resets RAS (PSU SP bits) on entry

; =============================================================================
; DO_FREE
; Syntax: FREE
; Prints the number of free bytes in program store: PROGLIM - PEH:PEL.
; PROGLIM = $1FFF (top of RAM). Free = $1FFF - current program end pointer.
; Note May need to change if PROGLIM is not 0x1FF 
; In:   PEH:PEL = program end pointer
; Out:  free byte count printed to COUT followed by CR/LF
; Clobbers: R0, EXPH, EXPL (via PRINT_S16)
; =============================================================================
DO_FREE:
        ; Compute Low Byte: EXPL = $FF - PEL (Never borrows, may change)
        LODI,R0 >PROGLIM                 ; Load $FF
        SUBA,R0 PEL 
        STRA,R0 EXPL 

        ; Compute High Byte: EXPH = $1F - PEH
        LODI,R0 <PROGLIM                 ; Load $1F
        SUBA,R0 PEH 
        STRA,R0 EXPH 

        BSTA,UN PRT_INT                  ; Print decimal value
        ; drop through
; =============================================================================
;  Shared character print routines -- $EC (COMA) byte-skip chain
; Each entry loads its character then falls through via the skip opcode trick.
PRT_CRLF:
        BSTR,UN PRT_CR                   ; Print CR/LF
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
;  DO_IF -- Conditional execution; THEN is optional.
;  Nests for free: "IF a IF b stmt" - the true-path dispatch is a JUMP to
;  STMT_EXEC, so if "stmt" is itself another IF, it costs no extra depth.
; Syntax: IF expr [THEN] stmt   (THEN is skipped by the 'T' row of TOK_CHARS)
; In:  IP -> first char after IF keyword
; Out: executes stmt if expr is nonzero; otherwise sequential return
; Clobbers: R0, R1, FA, FB, EXPH, EXPL (via EXPR, plus whatever the dispatched
;           statement clobbers on the true path).  False = FA exponent byte 0 (0.0).
DO_IF:
        LODA,R0 RXSAVE
        COMI,R0 A'P'            ; INPUT
        BCTA,EQ DO_ASK          ; 

        ZBSR *VPARSE_EXPR                      ; [+1] condition -> EXPH:EXPL
        LODA,R0 FA
        RETC,EQ                           ; FA exponent byte 0 = 0.0: false, return
        ; drop through
; =============================================================================
;  STMT_EXEC -- Decode and dispatch one BASIC statement from IP.
; In:  IPH:IPL -> first char of statement (after any leading whitespace)
; Out: control jumps to the matched DO_xxx handler, or falls into SE_NOTKW
; Clobbers: R0, R2, R1, GOTOH, GOTOL, IPH:IPL advanced past the keyword
;   on a match (unchanged on the bare-assignment path - see SE_NOTKW)
; RAS depth: 1 from REPL, 3 from DO_IF's true path (which jumps here).
STMT_EXEC:
        ZBSR *VWSKIP
        BSTA,UN PEEK_C2_ALPHA     ; R2 is 1st char
        BCFA,EQ SE_NOTKW          ; 2nd char is NOT a letter -> handle as var/expr
        LODI,R1 2                 ; peek 3rd char - disambiguates GOTO/GOSUB
        LODA,R0 *IPH,R1           ; and RUN/RETURN inside DO_GO/DO_RU
        STRA,R0 RXSAVE
        LODI,R1 21                ; start scan at the first STATEMENT row -
                                   ; skips the 7 operator rows (21 = 7*3) so
                                   ; an operator-led line (e.g. "-A", 2nd
                                   ; char is a letter) can never match one
                                   ; and mis-dispatch into a DO_xxx handler;
                                   ; falls through to SE_NOTKW instead (see
                                   ; pBASIC PAREN-fix history, same table)
MD_SCAN:
        LODA,R0 TOK_CHARS,R1              ; table char
        BCTR,EQ SE_NOTKW                  ; NUL row: no match -> bare assignment
        COMZ,R2                          ; 1 byte vs SUBA's 3 (r0:r2 -> CC);
        BCTR,EQ MD_HIT
        ADDI,R1 3                         ; next row (char + 2-byte handler)
        BCTR,UN MD_SCAN
MD_HIT:
        ZBSR *VEATWORD                  ; consume the whole keyword -
        ; Jump into point from relop vectors
JMP_VEC:
        LODA,R0 TOK_CHARS,R1+              ; handler hi (pre-inc: char->hi)
        STRA,R0 GOTOH
        LODA,R0 TOK_CHARS,R1+              ; handler lo (pre-inc: hi->lo)
        STRA,R0 GOTOL
        BCTA,UN *GOTOH                    ; indirect jump

; from EXPR operators
; =============================================================================
;  OPS_HIT_RET -- continuation after the right operand: pop the row offset and the LEFT operand (4 bytes) and dispatch the operator.
; In:  SWBASE top = [row offset][FA3][FA2][FA1][FA0] (pushed by OPS_HIT_COM); FA = right operand
; Out: FB = left operand, R1 = TOK_CHARS row offset, jump through JMP_VEC to DO_ADD/SUB/MUL/DIV/EQOP/LTOP/GTOP
; Clobbers: R0, R1, R3 (popped by 5), FB
OPS_HIT_RET:
        ; Pop the R1 table offset back off the stack
        LODA,R0 SWBASE,R3        ; R0 = top of stack (our saved R1 offset)
        STRZ,R1                  ; R1 = R0 
;  POP the left operand (4 bytes, pushed FA0..FA3) into FB
        LODA,R0 SWBASE,R3-      ; predecrement
        STRA,R0 FB+3
        LODA,R0 SWBASE,R3-
        STRA,R0 FB+2
        LODA,R0 SWBASE,R3-
        STRA,R0 FB+1
        LODA,R0 SWBASE,R3-
        STRA,R0 FB
        SUBI,R3 1
        ; jump to vector
        BCTR,UN JMP_VEC

SE_NOTKW:
        ; Bare variable assignment ("X=expr" - either the 2nd-char peek
        ; above wasn't a letter, or the 1st char matched no statement).
        BSTR,UN PARSE_VAR_SAVE            ; validates A-Z, R2 = letter, IP -> past it
        ZBSR *VWSKIP
        COMI,R0 A'='
        BCFA,EQ JSYNERR
        ZBSR *VINC_IP
        ; drop through
; =============================================================================
;  DL_EX / DL_STORE -- Variable assignment (v0.3 removed DO_LET's own
;  prologue; v2.7 brings the optional LET keyword back at no handler cost:
;  the 'L' row lands in DO_LIST, which sends LET straight to SE_NOTKW.  Reached
;  from SE_NOTKW's "V=expr" path (bare or after LET) and from DO_ASK.)
; In:  IP -> expression (DL_EX) or R2 = VARS byte-offset, FA = value
;      (DL_STORE - PARSE_VAR_SAVE already converts the letter to R2=index*4)
; Out: VARS[V..V+3] = FA
; Clobbers: R0 (DL_EX also: FA, FB, EXPH, EXPL via PARSE_EXPR and the FP library)
DL_EX:
        ZBSR *VPARSE_EXPR                 ; [+1]
DL_STORE:
        LODA,R0 FA       ; VARS[R2..R2+3] = FA (4 bytes, exponent first); R2 = letter*4
        STRA,R0 VARS,R2
        LODA,R0 FA+1
        STRA,R0 VARS+1,R2
        LODA,R0 FA+2
        STRA,R0 VARS+2,R2
        LODA,R0 FA+3
        STRA,R0 VARS+3,R2
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
        BSTR,UN GETLINE                   ; [+1] also sets IPH:IPL = IBUF
        BSTA,UN PF_INP                   ; [+1] FA = [+-]digits[.digits]
        BCTR,UN DL_STORE

; =============================================================================
; PARSE_VAR_SAVE -- skip whitespace, read var letter, range-check, save to
;                   R2, advance IP.
; Out: R2=letter (A-Z); IP advanced past letter
; Error: tail-jumps to JERRVAR (no return)
; Clobbers: R0, R1, R2
PARSE_VAR_SAVE:
        ZBSR *VWSKIP
        SUBI,R0 A'A'                     ; shift 'A'-'Z' down to 0-25
        COMI,R0 A'Z'-A'A'                ; single unsigned range test (COM=1
        BCTR,GT JERRVAR                  ; set globally) catches < 'A' and > 'Z'
        ADDZ,R0
        ADDZ,R0                          ; R0 = index*4 (MBF4 stride)
        STRZ,R2                          ; R2 = byte offset in VARS, for DL_STORE
        ZBRR *VINC_IP         ; tail call  

; =============================================================================
;  GETLINE -- Read a line from input into IBUF; also points IP at it
; In:  nothing
; Out: IBUF = NUL-terminated input line; IPH:IPL = IBUF (both callers used
;      to re-point IP via a separate SET_IP_IBUF call right after this -
;      folded in here instead). Was a single shared LODI while IBUF sat at
;      a hi==lo address; IBUF moved (v2.11, nibble-index RAM reorg) so this
;      is now two LODIs (+2 bytes, accepted cost - see VERSION HISTORY)
; Clobbers: R0, R1
GETLINE:
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
        LODI,R1 $FF                      ; R1 = empty-buffer sentinel (pre-inc convention)
GL_LP:
        BSTA,UN PRE_CHIN                 ; [+1] blocking read (stirs the RND LFSR first)
        COMI,R0 CR+1
        BCTR,LT GL_EOL                  ; everything less than CR
        STRA,R0 IBUF,R1+                 ; R1++ (pre-inc); IBUF[R1]=char
        ZBSR *VCOUT  
        BCTR,UN GL_LP
GL_EOL:
        EORZ,R0
        STRA,R0 IBUF,R1+                 ; R1++ (pre-inc, one past last char); NUL-terminate
        BCTA,UN PRT_CRLF                ; tail call

JERRVAR:
        LODI,R0 ERR_VAR
        db $EC                  ; COMA,R0: consume next 2 bytes
DGS_OFLOW:                              ; all report ERR_OOM: GOSUB/FOR stack full,
DRT_UFLOW:                              ; RETURN/NEXT with nothing pushed, program
        LODI,R0 ERR_OOM                 ; store full (OPEN_GAP) - one shared exit
        ZBRR *VDO_ERROR

DO_GO:
        LODA,R0 RXSAVE
        COMI,R0 A'S'
        BCFR,EQ DO_GOTO
        ; drop through
; =============================================================================
;  DO_GOSUB -- Subroutine call, up to GSSTKLIM/2 levels deep.  
; In:  IP -> first char after keyword; SWSTK already holds "resume after
;      this line" (DR_LP sets it before every STMT_EXEC dispatch - see
;      DR_CD), i.e. exactly the return point GOSUB needs to save.
; Out: GSSTK[GSSP]=SWSTK, GSSP+=2, then behaves exactly like GOTO to
;      redirect SWSTK at the target line.
; Error: GSSP=GSSTKLIM (stack full) -> ERR_OOM
; Clobbers: R0, R1, GSSP, GSSTK (plus everything DO_GOTO clobbers)
DO_GOSUB:
        LODA,R1 GSSP
        COMI,R1 GSSTKLIM
        BCFR,LT DGS_OFLOW                 ; GSSP >= limit: stack full
        LODA,R0 SWSTK
        STRA,R0 GSSTK,R1
        LODA,R0 SWSTK+1
        STRA,R0 GSSTK+1,R1
        ADDI,R1 2
        STRA,R1 GSSP      
        ; drop through
; =============================================================================
;  DO_GOTO -- Computed GOTO
; Syntax: GOTO expr
; In:  IP -> first char after GOTO keyword
; Out: if running, SWSTK = found record pointer (DR_CD resumes from there
;      unconditionally - see DR_CD); if not running (typed at the prompt,
;      outside RUN), a safe no-op, same as before.
; Clobbers: R0, EXPH, EXPL, LNUMH, LNUML, TMPH, TMPL, SWSTK (only if running)
DO_GOTO:
        ZBSR *VPARSE_EXPR                 ; [+1] EXPR_ATOM WSKIPs itself
        LODA,R0 RUNFLG
        RETC,EQ                           ; not running: safe no-op
        BSTA,UN FIX_EXP                   ; EXP = int16(FA), truncating (|x| >= 32768: ?R)
        LODI,R0 (IDX_EXP*16)+IDX_LNUM
        ZBSR *VREG16_TO_REG16              ; LNUMH:LNUML = EXPH:EXPL (target line)
        BSTA,UN FIND_LINE                  ; [+1] TMPH:TMPL = found record
        LODI,R0 (IDX_TMP*16)+IDX_SWSTK
        ZBRR *VREG16_TO_REG16              ; Tail call

; =============================================================================
;  DO_RETURN -- Return from subroutine.  Reached only from DO_RU.
; Syntax: RETURN
; Out: GSSP-=2; SWSTK = GSSTK[GSSP] (popped return point).
; Error: GSSP=0 (nothing pushed) -> ERR_OOM
; Clobbers: R0, R1, GSSP, SWSTK
DO_RETURN:
        LODA,R0 GSSP
        BCTR,EQ DRT_UFLOW                 ; GSSP==0: nothing to return to
        SUBI,R0 2
        STRA,R0 GSSP
        STRZ,R1                           ; R1 = popped-frame offset
        LODA,R0 GSSTK,R1
        STRA,R0 SWSTK
        LODA,R0 GSSTK+1,R1
        STRA,R0 SWSTK+1
        RETC,UN

; =============================================================================
;  DO_RU -- 'R' dispatch: RUN or RETURN, same reason as DO_GO (RETURN's 3rd
;  char is 'T', RUN's is 'N').  
DO_RU:
        LODA,R0 RXSAVE
        COMI,R0 A'M'    ; REM
        RETC,EQ         ; Rem just returns as does nothing
        COMI,R0 A'T'
        BCTR,EQ DO_RETURN
        ; fall through: not RETURN -> plain RUN
; =============================================================================
;  DO_RUN -- Execute stored program
; Syntax: RUN
; In:  PROG=program base, PEH:PEL=program end
; Out: runs until END, error, or exhausted; returns to REPL
; Clobbers: all
; =============================================================================
DO_RUN:
        LODI,R0 1
        STRA,R0 RUNFLG
        ZBSR *VSET_TMP_PROG
DR_LP:
        LODA,R0 RUNFLG
        RETC,EQ
        ; end of program? TMPH:TMPL >= PEH:PEL
        ZBSR *VCMP_TMP_PE

        BCFR,LT CLR_RUNFLG                ; not LT (GT or EQ): TMP>=PE, end
                                          ; of program - clear RUNFLG and
                                          ; return via CLR_RUNFLG's own
                                          ; RETC,UN. Bug fix: previously
                                          ; "BCTR,GT CLR_RUNFLG / RETC,EQ"
                                          ; left RUNFLG=1 on a clean end
                                          ; (TMP==PE exactly), so the next
                                          ; error typed at the prompt wrongly
                                          ; printed a stale "@line" suffix.

        ; save current line number for error reporting.  Indexed read only -
        ; TMP itself stays at the record's header start, which is exactly
        ; what ADV_TMP_PAST_REC (below) expects.
        LODI,R1 2
DR_HDR:
        LODA,R0 *TMPH,R1-                 ; R1 2->1: TMP[1]; 1->0: TMP[0]
        STRA,R0 CURH,R1                   ; CURH+1 = CURL
        BRNR,R1 DR_HDR
        ; IP = TMP (header start), via the shared TMP_TO_ET copy, then
        ; skip past the 2-byte header to the body - execute straight out
        ; of PROG, no copy. Bodies are NUL-terminated in storage now, so
        ; every NUL-terminated-string assumption already baked into
        ; STMT_EXEC/EXPR/etc. just works.
        LODI,R0 IDX_TMP*16                ; +IDX_IP (0)
        ZBSR *VREG16_TO_REG16
        
        ZBSR *VINC_IP
        ZBSR *VINC_IP
        ; advance TMP past this whole record (header+body+terminator) to
        ; find the next one; stash for SWSTK/DR_LP's next iteration (or a
        ; GOTO's redirection). Reuses the same shared scan FIND_LINE/
        ; FIND_INS/TSL_MATCH already use - one routine, one terminator.
        BSTA,UN ADV_TMP_PAST_REC
        LODI,R0 (IDX_TMP*16)+IDX_SWSTK
        ZBSR *VREG16_TO_REG16
        ; execute line
        BSTA,UN STMT_EXEC                ; [+1]

        ; resume from SWSTK unconditionally
        LODI,R0 (IDX_SWSTK*16)+IDX_TMP
        ZBSR *VREG16_TO_REG16
        BCTR,UN DR_LP

; =============================================================================
;  DO_NEW -- Reset PE and IP
; Syntax: NEW
; In:  nothing
DO_NEW:
        ; PEH:PEL = PROG.
        LODI,R0 <PROG
        STRA,R0 PEH
        LODI,R0 >PROG
        STRA,R0 PEL 
        ; fall through
; =============================================================================
;  DO_END -- Stop execution and clear all run state
; Syntax: END  (also called by DO_NEW, DO_ERROR, RESET)
; In:  nothing
; Out: RUNFLG=0
; Clobbers: R0
;DO_END:
CLR_RUNFLG:
        EORZ,R0
        STRA,R0 RUNFLG
        STRA,R0 GSSP        ; GOSUB stack empty at every RUNFLG-clear point
        STRA,R0 FSP         ; ... and the FOR stack
        RETC,UN

; =============================================================================
;  DO_N / DO_NEXT -- 'N' row: NEW's 3rd char is 'W', NEXT's is 'X'.
;  NEXT [var]: adds 1 to the innermost loop's variable (any var name after
;  NEXT is ignored); while var <= limit it jumps back to the body, else pops.
;  The 11-byte frame is read back with pre-decrement loads into FWRK; var += step and var : limit use FLT_ADD / FLT_CMP.
;  Loops again while (step >= 0 ? var <= limit : var >= limit) - the step's sign is bit 7 of FWRK+6.
; In:  FSP = bytes used
; Out: var+step stored; SWSTK = body (loop again) or the frame popped (finished)
;      Error: ERR_OOM if there is no FOR active
; Clobbers: R0, R1, R2, R3, FA, FB, FWRK and the FP scratch  (R3 = frame index, kept live: FP code works in the alternate bank)
DO_N:
        LODA,R0 RXSAVE
        COMI,R0 A'X'
        BCFR,EQ DO_NEW
DO_NEXT:
        LODA,R3 FSP
        BCTA,EQ DRT_UFLOW                ; NEXT with no FOR
        SUBI,R3 11                       ; R3 = frame base
        LODI,R2 9
DN_POP9:
        LODA,R0 FSTK-1,R3+               ; frame bytes 0..8 -> FWRK+8 .. FWRK+0 (var, limit, step)
        STRA,R0 FWRK,R2-
        BRNR,R2 DN_POP9                  ; R3 = base+9: the body pointer follows
        LODA,R2 FWRK                     ; R2 = var's VARS offset (for DL_STORE)
        LODA,R1 FWRK                     ; R1 = the same (for PF_LV)
        BSTA,UN PF_LV                    ; FA = var
        LODI,R1 4
        BSTA,UN BLK_TO_FB                ; FB = step
        PPSL PSW_RS
        BSTA,UN FLT_ADD                  ; FA = var + step
        CPSL PSW_RS
        BSTA,UN DL_STORE                 ; var = var + step
        LODI,R1 0
        BSTA,UN BLK_TO_FB                ; FB = limit
        PPSL PSW_RS
        BSTA,UN FLT_CMP                  ; CC = var : limit  (CPSL keeps CC)
        CPSL PSW_RS
        BCTR,EQ DN_GO                    ; var = limit: one more pass
        BCTR,GT DN_UP                    ; var above limit
        LODA,R0 FWRK+6                   ; var below limit: still going unless the step is negative
        BCTR,LT DN_POP                   ;   (bit 7 of the step's S|M1 byte)
        BCTR,UN DN_GO
DN_UP:
        LODA,R0 FWRK+6                   ; var above limit: going only if step < 0
        BCTR,LT DN_GO
DN_POP:
        SUBI,R3 9                        ; back to the frame base: pop it
        STRA,R3 FSP
        RETC,UN

DN_GO:
        LODI,R2 2
DN_BODY:
        LODA,R0 FSTK-1,R3+               ; body lo, body hi -> SWSTK: DR_LP
        STRA,R0 SWSTK,R2-                ; resumes there
        BRNR,R2 DN_BODY
        RETC,UN

; -----------------------------------------------------------------------------
; PEEK_C2_ALPHA - Checks if the 2nd character (IPH+1) is a letter (A-Z).
; Inputs:   None (uses IPH)
; Outputs:  Condition Code EQ if char 2 is A-Z.
;           Condition Code LT/GT (Not EQ) otherwise.
; -----------------------------------------------------------------------------
PEEK_C2_ALPHA:
        STRZ,R2            ; R2 = first char
PK_C2_NO_R2:
        LODI,R1 1
        LODA,R0 *IPH,R1                  ; peek 2nd char
        SUBI,R0 A'A'                     ; Shift 'A' down to 0
        COMI,R0 A'Z'-A'A'                ; Compare against 25 (length of alphabet - 1)
        RETC,GT                          ; Unsigned compare catches both < 'A' and > 'Z'  
PCA_MATCH:
        EORZ,R0             ; set EQ
PCA_RET:
        RETC,UN             ; Return to caller

; =============================================================================
;  DO_PRINT / PRTSTR -- Print statement and NUL-terminated string helper
; Syntax: PRINT [item {; item}]
;   item = "string" | CHR$(n) | TAB(n) | expr
;   CHR$(n): output low byte of n as a raw character (replaces WR statement)
;   TAB(n):  output n spaces (n=0 is a no-op; not a column-seek)
; In:  IP -> first char after PRINT keyword
; Out: text written to COUT; IP advanced past statement
; Clobbers: R0, R1, R2, EXPH, EXPL, TMPH, TMPL, NEGFLG, LNUMH, LNUML, SC0, SC1
DP_STRING:
        ZBSR *VINC_IP           ; 2b
PRTSTR:
        LODA,R0 *IPH            ; 3b
        RETC,EQ                 ; 1b: NUL before closing quote -> exit
        COMI,R0 DQ              ; 2b
        BCTR,EQ DP_SCLS         ; 2b
        ZBSR *VCOUT             ; 2b
        BCTR,UN DP_STRING       ; 2b

DP_EXPR:
        ZBSR *VPARSE_EXPR       ; 2b
        BSTA,UN PRT_FA          ; 3b
        db $EC                  ; 1b: COMA,R0 opcode - consumes next 2 bytes (ZBSR *VINC_IP)
DP_SCLS:
        ZBSR *VINC_IP           ; 2b
DP_SEP:
        ZBSR *VWSKIP       ; 2b
        COMI,R0 $3B             ; 2b: semicolon ';'
        BCFA,EQ PRT_CRLF        ; 3b: branch if NOT ';' (was 3b BCFA)
        ZBSR *VINC_IP           ; 2b
        ZBSR *VWSKIP       ; 2b
        RETC,EQ                 ; 1b: trailing ';' followed by NUL -> exit without CRLF
        BCTR,UN DP_ITEM         ; 2b

DO_PRINT:
        ZBSR *VWSKIP       ; 2b
        BCTA,EQ PRT_CRLF        ; 2b: bare PRINT -> print CRLF & return
DP_ITEM:
        COMI,R0 DQ              ; 2b
        BCTR,EQ DP_STRING       ; 2b
        
        BSTR,UN PEEK_C2_ALPHA   ; 2b
        BCFR,EQ DP_EXPR         ; 2b: 2nd char not alpha -> expr
        COMI,R2 A'C'            ; 2b
        BCTR,EQ DP_CK           ; 2b
        COMI,R2 A'T'            ; 2b
        BCFR,EQ DP_EXPR         ; 2b
DP_CK:
        LODI,R1 2               ; v3.3: CHR$ has 'R' as its 3rd letter and TAB( has 'B'; anything else (COS, TAN...) is an expression
        LODA,R0 *IPH,R1
        COMI,R0 A'R'
        BCTR,EQ DP_KW
        COMI,R0 A'B'
        BCFR,EQ DP_EXPR
DP_KW:
        ZBSR *VEATWORD          ; 2b: consume keyword
        ZBSR *VPARSE_EXPR       ; 2b: parse (n)
        BSTA,UN FIX_EXP         ; 3b: EXP = int16(n), truncating
        LODA,R0 EXPL            ; 3b: load low byte into R0 once for both CHR$ and TAB
        COMI,R2 A'C'            ; 2b
        BCTR,EQ DP_C_KW         ; 2b
        STRZ R1                 ; 1b: 2650 opcode $51 — copies R0 -> R1 & sets CC (EQ if 0)
        BCTA,EQ DP_SEP          ; 3b: TAB(0) no-op (absolute since v3.3: DP_CK moved DP_SEP out of BCTR range)
DP_TAB_LOOP:
        ZBSR *VPRT_SPACE        ; 2b
        BDRR,R1 DP_TAB_LOOP     ; 2b
        BCTA,UN DP_SEP          ; 3b
DP_C_KW:
        ZBSR *VCOUT             ; 2b: print R0 directly
        BCTA,UN DP_SEP          ; 3b

; =============================================================================
;  RND_SHUFFLE -- Advance 16-bit Galois LFSR (Little-Endian) in place
; In:  None (reads RNDSEED)
; Out: RNDSEED advanced one step
; Clobbers: R0, RNDSEED, PSL CC and Carry.  Works in register bank 1, so the
;      caller's bank-0 R1-R3 survive (GETLINE's index and DO_RND's counter
;      rely on this); R0 is not banked on the 2650, so it is NOT preserved.
;      RS/WC are clear on return, as every caller expects.
;      RND_SKIP (the store half) is also entered by MAIN to seed R0:R1.
RND_SHUFFLE:
        ; 16-bit Galois LFSR. R0=high byte, R1=low byte.
        ; CRITICAL: WC (PSL bit 3) must be SET for RRR to chain carry between
        ; registers. Without it, RRR is a per-register circular rotation (bit0
        ; wraps into bit7 of the SAME register) - the 16-bit shift breaks.
        ; Confirmed via -w watchpoint: seed cycled back to start after 8 steps.
        ; Sequence: shift R0 right (bit0 -> Carry), then R1 right (Carry ->
        ; bit7 of R1, R1's bit0 -> Carry as the feedback bit). TPSL 1 captures
        ; that feedback: CC=EQ if C=1 (apply XOR), CC=LT if C=0 (skip).
        ; Taps 0xB400 = x^16+x^14+x^13+x^11+1, standard maximal-length
        ; polynomial (period 65535).
        PPSL    PSW_RS+PSW_WC   ; bank 1 (keeps caller's R1-R3) + WC on
        LODA,R0 RNDSEED         ; Load seed high byte
        LODA,R1 RNDSEED+1       ; Load seed low byte
        CPSL    1               ; Clear Carry (C=0 shifts into bit7 of R0)
        RRR,R0                  ; Shift R0 right: bit0 of R0 -> Carry; 0 -> bit7
        RRR,R1                  ; Shift R1 right: Carry (bit0 of R0) -> bit7 of R1
                                 ;                 bit0 of R1 -> Carry (feedback)
        TPSL    1               ; Test Carry: CC=EQ if C=1, CC=LT if C=0
        BCTR,LT RND_SKIP        ; C=0 (CC=LT): feedback bit was 0, skip XOR
        EORI,R0 $B4             ; Apply taps high byte (0xB400)
RND_SKIP:
        STRA,R0 RNDSEED        ; Save seed high byte
        STRA,R1 RNDSEED+1      ; Save seed low byte
        CPSL    PSW_RS+PSW_WC   ; back to bank 0, WC off
        RETC,UN

; =============================================================================
; Character IO
; 110 Baud teletype from PIPBUG V1 as per Signetics M20 application note
; Simulator default expects CHIN $286 COUT $2B4
 ;       ORG $286

; PRE_CHIN -- stir the RND LFSR once per keystroke read, then fall into CHIN
; In:  nothing
; Out: R0 = character (from CHIN)
; Clobbers: R0, RNDSEED (R1-R3 preserved: RND_SHUFFLE and CHIN both use register
;      bank 1).  Needed because the simulator intercepts CHIN itself, so a
;      shuffle inside CHIN never ran there; and inside CHIN's wait loop it
;      stepped the LFSR once per poll, which ran in bank 0 and corrupted
;      GETLINE's R1 index on real hardware.
PRE_CHIN:
        BSTR,UN RND_SHUFFLE
CHIN:
        PPSL PSW_RS
        LODI,R0 $80
;        WRTC,R0        ; make space for shuffle
        LODI,R1 0
        LODI,R2 8
;ACHI:   
        SPSU
        BCTR,LT PRE_CHIN
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
;  CMP_TMP_PE -- Compare TMPH:TMPL against PEH:PEL (16-bit, byte-serial,
;  proper unsigned semantics via carry rather than SUBA's own CC, which is
;  only reliable over half the 0-255 byte range - see the TPSL $01 note
;  below). 
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
        RETC,GT                                   ; (GT/LT) is already the answer
        RETC,LT
        ; low byte
        LODA,R0 TMPL
        SUBA,R0 PEL
        TPSL $01                          ; carry-based unsigned compare:
        RETC,UN                           ; C=1 (no borrow) -> EQ, C=0 -> LT

; =============================================================================
;  TRY_STORE_LINE -- Store or delete a numbered line if IP starts with a digit
; In:  IPH:IPL -> input buffer
; Out: CC=GT if line stored/deleted; CC=EQ if not a numbered line
; Clobbers: R0, R1, R3, EXPH, EXPL, LNUMH, LNUML, IPH, IPL, TMPH, TMPL, PEH, PEL
;   (PE is updated as intended; R3 = record size while a line is being stored)
TRY_STORE_LINE:
        ZBSR *VDIGIT_CHECK
        BCFR,GT TSL_NUM                  ; in 0-9 -> a numbered line
TSL_NO:
        EORZ,R0                          ; CC=EQ: not a numbered line
        RETC,UN
TSL_NUM:
        BSTA,UN PF_INT                   ; [+1] IP already on the digit just tested; EXP = line number
                                          ; (>= 32768 raises ?R in FLT_TO_INT)
        LODI,R0 (IDX_EXP*16)+IDX_LNUM
        ZBSR *VREG16_TO_REG16             ; LNUMH:LNUML = parsed line number
TSL_FND:
        BSTA,UN FIND_LINE                ; [+1] TMP = insertion point (first record
                                          ; >= LNUM); CC=EQ iff that record IS LNUM
        BCFR,EQ TSL_WRITE                 ; no such line: go store the new one
        BSTR,UN DEL_REC                   ; remove the old copy, then re-find: the
        BCTR,UN TSL_FND                   ; insertion point is the same address, but
                                          ; TMP was consumed by DEL_REC
TSL_WRITE:
        ZBSR *VWSKIP
        BCTR,EQ TSL_DONE                  ; NUL: empty body = delete only (done)
        LODI,R3 $FF                       ; R3 = record size = 3 + strlen(body):
TSL_LEN:                                  ; 2 header + body + NUL. Pre-increment
        LODA,R0 *IPH,R3+                  ; index scan of the body at IP; R3 ends at
        BCFR,EQ TSL_LEN                   ; the NUL's index (= strlen)
        ADDI,R3 3
        BSTA,UN OPEN_GAP                  ; make room at TMP; PE += R3
        LODI,R1 2                         ; write the 2-byte line-number header
TSL_HDR:
        LODA,R0 LNUMH,R1-                 ; R1 2->1: LNUML; 1->0: LNUMH
        STRA,R0 *TMPH,R1                  ; -> TMP[1] then TMP[0]
        BRNR,R1 TSL_HDR
        ZBSR *VINC_TMP
        ZBSR *VINC_TMP
TSL_CPY:
        LODA,R0 *IPH
        BCTR,EQ TSL_CPYDONE                ; NUL: end of typed body
        STRA,R0 *TMPH                     ; copy one body byte
        ZBSR *VINC_TMP  
        ZBSR *VINC_IP  
        BCTR,UN TSL_CPY
TSL_CPYDONE:
        STRA,R0 *TMPH                     ; R0 is already 0 (NUL) from the LODA
                                          ; above - record terminator
TSL_DONE:                                 ; PE was already updated by OPEN_GAP
        LODI,R0 1                         ; CC=GT: line stored/deleted
        RETC,UN

; =============================================================================
;  DEL_REC -- Delete the stored record at TMP; close the hole; PE -= its length
;  Forward copy: src = TMP (start of the next record), dst = EXP (start of the
;  deleted one). When src reaches PE, dst IS the new PE - no length needed, and
;  deleting the last record simply runs the loop zero times.
; In:  TMP = start of the record to delete
; Out: PE reduced by the record's length
; Clobbers: R0, TMPH:TMPL, EXPH:EXPL
DEL_REC:
        LODI,R0 (IDX_TMP*16)+IDX_EXP
        ZBSR *VREG16_TO_REG16                ; EXP = dst = start of the doomed record
        BSTA,UN ADV_TMP_PAST_REC          ; TMP = src = start of the next record
DEL_LP:
        ZBSR *VCMP_TMP_PE
        BCFR,LT DEL_END                  ; src >= PE: everything moved
        LODA,R0 *TMPH
        STRA,R0 *EXPH
        ZBSR *VINC_TMP
        LODI,R0 EXPH-IPH
        ZBSR *VINC_ET                   ; dst++
        BCTR,UN DEL_LP
DEL_END:
        LODI,R0 (IDX_EXP*16)+IDX_PE
        ZBRR *VREG16_TO_REG16               ; PE = dst (tail call)

; =============================================================================
;  OPEN_GAP -- Open an R3-byte gap at TMP by moving [TMP,PE) up R3 bytes, and
;  advance PE by R3, refusing (ERR_OOM) if the new PE would pass PROGLIM.
;  The new PE is built first, in EXP (PE + R3 via R3 INC_ET steps, R2 counting),
;  and range-checked before anything is touched.  The backward copy then walks
;  PE itself down as the source pointer, so the existing CMP_TMP_PE is the loop
;  bound (TMP = fixed low end); dst = src + R3 comes free from indirect-indexed
;  STRA (*PEH,R3) - no second pointer to decrement.  PE = EXP at the end.
; In:  TMP = insertion point; R3 = gap size (1..255); PE = end of store
; Out: gap open at TMP (contents stale); PE += R3; TMP, R3 unchanged.
;      Store full: jumps to DO_ERROR (ERR_OOM); PE and the store are unchanged.
; Clobbers: R0, R1, R2, EXPH:EXPL
OPEN_GAP:
        LODI,R0 (IDX_PE*16)+IDX_EXP      ; EXP = PE
        ZBSR *VREG16_TO_REG16
        LODZ,R3
        STRZ,R2                          ; R2 = size, as the INC_ET step count
OG_ADD:
        LODI,R0 EXPH-IPH
        ZBSR *VINC_ET                   ; EXP++ (R2 times: EXP = PE + size)
        BDRR,R2 OG_ADD
        LODA,R0 EXPH
        COMI,R0 <PROGLIM+1
        BCFA,LT DRT_UFLOW                ; new PE is past PROGLIM: out of memory
OG_LP:
        ZBSR *VCMP_TMP_PE
        BCFR,LT OG_FIX                   ; PE has walked down to TMP: done
; =============================================================================
;  PEH:PEL -= 1, inlined (formerly a called DEC_PE; folded into this loop).
;  Borrow is read from carry (TPSL $01: EQ = C=1 = no borrow), not from the
;  result's CC - the CC after a SUB is only the sign of the result byte.
;  BUG FIX v2.10: the no-borrow fast path used to be RETC,EQ, a leftover
;  from when this was a separate BSTR-called subroutine. Inlined with no
;  call in between, that RETC returned out of OPEN_GAP itself (to TSL_WRITE)
;  after decrementing PE by 1 and copying nothing - the loop essentially
;  never ran. Now branches to OG_CPY instead, staying inside the loop.
; =============================================================================
        LODA,R0 PEL
        SUBI,R0 1
        STRA,R0 PEL
        TPSL $01
        BCTR,EQ OG_CPY                    ; no borrow: hi byte untouched
        LODA,R0 PEH
        SUBI,R0 1
        STRA,R0 PEH
        ; RETC,UN
OG_CPY:
        LODA,R0 *PEH                     ; byte at PE ...
        STRA,R0 *PEH,R3                  ; ... goes to PE+R3
        BCTR,UN OG_LP
OG_FIX:
        LODI,R0 (IDX_EXP*16)+IDX_PE
        ZBRR *VREG16_TO_REG16               ; PE = EXP (tail call)

; =============================================================================
;  FIND_LINE -- Search for line LNUMH:LNUML in program store
; Out: TMPH:TMPL = record start if found; CC=EQ found, CC=GT not found.
; Clobbers: R0, R1, TMPH, TMPL
FIND_LINE:
        BSTR,UN FIND_INS                 ; [+1]
        ; check if at end of program
        ZBSR *VCMP_TMP_PE
        BCFR,LT FL_RET_NF                 ; not LT (EQ/GT, at/past end) -> not found
FL_CHK:
        LODA,R0 *TMPH
        SUBA,R0 LNUMH
        BCTR,EQ FL_CHKLO
FL_RET_NF:
        LODI,R0 1                        ; CC=GT: not found
        RETC,UN

FL_CHKLO:
        LODI,R1 1                         ; record's lo line-number byte is
        LODA,R0 *TMPH,R1                  ; at TMP+1 - indirect-indexed peek,
        SUBA,R0 LNUML
        BCFR,EQ FL_RET_NF                 ; lo byte mismatch -> not found
FL_FOUND:
        EORZ,R0                          ; CC=EQ: found
        RETC,UN

; =============================================================================
;  ADV_TMP_PAST_REC -- Advance TMPH:TMPL past the current stored line record
; Skips the 2-byte line-number header, scans forward until NUL (end of that
; record's text), then skips the NUL too - leaves TMPH:TMPL pointing at the
; start of the NEXT record (or PE, if this was the last one).
; In:  TMPH:TMPL -> start of a stored record (its line-number hi byte)
; Out: TMPH:TMPL -> start of the next record
; Clobbers: R0
ADV_TMP_PAST_REC:
        ZBSR *VINC_TMP
APR_LP:
        ZBSR *VINC_TMP
        LODA,R0 *TMPH
        BCFR,EQ APR_LP                  ; free zero-test: NUL ends the body
        ZBRR *VINC_TMP                    ; skip the NUL itself

; =============================================================================
;  FIND_INS -- Find sorted insertion point for LNUMH:LNUML
; Returns TMPH:TMPL = address of first record with line >= LNUMH:LNUML,
; or PEH:PEL if all lines are smaller.
; In:  LNUMH:LNUML = target line number
; Out: TMPH:TMPL = insertion point
; Clobbers: R0, R1, TMPH, TMPL
FIND_INS:
        ZBSR *VSET_TMP_PROG
        db $EC                            ; COMA,R0 -- consume next 2 bytes
FI_ADV:
        BSTR,UN ADV_TMP_PAST_REC
        ZBSR *VCMP_TMP_PE
        RETC,GT
        RETC,EQ
        ;
        LODI,R1 0               ; Start index at 0 (High byte)
CHK_LP:
        LODA,R0 LNUMH,R1        ; Load target line number byte (R1=0: High, R1=1: Low)
        COMA,R0 *TMPH,R1        ; Unsigned compare with TMP line number byte
        BCTR,GT FI_ADV          ; If target > current, current line is smaller, keep searching
        RETC,LT                 ; If target < current, found the exact insertion point
        
        ; bytes are EQUAL, check if we need to test the low byte
        EORI,R1 1               ; XOR R1 with 1 (Toggles 0 -> 1 -> 0) & updates CC flags
        BCFR,EQ CHK_LP          ; If result is not 0 (we just did High byte), loop for Low byte
        RETC,UN                 ; If result is 0 (both bytes matched exactly), return

HI_LOOP:
        ZBSR *VWSKIP              ; R0 has character
        STRA,R0 SC0                      ; SC0 = char to match against operators
        LODI,R1 3                        ; last HI-row's char offset (2 rows:
                                          ; * and /, walking down to row 0)
HI_SCAN:
        LODA,R0 TOK_CHARS,R1
        SUBA,R0 SC0
        BCTR,EQ OPS_HIT_HI
        SUBI,R1 3
        BCFR,LT HI_SCAN                    ; loop while R1 still >=0
        ZBRR *VPARSER_RET                  ; no */  here - go check +-=<>

OPS_HIT_LO:
        BSTR,UN OPS_HIT_COM
        LODA,R0 SWBASE-2,R3       ; the table offset OPS_HIT_COM saved (it sits
        COMI,R0 12                ; under the RET it pushed); rows 12+ are the
        BCTR,LT OHL_HI            ; relops (= < >)
        BSTA,UN PUSH_LOLOOP       ; relop: right operand is a whole + - chain
OHL_HI:
        BSTA,UN PUSH_HILOOP       ; right operand can ITSELF start a higher-
                                  ; precedence chain ("3+4*5") - check HI
                                  ; first before this + is allowed to fire
        db $EC                           ; COMA,R0: consume next 2 bytes
OPS_HIT_HI:
        BSTR,UN OPS_HIT_COM
        ZBRR *VEXPR_ATOM         ; right operand is ONE atom only - further
                                  ; */  chaining ("2*3*4") happens via
                                  ; DO_MUL/DO_DIV looping back to HI_LOOP
                                  ; themselves, left-associatively, not here

; =============================================================================
;  OPS_HIT_COM -- push the LEFT operand (FA, 4 bytes) and the row offset, eat the operator character, push OPS_HIT_RET.
; In:  FA = left operand, R1 = TOK_CHARS row offset, R3 = SW-stack index, IP -> operator
; Out: stack grows by 5 + 2 bytes; IP past the operator (tail-jumps into PUSH_RET, which returns to the caller's caller)
; Clobbers: R0, R1, R3
OPS_HIT_COM:
        ; Push left operand (FA, 4 bytes) to the LIFO stack
        LODA,R0 FA
        STRA,R0 SWBASE,R3+
        LODA,R0 FA+1
        STRA,R0 SWBASE,R3+
        LODA,R0 FA+2
        STRA,R0 SWBASE,R3+
        LODA,R0 FA+3
        STRA,R0 SWBASE,R3+
        ; Push the R1 table offset to the stack to survive recursion
        LODZ,R1
        STRA,R0 SWBASE,R3+
        ZBSR *VINC_IP            ; consume the operator char
        LODI,R0 >OPS_HIT_RET
        LODI,R1 <OPS_HIT_RET
        ZBRR *VPUSH_RET         ; tail call

LO_LOOP:
        ZBSR *VWSKIP
        STRA,R0 SC0
        LODI,R1 18                       ; last LO-row's char offset (5 rows:
                                          ; + - = < >, walking down to row 6 -
                                          ; HI's own 2 rows sit below that)
LO_SCAN:
        LODA,R0 TOK_CHARS,R1
        SUBA,R0 SC0
        BCTA,EQ OPS_HIT_LO
        SUBI,R1 3
        COMI,R1 6                         ; stop at LO's own lower bound, not
        BCFR,LT LO_SCAN

        ; '!' relop-invert modifier: not one of the 7 known operators, but
LO_NOMATCH:
        LODA,R0 SC0
        COMI,R0 A'!'
        BCFA,EQ PARSER_RET      ; not bang so return
        LODI,R0 $FF
        STRA,R0 BANG                      ; armed - consumed once by whatever
        ZBSR *VINC_IP                     ; consume '!'
        BCTR,UN LO_LOOP                    ; loop - look for the real relop

; =============================================================================
;  DO_RND -- RND atom: step the LFSR 8 times, return the new seed as a float
; In:  IP -> "RND"
; Out: FA = float(new signed 16-bit seed); IP advanced; tail-jumps to PARSER_RET
; Clobbers: R0, R1, RNDSEED, EXPH:EXPL, FA.  R2 (target var offset) and R3 (SW
;      stack pointer) are live across EXPR and must NOT be used here.
DO_RND:
        ZBSR *VEATWORD          ; consume "RND"
        LODI,R1 8               ; shuffle a full byte's worth of taps
RNDF_MIX:
        BSTA,UN RND_SHUFFLE     ; preserves R1 (bank 1)
        BDRR,R1 RNDF_MIX
        LODI,R0 (IDX_RND*16)+IDX_EXP
        ZBSR *VREG16_TO_REG16   ; EXPH:EXPL = seed
        PPSL PSW_RS
        BSTA,UN FLT_FROM_INT    ; FA = float(seed)
        CPSL PSW_RS
        ZBRR *VPARSER_RET

; =============================================================================
;  EXPR -- Expression evaluator.
; In:  IPH:IPL -> expression string
; Out: FA = result (relops: -1.0 true / 0.0 false - see DO_EQOP)
; Clobbers: R0, R1, R3, BANG, SAVEH, SAVEL, NEGFLG, SC0, TMPH, TMPL
EXPR:
        LODI,R3 $FF                      ; SW cont-stack empty sentinel
        EORZ,R0
        STRA,R0 BANG                      ; relop-invert modifier off
        BSTA,UN PUSH_LOLOOP                 ; "once the HI-tier chain is fully
                                            ; exhausted, scan for +-=<> here"   (BSTA since v3.3: BSTR is out of range)
        BSTA,UN PUSH_HILOOP                 ; "once the first atom resolves,
                                            ; check for */  chaining first"
        db $EC                           ; COMA,R0: consume next 2 bytes
        ; drop through
; =============================================================================
;  EXPR_ATOM -- parse one atom: unary +/-, parens, RND, or a literal/variable.
; In:  IPH:IPL -> atom
; Out: EXPH:EXPL = value
; Clobbers: R0, R1 (RND path only, via DO_RND); R2 is NOT clobbered
;   here or by anything this calls - see REGISTER CONVENTIONS re: R2
EA_POS:
        ZBSR *VINC_IP  
EXPR_ATOM:
        ZBSR *VWSKIP            ; returns with R0 = char[0]
        COMI,R0 A'-'
        BCTR,EQ EA_NEG
        COMI,R0 A'+'
        BCTR,EQ EA_POS
        COMI,R0 A'('
        BCTR,EQ EA_PAREN

        ; --- FUNCTION CHECK ---
        BSTA,UN PK_C2_NO_R2
        BCFR,EQ END_FUNCS

        ; 2nd char is a letter. Check char[0] for known functions
        ZBSR *VWSKIP            ; re-peek with R0 = char[0]
        COMI,R0 A'R'
        BCTR,EQ DO_RND          ; Starts with 'R'? Try RND (absolute: DO_RND is out of BCTR range)
        COMI,R0 A'A'
        BCTR,EQ DO_ABS          ; Starts with 'A'? Try ABS
        COMI,R0 A'S'
        BCTA,EQ DO_SIN          ; v3.3: 'S'+letter = SIN
        COMI,R0 A'C'
        BCTA,EQ DO_COS          ; v3.3: 'C'+letter = COS  (CHR$ is a PRINT item and never reaches here)
        ; drop through not a known function
END_FUNCS:
        ZBSR *VWSKIP            ; re-peek with R0 = char[0]
        BSTA,UN PARSE_FACTOR    ; Parse as bare variable / factor
        ZBRR *VPARSER_RET 

EA_NEG:
        ZBSR *VINC_IP  
        LODI,R0 >NEG_RET
        LODI,R1 <NEG_RET
EX_PRA:
        ZBSR *VPUSH_RET
        ZBRR *VEXPR_ATOM                   ; parse the operand

DO_ABS:
        ZBSR *VEATWORD          ; consume "ABS"
        LODI,R0 >ABS_RET
        LODI,R1 <ABS_RET
        BCTR,UN EX_PRA          ; push RET and jump to EXPR_ATOM

; =============================================================================
;  PUSH_LOLOOP / PUSH_HILOOP -- shared "push a continuation of LO_LOOP/
;  HI_LOOP" leaf routines
;  Clobbers: R0, R1
PUSH_LOLOOP:
        LODI,R0 >LO_LOOP
        LODI,R1 <LO_LOOP
        ZBRR *VPUSH_RET                 ; tail call

PUSH_HILOOP:
        LODI,R0 >HI_LOOP
        LODI,R1 <HI_LOOP
        ZBRR *VPUSH_RET                 ; tail call

EA_PAREN:
        ZBSR *VINC_IP                       ; consume '('
        LODI,R0 >EP_RET
        LODI,R1 <EP_RET
        ZBSR *VPUSH_RET                     
        BSTR,UN PUSH_LOLOOP                
        BSTR,UN PUSH_HILOOP                 
        ZBRR *VEXPR_ATOM                   

; =============================================================================
;  DO_ADD / DO_SUB / DO_MUL / DO_DIV -- MBF4 operator handlers.  EXPR leaves the RIGHT
;  operand in FA; OPS_HIT_RET has popped the LEFT operand into FB.  Each handler runs
;  the library routine in the alternate bank, re-pushes the chain continuation
;  (LO_LOOP for + -, HI_LOOP for * /) and returns to PARSER_RET.
; In:  FA = right operand, FB = left operand
; Out: FA = result (FB destroyed); control resumes via PARSER_RET
; Errors: overflow ?O, divide by zero ?Z (library exits through VDO_ERROR)
; Clobbers: R0, R1, R3 (continuation push), FB and the library scratch
DO_SUB:                                 ; left - right = FB - FA = (-FA) + FB
        BSTA,UN FLT_NEGATE              ; R0 only: no bank bracket needed
DO_ADD:
        PPSL PSW_RS
        BSTA,UN FLT_ADD                 ; FA = FA + FB
        CPSL PSW_RS
        BSTR,UN PUSH_LOLOOP
        ZBRR *VPARSER_RET
DO_MUL:
        PPSL PSW_RS
        BSTA,UN FLT_MUL                 ; FA = FA * FB
        BCTR,UN DM_END
DO_DIV:                                 ; left / right = FB / FA: swap, then FA = FA / FB
        PPSL PSW_RS
        BSTA,UN FSWAP
        BSTA,UN FLT_DIV
DM_END:
        CPSL PSW_RS
        BSTR,UN PUSH_HILOOP             ; "2*3*4" chains left to right via HI_LOOP
        ZBRR *VPARSER_RET

; =============================================================================
;  FCMPW / DO_EQOP / DO_LTOP / DO_GTOP -- relop handlers.  FLT_CMP gives CC = FA : FB =
;  RIGHT : LEFT, so '<' (left < right) is true on GT and '>' on LT.  The result is
;  FA = -1.0 (81 80 00 00, true) or 0.0 (false); the '!' modifier (BANG) flips it.
;  All six relops are direct: <= is !>, >= is !<, <> is !=.
; In:  FA = right operand, FB = left operand
; Out: FA = -1.0 / 0.0; BANG cleared; control resumes via PARSER_RET
; Clobbers: R0, R1, FB (FLT_CMP may swap FA and FB; the result overwrites FA)
FCMPW:
        PPSL PSW_RS
        BSTA,UN FLT_CMP
        CPSL PSW_RS                     ; CPSL leaves CC alone
        RETC,UN
DO_EQOP:
        BSTR,UN FCMPW
        BCTR,EQ DOP_TRUE
        BCTR,UN DOP_FALSE
DO_LTOP:
        BSTR,UN FCMPW
        BCTR,GT DOP_TRUE
        BCTR,UN DOP_FALSE
DO_GTOP:
        BSTR,UN FCMPW
        BCTR,LT DOP_TRUE
        ; drop through: false
DOP_FALSE:
        EORZ,R0
        db $EC                            ; COMA,R0 -- consume next 2 bytes
DOP_TRUE:
        LODI,R0 $FF
        ; both paths converge here: R0 = $00 (false) or $FF (true), then the '!' modifier
        EORA,R0 BANG                      ; R0 ^= BANG ($00 no-op / $FF flips)
        STRZ,R1                           ; R1 = mask $FF (true) / $00 (false)
        ANDI,R0 $81
        STRA,R0 FA                        ; exponent: $81 (-1.0) or $00
        LODZ,R1
        ANDI,R0 $80
        STRA,R0 FA+1                      ; sign bit set for -1.0
        EORZ,R0
        STRA,R0 FA+2
        STRA,R0 FA+3
        STRA,R0 BANG                      ; BANG = 0 again
        ZBRR *VPARSER_RET

; =============================================================================
;  PARSE_FACTOR -- Parse a single value (variable or literal)
; In:  IPH:IPL -> first char of factor
; Out: EXPH:EXPL = value
; Clobbers: R0, R1
PARSE_FACTOR:
        LODA,R0 *IPH
        SUBI,R0 A'A'                      ; shift 'A' down to 0 - also
                                          ; doubles as PF_LOADVAR's index
                                          ; below, so a letter is never
                                          ; re-subtracted
        COMI,R0 A'Z'-A'A'                 ; single unsigned range test
        BCTR,GT PF_NUM                    ; not A-Z -> literal/number path
                                          ; (R0's shifted value is discarded
                                          ; there - PARSE_S16 reloads *IPH
                                          ; fresh via EORZ,R0 first thing)

        ; fall through: A-Z letter, R0 = index (0..25) already computed
; =============================================================================
;  PF_LOADVAR / PF_LV -- Load variable value from VARS into FA
; In:  R0 = index (0..25, PARSE_FACTOR's SUBI result - not re-subtracted
;      here); IP -> that letter char.  PF_LV: R1 = letter*4 (DO_NEXT enters here)
; Out: FA = variable value (4 bytes)
; Clobbers: R0, R1
PF_LOADVAR:
        ADDZ,R0
        ADDZ,R0                          ; R0 = index*4 (MBF4 stride)
        STRZ,R1                          ; R1 survives INC_IP (it works in the alternate bank)
        ZBSR *VINC_IP                     ; advance IP past the letter
PF_LV:
        LODA,R0 VARS,R1
        STRA,R0 FA
        LODA,R0 VARS+1,R1
        STRA,R0 FA+1
        LODA,R0 VARS+2,R1
        STRA,R0 FA+2
        LODA,R0 VARS+3,R1
        STRA,R0 FA+3
        RETC,UN

; =============================================================================
;  PF_INP / PF_NUM / PF_INT / FIX_EXP -- numeric literal glue for the MBF4 library
;  FLT_PARSE accepts an empty number (FA = 0, IP unmoved), so the entry points first
;  insist on a digit or '.', keeping uBASIC's syntax error for a malformed factor.
;  PF_INP: INPUT literal [+-]digits[.digits] (the digit test looks past one sign char)
;  PF_NUM: factor/literal digits[.digits] (EXPR_ATOM has already consumed unary + and -)
;  PF_INT: PF_NUM, then EXP = int16(FA)  (line numbers: TRY_STORE_LINE)
;  FIX_EXP: EXP = int16(FA), truncating toward zero; |FA| >= 32768 raises ?R
; In:  IP -> literal.  Out: FA (EXP for PF_INT/FIX_EXP), IP past the number.
; Clobbers: R0, R1; library scratch
; RAS: PF_NUM leaves by TAIL JUMP into FLT_PARSE (never BSTA): the FOR-literal path would reach depth 8.
PF_INP:
        LODA,R0 *IPH
        COMI,R0 A'-'
        BCTR,EQ PFI_SGN
        COMI,R0 A'+'
        BCFR,EQ PF_NUM
PFI_SGN:
        LODI,R1 1                         ; a sign is present: test the character after it
        db $EC                            ; COMA,R0: skip the next 2-byte LODI,R1
PF_NUM:
        LODI,R1 0
        LODA,R0 *IPH,R1
        SUBI,R0 A'0'
        COMI,R0 9
        BCFR,GT PF_GO                     ; a digit
        COMI,R0 A'.'-A'0'
        BCFA,EQ JSYNERR                   ; neither digit nor '.': syntax error
PF_GO:
        PPSL PSW_RS
        BCTA,UN FLT_PARSE                 ; tail jump; FLT_PARSE ends CPSL $10 / RETC
PF_INT:
        BSTR,UN PF_NUM
        ; drop through
FIX_EXP:
        PPSL PSW_RS
        BSTA,UN FLT_TO_INT                ; T0 (= EXP) = int16(FA)
        CPSL PSW_RS
        RETC,UN

; =============================================================================
;  PRT_INT / PRT_FA -- print through the MBF4 printer
;  PRT_INT: print the signed 16-bit integer in EXP (FREE, error line number, LIST)
;  PRT_FA : print FA (PRINT expression); FA is left as |FA|
; Clobbers: R0, R1, FA, FB and the library scratch (T0 = EXP)
PRT_INT:
        PPSL PSW_RS
        BSTA,UN FLT_FROM_INT              ; FA = float(EXP)
        BCTR,UN PRT_F2
PRT_FA:
        PPSL PSW_RS
PRT_F2:
        BSTA,UN FLT_PRINT
        CPSL PSW_RS
        RETC,UN

; =============================================================================
;  DO_LIST -- Print stored BASIC lines (v0.1b: whole program only -
; Syntax: LIST
; In:  PROG=program base, PEH:PEL=program end
; Out: whole program printed
; Clobbers: R0, IPH, IPL, TMPH, TMPL, EXPH, EXPL
DO_LIST:
        LODA,R0 RXSAVE                    ; 'L' row is shared: LIST's 3rd char is
        COMI,R0 A'T'                      ; 'S', LET's is 'T' - the word is already
        BCTA,EQ SE_NOTKW                  ; eaten, so LET = the bare assignment path
        ZBSR *VSET_TMP_PROG
        db $EC                  ; COMA,R0: consume next 2 bytes
DLS_NL:
        ZBSR *VINC_TMP                    ; skip over NUL
        BSTA,UN PRT_CRLF
DLS_LP:
        ; Check TMP against program end
        ZBSR *VCMP_TMP_PE
        RETC,GT
        RETC,EQ

        ; Read+print line number, then rest of line verbatim.  Walks TMP
        ; directly - PRINT_S16 no longer uses TMPL as scratch, so the old
        ; TMP->IP copy here and IP->TMP copy-back below are both gone.
        LODI,R1 2                         ; read the 2-byte line-number header
DLS_HDR:
        LODA,R0 *TMPH,R1-                 ; R1 2->1: TMP[1]; 1->0: TMP[0]
        STRA,R0 EXPH,R1                   ; EXPH+1 = EXPL
        BRNR,R1 DLS_HDR
        BSTR,UN PRT_INT
        ZBSR *VPRT_SPACE
;
        ZBSR *VINC_TMP           ; Skip 1st byte of line number
DLS_BLP:
        ZBSR *VINC_TMP           ; Skip 2nd byte / advance pointer
        LODA,R0 *TMPH            ; Fetch char
        BCTR,EQ DLS_NL           ; On NUL -> jump to line end routine
        ZBSR *VCOUT              ; Print character
        BCTR,UN DLS_BLP           ; Loop back to top (increment + fetch)

; =============================================================================
;  SHARED 16-BIT POINTER INCREMENT  - INC_ET family
; INC_TMP : TMPH:TMPL += 1   (offset TMPH-IPH from IPH)
; INC_IP  : IPH:IPL  += 1    (offset 0 from IPH)
; All share INC_ET body using register bank switch.
; Rule: NO BSTA inside these -- must not consume extra RAS depth.
; Offsets are assembly-time defines (e.g. EXPH-IPH=4) 
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
        CPSL PSW_RS                 ; switch back to primary bank if not already
        RETC,UN

; =============================================================================
;  REG16_TO_REG16 -- generic 16-bit copy between any two IPH-relative
;  register pairs, addressed by a packed nibble pair (v2.11 code-golf:
;  replaces the single-parameter EXP16_TO_ET/TMP_TO_ET family below plus
;  5 previously-inline manual copy loops -- see VERSION HISTORY).
; In:  R0 = (SRC_IDX<<4)|DST_IDX; each idx*2 = byte offset from IPH.
;      Use the IDX_* EQUs above. Valid idx range 0-15 (offsets 0-30).
; Out: dest 16-bit value = source 16-bit value
; Clobbers: R0. Primary R1/R2/R3 fully preserved via CPSL PSW_RS.
; =============================================================================
REG16_TO_REG16:
        PPSL PSW_RS             ; switch to alternate register bank
        STRZ,R2                 ; alt-R2 = R0 (packed byte SSSSDDDD)
        ANDI,R2 $0F             ; isolate dest nibble -> R2 = 0000DDDD
        EORZ,R2                 ; R0 ^= R2 -> SSSS0000 (cancels the low nibble)
        RRR,R0                  ; /2
        RRR,R0                  ; /2
        RRR,R0                  ; /2 -> R0 = source offset
        STRZ,R1                 ; alt-R1 = R0 (source offset)
        RRL,R2                  ; *2 -> alt-R2 = dest offset
        LODA,R0 IPH,R1
        STRA,R0 IPH,R2
        LODA,R0 IPL,R1
        STRA,R0 IPL,R2
        CPSL PSW_RS             ; restore primary bank
        RETC,UN

; =============================================================================
;  DO_FOR -- FOR var=start TO limit        (step 1; the body always runs once)
;  Assignment reuses SE_NOTKW (R2 survives the expression). The frame keeps
;  the address of the line AFTER the FOR (SWSTK, which DR_LP has already set
;  to the next line) so NEXT can jump straight back to the body.
; In:  IP -> "var=start TO limit"
; Out: var = start; frame pushed [body hi][body lo][limit hi][limit lo][var]
;      Errors: JSYNERR (no TO), ERR_OOM (frames full)
; Clobbers: R0, R1, R2, EXP, TMP
; -----------------------------------------------------------------------------
; FA_TO_BLK / BLK_TO_FB -- move a 4-byte value between FA/FB and the FOR work block
;   FA_TO_BLK: FWRK+1+R1 .. +4 = FA      BLK_TO_FB: FB = FWRK+1+R1 .. +4      (R1 = 0: limit, 4: step)
; Clobbers: R0.  Leaves primary R2/R3 alone.
FA_TO_BLK:
        LODA,R0 FA
        STRA,R0 FWRK+1,R1
        LODA,R0 FA+1
        STRA,R0 FWRK+2,R1
        LODA,R0 FA+2
        STRA,R0 FWRK+3,R1
        LODA,R0 FA+3
        STRA,R0 FWRK+4,R1
        RETC,UN
BLK_TO_FB:
        LODA,R0 FWRK+1,R1
        STRA,R0 FB
        LODA,R0 FWRK+2,R1
        STRA,R0 FB+1
        LODA,R0 FWRK+3,R1
        STRA,R0 FB+2
        LODA,R0 FWRK+4,R1
        STRA,R0 FB+3
        RETC,UN

; =============================================================================
;  DO_FOR -- FOR var=start TO limit [STEP step]   (limit and step are MBF4)
;  Assignment reuses SE_NOTKW (R2 survives the expression).  The frame keeps
;  the address of the line AFTER the FOR (SWSTK, which DR_LP has already set
;  to the next line) so NEXT can jump straight back to the body.
; In:  IP -> "var=start TO limit [STEP step]"
; Out: var = start; 11-byte frame pushed on FSTK: FWRK+8..FWRK+0 (step3..0, limit3..0, var offset), then body lo, body hi
;      Errors: JSYNERR (no TO), ERR_OOM (frames full)
; Clobbers: R0, R1, R2, FA, FB, FWRK, EXP, TMP
DO_FOR:
        LODA,R0 RXSAVE  ; check for FREE
        COMI,R0 A'R'
        BCFA,EQ DO_FREE
        ;        
        BSTA,UN SE_NOTKW                 ; var=start; R2 = var's VARS offset
        ZBSR *VWSKIP
        COMI,R0 A'T'
        BCFA,EQ JSYNERR                  ; must be TO
        ZBSR *VEATWORD
        ZBSR *VPARSE_EXPR                ; FA = limit
        LODI,R1 0
        BSTA,UN FA_TO_BLK                ; FWRK limit (safe across the STEP expression)
        ZBSR *VWSKIP
        COMI,R0 A'S'
        BCTR,EQ DF_STEP
        LODI,R0 $81                      ; no STEP: FA = 1.0 = 81 00 00 00
        STRA,R0 FA
        EORZ,R0
        STRA,R0 FA+1
        STRA,R0 FA+2
        STRA,R0 FA+3
        BCTR,UN DF_GOT
DF_STEP:
        ZBSR *VEATWORD
        ZBSR *VPARSE_EXPR                ; FA = step
DF_GOT:
        LODI,R1 4
        BSTA,UN FA_TO_BLK                ; FWRK step
        LODA,R1 FSP
        COMI,R1 FSTKLIM
        BCFA,LT DRT_UFLOW                ; all frames in use
        STRA,R2 FWRK                     ; var offset
        LODI,R2 9
DF_PUSH:
        LODA,R0 FWRK,R2-                 ; FWRK+8..FWRK+0
        STRA,R0 FSTK-1,R1+               ; pre-increment store: FSTK[FSP++]
        BRNR,R2 DF_PUSH
        LODI,R2 2
DF_BODY:
        LODA,R0 SWSTK,R2-                ; body lo, body hi
        STRA,R0 FSTK-1,R1+
        BRNR,R2 DF_BODY
        STRA,R1 FSP
        RETC,UN

; =============================================================================
;  TABLES 
BANNER:
        DB CR, LF, "miniBASIC2650 3.3", CR, LF, NUL        

; -- Combined operator + statement dispatch table
; Format: [char][hi][lo], stride 3, NUL-terminated.
; Statement scan (STMT_EXEC/MD_SCAN) safely runs the WHOLE table because
; its 2nd-char letter-gate guarantees the char being matched is A-Z,
; which none of the 7 operator chars are - no bounding needed there.
; Operator scan (HI_LOOP/LO_LOOP) is explicitly bounded to the first 7
; entries (rows 0-1 = HI tier * /, rows 2-6 = LO tier + - = < >).  Adding an
; operator row means bumping LO_LOOP's start offset and MD_SCAN's start offset.

TOK_CHARS:
        DB "*", <DO_MUL,    >DO_MUL       ; * (HI tier, offset 0)
        DB "/", <DO_DIV,    >DO_DIV       ; / (HI tier, offset 3)
        DB "+", <DO_ADD,    >DO_ADD       ; + (LO tier, offset 6)
        DB "-", <DO_SUB,    >DO_SUB       ; - (LO tier, offset 9)
        DB "=", <DO_EQOP,   >DO_EQOP      ; = (LO tier, offset 12; relop)
        DB "<", <DO_LTOP,   >DO_LTOP      ; < (LO tier, offset 15; relop)
        DB ">", <DO_GTOP,   >DO_GTOP      ; > (LO tier, offset 18; relop)
;        DB "A", <DO_ASK,    >DO_ASK       ; ASK
        DB "E", <CLR_RUNFLG,>CLR_RUNFLG   ; END
        DB "G", <DO_GO,     >DO_GO        ; GOTO / GOSUB
        DB "I", <DO_IF,     >DO_IF        ; IF / INPUT
        DB "L", <DO_LIST,   >DO_LIST      ; LIST
        DB "F", <DO_FOR,    >DO_FOR       ; FOR/FREE
        DB "N", <DO_N,      >DO_N         ; NEW / NEXT
        DB "P", <DO_PRINT,  >DO_PRINT     ; PRINT
        DB "R", <DO_RU,     >DO_RU        ; RUN / RETURN
        DB "T", <STMT_EXEC, >STMT_EXEC    ; THEN: MD_HIT's EATWORD eats the word, then
                                          ; this re-dispatches the statement after it
        DB NUL

; Helpers all called by ZBxx

; =============================================================================
;  EATWORD -- Consume [A-Z$] chars at IP
; In:  IPH:IPL -> current position
; Out: IP advanced past word
; Clobbers: R0
EATWORD:
        LODA,R0 *IPH
        SUBI,R0 A'A'                      ; shift 'A' down to 0 (also lets
        COMI,R0 A'Z'-A'A'                 ; the '$' test below reuse R0
        BCFR,GT EW_ADV                    ; without restoring it first)
        COMI,R0 A'$'-A'A'                 ; '$' compared in the same shifted
        BCFR,EQ EW_RET                    ; frame (wraps mod 256; EQ test is
EW_ADV:
        ZBSR *VINC_IP 
        BCTR,UN EATWORD

; =============================================================================
;  WSKIP -- Skip whitespace, then peek the current char at IP into R0
; Out: R0 = *IPH; CC set by that load (EQ if NUL)
; In:  IPH:IPL -> current position
; Out: IPH:IPL -> first non-space char
; Clobbers: R0
WSKIP_LOOP:
        ZBSR *VINC_IP           ; Advance IP (2 bytes)
WSKIP:
        LODA,R0 *IPH            ; Read char at IP (3 bytes)
        COMI,R0 SP              ; Is it a space? (2 bytes)
        BCTR,EQ WSKIP_LOOP      ; Yes -> loop back to increment IP (2 bytes)
        COMI,R0 $00             ; Refresh CC flags for R0 (2 bytes)
EW_RET:
        RETC,UN                 ; Return to caller (1 byte)

; =============================================================================
;  DIGIT_CHECK -- Is *IPH a decimal digit '0'-'9'?  (found via a duplicate-
;  byte-sequence scan: this 7-byte test was inlined 3x - TRY_STORE_LINE and
;  twice in PARSE_U16 - byte-for-byte identical each time, see PORT HISTORY)
; Out: R0 = char - '0'; CC=GT if not a digit (single unsigned range test)
; Clobbers: R0
DIGIT_CHECK:
        LODA,R0 *IPH
        SUBI,R0 A'0'
        COMI,R0 9
        RETC,UN

EP_RET:
        ZBSR *VWSKIP  
        ZBSR *VINC_IP                       ; consume ')'
        ZBRR *VPARSER_RET
NEG_RET:
        BSTA,UN FLT_NEGATE                  ; FA = -FA (R0 only: no bank bracket needed)
        ZBRR *VPARSER_RET
; ABS_RET -- continuation for ABS(...): the parenthesised expression is resolved; FA = |FA|
ABS_RET:
        BSTA,UN FLT_ABS
        ZBRR *VPARSER_RET

; =============================================================================
;  PUSH_RET / PARSER_RET / SWRETURN -- SW-managed call/return for expression
;  recursion (parens, operator right-operands, unary minus), replacing hw
;  RAS for this subsystem entirely. Ported from uBASIC2650's own mechanism.
;  In:  PUSH_RET: R0=lo, R1=hi of the continuation address to push
;  Out: PUSH_RET returns normally (RETC); PARSER_RET/SWRETURN never return -
;       they dispatch to whatever continuation is due (or, if the SW stack
;       is empty, do a real hw RETC to the true external caller of EXPR()).
;  Clobbers: R0 (all three); R1 (PUSH_RET only, on entry, consumed)
PUSH_RET:
        STRA,R0 SC0                          ; stash lo byte (memory, not a
                                              ; register - R2 is relied on by
                                              ; DL_STORE to survive an entire
                                              ; RHS expression evaluation;
                                              ; clobbering it there broke
                                              ; every "V=expr" assignment -
                                              ; MAIN's global COM=1 makes a
                                              ; direct COMI,R3 unsigned, and
                                              ; R3's $FF-empty sentinel reads
                                              ; as 255, past any small limit -
                                              ; normalize to a 0-based count
                                              ; first, same reason PARSER_RET
                                              ; uses EORI not a raw compare)
        LODZ,R3                              ; R0 = R3
        ADDI,R0 1                            ; R0 = count-so-far (0-based;
                                              ; $FF+1 wraps to 0, correctly)
        COMI,R0 SWCAP_LIMIT
        BCTR,LT PR_ROOM
        LODI,R0 ERR_EXPR
        ZBRR *VDO_ERROR                     ; tail call and bail 
PR_ROOM:
        LODA,R0 SC0                          ; restore lo byte
        STRA,R0 SWBASE,R3+                  ; push lo
        LODZ,R1                              ; R0 = R1 (hi byte)
        STRA,R0 SWBASE,R3+                  ; push hi
        RETC,UN

PARSER_RET:
        LODZ,R3                              ; R0 = R3
        EORI,R0 $FF                          ; R3==$FF (empty)? -> R0=0 (EQ)
        RETC,EQ                              ; SW stack empty: real hw return
        ; drop through
SWRETURN:
        LODA,R0 SWBASE,R3                   ; hi byte (topmost)
        STRA,R0 TEMPRETH
        SUBI,R3 1                            ; step to lo byte slot
        LODA,R0 SWBASE,R3                   ; lo byte
        STRA,R0 TEMPRETL
        SUBI,R3 1                            ; step past lo (now below this frame)
        BCTA,UN *TEMPRETH                    ; indirect jump to popped addr

; =============================================================================
;  SET_TMP_PROG -- Set TMPH:TMPL = PROG base address
; Clobbers: R0
SET_TMP_PROG:
        LODI,R0 <PROG
        STRA,R0 TMPH
        LODI,R0 >PROG
        STRA,R0 TMPL
        RETC,UN

; #############################################################################
; MBF4 FLOATING-POINT LIBRARY (mbf4_lib.asm v1.0 verbatim, S_GLUE..E_PRINT)
; #############################################################################
; =============================================================================
S_GLUE:
; -----------------------------------------------------------------------------
; FLT_A_TO_B -- FB = FA (the 4 packed bytes; guard bytes are not copied)
;   In     : FA
;   Out    : FB = FA
;   Clobber: R0, R1
;   RAS    : 0
FLT_A_TO_B:
        LODI,R1 4
FAB_L:
        LODA,R0 FA-1,R1
        STRA,R0 FB-1,R1
        BDRR,R1 FAB_L
        RETC,UN
; -----------------------------------------------------------------------------
; FLT_B_TO_A -- FA = FB (the 4 packed bytes; guard bytes are not copied)
;   In     : FB
;   Out    : FA = FB
;   Clobber: R0, R1
;   RAS    : 0
FLT_B_TO_A:
        LODI,R1 4
FBA_L:
        LODA,R0 FB-1,R1
        STRA,R0 FA-1,R1
        BDRR,R1 FBA_L
        RETC,UN
; -----------------------------------------------------------------------------
; SAVE_A -- park FA in PSAVE (replaces the 65C02 hardware-stack park; ONE level: a second SAVE_A overwrites)
;   In     : FA
;   Out    : PSAVE = FA
;   Clobber: R0, R1
;   RAS    : 0
SAVE_A:
        LODI,R1 4
SVA_L:
        LODA,R0 FA-1,R1
        STRA,R0 PSAVE-1,R1
        BDRR,R1 SVA_L
        RETC,UN
; -----------------------------------------------------------------------------
; REST_A -- restore FA from PSAVE
;   In     : PSAVE
;   Out    : FA = PSAVE (the guard byte FDB is not touched)
;   Clobber: R0, R1
;   RAS    : 0
REST_A:
        LODI,R1 4
RSA_L:
        LODA,R0 PSAVE-1,R1
        STRA,R0 FA-1,R1
        BDRR,R1 RSA_L
        RETC,UN
; -----------------------------------------------------------------------------
; FLT_ABS -- FA = |FA|
;   In     : FA
;   Out    : FA with the sign bit cleared
;   Clobber: R0
;   RAS    : 0
FLT_ABS:
        LODA,R0 FA+1
        ANDI,R0 $7F
        STRA,R0 FA+1
        RETC,UN
; -----------------------------------------------------------------------------
; FLT_NEGATE -- FA = -FA (zero is left as canonical zero)
;   In     : FA
;   Out    : FA with the sign bit flipped; unchanged if FA = 0
;   Clobber: R0
;   RAS    : 0
FLT_NEGATE:
        LODA,R0 FA
        RETC,EQ
        LODA,R0 FA+1
        EORI,R0 $80
        STRA,R0 FA+1
        RETC,UN
; -----------------------------------------------------------------------------
; FLT_TEN_B -- FB = 10.0 (packed $84 $20 $00 $00)
;   In     : -
;   Out    : FB = 10.0 (the guard byte FBG is not written)
;   Clobber: R0
;   RAS    : 0
FLT_TEN_B:
        LODI,R0 $84
        STRA,R0 FB
        LODI,R0 $20
        STRA,R0 FB+1
        EORZ,R0
        STRA,R0 FB+2
        STRA,R0 FB+3
        RETC,UN
E_GLUE:

; =============================================================================
; SECTION LOOPS -- 24-bit mantissa add / subtract / shift primitives
; =============================================================================
S_LOOPS:
; -----------------------------------------------------------------------------
; ADD_A_B -- FA mantissa += FB mantissa (24 bit; exponents and signs are ignored)
;   In     : FA+1..FA+3, FB+1..FB+3 (already aligned)
;   Out    : FA+1..FA+3 = sum mod 2^24;  C = carry out of bit 23.  WC=0
;   Clobber: R0, R1
;   RAS    : 0
ADD_A_B:
        CPSL    $01
        PPSL    $08
        LODI,R1 3
ADDLP:
        LODA,R0 FA,R1
        ADDA,R0 FB,R1
        STRA,R0 FA,R1
        BDRR,R1 ADDLP
        CPSL    $08
        RETC,UN
; -----------------------------------------------------------------------------
; SUB_A_B -- FA mantissa -= FB mantissa (24 bit) with the CALLER's carry-in
;   In     : FA+1..FA+3, FB+1..FB+3;  C = carry-in from the caller (C=1: no borrow pending)
;   Out    : FA+1..FA+3 = difference mod 2^24;  C=1 no borrow, C=0 borrow (FA < FB).  WC=0
;   Clobber: R0, R1
;   RAS    : 0
SUB_A_B:
        PPSL    $08
        LODI,R1 3
SUBLP:
        LODA,R0 FA,R1
        SUBA,R0 FB,R1
        STRA,R0 FA,R1
        BDRR,R1 SUBLP
        CPSL    $08
        RETC,UN
; -----------------------------------------------------------------------------
; SHR_A -- shift FA+1..FA+3 and the guard byte FDB right by one bit (32-bit shift through carry)
;   In     : C = bit shifted in at FA+1 bit 7
;   Out    : C = bit shifted out of FDB bit 0.  WC=0.  (SHR_A loads R1 = 0 and falls into SHR4.)
;   Clobber: R0, R1, R2
;   RAS    : 0
SHR_A:
        LODI,R1 0
; -----------------------------------------------------------------------------
; SHR4 -- generic SHR_A: R1 = base index (0: FA/FDB, 5: FB/FBG); the first byte shifted is base+1 (pre-increment)
;   In     : R1 = 0 or 5;  C = bit shifted in
;   Out    : FA+1..FDB (or FB+1..FBG) shifted right one bit;  C = bit shifted out of the guard byte;  R1 = base+4;  R2 = 0.  WC=0
;   Clobber: R0, R1, R2
;   RAS    : 0
SHR4:
        LODI,R2 4
        PPSL    $08
SHR4L:
        LODA,R0 FA,R1+
        RRR,R0
        STRA,R0 FA,R1
        BDRR,R2 SHR4L
        CPSL    $08
        RETC,UN
; -----------------------------------------------------------------------------
; SHL_MANTISSA -- shift FDB:FA+3:FA+2:FA+1 left by one bit, 0 shifted in at FDB bit 0
;   In     : FA+1..FA+3, FDB
;   Out    : shifted left;  C = old FA+1 bit 7.  WC=0
;   Clobber: R0, R1, R2
;   RAS    : 0
SHL_MANTISSA:
        LODI,R1 5
        LODI,R2 4
        CPSL    $01
        PPSL    $08
SHLL:
        LODA,R0 FA,R1-
        RRL,R0
        STRA,R0 FA,R1
        BDRR,R2 SHLL
        CPSL    $08
        RETC,UN
E_LOOPS:

; =============================================================================
; SECTION ADD -- FSWAP and FLT_ADD
; =============================================================================
S_ADD:
; -----------------------------------------------------------------------------
; FSWAP -- exchange the 4 packed bytes of FA and FB (the guard bytes FDB / FBG are not exchanged)
;   In     : FA, FB
;   Out    : FA <-> FB
;   Clobber: R0, R1, R2
;   RAS    : 0
FSWAP:
        LODI,R1 4
FSW_L:
        LODA,R0 FA-1,R1
        STRZ,R2
        LODA,R0 FB-1,R1
        STRA,R0 FA-1,R1
        LODZ,R2
        STRA,R0 FB-1,R1
        BDRR,R1 FSW_L
        RETC,UN
; -----------------------------------------------------------------------------
; FLT_ADD -- FA = FA + FB (mantissas aligned on the larger exponent, one guard byte, round-bit rounding)
;   In     : FA, FB (canonical; either may be zero)
;   Out    : FA = FA + FB, normalised and rounded; exact cancellation gives canonical zero.
;            Shortcuts: FA = 0 gives FA = FB;  FB = 0 leaves FA unchanged.
;   Clobber: R0-R3;  FB, FBG, FDB, FSA, FSB, FER.  FB is NOT preserved: it may be swapped with FA, is shifted for alignment
;            and has its sign bit replaced by the hidden bit.  Only the two zero shortcuts leave it intact.
;   Errors : exponent overflow -> FP_OVF ('O')
;   RAS    : 1  (FSWAP, SUB_A_B, ADD_A_B, SHR4/SHR_A; NORM_PACK is entered by jump and calls SHL_MANTISSA)
FLT_ADD:
        LODA,R0 FA
        BCFR,EQ FACKB
        BCTA,UN FLT_B_TO_A
; (FA = 0 above: result is FB.)
FACKB:
        LODA,R0 FB
        RETC,EQ
; make ea >= eb: swap the operands when FA's exponent is the smaller
        LODA,R0 FA
        COMA,R0 FB
        BCFR,LT FASG
        BSTR,UN FSWAP
; signs -> FSA / FSB, hidden bits explicit;  R3 = ea - eb;  FER = ea;  FDB = FBG = 0
FASG:
        LODA,R0 FA+1
        STRZ,R2
        ANDI,R0 $80
        STRA,R0 FSA
        LODZ,R2
        IORI,R0 $80
        STRA,R0 FA+1
        LODA,R0 FB+1
        STRZ,R2
        ANDI,R0 $80
        STRA,R0 FSB
        LODZ,R2
        IORI,R0 $80
        STRA,R0 FB+1
        LODA,R0 FA
        STRA,R0 FER
        SUBA,R0 FB
        STRZ,R3
        EORZ,R0
        STRA,R0 FDB
        STRA,R0 FBG
; ea-eb >= 25: FB is ignored (result = FA, straight to FANM2);  ea = eb: no alignment needed (FAOP)
        COMI,R3 25
        BCFA,LT FANM2
        COMI,R3 0
        BCTR,EQ FAOP
; align: shift FB (with its guard byte) right R3 places
FABT:
        CPSL    $01
        LODI,R1 5
        BSTA,UN SHR4
        BDRR,R3 FABT
; the last byte shifted out of FB (FBG) becomes the guard byte FDB of the sum (FA's own guard is 0)
        LODA,R0 FBG
        STRA,R0 FDB
; same signs: add (FASM);  different signs: subtract
FAOP:
        LODA,R0 FSA
        COMA,R0 FSB
        BCTR,EQ FASM
        LODI,R0 0                       ; subtract path: guard = 0 - FBG, and its borrow enters the 24-bit subtract
        SUBA,R0 FBG                     ; (WC=0) C=1 if FBG=0, else borrow
        STRA,R0 FDB
        BSTA,UN SUB_A_B
        TPSL    $01
        BCTR,EQ FANM2
; subtract path, C=0 (borrow): FA < FB, so negate FDB:FA+3:FA+2:FA+1 (two's complement), flip the sign FSA;
; an all-zero mantissa gives zero.  C=1: the difference is already positive (FANM2).
        PPSL    $09
        LODI,R1 4
NEGLP:
        LODI,R0 0
        SUBA,R0 FA,R1
        STRA,R0 FA,R1
        BDRR,R1 NEGLP
        CPSL    $08
        IORA,R0 FA+2
        IORA,R0 FA+3
        BCTA,EQ FLT_ZERO
        LODA,R0 FSA
        EORI,R0 $80
        STRA,R0 FSA
        BCTR,UN FANM2
; add: a carry out of bit 23 shifts the sum right once and raises the exponent (wrap to 0 -> overflow)
FASM:
        BSTA,UN ADD_A_B
        TPSL    $01
        BCTR,LT FANM2
        BSTA,UN SHR_A
        LODA,R0 FER
        ADDI,R0 1
        STRA,R0 FER
        BCTA,EQ FP_OVF
; common exit: normalise, round, pack
FANM2:
        BCTA,UN NORM_PACK
E_ADD:

; =============================================================================
; SECTION SUB -- FLT_SUB and FLT_NEGATE_B
; =============================================================================
S_SUB:
; -----------------------------------------------------------------------------
; FLT_SUB -- FA = FA - FB  (FLT_NEGATE_B, FLT_ADD, then falls into FLT_NEGATE_B)
;   In     : FA, FB (canonical; either may be zero)
;   Out    : FA = FA - FB, normalised and rounded
;   Clobber: R0-R3;  FBG, FDB, FSA, FSB, FER.  FB is restored only when FLT_ADD took a zero shortcut; otherwise it is undefined
;            (the closing FLT_NEGATE_B only undoes the first sign flip)
;   Errors : exponent overflow -> FP_OVF ('O')
;   RAS    : 2  (FLT_ADD, then FSWAP / SUB_A_B / ADD_A_B / SHR4 inside it)
FLT_SUB:
        BSTR,UN FLT_NEGATE_B
        BSTA,UN FLT_ADD
; -----------------------------------------------------------------------------
; FLT_NEGATE_B -- FB = -FB (zero is left as canonical zero)
;   In     : FB
;   Out    : FB with the sign bit flipped; unchanged if FB = 0
;   Clobber: R0
;   RAS    : 0
FLT_NEGATE_B:
        LODA,R0 FB
        RETC,EQ
        LODA,R0 FB+1
        EORI,R0 $80
        STRA,R0 FB+1
        RETC,UN
E_SUB:

; =============================================================================
; SECTION SIGN -- common set-up of MUL and DIV
; =============================================================================
S_SIGN:
; -----------------------------------------------------------------------------
; CALC_SIGN_EXP -- result exponent, result sign and explicit hidden bits for FLT_MUL / FLT_DIV
;   In     : R0 = starting exponent (FLT_MUL: FB's;  FLT_DIV: FA's)
;   Out    : FER = R0;  FSA = sign(FA) xor sign(FB) in bit 7 (other bits 0);
;            FA+1 and FB+1 bit 7 set: the hidden leading 1 is made explicit and the operands' sign bits are gone
;   Clobber: R0
;   RAS    : 0
;   Note   : SIGN_XOR was inlined here: it had no other caller and its BSTA/RETC pair cost a RAS level on the PARSE path.
CALC_SIGN_EXP:
        STRA,R0 FER
        LODA,R0 FA+1
        EORA,R0 FB+1
        ANDI,R0 $80
        STRA,R0 FSA
        LODA,R0 FA+1
        IORI,R0 $80
        STRA,R0 FA+1
        LODA,R0 FB+1
        IORI,R0 $80
        STRA,R0 FB+1
        RETC,UN
E_SIGN:

; =============================================================================
; SECTION ERR -- shared MUL/DIV exponent range check and the error exits
; =============================================================================
S_ERR:
; -----------------------------------------------------------------------------
; EXPCHK -- shared MUL/DIV exponent range check, called AFTER the normalising shift is known (E192*E192 is not an overflow)
;   In     : R0 = ea+eb (MUL) or ea-eb (DIV) as an 8-bit result;  C = carry out (MUL) / no borrow (DIV)
;   Out    : in range : R0 = R0 xor $80 = result exponent 1..255, CC = GT/LT (non-zero)
;            underflow: R0 = 0, CC = EQ (the caller flushes the result to zero)
;            overflow : does not return, jumps to FP_OVF
;   Clobber: R0, R2
;   RAS    : 0
;   Note   : C=1: R0 >= 128 overflows, otherwise e = R0+128.   C=0: R0 < 129 underflows, otherwise e = R0-128.
EXPCHK:
        STRZ,R2
        TPSL    $01
        BCFR,EQ EC_NC
        LODZ,R2
        BCTR,LT FP_OVF
        BCTR,UN EC_OK
EC_NC:
        LODZ,R2
        COMI,R0 129
        BCTR,LT EC_UF
EC_OK:
        EORI,R0 $80
        RETC,UN
EC_UF:
        EORZ,R0
        RETC,UN
; -----------------------------------------------------------------------------
; FP_OVF -- error exit 'O' (overflow).  Entered by jump from FLT_ADD, EXPCHK, NORM_PACK; never returns.
; FP_OVF and FP_DZERR share the tail through the skip-2 idiom DB $EC (ISA self-test T19): it swallows the
; following 2-byte LODI,R0, so each entry loads its own letter and runs into the common CPSL $10 / ZBRR.
;   Out    : R0 = 'O', RS=0, then ZBRR *VDO_ERROR
;   Clobber: CC
;   RAS    : 0 (no call is made; the pending return addresses are abandoned)
FP_OVF:
        LODI,R0 'O'
        DB      $EC
; -----------------------------------------------------------------------------
; FP_DZERR -- error exit 'Z' (divide by zero).  Entered by jump from FLT_DIV when FB = 0; never returns.
;   Out    : R0 = 'Z', RS=0, then ZBRR *VDO_ERROR
;   Clobber: CC
;   RAS    : 0 (as FP_OVF)
FP_DZERR:
        LODI,R0 'Z'
        DB      $EC
; -----------------------------------------------------------------------------
; FP_RANGE -- error exit 'R' (FIX out of range).  Entered by jump from FLT_TO_INT; never returns.
;   Out    : R0 = 'R', RS=0, then ZBRR *VDO_ERROR
;   Clobber: CC
;   RAS    : 0 (as FP_OVF)
FP_RANGE:
        LODI,R0 'R'
        CPSL    $10
        ZBRR    *VDO_ERROR
E_ERR:

; =============================================================================
; SECTION MUL -- MUL_BY_TEN and FLT_MUL
; =============================================================================
S_MUL:
; -----------------------------------------------------------------------------
; MUL_BY_TEN -- FA = FA * 10 (loads FB = 10.0 with FLT_TEN_B, then falls into FLT_MUL)
;   In     : FA
;   Out    : FA = FA * 10;  FB holds 10.0 afterwards (FB+1 bit 7 set when FA <> 0)
;   Clobber: as FLT_MUL
;   Errors : as FLT_MUL
;   RAS    : 1
MUL_BY_TEN:
        BSTA,UN FLT_TEN_B
; -----------------------------------------------------------------------------
; FLT_MUL -- FA = FA * FB (24-iteration shift-and-add on the 24-bit mantissas)
;   In     : FA, FB (canonical)
;   Out    : FA = product, normalised and rounded.  FA = 0 returns at once;  FB = 0 gives canonical zero (FLT_ZERO);
;            exponent underflow flushes to zero.
;   Clobber: R0-R3;  FB+1 (bit 7 forced to 1, the rest of FB is intact), FMA..FMA+2, FSA, FER, FDB
;   Errors : exponent overflow -> FP_OVF ('O')
;   RAS    : 1  (CALC_SIGN_EXP, ADD_A_B, SHR_A, SHL_MANTISSA, EXPCHK; NORM_PACK is entered by fall-through)
FLT_MUL:
        LODA,R0 FA
        RETC,EQ
        LODA,R0 FB
        BCFR,EQ FMNZ
        BCTA,UN FLT_ZERO
FMNZ:
        BSTA,UN CALC_SIGN_EXP           ; (R0 = exponent of FB: FER is overwritten below)
        LODI,R1 3
FM_CPY:
        LODA,R0 FA,R1
        STRA,R0 FMA-1,R1
        EORZ,R0
        STRA,R0 FA,R1
        BDRR,R1 FM_CPY
        STRA,R0 FDB
        LODI,R3 24
FML:
        CPSL    $01
        PPSL    $08
        LODI,R1 $FD
FMR:
        LODA,R0 FMA-$FD,R1
        RRR,R0
        STRA,R0 FMA-$FD,R1
        BIRR,R1 FMR
        CPSL    $08
        TPSL    $01
        BCTR,LT FMS
        BSTA,UN ADD_A_B
FMS:
        BSTA,UN SHR_A
        BDRR,R3 FML
        LODA,R0 FA+1
        BCTR,LT FMNS
        BSTA,UN SHL_MANTISSA
        LODA,R0 FB
        SUBI,R0 1                       ; normalising shift: exponent = ea + (eb-1) - 128
        BCTR,UN FMEX
FMNS:
        LODA,R0 FB
FMEX:
        ADDA,R0 FA
        BSTA,UN EXPCHK                  ; range check AFTER the shift (was before: false overflow at e=256)
        BCTA,EQ FLT_ZERO
        STRA,R0 FER
FMPK:
E_MUL:

; =============================================================================
; SECTION NP -- NORM_PACK: normalise, round, pack
; =============================================================================
S_NP:
; -----------------------------------------------------------------------------
; NORM_PACK -- normalise the FA mantissa (FA+1..FA+3, guard FDB), round on FDB bit 7, pack with FER / FSA
;   In     : FA+1..FA+3 = mantissa (hidden bit not necessarily in bit 7), FDB = guard byte (bit 7 = round bit),
;            FER = biased exponent, FSA = sign in bit 7
;   Out    : FA = [E][S|M1][M2][M3], normalised, rounded up when FDB bit 7 is set;  canonical zero when the mantissa is zero or the
;            exponent runs out during normalisation (FLT_ZERO is entered by jump)
;   Clobber: R0, R1, R2;  FA+0..FA+3, FDB, FER
;   Errors : rounding carries out of the top and the exponent wraps to 0 -> FP_OVF ('O')
;   RAS    : 1  (SHL_MANTISSA)
NORM_PACK:
NPL:
        LODA,R0 FA+1
        BCTR,LT NPRND
        BCFR,EQ NPBT
        LODA,R0 FER
        COMI,R0 9
        BCTR,LT NPZE
        SUBI,R0 8
        STRA,R0 FER
        LODA,R0 FA+2
        STRA,R0 FA+1
        LODA,R0 FA+3
        STRA,R0 FA+2
        LODA,R0 FDB
        STRA,R0 FA+3
        EORZ,R0
        STRA,R0 FDB
        BCTR,UN NPL
NPBT:
        BSTA,UN SHL_MANTISSA
        LODA,R0 FER
        SUBI,R0 1
        STRA,R0 FER
        BCFR,EQ NPL
NPZE:
        BCTA,UN FLT_ZERO
NPRND:
        LODA,R0 FDB
        BCFR,LT NPPK
        LODI,R1 3
NPRL:
        LODA,R0 FA,R1
        ADDI,R0 1
        STRA,R0 FA,R1
        BCFR,EQ NPPK
        BDRR,R1 NPRL
        LODI,R0 $80
        STRA,R0 FA+1
        LODA,R0 FER
        ADDI,R0 1
        STRA,R0 FER
        BCTA,EQ FP_OVF
NPPK:
        LODA,R0 FER
        STRA,R0 FA
        LODA,R0 FA+1
        ANDI,R0 $7F
        IORA,R0 FSA
        STRA,R0 FA+1
        RETC,UN
E_NP:

; =============================================================================
; SECTION DIV -- DIV_BY_TEN and FLT_DIV
; =============================================================================
S_DIV:
; -----------------------------------------------------------------------------
; DIV_BY_TEN -- FA = FA / 10 (loads FB = 10.0 with FLT_TEN_B, then falls into FLT_DIV)
;   In     : FA
;   Out    : FA = FA / 10;  FB holds 10.0 afterwards (FB+1 bit 7 set when FA <> 0)
;   Clobber: as FLT_DIV
;   Errors : as FLT_DIV
;   RAS    : 1
DIV_BY_TEN:
        BSTA,UN FLT_TEN_B
; -----------------------------------------------------------------------------
; FLT_DIV -- FA = FA / FB (32-iteration restoring division; the dividend is never pre-shifted)
;   In     : FA = dividend, FB = divisor (canonical)
;   Out    : FA = quotient, normalised and rounded.  FA = 0 returns at once;  exponent underflow flushes to zero.
;   Clobber: R0-R3;  FB+1 (bit 7 forced to 1, the rest of FB is intact), FDV..FDV+2, RD..RD+2, RT..RT+2, FSA, FER, FDB.
;            FDE is NOT touched (it is live in FLT_PARSE / FLT_PRINT across DIV_BY_TEN: FER is the scratch here).
;   Errors : FB = 0 -> FP_DZERR ('Z'), tested first even when FA = 0;  exponent overflow -> FP_OVF ('O')
;   RAS    : 1  (CALC_SIGN_EXP, EXPCHK, SHL_MANTISSA; NORM_PACK is entered by jump)
FLT_DIV:
        LODA,R0 FB
        BCFR,EQ FDBNZ
        BCTA,UN FP_DZERR
FDBNZ:
        LODA,R0 FA
        RETC,EQ
        BSTA,UN CALC_SIGN_EXP           ; (R0 = FA exponent: FER is overwritten below)
        LODI,R1 3
FD_CPY:
        LODA,R0 FB,R1
        STRA,R0 FDV-1,R1
        LODA,R0 FA,R1
        STRA,R0 RD-1,R1
        BDRR,R1 FD_CPY
        LODI,R1 $FF
        LODI,R2 3
FDC_L:
        LODA,R0 RD,R1+
        COMA,R0 FDV,R1
        BCFR,EQ FDC_D
        BDRR,R2 FDC_L
FDC_D:
        BCTR,LT FDPD            ; ma < mb: quotient starts at 0, 32 fractional bits
        PPSL    $09             ; ma >= mb: R -= D exactly (no shift: the old code halved R and lost its low bit)
        LODI,R1 3
FDS_L:
        LODA,R0 RD-1,R1
        SUBA,R0 FDV-1,R1
        STRA,R0 RD-1,R1
        BDRR,R1 FDS_L
        CPSL    $08
        LODI,R3 31              ; integer bit already in Q (=1), 31 more bits
        LODI,R0 1
        BCTR,UN FDQ
FDPD:
        LODI,R3 32
        EORZ,R0
FDQ:
        STRA,R0 FDB             ; Q = 0 or 1 in the low byte, rest cleared
        EORZ,R0
        STRA,R0 FA+1
        STRA,R0 FA+2
        STRA,R0 FA+3
        LODA,R0 FB                      ; exponent = ea - eb + 128 (+1 if quotient >= 1): eb + R3 - 32 = eb or eb-1
        ADDZ,R3
        SUBI,R0 32
        STRA,R0 FER                     ; (FDE is live in PARSE/PRINT across DIV_BY_TEN: use FER as the scratch)
        LODA,R0 FA
        SUBA,R0 FER
        BSTA,UN EXPCHK                  ; range check after the quotient size is known
        BCTA,EQ FLT_ZERO
        STRA,R0 FER
FDL:
        BSTA,UN SHL_MANTISSA
        LODI,R1 3
        CPSL    $01
        PPSL    $08
FDR_L:
        LODA,R0 RD-1,R1
        RRL,R0
        STRA,R0 RD-1,R1
        BDRR,R1 FDR_L
        CPSL    $08
        TPSL    $01
        BCTR,EQ FDFORCE
        PPSL    $09
        LODI,R1 3
FDT_L:
        LODA,R0 RD-1,R1
        SUBA,R0 FDV-1,R1
        STRA,R0 RT-1,R1
        BDRR,R1 FDT_L
        CPSL    $08
        TPSL    $01
        BCFR,EQ FDNX
        LODI,R1 3
FDK_L:
        LODA,R0 RT-1,R1
        STRA,R0 RD-1,R1
        BDRR,R1 FDK_L
        LODA,R0 FDB
        ADDI,R0 1
        STRA,R0 FDB
        BCTR,UN FDNX
FDFORCE:
        PPSL    $09
        LODI,R1 3
FDF_L:
        LODA,R0 RD-1,R1
        SUBA,R0 FDV-1,R1
        STRA,R0 RD-1,R1
        BDRR,R1 FDF_L
        CPSL    $08
        LODA,R0 FDB
        ADDI,R0 1
        STRA,R0 FDB
FDNX:
        BDRA R3,FDL
        BCTA,UN NORM_PACK
E_DIV:

; =============================================================================
; SECTION CMP -- FLT_CMP
; =============================================================================
S_CMP:
; -----------------------------------------------------------------------------
; FLT_CMP -- compare FA with FB (both canonical: zero is 00 00 00 00 and never negative)
;   In     : FA, FB
;   Out    : CC = LT / EQ / GT for FA < / = / > FB.  Operands of equal sign are compared as 32-bit unsigned big-endian numbers.
;   Clobber: R0, R1, R2;  when BOTH operands are negative FA and FB are swapped (FSWAP) and stay swapped
;   RAS    : 1  (FSWAP)
FLT_CMP:
        LODA,R0 FA+1
        EORA,R0 FB+1
        BCFR,LT FC_SAME
        LODA,R0 FA+1
        BCTR,LT FC_X
        LODI,R0 1
        COMI,R0 0
FC_X:
        RETC,UN
FC_SAME:
        LODA,R0 FA+1
        BCFR,LT FC_POS
        BSTA,UN FSWAP
FC_POS:
        LODI,R1 $FF
        LODI,R2 4
FC_L:
        LODA,R0 FA,R1+
        COMA,R0 FB,R1
        RETC,GT
        RETC,LT
        BDRR,R2 FC_L
        RETC,UN
E_CMP:

; =============================================================================
; SECTION CONV -- zero, integer <-> float
; =============================================================================
S_CONV:
; -----------------------------------------------------------------------------
; FLT_ZERO -- FA = canonical zero 00 00 00 00 (the guard byte FDB is not written)
;   In     : -
;   Out    : FA = 0
;   Clobber: R0, R1, R2
;   RAS    : 0
;   Note   : FLT_ZERO loads R1 = 0, DB $EC skips the next LODI,R1 (skip-2 idiom), and the code runs on into F_ZERO.
FLT_ZERO:
        LODI,R1 0
        DB      $EC
; -----------------------------------------------------------------------------
; FLT_ZERO_B -- FB = canonical zero 00 00 00 00 (the guard byte FBG is not written)
;   In     : -
;   Out    : FB = 0
;   Clobber: R0, R1, R2
;   RAS    : 0
;   Note   : F_ZERO below is the common tail: R1 = 0 zeroes FA, R1 = 5 zeroes FB (also entered by FLT_SHARED for a zero integer).
FLT_ZERO_B:
        LODI,R1 5
F_ZERO:
        LODI,R2 4
        EORZ,R0
FZL:
        STRA,R0 FA,R1
        ADDI,R1 1
        BDRR,R2 FZL
        RETC,UN
; -----------------------------------------------------------------------------
; FLT_FROM_INT -- FA = (float) the signed 16-bit integer in T0 (T0 = high byte, T0+1 = low byte)
;   In     : T0:T0+1 = -32768..32767
;   Out    : FA = value (exact: 16 bits fit the 24-bit mantissa); 0 gives canonical zero;  the guard byte FDB is not written
;   Clobber: R0, R1, T0:T0+1 (negated / normalised), FER, FSA;  R2 when T0 = 0
;   RAS    : 1  (NEG16)
;   Note   : FLT_FROM_INT loads R1 = 0 (FA), DB $EC skips the next LODI,R1 (skip-2 idiom), FLT_SHARED does the work.
FLT_FROM_INT:
        LODI,R1 0
        DB      $EC
; -----------------------------------------------------------------------------
; FLT_FROM_INT_B -- FB = (float) the signed 16-bit integer in T0 (as FLT_FROM_INT, but loads R1 = 5)
;   In     : T0:T0+1 = -32768..32767
;   Out    : FB = value;  0 gives canonical zero;  the guard byte FBG is not written
;   Clobber: R0, R1, T0:T0+1, FER, FSA;  R2 when T0 = 0
;   RAS    : 1  (NEG16)
FLT_FROM_INT_B:
        LODI,R1 5
FLT_SHARED:
        LODA,R0 T0
        IORA,R0 T0+1
        BCTR,EQ F_ZERO
        LODA,R0 T0
        ANDI,R0 $80
        STRA,R0 FSA
        BCTR,EQ F_POS
        BSTA,UN NEG16
F_POS:
        LODI,R0 $90
        STRA,R0 FER
F_NORM:
        LODA,R0 T0
        BCTR,LT F_PACK
        LODA,R0 T0+1
        ADDZ,R0
        STRA,R0 T0+1
        LODA,R0 T0
        PPSL    $08
        ADDZ,R0
        CPSL    $08
        STRA,R0 T0
        LODA,R0 FER
        SUBI,R0 1
        STRA,R0 FER
        BCFR,EQ F_NORM
F_PACK:
        LODA,R0 FER
        STRA,R0 FA,R1
        LODA,R0 T0
        ANDI,R0 $7F
        IORA,R0 FSA
        STRA,R0 FA+1,R1
        LODA,R0 T0+1
        STRA,R0 FA+2,R1
        EORZ,R0
        STRA,R0 FA+3,R1
        RETC,UN
; -----------------------------------------------------------------------------
; NEG16 -- T0:T0+1 = -(T0:T0+1) (16-bit two's complement; -32768 stays -32768)
;   In     : T0:T0+1
;   Out    : T0:T0+1 negated
;   Clobber: R0
;   RAS    : 0
NEG16:
        LODI,R0 0
        SUBA,R0 T0+1
        STRA,R0 T0+1
        LODI,R0 0
        PPSL    $08
        SUBA,R0 T0
        CPSL    $08
        STRA,R0 T0
        RETC,UN
; -----------------------------------------------------------------------------
; FLT_TO_INT -- T0:T0+1 = FA as a signed 16-bit integer, TRUNCATED toward zero (|FA| < 1 gives 0).
; FLT_FLOOR is an alias for the same entry (it does not floor negative fractions).
;   In     : FA
;   Out    : T0 = high byte, T0+1 = low byte, -32768..32767
;   Clobber: R0, R1, T0:T0+1, FDE (scratch: the integer-bit count)
;   Errors : |FA| beyond +32767 / -32768 -> FP_RANGE ('R'); the result never wraps and never saturates (v0.11)
;   RAS    : 1  (NEG16)
FLT_FLOOR:
FLT_TO_INT:
        EORZ,R0
        STRA,R0 T0
        STRA,R0 T0+1
        LODA,R0 FA
        COMI,R0 $81
        RETC,LT
        SUBI,R0 $80
        COMI,R0 17
        BCFA,LT FTIS
        STRA,R0 FDE
        LODA,R0 FA+1
        IORI,R0 $80
        STRA,R0 T0
        LODA,R0 FA+2
        STRA,R0 T0+1
        LODI,R0 16
        SUBA,R0 FDE
        BCTR,EQ FTIG
        STRZ,R1
FTIS2:
        CPSL    $01
        PPSL    $08
        LODA,R0 T0
        RRR,R0
        STRA,R0 T0
        LODA,R0 T0+1
        RRR,R0
        STRA,R0 T0+1
        CPSL    $08
        BDRR,R1 FTIS2
FTIG:
        LODA,R0 FA+1
        BCTR,LT FTIN
        LODA,R0 T0
        BCTA,LT FP_RANGE                ; positive magnitude >= 32768
        RETC,UN
FTIN:
        BSTA,UN NEG16
        LODA,R0 T0
        BCFA,LT FP_RANGE                ; negative result must have bit 15 set (-32768 is legal)
        RETC,UN
FTIS:
        BCTA,UN FP_RANGE
E_CONV:

; =============================================================================
; SECTION PARSE -- PINC and FLT_PARSE
; =============================================================================
S_PARSE:
; -----------------------------------------------------------------------------
; PINC -- IP = IP + 1 for alternate-bank code (INC_IP/INC_ET switch banks and return with RS=0, so RS=1 is re-entered here)
;   In     : RS=1
;   Out    : IP advanced by 1;  RS=1
;   Clobber: R0, alt R1
;   RAS    : 1  (ZBSR *VINC_IP; INC_IP counted as a leaf)
PINC:
        ZBSR    *VINC_IP
        PPSL    $10
        RETC,UN
; -----------------------------------------------------------------------------
; FLT_PARSE -- parse [+-]digits[.digits] at *IP into FA and advance IP past it (no E notation).
; Integer digits accumulate as FA = FA*10 + digit.  The 65C02's recursive PARSE_FRAC is a loop here: it
; reads the fraction digits backwards through *IP+R1 and adds them as (digit + fraction)/10 steps.
;   In     : RS=1 (caller did PPSL $10), IP -> first character (via IPH:IPL, high byte first)
;   Out    : FA = value (no digits gives 0);  IP -> first character after the number;  RS=0 (CPSL $10 inside, then RETC,UN)
;   Entry  : by tail jump from PARSE_FACTOR (PPSL $10 / BCTA,UN FLT_PARSE): no RAS level is used at entry
;   Clobber: R0-R3;  FA, FB, FBG, FDB, FSA, FSB, FER, FMA, FDV, RD, RT, T0, FDE (sign flag), FPN, PSAVE
;   Errors : value too large -> FP_OVF ('O');  too small flushes to 0
;   RAS    : 2  (PINC -> INC_IP;  MUL_BY_TEN;  FLT_ADD -> FSWAP;  INC_IP and DIGIT_CHECK counted as leaves)
FLT_PARSE:
        BSTA,UN FLT_ZERO
        EORZ,R0
        STRA,R0 FDE
        LODA,R0 *IPH
        COMI,R0 '-'
        BCFR,EQ FPNN
        LODI,R0 $80
        STRA,R0 FDE
        BSTR,UN PINC
        BCTR,UN FPAI
FPNN:
        COMI,R0 '+'
        BCFR,EQ FPAI
        BSTR,UN PINC
FPAI:
        ZBSR    *VDIGIT_CHECK
        BCTR,GT FPDT
        STRA,R0 T0+1
        EORZ,R0
        STRA,R0 T0
        BSTR,UN PINC
        BSTA,UN MUL_BY_TEN
        BSTA,UN FLT_FROM_INT_B
        BSTA,UN FLT_ADD
        BCTR,UN FPAI
FPDT:
        LODA,R0 *IPH
        COMI,R0 '.'
        BCFA,EQ FPSG
        BSTA,UN PINC
        BSTA,UN SAVE_A
        LODI,R1 0
FPCNT:
        LODA,R0 *IPH,R1
        SUBI,R0 '0'
        COMI,R0 10
        BCFR,LT FPCE
        ADDI,R1 1
        BCTR,UN FPCNT
FPCE:
        STRA,R1 FPN
        BSTA,UN FLT_ZERO
FPFL:
        LODA,R1 FPN
        BCTR,EQ FPFD
        SUBI,R1 1
        STRA,R1 FPN
        LODA,R0 *IPH,R1
        SUBI,R0 '0'
        STRA,R0 T0+1
        EORZ,R0
        STRA,R0 T0
        BSTA,UN FLT_FROM_INT_B
        BSTA,UN FLT_ADD
        BSTA,UN DIV_BY_TEN
        BCTR,UN FPFL
FPFD:
        LODI,R1 0
FPSK:
        LODA,R0 *IPH,R1
        SUBI,R0 '0'
        COMI,R0 10
        BCFR,LT FPSKD
        BSTA,UN PINC
        BCTR,UN FPSK
FPSKD:
        BSTA,UN FLT_A_TO_B
        BSTA,UN REST_A
        BSTA,UN FLT_ADD
FPSG:
        LODA,R0 FDE
        BCTR,EQ FPSX
        BSTA,UN FLT_NEGATE
FPSX:
        CPSL    $10                     ; FLT_PARSE owns the bank: entered by 'PPSL $10 / BCTA FLT_PARSE' from PARSE_FACTOR
        RETC,UN
E_PARSE:

; =============================================================================
; SECTION PRINT -- PUTC and FLT_PRINT
; =============================================================================
S_PRINT:
; -----------------------------------------------------------------------------
; PUTC -- send R0 to COUT, then re-enter the alternate bank (COUT returns with RS=0 and clobbers alt R1 and R2)
;   In     : R0 = character;  RS=1
;   Out    : character sent;  RS=1
;   Clobber: R0, alt R1, alt R2
;   RAS    : 2  (ZBSR *VCOUT; fpdepth.py allows COUT one more level of its own)
PUTC:
        ZBSR    *VCOUT
        PPSL    $10
        RETC,UN
; -----------------------------------------------------------------------------
; FLT_PRINT -- print FA in plain notation (no exponent), 6 significant digits, trailing fractional zeros trimmed.
; 7 digits are extracted and rounded half-up on the 7th.  Counters live in RAM / alt R3 because PUTC and the FP
; routines clobber alt R1/R2.
;   In     : FA (canonical);  RS=1
;   Out    : characters through PUTC / COUT:  "0" for zero;  "-" first for a negative value;  |x| < 1 prints as "0." + leading zeros + digits.
;            FA = |FA| on return (zero and positive: unchanged; the sign of a negative FA is NOT restored).
;   Clobber: R0-R3;  FA (see Out), FB, FBG, FDB, FSA, FSB, FER, FMA, FDV, RD, RT, PSAVE, T0, T2, FDE, FPLIM, FPY, DIGI, DIGV, DIG..DIG+6
;   Errors : none expected for a canonical FA: it is scaled into [1,10) before the MUL / DIV / FIX steps, so they cannot overflow or leave range
;   RAS    : 3  (FLT_SUB -> FLT_ADD -> FSWAP, or PUTC -> COUT -> its inner level)
FLT_PRINT:
        LODA,R0 FA
        BCFR,EQ FPNZ
        LODI,R0 '0'
        BCTR,UN PUTC
FPNZ:
        LODA,R0 FA+1
        BCFR,LT FPPS
        LODI,R0 '-'
        BSTR,UN PUTC
        BSTA,UN FLT_ABS
FPPS:
        BSTA,UN SAVE_A
        EORZ,R0
        STRA,R0 FDE
FPDN:
        BSTA,UN FLT_TEN_B
        BSTA,UN FLT_CMP
        BCTR,LT FPUP
        BSTA,UN DIV_BY_TEN
        LODA,R0 FDE
        ADDI,R0 1
        STRA,R0 FDE
        BCTR,UN FPDN
FPUP:
        LODA,R0 FA
        COMI,R0 $81
        BCFR,LT FPSC
        BSTA,UN MUL_BY_TEN
        LODA,R0 FDE
        SUBI,R0 1
        STRA,R0 FDE
        BCTR,UN FPUP
FPSC:
        LODA,R0 FDE
        STRA,R0 T2
        EORZ,R0
        STRA,R0 DIGI
FPDIG:
        BSTA,UN FLT_TO_INT
        LODA,R0 T0+1
        STRA,R0 DIGV
        EORZ,R0
        STRA,R0 T0
        BSTA,UN FLT_FROM_INT_B
        BSTA,UN FLT_SUB
        LODA,R0 FA+1
        BCFR,LT FPCL
        BSTA,UN FLT_ZERO
FPCL:
        BSTA,UN MUL_BY_TEN
        LODA,R0 DIGV
        IORI,R0 '0'
        LODA,R1 DIGI
        STRA,R0 DIG,R1
        ADDI,R1 1
        STRA,R1 DIGI
        COMI,R1 7
        BCFR,EQ FPDIG
FPRD:
        LODA,R0 T2
        STRA,R0 FDE
        LODA,R0 DIG+6
        COMI,R0 '5'
        BCTR,LT FPNRD
        LODI,R1 6
FPRU:
        LODA,R0 DIG-1,R1
        ADDI,R0 1
        STRA,R0 DIG-1,R1
        COMI,R0 ':'
        BCTR,LT FPNRD
        LODI,R0 '0'
        STRA,R0 DIG-1,R1
        BDRR,R1 FPRU
        LODI,R0 '1'
        STRA,R0 DIG
        LODA,R0 FDE
        ADDI,R0 1
        STRA,R0 FDE
FPNRD:
        LODI,R1 6
FPST:
        LODA,R0 DIG-1,R1
        COMI,R0 '0'
        BCFR,EQ FPSTD
        BDRR,R1 FPST
FPSTD:
        STRA,R1 FPLIM
        LODA,R0 FDE
        BCTA,LT FPLT1
        ADDI,R0 1
        STRZ,R3
        EORZ,R0
        STRA,R0 FPY
FPIT:
        LODI,R0 '0'
        LODA,R1 FPY
        COMI,R1 6
        BCFR,LT FPIT2
        LODA,R0 DIG,R1
        ADDI,R1 1
        STRA,R1 FPY
FPIT2:
        BSTA,UN PUTC
        BDRR,R3 FPIT
FPFR:
        LODA,R1 FPY
        COMI,R1 6
        BCFR,LT FPEND
        COMA,R1 FPLIM
        BCFR,LT FPEND
        LODI,R0 '.'
        BSTA,UN PUTC
FPFRL:
        LODA,R1 FPY
        LODA,R0 DIG,R1
        BSTA,UN PUTC
        LODA,R1 FPY
        ADDI,R1 1
        STRA,R1 FPY
        COMA,R1 FPLIM
        BCFR,LT FPEND
        COMI,R1 6
        BCTR,LT FPFRL
FPEND:
        BCTA,UN REST_A
FPLT1:
        LODI,R0 '0'
        BSTA,UN PUTC
        LODI,R0 '.'
        BSTA,UN PUTC
        LODA,R0 FDE
        EORI,R0 $FF
        BCTR,EQ FPLZD
        STRZ,R3
FPLZ:
        LODI,R0 '0'
        BSTA,UN PUTC
        BDRR,R3 FPLZ
FPLZD:
        EORZ,R0
        STRA,R0 FPY
        BCTR,UN FPFRL
E_PRINT:
; =============================================================================
; SECTION TRIG -- SIN and COS (radians), Stage 5 tier 1.  Outside the Stage 3 library (S_GLUE..E_PRINT), which is unchanged.
;
; Method: a = |x|;  y = a * 2/PI;  q = TRUNC(y) (16 bit, so |x| < 51471 or ?R);  f = y - q in [0,1).
;   n = (q + offset) AND 3 where offset = 0 (SIN, x >= 0), 2 (SIN, x < 0), 1 (COS).
;   n odd: t = 1 - f, else t = f;  r = sin(PI*t/2) = t * P(t*t), P a 5-coefficient minimax polynomial in HORNER_ODD;  n AND 2: r = -r.
;   (SIN: -x adds 2 to n.  COS(x) = SIN(x + PI/2): adds 1.)  Measured max error vs double precision: see tools/gen_coeffs.py and STAGE5_REPORT.
; Both functions take their operand exactly like ABS: EXPR_ATOM -> DO_SIN / DO_COS -> EX_PRA pushes SIN_RET / COS_RET and parses the
; operand; the continuation runs with FA = operand and ends in PARSER_RET.
; One table, SCT, feeds both the constants and the coefficients through LD_B_PTR (HPTR walks it in 4-byte steps):
;   2/PI, 1.0, c4, c3, c2, c1, c0   (c4 first: Horner order).  The table must not cross a 256-byte page (LD_B_PTR adds to the low byte only);
;   tools/check_tables.py verifies that on the .LST.
; =============================================================================
S_TRIG:
; -----------------------------------------------------------------------------
; DO_SIN / DO_COS -- SIN( / COS( atoms (entered by BCTA from EXPR_ATOM)
;   In     : IP -> the keyword (any letters are eaten, as for ABS)
;   Out    : IP past the keyword; SIN_RET / COS_RET pushed on the SW stack; the operand is parsed by EXPR_ATOM
;   Clobber: R0, R1;  R2 and R3 are live across EXPR and untouched
;   RAS    : 1  (EATWORD)
DO_SIN:
        ZBSR *VEATWORD
        LODI,R0 >SIN_RET
        LODI,R1 <SIN_RET
        BCTA,UN EX_PRA
DO_COS:
        ZBSR *VEATWORD
        LODI,R0 >COS_RET
        LODI,R1 <COS_RET
        BCTA,UN EX_PRA
; -----------------------------------------------------------------------------
; SIN_RET / COS_RET -- continuations: FA = operand.  Set the quadrant offset, point HPTR at SCT, run SC_CORE in the alternate bank.
;   In     : FA = x
;   Out    : FA = SIN(x) or COS(x); RS = 0; ends in PARSER_RET
;   Clobber: R0-R3 (both banks), FA, FB, FBG, FDB, PSAVE, T0, ZV, QF, HPTR, HCNT and the library scratch;  R2/R3 of bank 0 are untouched
;   Errors : |x| >= 51471 -> ?R (FLT_TO_INT);  overflow cannot occur
;   RAS    : 4 below the caller of PARSER_RET (SC_CORE, HORNER_ODD, FLT_MUL / FLT_ADD, their inner level)
SIN_RET:
        LODA,R0 FA+1
        BCTR,LT SC_NEG          ; sign bit set: x < 0
        EORZ,R0                 ; offset 0
        BCTR,UN SC_GO
SC_NEG:
        LODI,R0 2               ; offset 2: SIN(-x) = -SIN(x) is n + 2
        BCTR,UN SC_GO
COS_RET:
        LODI,R0 1               ; offset 1: COS(x) = SIN(x + PI/2); the sign of x does not matter
SC_GO:
        STRA,R0 QF
        LODI,R0 <(SCT-1)
        STRA,R0 HPTR
        LODI,R0 >(SCT-1)
        STRA,R0 HPTR+1
        LODI,R0 5               ; number of coefficients in the table
        STRA,R0 HCNT
        PPSL PSW_RS
        BSTR,UN SC_CORE
        CPSL PSW_RS
        ZBRR *VPARSER_RET
; -----------------------------------------------------------------------------
; SC_CORE -- the SIN / COS body (runs in the alternate bank)
;   In     : FA = x, QF = offset, HPTR -> SCT-1, HCNT = 5, RS = 1
;   Out    : FA = result
;   Clobber: R0-R3, FA, FB, PSAVE, T0, ZV, QF, HPTR, HCNT, and what FLT_MUL / FLT_ADD / FLT_SUB / FLT_TO_INT / FLT_FROM_INT clobber
;   Errors : ?R from FLT_TO_INT when |x|*2/PI >= 32768
;   RAS    : 3  (HORNER_ODD, then FLT_MUL or FLT_ADD and its inner level counted in the caller's 4)
SC_CORE:
        BSTA,UN FLT_ABS         ; FA = |x|
        BSTA,UN LD_B_PTR        ; FB = 2/PI
        BSTA,UN FLT_MUL         ; FA = y = |x| * 2/PI
        BSTA,UN SAVE_A          ; PSAVE = y
        BSTA,UN FLT_TO_INT      ; T0:T0+1 = q = TRUNC(y)
        LODA,R0 QF
        ADDA,R0 T0+1            ; offset + low byte of q
        ANDI,R0 3
        STRA,R0 QF              ; QF = n
        BSTA,UN FLT_FROM_INT    ; FA = q
        BSTA,UN FLT_A_TO_B      ; FB = q
        BSTA,UN REST_A          ; FA = y
        BSTA,UN FLT_SUB         ; FA = f = y - q, 0 <= f < 1
        BSTA,UN LD_B_PTR        ; FB = 1.0 (always loaded: it keeps HPTR in step)
        LODA,R0 QF
        ANDI,R0 1
        BCTR,EQ SC_P            ; n even: t = f
        BSTA,UN FLT_NEGATE
        BSTA,UN FLT_ADD         ; n odd: t = 1 - f  (FA = -f + FB)
SC_P:
        BSTR,UN HORNER_ODD      ; FA = sin(PI*t/2)
        LODA,R0 QF
        ANDI,R0 2
        RETC,EQ
        BCTA,UN FLT_NEGATE      ; n AND 2: FA = -FA (tail call)
; -----------------------------------------------------------------------------
; HORNER_ODD -- FA = t * P(t*t) with P from the coefficient stream (highest power first)
;   In     : FA = t, HPTR -> stream-1, HCNT = number of coefficients (>= 1), RS = 1
;   Out    : FA = t * P(t*t); HPTR advanced past the coefficients;  HCNT = 0
;   Clobber: R0-R3, FA, FB, PSAVE (= t), ZV (= t*t), HPTR, HCNT, and the FLT_MUL / FLT_ADD scratch
;   Errors : as FLT_MUL / FLT_ADD (cannot occur for the SIN / COS tables)
;   RAS    : 1 here + 1 for FLT_MUL / FLT_ADD (+ their inner level)
HORNER_ODD:
        BSTA,UN SAVE_A          ; PSAVE = t
        BSTA,UN FLT_A_TO_B      ; FB = t
        BSTA,UN FLT_MUL         ; FA = z = t*t
        BSTA,UN SAVE_Z          ; ZV = z
        BSTR,UN LD_B_PTR        ; FB = first (highest) coefficient
        BSTA,UN FLT_B_TO_A      ; FA = S
HO_L:
        LODA,R0 HCNT
        SUBI,R0 1
        STRA,R0 HCNT
        BCTR,EQ HO_D
        BSTR,UN LD_B_Z          ; FB = z
        BSTA,UN FLT_MUL         ; FA = S * z
        BSTR,UN LD_B_PTR        ; FB = next coefficient
        BSTA,UN FLT_ADD         ; FA = S * z + c
        BCTR,UN HO_L
HO_D:
        BSTA,UN FLT_A_TO_B      ; FB = P(z)
        BSTA,UN REST_A          ; FA = t
        BCTA,UN FLT_MUL         ; FA = t * P(z) (tail call)
; -----------------------------------------------------------------------------
; LD_B_PTR -- FB = the 4-byte entry after HPTR (first byte at HPTR+1), then HPTR += 4 (low byte only: no page crossing, see the header)
;   In     : HPTR
;   Out    : FB = entry (the guard byte FBG is not written);  HPTR advanced by 4
;   Clobber: R0, R1
;   RAS    : 0
LD_B_PTR:
        LODI,R1 4
LBP_L:
        LODA,R0 *HPTR,R1        ; R1 = 4..1: entry byte 3..0
        STRA,R0 FB-1,R1
        BDRR,R1 LBP_L
        LODA,R0 HPTR+1
        ADDI,R0 4
        STRA,R0 HPTR+1
        RETC,UN
; -----------------------------------------------------------------------------
; LD_B_Z -- FB = ZV   (4 packed bytes; FBG is not written)
;   In     : ZV
;   Out    : FB = ZV
;   Clobber: R0, R1
;   RAS    : 0
LD_B_Z:
        LODI,R1 4
LBZ_L:
        LODA,R0 ZV-1,R1
        STRA,R0 FB-1,R1
        BDRR,R1 LBZ_L
        RETC,UN
; -----------------------------------------------------------------------------
; SAVE_Z -- ZV = FA   (4 packed bytes; the guard byte FDB is not copied)
;   In     : FA
;   Out    : ZV = FA
;   Clobber: R0, R1
;   RAS    : 0
SAVE_Z:
        LODI,R1 4
SVZ_L:
        LODA,R0 FA-1,R1
        STRA,R0 ZV-1,R1
        BDRR,R1 SVZ_L
        RETC,UN
; -----------------------------------------------------------------------------
; SCT -- constant / coefficient stream for SIN and COS, 4 bytes per entry [E][S|M1][M2][M3] (MBF4):  2/PI, 1.0, then c4..c0 of
;   sin(PI*t/2) = t * (c0 + c1 t^2 + c2 t^4 + c3 t^6 + c4 t^8), minimax on t in [0,1] (tools/gen_coeffs.py --asm 5)
SCT:
        DB $80,$22,$F9,$83          ; 2/PI = 0.636619772
        DB $81,$00,$00,$00          ; 1.0
        DB $74,$1F,$CE,$19          ; c4 = 0.00015240199
        DB $79,$99,$37,$F9          ; c3 = -0.00467586191
        DB $7D,$23,$35,$25          ; c2 = 0.0796912089
        DB $80,$A5,$5D,$E7          ; c1 = -0.645964086
        DB $81,$49,$0F,$DB          ; c0 = 1.57079637
E_TRIG:
ROMEND: 

;  RAM variables -- sequential RES block 
 
        ORG     4096    ; half a 2650 8kbyte page

; --- Ordered group: offsets from IPH used by INC_ET/DEC_ET/NEG_SHARED and
; by REG16_TO_REG16's packed-nibble index scheme (see IDX_* EQUs, v2.11).
; IBUF's hi==lo GETLINE trick was dropped when this group grew past 16
; bytes to bring PE/SC0/RNDSEED into nibble range (+2B in GETLINE, noted
; in VERSION HISTORY).
IPH     RES 1       ; interpreter pointer hi       (INC_ET offset 0; IDX_IP=0)
IPL     RES 1       ; interpreter pointer lo
TMPH    RES 1       ; temp 16-bit hi               (INC_ET offset 2 = TMPH-IPH; IDX_TMP=1)
TMPL    RES 1       ; temp 16-bit lo
GOTOH   RES 1       ; pending target hi            (DEC_ET offset 8 = GOTOH-IPH; IDX_GOTO=2)
GOTOL   RES 1       ; pending target lo
CURH    RES 1       ; current line hi  (error reporting; IDX_CUR=3)
CURL    RES 1       ; current line lo

; v3.0: the FOR frame is built from FWRK (see DO_FOR), so SWSTK/EXP/LNUM/FVAR no longer have to be contiguous.  Historical note (v2.11): DO_FOR pushed
; the seven bytes with one loop and DO_NEXT pops them back the same way.
; During a FOR: EXP = step, LNUM = limit (LNUM is untouched by expression
; evaluation, so it can hold the limit while the STEP expression is parsed).
SWSTK   RES 2       ; next-line pointer cache [NLP_H][NLP_L] written by DR_EXEC (IDX_SWSTK=4)
EXPH    RES 1       ; expression result hi                                     (IDX_EXP=5)
EXPL    RES 1       ; expression result lo
LNUMH   RES 1       ; scratch line number hi                                   (IDX_LNUM=6)
LNUML   RES 1       ; scratch line number lo
FVAR    RES 1       ; spare since v3.0 (was FOR variable staging; FWRK replaces it) - kept so PE/SC0/RNDSEED stay in nibble range
NIBPAD  RES 1       ; unused - restores even alignment after odd-length FVAR so
                     ; everything below is reachable by REG16_TO_REG16 (idx*2)
PEH     DB <SHOWCASE_END       ; Program end pointer hi                        (IDX_PE=8)
PEL     DB >SHOWCASE_END       ; Program end pointer lo
SC0     RES 1       ; Scratch byte 0                                          (IDX_SC0=9)
SC1     RES 1       ; Scratch byte 1
RNDSEED  RES 2      ; pseudo random number                                    (IDX_RND=10)
FSP     RES 1       ; bytes used in FSTK: 0,7,14,21; 28=full - NOT nibble-indexed

; Buffers
IBUF    RES 64      ; Input buffer 64 bytes

; --- Remaining --- (order doesn't matter; not nibble-indexed)
TEMPRETH RES 1      ; SWRETURN scratch: popped continuation addr hi (was PDEPTH -
                     ; PDEPTH itself removed, see CHANGE HISTORY: paren nesting
                     ; no longer needs a separate SW-tracked depth counter, the
                     ; PUSH_RET/PARSER_RET continuation stack IS the tracking)
TEMPRETL RES 1      ; SWRETURN scratch: popped continuation addr lo

;  --- Flags & Stuff --- 
RUNFLG  RES 1       ; $01=running $00=immediate
RXSAVE  RES 1       ; Save/restore R3 in PARSE_U16, R1 in DO_MUL, and the
                     ; 3rd keyword char (STMT_EXEC -> DO_GO/DO_RU) - all
                     ; three uses have disjoint lifetimes, safe to share
BANG    RES 1       ; '!' relop-invert modifier: $00 clear, $FF armed -
GSSTK   RES 8       ; GOSUB return-address stack, 4 levels x [hi][lo]
GSSP    RES 1       ; GOSUB stack offset into GSSTK: 0,2,4,6; 8=full
FSTK    RES 44      ; FOR frames, 4 levels x 11 bytes (FWRK order reversed, see DO_FOR):
                    ; frame bytes 0..10 = FWRK+8..FWRK+0 [step3..step0][limit3..limit0][var], then body lo, body hi

;  SW call stack -- used by PARSE_EXPR / PRINT_S16 only
; R3 = index ($FF=empty, grows up). Each frame = [lo][hi].
; Push: STRA,R0 *SWBASE,R3+ (lo first), STRA,R0 *SWBASE,R3+ (hi).
; Pop:  LODA,R0 *SWBASE,R3- (hi first), LODA,R0 *SWBASE,R3- (lo).
; Also now holds PUSH_RET/SWRETURN continuation addresses (2 bytes each),
; interleaved LIFO with OPS_HIT's own operand pushes - see CHANGE HISTORY.
SWBASE  RES 96      ; SW stack base. Guard fires with ERR_EXPR when full.
SWCAP_LIMIT EQU 92   ; PUSH_RET refuses a new 2-byte push at/past this R3
                     ; value (leaves 2 bytes slack - a push adds 2 bytes and
                     ; R3 is the current top index, so at R3=46 the next push
                     ; would land at 46+47=bytes 46-47, exactly filling the
                     ; 48-byte SWBASE; refusing at 46 catches this cleanly)

VARS    RES 104     ; A-Z variables, 4 bytes each (MBF4: exponent first)

; --- MBF4 library work area (46 bytes in the library; T0 is the EXP pair here).  Order matters: FB = FA+5, FDB = FA+4.
FA      RES     4               ; FLT_A [E][S|M1][M2][M3]: left operand and result
FDB     RES     1               ; FA+4: guard byte below FA's mantissa (bit 7 = round bit)
FB      RES     4               ; FLT_B [E][S|M1][M2][M3]: right operand (FB = FA+5)
FBG     RES     1               ; FB+4: guard byte below FB's mantissa
FSA     RES     1               ; result sign (bit 7); FA's sign inside FLT_ADD
FSB     RES     1               ; FB's sign (bit 7), FLT_ADD only
FER     RES     1               ; working result exponent
FDE     RES     1               ; PARSE sign flag / PRINT decimal exponent (live across MUL_BY_TEN, DIV_BY_TEN); FLT_TO_INT scratch
FMA     RES     3               ; MUL multiplicand copy (hi,mid,lo)
FDV     RES     3               ; DIV divisor (hi,mid,lo)
RD      RES     3               ; DIV remainder (hi,mid,lo)
RT      RES     3               ; DIV trial-subtract result
T2      RES     1               ; PRINT: decimal exponent parked while FLT_TO_INT clobbers FDE
FPLIM   RES     1               ; PRINT: number of significant digits after trimming trailing zeros
FPY     RES     1               ; PRINT: index of the next digit in DIG
DIGI    RES     1               ; PRINT: digits extracted so far (0..7)
DIGV    RES     1               ; PRINT: current digit value
DIG     RES     8               ; PRINT digit buffer, DIG..DIG+6 used (the 65C02 used IBUF; here a private buffer)
; v3.3: the trig routines borrow the 8 bytes of DIG as scratch.  This is safe: FLT_PRINT fills DIG only after the expression has been
; evaluated, and no function call is live while a number is being printed.  No new RAM is used.
ZV      EQU     DIG             ; Horner: z = t*t parked here (4 bytes, packed like FA)
QF      EQU     DIG+4           ; SIN/COS: quadrant offset on entry, then n = (q+offset) AND 3
HPTR    EQU     DIG+5           ; coefficient-stream pointer, 2 bytes high byte first, points ONE BYTE BEFORE the next 4-byte entry
HCNT    EQU     DIG+7           ; Horner: coefficients still to apply
PSAVE   RES     4               ; parked copy of FA (replaces PUSH_FLT_A / POP_FLT_A)
FPN     RES     1               ; PARSE: count of fraction digits ahead of IP
T0      EQU EXPH            ; FLT_TO_INT result / FLT_FROM_INT input = EXPH:EXPL (high byte first)
FWRK    RES 9               ; FOR work block: [var offset][limit 4][step 4]


; =============================================================================
;  Pre-loaded SHOWCASE program (v3.3: floating point, 8-step RND, SIN/COS)
;
;  Line format: <lineno_hi> <lineno_lo> <body_ASCII> <NUL>
;  Format: DB hi,lo,"text",$00  -- hi-then-lo matches DR_EXEC record format.  DQ=$22 (double quote); a ';' outside a quoted DB string is $3B.
;  Every body stays under 60 bytes (IBUF is 64): the lines can be LISTed and typed back in.
;
;  Lines  10-190: feature demos: PRINT / CHR$ / TAB (30-44), integer arithmetic (60-70), FLOATING POINT (71-79: true division, decimals,
;                 6-digit printing, no 16-bit wrap, RND: 77 seed + 0..1 value, 78 five raw RNDs, 79 three 0..1 values; v3.2),
;                 comparisons incl. decimals (80-139), a GOTO loop (140-190)
;  Lines 195-241: GOSUB/RETURN, 4 levels deep
;  Lines 250-297: FOR/NEXT: STEP -4, nested 3x3 table, and (271-274) a fractional STEP 0.25; 297 jumps to the trig section
;  Lines 520-590: SIN and COS (v3.3): values, a degrees table, S^2+C^2, an odd/small argument, a one-period sine wave plot (TAB( with a float
;                 expression); 590 jumps back to 300
;  Lines 300-650: Mandelbrot set, floating point: 44 columns x 21 rows, escape count in a GOSUB'd FOR loop (levels: 2 loops + 1 inside the sub)
;
;  NOT demonstrated, because an error ENDS the run (try them at the prompt):
;     PRINT 1/0           ?Z   divide by zero (inside a running program: ?Z@line)
;     PRINT 100000*100000*100000*100000*100000*100000*100000*100000   ?O   overflow (result beyond about 1.7E38)
;     PRINT TAB(40000)    ?R   integer use out of range (|x| >= 32768: TAB(, CHR$(, GOTO/GOSUB target, line number)
;     PRINT SIN(60000)    ?R   SIN/COS argument beyond about +-51471 (|x|*2/PI >= 32768)
;  No LIST in the showcase.  Not shown either: E notation (unsupported), INT() (none).
; =============================================================================
PROG:
        DB 0,10,"REM -- Rem does nothing --", $00
        DB 0,20,"PRINT ",$22,"-- miniBASIC2650 Showcase --",$22,$00
        DB 0,30,"PRINT ",$22,"--- PRINT / CHR$ / TAB ---",$22,$00         ; 30
        DB 0,40,"PRINT CHR$(65);CHR$(66);CHR$(67)",$00                    ; 40  ABC via CHR$
        DB 0,43,"PRINT",$00                                                ; 43  newline
        DB 0,44,"PRINT TAB(4);",DQ,"Hi",DQ,$00                            ; 44  TAB(4) then "Hi"
        DB 0,50,"PRINT ",$22,"--- ARITHMETIC ---",$22,$00
        DB 0,60,"PRINT ",$22,"3+4=",$22,$3B,"3+4",$3B,$22,"  10-3=",$22,$3B,"10-3",$3B,$22,"  6*7=",$22,$3B,"6*7",$00
        DB 0,70,"PRINT ",$22,"20/4=",$22,$3B,"20/4",$00
        DB 0,71,"PRINT ",DQ,"--- FLOATING POINT ---",DQ,$00                  ; 71  v3.1: true division, decimals, 6 digits, RND
        DB 0,72,"PRINT ",DQ,"7/2=",DQ,";7/2;",DQ,"  1/3=",DQ,";1/3;",DQ,"  22/7=",DQ,";22/7",$00   ; 72  3.5  0.333333  3.14286
        DB 0,73,"PRINT ",DQ,"0.1+0.2=",DQ,";0.1+0.2;",DQ,"  2.5*1.5=",DQ,";2.5*1.5",$00         ; 73  0.3  3.75
        DB 0,74,"PRINT ",DQ,"-0.5*3=",DQ,";-0.5*3;",DQ,"  1-0.9=",DQ,";1-0.9",$00               ; 74  -1.5  0.1
        DB 0,75,"PRINT ",DQ,"1234567=",DQ,";1234567;",DQ,"  1/7=",DQ,";1/7",$00                  ; 75  6 significant digits: 1234570  0.142857
        DB 0,76,"PRINT ",DQ,"32767+1=",DQ,";32767+1;",DQ,"  100000*100000=",DQ,";100000*100000",$00   ; 76  no 16-bit wrap: 32768  10000000000
        DB 0,77,"PRINT ",DQ,"RND=",DQ,";RND;",DQ,"  ABS(RND)/32768=",DQ,";ABS(RND)/32768",$00     ; 77  integer seed, then a 0..1 value (varies per session)
        DB 0,78,"PRINT ",DQ,"RND x5: ",DQ,";RND;",DQ," ",DQ,";RND;",DQ," ",DQ,";RND;",DQ," ",DQ,";RND;",DQ," ",DQ,";RND",$00   ; 78  v3.2: five RNDs - no halving runs (v3.1 printed runs like 16352 8176 4088)
        DB 0,79,"PRINT ABS(RND)/32768;",DQ," ",DQ,";ABS(RND)/32768;",DQ," ",DQ,";ABS(RND)/32768",$00              ; 79  v3.2: three 0..1 values
        DB 0,80,"PRINT ",$22,"--- COMPARISONS ---",$22,$00
        DB 0,90,"IF 3<9 PRINT ",$22,"3<9 ok",$22,$00
        DB 0,100,"IF 7=7 PRINT ",$22,"7=7 ok",$22,$00
        DB 0,110,"IF 9!=2 PRINT ",$22,"9!=2 ok",$22,$00
        DB 0,120,"IF 9!<4 PRINT ",$22,"9!<4 ok",$22,$00
        DB 0,130,"IF 9>4 PRINT ",$22,"9>4 ok",$22,$00                      ; native '>'
        DB 0,131,"IF 5<2+9 PRINT ",$22,"5<2+9 ok",$22,$00                  ; relop RHS is a whole + - expr
        DB 0,132,"IF 1+2*3>4*5-14 PRINT ",$22,"7>6 ok",$22,$00             ; * / then + - then relop
        DB 0,133,"IF 0.5<0.75 PRINT ",DQ,"0.5<0.75 ok",DQ,$00                ; 133 v3.1: decimals compare
        DB 0,134,"IF 0.1+0.2=0.3 PRINT ",DQ,"0.1+0.2=0.3 ok",DQ,$00          ; 134 v3.1: '=' is exact, but the sum rounds to the same MBF4 value
        DB 0,135,"IF 6!>9 PRINT ",$22,"6<=9 ok",$22,$00                    ; '!' inverts '>'
        DB 0,136,"IF 9>4 THEN PRINT ",$22,"THEN ok",$22,$00                ; optional THEN
        DB 0,137,"LET K=5",$00                                             ; optional LET
        DB 0,138,"IF K>3 THEN LET K=K+1",$00                               ; THEN + LET + '>'
        DB 0,139,"PRINT ",$22,"LET/THEN: K=",$22,$3B,"K",$00               ; expect K=6
        DB 0,140,"PRINT ",$22,"--- LOOP via GOTO ---",$22,$00
        DB 0,150,"I=1",$00
        DB 0,160,"IF 5<I GOTO 190",$00
        DB 0,170,"PRINT I",$3B,$00
        DB 0,180,"I=I+1",$00
        DB 0,185,"GOTO 160",$00
        DB 0,190,"PRINT ",$22,"",$22,$00
        DB 0,195,"PRINT ",$22,"--- GOSUB/RETURN ---",$22,$00
        DB 0,196,"GOSUB 210",$00
        DB 0,197,"PRINT",$00                                                ; newline after the nested-call line
        DB 0,198,"GOTO 250",$00                                            ; over the subs, into FOR/NEXT
        DB 0,210,"PRINT ",$22,"L1 ",$22,$3B,$00
        DB 0,211,"GOSUB 220",$00
        DB 0,212,"PRINT ",$22,"L1-done ",$22,$3B,$00
        DB 0,213,"RETURN",$00
        DB 0,220,"PRINT ",$22,"L2 ",$22,$3B,$00
        DB 0,221,"GOSUB 230",$00
        DB 0,222,"PRINT ",$22,"L2-done ",$22,$3B,$00
        DB 0,223,"RETURN",$00
        DB 0,230,"PRINT ",$22,"L3 ",$22,$3B,$00
        DB 0,231,"GOSUB 240",$00
        DB 0,232,"PRINT ",$22,"L3-done ",$22,$3B,$00
        DB 0,233,"RETURN",$00
        DB 0,240,"PRINT ",$22,"L4-deepest ",$22,$3B,$00                     ; 4 levels deep - the documented max
        DB 0,241,"RETURN",$00
        DB 0,250,"PRINT ",$22,"--- FOR / NEXT ---",$22,$00
        DB 0,255,"FOR I=1 TO 5",$00
        DB 1,4,"PRINT I*I",$3B,$22," ",$22,$3B,$00                              ; 260  expect 1 4 9 16 25
        DB 1,9,"NEXT I",$00
        DB 1,14,"PRINT",$00
        DB 1,15,"FOR X=0 TO 1 STEP 0.25",$00                                  ; 271 v3.1: fractional STEP, expect 0 0.25 0.5 0.75 1
        DB 1,16,"PRINT X",$3B,DQ," ",DQ,$3B,$00                              ; 272
        DB 1,17,"NEXT X",$00                                                  ; 273
        DB 1,18,"PRINT",$00                                                   ; 274
        DB 1,19,"FOR A=1 TO 3",$00                                            ; nested: 3x3 table
        DB 1,24,"FOR B=1 TO 3",$00
        DB 1,29,"PRINT A*B",$3B,$22," ",$22,$3B,$00
        DB 1,34,"NEXT B",$00
        DB 1,35,"PRINT",$00
        DB 1,36,"NEXT A",$00
        DB 1,37,"FOR C=10 TO 2 STEP -4",$00                                     ; 293  STEP demo: expect 10 6 2
        DB 1,38,"PRINT C",$3B,$22," ",$22,$3B,$00                              ; 294
        DB 1,39,"NEXT C",$00                                                   ; 295
        DB 1,40,"PRINT",$00                                                    ; 296
        DB 1,41,"GOTO 520",$00                                                 ; 297 v3.3: on to the trig section (it ends with GOTO 300)
        DB 1,44,"PRINT ",$22,"--- MANDELBROT ---",$22,$00                  ; 300
        DB 1,49,"M=16",$00                                                 ; 305 iteration limit (a variable FOR limit)
        DB 1,54,"FOR R=0 TO 20",$00                                        ; 310 21 rows       (FOR level 1)
        DB 1,64,"LET D=(R-10)/8",$00                                       ; 320 imaginary part -1.25..1.25 (v3.1: was R*6-64 in 1/64 units)
        DB 1,84,"FOR Q=0 TO 43",$00                                        ; 340 44 columns    (FOR level 2)
        DB 1,94,"LET C=Q/16-2.25",$00                                      ; 350 real part -2.25..0.4375 (v3.1: was Q*4-144 in 1/64 units)
        DB 1,104,"A=C",$00                                                 ; 360
        DB 1,105,"B=D",$00                                                 ; 361
        DB 1,106,"E=0",$00                                                 ; 362
        DB 1,108,"GOSUB 600",$00                                           ; 364 escape-count subroutine (below)
        DB 1,174,"IF E>0 THEN PRINT CHR$(E+32);",$00                       ; 430 char for iteration depth
        DB 1,184,"IF E=0 THEN PRINT CHR$(32);",$00                         ; 440 space for unescaped
        DB 1,194,"NEXT Q",$00                                              ; 450
        DB 1,224,"PRINT",$00                                               ; 480 end of row newline
        DB 1,234,"NEXT R",$00                                              ; 490
        DB 1,254,"END",$00                                                 ; 510
        DB 2,8,"PRINT ",DQ,"--- TRIG: SIN COS (radians) ---",DQ,$00 ; 520 v3.3: trig section, runs between FOR/NEXT and the Mandelbrot plot
        DB 2,13,"PRINT ",DQ,"SIN(1)=",DQ,";SIN(1);",DQ,"  COS(1)=",DQ,";COS(1)",$00 ; 525 0.841471  0.540302
        DB 2,16,"K=0.0174532925",$00                        ; 528 radians per degree
        DB 2,18,"FOR D=0 TO 80 STEP 20",$00                 ; 530 degrees
        DB 2,23,"PRINT D;",DQ," deg  SIN ",DQ,";SIN(D*K);",DQ,"  COS ",DQ,";COS(D*K)",$00 ; 535 0.342020/0.939693 ... 0.984808/0.173648
        DB 2,28,"NEXT D",$00                                ; 540 
        DB 2,33,"PRINT ",DQ,"S^2+C^2=",DQ,";SIN(2.5)*SIN(2.5)+COS(2.5)*COS(2.5)",$00 ; 545 1 (to within 6 digits)
        DB 2,38,"PRINT ",DQ,"SIN(-1)=",DQ,";SIN(-1);",DQ,"  SIN(.001)=",DQ,";SIN(0.001)",$00 ; 550 odd function, small argument
        DB 2,43,"FOR X=0 TO 6.5 STEP 0.5",$00               ; 555 one period of a sine wave, TAB( takes the float
        DB 2,48,"PRINT TAB(20+SIN(X)*18);",DQ,"*",DQ,$00    ; 560 column 2..38
        DB 2,53,"NEXT X",$00                                ; 565 
        DB 2,78,"GOTO 300",$00                              ; 590 on to the Mandelbrot plot
        DB 2,88,"FOR N=1 TO M",$00                                         ; 600 escape-count subroutine, a FOR loop
        DB 2,98,"IF E>0 GOTO 640",$00                                      ; 610   (FOR level 3, inside a GOSUB, inside
        DB 2,108,"T=A*A-B*B+C",$00                                   ; 620    levels 1-2). Escaped points skip the
        DB 2,113,"B=2*A*B+D",$00                                        ; 625    maths by GOTOing the NEXT: leaving
        DB 2,114,"A=T",$00                                                 ; 626    via NEXT (not out of the loop) leaves
        DB 2,118,"IF 4<A*A+B*B THEN IF E=0 THEN E=N",$00           ; 630    no frame behind. No parentheses:
        DB 2,128,"NEXT N",$00                                              ; 640    the relop takes a whole + - * / RHS
        DB 2,138,"RETURN",$00                                              ; 650
SHOWCASE_END:

        END
