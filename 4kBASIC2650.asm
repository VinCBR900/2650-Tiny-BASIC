; 4kBASIC2650.asm       Fast 4k Tiny BASIC interpreter for Signetics 2650 
; Version: v5.7
; By Vincent Crabtree, 2026.  MIT License
; Date:    Oct 2026
;
; Target:  Standalone (no PIPBUG ROM). Code ORG 0. I/O routines embedded.
;          Single 8192-byte address space (2650 bits 15:13 always 0).
;
; Assembler: asm2650.c v1.13+  Simulator: pipbug_wrap.c v2.1
; Build:
;   gcc -Wall -O2 -o asm2650 asm2650.c
;   gcc -Wall -O2 -DGAMER -o pipbug_wrap pipbug_wrap.c
;
;   ./asm2650 --warn=ALL 4kBASIC2650.asm 4kBASIC2650.hex
;   grep -n "^CHIN \|^COUT \|^ROMEND " .\4kBASIC2650.LST
;   ./pipbug_wrap --entry 0 --chin 0x<addr> --cout 0x<addr> 4kBASIC2650.hex
;
;
; =============================================================================
; LANGUAGE REFERENCE (for users)
; =============================================================================
;
; Statements:
;   END  FOR <var>=<expr> TO <expr> [STEP <expr>]  FREE  GOSUB <expr>
;   GOTO <expr>  IF <expr> <relop> <expr> THEN <stmt>  INPUT <var>
;   [LET] <var>=<expr>  LIST [<start>,<end>]  NEW  NEXT [<var>]
;   POKE <addr>,<val>  PRINT  REM  RETURN  RUN
;
;   LET is optional, THEN is required.
;
; PRINT items: "literal", expr, TAB(n), CHR$(n), or HEX$(n); separate with
;   ';'. TAB(n)/CHR$(n)/HEX$(n) are PRINT-only, not general functions.
;   HEX$(n): 4-digit unsigned hex, n truncated to 16 bits.
;
; Functions (all take parenthesized args, e.g. ABS(n)):
;   ABS(n)  NEG(n)  NOT(n)  bitwise AND(a,b)/OR(a,b)/XOR(a,b)
;   PEEK(addr): byte at addr, 0-255.  POKE is a statement, see above.
;   RND(n): pseudorandom integer, 0 to n-1.
;   USR(addr): call machine code at addr.
;
; Arithmetic: + - * / % (mod), ^ (power), unary -.
; Precedence, tightest first: ^  then * / %  then + -  then relops.
;   * / % and + - are left-to-right among themselves; ^ binds to each atom
;   before * / are combined. Parentheses override.
; Relops: =  <  >  <=  <>  >=  all native (no inversion prefix needed).
;   Lowest precedence; a relop's operands are whole + - * / ^ expressions.
;
; Numbers  : signed 16-bit (-32768 .. 32767)
; Variables: A-Z (26), 16-bit signed; no arrays or string variables
;
; KNOWN LIMITATIONS
;
; UPPERCASE required for keywords and variable letters. PRINT "String
; literals" are taken verbatim regardless of case, as always.
;
; HEX LITERALS ($HHHH)
;   Accepted in stored lines and in direct/immediate entry (V5.1). Digits 0-9
;   and UPPERCASE A-F only; '$' not followed by a hex digit is a syntax error
;   (in a stored line it is kept as a plain '$'). Any number of digits is
;   accepted and wraps mod 65536 ($10000 = 0, $FFFF = -1). LIST always prints 4.
;
; GOSUB/RETURN NESTING
;   Up to 8 levels deep (GSSTKLIM=16 bytes, 2 per level).
;
; FOR / NEXT
;   FOR <var>=<start> TO <limit> [STEP <step>], then NEXT [<var>]. Default
;   step is 1 (positive or negative steps both work via STEP). A var+step that
;   overflows 16 bits ends the loop (V5.4), so any signed range terminates.
;   Up to 4 levels deep (FORSTKLIM=28 bytes, 7 per level)
;   - Test at end so Body always runs at least once 
;   - Variable after NEXT is consumed but not checked against the frame
;   - Leaving a loop early (GOTO/GOSUB target outside it) not supported.
;
; EXPRESSION NESTING
;   Guarded by PE_RAS_LIMIT (a hardware-RAS-depth check on PARSE_EXPR
;   entry): deeply nested parens/operators raise ?8 (ERR_NEST) gracefully
;   rather than corrupting the hardware return stack.
;
; STATEMENT DISPATCH matches full keyword spelling 
;
; STORAGE / INPUT
;   Full random line editing: a numbered line is inserted in order,
;   replaces an existing line of the same number, or (empty body)
;   deletes it. Permitted line numbers are 0-32767.
;
; SYNTAX VALIDATION
;   Minimal; malformed constructs may produce a generic syntax/runtime
;   error rather than a specialised diagnostic.
;
; =============================================================================
; IMPLEMENTATION NOTES (for maintainers)
; =============================================================================
;
;        CC SEMANTICS (2650 ALU)
;   ADD/SUB: result<0 -> LT   result>0 -> GT   result=0 -> EQ
;   Carry bit (PSL bit 0): C=1 = carry / no-borrow (independent of CC).
;   Carry test: TPSL $01  ->  CC=EQ if C=1 (carry), CC=LT if C=0 (no carry).
;   Carry skip (no carry): BCTR,LT. Carry taken (carry): fall through or BCTR,EQ.
;   PREFERRED: Use WC idiom (CPSL $08 / lo-add / PPSL $08 / hi-add / CPSL $08) for 16-bit adds.
;   Unsigned compare: COM=1 is the global default (set once in MAIN); COMA/COMI then
;   compare unsigned. SUBA is NOT a compare: its CC is the sign of the 8-bit RESULT.
;   Signed compare: CPSL $02 / COMA / PPSL $02 (see CMP_LNUM_EXP).
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
;   PARSE_EXPR entry guard: SPSU/ANDI/COMI fires ERR_NEST if SP>=5 at entry.
;
;        SCRATCH REGISTER CONVENTIONS
;   R0  working register, arithmetic, I/O.
;   R1  index register (LODA/STRA BASE,R1); also PRINT_S16 digit buffer index.
;       Clobbered by INC_ET (INC_TMP/INC_EXP shared body). Callers verified safe.
;   R2  long-lived variable letter (SE_NOTKW/DO_INPUT/DO_FOR, preserved across PARSE_EXPR)
;       Never written by subroutines except SE_NOTKW (LET/bare assign), DO_FOR.
;   R3  loop counter (BDRR/BIRR); STORE_LINE shift count. SW expr-stack pointer.
;       RDLINE (V4.2): IBUF offset, $FF=empty sentinel (",Rx+" pre-increments
;       before access -- see SWBASE convention -- so empty must be one below
;       the first valid index, not 0).
;
;        KNOWN OPEN ITEMS
;   COLON-01: ':' multi-statement not supported - likely never due to RAS.
;   FOR-01:   NEXT variable not checked against frame var (smallest code, by spec).
;   FOR-02:   Body always executes at least once (no skip-if-false-at-entry, by spec).
;   REC-01:   Triple-nested atom-dispatch (1+ABS(NEG(AND(...)))) hits RAS limit guard
;
; CHANGE HISTORY
;
;
; V5.7 (2026-10) - ROMEND $0E39 -> $0EBE (3641 -> 3774 bytes, +133).  SPEED.  
;   - Length-prefixed line records: [lineH][lineL][len][body...][NUL], len = body byte
;     count (NUL not counted).  
;   - ADV_TMP_PAST_REC is O(1): one length read + add. Writers/readers: TRY_STORE_LINE
;     TSL_BLEN, DR_EXEC, DO_LIST, FIND_LINE/FIND_INS, DEL_REC.  
;   - DR_EXEC register-pair copies inlined; backward GOTO/GOSUB line cache (GCACHE, PROTO).
;   - GOTO/GOSUB cache miss (DG_TOP): the next-slot update LODA/ADDI/COMI/BCFR/BCTR/EORZ
;     (assembler [skipbyte] advisory at DGC_NOWRAP) is now LODZ,R1 / EORI,R0 4: GC_NEXT
;     only holds 0 or 4 and R1 still holds it.  -9 bytes, -3 instructions per miss.
;
; V5.6 (2026-10) - ROMEND $0D94 -> $0E39 (3476 -> 3641 bytes, +165).  SPEED.
;   - #3 (+25 bytes, -4.8%): PE_SAFE no longer pushes the FT marker, sets TMP and runs
;     MATCH_KW/WORD_START at every expression start (~57 instructions) just to find that
;     the text is not a function.  A function only starts with a token, or two letters 
;     in direct mode; a running program is fully tokenized so RUNFLG skips WORD_START.
;   - #7 (+36 bytes, -22.4%): GOTO/GOSUB to a line above the current one (while running)
;     search from SWSTK, the next record, instead of from PROG (FIND_LINE_FWD).  Backward
;     jumps and self-loops still search from the top
;   - #2 (+95 bytes, -6.5%): DIV16 power-of-two divisor fast path: |D| = 1..128 shifts
;     |N| right k times in alternate-bank registers (3 instructions per bit) and takes
;     the remainder as |N| & (|D|-1); |D| = 256 is a byte move; everything else uses the
;     restoring loop unchanged.
;   - #4 (+9 bytes, -1.7%): MUL16 keeps the multiplier byte in R0 and tests its MSB with
;     IORZ (7 instructions per pass, 11 with an add, was 19); a 16-bit multiplier is two
;     byte passes (skipped if the high byte is 0); if only the right operand is >= 256
;     the operands are swapped so the multiplier is one byte.
;
; V5.5 (2026-10)
;   FIXED FUNC-01: single-argument functions (ABS NEG NOT PEEK USR RND) now require
;   '(' and ')' around the argument (ARG_OPEN/ARG_CLOSE, leaf helpers so nesting depth
;   per function is unchanged). Before, ABS(-5)+10 printed 5, NEG(3)+1 printed -4,
;   NOT(0)+1 printed -2: the argument parse swallowed everything after the ')'
;   (since V4.28). A function without parentheses ("ABS -5") is now a syntax error.
;   PARSE_2ARGS reuses ARG_OPEN. 
;
; V5.4 (2026-10) - ROMEND $0ECE -> $0D73 (3790 -> 3443 bytes, -347) from 4kBASIC2650wip.asm
;   - PRINT_S16 per-column rewrite, WC set once per column (-19).  Also fixes COUT being
;     called with WC=1 (its RRR would have rotated through carry on real hardware).
;   - NEGFLG polarity 0=negate, written unconditionally by PARSE_S16/SETUP_MULDIV, so no
;     clear in MU_DONE; ABS_TMP+ABS_EXP merged into ABS_X (+ABS_EXP entry) (-23).
;   - ADD_CORE (shared EXP += pair at IPH+R0), used by ADD16_SAVE_EXP, TL_MULADD and
;     DO_NEXT; TL_MULADD preloads EXP with the digit so CARRY_INTO_EXPH is gone;
;     PARSE_U16 calls TL_MULADD instead of its own x10 loop; DIGIT_CHECK shared by 4
;     sites (-74 together).  VARS moved up to sit within 255 bytes of IPH (ADD_CORE).
;   - CMP_LNUM_EXP (uBASIC CMP_OPS): signed 16-bit compare shared by IF and NEXT (-32).
;   - SWSTK is the program counter: GOTO/GOSUB resolve the line themselves, RETURN and
;     NEXT write SWSTK directly, DR_LP always does TMP=SWSTK.  GOTOFLG, DR_GOTO,
;     DR_FORNLP, DRT_GO, DO_GOSUB's duplicate tail, VCLR_RUNFLG gone (RUNFLG takes
;     GOTOFLG's RAM slot).  DO_FOR/DO_NEXT use FORBASE-1,R1+ / ,R2- indexing and a
;     5-byte pop loop, NEXT decides loop/exit from SPSL xor the step's sign bit (-133
;     together with the above).
;   - SET_TMP (TMP = R0:R1) replaces 5 inline "TMP = table" sequences (-18).
;   - DO_LET merged into SE_NOTKW (LET row points at it) -15; DO_NEW via TMP/REG16 -10;
;     GOSUB/RETURN stack pre-inc/dec -5; SET_IP_IBUF folded into RDLINE -7;
;     CMP_TMP_PE with COMA, INC_ET BCFR,EQ, WSKIP IORZ, dead LODI in CHIN, EATWORD and
;     PARSE_FACTOR single unsigned range test (-14 together).
;   - STMT_EXEC: blank line / empty THEN body no longer print ?4 (+3, uBASIC v2.15).
;   BUGS FIXED (all present in V5.3):
;   - IF compared wrongly whenever the operands' difference was >= 128 (biased SUBA's CC
;     is the sign of the 8-bit RESULT, for the hi byte AND the lo byte): IF 200>10 was
;     false, IF 200<10 true, 20000>-20000 false.  The Mandelbrot showcase rendered with
;     this bug.  NEXT used the same flawed compare.
;   - FOR frames were written at FORBASE+FORSP (FORSP bumped first): the 4th frame ran 7
;     bytes past FORBASE into IBUF, so INPUT inside a 4-deep FOR nest corrupted the loop.
;   - RUNFLG stayed set when a program ran off its end (PC == PE): the next immediate
;     mode error printed a stale "@line".
;   - MAIN's variable clear wrote VARS+51..+1 and never VARS+0 (BDRR decrements first).
;   - A FOR loop whose var+step overflows 16 bits now ends (NEXT tests OVF).
;
; V5.3 (2026-09) - ROMEND $0E61 -> $0EDC (3804 bytes)
;   - MUL16: MSB-first shift-add (Horner), 16 passes or 8 if multiplier<256.
;   - DIV16: restoring shift-subtract, 16 passes or 8 if dividend<256.
;   - MATCH_KW early-out: anything but two letters in a row no longer scans table.
;   - INC_IP: dedicated ~4-instruction path vs ~10 via  shared INC_ET.
;   - ADV_TMP_PAST_REC: indexed scan, TMP advanced once
;
; V5.2 (2026-09) - ROMEND $0E78 -> $0E61 (3681 bytes)
;   - Variable plumbing clean up: PARSE_VAR_SAVE now leaves R2 = VARS byte offset
;     (2*(letter-'A')) instead of ASCII letter, and drops dead SC0 copy.
;     DL_STORE uses R2 directly as index register: no letter->offset conversion.
;   - PARSE_VAR_SAVE range test is one SUBI/COMI (relies on global COM=1).
;   - DO_FOR now takes its variable via PARSE_VAR_SAVE (was: unvalidated letter,
;     dead STRA FORVAR, own INC_IP). 
;   - BUG FIXED: FOR with a non-A-Z variable (e.g. FOR [=1 TO 2) computed a VARS
;     offset outside VARS; it now raises the same variable error as LET/INPUT.
;   - PF_LOADVAR: index kept in R1 across INC_IP (bank-1, R1 survives).
;
; V5.1 (2026-09) - ROMEND $0E6A -> $0E78 (3704 bytes, +14)
;   - Direct/immediate-mode hex constants ($BEEF): PF_NUM now hands '$' to the
;     new PARSE_HEX. HEX_PEEK/PARSE_HEX are TL_MAYBEHEX's former inline code,
;     factored out so the tokenizer and the expression parser share one loop.
;   - PARSE_HEX is tail-jumped from PF_NUM (not called) to stay within the
;     PE_RAS_LIMIT budget: worst legal nesting peaks at SP=7, as for decimal.
;
; V5.0 (2026-09) - ROMEND $0E6A, 3690 bytes; PROG 1543 -> 1403 bytes
;   - Minor DO_PRINT code refactor.
;   - Added keyword token EQUs (TOK_END..TOK_RUN, TOK_ABS..TOK_XOR), generated
;     from KW_TAB/FUNC_TAB row order by tools/tokshowcase.py.
;   - SHOWCASE now stored pre-tokenized (73 records)
;
; V4.28 (2026-09) - ROMEND $0ED5 -> $0E70 (3696 bytes)
;   - Deleted GETCI and vector - WSKIP leaves char in R0, uses INC_IP instead. 
;     Cleaned up multiple unnecessary LODA,R0 *IPH after WSKIP calls.
;   - Abused EXPR for single arg function, deleted CHECK_LPARENS/Check_RPARENS 
;     and vectors. Inlined left Paren check at PARSE_2ARGS. 
;   - Zero-page vector sweep, added common REG16_TO_REG16 vector and shims.
;
; V4.27 (2026-09) - ROMEND $0EEF -> $0ED6 (3798 bytes)
;   - DO_FOR STEP test now peeks (WSKIP) instead of GETCI/DEC_IP on default-step.
;   - Deleted DEC_IP, DEC_ET body and VDEC_IP zero-page vector as no callers.
;   - BUG-FOREQ-01: space before '=' (FOR I = 1 TO 3, FOR I =1 TO 3) gave ?0.
;     DO_FOR used GETCI which does not skip whitespace, replaced with WSKIP.
;
; V4.26 (2026-09) - ROMEND $0F1F -> $0EEF 
;   - FETCH16_IP - Replaces four inline LODA *IPH/STRA/INC_IP x2
;     sequences: DLS_BODY, PF_CONST (tail-called), DLS_CONST, and DR_EXEC.
;   - DR_EXEC: the 2-byte line-number header is now IP=TMP, FETCH16_IP,
;     EXP->CUR instead of the *TMPH,R1- loop plus two INC_IPs 
;   - ADD16_SAVE_EXP and CP_EXP_RET: the inline SWBASE,R3- pop into SAVE is now
;     POP_SAVE_TO_TMP + REG16_TO_REG16 (TMP->SAVE).
;   Left as is: TSL_HDR (LNUM -> *TMP) 1 call so helper cost greater than benefit.
;   DEL_LP/OG_NB (memory-to-memory moves), FOR frame (DF_LOOP/DO_NEXT), GOSUB stack
;   (DO_GOSUB/DO_RETURN), PF_LOADVAR/DL_STORE (VARS,R1 element access), 
;   PUSH_EXP/PUSH_RET, and SWRETURN (see To-DO).
;
; V4.25 (2026-09) - ROMEND $0F8E -> $0F1F 
;   - Added REG16_TO_REG16, zero-page vector VREG16_TO_REG16 and IDX_* EQUs.
;   - Deleted superseded helpers: EXP16_TO_TMP, EXP16_TO_GOTO,
;     EXP16_TO_LNUM, EXP16_TO_ET, CUR_TO_EXP16, TMP_TO_EXP16, ET_TO_EXP16,
;     TMP_TO_IP, TMP_TO_SWSTK, TMP_TO_ET, IP_TO_TMP; and appropriat vectors.
;   - Converted additional 12-byte inline LODA/STRA copies:
;     SWSTK->TMP/GOTO->TMP (DR_LP), RNDSEED->TMP (RND), EXP->ARGA (PARSE_2ARGS),
;     ARGA->LNUM (LIST), EXP->POW/SAVE->TMP (CHECK_POW), PE->EXP (OPEN_GAP).
;   - DR_GOTO now copies GOTO->LNUM directly (was GOTO->EXP->LNUM; EXP is not
;     read again before the next statement re-parses).
;   - RAM: reordered so every copied pair is hi/lo even offset from IPH. 
;
; V4.24 (2026-09) - ROMEND $1019 -> $0FEE
;   Golf pass on the V4.23 tokenizer fixes -- no behaviour change
;   - TOKENIZE_LINE / TOK_MATCH_ROW / TL_MULADD added dedicated TKH:TKL
;     pair instead of TMPH:TMPL, TSL_WRITE no longer saves/restores TMP.
;   - TL_STR_LP re-reads the just-stored char with LODA,R0 TOKBUF,R3 instead
;     of caching it in SC0 across VINC_IP. 
;   - TL_DEC_DONE and TL_HEX_DONE shared one constant-emit tail
;     (TL_EMIT_CONST); they differed only in the token byte.
;   - ORG 4096 re-enabled; memory map is back to the real split (IPH=$1000).
;
; V4.23 (2026-09) - ROMEND $0FFC -> $1019 overflow
;   Tokenizer step 3 (encode side: TOKENIZE_LINE + TRY_STORE_LINE)
;   - TSL_WRITE: TOKENIZE_LINE clobbers TMPH:TMPL. Fix: TMPSAVEH/TMPSAVEL, 
;     superseded in V4.24 by TKH:TKL pair.
;   - TL_LP letter typo BCFR,LT instead of BCTR,LT
;   - TL_STR_LP tested R0 for the closing quote after ZBSR *VINC_IP clobber.
;     Fix: cache in SC0 across the call. 
;   - TSL_CPY copied TOKBUF up to the first $00 byte, truncating records at
;     a TOK_DEC/TOK_HEX payload zero byte. Fix: copy by count (R3-2=body+NUL)
;   - TL_MULADD: STRZ,R3 (multiplier) ran after EXP16_TO_TMP/CLR_EXP had
;     clobbered R0, so R3=0 and BDRR looped 256x: Fix: STRZ,R3 first.
;   - TL_HEX_LP: STRA,R0 SC0 ran after ZBSR *VINC_IP (R0 clobbered), so hex
;     digits accumulated a pointer byte. Fix: swap the two.
;   - TOKBUF (80) overflowed into VARS on dense-constant lines, expanded 
;     so TOKBUF now RES 128. 
;
; V4.22 (2026-09) - ROMEND $0DA3 -> $0E8D (+234 bytes)
;   Tokenizer project, step 2: dispatch + detokenization infrastructure.
;   - Token space: TOK_STMT_BASE=$80 (16 rows, $80-$8F), gap to
;     TOK_FUNC_BASE=$A0 (9 rows, $A0-$A8) reserved for future statements,
;     TOK_HEX=$FD/TOK_DEC=$FE (each followed by 2 payload bytes, hi/lo).
;   - KWH_xxx/FNH_xxx labels added to every KW_TAB/FUNC_TAB row (hi byte
;     position); TOK_STMT_OFS/TOK_FUNC_OFS are assembler-computed
;     row-index -> byte-offset tables built from those labels, so they
;     can't drift out of sync with the keyword text.
;   - MATCH_KW: O(1) fast path added ahead of the V4.21 text scan. A token
;     byte's own value range picks the offset table and row index directly
;     (no table-address comparison needed, so it's correct regardless of
;     where KW_TAB/FUNC_TAB sit in ROM); the offset then reuses a tail
;     almost identical to the old SE_HI_LO to load the handler and jump.
;   - PARSE_FACTOR: TOK_DEC/TOK_HEX recognized up front and loaded directly
;     from their 2-byte payload (O(1)) instead of PARSE_S16's per-digit
;     accumulation, which still runs unchanged for typed/direct-entry text.
;   - ADV_TMP_PAST_REC - NULL end of line skipped when part of token.
;   - DLS_BLPX (LIST) is now the detokenizer: statement/function tokens
;     print their full keyword text 
;
; V4.21 (2026-09) - ROMEND $0D99 -> $0DA3 
;  Tokenizer project step 1 - Table expansion.
;   - KW_TAB/FUNC_TAB expanded from a fixed 5-byte-stride [c1][c2][c3][hi]
;     [lo] row (matched on the first 2-3 chars only) to variable-length
;     [full keyword text][NUL][hi][lo] rows, matched on the COMPLETE
;     spelling. This table doubles as the LIST detokenizer's text
;     source once tokens land (Step 2) -- avoids 2nd duplicate table.
;   - MATCH_KW/SE_SCAN rewritten: Each row now compared char-by-char directly
;     against IP (peeked via *IPH, not consumed) using R0/R1 only. 
;   - Dispatch  requires FULL keyword spelling; trailing garbage after
;     a complete keyword is still swallowed by EATWORD like before.
;
; V4.20 (2026-09) - ROMEND $0DDE -> $0D99 
;   - Line storage/deletion (TRY_STORE_LINE/STORE_LINE/DELETE_LINE/MEMCPY/
;     DEC_PE) ported from uBASIC2650wip v2.10: merged into TRY_STORE_LINE
;     (find-and-delete loop shared by pure-delete and overwrite, then a
;     conditional insert) + DEL_REC (unchanged forward-copy delete) +
;     OPEN_GAP (builds the prospective new PE first via repeated INC_EXP,
;     checked against PROGLIM's high byte -- valid because PROGLIM=$1FFF
;     ends in $FF -- before touching anything, then shifts backward with
;     DEC_PE inlined, its only caller). MEMCPY deleted (no longer called).
;   - Line-number range validation (rejecting >32767, silently ignoring
;     line 0) unchanged from 4k's existing behavior
;
; V4.19 (2026-09) - ROMEND $0E09 -> $0DDE 
;   - SWSP/FORSP (GOSUB and FOR stack pointers) ported from uBASIC2650wip's
;     GSSP/FSP: plain 0-based byte count (0=empty), replacing the $FF/$F9
;     sentinel scheme. Removes DGS_FIRST/DGS_NEXT, DRT_SUB, DN_POP_EMPTY,
;     and DO_FOR's CPSL/PPSL $02 COM-mode toggle (the sentinel needed
;     signed interpretation under the program's global COM=1).
;   - BUG-GSSTKLIM-01 (found while re-deriving the new limit constants,
;     confirmed via a pipbug_wrap write-watchpoint on FORBASE): the old
;     GSSTKLIM=$0F let a 9th nested GOSUB push write 2 bytes past GS.
;   - DO_FOR now caches the loop variable's precomputed VARS offset in
;     FORVAR (from DL_STORE's R1, free) instead of the raw A-Z letter;
;     DO_NEXT loads directly vs recomputing (var-'A')*2 every call.
;
; V4.18 (2026-09) - ROMEND $0E82 -> $0E09 
;   - PRINT_S16: Replaced the SW-recursive shift/divide-by-10 engine (PREC,
;     16 iterations of a 4-byte rotate + conditional subtract, plus the
;     SWBASE push/pop recursion machinery) with uBASIC2650wip's iterative
;     repeated-subtraction-by-power-of-10 approach (P10_HI/P10_LO tables).
;   - PRINT_S16 no longer clobbers TMP (now clobbers R2 instead, which no
;     caller needs preserved across the call) or SC1/NEGFLG.
;   - SWRETURN kept in place (still shared with PARSE_EXPR's own recursion).
;
; V4.17 (2026-09) - ROMEND $0EB9 -> $0E85 
;   - MATCH_KW: TMP stays fixed at the table base (KW_TAB/FUNC_TAB); R1 walks
;     it via indirect-indexed addressing (*TMPH,R1) instead of advancing TMP
;     itself through ZBSR calls. Idea ported from uBASIC2650's MD_SCAN,
;     adapted to stay generic across both callers. -7 bytes.
;   - ADV_TMP_PAST_REC: now skips the 2-byte header itself (matches
;     uBASIC2650); DELETE_LINE/FIND_INS no longer pre-skip it.
;   - Shared helpers: Added TMP_TO_IP / TMP_TO_SWSTK (mirrors EXP16_TO_ET,
;     reuses its ET_STORE tail). DR_EXEC's header-save now an indexed
;     2-iteration loop (TMP untouched, enables the ADV_TMP_PAST_REC change
;     above); DLS_BODY also uses TMP_TO_IP. -15 bytes combined.
;   - Removed all case-folding (UPCASE deleted; call sites in GETCI,
;     EATWORD, PARSE_VAR_SAVE, PARSE_FACTOR, MATCH_KW stripped): keywords
;     and variable letters now require UPPERCASE, matching the spec.
;
; V4.16 (2026-09) - ROMEND $0EBA 
;   - FL_CHKLO / FI_LH: Direct-indexed peek (*TMPH,R1) replaces INC16_TMP_TO_EXP.
;   - Stored record format: Changed line terminator CR ($0D) -> NUL ($00).
;   - Shared helpers: Added ADV_TMP_PAST_REC to scan past NUL-terminated records.
;   - DR_EXEC: Direct-in-place execution from store (TMP+2), bypassing IBUF copy loop.
;   - FIXED BUG-PU16-01: Corrected PARSE_U16 pre-loop digit check bound (':' -> '9').
;   - Streamlined DO_NEW, SHOWCASE_END to PE setup deleted, now Assembly time.
;
; V4.15 (2026-09) - ROMEND $0F26 
;   - Shared helpers: Added CMP_TMP_PE for 16-bit TMP vs PE comparison.
;   - DELETE_LINE: Ported uBASIC2650 DEL_REC idiom (uses dst ptr as new PE).
;   - PARSE_U16 / TRY_STORE_LINE: Collapsed digit checks to single unsigned sub/cmp.
;   - STORE_LINE: Ported OPEN_GAP technique; indexed dst off walking PE (*PEH,R3).
;
; V4.14 (2026-09) - ROMEND $0FB7 
;   - Unsigned comparison: Enabled global COM=1 mode.
;
; V4.13 (2026-09) - ROMEND $0FB5 
;   - FIXED BUG-MUL-01: MU_LP decrement now checks carry flag (TPSL $01) instead
;     of raw CC, fixing 16-bit multiply for operands >= 256.
;
; V4.12 (2026-09) - ROMEND $0FB3 
;   - Shared helpers: Added CARRY_INTO_EXPH to consolidate carry propagation
;     in MUL16 and PARSE_U16.
;
; V4.11 (2026-09) - ROMEND $0FC8 
;   - CHIN: Removed hard ORG $286 pin to let subroutine space float naturally.
;
; V4.10 (2026-09) - Renamed 4kBASIC - ROMEND $0FC8
;   - ZPVEC-01: Vectored PUSH_EXP and PRINT_S16 via zero-page jump table (ZBSR).
;
; V4.9 (2026-08) - ROMEND $0FDB 
;   - FIXED BUG-DL2-01: DL2_LP copy loop now tests carry flag (TPSL $01) instead
;     of signed CC, fixing premature loop exit on page-boundary wraps during deletes.
;
; V4.8 (2026-08) - ROMEND $0FD9 
;   - PRINT HEX$(n): Added 4-digit unsigned 16-bit hex formatting in ROM slack.
;
; V4.7 (2026-08) - ROMEND $0F98
;   - Shared helpers: Added PUSH_RET to handle 16-bit SWBASE push.
;   - Operators: Added ^ (power) operator with error handling.
;   - FIXED BUG-MINLP-01: Fixed hang printing -32768 (invalid indexed LODR).
;
; V4.6 (2026-07) - 3879 bytes (ROMEND $0F27)
;   - FIXED: FUNCATOM-01 - functions now work as non-leading atoms, e.g.
;     "PRINT 10+ABS(A)" (previously only "PRINT ABS(A)+10" worked).
;   - Added FT_SP/FT_STK/FT_SAVE_SP/FT_SAVE/FT_N/FT_R2SAVE (72 RAM bytes)
;     and FUNC_EPILOG; PE_SAFE/EAM_ATOM/PE_NOFUNC/DO_END updated. 
;
; V4.5 (2026-06) - 3705 bytes
;   - FIXED: Function parser tracking for trailing operators (e.g., ABS(-5)+10).
;   - Relocated PEEK/USR/EXPH functions to optimize space post-COUT.
;
; V4.4 (2026-06) - 3636 bytes
;   - FIXED: RND 16-bit seed rotation bug by correctly setting PSL WC bit.
;   - FIXED: Subtraction left-operand dropping bug in EAM_MH_RET.
;   - Unified 2-argument parsing for AND/OR/XOR/POKE/LIST to save ~25 bytes.
;   - Deduplicated bare assignments (LET-less statements).
;
; V4.3 (2026-06) - 3672 bytes
;   - FIXED: Nested operator precedence clobbering bug using SWBASE stack.
;   - Added bitwise functions: AND(a,b), OR(a,b), XOR(a,b), NOT(a).
;   - Rewrote default showcase program with an expanded Mandelbrot finale.
;
; V4.2 (2026-06) - 3485 bytes
;   - Rewrote RDLINE using R3 as an IBUF offset optimization.
;   - Merged sign-handling and addition paths into shared ADD16_SAVE_EXP.
;   - Added optional line-range filtering to LIST [start,end].
;
; V4.0 - V4.1 (2026-06)
;   - Implemented function evaluation table (ABS, NEG, PEEK, USR, RND).
;   - FIXED: Signed subtraction boundary bug in line storage shift logic.
;   - Added POKE statement support.
;
; V3.0 - V3.9 (2026-06)
;   - Implemented recursive descent expression parser via software stack.
;   - Added full FOR/NEXT (4-level stack) and GOSUB/RETURN (8-level stack).
;   - Fixed critical memory layout aliasing and carry detection bugs.
;
; V2.3 - V2.8 (2026-05)
;   - Initial optimization baseline with TAB(), CHR$(), and Mandelbrot demo.
;
; =============================================================================

;  ASCII Defines
CR      EQU     $0D
LF      EQU     $0A
BS      EQU     $08
SP      EQU     $20
NUL     EQU     $00
DQ      EQU     $22

;  Token Defines (V4.22 -- tokenizer project step 2). Statement/function
;  tokens are the row index (in KW_TAB/FUNC_TAB source order) plus a base;
;  a gap is left between the two ranges for future statement growth.
FUNC_COUNT      EQU FUNC_TOKS_END-TOK_FUNC_OFS   ; number of function rows (assembler-computed)
TOK_STMT_BASE   EQU $80         ; statement token = TOK_STMT_BASE + KW_TAB row (16 rows: $80-$8F)
TOK_FUNC_BASE   EQU $A0         ; function token  = TOK_FUNC_BASE + FUNC_TAB row (9 rows: $A0-$A8)
                                 ; $90-$9F reserved (more statements later)
TOK_HEX         EQU $FD         ; [TOK_HEX][hi][lo] -- 16-bit hex constant
TOK_DEC         EQU $FE         ; [TOK_DEC][hi][lo] -- 16-bit decimal constant
TOK_REM_ROW     EQU 13          ; REM's KW_TAB row index (0-based) -- TOKENIZE_LINE
                                 ; needs this by row number, not by token value,

; ---- BEGIN keyword tokens (generated by tools/tokshowcase.py) ----
; token = TOK_STMT_BASE + KW_TAB row / TOK_FUNC_BASE + FUNC_TAB row.
; Row order = table order; run 'tokshowcase.py verify' after any table edit.
TOK_END         EQU $80         ; KW_TAB row 0
TOK_FOR         EQU $81         ; KW_TAB row 1
TOK_FREE        EQU $82         ; KW_TAB row 2
TOK_GOSUB       EQU $83         ; KW_TAB row 3
TOK_GOTO        EQU $84         ; KW_TAB row 4
TOK_IF          EQU $85         ; KW_TAB row 5
TOK_INPUT       EQU $86         ; KW_TAB row 6
TOK_LET         EQU $87         ; KW_TAB row 7
TOK_LIST        EQU $88         ; KW_TAB row 8
TOK_NEW         EQU $89         ; KW_TAB row 9
TOK_NEXT        EQU $8A         ; KW_TAB row 10
TOK_POKE        EQU $8B         ; KW_TAB row 11
TOK_PRINT       EQU $8C         ; KW_TAB row 12
TOK_REM         EQU $8D         ; KW_TAB row 13
TOK_RETURN      EQU $8E         ; KW_TAB row 14
TOK_RUN         EQU $8F         ; KW_TAB row 15
TOK_ABS         EQU $A0         ; FUNC_TAB row 0
TOK_AND         EQU $A1         ; FUNC_TAB row 1
TOK_NEG         EQU $A2         ; FUNC_TAB row 2
TOK_NOT         EQU $A3         ; FUNC_TAB row 3
TOK_OR          EQU $A4         ; FUNC_TAB row 4
TOK_PEEK        EQU $A5         ; FUNC_TAB row 5
TOK_RND         EQU $A6         ; FUNC_TAB row 6
TOK_USR         EQU $A7         ; FUNC_TAB row 7
TOK_XOR         EQU $A8         ; FUNC_TAB row 8
; ---- END keyword tokens ----

;  ERROR Defines
ERR_SYN         EQU '0'
; ERR_UND_LINE    EQU '1'         ; unused
ERR_DIV_ZERO    EQU '2'
ERR_OOM         EQU '3'
ERR_VAR         EQU '4'
ERR_RET         EQU '5'         ; RETURN without GOSUB (GOSUB stack underflow)
ERR_FOR         EQU '6'         ; Too many nested FORs (FORBASE stack overflow)
ERR_NXT         EQU '7'         ; NEXT without FOR (FORBASE stack underflow)
ERR_NEST        EQU '8'         ; Expression nesting too deep (RAS guard, v3.2 had '5')
ERR_OV          EQU '9'         ; ^ (power): negative exponent, or result overflows 16-bit

; RAS (hardware Return Address Stack) Defines
RAS_DEPTH       EQU 8           ; 2650 HW RAS depth (SPSU field is 3 bits, 0-7)
PE_RAS_LIMIT    EQU 5           ; PARSE_EXPR entry guard threshold; margin of
                                 ; RAS_DEPTH-3 covers PARSE_FACTOR+PARSE_S16+INC_IP,
                                 ; or (hex) PARSE_FACTOR+HEX_PEEK+TL_HEXVAL and
                                 ; PARSE_HEX(tail)+TL_MULADD+REG16_TO_REG16

; PSW Defines
PSW_RS          EQU     $10
PSW_WC          EQU     $08             ; WC (With Carry) bit in PSL (bit 3)
PSW_FLAG        EQU     $40

; REG16_TO_REG16 register indices (V4.25). 
; Usage: LODI,R0 (IDX_src*16)+IDX_dst  /  ZBSR *VREG16_TO_REG16  (dst = src)
IDX_IP    EQU 0
IDX_TMP   EQU 1
IDX_LNUM  EQU 3
IDX_EXP   EQU 4
IDX_SWSTK EQU 5
IDX_GOTO  EQU 6
IDX_CUR   EQU 7
IDX_SC0   EQU 8          ; SC0:SC1
IDX_PE    EQU 9
IDX_TK    EQU 10
IDX_ARGA  EQU 11         ; ARGAH:ARGAL
IDX_POW   EQU 12         ; POWCNTH:POWCNTL
IDX_SAVE  EQU 13         ; SAVEH:SAVEL
IDX_RND   EQU 14         ; RNDSEED

; System Defines
PROGLIM         EQU $1FFF   ; top of program store (numeric constant, not address)
;  GOSUB stack (v3.2) -- managed by SWSP
; Frame = [lo][hi] of NLP. SWSP=0=empty, counts up by 2/push (V4.19; was
; a $FF-sentinel "top index" scheme). 2 bytes/frame, 8 frames = 16 bytes.
GSSTKLIM        EQU $10    ; max SWSP before overflow (numeric constant, not address)
;  FOR/NEXT stack (v3.3) -- managed by FORSP
; Frame (7 bytes): [var][limH][limL][stpH][stpL][nlpH][nlpL]
;   var=VARS offset (0/2/4/../50, precomputed at push -- was the raw A-Z
;   letter pre-V4.19), lim=signed limit, stp=signed step, nlp=loop-back.
; FORSP=0=empty, counts up by 7/push (V4.19; was a $FF/$F9-sentinel
; scheme needing a local COM-mode toggle). Offsets: 0/7/14/21 for frames
; 1-4. 4 frames = 28 bytes.
; Overflow: FORSP >= FORSTKLIM before push -> ERR_FOR.
FORSTKLIM       EQU $1C   ; max FORSP before overflow (numeric constant, not address)

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
        DW INC_IP               ; 41 sites
VWSKIP:
        DW WSKIP                ; 28 sites
VINC_TMP:
        DW INC_TMP              ; 9 sites
VCOUT:
        DW COUT                 ; 15 sites
VPARSE_EXPR:
        DW PARSE_EXPR           ; 20 sites
VEATWORD:
        DW EATWORD              ; 9 sites
VPRT_SPACE:
        DW PRT_SPACE            ; 5 sites
VINC_EXP:
        DW INC_EXP              ; 4 sites
VCLR_EXP:
        DW CLR_EXP              ; 7 sites
VEAM_ATOM:
        DW EAM_ATOM             ; 9 sites
VEAM_HI:
        DW EAM_HI               ; 6 sites
VDO_ERROR:
        DW DO_ERROR             ; 4 sites
VJSYNERR:
        DW JSYNERR      ; 3 sites, multiple sites
VREG16_TO_REG16:
        DW REG16_TO_REG16       ; 29 sites (V4.25: generic packed-nibble 16-bit copy;
VFETCH16_IP:
        DW FETCH16_IP           ; 4 sites (V4.26: EXP = word at [IP], IP += 2)
VSET_TMP_PROG:
        DW SET_TMP_PROG ; 4 sites
VSET_TMP:
        DW SET_TMP      ; 5 sites
VFUNC_CONT:
        DW FUNC_EPILOG            ; 5 sites (ABS/NEG/RND + P2A_RET shared by
VPUSH_RET:
        DW PUSH_RET              ; 13 sites (v4.7 PUSHRET-01: replaces the
VCHECK_POW:
        DW CHECK_POW             ; 8 sites (v4.7 POW-01: power-operator (^)
VPUSH_EXP:
        DW PUSH_EXP              ; 4 sites, all BSTA -> ZBSR (v4.10 ZPVEC-01)
VPRINT_S16:
        DW PRINT_S16             ; 5 sites, all BSTA -> ZBSR (v4.10 ZPVEC-01)
VEXP_TO_LNUM:
        DW EXP_TO_LNUM  ; 3 sites, shim: LODI,R0 packed, falls into REG16_TO_REG16
VTMP_TO_EXP:
        DW TMP_TO_EXP   ; 3 sites, shim: LODI,R0 packed, falls into REG16_TO_REG16
VPARSER_RET:
        DW PARSER_RET   ; 4 sites
VDLS_BLPX:
        DW DLS_BLPX     ; 4 sites
VCMP_TMP_PE:
        DW CMP_TMP_PE   ; 4 sites
VDIGIT_CHECK:
        DW DIGIT_CHECK  ; 4 sites
VARG_OPEN:
        DW ARG_OPEN     ; 7 sites
VARG_CLOSE:
        DW ARG_CLOSE    ; 6 sites

; =============================================================================
MAIN:
        PPSL $02
       
        ; clear RUNFLG, SWSP, FORSP - change to DO_NEW for ROM
        BSTA,UN DO_END          

        ; clear A-Z variables (52 bytes)
        LODI,R3 52       ; BDRR decrements THEN branches if non-zero (Oracle v1.9): R3 = 52
        EORZ,R0          ; gives exactly 52 passes, indexes 52..1, so store at VARS-1,R3 to
                         ; cover VARS+51..VARS+0.  (Was LODI 51 / VARS,R3: wrote +51..+1 and
                         ; never cleared VARS+0, the hi byte of A, on real power-up RAM.)
        STRA,R0 GC_NEXT

CLRV:
        STRA,R0 VARS-1,R3
        BDRR,R3 CLRV

        ; Initialize RND seed
        LODI,R1 $AC
        LODI,R0 $E1
        BSTA,UN RND_SKIP

        ; print sign-on banner
        LODI,R0 <BANNER
        STRA,R0 IPH
        LODI,R0 >BANNER
        STRA,R0 IPL
        BSTA,UN PRTSTR
        BSTA,UN DO_FREE
        ; fall through to REPL

; =============================================================================
;  REPL -- Main read-eval-print loop
; In:  nothing
; Out: loops forever
; Clobbers: all
REPL:
        CPSL PSW_RS + 5             ; primary reg bank; clear PSL CC/flag bits.

        CPSU $07                    ; clear PSU SP field (bits 2:0 = HW RAS depth)
                                     ; MUST be separate from CPSL: SP is in PSU not PSL
        LODI,R0 '>'                    ; print prompt only used here
        ZBSR *VCOUT  
        ZBSR *VPRT_SPACE  
        BSTA,UN RDLINE                   ; also sets IPH:IPL = IBUF
        BSTA,UN TRY_STORE_LINE           ; CC=GT: line stored/deleted; CC=EQ: not a line
        BSTA,EQ STMT_EXEC               ; If CC=EQ (not a line), execute
        BCTR,UN REPL

; =============================================================================
;  CMP_LNUM_EXP -- signed 16-bit compare LNUMH:LNUML (left) : EXPH:EXPL (right)
;  Ported from uBASIC2650's CMP_OPS (v2.6).  The old IF code biased the high bytes
;  with XOR $80 and then used SUBA, whose CC comes from the signed 8-bit RESULT, so
;  operands whose biased high bytes differed by 128 or more (e.g. 20000 : -20000)
;  compared the wrong way round.  COMA with COM=0 on the high bytes is exact.
;  Also used by DO_NEXT (limit : variable).
; Out: CC = GT if LNUM > EXP, EQ if equal, LT if LNUM < EXP.  COM restored to 1.
; Clobbers: R0.  Leaf.
CMP_LNUM_EXP:
        LODA,R0 LNUMH
        CPSL $02                         ; COM=0: signed compare of the high bytes
        COMA,R0 EXPH
        PPSL $02                         ; COM=1 again (the global invariant); CC untouched
        RETC,GT
        RETC,LT
        LODA,R0 LNUML
        COMA,R0 EXPL                     ; low bytes: unsigned (COM=1)
        RETC,UN

; =============================================================================
;  DO_IF -- Conditional execution
; Syntax: IF expr relop expr THEN stmt
; In:  IP -> first char after IF keyword
; Out: executes stmt if condition true; otherwise sequential return
; Clobbers: R0, R1, EXPH, EXPL, LNUMH, LNUML, RELOP
; RAS: entry+1(PE)+1(PR)+1(PE)+1(SE) = entry+4. Max depth 7: ok.
DO_IF:
        ZBSR *VPARSE_EXPR                 ; [+1]
        ZBSR *VEXP_TO_LNUM ; LNUMH:LNUML = EXPH:EXPL (save left operand)
        BSTA,UN PARSE_RELOP              ; [+1]
        ZBSR *VPARSE_EXPR                 ; [+1]

        ; consume THEN keyword (touches IP and R0 only: LNUM/EXP stay intact for the compare)
        ZBSR *VWSKIP                      ; [+1] R0 = first non-space char (peek)
        COMI,R0 A'T'                      ; must be 'T'
        BCFR,EQ LSYNERR
        ZBSR *VINC_IP                     ; [+1] past the T (clobbers R0)
        LODA,R0 *IPH                      ; re-peek
        COMI,R0 A'H'                      ; must be 'H' (EATWORD below eats it)
        BCTR,EQ DIF_EW
LSYNERR:        
        ZBRR *VJSYNERR 

DIF_EW:
        ZBSR *VEATWORD                    ; [+1]
        ; signed compare LNUM (left) : EXP (right), then map the CC to a bitmask
        ; and AND it with RELOP (bit0=LT bit1=EQ bit2=GT)
        BSTR,UN CMP_LNUM_EXP             ; [+1] CC = GT/EQ/LT as left >/=/< right
        BCTR,EQ DIF_IS_EQ
        BCTR,LT DIF_IS_LT
        LODI,R0 4                        ; GT
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to next LODI
DIF_IS_LT:
        LODI,R0 1                        ; LT
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to next LODI
DIF_IS_EQ:
        LODI,R0 2                        ; EQ
        ANDA,R0 RELOP                    ; R0 &= RELOP (sets CC)
        RETC,EQ                          ; no match: condition false, sequential return
        ; drop through
; =============================================================================
;  STMT_EXEC -- Decode and dispatch one BASIC statement from IP.
; In:  IPH:IPL -> first char of statement (after any leading whitespace)
; RAS depth: 1 from REPL, 3 from DO_IF(THEN body).
STMT_EXEC:
        ZBSR *VWSKIP
        RETC,EQ                           ; blank line / empty THEN body (NUL): nothing to do
                                          ;   (uBASIC v2.15: no more spurious ?4)
        ; Look at statement KW_TAB with TMPH:TMPL as pointer
        LODI,R0 <KW_TAB
        LODI,R1 >KW_TAB
        ZBSR *VSET_TMP
        LODI,R1 >(KW_MISS-KW_TAB)         ; miss-row offset for MATCH_KW's early-out
        ; drop through

; =============================================================================
; MATCH_KW -- Match and Jump to handler based on KW
; KW_TAB/FUNC_TAB format (V4.21): [full keyword text][NUL][hi][lo], rows are
;   variable length (each row's own NUL marks its end). Terminator row is
;   just [NUL][hi][lo] (an empty "word") -- see SE_HI_LO. Matching now
;   requires the COMPLETE keyword spelling (was: first 2-3 chars); IF/OR
;   fall out naturally as 2-char rows, no wildcard byte needed any more.
; TMPH:TMPL selects the table (KW_TAB or FUNC_TAB) and stays fixed
;   throughout; R1 walks the table via indirect-indexed addressing
;   (*TMPH,R1) -- ported idea from uBASIC2650's MD_SCAN, adapted to stay
;   table-generic since this routine is shared by statement dispatch
;   (KW_TAB) and function dispatch (FUNC_TAB, 2 call sites).
; In:  IPH:IPL -> first char of statement (after any leading whitespace)
;      TMPH:TMPL -> table to search
;      R1 = offset of that table's miss row (KW_MISS-KW_TAB / FUNC_MISS-FUNC_TAB),
;      used only by the early-out for text that cannot be a word (see WORD_START)
; Out: handler called; IP advanced past the matched word/token (miss: IP
;      unchanged, see above)
; Clobbers: R0, R1, EXPH, EXPL, GOTOH, GOTOL
; =============================================================================
MATCH_KW:
        ZBSR *VWSKIP                      ; [+1]
        
        ; Golf #1: Pre-subtract the statement base. This collapses the token bounds 
        ; checks and completely eliminates the need to subtract it again in SF_STMT.
        SUBI,R0 TOK_STMT_BASE             ; R0 = token - $80
        BCTR,LT SE_SLOW                   ; < 0 (was < $80): not a token
        COMI,R0 TOK_HEX - TOK_STMT_BASE   ; >= $7D (was >=$FD)?
        BCFR,LT SE_SLOW                   ; constant token, let slow scan miss it
        
        COMI,R0 TOK_FUNC_BASE - TOK_STMT_BASE ; >= $20 (was >=$A0)?
        BCTR,LT SF_STMT                   ; < $20: Statement range
        
        SUBI,R0 TOK_FUNC_BASE - TOK_STMT_BASE ; Adjust index for Function table
        STRZ,R1
        LODA,R0 TOK_FUNC_OFS,R1           ; R0 = offset to func hi byte
        BCTR,UN SF_GOT_OFS
SF_STMT:
        STRZ,R1                           ; R0 is ALREADY the correct stmt index!
        LODA,R0 TOK_STMT_OFS,R1           ; R0 = offset to stmt hi byte
SF_GOT_OFS:
        ; Tail-merge into SE_HI_LO.
        SUBI,R0 1                         ; R1 = byte offset - 1
        STRZ,R1                           
        ZBSR *VINC_IP                     ; [+1] consume the 1-byte token
        BCTR,UN SE_HI_LO                  ; Dispatch via shared jump

SE_SLOW:
        BSTA,UN WORD_START                ; two letters? (EQ = maybe a word)
        BCFR,EQ SE_HI_LO                  ; no: cannot match any row -> miss row (R1 = its offset)
        LODI,R1 $FF                       ; pre-increment mode (2650 quirk)
SE_SCAN:
        LODI,R0 (IDX_IP*16)+IDX_EXP
        ZBSR *VREG16_TO_REG16             ; EXP = IP: save row-attempt's start IP
        LODA,R0 *TMPH,R1+                 ; row's first char
        BCTR,EQ SE_HI_LO                  ; NUL as first char: terminator row
SE_CHAR:
        ; Compares R0 (table char) to memory at *IPH (input char) without clobbering GOTOH.
        COMA,R0 *IPH                      
        BCFR,EQ SE_MISMATCH
        
        ZBSR *VINC_IP                     ; [+1] matched: consume input char
        LODA,R0 *TMPH,R1+                 ; next table char
        BCTR,EQ SE_MATCH                  ; row exhausted: full match
        BCTR,UN SE_CHAR
SE_MISMATCH:
        LODI,R0 (IDX_EXP*16)+IDX_IP
        ZBSR *VREG16_TO_REG16             ; IP = EXP: rewind IP
SE_SKIP:
        LODA,R0 *TMPH,R1+                 ; scan to this row's NUL terminator
        BCFR,EQ SE_SKIP
        ADDI,R1 2                         ; skip hi/lo -> next row's start
        BCTR,UN SE_SCAN
SE_MATCH:
        ZBSR *VEATWORD                    ; [+1] consume any trailing garbage
SE_HI_LO:
        LODA,R0 *TMPH,R1+                 ; handler hi
        STRA,R0 GOTOH                     ; store handler hi directly
        LODA,R0 *TMPH,R1+                 ; handler lo
        STRA,R0 GOTOL                     ; store handler lo directly
        BCTA,UN *GOTOH                    ; indirect jump

; =============================================================================
;  DO_NEW -- Clear program store
; Syntax: NEW
; In:  nothing
; Out: PEH:PEL = PROG (program store emptied); falls through to DO_END
; Clobbers: R0, TMPH, TMPL, SWSP, FORSP, RUNFLG
DO_NEW:
        ZBSR *VSET_TMP_PROG               ; TMP = PROG ...
        LODI,R0 (IDX_TMP*16)+IDX_PE
        ZBSR *VREG16_TO_REG16             ; ... PE = TMP (6 bytes vs 16 for direct stores)
        ; fall through to DO_END

; =============================================================================
;  DO_END -- Stop execution and clear all run state
; Syntax: END  (also called by DO_NEW, DO_ERROR, RESET)
; In:  nothing
; Out: SWSP=0, FORSP=0 (V4.19: 0-based), RUNFLG=0
; Clobbers: R0
DO_END:
        EORZ,R0                          ; V4.19: FORSP/SWSP are 0-based now
        STRA,R0 FORSP                    ; clear FOR stack
        STRA,R0 SWSP                     ; clear GOSUB stack
        STRA,R0 GC_NEXT
        LODI,R0 $FF                      ; FT_SP/FT_SAVE_SP: separate
        STRA,R0 FT_SP                    ; subsystem, still $FF=empty --
                                          ; FUNCATOM-01 (v4.6) dispatch-
                                          ; origin stack, untouched by V4.19
        STRA,R0 FT_SAVE_SP               ; ...and its byte-save stack too
        BCTA,UN CLR_RUNFLG              ; tail call (its EORZ,R0 supplies the zero)

; =============================================================================
;  SE_NOTKW -- Variable assignment: bare "V = expr" (reached when the statement is
;  not a keyword) and "LET V = expr" (the LET row's handler is this same code, as in
;  uBASIC2650 v2.7: LET costs nothing).
; Syntax: [LET] V = expr
; In:  IP -> variable letter (after LET's word, for LET)
; Out: VARS[V] = EXPH:EXPL
; Error: JERRVAR (not A-Z), JSYNERR (no '=')
; Clobbers: R0, R1, R2, SC0, SC1, EXPH, EXPL, TMPH, TMPL
; Note: DO_INPUT jumps to DL_STORE with R2 = VARS byte offset (see PARSE_VAR_SAVE).
SE_NOTKW:
        BSTA,UN PARSE_VAR_SAVE            ; validates A-Z, R2 = VARS byte offset
        ZBSR *VWSKIP
        COMI,R0 A'='
        BCFA,EQ JSYNERR
        ZBSR *VINC_IP
DL_EX:
        ZBSR *VPARSE_EXPR                 ; [+1]
;  DL_STORE -- store EXPH:EXPL into a variable (shared: SE_NOTKW = LET/bare assign,
;  DO_INPUT, DO_FOR)
; In:  R2 = VARS byte offset = 2*(letter-'A') (from PARSE_VAR_SAVE); EXPH:EXPL
; Out: VARS[R2..R2+1] = EXPH:EXPL; R2 unchanged
; Clobbers: R0
DL_STORE:
        LODA,R0 EXPH     ; R0 = High byte of expression
        STRA,R0 VARS,R2  ; Store directly to VARS array + offset (R2 is a valid index reg)
        LODA,R0 EXPL     ; R0 = Low byte of expression
        STRA,R0 VARS+1,R2; Store directly to VARS array + offset + 1
        ; fall through to DO_REM (RETC,UN)

; =============================================================================
;  DO_REM -- No-op / shared return
; Syntax: REM <anything>
; In:  nothing
; Out: nothing
; Clobbers: nothing
; PRTSTR_RET:
DO_REM:
        RETC,UN

; =============================================================================
;  DO_GOTO -- Computed GOTO
; Syntax: GOTO expr
; In:  IP -> first char after GOTO keyword
; Out: SWSTK = target record (FIND_LINE: the line itself, or the next higher one);
;      DR_LP's one path (TMP = SWSTK) then continues there.  This is uBASIC2650's
;      "SWSTK is the program counter" scheme: no GOTOFLG, no deferred FIND_LINE.
;      At the prompt it is harmless (SWSTK is rewritten by every RUN).
; Clobbers: R0, R1, EXPH, EXPL, LNUMH, LNUML, TMPH, TMPL, SWSTK
DO_GOTO:
        ZBSR *VWSKIP  
        ZBSR *VPARSE_EXPR                 ; [+1]
DG_GO:                                    ; DO_GOSUB joins here with EXP = target line
        ZBSR *VEXP_TO_LNUM
        ; V5.6: while running, SWSTK is the record after the current line.  A target line
        ; above the current one is therefore at or after SWSTK (records are sorted), so
        ; search from there instead of from PROG: forward jumps no longer walk the
        ; whole program.  Backward jumps and self-loops still search from the top.
        LODA,R0 RUNFLG
        BCTR,EQ DG_TOP                    ; at the prompt SWSTK is stale
        LODA,R0 LNUMH
        COMA,R0 CURH
        BCTR,GT DG_FWD                    ; target hi > current hi
        BCFR,EQ DG_TOP                    ; target hi < current hi
        LODA,R0 LNUML
        COMA,R0 CURL
        BCFR,GT DG_TOP                    ; target <= current line
DG_FWD:
        LODI,R0 (IDX_SWSTK*16)+IDX_TMP
        ZBSR *VREG16_TO_REG16             ; TMP = SWSTK
        BSTA,UN FIND_LINE_FWD             ; [+1] TMP = target record
        BCTA,UN DG_SET                    ; V5.7(PROTO): was BCTR -- cache block pushed DG_SET out of rel range
DG_TOP:
        ; V5.7(PROTO): 2-slot cache check, inline (no extra RAS level). On a
        ; hit, TMP is set directly and FIND_LINE is skipped entirely. On a
        ; miss, falls through to the original FIND_LINE call below and then
        ; remembers the result round-robin.
        LODI,R1 0
DGC_LP:
        COMI,R1 8
        BCTR,EQ DGC_MISS
        LODA,R0 GCACHE,R1
        COMA,R0 LNUMH
        BCFR,EQ DGC_NEXT
        LODA,R0 GCACHE+1,R1
        COMA,R0 LNUML
        BCFR,EQ DGC_NEXT
        LODA,R0 GCACHE+2,R1
        STRA,R0 TMPH
        LODA,R0 GCACHE+3,R1
        STRA,R0 TMPL
        BCTR,UN DG_SET
DGC_NEXT:
        ADDI,R1 4
        BCTR,UN DGC_LP
DGC_MISS:
        BSTA,UN FIND_LINE                 ; [+1] TMP = target record (unchanged)
        LODA,R1 GC_NEXT
        LODA,R0 LNUMH
        STRA,R0 GCACHE,R1
        LODA,R0 LNUML
        STRA,R0 GCACHE+1,R1
        LODA,R0 TMPH
        STRA,R0 GCACHE+2,R1
        LODA,R0 TMPL
        STRA,R0 GCACHE+3,R1
        ; GC_NEXT only ever holds 0 or 4 (DO_RUN resets it to 0), so "next slot, wrapping"
        ; is a toggle.  R1 still holds GC_NEXT (loaded above; the indexed stores do not
        ; modify it) so no reload.  Was LODA/ADDI/COMI/BCFR/BCTR/EORZ: -9 bytes, no branch.
        LODZ,R1                           ; R0 = old GC_NEXT
        EORI,R0 4                         ; 0 <-> 4
        STRA,R0 GC_NEXT
DG_SET:
        LODI,R0 (IDX_TMP*16)+IDX_SWSTK
        ZBRR *VREG16_TO_REG16             ; SWSTK = TMP (tail call)

; =============================================================================
;  DO_RETURN -- Return from subroutine
; Syntax: RETURN
; In:  SWSP = GOSUB stack bytes used; top frame = GSBASE[SWSP-2]=hi, [SWSP-1]=lo of NLP.
; Out: SWSTK = popped NLP (a program-store address; DR_LP continues from SWSTK).
; Clobbers: R0, R1, SWSTK, SWSP.
; Error: SWSP=0 (underflow) -> ERR_RET.
; V4.19: SWSP is 0-based (0=empty).  Pre-decrement loads: lo first, then hi.
DO_RETURN:
        LODA,R1 SWSP
        BCTA,EQ DRT_UNDERFLOW   ; SWSP==0: nothing pushed
        LODA,R0 GSBASE,R1-      ; pre-decrement loads: lo, then hi
        STRA,R0 SWSTK+1
        LODA,R0 GSBASE,R1-
        STRA,R0 SWSTK
        STRA,R1 SWSP            ; SWSP -= 2
        RETC,UN

; =============================================================================
;  SET_TMP_PROG -- Set TMPH:TMPL = PROG base address (falls into SET_TMP)
;  SET_TMP -- Set TMPH:TMPL = R0:R1.  Shared by the five "TMP = table" sites
;  (KW_TAB x2, FUNC_TAB x3: 10 bytes each inline, 6 now) and SET_TMP_PROG.
; Clobbers: R0, R1
SET_TMP_PROG:
        LODI,R0 <PROG
        LODI,R1 >PROG
SET_TMP:
        STRA,R0 TMPH
        STRA,R1 TMPL
        RETC,UN

; =============================================================================
; RND_SHUFFLE  Advance 16-bit Galois LFSR (Little-Endian)
; =============================================================================
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
        LODA,R0 RNDSEED         ; Load seed high byte
        LODA,R1 RNDSEED+1       ; Load seed low byte
        CPSL    1               ; Clear Carry (C=0 shifts into bit7 of R0)
        PPSL    PSW_WC          ; Enable WC: RRR now chains carry between regs
        RRR,R0                  ; Shift R0 right: bit0 of R0 -> Carry; 0 -> bit7
        RRR,R1                  ; Shift R1 right: Carry (bit0 of R0) -> bit7 of R1
                                 ;                 bit0 of R1 -> Carry (feedback)
        CPSL    PSW_WC          ; Disable WC (restore normal mode)
        TPSL    1               ; Test Carry: CC=EQ if C=1, CC=LT if C=0
        BCTR,LT RND_SKIP        ; C=0 (CC=LT): feedback bit was 0, skip XOR
        EORI,R0 $B4             ; Apply taps high byte (0xB400)
RND_SKIP:
        STRA,R0 RNDSEED        ; Save seed high byte
        STRA,R1 RNDSEED+1      ; Save seed low byte
        RETC,UN

; =============================================================================
; DO_RND_FUNC  RND(n) -> pseudo-random value in [0,n)
; BUG-RND-01 fix: a single LFSR shift/call left consecutive RND() draws in a
; tight loop highly correlated (only 1 bit of mixing between draws).
; =============================================================================
DO_RND_FUNC:
        ZBSR *VARG_OPEN
        ZBSR *VPARSE_EXPR       ; get range in EXP 
        ZBSR *VARG_CLOSE
        LODI,R2 8               ; shuffle a full byte's worth of taps
RNDF_MIX:
        BSTR,UN RND_SHUFFLE
        BDRR,R2 RNDF_MIX

        LODI,R0 (IDX_RND*16)+IDX_TMP
        ZBSR *VREG16_TO_REG16   ; TMP = RNDSEED
        BSTA,UN DIV16          ; Divide
        ZBSR *VTMP_TO_EXP ; EXP = remainder 
        ZBRR *VFUNC_CONT

; =============================================================================
;  DO_PEEK_FUNC / DO_USR_FUNC -- relocated here in v4.5 (see header note at
;  FUNCCONT-01 fix.
DO_PEEK_FUNC:
        ZBSR *VARG_OPEN
        ZBSR *VPARSE_EXPR
        ZBSR *VARG_CLOSE
        LODA,R0 *EXPH
        BCTR,UN EXPH_Z                   ; clear top byte

; =============================================================================
; Calling function retval in R0
DO_USR_FUNC:
        ZBSR *VARG_OPEN
        ZBSR *VPARSE_EXPR
        ZBSR *VARG_CLOSE
        BSTA,UN *EXPH   
EXPH_Z:        
        STRA,R0 EXPL
        EORZ,R0          ; clear top byte
        STRA,R0 EXPH
        ZBRR *VFUNC_CONT

; =============================================================================
; PARSE_VAR_SAVE -- skip whitespace, read var letter, range-check,
;                   save its VARS byte offset in R2, advance IP.
; Out: R2 = 2*(letter-'A') = byte offset into VARS (A-Z); IP advanced past letter
; Error: tail-jumps to JERRVAR (no return)
; Clobbers: R0, R2.  RAS: +2 (WSKIP's INC_IP nests inside; INC_IP is tail-called)
; Range test relies on the global COM=1 (unsigned) set in MAIN: any char below
; 'A' wraps to a large unsigned value, so one compare catches both ends.
PARSE_VAR_SAVE:
        ZBSR *VWSKIP  
        SUBI,R0 A'A'                     ; 'A'-'Z' -> 0-25
        COMI,R0 A'Z'-A'A'
        BCTA,GT JERRVAR                  ; out of range (either end) -- tail jump, no return
        ADDZ,R0                          ; R0 = index*2 (16-bit word stride)
        STRZ,R2                          ; R2 = byte offset, consumed by DL_STORE
        ZBRR *VINC_IP           ; tail call  

; =============================================================================
;  DO_INPUT -- Read signed integer from user into variable
; Syntax: INPUT V
; In:  IP -> variable letter
; Out: VARS[V] = parsed value
; Clobbers: R0, R2, SC0, SC1, EXPH, EXPL, TMPH, TMPL, IBUF
DO_INPUT:
        BSTR,UN PARSE_VAR_SAVE
        BSTA,UN PRT_QUEST
        ZBSR *VPRT_SPACE  
        BSTA,UN RDLINE                   ; [+1] also sets IPH:IPL = IBUF
        BSTA,UN PARSE_S16                ; [+1]
        BCTA,UN DL_STORE

; =============================================================================
; Character IO
; 110 Baud teletype from PIPBUG V1 as per Signetics M20 application note.
; v4.11: CHIN/COUT no longer pinned to PIPBUG V1's fixed $286/$2B4 (target
; is standalone, no PIPBUG ROM present; pipbug_wrap already takes --chin/
; --cout addresses from the LST after each build, so nothing depends on
; these being fixed). Floating frees the slack this boundary kept eating
; every time code ahead of it grew (see v4.10 ZPVEC-01 note below).
PRE_CHIN:
        BSTA,UN RND_SHUFFLE
CHIN:
        PPSL PSW_RS
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
;  DO_PRINT / PRTSTR -- Print statement and NUL-terminated string helper
; Syntax: PRINT [item {; item}]
;   item = "string" | expr | TAB(n) | CHR$(n) | HEX$(n)
;   Trailing ; suppresses newline. HEX$: see DP_NOTC (post-COUT zone).
; In:  IP -> first char after PRINT keyword
; Out: text written to COUT; IP advanced past statement
; Clobbers: R0, R1, EXPH, EXPL, TMPH, TMPL, NEGFLG, LNUMH, LNUML, SC0, SC1
DP_STRING:
        ZBSR *VINC_IP            ; skip opening quote
PRTSTR:
        LODA,R0 *IPH
        RETC,EQ                  ; NUL before closing ": bail
        COMI,R0 DQ
        BCTR,EQ DP_SCLS
        ZBSR *VCOUT  
        BCTR,UN DP_STRING        ; Loops back to VINC_IP above!

DO_PRINT:
        ZBSR *VWSKIP  
        BCTA,EQ PRT_CRLF         ; empty print -> tail call to newline

DP_ITEM:
        LODI,R1 0                ; initialize lookahead index
        COMI,R0 DQ
        BCTR,EQ DP_STRING

        COMI,R0 'C'
        BCTR,EQ CK_CH
        COMI,R0 'H'
        BCTR,EQ CK_HE
        COMI,R0 'T'
        BCTR,EQ CK_TA

DP_EXPR:
        ; IP was untouched, VPARSE_EXPR starts right at the expression.
        ZBSR *VPARSE_EXPR  
        ZBSR *VPRINT_S16
        BCTA,UN DP_SEP

CK_CH:
        LODA,R0 *IPH,R1+         ; R0 = 2nd char, R1 = 2
        COMI,R0 'H'
        BCFR,EQ DP_EXPR          ; Not "CH" -> must be expression
DP_CHAR:
        ZBSR *VEATWORD           ; Consumes "CHR$" starting from current IP
        ZBSR *VPARSE_EXPR  
        LODA,R0 EXPL
        ZBSR *VCOUT  
        DB $EC  ; swallow next 2 bytes
DP_SCLS:
        ZBSR *VINC_IP  
        BCTR,UN DP_SEP

CK_HE:
        LODA,R0 *IPH,R1+
        COMI,R0 'E'
        BCFR,EQ DP_EXPR
DP_HEXITEM:
        ZBSR *VEATWORD           ; Consumes "HEX$" 
        ZBSR *VPARSE_EXPR  
        LODI,R0 '$'              ; print dollar
        ZBSR *VCOUT  
        LODA,R0 EXPH
        BSTR,UN PRINT_HEX_BYTE
        LODA,R0 EXPL
        BSTR,UN PRINT_HEX_BYTE
        BCTR,UN DP_SEP

CK_TA:
        LODA,R0 *IPH,R1+
        COMI,R0 'A'
        BCFR,EQ DP_EXPR
DP_TAB:
        ZBSR *VEATWORD           ; Consumes "TAB"
        ZBSR *VPARSE_EXPR  
        LODA,R1 EXPL
        BCTR,EQ DP_SEP           ; TAB(0): skip
TAB_LOOP:
        ZBSR *VPRT_SPACE  
        BDRR,R1 TAB_LOOP
        ; fall through to DP_SEP

DP_SEP:
        ZBSR *VWSKIP  
        COMI,R0 $3B              ; semicolon?
        BCFA,EQ PRT_CRLF         ; If not, tail call to newline directly

        ZBSR *VINC_IP            ; Eat semicolon
        ZBSR *VWSKIP  
        RETC,EQ                  ; bail if NUL
        BCTA,UN DP_ITEM

; =============================================================================
;  PRINT_HEX_BYTE -- HEX$(n) support
; =============================================================================
PRINT_HEX_BYTE:
        STRZ,R1                  ; R1 = R0 (save original byte)
        RRR,R0
        RRR,R0
        RRR,R0
        RRR,R0                   ; Nibble-swap: hi nibble to low 4 bits
        BSTR,UN PHB_NIB          ; Print high nibble
        LODZ,R1                  ; Restore R0 for low nibble
        ; fall through for low nibble

PHB_NIB:
        ANDI,R0 $0F
        ADDI,R0 $30
        COMI,R0 $3A
        BCTR,LT PHB_OUT
        ADDI,R0 7
PHB_OUT:
        ZBRR *VCOUT              ; Tail call to VCOUT (returns to caller)

; =============================================================================
;  DO_GOSUB -- Subroutine call
; Syntax: GOSUB <line>
; In:  IP -> line number; SWSTK[0:1] = NLP from DR_EXEC; SWSP = stack ptr.
; Out: NLP pushed onto GSBASE, then as DO_GOTO (SWSTK = target record).
; Clobbers: as DO_GOTO, plus SWSP
; Stack: GSBASE[SWSP]=hi, GSBASE[SWSP+1]=lo. SWSP=bytes used, 0=empty.
DO_GOSUB:
        ZBSR *VWSKIP                      ; [+1]
        ZBSR *VPARSE_EXPR                 ; [+1] target line -> EXPH:EXPL
        ; overflow check: SWSP is 0-based (0=empty), so R1 (pre-push SWSP)
        ; doubles as the store index -- no first-push special case needed.
        LODA,R1 SWSP
        COMI,R1 GSSTKLIM
        BCTR,LT DGS_STORE
        LODI,R0 ERR_OOM
        ZBRR *VDO_ERROR 
DGS_STORE:
        LODA,R0 SWSTK                    ; NLP hi byte
        STRA,R0 GSBASE-1,R1+             ; pre-increment store: GSBASE[SWSP++] = hi
        LODA,R0 SWSTK+1                  ; NLP lo byte
        STRA,R0 GSBASE-1,R1+             ; GSBASE[SWSP++] = lo (DO_RETURN pops lo first)
        STRA,R1 SWSP                     ; R1 = SWSP+2 = new SWSP
        BCTA,UN DG_GO                    ; then exactly a GOTO

; =============================================================================
;  DO_FOR -- FOR loop setup
; Syntax: FOR V = start TO limit [STEP step]
; In:  IP past "FOR" keyword; SWSTK[0:1] = NLP of line after FOR (set by DR_EXEC).
; Out: Frame pushed on FORBASE; var initialised to start; falls through to body.
; Clobbers: R0, R1, R2, FORVAR, FORSP, LNUMH, LNUML, EXPH, EXPL, TMPH, TMPL.
; Errors: stack full -> ERR_FOR.
; Frame layout (7 bytes at FORBASE[FORSP]): [nlpL][nlpH][stpL][stpH][limL][limH][var]
; Loop body always executes at least once; exit tested at NEXT.
DO_FOR:
        ; --- get variable: validates A-Z, R2 = VARS byte offset (survives PARSE_EXPR) ---
        BSTA,UN PARSE_VAR_SAVE            ; [+1] IP past the letter

        ; --- stack overflow check (FORSP is 0-based, 0=empty; V4.19) ---
        LODA,R0 FORSP
        COMI,R0 FORSTKLIM
        BCFA,LT JFORERR                  ; If NOT Less-Than limit (>=), fail!

        ; --- skip '=' then parse start value -> EXPH:EXPL ---
        ZBSR *VWSKIP                      ; [+1] skip whitespace before '='
        ZBSR *VINC_IP                     ; [+1] consume '='
        ZBSR *VPARSE_EXPR                 ; [+1] start value -> EXPH:EXPL
        BSTA,UN DL_STORE                 ; [+1] VARS[R2] = EXPH:EXPL
        STRA,R2 FORVAR                   ; cache the VARS offset for the frame
        ; --- consume "TO" keyword ---
        ZBSR *VWSKIP                      ; [+1]
        ZBSR *VEATWORD                    ; [+1]
        ; --- parse limit -> LNUMH:LNUML ---
        ZBSR *VPARSE_EXPR                 ; [+1]
        ZBSR *VEXP_TO_LNUM ; LNUMH:LNUML = EXPH:EXPL (limit)
        ; --- check for STEP keyword ---
        ;   If 'S': EATWORD consumes all of "STEP", then parse step.
        ;   Else: use default step = +1.
        ZBSR *VWSKIP                      ; [+1] R0 = first non-space char (peek)
        COMI,R0 A'S'
        BCTR,EQ DF_STEP
        ; not 'S': use step=+1
        ZBSR *VCLR_EXP                          ; step hi = 0
        ZBSR *VINC_EXP                          ; step lo = 1
        BCTR,UN DF_PUSH
DF_STEP:
        ZBSR *VEATWORD                    ; [+1] consume "TEP"
        ZBSR *VPARSE_EXPR                 ; [+1] step -> EXPH:EXPL
DF_PUSH:
        ; Push the 7-byte frame at FORBASE[FORSP] (FORSP = bytes used, 0-based).
        ; RAM FORVAR..SWSTK+1 is read top-down and pushed with pre-increment stores, so
        ; FORSP is the next free offset (FORBASE-1,R1+ idiom, as uBASIC's DF_PUSH).
        ; (The old scheme bumped FORSP first and then wrote at FORBASE+FORSP: the 4th
        ; frame ran 7 bytes past FORBASE into IBUF, so INPUT in a 4-deep nest
        ; corrupted the loop.)
        ; Frame: [nlpL][nlpH][stpL][stpH][limL][limH][var]
        LODA,R1 FORSP
        LODI,R2 7
DF_LOOP:
        LODA,R0 FORVAR,R2-               ; pre-decrement: FORVAR+6 (SWSTK+1) ... FORVAR+0
        STRA,R0 FORBASE-1,R1+            ; pre-increment: FORBASE[FORSP++]
        BRNR,R2 DF_LOOP
        STRA,R1 FORSP                    ; R1 = old FORSP + 7
        RETC,UN

; =============================================================================
;  DO_NEXT -- FOR loop iteration (restructured after uBASIC2650 v2.13)
; Syntax: NEXT [V]
; In:  FORSP = FOR stack bytes used (0 = empty); top frame at FORBASE[FORSP-7 ..].
;      Frame: [nlpL][nlpH][stpL][stpH][limL][limH][var]; var = VARS byte offset.
; Out: Looping: SWSTK = the loop body (DR_LP continues there).
;      Done: frame popped (FORSP -= 7); sequential return.  A var+step that overflows
;      16 bits ends the loop (OVF test), so ranges up to +/-32767 terminate exactly.
; Method: pop var/limit/step back into FORVAR/LNUM/EXP (5-byte loop), new var =
;      var + step via ADD_CORE + DL_STORE, then one signed compare limit : var
;      (CMP_LNUM_EXP).  EQ -> loop again (body runs at the limit value).  Otherwise
;      the LT bit of that compare XOR the step's sign decides: they agree ->
;      still inside the range -> loop; else exit.
; Clobbers: R0, R1, R2, R3, FORVAR, EXPH, EXPL, LNUMH, LNUML, SWSTK, FORSP.
; Errors: FORSP=0 -> ERR_NXT.
; The variable name after NEXT is consumed but not checked (smallest code).
; =============================================================================
DO_NEXT:
        LODA,R3 FORSP
        BCTA,EQ JERR_NXT                  ; FORSP==0: no FOR active
        ZBSR *VWSKIP                      ; [+1]
        ZBSR *VEATWORD                    ; [+1] consume optional var name
        SUBI,R3 7                         ; R3 = frame base index
        LODI,R2 5
DN_POP:                                   ; frame[2..6] -> EXPL, EXPH, LNUML, LNUMH, FORVAR
        LODA,R0 FORBASE+1,R3+             ; pre-increment: frame[2] first
        STRA,R0 FORVAR,R2-                ; pre-decrement: FORVAR+4 (EXPL) down to FORVAR+0
        BRNR,R2 DN_POP
        LODA,R2 FORVAR                    ; R2 = VARS byte offset
        SUBI,R3 5                         ; R3 = frame base again
        LODI,R0 VARS-IPH
        ADDZ,R2
        BSTA,UN ADD_CORE                  ; [+1] EXP = step + var = the new value
        TPSL $04                          ; OVF: var+step overflowed 16 bits, so the loop is
        BCTR,EQ DN_DONE                   ;   over (the wrapped value must not be compared)
        BSTA,UN DL_STORE                  ; VARS[R2] = EXP
        LODA,R0 FORBASE+3,R3              ; step hi byte: its bit 7 is the step's sign
        STRZ,R2
        BSTA,UN CMP_LNUM_EXP              ; [+1] CC = limit : var (signed)
        BCTR,EQ DN_GO                     ; var = limit: one more pass
        SPSL                              ; R0 bit 7 = CC1 = (limit < var)
        EORZ,R2                           ; ^ step sign: bit 7 clear = agree = still in range
        BCFR,LT DN_GO
DN_DONE:
        STRA,R3 FORSP                     ; done: pop the frame (FORSP = its base)
        RETC,UN
DN_GO:
        LODI,R2 2                         ; loop body: frame nlp -> SWSTK (hi, lo)
DN_BODY:
        LODA,R0 FORBASE-1,R3+
        STRA,R0 SWSTK,R2-
        BRNR,R2 DN_BODY
        RETC,UN

; =============================================================================
;  DO_RUN -- Execute stored program
; Syntax: RUN
; In:  PROG=program base, PEH:PEL=program end
; Out: runs until END, error, or exhausted; returns to REPL
; Clobbers: all
; After STMT_EXEC, SWSTK always holds the next record to run: the following line
; (set by DR_EXEC) unless GOTO/GOSUB/RETURN/NEXT rewrote it.
DO_RUN:
        LODI,R0 1
        STRA,R0 RUNFLG
        LODI,R0 $FF              ; V5.7: invalidate GOTO-cache (sentinel;
        STRA,R0 GCACHE           ;   no real line number is $FFFF)
        STRA,R0 GCACHE+1
        STRA,R0 GCACHE+4
        STRA,R0 GCACHE+5
        EORZ,R0
        STRA,R0 GC_NEXT
        ZBSR *VSET_TMP_PROG
DR_LP:
        LODA,R0 RUNFLG
        RETC,EQ
        ; end of program? TMPH:TMPL >= PEH:PEL
        ZBSR *VCMP_TMP_PE
        BCFR,LT DR_STOP                  ; TMP >= PE (GT or EQ): end of program, clear RUNFLG
DR_EXEC:
        ; save current line number for error reporting (TMP unchanged)
        ; IP = TMP (record start); FETCH16_IP reads the 2-byte line-number header
        ; into EXP and leaves IP at the length byte (V5.7: length-prefixed
        ; records -- one more INC_IP lands IP at the body).
        ; CUR = line number for error reporting (EXP is free here:
        ; every statement re-parses before it reads EXP).
        ; V5.7: these 4 pairs are the hottest REG16_TO_REG16 call sites in the
        ; interpreter (every statement, unconditionally) -- inlined directly
        ; (+8 bytes/site) instead of through the generic bank-switching helper.
        LODA,R0 TMPH                      ; IP = TMP
        STRA,R0 IPH
        LODA,R0 TMPL
        STRA,R0 IPL
        ZBSR *VFETCH16_IP                 ; [+2]
        ZBSR *VINC_IP                     ; [+1] past the length byte (not read here)
        LODA,R0 EXPH                      ; CUR = EXP
        STRA,R0 CURH
        LODA,R0 EXPL
        STRA,R0 CURL
        ; advance TMP past this whole record to the next one, for SWSTK
        BSTA,UN ADV_TMP_PAST_REC           ; [+1]
        ; Save next-line pointer into SWSTK before STMT_EXEC clobbers SC0/SC1.
        ; SWSTK persists across STMT_EXEC; DO_GOSUB and DO_FOR read from it, and
        ; GOTO/GOSUB/RETURN/NEXT write the new target into it.
        LODA,R0 TMPH                      ; SWSTK = TMP
        STRA,R0 SWSTK
        LODA,R0 TMPL
        STRA,R0 SWSTK+1
        ; execute line
        BSTA,UN STMT_EXEC                ; [+1]
        LODA,R0 SWSTK                      ; TMP = SWSTK (next-record pointer)
        STRA,R0 TMPH
        LODA,R0 SWSTK+1
        STRA,R0 TMPL
        BCTA,UN DR_LP                     ; V5.7: was BCTR (inlined DR_EXEC pushed it to -68)
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
;  TOK_MATCH_ROW -- match the word at IP against a KW_TAB/FUNC_TAB-shaped
;  table, requiring the full spelling (this is MATCH_KW's V4.21 text scan,
;  reused as-is except for what happens on a match: TOKENIZE_LINE needs the
;  row INDEX, not a jump to that row's handler). Shared between KW_TAB and
;  FUNC_TAB attempts by TOKENIZE_LINE, same as MATCH_KW itself is shared.
; In:  IPH:IPL -> first letter; TKH:TKL -> table to search
; Out: matched: R2 = row index (0-based); IP advanced past the matched word
;      (trailing garbage swallowed by EATWORD, same as MATCH_KW)
;      no match: R2 = $FF; IP unchanged
; Clobbers: R0, R1, R2, EXPH, EXPL, GOTOH
TOK_MATCH_ROW:
        LODI,R1 $FF                       ; pre-increment mode
        LODI,R2 0                         ; row counter
TMR_SCAN:
        LODI,R0 (IDX_IP*16)+IDX_EXP
        ZBSR *VREG16_TO_REG16             ; EXP = IP: save this row-attempt's start IP
        LODA,R0 *TKH,R1+                 ; row's first char
        BCTR,EQ TMR_MISS                  ; NUL as first char: terminator row
TMR_CHAR:
        STRA,R0 GOTOH                     ; stash table char (temp scratch)
        LODA,R0 *IPH                      ; peek input char, not consumed yet
        SUBA,R0 GOTOH
        BCFR,EQ TMR_MISMATCH
        ZBSR *VINC_IP                     ; matched: consume this input char
        LODA,R0 *TKH,R1+                 ; next table char
        BCTA,EQ *VEATWORD                 ; row exhausted: full match
        BCTR,UN TMR_CHAR
TMR_MISMATCH:
        LODI,R0 (IDX_EXP*16)+IDX_IP
        ZBSR *VREG16_TO_REG16             ; IP = EXP: rewind IP to this row's start
TMR_SKIP:
        LODA,R0 *TKH,R1+                 ; scan to this row's NUL terminator
        BCFR,EQ TMR_SKIP
        ADDI,R1 2                         ; skip hi/lo -> next row's start
        ADDI,R2 1                         ; next row's index
        BCTR,UN TMR_SCAN
TMR_MISS:
        LODI,R2 $FF
        RETC,UN

; =============================================================================
;  TL_HEXVAL -- test/convert one uppercase hex digit character
; In:  R0 = character to test
; Out: is a hex digit ('0'-'9' or 'A'-'F'): R0 = value (0-15), R1 = 0
;      not a hex digit: R1 = $FF (R0 clobbered)
; Clobbers: R0, R1   (NOT R2: it is PARSE_EXPR's live variable letter, and
;   this is now also reached from PARSE_FACTOR via HEX_PEEK/PARSE_HEX)
TL_HEXVAL:
        STRZ,R1                           ; R1 = original char (temp use)
        SUBI,R0 A'0'
        COMI,R0 9
        BCTR,GT TL_HV_ALPHA                ; >9: not 0-9, check A-F
        LODI,R1 0                          ; matched: R0 already = value (0-9)
        RETC,UN
TL_HV_ALPHA:
        LODZ,R1                            ; R0 = original char again
        SUBI,R0 A'A'
        COMI,R0 5
        BCTR,GT TL_HV_NONE                  ; not 'A'-'F' either
        ADDI,R0 10                          ; rebase 0-5 -> 10-15
        LODI,R1 0
        RETC,UN
TL_HV_NONE:
        LODI,R1 $FF
        RETC,UN

; =============================================================================
;  TOKENIZE_LINE -- convert a typed statement body into tokenized form
; V4.22. Called only from TRY_STORE_LINE (a numbered/stored line), never for
;   direct/immediate entry -- this is what keeps the raw-text path fully
;   intact for immediate statements (STMT_EXEC's slow scan still handles
;   them exactly as before; they're just never routed through here).
; Single left-to-right pass. At any position outside a string or a REM
;   body: a run of A-Z is tried against KW_TAB then FUNC_TAB (full spelling
;   required, exactly as MATCH_KW's V4.21 scan); no valid identifier in
;   this language is ever more than one letter, so trying both tables at
;   every letter position can't misfire on well-formed input -- a single
;   variable letter is what's left when neither table matches. A digit run
;   becomes TOK_DEC + its 16-bit magnitude; '$' followed by a hex digit
;   becomes TOK_HEX + its magnitude (a bare '$' with no hex digit after it
;   is copied through as an ordinary byte, e.g. for HEX$/CHR$, which are
;   not tokenized -- Phase 2, deferred). A leading '-' is NOT folded into
;   either constant form: it's a separate unary operator (EAM_NEG) at
;   runtime and stays a plain byte here too, for both typed and tokenized
;   atoms alike. String literals and REM's comment body are copied through
;   completely verbatim -- no keyword/number recognition inside either.
; In:  IPH:IPL -> first char of the typed body (already past the line
;      number and following space)
; Out: TOKBUF = tokenized body, NUL-terminated; R3 = its length not
;      counting the NUL (TRY_STORE_LINE adds +4 for header+len+NUL itself)
; Clobbers: R0, R1, R2, R3, IPH, IPL, TKH, TKL, EXPH, EXPL, GOTOH, SC0,
;   R3SAVE (via the digit/hex accumulation -- same cell PARSE_S16 uses,
;   never live across a TOKENIZE_LINE call since it only runs at store
;   time, never from within expression parsing)
; Preserves TMPH:TMPL (V4.24): TRY_STORE_LINE holds its insertion point there
;   across this call. V4.23 fixed a hang caused by this routine using TMP as
;   its table pointer; V4.24 gave it its own TKH:TKL pair instead of having
;   the caller save/restore TMP (-22 bytes).
TOKENIZE_LINE:
        LODI,R3 $FF                       ; TOKBUF write index (pre-increment)
TL_LP:
        LODA,R0 *IPH
        BCTA,EQ TL_DONE                   ; NUL: end of typed body
        COMI,R0 DQ                        ; string literal?
        BCTR,EQ TL_STRING
        COMI,R0 A'A'
        BCTR,LT TL_NOTLETTER               
        COMI,R0 A'Z'+1
        BCTR,LT TL_WORD                   ; 'A'-'Z'
TL_NOTLETTER:
        COMI,R0 A'0'
        BCTR,LT TL_NOTDIGIT                
        COMI,R0 A'9'+1
        BCTA,LT TL_DECIMAL                 ; '0'-'9'
TL_NOTDIGIT:
        COMI,R0 '$'
        BCTA,EQ TL_MAYBEHEX
        ; ordinary byte (operator, punctuation, space, etc.): copy verbatim
TL_COPY1:                                  ; shared tail: R0 already loaded
        STRA,R0 TOKBUF,R3+                 ;   with the byte at IP -- also
        ZBSR *VINC_IP                      ;   used by TL_NOWORD/TL_DOLLAR_
        BCTR,UN TL_LP                      ;   PLAIN, same shape otherwise
TL_STRING:
        STRA,R0 TOKBUF,R3+                 ; copy the opening quote
        ZBSR *VINC_IP
TL_STR_LP:
        LODA,R0 *IPH
        BCTA,EQ TL_DONE                    ; unterminated string: same
                                          ;   tolerance as PRTSTR at runtime
        STRA,R0 TOKBUF,R3+
        ZBSR *VINC_IP                     
        LODA,R0 TOKBUF,R3
        COMI,R0 DQ
        BCFR,EQ TL_STR_LP                  ; keep going until the closing quote
        BCTR,UN TL_LP                      
                                          ;   SC0-cache fix grew this block
TL_WORD:
        LODI,R0 <KW_TAB
        STRA,R0 TKH
        LODI,R0 >KW_TAB
        STRA,R0 TKL
        BSTA,UN TOK_MATCH_ROW
        COMI,R2 $FF
        BCTR,EQ TL_TRYFUNC
        LODZ,R2                            ; R0 = row index
        ADDI,R0 TOK_STMT_BASE
        STRA,R0 TOKBUF,R3+
        COMI,R2 TOK_REM_ROW
        BCTR,EQ TL_REM
        BCTA,UN TL_LP
TL_TRYFUNC:
        LODI,R0 <FUNC_TAB
        STRA,R0 TKH
        LODI,R0 >FUNC_TAB
        STRA,R0 TKL
        BSTA,UN TOK_MATCH_ROW
        COMI,R2 $FF
        BCTR,EQ TL_NOWORD
        LODZ,R2
        ADDI,R0 TOK_FUNC_BASE
        STRA,R0 TOKBUF,R3+
        BCTA,UN TL_LP
TL_NOWORD:
        ; no keyword or function matched: this is a variable reference,
        ; always exactly one letter
        LODA,R0 *IPH
        BCTA,UN TL_COPY1
TL_REM:
        ; REM: the rest of the line is an opaque comment, copied verbatim
        LODA,R0 *IPH
        BCTA,EQ TL_DONE
        STRA,R0 TOKBUF,R3+
        ZBSR *VINC_IP
        BCTR,UN TL_REM
TL_DECIMAL:
        ZBSR *VCLR_EXP
TL_DEC_LP:
        ZBSR *VDIGIT_CHECK
        BCTR,GT TL_DEC_DONE                 ; non-digit: stop accumulating
        STRA,R0 SC0                          ; this digit's value (0-9)
        ZBSR *VINC_IP
        LODI,R0 10
        BSTA,UN TL_MULADD                    ; EXP = EXP*10 + SC0
        BCTR,UN TL_DEC_LP
TL_DEC_DONE:
        LODI,R0 TOK_DEC
TL_EMIT_CONST:                                ; V4.24: shared by TL_DEC_DONE, TL_MAYBEHEX
        STRA,R0 TOKBUF,R3+                    ;   In: R0 = TOK_DEC or TOK_HEX
        LODA,R0 EXPH
        STRA,R0 TOKBUF,R3+
        LODA,R0 EXPL
        STRA,R0 TOKBUF,R3+
        BCTA,UN TL_LP
TL_MAYBEHEX:
        BSTR,UN HEX_PEEK                      ; '$' + hex digit?
        BCTR,EQ TL_DOLLAR_PLAIN               ; no: plain '$'
        BSTR,UN PARSE_HEX                     ; EXP = value, IP past the digits
        LODI,R0 TOK_HEX
        BCTR,UN TL_EMIT_CONST

; =============================================================================
;  HEX_PEEK -- is IP -> '$' immediately followed by a hex digit?
;  (shared: TL_MAYBEHEX and PARSE_FACTOR)
; In:  IPH:IPL -> '$'
; Out: CC=EQ: no (R1 = $FF).  CC!=EQ: yes.  IP unchanged.
; Clobbers: R0, R1.  RAS: +2 (TL_HEXVAL nests inside)
HEX_PEEK:
        LODI,R1 1
        LODA,R0 *IPH,R1                      ; the char AFTER '$'
        BSTA,UN TL_HEXVAL
        COMI,R1 $FF
        RETC,UN

; =============================================================================
;  PARSE_HEX -- consume '$' and its hex digits into EXPH:EXPL
;  (shared: TL_MAYBEHEX and, via PF_NUM, direct-entry expressions)
; In:  IPH:IPL -> '$' followed by >= 1 hex digit (caller checked via HEX_PEEK)
; Out: EXPH:EXPL = value (16-bit, wraps mod 65536); IP -> first non-hex char
; Clobbers: R0, R1, SC0, EXPH, EXPL, TKH, TKL, R3SAVE.  R3 preserved.
; RAS: +2 (TL_MULADD nests its helpers). Tail-jump it (BCTA,UN) from
;   PARSE_FACTOR, never BSTA it, or the worst case reaches depth 8.
PARSE_HEX:
        ZBSR *VINC_IP                         ; consume '$'
        ZBSR *VCLR_EXP
PH_LP:
        LODA,R0 *IPH
        BSTA,UN TL_HEXVAL
        COMI,R1 $FF
        RETC,EQ                               ; non-hex char: done
        STRA,R0 SC0                           ; this digit's value (0-15)
        ZBSR *VINC_IP
        LODI,R0 16
        BSTR,UN TL_MULADD                     ; EXP = EXP*16 + SC0
        BCTR,UN PH_LP
TL_DOLLAR_PLAIN:
        LODA,R0 *IPH                             ; the '$' itself
        BCTA,UN TL_COPY1

; =============================================================================
;  TL_MULADD -- EXP = EXP*<multiplier> + SC0 (shared by TL_DECIMAL's x10 and
;  TL_MAYBEHEX's x16 accumulation -- both are "scale then add the next
;  digit", differing only in the multiplier, so one helper serves both
;  instead of two near-identical unrolled blocks). Same repeated-add
;  technique as PARSE_U16's x10 step, which now calls this routine too (V5.4:
;  its former private copy, PU16_M10, was deleted).
; In:  R0 = multiplier (10 or 16); SC0 = digit to add; EXPH:EXPL = value;
;      caller's R3 = TOKBUF write index
; Out: EXPH:EXPL = EXPH:EXPL * R0 + SC0; R3 unchanged (preserved via R3SAVE)
; Clobbers: R0, R1, TKH, TKL, R3SAVE.  Now also PARSE_U16's x10 step (V5.4)
TL_MULADD:
        STRA,R3 R3SAVE                       ; preserve caller's R3
        STRZ,R3                              
        LODI,R0 (IDX_EXP*16)+IDX_TK          ; V4.24: TK = EXP (value before
        ZBSR *VREG16_TO_REG16                ;   scaling); TK, not TMP
        EORZ,R0
        STRA,R0 EXPH
        LODA,R0 SC0
        STRA,R0 EXPL                         ; EXP = digit: the adds below accumulate into it,
                                             ;   giving digit + mult*value with no final add
TL_MULADD_LP:
        LODI,R0 TKH-IPH
        BSTA,UN ADD_CORE                     ; EXP += TK
        BDRR,R3 TL_MULADD_LP
        LODA,R3 R3SAVE                       ; restore caller's R3
        RETC,UN
TL_DONE:
        EORZ,R0
        STRA,R0 TOKBUF,R3+                        ; NUL-terminate
     
; =============================================================================
;  TRY_STORE_LINE -- Store or delete a numbered line if IP starts with a digit
; In:  IPH:IPL -> input buffer
; Out: CC=GT if line stored/deleted; CC=EQ if not a numbered line
; Clobbers: R0, R1, R2, R3, EXPH, EXPL, LNUMH, LNUML, IPH, IPL, TMPH, TMPL,
;   PEH, PEL, TKH, TKL
TRY_STORE_LINE:
        ZBSR *VDIGIT_CHECK
        BCFR,GT TSL_NUM                  ; not GT (R0<=9 unsigned): valid digit
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
        ZBSR *VEXP_TO_LNUM ; LNUMH:LNUML = EXPH:EXPL (parsed line number)
TSL_FND:
        BSTA,UN FIND_LINE                ; [+1] TMP = insertion point (first record
                                          ; >= LNUM); CC=EQ iff that record IS LNUM
        BCFR,EQ TSL_WRITE                 ; no such line: go store the new one
        BSTR,UN DEL_REC                   
        BCTR,UN TSL_FND
TSL_WRITE:
        ZBSR *VWSKIP                      ; [+1] skip space after line number
        LODA,R0 *IPH
        BCTR,EQ TSL_DONE                  ; NUL: empty body = delete only (done)
        BSTA,UN TOKENIZE_LINE             
        LODZ,R3                            ; V5.7: save body length (R3) before it's
        STRA,R0 TSL_BLEN                   ;   repurposed as the gap/copy size below
        ADDI,R3 4                          ; +2 header +1 len +1 NUL = total record size
        BSTA,UN OPEN_GAP                  ; make room at TMP; PE += R3 (V5.7: was BSTR, now
                                          ;   out of range (+64) after the length-byte code)
        LODI,R1 2                         ; write the 2-byte line-number header
TSL_HDR:
        LODA,R0 LNUMH,R1-                 ; R1 2->1: LNUML; 1->0: LNUMH
        STRA,R0 *TMPH,R1                  ; -> TMP[1] then TMP[0]
        BRNR,R1 TSL_HDR
        ZBSR *VINC_TMP
        ZBSR *VINC_TMP
        LODA,R0 TSL_BLEN                   ; V5.7: write the length byte
        STRA,R0 *TMPH
        ZBSR *VINC_TMP
        LODI,R1 $FF                       
        SUBI,R3 3                         ; R3: bodylen+4 -> bodylen+1 (body+NUL to copy)
TSL_CPY:
        LODA,R0 TOKBUF,R1+                ; V4.23 fix: copy by COUNT, not until
        STRA,R0 *TMPH                     ; the first NUL -- a TOK_DEC/TOK_HEX
        ZBSR *VINC_TMP                    ; payload can hold a $00 byte, which
        BDRR,R3 TSL_CPY                   ; truncated the record (TOKBUF's own
                                          ; NUL terminator is the last byte
                                          ; copied, so no separate store)
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
; Clobbers: R0, R1, TMPH:TMPL, EXPH:EXPL
DEL_REC:
        ZBSR *VTMP_TO_EXP ; EXP = dst = start of the doomed record
        BSTA,UN ADV_TMP_PAST_REC          ; TMP = src = start of the next record
DEL_LP:
        ZBSR *VCMP_TMP_PE ; (out of BSTR range from here)
        BCFR,LT DEL_END                  ; src >= PE: everything moved
        LODA,R0 *TMPH
        STRA,R0 *EXPH
        ZBSR *VINC_TMP
        ZBSR *VINC_EXP                   ; dst++ (already-named vector 
        BCTR,UN DEL_LP
DEL_END:
        LODI,R0 (IDX_EXP*16)+IDX_PE
        ZBRR *VREG16_TO_REG16             ; PE = dst (tail call)

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
; Clobbers: R0, R2, EXPH:EXPL
OPEN_GAP:
        LODI,R0 (IDX_PE*16)+IDX_EXP
        ZBSR *VREG16_TO_REG16             ; EXP = PE
        LODZ,R3
        STRZ,R2                          ; R2 = size, as the INC_ET step count
OG_ADD:
        ZBSR *VINC_EXP                   ; EXP++ (R2 times: EXP = PE + size)
        BDRR,R2 OG_ADD
        LODA,R0 EXPH
        COMI,R0 <PROGLIM+1
        BCFA,LT JERROOM                  ; new PE is past PROGLIM: out of memory
OG_LP:
        BSTR,UN CMP_TMP_PE
        BCFR,LT OG_FIX                   ; PE has walked down to TMP: done
        ; DEC_PE inlined here (was the only caller):
        LODA,R0 PEL
        SUBI,R0 1
        STRA,R0 PEL
        TPSL $01                         ; C=1(no borrow)->CC=EQ: skip hi--
        BCTR,EQ OG_NB
        LODA,R0 PEH
        SUBI,R0 1
        STRA,R0 PEH
OG_NB:
        LODA,R0 *PEH                     ; byte at PE ...
        STRA,R0 *PEH,R3                  ; ... goes to PE+R3 (indexed-indirect)
        BCTR,UN OG_LP
OG_FIX:
        LODI,R0 (IDX_EXP*16)+IDX_PE
        ZBRR *VREG16_TO_REG16             ; PE = EXP (tail call)

; =============================================================================
;  CMP_TMP_PE -- Compare TMPH:TMPL against PEH:PEL (16-bit unsigned compare,
;  byte-serial COMA under the global COM=1 -- ported from uBASIC2650 v2.13).
; Out: CC=GT if TMP > PE, CC=EQ if TMP = PE, CC=LT if TMP < PE
; Clobbers: R0
CMP_TMP_PE:
        LODA,R0 TMPH
        COMA,R0 PEH
        RETC,GT
        RETC,LT
        LODA,R0 TMPL
        COMA,R0 PEL
        RETC,UN

; =============================================================================
;  ADV_TMP_PAST_REC -- advance TMPH:TMPL past a stored record, landing on the
;  next record (ported from uBASIC2650).
;  V5.7: records are now length-prefixed -- [lineH][lineL][len][body...][NUL],
;  len = body byte count (TOKENIZE_LINE's own R3, not counting the NUL; see
;  TRY_STORE_LINE). This turns the skip into one length-byte read plus an
;  add-with-carry: O(1) instead of O(body length), still every statement
;  executed (DR_EXEC) and every candidate record FIND_LINE/FIND_INS rejects.
;  (V5.3's index-scan -- 4 instructions/body-byte -- is now obsolete; this
;  replaces it rather than building on it.) The NUL stays, unread by this
;  routine, so LIST's detokenizer (DLS_BLPX) needs no change.
; In:  TMPH:TMPL -> start of a record (its line-number hi byte)
; Out: TMPH:TMPL -> start of the next record
; Clobbers: R0, R1.  Leaf (no RAS use).  Record length must be < 252 bytes
;      (len byte, TOK_HEX/TOK_DEC range reserves $FD-$FF -- see TOKENIZE_LINE).
ADV_TMP_PAST_REC:
        LODI,R1 2
        LODA,R0 *TMPH,R1                 ; R0 = stored body length
        ADDI,R0 4                        ; + 2(line#) + 1(len byte) + 1(NUL)
        STRZ,R1                          ; R1 = total record size
        LODA,R0 TMPL
        ADDZ,R1                          ; TMPL += R1
        STRA,R0 TMPL
        TPSL $01                         ; carry out of the lo byte?
        RETC,LT                          ;   no: done
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
        RETC,UN

; =============================================================================
;  FIND_LINE -- Search for line LNUMH:LNUML in program store
; Out: TMPH:TMPL = record start if found; CC=EQ found, CC=GT not found.
; Clobbers: R0, R1, TMPH, TMPL
FIND_LINE_FWD:                           ; V5.6: same search, but it starts at TMP, not at PROG
        BSTR,UN FI_LP                    ; [+1]
        db $EC                           ; COMA,R0: swallows FIND_LINE's BSTR below (R0 and CC
                                         ;   are dead at FL_TAIL; the read address is harmless)
FIND_LINE:
        BSTR,UN FIND_INS                 ; [+1]
FL_TAIL:
        ; check if at end of program
        BSTR,UN CMP_TMP_PE                ; [+1]
        BCFR,LT FL_RET_NF                 ; not LT (GT or EQ): at/past end
FL_CHK:
        LODA,R0 *TMPH
        SUBA,R0 LNUMH
        BCTR,EQ FL_CHKLO
FL_RET_NF:
        LODI,R0 1                        ; CC=GT: not found
        RETC,UN

FL_CHKLO:
        LODI,R1 1
        LODA,R0 *TMPH,R1                 ; peek stored.lo at TMP+1 (indirect-indexed)
        SUBA,R0 LNUML
        BCFR,EQ FL_RET_NF
FL_FOUND:
        EORZ,R0                          ; CC=EQ: found
        RETC,UN

; =============================================================================
;  FIND_INS -- Find sorted insertion point for LNUMH:LNUML
; Returns TMPH:TMPL = address of first record with line >= LNUMH:LNUML,
; or PEH:PEL if all lines are smaller.
; In:  LNUMH:LNUML = target line number
; Out: TMPH:TMPL = insertion point
; Clobbers: R0, R1, TMPH, TMPL
FIND_INS:
        ZBSR *VSET_TMP_PROG
FI_LP:
        ZBSR *VCMP_TMP_PE
        RETC,GT
        RETC,EQ
;FI_CHK:
        LODA,R0 LNUMH
        SUBA,R0 *TMPH                    ; LNUMH - stored.hi
        BCTR,GT FI_ADV
        RETC,LT 
        ; hi bytes equal: check lo
        LODI,R1 1
        LODA,R0 LNUML
        COMA,R0 *TMPH,R1                  ; peek stored.lo at TMP+1 (indirect-
                                           ; indexed); unsigned (COM=1 global)
        BCTR,GT FI_ADV
        RETC,UN
FI_ADV:
        ; advance TMPH:TMPL past this record to the next one
        BSTA,UN ADV_TMP_PAST_REC           ; [+1] TMP = next record
        BCTR,UN FI_LP

; =============================================================================
;  PARSE_EXPR -- Recursive descent expression evaluator
; Handles: literals, variables (A-Z), unary +/-, parens, */% then +/-.
; In:  IPH:IPL -> expression string
; Out: EXPH:EXPL = 16-bit signed result
; Clobbers: R0, R3, SAVEH, SAVEL, NEGFLG, SC0, SC1, TMPH, TMPL
; RAS guard (v3.2): SPSU/ANDI/COMI fires ERR_NEST if SP>=5 at entry.
;   Inline (no BSTA): guard costs 0 RAS slots. Threshold 5: at SP=5, inner
;   calls (PARSE_FACTOR+PARSE_S16+inline INC_IP) would push SP to 7+, overflow.
PARSE_EXPR:
        ; RAS guard check
        SPSU                             ; R0 = PSU; SP in bits 2:0
        ANDI,R0 $07                      ; isolate SP field
        COMI,R0 PE_RAS_LIMIT             ; threshold
        BCTR,LT PE_SAFE                  ; SP < 5: safe to proceed
        LODI,R0 ERR_NEST
        ZBRR *VDO_ERROR                  ; abort gracefully
PE_SAFE:
        ; V5.6: only a function token (stored program) or a multi-letter word (direct-mode
        ; raw text) can start a function call.  Everything else (variable, number, '(' ...)
        ; went through marker push + SET_TMP + MATCH_KW + WORD_START + miss-row dispatch +
        ; marker pop, ~57 instructions, only to arrive at PE_NOFUNC_TOP.  Test first.
        ; IP is advanced past blanks exactly as MATCH_KW's own first step did.
        ZBSR *VWSKIP                      ; R0 = first non-blank char
        SUBI,R0 TOK_FUNC_BASE
        COMI,R0 FUNC_COUNT
        BCTR,LT PE_FSCAN                  ; function token: scan (O(1) token path in MATCH_KW)
        LODA,R0 RUNFLG
        BCFR,EQ PE_NOFUNC_TOP             ; running: stored lines are tokenized, no raw words
        BSTA,UN WORD_START                ; direct mode: two letters in a row = maybe a word
        BCFR,EQ PE_NOFUNC_TOP             ; no: straight to the atom
PE_FSCAN:
        ; FUNCATOM-01 (v4.6): push a $FF "top-level" origin marker onto
        ; FT_STK before scanning FUNC_TAB. EAM_ATOM's own scan (see below)
        ; pushes the live R3 instead of $FF; PE_NOFUNC/FUNC_EPILOG pop this
        ; to learn which completion path a dispatch needs. R3 is provably
        ; never $FF at the instant EAM_ATOM dispatches (every EAM_ATOM
        ; caller has already pushed something onto SWBASE first), so $FF
        ; is a safe, unambiguous top-level sentinel 
        LODA,R0 FT_SP
        ADDI,R0 1
        STRZ,R1
        STRA,R1 FT_SP
        LODI,R0 $FF
        STRA,R0 FT_STK,R1
        ; Check for functions and tail call them to return.
        ; Setup at statement FUNC_TAB with TMPH:TMPL as pointer
        LODI,R0 <FUNC_TAB
        LODI,R1 >FUNC_TAB
        ZBSR *VSET_TMP
        LODI,R1 >(FUNC_MISS-FUNC_TAB)   ; miss-row offset for MATCH_KW's early-out
        BCTA,UN MATCH_KW                ; resumes at PE_NOFUNC if not match
PE_NOFUNC:
        ; FUNCATOM-01: pop the origin marker pushed above (by PE_SAFE or
        ; EAM_ATOM) to learn which miss-handling path applies.
        LODA,R1 FT_SP
        LODA,R0 FT_STK,R1
        SUBI,R1 1
        STRA,R1 FT_SP
        COMI,R0 $FF
        BCTR,EQ PE_NOFUNC_TOP            ; PE_SAFE origin: existing behaviour
        ; EAM_ATOM origin: not a function name. R3/SWBASE were never
        ; modified by the scan attempt itself (untouched on a miss), so
        ; just fall to ordinary atom parsing exactly as EAM_ATOM's pre-fix
        ; fallthrough did. But EAM_ATOM saved a speculative SWBASE copy
        ; onto FT_SAVE before it knew hit/miss - discard it here (R0=N)
        ; so FT_SAVE_SP stays correctly paired for the next dispatch.
        STRZ,R1                          ; R1 = N
        LODA,R0 FT_SAVE_SP
        SUBZ,R1                          ; R0 -= N
        SUBI,R0 1                        ; R0 -= 1  (drop N+1 bytes total)
        STRA,R0 FT_SAVE_SP
        BSTA,UN PARSE_FACTOR
        ZBRR *VPARSER_RET
PE_NOFUNC_TOP:
        LODI,R3 $FF                     ; SW stack empty sentinel
EXPR_AM:
        LODI,R0 >EAM0_RET
        LODI,R1 <EAM0_RET
        ZBSR *VPUSH_RET
        ZBRR *VEAM_ATOM 
EAM0_RET:
        ZBSR *VCHECK_POW
        LODI,R0 >EAM_HI0_RET
        LODI,R1 <EAM_HI0_RET
        ZBSR *VPUSH_RET
        ZBRR *VEAM_HI 
EAM_HI0_RET:
EAM_LO_LOOP:
        ZBSR *VWSKIP  
        COMI,R0 A'+'
        BCTR,EQ EAM_PLUS
        COMI,R0 A'-'
        BCTR,EQ EAM_MINUS
        ZBRR *VPARSER_RET
EAM_PLUS:
        ; PAREN-NEST-01 fix (v4.3): push left operand onto SWBASE instead of
        ; flat SAVEH:SAVEL. A flat cell gets clobbered by a same-precedence
        ; op at a deeper recursion (e.g. "2*(3*4)"); the SW stack frame can't
        ; be, since it's LIFO and balanced with every push/pop in this loop.
        BSTR,UN PUSH_EXP
        LODI,R0 >EAM_P_RET
        LODI,R1 <EAM_P_RET
        ZBSR *VPUSH_RET
        ZBRR *VEAM_ATOM 
EAM_P_RET:
        ZBSR *VCHECK_POW
        LODI,R0 >ADD16_SAVE_EXP 
        LODI,R1 <ADD16_SAVE_EXP 
        ZBSR *VPUSH_RET
        ZBRR *VEAM_HI 
EAM_MINUS:
        BSTR,UN PUSH_EXP
        LODI,R0 >EAM_M_RET
        LODI,R1 <EAM_M_RET
        ZBSR *VPUSH_RET
        ZBRR *VEAM_ATOM 
EAM_M_RET:
        ZBSR *VCHECK_POW
        LODI,R0 >EAM_MH_RET
        LODI,R1 <EAM_MH_RET
        ZBSR *VPUSH_RET
        ZBRR *VEAM_HI 
EAM_MH_RET:
        BSTA,UN NEG_EXP_BODY              ; EXP = -EXP (V4.2: shared, was SUBA)
        BCTR,UN ADD16_SAVE_EXP            ; EXP = SAVE + (-EXP) = SAVE - EXP

; =============================================================================
;  PUSH_EXP -- push EXPH:EXPL onto SWBASE (lo,hi) and consume the operator
;  char at IP
; In:  EXPH:EXPL = operand to push; IP -> the operator char ('+','-','*', etc)
; Out: SWBASE top = pushed EXPH:EXPL (lo,hi); IP advanced past the operator
; Clobbers: R0, R3
PUSH_EXP:
        LODA,R0 EXPL
        STRA,R0 SWBASE,R3+
        LODA,R0 EXPH
        STRA,R0 SWBASE,R3+
        ZBRR *VINC_IP

; =============================================================================
;  PUSH_RET -- push a 16-bit literal return address onto SWBASE
; In:  R0 = low byte, R1 = high byte of the return address; R3 = SW SP
; Out: SWBASE[R3-1]=lo, SWBASE[R3]=hi (pushed); R3 += 2; R1 unchanged, R0 clobbered
; Clobbers: R0
PUSH_RET:
        STRA,R0 SWBASE,R3+
        LODZ,R1
        STRA,R0 SWBASE,R3+
        RETC,UN

; =============================================================================
;  ADD16_SAVE_EXP -- EXP = SAVE + EXP (16-bit, WC carry chain); resumes EAM loop
;  Shared tail for EAM_PLUS and EAM_MINUS (V4.2). EAM_MINUS negates EXP first.
;  V4.3 (PAREN-NEST-01): left operand is popped off SWBASE (pushed by
;  EAM_PLUS/EAM_MINUS) into SAVEH:SAVEL just before use, instead of being
;  read from a flat cell that recursion could have clobbered in the meantime.
; In:  EXPH:EXPL = right operand; SWBASE top = pushed left operand (lo,hi)
; Out: EXPH:EXPL = left + EXPH:EXPL; tail-jumps into EAM_LO_LOOP
; Clobbers: R0, R3 (popped by 2)
ADD16_SAVE_EXP:
        BSTA,UN POP_SAVE_TO_TMP            ; [+1] TMP = left operand (TMP is dead mid-expression); R3 -= 2
        LODI,R0 TMPH-IPH
        BSTR,UN ADD_CORE                   ; [+1] EXP += TMP
        BCTA,UN EAM_LO_LOOP

; =============================================================================
;  ADD_CORE -- shared 16-bit indexed add: EXPH:EXPL += the pair at IPH+R0 (hi byte).
;  Ported from uBASIC2650 v2.8 (via pBASIC v0.33): serves ADD16_SAVE_EXP, TL_MULADD
;  (and thereby PARSE_U16's x10) and DO_NEXT.
; In:  R0 = hi-byte offset of the pair from IPH (TMPH-IPH, TKH-IPH, VARS-IPH+2n); WC=0
; Out: EXPH:EXPL += *(IPH+off):*(IPL+off); WC=0
; Clobbers: R0, R1.  Leaf.
ADD_CORE:
        STRZ,R1
        LODA,R0 EXPL
        ADDA,R0 IPL,R1
        STRA,R0 EXPL
        PPSL PSW_WC                        ; WC=1: the high byte takes the carry
        LODA,R0 EXPH
        ADDA,R0 IPH,R1
        STRA,R0 EXPH
        CPSL PSW_WC
        RETC,UN

EAM_HI:
        ZBSR *VWSKIP  
        COMI,R0 A'*'
        BCTR,EQ EAM_MUL
        COMI,R0 A'/'
        BCTR,EQ EAM_DIV
        COMI,R0 A'%'
        BCTR,EQ EAM_MOD
        ZBRR *VPARSER_RET
EAM_MUL:
        ; PAREN-NEST-01 fix (v4.3): push, not flat E1SAVH:E1SAVL (see EAM_PLUS)
        ZBSR *VPUSH_EXP
        LODI,R0 >MU_AT_RET
        LODI,R1 <MU_AT_RET
        ZBSR *VPUSH_RET
        ZBRR *VEAM_ATOM 
MU_AT_RET:
        ZBSR *VCHECK_POW
        BSTR,UN POP_SAVE_TO_TMP           ; TMPH:TMPL = popped left operand
        BSTA,UN MUL16
        ZBRR *VEAM_HI 
EAM_DIV:
        ZBSR *VPUSH_EXP
        LODI,R0 >DV_AT_RET
        LODI,R1 <DV_AT_RET
        ZBSR *VPUSH_RET
        ZBRR *VEAM_ATOM 
DV_AT_RET:
        ZBSR *VCHECK_POW
        BSTR,UN POP_SAVE_TO_TMP
        BSTA,UN DIV16
        ZBRR *VEAM_HI 
EAM_MOD:
        ZBSR *VPUSH_EXP
        LODI,R0 >MD_AT_RET
        LODI,R1 <MD_AT_RET
        ZBSR *VPUSH_RET
        ZBRR *VEAM_ATOM 
MD_AT_RET:
        ZBSR *VCHECK_POW
        BSTR,UN POP_SAVE_TO_TMP
        BSTA,UN DIV16
        ZBSR *VTMP_TO_EXP ; EXPH:EXPL = TMPH:TMPL (remainder)
        ZBRR *VEAM_HI 

; =============================================================================
;  POP_SAVE_TO_TMP -- pop a 2-byte value pushed on SWBASE into TMPH:TMPL
;  Shared tail for EAM_MUL/EAM_DIV/EAM_MOD (V4.3, part of PAREN-NEST-01 fix).
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
        
EAM_ATOM:
        ZBSR *VWSKIP  
        COMI,R0 A'-'
        BCTA,EQ EAM_NEG
        COMI,R0 A'+'
        BCTA,EQ EAM_POS
        COMI,R0 A'('
        BCTA,EQ EAM_PAREN
        ; V5.3: only a function token or a multi-letter word can be a function.
        ; A variable/constant skips the FT save/scan/restore machinery (PE_NOFUNC's
        ; EAM-origin path did exactly this PARSE_FACTOR call after undoing it).
        ; Statement tokens fall to PARSE_FACTOR -> syntax error (they used to be
        ; dispatched through FUNC_TAB's offset table).
        SUBI,R0 TOK_FUNC_BASE
        COMI,R0 FUNC_COUNT
        BCTR,LT EA_FUNC                  ; function token: MATCH_KW's O(1) path
        LODA,R0 RUNFLG
        BCFR,EQ EA_PLAIN                 ; V5.6: running = tokenized text, no raw words
        BSTA,UN WORD_START               ; direct mode raw text: letter followed by letter?
        BCTR,EQ EA_FUNC
EA_PLAIN:
        BSTA,UN PARSE_FACTOR             ; plain atom: variable / constant / number
        ZBRR *VPARSER_RET
EA_FUNC:
        LODA,R0 FT_SP
        ADDI,R0 1
        STRZ,R1
        STRA,R1 FT_SP
        LODZ,R3
        STRA,R0 FT_STK,R1                ; FT_STK[level] = R3 (=N)
        LODZ,R2
        STRA,R0 FT_R2SAVE                ; stash caller's R2
        LODA,R0 FT_STK,R1                ; reload N (R0 was clobbered above)
        STRA,R0 FT_N                      ; stash N (R1 gets repurposed below)
        STRZ,R3                          ; R3 = N (SWBASE src idx, explicit
                                          ; SUBI each pass - see note below)
        ADDI,R0 1
        STRZ,R1                          
        LODA,R2 FT_SAVE_SP                ; R2 = FT_SAVE top ($FF=empty)
EAMS_SAVE_LP:
        ADDI,R2 1                        ; pre-increment to next free slot
        LODA,R0 SWBASE,R3                ; R0 = SWBASE[R3]   (R3=N,N-1,...,0)
        STRA,R0 FT_SAVE,R2               ; FT_SAVE[R2] = R0
        SUBI,R3 1
        BDRR,R1 EAMS_SAVE_LP             ; R1--; loop while R1!=0 (N+1 passes)
        LODZ,R2
        STRA,R0 FT_SAVE_SP                ; FT_SAVE_SP = R2 (new top=SWBASE[0])
        LODA,R0 FT_R2SAVE
        STRZ,R2                          ; restore caller's R2
        LODA,R0 FT_N                  
        STRZ,R3                         
        LODI,R0 <FUNC_TAB
        LODI,R1 >FUNC_TAB
        ZBSR *VSET_TMP
        ; R1 is not the miss offset: only a function token or a two-letter word gets here,
        ; so MATCH_KW's non-word early-out (which needs R1) cannot trigger.
        BCTA,UN MATCH_KW                 ; hit -> handler -> FUNC_EPILOG
                                          ; miss -> PE_NOFUNC
EAM_NEG:
        ZBSR *VINC_IP  
        LODI,R0 >NEG_AT_RET
        LODI,R1 <NEG_AT_RET
        ZBSR *VPUSH_RET
        ZBRR *VEAM_ATOM 
NEG_AT_RET:
        BSTA,UN NEG_EXP_BODY
        ZBSR *VCHECK_POW
        BCTR,UN PARSER_RET
EAM_POS:
        ZBSR *VINC_IP  
        LODI,R0 >POS_AT_RET
        LODI,R1 <POS_AT_RET
        ZBSR *VPUSH_RET
        ZBRR *VEAM_ATOM 
POS_AT_RET:
        ZBSR *VCHECK_POW
        BCTR,UN PARSER_RET
EAM_PAREN:
        ZBSR *VINC_IP  
        LODI,R0 >EP_RET
        LODI,R1 <EP_RET
        ZBSR *VPUSH_RET
        BCTA,UN EXPR_AM
EP_RET:
        ZBSR *VWSKIP  
        ZBSR *VINC_IP  
        ;drop through

; =============================================================================
;  PARSER_RET -- Shared parser return via RAS or SW stack
; In:  R3 = SW stack pointer ($FF = empty)
; Out: returns to caller via RAS (if R3=$FF) or SW stack
; Clobbers: R0
PARSER_RET:
        LODZ,R3
        EORI,R0 $FF                      ; $FF -> $00 (EQ): use RAS
        RETC,EQ
        BCTA,UN SWRETURN

; =============================================================================
;  FUNC_EPILOG -- shared exit for all function handlers (FUNCATOM-01 fix,
;  v4.6). VFUNC_CONT now points here so none of the 5 ZBRR *VFUNC_CONT call
;  sites need to change. Pops the origin marker pushed by PE_SAFE/EAM_ATOM
;  (see their headers) to pick the correct completion:
;    $FF (PE_SAFE / top-level)  -> FUNC_CONT below (unchanged: R3 reset,
;                                  resumes EAM0_RET/EAM_HI/EAM_LO_LOOP as a
;                                  fresh atom - reuses the SAME machinery
;                                  every ordinary atom already uses for its
;                                  trailing */,%/+,- continuation)
;    else (EAM_ATOM / mid-expr) -> byte-restore SWBASE[0..N] from FT_SAVE
;                                  (the handler's own argument-parse reset
;                                  R3=$FF and regrew SWBASE from 0, so the
;                                  pointer alone isn't enough - the data
;                                  underneath must come back too), restore
;                                  R3=N, and tail-jump to PARSER_RET -
;                                  exactly how an ordinary PARSE_FACTOR
;                                  atom exits EAM_ATOM.
; In:  EXPH:EXPL = function's result; FT_SP/FT_STK/FT_SAVE_SP/FT_SAVE =
;      origin stack (top entry belongs to THIS dispatch - see PE_SAFE
;      header for why this is always true even through nested
;      function-argument calls)
; Out: control passes to FUNC_CONT or PARSER_RET
; Clobbers: R0, R1, R2 (saved/restored), R3
; No HW call/RETC used - inline, SW-stack domain only (see EAM_ATOM header)
FUNC_EPILOG:
        LODA,R1 FT_SP
        LODA,R0 FT_STK,R1
        SUBI,R1 1
        STRA,R1 FT_SP
        COMI,R0 $FF
        BCTR,EQ FUNC_CONT
        STRA,R0 FT_N                      ; stash N (=R3 to restore)
        LODZ,R2
        STRA,R0 FT_R2SAVE                 ; stash caller's R2
        LODA,R0 FT_N
        ADDI,R0 1
        STRZ,R1                          
        EORZ,R0
        STRZ,R3                           ; R3 = 0 (SWBASE dest idx, explicit
                                           ; ADDI each pass)
        LODA,R2 FT_SAVE_SP                 ; R2 = FT_SAVE top (points at SWBASE[0])
FE_REST_LP:
        LODA,R0 FT_SAVE,R2                ; R0 = FT_SAVE[R2]  (R2 = top,top-1,...)
        STRA,R0 SWBASE,R3                 ; SWBASE[R3] = R0   (R3 = 0,1,...,N)
        ADDI,R3 1
        SUBI,R2 1
        BDRR,R1 FE_REST_LP                ; R1--; loop while R1!=0 (N+1 passes)
        LODZ,R2
        STRA,R0 FT_SAVE_SP                 ; FT_SAVE_SP = R2 (shrunk by N+1)
        LODA,R0 FT_R2SAVE
        STRZ,R2                           ; restore caller's R2
        LODA,R0 FT_N
        STRZ,R3                           ; restore outer SW-stack pointer
        ZBRR *VPARSER_RET
FUNC_CONT:
        LODI,R3 $FF
        BCTA,UN EAM0_RET

; =============================================================================
;  PARSE_FACTOR -- Parse a single value (variable or literal)
; V4.22: a tokenized numeric constant (TOK_DEC/TOK_HEX + 2 payload bytes,
;   see TOKENIZE_LINE) is recognized up front and loaded directly -- O(1),
;   vs. PARSE_S16's per-digit accumulation for untokenized/direct-entry
;   text, which still works exactly as before for every other case.
; V5.1: an untokenized '$'+hex digit (direct entry) goes to PARSE_HEX.
; In:  IPH:IPL -> first char of factor
; Out: EXPH:EXPL = value
; Clobbers: R0, R1, SC0 (+ TKH, TKL, R3SAVE on the hex path). R2, R3 preserved.
PARSE_FACTOR:
        LODA,R0 *IPH
        COMI,R0 TOK_DEC                  ; V4.22: tokenized decimal constant?
        BCTR,EQ PF_CONST
        COMI,R0 TOK_HEX                  ; tokenized hex constant?
        BCTR,EQ PF_CONST
        SUBI,R0 A'A'                     ; shift 'A'-'Z' to 0-25 (COM=1: one unsigned test);
        COMI,R0 A'Z'-A'A'                ; PF_LOADVAR uses the shifted value as its index
        BCFR,GT PF_LOADVAR               ; A-Z: R0 = index 0..25
PF_NUM:
        COMI,R0 '$'-A'A'                 ; R0 is still char-'A'
        BCFA,EQ PARSE_S16                ; not hex: decimal (tail: RETC,UN returns to our caller)
        BSTA,UN HEX_PEEK
        BCFA,EQ PARSE_HEX                ; '$' + hex digit: tail, returns to our caller
        ZBRR *VJSYNERR                   ; '$' + non-hex: syntax error
PF_CONST:
        ZBSR *VINC_IP                    ; [+1] past the token byte
        ZBRR *VFETCH16_IP                ; tail: EXP = payload, IP past it; returns to our caller

; =============================================================================
;  PF_LOADVAR -- Load variable value from VARS
; In:  R0 = variable index 0..25 (PARSE_FACTOR's SUBI result); IP -> that char
; Out: EXPH:EXPL = variable value; IP past the letter (INC_IP is tail-called,
;      so CC is INC_IP's)
; Clobbers: R0, R1.  INC_IP works in register bank 1, so R1 survives it.
PF_LOADVAR:
        ADDZ,R0                          ; R0 = index*2
        STRZ,R1                          ; R1 = VARS byte offset
        LODA,R0 VARS,R1                  ; hi byte
        STRA,R0 EXPH
        LODA,R0 VARS+1,R1                ; lo byte
        STRA,R0 EXPL
        ZBRR *VINC_IP                    ; tail call: consume the letter, return to caller

; =============================================================================
;  PARSE_RELOP -- Parse relational operator at IP, build bitmask in RELOP
; bit0=LT, bit1=EQ, bit2=GT. '<'->1, '='->2, '>'->4, '<='->3, '<>'->5, '>='->6
; In:  IP -> first char of relop
; Out: RELOP = bitmask
; Clobbers: R0, R1
PARSE_RELOP:
        ZBSR *VWSKIP                      ; [+1]
        EORZ,R0
        STRZ,R1                          ; R1 = 0 (mask accumulator)
PRO_LP:
        LODA,R0 *IPH
        COMI,R0 A'<'
        BCTR,EQ PRO_LT
        COMI,R0 A'='
        BCTR,EQ PRO_EQ
        COMI,R0 A'>'
        BCTR,EQ PRO_GT
        LODZ,R1
        BCTR,EQ PRO_NONE        ; surrogate for SYNERR
        STRA,R0 RELOP
        RETC,UN
PRO_LT:
        IORI,R1 1
        db $EC                  ; COMA,R0: skip next 2 bytes 
PRO_EQ:
        IORI,R1 2
        db $EC                  ; COMA,R0: skip next 2 bytes 
PRO_GT:
        IORI,R1 4
        ZBSR *VINC_IP  
        BCTR,UN PRO_LP
PRO_NONE:
        ZBRR *VJSYNERR 

; =============================================================================
;  PARSE_S16 -- Parse signed decimal integer
; In:  IPH:IPL -> first char (optional '-' then digits)
; Out: EXPH:EXPL = signed 16-bit value
; Clobbers: R0, NEGFLG (0 = negate), EXPH, EXPL
PARSE_S16:
        LODA,R0 *IPH
        EORI,R0 A'-'                     ; R0 = 0 iff a minus sign
        STRA,R0 NEGFLG                   ; NEGFLG polarity: 0 = negate the result (see
                                         ; NEG_EXP), any other value = leave. Written
                                         ; unconditionally, so no separate clear anywhere.
        BCFR,EQ PS16_UN                  ; not '-' (STRA leaves EORI's CC alone)
        ZBSR *VINC_IP  
PS16_UN:

; =============================================================================
;  PARSE_U16 -- Parse unsigned decimal digits -> EXPH:EXPL
; Jumps to JSYNERR if no digits found.
; In:  IPH:IPL -> first digit char
; Out: EXPH:EXPL = value
; Clobbers: R0, R1, SC0, EXPH, EXPL, TKH, TKL (R3 preserved via R3SAVE in TL_MULADD)
;PARSE_U16:
        ZBSR *VCLR_EXP
        ZBSR *VDIGIT_CHECK
        BCTR,GT PRO_NONE; surrogate for JSYNERR  ; BUG-PU16-01: a leading ':' must not start a number
PU16_LP:
        ZBSR *VDIGIT_CHECK
        BCTR,GT PU16_RET ; RETC,GT
PU16_DIG:
        STRA,R0 SC0                      ; R0 = digit value from DIGIT_CHECK
        ZBSR *VINC_IP
        LODI,R0 10
        BSTA,UN TL_MULADD                ; EXP = EXP*10 + SC0 (shared with the tokenizer)
        BCTR,UN PU16_LP

PU16_RET:
        ; drop through

; =============================================================================
;  NEG_EXP -- Negate EXPH:EXPL if NEGFLG says to
;  NEG_EXP_BODY -- Unconditional negate EXPH:EXPL
; In:  EXPH:EXPL = value; NEGFLG = flag (0 = negate, nonzero = leave)
; Out: EXPH:EXPL negated (two's complement) if NEGFLG==0
; Clobbers: R0, R1
NEG_EXP:
        LODA,R0 NEGFLG
        RETC,GT                          ; nonzero (either sign) -> leave; needs both,
        RETC,LT                          ; NEGFLG's non-zero values are arbitrary bytes
                                         ; (PARSE_S16) or $80 (SETUP_MULDIV/ABS_X)
NEG_EXP_BODY:
        LODI,R1 EXPH-IPH                 ; EXPH offset from IPH (= 4); R1 variant for NEG_SHARED
        BCTR,UN NEG_SHARED

; =============================================================================
;  DO_NEG_FUNC -- NEG(a), arithmetic negation (v4.5: paren-bounded, see
DO_NEG_FUNC:
        ZBSR *VARG_OPEN
        ZBSR *VPARSE_EXPR
        ZBSR *VARG_CLOSE
        BSTR,UN NEG_EXP_BODY              ; real call now (was tail-jump) so
        ZBRR *VFUNC_CONT                  ; control returns here for FUNC_CONT

; =============================================================================
;  DO_ABS_FUNC -- ABS(a), absolute value (v4.5: paren-bounded, see
DO_ABS_FUNC:
        ZBSR *VARG_OPEN
        ZBSR *VPARSE_EXPR
        ZBSR *VARG_CLOSE
        BSTR,UN ABS_EXP                    ; real call now (was fall-through)
        ZBRR *VFUNC_CONT                   ; control returns here for FUNC_CONT

; =============================================================================
;  ABS_EXP / ABS_X -- Absolute value of EXPH:EXPL (ABS_EXP) or of the 16-bit value
;  at IPH+R1 (ABS_X, R1 = hi byte offset, e.g. TMPH-IPH); toggles NEGFLG if it was
;  negative.  Falls into NEG_SHARED (ported from uBASIC2650 v2.14).
; In:  ABS_X: R1 = TMPH-IPH or EXPH-IPH.  NEGFLG = current flag ($00 / $80 sentinels)
; Out: value = |value|; NEGFLG toggled ($00<->$80) if it was negative
; Clobbers: R0, R1
ABS_EXP:
        LODI,R1 EXPH-IPH
ABS_X:
        LODA,R0 IPH,R1
        RETC,GT                          ; LODA's own CC separates
        RETC,EQ                          ; +ve (GT) / zero (EQ) / -ve (LT)
        LODA,R0 NEGFLG
        EORI,R0 $80                      ; toggle between the $00/$80 sentinels
        STRA,R0 NEGFLG
        ; fall through to NEG_SHARED (R1 is already the offset)

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
;  DO_LIST -- Epilogue Print stored BASIC lines, optionally filtered by range
; Syntax: LIST  |  LIST start,end
; In:  PROG=program base, PEH:PEL=program end
; Out: matching lines printed
; Clobbers: R0, R1, R3, IPH, IPL, LNUMH, LNUML, TMPH, TMPL, EXPH, EXPL,
;           CURH, CURL, SAVEH, SAVEL, NEGFLG, SC0, SC1, ARGAH, ARGAL
; Notes: CURH:CURL holds end line. $7FFF sentinel = no upper bound (full list).
DO_LIST:
        ; Peek first char: MATCH_KW leaves IP at space before args (if any).
        ZBSR *VWSKIP                      ; skip whitespace
        COMI,R0 CR                       ; no args
        BCFR,LT DLS_ARG                  ; digits - get args
;DLS_FULL:
        ; Otherwise Full list: set end sentinel $7FFF, IP = program start
        LODI,R0 $7F
        STRA,R0 CURH
        LODI,R0 $FF
        STRA,R0 CURL
        ZBSR *VSET_TMP_PROG
        BCTA,UN DLS_LP

; =============================================================================
;  DO_POKE -- Write to Memory, expressions allowed for Addr and Byte
; Syntax: POKE addr, Byte
; clobbers; EXP, ARGAH, ARGAL
DO_POKE:
        ZBSR *VWSKIP                    ; chew whitespace
        LODI,R0 P2A_POKEOP-P2A_ANDOP
        db $EC                          ; consume next 2 bytes
DLS_ARG:
        ; LIST start,end (v4.4.2: shares PARSE_2ARGS's body - tail-jumps in
        ; at P2A_NOPAREN; P2A_LISTOP does the LNUM/CUR setup and continues
        ; into DLS_LP below).
        LODI,R0 P2A_LISTOP-P2A_ANDOP
        STRA,R0 FUNCOP
        BCTR,UN P2A_NOPAREN
        
; =============================================================================
;  ARG_OPEN / ARG_CLOSE -- require and consume '(' / ')' around a function argument
;  FUNC-01 fix (V5.5).  The single-argument functions called PARSE_EXPR on "(x)+y",
;  so the operators after the ')' were swallowed into the argument: ABS(-5)+10 gave 5.
;  Now the handler requires '(', parses x (PARSE_EXPR stops at the ')'), requires ')'.
;  They are LEAF helpers called from the handler on purpose: a wrapper around
;  PARSE_EXPR would cost 2 return-stack levels per nested function instead of 1.
; In:  IP -> optional blanks, then the bracket
; Out: bracket consumed; error JSYNERR if it is not there
; Clobbers: R0, R1.  RAS: +1 (WSKIP +1 more, INC_IP is tail-called)
ARG_OPEN:
        LODI,R1 A'('
        db $EC                          ; COMA,R0 swallows ARG_CLOSE's LODI,R1
ARG_CLOSE:
        LODI,R1 A')'
        ZBSR *VWSKIP                    ; R0 = next non-blank (WSKIP leaves R1 alone)
        COMZ,R1
        BCFA,EQ JSYNERR
        ZBRR *VINC_IP                   ; consume the bracket (tail call)

; =============================================================================
;  DO_AND_FUNC / DO_OR_FUNC / DO_XOR_FUNC -- bitwise AND(a,b)/OR(a,b)/XOR(a,b)
; FUNCOP = assembly-time literal offset into P2A_ANDOP (see DO_AND_FUNC etc
;          and DO_POKE/DO_LIST below) - no runtime multiply needed.
DO_AND_FUNC:
        EORZ,R0         ; no offset
        db $EC                          ; consume next 2 bytes
DO_OR_FUNC:
        LODI,R0 P2A_OROP-P2A_ANDOP
        db $EC                          ; consume next 2 bytes
DO_XOR_FUNC:
        LODI,R0 P2A_XOROP-P2A_ANDOP
        STRA,R0 FUNCOP
        ; drop through
; =============================================================================
;  PARSE_2ARGS -- shared "(a,b)" / "a,b" parser + BXA dispatch (v4.4.2)
;  ONE body for AND(a,b)/OR(a,b)/XOR(a,b) *and* DO_POKE/DO_LIST's "a,b"
;  - AND/OR/XOR enter at PARSE_2ARGS (top): requires '(', requires ')'.
;  - DO_POKE/DO_LIST tail-jump (BCTA,UN, no RAS cost) straight into
;    P2A_NOPAREN, skipping the '(' check; the ')' check below is optional
;  - FUNCOP (set by the caller, see DO_AND_FUNC/DO_POKE/DO_LIST) is the
;    literal byte offset from P2A_ANDOP to the target handler
; In:  IP -> '(' (top entry) or first char of 'a' (P2A_NOPAREN entry)
;      FUNCOP = target offset from P2A_ANDOP
; Out: per-handler (see P2A_ANDOP/OROP/XOROP/POKEOP/LISTOP below)
; Clobbers: R0, R3, SAVEH, SAVEL, NEGFLG, SC0, SC1, TMPH, TMPL, ARGAH, ARGAL
PARSE_2ARGS:
        ZBSR *VARG_OPEN          ; require+consume '('

P2A_NOPAREN:
        ZBSR *VPARSE_EXPR               ; a -> EXP
        LODI,R0 (IDX_EXP*16)+IDX_ARGA
        ZBSR *VREG16_TO_REG16             
        ZBSR *VWSKIP
        COMI,R0 A','
        BCFA,EQ JSYNERR                 ; require comma
        ZBSR *VINC_IP
        ZBSR *VPARSE_EXPR               ; b -> EXP
        ZBSR *VWSKIP
        COMI,R0 A')'
        BCFR,EQ P2A_DISPATCH            ; no ')' (POKE/LIST): don't consume
        ZBSR *VINC_IP                    ; consume ')' (AND/OR/XOR)
P2A_DISPATCH:
        LODA,R3 FUNCOP                   ; literal offset from P2A_ANDOP
        LODA,R0 ARGAH
        BXA P2A_ANDOP,R3                  ; jumps STRAIGHT to the handler
P2A_ANDOP:
        ANDA,R0 EXPH
        STRA,R0 EXPH
        LODA,R0 ARGAL
        ANDA,R0 EXPL
        BCTR,UN P2A_RET        
P2A_OROP:
        IORA,R0 EXPH
        STRA,R0 EXPH
        LODA,R0 ARGAL
        IORA,R0 EXPL
P2A_RET:        
        STRA,R0 EXPL
        ZBRR *VFUNC_CONT      
P2A_XOROP:
        EORA,R0 EXPH
        STRA,R0 EXPH
        LODA,R0 ARGAL
        EORA,R0 EXPL
        BCTR,UN P2A_RET        
P2A_POKEOP:
        LODA,R0 EXPL                     ; payload byte (low byte only)
        STRA,R0 *ARGAH                   ; store at ARGAH:ARGAL pointer
        RETC,UN

; =============================================================================
;  DO_NOT_FUNC -- bitwise NOT(a), one's complement (no Boolean truth tables)
;  v4.5: paren-bounded (previously had none - same FUNCCONT-01 absorption
;  bug as ABS/NEG before their fix: "NOT(0)+1" computed NOT(0+1) instead of
;  NOT(0)+1). Continuation handled by the shared P2A_RET tail below.
DO_NOT_FUNC:
        ZBSR *VARG_OPEN
        ZBSR *VPARSE_EXPR
        ZBSR *VARG_CLOSE
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        BCTR,UN P2A_RET        

P2A_LISTOP:
        LODI,R0 (IDX_ARGA*16)+IDX_LNUM
        ZBSR *VREG16_TO_REG16             ; LNUMH:LNUML = ARGAH:ARGAL (start line)
        LODI,R0 (IDX_EXP*16)+IDX_CUR
        ZBSR *VREG16_TO_REG16             ; move end (EXP) to CUR
        BSTA,UN FIND_INS                  ; TMP = first record >= start (or prog end)
        ; drop through
; =============================================================================
;  DO_LIST -- Prologue  Print stored BASIC lines, optionally filtered by range
; See DO_LIST
DLS_LP:
        ; Check TMP against program end
        ZBSR *VCMP_TMP_PE ; [+1]
        RETC,GT
        RETC,EQ
;DLS_BODY:
        ; Copy TMP -> IP, read line number hi; check against end hi (CURH)
        LODI,R0 (IDX_TMP*16)+IDX_IP
        ZBSR *VREG16_TO_REG16
        ZBSR *VFETCH16_IP                 ; [+2] EXPH:EXPL = line number; IP past the 2-byte header
        ZBSR *VINC_IP                     ; [+1] V5.7: IP past the length byte too (not printed)
        ; Check line number (EXPH:EXPL) against end (CURH:CURL)
        LODA,R0 EXPH
        SUBA,R0 CURH
        RETC,GT                         ; line hi > end hi: past range
        BCTR,LT DLS_PRNUM                 ; line hi < end hi: in range
        LODA,R0 EXPL
        SUBA,R0 CURL
        RETC,GT                           ; hi equal, line lo > end lo: past range
DLS_PRNUM:
        ZBSR *VPRINT_S16
        ZBSR *VPRT_SPACE
; V4.22: DLS_BLPX is now the detokenizer. TMPH:TMPL is idle throughout 
DLS_BLPX:
        LODA,R0 *IPH
        BCTA,EQ DLS_NL                    ; NUL: end of body (free zero test)
        COMI,R0 TOK_STMT_BASE             ; unsigned: a token (>= $80)?
        BCTA,LT DLS_ECHO                  ; < $80: ordinary char
        COMI,R0 TOK_HEX                   ; unsigned: >= $FD (constant token)?
        BCFR,LT DLS_CONST
        COMI,R0 TOK_FUNC_BASE             ; unsigned: >= $A0 (function range)?
        BCTR,LT DLS_KWTOK                 ; $80-$9F: statement range
        SUBI,R0 TOK_FUNC_BASE             ; $A0-$FC: function row index
        STRZ,R2
        LODI,R0 <FUNC_TAB
        LODI,R1 >FUNC_TAB
        ZBSR *VSET_TMP
        BCTR,UN DTT_START
DLS_KWTOK:
        SUBI,R0 TOK_STMT_BASE             ; statement row index
        STRZ,R2
        LODI,R0 <KW_TAB
        LODI,R1 >KW_TAB
        ZBSR *VSET_TMP
; DTT_* -- walk R2 rows into the table at TMPH:TMPL (each row skipped via
; scan-to-NUL then +2 past hi/lo, same shape as MATCH_KW's SE_SKIP), then
; echo the target row's text. Not O(1) (unlike MATCH_KW's dispatch) but
; LIST is not a hot path, so a walk of at most 16 rows is fine.
; In: R2 = row index; TMPH:TMPL = table base. Clobbers: R0, R1, R2.
DTT_START:
        LODI,R1 $FF                       ; pre-increment mode
DTT_SKIP:
        COMI,R2 0
        BCTR,EQ DTT_ECHO
        SUBI,R2 1
DTT_SKIPROW:
        LODA,R0 *TMPH,R1+
        BCFR,EQ DTT_SKIPROW
        ADDI,R1 2                         ; past hi/lo -> next row's text
        BCTR,UN DTT_SKIP
DTT_ECHO:
        LODA,R0 *TMPH,R1+
        BCTR,EQ DTT_DONE
        ZBSR *VCOUT
        BCTR,UN DTT_ECHO
DTT_DONE:
        ZBSR *VINC_IP                     ; past the 1-byte token in the
        ZBRR *VDLS_BLPX ;   ACTUAL body (separate from the
                                          ;   TMPH:TMPL table walk above)
DLS_CONST:
        STRZ,R2                           ; R2 = TOK_HEX or TOK_DEC (from R0)
        ZBSR *VINC_IP                     ; past the token
        ZBSR *VFETCH16_IP                 ; [+2] EXPH:EXPL = payload; IP past it
        COMI,R2 TOK_HEX
        BCTR,EQ DLS_HEXFMT
        ZBSR *VPRINT_S16                  ; TOK_DEC: signed decimal
        ZBRR *VDLS_BLPX
DLS_HEXFMT:
        LODI,R0 '$'
        ZBSR *VCOUT
        LODA,R0 EXPH
        BSTA,UN PRINT_HEX_BYTE
        LODA,R0 EXPL
        BSTA,UN PRINT_HEX_BYTE
        ZBRR *VDLS_BLPX
DLS_ECHO:
        ZBSR *VCOUT
        ZBSR *VINC_IP
        ZBRR *VDLS_BLPX
DLS_NL:
        ZBSR *VINC_IP                     ; skip over the NUL
        BSTA,UN PRT_CRLF                  ; explicit visual CR/LF for the display
        ; Update TMP from IP for next iteration
        LODI,R0 (IDX_IP*16)+IDX_TMP
        ZBSR *VREG16_TO_REG16
        BCTA,UN DLS_LP

; =============================================================================
;  MUL16 -- Signed 16-bit multiply: TMPH:TMPL * EXPH:EXPL -> EXPH:EXPL
; v5.3: shift-and-add (was O(|TMP|) repeated addition).
; Method: MSB-first (Horner).  acc = 0; for each multiplier bit, MSB first:
;   acc = 2*acc; if bit then acc += |TMP|.  V5.6: the multiplier is processed one
;   BYTE at a time out of R0 (MU_BYTE), high byte first and only if it is non-zero,
;   and the operand with the smaller high byte is made the multiplier.  Low 16 bits
;   kept (two's complement wrap, same as before).
;   acc lives in the ALTERNATE bank (R1=lo, R2=hi) so primary R2 (variable
;   letter) and R3 (SW stack ptr) are untouched; R3' is the pass counter.
;   WC=1 chains carry through ADDA/RRL pairs.
; In:  TMPH:TMPL = left operand (multiplicand); EXPH:EXPL = right operand
; Out: EXPH:EXPL = product; TMP = |one operand| (swapped if only the right was >= 256)
; Clobbers: R0, R1, NEGFLG, TMPH, TMPL (abs), PSL C/CC.  RS,WC restored to 0.
; RAS: +1 (SETUP_MULDIV) +1 (ABS_X), NEG_EXP at MU_DONE: never deeper than
;   v5.2.  Loop calls nothing.
MUL16:
        BSTA,UN SETUP_MULDIV             ; [+1] NEGFLG, |TMP|, |EXP|
        ; V5.6: the multiplier should be the operand with the fewer significant bytes.
        ; EXP >= 256 but TMP < 256: swap, so the multiplier is one byte and half the passes go.
        LODA,R0 EXPH
        BCTR,EQ MU_NOSWAP                ; multiplier already < 256
        LODA,R0 TMPH
        BCFR,EQ MU_NOSWAP                ; both >= 256: nothing to gain
        LODA,R1 TMPL
        LODA,R0 EXPL
        STRA,R0 TMPL
        STRA,R1 EXPL                     ; TMPL <-> EXPL
        LODA,R1 EXPH
        STRA,R1 TMPH                     ; TMPH = old EXPH (TMPH was 0, so EXPH becomes 0)
        EORZ,R0
        STRA,R0 EXPH
MU_NOSWAP:
        PPSL PSW_RS + PSW_WC             ; alt bank R1-R3 (acc lo/hi, pass counter), WC=1
        EORZ,R0
        STRZ,R1                          ; acc lo = 0
        STRZ,R2                          ; acc hi = 0
        LODA,R0 EXPH
        BCTR,EQ MU_LO                    ; high byte 0: its 8 leading zero bits are skipped
        BSTR,UN MU_BYTE                  ; [+1] multiplier high byte
MU_LO:
        LODA,R0 EXPL
        BSTR,UN MU_BYTE                  ; [+1] multiplier low byte
        STRA,R1 EXPL                     ; EXP = acc
        STRA,R2 EXPH
        CPSL PSW_RS + PSW_WC             ; back to primary bank, WC=0
; -----------------------------------------------------------------------------
;  MU_DONE -- shared MUL16/DIV16 epilogue: apply result sign
; In:  EXPH:EXPL = |result|; NEGFLG = 0 if result negative; RS=WC=0
; Out: EXPH:EXPL = signed result.  (No NEGFLG cleanup: PARSE_S16 and SETUP_MULDIV
;      both write it unconditionally before NEG_EXP ever reads it.)
; Clobbers: R0, R1
MU_DONE:
        BCTA,UN NEG_EXP

; -----------------------------------------------------------------------------
;  MU_BYTE -- 8 Horner passes over the multiplier byte in R0 (MSB first): acc = 2*acc,
;  and acc += |TMP| when the bit is set.  The multiplier lives in R0 (common to both
;  banks) and is tested with IORZ, so a pass costs 7 instructions (11 with an add)
;  instead of 19 when it was kept in memory.  Shifting the multiplier with C != 0 only
;  puts a stray bit in bit 0, which cannot reach the MSB within 8 passes.
; In:  R0 = multiplier byte; alt bank R1:R2 = acc lo:hi; WC=1, RS=1
; Out: acc updated.  Clobbers: R0, alt R3, PSL C/CC.  Leaf.
MU_BYTE:
        LODI,R3 8
MU_LP:
        CPSL $01                         ; C=0
        RRL,R1                           ; acc *= 2
        RRL,R2
        IORZ,R0                          ; CC=LT iff the multiplier's MSB is set
        BCFR,LT MU_NX                    ; bit 0: no add
        CPSL $01
        ADDA,R1 TMPL                     ; acc += |TMP|
        ADDA,R2 TMPH
MU_NX:
        RRL,R0                           ; multiplier <<= 1
        BDRR,R3 MU_LP
        RETC,UN

; =============================================================================
;  SETUP_MULDIV -- Common sign preamble for MUL16 and DIV16
; Sets NEGFLG = $80 (leave sign), then takes |TMP| and |EXP|, toggling NEGFLG
; for each negative operand.  No copies or clears: MUL16/DIV16 work in place (v5.3).
; In:  TMPH:TMPL = left operand; EXPH:EXPL = right operand
; Out: TMP = |TMP|; EXP = |EXP|; NEGFLG = 0 if the result is to be negated (exactly
;      one operand negative), else $80
; Clobbers: R0, R1, NEGFLG, TMPH, TMPL, EXPH, EXPL
; RAS: costs 1 level, ABS_X 1 more.  ABS_EXP is a tail jump (no extra level).
SETUP_MULDIV:
        LODI,R0 $80
        STRA,R0 NEGFLG                   ; $80 = leave sign; toggled once per negative operand
        LODI,R1 TMPH-IPH
        BSTA,UN ABS_X                    ; [+1] |TMP|
        BCTA,UN ABS_EXP                  ; tail: |EXP|

; -----------------------------------------------------------------------------
;  JERRDIVZER -- divide-by-zero exit (DIV16 only); does not return
; In: none.  Out: ERR_DIV_ZERO raised via DO_ERROR.  Clobbers: R0
JERRDIVZER:
        LODI,R0 ERR_DIV_ZERO
        ZBRR *VDO_ERROR 

; =============================================================================
;  DIV16 -- Signed 16-bit divide: TMPH:TMPL / EXPH:EXPL -> EXPH:EXPL
; Remainder left in TMPH:TMPL (magnitude, i.e. |dividend| mod |divisor|, as in
; v5.2: only the quotient takes the sign).
; v5.3: restoring shift-subtract (was O(quotient) repeated subtraction).
; Method: 32-bit chain [rem | EXP] shifted LEFT one bit per pass (EXP holds
;   |dividend|, its top bit enters rem; vacated bit0 becomes the quotient
;   bit).  rem -= |divisor| (SC); no borrow -> keep, set quotient bit;
;   borrow -> add back.  rem < |divisor| <= 32768 always, so 2*rem+1 never
;   overflows 16 bits.  16 passes, or 8 when |dividend| < 256 (SKIP8: the 8
;   leading zero bits would only shift zeros into rem).
;   rem lives in ALTERNATE bank R1(lo)/R2(hi), R3' = pass counter.
; In:  TMPH:TMPL = dividend; EXPH:EXPL = divisor
; Out: EXPH:EXPL = quotient; TMPH:TMPL = remainder; SC0:SC1 = |divisor| (restoring
;      loop only; the power-of-two fast path leaves SC0 = remainder lo, SC1 untouched)
; Clobbers: R0, R1, NEGFLG, SC0, SC1, TMP, EXP, PSL C/CC.  RS,WC restored to 0.
; Error: divisor=0 -> ERR_DIV_ZERO
; RAS: max +3 (TMP_TO_EXP / REG16_TO_REG16 are leaf) -- same as v5.2 worst case.
DIV16:
        LODA,R0 EXPH
        BCFR,EQ DV_NZ                    ; check for zero
        LODA,R0 EXPL
        BCTR,EQ JERRDIVZER

        ; not zero
DV_NZ:
        BSTR,UN SETUP_MULDIV             ; [+1] NEGFLG, TMP=|N|, EXP=|D|
        ; ---- V5.6: power-of-two divisor fast path (still shift-based) ----
        ; |D| = 2^k (1..128): quotient = |N| >> k, remainder = |N| & (|D|-1).  The shifts
        ; run in alternate-bank registers (3 instructions per bit) and |D| itself is
        ; shifted down to 1 to count k.  |D| = 256 is a byte move.  Any other divisor
        ; takes the restoring loop below, unchanged.
        LODA,R0 EXPH
        BCFA,EQ DV_BIG                   ; |D| >= 256
        LODA,R1 EXPL                     ; R1 = |D| (1..255)
        LODZ,R1
        SUBI,R0 1                        ; R0 = |D|-1
        ANDZ,R1                          ; (|D|-1) & |D| == 0  <=>  power of two
        BCFA,EQ DV_SLOW
        LODZ,R1
        SUBI,R0 1
        ANDA,R0 TMPL                     ; remainder = |N| & (|D|-1) (0..127; hi byte is 0)
        STRA,R0 SC0                      ; park it (SC0 is scratch on this path)
        PPSL PSW_RS + PSW_WC             ; alt bank R1-R3, WC=1
        LODA,R1 TMPH                     ; R1' = |N| hi
        LODA,R2 TMPL                     ; R2' = |N| lo
        LODA,R3 EXPL                     ; R3' = |D|
DVP_LP:
        COMI,R3 1
        BCTR,EQ DVP_DONE                 ; |D| shifted down to 1: done
        CPSL $01                         ; C=0: a 0 enters the top bit
        RRR,R1
        RRR,R2                           ; |N| >>= 1 (the carry chains hi -> lo)
        CPSL $01
        RRR,R3                           ; |D| >>= 1
        BCTR,UN DVP_LP
DVP_DONE:
        STRA,R1 EXPH                     ; quotient = |N| >> k
        STRA,R2 EXPL
        CPSL PSW_RS + PSW_WC             ; back to primary bank, WC=0
        LODA,R0 SC0
        STRA,R0 TMPL                     ; remainder (magnitude)
        EORZ,R0
        STRA,R0 TMPH
        BCTA,UN MU_DONE                  ; sign on the quotient
DV_BIG:
        COMI,R0 1
        BCFR,EQ DV_SLOW                  ; EXPH != 1
        LODA,R0 EXPL
        BCFR,EQ DV_SLOW                  ; |D| != 256
        LODA,R0 TMPH                     ; |D| = 256: quotient = |N| hi, remainder = |N| lo
        STRA,R0 EXPL
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 TMPH
        BCTA,UN MU_DONE
DV_SLOW:
        LODI,R0 (IDX_EXP*16)+IDX_SC0
        ZBSR *VREG16_TO_REG16            ; SC0:SC1 = |D|
        ZBSR *VTMP_TO_EXP                ; EXP = |N| (dividend, becomes quotient)
        PPSL PSW_RS + PSW_WC             ; alt bank R1-R3, WC=1
        LODI,R3 16
        EORZ,R0
        STRZ,R1                          ; rem lo = 0
        STRZ,R2                          ; rem hi = 0
        LODA,R0 EXPH                     ; SKIP8: dividend < 256 ?
        BCFR,EQ DV_LP                    ;   no: full 16 passes
        LODA,R0 EXPL                     ;   yes: EXP <<= 8, 8 passes
        STRA,R0 EXPH
        EORZ,R0
        STRA,R0 EXPL
        LODI,R3 8
DV_LP:
        CPSL $01                         ; C=0: bit shifted into EXP bit0
        LODA,R0 EXPL
        RRL,R0
        STRA,R0 EXPL
        LODA,R0 EXPH
        RRL,R0
        STRA,R0 EXPH                     ; C = old dividend top bit
        RRL,R1                           ; rem = rem*2 + C
        RRL,R2
        PPSL $01                         ; C=1: SUB without borrow-in
        SUBA,R1 SC1                      ; rem -= |D|
        SUBA,R2 SC0                      ; C=1 <=> no borrow
        TPSL $01
        BCTR,EQ DV_Q1                    ; no borrow: keep, quotient bit = 1
        CPSL $01
        ADDA,R1 SC1                      ; borrow: restore rem
        ADDA,R2 SC0
        BCTR,UN DV_NX
DV_Q1:
        LODA,R0 EXPL
        IORI,R0 1
        STRA,R0 EXPL
DV_NX:
        BDRR,R3 DV_LP
        STRA,R1 TMPL                     ; remainder -> TMP
        STRA,R2 TMPH
        CPSL PSW_RS + PSW_WC             ; back to primary bank, WC=0
        BCTA,UN MU_DONE                  ; NEG_EXP on quotient, clear NEGFLG

; =============================================================================
;  PRINT_S16 -- Print signed 16-bit value EXPH:EXPL as decimal
; In:  EXPH:EXPL = signed value
; Out: decimal digits written to COUT
; Clobbers: R0, R1, R2, R3, SC0 (TMP is NOT clobbered)
PRINT_S16:
        LODA,R0 EXPH             ; get high byte & establish CC
        BCFR,LT PS_DIGITS        ; not negative
        LODI,R0 A'-'
        ZBSR *VCOUT
        BSTA,UN NEG_EXP_BODY     ; Negate, making EXPH:EXPL positive
PS_DIGITS:
        EORZ,R0
        STRZ,R3                  ; R3 = OR of digit counts so far (0 = still leading zeros)
        LODI,R2 5                ; R2 = column counter 5..1; P10 tables are indexed R2-1
DIGIT_LOOP:
        PPSL $09                 ; WC=1, C=1 (no borrow in): 16-bit subtract chains via carry
        LODI,R1 0                ; R1 = digit count
SUB_LOOP:
        LODA,R0 EXPL
        SUBA,R0 P10_LO-1,R2      ; Subtract low byte (C=1 on entry: first pass or no borrow last pass)
        STRA,R0 SC0              ; Save tentatively (PRINT_S16 must not
                                 ; clobber TMP -- DO_LIST relies on that)
        LODA,R0 EXPH
        SUBA,R0 P10_HI-1,R2      ; Subtract high byte with the low byte's borrow
        BCTR,LT BORROW           ; Borrow (C=0): result LT, stop subtracting

        ; No borrow: commit result and subtract again
        STRA,R0 EXPH
        LODA,R0 SC0
        STRA,R0 EXPL
        BIRR,R1 SUB_LOOP         ; digit++ (flags untouched); never wraps to 0, always loops

BORROW:
        CPSL $08                 ; WC=0 from here, also for COUT (its RRR would chain carry)
        LODZ,R1
        IORZ,R3
        STRZ,R3                  ; R3 |= digit count; CC=EQ while every digit so far is zero
        BCFR,EQ PRINT_IT         ; non-zero digit seen: print
        COMI,R2 1                ; final (1s) column?
        BCFR,EQ NEXT_DIG         ; no: skip the leading zero
PRINT_IT:
        LODZ,R1
        ADDI,R0 A'0'
        ZBSR *VCOUT              ; Print the character
NEXT_DIG:
        BDRR,R2 DIGIT_LOOP       ; next column down to the 1s
        RETC,UN                  ; Return to caller

; =============================================================================
;  SWRETURN -- Return via SW stack
; In:  R3 = SW stack pointer; SWBASE[R3] = hi, SWBASE[R3-1] = lo of return addr
; Out: jumps to address popped from SW stack
; Clobbers: R0, TEMPRETH, TEMPRETL
SWRETURN:
        LODA,R0 SWBASE,R3
        STRA,R0 TEMPRETH
        LODA,R0 SWBASE,R3-
        STRA,R0 TEMPRETL
        SUBI,R3 1
        BCTA,UN *TEMPRETH

P10_HI:                          ; powers of 10 (1, 10, 100, 1000, 10000), indexed R2-1
        db $00, $00, $00, $03, $27
P10_LO:
        db $01, $0A, $64, $E8, $10

; =============================================================================
;  RDLINE -- Read a line from input into IBUF with echo and backspace
; In:  nothing
; Out: IBUF = NUL-terminated input line; IPH:IPL = IBUF (both callers re-pointed IP
;      with a separate SET_IP_IBUF call right after -- folded in here).
;      R3 is used as an index into IBUF instead (cheaper bounds check and
;      backspace than full 16-bit pointer arithmetic). R3=$FF means empty
;      (matches the SWBASE empty-stack convention) since the 2650's ",R3+"
;      addressing mode pre-increments before the access.
; Clobbers: R0, R1, R3
RDLINE:
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
        LODI,R3 $FF                      ; R3 = empty-buffer sentinel (pre-inc convention)
RL_LP:
        BSTA,UN PRE_CHIN                     ; [+1] blocking read
        STRZ,R1
        COMI,R1 BS
        BCTR,EQ RL_BS
        COMI,R1 CR+1
        BCTR,LT RL_EOL
        ; buffer full check: room while R3 < 62 (last slot reserved for NUL)
        ; V4.14 COM01: R3=$FF is the empty-buffer sentinel, relying on signed
        ; interpretation (-1 < 62); must run under COM=0 despite the global
        ; COM=1 set in MAIN, or the first char of every line gets dropped.
        CPSL $02
        COMI,R3 62
        PPSL $02
        BCFR,LT RL_LP
;RL_STORE:
        LODZ,R1                          ; R0 = char (indexed store always uses R0)
        STRA,R0 IBUF,R3+                 ; R3++ (pre-inc); IBUF[R3]=char
        ZBSR *VCOUT  
        BCTR,UN RL_LP
RL_BS:
        COMI,R3 $FF
        BCTR,EQ RL_LP                    ; empty: ignore backspace
        SUBI,R3 1
        BSTA,UN PRT_BS
        ZBSR *VPRT_SPACE  
        BSTA,UN PRT_BS
        BCTR,UN RL_LP
RL_EOL:
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
        SUBI,R0 A'A'                     ; shift 'A' down to 0 (COM=1: one unsigned test)
        COMI,R0 A'Z'-A'A'
        BCFR,GT EW_ADV                   ; A-Z: consume
        COMI,R0 A'$'-A'A'                ; '$' in the same shifted frame (wraps mod 256)
        BCFR,EQ WSKIPRET
EW_ADV:
        ZBSR *VINC_IP 
        BCTR,UN EATWORD

; =============================================================================
;  WORD_START -- can the text at IP be a multi-letter keyword/function word?
;  Every table row is 2+ letters, so a single letter (a variable) or anything
;  that does not start with two letters (constant, number, token) can never
;  match: callers use this to skip the table scan / FT save machinery.
; In:  IPH:IPL -> char (after WSKIP)
; Out: CC=EQ: two letters A-Z in a row (maybe a word).  CC=GT: no.
; Clobbers: R0 and CC only -- works in register bank 1, so R1-R3 survive.
; RAS: leaf.  Relies on the global COM=1 (unsigned) set in MAIN.
WORD_START:
        PPSL PSW_RS
        LODI,R1 $FF                      ; pre-increment index
WS_LP:
        LODA,R0 *IPH,R1+
        SUBI,R0 A'A'
        COMI,R0 A'Z'-A'A'
        BCTR,GT WS_RET                   ; not a letter: CC=GT
        COMI,R1 1
        BCFR,EQ WS_LP                    ; test the 2nd char too
WS_RET:
        CPSL PSW_RS                      ; CPSL leaves CC alone
        RETC,UN

; =============================================================================
;  WSKIP -- Skip spaces at IP
; In:  IPH:IPL -> current position
; Out: IPH:IPL -> first non-space char
; Clobbers: R0
WSKIP_LOOP:
        ZBSR *VINC_IP           ; Advance IP (2 bytes)
WSKIP:
        LODA,R0 *IPH            ; Read char at IP (3 bytes)
        COMI,R0 SP              ; Is it a space? (2 bytes)
        BCTR,EQ WSKIP_LOOP      ; Yes -> loop back to increment IP (2 bytes)
        IORZ,R0                 ; Refresh CC flags for R0 (1 byte; EQ iff NUL)
WSKIPRET:
        RETC,UN                 ; Return to caller (1 byte)

; =============================================================================
;  SHARED 16-BIT POINTER INCREMENT  - INC_ET family
; INC_EXP : EXPH:EXPL += 1   (offset EXPH-IPH from IPH)
; INC_TMP : TMPH:TMPL += 1   (offset TMPH-IPH from IPH)
; INC_IP  : IPH:IPL  += 1    (dedicated fast path above; INC_ET with R0=0 also works)
; INC_EXP/INC_TMP share the INC_ET body using register bank switch.
; Rule: NO BSTA inside these -- must not consume extra RAS depth.
; Offsets are assembly-time expressions (e.g. EXPH-IPH=4) -- sequential
; ordering of the IPH..LNUML block must be preserved or these silently break.
; =============================================================================
;  INC_IP -- IPH:IPL += 1   (dedicated fast path; ~4 instructions vs ~10 via
;  the generic INC_ET body, and IP is advanced from nearly every parse loop)
; In:  IP.  Out: IP += 1.  Clobbers: R0, CC.  Leaf (no RAS use).
; Wrap of IPL is the only case that reaches the hi-byte increment: after
; ADDI the CC is EQ only for a zero result, so both non-zero returns fall out.
INC_IP:
        LODA,R0 IPL
        ADDI,R0 1
        STRA,R0 IPL
        RETC,GT                          ; result 1..127: no wrap
        RETC,LT                          ; result 128..255: no wrap
        LODA,R0 IPH                      ; result 0: carry into the hi byte
        ADDI,R0 1
        STRA,R0 IPH
        RETC,UN

INC_EXP:
        LODI,R0 EXPH-IPH        ; EXP offset from IPH (= 4); assembly-time expression
        db $EC                  ; COMA,R0 -- consume next 2 bytes (skip to INC_ET)
INC_TMP:
        LODI,R0 TMPH-IPH        ; TMP offset from IPH (= 2); assembly-time expression
; Can jump in here with R0 set for offset (INC_ET with R0=0 still works for IP)
INC_ET:
        PPSL PSW_RS                 ; switch to alternate register bank
        STRZ R1                 ; R1 = offset
        LODA,R0 IPL,R1          ; load lo byte
        ADDI,R0 1
        STRA,R0 IPL,R1
        BCFR,EQ ET_RET          ; ADDI result != 0: no carry out of the low byte, done
        LODA,R0 IPH,R1          ; carry: increment hi byte
        ADDI,R0 1
        STRA,R0 IPH,R1
ET_RET:
        CPSL PSW_RS                 ; switch back to primary bank
        RETC,UN

; =============================================================================
;  EXP_TO_LNUM / TMP_TO_EXP -- REG16_TO_REG16 with the packed byte preloaded.
;  Entered by ZBSR/ZBRR *VEXP_TO_LNUM / *VTMP_TO_EXP (2 bytes) instead of
;  LODI,R0 + ZBSR *VREG16_TO_REG16 (4 bytes). Only pairs with 3+ sites pay:
;  a shim costs 3 bytes (LODI + DB $EC; 2 for the last) plus a 2-byte vector.
; In:  none of its own (R0 is loaded here); WC must be 0 as for REG16_TO_REG16
; Out: EXP_TO_LNUM: LNUMH:LNUML = EXPH:EXPL      TMP_TO_EXP: EXPH:EXPL = TMPH:TMPL
;      (source unchanged; CC as REG16_TO_REG16)
; Clobbers: R0 (and CC).  RAS: same as REG16_TO_REG16.
; =============================================================================
EXP_TO_LNUM:
        LODI,R0 (IDX_EXP*16)+IDX_LNUM
        DB $EC                          ; COMA,R0: swallows the next LODI
TMP_TO_EXP:
        LODI,R0 (IDX_TMP*16)+IDX_EXP
        ; drop through
; =============================================================================
;  REG16_TO_REG16 -- generic 16-bit copy between any two IPH-relative RAM
;  register pairs, addressed by a packed nibble pair.
; In:  R0 = (SRC_IDX<<4)|DST_IDX; each idx*2 = byte offset of the pair's hi
;      byte from IPH.  Use the IDX_* EQUs, e.g.
;          LODI,R0 (IDX_EXP*16)+IDX_TMP
;          ZBSR *VREG16_TO_REG16            ; TMP = EXP
;      Valid idx 0-15 (offsets 0-30); pair = hi byte then lo byte.
;      WC (PSL bit 3) must be 0 so RRR/RRL are plain 8-bit rotates.
; Out: destination pair = source pair (source unchanged).
;      CC = flags of the last byte loaded (the source low byte).
; Clobbers: R0 (and CC).  Primary-bank R1/R2/R3 preserved (alternate bank
;      used, restored by CPSL PSW_RS).
; RAS: leaf routine -- no BSTA inside, costs exactly one level at the caller.
; =============================================================================
REG16_TO_REG16:
        PPSL PSW_RS             ; switch to alternate register bank
        STRZ,R2                 ; alt-R2 = R0 (packed byte SSSSDDDD)
        ANDI,R2 $0F             ; isolate dest nibble -> R2 = 0000DDDD
        EORZ,R2                 ; R0 ^= R2 -> SSSS0000 (cancels the low nibble)
        RRR,R0                  ; /2
        RRR,R0                  ; /2
        RRR,R0                  ; /2 -> R0 = source offset (SSSS*2)
        STRZ,R1                 ; alt-R1 = source offset
        RRL,R2                  ; *2 -> alt-R2 = dest offset (DDDD*2)
        LODA,R0 IPH,R1
        STRA,R0 IPH,R2
        LODA,R0 IPL,R1
        STRA,R0 IPL,R2
        CPSL PSW_RS             ; restore primary bank
        RETC,UN

; =============================================================================
;  FETCH16_IP -- fetch the 16-bit word at [IP] (hi byte first) into EXPH:EXPL
;  and advance IP past it.
; In:  IPH:IPL -> hi byte of the word
; Out: EXPH:EXPL = the word; IPH:IPL += 2
; Clobbers: R0, EXPH:EXPL (INC_IP works in the alternate bank; primary R1-R3 kept)
; RAS: +2 when ZBSR'd (INC_IP nests inside); +1 when reached by ZBRR tail.
; =============================================================================
FETCH16_IP:
        LODA,R0 *IPH                    ; word hi
        STRA,R0 EXPH
        ZBSR *VINC_IP                   ; [+1]
        LODA,R0 *IPH                    ; word lo
        STRA,R0 EXPL
        ZBRR *VINC_IP                   ; tail: IP past the word, return to caller

; =============================================================================
;  JERRVAR -- Error with variable
;  JSYNERR -- Syntax error jump
;  DO_RETURN -- underflow error jump
; In:  nothing (R0 irrelevant)
; Out: jumps to DO_ERROR 
; Clobbers: R0
JERR_NXT:
        LODI,R0 ERR_NXT
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to BCTA
JERRVAR:
        LODI,R0 ERR_VAR
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to BCTA
JSYNERR:
        LODI,R0 ERR_SYN
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to BCTA
JERROOM:
        LODI,R0 ERR_OOM
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to BCTA
JFORERR:
        LODI,R0 ERR_FOR
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to BCTA
DRT_UNDERFLOW:
        LODI,R0 ERR_RET
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
        LODI,R0 (IDX_CUR*16)+IDX_EXP
        ZBSR *VREG16_TO_REG16             ; EXPH:EXPL = CURH:CURL
        ZBSR *VPRINT_S16                ; [+1]
DE_NL:
        BSTR,UN PRT_CRLF
        BSTA,UN DO_END                   ; [+1] clears SWSP, FORSP, RUNFLG
        BCTA,UN REPL                     ; REPL resets RAS (PSU SP bits) on entry

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

        ; Print the result
        ZBSR *VPRINT_S16                ; Print decimal value
        ; drop through

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
        db $EC
PRT_BS:
        LODI,R0 BS
        ZBRR *VCOUT 

; =============================================================================
;  DIGIT_CHECK -- Is *IPH a decimal digit '0'-'9'?  (uBASIC v2.7: this 7-byte test
;  was inlined 4x here - TRY_STORE_LINE, PARSE_U16 x2, TL_DEC_LP - byte-identical)
; Out: R0 = char - '0'; CC=GT if not a digit (single unsigned range test, COM=1)
; Clobbers: R0
DIGIT_CHECK:
        LODA,R0 *IPH
        SUBI,R0 A'0'
        COMI,R0 9
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
;  CHECK_POW -- power operator (^) precedence hook (v4.7, POW-01/POW-02)
; In:  EXPH:EXPL = just-parsed atom value; IPH:IPL = current parse position
; Out: EXPH:EXPL = base^exponent if '^' found and consumed, else unchanged
; Clobbers: R0, R1, R3, TMPH, TMPL, SAVEH, SAVEL, POWCNTH, POWCNTL
CHECK_POW:
        LODA,R0 *IPH
        COMI,R0 A'^'
        BCTR,EQ CP_HIT
        RETC,UN
CP_HIT:
        ZBSR *VPUSH_EXP           ; push base(EXPH:EXPL) onto SWBASE; tail
                                    ; branch inside also advances IP past '^'
        LODI,R0 >CP_EXP_RET
        LODI,R1 <CP_EXP_RET
        ZBSR *VPUSH_RET
        ZBRR *VEAM_ATOM             ; parse exponent atom -> EXPH:EXPL
CP_EXP_RET:
        BSTA,UN POP_SAVE_TO_TMP            ; [+1] TMP = base (TMP is dead mid-expression); R3 -= 2
        LODI,R0 (IDX_TMP*16)+IDX_SAVE
        ZBSR *VREG16_TO_REG16             ; [+1] SAVE = TMP
        ; SAVEH:SAVEL = base; EXPH:EXPL = exponent; SW stack balanced

        LODA,R0 EXPH               ; negative exponent -> error (undefined
        ANDI,R0 $80                 ; for an integer-only result)
        BCTR,EQ CP_EXP_OK
        LODI,R0 ERR_OV
        ZBRR *VDO_ERROR 
CP_EXP_OK:
        LODA,R1 EXPH                ; exponent == 0 -> result = 1
        LODA,R0 EXPL                 ; (standard convention, incl. 0^0 == 1)
        IORZ R1
        BCFR,EQ CP_POW_LOOP
        EORZ,R0
        STRA,R0 EXPH
        LODI,R0 1
        STRA,R0 EXPL
        BCTR,UN CP_CHAIN

CP_POW_LOOP:
        LODI,R0 (IDX_EXP*16)+IDX_POW
        ZBSR *VREG16_TO_REG16             ; POWCNT = exponent (EXPH:EXPL becomes the result
        EORZ,R0                    ; result accumulator = 1
        STRA,R0 EXPH
        LODI,R0 1
        STRA,R0 EXPL

CP_MUL_LOOP:
        LODI,R0 (IDX_SAVE*16)+IDX_TMP
        ZBSR *VREG16_TO_REG16             ; TMP = base (SAVEH:SAVEL)
        BSTA,UN MUL16

        LODA,R0 POWCNTL                ; exponent counter -= 1 (16-bit)
        SUBI,R0 1
        STRA,R0 POWCNTL
        BCFR,LT CP_CNT_NB
        LODA,R0 POWCNTH
        SUBI,R0 1
        STRA,R0 POWCNTH
CP_CNT_NB:
        LODA,R1 POWCNTH
        LODA,R0 POWCNTL
        IORZ R1
        BCFR,EQ CP_MUL_LOOP
        ; fall through: loop done

CP_CHAIN:
        LODA,R0 *IPH                   ; chained ^ is left-associative:
        COMI,R0 A'^'                    ; (2^3^2) == (2^3)^2, so loop back
        BCTA,EQ CP_HIT                  ; to CP_HIT with the current result
        RETC,UN                          ; as the new base instead of recursing

; =============================================================================
;  TABLES 
BANNER:
        DB CR, LF, "uBASIC 2650 V5.7", CR, LF, "Bytes Free:",NUL

; -- Keyword dispatch table (V4.21: full text, was 3-char prefix)
; Format: [full keyword text][NUL][hi][lo], variable-length rows, each row's
; own NUL marks where hi:lo starts. Matched in full (see MATCH_KW). This
; table is also the LIST detokenizer's text source once tokens land.
; THEN matched internally by DO_IF not here.
; Per-row KWH_xxx/FNH_xxx labels mark each row's hi byte, so
; TOK_STMT_OFS/TOK_FUNC_OFS below are computed by the assembler (never
; hand-counted) -- see MATCH_KW fast path.
KW_TAB:
        DB "END",    NUL
KWH_END:
        DB <DO_END,    >DO_END
        DB "FOR",    NUL
KWH_FOR:
        DB <DO_FOR,    >DO_FOR
        DB "FREE",   NUL
KWH_FREE:
        DB <DO_FREE,   >DO_FREE
        DB "GOSUB",  NUL
KWH_GOSUB:
        DB <DO_GOSUB,  >DO_GOSUB
        DB "GOTO",   NUL
KWH_GOTO:
        DB <DO_GOTO,   >DO_GOTO
        DB "IF",     NUL
KWH_IF:
        DB <DO_IF,     >DO_IF
        DB "INPUT",  NUL
KWH_INPUT:
        DB <DO_INPUT,  >DO_INPUT
        DB "LET",    NUL
KWH_LET:
        DB <SE_NOTKW,  >SE_NOTKW
        DB "LIST",   NUL
KWH_LIST:
        DB <DO_LIST,   >DO_LIST
        DB "NEW",    NUL
KWH_NEW:
        DB <DO_NEW,    >DO_NEW
        DB "NEXT",   NUL
KWH_NEXT:
        DB <DO_NEXT,   >DO_NEXT
        DB "POKE",   NUL
KWH_POKE:
        DB <DO_POKE,   >DO_POKE
        DB "PRINT",  NUL
KWH_PRINT:
        DB <DO_PRINT,  >DO_PRINT
        DB "REM",    NUL
KWH_REM:
        DB <DO_REM,    >DO_REM
        DB "RETURN", NUL
KWH_RETURN:
        DB <DO_RETURN, >DO_RETURN
        DB "RUN",    NUL
KWH_RUN:
        DB <DO_RUN,    >DO_RUN
KW_MISS:
        DB NUL,      <SE_NOTKW,  >SE_NOTKW   ; No match handler

; Row order here MUST match KW_TAB's row order above (token = TOK_STMT_BASE
; + row index); values are assembler-computed byte offsets from KW_TAB to
; that row's hi byte -- never hand-counted, safe if KW_TAB text ever changes.
; NOTE: '>' here means LOW byte (this assembler's convention, see file
; header) -- these are one-byte offsets, not addresses, so we want the low
; byte of the (small) difference, not its high byte (which is always 0).
TOK_STMT_OFS:
        DB >(KWH_END-KW_TAB),    >(KWH_FOR-KW_TAB),   >(KWH_FREE-KW_TAB)
        DB >(KWH_GOSUB-KW_TAB),  >(KWH_GOTO-KW_TAB),  >(KWH_IF-KW_TAB)
        DB >(KWH_INPUT-KW_TAB),  >(KWH_LET-KW_TAB),   >(KWH_LIST-KW_TAB)
        DB >(KWH_NEW-KW_TAB),    >(KWH_NEXT-KW_TAB),  >(KWH_POKE-KW_TAB)
        DB >(KWH_PRINT-KW_TAB),  >(KWH_REM-KW_TAB),   >(KWH_RETURN-KW_TAB)
        DB >(KWH_RUN-KW_TAB)

; -- Function Dispatch Table - EXP is input/output (V4.21: full text)
; note TAB and CHR$ handled by PRINT as only meaningful there
FUNC_TAB:
        DB "ABS",  NUL
FNH_ABS:
        DB <DO_ABS_FUNC,  >DO_ABS_FUNC
        DB "AND",  NUL
FNH_AND:
        DB <DO_AND_FUNC,  >DO_AND_FUNC
        DB "NEG",  NUL
FNH_NEG:
        DB <DO_NEG_FUNC,  >DO_NEG_FUNC
        DB "NOT",  NUL
FNH_NOT:
        DB <DO_NOT_FUNC,  >DO_NOT_FUNC
        DB "OR",   NUL
FNH_OR:
        DB <DO_OR_FUNC,   >DO_OR_FUNC
        DB "PEEK", NUL
FNH_PEEK:
        DB <DO_PEEK_FUNC, >DO_PEEK_FUNC
        DB "RND",  NUL
FNH_RND:
        DB <DO_RND_FUNC,  >DO_RND_FUNC
        DB "USR",  NUL
FNH_USR:
        DB <DO_USR_FUNC,  >DO_USR_FUNC
        DB "XOR",  NUL
FNH_XOR:
        DB <DO_XOR_FUNC,  >DO_XOR_FUNC
FUNC_MISS:
        DB NUL,    <PE_NOFUNC,   >PE_NOFUNC        ; No match handler

; Row order MUST match FUNC_TAB's row order above (token = TOK_FUNC_BASE +
; row index); assembler-computed, see TOK_STMT_OFS note above.
TOK_FUNC_OFS:
        DB >(FNH_ABS-FUNC_TAB), >(FNH_AND-FUNC_TAB), >(FNH_NEG-FUNC_TAB)
        DB >(FNH_NOT-FUNC_TAB), >(FNH_OR-FUNC_TAB),  >(FNH_PEEK-FUNC_TAB)
        DB >(FNH_RND-FUNC_TAB), >(FNH_USR-FUNC_TAB), >(FNH_XOR-FUNC_TAB)

FUNC_TOKS_END:
ROMEND: 

;  RAM variables -- sequential RES block 
 
        ORG     4096    ; half a 2650 8kbyte page

; --- Ordered group (V4.25): 16-bit register pairs at IDX_xx*2 bytes from IPH.
;     Used by INC_ET/NEG_SHARED (offset = XH-IPH, assembler-computed) and
;     by REG16_TO_REG16's packed-nibble scheme (IDX_* EQUs near the top).
;     Every pair is hi byte then lo byte on an EVEN offset with idx <= 15.
;     The IDX_* EQUs MUST match this order: reorder both together.
IPH     RES 1       ; interpreter pointer hi       (offset  0, IDX_IP=0)
IPL     RES 1       ; interpreter pointer lo
TMPH    RES 1       ; temp 16-bit hi               (offset  2, IDX_TMP=1)
TMPL    RES 1       ; temp 16-bit lo
; Slot 2 (offset 4-5) holds two single bytes, no pair (no IDX). FORVAR must stay
; immediately before LNUMH: DO_FOR/DO_NEXT copy FORVAR,LNUM,EXP,SWSTK (7 bytes) as-is.
RUNFLG  RES 1       ; $01=running $00=immediate (took GOTOFLG's slot: no longer needed)
FORVAR  RES 1       ; FOR loop var's precomputed VARS offset (V4.19; was A-Z letter)
LNUMH   RES 1       ; scratch line number hi       (offset  6, IDX_LNUM=3)
LNUML   RES 1       ; scratch line number lo
EXPH    RES 1       ; expression result hi         (offset  8, IDX_EXP=4)
EXPL    RES 1       ; expression result lo
SWSTK   RES 2       ; next-line pointer cache [NLP_H][NLP_L] written by DR_EXEC
                    ;                              (offset 10, IDX_SWSTK=5)
GOTOH   RES 1       ; pending target hi            (offset 12, IDX_GOTO=6)
GOTOL   RES 1       ; pending target lo
CURH    RES 1       ; current line hi  (error reporting)  (offset 14, IDX_CUR=7)
CURL    RES 1       ; current line lo
SC0     RES 1       ; Scratch byte 0               (offset 16, IDX_SC0=8)
SC1     RES 1       ; Scratch byte 1
PEH     DB <SHOWCASE_END       ; Program end pointer hi   (offset 18, IDX_PE=9)
PEL     DB >SHOWCASE_END       ; Program end pointer lo
TKH     RES 1       ; V4.24: TOKENIZE_LINE's own scratch pointer pair (KW_TAB/
TKL     RES 1       ;   FUNC_TAB scan pointer, and TL_MULADD's scale temp).
                     ;   Dedicated so TMPH:TMPL -- TRY_STORE_LINE's live
                     ;   insertion point -- survives the call with no
                     ;   save/restore (same idea as ARGAH/ARGAL, PARSE_2ARGS).
                     ;   (offset 20, IDX_TK=10)
ARGAH   RES 1       ; PARSE_2ARGS: arg 'a' scratch (hi); NOT TMPH - PARSE_EXPR
ARGAL   RES 1       ; itself clobbers TMPH:TMPL while parsing arg 'b'  (offset 22, IDX_ARGA=11)
POWCNTH RES 1       ; CHECK_POW: exponent down-counter (hi)  (offset 24, IDX_POW=12)
POWCNTL RES 1       ; CHECK_POW: exponent down-counter (lo)
SAVEH   RES 1       ; ADD16_SAVE_EXP/CHECK_POW: popped left operand scratch (hi)
SAVEL   RES 1       ; ...(lo)                      (offset 26, IDX_SAVE=13)
RNDSEED RES 2       ; 16 bit Random                (offset 28, IDX_RND=14)
VARS    RES 52      ; A-Z variables 2 bytes each.  Kept within 255 bytes of IPH so
                    ;   ADD_CORE can reach VARS[n] as VARS-IPH+2n (DO_NEXT)

; --- Remaining (not nibble-indexed) ---
FUNCOP  RES 1       ; PARSE_2ARGS: AND/OR/XOR op selector
FT_SP   RES 1       ; FUNCATOM-01 (v4.6): FT_STK pointer ($FF=empty)
FT_STK  RES 4       ; FUNCATOM-01: per-level dispatch-origin marker, pushed
                     ; by PE_SAFE ($FF=top-level) or EAM_ATOM (live R3
                     ; snapshot N), popped by PE_NOFUNC/FUNC_EPILOG. 4
                     ; levels = 4 nested function-call arguments deep - see
                     ; PE_SAFE/EAM_ATOM/FUNC_EPILOG headers for the scheme.
FT_SAVE_SP RES 1    ; FUNCATOM-01: FT_SAVE byte-stack pointer ($FF=empty)
FT_SAVE RES 64      ; FUNCATOM-01: saved SWBASE[0..N] bytes, one contiguous
                     ; LIFO byte-stack shared across nesting levels (not
                     ; fixed per-level slots - see EAM_ATOM/FUNC_EPILOG).
                     ; 64 bytes is generous headroom over SWBASE's own 32.
FT_N    RES 1       ; FUNCATOM-01: scratch - N handed within FUNC_EPILOG
FT_R2SAVE RES 1     ; FUNCATOM-01: scratch - caller's R2 preserved across
                     ; the EAM_ATOM save / FUNC_EPILOG restore byte-copy
                     ; (R2 is reserved for SE_NOTKW/DO_FOR/DO_INPUT across
                     ; PARSE_EXPR - see header - so must never leak here)
TEMPRETH RES 1      ; SW return address hi
TEMPRETL RES 1      ; SW return address lo

;  --- Flags & Stuff --- 
R3SAVE  RES 1       ; Save/restore R3 across PARSE_U16 multiply loop
NEGFLG  RES 1       ; Sign flag
FORSP   RES 1       ; FOR stack pointer (0=empty, V4.19; 0/7/14/21=frame offsets)
SWSP    RES 1       ; GOSUB stack pointer (0=empty, V4.19)
RELOP   RES 1       ; Relational op bitmask: bit0=LT bit1=EQ bit2=GT

;  SW call stack -- used by PARSE_EXPR / PRINT_S16 only
; R3 = index ($FF=empty, grows up). Each frame = [lo][hi].
; Push: STRA,R0 *SWBASE,R3+ (lo first), STRA,R0 *SWBASE,R3+ (hi).
; Pop:  LODA,R0 *SWBASE,R3- (hi first), LODA,R0 *SWBASE,R3- (lo).
SWBASE  RES 32      ; SW stack base: 32 bytes

;  GOSUB stack (v3.2) -- managed by SWSP
GSBASE  RES 16      ; GOSUB stack base: 16 bytes

;  FOR/NEXT stack (v3.3) -- managed by FORSP
FORBASE RES 28      ; FOR stack base: 28 bytes

; Buffers
IBUF    RES 64      ; Input buffer 64 bytes
TOKBUF  RES 128     ; V4.22: TOKENIZE_LINE's output scratch (tokenized body,
                    ; NUL-terminated); TRY_STORE_LINE copies it into the
                    ; program store.
                    ; V4.23: was 80, which overflowed into VARS (clobbering
                    ; A,B,...) on dense-constant lines. Worst case: IBUF holds
                    ; 63 chars, so at most 62 body chars of "1;1;1;..." =
                    ; 31 constants x (3-byte token + 1 separator) = 124 bytes
                    ; + NUL = 125. 128 covers it with no bounds check needed.
                    ; (Costs 48 bytes of program space vs the old 80.)

; V5.7(PROTO): backward-GOTO/GOSUB line cache -- 2 slots x [lnH][lnL][ptrH][ptrL],
; reset every RUN (see DO_RUN). Checked inline at DG_TOP (0 extra RAS).
GCACHE  RES 8
GC_NEXT RES 1       ; round-robin next-slot write offset (0 or 4)
TSL_BLEN RES 1      ; V5.7: TRY_STORE_LINE's saved body length (TOKENIZE_LINE's
                    ;   R3, held here while R3 is repurposed for OPEN_GAP/copy)

; =============================================================================
;  Pre-loaded SHOWCASE program
;
;  Line format: <lineno_hi> <lineno_lo> <len> <tokenized body> <NUL>
;  (V4.29: pre-tokenized, the same bytes TOKENIZE_LINE stores for typed lines.
;  V5.7: <len> = body byte count, NUL not counted, TOK_DEC/TOK_HEX count 3 -- the
;  byte TRY_STORE_LINE stores for typed lines; ADV_TMP_PAST_REC relies on it.
;  GENERATED by tools/tokshowcase.py from showcase.bas, then tools/showcase_addlen.py
;  convert adds <len> -- do not hand-edit; 'showcase_addlen.py verify' re-checks it.)
;  Lines  10-190: feature demos (PRINT, CHR$, arithmetic, comparisons, GOTO loop)
;  Lines 192-218: FOR/NEXT and GOSUB/RETURN demos
;  Lines 220-240: function demos (ABS/NEG/AND/OR/XOR/NOT/PEEK/POKE/RND/LIST)
;  Lines 300-510: Mandelbrot set renderer (v4.3: widened, C=-144..28 step 4,
;                 44 cols vs v4.2's 32; row range I=-64..56 step 6 unchanged)
;  Line  530:     GOSUB subroutine (PRINT "sub"; / RETURN)
;
;  Format: DB hi,lo,len,<items>,$00  -- hi-then-lo matches DR_EXEC record format.
;  Items: TOK_xxx keyword tokens, TOK_DEC/TOK_HEX + hi,lo payload, "text" runs,
;  $22=DQ $3B=semicolon (in-string chars that need escaping). A record may
;  span several DB lines; the trailing comment shows the source line.
; =============================================================================
PROG:
        DB 0,10,27,TOK_REM," uBASIC 2650 - SHOWCASE V5",$00                                   ; 10  REM uBASIC 2650 - SHOWCASE V4.6
        DB 0,20,33,TOK_PRINT," ",$22,"-- uBASIC 2650 V5 Showcase --",$22,$00                  ; 20  PRINT "-- uBASIC 2650 V4.6 Showcase --"
        DB 0,30,24,TOK_PRINT," ",$22,"--- PRINT / CHR$ ---",$22,$00                             ; 30  PRINT "--- PRINT / CHR$ ---"
        DB 0,40,31,TOK_PRINT," CHR$(",TOK_DEC,0,65,")",$3B,"CHR$(",TOK_DEC,0,66,")",$3B,"CHR$("  ; 40  PRINT CHR$(65);CHR$(66);CHR$(67)
        DB TOK_DEC,0,67,")",$00
        DB 0,50,22,TOK_PRINT," ",$22,"--- ARITHMETIC ---",$22,$00                               ; 50  PRINT "--- ARITHMETIC ---"
        DB 0,60,51,TOK_PRINT," ",$22,"3+4=",$22,$3B,TOK_DEC,0,3,"+",TOK_DEC,0,4,$3B,$22,"  10-3=",$22  ; 60  PRINT "3+4=";3+4;"  10-3=";10-3;"  6*7=";6*7
        DB $3B,TOK_DEC,0,10,"-",TOK_DEC,0,3,$3B,$22,"  6*7=",$22,$3B,TOK_DEC,0,6,"*",TOK_DEC,0,7,$00
        DB 0,70,35,TOK_PRINT," ",$22,"20/4=",$22,$3B,TOK_DEC,0,20,"/",TOK_DEC,0,4,$3B,$22,"  17%5=",$22  ; 70  PRINT "20/4=";20/4;"  17%5=";17%5
        DB $3B,TOK_DEC,0,17,"%",TOK_DEC,0,5,$00
        DB 0,80,23,TOK_PRINT," ",$22,"--- COMPARISONS ---",$22,$00                              ; 80  PRINT "--- COMPARISONS ---"
        DB 0,90,25,TOK_IF," ",TOK_DEC,0,5,">",TOK_DEC,0,3," THEN ",TOK_PRINT," ",$22,"5>3 ok",$22,$00  ; 90  IF 5>3 THEN PRINT "5>3 ok"
        DB 0,100,25,TOK_IF," ",TOK_DEC,0,3,"<",TOK_DEC,0,5," THEN ",TOK_PRINT," ",$22,"3<5 ok",$22,$00  ; 100 IF 3<5 THEN PRINT "3<5 ok"
        DB 0,110,27,TOK_IF," ",TOK_DEC,0,3,">=",TOK_DEC,0,3," THEN ",TOK_PRINT," ",$22,"3>=3 ok",$22  ; 110 IF 3>=3 THEN PRINT "3>=3 ok"
        DB $00
        DB 0,120,27,TOK_IF," ",TOK_DEC,0,4,"<>",TOK_DEC,0,3," THEN ",TOK_PRINT," ",$22,"4<>3 ok",$22  ; 120 IF 4<>3 THEN PRINT "4<>3 ok"
        DB $00
        DB 0,130,25,TOK_IF," ",TOK_DEC,0,3,"=",TOK_DEC,0,3," THEN ",TOK_PRINT," ",$22,"3=3 ok",$22,$00  ; 130 IF 3=3 THEN PRINT "3=3 ok"
        DB 0,140,25,TOK_PRINT," ",$22,"--- LOOP via GOTO ---",$22,$00                           ; 140 PRINT "--- LOOP via GOTO ---"
        DB 0,150,5,"I=",TOK_DEC,0,1,$00                                                         ; 150 I=1
        DB 0,160,18,TOK_IF," I>",TOK_DEC,0,5," THEN ",TOK_GOTO," ",TOK_DEC,0,190,$00            ; 160 IF I>5 THEN GOTO 190
        DB 0,170,4,TOK_PRINT," I",$3B,$00                                                       ; 170 PRINT I;
        DB 0,180,7,"I=I+",TOK_DEC,0,1,$00                                                       ; 180 I=I+1
        DB 0,185,5,TOK_GOTO," ",TOK_DEC,0,160,$00                                               ; 185 GOTO 160
        DB 0,190,4,TOK_PRINT," ",$22,$22,$00                                                    ; 190 PRINT ""
        DB 0,192,25,TOK_PRINT," ",$22,"--- FOR/NEXT LOOP ---",$22,$00                           ; 192 PRINT "--- FOR/NEXT LOOP ---"
        DB 0,194,14,TOK_FOR," I=",TOK_DEC,0,1," TO ",TOK_DEC,0,5,$00                            ; 194 FOR I=1 TO 5
        DB 0,196,4,TOK_PRINT," I",$3B,$00                                                       ; 196 PRINT I;
        DB 0,198,3,TOK_NEXT," I",$00                                                            ; 198 NEXT I
        DB 0,199,4,TOK_PRINT," ",$22,$22,$00                                                    ; 199 PRINT ""
        DB 0,201,22,TOK_PRINT," ",$22,"--- FOR STEP 2 ---",$22,$00                              ; 201 PRINT "--- FOR STEP 2 ---"
        DB 0,203,23,TOK_FOR," I=",TOK_DEC,0,0," TO ",TOK_DEC,0,10," STEP ",TOK_DEC,0,2,$00      ; 203 FOR I=0 TO 10 STEP 2
        DB 0,205,4,TOK_PRINT," I",$3B,$00                                                       ; 205 PRINT I;
        DB 0,207,3,TOK_NEXT," I",$00                                                            ; 207 NEXT I
        DB 0,208,4,TOK_PRINT," ",$22,$22,$00                                                    ; 208 PRINT ""
        DB 0,210,24,TOK_PRINT," ",$22,"--- GOSUB/RETURN ---",$22,$00                            ; 210 PRINT "--- GOSUB/RETURN ---"
        DB 0,212,5,TOK_GOSUB," ",TOK_DEC,2,18,$00                                               ; 212 GOSUB 530
        DB 0,214,5,TOK_GOSUB," ",TOK_DEC,2,18,$00                                               ; 214 GOSUB 530
        DB 0,216,4,TOK_PRINT," ",$22,$22,$00                                                    ; 216 PRINT ""
        DB 0,220,21,TOK_PRINT," ",$22,"--- FUNCTIONS ---",$22,$00                               ; 220 PRINT "--- FUNCTIONS ---"
        DB 0,222,39,TOK_PRINT," ",$22,"ABS(-7)=",$22,$3B,TOK_ABS,"(-",TOK_DEC,0,7,")",$3B,$22   ; 222 PRINT "ABS(-7)=";ABS(-7);"  NEG(7)=";NEG(7)
        DB "  NEG(7)=",$22,$3B,TOK_NEG,"(",TOK_DEC,0,7,")",$00
        DB 0,224,52,TOK_PRINT," ",$22,"AND(12,10)=",$22,$3B,TOK_AND,"(",TOK_DEC,0,12,",",TOK_DEC,0,10  ; 224 PRINT "AND(12,10)=";AND(12,10);"  OR(12,10)=";OR(12,10)
        DB ")",$3B,$22,"  OR(12,10)=",$22,$3B,TOK_OR,"(",TOK_DEC,0,12,",",TOK_DEC,0,10,")",$00
        DB 0,226,45,TOK_PRINT," ",$22,"XOR(12,10)=",$22,$3B,TOK_XOR,"(",TOK_DEC,0,12,",",TOK_DEC,0,10  ; 226 PRINT "XOR(12,10)=";XOR(12,10);"  NOT(0)=";NOT(0)
        DB ")",$3B,$22,"  NOT(0)=",$22,$3B,TOK_NOT,"(",TOK_DEC,0,0,")",$00
        DB 0,228,9,TOK_POKE," ",TOK_DEC,31,64,",",TOK_DEC,0,42,$00                              ; 228 POKE 8000,42
        DB 0,230,28,TOK_PRINT," ",$22,"PEEK(8000)=",$22,$3B,"HEX$(",TOK_PEEK,"(",TOK_DEC,31,64,"))",$00  ; 230 PRINT "PEEK(8000)=";HEX$(PEEK(8000))
        DB 0,232,41,TOK_PRINT," ",$22,"RND(100)=",$22,$3B,TOK_RND,"(",TOK_DEC,0,100,")",$3B,$22  ; 232 PRINT "RND(100)=";RND(100);"  RND(100)=";RND(100)
        DB "  RND(100)=",$22,$3B,TOK_RND,"(",TOK_DEC,0,100,")",$00
        DB 0,234,4,TOK_PRINT," ",$22,$22,$00                                                    ; 234 PRINT ""
        DB 0,236,22,TOK_PRINT," ",$22,"--- LIST 40,60 ---",$22,$00                              ; 236 PRINT "--- LIST 40,60 ---"
        DB 0,238,9,TOK_LIST," ",TOK_DEC,0,40,",",TOK_DEC,0,60,$00                               ; 238 LIST 40,60
        DB 0,240,5,TOK_GOTO," ",TOK_DEC,1,44,$00                                                ; 240 GOTO 300
        DB 1,44,22,TOK_PRINT," ",$22,"--- MANDELBROT ---",$22,$00                               ; 300 PRINT "--- MANDELBROT ---"
        DB 1,54,6,"I=-",TOK_DEC,0,64,$00                                                        ; 310 I=-64
        DB 1,64,18,TOK_IF," I>",TOK_DEC,0,56," THEN ",TOK_GOTO," ",TOK_DEC,1,254,$00            ; 320 IF I>56 THEN GOTO 510
        DB 1,74,3,"D=I",$00                                                                     ; 330 D=I
        DB 1,84,6,"C=-",TOK_DEC,0,144,$00                                                       ; 340 C=-144 (widened from -120)
        DB 1,94,18,TOK_IF," C>",TOK_DEC,0,28," THEN ",TOK_GOTO," ",TOK_DEC,1,224,$00            ; 350 IF C>28 THEN GOTO 480 (widened from 4)
        DB 1,104,3,"A=C",$00                                                                    ; 360 A=C
        DB 1,105,3,"B=D",$00                                                                    ; 361 B=D
        DB 1,106,5,"E=",TOK_DEC,0,0,$00                                                         ; 362 E=0
        DB 1,107,5,"N=",TOK_DEC,0,1,$00                                                         ; 363 N=1
        DB 1,114,18,TOK_IF," N>",TOK_DEC,0,16," THEN ",TOK_GOTO," ",TOK_DEC,1,164,$00           ; 370 IF N>16 THEN GOTO 420
        DB 1,124,18,TOK_IF," E>",TOK_DEC,0,0," THEN ",TOK_GOTO," ",TOK_DEC,1,154,$00            ; 380 IF E>0 THEN GOTO 410
        DB 1,134,19,"T=A*A/",TOK_DEC,0,64,"-B*B/",TOK_DEC,0,64,"+C",$00                         ; 390 T=A*A/64-B*B/64+C
        DB 1,144,15,"B=",TOK_DEC,0,2,"*A*B/",TOK_DEC,0,64,"+D",$00                              ; 400 B=2*A*B/64+D
        DB 1,145,3,"A=T",$00                                                                    ; 401 A=T
        DB 1,154,43,TOK_IF," A*A/",TOK_DEC,0,64,"+B*B/",TOK_DEC,0,64,">",TOK_DEC,1,0," THEN ",TOK_IF  ; 410 IF A*A/64+B*B/64>256 THEN IF E=0 THEN E=N
        DB " E=",TOK_DEC,0,0," THEN E=N",$00
        DB 1,164,7,"N=N+",TOK_DEC,0,1,$00                                                       ; 420 N=N+1
        DB 1,165,19,TOK_IF," N<=",TOK_DEC,0,16," THEN ",TOK_GOTO," ",TOK_DEC,1,114,$00          ; 421 IF N<=16 THEN GOTO 370
        DB 1,174,27,TOK_IF," E>",TOK_DEC,0,0," THEN ",TOK_PRINT," CHR$(E+",TOK_DEC,0,32,")",$3B,$00  ; 430 IF E>0 THEN PRINT CHR$(E+32);
        DB 1,184,25,TOK_IF," E=",TOK_DEC,0,0," THEN ",TOK_PRINT," CHR$(",TOK_DEC,0,32,")",$3B,$00  ; 440 IF E=0 THEN PRINT CHR$(32);
        DB 1,194,7,"C=C+",TOK_DEC,0,4,$00                                                       ; 450 C=C+4
        DB 1,204,5,TOK_GOTO," ",TOK_DEC,1,94,$00                                                ; 460 GOTO 350
        DB 1,224,1,TOK_PRINT,$00                                                                ; 480 PRINT
        DB 1,234,7,"I=I+",TOK_DEC,0,6,$00                                                       ; 490 I=I+6
        DB 1,244,5,TOK_GOTO," ",TOK_DEC,1,64,$00                                                ; 500 GOTO 320
        DB 1,254,1,TOK_END,$00                                                                  ; 510 END
        DB 2,18,8,TOK_PRINT," ",$22,"sub",$22,$3B,$00                                           ; 530 PRINT "sub";
        DB 2,20,1,TOK_RETURN,$00                                                                ; 532 RETURN
SHOWCASE_END:

; =============================================================================
; TO-DO 
;   - Full pass over the file for inaccurate/stale comments
;   - Review scratch lifetimes: TMP,LNUM,SC0/1,TKH/L,ARGA,POWCNT,SAVE
;     TEMPRET, and potential to consolidate

        END
