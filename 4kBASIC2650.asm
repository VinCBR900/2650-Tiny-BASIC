; 4kBASIC2650.asm       4k Tiny BASIC interpreter for Signetics 2650 (was uBASIC)
; Version: v4.28
; By Vincent Crabtree, 2026.  MIT License
; Date:    2026-09-29
;
; Target:  Standalone (no PIPBUG ROM). Code ORG 0. I/O routines embedded.
;          Single 8192-byte address space (2650 bits 15:13 always 0).
;
; Assembler: asm2650.c v1.13+  Simulator: pipbug_wrap.c v2.1
; Build:
;   gcc -Wall -O2 -o asm2650 asm2650.c
;   gcc -Wall -O2 -DGAMER -o pipbug_wrap pipbug_wrap.c
;
;   ./asm2650 4kBASIC2650.asm 4kBASIC2650.hex
;   grep -n "^CHIN \|^COUT \|^ROMEND " .\4kBASIC2650.LST
;   ./pipbug_wrap --entry 0 --chin 0x<addr> --cout 0x<addr> 4kBASIC2650.hex
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
;   Only stored (numbered) lines can use Hex Literals as Direct/immediate
;   entry has no untokenized hex parser. The tokenizer accepts any number 
;   of hex digits, LIST always prints 4.
;
; GOSUB/RETURN NESTING
;   Up to 8 levels deep (GSSTKLIM=16 bytes, 2 per level).
;
; FOR / NEXT
;   FOR <var>=<start> TO <limit> [STEP <step>], then NEXT [<var>]. Default
;   step is 1 (positive or negative steps both work via STEP).
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
;   PARSE_EXPR entry guard: SPSU/ANDI/COMI fires ERR_NEST if SP>=5 at entry.
;
;        SCRATCH REGISTER CONVENTIONS
;   R0  working register, arithmetic, I/O.
;   R1  index register (LODA/STRA BASE,R1); also PRINT_S16 digit buffer index.
;       Clobbered by INC_ET (INC_TMP/INC_EXP shared body). Callers verified safe.
;   R2  long-lived variable letter (DO_LET/DO_INPUT/DO_FOR, preserved across PARSE_EXPR)
;       Never written by subroutines except DO_LET, SE_BAREASS, DO_FOR.
;   R3  loop counter (BDRR/BIRR); STORE_LINE shift count. SW expr-stack pointer.
;       RDLINE (V4.2): IBUF offset, $FF=empty sentinel (",Rx+" pre-increments
;       before access -- see SWBASE convention -- so empty must be one below
;       the first valid index, not 0).
;
;        KNOWN OPEN ITEMS
;   COLON-01: ':' multi-statement not supported - likely never due to RAS.
;   OPT-16:   MUL16/DIV16 uses O(N) loop for size - O(16) bit-serial deferred.
;   FOR-01:   NEXT variable not checked against frame var (smallest code, by spec).
;   FOR-02:   Body always executes at least once (no skip-if-false-at-entry, by spec).
;   REC-01:   Triple-nested atom-dispatch (1+ABS(NEG(AND(...)))) hits RAS limit guard
;
; CHANGE HISTORY
;
; V4.28 (2026-09-29) - ROMEND $0ED5 -> $0E92 (3730 bytes)
;   - Deleted GETCI and vector - WSKIP leaves char in R0, uses INC_IP instead. 
;     Cleaned up multiple unecessary LODA,R0 *IPH after WSKIP calls.
;   - Zero-page vector sweep, added common REG16_TO_REG16 vector and shims.
;
; V4.27 (2026-09-29) - ROMEND $0EEF -> $0ED6 (3798 bytes)
;   - DO_FOR STEP test now peeks (WSKIP) instead of GETCI/DEC_IP on default-step.
;   - Deleted DEC_IP, DEC_ET body and VDEC_IP zero-page vector as no callers.
;   - BUG-FOREQ-01: space before '=' (FOR I = 1 TO 3, FOR I =1 TO 3) gave ?0.
;     DO_FOR used GETCI which does not skip whitespace, replaced with WSKIP.
;
; V4.26 (2026-09-29) - ROMEND $0F1F -> $0EEF 
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
; V4.25 (2026-09-28) - ROMEND $0F8E -> $0F1F 
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
; V4.24 (2026-09-28) - ROMEND $1019 -> $0FEE
;   Golf pass on the V4.23 tokenizer fixes -- no behaviour change
;   - TOKENIZE_LINE / TOK_MATCH_ROW / TL_MULADD added dedicated TKH:TKL
;     pair instead of TMPH:TMPL, TSL_WRITE no longer saves/restores TMP.
;   - TL_STR_LP re-reads the just-stored char with LODA,R0 TOKBUF,R3 instead
;     of caching it in SC0 across VINC_IP. 
;   - TL_DEC_DONE and TL_HEX_DONE shared one constant-emit tail
;     (TL_EMIT_CONST); they differed only in the token byte.
;   - ORG 4096 re-enabled; memory map is back to the real split (IPH=$1000).
;
; V4.23 (2026-09-28) - ROMEND $0FFC -> $1019 overflow
;   Tokenizer step 3 (encode side: TOKENIZE_LINE + TRY_STORE_LINE)
;   - TSL_WRITE: TOKENIZE_LINE clobbers TMPH:TMPL. Fix: TMPSAVEH/TMPSAVEL, 
;     superseded in V4.24 by TKH:TKL pair.
;   - TL_LP letter typo BCFR,LT instead of BCTR,LT
;   - TL_STR_LP tested R0 for the closing quote after ZBSR *VINC_IP clobber.
;     Fix: cachein SC0 across the call. 
;   - TSL_CPY copied TOKBUF up to the first $00 byte, truncating records at
;     a TOK_DEC/TOK_HEX payload zero byte. Fix: copy by count (R3-2=body+NUL)
;   - TL_MULADD: STRZ,R3 (multiplier) ran after EXP16_TO_TMP/CLR_EXP had
;     clobbered R0, so R3=0 and BDRR looped 256x: Fix: STRZ,R3 first.
;   - TL_HEX_LP: STRA,R0 SC0 ran after ZBSR *VINC_IP (R0 clobbered), so hex
;     digits accumulated a pointer byte. Fix: swap the two.
;   - TOKBUF (80) overflowed into VARS on dense-constant lines, expanded 
;     so TOKBUF now RES 128. 
;
; V4.22 (2026-09-26) - ROMEND $0DA3 -> $0E8D (+234 bytes)
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
; V4.21 (2026-09-25) - ROMEND $0D99 -> $0DA3 
;  Tokenizer project step 1 - Table expansion.
;   - KW_TAB/FUNC_TAB expanded from a fixed 5-byte-stride [c1][c2][c3][hi]
;     [lo] row (matched on the first 2-3 chars only) to variable-length
;     [full keyword text][NUL][hi][lo] rows, matched on the COMPLETE
;     spelling. This table doubles as the LIST detokenizer's text
;     source once tokens land (Step 2) -- avoids 2nd duplicate table.
;   - MATCH_KW/SE_SCAN rewritten: no more up-front 2-char GETCI prefetch
;     into SC0/SC1 (dropped from the clobbers list entirely -- nothing else
;     in the file depended on MATCH_KW leaving them set). Each row is now
;     compared char-by-char directly against IP (peeked via *IPH, not
;     consumed) using R0/R1 only 
;   - Dispatch now requires the FULL keyword spelling; trailing garbage after
;     a complete keyword is still swallowed by EATWORD like before.
;
; V4.20 (2026-09-24) - ROMEND $0DDE -> $0D99 
;   - Line storage/deletion (TRY_STORE_LINE/STORE_LINE/DELETE_LINE/MEMCPY/
;     DEC_PE) ported from uBASIC2650wip v2.10: merged into TRY_STORE_LINE
;     (find-and-delete loop shared by pure-delete and overwrite, then a
;     conditional insert) + DEL_REC (unchanged forward-copy delete) +
;     OPEN_GAP (builds the prospective new PE first via repeated INC_EXP,
;     checked against PROGLIM's high byte -- valid because PROGLIM=$1FFF
;     ends in $FF -- before touching anything, then shifts backward with
;     DEC_PE inlined, its only caller). MEMCPY deleted (no longer called).
;   - Calling-convention adaptations for 4k (which doesn't vector CMP_TMP_PE
;     or INC_ET/EXP16_TO_ET the way wip does): direct BSTA/BSTR calls to
;     CMP_TMP_PE, EXP16_TO_ET; the two "preload offset then call the
;     generic INC_ET" sites both happened to be incrementing EXP
;     specifically, so both use 4k's existing VINC_EXP vector directly
;   - Line-number range validation (rejecting >32767, silently ignoring
;     line 0) unchanged from 4k's existing behavior
;
; V4.19 (2026-09-24) - ROMEND $0E09 -> $0DDE 
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
; V4.18 (2026-09-24) - ROMEND $0E82 -> $0E09 
;   - PRINT_S16: Replaced the SW-recursive shift/divide-by-10 engine (PREC,
;     16 iterations of a 4-byte rotate + conditional subtract, plus the
;     SWBASE push/pop recursion machinery) with uBASIC2650wip's iterative
;     repeated-subtraction-by-power-of-10 approach (P10_HI/P10_LO tables).
;   - Removed MSG_MIN/MIN_LP (the BUG-MINLP-01 -32768 special case): the
;     ported algorithm's WC-based unsigned subtraction is correct for the
;     full 0-65535 range regardless of sign, so 32768 no longer needs
;     separate handling.
;   - PRINT_S16 no longer clobbers TMP (now clobbers R2 instead, which no
;     caller needs preserved across the call) or SC1/NEGFLG.
;   - SWRETURN kept in place (still shared with PARSE_EXPR's own recursion).
;
; V4.17 (2026-09-22) - ROMEND $0EB9 -> $0E85 
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
; V4.16 (2026-09-22) - ROMEND $0EBA 
;   - FL_CHKLO / FI_LH: Direct-indexed peek (*TMPH,R1) replaces INC16_TMP_TO_EXP.
;   - Stored record format: Changed line terminator CR ($0D) -> NUL ($00).
;   - Shared helpers: Added ADV_TMP_PAST_REC to scan past NUL-terminated records.
;   - DR_EXEC: Direct-in-place execution from store (TMP+2), bypassing IBUF copy loop.
;   - FIXED BUG-PU16-01: Corrected PARSE_U16 pre-loop digit check bound (':' -> '9').
;   - Streamlined DO_NEW, SHOWCASE_END to PE setup deleted, now Assembly time.
;
; V4.15 (2026-09-22) - ROMEND $0F26 
;   - Shared helpers: Added CMP_TMP_PE for 16-bit TMP vs PE comparison.
;   - DELETE_LINE: Ported uBASIC2650 DEL_REC idiom (uses dst ptr as new PE).
;   - PARSE_U16 / TRY_STORE_LINE: Collapsed digit checks to single unsigned sub/cmp.
;   - STORE_LINE: Ported OPEN_GAP technique; indexed dst off walking PE (*PEH,R3).
;
; V4.14 (2026-09-21) - ROMEND $0FB7 
;   - Unsigned comparison: Enabled global COM=1 mode.
;
; V4.13 (2026-09-20) - ROMEND $0FB5 
;   - FIXED BUG-MUL-01: MU_LP decrement now checks carry flag (TPSL $01) instead
;     of raw CC, fixing 16-bit multiply for operands >= 256.
;
; V4.12 (2026-09-20) - ROMEND $0FB3 
;   - Shared helpers: Added CARRY_INTO_EXPH to consolidate carry propagation
;     in MUL16 and PARSE_U16.
;
; V4.11 (2026-09-20) - ROMEND $0FC8 
;   - CHIN: Removed hard ORG $286 pin to let subroutine space float naturally.
;
; V4.10 (2026-09-20) - Renamed 4kBASIC - ROMEND $0FC8
;   - ZPVEC-01: Vectored PUSH_EXP and PRINT_S16 via zero-page jump table (ZBSR).
;
; V4.9 (2026-08-16) - ROMEND $0FDB 
;   - FIXED BUG-DL2-01: DL2_LP copy loop now tests carry flag (TPSL $01) instead
;     of signed CC, fixing premature loop exit on page-boundary wraps during deletes.
;
; V4.8 (2026-08-15) - ROMEND $0FD9 
;   - PRINT HEX$(n): Added 4-digit unsigned 16-bit hex formatting in ROM slack.
;
; V4.7 (2026-08-10) - ROMEND $0F98
;   - Shared helpers: Added PUSH_RET to handle 16-bit SWBASE push.
;   - Operators: Added ^ (power) operator with error handling.
;   - FIXED BUG-MINLP-01: Fixed hang printing -32768 (invalid indexed LODR).
;
; V4.6 (2026-07-03) - 3879 bytes (ROMEND $0F27)
;   - FIXED: FUNCATOM-01 - functions now work as non-leading atoms, e.g.
;     "PRINT 10+ABS(A)" (previously only "PRINT ABS(A)+10" worked).
;   - Added FT_SP/FT_STK/FT_SAVE_SP/FT_SAVE/FT_N/FT_R2SAVE (72 RAM bytes)
;     and FUNC_EPILOG; PE_SAFE/EAM_ATOM/PE_NOFUNC/DO_END updated. 
;
; V4.5 (2026-06-30) - 3705 bytes
;   - FIXED: Function parser tracking for trailing operators (e.g., ABS(-5)+10).
;   - Relocated PEEK/USR/EXPH functions to optimize space post-COUT.
;
; V4.4 (2026-06-30) - 3636 bytes
;   - FIXED: RND 16-bit seed rotation bug by correctly setting PSL WC bit.
;   - FIXED: Subtraction left-operand dropping bug in EAM_MH_RET.
;   - Unified 2-argument parsing for AND/OR/XOR/POKE/LIST to save ~25 bytes.
;   - Deduplicated bare assignments (LET-less statements).
;
; V4.3 (2026-06-25) - 3672 bytes
;   - FIXED: Nested operator precedence clobbering bug using SWBASE stack.
;   - Added bitwise functions: AND(a,b), OR(a,b), XOR(a,b), NOT(a).
;   - Rewrote default showcase program with an expanded Mandelbrot finale.
;
; V4.2 (2026-06-24) - 3485 bytes
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
TOK_STMT_BASE   EQU $80         ; statement token = TOK_STMT_BASE + KW_TAB row (16 rows: $80-$8F)
TOK_FUNC_BASE   EQU $A0         ; function token  = TOK_FUNC_BASE + FUNC_TAB row (9 rows: $A0-$A8)
                                 ; $90-$9F reserved (more statements later)
TOK_HEX         EQU $FD         ; [TOK_HEX][hi][lo] -- 16-bit hex constant
TOK_DEC         EQU $FE         ; [TOK_DEC][hi][lo] -- 16-bit decimal constant
TOK_REM_ROW     EQU 13          ; REM's KW_TAB row index (0-based) -- TOKENIZE_LINE
                                 ; needs this by row number, not by token value,
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
                                 ; RAS_DEPTH-3 covers PARSE_FACTOR+PARSE_S16+INC_IP

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
VCLR_RUNFLG:
        DW CLR_RUNFLG           ; 3 sites
VREG16_TO_REG16:
        DW REG16_TO_REG16       ; 29 sites (V4.25: generic packed-nibble 16-bit copy;
VFETCH16_IP:
        DW FETCH16_IP           ; 4 sites (V4.26: EXP = word at [IP], IP += 2)
VSET_TMP_PROG:
        DW SET_TMP_PROG ; 3 sites
VCHECK_LPAREN:
        DW CHECK_LPAREN          ; 7 sites (v4.5 FUNCCONT-01: ABS/NEG/NOT/
VCHECK_RPAREN:
        DW CHECK_RPAREN          ; 6 sites (ABS/NEG/NOT/PEEK/USR/RND)
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
VCARRY_INTO_EXPH:
        DW CARRY_INTO_EXPH; 4 sites

; =============================================================================
MAIN:
        PPSL $02
       
        ; clear RUNFLG, SWSP, FORSP, GOTOFLG - change to DO_NEW for ROM
        BSTA,UN DO_END          

        ; clear A-Z variables (52 bytes) 
        LODI,R3 51       ; Loop bounds: 51 down to 0 (52 total bytes)
        EORZ,R0          ; Clear R0 (Stays zero; STRA doesn't alter ALU states)
CLRV:
        STRA,R0 VARS,R3  ; Clear target index byte directly
        BDRR,R3 CLRV     ; Decrement R3 and loop until underflow to $FF

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
        BSTA,UN RDLINE
        BSTA,UN SET_IP_IBUF ; IPH:IPL = IBUF
        BSTA,UN TRY_STORE_LINE           ; CC=GT: line stored/deleted; CC=EQ: not a line
        BSTA,EQ STMT_EXEC               ; If CC=EQ (not a line), execute
        BCTR,UN REPL

; =============================================================================
;  DO_IF -- Conditional execution
; Syntax: IF expr relop expr THEN stmt
; In:  IP -> first char after IF keyword
; Out: executes stmt if condition true; otherwise sequential return
; Clobbers: R0, R1, EXPH, EXPL, LNUMH, LNUML, SC0, SC1, RELOP
; RAS: entry+1(PE)+1(PR)+1(PE)+1(SE) = entry+4. Max depth 7: ok.
DO_IF:
        ZBSR *VPARSE_EXPR                 ; [+1]
        ZBSR *VEXP_TO_LNUM ; LNUMH:LNUML = EXPH:EXPL (save left operand)
        BSTA,UN PARSE_RELOP              ; [+1]
        ZBSR *VPARSE_EXPR                 ; [+1]

        ; signed 16-bit compare: LNUMH:LNUML (left) vs EXPH:EXPL (right)
        ; bias hi bytes by XOR $80 for unsigned compare
        LODA,R0 LNUMH
        EORI,R0 $80
        STRA,R0 SC0
        LODA,R0 EXPH
        EORI,R0 $80
        SUBA,R0 SC0                      ; biased right.hi - biased left.hi
        BCTR,LT DIF_LT
        BCTR,GT DIF_GT
        ; hi bytes equal: compare lo (unsigned)
        LODA,R0 EXPL
        SUBA,R0 LNUML
        BCTR,LT DIF_LT
        BCTR,GT DIF_GT
        EORZ,R0
        STRA,R0 SC1
        BCTR,UN DIF_TH                   ; EQ
DIF_LT:
        LODI,R0 $01                      ; left > right
        STRA,R0 SC1
        BCTR,UN DIF_TH
DIF_GT:
        LODI,R0 $FF                      ; left < right
        STRA,R0 SC1

DIF_TH:
        ; consume THEN keyword
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
        ; map SC1 to bitmask, AND with RELOP
        ;   SC1=$FF -> LT -> bit 0 ($01)
        ;   SC1=$00 -> EQ -> bit 1 ($02)
        ;   SC1=$01 -> GT -> bit 2 ($04)
        LODA,R0 SC1
        BCTR,EQ DIF_IS_EQ
        COMI,R0 $FF
        BCTR,EQ DIF_IS_LT
        LODI,R0 4                        ; GT
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to next LODI
DIF_IS_LT:
        LODI,R0 1                        ; LT
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to next LODI
DIF_IS_EQ:
        LODI,R0 2                        ; EQ
        LODA,R1 RELOP
        ANDZ,R1                          ; R0 &= R1
        RETC,EQ                          ; no match: condition false, sequential return
        ; drop through
; =============================================================================
;  STMT_EXEC -- Decode and dispatch one BASIC statement from IP.
; In:  IPH:IPL -> first char of statement (after any leading whitespace)
; RAS depth: 1 from REPL, 3 from DO_IF(THEN body).
STMT_EXEC:
        ; Look at statement KW_TAB with TMPH:TMPL as pointer
        LODI,R0 <KW_TAB
        STRA,R0 TMPH
        LODI,R0 >KW_TAB
        STRA,R0 TMPL
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
; Out: handler called; IP advanced past the matched word/token (miss: IP
;      unchanged, see above)
; Clobbers: R0, R1, EXPH, EXPL, GOTOH, GOTOL
; =============================================================================
; MATCH_KW -- Match and Jump to handler based on KW
; Clobbers: R0, R1, EXPH, EXPL, GOTOH, GOTOL
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

SE_NOTKW:
        BSTA,UN PARSE_VAR_SAVE            ; validates A-Z, SC0/R2 = letter
        ZBSR *VWSKIP
        COMI,R0 A'='
        BCFA,EQ JSYNERR
        ZBSR *VINC_IP
        BCTR,UN DL_EX                     ; expression follows

; =============================================================================
;  DO_NEW -- Clear program store
; Syntax: NEW
; In:  nothing
; Out: PEH:PEL = VARS; program store ($10E0-$1FFF) zeroed; falls through to DO_END
; Clobbers: R0, R1, IPH, IPL, SWSP, FORSP, GOTOFLG, RUNFLG
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
; Out: SWSP=0, FORSP=0 (V4.19: 0-based), GOTOFLG=0, RUNFLG=0
; Clobbers: R0
DO_END:
        EORZ,R0                          ; V4.19: FORSP/SWSP are 0-based now
        STRA,R0 FORSP                    ; clear FOR stack
        STRA,R0 SWSP                     ; clear GOSUB stack
        LODI,R0 $FF                      ; FT_SP/FT_SAVE_SP: separate
        STRA,R0 FT_SP                    ; subsystem, still $FF=empty --
                                          ; FUNCATOM-01 (v4.6) dispatch-
                                          ; origin stack, untouched by V4.19
        STRA,R0 FT_SAVE_SP               ; ...and its byte-save stack too
        EORZ,R0
        STRA,R0 GOTOFLG
        ZBRR *VCLR_RUNFLG               ; tail call

; =============================================================================
;  DO_LET -- Variable assignment
; Syntax: LET V = expr   (also handles bare "V = expr" via SE_BAREASS)
; In:  IP -> variable letter
; Out: VARS[V] = EXPH:EXPL
; Clobbers: R0, R2, SC0, SC1, EXPH, EXPL, TMPH, TMPL, R1
; Note: DO_INPUT jumps to DL_STORE with variable letter in SC0 and R2.
DO_LET:
        BSTA,UN PARSE_VAR_SAVE
        ZBSR *VWSKIP                      ; [+1]
        COMI,R0 A'='
        BCTR,EQ DL_EQC
        ZBRR *VJSYNERR 
DL_EQC:
        ZBSR *VINC_IP  
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
; Out: GOTOH:GOTOL = target line; GOTOFLG=$01
; Clobbers: R0, EXPH, EXPL, GOTOH, GOTOL, GOTOFLG
DO_GOTO:
        ZBSR *VWSKIP  
        ZBSR *VPARSE_EXPR                 ; [+1]
        LODI,R0 (IDX_EXP*16)+IDX_GOTO
        ZBSR *VREG16_TO_REG16             ; GOTOH:GOTOL = EXPH:EXPL
        LODI,R0 1
        STRA,R0 GOTOFLG
        LODA,R0 RUNFLG                   ; OPT-10
        RETC,GT                          ; return if running
        ZBRR *VCLR_RUNFLG 

; =============================================================================
;  DO_RETURN -- Return from subroutine
; Syntax: RETURN
; In:  SWSP = GOSUB stack pointer; GSBASE[SWSP]=lo, GSBASE[SWSP+1]=hi of NLP.
; Out: GOTOH:GOTOL = popped NLP; GOTOFLG=$03 (direct NLP, no FIND_LINE).
; Clobbers: R0, R1, GOTOH, GOTOL, GOTOFLG, SWSP.
; Error: SWSP=0 (underflow) -> ERR_RET.
; BUG-RET-01 FIX: GOTOFLG must be $03 (direct address) not $01 (FIND_LINE).
;   GSBASE stores program-store addresses, not BASIC line numbers.
; V4.19: SWSP is 0-based (0=empty) -- decrement first, then read at the
;   new (post-decrement) index; no roll-to-sentinel special case needed.
DO_RETURN:
        LODA,R0 SWSP
        BCTA,EQ DRT_UNDERFLOW   ; SWSP==0: nothing pushed
        SUBI,R0 2
        STRA,R0 SWSP            ; Store updated SWSP back to RAM
        STRZ,R1                 ; R1 = new SWSP = read index

        ; Fetch 16-Bit Address 
        LODA,R0 GSBASE,R1       ; Load Lo-Byte
        STRA,R0 GOTOL           ; Store Lo-Byte                         
        LODA,R0 GSBASE,R1+      ; Load Hi-Byte & auto-increment R1      
        STRA,R0 GOTOH           ; Store Hi-Byte                         
; ---  DO_NEXT Entry Point Preserved  ---
DRT_GO:
        LODI,R0 3               ; GOTOFLG=$03 direct NLP                
        STRA,R0 GOTOFLG         ;      [3B]
        RETC,UN                 ; Return                                [1B]

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
; tight loop highly correlated (only 1 bit of mixing between draws). Now
; shuffles a full byte (8 shifts) per call; CHIN's incidental shuffle on
; keypress still adds independent async entropy on top.
; v4.4: loop via R2+BDRR instead of unrolled (R2 is untouched by RND_SHUFFLE
; and not live here - this is a FUNC_TAB-dispatched call, see PARSE_2ARGS
; header note on register lifetimes) - 6 bytes vs the unroll's 16.
; =============================================================================
DO_RND_FUNC:
        ZBSR *VCHECK_LPAREN
        ZBSR *VPARSE_EXPR       ; get range in EXP 
        ZBSR *VCHECK_RPAREN
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
;  their old pre-CHIN location): both paren-bounded now (CHECK_LPAREN/
;  CHECK_RPAREN) and resume via FUNC_CONT (shared EXPH_Z tail), part of the
;  FUNCCONT-01 fix.
DO_PEEK_FUNC:
        ZBSR *VCHECK_LPAREN
        ZBSR *VPARSE_EXPR
        ZBSR *VCHECK_RPAREN
        LODA,R0 *EXPH
        BCTR,UN EXPH_Z                   ; clear top byte

; =============================================================================
; Calling function retval in R0
DO_USR_FUNC:
        ZBSR *VCHECK_LPAREN
        ZBSR *VPARSE_EXPR
        ZBSR *VCHECK_RPAREN
        BSTA,UN *EXPH   
EXPH_Z:        
        STRA,R0 EXPL
        EORZ,R0          ; clear top byte
        STRA,R0 EXPH
        ZBRR *VFUNC_CONT

; =============================================================================
; PARSE_VAR_SAVE -- skip whitespace, read var letter, range-check,
;                   save to SC0 and R2, advance IP.
; Out: SC0=R2=letter (A-Z); IP advanced past letter
; Error: tail-jumps to JERRVAR (no return)
; Clobbers: R0, R1, R2, SC0
PARSE_VAR_SAVE:
        ZBSR *VWSKIP  
        COMI,R0 A'A'
        BCTA,LT JERRVAR       ; out of range low  -- tail jump, no return
        COMI,R0 A'Z'+1
        BCFA,LT JERRVAR       ; out of range high -- tail jump, no return
        STRA,R0 SC0
        STRZ,R2                          ; save in R2 for DL_STORE
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
        BSTA,UN RDLINE                   ; [+1]
        BSTA,UN SET_IP_IBUF ; IPH:IPL = IBUF
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
CHIN:
        BSTA,UN RND_SHUFFLE
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
        ZBSR *VWSKIP  
        LODI,R1 0                ; initialize lookahead index
        LODA,R0 *IPH,R1         ; R0 = 1st char, R1 = 1 (IP untouched!)
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
; Out: GOTOH:GOTOL = target line; GOTOFLG=$02; NLP pushed onto GSBASE.
; Clobbers: R0, R1, EXPH, EXPL, GOTOH, GOTOL, GOTOFLG, SWSP
; Stack: GSBASE[SWSP]=lo, GSBASE[SWSP+1]=hi. SWSP=0=empty (V4.19).
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
        LODA,R0 SWSTK+1                  ; NLP lo byte
        STRA,R0 GSBASE,R1                ; GSBASE[SWSP] = lo (plain: ,Rx+ PRE-
                                          ; increments, so lo must come first
                                          ; unindexed -- matches DO_RETURN's pop)
        LODA,R0 SWSTK                    ; NLP hi byte
        STRA,R0 GSBASE,R1+                ; GSBASE[SWSP+1] = hi; R1 -> SWSP+1
        ADDI,R1 1                        ; R1 = SWSP+2 = new SWSP
        STRA,R1 SWSP
        LODI,R0 (IDX_EXP*16)+IDX_GOTO
        ZBSR *VREG16_TO_REG16             ; GOTOH:GOTOL = EXPH:EXPL (target line)
        LODI,R0 2                        ; GOTOFLG=$02 = GOSUB pending
        STRA,R0 GOTOFLG
        LODA,R0 RUNFLG
        RETC,GT
        ZBRR *VCLR_RUNFLG 

; =============================================================================
;  DO_FOR -- FOR loop setup
; Syntax: FOR V = start TO limit [STEP step]
; In:  IP past "FOR" keyword; SWSTK[0:1] = NLP of line after FOR (set by DR_EXEC).
; Out: Frame pushed on FORBASE; var initialised to start; falls through to body.
; Clobbers: R0, R1, R2, FORVAR, FORSP, LNUMH, LNUML, EXPH, EXPL, TMPH, TMPL.
; Errors: stack full -> ERR_FOR.
; Frame layout (7 bytes at FORBASE+FORSP): [var][limH][limL][stpH][stpL][nlpH][nlpL]
; Loop body always executes at least once; exit tested at NEXT.
DO_FOR:
        ; --- get variable letter ---
        ZBSR *VWSKIP                      ; [+1] skip whitespace; R0 = var letter (peek)
        STRA,R0 FORVAR
        STRZ,R2                          ; R2 = var letter (survives PARSE_EXPR)
        ZBSR *VINC_IP                     ; [+1] consume the letter (R0 clobbered; kept in R2)

        ; --- stack overflow check (FORSP is 0-based, 0=empty; V4.19) ---
        LODA,R0 FORSP
        COMI,R0 FORSTKLIM
        BCFA,LT JFORERR                  ; If NOT Less-Than limit (>=), fail!
        ADDI,R0 7                        ; Normal frame adds 7 / Empty frame ends up at 0
        STRA,R0 FORSP

        ; --- skip '=' then parse start value -> EXPH:EXPL ---
        ZBSR *VWSKIP                      ; [+1] skip whitespace before '='
        ZBSR *VINC_IP                     ; [+1] consume '='
        ZBSR *VPARSE_EXPR                 ; [+1] start value -> EXPH:EXPL
        BSTA,UN DL_STORE                 ; [+1] VARS[R2] = EXPH:EXPL
        STRA,R1 FORVAR                   ; DL_STORE leavesVARSin R1 -- cache it here
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
        ; Push 7-byte frame at FORBASE[FORSP] using R1 as walking index.
        ; Layout: FORVAR, LNUMH, LNUML, EXPH, EXPL, SWSTK, SWSTK+1.
        LODA,R1 FORSP                    ; R1 = frame base offset
        LODI,R3 -7                      ; [2 bytes] Initialize loop counter to -7
        LODI,R2, -1                     ; VARS ptr

DF_LOOP:
        LODA,R0 FORVAR,R2+              ; get VARS value
        STRA,R0 FORBASE,R1              ; push on frame
        ADDI,R1 1
        BIRR,R3 DF_LOOP        ; [2 bytes] Increment R3; branch to loop if R3 != 0
        RETC,UN

; =============================================================================
;  DO_NEXT -- FOR loop iteration
; Syntax: NEXT [V]
; In:  FORSP = FOR stack pointer; top frame at FORBASE[FORSP].
;      Frame: [var][limH][limL][stpH][stpL][nlpH][nlpL]
;      var = VARS offset, precomputed by DO_FOR (V4.19; was the raw letter,
;      recomputed here every call via (var-'A')*2 pre-V4.19).
; Out: If looping: GOTOH:GOTOL=nlp, GOTOFLG=$03 (direct addr branch).
;      If done: FORSP-=7 (0-based since V4.19; was $FF-sentinel), sequential return.
; Clobbers: R0, R1, EXPH, EXPL, LNUMH, LNUML, SC0, GOTOH, GOTOL, GOTOFLG.
; Errors: FORSP=0 -> ERR_NXT.
; Variable name after NEXT consumed but not checked against frame (smallest code).
; =============================================================================
DO_NEXT:
        LODA,R0 FORSP
        BCTA,EQ JERR_NXT                  ; FORSP==0: no FOR active
        ZBSR *VWSKIP                      ; [+1]
        ZBSR *VEATWORD                    ; [+1] consume optional var name

        ; --- read step from frame[3:4] -> EXPH:EXPL ---
        LODA,R1 FORSP
        LODA,R0 FORBASE+3,R1              ; frame[3] = stpH
        STRA,R0 EXPH
        LODA,R0 FORBASE+4,R1              ; frame[4] = stpL
        STRA,R0 EXPL

        ; Golf #2: Directly load var offset into R1, eliminating SC0 storage
        LODA,R0 FORBASE,R1                ; frame[0] = precomputed VARS offset
        STRZ,R1

        ; --- 16-bit signed add in-place: var += step against VARS memory and LNUM
        ; simultaneousy, remove need to load var into LNUM first
        CPSL PSW_WC                       ; clear WC
        LODA,R0 VARS+1,R1                 ; var lo
        ADDA,R0 EXPL                      ; var_lo + step_lo
        STRA,R0 VARS+1,R1                 ; write new var lo to VARS
        STRA,R0 LNUML                     ; keep in LNUML for compare phase
        
        PPSL PSW_WC                       ; set WC: carry propagates
        LODA,R0 VARS,R1                   ; var hi
        ADDA,R0 EXPH                      ; var_hi + step_hi + carry
        CPSL PSW_WC                       ; clear WC
        STRA,R0 VARS,R1                   ; write new var hi to VARS
        STRA,R0 LNUMH                     ; keep in LNUMH for compare phase

        ; --- signed 16-bit compare: var vs limit ---
        ; Use direct frame offsets again; no need to stash limL in EXPL.
        LODA,R1 FORSP                     ; R1 = FORSP
        LODA,R0 FORBASE+1,R1              ; frame[1] = limH
        EORI,R0 $80                       ; biased limH
        STRA,R0 SC0
        
        LODA,R0 LNUMH
        EORI,R0 $80                       ; biased varH
        SUBA,R0 SC0                       ; biased(varH) - biased(limH)
        BCTR,GT DN_VAR_GT                 ; var > lim (hi bytes)
        BCTR,LT DN_VAR_LT                 ; var < lim (hi bytes)
        
        ; hi bytes equal: compare lo bytes
        LODA,R0 FORBASE+2,R1              ; frame[2] = limL (read directly!)
        SUBA,R0 LNUML                     ; limL - varL
        BCTR,GT DN_VAR_LT                 ; lim.lo > var.lo -> var < lim
        BCTR,LT DN_VAR_GT                 ; lim.lo < var.lo -> var > lim
        BCTR,UN DN_LOOP                   ; equal: body runs at limit value

DN_VAR_LT:
        LODA,R0 EXPH                      ; step sign test
        BCTR,LT DN_EXIT
        ; positive step: fall through to DN_LOOP

DN_LOOP:
        ; R1  holds FORSP securely from the compare block 
        LODA,R0 FORBASE+5,R1              ; frame[5] = nlpH
        STRA,R0 GOTOH
        LODA,R0 FORBASE+6,R1              ; frame[6] = nlpL
        STRA,R0 GOTOL
        BCTA,UN DRT_GO                    ; GOTOFLG $03 is set internally

DN_VAR_GT:
        LODA,R0 EXPH
        BCTR,LT DN_LOOP                   ; negative step: keep going down
        ; positive step: fall through to DN_EXIT

DN_EXIT:
        LODA,R0 FORSP
        SUBI,R0 7
        STRA,R0 FORSP
        RETC,UN

; =============================================================================
;  DO_RUN -- Execute stored program
; Syntax: RUN
; In:  PROG=program base, PEH:PEL=program end
; Out: runs until END, error, or exhausted; returns to REPL
; Clobbers: all
; GOTOFLG after STMT_EXEC: $00=sequential, $01=GOTO, $02=GOSUB, $03=FOR direct NLP.
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
        ZBSR *VCMP_TMP_PE
        BCTR,GT DR_STOP
        RETC,EQ
DR_EXEC:
        ; save current line number for error reporting (TMP unchanged)
        ; IP = TMP (record start); FETCH16_IP reads the 2-byte line-number header
        ; into EXP and leaves IP at the body 
        ; CUR = line number for error reporting (EXP is free here:
        ; every statement re-parses before it reads EXP).
        LODI,R0 (IDX_TMP*16)+IDX_IP
        ZBSR *VREG16_TO_REG16
        ZBSR *VFETCH16_IP                 ; [+2]
        LODI,R0 (IDX_EXP*16)+IDX_CUR
        ZBSR *VREG16_TO_REG16
        ; advance TMP past this whole record to the next one, for SWSTK
        BSTA,UN ADV_TMP_PAST_REC           ; [+1]
        ; Save next-line pointer into SWSTK before STMT_EXEC clobbers SC0/SC1.
        ; SWSTK persists across STMT_EXEC; DO_GOSUB and DO_FOR read from it.
        LODI,R0 (IDX_TMP*16)+IDX_SWSTK
        ZBSR *VREG16_TO_REG16
        ; execute line
        BSTA,UN STMT_EXEC                ; [+1]
        ; dispatch on GOTOFLG
        LODA,R0 GOTOFLG
        BCTR,EQ DR_SEQ                   ; $00: sequential
        COMI,R0 3
        BCTR,EQ DR_FORNLP                ; $03: FOR direct address
        BCTR,UN DR_GOTO                  ; $01/$02: line-number goto/gosub
DR_SEQ:
        LODI,R0 (IDX_SWSTK*16)+IDX_TMP
        ZBSR *VREG16_TO_REG16             ; TMP = SWSTK (next-line pointer)
        BCTR,UN DR_LP
DR_FORNLP:
        ; FOR/NEXT loop-back: GOTOH:GOTOL is a direct program-store address.
        EORZ,R0
        STRA,R0 GOTOFLG
        LODI,R0 (IDX_GOTO*16)+IDX_TMP
        ZBSR *VREG16_TO_REG16             ; TMP = GOTOH:GOTOL (direct program-store address)
        BCTR,UN DR_LP
DR_GOTO:
        ; GOTOFLG=$01 (GOTO) or $02 (GOSUB, return addr already on GSBASE).
        EORZ,R0
        STRA,R0 GOTOFLG
        LODI,R0 (IDX_GOTO*16)+IDX_LNUM
        ZBSR *VREG16_TO_REG16             ; LNUMH:LNUML = GOTOH:GOTOL (target line; V4.25: direct,
        BSTA,UN FIND_LINE                ; [+1] sets TMPH:TMPL
        BCTA,UN DR_LP
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
        BCTR,EQ TMR_MATCH                 ; row exhausted: full match
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
TMR_MATCH:
        ZBSR *VEATWORD                    ; consume any trailing garbage
        RETC,UN
TMR_MISS:
        LODI,R2 $FF
        RETC,UN

; =============================================================================
;  TL_HEXVAL -- test/convert one uppercase hex digit character
; In:  R0 = character to test
; Out: is a hex digit ('0'-'9' or 'A'-'F'): R0 = value (0-15), R2 = 0
;      not a hex digit: R2 = $FF (R0 clobbered)
; Clobbers: R0, R2
TL_HEXVAL:
        STRZ,R2                           ; R2 = original char (temp use)
        SUBI,R0 A'0'
        COMI,R0 9
        BCTR,GT TL_HV_ALPHA                ; >9: not 0-9, check A-F
        LODI,R2 0                          ; matched: R0 already = value (0-9)
        RETC,UN
TL_HV_ALPHA:
        LODZ,R2                            ; R0 = original char again
        SUBI,R0 A'A'
        COMI,R0 5
        BCTR,GT TL_HV_NONE                  ; not 'A'-'F' either
        ADDI,R0 10                          ; rebase 0-5 -> 10-15
        LODI,R2 0
        RETC,UN
TL_HV_NONE:
        LODI,R2 $FF
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
;      counting the NUL (TRY_STORE_LINE adds +3 for header+NUL itself)
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
        LODA,R0 *IPH
        SUBI,R0 A'0'
        COMI,R0 9
        BCTR,GT TL_DEC_DONE                 ; non-digit: stop accumulating
        STRA,R0 SC0                          ; this digit's value (0-9)
        ZBSR *VINC_IP
        LODI,R0 10
        BSTA,UN TL_MULADD                    ; EXP = EXP*10 + SC0
        BCTR,UN TL_DEC_LP
TL_DEC_DONE:
        LODI,R0 TOK_DEC
TL_EMIT_CONST:                                ; V4.24: shared by TL_HEX_DONE
        STRA,R0 TOKBUF,R3+                    ;   In: R0 = TOK_DEC or TOK_HEX
        LODA,R0 EXPH
        STRA,R0 TOKBUF,R3+
        LODA,R0 EXPL
        STRA,R0 TOKBUF,R3+
        BCTA,UN TL_LP
TL_MAYBEHEX:
        LODI,R1 1
        LODA,R0 *IPH,R1                      ; peek the char AFTER '$'
        BSTA,UN TL_HEXVAL
        COMI,R2 $FF
        BCTR,EQ TL_DOLLAR_PLAIN               ; not a hex digit: plain '$'
        ZBSR *VINC_IP                          ; consume '$'
        ZBSR *VCLR_EXP
TL_HEX_LP:
        LODA,R0 *IPH
        BSTA,UN TL_HEXVAL
        COMI,R2 $FF
        BCTR,EQ TL_HEX_DONE
        STRA,R0 SC0                            ; this digit's value (0-15)
        ZBSR *VINC_IP                         
        LODI,R0 16
        BSTR,UN TL_MULADD                      ; EXP = EXP*16 + SC0
        BCTR,UN TL_HEX_LP
TL_HEX_DONE:
        LODI,R0 TOK_HEX
        BCTR,UN TL_EMIT_CONST                 
TL_DOLLAR_PLAIN:
        LODA,R0 *IPH                             ; the '$' itself
        BCTA,UN TL_COPY1

; =============================================================================
;  TL_MULADD -- EXP = EXP*<multiplier> + SC0 (shared by TL_DECIMAL's x10 and
;  TL_MAYBEHEX's x16 accumulation -- both are "scale then add the next
;  digit", differing only in the multiplier, so one helper serves both
;  instead of two near-identical unrolled blocks). Same repeated-add
;  technique as PARSE_S16/PARSE_U16's PU16_M10 (not reused directly: that
;  code has no standalone entry point, see TOKENIZE_LINE's header).
; In:  R0 = multiplier (10 or 16); SC0 = digit to add; EXPH:EXPL = value;
;      caller's R3 = TOKBUF write index
; Out: EXPH:EXPL = EXPH:EXPL * R0 + SC0; R3 unchanged (preserved via R3SAVE)
; Clobbers: R0, R3, TKH, TKL, R3SAVE
TL_MULADD:
        STRA,R3 R3SAVE                       ; preserve caller's R3
        STRZ,R3                              
        LODI,R0 (IDX_EXP*16)+IDX_TK          ; V4.24: TK = EXP (value before
        ZBSR *VREG16_TO_REG16                ;   scaling); TK, not TMP
        ZBSR *VCLR_EXP
TL_MULADD_LP:
        LODA,R0 EXPL
        ADDA,R0 TKL
        ZBSR *VCARRY_INTO_EXPH
        LODA,R0 EXPH
        ADDA,R0 TKH
        STRA,R0 EXPH
        BDRR,R3 TL_MULADD_LP
        LODA,R3 R3SAVE                       ; restore caller's R3
        LODA,R0 EXPL
        ADDA,R0 SC0
        ZBSR *VCARRY_INTO_EXPH
        RETC,UN
TL_DONE:
        LODI,R0 0
        STRA,R0 TOKBUF,R3+                        ; NUL-terminate
     
; =============================================================================
;  TRY_STORE_LINE -- Store or delete a numbered line if IP starts with a digit
; In:  IPH:IPL -> input buffer
; Out: CC=GT if line stored/deleted; CC=EQ if not a numbered line
; Clobbers: R0, R1, R2, R3, EXPH, EXPL, LNUMH, LNUML, IPH, IPL, TMPH, TMPL,
;   PEH, PEL, TKH, TKL
TRY_STORE_LINE:
        LODA,R0 *IPH
        SUBI,R0 A'0'                     ; unsigned range test (COM=1 global):
        COMI,R0 9                        ; R0 wraps large if char < '0'
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
        ADDI,R3 3                          ; +2 header +1 NUL = total record size
        BSTR,UN OPEN_GAP                  ; make room at TMP; PE += R3 (V4.24: relative
                                          ;   again, offset 62 of 63)
        LODI,R1 2                         ; write the 2-byte line-number header
TSL_HDR:
        LODA,R0 LNUMH,R1-                 ; R1 2->1: LNUML; 1->0: LNUMH
        STRA,R0 *TMPH,R1                  ; -> TMP[1] then TMP[0]
        BRNR,R1 TSL_HDR
        ZBSR *VINC_TMP
        ZBSR *VINC_TMP
        LODI,R1 $FF                       
        SUBI,R3 2                         
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
; Clobbers: R0, TMPH:TMPL, EXPH:EXPL
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
;  CMP_TMP_PE -- Compare TMPH:TMPL against PEH:PEL (16-bit unsigned compare
;  via a carry test on the low byte -- ported from uBASIC2650).
; Out: CC=GT if TMP >  PE (hi bytes differed)
;      CC=LT if TMP <  PE (hi bytes differed, or hi equal and lo byte borrowed)
;      CC=EQ if hi bytes equal and lo byte did not borrow (TMP >= PE there)
; Clobbers: R0
CMP_TMP_PE:
        LODA,R0 TMPH
        SUBA,R0 PEH
        RETC,GT
        RETC,LT
        LODA,R0 TMPL
        SUBA,R0 PEL
        TPSL $01
        RETC,UN

; =============================================================================
;  ADV_TMP_PAST_REC -- advance TMPH:TMPL past a stored record (2-byte header,
;  NUL-terminated body, and the NUL itself), landing on the next record
;  (ported from uBASIC2650).
; In:  TMPH:TMPL -> start of a record (its line-number hi byte)
; Out: TMPH:TMPL -> start of the next record
; Clobbers: R0
ADV_TMP_PAST_REC:
        ZBSR *VINC_TMP
APR_LP:
        ZBSR *VINC_TMP
        LODA,R0 *TMPH
        BCTR,EQ APR_DONE                 ; NUL: body ends here
        COMI,R0 TOK_HEX                  ; unsigned: >= TOK_HEX ($FD)?
        BCTR,LT APR_LP                   ; ordinary body byte: keep scanning
        ZBSR *VINC_TMP                   ; constant token: skip its 2-byte
        ZBSR *VINC_TMP                   ;   payload without NUL-testing it
        BCTR,UN APR_LP
APR_DONE:
        ZBSR *VINC_TMP                    ; skip the NUL itself
        RETC,UN

; =============================================================================
;  FIND_LINE -- Search for line LNUMH:LNUML in program store
; Out: TMPH:TMPL = record start if found; CC=EQ found, CC=GT not found.
; Clobbers: R0, R1, TMPH, TMPL
FIND_LINE:
        BSTR,UN FIND_INS                 ; [+1]
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
        BCTR,LT FI_RET
        ; hi bytes equal: check lo
        LODI,R1 1
        LODA,R0 LNUML
        COMA,R0 *TMPH,R1                  ; peek stored.lo at TMP+1 (indirect-
                                           ; indexed); unsigned (COM=1 global)
        BCTR,GT FI_ADV
FI_RET:
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
        STRA,R0 TMPH
        LODI,R0 >FUNC_TAB
        STRA,R0 TMPL
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
        LODI,R0 (IDX_TMP*16)+IDX_SAVE
        ZBSR *VREG16_TO_REG16             ; [+1] SAVE = TMP
        CPSL PSW_WC
        LODA,R0 SAVEL
        ADDA,R0 EXPL
        STRA,R0 EXPL
        PPSL PSW_WC
        LODA,R0 SAVEH
        ADDA,R0 EXPH
        STRA,R0 EXPH
        CPSL PSW_WC
        BCTA,UN EAM_LO_LOOP

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
        STRA,R0 TMPH
        LODI,R0 >FUNC_TAB
        STRA,R0 TMPL
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
;  CHECK_LPAREN -- require and consume '(' at IP (v4.5, FUNCCONT-01 fix)
;  CHECK_RPAREN -- require and consume ')' at IP (v4.5, FUNCCONT-01 fix)
; Out: IP advanced past parens; tail-jumps to JSYNERR (no return) if missing
; Clobbers: R0, R1
CHECK_LPAREN:
        LODI,R1 A'('
        DB $EC
        ; drop through
CHECK_RPAREN:
        LODI,R1 A')'
        ZBSR *VWSKIP
        COMZ,R1
        BCFA,EQ JSYNERR
        ZBRR *VINC_IP            ; tail call: consumes ')' and returns to caller

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
; In:  IPH:IPL -> first char of factor
; Out: EXPH:EXPL = value
; Clobbers: R0, R1, SC0
PARSE_FACTOR:
        LODA,R0 *IPH
        COMI,R0 TOK_DEC                  ; V4.22: tokenized decimal constant?
        BCTR,EQ PF_CONST
        COMI,R0 TOK_HEX                  ; tokenized hex constant?
        BCTR,EQ PF_CONST
        COMI,R0 A'A'
        BCTR,LT PF_NUM
        COMI,R0 A'Z'+1
        BCTR,LT PF_LOADVAR
PF_NUM:
        BCTA,UN PARSE_S16                ; tail call: PARSE_S16's RETC,UN returns to our caller
PF_CONST:
        ZBSR *VINC_IP                    ; [+1] past the token byte
        ZBRR *VFETCH16_IP                ; tail: EXP = payload, IP past it; returns to our caller

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
; Clobbers: R0, NEGFLG, EXPH, EXPL
PARSE_S16:
        EORZ,R0
        STRA,R0 NEGFLG
        LODA,R0 *IPH
        COMI,R0 A'-'
        BCFR,EQ PS16_UN
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
        SUBI,R0 A'0'
        COMI,R0 9                        ; BUG-PU16-01 FIX: was COMI,R0 10 (=
        BCTR,GT PRO_NONE; surrogate for JSYNERR  ; ':'-'0'), wrongly letting a
                                          ; leading ':' start a number
PU16_LP:
        LODA,R0 *IPH
        SUBI,R0 A'0'
        COMI,R0 9
        BCTR,GT PU16_RET ; RETC,GT
PU16_DIG:
        STRA,R0 SC0                      ; R0 already = char-'0' from the test above
        ZBSR *VINC_IP
PU16_DNC:
        STRA,R3 R3SAVE                   ; save SW stack pointer
        LODI,R0 (IDX_EXP*16)+IDX_TMP
        ZBSR *VREG16_TO_REG16
        ZBSR *VCLR_EXP
        LODI,R3 10
PU16_M10:
        LODA,R0 EXPL
        ADDA,R0 TMPL
        ZBSR *VCARRY_INTO_EXPH
        LODA,R0 EXPH
        ADDA,R0 TMPH
        STRA,R0 EXPH
        BDRR,R3 PU16_M10
        LODA,R3 R3SAVE                   ; restore SW stack pointer
        LODA,R0 EXPL
        ADDA,R0 SC0
        ZBSR *VCARRY_INTO_EXPH
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
;  DO_NEG_FUNC -- NEG(a), arithmetic negation (v4.5: paren-bounded, see
;  CHECK_LPAREN/CHECK_RPAREN/FUNC_CONT for the FUNCCONT-01 fix)
DO_NEG_FUNC:
        ZBSR *VCHECK_LPAREN
        ZBSR *VPARSE_EXPR
        ZBSR *VCHECK_RPAREN
        BSTR,UN NEG_EXP_BODY              ; real call now (was tail-jump) so
        ZBRR *VFUNC_CONT                  ; control returns here for FUNC_CONT

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
;  DO_ABS_FUNC -- ABS(a), absolute value (v4.5: paren-bounded, see
;  CHECK_LPAREN/CHECK_RPAREN/FUNC_CONT for the FUNCCONT-01 fix)
DO_ABS_FUNC:
        ZBSR *VCHECK_LPAREN
        ZBSR *VPARSE_EXPR
        ZBSR *VCHECK_RPAREN
        BSTR,UN ABS_EXP                    ; real call now (was fall-through)
        ZBRR *VFUNC_CONT                   ; control returns here for FUNC_CONT

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
;PARSE_2ARGS:
        ZBSR *VCHECK_LPAREN              ; require+consume '(' 
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
        ZBSR *VCHECK_LPAREN
        ZBSR *VPARSE_EXPR
        ZBSR *VCHECK_RPAREN
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
        STRA,R0 TMPH
        LODI,R0 >FUNC_TAB
        STRA,R0 TMPL
        BCTR,UN DTT_START
DLS_KWTOK:
        SUBI,R0 TOK_STMT_BASE             ; statement row index
        STRZ,R2
        LODI,R0 <KW_TAB
        STRA,R0 TMPH
        LODI,R0 >KW_TAB
        STRA,R0 TMPL
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
        LODI,R0 (IDX_EXP*16)+IDX_SC0
        ZBSR *VREG16_TO_REG16           ; SC0:SC1 = |EXP|
        ZBSR *VCLR_EXP                  ; clear EXP (accumulator starts at 0)
        RETC,UN

; =============================================================================
;  CARRY_INTO_EXPH -- store R0 into EXPL, propagate carry into EXPH.
; In:  R0 = EXPL + <addend>, i.e. caller has just done LODA,R0 EXPL /
;      ADDA,R0 <addend> and Carry still reflects that add (STRA and a
;      BSTA/BCTA call in between don't disturb it -- confirmed empirically
;      via pipbug_wrap regression; same technique already used in
;      uBASIC2650's CARRY_INTO_EXPH).
; Out: EXPL = R0; EXPH += 1 iff the add into EXPL carried
; Clobbers: R0
CARRY_INTO_EXPH:
        STRA,R0 EXPL
        TPSL $01
        RETC,LT
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
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
        BCTR,GT MU_ADD
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
        TPSL $01                         ; BUG-MUL-01 FIX: test carry, not raw CC
        BCTR,EQ MU_TNB                   ; C=1 (no borrow): skip hi decrement
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
        BCTR,EQ JERRDIVZER
        
        ; not zero
DV_NZ:
        BSTA,UN SETUP_MULDIV             ; [+1] sign setup, |EXP|->SC0:SC1, EXP=0
DV_LP:
        LODA,R0 TMPH
        SUBA,R0 SC0
        BCTR,LT MU_DONE ; DV_DONE
        BCTR,GT DV_SUB
        LODA,R0 TMPL
        SUBA,R0 SC1
        TPSL $01
        BCFR,EQ MU_DONE
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
        ZBSR *VINC_EXP  
        BCTR,UN DV_LP

JERRDIVZER:
        LODI,R0 ERR_DIV_ZERO
        ZBRR *VDO_ERROR 

; =============================================================================
;  PRINT_S16 -- Print signed 16-bit value EXPH:EXPL as decimal
; In:  EXPH:EXPL = signed value
; Out: decimal digits written to COUT
; Clobbers: R0, R1, R2, R3, SC0 (TMP is NOT clobbered)
PRINT_S16:
        LODA,R0 EXPH             ; get high byte & establish CC
        BCTR,LT IS_NEG           ; branch if negative (bit 7 set)

        IORA,R0 EXPL             ; Check for ZERO
        BCFR,EQ PS_DIGITS        ; >0, flow into subtract printer

        LODI,R0 A'0'             ; Handle Zero
        ZBRR *VCOUT              ; Print '0' and tail call return
IS_NEG:
        LODI,R0 A'-'
        ZBSR *VCOUT
        BSTA,UN NEG_EXP_BODY     ; Negate, making EXPH:EXPL positive
PS_DIGITS:
        EORZ,R0
        STRZ,R2                  ; R2 = P10 table index (0 to 4)
        STRZ,R3                  ; R3 = leading zero flag (0=leading, >0=printing)
DIGIT_LOOP:
        LODI,R1 A'0'-1           ; R1 = ASCII digit character
SUB_LOOP:
        CPSL $08                 ; Clear WC bit for standard 8-bit math
        ADDI,R1 1                ; Increment digit
        LODA,R0 EXPL
        SUBA,R0 P10_LO,R2        ; Subtract low byte
        STRA,R0 SC0              ; Save tentatively (PRINT_S16 must not
                                 ; clobber TMP -- DO_LIST relies on that)
        PPSL $08                 ; Set WC bit (enables Carry-In/Borrow)
        LODA,R0 EXPH
        SUBA,R0 P10_HI,R2        ; Subtract high byte
        BCTR,LT BORROW           ; Borrow (C=0): result LT, stop subtracting

        ; No borrow: commit result and subtract again
        STRA,R0 EXPH
        LODA,R0 SC0
        STRA,R0 EXPL
        BCTR,UN SUB_LOOP

BORROW:
        ; Check for leading zero suppression
        COMI,R1 A'0'
        BCTR,GT PRINT_IT         ; Not '0', must print
        EORZ,R0
        IORZ,R3                  ; Test leading-zero flag
        BCFR,EQ PRINT_IT         ; Already printing a digit: print zero too
        COMI,R2 4                ; Is it the final column (1s)?
        BCFR,EQ NEXT_DIG         ; not the final column -> skip the leading zero

PRINT_IT:
        LODI,R3 1                ; Set leading zero flag
        LODZ,R1                  ; R0 = R1 (destination is always R0)
        ZBSR *VCOUT              ; Print the character
NEXT_DIG:
        ADDI,R2 1                ; Advance to next power of 10
        COMI,R2 5                ; Have we processed all 5 powers?
        BCTR,LT DIGIT_LOOP
        CPSL $08                 ; Clear WC bit
        RETC,UN                  ; Return to caller

; =============================================================================
;  SWRETURN -- Return via SW stack
; In:  R3 = SW stack pointer; SWBASE[R3] = hi, SWBASE[R3-1] = lo of return addr
; Out: jumps to address popped from SW stack
; Clobbers: R0, TEMPRETH, TEMPRETL
SWRETURN:
        LODA,R0 SWBASE,R3
        STRA,R0 TEMPRETH
        ;SUBI,R3 1
        LODA,R0 SWBASE,R3-
        STRA,R0 TEMPRETL
        SUBI,R3 1
        BCTA,UN *TEMPRETH

P10_HI:
        db $27, $03, $00, $00, $00
P10_LO:
        db $10, $E8, $64, $0A, $01

; =============================================================================
;  RDLINE -- Read a line from input into IBUF with echo and backspace
; In:  nothing
; Out: IBUF = NUL-terminated input line.
;      R3 is used as an index into IBUF instead (cheaper bounds check and
;      backspace than full 16-bit pointer arithmetic). R3=$FF means empty
;      (matches the SWBASE empty-stack convention) since the 2650's ",R3+"
;      addressing mode pre-increments before the access.
; Clobbers: R0, R1, R3
RDLINE:
        LODI,R3 $FF                      ; R3 = empty-buffer sentinel (pre-inc convention)
RL_LP:
        BSTA,UN CHIN                     ; [+1] blocking read
        COMI,R0 NUL
        BCTR,EQ RL_EOL
        STRZ,R1
        COMI,R1 CR
        BCTR,EQ RL_EOL
        COMI,R1 LF
        BCTR,EQ RL_EOL
        COMI,R1 BS
        BCTR,EQ RL_BS
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
WSKIP_LOOP:
        ZBSR *VINC_IP           ; Advance IP (2 bytes)
WSKIP:
        LODA,R0 *IPH            ; Read char at IP (3 bytes)
        COMI,R0 SP              ; Is it a space? (2 bytes)
        BCTR,EQ WSKIP_LOOP      ; Yes -> loop back to increment IP (2 bytes)
        COMI,R0 $00             ; Refresh CC flags for R0 (2 bytes)
WSKIPRET:
        RETC,UN                 ; Return to caller (1 byte)

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
        BSTA,UN DO_END                   ; [+1] clears SWSP, FORSP, GOTOFLG, RUNFLG
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
        LODI,R0 0                    ; result accumulator = 1
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
        DB CR, LF, "uBASIC 2650 V4.28", CR, LF, "Bytes Free:",NUL

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
        DB <DO_LET,    >DO_LET
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
        DB NUL,    <PE_NOFUNC,   >PE_NOFUNC        ; No match handler

; Row order MUST match FUNC_TAB's row order above (token = TOK_FUNC_BASE +
; row index); assembler-computed, see TOK_STMT_OFS note above.
TOK_FUNC_OFS:
        DB >(FNH_ABS-FUNC_TAB), >(FNH_AND-FUNC_TAB), >(FNH_NEG-FUNC_TAB)
        DB >(FNH_NOT-FUNC_TAB), >(FNH_OR-FUNC_TAB),  >(FNH_PEEK-FUNC_TAB)
        DB >(FNH_RND-FUNC_TAB), >(FNH_USR-FUNC_TAB), >(FNH_XOR-FUNC_TAB)

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
; immediately before LNUMH: DO_FOR copies FORVAR,LNUM,EXP,SWSTK (7 bytes) as-is.
GOTOFLG RES 1       ; $00=sequential $01=GOTO $02=GOSUB $03=FOR direct addr
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
                     ; (R2 is reserved for DO_LET/DO_FOR/DO_INPUT across
                     ; PARSE_EXPR - see header - so must never leak here)
TEMPRETH RES 1      ; SW return address hi
TEMPRETL RES 1      ; SW return address lo

;  --- Flags & Stuff --- 
RUNFLG  RES 1       ; $01=running $00=immediate
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
VARS    RES 52      ; A-Z variables 2 bytes each

; =============================================================================
;  Pre-loaded SHOWCASE program
;
;  Line format: <lineno_hi> <lineno_lo> <body_ASCII> <NUL>
;  Lines  10-190: feature demos (PRINT, CHR$, arithmetic, comparisons, GOTO loop)
;  Lines 192-218: FOR/NEXT and GOSUB/RETURN demos
;  Lines 220-240: function demos (ABS/NEG/AND/OR/XOR/NOT/PEEK/POKE/RND/LIST)
;  Lines 300-510: Mandelbrot set renderer (v4.3: widened, C=-144..28 step 4,
;                 44 cols vs v4.2's 32; row range I=-64..56 step 6 unchanged)
;  Line  530:     GOSUB subroutine (PRINT "sub"; / RETURN)
;
;  Format: DB hi,lo,"text",$00  -- hi-then-lo matches DR_EXEC record format.
;  $22=DQ $3B=semicolon  in-string chars that need escaping.
; =============================================================================
PROG:
        DB 0,10,"REM uBASIC 2650 - SHOWCASE V4.6",$00
        DB 0,20,"PRINT ",$22,"-- uBASIC 2650 V4.6 Showcase --",$22,$00
        DB 0,30,"PRINT ",$22,"--- PRINT / CHR$ ---",$22,$00                    ; 30  PRINT "--- PRINT / CHR$ ---"
        DB 0,40,"PRINT CHR$(65)",$3B,"CHR$(66)",$3B,"CHR$(67)",$00             ; 40  PRINT CHR$(65);CHR$(66);CHR$(67)
        DB 0,50,"PRINT ",$22,"--- ARITHMETIC ---",$22,$00                      ; 50  PRINT "--- ARITHMETIC ---"
        DB 0,60,"PRINT ",$22,"3+4=",$22,$3B,"3+4",$3B,$22,"  10-3=",$22,$3B,"10-3",$3B,$22,"  6*7=",$22,$3B,"6*7",$00  ; 60  PRINT "3+4=";3+4;"  10-3=";10-3;"  6*7=";6*7
        DB 0,70,"PRINT ",$22,"20/4=",$22,$3B,"20/4",$3B,$22,"  17%5=",$22,$3B,"17%5",$00  ; 70  PRINT "20/4=";20/4;"  17%5=";17%5
        DB 0,80,"PRINT ",$22,"--- COMPARISONS ---",$22,$00                     ; 80  PRINT "--- COMPARISONS ---"
        DB 0,90,"IF 5>3 THEN PRINT ",$22,"5>3 ok",$22,$00                      ; 90  IF 5>3 THEN PRINT "5>3 ok"
        DB 0,100,"IF 3<5 THEN PRINT ",$22,"3<5 ok",$22,$00                     ; 100 IF 3<5 THEN PRINT "3<5 ok"
        DB 0,110,"IF 3>=3 THEN PRINT ",$22,"3>=3 ok",$22,$00                   ; 110 IF 3>=3 THEN PRINT "3>=3 ok"
        DB 0,120,"IF 4<>3 THEN PRINT ",$22,"4<>3 ok",$22,$00                   ; 120 IF 4<>3 THEN PRINT "4<>3 ok"
        DB 0,130,"IF 3=3 THEN PRINT ",$22,"3=3 ok",$22,$00                     ; 130 IF 3=3 THEN PRINT "3=3 ok"
        DB 0,140,"PRINT ",$22,"--- LOOP via GOTO ---",$22,$00                  ; 140 PRINT "--- LOOP via GOTO ---"
        DB 0,150,"I=1",$00                                                      ; 150 I=1
        DB 0,160,"IF I>5 THEN GOTO 190",$00                                    ; 160 IF I>5 THEN GOTO 190
        DB 0,170,"PRINT I",$3B,$00                                              ; 170 PRINT I;
        DB 0,180,"I=I+1",$00                                                    ; 180 I=I+1
        DB 0,185,"GOTO 160",$00                                                 ; 185 GOTO 160
        DB 0,190,"PRINT ",$22,"",$22,$00                                        ; 190 PRINT ""
        DB 0,192,"PRINT ",$22,"--- FOR/NEXT LOOP ---",$22,$00                  ; 192 PRINT "--- FOR/NEXT LOOP ---"
        DB 0,194,"FOR I=1 TO 5",$00                                             ; 194 FOR I=1 TO 5
        DB 0,196,"PRINT I",$3B,$00                                              ; 196 PRINT I;
        DB 0,198,"NEXT I",$00                                                   ; 198 NEXT I
        DB 0,199,"PRINT ",$22,"",$22,$00                                        ; 199 PRINT ""
        DB 0,201,"PRINT ",$22,"--- FOR STEP 2 ---",$22,$00                     ; 201 PRINT "--- FOR STEP 2 ---"
        DB 0,203,"FOR I=0 TO 10 STEP 2",$00                                    ; 203 FOR I=0 TO 10 STEP 2
        DB 0,205,"PRINT I",$3B,$00                                              ; 205 PRINT I;
        DB 0,207,"NEXT I",$00                                                   ; 207 NEXT I
        DB 0,208,"PRINT ",$22,"",$22,$00                                        ; 208 PRINT ""
        DB 0,210,"PRINT ",$22,"--- GOSUB/RETURN ---",$22,$00                   ; 210 PRINT "--- GOSUB/RETURN ---"
        DB 0,212,"GOSUB 530",$00                                                ; 212 GOSUB 530
        DB 0,214,"GOSUB 530",$00                                                ; 214 GOSUB 530
        DB 0,216,"PRINT ",$22,"",$22,$00                                        ; 216 PRINT ""
        DB 0,220,"PRINT ",$22,"--- FUNCTIONS ---",$22,$00                       ; 220 PRINT "--- FUNCTIONS ---"
        DB 0,222,"PRINT ",$22,"ABS(-7)=",$22,$3B,"ABS(-7)",$3B,$22,"  NEG(7)=",$22,$3B,"NEG(7)",$00  ; 222 PRINT "ABS(-7)=";ABS(-7);"  NEG(7)=";NEG(7)
        DB 0,224,"PRINT ",$22,"AND(12,10)=",$22,$3B,"AND(12,10)",$3B,$22,"  OR(12,10)=",$22,$3B,"OR(12,10)",$00  ; 224 PRINT "AND(12,10)=";AND(12,10);"  OR(12,10)=";OR(12,10)
        DB 0,226,"PRINT ",$22,"XOR(12,10)=",$22,$3B,"XOR(12,10)",$3B,$22,"  NOT(0)=",$22,$3B,"NOT(0)",$00  ; 226 PRINT "XOR(12,10)=";XOR(12,10);"  NOT(0)=";NOT(0)
        DB 0,228,"POKE 8000,42",$00                                             ; 228 POKE 8000,42
        DB 0,230,"PRINT ",$22,"PEEK(8000)=",$22,$3B,"HEX$(PEEK(8000))",$00            ; 230 PRINT "PEEK(8000)=";PEEK(8000)
        DB 0,232,"PRINT ",$22,"RND(100)=",$22,$3B,"RND(100)",$3B,$22,"  RND(100)=",$22,$3B,"RND(100)",$00  ; 232 PRINT "RND(100)=";RND(100);"  RND(100)=";RND(100)
        DB 0,234,"PRINT ",$22,"",$22,$00                                        ; 234 PRINT ""
        DB 0,236,"PRINT ",$22,"--- LIST 40,60 ---",$22,$00                      ; 236 PRINT "--- LIST 40,60 ---"
        DB 0,238,"LIST 40,60",$00                                               ; 238 LIST 40,60
        DB 0,240,"GOTO 300",$00                                                 ; 240 GOTO 300
        DB 1,44,"PRINT ",$22,"--- MANDELBROT ---",$22,$00                      ; 300 PRINT "--- MANDELBROT ---"
        DB 1,54,"I=-64",$00                                                     ; 310 I=-64
        DB 1,64,"IF I>56 THEN GOTO 510",$00                                    ; 320 IF I>56 THEN GOTO 510
        DB 1,74,"D=I",$00                                                       ; 330 D=I
        DB 1,84,"C=-144",$00                                                    ; 340 C=-144 (widened from -120)
        DB 1,94,"IF C>28 THEN GOTO 480",$00                                    ; 350 IF C>28 THEN GOTO 480 (widened from 4)
        DB 1,104,"A=C",$00                                                      ; 360 A=C
        DB 1,105,"B=D",$00                                                      ; 361 B=D
        DB 1,106,"E=0",$00                                                      ; 362 E=0
        DB 1,107,"N=1",$00                                                      ; 363 N=1
        DB 1,114,"IF N>16 THEN GOTO 420",$00                                   ; 370 IF N>16 THEN GOTO 420
        DB 1,124,"IF E>0 THEN GOTO 410",$00                                    ; 380 IF E>0 THEN GOTO 410
        DB 1,134,"T=A*A/64-B*B/64+C",$00                                       ; 390 T=A*A/64-B*B/64+C
        DB 1,144,"B=2*A*B/64+D",$00                                             ; 400 B=2*A*B/64+D
        DB 1,145,"A=T",$00                                                      ; 401 A=T
        DB 1,154,"IF A*A/64+B*B/64>256 THEN IF E=0 THEN E=N",$00               ; 410 IF A*A/64+B*B/64>256 THEN IF E=0 THEN E=N
        DB 1,164,"N=N+1",$00                                                    ; 420 N=N+1
        DB 1,165,"IF N<=16 THEN GOTO 370",$00                                  ; 421 IF N<=16 THEN GOTO 370
        DB 1,174,"IF E>0 THEN PRINT CHR$(E+32)",$3B,$00                        ; 430 IF E>0 THEN PRINT CHR$(E+32);
        DB 1,184,"IF E=0 THEN PRINT CHR$(32)",$3B,$00                          ; 440 IF E=0 THEN PRINT CHR$(32);
        DB 1,194,"C=C+4",$00                                                    ; 450 C=C+4
        DB 1,204,"GOTO 350",$00                                                 ; 460 GOTO 350
        DB 1,224,"PRINT",$00                                                    ; 480 PRINT
        DB 1,234,"I=I+6",$00                                                    ; 490 I=I+6
        DB 1,244,"GOTO 320",$00                                                 ; 500 GOTO 320
        DB 1,254,"END",$00                                                      ; 510 END
        DB 2,18,"PRINT ",$22,"sub",$22,$3B,$00                                 ; 530 PRINT "sub";
        DB 2,20,"RETURN",$00                                                   ; 532 RETURN
SHOWCASE_END:

; =============================================================================
; TO-DO 
;   - Full pass over the file for inaccurate/stale comments
;   - Review scratch lifetimes: TMP,LNUM,SC0/1,TKH/L,ARGA,POWCNT,SAVE
;     TEMPRET, and potential to consolidate
;   - Add $FFFF hex entry on direct path, leveraging TL_HEXVAL,TL_MULADD like  
;     TL_MAYBE_HEX
;   - SWRETURN could pop via POP_SAVE_TO_TMP and BCTA,UN *TMPH (about -12
;     bytes, -2 RAM for TEMPRET) but it would nest one extra RAS level at the
;     point where PARSER_RET can already be at depth 6-7 -- needs a depth audit.
;   - PARSE_VAR_SAVE/DL_STORE use var in SC0 and R2 - clean up as per pbasic 
;   - In why is LODA,R0 *IPH,R1 necessary in DP_ITEM 

        END
