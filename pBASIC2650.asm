; pBASIC2650.asm - PoC Minimal Tiny BASIC for Signetics 2650
; v0.30 - Sep 2026
; Vincent Crabtree - MIT License
;
; TARGET
;   Standalone Signetics 2650, ORG 0, 8 KB address space.
;   I/O routines embedded; no PIPBUG ROM required.
;   Goal: <2 KB ROM at $0000, RAM at $1000 upwards.
;
; BUILD
;   gcc -Wall -O2 -o asm2650 asm2650.c
;   gcc -O2 -DGAMER -o pipbug_wrap pipbug_wrap.c
;
;   ./asm2650 pBASIC2650.asm pBASIC2650.hex
;   grep -n "^CHIN \|^COUT \|^ROMEND " pBASIC2650.LST
;   ./pipbug_wrap --entry 0 --chin 0x<addr> --cout 0x<addr> pBASIC2650.hex
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
;   R2  current variable's byte offset into VARS (2*(letter-'A')), set by
;       PARSE_VAR_SAVE, consumed by DL_STORE; preserved across expressions
;   R3  loop counter / Expression SW stack 
;
; RAS / RECURSION
;   The 2650 has an 8-level hardware return-address stack, not user accessible.
;   BSxx/ZBSR consume a RAS entry; BCxx/ZBRR do not.
;   PARSE_EXPR guards against excessive hardware-stack depth.
;   Parentheses use a software depth counter to reduce RAS usage.
;
; BASIC LANGUAGE
;   26 uppercase variables (A-Z), 16-bit signed integers.
;   No arrays or string variables.
;   PRINT supports string literals.
;   No built-in functions.
;   Operators: + - * / = <

;
;        KNOWN LIMITATIONS
;
; UPPERCASE only apart from PRINT "String literals"
;
; STATEMENT DISPATCH matches ONLY the first character of a line against
;   a 9-entry table (A/E/G/I/L/N/P/R/W for ASK/END/GOTO/IF/LIST/NEW/
;   PRINT/RUN/WR. It first checks the 2nd is a LETTER otherwise VAR.
;
; STORAGE / INPUT
;   Program lines are append-only.
;   A new line number must be greater than the current highest,
;   or equal to it to replace/delete the last line.
;   Input is not bounds-checked; overlong lines will corrupt memory.
;
; NESTED UNARY MINUS ( -(-(-(...  about 6 deep ) recurses on the hardware RAS
;   with no guard and can wrap it; parentheses are guarded (PDEPTH/PE_RAS_LIMIT),
;   unary minus is not.
;
; DELIBERATE LIMITS
;   No BODMAS/PEMDAS precedence - all operators are equal and evaluate
;   left to right, so "1+2*3" evaluates as "(1+2)*3" = 9, not 7. Use
;   explicit parentheses for grouping.
;   Parenthesis nesting is RAS-limited via a software depth counter;
;   deepest guaranteed safe level is 3, e.g. "(((expr)))". Excess depth
;   generates ERR_NEST.
;   Minimal syntax validation; malformed constructs may produce the
;   normal syntax/runtime error rather than a specialised diagnostic.
;
; =============================================================================
; VERSION HISTORY (pBASIC2650)
; =============================================================================
;
; v0.30 (Sep 2026) - ROMEND: $05B2 (1458 bytes)   [v0.29 as uploaded: $0645 / 1605]
;   BUG FIXES (found while checking line entry after NEW)
;   - Line entry: TSL_CPYDONE's "tail call" into TMP_TO_ET was a ZBSR, so after
;     storing a line it returned and FELL THROUGH into FIND_LINE, whose CC=EQ
;     made REPL also execute the stored line - every stored line printed ?4
;     (and a replaced line printed ?4@garbage). Now ZBRR; returns CC=GT. 0 bytes.
;   - RUN left RUNFLG=1 after a normal program end (TMP==PE returned via RETC,EQ
;     without clearing it), so a later immediate-mode error printed a stale
;     "@line". DR_LP now clears on GT or EQ (BCFA,LT CLR_RUNFLG). -1 byte.
;   - Operator-first lines ("-A", "*A", "/A", "=A", "<A") were silently accepted
;     (or ran a division): MD_SCAN began at table row 0, the operator rows. It now
;     starts at the first statement row (literal 18), so they give ?4. 0 bytes.
;     NB asm2650 silently assembles the expression "6*3" as 6 - use literals.
;   SIZE (ROMEND $0645 -> $05B2 with the fixes above)
;   - GETLINE sets IPH:IPL itself: SET_IP_IBUF, its vector slot and 2 call sites
;     are gone. IBUF moved to $1010 (hi == lo) so IP = IBUF is one LODI + two STRA.
;     RAM total unchanged (SC1 -> IBUF -> PEH ...).
;   - MD_HIT: step-then-test keyword eater, one branch instead of two.
;   - R2 now holds the variable's VARS byte offset (2*index): PARSE_VAR_SAVE
;     drops its ADDI, DL_STORE its recompute and indexes with R2 directly;
;     PF_LOADVAR uses ADDZ,R0 instead of STRZ/ADDZ/STRZ.
;   - TRY_STORE_LINE digit test: SUBI/COMI + BCFR,GT TSL_NUM. The inverted sense
;     needs no "not equal" condition (and no db $EC skip).
;   - Dead WSKIP calls removed: DO_GOTO/DO_WR (EXPR_ATOM skips leading spaces),
;     TSL_NUM (IP already on the digit), TSL_NZ (TSL_WRITE's WSKIP_PEEK does it).
;   - NEGFLG polarity is now 0 = negate: PARSE_S16 sets it with one EORI/STRA
;     (0 exactly for '-'), ABS_TMP writes it fresh, so no clears anywhere;
;     ABS_EXP toggles $80; NEG_EXP returns on GT/LT.
;   - PRINT_S16: R2 save/restore removed (nothing needs R2 after a print, and the
;     digit path always clobbered it - only the zero path used to restore it).
;   - DO_NEW: SET_TMP_PROG + TMP_TO_ET (its IP write had no reader).
;   - Arithmetic core: DV_LP subtracts IN PLACE with a WC-chained 16-bit
;     subtract and tests the final carry (no compare pass, no scratch: the
;     remainder is dead). MULT_LOOP is a leaf (decrement-with-borrow test +
;     inline WC add) shared by DO_MUL and PU16; PU16 preloads EXP with the digit
;     so MULT_LOOP yields digit + 10*value (no final add / carry step). DO_ADD
;     uses a WC add, so CARRY_INTO_EXPH (+ slot) and EXP16_TO_TMP are gone.
;   - Relops park their partial result in R1 (EORA/STRZ,R1/COMZ,R1), not SC0.
;   - FIND_LINE/FIND_INS read the stored line's lo byte with *TMPH,R1 (indirect
;     indexed) instead of computing TMP+1 - INC16_TMP_TO_EXP is gone.
;   - Vector-table caller counts regenerated (several were stale).
;   REGRESSION: 150 inputs, output-identical to v0.29+fixes: line entry after NEW
;   (append / replace-last / delete-last / out-of-order / delete-only-line /
;   zero and large line numbers), expressions and sign sweeps, all statements,
;   error paths, ASK, GOTO/IF/NEW-in-RUN, showcase incl. Mandelbrot, and random
;   fuzz (token-soup + valid-expression). Only intended differences: operator-
;   first lines (?4). Peak hardware RAS depth identical on every test (showcase 7/8).
;   Known/untouched: -(-(-(-(-(-1))))) recurses on the hardware RAS unguarded
;   (added to KNOWN LIMITATIONS); v0.29's PU16 range test also makes ':' a syntax
;   error where it used to parse as 0.
; v0.29 (Sep 2026) - ROMEND: $$0645 (1605 bytes)
;   - Dropped the TSL_ERR indirection 
;   - DO_RUN: IPH:IPL = TMPH:TMPL now via the shared TMP_TO_ET copier
;   - DR_LP's loop-back: BCTA,UN DR_LP -> BCTR,UN DR_LP (in range here too).
;   - EATWORD inlined at MD_HIT and remoeved $ check
;   - PARSE_FACTOR/PF_LOADVAR: PARSE_FACTOR's 2-comparison A-Z test
;     replaced with the SUBI/COMI range test
;   - PU16's two digit-range tests SUBI/COMI idiom
;   - MU_DONE's post-negate CLR_NEGFLAG confirmed dead and rmeoved.
;   - TRY_STORE_LINE's digit test: tried same SUBI/COMI idea, "BCTR,NE"
;     condition this CPU doesn't have (only EQ/GT/LT/UN exist) - reverted
;
; v0.28 (Sep 2026) - Stored-program record terminator: CR -> NUL, ported from
;   uBASIC2650wip.asm's same conversion.
;   - Bodies are now NUL-terminated in storage, matching what STMT_EXEC/EXPR/
;     etc already assume for IBUF - DO_RUN executes straight out of PROG
;     (no more per-line copy into IBUF, no manufactured terminator).
;   - ADV_PAST_RECORD (shared by FIND_INS/TSL_MATCH), DLS_BLPX and
;     TSL_CPYDONE updated to match; each drops its CR compare entirely in
;     favour of the free CC=EQ a LODA already gives on a zero byte.
;     TSL_CPYDONE no longer manufactures a terminator at all - R0 is
;     already 0 from the LODA that branched there.
;   - Showcase's 54 line terminators (incl. commented-out demo lines)
;     changed $0D/CR -> $00/NUL to match.
;   - Regression: showcase RUN output unchanged; also re-tested LIST,
;     storing/deleting the last line, appending, GOTO line lookup, and
;     error line reporting - all correct.
;   - ROMEND: $0681 -> $0670 (1665 -> 1648 bytes)
; v0.27 (Sep 2026) - Ported 3 small idioms from uBASIC2650wip.asm
;   - DO_IF: BSTA,UN EXPR -> ZBSR *VPARSE_EXPR 
;   - GETLINE/GL_LP: collapsed separate CR and LF checks into one
;   - DO_ERROR: CURH/CURL -> EXPH/EXPL copy changed to loop
;   - ROMEND: $0694 -> $0681 (1684 -> 1665 bytes)
; v0.26 (Sep 2026) 
;   - PRINT_S16's tentative-low-byte scratch moved TMPL -> SC0. DO_LIST now walks 
;     TMP directly, removing TMP->IP back copies in DLS_LP and DLS_NL.
;   - ROMEND: $06BB -> $0694 (1684 bytes)
; v0.24 (Sep 2026) - Vector audit refresh 
;   - Vector audit: ABS_TMP, EXP16_TO_ET and DR_LP vectors removed; TMP_TO_ET
;     TMP_TO_SWSTK, CARRY_INTO_EXPH, INC_ET added
;   - Comments, - Stale comment pass, Commented out `!` relpop modifier 
;   - ROMEND: $06DA -> $06A2 (1698 bytes)
; v0.23 (Sep 2026) - TMP_TO_ET extraction; DO_END/CLR_RUNFLG merge; DO_MUL/
;     DO_DIV dispatch reworked to key off the R1 table offset directly
;   - TMP_TO_ET replaces three separate inline "TMPH:TMPL ->
;     somewhere" copies.  TMP_TO_SWSTK folded in as another thin wrapper.
;   - DO_END absorbed by CLR_RUNFLG.
;   - ROMEND: $072C -> $06da (1836 -> 1754 bytes).
; v0.22 (Sep 2026) - Refactor DO_MUL/DO_DIV to share one setup body and pick 
;   - their loop via R1 used in OPS_HIT.
;   - ROMEND: $06da (1754 bytes)
; v0.21 (Sep 2026) - Refactor for size inc OPS_LP. 
;   - ROMEND: $075A (1882 bytes)
; v0.20 (Sep 2026)
;   - SUBA/COMA audit - set COM=1 once in MAIN to eliminate redundant PPSL/CPSL loads
;   - ROMEND: $07D3 -> $07C4 
; v0.19 (Sep 2026)
;   - Refactor STMT_EXEC for size.
;   - Fixed DO_LTOP '<' comparison bug caused by raw CC range limits
;   - ROMEND: $07CB -> $07D3 
; v0.18 (Sep 2026)
;   - Added '!' relop-invert modifier (!=, !<) using a BANG flag and XORing
;     into the boolean result at convergence.
;   - ROMEND: $07BC -> $07D6 
; v0.17 (Sep 2026)
;   - Factored out WSKIP_PEEK subroutine and ADV_PAST_RECORD
;   - ROMEND: $07D7 -> $07BC 
; v0.16 (Sep 2026)
;   - Code golf pass: collapsed branch-to-return into RETC,EQ and removed
;     redundant whitespace skips.
;   - ROMEND: $07DA -> $07D7 
; v0.15 (Sep 2026)
;   - Added software paren-nesting tracker (PDEPTH) to eliminate hardware call
;     frame consumption in EA_PAREN, allowing deeper expression nesting.
;   - Repurposed dead TEMPRETH/TEMPRETL RAM cells for PDEPTH.
;   - ROMEND: $07C5 -> $07DA 
; v0.14 (Sep 2026)
;   - Optimized branch elimination (RETC,LT collapses) and converted keyword
;     dispatch (MD_SCAN/MD_HIT) to use direct register-indexed addressing on TOK_CHARS.
;   - ROMEND: $07C5 (1989 bytes).
; v0.13 (Sep 2026)
;   - Removed UPCASE case-folding entirely; forced strict uppercase syntax.
;   - ROMEND: $07F5 -> $07D7
; V0.12 (Sep 2026)
;   - Code golf pass. ROMEND: $7FB.
; v0.11 (Sep 2026)
;   - Fixed -32768 overflow bugs in MUL16 (MU_LP updated to BCFR,EQ) and
;     DIV16 (DV_LP updated to use COMA,R0 SC0).
;   - ROMEND: $0825 -> $082A
; v0.10 (Sep 2026)
;   - Fixed MUL16 loop counter decrement bug (replaced faulty BCFR,LT borrow test).
; v0.9 (Sep 2026) - PRINT_S16 replaced with a flat power-of-10 loop
;   - ROMEND $08C1 -> $0823 (2241 -> 2083 bytes)
; v0.8 (Sep 2026) - Size pass: CMP_TMP_PE extraction, PRT_BS removal
;   - ROMEND $090D -> $08C1
; v0.7 (Sep 2026) - PAREN-NEST-02: fix wrong-operator bug in EXPR
;   - ROMEND $08F3 -> $090D
; v0.6 (Sep 2026) - Delete GETCI_UC, TMP_TO_EXP16, RND_SHUFFLE/VRND_SHUFFLE/RNDSEED
;   - Inlined PUSH_RET and IP_TO_TMP    
; v0.5 (Aug 2026) - Flatten precedence
;   - Replaced PARSE_EXPR/EAM_ATOM/EAM_HI/EAM_LO_LOOP + SW- trampolining 
;     (PUSH_RET/PARSER_RET/SWRETURN dance) with flat EXPR/EXPR_ATOM/EXPR_LOOP: all 
;     operators (+-*/=<) are at one precedence, left to right: "1+2*3" = "(1+2)*3".
;   - ROM: ROMEND $0AE6 (2790) -> $093D (2365 bytes)
; v0.4 (Aug 2026) - Stage 4: narrow relops to = and <, plus a golf pass
;   - PARSE_RELOP: Now matches only '=' or '<' directly.
;   - ROM: ROMEND $0B00 (2816) -> $0AE6 (2790 bytes)
; v0.3 (Aug 2026) - Stage 3: single-char + letter statement dispatch
;   - Replaced KW_TAB's 2-3 char match (MATCH_KW, stride 5) with TOK_CHARS
;   - Deleted LET, REM, THEN
;   - ROM: ROMEND $0B17 (2839) -> $0B00 (2816 bytes)
; v0.2 (Aug 2026) - Stage 2: Implimented minimal line handling
;   - Append-only BASIC line handling - line number is accepted only if greater than
;     all  stored lines (append) or exactly equal to the current LAST line (in-place
;     replace, or delete on an empty body). Anything else is a syntax error.
;   - Removed STORE_LINE, DELETE_LINE, MEMCPY, and DEC_LNUM/DEC_GOTO.
;   - ROM: ROMEND $0BD9 (3033) -> $0B18 (2840 bytes)
; v0.1 (Aug 2026) - Initial port from uBASIC2650, inspired by pBASIC65c02.
;   - Cut statements and functions, cleaned showcase.
;   - Removed PRINT's CHR$(n)/TAB(n)/HEX$(n) 
;   - Added DO_WR: `WR expr` equivalent of `PRINT CHR$(n);`
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
ERR_DIV_ZERO    EQU '2'
; ERR_OOM         EQU '3'       ; Unused
ERR_VAR         EQU '4'
ERR_NEST        EQU '8'         ; Expression nesting too deep 

; Error check to make sure we dont crash due to HW Stack overflow 
PE_RAS_LIMIT    EQU 7

; PSW Defines
PSW_RS          EQU     $10
PSW_WC          EQU     $08             ; WC (With Carry) bit in PSL (bit 3)
PSW_FLAG        EQU     $40

;  CODE starts at Zero (No Pipbug)
        ORG 0

; =============================================================================
;  RESET / ENTRY + PAGE-ZERO VECTOR TABLE
;
; Page-zero subroutine vector table.
;
RESET:
        BCTR,UN MAIN            ; trampoline over vector table ($0000)
VINC_IP:
        DW INC_IP           ; 18 callers (16 ZBSR, 2 ZBRR)
VWSKIP:
        DW WSKIP            ; 3 callers (all ZBSR)
VINC_TMP:
        DW INC_TMP          ; 12 callers (all ZBSR)
VCOUT:
        DW COUT             ; 11 callers (8 ZBSR, 3 ZBRR)
VPARSE_EXPR:
        DW EXPR             ; 5 callers (all ZBSR)

VPRT_SPACE:
        DW PRT_SPACE        ; 3 callers (all ZBSR)
VCLR_EXP:
        DW CLR_EXP          ; 3 callers (all ZBSR)
VDO_ERROR:
        DW DO_ERROR         ; 2 callers (all ZBRR)
VJSYNERR:
        DW JSYNERR          ; 3 callers (all ZBRR)
VCLR_RUNFLG:
        DW CLR_RUNFLG       ; 2 callers (all ZBSR)
VEXP16_TO_LNUM:
        DW EXP16_TO_LNUM    ; 2 callers (all ZBSR)
VSET_TMP_PROG:
        DW SET_TMP_PROG     ; 4 callers (all ZBSR)
VCMP_TMP_PE:
        DW CMP_TMP_PE       ; 5 callers (all ZBSR)
VWSKIP_PEEK:
        DW WSKIP_PEEK       ; 4 callers (all ZBSR)
VNEG_EXP_BODY:
        DW NEG_EXP_BODY     ; 3 callers (2 ZBSR, 1 ZBRR)
VEXPR_LOOP:
        DW EXPR_LOOP        ; 3 callers (all ZBRR)
VPARSE_S16:
        DW PARSE_S16        ; 2 callers (all ZBSR)
VPRT_CRLF:
        DW PRT_CRLF         ; 2 callers (1 ZBSR, 1 ZBRR)
VPRINT_S16:
        DW PRINT_S16        ; 2 callers (all ZBSR)
VFIND_LINE:
        DW FIND_LINE        ; 2 callers (all ZBSR)
VTMP_TO_ET:
        DW TMP_TO_ET        ; 4 callers (3 ZBSR, 1 ZBRR)
VTMP_TO_SWSTK:
        DW TMP_TO_SWSTK     ; 2 callers (1 ZBSR, 1 ZBRR)
VINC_ET:
        DW INC_ET           ; 2 callers (1 ZBSR, 1 ZBRR)

; =============================================================================
; MAIN - Program init
; =============================================================================
MAIN:
       ; 10 bytes - Delete for ROM 
        LODI,R0 <SHOWCASE_END
        STRA,R0 PEH
        LODI,R0 >SHOWCASE_END
        STRA,R0 PEL

        PPSL $02                ; COM=1 (unsigned compare mode) for the entire

        ; clear Run flag - change to DO_NEW for ROM
        ZBSR *VCLR_RUNFLG             

        ; print sign-on banner
        LODI,R0 <BANNER
        STRA,R0 IPH
        LODI,R0 >BANNER
        STRA,R0 IPL
        BSTA,UN PRTSTR
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
        BSTA,UN GETLINE                   ; also leaves IPH:IPL = IBUF
        BSTA,UN TRY_STORE_LINE           ; CC=GT: line stored/deleted; CC=EQ: not a line
        BSTR,EQ STMT_EXEC               ; If CC=EQ (not a line), execute
        BCTR,UN REPL

; =============================================================================
;  DO_NEW -- Reset PE and IP
; Syntax: NEW
; In:  nothing
DO_NEW:
        ; PEH:PEL = PROG, via TMP (SET_TMP_PROG + TMP_TO_ET, both shared). IP is no longer
        ; set here: nothing reads it after NEW (GETLINE sets it; DR_LP re-derives it per line).
        ; TMP is clobbered - fine, DR_LP reloads it from SWSTK after every statement.
        ZBSR *VSET_TMP_PROG
        LODI,R0 PEH-IPH
        ZBSR *VTMP_TO_ET
        ; fall through
; =============================================================================
;  CLR_RUNFLG -- Clear RUNFLG (stop any run in progress)
; Also the direct handler for END (dispatched from the statement table,
; TOK_CHARS's "E" row) since v0.23 merged DO_END into this - END never
; needed anything beyond what this already did.
; In:  nothing
; Out: RUNFLG=0
; Clobbers: R0
CLR_RUNFLG:
        EORZ,R0
        STRA,R0 RUNFLG
        RETC,UN

; =============================================================================
;  DO_IF -- Conditional execution, no THEN keyword.
;  Nests for free: "IF a IF b stmt" - the true-path dispatch is a JUMP to
;  STMT_EXEC, so if "stmt" is itself another IF, it costs no extra depth.
; Syntax: IF expr stmt
; In:  IP -> first char after IF keyword
; Out: executes stmt if expr is nonzero; otherwise sequential return
; Clobbers: R0, R1, EXPH, EXPL (via EXPR, plus whatever the dispatched
;           statement clobbers on the true path)
DO_IF:
        ZBSR *VPARSE_EXPR                 ; [+1] condition -> EXPH:EXPL
        LODA,R0 EXPL
        IORA,R0 EXPH
        RETC,EQ                           ; both zero: false, return
        ; drop through
; =============================================================================
;  STMT_EXEC -- Decode and dispatch one BASIC statement from IP.
; In:  IPH:IPL -> first char of statement (after any leading whitespace)
; Out: control jumps to the matched DO_xxx handler, or falls into SE_NOTKW
; Clobbers: R0, R2, R1, GOTOH, GOTOL, IPH:IPL advanced past the keyword
;   on a match (unchanged on the bare-assignment path - see SE_NOTKW)
; RAS depth: 1 from REPL, 3 from DO_IF(THEN body).
STMT_EXEC:
        ZBSR *VWSKIP  
        STRZ,R2                          ; cache target char in R2 (1 byte,
        LODI,R1 1
        LODA,R0 *IPH,R1                   ; peek 2nd char
        SUBI,R0 A'A'                     ; Shift 'A' down to 0
        COMI,R0 A'Z'-A'A'                ; Compare against 25 (length of alphabet - 1)
        BCTR,GT SE_NOTKW                 ; Unsigned compare catches both < 'A' and > 'Z'
        LODI,R1 18                       ; scan from the first STATEMENT row (6 operator rows x 3 bytes;
                                         ; NB asm2650 silently mis-assembles "6*3" as 6, so a literal).
                                         ; Starting at row 0 let "-A" / "*A" match an operator row and be
                                         ; silently accepted as a no-op; now they fall to SE_NOTKW -> ?4.
                                         ; It also guarantees MD_HIT's first char is a letter.
MD_SCAN:
        LODA,R0 TOK_CHARS,R1              ; table char
        BCTR,EQ SE_NOTKW                  ; NUL row: no match -> bare assignment
        COMZ,R2                          ; 1 byte vs SUBA's 3 (r0:r2 -> CC);
        BCTR,EQ MD_HIT
        ADDI,R1 3                         ; next row (char + 2-byte handler)
        BCTR,UN MD_SCAN
MD_HIT:
        ; consume the whole matched keyword (pBASIC has no $ suffix
        ; keywords - CHR$/HEX$ are cut - so this is a plain A-Z scan,
        ; inlined from the old single-caller EATWORD using the same
        ; unsigned range-test idiom as the dispatch just above).
        ; The first char is a known letter (MD_SCAN starts past the operator
        ; rows), so step first, then test - one branch instead of two.
MDH_LP:
        ZBSR *VINC_IP
        LODA,R0 *IPH
        SUBI,R0 A'A'
        COMI,R0 A'Z'-A'A'
        BCFR,GT MDH_LP                   ; still A-Z: keep eating
; Jump into point from relop vectors
JMP_VEC:
        LODA,R0 TOK_CHARS,R1+              ; handler hi (pre-inc: char->hi)
        STRA,R0 GOTOH
        LODA,R0 TOK_CHARS,R1+              ; handler lo (pre-inc: hi->lo)
        STRA,R0 GOTOL
        BCTA,UN *GOTOH                    ; indirect jump

SE_NOTKW:
        ; Bare variable assignment ("X=expr" - either the 2nd-char peek
        ; above wasn't a letter, or the 1st char matched no statement).
        BSTR,UN PARSE_VAR_SAVE            ; validates A-Z, R2 = var byte offset, IP -> past it
        ZBSR *VWSKIP_PEEK
        COMI,R0 A'='
        BCFA,EQ JSYNERR
        ZBSR *VINC_IP
        ; drop through
; =============================================================================
;  DL_EX / DL_STORE -- Variable assignment. No LET keyword exists (see
;  STMT_EXEC) - reached only from SE_NOTKW's bare "V=expr" path and from
;  DO_ASK.
; In:  IP -> expression (DL_EX) or R2 = var byte offset, EXPH:EXPL = value (DL_STORE)
; Out: VARS[V] = EXPH:EXPL
; Clobbers: R0, R1, EXPH, EXPL, TMPH, TMPL (via PARSE_EXPR, DL_EX only)
DL_EX:
        ZBSR *VPARSE_EXPR                 ; [+1]
DL_STORE:
        LODA,R0 EXPH     ; R0 = High byte of expression
        STRA,R0 VARS,R2  ; Store directly to VARS array + offset (R2 is a valid index register)
        LODA,R0 EXPL     ; R0 = Low byte of expression
        STRA,R0 VARS+1,R2; Store directly to VARS array + offset + 1
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
        BSTR,UN GETLINE                   ; [+1] also leaves IPH:IPL = IBUF
        ZBSR *VPARSE_S16                ; [+1]
        BCTR,UN DL_STORE

; =============================================================================
;  DO_GOTO -- Computed GOTO
; Syntax: GOTO expr
; In:  IP -> first char after GOTO keyword
; Out: if running, SWSTK = found record pointer (DR_CD resumes from there
;      unconditionally - see DR_CD); if not running (typed at the prompt,
;      outside RUN), a safe no-op, same as before.
; Clobbers: R0, EXPH, EXPL, LNUMH, LNUML, TMPH, TMPL, SWSTK (only if running)
DO_GOTO:
        ZBSR *VPARSE_EXPR                 ; [+1] (EXPR_ATOM skips leading spaces itself)
        LODA,R0 RUNFLG
        RETC,EQ                           ; not running: safe no-op
        ZBSR *VEXP16_TO_LNUM              ; LNUMH:LNUML = EXPH:EXPL (target line)
        ZBSR *VFIND_LINE                  ; [+1] TMPH:TMPL = found record
        ZBRR *VTMP_TO_SWSTK              ; Tail call

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
; PARSE_VAR_SAVE -- skip whitespace, read var letter, range-check,
;                   save to R2, advance IP.
; Out: R2 = 2*(letter-'A') = byte offset into VARS; IP advanced past letter
; Error: tail-jumps to JERRVAR (no return)
; Clobbers: R0, R1, R2, SC0
PARSE_VAR_SAVE:
        ZBSR *VWSKIP_PEEK
        SUBI,R0 A'A'                     ; Shift 'A'-'Z' down to 0-25
        COMI,R0 A'Z'-A'A'                ; Compare against 25
        BCTA,GT JERRVAR                  ; Catches both < 'A' and > 'Z'
        ADDZ,R0                          ; R0 = index*2 (16-bit word stride)
        STRZ,R2                          ; R2 = byte offset of the variable in VARS, for DL_STORE
                                         ; (was: restore the ASCII letter here, recompute the offset there)
        ZBRR *VINC_IP                    ; tail call  

; =============================================================================
;  GETLINE -- Read a line from input into IBUF
; In:  nothing
; Out: IBUF = NUL-terminated input line.
;      IPH:IPL = IBUF on return (both callers need that; this used to be a
;      separate SET_IP_IBUF routine + vector slot + two call sites).
;      R1 is used as the index into IBUF while reading. R1=$FF means
;      empty (matches the SWBASE convention) since the 2650's ",R1+"
;      addressing mode pre-increments before the access.
; Clobbers: R0, R1
GETLINE:
        LODI,R0 <IBUF                    ; IP = IBUF.  IBUF is deliberately placed at $1010 (hi == lo)
        STRA,R0 IPH                      ; so ONE LODI serves both bytes.  If IBUF is ever moved,
        STRA,R0 IPL                      ; restore the >IBUF LODI for IPL.
        LODI,R1 $FF                      ; R1 = empty-buffer sentinel (pre-inc convention)
GL_LP:
        BSTR,UN CHIN                     ; [+1] blocking read
        COMI,R0 CR+1
        BCTR,LT GL_EOL                  ; catches CR and LF (LT CR+1)
        STRA,R0 IBUF,R1+                 ; R1++ (pre-inc); IBUF[R1]=char
        ZBSR *VCOUT  
        BCTR,UN GL_LP
GL_EOL:
        EORZ,R0
        STRA,R0 IBUF,R1+                 ; R1++ (pre-inc, one past last char); NUL-terminate
        ZBRR *VPRT_CRLF                ; tail call

; =============================================================================
; Character IO
; 110 Baud teletype from PIPBUG V1 as per Signetics M20 application note
; CHIN/COUT float to wherever the assembler places them - not pinned to
; PIPBUG-compatible addresses. Read their actual addresses from the .LST
; file and pass them to pipbug_wrap for testing,
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
;  WSKIP_PEEK -- Skip whitespace, then peek the current char at IP into R0
; Out: R0 = *IPH; CC set by that load (EQ if NUL)
; Clobbers: R0
WSKIP_PEEK:
        ZBSR *VWSKIP
        LODA,R0 *IPH
        RETC,UN

; =============================================================================
;  DO_PRINT / PRTSTR -- Print statement and NUL-terminated string helper
; Syntax: PRINT [item {; item}]
;   item = "string" | expr
; In:  IP -> first char after PRINT keyword
; Out: text written to COUT; IP advanced past statement
; Clobbers: R0, R1, EXPH, EXPL, TMPH, TMPL, NEGFLG, LNUMH, LNUML, SC0, SC1
DO_PRINT:
        BSTR,UN WSKIP_PEEK
        BCTR,EQ DP_SEP
DP_ITEM:
        BSTR,UN WSKIP_PEEK
        COMI,R0 DQ
        BCTR,EQ DP_STRING
        ; Expression
        ZBSR *VPARSE_EXPR  
        ZBSR *VPRINT_S16
        db $EC                            ; COMA,R0 -- consume next 2 bytes
DP_SCLS:
        ZBSR *VINC_IP  
DP_SEP:
        BSTR,UN WSKIP_PEEK
        COMI,R0 $3B                     ; semicolon
        BCFA,EQ PRT_CRLF          ; tail call: return from DO_PRINT
        ZBSR *VINC_IP  
        BSTR,UN WSKIP_PEEK
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
        BCTR,UN DP_STRING

; =============================================================================
;  CMP_TMP_PE -- Compare TMPH:TMPL against PEH:PEL (16-bit, byte-serial,
;  proper unsigned semantics via carry rather than SUBA's own CC, which is
;  only reliable over half the 0-255 byte range - see the TPSL $01 note
;  below). Factored out of six near-identical inlined copies (DR_LP,
;  TRY_STORE_LINE x2, FIND_LINE, FIND_INS, DO_LIST).
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
        BSTR,UN CMP_TMP_PE
        BCFA,LT CLR_RUNFLG                ; TMP >= PE (GT or EQ): program finished - clear RUNFLG

        ; save current line number for error reporting.  The 2-byte field is
        ; moved by a 2-pass loop: R1 auto-decrements 2->1->0 in the load, so
        ; the same index addresses TMP[n] and CURH+n (CURH/CURL are
        ; consecutive).  BRNR tests R1 without modifying it.  Leaves the two
        ; INC_TMPs adjacent.
        LODI,R1 2
DR_HDR:
        LODA,R0 *TMPH,R1-                 ; R1 2->1: TMP[1]; 1->0: TMP[0]
        STRA,R0 CURH,R1                   ; CURH+1 = CURL
        BRNR,R1 DR_HDR
        ; IP = TMP + 2 (body start) - execute straight out of PROG, no copy.
        ; Bodies are NUL-terminated in storage now (same convention IBUF
        ; already used), so STMT_EXEC/EXPR/etc's NUL-terminated-string
        ; assumptions just work without a manufactured terminator.
        ; IPH:IPL = TMPH:TMPL via the shared TMP_TO_ET copier (offset 0 =
        ; IPH itself) - TMP_TO_ET only reads TMP, so it's left untouched
        ; for ADV_PAST_RECORD below to re-walk from the record start.
        EORZ,R0                            ; offset 0 (IPH-IPH)
        ZBSR *VTMP_TO_ET
        ZBSR *VINC_IP
        ZBSR *VINC_IP
        ; advance TMP past this whole record (header+body+terminator) to
        ; find the next one; stash for SWSTK/DR_LP's next iteration (or a
        ; GOTO's redirection). Reuses the same shared scan FIND_INS/
        ; TSL_MATCH already use - one routine, one terminator.
        BSTA,UN ADV_PAST_RECORD
        ZBSR *VTMP_TO_SWSTK
        ; execute line
        BSTA,UN STMT_EXEC                ; [+1]
        ; resume from SWSTK unconditionally
        LODI,R1 2
DR_SWSTK:
        LODA,R0 SWSTK,R1-                 ; 
        STRA,R0 TMPH,R1                   ; 
        BRNR,R1 DR_SWSTK
        BCTR,UN DR_LP

; =============================================================================
;  TRY_STORE_LINE -- Store or delete a numbered line if IP starts with a digit
; In:  IPH:IPL -> input buffer
; Out: CC=GT if line stored/deleted; CC=EQ if not a numbered line
; Clobbers: R0, EXPH, EXPL, LNUMH, LNUML, TMPH, TMPL, CURH, CURL
TRY_STORE_LINE:
        LODA,R0 *IPH
        SUBI,R0 A'0'
        COMI,R0 9
        BCFR,GT TSL_NUM                  ; unsigned range test: 0-9 ("not greater" - the inverted sense
                                         ; needs no "not equal" condition, and no db $EC skip either)
TSL_NO:
        EORZ,R0                          ; CC=EQ: not a numbered line
        RETC,UN
TSL_NUM:
        ZBSR *VPARSE_S16                ; [+1]  (no WSKIP first: IP is already on the first digit)
        LODA,R0 EXPH
        BCTR,GT TSL_NZ
        LODA,R0 EXPL
        BCTR,EQ TSL_NO                   ; line number zero: not stored
TSL_NZ:
        ZBSR *VEXP16_TO_LNUM             ; LNUMH:LNUML = EXPH:EXPL (parsed line number)
                                          ; (space after the number is skipped by TSL_WRITE's WSKIP_PEEK)
        ZBSR *VFIND_LINE                ; [+1] TMP=matched record (CC=EQ), or
                                          ; FIND_INS's insertion point (CC=GT)
        BCTR,EQ TSL_MATCH                ; exact match exists somewhere in the store
        ; No exact match: TMP = first record with line > target, or PE if
        ; none. Legal only if TMP == PE (target exceeds every stored line -
        ; a plain append); otherwise some stored line already exceeds
        ; target with no exact match, which is out of order.
        ZBSR *VCMP_TMP_PE
        BCTR,EQ TSL_WRITE                ; TMP == PE exactly: legal append
        ZBRR *VJSYNERR
TSL_MATCH:
        ; Exact match at TMP. Legal only if it's the LAST stored line:
        ; save its start, advance a check past it, compare to PE.
        LODI,R0 CURH-IPH
        ZBSR *VTMP_TO_ET
        BSTA,UN ADV_PAST_RECORD
        ZBSR *VCMP_TMP_PE
        BCTR,EQ TSL_EXCISE
        ZBRR *VJSYNERR
TSL_EXCISE:
        LODI,R1 2                         ; REQUIRED, not redundant: R1 is
                                          ; whatever GETLINE left it as (the
                                          ; typed line's length) - only
                                          ; coincidentally 2 for a 2-char
                                          ; delete/replace. Verified: dropping
                                          ; this corrupts PE/TMP by 1 byte and
                                          ; silently loses a line for any
                                          ; other length. See v0.29 history.
TSL_LOOP:
        ; It's the last line: truncate the store back to where it started -
        ; nothing after it, so no shifting needed.
        LODA,R0 CURH,R1-                 ; R1 2->1: TMP[1]; 1->0: TMP[0]
        STRA,R0 PEH,R1                   ; CURH+1 = CURL
        STRA,R0 TMPH,R1                   ; CURH+1 = CURL
        BRNR,R1 TSL_LOOP 
TSL_WRITE:
        ZBSR *VWSKIP_PEEK
        BCTR,EQ TSL_DONE                  ; Arithmetic class) - EQ means NUL (empty
                                          ; body): delete-only (or no-op append).
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
                                          ; above that branched here - the stored
                                          ; record format now uses the same NUL
                                          ; terminator IBUF already used, so no
                                          ; terminator needs manufacturing at all
        ZBSR *VINC_TMP
;TMP_TO_PE:
        LODI,R0 PEH-IPH
        ZBRR *VTMP_TO_ET                ; Tail call - MUST be ZBRR: as ZBSR it returned and fell into
                                        ; FIND_LINE, whose CC=EQ made REPL also execute the stored line.
                                        ; Returns CC=GT (LODA,R0 TMPH, PE hi is $10+) = "line stored"


; =============================================================================
;  FIND_LINE -- Search for line LNUMH:LNUML in program store
; Out: TMPH:TMPL = record start if found; CC=EQ found, CC=GT not found.
; Clobbers: R0, TMPH, TMPL, EXPH, EXPL
FIND_LINE:
        BSTR,UN FIND_INS                 ; [+1]
        ; check if at end of program
        ZBSR *VCMP_TMP_PE
        BCFR,LT FL_RET_NF
FL_CHK:
        LODA,R0 *TMPH
        SUBA,R0 LNUMH
        BCTR,EQ FL_CHKLO
FL_RET_NF:
TSL_DONE:
        LODI,R0 1                        ; CC=GT: line stored/deleted
        RETC,UN

FL_CHKLO:
        LODI,R1 1                        ; read the record's lo byte at TMP+1 directly: indirect *indexed*
        LODA,R0 *TMPH,R1                 ; addressing, no need to compute TMP+1 into EXP first
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
; Clobbers: R0, TMPH, TMPL, EXPH, EXPL
FIND_INS:
        ZBSR *VSET_TMP_PROG
        db $EC                            ; COMA,R0 -- consume next 2 bytes
FI_ADV:
        BSTR,UN ADV_PAST_RECORD
        ZBSR *VCMP_TMP_PE
        RETC,GT
        RETC,EQ
        ;
        LODA,R0 LNUMH
        SUBA,R0 *TMPH                    ; LNUMH - stored.hi
        BCTR,GT FI_ADV
        RETC,LT                          ; was BCTR,LT FI_RET (branch-to-return; RETC has no false-polarity form, but BCTR's TRUE-polarity LT collapses directly)
        ; hi bytes equal: check lo
        LODI,R1 1                         ; stored lo byte is at TMP+1 (indirect indexed, as in FL_CHKLO)
        LODA,R0 LNUML
        COMA,R0 *TMPH,R1                  ; unsigned compare (COM=1 set in MAIN
        BCTR,GT FI_ADV
        RETC,UN

; =============================================================================
;  ADV_PAST_RECORD -- Advance TMPH:TMPL past the current stored line record
; Skips the 2-byte line-number header, scans forward until NUL (end of that
; record's text), then skips the NUL too - leaves TMPH:TMPL pointing at the
; start of the NEXT record (or PE, if this was the last one). Factored out
; of two identical inlined copies (TRY_STORE_LINE's TSL_MAS/TSL_MADONE,
; FIND_INS's FI_AS/FI_ADV/FI_DONE) found via the same duplicate-byte-
; sequence scan technique as CMP_TMP_PE.
; In:  TMPH:TMPL -> start of a stored record (its line-number hi byte)
; Out: TMPH:TMPL -> start of the next record
; Clobbers: R0
ADV_PAST_RECORD:
        ZBSR *VINC_TMP
        ZBSR *VINC_TMP
APR_LP:
        LODA,R0 *TMPH
        BCTR,EQ APR_DONE                  ; free zero-test: NUL ends the body
        ZBSR *VINC_TMP  
        BCTR,UN APR_LP
APR_DONE:
        ZBSR *VINC_TMP                    ; skip the NUL itself
        RETC,UN

; =============================================================================
;  EXPR -- Flat-precedence expression evaluator.  One left-to-right pass -
;  all six operators (+-*/=<) sit at the same precedence, matching
;  pBASIC65c02's EXPR/EXPR_LOOP exactly. "1+2*3" evaluates as "(1+2)*3" -
;  deliberate, not a bug.
; In:  IPH:IPL -> expression string
; Out: EXPH:EXPL = 16-bit result ($0000/$FFFF for relops - see DO_EQOP/
; Clobbers: R0, R1, R3, PDEPTH, BANG, SAVEH, SAVEL, NEGFLG, SC0, TMPH, TMPL
EXPR:
        LODI,R3 $FF                      ; SW operand-stack empty sentinel -
        EORZ,R0
        STRA,R0 PDEPTH                    ; SW nesting depth = 0 (genuine
 ;       STRA,R0 BANG                      ; relop-invert modifier off too -

EXPR_GUARDED:
        SPSU                             ; R0 = PSU; SP in bits 2:0
        ANDI,R0 $07
        COMI,R0 PE_RAS_LIMIT
        BCTR,LT EXPR_OK
        LODI,R0 ERR_NEST
        ZBRR *VDO_ERROR                 ; tail call and bail

EXPR_OK:
        BSTA,UN EXPR_ATOM
EXPR_LOOP:
        ZBSR *VWSKIP_PEEK              ; R0 has character
        STRA,R0 SC0                      ; SC0 = char to match against operators

        LODI,R1 15                       ; last row's char offset (6 rows x
                                          ; 3 bytes, walking down to row 0)
OPS_LP:
        LODA,R0 TOK_CHARS,R1              ; table char - direct indexed, no
        SUBA,R0 SC0
        BCTR,EQ OPS_HIT
        SUBI,R1 3                         ; 3-byte stride
        BCFR,LT OPS_LP                    ; loop while R1 still >=0 as a
        
        ; '!' relop-invert modifier: not one of the 6 known operators, but
;        LODA,R0 SC0
;        COMI,R0 A'!'
;        BCFR,EQ EL_NOTBANG
;        LODI,R0 $FF
;        STRA,R0 BANG                      ; armed - consumed once by whatever
;        ZBSR *VINC_IP                     ; consume '!'
;        BCTR,UN EXPR_LOOP                 ; loop - look for the real relop
        ; No operator matched: this level's flat chain is done. If we're
EL_NOTBANG:
        LODA,R0 PDEPTH
        RETC,EQ
        SUBI,R0 1
        STRA,R0 PDEPTH                    ; EXPR_LOOP's own top-of-loop
        ZBRR *VINC_IP                     ; consume ')' tail call

OPS_HIT:
        ; PAREN-NEST-01: Push left operand to the LIFO stack
        LODA,R0 EXPL
        STRA,R0 SWBASE,R3+
        LODA,R0 EXPH
        STRA,R0 SWBASE,R3+
        
        ; PAREN-NEST-02: Push the R1 table offset to the stack to survive recursion
        LODZ,R1                  ; R0 = R1 (Destination is ALWAYS R0)
        STRA,R0 SWBASE,R3+       ; Push offset to top of stack

        ZBSR *VINC_IP            ; consume the operator char
        BSTR,UN EXPR_ATOM        ; parse the right operand

        ; Pop the R1 table offset back off the stack
        LODA,R0 SWBASE,R3        ; R0 = top of stack (our saved R1 offset)
        STRZ,R1                  ; R1 = R0 (Source is ALWAYS R0)

;  POP_TMP -- pop a 2-byte value pushed on SWBASE into TMPH:TMPL and jump to it
        LODA,R0 SWBASE,R3-      ; predecrement
        STRA,R0 TMPH
        LODA,R0 SWBASE,R3-
        STRA,R0 TMPL
        SUBI,R3 1
        ; jump to vector
        BCTA,UN JMP_VEC

; =============================================================================
;  EXPR_ATOM -- parse one atom: unary +/-, parens, or a literal/variable.
; In:  IPH:IPL -> atom
; Out: EXPH:EXPL = value
; Clobbers: R0; PDEPTH too, via EA_PAREN (see there) -- net zero on return,
;   since the matching close-paren decrements it back before this level's
;   RETC fires (OPS_LP exit).
EA_POS:
        ZBSR *VINC_IP  
EXPR_ATOM:
        ZBSR *VWSKIP    ; returns with R0 = char
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
        ZBRR *VNEG_EXP_BODY              ; tail call: negate, return
EA_PAREN:
        ZBSR *VINC_IP                     ; consume '('
        LODA,R0 PDEPTH
        ADDI,R0 1
        STRA,R0 PDEPTH                    ; PDEPTH++ (SW-tracked, not hw RAS)
        BCTA,UN EXPR_GUARDED              ; TAIL JUMP, not call -- costs 0 hw

; =============================================================================
;  DO_ADD / DO_SUB / DO_MUL / DO_DIV / DO_EQOP / DO_LTOP -- flat operator
;  handlers. Each pops the left operand pushed by OPS_HIT, combines with
;  EXPH:EXPL (the just-parsed right operand), and jumps back to
;  EXPR_LOOP to look for more operators at the same precedence.
; In:  EXPH:EXPL = right operand; SWBASE top = pushed left operand (lo,hi)
; Out: EXPH:EXPL = combined result; control resumes at EXPR_LOOP
; Clobbers: R0, R3 (popped by 2), plus per-operator (see each)
DO_SUB:
        ZBSR *VNEG_EXP_BODY              ; EXP = -EXP
        ; drop through
DO_ADD:
        LODA,R0 TMPL
        ADDA,R0 EXPL
        STRA,R0 EXPL
        PPSL $08                          ; WC=1: the high byte takes the carry
        LODA,R0 TMPH
        ADDA,R0 EXPH
        STRA,R0 EXPH
        CPSL $08
        ZBRR *VEXPR_LOOP

; =============================================================================
;  DO_EQOP / DO_LTOP -- relop handlers, folded into the flat operator
;  table. Pop left, compare against right (EXPH:EXPL), leave $0000 (false)
;  or $FFFF (true) in EXPH:EXPL - not "0/1" as an earlier comment here (and
;  in EXPR's own header) used to claim; DO_IF's own IORA-then-RETC,EQ test
;  never cared (any nonzero value reads as true), so the mismatch was
;  silent until corrected.
DO_EQOP:
        LODA,R0 TMPH
        EORA,R0 EXPH                      ; 0 iff the hi bytes are equal (EOR, not SUB: same test, and the
        STRZ,R1                           ; partial result now parks in R1 - free here - not via SC0)
        LODA,R0 TMPL
        EORA,R0 EXPL
        IORZ R1                           ; 0 iff both bytes equal
        BCTR,EQ DOP_TRUE
        BCTR,UN DOP_FALSE
DO_LTOP:
        LODA,R0 TMPH
        EORI,R0 $80
        STRZ,R1                           ; biased(left.hi) parked in R1 (free here), not SC0
        LODA,R0 EXPH
        EORI,R0 $80
        COMZ,R1                           ; biased(right.hi) : biased(left.hi)
        BCTR,GT DOP_TRUE                  ; right.hi > left.hi -> left<right
        BCTR,LT DOP_FALSE
        LODA,R0 EXPL
        COMA,R0 TMPL                      ; right.lo : left.lo (hi bytes equal)
        BCTR,GT DOP_TRUE
DOP_FALSE:
        EORZ,R0
        db $EC                            ; COMA,R0 -- consume next 2 bytes
DOP_TRUE:
        LODI,R0 $FF
        ; both paths converge here (see EORZ,R0/db $EC above): R0 = $00
        ; (false) or $FF (true). Apply the '!' modifier, if OPS_LP armed
        ; it, then disarm it - consumed once per relop, regardless of
;        EORA,R0 BANG                      ; R0 ^= BANG ($00 no-op / $FF flips)
        STRA,R0 EXPH
        STRA,R0 EXPL
;        EORZ,R0
;        STRA,R0 BANG                      ; BANG = 0 again
        ZBRR *VEXPR_LOOP

; =============================================================================
;  PARSE_FACTOR -- Parse a single value (variable or literal)
; In:  IPH:IPL -> first char of factor
; Out: EXPH:EXPL = value
; Clobbers: R0, R1
; No longer upcases - see KNOWN LIMITATIONS (uppercase-only entry).
PARSE_FACTOR:
        LODA,R0 *IPH
        SUBI,R0 A'A'                      ; shift 'A' down to 0 - also doubles
                                          ; as PF_LOADVAR's index below, so a
                                          ; letter never needs re-subtracting
        COMI,R0 A'Z'-A'A'                 ; unsigned range test (STMT_EXEC idiom)
        BCTR,GT PF_NUM                    ; not A-Z -> literal/number path
        ; fall through: A-Z letter, R0 = index (0..25) already computed
; =============================================================================
;  PF_LOADVAR -- Load variable value from VARS
; In:  R0 = variable index (0..25, i.e. char-'A'); IP -> that letter
; Out: EXPH:EXPL = variable value
; Clobbers: R0, R1
PF_LOADVAR:
        ADDZ,R0                          ; R0 = index*2
        STRZ,R1                          ; R1 = index*2
        ZBSR *VINC_IP                     ; advance IP past the letter - clobbers
                                          ; R0, but R1 (index*2) survives INC_IP's
                                          ; bank switch, so no SC0 round-trip needed
        LODA,R0 VARS,R1                  ; hi byte
        STRA,R0 EXPH
        LODA,R0 VARS+1,R1               ; lo byte
        STRA,R0 EXPL
        RETC,UN

PRO_NONE:
        ZBRR *VJSYNERR 

PF_NUM:
;       drop through
; =============================================================================
;  PARSE_S16 -- Parse signed decimal integer
; In:  IPH:IPL -> first char (optional '-' then digits)
; Out: EXPH:EXPL = signed 16-bit value
; Clobbers: R0, NEGFLG, EXPH, EXPL
PARSE_S16:
        LODA,R0 *IPH
        EORI,R0 A'-'                     ; R0 = 0 iff a minus sign
        STRA,R0 NEGFLG                   ; NEGFLG polarity: 0 = negate the result (see NEG_EXP), any
                                         ; other value = leave it. Written unconditionally, so no
                                         ; separate clear is needed.
        BCFR,EQ PS16_UN                  ; not '-' (EORI's CC survives the STRA)
PS16_NEG:
        ZBSR *VINC_IP  
PS16_UN:
;       drop through
; =============================================================================
;  PARSE_U16 -- Parse unsigned decimal digits -> EXPH:EXPL
; Jumps to JSYNERR if no digits found.
; In:  IPH:IPL -> first digit char
; Out: EXPH:EXPL = value
; Clobbers: R0, R3, SC0, EXPH, EXPL, TMPH, TMPL (RXSAVE used to preserve R3)
;PARSE_U16:
        ZBSR *VCLR_EXP
        LODA,R0 *IPH
        SUBI,R0 A'0'
        COMI,R0 9
        BCTR,GT PRO_NONE; surrogate for JSYNERR
PU16_LP:
        LODA,R0 *IPH
        SUBI,R0 A'0'
        COMI,R0 9
        BCTR,GT PU16_RET                  ; unsigned range test: not 0-9 -> done
PU16_DIG:
        STRZ,R1                          ; digit -> R1 (INC_IP, EXP16_TO_ET, CLR_EXP and MULT_LOOP leave R1 alone)
        ZBSR *VINC_IP
        LODI,R0 SC0-IPH
        BSTA,UN EXP16_TO_ET              ; SC0:SC1 = EXP (the value so far)
        ZBSR *VCLR_EXP                   ; EXP = 0 ...
        LODZ,R1
        STRA,R0 EXPL                     ; ... then = the digit: MULT_LOOP *accumulates* into EXP, so
                                         ; preloading the digit gives digit + 10*value in one go (no
                                         ; separate final add or carry step)
        LODI,R0 10
        STRA,R0 TMPL                     ; TMP = 10: x10 via the shared MULT_LOOP
        EORZ,R0
        STRA,R0 TMPH
        BSTA,UN MULT_LOOP                ; [+1] EXP = digit + value*10
        BCTR,UN PU16_LP

PU16_RET:
        ; drop through

; =============================================================================
;  NEG_EXP -- Negate EXPH:EXPL if NEGFLG == 0 (0 = "negate", see PARSE_S16/ABS_TMP)
;  NEG_EXP_BODY -- Unconditional negate EXPH:EXPL
; In:  EXPH:EXPL = value; NEGFLG = flag
; Out: EXPH:EXPL negated (two's complement) if NEGFLG==0
; Clobbers: R0, R1
NEG_EXP:
        LODA,R0 NEGFLG
        RETC,GT                          ; nonzero, positive  } leave it
        RETC,LT                          ; nonzero, negative  }
NEG_EXP_BODY:
        LODI,R1 EXPH-IPH                 ; EXPH offset from IPH (= 4); R1 variant for NEG_SHARED
        BCTR,UN NEG_SHARED

; =============================================================================
;  ABS_TMP -- Absolute value of TMPH:TMPL; (re)initialises NEGFLG from its sign
; In:  TMPH:TMPL = signed value
; Out: TMPH:TMPL = |value|; NEGFLG = 0 if it was negative (result to be negated),
;      $80 if not
; Clobbers: R0, R1
ABS_TMP:
        LODA,R0 TMPH
        ANDI,R0 $80                      ; $80 if negative, else 0
        EORI,R0 $80                      ; flip: negative -> 0 (= negate), positive -> $80 (= leave)
        STRA,R0 NEGFLG                   ; unconditional write - no prior clear needed
        RETC,LT                          ; positive ($80 -> LT): nothing to negate
NEG_TMP:        
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
        LODZ,R1
        ZBRR *VINC_ET                   ; tail call: adds 1 (INC_ET uses alt bank R1)

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
        EORI,R0 $80                      ; toggle 0 <-> $80
        STRA,R0 NEGFLG
        BCTR,UN NEG_EXP_BODY

; =============================================================================
;  MUL16 -- Signed 16-bit multiply: TMPH:TMPL * EXPH:EXPL -> EXPH:EXPL
;  (Setup is DO_MUL's, shared with DIV16 below; the multiply itself is the
;  leaf MULT_LOOP after DV_LP, also used by PU16 for its x10.)
; In:  TMPH:TMPL = left operand; EXPH:EXPL = right operand
; Out: EXPH:EXPL = product (16-bit two's complement wrap)
; Clobbers: R0, R1, NEGFLG, SC0, SC1, TMPH, TMPL, EXPH, EXPL
; RAS: called at depth 6 (see EXPR_GUARDED); own peak (inlined setup's
;   ABS_TMP/ABS_EXP/EXP16_TO_ET/CLR_EXP sub-calls) is depth 7, not 8.

; =============================================================================
;  DIV16 -- Signed 16-bit divide: TMPH:TMPL / EXPH:EXPL -> EXPH:EXPL
; The remainder is NOT kept: the in-place subtract destroys TMPH:TMPL.
; In:  TMPH:TMPL = dividend; EXPH:EXPL = divisor
; Out: EXPH:EXPL = quotient
; Clobbers: R0, R1, NEGFLG, SC0, SC1, TMPH, TMPL
; Error: divisor=0 -> ERR_DIV_ZERO
DO_DIV:
        LODA,R0 EXPL    ; check for zero
        IORA,R0 EXPH        
        BCFR,EQ DO_MUL
        LODI,R0 ERR_DIV_ZERO
        ZBRR *VDO_ERROR 

        ; not zero
DO_MUL:
        ; R1 holds the offset of the matched table row ('*' or '/') from
        ; EXPR_LOOP's own dispatch - save it in RXSAVE since R1 itself
        ; gets clobbered below (ABS_TMP/ABS_EXP/EXP16_TO_ET all use it).
        STRA,R1 RXSAVE
        ; Setup
        BSTA,UN ABS_TMP                  ; [+1] NEGFLG = 0 if TMP was negative else $80 (writes it fresh)
        BSTR,UN ABS_EXP                  ; [+1] toggles NEGFLG if EXP was negative
        LODI,R0 SC0-IPH                 ; offset to SCO and 1, SC1 = |EXP| lo
        BSTA,UN EXP16_TO_ET             ; SC0 = |EXP| hi
        ZBSR *VCLR_EXP                  ; clear EXP (accumulator starts at 0)

        ; Which path: recover the saved table offset and check if it was '*'
        LODA,R0 RXSAVE
        COMI,R0 8                       ; 8 = '*' row's offset
        BCFR,EQ DV_LP                   ; not '*': divide
        BSTR,UN MULT_LOOP               ; [+1] EXP = TMP * SC0:SC1
MU_DONE:
        BSTA,UN NEG_EXP
        ZBRR *VEXPR_LOOP

        ; Div loop: quotient = number of times SC0:SC1 (|divisor|) can be subtracted from TMP (|dividend|).
        ; Subtract IN PLACE with a WC-chained 16-bit subtract and test the final carry: a borrow means
        ; TMP < divisor, i.e. done - and TMP (the remainder) is dead by then (nothing reads it), so it
        ; needs no compare pass, no tentative scratch and no restore.
DV_LP:
        LODA,R0 TMPL
        SUBA,R0 SC1                     ; WC=0: no borrow-in
        STRA,R0 TMPL
        PPSL $08                        ; WC=1: the high byte takes the borrow
        LODA,R0 TMPH
        SUBA,R0 SC0
        CPSL $08
        TPSL $01                        ; C=1 no borrow (EQ) / C=0 borrow (LT)
        BCTR,LT MU_DONE                 ; TMP < divisor: done
        STRA,R0 TMPH                    ; (R0 = new TMPH; TPSL/CPSL leave it alone)
        LODI,R0 EXPH-IPH                ; EXP offset from IPH (= 4); assembly-time expression
        ZBSR *VINC_ET                   ; quotient++
        BCTR,UN DV_LP

MULT_LOOP:
        ; Leaf (calls nothing): EXPH:EXPL += SC0:SC1, TMPH:TMPL times (TMP is consumed).  Used by DO_MUL
        ; and by PU16's x10.  Decrement-first with a borrow test, using WC (carry-in) for the high byte:
        ;   TMP-1 borrows out only when TMP was 0 - i.e. after exactly TMP additions - and that ends it.
        LODA,R0 TMPL
        SUBI,R0 1                        ; WC=0 here: C=0 iff TMPL was 0 (borrow)
        STRA,R0 TMPL
        PPSL $08                         ; WC=1: the high byte takes the borrow
        LODA,R0 TMPH
        SUBI,R0 0
        STRA,R0 TMPH
        CPSL $08
        TPSL $01                         ; C=1 no borrow (EQ) / C=0 borrow (LT)
        RETC,LT                          ; TMP was 0: all additions done
        LODA,R0 EXPL
        ADDA,R0 SC1
        STRA,R0 EXPL
        PPSL $08                         ; WC=1: the high byte takes the carry
        LODA,R0 EXPH
        ADDA,R0 SC0
        STRA,R0 EXPH
        CPSL $08
        BCTR,UN MULT_LOOP

; =============================================================================
;  DO_LIST -- Print stored BASIC lines (v0.1b: whole program only -
; Syntax: LIST
; In:  PROG=program base, PEH:PEL=program end
; Out: whole program printed
; Clobbers: R0, R1, TMPH, TMPL, EXPH, EXPL, SC0, SC1 (IP untouched)
DO_LIST:
        ZBSR *VSET_TMP_PROG
        db $EC                  ; COMA,R0: skip next 2 bytes 
DLS_NL:
        ZBSR *VINC_TMP                    ; skip over NUL char
        ZBSR *VPRT_CRLF
DLS_LP:
        ; Check TMP against program end
        ZBSR *VCMP_TMP_PE
        RETC,GT
        RETC,EQ

        ; Read+print line number, then rest of line verbatim.  Walks TMP
        ; directly - PRINT_S16 no longer uses TMPL as scratch, so the old
        ; TMP->IP copy here and IP->TMP copy-back at DLS_NL are both gone.
        LODI,R1 2                         ; read the 2-byte line-number header
DLS_HDR:
        LODA,R0 *TMPH,R1-                 ; R1 2->1: TMP[1]; 1->0: TMP[0]
        STRA,R0 EXPH,R1                   ; EXPH+1 = EXPL
        BRNR,R1 DLS_HDR
        ZBSR *VINC_TMP
        ZBSR *VINC_TMP
        BSTR,UN PRINT_S16
        ZBSR *VPRT_SPACE
DLS_BLPX:
        LODA,R0 *TMPH
        BCTR,EQ DLS_NL                     ; free zero-test: NUL ends the body
        ZBSR *VCOUT
        ZBSR *VINC_TMP
        BCTR,UN DLS_BLPX

; =============================================================================
;  PRINT_S16 -- Print signed 16-bit value EXPH:EXPL as decimal
; In:  EXPH:EXPL = signed value
; Out: decimal digits written to COUT
; Clobbers: R0, R1, R2, R3, SC0 (TMP is NOT clobbered - DO_LIST relies on that
;   to walk TMP directly). R2 is no longer saved/restored: no caller needs it
;   after a print (R2 only carries a variable offset from PARSE_VAR_SAVE to
;   DL_STORE, and nothing prints in between), and the digit path always
;   clobbered it anyway - only the zero path used to restore it.
PRINT_S16:
        LODA,R0 EXPH             ; get high byte & establish CC
        BCTR,LT IS_NEG           ; branch if negative (bit 7 set)

        IORA,R0 EXPL             ; Check for ZERO
        BCFR,EQ PS_DIGITS       ; >0, flow into subtract printer

        LODI,R0 A'0'             ; Handle Zero
        ZBRR *VCOUT              ; Print '0' and tail call return
IS_NEG:
        LODI,R0 A'-'
        ZBSR *VCOUT
        ZBSR *VNEG_EXP_BODY     ; Negate, making EXPH:EXPL positive
PS_DIGITS:
        EORZ,R0                 
        STRZ,R2                 ; R2 = P10 table index (0 to 4)
        STRZ,R3                 ; R3 = leading zero flag (0 = leading, >0 = printing)
DIGIT_LOOP:
        LODI,R1 A'0'-1           ; R1 = ASCII digit character
SUB_LOOP:
        CPSL $08                 ; Clear WC bit for standard 8-bit math
        ADDI,R1 1                ; Increment digit
        LODA,R0 EXPL
        SUBA,R0 P10_LO,R2        ; Subtract low byte
        STRA,R0 SC0              ; Save tentatively (was TMPL - PRINT_S16
                                 ; must not clobber TMP, see DO_LIST)
        PPSL $08                 ; Set WC bit (enables Carry-In/Borrow)
        LODA,R0 EXPH
        SUBA,R0 P10_HI,R2        ; Subtract high byte
        BCTR,LT BORROW           ; If borrow occurred (C=0), result is LT. Stop subtracting.

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
        IORZ,R3                  ; Test R3 flag
        BCFR,EQ PRINT_IT         ; Flag set (>0), print the zero
        COMI,R2 4                ; Is it the final column (1s)?
        BCFR,EQ NEXT_DIG
PRINT_IT:
        LODI,R3 1                ; Set leading zero flag
        LODZ,R1                  ; R0 = R1 (destination is always R0)
        ZBSR *VCOUT              ; Print the character
NEXT_DIG:
        ADDI,R2 1                ; Advance to next power of 10
        COMI,R2 5                ; Have we processed all 5 powers?
        BCTR,LT DIGIT_LOOP
        CPSL $08                  ; Clear WC bit
WSKIPRET:
        RETC,UN                  ; Return to caller

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
;  EXP16_TO_ET family -- copy EXPH:EXPL to any RAM register pair.
;  TMP_TO_ET family -- copy TMPH:TMPL to any RAM register pair.
;
;  Each entry loads its offset (XYZH-IPH) into R0 (always bank-0, unaffected
;  by PSW_RS), falls through to body.  STRZ R1 copies R0 into alt-R1 for
;  indexed addressing.  Primary R1/R2/R3 fully preserved via CPSL PSW_RS.
;  Clobbers R0 only.  NO BSTA inside body.
;  Direct BSTA,UN (no ZP slot): CUR_TO_EXP16 (1 site).
;  Note: the reverse ([offset]->EXP / ET_TO_EXP16) direction this comment
;  used to also describe has since been removed - CUR_TO_EXP16 is gone;
;  nothing currently needs an "[offset]->EXP" or "[offset]->TMP" primitive
;  enough times to be worth re-adding it.
EXP16_TO_LNUM:
        LODI,R0 LNUMH-IPH       ; LNUMH offset from IPH (= 12)
EXP16_TO_ET:
        PPSL PSW_RS             ; switch to alternate register bank
        STRZ R1                 ; alt-R1 = R0 = destination offset
        LODA,R0 EXPL
        STRA,R0 IPL,R1          ; store lo byte to dest+1
        LODA,R0 EXPH
        BCTR,UN ET_STORE        ; store hi byte, restore bank, return

; Now for TMP
TMP_TO_SWSTK:
        LODI,R0 SWSTK-IPH
TMP_TO_ET:
        PPSL PSW_RS             ; switch to alternate register bank
        STRZ R1                 ; alt-R1 = R0 = destination offset
        LODA,R0 TMPL
        STRA,R0 IPL,R1          ; store lo byte to dest+1
        LODA,R0 TMPH
        BCTR,UN ET_STORE        ; store hi byte, restore bank, return

; =============================================================================
;  JERRVAR -- Error with variable
;  JSYNERR -- Syntax error jump
; In:  nothing (R0 irrelevant)
; Out: jumps to DO_ERROR
; Clobbers: R0
JERRVAR:
        LODI,R0 ERR_VAR
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to BCTA
JSYNERR:
        LODI,R0 ERR_SYN
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
        LODI,R1 2
DE_LOOP:
        LODA,R0 CURH,R1-
        STRA,R0 EXPH,R1
        BRNR,R1 DE_LOOP
        ZBSR *VPRINT_S16                ; [+1]
DE_NL:
        BSTR,UN PRT_CRLF
        ZBSR *VCLR_RUNFLG                ; Not running
        BCTA,UN REPL                     ; REPL resets RAS (PSU SP bits) on entry

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
; CLR_EXP -- Helper Zeroes EXP
; Clobbers R0
CLR_EXP:
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 EXPL
        RETC,UN

; =============================================================================
;  DO_WR -- Write raw character (v0.1c: replaces CHR$, which is cut. This is
;  a statement, not a PRINT item - consecutive WRs need no PRINT/semicolon
;  wrapper the way "PRINT CHR$(a);CHR$(b);" did.)
; Syntax: WR expr
; In:  IP -> first char after WR keyword
; Out: low byte of expr's value written to COUT (no newline)
; Clobbers: R0, EXPH, EXPL, TMPH, TMPL, NEGFLG, LNUMH, LNUML, SC0, SC1
DO_WR:
        ZBSR *VPARSE_EXPR                 ; (EXPR_ATOM skips leading spaces itself)
        LODA,R0 EXPL
        ZBRR *VCOUT              ; tail call

; =============================================================================
;  TABLES 
BANNER:
        DB CR, LF, "pBASIC 0.30", CR, LF, NUL

; -- Combined operator + statement dispatch table
; Format: [char][hi][lo], stride 3, NUL-terminated.
; Statement scan (STMT_EXEC/MD_SCAN) safely runs the WHOLE table because
; its 2nd-char letter-gate guarantees the char being matched is A-Z,
; which none of the 6 operator chars are - no bounding needed there.
; Operator scan (EXPR_LOOP/OP_SCAN) is explicitly bounded to the first 6
; entries
TOK_CHARS:
        DB "+", <DO_ADD,    >DO_ADD       ; +
        DB "-", <DO_SUB,    >DO_SUB       ; -
        DB "*", <DO_MUL,    >DO_MUL       ; *
        DB "/", <DO_DIV,    >DO_DIV       ; /
        DB "=", <DO_EQOP,   >DO_EQOP      ; = (relop, folded into flat table)
        DB "<", <DO_LTOP,   >DO_LTOP      ; < (relop, folded into flat table)
        DB "A", <DO_ASK,    >DO_ASK       ; ASK
        DB "E", <CLR_RUNFLG,>CLR_RUNFLG   ; END
        DB "G", <DO_GOTO,   >DO_GOTO      ; GOTO
        DB "I", <DO_IF,     >DO_IF        ; IF
        DB "L", <DO_LIST,   >DO_LIST      ; LIST
        DB "N", <DO_NEW,    >DO_NEW       ; NEW
        DB "P", <DO_PRINT,  >DO_PRINT     ; PRINT
        DB "R", <DO_RUN,    >DO_RUN       ; RUN
        DB "W", <DO_WR,     >DO_WR        ; WR
        DB NUL

; Powers of 10 Tables (10000, 1000, 100, 10, 1)
P10_HI:
        db $27, $03, $00, $00, $00
P10_LO:
        db $10, $E8, $64, $0A, $01

ROMEND: 

;  RAM variables -- sequential RES block 
 
        ORG     4096    ; half a 2650 8kbyte page

; --- Ordered group: offsets from IPH used by INC_ET, EXP16_TO_ET's family,
;     TMP_TO_ET's family, and NEG_SHARED (DEC_ET, which used to share this
;     group too, was removed - see VERSION HISTORY) ---
IPH     RES 1       ; interpreter pointer hi       (INC_ET offset 0)
IPL     RES 1       ; interpreter pointer lo
TMPH    RES 1       ; temp 16-bit hi               (INC_ET offset 2 = TMPH-IPH)
TMPL    RES 1       ; temp 16-bit lo
GOTOH   RES 1       ; pending target hi (handler address for the matched
                    ; operator/keyword - not currently read via an offset)
GOTOL   RES 1       ; pending target lo
CURH    RES 1       ; current line hi  (error reporting; TMP_TO_ET offset
                    ; CURH-IPH copies TMP here from TSL_MATCH)
CURL    RES 1       ; current line lo

LNUMH   RES 1       ; scratch line number hi (EXP16_TO_LNUM offset 12 =
                    ; LNUMH-IPH copies EXP here)
LNUML   RES 1       ; scratch line number lo
EXPH    RES 1       ; expression result hi         (INC_ET offset 4 = EXPH-IPH)
EXPL    RES 1       ; expression result lo
SWSTK   RES 2       ; next-line pointer cache [NLP_H][NLP_L] written by DR_EXEC

; --- Remaining ---
SC0     RES 1       ; Scratch byte 0
SC1     RES 1       ; Scratch byte 1
IBUF    RES 64      ; Input buffer 64 bytes - MUST sit at $1010 (hi == lo, see GETLINE)
PEH     RES 1       ; Program end pointer hi
PEL     RES 1       ; Program end pointer lo
;PEH     DB <SHOWCASE_END       ; Program end pointer hi
;PEL     DB >SHOWCASE_END       ; Program end pointer lo
PDEPTH  RES 1       ; SW-tracked paren nesting depth 

;  --- Flags & Stuff --- 
RUNFLG  RES 1       ; $01=running $00=immediate
RXSAVE  RES 1       ; Save/restore R3 in PARSE_U16 and R1 in DO_MUL 
NEGFLG  RES 1       ; Sign flag: 0 = negate the result, nonzero = leave (see NEG_EXP)
;BANG    RES 1       ; '!' relop-invert modifier: $00 clear, $FF armed -

;  SW call stack -- used by PARSE_EXPR / PRINT_S16 only
; R3 = index ($FF=empty, grows up). Each frame = [lo][hi].
; Push: STRA,R0 *SWBASE,R3+ (lo first), STRA,R0 *SWBASE,R3+ (hi).
; Pop:  LODA,R0 *SWBASE,R3- (hi first), LODA,R0 *SWBASE,R3- (lo).
SWBASE  RES 16      ; SW stack base 

VARS    RES 52      ; A-Z variables 2 bytes each

; =============================================================================
;  Pre-loaded SHOWCASE program
;
;  Line format: <lineno_hi> <lineno_lo> <body_ASCII> <NUL>
;  Lines  10-190: feature demos (PRINT, WR, arithmetic, comparisons, GOTO loop)
;  Lines 300-510: Mandelbrot set renderer )
;
;  Format: DB hi,lo,"text",$00  -- hi-then-lo matches DR_EXEC record format.
;  $22=DQ $3B=semicolon  in-string chars that need escaping.
; =============================================================================
PROG:
        DB 0,20,"PRINT ",$22,"-- pBASIC2650 Showcase --",$22,$00
        DB 0,30,"PRINT ",$22,"--- PRINT / WR ---",$22,$00                      ; 30  PRINT "--- PRINT / WR ---"
        DB 0,40,"WR 65",$00                                                    ; 40  WR 65
        DB 0,41,"WR 66",$00                                                    ; 41  WR 66
        DB 0,42,"WR 67",$00                                                    ; 42  WR 67
        DB 0,43,"PRINT",$00                                                    ; 43  PRINT
        DB 0,50,"PRINT ",$22,"--- ARITHMETIC ---",$22,$00                      ; 50  PRINT "--- ARITHMETIC ---"
        DB 0,60,"PRINT ",$22,"3+4=",$22,$3B,"3+4",$3B,$22,"  10-3=",$22,$3B,"10-3",$3B,$22,"  6*7=",$22,$3B,"6*7",$00  ; 60  PRINT "3+4=";3+4;"  10-3=";10-3;"  6*7=";6*7
        DB 0,70,"PRINT ",$22,"20/4=",$22,$3B,"20/4",$00                        ; 70  PRINT "20/4=";20/4
        DB 0,80,"PRINT ",$22,"--- COMPARISONS ---",$22,$00                     ; 80  PRINT "--- COMPARISONS ---"
        DB 0,90,"IF 3<9 PRINT ",$22,"3<9 ok",$22,$00                      ; 90  IF 3<9 PRINT "3<9 ok"
        DB 0,100,"IF 7=7 PRINT ",$22,"7=7 ok",$22,$00                     ; 100 IF 7=7 PRINT "7=7 ok"
;        DB 0,110,"IF 9!=2 PRINT ",$22,"9!=2 ok",$22,$00                   ; 110 IF 9!=2 PRINT "9!=2 ok"
;        DB 0,120,"IF 9!<4 PRINT ",$22,"9!<4 ok",$22,$00                   ; 120 IF 9!<4 PRINT "9!<4 ok" (9>=4)
        DB 0,130,"IF 4<9 PRINT ",$22,"9>4 ok",$22,$00                     ; 130 IF 4<9 PRINT "9>4 ok" (reversed operands for >)
;        DB 0,135,"IF 9!<6 PRINT ",$22,"6<=9 ok",$22,$00                   ; 135 IF 9!<6 PRINT "6<=9 ok" (reversed operands for <=)
        DB 0,140,"PRINT ",$22,"--- LOOP via GOTO ---",$22,$00                  ; 140 PRINT "--- LOOP via GOTO ---"
        DB 0,150,"I=1",$00                                                      ; 150 I=1
        DB 0,160,"IF 5<I GOTO 190",$00                                    ; 160 IF 5<I GOTO 190
        DB 0,170,"PRINT I",$3B,$00                                              ; 170 PRINT I;
        DB 0,180,"I=I+1",$00                                                    ; 180 I=I+1
        DB 0,185,"GOTO 160",$00                                                 ; 185 GOTO 160
        DB 0,190,"PRINT ",$22,"",$22,$00                                        ; 190 PRINT ""
;        DB 0,236,"PRINT ",$22,"--- LIST ---",$22,$00                            ; 236 PRINT "--- LIST ---"
;        DB 0,238,"LIST",$00                                                     ; 238 LIST
        DB 0,240,"GOTO 300",$00                                                 ; 240 GOTO 300
        DB 1,44,"PRINT ",$22,"--- MANDELBROT ---",$22,$00                      ; 300 PRINT "--- MANDELBROT ---"
        DB 1,54,"I=-64",$00                                                     ; 310 I=-64
        DB 1,64,"IF 56<I GOTO 510",$00                                    ; 320 IF 56<I GOTO 510
        DB 1,74,"D=I",$00                                                       ; 330 D=I
        DB 1,84,"C=-144",$00                                                    ; 340 C=-144 (widened from -120)
        DB 1,94,"IF 28<C GOTO 480",$00                                    ; 350 IF 28<C GOTO 480 (widened from 4)
        DB 1,104,"A=C",$00                                                      ; 360 A=C
        DB 1,105,"B=D",$00                                                      ; 361 B=D
        DB 1,106,"E=0",$00                                                      ; 362 E=0
        DB 1,107,"N=1",$00                                                      ; 363 N=1
        DB 1,114,"IF 16<N GOTO 420",$00                                   ; 370 IF 16<N GOTO 420
        DB 1,124,"IF 0<E GOTO 410",$00                                    ; 380 IF 0<E GOTO 410
;        DB 1,134,"T=A*A/64-B*B/64+C",$00                                       ; 390 T=A*A/64-B*B/64+C
        DB 1,134,"T=(A*A/64)-(B*B/64)+C",NUL             
        DB 1,144,"B=2*A*B/64+D",$00                                             ; 400 B=2*A*B/64+D
        DB 1,145,"A=T",$00                                                      ; 401 A=T
        DB 1,154,"IF 256<((A*A/64)+(B*B/64)) IF E=0 E=N",NUL                     ; 410 IF 256<((A*A/64)+(B*B/64)) IF E=0 E=N
        DB 1,164,"N=N+1",$00                                                    ; 420 N=N+1
        DB 1,165,"IF 16<N GOTO 430",$00                                   ; 421 IF 16<N GOTO 430
        DB 1,166,"GOTO 370",$00                                                 ; 422 GOTO 370
        DB 1,174,"IF 0<E WR E+32",$00                                     ; 430 IF 0<E WR E+32
        DB 1,184,"IF E=0 WR 32",$00                                       ; 440 IF E=0 WR 32
        DB 1,194,"C=C+4",$00                                                    ; 450 C=C+4
        DB 1,204,"GOTO 350",$00                                                 ; 460 GOTO 350
        DB 1,224,"PRINT",$00                                                    ; 480 PRINT
        DB 1,234,"I=I+6",$00                                                    ; 490 I=I+6
        DB 1,244,"GOTO 320",$00                                                 ; 500 GOTO 320
        DB 1,254,"END",$00                                                      ; 510 END
SHOWCASE_END:

        END
