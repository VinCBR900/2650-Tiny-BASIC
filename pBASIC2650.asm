; pBASIC2650.asm - PoC Minimal Tiny BASIC for Signetics 2650
; v0.35 - Oct 2026
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
;   ./asm2650 --warn=ALL pBASIC2650.asm pBASIC2650.hex
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
;   Statements: ASK V   END   GOTO expr   IF expr stmt   LIST   NEW
;               PRINT [item {;item}]   RUN   WR expr   V=expr
;     (no LET or THEN keyword exists - assignment and IF-bodies are bare)
;   PRINT items: "literal", or an expression; separate with ';'.
;   Arithmetic: + - * /  (unary -).
;   Relops: `=`  `<` only - see DELIBERATE LIMITS below for precedence.
;   Numbers : signed 16-bit  (-32768 .. 32767)
;   Variables: A-Z (26), 16-bit signed; no arrays or string variables
;   Print   : "literals", expr, `;` separator - no string vars
;
;   NOT IMPLEMENTED (smallest-useful-set is deliberate, not an oversight):
;     FOR/NEXT, GOSUB/RETURN, RND, arrays, string variables,
;     CHR$()/TAB()/HEX$(), LET, THEN, DELETE of an arbitrary line
;     (program storage is append-only; only the last line can be
;     replaced or deleted - see STORAGE / INPUT below).
;
;        KNOWN LIMITATIONS
;
; UPPERCASE only apart from PRINT "String literals"
;
; STATEMENT DISPATCH matches ONLY the first character of a line against
;   a 9-entry table (A/E/G/I/L/N/P/R/W for ASK/END/GOTO/IF/LIST/NEW/
;   PRINT/RUN/WR. It first checks the 2nd is a LETTER otherwise VAR.
;
; PROGRAM COUNTER: the same pointer (PCH:PCL) drives RUN, GOTO, LIST and line storage,
;   so LIST or NEW executed from inside a running program ends the run.
;
; STORAGE / INPUT
;   Program lines are append-only.
;   A new line number must be greater than the current highest,
;   or equal to it to replace/delete the last line.
;   Input is not bounds-checked; overlong lines will corrupt memory.
;   Line 0 is an ordinary line: "0 PRINT 1" is stored/listed/RUN able.
;
; DELIBERATE LIMITS
;   No BODMAS/PEMDAS precedence - all operators are equal and evaluate
;   left to right, so "1+2*3" evaluates as "(1+2)*3" = 9, not 7. Use
;   explicit parentheses for grouping.
;   Nesting is limited by the 8-deep hardware RAS: every parenthesis and every
;   unary minus costs one frame, and each atom is checked at EXPR_ATOM entry
;   (PE_RAS_LIMIT). 3 levels, e.g. "(((expr)))", at the prompt and in RUN
;   (RUN resets the RAS so both start at the same depth); "(((-5)))" and
;   "----5" are one level too deep. Excess depth generates ERR_NEST.
;
;   Minimal syntax validation; malformed constructs may produce the
;   normal syntax/runtime error rather than a specialised diagnostic.
;
; =============================================================================
; VERSION HISTORY (pBASIC2650)
; =============================================================================
;
; v0.35 (Oct 2026) - Code Golf, ROMEND: $04D6 (1238 bytes)   
;   - EXPR: RAS guard is TPSU + BCTA,EQ DO_ERROR; COMA,R0 TOK_CHARS,R1 (no SC0 store); OPS_HIT push loop.
;   - FIND_INS returns a 3-way CC (EQ end / LT later line / GT exact): FIND_LINE deleted, DO_GOTO tail-calls it.
;   - TRY_STORE_LINE: one-loop body copy (STRA leaves CC), ADV_PC_PAST_REC finds the new PE.
;   - SWSTK is now PCH:PCL, the program counter for RUN/GOTO/LIST/store (was TMP): no per-line copies.
;     FIND_INS compares EXP directly, LNUM deleted. LIST/NEW inside a program now end the run.
;   - PRINT_S16: no zero case, OR-accumulated leading-zero flag, P10 tables ascending, BIRR digit count.
;   - MUL/DIV: ABS_X + MD_SETUP replace ABS_TMP/ABS_EXP/RXSAVE (RAS peak unchanged).
;   - STMT_EXEC letter tests are ANDI,R0 $40. Unary plus (EA_POS) removed. Banner updated.
;   - FIX: a stored line leaving PEL=$00 returned CC=EQ and REPL executed it.
;   - FIX: PRINT_S16 called COUT with WC=1 (COUT's RRR would rotate through carry).
;   - FIX: Moved RAS guard to EXPR_ATOM entry, so every paren and unary minus is checked. 
;     DO_RUN resets RAS and exits via REPL, RUN has prompt's depth: 3 levels in both, ERR_NEST beyond.
;
; v0.34 (Sep 2026) - Code golf: REG16_TO_REG16 generic 16-bit copy.
;   - Replaced EXP16_TO_ET/TMP_TO_ET and DE_LOOP, DR_SWSTK copy loops with
;     REG16_TO_REG16 using R0 = packed (SRC_IDX<<4)|DST_IDX nibble pair. 
;   - RAM reorg: PEH/PEL and SC0/SC1 moved contiguous for use with REG16.
;   - ROMEND $056E (1390) -> $0562 (1378 bytes).
;
; v0.33 (Sep 2026) - ROMEND: $056F (1391 bytes)   
;   - Factored out ADD_CORE utilized by DO_ADD and MULT_LOOP
;   - Code golf.
;
; v0.32 (Sep 2026) - ROMEND: $058e (1422 bytes)
;   - Ported DO_EQOP/DO_LTOP share CMP_OPS compare, from uBASICwip.
;   - Factor out DIGIT_CHECK digit-range test. 
;   - File header rewritten for consistancy with other variants.
;
; v0.31 (Sep 2026) - ROMEND: $059E (1438 bytes)   [v0.30: $05B2 / 1458]
;   - PEH:PEL now preset at Assembly time for showcase demo - delete for ROM. 
;   - Line number 0 is no longer special-cased in TRY_STORE_LINE (-10 bytes)
;
; v0.30 (Sep 2026) - ROMEND: $05B2 (1458 bytes)   
;   - TSL_CPYDONE's "tail call" into TMP_TO_ET used a ZBSR not ZBRR - corrected.
;   - RUN left RUNFLG=1 after a normal program end. DR_LP now clears on GT or EQ
;   - Operator-first lines ("-A", "*A", "/A", "=A", "<A") were silently accepted
;   - GETLINE sets IPH:IPL itself: SET_IP_IBUF deleted.
;   - MD_HIT: step-then-test keyword eater, one branch instead of two.
;   - R2 now holds the variable's VARS byte offset (2*index): PARSE_VAR_SAVE
;   - TRY_STORE_LINE digit test: SUBI/COMI + BCFR,GT TSL_NUM.
;   - Dead WSKIP calls removed: DO_GOTO/DO_WR/TSL_NUM (IP already on the digit),
;     TSL_NZ (TSL_WRITE's WSKIP does it).
;   - NEGFLG polarity is now 0 = negate: PARSE_S16 sets it with one EORI/STRA
;   - PRINT_S16: R2 save/restore removed 
;   - DO_NEW: SET_PC_PROG + TMP_TO_ET (its IP write had no reader).
;   - Arithmetic core: Added MULT_LOOP decrement-with-borrow test + inline WC add)
;     shared by DO_MUL and PU16, CARRY_INTO_EXPH & EXP16_TO_TMP deleted.
;   - Relops park their partial result in R1 (EORA/STRZ,R1/COMZ,R1), not SC0.
;   - FIND_LINE/FIND_INS uses indirect indexed, INC16_TMP_TO_EXP is gone.
;   - Vector-table caller counts regenerated (several were stale).
;
; v0.29 (Sep 2026) - ROMEND: $0645 (1605 bytes)
;   - Dropped the TSL_ERR indirection 
;   - DO_RUN: IPH:IPL = TMPH:TMPL now via the shared TMP_TO_ET copier
;   - DR_LP's loop-back: BCTA,UN DR_LP -> BCTR,UN DR_LP (in range here too).
;   - EATWORD inlined at MD_HIT and remoeved $ check
;   - PARSE_FACTOR/PF_LOADVAR: PARSE_FACTOR's 2-comparison A-Z test
;     replaced with the SUBI/COMI range test
;   - PU16's two digit-range tests SUBI/COMI idiom
;   - MU_DONE's post-negate CLR_NEGFLAG confirmed dead and rmeoved.
;
; v0.28 (Sep 2026) - Stored-program record terminator: CR -> NUL, ported from
;   uBASIC2650wip.asm's same conversion.
;   - Bodies are now NUL-terminated in storage, matching what STMT_EXEC/EXPR/
;     etc already assume for IBUF - DO_RUN executes straight out of PROG
;     (no more per-line copy into IBUF, no manufactured terminator).
;   - ADV_PC_PAST_REC (shared by FIND_INS/TSL_MATCH), DLS_BLPX and
;     TSL_CPYDONE updated to match; each drops its CR compare entirely in
;     favour of the free CC=EQ a LODA already gives on a zero byte.
;   - Showcase's terminators changed $0D/CR -> $00/NUL to match.
;   - ROMEND: $0681 -> $0670 (1665 -> 1648 bytes)
;
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
;   - Factored out WSKIP subroutine and ADV_PC_PAST_REC
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
; v0.8 (Sep 2026) - Size pass: CMP_PC_PE extraction, PRT_BS removal
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
PE_RAS_LIMIT    EQU 7                       ; TPSU mask at EXPR_ATOM entry: ERR_NEST when PSU.SP has all these
                                            ; bits set (7 = SP==7). The deepest leaf under an atom call (PU16 ->
                                            ; MULT_LOOP -> ADD_CORE) adds 2 more frames: SP 6 + 2 = 8 fills the
                                            ; 8-slot RAS exactly, SP 7 would wrap it.

; PSW Defines
PSW_RS          EQU     $10
PSW_WC          EQU     $08             ; WC (With Carry) bit in PSL (bit 3)
PSW_FLAG        EQU     $40

; REG16_TO_REG16 register indices (idx*2 = byte offset from IPH; see the
; "Ordered group" RAM comment). v0.34.
IDX_IP    EQU 0
IDX_TMP   EQU 1
IDX_GOTO  EQU 2
IDX_CUR   EQU 3
IDX_PC    EQU 4
IDX_EXP   EQU 5
IDX_PE    EQU 6
IDX_SC0   EQU 7

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
VINC_PC:
        DW INC_PC          ; 12 callers (all ZBSR)
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
VREG16_TO_REG16:
        DW REG16_TO_REG16   ; v0.34: generic nibble-packed 16-bit copy
VJSYNERR:
        DW JSYNERR          ; 3 callers (all ZBRR)
VSET_PC_PROG:
        DW SET_PC_PROG     ; 4 callers (all ZBSR)
VCMP_PC_PE:
        DW CMP_PC_PE       ; 5 callers (all ZBSR)
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
VFIND_INS:
        DW FIND_INS         ; 2 callers (all ZBSR)
VINC_ET:
        DW INC_ET           ; 2 callers (1 ZBSR, 1 ZBRR)
VDIGIT_CHECK:
        DW DIGIT_CHECK      ; 3 callers (all ZBSR) - ported from uBASIC
VADV_PC_PAST_REC:
        DW ADV_PC_PAST_REC

; =============================================================================
; MAIN - Program init
; =============================================================================
MAIN:
        PPSL $02                ; COM=1 (unsigned compare mode) for the entire

        ; RUNFLG is cleared by REPL. For a ROM build add "BSTA,UN DO_NEW" here (see PEH)

        ; print sign-on banner - delete saves 13 bytes plus ASCIIz
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
        CPSL PSW_RS + 5                 ; primary reg bank; clear C/OVF/RS -
        CPSU $07                        ; clear PSU SP field (bits 2:0 = HW RAS depth)
        BSTA,UN CLR_RUNFLG              ; not running (END, error and end of program all land here)
        LODI,R0 '>'                     ; print prompt only used here
        ZBSR *VCOUT  
        ZBSR *VPRT_SPACE  
        BSTA,UN GETLINE                 ; also leaves IPH:IPL = IBUF
        BSTA,UN TRY_STORE_LINE          ; CC=GT: line stored/deleted; CC=EQ: not a line
        BSTR,EQ STMT_EXEC               ; If CC=EQ (not a line), execute
        BCTR,UN REPL

; =============================================================================
;  DO_RUN -- Execute stored program
; Syntax: RUN
; In:  PROG=program base, PEH:PEL=program end
; Out: runs until END, error, or exhausted; returns to REPL
; Clobbers: all
; =============================================================================
DO_RUN:
        CPSU $07                        ; RAS: drop REPL's frame, RUN starts at the prompt's depth
        LODI,R0 1
        STRA,R0 RUNFLG
        ZBSR *VSET_PC_PROG              ; PC = first record
DR_LP:
        LODA,R0 RUNFLG
        BCTR,EQ REPL             ; END: back to the prompt (the RAS was reset, so no RETC)
        ; end of program? PCH:PCL >= PEH:PEL
        ZBSR *VCMP_PC_PE
        BCFR,LT REPL             ; PC >= PE: program finished

        ; save current line number for error reporting.  The 2-byte field is
        ; moved by a 2-pass loop: R1 auto-decrements 2->1->0 in the load, so
        ; the same index addresses PC[n] and CURH+n (CURH/CURL are
        ; consecutive).  BRNR tests R1 without modifying it.
        LODI,R1 2
DR_HDR:
        LODA,R0 *PCH,R1-                  ; R1 2->1: PC[1]; 1->0: PC[0]
        STRA,R0 CURH,R1                   ; CURH+1 = CURL
        BRNR,R1 DR_HDR
        ; IP = PC + 2 (body start) - execute straight out of PROG
        LODI,R0 (IDX_PC*16)+IDX_IP
        ZBSR *VREG16_TO_REG16
        ZBSR *VINC_IP           ; skip over line number
        ZBSR *VINC_IP
        ; advance PC past this whole line/record: PC is the run's program counter, and
        ; (unlike TMP) is never touched by expression evaluation, so no save/restore.
        ZBSR *VADV_PC_PAST_REC
        BSTR,UN STMT_EXEC                ; [+1]  (GOTO simply overwrites PC)
        BCTR,UN DR_LP

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
        ANDI,R0 $40                      ; letters have bit 6 set; NUL, space, digits, '=' do not
        BCTR,EQ SE_NOTKW                 ; not a letter: bare assignment
        LODI,R1 18                       ; scan from the first STATEMENT row (6 operator rows x 3 bytes;
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
MDH_LP:
        ZBSR *VINC_IP
        LODA,R0 *IPH
        ANDI,R0 $40                      ; bit 6 set = letter
        BCFR,EQ MDH_LP                   ; still a letter: keep eating

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
        ZBSR *VWSKIP
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
; PARSE_VAR_SAVE -- skip whitespace, read var letter, range-check,
;                   save to R2, advance IP.
; Out: R2 = 2*(letter-'A') = byte offset into VARS; IP advanced past letter
; Error: tail-jumps to JERRVAR (no return)
; Clobbers: R0, R1, R2, SC0
PARSE_VAR_SAVE:
        ZBSR *VWSKIP
        SUBI,R0 A'A'                     ; Shift 'A'-'Z' down to 0-25
        COMI,R0 A'Z'-A'A'                ; Compare against 25
        BCTA,GT JERRVAR                  ; Catches both < 'A' and > 'Z'
        ADDZ,R0                          ; R0 = index*2 (16-bit word stride)
        STRZ,R2                          ; R2 = byte offset of the variable in VARS, for DL_STORE
        ZBRR *VINC_IP                    ; tail call  

; =============================================================================
;  DO_NEW -- Reset PE and IP
; Syntax: NEW
; In:  nothing
DO_NEW:
        ; PEH:PEL = PROG, via TMP (SET_PC_PROG + TMP_TO_ET, both shared).
        ZBSR *VSET_PC_PROG
        LODI,R0 (IDX_PC*16)+IDX_PE
        ZBSR *VREG16_TO_REG16
        ; drop through
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
;  DO_ASK -- Read signed integer from user into variable 
;  AKA INPUT - I is taken by IF under 1-char statement dispatch
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
; Out: PCH:PCL = found record pointer (DR_LP resumes from there); typed at the
;      prompt it is a harmless no-op (PC is re-initialised by RUN/LIST/store).
; Clobbers: R0, R1, EXPH, EXPL, PCH, PCL
DO_GOTO:
        ZBSR *VPARSE_EXPR                 ; [+1] target line -> EXPH:EXPL (EXPR_ATOM skips leading spaces itself)
        ZBRR *VFIND_INS                   ; tail call: PCH:PCL = target record (or next higher line / PE).

; =============================================================================
;  SET_PC_PROG -- Set TMPH:TMPL = PROG base address
; Clobbers: R0
SET_PC_PROG:
        LODI,R0 <PROG
        STRA,R0 PCH
        LODI,R0 >PROG
        STRA,R0 PCL
        RETC,UN

; =============================================================================
;  GETLINE -- Read a line from input into IBUF
; In:  nothing
; Out: IBUF = NUL-terminated input line.
;      IPH:IPL = IBUF on return
;      R1 is used as the index into IBUF while reading. R1=$FF means
;      empty (matches the SWBASE convention) 
; Clobbers: R0, R1
GETLINE:
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF                    ; IBUF moved (v0.34 nibble-index RAM
        STRA,R0 IPL                      ; reorg) - no longer hi==lo, +2B here (accepted)
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
;        WRTC,R0        ; paper tape
        LODI,R1 0
        LODI,R2 8
;ACHI:   
        SPSU
        BCTR,LT CHIN
        EORZ,R0
;        WRTC,R0        ; paper tape
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
;  CMP_PC_PE -- Compare TMPH:TMPL against PEH:PEL (16-bit, byte-serial,
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
CMP_PC_PE:
        LODA,R0 PCH
        SUBA,R0 PEH
        RETC,GT                                   ; (GT/LT) is already the answer
        RETC,LT
        ; low byte
        LODA,R0 PCL
        SUBA,R0 PEL
        TPSL $01                          ; carry-based unsigned compare:
        RETC,UN                           ; C=1 (no borrow) -> EQ, C=0 -> LT

; =============================================================================
;  TRY_STORE_LINE -- Store or delete a numbered line if IP starts with a digit
; In:  IPH:IPL -> input buffer
; Out: CC=GT if line stored/deleted; CC=EQ if not a numbered line
; Clobbers: R0, R1, R2, EXPH, EXPL, TMPH, TMPL, CURH, CURL, PEH, PEL, SC0, SC1
TRY_STORE_LINE:
        ZBSR *VDIGIT_CHECK                ; R0 = char-'0'
        BCFR,GT TSL_NUM                  ; unsigned range test: 0-9
TSL_NO:
        EORZ,R0                          ; CC=EQ: not a numbered line
        RETC,UN
TSL_NUM:
        ZBSR *VPARSE_S16                ; IP is already on the first digit
        ZBSR *VFIND_INS                  ; TMP = first record >= target, or PE
        BCTR,EQ TSL_WRITE                ; EQ: TMP == PE, plain append
        BCFR,GT TSL_ERR                  ; LT: a later line exists with no exact match: out of order
        ; GT: exact match at TMP. Legal only if it is the LAST stored line:
        ; save its start, advance past it, compare to PE.
        LODI,R0 (IDX_PC*16)+IDX_CUR
        ZBSR *VREG16_TO_REG16
        ZBSR *VADV_PC_PAST_REC
        ZBSR *VCMP_PC_PE
        BCFR,EQ TSL_ERR                  ; not the last line
        LODI,R0 (IDX_CUR*16)+IDX_PC     ; TMP = start of the last record (it is overwritten or dropped)
        ZBSR *VREG16_TO_REG16
TSL_WRITE:
        ZBSR *VWSKIP
        BCTR,EQ TSL_FIN                  ; EQ = NUL: empty body, delete-only (or no-op append)
        LODI,R1 2                        ; write the 2-byte line-number header
TSL_HDR:
        LODA,R0 EXPH,R1-                ; R1 2->1: EXPL; 1->0: EXPH
        STRA,R0 *PCH,R1                 ; -> PC[1] then PC[0]
        BRNR,R1 TSL_HDR
        LODI,R1 $FF                      ; src index (pre-inc: 0 on first byte)
        LODI,R2 1                        ; dst index (pre-inc: 2 on first byte = past the header)
TSL_CPY:
        LODA,R0 *IPH,R1+                 ; sets CC; STRA leaves it alone, so the NUL is copied then ends the loop
        STRA,R0 *PCH,R2+
        BCFR,EQ TSL_CPY
        ZBSR *VADV_PC_PAST_REC          ; TMP = end of the new record
TSL_FIN:
        LODI,R0 (IDX_PC*16)+IDX_PE      ; PE = TMP
        ZBSR *VREG16_TO_REG16
        LODI,R0 1                        ; CC=GT: line stored/deleted (REG16's own CC depends on PEL: not reliable)
        RETC,UN
TSL_ERR:
        ZBRR *VJSYNERR

; =============================================================================
;  FIND_INS -- Find sorted insertion point for LNUMH:LNUML
; In:  LNUMH:LNUML = target line number
; Out: TMPH:TMPL = address of the first record with line >= target, or PEH:PEL
;      if all lines are smaller.
;      CC=EQ: reached PE (no such record); CC=LT: record found with line > target;
;      CC=GT: record found with line == target (exact match).
; Clobbers: R0, R1, TMPH, TMPL
FIND_INS:
        ZBSR *VSET_PC_PROG
        db $EC                            ; COMA,R0 -- consume next 2 bytes
FI_ADV:
        ZBSR *VADV_PC_PAST_REC
        ZBSR *VCMP_PC_PE
        RETC,EQ                           ; TMP == PE (hi bytes can't differ: GT never happens)
        LODI,R1 0               ; Start index at 0 (High byte)
CHK_LP:
        LODA,R0 EXPH,R1        ; Load target line number byte (R1=0: High, R1=1: Low)
        COMA,R0 *PCH,R1        ; Unsigned compare with TMP line number byte
        BCTR,GT FI_ADV          ; If target > current, current line is smaller, keep searching
        RETC,LT                 ; If target < current, found the insertion point (no exact match)
        EORI,R1 1               ; XOR R1 with 1 (Toggles 0 -> 1 -> 0) & updates CC flags
        BCFR,EQ CHK_LP          ; If result is not 0 (we just did High byte), loop for Low byte
        LODI,R0 1               ; both bytes matched: CC=GT = exact match
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
        ZBSR *VNEG_EXP_BODY              ; EXP = -EXP
        ; drop through
DO_ADD:
        LODI,R0 TMPH-IPH                 ; TMPH offset from IPH (= 2)
        BSTR,UN ADD_CORE                 ; [+1] EXP += TMP
        ZBRR *VEXPR_LOOP

; =============================================================================
;  ADD_CORE -- shared 16-bit indexed add: EXPH:EXPL += value at IPH+off.
; In:  R0 = 16-bit field offset from IPH (e.g. TMPH-IPH, SC0-IPH)
; Out: EXPH:EXPL += *(IPH+off):*(IPL+off)
; Clobbers: R0, R1
ADD_CORE:
        STRZ,R1
        LODA,R0 EXPL
        ADDA,R0 IPL,R1
        STRA,R0 EXPL
        PPSL $08                          ; WC=1: the high byte takes the carry
        LODA,R0 EXPH
        ADDA,R0 IPH,R1
        STRA,R0 EXPH
        CPSL $08
        RETC,UN

; =============================================================================
;  CMP_OPS -- shared signed 16-bit 3-way compare. 
; In:  TMPH:TMPL = left operand, EXPH:EXPL = right operand
; Out: CC = LT/EQ/GT as left is <, =, > right
; Clobbers: R0, R1
CMP_OPS:
        LODA,R0 EXPH
        EORI,R0 $80                       ; bias hi bytes: signed order becomes
        STRZ,R1                           ; unsigned order (COM=1 compares)
        LODA,R0 TMPH
        EORI,R0 $80
        COMZ,R1                           ; biased left.hi : right.hi
        RETC,GT
        RETC,LT
        LODA,R0 TMPL                      ; hi bytes equal: unsigned lo compare
        COMA,R0 EXPL
        RETC,UN

; =============================================================================
;  DO_EQOP / DO_LTOP -- relop handlers, folded into the flat operator
;  table. Pop left, compare against right via CMP_OPS, leave $0000 (false)
;  or $FFFF (true) in EXPH:EXPL ; DO_IF's own IORA-then-RETC,EQ test
;  never cared (any nonzero value reads as true). DO_GTOP deliberately 
;  not implemented - pBASIC's are = and < only (
DO_EQOP:
        BSTR,UN CMP_OPS
        BCTR,EQ DOP_TRUE
        BCTR,UN DOP_FALSE
DO_LTOP:
        BSTR,UN CMP_OPS
        BCTR,LT DOP_TRUE
;        BCTR,UN DOP_FALSE
;DO_GTOP:
;        BSTR,UN CMP_OPS
;        BCTR,GT DOP_TRUE
        ; drop through: false
DOP_FALSE:
        EORZ,R0
        db $EC                           ; COMA,R0 -- consume next 2 bytes
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

; --- OPS_HIT: operator found. (Placed ahead of EXPR so OPS_LP's BCTR reaches it backwards.)
OPS_HIT:
        ; Frame, bottom->top: [R1 table offset][EXPL][EXPH]   (R3 -> EXPH)
        LODZ,R1
        STRA,R0 SWBASE,R3+               ; push operator row offset (survives recursion)
        LODI,R1 2
OH_PUSH:
        LODA,R0 EXPH,R1-                 ; R1 2->1: EXPL; 1->0: EXPH
        STRA,R0 SWBASE,R3+
        BRNR,R1 OH_PUSH
        ZBSR *VINC_IP                    ; consume the operator char
        BSTA,UN EXPR_ATOM                ; parse the right operand
        LODA,R0 SWBASE,R3                ; pop EXPH -> TMPH
        STRA,R0 TMPH
        LODA,R0 SWBASE,R3-               ; pre-dec: EXPL -> TMPL
        STRA,R0 TMPL
        LODA,R0 SWBASE,R3-               ; pre-dec: R1 = operator row offset
        STRZ,R1
        SUBI,R3 1                        ; drop the offset slot
        BCTA,UN JMP_VEC                  ; dispatch handler

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
        BSTR,UN EXPR_ATOM
EXPR_LOOP:
        ZBSR *VWSKIP                     ; R0 = operator char
        LODI,R1 15                       ; last row's char offset (6 rows x
                                         ; 3 bytes, walking down to row 0)
OPS_LP:
        COMA,R0 TOK_CHARS,R1             ; indexed compare, R0 preserved (no scratch cell)
        BCTR,EQ OPS_HIT
        SUBI,R1 3                        ; 3-byte stride
        BCFR,LT OPS_LP                   ; loop while R1 still >=0
        ; No operator matched: this level's flat chain is done.
EL_NOTBANG:
        LODA,R0 PDEPTH
        RETC,EQ
        SUBI,R0 1
        STRA,R0 PDEPTH
        ZBRR *VINC_IP                    ; consume ')' tail call

; =============================================================================
;  EXPR_ATOM -- parse one atom: unary -, parens, or a literal/variable.
; In:  IPH:IPL -> atom
; Out: EXPH:EXPL = value
; Clobbers: R0; PDEPTH too, via EA_PAREN (see there) -- net zero on return,
;   since the matching close-paren decrements it back before this level's
;   RETC fires (OPS_LP exit).
EA_NEG:
        ZBSR *VINC_IP
        BSTR,UN EXPR_ATOM                 ; real recursive call (operand)
        ZBRR *VNEG_EXP_BODY              ; tail call: negate, return
EA_PAREN:
        ZBSR *VINC_IP                     ; consume '('
        LODA,R0 PDEPTH
        ADDI,R0 1
        STRA,R0 PDEPTH                    ; PDEPTH++ (SW-tracked, not hw RAS)
        BCTR,UN EXPR_GUARDED              ; TAIL JUMP, not call -- costs 0 hw
EXPR_ATOM:
        LODI,R0 ERR_NEST                 ; (loaded first: the conditional tail-jump below needs it)
        TPSU PE_RAS_LIMIT                ; EQ iff all mask bits of PSU.SP are set, i.e. SP == mask
        BCTA,EQ DO_ERROR                 ; every atom call is guarded: parens AND unary minus recursion
        ZBSR *VWSKIP                      ; returns with R0 = char
        COMI,R0 A'-'
        BCTR,EQ EA_NEG
        COMI,R0 A'('
        BCTR,EQ EA_PAREN
        ; not a sign or paren: falls straight into PARSE_FACTOR (placed next)

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
        LODA,R0 VARS,R1                  ; hi byte
        STRA,R0 EXPH
        LODA,R0 VARS+1,R1               ; lo byte
        STRA,R0 EXPL
        ZBRR *VINC_IP                  

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
; Clobbers: R0, R3, SC0, EXPH, EXPL, TMPH, TMPL
;PARSE_U16:
        ZBSR *VCLR_EXP
        ZBSR *VDIGIT_CHECK
        BCTA,GT JSYNERR
PU16_LP:
        ZBSR *VDIGIT_CHECK
        BCTR,GT PU16_RET                  ; unsigned range test: not 0-9 -> done
PU16_DIG:
        STRZ,R1                          ; digit -> R1 (INC_IP, EXP16_TO_ET, CLR_EXP and MULT_LOOP leave R1 alone)
        ZBSR *VINC_IP
        LODI,R0 (IDX_EXP*16)+IDX_SC0
        ZBSR *VREG16_TO_REG16              ; SC0:SC1 = EXP (the value so far)
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
;  NEG_EXP -- Negate EXPH:EXPL if NEGFLG == 0 (0 = "negate", see PARSE_S16/MD_SETUP)
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
;  ABS_X -- Absolute value of the 16-bit value at IPH+R1 (R1 = hi byte offset);
;  toggles NEGFLG if it was negative.  Falls through into NEG_SHARED.
; In:  R1 = TMPH-IPH or EXPH-IPH; NEGFLG = current flag
; Out: value = |value|; NEGFLG toggled if it was negative
; Clobbers: R0
ABS_X:
        LODA,R0 IPH,R1
        RETC,GT
        RETC,EQ                          ; hi byte >= 0: already positive
        LODA,R0 NEGFLG
        EORI,R0 $80                      ; toggle 0 <-> $80
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
        LODZ,R1
        ZBRR *VINC_ET                   ; tail call: adds 1 (INC_ET uses alt bank R1)

; =============================================================================
;  MD_SETUP -- Shared MUL/DIV setup: both operands made positive, sign recorded
; In:  TMPH:TMPL = left operand; EXPH:EXPL = right operand
; Out: TMP = |left|; SC0:SC1 = |right|; EXP = 0; NEGFLG = 0 if the result is to be
;      negated (exactly one operand was negative), else $80
; Clobbers: R0, R1, NEGFLG, SC0, SC1, EXPH, EXPL, TMPH, TMPL
; RAS: peak is the caller's depth +2 (ABS_X / REG16 / CLR_EXP inside this call) -
;   the same peak MULT_LOOP -> ADD_CORE already reaches.
MD_SETUP:
        LODI,R0 $80
        STRA,R0 NEGFLG                   ; $80 = leave sign; toggled once per negative operand
        LODI,R1 TMPH-IPH
        BSTR,UN ABS_X                    ; [+1]
        LODI,R1 EXPH-IPH
        BSTR,UN ABS_X                    ; [+1]
        LODI,R0 (IDX_EXP*16)+IDX_SC0
        ZBSR *VREG16_TO_REG16            ; SC = |EXP|
        ZBRR *VCLR_EXP                   ; tail call: clear EXP (accumulator / quotient starts at 0)

; =============================================================================
;  MUL16 -- Signed 16-bit multiply: TMPH:TMPL * EXPH:EXPL -> EXPH:EXPL
;  (The multiply itself is the leaf MULT_LOOP after DV_LP, also used by PU16 for its x10.)
; In:  TMPH:TMPL = left operand; EXPH:EXPL = right operand
; Out: EXPH:EXPL = product (16-bit two's complement wrap); tail-jumps to EXPR_LOOP
; Clobbers: R0, R1, NEGFLG, SC0, SC1, TMPH, TMPL, EXPH, EXPL
; RAS: called at depth 6 (see EXPR_GUARDED); peak is depth +2 (MD_SETUP's sub-calls
;   or MULT_LOOP -> ADD_CORE).
DO_MUL:
        BSTR,UN MD_SETUP                 ; [+1]
        BSTR,UN MULT_LOOP                ; [+1] EXP = TMP * SC0:SC1
MU_DONE:
        BSTA,UN NEG_EXP
        ZBRR *VEXPR_LOOP

; =============================================================================
;  DIV16 -- Signed 16-bit divide: TMPH:TMPL / EXPH:EXPL -> EXPH:EXPL
; The remainder is NOT kept: the in-place subtract destroys TMPH:TMPL.
; In:  TMPH:TMPL = dividend; EXPH:EXPL = divisor
; Out: EXPH:EXPL = quotient; tail-jumps to EXPR_LOOP (via MU_DONE)
; Clobbers: R0, R1, NEGFLG, SC0, SC1, TMPH, TMPL
; Error: divisor=0 -> ERR_DIV_ZERO
DO_DIV:
        LODA,R0 EXPL    ; check for zero
        IORA,R0 EXPH
        BCFR,EQ DV_GO
        LODI,R0 ERR_DIV_ZERO
        ZBRR *VDO_ERROR
DV_GO:
        BSTR,UN MD_SETUP                 ; [+1]
        ; Div loop: quotient = number of times SC0:SC1 (|divisor|) can be subtracted from TMP (|dividend|).
        ; Subtract IN PLACE with a WC-chained 16-bit subtract and test the final carry: a borrow means
        ; TMP < divisor, i.e. done - and TMP (the remainder) is dead by then.
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
        ; EXPH:EXPL += SC0:SC1, TMPH:TMPL times (TMP is consumed).
        ; Used by DO_MUL and by PU16's x10.  Decrement-first with a borrow test, using WC
        ; (carry-in) for the high byte: TMP-1 borrows out only when TMP was 0 - i.e. after
        ; exactly TMP additions - and that ends it. 
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
        LODI,R0 SC0-IPH                  ; SC0 offset from IPH (= 14)
        BSTA,UN ADD_CORE                 ; [+1] EXP += SC
        BCTR,UN MULT_LOOP

; =============================================================================
;  DO_LIST -- Print stored BASIC lines (v0.1b: whole program only -
; Syntax: LIST
; In:  PROG=program base, PEH:PEL=program end
; Out: whole program printed
; Clobbers: R0, R1, TMPH, TMPL, EXPH, EXPL, SC0, SC1 (IP untouched)
DO_LIST:
        ZBSR *VSET_PC_PROG
        db $EC                  ; COMA,R0: skip next 2 bytes 
DLS_NL:
        ZBSR *VINC_PC                    ; skip over NUL char
        ZBSR *VPRT_CRLF
DLS_LP:
        ; Check TMP against program end
        ZBSR *VCMP_PC_PE
        RETC,EQ                           ; PC == PE (GT cannot happen)

        ; Read+print line number, then rest of line verbatim.  Walks TMP
        ; directly - PRINT_S16 no longer uses TMPL as scratch, so the old
        ; TMP->IP copy here and IP->TMP copy-back at DLS_NL are both gone.
        LODI,R1 2                         ; read the 2-byte line-number header
DLS_HDR:
        LODA,R0 *PCH,R1-                 ; R1 2->1: TMP[1]; 1->0: TMP[0]
        STRA,R0 EXPH,R1                   ; EXPH+1 = EXPL
        BRNR,R1 DLS_HDR
        BSTR,UN PRINT_S16                ; print line numner and space
        ZBSR *VPRT_SPACE
        ;
        ZBSR *VINC_PC           ; Skip 1st byte of line number
DLS_BLP:
        ZBSR *VINC_PC           ; Skip 2nd byte / advance pointer
        LODA,R0 *PCH            ; Fetch char
        BCTR,EQ DLS_NL           ; On NUL -> jump to line end routine
        ZBSR *VCOUT              ; Print character
        BCTR,UN DLS_BLP           ; Loop back to top (increment + fetch)

; =============================================================================
;  PRINT_S16 -- Print signed 16-bit value EXPH:EXPL as decimal
; In:  EXPH:EXPL = signed value; WC=0
; Out: decimal digits written to COUT (leading zeros suppressed; 0 prints "0")
; Clobbers: R0, R1, R2, R3, SC0, EXPH, EXPL (TMP is NOT clobbered - DO_LIST relies on that
;   to walk TMP directly). Returns with WC=0, so COUT is always called with WC=0.
;   R1 = digit value (counted by BIRR, which leaves carry alone), R2 = P10 column
;   index 4..0 (10000..1), R3 = OR of the digit values so far (0 = still leading).
PRINT_S16:
        LODA,R0 EXPH             ; get high byte & establish CC
        BCFR,LT PS_DIGITS        ; positive: no sign
        LODI,R0 A'-'
        ZBSR *VCOUT
        ZBSR *VNEG_EXP_BODY      ; Negate, making EXPH:EXPL positive
PS_DIGITS:
        LODI,R2 4                ; first column (10000)
        EORZ,R0
        STRZ,R3                  ; R3 = 0: no non-zero digit seen yet
DIGIT_LOOP:
        PPSL $09                 ; WC=1, C=1 (= no borrow in): the 16-bit subtract chains via carry
        LODI,R1 0                ; digit counter
SUB_LOOP:
        LODA,R0 EXPL
        SUBA,R0 P10_LO,R2        ; low byte (C=1 on entry: first pass, or no-borrow from last pass)
        STRA,R0 SC0              ; tentative
        LODA,R0 EXPH
        SUBA,R0 P10_HI,R2        ; high byte, borrow-in from the low byte
        BCTR,LT BORROW           ; borrow (C=0) shows as LT: this column is finished
        STRA,R0 EXPH             ; no borrow: commit and subtract again
        LODA,R0 SC0
        STRA,R0 EXPL
        BIRR,R1 SUB_LOOP         ; digit++ (flags untouched), always non-zero: loop
BORROW:
        CPSL $08                 ; WC=0 from here on (also for COUT)
        LODZ,R1
        IORZ,R3
        STRZ,R3                  ; R3 |= digit; CC=EQ while all digits so far are zero
        BCFR,EQ PRINT_IT         ; seen a non-zero digit: print
        BRNR,R2 NEXT_DIG         ; leading zero, not the 1s column: skip it
PRINT_IT:
        LODZ,R1
        ADDI,R0 A'0'
        ZBSR *VCOUT
NEXT_DIG:
        SUBI,R2 1
        BCFR,LT DIGIT_LOOP       ; R2 4..0, then -1 ends
        RETC,UN

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
        IORZ,R0                 ; Refresh CC flags for R0 (1 byte)
        RETC,UN                 ; Return to caller (1 byte)

; =============================================================================
;  DIGIT_CHECK -- Is *IPH a decimal digit '0'-'9'?  Ported from
;  uBASIC2650wip.asm: the identical 3-instruction test was inlined 3x
;  here (TRY_STORE_LINE, PU16 x2) before this port.
; Out: R0 = char - '0'; CC=GT if not a digit (single unsigned range test)
; Clobbers: R0
DIGIT_CHECK:
        LODA,R0 *IPH
        SUBI,R0 A'0'
        COMI,R0 9
        RETC,UN

; =============================================================================
;  SHARED 16-BIT POINTER INCREMENT  - INC_ET family
; INC_PC : TMPH:TMPL += 1   (offset TMPH-IPH from IPH)
; INC_IP  : IPH:IPL  += 1    (offset 0 from IPH)
; All share INC_ET body using register bank switch.
; Rule: NO BSTA inside these -- must not consume extra RAS depth.
; Offsets are assembly-time defines (e.g. EXPH-IPH=4) 
INC_PC:
        LODI,R0 PCH-IPH        ; TMP offset from IPH (= 2); assembly-time expression
        db $E4                  ; COMI,R0 -- consume next 1 byte
INC_IP:
        EORZ,R0                 ; offset = 0 (IPH itself)
; Can jump in here with R0 set for offset
INC_ET:
        PPSL PSW_RS             ; switch to alternate register bank
        STRZ R1                 ; R1 = offset
        LODA,R0 IPL,R1          ; load lo byte
        ADDI,R0 1
        STRA,R0 IPL,R1          ; (STRA leaves CC alone)
        BCFR,EQ ET_RET          ; ADDI result != 0: no carry out of the low byte, done
        LODA,R0 IPH,R1          ; carry: increment hi byte
        ADDI,R0 1
ET_STORE:
        STRA,R0 IPH,R1
ET_RET:
        CPSL PSW_RS                 ; switch back to primary bank if not already
        RETC,UN

; =============================================================================
;  REG16_TO_REG16 -- generic 16-bit copy between any two IPH-relative
;  register pairs, addressed by a packed nibble pair (v0.34 code-golf:
;  replaces the single-parameter EXP16_TO_ET/TMP_TO_ET family below plus
;  2 previously-inline manual copy loops -- see VERSION HISTORY).
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
        LODI,R0 (IDX_CUR*16)+IDX_EXP
        ZBSR *VREG16_TO_REG16
        ZBSR *VPRINT_S16                ; [+1]
DE_NL:
        BSTR,UN PRT_CRLF
        ZBRR REPL                       ; REPL resets RAS (PSU SP bits) on entry
        
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
;  DO_PRINT / PRTSTR -- Print statement and NUL-terminated string helper
; Syntax: PRINT [item {; item}]
;   item = "string" | expr
; In:  IP -> first char after PRINT keyword
; Out: text written to COUT; IP advanced past statement
; Clobbers: R0, R1, EXPH, EXPL, TMPH, TMPL, NEGFLG, SC0, SC1
DO_PRINT:
        ZBSR *VWSKIP      ; 1st entry for bare PRINT, loop
        BCTR,EQ DP_SEP          ; blank line or null -local jump for CRLF
DP_ITEM:
        COMI,R0 DQ
        BCTR,EQ DP_STRING
        ; Expression
        ZBSR *VPARSE_EXPR  
        ZBSR *VPRINT_S16
        db $EC                            ; COMA,R0 -- consume next 2 bytes
DP_SCLS:
        ZBSR *VINC_IP  
DP_SEP:
        ZBSR *VWSKIP
        COMI,R0 $3B             ; semicolon
        BCFR,EQ PRT_CRLF        ; tail call: return from DO_PRINT
        ZBSR *VINC_IP  
        ZBSR *VWSKIP      ; get new character
        RETC,EQ                 ; no CR
        BCTR,UN DP_ITEM        ; more items to print

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
; CLR_EXP -- Helper Zeroes EXP
; Clobbers R0
CLR_EXP:
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 EXPL
        RETC,UN

; =============================================================================
;  DO_WR -- Write raw character, replaces CHR$ A statement, not a PRINT item
;  Consecutive WRs need no PRINT/semicolon like  "PRINT CHR$(a);CHR$(b);" does
; Syntax: WR expr
; In:  IP -> first char after WR keyword
; Out: low byte of expr's value written to COUT (no newline)
; Clobbers: R0, EXPH, EXPL, TMPH, TMPL, NEGFLG, SC0, SC1
DO_WR:
        ZBSR *VPARSE_EXPR                 ; (EXPR_ATOM skips leading spaces itself)
        LODA,R0 EXPL
        ZBRR *VCOUT              ; tail call

; =============================================================================
;  ADV_PC_PAST_REC -- Advance TMPH:TMPL past the current stored line record
; Skips the 2-byte line-number header, scans forward until NUL (end of that
; record's text), then skips the NUL too - leaves TMPH:TMPL pointing at the
; start of the NEXT record (or PE, if this was the last one). 
; In:  TMPH:TMPL -> start of a stored record (its line-number hi byte)
; Out: TMPH:TMPL -> start of the next record
; Clobbers: R0
ADV_PC_PAST_REC:
        ZBSR *VINC_PC
APR_LP:
        ZBSR *VINC_PC
        LODA,R0 *PCH
        BCFR,EQ APR_LP                  ; free zero-test: NUL ends the body
        ZBRR *VINC_PC                    ; skip the NUL itself

; =============================================================================
;  TABLES 
BANNER:
        DB CR, LF, "pBASIC 0.35", CR, LF, NUL

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

; Powers of 10 Tables, ascending: index 0..4 = 1, 10, 100, 1000, 10000
P10_HI:
        db $00, $00, $00, $03, $27
P10_LO:
        db $01, $0A, $64, $E8, $10

ROMEND: ; for auditing

;  RAM variables -- sequential RES block 
 
        ORG     4096    ; half a 2650 8kbyte page

; --- Ordered group: offsets from IPH used by INC_ET, NEG_SHARED, and
;     REG16_TO_REG16's packed-nibble index scheme (see IDX_* EQUs, v0.34).
;     No FOR/NEXT in this build, so (unlike uBASIC) there's no push/pop
;     block that constrains ordering here - laid out tightly, no padding
;     needed since every entry is exactly 2 bytes.
;     IBUF's hi==lo GETLINE trick was dropped when this group grew past 16
;     bytes to bring PE/SC0 into nibble range (+2B in GETLINE, see VERSION
;     HISTORY).
IPH     RES 1       ; interpreter pointer hi       (INC_ET offset 0; IDX_IP=0)
IPL     RES 1       ; interpreter pointer lo
TMPH    RES 1       ; temp 16-bit hi               (INC_ET offset 2 = TMPH-IPH; IDX_TMP=1)
TMPL    RES 1       ; temp 16-bit lo
GOTOH   RES 1       ; pending target hi (handler address for the matched
                    ; operator/keyword - not currently read via an offset; IDX_GOTO=2)
GOTOL   RES 1       ; pending target lo
CURH    RES 1       ; current line hi  (error reporting; IDX_CUR=3)
CURL    RES 1       ; current line lo
PCH     RES 1       ; program counter / record pointer hi (RUN, GOTO, LIST, line store; IDX_PC=4)
PCL     RES 1       ; program counter lo
EXPH    RES 1       ; expression result hi         (INC_ET offset 4 = EXPH-IPH; IDX_EXP=5)
EXPL    RES 1       ; expression result lo
; Program end pointer. DEMO/TEST build: preset by the ASSEMBLER to the end of the
; preloaded showcase (RAM contents ride in with the hex file, exactly like the
; showcase text itself), so MAIN needs no code to set it.
; ROM build: make these "PEH RES 1" / "PEL RES 1" (delete the EQU), drop the
; showcase DB block at PROG, and in MAIN add
; "BSTA,UN DO_NEW" (DO_NEW sets PE = PROG and then clears RUNFLG).
; Deliberately ONE "DW" (hi, lo) with PEL as an EQU: asm2650 (through at least
; v1.17) counts a "DB" whose operand is a FORWARD label as 0 bytes in pass 1, which
; silently shifts every later label; "DW" is counted correctly.
PEH     DW SHOWCASE_END        ; Program end pointer hi (then lo)          (IDX_PE=6)
PEL     EQU PEH+1              ; Program end pointer lo
SC0     RES 1       ; Scratch byte 0                                      (IDX_SC0=7)
SC1     RES 1       ; Scratch byte 1

; Buffers
IBUF    RES 64      ; Input buffer 64 bytes

; --- Remaining --- (not nibble-indexed)
PDEPTH  RES 1       ; SW-tracked paren nesting depth 

;  --- Flags & Stuff --- 
RUNFLG  RES 1       ; $01=running $00=immediate
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
        DB 0,10,"PRINT ",$22,"-- Append/Delete Only Line Edits --",$22,$00
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
