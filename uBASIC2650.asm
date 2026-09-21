; =============================================================================
; uBASIC2650 v2.7  --  Minimal Tiny BASIC for the Signetics 2650
; Copyright (c) 2026 Vincent Crabtree, licensed under the MIT License, see LICENSE
;
; Note: standalone build - I/O is bit-banged in software, no PIPBUG ROM or
; UART/ACIA hardware required.
;
;   CPU    : Signetics 2650
;   ROM    : <2 KB target, $0000 upward (currently 2002 bytes - see ROMEND)
;   RAM    : ~213 bytes, $1000 upward (half an 8 KB page above the code)
;   I/O    : CHIN/COUT, bit-banged software serial via PSU/PSL flag bits.
;            Addresses float per build - read them from the .LST (see BUILD)
;   NMI/IRQ: Not used
;
; Statements:
;   INPUT  END  GOTO <expr>  GOSUB <expr>  IF <cond> [THEN] <stmt>
;   LIST  NEW  PRINT  RETURN  RUN  [LET] <var>=<expr>  (LET and THEN optional)
;   FOR <var>=<expr> TO <expr>   NEXT [<var>]
;
; PRINT items: "literal", CHR$(n), TAB(n), or an expression; separate with ';'.
;
; Arithmetic: + - * /  (unary -).  
; Precedence: BODMAS-lite * / binds before + -, which binds before relops (= < >).
; Relops: `=`  `<`  `>`  prefix with ! to invert: `!=`, `!<`, `!>` (<= is !>, >= is !<).
; RND: niladic pseudorandom 16-bit signed function, no parenthesis/arg.
;
; Numbers : signed 16-bit  (-32768 .. 32767)
; Variables: A-Z (26), 16-bit signed; no arrays or string variables
; Print   : "literals", CHR$(n), TAB(n), `;` separator - no string vars
;
; KNOWN LIMITATIONS
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
;   FOR <var>=<start> TO <limit>, then NEXT [<var>].  Step is always 1.
;   - Test at end so Body always runs at least once
;   - NEXT always closes the innermost loop
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
;   ./asm2650 uBASIC2650.asm uBASIC2650.hex
;   grep -n "^CHIN \|^COUT \|^ROMEND " .\uBASIC2650.LST
;   ./pipbug_wrap --entry 0 --chin 0x<addr> --cout 0x<addr> uBASIC2650.hex
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
; v2.7 (Sep 2026) - BASIC line editing, LET/THEN/INPUT and FOR/NEXT restored.
;   - TRY_STORE_LINE: FIND_LINE; if the line exists DEL_REC removes it and
;     FIND_LINE is repeated.  New DEL_REC (fwd copy), OPEN_GAP (backward copy)
;     and DEC_PE. Line numbers > 32767 rejected with syntax error. 
;   - PROGLIM (EQU $1FFF, must be $xxFF): OPEN_GAP builds the new PE in EXP,
;     checks its hi byte, and only then moves anything; store full gives ?3
;     (ERR_OOM) with the store untouched (+10). A REPLACE that does not fit has
;     already deleted the old line. 
;   - Relop refactor, added '>': 3-way compare, each handler picks its CC (EQ/LT/GT).
;   - FOR/NEXT note STEP 1 only, body runs at least once.
;   - SHOWCASE updated for new keywords amd relops.
;   - ROMEND $06F6 (1782) -> $07D6 (2006 bytes)
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
;   - DR_HDR: rraccred to use shared TMP_TO_ET call 
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
; Branched from pBASIC, variants below
; v0.22 (Sep 2026) - Zero-page vector audit (ZBSR/ZBRR vs direct BSTA/BCTA)
;   - Refactor DO_MUL/DO_DIV to share one setup body and pick their loop via R1
;     used in OPS_HIT.
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
ERR_OOM         EQU '3'         ; GOSUB/FOR stack full, RETURN/NEXT with nothing pushed, program store full
ERR_VAR         EQU '4'
ERR_EXPR        EQU '8'         ; Expression too complex (SW stack full)
GSSTKLIM        EQU 8           ; 4 GOSUB levels x 2 bytes
FSTKLIM         EQU 20          ; 4 FOR levels x 5 bytes
PROGLIM         EQU $1FFF       ; last usable program-store address. MUST be $xxFF (one
                                ; below a page boundary): OPEN_GAP checks the new PE's
                                ; hi byte only. Change for other RAM sizes (1 KB: $13FF)

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
VCLR_EXP:
        DW CLR_EXP
VDO_ERROR:
        DW DO_ERROR
VSET_TMP_PROG:
        DW SET_TMP_PROG
VCMP_TMP_PE:
        DW CMP_TMP_PE
VWSKIP_PEEK:
        DW WSKIP_PEEK
VNEG_EXP_BODY:
        DW NEG_EXP_BODY
VEXP16_TO_ET:
        DW EXP16_TO_ET
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
        LODA,R0 CURH,R1-                 ; 
        STRA,R0 EXPH,R1                   ; 
        BRNR,R1 DE_LOOP
        BSTA,UN PRINT_S16                ; [+1]
DE_NL:
        BSTR,UN PRT_CRLF
        BSTA,UN CLR_RUNFLG                ; Not running
        BCTR,UN REPL                     ; REPL resets RAS (PSU SP bits) on entry

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
; Clobbers: R0, R1, EXPH, EXPL (via EXPR, plus whatever the dispatched
;           statement clobbers on the true path)
DO_IF:
        LODA,R0 RXSAVE
        COMI,R0 A'P'            ; INPUT
        BCTA,EQ DO_ASK          ; 

        ZBSR *VPARSE_EXPR                      ; [+1] condition -> EXPH:EXPL
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
; RAS depth: 1 from REPL, 3 from DO_IF's true path (which jumps here).
STMT_EXEC:
        ZBSR *VWSKIP_PEEK
        BSTA,UN PEEK_C2_ALPHA     ; R2 is 1st char
        BCFR,EQ SE_NOTKW          ; 2nd char is NOT a letter -> handle as var/expr
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
OPS_HIT_RET:
        ; Pop the R1 table offset back off the stack
        LODA,R0 SWBASE,R3        ; R0 = top of stack (our saved R1 offset)
        STRZ,R1                  ; R1 = R0 
;  POP_TMP -- pop a 2-byte value pushed on SWBASE into TMPH:TMPL and jump to it
        LODA,R0 SWBASE,R3-      ; predecrement
        STRA,R0 TMPH
        LODA,R0 SWBASE,R3-
        STRA,R0 TMPL
        SUBI,R3 1
        ; jump to vector
        BCTR,UN JMP_VEC

SE_NOTKW:
        ; Bare variable assignment ("X=expr" - either the 2nd-char peek
        ; above wasn't a letter, or the 1st char matched no statement).
        BSTR,UN PARSE_VAR_SAVE            ; validates A-Z, R2 = letter, IP -> past it
        ZBSR *VWSKIP_PEEK
        COMI,R0 A'='
        BCFA,EQ JSYNERR
        ZBSR *VINC_IP
        ; drop through
; =============================================================================
;  DL_EX / DL_STORE -- Variable assignment (v0.3 removed DO_LET's own
;  prologue; v2.7 brings the optional LET keyword back at no handler cost:
;  the 'L' row lands in DO_LIST, which sends LET straight to SE_NOTKW.  Reached
;  from SE_NOTKW's "V=expr" path (bare or after LET) and from DO_ASK.)
; In:  IP -> expression (DL_EX) or R2 = VARS byte-offset, EXPH:EXPL = value
;      (DL_STORE - PARSE_VAR_SAVE already converts the letter to R2=index*2)
; Out: VARS[V] = EXPH:EXPL
; Clobbers: R0, EXPH, EXPL, TMPH, TMPL (via PARSE_EXPR, DL_EX only)
DL_EX:
        ZBSR *VPARSE_EXPR                 ; [+1]
DL_STORE:
        LODA,R0 EXPH     ; R0 = High byte of expression
        STRA,R0 VARS,R2  ; Store directly to VARS array + offset (R2 is a
        LODA,R0 EXPL     ; R0 = Low byte of expression                  valid
        STRA,R0 VARS+1,R2; Store directly to VARS array + offset + 1    index reg)
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
        BSTA,UN PARSE_S16                ; [+1]
        BCTR,UN DL_STORE

; =============================================================================
; PARSE_VAR_SAVE -- skip whitespace, read var letter, range-check, save to
;                   R2, advance IP.
; Out: R2=letter (A-Z); IP advanced past letter
; Error: tail-jumps to JERRVAR (no return)
; Clobbers: R0, R1, R2
PARSE_VAR_SAVE:
        ZBSR *VWSKIP_PEEK
        SUBI,R0 A'A'                     ; shift 'A'-'Z' down to 0-25
        COMI,R0 A'Z'-A'A'                ; single unsigned range test (COM=1
        BCTR,GT JERRVAR                  ; set globally) catches < 'A' and > 'Z'
        ADDZ,R0                          ; R0 = index*2 (16-bit word stride)
        STRZ,R2                          ; R2 = byte offset in VARS, for DL_STORE
        ZBRR *VINC_IP         ; tail call  

; =============================================================================
;  GETLINE -- Read a line from input into IBUF; also points IP at it
; In:  nothing
; Out: IBUF = NUL-terminated input line; IPH:IPL = IBUF (both callers used
;      to re-point IP via a separate SET_IP_IBUF call right after this -
;      folded in here instead, since IBUF's address ($1010) has hi==lo,
;      so one LODI sets both bytes)
; Clobbers: R0, R1
GETLINE:
        LODI,R0 <IBUF                    ; IBUF hi byte == lo byte ($10) -
        STRA,R0 IPH                      ; one load sets both IPH and IPL
        STRA,R0 IPL
        LODI,R1 $FF                      ; R1 = empty-buffer sentinel (pre-inc convention)
GL_LP:
        BSTA,UN CHIN                     ; [+1] blocking read
        COMI,R0 CR+1
        BCTR,LT GL_EOL                  ; everything less than CR
        STRA,R0 IBUF,R1+                 ; R1++ (pre-inc); IBUF[R1]=char
        ZBSR *VCOUT  
        BCTR,UN GL_LP
GL_EOL:
        EORZ,R0
        STRA,R0 IBUF,R1+                 ; R1++ (pre-inc, one past last char); NUL-terminate
        BCTA,UN PRT_CRLF                ; tail call

; =============================================================================
;  DO_GO -- 'G' dispatch: GOTO or GOSUB.  TOK_CHARS only matches the first
;  letter, so the two share a row; STMT_EXEC stashes the keyword's 3rd char
;  in RXSAVE before dispatch (GOTO's is 'T', GOSUB's is 'S'). 
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
        BSTA,UN EXP16_TO_LNUM              ; LNUMH:LNUML = EXPH:EXPL (target line)
        BSTA,UN FIND_LINE                  ; [+1] TMPH:TMPL = found record
        BCTA,UN TMP_TO_SWSTK              ; Tail call

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
        ; what ADV_PAST_RECORD (below) expects.
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
        EORZ,R0                           ; offset 0 (IPH-IPH)
        BSTA,UN TMP_TO_ET
        
        ZBSR *VINC_IP
        ZBSR *VINC_IP
        ; advance TMP past this whole record (header+body+terminator) to
        ; find the next one; stash for SWSTK/DR_LP's next iteration (or a
        ; GOTO's redirection). Reuses the same shared scan FIND_LINE/
        ; FIND_INS/TSL_MATCH already use - one routine, one terminator.
        BSTA,UN ADV_PAST_RECORD
        BSTA,UN TMP_TO_SWSTK
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
;  Frame is read back-to-front with pre-decrement loads.
; In:  FSP = bytes used
; Out: var+1; SWSTK = body (loop again) or frame popped (finished)
;      Error: ERR_OOM if there is no FOR active
; Clobbers: R0, R1, R2, R3, EXP, TMP  (R3 = frame index: CMP_OPS trashes R1)
DO_N:
        LODA,R0 RXSAVE
        COMI,R0 A'X'
        BCFR,EQ DO_NEW
DO_NEXT:
        LODA,R3 FSP
        BCTA,EQ DRT_UFLOW                ; NEXT with no FOR
        LODA,R0 FSTK,R3-                 ; var's VARS offset
        STRZ,R2
        ADDI,R0 VARS-IPH
        ZBSR *VINC_ET                    ; var++ (INC_ET keeps R3,R2)
        LODA,R0 VARS,R2                  ; TMP = var
        STRA,R0 TMPH
        LODA,R0 VARS+1,R2
        STRA,R0 TMPL
        LODA,R0 FSTK,R3-                 ; EXP = limit
        STRA,R0 EXPL
        LODA,R0 FSTK,R3-
        STRA,R0 EXPH
        BSTA,UN CMP_OPS                  ; var : limit (signed)
        BCTR,GT DN_POP                   ; var > limit: loop is finished
        LODA,R0 FSTK,R3-                 ; body -> SWSTK: DR_CD resumes there
        STRA,R0 SWSTK+1
        LODA,R0 FSTK,R3-
        STRA,R0 SWSTK
        RETC,UN
DN_POP:
        SUBI,R3 2                        ; R3 = frame base: pop it
        STRA,R3 FSP
        RETC,UN

; -----------------------------------------------------------------------------
; PEEK_C2_ALPHA - Checks if the 2nd character (IPH+1) is a letter (A-Z).
; Inputs:   None (uses IPH)
; Outputs:  Condition Code EQ if char 2 is A-Z.
;           Condition Code LT/GT (Not EQ) otherwise.
; -----------------------------------------------------------------------------
PEEK_C2_ALPHA:
        STRZ,R2            ; R2 = first char
        LODI,R1 1
        LODA,R0 *IPH,R1                  ; peek 2nd char
        SUBI,R0 A'A'                     ; Shift 'A' down to 0
        COMI,R0 A'Z'-A'A'                ; Compare against 25 (length of alphabet - 1)
        BCTR,GT PCA_RET                 ; Unsigned compare catches both < 'A' and > 'Z'      
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
        BSTA,UN PRINT_S16        ; 2b
        db $EC                  ; 1b: COMA,R0 opcode - consumes next 2 bytes (ZBSR *VINC_IP)
DP_SCLS:
        ZBSR *VINC_IP           ; 2b
DP_SEP:
        ZBSR *VWSKIP_PEEK       ; 2b
        COMI,R0 $3B             ; 2b: semicolon ';'
        BCFA,EQ PRT_CRLF        ; 3b: branch if NOT ';' (was 3b BCFA)
        ZBSR *VINC_IP           ; 2b
        ZBSR *VWSKIP_PEEK       ; 2b
        RETC,EQ                 ; 1b: trailing ';' followed by NUL -> exit without CRLF
        BCTR,UN DP_ITEM         ; 2b

DO_PRINT:
        ZBSR *VWSKIP_PEEK       ; 2b
        BCTA,EQ PRT_CRLF        ; 2b: bare PRINT -> print CRLF & return
DP_ITEM:
        COMI,R0 DQ              ; 2b
        BCTR,EQ DP_STRING       ; 2b
        
        BSTR,UN PEEK_C2_ALPHA   ; 2b
        BCFR,EQ DP_EXPR         ; 2b: 2nd char not alpha -> expr
        COMI,R2 A'C'            ; 2b
        BCTR,EQ DP_KW           ; 2b
        COMI,R2 A'T'            ; 2b
        BCFR,EQ DP_EXPR         ; 2b
DP_KW:
        ZBSR *VEATWORD          ; 2b: consume keyword
        ZBSR *VPARSE_EXPR       ; 2b: parse (n)
        LODA,R0 EXPL            ; 3b: load low byte into R0 once for both CHR$ and TAB
        COMI,R2 A'C'            ; 2b
        BCTR,EQ DP_C_KW         ; 2b
        STRZ R1                 ; 1b: 2650 opcode $51 — copies R0 -> R1 & sets CC (EQ if 0)
        BCTR,EQ DP_SEP          ; 2b: TAB(0) no-op
DP_TAB_LOOP:
        ZBSR *VPRT_SPACE        ; 2b
        BDRR,R1 DP_TAB_LOOP     ; 2b
        BCTR,UN DP_SEP          ; 2b
DP_C_KW:
        ZBSR *VCOUT             ; 2b: print R0 directly
        BCTR,UN DP_SEP          ; 2b

; =============================================================================
;  GET_RND -- snapshot current seed as the RND result, then advance it
; In:  None
; Out: EXPH:EXPL = pseudorandom 16-bit value (the seed BEFORE this call)
; Clobbers: R0, R1, RNDSEED (falls through into RND_SHUFFLE - one shared RETC)
GET_RND:
        LODI,R1 2
GR_LP:
        LODA,R0 RNDSEED,R1-                
        STRA,R0 EXPH,R1                 
        BRNR,R1 GR_LP
        ; drop through
; =============================================================================
;  RND_SHUFFLE -- Advance 16-bit Galois LFSR (Little-Endian) in place
; In:  None (reads RNDSEED)
; Out: RNDSEED advanced one step
; Clobbers: R0, R1, RNDSEED
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
; Character IO
; 110 Baud teletype from PIPBUG V1 as per Signetics M20 application note
; Simulator default expects CHIN $286 COUT $2B4
 ;       ORG $286

CHIN:
        BSTR,UN RND_SHUFFLE 
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
        LODA,R0 *IPH
        SUBI,R0 A'0'                     ; shift '0'-'9' down to 0-9
        COMI,R0 9                        ; single unsigned range test
        BCFR,GT TSL_NUM                  ; in 0-9 -> a numbered line
TSL_NO:
        EORZ,R0                          ; CC=EQ: not a numbered line
        RETC,UN
TSL_NUM:
        BSTA,UN PARSE_S16                ; [+1] IP already on the digit just tested
        LODA,R0 EXPH                     ; digits only, so bit 7 set means the
        BCTA,LT JSYNERR                  ; number is > 32767 ("negative"): reject
        BSTA,UN EXP16_TO_LNUM             ; LNUMH:LNUML = parsed line number
TSL_FND:
        BSTA,UN FIND_LINE                ; [+1] TMP = insertion point (first record
                                          ; >= LNUM); CC=EQ iff that record IS LNUM
        BCFR,EQ TSL_WRITE                 ; no such line: go store the new one
        BSTR,UN DEL_REC                   ; remove the old copy, then re-find: the
        BCTR,UN TSL_FND                   ; insertion point is the same address, but
                                          ; TMP was consumed by DEL_REC
TSL_WRITE:
        ZBSR *VWSKIP_PEEK
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
        LODI,R0 EXPH-IPH
        BSTA,UN TMP_TO_ET                ; EXP = dst = start of the doomed record
        BSTA,UN ADV_PAST_RECORD          ; TMP = src = start of the next record
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
        LODI,R0 PEH-IPH
        ZBRR *VEXP16_TO_ET               ; PE = dst (tail call)

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
        LODI,R1 2                        ; EXP = PE
OG_SAV:
        LODA,R0 PEH,R1-                  ; R1 2->1: PEL; 1->0: PEH
        STRA,R0 EXPH,R1
        BRNR,R1 OG_SAV
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
        BSTR,UN DEC_PE
        LODA,R0 *PEH                     ; byte at PE ...
        STRA,R0 *PEH,R3                  ; ... goes to PE+R3
        BCTR,UN OG_LP
OG_FIX:
        LODI,R0 PEH-IPH
        ZBRR *VEXP16_TO_ET               ; PE = EXP (tail call)

; =============================================================================
;  DEC_PE -- PEH:PEL -= 1
;  Borrow is read from carry (TPSL $01: EQ = C=1 = no borrow), not from the
;  result's CC - the CC after a SUB is only the sign of the result byte.
; Out: PE decremented
; Clobbers: R0
DEC_PE:
        LODA,R0 PEL
        SUBI,R0 1
        STRA,R0 PEL
        TPSL $01
        RETC,EQ                          ; no borrow: hi byte untouched
        LODA,R0 PEH
        SUBI,R0 1
        STRA,R0 PEH
        RETC,UN

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
                                          ; no need to materialize TMP+1
                                          ; anywhere (was INC16_TMP_TO_EXP)
        SUBA,R0 LNUML
        BCFR,EQ FL_RET_NF                 ; lo byte mismatch -> not found
FL_FOUND:
        EORZ,R0                          ; CC=EQ: found
        RETC,UN

; =============================================================================
;  ADV_PAST_RECORD -- Advance TMPH:TMPL past the current stored line record
; Skips the 2-byte line-number header, scans forward until NUL (end of that
; record's text), then skips the NUL too - leaves TMPH:TMPL pointing at the
; start of the NEXT record (or PE, if this was the last one).
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
        BSTR,UN ADV_PAST_RECORD
        ZBSR *VCMP_TMP_PE
        RETC,GT
        RETC,EQ
        ;
        LODA,R0 LNUMH
        SUBA,R0 *TMPH                    ; LNUMH - stored.hi (signed: fine, both < $80)
        BCTR,GT FI_ADV
        RETC,LT                          ; v1.2: was BCTR,LT FI_RET (branch-to-return; RETC has no false-polarity form, but BCTR's TRUE-polarity LT collapses directly)
        ; hi bytes equal: check lo
        LODI,R1 1                         ; stored lo byte is at TMP+1
        LODA,R0 LNUML
        COMA,R0 *TMPH,R1                  ; unsigned compare (COM=1 set in MAIN);
                                          ; indirect-indexed peek (was
                                          ; INC16_TMP_TO_EXP)
        BCTR,GT FI_ADV
        RETC,UN

; =============================================================================
;  EXPR -- Expression evaluator.
; In:  IPH:IPL -> expression string
; Out: EXPH:EXPL = 16-bit result ($0000/$FFFF for relops - see DO_EQOP/
; Clobbers: R0, R1, R3, BANG, SAVEH, SAVEL, NEGFLG, SC0, TMPH, TMPL


HI_LOOP:
        ZBSR *VWSKIP_PEEK              ; R0 has character
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

OPS_HIT_COM:
        ; Push left operand to the LIFO stack
        LODA,R0 EXPL
        STRA,R0 SWBASE,R3+
        LODA,R0 EXPH
        STRA,R0 SWBASE,R3+
        ; Push the R1 table offset to the stack to survive recursion
        LODZ,R1
        STRA,R0 SWBASE,R3+
        ZBSR *VINC_IP            ; consume the operator char
        LODI,R0 >OPS_HIT_RET
        LODI,R1 <OPS_HIT_RET
        ZBRR *VPUSH_RET         ; tail call

LO_LOOP:
        ZBSR *VWSKIP_PEEK
        STRA,R0 SC0
        LODI,R1 18                       ; last LO-row's char offset (5 rows:
                                          ; + - = < >, walking down to row 6 -
                                          ; HI's own 2 rows sit below that)
LO_SCAN:
        LODA,R0 TOK_CHARS,R1
        SUBA,R0 SC0
        BCTR,EQ OPS_HIT_LO
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
        ; No operator matched: this level's flat chain is done - go find out
        ; where to resume (another paren level, an outer operator, or truly
        ; done). Whichever continuation is on top of the SW stack (or
        ; hardware, if empty) knows what to do - including consuming a ')'
        ; if that's what EA_PAREN's own EP_RET (below) is waiting for.

; OPS_HIT_RET is near STMT_EXEC

EXPR:
        LODI,R3 $FF                      ; SW cont-stack empty sentinel
        EORZ,R0
        STRA,R0 BANG                      ; relop-invert modifier off
        BSTR,UN PUSH_LOLOOP                 ; "once the HI-tier chain is fully
                                            ; exhausted, scan for +-=<> here"
        BSTR,UN PUSH_HILOOP                 ; "once the first atom resolves,
                                            ; check for */  chaining first"
        db $EC                           ; COMA,R0: consume next 2 bytes
        ; drop through
; =============================================================================
;  EXPR_ATOM -- parse one atom: unary +/-, parens, RND, or a literal/variable.
; In:  IPH:IPL -> atom
; Out: EXPH:EXPL = value
; Clobbers: R0, R1 (RND path only, via TRY_RND/GET_RND); R2 is NOT clobbered
;   here or by anything this calls - see REGISTER CONVENTIONS re: R2
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

        COMI,R0 A'R'
        BCFR,EQ END_RND                      ; 'R': maybe RND 
;TRY_RND:
        LODI,R1 1
        LODA,R0 *IPH,R1          ; peek 2nd char (indirect-indexed, r0=dest)
        SUBI,R0 A'A'
        COMI,R0 A'Z'-A'A'
        BCTR,GT END_RND          ; not alpha -> bare variable R
        ZBSR *VEATWORD           ; consume RND
        BSTA,UN GET_RND          ; EXPH:EXPL = seed; advances LFSR for next call
        ZBRR *VPARSER_RET

END_RND:
        BSTA,UN PARSE_FACTOR     ; only caller but              
        ZBRR *VPARSER_RET

EA_NEG:
        ZBSR *VINC_IP  
        LODI,R0 >NEG_RET
        LODI,R1 <NEG_RET
        ZBSR *VPUSH_RET
        ZBRR *VEXPR_ATOM                   ; parse the operand

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
        BSTA,UN CARRY_INTO_EXPH            ; EXPL=result; EXPH+=1 iff carried
        LODA,R0 EXPH
        ADDA,R0 TMPH
        STRA,R0 EXPH
        BSTR,UN PUSH_LOLOOP
        ZBRR *VPARSER_RET

; =============================================================================
;  CMP_OPS / DO_EQOP / DO_LTOP / DO_GTOP -- relop handlers. One shared signed
;  3-way compare of popped LH operand (TMPH:TMPL) vs RH (EXPH:EXPL), then each
;  handler just picks which CC means true; the result is $0000 (false) or
;  $FFFF (true) in EXPH:EXPL then CC bawed on desired test. 
;  All six relops are direct: <=  is  !>,  >=  is  !<, ;  <>/!=  is  !=.
; In:  TMP = left, EXP = right (CMP_OPS)
; Out: CC = LT/EQ/GT as left is <, =, > right (CMP_OPS); handlers jump on
;      to DOP_TRUE / DOP_FALSE
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

DO_EQOP:
        BSTR,UN CMP_OPS
        BCTR,EQ DOP_TRUE
        BCTR,UN DOP_FALSE
DO_LTOP:
        BSTR,UN CMP_OPS
        BCTR,LT DOP_TRUE
        BCTR,UN DOP_FALSE
DO_GTOP:
        BSTR,UN CMP_OPS
        BCTR,GT DOP_TRUE
        ; drop through: false
DOP_FALSE:
        EORZ,R0
        db $EC                            ; COMA,R0 -- consume next 2 bytes
DOP_TRUE:
        LODI,R0 $FF
        ; both paths converge here (see EORZ,R0/db $EC above): R0 = $00
        ; (false) or $FF (true). Apply the '!' modifier, if OPS_LP armed
        ; it, then disarm it - consumed once per relop, regardless of
        EORA,R0 BANG                      ; R0 ^= BANG ($00 no-op / $FF flips)
        STRA,R0 EXPH
        STRA,R0 EXPL
        EORZ,R0
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
;  PF_LOADVAR -- Load variable value from VARS
; In:  R0 = index (0..25, PARSE_FACTOR's SUBI result - not re-subtracted
;      here); IP -> that letter char
; Out: EXPH:EXPL = variable value
; Clobbers: R0, R1
PF_LOADVAR:
        ADDZ,R0                          ; R0 = index*2 (word stride)
        STRZ,R1                          ; R1 = index*2 - survives INC_IP:
                                          ; the INC_ET family bank-switches
                                          ; (PSW_RS) so primary R1/R2/R3 are
                                          ; preserved across the call, only
                                          ; R0 is clobbered
        ZBSR *VINC_IP                     ; advance IP past the letter
        LODA,R0 VARS,R1                  ; hi byte
        STRA,R0 EXPH
        LODA,R0 VARS+1,R1               ; lo byte
        STRA,R0 EXPL
        RETC,UN

PRO_NONE:
        BCTA,UN JSYNERR 

PF_NUM:
;       drop through
; =============================================================================
;  PARSE_S16 -- Parse signed decimal integer
; In:  IPH:IPL -> first char (optional '-' then digits)
; Out: EXPH:EXPL = signed 16-bit value
; Clobbers: R0, NEGFLG, EXPH, EXPL
PARSE_S16:
        LODA,R0 *IPH
        EORI,R0 A'-'                      ; R0 = 0 iff a minus sign
        STRA,R0 NEGFLG                    ; NEGFLG polarity: 0 = negate the
                                          ; result (see NEG_EXP), any other
                                          ; value = leave. Written
                                          ; unconditionally so no separate
                                          ; clear is needed anywhere.
        BCFR,EQ PS16_UN                   ; not '-' (EORI's CC survives the
                                          ; STRA - STRA never touches CC)
PS16_NEG:
        ZBSR *VINC_IP  
PS16_UN:
;       drop through
; =============================================================================
;  PARSE_U16 -- Parse unsigned decimal digits -> EXPH:EXPL
; Jumps to JSYNERR if no digits found.
; In:  IPH:IPL -> first digit char
; Out: EXPH:EXPL = value
; Clobbers: R0, R1, SC0, SC1, EXPH, EXPL, TMPH, TMPL
;PARSE_U16:
        ZBSR *VCLR_EXP
        LODA,R0 *IPH
        SUBI,R0 A'0'                      ; shift '0'-'9' down to 0-9
        COMI,R0 9                         ; single unsigned range test
        BCTR,GT PRO_NONE; surrogate for JSYNERR
PU16_LP:
        LODA,R0 *IPH
        SUBI,R0 A'0'
        COMI,R0 9
        BCTR,GT NEG_EXP                    ; not a digit -> end of number
PU16_DIG:
        STRZ,R1                          ; digit -> R1 (INC_IP, EXP16_TO_ET,
                                         ; CLR_EXP and MULT_LOOP all leave
                                         ; primary R1 alone - MULT_LOOP is a
                                         ; leaf using R0 only, so this also
                                         ; needs no R3 save/restore the way
                                         ; the old fixed-count BDRR loop did)
        ZBSR *VINC_IP
        LODI,R0 SC0-IPH
        ZBSR *VEXP16_TO_ET              ; SC0:SC1 = EXP (the value so far)
        ZBSR *VCLR_EXP                   ; EXP = 0 ...
        LODZ,R1
        STRA,R0 EXPL                     ; ... then = the digit: MULT_LOOP
                                         ; *accumulates* into EXP, so
                                         ; preloading the digit gives
                                         ; digit + 10*value in one go (no
                                         ; separate final add/carry step)
        LODI,R0 10
        STRA,R0 TMPL                     ; TMP = 10: x10 via the shared MULT_LOOP
        EORZ,R0
        STRA,R0 TMPH
        BSTA,UN MULT_LOOP                ; EXP = digit + value*10
        BCTR,UN PU16_LP

; =============================================================================
;  NEG_EXP -- Negate EXPH:EXPL if NEGFLG says to
;  NEG_EXP_BODY -- Unconditional negate EXPH:EXPL
; In:  EXPH:EXPL = value; NEGFLG = flag (0 = negate, nonzero = leave)
; Out: EXPH:EXPL negated (two's complement) if NEGFLG==0
; Clobbers: R0, R1
NEG_EXP:
        LODA,R0 NEGFLG
        RETC,GT                          ; nonzero (either sign) -> leave;
        RETC,LT                          ; needs both since NEGFLG's non-
                                         ; zero producers use arbitrary
                                         ; byte patterns (PARSE_S16) or a
                                         ; specific negative one ($80, from
                                         ; ABS_TMP/ABS_EXP below) - only a
                                         ; true zero means "negate"
NEG_EXP_BODY:
        LODI,R1 EXPH-IPH                 ; EXPH offset from IPH (= 4); R1 variant for NEG_SHARED
        BCTR,UN NEG_SHARED

; =============================================================================
;  ABS_TMP -- Absolute value of TMPH:TMPL; sets NEGFLG to reflect the sign
; In:  TMPH:TMPL = signed value
; Out: TMPH:TMPL = |value|; NEGFLG = $00 if was negative (see NEG_EXP),
;      $80 if was positive. Written unconditionally - caller does not need
;      to pre-clear NEGFLG.
; Clobbers: R0, R1
ABS_TMP:
        LODA,R0 TMPH
        ANDI,R0 $80                      ; $80 if negative, $00 if positive
        EORI,R0 $80                      ; flip: negative->$00 (=negate),
        STRA,R0 NEGFLG                   ; positive->$80 (=leave)
        RETC,LT                          ; R0=$80 (was positive): done, |x|=x
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
; In:  EXPH:EXPL = signed value; NEGFLG = current flag (from ABS_TMP)
; Out: EXPH:EXPL = |value|; NEGFLG toggled ($00<->$80) if was negative
; Clobbers: R0, R1
ABS_EXP:
        LODA,R0 EXPH
        ANDI,R0 $80
        RETC,EQ
        LODA,R0 NEGFLG
        EORI,R0 $80                      ; toggle between the $00/$80 sentinels
        STRA,R0 NEGFLG
        ZBRR *VNEG_EXP_BODY

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
        RETC,LT                          ; v1.2: was BCTR,LT CIE_NC (branch-to-return)
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
        RETC,UN

; =============================================================================
;  MULT_LOOP -- Shared repeated-add core for MUL16's multiply and PU16's x10
; EXPH:EXPL += SC0:SC1, TMPH:TMPL times (TMP is consumed/left at 0).
; Leaf: uses R0 only, no calls - safe to invoke with R1/R2/R3 live (PU16
; relies on this to keep R3, the live SW-continuation-stack pointer during
; expression parsing, untouched - no save/restore needed here, unlike the
; old PU16_M10's R3-based BDRR loop).
; Decrement-first with a borrow test, using WC (carry-in) for the high
; byte: TMP-- happens before the "was TMP already 0" question is even
; asked - the decrement's own borrow flag answers it, so there's no
; separate up-front zero test the way MU_LP used to need one.
; In:  EXPH:EXPL = running total; SC0:SC1 = value to add; TMPH:TMPL = count
; Out: EXPH:EXPL += SC0:SC1 * (original TMPH:TMPL)
; Clobbers: R0, TMPH, TMPL, EXPH, EXPL
MULT_LOOP:
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
;  DIV16 -- Signed 16-bit divide: TMPH:TMPL / EXPH:EXPL -> EXPH:EXPL
; Remainder is NOT preserved (nothing in this interpreter reads it - no MOD
; operator) - DV_LP subtracts destructively and stops on the final,
; unsuccessful subtract without backing it out, matching pBASIC's DIV16.
; In:  TMPH:TMPL = dividend; EXPH:EXPL = divisor
; Out: EXPH:EXPL = quotient; TMPH:TMPL left in an undefined state
; Clobbers: R0, NEGFLG, SC0, SC1, TMPH, TMPL
; Error: divisor=0 -> ERR_DIV_ZERO
DO_DIV:
        LODA,R0 EXPL    ; check for zero
        IORA,R0 EXPH        
        BCTA,EQ JERRDIVZER
        ; not zero
DO_MUL:
        ; R1 contains TOK_CHAR offset to decide if div or mul
        STRA,R1 RXSAVE                  ; ;keep it somewhere save
        ; Setup - no NEGFLG pre-clear needed: ABS_TMP (next) writes it
        ; unconditionally regardless of any prior value
        BSTA,UN ABS_TMP                  ; [+1] sets NEGFLG from TMP's sign
        BSTA,UN ABS_EXP                  ; [+1] toggles NEGFLG if EXP was negative
        LODI,R0 SC0-IPH                 ; offset to SCO and 1, SC1 = |EXP| lo
        ZBSR *VEXP16_TO_ET             ; SC0 = |EXP| hi
        ZBSR *VCLR_EXP                  ; clear EXP (accumulator starts at 0)

        ; Check if we are on Mul or Div path
        LODA,R0 RXSAVE                  ; There you are !
        COMI,R0 2                       ; Is it MUL? ('*' now at TOK_CHARS
                                         ; offset 0, +2 pre-increments = 2 -
                                         ; verified via simulator trace, not
                                         ; assumed, after the v0.24 table
                                         ; reorder put */ first for HI tier)
        BCFR,EQ DV_LP                    ; not MUL -> fall to Div Loop
        BSTA,UN MULT_LOOP                ; MUL: shared repeated-add loop
        BCTR,UN MU_DONE
        ; Div Loop - single-pass destructive subtract (WC-chained), no
        ; separate compare-then-subtract: subtract speculatively, then
        ; check the borrow. If it borrowed, TMP was already < divisor,
        ; so this iteration's (wrong) subtraction is simply abandoned -
        ; safe because the remainder is dead (see header).
DV_LP:
        CPSL $08                          ; WC=0 for the low-byte subtract
        LODA,R0 TMPL
        SUBA,R0 SC1
        STRA,R0 TMPL
        PPSL $08                          ; WC=1: high byte takes the borrow
        LODA,R0 TMPH
        SUBA,R0 SC0
        CPSL $08
        TPSL $01                          ; C=1 no borrow (EQ) / C=0 borrow (LT)
        BCTR,LT MU_DONE                   ; TMP < divisor: done
        STRA,R0 TMPH                      ; commit (R0=new TMPH; TPSL/CPSL left it)
        LODI,R0 EXPH-IPH        ; EXP offset from IPH (= 4); assembly-time expression
        ZBSR *VINC_ET         ; quotient++
        BCTR,UN DV_LP

; entry point for MUL16 done too (BCTR,UN MU_DONE after the MULT_LOOP call, above)
MU_DONE:
        BSTA,UN NEG_EXP
        ; No NEGFLG cleanup needed here: both producers (PARSE_S16, ABS_TMP)
        ; write it unconditionally before it is ever read again
        ; Push HI_LOOP so that * / chaining is checked after this result.
        ; "2*3*4" correctly becomes (2*3)*4: after computing 2*3=6, we
        ; resume at HI_LOOP which finds the second *, computes 6*4=24.
        BSTA,UN PUSH_HILOOP
        ZBRR *VPARSER_RET

JERRDIVZER:
        LODI,R0 ERR_DIV_ZERO
        ZBRR *VDO_ERROR 

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
; Clobbers: R0, R1, R2, R3, SC0 (TMP is NOT clobbered - DO_LIST relies on
;   that to walk TMP directly). R2 is NOT saved/restored: none of the 3
;   callers (DO_PRINT, DO_LIST, DO_ERROR) need it back afterward - the
;   digit-printing path always clobbered it anyway, so the old save/
;   restore only ever fired on the zero-value path, for no live caller.
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
        BCFR,EQ NEXT_DIG         ; not the final column -> skip the leading zero

PRINT_IT:
        LODI,R3 1                ; Set leading zero flag
        LODZ,R1                  ; R0 = R1 (destination is always R0)
        ZBSR *VCOUT              ; Print the character
NEXT_DIG:
        ADDI,R2 1                ; Advance to next power of 10
        COMI,R2 5                ; Have we processed all 5 powers?
        BCTR,LT DIGIT_LOOP
        CPSL $08                  ; Clear WC bit
        RETC,UN                  ; Return to caller

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
EXP16_TO_LNUM:
        LODI,R0 LNUMH-IPH       ; LNUMH offset from IPH (= 12)
EXP16_TO_ET:
        PPSL PSW_RS             ; switch to alternate register bank
        STRZ R1                 ; alt-R1 = R0 = destination offset
        LODA,R0 EXPL
        STRA,R0 IPL,R1          ; store lo byte to dest+1
        LODA,R0 EXPH
        BCTR,UN ET_STORE        ; store hi byte, restore bank, return

; Same for TMP
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
;  DO_FOR -- FOR var=start TO limit        (step 1; the body always runs once)
;  Assignment reuses SE_NOTKW (R2 survives the expression). The frame keeps
;  the address of the line AFTER the FOR (SWSTK, which DR_LP has already set
;  to the next line) so NEXT can jump straight back to the body.
; In:  IP -> "var=start TO limit"
; Out: var = start; frame pushed [body hi][body lo][limit hi][limit lo][var]
;      Errors: JSYNERR (no TO), ERR_OOM (frames full)
; Clobbers: R0, R1, R2, EXP, TMP
DO_FOR:
        BSTA,UN SE_NOTKW                 ; var=start; R2 = var's VARS offset
        ZBSR *VWSKIP_PEEK
        COMI,R0 A'T'
        BCFA,EQ JSYNERR                  ; must be TO
        ZBSR *VEATWORD
        ZBSR *VPARSE_EXPR                ; EXP = limit
        LODA,R1 FSP
        COMI,R1 FSTKLIM
        BCFA,LT DRT_UFLOW                ; all frames in use
        LODA,R0 SWSTK
        STRA,R0 FSTK-1,R1+               ; pre-increment store: FSTK[FSP++]
        LODA,R0 SWSTK+1
        STRA,R0 FSTK-1,R1+
        LODA,R0 EXPH
        STRA,R0 FSTK-1,R1+
        LODA,R0 EXPL
        STRA,R0 FSTK-1,R1+
        LODZ,R2
        STRA,R0 FSTK-1,R1+
        STRA,R1 FSP
        RETC,UN

; =============================================================================
;  TABLES 
; Powers of 10 Tables (10000, 1000, 100, 10, 1)
P10_HI:
        db $27, $03, $00, $00, $00
P10_LO:
        db $10, $E8, $64, $0A, $01

BANNER:
        DB CR, LF, "uBASIC 2.7", CR, LF, NUL        

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
        DB "F", <DO_FOR,    >DO_FOR       ; FOR
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
                                          ; unaffected by signedness either way)
EW_ADV:
        ZBSR *VINC_IP 
        BCTR,UN EATWORD
EW_RET:
        RETC,UN

; =============================================================================
;  WSKIP_PEEK -- Skip whitespace, then peek the current char at IP into R0
; Out: R0 = *IPH; CC set by that load (EQ if NUL)
; Clobbers: R0
WSKIP_PEEK:
        ZBSR *VWSKIP
        LODA,R0 *IPH
WSKIPRET:
        RETC,UN

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
; CLR_EXP -- Helper Zeroes EXP
; Clobbers R0
CLR_EXP:
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 EXPL
        RETC,UN

EP_RET:
        ZBSR *VWSKIP  
        ZBSR *VINC_IP                       ; consume ')'
        ; drop through
        db $EC                  ; COMA,R0: consume next 2 bytes
NEG_RET:
        ZBSR *VNEG_EXP_BODY
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


ROMEND: 



;  RAM variables -- sequential RES block 
 
        ORG     4096    ; half a 2650 8kbyte page

; --- Ordered group: offsets from IPH used by INC_ET/DEC_ET/NEG_SHARED ---
; Exactly 16 bytes (IPH..SC1) so IBUF, right after, lands at $1010 - hi
; byte == lo byte, letting GETLINE set IPH:IPL with a single LODI (see
; GETLINE and the now-deleted SET_IP_IBUF).
IPH     RES 1       ; interpreter pointer hi       (INC_ET offset 0)
IPL     RES 1       ; interpreter pointer lo
TMPH    RES 1       ; temp 16-bit hi               (INC_ET offset 2 = TMPH-IPH)
TMPL    RES 1       ; temp 16-bit lo
GOTOH   RES 1       ; pending target hi            (DEC_ET offset 8 = GOTOH-IPH)
GOTOL   RES 1       ; pending target lo
CURH    RES 1       ; current line hi  (error reporting)
CURL    RES 1       ; current line lo

LNUMH   RES 1       ; scratch line number hi       (DEC_ET offset 12 = LNUMH-IPH)
LNUML   RES 1       ; scratch line number lo
EXPH    RES 1       ; expression result hi         (INC_ET offset 4 = EXPH-IPH)
EXPL    RES 1       ; expression result lo
SWSTK   RES 2       ; next-line pointer cache [NLP_H][NLP_L] written by DR_EXEC

SC0     RES 1       ; Scratch byte 0
SC1     RES 1       ; Scratch byte 1

; Buffers - IBUF MUST be exactly 16 bytes from IPH (see above)
IBUF    RES 64      ; Input buffer 64 bytes

; --- Remaining --- (order doesn't matter - none of these sit in the
; offset-from-IPH group above; PEH-IPH is still a valid, just larger,
; assembly-time constant wherever PEH ends up)
PEH     DB <SHOWCASE_END       ; Program end pointer hi
PEL     DB >SHOWCASE_END       ; Program end pointer lo
TEMPRETH RES 1      ; SWRETURN scratch: popped continuation addr hi (was PDEPTH -
                     ; PDEPTH itself removed, see CHANGE HISTORY: paren nesting
                     ; no longer needs a separate SW-tracked depth counter, the
                     ; PUSH_RET/PARSER_RET continuation stack IS the tracking)
TEMPRETL RES 1      ; SWRETURN scratch: popped continuation addr lo
RNDSEED  RES 2      ; pseudo random number

;  --- Flags & Stuff --- 
RUNFLG  RES 1       ; $01=running $00=immediate
RXSAVE  RES 1       ; Save/restore R3 in PARSE_U16, R1 in DO_MUL, and the
                     ; 3rd keyword char (STMT_EXEC -> DO_GO/DO_RU) - all
                     ; three uses have disjoint lifetimes, safe to share
NEGFLG  RES 1       ; Sign flag
BANG    RES 1       ; '!' relop-invert modifier: $00 clear, $FF armed -
GSSTK   RES 8       ; GOSUB return-address stack, 4 levels x [hi][lo]
GSSP    RES 1       ; GOSUB stack offset into GSSTK: 0,2,4,6; 8=full
FSTK    RES 20      ; FOR frames, 4 levels x 5 bytes, pushed in this order:
                    ; [body hi][body lo][limit hi][limit lo][VARS offset]
FSP     RES 1       ; bytes used in FSTK: 0,5,10,15; 20=full

;  SW call stack -- used by PARSE_EXPR / PRINT_S16 only
; R3 = index ($FF=empty, grows up). Each frame = [lo][hi].
; Push: STRA,R0 *SWBASE,R3+ (lo first), STRA,R0 *SWBASE,R3+ (hi).
; Pop:  LODA,R0 *SWBASE,R3- (hi first), LODA,R0 *SWBASE,R3- (lo).
; Also now holds PUSH_RET/SWRETURN continuation addresses (2 bytes each),
; interleaved LIFO with OPS_HIT's own operand pushes - see CHANGE HISTORY.
SWBASE  RES 64      ; SW stack base. Guard fires with ERR_EXPR when full.
SWCAP_LIMIT EQU 62   ; PUSH_RET refuses a new 2-byte push at/past this R3
                     ; value (leaves 2 bytes slack - a push adds 2 bytes and
                     ; R3 is the current top index, so at R3=46 the next push
                     ; would land at 46+47=bytes 46-47, exactly filling the
                     ; 48-byte SWBASE; refusing at 46 catches this cleanly)

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
        DB 0,10,"REM -- Rem does nothing --", $00
        DB 0,20,"PRINT ",$22,"-- uBASIC2650 Showcase --",$22,$00
        DB 0,30,"PRINT ",$22,"--- PRINT / CHR$ / TAB ---",$22,$00         ; 30
        DB 0,40,"PRINT CHR$(65);CHR$(66);CHR$(67)",$00                    ; 40  ABC via CHR$
        DB 0,43,"PRINT",$00                                                ; 43  newline
        DB 0,44,"PRINT TAB(4);",DQ,"Hi",DQ,$00                            ; 44  TAB(4) then "Hi"
        DB 0,50,"PRINT ",$22,"--- ARITHMETIC ---",$22,$00
        DB 0,60,"PRINT ",$22,"3+4=",$22,$3B,"3+4",$3B,$22,"  10-3=",$22,$3B,"10-3",$3B,$22,"  6*7=",$22,$3B,"6*7",$00
        DB 0,70,"PRINT ",$22,"20/4=",$22,$3B,"20/4",$00
        DB 0,80,"PRINT ",$22,"--- COMPARISONS ---",$22,$00
        DB 0,90,"IF 3<9 PRINT ",$22,"3<9 ok",$22,$00
        DB 0,100,"IF 7=7 PRINT ",$22,"7=7 ok",$22,$00
        DB 0,110,"IF 9!=2 PRINT ",$22,"9!=2 ok",$22,$00
        DB 0,120,"IF 9!<4 PRINT ",$22,"9!<4 ok",$22,$00
        DB 0,130,"IF 9>4 PRINT ",$22,"9>4 ok",$22,$00                      ; native '>'
        DB 0,131,"IF 5<2+9 PRINT ",$22,"5<2+9 ok",$22,$00                  ; relop RHS is a whole + - expr
        DB 0,132,"IF 1+2*3>4*5-14 PRINT ",$22,"7>6 ok",$22,$00             ; * / then + - then relop
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
        DB 1,19,"FOR A=1 TO 3",$00                                            ; nested: 3x3 table
        DB 1,24,"FOR B=1 TO 3",$00
        DB 1,29,"PRINT A*B",$3B,$22," ",$22,$3B,$00
        DB 1,34,"NEXT B",$00
        DB 1,35,"PRINT",$00
        DB 1,36,"NEXT A",$00
        DB 1,39,"GOTO 300",$00
        DB 1,44,"PRINT ",$22,"--- MANDELBROT ---",$22,$00                  ; 300
        DB 1,49,"M=16",$00                                                 ; 305 iteration limit (a variable FOR limit)
        DB 1,54,"FOR R=0 TO 20",$00                                        ; 310 21 rows       (FOR level 1)
        DB 1,64,"LET D=R*6-64",$00                                         ; 320 imaginary part (was I=-64 step 6)
        DB 1,84,"FOR Q=0 TO 43",$00                                        ; 340 44 columns    (FOR level 2)
        DB 1,94,"LET C=Q*4-144",$00                                        ; 350 real part      (was C=-144 step 4)
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
        DB 2,88,"FOR N=1 TO M",$00                                         ; 600 escape-count subroutine, a FOR loop
        DB 2,98,"IF E>0 GOTO 640",$00                                      ; 610   (FOR level 3, inside a GOSUB, inside
        DB 2,108,"T=A*A/64-B*B/64+C",$00                                   ; 620    levels 1-2). Escaped points skip the
        DB 2,113,"B=2*A*B/64+D",$00                                        ; 625    maths by GOTOing the NEXT: leaving
        DB 2,114,"A=T",$00                                                 ; 626    via NEXT (not out of the loop) leaves
        DB 2,118,"IF 256<A*A/64+B*B/64 THEN IF E=0 THEN E=N",$00           ; 630    no frame behind. No parentheses:
        DB 2,128,"NEXT N",$00                                              ; 640    the relop takes a whole + - * / RHS
        DB 2,138,"RETURN",$00                                              ; 650
SHOWCASE_END:

        END
