; uBASIC2650.asm       Tiny BASIC interpreter for Signetics 2650
; Version: v4.3
; By Vincent Crabtree, 2026.  MIT License
; Date:    2026-06-26
;
; Target:  Standalone (no PIPBUG ROM). Code ORG 0. I/O routines embedded.
;          Single 8192-byte address space (2650 bits 15:13 always 0).
;          CHIN=$0286  COUT=$02B4 (check for changes with edits).
;
; Assembler: asm2650.c v1.13+  Simulator: pipbug_wrap.c v2.1
; Build:
;   gcc -Wall -O2 -o asm2650 asm2650.c
;   gcc -Wall -O2 -DGAMER -o pipbug_wrap pipbug_wrap.c
;   ./asm2650 uBASIC2650.asm uBASIC2650.hex
;   ./pipbug_wrap --entry 0 --chin 0x286 --cout 0x2B4 --crlf 0x7fff uBASIC2650.hex
;
; IMPORTANT: --crlf must be harmless NOT old PIPBUG $008A.
;       Wrong --crlf fires mid-instruction, corrupting RAS and breaking LIST/RUN
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
;   FUNCATOM-01: FUNC_TAB names (ABS/NEG/AND/OR/XOR/NOT/RND/PEEK/USR) are only
;       recognized as the LEADING token of a fresh PARSE_EXPR call (right
;       after a statement keyword or an open paren). EAM_ATOM does not
;       re-check FUNC_TAB for an atom following an operator, so it falls
;       through to PARSE_FACTOR and misreads the function name as a bare
;       variable: "PRINT 10+ABS(A)" silently parses as "10+A" (works fine
;       as "PRINT ABS(A)+10" - leading position only). Confirmed present in
;       v4.2 already, not a v4.3 regression. Fix belongs in EAM_ATOM (teach
;       it to FUNC_TAB-match before falling to PARSE_FACTOR); deferred -
;       bigger structural change than a golf pass, needs its own RAS/byte
;       budget review.
;
;        CHANGE HISTORY
;   V4.3  2026-06-25 - 3672 bytes to ROMEND
;         PAREN-NEST-01 FIXED: EAM_PLUS/EAM_MINUS/EAM_MUL/EAM_DIV/EAM_MOD now
;         push the left operand onto SWBASE instead of a flat SAVEH/SAVEL or
;         E1SAVH/E1SAVL cell, so nested same-precedence ops (e.g. "2*(3*4)")
;         can no longer clobber a pending operand. ADD16_SAVE_EXP pops
;         SAVEH:SAVEL from SWBASE just before use; new shared POP_SAVE_TO_TMP
;         does the same for EAM_MUL/DIV/MOD into TMPH:TMPL. E1SAVH/E1SAVL
;         RAM cells and the EXP16_TO_E1SAV/EXP16_TO_SAVE vectors+chain
;         entries removed as dead code (net -1 zero-page vector slot vs v4.2).
;         BUG-RND-01 FIXED: DO_RND_FUNC now shuffles the LFSR 8x per call
;         (was 1x), fixing correlated draws in tight RND() loops.
;         RAS_DEPTH / PE_RAS_LIMIT EQUs added; PARSE_EXPR guard literal '5'
;         replaced with PE_RAS_LIMIT.
;         IP_TO_TMP helper added (new VIP_TO_TMP vector), consolidating 3
;         inline IPH:IPL->TMPH:TMPL copies (DLS_NL, STORE_LINE x2).
;         Added AND(a,b)/OR(a,b)/XOR(a,b)/NOT(a) bitwise functions via
;         shared PARSE_2ARGS helper and new FUNC_TAB entries.
;         Showcase (PROG default) rewritten: wider Mandelbrot finale, now
;         exercises ABS/NEG/PEEK/USR/RND(n)/AND/OR/XOR/NOT/LIST range.
;   V4.2  2026-06-24 - 3485 bytes to ROMEND
;         RDLINE rewritten to use R3 as an IBUF offset (pre-increment/$FF-
;         empty convention, matching SWBASE) instead of full IPH:IPL pointer
;         chasing for bounds-check and backspace. IPH:IPL no longer valid
;         during/after RDLINE; both callers already re-set it immediately.
;         EAM_PLUS/EAM_MINUS WC add/sub tails merged into shared
;         ADD16_SAVE_EXP; EAM_MINUS now negates EXP via existing
;         NEG_EXP_BODY and falls into the same add path as EAM_PLUS.
;         Added LIST [start,end] optional filter.
;         Replaced replaced with PARSE_S16 for RAS.
;
;   V4.1  2026-06-23 - 3581 interpreter bytes
;         Refactored STMT_EXEC to extract MATCH_KW for general use with no-match vector.
;         Function table match added to PARSE_EXPR as Highest precidence.
;         Added ABS, NEG, PEEK, USR, RND(range) functions. Refactor DO_RETURN for size. 
;
;   V4.0  2026-06-22 - 3486 ROM bytes
;         BUG-SL-02: STORE_LINE / SL_SHLOOP lo-byte termination comparison.
;         SUBA,R0 EXPL sets CC by signed subtraction; when LNUML has bit7 set
;         and EXPL is small (e.g. LNUML=$FF, EXPL=$04), result $FB is negative
;         (CC=LT), causing the shift loop to exit up to 131 bytes early.
;         Implemented DF_PUSH loop-based frame write after VARS reorg.
;         Added POKE addr,val, refactor STMT_EXEC for size.
; 
;   V3.9  2026-06-16 
;         BUG-SL-01 (complete fix): STORE_LINE / SL_SHLOOP GOTOH carry detection.
;         v3.7b attempted SUBA,R0 PEL to detect carry after ADDA,R0 SC1, but
;         two problems remained: (1) LODA,R0 PEH between SUBA and BCFR,LT
;         clobbered CC; (2) even without the LODA, SUBA,R0 PEL gives wrong
;         result when carry has occurred -- (PEL+SC1)_lo - PEL = SC1 (positive,
;         CC=GT) even though carry fired, because the wrapped sum minus the
;         addend gives SC1 not a negative value.
;         Correct test: SUBA,R0 SC1 after STRA,R0 GOTOL. sum_lo - SC1 = PEL
;         (positive/zero, no carry) or negative/LT (sum wrapped below SC1,
;         carry occurred). Branch BCFR,LT to no-carry path before any LODA.
;         Verified by trace: GOTOH correctly set to PEH+1 for PEL=$F8, SC1=$13.
;         TSL inline EXP->LNUM: TRY_STORE_LINE had one inline EXPH:EXPL->LNUMH:LNUML
;         copy not converted to EXP16_TO_LNUM helper. Now uses BSTA,UN EXP16_TO_LNUM.
;
;   V3.7b 2026-06-15  3543 rom bytes
;         BUG-DEC-01: DEC_ET borrow detection used BCFR,LT (sign test)
;         not carry. Any lo-byte in $80-$FF range sets CC=LT after SUBI,R0 1
;         even without borrow, causing spurious hi-byte decrement.
;         Fix: TPSL $01 / BCTR,EQ ET_RET (C=1=no borrow -> CC=EQ -> skip hi--).
;         Root cause: latent since v3.6c; exposed when PROG moved from $1703
;         to $10E0, putting lo-bytes in $E0+ range on first decrement.
;         BUG-DL-01: DELETE_LINE / DL2_LP early exit.
;         TPSL $01 / BCTR,EQ DL2_DONE exited when TMPL > PEL
;         (TPSL maps GT->EQ same as EQ, causing early loop exit).
;         Fix: BCFR,LT DL2_DONE (branch if TMPL >= PEL, i.e. no borrow).
;         BUG-SL-01 (partial): STORE_LINE / SL_SHLOOP GOTOH carry detection.
;         STRA,R0 GOTOL between ADDA,R0 SC1 and TPSL $01 described as clobbering
;         carry; fix attempted via SUBA,R0 PEL but LODA,R0 PEH before branch
;         still clobbered CC. See V3.9 for complete fix.
;
;   V3.7  2026-06-14  Interpreter: 3567 bytes
;         Page-zero vector table + Zxxx size optimisation. 
;         DO_NEW memory clear. Refactor PRINT_S16 intro for size.
;         SWSTK RES 1->2: fixed SWSTK+1/RELOP aliasing bug (v36d fix).
;
;   V3.6G 2026-06-12  Interpreter: 3700 bytes
;         Code ORG now 0x0, added CHIN/COUT routines from Pipbug M20 App note,
;           hand-optimized placement to $286/$2B4 for PIPBUG 1 compatibility.
;         Refactor DO_ERROR to use DO_END.  Refactor DO_FOR for size.
;         Helpers: FI_ADV, SETUP_MULDIV, PARSE_VAR_SAVE.
;         VARS moved to $1000+, RAMTOP $1FFF.
;
;   V3.6c  2026-06-10  - 3748 Interpreter bytes
;         Helpers - SET_IP_IBUF, EXP16_TO_GOTO, EXP16_TO_LNUM, ERRFLG eliminated.
;         DEC_ET family: mirrors INC_ET, Refactor DL2_SCAN to use 2x INC_TMP.
;         FREE memory keyword added.
;
;   V3.5  2026-06-09  Merged v3.3+v3.4 FOR/NEXT into v3.2 optimised baseline.
;         DO_FOR:  v3.3 parse GETCI_UC for '=', LNUMH/LNUML for limit,
;         DO_NEXT: v3.4 VARS access (direct VARS,R1 indexed -- VARS_FP dropped).
;         DO_RETURN: v3.3 GOTOFLG=$03 (direct NLP). v3.4 regressed to $01.
;         DR_EXEC:   v3.3 three-way GOTOFLG dispatch (inline COMI $03).
;         STORE_LINE: v3.4 BUG-LE fix (BCTR,LT SL_NOSHIFT both paths).
;         PARSE_EXPR: v3.3 body + v3.2 + RAS guard .
;
;   V3.4  2026-06-09  FOR/NEXT variant 2 (v3.3 parallel branch).
;         BUG-SE-01, BUG-DN-01..04, BUG-LE fixes.
;         DF_PUSH R1-indexed frame write (smaller than INC_TMP chain).
;         DO_NEXT VARS,R1 indexed (drops VARS_FP subroutine).
;
;   V3.3  2026-06-07  FOR/NEXT and GOSUB/RETURN complete. ROMEND=$13AC.
;         FORBASE=$1670: 4-level FOR stack, 7 bytes/frame.
;         FOR_FP inlined into DO_NEXT (OPT-FP1).
;         GOTOFLG=$03 = DR_FORNLP direct address branch.
;         BUG-RET-01 fixed (GOTOFLG=$03 for RETURN).
;
;   V3.2  2026-06-04  GOSUB/RETURN added.  ROMEND=$0D5E (2590 interp bytes)
;         KW_TAB 3-char matching [c1][c2][c3][hi][lo], stride 5.
;         GSBASE=$1660 8-frame GOSUB stack. SWSP=$162D.
;
;   V3.1  2026-06-04  ROMEND=$0CF2 (2482 interp bytes)
;         Fix: PARSE_U16 multiply-by-10 clobbered R3 (SW stack ptr).
;
;   V3.0  2026-06-04  SW-stack recursive descent PARSE_EXPR.
;   V2.8  2026-05-30  3576 total bytes. Code size refactor.
;   V2.7  2026-05-29  TAB() in PRINT. OPT-15 sign-handling subroutines.
;   V2.6  2026-05-23  CHR$(). Bug fixes. OPT-2..10.
;   V2.5  2026-05-22  BUG-FL-02, BUG-CHR-01.
;   V2.4  2026-05-19  Showcase + Mandelbrot appended.
;   V2.3  BUG-FL-01/RAS-01/MAND-01/FI-01/DIV-ZCHK-01 fixed.

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
ERR_RET         EQU '5'         ; RETURN without GOSUB (GOSUB stack underflow)
ERR_FOR         EQU '6'         ; Too many nested FORs (FORBASE stack overflow)
ERR_NXT         EQU '7'         ; NEXT without FOR (FORBASE stack underflow)
ERR_NEST        EQU '8'         ; Expression nesting too deep (RAS guard, v3.2 had '5')

; RAS (hardware Return Address Stack) Defines
RAS_DEPTH       EQU 8           ; 2650 HW RAS depth (SPSU field is 3 bits, 0-7)
PE_RAS_LIMIT    EQU 5           ; PARSE_EXPR entry guard threshold; margin of
                                 ; RAS_DEPTH-3 covers PARSE_FACTOR+PARSE_S16+INC_IP

; PSW Defines
PSW_RS          EQU     $10
PSW_WC          EQU     $08             ; WC (With Carry) bit in PSL (bit 3)
PSW_FLAG        EQU     $40

; System Defines
PROGLIM         EQU $1FFF   ; top of program store (numeric constant, not address)
;  GOSUB stack (v3.2) -- managed by SWSP
; Frame = [lo][hi] of NLP. SWSP=$FF=empty. 2 bytes/frame, 8 frames.
GSSTKLIM        EQU $0F    ; max SWSP before overflow (numeric constant, not address)
;  FOR/NEXT stack (v3.3) -- managed by FORSP
; Frame (7 bytes): [var][limH][limL][stpH][stpL][nlpH][nlpL]
;   var=letter A-Z, lim=signed limit, stp=signed step, nlp=loop-back address.
; FORSP=$FF=empty. Offsets: 0/7/14/21 for frames 1-4. 4 frames = 28 bytes.
; Overflow: FORSP >= FORSTKLIM before push -> ERR_FOR.
FORSTKLIM       EQU $15   ; max FORSP before overflow (numeric constant, not address)

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
        DW PARSE_EXPR           ; 19 sites
VGETCI_UC:
        DW GETCI_UC             ; 7 sites
VEATWORD:
        DW EATWORD              ; 7 sites
VSET_IP_IBUF:
        DW SET_IP_IBUF          ; 4 sites
VPRT_SPACE:
        DW PRT_SPACE            ; 5 sites
VINC_EXP:
        DW INC_EXP              ; 5 sites
VCLR_EXP:
        DW CLR_EXP              ; 5 sites
VEAM_ATOM:
        DW EAM_ATOM             ; 8 sites
VEAM_HI:
        DW EAM_HI               ; 6 sites
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
                                 ; directly from ARGAH:ARGAL post-v4.4)
VTMP_TO_EXP16:
        DW TMP_TO_EXP16         ; 4 sites
VDEC_IP:
        DW DEC_IP               ; 4 uses (2 are a double-up)
VRND_SHUFFLE:
        DW RND_SHUFFLE
VSET_TMP_PROG:
        DW SET_TMP_PROG
VIP_TO_TMP:
        DW IP_TO_TMP            ; 3 sites (v4.3 TMP-copy consolidation)
MAIN:
        ; Pre-load SHOWCASE_END as program so RUN executes the showcase.
        ; Delete for ROM 
        LODI,R0 <SHOWCASE_END
        STRA,R0 PEH
        LODI,R0 >SHOWCASE_END
        STRA,R0 PEL
       
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
        CPSL PSW_RS + 7             ; ensure primary reg bank, Clear stack so SP=0
        LODI,R0 '>'                    ; print prompt only used here
        ZBSR *VCOUT  
        ZBSR *VPRT_SPACE  
        BSTA,UN RDLINE
        ZBSR *VSET_IP_IBUF                ; IPH:IPL = IBUF
        BSTA,UN TRY_STORE_LINE           ; CC=GT: line stored/deleted; CC=EQ: not a line
        BSTR,EQ STMT_EXEC               ; If CC=EQ (not a line), execute
        BCTR,UN REPL

; =============================================================================
;  STMT_EXEC -- Decode and dispatch one BASIC statement from IP.
; In:  IPH:IPL -> first char of statement (after any leading whitespace)
; RAS depth: 1 from REPL, 3 from DO_IF(THEN body).
; Worst inner depth from here: +4 (DO_xxx->PARSE_EXPR->PARSE_FACTOR->UPCASE)
STMT_EXEC:
        ; Look at statement KW_TAB with TMPH:TMPL as pointer
        LODI,R0 <KW_TAB
        STRA,R0 TMPH
        LODI,R0 >KW_TAB
        STRA,R0 TMPL
        ; drop through

; =============================================================================
; MATCH_KW -- Match and Jump to handler based on KW
; KW_TAB format: [c1][c2][c3][hi][lo], stride 5.
;   c3=A' ' (space) = wildcard (short keywords: IF).
;   c3 is peeked from *IPH uppercase without consuming; EATWORD cleans up.
; In:  IPH:IPL -> first char of statement (after any leading whitespace)
;      SWBASE -> Address of No_Match failure 
; Out: handler called; IP advanced past statement
; Clobbers: R0, SC0, SC1, TMPH, TMPL, EXPH, EXPL, GOTOH, GOTOL
MATCH_KW:
        ZBSR *VWSKIP                      ; [+1]
        ZBSR *VGETCI_UC  
        STRA,R0 SC0                       ; char1 uppercase
        ZBSR *VGETCI_UC  
        STRA,R0 SC1                       ; char2 uppercase
SE_SCAN:
        LODA,R0 *TMPH                     ; c1
        BCTR,EQ SE_NOMATCH                ; end of table, jump to nomatch
        SUBA,R0 SC0
        BCFR,EQ SE_FAIL_5                 ; Mismatch? Drop to +5 cascade

        ZBSR *VINC_TMP                    ; point to c2
        LODA,R0 *TMPH
        SUBA,R0 SC1
        BCFR,EQ SE_FAIL_4                 ; Mismatch? Drop to +4 cascade

        ZBSR *VINC_TMP                    ; point to c3
        LODA,R0 *TMPH
        COMI,R0 A' '
        BCTR,EQ SE_MATCH                  ; Wildcard: accept
        STRA,R0 EXPL                      ; save table-c3 in EXPL
        LODA,R0 *IPH                      ; peek c3
        BSTA,UN UPCASE                    ; [+1]
        SUBA,R0 EXPL
        BCFR,EQ SE_FAIL_3                 ; Mismatch? Drop to +3 cascade

SE_MATCH:
        ZBSR *VEATWORD                    ; [+1] consume remaining
SE_NOMATCH:
        ZBSR *VINC_TMP  
        LODA,R0 *TMPH
        STRA,R0 GOTOH                     ; store handler hi directly
        ZBSR *VINC_TMP  
        LODA,R0 *TMPH
        STRA,R0 GOTOL                     ; store handler lo directly
        BCTA,UN *GOTOH                    ; indirect jump

; --- Cascading Advancement ---
SE_FAIL_5:
        ZBSR *VINC_TMP                    ; 
SE_FAIL_4:
        ZBSR *VINC_TMP                    ; 
SE_FAIL_3:
        ZBSR *VINC_TMP                    ; 
        ZBSR *VINC_TMP                    ; 
        ZBSR *VINC_TMP                    ; 
        BCTA,UN SE_SCAN                   ; Loop back

KWSYNERR:
        ZBRR *VJSYNERR 
SE_NOTKW:
        ; Keyword table exhausted. Check for bare variable assignment:
        ;   SC0 = first char (A-Z), SC1 = second char ('=').
        ; If SC0 is A-Z and SC1 is '=': jump straight to DL_EX.
        LODA,R0 SC0
        COMI,R0 A'A'
        BCTR,LT KWSYNERR ; - surrogate for JSYNERR
        COMI,R0 A'Z'+1
        BCTR,GT KWSYNERR ; surrogate for JSYNERR
        LODA,R0 SC1
        COMI,R0 A'='
        BCFR,EQ KWSYNERR ; surrogate for JSYNERR

; Bare assignment no LET
        LODA,R0 SC0
        STRZ,R2                          ; save letter in R2 (survives PARSE_EXPR)
        BCTA,UN DL_EX                    ; IP already past '=', expression follows

; =============================================================================
;  DO_NEW -- Clear program store
; Syntax: NEW
; In:  nothing
; Out: PEH:PEL = VARS; program store ($10E0-$1FFF) zeroed; falls through to DO_END
; Clobbers: R0, R1, IPH, IPL, SWSP, FORSP, GOTOFLG, RUNFLG
DO_NEW:
        ; Zero VARS + program store using IPH:IPL as write pointer.
        ; R1 = 0 
        ; Stop when IPH reaches $20 (i.e. address wrapped past $1FFF).
        LODI,R0 <VARS
        STRA,R0 IPH
        LODI,R0 >VARS
        STRA,R0 IPL
        EORZ,R1                          ; Zero, is CR better?
DN_CLR:
        STRA,R1 *IPH                     ; zero byte at IPH:IPL
        ZBSR *VINC_IP
        LODA,R0 IPH                      ; past $1FFF? $2000 hi = $20
        COMI,R0 <PROGLIM+1               ; Change for other mem configs
        BCTR,LT DN_CLR                   ; no: continue 
 
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
; Out: SWSP=$FF, FORSP=$FF, GOTOFLG=0, RUNFLG=0
; Clobbers: R0
DO_END:
        BSTA,UN DN_POP_EMPTY
        STRA,R0 SWSP                     ; clear GOSUB stack
        EORZ,R0
        STRA,R0 GOTOFLG
        ZBRR *VCLR_RUNFLG               ; tail call

; =============================================================================
;  DO_PRINT / PRTSTR -- Print statement and NUL-terminated string helper
; Syntax: PRINT [item {; item}]
;   item = "string" | expr | TAB(n) | CHR$(n)
;   Trailing ; suppresses newline.
; In:  IP -> first char after PRINT keyword
; Out: text written to COUT; IP advanced past statement
; Clobbers: R0, R1, EXPH, EXPL, TMPH, TMPL, NEGFLG, LNUMH, LNUML, SC0, SC1
DO_PRINT:
        ZBSR *VWSKIP  
        LODA,R0 *IPH
        BCTA,EQ DP_NL

DP_ITEM:
        ZBSR *VWSKIP  
        LODA,R0 *IPH
        COMI,R0 DQ
        BCTA,EQ DP_STRING
        COMI,R0 'C'
        BCFR,EQ DP_TAB          ; not 'C': forward to DP_TAB

        ZBSR *VINC_IP  
        LODA,R0 *IPH
        COMI,R0 'H'
        BCTR,EQ DP_CHAR

DP_BACKUP:
        ZBSR *VDEC_IP
DP_EXPR:
        ZBSR *VPARSE_EXPR  
        BSTA,UN PRINT_S16
        BCTR,UN DP_SEP

DP_CHAR:
        ZBSR *VEATWORD  
        ZBSR *VPARSE_EXPR  
        LODA,R0 EXPL
        ZBSR *VCOUT  
        BCTR,UN DP_SEP

DP_TAB:
        COMI,R0 'T'
        BCFR,EQ DP_EXPR         ; not 'T': fall back to DP_EXPR
        ZBSR *VINC_IP  
        LODA,R0 *IPH
        COMI,R0 'A'
        BCFR,EQ DP_BACKUP
        ZBSR *VEATWORD  
        ZBSR *VPARSE_EXPR  
        LODA,R1 EXPL
        BCTR,EQ DP_SEP          ; TAB(0): skip
TAB_LOOP:
        ZBSR *VPRT_SPACE  
        BDRR,R1 TAB_LOOP
        ; fall through to DP_SEP

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
        BCTA,UN DP_ITEM

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
;  DO_LET -- Variable assignment
; Syntax: LET V = expr   (also handles bare "V = expr" via SE_BAREASS)
; In:  IP -> variable letter
; Out: VARS[V] = EXPH:EXPL
; Clobbers: R0, R2, SC0, SC1, EXPH, EXPL, TMPH, TMPL, R1
; Note: DO_INPUT jumps to DL_STORE with variable letter in SC0 and R2.
;       DO_FOR also calls DL_STORE (via BSTA) with same convention.
DO_LET:
        BSTA,UN PARSE_VAR_SAVE
        ZBSR *VWSKIP                      ; [+1]
        LODA,R0 *IPH
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
        BSTA,UN EXP16_TO_GOTO             ; GOTOH:GOTOL = EXPH:EXPL
        LODI,R0 1
        STRA,R0 GOTOFLG
        LODA,R0 RUNFLG                   ; OPT-10
        RETC,GT                          ; return if running
        ZBRR *VCLR_RUNFLG 

; =============================================================================
;  DO_IF -- Conditional execution
; Syntax: IF expr relop expr THEN stmt
; In:  IP -> first char after IF keyword
; Out: executes stmt if condition true; otherwise sequential return
; Clobbers: R0, R1, EXPH, EXPL, LNUMH, LNUML, SC0, SC1, RELOP
; RAS: entry+1(PE)+1(PR)+1(PE)+1(SE) = entry+4. Max depth 7: ok.
DO_IF:
        ZBSR *VPARSE_EXPR                 ; [+1]
        ZBSR *VEXP16_TO_LNUM             ; LNUMH:LNUML = EXPH:EXPL (save left operand)
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
        ZBSR *VWSKIP                      ; [+1]
        ZBSR *VGETCI_UC                   ; [+1] must be 'T'
        COMI,R0 A'T'
        BCTR,EQ DIF_TH2
        BCTR,UN LSYNERR ; SYNERR
DIF_TH2:
        ZBSR *VGETCI_UC                   ; [+1] must be 'H'
        COMI,R0 A'H'
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
        BCTA,UN STMT_EXEC                ; [+1] execute THEN body

; =============================================================================
;  DO_RETURN -- Return from subroutine
; Syntax: RETURN
; In:  SWSP = GOSUB stack pointer; GSBASE[SWSP]=lo, GSBASE[SWSP+1]=hi of NLP.
; Out: GOTOH:GOTOL = popped NLP; GOTOFLG=$03 (direct NLP, no FIND_LINE).
; Clobbers: R0, R1, GOTOH, GOTOL, GOTOFLG, SWSP.
; Error: SWSP=$FF (underflow) -> ERR_RET.
; BUG-RET-01 FIX: GOTOFLG must be $03 (direct address) not $01 (FIND_LINE).
;   GSBASE stores program-store addresses, not BASIC line numbers.
DO_RETURN:
        LODA,R1 SWSP            ; R1 = original SWSP                    
        COMI,R1 $FF             ; Is the stack empty?                   
        BCTA,EQ DRT_UNDERFLOW   ; If underflow, bail out                

        ; Update SWSP in RAM First 
        LODZ R1                 ; R0 = copy of original SWSP            
        BCFR,EQ DRT_SUB         ; If SWSP != 0, go subtract 2           
        LODI,R0 $FF             ; If SWSP == 0, roll over to empty      
        BCTR,UN DRT_WRITE       ; Jump to the unified store             
DRT_SUB:
        SUBI,R0 2               ; Decrement stack frame pointer         
DRT_WRITE:
        STRA,R0 SWSP            ; Store updated SWSP back to RAM        

        ; Fetch 16-Bit Address 
        LODA,R0 GSBASE,R1       ; Load Lo-Byte (R1 is now SWSP+1)       
        STRA,R0 GOTOL           ; Store Lo-Byte                         
        LODA,R0 GSBASE,R1+      ; Load Hi-Byte & auto-increment R1      
        STRA,R0 GOTOH           ; Store Hi-Byte                         
; ---  DO_NEXT Entry Point Preserved  ---
DRT_GO:
        LODI,R0 3               ; GOTOFLG=$03 direct NLP                
        STRA,R0 GOTOFLG         ;      [3B]
        RETC,UN                 ; Return                                [1B]

; =============================================================================
DO_PEEK_FUNC:
        ZBSR *VPARSE_EXPR
        LODA,R0 *EXPH
        BCTR,UN EXPH_Z                   ; clear top byte

; =============================================================================
; Calling function retval in R0
DO_USR_FUNC:
        ZBSR *VPARSE_EXPR
        BSTA,UN *EXPH   
EXPH_Z:        
        STRA,R0 EXPL
        EORZ,R0          ; clear top byte
        STRA,R0 EXPH
        RETC,UN

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
; Character IO
; 110 Baud teletype from PIPBUG V1 as per Signetics M20 application note
; CHIN must be at $286, COUt must be at $2B4 for Pipbug 1 compatability
        ORG $286
CHIN:
        ZBSR *VRND_SHUFFLE
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
; RND_SHUFFLE  Advance 16-bit Galois LFSR (Little-Endian)
; =============================================================================
RND_SHUFFLE:
        LODA,R0 RNDSEED         ; Load seed high byte (0xAC)    
        LODA,R1 RNDSEED+1       ; Load seed low byte  (0xE1)    
        CPSL    1               ; Clear Carry for logical shift 
        RRR,R0                  ; Shift high byte right         
        RRR,R1                  ; Shift low byte right          
        TPSL    1               ; Test Carry bit (PSL bit 0)    
        BCTR,EQ RND_SKIP        ; If Carry clear, skip XOR taps 
        EORI,R0 $AC             ; XOR high byte with 0xA0       
        EORI,R1 $E1             ; XOR low byte with 0x01        
RND_SKIP:
        STRA,R0 RNDSEED+1      ; Save seed Hi  byte           
        STRA,R1 RNDSEED        ; Save seed Lo byte            
        RETC,UN                ; Return     

; =============================================================================
; DO_RND_FUNC  RND(n) -> pseudo-random value in [0,n)
; BUG-RND-01 fix: a single LFSR shift/call left consecutive RND() draws in a
; tight loop highly correlated (only 1 bit of mixing between draws). Now
; shuffles a full byte (8 shifts) per call; CHIN's incidental shuffle on
; keypress still adds independent async entropy on top.
; v4.3.4: relocated here (after RND_SHUFFLE, past CHIN/COUT) - the original
; 8x-unroll added 14 bytes that pushed DO_PEEK_FUNC/DO_USR_FUNC/EXPH_Z past
; the hard ORG $286 boundary, and asm2650 does not flag an ORG target behind
; the current address, so it silently let CHIN's bytes overwrite the tail of
; that block. Confirmed via simulator: PEEK/POKE both broke as a result.
; v4.4: loop via R2+BDRR instead of unrolled (R2 is untouched by RND_SHUFFLE
; and not live here - this is a FUNC_TAB-dispatched call, see PARSE_2ARGS
; header note on register lifetimes) - 6 bytes vs the unroll's 16.
; =============================================================================
DO_RND_FUNC:
        ZBSR *VPARSE_EXPR       ; get range in EXP 
        LODI,R2 8               ; shuffle a full byte's worth of taps
RNDF_MIX:
        ZBSR *VRND_SHUFFLE
        BDRR,R2 RNDF_MIX
        LODA,R0 RNDSEED         ; Copy rnd into TMP
        STRA,R0 TMPH
        LODA,R0 RNDSEED+1       ; 
        STRA,R0 TMPL
        BSTA,UN DIV16          ; Divivide
        ZBRR *VTMP_TO_EXP16            ; Get remainder

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
        ZBSR *VSET_IP_IBUF                ; IPH:IPL = IBUF
        BSTA,UN PARSE_S16                ; [+1]
        BCTA,UN DL_STORE

; =============================================================================
;  DO_GOSUB -- Subroutine call
; Syntax: GOSUB <line>
; In:  IP -> line number; SWSTK[0:1] = NLP from DR_EXEC; SWSP = stack ptr.
; Out: GOTOH:GOTOL = target line; GOTOFLG=$02; NLP pushed onto GSBASE.
; Clobbers: R0, R1, EXPH, EXPL, GOTOH, GOTOL, GOTOFLG, SWSP
; Stack: GSBASE[SWSP]=lo, GSBASE[SWSP+1]=hi. SWSP=$FF=empty.
DO_GOSUB:
        ZBSR *VWSKIP                      ; [+1]
        ZBSR *VPARSE_EXPR                 ; [+1] target line -> EXPH:EXPL
        ; overflow check
        LODA,R0 SWSP
        COMI,R0 $FF
        BCTR,EQ DGS_FIRST
        COMI,R0 GSSTKLIM
        BCTR,LT DGS_NEXT
        LODI,R0 ERR_OOM
        ZBRR *VDO_ERROR 
DGS_FIRST:
        EORZ,R0
        STRA,R0 SWSP
        BCTR,UN DGS_STORE
DGS_NEXT:
        LODA,R0 SWSP
        ADDI,R0 2
        STRA,R0 SWSP
DGS_STORE:
        LODA,R0 SWSTK+1                  ; NLP lo byte
        LODA,R1 SWSP
        STRA,R0 GSBASE,R1                ; GSBASE[SWSP] = lo
        LODA,R0 SWSTK                    ; NLP hi byte
        STRA,R0 GSBASE,R1+                ; GSBASE[SWSP+1] = hi
        BSTA,UN EXP16_TO_GOTO             ; GOTOH:GOTOL = EXPH:EXPL (target line)
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
        ZBSR *VWSKIP                      ; [+1] skip whitespace before var
        ZBSR *VGETCI_UC                   ; [+1] R0 = uppercase var letter
        STRA,R0 FORVAR
        STRZ,R2                          ; R2 = var letter (survives PARSE_EXPR)

        ; --- stack overflow & initialization check ---
        LODA,R0 FORSP
        COMI,R0 $FF
        BCFR,EQ DF_NOTFIRST              ; FIXED: Branch if False on Equal (Not Equal)
        LODI,R0 $F9                      ; Math Hack: Load -7 ($F9) if empty.
                                         ; -7 + 7 will result in 0 later
DF_NOTFIRST:
        COMI,R0 FORSTKLIM
        BCFA,LT JFORERR                  ; If NOT Less-Than limit (>=), fail!
        ADDI,R0 7                        ; Normal frame adds 7 / Empty frame ends up at 0
        STRA,R0 FORSP

        ; --- skip '=' then parse start value -> EXPH:EXPL ---
        ; OPT-F2: GETCI_UC skips whitespace + reads '=' in one call.
        ZBSR *VGETCI_UC                   ; [+1] skip whitespace + consume '='
        ZBSR *VPARSE_EXPR                 ; [+1] start value -> EXPH:EXPL
        BSTA,UN DL_STORE                 ; [+1] VARS[R2] = EXPH:EXPL
        ; --- consume "TO" keyword ---
        ZBSR *VWSKIP                      ; [+1]
        ZBSR *VEATWORD                    ; [+1]
        ; --- parse limit -> LNUMH:LNUML ---
        ZBSR *VPARSE_EXPR                 ; [+1]
        ZBSR *VEXP16_TO_LNUM             ; LNUMH:LNUML = EXPH:EXPL (limit)
        ; --- check for STEP keyword ---
        ; OPT-F3: GETCI_UC peeks first non-space char.
        ;   If 'S': consume rest of "STEP" with EATWORD, then parse step.
        ;   Else: un-consume, use default step = +1.
        ZBSR *VGETCI_UC                   ; [+1] R0 = first non-space char (consumed)
        COMI,R0 A'S'
        BCTR,EQ DF_STEP
        ; not 'S': put char back, use step=+1
        ZBSR *VDEC_IP
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
; Out: If looping: GOTOH:GOTOL=nlp, GOTOFLG=$03 (direct addr branch).
;      If done: FORSP-=7 (or $FF if was 0), sequential return.
; Clobbers: R0, R1, EXPH, EXPL, LNUMH, LNUML, SC0, GOTOH, GOTOL, GOTOFLG.
; Errors: FORSP=$FF -> ERR_NXT.
; Variable name after NEXT consumed but not checked against frame (smallest code).
DO_NEXT:
        LODA,R0 FORSP
        COMI,R0 $FF
        BCTA,EQ JERR_NXT                    ; not $FF: Error
        ZBSR *VWSKIP                      ; [+1]
        ZBSR *VEATWORD                    ; [+1] consume optional var name

        ; --- inline FOR_FP: compute VARS index for loop var ---
        ; Read frame[0]=var letter; compute R1 = (var-'A')*2 for VARS indexing.
        LODA,R1 FORSP
        LODA,R0 FORBASE,R1               ; frame[0] = var letter
        STRA,R0 FORVAR
        SUBI,R0 A'A'                     ; 0..25
        STRZ,R1                          ; R1 = index
        ADDZ,R1                          ; R0 = index*2
        STRZ,R1                          ; R1 = index*2

        ; --- read step from frame[3:4] -> EXPH:EXPL ---
        ; (need R1 restored to FORSP base for frame access; save index in SC0)
        STRA,R0 SC0                      ; SC0 = index*2 (save for var write-back)
        LODA,R1 FORSP
        ADDI,R1 3
        LODA,R0 FORBASE,R1               ; frame[3] = stpH
        STRA,R0 EXPH
        LODA,R0 FORBASE,R1+               ; frame[4] = stpL
        STRA,R0 EXPL

        ; --- load current var value -> LNUMH:LNUML (via VARS,R1 indexed) ---
        LODA,R1 SC0                      ; R1 = index*2
        LODA,R0 VARS,R1                  ; var hi
        STRA,R0 LNUMH
        LODA,R0 VARS+1,R1                ; var lo
        STRA,R0 LNUML

        ; --- 16-bit signed add: var += step ---
        ; WC idiom: CPSL $08 clears carry; PPSL $08 sets WC so hi-add includes carry.
        CPSL PSW_WC                      ; clear WC
        LODA,R0 LNUML
        ADDA,R0 EXPL                     ; lo: var_lo + step_lo
        STRA,R0 LNUML                    ; new var lo
        PPSL PSW_WC                      ; set WC: carry propagates into hi add
        LODA,R0 LNUMH
        ADDA,R0 EXPH                     ; hi: var_hi + step_hi + carry
        CPSL PSW_WC                      ; clear WC
        STRA,R0 LNUMH                    ; new var hi

        ; --- write updated var back to VARS ---
        LODA,R1 SC0                      ; R1 = index*2
        STRA,R0 VARS,R1                  ; write var hi  (R0 still = new var hi)
        LODA,R0 LNUML
        STRA,R0 VARS,R1+                  ; write var lo

        ; --- signed 16-bit compare: var vs limit ---
        ; Read limit from frame[1:2]
        LODA,R1 FORSP
        LODA,R0 FORBASE,R1+               ; frame[1] = limH
        STRA,R0 SC0                      ; SC0 = limH (biased below)
        LODA,R0 FORBASE,R1+               ; frame[2] = limL
        STRA,R0 EXPL                     ; EXPL = limL (scratch; step already saved)
        ; Note: EXPH still holds stpH -- needed for step sign test below.
        ; Shared biased signed compare:
        ;   biased(limH) - biased(varH): GT -> lim>var (var<lim), LT -> lim<var (var>lim)
        LODA,R0 SC0
        EORI,R0 $80                      ; biased limH
        STRA,R0 SC0
        LODA,R0 LNUMH
        EORI,R0 $80                      ; biased varH
        SUBA,R0 SC0                      ; biased(varH) - biased(limH)
        BCTR,GT DN_VAR_GT                ; var > lim (hi bytes)
        BCTR,LT DN_VAR_LT                ; var < lim (hi bytes)
        ; hi bytes equal: compare lo bytes (unsigned, no bias needed)
        LODA,R0 EXPL                     ; limL
        SUBA,R0 LNUML                    ; limL - varL
        BCTR,GT DN_VAR_LT                ; lim.lo > var.lo -> var < lim
        BCTR,LT DN_VAR_GT                ; lim.lo < var.lo -> var > lim
        BCTR,UN DN_LOOP                  ; equal: body runs at limit value
DN_VAR_LT:
        ; var < lim: loop if positive step, exit if negative.
        ; EXPH = stpH. bit7=1 (LT after LODA) -> negative step -> exit.
        LODA,R0 EXPH
        BCTR,LT DN_EXIT
        ; positive step: fall through to DN_LOOP
DN_LOOP:
        ; Branch back to loop body: load nlp from frame[5:6]
        LODA,R1 FORSP
        ADDI,R1 5
        LODA,R0 FORBASE,R1               ; frame[5] = nlpH
        STRA,R0 GOTOH
        LODA,R0 FORBASE,R1+               ; frame[6] = nlpL
        STRA,R0 GOTOL
        BCTA,UN DRT_GO                   ; Set GOTOFLG $03 = FOR direct NLP branch (DO_RETURN)
DN_VAR_GT:
        ; var > lim: exit if positive step, loop if negative.
        LODA,R0 EXPH
        BCTR,LT DN_LOOP                  ; negative step: keep going down
        ; positive step: fall through to DN_EXIT
DN_EXIT:
        ; pop frame: FORSP -= 7, or $FF if was 0 (stack now empty)
        LODA,R0 FORSP
        BCTR,EQ DN_POP_EMPTY
        SUBI,R0 7
        db $EC                          ; consume next 2 bytes
DN_POP_EMPTY:
        LODI,R0 $FF
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
        LODA,R0 TMPH
        SUBA,R0 PEH
        BCTA,GT DR_STOP
        BCTR,LT DR_EXEC
        LODA,R0 TMPL
        SUBA,R0 PEL
        TPSL $01
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
        COMI,R0 3
        BCTR,EQ DR_FORNLP                ; $03: FOR direct address
        BCTR,UN DR_GOTO                  ; $01/$02: line-number goto/gosub
DR_SEQ:
        LODA,R0 SWSTK
        STRA,R0 TMPH
        LODA,R0 SWSTK+1
        STRA,R0 TMPL
        ZBRR *VDR_LP 
DR_FORNLP:
        ; FOR/NEXT loop-back: GOTOH:GOTOL is a direct program-store address.
        EORZ,R0
        STRA,R0 GOTOFLG
        LODA,R0 GOTOH
        STRA,R0 TMPH
        LODA,R0 GOTOL
        STRA,R0 TMPL
        ZBRR *VDR_LP 
DR_GOTO:
        ; GOTOFLG=$01 (GOTO) or $02 (GOSUB, return addr already on GSBASE).
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
; Clobbers: R0, EXPH, EXPL, LNUMH, LNUML, TMPH, TMPL
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
        LODA,R0 *IPH
        BCTR,EQ TSL_DEL
        BSTR,UN STORE_LINE               ; [+1]
        BCTR,UN TSL_DONE
TSL_DEL:
        BSTA,UN DELETE_LINE              ; [+1]
TSL_DONE:
        LODI,R0 1                        ; CC=GT: line stored/deleted
        RETC,UN

; =============================================================================
;  STORE_LINE -- Insert a numbered line into the sorted program store
; Record format: [linehi][linelo][body...][CR]
; In:  LNUMH:LNUML = line number; IPH:IPL -> body (NUL-terminated)
; Out: line inserted; PEH:PEL updated
; Clobbers: R0, R1, R3, SC0, SC1, CURH, CURL, TMPH, TMPL, EXPH, EXPL, GOTOH, GOTOL
STORE_LINE:
        BSTA,UN DELETE_LINE              ; [+1] remove if exists

        ; measure body length via TMPH:TMPL
        ZBSR *VIP_TO_TMP
        LODI,R3 0
SL_MEAS:
        LODA,R0 *TMPH
        BCTR,EQ SL_MEASD
        ZBSR *VINC_TMP  
        BIRR,R3 SL_MEAS
SL_MEASD:
        STRA,R3 SC0                      ; SC0 = body length
        LODA,R0 SC0
        ADDI,R0 3                        ; record size = 2+body+CR
        STRA,R0 SC1

        ; check free space: PROGLIM - PEL:PEH >= SC1
        LODI,R0 >PROGLIM
        SUBA,R0 PEL
        STRA,R0 CURL
        LODI,R0 <PROGLIM
        SUBA,R0 PEH
        BCFR,LT SL_NBC
        SUBI,R0 1
SL_NBC:
        STRA,R0 CURH
        LODA,R0 CURH
        BCTR,GT SL_ROOM
        LODA,R0 CURL
        SUBA,R0 SC1
        BCTA,LT JERROOM

SL_ROOM:
        BSTA,UN FIND_INS                 ; [+1] result -> TMPH:TMPL (insertion point)
        ZBSR *VTMP_TO_EXP16              ; EXPH:EXPL = TMPH:TMPL (insertion point)

        ; save line number before shift loop clobbers LNUMH:LNUML
        LODA,R0 LNUMH
        STRA,R0 CURH
        LODA,R0 LNUML
        STRA,R0 CURL

        ; shift loop: src = PE, dst = PE + SC1; move backwards to insertion point
        LODA,R0 PEH
        STRA,R0 LNUMH
        LODA,R0 PEL
        STRA,R0 LNUML
        ADDA,R0 SC1                      ; R0 = sum_lo = (PEL + SC1) mod 256
        STRA,R0 GOTOL                    ; GOTOL = sum_lo
        SUBA,R0 SC1                      ; BUG-SL-01 FIX: sum_lo - SC1 = PEL if no carry,
                                         ; negative (LT) if carry (sum_lo wrapped below SC1)
        BCFR,LT SL_GNC                   ; CC != LT: no carry -> GOTOH = PEH
        LODA,R0 PEH                      ; carry: GOTOH = PEH + 1
        ADDI,R0 1
        BCTR,UN SL_DNC
SL_GNC:
        LODA,R0 PEH                      ; no carry: GOTOH = PEH
SL_DNC:
        STRA,R0 GOTOH

SL_SHLOOP:
        ; stop when src == insertion point
        LODA,R0 LNUMH
        SUBA,R0 EXPH
        BCTR,GT SL_DOMOV
        BCTR,LT SL_NOSHIFT               ; BUG-LE FIX: src.hi < ins.hi: done
        LODA,R0 LNUML
        PPSL $02                         ; unsigned compare mode
        COMA,R0 EXPL
        CPSL $02
        BCTR,EQ SL_NOSHIFT               ; src == ins (unsigned): done
        BCTR,LT SL_NOSHIFT               ; src.lo < ins.lo (unsigned): done

SL_DOMOV:
        BSTA,UN DEC_LNUM                 ; [+1] pre-decrement src (LNUMH:LNUML)
        BSTA,UN DEC_GOTO                 ; [+1] pre-decrement dst (GOTOH:GOTOL)
        LODA,R1 *LNUMH
        STRA,R1 *GOTOH
        BCTR,UN SL_SHLOOP

SL_NOSHIFT:
        ; write record at insertion point EXPH:EXPL
        LODA,R0 CURH
        STRA,R0 LNUMH
        LODA,R0 CURL
        STRA,R0 LNUML
        ZBSR *VIP_TO_TMP
        LODA,R0 LNUMH
        STRA,R0 *EXPH                    ; write line hi
        ZBSR *VINC_EXP  
        LODA,R0 LNUML
        STRA,R0 *EXPH                    ; write line lo
        ZBSR *VINC_EXP  
SL_WBODY:
        LODA,R1 *TMPH
        BCTR,EQ SL_WDONE
        BSTR,UN MEMCPY
        BCTR,UN SL_WBODY
SL_WDONE:
        LODI,R0 CR
        STRA,R0 *EXPH                    ; write CR terminator
        ZBSR *VINC_EXP  
        ; update PEH:PEL += SC1
        LODA,R0 PEL
        ADDA,R0 SC1
        STRA,R0 PEL
        TPSL $01
        RETC,LT                          ; no carry: done
        LODA,R0 PEH
        ADDI,R0 1
        STRA,R0 PEH
        RETC,UN

; =============================================================================
;  MEMCPY -- Copy single byte: *EXP++ = *TMP++
; In:  TMPH:TMPL -> source, EXPH:EXPL -> dest
; Out: one byte copied; both pointers incremented
; Clobbers: R0, R1
MEMCPY:
        LODA,R1 *TMPH
        STRA,R1 *EXPH
        ZBSR *VINC_TMP  
        ZBRR *VINC_EXP                   ; tail call: INC_EXP's RETC,UN returns to our caller

; =============================================================================
;  DELETE_LINE -- Remove a line from the program store (silent no-op if not found)
; In:  LNUMH:LNUML = line number
; Out: line removed; PEH:PEL updated. CC=EQ found/deleted, CC=GT not found (silent).
; Clobbers: R0, R1, SC0, CURH, CURL, TMPH, TMPL, EXPH, EXPL
DELETE_LINE:
        BSTA,UN FIND_LINE                ; [+1] CC=EQ found, CC=GT not found
        RETC,GT                          ; not found: silent return

        ZBSR *VTMP_TO_EXP16              ; EXP = record start
        ; advance past line number bytes (skip 2)
        ZBSR *VINC_TMP                    ; [+1] skip hi byte of line number
        ZBSR *VINC_TMP                    ; [+1] skip lo byte of line number
        ; scan body until CR to get record size in SC0
        LODI,R0 3                        ; start at 3 (hi+lo+CR)
        STRA,R0 SC0
DL2_SCAN:
        LODA,R1 *TMPH
        COMI,R1 CR
        BCTR,EQ DL2_SCAND
        ZBSR *VINC_TMP  
        LODA,R0 SC0
        ADDI,R0 1
        STRA,R0 SC0
        BCTR,UN DL2_SCAN
DL2_SCAND:
        ZBSR *VINC_TMP                    ; skip CR byte itself
        ; copy TMPH:TMPL..PE-1 to EXPH:EXPL
DL2_LP:
        LODA,R0 TMPH
        SUBA,R0 PEH
        BCTR,GT DL2_DONE
        BCTR,LT DL2_MOV
        LODA,R0 TMPL
        SUBA,R0 PEL
        BCFR,LT DL2_DONE                 ; TMPL >= PEL (no borrow): at or past end
DL2_MOV:
        BSTA,UN MEMCPY
        BCTR,UN DL2_LP
DL2_DONE:
        ; PEH:PEL -= SC0
        LODA,R0 PEL
        SUBA,R0 SC0
        STRA,R0 PEL
        TPSL $01
        RETC,EQ                          ; no borrow
        LODA,R0 PEH
        SUBI,R0 1
        STRA,R0 PEH
        RETC,UN

; =============================================================================
;  FIND_LINE -- Search for line LNUMH:LNUML in program store
; Out: TMPH:TMPL = record start if found; CC=EQ found, CC=GT not found.
; Clobbers: R0, TMPH, TMPL, EXPH, EXPL
FIND_LINE:
        BSTA,UN FIND_INS                 ; [+1]
        ; check if at end of program
        LODA,R0 TMPH
        SUBA,R0 PEH
        BCTR,GT FL_RET_NF
        BCTR,LT FL_CHK
        LODA,R0 TMPL
        SUBA,R0 PEL
        TPSL $01
        BCTR,EQ FL_RET_NF
        BCTR,UN FL_CHK
FL_CHK:
        LODA,R0 *TMPH
        SUBA,R0 LNUMH
        BCTR,EQ FL_CHKLO
FL_RET_NF:
        LODI,R0 1                        ; CC=GT: not found
        RETC,UN

FL_CHKLO:
        LODA,R0 TMPL
        ADDI,R0 1
        STRA,R0 EXPL
        LODA,R0 TMPH
        TPSL $01
        BCTR,LT FL_LH
        ADDI,R0 1
FL_LH:
        STRA,R0 EXPH
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
        LODA,R0 TMPH
        SUBA,R0 PEH
        RETC,GT
        BCTR,LT FI_CHK
        LODA,R0 TMPL
        SUBA,R0 PEL
        TPSL $01
        RETC,EQ
FI_CHK:
        LODA,R0 LNUMH
        SUBA,R0 *TMPH                    ; LNUMH - stored.hi
        BCTR,GT FI_ADV
        BCTR,LT FI_RET
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
        BCTA,UN FI_LP

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
        ; Check for functions and tail call them to return.
        ; Setup at statement FUNC_TAB with TMPH:TMPL as pointer
        LODI,R0 <FUNC_TAB
        STRA,R0 TMPH
        LODI,R0 >FUNC_TAB
        STRA,R0 TMPL
        BCTA,UN MATCH_KW                ; resumes at PE_NOFUNC if not match
PE_NOFUNC:
        ZBSR *VDEC_IP                   ; backup IP 2 slots from KW Match
        ZBSR *VDEC_IP
        LODI,R3 $FF                     ; SW stack empty sentinel
EXPR_AM:
        LODI,R0 >EAM0_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <EAM0_RET
        STRA,R0 SWBASE,R3+
        ZBRR *VEAM_ATOM 
EAM0_RET:
        LODI,R0 >EAM_HI0_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <EAM_HI0_RET
        STRA,R0 SWBASE,R3+
        ZBRR *VEAM_HI 
EAM_HI0_RET:
EAM_LO_LOOP:
;        ZBSR *VWSKIP  
        LODA,R0 *IPH
        COMI,R0 A'+'
        BCTR,EQ EAM_PLUS
        COMI,R0 A'-'
        BCTR,EQ EAM_MINUS
        BCTA,UN PARSER_RET
EAM_PLUS:
        ; PAREN-NEST-01 fix (v4.3): push left operand onto SWBASE instead of
        ; flat SAVEH:SAVEL. A flat cell gets clobbered by a same-precedence
        ; op at a deeper recursion (e.g. "2*(3*4)"); the SW stack frame can't
        ; be, since it's LIFO and balanced with every push/pop in this loop.
        LODA,R0 EXPL
        STRA,R0 SWBASE,R3+               ; push left lo
        LODA,R0 EXPH
        STRA,R0 SWBASE,R3+               ; push left hi
        ZBSR *VINC_IP  
        LODI,R0 >EAM_P_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <EAM_P_RET
        STRA,R0 SWBASE,R3+
        ZBRR *VEAM_ATOM 
EAM_P_RET:
        LODI,R0 >ADD16_SAVE_EXP ; >EAM_PH_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <ADD16_SAVE_EXP ; <EAM_PH_RET
        STRA,R0 SWBASE,R3+
        ZBRR *VEAM_HI 
;EAM_PH_RET:
;        BCTR,UN ADD16_SAVE_EXP            ; EXP = SAVE + EXP (V4.2: shared)
EAM_MINUS:
        LODA,R0 EXPL
        STRA,R0 SWBASE,R3+               ; push left lo
        LODA,R0 EXPH
        STRA,R0 SWBASE,R3+               ; push left hi
        ZBSR *VINC_IP  
        LODI,R0 >EAM_M_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <EAM_M_RET
        STRA,R0 SWBASE,R3+
        ZBRR *VEAM_ATOM 
EAM_M_RET:
        LODI,R0 >EAM_MH_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <EAM_MH_RET
        STRA,R0 SWBASE,R3+
        ZBRR *VEAM_HI 
EAM_MH_RET:
        BSTA,UN NEG_EXP_BODY              ; EXP = -EXP (V4.2: shared, was SUBA)
;        BCTR,UN ADD16_SAVE_EXP            ; EXP = SAVE + (-EXP) = SAVE - EXP
        ; drop through
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
        BCTA,UN EAM_LO_LOOP

EAM_HI:
        ZBSR *VWSKIP  
        LODA,R0 *IPH
        COMI,R0 A'*'
        BCTR,EQ EAM_MUL
        COMI,R0 A'/'
        BCTR,EQ EAM_DIV
        COMI,R0 A'%'
        BCTA,EQ EAM_MOD
        BCTA,UN PARSER_RET
EAM_MUL:
        ; PAREN-NEST-01 fix (v4.3): push, not flat E1SAVH:E1SAVL (see EAM_PLUS)
        LODA,R0 EXPL
        STRA,R0 SWBASE,R3+
        LODA,R0 EXPH
        STRA,R0 SWBASE,R3+
        ZBSR *VINC_IP  
        LODI,R0 >MU_AT_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <MU_AT_RET
        STRA,R0 SWBASE,R3+
        ZBRR *VEAM_ATOM 
MU_AT_RET:
        BSTA,UN POP_SAVE_TO_TMP           ; TMPH:TMPL = popped left operand
        BSTA,UN MUL16
        ZBRR *VEAM_HI 
EAM_DIV:
        LODA,R0 EXPL
        STRA,R0 SWBASE,R3+
        LODA,R0 EXPH
        STRA,R0 SWBASE,R3+
        ZBSR *VINC_IP  
        LODI,R0 >DV_AT_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <DV_AT_RET
        STRA,R0 SWBASE,R3+
        ZBRR *VEAM_ATOM 
DV_AT_RET:
        BSTR,UN POP_SAVE_TO_TMP
        BSTA,UN DIV16
        ZBRR *VEAM_HI 
EAM_MOD:
        LODA,R0 EXPL
        STRA,R0 SWBASE,R3+
        LODA,R0 EXPH
        STRA,R0 SWBASE,R3+
        ZBSR *VINC_IP  
        LODI,R0 >MD_AT_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <MD_AT_RET
        STRA,R0 SWBASE,R3+
        ZBRR *VEAM_ATOM 
MD_AT_RET:
        BSTR,UN POP_SAVE_TO_TMP
        BSTA,UN DIV16
        ZBSR *VTMP_TO_EXP16              ; EXPH:EXPL = TMPH:TMPL (remainder)
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
        LODA,R0 *IPH
        COMI,R0 A'-'
        BCTR,EQ EAM_NEG
        COMI,R0 A'+'
        BCTR,EQ EAM_POS
        COMI,R0 A'('
        BCTR,EQ EAM_PAREN
        BSTR,UN PARSE_FACTOR
        BCTR,UN PARSER_RET
EAM_NEG:
        ZBSR *VINC_IP  
        LODI,R0 >NEG_AT_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <NEG_AT_RET
        STRA,R0 SWBASE,R3+
        ZBRR *VEAM_ATOM 
NEG_AT_RET:
        BSTA,UN NEG_EXP_BODY
        BCTR,UN PARSER_RET
EAM_POS:
        ZBSR *VINC_IP  
        LODI,R0 >PARSER_RET ; >POS_AT_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <PARSER_RET; <POS_AT_RET
        STRA,R0 SWBASE,R3+
        ZBRR *VEAM_ATOM 
;POS_AT_RET:
;        BCTR,UN PARSER_RET
EAM_PAREN:
        ZBSR *VINC_IP  
        LODI,R0 >EP_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <EP_RET
        STRA,R0 SWBASE,R3+
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
        BCTA,UN PARSE_S16                ; tail call: PARSE_S16's RETC,UN returns to our caller

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
        BCTA,LT PU16_RET ; RETC,LT
        COMI,R0 A'9'+1
        BCTR,LT PU16_DIG
        BCTA,UN PU16_RET ; RETC,UN
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
        STRA,R0 EXPL
        TPSL $01
        BCTR,LT PU16_MNC
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
PU16_MNC:
        LODA,R0 EXPH
        ADDA,R0 TMPH
        STRA,R0 EXPH
        BDRR,R3 PU16_M10
        LODA,R3 R3SAVE                   ; restore SW stack pointer
        LODA,R0 EXPL
        ADDA,R0 SC0
        STRA,R0 EXPL
        TPSL $01
        BCTR,LT PU16_DIG_NC
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
PU16_DIG_NC:
        BCTA,UN PU16_LP

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
DO_NEG_FUNC:
        ZBSR *VPARSE_EXPR
        BCTR,UN NEG_EXP_BODY

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
DO_ABS_FUNC:
        ZBSR *VPARSE_EXPR
        ; drop through

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
        LODA,R0 *IPH
        COMI,R0 CR                       ; no args
        BCFR,LT DLS_ARG                  ; digits - get args
DLS_FULL:
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
        ; v4.3.3 BUG-POKEADDR-01 fix (pre-existing, found while testing the
        ; showcase's new POKE/PEEK demo): address was saved via EXP16_TO_TMP
        ; into TMPH:TMPL, but PARSE_EXPR clobbers TMPH:TMPL internally while
        ; parsing the payload expression below, corrupting the address
        ; before the store executes - confirmed in the simulator: it wrote
        ; to a garbage location and hung the interpreter on the next
        ; statement. ARGAH:ARGAL (added for AND/OR/XOR, same failure mode)
        ; is never touched by PARSE_EXPR and survives the second parse.
        ; v4.4.2: shares PARSE_2ARGS's body (see that header) - tail-jumps
        ; in at P2A_NOPAREN, skipping the '(' check it never wanted.
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
;  v4.3.3: BUG-FUNCRAS-01 fix (RAS depth) + BUG-FUNCOP-01 fix (R1 corruption)
;  + BUG-FUNCARG-01 fix (TMP corruption). Confirmed via simulator trace:
;  - v4.3.1 passed the op selector in R1 across the two VPARSE_EXPR calls,
;    but R1 is clobbered somewhere inside PARSE_EXPR (came back holding the
;    ASCII code of the last digit parsed). Selector now lives in RAM cell
;    FUNCOP instead (v4.3.2).
;  - v4.3.2 stashed arg 'a' in TMPH:TMPL via EXP16_TO_TMP, but PARSE_EXPR's
;    own clobber list includes TMPH/TMPL - parsing arg 'b' destroys 'a'
;    before the dispatch ever uses it. Arg 'a' now lives in dedicated
;    ARGAH:ARGAL cells that PARSE_EXPR never touches.
;
;  Register-lifetime note (why it's safe to clobber R1/R2/R3 freely in all
;  of PARSE_2ARGS/DO_RND_FUNC/etc.): these are all reached ONLY via
;  FUNC_TAB's dispatch out of PARSE_EXPR (MATCH_KW tail-jumps to the
;  handler, which RETC,UN's straight back to whatever called PARSE_EXPR -
;  it never passes through PARSER_RET's R3-based SW-stack logic). Nothing
;  downstream depends on R1/R2/R3 surviving a function call. Note this
;  also means FUNC_TAB names are only recognized as the LEADING token of a
;  fresh PARSE_EXPR call (right after a statement keyword, or right after
;  a paren) - EAM_ATOM does not re-check FUNC_TAB for atoms after an
;  operator, so e.g. "PRINT 1+ABS(-5)" misparses ABS as the bare variable
;  A (giving 1+A, not 1+5). This is a pre-existing limitation (confirmed
;  present in v4.2 too, not a v4.3 regression) - see FUNCATOM-01 in the
;  KNOWN OPEN ITEMS at the top of this file.
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
;  argument pairs - this replaces HELPER entirely (no separate routine).
;  - AND/OR/XOR enter at PARSE_2ARGS (top): requires '(', requires ')'.
;  - DO_POKE/DO_LIST tail-jump (BCTA,UN, no RAS cost) straight into
;    P2A_NOPAREN, skipping the '(' check; the ')' check below is optional
;    (consumed if present, skipped otherwise) since POKE/LIST have no
;    closing paren at all, just a CR.
;  - FUNCOP (set by the caller, see DO_AND_FUNC/DO_POKE/DO_LIST) is the
;    literal byte offset from P2A_ANDOP to the target handler, computed at
;    ASSEMBLY time - BXA jumps STRAIGHT to the handler (no jump table: an
;    earlier draft had one, but a literal per-handler offset makes it
;    pure dead weight - BXA already lands exactly on the handler).
; In:  IP -> '(' (top entry) or first char of 'a' (P2A_NOPAREN entry)
;      FUNCOP = target offset from P2A_ANDOP
; Out: per-handler (see P2A_ANDOP/OROP/XOROP/POKEOP/LISTOP below)
; Clobbers: R0, R3, SAVEH, SAVEL, NEGFLG, SC0, SC1, TMPH, TMPL, ARGAH, ARGAL
PARSE_2ARGS:
        ZBSR *VWSKIP
        LODA,R0 *IPH
        COMI,R0 A'('
        BCFA,EQ JSYNERR                 ; require opening paren
        ZBSR *VINC_IP
P2A_NOPAREN:
        ZBSR *VPARSE_EXPR               ; a -> EXP
        LODA,R0 EXPH                     ; ARGAH:ARGAL = a (NOT TMPH:TMPL -
        STRA,R0 ARGAH                    ; PARSE_EXPR clobbers TMP while
        LODA,R0 EXPL                      ; parsing arg 'b' below)
        STRA,R0 ARGAL
        ZBSR *VWSKIP
        LODA,R0 *IPH
        COMI,R0 A','
        BCFA,EQ JSYNERR                 ; require comma
        ZBSR *VINC_IP
        ZBSR *VPARSE_EXPR               ; b -> EXP
        ZBSR *VWSKIP
        LODA,R0 *IPH
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
        RETC,UN
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
DO_NOT_FUNC:
        ZBSR *VPARSE_EXPR
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        BCTR,UN P2A_RET        

P2A_LISTOP:
        STRA,R0 LNUMH
        LODA,R0 ARGAL
        STRA,R0 LNUML
        LODI,R0 CURH-IPH                 ; CURH offset from IPH
        BSTA,UN EXP16_TO_ET               ; move end (EXP) to CUR
        BSTA,UN FIND_INS                  ; TMP = first record >= start (or prog end)
        ; drop through
; =============================================================================
;  DO_LIST -- Prologue  Print stored BASIC lines, optionally filtered by range
; See DO_LIST
DLS_LP:
        ; Check TMP against program end
        LODA,R0 TMPH
        SUBA,R0 PEH
        RETC,GT
        BCTR,LT DLS_BODY
        LODA,R0 TMPL
        SUBA,R0 PEL
        TPSL $01
        RETC,EQ
DLS_BODY:
        ; Copy TMP -> IP, read line number hi; check against end hi (CURH)
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
        ; Check line number (EXPH:EXPL) against end (CURH:CURL)
        LODA,R0 EXPH
        SUBA,R0 CURH
        RETC,GT                         ; line hi > end hi: past range
        BCTR,LT DLS_PRNUM                 ; line hi < end hi: in range
        LODA,R0 EXPL
        SUBA,R0 CURL
        RETC,GT                           ; hi equal, line lo > end lo: past range
DLS_PRNUM:
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
        ; Update TMP from IP for next iteration
        ZBSR *VIP_TO_TMP
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
        STRA,R0 EXPL
        TPSL $01
        BCTR,LT MU_MNC
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
MU_MNC:
        LODA,R0 EXPH
        ADDA,R0 SC0
        STRA,R0 EXPH
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
        ZBSR *VINC_EXP  
        BCTR,UN DV_LP

JERRDIVZER:
        LODI,R0 ERR_DIV_ZERO
        ZBRR *VDO_ERROR 

; =============================================================================
;  PRINT_S16 -- Print signed 16-bit value EXPH:EXPL as decimal
; In:  EXPH:EXPL = signed value
; Out: decimal digits written to COUT
; Clobbers: R0, R1, R3, TMPH, TMPL, NEGFLG, SC0, SC1
PRINT_S16:
        LODA,R1 EXPH            ; get high byte
        BCTR,LT IS_NEG          ; If negative, jump to handle '-'

        ; Check for ZERO
        LODA,R0 EXPL            ; get low byte
        IORZ R1                 ; R0 = EXPL | EXPH
        BCFR,EQ PS_NZ           ; >0, flow into PS_NZ (Digit Parser)

        LODI,R0 A'0'            ; Handle Zero
        ZBRR *VCOUT             ; Print '0' and tail call return
IS_NEG:
        LODI,R0 A'-'            ; Its negative so print minus but neg what?
        ZBSR *VCOUT
        
        ; Check for -32768
        LODA,R0 EXPL            ; check if EXPL is zero
        BCFR,EQ DO_NEG          ; nope its a real neg number
        COMI,R1 $80             ; from before, check if its exactly 0x80
        BCFR,EQ DO_NEG          ; nope again real neg number
        ; otherwise it is the 32768 magic number
        LODI,R1 0               ; string offset
MIN_LP:                         ; BDRR no advantage as still need a RETC
        LODR,R0 MSG_MIN-1,R1+   ; Load char from table
        RETC,EQ                 ; return on null
        ZBSR *VCOUT             ; Print
        BCTR,UN MIN_LP
MSG_MIN:
        db "32768",0
DO_NEG:
        BSTA,UN NEG_EXP_BODY    ; Negate and fall into PS_NZ
PS_NZ:
        STRA,R3 R3SAVE          ; setup SW stack
        LODI,R3, $FF
        ; 
        LODI,R0 >PS_DONE        ; outer return
        STRA,R0 SWBASE,R3+
        LODI,R0 <PS_DONE
        STRA,R0 SWBASE,R3+
        ; fall through to PREC

; =============================================================================
;  PREC -- SW-recursive digit printer (divide EXP by 10, recurse, print)
; In:  EXPH:EXPL = value to print (>0)
; Out: digits written via COUT
; Clobbers: R0, R3, TMPH, TMPL, NEGFLG, SC0, SC1
PREC:
        BSTA,UN EXP16_TO_TMP            ; save address in TMP
        ZBSR *VCLR_EXP
        STRA,R0 NEGFLG
        STRA,R0 SC1
        LODI,R0 16
        STRA,R0 SC0
PR_LP:
        PPSL PSW_WC
        CPSL $01
        LODA,R0 TMPL
        RRL,R0
        STRA,R0 TMPL
        LODA,R0 TMPH
        RRL,R0
        STRA,R0 TMPH
        LODA,R0 SC1
        RRL,R0
        STRA,R0 SC1
        LODA,R0 NEGFLG
        RRL,R0
        STRA,R0 NEGFLG
        CPSL $01
        LODA,R0 EXPL
        RRL,R0
        STRA,R0 EXPL
        LODA,R0 EXPH
        RRL,R0
        STRA,R0 EXPH
        CPSL PSW_WC
        LODA,R0 NEGFLG
        BCTR,GT PR_QBIT
        LODA,R0 SC1
        COMI,R0 10
        BCTR,LT PR_NOQBIT
PR_QBIT:
        LODA,R0 SC1
        SUBI,R0 10
        STRA,R0 SC1
        TPSL $01
        BCTR,EQ PR_SNB
        LODA,R0 NEGFLG
        SUBI,R0 1
        STRA,R0 NEGFLG
PR_SNB:
        LODA,R0 EXPL
        IORI,R0 $01
        STRA,R0 EXPL
PR_NOQBIT:
        LODA,R0 SC0
        SUBI,R0 1
        STRA,R0 SC0
        BCTA,GT PR_LP
        LODA,R0 SC1
        STRA,R0 SWBASE,R3+
        LODA,R0 EXPH
        BCTR,GT PR_REC
        LODA,R0 EXPL
        BCTR,EQ PR_PRINT
PR_REC:
        LODI,R0 >PR_PRINT
        STRA,R0 SWBASE,R3+
        LODI,R0 <PR_PRINT
        STRA,R0 SWBASE,R3+
        BCTA,UN PREC
PR_PRINT:
        LODA,R0 SWBASE,R3
        SUBI,R3 1
        ADDI,R0 A'0'
        ZBSR *VCOUT  
        ; fall through to SWRETURN

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

PS_DONE:
        LODA,R3 R3SAVE  ; restore R3
        RETC,UN

; =============================================================================
;  RDLINE -- Read a line from input into IBUF with echo and backspace
; In:  nothing
; Out: IBUF = NUL-terminated input line.
;      V4.2: IPH:IPL is NOT maintained as a valid pointer during/after input;
;      R3 is used as an index into IBUF instead (cheaper bounds check and
;      backspace than full 16-bit pointer arithmetic). R3=$FF means empty
;      (matches the SWBASE empty-stack convention) since the 2650's ",R3+"
;      addressing mode pre-increments before the access. Both callers
;      (REPL, DO_INPUT) already re-point IP via VSET_IP_IBUF immediately
;      after calling RDLINE, so this is safe -- do not add a 3rd caller that
;      expects IPH:IPL -> one past last char without re-checking this.
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
        COMI,R3 62
        BCTR,LT RL_STORE
        BCTR,UN RL_LP
RL_STORE:
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
;  GETCI_UC -- Read *IPH uppercase into R0, advance IP
; In:  IPH:IPL -> current position
; Out: R0 = uppercased char; IP advanced by 1; R1 clobbered
; Clobbers: R0, R1
GETCI_UC:
        LODA,R0 *IPH
        BSTR,UN UPCASE                   ; [+1]
        STRZ,R1                          ; save before INC_IP clobbers R0
        ZBSR *VINC_IP                    ; [+1]
        LODZ,R1                          ; restore
WSKIPRET:
        RETC,UN

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
        RETC,UN

; =============================================================================
;  SHARED 16-BIT POINTER DECREMENT -- DEC_ET family
; DEC_LNUM: LNUMH:LNUML -= 1   (offset LNUMH-IPH from IPH)
; DEC_GOTO:  GOTOH:GOTOL -= 1   (offset GOTOH-IPH from IPH)
; DEC_IP:    IPH:IPL    -= 1    (offset  0 from IPH)
; All share DEC_ET body via register bank switch, mirroring INC_ET.
; Byte-skip chain: $EC (COMA, skips 2) and $C4 (COMI,R0, skips 1).
; Borrow: after SUBI R0,1 -- R0 was 0 -> result $FF, CC=LT (borrow).
;   BCFR,LT branches when CC != LT (no borrow) -- skip hi decrement.
;   Saves 1 byte vs TPSL $01 / RETC,EQ idiom used in INC_ET.
; BUG-DEC-01 FIX retained: borrow detected via carry (BCFR,LT), not sign.
; RAS rule: NO BSTA inside body -- must not consume extra depth.
; DEC_EXP/DEC_TMP omitted: MUL16 call site is at RAS depth 5+1=6 (unsafe).
DEC_LNUM:
        LODI,R0 LNUMH-IPH       ; LNUMH offset from IPH (= 12); assembly-time expression
        db $EC                  ; COMA,R0: skip next 2 bytes (the LODI,R0 8)
DEC_GOTO:
        LODI,R0 GOTOH-IPH       ; GOTOH offset from IPH (= 8); assembly-time expression
        db $C4                  ; COMI,R0: skip next 1 byte (the EORZ,R0)
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
        db $EC                  ; COMA,R0: skip next 2 bytes
TMP_TO_EXP16:
        LODI,R0 TMPH-IPH        ; TMPH offset from IPH (= 2)
ET_TO_EXP16:
        PPSL PSW_RS             ; switch to alternate register bank
        STRZ R1                 ; alt-R1 = R0 = source offset
        LODA,R0 IPH,R1          ; load hi byte from source
        STRA,R0 EXPH
        LODA,R0 IPL,R1          ; load lo byte from source
        STRA,R0 EXPL
        BCTR,UN ET_RET

; =============================================================================
;  IP_TO_TMP -- copy IPH:IPL to TMPH:TMPL (v4.3, replaces 3 inline copies)
; In:  IPH:IPL = source pointer
; Out: TMPH:TMPL = IPH:IPL
; Clobbers: R0
IP_TO_TMP:
        LODA,R0 IPH
        STRA,R0 TMPH
        LODA,R0 IPL
        STRA,R0 TMPL
        RETC,UN

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
        BSTA,UN CUR_TO_EXP16             ; EXPH:EXPL = CURH:CURL
        BSTA,UN PRINT_S16                ; [+1]
DE_NL:
        BSTR,UN PRT_CRLF
        BSTA,UN DO_END                   ; [+1] clears SWSP, FORSP, GOTOFLG, RUNFLG
        BCTA,UN REPL                     ; Resets RAS

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
        BSTA,UN PRINT_S16                ; Print decimal value
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
;  TABLES 
BANNER:
        DB CR, LF, "uBASIC 2650 V4.3", CR, LF, "Bytes Free:",NUL

; -- Keyword dispatch table
; Format: [c1][c2][c3][hi][lo]  NUL-terminated on c1, followed by no match handler
; hi:lo = handler address. Matched on first three uppercase chars.
; c3=A' ' (space) = wildcard (IF -- only 2 chars before body).
; THEN matched internally by DO_IF not here.
KW_TAB:
        DB "END", <DO_END,    >DO_END     ; END
        DB "FOR", <DO_FOR,    >DO_FOR     ; FOR
        DB "FRE", <DO_FREE,   >DO_FREE    ; FREE
        DB "GOS", <DO_GOSUB,  >DO_GOSUB   ; GOSUB
        DB "GOT", <DO_GOTO,   >DO_GOTO    ; GOTO
        DB "IF ", <DO_IF,     >DO_IF      ; IF (wildcard)
        DB "INP", <DO_INPUT,  >DO_INPUT   ; INPUT
        DB "LET", <DO_LET,    >DO_LET     ; LET
        DB "LIS", <DO_LIST,   >DO_LIST    ; LIST
        DB "NEW", <DO_NEW,    >DO_NEW     ; NEW
        DB "NEX", <DO_NEXT,   >DO_NEXT    ; NEXT
        DB "POK", <DO_POKE,   >DO_POKE    ; POKE
        DB "PRI", <DO_PRINT,  >DO_PRINT   ; PRINT
        DB "REM", <DO_REM,    >DO_REM     ; REM
        DB "RET", <DO_RETURN, >DO_RETURN  ; RETURN
        DB "RUN", <DO_RUN,    >DO_RUN     ; RUN
        DB NUL,   <SE_NOTKW,  >SE_NOTKW   ; No match handler

; -- Function Dispatch Table - EXP is input/output
; note TAB and CHR$ handled by PRINT as only meaningful there
FUNC_TAB:
        DB "ABS", <DO_ABS_FUNC, >DO_ABS_FUNC
        DB "AND", <DO_AND_FUNC, >DO_AND_FUNC
        DB "NEG", <DO_NEG_FUNC, >DO_NEG_FUNC
        DB "NOT", <DO_NOT_FUNC, >DO_NOT_FUNC
        DB "OR ", <DO_OR_FUNC,  >DO_OR_FUNC    ; OR (wildcard, 2-char keyword)
        DB "PEE", <DO_PEEK_FUNC,>DO_PEEK_FUNC
        DB "RND", <DO_RND_FUNC, >DO_RND_FUNC
        DB "USR", <DO_USR_FUNC, >DO_USR_FUNC
        DB "XOR", <DO_XOR_FUNC, >DO_XOR_FUNC
        DB NUL,   <PE_NOFUNC,   >PE_NOFUNC        ; No match handler
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

; FOR stack frame ordering
FORVAR  RES 1       ; FOR loop variable letter (A-Z)
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
FUNCOP  RES 1       ; PARSE_2ARGS: AND/OR/XOR op selector (0/1/2); RAM not a
                     ; register since R1 is clobbered inside PARSE_EXPR
ARGAH   RES 1       ; PARSE_2ARGS: arg 'a' scratch (hi); NOT TMPH - PARSE_EXPR
ARGAL   RES 1       ; itself clobbers TMPH:TMPL while parsing arg 'b'
SAVEL   RES 1       ; ADD16_SAVE_EXP: popped left operand scratch (lo)
TEMPRETH RES 1      ; SW return address hi
TEMPRETL RES 1      ; SW return address lo
RNDSEED RES 2       ; 16 bit Random

;  --- Flags & Stuff --- 
RUNFLG  RES 1       ; $01=running $00=immediate
R3SAVE  RES 1       ; Save/restore R3 across PARSE_U16 multiply loop
NEGFLG  RES 1       ; Sign flag
FORSP   RES 1       ; FOR stack pointer ($FF=empty, 0/7/14/21=frame offsets)
SWSP    RES 1       ; GOSUB stack pointer ($FF=empty)
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
VARS    RES 52      ; A-Z variables 2 bytes each

; =============================================================================
;  Pre-loaded SHOWCASE program
;
;  Line format: <lineno_hi> <lineno_lo> <body_ASCII> <CR>
;  Lines  10-190: feature demos (PRINT, CHR$, arithmetic, comparisons, GOTO loop)
;  Lines 192-218: FOR/NEXT and GOSUB/RETURN demos
;  Lines 220-240: function demos (ABS/NEG/AND/OR/XOR/NOT/PEEK/POKE/RND/LIST)
;  Lines 300-510: Mandelbrot set renderer (v4.3: widened, C=-144..28 step 4,
;                 44 cols vs v4.2's 32; row range I=-64..56 step 6 unchanged)
;  Line  530:     GOSUB subroutine (PRINT "sub"; / RETURN)
;
;  Format: DB hi,lo,"text",$0D  -- hi-then-lo matches DR_EXEC record format.
;  $22=DQ $3B=semicolon  in-string chars that need escaping.
; =============================================================================
PROG:
        DB 0,10,"REM uBASIC 2650 - SHOWCASE V4.3",$0D
        DB 0,20,"PRINT ",$22,"-- uBASIC 2650 V4.3 Showcase --",$22,$0D
        DB 0,30,"PRINT ",$22,"--- PRINT / CHR$ ---",$22,$0D                    ; 30  PRINT "--- PRINT / CHR$ ---"
        DB 0,40,"PRINT CHR$(65)",$3B,"CHR$(66)",$3B,"CHR$(67)",$0D             ; 40  PRINT CHR$(65);CHR$(66);CHR$(67)
        DB 0,50,"PRINT ",$22,"--- ARITHMETIC ---",$22,$0D                      ; 50  PRINT "--- ARITHMETIC ---"
        DB 0,60,"PRINT ",$22,"3+4=",$22,$3B,"3+4",$3B,$22,"  10-3=",$22,$3B,"10-3",$3B,$22,"  6*7=",$22,$3B,"6*7",$0D  ; 60  PRINT "3+4=";3+4;"  10-3=";10-3;"  6*7=";6*7
        DB 0,70,"PRINT ",$22,"20/4=",$22,$3B,"20/4",$3B,$22,"  17%5=",$22,$3B,"17%5",$0D  ; 70  PRINT "20/4=";20/4;"  17%5=";17%5
        DB 0,80,"PRINT ",$22,"--- COMPARISONS ---",$22,$0D                     ; 80  PRINT "--- COMPARISONS ---"
        DB 0,90,"IF 5>3 THEN PRINT ",$22,"5>3 ok",$22,$0D                      ; 90  IF 5>3 THEN PRINT "5>3 ok"
        DB 0,100,"IF 3<5 THEN PRINT ",$22,"3<5 ok",$22,$0D                     ; 100 IF 3<5 THEN PRINT "3<5 ok"
        DB 0,110,"IF 3>=3 THEN PRINT ",$22,"3>=3 ok",$22,$0D                   ; 110 IF 3>=3 THEN PRINT "3>=3 ok"
        DB 0,120,"IF 4<>3 THEN PRINT ",$22,"4<>3 ok",$22,$0D                   ; 120 IF 4<>3 THEN PRINT "4<>3 ok"
        DB 0,130,"IF 3=3 THEN PRINT ",$22,"3=3 ok",$22,$0D                     ; 130 IF 3=3 THEN PRINT "3=3 ok"
        DB 0,140,"PRINT ",$22,"--- LOOP via GOTO ---",$22,$0D                  ; 140 PRINT "--- LOOP via GOTO ---"
        DB 0,150,"I=1",$0D                                                      ; 150 I=1
        DB 0,160,"IF I>5 THEN GOTO 190",$0D                                    ; 160 IF I>5 THEN GOTO 190
        DB 0,170,"PRINT I",$3B,$0D                                              ; 170 PRINT I;
        DB 0,180,"I=I+1",$0D                                                    ; 180 I=I+1
        DB 0,185,"GOTO 160",$0D                                                 ; 185 GOTO 160
        DB 0,190,"PRINT ",$22,"",$22,$0D                                        ; 190 PRINT ""
        DB 0,192,"PRINT ",$22,"--- FOR/NEXT LOOP ---",$22,$0D                  ; 192 PRINT "--- FOR/NEXT LOOP ---"
        DB 0,194,"FOR I=1 TO 5",$0D                                             ; 194 FOR I=1 TO 5
        DB 0,196,"PRINT I",$3B,$0D                                              ; 196 PRINT I;
        DB 0,198,"NEXT I",$0D                                                   ; 198 NEXT I
        DB 0,199,"PRINT ",$22,"",$22,$0D                                        ; 199 PRINT ""
        DB 0,201,"PRINT ",$22,"--- FOR STEP 2 ---",$22,$0D                     ; 201 PRINT "--- FOR STEP 2 ---"
        DB 0,203,"FOR I=0 TO 10 STEP 2",$0D                                    ; 203 FOR I=0 TO 10 STEP 2
        DB 0,205,"PRINT I",$3B,$0D                                              ; 205 PRINT I;
        DB 0,207,"NEXT I",$0D                                                   ; 207 NEXT I
        DB 0,208,"PRINT ",$22,"",$22,$0D                                        ; 208 PRINT ""
        DB 0,210,"PRINT ",$22,"--- GOSUB/RETURN ---",$22,$0D                   ; 210 PRINT "--- GOSUB/RETURN ---"
        DB 0,212,"GOSUB 530",$0D                                                ; 212 GOSUB 530
        DB 0,214,"GOSUB 530",$0D                                                ; 214 GOSUB 530
        DB 0,216,"PRINT ",$22,"",$22,$0D                                        ; 216 PRINT ""
        DB 0,220,"PRINT ",$22,"--- FUNCTIONS ---",$22,$0D                       ; 220 PRINT "--- FUNCTIONS ---"
        DB 0,222,"PRINT ",$22,"ABS(-7)=",$22,$3B,"ABS(-7)",$3B,$22,"  NEG(7)=",$22,$3B,"NEG(7)",$0D  ; 222 PRINT "ABS(-7)=";ABS(-7);"  NEG(7)=";NEG(7)
        DB 0,224,"PRINT ",$22,"AND(12,10)=",$22,$3B,"AND(12,10)",$3B,$22,"  OR(12,10)=",$22,$3B,"OR(12,10)",$0D  ; 224 PRINT "AND(12,10)=";AND(12,10);"  OR(12,10)=";OR(12,10)
        DB 0,226,"PRINT ",$22,"XOR(12,10)=",$22,$3B,"XOR(12,10)",$3B,$22,"  NOT(0)=",$22,$3B,"NOT(0)",$0D  ; 226 PRINT "XOR(12,10)=";XOR(12,10);"  NOT(0)=";NOT(0)
        DB 0,228,"POKE 8000,42",$0D                                             ; 228 POKE 8000,42
        DB 0,230,"PRINT ",$22,"PEEK(8000)=",$22,$3B,"PEEK(8000)",$0D            ; 230 PRINT "PEEK(8000)=";PEEK(8000)
        DB 0,232,"PRINT ",$22,"RND(100)=",$22,$3B,"RND(100)",$3B,$22,"  RND(100)=",$22,$3B,"RND(100)",$0D  ; 232 PRINT "RND(100)=";RND(100);"  RND(100)=";RND(100)
        DB 0,234,"PRINT ",$22,"",$22,$0D                                        ; 234 PRINT ""
        DB 0,236,"PRINT ",$22,"--- LIST 40,60 ---",$22,$0D                      ; 236 PRINT "--- LIST 40,60 ---"
        DB 0,238,"LIST 40,60",$0D                                               ; 238 LIST 40,60
        DB 0,240,"GOTO 300",$0D                                                 ; 240 GOTO 300
        DB 1,44,"PRINT ",$22,"--- MANDELBROT ---",$22,$0D                      ; 300 PRINT "--- MANDELBROT ---"
        DB 1,54,"I=-64",$0D                                                     ; 310 I=-64
        DB 1,64,"IF I>56 THEN GOTO 510",$0D                                    ; 320 IF I>56 THEN GOTO 510
        DB 1,74,"D=I",$0D                                                       ; 330 D=I
        DB 1,84,"C=-144",$0D                                                    ; 340 C=-144 (widened from -120)
        DB 1,94,"IF C>28 THEN GOTO 480",$0D                                    ; 350 IF C>28 THEN GOTO 480 (widened from 4)
        DB 1,104,"A=C",$0D                                                      ; 360 A=C
        DB 1,105,"B=D",$0D                                                      ; 361 B=D
        DB 1,106,"E=0",$0D                                                      ; 362 E=0
        DB 1,107,"N=1",$0D                                                      ; 363 N=1
        DB 1,114,"IF N>16 THEN GOTO 420",$0D                                   ; 370 IF N>16 THEN GOTO 420
        DB 1,124,"IF E>0 THEN GOTO 410",$0D                                    ; 380 IF E>0 THEN GOTO 410
        DB 1,134,"T=A*A/64-B*B/64+C",$0D                                       ; 390 T=A*A/64-B*B/64+C
        DB 1,144,"B=2*A*B/64+D",$0D                                             ; 400 B=2*A*B/64+D
        DB 1,145,"A=T",$0D                                                      ; 401 A=T
        DB 1,154,"IF A*A/64+B*B/64>256 THEN IF E=0 THEN E=N",$0D               ; 410 IF A*A/64+B*B/64>256 THEN IF E=0 THEN E=N
        DB 1,164,"N=N+1",$0D                                                    ; 420 N=N+1
        DB 1,165,"IF N<=16 THEN GOTO 370",$0D                                  ; 421 IF N<=16 THEN GOTO 370
        DB 1,174,"IF E>0 THEN PRINT CHR$(E+32)",$3B,$0D                        ; 430 IF E>0 THEN PRINT CHR$(E+32);
        DB 1,184,"IF E=0 THEN PRINT CHR$(32)",$3B,$0D                          ; 440 IF E=0 THEN PRINT CHR$(32);
        DB 1,194,"C=C+4",$0D                                                    ; 450 C=C+4
        DB 1,204,"GOTO 350",$0D                                                 ; 460 GOTO 350
        DB 1,224,"PRINT",$0D                                                    ; 480 PRINT
        DB 1,234,"I=I+6",$0D                                                    ; 490 I=I+6
        DB 1,244,"GOTO 320",$0D                                                 ; 500 GOTO 320
        DB 1,254,"END",$0D                                                      ; 510 END
        DB 2,18,"PRINT ",$22,"sub",$22,$3B,$0D                                 ; 530 PRINT "sub";
        DB 2,20,"RETURN",$0D                                                   ; 532 RETURN
SHOWCASE_END:

        END
