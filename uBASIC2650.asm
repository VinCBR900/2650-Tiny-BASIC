; uBASIC2650.asm       Tiny BASIC interpreter for Signetics 2650
; Version: v4.7
; By Vincent Crabtree, 2026.  MIT License
; Date:    2026-08-10
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
;   REC-01:   Triple-nested atom-dispatch (1+ABS(NEG(AND(...)))) hits
;     the existing PE_RAS_LIMIT=5 guard (?8 ERR_NEST).
;
; RECENT CHANGE HISTORY
;
; V4.7 (2026-08-10) - ROMEND $0F98 
;   - PUSH_RET helper: R0=lo,R1=hi 16-bit return-address push onto SWBASE,
;     replacing the repeated LODI/STRA x2 idiom used before every ZBRR.
;   - BUG: indexed autoincrement STRA (abs,R3+) only exists for R0 on
;     the 2650 -- STRA,R1 abs,R3+ silently assembles identical to STRA,R0
;     (same opcode byte) and does NOT store R1. PUSH_RET uses LODZ,R1
;     (R0=R1) before the second store to work around this.
;   - Added ^ (power) operator: CHECK_POW hook, called after every atom-
;     fetch returns (8 sites: EAM0_RET/EAM_P_RET/EAM_M_RET/MU_AT_RET/
;     DV_AT_RET/MD_AT_RET/NEG_AT_RET/POS_AT_RET -- the last needed its own
;     landing label, previously EAM_POS just reused PARSER_RET directly
;     since unary + is a no-op). Left-associative (2^3^2 == (2^3)^2),
;     binds tighter than */÷/% and looser than atoms; unary - binds
;     tighter than ^ (-2^2 == (-2)^2 == 4). 0^0==1. Negative exponent ->
;     ERR_OV ('9', new error code). 
;   - FIXED print variable -32768 (e.g. 2^15), would hang. Root cause
;     (BUG-MINLP-01): the -32768 special-case string-print loop (MIN_LP)
;     used LODR,R0 MSG_MIN-1,R1+ to walk the "32768" literal -- but LODR
;     has no indexed addressing on this CPU 
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
;   - Integrated emulator SP-reset override fix in pipbug_wrap.c.
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
VCHECK_LPAREN:
        DW CHECK_LPAREN          ; 7 sites (v4.5 FUNCCONT-01: ABS/NEG/NOT/
                                  ; PEEK/USR/RND + PARSE_2ARGS golf-down)
VCHECK_RPAREN:
        DW CHECK_RPAREN          ; 6 sites (ABS/NEG/NOT/PEEK/USR/RND)
VFUNC_CONT:
        DW FUNC_EPILOG            ; 5 sites (ABS/NEG/RND + P2A_RET shared by
                                  ; AND/OR/XOR/NOT + EXPH_Z shared by PEEK/USR)
                                  ; v4.6 FUNCATOM-01: routes to FUNC_CONT
                                  ; (top-level) or PARSER_RET (mid-expr)
VPUSH_RET:
        DW PUSH_RET              ; 13 sites (v4.7 PUSHRET-01: replaces the
                                  ; LODI/STRA x2 literal-return-address push
                                  ; idiom in front of ZBRR *VEAM_ATOM/VEAM_HI)
VCHECK_POW:
        DW CHECK_POW             ; 8 sites (v4.7 POW-01: power-operator (^)
                                  ; hook, called after every atom-fetch
                                  ; returns, before the caller's next step)
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

        ; print sign-on 
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
        CPSL PSW_RS + 7             ; primary reg bank; clear PSL CC/flag bits
        CPSU $07                    ; clear PSU SP field (bits 2:0 = HW RAS depth)
                                     ; MUST be separate from CPSL: SP is in PSU not PSL
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

SE_NOTKW:
        ; Keyword table exhausted. Check for bare variable assignment
        ; ("X=expr" with no LET). MATCH_KW's two GETCI_UC calls already
        ; consumed SC0 (letter) and SC1 ('=') from the buffer, so rewind IP
        ; by 2 and fully delegate the letter-validate-and-store step to
        ; PARSE_VAR_SAVE (v4.4.3: was a duplicated inline A-Z range check -
        ; one source of truth now, and net smaller: 22 bytes here + 0 for
        ; the now-dead KWSYNERR trampoline, vs 25+2 before).
        ZBSR *VDEC_IP
        ZBSR *VDEC_IP
        BSTA,UN PARSE_VAR_SAVE            ; validates A-Z, SC0/R2 = letter, IP -> past it
        ZBSR *VWSKIP
        LODA,R0 *IPH
        COMI,R0 A'='
        BCFA,EQ JSYNERR
        ZBSR *VINC_IP
        BCTA,UN DL_EX                    ; expression follows

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
        STRA,R0 FT_SP                    ; FUNCATOM-01 (v4.6): clear dispatch-
                                          ; origin stack (R0 still $FF here)
        STRA,R0 FT_SAVE_SP               ; ...and its byte-save stack too
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
; (DO_PEEK_FUNC/DO_USR_FUNC relocated post-COUT in v4.5, alongside
;  DO_RND_FUNC - see that routine's header for why this constrained
;  pre-CHIN zone is the wrong place for functions needing more bytes for
;  the FUNCCONT-01 paren-bounding fix.)

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
        ZBSR *VCHECK_LPAREN
        ZBSR *VPARSE_EXPR       ; get range in EXP 
        ZBSR *VCHECK_RPAREN
        LODI,R2 8               ; shuffle a full byte's worth of taps
RNDF_MIX:
        ZBSR *VRND_SHUFFLE
        BDRR,R2 RNDF_MIX
        LODA,R0 RNDSEED         ; Copy rnd into TMP
        STRA,R0 TMPH
        LODA,R0 RNDSEED+1       ; 
        STRA,R0 TMPL
        BSTA,UN DIV16          ; Divivide
        ZBSR *VTMP_TO_EXP16    ; Get remainder (v4.5: real call now, was
                                 ; tail-call - need control back for FUNC_CONT)
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
        ; FUNCATOM-01 (v4.6): push a $FF "top-level" origin marker onto
        ; FT_STK before scanning FUNC_TAB. EAM_ATOM's own scan (see below)
        ; pushes the live R3 instead of $FF; PE_NOFUNC/FUNC_EPILOG pop this
        ; to learn which completion path a dispatch needs. R3 is provably
        ; never $FF at the instant EAM_ATOM dispatches (every EAM_ATOM
        ; caller has already pushed something onto SWBASE first), so $FF
        ; is a safe, unambiguous top-level sentinel - no separate flag
        ; byte needed. This correctly nests through recursive PARSE_EXPR
        ; calls (e.g. a function's own argument, itself containing another
        ; function call) since FT_STK/FT_SP mirrors true call nesting.
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
        ZBSR *VDEC_IP
        ZBSR *VDEC_IP
        BSTA,UN PARSE_FACTOR
        BCTA,UN PARSER_RET
PE_NOFUNC_TOP:
        ZBSR *VDEC_IP                   ; backup IP 2 slots from KW Match
        ZBSR *VDEC_IP
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
        ; v4.4.3: PUSH_EXP shared with EAM_MINUS/MUL/DIV/MOD below.
        BSTR,UN PUSH_EXP
        LODI,R0 >EAM_P_RET
        LODI,R1 <EAM_P_RET
        ZBSR *VPUSH_RET
        ZBRR *VEAM_ATOM 
EAM_P_RET:
        ZBSR *VCHECK_POW
        LODI,R0 >ADD16_SAVE_EXP ; >EAM_PH_RET
        LODI,R1 <ADD16_SAVE_EXP ; <EAM_PH_RET
        ZBSR *VPUSH_RET
        ZBRR *VEAM_HI 
;EAM_PH_RET:
;        BCTR,UN ADD16_SAVE_EXP            ; EXP = SAVE + EXP (V4.2: shared)
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
        ; v4.4.3 BUG-PUSHEXP-01 fix: this MUST be an explicit jump, not a
        ; comment-only "drop through" - PUSH_EXP's body now sits physically
        ; between here and ADD16_SAVE_EXP (see below), so falling through
        ; would run PUSH_EXP's code (spurious SWBASE push + stray INC_IP)
        ; instead of the actual add. Confirmed via simulator: "10-3" gave
        ; -3 (the negated RHS alone, LHS never added) before this fix.
; =============================================================================
;  PUSH_EXP -- push EXPH:EXPL onto SWBASE (lo,hi) and consume the operator
;  char at IP (v4.4.3). Shared by EAM_PLUS/EAM_MINUS/EAM_MUL/EAM_DIV/EAM_MOD,
;  replacing 5 copies of the same 5-instruction block. INC_IP is tail-called
;  (ZBRR, not ZBSR) so it doesn't cost PUSH_EXP its own extra RAS frame; and
;  PUSH_EXP itself fully resolves and returns *before* its caller's
;  recursive ZBRR *VEAM_ATOM, so - like the INC_IP calls already at every
;  one of these sites - it doesn't accumulate hardware RAS depth across
;  expression nesting levels (verified empirically: 14 levels of "(((1+1)
;  +1)...)" nesting still runs clean after this change).
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
;  PUSH_RET -- push a 16-bit literal return address onto SWBASE (v4.7,
;  PUSHRET-01). Replaces the 4-instruction LODI/STRA x2 idiom that preceded
;  every ZBRR *VEAM_ATOM/*VEAM_HI dispatch with a 2-instruction load + one
;  shared call. Caller loads R0=lo, R1=hi of the target label, ZBSR's here,
;  then falls through to its own ZBRR *VTARGET. Push order (lo then hi)
;  matches SWRETURN's pop order (hi popped first, top of stack).
;  NOTE: indexed STRA abs,R3+ only exists for R0 (2650 addressing constraint,
;  confirmed against the ISA doc) -- STRA,R1 SWBASE,R3+ silently assembles
;  identically to STRA,R0 and does NOT store R1. LODZ,R1 (R0=R1) is used to
;  bring the high byte into R0 before the second store.
;  Net: 12 bytes/site inline -> 8 bytes/site + this 8-byte routine once.
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
        BCTR,EQ EAM_MOD
        BCTA,UN PARSER_RET
EAM_MUL:
        ; PAREN-NEST-01 fix (v4.3): push, not flat E1SAVH:E1SAVL (see EAM_PLUS)
        BSTA,UN PUSH_EXP
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
        BSTA,UN PUSH_EXP
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
        BSTA,UN PUSH_EXP
        LODI,R0 >MD_AT_RET
        LODI,R1 <MD_AT_RET
        ZBSR *VPUSH_RET
        ZBRR *VEAM_ATOM 
MD_AT_RET:
        ZBSR *VCHECK_POW
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
        BCTA,EQ EAM_NEG
        COMI,R0 A'+'
        BCTA,EQ EAM_POS
        COMI,R0 A'('
        BCTA,EQ EAM_PAREN
        ; FUNCATOM-01 fix (v4.6): also scan FUNC_TAB here so functions are
        ; recognised as non-leading atoms too (e.g. "10+ABS(A)"). Push the
        ; live R3 (SWBASE pointer - the pending outer operator context)
        ; onto FT_STK, then byte-copy the live SWBASE[0..R3] region onto
        ; FT_SAVE. The copy is required: R3 alone isn't enough to protect
        ; the outer context, because on a HIT the handler's own argument
        ; parse (ZBSR *VPARSE_EXPR) resets R3=$FF and regrows SWBASE from
        ; index 0 upward, physically overwriting the very bytes R3 points
        ; at. FUNC_EPILOG copies them back. On a MISS, PE_NOFUNC discards
        ; this save unread (R3/SWBASE were never touched by a plain scan
        ; attempt) - see PE_NOFUNC. No HW call/RETC used anywhere in this
        ; block: it's all inline, staying in the SW-stack domain like the
        ; rest of the expression parser (the HW RAS is scarce - 8 slots,
        ; 3 already spoken for by the main loop/char IO).
        ; Clobbers: R0, R1, R2 (saved/restored - reserved for DO_LET et al
        ; across PARSE_EXPR, see header). R3 is read but never written here.
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
        STRZ,R1                          ; R1 = N+1 (BDRR loop count ONLY -
                                          ; confirmed via 2650.c: BDRR,rn
                                          ; decrements once then loops while
                                          ; the RESULT IS NONZERO, i.e. an
                                          ; initial value of V gives exactly
                                          ; V passes, not V+1 - so R1 must
                                          ; NOT double as the data index or
                                          ; the last element (index 0) is
                                          ; silently dropped. R3/R2 below
                                          ; are stepped explicitly instead.)
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
        LODA,R0 FT_N                      ; reload N (stashed above - R1 no
                                          ; longer holds the level index,
                                          ; it was repurposed as BDRR count)
        STRZ,R3                          ; restore R3=N (the loop above used
                                          ; R3 as its working SWBASE index and
                                          ; left it at -1/$FF - R3 is the LIVE
                                          ; SWBASE pointer everything past
                                          ; this point depends on, so it MUST
                                          ; come back to N here)
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
; Out: IP advanced past '('; tail-jumps to JSYNERR (no return) if missing
; Clobbers: R0
CHECK_LPAREN:
        ZBSR *VWSKIP
        LODA,R0 *IPH
        COMI,R0 A'('
        BCFA,EQ JSYNERR
        ZBRR *VINC_IP            ; tail call: consumes '(' and returns to caller

; =============================================================================
;  CHECK_RPAREN -- require and consume ')' at IP (v4.5, FUNCCONT-01 fix)
; Out: IP advanced past ')'; tail-jumps to JSYNERR (no return) if missing
; Clobbers: R0
CHECK_RPAREN:
        ZBSR *VWSKIP
        LODA,R0 *IPH
        COMI,R0 A')'
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
        STRZ,R1                           ; R1 = N+1 (BDRR loop count ONLY -
                                           ; see EAM_ATOM's note on BDRR's
                                           ; confirmed decrement-then-loop-
                                           ; while-nonzero semantics)
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
        BCTA,UN PARSER_RET
FUNC_CONT:
        LODI,R3 $FF
        BCTA,UN EAM0_RET

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
;  downstream depends on R1/R2/R3 surviving a function call. (This is also
;  why FUNCATOM-01 - functions only matching as the LEADING atom, see
;  KNOWN OPEN ITEMS - remains unfixed rather than a quirk of this dispatch
;  scheme specifically: a fix would need EAM_ATOM to run this exact same
;  dispatch for non-leading atoms too, which was attempted and reverted.)
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
        ZBSR *VCHECK_LPAREN              ; require+consume '(' (v4.5: was
                                          ; inline WSKIP/COMI/BCFA/INC_IP,
                                          ; now shared with the single-arg
                                          ; functions' FUNCCONT-01 fix)
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
        ZBRR *VFUNC_CONT      ; v4.5 FUNCCONT-01 fix: was RETC,UN. Fixes
                               ; AND/OR/XOR/NOT continuation simultaneously
                               ; (all 4 share this tail).
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
        ; v4.7 BUG-MINLP-01 fix: was LODR,R0 MSG_MIN-1,R1+ -- LODR has no
        ; indexed addressing on this CPU (oracle: "LODR,rn rel ; rn=*(rel)",
        ; a bare PC-relative single-byte load, no register-index form at
        ; all). The ,R1+ was silently accepted by the assembler and
        ; discarded (assembled to LODR's normal 2-byte encoding, nothing
        ; more), so R1 never advanced and the loop re-read the same fixed
        ; byte forever -- an infinite ZBSR *VCOUT loop whenever this path
        ; was reached. LODA,r0 abs,x+ is the correct indexed-with-
        ; autoincrement form (r0 is mandatory for the data register in
        ; this mode -- same restriction found earlier for STRA's indexed
        ; form in PUSH_RET). It's pre-increment (x++ happens BEFORE
        ; addressing, confirmed against the oracle and against how
        ; SWBASE push/pop already uses R3=$FF for the same reason), so
        ; R1 starts at $FF here too, landing on MSG_MIN+0 on the first
        ; actual read -- no "-1" address adjustment needed.
        LODI,R1 $FF              ; string offset (pre-increment convention)
MIN_LP:
        LODA,R0 MSG_MIN,R1+     ; Load char from table
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
        LODI,R1 <PR_PRINT
        ZBSR *VPUSH_RET
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
        DB CR, LF, "uBASIC 2650 V4.8", CR, LF, "Bytes Free:",NUL

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
        BSTA,UN PUSH_EXP           ; push base(EXPH:EXPL) onto SWBASE; tail
                                    ; branch inside also advances IP past '^'
        LODI,R0 >CP_EXP_RET
        LODI,R1 <CP_EXP_RET
        ZBSR *VPUSH_RET
        ZBRR *VEAM_ATOM             ; parse exponent atom -> EXPH:EXPL
CP_EXP_RET:
        LODA,R0 SWBASE,R3          ; base hi (top, no dec)
        STRA,R0 SAVEH
        LODA,R0 SWBASE,R3-         ; base lo
        STRA,R0 SAVEL
        SUBI,R3 1
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
        BCTA,UN CP_CHAIN

CP_POW_LOOP:
        LODA,R0 EXPH                ; move exponent into the down-counter
        STRA,R0 POWCNTH             ; (EXPH:EXPL becomes the result accum
        LODA,R0 EXPL                 ; below, needs the exponent out of it)
        STRA,R0 POWCNTL

        LODI,R0 0                    ; result accumulator = 1
        STRA,R0 EXPH
        LODI,R0 1
        STRA,R0 EXPL

CP_MUL_LOOP:
        LODA,R0 SAVEH
        STRA,R0 TMPH
        LODA,R0 SAVEL
        STRA,R0 TMPL
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
POWCNTH RES 1       ; CHECK_POW: exponent down-counter (hi/lo); accessed by
POWCNTL RES 1       ; name throughout, not indexed, so adjacency doesn't matter
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
        DB 0,10,"REM uBASIC 2650 - SHOWCASE V4.6",$0D
        DB 0,20,"PRINT ",$22,"-- uBASIC 2650 V4.6 Showcase --",$22,$0D
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
