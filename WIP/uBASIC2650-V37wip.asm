; uBASIC2650.asm       Tiny BASIC interpreter for Signetics 2650
; Version: v3.7
; By Vincent Crabtree, 2026.  MIT License
; Date:    2026-06-14
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
;   R2  long-lived variable letter (DO_LET/DO_INPUT/DO_FOR, preserved across PARSE_EXPR).
;       Never written by subroutines except DO_LET, SE_BAREASS, DO_FOR.
;   R3  loop counter (BDRR/BIRR); STORE_LINE shift count. SW expr-stack pointer.
;
;        KNOWN OPEN ITEMS
;   COLON-01: ':' multi-statement separator not supported.
;   OPT-16:   MUL16/DIV16 naive O(N) loop -- O(16) bit-serial deferred.
;   OPT-FOR:  DF_PUSH loop-based frame write 
;   FOR-01:   NEXT variable not checked against frame var (smallest code, by spec).
;   FOR-02:   Body always executes at least once (no skip-if-false-at-entry, by spec).
;
;        CHANGE HISTORY
;   V3.7  2026-06-14  Interpreter: 3593 bytes
;         Page-zero vector table + Zxxx size optimisation 
;         DO_NEW memory clear.
;         SWSTK RES 1->2: fixed SWSTK+1/RELOP aliasing bug (v36d fix).
;         --crlf must be PRT_CRLF from .sym (not old PIPBUG $008A).
;
;   V3.6G 2026-06-12  Interpreter: 3700 bytes
;         Code ORG now 0x0, added CHIN/COUT routines from Pipbug M20 App note,
;           hand-optimized placement to $286/$2B4 for PIPBUG 1 compatibility.
;         Refactor DO_ERROR to use DO_END.  Refactor DO_FOR for size.
;         Helpers: ADV_TMP_BY_R0, FI_ADV, SETUP_MULDIV, PARSE_VAR_SAVE.
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

; PSW Defines
PSW_RS          EQU     $10
PSW_WC          EQU     $08             ; WC (With Carry) bit in PSL (bit 3)
PSW_FLAG        EQU     $40

; System Defines
PROGLIM         EQU $1FFF   ; top of program store (numeric constant, not address)
;  GOSUB stack (v3.2) -- managed by SWSP
; Frame = [lo][hi] of NLP. SWSP=$FF=empty. 2 bytes/frame, 8 frames.
GSSTKLIM EQU $0F    ; max SWSP before overflow (numeric constant, not address)
;  FOR/NEXT stack (v3.3) -- managed by FORSP
; Frame (7 bytes): [var][limH][limL][stpH][stpL][nlpH][nlpL]
;   var=letter A-Z, lim=signed limit, stp=signed step, nlp=loop-back address.
; FORSP=$FF=empty. Offsets: 0/7/14/21 for frames 1-4. 4 frames = 28 bytes.
; Overflow: FORSP >= FORSTKLIM before push -> ERR_FOR.
FORSTKLIM EQU $15   ; max FORSP before overflow (numeric constant, not address)

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
        DW INC_IP               ; $0002  ZBSR *VINC_IP
VWSKIP:
        DW WSKIP                ; $0004  ZBSR *VWSKIP
VINC_TMP:
        DW INC_TMP              ; $0006  ZBSR *VINC_TMP
VCOUT:
        DW COUT                 ; $0008  ZBSR *VCOUT
VPARSE_EXPR:
        DW PARSE_EXPR           ; $000A  ZBSR *VPARSE_EXPR
VGETCI_UC:
        DW GETCI_UC             ; $000C  ZBSR *VGETCI_UC
VEATWORD:
        DW EATWORD              ; $000E  ZBSR *VEATWORD
VSET_IP_IBUF:
        DW SET_IP_IBUF          ; $0010  ZBSR *VSET_IP_IBUF
VPRT_SPACE:
        DW PRT_SPACE            ; $0012  ZBSR *VPRT_SPACE
VINC_EXP:
        DW INC_EXP              ; $0014  ZBSR *VINC_EXP
VEAM_ATOM:
        DW EAM_ATOM             ; $0016  ZBRR *VEAM_ATOM
VEAM_HI:
        DW EAM_HI               ; $0018  ZBRR *VEAM_HI
VDO_ERROR:
        DW DO_ERROR             ; $001A  ZBRR *VDO_ERROR
VJSYNERR:
        DW JSYNERR              ; $001C  ZBRR *VJSYNERR
VDR_LP:
        DW DR_LP                ; $001E  ZBRR *VDR_LP
VCLR_RUNFLG:
        DW CLR_RUNFLG           ; $0020  ZBRR *VCLR_RUNFLG
MAIN:
        CPSL $FF                ; clear PSL: CC=EQ, C=0, RS=0, SP=0

        ; Pre-load SHOWCASE_END as program so RUN executes the showcase.
        ; Change BSTA DO_END to BSTA DO_NEW below to start with empty program.
        LODI,R0 <SHOWCASE_END
        STRA,R0 PEH
        LODI,R0 >SHOWCASE_END
        STRA,R0 PEL

        BSTA,UN DO_END          ; clear RUNFLG, SWSP, FORSP, GOTOFLG

        ; clear A-Z variables (52 bytes) 
        LODI,R3 51       ; Loop bounds: 51 down to 0 (52 total bytes)
        EORZ,R0          ; Clear R0 (Stays zero; STRA doesn't alter ALU states)
CLRV:
        STRA,R0 VARS,R3  ; Clear target index byte directly
        BDRR,R3 CLRV     ; Decrement R3 and loop until underflow to $FF

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
        LODI,R0 '>'                    ; print prompt 
        ZBSR *VCOUT  
        ZBSR *VPRT_SPACE  
        BSTA,UN RDLINE
        ZBSR *VSET_IP_IBUF                ; IPH:IPL = IBUF
        BSTA,UN TRY_STORE_LINE           ; CC=GT: line stored/deleted; CC=EQ: not a line
        BCTR,GT REPL                     ; line stored: back to prompt
        BSTR,UN STMT_EXEC
        BCTR,UN REPL

; =============================================================================
;  STMT_EXEC -- Decode and dispatch one BASIC statement from IP.
; KW_TAB format: [c1][c2][c3][hi][lo], stride 5.
;   c3=A' ' (space) = wildcard (short keywords: IF).
;   c3 is peeked from *IPH uppercase without consuming; EATWORD cleans up.
; In:  IPH:IPL -> first char of statement (after any leading whitespace)
; Out: handler called; IP advanced past statement
; Clobbers: R0, SC0, SC1, TMPH, TMPL, EXPH, EXPL, GOTOH, GOTOL
; RAS depth: 1 from REPL, 3 from DO_IF(THEN body).
; Worst inner depth from here: +4 (DO_xxx->PARSE_EXPR->PARSE_FACTOR->UPCASE)
STMT_EXEC:
        ZBSR *VWSKIP                      ; [+1]
        LODA,R0 *IPH
        RETC,EQ                          ; blank line: return

        ZBSR *VGETCI_UC  
        STRA,R0 SC0                      ; char1 uppercase, IP advanced
        ZBSR *VGETCI_UC  
        STRA,R0 SC1                      ; char2 uppercase, IP advanced

        ; scan KW_TAB with TMPH:TMPL as pointer
        LODI,R0 <KW_TAB
        STRA,R0 TMPH
        LODI,R0 >KW_TAB
        STRA,R0 TMPL
SE_SCAN:
        LODA,R0 *TMPH                    ; c1
        BCTA,EQ SE_NOTKW                 ; end of table: check bare assignment
        SUBA,R0 SC0
        BCTR,EQ SE_CHK2
        ; c1 mismatch: advance 5 bytes to next entry
        LODI,R0 5
        BSTA,UN ADV_TMP_BY_R0
        BCTR,UN SE_SCAN
SE_CHK2:
        ZBSR *VINC_TMP                    ; point to c2 byte
        LODA,R0 *TMPH
        SUBA,R0 SC1
        BCTR,EQ SE_CHK3
        ; c2 mismatch: advance remaining 4 bytes
        LODI,R0 4
        BSTA,UN ADV_TMP_BY_R0
        BCTR,UN SE_SCAN
SE_CHK3:
        ; c1+c2 matched. Read table c3.
        ; If table c3 == A' ': wildcard, accept without consuming input c3.
        ; Otherwise: peek *IPH uppercase, compare; IP not advanced (EATWORD handles rest).
        ; BUG-SE-01 FIX: c3-mismatch stride must be +3 (c3+hi+lo -> next c1), not +2.
        ZBSR *VINC_TMP                    ; point to c3 byte in table
        LODA,R0 *TMPH
        COMI,R0 A' '
        BCTR,EQ SE_MATCH                 ; wildcard: accept
        STRA,R0 EXPL                     ; save table-c3 in EXPL (scratch)
        LODA,R0 *IPH                     ; peek input stream c3 (do NOT advance IP)
        BSTA,UN UPCASE                   ; [+1]
        SUBA,R0 EXPL
        BCTR,EQ SE_MATCH                 ; c3 matched
        ; c3 mismatch: advance remaining 3 bytes (c3+hi+lo -> next c1)
        LODI,R0 3
        BSTA,UN ADV_TMP_BY_R0
        BCTA,UN SE_SCAN
SE_MATCH:
        ZBSR *VEATWORD                    ; [+1] consume remaining alpha chars
        ; load handler address from next 2 bytes: [hi][lo]
        ZBSR *VINC_TMP  
        LODA,R0 *TMPH
        STRA,R0 EXPH                     ; handler hi
        ZBSR *VINC_TMP  
        LODA,R0 *TMPH
        STRA,R0 EXPL                     ; handler lo
        BSTA,UN EXP16_TO_GOTO            ; GOTOH:GOTOL = EXPH:EXPL (handler address)
        BCTA,UN *GOTOH                   ; indirect jump to handler

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
        BCTR,EQ SE_BAREASS
KWSYNERR:
        ZBRR *VJSYNERR 
SE_BAREASS:
        LODA,R0 SC0
        STRZ,R2                          ; save letter in R2 (survives PARSE_EXPR)
        BCTA,UN DL_EX                    ; IP already past '=', expression follows

; =============================================================================
;  DO_NEW -- Clear program store
; Syntax: NEW
; In:  nothing
; Out: PEH:PEL = PROG; program store ($10E0-$1FFF) zeroed; falls through to DO_END
; Clobbers: R0, R1, IPH, IPL, SWSP, FORSP, GOTOFLG, RUNFLG
DO_NEW:
        ; Zero program store using IPH:IPL as write pointer.
        ; R1 = 0 
        ; Stop when IPH reaches $20 (i.e. address wrapped past $1FFF).
        LODI,R0 <PROG
        STRA,R0 IPH
        LODI,R0 >PROG
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
        LODI,R0 $FF
        STRA,R0 SWSP                     ; clear GOSUB stack
        STRA,R0 FORSP                    ; clear FOR stack
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
        BSTA,UN DEC_IP          ; fall through to DP_EXPR

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
        RETC,EQ
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
        BSTR,UN EXP16_TO_GOTO            ; GOTOH:GOTOL = EXPH:EXPL
        LODI,R0 1
        STRA,R0 GOTOFLG
        LODA,R0 RUNFLG                   ; OPT-10
        RETC,GT                          ; return if running
        ZBRR *VCLR_RUNFLG 

; =============================================================================
;  EXP16_TO_GOTO -- Copy EXPH:EXPL to GOTOH:GOTOL
; In:  EXPH:EXPL = 16-bit value
; Out: GOTOH = EXPH, GOTOL = EXPL
; Clobbers: R0
; Used by: SE_MATCH (indirect branch target), DO_GOTO, DO_GOSUB.
EXP16_TO_GOTO:
        LODA,R0 EXPH
        STRA,R0 GOTOH
        LODA,R0 EXPL
        STRA,R0 GOTOL
        RETC,UN 

; =============================================================================
;  EXP16_TO_LNUM -- Copy EXPH:EXPL to LNUMH:LNUML
; In:  EXPH:EXPL = 16-bit value
; Out: LNUMH = EXPH, LNUML = EXPL
; Clobbers: R0
; Used by: DO_IF (save left operand), DO_FOR (save limit), DR_GOTO (load target).
EXP16_TO_LNUM:
        LODA,R0 EXPH
        STRA,R0 LNUMH
        LODA,R0 EXPL
        STRA,R0 LNUML
        RETC,UN

; =============================================================================
;  DO_IF -- Conditional execution
; Syntax: IF expr relop expr THEN stmt
; In:  IP -> first char after IF keyword
; Out: executes stmt if condition true; otherwise sequential return
; Clobbers: R0, R1, EXPH, EXPL, LNUMH, LNUML, SC0, SC1, RELOP
; RAS: entry+1(PE)+1(PR)+1(PE)+1(SE) = entry+4. Max depth 7: ok.
DO_IF:
        ZBSR *VPARSE_EXPR                 ; [+1]
        BSTR,UN EXP16_TO_LNUM            ; LNUMH:LNUML = EXPH:EXPL (save left operand)
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
        BCTR,UN DIF_ANDTEST
DIF_IS_LT:
        LODI,R0 1                        ; LT
        BCTR,UN DIF_ANDTEST
DIF_IS_EQ:
        LODI,R0 2                        ; EQ
DIF_ANDTEST:
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
        LODA,R0 SWSP
        COMI,R0 $FF
        BCTA,EQ DRT_UNDERFLOW
        LODA,R1 SWSP
        ADDI,R1 1
        LODA,R0 GSBASE,R1                ; hi byte
        STRA,R0 GOTOH
        LODA,R1 SWSP
        LODA,R0 GSBASE,R1                ; lo byte
        STRA,R0 GOTOL
        ; decrement SWSP: 0 -> $FF (empty), else -= 2
        LODA,R0 SWSP
        BCTR,EQ DRT_WAS_ZERO
        SUBI,R0 2
        STRA,R0 SWSP
        BCTR,UN DRT_GO
DRT_WAS_ZERO:
        LODI,R0 $FF
        STRA,R0 SWSP
DRT_GO:
        LODI,R0 3                        ; GOTOFLG=$03 = direct NLP (no FIND_LINE)
        STRA,R0 GOTOFLG
        RETC,UN

; =============================================================================
; Character IO
; 110 Baud teletype from PIPBUG V1 as per Signetics M20 application note
; CHIN must be at $286, COUt must be at $2B4 for Pipbug 1 compatability
      ;  ORG $286
CHIN:
        PPSL PSW_RS
        LODI,R0 $80
        WRTC,R0
        LODI,R1 0
        LODI,R2 8
ACHI:   
        SPSU
        BCTR,LT CHIN
        EORZ,R0
        WRTC,R0
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
ZERO:
        BDRR,R1 ACOU
        BSTR,UN DLAY
        PPSU PSW_FLAG
        CPSL PSW_RS
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
;  DO_LIST -- Print all stored BASIC lines
; Syntax: LIST
; In:  PROG=program base, PEH:PEL=program end
; Out: all lines printed
; Clobbers: R0, R1, IPH, IPL, LNUMH, LNUML, TMPH, TMPL
DO_LIST:
        LODI,R0 <PROG
        STRA,R0 IPH
        LODI,R0 >PROG
        STRA,R0 IPL
DLS_LP:
        LODA,R0 IPH
        SUBA,R0 PEH
        RETC,GT
        BCTR,LT DLS_BODY
        LODA,R0 IPL
        SUBA,R0 PEL
        TPSL $01
        RETC,EQ
DLS_BODY:
        LODA,R0 *IPH
        STRA,R0 EXPH
        ZBSR *VINC_IP  
        LODA,R0 *IPH
        STRA,R0 EXPL
        ZBSR *VINC_IP  
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
        ZBSR *VINC_IP                     ; skip past CR
        BSTA,UN PRT_CRLF
        BCTR,UN DLS_LP

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
        ADDI,R1 1
        STRA,R0 GSBASE,R1                ; GSBASE[SWSP+1] = hi
        BSTA,UN EXP16_TO_GOTO            ; GOTOH:GOTOL = EXPH:EXPL (target line)
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
; Parse strategy (v3.3 OPT-F2/F3/F4):
;   GETCI_UC skips whitespace and reads char -- used for '=' (saves WSKIP+INC_IP).
;   Limit stored in LNUMH:LNUML (avoids FORLIMH/FORLIML scratch cells).
;   Step stays in EXPH:EXPL until DF_PUSH (avoids FORSTPH/FORSTPL scratch cells).
;   GETCI_UC peeks first char after limit; if 'S' -> STEP; else DEC_IP and default.
; Push strategy (v3.4 OPT-DF_PUSH):
;   R1 = FORSP as walking index into FORBASE; FORBASE,R1 indexed addressing.
;   No INC_TMP chain; saves ~21 bytes vs v3.3.
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
        BSTA,UN EXP16_TO_LNUM            ; LNUMH:LNUML = EXPH:EXPL (limit)
        ; --- check for STEP keyword ---
        ; OPT-F3: GETCI_UC peeks first non-space char.
        ;   If 'S': consume rest of "STEP" with EATWORD, then parse step.
        ;   Else: DEC_IP to un-consume, use default step = +1.
        ZBSR *VGETCI_UC                   ; [+1] R0 = first non-space char (consumed)
        COMI,R0 A'S'
        BCTR,EQ DF_STEP
        ; not 'S': put char back, use step=+1
        BSTA,UN DEC_IP                   ; [+1]
        EORZ,R0
        STRA,R0 EXPH                     ; step hi = 0
        LODI,R0 1
        STRA,R0 EXPL                     ; step lo = 1
        BCTR,UN DF_PUSH
DF_STEP:
        ZBSR *VEATWORD                    ; [+1] consume "TEP"
        ZBSR *VPARSE_EXPR                 ; [+1] step -> EXPH:EXPL
DF_PUSH:
        ; Push 7-byte frame at FORBASE[FORSP] using R1 as walking index.
        ; Sources: FORVAR, LNUMH, LNUML, EXPH, EXPL, SWSTK, SWSTK+1.
        LODA,R1 FORSP                    ; R1 = frame base offset
        LODA,R0 FORVAR
        STRA,R0 FORBASE,R1               ; [0] var
        ADDI,R1 1
        LODA,R0 LNUMH
        STRA,R0 FORBASE,R1               ; [1] limH
        ADDI,R1 1
        LODA,R0 LNUML
        STRA,R0 FORBASE,R1               ; [2] limL
        ADDI,R1 1
        LODA,R0 EXPH
        STRA,R0 FORBASE,R1               ; [3] stpH  (OPT-F4: direct from EXPH)
        ADDI,R1 1
        LODA,R0 EXPL
        STRA,R0 FORBASE,R1               ; [4] stpL  (OPT-F4: direct from EXPL)
        ADDI,R1 1
        LODA,R0 SWSTK
        STRA,R0 FORBASE,R1               ; [5] nlpH
        ADDI,R1 1
        LODA,R0 SWSTK+1
        STRA,R0 FORBASE,R1               ; [6] nlpL
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
        ADDI,R1 1
        LODA,R0 FORBASE,R1               ; frame[4] = stpL
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
        ADDI,R1 1
        STRA,R0 VARS,R1                  ; write var lo

        ; --- signed 16-bit compare: var vs limit ---
        ; Read limit from frame[1:2]
        LODA,R1 FORSP
        ADDI,R1 1
        LODA,R0 FORBASE,R1               ; frame[1] = limH
        STRA,R0 SC0                      ; SC0 = limH (biased below)
        ADDI,R1 1
        LODA,R0 FORBASE,R1               ; frame[2] = limL
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
        ADDI,R1 1
        LODA,R0 FORBASE,R1               ; frame[6] = nlpL
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
;  SET_IP_IBUF -- Set IPH:IPL = IBUF base address
; In:  nothing
; Out: IPH = <IBUF, IPL = >IBUF
; Clobbers: R0
; Used by: REPL, DO_INPUT, DR_EXEC (x2), RDLINE.
; RAS: all call sites depth <= 3. Safe (guard fires at 5).
SET_IP_IBUF:
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
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
        LODI,R0 <PROG
        STRA,R0 TMPH
        LODI,R0 >PROG
        STRA,R0 TMPL
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
        BSTA,UN EXP16_TO_LNUM            ; LNUMH:LNUML = GOTOH:GOTOL (target line)
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
        BSTA,UN PARSE_U16                ; [+1]
        LODA,R0 EXPH
        BCTR,GT TSL_NZ
        LODA,R0 EXPL
        BCTR,EQ TSL_NO                   ; line number zero: not stored
TSL_NZ:
        LODA,R0 EXPH
        STRA,R0 LNUMH
        LODA,R0 EXPL
        STRA,R0 LNUML
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
        LODA,R0 IPH
        STRA,R0 TMPH
        LODA,R0 IPL
        STRA,R0 TMPL
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
        LODA,R0 TMPH
        STRA,R0 EXPH
        LODA,R0 TMPL
        STRA,R0 EXPL                     ; save insertion point in EXPH:EXPL

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
        ADDA,R0 SC1                      ; R0 = PEL + SC1
        STRA,R0 GOTOL
        LODA,R0 PEH
        TPSL $01
        BCTR,LT SL_DNC
        ADDI,R0 1
SL_DNC:
        STRA,R0 GOTOH

SL_SHLOOP:
        ; stop when src == insertion point
        LODA,R0 LNUMH
        SUBA,R0 EXPH
        BCTR,GT SL_DOMOV
        BCTR,LT SL_NOSHIFT               ; BUG-LE FIX: src.hi < ins.hi: done
        LODA,R0 LNUML
        SUBA,R0 EXPL
;        BCTR,EQ SL_NOSHIFT               ; src == ins: done
;        BCTR,LT SL_NOSHIFT               ; BUG-LE FIX: src.lo < ins.lo: done
        BCFR,GT SL_NOSHIFT               ; BUG-LE FIX: src.lo < ins.lo: done

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
        LODA,R0 IPH
        STRA,R0 TMPH
        LODA,R0 IPL
        STRA,R0 TMPL
        LODA,R0 LNUMH
        STRA,R0 *EXPH                    ; write line hi
        ZBSR *VINC_EXP  
        LODA,R0 LNUML
        STRA,R0 *EXPH                    ; write line lo
        ZBSR *VINC_EXP  
SL_WBODY:
        LODA,R1 *TMPH
        BCTR,EQ SL_WDONE
        BSTR,UN TMP2EXP
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
;  ADV_TMP_BY_R0 -- Advance TMPH:TMPL by R0 bytes with 16-bit carry
; In:  R0 = number of bytes to advance; TMPH:TMPL = pointer
; Out: TMPH:TMPL += R0 (carry propagated to hi byte)
; Clobbers: R0
; Note: used by STMT_EXEC mismatch strides (3/4/5) and FIND_INS FI_ADV (stride 2).
;   BSTA call costs 1 extra RAS vs former inline code; all call sites are in
;   deterministic line-handling paths (not deep expression recursion). Safe.
ADV_TMP_BY_R0:
        ADDA,R0 TMPL
        STRA,R0 TMPL
        TPSL $01                         ; test carry from lo-byte add
        RETC,LT                          ; CC=LT means C=0: no carry, done
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
        RETC,UN

; =============================================================================
;  TMP2EXP -- Copy single byte: *EXP++ = *TMP++
; In:  TMPH:TMPL -> source, EXPH:EXPL -> dest
; Out: one byte copied; both pointers incremented
; Clobbers: R0, R1
TMP2EXP:
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
        BCTR,EQ DL2_FOUND
        RETC,UN                          ; not found: silent return
DL2_FOUND:
        LODA,R0 TMPH
        STRA,R0 EXPH
        LODA,R0 TMPL
        STRA,R0 EXPL                     ; save record start
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
        TPSL $01
        BCTR,EQ DL2_DONE
DL2_MOV:
        BSTA,UN TMP2EXP
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
        LODI,R0 <PROG
        STRA,R0 TMPH
        LODI,R0 >PROG
        STRA,R0 TMPL
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
        LODI,R0 2
        BSTA,UN ADV_TMP_BY_R0            ; advance TMPH:TMPL by 2 (past line number bytes)
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
; Clobbers: R0, R3, SAVEH, SAVEL, E1SAVH, E1SAVL, NEGFLG, SC0, SC1, TMPH, TMPL
; RAS guard (v3.2): SPSU/ANDI/COMI fires ERR_NEST if SP>=5 at entry.
;   Inline (no BSTA): guard costs 0 RAS slots. Threshold 5: at SP=5, inner
;   calls (PARSE_FACTOR+PARSE_S16+inline INC_IP) would push SP to 7+, overflow.
PARSE_EXPR:
        ; RAS guard: does not consume a RAS slot.
        SPSU                             ; R0 = PSU; SP in bits 2:0
        ANDI,R0 $07                      ; isolate SP field
        COMI,R0 5                        ; threshold
        BCTR,LT PE_SAFE                  ; SP < 5: safe to proceed
        LODI,R0 ERR_NEST
        ZBRR *VDO_ERROR                  ; abort gracefully
PE_SAFE:
        LODI,R3 $FF                      ; SW stack empty sentinel
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
        ZBSR *VWSKIP  
        LODA,R0 *IPH
        COMI,R0 A'+'
        BCTR,EQ EAM_PLUS
        COMI,R0 A'-'
        BCTA,EQ EAM_MINUS
        BCTA,UN PARSER_RET
EAM_PLUS:
        LODA,R0 EXPH
        STRA,R0 SAVEH
        LODA,R0 EXPL
        STRA,R0 SAVEL
        ZBSR *VINC_IP  
        LODI,R0 >EAM_P_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <EAM_P_RET
        STRA,R0 SWBASE,R3+
        ZBRR *VEAM_ATOM 
EAM_P_RET:
        LODI,R0 >EAM_PH_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <EAM_PH_RET
        STRA,R0 SWBASE,R3+
        ZBRR *VEAM_HI 
EAM_PH_RET:
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
EAM_MINUS:
        LODA,R0 EXPH
        STRA,R0 SAVEH
        LODA,R0 EXPL
        STRA,R0 SAVEL
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
        CPSL PSW_WC
        LODA,R0 SAVEL
        SUBA,R0 EXPL
        STRA,R0 EXPL
        PPSL PSW_WC
        LODA,R0 SAVEH
        SUBA,R0 EXPH
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
        LODA,R0 EXPH
        STRA,R0 E1SAVH
        LODA,R0 EXPL
        STRA,R0 E1SAVL
        ZBSR *VINC_IP  
        LODI,R0 >MU_AT_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <MU_AT_RET
        STRA,R0 SWBASE,R3+
        ZBRR *VEAM_ATOM 
MU_AT_RET:
        LODA,R0 E1SAVH
        STRA,R0 TMPH
        LODA,R0 E1SAVL
        STRA,R0 TMPL
        BSTA,UN MUL16
        ZBRR *VEAM_HI 
EAM_DIV:
        LODA,R0 EXPH
        STRA,R0 E1SAVH
        LODA,R0 EXPL
        STRA,R0 E1SAVL
        ZBSR *VINC_IP  
        LODI,R0 >DV_AT_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <DV_AT_RET
        STRA,R0 SWBASE,R3+
        ZBRR *VEAM_ATOM 
DV_AT_RET:
        LODA,R0 E1SAVH
        STRA,R0 TMPH
        LODA,R0 E1SAVL
        STRA,R0 TMPL
        BSTA,UN DIV16
        ZBRR *VEAM_HI 
EAM_MOD:
        LODA,R0 EXPH
        STRA,R0 E1SAVH
        LODA,R0 EXPL
        STRA,R0 E1SAVL
        ZBSR *VINC_IP  
        LODI,R0 >MD_AT_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <MD_AT_RET
        STRA,R0 SWBASE,R3+
        ZBRR *VEAM_ATOM 
MD_AT_RET:
        LODA,R0 E1SAVH
        STRA,R0 TMPH
        LODA,R0 E1SAVL
        STRA,R0 TMPL
        BSTA,UN DIV16
        LODA,R0 TMPH
        STRA,R0 EXPH
        LODA,R0 TMPL
        STRA,R0 EXPL
        ZBRR *VEAM_HI 
EAM_ATOM:
        ZBSR *VWSKIP  
        LODA,R0 *IPH
        COMI,R0 A'-'
        BCTR,EQ EAM_NEG
        COMI,R0 A'+'
        BCTR,EQ EAM_POS
        COMI,R0 A'('
        BCTR,EQ EAM_PAREN
        BSTA,UN PARSE_FACTOR
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
        LODI,R0 >POS_AT_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <POS_AT_RET
        STRA,R0 SWBASE,R3+
        ZBRR *VEAM_ATOM 
POS_AT_RET:
        BCTR,UN PARSER_RET
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
        BCTR,UN PRO_JMP
PRO_EQ:
        IORI,R1 2
        BCTR,UN PRO_JMP
PRO_GT:
        IORI,R1 4
PRO_JMP:
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
        BSTR,UN PARSE_U16                ; [+1]
        BCTA,UN NEG_EXP                  ; tail call: negate if NEGFLG set

; =============================================================================
;  PARSE_U16 -- Parse unsigned decimal digits -> EXPH:EXPL
; Jumps to JSYNERR if no digits found.
; In:  IPH:IPL -> first digit char
; Out: EXPH:EXPL = value
; Clobbers: R0, R3, SC0, EXPH, EXPL, TMPH, TMPL (R3SAVE used to preserve R3)
PARSE_U16:
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 EXPL
        LODA,R0 *IPH
        COMI,R0 A'0'
        BCTR,LT PRO_NONE; surrogate for JSYNERR
        COMI,R0 A'9'+1
        BCTR,GT PRO_NONE; surrogate for JSYNERR
PU16_LP:
        LODA,R0 *IPH
        COMI,R0 A'0'
        RETC,LT
        COMI,R0 A'9'+1
        BCTR,LT PU16_DIG
        RETC,UN
PU16_DIG:
        SUBI,R0 A'0'
        STRA,R0 SC0
        ; inline INC_IP to save a RAS slot at deepest call path
        LODA,R0 IPL
        ADDI,R0 1
        STRA,R0 IPL
        TPSL $01
        BCTR,LT PU16_DNC
        LODA,R0 IPH
        ADDI,R0 1
        STRA,R0 IPH
PU16_DNC:
        STRA,R3 R3SAVE                   ; save SW stack pointer
        LODA,R0 EXPH
        STRA,R0 TMPH
        LODA,R0 EXPL
        STRA,R0 TMPL
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 EXPL
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
        BSTR,UN ABS_TMP                  ; [+1] sets NEGFLG=1 if TMP was negative
        BSTR,UN ABS_EXP                  ; [+1] toggles NEGFLG if EXP was negative
        LODA,R0 EXPH
        STRA,R0 SC0                      ; SC0 = |EXP| hi
        LODA,R0 EXPL
        STRA,R0 SC1                      ; SC1 = |EXP| lo
        EORZ,R0
        STRA,R0 EXPH                     ; clear EXP (accumulator starts at 0)
        STRA,R0 EXPL
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
        BCTR,GT DV_NZ
        BCTR,LT DV_NZ
        LODA,R0 EXPL
        BCTR,EQ JERRDIVZER
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
        STRA,R3 R3SAVE                   ; save caller R3
        LODI,R3 $FF                      ; fresh SW stack
        LODA,R0 EXPH
        ANDI,R0 $80
        BCTR,EQ PS_POS
        LODI,R0 A'-'
        ZBSR *VCOUT  
        LODA,R0 EXPH
        COMI,R0 $80
        BCTR,EQ PS_CHKMIN
PS_NEGNORM:
        BSTA,UN NEG_EXP_BODY
        BCTR,UN PS_POS
PS_CHKMIN:
        LODA,R0 EXPL
        BCTR,EQ PS_MIN
        BCTR,UN PS_NEGNORM
PS_MIN: ; character print to avoid RAS usage
        LODI,R0 A'3'
        ZBSR *VCOUT  
        LODI,R0 A'2'
        ZBSR *VCOUT  
        LODI,R0 A'7'
        ZBSR *VCOUT  
        LODI,R0 A'6'
        ZBSR *VCOUT  
        LODI,R0 A'8'
        ZBRR *VCOUT 
PS_POS:
        LODA,R0 EXPH
        BCTR,GT PS_NZ
        BCTR,LT PS_NZ
        LODA,R0 EXPL
        BCTR,EQ PS_ZERO
        BCTR,UN PS_NZ
PS_ZERO:
        LODI,R0 A'0'
        ZBRR *VCOUT 
PS_NZ:
        LODI,R0 >PS_DONE
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
        LODA,R0 EXPH
        STRA,R0 TMPH
        LODA,R0 EXPL
        STRA,R0 TMPL
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 EXPL
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
        SUBI,R3 1
        LODA,R0 SWBASE,R3
        STRA,R0 TEMPRETL
        SUBI,R3 1
        BCTA,UN *TEMPRETH

PS_DONE:
        LODZ,R3
        LODA,R3 R3SAVE
        RETC,UN

; =============================================================================
;  RDLINE -- Read a line from input into IBUF with echo and backspace
; In:  nothing
; Out: IBUF = NUL-terminated input line; IPH:IPL -> one past last char
; Clobbers: R0, R1, IPH, IPL
RDLINE:
        ZBSR *VSET_IP_IBUF                ; IPH:IPL = IBUF
RL_LP:
        BSTA,UN CHIN                     ; [+1] blocking read
        COMI,R0 NUL
        BCTA,EQ RL_EOL
        STRZ,R1
        COMI,R1 CR
        BCTA,EQ RL_EOL
        COMI,R1 LF
        BCTA,EQ RL_EOL
        COMI,R1 BS
        BCTR,EQ RL_BS
        ; buffer full check
        LODA,R0 IPH
        SUBI,R0 <IBUF
        BCTR,GT RL_FULL
        BCTR,LT RL_STORE
        LODA,R0 IPL
        SUBI,R0 >IBUF+63
        BCTR,LT RL_STORE
RL_FULL:
        BCTR,UN RL_LP
RL_STORE:
        STRA,R1 *IPH
        LODZ,R1
        ZBSR *VCOUT  
        ZBSR *VINC_IP  
        BCTR,UN RL_LP
RL_BS:
        LODA,R0 IPH
        SUBI,R0 <IBUF
        BCTR,GT RL_BSDO
        BCTR,LT RL_LP
        LODA,R0 IPL
        SUBI,R0 >IBUF
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
        BSTA,UN PRT_BS
        ZBSR *VPRT_SPACE  
        BSTA,UN PRT_BS
        BCTA,UN RL_LP
RL_EOL:
        EORZ,R0
        STRA,R0 *IPH
        BCTA,UN PRT_CRLF

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
        ZBRR *VEATWORD 

; =============================================================================
;  WSKIP -- Skip spaces at IP
; In:  IPH:IPL -> current position
; Out: IPH:IPL -> first non-space char
; Clobbers: R0
WSKIP:
        LODA,R0 *IPH
        COMI,R0 SP
        BCFR,EQ WSKIPRET
WS_ADV:
        ZBSR *VINC_IP 
        ZBRR *VWSKIP 

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
UC_DO:
        SUBI,R0 32
        RETC,UN

; =============================================================================
;  SHARED 16-BIT POINTER INCREMENT/DECREMENT
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
        BCFR,LT ET_RET      ; CC != LT -> no borrow: hi unchanged
        LODA,R0 IPH,R1          ; borrow: decrement hi byte
        SUBI,R0 1
        BCTR,UN ET_STORE        ; borrow tail from INC_xx

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
        STRA,R0 SC0                      ; save error code
        BSTR,UN PRT_QUEST
        LODA,R0 SC0
        ZBSR *VCOUT                      ; print error code
        LODA,R0 RUNFLG                  ; OPT-10: SC1=RUNFLG, 0->EQ, 1->GT
        BCTR,EQ DE_NL                   ; not running, no line number
        LODI,R0 '@'
        ZBSR *VCOUT  
        LODA,R0 CURH
        STRA,R0 EXPH
        LODA,R0 CURL
        STRA,R0 EXPL
        BSTA,UN PRINT_S16                ; [+1]
DE_NL:
        BSTR,UN PRT_CRLF
        BSTA,UN DO_END                   ; [+1] clears SWSP, FORSP, GOTOFLG, RUNFLG
        BCTA,UN REPL                     ; kills full hardware RAS

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
PRT_CRLF:
        BSTR,UN PRT_CR                   ; Print CR/LF
        ; drop through

; =============================================================================
;  Shared character print routines -- $EC (COMA) byte-skip chain
; Each entry loads its character then falls through via the skip opcode trick.
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
;  TABLES 
BANNER:
        DB CR, LF, "uBASIC 2650 V3.7", CR, LF, "Bytes Free:",NUL

; -- Keyword dispatch table
; Format: [c1][c2][c3][hi][lo]  NUL-terminated on c1.
; hi:lo = handler address. Matched on first three uppercase chars.
; c3=A' ' (space) = wildcard (IF -- only 2 chars before body).
; THEN matched internally by DO_IF -- not dispatched here.
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
        DB "NEX", <DO_NEXT,   >DO_NEXT    ; NEXT (c3='X' vs NEW c3='W')
        DB "PRI", <DO_PRINT,  >DO_PRINT   ; PRINT
        DB "REM", <DO_REM,    >DO_REM     ; REM
        DB "RET", <DO_RETURN, >DO_RETURN  ; RETURN
        DB "RUN", <DO_RUN,    >DO_RUN     ; RUN
        DB NUL

ROMEND: 

;  RAM variables -- sequential RES block 
;  Page $14xx chosen so vars clear ROMEND (~$12E4) by ~284 bytes (safe growth margin)
;  and IBUF lo-byte+63 = $6C+63 = $AB (no page wrap in RDLINE buffer check).
;  PROG base moves to $14E0, giving 2847 bytes of BASIC program store (vs 2300 before).
;
;  ORDERING CONSTRAINT: IPH..LNUML must remain sequential with no gaps.
;    INC_ET/DEC_ET use LODI,R0 EXPH-IPH / TMPH-IPH / GOTOH-IPH / LNUMH-IPH
;    as R1 offsets for R1-indexed addressing (evaluates to 4, 2, 8, 12).
;    NEG_EXP_BODY/ABS_TMP use LODI,R1 EXPH-IPH / TMPH-IPH (same values, R1).
;    These are now assembly-time expressions -- reordering will silently break them.
;
;  GSSTKLIM, FORSTKLIM, PROGLIM: numeric constants (not addresses), kept as EQU.

        ORG     4096

; --- Ordered group: offsets from IPH used by INC_ET/DEC_ET/NEG_SHARED ---
IPH     RES 1       ; $1400  interpreter pointer hi       (INC_ET offset 0)
IPL     RES 1       ; $1401  interpreter pointer lo
TMPH    RES 1       ; $1402  temp 16-bit hi               (INC_ET offset 2 = TMPH-IPH)
TMPL    RES 1       ; $1403  temp 16-bit lo
EXPH    RES 1       ; $1404  expression result hi         (INC_ET offset 4 = EXPH-IPH)
EXPL    RES 1       ; $1405  expression result lo
RUNFLG  RES 1       ; $1406  $01=running $00=immediate
GOTOFLG RES 1       ; $1407  $00=sequential $01=GOTO $02=GOSUB $03=FOR direct addr
GOTOH   RES 1       ; $1408  pending target hi            (DEC_ET offset 8 = GOTOH-IPH)
GOTOL   RES 1       ; $1409  pending target lo
CURH    RES 1       ; $140A  current line hi  (error reporting)
CURL    RES 1       ; $140B  current line lo
LNUMH   RES 1       ; $140C  scratch line number hi       (DEC_ET offset 12 = LNUMH-IPH)
LNUML   RES 1       ; $140D  scratch line number lo
; --- End ordered group ---

SC0     RES 1       ; $140E  scratch byte 0
SC1     RES 1       ; $140F  scratch byte 1
NEGFLG  RES 1       ; $1410  sign flag
PEH     RES 1       ; $1411  program end pointer hi
PEL     RES 1       ; $1412  program end pointer lo
SAVEH   RES 1       ; $1413  EXPR_AM: saved left hi for +/-
SAVEL   RES 1       ; $1414  EXPR_AM: saved left lo for +/-
E1SAVH  RES 1       ; $1415  EAM_HI: saved left hi for *//%
E1SAVL  RES 1       ; $1416  EAM_HI: saved left lo for *//%
FORVAR  RES 1       ; $1417  FOR loop variable letter (A-Z)
FORSP   RES 1       ; $1418  FOR stack pointer ($FF=empty, 0/7/14/21=frame offsets)
SWSP    RES 1       ; $1419  GOSUB stack pointer ($FF=empty)
SWSTK   RES 2       ; $141A  next-line pointer cache [NLP_H][NLP_L] written by DR_EXEC
RELOP   RES 1       ; $141C  relational op bitmask: bit0=LT bit1=EQ bit2=GT
TEMPRETH RES 1      ; $141D  SW return address hi
TEMPRETL RES 1      ; $141E  SW return address lo
R3SAVE  RES 1       ; $141F  save/restore R3 across PARSE_U16 multiply loop

;  SW call stack -- used by PARSE_EXPR / PRINT_S16 only
; R3 = index ($FF=empty, grows up). Each frame = [lo][hi].
; Push: STRA,R0 *SWBASE,R3+ (lo first), STRA,R0 *SWBASE,R3+ (hi).
; Pop:  LODA,R0 *SWBASE,R3- (hi first), LODA,R0 *SWBASE,R3- (lo).
SWBASE  RES 32      ; $1420  SW stack base: 32 bytes  $1420-$143F

;  GOSUB stack (v3.2) -- managed by SWSP
GSBASE  RES 16      ; $1440  GOSUB stack base: 16 bytes  $1440-$144F

;  FOR/NEXT stack (v3.3) -- managed by FORSP
FORBASE RES 28      ; $1450  FOR stack base: 28 bytes  $1450-$146B

; Buffers
IBUF    RES 64      ; $146C  input buffer 64 bytes  $146C-$14AB
                    ;        RDLINE check: >IBUF+63 = $6C+63 = $AB (no page wrap)
VARS    RES 52      ; $14AC  A-Z variables 2 bytes each  $14AC-$14DF

; =============================================================================
;  Pre-loaded SHOWCASE program
;
;  Line format: <lineno_hi> <lineno_lo> <body_ASCII> <CR>
;  Lines  10-190: feature demos (PRINT, CHR$, arithmetic, comparisons, GOTO loop)
;  Lines 192-218: FOR/NEXT and GOSUB/RETURN demos
;  Lines 270-480: Mandelbrot set renderer
;  Line  500:     GOSUB subroutine (PRINT "sub"; / RETURN)
;
;  Format: DB hi,lo,"text",$0D  -- hi-then-lo matches DR_EXEC record format.
;  $22=DQ $3B=semicolon  in-string chars that need escaping.
; =============================================================================
PROG:
        DB 0,10,"REM uBASIC 2650 - SHOWCASE V3.7",$0D                          ; 10  REM uBASIC 2650 - SHOWCASE V3.6d
        DB 0,20,"PRINT ",$22,"-- uBASIC 2650 V3.7 Showcase --",$22,$0D         ; 20  PRINT "-- uBASIC 2650 V3.6d Showcase --"
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
        DB 0,212,"GOSUB 500",$0D                                                ; 212 GOSUB 500
        DB 0,214,"GOSUB 500",$0D                                                ; 214 GOSUB 500
        DB 0,216,"PRINT ",$22,"",$22,$0D                                        ; 216 PRINT ""
        DB 0,218,"GOTO 270",$0D                                                 ; 218 GOTO 270
        DB 1,14,"PRINT ",$22,"--- MANDELBROT ---",$22,$0D                      ; 270 PRINT "--- MANDELBROT ---"
        DB 1,24,"I=-64",$0D                                                     ; 280 I=-64
        DB 1,34,"IF I>56 THEN GOTO 480",$0D                                    ; 290 IF I>56 THEN GOTO 480
        DB 1,44,"D=I",$0D                                                       ; 300 D=I
        DB 1,54,"C=-120",$0D                                                    ; 310 C=-120
        DB 1,64,"IF C>4 THEN GOTO 450",$0D                                     ; 320 IF C>4 THEN GOTO 450
        DB 1,74,"A=C",$0D                                                       ; 330 A=C
        DB 1,75,"B=D",$0D                                                       ; 331 B=D
        DB 1,76,"E=0",$0D                                                       ; 332 E=0
        DB 1,77,"N=1",$0D                                                       ; 333 N=1
        DB 1,84,"IF N>16 THEN GOTO 390",$0D                                    ; 340 IF N>16 THEN GOTO 390
        DB 1,94,"IF E>0 THEN GOTO 380",$0D                                     ; 350 IF E>0 THEN GOTO 380
        DB 1,104,"T=A*A/64-B*B/64+C",$0D                                       ; 360 T=A*A/64-B*B/64+C
        DB 1,114,"B=2*A*B/64+D",$0D                                             ; 370 B=2*A*B/64+D
        DB 1,115,"A=T",$0D                                                      ; 371 A=T
        DB 1,124,"IF A*A/64+B*B/64>256 THEN IF E=0 THEN E=N",$0D               ; 380 IF A*A/64+B*B/64>256 THEN IF E=0 THEN E=N
        DB 1,134,"N=N+1",$0D                                                    ; 390 N=N+1
        DB 1,135,"IF N<=16 THEN GOTO 340",$0D                                  ; 391 IF N<=16 THEN GOTO 340
        DB 1,144,"IF E>0 THEN PRINT CHR$(E+32)",$3B,$0D                        ; 400 IF E>0 THEN PRINT CHR$(E+32);
        DB 1,154,"IF E=0 THEN PRINT CHR$(32)",$3B,$0D                          ; 410 IF E=0 THEN PRINT CHR$(32);
        DB 1,164,"C=C+4",$0D                                                    ; 420 C=C+4
        DB 1,174,"GOTO 320",$0D                                                 ; 430 GOTO 320
        DB 1,194,"PRINT",$0D                                                    ; 450 PRINT
        DB 1,204,"I=I+6",$0D                                                    ; 460 I=I+6
        DB 1,214,"GOTO 290",$0D                                                 ; 470 GOTO 290
        DB 1,224,"END",$0D                                                      ; 480 END
        DB 1,244,"PRINT ",$22,"sub",$22,$3B,$0D                                 ; 500 PRINT "sub";
        DB 1,246,"RETURN",$0D                                                   ; 502 RETURN
SHOWCASE_END:

        END
