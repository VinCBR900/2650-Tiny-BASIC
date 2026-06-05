; uBASIC2650.asm       Tiny BASIC interpreter for Signetics 2650
; Version: v3.1
; Date:    2026-06-04
;
; Target : PIPBUG 1 monitor (ROM $0000-$03FF, RAM $0400-$043F)
;          Code base $0440.  Single 8192-byte address page (bit15-13 always 0).
;          I/O via PIPBUG ROM stubs: COUT=$02B4 (R0=char), CHIN=$0286 (R0   key).
;          CRLF inlined (2 bytes)     avoids consuming a RAS slot vs PIPBUG CRLF.
;
; Assembler: asm2650.c v1.8+   Simulator: pipbug_wrap v1.1
; Build:
;   gcc -Wall -O2 -o asm2650 asm2650.c
;   gcc -Wall -O2 -DGAMER -o pipbug_wrap pipbug_wrap.c
;   ./asm2650 uBASIC2650.asm uBASIC2650.hex
;   ./pipbug_wrap uBASIC2650.hex
;
;        CC SEMANTICS (2650 ALU)                                                                                                                                                       
;   ADD/SUB: result   128   LT  result>0,<128   GT  result=0   EQ
;   Carry bit (PSL bit 0): C=1 = carry / no-borrow (independent of CC).
;   Carry test: TPSL $01       CC=EQ if C=1 (carry), CC=LT if C=0 (no carry).
;   Carry skip: RETC,LT / BCTA,LT = branch if C=0 (no carry).
;   Unsigned compare: PPSL $02 / COMA or SUBA / CPSL $02.
;   Binary flag (0 or 1): after LODA CC=EQ(0) or GT(1); use BCTR,GT not COMI $01.
;   Single page: all addresses $0000-$1FFF; hi-byte carry in stack indexing impossible.
;
;        HI/LO OPERATOR CONVENTION                                                                                                                                                 
;   <ADDR = HIGH byte (bits 15:8)   e.g. <$1634 = $16
;   >ADDR = LOW  byte (bits  7:0)   e.g. >$1634 = $34
;
;        RAS DEPTH BUDGET (8-level hardware stack)                                                                                                 
;   Every BSxx (BSTA/BSTR/BSFA/BSFR) consumes one slot regardless of condition.
;   BCTA/BCTR/BCFA/BCFR are plain branches     no RAS cost.
;   COUT/CHIN use 1 internal sub: add 1 to caller depth.
;
;        SCRATCH REGISTER CONVENTIONS                                                                                                                                        
;   R0  working register, arithmetic, I/O.
;   R1  index register (LODA/STRA BASE,R1); also PRINT_S16 digit buffer index.
;       Clobbered by INC_ET (INC_TMP/INC_EXP shared body). Callers verified safe.
;   R2  long-lived variable letter (DO_LET/DO_INPUT, preserved across PARSE_EXPR).
;       Never written by any subroutine except DO_LET and SE_BAREASS.
;   R3  loop counter (BDRR/BIRR); STORE_LINE shift count.
;
;        KNOWN OPEN ITEMS                                                                                                                                                                            
;   COLON-01: ':' multi-statement separator not supported ("A=1:GOTO 10").
;             Unlikely to be implemented, would consume a RAS slot per statement.
;   OPT-16:   MUL16/DIV16 use naive loop (O(N)). Bit-serial with RRL/RRR rotate
;             instructions provides O(16) at approx same code size.
;             Worthwhile for real-hardware performance; deferred.
;
;        CHANGE HISTORY                                                                                                                                                                                  
;
;   V3.1  2026-06-05
;         BUG-SL Fix: SL_DOMOV pointer decrements used BCFR,LT to detect borrow
;         after SUBI,R0 1, but BCFR,LT tests the CC field (set from result value
;         bit-7), not the carry flag. Any lo-byte in $80..$FF gives CC=LT
;         regardless of borrow, causing spurious hi-byte decrements and runaway
;         src/dst pointers. Fix: replace BCFR,LT with TPSL $01 / BCTR,EQ to test
;         carry directly. Loop-back BCTR,UN extended to BCTA,UN (+2 bytes overhead).
;         Verified: insert/replace/delete all lines, first/middle/last positions.
;         BUG-LE Fix: SL_SHLOOP stop condition in STORE_LINE was missing
;         BCTR,LT SL_NOSHIFT after hi-byte subtract, and the lo-byte path
;         fell through to SL_DOMOV when src.lo < ins.lo (CC=LT). Both cases
;         caused the shift loop to walk src past the insertion point, corrupting
;         the program store when replacing a line with content of different length.
;         Fix: added BCTR,LT SL_NOSHIFT (hi-byte) and BCTR,GT/BCTR,UN (lo-byte)
;         so all six hi/lo comparison outcomes route correctly.
;         Verified with pipbug_wrap: 10 PRINT / 20 PRINT / 30 PRINT / LIST /
;         20 PRINT "HELLO" / LIST now correctly shows updated line 20.
;
;   V3.0  2026-06-04  3306 Interpreter bytes 
;         Bug Fix: PARSE_U16 used LODI,R3 10 / BDRR,R3 for multiply-by-10
;         loop, clobbering R3 which holds the SW call stack pointer.
;         On return, PARSER_RET saw R3=0 (not $FF empty sentinel) and called
;         SWRETURN with a corrupt stack pointer, jumping to garbage.
;         Fix: STRA,R3 R3SAVE / LODI,R3 10 / loop / LODA,R3 R3SAVE around
;         the multiply section. Showcase runs to completion;
;         Replaced shunting-yard PARSE_EXPR (737B) + GET_PREC/APPLY_OP (185B)
;           with recursive descent + SW stack (PARSER_RET 7B + PARSE_EXPR 451B).
;           v28 PARSE_S16/PARSE_U16 retained verbatim (known-good).
;           v28 PARSE_FACTOR retained verbatim.
;
;   RAM CHANGES vs v28:
;     Removed: OPSTK($1614-$161B), VALSH($161C-$1623), VALSL($1624-$162B),
;              STKIDX($162C), PRECTMP($163D)
;     Added:   SAVEH($1614), SAVEL($1615), E1SAVH($1616), E1SAVL($1617)
;
;   V2.8  2026-05-30 - 3576 ROM bytes
;         General code refactor for size. PF_LOADVAR & DO_LIST refactored.
;         Computed GOTO implemented from PARSE_S16
;   v2.7  2026-05-29  
;         TAB(spaces) added to DO_PRINT (print n spaces).
;         OPT-15: Shared sign-handling subroutines NEG_EXP / NEG_EXP_BODY /
;               ABS_TMP / ABS_EXP replacing inline code in MUL16, DIV16,
;               PARSE_S16, PRINT_S16, PX_UNEG. ~190 bytes saved.
;         OPT-15b: ABS_TMP tail-calls INC_TMP (saves RAS slot + 11 bytes).
;               ABS_EXP tail-calls NEG_EXP_BODY (saves RAS slot).
;               Both set/toggle NEGFLG before tail-call.
;   v2.6  2026-05-23
;         CHR$() inlined into DO_PRINT; INC_ET shared INC_EXP/INC_TMP body.
;         BUG-DP-01: BCTA,UN EATWORD     BSTA (plain branch lost return address).
;         BUG-DEC-01: DEC_IP RETC,LT     TPSL $01 / RETC,EQ (sign vs carry).
;         ERRFLG refactor: PARSE_U16     JSYNERR on no-digit; removed ERRFLG
;               polls at callers. REPL clears ERRFLG each iteration.
;         BUG-NF-01: FL_RET_NF split from JERRLINE; DELETE_LINE now gets
;               normal return when line not found (was crashing to DO_ERROR).
;         BUG-NF-02: REPL clears ERRFLG before TRY_STORE_LINE each iteration.
;         OPT-2..4: duplicate LODA, AO_STORE STKIDX reuse, PPSL   SUBA. -22B.
;         OPT-5a: removed dead carry checks from all 19 stack-index patterns
;               (BCTR,GT/ADDI dead code     hi byte always <$80, max index 7,
;               lo+7 < 256, carry impossible in single-page addressing). -76B.
;         OPT-10: LODA binary-flag / COMI,R0 $01 / BCT     LODA / BCTR,GT.
;               Applied to ERRFLG, RUNFLG, GOTOFLG, SC1(saved RUNFLG). -8B.
;   v2.5  2026-05-22
;         BUG-FL-02: FI_CHK unsigned lo-byte compare (PPSL $02 / COMA).
;         BUG-CHR-01: PF_CHR_TRY double INC_IP     jump to PF_LVNCA.
;   v2.4  2026-05-19  Showcase + Mandelbrot appended.
;   v2.3  BUG-FL-01 partial: FL_CHKLO RETC,LT   BCTR,LT. BUG-RAS-01,
;         BUG-MAND-01, BUG-FI-01, BUG-DIV-ZCHK-01 fixed.

;  ASCII 
CR      EQU     $0D
LF      EQU     $0A
BS      EQU     $08
SP      EQU     $20
NUL     EQU     $00
DQ      EQU     $22

;  ERRORS 
ERR_SYN         EQU '0'
ERR_UND_LINE    EQU '1'
ERR_DIV_ZERO    EQU '2'
ERR_OOM         EQU '3'
ERR_VAR         EQU '4'

;  PIPBUG 1 I/O entry points 
COUT    EQU     $02B4   ; putchar: R0 = char to output
CHIN    EQU     $0286   ; getchar: blocking: R0 =  key
RS      EQU     $10

;  RAM variables  pinned above code, below PROGLIM 
; Code ceiling: ~$15FF (code must not reach $1600 or crash).
; IP, TMP, EXP must be in this order
IPH     EQU $1600   ; interpreter pointer hi
IPL     EQU $1601   ; interpreter pointer lo
TMPH    EQU $1602   ; temp 16-bit hi
TMPL    EQU $1603   ; temp 16-bit lo
EXPH    EQU $1604   ; expression result hi
EXPL    EQU $1605   ; expression result lo

; Other Vars
RUNFLG  EQU $1606   ; $01=running $00=immediate
GOTOFLG EQU $1607   ; $01=GOTO/GOSUB pending
GOTOH   EQU $1608   ; pending target line hi
GOTOL   EQU $1609   ; pending target line lo
CURH    EQU $160A   ; current line hi  (error reporting)
CURL    EQU $160B   ; current line lo
LNUMH   EQU $160C   ; scratch line number hi
LNUML   EQU $160D   ; scratch line number lo
SC0     EQU $160E   ; scratch byte 0
SC1     EQU $160F   ; scratch byte 1
ERRFLG  EQU $1610   ; error flag $00=ok $01=error/handled
NEGFLG  EQU $1611   ; sign flag
PEH     EQU $1612   ; program end pointer hi
PEL     EQU $1613   ; program end pointer lo
SAVEH   EQU $1614   ; EXPR_AM: saved left hi for +/-
SAVEL   EQU $1615   ; EXPR_AM: saved left lo for +/-
E1SAVH  EQU $1616   ; EAM_HI: saved left hi for *//%
E1SAVL  EQU $1617   ; EAM_HI: saved left lo for *//%
SWSP    EQU $162D   ; SW call stack pointer ($FF=empty)
SWSTK   EQU $162E   ; SW call stack 82 bytes  $012E-$013D
RELOP   EQU $163E   ; relational op 1-6

;  SW call stack (v2.0) 
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
VARS    EQU $16A3   ; A-Z variables 2 bytes each  
PROG    EQU $16d7   ; program store base (VARS+52)
PROGLIM EQU $1fff   ; one past end of program store

;  CODE starts at $0440 (after Pipbug 1kB ROM + 64B RAM) 
        ORG     $0440

; =============================================================================
;  RESET / ENTRY  
RESET:
        ; Setup Stack Ptr - oh, wait...
        CPSL $FF                ; clear everything  

        ; Sets up showcase
        LODI,R0 <SHOWCASE_END
        STRA,R0 PEH
        LODI,R0 >SHOWCASE_END   ; BUG-SC-01 FIX: was < (hi byte), must be > (lo byte)
        STRA,R0 PEL

        ; Actual start
        BSTA,UN DO_END      ; setup RUNFLAG etc - Change to DO_NEW for no showcase 
        
        ; clear A-Z variables (52 bytes) using IPH:IPL as scratch pointer
        LODI,R0 <VARS
        STRA,R0 IPH
        LODI,R0 >VARS
        STRA,R0 IPL
        LODI,R3 $34             ; 52 iterations: R3 counts $34$33...$01$00exit
CLRV:
        EORZ,R0 ; Clear R0
        STRA,R0 *IPH
        BSTA,UN INC_IP
        BDRR,R3 CLRV            ; R3--; if R3!=0 branch
; signon banner
        LODI,R0 <BANNER
        STRA,R0 IPH
        LODI,R0 >BANNER
        STRA,R0 IPL
        BSTA,UN PRTSTR
        ; drop through
; =============================================================================
;  Main Loop 
REPL:
        CPSL RS + 7               ; Ensure using primary reg bank and SP is zero  
        BSTA,UN PRT_CHEV    ; print chevron
        BSTA,UN PRT_SPACE
        BSTA,UN RDLINE
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
        EORZ,R0             ; BUG-NF-02 FIX: clear ERRFLG before TRY_STORE_LINE;
        STRA,R0 ERRFLG      ; stale $01 from previous line-store would suppress STMT_EXEC
        BSTA,UN TRY_STORE_LINE
        LODA,R0 ERRFLG                   ; OPT-10: LODA sets CC; ERRFLG=0EQ, 1GT
        BCTR,GT REPL             ; branch if ERRFLG=1 (was COMI $01/BCTR,EQ)
        BSTR,UN STMT_EXEC
        BCTR,UN REPL

; =============================================================================
BANNER:
        DB CR, LF, "uBASIC 2650 V3.1", CR, LF, NUL

; =============================================================================
;  STMT_EXEC 
; Decode and dispatch one statement from IP.
; KW_TAB format: [c1][c2][hi][lo] where hi:lo = handler address.
; SE_SCAN advances TMPH:TMPL by 4 per entry; at match loads hi:lo into
; EXPH:EXPL and branches via BCTA,UN *EXPH (absolute indirect jump).
; RAS depth: 1 from REPL, or 3 from DO_IF (THEN body).
; Worst inner depth from here: +4 (->DO_xxx->PARSE_EXPR->PARSE_FACTOR->UPCASE)
STMT_EXEC:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        RETC,EQ                          ; blank line  return

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
        BCTA,EQ SE_NOTKW                 ; BUG-LET-01 FIX: end of table  check bare assignment
        SUBA,R0 SC0
        BCTR,EQ SE_CHK2

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

SE_NOTKW:
        ; BUG-LET-01 FIX: keyword table exhausted. Check for bare variable assignment:
        ;   SC0 = first char (A-Z), SC1 = second char ('=').
        ;   GETCI_UC already consumed both; IP now points at the RHS expression.
        ;   If SC0 is A-Z and SC1 is '=' we can jump straight to DL_EX with
        ;   SC0 (and R2) holding the variable letter  exactly what DO_LET does
        ;   after consuming 'LET <var> ='.
        LODA,R0 SC0
        COMI,R0 A'A'
        BCTR,LT JSYNERR          ; SC0 < 'A'  not a variable
        COMI,R0 A'Z'+1
        BCTR,GT JSYNERR          ; SC0 > 'Z'  not a variable (GT because unsigned)
        LODA,R0 SC1
        COMI,R0 A'='
        BCTR,EQ SE_BAREASS       ; SC1 == '='  bare assignment
        BCTR,UN JSYNERR          ; not '='  true syntax error
SE_BAREASS:
        LODA,R0 SC0
        STRZ,R2                  ; save letter in R2 (survives PARSE_EXPR, per DO_LET convention)
        BCTA,UN DL_EX            ; IP already past '=', expression follows

; =============================================================================
; Global Syntax Error handler
; Clobebrs: R0 but resets everything so doenst matter
JSYNERR:
        LODI,R0 ERR_SYN
        BCTA,UN DO_ERROR

; =============================================================================
;  DO_NEW setup system VARS 
;  DO_END clear system VARS 
; Clobbers: R0
; Should probably clear PROG memory
DO_NEW:
        LODI,R0 <PROG
        STRA,R0 PEH
        LODI,R0 >PROG
        STRA,R0 PEL
        ; drop through
DO_END:
        LODI,R0 $FF
        STRA,R0 SWSP
        EORZ,R0 ; Clear R0
        STRA,R0 GOTOFLG
        BCTA,UN CLR_RUNFLG      ; tail call

; =============================================================================
;  DO_PRINT combined with PRTSTR 
; PRINT [item1][;]...[itemx][;]    
;   item = "string" | expr | TAB(spaces) | CHR$(expr) 
;   Inline check for CHR$ and TAB
; PRTSTR Print NUL-terminated string at IPH:IPL.
; Clobbers: R0 
DO_PRINT:
    BSTA,UN WSKIP     
    LODA,R0 *IPH
    BCTA,EQ DP_NL     ; Keep absolute: DP_NL is likely > 63 bytes away

DP_ITEM:
    BSTA,UN WSKIP
    LODA,R0 *IPH
    COMI,R0 DQ          ; Is it opening "
    BCTA,EQ DP_STRING   ; Keep absolute: DP_STRING is at the bottom
    COMI,R0 'C'
    BCFR,EQ DP_TAB     ; If not 'C', safely jump forward to DP_TAB
   
    BSTA,UN INC_IP
    LODA,R0 *IPH
    COMI,R0 'H'
    BCTR,EQ DP_CHAR

DP_BACKUP:
    BSTA,UN DEC_IP     ; Fall through to DP_EXPR (Saves a jump)

DP_EXPR:
    BSTA,UN PARSE_EXPR
    BSTA,UN PRINT_S16
    BCTR,UN DP_SEP     

DP_CHAR:
    BSTA,UN EATWORD
    BSTA,UN PARSE_EXPR
    LODA,R0 EXPL
    BSTA,UN COUT
    BCTR,UN DP_SEP     

DP_TAB:
    COMI,R0 'T'
    BCFR,EQ DP_EXPR    ; Relative jump backwards to DP_EXPR
    BSTA,UN INC_IP
    LODA,R0 *IPH
    COMI,R0 'A'
    BCFR,EQ DP_BACKUP   ; Relative jump backwards to DP_BACKUP
    BSTA,UN EATWORD
    BSTA,UN PARSE_EXPR
    LODA,R1 EXPL
    BCTR,EQ DP_SEP      ; skip if zero otherwise 255 spaces
TAB_LOOP:
    BSTA,UN PRT_SPACE
    BDRR,R1 TAB_LOOP
                ; Fall through directly into DP_SEP
DP_SEP:
    BSTA,UN WSKIP
    LODA,R0 *IPH
    COMI,R0 $3B		; semicolon
    BCTR,EQ DP_SEMI
                ; Fall through directly into DP_NL
DP_NL:
    BSTA,UN PRT_CR
    BCTA,UN PRT_LF          ; Exit routine

DP_SEMI:
    BSTA,UN INC_IP
    BSTA,UN WSKIP
    LODA,R0 *IPH
    RETC,EQ
    BCTA,UN DP_ITEM         

DP_STRING:
    BSTA,UN INC_IP
PRTSTR:
    LODA,R0 *IPH
    RETC,EQ             ; Hard Bail if NUL before "
    COMI,R0 DQ         ; Closing " still part of print
    BCTR,EQ DP_SCLS     ; test
    BSTA,UN COUT
    BSTA,UN INC_IP
    BCTR,UN PRTSTR

DP_SCLS:
    BSTA,UN INC_IP
    BCTR,UN DP_SEP          ; Relative jump backwards to DP_SEP

; =============================================================================
; DO_LET / DO_INPUT shared store path 
; DO_INPUT jumps to DL_STORE with SC0 = variable letter already set.
DO_LET:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        BSTA,UN UPCASE  ; [+1]
        COMI,R0 A'A'
        BCTR,LT JERRVAR
        COMI,R0 A'Z'+1
        BCTR,LT DL_VAROK
JERRVAR:
        LODI,R0 ERR_VAR
        BCTA,UN DO_ERROR
DL_VAROK:
        STRA,R0 SC0                      ; save variable letter in SC0 (immediate use)
        STRZ,R2
        ; SC0 is general scratch clobbered by PARSE_EXPR (operator-stack ops
        ; write SC0 repeatedly). R2 is never written by any routine and
        ; survives the full PARSE_EXPR call below.
        BSTA,UN INC_IP
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 A'='
        BCTR,EQ DL_EQC
        BCTA,UN JSYNERR
DL_EQC:
        BSTA,UN INC_IP
DL_EX:
        BSTA,UN PARSE_EXPR               ; [+1] on error jumps directly to JSYNERR
DL_STORE:
        ; address = VARS + (SC0 - 'A') * 2
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
        LODA,R0 EXPL
        STRA,R0 *TMPH  ; store lo
        ; drop through

; =============================================================================
;  DO_REM - do nothing 
PRTSTR_RET:
DO_REM:
        RETC,UN

; =============================================================================
;  DO_INPUT 
DO_INPUT:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        BSTA,UN UPCASE  ; [+1]
        COMI,R0 A'A'
        BCTR,LT DIN_ERR
        COMI,R0 A'Z'+1
        BCTR,LT DIN_VAROK
DIN_ERR:
        BCTA,UN JERRVAR
DIN_VAROK:
        STRA,R0 SC0                      ; save variable letter
        STRZ,R2                          ; also save in R2 for DL_STORE (SC0 clobbered by PARSE_S16)
        BSTA,UN INC_IP
        BSTA,UN PRT_QUEST
        BSTA,UN PRT_SPACE
        BSTA,UN RDLINE                   ; [+1]
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
        BSTA,UN PARSE_S16                ; [+1] on error jumps directly to JSYNERR
        BCTA,UN DL_STORE

; =============================================================================
;  DO_IF 
; IF expr relop expr THEN stmt
; Depth at entry: 2 (from REPL->STMT_EXEC) or 4 (from REPL->STMT_EXEC->DO_IF->STMT_EXEC->here)
; After THEN: calls STMT_EXEC at +1, which can call DO_xxx at +1, PARSE_EXPR at +1,
;             PARSE_FACTOR at +1  max total 2+1+1+1+1+1 = depth 7 OK.
DO_IF:
        BSTA,UN PARSE_EXPR               ; [+1] on error jumps directly to JSYNERR
        LODA,R0 EXPH
        STRA,R0 LNUMH  ; BUG-T6 FIX: save left in LNUMH:LNUML (TMPH:TMPL clobbered
        LODA,R0 EXPL   ;   by PARSE_EXPR's PX_PUSHV writing <VALSH/$15 to TMPH)
        STRA,R0 LNUML
        BSTA,UN PARSE_RELOP              ; [+1]
        BSTA,UN PARSE_EXPR               ; [+1]

        ; signed 16-bit compare: LNUMH:LNUML (left) vs EXPH:EXPL (right)
        ; bias hi bytes by XOR $80  unsigned compare
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
        LODI,R0 $01          ; right-hi < left-hi: left > right  SC1=$01
        STRA,R0 SC1
        BCTR,UN DIF_TH  ; LT (result: left > right)
DIF_GT:
        LODI,R0 $FF          ; right-hi > left-hi: left < right  SC1=$FF
        STRA,R0 SC1  ; GT (result: left < right)

DIF_TH:
        ; consume THEN keyword: expect T then H then EATWORD
        BSTA,UN WSKIP                    ; [+1]
        BSTA,UN GETCI_UC                 ; [+1]  must be A'T'
        COMI,R0 A'T'
        BCTR,EQ DIF_TH2
        BCTA,UN JSYNERR
DIF_TH2:
        BSTA,UN GETCI_UC                 ; [+1]  must be A'H'
        COMI,R0 A'H'
        BCTR,EQ DIF_EW
        BCTA,UN JSYNERR
DIF_EW:
        BSTA,UN EATWORD                  ; [+1]

        ; BUG-RELOP-02 FIX: TMI,R0 RELOP was wrong  TMI uses RELOP as an
        ; immediate byte (not a runtime RAM read), always assembling as $00.
        ; Fix: map SC1 result to a bitmask in R0, then AND against RELOP at runtime.
        ;   SC1=$FF  LT  bit 0 ($01)
        ;   SC1=$00  EQ  bit 1 ($02)
        ;   SC1=$01  GT  bit 2 ($04)
        ; ANDZ,R1: R0 &= R1.  If result=0: no match  false.
        LODA,R0 SC1
        BCTR,EQ DIF_IS_EQ
        COMI,R0 $FF
        BCTR,EQ DIF_IS_LT
        LODI,R0 4                        ; GT  bit 2
        BCTR,UN DIF_ANDTEST
DIF_IS_LT:
        LODI,R0 1                        ; LT  bit 0
        BCTR,UN DIF_ANDTEST
DIF_IS_EQ:
        LODI,R0 2                        ; EQ  bit 1
DIF_ANDTEST:
        LODA,R1 RELOP                    ; R1 = runtime bitmask from RAM
        ANDZ,R1                          ; R0 &= R1  (ANDZ,rn: R0 &= rn)
        RETC,EQ ; DIF_FALSE                ; zero  no bit match  condition false
        BCTA,UN STMT_EXEC                ; [+1]  execute THEN body

; =============================================================================
; Do_GOTO allows computed gotos - no worse for RAS than
; if expr then print expr
; if expr then goto expr
DO_GOTO:
        BSTA,UN WSKIP
        BSTA,UN PARSE_EXPR                 ; [+1]
        LODA,R0 EXPH
        STRA,R0 GOTOH
        LODA,R0 EXPL
        STRA,R0 GOTOL
        LODI,R0 1               ; ISSUE-03 FIX: was EORZ/STRA ($00)  must be $01
        STRA,R0 GOTOFLG
        LODA,R0 RUNFLG                   ; OPT-10: RUNFLG=0EQ, 1GT
        RETC,GT                  ; return if running (was COMI $01/RETC,EQ)
        BCTA,UN CLR_RUNFLG

; =============================================================================
; DO_LIST: Lists program lines from PROG to PEH:PEL
DO_LIST:
        LODI,R0 <PROG
        STRA,R0 IPH                      ; Use IPH instead of TMPH
        LODI,R0 >PROG
        STRA,R0 IPL                      ; Use IPL instead of TMPL
DLS_LP:
        ; unsigned 16-bit: if IPH:IPL >= PEH:PEL  done
        LODA,R0 IPH
        SUBA,R0 PEH                      ; signed OK: PEH < $80
        RETC,GT                          ; IPH > PEH  past end
        BCTR,LT DLS_BODY                 ; IPH < PEH  before end
        LODA,R0 IPL
        SUBA,R0 PEL
        TPSL $01                         ; C=1 (no borrow) means IPL >= PEL
        RETC,EQ                          ; CC=EQ means C=1  done
DLS_BODY:
        ; Fetch line number hi:lo directly into EXPH:EXPL using IPH
        LODA,R0 *IPH
        STRA,R0 EXPH
        BSTA,UN INC_IP  ; advance
        LODA,R0 *IPH
        STRA,R0 EXPL
        BSTA,UN INC_IP  ; advance to string data

        BSTA,UN PRINT_S16               ; Print Line Number
        BSTA,UN PRT_SPACE               ; Print Space delimiter
 
        ; IPH:IPL is already pointing at the string data so
        ; print body bytes until CR (CR-terminated format)
DLS_BLPX:
        LODA,R0 *IPH
        COMI,R0 CR      ; end of line
        BCTR,EQ DLS_NL
        BSTA,UN COUT
        BSTA,UN INC_IP
        BCTR,UN DLS_BLPX  

DLS_NL:
        BSTA,UN INC_IP                   ; skip past CR 
        BSTA,UN PRT_CR
        BSTA,UN PRT_LF
        BCTA,UN DLS_LP

; =============================================================================
;  DO_RUN 
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
        RETC,EQ
        ; end of program? unsigned 16-bit: TMPH:TMPL >= PEH:PEL  stop
        ; OPT-4: PEH is always $1A-$1B (<$80) so signed SUBA gives same CC as unsigned for hi byte
        LODA,R0 TMPH
        SUBA,R0 PEH                      ; signed OK: PEH < $80
        BCTA,GT DR_STOP                  ; TMPH > PEH  past end
        BCTR,LT DR_EXEC                  ; TMPH < PEH  before end
        ; TMPH == PEH: lo byte
        LODA,R0 TMPL
        SUBA,R0 PEL
        TPSL $01                         ; C=1  no borrow  TMPL >= PEL  at/past end
        RETC,EQ                          ; done if TMPL >= PEL
DR_EXEC:
        ; save line number for error reporting
        LODA,R0 *TMPH
        STRA,R0 CURH
        BSTA,UN INC_TMP
        LODA,R0 *TMPH
        STRA,R0 CURL
        BSTA,UN INC_TMP
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
        EORZ,R0
        STRA,R0 *IPH  ; NUL-terminate IBUF
        
        ; SC0:SC1. SC0 and SC1 are scratch bytes clobbered by STMT_EXEC (used
        ; by PRINT_S16, STORE_LINE, parser, etc.).  SWSTK is the GOSUB return
        ; SWSP=$FF (empty) and GOSUB is not yet implemented.
        LODA,R0 TMPH
        STRA,R0 SC0      ; SC0:SC1 still set (DO_GOSUB reads them for return addr)
        ; address stored AT SWSTK ($012E:$012F), not into SWSTK itself. After CLRV
        ; SWSTK contains $0000, so the next-line pointer hi byte was written into
        ; PIPBUG ROM at $0000. Fix: direct STRA,R0 SWSTK stores into $012E.
        STRA,R0 SWSTK    ; NLP_H: save hi byte of next-line ptr directly into $012E
        LODA,R0 TMPL
        STRA,R0 SC1
        ; OPT-2: R0 still = TMPL here (STRA does not change R0)
        STRA,R0 SWSTK+1  ; NLP_L: save lo byte directly into $012F
        ; execute line
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
        BSTA,UN STMT_EXEC                ; [+1]
        ; check GOTO/GOSUB/RETURN flag
        LODA,R0 GOTOFLG                  ; OPT-10: GOTOFLG=0EQ, 1GT
        BCTR,GT DR_GOTO          ; branch if GOTO pending (was COMI $01/BCTR,EQ)
        ; advance: restore next-line pointer from SWSTK[0:1] (SC0:SC1 clobbered)
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
        BCTA,UN DR_LP
DR_STOP:
        ; drop through
; =============================================================================
; CLR_RUNFLG - Global Clears runflag helper
CLR_RUNFLG:
        EORZ,R0 ; Clear R0
        STRA,R0 RUNFLG
        RETC,UN

; =============================================================================
;  TRY_STORE_LINE 
; If IP starts with a digit, parse and store/delete the numbered line.
TRY_STORE_LINE:
        LODA,R0 *IPH
        COMI,R0 A'0'
        RETC,LT
        COMI,R0 A'9'+1
        BCTR,LT TSL_NUM
        RETC,UN
TSL_NUM:
        BSTA,UN WSKIP
        BSTA,UN PARSE_U16                ; [+1]
        LODA,R0 EXPH
        BCTR,GT TSL_NZ
        LODA,R0 EXPL
        RETC,EQ
TSL_NZ:
        LODA,R0 EXPH
        STRA,R0 LNUMH
        LODA,R0 EXPL
        STRA,R0 LNUML
        BSTA,UN WSKIP                    ; [+1]  skip space after line number
        LODA,R0 *IPH
        BCTR,EQ TSL_DEL
        BSTR,UN STORE_LINE               ; [+1]
        BCTR,UN TSL_DONE
TSL_DEL:
        BSTA,UN DELETE_LINE              ; [+1]
TSL_DONE:
        LODI,R0 1
        STRA,R0 ERRFLG
        RETC,UN

; =============================================================================
;  STORE_LINE 
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
        BCTR,EQ SL_MEASD
        BSTA,UN INC_TMP
        BIRR,R3 SL_MEAS         ; R3++ always (counts: 012...)
SL_MEASD:
        ; R3 = body length.  SC0 = body len.  SC1 = record size = 2+bodylen+1 (hi:lo:body:CR).
        STRA,R3 SC0
        LODA,R0 SC0
        ADDI,R0 3
        STRA,R0 SC1
        ; TMPH:TMPL already at body start  restore for space-check then write

        ; check free space: PROGLIM - PE >= SC1 (scratch: CURH:CURL  safe during entry)
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
        BCTR,GT SL_ROOM
        LODA,R0 CURL
        SUBA,R0 SC1
        BCFR,LT SL_ROOM  ; free >= needed?
        ; drop through
JERROOM:
        LODI,R0 ERR_OOM
        BCTA,UN DO_ERROR  ; out of memory

SL_ROOM:
        ; find sorted insertion point (FIND_INS clobbers TMPH:TMPL  that is fine,
        ; body start is in IP which survives, line number is in LNUMH:LNUML)
        BSTA,UN FIND_INS                 ; [+1]  result  TMPH:TMPL
        ; save insertion point in EXPH:EXPL
        LODA,R0 TMPH
        STRA,R0 EXPH
        LODA,R0 TMPL
        STRA,R0 EXPL

        ; save line number to CURH:CURL before shift loop clobbers LNUMH:LNUML
        LODA,R0 LNUMH
        STRA,R0 CURH
        LODA,R0 LNUML
        STRA,R0 CURL

        ; --- START GOLFED SHIFT LOOP ---
        ; Setup src pointer (LNUMH:LNUML) = PE
        LODA,R0 PEH
        STRA,R0 LNUMH
        LODA,R0 PEL
        STRA,R0 LNUML

        ; Setup dst pointer (GOTOH:GOTOL) = PE + SC1 (new record size)
        ADDA,R0 SC1             ; R0 is still PEL
        STRA,R0 GOTOL
        LODA,R0 PEH
        TPSL $01                ; Check carry from low-byte add
        BCTR,LT SL_DNC          ; Branch if C=0 (no carry)
        ADDI,R0 1               ; Add carry to high byte
SL_DNC:
        STRA,R0 GOTOH

SL_SHLOOP:
        ; Check-before-decrement. src starts at PE (one past last byte).
        ; Loop stops when src has been decremented to the insertion point,
        ; having moved all bytes in [ins .. PE-1] up by SC1.
        ; Full 16-bit signed comparison covering all six hi/lo cases:
        LODA,R0 LNUMH
        SUBA,R0 EXPH
        BCTR,GT SL_DOMOV         ; src.hi > ins.hi: keep moving
        BCTR,LT SL_NOSHIFT       ; src.hi < ins.hi: done
        LODA,R0 LNUML            ; src.hi == ins.hi: check lo
        SUBA,R0 EXPL
        BCTR,GT SL_DOMOV         ; src.lo > ins.lo: keep moving
        BCTR,UN SL_NOSHIFT       ; src.lo <= ins.lo: done (EQ=at ins, LT=overshot)
SL_DOMOV:
        ; Pre-decrement src (LNUMH:LNUML)
        LODA,R0 LNUML
        SUBI,R0 1
        STRA,R0 LNUML
        TPSL $01                ; C=1 = no borrow, C=0 = borrow
        BCTR,EQ SL_SNC          ; C=1 (no borrow): skip hi-byte dec
        LODA,R0 LNUMH
        SUBI,R0 1
        STRA,R0 LNUMH
SL_SNC:
        ; Pre-decrement dst (GOTOH:GOTOL)
        LODA,R0 GOTOL
        SUBI,R0 1
        STRA,R0 GOTOL
        TPSL $01                ; C=1 = no borrow, C=0 = borrow
        BCTR,EQ SL_DNC2         ; C=1 (no borrow): skip hi-byte dec
        LODA,R0 GOTOH
        SUBI,R0 1
        STRA,R0 GOTOH
SL_DNC2:
        ; Move byte *dst = *src
        LODA,R1 *LNUMH
        STRA,R1 *GOTOH
        BCTA,UN SL_SHLOOP        ; loop back to check (absolute: TPSL added 4 bytes, BCTR out of range)
        ; --- END GOLFED SHIFT LOOP ---

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
        LODA,R0 LNUML
        STRA,R0 *EXPH  ; write line lo
        BSTA,UN INC_EXP

        ; write body bytes until NUL (CR-terminated format  no bodylen byte)
SL_WBODY:
        LODA,R1 *TMPH
        BCTR,EQ SL_WDONE
        BSTA,UN TMP2EXP
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
        RETC,LT                  ; C=0 (no carry)  done
        LODA,R0 PEH
        ADDI,R0 1
        STRA,R0 PEH
        RETC,UN

; =============================================================================
;  DELETE_LINE 
DELETE_LINE:
        BSTA,UN FIND_LINE                ; [+1]
        LODA,R0 ERRFLG
        BCTR,EQ DL2_FOUND               ; ERRFLG=0 (EQ)  found
        RETC,UN                          ; not found  silent return
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

        ; copy TMPH:TMPL..PE-1 to EXPH:EXPL
DL2_LP:
        LODA,R0 TMPH
        SUBA,R0 PEH                      ; signed OK: PEH=$1A < $80
        BCTR,GT DL2_DONE
        BCTR,LT DL2_MOV
        ; TMPH == PEH: unsigned lo via carry
        LODA,R0 TMPL
        SUBA,R0 PEL
        TPSL $01                         ; C=1  TMPL >= PEL  done
        BCTR,EQ DL2_DONE
DL2_MOV:
        BSTR,UN TMP2EXP
        BCTR,UN DL2_LP
DL2_DONE:
        ; PE -= SC0
        LODA,R0 PEL
        SUBA,R0 SC0
        STRA,R0 PEL
        TPSL $01                 ; CC=EQ if C=1 (no borrow), CC=LT if C=0 (borrow)
        RETC,EQ                  ; no borrow  done
        LODA,R0 PEH
        SUBI,R0 1
        STRA,R0 PEH
        RETC,UN

; =============================================================================
; TMP2EXP Heper - Copy Single byte *EXP++ = *TMP++; 
TMP2EXP:
        LODA,R1 *TMPH
        STRA,R1 *EXPH
        BSTA,UN INC_TMP
        BSTA,UN INC_EXP
        RETC,UN

; =============================================================================
;  FIND_LINE 
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
        TPSL $01                         ; C=1  no borrow  TMPL >= PEL
        BCTR,EQ FL_RET_NF               ; TMPL >= PEL  at/past end (C=1, CC=EQ)
        BCTR,UN FL_CHK                   ; TMPL < PEL  check record
FL_CHK:
        ; Check exact match: *TMPH == LNUMH and *(TMPH:TMPL+1) == LNUML
        LODA,R0 *TMPH
        SUBA,R0 LNUMH
        BCTR,EQ FL_CHKLO
FL_RET_NF:
        ; BUG-NF-01 FIX: FL_RET_NF must return to caller with ERRFLG=$01 (not found).
        ; Previously FL_RET_NF fell into JERRLINE which jumped to DO_ERROR, jettisoning
        ; the RAS. DELETE_LINE calls FIND_LINE and needs a normal return when the line
        ; doesn't exist (silent no-op). JERRLINE is kept as a separate label for callers
        ; (DR_GOTO) that always want an error on not-found.
        LODI,R0 1
        STRA,R0 ERRFLG
        RETC,UN
JERRLINE:
        LODI,R0 ERR_UND_LINE
        BCTA,UN DO_ERROR  ; undefined line  returns to REPL
FL_CHKLO:
        LODA,R0 TMPL
        ADDI,R0 1
        STRA,R0 EXPL
        LODA,R0 TMPH
        TPSL $01
        BCTR,LT FL_LH                    ; BUG-FL-01 FIX: C=0 (no carry)  hi unchanged.
                                          ; TPSL $01: CC=LT if C=0, CC=EQ if C=1.
                                          ; Original bug was RETC,LT here which skipped the STRA below.
        ADDI,R0 1                        ; C=1 (carry from ADDI): increment hi byte
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

; =============================================================================
;  FIND_INS 
; Find sorted insertion point for LNUMH:LNUML.
; Returns TMPH:TMPL = address of first record with line >= LNUMH:LNUML,
; or PEH:PEL if all lines are smaller (insert at end).
FIND_INS:
        LODI,R0 <PROG
        STRA,R0 TMPH
        LODI,R0 >PROG
        STRA,R0 TMPL
FI_LP:
        ; boundary check: TMPH:TMPL >= PEH:PEL  done (unsigned)
        LODA,R0 TMPH
        SUBA,R0 PEH                      ; signed OK: PEH=$1A always < $80
        RETC,GT
        BCTR,LT FI_CHK
        ; TMPH == PEH: unsigned lo via carry
        LODA,R0 TMPL
        SUBA,R0 PEL
        TPSL $01                         ; C=1  no borrow  TMPL >= PEL  done
        RETC,EQ                          ; C=1  at/past end
FI_CHK:
        ; Compare target (LNUMH:LNUML) vs stored record line number.
        ; Reverse subtraction: LNUMH - stored.hi avoids signed byte-wrap bug.
        ; (stored - target wraps when values span $7F, giving wrong CC)
        LODA,R0 LNUMH
        SUBA,R0 *TMPH                    ; LNUMH - stored.hi
        BCTR,GT FI_ADV                   ; target.hi > stored.hi  advance
        BCTR,LT FI_RET                   ; target.hi < stored.hi  insertion point
        ; hi bytes equal: check lo
        LODA,R0 TMPL
        ADDI,R0 1
        STRA,R0 EXPL
        LODA,R0 TMPH
        TPSL $01
        BCTR,LT FI_LH                   ; BUG-FL-01 FIX (mirror of FL_CHKLO): C=0  no carry, hi unchanged.
        ADDI,R0 1                        ; C=1: increment hi byte
FI_LH:
        STRA,R0 EXPH
        ; BUG-FL-02 FIX: lo-byte compare must be unsigned.
        ; SUBA gives CC from signed result: $A0-$0A=$96, bit7 set  CC=LT
        ; even though $A0 > $0A unsigned. Any line number with lo byte >= $80
        ; (i.e. lo >= 128, e.g. lines 128, 160, 185, 255 etc) was misrouted.
        ; Fix: PPSL $02 unsigned-compare mode (same pattern as DR_LP boundary).
        LODA,R0 LNUML
        PPSL $02                         ; unsigned compare mode
        COMA,R0 *EXPH                    ; unsigned: LNUML vs stored.lo byte
        CPSL $02                         ; restore signed mode
        BCTR,GT FI_ADV                   ; LNUML > stored.lo (unsigned)  advance
        ; LNUML <= stored.lo  insertion point (EQ = exact match)
FI_RET:
        RETC,UN
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



; == PARSER_RET ==============================================================
; Shared parser return: SW stack if active (R3!=$FF), else RAS (RETC,UN).
; R3=$FF = SW stack empty. EORI $FF: $FF->$00 (EQ->RETC), other->non-zero (BCTA SWRETURN).
; In:  R3 = SW stack pointer
; Out: returns to caller via RAS or SW stack
; Clobbers: R0
PARSER_RET:
        LODZ,R3                          ; R0 = R3 (SW stack pointer)
        EORI,R0 $FF                      ; $FF (empty) -> $00 EQ; else non-zero
        RETC,EQ                          ; R3=$FF means SW stack empty -> return via RAS
        BCTA,UN SWRETURN                 ; else pop return address from SW stack

; == PARSE_EXPR ===============================================================
; Recursive descent expression evaluator with SW stack.
; Handles: literals, variables (A-Z), unary +/-, parens, */% then +/-.
; Callers handle relational operators after this returns.
; In:  IPH:IPL = pointer to expression string
; Out: EXPH:EXPL = 16-bit signed result; ERRFLG cleared
; Clobbers: R0, R3, SAVEH, SAVEL, E1SAVH, E1SAVL, NEGFLG, SC0, SC1, TMPH, TMPL
PARSE_EXPR:
        EORZ,R0
        STRA,R0 ERRFLG
        LODI,R3 $FF                      ; SW stack empty sentinel (pre-increment: first write goes to SWBASE[0])
EXPR_AM_RAS:
EXPR_AM:
        LODI,R0 >EAM0_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <EAM0_RET
        STRA,R0 SWBASE,R3+
        BCTA,UN EAM_ATOM
EAM0_RET:
        LODI,R0 >EAM_HI0_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <EAM_HI0_RET
        STRA,R0 SWBASE,R3+
        BCTA,UN EAM_HI
EAM_HI0_RET:
EAM_LO_LOOP:
        BSTA,UN WSKIP
        LODA,R0 *IPH
        COMI,R0 A'+'
        BCTR,EQ EAM_PLUS
        COMI,R0 A'-'
        BCTA,EQ EAM_MINUS
        BCTR,UN PARSER_RET
EAM_PLUS:
        LODA,R0 EXPH
        STRA,R0 SAVEH
        LODA,R0 EXPL
        STRA,R0 SAVEL
        BSTA,UN INC_IP
        LODI,R0 >EAM_P_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <EAM_P_RET
        STRA,R0 SWBASE,R3+
        BCTA,UN EAM_ATOM
EAM_P_RET:
        LODI,R0 >EAM_PH_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <EAM_PH_RET
        STRA,R0 SWBASE,R3+
        BCTA,UN EAM_HI
EAM_PH_RET:
        CPSL $08
        LODA,R0 SAVEL
        ADDA,R0 EXPL
        STRA,R0 EXPL
        PPSL $08
        LODA,R0 SAVEH
        ADDA,R0 EXPH
        STRA,R0 EXPH
        CPSL $08
        BCTA,UN EAM_LO_LOOP
EAM_MINUS:
        LODA,R0 EXPH
        STRA,R0 SAVEH
        LODA,R0 EXPL
        STRA,R0 SAVEL
        BSTA,UN INC_IP
        LODI,R0 >EAM_M_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <EAM_M_RET
        STRA,R0 SWBASE,R3+
        BCTA,UN EAM_ATOM
EAM_M_RET:
        LODI,R0 >EAM_MH_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <EAM_MH_RET
        STRA,R0 SWBASE,R3+
        BCTR,UN EAM_HI
EAM_MH_RET:
        CPSL $08
        LODA,R0 SAVEL
        SUBA,R0 EXPL
        STRA,R0 EXPL
        PPSL $08
        LODA,R0 SAVEH
        SUBA,R0 EXPH
        STRA,R0 EXPH
        CPSL $08
        BCTA,UN EAM_LO_LOOP
EAM_HI:
        BSTA,UN WSKIP
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
        BSTA,UN INC_IP
        LODI,R0 >MU_AT_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <MU_AT_RET
        STRA,R0 SWBASE,R3+
        BCTA,UN EAM_ATOM
MU_AT_RET:
        LODA,R0 E1SAVH
        STRA,R0 TMPH
        LODA,R0 E1SAVL
        STRA,R0 TMPL
        BSTA,UN MUL16
        BCTA,UN EAM_HI
EAM_DIV:
        LODA,R0 EXPH
        STRA,R0 E1SAVH
        LODA,R0 EXPL
        STRA,R0 E1SAVL
        BSTA,UN INC_IP
        LODI,R0 >DV_AT_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <DV_AT_RET
        STRA,R0 SWBASE,R3+
        BCTA,UN EAM_ATOM
DV_AT_RET:
        LODA,R0 E1SAVH
        STRA,R0 TMPH
        LODA,R0 E1SAVL
        STRA,R0 TMPL
        BSTA,UN DIV16
        BCTA,UN EAM_HI
EAM_MOD:
        LODA,R0 EXPH
        STRA,R0 E1SAVH
        LODA,R0 EXPL
        STRA,R0 E1SAVL
        BSTA,UN INC_IP
        LODI,R0 >MD_AT_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <MD_AT_RET
        STRA,R0 SWBASE,R3+
        BCTR,UN EAM_ATOM
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
        BCTA,UN EAM_HI
EAM_ATOM:
        BSTA,UN WSKIP
        LODA,R0 *IPH
        COMI,R0 A'-'
        BCTR,EQ EAM_NEG
        COMI,R0 A'+'
        BCTR,EQ EAM_POS
        COMI,R0 A'('
        BCTR,EQ EAM_PAREN
        BSTA,UN PARSE_FACTOR
        BCTA,UN PARSER_RET
EAM_NEG:
        BSTA,UN INC_IP
        LODI,R0 >NEG_AT_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <NEG_AT_RET
        STRA,R0 SWBASE,R3+
        BCTR,UN EAM_ATOM
NEG_AT_RET:
        BSTA,UN NEG_EXP_BODY
        BCTA,UN PARSER_RET
EAM_POS:
        BSTA,UN INC_IP
        LODI,R0 >POS_AT_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <POS_AT_RET
        STRA,R0 SWBASE,R3+
        BCTR,UN EAM_ATOM
POS_AT_RET:
        BCTA,UN PARSER_RET
EAM_PAREN:
        BSTA,UN INC_IP
        LODI,R0 >EP_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <EP_RET
        STRA,R0 SWBASE,R3+
        BCTA,UN EXPR_AM
EP_RET:
        BSTA,UN WSKIP
        BSTA,UN INC_IP
        BCTA,UN PARSER_RET

; =============================================================================
PARSE_FACTOR:
        LODA,R0 *IPH
        ; RAS-FIX: inline UPCASE here instead of BSTA UPCASE (+1 slot).
        ; Equivalent to: if(r0>='a' && r0<='z') r0-=32
        COMI,R0 A'a'
        BCTR,LT PF_UC_DONE       ; < 'a'  already uppercase or not alpha
        COMI,R0 A'z'+1
        BCTR,GT PF_UC_DONE       ; > 'z'  not lowercase
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

; =============================================================================
; load variable value from VARS 
PF_LOADVAR:
        STRA,R0 SC0              
        BSTA,UN INC_IP           
        LODA,R0 SC0              
        SUBI,R0 A'A'             
        STRZ,R1                  ; (STRZ r moves R0 -> R1) R1 = Index (0-25)
        ADDZ,R1                  ; R0 = R0 + R1 = Index * 2
        STRZ,R1                  ; R1 = Index * 2 (0-50)
        LODA,R0 VARS,R1          ; Native Indexed Load Hi byte
        STRA,R0 EXPH             
        LODA,R0 VARS+1,R1        ; Native Indexed Load Lo byte
        STRA,R0 EXPL             
        EORZ,R0 
        STRA,R0 ERRFLG           
        RETC,UN

; =============================================================================
;  PARSE_RELOP 
; Scan relational operator(s) at IP, build bitmask in RELOP.
;   '<' sets bit 0 (LT=1), '=' sets bit 1 (EQ=2), '>' sets bit 2 (GT=4)
;   So: < =1  = =2  > =4  <= =3  <> =5  >= =6
; Jumps to JSYNERR if any relop error found
; Clobbers: R0, R1 (R1 used as mask accumulator)
; Input : IP at first char of relop
; Output: RELOP = bitmask, ERRFLG=$00 ok / $01 none
PARSE_RELOP:
        BSTA,UN WSKIP                    ; [+1] skip leading space
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
        ; not a relop char  stop
        LODZ,R1                          ; R0 = mask, CC set from R1 (EQ if no relop seen)
        BCTR,EQ PRO_NONE                 ; R1==0  no relop chars seen  error
        STRA,R0 RELOP                    ; store mask (R0 already holds it from LODZ,R1)
        RETC,UN
PRO_LT:
        IORI,R1 1                        ; set LT bit
        BCTR,UN PRO_JMP                  ; BUG-RAS-01 FIX: was BSTR (leaked RAS slot)
PRO_EQ:
        IORI,R1 2                        ; set EQ bit
        BCTR,UN PRO_JMP                  ; BUG-RAS-01 FIX: was BSTR (leaked RAS slot)
PRO_GT:
        IORI,R1 4                        ; set GT bit
PRO_JMP:
        BSTA,UN INC_IP
        BCTR,UN PRO_LP

PRO_NONE:
        BCTA,UN JSYNERR

; =============================================================================
PARSE_S16:
        ; and PARSE_S16 is only called from PARSE_FACTOR(PF_NUM) or DO_INPUT.
        ; DO_INPUT calls PARSE_S16 after RDLINE which starts a fresh buffer  no
        ; leading spaces possible. Saves 1 RAS slot on the hot path:
        ;   REPLSTMT_EXECDO_PRINTPARSE_EXPRPARSE_FACTORPARSE_S16PARSE_U16
        ;   ...PARSE_S16PARSE_U16INC_IP = 6 (SP=6, safe).
        EORZ,R0 ; Clear R0
        STRA,R0 NEGFLG
        LODA,R0 *IPH
        COMI,R0 A'-'
        BCTR,EQ PS16_NEG
        BCTR,UN PS16_UN
PS16_NEG:
        BSTA,UN INC_IP
        LODI,R0 1
        STRA,R0 NEGFLG
PS16_UN:
        BSTR,UN PARSE_U16                ; [+1]
        BCTA,UN NEG_EXP                  ; OPT-15: negate if NEGFLG set (tail-call)

; =============================================================================
;  PARSE_U16 
; Parse unsigned decimal digits  EXPH:EXPL.
; Jumps to JSYNERR if no digits found (caller no longer needs ERRFLG check).
; RAS-FIX: WSKIP removed entirely from PARSE_U16. All callers must
; call WSKIP before invoking PARSE_U16 (PARSE_S16 does; DO_GOTO and
PARSE_U16:
        EORZ,R0 ; Clear R0
        STRA,R0 EXPH
        STRA,R0 EXPL
        ; Check first char  must be a digit or it is a syntax error.
        LODA,R0 *IPH
        COMI,R0 A'0'
        BCTA,LT JSYNERR          ; < '0'  not a digit  syntax error
        COMI,R0 A'9'+1
        BCTA,GT JSYNERR          ; > '9'  not a digit  syntax error
        ; First digit confirmed  fall into digit loop
PU16_LP:
        LODA,R0 *IPH
        COMI,R0 A'0'
        RETC,LT                  ; < '0'  end of number (valid exit)
        COMI,R0 A'9'+1
        BCTR,LT PU16_DIG
        RETC,UN                  ; > '9'  end of number (valid exit)
PU16_DIG:
        SUBI,R0 A'0'
        STRA,R0 SC0  ; digit value 0-9
        ; INC_IP inlined to save RAS slot (deepest path: DO_RUNSTMT_EXECDO_IF
        ; PARSE_EXPRPARSE_FACTORPARSE_S16PARSE_U16INC_IP would overflow SP=7)
        LODA,R0 IPL
        ADDI,R0 1
        STRA,R0 IPL
        TPSL $01
        BCTR,LT PU16_DNC
        LODA,R0 IPH
        ADDI,R0 1
        STRA,R0 IPH
PU16_DNC:
        ; Save SW stack pointer R3 before clobbering with loop counter
        STRA,R3 R3SAVE
        LODA,R0 EXPH
        STRA,R0 TMPH
        LODA,R0 EXPL
        STRA,R0 TMPL  ; TMPH:TMPL = old EXP
        EORZ,R0 ; Clear R0
        STRA,R0 EXPH
        STRA,R0 EXPL
        LODI,R3 10              ; 10 iterations: R3 counts 109...10exit
PU16_M10:
        LODA,R0 EXPL
        ADDA,R0 TMPL
        STRA,R0 EXPL
        TPSL $01
        BCTR,LT PU16_MNC         ; branch if C=0 (no carry)
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
PU16_MNC:
        LODA,R0 EXPH
        ADDA,R0 TMPH
        STRA,R0 EXPH
        BDRR,R3 PU16_M10       ; R3--; if R3!=0 branch
        ; Restore SW stack pointer
        LODA,R3 R3SAVE
        ; EXP += digit
        LODA,R0 EXPL
        ADDA,R0 SC0
        STRA,R0 EXPL
        TPSL $01
        BCTR,LT PU16_DIG_NC      ; branch if C=0 (no carry)
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
PU16_DIG_NC:
        BCTA,UN PU16_LP

; =============================================================================
; NEG_EXP: negate EXPH:EXPL if NEGFLG=0. Clobbers R0, R1.
; NEG_EXP_BODY: unconditional negate. Clobbers R0, R1.
NEG_EXP:
        LODA,R0 NEGFLG
        RETC,EQ                          ; NEGFLG=0  nothing to do
NEG_EXP_BODY:                            ; entry for unconditional negate
        LODI,R1 4                        ; R1 = 4 (Offset for EXPH/EXPL from IP)
        BCTR,UN NEG_SHARED

; =============================================================================
; ABS_TMP: abs(TMPH:TMPL) in place; set NEGFLG=1 if was negative. 
; Input:  TMPH:TMPL = signed 16-bit value, NEGFLG already cleared by caller.
; Output: TMPH:TMPL = |input|, NEGFLG=1 if input was negative.
; Clobbers: R0, R1.
ABS_TMP:
        LODA,R0 TMPH
        ANDI,R0 $80
        RETC,EQ                          ; positive     return, NEGFLG unchanged
        LODI,R0 1
        STRA,R0 NEGFLG                   ; set flag BEFORE tail-call
        LODI,R1 2                        ; R1 = 2 (Offset for TMPH/TMPL from IP)
        ; Fall through to NEG_SHARED

; --- Shared Negation Core ---
; In: R1 = offset (4 for EXP, 2 for TMP)
; Out: 1's complement applied. Jumps to INC_ET for 2's complement.
NEG_SHARED:
        LODA,R0 IPH,R1                   ; Accesses IPH+4 (EXPH) or IPH+2 (TMPH)
        EORI,R0 $FF
        STRA,R0 IPH,R1
        LODA,R0 IPL,R1                   ; Accesses IPL+4 (EXPL) or IPL+2 (TMPL)
        EORI,R0 $FF
        STRA,R0 IPL,R1
        LODZ R1                          ; R0 <- R1 (1 byte move! Puts offset in R0 for INC_ET)
        BCTA,UN INC_ET                   ; Tail-call to your existing bank-switching INC routine

; =============================================================================
; ABS_EXP     absolute value of EXPH:EXPL; toggle NEGFLG if input was negative.
; In:  EXPH:EXPL = signed 16-bit.
; Out: EXPH:EXPL = |value|; NEGFLG toggled if was negative, else unchanged.
; Clobbers: R0, R1.
ABS_EXP:
        LODA,R0 EXPH
        ANDI,R0 $80
        RETC,EQ                          ; positive     return, NEGFLG unchanged
        LODA,R0 NEGFLG
        EORI,R0 $01
        STRA,R0 NEGFLG                   ; toggle flag BEFORE tail-call
        LODI,R1 4                        ; R1 = 4 (Offset for EXPH/EXPL from IP)
        BCTR,UN NEG_SHARED

; =============================================================================
;  MUL16 
; Signed TMPH:TMPL  EXPH:EXPL  EXPH:EXPL  (16-bit two's complement wrap)
MUL16:
        EORZ,R0 ; Clear R0
        STRA,R0 NEGFLG
        BSTR,UN ABS_TMP                  ; OPT-15: abs(left), set NEGFLG=1 if neg
MU_LA:
        BSTR,UN ABS_EXP                  ; OPT-15: abs(right), toggle NEGFLG if neg
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
        BCTR,GT MU_ADD
        LODA,R0 TMPL
        BCTR,EQ MU_DONE
MU_ADD:
        LODA,R0 EXPL
        ADDA,R0 SC1
        STRA,R0 EXPL
        TPSL $01
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
        BSTA,UN NEG_EXP                  ; OPT-15: negate result if NEGFLG set
        EORZ,R0
        STRA,R0 NEGFLG                   ; exit  
        RETC,UN

; =============================================================================
;  DIV16 
; Signed TMPH:TMPL  EXPH:EXPL  EXPH:EXPL  (truncate toward zero)
; ERRFLG=$01 and DO_ERROR called on divide-by-zero.
DIV16:
        EORZ,R0 ; Clear R0
        STRA,R0 ERRFLG
        LODA,R0 EXPH
        BCTR,GT DV_NZ
        BCTR,LT DV_NZ
        LODA,R0 EXPL
        BCTA,EQ JERRDIVZER
DV_NZ:
        EORZ,R0 ; Clear R0
        STRA,R0 NEGFLG
        BSTA,UN ABS_TMP                  ; OPT-15: abs(dividend), set NEGFLG=1 if neg
DV_DA:
        BSTA,UN ABS_EXP                  ; OPT-15: abs(divisor), toggle NEGFLG if neg
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
        BCTR,LT DV_DONE           ; TMPH < SC0 (signed OK if SC0 < $80)  done
        BCTR,GT DV_SUB            ; TMPH > SC0  subtract
        ; TMPH == SC0: unsigned lo comparison via carry
        LODA,R0 TMPL
        SUBA,R0 SC1
        TPSL $01                  ; C=1  no borrow  TMPL >= SC1  subtract
        BCTR,EQ DV_SUB            ; C=1  TMPL >= SC1  continue subtract
        BCTR,UN DV_DONE           ; C=0  TMPL < SC1  done
DV_SUB:
        LODA,R0 TMPL
        SUBA,R0 SC1
        STRA,R0 TMPL
        TPSL $01                  ; C=1  no borrow  skip hi decrement
        BCTR,EQ DV_SNB            ; C=1  no borrow
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
        BSTA,UN NEG_EXP                  ; OPT-15: negate result if NEGFLG set
        EORZ,R0
        STRA,R0 NEGFLG                   ; exit  
        RETC,UN
JERRDIVZER:
        LODI,R0 ERR_DIV_ZERO
        BCTA,UN DO_ERROR  ; divide by zero error

; =============================================================================
;  PRINT_S16 
; Recursive Print signed 16-bit value EXPH:EXPL as decimal.
; clobbers TMP
PRINT_S16:
        ; Save caller R3 and switch to dedicated recursive print SW stack.
        STRA,R3 R3SAVE
        LODI,R3 $FF                      ; SW stack empty sentinel

        LODA,R0 EXPH
        ANDI,R0 $80
        BCTR,EQ PS_POS
        LODI,R0 A'-'
        BSTA,UN COUT
        LODA,R0 EXPH
        COMI,R0 $80
        BCTR,EQ PS_CHKMIN
PS_NEGNORM:
        BSTA,UN NEG_EXP_BODY             ; negate EXPH:EXPL then fall into PS_POS
        BCTR,UN PS_POS
PS_CHKMIN:
        LODA,R0 EXPL
        BCTR,EQ PS_MIN
        BCTR,UN PS_NEGNORM
PS_MIN:
        LODI,R0 A'3'
        BSTA,UN COUT
        LODI,R0 A'2'
        BSTA,UN COUT
        LODI,R0 A'7'
        BSTA,UN COUT
        LODI,R0 A'6'
        BSTA,UN COUT
        LODI,R0 A'8'
        BCTA,UN COUT

PS_POS:
        LODA,R0 EXPH
        BCTR,GT PS_NZ
        BCTR,LT PS_NZ
        LODA,R0 EXPL
        BCTR,EQ PS_ZERO
        BCTR,UN PS_NZ
PS_ZERO:
        LODI,R0 A'0'
        BCTA,UN COUT

PS_NZ:
        ; SWJSR: push PS_DONE return addr, drop into PREC
        LODI,R0 >PS_DONE
        STRA,R0 SWBASE,R3+
        LODI,R0 <PS_DONE
        STRA,R0 SWBASE,R3+
        ; fall through into PREC

; PREC  SW recursive digit printer (divide EXP by 10, recurse, print)
PREC:
        LODA,R0 EXPH
        STRA,R0 TMPH
        LODA,R0 EXPL
        STRA,R0 TMPL            ; dividend  TMPH:TMPL
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 EXPL            ; quotient = 0
        STRA,R0 NEGFLG
        STRA,R0 SC1             ; remainder = 0
        LODI,R0 16
        STRA,R0 SC0             ; loop counter
PR_LP:
        PPSL $08
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
        CPSL $08

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
        LODA,R0 SWBASE,R3                ; read digit (at current R3, last pre-incremented slot)
        SUBI,R3 1                        ; restore R3
        ADDI,R0 A'0'
        BSTA,UN COUT
        ; drop through
; =============================================================================
; Returns from SW return Stack
SWRETURN:
        LODA,R0 SWBASE,R3                ; read hi byte (at current R3, last pre-incremented slot)
        STRA,R0 TEMPRETH
        SUBI,R3 1                        ; move to lo byte slot
        LODA,R0 SWBASE,R3                ; read lo byte
        STRA,R0 TEMPRETL
        SUBI,R3 1                        ; restore R3 to pre-push state
        BCTA,UN *TEMPRETH

PS_DONE:
        ; restore caller R3 and return
        LODZ,R3
        LODA,R3 R3SAVE
        RETC,UN

; =============================================================================
;  RDLINE 
; Read a line from input into IBUF, echo with backspace support. NUL-terminates.
; Uses PIPBUG CHIN for blocking input. Char received in R0 at each step;
; saved to R1 for storage/echo so R0 is free for pointer arithmetic.
RDLINE:
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
RL_LP:
        BSTA,UN CHIN          ; [+1] blocking  R0 = char
        COMI,R0 NUL
        BCTA,EQ RL_EOL          ;   Treat as end-of-line so we don't flood IBUF
        ;                       ;   (and overwrite VARS) after stdin is exhausted.
        STRZ,R1                 ; R1 = char (R0 still has char for CR/BS checks)
        COMI,R1 CR
        BCTA,EQ RL_EOL
        COMI,R1 LF
        BCTA,EQ RL_EOL
        ; ISSUE-06 FIX: removed redundant second COMI,R1 NUL / BCTA,EQ RL_EOL here.
        ; catches EOF before we reach this point  second check was dead code.
        COMI,R1 BS
        BCTR,EQ RL_BS
        ; buffer full?  IP >= IBUF+63
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
        ; at IBUF start?  no backspace if buffer empty
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
        BSTA,UN PRT_BS
        BSTA,UN PRT_SPACE
        BSTA,UN PRT_BS
        BCTA,UN RL_LP
RL_EOL:
        EORZ,R0
        STRA,R0 *IPH            ; NUL-terminate buffer
        BSTA,UN PRT_CR
        BCTA,UN PRT_LF

;  WSKIP 
; Skips spaces
WSKIP:
        LODA,R0 *IPH
        COMI,R0 SP
        BCTR,EQ WS_ADV
        RETC,UN
WS_ADV:
        BSTR,UN INC_IP
        BCTR,UN WSKIP

; =============================================================================
;  GETCI_UC 
; Read *IPH uppercase into R0, advance IP.
; across the INC_IP call using STRZ,R1 / LODZ,R1 sandwich.
; Clobbers: R1 (caller must not rely on R1 across GETCI_UC call)
GETCI_UC:
        LODA,R0 *IPH
        BSTR,UN UPCASE                   ; [+1] R0 = uppercased char
        STRZ,R1                          ; R1 = char (save before INC_IP clobbers R0)
        BSTR,UN INC_IP                   ; [+1] advance IP (clobbers R0)
        LODZ,R1                          ; R0 = char (restore)
        RETC,UN

; =============================================================================
;  UPCASE 
UPCASE:
        COMI,R0 A'a'
        RETC,LT
        COMI,R0 A'z'+1
        BCTR,LT UC_DO
        RETC,UN
UC_DO:
        SUBI,R0 32
        RETC,UN

; =============================================================================
;  EATWORD 
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

; =============================================================================
;  SHARED 16-BIT POINTER INCREMENT/DECREMENT SUBROUTINES 
; Regs must be in this order
; INC_IP  : IPH:IPL  += 1   (clobbers R0)
; INC_TMP : TMPH:TMPL += 1  (clobbers R0)
; INC_EXP : EXPH:EXPL += 1  (clobbers R0)
; DEC_TMP : TMPH:TMPL -= 1  (clobbers R0)
; Rule: NO BSTA inside these  must not consume extra RAS depth.
; Carry idiom: ADDI sets no-carry->GT, carry->EQ/LT.
;   BCTA,LT skip = branch on no-carry (C=0). BCFA,LT skip = branch on no-borrow (C=1).
; Borrow idiom: SUBI sets no-borrow->GT/EQ, borrow->LT.
;   BCFA,LT skip  =  skip hi-byte decrement if no borrow (C=1).

INC_EXP:
        LODI,R0 4               ; EXP is 4 bytes after IP    
        db $EC     ; consume next 2 bytes with COMA,R0 opcode 
INC_TMP:
        LODI,R0 2       ; TMP is 2 bytes after IP
        db $C4     ; consume next 1 bytes with COMI,R0 opcode 
INC_IP:
        EORZ,R0            ; 1 bytes
; Used offsets to 
INC_ET:
        PPSL RS             ; switch reg bank
        STRZ R1             ; get offset in R1    
        LODA,R0 IPL,R1     ; 3 bytes  (TMPL=base, R1=0TMP, R1=2EXP)
        ADDI,R0 1           ; 2 bytes
        STRA,R0 IPL,R1     ; 3 bytes
        TPSL $01            ; 2 bytes
        BCTR,LT ET_RET      ; 1 byte
        ;RETC,LT
        LODA,R0 IPH,R1     ; 3 bytes
        ADDI,R0 1           ; 2 bytes
        STRA,R0 IPH,R1     ; 3 bytes
ET_RET:
        CPSL RS               ; switch back  
        RETC,UN             ; 1 byte

; =============================================================================
; Decrement - only used in 1 place for now
DEC_IP:
        LODA,R0 IPL
        SUBI,R0 1
        STRA,R0 IPL
        ; BUG-DEC-01 FIX: SUBI C=1 means no-borrow (hi unchanged), C=0 means borrow (hi needs decrement).
        ; RETC,LT was wrong  LT fires on result sign, not carry. Use TPSL to isolate carry bit.
        ; TPSL $01: CC=EQ if C=1 (no borrow), CC=LT if C=0 (borrow). RETC,EQ returns on no-borrow.
        TPSL $01
        RETC,EQ                  ; C=1  no borrow  hi byte unchanged, return
        LODA,R0 IPH
        SUBI,R0 1
        STRA,R0 IPH
        RETC,UN

; =============================================================================
;  Shared Character PRINT routines 
PRT_BS:
        LODI,R0 BS
        db $EC     ; consume next 2 bytes with COMA,R0 opcode 
PRT_CR:
        LODI,R0 CR
        db $EC     ; consume next 2 bytes with COMA,R0 opcode 
PRT_LF:
        LODI,R0 LF
        db $EC     ; consume next 2 bytes with COMA,R0 opcode 
PRT_CHEV:
        LODI,R0 '>'
        db $EC     ; consume next 2 bytes with COMA,R0 opcode 
PRT_SPACE:
        LODI,R0 32
        db $EC     ; consume next 2 bytes with COMA,R0 opcode 
PRT_AT:
        LODI,R0 '@'
        db $EC     ; consume next 2 bytes with COMA,R0 opcode 
PRT_QUEST:
        LODI,R0 '?'
        BCTA,UN COUT    ; tail call

; =============================================================================
;  DO_ERROR 
; Entry: R0 = error code (0-5).
; Saves RUNFLG, clears all run state, prints "?n[@line]", jumps to REPL.
; This is a tail-jump (BCTA,UN DO_ERROR from callers), so it kills the full RAS.
DO_ERROR:
        STRA,R0 SC0                      ; save error code
        LODA,R0 RUNFLG
        STRA,R0 SC1  ; save run state
        ;EORZ,R0 ; Clear R0
        ;STRA,R0 RUNFLG  ; clear run
        BSTA,UN CLR_RUNFLG ; clear run
        LODI,R0 $FF
        STRA,R0 SWSP  ; clear GOSUB stack
        BSTR,UN PRT_QUEST
        LODA,R0 SC0
        BSTA,UN COUT    ; print error number
        LODA,R0 SC1                      ; OPT-10: SC1=saved RUNFLG, 0EQ, 1GT
        BCTR,GT DE_IN            ; was COMI $01/BCTR,EQ
        BCTR,UN DE_NL
DE_IN:
        BSTR,UN PRT_SPACE
        BSTR,UN PRT_AT
        LODA,R0 CURH
        STRA,R0 EXPH
        LODA,R0 CURL
        STRA,R0 EXPL
        BSTA,UN PRINT_S16                ; [+1]
DE_NL:
        BSTA,UN PRT_CR
        BSTA,UN PRT_LF
        BCTA,UN REPL                     ; jump to REPL  clears full hardware RAS

; =============================================================================
;  TABLES 
; Keyword table: [c1][c2][hi][lo]  NUL-terminated.
; hi:lo = address of handler routine. Matched on first two uppercase chars.
; SE_SCAN loads hi:lo, stores to TMPH:TMPL, branches via *TMPH (indirect jump).
; THEN matched internally by DO_IF  not dispatched here.
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

ROMEND: ; so we can measure Binary rom size

; =============================================================================
; Pre-loaded showcase program  
;
;   Stored as raw ASCII.  Line format: <lineno_lo> <lineno_hi> <body> <CR>
;
;   Lines  10-260: feature demos (PRINT, CHR$, arithmetic, comparisons, loops)
;   Lines 270-480: Mandelbrot set renderer
;
;   v1.1: Mandelbrot column scan adjusted from -128..16 to -120..4 for a
;         better-centred render.
; =============================================================================
         ORG PROG

         DB $00,$0A,$52,$45,$4D,$20,$75,$42,$41,$53,$49,$43,$20,$2D,$20,$53,$48,$4F,$57,$43,$41,$53,$45,$0D  ; 10 REM uBASIC - SHOWCASE
         DB $00,$14,$50,$52,$49,$4E,$54,$20,$22,$2D,$2D,$20,$75,$42,$41,$53,$49,$43,$20,$32,$36,$35,$30,$20,$56,$33,$2E,$30,$20,$53,$68,$6F,$77,$63,$61,$73,$65,$20,$2D,$2D,$22,$0D  ; 20 PRINT "-- uBASIC 2650 Vx.x Showcase --" 
         DB $00,$1E,$50,$52,$49,$4E,$54,$20,$22,$2D,$2D,$2D,$20,$50,$52,$49,$4E,$54,$20,$2F,$20,$43,$48,$52,$24,$20,$2D,$2D,$2D,$22,$0D  ; 30 PRINT "--- PRINT / CHR$ ---"
         DB $00,$28,$50,$52,$49,$4E,$54,$20,$43,$48,$52,$24,$28,$36,$35,$29,$3B,$43,$48,$52,$24,$28,$36,$36,$29,$3B,$43,$48,$52,$24,$28,$36,$37,$29,$0D  ; 40 PRINT CHR$(65);CHR$(66);CHR$(67)
         DB $00,$32,$50,$52,$49,$4E,$54,$20,$22,$2D,$2D,$2D,$20,$41,$52,$49,$54,$48,$4D,$45,$54,$49,$43,$20,$2D,$2D,$2D,$22,$0D  ; 50 PRINT "--- ARITHMETIC ---"
         DB $00,$3C,$50,$52,$49,$4E,$54,$20,$22,$33,$2B,$34,$3D,$22,$3B,$33,$2B,$34,$3B,$22,$20,$20,$31,$30,$2D,$33,$3D,$22,$3B,$31,$30,$2D,$33,$3B,$22,$20,$20,$36,$2A,$37,$3D,$22,$3B,$36,$2A,$37,$0D  ; 60 PRINT "3+4=";3+4;"  10-3=";10-3;"  6*7=";6*7
         DB $00,$46,$50,$52,$49,$4E,$54,$20,$22,$32,$30,$2F,$34,$3D,$22,$3B,$32,$30,$2F,$34,$3B,$22,$20,$20,$31,$37,$25,$35,$3D,$22,$3B,$31,$37,$25,$35,$0D  ; 70 PRINT "20/4=";20/4;"  17%5=";17%5
         DB $00,$50,$50,$52,$49,$4E,$54,$20,$22,$2D,$2D,$2D,$20,$43,$4F,$4D,$50,$41,$52,$49,$53,$4F,$4E,$53,$20,$2D,$2D,$2D,$22,$0D  ; 80 PRINT "--- COMPARISONS ---"
         DB $00,$5A,$49,$46,$20,$35,$3E,$33,$20,$54,$48,$45,$4E,$20,$50,$52,$49,$4E,$54,$20,$22,$35,$3E,$33,$20,$6F,$6B,$22,$0D  ; 90 IF 5>3 THEN PRINT "5>3 ok"
         DB $00,$64,$49,$46,$20,$33,$3C,$35,$20,$54,$48,$45,$4E,$20,$50,$52,$49,$4E,$54,$20,$22,$33,$3C,$35,$20,$6F,$6B,$22,$0D  ; 100 IF 3<5 THEN PRINT "3<5 ok"
         DB $00,$6E,$49,$46,$20,$33,$3E,$3D,$33,$20,$54,$48,$45,$4E,$20,$50,$52,$49,$4E,$54,$20,$22,$33,$3E,$3D,$33,$20,$6F,$6B,$22,$0D  ; 110 IF 3>=3 THEN PRINT "3>=3 ok"
         DB $00,$78,$49,$46,$20,$34,$3C,$3E,$33,$20,$54,$48,$45,$4E,$20,$50,$52,$49,$4E,$54,$20,$22,$34,$3C,$3E,$33,$20,$6F,$6B,$22,$0D  ; 120 IF 4<>3 THEN PRINT "4<>3 ok"
         DB $00,$82,$49,$46,$20,$33,$3D,$33,$20,$54,$48,$45,$4E,$20,$50,$52,$49,$4E,$54,$20,$22,$33,$3D,$33,$20,$6F,$6B,$22,$0D  ; 130 IF 3=3 THEN PRINT "3=3 ok"
         DB $00,$8C,$50,$52,$49,$4E,$54,$20,$22,$2D,$2D,$2D,$20,$4C,$4F,$4F,$50,$20,$76,$69,$61,$20,$47,$4F,$54,$4F,$20,$2D,$2D,$2D,$22,$0D  ; 140 PRINT "--- LOOP via GOTO ---"
         DB $00,$96,$49,$3D,$31,$0D                                                                                                          ; 150 I=1        
         DB $00,$A0,$49,$46,$20,$49,$3E,$35,$20,$54,$48,$45,$4E,$20,$47,$4F,$54,$4F,$20,$31,$39,$30,$0D                                      ; 160 IF I>5 THEN GOTO 190
         DB $00,$AA,$50,$52,$49,$4E,$54,$20,$49,$3B,$0D                                                                                      ; 170 PRINT I;
         DB $00,$B4,$49,$3D,$49,$2B,$31,$0D                                                                                                   ; 180 I=I+1     
         DB $00,$B9,$47,$4F,$54,$4F,$20,$31,$36,$30,$0D                                                                                      ; 185 GOTO 160
         DB $00,$BE,$50,$52,$49,$4E,$54,$20,$22,$22,$0D                                                                                      ; 190 PRINT ""
         DB $00,$C8,$50,$52,$49,$4E,$54,$20,$22,$2D,$2D,$2D,$20,$4E,$45,$53,$54,$45,$44,$20,$4C,$4F,$4F,$50,$20,$2D,$2D,$2D,$22,$0D          ; 200 PRINT "--- NESTED LOOP ---"
         DB $00,$D2,$49,$3D,$31,$0D                                                                                                           ; 210 I=1      
         DB $00,$DC,$49,$46,$20,$49,$3E,$33,$20,$54,$48,$45,$4E,$20,$47,$4F,$54,$4F,$20,$32,$37,$30,$0D                                      ; 220 IF I>3 THEN GOTO 270
         DB $00,$E6,$4A,$3D,$31,$0D                                                                                                           ; 230 J=1       
         DB $00,$F0,$49,$46,$20,$4A,$3E,$33,$20,$54,$48,$45,$4E,$20,$47,$4F,$54,$4F,$20,$32,$36,$30,$0D                                      ; 240 IF J>3 THEN GOTO 260
         DB $00,$FA,$50,$52,$49,$4E,$54,$20,$4A,$3B,$0D                                                                                      ; 250 PRINT J;
         DB $00,$FF,$4A,$3D,$4A,$2B,$31,$0D                                                                                                   ; 255 J=J+1      (split: was J=J+1:GOTO 240)
         DB $01,$00,$47,$4F,$54,$4F,$20,$32,$34,$30,$0D                                                                                      ; 256 GOTO 240   (split from 255)
         DB $01,$04,$50,$52,$49,$4E,$54,$20,$22,$22,$0D                                                                                      ; 260 PRINT ""   (split: was PRINT "":I=I+1:GOTO 220)
         DB $01,$05,$49,$3D,$49,$2B,$31,$0D                                                                                                   ; 261 I=I+1      (split from 260)
         DB $01,$06,$47,$4F,$54,$4F,$20,$32,$32,$30,$0D                                                                                      ; 262 GOTO 220   (split from 260)
         DB $01,$0E,$50,$52,$49,$4E,$54,$20,$22,$2D,$2D,$2D,$20,$4D,$41,$4E,$44,$45,$4C,$42,$52,$4F,$54,$20,$2D,$2D,$2D,$22,$0D             ; 270 PRINT "--- MANDELBROT ---"
         DB $01,$18,$49,$3D,$2D,$36,$34,$0D                                                                                                  ; 280 I=-64
         DB $01,$22,$49,$46,$20,$49,$3E,$35,$36,$20,$54,$48,$45,$4E,$20,$47,$4F,$54,$4F,$20,$34,$38,$30,$0D                                  ; 290 IF I>56 THEN GOTO 480
         DB $01,$2C,$44,$3D,$49,$0D                                                                                                          ; 300 D=I
         DB $01,$36,$43,$3D,$2D,$31,$32,$30,$0D                                                                                              ; 310 C=-120
         DB $01,$40,$49,$46,$20,$43,$3E,$34,$20,$54,$48,$45,$4E,$20,$47,$4F,$54,$4F,$20,$34,$35,$30,$0D                                      ; 320 IF C>4 THEN GOTO 450
         DB $01,$4A,$41,$3D,$43,$0D                                                                                                           ; 330 A=C        (split: was A=C:B=D:E=0:N=1)
         DB $01,$4B,$42,$3D,$44,$0D                                                                                                           ; 331 B=D        (split from 330)
         DB $01,$4C,$45,$3D,$30,$0D                                                                                                           ; 332 E=0        (split from 330)
         DB $01,$4D,$4E,$3D,$31,$0D                                                                                                           ; 333 N=1        (split from 330)
         DB $01,$54,$49,$46,$20,$4E,$3E,$31,$36,$20,$54,$48,$45,$4E,$20,$47,$4F,$54,$4F,$20,$33,$39,$30,$0D                                  ; 340 IF N>16 THEN GOTO 390
         DB $01,$5E,$49,$46,$20,$45,$3E,$30,$20,$54,$48,$45,$4E,$20,$47,$4F,$54,$4F,$20,$33,$38,$30,$0D                                      ; 350 IF E>0 THEN GOTO 380
         DB $01,$68,$54,$3D,$41,$2A,$41,$2F,$36,$34,$2D,$42,$2A,$42,$2F,$36,$34,$2B,$43,$0D                                                  ; 360 T=A*A/64-B*B/64+C
         DB $01,$72,$42,$3D,$32,$2A,$41,$2A,$42,$2F,$36,$34,$2B,$44,$0D                                                                      ; 370 B=2*A*B/64+D  (split: was B=...:A=T)
         DB $01,$73,$41,$3D,$54,$0D                                                                                                           ; 371 A=T        (split from 370)
         DB $01,$7C,$49,$46,$20,$41,$2A,$41,$2F,$36,$34,$2B,$42,$2A,$42,$2F,$36,$34,$3E,$32,$35,$36,$20,$54,$48,$45,$4E,$20,$49,$46,$20,$45,$3D,$30,$20,$54,$48,$45,$4E,$20,$45,$3D,$4E,$0D  ; 380 IF A*A/64+B*B/64>256 THEN IF E=0 THEN E=N
         DB $01,$86,$4E,$3D,$4E,$2B,$31,$0D                                                                                                   ; 390 N=N+1      (split: was N=N+1:IF N<=16...)
         DB $01,$87,$49,$46,$20,$4E,$3C,$3D,$31,$36,$20,$54,$48,$45,$4E,$20,$47,$4F,$54,$4F,$20,$33,$34,$30,$0D                              ; 391 IF N<=16 THEN GOTO 340  (split from 390)
         DB $01,$90,$49,$46,$20,$45,$3E,$30,$20,$54,$48,$45,$4E,$20,$50,$52,$49,$4E,$54,$20,$43,$48,$52,$24,$28,$45,$2B,$33,$32,$29,$3B,$0D  ; 400 IF E>0 THEN PRINT CHR$(E+32);
         DB $01,$9A,$49,$46,$20,$45,$3D,$30,$20,$54,$48,$45,$4E,$20,$50,$52,$49,$4E,$54,$20,$43,$48,$52,$24,$28,$33,$32,$29,$3B,$0D          ; 410 IF E=0 THEN PRINT CHR$(32);
         DB $01,$A4,$43,$3D,$43,$2B,$34,$0D                                                                                                  ; 420 C=C+4
         DB $01,$AE,$47,$4F,$54,$4F,$20,$33,$32,$30,$0D                                                                                      ; 430 GOTO 320
         DB $01,$C2,$50,$52,$49,$4E,$54,$0D                                                                                                  ; 450 PRINT
         DB $01,$CC,$49,$3D,$49,$2B,$36,$0D                                                                                                  ; 460 I=I+6
         DB $01,$D6,$47,$4F,$54,$4F,$20,$32,$39,$30,$0D                                                                                      ; 470 GOTO 290
         DB $01,$E0,$45,$4E,$44,$0D                                                                                                          ; 480 END

SHOWCASE_END:
        DB $D, $D, $D, $D

        END
