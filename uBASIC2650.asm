; uBASIC2650.asm  —  Tiny BASIC for Signetics 2650
; Version: v1.2-pipbug
;
; Change history:
;   v1.2  BUG-BASIC-01 FIXED: All < / > HI/LO operators corrected throughout.
;           66 lines had < (HIGH) and > (LOW) swapped. Convention is:
;             <ADDR = HIGH byte (bits 15:8),  >ADDR = LOW byte (bits 7:0)
;           Applied global swap from original v1.1 source for correct pointer
;           loads, arithmetic base addresses and buffer comparisons.
;         Requires asm2650 v1.4+ (BUG-ASM-01 same-line label fix needed).
;   v1.1  Initial PIPBUG 1 port.
;
; Target: PIPBUG 1 monitor (1kB ROM $0000-$03FF, 64B RAM $0400-$043F)
;   Program loads at $0440.  All RAM EQUs remain at $1400+ (plenty of room).
;   I/O via Pipbug ROM calls (replaces direct WRTD/REDE hardware instructions):
;     COUT $02B4  — putchar: R0 = char to print
;     CHIN $0286  — getchar blocking: returns R0=ASCII
;     CRLF $008A  — print CR+LF
;
; CC SEMANTICS (set_cc_add / set_cc_sub from sim2650):
;   ADD: no-carry->GT  carry+zero->EQ  carry+nonzero->LT
;   SUB: no-borrow+nonzero->GT  no-borrow+zero->EQ  borrow->LT
;   Carry  skip idiom: BCTA,GT lbl   (GT = C=0 = no carry)
;   Borrow skip idiom: BCFA,LT lbl   (NOT LT = C=1 = no borrow — covers GT+EQ)
;   NEVER use BCTA,GT for borrow skip — misses the zero-result case.
;
; 16-bit pointer advance inline pattern (preserves R1):
;   LODA,R0 PTR_L; ADDI,R0 1; STRA,R0 PTR_L; BCTA,GT lbl; LODA,R0 PTR_H; ADDI,R0 1; STRA,R0 PTR_H; lbl:
;
; DO_ERROR convention: LODI,R0 <code>; BCTA,UN DO_ERROR
;   DO_ERROR saves RUNFLG, clears state, then BCTA,UN REPL (kills full RAS).
;
; I/O CONVENTION (Pipbug):
;   Output: call BSTA,UN PUTCH with char already in R1 
;   Input:  BSTA,UN GETKEY — waits until press, returns char in R0
;   RDLINE: fills IBUF using GETKEY; echoes via COUT; result in R1 at each step
;
; RAS DEPTH BUDGET:
;   REPL(1) -> STMT_EXEC(2) -> DO_xxx(3) -> PARSE_EXPR(4) -> PARSE_FACTOR(5)
;     -> UPCASE(6)  [max 6 — safe within 8-level hardware RAS]
;   COUT/CHIN are intercepted by sim (or are short ROM calls) — they push
;   one level (BSTA) and pop on return: net depth = caller+1 at call site only.

; ─── ASCII ────────────────────────────────────────────────────────────────────
CR      EQU     $0D
LF      EQU     $0A
BS      EQU     $08
SP      EQU     $20
NUL     EQU     $00
DQ      EQU     $22

; ─── PIPBUG 1 I/O entry points ────────────────────────────────────────────────
COUT    EQU     $02B4   ; putchar: R0 = char to output
CHIN    EQU     $0286   ; getchar non-blocking: R0=0 if no key
CRLF    EQU     $008A   ; print CR+LF (no registers used/changed)

; ─── RAM $1400–$1BFF ──────────────────────────────────────────────────────────
IPH     EQU     $1400   ; interpreter pointer hi
IPL     EQU     $1401   ; interpreter pointer lo
PEH     EQU     $1402   ; program end pointer hi
PEL     EQU     $1403   ; program end pointer lo
RUNFLG  EQU     $1404   ; $01=running $00=immediate
GOTOFLG EQU     $1405   ; $01=GOTO/GOSUB pending
GOTOH   EQU     $1406   ; pending target line hi
GOTOL   EQU     $1407   ; pending target line lo
CURH    EQU     $1408   ; current line hi  (error reporting)
CURL    EQU     $1409   ; current line lo
LNUMH   EQU     $140A   ; scratch line number hi
LNUML   EQU     $140B   ; scratch line number lo
SC0     EQU     $140C   ; scratch byte 0
SC1     EQU     $140D   ; scratch byte 1
ERRFLG  EQU     $140E   ; error flag $00=ok
NEGFLG  EQU     $140F   ; sign / CHR$ flag
EXPH    EQU     $1410   ; expression result hi
EXPL    EQU     $1411   ; expression result lo
TMPH    EQU     $1412   ; temp 16-bit hi
TMPL    EQU     $1413   ; temp 16-bit lo
OPSTK   EQU     $1414   ; operator stack [8]  $1414-$141B
VALSH   EQU     $141C   ; value stack hi  [8]  $141C-$1423
VALSL   EQU     $1424   ; value stack lo  [8]  $1424-$142B
STKIDX  EQU     $142C   ; parser stack top ($FF=empty)
SWSP    EQU     $142D   ; SW call stack pointer ($FF=empty)
SWSTK   EQU     $142E   ; SW call stack 8×2 bytes  $142E-$143D
RELOP   EQU     $143E   ; relational op 1-6
IBUF    EQU     $1440   ; input buffer 64 bytes  $1440-$147F
VARS    EQU     $1480   ; A-Z variables 2 bytes each  $1480-$14B3
PROG    EQU     $14C0   ; program store base
PROGLIM EQU     $1C00   ; one past end of program store

; ─── CODE starts at $0440 (after Pipbug 1kB ROM + 64B RAM) ───────────────────
        ORG     $0440

; ─── RESET / ENTRY ────────────────────────────────────────────────────────────
RESET:
        LODI,R0 <PROG
        STRA,R0 PEH
        LODI,R0 >PROG
        STRA,R0 PEL
        LODI,R0 $FF
        STRA,R0 SWSP
        LODI,R0 $00
        STRA,R0 RUNFLG
        STRA,R0 GOTOFLG
        ; clear A-Z variables (52 bytes) using IPH:IPL as scratch pointer
        LODI,R0 <VARS
        STRA,R0 IPH
        LODI,R0 >VARS
        STRA,R0 IPL
        LODI,R3 $34
CLRV:
        LODI,R0 $00
        STRA,R0 *IPH
        BSTA,UN INC_IP
CLRV_NC:
        BRNR,R3 CLRV
        LODI,R0 <BANNER
        STRA,R0 IPH
        LODI,R0 >BANNER
        STRA,R0 IPL
        BSTA,UN PRTSTR

; ─── REPL ────────────────────────────────────────────────────────────────────
REPL:
        LODI,R0 A'>'
        BSTA,UN COUT
        LODI,R0 SP
        BSTA,UN COUT
        BSTA,UN RDLINE
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
        BSTA,UN TRY_STORE_LINE
        LODA,R0 ERRFLG
        COMI,R0 $01
        BCTA,EQ REPL
        BSTA,UN STMT_EXEC
        BCTA,UN REPL

; ─── TABLES ───────────────────────────────────────────────────────────────────
BANNER:
        DB CR, LF
        DB A'u',A'B',A'A',A'S',A'I',A'C',A' ',A'2',A'6',A'5',A'0',A' ',A'v',A'1',A'.',A'0'
        DB CR, LF, NUL

; Keyword table: [c1][c2][token]  NUL-terminated.
; Matched on first two uppercase chars; EATWORD skips the rest.
; Token 11 (THEN) matched internally by DO_IF — not dispatched here.
KW_TAB:
        DB A'P',A'R', 1   ; PRINT / PR
        DB A'L',A'E', 2   ; LET   / LE
        DB A'L',A'I', 3   ; LIST  / LI
        DB A'R',A'E', 4   ; REM   / RE
        DB A'R',A'U', 5   ; RUN   / RU
        DB A'E',A'N', 6   ; END   / EN
        DB A'I',A'N', 7   ; INPUT / IN
        DB A'I',A'F', 8   ; IF
        DB A'N',A'E', 9   ; NEW   / NE
        DB A'G',A'O',10   ; GOTO  / GO
        DB A'P',A'O',12   ; POKE  / PO
        DB NUL

; Divisor table for PRINT_S16: big-endian pairs, sentinel $0000
DIVTAB:
        DB $27,$10      ; 10000
        DB $03,$E8      ;  1000
        DB $00,$64      ;   100
        DB $00,$0A      ;    10
        DB $00,$00      ; sentinel

; ─── STMT_EXEC ────────────────────────────────────────────────────────────────
; Decode and dispatch one statement from IP.
; RAS depth: 1 from REPL, or 3 from DO_IF (THEN body).
; Worst inner depth from here: +4 (->DO_xxx->PARSE_EXPR->PARSE_FACTOR->UPCASE)
STMT_EXEC:
        BSTA,UN WSKIP                   ; [+1]
        LODA,R0 *IPH
        COMI,R0 NUL
        BCTA,EQ SE_RET  ; blank line

        BSTA,UN GETCI_UC
        STRA,R0 SC0  ; [+1] char1 uppercase, IP advanced
        BSTA,UN GETCI_UC
        STRA,R0 SC1  ; [+1] char2 uppercase, IP advanced

        ; scan KW_TAB with TMPH:TMPL as pointer
        LODI,R0 <KW_TAB
        STRA,R0 TMPH
        LODI,R0 >KW_TAB
        STRA,R0 TMPL
SE_SCAN:
        LODA,R0 *TMPH
        COMI,R0 NUL
        BCTA,EQ SE_SYNERR  ; end of table
        SUBA,R0 SC0
        BCTA,EQ SE_CHK2  ; c1 matches
        ; advance 3 bytes to next entry
        LODA,R0 TMPL
        ADDI,R0 3
        STRA,R0 TMPL
        BCTA,GT SE_SCAN
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
        BCTA,UN SE_SCAN
SE_CHK2:
        ; advance to c2
        BSTA,UN INC_TMP
SE_C2N:
        LODA,R0 *TMPH
        SUBA,R0 SC1
        BCTA,EQ SE_MATCH
        ; c2 mismatch: advance 2 more bytes to next entry
        LODA,R0 TMPL
        ADDI,R0 2
        STRA,R0 TMPL
        BCTA,GT SE_SCAN
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
        BCTA,UN SE_SCAN
SE_MATCH:
        ; advance to token byte
        BSTA,UN INC_TMP
SE_TN:
        LODA,R0 *TMPH
        STRA,R0 SC0  ; SC0 = token id
        BSTA,UN EATWORD                  ; [+1] consume remaining alpha chars

        ; dispatch
        LODA,R0 SC0
        COMI,R0  1
        BCTA,EQ DO_PRINT
        COMI,R0  2
        BCTA,EQ DO_LET
        COMI,R0  3
        BCTA,EQ DO_LIST
        COMI,R0  4
        BCTA,EQ DO_REM
        COMI,R0  5
        BCTA,EQ DO_RUN
        COMI,R0  6
        BCTA,EQ DO_END
        COMI,R0  7
        BCTA,EQ DO_INPUT
        COMI,R0  8
        BCTA,EQ DO_IF
        COMI,R0  9
        BCTA,EQ DO_NEW
        COMI,R0 10
        BCTA,EQ DO_GOTO
SE_SYNERR:
        LODI,R0 0
        BCTA,UN DO_ERROR
SE_RET:
        RETC,UN

; ─── SIMPLE STATEMENTS ────────────────────────────────────────────────────────
DO_REM:
        RETC,UN

DO_END:
        LODA,R0 RUNFLG
        COMI,R0 $00
        BCTA,EQ DE_HALT
        LODI,R0 $00
        STRA,R0 RUNFLG
        RETC,UN
DE_HALT:
        HALT

DO_NEW:
        LODI,R0 <PROG
        STRA,R0 PEH
        LODI,R0 >PROG
        STRA,R0 PEL
        RETC,UN

; ─── DO_PRINT ─────────────────────────────────────────────────────────────────
; PRINT [item {, item}]    item = "string" | expr
; CHR$ flag: NEGFLG=$01 after PARSE_FACTOR detects CHR$ — print EXPL as char.
DO_PRINT:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 NUL
        BCTA,EQ DP_NL

DP_ITEM:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 DQ
        BCTA,EQ DP_STRING
        LODI,R0 $00
        STRA,R0 NEGFLG  ; clear CHR$ flag before parse
        BSTA,UN PARSE_EXPR               ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DP_NUM
        BSTA,UN PRTSTR_IP
        BCTA,UN DP_NL  ; [+1] raw text fallback
DP_NUM:
        LODA,R0 NEGFLG
        COMI,R0 $01
        BCTA,EQ DP_CHAR
        BSTA,UN PRINT_S16
        BCTA,UN DP_SEP  ; [+1]
DP_CHAR:
        LODA,R0 EXPL
        BSTA,UN COUT
        BCTA,UN DP_SEP

DP_STRING:
        ; consume opening "
        BSTA,UN INC_IP
DP_SLP:
        LODA,R1 *IPH
        COMI,R1 NUL
        BCTA,EQ DP_SDONE
        COMI,R1 DQ
        BCTA,EQ DP_SCLS
        LODZ,R1
        BSTA,UN COUT
        BSTA,UN INC_IP
        BCTA,UN DP_SLP
DP_SCLS:
        ; consume closing "
        BSTA,UN INC_IP
DP_SDONE:
DP_SEP:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 A','
        BCTA,EQ DP_COMMA
        BCTA,UN DP_NL
DP_COMMA:
        BSTA,UN INC_IP
        BCTA,UN DP_ITEM
DP_NL:
        BSTA,UN CRLF
        RETC,UN

; ─── DO_LET / shared store path ───────────────────────────────────────────────
; DO_INPUT jumps to DL_STORE with SC0 = variable letter already set.
DO_LET:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        BSTA,UN UPCASE  ; [+1]
        COMI,R0 A'A'
        BCTA,LT DL_ERR
        COMI,R0 A'Z'+1
        BCTA,LT DL_VAROK
DL_ERR:
        LODI,R0 4
        BCTA,UN DO_ERROR
DL_VAROK:
        STRA,R0 SC0                      ; save variable letter
        BSTA,UN INC_IP
DL_EQ:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 A'='
        BCTA,EQ DL_EQC
        LODI,R0 0
        BCTA,UN DO_ERROR
DL_EQC:
        BSTA,UN INC_IP
DL_EX:
        BSTA,UN PARSE_EXPR               ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DL_STORE
        LODI,R0 0
        BCTA,UN DO_ERROR
DL_STORE:
        ; address = VARS + (SC0 - 'A') * 2
        LODA,R0 SC0
        SUBI,R0 A'A'  ; 0-25
        STRA,R0 SC1
        ADDA,R0 SC1  ; *2  (SC1 = index, R0 = index*2)
        LODI,R1 >VARS
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VARS
        BCTA,GT DL_NC
        ADDI,R0 1
DL_NC:
        STRA,R0 TMPH
        LODA,R0 EXPH
        STRA,R0 *TMPH  ; store hi
        BSTA,UN INC_TMP
DL_NC2:
        LODA,R0 EXPL
        STRA,R0 *TMPH  ; store lo
        RETC,UN

; ─── DO_INPUT ─────────────────────────────────────────────────────────────────
DO_INPUT:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        BSTA,UN UPCASE  ; [+1]
        COMI,R0 A'A'
        BCTA,LT DIN_ERR
        COMI,R0 A'Z'+1
        BCTA,LT DIN_VAROK
DIN_ERR:
        LODI,R0 4
        BCTA,UN DO_ERROR
DIN_VAROK:
        STRA,R0 SC0                      ; save variable letter
        BSTA,UN INC_IP
DIN_PR:
        LODI,R0 A'?'
        BSTA,UN COUT
        LODI,R0 SP
        BSTA,UN COUT
        BSTA,UN RDLINE                   ; [+1]
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
        BSTA,UN PARSE_S16                ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DL_STORE
        LODI,R0 0
        BCTA,UN DO_ERROR

; ─── DO_IF ────────────────────────────────────────────────────────────────────
; IF expr relop expr THEN stmt
; Depth at entry: 2 (from REPL->STMT_EXEC) or 4 (from REPL->STMT_EXEC->DO_IF->STMT_EXEC->here)
; After THEN: calls STMT_EXEC at +1, which can call DO_xxx at +1, PARSE_EXPR at +1,
;             PARSE_FACTOR at +1 → max total 2+1+1+1+1+1 = depth 7 OK.
DO_IF:
        BSTA,UN PARSE_EXPR               ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DIF_LS
        LODI,R0 0
        BCTA,UN DO_ERROR
DIF_LS:
        LODA,R0 EXPH
        STRA,R0 TMPH  ; save left in TMPH:TMPL
        LODA,R0 EXPL
        STRA,R0 TMPL
        BSTA,UN PARSE_RELOP              ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DIF_RP
        LODI,R0 0
        BCTA,UN DO_ERROR
DIF_RP:
        BSTA,UN PARSE_EXPR               ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DIF_EVAL
        LODI,R0 0
        BCTA,UN DO_ERROR
DIF_EVAL:
        ; signed 16-bit compare: TMPH:TMPL (left) vs EXPH:EXPL (right)
        ; bias hi bytes by XOR $80 → unsigned compare
        LODA,R0 TMPH
        EORI,R0 $80
        STRA,R0 SC0
        LODA,R0 EXPH
        EORI,R0 $80
        SUBA,R0 SC0             ; biased right.hi - biased left.hi
        BCTA,LT DIF_LT
        BCTA,GT DIF_GT
        ; hi bytes equal: compare lo (unsigned)
        LODA,R0 EXPL
        SUBA,R0 TMPL
        BCTA,LT DIF_LT
        BCTA,GT DIF_GT
        LODI,R0 $00
        STRA,R0 SC1
        BCTA,UN DIF_TH  ; EQ
DIF_LT:
        LODI,R0 $FF
        STRA,R0 SC1
        BCTA,UN DIF_TH  ; LT
DIF_GT:
        LODI,R0 $01
        STRA,R0 SC1  ; GT

DIF_TH:
        ; consume THEN keyword: expect T then H then EATWORD
        BSTA,UN WSKIP                    ; [+1]
        BSTA,UN GETCI_UC                 ; [+1]  must be A'T'
        COMI,R0 A'T'
        BCTA,EQ DIF_TH2
        LODI,R0 0
        BCTA,UN DO_ERROR
DIF_TH2:
        BSTA,UN GETCI_UC                 ; [+1]  must be A'H'
        COMI,R0 A'H'
        BCTA,EQ DIF_EW
        LODI,R0 0
        BCTA,UN DO_ERROR
DIF_EW:
        BSTA,UN EATWORD                  ; [+1]

        ; test condition using SC1 ($FF=LT $00=EQ $01=GT) vs RELOP
        LODA,R0 RELOP
        COMI,R0 1
        BCTA,EQ DIF_CEQ  ; =
        COMI,R0 2
        BCTA,EQ DIF_CNE  ; <>
        COMI,R0 3
        BCTA,EQ DIF_CLT  ; <
        COMI,R0 4
        BCTA,EQ DIF_CGT  ; >
        COMI,R0 5
        BCTA,EQ DIF_CLE  ; <=
        COMI,R0 6
        BCTA,EQ DIF_CGE  ; >=
        LODI,R0 0
        BCTA,UN DO_ERROR

DIF_CEQ: LODA,R0 SC1
        COMI,R0 $00
        BCTA,EQ DIF_TRUE
        BCTA,UN DIF_FALSE
DIF_CNE: LODA,R0 SC1
        COMI,R0 $00
        BCFA,EQ DIF_TRUE
        BCTA,UN DIF_FALSE
DIF_CLT: LODA,R0 SC1
        COMI,R0 $FF
        BCTA,EQ DIF_TRUE
        BCTA,UN DIF_FALSE
DIF_CGT: LODA,R0 SC1
        COMI,R0 $01
        BCTA,EQ DIF_TRUE
        BCTA,UN DIF_FALSE
DIF_CLE: LODA,R0 SC1
        COMI,R0 $01
        BCFA,EQ DIF_TRUE
        BCTA,UN DIF_FALSE
DIF_CGE: LODA,R0 SC1
        COMI,R0 $FF
        BCFA,EQ DIF_TRUE
        BCTA,UN DIF_FALSE

DIF_TRUE:
        BSTA,UN STMT_EXEC                ; [+1]  execute THEN body
DIF_FALSE:
        RETC,UN

; ─── DO_GOTO ──────────────────────────────────────────────────────────────────
DO_GOTO:
        BSTA,UN PARSE_U16                ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DG_OK
        LODI,R0 0
        BCTA,UN DO_ERROR
DG_OK:
        LODA,R0 EXPH
        STRA,R0 GOTOH
        LODA,R0 EXPL
        STRA,R0 GOTOL
        LODI,R0 $01
        STRA,R0 GOTOFLG
        LODA,R0 RUNFLG
        COMI,R0 $01
        BCTA,EQ DG_RET
        LODI,R0 $01
        STRA,R0 RUNFLG  ; start run if in immediate mode
DG_RET:
        RETC,UN

; ─── DO_GOSUB / SRET / DO_RETURN ──────────────────────────────────────────────
; SW call stack stores the *next-line pointer* (TMPH:TMPL value in DO_RUN
; just after the executing line — i.e. the line after the GOSUB).
; DO_RUN saves this in SC0:SC1 before calling STMT_EXEC.
; DO_GOSUB reads SC0:SC1 and pushes to SWSTK[SWSP].
DO_LIST:
        LODI,R0 <PROG
        STRA,R0 TMPH
        LODI,R0 >PROG
        STRA,R0 TMPL
DLS_LP:
        LODA,R0 TMPH
        SUBA,R0 PEH
        BCTA,GT DLS_RET
        BCTA,LT DLS_BODY
        LODA,R0 TMPL
        SUBA,R0 PEL
        BCTA,LT DLS_BODY
        BCTA,UN DLS_RET
DLS_BODY:
        ; line number hi:lo
        LODA,R0 *TMPH
        STRA,R0 EXPH
        BSTA,UN INC_TMP
DLS_N1:
        LODA,R0 *TMPH
        STRA,R0 EXPL
        BSTA,UN INC_TMP
DLS_N2:
        BSTA,UN PRINT_S16                ; [+1]
        LODI,R0 SP
        BSTA,UN COUT
        ; body length into R3
        LODA,R3 *TMPH
        BSTA,UN INC_TMP
DLS_N3:
        COMI,R3 $00
        BCTA,EQ DLS_NL
DLS_BLPX:
        LODA,R0 *TMPH
        BSTA,UN COUT
        BSTA,UN INC_TMP
DLS_BNC:
        BRNR,R3 DLS_BLPX
DLS_NL:
        BSTA,UN CRLF
        BCTA,UN DLS_LP
DLS_RET:
        RETC,UN

; ─── DO_RUN ───────────────────────────────────────────────────────────────────
; Executes stored lines sequentially, honouring GOTOFLG for GOTO/GOSUB/RETURN.
; SC0:SC1 = next-line-pointer saved BEFORE STMT_EXEC so DO_GOSUB can read it.
DO_RUN:
        LODI,R0 $01
        STRA,R0 RUNFLG
        LODI,R0 $00
        STRA,R0 GOTOFLG
        LODI,R0 <PROG
        STRA,R0 TMPH
        LODI,R0 >PROG
        STRA,R0 TMPL
DR_LP:
        LODA,R0 RUNFLG
        COMI,R0 $00
        BCTA,EQ DR_RET
        ; end of program?
        LODA,R0 TMPH
        SUBA,R0 PEH
        BCTA,GT DR_STOP
        BCTA,LT DR_EXEC
        LODA,R0 TMPL
        SUBA,R0 PEL
        BCTA,LT DR_EXEC
        BCTA,UN DR_STOP
DR_EXEC:
        ; save line number for error reporting
        LODA,R0 *TMPH
        STRA,R0 CURH
        BSTA,UN INC_TMP
DR_N1:
        LODA,R0 *TMPH
        STRA,R0 CURL
        BSTA,UN INC_TMP
DR_N2:
        ; body length into R3
        LODA,R3 *TMPH
        BSTA,UN INC_TMP
DR_N3:
        ; copy body to IBUF
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
        COMI,R3 $00
        BCTA,EQ DR_CD
DR_CPY:
        LODA,R1 *TMPH
        STRA,R1 *IPH
        BSTA,UN INC_TMP
DR_TNC:
        BSTA,UN INC_IP
DR_INC:
        BRNR,R3 DR_CPY
DR_CD:
        LODI,R1 NUL
        STRA,R1 *IPH  ; NUL-terminate
        ; save next-line pointer in SC0:SC1 BEFORE executing
        ; (DO_GOSUB reads SC0:SC1 to know return address)
        LODA,R0 TMPH
        STRA,R0 SC0
        LODA,R0 TMPL
        STRA,R0 SC1
        ; execute line
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
        BSTA,UN STMT_EXEC                ; [+1]
        ; check GOTO/GOSUB/RETURN flag
        LODA,R0 GOTOFLG
        COMI,R0 $01
        BCTA,EQ DR_GOTO
        ; advance: restore next-line pointer from SC0:SC1
        LODA,R0 SC0
        STRA,R0 TMPH
        LODA,R0 SC1
        STRA,R0 TMPL
        BCTA,UN DR_LP
DR_GOTO:
        LODI,R0 $00
        STRA,R0 GOTOFLG
        LODA,R0 GOTOH
        STRA,R0 LNUMH
        LODA,R0 GOTOL
        STRA,R0 LNUML
        BSTA,UN FIND_LINE                ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DR_LP
        LODI,R0 1
        BSTA,UN DO_ERROR  ; [+1] undefined line — returns to REPL
        BCTA,UN DR_LP
DR_STOP:
        LODI,R0 $00
        STRA,R0 RUNFLG
DR_RET:
        RETC,UN

; ─── TRY_STORE_LINE ───────────────────────────────────────────────────────────
; If IP starts with a digit, parse and store/delete the numbered line.
; Returns ERRFLG=$01 if handled as a numbered line, $00 if immediate.
TRY_STORE_LINE:
        LODI,R0 $00
        STRA,R0 ERRFLG
        LODA,R0 *IPH
        COMI,R0 A'0'
        BCTA,LT TSL_RET
        COMI,R0 A'9'+1
        BCTA,LT TSL_NUM
TSL_RET:
        RETC,UN
TSL_NUM:
        BSTA,UN PARSE_U16                ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ TSL_GOT
        BCTA,UN TSL_RET
TSL_GOT:
        ; validate 1..32767
        LODA,R0 EXPH
        COMI,R0 $80
        BCTA,LT TSL_RNG
        LODI,R0 $01
        STRA,R0 ERRFLG
        RETC,UN  ; >=32768 silently ignore
TSL_RNG:
        LODA,R0 EXPH
        COMI,R0 $00
        BCTA,GT TSL_NZ
        LODA,R0 EXPL
        COMI,R0 $00
        BCTA,EQ TSL_RET2  ; line 0 invalid
TSL_NZ:
        LODA,R0 EXPH
        STRA,R0 LNUMH
        LODA,R0 EXPL
        STRA,R0 LNUML
        BSTA,UN WSKIP                    ; [+1]  skip space after line number
        LODA,R0 *IPH
        COMI,R0 NUL
        BCTA,EQ TSL_DEL
        BSTA,UN STORE_LINE               ; [+1]
        BCTA,UN TSL_DONE
TSL_DEL:
        BSTA,UN DELETE_LINE              ; [+1]
TSL_DONE:
        LODI,R0 $01
        STRA,R0 ERRFLG
        RETC,UN
TSL_RET2:
        RETC,UN

; ─── STORE_LINE ───────────────────────────────────────────────────────────────
; Insert line LNUMH:LNUML with body at IP into program store (sorted).
; Record format: [linehi][linelo][bodylen][body...]
; Strategy: delete existing line, measure body, check space, find insertion
;           point (EXPH:EXPL), shift existing records up, write new record.
STORE_LINE:
        BSTA,UN DELETE_LINE              ; [+1]  remove if exists

        ; measure body length: walk from IP to NUL, count in R3
        LODA,R0 IPH
        STRA,R0 TMPH
        LODA,R0 IPL
        STRA,R0 TMPL  ; TMPH:TMPL = body start (save for write)
        LODI,R3 0
SL_MEAS:
        LODA,R0 *TMPH
        COMI,R0 NUL
        BCTA,EQ SL_MEASD
        BSTA,UN INC_TMP
SL_MNC:
        BIRR,R3 SL_MEAS         ; R3++ then always branch (counts: 0→1→2...)
SL_MEASD:
        ; R3 = body length.  SC0 = body len.  SC1 = record size = 3 + R3.
        STRA,R3 SC0
        LODA,R0 SC0
        ADDI,R0 3
        STRA,R0 SC1

        ; check free space: PROGLIM - PE >= SC1
        LODI,R0 >PROGLIM
        SUBA,R0 PEL
        STRA,R0 TMPL
        LODI,R0 <PROGLIM
        SUBA,R0 PEH
        BCFA,LT SL_NBC
        SUBI,R0 1  ; borrow skip: BCFA,LT
SL_NBC:
        STRA,R0 TMPH            ; TMPH:TMPL = free bytes
        LODA,R0 TMPH
        COMI,R0 $00
        BCTA,GT SL_ROOM
        LODA,R0 TMPL
        SUBA,R0 SC1
        BCFA,LT SL_ROOM  ; free >= needed?
        LODI,R0 3
        BCTA,UN DO_ERROR  ; out of memory

SL_ROOM:
        ; find sorted insertion point → EXPH:EXPL
        BSTA,UN FIND_INS                 ; [+1]  sets TMPH:TMPL
        ; save insertion point in EXPH:EXPL (TMPH:TMPL will be used as walk pointer)
        LODA,R0 TMPH
        STRA,R0 EXPH
        LODA,R0 TMPL
        STRA,R0 EXPL

        ; shift bytes PE-1 down to EXPH:EXPL upward by SC1 positions (backwards copy)
        ; shift count = PE - EXPH:EXPL
        LODA,R0 PEL
        SUBA,R0 EXPL
        STRA,R0 TMPL
        LODA,R0 PEH
        SUBA,R0 EXPH
        BCFA,LT SL_SHCNB
        SUBI,R0 1
SL_SHCNB:
        STRA,R0 TMPH            ; TMPH:TMPL = shift count

        ; if shift count == 0 skip loop
        LODA,R0 TMPH
        COMI,R0 $00
        BCTA,GT SL_DOSHIFT
        LODA,R0 TMPL
        COMI,R0 $00
        BCTA,EQ SL_NOSHIFT
SL_DOSHIFT:
        ; src = PE-1 in NEGFLG:SC1 (use two scratch bytes; LNUMH:LNUML free now)
        LODA,R0 PEL
        SUBI,R0 1
        STRA,R0 LNUML
        LODA,R0 PEH
        BCFA,LT SL_SNBR
        SUBI,R0 1
SL_SNBR:
        STRA,R0 LNUMH           ; LNUMH:LNUML = src = PE-1
        ; dst = src + SC1  → SC0 now holds body len not record size! Use SC1.
        ; Actually SC1 = record size = shift amount.
        LODA,R0 LNUML
        ADDA,R0 SC1
        STRA,R0 GOTOL
        LODA,R0 LNUMH
        BCTA,GT SL_DSNCA
        ADDI,R0 1
SL_DSNCA:
        STRA,R0 GOTOH           ; GOTOH:GOTOL = dst = PE-1+SC1

        ; use R3 as count (shift count lo; assume <256 for any real program)
        LODA,R3 TMPL
SL_SHLOOP:
        COMI,R3 $00
        BCTA,EQ SL_NOSHIFT
        ; read from LNUMH:LNUML
        LODA,R1 *LNUMH
        ; write to GOTOH:GOTOL
        STRA,R1 *GOTOH
        ; decrement both pointers
        LODA,R0 LNUML
        SUBI,R0 1
        STRA,R0 LNUML
        BCFA,LT SL_SRNB
        LODA,R0 LNUMH
        SUBI,R0 1
        STRA,R0 LNUMH
SL_SRNB:
        LODA,R0 GOTOL
        SUBI,R0 1
        STRA,R0 GOTOL
        BCFA,LT SL_DRNB
        LODA,R0 GOTOH
        SUBI,R0 1
        STRA,R0 GOTOH
SL_DRNB:
        BRNR,R3 SL_SHLOOP

SL_NOSHIFT:
        ; write record at EXPH:EXPL (insertion point)
        ; restore IP body start to TMPH:TMPL (saved at top of STORE_LINE)
        ; TMPH:TMPL currently = shift count — need to reload body start from IP
        ; IP still points to body start (WSKIP was called before STORE_LINE)
        LODA,R0 IPH
        STRA,R0 TMPH
        LODA,R0 IPL
        STRA,R0 TMPL

        LODA,R0 LNUMH
        STRA,R0 *EXPH  ; write line hi
        BSTA,UN INC_EXP
SL_WN1:
        LODA,R0 LNUML
        STRA,R0 *EXPH  ; write line lo
        BSTA,UN INC_EXP
SL_WN2:
        LODA,R0 SC0
        STRA,R0 *EXPH  ; write body length
        BSTA,UN INC_EXP
SL_WN3:
        ; write body bytes (R3 = body len from SC0)
        LODA,R3 SC0
        COMI,R3 $00
        BCTA,EQ SL_WDONE
SL_WBODY:
        LODA,R1 *TMPH
        STRA,R1 *EXPH  ; copy body byte
        BSTA,UN INC_TMP
SL_WBNC:
        BSTA,UN INC_EXP
SL_WENC:
        BRNR,R3 SL_WBODY
SL_WDONE:
        ; update PE += SC1 (record size)
        LODA,R0 PEL
        ADDA,R0 SC1
        STRA,R0 PEL
        BCTA,GT SL_PENC
        LODA,R0 PEH
        ADDI,R0 1
        STRA,R0 PEH
SL_PENC:
        RETC,UN

; ─── DELETE_LINE ──────────────────────────────────────────────────────────────
DELETE_LINE:
        BSTA,UN FIND_LINE                ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DL2_FOUND
        RETC,UN
DL2_FOUND:
        ; record start in TMPH:TMPL.  Get size: 3 + bodylen at TMPH:TMPL+2.
        LODA,R0 TMPH
        STRA,R0 EXPH  ; save record start in EXPH:EXPL
        LODA,R0 TMPL
        STRA,R0 EXPL
        LODA,R0 TMPL
        ADDI,R0 2
        STRA,R0 TMPL
        BCTA,GT DL2_BLN
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
DL2_BLN:
        LODA,R0 *TMPH
        ADDI,R0 3
        STRA,R0 SC0  ; SC0 = record size
        ; advance TMPH:TMPL past record to get src for copy
        LODA,R3 SC0
        SUBI,R3 2  ; R3 = bodylen + 1  (skip len byte + body)
DL2_SKIP:
        COMI,R3 $00
        BCTA,EQ DL2_COPY
        BSTA,UN INC_TMP
DL2_SN:
        BRNR,R3 DL2_SKIP
DL2_COPY:
        ; copy TMPH:TMPL..PE-1 to EXPH:EXPL
DL2_LP:
        LODA,R0 TMPH
        SUBA,R0 PEH
        BCTA,GT DL2_DONE
        BCTA,LT DL2_MOV
        LODA,R0 TMPL
        SUBA,R0 PEL
        BCTA,LT DL2_MOV
        BCTA,UN DL2_DONE
DL2_MOV:
        LODA,R1 *TMPH
        STRA,R1 *EXPH
        BSTA,UN INC_TMP
DL2_TNC:
        BSTA,UN INC_EXP
DL2_ENC:
        BCTA,UN DL2_LP
DL2_DONE:
        ; PE -= SC0
        LODA,R0 PEL
        SUBA,R0 SC0
        STRA,R0 PEL
        BCFA,LT DL2_PNC
        LODA,R0 PEH
        SUBI,R0 1
        STRA,R0 PEH
DL2_PNC:
        RETC,UN

; ─── FIND_LINE ────────────────────────────────────────────────────────────────
; Search for line LNUMH:LNUML in program store (sorted ascending).
; Returns: TMPH:TMPL = record start if found; ERRFLG=$00 found / $01 not found.
FIND_LINE:
        LODI,R0 $01
        STRA,R0 ERRFLG
        LODI,R0 <PROG
        STRA,R0 TMPH
        LODI,R0 >PROG
        STRA,R0 TMPL
FL_LP:
        LODA,R0 TMPH
        SUBA,R0 PEH
        BCTA,GT FL_RET
        BCTA,LT FL_CHK
        LODA,R0 TMPL
        SUBA,R0 PEL
        BCTA,LT FL_CHK
        BCTA,UN FL_RET
FL_CHK:
        LODA,R0 *TMPH
        SUBA,R0 LNUMH
        BCTA,LT FL_ADV
        BCTA,GT FL_RET  ; stored.hi > target → not found
        ; hi bytes equal: check lo at TMPH:TMPL+1
        LODA,R0 TMPL
        ADDI,R0 1
        STRA,R0 EXPL
        LODA,R0 TMPH
        BCTA,GT FL_LH
        ADDI,R0 1
FL_LH:
        STRA,R0 EXPH                     ; EXPH:EXPL = lo byte address
        LODA,R0 *EXPH
        SUBA,R0 LNUML
        BCTA,LT FL_ADV
        BCTA,GT FL_RET
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN  ; exact match
FL_ADV:
        ; advance TMPH:TMPL by 3 + bodylen
        LODA,R0 TMPL
        ADDI,R0 2
        STRA,R0 TMPL
        BCTA,GT FL_AN
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
FL_AN:
        LODA,R3 *TMPH                    ; bodylen
        BSTA,UN INC_TMP
FL_AN2:
        COMI,R3 $00
        BCTA,EQ FL_LP
FL_AS:
        BSTA,UN INC_TMP
FL_ASN:
        BRNR,R3 FL_AS
        BCTA,UN FL_LP
FL_RET:
        RETC,UN

; ─── FIND_INS ─────────────────────────────────────────────────────────────────
; Find sorted insertion point for LNUMH:LNUML.
; Returns TMPH:TMPL = address where new record should be inserted.
FIND_INS:
        LODI,R0 <PROG
        STRA,R0 TMPH
        LODI,R0 >PROG
        STRA,R0 TMPL
FI_LP:
        LODA,R0 TMPH
        SUBA,R0 PEH
        BCTA,GT FI_RET
        BCTA,LT FI_CHK
        LODA,R0 TMPL
        SUBA,R0 PEL
        BCTA,LT FI_CHK
        BCTA,UN FI_RET
FI_CHK:
        LODA,R0 *TMPH
        SUBA,R0 LNUMH
        BCTA,LT FI_ADV
        BCTA,UN FI_RET  ; stored.hi >= new → insert here
        ; if EQ: check lo
        LODA,R0 TMPL
        ADDI,R0 1
        STRA,R0 EXPL
        LODA,R0 TMPH
        BCTA,GT FI_LH
        ADDI,R0 1
FI_LH:
        STRA,R0 EXPH
        LODA,R0 *EXPH
        SUBA,R0 LNUML
        BCTA,LT FI_ADV
        BCTA,UN FI_RET
FI_ADV:
        LODA,R0 TMPL
        ADDI,R0 2
        STRA,R0 TMPL
        BCTA,GT FI_AN
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
FI_AN:
        LODA,R3 *TMPH
        BSTA,UN INC_TMP
FI_AN2:
        COMI,R3 $00
        BCTA,EQ FI_LP
FI_AS:
        BSTA,UN INC_TMP
FI_ASN:
        BRNR,R3 FI_AS
        BCTA,UN FI_LP
FI_RET:
        RETC,UN

; ─── PARSE_EXPR ───────────────────────────────────────────────────────────────
; Shunting-yard iterative operator-precedence parser.
; Entry: IP at expression.  Exit: EXPH:EXPL = result, ERRFLG=$00.
; RAS budget: this routine is at depth N; calls PARSE_FACTOR at N+1.
; Max depth from caller: +2. PARSE_FACTOR may call PARSE_EXPR for functions
; at N+1+1 = N+2 total extra levels — tight at deepest path, see ARCH §12.
;
; OPSTK[0..STKIDX]: operator stack   '(' = sentinel (prec 0)
; VALSH/VALSL[0..STKIDX]: value stack
;
; Operator precedences: '('=0 (sentinel, never reduces), '+''-'=1, '*''/'=2
; Reduction: while top-op-prec >= cur-op-prec AND top-op != '(': apply top op
PARSE_EXPR:
        LODI,R0 $FF
        STRA,R0 STKIDX
        LODI,R0 $00
        STRA,R0 ERRFLG

PX_ATOM:
        ; skip spaces then parse one atom (number, variable, unary, paren)
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 A'('
        BCTA,EQ PX_LPAR
        COMI,R0 A'-'
        BCTA,EQ PX_UNEG
        COMI,R0 A'+'
        BCTA,EQ PX_UPOS
        BSTA,UN PARSE_FACTOR             ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ PX_PUSHV
        RETC,UN

PX_LPAR:
        ; push '(' sentinel onto OPSTK
        BSTA,UN INC_IP
PX_LPN:
        LODA,R0 STKIDX
        ADDI,R0 1
        STRA,R0 STKIDX
        LODI,R1 >OPSTK
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <OPSTK
        BCTA,GT PX_LPNCA
        ADDI,R0 1
PX_LPNCA:
        STRA,R0 TMPH
        LODI,R0 A'('
        STRA,R0 *TMPH
        BCTA,UN PX_ATOM

PX_UNEG:
        ; consume '-', parse factor, negate result
        BSTA,UN INC_IP
PX_UNN:
        BSTA,UN PARSE_FACTOR             ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ PX_NEG
        RETC,UN
PX_NEG:
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
        BCTA,UN PX_PUSHV

PX_UPOS:
        ; consume '+', parse factor — result unchanged
        BSTA,UN INC_IP
PX_UPN:
        BSTA,UN PARSE_FACTOR             ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ PX_PUSHV
        RETC,UN

PX_PUSHV:
        ; push EXPH:EXPL to value stack at STKIDX+1
        LODA,R0 STKIDX
        ADDI,R0 1
        STRA,R0 STKIDX
        LODI,R1 >VALSH
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSH
        BCTA,GT PX_VHN
        ADDI,R0 1
PX_VHN:
        STRA,R0 TMPH
        LODA,R0 EXPH
        STRA,R0 *TMPH
        LODA,R0 STKIDX
        LODI,R1 >VALSL
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSL
        BCTA,GT PX_VLN
        ADDI,R0 1
PX_VLN:
        STRA,R0 TMPH
        LODA,R0 EXPL
        STRA,R0 *TMPH

PX_PEEKOP:
        ; peek next char for operator
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH

        COMI,R0 A')'
        BCTA,EQ PX_RPAR  ; A')' → reduce until A'(' sentinel

        BSTA,UN GET_PREC                 ; [+1]  R0 = prec(cur op)  ; 0=not an op
        COMI,R0 $00
        BCTA,EQ PX_RALL  ; end of expression → reduce all
        STRA,R0 SC1                      ; SC1 = cur op prec

PX_REDLP:
        ; while STKIDX >= 1 and top-op-prec >= SC1: reduce
        LODA,R0 STKIDX
        COMI,R0 $00
        BCTA,EQ PX_PUSHOP  ; only 1 value
        ; get top op from OPSTK[STKIDX-1]
        SUBI,R0 1
        LODI,R1 >OPSTK
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <OPSTK
        BCTA,GT PX_TOPNC
        ADDI,R0 1
PX_TOPNC:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 SC0  ; SC0 = top op byte
        COMI,R0 A'('
        BCTA,EQ PX_PUSHOP  ; sentinel → stop reducing
        BSTA,UN GET_PREC_SC0             ; [+1]  R0 = prec(SC0)
        SUBA,R0 SC1                      ; top_prec - cur_prec
        BCTA,LT PX_PUSHOP                ; top_prec < cur_prec → push new op
        BSTA,UN APPLY_OP                 ; [+1]  reduce top pair
        BCTA,UN PX_REDLP

PX_PUSHOP:
        ; push cur op byte onto OPSTK[STKIDX] and consume from IP
        LODA,R0 *IPH
        STRA,R0 SC0
        BSTA,UN INC_IP
PX_PON:
        LODA,R0 STKIDX
        LODI,R1 >OPSTK
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <OPSTK
        BCTA,GT PX_OPN
        ADDI,R0 1
PX_OPN:
        STRA,R0 TMPH
        LODA,R0 SC0
        STRA,R0 *TMPH
        BCTA,UN PX_ATOM                  ; parse next value

PX_RPAR:
        ; consume ')'
        BSTA,UN INC_IP
PX_RPNCA:
        ; reduce until '(' sentinel
PX_RPLP:
        LODA,R0 STKIDX
        COMI,R0 $00
        BCTA,EQ PX_RPDONE  ; guard
        SUBI,R0 1
        LODI,R1 >OPSTK
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <OPSTK
        BCTA,GT PX_RPNCA2
        ADDI,R0 1
PX_RPNCA2:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 SC0
        COMI,R0 A'('
        BCTA,EQ PX_POPSENT
        BSTA,UN APPLY_OP                 ; [+1]
        BCTA,UN PX_RPLP
PX_POPSENT:
        ; pop '(' sentinel: STKIDX-- (removes the op slot; val result stays at top)
        LODA,R0 STKIDX
        SUBI,R0 1
        STRA,R0 STKIDX
PX_RPDONE:
        BCTA,UN PX_PEEKOP                ; continue scanning for more operators

PX_RALL:
        ; reduce all remaining ops
PX_RALL_LP:
        LODA,R0 STKIDX
        COMI,R0 $00
        BCTA,EQ PX_DONE
        SUBI,R0 1
        LODI,R1 >OPSTK
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <OPSTK
        BCTA,GT PX_RANC
        ADDI,R0 1
PX_RANC:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 SC0
        BSTA,UN APPLY_OP                 ; [+1]
        BCTA,UN PX_RALL_LP
PX_DONE:
        ; result is VALSH[0]:VALSL[0]
        LODA,R0 VALSH
        STRA,R0 EXPH
        LODA,R0 VALSL
        STRA,R0 EXPL
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN

; ─── GET_PREC ─────────────────────────────────────────────────────────────────
; R0 = precedence of *IPH  (0=not-an-op, 1=+/-, 2=*/)
GET_PREC:
        LODA,R0 *IPH
        ; fall through to GET_PREC_SC0

; R0 = precedence of char in R0
GET_PREC_SC0:
        COMI,R0 A'+'
        BCTA,EQ GP_LOW
        COMI,R0 A'-'
        BCTA,EQ GP_LOW
        COMI,R0 A'*'
        BCTA,EQ GP_HIGH
        COMI,R0 A'/'
        BCTA,EQ GP_HIGH
        LODI,R0 0
        RETC,UN
GP_LOW:  LODI,R0 1
        RETC,UN
GP_HIGH: LODI,R0 2
        RETC,UN

; ─── APPLY_OP ─────────────────────────────────────────────────────────────────
; Apply operator SC0 to top two stack values. Result → VALSH/VALSL[STKIDX-1].
; STKIDX decremented (one value consumed).
; Uses NEGFLG:SC1 as temp for left value during computation.
APPLY_OP:
        ; load right value: VALSH/VALSL[STKIDX]
        LODA,R0 STKIDX
        LODI,R1 >VALSH
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSH
        BCTA,GT AO_RHN
        ADDI,R0 1
AO_RHN:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 EXPH  ; right.hi
        LODA,R0 STKIDX
        LODI,R1 >VALSL
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSL
        BCTA,GT AO_RLN
        ADDI,R0 1
AO_RLN:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 EXPL  ; right.lo

        ; load left value: VALSH/VALSL[STKIDX-1]
        LODA,R0 STKIDX
        SUBI,R0 1
        LODI,R1 >VALSH
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSH
        BCTA,GT AO_LHN
        ADDI,R0 1
AO_LHN:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 NEGFLG  ; left.hi → NEGFLG temp
        LODA,R0 STKIDX
        SUBI,R0 1
        LODI,R1 >VALSL
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSL
        BCTA,GT AO_LLN
        ADDI,R0 1
AO_LLN:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 SC1  ; left.lo → SC1

        ; left = NEGFLG:SC1,  right = EXPH:EXPL
        ; dispatch on SC0
        LODA,R0 SC0
        COMI,R0 A'+'
        BCTA,EQ AO_ADD
        COMI,R0 A'-'
        BCTA,EQ AO_SUB
        COMI,R0 A'*'
        BCTA,EQ AO_MUL
        COMI,R0 A'/'
        BCTA,EQ AO_DIV
        RETC,UN

AO_ADD:
        ; EXPH:EXPL = NEGFLG:SC1 + EXPH:EXPL
        LODA,R0 SC1
        ADDA,R0 EXPL
        STRA,R0 EXPL
        BCTA,GT AO_ADDNC
        LODA,R0 NEGFLG
        ADDI,R0 1
        BCTA,UN AO_ADDHI
AO_ADDNC:
        LODA,R0 NEGFLG
AO_ADDHI:
        ADDA,R0 EXPH
        STRA,R0 EXPH
        BCTA,UN AO_STORE

AO_SUB:
        ; EXPH:EXPL = NEGFLG:SC1 - EXPH:EXPL
        LODA,R0 SC1
        SUBA,R0 EXPL
        STRA,R0 EXPL
        BCFA,LT AO_SUBNB                 ; no borrow → skip hi decrement
        LODA,R0 NEGFLG
        SUBI,R0 1
        BCTA,UN AO_SUBHI
AO_SUBNB:
        LODA,R0 NEGFLG
AO_SUBHI:
        SUBA,R0 EXPH
        STRA,R0 EXPH
        BCTA,UN AO_STORE

AO_MUL:
        ; MUL16: TMPH:TMPL * EXPH:EXPL → EXPH:EXPL  (NEGFLG:SC1 = left)
        LODA,R0 NEGFLG
        STRA,R0 TMPH
        LODA,R0 SC1
        STRA,R0 TMPL
        BSTA,UN MUL16                    ; [+1]
        BCTA,UN AO_STORE

AO_DIV:
        LODA,R0 NEGFLG
        STRA,R0 TMPH
        LODA,R0 SC1
        STRA,R0 TMPL
        BSTA,UN DIV16                    ; [+1]
        ; ERRFLG=$01 on /0 — DO_ERROR called inside DIV16
        BCTA,UN AO_STORE

AO_STORE:
        ; write EXPH:EXPL to VALSH/VALSL[STKIDX-1]; STKIDX--
        LODA,R0 STKIDX
        SUBI,R0 1
        LODI,R1 >VALSH
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSH
        BCTA,GT AO_SHN
        ADDI,R0 1
AO_SHN:
        STRA,R0 TMPH
        LODA,R0 EXPH
        STRA,R0 *TMPH
        LODA,R0 STKIDX
        SUBI,R0 1
        LODI,R1 >VALSL
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VALSL
        BCTA,GT AO_SLN
        ADDI,R0 1
AO_SLN:
        STRA,R0 TMPH
        LODA,R0 EXPL
        STRA,R0 *TMPH
        LODA,R0 STKIDX
        SUBI,R0 1
        STRA,R0 STKIDX
        RETC,UN

; ─── PARSE_FACTOR ─────────────────────────────────────────────────────────────
; Parse one atom: variable A-Z, signed decimal, PEEK(), CHR$(), USR().
; Called from PARSE_EXPR at depth N+1. May call PARSE_EXPR for function args
; (adds 1 more level). Unary - and + handled by PARSE_EXPR before calling here.
; CHR$ result: sets NEGFLG=$01 so DO_PRINT outputs EXPL as a character.
PARSE_FACTOR:
        LODI,R0 $00
        STRA,R0 NEGFLG  ; clear CHR$ flag
        LODA,R0 *IPH
        BSTA,UN UPCASE  ; [+1]

        ; check for variable A-Z
        COMI,R0 A'A'
        BCTA,LT PF_NUM
        COMI,R0 A'Z'+1
        BCTA,LT PF_LOADVAR

PF_NUM:
        ; decimal number (may have leading '-' but unary is in PARSE_EXPR)
        BSTA,UN PARSE_S16                ; [+1]
        RETC,UN

PF_LOADVAR:
        ; load variable value from VARS
        ; consume the variable character
        BSTA,UN INC_IP
PF_LVNCA:
        LODA,R0 SC0
        SUBI,R0 A'A'  ; 0-25
        STRA,R0 SC1
        ADDA,R0 SC1  ; *2
        LODI,R1 >VARS
        ADDZ,R1
        STRA,R0 TMPL
        LODI,R0 <VARS
        BCTA,GT PF_LVN
        ADDI,R0 1
PF_LVN:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 EXPH
        BSTA,UN INC_TMP
PF_LVN2:
        LODA,R0 *TMPH
        STRA,R0 EXPL
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN


PARSE_RELOP:
        LODI,R0 $01
        STRA,R0 ERRFLG
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 A'='
        BCTA,EQ PRO_EQ
        COMI,R0 A'<'
        BCTA,EQ PRO_LT
        COMI,R0 A'>'
        BCTA,EQ PRO_GT
        RETC,UN

PRO_EQ:
        BSTA,UN INC_IP
PRO_EQN:
        LODI,R0 1
        STRA,R0 RELOP
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN

PRO_LT:
        BSTA,UN INC_IP
PRO_LTN:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 A'='
        BCTA,EQ PRO_LE
        COMI,R0 A'>'
        BCTA,EQ PRO_NE
        LODI,R0 3
        STRA,R0 RELOP
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN
PRO_LE:
        BSTA,UN INC_IP
PRO_LEN:
        LODI,R0 5
        STRA,R0 RELOP
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN
PRO_NE:
        BSTA,UN INC_IP
PRO_NEN:
        LODI,R0 2
        STRA,R0 RELOP
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN
PRO_GT:
        BSTA,UN INC_IP
PRO_GTN:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 A'='
        BCTA,EQ PRO_GE
        LODI,R0 4
        STRA,R0 RELOP
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN
PRO_GE:
        BSTA,UN INC_IP
PRO_GEN:
        LODI,R0 6
        STRA,R0 RELOP
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN

; ─── PARSE_S16 ────────────────────────────────────────────────────────────────
; Parse optional leading '-' then decimal digits → EXPH:EXPL. ERRFLG=$00 if digits.
PARSE_S16:
        LODI,R0 $00
        STRA,R0 NEGFLG
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 A'-'
        BCTA,EQ PS16_NEG
        BCTA,UN PS16_UN
PS16_NEG:
        BSTA,UN INC_IP
PS16_NN:
        LODI,R0 $01
        STRA,R0 NEGFLG
PS16_UN:
        BSTA,UN PARSE_U16                ; [+1]
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ PS16_CHK
        RETC,UN
PS16_CHK:
        LODA,R0 NEGFLG
        COMI,R0 $00
        BCTA,EQ PS16_RET
        ; negate EXPH:EXPL
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
PS16_RET:
        RETC,UN

; ─── PARSE_U16 ────────────────────────────────────────────────────────────────
; Parse unsigned decimal digits → EXPH:EXPL. ERRFLG=$00 if ≥1 digit.
PARSE_U16:
        LODI,R0 $00
        STRA,R0 EXPH
        STRA,R0 EXPL
        LODI,R0 $01
        STRA,R0 ERRFLG  ; assume failure
PU16_LP:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 A'0'
        BCTA,LT PU16_DONE
        COMI,R0 A'9'+1
        BCTA,LT PU16_DIG
        BCTA,UN PU16_DONE
PU16_DIG:
        SUBI,R0 A'0'
        STRA,R0 SC0  ; digit value 0-9
        BSTA,UN INC_IP
PU16_DNC:
        ; EXP = EXP*10 using R3 loop (BRNR counts down to 0)
        LODA,R0 EXPH
        STRA,R0 TMPH
        LODA,R0 EXPL
        STRA,R0 TMPL  ; TMPH:TMPL = old EXP
        LODI,R0 $00
        STRA,R0 EXPH
        STRA,R0 EXPL
        LODI,R3 10
PU16_M10:
        LODA,R0 EXPL
        ADDA,R0 TMPL
        STRA,R0 EXPL
        BCTA,GT PU16_MNC
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
PU16_MNC:
        LODA,R0 EXPH
        ADDA,R0 TMPH
        STRA,R0 EXPH
        BRNR,R3 PU16_M10
        ; EXP += digit
        LODA,R0 EXPL
        ADDA,R0 SC0
        STRA,R0 EXPL
        BCTA,GT PU16_DIG_NC
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
PU16_DIG_NC:
        LODI,R0 $00
        STRA,R0 ERRFLG  ; success: at least one digit
        BCTA,UN PU16_LP
PU16_DONE:
        RETC,UN

; ─── MUL16 ────────────────────────────────────────────────────────────────────
; Signed TMPH:TMPL × EXPH:EXPL → EXPH:EXPL  (16-bit two's complement wrap)
MUL16:
        LODI,R0 $00
        STRA,R0 NEGFLG
        ; abs(left) TMPH:TMPL
        LODA,R0 TMPH
        COMI,R0 $80
        BCTA,LT MU_LA
        LODA,R0 TMPH
        EORI,R0 $FF
        STRA,R0 TMPH
        LODA,R0 TMPL
        EORI,R0 $FF
        STRA,R0 TMPL
        LODA,R0 TMPL
        ADDI,R0 1
        STRA,R0 TMPL
        BCTA,GT MU_LA
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
        LODI,R0 $01
        STRA,R0 NEGFLG
MU_LA:
        ; abs(right) EXPH:EXPL
        LODA,R0 EXPH
        COMI,R0 $80
        BCTA,LT MU_RA
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        LODA,R0 EXPL
        ADDI,R0 1
        STRA,R0 EXPL
        BCTA,GT MU_RA
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
        LODA,R0 NEGFLG
        EORI,R0 $01
        STRA,R0 NEGFLG
MU_RA:
        ; save right in SC0:SC1; result EXP=0
        LODA,R0 EXPH
        STRA,R0 SC0
        LODA,R0 EXPL
        STRA,R0 SC1
        LODI,R0 $00
        STRA,R0 EXPH
        STRA,R0 EXPL
MU_LP:
        LODA,R0 TMPH
        COMI,R0 $00
        BCTA,GT MU_ADD
        LODA,R0 TMPL
        COMI,R0 $00
        BCTA,EQ MU_DONE
MU_ADD:
        LODA,R0 EXPL
        ADDA,R0 SC1
        STRA,R0 EXPL
        BCTA,GT MU_MNC
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
        BCFA,LT MU_TNB
        LODA,R0 TMPH
        SUBI,R0 1
        STRA,R0 TMPH
MU_TNB:
        BCTA,UN MU_LP
MU_DONE:
        LODA,R0 NEGFLG
        COMI,R0 $00
        BCTA,EQ MU_RET
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
MU_RET:
        RETC,UN

; ─── DIV16 ────────────────────────────────────────────────────────────────────
; Signed TMPH:TMPL ÷ EXPH:EXPL → EXPH:EXPL  (truncate toward zero)
; ERRFLG=$01 and DO_ERROR called on divide-by-zero.
DIV16:
        LODI,R0 $00
        STRA,R0 ERRFLG
        LODA,R0 EXPH
        COMI,R0 $00
        BCTA,GT DV_NZ
        LODA,R0 EXPL
        COMI,R0 $00
        BCTA,EQ DV_ZERO
DV_NZ:
        LODI,R0 $00
        STRA,R0 NEGFLG
        ; abs(dividend) TMPH:TMPL
        LODA,R0 TMPH
        COMI,R0 $80
        BCTA,LT DV_DA
        LODA,R0 TMPH
        EORI,R0 $FF
        STRA,R0 TMPH
        LODA,R0 TMPL
        EORI,R0 $FF
        STRA,R0 TMPL
        LODA,R0 TMPL
        ADDI,R0 1
        STRA,R0 TMPL
        BCTA,GT DV_DA
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
        LODI,R0 $01
        STRA,R0 NEGFLG
DV_DA:
        ; abs(divisor) EXPH:EXPL
        LODA,R0 EXPH
        COMI,R0 $80
        BCTA,LT DV_VA
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        LODA,R0 EXPL
        ADDI,R0 1
        STRA,R0 EXPL
        BCTA,GT DV_VA
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
        LODA,R0 NEGFLG
        EORI,R0 $01
        STRA,R0 NEGFLG
DV_VA:
        LODA,R0 EXPH
        STRA,R0 SC0  ; divisor hi
        LODA,R0 EXPL
        STRA,R0 SC1  ; divisor lo
        LODI,R0 $00
        STRA,R0 EXPH
        STRA,R0 EXPL  ; quotient = 0
DV_LP:
        ; while TMPH:TMPL >= SC0:SC1
        LODA,R0 TMPH
        SUBA,R0 SC0
        BCTA,LT DV_DONE
        BCTA,GT DV_SUB
        LODA,R0 TMPL
        SUBA,R0 SC1
        BCTA,LT DV_DONE
DV_SUB:
        LODA,R0 TMPL
        SUBA,R0 SC1
        STRA,R0 TMPL
        BCFA,LT DV_SNB
        LODA,R0 TMPH
        SUBI,R0 1
        STRA,R0 TMPH
DV_SNB:
        LODA,R0 TMPH
        SUBA,R0 SC0
        STRA,R0 TMPH
        ; quotient++
        BSTA,UN INC_EXP
        BCTA,UN DV_LP
DV_DONE:
        LODA,R0 NEGFLG
        COMI,R0 $00
        BCTA,EQ DV_RET
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
DV_RET:
        RETC,UN
DV_ZERO:
        LODI,R0 2
        BCTA,UN DO_ERROR  ; divide by zero error

; ─── PRINT_S16 ────────────────────────────────────────────────────────────────
; Print signed 16-bit value EXPH:EXPL as decimal.
; Uses DIVTAB for digit extraction. NEGFLG = leading-zero suppression flag.
PRINT_S16:
        LODA,R0 EXPH
        COMI,R0 $80
        BCTA,LT PS16P_POS
        LODI,R0 A'-'
        BSTA,UN COUT
        ; negate
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
PS16P_POS:
        LODA,R0 EXPH
        COMI,R0 $00
        BCTA,GT PS16P_NZ
        LODA,R0 EXPL
        COMI,R0 $00
        BCTA,GT PS16P_NZ
        LODI,R0 A'0'
        BSTA,UN COUT
        RETC,UN

PS16P_NZ:
        LODI,R0 <DIVTAB
        STRA,R0 TMPH
        LODI,R0 >DIVTAB
        STRA,R0 TMPL
        LODI,R0 $00
        STRA,R0 NEGFLG  ; leading-zero flag
PS16P_DIVLP:
        ; load next divisor pair from DIVTAB
        LODA,R0 *TMPH
        STRA,R0 SC0  ; div hi
        BSTA,UN INC_TMP
PS16P_D1:
        LODA,R0 *TMPH
        STRA,R0 SC1  ; div lo
        BSTA,UN INC_TMP
PS16P_D2:
        ; sentinel 0,0 → print final ones digit
        LODA,R0 SC0
        COMI,R0 $00
        BCTA,GT PS16P_CNT
        LODA,R0 SC1
        COMI,R0 $00
        BCTA,EQ PS16P_LAST
PS16P_CNT:
        ; count subtractions using R3 (BIRR: increment and branch while nonzero)
        LODI,R3 $00
PS16P_SLP:
        LODA,R0 EXPH
        SUBA,R0 SC0
        BCTA,LT PS16P_EMIT
        BCTA,GT PS16P_DO
        LODA,R0 EXPL
        SUBA,R0 SC1
        BCTA,LT PS16P_EMIT
PS16P_DO:
        LODA,R0 EXPL
        SUBA,R0 SC1
        STRA,R0 EXPL
        BCFA,LT PS16P_SNB
        LODA,R0 EXPH
        SUBI,R0 1
        STRA,R0 EXPH
PS16P_SNB:
        LODA,R0 EXPH
        SUBA,R0 SC0
        STRA,R0 EXPH
        BIRR,R3 PS16P_SLP       ; R3++  ; loop
PS16P_EMIT:
        ; R3 = digit value
        LODA,R0 NEGFLG
        COMI,R0 $00
        BCTA,GT PS16P_FPRINT  ; already printing
        ; leading zero check: LODZ,R3 → R0 = R3
        LODZ,R3                 ; R0 = R3 (digit count, LODZ Rn loads Rn into R0)
        COMI,R0 $00
        BCTA,EQ PS16P_DIVLP  ; skip leading zero
PS16P_FPRINT:
        LODZ,R3                 ; R0 = R3 (digit value 0-9)
        ADDI,R0 A'0'            ; R0 = ASCII digit
        BSTA,UN COUT
        LODI,R0 $01
        STRA,R0 NEGFLG
        BCTA,UN PS16P_DIVLP
PS16P_LAST:
        LODA,R0 EXPL
        ADDI,R0 A'0'
        BSTA,UN COUT
        RETC,UN

; ─── GETKEY ───────────────────────────────────────────────────────────────────
; Blocking keyboard read via Pipbug CHIN.
; CHIN is blocking — waits for a keypress before returning.
; Returns char in R0.  Clobbers R0 only.
GETKEY:
        BSTA,UN CHIN            ; R0 = char (CHIN blocks until key pressed)
        RETC,UN

; ─── RDLINE ───────────────────────────────────────────────────────────────────
; Read a line from input into IBUF, echo with backspace support. NUL-terminates.
; Uses GETKEY (via CHIN) for blocking input. Char received in R0 at each step;
; saved to R1 for storage/echo so R0 is free for pointer arithmetic.
RDLINE:
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
RL_LP:
        BSTA,UN GETKEY          ; [+1] blocking — R0 = char
        STRZ,R1                 ; R1 = R0 (char  ; R0 now free for pointer arithmetic)
        COMI,R1 CR
        BCTA,EQ RL_EOL
        COMI,R1 LF
        BCTA,EQ RL_EOL
        COMI,R1 BS
        BCTA,EQ RL_BS
        ; buffer full?  IP >= IBUF+63
        LODA,R0 IPH
        SUBA,R0 <IBUF
        BCTA,GT RL_FULL
        BCTA,LT RL_STORE
        LODA,R0 IPL
        SUBA,R0 >IBUF+63
        BCTA,LT RL_STORE
RL_FULL:
        BCTA,UN RL_LP
RL_STORE:
        STRA,R1 *IPH            ; store char to buffer
        LODZ,R1
        BSTA,UN COUT            ; echo char
        BSTA,UN INC_IP
        BCTA,UN RL_LP
RL_BS:
        ; at IBUF start? — no backspace if buffer empty
        LODA,R0 IPH
        SUBA,R0 <IBUF
        BCTA,GT RL_BSDO
        BCTA,LT RL_LP
        LODA,R0 IPL
        SUBA,R0 >IBUF
        BCTA,EQ RL_LP
RL_BSDO:
        LODA,R0 IPL
        SUBI,R0 1
        STRA,R0 IPL
        BCFA,LT RL_BSNB
        LODA,R0 IPH
        SUBI,R0 1
        STRA,R0 IPH
RL_BSNB:
        LODI,R0 BS
        BSTA,UN COUT
        LODI,R0 SP
        BSTA,UN COUT
        LODI,R0 BS
        BSTA,UN COUT
        BCTA,UN RL_LP
RL_EOL:
        LODI,R1 NUL
        STRA,R1 *IPH            ; NUL-terminate buffer
        BSTA,UN CRLF
        RETC,UN

; ─── PRTSTR / PRTSTR_IP ───────────────────────────────────────────────────────
; Print NUL-terminated string at IPH:IPL.
PRTSTR:
        LODA,R1 *IPH
        COMI,R1 NUL
        BCTA,EQ PRTSTR_RET
        LODZ,R1
        BSTA,UN COUT
        BSTA,UN INC_IP
        BCTA,UN PRTSTR
PRTSTR_RET:
        RETC,UN
; PRTSTR_IP is the same routine, just an alias for clarity at the call site.
PRTSTR_IP:
        BCTA,UN PRTSTR

; ─── WSKIP ────────────────────────────────────────────────────────────────────
WSKIP:
        LODA,R0 *IPH
        COMI,R0 SP
        BCTA,EQ WS_ADV
        RETC,UN
WS_ADV:
        BSTA,UN INC_IP
        BCTA,UN WSKIP

; ─── GETCI_UC ─────────────────────────────────────────────────────────────────
; Read *IPH uppercase into R0, advance IP.
GETCI_UC:
        LODA,R0 *IPH
        BSTA,UN UPCASE                   ; [+1]
        BSTA,UN INC_IP
GETCI_UC_RET:
        RETC,UN

; ─── UPCASE ───────────────────────────────────────────────────────────────────
UPCASE:
        COMI,R0 A'a'
        BCTA,LT UC_RET
        COMI,R0 A'z'+1
        BCTA,LT UC_DO
        BCTA,UN UC_RET
UC_DO:  SUBI,R0 32
UC_RET: RETC,UN

; ─── EATWORD ──────────────────────────────────────────────────────────────────
; Skip [A-Za-z$] at IP.
EATWORD:
        LODA,R0 *IPH
        BSTA,UN UPCASE  ; [+1]
        COMI,R0 A'A'
        BCTA,LT EW_DS
        COMI,R0 A'Z'+1
        BCTA,LT EW_ADV
EW_DS:  COMI,R0 A'$'
        BCTA,EQ EW_ADV
        RETC,UN
EW_ADV:
        BSTA,UN INC_IP
        BCTA,UN EATWORD

; ─── SHARED 16-BIT POINTER INCREMENT/DECREMENT SUBROUTINES ───────────────────
; INC_IP  : IPH:IPL  += 1   (clobbers R0)
; INC_TMP : TMPH:TMPL += 1  (clobbers R0)
; INC_EXP : EXPH:EXPL += 1  (clobbers R0)
; DEC_TMP : TMPH:TMPL -= 1  (clobbers R0)
; Rule: NO BSTA inside these — must not consume extra RAS depth.
; Carry idiom: ADDI sets no-carry->GT, carry->EQ/LT.
;   BCTA,GT skip  =  skip hi-byte increment if no carry from lo-byte add.
; Borrow idiom: SUBI sets no-borrow->GT/EQ, borrow->LT.
;   BCFA,LT skip  =  skip hi-byte decrement if no borrow (C=1).

INC_IP:
        LODA,R0 IPL
        ADDI,R0 1
        STRA,R0 IPL
        BCTA,GT INC_IP_RET      ; no carry — hi byte unchanged
        LODA,R0 IPH
        ADDI,R0 1
        STRA,R0 IPH
INC_IP_RET:
        RETC,UN

INC_TMP:
        LODA,R0 TMPL
        ADDI,R0 1
        STRA,R0 TMPL
        BCTA,GT INC_TMP_RET     ; no carry
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
INC_TMP_RET:
        RETC,UN

INC_EXP:
        LODA,R0 EXPL
        ADDI,R0 1
        STRA,R0 EXPL
        BCTA,GT INC_EXP_RET     ; no carry
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
INC_EXP_RET:
        RETC,UN

DEC_TMP:
        LODA,R0 TMPL
        SUBI,R0 1
        STRA,R0 TMPL
        BCFA,LT DEC_TMP_RET     ; no borrow (C=1) — hi byte unchanged
        LODA,R0 TMPH
        SUBI,R0 1
        STRA,R0 TMPH
DEC_TMP_RET:
        RETC,UN


; ─── DO_ERROR ─────────────────────────────────────────────────────────────────
; Entry: R0 = error code (0-5).
; Saves RUNFLG, clears all run state, prints "?n [IN line]", jumps to REPL.
; This is a tail-jump (BCTA,UN DO_ERROR from callers), so it kills the full RAS.
DO_ERROR:
        STRA,R0 SC0                      ; save error code
        LODA,R0 RUNFLG
        STRA,R0 SC1  ; save run state
        LODI,R0 $00
        STRA,R0 RUNFLG  ; clear run
        LODI,R0 $FF
        STRA,R0 SWSP  ; clear GOSUB stack
        LODI,R0 A'?'
        BSTA,UN COUT
        LODA,R0 SC0
        ADDI,R0 A'0'
        BSTA,UN COUT
        LODA,R0 SC1
        COMI,R0 $01
        BCTA,EQ DE_IN
        BCTA,UN DE_NL
DE_IN:
        LODI,R0 SP
        BSTA,UN COUT
        LODI,R0 A'I'
        BSTA,UN COUT
        LODI,R0 A'N'
        BSTA,UN COUT
        LODI,R0 SP
        BSTA,UN COUT
        LODA,R0 CURH
        STRA,R0 EXPH
        LODA,R0 CURL
        STRA,R0 EXPL
        BSTA,UN PRINT_S16                ; [+1]
DE_NL:
        BSTA,UN CRLF
        BCTA,UN REPL                     ; jump to REPL — clears full hardware RAS

        END
