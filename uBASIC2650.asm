; ubasic_v0.1.asm  -  Tiny BASIC work-in-progress for Signetics 2650
; -----------------------------------------------------------------------
; TARGET:   Signetics 2650 @ 1 MHz
;           ROM $0000-$0FFF (4KB), RAM $1000-$17FF (2KB)
; SIM I/O:  WRTD,R1 = putchar,  REDE,R1 = getchar
;
; v0.4: continues port by adding more Tiny BASIC statements:
;       - LET / LE  variable assignment (A-Z, 8-bit value in low byte)
;       - PRINT / PR now prints quoted strings OR numeric value expressions
;       - expressions support value chains with + / - (8-bit wrap)
;       - INPUT / IN reads decimal into variable
;       - IF <expr><relop><expr> THEN <statement>
;       - signed 16-bit variable/value path for LET/PRINT/INPUT/IF
;       - REM / RE (no-op), END / EN (halt), ?0 syntax errors
;
; RAM MAP ($1000-$17FF):
;   $1000-$1001  SPTR    string walk pointer (2 bytes)
;   $1002-$1003  LPTR    line buffer write pointer (2 bytes)
;   $1004        LCNT    line buffer char count (1 byte)
;   $1005-$104F  IBUF    input line buffer (75 bytes + NUL)
;   $1050-$1051  IPTR    parse pointer into IBUF (2 bytes)
;   $1052-$1085  VARS    A-Z variables, 2 bytes each (hi,lo)
;   $1086-       parser/print scratch
; -----------------------------------------------------------------------

CR      EQU     $0D
LF      EQU     $0A
BS      EQU     $08
SP      EQU     $20
NUL     EQU     $00
DQ      EQU     '"'
IBUFSZ  EQU     75
PROGBASE EQU    $1100
PROGLIM  EQU    $1800

SPTR    EQU     $1000
LPTR    EQU     $1002
LCNT    EQU     $1004
IBUF    EQU     $1005
IPTR    EQU     $1050
VARS    EQU     $1052

TMPCHR  EQU     $1086
TMPNUM  EQU     $1087
FOUND   EQU     $1088
ERRFLG  EQU     $1089
NUMVAL  EQU     $108A
NUMTMP  EQU     $108B
DIGIT   EQU     $108C
HUND    EQU     $108D
TENS    EQU     $108E
ONES    EQU     $108F
LEFTV   EQU     $1090
RIGHTV  EQU     $1091
RELOP   EQU     $1092
VALHI   EQU     $1093
VALLO   EQU     $1094
ACCHI   EQU     $1095
ACCLO   EQU     $1096
TMPHI2  EQU     $1097
TMPLO2  EQU     $1098
NEGFLG  EQU     $1099
DIGCNT  EQU     $109A
RGTHI   EQU     $109B
RGTLO   EQU     $109C
PROGHI  EQU     $109D
PROGLO  EQU     $109E
P1HI    EQU     $109F
P1LO    EQU     $10A0
P2HI    EQU     $10A1
P2LO    EQU     $10A2
RUNFLG  EQU     $10A3
GOTOFLG EQU     $10A4
GOTOLHI EQU     $10A5
GOTOLLO EQU     $10A6
TMPLEN  EQU     $10A7
CURHI   EQU     $10A8
CURLO   EQU     $10A9
NEXTHI  EQU     $10AA
NEXTLO  EQU     $10AB
LNUMHI  EQU     $10AC
LNUMLO  EQU     $10AD

        ORG     $0000

; ════════════════════════════════════════════════════════════════
; RESET
; ════════════════════════════════════════════════════════════════
RESET:
        BSTA,UN INIT_PROG
        LODI,R0 >BANNER
        STRA,R0 SPTR
        LODI,R0 <BANNER
        STRA,R0 SPTR+1
        BSTA,UN PRTSTR

; ════════════════════════════════════════════════════════════════
; REPL  —  prompt, read line, execute, repeat
; ════════════════════════════════════════════════════════════════
REPL:
        LODI,R1 '>'
        BSTA,UN PUTCH
        LODI,R1 SP
        BSTA,UN PUTCH

        BSTA,UN RDLINE              ; fills IBUF, NUL-terminated

        ; IPTR -> IBUF
        LODI,R0 >IBUF
        STRA,R0 IPTR
        LODI,R0 <IBUF
        STRA,R0 IPTR+1

        BSTA,UN TRY_STORE_LINE
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,GT REPL

        BSTA,UN STMT_LINE
        BCTA,UN REPL

; ════════════════════════════════════════════════════════════════
; STMT_LINE  — immediate-mode statement decode
; Supports PR/PRINT, LE/LET, IN/INPUT, IF, RE/REM, EN/END
; ════════════════════════════════════════════════════════════════
STMT_LINE:
        BSTA,UN WPEEK_UC            ; R1 = first non-space, uppercased
        COMI,R1 NUL
        BCTA,EQ ST_RET              ; blank line

        COMI,R1 'P'
        BCTA,EQ ST_PRINT
        COMI,R1 'L'
        BCTA,EQ ST_LET
        COMI,R1 'R'
        BCTA,EQ ST_REM
        COMI,R1 'E'
        BCTA,EQ ST_END
        COMI,R1 'I'
        BCTA,EQ ST_I
        COMI,R1 'N'
        BCTA,EQ ST_NEW
        COMI,R1 'G'
        BCTA,EQ ST_GOTO

        BCTA,UN SYNERR

ST_PRINT:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'P'
        BCTA,EQ ST_PRINT_2
        BCTA,UN SYNERR
ST_PRINT_2:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'R'
        BCTA,EQ ST_PRINT_OK
        BCTA,UN SYNERR
ST_PRINT_OK:
        BSTA,UN EATWORD             ; allow full PRINT
        BSTA,UN DO_PRINT
        BCTA,UN ST_RET

ST_LET:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'E'
        BCTA,EQ ST_LET_2
        COMI,R1 'I'
        BCTA,EQ ST_LIST_2
        BCTA,UN SYNERR
ST_LET_2:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'E'
        BCTA,EQ ST_LET_OK
        BCTA,UN SYNERR
ST_LET_OK:
        BSTA,UN EATWORD             ; allow full LET
        BSTA,UN DO_LET
        BCTA,UN ST_RET

ST_LIST_2:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'S'
        BCTA,EQ ST_LIST_3
        BCTA,UN SYNERR
ST_LIST_3:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'T'
        BCTA,EQ ST_LIST_OK
        BCTA,UN SYNERR
ST_LIST_OK:
        BSTA,UN EATWORD
        BSTA,UN CHECK_EOL
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ ST_LIST_DO
        BCTA,UN SYNERR
ST_LIST_DO:
        BSTA,UN DO_LIST
        BCTA,UN ST_RET

ST_REM:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'E'
        BCTA,EQ ST_REM_2
        COMI,R1 'U'
        BCTA,EQ ST_RUN_2
        BCTA,UN SYNERR
ST_REM_2:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'E'
        BCTA,EQ ST_REM_OK
        BCTA,UN SYNERR
ST_REM_OK:
        BSTA,UN EATWORD
        BCTA,UN ST_RET

ST_RUN_2:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'N'
        BCTA,EQ ST_RUN_OK
        BCTA,UN SYNERR
ST_RUN_OK:
        BSTA,UN EATWORD
        BSTA,UN CHECK_EOL
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ ST_RUN_DO
        BCTA,UN SYNERR
ST_RUN_DO:
        BSTA,UN DO_RUN
        BCTA,UN ST_RET

ST_END:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'E'
        BCTA,EQ ST_END_2
        BCTA,UN SYNERR
ST_END_2:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'N'
        BCTA,EQ ST_END_OK
        BCTA,UN SYNERR
ST_END_OK:
        BSTA,UN EATWORD
        LODA,R0 RUNFLG
        COMI,R0 $00
        BCTA,EQ ST_END_HALT
        LODI,R0 $00
        STRA,R0 RUNFLG
        RETC,UN
ST_END_HALT:
        HALT

ST_I:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'I'
        BCTA,EQ ST_I_2
        BCTA,UN SYNERR
ST_I_2:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'N'
        BCTA,EQ ST_INPUT_OK
        COMI,R1 'F'
        BCTA,EQ ST_IF_OK
        BCTA,UN SYNERR

ST_NEW:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'E'
        BCTA,EQ ST_NEW_2
        BCTA,UN SYNERR
ST_NEW_2:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'W'
        BCTA,EQ ST_NEW_OK
        BCTA,UN SYNERR
ST_NEW_OK:
        BSTA,UN EATWORD
        BSTA,UN CHECK_EOL
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ ST_NEW_DO
        BCTA,UN SYNERR
ST_NEW_DO:
        BSTA,UN INIT_PROG
        BCTA,UN ST_RET

ST_GOTO:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'O'
        BCTA,EQ ST_GOTO_2
        BCTA,UN SYNERR
ST_GOTO_2:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'T'
        BCTA,EQ ST_GOTO_3
        BCTA,UN SYNERR
ST_GOTO_3:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'O'
        BCTA,EQ ST_GOTO_OK
        BCTA,UN SYNERR
ST_GOTO_OK:
        BSTA,UN EATWORD
        BSTA,UN DO_GOTO
        BCTA,UN ST_RET

ST_INPUT_OK:
        BSTA,UN EATWORD             ; allow full INPUT
        BSTA,UN DO_INPUT
        BCTA,UN ST_RET

ST_IF_OK:
        BSTA,UN DO_IF
        BCTA,UN ST_RET

ST_RET:
        RETC,UN

; ════════════════════════════════════════════════════════════════
; DO_LET  — LET A=expr
; expr: decimal literal or variable
; ════════════════════════════════════════════════════════════════
DO_LET:
        ; read variable name
        BSTA,UN WPEEK_UC
        COMI,R1 'A'
        BCTA,LT SYNERR
        COMI,R1 'Z'+1
        BCTA,LT DOL_VAROK
        BCTA,UN SYNERR
DOL_VAROK:
        BSTA,UN GETCI               ; consume variable char
        BSTA,UN UC
        STRA,R1 TMPCHR

        ; expect '='
        BSTA,UN WPEEK
        COMI,R1 '='
        BCTA,EQ DOL_EQ
        BCTA,UN SYNERR
DOL_EQ:
        BSTA,UN GETCI               ; consume '='

        ; parse expression into VALHI:VALLO
        BSTA,UN PARSE_EXPR
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DOL_STORE
        BCTA,UN SYNERR

DOL_STORE:
        ; locate variable slot
        LODA,R1 TMPCHR
        BSTA,UN GET_VARPTR          ; SPTR -> var hi byte

        ; hi byte
        LODA,R0 VALHI
        STRA,R0 *SPTR
        ; low byte
        BSTA,UN INC_SPTR
        LODA,R0 VALLO
        STRA,R0 *SPTR
        RETC,UN

; ════════════════════════════════════════════════════════════════
; DO_PRINT  — PRINT "string" or PRINT expr
; expr: decimal literal or variable
; ════════════════════════════════════════════════════════════════
DO_PRINT:
        BSTA,UN WPEEK
        COMI,R1 DQ
        BCTA,EQ PR_QUOTED

        ; try expression path first
        BSTA,UN PARSE_EXPR
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ PR_NUM

        ; fallback: print tail text as-is
        LODA,R0 IPTR
        STRA,R0 SPTR
        LODA,R0 IPTR+1
        STRA,R0 SPTR+1
        BSTA,UN PRTSTR
        BSTA,UN PRNL
        RETC,UN

PR_NUM:
        BSTA,UN PRINT_S16
        BSTA,UN PRNL
        RETC,UN

PR_QUOTED:
        BSTA,UN GETCI               ; consume opening quote
PRQ_LP:
        BSTA,UN GETCI
        COMI,R1 NUL
        BCTA,EQ PRQ_DONE
        COMI,R1 DQ
        BCTA,EQ PRQ_DONE
        BSTA,UN PUTCH
        BCTA,UN PRQ_LP
PRQ_DONE:
        BSTA,UN PRNL
        RETC,UN

; ════════════════════════════════════════════════════════════════
; DO_INPUT  — INPUT A
; Reads a decimal number and stores it to variable
; ════════════════════════════════════════════════════════════════
DO_INPUT:
        BSTA,UN WPEEK_UC
        COMI,R1 'A'
        BCTA,LT SYNERR
        COMI,R1 'Z'+1
        BCTA,LT DIN_VAROK
        BCTA,UN SYNERR
DIN_VAROK:
        BSTA,UN GETCI
        BSTA,UN UC
        STRA,R1 TMPCHR

        BSTA,UN CHECK_EOL
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DIN_PROMPT
        BCTA,UN SYNERR

DIN_PROMPT:
        LODI,R1 '?'
        BSTA,UN PUTCH
        LODI,R1 SP
        BSTA,UN PUTCH

        BSTA,UN RDLINE
        LODI,R0 >IBUF
        STRA,R0 IPTR
        LODI,R0 <IBUF
        STRA,R0 IPTR+1

        BSTA,UN PARSE_S16
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ SYNERR
        BSTA,UN CHECK_EOL
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DIN_STORE
        BCTA,UN SYNERR

DIN_STORE:
        LODA,R1 TMPCHR
        BSTA,UN GET_VARPTR
        LODA,R0 VALHI
        STRA,R0 *SPTR
        BSTA,UN INC_SPTR
        LODA,R0 VALLO
        STRA,R0 *SPTR
        RETC,UN

; ════════════════════════════════════════════════════════════════
; DO_IF  — IF expr relop expr THEN statement
; relop: =  <>  <  >  <=  >=
; ════════════════════════════════════════════════════════════════
DO_IF:
        BSTA,UN PARSE_EXPR
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DIF_LSAVE
        BCTA,UN SYNERR
DIF_LSAVE:
        LODA,R0 VALHI
        STRA,R0 LEFTV
        LODA,R0 VALLO
        STRA,R0 RIGHTV

        BSTA,UN PARSE_RELOP
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DIF_RPARSE
        BCTA,UN SYNERR

DIF_RPARSE:
        BSTA,UN PARSE_EXPR
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DIF_RSAVE
        BCTA,UN SYNERR
DIF_RSAVE:
        LODA,R0 VALHI
        STRA,R0 RGTHI
        LODA,R0 VALLO
        STRA,R0 RGTLO

        BSTA,UN EXPECT_THEN
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DIF_EVAL
        BCTA,UN SYNERR

DIF_EVAL:
        LODI,R0 $00
        STRA,R0 FOUND
        BSTA,UN CMP_LR_S16          ; NUMTMP: -1 LT,0 EQ,+1 GT
        LODA,R1 RELOP
        COMI,R1 1
        BCTA,EQ DIF_EQ
        COMI,R1 2
        BCTA,EQ DIF_NE
        COMI,R1 3
        BCTA,EQ DIF_LT
        COMI,R1 4
        BCTA,EQ DIF_GT
        COMI,R1 5
        BCTA,EQ DIF_LE
        COMI,R1 6
        BCTA,EQ DIF_GE
        BCTA,UN SYNERR

DIF_EQ:
        LODA,R0 NUMTMP
        COMI,R0 $00
        BCTA,EQ DIF_TRUE
        BCTA,UN DIF_DONE
DIF_NE:
        LODA,R0 NUMTMP
        COMI,R0 $00
        BCTA,EQ DIF_DONE
        BCTA,UN DIF_TRUE
DIF_LT:
        LODA,R0 NUMTMP
        COMI,R0 $FF
        BCTA,EQ DIF_TRUE
        BCTA,UN DIF_DONE
DIF_GT:
        LODA,R0 NUMTMP
        COMI,R0 $01
        BCTA,EQ DIF_TRUE
        BCTA,UN DIF_DONE
DIF_LE:
        LODA,R0 NUMTMP
        COMI,R0 $FF
        BCTA,EQ DIF_TRUE
        COMI,R0 $00
        BCTA,EQ DIF_TRUE
        BCTA,UN DIF_DONE
DIF_GE:
        LODA,R0 NUMTMP
        COMI,R0 $01
        BCTA,EQ DIF_TRUE
        COMI,R0 $00
        BCTA,EQ DIF_TRUE
        BCTA,UN DIF_DONE

DIF_TRUE:
        LODI,R0 $01
        STRA,R0 FOUND

DIF_DONE:
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ DIF_RET
        BSTA,UN STMT_LINE
DIF_RET:
        RETC,UN

; ════════════════════════════════════════════════════════════════
; DO_GOTO — GOTO n (effective during RUN)
; ════════════════════════════════════════════════════════════════
DO_GOTO:
        BSTA,UN PARSE_U16
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ SYNERR
        LODA,R0 VALHI
        STRA,R0 GOTOLHI
        LODA,R0 VALLO
        STRA,R0 GOTOLLO
        BSTA,UN CHECK_EOL
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DG_CHK
        BCTA,UN SYNERR
DG_CHK:
        LODA,R0 RUNFLG
        COMI,R0 $00
        BCTA,EQ SYNERR
        LODI,R0 $01
        STRA,R0 GOTOFLG
        RETC,UN

; ════════════════════════════════════════════════════════════════
; TRY_STORE_LINE — if line starts with number: store/delete program line
; FOUND=1 when handled as program line; FOUND=0 immediate command
; ════════════════════════════════════════════════════════════════
TRY_STORE_LINE:
        LODI,R0 $00
        STRA,R0 FOUND
        BSTA,UN WPEEK
        COMI,R1 '0'
        BCTA,LT TSL_RET
        COMI,R1 '9'+1
        BCTA,LT TSL_PARSE
        BCTA,UN TSL_RET
TSL_PARSE:
        BSTA,UN PARSE_U16
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ TSL_RET
        LODA,R0 VALHI
        STRA,R0 LNUMHI
        LODA,R0 VALLO
        STRA,R0 LNUMLO
        BSTA,UN WPEEK
        COMI,R1 NUL
        BCTA,EQ TSL_DEL
        BSTA,UN STORE_LINE
        BCTA,UN TSL_DONE
TSL_DEL:
        BSTA,UN DELETE_LINE
TSL_DONE:
        LODI,R0 $01
        STRA,R0 FOUND
TSL_RET:
        RETC,UN

; STORE_LINE — line number in LNUMHI:LNUMLO, text tail at IPTR
STORE_LINE:
        BSTA,UN DELETE_LINE
        ; measure tail length into TMPLEN
        LODI,R0 $00
        STRA,R0 TMPLEN
        LODA,R0 IPTR
        STRA,R0 P1HI
        LODA,R0 IPTR+1
        STRA,R0 P1LO
SL_MLP:
        LODA,R1 *P1HI
        COMI,R1 NUL
        BCTA,EQ SL_MDONE
        BSTA,UN INC_P1
        LODA,R0 TMPLEN
        ADDI,R0 1
        STRA,R0 TMPLEN
        BCTA,UN SL_MLP
SL_MDONE:
        LODA,R0 TMPLEN
        ADDI,R0 3
        STRA,R0 NUMTMP
        BSTA,UN CHECK_ROOM_NUMTMP
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ OOMERR
        ; find insertion pointer in P1
        BSTA,UN FIND_LINE_PTR
        ; P2 = current top
        LODA,R0 PROGHI
        STRA,R0 P2HI
        LODA,R0 PROGLO
        STRA,R0 P2LO
        ; shift up by (2+TMPLEN)
SL_SHP:
        LODA,R0 P2HI
        SUBA,R0 P1HI
        BCTA,LT SL_SHDONE
        BCTA,GT SL_SHMOVE
        LODA,R0 P2LO
        SUBA,R0 P1LO
        BCTA,LT SL_SHDONE
SL_SHMOVE:
        LODA,R1 *P2HI
        BSTA,UN ADD_NUMTMP_P2
        STRA,R1 *P2HI
        BSTA,UN DEC_P2
        BCTA,UN SL_SHP
SL_SHDONE:
        ; write record [line_hi][line_lo][len][text...]
        LODA,R1 LNUMHI
        STRA,R1 *P1HI
        BSTA,UN INC_P1
        LODA,R1 LNUMLO
        STRA,R1 *P1HI
        BSTA,UN INC_P1
        LODA,R1 TMPLEN
        STRA,R1 *P1HI
        BSTA,UN INC_P1
        LODA,R0 IPTR
        STRA,R0 P2HI
        LODA,R0 IPTR+1
        STRA,R0 P2LO
SL_WLP:
        LODA,R0 TMPLEN
        COMI,R0 $00
        BCTA,EQ SL_TOP
        LODA,R1 *P2HI
        STRA,R1 *P1HI
        BSTA,UN INC_P1
        BSTA,UN INC_P2
        LODA,R0 TMPLEN
        SUBI,R0 1
        STRA,R0 TMPLEN
        BCTA,UN SL_WLP
SL_TOP:
        BSTA,UN ADD_NUMTMP_PROG
        RETC,UN

; DELETE_LINE — remove LNUMHI:LNUMLO record if present
DELETE_LINE:
        BSTA,UN FIND_LINE_PTR
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ DL_RET
        ; P2 = next record
        LODA,R0 P1HI
        STRA,R0 P2HI
        LODA,R0 P1LO
        STRA,R0 P2LO
        BSTA,UN INC_P2              ; skip line_lo
        BSTA,UN INC_P2              ; read length at +2
        LODA,R0 *P2HI
        ADDI,R0 3
        STRA,R0 NUMTMP
        STRA,R0 TMPLEN
        ; restore P2 to record start, then advance to next
        BSTA,UN DEC_P2
        BSTA,UN REC_NEXT_P2
DL_CPY:
        LODA,R0 PROGHI
        SUBA,R0 P2HI
        BCTA,LT DL_FIN
        BCTA,GT DL_MOV
        LODA,R0 PROGLO
        SUBA,R0 P2LO
        BCTA,LT DL_FIN
DL_MOV:
        LODA,R1 *P2HI
        STRA,R1 *P1HI
        BSTA,UN INC_P1
        BSTA,UN INC_P2
        BCTA,UN DL_CPY
DL_FIN:
        LODA,R0 TMPLEN
        STRA,R0 NUMTMP
        BSTA,UN SUB_NUMTMP_PROG
        RETC,UN
DL_RET:
        RETC,UN

; FIND_LINE_PTR — search LNUMHI:LNUMLO
; returns P1 at found line or insertion point, FOUND=1 if exact
FIND_LINE_PTR:
        LODI,R0 $00
        STRA,R0 FOUND
        LODI,R0 >PROGBASE
        STRA,R0 P1HI
        LODI,R0 <PROGBASE
        STRA,R0 P1LO
FLP_LP:
        LODA,R0 P1HI
        SUBA,R0 PROGHI
        BCTA,GT FLP_DONE
        BCTA,LT FLP_CHK
        LODA,R0 P1LO
        SUBA,R0 PROGLO
        BCTA,GT FLP_DONE
        BCTA,EQ FLP_DONE
FLP_CHK:
        LODA,R0 *P1HI
        SUBA,R0 LNUMHI
        BCTA,LT FLP_ADV
        BCTA,GT FLP_DONE
        BSTA,UN INC_P1
        LODA,R0 *P1HI
        SUBA,R0 LNUMLO
        BSTA,UN DEC_P1
        BCTA,EQ FLP_HIT
        BCTA,GT FLP_DONE
FLP_ADV:
        BSTA,UN REC_NEXT_P1
        BCTA,UN FLP_LP
FLP_HIT:
        LODI,R0 $01
        STRA,R0 FOUND
FLP_DONE:
        RETC,UN

; DO_LIST — print stored lines
DO_LIST:
        LODI,R0 >PROGBASE
        STRA,R0 P1HI
        LODI,R0 <PROGBASE
        STRA,R0 P1LO
DLIST_LP:
        LODA,R0 P1HI
        SUBA,R0 PROGHI
        BCTA,GT DLIST_RET
        BCTA,LT DLIST_P
        LODA,R0 P1LO
        SUBA,R0 PROGLO
        BCTA,GT DLIST_RET
        BCTA,EQ DLIST_RET
DLIST_P:
        LODA,R0 P1HI
        STRA,R0 CURHI
        LODA,R0 P1LO
        STRA,R0 CURLO
        BSTA,UN VALID_REC_CUR
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ MALFERR
        LODA,R0 *P1HI
        STRA,R0 VALHI
        BSTA,UN INC_P1
        LODA,R0 *P1HI
        STRA,R0 VALLO
        BSTA,UN PRINT_U16
        LODI,R1 SP
        BSTA,UN PUTCH
        BSTA,UN INC_P1
        LODA,R0 *P1HI
        STRA,R0 TMPLEN
        BSTA,UN INC_P1
DLIST_TX:
        LODA,R0 TMPLEN
        COMI,R0 $00
        BCTA,EQ DLIST_NL
        LODA,R1 *P1HI
        BSTA,UN PUTCH
        BSTA,UN INC_P1
        LODA,R0 TMPLEN
        SUBI,R0 1
        STRA,R0 TMPLEN
        BCTA,UN DLIST_TX
DLIST_NL:
        BSTA,UN PRNL
        BCTA,UN DLIST_LP
DLIST_RET:
        RETC,UN

; DO_RUN — execute stored lines
DO_RUN:
        LODI,R0 $01
        STRA,R0 RUNFLG
        LODI,R0 $00
        STRA,R0 GOTOFLG
        LODI,R0 >PROGBASE
        STRA,R0 CURHI
        LODI,R0 <PROGBASE
        STRA,R0 CURLO
DR_LP:
        LODA,R0 RUNFLG
        COMI,R0 $00
        BCTA,EQ DR_RET
        ; end?
        LODA,R0 CURHI
        SUBA,R0 PROGHI
        BCTA,GT DR_STOP
        BCTA,LT DR_EXEC
        LODA,R0 CURLO
        SUBA,R0 PROGLO
        BCTA,GT DR_STOP
        BCTA,EQ DR_STOP
DR_EXEC:
        ; load text to IBUF
        LODA,R0 CURHI
        STRA,R0 P1HI
        LODA,R0 CURLO
        STRA,R0 P1LO
        ; defensive header check: need line_hi,line_lo,len
        BSTA,UN VALID_REC_CUR
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ MALFERR
        BSTA,UN INC_P1              ; skip line_hi
        BSTA,UN INC_P1              ; skip line_lo
        LODA,R0 *P1HI
        STRA,R0 TMPLEN
        BSTA,UN INC_P1
        LODI,R0 >IBUF
        STRA,R0 P2HI
        LODI,R0 <IBUF
        STRA,R0 P2LO
DR_CPY:
        LODA,R0 TMPLEN
        COMI,R0 $00
        BCTA,EQ DR_NUL
        LODA,R1 *P1HI
        STRA,R1 *P2HI
        BSTA,UN INC_P1
        BSTA,UN INC_P2
        LODA,R0 TMPLEN
        SUBI,R0 1
        STRA,R0 TMPLEN
        BCTA,UN DR_CPY
DR_NUL:
        LODI,R1 NUL
        STRA,R1 *P2HI
        ; set next pointer
        LODA,R0 CURHI
        STRA,R0 NEXTHI
        LODA,R0 CURLO
        STRA,R0 NEXTLO
        BSTA,UN REC_NEXT_NEXT
        ; execute line
        LODI,R0 >IBUF
        STRA,R0 IPTR
        LODI,R0 <IBUF
        STRA,R0 IPTR+1
        BSTA,UN STMT_LINE
        LODA,R0 GOTOFLG
        COMI,R0 $00
        BCTA,EQ DR_NEXT
        LODI,R0 $00
        STRA,R0 GOTOFLG
        LODA,R0 GOTOLHI
        STRA,R0 LNUMHI
        LODA,R0 GOTOLLO
        STRA,R0 LNUMLO
        BSTA,UN FIND_LINE_PTR
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ BADGOTO
        LODA,R0 P1HI
        STRA,R0 CURHI
        LODA,R0 P1LO
        STRA,R0 CURLO
        BCTA,UN DR_LP
DR_NEXT:
        LODA,R0 NEXTHI
        STRA,R0 CURHI
        LODA,R0 NEXTLO
        STRA,R0 CURLO
        BCTA,UN DR_LP
DR_STOP:
        LODI,R0 $00
        STRA,R0 RUNFLG
DR_RET:
        RETC,UN

; INIT_PROG — clear stored program area
INIT_PROG:
        LODI,R0 >PROGBASE
        STRA,R0 PROGHI
        LODI,R0 <PROGBASE
        STRA,R0 PROGLO
        LODI,R0 $00
        STRA,R0 RUNFLG
        STRA,R0 GOTOFLG
        RETC,UN

; REC_NEXT_P1/P2/NEXT — pointer += 3 + record_length
REC_NEXT_P1:
        BSTA,UN INC_P1
        BSTA,UN INC_P1
        LODA,R0 *P1HI
        STRA,R0 NUMTMP
        BSTA,UN INC_P1
RN1_LP:
        LODA,R0 NUMTMP
        COMI,R0 $00
        BCTA,EQ RN1_RET
        BSTA,UN INC_P1
        LODA,R0 NUMTMP
        SUBI,R0 1
        STRA,R0 NUMTMP
        BCTA,UN RN1_LP
RN1_RET:
        RETC,UN

REC_NEXT_P2:
        BSTA,UN INC_P2
        BSTA,UN INC_P2
        LODA,R0 *P2HI
        STRA,R0 NUMTMP
        BSTA,UN INC_P2
RN2_LP:
        LODA,R0 NUMTMP
        COMI,R0 $00
        BCTA,EQ RN2_RET
        BSTA,UN INC_P2
        LODA,R0 NUMTMP
        SUBI,R0 1
        STRA,R0 NUMTMP
        BCTA,UN RN2_LP
RN2_RET:
        RETC,UN

REC_NEXT_NEXT:
        ; uses NEXTHI:NEXTLO as pointer
        BSTA,UN INC_NEXT
        BSTA,UN INC_NEXT
        LODA,R0 *NEXTHI
        STRA,R0 NUMTMP
        BSTA,UN INC_NEXT
RNN_LP:
        LODA,R0 NUMTMP
        COMI,R0 $00
        BCTA,EQ RNN_RET
        BSTA,UN INC_NEXT
        LODA,R0 NUMTMP
        SUBI,R0 1
        STRA,R0 NUMTMP
        BCTA,UN RNN_LP
RNN_RET:
        RETC,UN

ADD_NUMTMP_P2:
ANP2_LP:
        LODA,R0 NUMTMP
        COMI,R0 $00
        BCTA,EQ ANP2_RET
        BSTA,UN INC_P2
        LODA,R0 NUMTMP
        SUBI,R0 1
        STRA,R0 NUMTMP
        BCTA,UN ANP2_LP
ANP2_RET:
        RETC,UN

ADD_NUMTMP_PROG:
ANPG_LP:
        LODA,R0 NUMTMP
        COMI,R0 $00
        BCTA,EQ ANPG_RET
        LODA,R0 PROGLO
        ADDI,R0 1
        STRA,R0 PROGLO
        BCTA,GT ANPG_NC
        LODA,R0 PROGHI
        ADDI,R0 1
        STRA,R0 PROGHI
ANPG_NC:
        LODA,R0 NUMTMP
        SUBI,R0 1
        STRA,R0 NUMTMP
        BCTA,UN ANPG_LP
ANPG_RET:
        RETC,UN

SUB_NUMTMP_PROG:
SNPG_LP:
        LODA,R0 NUMTMP
        COMI,R0 $00
        BCTA,EQ SNPG_RET
        LODA,R0 PROGLO
        SUBI,R0 1
        STRA,R0 PROGLO
        BCTA,GT SNPG_NB
        LODA,R0 PROGHI
        SUBI,R0 1
        STRA,R0 PROGHI
SNPG_NB:
        LODA,R0 NUMTMP
        SUBI,R0 1
        STRA,R0 NUMTMP
        BCTA,UN SNPG_LP
SNPG_RET:
        RETC,UN

; CHECK_ROOM_NUMTMP — FOUND=1 if PROG + NUMTMP <= PROGLIM
CHECK_ROOM_NUMTMP:
        LODI,R0 $00
        STRA,R0 FOUND
        LODA,R0 PROGHI
        STRA,R0 P2HI
        LODA,R0 PROGLO
        STRA,R0 P2LO
        BSTA,UN ADD_NUMTMP_P2
        LODA,R0 P2HI
        SUBI,R0 >PROGLIM
        BCTA,LT CRM_OK
        BCTA,GT CRM_RET
        LODA,R0 P2LO
        SUBI,R0 <PROGLIM
        BCTA,GT CRM_RET
CRM_OK:
        LODI,R0 $01
        STRA,R0 FOUND
CRM_RET:
        RETC,UN

; VALID_REC_CUR — validate record at CURHI:CURLO against PROG top
; FOUND=1 valid, FOUND=0 malformed
VALID_REC_CUR:
        LODI,R0 $00
        STRA,R0 FOUND
        ; need at least 3 bytes before top
        LODA,R0 CURHI
        STRA,R0 P1HI
        LODA,R0 CURLO
        STRA,R0 P1LO
        BSTA,UN INC_P1
        BSTA,UN INC_P1
        ; now at len byte
        LODA,R0 P1HI
        SUBA,R0 PROGHI
        BCTA,GT VRC_RET
        BCTA,LT VRC_LEN
        LODA,R0 P1LO
        SUBA,R0 PROGLO
        BCTA,GT VRC_RET
        BCTA,EQ VRC_RET
VRC_LEN:
        LODA,R0 *P1HI
        STRA,R0 NUMTMP
        ; P1 currently on len; advance to first byte after record
        BSTA,UN INC_P1
VRC_LP:
        LODA,R0 NUMTMP
        COMI,R0 $00
        BCTA,EQ VRC_CHKTOP
        BSTA,UN INC_P1
        LODA,R0 NUMTMP
        SUBI,R0 1
        STRA,R0 NUMTMP
        BCTA,UN VRC_LP
VRC_CHKTOP:
        LODA,R0 P1HI
        SUBA,R0 PROGHI
        BCTA,GT VRC_RET
        BCTA,LT VRC_OK
        LODA,R0 P1LO
        SUBA,R0 PROGLO
        BCTA,GT VRC_RET
VRC_OK:
        LODI,R0 $01
        STRA,R0 FOUND
VRC_RET:
        RETC,UN

INC_P1:
        LODA,R0 P1LO
        ADDI,R0 1
        STRA,R0 P1LO
        BCTA,GT IP1_RET
        LODA,R0 P1HI
        ADDI,R0 1
        STRA,R0 P1HI
IP1_RET:
        RETC,UN

DEC_P1:
        LODA,R0 P1LO
        SUBI,R0 1
        STRA,R0 P1LO
        BCTA,GT DP1_RET
        LODA,R0 P1HI
        SUBI,R0 1
        STRA,R0 P1HI
DP1_RET:
        RETC,UN

INC_P2:
        LODA,R0 P2LO
        ADDI,R0 1
        STRA,R0 P2LO
        BCTA,GT IP2_RET
        LODA,R0 P2HI
        ADDI,R0 1
        STRA,R0 P2HI
IP2_RET:
        RETC,UN

DEC_P2:
        LODA,R0 P2LO
        SUBI,R0 1
        STRA,R0 P2LO
        BCTA,GT DP2_RET
        LODA,R0 P2HI
        SUBI,R0 1
        STRA,R0 P2HI
DP2_RET:
        RETC,UN

INC_NEXT:
        LODA,R0 NEXTLO
        ADDI,R0 1
        STRA,R0 NEXTLO
        BCTA,GT INX_RET
        LODA,R0 NEXTHI
        ADDI,R0 1
        STRA,R0 NEXTHI
INX_RET:
        RETC,UN

; ════════════════════════════════════════════════════════════════
; PARSE_FACTOR — variable, literal, parenthesized expr, unary +/- factor
; Exit: VALHI:VALLO set, ERRFLG=0 on success
;       ERRFLG=1 on failure
; ════════════════════════════════════════════════════════════════
PARSE_FACTOR:
        LODI,R0 $01
        STRA,R0 ERRFLG
        BSTA,UN WPEEK_UC

        COMI,R1 '+'
        BCTA,EQ PF_UPLUS
        COMI,R1 '-'
        BCTA,EQ PF_UMINUS
        COMI,R1 '('
        BCTA,EQ PF_PAREN

        ; variable A-Z?
        COMI,R1 'A'
        BCTA,LT PV_TRY_NUM
        COMI,R1 'Z'+1
        BCTA,LT PV_VAR

PV_TRY_NUM:
        COMI,R1 '-'
        BCTA,EQ PV_NUM
        ; decimal digit?
        BSTA,UN WPEEK
        COMI,R1 '0'
        BCTA,LT PV_FAIL
        COMI,R1 '9'+1
        BCTA,LT PV_NUM
        BCTA,UN PV_FAIL

PV_VAR:
        BSTA,UN GETCI
        BSTA,UN UC
        BSTA,UN GET_VARPTR
        LODA,R0 *SPTR
        STRA,R0 VALHI
        BSTA,UN INC_SPTR
        LODA,R0 *SPTR
        STRA,R0 VALLO
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN

PV_NUM:
        BSTA,UN PARSE_S16
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ PV_FAIL
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN

PV_FAIL:
        LODI,R0 $01
        STRA,R0 ERRFLG
        RETC,UN

PF_UPLUS:
        BSTA,UN GETCI
        BSTA,UN PARSE_FACTOR
        RETC,UN

PF_UMINUS:
        BSTA,UN GETCI
        BSTA,UN PARSE_FACTOR
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ PF_NEG
        RETC,UN
PF_NEG:
        BSTA,UN NEG_VAL16
        RETC,UN

PF_PAREN:
        BSTA,UN GETCI
        BSTA,UN PARSE_EXPR
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ PF_PCH
        RETC,UN
PF_PCH:
        BSTA,UN WPEEK
        COMI,R1 ')'
        BCTA,EQ PF_POK
        LODI,R0 $01
        STRA,R0 ERRFLG
        RETC,UN
PF_POK:
        BSTA,UN GETCI
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN

; PARSE_TERM  — PARSE_FACTOR { (*|/) PARSE_FACTOR }*
PARSE_TERM:
        LODI,R0 $01
        STRA,R0 ERRFLG
        BSTA,UN PARSE_FACTOR
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ PT_HAVE
        RETC,UN
PT_HAVE:
        LODA,R0 VALHI
        STRA,R0 ACCHI
        LODA,R0 VALLO
        STRA,R0 ACCLO
PT_LP:
        BSTA,UN WPEEK
        COMI,R1 '*'
        BCTA,EQ PT_MUL
        COMI,R1 '/'
        BCTA,EQ PT_DIV
        BCTA,UN PT_DONE
PT_MUL:
        BSTA,UN GETCI
        BSTA,UN PARSE_FACTOR
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ PT_DOMUL
        RETC,UN
PT_DOMUL:
        BSTA,UN ACC_MUL_VAL
        BCTA,UN PT_LP
PT_DIV:
        BSTA,UN GETCI
        BSTA,UN PARSE_FACTOR
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ PT_DODIV
        RETC,UN
PT_DODIV:
        BSTA,UN ACC_DIV_VAL
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ PT_LP
        RETC,UN
PT_DONE:
        LODA,R0 ACCHI
        STRA,R0 VALHI
        LODA,R0 ACCLO
        STRA,R0 VALLO
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN

; ════════════════════════════════════════════════════════════════
; PARSE_EXPR  — PARSE_TERM { (+|-) PARSE_TERM }*
; Exit: VALHI:VALLO=result, ERRFLG=0 success
; ════════════════════════════════════════════════════════════════
PARSE_EXPR:
        LODI,R0 $01
        STRA,R0 ERRFLG
        BSTA,UN PARSE_TERM
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ PX_HAVE
        RETC,UN
PX_HAVE:
        LODA,R0 VALHI
        STRA,R0 ACCHI
        LODA,R0 VALLO
        STRA,R0 ACCLO

PX_LP:
        BSTA,UN WPEEK
        COMI,R1 '+'
        BCTA,EQ PX_ADD
        COMI,R1 '-'
        BCTA,EQ PX_SUB
        BCTA,UN PX_DONE

PX_ADD:
        BSTA,UN GETCI               ; consume '+'
        BSTA,UN PARSE_TERM
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ PX_DOADD
        RETC,UN
PX_DOADD:
        BSTA,UN ACC_ADD_VAL
        BCTA,UN PX_LP

PX_SUB:
        BSTA,UN GETCI               ; consume '-'
        BSTA,UN PARSE_TERM
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ PX_DOSUB
        RETC,UN
PX_DOSUB:
        BSTA,UN ACC_SUB_VAL
        BCTA,UN PX_LP

PX_DONE:
        LODA,R0 ACCHI
        STRA,R0 VALHI
        LODA,R0 ACCLO
        STRA,R0 VALLO
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN

; ════════════════════════════════════════════════════════════════
; PARSE_S16  — parse optional '-' + decimal digits into VALHI:VALLO
; FOUND=1 when at least one digit parsed
; ════════════════════════════════════════════════════════════════
PARSE_S16:
        LODI,R0 $00
        STRA,R0 NEGFLG
        BSTA,UN WPEEK
        COMI,R1 '-'
        BCTA,EQ PS16_NEG
        BCTA,UN PS16_UN
PS16_NEG:
        BSTA,UN GETCI
        LODI,R0 $01
        STRA,R0 NEGFLG
PS16_UN:
        BSTA,UN PARSE_U16
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ PS16_RET
        LODA,R0 NEGFLG
        COMI,R0 $00
        BCTA,EQ PS16_RET
        BSTA,UN NEG_VAL16
PS16_RET:
        RETC,UN

; ════════════════════════════════════════════════════════════════
; PARSE_U16  — parse unsigned decimal into VALHI:VALLO
; FOUND=1 if at least one digit
; ════════════════════════════════════════════════════════════════
PARSE_U16:
        LODI,R0 $00
        STRA,R0 VALHI
        STRA,R0 VALLO
        STRA,R0 FOUND
PU16_LP:
        BSTA,UN WPEEK
        COMI,R1 '0'
        BCTA,LT PU16_DONE
        COMI,R1 '9'+1
        BCTA,LT PU16_DIG
        BCTA,UN PU16_DONE
PU16_DIG:
        BSTA,UN GETCI
        SUBI,R1 '0'
        STRA,R1 DIGIT
        BSTA,UN MUL10_VAL16
        LODA,R0 VALLO
        ADDA,R0 DIGIT
        STRA,R0 VALLO
        BCTA,GT PU16_NC
        LODA,R0 VALHI
        ADDI,R0 1
        STRA,R0 VALHI
PU16_NC:
        LODI,R0 $01
        STRA,R0 FOUND
        BCTA,UN PU16_LP
PU16_DONE:
        RETC,UN

; MUL10_VAL16: VAL = VAL * 10
MUL10_VAL16:
        LODA,R0 VALHI
        STRA,R0 TMPHI2
        LODA,R0 VALLO
        STRA,R0 TMPLO2
        LODI,R0 $00
        STRA,R0 VALHI
        STRA,R0 VALLO
        LODI,R2 10
M10_LP:
        LODA,R0 VALLO
        ADDA,R0 TMPLO2
        STRA,R0 VALLO
        BCTA,GT M10_NC
        LODA,R0 VALHI
        ADDI,R0 1
        STRA,R0 VALHI
M10_NC:
        LODA,R0 VALHI
        ADDA,R0 TMPHI2
        STRA,R0 VALHI
        SUBI,R2 1
        BCTA,GT M10_LP
        RETC,UN

; NEG_VAL16: VAL = -VAL
NEG_VAL16:
        LODA,R0 VALHI
        EORI,R0 $FF
        STRA,R0 VALHI
        LODA,R0 VALLO
        EORI,R0 $FF
        STRA,R0 VALLO
        LODA,R0 VALLO
        ADDI,R0 1
        STRA,R0 VALLO
        BCTA,GT NV16_RET
        LODA,R0 VALHI
        ADDI,R0 1
        STRA,R0 VALHI
NV16_RET:
        RETC,UN

; ACC += VAL
ACC_ADD_VAL:
        LODA,R0 ACCLO
        ADDA,R0 VALLO
        STRA,R0 ACCLO
        BCTA,GT AAV_NC
        LODA,R0 ACCHI
        ADDI,R0 1
        STRA,R0 ACCHI
AAV_NC:
        LODA,R0 ACCHI
        ADDA,R0 VALHI
        STRA,R0 ACCHI
        RETC,UN

; ACC -= VAL
ACC_SUB_VAL:
        LODA,R0 ACCLO
        STRA,R0 TMPLO2
        SUBA,R0 VALLO
        STRA,R0 ACCLO
        LODA,R0 ACCHI
        SUBA,R0 VALHI
        STRA,R0 ACCHI
        LODA,R0 TMPLO2
        SUBA,R0 VALLO
        BCTA,GT ASV_RET
        LODA,R0 ACCHI
        SUBI,R0 1
        STRA,R0 ACCHI
ASV_RET:
        RETC,UN

; ACC *= VAL (signed 16-bit, wrap on overflow)
ACC_MUL_VAL:
        ; track sign in DIGCNT (0=+,1=-)
        LODI,R0 $00
        STRA,R0 DIGCNT
        ; abs(ACC) -> LEFTV:RIGHTV
        LODA,R0 ACCHI
        STRA,R0 LEFTV
        LODA,R0 ACCLO
        STRA,R0 RIGHTV
        LODA,R0 LEFTV
        COMI,R0 $80
        BCTA,LT AMV_ACCABS
        LODI,R0 $01
        STRA,R0 DIGCNT
        BSTA,UN NEG_LEFT16
AMV_ACCABS:
        ; abs(VAL) -> RGTHI:RGTLO
        LODA,R0 VALHI
        STRA,R0 RGTHI
        LODA,R0 VALLO
        STRA,R0 RGTLO
        LODA,R0 RGTHI
        COMI,R0 $80
        BCTA,LT AMV_VALABS
        LODA,R0 DIGCNT
        EORI,R0 $01
        STRA,R0 DIGCNT
        BSTA,UN NEG_RGT16
AMV_VALABS:
        ; result = 0
        LODI,R0 $00
        STRA,R0 ACCHI
        STRA,R0 ACCLO
AMV_LP:
        LODA,R0 RGTHI
        COMI,R0 $00
        BCTA,GT AMV_ADD
        LODA,R0 RGTLO
        COMI,R0 $00
        BCTA,EQ AMV_DONE
AMV_ADD:
        LODA,R0 ACCLO
        ADDA,R0 RIGHTV
        STRA,R0 ACCLO
        BCTA,GT AMV_NC
        LODA,R0 ACCHI
        ADDI,R0 1
        STRA,R0 ACCHI
AMV_NC:
        LODA,R0 ACCHI
        ADDA,R0 LEFTV
        STRA,R0 ACCHI
        BSTA,UN DEC_RGT16
        BCTA,UN AMV_LP
AMV_DONE:
        LODA,R0 DIGCNT
        COMI,R0 $00
        BCTA,EQ AMV_RET
        LODA,R0 ACCHI
        STRA,R0 VALHI
        LODA,R0 ACCLO
        STRA,R0 VALLO
        BSTA,UN NEG_VAL16
        LODA,R0 VALHI
        STRA,R0 ACCHI
        LODA,R0 VALLO
        STRA,R0 ACCLO
AMV_RET:
        RETC,UN

; ACC /= VAL (signed 16-bit, trunc toward zero), ERRFLG=1 on divide-by-zero
ACC_DIV_VAL:
        LODI,R0 $00
        STRA,R0 ERRFLG
        ; divide-by-zero?
        LODA,R0 VALHI
        COMI,R0 $00
        BCTA,GT ADV_NZ
        LODA,R0 VALLO
        COMI,R0 $00
        BCTA,EQ ADV_Z
ADV_NZ:
        LODI,R0 $00
        STRA,R0 DIGCNT              ; sign flag
        ; abs(dividend) in LEFTV:RIGHTV
        LODA,R0 ACCHI
        STRA,R0 LEFTV
        LODA,R0 ACCLO
        STRA,R0 RIGHTV
        LODA,R0 LEFTV
        COMI,R0 $80
        BCTA,LT ADV_DA
        LODI,R0 $01
        STRA,R0 DIGCNT
        BSTA,UN NEG_LEFT16
ADV_DA:
        ; abs(divisor) in RGTHI:RGTLO
        LODA,R0 VALHI
        STRA,R0 RGTHI
        LODA,R0 VALLO
        STRA,R0 RGTLO
        LODA,R0 RGTHI
        COMI,R0 $80
        BCTA,LT ADV_VA
        LODA,R0 DIGCNT
        EORI,R0 $01
        STRA,R0 DIGCNT
        BSTA,UN NEG_RGT16
ADV_VA:
        ; quotient in ACCHI:ACCLO = 0
        LODI,R0 $00
        STRA,R0 ACCHI
        STRA,R0 ACCLO
ADV_LP:
        BSTA,UN CMP_LEFT_RGT_U16
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ ADV_DONE
        BSTA,UN SUB_LEFT_RGT_U16
        ; quotient++
        LODA,R0 ACCLO
        ADDI,R0 1
        STRA,R0 ACCLO
        BCTA,GT ADV_LP
        LODA,R0 ACCHI
        ADDI,R0 1
        STRA,R0 ACCHI
        BCTA,UN ADV_LP
ADV_DONE:
        LODA,R0 DIGCNT
        COMI,R0 $00
        BCTA,EQ ADV_RET
        LODA,R0 ACCHI
        STRA,R0 VALHI
        LODA,R0 ACCLO
        STRA,R0 VALLO
        BSTA,UN NEG_VAL16
        LODA,R0 VALHI
        STRA,R0 ACCHI
        LODA,R0 VALLO
        STRA,R0 ACCLO
ADV_RET:
        RETC,UN
ADV_Z:
        LODI,R0 $01
        STRA,R0 ERRFLG
        RETC,UN

; CMP_LR_S16:
; LEFT  = LEFTV:RIGHTV
; RIGHT = RGTHI:RGTLO
; NUMTMP = $FF (LT), $00 (EQ), $01 (GT), signed compare
CMP_LR_S16:
        LODA,R0 LEFTV
        EORI,R0 $80
        STRA,R0 TMPHI2
        LODA,R0 RGTHI
        EORI,R0 $80
        STRA,R0 NUMVAL
        LODA,R0 TMPHI2
        SUBA,R0 NUMVAL
        BCTA,LT CLS_LT
        BCTA,GT CLS_GT
        LODA,R0 RIGHTV
        SUBA,R0 RGTLO
        BCTA,LT CLS_LT
        BCTA,GT CLS_GT
        LODI,R0 $00
        STRA,R0 NUMTMP
        RETC,UN
CLS_LT:
        LODI,R0 $FF
        STRA,R0 NUMTMP
        RETC,UN
CLS_GT:
        LODI,R0 $01
        STRA,R0 NUMTMP
        RETC,UN

; LEFTV:RIGHTV = -LEFTV:RIGHTV
NEG_LEFT16:
        LODA,R0 LEFTV
        EORI,R0 $FF
        STRA,R0 LEFTV
        LODA,R0 RIGHTV
        EORI,R0 $FF
        STRA,R0 RIGHTV
        LODA,R0 RIGHTV
        ADDI,R0 1
        STRA,R0 RIGHTV
        BCTA,GT NL16_RET
        LODA,R0 LEFTV
        ADDI,R0 1
        STRA,R0 LEFTV
NL16_RET:
        RETC,UN

; RGTHI:RGTLO = -RGTHI:RGTLO
NEG_RGT16:
        LODA,R0 RGTHI
        EORI,R0 $FF
        STRA,R0 RGTHI
        LODA,R0 RGTLO
        EORI,R0 $FF
        STRA,R0 RGTLO
        LODA,R0 RGTLO
        ADDI,R0 1
        STRA,R0 RGTLO
        BCTA,GT NR16_RET
        LODA,R0 RGTHI
        ADDI,R0 1
        STRA,R0 RGTHI
NR16_RET:
        RETC,UN

DEC_RGT16:
        LODA,R0 RGTLO
        SUBI,R0 1
        STRA,R0 RGTLO
        BCTA,GT DR16_RET
        LODA,R0 RGTHI
        SUBI,R0 1
        STRA,R0 RGTHI
DR16_RET:
        RETC,UN

; FOUND=1 if LEFTV:RIGHTV >= RGTHI:RGTLO (unsigned)
CMP_LEFT_RGT_U16:
        LODI,R0 $00
        STRA,R0 FOUND
        LODA,R0 LEFTV
        SUBA,R0 RGTHI
        BCTA,GT CLU_Y
        BCTA,LT CLU_R
        LODA,R0 RIGHTV
        SUBA,R0 RGTLO
        BCTA,LT CLU_R
CLU_Y:
        LODI,R0 $01
        STRA,R0 FOUND
CLU_R:
        RETC,UN

; LEFTV:RIGHTV -= RGTHI:RGTLO
SUB_LEFT_RGT_U16:
        LODA,R0 RIGHTV
        STRA,R0 TMPLO2
        SUBA,R0 RGTLO
        STRA,R0 RIGHTV
        LODA,R0 LEFTV
        SUBA,R0 RGTHI
        STRA,R0 LEFTV
        LODA,R0 TMPLO2
        SUBA,R0 RGTLO
        BCTA,GT SLR_RET
        LODA,R0 LEFTV
        SUBI,R0 1
        STRA,R0 LEFTV
SLR_RET:
        RETC,UN

; ════════════════════════════════════════════════════════════════
; PARSE_RELOP  — parses relational operator into RELOP code
; 1:=  2:<>  3:<  4:>  5:<=  6:>=
; ════════════════════════════════════════════════════════════════
PARSE_RELOP:
        LODI,R0 $01
        STRA,R0 ERRFLG
        BSTA,UN WPEEK
        COMI,R1 '='
        BCTA,EQ PRO_EQ
        COMI,R1 '<'
        BCTA,EQ PRO_LT
        COMI,R1 '>'
        BCTA,EQ PRO_GT
        RETC,UN

PRO_EQ:
        BSTA,UN GETCI
        LODI,R0 1
        STRA,R0 RELOP
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN

PRO_LT:
        BSTA,UN GETCI
        BSTA,UN WPEEK
        COMI,R1 '='
        BCTA,EQ PRO_LE
        COMI,R1 '>'
        BCTA,EQ PRO_NE
        LODI,R0 3
        STRA,R0 RELOP
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN

PRO_LE:
        BSTA,UN GETCI
        LODI,R0 5
        STRA,R0 RELOP
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN

PRO_NE:
        BSTA,UN GETCI
        LODI,R0 2
        STRA,R0 RELOP
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN

PRO_GT:
        BSTA,UN GETCI
        BSTA,UN WPEEK
        COMI,R1 '='
        BCTA,EQ PRO_GE
        LODI,R0 4
        STRA,R0 RELOP
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN

PRO_GE:
        BSTA,UN GETCI
        LODI,R0 6
        STRA,R0 RELOP
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN

; ════════════════════════════════════════════════════════════════
; EXPECT_THEN  — parses THEN (accepts TH or THEN...)
; ════════════════════════════════════════════════════════════════
EXPECT_THEN:
        LODI,R0 $01
        STRA,R0 ERRFLG
        BSTA,UN WPEEK_UC
        COMI,R1 'T'
        BCTA,EQ ETH_T
        RETC,UN
ETH_T:
        BSTA,UN GETCI
        BSTA,UN UC
        BSTA,UN WPEEK_UC
        COMI,R1 'H'
        BCTA,EQ ETH_H
        RETC,UN
ETH_H:
        BSTA,UN GETCI
        BSTA,UN UC
        BSTA,UN EATWORD
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN

; ════════════════════════════════════════════════════════════════
; PRINT_U16  — print unsigned 16-bit VALHI:VALLO
; ════════════════════════════════════════════════════════════════
PRINT_U16:
        ; zero?
        LODA,R0 VALHI
        COMI,R0 $00
        BCTA,GT PU16_NZ
        LODA,R0 VALLO
        COMI,R0 $00
        BCTA,GT PU16_NZ
        LODI,R1 '0'
        BSTA,UN PUTCH
        RETC,UN
PU16_NZ:
        LODI,R0 $00
        STRA,R0 DIGCNT
PU16_10K:
        LODI,R0 $00
        STRA,R0 NUMTMP
PU16_10K_LP:
        BSTA,UN GE_10000
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ PU16_1K
        BSTA,UN SUB_10000
        LODA,R0 NUMTMP
        ADDI,R0 1
        STRA,R0 NUMTMP
        BCTA,UN PU16_10K_LP
PU16_1K:
        LODA,R0 NUMTMP
        COMI,R0 $00
        BCTA,EQ PU16_1K_INIT
        ADDI,R0 '0'
        LODZ,R1
        BSTA,UN PUTCH
        LODI,R0 $01
        STRA,R0 DIGCNT
PU16_1K_INIT:
        LODI,R0 $00
        STRA,R0 NUMTMP
PU16_1K_LP:
        BSTA,UN GE_1000
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ PU16_100_INIT
        BSTA,UN SUB_1000
        LODA,R0 NUMTMP
        ADDI,R0 1
        STRA,R0 NUMTMP
        BCTA,UN PU16_1K_LP
PU16_100_INIT:
        LODA,R0 DIGCNT
        COMI,R0 $00
        BCTA,GT PU16_1K_P
        LODA,R0 NUMTMP
        COMI,R0 $00
        BCTA,EQ PU16_100
PU16_1K_P:
        LODA,R0 NUMTMP
        ADDI,R0 '0'
        LODZ,R1
        BSTA,UN PUTCH
        LODI,R0 $01
        STRA,R0 DIGCNT
PU16_100:
        LODI,R0 $00
        STRA,R0 NUMTMP
PU16_100_LP:
        BSTA,UN GE_100
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ PU16_10_INIT
        BSTA,UN SUB_100
        LODA,R0 NUMTMP
        ADDI,R0 1
        STRA,R0 NUMTMP
        BCTA,UN PU16_100_LP
PU16_10_INIT:
        LODA,R0 DIGCNT
        COMI,R0 $00
        BCTA,GT PU16_100_P
        LODA,R0 NUMTMP
        COMI,R0 $00
        BCTA,EQ PU16_10
PU16_100_P:
        LODA,R0 NUMTMP
        ADDI,R0 '0'
        LODZ,R1
        BSTA,UN PUTCH
        LODI,R0 $01
        STRA,R0 DIGCNT
PU16_10:
        LODI,R0 $00
        STRA,R0 NUMTMP
PU16_10_LP:
        BSTA,UN GE_10
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ PU16_1
        BSTA,UN SUB_10
        LODA,R0 NUMTMP
        ADDI,R0 1
        STRA,R0 NUMTMP
        BCTA,UN PU16_10_LP
PU16_1:
        LODA,R0 DIGCNT
        COMI,R0 $00
        BCTA,GT PU16_10_P
        LODA,R0 NUMTMP
        COMI,R0 $00
        BCTA,EQ PU16_LAST
PU16_10_P:
        LODA,R0 NUMTMP
        ADDI,R0 '0'
        LODZ,R1
        BSTA,UN PUTCH
PU16_LAST:
        LODA,R0 VALLO
        ADDI,R0 '0'
        LODZ,R1
        BSTA,UN PUTCH
        RETC,UN

; ════════════════════════════════════════════════════════════════
; PRINT_S16  — print signed 16-bit VALHI:VALLO
; ════════════════════════════════════════════════════════════════
PRINT_S16:
        ; sign handling
        LODA,R0 VALHI
        COMI,R0 $80
        BCTA,LT PS16_POS
        LODI,R1 '-'
        BSTA,UN PUTCH
        BSTA,UN NEG_VAL16
PS16_POS:
        ; zero?
        LODA,R0 VALHI
        COMI,R0 $00
        BCTA,GT PS16_NZ
        LODA,R0 VALLO
        COMI,R0 $00
        BCTA,GT PS16_NZ
        LODI,R1 '0'
        BSTA,UN PUTCH
        RETC,UN
PS16_NZ:
        LODI,R0 $00
        STRA,R0 DIGCNT              ; printed-flag

        ; 10000
        LODI,R0 $00
        STRA,R0 NUMTMP
PS16_10K_LP:
        BSTA,UN GE_10000
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ PS16_10K_DONE
        BSTA,UN SUB_10000
        LODA,R0 NUMTMP
        ADDI,R0 1
        STRA,R0 NUMTMP
        BCTA,UN PS16_10K_LP
PS16_10K_DONE:
        LODA,R0 NUMTMP
        COMI,R0 $00
        BCTA,EQ PS16_1K
        ADDI,R0 '0'
        LODZ,R1
        BSTA,UN PUTCH
        LODI,R0 $01
        STRA,R0 DIGCNT

        ; 1000
PS16_1K:
        LODI,R0 $00
        STRA,R0 NUMTMP
PS16_1K_LP:
        BSTA,UN GE_1000
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ PS16_1K_DONE
        BSTA,UN SUB_1000
        LODA,R0 NUMTMP
        ADDI,R0 1
        STRA,R0 NUMTMP
        BCTA,UN PS16_1K_LP
PS16_1K_DONE:
        LODA,R0 DIGCNT
        COMI,R0 $00
        BCTA,GT PS16_1K_P
        LODA,R0 NUMTMP
        COMI,R0 $00
        BCTA,EQ PS16_100
PS16_1K_P:
        LODA,R0 NUMTMP
        ADDI,R0 '0'
        LODZ,R1
        BSTA,UN PUTCH
        LODI,R0 $01
        STRA,R0 DIGCNT

        ; 100
PS16_100:
        LODI,R0 $00
        STRA,R0 NUMTMP
PS16_100_LP:
        BSTA,UN GE_100
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ PS16_100_DONE
        BSTA,UN SUB_100
        LODA,R0 NUMTMP
        ADDI,R0 1
        STRA,R0 NUMTMP
        BCTA,UN PS16_100_LP
PS16_100_DONE:
        LODA,R0 DIGCNT
        COMI,R0 $00
        BCTA,GT PS16_100_P
        LODA,R0 NUMTMP
        COMI,R0 $00
        BCTA,EQ PS16_10
PS16_100_P:
        LODA,R0 NUMTMP
        ADDI,R0 '0'
        LODZ,R1
        BSTA,UN PUTCH
        LODI,R0 $01
        STRA,R0 DIGCNT

        ; 10
PS16_10:
        LODI,R0 $00
        STRA,R0 NUMTMP
PS16_10_LP:
        BSTA,UN GE_10
        LODA,R0 FOUND
        COMI,R0 $00
        BCTA,EQ PS16_10_DONE
        BSTA,UN SUB_10
        LODA,R0 NUMTMP
        ADDI,R0 1
        STRA,R0 NUMTMP
        BCTA,UN PS16_10_LP
PS16_10_DONE:
        LODA,R0 DIGCNT
        COMI,R0 $00
        BCTA,GT PS16_10_P
        LODA,R0 NUMTMP
        COMI,R0 $00
        BCTA,EQ PS16_1
PS16_10_P:
        LODA,R0 NUMTMP
        ADDI,R0 '0'
        LODZ,R1
        BSTA,UN PUTCH

PS16_1:
        LODA,R0 VALLO
        ADDI,R0 '0'
        LODZ,R1
        BSTA,UN PUTCH
        RETC,UN

; unsigned compare helpers: FOUND=1 when VAL >= const
GE_10000:
        LODI,R0 $00
        STRA,R0 FOUND
        LODA,R0 VALHI
        COMI,R0 $27
        BCTA,GT GE10K_T
        BCTA,LT GE10K_R
        LODA,R0 VALLO
        COMI,R0 $10
        BCTA,LT GE10K_R
GE10K_T:
        LODI,R0 $01
        STRA,R0 FOUND
GE10K_R:
        RETC,UN

GE_1000:
        LODI,R0 $00
        STRA,R0 FOUND
        LODA,R0 VALHI
        COMI,R0 $03
        BCTA,GT GE1K_T
        BCTA,LT GE1K_R
        LODA,R0 VALLO
        COMI,R0 $E8
        BCTA,LT GE1K_R
GE1K_T:
        LODI,R0 $01
        STRA,R0 FOUND
GE1K_R:
        RETC,UN

GE_100:
        LODI,R0 $00
        STRA,R0 FOUND
        LODA,R0 VALHI
        COMI,R0 $00
        BCTA,GT GE100_T
        LODA,R0 VALLO
        COMI,R0 $64
        BCTA,LT GE100_R
GE100_T:
        LODI,R0 $01
        STRA,R0 FOUND
GE100_R:
        RETC,UN

GE_10:
        LODI,R0 $00
        STRA,R0 FOUND
        LODA,R0 VALHI
        COMI,R0 $00
        BCTA,GT GE10_T
        LODA,R0 VALLO
        COMI,R0 $0A
        BCTA,LT GE10_R
GE10_T:
        LODI,R0 $01
        STRA,R0 FOUND
GE10_R:
        RETC,UN

SUB_10000:
        LODA,R0 VALLO
        STRA,R0 TMPLO2
        SUBI,R0 $10
        STRA,R0 VALLO
        LODA,R0 VALHI
        SUBI,R0 $27
        STRA,R0 VALHI
        LODA,R0 TMPLO2
        COMI,R0 $10
        BCTA,LT S10K_B
        RETC,UN
S10K_B:
        LODA,R0 VALHI
        SUBI,R0 1
        STRA,R0 VALHI
        RETC,UN

SUB_1000:
        LODA,R0 VALLO
        STRA,R0 TMPLO2
        SUBI,R0 $E8
        STRA,R0 VALLO
        LODA,R0 VALHI
        SUBI,R0 $03
        STRA,R0 VALHI
        LODA,R0 TMPLO2
        COMI,R0 $E8
        BCTA,LT S1K_B
        RETC,UN
S1K_B:
        LODA,R0 VALHI
        SUBI,R0 1
        STRA,R0 VALHI
        RETC,UN

SUB_100:
        LODA,R0 VALLO
        STRA,R0 TMPLO2
        SUBI,R0 $64
        STRA,R0 VALLO
        LODA,R0 TMPLO2
        COMI,R0 $64
        BCTA,LT S100_B
        RETC,UN
S100_B:
        LODA,R0 VALHI
        SUBI,R0 1
        STRA,R0 VALHI
        RETC,UN

SUB_10:
        LODA,R0 VALLO
        STRA,R0 TMPLO2
        SUBI,R0 $0A
        STRA,R0 VALLO
        LODA,R0 TMPLO2
        COMI,R0 $0A
        BCTA,LT S10_B
        RETC,UN
S10_B:
        LODA,R0 VALHI
        SUBI,R0 1
        STRA,R0 VALHI
        RETC,UN

; ════════════════════════════════════════════════════════════════
; PARSE_U8  — parse unsigned decimal at IPTR
; Exit: R1=value (mod 256), FOUND=1 if at least one digit else 0
; ════════════════════════════════════════════════════════════════
PARSE_U8:
        LODI,R0 $00
        STRA,R0 NUMVAL
        STRA,R0 FOUND

PU_LP:
        BSTA,UN WPEEK
        COMI,R1 '0'
        BCTA,LT PU_DONE
        COMI,R1 '9'+1
        BCTA,LT PU_DIG
        BCTA,UN PU_DONE

PU_DIG:
        BSTA,UN GETCI               ; consume digit char into R1
        SUBI,R1 '0'
        STRA,R1 DIGIT

        ; NUMTMP = NUMVAL
        LODA,R0 NUMVAL
        STRA,R0 NUMTMP

        ; NUMVAL = NUMTMP * 10
        LODI,R0 $00
        STRA,R0 NUMVAL
        LODI,R2 10
PU_M10:
        LODA,R0 NUMVAL
        ADDA,R0 NUMTMP
        STRA,R0 NUMVAL
        SUBI,R2 1
        BCTA,GT PU_M10

        ; NUMVAL += DIGIT
        LODA,R0 NUMVAL
        ADDA,R0 DIGIT
        STRA,R0 NUMVAL

        LODI,R0 $01
        STRA,R0 FOUND
        BCTA,UN PU_LP

PU_DONE:
        LODA,R1 NUMVAL
        RETC,UN

; ════════════════════════════════════════════════════════════════
; PRINT_U8  — print unsigned value in R1 as decimal
; ════════════════════════════════════════════════════════════════
PRINT_U8:
        STRA,R1 NUMVAL
        LODI,R0 $00
        STRA,R0 HUND
        STRA,R0 TENS

        ; hundreds = NUMVAL / 100
PU8_HLP:
        LODA,R0 NUMVAL
        COMI,R0 100
        BCTA,LT PU8_TENS
        SUBI,R0 100
        STRA,R0 NUMVAL
        LODA,R0 HUND
        ADDI,R0 1
        STRA,R0 HUND
        BCTA,UN PU8_HLP

PU8_TENS:
        LODA,R0 NUMVAL
        COMI,R0 10
        BCTA,LT PU8_ONES
        SUBI,R0 10
        STRA,R0 NUMVAL
        LODA,R0 TENS
        ADDI,R0 1
        STRA,R0 TENS
        BCTA,UN PU8_TENS

PU8_ONES:
        LODA,R0 NUMVAL
        STRA,R0 ONES

        ; print hundreds if non-zero
        LODA,R0 HUND
        COMI,R0 $00
        BCTA,EQ PU8_CHK_T
        ADDI,R0 '0'
        LODZ,R1
        BSTA,UN PUTCH

PU8_CHK_T:
        ; print tens if hundreds>0 or tens>0
        LODA,R0 HUND
        COMI,R0 $00
        BCTA,GT PU8_PTENS
        LODA,R0 TENS
        COMI,R0 $00
        BCTA,EQ PU8_PONES
PU8_PTENS:
        LODA,R0 TENS
        ADDI,R0 '0'
        LODZ,R1
        BSTA,UN PUTCH

PU8_PONES:
        LODA,R0 ONES
        ADDI,R0 '0'
        LODZ,R1
        BSTA,UN PUTCH
        RETC,UN

; ════════════════════════════════════════════════════════════════
; GET_VARPTR  — SPTR -> variable storage slot from variable letter in R1
; Entry: R1='A'..'Z'
; Exit:  SPTR points to high byte of that variable
; ════════════════════════════════════════════════════════════════
GET_VARPTR:
        SUBI,R1 'A'                 ; index 0..25
        LODI,R0 >VARS
        STRA,R0 SPTR
        LODI,R0 <VARS
        STRA,R0 SPTR+1
GVP_LP:
        COMI,R1 $00
        BCTA,EQ GVP_RET
        BSTA,UN INC_SPTR
        BSTA,UN INC_SPTR
        SUBI,R1 1
        BCTA,UN GVP_LP
GVP_RET:
        RETC,UN

; ════════════════════════════════════════════════════════════════
; SYNERR  — print ?0 + newline
; ════════════════════════════════════════════════════════════════
SYNERR:
        LODI,R1 '?'
        BSTA,UN PUTCH
        LODI,R1 '0'
        BSTA,UN PUTCH
        BSTA,UN PRNL
        RETC,UN

OOMERR:
        LODI,R1 '?'
        BSTA,UN PUTCH
        LODI,R1 '1'
        BSTA,UN PUTCH
        BSTA,UN PRNL
        RETC,UN

BADGOTO:
        LODI,R1 '?'
        BSTA,UN PUTCH
        LODI,R1 '2'
        BSTA,UN PUTCH
        BSTA,UN PRNL
        LODI,R0 $00
        STRA,R0 RUNFLG
        RETC,UN

MALFERR:
        LODI,R1 '?'
        BSTA,UN PUTCH
        LODI,R1 '3'
        BSTA,UN PUTCH
        BSTA,UN PRNL
        LODI,R0 $00
        STRA,R0 RUNFLG
        RETC,UN

; ════════════════════════════════════════════════════════════════
; EATWORD  — consume trailing alphabetic chars [A-Za-z]
; Entry: IPTR at first char after 2-letter prefix
; ════════════════════════════════════════════════════════════════
EATWORD:
EW_LP:
        BSTA,UN PEEKC
        BSTA,UN UC
        COMI,R1 'A'
        BCTA,LT EW_RET
        COMI,R1 'Z'+1
        BCTA,LT EW_CONS
        BCTA,UN EW_RET
EW_CONS:
        BSTA,UN INC_IPTR
        BCTA,UN EW_LP
EW_RET:
        RETC,UN

; ════════════════════════════════════════════════════════════════
; CHECK_EOL  — after optional spaces, require NUL
; Exit: ERRFLG=0 if end-of-line, else ERRFLG=1
; ════════════════════════════════════════════════════════════════
CHECK_EOL:
        LODI,R0 $01
        STRA,R0 ERRFLG
        BSTA,UN WPEEK
        COMI,R1 NUL
        BCTA,EQ CE_OK
        RETC,UN
CE_OK:
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN

; ════════════════════════════════════════════════════════════════
; WPEEK  — skip spaces, then peek char into R1 (raw)
; ════════════════════════════════════════════════════════════════
WPEEK:
WP_LP:
        BSTA,UN PEEKC
        COMI,R1 SP
        BCTA,EQ WP_ADV
        RETC,UN
WP_ADV:
        BSTA,UN INC_IPTR
        BCTA,UN WP_LP

; ════════════════════════════════════════════════════════════════
; WPEEK_UC  — skip spaces, peek uppercase char into R1
; ════════════════════════════════════════════════════════════════
WPEEK_UC:
        BSTA,UN WPEEK
        BSTA,UN UC
        RETC,UN

; ════════════════════════════════════════════════════════════════
; PEEKC / GETCI / INC_IPTR helpers
; ════════════════════════════════════════════════════════════════
PEEKC:
        LODA,R1 *IPTR
        RETC,UN

GETCI:
        LODA,R1 *IPTR
        BSTA,UN INC_IPTR
        RETC,UN

INC_IPTR:
        LODA,R0 IPTR+1
        ADDI,R0 1
        STRA,R0 IPTR+1
        BCTA,GT INCIP_RET
        LODA,R0 IPTR
        ADDI,R0 1
        STRA,R0 IPTR
INCIP_RET:
        RETC,UN

; ════════════════════════════════════════════════════════════════
; INC_SPTR  — increment SPTR pointer
; ════════════════════════════════════════════════════════════════
INC_SPTR:
        LODA,R0 SPTR+1
        ADDI,R0 1
        STRA,R0 SPTR+1
        BCTA,GT INCSP_RET
        LODA,R0 SPTR
        ADDI,R0 1
        STRA,R0 SPTR
INCSP_RET:
        RETC,UN

; ════════════════════════════════════════════════════════════════
; UC  — uppercase conversion for ASCII in R1
; ════════════════════════════════════════════════════════════════
UC:
        COMI,R1 'a'
        BCTA,LT UC_RET
        COMI,R1 'z'+1
        BCTA,LT UC_DO
        BCTA,UN UC_RET
UC_DO:
        SUBI,R1 32
UC_RET:
        RETC,UN

; ════════════════════════════════════════════════════════════════
; PUTCH / GETCH / PRNL
; ════════════════════════════════════════════════════════════════
PUTCH:
        WRTD,R1
        RETC,UN

GETCH:
        REDE,R1
        RETC,UN

PRNL:
        LODI,R1 CR
        BSTA,UN PUTCH
        LODI,R1 LF
        BSTA,UN PUTCH
        RETC,UN

; ════════════════════════════════════════════════════════════════
; PRTSTR  — print NUL-terminated string
; Entry: SPTR points to string
; ════════════════════════════════════════════════════════════════
PRTSTR:
PRTSTR_LP:
        LODA,R1 *SPTR
        COMI,R1 NUL
        BCTA,EQ PRTSTR_RET
        BSTA,UN PUTCH
        LODA,R0 SPTR+1
        ADDI,R0 1
        STRA,R0 SPTR+1
        BCTA,GT PRTSTR_LP
        LODA,R0 SPTR
        ADDI,R0 1
        STRA,R0 SPTR
        BCTA,UN PRTSTR_LP
PRTSTR_RET:
        RETC,UN

; ════════════════════════════════════════════════════════════════
; RDLINE  — read line into IBUF, with backspace handling
; ════════════════════════════════════════════════════════════════
RDLINE:
        LODI,R0 >IBUF
        STRA,R0 LPTR
        LODI,R0 <IBUF
        STRA,R0 LPTR+1
        LODI,R0 $00
        STRA,R0 LCNT

RDLINE_LP:
        BSTA,UN GETCH
        COMI,R1 CR
        BCTA,EQ RDLINE_EOL
        COMI,R1 LF
        BCTA,EQ RDLINE_EOL

        COMI,R1 BS
        BCTA,EQ RDLINE_BS

        LODA,R0 LCNT
        COMI,R0 IBUFSZ
        BCTA,EQ RDLINE_LP
        BCTA,GT RDLINE_LP

        STRA,R1 *LPTR
        BSTA,UN PUTCH

        LODA,R0 LPTR+1
        ADDI,R0 1
        STRA,R0 LPTR+1
        BCTA,GT RDLINE_NCARRY
        LODA,R0 LPTR
        ADDI,R0 1
        STRA,R0 LPTR
RDLINE_NCARRY:
        LODA,R0 LCNT
        ADDI,R0 1
        STRA,R0 LCNT
        BCTA,UN RDLINE_LP

RDLINE_BS:
        LODA,R0 LCNT
        COMI,R0 $00
        BCTA,EQ RDLINE_LP

        LODA,R0 LPTR+1
        SUBI,R0 1
        STRA,R0 LPTR+1
        BCTA,GT RDLINE_BS_NCARRY
        LODA,R0 LPTR
        SUBI,R0 1
        STRA,R0 LPTR
RDLINE_BS_NCARRY:
        LODA,R0 LCNT
        SUBI,R0 1
        STRA,R0 LCNT

        LODI,R1 BS
        BSTA,UN PUTCH
        LODI,R1 SP
        BSTA,UN PUTCH
        LODI,R1 BS
        BSTA,UN PUTCH
        BCTA,UN RDLINE_LP

RDLINE_EOL:
        LODI,R1 NUL
        STRA,R1 *LPTR
        BSTA,UN PRNL
        RETC,UN

; ════════════════════════════════════════════════════════════════
; STRING CONSTANTS
; ════════════════════════════════════════════════════════════════
BANNER:
        DB      CR,LF
        DB      'u','B','A','S','I','C',' ','v','0','.','4'
        DB      CR,LF
        DB      '2','6','5','0',' ','p','o','r','t',' ','(','W','I','P',')'
        DB      CR,LF,NUL

        END
