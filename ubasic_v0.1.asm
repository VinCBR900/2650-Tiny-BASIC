; ubasic_v0.1.asm  -  Tiny BASIC work-in-progress for Signetics 2650
; -----------------------------------------------------------------------
; TARGET:   Signetics 2650 @ 1 MHz
;           ROM $0000-$0FFF (4KB), RAM $1000-$17FF (2KB)
; SIM I/O:  WRTD,R1 = putchar,  REDE,R1 = getchar
;
; v0.3: continues port by adding minimal numeric/variable support:
;       - LET / LE  variable assignment (A-Z, 8-bit value in low byte)
;       - PRINT / PR now prints quoted strings OR numeric value expressions
;       - expressions currently support: unsigned decimal literal, or variable
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

        ORG     $0000

; ════════════════════════════════════════════════════════════════
; RESET
; ════════════════════════════════════════════════════════════════
RESET:
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

        BSTA,UN STMT_LINE
        BCTA,UN REPL

; ════════════════════════════════════════════════════════════════
; STMT_LINE  — immediate-mode statement decode
; Supports PR/PRINT, LE/LET, RE/REM, EN/END
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
        COMI,R1 'L'
        BCTA,EQ ST_LET_2
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

ST_REM:
        BSTA,UN GETCI
        BSTA,UN UC
        COMI,R1 'R'
        BCTA,EQ ST_REM_2
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
        HALT

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

        ; parse expression into R1
        BSTA,UN PARSE_VALUE
        LODA,R0 ERRFLG
        COMI,R0 $00
        BCTA,EQ DOL_STORE
        BCTA,UN SYNERR

DOL_STORE:
        ; locate variable slot
        LODA,R1 TMPCHR
        BSTA,UN GET_VARPTR          ; SPTR -> var hi byte

        ; hi byte = 0
        LODI,R0 $00
        STRA,R0 *SPTR
        ; low byte = parsed value
        BSTA,UN INC_SPTR
        LODA,R0 TMPNUM
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
        BSTA,UN PARSE_VALUE
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
        BSTA,UN PRINT_U8
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
; PARSE_VALUE  — expression parser (decimal literal or variable)
; Exit: R1=value, TMPNUM=value, ERRFLG=0 on success
;       ERRFLG=1 on failure
; ════════════════════════════════════════════════════════════════
PARSE_VALUE:
        LODI,R0 $01
        STRA,R0 ERRFLG
        BSTA,UN WPEEK_UC

        ; variable A-Z?
        COMI,R1 'A'
        BCTA,LT PV_TRY_NUM
        COMI,R1 'Z'+1
        BCTA,LT PV_VAR

PV_TRY_NUM:
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
        BSTA,UN INC_SPTR            ; low byte
        LODA,R1 *SPTR
        STRA,R1 TMPNUM
        LODI,R0 $00
        STRA,R0 ERRFLG
        RETC,UN

PV_NUM:
        BSTA,UN PARSE_U8
        STRA,R1 TMPNUM
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
        DB      'u','B','A','S','I','C',' ','v','0','.','3'
        DB      CR,LF
        DB      '2','6','5','0',' ','p','o','r','t',' ','(','W','I','P',')'
        DB      CR,LF,NUL

        END
