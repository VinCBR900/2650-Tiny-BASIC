; ubasic_v0.1.asm  -  Tiny BASIC work-in-progress for Signetics 2650
; -----------------------------------------------------------------------
; TARGET:   Signetics 2650 @ 1 MHz
;           ROM $0000-$0FFF (4KB), RAM $1000-$17FF (2KB)
; SIM I/O:  WRTD,R1 = putchar,  REDE,R1 = getchar
;
; v0.2: adds immediate-mode statement dispatcher with:
;       - PRINT / PR   (quoted strings or tail-of-line text)
;       - REM   / RE   (comment/no-op)
;       - END   / EN   (halts CPU)
;       - syntax error reporting (?0)
;
; RAM MAP ($1000-$17FF):
;   $1000-$1001  SPTR    string walk pointer (2 bytes)
;   $1002-$1003  LPTR    line buffer write pointer (2 bytes)
;   $1004        LCNT    line buffer char count (1 byte)
;   $1005-$104F  IBUF    input line buffer (75 bytes + NUL)
;   $1050-$1051  IPTR    parse pointer into IBUF (2 bytes)
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
; Supports PR/PRINT, RE/REM, EN/END
; ════════════════════════════════════════════════════════════════
STMT_LINE:
        BSTA,UN WPEEK_UC            ; R1 = first non-space, uppercased
        COMI,R1 NUL
        BCTA,EQ ST_RET              ; blank line

        COMI,R1 'P'
        BCTA,EQ ST_PRINT
        COMI,R1 'R'
        BCTA,EQ ST_REM
        COMI,R1 'E'
        BCTA,EQ ST_END

        BCTA,UN SYNERR

ST_PRINT:
        BSTA,UN GETCI               ; consume first char
        BSTA,UN UC
        COMI,R1 'P'
        BCTA,EQ ST_PRINT_2
        BCTA,UN SYNERR
ST_PRINT_2:
        BSTA,UN GETCI               ; consume second char
        BSTA,UN UC
        COMI,R1 'R'
        BCTA,EQ ST_PRINT_OK
        BCTA,UN SYNERR
ST_PRINT_OK:
        BSTA,UN EATWORD             ; allow full PRINT
        BSTA,UN DO_PRINT
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
        BSTA,UN EATWORD             ; consume REM if present
        BCTA,UN ST_RET              ; ignore rest of line

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
        BSTA,UN EATWORD             ; consume END if present
        HALT

ST_RET:
        RETC,UN

; ════════════════════════════════════════════════════════════════
; DO_PRINT  — print either:
;   PRINT "string"
;   PRINT any text until end of line
; ════════════════════════════════════════════════════════════════
DO_PRINT:
        BSTA,UN WPEEK               ; R1 = first non-space (not uppercased)
        COMI,R1 DQ
        BCTA,EQ PR_QUOTED

        ; unquoted: print from current IPTR until NUL
        LODA,R0 IPTR
        STRA,R0 SPTR
        LODA,R0 IPTR+1
        STRA,R0 SPTR+1
        BSTA,UN PRTSTR
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
        BSTA,UN PEEKC               ; R1 = next char
        BSTA,UN UC                  ; uppercase copy in R1 if alpha
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
; UC  — uppercase conversion for ASCII in R1
;      'a'..'z' => 'A'..'Z', others unchanged
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
; PUTCH  —  output char in R1
; ════════════════════════════════════════════════════════════════
PUTCH:
        WRTD,R1
        RETC,UN

; ════════════════════════════════════════════════════════════════
; GETCH  —  input char into R1
; ════════════════════════════════════════════════════════════════
GETCH:
        REDE,R1
        RETC,UN

; ════════════════════════════════════════════════════════════════
; PRNL  —  print CR + LF
; ════════════════════════════════════════════════════════════════
PRNL:
        LODI,R1 CR
        BSTA,UN PUTCH
        LODI,R1 LF
        BSTA,UN PUTCH
        RETC,UN

; ════════════════════════════════════════════════════════════════
; PRTSTR  —  print NUL-terminated string
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
; RDLINE  —  read line from input into IBUF
; Reads until CR or LF. Stores with NUL terminator.
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
        DB      'u','B','A','S','I','C',' ','v','0','.','2'
        DB      CR,LF
        DB      '2','6','5','0',' ','p','o','r','t',' ','(','W','I','P',')'
        DB      CR,LF,NUL

        END
