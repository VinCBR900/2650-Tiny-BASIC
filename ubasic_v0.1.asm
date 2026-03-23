; ubasic_v0.1.asm  -  Tiny BASIC skeleton for Signetics 2650
; -----------------------------------------------------------------------
; TARGET:   Signetics 2650 @ 1 MHz
;           ROM $0000-$0FFF (4KB), RAM $1000-$17FF (2KB)
; SIM I/O:  WRTD,R1 = putchar,  REDE,R1 = getchar  (Kowalski-style)
; HW I/O:   bit-bang PUTCH/GETCH via FLAG/SENSE (swap-in later)
;
; v0.1: skeleton — RAM map, I/O primitives, PRTSTR, RDLINE, REPL stub.
;       Prints banner, reads a line (with backspace), echoes it.
;       Confirms the full toolchain works end-to-end.
;       No expression parsing yet.
;
; POINTER CONVENTION:
;   2650 indirect addressing: LODA,R1 *PTRL reads address from
;   mem[PTRL] (high 7 bits) and mem[PTRL+1] (low 8 bits), then
;   loads mem[that address] into R1.
;   We keep 2-byte (big-endian 15-bit) pointers in RAM.
;
; RAM MAP ($1000-$17FF):
;   $1000-$1001  SPTR    string walk pointer (2 bytes, big-endian)
;   $1002-$1003  LPTR    line buffer write pointer (2 bytes)
;   $1004        LCNT    line buffer char count (1 byte)
;   $1005-$104F  IBUF    input line buffer (75 bytes + NUL)
;   $1050-$1051  IPTR    parse pointer into IBUF (2 bytes)
;   $1052-$1083  VARS    A-Z variables, 2 bytes each (52 bytes)
;   $1084        PROG    BASIC program storage (grows up)
; -----------------------------------------------------------------------

CR      EQU     $0D
LF      EQU     $0A
BS      EQU     $08
SP      EQU     $20
NUL     EQU     $00
IBUFSZ  EQU     75

SPTR    EQU     $1000       ; 2-byte string pointer
LPTR    EQU     $1002       ; 2-byte line write pointer
LCNT    EQU     $1004       ; line char count
IBUF    EQU     $1005       ; input buffer

        ORG     $0000

; ════════════════════════════════════════════════════════════════
; RESET
; ════════════════════════════════════════════════════════════════
RESET:
        ; set up SPTR → BANNER and print it
        LODI,R0 >BANNER     ; high byte of BANNER address
        STRA,R0 SPTR
        LODI,R0 <BANNER     ; low byte
        STRA,R0 SPTR+1
        BSTA,UN PRTSTR

; ════════════════════════════════════════════════════════════════
; REPL  —  prompt, read line, echo, repeat
; ════════════════════════════════════════════════════════════════
REPL:
        LODI,R1 '?'
        BSTA,UN PUTCH
        LODI,R1 SP
        BSTA,UN PUTCH

        BSTA,UN RDLINE      ; fills IBUF, NUL-terminated

        ; echo with "> " prefix
        LODI,R1 '>'
        BSTA,UN PUTCH
        LODI,R1 SP
        BSTA,UN PUTCH

        ; set SPTR → IBUF and print it
        LODI,R0 >IBUF
        STRA,R0 SPTR
        LODI,R0 <IBUF
        STRA,R0 SPTR+1
        BSTA,UN PRTSTR
        BSTA,UN PRNL

        BCTA,UN REPL

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
; Entry: SPTR (RAM $1000-$1001) = address of string
; Uses: R0, R1; preserves R2, R3
; Method: LODA,R1 *SPTR (indirect), check NUL, print, increment SPTR+1,
;         handle carry into SPTR when low byte wraps.
; ════════════════════════════════════════════════════════════════
PRTSTR:
PRTSTR_LP:
        LODA,R1 *SPTR       ; R1 = mem[mem[SPTR]:mem[SPTR+1]]
        COMI,R1 NUL         ; NUL terminator?
        BCTA,EQ PRTSTR_RET  ; yes → done
        BSTA,UN PUTCH       ; print char
        ; increment SPTR (16-bit pointer: lo byte at SPTR+1)
        LODA,R0 SPTR+1      ; R0 = lo byte
        ADDI,R0 1           ; R0++
        STRA,R0 SPTR+1      ; store back
        BCTA,GT PRTSTR_LP   ; no carry (GT = positive, i.e. non-zero result) → loop
        ; carry: lo byte wrapped ($FF→$00), increment hi byte
        LODA,R0 SPTR
        ADDI,R0 1
        STRA,R0 SPTR
        BCTA,UN PRTSTR_LP
PRTSTR_RET:
        RETC,UN

; ════════════════════════════════════════════════════════════════
; RDLINE  —  read line from input into IBUF
; Reads until CR or LF. Stores with NUL terminator.
; Backspace: erase last char (terminal echo: BS SP BS).
; Entry: (none)
; Exit:  IBUF = null-terminated input line, CR stripped
;        LCNT = number of chars stored
;        R0/R1/R2 clobbered
; ════════════════════════════════════════════════════════════════
RDLINE:
        ; initialise write pointer LPTR → IBUF
        LODI,R0 >IBUF
        STRA,R0 LPTR
        LODI,R0 <IBUF
        STRA,R0 LPTR+1
        ; clear char count
        LODI,R0 $00
        STRA,R0 LCNT

RDLINE_LP:
        BSTA,UN GETCH       ; R1 = char

        ; end of line?
        COMI,R1 CR
        BCTA,EQ RDLINE_EOL
        COMI,R1 LF
        BCTA,EQ RDLINE_EOL

        ; backspace?
        COMI,R1 BS
        BCTA,EQ RDLINE_BS

        ; buffer full? (LCNT >= IBUFSZ)
        LODA,R0 LCNT
        COMI,R0 IBUFSZ
        BCTA,EQ RDLINE_LP   ; equal → full, discard
        BCTA,GT RDLINE_LP   ; GT → overful (shouldn't happen), discard

        ; store char via LPTR (indirect)
        STRA,R1 *LPTR       ; mem[LPTR] = R1
        ; echo
        BSTA,UN PUTCH
        ; increment LPTR
        LODA,R0 LPTR+1
        ADDI,R0 1
        STRA,R0 LPTR+1
        BCTA,GT RDLINE_NCARRY
        LODA,R0 LPTR
        ADDI,R0 1
        STRA,R0 LPTR
RDLINE_NCARRY:
        ; increment LCNT
        LODA,R0 LCNT
        ADDI,R0 1
        STRA,R0 LCNT
        BCTA,UN RDLINE_LP

RDLINE_BS:
        LODA,R0 LCNT
        COMI,R0 $00         ; buffer empty?
        BCTA,EQ RDLINE_LP   ; yes → ignore
        ; decrement LPTR
        LODA,R0 LPTR+1
        SUBI,R0 1
        STRA,R0 LPTR+1
        ; check borrow (C=0 means borrow on 2650 subtract)
        ; After SUBI: if result went negative (wrapped), decrement hi byte
        ; Use CC: if R0 went from $00 to $FF, CC=LT (negative as signed)
        ; Actually simpler: just check if original was $00
        ; Re-examine: ADDI,R0 1 sets C if result wrapped 255→0
        ; SUBI,R0 1 sets C if no borrow (result >= 0), clears C if borrow
        ; So after SUBI: C=0 → borrow → decrement hi byte
        BCTA,GT RDLINE_BS_NCARRY  ; GT=positive → no borrow
        LODA,R0 LPTR
        SUBI,R0 1
        STRA,R0 LPTR
RDLINE_BS_NCARRY:
        ; decrement count
        LODA,R0 LCNT
        SUBI,R0 1
        STRA,R0 LCNT
        ; erase terminal: BS SP BS
        LODI,R1 BS
        BSTA,UN PUTCH
        LODI,R1 SP
        BSTA,UN PUTCH
        LODI,R1 BS
        BSTA,UN PUTCH
        BCTA,UN RDLINE_LP

RDLINE_EOL:
        ; store NUL at current LPTR
        LODI,R1 NUL
        STRA,R1 *LPTR
        BSTA,UN PRNL        ; echo newline
        RETC,UN

; ════════════════════════════════════════════════════════════════
; STRING CONSTANTS
; ════════════════════════════════════════════════════════════════
BANNER:
        DB      CR,LF
        DB      'u','B','A','S','I','C',' ','v','0','.','1'
        DB      CR,LF
        DB      'S','i','g','n','e','t','i','c','s',' ','2','6','5','0'
        DB      CR,LF,NUL

        END
