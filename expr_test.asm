; =============================================================================
; expr_test.asm  v0.5
; Recursive descent parser + SW recursive PRINT_S16
;
; SW STACK CONVENTION:
;   R3 = index into SWSTK (starts $FF = empty)
;   STRA,R0 SWSTK,R3+   push: R3++ then store at SWSTK+R3
;   LODA,R0 SWSTK,R3-   pop:  R3-- then load from SWSTK+R3
;   SWJSR (10 bytes per call site):
;     LODI,R0 >RETADDR   ; lo of return address
;     STRA,R0 SWSTK,R3+  ; push lo
;     LODI,R0 <RETADDR   ; hi of return address
;     STRA,R0 SWSTK,R3+  ; push hi
;     BCTA,UN TARGET
;   RETADDR:
;   SWRETURN (shared, ~10 bytes):
;     pops hi then lo, indirect jump
;
; MUL16 uses PPSL $08 (WC=1) so RRL/RRR rotate through carry (9-bit).
;   CPSL $01 before each shift clears carry → logical shift (0 shifts in).
;
; PARSE_NUM: EXP*10 = EXP*8 + EXP*2 via shifts. ~20 bytes vs BDRR*10 loop.
;
; DIV16: restoring shift-subtract, 16 fixed iterations.
;   Dividend in TMPH:TMPL, quotient builds in EXPH:EXPL,
;   remainder builds in NEGFLG:SC1.
;
; PRINT_S16: SW recursive.
;   Sign+zero handled at top (HW call). Then SW-calls PREC.
;   PREC: divide EXP by 10 → quotient in EXP, remainder in SC1.
;         Push digit (SC1). If quotient!=0: SW-recurse PREC.
;         On return: pop digit, print it. SW-return.
;   R3 CONFLICT: PREC uses NEGFLG as loop counter (not R3).
;
; =============================================================================

COUT    EQU     $02B4
CRLF    EQU     $008A
CR      EQU     $0D
SP      EQU     $20
NUL     EQU     $00

; RAM
IPH     EQU     $1600
IPL     EQU     $1601
EXPH    EQU     $1602
EXPL    EQU     $1603
NEGFLG  EQU     $1604
SC0     EQU     $1605
SC1     EQU     $1606
TMPH    EQU     $1607
TMPL    EQU     $1608
RELOP   EQU     $1609
LEFTH   EQU     $160A
LEFTL   EQU     $160B
SAVEH   EQU     $160C
SAVEL   EQU     $160D
E1SAVH  EQU     $160E
E1SAVL  EQU     $160F
DIVH    EQU     $1610
DIVL    EQU     $1611
SWRETH  EQU     $1612
SWRETL  EQU     $1613
SWSTK   EQU     $1614   ; 16 bytes: $1614-$1623
VARS    EQU     $1630

        ORG     $0440

; =============================================================================
; MAIN — test harness
; R3 = SW stack pointer, initialised to $FF (empty)
; =============================================================================
MAIN:
        LODI,R3 $FF

        ; Test 1: 0
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 EXPL
        BSTA,UN PRINT_S16

        ; Test 2: 42
        EORZ,R0
        STRA,R0 EXPH
        LODI,R0 42
        STRA,R0 EXPL
        BSTA,UN PRINT_S16

        ; Test 3: -1
        LODI,R0 $FF
        STRA,R0 EXPH
        STRA,R0 EXPL
        BSTA,UN PRINT_S16

        ; Test 4: 32767
        LODI,R0 $7F
        STRA,R0 EXPH
        LODI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN PRINT_S16

        ; Test 5: -32768
        LODI,R0 $80
        STRA,R0 EXPH
        EORZ,R0
        STRA,R0 EXPL
        BSTA,UN PRINT_S16

        ; Test 6: "123" → 123
        LODI,R0 <TSTR1
        STRA,R0 IPH
        LODI,R0 >TSTR1
        STRA,R0 IPL
        BSTA,UN EXPR
        BSTA,UN PRINT_S16

        ; Test 7: "10+5*3" → 25
        LODI,R0 <TSTR2
        STRA,R0 IPH
        LODI,R0 >TSTR2
        STRA,R0 IPL
        BSTA,UN EXPR
        BSTA,UN PRINT_S16

        ; Test 8: "-7" → -7
        LODI,R0 <TSTR3
        STRA,R0 IPH
        LODI,R0 >TSTR3
        STRA,R0 IPL
        BSTA,UN EXPR
        BSTA,UN PRINT_S16

        ; Test 9: "100/4" → 25
        LODI,R0 <TSTR4
        STRA,R0 IPH
        LODI,R0 >TSTR4
        STRA,R0 IPL
        BSTA,UN EXPR
        BSTA,UN PRINT_S16

        ; Test 10: "17%5" → 2
        LODI,R0 <TSTR5
        STRA,R0 IPH
        LODI,R0 >TSTR5
        STRA,R0 IPL
        BSTA,UN EXPR
        BSTA,UN PRINT_S16

        ; Test 11: "3*-4" → -12
        LODI,R0 <TSTR6
        STRA,R0 IPH
        LODI,R0 >TSTR6
        STRA,R0 IPL
        BSTA,UN EXPR
        BSTA,UN PRINT_S16

        ; Test 12: "2+3=5" → -1 (true)
        LODI,R0 <TSTR7
        STRA,R0 IPH
        LODI,R0 >TSTR7
        STRA,R0 IPL
        BSTA,UN EXPR
        BSTA,UN PRINT_S16

        ; Test 13: "100*200" → 20000
        LODI,R0 <TSTR8
        STRA,R0 IPH
        LODI,R0 >TSTR8
        STRA,R0 IPL
        BSTA,UN EXPR
        BSTA,UN PRINT_S16

        HALT

TSTR1:  
        ; DB A'1',A'2',A'3',NUL
        DB "123"
        DB 0
TSTR2:  
        DB A'1',A'0',A'+',A'5',A'*',A'3',NUL
TSTR3:  
        DB A'-',A'7',NUL
TSTR4:  
        DB A'1',A'0',A'0',A'/',A'4',NUL
TSTR5:  
        DB A'1',A'7',A'%',A'5',NUL
TSTR6:  
        DB A'3',A'*',A'-',A'4',NUL
TSTR7:  
        DB A'2',A'+',A'3',A'=',A'5',NUL
TSTR8:  
        DB A'1',A'0',A'0',A'*',A'2',A'0',A'0',NUL

; =============================================================================
; PRINT_S16 — signed decimal print, SW recursive via PREC
; Entry: EXPH:EXPL = value, R3 = SW stack pointer
; =============================================================================
PRINT_S16:
        LODA,R0 EXPH
        ANDI,R0 $80
        BCTA,EQ PS_POS
        LODI,R0 A'-'
        BSTA,UN COUT
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
PS_POS:
        ; Zero check (unsigned — value is now abs, max $8000 treated as positive)
        ; Use COM=0 (default signed), but $8000 has bit7 set so COMI GT works
        LODA,R0 EXPH
        COMI,R0 $00
        BCTA,GT PS_NZ
        LODA,R0 EXPL
        COMI,R0 $00
        BCTA,GT PS_NZ
        LODI,R0 A'0'
        BSTA,UN COUT
        BSTA,UN CRLF
        RETC,UN
PS_NZ:
        ; SWJSR to PREC; return to PS_CRLF
        LODI,R0 <PS_CRLF        ; hi byte first
        STRA,R0 SWSTK,R3+
        LODI,R0 >PS_CRLF        ; lo byte second
        STRA,R0 SWSTK,R3+
        BCTA,UN PREC
PS_CRLF:
        BSTA,UN CRLF
        RETC,UN

; =============================================================================
; PREC — SW recursive digit printer
; Entry: EXPH:EXPL = positive value >=1, R3 = SW stack pointer
;
; 1. Divide EXPH:EXPL by 10 (inlined 16-bit restoring shift-subtract)
;    Quotient → EXPH:EXPL, remainder (0-9) → SC1
; 2. Push SC1 (digit) onto SW stack
; 3. If quotient != 0: SWJSR self
; 4. Pop digit, print (ASCII), SWRETURN
;
; R3 CONFLICT: division loop uses NEGFLG as counter (not R3)
; WC conflict: set WC=1 for shifts, clear after
; =============================================================================
PREC:
        ; Divide EXPH:EXPL by 10 — restoring shift-subtract, 16 iters
        ; Quotient builds in EXPH:EXPL (shift left each iter, set bit0 when rem>=div)
        ; Remainder builds in NEGFLG:SC1 (shift left, pull MSB from dividend)
        ; Dividend bits consumed from TMPH:TMPL — copy first
        LODA,R0 EXPH
        STRA,R0 TMPH
        LODA,R0 EXPL
        STRA,R0 TMPL
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 EXPL            ; quotient = 0
        STRA,R0 NEGFLG
        STRA,R0 SC1             ; remainder = 0
        LODI,R0 16
        STRA,R0 SC0             ; SC0 = loop counter (NEGFLG in use for remainder)
        PPSL $08                ; WC=1: RRL/RRR rotate through carry
PR_LP:
        ; Shift dividend TMPH:TMPL left, MSB goes to carry
        CPSL $01
        LODA,R0 TMPL
        RRL,R0
        STRA,R0 TMPL
        LODA,R0 TMPH
        RRL,R0                  ; old TMPH bit7 → carry
        STRA,R0 TMPH
        ; Shift remainder NEGFLG:SC1 left, carry (old dividend MSB) → bit0 of SC1
        LODA,R0 SC1
        RRL,R0
        STRA,R0 SC1
        LODA,R0 NEGFLG
        RRL,R0
        STRA,R0 NEGFLG
        ; Shift quotient EXPH:EXPL left
        CPSL $01
        LODA,R0 EXPL
        RRL,R0
        STRA,R0 EXPL
        LODA,R0 EXPH
        RRL,R0
        STRA,R0 EXPH
        ; If remainder (NEGFLG:SC1) >= 10: subtract 10, set quotient bit0
        ; divisor = 10 = $000A, so NEGFLG must be 0 and SC1 >= 10
        LODA,R0 NEGFLG
        COMI,R0 $00
        BCTA,GT PR_QBIT         ; rem_hi > 0 means rem >= 256 >= 10
        LODA,R0 SC1
        COMI,R0 10
        BCTA,LT PR_NOQBIT
PR_QBIT:
        ; remainder -= 10
        LODA,R0 SC1
        SUBI,R0 10
        STRA,R0 SC1
        TPSL $01
        BCTA,EQ PR_SNB          ; C=1: no borrow
        LODA,R0 NEGFLG
        SUBI,R0 1
        STRA,R0 NEGFLG
PR_SNB:
        LODA,R0 EXPL
        IORI,R0 $01
        STRA,R0 EXPL            ; quotient bit0 = 1
PR_NOQBIT:
        LODA,R0 SC0
        SUBI,R0 1
        STRA,R0 SC0
        COMI,R0 $00
        BCTA,GT PR_LP
        CPSL $08                ; clear WC
        ; SC1 = remainder digit (0-9)
        ; Push digit onto SW stack
        LODA,R0 SC1
        STRA,R0 SWSTK,R3+
        ; If quotient (EXPH:EXPL) != 0: SW-recurse
        LODA,R0 EXPH
        COMI,R0 $00
        BCTA,GT PR_REC
        LODA,R0 EXPL
        COMI,R0 $00
        BCTA,EQ PR_PRINT        ; quotient=0, base case
PR_REC:
        ; SWJSR self; return to PR_PRINT
        LODI,R0 <PR_PRINT       ; hi byte first
        STRA,R0 SWSTK,R3+
        LODI,R0 >PR_PRINT       ; lo byte second
        STRA,R0 SWSTK,R3+
        BCTA,UN PREC
PR_PRINT:
        ; Pop digit, print it
        LODA,R0 SWSTK,R3-
        ADDI,R0 A'0'
        BSTA,UN COUT
        BCTA,UN SWRETURN

; =============================================================================
; SWRETURN — pop 2-byte return address and jump to it
; =============================================================================
SWRETURN:
        LODA,R0 SWSTK,R3-       ; pop hi
        STRA,R0 SWRETH
        LODA,R0 SWSTK,R3-       ; pop lo
        STRA,R0 SWRETL
        BCTA,UN *SWRETH

; =============================================================================
; EXPR — relational level
; =============================================================================
EXPR:
        BSTA,UN EXPR_ADD
        LODA,R0 EXPH
        STRA,R0 LEFTH
        LODA,R0 EXPL
        STRA,R0 LEFTL
        EORZ,R1
EX_WS:
        LODA,R0 *IPH
        COMI,R0 SP
        BCTR,EQ EX_WSA
        BCTA,UN EX_CHK
EX_WSA:
        BSTA,UN INC_IP
        BCTA,UN EX_WS
EX_CHK:
        COMI,R0 A'<'
        BCTR,EQ EX_LT
        COMI,R0 A'='
        BCTR,EQ EX_EQ
        COMI,R0 A'>'
        BCTR,EQ EX_GT
        LODZ,R1
        COMI,R0 $00
        BCTA,EQ EX_NONE
        STRA,R0 RELOP
        BSTA,UN EXPR_ADD
        BCTA,UN DO_RELOP
EX_LT:
        IORI,R1 1
        BSTA,UN INC_IP
        LODA,R0 *IPH
        BCTA,UN EX_CHK
EX_EQ:
        IORI,R1 2
        BSTA,UN INC_IP
        LODA,R0 *IPH
        BCTA,UN EX_CHK
EX_GT:
        IORI,R1 4
        BSTA,UN INC_IP
        LODA,R0 *IPH
        BCTA,UN EX_CHK
EX_NONE:
        RETC,UN

; =============================================================================
; DO_RELOP — signed compare LEFTH:LEFTL vs EXPH:EXPL → $FFFF/$0000
; =============================================================================
DO_RELOP:
        LODA,R0 LEFTH
        EORI,R0 $80
        STRA,R0 SC0
        LODA,R0 EXPH
        EORI,R0 $80
        SUBA,R0 SC0
        BCTA,LT DR_LT
        BCTA,GT DR_GT
        LODA,R0 EXPL
        SUBA,R0 LEFTL
        BCTA,LT DR_LT
        BCTA,GT DR_GT
        LODI,R0 2
        BCTA,UN DR_TEST
DR_LT:
        LODI,R0 1
        BCTA,UN DR_TEST
DR_GT:
        LODI,R0 4
DR_TEST:
        TMI,R0 RELOP
        BCTR,EQ DR_TRUE
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 EXPL
        RETC,UN
DR_TRUE:
        LODI,R0 $FF
        STRA,R0 EXPH
        STRA,R0 EXPL
        RETC,UN

; =============================================================================
; EXPR_ADD — additive level, left save in SAVEH:SAVEL
; =============================================================================
EXPR_ADD:
        BSTA,UN EXPR1
EA_LP:
EA_WS:
        LODA,R0 *IPH
        COMI,R0 SP
        BCTR,EQ EA_WSA
        BCTA,UN EA_CHK
EA_WSA:
        BSTA,UN INC_IP
        BCTA,UN EA_WS
EA_CHK:
        COMI,R0 A'+'
        BCTR,EQ EA_ADD
        COMI,R0 A'-'
        BCTR,EQ EA_SUB
        RETC,UN
EA_ADD:
        LODA,R0 EXPH
        STRA,R0 SAVEH
        LODA,R0 EXPL
        STRA,R0 SAVEL
        BSTA,UN INC_IP
        BSTA,UN EXPR1
        LODA,R0 SAVEL
        ADDA,R0 EXPL
        STRA,R0 EXPL
        TPSL $01
        BCTA,LT EA_ANC
        LODA,R0 SAVEH
        ADDI,R0 1
        BCTA,UN EA_AHI
EA_ANC:
        LODA,R0 SAVEH
EA_AHI:
        ADDA,R0 EXPH
        STRA,R0 EXPH
        BCTA,UN EA_LP
EA_SUB:
        LODA,R0 EXPH
        STRA,R0 SAVEH
        LODA,R0 EXPL
        STRA,R0 SAVEL
        BSTA,UN INC_IP
        BSTA,UN EXPR1
        LODA,R0 SAVEL
        SUBA,R0 EXPL
        STRA,R0 EXPL
        TPSL $01
        BCTA,EQ EA_SNB          ; C=1: no borrow
        LODA,R0 SAVEH
        SUBI,R0 1
        BCTA,UN EA_SHI
EA_SNB:
        LODA,R0 SAVEH
EA_SHI:
        SUBA,R0 EXPH
        STRA,R0 EXPH
        BCTA,UN EA_LP

; =============================================================================
; EXPR1 — multiplicative level, left save in E1SAVH:E1SAVL
; =============================================================================
EXPR1:
        BSTA,UN EXPR2
E1_LP:
E1_WS:
        LODA,R0 *IPH
        COMI,R0 SP
        BCTR,EQ E1_WSA
        BCTA,UN E1_CHK
E1_WSA:
        BSTA,UN INC_IP
        BCTA,UN E1_WS
E1_CHK:
        COMI,R0 A'*'
        BCTA,EQ E1_MUL
        COMI,R0 A'/'
        BCTA,EQ E1_DIV
        COMI,R0 A'%'
        BCTA,EQ E1_MOD
        RETC,UN
E1_MUL:
        LODA,R0 EXPH
        STRA,R0 E1SAVH
        LODA,R0 EXPL
        STRA,R0 E1SAVL
        BSTA,UN INC_IP
        BSTA,UN EXPR2
        BSTA,UN MUL16
        BCTA,UN E1_LP
E1_DIV:
        LODA,R0 EXPH
        STRA,R0 E1SAVH
        LODA,R0 EXPL
        STRA,R0 E1SAVL
        BSTA,UN INC_IP
        BSTA,UN EXPR2
        LODA,R0 EXPH
        STRA,R0 DIVH
        LODA,R0 EXPL
        STRA,R0 DIVL
        LODA,R0 E1SAVH
        STRA,R0 EXPH
        LODA,R0 E1SAVL
        STRA,R0 EXPL
        BSTA,UN DIV16
        BCTA,UN E1_LP
E1_MOD:
        LODA,R0 EXPH
        STRA,R0 E1SAVH
        LODA,R0 EXPL
        STRA,R0 E1SAVL
        BSTA,UN INC_IP
        BSTA,UN EXPR2
        LODA,R0 EXPH
        STRA,R0 DIVH
        LODA,R0 EXPL
        STRA,R0 DIVL
        LODA,R0 E1SAVH
        STRA,R0 EXPH
        LODA,R0 E1SAVL
        STRA,R0 EXPL
        BSTA,UN DIV16
        LODA,R0 TMPH
        STRA,R0 EXPH
        LODA,R0 TMPL
        STRA,R0 EXPL
        BCTA,UN E1_LP

; =============================================================================
; EXPR2 — atom: number, variable, unary -, paren
; =============================================================================
EXPR2:
E2_WS:
        LODA,R0 *IPH
        COMI,R0 SP
        BCTR,EQ E2_WSA
        BCTA,UN E2_CHK
E2_WSA:
        BSTA,UN INC_IP
        BCTA,UN E2_WS
E2_CHK:
        COMI,R0 A'('
        BCTA,EQ E2_PAR
        COMI,R0 A'-'
        BCTA,EQ E2_NEG
        COMI,R0 A'+'
        BCTA,EQ E2_POS
        COMI,R0 A'A'
        BCTA,LT E2_LCCHK
        COMI,R0 A'Z'+1
        BCTA,LT E2_VAR
E2_LCCHK:
        COMI,R0 A'a'
        BCTA,LT E2_NUM
        COMI,R0 A'z'+1
        BCTA,LT E2_VARUC
E2_NUM:
        BSTA,UN PARSE_NUM
        RETC,UN
E2_VAR:
        STRA,R0 SC0
        BSTA,UN INC_IP
        LODA,R0 SC0
        SUBI,R0 A'A'
        STRA,R0 SC1
        ADDA,R0 SC1
        ADDI,R0 >VARS
        STRA,R0 TMPL
        LODI,R0 <VARS
        TPSL $01
        BCTA,LT E2_VNC
        ADDI,R0 1
E2_VNC:
        STRA,R0 TMPH
        LODA,R0 *TMPH
        STRA,R0 EXPH
        BSTA,UN INC_TMP
        LODA,R0 *TMPH
        STRA,R0 EXPL
        RETC,UN
E2_VARUC:
        SUBI,R0 32
        BCTA,UN E2_VAR
E2_PAR:
        BSTA,UN INC_IP
        BSTA,UN EXPR
E2_PARS:
        LODA,R0 *IPH
        COMI,R0 SP
        BCTR,EQ E2_PARSA
        BCTA,UN E2_PARD
E2_PARSA:
        BSTA,UN INC_IP
        BCTA,UN E2_PARS
E2_PARD:
        BSTA,UN INC_IP          ; consume ')'
        RETC,UN
E2_NEG:
        BSTA,UN INC_IP
        BSTA,UN EXPR2
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
        RETC,UN
E2_POS:
        BSTA,UN INC_IP
        BCTA,UN EXPR2

; =============================================================================
; PARSE_NUM — parse unsigned decimal at IP → EXPH:EXPL
; EXP*10 = EXP*8 + EXP*2 via shifts
; =============================================================================
PARSE_NUM:
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 EXPL
PN_LP:
        LODA,R0 *IPH
        COMI,R0 A'0'
        RETC,LT
        COMI,R0 A'9'+1
        RETC,GT
        SUBI,R0 A'0'
        STRA,R0 SC0             ; digit
        BSTA,UN INC_IP
        ; EXP*2
        PPSL $08                ; WC=1 for logical shift
        CPSL $01
        LODA,R0 EXPL
        RRL,R0
        STRA,R0 EXPL
        LODA,R0 EXPH
        RRL,R0
        STRA,R0 EXPH
        ; save EXP*2 in SC1:NEGFLG
        STRA,R0 NEGFLG
        LODA,R0 EXPL
        STRA,R0 SC1
        ; EXP*2 → *4
        CPSL $01
        LODA,R0 EXPL
        RRL,R0
        STRA,R0 EXPL
        LODA,R0 EXPH
        RRL,R0
        STRA,R0 EXPH
        ; EXP*4 → *8
        CPSL $01
        LODA,R0 EXPL
        RRL,R0
        STRA,R0 EXPL
        LODA,R0 EXPH
        RRL,R0
        STRA,R0 EXPH
        CPSL $08                ; clear WC
        ; EXP*8 + EXP*2
        LODA,R0 EXPL
        ADDA,R0 SC1
        STRA,R0 EXPL
        TPSL $01
        BCTA,LT PN_MNC
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
PN_MNC:
        LODA,R0 EXPH
        ADDA,R0 NEGFLG
        STRA,R0 EXPH
        ; + digit
        LODA,R0 EXPL
        ADDA,R0 SC0
        STRA,R0 EXPL
        TPSL $01
        BCTA,LT PN_DNC
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
PN_DNC:
        BCTA,UN PN_LP

; =============================================================================
; MUL16 — signed E1SAVH:E1SAVL * EXPH:EXPL → EXPH:EXPL
; Shift-and-add, 16 fixed iterations.
; Multiplier  (shift right): E1SAVH:E1SAVL
; Multiplicand (shift left):  EXPH:EXPL
; Accumulator: TMPH:TMPL
; Sign: SC0 (0=pos, 1=neg)
; Uses BDRR,R3 for loop — R3 safe here (not in SW context)
; =============================================================================
MUL16:
        EORZ,R0
        STRA,R0 SC0
        ; abs(left) E1SAVH:E1SAVL
        LODA,R0 E1SAVH
        ANDI,R0 $80
        BCTA,EQ MU_LA
        LODA,R0 E1SAVH
        EORI,R0 $FF
        STRA,R0 E1SAVH
        LODA,R0 E1SAVL
        EORI,R0 $FF
        STRA,R0 E1SAVL
        BSTA,UN INC_E1
        LODI,R0 1
        STRA,R0 SC0
MU_LA:
        ; abs(right) EXPH:EXPL
        LODA,R0 EXPH
        ANDI,R0 $80
        BCTA,EQ MU_RA
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
        LODA,R0 SC0
        EORI,R0 $01
        STRA,R0 SC0
MU_RA:
        EORZ,R0
        STRA,R0 TMPH
        STRA,R0 TMPL
        PPSL $08                ; WC=1: RRL/RRR rotate through carry
        LODI,R3 16
MU_LP:
        LODA,R0 E1SAVL
        ANDI,R0 $01
        BCTA,EQ MU_NOADD
        LODA,R0 TMPL
        ADDA,R0 EXPL
        STRA,R0 TMPL
        TPSL $01
        BCTA,LT MU_ANC
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
MU_ANC:
        LODA,R0 TMPH
        ADDA,R0 EXPH
        STRA,R0 TMPH
MU_NOADD:
        ; Logical shift E1SAVH:E1SAVL right (MSB=0)
        CPSL $01
        LODA,R0 E1SAVH
        RRR,R0
        STRA,R0 E1SAVH
        LODA,R0 E1SAVL
        RRR,R0
        STRA,R0 E1SAVL
        ; Logical shift EXPH:EXPL left (LSB=0)
        CPSL $01
        LODA,R0 EXPL
        RRL,R0
        STRA,R0 EXPL
        LODA,R0 EXPH
        RRL,R0
        STRA,R0 EXPH
        BDRA,R3 MU_LP
        CPSL $08                ; clear WC
        LODA,R0 TMPH
        STRA,R0 EXPH
        LODA,R0 TMPL
        STRA,R0 EXPL
        LODA,R0 SC0
        COMI,R0 $00
        RETC,EQ
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
        RETC,UN

; =============================================================================
; DIV16 — signed EXPH:EXPL / DIVH:DIVL → EXPH:EXPL (quotient)
;          remainder → TMPH:TMPL
; Restoring shift-subtract, 16 fixed iterations.
; Dividend: TMPH:TMPL (moved there from EXPH:EXPL after abs)
; Quotient:  EXPH:EXPL (built here)
; Remainder: NEGFLG:SC1 (partial remainder)
; Sign: SC0
; Uses BDRR,R3 — safe (not in SW context)
; =============================================================================
DIV16:
        LODA,R0 DIVH
        COMI,R0 $00
        BCTA,GT DV_NZ
        LODA,R0 DIVL
        COMI,R0 $00
        BCTA,EQ DV_ZERO
DV_NZ:
        EORZ,R0
        STRA,R0 SC0
        ; abs(dividend) EXPH:EXPL
        LODA,R0 EXPH
        ANDI,R0 $80
        BCTA,EQ DV_DA
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
        LODI,R0 1
        STRA,R0 SC0
DV_DA:
        ; abs(divisor) DIVH:DIVL
        LODA,R0 DIVH
        ANDI,R0 $80
        BCTA,EQ DV_VA
        LODA,R0 DIVH
        EORI,R0 $FF
        STRA,R0 DIVH
        LODA,R0 DIVL
        EORI,R0 $FF
        STRA,R0 DIVL
        BSTA,UN INC_DIV
        LODA,R0 SC0
        EORI,R0 $01
        STRA,R0 SC0
DV_VA:
        ; Move dividend to TMPH:TMPL; quotient starts 0 in EXPH:EXPL
        LODA,R0 EXPH
        STRA,R0 TMPH
        LODA,R0 EXPL
        STRA,R0 TMPL
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 EXPL
        STRA,R0 NEGFLG
        STRA,R0 SC1             ; remainder = 0
        PPSL $08                ; WC=1
        LODI,R3 16
DV_LP:
        ; Shift dividend TMPH:TMPL left; MSB goes to carry
        CPSL $01
        LODA,R0 TMPL
        RRL,R0
        STRA,R0 TMPL
        LODA,R0 TMPH
        RRL,R0                  ; carry = old bit7 of TMPH
        STRA,R0 TMPH
        ; Shift remainder NEGFLG:SC1 left; carry (old dividend MSB) → bit0 of SC1
        LODA,R0 SC1
        RRL,R0
        STRA,R0 SC1
        LODA,R0 NEGFLG
        RRL,R0
        STRA,R0 NEGFLG
        ; Shift quotient EXPH:EXPL left
        CPSL $01
        LODA,R0 EXPL
        RRL,R0
        STRA,R0 EXPL
        LODA,R0 EXPH
        RRL,R0
        STRA,R0 EXPH
        ; If remainder >= divisor: subtract, set quotient bit0
        LODA,R0 NEGFLG
        SUBA,R0 DIVH
        BCTA,LT DV_NOQBIT
        BCTA,GT DV_QBIT
        LODA,R0 SC1
        SUBA,R0 DIVL
        BCTA,LT DV_NOQBIT
DV_QBIT:
        LODA,R0 SC1
        SUBA,R0 DIVL
        STRA,R0 SC1
        TPSL $01
        BCTA,EQ DV_SNB          ; C=1: no borrow
        LODA,R0 NEGFLG
        SUBI,R0 1
        STRA,R0 NEGFLG
DV_SNB:
        LODA,R0 NEGFLG
        SUBA,R0 DIVH
        STRA,R0 NEGFLG
        LODA,R0 EXPL
        IORI,R0 $01
        STRA,R0 EXPL
DV_NOQBIT:
        BDRA,R3 DV_LP
        CPSL $08                ; clear WC
        ; Remainder NEGFLG:SC1 → TMPH:TMPL
        LODA,R0 NEGFLG
        STRA,R0 TMPH
        LODA,R0 SC1
        STRA,R0 TMPL
        ; Apply sign
        LODA,R0 SC0
        COMI,R0 $00
        RETC,EQ
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
        RETC,UN
DV_ZERO:
        LODI,R0 A'?'
        BSTA,UN COUT
        HALT

; =============================================================================
; INC_EXP, INC_E1, INC_DIV, INC_IP, INC_TMP
; =============================================================================
INC_EXP:
        LODA,R0 EXPL
        ADDI,R0 1
        STRA,R0 EXPL
        TPSL $01
        RETC,LT
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
        RETC,UN

INC_E1:
        LODA,R0 E1SAVL
        ADDI,R0 1
        STRA,R0 E1SAVL
        TPSL $01
        RETC,LT
        LODA,R0 E1SAVH
        ADDI,R0 1
        STRA,R0 E1SAVH
        RETC,UN

INC_DIV:
        LODA,R0 DIVL
        ADDI,R0 1
        STRA,R0 DIVL
        TPSL $01
        RETC,LT
        LODA,R0 DIVH
        ADDI,R0 1
        STRA,R0 DIVH
        RETC,UN

INC_IP:
        LODA,R0 IPL
        ADDI,R0 1
        STRA,R0 IPL
        TPSL $01
        RETC,LT
        LODA,R0 IPH
        ADDI,R0 1
        STRA,R0 IPH
        RETC,UN

INC_TMP:
        LODA,R0 TMPL
        ADDI,R0 1
        STRA,R0 TMPL
        TPSL $01
        RETC,LT
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
        RETC,UN

        END
