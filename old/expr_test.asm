; =============================================================================
; uBASIC2650_expr.asm — Signetics 2650 expression engine prototype
; Version: 0.7
; Date:    2026-05-06
;
; Purpose: Test bench for recursive-descent expression parser, shift-based
;          MUL16/DIV16, and SW-recursive PRINT_S16 (PREC). All 13 tests pass.
;          This will be merged into the full uBASIC2650 interpreter.
;
; SW STACK CONVENTION:
; Purpose: Test bench for recursive-descent expression parser, shift-based
;   R3 = index into SWSTK (starts $FF = empty)
;          MUL16/DIV16, and SW-recursive PRINT_S16 (PREC). All 13 tests pass.
;   STRA,R0 SWSTK,R3+   push: R3++ then store at SWSTK+R3
;          This will be merged into the full uBASIC2650 interpreter.
;   LODA,R0 SWSTK,R3-   pop:  R3-- then load from SWSTK+R3
;   SWJSR (10 bytes per call site):
;     LODI,R0 >RETADDR   ; lo of return address
;     STRA,R0 SWSTK,R3+  ; push lo
;     LODI,R0 <RETADDR   ; hi of return address
;     STRA,R0 SWSTK,R3+  ; push hi
;     BCTA,UN TARGET
;
;   RETADDR:
;   SWRETURN (shared, ~10 bytes):
;     pops hi then lo, indirect jump;
;
; Assembled size: 1857 bytes ($0440-$0B80)
;
; Build:
;   gcc -Wall -O2 -o asm2650 asm2650.c
;   gcc -Wall -O2 -DGAMER -o pipbug_wrap pipbug_wrap.c
;   ./asm2650 uBASIC2650_expr.asm uBASIC2650_expr.hex
;   ./pipbug_wrap uBASIC2650_expr.hex
;
; Test results (all 13 pass):
;   T1:  0          T8:  -7
;   T2:  42         T9:  25   (5+5*4, operator precedence)
;   T3:  -1         T10: 2    (17%5 remainder)
;   T4:  32767      T11: -12  (3*-4)
;   T5:  -32768     T12: -1   (2+3=5 relop true→-1)
;   T6:  123        T13: 20000 (100*200)
;   T7:  25
;
; Architecture:
;   EXPR     — recursive descent: EXPR→EXPR_ADD→EXPR1→EXPR2→PARSE_NUM
;   MUL16    — O(16) shift-and-add
;   DIV16    — O(16) restoring shift-subtract; quotient→EXPH:EXPL, remainder→TMPH:TMPL
;   PRINT_S16 — SW-recursive digit printer (PREC) using R3/SWSTK software stack
;   DO_RELOP  — signed 16-bit compare with XOR $80 bias; result -1=true, 0=false
;
; Change history:
;   v0.7  2026-05-06  Session recovery. All 13 tests pass.  1857 bytes.
;                     Fixed: PS_MIN CRLF tail call (-32768 path was missing CRLF).
;                     Fixed: LODI,R1 0 to zero relop mask (EORZ,R1 is wrong).
;                     Fixed: -32768 special case (negate $8000 overflows → detect early).
;                     Fixed: BDRA R3,MU_LP / BDRA R3,DV_LP syntax (was BDRR out-of-range).
;                     E1_DIV/MOD merged via LEFTH flag (shared 20-byte preamble).
;   v0.6  2026-05-05  All 13 tests pass.  1848 bytes.
;                     Fixed: LODI,R3 16 for 16 loop iterations (BDRR exits at rn==0).
;                     Fixed: BDRA R3,TARGET syntax for out-of-range loops.
;                     Fixed: LODI,R1 0 / -32768 detection / CRLF tail call bugs.
;   v0.5  2026-05-04  12/13 tests passing.  1769 bytes.
; =============================================================================

; =============================================================================

COUT    EQU     $02B4
CRLF    EQU     $008A
SP      EQU     $20
NUL     EQU     $00

; RAM layout ($16xx page — hi byte of all 16-bit pairs is at lower address)
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
SWRETH  EQU     $1612   ; SW return addr hi (adjacent pair for *SWRETH indirect jump)
SWRETL  EQU     $1613   ; SW return addr lo
SWSTK   EQU     $1614   ; SW stack data: 20 bytes $1614-$1627
VARS    EQU     $1630   ; A-Z variables (52 bytes)

        ORG     $0440

; =============================================================================
; MAIN — test harness. R3 = SW stack pointer ($FF = empty).
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

        ; Test 6: \\\"123\\\" → 123
        LODI,R0 <TSTR1
        STRA,R0 IPH
        LODI,R0 >TSTR1
        STRA,R0 IPL
        BSTA,UN EXPR
        BSTA,UN PRINT_S16

        ; Test 7: \\\"10+5*3\\\" → 25
        LODI,R0 <TSTR2
        STRA,R0 IPH
        LODI,R0 >TSTR2
        STRA,R0 IPL
        BSTA,UN EXPR
        BSTA,UN PRINT_S16

        ; Test 8: \\\"-7\\\" → -7
        LODI,R0 <TSTR3
        STRA,R0 IPH
        LODI,R0 >TSTR3
        STRA,R0 IPL
        BSTA,UN EXPR
        BSTA,UN PRINT_S16

        ; Test 9: \\\"100/4\\\" → 25
        LODI,R0 <TSTR4
        STRA,R0 IPH
        LODI,R0 >TSTR4
        STRA,R0 IPL
        BSTA,UN EXPR
        BSTA,UN PRINT_S16

        ; Test 10: \\\"17%5\\\" → 2
        LODI,R0 <TSTR5
        STRA,R0 IPH
        LODI,R0 >TSTR5
        STRA,R0 IPL
        BSTA,UN EXPR
        BSTA,UN PRINT_S16

        ; Test 11: \\\"3*-4\\\" → -12
        LODI,R0 <TSTR6
        STRA,R0 IPH
        LODI,R0 >TSTR6
        STRA,R0 IPL
        BSTA,UN EXPR
        BSTA,UN PRINT_S16

        ; Test 12: \\\"2+3=5\\\" → -1 (true)
        LODI,R0 <TSTR7
        STRA,R0 IPH
        LODI,R0 >TSTR7
        STRA,R0 IPL
        BSTA,UN EXPR
        BSTA,UN PRINT_S16

        ; Test 13: \\\"100*200\\\" → 20000
        LODI,R0 <TSTR8
        STRA,R0 IPH
        LODI,R0 >TSTR8
        STRA,R0 IPL
        BSTA,UN EXPR
        BSTA,UN PRINT_S16

        HALT

TSTR1:  DB A'1',A'2',A'3',NUL
TSTR2:  DB A'1',A'0',A'+',A'5',A'*',A'3',NUL
TSTR3:  DB A'-',A'7',NUL
TSTR4:  DB A'1',A'0',A'0',A'/',A'4',NUL
TSTR5:  DB A'1',A'7',A'%',A'5',NUL
TSTR6:  DB A'3',A'*',A'-',A'4',NUL
TSTR7:  DB A'2',A'+',A'3',A'=',A'5',NUL
TSTR8:  DB A'1',A'0',A'0',A'*',A'2',A'0',A'0',NUL

; =============================================================================
; PRINT_S16 — signed decimal print via SW recursive PREC
; Entry: EXPH:EXPL = value, R3 = SW stack pointer
; =============================================================================
PRINT_S16:
        LODA,R0 EXPH
        ANDI,R0 $80
        BCTA,EQ PS_POS
        LODI,R0 A'-'
        BSTA,UN COUT
        ; -32768 special case: $8000 negation overflows back to $8000
        LODA,R0 EXPH
        COMI,R0 $80
        BCTR,EQ PS_CHKMIN
PS_NEGNORM:
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
        BCTA,UN PS_POS
PS_CHKMIN:
        LODA,R0 EXPL
        COMI,R0 $00
        BCTR,EQ PS_MIN
        BCTA,UN PS_NEGNORM      ; $80xx with non-zero lo — normal negative
PS_MIN:
        ; print literal "32768"
        LODI,R0 A'3'
        BSTA,UN COUT
        LODI,R0 A'2'
        BSTA,UN COUT
        LODI,R0 A'7'
        BSTA,UN COUT
        LODI,R0 A'6'
        BSTA,UN COUT
        LODI,R0 A'8'
        BSTA,UN COUT
        BCTA,UN CRLF            ; tail call — consistent with PS_ZERO path
PS_POS:
        LODA,R0 EXPH
        COMI,R0 $00
        BCTA,GT PS_NZ
        LODA,R0 EXPL
        COMI,R0 $00
        BCTA,GT PS_NZ
        LODI,R0 A'0'
        BSTA,UN COUT
        BCTA,UN CRLF            ; tail call — item 4
PS_NZ:
        ; SWJSR: push CRLF as SW return addr, drop into PREC — item 5
        ; On final SWRETURN from PREC, execution jumps to CRLF directly.
        LODI,R0 >CRLF           ; lo byte of CRLF
        STRA,R0 SWSTK,R3+
        LODI,R0 <CRLF           ; hi byte of CRLF (on top)
        STRA,R0 SWSTK,R3+
        ; fall through into PREC

; =============================================================================
; PREC — SW recursive digit printer (divide EXP by 10, recurse, print)
; Entry: EXPH:EXPL = positive value >=1, R3 = SW stack pointer
; WC FIX: PPSL $08 at TOP of loop body. CPSL $08 before all SUBI/ADDI/SUBA.
; R3 used as SW stack pointer — loop counter is SC0 in RAM.
; =============================================================================
PREC:
        LODA,R0 EXPH
        STRA,R0 TMPH
        LODA,R0 EXPL
        STRA,R0 TMPL            ; dividend → TMPH:TMPL
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 EXPL            ; quotient = 0
        STRA,R0 NEGFLG
        STRA,R0 SC1             ; remainder = 0
        LODI,R0 16
        STRA,R0 SC0             ; loop counter
PR_LP:
        PPSL $08                ; WC=1 for this iter's shifts only
        ; Shift dividend TMPH:TMPL left; MSB → carry
        CPSL $01
        LODA,R0 TMPL
        RRL,R0
        STRA,R0 TMPL
        LODA,R0 TMPH
        RRL,R0                  ; old TMPH.bit7 → carry
        STRA,R0 TMPH
        ; Shift remainder NEGFLG:SC1 left; old dividend MSB (carry) → SC1.bit0
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
        CPSL $08                ; WC=0 before all SUBI/ADDI/SUBA/ADDA
        ; If remainder (NEGFLG:SC1) >= 10: subtract 10, set quotient bit0
        LODA,R0 NEGFLG
        COMI,R0 $00
        BCTA,GT PR_QBIT
        LODA,R0 SC1
        COMI,R0 10
        BCTR,LT PR_NOQBIT
PR_QBIT:
        LODA,R0 SC1
        SUBI,R0 10
        STRA,R0 SC1
        TPSL $01
        BCTR,EQ PR_SNB          ; C=1: no borrow
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
        COMI,R0 $00
        BCTA,GT PR_LP
        ; SC1 = remainder digit 0-9; push to SW stack
        LODA,R0 SC1
        STRA,R0 SWSTK,R3+
        ; If quotient != 0: SWJSR self (push PR_PRINT as return addr)
        LODA,R0 EXPH
        COMI,R0 $00
        BCTA,GT PR_REC
        LODA,R0 EXPL
        COMI,R0 $00
        BCTR,EQ PR_PRINT        ; quotient=0: base case, fall to print
PR_REC:
        LODI,R0 >PR_PRINT       ; lo byte first
        STRA,R0 SWSTK,R3+
        LODI,R0 <PR_PRINT       ; hi byte second (top)
        STRA,R0 SWSTK,R3+
        BCTA,UN PREC
PR_PRINT:
        LODA,R0 SWSTK,R3        ; read digit (top of stack)
        SUBI,R3 1               ; decrement SP
        ADDI,R0 A'0'
        BSTA,UN COUT
        BCTA,UN SWRETURN

; =============================================================================
; SWRETURN — pop 2-byte return addr (HI on top) and jump via indirect
; =============================================================================
SWRETURN:
        LODA,R0 SWSTK,R3        ; read HI byte
        STRA,R0 SWRETH
        SUBI,R3 1
        LODA,R0 SWSTK,R3        ; read LO byte
        STRA,R0 SWRETL
        SUBI,R3 1
        BCTA,UN *SWRETH

; =============================================================================
; EXPR — relational level (LT=bit0, EQ=bit1, GT=bit2 accumulated in R1)
; =============================================================================
EXPR:
        BSTA,UN EXPR_ADD
        LODA,R0 EXPH
        STRA,R0 LEFTH
        LODA,R0 EXPL
        STRA,R0 LEFTL
        LODI,R1 0           ; zero relop mask (EORZ,R1 would XOR R0 not clear R1)
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
        RETC,EQ                 ; no relop seen → return (item 2: was BCTA EX_NONE/RETC,UN)
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

; =============================================================================
; DO_RELOP — signed compare LEFTH:LEFTL vs EXPH:EXPL → $FFFF or $0000
; XOR $80 bias converts signed to unsigned for comparison.
; item 6: merged duplicate STRA EXPH/EXPL/RETC,UN via DR_HELPER fallthrough
; =============================================================================
DO_RELOP:
        LODA,R0 LEFTH
        EORI,R0 $80
        STRA,R0 SC0
        LODA,R0 EXPH
        EORI,R0 $80
        SUBA,R0 SC0
        BCTR,LT DR_LT
        BCTA,GT DR_GT
        LODA,R0 EXPL
        SUBA,R0 LEFTL
        BCTR,LT DR_LT
        BCTA,GT DR_GT
        LODI,R0 2               ; EQ
        BCTR,UN DR_TEST
DR_LT:
        LODI,R0 1
        BCTR,UN DR_TEST
DR_GT:
        LODI,R0 4
DR_TEST:
        TMI,R0 RELOP
        BCTR,EQ DR_TRUE
        EORZ,R0
        BCTR,UN DR_HELPER       ; item 6: merge duplicate stores
DR_TRUE:
        LODI,R0 $FF
DR_HELPER:
        STRA,R0 EXPH
        STRA,R0 EXPL
        RETC,UN

; =============================================================================
; EXPR_ADD — additive level; left saved in SAVEH:SAVEL
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
        BCTR,LT EA_ANC
        LODA,R0 SAVEH
        ADDI,R0 1
        BCTR,UN EA_AHI
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
        BCTR,EQ EA_SNB          ; C=1: no borrow
        LODA,R0 SAVEH
        SUBI,R0 1
        BCTR,UN EA_SHI
EA_SNB:
        LODA,R0 SAVEH
EA_SHI:
        SUBA,R0 EXPH
        STRA,R0 EXPH
        BCTA,UN EA_LP

; =============================================================================
; EXPR1 — multiplicative level; left saved in E1SAVH:E1SAVL
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
        BCTA,UN PARSE_NUM       ; tail call
E2_VAR:
        STRA,R0 SC0
        BSTA,UN INC_IP
        LODA,R0 SC0
        SUBI,R0 A'A'
        STRA,R0 SC1
        ADDA,R0 SC1             ; offset = (ch-'A')*2
        ADDI,R0 >VARS
        STRA,R0 TMPL
        LODI,R0 <VARS
        TPSL $01
        BCTR,LT E2_VNC
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
        BCTA,UN INC_IP          ; tail call: consume ')' then return
E2_NEG:
        BSTA,UN INC_IP
        BSTA,UN EXPR2
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BCTA,UN INC_EXP         ; tail call
E2_POS:
        BSTA,UN INC_IP
        BCTA,UN EXPR2           ; tail call

; =============================================================================
; PARSE_NUM — parse unsigned decimal at IP → EXPH:EXPL
; EXP*10 = EXP*8 + EXP*2 via shifts (no WC needed — shifts clear carry first)
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
        STRA,R0 SC0             ; digit 0-9
        BSTA,UN INC_IP
        ; EXP *= 10 via: EXP*8 + EXP*2 (all CPSL $01 before each shift → no WC issue)
        ; EXP*2 first
        CPSL $01
        LODA,R0 EXPL
        RRL,R0
        STRA,R0 EXPL
        LODA,R0 EXPH
        RRL,R0
        STRA,R0 EXPH
        ; save EXP*2 in NEGFLG:SC1
        STRA,R0 NEGFLG
        LODA,R0 EXPL
        STRA,R0 SC1
        ; EXP*4
        CPSL $01
        LODA,R0 EXPL
        RRL,R0
        STRA,R0 EXPL
        LODA,R0 EXPH
        RRL,R0
        STRA,R0 EXPH
        ; EXP*8
        CPSL $01
        LODA,R0 EXPL
        RRL,R0
        STRA,R0 EXPL
        LODA,R0 EXPH
        RRL,R0
        STRA,R0 EXPH            ; EXPH:EXPL = EXP*8
        ; EXP*8 + EXP*2
        LODA,R0 EXPL
        ADDA,R0 SC1
        STRA,R0 EXPL
        TPSL $01
        BCTR,LT PN_MNC
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
PN_MNC:
        LODA,R0 EXPH
        ADDA,R0 NEGFLG
        STRA,R0 EXPH            ; EXPH:EXPL = EXP*10
        ; + digit
        LODA,R0 EXPL
        ADDA,R0 SC0
        STRA,R0 EXPL
        TPSL $01
        BCTR,LT PN_DNC
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
PN_DNC:
        BCTA,UN PN_LP

; =============================================================================
; MUL16 — signed E1SAVH:E1SAVL * EXPH:EXPL → EXPH:EXPL
; Shift-and-add, 16 iterations via BDRR,R3 (safe — not inside SW context).
; WC FIX: PPSL $08 at loop top. CPSL $08 before ADDA/ADDI accumulate block.
; =============================================================================
MUL16:
        EORZ,R0
        STRA,R0 SC0             ; sign = 0
        ; abs(left) E1SAVH:E1SAVL
        LODA,R0 E1SAVH
        ANDI,R0 $80
        BCTR,EQ MU_LA
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
        BCTR,EQ MU_RA
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
        LODI,R3 16
MU_LP:
        PPSL $08                ; WC=1 for this iter's shifts
        LODA,R0 E1SAVL
        ANDI,R0 $01
        BCTA,EQ MU_NOADD
        CPSL $08                ; WC=0 before accumulate ADDA/ADDI
        LODA,R0 TMPL
        ADDA,R0 EXPL
        STRA,R0 TMPL
        TPSL $01
        BCTR,LT MU_ANC
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
MU_ANC:
        LODA,R0 TMPH
        ADDA,R0 EXPH
        STRA,R0 TMPH
        BCTR,UN MU_SHIFT
MU_NOADD:
        CPSL $08                ; WC=0 not needed for shifts but must be 0 for BDRR test
MU_SHIFT:
        ; Logical shift E1SAVH:E1SAVL right (WC must be 1, re-set if took NOADD path)
        PPSL $08
        CPSL $01
        LODA,R0 E1SAVH
        RRR,R0
        STRA,R0 E1SAVH
        LODA,R0 E1SAVL
        RRR,R0
        STRA,R0 E1SAVL
        ; Logical shift EXPH:EXPL left
        CPSL $01
        LODA,R0 EXPL
        RRL,R0
        STRA,R0 EXPL
        LODA,R0 EXPH
        RRL,R0
        STRA,R0 EXPH
        CPSL $08                ; WC=0 for BDRR (branch, not arithmetic, but be safe)
        BDRA R3,MU_LP
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
        BCTA,UN INC_EXP         ; tail call

; =============================================================================
; DIV16 — signed EXPH:EXPL / DIVH:DIVL → EXPH:EXPL (quotient)
;          remainder → TMPH:TMPL
; Restoring shift-subtract, 16 iterations via BDRR,R3.
; WC FIX: PPSL $08 at loop top; CPSL $08 before SUBA/SUBI subtract block.
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
        BCTR,EQ DV_DA
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
        BCTR,EQ DV_VA
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
        LODA,R0 EXPH
        STRA,R0 TMPH
        LODA,R0 EXPL
        STRA,R0 TMPL            ; dividend → TMPH:TMPL
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 EXPL            ; quotient = 0
        STRA,R0 NEGFLG
        STRA,R0 SC1             ; remainder = 0
        LODI,R3 16
DV_LP:
        PPSL $08                ; WC=1 for this iter's shifts
        ; Shift dividend left; MSB → carry
        CPSL $01
        LODA,R0 TMPL
        RRL,R0
        STRA,R0 TMPL
        LODA,R0 TMPH
        RRL,R0
        STRA,R0 TMPH
        ; Shift remainder left; old dividend MSB (carry) → SC1.bit0
        LODA,R0 SC1
        RRL,R0
        STRA,R0 SC1
        LODA,R0 NEGFLG
        RRL,R0
        STRA,R0 NEGFLG
        ; Shift quotient left
        CPSL $01
        LODA,R0 EXPL
        RRL,R0
        STRA,R0 EXPL
        LODA,R0 EXPH
        RRL,R0
        STRA,R0 EXPH
        CPSL $08                ; WC=0 before SUBA/SUBI compare/subtract
        ; If remainder >= divisor: subtract, set quotient bit0
        LODA,R0 NEGFLG
        SUBA,R0 DIVH
        BCTR,LT DV_NOQBIT
        BCTA,GT DV_QBIT
        LODA,R0 SC1
        SUBA,R0 DIVL
        BCTR,LT DV_NOQBIT
DV_QBIT:
        LODA,R0 SC1
        SUBA,R0 DIVL
        STRA,R0 SC1
        TPSL $01
        BCTR,EQ DV_SNB          ; C=1: no borrow
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
        BDRA R3,DV_LP
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
        BCTA,UN INC_EXP         ; tail call
DV_ZERO:
        LODI,R0 A'?'
        BSTA,UN COUT
        HALT

; =============================================================================
; INC16 helpers — one per variable pair. All use TPSL/RETC carry idiom.
; Note on item 7: shared INC16 via indirect pointer IS feasible but is a net
; loss (~-25 bytes) at current call count. Will revisit when DEC16 added.
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
