; parser_sw_test.asm - SW-stack recursive descent expression parser test harness
; Version: 0.2
; Build:   ./asm2650 parser_sw_test.asm parser_sw_test.hex
; Run:     ./pipbug_wrap parser_sw_test.hex
;
; Purpose:
;   Standalone test harness for the SW-stack recursive descent expression parser.
;   Proves correctness before porting into uBASIC2650 v27.
;   Uses v27's MUL16, DIV16, PRINT_S16 and INC_EXP verbatim.
;
; SW Stack Design:
;   The 2650 RAS is 8 slots deep. With SP=3 at PARSE_EXPR entry, naive recursive
;   descent overflows for nested parentheses (e.g. ((x+y)*z)).
;   Solution: all intra-parser calls use a software stack in RAM (SWBASE) instead
;   of BSTA. Return addresses are pushed as 2 bytes (lo then hi per v27 PREC
;   convention), and SWRETURN pops and jumps indirect.
;   External calls to PARSE_EXPR remain as BSTA (normal RAS). Leaf calls
;   (INC_IP, MUL16, DIV16) also remain as BSTA — safe at SP=5 max.
;   R3 is the SW stack pointer, initialised to $FF (empty sentinel) at
;   EXPR_AM_RAS entry. PARSER_RET returns via SW if R3!=$FF, else via RAS.
;
; Push convention (matches v27 PREC):
;   LODI,R0 >LABEL   ; lo byte first (pre-increment stores to SWBASE[R3++])
;   STRA,R0 SWBASE,R3+
;   LODI,R0 <LABEL   ; hi byte second
;   STRA,R0 SWBASE,R3+
;   BCTA,UN TARGET   ; tail call — no RAS push
;
; Expected output:
;   3     (T1:  literal 3)
;   3     (T2:  1+2)
;   6     (T3:  2*3)
;   14    (T4:  2+3*4 — operator precedence)
;   20    (T5:  (2+3)*4 — single paren)
;   20    (T6:  ((2+3))*4 — nested paren)
;   -3    (T7:  -5+2 — unary minus)
;   3     (T8:  10/3)
;   1     (T9:  10%3)
;   -10   (T10: -(5*2) — unary minus on paren)
;
; Version history:
;   0.1  Initial working version. Bugs fixed:
;          - SWRETURN pre-decremented R3 before first read (off-by-one vs v27)
;          - Entry point: SWRETURN was at $0440, needed BCTA START first
;          - PARSER_RET used COMI $FF (signed): R3=$01 compared GT not LT
;            Fixed with EORI $FF / RETC,EQ pattern
;   0.2  Added -(5*2) test. Added full subroutine headers.

; PIPBUG entry points
COUT       EQU $02B4
CRLF       EQU $008A

; RAM layout (compatible with v27)
IPH        EQU $1600   ; instruction pointer hi (page of current char)
IPL        EQU $1601   ; instruction pointer lo (offset of current char)
TMPH       EQU $1602   ; MUL16/DIV16 left operand hi / DIV16 remainder hi
TMPL       EQU $1603   ; MUL16/DIV16 left operand lo / DIV16 remainder lo
EXPH       EQU $1604   ; expression result hi
EXPL       EQU $1605   ; expression result lo
ERRFLG     EQU $1610   ; error flag (used by DIV16)
SC0        EQU $160E   ; scratch byte 0 (PARSE_U16: digit; MUL16/DIV16 internal)
SC1        EQU $160F   ; scratch byte 1 (PARSE_U16: EXP*2 lo)
NEGFLG     EQU $1611   ; sign flag (PARSE_S16: 1=negative; PARSE_U16: EXP*2 hi)

; SW stack (shared with PRINT_S16/PREC — safe, no overlap in call time)
SWBASE     EQU $1640   ; SW stack base array (grows up from index $FF)
TEMPRETH   EQU $1680   ; SWRETURN indirect jump vector: hi byte of target
TEMPRETL   EQU $1681   ; SWRETURN indirect jump vector: lo byte of target
R3SAVE     EQU $1682   ; PRINT_S16 saves caller R3 here

; Parser temporaries
SAVEH      EQU $1690   ; EXPR_AM: saved left operand hi for +/-
SAVEL      EQU $1691   ; EXPR_AM: saved left operand lo for +/-
E1SAVH     EQU $1692   ; EAM_HI: saved left operand hi for *//%
E1SAVL     EQU $1693   ; EAM_HI: saved left operand lo for *//%
RELOP      EQU $1694   ; relational operator code (unused in harness)

; Test input buffer
IBUF       EQU $1700   ; null-terminated expression string for test

           ORG $0440
           BCTA,UN START

; ── SWRETURN ─────────────────────────────────────────────────────────────────
; Pop 2-byte return address from SW stack and jump to it (indirect).
; Push convention: lo byte stored first at SWBASE[R3] (pre-increment), then hi.
; After push R3 points to last written (hi byte). Pop reads hi then lo.
; In:  R3 = SW stack pointer (pointing at hi byte of top entry)
;      SWBASE[R3]   = hi byte of return address
;      SWBASE[R3-1] = lo byte of return address
; Out: jumps to (SWBASE[R3]<<8)|SWBASE[R3-1]; R3 decremented by 2
; Clobbers: R0, TEMPRETH, TEMPRETL
SWRETURN:
           LODA,R0 SWBASE,R3
           STRA,R0 TEMPRETH
           SUBI,R3 1
           LODA,R0 SWBASE,R3
           STRA,R0 TEMPRETL
           SUBI,R3 1
           BCTA,UN *TEMPRETH

; ── PARSER_RET ───────────────────────────────────────────────────────────────
; Shared parser return: uses SW stack if active, else RAS.
; R3=$FF means SW stack empty (called via BSTA) -> RETC,UN.
; R3!=$FF means SW stack has data (called via BCTA) -> SWRETURN.
; Test: EORI $FF. If R3=$FF, result $00 (EQ) -> RETC. Else BCTA SWRETURN.
; In:  R3 = SW stack pointer
; Out: returns to caller (via RAS or SW stack)
; Clobbers: R0
PARSER_RET:
           LODZ,R3
           EORI,R0 $FF
           RETC,EQ
           BCTA,UN SWRETURN

; ── INC_IP ───────────────────────────────────────────────────────────────────
; Advance instruction pointer (IPH:IPL) by 1, wrapping within 8K page.
; In:  IPH:IPL = current position
; Out: IPH:IPL advanced by 1
; Clobbers: R0
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

; ── PARSE_EXPR ───────────────────────────────────────────────────────────────
; Top-level expression evaluator entry point. Called via BSTA by DO_PRINT etc.
; Evaluates arithmetic expression at IP. Handles optional relational operator.
; In:  IPH:IPL = pointer to null-terminated expression string
; Out: EXPH:EXPL = 16-bit signed result
; Clobbers: R0, R3, SAVEH, SAVEL, E1SAVH, E1SAVL, NEGFLG, SC0, SC1,
;           TMPH, TMPL (via MUL16/DIV16), ERRFLG
PARSE_EXPR:
           BSTA,UN EXPR_AM_RAS
           RETC,UN

; ── EXPR_AM_RAS ──────────────────────────────────────────────────────────────
; RAS entry point for EXPR_AM. Initialises SW stack pointer R3=$FF then
; falls through to EXPR_AM. Called via BSTA from PARSE_EXPR.
; In:  IPH:IPL = pointer to expression
; Out: EXPH:EXPL = result
; Clobbers: R0, R3, SAVEH, SAVEL (see EXPR_AM)
EXPR_AM_RAS:
           LODI,R3 $FF

; ── EXPR_AM ──────────────────────────────────────────────────────────────────
; Additive expression: parses one term then loops on +/- operators.
; SW entry: R3 already set by caller (via BCTA from EAM_PAREN).
; In:  IPH:IPL = pointer to expression; R3 = SW stack pointer
; Out: EXPH:EXPL = result of additive expression
; Clobbers: R0, R3, SAVEH, SAVEL
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
           LODA,R0 *IPH
           COMI,R0 A'+'
           BCTA,EQ EAM_PLUS
           COMI,R0 A'-'
           BCTA,EQ EAM_MINUS
           BCTA,UN PARSER_RET
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
           BCTA,UN EAM_HI
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

; ── EAM_HI ───────────────────────────────────────────────────────────────────
; Multiplicative expression: checks for * / % operators on the current token.
; Called via SW_CALL from EXPR_AM after each atom. Returns via PARSER_RET.
; In:  EXPH:EXPL = left operand; IPH:IPL = pointing at potential operator
;      R3 = SW stack pointer
; Out: EXPH:EXPL = result after applying any */% chain
; Clobbers: R0, R3, E1SAVH, E1SAVL, TMPH, TMPL (via MUL16/DIV16)
EAM_HI:
           LODA,R0 *IPH
           COMI,R0 A'*'
           BCTA,EQ EAM_MUL
           COMI,R0 A'/'
           BCTA,EQ EAM_DIV
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
           BCTA,UN EAM_ATOM
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

; ── EAM_ATOM ─────────────────────────────────────────────────────────────────
; Parse one atomic value: number, unary +/-, or parenthesised expression.
; Called via SW_CALL from EXPR_AM/EAM_HI. Returns via PARSER_RET.
; In:  IPH:IPL = pointing at first char of atom; R3 = SW stack pointer
; Out: EXPH:EXPL = value of atom
; Clobbers: R0, R3, NEGFLG, SC0, SC1 (via PARSE_U16)
EAM_ATOM:
           LODA,R0 *IPH
           COMI,R0 A'-'
           BCTA,EQ EAM_NEG
           COMI,R0 A'+'
           BCTA,EQ EAM_POS
           COMI,R0 A'('
           BCTA,EQ EAM_PAREN
           BSTA,UN PARSE_S16
           BCTA,UN PARSER_RET

; ── EAM_NEG ──────────────────────────────────────────────────────────────────
; Unary minus: skip '-', evaluate atom, two's complement negate result.
; Tail of EAM_ATOM. Returns via PARSER_RET.
; In:  IPH:IPL pointing at '-'; R3 = SW stack pointer
; Out: EXPH:EXPL = negated atom value
; Clobbers: R0, R3
EAM_NEG:
           BSTA,UN INC_IP
           LODI,R0 >NEG_AT_RET
           STRA,R0 SWBASE,R3+
           LODI,R0 <NEG_AT_RET
           STRA,R0 SWBASE,R3+
           BCTA,UN EAM_ATOM
NEG_AT_RET:
           LODA,R0 EXPH
           EORI,R0 $FF
           STRA,R0 EXPH
           LODA,R0 EXPL
           EORI,R0 $FF
           STRA,R0 EXPL
           BSTA,UN INC_EXP
           BCTA,UN PARSER_RET

; ── EAM_POS ──────────────────────────────────────────────────────────────────
; Unary plus: skip '+', evaluate atom unchanged.
; Tail of EAM_ATOM. Returns via PARSER_RET.
; In:  IPH:IPL pointing at '+'; R3 = SW stack pointer
; Out: EXPH:EXPL = atom value (unchanged sign)
; Clobbers: R0, R3
EAM_POS:
           BSTA,UN INC_IP
           LODI,R0 >POS_AT_RET
           STRA,R0 SWBASE,R3+
           LODI,R0 <POS_AT_RET
           STRA,R0 SWBASE,R3+
           BCTA,UN EAM_ATOM
POS_AT_RET:
           BCTA,UN PARSER_RET

; ── EAM_PAREN ────────────────────────────────────────────────────────────────
; Parenthesised subexpression: skip '(', evaluate EXPR_AM, skip ')'.
; Uses SW_CALL for EXPR_AM so nested parens don't overflow the RAS.
; RAS depth stays at 5 throughout (INC_IP is the only BSTA here).
; Tail of EAM_ATOM. Returns via PARSER_RET.
; In:  IPH:IPL pointing at '('; R3 = SW stack pointer
; Out: EXPH:EXPL = value of subexpression
; Clobbers: R0, R3 (and all of EXPR_AM)
EAM_PAREN:
           BSTA,UN INC_IP
           LODI,R0 >EP_RET
           STRA,R0 SWBASE,R3+
           LODI,R0 <EP_RET
           STRA,R0 SWBASE,R3+
           BCTA,UN EXPR_AM
EP_RET:
           BSTA,UN INC_IP
           BCTA,UN PARSER_RET

; ── PARSE_S16 ────────────────────────────────────────────────────────────────
; Parse signed decimal integer at IP into EXPH:EXPL.
; Handles optional leading '-'. Calls PARSE_U16 for unsigned part.
; In:  IPH:IPL = pointer to digit string (optional leading '-')
; Out: EXPH:EXPL = signed 16-bit integer; IP advanced past last digit
; Clobbers: R0, NEGFLG, SC0, SC1
PARSE_S16:
           EORZ,R0
           STRA,R0 NEGFLG
           LODA,R0 *IPH
           COMI,R0 A'-'
           BCTA,EQ PS16_NEG
           BCTA,UN PS16_UN
PS16_NEG:
           LODI,R0 1
           STRA,R0 NEGFLG
           BSTA,UN INC_IP
PS16_UN:
           BSTA,UN PARSE_U16
           LODA,R0 NEGFLG
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

; ── PARSE_U16 / PARSE_NUM ────────────────────────────────────────────────────
; Parse unsigned decimal integer at IP into EXPH:EXPL.
; Shift-based multiply by 10: EXP*10 = (EXP*8) + (EXP*2).
; Uses WC bit for carry propagation in the 16-bit additions.
; In:  IPH:IPL = pointer to digit string
; Out: EXPH:EXPL = unsigned 16-bit value; IP advanced past last digit
;      Returns immediately (RETC,LT / RETC,GT) if first char not a digit
; Clobbers: R0, SC0 (current digit), SC1 (EXP*2 lo), NEGFLG (EXP*2 hi)
PARSE_U16:
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
           STRA,R0 SC0
           BSTA,UN INC_IP
           CPSL $01
           LODA,R0 EXPL
           RRL,R0
           STRA,R0 SC1
           STRA,R0 EXPL
           LODA,R0 EXPH
           RRL,R0
           STRA,R0 NEGFLG
           STRA,R0 EXPH
           CPSL $01
           LODA,R0 EXPL
           RRL,R0
           STRA,R0 EXPL
           LODA,R0 EXPH
           RRL,R0
           STRA,R0 EXPH
           CPSL $01
           LODA,R0 EXPL
           RRL,R0
           STRA,R0 EXPL
           LODA,R0 EXPH
           RRL,R0
           STRA,R0 EXPH
           CPSL $08
           LODA,R0 EXPL
           ADDA,R0 SC1
           STRA,R0 EXPL
           PPSL $08
           LODA,R0 EXPH
           ADDA,R0 NEGFLG
           STRA,R0 EXPH
           CPSL $08
           LODA,R0 EXPL
           ADDA,R0 SC0
           STRA,R0 EXPL
           PPSL $08
           LODA,R0 EXPH
           ADDI,R0 0
           STRA,R0 EXPH
           CPSL $08
           BCTA,UN PN_LP

; ── RUN_TEST ─────────────────────────────────────────────────────────────────
; Point IP at IBUF, call PARSE_EXPR, print result, print CRLF.
; In:  IBUF = null-terminated expression string
; Out: result printed to COUT
; Clobbers: all (via PARSE_EXPR and PRINT_S16)
RUN_TEST:
           LODI,R0 <IBUF
           STRA,R0 IPH
           LODI,R0 >IBUF
           STRA,R0 IPL
           BSTA,UN PARSE_EXPR
           BSTA,UN PRINT_S16
           BSTA,UN CRLF
           RETC,UN

; ── START ────────────────────────────────────────────────────────────────────
; Test driver: builds each expression in IBUF, calls RUN_TEST.
START:
           ; T1: "3"  expect 3
           LODI,R0 A'3'
           STRA,R0 IBUF
           EORZ,R0
           STRA,R0 IBUF+1
           BSTA,UN RUN_TEST

           ; T2: "1+2"  expect 3
           LODI,R0 A'1'
           STRA,R0 IBUF
           LODI,R0 A'+'
           STRA,R0 IBUF+1
           LODI,R0 A'2'
           STRA,R0 IBUF+2
           EORZ,R0
           STRA,R0 IBUF+3
           BSTA,UN RUN_TEST

           ; T3: "2*3"  expect 6
           LODI,R0 A'2'
           STRA,R0 IBUF
           LODI,R0 A'*'
           STRA,R0 IBUF+1
           LODI,R0 A'3'
           STRA,R0 IBUF+2
           EORZ,R0
           STRA,R0 IBUF+3
           BSTA,UN RUN_TEST

           ; T4: "2+3*4"  expect 14 (operator precedence)
           LODI,R0 A'2'
           STRA,R0 IBUF
           LODI,R0 A'+'
           STRA,R0 IBUF+1
           LODI,R0 A'3'
           STRA,R0 IBUF+2
           LODI,R0 A'*'
           STRA,R0 IBUF+3
           LODI,R0 A'4'
           STRA,R0 IBUF+4
           EORZ,R0
           STRA,R0 IBUF+5
           BSTA,UN RUN_TEST

           ; T5: "(2+3)*4"  expect 20 (single paren)
           LODI,R0 A'('
           STRA,R0 IBUF
           LODI,R0 A'2'
           STRA,R0 IBUF+1
           LODI,R0 A'+'
           STRA,R0 IBUF+2
           LODI,R0 A'3'
           STRA,R0 IBUF+3
           LODI,R0 A')'
           STRA,R0 IBUF+4
           LODI,R0 A'*'
           STRA,R0 IBUF+5
           LODI,R0 A'4'
           STRA,R0 IBUF+6
           EORZ,R0
           STRA,R0 IBUF+7
           BSTA,UN RUN_TEST

           ; T6: "((2+3))*4"  expect 20 (nested paren — tests unlimited RAS depth)
           LODI,R0 A'('
           STRA,R0 IBUF
           LODI,R0 A'('
           STRA,R0 IBUF+1
           LODI,R0 A'2'
           STRA,R0 IBUF+2
           LODI,R0 A'+'
           STRA,R0 IBUF+3
           LODI,R0 A'3'
           STRA,R0 IBUF+4
           LODI,R0 A')'
           STRA,R0 IBUF+5
           LODI,R0 A')'
           STRA,R0 IBUF+6
           LODI,R0 A'*'
           STRA,R0 IBUF+7
           LODI,R0 A'4'
           STRA,R0 IBUF+8
           EORZ,R0
           STRA,R0 IBUF+9
           BSTA,UN RUN_TEST

           ; T7: "-5+2"  expect -3 (unary minus)
           LODI,R0 A'-'
           STRA,R0 IBUF
           LODI,R0 A'5'
           STRA,R0 IBUF+1
           LODI,R0 A'+'
           STRA,R0 IBUF+2
           LODI,R0 A'2'
           STRA,R0 IBUF+3
           EORZ,R0
           STRA,R0 IBUF+4
           BSTA,UN RUN_TEST

           ; T8: "10/3"  expect 3
           LODI,R0 A'1'
           STRA,R0 IBUF
           LODI,R0 A'0'
           STRA,R0 IBUF+1
           LODI,R0 A'/'
           STRA,R0 IBUF+2
           LODI,R0 A'3'
           STRA,R0 IBUF+3
           EORZ,R0
           STRA,R0 IBUF+4
           BSTA,UN RUN_TEST

           ; T9: "10%3"  expect 1
           LODI,R0 A'1'
           STRA,R0 IBUF
           LODI,R0 A'0'
           STRA,R0 IBUF+1
           LODI,R0 A'%'
           STRA,R0 IBUF+2
           LODI,R0 A'3'
           STRA,R0 IBUF+3
           EORZ,R0
           STRA,R0 IBUF+4
           BSTA,UN RUN_TEST

           ; T10: "-(5*2)"  expect -10 (unary minus on parenthesised expression)
           LODI,R0 A'-'
           STRA,R0 IBUF
           LODI,R0 A'('
           STRA,R0 IBUF+1
           LODI,R0 A'5'
           STRA,R0 IBUF+2
           LODI,R0 A'*'
           STRA,R0 IBUF+3
           LODI,R0 A'2'
           STRA,R0 IBUF+4
           LODI,R0 A')'
           STRA,R0 IBUF+5
           EORZ,R0
           STRA,R0 IBUF+6
           BSTA,UN RUN_TEST

           HALT


; ── MUL16/DIV16/PRINT_S16/INC_EXP (verbatim from v27) ──────────────────────
MUL16:
        EORZ,R0 ; Clear R0
        STRA,R0 NEGFLG
        ; abs(left) TMPH:TMPL
        LODA,R0 TMPH
        ANDI,R0 $80
        BCTR,EQ MU_LA
        LODA,R0 TMPH
        EORI,R0 $FF
        STRA,R0 TMPH
        LODA,R0 TMPL
        EORI,R0 $FF
        STRA,R0 TMPL
        LODA,R0 TMPL
        ADDI,R0 1
        STRA,R0 TMPL
        TPSL $01                 ; BUG-SCA-14 FIX: carry from lo-byte +1
        BCTR,LT MU_LNC           ; branch if C=0 (no carry)
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
MU_LNC:
        LODI,R0 1               ; ISSUE-01 FIX (corrected): set NEGFLG=1 on BOTH
        STRA,R0 NEGFLG          ; carry and no-carry paths     left was negative
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
        LODA,R0 EXPL
        ADDI,R0 1
        STRA,R0 EXPL
        ; BUG-SCA-09 FIX: was BCTA,GT MU_RA     this jumped over BOTH the hi-byte
        ; increment AND the NEGFLG toggle, so for most negative right values (those
        ; whose +1 does not carry to hi byte, e.g. -3   $FFFD, abs=$0003) NEGFLG was
        ; never toggled     wrong sign (3*-3=+9 not -9). Fix: introduce MU_RA_NC so
        ; no-carry path skips only the hi-byte increment, then BOTH paths toggle.
        TPSL $01                 ; BUG-SCA-14 FIX: carry from lo-byte +1
        BCTR,LT MU_RA_NC         ; branch if C=0 (no carry): skip hi increment
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
MU_RA_NC:
        LODA,R0 NEGFLG          ; toggle sign on BOTH carry and no-carry paths
        EORI,R0 $01
        STRA,R0 NEGFLG
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
        TPSL $01                 ; BUG-SCA-14 FIX: carry from lo-byte add
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
        LODA,R0 NEGFLG
        BCTR,EQ MU_RET
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
MU_RET:
        EORZ,R0                          ; ISSUE-01 RE-FIX pt2: clear NEGFLG on
        STRA,R0 NEGFLG                   ; exit     dual-use with CHR$ flag in DO_PRINT
        RETC,UN

;           DIV16                                                                                                                                                                                                             
; Signed TMPH:TMPL    EXPH:EXPL     EXPH:EXPL  (truncate toward zero)
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
        ; abs(dividend) TMPH:TMPL
        LODA,R0 TMPH
        ANDI,R0 $80
        BCTR,EQ DV_DA
        LODA,R0 TMPH
        EORI,R0 $FF
        STRA,R0 TMPH
        LODA,R0 TMPL
        EORI,R0 $FF
        STRA,R0 TMPL
        LODA,R0 TMPL
        ADDI,R0 1
        STRA,R0 TMPL
        TPSL $01                 ; BUG-SCA-14 FIX: carry from lo-byte +1
        BCTR,LT DV_DNC           ; branch if C=0 (no carry)
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
DV_DNC:
        LODI,R0 1               ; ISSUE-01 FIX (corrected): set NEGFLG=1 on BOTH
        STRA,R0 NEGFLG          ; carry and no-carry paths     dividend was negative
DV_DA:
        ; abs(divisor) EXPH:EXPL
        LODA,R0 EXPH
        ANDI,R0 $80
        BCTR,EQ DV_VA
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        LODA,R0 EXPL
        ADDI,R0 1
        STRA,R0 EXPL
        ; BUG-SCA-09b FIX: same as MUL16 right-operand fix. BCTA,GT DV_VA jumped
        ; over BOTH hi-byte increment AND NEGFLG toggle for no-carry cases.
        ; Fix: introduce DV_VA_NC so no-carry skips only the hi-byte increment.
        TPSL $01                 ; BUG-SCA-14 FIX: carry from lo-byte +1
        BCTR,LT DV_VA_NC         ; branch if C=0 (no carry): skip hi increment
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
DV_VA_NC:
        LODA,R0 NEGFLG          ; toggle sign on BOTH paths
        EORI,R0 $01
        STRA,R0 NEGFLG
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
        BCTR,LT DV_DONE           ; TMPH < SC0 (signed OK if SC0 < $80)     done
        BCTR,GT DV_SUB            ; TMPH > SC0     subtract
        ; TMPH == SC0: unsigned lo comparison via carry
        LODA,R0 TMPL
        SUBA,R0 SC1
        TPSL $01                  ; C=1     no borrow     TMPL >= SC1     subtract
        BCTR,EQ DV_SUB            ; C=1     TMPL >= SC1     continue subtract
        BCTR,UN DV_DONE           ; C=0     TMPL < SC1     done
DV_SUB:
        LODA,R0 TMPL
        SUBA,R0 SC1
        STRA,R0 TMPL
        TPSL $01                  ; C=1     no borrow     skip hi decrement
        BCTR,EQ DV_SNB            ; C=1     no borrow
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
        LODA,R0 NEGFLG
        BCTR,EQ DV_RET
        LODA,R0 EXPH
        EORI,R0 $FF
        STRA,R0 EXPH
        LODA,R0 EXPL
        EORI,R0 $FF
        STRA,R0 EXPL
        BSTA,UN INC_EXP
DV_RET:
        EORZ,R0                          ; ISSUE-01 RE-FIX pt2: clear NEGFLG on
        STRA,R0 NEGFLG                   ; exit     dual-use with CHR$ flag in DO_PRINT
        RETC,UN
JERRDIVZER:
        HALT                    ; divide by zero

;           PRINT_S16                                                                                                                                                                                                 
; Recursive Print signed 16-bit value EXPH:EXPL as decimal.
PRINT_S16:
        ; Save caller R3 and switch to dedicated recursive print SW stack.
        STRA,R3 R3SAVE
        LODI,R3 $FF

        LODA,R0 EXPH
        ANDI,R0 $80
        BCTA,EQ PS_POS
        LODI,R0 A'-'
        BSTA,UN COUT
        ; -32768 special case
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

; PREC     SW recursive digit printer (divide EXP by 10, recurse, print)
PREC:
        LODA,R0 EXPH
        STRA,R0 TMPH
        LODA,R0 EXPL
        STRA,R0 TMPL            ; dividend     TMPH:TMPL
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
        LODA,R0 SWBASE,R3
        SUBI,R3 1
        ADDI,R0 A'0'
        BSTA,UN COUT
SWRETURN:
        LODA,R0 SWBASE,R3
        STRA,R0 TEMPRETH
        SUBI,R3 1
        LODA,R0 SWBASE,R3
        STRA,R0 TEMPRETL
        SUBI,R3 1
        BCTA,UN *TEMPRETH

PS_DONE:
        ; restore caller R3 and return
        LODZ,R3
        LODA,R3 R3SAVE
        RETC,UN

; Increment 16 bit regsiter
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
