; uBASIC2650.asm       Tiny BASIC interpreter for Signetics 2650
; Version: v3.6
; Date:    2026-06-10
;
; Target : PIPBUG 1 monitor (ROM $0000-$03FF, RAM $0400-$043F)
;          Code base $0440.  Single 8192-byte address page (bit15-13 always 0).
;          I/O via PIPBUG ROM stubs: COUT=$02B4 (R0=char), CHIN=$0286 (R0=key).
;          CRLF inlined (2 bytes) avoids consuming a RAS slot vs PIPBUG CRLF.
;
; Assembler: asm2650.c v1.9+   Simulator: pipbug_wrap.c v2.0
; Build:
;   gcc -Wall -O2 -o asm2650 asm2650.c
;   gcc -Wall -O2 -DGAMER -o pipbug_wrap pipbug_wrap.c
;   ./asm2650 uBASIC2650.asm uBASIC2650.hex
;   ./pipbug_wrap uBASIC2650.hex
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
;   Every BSxx (BSTA/BSTR/BSFA/BSFR) consumes one slot regardless of condition.
;   BCTA/BCTR/BCFA/BCFR are plain branches -- no RAS cost.
;   COUT/CHIN use 1 internal sub: add 1 to caller depth.
;   PARSE_EXPR entry guard: SPSU/ANDI/COMI fires ERR_NEST if SP>=5 at entry.
;   This costs 0 RAS slots (no BSTA) and protects against stack overflow.
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
;
;   V3.6  2026-06-10  - 3748 Interpreter bytes
;         SET_IP_IBUF helper: LODI IPH=<IBUF / IPL=>IBUF (5 call sites, ~25 bytes saved).
;         EXP16_TO_GOTO helper: EXPH->GOTOH / EXPL->GOTOL (3 sites, ~7 bytes saved).
;         EXP16_TO_LNUM helper: EXPH->LNUMH / EXPL->LNUML (3 sites, ~7 bytes saved).
;         ERRFLG eliminated: status returned in CC (R0 side-effect), not RAM flag.
;           TRY_STORE_LINE: RETC,UN with CC=GT (stored) or CC=EQ (not stored).
;           FIND_LINE: RETC,UN with CC=GT (not found) or CC=EQ (found).
;           PARSE_EXPR/PF_LOADVAR/DIV16: dead ERRFLG clears and EORZ removed.
;           RAM cell $1610 freed. Net ~28 bytes saved.
;         DEC_ET family: mirrors INC_ET using $EC/$C4 byte-skip chain for offsets 0/8/12.
;           DEC_IP: rewritten as 1-byte stub (EORZ,R0) falling into shared DEC_ET body.
;           DEC_GOTO (offset 8): LODI,R0 8 + $C4 stub -> replaces SL_SNC inline (saves 15).
;           DEC_LNUM (offset 12): LODI,R0 12 + $EC stub -> replaces SL_DOMOV inline (saves 15).
;           DEC_EXP/DEC_TMP omitted: MUL16 site is at RAS depth 5+1=6 (unsafe).
;           Net: 44 bytes saved at call sites, +11 bytes family overhead = 33 bytes net.
;         DL2_SCAN: inline TMP+2 replaced by 2x BSTA INC_TMP. Saves 14 bytes.
;         FREE memory keyword added.
;
;   V3.5  2026-06-09  Merged v3.3+v3.4 FOR/NEXT into v3.2 optimised baseline.
;         Source policy: smallest code + correct functionality.
;         DO_FOR:  v3.3 parse (GETCI_UC for '=', LNUMH/LNUML for limit,
;                  EXPH/EXPL direct for step -- OPT-F2/F3/F4).
;                  v3.4 DF_PUSH (R1-indexed FORBASE,R1 -- no INC_TMP chain,
;                  reads LNUMH/LNUML for limit, EXPH/EXPL for step).
;         DO_NEXT: v3.4 VARS access (direct VARS,R1 indexed -- VARS_FP dropped).
;                  v3.3 compare (shared biased-compare with fall-through,
;                  step sign from EXPH bit7 -- shorter than v3.4 TMI dual path).
;                  v3.3 WC idiom for 16-bit step add.
;                  v3.4 FORBASE,R1 for nlp load and frame pop.
;         DO_RETURN: v3.3 GOTOFLG=$03 (direct NLP). v3.4 regressed to $01.
;         DR_EXEC:   v3.3 three-way GOTOFLG dispatch (inline COMI $03).
;         STORE_LINE: v3.4 BUG-LE fix (BCTR,LT SL_NOSHIFT both paths).
;         PARSE_EXPR: v3.3 body + v3.2 RAS guard restored.
;                     ERR_NEST='8' (v3.2 had '5'; '5'/'6'/'7' now used by
;                     ERR_RET/ERR_FOR/ERR_NXT respectively).
;         VARS_FP: dropped (replaced by inline VARS,R1 indexing in DO_NEXT).
;
;   V3.4  2026-06-09  FOR/NEXT variant 2 (v3.3 parallel branch).
;         BUG-SE-01, BUG-DN-01..04, BUG-LE fixes.
;         DF_PUSH R1-indexed frame write (smaller than INC_TMP chain).
;         DO_NEXT VARS,R1 indexed (drops VARS_FP subroutine).
;         Regression: DO_RETURN GOTOFLG=$01 (should be $03). Not carried forward.
;
;   V3.3  2026-06-07  FOR/NEXT and GOSUB/RETURN complete. ROMEND=$13AC.
;         ERR_FOR='6', ERR_NXT='7'. PSL_WC EQU $08.
;         FORBASE=$1670: 4-level FOR stack, 7 bytes/frame.
;         OPT-F2/F3/F4 applied to DO_FOR parse.
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
ERR_RET         EQU '5'         ; RETURN without GOSUB (GOSUB stack underflow)
ERR_FOR         EQU '6'         ; Too many nested FORs (FORBASE stack overflow)
ERR_NXT         EQU '7'         ; NEXT without FOR (FORBASE stack underflow)
ERR_NEST        EQU '8'         ; Expression nesting too deep (RAS guard, v3.2 had '5')

;  PIPBUG 1 I/O entry points
COUT    EQU     $02B4   ; putchar: R0 = char to output
CHIN    EQU     $0286   ; getchar: blocking: R0 = key
RS      EQU     $10
PSL_WC  EQU     $08             ; WC (With Carry) bit in PSL (bit 3)

;  RAM variables  pinned above code, below PROGLIM
; Code ceiling: ~$15FF (code must not reach $1600 or crash).
; IP, TMP, EXP must be in this order (INC_ET uses R1-indexed addressing from IPH base).
IPH     EQU $1600   ; interpreter pointer hi
IPL     EQU $1601   ; interpreter pointer lo
TMPH    EQU $1602   ; temp 16-bit hi
TMPL    EQU $1603   ; temp 16-bit lo
EXPH    EQU $1604   ; expression result hi
EXPL    EQU $1605   ; expression result lo

; Other Vars
RUNFLG  EQU $1606   ; $01=running $00=immediate
GOTOFLG EQU $1607   ; $00=sequential $01=GOTO $02=GOSUB $03=FOR direct addr
GOTOH   EQU $1608   ; pending target hi
GOTOL   EQU $1609   ; pending target lo
CURH    EQU $160A   ; current line hi  (error reporting)
CURL    EQU $160B   ; current line lo
LNUMH   EQU $160C   ; scratch line number hi  (also FOR limit hi during DO_FOR)
LNUML   EQU $160D   ; scratch line number lo  (also FOR limit lo during DO_FOR)
SC0     EQU $160E   ; scratch byte 0
SC1     EQU $160F   ; scratch byte 1
                    ; $1610 free (was ERRFLG, removed v3.6 -- status now in CC)
NEGFLG  EQU $1611   ; sign flag
PEH     EQU $1612   ; program end pointer hi
PEL     EQU $1613   ; program end pointer lo
SAVEH   EQU $1614   ; EXPR_AM: saved left hi for +/-
SAVEL   EQU $1615   ; EXPR_AM: saved left lo for +/-
E1SAVH  EQU $1616   ; EAM_HI: saved left hi for *//%
E1SAVL  EQU $1617   ; EAM_HI: saved left lo for *//%
FORVAR  EQU $161C   ; FOR loop variable letter (A-Z)
FORSP   EQU $161D   ; FOR stack pointer ($FF=empty, 0/7/14/21=frame offsets)
                    ; $161E-$162C free
SWSP    EQU $162D   ; GOSUB stack pointer ($FF=empty); cleared by DO_END/DO_NEW/DO_ERROR
SWSTK   EQU $162E   ; next-line pointer cache [NLP_H][NLP_L] written by DR_EXEC
RELOP   EQU $163E   ; relational op bitmask: bit0=LT bit1=EQ bit2=GT

;  SW call stack -- used by PARSE_EXPR / PRINT_S16 only
; R3 = index ($FF=empty, grows up). Each frame = [lo][hi].
; Push: STRA,R0 *SWBASE,R3+ (lo first), STRA,R0 *SWBASE,R3+ (hi).
; Pop:  LODA,R0 *SWBASE,R3- (hi first), LODA,R0 *SWBASE,R3- (lo).
SWBASE   EQU $1640  ; SW stack base: 32 bytes  $1640-$165F

;  GOSUB stack (v3.2) -- managed by SWSP
; Frame = [lo][hi] of NLP. SWSP=$FF=empty. 2 bytes/frame, 8 frames.
GSBASE   EQU $1660  ; GOSUB stack base: 16 bytes  $1660-$166F
GSSTKLIM EQU $0F    ; max SWSP before overflow

;  FOR/NEXT stack (v3.3) -- managed by FORSP
; Frame (7 bytes): [var][limH][limL][stpH][stpL][nlpH][nlpL]
;   var=letter A-Z, lim=signed limit, stp=signed step, nlp=loop-back address.
; FORSP=$FF=empty. Offsets: 0/7/14/21 for frames 1-4. 4 frames = 28 bytes.
; Overflow: FORSP >= FORSTKLIM before push -> ERR_FOR.
FORBASE  EQU $1670  ; FOR stack base: 28 bytes  $1670-$168B
FORSTKLIM EQU $15   ; max FORSP before overflow (offset 21=$15 = 4th frame start)

TEMPRETH EQU $168C  ; SW return address hi
TEMPRETL EQU $168D  ; SW return address lo
R3SAVE   EQU $168E  ; save/restore R3 across PARSE_U16 multiply loop
IBUF    EQU $168F   ; input buffer 64 bytes  $168F-$16CE
VARS    EQU $16CF   ; A-Z variables 2 bytes each  $16CF-$1702
PROG    EQU $1703   ; program store base (VARS+52)
PROGLIM EQU $1FFF   ; one past end of program store

;  CODE starts at $0440 (after Pipbug 1kB ROM + 64B RAM)
        ORG     $0440

; =============================================================================
;  RESET / ENTRY
; In:  nothing (cold start)
; Out: banner printed, REPL entered
; Clobbers: all
RESET:
        CPSL $FF                ; clear PSL: CC=EQ, C=0, RS=0, SP=0

        ; Pre-load SHOWCASE_END as program so RUN executes the showcase.
        ; Delete and change BSTA DO_END to BSTA DO_NEW below to start with empty program.
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
        CPSL RS + 7             ; ensure primary reg bank, SP=0
        BSTA,UN PRT_CHEV
        BSTA,UN PRT_SPACE
        BSTA,UN RDLINE
        BSTA,UN SET_IP_IBUF              ; IPH:IPL = IBUF
        BSTA,UN TRY_STORE_LINE           ; CC=GT: line stored/deleted; CC=EQ: not a line
        BCTR,GT REPL                     ; line stored: back to prompt
        BSTR,UN STMT_EXEC
        BCTR,UN REPL

; =============================================================================
BANNER:
        DB CR, LF, "uBASIC 2650 V3.6", CR, LF, "Bytes Free:",NUL

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
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        RETC,EQ                          ; blank line: return

        BSTA,UN GETCI_UC
        STRA,R0 SC0                      ; char1 uppercase, IP advanced
        BSTA,UN GETCI_UC
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
        LODA,R0 TMPL
        ADDI,R0 5
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
        BCTR,EQ SE_CHK3
        ; c2 mismatch: advance remaining 4 bytes
        LODA,R0 TMPL
        ADDI,R0 4
        STRA,R0 TMPL
        TPSL $01
        BCTR,LT SE_SCAN
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
        BCTA,UN SE_SCAN
SE_CHK3:
        ; c1+c2 matched. Read table c3.
        ; If table c3 == A' ': wildcard, accept without consuming input c3.
        ; Otherwise: peek *IPH uppercase, compare; IP not advanced (EATWORD handles rest).
        ; BUG-SE-01 FIX: c3-mismatch stride must be +3 (c3+hi+lo -> next c1), not +2.
        BSTA,UN INC_TMP                  ; point to c3 byte in table
        LODA,R0 *TMPH
        COMI,R0 A' '
        BCTR,EQ SE_MATCH                 ; wildcard: accept
        STRA,R0 EXPL                     ; save table-c3 in EXPL (scratch)
        LODA,R0 *IPH                     ; peek input stream c3 (do NOT advance IP)
        BSTA,UN UPCASE                   ; [+1]
        SUBA,R0 EXPL
        BCTR,EQ SE_MATCH                 ; c3 matched
        ; c3 mismatch: advance remaining 3 bytes (c3+hi+lo -> next c1)
        LODA,R0 TMPL
        ADDI,R0 3
        STRA,R0 TMPL
        TPSL $01
        BCTA,LT SE_SCAN
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
        BCTA,UN SE_SCAN
SE_MATCH:
        BSTA,UN EATWORD                  ; [+1] consume remaining alpha chars
        ; load handler address from next 2 bytes: [hi][lo]
        BSTA,UN INC_TMP
        LODA,R0 *TMPH
        STRA,R0 EXPH                     ; handler hi
        BSTA,UN INC_TMP
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
        BCTR,LT JSYNERR
        COMI,R0 A'Z'+1
        BCTR,GT JSYNERR
        LODA,R0 SC1
        COMI,R0 A'='
        BCTR,EQ SE_BAREASS
        BCTR,UN JSYNERR
SE_BAREASS:
        LODA,R0 SC0
        STRZ,R2                          ; save letter in R2 (survives PARSE_EXPR)
        BCTA,UN DL_EX                    ; IP already past '=', expression follows

; =============================================================================
;  JSYNERR -- Global syntax error jump
; In:  nothing (R0 irrelevant)
; Out: jumps to DO_ERROR with ERR_SYN
; Clobbers: R0
JSYNERR:
        LODI,R0 ERR_SYN
        BCTA,UN DO_ERROR

; =============================================================================
;  DO_NEW -- Clear program store
; Syntax: NEW
; In:  nothing
; Out: PEH:PEL = PROG (empty program store); falls through to DO_END
; Clobbers: R0, SWSP, FORSP, GOTOFLG, RUNFLG
DO_NEW:
        LODI,R0 <PROG
        STRA,R0 PEH
        LODI,R0 >PROG
        STRA,R0 PEL
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
        BCTA,UN CLR_RUNFLG              ; tail call

; =============================================================================
;  DO_PRINT / PRTSTR -- Print statement and NUL-terminated string helper
; Syntax: PRINT [item {; item}]
;   item = "string" | expr | TAB(n) | CHR$(n)
;   Trailing ; suppresses newline.
; In:  IP -> first char after PRINT keyword
; Out: text written to COUT; IP advanced past statement
; Clobbers: R0, R1, EXPH, EXPL, TMPH, TMPL, NEGFLG, LNUMH, LNUML, SC0, SC1
DO_PRINT:
        BSTA,UN WSKIP
        LODA,R0 *IPH
        BCTA,EQ DP_NL

DP_ITEM:
        BSTA,UN WSKIP
        LODA,R0 *IPH
        COMI,R0 DQ
        BCTA,EQ DP_STRING
        COMI,R0 'C'
        BCFR,EQ DP_TAB          ; not 'C': forward to DP_TAB

        BSTA,UN INC_IP
        LODA,R0 *IPH
        COMI,R0 'H'
        BCTR,EQ DP_CHAR

DP_BACKUP:
        BSTA,UN DEC_IP          ; fall through to DP_EXPR

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
        BCFR,EQ DP_EXPR         ; not 'T': fall back to DP_EXPR
        BSTA,UN INC_IP
        LODA,R0 *IPH
        COMI,R0 'A'
        BCFR,EQ DP_BACKUP
        BSTA,UN EATWORD
        BSTA,UN PARSE_EXPR
        LODA,R1 EXPL
        BCTR,EQ DP_SEP          ; TAB(0): skip
TAB_LOOP:
        BSTA,UN PRT_SPACE
        BDRR,R1 TAB_LOOP
        ; fall through to DP_SEP

DP_SEP:
        BSTA,UN WSKIP
        LODA,R0 *IPH
        COMI,R0 $3B             ; semicolon
        BCTR,EQ DP_SEMI
        ; fall through to DP_NL

DP_NL:
        BCTA,UN PRT_CRLF          ; tail call: return from DO_PRINT

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
        RETC,EQ                 ; NUL before closing ": bail
        COMI,R0 DQ
        BCTR,EQ DP_SCLS
        BSTA,UN COUT
        BSTA,UN INC_IP
        BCTR,UN PRTSTR

DP_SCLS:
        BSTA,UN INC_IP
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
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        BSTA,UN UPCASE                   ; [+1]
        COMI,R0 A'A'
        BCTR,LT JERRVAR
        COMI,R0 A'Z'+1
        BCTR,LT DL_VAROK
JERRVAR:
        LODI,R0 ERR_VAR
        BCTA,UN DO_ERROR
DL_VAROK:
        STRA,R0 SC0                      ; save variable letter in SC0
        STRZ,R2                          ; save in R2 (survives PARSE_EXPR)
        BSTA,UN INC_IP
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        COMI,R0 A'='
        BCTR,EQ DL_EQC
        BCTA,UN JSYNERR
DL_EQC:
        BSTA,UN INC_IP
DL_EX:
        BSTA,UN PARSE_EXPR               ; [+1]
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
;  DO_INPUT -- Read signed integer from user into variable
; Syntax: INPUT V
; In:  IP -> variable letter
; Out: VARS[V] = parsed value
; Clobbers: R0, R2, SC0, SC1, EXPH, EXPL, TMPH, TMPL, IBUF
DO_INPUT:
        BSTA,UN WSKIP                    ; [+1]
        LODA,R0 *IPH
        BSTA,UN UPCASE                   ; [+1]
        COMI,R0 A'A'
        BCTR,LT DIN_ERR
        COMI,R0 A'Z'+1
        BCTR,LT DIN_VAROK
DIN_ERR:
        BCTA,UN JERRVAR
DIN_VAROK:
        STRA,R0 SC0
        STRZ,R2                          ; save in R2 for DL_STORE
        BSTA,UN INC_IP
        BSTA,UN PRT_QUEST
        BSTA,UN PRT_SPACE
        BSTA,UN RDLINE                   ; [+1]
        BSTR,UN SET_IP_IBUF              ; IPH:IPL = IBUF
        BSTA,UN PARSE_S16                ; [+1]
        BCTR,UN DL_STORE
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
;  DO_IF -- Conditional execution
; Syntax: IF expr relop expr THEN stmt
; In:  IP -> first char after IF keyword
; Out: executes stmt if condition true; otherwise sequential return
; Clobbers: R0, R1, EXPH, EXPL, LNUMH, LNUML, SC0, SC1, RELOP
; RAS: entry+1(PE)+1(PR)+1(PE)+1(SE) = entry+4. Max depth 7: ok.
DO_IF:
        BSTA,UN PARSE_EXPR               ; [+1]
        BSTA,UN EXP16_TO_LNUM            ; LNUMH:LNUML = EXPH:EXPL (save left operand)
        BSTA,UN PARSE_RELOP              ; [+1]
        BSTA,UN PARSE_EXPR               ; [+1]

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
        BSTA,UN WSKIP                    ; [+1]
        BSTA,UN GETCI_UC                 ; [+1] must be 'T'
        COMI,R0 A'T'
        BCTR,EQ DIF_TH2
        BCTA,UN JSYNERR
DIF_TH2:
        BSTA,UN GETCI_UC                 ; [+1] must be 'H'
        COMI,R0 A'H'
        BCTR,EQ DIF_EW
        BCTA,UN JSYNERR
DIF_EW:
        BSTA,UN EATWORD                  ; [+1]

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
;  DO_GOTO -- Computed GOTO
; Syntax: GOTO expr
; In:  IP -> first char after GOTO keyword
; Out: GOTOH:GOTOL = target line; GOTOFLG=$01
; Clobbers: R0, EXPH, EXPL, GOTOH, GOTOL, GOTOFLG
DO_GOTO:
        BSTA,UN WSKIP
        BSTA,UN PARSE_EXPR               ; [+1]
        BSTR,UN EXP16_TO_GOTO            ; GOTOH:GOTOL = EXPH:EXPL
        LODI,R0 1
        STRA,R0 GOTOFLG
        LODA,R0 RUNFLG                   ; OPT-10
        RETC,GT                          ; return if running
        BCTA,UN CLR_RUNFLG

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
;  DO_GOSUB -- Subroutine call
; Syntax: GOSUB <line>
; In:  IP -> line number; SWSTK[0:1] = NLP from DR_EXEC; SWSP = stack ptr.
; Out: GOTOH:GOTOL = target line; GOTOFLG=$02; NLP pushed onto GSBASE.
; Clobbers: R0, R1, EXPH, EXPL, GOTOH, GOTOL, GOTOFLG, SWSP
; Stack: GSBASE[SWSP]=lo, GSBASE[SWSP+1]=hi. SWSP=$FF=empty.
DO_GOSUB:
        BSTA,UN WSKIP                    ; [+1]
        BSTA,UN PARSE_EXPR               ; [+1] target line -> EXPH:EXPL
        ; overflow check
        LODA,R0 SWSP
        COMI,R0 $FF
        BCTR,EQ DGS_FIRST
        COMI,R0 GSSTKLIM
        BCTR,LT DGS_NEXT
        LODI,R0 ERR_OOM
        BCTA,UN DO_ERROR
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
        BCTA,UN CLR_RUNFLG

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
        BCTR,EQ DRT_UNDERFLOW
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
DRT_UNDERFLOW:
        LODI,R0 ERR_RET
        BCTA,UN DO_ERROR

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
        BSTA,UN WSKIP                    ; [+1] skip whitespace before var
        BSTA,UN GETCI_UC                 ; [+1] R0 = uppercase var letter
        STRA,R0 FORVAR
        STRZ,R2                          ; R2 = var letter (survives PARSE_EXPR)
        ; --- stack overflow check ---
        LODA,R0 FORSP
        COMI,R0 $FF
        BCTR,EQ DF_FIRST                 ; $FF=empty: first frame at offset 0
        COMI,R0 FORSTKLIM
        BCTR,LT DF_ROOM
        LODI,R0 ERR_FOR
        BCTA,UN DO_ERROR
DF_FIRST:
        EORZ,R0
        STRA,R0 FORSP
        BCTR,UN DF_PARSE
DF_ROOM:
        LODA,R0 FORSP
        ADDI,R0 7
        STRA,R0 FORSP
DF_PARSE:
        ; --- skip '=' then parse start value -> EXPH:EXPL ---
        ; OPT-F2: GETCI_UC skips whitespace + reads '=' in one call.
        BSTA,UN GETCI_UC                 ; [+1] skip whitespace + consume '='
        BSTA,UN PARSE_EXPR               ; [+1] start value -> EXPH:EXPL
        BSTA,UN DL_STORE                 ; [+1] VARS[R2] = EXPH:EXPL
        ; --- consume "TO" keyword ---
        BSTA,UN WSKIP                    ; [+1]
        BSTA,UN EATWORD                  ; [+1]
        ; --- parse limit -> LNUMH:LNUML ---
        BSTA,UN PARSE_EXPR               ; [+1]
        BSTA,UN EXP16_TO_LNUM            ; LNUMH:LNUML = EXPH:EXPL (limit)
        ; --- check for STEP keyword ---
        ; OPT-F3: GETCI_UC peeks first non-space char.
        ;   If 'S': consume rest of "STEP" with EATWORD, then parse step.
        ;   Else: DEC_IP to un-consume, use default step = +1.
        BSTA,UN GETCI_UC                 ; [+1] R0 = first non-space char (consumed)
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
        BSTA,UN EATWORD                  ; [+1] consume "TEP"
        BSTA,UN PARSE_EXPR               ; [+1] step -> EXPH:EXPL
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
; Implementation notes:
;   VARS access: v3.4 direct VARS,R1 indexed (VARS_FP subroutine dropped).
;   Step add:    v3.3 WC idiom (CPSL $08/lo-add/PPSL $08/hi-add/CPSL $08).
;   Compare:     v3.3 shared biased-compare with fall-through DN_VAR_LT->DN_LOOP.
;                Step sign from EXPH bit7 (LT=negative). Shorter than v3.4 TMI paths.
;   nlp/pop:     v3.4 FORBASE,R1 indexed.
DO_NEXT:
        LODA,R0 FORSP
        COMI,R0 $FF
        BCFR,EQ DN_OK                    ; not $FF: proceed
        LODI,R0 ERR_NXT
        BCTA,UN DO_ERROR
DN_OK:
        BSTA,UN WSKIP                    ; [+1]
        BSTA,UN EATWORD                  ; [+1] consume optional var name

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
        CPSL PSL_WC                      ; clear WC
        LODA,R0 LNUML
        ADDA,R0 EXPL                     ; lo: var_lo + step_lo
        STRA,R0 LNUML                    ; new var lo
        PPSL PSL_WC                      ; set WC: carry propagates into hi add
        LODA,R0 LNUMH
        ADDA,R0 EXPH                     ; hi: var_hi + step_hi + carry
        CPSL PSL_WC                      ; clear WC
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
        LODI,R0 3
        STRA,R0 GOTOFLG                  ; $03 = FOR direct NLP branch
        RETC,UN
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
        STRA,R0 FORSP
        RETC,UN
DN_POP_EMPTY:
        LODI,R0 $FF
        STRA,R0 FORSP
        RETC,UN

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
        BSTA,UN INC_IP
        LODA,R0 *IPH
        STRA,R0 EXPL
        BSTA,UN INC_IP
        BSTA,UN PRINT_S16
        BSTA,UN PRT_SPACE
DLS_BLPX:
        LODA,R0 *IPH
        COMI,R0 CR
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
        BSTA,UN INC_TMP
        LODA,R0 *TMPH
        STRA,R0 CURL
        BSTA,UN INC_TMP
        ; copy body to IBUF until CR, NUL-terminate
        BSTA,UN SET_IP_IBUF              ; IPH:IPL = IBUF
DR_CPY:
        LODA,R1 *TMPH
        COMI,R1 CR
        BCTR,EQ DR_CD
        STRA,R1 *IPH
        BSTA,UN INC_TMP
        BSTA,UN INC_IP
        BCTR,UN DR_CPY
DR_CD:
        BSTA,UN INC_TMP                  ; skip past CR in store
        EORZ,R0
        STRA,R0 *IPH                     ; NUL-terminate IBUF
        ; Save next-line pointer into SWSTK before STMT_EXEC clobbers SC0/SC1.
        ; SWSTK persists across STMT_EXEC; DO_GOSUB and DO_FOR read from it.
        LODA,R0 TMPH
        STRA,R0 SWSTK
        LODA,R0 TMPL
        STRA,R0 SWSTK+1
        ; execute line
        BSTA,UN SET_IP_IBUF              ; IPH:IPL = IBUF
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
        BCTA,UN DR_LP
DR_FORNLP:
        ; FOR/NEXT loop-back: GOTOH:GOTOL is a direct program-store address.
        EORZ,R0
        STRA,R0 GOTOFLG
        LODA,R0 GOTOH
        STRA,R0 TMPH
        LODA,R0 GOTOL
        STRA,R0 TMPL
        BCTA,UN DR_LP
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
        BCTA,UN DR_LP
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
        BSTA,UN WSKIP
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
        BSTA,UN WSKIP                    ; [+1] skip space after line number
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
        BSTA,UN INC_TMP
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
        BCFR,LT SL_ROOM
JERROOM:
        LODI,R0 ERR_OOM
        BCTA,UN DO_ERROR

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
        BCTR,EQ SL_NOSHIFT               ; src == ins: done
        BCTR,LT SL_NOSHIFT               ; BUG-LE FIX: src.lo < ins.lo: done
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
        BSTA,UN INC_EXP
        LODA,R0 LNUML
        STRA,R0 *EXPH                    ; write line lo
        BSTA,UN INC_EXP
SL_WBODY:
        LODA,R1 *TMPH
        BCTR,EQ SL_WDONE
        BSTA,UN TMP2EXP
        BCTR,UN SL_WBODY
SL_WDONE:
        LODI,R0 CR
        STRA,R0 *EXPH                    ; write CR terminator
        BSTA,UN INC_EXP
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
        BSTA,UN INC_TMP                  ; [+1] skip hi byte of line number
        BSTA,UN INC_TMP                  ; [+1] skip lo byte of line number
        ; scan body until CR to get record size in SC0
        LODI,R0 3                        ; start at 3 (hi+lo+CR)
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
        BSTA,UN INC_TMP                  ; skip CR byte itself
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
        BSTR,UN TMP2EXP
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
;  TMP2EXP -- Copy single byte: *EXP++ = *TMP++
; In:  TMPH:TMPL -> source, EXPH:EXPL -> dest
; Out: one byte copied; both pointers incremented
; Clobbers: R0, R1
TMP2EXP:
        LODA,R1 *TMPH
        STRA,R1 *EXPH
        BSTA,UN INC_TMP
        BSTA,UN INC_EXP
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
JERRLINE:
        LODI,R0 ERR_UND_LINE
        BCTA,UN DO_ERROR
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
        LODA,R0 TMPL
        ADDI,R0 2
        STRA,R0 TMPL
        TPSL $01
        BCTR,LT FI_AN
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
FI_AN:
FI_AS:
        LODA,R0 *TMPH
        COMI,R0 CR
        BCTR,EQ FI_DONE
        BSTA,UN INC_TMP
        BCTR,UN FI_AS
FI_DONE:
        BSTA,UN INC_TMP                  ; skip the CR itself
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
        ; RAS guard: inline -- no BSTA, does not consume a RAS slot.
        SPSU                             ; R0 = PSU; SP in bits 2:0
        ANDI,R0 $07                      ; isolate SP field
        COMI,R0 5                        ; threshold
        BCTR,LT PE_SAFE                  ; SP < 5: safe to proceed
        LODI,R0 ERR_NEST
        BCTA,UN DO_ERROR                 ; abort gracefully
PE_SAFE:
        LODI,R3 $FF                      ; SW stack empty sentinel
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
        CPSL PSL_WC
        LODA,R0 SAVEL
        ADDA,R0 EXPL
        STRA,R0 EXPL
        PPSL PSL_WC
        LODA,R0 SAVEH
        ADDA,R0 EXPH
        STRA,R0 EXPH
        CPSL PSL_WC
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
        CPSL PSL_WC
        LODA,R0 SAVEL
        SUBA,R0 EXPL
        STRA,R0 EXPL
        PPSL PSL_WC
        LODA,R0 SAVEH
        SUBA,R0 EXPH
        STRA,R0 EXPH
        CPSL PSL_WC
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
        BCTR,UN PARSER_RET
EAM_NEG:
        BSTA,UN INC_IP
        LODI,R0 >NEG_AT_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <NEG_AT_RET
        STRA,R0 SWBASE,R3+
        BCTR,UN EAM_ATOM
NEG_AT_RET:
        BSTA,UN NEG_EXP_BODY
        BCTR,UN PARSER_RET
EAM_POS:
        BSTA,UN INC_IP
        LODI,R0 >POS_AT_RET
        STRA,R0 SWBASE,R3+
        LODI,R0 <POS_AT_RET
        STRA,R0 SWBASE,R3+
        BCTR,UN EAM_ATOM
POS_AT_RET:
        BCTR,UN PARSER_RET
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
        BSTA,UN PARSE_S16                ; [+1]
        RETC,UN

; =============================================================================
;  PF_LOADVAR -- Load variable value from VARS
; In:  R0 = uppercase variable letter A-Z; IP -> that char
; Out: EXPH:EXPL = variable value
; Clobbers: R0, R1, SC0
PF_LOADVAR:
        STRA,R0 SC0
        BSTA,UN INC_IP
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
        BSTA,UN WSKIP                    ; [+1]
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
        BCTR,EQ PRO_NONE
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
        BSTA,UN INC_IP
        BCTR,UN PRO_LP
PRO_NONE:
        BCTA,UN JSYNERR

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
        BSTA,UN INC_IP
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
        BCTA,LT JSYNERR
        COMI,R0 A'9'+1
        BCTA,GT JSYNERR
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
        LODI,R1 4                        ; offset for EXPH/EXPL from IPH
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
        LODI,R1 2                        ; offset for TMPH/TMPL from IPH
        ; fall through to NEG_SHARED

; =============================================================================
;  NEG_SHARED -- Shared negation core (two's complement via 1s complement + INC_ET)
; In:  R1 = offset (4=EXP, 2=TMP)
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
        LODI,R1 4
        BCTR,UN NEG_SHARED

; =============================================================================
;  MUL16 -- Signed 16-bit multiply: TMPH:TMPL * EXPH:EXPL -> EXPH:EXPL
; In:  TMPH:TMPL = left operand; EXPH:EXPL = right operand
; Out: EXPH:EXPL = product (16-bit two's complement wrap)
; Clobbers: R0, NEGFLG, SC0, SC1, TMPH, TMPL
MUL16:
        EORZ,R0
        STRA,R0 NEGFLG
        BSTR,UN ABS_TMP
        BSTR,UN ABS_EXP

        LODA,R0 EXPH
        STRA,R0 SC0
        LODA,R0 EXPL
        STRA,R0 SC1
        EORZ,R0
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
        BCTA,EQ JERRDIVZER
DV_NZ:
        EORZ,R0
        STRA,R0 NEGFLG
        BSTA,UN ABS_TMP
        BSTA,UN ABS_EXP

        LODA,R0 EXPH
        STRA,R0 SC0
        LODA,R0 EXPL
        STRA,R0 SC1
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 EXPL
DV_LP:
        LODA,R0 TMPH
        SUBA,R0 SC0
        BCTR,LT MU_DONE ; DV_DONE
        BCTR,GT DV_SUB
        LODA,R0 TMPL
        SUBA,R0 SC1
        TPSL $01
        BCTR,EQ DV_SUB
        BCTA,UN MU_DONE ; DV_DONE
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
        BSTA,UN INC_EXP
        BCTR,UN DV_LP

JERRDIVZER:
        LODI,R0 ERR_DIV_ZERO
        BCTA,UN DO_ERROR

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
        BSTA,UN COUT
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
        PPSL PSL_WC
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
        CPSL PSL_WC
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
        BSTA,UN SET_IP_IBUF              ; IPH:IPL = IBUF
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
        BSTA,UN COUT
        BSTA,UN INC_IP
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
        BSTA,UN PRT_SPACE
        BSTA,UN PRT_BS
        BCTA,UN RL_LP
RL_EOL:
        EORZ,R0
        STRA,R0 *IPH
        BCTA,UN PRT_CRLF

; =============================================================================
;  WSKIP -- Skip spaces at IP
; In:  IPH:IPL -> current position
; Out: IPH:IPL -> first non-space char
; Clobbers: R0
WSKIP:
        LODA,R0 *IPH
        COMI,R0 SP
        BCTR,EQ WS_ADV
        RETC,UN
WS_ADV:
        BSTR,UN INC_IP
        BCTR,UN WSKIP

; =============================================================================
;  GETCI_UC -- Read *IPH uppercase into R0, advance IP
; In:  IPH:IPL -> current position
; Out: R0 = uppercased char; IP advanced by 1; R1 clobbered
; Clobbers: R0, R1
GETCI_UC:
        LODA,R0 *IPH
        BSTR,UN UPCASE                   ; [+1]
        STRZ,R1                          ; save before INC_IP clobbers R0
        BSTR,UN INC_IP                   ; [+1]
        LODZ,R1                          ; restore
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
        BCTR,LT UC_DO
        RETC,UN
UC_DO:
        SUBI,R0 32
        RETC,UN

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
        BCTR,EQ EW_ADV
        RETC,UN
EW_ADV:
        BSTR,UN INC_IP
        BCTR,UN EATWORD

; =============================================================================
;  SHARED 16-BIT POINTER INCREMENT/DECREMENT
; INC_EXP : EXPH:EXPL += 1   (offset 4 from IPH)
; INC_TMP : TMPH:TMPL += 1   (offset 2 from IPH)
; INC_IP  : IPH:IPL  += 1    (offset 0 from IPH)
; All share INC_ET body using register bank switch.
; Rule: NO BSTA inside these -- must not consume extra RAS depth.
; Carry idiom: ADDI R0,1: no-carry -> R0 non-zero (GT if was $01..$FE),
;   carry -> R0 wraps to $00 (EQ). TPSL $01: EQ=carry, LT=no-carry.
INC_EXP:
        LODI,R0 4               ; EXP is 4 bytes after IP
        db $EC                  ; COMA,R0 -- consume next 2 bytes (skip to INC_IP path)
INC_TMP:
        LODI,R0 2               ; TMP is 2 bytes after IP
        db $C4                  ; COMI,R0 -- consume next 1 byte
INC_IP:
        EORZ,R0                 ; offset = 0 (IPH itself)
; Can jump in here with R1 set for offset
INC_ET:
        PPSL RS                 ; switch to alternate register bank
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
        CPSL RS                 ; switch back to primary bank
        RETC,UN

; =============================================================================
;  SHARED 16-BIT POINTER DECREMENT -- DEC_ET family
; DEC_LNUM: LNUMH:LNUML -= 1   (offset 12 from IPH/$1600)
; DEC_GOTO:  GOTOH:GOTOL -= 1   (offset  8 from IPH/$1600)
; DEC_IP:    IPH:IPL    -= 1    (offset  0 from IPH/$1600)
; All share DEC_ET body via register bank switch, mirroring INC_ET.
; Byte-skip chain: $EC (COMA, skips 2) and $C4 (COMI,R0, skips 1).
; Borrow: after SUBI R0,1 -- R0 was 0 -> result $FF, CC=LT (borrow).
;   BCFR,LT branches when CC != LT (no borrow) -- skip hi decrement.
;   Saves 1 byte vs TPSL $01 / RETC,EQ idiom used in INC_ET.
; BUG-DEC-01 FIX retained: borrow detected via carry (BCFR,LT), not sign.
; RAS rule: NO BSTA inside body -- must not consume extra depth.
; DEC_EXP/DEC_TMP omitted: MUL16 call site is at RAS depth 5+1=6 (unsafe).
DEC_LNUM:
        LODI,R0 12              ; LNUMH is 12 bytes after IPH ($160C-$1600)
        db $EC                  ; COMA,R0: skip next 2 bytes (the LODI,R0 8)
DEC_GOTO:
        LODI,R0 8               ; GOTOH is 8 bytes after IPH ($1608-$1600)
        db $C4                  ; COMI,R0: skip next 1 byte (the EORZ,R0)
DEC_IP:
        EORZ,R0                 ; offset = 0 (IPH:IPL)
DEC_ET:
        PPSL RS                 ; switch to alternate register bank
        STRZ R1                 ; R1 = offset
        LODA,R0 IPL,R1          ; load lo byte
        SUBI,R0 1
        STRA,R0 IPL,R1
        BCFR,LT ET_RET      ; CC != LT -> no borrow: hi unchanged
        LODA,R0 IPH,R1          ; borrow: decrement hi byte
        SUBI,R0 1
        BCTR,UN ET_STORE        ; borrow tail

; =============================================================================
; DO_FREE
; Syntax: FREE
; Prints the number of free bytes in program store: PROGLIM - PEH:PEL.
; PROGLIM = $1FFF (top of RAM). Free = $1FFF - current program end pointer.
; In:  PEH:PEL = program end pointer
; Out: free byte count printed to COUT followed by CR/LF
; Clobbers: R0, EXPH, EXPL (via PRINT_S16)
DO_FREE:
        ; compute EXPH:EXPL = $1FFF - PEH:PEL
        LODI,R0 >PROGLIM
        SUBA,R0 PEL                      ; lo = $FF - PEL
        STRA,R0 EXPL                     ; PSL_C: 1=no-borrow, 0=borrow
        LODI,R0 <PROGLIM                      ; hi base ($1FFF hi byte)
        TPSL $01                         ; EQ=no-borrow(C=1), LT=borrow(C=0)
        BCTR,EQ DF_NB                    ; no borrow: use $1F as-is
        SUBI,R0 1                        ; borrow: hi = $1E
DF_NB:
        SUBA,R0 PEH                      ; hi = $1F/$1E - PEH
        STRA,R0 EXPH
        BSTA,UN PRINT_S16                ; [+1] print decimal
PRT_CRLF:
        BSTR,UN PRT_CR
        ; drop through

; =============================================================================
;  Shared character print routines -- $EC (COMA) byte-skip chain
; Each entry loads its character then falls through via the skip trick.
; PRT_QUEST is the end of chain: uses BCTA to COUT (tail call).
PRT_LF:
        LODI,R0 LF
        db $EC
PRT_BS:
        LODI,R0 BS
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to next LODI
PRT_CR:
        LODI,R0 CR
        db $EC
PRT_CHEV:
        LODI,R0 '>'
        db $EC
PRT_SPACE:
        LODI,R0 32
        db $EC
PRT_AT:
        LODI,R0 '@'
        db $EC
PRT_QUEST:
        LODI,R0 '?'
        BCTA,UN COUT

; =============================================================================
;  DO_ERROR -- Print error, clear run state, return to REPL
; Entry: R0 = error code character ('0'..'8').
; Clears RUNFLG, SWSP, FORSP. Prints "?n" or "?n @ line" if running.
; Tail-jumps to REPL (clears full hardware RAS).
; In:  R0 = error code
; Out: jumps to REPL
; Clobbers: all (RAS cleared by BCTA REPL)
DO_ERROR:
        STRA,R0 SC0                      ; save error code
        LODA,R0 RUNFLG
        STRA,R0 SC1                      ; save run state
        BSTA,UN CLR_RUNFLG
        LODI,R0 $FF
        STRA,R0 SWSP
        STRA,R0 FORSP
        BSTR,UN PRT_QUEST
        LODA,R0 SC0
        BSTA,UN COUT
        LODA,R0 SC1                      ; OPT-10: SC1=RUNFLG, 0->EQ, 1->GT
        BCTR,GT DE_IN
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
        BSTA,UN PRT_CRLF
        BCTA,UN REPL                     ; kills full hardware RAS

; =============================================================================
;  TABLES -- Keyword dispatch table
; Format: [c1][c2][c3][hi][lo]  NUL-terminated on c1.
; hi:lo = handler address. Matched on first three uppercase chars.
; c3=A' ' (space) = wildcard (IF -- only 2 chars before body).
; THEN matched internally by DO_IF -- not dispatched here.
KW_TAB:
        DB A'P',A'R',A'I', <DO_PRINT,  >DO_PRINT   ; PRINT
        DB A'L',A'E',A'T', <DO_LET,    >DO_LET     ; LET
        DB A'L',A'I',A'S', <DO_LIST,   >DO_LIST    ; LIST
        DB A'R',A'E',A'M', <DO_REM,    >DO_REM     ; REM
        DB A'R',A'E',A'T', <DO_RETURN, >DO_RETURN  ; RETURN
        DB A'R',A'U',A'N', <DO_RUN,    >DO_RUN     ; RUN
        DB A'E',A'N',A'D', <DO_END,    >DO_END     ; END
        DB A'I',A'N',A'P', <DO_INPUT,  >DO_INPUT   ; INPUT
        DB A'I',A'F',A' ', <DO_IF,     >DO_IF      ; IF (wildcard)
        DB A'N',A'E',A'W', <DO_NEW,    >DO_NEW     ; NEW
        DB A'G',A'O',A'T', <DO_GOTO,   >DO_GOTO    ; GOTO
        DB A'G',A'O',A'S', <DO_GOSUB,  >DO_GOSUB   ; GOSUB
        DB A'F',A'O',A'R', <DO_FOR,    >DO_FOR     ; FOR
        DB A'N',A'E',A'X', <DO_NEXT,   >DO_NEXT    ; NEXT (c3='X' vs NEW c3='W')
        DB "FRE", <DO_FREE, >DO_FREE               ; FREE
        DB NUL



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

ROMEND: ; measure interpreter size: ROMEND-$0440

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
        ORG PROG

        DB 0,10,"REM uBASIC 2650 - SHOWCASE V3.6",$0D                          ; 10  REM uBASIC 2650 - SHOWCASE V3.6
        DB 0,20,"PRINT ",$22,"-- uBASIC 2650 V3.6 Showcase --",$22,$0D         ; 20  PRINT "-- uBASIC 2650 V3.6 Showcase --"
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
        DB $0D, $0D, $0D, $0D

        END
