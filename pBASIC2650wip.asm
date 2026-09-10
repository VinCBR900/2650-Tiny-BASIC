; pBASIC2650.asm - PoC Minimal Tiny BASIC for Signetics 2650
; v0.21 - Sep 2026
; Vincent Crabtree - MIT License
;
; TARGET
;   Standalone Signetics 2650, ORG 0, 8 KB address space.
;   I/O routines embedded; no PIPBUG ROM required.
;   Goal: <2 KB ROM at $0000, RAM at $1000 upwards.
;
; BUILD
;   gcc -Wall -O2 -o asm2650 asm2650.c
;   gcc -O2 -DGAMER -o pipbug_wrap pipbug_wrap.c
;
;   ./asm2650 pBASIC2650.asm pBASIC2650.hex
;   grep -n "^CHIN \|^COUT " pBASIC2650.LST
;   ./pipbug_wrap --entry 0 --chin 0x<addr> --cout 0x<addr> pBASIC2650.hex
;
; 2650 FLAGS / COMPARES
;   SUB/ADD: CC = LT, EQ or GT from the signed 8-bit result.
;   Carry (PSL bit 0): C=1 = no borrow / carry, C=0 = borrow.
;   Carry test: TPSL $01 -> EQ if C=1, LT if C=0.
;   Unsigned compare: COMA.  COM mode is set once at boot and left set.
;   All COMI inputs here are 0-127 values.
;
; 16-BIT CONVENTION
;   <$ADDR = high byte
;   >$ADDR = low byte
;   Example: <$1634 = $16, >$1634 = $34
;
; REGISTER CONVENTIONS
;   R0  general working register / arithmetic / I/O
;   R1  index register; also used by PRINT_S16
;   R2  current variable letter; normally preserved across expressions
;   R3  loop counter / software expression-depth state
;
; RAS / RECURSION
;   The 2650 has an 8-level hardware return-address stack, not user accessible.
;   BSxx/ZBSR consume a RAS entry; BCxx/ZBRR do not.
;   PARSE_EXPR guards against excessive hardware-stack depth.
;   Parentheses use a software depth counter to reduce RAS usage.
;
; BASIC LANGUAGE
;   26 uppercase variables (A-Z), 16-bit signed integers.
;   No arrays or string variables.
;   PRINT supports string literals.
;   No built-in functions.
;   Operators: + - * / = <
;   '!' prefix inverts != for not-equal, !< for not-less-than (>=). All six relops
;     are available by combining reversed operands (A>B is B<A; A<=B is B!<A).
;
;        KNOWN LIMITATIONS
;
; UPPERCASE only apart from PRINT "String literals"
;
; All operators have equal precedence and evaluate left-to-right. So 
;   "1+2*3" evaluates as "(1+2)*3" = 9, not 7.
;   Use parentheses for grouping Max 3 level nesting ((()))
;   Uppercase only outside string literals.
;
; STATEMENT DISPATCH matches ONLY the first character of a line against
;   a 9-entry table (A/E/G/I/L/N/P/R/W for ASK/END/GOTO/IF/LIST/NEW/
;   PRINT/RUN/WR. It first checks the 2nd is a LETTER otherwise VAR.
;
; STORAGE / INPUT
;   Program lines are append-only.
;   A new line number must be greater than the current highest,
;   or equal to it to replace/delete the last line.
;   Input is not bounds-checked; overlong lines will corrupt memory.
;
; DELIBERATE LIMITS
;   Parenthesis nesting is RAS-limited; deepest guaranteed safe level
;   depends on call context. Excess depth generates ERR_NEST.
;   No BODMAS/PEMDAS precedence - left to right, 1+2*3" evaluates
;     as "(1+2)*3" = 9, not 7. Use explicit parens.
;   Minimal syntax validation; malformed constructs may produce the
;   normal syntax/runtime error rather than a specialised diagnostic.
;
; =============================================================================
; VERSION HISTORY (pBASIC2650)
; =============================================================================
;
; v0.21 (Sep 2026) - Refactor OPS_LP for size.
;   - ROMEND: $0774 (1908 bytes).
; v0.20 (Sep 2026)
;   - SUBA/COMA audit - set COM=1 once in MAIN to eliminate redundant PPSL/CPSL loads
;   - ROMEND: $07D3 -> $07C4 (-15 bytes).
; v0.19 (Sep 2026)
;   - Refactor STMT_EXEC for size.
;     Fixed DO_LTOP '<' comparison bug caused by raw CC range limits
;   - ROMEND: $07CB -> $07D3 (+8 bytes).
; v0.18 (Sep 2026)
;   - Added '!' relop-invert modifier (!=, !<) using a BANG flag and XORing
;     into the boolean result at convergence.
;   - ROMEND: $07BC -> $07D6 (+26 bytes).
; v0.17 (Sep 2026)
;   - Factored out WSKIP_PEEK subroutine (9 call sites) and ADV_PAST_RECORD
;     subroutine (2 call sites).
;   - ROMEND: $07D7 -> $07BC (-27 bytes).
; v0.16 (Sep 2026)
;   - Code golf pass: collapsed branch-to-return into RETC,EQ and removed
;     redundant whitespace skips.
;   - ROMEND: $07DA -> $07D7 (-3 bytes).
; v0.15 (Sep 2026)
;   - Added software paren-nesting tracker (PDEPTH) to eliminate hardware call
;     frame consumption in EA_PAREN, allowing deeper expression nesting.
;   - Repurposed dead TEMPRETH/TEMPRETL RAM cells for PDEPTH.
;   - ROMEND: $07C5 -> $07DA (+21 bytes).
; v0.14 (Sep 2026)
;   - Optimized branch elimination (RETC,LT collapses) and converted keyword
;     dispatch (MD_SCAN/MD_HIT) to use direct register-indexed addressing on TOK_CHARS.
;   - ROMEND: $07C5 (1989 bytes).
; v0.13 (Sep 2026)
;   - Removed UPCASE case-folding entirely; forced strict uppercase syntax.
;   - ROMEND: $07F5 -> $07D7 (-30 bytes).
; V0.12 (Sep 2026)
;   - Code golf pass. ROMEND: $7FB.
; v0.11 (Sep 2026)
;   - Fixed -32768 overflow bugs in MUL16 (MU_LP updated to BCFR,EQ) and
;     DIV16 (DV_LP updated to use COMA,R0 SC0).
;   - ROMEND: $0825 -> $082A (+5 bytes).
; v0.10 (Sep 2026)
;   - Fixed MUL16 loop counter decrement bug (replaced faulty BCFR,LT borrow test).
; v0.9 (Sep 2026) - PRINT_S16 replaced with a flat power-of-10 loop
;   - ROMEND $08C1 -> $0823 (2241 -> 2083 bytes)
; v0.8 (Sep 2026) - Size pass: CMP_TMP_PE extraction, PRT_BS removal
;   - ROMEND $090D -> $08C1
; v0.7 (Sep 2026) - PAREN-NEST-02: fix wrong-operator bug in EXPR
;   - ROMEND $08F3 -> $090D
; v0.6 (Sep 2026) - Delete GETCI_UC, TMP_TO_EXP16, RND_SHUFFLE/VRND_SHUFFLE/RNDSEED
;   - Inlined PUSH_RET and IP_TO_TMP    
; v0.5 (Aug 2026) - Flatten precedence
;   - Replaced PARSE_EXPR/EAM_ATOM/EAM_HI/EAM_LO_LOOP + SW- trampolining 
;     (PUSH_RET/PARSER_RET/SWRETURN dance) with flat EXPR/EXPR_ATOM/EXPR_LOOP: all 
;     operators (+-*/=<) are at one precedence, left to right: "1+2*3" = "(1+2)*3".
;   - ROM: ROMEND $0AE6 (2790) -> $093D (2365 bytes)
; v0.4 (Aug 2026) - Stage 4: narrow relops to = and <, plus a golf pass
;   - PARSE_RELOP: Now matches only '=' or '<' directly.
;   - ROM: ROMEND $0B00 (2816) -> $0AE6 (2790 bytes)
; v0.3 (Aug 2026) - Stage 3: single-char + letter statement dispatch
;   - Replaced KW_TAB's 2-3 char match (MATCH_KW, stride 5) with TOK_CHARS
;   - Deleted LET, REM, THEN
;   - ROM: ROMEND $0B17 (2839) -> $0B00 (2816 bytes)
; v0.2 (Aug 2026) - Stage 2: Implimented minimal line handling
;   - Append-only BASIC line handling - line number is accepted only if greater than
;     all  stored lines (append) or exactly equal to the current LAST line (in-place
;     replace, or delete on an empty body). Anything else is a syntax error.
;   - Removed STORE_LINE, DELETE_LINE, MEMCPY, and DEC_LNUM/DEC_GOTO.
;   - ROM: ROMEND $0BD9 (3033) -> $0B18 (2840 bytes)
; v0.1 (Aug 2026) - Initial port from uBASIC2650, inspired by pBASIC65c02.
;   - Cut statements and functions, cleaned showcase.
;   - Removed PRINT's CHR$(n)/TAB(n)/HEX$(n) 
;   - Added DO_WR: `WR expr` equivalent of `PRINT CHR$(n);`
; =============================================================================

;  ASCII Defines
CR      EQU     $0D
LF      EQU     $0A
SP      EQU     $20
NUL     EQU     $00
DQ      EQU     $22

;  ERROR Defines
ERR_SYN         EQU '0'
; ERR_UND_LINE    EQU '1'         ; unused
ERR_DIV_ZERO    EQU '2'
; ERR_OOM         EQU '3'       ; Unused
ERR_VAR         EQU '4'
ERR_NEST        EQU '8'         ; Expression nesting too deep (RAS guard, v3.2 had '5')

; Error check to make sure we dont crash due to HW Stack overflow 
PE_RAS_LIMIT    EQU 7

; PSW Defines
PSW_RS          EQU     $10
PSW_WC          EQU     $08             ; WC (With Carry) bit in PSL (bit 3)
PSW_FLAG        EQU     $40

;  CODE starts at Zero (No Pipbug)
        ORG 0

; =============================================================================
;  RESET / ENTRY + PAGE-ZERO VECTOR TABLE
; In:  nothing (cold start)
; Out: nothing
;
; Page-zero subroutine vector table.
; Each DW entry holds the absolute address of the subroutine.
; Callers use ZBRR/ZBSR *Vxxx (2 bytes) vs BCTA/BSTA,UN xxx (3 bytes)
;
RESET:
        BCTR,UN MAIN            ; trampoline over vector table ($0000)
VINC_IP:
        DW INC_IP               ; 28 sites
VWSKIP:
        DW WSKIP                ; 23 sites
VINC_TMP:
        DW INC_TMP              ; 23 sites
VCOUT:
        DW COUT                 ; 10 sites
VPARSE_EXPR:
        DW EXPR                 ; 19 sites (v0.5: was PARSE_EXPR)
VEATWORD:
        DW EATWORD              ; 7 sites
VSET_IP_IBUF:
        DW SET_IP_IBUF          ; 4 sites
VPRT_SPACE:
        DW PRT_SPACE            ; 4 sites (v0.1c: was 5 - TAB()'s call went
VCLR_EXP:
        DW CLR_EXP              ; 6 sites (v2.0: MUL16/DIV16 each call
                                 ; it directly now, was 1 shared call via SETUP_MULDIV)
VDO_ERROR:
        DW DO_ERROR             ; 3 sites
VJSYNERR:
        DW JSYNERR              ; multiple sites
VDR_LP:
        DW DR_LP                ; 3 sites
VCLR_RUNFLG:
        DW CLR_RUNFLG           ; 3 sites
VEXP16_TO_LNUM:
        DW EXP16_TO_LNUM        ; 4 sites (was 5; DO_LIST now sets LNUM
VSET_TMP_PROG:
        DW SET_TMP_PROG

MAIN:
       ; 10 bytes - Delete for ROM 
        LODI,R0 <SHOWCASE_END
        STRA,R0 PEH
        LODI,R0 >SHOWCASE_END
        STRA,R0 PEL

        PPSL $02                ; COM=1 (unsigned compare mode) for the entire

        ; clear flags - change to DO_NEW for ROM
        BSTR,UN DO_END          

        ; print sign-on banner
        LODI,R0 <BANNER
        STRA,R0 IPH
        LODI,R0 >BANNER
        STRA,R0 IPL
        BSTA,UN PRTSTR
        ; fall through to REPL

; =============================================================================
;  REPL -- Main read-eval-print loop
; In:  nothing
; Out: loops forever
; Clobbers: all
REPL:
        CPSL PSW_RS + 5             ; primary reg bank; clear C/OVF/RS -
        CPSU $07                    ; clear PSU SP field (bits 2:0 = HW RAS depth)
        LODI,R0 '>'                    ; print prompt only used here
        ZBSR *VCOUT  
        ZBSR *VPRT_SPACE  
        BSTA,UN GETLINE
        ZBSR *VSET_IP_IBUF                ; IPH:IPL = IBUF
        BSTA,UN TRY_STORE_LINE           ; CC=GT: line stored/deleted; CC=EQ: not a line
        BSTR,EQ STMT_EXEC               ; If CC=EQ (not a line), execute
        BCTR,UN REPL

; =============================================================================
;  DO_NEW -- Reset PE and IP
; Syntax: NEW
; In:  nothing
DO_NEW:
        ; set both PEH:PEL and IPH:IPL to PROG in one pass
        LODI,R0 <PROG
        STRA,R0 PEH
        STRA,R0 IPH
        LODI,R0 >PROG
        STRA,R0 PEL 
        STRA,R0 IPL
        ; fall through to DO_END
; =============================================================================
;  DO_END -- Stop execution and clear all run state
; Syntax: END  (also called by DO_NEW, DO_ERROR, RESET)
; In:  nothing
; Out: GOTOFLG=0, RUNFLG=0
; Clobbers: R0
DO_END:
        EORZ,R0
        STRA,R0 GOTOFLG
        ZBRR *VCLR_RUNFLG               ; tail call

; =============================================================================
;  DO_IF -- Conditional execution (v0.5: THEN is gone - relops are now
;  flat operators inside EXPR itself, so the expression's own result IS
;  the whole truth value. No separate relop parse, no THEN keyword.
;  Nests for free: "IF a IF b stmt" - the true-path dispatch is a JUMP to
;  STMT_EXEC, so if "stmt" is itself another IF, it costs no extra depth.)
; Syntax: IF expr stmt
; In:  IP -> first char after IF keyword
; Out: executes stmt if expr is nonzero; otherwise sequential return
; Clobbers: R0, R1, EXPH, EXPL (via EXPR, plus whatever the dispatched
;           statement clobbers on the true path)
DO_IF:
        BSTA,UN EXPR                      ; [+1] condition -> EXPH:EXPL
        LODA,R0 EXPL
        IORA,R0 EXPH
        RETC,EQ                           ; both zero: false, sequential return
        ; drop through
; =============================================================================
;  STMT_EXEC -- Decode and dispatch one BASIC statement from IP.
; In:  IPH:IPL -> first char of statement (after any leading whitespace)
; Out: control jumps to the matched DO_xxx handler, or falls into SE_NOTKW
; Clobbers: R0, R2, R3, GOTOH, GOTOL, IPH:IPL advanced past the keyword
;   on a match (unchanged on the bare-assignment path - see SE_NOTKW)
; RAS depth: 1 from REPL, 3 from DO_IF(THEN body).
STMT_EXEC:
        ZBSR *VWSKIP  
        ; peek 2nd char (IP+1) without consuming - v2.0: indexed peek via R3
        LODI,R3 1
        LODA,R0 *IPH,R3                    ; peek 2nd char
        COMI,R0 A'A'
        BCTR,LT SE_NOTKW                  ; not a letter: bare assignment
        COMI,R0 A'Z'+1
        BCTR,GT SE_NOTKW                  ; not a letter: bare assignment

        LODA,R0 *IPH,R3-                   ; peek 1st char; R3 pre-decrements
        STRZ,R2                          ; cache target char in R2 (1 byte,
MD_SCAN:
        LODA,R0 TOK_CHARS,R3              ; table char
        BCTR,EQ SE_NOTKW                  ; NUL row: no match -> bare assignment
        COMZ,R2                          ; 1 byte vs SUBA's 3 (r0:r2 -> CC);
        BCTR,EQ MD_HIT
        ADDI,R3 3                         ; next row (char + 2-byte handler)
        BCTR,UN MD_SCAN
MD_HIT:
        ZBSR *VEATWORD                    ; [+1] consume the whole keyword -
                                          ; clobbers R0 only, R3 survives
        LODA,R0 TOK_CHARS,R3+              ; handler hi (pre-inc: char->hi)
        STRA,R0 GOTOH
        LODA,R0 TOK_CHARS,R3+              ; handler lo (pre-inc: hi->lo)
        STRA,R0 GOTOL
        BCTA,UN *GOTOH                    ; indirect jump

SE_NOTKW:
        ; Bare variable assignment ("X=expr" - either the 2nd-char peek
        ; above wasn't a letter, or the 1st char matched no statement).
        BSTA,UN PARSE_VAR_SAVE            ; validates A-Z, SC0/R2 = letter, IP -> past it
        BSTA,UN WSKIP_PEEK
        COMI,R0 A'='
        BCFA,EQ JSYNERR
        ZBSR *VINC_IP
        ; drop through
; =============================================================================
;  DL_EX / DL_STORE -- Variable assignment (v0.3: DO_LET's own prologue,
;  reachable only via the explicit "LET" keyword, is gone along with LET
;  itself - see STMT_EXEC. This is now reached only from SE_NOTKW's bare
;  "V=expr" path and from DO_ASK.)
; In:  IP -> expression (DL_EX) or SC0/R2 = var letter, EXPH:EXPL = value (DL_STORE)
; Out: VARS[V] = EXPH:EXPL
; Clobbers: R0, R1, EXPH, EXPL, TMPH, TMPL (via PARSE_EXPR, DL_EX only)
DL_EX:
        ZBSR *VPARSE_EXPR                 ; [+1]
DL_STORE:
        LODZ,R2          ; R0 = R2 (Variable character letter)
        SUBI,R0 A'A'     ; R0 = R0 - 'A' (0 to 25)
        ADDZ,R0          ; R0 = R0 * 2 (Double for 16-bit word stride)
        STRZ,R1          ; R1 = R0 (Transfer offset to Index Register R1)
        LODA,R0 EXPH     ; R0 = High byte of expression
        STRA,R0 VARS,R1  ; Store directly to VARS array + offset
        LODA,R0 EXPL     ; R0 = Low byte of expression
        STRA,R0 VARS+1,R1; Store directly to VARS array + offset + 1
        RETC,UN

; =============================================================================
;  DO_ASK -- Read signed integer from user into variable (v0.1b: renamed
;  from INPUT - I is taken by IF under 1-char statement dispatch)
; Syntax: ASK V
; In:  IP -> variable letter
; Out: VARS[V] = parsed value
; Clobbers: R0, R2, SC0, SC1, EXPH, EXPL, TMPH, TMPL, IBUF
DO_ASK:
        BSTR,UN PARSE_VAR_SAVE
        BSTA,UN PRT_QUEST
        ZBSR *VPRT_SPACE  
        BSTR,UN GETLINE                   ; [+1]
        ZBSR *VSET_IP_IBUF                ; IPH:IPL = IBUF
        BSTA,UN PARSE_S16                ; [+1]
        BCTR,UN DL_STORE

; =============================================================================
;  DO_GOTO -- Computed GOTO
; Syntax: GOTO expr
; In:  IP -> first char after GOTO keyword
; Out: GOTOH:GOTOL = target line; GOTOFLG=$01
; Clobbers: R0, EXPH, EXPL, GOTOH, GOTOL, GOTOFLG
DO_GOTO:
        ZBSR *VWSKIP  
        ZBSR *VPARSE_EXPR                 ; [+1]
        BSTA,UN EXP16_TO_GOTO             ; GOTOH:GOTOL = EXPH:EXPL
        LODI,R0 1
        STRA,R0 GOTOFLG
        LODA,R0 RUNFLG                   ; OPT-10
        RETC,GT                          ; return if running
        ZBRR *VCLR_RUNFLG 

; =============================================================================
;  SET_TMP_PROG -- Set TMPH:TMPL = PROG base address
; Clobbers: R0
SET_TMP_PROG:
        LODI,R0 <PROG
        STRA,R0 TMPH
        LODI,R0 >PROG
        STRA,R0 TMPL
        RETC,UN

; =============================================================================
; PARSE_VAR_SAVE -- skip whitespace, read var letter, range-check,
;                   save to SC0 and R2, advance IP.
; Out: SC0=R2=letter (A-Z); IP advanced past letter
; Error: tail-jumps to JERRVAR (no return)
; Clobbers: R0, R1, R2, SC0
PARSE_VAR_SAVE:
        BSTA,UN WSKIP_PEEK
        COMI,R0 A'A'
        BCTA,LT JERRVAR       ; out of range low  -- tail jump, no return
        COMI,R0 A'Z'+1
        BCFA,LT JERRVAR       ; out of range high -- tail jump, no return
        STRA,R0 SC0
        STRZ,R2                          ; save in R2 for DL_STORE
        ZBRR *VINC_IP           ; tail call  

; =============================================================================
;  GETLINE -- Minimal read a line from input into IBUF (v0.2, was RDLINE)
; In:  nothing
; Out: IBUF = NUL-terminated input line.
;      R3 is used as an index into IBUF (not IPH:IPL - both callers re-point
;      IP via VSET_IP_IBUF immediately after calling this). R3=$FF means
;      empty (matches the SWBASE convention) since the 2650's ",R3+"
;      addressing mode pre-increments before the access.
; Clobbers: R0, R1, R3
GETLINE:
        LODI,R3 $FF                      ; R3 = empty-buffer sentinel (pre-inc convention)
GL_LP:
        BSTR,UN CHIN                     ; [+1] blocking read
        STRZ,R1
        COMI,R1 CR
        BCTR,EQ GL_EOL
        COMI,R1 LF
        BCTR,EQ GL_EOL
        LODZ,R1                          ; R0 = char (indexed autoinc store only works for R0)
        STRA,R0 IBUF,R3+                 ; R3++ (pre-inc); IBUF[R3]=char
        ZBSR *VCOUT  
        BCTR,UN GL_LP
GL_EOL:
        EORZ,R0
        STRA,R0 IBUF,R3+                 ; R3++ (pre-inc, one past last char); NUL-terminate
        BCTA,UN PRT_CRLF                ; tail call

; =============================================================================
; Character IO
; 110 Baud teletype from PIPBUG V1 as per Signetics M20 application note
; v0.5: ORG $286 pin removed - CHIN/COUT no longer forced to PIPBUG-
; compatible addresses. CHIN/COUT now float to
; wherever they land; their real addresses must be read from the .LST
; file and passed to pipbug_wrap's --chin/--cout flags for testing,
; since the old fixed 0x286/0x2B4 no longer apply.
CHIN:
        PPSL PSW_RS
        LODI,R0 $80
;        WRTC,R0        ; make space for shuffle
        LODI,R1 0
        LODI,R2 8
;ACHI:   
        SPSU
        BCTR,LT CHIN
        EORZ,R0
;        WRTC,R0        ; make space for shuffle
        BSTR,UN DLY
BCHI:
        BSTR,UN DLAY
        SPSU
        ANDI,R0 $80
        RRR,R1
        IORZ,R1
        STRZ,R1
        BDRR,R2 BCHI
        BSTR,UN DLAY
        ANDI,R1 $7f
        LODZ,R1
        CPSL PSW_RS + PSW_WC
        RETC,UN
; Delay for 1 bit time
DLAY:
        EORZ,R0
        BDRR,R0 $
        BDRR,R0 $
DLY:
        BDRR,R0 $
        LODI,R0 $e5
        BDRR,R0 $
        RETC,UN

COUT:
        PPSL PSW_RS
        PPSU PSW_FLAG
        STRZ,R2
        LODI,R1 8
        BSTR,UN DLAY
        BSTR,UN DLAY
        CPSU PSW_FLAG
ACOU:
        BSTR,UN DLAY
        RRR,R2
        BCTR,LT ONE
        CPSU PSW_FLAG
ONE:
        PPSU PSW_FLAG
;ZERO:
        BDRR,R1 ACOU
        BSTR,UN DLAY
        PPSU PSW_FLAG
        CPSL PSW_RS
        RETC,UN

; =============================================================================
;  WSKIP_PEEK -- Skip whitespace, then peek the current char at IP into R0
; Out: R0 = *IPH; CC set by that load (EQ if NUL)
; Clobbers: R0
WSKIP_PEEK:
        ZBSR *VWSKIP
        LODA,R0 *IPH
        RETC,UN

; =============================================================================
;  DO_PRINT / PRTSTR -- Print statement and NUL-terminated string helper
; Syntax: PRINT [item {; item}]
;   item = "string" | expr
; In:  IP -> first char after PRINT keyword
; Out: text written to COUT; IP advanced past statement
; Clobbers: R0, R1, EXPH, EXPL, TMPH, TMPL, NEGFLG, LNUMH, LNUML, SC0, SC1
DO_PRINT:
        BSTR,UN WSKIP_PEEK
        BCTR,EQ DP_NL

DP_ITEM:
        BSTR,UN WSKIP_PEEK
        COMI,R0 DQ
        BCTR,EQ DP_STRING
        ; Expression
        ZBSR *VPARSE_EXPR  
        BSTA,UN PRINT_S16
        BCTR,UN DP_SEP

DP_SEP:
        BSTR,UN WSKIP_PEEK
        COMI,R0 $3B             ; semicolon
        BCTR,EQ DP_SEMI
        ; fall through to DP_NL
DP_NL:
        BCTA,UN PRT_CRLF          ; tail call: return from DO_PRINT

DP_SEMI:
        ZBSR *VINC_IP  
        BSTR,UN WSKIP_PEEK
        RETC,EQ                 ; bail if NUL
        BCTR,UN DP_ITEM

DP_STRING:
        ZBSR *VINC_IP  
PRTSTR:
        LODA,R0 *IPH
        RETC,EQ                 ; NUL before closing ": bail
        COMI,R0 DQ
        BCTR,EQ DP_SCLS
        ZBSR *VCOUT  
        ZBSR *VINC_IP  
        BCTR,UN PRTSTR

DP_SCLS:
        ZBSR *VINC_IP  
        BCTR,UN DP_SEP

; =============================================================================
;  DO_WR -- Write raw character (v0.1c: replaces CHR$, which is cut. This is
;  a statement, not a PRINT item - consecutive WRs need no PRINT/semicolon
;  wrapper the way "PRINT CHR$(a);CHR$(b);" did.)
; Syntax: WR expr
; In:  IP -> first char after WR keyword
; Out: low byte of expr's value written to COUT (no newline)
; Clobbers: R0, EXPH, EXPL, TMPH, TMPL, NEGFLG, LNUMH, LNUML, SC0, SC1
DO_WR:
        ZBSR *VWSKIP  
        ZBSR *VPARSE_EXPR  
        LODA,R0 EXPL
        ZBRR *VCOUT              ; tail call

; =============================================================================
;  CMP_TMP_PE -- Compare TMPH:TMPL against PEH:PEL (16-bit, byte-serial,
;  proper unsigned semantics via carry rather than SUBA's own CC, which is
;  only reliable over half the 0-255 byte range - see the TPSL $01 note
;  below). Factored out of six near-identical inlined copies (DR_LP,
;  TRY_STORE_LINE x2, FIND_LINE, FIND_INS, DO_LIST) - v0.7 size pass.
; Out: CC=GT if TMPH:TMPL >  PEH:PEL (hi bytes differed)
;      CC=LT if TMPH:TMPL <  PEH:PEL (hi bytes differed, or hi equal and
;            the lo-byte subtraction borrowed)
;      CC=EQ if hi bytes equal and lo bytes did not borrow (TMPL>=PEL -
;            ambiguous between "exactly equal" and "TMPL>PEL"; every call
;            site already only relied on this same ambiguous EQ meaning,
;            since TMP is guaranteed <= PE by construction wherever it's
;            used, so the two coincide in practice)
; Clobbers: R0
CMP_TMP_PE:
        LODA,R0 TMPH
        SUBA,R0 PEH
        RETC,GT                                   ; (GT/LT) is already the answer
        RETC,LT
        ; low byte
        LODA,R0 TMPL
        SUBA,R0 PEL
        TPSL $01                          ; carry-based unsigned compare:
        RETC,UN                           ; C=1 (no borrow) -> EQ, C=0 -> LT

; =============================================================================
;  DO_RUN -- Execute stored program
; Syntax: RUN
; In:  PROG=program base, PEH:PEL=program end
; Out: runs until END, error, or exhausted; returns to REPL
; Clobbers: all
; GOTOFLG after STMT_EXEC: $00=sequential, $01=GOTO.
DO_RUN:
        LODI,R0 1
        STRA,R0 RUNFLG
        EORZ,R0
        STRA,R0 GOTOFLG
        ZBSR *VSET_TMP_PROG
DR_LP:
        LODA,R0 RUNFLG
        RETC,EQ
        ; end of program? TMPH:TMPL >= PEH:PEL
        BSTR,UN CMP_TMP_PE
        BCTA,GT DR_STOP
        RETC,EQ

        ; save current line number for error reporting
        LODA,R0 *TMPH
        STRA,R0 CURH
        ZBSR *VINC_TMP  
        LODA,R0 *TMPH
        STRA,R0 CURL
        ZBSR *VINC_TMP  
        ; copy body to IBUF until CR, NUL-terminate
        ZBSR *VSET_IP_IBUF                ; IPH:IPL = IBUF
DR_CPY:
        LODA,R1 *TMPH
        COMI,R1 CR
        BCTR,EQ DR_CD
        STRA,R1 *IPH
        ZBSR *VINC_TMP  
        ZBSR *VINC_IP  
        BCTR,UN DR_CPY
DR_CD:
        ZBSR *VINC_TMP                    ; skip past CR in store
        EORZ,R0
        STRA,R0 *IPH                     ; NUL-terminate IBUF
        ; Save next-line pointer into SWSTK before STMT_EXEC clobbers SC0/SC1.
        LODA,R0 TMPH
        STRA,R0 SWSTK
        LODA,R0 TMPL
        STRA,R0 SWSTK+1
        ; execute line
        ZBSR *VSET_IP_IBUF                ; IPH:IPL = IBUF
        BSTA,UN STMT_EXEC                ; [+1]
        ; dispatch on GOTOFLG
        LODA,R0 GOTOFLG
        BCTR,EQ DR_SEQ                   ; $00: sequential
        BCTR,UN DR_GOTO                  ; $01: GOTO (only remaining setter)
DR_SEQ:
        LODA,R0 SWSTK
        STRA,R0 TMPH
        LODA,R0 SWSTK+1
        STRA,R0 TMPL
        ZBRR *VDR_LP 
DR_GOTO:
        ; GOTOFLG=$01 (GOTO).
        EORZ,R0
        STRA,R0 GOTOFLG
        LODA,R0 GOTOH
        STRA,R0 EXPH
        LODA,R0 GOTOL
        STRA,R0 EXPL
        ZBSR *VEXP16_TO_LNUM             ; LNUMH:LNUML = GOTOH:GOTOL (target line)
        BSTA,UN FIND_LINE                ; [+1] sets TMPH:TMPL
        ZBRR *VDR_LP 
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
; Clobbers: R0, EXPH, EXPL, LNUMH, LNUML, TMPH, TMPL, CURH, CURL
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
        ZBSR *VWSKIP  
        BSTA,UN PARSE_S16                ; [+1]
        LODA,R0 EXPH
        BCTR,GT TSL_NZ
        LODA,R0 EXPL
        BCTR,EQ TSL_NO                   ; line number zero: not stored
TSL_NZ:
        ZBSR *VEXP16_TO_LNUM             ; LNUMH:LNUML = EXPH:EXPL (parsed line number)
        ZBSR *VWSKIP                      ; [+1] skip space after line number
        BSTA,UN FIND_LINE                ; [+1] TMP=matched record (CC=EQ), or
                                          ; FIND_INS's insertion point (CC=GT)
        BCTR,EQ TSL_MATCH                ; exact match exists somewhere in the store
        ; No exact match: TMP = first record with line > target, or PE if
        ; none. Legal only if TMP == PE (target exceeds every stored line -
        ; a plain append); otherwise some stored line already exceeds
        ; target with no exact match, which is out of order.
        BSTA,UN CMP_TMP_PE
        BCTR,EQ TSL_WRITE                ; TMP == PE exactly: legal append
        BCTA,UN TSL_ERR
TSL_MATCH:
        ; Exact match at TMP. Legal only if it's the LAST stored line:
        ; save its start, advance a check past it, compare to PE.
        LODA,R0 TMPH
        STRA,R0 CURH
        LODA,R0 TMPL
        STRA,R0 CURL
        BSTA,UN ADV_PAST_RECORD
        BSTA,UN CMP_TMP_PE
        BCTR,EQ TSL_EXCISE
        BCTA,UN TSL_ERR
TSL_EXCISE:
        ; It's the last line: truncate the store back to where it started -
        ; nothing after it, so no shifting needed.
        LODA,R0 CURH
        STRA,R0 PEH
        STRA,R0 TMPH
        LODA,R0 CURL
        STRA,R0 PEL
        STRA,R0 TMPL
TSL_WRITE:
        BSTA,UN WSKIP_PEEK
        BCTR,EQ TSL_DONE                  ; Arithmetic class) - EQ means NUL (empty
                                          ; body): delete-only (or no-op append).
                                          ; IBUF is NUL-terminated, not CR-terminated
        LODA,R0 LNUMH
        STRA,R0 *TMPH                     ; write line hi
        ZBSR *VINC_TMP  
        LODA,R0 LNUML
        STRA,R0 *TMPH                     ; write line lo
        ZBSR *VINC_TMP  
TSL_CPY:
        LODA,R0 *IPH
        BCTR,EQ TSL_CPYDONE                ; NUL: end of typed body
        STRA,R0 *TMPH                     ; copy one body byte
        ZBSR *VINC_TMP  
        ZBSR *VINC_IP  
        BCTR,UN TSL_CPY
TSL_CPYDONE:
        LODI,R0 CR                        ; manufacture the CR terminator ourselves -
        STRA,R0 *TMPH                     ; the stored record format needs one, but
        ZBSR *VINC_TMP                     ; IBUF (NUL-terminated) never contains one
TMP_TO_EXP:
        LODA,R0 TMPH
        STRA,R0 PEH
        LODA,R0 TMPL
        STRA,R0 PEL
TSL_DONE:
        LODI,R0 1                        ; CC=GT: line stored/deleted
        RETC,UN
TSL_ERR:
        ZBRR *VJSYNERR                    ; v0.6: was BCTA,UN JSYNERR (3B); vector already exists

; =============================================================================
;  INC16_TMP_TO_EXP -- EXPH:EXPL = TMPH:TMPL + 1
; Factored out of two identical inlined copies (FIND_LINE's FL_CHKLO,
; FIND_INS's FI_CHK hi-equal path) found via the v0.8 duplicate-byte-
; sequence scan - both need "the byte right after TMP" (the record's
; lo line-number byte lives at TMP+1) and computed it inline.
; In:  TMPH:TMPL
; Out: EXPH:EXPL = TMPH:TMPL + 1
; Clobbers: R0
INC16_TMP_TO_EXP:
        LODA,R0 TMPL
        ADDI,R0 1
        STRA,R0 EXPL
        LODA,R0 TMPH
        TPSL $01
        BCTR,LT I16TE_NC
        ADDI,R0 1
I16TE_NC:
        STRA,R0 EXPH
        RETC,UN

; =============================================================================
;  FIND_LINE -- Search for line LNUMH:LNUML in program store
; Out: TMPH:TMPL = record start if found; CC=EQ found, CC=GT not found.
; Clobbers: R0, TMPH, TMPL, EXPH, EXPL
FIND_LINE:
        BSTR,UN FIND_INS                 ; [+1]
        ; check if at end of program
        BSTA,UN CMP_TMP_PE
        BCTR,LT FL_CHK
        BCTR,UN FL_RET_NF
FL_CHK:
        LODA,R0 *TMPH
        SUBA,R0 LNUMH
        BCTR,EQ FL_CHKLO
FL_RET_NF:
        LODI,R0 1                        ; CC=GT: not found
        RETC,UN

FL_CHKLO:
        BSTR,UN INC16_TMP_TO_EXP
        LODA,R0 *EXPH
        SUBA,R0 LNUML
        BCTR,EQ FL_FOUND
        BCTR,UN FL_RET_NF
FL_FOUND:
        EORZ,R0                          ; CC=EQ: found
        RETC,UN

; =============================================================================
;  ADV_PAST_RECORD -- Advance TMPH:TMPL past the current stored line record
; Skips the 2-byte line-number header, scans forward until CR (end of that
; record's text), then skips the CR too - leaves TMPH:TMPL pointing at the
; start of the NEXT record (or PE, if this was the last one). Factored out
; of two identical inlined copies (TRY_STORE_LINE's TSL_MAS/TSL_MADONE,
; FIND_INS's FI_AS/FI_ADV/FI_DONE) found via a duplicate-byte-sequence
; scan - same v0.8 CMP_TMP_PE technique.
; In:  TMPH:TMPL -> start of a stored record (its line-number hi byte)
; Out: TMPH:TMPL -> start of the next record
; Clobbers: R0
ADV_PAST_RECORD:
        ZBSR *VINC_TMP
        ZBSR *VINC_TMP
APR_LP:
        LODA,R0 *TMPH
        COMI,R0 CR
        BCTR,EQ APR_DONE
        ZBSR *VINC_TMP  
        BCTR,UN APR_LP
APR_DONE:
        ZBSR *VINC_TMP                    ; skip the CR itself
        RETC,UN

; =============================================================================
;  FIND_INS -- Find sorted insertion point for LNUMH:LNUML
; Returns TMPH:TMPL = address of first record with line >= LNUMH:LNUML,
; or PEH:PEL if all lines are smaller.
; In:  LNUMH:LNUML = target line number
; Out: TMPH:TMPL = insertion point
; Clobbers: R0, TMPH, TMPL, EXPH, EXPL
FIND_INS:
        ZBSR *VSET_TMP_PROG
FI_LP:
        BSTA,UN CMP_TMP_PE
        RETC,GT
        RETC,EQ
        ;
        LODA,R0 LNUMH
        SUBA,R0 *TMPH                    ; LNUMH - stored.hi
        BCTR,GT FI_ADV
        RETC,LT                          ; v1.2: was BCTR,LT FI_RET (branch-to-return; RETC has no false-polarity form, but BCTR's TRUE-polarity LT collapses directly)
        ; hi bytes equal: check lo
        BSTA,UN INC16_TMP_TO_EXP
        LODA,R0 LNUML
        COMA,R0 *EXPH                     ; unsigned compare (COM=1 set in MAIN
        BCTR,GT FI_ADV
        RETC,UN
FI_ADV:
        BSTR,UN ADV_PAST_RECORD
        BCTR,UN FI_LP

; =============================================================================
;  EXPR -- Flat-precedence expression evaluator.  One left-to-right pass -
;  all six operators (+-*/=<) sit at the same precedence, matching
;  pBASIC65c02's EXPR/EXPR_LOOP exactly. "1+2*3" evaluates as "(1+2)*3" -
;  deliberate, not a bug.
; In:  IPH:IPL -> expression string
; Out: EXPH:EXPL = 16-bit result ($0000/$FFFF for relops - see DO_EQOP/
; Clobbers: R0, R1, R3, PDEPTH, BANG, SAVEH, SAVEL, NEGFLG, SC0, TMPH, TMPL
EXPR:
        LODI,R3 $FF                      ; SW operand-stack empty sentinel -
        EORZ,R0
        STRA,R0 PDEPTH                    ; SW nesting depth = 0 (genuine
        STRA,R0 BANG                      ; relop-invert modifier off too -
EXPR_GUARDED:
        SPSU                             ; R0 = PSU; SP in bits 2:0
        ANDI,R0 $07
        COMI,R0 PE_RAS_LIMIT
        BCTR,LT EXPR_OK
        LODI,R0 ERR_NEST
        ZBRR *VDO_ERROR
EXPR_OK:
        BSTA,UN EXPR_ATOM
EXPR_LOOP:
        BSTA,UN WSKIP_PEEK
        STRA,R0 SC0                      ; SC0 = char to match against operators
        LODI,R1 15                       ; last row's char offset (6 rows x
                                          ; 3 bytes, walking down to row 0)
OPS_LP:
        LODA,R0 TOK_CHARS,R1              ; table char - direct indexed, no
        SUBA,R0 SC0
        BCTR,EQ OPS_HIT
        SUBI,R1 3                         ; 3-byte stride
        BCFR,LT OPS_LP                    ; loop while R1 still >=0 as a
        
        ; '!' relop-invert modifier: not one of the 6 known operators, but
        LODA,R0 SC0
        COMI,R0 A'!'
        BCFR,EQ EL_NOTBANG
        LODI,R0 $FF
        STRA,R0 BANG                      ; armed - consumed once by whatever
        ZBSR *VINC_IP                     ; consume '!'
        BCTR,UN EXPR_LOOP                 ; loop - look for the real relop
        ; No operator matched: this level's flat chain is done. If we're
EL_NOTBANG:
        LODA,R0 PDEPTH
        RETC,EQ
        SUBI,R0 1
        STRA,R0 PDEPTH                    ; EXPR_LOOP's own top-of-loop
        ZBRR *VINC_IP                     ; consume ')' tail call

OPS_HIT:
        ; PAREN-NEST-01: Push left operand to the LIFO stack
        LODA,R0 EXPL
        STRA,R0 SWBASE,R3+
        LODA,R0 EXPH
        STRA,R0 SWBASE,R3+
        
        ; PAREN-NEST-02: Push the R1 table offset to the stack to survive recursion
        LODZ,R1                  ; R0 = R1 (Destination is ALWAYS R0)
        STRA,R0 SWBASE,R3+       ; Push offset to top of stack

        ZBSR *VINC_IP            ; consume the operator char
        BSTR,UN EXPR_ATOM        ; parse the right operand

        ; Pop the R1 table offset back off the stack
        LODA,R0 SWBASE,R3        ; R0 = top of stack (our saved R1 offset)
        STRZ,R1                  ; R1 = R0 (Source is ALWAYS R0)
        SUBI,R3 1                ; Realign stack pointer down to EXPH for DO_xxx handlers

        ; Now extract the vector safely and jump
        LODA,R0 TOK_CHARS,R1+    ; handler hi (pre-inc: char->hi)
        STRA,R0 GOTOH
        LODA,R0 TOK_CHARS,R1+    ; handler lo (pre-inc: hi->lo)
        STRA,R0 GOTOL
        BCTA,UN *GOTOH           ; jump to handler

; =============================================================================
;  EXPR_ATOM -- parse one atom: unary +/-, parens, or a literal/variable.
; In:  IPH:IPL -> atom
; Out: EXPH:EXPL = value
; Clobbers: R0; PDEPTH too, via EA_PAREN (see there) -- net zero on return,
;   since the matching close-paren decrements it back before this level's
;   RETC fires (OPS_LP exit).
EA_POS:
        ZBSR *VINC_IP  
EXPR_ATOM:
        ZBSR *VWSKIP    ; returns with R0 = char
        COMI,R0 A'-'
        BCTR,EQ EA_NEG
        COMI,R0 A'+'
        BCTR,EQ EA_POS
        COMI,R0 A'('
        BCTR,EQ EA_PAREN
        BCTA,UN PARSE_FACTOR              ; tail call
EA_NEG:
        ZBSR *VINC_IP  
        BSTR,UN EXPR_ATOM                 ; real recursive call (operand)
        BCTA,UN NEG_EXP_BODY              ; tail call: negate, return
EA_PAREN:
        ZBSR *VINC_IP                     ; consume '('
        LODA,R0 PDEPTH
        ADDI,R0 1
        STRA,R0 PDEPTH                    ; PDEPTH++ (SW-tracked, not hw RAS)
        BCTA,UN EXPR_GUARDED              ; TAIL JUMP, not call -- costs 0 hw

; =============================================================================
;  DO_ADD / DO_SUB / DO_MUL / DO_DIV / DO_EQOP / DO_LTOP -- flat operator
;  handlers. Each pops the left operand pushed by OPS_HIT, combines with
;  EXPH:EXPL (the just-parsed right operand), and jumps back to
;  EXPR_LOOP to look for more operators at the same precedence.
; In:  EXPH:EXPL = right operand; SWBASE top = pushed left operand (lo,hi)
; Out: EXPH:EXPL = combined result; control resumes at EXPR_LOOP
; Clobbers: R0, R3 (popped by 2), plus per-operator (see each)
DO_SUB:
        BSTA,UN NEG_EXP_BODY              ; EXP = -EXP
        ; drop through
; =============================================================================
;  ADD16_SAVE_EXP -- EXP = SAVE + EXP (16-bit, WC carry chain); resumes
;  EXPR_LOOP. Left operand is popped off SWBASE (pushed by OPS_HIT above)
;  into SAVEH:SAVEL just before use - a flat cell could have been
;  clobbered by a nested same-precedence op in the meantime.
; In:  EXPH:EXPL = right operand; SWBASE top = pushed left operand (lo,hi)
; Out: EXPH:EXPL = left + EXPH:EXPL; tail-jumps into EXPR_LOOP
; Clobbers: R0, R3 (popped by 2)
DO_ADD:
;ADD16_SAVE_EXP:
        LODA,R0 SWBASE,R3                ; left hi (top, no dec)
        STRA,R0 SAVEH
        LODA,R0 SWBASE,R3-               ; left lo, then R3--
        STRA,R0 SAVEL
        SUBI,R3 1                        ; drop the hi slot too
        CPSL PSW_WC
        LODA,R0 SAVEL
        ADDA,R0 EXPL
        STRA,R0 EXPL
        PPSL PSW_WC
        LODA,R0 SAVEH
        ADDA,R0 EXPH
        STRA,R0 EXPH
        CPSL PSW_WC
        BCTA,UN EXPR_LOOP

DO_MUL:
        BSTR,UN POP_SAVE_TO_TMP           ; TMPH:TMPL = popped left operand
        BSTA,UN MUL16
        BCTA,UN EXPR_LOOP
DO_DIV:
        BSTR,UN POP_SAVE_TO_TMP
        BSTA,UN DIV16
        BCTA,UN EXPR_LOOP

; =============================================================================
;  POP_SAVE_TO_TMP -- pop a 2-byte value pushed on SWBASE into TMPH:TMPL
; In:  R3 = SW stack pointer; top (R3)=hi, (R3-1)=lo of pushed left operand
; Out: TMPH:TMPL = popped value; R3 -= 2
; Clobbers: R0
POP_SAVE_TO_TMP:
        LODA,R0 SWBASE,R3
        STRA,R0 TMPH
        LODA,R0 SWBASE,R3-
        STRA,R0 TMPL
        SUBI,R3 1
        RETC,UN

; =============================================================================
;  DO_EQOP / DO_LTOP -- relop handlers (v0.5, folded into the flat table -
;  see PORT HISTORY). Pop left, compare against right (EXPH:EXPL), leave
;  $0000 (false) or $FFFF (true) in EXPH:EXPL - corrected v1.6: previously
;  documented here (and in EXPR's own header) as "0/1", which was never
;  what DOP_TRUE actually stored; DO_IF's own IORA-then-RETC,EQ test never
;  cared (any nonzero value reads as true), so the mismatch was silent.
DO_EQOP:
        BSTR,UN POP_SAVE_TO_TMP           ; TMPH:TMPL = left
        LODA,R0 TMPH
        SUBA,R0 EXPH
        STRA,R0 SC0
        LODA,R0 TMPL
        SUBA,R0 EXPL
        LODA,R1 SC0
        IORZ R1
        BCTR,EQ DOP_TRUE
        BCTR,UN DOP_FALSE
DO_LTOP:
        BSTR,UN POP_SAVE_TO_TMP           ; TMPH:TMPL = left
        LODA,R0 TMPH
        EORI,R0 $80
        STRA,R0 SC0
        LODA,R0 EXPH
        EORI,R0 $80
        COMA,R0 SC0                       ; biased(right.hi) : biased(left.hi)
        BCTR,GT DOP_TRUE                  ; right.hi > left.hi -> left<right
        BCTR,LT DOP_FALSE
        LODA,R0 EXPL
        COMA,R0 TMPL                      ; right.lo : left.lo (hi bytes equal)
        BCTR,GT DOP_TRUE
DOP_FALSE:
        EORZ,R0
        db $EC                            ; COMA,R0 -- consume next 2 bytes
DOP_TRUE:
        LODI,R0 $FF
        ; both paths converge here (see EORZ,R0/db $EC above): R0 = $00
        ; (false) or $FF (true). Apply the '!' modifier, if OPS_LP armed
        ; it, then disarm it - consumed once per relop, regardless of
        EORA,R0 BANG                      ; R0 ^= BANG ($00 no-op / $FF flips)
        STRA,R0 EXPH
        STRA,R0 EXPL
        EORZ,R0
        STRA,R0 BANG                      ; BANG = 0 again
        BCTA,UN EXPR_LOOP

; =============================================================================
;  PF_LOADVAR -- Load variable value from VARS
; In:  R0 = uppercase variable letter A-Z; IP -> that char
; Out: EXPH:EXPL = variable value
; Clobbers: R0, R1, SC0
PF_LOADVAR:
        STRA,R0 SC0
        ZBSR *VINC_IP  
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

PRO_NONE:
        ZBRR *VJSYNERR 

; =============================================================================
;  PARSE_FACTOR -- Parse a single value (variable or literal)
; In:  IPH:IPL -> first char of factor
; Out: EXPH:EXPL = value
; Clobbers: R0, R1, SC0
; v1.1: no longer upcases - see KNOWN LIMITATIONS (uppercase-only entry).
PARSE_FACTOR:
        LODA,R0 *IPH
        COMI,R0 A'A'
        BCTR,LT PF_NUM
        COMI,R0 A'Z'+1
        BCTR,LT PF_LOADVAR
PF_NUM:
;       drop through
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
        ZBSR *VINC_IP  
        LODI,R0 1
        STRA,R0 NEGFLG
PS16_UN:
;       drop through
; =============================================================================
;  PARSE_U16 -- Parse unsigned decimal digits -> EXPH:EXPL
; Jumps to JSYNERR if no digits found.
; In:  IPH:IPL -> first digit char
; Out: EXPH:EXPL = value
; Clobbers: R0, R3, SC0, EXPH, EXPL, TMPH, TMPL (R3SAVE used to preserve R3)
;PARSE_U16:
        ZBSR *VCLR_EXP
        LODA,R0 *IPH
        COMI,R0 A'0'
        BCTR,LT PRO_NONE; surrogate for JSYNERR
        COMI,R0 A'9'+1
        BCTR,GT PRO_NONE; surrogate for JSYNERR
PU16_LP:
        LODA,R0 *IPH
        COMI,R0 A'0'
        BCTR,LT PU16_RET ; RETC,LT
        COMI,R0 A'9'+1
        BCTR,LT PU16_DIG
        BCTR,UN PU16_RET ; RETC,UN
PU16_DIG:
        SUBI,R0 A'0'
        STRA,R0 SC0
        ZBSR *VINC_IP
        STRA,R3 R3SAVE                   ; save SW stack pointer
        BSTA,UN EXP16_TO_TMP
        ZBSR *VCLR_EXP
        LODI,R3 10
PU16_M10:
        LODA,R0 EXPL
        ADDA,R0 TMPL
        BSTA,UN CARRY_INTO_EXPH
        LODA,R0 EXPH
        ADDA,R0 TMPH
        STRA,R0 EXPH
        BDRR,R3 PU16_M10
        LODA,R3 R3SAVE                   ; restore SW stack pointer
        LODA,R0 EXPL
        ADDA,R0 SC0
        BSTR,UN CARRY_INTO_EXPH
        BCTR,UN PU16_LP

PU16_RET:
        ; drop through

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
        LODI,R1 EXPH-IPH                 ; EXPH offset from IPH (= 4); R1 variant for NEG_SHARED
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
        LODI,R1 TMPH-IPH                 ; TMPH offset from IPH (= 2); R1 variant for NEG_SHARED
        ; fall through to NEG_SHARED

; =============================================================================
;  NEG_SHARED -- Shared negation core (two's complement via 1s complement + INC_ET)
; In:  R1 = offset (EXPH-IPH for EXP, TMPH-IPH for TMP)
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
        LODI,R1 EXPH-IPH
        BCTR,UN NEG_SHARED
; =============================================================================
;  CARRY_INTO_EXPH -- Store R0 into EXPL, propagating carry into EXPH.
; Factored out of three identical inlined copies (PU16_M10, PU16's final-
; digit add, MUL16's MU_ADD) found via a duplicate-byte-sequence scan -
; same v0.8 CMP_TMP_PE technique. Caller does the 8-bit ADDA into R0
; itself (the addend differs per site); this shared tail just stores the
; result and carries into EXPH, exactly as each site did inline.
; In:  R0 = EXPL + <addend>, CC set by that ADDA
; Out: EXPL = R0; EXPH += 1 iff the ADDA carried
; Clobbers: R0
CARRY_INTO_EXPH:
        STRA,R0 EXPL
        TPSL $01
        RETC,LT                          ; v1.2: was BCTR,LT CIE_NC (branch-to-return)
        LODA,R0 EXPH
        ADDI,R0 1
        STRA,R0 EXPH
        RETC,UN

; =============================================================================
;  MUL16 -- Signed 16-bit multiply: TMPH:TMPL * EXPH:EXPL -> EXPH:EXPL
; In:  TMPH:TMPL = left operand; EXPH:EXPL = right operand
; Out: EXPH:EXPL = product (16-bit two's complement wrap)
; Clobbers: R0, R1, NEGFLG, SC0, SC1, TMPH, TMPL, EXPH, EXPL
; RAS: called at depth 6 (see EXPR_GUARDED); own peak (inlined setup's
;   ABS_TMP/ABS_EXP/EXP16_TO_ET/CLR_EXP sub-calls) is depth 7, not 8 --
;   v2.0, was 8 when setup ran through a separate SETUP_MULDIV frame.
MUL16:
        EORZ,R0
        STRA,R0 NEGFLG
        BSTA,UN ABS_TMP                  ; [+1] sets NEGFLG=1 if TMP was negative
        BSTR,UN ABS_EXP                  ; [+1] toggles NEGFLG if EXP was negative
        LODI,R0 SC0-IPH                 ; offset to SCO and 1, SC1 = |EXP| lo
        BSTA,UN EXP16_TO_ET             ; SC0 = |EXP| hi
        ZBSR *VCLR_EXP                  ; clear EXP (accumulator starts at 0)
MU_LP:
        LODA,R0 TMPH
        BCFR,EQ MU_ADD  ; v0.11 fix: was BCTR,GT MU_ADD - see KNOWN BUGS/
                        ; PORT HISTORY. TMPH here is an UNSIGNED magnitude
                        ; byte (post-ABS_TMP), so "is it zero" must ignore
                        ; the sign bit; BCTR,GT wrongly excluded TMPH in
                        ; $80-$FF (bit 7 set, so CC=LT even though nonzero)
                        ; from continuing the loop. BCFR,EQ branches on
                        ; "not exactly zero" instead, covering GT and LT
                        ; alike, same 2 bytes either way.
        LODA,R0 TMPL
        BCTR,EQ MU_DONE
MU_ADD:
        LODA,R0 EXPL
        ADDA,R0 SC1
        BSTR,UN CARRY_INTO_EXPH
        LODA,R0 EXPH
        ADDA,R0 SC0
        STRA,R0 EXPH
        LODA,R0 TMPL
        SUBI,R0 1
        STRA,R0 TMPL
        TPSL $01                         ; C=1 no borrow(EQ) / C=0 borrow(LT) -- v0.10 fix, was BCFR,LT off raw CC (see KNOWN BUGS/PORT HISTORY)
        BCTR,EQ MU_TNB
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
        BCFR,EQ DV_NZ   ; check for zero
        LODA,R0 EXPL
        BCTA,EQ JERRDIVZER
        ; not zero
DV_NZ:
        EORZ,R0
        STRA,R0 NEGFLG
        BSTA,UN ABS_TMP                  ; [+1] sets NEGFLG=1 if TMP was negative
        BSTA,UN ABS_EXP                  ; [+1] toggles NEGFLG if EXP was negative
        LODI,R0 SC0-IPH                 ; offset to SCO and 1, SC1 = |EXP| lo
        BSTA,UN EXP16_TO_ET             ; SC0 = |EXP| hi
        ZBSR *VCLR_EXP                  ; clear EXP (accumulator starts at 0)
DV_LP:
        LODA,R0 TMPH
        COMA,R0 SC0               ; unsigned compare (COM=1 set once globally
        BCTR,LT MU_DONE ; DV_DONE
        BCTR,GT DV_SUB
        LODA,R0 TMPL
        SUBA,R0 SC1
        TPSL $01
        BCTR,EQ DV_SUB
        BCTR,UN MU_DONE ; DV_DONE
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
        ZBRR *VDO_ERROR 

; =============================================================================
;  DO_LIST -- Print stored BASIC lines (v0.1b: whole program only -
; Syntax: LIST
; In:  PROG=program base, PEH:PEL=program end
; Out: whole program printed
; Clobbers: R0, IPH, IPL, TMPH, TMPL, EXPH, EXPL
DO_LIST:
        ZBSR *VSET_TMP_PROG
DLS_LP:
        ; Check TMP against program end
        BSTA,UN CMP_TMP_PE
        RETC,GT
        RETC,EQ

        ; Copy TMP -> IP, read+print line number, then rest of line verbatim
        LODA,R0 TMPH
        STRA,R0 IPH
        LODA,R0 TMPL
        STRA,R0 IPL
        ;
        LODA,R0 *IPH
        STRA,R0 EXPH
        ZBSR *VINC_IP                     ; advance past line hi byte
        LODA,R0 *IPH
        STRA,R0 EXPL
        ZBSR *VINC_IP                     ; advance past line lo byte
        BSTR,UN PRINT_S16
        ZBSR *VPRT_SPACE
DLS_BLPX:
        LODA,R0 *IPH
        COMI,R0 CR
        BCTR,EQ DLS_NL
        ZBSR *VCOUT
        ZBSR *VINC_IP
        BCTR,UN DLS_BLPX
DLS_NL:
        ZBSR *VINC_IP                     ; skip over CR
        BSTA,UN PRT_CRLF
        ;
        LODA,R0 IPH
        STRA,R0 TMPH
        LODA,R0 IPL
        STRA,R0 TMPL
        BCTA,UN DLS_LP

; =============================================================================
;  PRINT_S16 -- Print signed 16-bit value EXPH:EXPL as decimal
; In:  EXPH:EXPL = signed value
; Out: decimal digits written to COUT
; Clobbers: R0, R1, R3, TMPL (R2 saved/restored internally - see v0.9 note)
PRINT_S16:
        LODZ,R2                  ; save caller's R2 (see note above)
        STRA,R0 SC1
        LODA,R0 EXPH             ; get high byte & establish CC
        BCTR,LT IS_NEG           ; branch if negative (bit 7 set)

        IORA,R0 EXPL             ; Check for ZERO
        BCFR,EQ PS_DIGITS       ; >0, flow into subtract printer

        LODA,R0 SC1               ; restore R2/R3 before this tail-call exit -
        STRZ,R2                  ; execution never returns here afterward
        LODI,R0 A'0'             ; Handle Zero
        ZBRR *VCOUT              ; Print '0' and tail call return
IS_NEG:
        LODI,R0 A'-'
        ZBSR *VCOUT
        BSTA,UN NEG_EXP_BODY     ; Negate, making EXPH:EXPL positive
PS_DIGITS:
        EORZ,R0                 
        STRZ,R2                 ; R2 = P10 table index (0 to 4)
        STRZ,R3                 ; R3 = leading zero flag (0 = leading, >0 = printing)
DIGIT_LOOP:
        LODI,R1 A'0'-1           ; R1 = ASCII digit character
SUB_LOOP:
        ; v0.9 fix: CPSL $08 must run before ADDI,R1 1, not after it. The
        ; oracle's own canonical WC idiom clears WC both before AND after
        ; a multi-byte with-carry op ("CPSL $08 to restore standard 8-bit
        ; math mode") - this loop only cleared it before the low-byte sub,
        ; never after the high-byte sub's PPSL $08. So on every loop-back
        ; (and on entry to a fresh digit column after a borrow), ADDI,R1 1
        ; ran with WC still enabled from the previous pass's high-byte
        ; subtract - and per the oracle, WC=1 makes ADDI inject the
        ; leftover carry too, silently adding +2 instead of +1 whenever
        ; the previous pass's high-byte subtraction hadn't borrowed (the
        ; normal case). Confirmed via instruction trace: R1 was observed
        ; jumping 0x30->0x32 (skipping the digit '1' entirely) on exactly
        ; this transition. Moving CPSL $08 here, before ADDI, guarantees
        ; WC is clear on every entry to this loop regardless of path.
        CPSL $08                 ; Clear WC bit for standard 8-bit math
        ADDI,R1 1                ; Increment digit
        LODA,R0 EXPL
        SUBA,R0 P10_LO,R2        ; Subtract low byte
        STRA,R0 TMPL             ; Save tentatively
        PPSL $08                 ; Set WC bit (enables Carry-In/Borrow)
        LODA,R0 EXPH
        SUBA,R0 P10_HI,R2        ; Subtract high byte
        BCTR,LT BORROW           ; If borrow occurred (C=0), result is LT. Stop subtracting.

        ; No borrow: commit result and subtract again
        STRA,R0 EXPH
        LODA,R0 TMPL
        STRA,R0 EXPL
        BCTR,UN SUB_LOOP

BORROW:
        ; Check for leading zero suppression
        COMI,R1 A'0'
        BCTR,GT PRINT_IT         ; Not '0', must print
        EORZ,R0                  ; v0.9 fix: R0 still held the borrow
                                  ; subtraction's leftover (nonzero) value
                                  ; here, so IORZ,R3 (r0|=r3) wasn't
                                  ; actually testing R3 alone - clear R0
                                  ; first so the OR reflects R3 only.
        IORZ,R3                  ; Test R3 flag
        BCFR,EQ PRINT_IT         ; Flag set (>0), print the zero
        COMI,R2 4                ; Is it the final column (1s)?
        BCTR,EQ PRINT_IT         ; Always print the final digit
        BCTR,UN NEXT_DIG         ; Skip printing leading zero

PRINT_IT:
        LODI,R3 1                ; Set leading zero flag
        LODZ,R1                  ; R0 = R1 (destination is always R0)
        ZBSR *VCOUT              ; Print the character
NEXT_DIG:
        ADDI,R2 1                ; Advance to next power of 10
        COMI,R2 5                ; Have we processed all 5 powers?
        BCTR,LT DIGIT_LOOP
        CPSL $08                  ; Clear WC bit
WSKIPRET:
        RETC,UN                  ; Return to caller

; =============================================================================
;  EATWORD -- Consume [A-Z$] chars at IP
; In:  IPH:IPL -> current position
; Out: IP advanced past word
; Clobbers: R0
EATWORD:
        LODA,R0 *IPH
        COMI,R0 A'A'
        BCTR,LT EW_DS
        COMI,R0 A'Z'+1
        BCTR,LT EW_ADV
EW_DS:
        COMI,R0 A'$'
        BCFR,EQ WSKIPRET
EW_ADV:
        ZBSR *VINC_IP 
        BCTR,UN EATWORD

; =============================================================================
;  WSKIP -- Skip spaces at IP
; In:  IPH:IPL -> current position
; Out: IPH:IPL -> first non-space char
; Clobbers: R0
WSKIP:
        LODA,R0 *IPH
        COMI,R0 SP
        BCFR,EQ WSKIPRET
        ZBSR *VINC_IP 
        BCTR,UN WSKIP 

; =============================================================================
;  SHARED 16-BIT POINTER INCREMENT  - INC_ET family
; INC_EXP : EXPH:EXPL += 1   (offset EXPH-IPH from IPH)
; INC_TMP : TMPH:TMPL += 1   (offset TMPH-IPH from IPH)
; INC_IP  : IPH:IPL  += 1    (offset 0 from IPH)
; All share INC_ET body using register bank switch.
; Rule: NO BSTA inside these -- must not consume extra RAS depth.
; Offsets are assembly-time expressions (e.g. EXPH-IPH=4) -- sequential
; ordering of the IPH..LNUML block must be preserved or these silently break.
INC_EXP:
        LODI,R0 EXPH-IPH        ; EXP offset from IPH (= 4); assembly-time expression
        db $EC                  ; COMA,R0 -- consume next 2 bytes (skip to INC_IP path)
INC_TMP:
        LODI,R0 TMPH-IPH        ; TMP offset from IPH (= 2); assembly-time expression
        db $C4                  ; COMI,R0 -- consume next 1 byte
INC_IP:
        EORZ,R0                 ; offset = 0 (IPH itself)
; Can jump in here with R0 set for offset
INC_ET:
        PPSL PSW_RS                 ; switch to alternate register bank
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
        CPSL PSW_RS                 ; switch back to primary bank
        RETC,UN

; =============================================================================
;  EXP16_TO_ET family -- copy EXPH:EXPL to any RAM register pair.
;  ET_TO_EXP16 family -- copy any RAM register pair to EXPH:EXPL.
;
;  Placed immediately after INC_ET so BCTR,UN ET_STORE / BCTR,UN ET_RET
;  reach the shared tails above within ±63 bytes 
;  Each entry loads its offset (XYZH-IPH) into R0 (always bank-0, unaffected
;  by PSW_RS), falls through to body.  STRZ R1 copies R0 into alt-R1 for
;  indexed addressing.  Primary R1/R2/R3 fully preserved via CPSL PSW_RS.
;  Clobbers R0 only.  NO BSTA inside body.
;  Direct BSTA,UN (no ZP slot): CUR_TO_EXP16 (1 site).
EXP16_TO_TMP:
        LODI,R0 TMPH-IPH      ; TMPH offset from IPH
        db $EC                  ; COMA,R0: skip next 2 bytes
EXP16_TO_GOTO:
        LODI,R0 GOTOH-IPH       ; GOTOH offset from IPH (= 8)
        db $EC                  ; COMA,R0: skip next 2 bytes
EXP16_TO_LNUM:
        LODI,R0 LNUMH-IPH       ; LNUMH offset from IPH (= 12)
EXP16_TO_ET:
        PPSL PSW_RS             ; switch to alternate register bank
        STRZ R1                 ; alt-R1 = R0 = destination offset
        LODA,R0 EXPL
        STRA,R0 IPL,R1          ; store lo byte to dest+1
        LODA,R0 EXPH
        BCTR,UN ET_STORE        ; store hi byte, restore bank, return

; -----------------------------------------------------------------------------
CUR_TO_EXP16:
        LODI,R0 CURH-IPH        ; CURH offset from IPH (= 10)
ET_TO_EXP16:
        PPSL PSW_RS             ; switch to alternate register bank
        STRZ R1                 ; alt-R1 = R0 = source offset
        LODA,R0 IPH,R1          ; load hi byte from source
        STRA,R0 EXPH
        LODA,R0 IPL,R1          ; load lo byte from source
        STRA,R0 EXPL
        BCTR,UN ET_RET


; =============================================================================
;  JERRVAR -- Error with variable
;  JSYNERR -- Syntax error jump
; In:  nothing (R0 irrelevant)
; Out: jumps to DO_ERROR
; Clobbers: R0
JERRVAR:
        LODI,R0 ERR_VAR
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to BCTA
JSYNERR:
        LODI,R0 ERR_SYN
        ; drop through
; =============================================================================
;  DO_ERROR -- Print error, clear run state, return to REPL
; Entry: R0 = error code character ('0'..'8').
; Clears RUNFLG, SWSP, FORSP. Prints "?n" or "?n@line" if running.
; Tail-jumps to REPL (clears full hardware RAS).
; In:  R0 = error code
; Out: jumps to REPL
; Clobbers: all (RAS cleared by REPL)
DO_ERROR:
        STRZ,R1                         ; Save ASCII error code
        BSTR,UN PRT_QUEST               ; Print Question mark
        LODZ,R1
        ZBSR *VCOUT                     ; print error code
        LODA,R0 RUNFLG                  ; OPT-10: SC1=RUNFLG, 0->EQ, 1->GT
        BCTR,EQ DE_NL                   ; not running, no line number
        LODI,R0 '@'                     
        ZBSR *VCOUT                     ; Print at line
        BSTR,UN CUR_TO_EXP16             ; EXPH:EXPL = CURH:CURL
        BSTA,UN PRINT_S16                ; [+1]
DE_NL:
        BSTR,UN PRT_CRLF
        BSTA,UN DO_END                   ; [+1] clears SWSP, FORSP, GOTOFLG, RUNFLG
        BCTA,UN REPL                     ; REPL resets RAS (PSU SP bits) on entry

; =============================================================================
;  Shared character print routines -- $EC (COMA) byte-skip chain
; Each entry loads its character then falls through via the skip opcode trick.
PRT_CRLF:
        BSTR,UN PRT_CR                   ; Print CR/LF
        LODI,R0 LF
        db $EC
PRT_QUEST:
        LODI,R0 '?'
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to next LODI
PRT_CR:
        LODI,R0 CR
        db $EC
PRT_SPACE:
        LODI,R0 32
        ZBRR *VCOUT                
        
; =============================================================================
;  SET_IP_IBUF -- Set IPH:IPL = IBUF base address
; In:  nothing
; Out: IPH = <IBUF, IPL = >IBUF
; Clobbers: R0
SET_IP_IBUF:
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
        RETC,UN

; =============================================================================
; CLR_EXP -- Helper Zeroes EXP
; Clobbers R0
CLR_EXP:
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 EXPL
        RETC,UN
        
; =============================================================================
;  TABLES 
BANNER:
        DB CR, LF, "pBASIC2650 0.21", CR, LF, NUL

; -- Combined operator + statement dispatch table
; Format: [char][hi][lo], stride 3, NUL-terminated.
; Statement scan (STMT_EXEC/MD_SCAN) safely runs the WHOLE table because
; its 2nd-char letter-gate guarantees the char being matched is A-Z,
; which none of the 6 operator chars are - no bounding needed there.
; Operator scan (EXPR_LOOP/OP_SCAN) is explicitly bounded to the first 6
; entries
TOK_CHARS:
        DB "+", <DO_ADD,    >DO_ADD       ; +
        DB "-", <DO_SUB,    >DO_SUB       ; -
        DB "*", <DO_MUL,    >DO_MUL       ; *
        DB "/", <DO_DIV,    >DO_DIV       ; /
        DB "=", <DO_EQOP,   >DO_EQOP      ; = (relop, folded in - v0.5)
        DB "<", <DO_LTOP,   >DO_LTOP      ; < (relop, folded in - v0.5)
        DB "A", <DO_ASK,    >DO_ASK       ; ASK
        DB "E", <DO_END,    >DO_END       ; END
        DB "G", <DO_GOTO,   >DO_GOTO      ; GOTO
        DB "I", <DO_IF,     >DO_IF        ; IF
        DB "L", <DO_LIST,   >DO_LIST      ; LIST
        DB "N", <DO_NEW,    >DO_NEW       ; NEW
        DB "P", <DO_PRINT,  >DO_PRINT     ; PRINT
        DB "R", <DO_RUN,    >DO_RUN       ; RUN
        DB "W", <DO_WR,     >DO_WR        ; WR
        DB NUL

; Powers of 10 Tables (10000, 1000, 100, 10, 1)
P10_HI:
        db $27, $03, $00, $00, $00
P10_LO:
        db $10, $E8, $64, $0A, $01

ROMEND: 

;  RAM variables -- sequential RES block 
 
        ORG     4096    ; half a 2650 8kbyte page

; --- Ordered group: offsets from IPH used by INC_ET/DEC_ET/NEG_SHARED ---
IPH     RES 1       ; interpreter pointer hi       (INC_ET offset 0)
IPL     RES 1       ; interpreter pointer lo
TMPH    RES 1       ; temp 16-bit hi               (INC_ET offset 2 = TMPH-IPH)
TMPL    RES 1       ; temp 16-bit lo
GOTOFLG RES 1       ; $00=sequential $01=GOTO $02=GOSUB $03=FOR direct addr
GOTOH   RES 1       ; pending target hi            (DEC_ET offset 8 = GOTOH-IPH)
GOTOL   RES 1       ; pending target lo
CURH    RES 1       ; current line hi  (error reporting)
CURL    RES 1       ; current line lo

LNUMH   RES 1       ; scratch line number hi       (DEC_ET offset 12 = LNUMH-IPH)
LNUML   RES 1       ; scratch line number lo
EXPH    RES 1       ; expression result hi         (INC_ET offset 4 = EXPH-IPH)
EXPL    RES 1       ; expression result lo
SWSTK   RES 2       ; next-line pointer cache [NLP_H][NLP_L] written by DR_EXEC

; --- Remaining ---
SC0     RES 1       ; Scratch byte 0
SC1     RES 1       ; Scratch byte 1
PEH     RES 1       ; Program end pointer hi
PEL     RES 1       ; Program end pointer lo
;PEH     DB <SHOWCASE_END       ; Program end pointer hi
;PEL     DB >SHOWCASE_END       ; Program end pointer lo
SAVEH   RES 1       ; ADD16_SAVE_EXP: popped left operand scratch (hi)
SAVEL   RES 1       ; ADD16_SAVE_EXP: popped left operand scratch (lo)
PDEPTH  RES 1       ; SW-tracked paren nesting depth 

;  --- Flags & Stuff --- 
RUNFLG  RES 1       ; $01=running $00=immediate
R3SAVE  RES 1       ; Save/restore R3 across PARSE_U16 multiply loop
NEGFLG  RES 1       ; Sign flag
BANG    RES 1       ; '!' relop-invert modifier: $00 clear, $FF armed -
                    ; XORed into the boolean result in DOP_TRUE, then
                    ; cleared there. Also cleared at EXPR's top-level entry.

;  SW call stack -- used by PARSE_EXPR / PRINT_S16 only
; R3 = index ($FF=empty, grows up). Each frame = [lo][hi].
; Push: STRA,R0 *SWBASE,R3+ (lo first), STRA,R0 *SWBASE,R3+ (hi).
; Pop:  LODA,R0 *SWBASE,R3- (hi first), LODA,R0 *SWBASE,R3- (lo).
SWBASE  RES 16      ; SW stack base 

; Buffers
IBUF    RES 64      ; Input buffer 64 bytes
VARS    RES 52      ; A-Z variables 2 bytes each

; =============================================================================
;  Pre-loaded SHOWCASE program
;
;  Line format: <lineno_hi> <lineno_lo> <body_ASCII> <CR>
;  Lines  10-190: feature demos (PRINT, WR, arithmetic, comparisons, GOTO loop)
;  Lines 236-238: LIST demo (v0.1b: FOR/NEXT, GOSUB/RETURN demos and the
;                 line-530 GOSUB target removed; LIST is whole-program only)
;  Lines 300-510: Mandelbrot set renderer (v4.3: widened, C=-144..28 step 4,
;                 44 cols vs v4.2's 32; row range I=-64..56 step 6 unchanged)
;  Line  530:     GOSUB subroutine (PRINT "sub"; / RETURN)
;
;  Format: DB hi,lo,"text",$0D  -- hi-then-lo matches DR_EXEC record format.
;  $22=DQ $3B=semicolon  in-string chars that need escaping.
; =============================================================================
PROG:
        DB 0,20,"PRINT ",$22,"-- pBASIC2650 Showcase --",$22,$0D
        DB 0,30,"PRINT ",$22,"--- PRINT / WR ---",$22,$0D                      ; 30  PRINT "--- PRINT / WR ---"
        DB 0,40,"WR 65",$0D                                                    ; 40  WR 65
        DB 0,41,"WR 66",$0D                                                    ; 41  WR 66
        DB 0,42,"WR 67",$0D                                                    ; 42  WR 67
        DB 0,43,"PRINT",$0D                                                    ; 43  PRINT
        DB 0,50,"PRINT ",$22,"--- ARITHMETIC ---",$22,$0D                      ; 50  PRINT "--- ARITHMETIC ---"
        DB 0,60,"PRINT ",$22,"3+4=",$22,$3B,"3+4",$3B,$22,"  10-3=",$22,$3B,"10-3",$3B,$22,"  6*7=",$22,$3B,"6*7",$0D  ; 60  PRINT "3+4=";3+4;"  10-3=";10-3;"  6*7=";6*7
        DB 0,70,"PRINT ",$22,"20/4=",$22,$3B,"20/4",$0D                        ; 70  PRINT "20/4=";20/4
        DB 0,80,"PRINT ",$22,"--- COMPARISONS ---",$22,$0D                     ; 80  PRINT "--- COMPARISONS ---"
        DB 0,90,"IF 3<9 PRINT ",$22,"3<9 ok",$22,$0D                      ; 90  IF 3<9 PRINT "3<9 ok"
        DB 0,100,"IF 7=7 PRINT ",$22,"7=7 ok",$22,$0D                     ; 100 IF 7=7 PRINT "7=7 ok"
        DB 0,110,"IF 9!=2 PRINT ",$22,"9!=2 ok",$22,$0D                   ; 110 IF 9!=2 PRINT "9!=2 ok"
        DB 0,120,"IF 9!<4 PRINT ",$22,"9!<4 ok",$22,$0D                   ; 120 IF 9!<4 PRINT "9!<4 ok" (9>=4)
        DB 0,130,"IF 4<9 PRINT ",$22,"9>4 ok",$22,$0D                     ; 130 IF 4<9 PRINT "9>4 ok" (reversed operands for >)
        DB 0,135,"IF 9!<6 PRINT ",$22,"6<=9 ok",$22,$0D                   ; 135 IF 9!<6 PRINT "6<=9 ok" (reversed operands for <=)
        DB 0,140,"PRINT ",$22,"--- LOOP via GOTO ---",$22,$0D                  ; 140 PRINT "--- LOOP via GOTO ---"
        DB 0,150,"I=1",$0D                                                      ; 150 I=1
        DB 0,160,"IF 5<I GOTO 190",$0D                                    ; 160 IF 5<I GOTO 190
        DB 0,170,"PRINT I",$3B,$0D                                              ; 170 PRINT I;
        DB 0,180,"I=I+1",$0D                                                    ; 180 I=I+1
        DB 0,185,"GOTO 160",$0D                                                 ; 185 GOTO 160
        DB 0,190,"PRINT ",$22,"",$22,$0D                                        ; 190 PRINT ""
        DB 0,216,"PRINT ",$22,"",$22,$0D                                        ; 216 PRINT ""
;        DB 0,236,"PRINT ",$22,"--- LIST ---",$22,$0D                            ; 236 PRINT "--- LIST ---"
;        DB 0,238,"LIST",$0D                                                     ; 238 LIST
        DB 0,240,"GOTO 300",$0D                                                 ; 240 GOTO 300
        DB 1,44,"PRINT ",$22,"--- MANDELBROT ---",$22,$0D                      ; 300 PRINT "--- MANDELBROT ---"
        DB 1,54,"I=-64",$0D                                                     ; 310 I=-64
        DB 1,64,"IF 56<I GOTO 510",$0D                                    ; 320 IF 56<I GOTO 510
        DB 1,74,"D=I",$0D                                                       ; 330 D=I
        DB 1,84,"C=-144",$0D                                                    ; 340 C=-144 (widened from -120)
        DB 1,94,"IF 28<C GOTO 480",$0D                                    ; 350 IF 28<C GOTO 480 (widened from 4)
        DB 1,104,"A=C",$0D                                                      ; 360 A=C
        DB 1,105,"B=D",$0D                                                      ; 361 B=D
        DB 1,106,"E=0",$0D                                                      ; 362 E=0
        DB 1,107,"N=1",$0D                                                      ; 363 N=1
        DB 1,114,"IF 16<N GOTO 420",$0D                                   ; 370 IF 16<N GOTO 420
        DB 1,124,"IF 0<E GOTO 410",$0D                                    ; 380 IF 0<E GOTO 410
;        DB 1,134,"T=A*A/64-B*B/64+C",$0D                                       ; 390 T=A*A/64-B*B/64+C
        DB 1,134,"T=(A*A/64)-(B*B/64)+C",CR             
        DB 1,144,"B=2*A*B/64+D",$0D                                             ; 400 B=2*A*B/64+D
        DB 1,145,"A=T",$0D                                                      ; 401 A=T
        DB 1,154,"IF 256<((A*A/64)+(B*B/64)) IF E=0 E=N",CR                     ; 410 restored - v0.6 split this into a 405 helper line to dodge a RAS-01 crash; v1.3 fixed the underlying paren-depth bug, so the original 2-level nested line works again
        DB 1,164,"N=N+1",$0D                                                    ; 420 N=N+1
        DB 1,165,"IF 16<N GOTO 430",$0D                                   ; 421 IF 16<N GOTO 430
        DB 1,166,"GOTO 370",$0D                                                 ; 422 GOTO 370
        DB 1,174,"IF 0<E WR E+32",$0D                                     ; 430 IF 0<E WR E+32
        DB 1,184,"IF E=0 WR 32",$0D                                       ; 440 IF E=0 WR 32
        DB 1,194,"C=C+4",$0D                                                    ; 450 C=C+4
        DB 1,204,"GOTO 350",$0D                                                 ; 460 GOTO 350
        DB 1,224,"PRINT",$0D                                                    ; 480 PRINT
        DB 1,234,"I=I+6",$0D                                                    ; 490 I=I+6
        DB 1,244,"GOTO 320",$0D                                                 ; 500 GOTO 320
        DB 1,254,"END",$0D                                                      ; 510 END
SHOWCASE_END:

        END
