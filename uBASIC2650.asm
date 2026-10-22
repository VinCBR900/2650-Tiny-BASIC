; =============================================================================
; uBASIC2650 v2.14  --  Integer Tiny BASIC for the Signetics 2650
; Copyright (c) 2026 Vincent Crabtree, licensed under the MIT License, see LICENSE
;
; Note: standalone build - I/O is bit-banged in software, no PIPBUG ROM or
; UART/ACIA hardware required.
;
;   CPU    : Signetics 2650
;   ROM    : <2 KB target, $0000 upward (currently 1784 bytes, ROMEND $06F8)
;   RAM    : ~213 bytes, $1000 upward (half an 8 KB page above the code)
;   I/O    : CHIN/COUT, bit-banged software serial via PSU/PSL flag bits.
;            Addresses float per build - read them from the .LST (see BUILD)
;   NMI/IRQ: Not used
;
; USAGE
;   Burn the image at $0000 (fits a 2716); RAM starts at $1000. Reset enters at
;   $0000. The banner and "Free:<bytes>" print, then the ">" prompt. Type a
;   statement to run it at once, or start a line with a number (0-32767) to store
;   it in the program; a number alone deletes that line. RUN executes the stored
;   program, LIST prints it, NEW clears it. A demo program is preloaded: RUN it,
;   or NEW first. Serial is 110 baud, bit-banged on SENSE (in) / FLAG (out).
;   Errors print "?n" (plus "@line" while running): 0 syntax, 2 divide by zero,
;   3 out of memory/stack, 4 bad variable, 8 expression too complex.
;
; Statements:
;   INPUT  END  FREE  GOTO <expr>  GOSUB <expr>  IF <cond> [THEN] <stmt>
;   LIST  NEW  PRINT  RETURN  RUN  [LET] <var>=<expr>  (LET and THEN optional)
;   FOR <var>=<expr> TO <expr> [STEP <expr>]   NEXT [<var>]
;
; PRINT items: "literal", CHR$(n), TAB(n), or an expression; separate with ';'.
;
; Arithmetic: + - * /  (unary -).  
; Precedence: BODMAS-lite * / binds before + -, which binds before relops (= < >).
; Relops: `=`  `<`  `>`  prefix with ! to invert: `!=`, `!<`, `!>` (<= is !>, >= is !<).
; RND: niladic pseudorandom 16-bit signed function, no parenthesis/arg.
; ABS(expr): absolute value function - expressions allowed
;
; Numbers : signed 16-bit  (-32768 .. 32767)
; Variables: A-Z (26), 16-bit signed; no arrays or string variables
; Print   : "literals", CHR$(n), TAB(n), `;` separator - no string vars
;
; KNOWN LIMITATIONS
;
; UPPERCASE only apart from PRINT "String literals"
;
; RELATIONAL OPERATORS
;   =  <  > are native; '!' inverts the one that follows it (!=, !<, !>),
;   so all six are available: A<=B is A!>B; A>=B is A!<B; A<>B is A!=B.
;   A relop yields -1 (true) or 0 (false).
;   Relops have the LOWEST precedence and associate to the RIGHT: a relop's right
;   operand is a whole + - * / expression, so A<B+1 is A<(B+1) and
;   1+A<B*2 is (1+A)<(B*2).  Do not chain relops: 1<2<3 is 1<(2<3.  Keep a '!' 
;   relop's right operand free of other relops (use a variable or separate IFs).
;
; OPERATOR PRECEDENCE
;   BODMAS-lite, tightest first: * /   then   + -   then relops.
;   So "1+2*3" is 7 and "1+2>2*2" is (1+2)>(2*2).  * / and + - are left-to-right
;   among themselves; relops are right-to-left (see above).  Parentheses override.
;
; PARENTHESIS NESTING
;   Bounded by the SW stack (SWBASE, 64 bytes). Limit depends on expression
;   complexity - operators within a level consume additional stack so a
;   deeply-nested expression with many operators uses more stack per level.
;   ERR_EXPR ('?8') fires gracefully when the stack is full. Practical
;   ceiling for typical BASIC expressions is 5-6 levels of mixed
;   paren/operator nesting.
;
; GOSUB/RETURN NESTING
;   Fixed ceiling of 4 levels deep (GSSTKLIM=8 bytes, 2 per level). A 5th
;   nested GOSUB, or a RETURN with nothing pushed, raises ERR_OOM ('?3').
;
; FOR / NEXT
;   FOR <var>=<start> TO <limit> [STEP <step>], then NEXT [<var>].  Step
;   defaults to 1 if omitted; a negative STEP counts down (limit < start).
;   - Test at end so Body always runs at least once
;   - NEXT always closes the innermost loop
;   - STEP 0 loops forever (var never passes the limit) - no check for it.
;   - Leaving a loop early (GOTO / GOSUB target outside it) not permitted.
;
; RUN-STATE LIMITATIONS (v2.14)
;   - LIST inside a running program prints the listing and then ends the run:
;     the program counter (SWSTK) is the listing cursor, so it is left at the
;     end of the program.
;   - GOSUB/FOR state does not survive past the line that created it: the
;     prompt clears the GOSUB and FOR stacks before every line, so an immediate
;     "FOR ..." then a later "NEXT" at the prompt gives ?3 (nothing to NEXT).
;   - Unary plus is not accepted ("+5" is a syntax error); unary minus is.
;
; STATEMENT DISPATCH matches ONLY the first character of a line.
;   GOTO/GOSUB, RUN/RETURN/REM, LIST/LET and NEW/NEXT dispatched on 3rd char.
;   LET and THEN are optional.
;
; STORAGE / INPUT
;   Full random line editing: a numbered line is inserted in order,
;   replaces an existing line of the same number, or (empty body) deletes it.
;   Permitted Line numbers are 0-32767.
;   Input is not bounds-checked; overlong lines will corrupt memory.
;
; SYNTAX VALIDATION
;   Minimal; malformed constructs may produce a generic syntax/runtime
;   error rather than a specialised diagnostic.
;
; =============================================================================
; IMPLEMENTATION NOTES (for maintainers)
; =============================================================================
;
; BUILD
;   gcc -Wall -O2 -o asm2650 asm2650.c
;   gcc -O2 -DGAMER -o pipbug_wrap pipbug_wrap.c
;
;   ./asm2650 --warn=all uBASIC2650.asm uBASIC2650.hex
;   grep -n "^CHIN \|^COUT \|^ROMEND " .\uBASIC2650.LST
;   ./pipbug_wrap --entry 0 --chin 0x<addr> --cout 0x<addr> uBASIC2650.hex
;
; ROMEND is used as ROM tide mark
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
;   R2  current variable letter; MUST survive the entire RHS expression
;       evaluation of a "V=expr" assignment (DL_STORE reads it back after
;       EXPR returns) - do not use as scratch inside EXPR/EXPR_ATOM or any
;       routine they call. PEEK_C2_ALPHA clobbers R2 by design (its 2-letter
;       keyword check for DO_PRINT) and is therefore only safe to call from
;       statement-level dispatch, never from inside expression parsing.
;   R3  loop counter / Expression SW stack 
;
; RAS / RECURSION
;   The 2650 has an 8-level hardware return-address stack, not user accessible.
;   BSxx/ZBSR consume a RAS entry; BCxx/ZBRR do not.
;   PARSE_EXPR guards against excessive hardware-stack depth.
;   Parentheses use a software depth counter to reduce RAS usage.
;   GOSUB/RETURN do NOT consume RAS for BASIC-level nesting depth - SWSTK
;   (not a real call) carries the redirect, so 4 levels of GOSUB cost the
;   same hardware stack as 1.
;
; =============================================================================
; VERSION HISTORY 
; =============================================================================
;
; v2.14 (Oct 2026) - Golf ported from pBASIC2650 v0.35. ROMEND $06F8 (1784 bytes)
;   - BUG FIX: PRINT_S16 called COUT with WC=1, COUT's RRR would rotate through
;     carry, but Simulator intercepted COUT would not error. WC now cleared before.
;   - PRINT_S16: WC/carry set once per column (PPSL $09), digit counted with BIRR,
;     leading-zero flag is the OR of the digit counts (-4).
;   - Program counter is now SWSTK (was TMP, copied to/from SWSTK every
;     line): DO_RUN/DO_GOTO/FIND_LINE/ADV_PC_PAST_REC/DEL_REC/OPEN_GAP/DO_LIST
;     all walk SWSTK directly. INC_TMP/CMP_TMP_PE/SET_TMP_PROG/ADV_TMP_PAST_REC
;     renamed INC_PC/CMP_PC_PE/SET_PC_PROG/ADV_PC_PAST_REC.
;   - REPL now calls CLR_RUNFLG before every line (MAIN and DO_ERROR no longer do):
;     run state, GOSUB and FOR stacks are always empty at the prompt
;   - OPS_HIT_COM pushes [R1][EXPL][EXPH] with a BRNR loop; OPS_HIT_RET pops in
;     the matching order (-2). DO_NEW copies PROG to PE via SWSTK/REG16
;   - ABS_X (EXP or TMP via R1, falls into NEG_SHARED) + MD_PREP replace ABS_TMP,
;     NEG_TMP and the old ABS_EXP/MD_PREP pair (-6). WSKIP ends in IORZ,R0
;   - PEEK_C2_ALPHA tests bit 6 (ANDI $40) instead of a range compare
;   - Unary plus (EA_POS) removed - it was never documented 
;
; v2.13 (Oct 2026) - Golf pass. ROMEND $0722 (1826 bytes)
;   - FIND_LINE/FIND_INS merged: the CC of the compare that stops the walk is the
;     found flag, so the second look at the record is gone
;   - SW stack now holds "bytes used" in R3 and uses pre-increment/pre-decrement
;     indexed stores/loads (PUSH_RET, SWRETURN, OPS_HIT_*); DO_GOSUB/DO_RETURN and
;     the FOR frame push/pop do the same.
;   - PUSH_RET takes only the low byte: OPS_HIT_RET, HI_LOOP, LO_LOOP, EP_RET,
;     NEG_RET and ABS_RET must share one 256-byte page (they are grouped in the
;     "EXPRESSION CONTINUATION PAGE" block; the build checks it). PUSH_LOLOOP falls
;     into PUSH_HILOOP so one call stacks both.
;   - CMP_TMP_PE and CMP_OPS use COMA (CMP_OPS drops to signed COM=0 for the hi
;     byte); DO_NEXT decides loop/exit from SPSL xor the step's sign.
;   - New helpers DEC16 (OPEN_GAP, MULT_LOOP), MD_PREP (DO_MUL/DO_DIV) and a
;     $EC-chained error exit (JERRDIVZER/JERRVAR/DRT_UFLOW); PRINT_S16 rewritten
;     (VPRINT_S16 vector); TRY_STORE_LINE returns CC=GT for "not a numbered line".
;   - STMT_EXEC leaves the 3rd keyword character in R3 (RXSAVE RAM byte removed).
;   - DO_PRINT family and PEEK_C2_ALPHA moved next to PRT_CRLF; relative branches.
;
; v2.12 (Oct 2026) - RND entropy: LFSR is stepped on every keystroke read and
;   8 steps per RND call. 
;   - ROMEND $07D9 (2009 bytes); 
;
; v2.11 (Sep 2026) - Code golf: REG16_TO_REG16 generic 16-bit copy.
;   - Replaced EXP16_TO_ET/TMP_TO_ET and DE_LOOP, DR_SWSTK,DN_LIM, GR_LP, OG_SAV
;     copy loops with REG16_TO_REG16 using R0 = packed (SRC_IDX<<4)|DST_IDX pair. 
;   - RAM reorg: PEH/PEL and SC0/SC1 moved contiguous for use with REG16.
;   - ROMEND $07EF (2031) -> $07D5 (2005 bytes)
;
; v2.10 (Sep 2026) - BUG FIX: mid-list line insert corrupted following line.
;   - OPEN_GAP's move loop decrements PE via an inlined former DEC_PE.
;     However still used "RETC,EQ" for fast path so bailed early. Fixed by 
;     jumping to where subroutine end would be.
;     ROMEND $07EF (2031 bytes)
;
; v2.9 (Sep 2026) - Added ABS(x) function to EXPR, and FREE memory statement.
;   -  ROMEND $07C5 (1989) -> $07EE (2030 bytes).
;
; v2.8 (Sep 2026) - Code golf, ported from pBASIC2650.asm v0.33.
;   - New ADD_CORE: shared 16-bit indexed add (EXPH:EXPL += *(IPH+off)) used by
;     DO_ADD and MULT_LOOP, deleting CARRY_INTO_EXPH. 
;   - Minor Refactor ABS_EXP for size.
;   - DV_LP: removed leading CPSL $08 (matches pBASIC's) global invariant.
;   - ROMEND $07E1 (2017) -> $07C7 (1991 bytes).
;
; v2.7 (Sep 2026) - BASIC line editing, LET/THEN/INPUT and FOR/NEXT restored.
;   - TRY_STORE_LINE: FIND_LINE; if the line exists DEL_REC removes it and
;     FIND_LINE is repeated.  New DEL_REC (fwd copy), OPEN_GAP (backward copy)
;     and DEC_PE. Line numbers > 32767 rejected with syntax error. 
;   - PROGLIM (EQU $1FFF, must be $xxFF): OPEN_GAP builds the new PE in EXP,
;     checks its hi byte, and only then moves anything; store full gives ?3
;     (ERR_OOM) with the store untouched (+10). A REPLACE that does not fit has
;     already deleted the old line. 
;   - Relop refactor, added '>': 3-way compare, each handler picks its CC (EQ/LT/GT).
;   - FOR/NEXT: body runs at least once.
;   - FOR/NEXT frame is pushed/popped by loops 
;   - STEP added, frame widened to 7 bytes, holding [var][limit lo/hi]
;     [step lo/hi][body lo/hi]; 
;   - SHOWCASE updated for new keywords amd relops.
;   - Golf pass - duplicate digit test (LODA,R0 *IPH; SUBI,R0 A'0';
;     COMI,R0 9) inlined in TRY_STORE_LINE and PARSE_U16, now DIGIT_CHECK.
;     ROMEND $07FA (2042) -> $07F5 (2037 bytes);
;
; v2.6 (Sep 2026) - Code-golf & Bugfix. ROMEND: $06f6 (1782 bytes)
;   - STMT_EXEC/MD_SCAN: dispatch scan now starts at 1st STATEMENT row, skipping
;     operator rows to fix crash bug when a line started with an operator.
;   - BUG FIX: TSL_CPYDONE's "tail call" into TMP_TO_ET used BSTA instead
;     of BCTA.
;   - BUG FIX DR_LP only cleared RUNFLG when TMP>PE (overrun), not equal. 
;   - PARSE_VAR_SAVE/PARSE_FACTOR/TRY_STORE_LINE/PU16/EATWORD: two-comparison 
;     signed replaced with SUBI+COMI+single unsigned branch throughout. 
;   - PARSE_VAR_SAVE now threads the VARS byte-offset (index*2) through R2
;     instead of the raw letter;
;   - FIND_LINE/FIND_INS: INC16_TMP_TO_EXP deleted.
;   - DO_EQOP/DO_LTOP: intermediate compare byte now parked in R1 not SC0.
;   - DIV16: remainder was unused, no MOD operator. DV_LP/DV_SUB/DV_SNB's 
;     refactored into destructive subtract-and-test-borrow pass
;   - PRINT_S16: removed the R2 save/restore 
;   - DR_HDR: refactored to use shared TMP_TO_ET call 
;   - NEGFLG polarity flipped to 0=negate across PARSE_S16,ABS_TMP, ABS_EXP,
;     NEG_EXP, DO_MUL, MU_DONE. 
;   - IBUF moved to $1010 so GETLINE now sets IPH:IPL with one LODI.
;   - MUL16's multiply loop and PU16's x10 shareMULT_LOOP leaf sub. PU16_DIG
;     doesnt need EXP16_TO_TMP so deleted.
;   - 
; v2.5 (Sep 2026) - Added niladic RND to the expression parser.
;   - EXPR_ATOM: added 'R'-then-alpha check - cant use PEEK_C2_ALPHA due to R2
;   - Golf pass, all over.
;   - ROMEND: $0800 after RND detection -> $07AF (1976 bytes)
;   - Stored-program end of line terminatorchanged to NUL from CR, matches what
;     STMT_EXEC/EXPR/EATWORD/etc need so DO_RUN executes straight out of PROG. 
;     Updated Showcase's terminator lines to match.
;   - ROMEND: $07AF -> $0797 (1943 bytes)
; v2.4 (Sep 2026) - Added GOSUB/RETURN; golf pass; showcase exercises both.
;   - GOSUB/RETURN: TOK_CHARS only matches 1st ketter, so GOTO/GOSUB share a
;     row (as do RUN/RETURN) 
;   - Showcase: added "--- GOSUB/RETURN ---" section and updated Mandelbrot.
;   - Added RND_Shuffle pseduo random sequnce ready for RND keyword.
;   - ROMEND: $0774 -> $07DA 
; v2.3 (Sep 2026) - Fixed broken Mandelbrot output; golf pass.
;   - Golf pass
;   - ROMEND: $0784 -> $0774 (1925 -> 1908 bytes)
; v2.2 (Sep 2026) - CHR$(n) and TAB(n) added to PRINT; WR statement removed
;   - Showcase updated
;   - ROMEND: $0785 (1925 bytes)
; v2.1 (Sep 2026) - Recursive parens SW stack BODMAS-lite restored on top.
;   - SWBASE grown to 64 bytes; MU_DONE now pushes HI_LOOP continuation so
;     */  chaining (e.g. "2*3*4", "100/5/2") evaluates left-to-right correctly.
;   - ROMEND: $0767 (1895 bytes)
; V2.0 (sep 2026) - 
;   - BODMAS layered on afterward: every "give me a fully-resolved value"
;     site now pushes {LO_LOOP, HI_LOOP} (HI checked first); LO operators'
;     own right-operand ALSO pushes {HI_LOOP} (so "3+4*5" groups as
;     "3+(4*5)"); HI operators' own right-operand does not - chaining
;     ("2*3*4") happens via DO_MUL/DO_DIV looping back to HI_LOOP directly,
;     preserving left-to-right associativity (required for correct "/"
;     chaining, e.g. "100/5/2"). TOK_CHARS reordered (*/  first) so each
;     tier's scan is a contiguous sub-range; DO_MUL's RXSAVE-based MUL-vs-
;     DIV check re-derived from the new table position (offset 2, not 8)
;     by simulator trace, not assumed.
;   - ROMEND: $075D (1885 bytes)
;
; Branched from pBASIC V0.22
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
ERR_OOM         EQU '3'         ; GOSUB/FOR stack full, RETURN/NEXT with nothing pushed, program store full
ERR_VAR         EQU '4'
ERR_EXPR        EQU '8'         ; Expression too complex (SW stack full)
GSSTKLIM        EQU 8           ; 4 GOSUB levels x 2 bytes
FSTKLIM         EQU 28          ; 4 FOR levels x 7 bytes
PROGLIM         EQU $1FFF       ; last usable program-store address. MUST be $xxFF (one
                                ; below a page boundary): OPEN_GAP checks the new PE's
                                ; hi byte only. Change for other RAM sizes (1 KB: $13FF)

; Error check to make sure we dont crash due to HW Stack overflow 
PE_RAS_LIMIT    EQU 7

; PSW Defines
PSW_RS          EQU     $10
PSW_WC          EQU     $08             ; WC (With Carry) bit in PSL (bit 3)
PSW_FLAG        EQU     $40

; REG16_TO_REG16 register indices (idx*2 = byte offset from IPH; see the
; "Ordered group" RAM comment). v2.11.
IDX_IP    EQU 0
IDX_TMP   EQU 1
IDX_GOTO  EQU 2
IDX_CUR   EQU 3
IDX_SWSTK EQU 4
IDX_EXP   EQU 5
IDX_LNUM  EQU 6
IDX_PE    EQU 8
IDX_SC0   EQU 9
IDX_RND   EQU 10

;  CODE starts at Zero (No Pipbug)
        ORG 0

; =============================================================================
;  RESET / ENTRY + PAGE-ZERO VECTOR TABLE
;
; Page-zero subroutine vector table.
; Each DW entry holds the absolute address of the subroutine.
; Callers use ZBRR/ZBSR *Vxxx (2 bytes) vs BCTA/BSTA,UN xxx (3 bytes)
;
RESET:
        BCTR,UN MAIN            ; trampoline over vector table ($0000)
;  Page-zero vector table for ZBSR/ZBRR *Vxxx (2 bytes vs 3 for BSTA/BCTA).
;  A vector costs 2 bytes and saves 1 per call, so an entry pays for itself
;  only from 3 callers up: keep every entry >= 3, and re-count after any
;  change that adds or removes calls. Only unconditional BSTA/BCTA convert
;  (there is no conditional ZB form). Reach: ZBSR/ZBRR take a 7-bit signed
;  displacement, so entries must sit at addresses $02..$3E (max 31 entries).
;  Do not point a vector at a routine that a "db $EC" skip trick skips over
;  unless the routine stays 2 bytes.
VINC_IP:
        DW INC_IP
VWSKIP:
        DW WSKIP
VINC_PC:
        DW INC_PC
VCOUT:
        DW COUT
VPARSE_EXPR:
        DW EXPR
VPRT_SPACE:
        DW PRT_SPACE
VCLR_EXP:
        DW CLR_EXP
VDO_ERROR:
        DW DO_ERROR
VSET_PC_PROG:
        DW SET_PC_PROG
VCMP_PC_PE:
        DW CMP_PC_PE
VNEG_EXP_BODY:
        DW NEG_EXP_BODY
VREG16_TO_REG16:
        DW REG16_TO_REG16    ; v2.11: generic nibble-packed 16-bit copy
VPUSH_RET:
        DW PUSH_RET
VPARSER_RET:
        DW PARSER_RET
VEATWORD:
        DW EATWORD
VEXPR_ATOM:
        DW EXPR_ATOM
VINC_ET:
        DW INC_ET
VDIGIT_CHECK:
        DW DIGIT_CHECK
VPRINT_S16:
        DW PRINT_S16

; =============================================================================
; MAIN - Program init
; =============================================================================
MAIN:
        ; Initialize RND seed
        LODI,R1 $AC
        LODI,R0 $E1
        BSTA,UN RND_SKIP

        PPSL $02                ; COM=1 (unsigned compare mode) for the entire

        ; RUNFLG/GSSP/FSP are cleared by REPL (CLR_RUNFLG) before every line.
        ; For a ROM build with no preloaded program, call DO_NEW here instead.

        ; print sign-on banner
        LODI,R0 <BANNER
        STRA,R0 IPH
        LODI,R0 >BANNER
        STRA,R0 IPL
        BSTA,UN PRTSTR
        BSTR,UN DO_FREE         ; free memory 
        ; fall through to REPL

; =============================================================================
;  REPL -- Main read-eval-print loop
; =============================================================================
REPL:
        CPSL PSW_RS + 5             ; primary reg bank; clear C/OVF/RS -
        CPSU $07                    ; clear PSU SP field (bits 2:0 = HW RAS depth)
        BSTA,UN CLR_RUNFLG          ; not running, GOSUB/FOR stacks empty
        LODI,R0 '>'                    ; print prompt only used here
        ZBSR *VCOUT  
        ZBSR *VPRT_SPACE  
        BSTA,UN GETLINE                  ; also sets IPH:IPL = IBUF
        BSTA,UN TRY_STORE_LINE           ; CC=EQ: line stored/deleted; CC=GT: not a line
        BSTA,GT STMT_EXEC               ; If CC=GT (not a line), execute
        BCTR,UN REPL

; =============================================================================
;  JSYNERR -- Syntax error jump
; In:  nothing (R0 irrelevant)
; Out: jumps to DO_ERROR
; Clobbers: R0
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
        LODI,R0 (IDX_CUR*16)+IDX_EXP
        ZBSR *VREG16_TO_REG16
        ZBSR *VPRINT_S16                ; [+1]
DE_NL:
        BSTR,UN PRT_CRLF
        BCTR,UN REPL                     ; REPL clears run state + RAS (PSU SP bits)

; =============================================================================
; DO_FREE
; Syntax: FREE
; Prints the number of free bytes in program store: PROGLIM - PEH:PEL.
; PROGLIM = $1FFF (top of RAM). Free = $1FFF - current program end pointer.
; Note May need to change if PROGLIM is not 0x1FF 
; In:   PEH:PEL = program end pointer
; Out:  free byte count printed to COUT followed by CR/LF
; Clobbers: R0, EXPH, EXPL (via PRINT_S16)
; =============================================================================
DO_FREE:
        ; Compute Low Byte: EXPL = $FF - PEL (Never borrows, may change)
        LODI,R0 >PROGLIM                 ; Load $FF
        SUBA,R0 PEL 
        STRA,R0 EXPL 

        ; Compute High Byte: EXPH = $1F - PEH
        LODI,R0 <PROGLIM                 ; Load $1F
        SUBA,R0 PEH 
        STRA,R0 EXPH 

        ZBSR *VPRINT_S16                ; Print decimal value
        ; drop through
; =============================================================================
;  Shared character print routines -- $EC (COMA) byte-skip chain
; Each entry loads its character then falls through via the skip opcode trick.
; PRT_CRLF prints CR LF, PRT_QUEST '?', PRT_CR CR, PRT_SPACE a space.
; In:  nothing     Out: the character(s) written to COUT
; Clobbers: R0
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
;  DO_PRINT / PRTSTR -- Print statement and NUL-terminated string helper
; Syntax: PRINT [item {; item}]
;   item = "string" | CHR$(n) | TAB(n) | expr
;   CHR$(n): output low byte of n as a raw character (replaces WR statement)
;   TAB(n):  output n spaces (n=0 is a no-op; not a column-seek)
; In:  IP -> first char after PRINT keyword
; Out: text written to COUT; IP advanced past statement
; Clobbers: R0, R1, R2, EXPH, EXPL, TMPH, TMPL, NEGFLG, LNUMH, LNUML, SC0, SC1
DP_STRING:
        ZBSR *VINC_IP           ; 2b
; PRTSTR -- print the string at IP up to a closing double quote or NUL
; In:  IP -> first char to print     Out: chars sent; IP at the quote/NUL
; Clobbers: R0, R1, R2 (COUT)
PRTSTR:
        LODA,R0 *IPH            ; 3b
        RETC,EQ                 ; 1b: NUL before closing quote -> exit
        COMI,R0 DQ              ; 2b
        BCTR,EQ DP_SCLS         ; 2b
        ZBSR *VCOUT             ; 2b
        BCTR,UN DP_STRING       ; 2b

DP_EXPR:
        ZBSR *VPARSE_EXPR       ; 2b
        ZBSR *VPRINT_S16       ; 2b
        db $EC                  ; 1b: COMA,R0 opcode - consumes next 2 bytes (ZBSR *VINC_IP)
DP_SCLS:
        ZBSR *VINC_IP           ; 2b
DP_SEP:
        ZBSR *VWSKIP       ; 2b
        COMI,R0 $3B             ; 2b: semicolon ';'
        BCFR,EQ PRT_CRLF        ; not ';' -> CRLF and done
        ZBSR *VINC_IP           ; 2b
        ZBSR *VWSKIP       ; 2b
        RETC,EQ                 ; 1b: trailing ';' followed by NUL -> exit without CRLF
        BCTR,UN DP_ITEM         ; 2b

DO_PRINT:
        ZBSR *VWSKIP       ; 2b
        BCTR,EQ PRT_CRLF        ; bare PRINT -> print CRLF & return
DP_ITEM:
        COMI,R0 DQ              ; 2b
        BCTR,EQ DP_STRING       ; 2b
        
        BSTR,UN PEEK_C2_ALPHA   ; 2b
        BCTR,EQ DP_EXPR         ; 2b: 2nd char not alpha -> expr
        COMI,R2 A'C'            ; 2b
        BCTR,EQ DP_KW           ; 2b
        COMI,R2 A'T'            ; 2b
        BCFR,EQ DP_EXPR         ; 2b
DP_KW:
        ZBSR *VEATWORD          ; 2b: consume keyword
        ZBSR *VPARSE_EXPR       ; 2b: parse (n)
        LODA,R0 EXPL            ; 3b: load low byte into R0 once for both CHR$ and TAB
        COMI,R2 A'C'            ; 2b
        BCTR,EQ DP_C_KW         ; 2b
        STRZ R1                 ; 1b: 2650 opcode $51 — copies R0 -> R1 & sets CC (EQ if 0)
        BCTR,EQ DP_SEP          ; 2b: TAB(0) no-op
DP_TAB_LOOP:
        ZBSR *VPRT_SPACE        ; 2b
        BDRR,R1 DP_TAB_LOOP     ; 2b
        BCTR,UN DP_SEP          ; 2b
DP_C_KW:
        ZBSR *VCOUT             ; 2b: print R0 directly
        BCTR,UN DP_SEP          ; 2b

; -----------------------------------------------------------------------------
; PEEK_C2_ALPHA - Checks if the 2nd character (IPH+1) looks like a letter.
; Inputs:   None (uses IPH)
; Outputs:  CC=EQ if char 2 is NOT a letter, else GT (bit 6 test: 'A'-'Z' have
;           it set, so do a few punctuation chars - garbage input only).
; Clobbers: R0, R1 (and R2 = first char on the PEEK_C2_ALPHA entry)
; -----------------------------------------------------------------------------
PEEK_C2_ALPHA:
        STRZ,R2            ; R2 = first char
PK_C2_NO_R2:                             ; same test, but leaves R2 alone
        LODI,R1 1
        LODA,R0 *IPH,R1                  ; peek 2nd char
        ANDI,R0 $40                      ; letters have bit 6 set; NUL, space, digits, '=' do not
        RETC,UN             ; Return to caller: EQ = not a letter

; =============================================================================
;  DO_IF -- Conditional execution; THEN is optional.
;  Nests for free: "IF a IF b stmt" - the true-path dispatch is a JUMP to
;  STMT_EXEC, so if "stmt" is itself another IF, it costs no extra depth.
; Syntax: IF expr [THEN] stmt   (THEN is skipped by the 'T' row of TOK_CHARS)
; In:  IP -> first char after IF keyword
; Out: executes stmt if expr is nonzero; otherwise sequential return
; Clobbers: R0, R1, EXPH, EXPL (via EXPR, plus whatever the dispatched
;           statement clobbers on the true path)
DO_IF:
        COMI,R3 A'P'            ; 3rd char of INPUT (R3 = 3rd keyword char, see STMT_EXEC)
        BCTA,EQ DO_ASK          ;

        ZBSR *VPARSE_EXPR                      ; [+1] condition -> EXPH:EXPL
        LODA,R0 EXPL
        IORA,R0 EXPH
        RETC,EQ                           ; both zero: false, return
        ; drop through
; =============================================================================
;  STMT_EXEC -- Decode and dispatch one BASIC statement from IP.
; In:  IPH:IPL -> first char of statement (after any leading whitespace)
; Out: control jumps to the matched DO_xxx handler, or falls into SE_NOTKW
; Clobbers: R0, R2, R1, GOTOH, GOTOL, IPH:IPL advanced past the keyword
;   on a match (unchanged on the bare-assignment path - see SE_NOTKW)
; RAS depth: 1 from REPL, 3 from DO_IF's true path (which jumps here).
STMT_EXEC:
        ZBSR *VWSKIP
        BSTR,UN PEEK_C2_ALPHA     ; R2 is 1st char
        BCTR,EQ SE_NOTKW          ; 2nd char is NOT a letter -> handle as var/expr
        LODI,R1 2                 ; peek 3rd char - disambiguates GOTO/GOSUB
        LODA,R0 *IPH,R1           ; and RUN/RETURN inside DO_GO/DO_RU
        STRZ,R3                   ; R3 = 3rd char, tested by the shared-row handlers
        LODI,R1 21               ; start scan at the first STATEMENT row -
                                   ; skips the 7 operator rows (21 = 7*3) so
                                   ; an operator-led line (e.g. "-A", 2nd
                                   ; char is a letter) can never match one
                                   ; and mis-dispatch into a DO_xxx handler;
                                   ; falls through to SE_NOTKW instead (see
                                   ; pBASIC PAREN-fix history, same table)
MD_SCAN:
        LODA,R0 TOK_CHARS,R1              ; table char
        BCTR,EQ SE_NOTKW                  ; NUL row: no match -> bare assignment
        COMZ,R2                          ; 1 byte vs SUBA's 3 (r0:r2 -> CC);
        BCTR,EQ MD_HIT
        ADDI,R1 3                         ; next row (char + 2-byte handler)
        BCTR,UN MD_SCAN
MD_HIT:
        ZBSR *VEATWORD                  ; consume the whole keyword -
        ; Jump into point from relop vectors
JMP_VEC:
        LODA,R0 TOK_CHARS,R1+              ; handler hi (pre-inc: char->hi)
        STRA,R0 GOTOH
        LODA,R0 TOK_CHARS,R1+              ; handler lo (pre-inc: hi->lo)
        STRA,R0 GOTOL
        BCTA,UN *GOTOH                    ; indirect jump


; SE_NOTKW -- bare assignment "V=expr" (also LET V=expr), reached when the
;  statement is not a keyword; falls into DL_EX/DL_STORE.
; In:  IP -> variable letter      Out: VARS[V] = value of expr
; Error: JERRVAR (not A-Z), JSYNERR (no '=')     Clobbers: R0, R1, R2, EXP, TMP
SE_NOTKW:
        ; Bare variable assignment ("X=expr" - either the 2nd-char peek
        ; above wasn't a letter, or the 1st char matched no statement).
        BSTR,UN PARSE_VAR_SAVE            ; validates A-Z, R2 = letter, IP -> past it
        ZBSR *VWSKIP
        COMI,R0 A'='
        BCFA,EQ JSYNERR
        ZBSR *VINC_IP
        ; drop through
; =============================================================================
;  DL_EX / DL_STORE -- Variable assignment (v0.3 removed DO_LET's own
;  prologue; v2.7 brings the optional LET keyword back at no handler cost:
;  the 'L' row lands in DO_LIST, which sends LET straight to SE_NOTKW.  Reached
;  from SE_NOTKW's "V=expr" path (bare or after LET) and from DO_ASK.)
; In:  IP -> expression (DL_EX) or R2 = VARS byte-offset, EXPH:EXPL = value
;      (DL_STORE - PARSE_VAR_SAVE already converts the letter to R2=index*2)
; Out: VARS[V] = EXPH:EXPL
; Clobbers: R0, EXPH, EXPL, TMPH, TMPL (via PARSE_EXPR, DL_EX only)
DL_EX:
        ZBSR *VPARSE_EXPR                 ; [+1]
DL_STORE:
        LODA,R0 EXPH     ; R0 = High byte of expression
        STRA,R0 VARS,R2  ; Store directly to VARS array + offset (R2 is a
        LODA,R0 EXPL     ; R0 = Low byte of expression                  valid
        STRA,R0 VARS+1,R2; Store directly to VARS array + offset + 1    index reg)
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
        BSTR,UN GETLINE                   ; [+1] also sets IPH:IPL = IBUF
        BSTA,UN PARSE_S16                ; [+1]
        BCTR,UN DL_STORE

; =============================================================================
; PARSE_VAR_SAVE -- skip whitespace, read var letter, range-check, save to
;                   R2, advance IP.
; In:  IP -> variable letter (leading spaces allowed)
; Out: R2 = VARS byte offset (index*2); IP advanced past the letter
; Error: tail-jumps to JERRVAR (no return)
; Clobbers: R0, R1, R2
PARSE_VAR_SAVE:
        ZBSR *VWSKIP
        SUBI,R0 A'A'                     ; shift 'A'-'Z' down to 0-25
        COMI,R0 A'Z'-A'A'                ; single unsigned range test (COM=1
        BCTR,GT JERRVAR                  ; set globally) catches < 'A' and > 'Z'
        ADDZ,R0                          ; R0 = index*2 (16-bit word stride)
        STRZ,R2                          ; R2 = byte offset in VARS, for DL_STORE
        ZBRR *VINC_IP         ; tail call  

; =============================================================================
;  GETLINE -- Read a line from input into IBUF; also points IP at it
; In:  nothing
; Out: IBUF = NUL-terminated input line; IPH:IPL = IBUF (both callers used
;      to re-point IP via a separate call right after this -
;      folded in here instead). Was a single shared LODI while IBUF sat at
;      a hi==lo address; IBUF moved (v2.11, nibble-index RAM reorg) so this
;      is now two LODIs (+2 bytes, accepted cost - see VERSION HISTORY)
; Clobbers: R0, R1
GETLINE:
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
        LODI,R1 $FF                      ; R1 = empty-buffer sentinel (pre-inc convention)
GL_LP:
        BSTA,UN PRE_CHIN                     ; [+1] blocking read
        COMI,R0 CR+1
        BCTR,LT GL_EOL                  ; everything less than CR
        STRA,R0 IBUF,R1+                 ; R1++ (pre-inc); IBUF[R1]=char
        ZBSR *VCOUT  
        BCTR,UN GL_LP
GL_EOL:
        EORZ,R0
        STRA,R0 IBUF,R1+                 ; R1++ (pre-inc, one past last char); NUL-terminate
        BCTA,UN PRT_CRLF                ; tail call

JERRDIVZER:
        LODI,R0 ERR_DIV_ZERO
        db $EC                          ; COMA,R0: consume next 2 bytes
JERRVAR:
        LODI,R0 ERR_VAR
        db $EC                          ; COMA,R0: consume next 2 bytes
DGS_OFLOW:                              ; all report ERR_OOM: GOSUB/FOR stack full,
DRT_UFLOW:                              ; RETURN/NEXT with nothing pushed, program
        LODI,R0 ERR_OOM                 ; store full (OPEN_GAP) - one shared exit
        ZBRR *VDO_ERROR

; DO_GO -- 'G' row: GOSUB (3rd char 'S') falls into DO_GOSUB, else GOTO.
; In:  R3 = 3rd keyword char, IP past the keyword      Out/Clobbers: see DO_GOSUB/DO_GOTO
DO_GO:
        COMI,R3 A'S'
        BCFR,EQ DO_GOTO
        ; drop through
; =============================================================================
;  DO_GOSUB -- Subroutine call, up to GSSTKLIM/2 levels deep.  
; In:  IP -> first char after keyword; SWSTK already holds "resume after
;      this line" (DR_LP sets it before every STMT_EXEC dispatch - see
;      DR_CD), i.e. exactly the return point GOSUB needs to save.
; Out: GSSTK[GSSP]=SWSTK, GSSP+=2, then behaves exactly like GOTO to
;      redirect SWSTK at the target line.
; Error: GSSP=GSSTKLIM (stack full) -> ERR_OOM
; Clobbers: R0, R1, GSSP, GSSTK (plus everything DO_GOTO clobbers)
DO_GOSUB:
        LODA,R1 GSSP
        COMI,R1 GSSTKLIM
        BCFR,LT DGS_OFLOW                 ; GSSP >= limit: stack full
        LODA,R0 SWSTK
        STRA,R0 GSSTK-1,R1+               ; pre-increment store: GSSTK[GSSP++]
        LODA,R0 SWSTK+1
        STRA,R0 GSSTK-1,R1+
        STRA,R1 GSSP      
        ; drop through
; =============================================================================
;  DO_GOTO -- Computed GOTO
; Syntax: GOTO expr
; In:  IP -> first char after GOTO keyword
; Out: if running, SWSTK = found record pointer (DR_CD resumes from there
;      unconditionally - see DR_CD); if not running (typed at the prompt,
;      outside RUN), a safe no-op, same as before.
; Clobbers: R0, EXPH, EXPL, LNUMH, LNUML, TMPH, TMPL, SWSTK (only if running)
DO_GOTO:
        ZBSR *VPARSE_EXPR                 ; [+1] EXPR_ATOM WSKIPs itself
        LODA,R0 RUNFLG
        RETC,EQ                           ; not running: safe no-op
        LODI,R0 (IDX_EXP*16)+IDX_LNUM
        ZBSR *VREG16_TO_REG16              ; LNUMH:LNUML = EXPH:EXPL (target line)
        BCTA,UN FIND_LINE                  ; SWSTK = found record (tail call)

; =============================================================================
;  DO_RETURN -- Return from subroutine.  Reached only from DO_RU.
; Syntax: RETURN
; Out: GSSP-=2; SWSTK = GSSTK[GSSP] (popped return point).
; Error: GSSP=0 (nothing pushed) -> ERR_OOM
; Clobbers: R0, R1, GSSP, SWSTK
DO_RETURN:
        LODA,R1 GSSP
        BCTR,EQ DRT_UFLOW                 ; GSSP==0: nothing to return to
        LODA,R0 GSSTK,R1-                 ; pre-decrement loads: lo, then hi
        STRA,R0 SWSTK+1
        LODA,R0 GSSTK,R1-
        STRA,R0 SWSTK
        STRA,R1 GSSP
        RETC,UN

; =============================================================================
;  DO_RU -- 'R' dispatch: REM (3rd char 'M') does nothing, RETURN (3rd char
;  'T') goes to DO_RETURN, anything else falls into DO_RUN.
; In:  R3 = 3rd keyword char     Out/Clobbers: see DO_RETURN / DO_RUN
DO_RU:
        COMI,R3 A'M'    ; REM
        RETC,EQ         ; Rem just returns as does nothing
        COMI,R3 A'T'
        BCTR,EQ DO_RETURN
        ; fall through: not RETURN -> plain RUN
; =============================================================================
;  DO_RUN -- Execute stored program
; Syntax: RUN
; In:  PROG=program base, PEH:PEL=program end
; Out: runs until END, error, or exhausted; returns to REPL
; Clobbers: all
; =============================================================================
DO_RUN:
        STRA,R3 RUNFLG                    ; R3 = 3rd keyword char ('N'): any nonzero = running
        ZBSR *VSET_PC_PROG
DR_LP:
        LODA,R0 RUNFLG
        RETC,EQ
        ; end of program? SWSTK (the program counter) >= PEH:PEL
        ZBSR *VCMP_PC_PE

        BCFR,LT CLR_RUNFLG                ; not LT (GT or EQ): PC>=PE, end
                                          ; of program - clear RUNFLG and
                                          ; return via CLR_RUNFLG's own
                                          ; RETC,UN. Bug fix: previously
                                          ; "BCTR,GT CLR_RUNFLG / RETC,EQ"
                                          ; left RUNFLG=1 on a clean end
                                          ; (TMP==PE exactly), so the next
                                          ; error typed at the prompt wrongly
                                          ; printed a stale "@line" suffix.

        ; save current line number for error reporting.  Indexed read only -
        ; SWSTK itself stays at the record's header start, which is exactly
        ; what ADV_PC_PAST_REC (below) expects.
        LODI,R1 2
DR_HDR:
        LODA,R0 *SWSTK,R1-                ; R1 2->1: PC[1]; 1->0: PC[0]
        STRA,R0 CURH,R1                   ; CURH+1 = CURL
        BRNR,R1 DR_HDR
        ; IP = PC (header start), via REG16_TO_REG16, then
        ; skip past the 2-byte header to the body - execute straight out
        ; of PROG, no copy. Bodies are NUL-terminated in storage now, so
        ; every NUL-terminated-string assumption already baked into
        ; STMT_EXEC/EXPR/etc. just works.
        LODI,R0 IDX_SWSTK*16              ; +IDX_IP (0)
        ZBSR *VREG16_TO_REG16
        
        ZBSR *VINC_IP
        ZBSR *VINC_IP
        ; advance the PC (SWSTK) past this whole record (header+body+terminator)
        ; to the next one: DR_LP's next iteration, or a GOTO's redirection (which
        ; overwrites SWSTK). Reuses the shared ADV_PC_PAST_REC scan that
        ; FIND_LINE and DEL_REC also use - one routine, one terminator.
        BSTA,UN ADV_PC_PAST_REC
        ; execute line
        BSTA,UN STMT_EXEC                ; [+1]
        BCTR,UN DR_LP                    ; resume from SWSTK unconditionally

; =============================================================================
;  DO_NEW -- Reset PE and IP
; Syntax: NEW
; In:  nothing
DO_NEW:
        ; PEH:PEL = PROG (via the PC: 6 bytes vs 10 for direct stores)
        ZBSR *VSET_PC_PROG
        LODI,R0 (IDX_SWSTK*16)+IDX_PE
        ZBSR *VREG16_TO_REG16
        ; fall through
; =============================================================================
;  DO_END -- Stop execution and clear all run state
; Syntax: END  (also called by DO_NEW, DO_ERROR, RESET)
; In:  nothing
; Out: RUNFLG=0
; Clobbers: R0
;DO_END:
CLR_RUNFLG:
        EORZ,R0
        STRA,R0 RUNFLG
        STRA,R0 GSSP        ; GOSUB stack empty at every RUNFLG-clear point
        STRA,R0 FSP         ; ... and the FOR stack
        RETC,UN

; =============================================================================
;  DO_N / DO_NEXT -- 'N' row: NEW's 3rd char is 'W', NEXT's is 'X'.
;  NEXT [var]: adds 1 to the innermost loop's variable (any var name after
;  NEXT is ignored); while var <= limit it jumps back to the body, else pops.
;  Frame is read back-to-front with pre-decrement loads.
; In:  FSP = bytes used
; Out: var+1; SWSTK = body (loop again) or frame popped (finished)
;      Error: ERR_OOM if there is no FOR active
; Clobbers: R0, R1, R2, R3, EXP, TMP  (R3 = frame index: CMP_OPS trashes R1)
DO_N:
        COMI,R3 A'X'
        BCFR,EQ DO_NEW
DO_NEXT:
        LODA,R3 FSP
        BCTA,EQ DRT_UFLOW                ; NEXT with no FOR
        SUBI,R3 7                        ; R3 = frame base
        LODI,R2 5       
DN_POP5:
        LODA,R0 FSTK-1,R3+               ; var, limit lo, limit hi, step lo, step hi ...
        STRA,R0 EXPH,R2-                 ; ... into FVAR, LNUML, LNUMH, EXPL, EXPH
        BRNR,R2 DN_POP5
        LODA,R2 FVAR
        LODI,R0 VARS-IPH                 ; ADD_CORE offset of VARS[R2] from IPH
        ADDZ,R2                          ; (VARS-IPH+$32 must stay < 256)
        BSTA,UN ADD_CORE                 ; EXP = step + var
        BSTA,UN DL_STORE                 ; var = var + step
        LODA,R0 FSTK-1,R3                ; step hi byte still in the frame (R3 = base+5)
        STRZ,R2                          ; R2 = step's sign
        LODI,R0 (IDX_LNUM*16)+IDX_TMP     ; TMP = limit (CMP_OPS's left operand)
        ZBSR *VREG16_TO_REG16
        BSTA,UN CMP_OPS                  ; limit (TMP) : var (EXP), signed
        BCTR,EQ DN_GO                    ; var = limit: one more pass
        SPSL                             ; R0 bit7 = CC1 = "limit below var"
        EORZ,R2                          ; ^ step sign: bit7 set = they disagree
        BCFR,LT DN_GO                    ; agree (up&step>=0, down&step<0): loop
DN_POP:
        SUBI,R3 5                        ; back to the frame base: pop it
        STRA,R3 FSP
        RETC,UN

DN_GO:
        LODI,R2 2
DN_BODY:
        LODA,R0 FSTK-1,R3+               ; body lo, body hi -> SWSTK: DR_CD
        STRA,R0 SWSTK,R2-                ; resumes there
        BRNR,R2 DN_BODY
        RETC,UN

; =============================================================================
;  EXPRESSION CONTINUATION PAGE -- OPS_HIT_RET, HI_LOOP, LO_LOOP, EP_RET,
;  NEG_RET and ABS_RET are the only addresses PUSH_RET ever pushes. They are
;  kept together in ONE 256-byte page so PUSH_RET needs just the low byte
;  (callers save a LODI each) and supplies the shared high byte itself.
;  Keep them in one page (the build checks this).
; =============================================================================
; OPS_HIT_RET -- continuation pushed by OPS_HIT_COM: runs once the right
;  operand is resolved. Pops the saved table offset into R1 and the left
;  operand into TMPH:TMPL, then dispatches the operator's handler via JMP_VEC.
; In:  EXPH:EXPL = right operand; SW stack top = [R1 offset][left lo][left hi]
; Out: jumps to the DO_xxx handler (TMP = left, EXP = right); R3 popped by 3
; Clobbers: R0, R1, TMPH:TMPL, GOTOH:GOTOL
OPS_HIT_RET:
        LODA,R0 SWBASE,R3-       ; R0 = top of stack (left hi); R3 = bytes used,
        STRA,R0 TMPH             ; pop = pre-decrement load
        LODA,R0 SWBASE,R3-
        STRA,R0 TMPL
        LODA,R0 SWBASE,R3-       ; saved R1 table offset
        STRZ,R1
        ; jump to vector
        BCTA,UN JMP_VEC

; HI_LOOP -- continuation: scan for a * or / at IP; none -> next continuation.
; In:  IP -> position after an operand      Out: dispatches OPS_HIT_HI, else PARSER_RET
; Clobbers: R0, R1
HI_LOOP:
        ZBSR *VWSKIP              ; R0 = char to match against operators (kept in R0)
        LODI,R1 3                        ; last HI-row's char offset (2 rows:
                                          ; * and /, walking down to row 0)
HI_SCAN:
        COMA,R0 TOK_CHARS,R1
        BCTR,EQ OPS_HIT_HI
        SUBI,R1 3
        BCFR,LT HI_SCAN                    ; loop while R1 still >=0
        ZBRR *VPARSER_RET                  ; no */  here - go check +-=<>

; OPS_HIT_LO / OPS_HIT_HI -- an operator matched at table offset R1: save the
;  left operand and push the continuations that resolve the right operand.
;  LO tier: pushes HI_LOOP (and LO_LOOP first for relops, whose right operand is
;  a whole + - chain). HI tier: right operand is a single atom.
; In:  R1 = TOK_CHARS row offset, EXPH:EXPL = left operand, IP at the operator
; Out: operator consumed, continuations pushed, jumps to EXPR_ATOM for the operand
; Clobbers: R0, R1 (restored by OPS_HIT_RET), R3
OPS_HIT_LO:
        BSTR,UN OPS_HIT_COM
        LODA,R0 SWBASE-5,R3       ; the table offset OPS_HIT_COM saved (it sits
        COMI,R0 12                ; under left+RET it pushed); rows 12+ are the
        BCTR,LT OHL_HI            ; relops (= < >)
        BSTR,UN PUSH_LOLOOP       ; relop: right operand is a whole + - chain
                                  ; (pushes LO_LOOP, then HI_LOOP on top)
        db $EC                    ; COMA,R0: skip OHL_HI's call (it pushed HI already)
OHL_HI:
        BSTR,UN PUSH_HILOOP       ; right operand can ITSELF start a higher-
                                  ; precedence chain ("3+4*5") - check HI
                                  ; first before this + is allowed to fire
        db $EC                           ; COMA,R0: consume next 2 bytes
OPS_HIT_HI:
        BSTR,UN OPS_HIT_COM
        ZBRR *VEXPR_ATOM         ; right operand is ONE atom only - further
                                  ; */  chaining ("2*3*4") happens via
                                  ; DO_MUL/DO_DIV looping back to HI_LOOP
                                  ; themselves, left-associatively, not here

; PUSH_LOLOOP / PUSH_HILOOP -- push the LO_LOOP and/or HI_LOOP continuation.
;  PUSH_LOLOOP pushes LO_LOOP then falls into PUSH_HILOOP, so one call stacks
;  both with HI_LOOP on top (it runs first); PUSH_HILOOP alone pushes HI only.
; In:  R3 = SW stack bytes used     Out: continuation(s) pushed (see PUSH_RET)
; Clobbers: R0
PUSH_LOLOOP:
        LODI,R0 >LO_LOOP
        ZBSR *VPUSH_RET
PUSH_HILOOP:
        LODI,R0 >HI_LOOP
        ZBRR *VPUSH_RET                 ; tail call

; OPS_HIT_COM -- shared by OPS_HIT_LO/HI: stack [R1][left lo][left hi], skip
;  the operator char, then push the OPS_HIT_RET continuation.
; In:  R1 = table offset, EXPH:EXPL = left operand     Out: 3 bytes + RET pushed
; Clobbers: R0, R1 (=0), R3 (SW stack index)
OPS_HIT_COM:
        ; Push the R1 table offset first (survives recursion), then EXPL, EXPH
        LODZ,R1
        STRA,R0 SWBASE-1,R3+             ; push = pre-increment store (R3 = bytes used)
        LODI,R1 2
OHC_PUSH:
        LODA,R0 EXPH,R1-                 ; R1 2->1: EXPL; 1->0: EXPH (pre-decrement)
        STRA,R0 SWBASE-1,R3+
        BRNR,R1 OHC_PUSH
        ZBSR *VINC_IP            ; consume the operator char
        LODI,R0 >OPS_HIT_RET
        ZBRR *VPUSH_RET         ; tail call

; LO_LOOP -- continuation: scan for + - = < > (and the '!' invert prefix) at IP;
;  no operator -> next continuation (PARSER_RET).
; In:  IP -> position after an operand      Out: dispatches OPS_HIT_LO, else PARSER_RET
; Clobbers: R0, R1, BANG
LO_LOOP:
        ZBSR *VWSKIP                     ; R0 = char to match (kept in R0)
        LODI,R1 18                       ; last LO-row's char offset (5 rows:
                                          ; + - = < >, walking down to row 6 -
                                          ; HI's own 2 rows sit below that)
LO_SCAN:
        COMA,R0 TOK_CHARS,R1
        BCTR,EQ OPS_HIT_LO
        SUBI,R1 3
        COMI,R1 6                         ; stop at LO's own lower bound, not
        BCFR,LT LO_SCAN

        ; '!' relop-invert modifier: not one of the 7 known operators, but
LO_NOMATCH:
        COMI,R0 A'!'
        BCFA,EQ PARSER_RET      ; not bang so return
        LODI,R0 $FF
        STRA,R0 BANG                      ; armed - consumed once by whatever
        ZBSR *VINC_IP                     ; consume '!'
        BCTR,UN LO_LOOP                    ; loop - look for the real relop

; EP_RET / NEG_RET -- continuations. EP_RET: after "(expr": skip to and consume
;  the ')' then continue as NEG_RET's tail. NEG_RET: negate EXP (unary minus).
; In:  EXPH:EXPL = value     Out: EXP (negated for NEG_RET); resumes via PARSER_RET
; Clobbers: R0, R1
EP_RET:
        ZBSR *VWSKIP  
        ZBSR *VINC_IP                       ; consume ')'
        ; drop through
        db $EC                  ; COMA,R0: consume next 2 bytes
NEG_RET:
        ZBSR *VNEG_EXP_BODY
        ZBRR *VPARSER_RET

; ABS_RET -- continuation for ABS(...): EXPR_ATOM (via EA_PAREN) has already
; fully resolved the parenthesised expression, '(' and ')' included, by the
; time control reaches here - so this just takes the absolute value of
; whatever EXPR_ATOM left in EXPH:EXPL and returns.
ABS_RET:
        BSTA,UN ABS_EXP
        ZBRR *VPARSER_RET

; =============================================================================
;  RND_SHUFFLE -- Advance 16-bit Galois LFSR (Little-Endian) in place
; In:  None (reads RNDSEED)
; Out: RNDSEED advanced one step
; Clobbers: RNDSEED, PSL CC and Carry. Works in register bank 1 so the
;      caller's bank-0 R0/R1 survive (GETLINE's index and DO_RND's counter
;      rely on this). RS/WC are clear on return, as every caller expects.
RND_SHUFFLE:
        ; 16-bit Galois LFSR. R0=high byte, R1=low byte.
        ; CRITICAL: WC (PSL bit 3) must be SET for RRR to chain carry between
        ; registers. Without it, RRR is a per-register circular rotation (bit0
        ; wraps into bit7 of the SAME register) - the 16-bit shift breaks.
        ; Confirmed via -w watchpoint: seed cycled back to start after 8 steps.
        ; Sequence: shift R0 right (bit0 -> Carry), then R1 right (Carry ->
        ; bit7 of R1, R1's bit0 -> Carry as the feedback bit). TPSL 1 captures
        ; that feedback: CC=EQ if C=1 (apply XOR), CC=LT if C=0 (skip).
        ; Taps 0xB400 = x^16+x^14+x^13+x^11+1, standard maximal-length
        ; polynomial (period 65535).
        PPSL    PSW_RS+PSW_WC   ; bank 1 (keeps caller's R0/R1) + WC on
        LODA,R0 RNDSEED         ; Load seed high byte
        LODA,R1 RNDSEED+1       ; Load seed low byte
        CPSL    1               ; Clear Carry (C=0 shifts into bit7 of R0)
        RRR,R0                  ; Shift R0 right: bit0 of R0 -> Carry; 0 -> bit7
        RRR,R1                  ; Shift R1 right: Carry (bit0 of R0) -> bit7 of R1
                                 ;                 bit0 of R1 -> Carry (feedback)
        TPSL    1               ; Test Carry: CC=EQ if C=1, CC=LT if C=0
        BCTR,LT RND_SKIP        ; C=0 (CC=LT): feedback bit was 0, skip XOR
        EORI,R0 $B4             ; Apply taps high byte (0xB400)
; RND_SKIP -- store R0:R1 as the new seed and return (also MAIN's seed init)
RND_SKIP:
        STRA,R0 RNDSEED        ; Save seed high byte
        STRA,R1 RNDSEED+1      ; Save seed low byte
        CPSL    PSW_RS+PSW_WC   ; back to bank 0, WC off
        RETC,UN

; =============================================================================
; Character IO
; 110 Baud teletype from PIPBUG V1 as per Signetics M20 application note
; Simulator default expects CHIN $286 COUT $2B4
 ;       ORG $286

; PRE_CHIN -- stir the RND LFSR once per keystroke read, then fall into CHIN
; In:  nothing
; Out: R0 = character (from CHIN)
; Clobbers: R0, RNDSEED (R1/R2 preserved: RND_SHUFFLE uses register bank 1)
PRE_CHIN:
        BSTR,UN RND_SHUFFLE 
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

;  DLAY / DLY -- one 110-baud bit time (DLY enters part-way, for half-bit use)
; In:  nothing     Out: nothing     Clobbers: R0
DLAY:
        EORZ,R0
        BDRR,R0 $
        BDRR,R0 $
DLY:
        BDRR,R0 $
        LODI,R0 $e5
        BDRR,R0 $
        RETC,UN

; =============================================================================
;  COUT -- Transmit R0 as one serial character (start bit, 8 data bits LSB
;  first, stop) on the FLAG output, bit-banged at 110 baud.
; In:  R0 = character      Out: character sent
; Clobbers: R0 (works in register bank 1, so R1/R2 of the caller survive)
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
;  CMP_PC_PE -- Unsigned 16-bit compare of the PC (SWSTK) against PEH:PEL
;  (byte-serial; COMA under COM=1 gives a true unsigned 3-way result per byte).
; In:  SWSTK, PEH:PEL
; Out: CC=GT if PC > PE, CC=EQ if PC = PE, CC=LT if PC < PE
; Clobbers: R0
CMP_PC_PE:
        LODA,R0 SWSTK
        COMA,R0 PEH
        RETC,GT                                   ; hi bytes differ: that is the answer
        RETC,LT
        LODA,R0 SWSTK+1                           ; hi equal: lo bytes decide
        COMA,R0 PEL
        RETC,UN

; =============================================================================
;  TRY_STORE_LINE -- Store or delete a numbered line if IP starts with a digit
; In:  IPH:IPL -> input buffer
; Out: CC=GT if not a numbered line (caller executes it); CC=EQ if the line was
;      stored/deleted
; Clobbers: R0, R1, R2, R3, EXPH, EXPL, LNUMH, LNUML, IPH, IPL, SWSTK, PEH, PEL
;   (PE is updated as intended; R3 = record size while a line is being stored)
TRY_STORE_LINE:
        ZBSR *VDIGIT_CHECK
        RETC,GT                          ; not a digit: CC=GT, not a numbered line
        BSTA,UN PARSE_S16                ; [+1] IP already on the digit just tested
        LODA,R0 EXPH                     ; digits only, so bit 7 set means the
        BCTA,LT JSYNERR                  ; number is > 32767 ("negative"): reject
        LODI,R0 (IDX_EXP*16)+IDX_LNUM
        ZBSR *VREG16_TO_REG16             ; LNUMH:LNUML = parsed line number
TSL_FND:
        BSTA,UN FIND_LINE                ; [+1] PC = insertion point (first record
                                          ; >= LNUM); CC=EQ iff that record IS LNUM
        BCFR,EQ TSL_WRITE                 ; no such line: go store the new one
        BSTR,UN DEL_REC                   ; remove the old copy, then re-find: the
        BCTR,UN TSL_FND                   ; insertion point is the same address, but
                                          ; PC was consumed by DEL_REC
TSL_WRITE:
        ZBSR *VWSKIP
        RETC,EQ                           ; NUL: empty body = delete only (CC=EQ: done)
        LODI,R3 $FF                       ; R3 = record size = 3 + strlen(body):
TSL_LEN:                                  ; 2 header + body + NUL. Pre-increment
        LODA,R0 *IPH,R3+                  ; index scan of the body at IP; R3 ends at
        BCFR,EQ TSL_LEN                   ; the NUL's index (= strlen)
        ADDI,R3 3
        BSTR,UN OPEN_GAP                  ; make room at PC; PE += R3
        LODI,R1 $FF                       ; R1 = dest index into the gap (pre-increment)
        LODA,R0 LNUMH                     ; 2-byte line-number header
        STRA,R0 *SWSTK,R1+
        LODA,R0 LNUML
        STRA,R0 *SWSTK,R1+
        LODI,R2 $FF                       ; R2 = source index into the body (pre-increment)
TSL_CPY:
        LODA,R0 *IPH,R2+                  ; copy the body through its NUL terminator
        STRA,R0 *SWSTK,R1+
        BCFR,EQ TSL_CPY                   ; STRA leaves CC alone: loop until NUL (CC=EQ)
        RETC,UN                           ; PE was already updated by OPEN_GAP; CC=EQ

; =============================================================================
;  DEL_REC -- Delete the stored record at PC; close the hole; PE -= its length
;  Forward copy: src = PC (start of the next record), dst = EXP (start of the
;  deleted one). When src reaches PE, dst IS the new PE - no length needed, and
;  deleting the last record simply runs the loop zero times.
; In:  PC = start of the record to delete
; Out: PE reduced by the record's length
; Clobbers: R0, SWSTK, EXPH:EXPL
DEL_REC:
        LODI,R0 (IDX_SWSTK*16)+IDX_EXP
        ZBSR *VREG16_TO_REG16                ; EXP = dst = start of the doomed record
        BSTA,UN ADV_PC_PAST_REC           ; PC = src = start of the next record
DEL_LP:
        ZBSR *VCMP_PC_PE
        BCFR,LT DEL_END                  ; src >= PE: everything moved
        LODA,R0 *SWSTK
        STRA,R0 *EXPH
        ZBSR *VINC_PC
        LODI,R0 EXPH-IPH
        ZBSR *VINC_ET                   ; dst++
        BCTR,UN DEL_LP
DEL_END:
        LODI,R0 (IDX_EXP*16)+IDX_PE
        ZBRR *VREG16_TO_REG16               ; PE = dst (tail call)

; =============================================================================
;  OPEN_GAP -- Open an R3-byte gap at PC by moving [PC,PE) up R3 bytes, and
;  advance PE by R3, refusing (ERR_OOM) if the new PE would pass PROGLIM.
;  The new PE is built first, in EXP (PE + R3 via R3 INC_ET steps, R2 counting),
;  and range-checked before anything is touched.  The backward copy then walks
;  PE itself down as the source pointer, so the existing CMP_PC_PE is the loop
;  bound (PC = fixed low end); dst = src + R3 comes free from indirect-indexed
;  STRA (*PEH,R3) - no second pointer to decrement.  PE = EXP at the end.
; In:  PC = insertion point; R3 = gap size (1..255); PE = end of store
; Out: gap open at PC (contents stale); PE += R3; PC, R3 unchanged.
;      Store full: jumps to DO_ERROR (ERR_OOM); PE and the store are unchanged.
; Clobbers: R0, R1, R2, EXPH:EXPL
OPEN_GAP:
        LODI,R0 (IDX_PE*16)+IDX_EXP      ; EXP = PE
        ZBSR *VREG16_TO_REG16
        LODZ,R3
        STRZ,R2                          ; R2 = size, as the INC_ET step count
OG_ADD:
        LODI,R0 EXPH-IPH
        ZBSR *VINC_ET                   ; EXP++ (R2 times: EXP = PE + size)
        BDRR,R2 OG_ADD
        LODA,R0 EXPH
        COMI,R0 <PROGLIM+1
        BCFA,LT DRT_UFLOW                ; new PE is past PROGLIM: out of memory
OG_LP:
        ZBSR *VCMP_PC_PE
        BCFR,LT OG_FIX                   ; PE has walked down to PC: done
        LODI,R1 PEH-IPH                  ; PE -= 1 (borrow chained into the hi byte)
        BSTR,UN DEC16
OG_CPY:
        LODA,R0 *PEH                     ; byte at PE ...
        STRA,R0 *PEH,R3                  ; ... goes to PE+R3
        BCTR,UN OG_LP
OG_FIX:
        LODI,R0 (IDX_EXP*16)+IDX_PE
        ZBRR *VREG16_TO_REG16               ; PE = EXP (tail call)

; =============================================================================
;  DEC16 -- 16-bit decrement of the pair at IPH+R1:IPL+R1, borrow chained via WC
;  (shared by OPEN_GAP's PE walk and MULT_LOOP's count).
; In:  R1 = offset of the hi byte from IPH (e.g. TMPH-IPH)
; Out: value -= 1; C (PSL bit 0) = 0 iff the value was 0 (borrow out), see TPSL $01
; Clobbers: R0
DEC16:
        LODA,R0 IPL,R1
        SUBI,R0 1                        ; WC=0 here: C=0 iff the lo byte was 0
        STRA,R0 IPL,R1
        PPSL PSW_WC                      ; WC=1: the high byte takes the borrow
        LODA,R0 IPH,R1
        SUBI,R0 0
        STRA,R0 IPH,R1
        CPSL PSW_WC
        RETC,UN

; =============================================================================
;  ADV_PC_PAST_REC -- Advance the PC (SWSTK) past the current stored line record
; Skips the 2-byte line-number header, scans forward until NUL (end of that
; record's text), then skips the NUL too - leaves SWSTK pointing at the
; start of the NEXT record (or PE, if this was the last one).
; In:  SWSTK -> start of a stored record (its line-number hi byte)
; Out: SWSTK -> start of the next record
; Clobbers: R0
ADV_PC_PAST_REC:
        ZBSR *VINC_PC
APR_LP:
        ZBSR *VINC_PC
        LODA,R0 *SWSTK
        BCFR,EQ APR_LP                  ; free zero-test: NUL ends the body
        ZBRR *VINC_PC                     ; skip the NUL itself

; =============================================================================
;  FIND_LINE -- Find line LNUMH:LNUML, or its sorted insertion point
; Walks the store for the first record with line >= LNUMH:LNUML. The CC of the
; compare that stopped the walk doubles as the found flag, so no re-check.
; In:  LNUMH:LNUML = target line number
; Out: SWSTK = first record with line >= target, or PEH:PEL if none
;      CC=EQ iff that record IS the target line (found); else CC=LT/GT (not found)
; Clobbers: R0, R1, SWSTK
FIND_LINE:
        ZBSR *VSET_PC_PROG
        db $EC                            ; COMA,R0 -- consume next 2 bytes
FI_ADV:
        BSTR,UN ADV_PC_PAST_REC
        ZBSR *VCMP_PC_PE
        BCTR,LT FI_CHK                    ; PC < PE: examine this record
        LODI,R0 1                         ; at/past the end: CC=GT, not found
        RETC,UN
FI_CHK:
        LODI,R1 0               ; Start index at 0 (High byte)
CHK_LP:
        LODA,R0 LNUMH,R1        ; Load target line number byte (R1=0: High, R1=1: Low)
        COMA,R0 *SWSTK,R1       ; Unsigned compare with PC record's line number byte
        BCTR,GT FI_ADV          ; If target > current, current line is smaller, keep searching
        RETC,LT                 ; If target < current, found the exact insertion point
        
        ; bytes are EQUAL, check if we need to test the low byte
        EORI,R1 1               ; XOR R1 with 1 (Toggles 0 -> 1 -> 0) & updates CC flags
        BCFR,EQ CHK_LP          ; If result is not 0 (we just did High byte), loop for Low byte
        RETC,UN                 ; If result is 0 (both bytes matched exactly), return

; =============================================================================
; DO_RND -- RND atom: step the LFSR 8 times, return the seed as the value
; In:  IP -> "RND"
; Out: EXPH:EXPL = new seed; IP advanced; tail-jumps to PARSER_RET
; Clobbers: R0, R1, RNDSEED. R2 (target var offset) and R3 (SW stack pointer)
;      are live across EXPR and must NOT be used here.
DO_RND:
        ZBSR *VEATWORD          ; consume "RND"
        LODI,R1 8               ; shuffle a full byte's worth of taps
RNDF_MIX:
        BSTA,UN RND_SHUFFLE     ; preserves R0/R1 (bank 1)
        BDRR,R1 RNDF_MIX
        LODI,R0 (IDX_RND*16)+IDX_EXP
        ZBSR *VREG16_TO_REG16
        ZBRR *VPARSER_RET

; -----------------------------------------------------------------------------
;  DO_ABS -- ABS(expr) atom: pushes the ABS_RET continuation, then parses the
;  parenthesised operand through EXPR_ATOM (EA_PAREN consumes the brackets).
; In:  IP -> "ABS(expr)"
; Out: (via ABS_RET) EXPH:EXPL = |expr|; IP advanced past ')'
; Clobbers: R0 (plus whatever EXPR_ATOM and ABS_RET clobber)
DO_ABS:
        ZBSR *VEATWORD          ; consume "ABS"
        LODI,R0 >ABS_RET
        BCTR,UN EX_PRA          ; push RET and jump to EXPR_ATOM

; =============================================================================
;  EXPR -- Expression evaluator (entry for every "give me a value" site).
;  Empties the SW continuation stack, pushes the LO_LOOP/HI_LOOP continuations
;  and falls into EXPR_ATOM; the continuations then run the operator scans.
; In:  IPH:IPL -> expression string
; Out: EXPH:EXPL = 16-bit result ($0000/$FFFF for relops - see DO_EQOP etc.);
;      IP advanced past the expression
; Clobbers: R0, R1, R3 (SW stack index, reset), BANG, NEGFLG, SC0, SC1,
;      TMPH:TMPL, EXPH:EXPL.  R2 (target variable) is preserved.
EXPR:
        EORZ,R0
        STRZ,R3                          ; R3 = 0: SW cont-stack empty (bytes used)
        STRA,R0 BANG                      ; relop-invert modifier off
        BSTA,UN PUSH_LOLOOP                 ; pushes LO_LOOP ("once the HI-tier chain is
                                            ; exhausted, scan for +-=<>") then HI_LOOP
                                            ; ("once the first atom resolves, check
                                            ; for */ chaining first")
        ; drop through
; =============================================================================
;  EXPR_ATOM -- parse one atom: unary -, parens, RND, ABS, or a literal/variable.
; In:  IPH:IPL -> atom
; Out: EXPH:EXPL = value
; Clobbers: R0, R1 (RND path only, via TRY_RND/GET_RND); R2 is NOT clobbered
;   here or by anything this calls - see REGISTER CONVENTIONS re: R2
EXPR_ATOM:
        ZBSR *VWSKIP            ; returns with R0 = char[0]
        COMI,R0 A'-'
        BCTR,EQ EA_NEG
        COMI,R0 A'('
        BCTR,EQ EA_PAREN

        ; --- FUNCTION CHECK ---
        BSTA,UN PK_C2_NO_R2
        BCTR,EQ END_FUNCS

        ; 2nd char is a letter. Check char[0] for known functions
        ZBSR *VWSKIP            ; re-peek with R0 = char[0]
        COMI,R0 A'R'
        BCTR,EQ DO_RND          ; Starts with 'R'? Try RND
        COMI,R0 A'A'
        BCTR,EQ DO_ABS          ; Starts with 'A'? Try ABS
        ; drop through not a known function
END_FUNCS:
        BSTA,UN PARSE_FACTOR    ; Parse as bare variable / factor (reloads *IPH itself)
        ZBRR *VPARSER_RET 

EA_NEG:
        ZBSR *VINC_IP  
        LODI,R0 >NEG_RET
EX_PRA:
        ZBSR *VPUSH_RET
        ZBRR *VEXPR_ATOM                   ; parse the operand

EA_PAREN:
        ZBSR *VINC_IP                       ; consume '('
        LODI,R0 >EP_RET
        ZBSR *VPUSH_RET                     
        BSTA,UN PUSH_LOLOOP                 ; LO_LOOP then HI_LOOP continuations
        ZBRR *VEXPR_ATOM                   

; =============================================================================
;  DO_ADD / DO_SUB / DO_MUL / DO_DIV / DO_EQOP / DO_LTOP -- flat operator
;  handlers. Each pops the left operand pushed by OPS_HIT, combines with
;  EXPH:EXPL (the just-parsed right operand), and jumps back to
;  EXPR_LOOP to look for more operators at the same precedence.
; In:  EXPH:EXPL = right operand; TMPH:TMPL = left operand (OPS_HIT_RET popped
;      it from the SW stack before dispatching here)
; Out: EXPH:EXPL = combined result; resumes at LO_LOOP (+ - and relops) or
;      HI_LOOP (* /) to look for more operators of the same tier
; Clobbers: R0, R1, plus per-operator (see each)
DO_SUB:
        ZBSR *VNEG_EXP_BODY              ; EXP = -EXP
        ; drop through
DO_ADD:
        LODI,R0 TMPH-IPH                 ; TMPH offset from IPH (= 2)
        BSTR,UN ADD_CORE                 ; EXP += TMP
        BCTA,UN LO_LOOP                  ; look for more + - = < >

; =============================================================================
;  ADD_CORE -- shared 16-bit indexed add: EXPH:EXPL += value at IPH+off.
;  Ported from pBASIC2650.asm v0.33. Replaces this file's two previously
;  independent 16-bit-add implementations: DO_ADD's old CARRY_INTO_EXPH tail
;  (deleted - it had exactly one caller, this one) and MULT_LOOP's own
;  inlined WC-chained add (see MULT_LOOP below, which now calls this too).
; In:  R0 = 16-bit field offset from IPH (e.g. TMPH-IPH, SC0-IPH)
; Out: EXPH:EXPL += *(IPH+off):*(IPL+off)
; Clobbers: R0, R1
ADD_CORE:
        STRZ,R1
        LODA,R0 EXPL
        ADDA,R0 IPL,R1
        STRA,R0 EXPL
        PPSL $08                          ; WC=1: the high byte takes the carry
        LODA,R0 EXPH
        ADDA,R0 IPH,R1
        STRA,R0 EXPH
        CPSL $08
        RETC,UN

; =============================================================================
;  CMP_OPS / DO_EQOP / DO_LTOP / DO_GTOP -- relop handlers. One shared signed
;  3-way compare of popped LH operand (TMPH:TMPL) vs RH (EXPH:EXPL), then each
;  handler just picks which CC means true; the result is $0000 (false) or
;  $FFFF (true) in EXPH:EXPL then CC bawed on desired test. 
;  All six relops are direct: <=  is  !>,  >=  is  !<, ;  <>/!=  is  !=.
; In:  TMP = left, EXP = right (CMP_OPS)
; Out: CC = LT/EQ/GT as left is <, =, > right (CMP_OPS); handlers jump on
;      to DOP_TRUE / DOP_FALSE
; Clobbers: R0, R1
CMP_OPS:
        LODA,R0 TMPH
        CPSL $02                          ; COM=0: signed compare of the hi bytes
        COMA,R0 EXPH                      ; left.hi : right.hi
        PPSL $02                          ; COM=1 again (CC unaffected)
        RETC,GT
        RETC,LT
        LODA,R0 TMPL                      ; hi bytes equal: unsigned lo compare
        COMA,R0 EXPL
        RETC,UN

DO_EQOP:
        BSTR,UN CMP_OPS
        BCTR,EQ DOP_TRUE
        BCTR,UN DOP_FALSE
DO_LTOP:
        BSTR,UN CMP_OPS
        BCTR,LT DOP_TRUE
        BCTR,UN DOP_FALSE
DO_GTOP:
        BSTR,UN CMP_OPS
        BCTR,GT DOP_TRUE
        ; drop through: false
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
        ZBRR *VPARSER_RET

; =============================================================================
;  PARSE_FACTOR -- Parse a single value (variable or literal)
; In:  IPH:IPL -> first char of factor
; Out: EXPH:EXPL = value
; Clobbers: R0, R1
PARSE_FACTOR:
        LODA,R0 *IPH
        SUBI,R0 A'A'                      ; shift 'A' down to 0 - also
                                          ; doubles as PF_LOADVAR's index
                                          ; below, so a letter is never
                                          ; re-subtracted
        COMI,R0 A'Z'-A'A'                 ; single unsigned range test
        BCTR,GT PF_NUM                    ; not A-Z -> literal/number path
                                          ; (R0's shifted value is discarded
                                          ; there - PARSE_S16 reloads *IPH
                                          ; fresh via EORZ,R0 first thing)

        ; fall through: A-Z letter, R0 = index (0..25) already computed
; =============================================================================
;  PF_LOADVAR -- Load variable value from VARS
; In:  R0 = index (0..25, PARSE_FACTOR's SUBI result - not re-subtracted
;      here); IP -> that letter char
; Out: EXPH:EXPL = variable value
; Clobbers: R0, R1
PF_LOADVAR:
        ADDZ,R0                          ; R0 = index*2 (word stride)
        STRZ,R1                          ; R1 = index*2 - survives INC_IP:
                                          ; the INC_ET family bank-switches
                                          ; (PSW_RS) so primary R1/R2/R3 are
                                          ; preserved across the call, only
                                          ; R0 is clobbered
        ZBSR *VINC_IP                     ; advance IP past the letter
        LODA,R0 VARS,R1                  ; hi byte
        STRA,R0 EXPH
        LODA,R0 VARS+1,R1               ; lo byte
        STRA,R0 EXPL
        RETC,UN

PF_NUM:
;       drop through
; =============================================================================
;  PARSE_S16 -- Parse signed decimal integer
; In:  IPH:IPL -> first char (optional '-' then digits)
; Out: EXPH:EXPL = signed 16-bit value
; Clobbers: R0, NEGFLG, EXPH, EXPL
PARSE_S16:
        LODA,R0 *IPH
        EORI,R0 A'-'                      ; R0 = 0 iff a minus sign
        STRA,R0 NEGFLG                    ; NEGFLG polarity: 0 = negate the
                                          ; result (see NEG_EXP), any other
                                          ; value = leave. Written
                                          ; unconditionally so no separate
                                          ; clear is needed anywhere.
        BCFR,EQ PS16_UN                   ; not '-' (EORI's CC survives the
                                          ; STRA - STRA never touches CC)
PS16_NEG:
        ZBSR *VINC_IP  
PS16_UN:
;       drop through
; =============================================================================
;  PARSE_U16 -- Parse unsigned decimal digits -> EXPH:EXPL
; Jumps to JSYNERR if no digits found.
; In:  IPH:IPL -> first digit char
; Out: EXPH:EXPL = value
; Clobbers: R0, R1, SC0, SC1, EXPH, EXPL, TMPH, TMPL
;PARSE_U16:
        ZBSR *VCLR_EXP
        ZBSR *VDIGIT_CHECK
        BCTA,GT JSYNERR
PU16_LP:
        ZBSR *VDIGIT_CHECK
        BCTR,GT NEG_EXP                    ; not a digit -> end of number
PU16_DIG:
        STRZ,R1                          ; digit -> R1 (must survive INC_IP,
                                         ; EXP16_TO_ET and CLR_EXP below,
                                         ; which all leave primary R1 alone)
        ZBSR *VINC_IP
        LODI,R0 (IDX_EXP*16)+IDX_SC0
        ZBSR *VREG16_TO_REG16              ; SC0:SC1 = EXP (the value so far)
        ZBSR *VCLR_EXP                   ; EXP = 0 ...
        LODZ,R1                          ; ... R1 consumed here - MULT_LOOP
                                         ; below now clobbers R1 (via
                                         ; ADD_CORE) but nothing after this
                                         ; point still needs it. R3, the live
                                         ; SW-continuation-stack pointer
                                         ; during expression parsing, is
                                         ; still untouched by MULT_LOOP, so
                                         ; still no save/restore needed there
                                         ; (unlike the old fixed-count BDRR
                                         ; loop)
        STRA,R0 EXPL                     ; ... then = the digit: MULT_LOOP
                                         ; *accumulates* into EXP, so
                                         ; preloading the digit gives
                                         ; digit + 10*value in one go (no
                                         ; separate final add/carry step)
        LODI,R0 10
        STRA,R0 TMPL                     ; TMP = 10: x10 via the shared MULT_LOOP
        EORZ,R0
        STRA,R0 TMPH
        BSTR,UN MULT_LOOP                ; EXP = digit + value*10
        BCTR,UN PU16_LP

; =============================================================================
;  NEG_EXP -- Negate EXPH:EXPL if NEGFLG says to
;  NEG_EXP_BODY -- Unconditional negate EXPH:EXPL
; In:  EXPH:EXPL = value; NEGFLG = flag (0 = negate, nonzero = leave)
; Out: EXPH:EXPL negated (two's complement) if NEGFLG==0
; Clobbers: R0, R1
NEG_EXP:
        LODA,R0 NEGFLG
        RETC,GT                          ; nonzero (either sign) -> leave;
        RETC,LT                          ; needs both since NEGFLG's non-
                                         ; zero producers use arbitrary
                                         ; byte patterns (PARSE_S16) or a
                                         ; specific negative one ($80, from
                                         ; MD_PREP/ABS_X below) - only a
                                         ; true zero means "negate"
NEG_EXP_BODY:
        LODI,R1 EXPH-IPH                 ; EXPH offset from IPH (= 4); R1 variant for NEG_SHARED
        BCTR,UN NEG_SHARED

; =============================================================================
;  ABS_EXP / ABS_X -- Absolute value of EXPH:EXPL (ABS_EXP) or of the 16-bit
;  value at IPH+R1 (ABS_X, R1 = hi byte offset); toggles NEGFLG if it was
;  negative.  Falls through into NEG_SHARED.
; In:  ABS_X: R1 = TMPH-IPH or EXPH-IPH.  NEGFLG = current flag
; Out: value = |value|; NEGFLG toggled ($00<->$80) if it was negative
; Clobbers: R0, R1
ABS_EXP:
        LODI,R1 EXPH-IPH                 ; EXPH offset from IPH (= 4)
ABS_X:
        LODA,R0 IPH,R1
        RETC,GT                          ; LODA's own CC separates
        RETC,EQ                          ; +ve (GT) / zero (EQ) / -ve (LT)
        LODA,R0 NEGFLG
        EORI,R0 $80                      ; toggle between the $00/$80 sentinels
        STRA,R0 NEGFLG
        ; fall through to NEG_SHARED (R1 is already the offset)

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
        LODZ,R1
        ZBRR *VINC_ET                   ; tail call: adds 1 (INC_ET uses alt bank R1)


; =============================================================================
;  MULT_LOOP -- Shared repeated-add core for MUL16's multiply and PU16's x10
; EXPH:EXPL += SC0:SC1, TMPH:TMPL times (TMP is consumed/left at 0).
; v2.8: the add step now calls ADD_CORE instead of inlining its own
; WC-chained add (see ADD_CORE, near DO_ADD). This is no longer a leaf and
; no longer "uses R0 only" - it clobbers R1 too via ADD_CORE. R3 (the live
; SW-continuation-stack pointer during expression parsing) is still
; untouched, which is what PU16_DIG's call site actually depends on - see
; the comment there. CARRY_INTO_EXPH (the old DO_ADD tail, single caller)
; is deleted; ADD_CORE now covers both call sites.
; Decrement-first with a borrow test, using WC (carry-in) for the high
; byte: TMP-- happens before the "was TMP already 0" question is even
; asked - the decrement's own borrow flag answers it, so there's no
; separate up-front zero test the way MU_LP used to need one.
; In:  EXPH:EXPL = running total; SC0:SC1 = value to add; TMPH:TMPL = count
; Out: EXPH:EXPL += SC0:SC1 * (original TMPH:TMPL)
; Clobbers: R0, R1, TMPH, TMPL, EXPH, EXPL
MULT_LOOP:
        LODI,R1 TMPH-IPH                 ; TMP -= 1
        BSTA,UN DEC16
        TPSL $01                         ; C=1 no borrow (EQ) / C=0 borrow (LT)
        RETC,LT                          ; TMP was 0: all additions done
        LODI,R0 SC0-IPH                  ; SC0 offset from IPH
        BSTA,UN ADD_CORE                 ; EXP += SC
        BCTR,UN MULT_LOOP

; =============================================================================
;  MD_PREP -- Common set-up for DO_MUL / DO_DIV
; In:  TMP = left operand, EXP = right operand
; Out: TMP = |left|; SC0:SC1 = |right|; EXP = 0; NEGFLG = $00 if the result is
;      to be negated (exactly one operand negative), else $80
; Clobbers: R0, R1, NEGFLG
MD_PREP:
        LODI,R0 $80
        STRA,R0 NEGFLG                   ; $80 = leave sign; toggled once per negative operand
        LODI,R1 TMPH-IPH                 ; TMPH offset from IPH (= 2)
        BSTR,UN ABS_X                    ; [+1]
        BSTR,UN ABS_EXP                  ; [+1]
        LODI,R0 (IDX_EXP*16)+IDX_SC0     ; SC0:SC1 = |EXP|
        ZBSR *VREG16_TO_REG16
        ZBRR *VCLR_EXP                   ; clear EXP (accumulator starts at 0); tail call

; =============================================================================
;  DO_MUL -- Signed 16-bit multiply: TMPH:TMPL * EXPH:EXPL -> EXPH:EXPL
;  (|left| added |right| times via MULT_LOOP; MU_DONE fixes the sign)
; In:  TMPH:TMPL = left operand; EXPH:EXPL = right operand
; Out: EXPH:EXPL = product; resumes at HI_LOOP
; Clobbers: R0, R1, NEGFLG, SC0, SC1, TMPH, TMPL
DO_MUL:
        BSTR,UN MD_PREP                  ; |TMP| -> TMP, |EXP| -> SC, EXP = 0
        BSTR,UN MULT_LOOP                ; shared repeated-add loop
        BCTR,UN MU_DONE

; -----------------------------------------------------------------------------
;  DO_DIV -- Signed 16-bit divide: TMPH:TMPL / EXPH:EXPL -> EXPH:EXPL
; Remainder is NOT preserved (nothing in this interpreter reads it - no MOD
; operator) - DV_LP subtracts destructively and stops on the final,
; unsuccessful subtract without backing it out, matching pBASIC's DIV16.
; In:  TMPH:TMPL = dividend; EXPH:EXPL = divisor
; Out: EXPH:EXPL = quotient; TMPH:TMPL left in an undefined state
; Clobbers: R0, NEGFLG, SC0, SC1, TMPH, TMPL
; Error: divisor=0 -> ERR_DIV_ZERO
DO_DIV:
        LODA,R0 EXPL                     ; check for zero divisor
        IORA,R0 EXPH
        BCTA,EQ JERRDIVZER
        BSTR,UN MD_PREP
        ; Div Loop - single-pass destructive subtract (WC-chained), no
        ; separate compare-then-subtract: subtract speculatively, then
        ; check the borrow. If it borrowed, TMP was already < divisor,
        ; so this iteration's (wrong) subtraction is simply abandoned -
        ; safe because the remainder is dead (see header).
DV_LP:
        LODA,R0 TMPL
        SUBA,R0 SC1
        STRA,R0 TMPL
        PPSL $08                          ; WC=1: high byte takes the borrow
        LODA,R0 TMPH
        SUBA,R0 SC0
        CPSL $08
        TPSL $01                          ; C=1 no borrow (EQ) / C=0 borrow (LT)
        BCTR,LT MU_DONE                   ; TMP < divisor: done
        STRA,R0 TMPH                      ; commit (R0=new TMPH; TPSL/CPSL left it)
        LODI,R0 EXPH-IPH        ; EXP offset from IPH (= 4); assembly-time expression
        ZBSR *VINC_ET         ; quotient++
        BCTR,UN DV_LP

; entry point for MUL16 done too (BCTR,UN MU_DONE after the MULT_LOOP call, above)
MU_DONE:
        BSTA,UN NEG_EXP
        ; No NEGFLG cleanup needed here: both producers (PARSE_S16, MD_PREP)
        ; write it unconditionally before it is ever read again
        ; Resume at HI_LOOP so that * / chaining is checked after this result.
        ; "2*3*4" correctly becomes (2*3)*4: after computing 2*3=6, we
        ; resume at HI_LOOP which finds the second *, computes 6*4=24.
        BCTA,UN HI_LOOP


; =============================================================================
;  DO_LIST -- Print stored BASIC lines 
; Syntax: LIST
; In:  PROG=program base, PEH:PEL=program end
; Out: whole program printed
; Clobbers: R0, R1, R2, R3, SWSTK, EXPH, EXPL, SC0.  LIST inside a running
;   program leaves PC = PE, so it ends the run (see KNOWN LIMITATIONS).
DO_LIST:
        COMI,R3 A'T'                      ; 'L' row is shared: LIST's 3rd char is
                                          ; 'S', LET's is 'T' - the word is already
        BCTA,EQ SE_NOTKW                  ; eaten, so LET = the bare assignment path
        ZBSR *VSET_PC_PROG
        db $EC                  ; COMA,R0: consume next 2 bytes
DLS_NL:
        ZBSR *VINC_PC                     ; skip over NUL
        BSTA,UN PRT_CRLF
DLS_LP:
        ; Check PC against program end
        ZBSR *VCMP_PC_PE
        RETC,EQ                           ; PC == PE (GT cannot happen: records end exactly at PE)

        ; Read+print line number, then rest of line verbatim.  Walks the PC
        ; (SWSTK) directly - PRINT_S16 does not touch it, so no copy is needed.
        LODI,R1 2                         ; read the 2-byte line-number header
DLS_HDR:
        LODA,R0 *SWSTK,R1-                ; R1 2->1: PC[1]; 1->0: PC[0]
        STRA,R0 EXPH,R1                   ; EXPH+1 = EXPL
        BRNR,R1 DLS_HDR
        BSTR,UN PRINT_S16
        ZBSR *VPRT_SPACE
;
        ZBSR *VINC_PC            ; Skip 1st byte of line number
DLS_BLP:
        ZBSR *VINC_PC            ; Skip 2nd byte / advance pointer
        LODA,R0 *SWSTK           ; Fetch char
        BCTR,EQ DLS_NL           ; On NUL -> jump to line end routine
        ZBSR *VCOUT              ; Print character
        BCTR,UN DLS_BLP           ; Loop back to top (increment + fetch)

; =============================================================================
;  PRINT_S16 -- Print signed 16-bit value EXPH:EXPL as decimal
; In:  EXPH:EXPL = signed value
; Out: decimal digits written to COUT
; Clobbers: R0, R1, R2, R3, SC0, EXPH, EXPL (SWSTK is NOT clobbered - DO_LIST
;   relies on that to walk the PC directly). Returns with WC=0 and calls COUT
;   only with WC=0. R2 is NOT saved/restored: none of the 3
;   callers (DO_PRINT, DO_LIST, DO_ERROR) need it back afterward - the
;   digit-printing path always clobbered it anyway, so the old save/
;   restore only ever fired on the zero-value path, for no live caller.
PRINT_S16:
        LODA,R0 EXPH             ; get high byte & establish CC
        BCFR,LT PS_DIGITS        ; not negative
        LODI,R0 A'-'
        ZBSR *VCOUT
        ZBSR *VNEG_EXP_BODY     ; Negate, making EXPH:EXPL positive
PS_DIGITS:
        EORZ,R0
        STRZ,R3                 ; R3 = OR of digit counts so far (0 = still leading zeros)
        LODI,R2 5               ; R2 = column counter 5..1; P10 tables are indexed R2-1
DIGIT_LOOP:
        PPSL $09                 ; WC=1, C=1 (no borrow in): the 16-bit subtract chains via carry
        LODI,R1 0                ; R1 = digit count
SUB_LOOP:
        LODA,R0 EXPL
        SUBA,R0 P10_LO-1,R2      ; Subtract low byte (C=1 on entry: first pass or no borrow last pass)
        STRA,R0 SC0              ; Save tentatively (was TMPL - PRINT_S16
                                 ; must not clobber TMP, see DO_LIST)
        LODA,R0 EXPH
        SUBA,R0 P10_HI-1,R2      ; Subtract high byte with borrow-in from the low byte
        BCTR,LT BORROW           ; If borrow occurred (C=0), result is LT. Stop subtracting.

        ; No borrow: commit result and subtract again
        STRA,R0 EXPH
        LODA,R0 SC0
        STRA,R0 EXPL
        BIRR,R1 SUB_LOOP         ; digit++ (flags untouched), never wraps to 0: always loops

BORROW:
        CPSL $08                 ; WC=0 from here (also for COUT, whose RRR would chain carry)
        LODZ,R1
        IORZ,R3
        STRZ,R3                  ; R3 |= digit count; CC=EQ while all digits so far are zero
        BCFR,EQ PRINT_IT         ; non-zero digit seen: print
        COMI,R2 1                ; Is it the final column (1s)?
        BCFR,EQ NEXT_DIG         ; not the final column -> skip the leading zero
PRINT_IT:
        LODZ,R1
        ADDI,R0 A'0'
        ZBSR *VCOUT              ; Print the character
NEXT_DIG:
        BDRR,R2 DIGIT_LOOP       ; next column down to the 1s
        RETC,UN                  ; Return to caller

; =============================================================================
;  SHARED 16-BIT POINTER INCREMENT  - INC_ET family
; INC_PC  : SWSTK += 1 (the program counter; offset SWSTK-IPH from IPH)
; INC_IP  : IPH:IPL  += 1    (offset 0 from IPH)
; INC_ET  : the 16-bit field at IPH+R0 += 1 (R0 = hi-byte offset from IPH)
; In:  R0 = field offset (INC_ET only)     Out: the field incremented
; Clobbers: R0 only (runs in register bank 1; R1-R3 of the caller survive)
; All share INC_ET body using register bank switch.
; Rule: NO BSTA inside these -- must not consume extra RAS depth.
; Offsets are assembly-time defines (e.g. EXPH-IPH=4) 
INC_PC:
        LODI,R0 SWSTK-IPH       ; PC offset from IPH (= 8); assembly-time expression
        db $E4                  ; COMI,R0 -- consume next 1 byte
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
        CPSL PSW_RS                 ; switch back to primary bank if not already
        RETC,UN

; =============================================================================
;  REG16_TO_REG16 -- generic 16-bit copy between any two IPH-relative
;  register pairs, addressed by a packed nibble pair (v2.11 code-golf:
;  replaces the single-parameter EXP16_TO_ET/TMP_TO_ET family below plus
;  5 previously-inline manual copy loops -- see VERSION HISTORY).
; In:  R0 = (SRC_IDX<<4)|DST_IDX; each idx*2 = byte offset from IPH.
;      Use the IDX_* EQUs above. Valid idx range 0-15 (offsets 0-30).
; Out: dest 16-bit value = source 16-bit value
; Clobbers: R0. Primary R1/R2/R3 fully preserved via CPSL PSW_RS.
; =============================================================================
REG16_TO_REG16:
        PPSL PSW_RS             ; switch to alternate register bank
        STRZ,R2                 ; alt-R2 = R0 (packed byte SSSSDDDD)
        ANDI,R2 $0F             ; isolate dest nibble -> R2 = 0000DDDD
        EORZ,R2                 ; R0 ^= R2 -> SSSS0000 (cancels the low nibble)
        RRR,R0                  ; /2
        RRR,R0                  ; /2
        RRR,R0                  ; /2 -> R0 = source offset
        STRZ,R1                 ; alt-R1 = R0 (source offset)
        RRL,R2                  ; *2 -> alt-R2 = dest offset
        LODA,R0 IPH,R1
        STRA,R0 IPH,R2
        LODA,R0 IPL,R1
        STRA,R0 IPL,R2
        CPSL PSW_RS             ; restore primary bank
        RETC,UN

; =============================================================================
;  DO_FOR -- FOR var=start TO limit        (step 1; the body always runs once)
;  Assignment reuses SE_NOTKW (R2 survives the expression). The frame keeps
;  the address of the line AFTER the FOR (SWSTK, which DR_LP has already set
;  to the next line) so NEXT can jump straight back to the body.
; In:  IP -> "var=start TO limit"
; Out: var = start; frame pushed [body hi][body lo][limit hi][limit lo][var]
;      Errors: JSYNERR (no TO), ERR_OOM (frames full)
; Clobbers: R0, R1, R2, EXP, TMP
DO_FOR:
        COMI,R3 A'R'    ; check for FREE (R3 = 3rd keyword char)
        BCFA,EQ DO_FREE
        ;        
        BSTA,UN SE_NOTKW                 ; var=start; R2 = var's VARS offset
        ZBSR *VWSKIP
        COMI,R0 A'T'
        BCFA,EQ JSYNERR                  ; must be TO
        ZBSR *VEATWORD
        ZBSR *VPARSE_EXPR                ; EXP = limit
        LODI,R0 (IDX_EXP*16)+IDX_LNUM
        ZBSR *VREG16_TO_REG16               ; LNUM = limit (safe across the STEP expression)
        ZBSR *VWSKIP
        COMI,R0 A'S'
        BCTR,EQ DF_STEP
        ZBSR *VCLR_EXP                   ; no STEP: EXP = 1
        LODI,R0 EXPH-IPH
        ZBSR *VINC_ET
        BCTR,UN DF_GOT
DF_STEP:
        ZBSR *VEATWORD
        ZBSR *VPARSE_EXPR                ; EXP = step
DF_GOT:
        LODA,R1 FSP
        COMI,R1 FSTKLIM
        BCFA,LT DRT_UFLOW                ; all frames in use
        STRA,R2 FVAR                     ; var offset joins SWSTK, EXP and LNUM in RAM
        LODI,R2 7
DF_PUSH:
        LODA,R0 SWSTK,R2-                ; SWSTK+6..+0 = FVAR, LNUML, LNUMH, EXPL, EXPH, SWSTKL, SWSTKH
        STRA,R0 FSTK-1,R1+               ; pre-increment store: FSTK[FSP++]
        BRNR,R2 DF_PUSH
        STRA,R1 FSP
        RETC,UN

; =============================================================================
;  TABLES 
; Powers of 10 Tables (1, 10, 100, 1000, 10000), indexed by column counter - 1
P10_HI:
        db $00, $00, $00, $03, $27
P10_LO:
        db $01, $0A, $64, $E8, $10

BANNER:
        DB CR, LF, "uBASIC 2.14", CR, LF, "Free:",NUL        

; -- Combined operator + statement dispatch table
; Format: [char][hi][lo], stride 3, NUL-terminated.
; Statement scan (STMT_EXEC/MD_SCAN) safely runs the WHOLE table because
; its 2nd-char letter-gate guarantees the char being matched is A-Z,
; which none of the 7 operator chars are - no bounding needed there.
; Operator scan (HI_LOOP/LO_LOOP) is explicitly bounded to the first 7
; entries (rows 0-1 = HI tier * /, rows 2-6 = LO tier + - = < >).  Adding an
; operator row means bumping LO_LOOP's start offset and MD_SCAN's start offset.

TOK_CHARS:
        DB "*", <DO_MUL,    >DO_MUL       ; * (HI tier, offset 0)
        DB "/", <DO_DIV,    >DO_DIV       ; / (HI tier, offset 3)
        DB "+", <DO_ADD,    >DO_ADD       ; + (LO tier, offset 6)
        DB "-", <DO_SUB,    >DO_SUB       ; - (LO tier, offset 9)
        DB "=", <DO_EQOP,   >DO_EQOP      ; = (LO tier, offset 12; relop)
        DB "<", <DO_LTOP,   >DO_LTOP      ; < (LO tier, offset 15; relop)
        DB ">", <DO_GTOP,   >DO_GTOP      ; > (LO tier, offset 18; relop)
;        DB "A", <DO_ASK,    >DO_ASK       ; ASK
        DB "E", <CLR_RUNFLG,>CLR_RUNFLG   ; END
        DB "G", <DO_GO,     >DO_GO        ; GOTO / GOSUB
        DB "I", <DO_IF,     >DO_IF        ; IF / INPUT
        DB "L", <DO_LIST,   >DO_LIST      ; LIST
        DB "F", <DO_FOR,    >DO_FOR       ; FOR/FREE
        DB "N", <DO_N,      >DO_N         ; NEW / NEXT
        DB "P", <DO_PRINT,  >DO_PRINT     ; PRINT
        DB "R", <DO_RU,     >DO_RU        ; RUN / RETURN
        DB "T", <STMT_EXEC, >STMT_EXEC    ; THEN: MD_HIT's EATWORD eats the word, then
                                          ; this re-dispatches the statement after it
        DB NUL

; Helpers all called by ZBxx

; =============================================================================
;  EATWORD -- Consume [A-Z$] chars at IP
; In:  IPH:IPL -> current position
; Out: IP advanced past word
; Clobbers: R0
EATWORD:
        LODA,R0 *IPH
        SUBI,R0 A'A'                      ; shift 'A' down to 0 (also lets
        COMI,R0 A'Z'-A'A'                 ; the '$' test below reuse R0
        BCFR,GT EW_ADV                    ; without restoring it first)
        COMI,R0 A'$'-A'A'                 ; '$' compared in the same shifted
        BCFR,EQ EW_RET                    ; frame (wraps mod 256; EQ test is
EW_ADV:
        ZBSR *VINC_IP 
        BCTR,UN EATWORD

; =============================================================================
;  WSKIP -- Skip whitespace, then peek the current char at IP into R0
; Out: R0 = *IPH; CC set by that load (EQ if NUL)
; In:  IPH:IPL -> current position
; Out: IPH:IPL -> first non-space char
; Clobbers: R0
WSKIP_LOOP:
        ZBSR *VINC_IP           ; Advance IP (2 bytes)
WSKIP:
        LODA,R0 *IPH            ; Read char at IP (3 bytes)
        COMI,R0 SP              ; Is it a space? (2 bytes)
        BCTR,EQ WSKIP_LOOP      ; Yes -> loop back to increment IP (2 bytes)
        IORZ,R0                 ; Refresh CC flags for R0 (1 byte; EQ iff NUL)
EW_RET:
        RETC,UN                 ; Return to caller (1 byte)

; =============================================================================
;  DIGIT_CHECK -- Is *IPH a decimal digit '0'-'9'?  (found via a duplicate-
;  byte-sequence scan: this 7-byte test was inlined 3x - TRY_STORE_LINE and
;  twice in PARSE_U16 - byte-for-byte identical each time, see PORT HISTORY)
; Out: R0 = char - '0'; CC=GT if not a digit (single unsigned range test)
; Clobbers: R0
DIGIT_CHECK:
        LODA,R0 *IPH
        SUBI,R0 A'0'
        COMI,R0 9
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
;  PUSH_RET / PARSER_RET / SWRETURN -- SW-managed call/return for expression
;  recursion (parens, operator right-operands, unary minus), replacing hw
;  RAS for this subsystem entirely. Ported from uBASIC2650's own mechanism.
;  In:  PUSH_RET: R0=lo byte of the continuation address to push (the hi byte
;       is the continuation page, see EXPRESSION CONTINUATION PAGE)
;  Out: PUSH_RET returns normally (RETC); PARSER_RET/SWRETURN never return -
;       they dispatch to whatever continuation is due (or, if the SW stack
;       is empty, do a real hw RETC to the true external caller of EXPR()).
;  Clobbers: R0 (all three)
PUSH_RET:
        COMI,R3 SWCAP_LIMIT                  ; R3 = bytes used (0 = empty), unsigned
        BCFR,LT PR_FULL                      ; no room for 2 more bytes
        STRA,R0 SWBASE-1,R3+                 ; push lo (pre-increment store)
        LODI,R0 <OPS_HIT_RET                 ; hi byte: the shared continuation page
        STRA,R0 SWBASE-1,R3+                 ; push hi
        RETC,UN
PR_FULL:
        LODI,R0 ERR_EXPR
        ZBRR *VDO_ERROR                      ; tail call and bail 

PARSER_RET:
        LODZ,R3                              ; R0 = R3 = bytes used
        RETC,EQ                              ; SW stack empty: real hw return
        ; drop through
SWRETURN:
        LODA,R0 SWBASE,R3-                   ; hi byte (topmost); pop = pre-decrement load
        STRA,R0 TEMPRETH
        LODA,R0 SWBASE,R3-                   ; lo byte
        STRA,R0 TEMPRETL
        BCTA,UN *TEMPRETH                    ; indirect jump to popped addr

; =============================================================================
;  SET_PC_PROG -- Set the PC (SWSTK) = PROG base address
; Clobbers: R0
SET_PC_PROG:
        LODI,R0 <PROG
        STRA,R0 SWSTK
        LODI,R0 >PROG
        STRA,R0 SWSTK+1
        RETC,UN
ROMEND: 

;  RAM variables -- sequential RES block 
 
        ORG     4096    ; half a 2650 8kbyte page

; --- Ordered group: offsets from IPH used by INC_ET/DEC_ET/NEG_SHARED and
; by REG16_TO_REG16's packed-nibble index scheme (see IDX_* EQUs, v2.11).
; IBUF's hi==lo GETLINE trick was dropped when this group grew past 16
; bytes to bring PE/SC0/RNDSEED into nibble range (+2B in GETLINE, noted
; in VERSION HISTORY).
IPH     RES 1       ; interpreter pointer hi       (INC_ET offset 0; IDX_IP=0)
IPL     RES 1       ; interpreter pointer lo
TMPH    RES 1       ; temp 16-bit hi               (INC_ET offset 2 = TMPH-IPH; IDX_TMP=1)
TMPL    RES 1       ; temp 16-bit lo
GOTOH   RES 1       ; pending target hi            (DEC_ET offset 8 = GOTOH-IPH; IDX_GOTO=2)
GOTOL   RES 1       ; pending target lo
CURH    RES 1       ; current line hi  (error reporting; IDX_CUR=3)
CURL    RES 1       ; current line lo

; SWSTK, EXP, LNUM and FVAR MUST stay contiguous, in this order: DO_FOR pushes
; the seven bytes with one loop and DO_NEXT pops them back the same way.
; During a FOR: EXP = step, LNUM = limit (LNUM is untouched by expression
; evaluation, so it can hold the limit while the STEP expression is parsed).
SWSTK   RES 2       ; PROGRAM COUNTER: while running, the next record to execute; also the
                     ; cursor of FIND_LINE/LIST/store/delete (IDX_SWSTK=4)
EXPH    RES 1       ; expression result hi                                     (IDX_EXP=5)
EXPL    RES 1       ; expression result lo
LNUMH   RES 1       ; scratch line number hi                                   (IDX_LNUM=6)
LNUML   RES 1       ; scratch line number lo
FVAR    RES 1       ; FOR: variable's VARS offset (staging for frame push/pop) - NOT nibble-indexed
NIBPAD  RES 1       ; unused - restores even alignment after odd-length FVAR so
                     ; everything below is reachable by REG16_TO_REG16 (idx*2)
PEH     DB <SHOWCASE_END       ; Program end pointer hi                        (IDX_PE=8)
PEL     DB >SHOWCASE_END       ; Program end pointer lo
SC0     RES 1       ; Scratch byte 0                                          (IDX_SC0=9)
SC1     RES 1       ; Scratch byte 1
RNDSEED  RES 2      ; pseudo random number                                    (IDX_RND=10)
FSP     RES 1       ; bytes used in FSTK: 0,7,14,21; 28=full - NOT nibble-indexed

; Buffers
IBUF    RES 64      ; Input buffer 64 bytes

; --- Remaining --- (order doesn't matter; not nibble-indexed)
TEMPRETH RES 1      ; SWRETURN scratch: popped continuation addr hi (was PDEPTH -
                     ; PDEPTH itself removed, see CHANGE HISTORY: paren nesting
                     ; no longer needs a separate SW-tracked depth counter, the
                     ; PUSH_RET/PARSER_RET continuation stack IS the tracking)
TEMPRETL RES 1      ; SWRETURN scratch: popped continuation addr lo

;  --- Flags & Stuff --- 
RUNFLG  RES 1       ; 0=immediate, nonzero=running (DO_RUN stores the 3rd keyword char)
NEGFLG  RES 1       ; Sign flag
BANG    RES 1       ; '!' relop-invert modifier: $00 clear, $FF armed -
GSSTK   RES 8       ; GOSUB return-address stack, 4 levels x [hi][lo]
GSSP    RES 1       ; GOSUB stack offset into GSSTK: 0,2,4,6; 8=full
FSTK    RES 28      ; FOR frames, 4 levels x 7 bytes, pushed in this order:
                    ; [VARS offset][limit lo][limit hi][step lo][step hi][body lo][body hi]

;  SW call stack -- used by EXPR and its continuations
; R3 = bytes used (0 = empty, grows up). Each frame = [lo][hi].
; Push: STRA,R0 SWBASE-1,R3+ (lo first), STRA,R0 SWBASE-1,R3+ (hi).
; Pop:  LODA,R0 SWBASE,R3- (hi first), LODA,R0 SWBASE,R3- (lo).
; Also now holds PUSH_RET/SWRETURN continuation addresses (2 bytes each),
; interleaved LIFO with OPS_HIT's own operand pushes - see CHANGE HISTORY.
SWBASE  RES 64      ; SW stack base. Guard fires with ERR_EXPR when full.
SWCAP_LIMIT EQU 62   ; PUSH_RET refuses a new 2-byte push at/past this R3
                     ; (bytes used) value: leaves 2 bytes of slack in the
                     ; 64-byte SWBASE for OPS_HIT_COM's unchecked 3-byte push

VARS    RES 52      ; A-Z variables 2 bytes each


; =============================================================================
;  Pre-loaded SHOWCASE program
;
;  Line format: <lineno_hi> <lineno_lo> <body_ASCII> <NUL>
;  Lines  10-190: feature demos (PRINT, WR, arithmetic, comparisons, GOTO loop)
;  Lines 300-510: Mandelbrot set renderer )
;
;  Format: DB hi,lo,"text",$00  -- hi-then-lo matches DR_EXEC record format.
;  $22=DQ $3B=semicolon  in-string chars that need escaping.
; =============================================================================
PROG:
        DB 0,10,"REM -- Rem does nothing --", $00
        DB 0,20,"PRINT ",$22,"-- uBASIC2650 Showcase --",$22,$00
        DB 0,30,"PRINT ",$22,"--- PRINT / CHR$ / TAB ---",$22,$00         ; 30
        DB 0,40,"PRINT CHR$(65);CHR$(66);CHR$(67)",$00                    ; 40  ABC via CHR$
        DB 0,43,"PRINT",$00                                                ; 43  newline
        DB 0,44,"PRINT TAB(4);",DQ,"Hi",DQ,$00                            ; 44  TAB(4) then "Hi"
        DB 0,50,"PRINT ",$22,"--- ARITHMETIC ---",$22,$00
        DB 0,60,"PRINT ",$22,"3+4=",$22,$3B,"3+4",$3B,$22,"  10-3=",$22,$3B,"10-3",$3B,$22,"  6*7=",$22,$3B,"6*7",$00
        DB 0,70,"PRINT ",$22,"20/4=",$22,$3B,"20/4",$00
        DB 0,80,"PRINT ",$22,"--- COMPARISONS ---",$22,$00
        DB 0,90,"IF 3<9 PRINT ",$22,"3<9 ok",$22,$00
        DB 0,100,"IF 7=7 PRINT ",$22,"7=7 ok",$22,$00
        DB 0,110,"IF 9!=2 PRINT ",$22,"9!=2 ok",$22,$00
        DB 0,120,"IF 9!<4 PRINT ",$22,"9!<4 ok",$22,$00
        DB 0,130,"IF 9>4 PRINT ",$22,"9>4 ok",$22,$00                      ; native '>'
        DB 0,131,"IF 5<2+9 PRINT ",$22,"5<2+9 ok",$22,$00                  ; relop RHS is a whole + - expr
        DB 0,132,"IF 1+2*3>4*5-14 PRINT ",$22,"7>6 ok",$22,$00             ; * / then + - then relop
        DB 0,135,"IF 6!>9 PRINT ",$22,"6<=9 ok",$22,$00                    ; '!' inverts '>'
        DB 0,136,"IF 9>4 THEN PRINT ",$22,"THEN ok",$22,$00                ; optional THEN
        DB 0,137,"LET K=5",$00                                             ; optional LET
        DB 0,138,"IF K>3 THEN LET K=K+1",$00                               ; THEN + LET + '>'
        DB 0,139,"PRINT ",$22,"LET/THEN: K=",$22,$3B,"K",$00               ; expect K=6
        DB 0,140,"PRINT ",$22,"--- LOOP via GOTO ---",$22,$00
        DB 0,150,"I=1",$00
        DB 0,160,"IF 5<I GOTO 190",$00
        DB 0,170,"PRINT I",$3B,$00
        DB 0,180,"I=I+1",$00
        DB 0,185,"GOTO 160",$00
        DB 0,190,"PRINT ",$22,"",$22,$00
        DB 0,195,"PRINT ",$22,"--- GOSUB/RETURN ---",$22,$00
        DB 0,196,"GOSUB 210",$00
        DB 0,197,"PRINT",$00                                                ; newline after the nested-call line
        DB 0,198,"GOTO 250",$00                                            ; over the subs, into FOR/NEXT
        DB 0,210,"PRINT ",$22,"L1 ",$22,$3B,$00
        DB 0,211,"GOSUB 220",$00
        DB 0,212,"PRINT ",$22,"L1-done ",$22,$3B,$00
        DB 0,213,"RETURN",$00
        DB 0,220,"PRINT ",$22,"L2 ",$22,$3B,$00
        DB 0,221,"GOSUB 230",$00
        DB 0,222,"PRINT ",$22,"L2-done ",$22,$3B,$00
        DB 0,223,"RETURN",$00
        DB 0,230,"PRINT ",$22,"L3 ",$22,$3B,$00
        DB 0,231,"GOSUB 240",$00
        DB 0,232,"PRINT ",$22,"L3-done ",$22,$3B,$00
        DB 0,233,"RETURN",$00
        DB 0,240,"PRINT ",$22,"L4-deepest ",$22,$3B,$00                     ; 4 levels deep - the documented max
        DB 0,241,"RETURN",$00
        DB 0,250,"PRINT ",$22,"--- FOR / NEXT ---",$22,$00
        DB 0,255,"FOR I=1 TO 5",$00
        DB 1,4,"PRINT I*I",$3B,$22," ",$22,$3B,$00                              ; 260  expect 1 4 9 16 25
        DB 1,9,"NEXT I",$00
        DB 1,14,"PRINT",$00
        DB 1,19,"FOR A=1 TO 3",$00                                            ; nested: 3x3 table
        DB 1,24,"FOR B=1 TO 3",$00
        DB 1,29,"PRINT A*B",$3B,$22," ",$22,$3B,$00
        DB 1,34,"NEXT B",$00
        DB 1,35,"PRINT",$00
        DB 1,36,"NEXT A",$00
        DB 1,37,"FOR C=10 TO 2 STEP -4",$00                                     ; 293  STEP demo: expect 10 6 2
        DB 1,38,"PRINT C",$3B,$22," ",$22,$3B,$00                              ; 294
        DB 1,39,"NEXT C",$00                                                   ; 295
        DB 1,40,"PRINT",$00                                                    ; 296
        DB 1,41,"GOTO 300",$00                                                 ; 297
        DB 1,44,"PRINT ",$22,"--- MANDELBROT ---",$22,$00                  ; 300
        DB 1,49,"M=16",$00                                                 ; 305 iteration limit (a variable FOR limit)
        DB 1,54,"FOR R=0 TO 20",$00                                        ; 310 21 rows       (FOR level 1)
        DB 1,64,"LET D=R*6-64",$00                                         ; 320 imaginary part (was I=-64 step 6)
        DB 1,84,"FOR Q=0 TO 43",$00                                        ; 340 44 columns    (FOR level 2)
        DB 1,94,"LET C=Q*4-144",$00                                        ; 350 real part      (was C=-144 step 4)
        DB 1,104,"A=C",$00                                                 ; 360
        DB 1,105,"B=D",$00                                                 ; 361
        DB 1,106,"E=0",$00                                                 ; 362
        DB 1,108,"GOSUB 600",$00                                           ; 364 escape-count subroutine (below)
        DB 1,174,"IF E>0 THEN PRINT CHR$(E+32);",$00                       ; 430 char for iteration depth
        DB 1,184,"IF E=0 THEN PRINT CHR$(32);",$00                         ; 440 space for unescaped
        DB 1,194,"NEXT Q",$00                                              ; 450
        DB 1,224,"PRINT",$00                                               ; 480 end of row newline
        DB 1,234,"NEXT R",$00                                              ; 490
        DB 1,254,"END",$00                                                 ; 510
        DB 2,88,"FOR N=1 TO M",$00                                         ; 600 escape-count subroutine, a FOR loop
        DB 2,98,"IF E>0 GOTO 640",$00                                      ; 610   (FOR level 3, inside a GOSUB, inside
        DB 2,108,"T=A*A/64-B*B/64+C",$00                                   ; 620    levels 1-2). Escaped points skip the
        DB 2,113,"B=2*A*B/64+D",$00                                        ; 625    maths by GOTOing the NEXT: leaving
        DB 2,114,"A=T",$00                                                 ; 626    via NEXT (not out of the loop) leaves
        DB 2,118,"IF 256<A*A/64+B*B/64 THEN IF E=0 THEN E=N",$00           ; 630    no frame behind. No parentheses:
        DB 2,128,"NEXT N",$00                                              ; 640    the relop takes a whole + - * / RHS
        DB 2,138,"RETURN",$00                                              ; 650
SHOWCASE_END:

        END
