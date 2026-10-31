; 4kBASIC2650.asm       Fast 4k Tiny BASIC interpreter for Signetics 2650
; Version: V5.11
; By Vincent Crabtree, 2026.  MIT License
; Date:    Nov 2026
;
; Target:  Standalone (no PIPBUG ROM). Code ORG 0. I/O routines embedded.
;          Single 8192-byte address space (2650 bits 15:13 always 0).
;
; Assembler: asm2650.c v1.13+  Simulator: pipbug_wrap.c v2.1
; Build:
;   gcc -Wall -O2 -o asm2650 asm2650.c
;   gcc -Wall -O2 -DGAMER -o pipbug_wrap pipbug_wrap.c
;
;   ./asm2650 --warn=all 4kBASIC2650.asm 4kBASIC2650.hex
;       note: `all` is lower case
;
;   grep -n "^CHIN \|^COUT \|^ROMEND " .\4kBASIC2650.LST
;   ./pipbug_wrap --entry 0 --chin 0x<addr> --cout 0x<addr> 4kBASIC2650.hex
;
; Regression: tools/check_all.sh.  Nesting limits: tools/nestprobe.py.
; RAS depth:  tools/rasdepth.sh.
;
; =============================================================================
; LANGUAGE REFERENCE (for users)
; =============================================================================
;
; Statements:
;   END  FOR <var>=<expr> TO <expr> [STEP <expr>]  FREE  GOSUB <expr>
;   GOTO <expr>  IF <expr> <relop> <expr> THEN <stmt>  INPUT <var>
;   [LET] <var>=<expr>  LIST [<start>,<end>]  NEW  NEXT [<var>]
;   POKE <addr>,<val>  PRINT  REM  RETURN  RUN
;
;   LET is optional, THEN is required.
;
; PRINT items: "literal", expr, TAB(n), CHR$(n), or HEX$(n); separate with
;   ';'. TAB(n)/CHR$(n)/HEX$(n) are PRINT-only, not general functions.
;   HEX$(n): '$' and 4 hex digits, unsigned.
;
; Functions (all take parenthesized args, e.g. ABS(n)):
;   ABS(n)  NEG(n)  NOT(n)  bitwise AND(a,b)/OR(a,b)/XOR(a,b)
;   PEEK(addr): byte at addr, 0-255.  POKE is a statement, see above.
;   RND(n): pseudorandom integer, 0 to n-1.
;   USR(addr): call machine code at addr; its R0 is the result (see MEMORY ACCESS).
;   Calls nest inside expressions and each other, e.g. ABS(NEG(ABS(-4))), AND(7,OR(8,1)).
;
; Arithmetic: + - * / % (mod), ^ (power), unary -.
; Precedence, tightest first: ^  then * / %  then + -  then relops.
;   * / % and + - are left-to-right among themselves; ^ binds to each atom
;   before * / are combined. Parentheses override.
; Relops: =  <  >  <=  <>  >=  all native (no inversion prefix needed).
;   Lowest precedence; a relop's operands are whole + - * / ^ expressions.
;
; Numbers  : signed 16-bit (-32768 .. 32767), unsigned hex $BEEF (0x0000-0xffff)
; Variables: A-Z (26), 16-bit signed; no arrays or string variables
;
; KNOWN LIMITATIONS
;
; SYNTAX
;   Keywords and variable letters must be UPPERCASE (PRINT "string literals" are verbatim).
;   THEN is a reserved word (token): the letters T,H,E,N in a row outside strings/REM are THEN.
;   Keywords must be spelled in full; letters trailing a keyword are swallowed
;   (PRINTX = PRINT, ABSX(5) = ABS(5)).  A function word at statement level is an error.
;   No ':' multi-statement lines (not planned: RAS depth).  IF takes one statement after THEN.
;   Validation is minimal: malformed input gives a generic error (usually ?0).
;   Input lines are limited to 63 characters; the rest is silently dropped by RDLINE.
;
; PROGRAM STORE
;   Full random line editing: a numbered line is inserted in order, replaces a line of
;   the same number, or (empty body) deletes it.  Line numbers 1-32767.  A line that does
;   not fit gives ?3 and leaves the store unchanged.
;   GOTO/GOSUB to a line that does not exist continues at the next higher line, or ends
;   the program past the last line.  There is no undefined-line error (?1 is unused).
;
; ARITHMETIC
;   + - * and ^ wrap silently mod 65536.  ^ gives ?9 only for a negative exponent; it is
;   repeated multiplication, so its time grows with the exponent.
;   % returns |a| mod |b| (never negative); only the quotient takes the sign.
;
; HEX LITERALS ($HHHH)
;   Accepted in stored lines and in direct entry.  Digits 0-9 and UPPERCASE A-F only;
;   '$' not followed by a hex digit is a syntax error (in a stored line it is kept as a
;   plain '$').  Any number of digits is accepted and wraps mod 65536 ($10000 = 0,
;   $FFFF = -1).  LIST always prints 4 digits.
;
; GOSUB / RETURN
;   Up to 8 levels deep (GSSTKLIM=16 bytes, 2 per level); the 9th gives ?3.
;   Leaving a subroutine other than by RETURN leaves its frame on the stack.
;
; FOR / NEXT
;   FOR <var>=<start> TO <limit> [STEP <step>], then NEXT [<var>].  Default step is 1;
;   negative steps work.  A var+step that overflows 16 bits ends the loop, so any signed
;   range terminates.  Up to 4 levels deep (FORSTKLIM=28 bytes, 7 per level); the 5th
;   gives ?6.
;   - The body always runs at least once (the test is at NEXT).
;   - The variable after NEXT is consumed but not checked against the frame.
;   - Leaving a loop early (GOTO/GOSUB out of it) is not supported: its frame stays on
;     the FOR stack.
;
; EXPRESSION NESTING
;   Parentheses, signs, operators and function calls (with their arguments) nest on the
;   SW stack (SWBASE, SW_SIZE=64 bytes), never on the hardware return stack.  PUSH_RET
;   raises ?8 (ERR_NEST) at SW_LIMIT (59 bytes in use), so over-deep input fails cleanly.
;   Cost per open level:   '('   4 bytes        unary sign  2        a+(...)  6
;                          1-arg function  ABS( NEG( NOT( PEEK( RND( USR(     5
;                          2-arg AND/OR/XOR/POKE/LIST: nested in the FIRST argument 5,
;                          nested in the SECOND argument 7.
;   Measured (direct mode): 14 parentheses (the 15th gives ?8); 28 unary signs (the 29th
;     gives ?8); 1+(1+(1+( ... 9 deep.  Function nesting hits the 63-character line first:
;     about 11 deep for 1-arg and 8 deep for 2-arg functions, 10 and 7 in a stored line
;     (line number, space and PRINT take 9 characters).  Functions inside parentheses
;     share the SW budget: 4 parentheses leave 8 function levels, 8 leave 5, 10 leave 3,
;     12 leave 1.
;   The ')' closing AND/OR/XOR, POKE and LIST arguments is optional (so one lost to the
;   63-character limit is harmless there); every other bracket is required.
;
; MEMORY ACCESS (PEEK / POKE / USR)
;   Unchecked.  RAM is $1000-$1FFF: interpreter variables first, program store above, so
;   POKE or a USR routine can corrupt either; PEEK(4096..) reads interpreter RAM.
;   A USR routine must return with RETC,UN and leave PSL COM=1, RS=0 and WC=0 (the
;   interpreter relies on all three), and keep its own call depth small (8-level HW stack).
;
; =============================================================================
; IMPLEMENTATION NOTES (for maintainers)
; =============================================================================
;
;        CC SEMANTICS (2650 ALU)
;   ADD/SUB: result<0 -> LT   result>0 -> GT   result=0 -> EQ
;   Carry bit (PSL bit 0): C=1 = carry / no-borrow (independent of CC).
;   Carry test: TPSL $01  ->  CC=EQ if C=1 (carry), CC=LT if C=0 (no carry).
;   Carry skip (no carry): BCTR,LT. Carry taken (carry): fall through or BCTR,EQ.
;   PREFERRED: Use WC idiom (CPSL $08 / lo-add / PPSL $08 / hi-add / CPSL $08) for 16-bit adds.
;   Unsigned compare: COM=1 is the global default (set once in MAIN); COMA/COMI then
;   compare unsigned. SUBA is NOT a compare: its CC is the sign of the 8-bit RESULT.
;   Signed compare: CPSL $02 / COMA / PPSL $02 (see CMP_LNUM_EXP).
;   Binary flag (0 or 1): after LODA CC=EQ(0) or GT(1); use BCTR,GT not COMI $01.
;   Single page: all addresses $0000-$1FFF; hi-byte carry impossible.
;
;        HI/LO OPERATOR CONVENTION
;   <ADDR = HIGH byte (bits 15:8)   e.g. <$1634 = $16
;   >ADDR = LOW  byte (bits  7:0)   e.g. >$1634 = $34
;
;        RAS DEPTH BUDGET (8-level hardware stack)
;   Every followed BSxx/ZBSR consumes one slot regardless of condition.
;   BCxx/ZBRR are plain branches -- no RAS cost.
;   Expression nesting (parens, signs, operators, function calls and their arguments)
;   lives on the SW stack, so the HW RAS depth is fixed per statement: PARSE_EXPR +1,
;   a function handler +1 (FB_CALL).  Measured worst case is 7 of 8 levels
;   (RND -> DIV16 -> SETUP_MULDIV -> ABS_X), the same for PRINT/LET/IF/FOR/GOTO/POKE,
;   nested IFs and LIST.
;
;        SCRATCH REGISTER CONVENTIONS
;   R0  working register, arithmetic, I/O.
;   R1  index register (LODA/STRA BASE,R1); also PRINT_S16 digit count.
;   R2  variable offset of an assignment/INPUT/FOR (SE_NOTKW/DO_INPUT/DO_FOR), live across
;       PARSE_EXPR, function handlers included (FB_CALL parks it in R2SAVE).
;       Written by subroutines only in SE_NOTKW (LET/bare assign) and DO_FOR.
;   R3  loop counter (BDRR/BIRR); gap/copy size in TRY_STORE_LINE/OPEN_GAP.  In
;       expression parsing: SW stack pointer ($FF = empty; index of the top byte; pushes
;       pre-increment).  RDLINE: IBUF offset, $FF = empty (",Rx+" pre-increments before
;       access, so empty must be one below the first valid index, not 0).
;   INC_IP, INC_EXP, INC_TMP, WORD_START and REG16_TO_REG16 leave R1-R3 intact (they use
;   R0 only or the alternate register bank).
;
;        RAM OVERLAYS (these lifetimes must stay disjoint)
;   LBUF (136 bytes) = [SWBASE 64][GCACHE 8][IBUF 64], one buffer for the typed line, the
;                    SW stack, the GOTO cache and the tokenizer's output (TOKBUF = LBUF).
;                    SWBASE/IBUF are live together (a direct-mode line is parsed while its
;                    expression nests on the SW stack); GCACHE/IBUF are live together (INPUT
;                    inside a RUN rewrites IBUF; the cache lives for the whole RUN); SWBASE/
;                    GCACHE are live together (RUN).  So those three are disjoint, and LBUF
;                    is the sum of their sizes.
;   TOKBUF = LBUF    TOKENIZE_LINE works IN PLACE: its output (TOKBUF, from byte 0) trails its
;                    read pointer through IBUF (the top 64 bytes).  Single-digit constants
;                    expand 1 -> 3 bytes, so output can gain on input by up to 62 bytes over
;                    a 62-character body ("(1+1+...+1"); IBUF starts 72 bytes up, leaving 11
;                    spare.  Build check: IBUF-LBUF >= 61.  TOKBUF is live only inside
;                    TRY_STORE_LINE at the prompt, where the line is dead once tokenized and
;                    neither the SW stack nor the cache is in use or needed.  Stale cache
;                    entries at the prompt are harmless (RUN invalidates them).
;   POWCNT = TK      CHECK_POW's loop calls only MUL16 and REG16_TO_REG16.
;   SAVE = SC0:SC1   likewise.
;   R2SAVE = TKH     live only across a function handler, which never reaches MATCH_ROW
;                    or TL_MULADD.
;   TEMPRET = GOTO   jump trampolines, each live for 2-3 instructions; EAM_ATOM_C/EAM_HI_C also
;                    park a continuation in it across one WSKIP/INC_IP (no dispatch inside).
;   TSL_BLEN = RELOP TRY_STORE_LINE (prompt) vs DO_IF (statement).
;   GC_NEXT is NOT overlaid: it indexes GCACHE and must always be 0 or 4.
;
; =============================================================================
; CHANGE HISTORY
;
; V5.11 (2026-11) - ~25% speed increase - ROMEND $0E5D  (3677 of 4096)   
;   - RAM -56 bytes (more program space): IBUF and TOKBUF merged into one 136-byte LBUF and
;     TOKENIZE_LINE now tokenizes in place (read pointer IP in IBUF, write index R3 into
;     TOKBUF = LBUF, trailing the reader).  See RAM OVERLAYS for details.
;   - EAM_ATOM_C: a variable/constant atom in a RUNning program is read inline and control jumps
;     straight to the continuation (parked in TEMPRET), so no push/pop; other atoms and typed text
;     push and take EAM_ATOM as before.  EAM_HI_C: skip the * / % frame when none follows.
;   - Inline blank test (LODA/COMI/BCFR) before WSKIP at the 4 hot sites; variables advance IP
;     inline; ADD16_SAVE_EXP adds straight from the SW stack (no POP_SAVE_TO_TMP/ADD_CORE).
;   - DR_EXEC reads line#/len with *TMPH,R1+ and forms IP and SWSTK directly (-23 instr/stmt).
;     TMP stays at the current record during a statement; no handler reads it on entry.
;   - THEN is statement token $90 (KW_TAB row, VS_THEN -> JSYNERR): 3 bytes/IF smaller, DO_IF
;     takes it with one INC_IP; typed text keeps the raw T/H/EATWORD path.  Behaviour: THEN is
;     reserved; letters right after it are swallowed (IF A THENPRINT worked by accident, now ?4).
;   - +2 vectors (VEAM_ATOM_C, VEAM_HI_C); showcase re-encoded (14 records).  Max HW RAS still 7.
;   - NOT,XOR,AND loop golf as infrequntly called - inline if doing Bitcoin in BASIC...
;
; V5.10 (2026-10) - ROMEND $0EEF -> $0DA6 (-329).  Smaller; speed cost ~+24 instructions
;   per ABS-type call, ~+37 per 2-arg call; -14 per plain statement.
;   - Token class in the top bits (statement / niladic / 1-arg / 2-arg function): O(1)
;     dispatch, one VEC table, one [text][NUL][token] row format (KW_TAB..FUNC_TAB).
;     Token values are assembler-derived from VEC order; function order is now grouped
;     by argument count.
;   - Function calls moved off the HW RAS onto the SW stack (FUNC_BODY, FB_A, FB_B,
;     FB_CALL); POKE and LIST use the same chain.  PUSH_RET guards the SW stack (ERR_NEST).
;     SW_SIZE 32 -> 64.  The PARSE_EXPR entry guard and the FT_* save area are gone.
;   - FIXED: functions nested in functions, or inside RUN, gave ?8.
;   - FIXED: RND/USR clobbered R2 (B=RND(5) assigned A).
;   - FIXED: a function token in statement position hung; it is now an ordinary error.
;   - RAM -80 bytes (more program space): the SW stack and the GOTO cache overlay TOKBUF;
;     SAVE, POWCNT, TEMPRET, R2SAVE and TSL_BLEN share bytes with idle scratch.
;   - Banner shows V5.10.
;
; V5.9 (2026-10)
;   - DO_IF: PARSE_RELOP removed, relop parse and signed compare inlined (RAS -1).
;
; V5.8 (2026-10) - ROMEND $0EEF -> $0EF1 (+2)
;   - FIXED LIST-01: "LIST start,end" ignored the range and listed the whole program.
;
; V5.7 (2026-10) - ROMEND $0E39 -> $0EBE (+133).  SPEED.
;   - Length-prefixed line records [lineH][lineL][len][body...][NUL]: ADV_TMP_PAST_REC
;     is O(1).
;   - DR_EXEC register-pair copies inlined.
;   - 2-slot backward GOTO/GOSUB line cache (GCACHE), reset by RUN.
;   - Cache-miss slot update is a 0<->4 toggle (-9 bytes).
;
; V5.6 (2026-10) - ROMEND $0D94 -> $0E39 (+165).  SPEED.
;   - Expression start skips the keyword table unless a function word is possible
;     (+25 bytes, -4.8% time).
;   - Forward GOTO/GOSUB searches from the next record, not from PROG (+36, -22.4%).
;   - DIV16 power-of-two divisor shift path (+95, -6.5%).
;   - MUL16 multiplier kept in R0 a byte at a time; operands swapped so the multiplier
;     is one byte where possible (+9, -1.7%).
;
; V5.5 (2026-10)
;   - FIXED FUNC-01: single-argument functions now require '(' and ')' (ABS(-5)+10
;     printed 5).  "ABS -5" is a syntax error.
;
; V5.4 (2026-10) - ROMEND $0ECE -> $0D73 (-347).  SIZE.
;   - PRINT_S16 per-column rewrite; ADD_CORE, DIGIT_CHECK, SET_TMP, CMP_LNUM_EXP shared.
;   - SWSTK is the program counter (GOTO/GOSUB/RETURN/NEXT write it); GOTOFLG, DR_GOTO
;     and DO_GOSUB's duplicate tail gone.
;   - LET merged into SE_NOTKW; DO_NEW via TMP/REG16; blank line / empty THEN body no
;     longer print ?4.
;   - FIXED: IF and NEXT compared wrongly when operands differed by >= 128 (IF 200>10
;     was false).
;   - FIXED: the 4th FOR frame overran FORBASE into IBUF.
;   - FIXED: RUNFLG stayed set after a program ran off its end (stale "@line").
;   - FIXED: MAIN's variable clear skipped VARS+0.
;   - FIXED: a FOR loop whose var+step overflows 16 bits now ends.
;
; V5.3 (2026-09) - ROMEND $0E61 -> $0EDC (3804 bytes)
;   - MUL16: MSB-first shift-add (Horner), 16 passes or 8 if multiplier<256.
;   - DIV16: restoring shift-subtract, 16 passes or 8 if dividend<256.
;   - MATCH_KW early-out: anything but two letters in a row no longer scans table.
;   - INC_IP: dedicated ~4-instruction path vs ~10 via shared INC_ET.
;   - ADV_TMP_PAST_REC: indexed scan, TMP advanced once
;
; V5.2 (2026-09) - ROMEND $0E78 -> $0E61 (3681 bytes)
;   - Variable plumbing clean up: PARSE_VAR_SAVE now leaves R2 = VARS byte offset
;     (2*(letter-'A')) instead of ASCII letter, and drops dead SC0 copy.
;     DL_STORE uses R2 directly as index register: no letter->offset conversion.
;   - PARSE_VAR_SAVE range test is one SUBI/COMI (relies on global COM=1).
;   - DO_FOR now takes its variable via PARSE_VAR_SAVE (was: unvalidated letter,
;     dead STRA FORVAR, own INC_IP). 
;   - BUG FIXED: FOR with a non-A-Z variable (e.g. FOR [=1 TO 2) computed a VARS
;     offset outside VARS; it now raises the same variable error as LET/INPUT.
;   - PF_LOADVAR: index kept in R1 across INC_IP (bank-1, R1 survives).
;
; V5.1 (2026-09) - ROMEND $0E6A -> $0E78 (3704 bytes, +14)
;   - Direct-mode hex constants ($BEEF): PF_NUM hands '$' to the new PARSE_HEX.
;     HEX_PEEK/PARSE_HEX factored out of TL_MAYBEHEX so the tokenizer and the
;     expression parser share one loop.
;   - PARSE_HEX is tail-jumped from PF_NUM (not called) to keep worst-case RAS depth at 7.
;
; V5.0 (2026-09) - ROMEND $0E6A, 3690 bytes; PROG 1543 -> 1403 bytes
;   - Minor DO_PRINT code refactor.
;   - Added keyword token EQUs (TOK_END..TOK_RUN, TOK_ABS..TOK_XOR).
;   - SHOWCASE now stored pre-tokenized (73 records)
;
; V4.28 (2026-09) - ROMEND $0ED5 -> $0E70 (3696 bytes)
;   - Deleted GETCI and vector - WSKIP leaves char in R0, uses INC_IP instead. 
;     Cleaned up multiple unnecessary LODA,R0 *IPH after WSKIP calls.
;   - Abused EXPR for single arg function, deleted CHECK_LPARENS/Check_RPARENS 
;     and vectors. Inlined left Paren check at PARSE_2ARGS. 
;   - Zero-page vector sweep, added common REG16_TO_REG16 vector and shims.
;
; V4.27 (2026-09) - ROMEND $0EEF -> $0ED6 (3798 bytes)
;   - DO_FOR STEP test now peeks (WSKIP) instead of GETCI/DEC_IP on default-step.
;   - Deleted DEC_IP, DEC_ET body and VDEC_IP zero-page vector as no callers.
;   - BUG-FOREQ-01: space before '=' (FOR I = 1 TO 3, FOR I =1 TO 3) gave ?0.
;     DO_FOR used GETCI which does not skip whitespace, replaced with WSKIP.
;
; V4.26 (2026-09) - ROMEND $0F1F -> $0EEF 
;   - FETCH16_IP - Replaces four inline LODA *IPH/STRA/INC_IP x2
;     sequences: DLS_BODY, PF_CONST (tail-called), DLS_CONST, and DR_EXEC.
;   - DR_EXEC: the 2-byte line-number header is now IP=TMP, FETCH16_IP,
;     EXP->CUR instead of the *TMPH,R1- loop plus two INC_IPs 
;   - ADD16_SAVE_EXP and CP_EXP_RET: the inline SWBASE,R3- pop into SAVE is now
;     POP_SAVE_TO_TMP + REG16_TO_REG16 (TMP->SAVE).
;
; V4.25 (2026-09) - ROMEND $0F8E -> $0F1F 
;   - Added REG16_TO_REG16, zero-page vector VREG16_TO_REG16 and IDX_* EQUs.
;   - Deleted superseded helpers: EXP16_TO_TMP, EXP16_TO_GOTO,
;     EXP16_TO_LNUM, EXP16_TO_ET, CUR_TO_EXP16, TMP_TO_EXP16, ET_TO_EXP16,
;     TMP_TO_IP, TMP_TO_SWSTK, TMP_TO_ET, IP_TO_TMP; and appropriat vectors.
;   - Converted additional 12-byte inline LODA/STRA copies:
;     SWSTK->TMP/GOTO->TMP (DR_LP), RNDSEED->TMP (RND), EXP->ARGA (PARSE_2ARGS),
;     ARGA->LNUM (LIST), EXP->POW/SAVE->TMP (CHECK_POW), PE->EXP (OPEN_GAP).
;   - DR_GOTO now copies GOTO->LNUM directly (was GOTO->EXP->LNUM; EXP is not
;     read again before the next statement re-parses).
;   - RAM: reordered so every copied pair is hi/lo even offset from IPH. 
;
; V4.24 (2026-09) - ROMEND $1019 -> $0FEE
;   Golf pass on the V4.23 tokenizer fixes -- no behaviour change
;   - TOKENIZE_LINE / TOK_MATCH_ROW / TL_MULADD added dedicated TKH:TKL
;     pair instead of TMPH:TMPL, TSL_WRITE no longer saves/restores TMP.
;   - TL_STR_LP re-reads the just-stored char with LODA,R0 TOKBUF,R3 instead
;     of caching it in SC0 across VINC_IP. 
;   - TL_DEC_DONE and TL_HEX_DONE shared one constant-emit tail
;     (TL_EMIT_CONST); they differed only in the token byte.
;   - ORG 4096 re-enabled; memory map is back to the real split (IPH=$1000).
;
; V4.23 (2026-09) - ROMEND $0FFC -> $1019 overflow
;   Tokenizer step 3 (encode side: TOKENIZE_LINE + TRY_STORE_LINE)
;   - TSL_WRITE: TOKENIZE_LINE clobbers TMPH:TMPL. Fix: TMPSAVEH/TMPSAVEL, 
;     superseded in V4.24 by TKH:TKL pair.
;   - TL_LP letter typo BCFR,LT instead of BCTR,LT
;   - TL_STR_LP tested R0 for the closing quote after ZBSR *VINC_IP clobber.
;     Fix: cache in SC0 across the call. 
;   - TSL_CPY copied TOKBUF up to the first $00 byte, truncating records at
;     a TOK_DEC/TOK_HEX payload zero byte. Fix: copy by count (R3-2=body+NUL)
;   - TL_MULADD: STRZ,R3 (multiplier) ran after EXP16_TO_TMP/CLR_EXP had
;     clobbered R0, so R3=0 and BDRR looped 256x: Fix: STRZ,R3 first.
;   - TL_HEX_LP: STRA,R0 SC0 ran after ZBSR *VINC_IP (R0 clobbered), so hex
;     digits accumulated a pointer byte. Fix: swap the two.
;   - TOKBUF (80) overflowed into VARS on dense-constant lines, expanded 
;     so TOKBUF now RES 128. 
;
; V4.22 (2026-09) - ROMEND $0DA3 -> $0E8D (+234 bytes)
;   Tokenizer project, step 2: dispatch + detokenization infrastructure.
;   - Token space: TOK_STMT_BASE=$80 (16 rows, $80-$8F), gap to
;     TOK_FUNC_BASE=$A0 (9 rows, $A0-$A8) reserved for future statements,
;     TOK_HEX=$FD/TOK_DEC=$FE (each followed by 2 payload bytes, hi/lo).
;   - KWH_xxx/FNH_xxx labels added to every KW_TAB/FUNC_TAB row (hi byte
;     position); TOK_STMT_OFS/TOK_FUNC_OFS are assembler-computed
;     row-index -> byte-offset tables built from those labels, so they
;     can't drift out of sync with the keyword text.
;   - MATCH_KW: O(1) fast path added ahead of the V4.21 text scan. A token
;     byte's own value range picks the offset table and row index directly
;     (no table-address comparison needed, so it's correct regardless of
;     where KW_TAB/FUNC_TAB sit in ROM); the offset then reuses a tail
;     almost identical to the old SE_HI_LO to load the handler and jump.
;   - PARSE_FACTOR: TOK_DEC/TOK_HEX recognized up front and loaded directly
;     from their 2-byte payload (O(1)) instead of PARSE_S16's per-digit
;     accumulation, which still runs unchanged for typed/direct-entry text.
;   - ADV_TMP_PAST_REC - NULL end of line skipped when part of token.
;   - DLS_BLPX (LIST) is now the detokenizer: statement/function tokens
;     print their full keyword text 
;
; V4.21 (2026-09) - ROMEND $0D99 -> $0DA3 
;  Tokenizer project step 1 - Table expansion.
;   - KW_TAB/FUNC_TAB expanded from a fixed 5-byte-stride [c1][c2][c3][hi]
;     [lo] row (matched on the first 2-3 chars only) to variable-length
;     [full keyword text][NUL][hi][lo] rows, matched on the COMPLETE
;     spelling. This table doubles as the LIST detokenizer's text
;     source once tokens land (Step 2) -- avoids 2nd duplicate table.
;   - MATCH_KW/SE_SCAN rewritten: Each row now compared char-by-char directly
;     against IP (peeked via *IPH, not consumed) using R0/R1 only. 
;   - Dispatch  requires FULL keyword spelling; trailing garbage after
;     a complete keyword is still swallowed by EATWORD like before.
;
; V4.20 (2026-09) - ROMEND $0DDE -> $0D99 
;   - Line storage/deletion (TRY_STORE_LINE/STORE_LINE/DELETE_LINE/MEMCPY/DEC_PE)
;     merged into TRY_STORE_LINE (find-and-delete loop shared by pure delete and
;     overwrite, then a conditional insert) + DEL_REC + OPEN_GAP (builds and
;     range-checks the new PE before touching anything, then shifts backward).
;     MEMCPY deleted.
;
; V4.19 (2026-09) - ROMEND $0E09 -> $0DDE 
;   - SWSP/FORSP (GOSUB and FOR stack pointers): plain 0-based byte count (0=empty),
;     replacing the $FF/$F9 sentinel scheme.  Removes DGS_FIRST/DGS_NEXT, DRT_SUB,
;     DN_POP_EMPTY and DO_FOR's COM-mode toggle.
;   - FIXED BUG-GSSTKLIM-01: GSSTKLIM=$0F let a 9th nested GOSUB push write 2 bytes past GS.
;   - DO_FOR now caches the loop variable's precomputed VARS offset in
;     FORVAR (from DL_STORE's R1, free) instead of the raw A-Z letter;
;     DO_NEXT loads directly vs recomputing (var-'A')*2 every call.
;
; V4.18 (2026-09) - ROMEND $0E82 -> $0E09 
;   - PRINT_S16: SW-recursive shift/divide-by-10 engine replaced by iterative repeated
;     subtraction of powers of 10 (P10_HI/P10_LO tables).  No longer clobbers TMP, SC1
;     or NEGFLG (clobbers R2 instead).
;
; V4.17 (2026-09) - ROMEND $0EB9 -> $0E85 
;   - MATCH_KW: TMP stays fixed at the table base (KW_TAB/FUNC_TAB); R1 walks
;     it via indirect-indexed addressing (*TMPH,R1) instead of advancing TMP
;     itself through ZBSR calls. -7 bytes.
;   - ADV_TMP_PAST_REC: now skips the 2-byte header itself;
;     DELETE_LINE/FIND_INS no longer pre-skip it.
;   - Shared helpers: Added TMP_TO_IP / TMP_TO_SWSTK (mirrors EXP16_TO_ET,
;     reuses its ET_STORE tail). DR_EXEC's header-save now an indexed
;     2-iteration loop (TMP untouched, enables the ADV_TMP_PAST_REC change
;     above); DLS_BODY also uses TMP_TO_IP. -15 bytes combined.
;   - Removed all case-folding (UPCASE deleted; call sites in GETCI,
;     EATWORD, PARSE_VAR_SAVE, PARSE_FACTOR, MATCH_KW stripped): keywords
;     and variable letters now require UPPERCASE, matching the spec.
;
; V4.16 (2026-09) - ROMEND $0EBA 
;   - FL_CHKLO / FI_LH: Direct-indexed peek (*TMPH,R1) replaces INC16_TMP_TO_EXP.
;   - Stored record format: Changed line terminator CR ($0D) -> NUL ($00).
;   - Shared helpers: Added ADV_TMP_PAST_REC to scan past NUL-terminated records.
;   - DR_EXEC: Direct-in-place execution from store (TMP+2), bypassing IBUF copy loop.
;   - FIXED BUG-PU16-01: Corrected PARSE_U16 pre-loop digit check bound (':' -> '9').
;   - Streamlined DO_NEW, SHOWCASE_END to PE setup deleted, now Assembly time.
;
; V4.15 (2026-09) - ROMEND $0F26 
;   - Shared helpers: Added CMP_TMP_PE for 16-bit TMP vs PE comparison.
;   - DELETE_LINE: uses the destination pointer as the new PE.
;   - PARSE_U16 / TRY_STORE_LINE: Collapsed digit checks to single unsigned sub/cmp.
;   - STORE_LINE: OPEN_GAP technique; indexed dst off walking PE (*PEH,R3).
;
; V4.14 (2026-09) - ROMEND $0FB7 
;   - Unsigned comparison: Enabled global COM=1 mode.
;
; V4.13 (2026-09) - ROMEND $0FB5 
;   - FIXED BUG-MUL-01: MU_LP decrement now checks carry flag (TPSL $01) instead
;     of raw CC, fixing 16-bit multiply for operands >= 256.
;
; V4.12 (2026-09) - ROMEND $0FB3 
;   - Shared helpers: Added CARRY_INTO_EXPH to consolidate carry propagation
;     in MUL16 and PARSE_U16.
;
; V4.11 (2026-09) - ROMEND $0FC8 
;   - CHIN: Removed hard ORG $286 pin to let subroutine space float naturally.
;
; V4.10 (2026-09) - Renamed 4kBASIC - ROMEND $0FC8
;   - ZPVEC-01: Vectored PUSH_EXP and PRINT_S16 via zero-page jump table (ZBSR).
;
; V4.9 (2026-08) - ROMEND $0FDB 
;   - FIXED BUG-DL2-01: DL2_LP copy loop now tests carry flag (TPSL $01) instead
;     of signed CC, fixing premature loop exit on page-boundary wraps during deletes.
;
; V4.8 (2026-08) - ROMEND $0FD9 
;   - PRINT HEX$(n): Added 4-digit unsigned 16-bit hex formatting in ROM slack.
;
; V4.7 (2026-08) - ROMEND $0F98
;   - Shared helpers: Added PUSH_RET to handle 16-bit SWBASE push.
;   - Operators: Added ^ (power) operator with error handling.
;   - FIXED BUG-MINLP-01: Fixed hang printing -32768 (invalid indexed LODR).
;
; V4.6 (2026-07) - 3879 bytes (ROMEND $0F27)
;   - FIXED: FUNCATOM-01 - functions now work as non-leading atoms, e.g.
;     "PRINT 10+ABS(A)" (previously only "PRINT ABS(A)+10" worked).
;   - Added FT_SP/FT_STK/FT_SAVE_SP/FT_SAVE/FT_N/FT_R2SAVE (72 RAM bytes)
;     and FUNC_EPILOG; PE_SAFE/EAM_ATOM/PE_NOFUNC/DO_END updated. 
;
; V4.5 (2026-06) - 3705 bytes
;   - FIXED: Function parser tracking for trailing operators (e.g., ABS(-5)+10).
;   - Relocated PEEK/USR/EXPH functions to optimize space post-COUT.
;
; V4.4 (2026-06) - 3636 bytes
;   - FIXED: RND 16-bit seed rotation bug by correctly setting PSL WC bit.
;   - FIXED: Subtraction left-operand dropping bug in EAM_MH_RET.
;   - Unified 2-argument parsing for AND/OR/XOR/POKE/LIST to save ~25 bytes.
;   - Deduplicated bare assignments (LET-less statements).
;
; V4.3 (2026-06) - 3672 bytes
;   - FIXED: Nested operator precedence clobbering bug using SWBASE stack.
;   - Added bitwise functions: AND(a,b), OR(a,b), XOR(a,b), NOT(a).
;   - Rewrote default showcase program with an expanded Mandelbrot finale.
;
; V4.2 (2026-06) - 3485 bytes
;   - Rewrote RDLINE using R3 as an IBUF offset optimization.
;   - Merged sign-handling and addition paths into shared ADD16_SAVE_EXP.
;   - Added optional line-range filtering to LIST [start,end].
;
; V4.0 - V4.1 (2026-06)
;   - Implemented function evaluation table (ABS, NEG, PEEK, USR, RND).
;   - FIXED: Signed subtraction boundary bug in line storage shift logic.
;   - Added POKE statement support.
;
; V3.0 - V3.9 (2026-06)
;   - Implemented recursive descent expression parser via software stack.
;   - Added full FOR/NEXT (4-level stack) and GOSUB/RETURN (8-level stack).
;   - Fixed critical memory layout aliasing and carry detection bugs.
;
; V2.3 - V2.8 (2026-05)
;   - Initial optimization baseline with TAB(), CHR$(), and Mandelbrot demo.
;
; =============================================================================

;  ASCII Defines
CR      EQU     $0D
LF      EQU     $0A
BS      EQU     $08
SP      EQU     $20
NUL     EQU     $00
DQ      EQU     $22

;  Token Defines.  A keyword is stored as ONE byte >= $80.  The top three bits are its
;  CLASS, so dispatch needs no table walk (arity lives in the token, never in ROM placement):
;       0xxxxxxx  ASCII, copied verbatim
;       100xxxxx  statement                 $80-$9F   VEC index = token AND $1F
;       101xxxxx  function, no arguments    $A0-$BF   (no '(' ')': PRINT TOP+1)
;       110xxxxx  function, 1 argument      $C0-$DF   '(' expr ')'
;       111xxxxx  function, 2 arguments     $E0-$FC   '(' expr ',' expr [')']
;       $FD TOK_HEX, $FE TOK_DEC            each followed by 2 payload bytes (hi, lo)
;  The three function classes share ONE index space: VEC index = NSTMT + (token AND $1F),
;  so the low 5 bits of a function token are unique across all of them (max 32 functions,
;  32 statements; both checked at assembly time under VEC).
;  Text -> token is carried by the table row itself ([text][NUL][token]).
;  The TOK_xxx values are not typed in: each is its class base plus the position of the
;  handler's row in VEC (labels VS_xxx / VF_xxx), so VEC order and token numbers cannot
;  disagree.  Adding a function renumbers the others; nothing persistent holds a token.
;  To ADD A FUNCTION (three edits, no numbers):
;    1. a text row in FUNC_TAB:           DB "SGN", NUL, TOK_SGN
;    2. a handler row in VEC, any place among the function rows, with a label:
;                                  VF_SGN:  DB <DO_SGN, >DO_SGN
;    3. its token here, choosing the class by arity (ARG1 = one argument, ARG2 = two,
;       FUNC = none):                  TOK_SGN EQU TOK_ARG1_BASE+(VF_SGN-VEC_FN)/2
;  The handler is a subroutine ending in RETC: EXP = the argument (or b), TMP = a (2-arg
;  only); result in EXP.  See FB_CALL for what it may clobber.  A statement is the same
;  with KW_TAB, a VS_xxx row among the first NSTMT rows of VEC and TOK_STMT_BASE.
TOK_STMT_BASE   EQU $80         ; statement tokens
TOK_FUNC_BASE   EQU $A0         ; function tokens: niladic $A0, 1-arg $C0, 2-arg $E0
TOK_ARG1_BASE   EQU $C0         ; first token with an argument list ('(' required)
TOK_ARG2_BASE   EQU $E0         ; first token with two arguments ("a,b")
TOK_HEX         EQU $FD         ; [TOK_HEX][hi][lo] -- 16-bit hex constant
TOK_DEC         EQU $FE         ; [TOK_DEC][hi][lo] -- 16-bit decimal constant
NSTMT           EQU (VEC_FN-VEC)/2   ; statement rows; VEC[NSTMT..] are the functions

; ---- BEGIN keyword tokens (assembler-derived from VEC order; tools/tokmap.py prints them) ----
TOK_END         EQU TOK_STMT_BASE+(VS_END-VEC)/2
TOK_FOR         EQU TOK_STMT_BASE+(VS_FOR-VEC)/2
TOK_FREE        EQU TOK_STMT_BASE+(VS_FREE-VEC)/2
TOK_GOSUB       EQU TOK_STMT_BASE+(VS_GOSUB-VEC)/2
TOK_GOTO        EQU TOK_STMT_BASE+(VS_GOTO-VEC)/2
TOK_IF          EQU TOK_STMT_BASE+(VS_IF-VEC)/2
TOK_INPUT       EQU TOK_STMT_BASE+(VS_INPUT-VEC)/2
TOK_LET         EQU TOK_STMT_BASE+(VS_LET-VEC)/2
TOK_LIST        EQU TOK_STMT_BASE+(VS_LIST-VEC)/2
TOK_NEW         EQU TOK_STMT_BASE+(VS_NEW-VEC)/2
TOK_NEXT        EQU TOK_STMT_BASE+(VS_NEXT-VEC)/2
TOK_POKE        EQU TOK_STMT_BASE+(VS_POKE-VEC)/2
TOK_PRINT       EQU TOK_STMT_BASE+(VS_PRINT-VEC)/2
TOK_REM         EQU TOK_STMT_BASE+(VS_REM-VEC)/2
TOK_RETURN      EQU TOK_STMT_BASE+(VS_RETURN-VEC)/2
TOK_RUN         EQU TOK_STMT_BASE+(VS_RUN-VEC)/2
TOK_THEN        EQU TOK_STMT_BASE+(VS_THEN-VEC)/2   ; consumed by DO_IF; as a statement: ?0
TOK_ABS         EQU TOK_ARG1_BASE+(VF_ABS-VEC_FN)/2     ; 1 argument
TOK_NEG         EQU TOK_ARG1_BASE+(VF_NEG-VEC_FN)/2
TOK_NOT         EQU TOK_ARG1_BASE+(VF_NOT-VEC_FN)/2
TOK_PEEK        EQU TOK_ARG1_BASE+(VF_PEEK-VEC_FN)/2
TOK_RND         EQU TOK_ARG1_BASE+(VF_RND-VEC_FN)/2
TOK_USR         EQU TOK_ARG1_BASE+(VF_USR-VEC_FN)/2
TOK_AND         EQU TOK_ARG2_BASE+(VF_AND-VEC_FN)/2     ; 2 arguments
TOK_OR          EQU TOK_ARG2_BASE+(VF_OR-VEC_FN)/2
TOK_XOR         EQU TOK_ARG2_BASE+(VF_XOR-VEC_FN)/2
TOK_POKEOP      EQU TOK_ARG2_BASE+(VF_POKEOP-VEC_FN)/2  ; internal: POKE/LIST run their "a,b"
TOK_LISTOP      EQU TOK_ARG2_BASE+(VF_LISTOP-VEC_FN)/2  ;   through the function machinery; no
                                                          ;   text row, never in a stored line
; ---- END keyword tokens ----
;  ERROR Defines
ERR_SYN         EQU '0'                 ; syntax (generic)
; ERR_UND_LINE    EQU '1'         ; unused
ERR_DIV_ZERO    EQU '2'                 ; divide by zero
ERR_OOM         EQU '3'                 ; program store full, or GOSUB stack full
ERR_VAR         EQU '4'                 ; variable letter not A-Z
ERR_RET         EQU '5'         ; RETURN without GOSUB (GOSUB stack underflow)
ERR_FOR         EQU '6'         ; Too many nested FORs (FORBASE stack overflow)
ERR_NXT         EQU '7'         ; NEXT without FOR (FORBASE stack underflow)
ERR_NEST        EQU '8'         ; Expression nesting too deep (SW stack full, see PUSH_RET)
ERR_OV          EQU '9'         ; ^ (power): negative exponent

; Stack Defines.  Expression nesting (parentheses, signs, operators, function calls and
; their arguments) uses the SW stack (SWBASE, R3 = index), not the HW RAS; PUSH_RET raises
; ERR_NEST at SW_LIMIT.  The HW RAS only serves fixed-depth helper calls (see RAS DEPTH
; BUDGET).
SW_SIZE         EQU 64          ; bytes reserved for SWBASE
SW_LIMIT        EQU SW_SIZE-5   ; PUSH_RET errors when R3 reaches this after a push; the
                                 ; slack covers the 1-2 unchecked bytes pushed between checks
; PSW Defines
PSW_RS          EQU     $10
PSW_WC          EQU     $08             ; WC (With Carry) bit in PSL (bit 3)
PSW_FLAG        EQU     $40

; REG16_TO_REG16 register indices.
; Usage: LODI,R0 (IDX_src*16)+IDX_dst  /  ZBSR *VREG16_TO_REG16  (dst = src)
IDX_IP    EQU 0
IDX_TMP   EQU 1
IDX_LNUM  EQU 3
IDX_EXP   EQU 4
IDX_SWSTK EQU 5
IDX_GOTO  EQU 6
IDX_CUR   EQU 7
IDX_SC0   EQU 8          ; SC0:SC1
IDX_PE    EQU 9
IDX_TK    EQU 10
IDX_POW   EQU IDX_TK     ; POWCNTH:POWCNTL overlays TKH:TKL
IDX_SAVE  EQU IDX_SC0    ; SAVEH:SAVEL overlays SC0:SC1
IDX_RND   EQU 11         ; RNDSEED

; System Defines
PROGLIM         EQU $1FFF   ; top of program store (numeric constant, not address)
;  GOSUB stack -- managed by SWSP
; Frame = [hi][lo] of NLP.  SWSP = bytes used (0 = empty), +2 per push; 8 frames = 16 bytes.
GSSTKLIM        EQU $10    ; max SWSP before overflow (numeric constant, not address)
;  FOR/NEXT stack -- managed by FORSP
; Frame (7 bytes): [nlpL][nlpH][stpL][stpH][limL][limH][var]; var = VARS byte offset
;   (0/2/../50), lim = signed limit, stp = signed step, nlp = loop-back address.
; FORSP = bytes used (0 = empty), +7 per push (offsets 0/7/14/21); 4 frames = 28 bytes.
; Overflow: FORSP >= FORSTKLIM before push -> ERR_FOR.
FORSTKLIM       EQU $1C   ; max FORSP before overflow (numeric constant, not address)

;  CODE starts at Zero (No Pipbug)
        ORG 0

; =============================================================================
;  RESET / ENTRY + PAGE-ZERO VECTOR TABLE
; In:  nothing (cold start)
; Out: branches to MAIN
;
; Page-zero subroutine vector table.
; Each DW entry holds the absolute address of the subroutine.
; Callers use ZBRR/ZBSR *Vxxx (2 bytes) vs BCTA/BSTA,UN xxx (3 bytes)
;
RESET:
        BCTR,UN MAIN            ; trampoline over vector table ($0000)
VINC_IP:
        DW INC_IP               ; 39 sites
VWSKIP:
        DW WSKIP                ; 24 sites
VINC_TMP:
        DW INC_TMP              ; 5 sites
VCOUT:
        DW COUT                 ; 14 sites
VPARSE_EXPR:
        DW PARSE_EXPR           ; 12 sites
VEATWORD:
        DW EATWORD              ; 8 sites
VPRT_SPACE:
        DW PRT_SPACE            ; 5 sites
VINC_EXP:
        DW INC_EXP              ; 3 sites
VCLR_EXP:
        DW CLR_EXP              ; 4 sites
VEAM_ATOM:
        DW EAM_ATOM             ; 3 sites
VEAM_HI:
        DW EAM_HI               ; 4 sites
VDO_ERROR:
        DW DO_ERROR             ; 3 sites
VJSYNERR:
        DW JSYNERR      ; 3 sites
VREG16_TO_REG16:
        DW REG16_TO_REG16       ; 20 sites; generic packed-nibble 16-bit copy
VFETCH16_IP:
        DW FETCH16_IP           ; 3 sites
VSET_TMP_PROG:
        DW SET_TMP_PROG ; 4 sites
VPUSH_RET:
        DW PUSH_RET              ; 7 sites
VCHECK_POW:
        DW CHECK_POW             ; 8 sites
VPUSH_EXP:
        DW PUSH_EXP              ; 4 sites
VPRINT_S16:
        DW PRINT_S16             ; 5 sites
VEXP_TO_LNUM:
        DW EXP_TO_LNUM  ; 3 sites, shim: LODI,R0 packed, falls into REG16_TO_REG16
VTMP_TO_EXP:
        DW TMP_TO_EXP   ; 4 sites, shim: LODI,R0 packed, falls into REG16_TO_REG16
VPARSER_RET:
        DW PARSER_RET   ; 4 sites
VDLS_BLPX:
        DW DLS_BLPX     ; 4 sites
VCMP_TMP_PE:
        DW CMP_TMP_PE   ; 4 sites
VDIGIT_CHECK:
        DW DIGIT_CHECK  ; 4 sites
VEAM_ATOM_C:
        DW EAM_ATOM_C           ; 8 sites: atom + continuation in R0/R1, frame only if needed
VEAM_HI_C:
        DW EAM_HI_C             ; 3 sites: * / % level only if an operator is next

; =============================================================================
;  MAIN -- cold start: COM=1, clear run state and variables, seed RND, print banner and
;  free bytes, fall into REPL
; In:  nothing
; Out: falls into REPL
; Clobbers: all
MAIN:
        PPSL $02                        ; COM=1: unsigned compares (global)
       
        ; clear RUNFLG, SWSP, FORSP, GC_NEXT (DO_NEW instead for a ROM build)
        BSTA,UN DO_END          

        ; clear A-Z variables (52 bytes)
        LODI,R3 52       ; BDRR decrements, then branches if non-zero: 52 passes, R3 = 52..1,
        EORZ,R0          ; so store at VARS-1,R3 to cover VARS+51..VARS+0
        STRA,R0 GC_NEXT

CLRV:
        STRA,R0 VARS-1,R3
        BDRR,R3 CLRV

        ; Initialize RND seed
        LODI,R1 $AC
        LODI,R0 $E1
        BSTA,UN RND_SKIP

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
        CPSL PSW_RS + 5             ; primary reg bank; clear PSL CC/flag bits.

        CPSU $07                    ; clear PSU SP field (bits 2:0 = HW RAS depth)
                                     ; MUST be separate from CPSL: SP is in PSU not PSL
        LODI,R0 '>'                    ; print prompt only used here
        ZBSR *VCOUT  
        ZBSR *VPRT_SPACE  
        BSTA,UN RDLINE                   ; also sets IPH:IPL = IBUF
        BSTA,UN TRY_STORE_LINE           ; CC=GT: line stored/deleted; CC=EQ: not a line
        BSTA,EQ STMT_EXEC               ; If CC=EQ (not a line), execute
        BCTR,UN REPL

; =============================================================================
;  CMP_LNUM_EXP -- signed 16-bit compare LNUMH:LNUML (left) : EXPH:EXPL (right)
;  The high bytes are compared with COM=0 (signed), the low bytes with COM=1 (unsigned).
;  Used by DO_NEXT (limit : variable); DO_IF has its own inline copy.
; In:  LNUM, EXP
; Out: CC = GT if LNUM > EXP, EQ if equal, LT if LNUM < EXP.  COM restored to 1.
; Clobbers: R0.  Leaf.
CMP_LNUM_EXP:
        LODA,R0 LNUMH
        CPSL $02                         ; COM=0: signed compare of the high bytes
        COMA,R0 EXPH
        PPSL $02                         ; COM=1 again (the global invariant); CC untouched
        RETC,GT
        RETC,LT
        LODA,R0 LNUML
        COMA,R0 EXPL                     ; low bytes: unsigned (COM=1)
        RETC,UN

; =============================================================================
;  DO_IF -- Conditional execution
; Syntax: IF expr relop expr THEN stmt
; In:  IP -> first char after IF keyword
; Out: condition true: falls into STMT_EXEC for the THEN body; false: RETC
; Clobbers: R0, R1, EXP, LNUM, RELOP (+ those of PARSE_EXPR and the THEN statement)
; Notes: RELOP = bitmask of the relops seen (bit0 LT, bit1 EQ, bit2 GT).  The relop parse
;   and the signed compare are inline.  THEN: a token in stored lines; typed text has raw
;   'T','H' + EATWORD.  Nested IFs add no RAS depth.
DO_IF:
        ZBSR *VPARSE_EXPR                 ; [+1]
        LODA,R0 EXPH                      ; LNUMH:LNUML = EXPH:EXPL (save left operand)
        STRA,R0 LNUMH
        LODA,R0 EXPL
        STRA,R0 LNUML
        ; parse the relop into RELOP's bitmask
        ZBSR *VWSKIP                      ; [+1]
        EORZ,R0
        STRZ,R1                          ; R1 = 0 (mask accumulator)
DIF_RELOP_LP:
        LODA,R0 *IPH
        COMI,R0 A'<'
        BCTR,EQ DIF_RELOP_LT
        COMI,R0 A'='
        BCTR,EQ DIF_RELOP_EQ
        COMI,R0 A'>'
        BCTR,EQ DIF_RELOP_GT
        LODZ,R1
        BCTR,EQ LSYNERR                  ; mask still empty: no relop seen at all
        STRA,R0 RELOP
        BCTR,UN DIF_RELOP_DONE
DIF_RELOP_LT:
        IORI,R1 1
        db $EC                  ; COMA,R0: skip next 2 bytes 
DIF_RELOP_EQ:
        IORI,R1 2
        db $EC                  ; COMA,R0: skip next 2 bytes 
DIF_RELOP_GT:
        IORI,R1 4
        ZBSR *VINC_IP  
        BCTR,UN DIF_RELOP_LP
DIF_RELOP_DONE:
        ZBSR *VPARSE_EXPR                 ; [+1]

        ; consume THEN keyword (touches IP and R0 only: LNUM/EXP stay intact for the compare)
        ZBSR *VWSKIP                      ; [+1] R0 = first non-space char (peek)
        COMI,R0 TOK_THEN                  ; stored line: one token
        BCFR,EQ DIF_RAW
        ZBSR *VINC_IP                     ; [+1] consume it
        BCTR,UN DIF_CMP
DIF_RAW:
        COMI,R0 A'T'                      ; typed text: must be 'T'
        BCFR,EQ LSYNERR
        ZBSR *VINC_IP                     ; [+1] past the T (clobbers R0)
        LODA,R0 *IPH                      ; re-peek
        COMI,R0 A'H'                      ; must be 'H' (EATWORD below eats it)
        BCTR,EQ DIF_EW
LSYNERR:        
        ZBRR *VJSYNERR 

DIF_EW:
        ZBSR *VEATWORD                    ; [+1]
DIF_CMP:
        ; signed compare LNUM (left) : EXP (right)
        LODA,R0 LNUMH
        CPSL $02                         ; COM=0: signed compare of the high bytes
        COMA,R0 EXPH
        PPSL $02                         ; COM=1 again (the global invariant); CC untouched
        BCTR,GT DIF_RESULT
        BCTR,LT DIF_RESULT
        LODA,R0 LNUML
        COMA,R0 EXPL                     ; low bytes: unsigned (COM=1)
DIF_RESULT:
        ; map the CC to a bitmask and AND it with RELOP (bit0=LT bit1=EQ bit2=GT)
        BCTR,EQ DIF_IS_EQ
        BCTR,LT DIF_IS_LT
        LODI,R0 4                        ; GT
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to next LODI
DIF_IS_LT:
        LODI,R0 1                        ; LT
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to next LODI
DIF_IS_EQ:
        LODI,R0 2                        ; EQ
        ANDA,R0 RELOP                    ; R0 &= RELOP (sets CC)
        RETC,EQ                          ; no match: condition false, sequential return
        ; drop through
; =============================================================================
;  STMT_EXEC -- Decode and dispatch one BASIC statement from IP (falls into MATCH_KW)
; In:  IPH:IPL -> statement (leading blanks allowed)
; Out: blank statement (NUL): RETC.  Otherwise the handler runs, and its RETC returns to
;      STMT_EXEC's caller (REPL or DR_EXEC)
; Clobbers: R0 (+ the handler's)
STMT_EXEC:
        ZBSR *VWSKIP
        RETC,EQ                           ; blank line / empty THEN body (NUL): nothing to do
                                          ; R0 = first char still, from the WSKIP above
        ; drop through

; =============================================================================
; MATCH_KW -- statement dispatch: a statement token (stored line) or a raw KW_TAB word
;   (direct mode) -> VEC[index] -> handler.
; A token is classified by its bits alone (TOK_STMT_BASE: 100xxxxx), so a stored
;   statement costs no table walk.  A raw word goes through MATCH_ROW and ends up as
;   the same token, then the same index/dispatch tail (SE_IDX).  Function words and
;   function tokens are never statements: they take the miss path (SE_NOTKW) exactly
;   like any other unknown text.
; In:  IPH:IPL -> first char of statement (after any leading whitespace); R0 = that char
; Out: handler jumped to; IP advanced past the matched word/token (miss: IP unchanged
;      -> SE_NOTKW)
; Clobbers: R0, R1, EXPH, EXPL, TKH, TKL, GOTOH, GOTOL
; =============================================================================
MATCH_KW:
        SUBI,R0 TOK_STMT_BASE             ; R0 = token - $80
        COMI,R0 $20                       ; unsigned (COM=1): anything below $80 wrapped high
        BCFR,LT SE_SLOW                   ; not a statement token: raw text
        STRZ,R1                           ; keep the index across INC_IP
        ZBSR *VINC_IP                     ; [+1] consume the 1-byte token
        LODZ,R1                           ; R0 = statement index 0..$1F
SE_IDX:
        ADDZ,R0                           ; 2 bytes per VEC row
        SUBI,R0 1                         ; pre-increment convention of SE_HI_LO
        STRZ,R1
; SE_HI_LO -- VEC row -> GOTOH:GOTOL -> jump.  Statements reach it by branch; FB_CALL
;   reaches it by BSTR/BSTA, so the handler's own RETC returns to the caller.
; In:  R1 = (VEC byte offset of the row) - 1
; Out: jumps to (or calls) the handler
; Clobbers: R0, R1, GOTOH:GOTOL
SE_HI_LO:
        LODA,R0 VEC,R1+                   ; handler hi
        STRA,R0 GOTOH
        LODA,R0 VEC,R1+                   ; handler lo
        STRA,R0 GOTOL
        LODI,R1 2                         ; common loop counter for NOT, AND, OR, XOR
        BCTA,UN *GOTOH                    ; indirect jump
SE_SLOW:
        BSTA,UN WORD_START                ; two letters? (EQ = maybe a word)
        BCFR,EQ SE_NOTKW                  ; no: cannot match any row
        LODI,R0 <KW_TAB
        LODI,R1 >KW_TAB
        BSTA,UN MATCH_ROW                 ; [+1] R1 = token, or 0
        LODZ,R1
        BCTR,EQ SE_NOTKW                  ; no such word
        SUBI,R0 TOK_STMT_BASE
        COMI,R0 $20
        BCTR,LT SE_IDX                    ; a statement word
        LODI,R0 (IDX_EXP*16)+IDX_IP       ; a function word in statement position: MATCH_ROW
        ZBSR *VREG16_TO_REG16             ;   left EXP = the word's start; un-consume it
        BCTR,UN SE_NOTKW                  ;   and take the miss path

; =============================================================================
;  DO_NEW -- Clear program store
; Syntax: NEW
; In:  nothing
; Out: PEH:PEL = PROG (program store emptied); falls through to DO_END
; Clobbers: R0, R1, TMP, SWSP, FORSP, GC_NEXT, RUNFLG
DO_NEW:
        ZBSR *VSET_TMP_PROG               ; TMP = PROG ...
        LODI,R0 (IDX_TMP*16)+IDX_PE
        ZBSR *VREG16_TO_REG16             ; ... PE = TMP
        ; fall through to DO_END

; =============================================================================
;  DO_END -- Stop execution and clear all run state
; Syntax: END  (also called by DO_NEW, DO_ERROR, MAIN)
; In:  nothing
; Out: SWSP=0, FORSP=0, GC_NEXT=0, RUNFLG=0 (tail-calls CLR_RUNFLG)
; Clobbers: R0
DO_END:
        EORZ,R0
        STRA,R0 FORSP                    ; clear FOR stack
        STRA,R0 SWSP                     ; clear GOSUB stack
        STRA,R0 GC_NEXT
        BCTA,UN CLR_RUNFLG               ; tail call (its EORZ,R0 supplies the zero)

; =============================================================================
;  SE_NOTKW -- Variable assignment: bare "V = expr" (reached when the statement is
;  not a keyword) and "LET V = expr" (the LET row of VEC points here too).
; Syntax: [LET] V = expr
; In:  IP -> variable letter (after LET's word, for LET)
; Out: VARS[V] = EXPH:EXPL
; Error: JERRVAR (not A-Z), JSYNERR (no '=')
; Clobbers: R0, R1, R2, SC0, SC1, EXPH, EXPL, TMPH, TMPL
; Note: DO_INPUT jumps to DL_STORE with R2 = VARS byte offset (see PARSE_VAR_SAVE).
SE_NOTKW:
        BSTR,UN PARSE_VAR_SAVE            ; validates A-Z, R2 = VARS byte offset
        ZBSR *VWSKIP
        COMI,R0 A'='
        BCFA,EQ JSYNERR
        ZBSR *VINC_IP
DL_EX:
        ZBSR *VPARSE_EXPR                 ; [+1]
;  DL_STORE -- store EXPH:EXPL into a variable (shared: SE_NOTKW = LET/bare assign,
;  DO_INPUT, DO_FOR)
; In:  R2 = VARS byte offset = 2*(letter-'A') (from PARSE_VAR_SAVE); EXPH:EXPL
; Out: VARS[R2..R2+1] = EXPH:EXPL; R2 unchanged
; Clobbers: R0
DL_STORE:
        LODA,R0 EXPH     ; R0 = High byte of expression
        STRA,R0 VARS,R2  ; Store directly to VARS array + offset (R2 is a valid index reg)
        LODA,R0 EXPL     ; R0 = Low byte of expression
        STRA,R0 VARS+1,R2; Store directly to VARS array + offset + 1
        ; fall through to DO_REM (RETC,UN)

; =============================================================================
;  DO_REM -- No-op / shared return
; Syntax: REM <anything>
; In:  nothing
; Out: nothing
; Clobbers: nothing
DO_REM:
        RETC,UN

; =============================================================================
;  DO_INPUT -- Read signed integer from user into variable
; Syntax: INPUT V
; In:  IP -> variable letter
; Out: VARS[V] = parsed value
; Clobbers: R0, R2, SC0, SC1, EXPH, EXPL, TMPH, TMPL, IBUF
DO_INPUT:
        BSTR,UN PARSE_VAR_SAVE
        BSTA,UN PRT_QUEST
        ZBSR *VPRT_SPACE  
        BSTA,UN RDLINE                   ; [+1] also sets IPH:IPL = IBUF
        BSTA,UN PARSE_S16                ; [+1]
        BCTR,UN DL_STORE

; =============================================================================
; PARSE_VAR_SAVE -- skip whitespace, read var letter, range-check,
;                   save its VARS byte offset in R2, advance IP.
; In:  IP -> variable letter (blanks allowed)
; Out: R2 = 2*(letter-'A') = byte offset into VARS (A-Z); IP advanced past letter
; Error: tail-jumps to JERRVAR (no return)
; Clobbers: R0, R2.  RAS: +2 (WSKIP's INC_IP nests inside; INC_IP is tail-called)
; Range test relies on the global COM=1 (unsigned) set in MAIN: any char below
; 'A' wraps to a large unsigned value, so one compare catches both ends.
PARSE_VAR_SAVE:
        ZBSR *VWSKIP  
        SUBI,R0 A'A'                     ; 'A'-'Z' -> 0-25
        COMI,R0 A'Z'-A'A'
        BCTA,GT JERRVAR                  ; out of range (either end) -- tail jump, no return
        ADDZ,R0                          ; R0 = index*2 (16-bit word stride)
        STRZ,R2                          ; R2 = byte offset, consumed by DL_STORE
        ZBRR *VINC_IP           ; tail call  

; =============================================================================
;  DO_GOTO -- Computed GOTO
; Syntax: GOTO expr
; In:  IP -> first char after GOTO keyword.  DG_GO (entered from DO_GOSUB): EXP = target line
; Out: SWSTK = target record (the line itself, or the next higher one); DR_LP continues
;      there (SWSTK is the program counter).  At the prompt it is harmless: every RUN
;      rewrites SWSTK.
; Clobbers: R0, R1, EXP, LNUM, TMP, SWSTK, GCACHE, GC_NEXT
; Notes: while running, a target above the current line is searched from SWSTK (the next
;   record); otherwise from PROG, after a 2-slot cache (GCACHE) of recent targets.
DO_GOTO:
        ZBSR *VWSKIP  
        ZBSR *VPARSE_EXPR                 ; [+1]
DG_GO:                                    ; DO_GOSUB joins here with EXP = target line
        ZBSR *VEXP_TO_LNUM
        ; running: SWSTK = record after the current line, so a target above the current
        ; line is at or after SWSTK (records are sorted): search from there.  Backward
        ; jumps and self-loops search from the top.
        LODA,R0 RUNFLG
        BCTR,EQ DG_TOP                    ; at the prompt SWSTK is stale
        LODA,R0 LNUMH
        COMA,R0 CURH
        BCTR,GT DG_FWD                    ; target hi > current hi
        BCFR,EQ DG_TOP                    ; target hi < current hi
        LODA,R0 LNUML
        COMA,R0 CURL
        BCFR,GT DG_TOP                    ; target <= current line
DG_FWD:
        LODI,R0 (IDX_SWSTK*16)+IDX_TMP
        ZBSR *VREG16_TO_REG16             ; TMP = SWSTK
        BSTA,UN FIND_LINE_FWD             ; [+1] TMP = target record
        BCTA,UN DG_SET
DG_TOP:
        ; 2-slot cache check, inline (no RAS).  Hit: TMP set, no search.  Miss: FIND_LINE,
        ; then the result is stored round-robin.  GCACHE lies inside LBUF (the tokenizer's
        ; output overwrites it), so at the prompt it may hold stale or garbage entries; that
        ; is harmless (RUN invalidates it).  It sits below IBUF, so INPUT cannot touch it.
        LODI,R1 0
DGC_LP:
        COMI,R1 8
        BCTR,EQ DGC_MISS
        LODA,R0 GCACHE,R1
        COMA,R0 LNUMH
        BCFR,EQ DGC_NEXT
        LODA,R0 GCACHE+1,R1
        COMA,R0 LNUML
        BCFR,EQ DGC_NEXT
        LODA,R0 GCACHE+2,R1
        STRA,R0 TMPH
        LODA,R0 GCACHE+3,R1
        STRA,R0 TMPL
        BCTR,UN DG_SET
DGC_NEXT:
        ADDI,R1 4
        BCTR,UN DGC_LP
DGC_MISS:
        BSTA,UN FIND_LINE                 ; [+1] TMP = target record (unchanged)
        LODA,R1 GC_NEXT
        LODA,R0 LNUMH
        STRA,R0 GCACHE,R1
        LODA,R0 LNUML
        STRA,R0 GCACHE+1,R1
        LODA,R0 TMPH
        STRA,R0 GCACHE+2,R1
        LODA,R0 TMPL
        STRA,R0 GCACHE+3,R1
        ; GC_NEXT is 0 or 4 (DO_RUN resets it), so the next slot is a toggle; R1 still
        ; holds GC_NEXT.  It must never be overlaid: another value would index outside GCACHE.
        LODZ,R1                           ; R0 = old GC_NEXT
        EORI,R0 4                         ; 0 <-> 4
        STRA,R0 GC_NEXT
DG_SET:
        LODI,R0 (IDX_TMP*16)+IDX_SWSTK
        ZBRR *VREG16_TO_REG16             ; SWSTK = TMP (tail call)

; =============================================================================
;  DO_RETURN -- Return from subroutine
; Syntax: RETURN
; In:  SWSP = GOSUB stack bytes used; top frame = GSBASE[SWSP-2]=hi, [SWSP-1]=lo of NLP.
; Out: SWSTK = popped NLP (a program-store address; DR_LP continues from SWSTK).
; Clobbers: R0, R1, SWSTK, SWSP.
; Error: SWSP=0 (underflow) -> ERR_RET.
; Pops lo first, then hi (pre-decrement loads).
DO_RETURN:
        LODA,R1 SWSP
        BCTA,EQ DRT_UNDERFLOW   ; SWSP==0: nothing pushed
        LODA,R0 GSBASE,R1-      ; pre-decrement loads: lo, then hi
        STRA,R0 SWSTK+1
        LODA,R0 GSBASE,R1-
        STRA,R0 SWSTK
        STRA,R1 SWSP            ; SWSP -= 2
        RETC,UN

; =============================================================================
;  SET_TMP_PROG -- TMP = PROG base (falls into SET_TMP)
;  SET_TMP -- TMP = R0:R1 (hi:lo)
; In:  SET_TMP: R0 = hi, R1 = lo
; Out: TMPH:TMPL set
; Clobbers: SET_TMP_PROG: R0, R1.  SET_TMP: none.  Leaf.
SET_TMP_PROG:
        LODI,R0 <PROG
        LODI,R1 >PROG
SET_TMP:
        STRA,R0 TMPH
        STRA,R1 TMPL
        RETC,UN



; =============================================================================
;  RND_SHUFFLE -- advance the 16-bit Galois LFSR (RNDSEED)
; In:  RNDSEED
; Out: RNDSEED advanced one step; R0:R1 = new seed (hi:lo).  RND_SKIP stores R0:R1 as RNDSEED.
; Clobbers: R0, R1, PSL C and WC (WC=0 on exit)
; Notes: taps $B400 (x^16+x^14+x^13+x^11+1), period 65535.  WC must be SET for RRR to chain
;   carry between R0 and R1; with WC clear each register rotates on its own.  CHIN calls
;   this while waiting for a key, so typing time seeds RND.
; =============================================================================
RND_SHUFFLE:
        LODA,R0 RNDSEED         ; Load seed high byte
        LODA,R1 RNDSEED+1       ; Load seed low byte
        CPSL    1               ; Clear Carry (C=0 shifts into bit7 of R0)
        PPSL    PSW_WC          ; Enable WC: RRR now chains carry between regs
        RRR,R0                  ; Shift R0 right: bit0 of R0 -> Carry; 0 -> bit7
        RRR,R1                  ; Shift R1 right: Carry (bit0 of R0) -> bit7 of R1
                                 ;                 bit0 of R1 -> Carry (feedback)
        CPSL    PSW_WC          ; Disable WC (restore normal mode)
        TPSL    1               ; Test Carry: CC=EQ if C=1, CC=LT if C=0
        BCTR,LT RND_SKIP        ; C=0 (CC=LT): feedback bit was 0, skip XOR
        EORI,R0 $B4             ; Apply taps high byte (0xB400)
RND_SKIP:
        STRA,R0 RNDSEED        ; Save seed high byte
        STRA,R1 RNDSEED+1      ; Save seed low byte
        RETC,UN



; =============================================================================
; Character IO
; 110 Baud teletype from PIPBUG V1 as per Signetics M20 application note.
; CHIN/COUT are not pinned to fixed addresses: pipbug_wrap takes them from the .LST
; (see Build).
; =============================================================================
;  PRE_CHIN / CHIN -- read one character from the serial input (blocking)
; In:  WC=0
; Out: R0 = character (bit 7 clear).  RS=WC=0 on exit.
; Clobbers: R0, R1 (via RND_SHUFFLE), alternate-bank R1-R2, RNDSEED
; Notes: PRE_CHIN steps RND_SHUFFLE once and falls into CHIN, which loops back to
;   PRE_CHIN until the start bit arrives.
PRE_CHIN:
        BSTR,UN RND_SHUFFLE
CHIN:
        PPSL PSW_RS
        LODI,R1 0
        LODI,R2 8
        SPSU
        BCTR,LT PRE_CHIN
        EORZ,R0
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
; =============================================================================
;  DLAY -- delay of one bit time (110 baud).  DLY -- about half a bit time (centres the
;  sample after the start bit).
; In:  DLY: R0 = 0
; Out: delay elapsed
; Clobbers: R0.  Leaf.
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
;  COUT -- write one character to the serial output (start bit, 8 data bits LSB first, stop)
; In:  R0 = character; WC=0
; Out: character sent; FLAG left set (idle).  RS=0.
; Clobbers: R0, alternate-bank R1-R2 (primary R1-R3 preserved)
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
        BDRR,R1 ACOU
        BSTR,UN DLAY
        PPSU PSW_FLAG
        CPSL PSW_RS
        RETC,UN

; =============================================================================
;  DO_PRINT -- Print statement.  DP_STRING / PRTSTR: string printer (also used for the banner)
; Syntax: PRINT [item {; item}]
;   item = "string" | expr | TAB(n) | CHR$(n) | HEX$(n)
;   Trailing ';' suppresses the newline.
; In:  IP -> first char after PRINT keyword
; Out: text written to COUT; IP advanced past statement
; Clobbers: R0-R3, EXP, TMP, NEGFLG, SC0, SC1 (+ the rest of PARSE_EXPR's)
; Notes: items are recognised by their first two letters (CH, HE, TA), else parsed as an
;   expression.  PRTSTR prints from IP to the closing quote (consumed) or NUL (returns);
;   DP_STRING first skips the opening quote.
DP_STRING:
        ZBSR *VINC_IP            ; skip opening quote
PRTSTR:
        LODA,R0 *IPH
        RETC,EQ                  ; NUL before closing ": bail
        COMI,R0 DQ
        BCTR,EQ DP_SCLS
        ZBSR *VCOUT  
        BCTR,UN DP_STRING        ; advance past the char, continue

DO_PRINT:
        ZBSR *VWSKIP  
        BCTA,EQ PRT_CRLF         ; empty print -> tail call to newline

DP_ITEM:
        LODI,R1 0                ; initialize lookahead index
        COMI,R0 DQ
        BCTR,EQ DP_STRING

        COMI,R0 'C'
        BCTR,EQ CK_CH
        COMI,R0 'H'
        BCTR,EQ CK_HE
        COMI,R0 'T'
        BCTR,EQ CK_TA

DP_EXPR:
        ; IP was untouched, VPARSE_EXPR starts right at the expression.
        ZBSR *VPARSE_EXPR  
        ZBSR *VPRINT_S16
        BCTA,UN DP_SEP

CK_CH:
        LODA,R0 *IPH,R1+         ; R0 = 2nd char, R1 = 2
        COMI,R0 'H'
        BCFR,EQ DP_EXPR          ; Not "CH" -> must be expression
DP_CHAR:
        ZBSR *VEATWORD           ; Consumes "CHR$" starting from current IP
        ZBSR *VPARSE_EXPR  
        LODA,R0 EXPL
        ZBSR *VCOUT  
        DB $EC  ; swallow next 2 bytes
DP_SCLS:
        ZBSR *VINC_IP  
        BCTR,UN DP_SEP

CK_HE:
        LODA,R0 *IPH,R1+
        COMI,R0 'E'
        BCFR,EQ DP_EXPR
DP_HEXITEM:
        ZBSR *VEATWORD           ; Consumes "HEX$" 
        ZBSR *VPARSE_EXPR  
        LODI,R0 '$'              ; print dollar
        ZBSR *VCOUT  
        LODA,R0 EXPH
        BSTR,UN PRINT_HEX_BYTE
        LODA,R0 EXPL
        BSTR,UN PRINT_HEX_BYTE
        BCTR,UN DP_SEP

CK_TA:
        LODA,R0 *IPH,R1+
        COMI,R0 'A'
        BCFR,EQ DP_EXPR
DP_TAB:
        ZBSR *VEATWORD           ; Consumes "TAB"
        ZBSR *VPARSE_EXPR  
        LODA,R1 EXPL
        BCTR,EQ DP_SEP           ; TAB(0): skip
TAB_LOOP:
        ZBSR *VPRT_SPACE  
        BDRR,R1 TAB_LOOP
        ; fall through to DP_SEP

DP_SEP:
        ZBSR *VWSKIP  
        COMI,R0 $3B              ; semicolon?
        BCFA,EQ PRT_CRLF         ; If not, tail call to newline directly

        ZBSR *VINC_IP            ; Eat semicolon
        ZBSR *VWSKIP  
        RETC,EQ                  ; bail if NUL
        BCTA,UN DP_ITEM

; =============================================================================
;  PRINT_HEX_BYTE -- print a byte as two hex digits (HEX$, LIST)
; In:  R0 = byte
; Out: 2 characters to COUT
; Clobbers: R0, R1
; =============================================================================
PRINT_HEX_BYTE:
        STRZ,R1                  ; R1 = R0 (save original byte)
        RRR,R0
        RRR,R0
        RRR,R0
        RRR,R0                   ; Nibble-swap: hi nibble to low 4 bits
        BSTR,UN PHB_NIB          ; Print high nibble
        LODZ,R1                  ; Restore R0 for low nibble
        ; fall through for low nibble

; PHB_NIB -- print the low nibble of R0 as a hex digit
; In:  R0 (low nibble).  Out: 1 character to COUT (tail call).  Clobbers: R0.
PHB_NIB:
        ANDI,R0 $0F
        ADDI,R0 $30
        COMI,R0 $3A
        BCTR,LT PHB_OUT
        ADDI,R0 7
PHB_OUT:
        ZBRR *VCOUT              ; Tail call to VCOUT (returns to caller)

; =============================================================================
;  DO_GOSUB -- Subroutine call
; Syntax: GOSUB <line>
; In:  IP -> line number; SWSTK[0:1] = NLP from DR_EXEC; SWSP = stack ptr.
; Out: NLP pushed onto GSBASE, then as DO_GOTO (SWSTK = target record).
; Clobbers: as DO_GOTO, plus SWSP
; Error: SWSP >= GSSTKLIM -> ERR_OOM
; Stack: GSBASE[SWSP]=hi, GSBASE[SWSP+1]=lo. SWSP=bytes used, 0=empty.
DO_GOSUB:
        ZBSR *VWSKIP                      ; [+1]
        ZBSR *VPARSE_EXPR                 ; [+1] target line -> EXPH:EXPL
        ; overflow check; R1 (pre-push SWSP) doubles as the store index
        LODA,R1 SWSP
        COMI,R1 GSSTKLIM
        BCTR,LT DGS_STORE
        LODI,R0 ERR_OOM
        ZBRR *VDO_ERROR 
DGS_STORE:
        LODA,R0 SWSTK                    ; NLP hi byte
        STRA,R0 GSBASE-1,R1+             ; pre-increment store: GSBASE[SWSP++] = hi
        LODA,R0 SWSTK+1                  ; NLP lo byte
        STRA,R0 GSBASE-1,R1+             ; GSBASE[SWSP++] = lo (DO_RETURN pops lo first)
        STRA,R1 SWSP                     ; R1 = SWSP+2 = new SWSP
        BCTA,UN DG_GO                    ; then exactly a GOTO

; =============================================================================
;  DO_FOR -- FOR loop setup
; Syntax: FOR V = start TO limit [STEP step]
; In:  IP past "FOR" keyword; SWSTK[0:1] = NLP of line after FOR (set by DR_EXEC).
; Out: Frame pushed on FORBASE; var initialised to start; falls through to body.
; Clobbers: R0, R1, R2, FORVAR, FORSP, LNUMH, LNUML, EXPH, EXPL, TMPH, TMPL.
; Errors: stack full -> ERR_FOR.
; Frame layout (7 bytes at FORBASE[FORSP]): [nlpL][nlpH][stpL][stpH][limL][limH][var]
; Loop body always executes at least once; exit tested at NEXT.
DO_FOR:
        ; --- get variable: validates A-Z, R2 = VARS byte offset (survives PARSE_EXPR) ---
        BSTA,UN PARSE_VAR_SAVE            ; [+1] IP past the letter

        ; --- stack overflow check ---
        LODA,R0 FORSP
        COMI,R0 FORSTKLIM
        BCFA,LT JFORERR                  ; If NOT Less-Than limit (>=), fail!

        ; --- skip '=' then parse start value -> EXPH:EXPL ---
        ZBSR *VWSKIP                      ; [+1] skip whitespace before '='
        ZBSR *VINC_IP                     ; [+1] consume '='
        ZBSR *VPARSE_EXPR                 ; [+1] start value -> EXPH:EXPL
        BSTA,UN DL_STORE                 ; [+1] VARS[R2] = EXPH:EXPL
        STRA,R2 FORVAR                   ; cache the VARS offset for the frame
        ; --- consume "TO" keyword ---
        ZBSR *VWSKIP                      ; [+1]
        ZBSR *VEATWORD                    ; [+1]
        ; --- parse limit -> LNUMH:LNUML ---
        ZBSR *VPARSE_EXPR                 ; [+1]
        ZBSR *VEXP_TO_LNUM ; LNUMH:LNUML = EXPH:EXPL (limit)
        ; --- check for STEP keyword ---
        ;   If 'S': EATWORD consumes all of "STEP", then parse step.
        ;   Else: use default step = +1.
        ZBSR *VWSKIP                      ; [+1] R0 = first non-space char (peek)
        COMI,R0 A'S'
        BCTR,EQ DF_STEP
        ; not 'S': use step=+1
        ZBSR *VCLR_EXP                          ; step hi = 0
        ZBSR *VINC_EXP                          ; step lo = 1
        BCTR,UN DF_PUSH
DF_STEP:
        ZBSR *VEATWORD                    ; [+1] consume "TEP"
        ZBSR *VPARSE_EXPR                 ; [+1] step -> EXPH:EXPL
DF_PUSH:
        ; Push the 7-byte frame at FORBASE[FORSP]: RAM FORVAR..SWSTK+1 is read top-down and
        ; stored with pre-increment (FORBASE-1,R1+).
        ; Frame: [nlpL][nlpH][stpL][stpH][limL][limH][var]
        LODA,R1 FORSP
        LODI,R2 7
DF_LOOP:
        LODA,R0 FORVAR,R2-               ; pre-decrement: FORVAR+6 (SWSTK+1) ... FORVAR+0
        STRA,R0 FORBASE-1,R1+            ; pre-increment: FORBASE[FORSP++]
        BRNR,R2 DF_LOOP
        STRA,R1 FORSP                    ; R1 = old FORSP + 7
        RETC,UN

; =============================================================================
;  DO_NEXT -- FOR loop iteration
; Syntax: NEXT [V]
; In:  FORSP = FOR stack bytes used (0 = empty); top frame at FORBASE[FORSP-7 ..].
;      Frame: [nlpL][nlpH][stpL][stpH][limL][limH][var]; var = VARS byte offset.
; Out: Looping: SWSTK = the loop body (DR_LP continues there).
;      Done: frame popped (FORSP -= 7); sequential return.  A var+step that overflows
;      16 bits ends the loop (OVF test), so ranges up to +/-32767 terminate exactly.
; Method: pop var/limit/step back into FORVAR/LNUM/EXP (5-byte loop), new var =
;      var + step via ADD_CORE + DL_STORE, then one signed compare limit : var
;      (CMP_LNUM_EXP).  EQ -> loop again (body runs at the limit value).  Otherwise
;      the LT bit of that compare XOR the step's sign decides: they agree ->
;      still inside the range -> loop; else exit.
; Clobbers: R0, R1, R2, R3, FORVAR, EXPH, EXPL, LNUMH, LNUML, SWSTK, FORSP.
; Errors: FORSP=0 -> ERR_NXT.
; The variable name after NEXT is consumed but not checked (smallest code).
; =============================================================================
DO_NEXT:
        LODA,R3 FORSP
        BCTA,EQ JERR_NXT                  ; FORSP==0: no FOR active
        ZBSR *VWSKIP                      ; [+1]
        ZBSR *VEATWORD                    ; [+1] consume optional var name
        SUBI,R3 7                         ; R3 = frame base index
        LODI,R2 5
DN_POP:                                   ; frame[2..6] -> EXPL, EXPH, LNUML, LNUMH, FORVAR
        LODA,R0 FORBASE+1,R3+             ; pre-increment: frame[2] first
        STRA,R0 FORVAR,R2-                ; pre-decrement: FORVAR+4 (EXPL) down to FORVAR+0
        BRNR,R2 DN_POP
        LODA,R2 FORVAR                    ; R2 = VARS byte offset
        SUBI,R3 5                         ; R3 = frame base again
        LODI,R0 VARS-IPH
        ADDZ,R2
        BSTA,UN ADD_CORE                  ; [+1] EXP = step + var = the new value
        TPSL $04                          ; OVF: var+step overflowed 16 bits, so the loop is
        BCTR,EQ DN_DONE                   ;   over (the wrapped value must not be compared)
        BSTA,UN DL_STORE                  ; VARS[R2] = EXP
        LODA,R0 FORBASE+3,R3              ; step hi byte: its bit 7 is the step's sign
        STRZ,R2
        BSTA,UN CMP_LNUM_EXP              ; [+1] CC = limit : var (signed)
        BCTR,EQ DN_GO                     ; var = limit: one more pass
        SPSL                              ; R0 bit 7 = CC1 = (limit < var)
        EORZ,R2                           ; ^ step sign: bit 7 clear = agree = still in range
        BCFR,LT DN_GO
DN_DONE:
        STRA,R3 FORSP                     ; done: pop the frame (FORSP = its base)
        RETC,UN
DN_GO:
        LODI,R2 2                         ; loop body: frame nlp -> SWSTK (hi, lo)
DN_BODY:
        LODA,R0 FORBASE-1,R3+
        STRA,R0 SWSTK,R2-
        BRNR,R2 DN_BODY
        RETC,UN

; =============================================================================
;  DO_RUN -- Execute stored program
; Syntax: RUN
; In:  PROG=program base, PEH:PEL=program end
; Out: runs until END, error, or exhausted; returns to REPL
; Clobbers: all
; After STMT_EXEC, SWSTK always holds the next record to run: the following line
; (set by DR_EXEC) unless GOTO/GOSUB/RETURN/NEXT rewrote it.
DO_RUN:
        LODI,R0 $FF              ; invalidate the GOTO cache (sentinel;
        STRA,R0 GCACHE           ;   no real line number is $FFFF)
        STRA,R0 GCACHE+1
        STRA,R0 GCACHE+4
        STRA,R0 GCACHE+5
        EORZ,R0
        STRA,R0 GC_NEXT
        LODI,R0 1
        STRA,R0 RUNFLG
        ZBSR *VSET_TMP_PROG
DR_LP:
        LODA,R0 RUNFLG
        RETC,EQ
        ; end of program? TMPH:TMPL >= PEH:PEL
        ZBSR *VCMP_TMP_PE
        BCFA,LT DR_STOP                  ; TMP >= PE (GT or EQ): end of program, clear RUNFLG
DR_EXEC:
        ; Setup straight from the record at TMP: CUR = line number; IP = TMP+3 (body);
        ; SWSTK = TMP+len+4 (next record; GOTO/GOSUB/RETURN/NEXT write it, TMP is reloaded
        ; from it).  TMP stays at this record during the statement.
        LODI,R1 $FF
        LODA,R0 *TMPH,R1+                 ; line hi
        STRA,R0 CURH
        LODA,R0 *TMPH,R1+                 ; line lo
        STRA,R0 CURL
        LODA,R0 *TMPH,R1+                 ; body length
        ADDI,R0 4                         ; + line#(2) + len(1) + NUL(1)
        STRZ,R1                           ; R1 = record size
        LODA,R0 TMPL                      ; IP = TMP+3
        ADDI,R0 3
        STRA,R0 IPL
        LODA,R0 TMPH
        TPSL $01                          ; carry?
        BCTR,LT DRX_H1
        ADDI,R0 1
DRX_H1:
        STRA,R0 IPH
        LODA,R0 TMPL                      ; SWSTK = TMP + size
        ADDZ,R1
        STRA,R0 SWSTK+1
        LODA,R0 TMPH
        TPSL $01
        BCTR,LT DRX_H2
        ADDI,R0 1
DRX_H2:
        STRA,R0 SWSTK
        ; execute line
        BSTA,UN STMT_EXEC                ; [+1]
        ;
        LODA,R0 SWSTK                      ; TMP = SWSTK (next-record pointer)
        STRA,R0 TMPH
        LODA,R0 SWSTK+1
        STRA,R0 TMPL
        ;
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
;  MATCH_ROW -- match the word at IP against KW_TAB / FUNC_TAB (full spelling required).
;  Used by:
;    - TOKENIZE_LINE (KW_TAB, which runs on through FUNC_TAB)   wants the token
;    - MATCH_KW      (KW_TAB, direct mode)                      wants the token
;    - EAM_ATOM      (FUNC_TAB, direct mode)                    wants the token
; Table format: [text][NUL][token] per row; the terminator row is a lone NUL.  The token
;   is the row's own payload, so row order is free of token values, and no counter is
;   needed: R2 and R3 are NOT touched (R2 is the live variable letter of an assignment,
;   R3 the tokenizer's TOKBUF index).
; In:  IPH:IPL -> first letter; R0:R1 = table address (hi:lo; stored to TKH:TKL)
; Out: matched: R1 = the row's token (>= $80); IP advanced past the word (trailing letters
;      swallowed by EATWORD); EXP = IP as it was before the word (callers that reject the
;      match rewind with IDX_EXP -> IDX_IP)
;      no match: R1 = 0; IP unchanged
; Clobbers: R0, R1, EXPH, EXPL, TKH, TKL.
; RAS: +1 (REG16/INC_IP nest inside, EATWORD is tail-called)
MATCH_ROW:
        STRA,R0 TKH
        STRA,R1 TKL
        LODI,R1 $FF                       ; pre-increment mode (2650 quirk)
MR_SCAN:
        LODI,R0 (IDX_IP*16)+IDX_EXP
        ZBSR *VREG16_TO_REG16             ; EXP = IP: save this row-attempt's start IP
        LODA,R0 *TKH,R1+                  ; row's first char
        BCTR,EQ MR_MISS                   ; NUL as first char: terminator row
MR_CHAR:
        COMA,R0 *IPH                      ; table char vs input char (nothing clobbered)
        BCFR,EQ MR_MISMATCH
        ZBSR *VINC_IP                     ; matched: consume this input char
        LODA,R0 *TKH,R1+                  ; next table char
        BCFR,EQ MR_CHAR                   ; not the NUL yet
        LODA,R0 *TKH,R1+                  ; row exhausted: its token
        STRZ,R1
        ZBRR *VEATWORD                    ; tail call: swallow the rest of the word
MR_MISMATCH:
        LODI,R0 (IDX_EXP*16)+IDX_IP
        ZBSR *VREG16_TO_REG16             ; IP = EXP: rewind IP to this row's start
MR_SKIP:
        LODA,R0 *TKH,R1+                  ; scan to this row's NUL terminator
        BCFR,EQ MR_SKIP
        ADDI,R1 1                         ; skip the token byte -> next row's start
        BCTR,UN MR_SCAN
MR_MISS:
        LODI,R1 0
        RETC,UN

; =============================================================================
;  TL_HEXVAL -- test/convert one uppercase hex digit character
; In:  R0 = character to test
; Out: is a hex digit ('0'-'9' or 'A'-'F'): R0 = value (0-15), R1 = 0
;      not a hex digit: R1 = $FF (R0 clobbered)
; Clobbers: R0, R1 (NOT R2: PARSE_EXPR's live variable offset; also reached from
;   PARSE_FACTOR via HEX_PEEK/PARSE_HEX)
TL_HEXVAL:
        STRZ,R1                           ; R1 = original char (temp use)
        SUBI,R0 A'0'
        COMI,R0 9
        BCTR,GT TL_HV_ALPHA                ; >9: not 0-9, check A-F
        LODI,R1 0                          ; matched: R0 already = value (0-9)
        RETC,UN
TL_HV_ALPHA:
        LODZ,R1                            ; R0 = original char again
        SUBI,R0 A'A'
        COMI,R0 5
        BCTR,GT TL_HV_NONE                  ; not 'A'-'F' either
        ADDI,R0 10                          ; rebase 0-5 -> 10-15
        LODI,R1 0
        RETC,UN
TL_HV_NONE:
        LODI,R1 $FF
        RETC,UN

; =============================================================================
;  TOKENIZE_LINE -- convert a typed statement body into tokenized form
;  Called only from TRY_STORE_LINE (stored lines); direct statements stay raw text.
;  Single left-to-right pass.  Outside strings and REM bodies, a run of A-Z is tried
;  against KW_TAB then FUNC_TAB (full spelling); if neither matches, one letter is a
;  variable and is copied.  A digit run becomes TOK_DEC + its 16-bit value; '$' + a hex
;  digit becomes TOK_HEX + its value (a bare '$' is copied, e.g. in HEX$/CHR$, which are
;  not tokenized).  A leading '-' is not folded into a constant (it is a unary operator
;  at run time).  String literals and REM bodies are copied verbatim.
; In:  IPH:IPL -> first char of the typed body (past the line number and following space),
;      inside IBUF (the top 64 bytes of LBUF)
; Out: TOKBUF = tokenized body, NUL-terminated; R3 = its length excluding the NUL.
;      IN PLACE: TOKBUF is LBUF's byte 0 upward and overwrites the typed line behind the
;      reader, so IBUF is destroyed.  Safe because the write index never passes IP: a
;      keyword emits 1 byte for >= 2 chars, a variable/string/REM/punctuation byte is 1:1,
;      only a constant expands (1 digit: +2, 2 digits or '$'+1 hex: +1), by at most 62
;      bytes in all; IBUF starts 72 bytes into LBUF (see RAM OVERLAYS).
; Clobbers: R0, R1, R3, IP, EXP, TK, SC0, R3SAVE.  TMP is preserved (TRY_STORE_LINE keeps
;   its insertion point there); R2 is preserved.
; Note: TL_DONE has no RETC: it falls into TRY_STORE_LINE, whose digit test fails on the
;   NUL at IP and returns.  Keep TRY_STORE_LINE directly after TL_DONE.
TOKENIZE_LINE:
        LODI,R3 $FF                       ; TOKBUF write index (pre-increment)
TL_LP:
        LODA,R0 *IPH
        BCTA,EQ TL_DONE                   ; NUL: end of typed body
        COMI,R0 DQ                        ; string literal?
        BCTR,EQ TL_STRING
        COMI,R0 A'A'
        BCTR,LT TL_NOTLETTER               
        COMI,R0 A'Z'+1
        BCTR,LT TL_WORD                   ; 'A'-'Z'
TL_NOTLETTER:
        COMI,R0 A'0'
        BCTR,LT TL_NOTDIGIT                
        COMI,R0 A'9'+1
        BCTA,LT TL_DECIMAL                 ; '0'-'9'
TL_NOTDIGIT:
        COMI,R0 '$'
        BCTA,EQ TL_MAYBEHEX
        ; ordinary byte (operator, punctuation, space, etc.): copy verbatim
TL_COPY1:                                  ; copy the byte in R0 (also entered from TL_NOWORD, TL_DOLLAR_PLAIN)
        STRA,R0 TOKBUF,R3+
        ZBSR *VINC_IP
        BCTR,UN TL_LP
TL_STRING:
        STRA,R0 TOKBUF,R3+                 ; copy the opening quote
        ZBSR *VINC_IP
TL_STR_LP:
        LODA,R0 *IPH
        BCTA,EQ TL_DONE                    ; unterminated string: same
                                          ;   tolerance as PRTSTR at runtime
        STRA,R0 TOKBUF,R3+
        ZBSR *VINC_IP                     
        LODA,R0 TOKBUF,R3
        COMI,R0 DQ
        BCFR,EQ TL_STR_LP                  ; keep going until the closing quote
        BCTR,UN TL_LP                      
TL_WORD:
        LODI,R0 <KW_TAB
        LODI,R1 >KW_TAB
        BSTA,UN MATCH_ROW                  ; KW_TAB continues into FUNC_TAB: R1 = token or 0
        LODZ,R1
        BCTR,EQ TL_NOWORD
        STRA,R0 TOKBUF,R3+                 ; the token
        COMI,R1 TOK_REM
        BCTR,EQ TL_REM
        BCTA,UN TL_LP
TL_NOWORD:
        ; no keyword or function matched: this is a variable reference,
        ; always exactly one letter
        LODA,R0 *IPH
        BCTR,UN TL_COPY1
TL_REM:
        ; REM: the rest of the line is an opaque comment, copied verbatim
        LODA,R0 *IPH
        BCTA,EQ TL_DONE
        STRA,R0 TOKBUF,R3+
        ZBSR *VINC_IP
        BCTR,UN TL_REM
TL_DECIMAL:
        ZBSR *VCLR_EXP
TL_DEC_LP:
        ZBSR *VDIGIT_CHECK
        BCTR,GT TL_DEC_DONE                 ; non-digit: stop accumulating
        STRA,R0 SC0                          ; this digit's value (0-9)
        ZBSR *VINC_IP
        LODI,R0 10
        BSTA,UN TL_MULADD                    ; EXP = EXP*10 + SC0
        BCTR,UN TL_DEC_LP
TL_DEC_DONE:
        LODI,R0 TOK_DEC
TL_EMIT_CONST:                                ; shared by TL_DEC_DONE, TL_MAYBEHEX
        STRA,R0 TOKBUF,R3+                    ;   In: R0 = TOK_DEC or TOK_HEX
        LODA,R0 EXPH
        STRA,R0 TOKBUF,R3+
        LODA,R0 EXPL
        STRA,R0 TOKBUF,R3+
        BCTA,UN TL_LP
TL_MAYBEHEX:
        BSTR,UN HEX_PEEK                      ; '$' + hex digit?
        BCTR,EQ TL_DOLLAR_PLAIN               ; no: plain '$'
        BSTR,UN PARSE_HEX                     ; EXP = value, IP past the digits
        LODI,R0 TOK_HEX
        BCTR,UN TL_EMIT_CONST

; =============================================================================
;  HEX_PEEK -- is IP -> '$' immediately followed by a hex digit?
;  (shared: TL_MAYBEHEX and PARSE_FACTOR)
; In:  IPH:IPL -> '$'
; Out: CC=EQ: no (R1 = $FF).  CC!=EQ: yes.  IP unchanged.
; Clobbers: R0, R1.  RAS: +2 (TL_HEXVAL nests inside)
HEX_PEEK:
        LODI,R1 1
        LODA,R0 *IPH,R1                      ; the char AFTER '$'
        BSTA,UN TL_HEXVAL
        COMI,R1 $FF
        RETC,UN

; =============================================================================
;  PARSE_HEX -- consume '$' and its hex digits into EXPH:EXPL
;  (shared: TL_MAYBEHEX and, via PF_NUM, direct-entry expressions)
; In:  IPH:IPL -> '$' followed by >= 1 hex digit (caller checked via HEX_PEEK)
; Out: EXPH:EXPL = value (16-bit, wraps mod 65536); IP -> first non-hex char
; Clobbers: R0, R1, SC0, EXPH, EXPL, TKH, TKL, R3SAVE.  R3 preserved.
; RAS: +2 (TL_MULADD nests its helpers). Tail-jump it (BCTA,UN) from
;   PARSE_FACTOR, never BSTA it, or the worst case reaches depth 8.
PARSE_HEX:
        ZBSR *VINC_IP                         ; consume '$'
        ZBSR *VCLR_EXP
PH_LP:
        LODA,R0 *IPH
        BSTA,UN TL_HEXVAL
        COMI,R1 $FF
        RETC,EQ                               ; non-hex char: done
        STRA,R0 SC0                           ; this digit's value (0-15)
        ZBSR *VINC_IP
        LODI,R0 16
        BSTR,UN TL_MULADD                     ; EXP = EXP*16 + SC0
        BCTR,UN PH_LP
TL_DOLLAR_PLAIN:
        LODA,R0 *IPH                             ; the '$' itself
        BCTA,UN TL_COPY1

; =============================================================================
;  TL_MULADD -- EXP = EXP*<multiplier> + SC0, by repeated addition.  Shared by the
;  tokenizer's decimal (x10) and hex (x16) accumulation and by PARSE_U16.
; In:  R0 = multiplier (10 or 16); SC0 = digit to add; EXPH:EXPL = value;
;      R3 = caller's index (TOKBUF write index in the tokenizer)
; Out: EXPH:EXPL = EXPH:EXPL * R0 + SC0; R3 unchanged (kept in R3SAVE)
; Clobbers: R0, R1, TKH, TKL, R3SAVE
TL_MULADD:
        STRA,R3 R3SAVE                       ; preserve caller's R3
        STRZ,R3                              
        LODI,R0 (IDX_EXP*16)+IDX_TK          ; TK = EXP (value before scaling)
        ZBSR *VREG16_TO_REG16
        EORZ,R0
        STRA,R0 EXPH
        LODA,R0 SC0
        STRA,R0 EXPL                         ; EXP = digit: the adds below accumulate into it,
                                             ;   giving digit + mult*value with no final add
TL_MULADD_LP:
        LODI,R0 TKH-IPH
        BSTA,UN ADD_CORE                     ; EXP += TK
        BDRR,R3 TL_MULADD_LP
        LODA,R3 R3SAVE                       ; restore caller's R3
        RETC,UN
TL_DONE:
        EORZ,R0
        STRA,R0 TOKBUF,R3+                        ; NUL-terminate
     
; =============================================================================
;  TRY_STORE_LINE -- Store or delete a numbered line if IP starts with a digit
; In:  IPH:IPL -> input buffer
; Out: CC=GT if line stored/deleted; CC=EQ if not a numbered line
; Clobbers: R0-R3, EXP, LNUM, IP, TMP, PE, TK, SC0, R3SAVE
; Error: ERR_OOM if the line does not fit (store unchanged)
TRY_STORE_LINE:
        ZBSR *VDIGIT_CHECK
        BCFR,GT TSL_NUM                  ; not GT (R0<=9 unsigned): valid digit
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
        ZBSR *VEXP_TO_LNUM ; LNUMH:LNUML = EXPH:EXPL (parsed line number)
TSL_FND:
        BSTA,UN FIND_LINE                ; [+1] TMP = insertion point (first record
                                          ; >= LNUM); CC=EQ iff that record IS LNUM
        BCFR,EQ TSL_WRITE                 ; no such line: go store the new one
        BSTR,UN DEL_REC                   
        BCTR,UN TSL_FND
TSL_WRITE:
        ZBSR *VWSKIP                      ; [+1] skip space after line number
        LODA,R0 *IPH
        BCTR,EQ TSL_DONE                  ; NUL: empty body = delete only (done)
        BSTA,UN TOKENIZE_LINE             
        LODZ,R3                            ; save the body length (R3) before R3 becomes the gap/copy size
        STRA,R0 TSL_BLEN
        ADDI,R3 4                          ; +2 header +1 len +1 NUL = total record size
        BSTA,UN OPEN_GAP                  ; make room at TMP; PE += R3
        LODI,R1 2                         ; write the 2-byte line-number header
TSL_HDR:
        LODA,R0 LNUMH,R1-                 ; R1 2->1: LNUML; 1->0: LNUMH
        STRA,R0 *TMPH,R1                  ; -> TMP[1] then TMP[0]
        BRNR,R1 TSL_HDR
        ZBSR *VINC_TMP
        ZBSR *VINC_TMP
        LODA,R0 TSL_BLEN                   ; write the length byte
        STRA,R0 *TMPH
        ZBSR *VINC_TMP
        LODI,R1 $FF                       
        SUBI,R3 3                         ; R3: bodylen+4 -> bodylen+1 (body+NUL to copy)
TSL_CPY:
        LODA,R0 TOKBUF,R1+                ; copy body+NUL by count (a TOK_DEC/TOK_HEX payload can hold $00)
        STRA,R0 *TMPH
        ZBSR *VINC_TMP
        BDRR,R3 TSL_CPY
TSL_DONE:                                 ; PE was already updated by OPEN_GAP
        LODI,R0 1                         ; CC=GT: line stored/deleted
        RETC,UN

; =============================================================================
;  DEL_REC -- Delete the stored record at TMP; close the hole; PE -= its length
;  Forward copy: src = TMP (start of the next record), dst = EXP (start of the
;  deleted one). When src reaches PE, dst IS the new PE - no length needed, and
;  deleting the last record simply runs the loop zero times.
; In:  TMP = start of the record to delete
; Out: PE reduced by the record's length
; Clobbers: R0, R1, TMPH:TMPL, EXPH:EXPL
DEL_REC:
        ZBSR *VTMP_TO_EXP ; EXP = dst = start of the doomed record
        BSTA,UN ADV_TMP_PAST_REC          ; TMP = src = start of the next record
DEL_LP:
        ZBSR *VCMP_TMP_PE
        BCFR,LT DEL_END                  ; src >= PE: everything moved
        LODA,R0 *TMPH
        STRA,R0 *EXPH
        ZBSR *VINC_TMP
        ZBSR *VINC_EXP                   ; dst++
        BCTR,UN DEL_LP
DEL_END:
        LODI,R0 (IDX_EXP*16)+IDX_PE
        ZBRR *VREG16_TO_REG16             ; PE = dst (tail call)

; =============================================================================
;  OPEN_GAP -- Open an R3-byte gap at TMP by moving [TMP,PE) up R3 bytes; PE += R3
; In:  TMP = insertion point; R3 = gap size (1..255); PE = end of store
; Out: gap open at TMP (contents stale); PE += R3; TMP, R3 unchanged.
;      New PE past PROGLIM: ERR_OOM via JERROOM; PE and the store are unchanged.
; Clobbers: R0, R2, EXP
; Notes: the new PE is built in EXP and range-checked before anything moves.  The backward
;   copy walks PE down as the source (CMP_TMP_PE is the loop bound); the destination comes
;   free from the indexed store (*PEH,R3).  PE = EXP at the end.
OPEN_GAP:
        LODI,R0 (IDX_PE*16)+IDX_EXP
        ZBSR *VREG16_TO_REG16             ; EXP = PE
        LODZ,R3
        STRZ,R2                          ; R2 = size, as the INC_ET step count
OG_ADD:
        ZBSR *VINC_EXP                   ; EXP++ (R2 times: EXP = PE + size)
        BDRR,R2 OG_ADD
        LODA,R0 EXPH
        COMI,R0 <PROGLIM+1
        BCFA,LT JERROOM                  ; new PE is past PROGLIM: out of memory
OG_LP:
        BSTR,UN CMP_TMP_PE
        BCFR,LT OG_FIX                   ; PE has walked down to TMP: done
        ; PE-- (16-bit)
        LODA,R0 PEL
        SUBI,R0 1
        STRA,R0 PEL
        TPSL $01                         ; C=1(no borrow)->CC=EQ: skip hi--
        BCTR,EQ OG_NB
        LODA,R0 PEH
        SUBI,R0 1
        STRA,R0 PEH
OG_NB:
        LODA,R0 *PEH                     ; byte at PE ...
        STRA,R0 *PEH,R3                  ; ... goes to PE+R3 (indexed-indirect)
        BCTR,UN OG_LP
OG_FIX:
        LODI,R0 (IDX_EXP*16)+IDX_PE
        ZBRR *VREG16_TO_REG16             ; PE = EXP (tail call)

; =============================================================================
;  CMP_TMP_PE -- Compare TMPH:TMPL against PEH:PEL (16-bit unsigned compare,
;  byte-serial COMA under the global COM=1).
; In:  TMP, PE
; Out: CC=GT if TMP > PE, CC=EQ if TMP = PE, CC=LT if TMP < PE
; Clobbers: R0
CMP_TMP_PE:
        LODA,R0 TMPH
        COMA,R0 PEH
        RETC,GT
        RETC,LT
        LODA,R0 TMPL
        COMA,R0 PEL
        RETC,UN

; =============================================================================
;  ADV_TMP_PAST_REC -- advance TMPH:TMPL past a stored record, landing on the next one
;  Records are [lineH][lineL][len][body...][NUL], len = body byte count (NUL not counted),
;  so the skip is one length read plus an add: O(1).
; In:  TMPH:TMPL -> start of a record (its line-number hi byte)
; Out: TMPH:TMPL -> start of the next record
; Clobbers: R0, R1.  Leaf (no RAS use).  len+4 must fit a byte (the tokenizer limits it to 125).
ADV_TMP_PAST_REC:
        LODI,R1 2
        LODA,R0 *TMPH,R1                 ; R0 = stored body length
        ADDI,R0 4                        ; + 2(line#) + 1(len byte) + 1(NUL)
        STRZ,R1                          ; R1 = total record size
        LODA,R0 TMPL
        ADDZ,R1                          ; TMPL += R1
        STRA,R0 TMPL
        TPSL $01                         ; carry out of the lo byte?
        RETC,LT                          ;   no: done
        LODA,R0 TMPH
        ADDI,R0 1
        STRA,R0 TMPH
        RETC,UN

; =============================================================================
;  FIND_LINE -- Search for line LNUMH:LNUML in the program store
;  FIND_LINE_FWD -- the same, starting at TMP instead of PROG
; In:  LNUM = target line; FIND_LINE_FWD: TMP = first record to examine
; Out: TMP = the record if found, else the first record above LNUM (or PE).
;      CC=EQ found, CC=GT not found.
; Clobbers: R0, R1, TMP.  RAS: +1 (FIND_INS / FI_LP)
FIND_LINE_FWD:
        BSTR,UN FI_LP                    ; [+1]
        db $EC                           ; COMA,R0: swallows FIND_LINE's BSTR below (R0 and CC
                                         ;   are dead at FL_TAIL; the read address is harmless)
FIND_LINE:
        BSTR,UN FIND_INS                 ; [+1]
FL_TAIL:
        ; check if at end of program
        BSTR,UN CMP_TMP_PE                ; [+1]
        BCFR,LT FL_RET_NF                 ; not LT (GT or EQ): at/past end
FL_CHK:
        LODA,R0 *TMPH
        SUBA,R0 LNUMH
        BCTR,EQ FL_CHKLO
FL_RET_NF:
        LODI,R0 1                        ; CC=GT: not found
        RETC,UN

FL_CHKLO:
        LODI,R1 1
        LODA,R0 *TMPH,R1                 ; peek stored.lo at TMP+1 (indirect-indexed)
        SUBA,R0 LNUML
        BCFR,EQ FL_RET_NF
FL_FOUND:
        EORZ,R0                          ; CC=EQ: found
        RETC,UN

; =============================================================================
;  FIND_INS -- Find sorted insertion point for LNUMH:LNUML
; Returns TMPH:TMPL = address of first record with line >= LNUMH:LNUML,
; or PEH:PEL if all lines are smaller.
; In:  LNUMH:LNUML = target line number
; Out: TMPH:TMPL = insertion point
; Clobbers: R0, R1, TMPH, TMPL
FIND_INS:
        ZBSR *VSET_TMP_PROG
FI_LP:
        ZBSR *VCMP_TMP_PE
        RETC,GT
        RETC,EQ
        LODA,R0 LNUMH
        SUBA,R0 *TMPH                    ; LNUMH - stored.hi
        BCTR,GT FI_ADV
        RETC,LT 
        ; hi bytes equal: check lo
        LODI,R1 1
        LODA,R0 LNUML
        COMA,R0 *TMPH,R1                  ; peek stored.lo at TMP+1 (indirect-
                                           ; indexed); unsigned (COM=1 global)
        BCTR,GT FI_ADV
        RETC,UN
FI_ADV:
        ; advance TMPH:TMPL past this record to the next one
        BSTA,UN ADV_TMP_PAST_REC           ; [+1] TMP = next record
        BCTR,UN FI_LP

; =============================================================================
;  EAM_ATOM -- parse one atom and return through PARSER_RET: a signed atom, a
;  parenthesised expression, a function call, or a variable/constant (PARSE_FACTOR).
;  A function call is an atom like '(': its arguments are parsed on the live SW stack,
;  so it uses no HW RAS levels.  See FUNC_BODY for the frame.
;  Call sites go through EAM_ATOM_C, which handles plain atoms without a frame.
; In:  IP -> atom; R3 = SW stack pointer, the caller's continuation already on it
; Out: EXPH:EXPL = value, IP past the atom; returns via PARSER_RET
; Clobbers: R0, R1, TMP, EXP, SC0 ... (see FB_CALL / PARSE_FACTOR); R2 and R3 preserved
EAM_ATOM:
        ZBSR *VWSKIP
        COMI,R0 A'-'
        BCTA,EQ EAM_NEG
        COMI,R0 A'+'
        BCTA,EQ EAM_POS
        COMI,R0 A'('
        BCTA,EQ EAM_PAREN
        SUBI,R0 TOK_FUNC_BASE
        COMI,R0 TOK_HEX-TOK_FUNC_BASE     ; $A0-$FC: a function token, classified by its bits
        BCTR,LT EA_TOKEN
        LODA,R0 RUNFLG
        BCFR,EQ EA_PLAIN                  ; running: stored text is tokenized, no raw words
        BSTA,UN WORD_START                ; direct mode: two letters in a row = maybe a word
        BCFR,EQ EA_PLAIN
        LODI,R0 <FUNC_TAB
        LODI,R1 >FUNC_TAB
        BSTA,UN MATCH_ROW                 ; [+1] R1 = token, or 0
        BRNR,R1 FUNC_BODY                 ; a function word (IP is past it)
EA_PLAIN:
        BSTA,UN PARSE_FACTOR              ; plain atom: variable / constant / number
        ZBRR *VPARSER_RET
; =============================================================================
;  ARG_OPEN / ARG_CLOSE -- require and consume '(' / ')' around a function argument.
;  The argument is parsed by EXPR_AM, which stops at the ')'.  Leaf helpers for
;  FUNC_BODY / FB_A.
; In:  IP -> optional blanks, then the bracket
; Out: bracket consumed; error JSYNERR if it is not there
; Clobbers: R0, R1.  RAS: +1 (WSKIP +1 more, INC_IP is tail-called)
ARG_OPEN:
        LODI,R1 A'('
        db $EC                          ; COMA,R0 swallows ARG_CLOSE's LODI,R1
ARG_CLOSE:
        LODI,R1 A')'
        ZBSR *VWSKIP                    ; R0 = next non-blank (WSKIP leaves R1 alone)
        COMZ,R1
        BCFA,EQ JSYNERR
        ZBRR *VINC_IP                   ; consume the bracket (tail call)
; EA_TOKEN -- function token at IP: consume it (R1 = token) and fall into FUNC_BODY
EA_TOKEN:
        ADDI,R0 TOK_FUNC_BASE             ; R0 = the token again
        STRZ,R1
        ZBSR *VINC_IP                     ; consume it (R1 survives)
        ; drop through
; =============================================================================
;  FUNC_BODY -- a function call, evaluated on the SW stack (no HW RAS).
;  Frame, bottom to top:  [token] [a, 2 bytes: 2-arg only] [continuation: FB_A or FB_B]
;  The token is the frame's identity: its class bits say how many arguments to parse
;  (FB_A tests them) and its low 5 bits index VEC at the end (FB_CALL).  Control moves
;  only by jumps through SWRETURN, so nothing is left on the HW RAS between the pieces.
;     FUNC_BODY   push the token; '(' ; push FB_A ; parse the first argument (EXPR_AM)
;     FB_A        1 arg: ')' -> FB_CALL.   2 args: ',' ; push a ; push FB_B ; parse b
;     FB_B        optional ')' ; TMP = a ; fall into FB_CALL
;     FB_CALL     pop the token, call VEC[token] (EXP = argument or b, TMP = a), then
;                 return the result like any atom (PARSER_RET)
;  The handlers are plain subroutines (RETC).  R2 (an assignment's variable offset) and
;  R3 (the SW stack pointer) are saved around the call in R2SAVE/R3SAVE, so a handler may
;  clobber them -- but must not use TL_MULADD, PARSE_U16 or MATCH_ROW: they share R3SAVE
;  and TK (R2SAVE overlays TKH).  None does.
;  POKE and LIST run through the same code: DO_POKE/DO_LIST start a frame at the bottom
;  of an empty stack with the internal tokens TOK_POKEOP/TOK_LISTOP, enter at FB_PARSE
;  (no '('), and the final PARSER_RET sees R3=$FF and RETCs straight to the dispatcher.
; In:  R1 = token (FUNC_BODY); IP past the token/word; R3 = SW stack pointer
; Out: EXPH:EXPL = result; returns via PARSER_RET.  Errors: JSYNERR (bracket/comma),
;      ERR_NEST (PUSH_RET).
; Clobbers: R0, R1, TMP, EXP (+ the handler's), SW stack; R2/R3 preserved
FUNC_BODY:
        LODZ,R1
        STRA,R0 SWBASE,R3+                ; push the token
        COMI,R1 TOK_ARG1_BASE
        BCTR,LT FB_CALL                   ; class 101xxxxx: no arguments at all
        BSTR,UN ARG_OPEN                  ; '(' required
FB_PARSE:
        LODI,R0 >FB_A
        LODI,R1 <FB_A
        ZBSR *VPUSH_RET
        BCTA,UN EXPR_AM                   ; EXP = first argument, then SWRETURN -> FB_A
FB_A:
        LODA,R0 SWBASE,R3                 ; the frame's token
        COMI,R0 TOK_ARG2_BASE
        BCFR,LT FB_2ARG                   ; class 111xxxxx: "a,b"
        BSTR,UN ARG_CLOSE                 ; one argument: ')' required
        BCTR,UN FB_CALL
FB_2ARG:
        ZBSR *VWSKIP
        COMI,R0 A','
        BCFA,EQ JSYNERR
        BSTA,UN PUSH_EXP                  ; push a (EXP); consumes the ','
        LODI,R0 >FB_B
        LODI,R1 <FB_B
        ZBSR *VPUSH_RET
        BCTR,UN EXPR_AM                   ; EXP = b, then SWRETURN -> FB_B
FB_B:
        ZBSR *VWSKIP
        COMI,R0 A')'
        BCFR,EQ FB_B1                     ; ')' is optional here (AND/OR/XOR, POKE, LIST)
        ZBSR *VINC_IP
FB_B1:
        BSTA,UN POP_SAVE_TO_TMP           ; TMP = a
FB_CALL:
        LODA,R0 SWBASE,R3                 ; the token
        SUBI,R3 1                         ; pop it
        STRA,R3 R3SAVE
        STRA,R2 R2SAVE
        ANDI,R0 $1F                       ; function index 0..$1F ...
        ADDZ,R0                           ; ... * 2 bytes per row
        ADDI,R0 (2*NSTMT)-1               ; VEC[NSTMT+index], pre-increment form
        STRZ,R1
        BSTA,UN SE_HI_LO                  ; [+1] call the handler; its RETC comes back here
        LODA,R2 R2SAVE
        LODA,R3 R3SAVE
        ZBRR *VPARSER_RET

; =============================================================================
;  PARSE_EXPR -- expression evaluator (recursive descent on the SW stack)
; Handles: literals, variables (A-Z), unary +/-, parens, functions, ^ then */% then +/-.
; In:  IPH:IPL -> expression string
; Out: EXPH:EXPL = 16-bit signed result; IP past the expression
; Clobbers: R0, R1, R3 ($FF on exit), TMP, EXP, NEGFLG, SC0, SC1, TK, GOTOH:GOTOL, R3SAVE
;   (SAVE, POWCNT, TEMPRET and R2SAVE overlay these; R2 preserved)
; Errors: ERR_NEST (SW stack full), ERR_SYN, ERR_DIV_ZERO, ERR_OV
; Notes: the only HW-RAS entry into the expression machinery (+1 level whatever the
;   nesting): it empties the SW stack (R3=$FF) and falls into EXPR_AM.  Everything nests on
;   the SW stack and unwinds through PARSER_RET, whose last step (R3=$FF) is the RETC back
;   to the caller.
PARSE_EXPR:
        LODI,R3 $FF                       ; SW stack empty
        ; drop through
;  EXPR_AM -- parse an expression on the live SW stack (parentheses, function arguments)
; In:  IP -> expression; R3 = SW stack pointer, the caller's continuation already pushed
; Out: EXPH:EXPL = value; returns via PARSER_RET to that continuation
EXPR_AM:
        LODI,R0 >EAM0_RET
        LODI,R1 <EAM0_RET
        ZBRR *VEAM_ATOM_C
EAM0_RET:
        ZBSR *VCHECK_POW
        LODI,R0 >EAM_HI0_RET
        LODI,R1 <EAM_HI0_RET
        ZBRR *VEAM_HI_C
EAM_HI0_RET:
; EAM_LO_LOOP handles + - ; EAM_HI handles * / %.  A left operand is saved with PUSH_EXP and
; a continuation with PUSH_RET (both on the SW stack), so a same-precedence operator at a
; deeper level cannot clobber it.
EAM_LO_LOOP:
        LODA,R0 *IPH
        COMI,R0 SP
        BCFR,EQ ELL_NB                    ; no blank: skip the call
        ZBSR *VWSKIP
ELL_NB:
        COMI,R0 A'+'
        BCTR,EQ EAM_PLUS
        COMI,R0 A'-'
        BCTR,EQ EAM_MINUS
        ZBRR *VPARSER_RET
EAM_PLUS:
        BSTR,UN PUSH_EXP
        LODI,R0 >EAM_P_RET
        LODI,R1 <EAM_P_RET
        ZBRR *VEAM_ATOM_C
EAM_P_RET:
        ZBSR *VCHECK_POW
        LODI,R0 >ADD16_SAVE_EXP 
        LODI,R1 <ADD16_SAVE_EXP 
        ZBRR *VEAM_HI_C
EAM_MINUS:
        BSTR,UN PUSH_EXP
        LODI,R0 >EAM_M_RET
        LODI,R1 <EAM_M_RET
        ZBRR *VEAM_ATOM_C
EAM_M_RET:
        ZBSR *VCHECK_POW
        LODI,R0 >EAM_MH_RET
        LODI,R1 <EAM_MH_RET
        ZBRR *VEAM_HI_C
EAM_MH_RET:
        BSTA,UN NEG_EXP_BODY              ; EXP = -EXP
        BCTR,UN ADD16_SAVE_EXP            ; EXP = SAVE + (-EXP) = SAVE - EXP

; =============================================================================
;  PUSH_EXP -- push EXPH:EXPL onto SWBASE (lo,hi) and consume the operator
;  char at IP
; In:  EXPH:EXPL = operand to push; IP -> the operator char ('+','-','*', etc)
; Out: SWBASE top = pushed EXPH:EXPL (lo,hi); IP advanced past the operator
; Clobbers: R0, R3
PUSH_EXP:
        LODA,R0 EXPL
        STRA,R0 SWBASE,R3+
        LODA,R0 EXPH
        STRA,R0 SWBASE,R3+
        ZBRR *VINC_IP

; =============================================================================
;  PUSH_RET -- push a 16-bit literal return address onto SWBASE
; In:  R0 = low byte, R1 = high byte of the return address; R3 = SW SP
; Out: SWBASE[R3-1]=lo, SWBASE[R3]=hi (pushed); R3 += 2; R1 unchanged, R0 clobbered.
;      ERR_NEST (JERRNEST) if R3 has reached SW_LIMIT.
; Clobbers: R0
PUSH_RET:
        STRA,R0 SWBASE,R3+
        LODZ,R1
        STRA,R0 SWBASE,R3+
        COMI,R3 SW_LIMIT                  ; SW stack nearly full?  (the biggest push between
        RETC,LT                           ;   two PUSH_RETs is 2 bytes, SW_SIZE-SW_LIMIT leaves room)
        BCTA,UN JERRNEST

; =============================================================================
;  ADD16_SAVE_EXP -- EXP = left + EXP (16-bit); resumes EAM_LO_LOOP
;  Shared tail for EAM_PLUS and EAM_MINUS (EAM_MINUS negates EXP first).  Adds straight
;  from the SW stack (WC carry chain; WC=0 on entry, so no carry into the low add).
; In:  EXPH:EXPL = right operand; SW stack top = pushed left operand (lo at R3-1, hi at R3)
; Out: EXPH:EXPL = left + EXPH:EXPL; R3 -= 2; tail-jumps into EAM_LO_LOOP; WC=0
; Clobbers: R0, R3, PSL C/CC.  RAS: none.
ADD16_SAVE_EXP:
        LODA,R0 EXPL
        ADDA,R0 SWBASE,R3-                ; lo byte; R3 -> pushed lo
        STRA,R0 EXPL
        PPSL PSW_WC
        LODA,R0 EXPH
        ADDA,R0 SWBASE+1,R3               ; hi byte + carry
        STRA,R0 EXPH
        CPSL PSW_WC
        SUBI,R3 1
        BCTA,UN EAM_LO_LOOP

; =============================================================================
;  ADD_CORE -- shared 16-bit indexed add: EXPH:EXPL += the pair at IPH+R0 (hi byte).
;  Serves TL_MULADD (and thereby PARSE_U16's x10) and DO_NEXT.
; In:  R0 = hi-byte offset of the pair from IPH (TMPH-IPH, TKH-IPH, VARS-IPH+2n); WC=0
; Out: EXPH:EXPL += *(IPH+off):*(IPL+off); WC=0
; Clobbers: R0, R1.  Leaf.
ADD_CORE:
        STRZ,R1
        LODA,R0 EXPL
        ADDA,R0 IPL,R1
        STRA,R0 EXPL
        PPSL PSW_WC                        ; WC=1: the high byte takes the carry
        LODA,R0 EXPH
        ADDA,R0 IPH,R1
        STRA,R0 EXPH
        CPSL PSW_WC
        RETC,UN

; =============================================================================
;  EAM_ATOM_C -- parse one atom; push the continuation only if the atom is not plain.
;  A plain atom (variable A-Z, constant token) in a RUNning program is read inline and
;  control jumps straight to the continuation: no push, no pop.  Typed text and every other
;  atom push the frame and take EAM_ATOM as before.
; In:  R0 = continuation lo, R1 = hi (as PUSH_RET); IP -> atom
; Out: plain: EXP = value, IP past it, continuation entered.  Else: frame pushed, EAM_ATOM.
; Clobbers: R0, R1, TEMPRET.  R2, R3 preserved.  RAS: WSKIP +1 (blanks only), PUSH_RET +1.
EAM_ATOM_C:
        STRA,R0 TEMPRETL                  ; park the continuation
        STRA,R1 TEMPRETH
        LODA,R0 RUNFLG
        BCTR,EQ EAC_SLOW                  ; typed text: may be a word
        LODA,R0 *IPH
        COMI,R0 SP
        BCFR,EQ EAC_NB
        ZBSR *VWSKIP                      ; blanks (rare)
EAC_NB:
        COMI,R0 TOK_DEC
        BCTR,EQ EAC_CONST
        COMI,R0 TOK_HEX
        BCTR,EQ EAC_CONST
        SUBI,R0 A'A'                      ; COM=1: one unsigned test for A-Z
        COMI,R0 A'Z'-A'A'
        BCFR,GT EAC_VAR
EAC_SLOW:
        LODA,R0 TEMPRETL
        LODA,R1 TEMPRETH
        ZBSR *VPUSH_RET
        ZBRR *VEAM_ATOM
EAC_VAR:
        ADDZ,R0                           ; VARS offset = index*2
        STRZ,R1
        LODA,R0 VARS,R1
        STRA,R0 EXPH
        LODA,R0 VARS+1,R1
        STRA,R0 EXPL
        LODA,R0 IPL                       ; IP += 1 inline
        ADDI,R0 1
        STRA,R0 IPL
        BCFR,EQ EAC_GO                    ; no wrap
        LODA,R0 IPH
        ADDI,R0 1
        STRA,R0 IPH
        BCTA,UN *TEMPRETH
EAC_CONST:
        LODI,R1 0                         ; payload at IP+1 (hi), IP+2 (lo)
        LODA,R0 *IPH,R1+
        STRA,R0 EXPH
        LODA,R0 *IPH,R1+
        STRA,R0 EXPL
        LODA,R0 IPL                       ; IP += 3
        ADDI,R0 3
        STRA,R0 IPL
        COMI,R0 3                         ; unsigned: wrapped iff sum < 3
        BCFR,LT EAC_GO
        LODA,R0 IPH
        ADDI,R0 1
        STRA,R0 IPH
EAC_GO:
        BCTA,UN *TEMPRETH

; =============================================================================
;  EAM_HI_C -- enter the * / % level only if one of those operators is next
; In:  R0 = continuation lo, R1 = hi (as PUSH_RET); IP -> text after an atom
; Out: no * / %: R0 = next non-blank char, continuation entered, no frame.
;      else: frame pushed, EAM_HI.
; Clobbers: R0, R1, TEMPRET.  RAS: WSKIP +1 (blanks only), PUSH_RET +1.
EAM_HI_C:
        STRA,R0 TEMPRETL
        STRA,R1 TEMPRETH
        LODA,R0 *IPH
        COMI,R0 SP
        BCFR,EQ EHC_NB
        ZBSR *VWSKIP                      ; blanks (rare)
EHC_NB:
        COMI,R0 A'*'
        BCTR,EQ EHC_PUSH
        COMI,R0 A'/'
        BCTR,EQ EHC_PUSH
        COMI,R0 A'%'
        BCFA,EQ *TEMPRETH                 ; none: straight to the continuation
EHC_PUSH:
        LODA,R0 TEMPRETL
        LODA,R1 TEMPRETH
        ZBSR *VPUSH_RET
        ZBRR *VEAM_HI

; =============================================================================
;  EAM_HI -- * / % level of the expression parser (left to right)
; In:  IP; EXP = left operand
; Out: EXPH:EXPL = value; returns via PARSER_RET
; Clobbers: as PARSE_EXPR
EAM_HI:
        LODA,R0 *IPH
        COMI,R0 SP
        BCFR,EQ EH_NB                    ; no blank: skip the call
        ZBSR *VWSKIP
EH_NB:
        COMI,R0 A'*'
        BCTR,EQ EAM_MUL
        COMI,R0 A'/'
        BCTR,EQ EAM_DIV
        COMI,R0 A'%'
        BCTR,EQ EAM_MOD
        ZBRR *VPARSER_RET
EAM_MUL:
        ZBSR *VPUSH_EXP
        LODI,R0 >MU_AT_RET
        LODI,R1 <MU_AT_RET
        ZBRR *VEAM_ATOM_C
MU_AT_RET:
        ZBSR *VCHECK_POW
        BSTR,UN POP_SAVE_TO_TMP           ; TMPH:TMPL = popped left operand
        BSTA,UN MUL16
        ZBRR *VEAM_HI 
EAM_DIV:
        ZBSR *VPUSH_EXP
        LODI,R0 >DV_AT_RET
        LODI,R1 <DV_AT_RET
        ZBRR *VEAM_ATOM_C
DV_AT_RET:
        ZBSR *VCHECK_POW
        BSTR,UN POP_SAVE_TO_TMP
        BSTA,UN DIV16
        ZBRR *VEAM_HI 
EAM_MOD:
        ZBSR *VPUSH_EXP
        LODI,R0 >MD_AT_RET
        LODI,R1 <MD_AT_RET
        ZBRR *VEAM_ATOM_C
MD_AT_RET:
        ZBSR *VCHECK_POW
        BSTR,UN POP_SAVE_TO_TMP
        BSTA,UN DIV16
        ZBSR *VTMP_TO_EXP ; EXPH:EXPL = TMPH:TMPL (remainder)
        ZBRR *VEAM_HI 

; =============================================================================
;  POP_SAVE_TO_TMP -- pop a 2-byte value pushed on SWBASE into TMPH:TMPL
;  Used by EAM_MUL/EAM_DIV/EAM_MOD, FB_B and CHECK_POW.
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
        
; EAM_NEG -- unary minus: parse the atom, then negate it
EAM_NEG:
        ZBSR *VINC_IP  
        LODI,R0 >NEG_AT_RET
        LODI,R1 <NEG_AT_RET
        ZBRR *VEAM_ATOM_C
NEG_AT_RET:
        BSTA,UN NEG_EXP_BODY
        ZBSR *VCHECK_POW
        BCTR,UN PARSER_RET
; EAM_POS -- unary plus
EAM_POS:
        ZBSR *VINC_IP  
        LODI,R0 >POS_AT_RET
        LODI,R1 <POS_AT_RET
        ZBRR *VEAM_ATOM_C
POS_AT_RET:
        ZBSR *VCHECK_POW
        BCTR,UN PARSER_RET
; EAM_PAREN -- parenthesised expression: '(' expr ')'
EAM_PAREN:
        ZBSR *VINC_IP  
        LODI,R0 >EP_RET
        LODI,R1 <EP_RET
        ZBSR *VPUSH_RET
        BCTA,UN EXPR_AM
EP_RET:
        ZBSR *VWSKIP  
        ZBSR *VINC_IP  
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
        ; drop through
; =============================================================================
;  SWRETURN -- Return via SW stack
; In:  R3 = SW stack pointer; SWBASE[R3] = hi, SWBASE[R3-1] = lo of return addr
; Out: jumps to address popped from SW stack
; Clobbers: R0, TEMPRETH, TEMPRETL
SWRETURN:
        LODA,R0 SWBASE,R3
        STRA,R0 TEMPRETH
        LODA,R0 SWBASE,R3-
        STRA,R0 TEMPRETL
        SUBI,R3 1
        BCTA,UN *TEMPRETH

; =============================================================================
;  PARSE_FACTOR -- Parse a single value (variable or literal)
; A tokenized constant (TOK_DEC/TOK_HEX + 2 payload bytes) is loaded directly; typed
; decimal goes to PARSE_S16 and typed '$'+hex digit to PARSE_HEX.
; In:  IPH:IPL -> first char of factor
; Out: EXPH:EXPL = value
; Clobbers: R0, R1, SC0 (+ TKH, TKL, R3SAVE on the hex path). R2, R3 preserved.
PARSE_FACTOR:
        LODA,R0 *IPH
        COMI,R0 TOK_DEC                  ; tokenized decimal constant?
        BCTR,EQ PF_CONST
        COMI,R0 TOK_HEX                  ; tokenized hex constant?
        BCTR,EQ PF_CONST
        SUBI,R0 A'A'                     ; shift 'A'-'Z' to 0-25 (COM=1: one unsigned test);
        COMI,R0 A'Z'-A'A'                ; PF_LOADVAR uses the shifted value as its index
        BCFR,GT PF_LOADVAR               ; A-Z: R0 = index 0..25
PF_NUM:
        COMI,R0 '$'-A'A'                 ; R0 is still char-'A'
        BCFR,EQ PARSE_S16                ; not hex: decimal (tail: RETC,UN returns to our caller)
        BSTA,UN HEX_PEEK
        BCFA,EQ PARSE_HEX                ; '$' + hex digit: tail, returns to our caller
        ZBRR *VJSYNERR                   ; '$' + non-hex: syntax error
PF_CONST:
        ZBSR *VINC_IP                    ; [+1] past the token byte
        ZBRR *VFETCH16_IP                ; tail: EXP = payload, IP past it; returns to our caller

; =============================================================================
;  PF_LOADVAR -- Load variable value from VARS
; In:  R0 = variable index 0..25 (PARSE_FACTOR's SUBI result); IP -> that char
; Out: EXPH:EXPL = variable value; IP past the letter (INC_IP is tail-called,
;      so CC is INC_IP's)
; Clobbers: R0, R1.  INC_IP uses R0 only, so R1 survives it.
PF_LOADVAR:
        ADDZ,R0                          ; R0 = index*2
        STRZ,R1                          ; R1 = VARS byte offset
        LODA,R0 VARS,R1                  ; hi byte
        STRA,R0 EXPH
        LODA,R0 VARS+1,R1                ; lo byte
        STRA,R0 EXPL
        ZBRR *VINC_IP                    ; tail call: consume the letter, return to caller

; =============================================================================
;  PARSE_S16 -- Parse signed decimal integer
; In:  IPH:IPL -> first char (optional '-' then digits)
; Out: EXPH:EXPL = signed 16-bit value
; Clobbers: R0, R1, NEGFLG (0 = negate), SC0, EXP, TK, R3SAVE (falls into PARSE_U16)
PARSE_S16:
        LODA,R0 *IPH
        EORI,R0 A'-'                     ; R0 = 0 iff a minus sign
        STRA,R0 NEGFLG                   ; NEGFLG polarity: 0 = negate the result (see
                                         ; NEG_EXP), any other value = leave. Written
                                         ; unconditionally, so no separate clear anywhere.
        BCFR,EQ PS16_UN                  ; not '-' (STRA leaves EORI's CC alone)
        ZBSR *VINC_IP  
PS16_UN:
        ; drop through
; =============================================================================
;  PARSE_U16 -- Parse unsigned decimal digits -> EXPH:EXPL
; No separate entry: PARSE_S16 falls into it.  Jumps to JSYNERR if no digit follows.
; In:  IPH:IPL -> first digit char
; Out: EXPH:EXPL = value
; Clobbers: R0, R1, SC0, EXPH, EXPL, TKH, TKL (R3 preserved via R3SAVE in TL_MULADD)
;PARSE_U16:
        ZBSR *VCLR_EXP
        ZBSR *VDIGIT_CHECK
        BCTA,GT *VJSYNERR ; first char not a digit: syntax error
PU16_LP:
        ZBSR *VDIGIT_CHECK
        BCTR,GT PU16_RET ; RETC,GT
PU16_DIG:
        STRA,R0 SC0                      ; R0 = digit value from DIGIT_CHECK
        ZBSR *VINC_IP
        LODI,R0 10
        BSTA,UN TL_MULADD                ; EXP = EXP*10 + SC0 (shared with the tokenizer)
        BCTR,UN PU16_LP

PU16_RET:
        ; drop through
; =============================================================================
;  NEG_EXP -- Negate EXPH:EXPL if NEGFLG says to
;  NEG_EXP_BODY -- Unconditional negate EXPH:EXPL
; In:  EXPH:EXPL = value; NEGFLG = flag (0 = negate, nonzero = leave)
; Out: EXPH:EXPL negated (two's complement) if NEGFLG==0
; Clobbers: R0, R1
NEG_EXP:
        LODA,R0 NEGFLG
        RETC,GT                          ; nonzero (either sign) -> leave; needs both,
        RETC,LT                          ; NEGFLG's non-zero values are arbitrary bytes
                                         ; (PARSE_S16) or $80 (SETUP_MULDIV/ABS_X)
NEG_EXP_BODY:
        LODI,R1 EXPH-IPH                 ; EXPH offset from IPH (= 8); R1 variant for NEG_SHARED
        BCTR,UN NEG_SHARED

; =============================================================================
;  DO_PEEK / DO_USR -- PEEK(a) and USR(a): EXP = byte at address a / the byte returned
;  in R0 by the machine-code routine called at a (R2/R3 are saved around the call by
;  FB_CALL, so the routine may clobber them).
; In:  EXPH:EXPL = a;  Out: EXPH:EXPL = 0..255;  Clobbers: R0 (+ the USR routine's)
DO_PEEK:
        LODA,R0 *EXPH
        BCTR,UN EXPH_Z                   ; clear top byte
DO_USR:
        BSTA,UN *EXPH   
EXPH_Z:        
        STRA,R0 EXPL
        EORZ,R0          ; clear top byte
        STRA,R0 EXPH
        RETC,UN

; =============================================================================
;  DO_NOT -- bitwise NOT(a), one's complement
; In:  EXPH:EXPL = a.  Out: EXPH:EXPL = ~a.  Clobbers: R0, R1.  Leaf.
; Looped as rarely used
DO_NOT:
;        LODI,R1 2               ; 2
NOT_LOOP:        
        LODA,R0 EXPH-1,R1       ; 3
        EORI,R0 $FF             ; 2
        STRA,R0 EXPH-1,R1       ; 3
        BDRR,R1 NOT_LOOP        ; 2 = 12 bytes 
        RETC,UN

; =============================================================================
;  DO_RND -- RND(n): pseudo-random value in [0,n)
; In:  EXPH:EXPL = n
; Out: EXPH:EXPL = |seed| mod |n|
; Clobbers: R0, R1, R2 (saved by FB_CALL), TMP, EXP, NEGFLG, SC0, SC1, RNDSEED, PSL C/WC
; Error: n = 0 -> ERR_DIV_ZERO
; Notes: the LFSR is stepped 8 times per draw so consecutive draws are uncorrelated.
; =============================================================================
DO_RND:
        LODI,R2 8               ; shuffle a full byte's worth of taps
RNDF_MIX:
        BSTA,UN RND_SHUFFLE
        BDRR,R2 RNDF_MIX

        LODI,R0 (IDX_RND*16)+IDX_TMP
        ZBSR *VREG16_TO_REG16   ; TMP = RNDSEED
        BSTA,UN DIV16          ; Divide
        ZBRR *VTMP_TO_EXP ; EXP = remainder 
        
; =============================================================================
;  ABS_EXP / ABS_X -- Absolute value of EXPH:EXPL (ABS_EXP) or of the 16-bit value
;  at IPH+R1 (ABS_X, R1 = hi byte offset, e.g. TMPH-IPH); toggles NEGFLG if it was
;  negative.  Falls into NEG_SHARED.  ABS_EXP is also the ABS() handler.
; In:  ABS_X: R1 = TMPH-IPH or EXPH-IPH.  NEGFLG = current flag ($00 / $80 sentinels)
; Out: value = |value|; NEGFLG toggled ($00<->$80) if it was negative
; Clobbers: R0, R1
ABS_EXP:
        LODI,R1 EXPH-IPH
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
        LODZ R1
        BCTA,UN INC_ET                   ; tail call: adds 1 (INC_ET uses alt bank R1)

; =============================================================================
;  DO_POKE -- POKE addr,byte (expressions allowed for both)
;  DLS_ARG -- LIST start,end (entered from DO_LIST)
;  Both are statement-level function frames: the internal token TOK_POKEOP / TOK_LISTOP
;  goes at the bottom of an empty SW stack and FUNC_BODY's FB_PARSE parses "a,b" (no
;  brackets; a trailing ')' is accepted).  OP_POKE / OP_LIST then run as 2-argument
;  handlers (TMP = a, EXP = b), and the final PARSER_RET (R3=$FF) RETCs to the dispatcher.
; In:  IP -> "a,b"
; Out: per handler
; Clobbers: as PARSE_EXPR (+ OP_LIST's for LIST)
DO_POKE:
        LODI,R0 TOK_POKEOP
        db $EC                          ; COMA swallows DLS_ARG's LODI
DLS_ARG:
        LODI,R0 TOK_LISTOP
        LODI,R3 $FF
        STRA,R0 SWBASE,R3+              ; SWBASE[0] = the frame's token
        BCTA,UN FB_PARSE

; =============================================================================
;  OP_AND / OP_OR / OP_XOR -- bitwise AND(a,b), OR(a,b), XOR(a,b)
; In:  TMPH:TMPL = a, EXPH:EXPL = b  (TMPL must directly follow TMPH)
; Out: EXPH:EXPL = a op b
; Clobbers: R0, R1.  Leaf.
; Looped, low byte first (R1 = 2 -> 1)
OP_AND:
;        LODI,R1 2               ; 2
AND_LOOP:
        LODA,R0 TMPH-1,R1       ; 3
        ANDA,R0 EXPH-1,R1       ; 3
        STRA,R0 EXPH-1,R1       ; 3
        BDRR,R1 AND_LOOP        ; 2
        RETC,UN                 ; 1 = 14
OP_OR:
;        LODI,R1 2
OR_LOOP:
        LODA,R0 TMPH-1,R1
        IORA,R0 EXPH-1,R1
        STRA,R0 EXPH-1,R1
        BDRR,R1 OR_LOOP
        RETC,UN                 ; = 14
OP_XOR:
;        LODI,R1 2
XOR_LOOP:
        LODA,R0 TMPH-1,R1
        EORA,R0 EXPH-1,R1
        STRA,R0 EXPH-1,R1
        BDRR,R1 XOR_LOOP
        RETC,UN                 ; = 14

; =============================================================================
;  OP_POKE -- store the low byte of b at address a
; In:  TMPH:TMPL = a (address), EXPL = b (byte)
; Out: memory[a] = b
; Clobbers: R0
OP_POKE:
        LODA,R0 EXPL
        STRA,R0 *TMPH
        RETC,UN

; =============================================================================
;  OP_LIST -- LIST start,end: range setup, then list from the first line >= start
; In:  TMPH:TMPL = start line, EXPH:EXPL = end line (both already parsed)
; Out: LNUM = start, CUR = end, TMP = first record >= start; continues into DLS_LP
;      (which lists until the end line / program end, then RETCs back to FB_CALL)
; Clobbers: as DO_LIST
OP_LIST:
        LODI,R0 (IDX_TMP*16)+IDX_LNUM
        ZBSR *VREG16_TO_REG16             ; LNUMH:LNUML = TMPH:TMPL (start line)
        LODI,R0 (IDX_EXP*16)+IDX_CUR
        ZBSR *VREG16_TO_REG16             ; CUR = EXP (end line)
        BSTA,UN FIND_INS                  ; TMP = first record >= start (or prog end)
        BCTR,UN DLS_LP                    ; skip DO_LIST's head (it would reset CUR/TMP)

; =============================================================================
;  DO_LIST -- list the stored lines, optionally within a range
; Syntax: LIST  |  LIST start,end
; In:  PROG=program base, PEH:PEL=program end
; Out: matching lines printed
; Clobbers: R0-R3, IP, TMP, EXP, LNUM, CUR, SC0 (+ PARSE_EXPR's for LIST a,b)
; Notes: CURH:CURL holds the end line; $7FFF = no upper bound (full list).
DO_LIST:
        ; Peek first char: MATCH_KW leaves IP at space before args (if any).
        ZBSR *VWSKIP                      ; skip whitespace
        COMI,R0 CR                       ; no args
        BCFA,LT DLS_ARG                  ; digits - get args
DLS_FULL:
        ; Otherwise Full list: set end sentinel $7FFF, IP = program start
        LODI,R0 $7F
        STRA,R0 CURH
        LODI,R0 $FF
        STRA,R0 CURL
        ZBSR *VSET_TMP_PROG
DLS_LP:
        ; Check TMP against program end
        ZBSR *VCMP_TMP_PE ; [+1]
        RETC,GT
        RETC,EQ
        ; Copy TMP -> IP, read line number hi; check against end hi (CURH)
        LODI,R0 (IDX_TMP*16)+IDX_IP
        ZBSR *VREG16_TO_REG16
        ZBSR *VFETCH16_IP                 ; [+2] EXPH:EXPL = line number; IP past the 2-byte header
        ZBSR *VINC_IP                     ; [+1] IP past the length byte too (not printed)
        ; Check line number (EXPH:EXPL) against end (CURH:CURL)
        LODA,R0 EXPH
        SUBA,R0 CURH
        RETC,GT                         ; line hi > end hi: past range
        BCTR,LT DLS_PRNUM                 ; line hi < end hi: in range
        LODA,R0 EXPL
        SUBA,R0 CURL
        RETC,GT                           ; hi equal, line lo > end lo: past range
DLS_PRNUM:
        ZBSR *VPRINT_S16
        ZBSR *VPRT_SPACE
; DLS_BLPX -- print one stored line body, expanding tokens (loops until the NUL, then
;   continues at DLS_NL).  TMP is idle here (rebuilt from IP at DLS_NL).
; In:  IP -> body.  Out: body printed.  Clobbers: R0-R3, EXP
DLS_BLPX:
        LODA,R0 *IPH
        BCTA,EQ DLS_NL                    ; NUL: end of body (free zero test)
        COMI,R0 TOK_STMT_BASE             ; unsigned: a token (>= $80)?
        BCTA,LT DLS_ECHO                  ; < $80: ordinary char
        COMI,R0 TOK_HEX                   ; unsigned: >= $FD (constant token)?
        BCFR,LT DLS_CONST
; $80-$FC: a keyword/function token.  Walk the combined table (KW_TAB..FUNC_TAB, rows
; [text][NUL][token], under 256 bytes) to the row with this token and echo its text.
; In: R0 = token.  TMP = table pointer, R2 = wanted token, R3 = row text index (R2/R3 are
;   free here: FB_CALL saved them for OP_LIST).
        STRZ,R2                           ; R2 = token wanted
        LODI,R0 <KW_TAB
        LODI,R1 >KW_TAB
        BSTA,UN SET_TMP
        LODI,R1 $FF                       ; row index, pre-increment
DTT_ROW:
        LODZ,R1
        STRZ,R3                           ; R3 = index just before this row's text
DTT_SCAN:
        LODA,R0 *TMPH,R1+
        BCFR,EQ DTT_SCAN                  ; on to the NUL
        LODA,R0 *TMPH,R1+                 ; the row's token byte
        COMZ,R2
        BCFR,EQ DTT_ROW                   ; not ours: next row
DTT_ECHO:
        LODA,R0 *TMPH,R3+
        BCTR,EQ DTT_DONE
        ZBSR *VCOUT
        BCTR,UN DTT_ECHO
DTT_DONE:
        ZBSR *VINC_IP                     ; past the 1-byte token in the
        ZBRR *VDLS_BLPX                   ;   ACTUAL body (TMP is the table walk's)
DLS_CONST:
        STRZ,R2                           ; R2 = TOK_HEX or TOK_DEC (from R0)
        ZBSR *VINC_IP                     ; past the token
        ZBSR *VFETCH16_IP                 ; [+2] EXPH:EXPL = payload; IP past it
        COMI,R2 TOK_HEX
        BCTR,EQ DLS_HEXFMT
        ZBSR *VPRINT_S16                  ; TOK_DEC: signed decimal
        ZBRR *VDLS_BLPX
DLS_HEXFMT:
        LODI,R0 '$'
        ZBSR *VCOUT
        LODA,R0 EXPH
        BSTA,UN PRINT_HEX_BYTE
        LODA,R0 EXPL
        BSTA,UN PRINT_HEX_BYTE
        ZBRR *VDLS_BLPX
DLS_ECHO:
        ZBSR *VCOUT
        ZBSR *VINC_IP
        ZBRR *VDLS_BLPX
DLS_NL:
        ZBSR *VINC_IP                     ; skip over the NUL
        BSTA,UN PRT_CRLF                  ; explicit visual CR/LF for the display
        ; Update TMP from IP for next iteration
        LODI,R0 (IDX_IP*16)+IDX_TMP
        ZBSR *VREG16_TO_REG16
        BCTA,UN DLS_LP

; =============================================================================
;  MUL16 -- Signed 16-bit multiply: TMPH:TMPL * EXPH:EXPL -> EXPH:EXPL
; In:  TMPH:TMPL = left operand; EXPH:EXPL = right operand
; Out: EXPH:EXPL = product (low 16 bits, two's-complement wrap); TMP = |one operand|
;      (the operands are swapped if only the right one is >= 256)
; Clobbers: R0, R1, NEGFLG, TMP, PSL C/CC.  RS,WC restored to 0.
; RAS: +2 (SETUP_MULDIV, ABS_X); the loops call nothing.
; Method: MSB-first shift-add (Horner): acc = 0; per multiplier bit, MSB first,
;   acc = 2*acc and acc += |TMP| if the bit is set.  The multiplier is taken a byte at a
;   time from R0 (the high byte only if non-zero); the operand with the smaller high byte
;   is made the multiplier.  acc lives in the ALTERNATE bank (R1 = lo, R2 = hi), so primary
;   R2 (variable offset) and R3 (SW stack pointer) are untouched; alternate R3 is the pass
;   counter.  WC=1 chains carry through the ADDA/RRL pairs.
MUL16:
        BSTA,UN SETUP_MULDIV             ; [+1] NEGFLG, |TMP|, |EXP|
        ; the multiplier should be the operand with fewer significant bytes: if EXP >= 256
        ; and TMP < 256, swap so the multiplier is one byte (half the passes).
        LODA,R0 EXPH
        BCTR,EQ MU_NOSWAP                ; multiplier already < 256
        LODA,R0 TMPH
        BCFR,EQ MU_NOSWAP                ; both >= 256: nothing to gain
        LODA,R1 TMPL
        LODA,R0 EXPL
        STRA,R0 TMPL
        STRA,R1 EXPL                     ; TMPL <-> EXPL
        LODA,R1 EXPH
        STRA,R1 TMPH                     ; TMPH = old EXPH (TMPH was 0, so EXPH becomes 0)
        EORZ,R0
        STRA,R0 EXPH
MU_NOSWAP:
        PPSL PSW_RS + PSW_WC             ; alt bank R1-R3 (acc lo/hi, pass counter), WC=1
        EORZ,R0
        STRZ,R1                          ; acc lo = 0
        STRZ,R2                          ; acc hi = 0
        LODA,R0 EXPH
        BCTR,EQ MU_LO                    ; high byte 0: its 8 leading zero bits are skipped
        BSTR,UN MU_BYTE                  ; [+1] multiplier high byte
MU_LO:
        LODA,R0 EXPL
        BSTR,UN MU_BYTE                  ; [+1] multiplier low byte
        STRA,R1 EXPL                     ; EXP = acc
        STRA,R2 EXPH
        CPSL PSW_RS + PSW_WC             ; back to primary bank, WC=0
; -----------------------------------------------------------------------------
;  MU_DONE -- shared MUL16/DIV16 epilogue: apply result sign
; In:  EXPH:EXPL = |result|; NEGFLG = 0 if result negative; RS=WC=0
; Out: EXPH:EXPL = signed result.  (No NEGFLG cleanup: PARSE_S16 and SETUP_MULDIV
;      both write it unconditionally before NEG_EXP ever reads it.)
; Clobbers: R0, R1
MU_DONE:
        BCTA,UN NEG_EXP

; -----------------------------------------------------------------------------
;  MU_BYTE -- 8 Horner passes over the multiplier byte in R0 (MSB first): acc = 2*acc,
;  and acc += |TMP| when the bit is set.  The multiplier lives in R0 (common to both
;  banks) and is tested with IORZ, so a pass costs 7 instructions (11 with an add).
;  Shifting the multiplier with C != 0 only
;  puts a stray bit in bit 0, which cannot reach the MSB within 8 passes.
; In:  R0 = multiplier byte; alt bank R1:R2 = acc lo:hi; WC=1, RS=1
; Out: acc updated.  Clobbers: R0, alt R3, PSL C/CC.  Leaf.
MU_BYTE:
        LODI,R3 8
MU_LP:
        CPSL $01                         ; C=0
        RRL,R1                           ; acc *= 2
        RRL,R2
        IORZ,R0                          ; CC=LT iff the multiplier's MSB is set
        BCFR,LT MU_NX                    ; bit 0: no add
        CPSL $01
        ADDA,R1 TMPL                     ; acc += |TMP|
        ADDA,R2 TMPH
MU_NX:
        RRL,R0                           ; multiplier <<= 1
        BDRR,R3 MU_LP
        RETC,UN

; =============================================================================
;  SETUP_MULDIV -- Common sign preamble for MUL16 and DIV16
; Sets NEGFLG = $80 (leave sign), then takes |TMP| and |EXP|, toggling NEGFLG
; for each negative operand.  No copies or clears: MUL16/DIV16 work in place.
; In:  TMPH:TMPL = left operand; EXPH:EXPL = right operand
; Out: TMP = |TMP|; EXP = |EXP|; NEGFLG = 0 if the result is to be negated (exactly
;      one operand negative), else $80
; Clobbers: R0, R1, NEGFLG, TMPH, TMPL, EXPH, EXPL
; RAS: costs 1 level, ABS_X 1 more.  ABS_EXP is a tail jump (no extra level).
SETUP_MULDIV:
        LODI,R0 $80
        STRA,R0 NEGFLG                   ; $80 = leave sign; toggled once per negative operand
        LODI,R1 TMPH-IPH
        BSTA,UN ABS_X                    ; [+1] |TMP|
        BCTA,UN ABS_EXP                  ; tail: |EXP|

; -----------------------------------------------------------------------------
;  JERRDIVZER -- divide-by-zero exit (DIV16 only); does not return
; In: none.  Out: ERR_DIV_ZERO raised via DO_ERROR.  Clobbers: R0
JERRDIVZER:
        LODI,R0 ERR_DIV_ZERO
        ZBRR *VDO_ERROR 

; =============================================================================
;  DIV16 -- Signed 16-bit divide: TMPH:TMPL / EXPH:EXPL -> EXPH:EXPL
; The remainder is left in TMPH:TMPL as a magnitude (|dividend| mod |divisor|); only the
; quotient takes the sign.
; Method: restoring shift-subtract.  32-bit chain [rem | EXP] shifted LEFT one bit per pass
;   (EXP holds |dividend|; its top bit enters rem, the vacated bit0 becomes the quotient
;   bit).  rem -= |divisor| (SC); no borrow -> keep and set the quotient bit; borrow -> add
;   back.  rem < |divisor| <= 32768, so 2*rem+1 never overflows 16 bits.  16 passes, or 8
;   when |dividend| < 256.  rem lives in ALTERNATE bank R1(lo)/R2(hi), R3' = pass counter.
;   A power-of-two |divisor| <= 256 takes a shift/mask fast path.
; In:  TMPH:TMPL = dividend; EXPH:EXPL = divisor
; Out: EXPH:EXPL = quotient; TMPH:TMPL = remainder; SC0:SC1 = |divisor| (restoring
;      loop only; the power-of-two fast path leaves SC0 = remainder lo, SC1 untouched)
; Clobbers: R0, R1, NEGFLG, SC0, SC1, TMP, EXP, PSL C/CC.  RS,WC restored to 0.
; Error: divisor=0 -> ERR_DIV_ZERO
; RAS: max +3 (TMP_TO_EXP / REG16_TO_REG16 are leaf)
DIV16:
        LODA,R0 EXPH
        BCFR,EQ DV_NZ                    ; check for zero
        LODA,R0 EXPL
        BCTR,EQ JERRDIVZER

        ; not zero
DV_NZ:
        BSTR,UN SETUP_MULDIV             ; [+1] NEGFLG, TMP=|N|, EXP=|D|
        ; ---- power-of-two divisor fast path ----
        ; |D| = 2^k (1..128): quotient = |N| >> k, remainder = |N| & (|D|-1).  The shifts run
        ; in alternate-bank registers and |D| is shifted down to 1 to count k.  |D| = 256 is a
        ; byte move.  Any other divisor takes the restoring loop below.
        LODA,R0 EXPH
        BCFA,EQ DV_BIG                   ; |D| >= 256
        LODA,R1 EXPL                     ; R1 = |D| (1..255)
        LODZ,R1
        SUBI,R0 1                        ; R0 = |D|-1
        ANDZ,R1                          ; (|D|-1) & |D| == 0  <=>  power of two
        BCFA,EQ DV_SLOW
        LODZ,R1
        SUBI,R0 1
        ANDA,R0 TMPL                     ; remainder = |N| & (|D|-1) (0..127; hi byte is 0)
        STRA,R0 SC0                      ; park it (SC0 is scratch on this path)
        PPSL PSW_RS + PSW_WC             ; alt bank R1-R3, WC=1
        LODA,R1 TMPH                     ; R1' = |N| hi
        LODA,R2 TMPL                     ; R2' = |N| lo
        LODA,R3 EXPL                     ; R3' = |D|
DVP_LP:
        COMI,R3 1
        BCTR,EQ DVP_DONE                 ; |D| shifted down to 1: done
        CPSL $01                         ; C=0: a 0 enters the top bit
        RRR,R1
        RRR,R2                           ; |N| >>= 1 (the carry chains hi -> lo)
        CPSL $01
        RRR,R3                           ; |D| >>= 1
        BCTR,UN DVP_LP
DVP_DONE:
        STRA,R1 EXPH                     ; quotient = |N| >> k
        STRA,R2 EXPL
        CPSL PSW_RS + PSW_WC             ; back to primary bank, WC=0
        LODA,R0 SC0
        STRA,R0 TMPL                     ; remainder (magnitude)
        EORZ,R0
        STRA,R0 TMPH
        BCTA,UN MU_DONE                  ; sign on the quotient
DV_BIG:
        COMI,R0 1
        BCFR,EQ DV_SLOW                  ; EXPH != 1
        LODA,R0 EXPL
        BCFR,EQ DV_SLOW                  ; |D| != 256
        LODA,R0 TMPH                     ; |D| = 256: quotient = |N| hi, remainder = |N| lo
        STRA,R0 EXPL
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 TMPH
        BCTA,UN MU_DONE
DV_SLOW:
        LODI,R0 (IDX_EXP*16)+IDX_SC0
        ZBSR *VREG16_TO_REG16            ; SC0:SC1 = |D|
        ZBSR *VTMP_TO_EXP                ; EXP = |N| (dividend, becomes quotient)
        PPSL PSW_RS + PSW_WC             ; alt bank R1-R3, WC=1
        LODI,R3 16
        EORZ,R0
        STRZ,R1                          ; rem lo = 0
        STRZ,R2                          ; rem hi = 0
        LODA,R0 EXPH                     ; SKIP8: dividend < 256 ?
        BCFR,EQ DV_LP                    ;   no: full 16 passes
        LODA,R0 EXPL                     ;   yes: EXP <<= 8, 8 passes
        STRA,R0 EXPH
        EORZ,R0
        STRA,R0 EXPL
        LODI,R3 8
DV_LP:
        CPSL $01                         ; C=0: bit shifted into EXP bit0
        LODA,R0 EXPL
        RRL,R0
        STRA,R0 EXPL
        LODA,R0 EXPH
        RRL,R0
        STRA,R0 EXPH                     ; C = old dividend top bit
        RRL,R1                           ; rem = rem*2 + C
        RRL,R2
        PPSL $01                         ; C=1: SUB without borrow-in
        SUBA,R1 SC1                      ; rem -= |D|
        SUBA,R2 SC0                      ; C=1 <=> no borrow
        TPSL $01
        BCTR,EQ DV_Q1                    ; no borrow: keep, quotient bit = 1
        CPSL $01
        ADDA,R1 SC1                      ; borrow: restore rem
        ADDA,R2 SC0
        BCTR,UN DV_NX
DV_Q1:
        LODA,R0 EXPL
        IORI,R0 1
        STRA,R0 EXPL
DV_NX:
        BDRR,R3 DV_LP
        STRA,R1 TMPL                     ; remainder -> TMP
        STRA,R2 TMPH
        CPSL PSW_RS + PSW_WC             ; back to primary bank, WC=0
        BCTA,UN MU_DONE                  ; NEG_EXP on the quotient

; =============================================================================
;  PRINT_S16 -- Print signed 16-bit value EXPH:EXPL as decimal
; In:  EXPH:EXPL = signed value
; Out: decimal digits written to COUT; EXP is consumed (0 on exit)
; Clobbers: R0-R3, EXP, SC0 (TMP is NOT clobbered)
PRINT_S16:
        LODA,R0 EXPH             ; get high byte & establish CC
        BCFR,LT PS_DIGITS        ; not negative
        LODI,R0 A'-'
        ZBSR *VCOUT
        BSTA,UN NEG_EXP_BODY     ; Negate, making EXPH:EXPL positive
PS_DIGITS:
        EORZ,R0
        STRZ,R3                  ; R3 = OR of digit counts so far (0 = still leading zeros)
        LODI,R2 5                ; R2 = column counter 5..1; P10 tables are indexed R2-1
DIGIT_LOOP:
        PPSL $09                 ; WC=1, C=1 (no borrow in): 16-bit subtract chains via carry
        LODI,R1 0                ; R1 = digit count
SUB_LOOP:
        LODA,R0 EXPL
        SUBA,R0 P10_LO-1,R2      ; Subtract low byte (C=1 on entry: first pass or no borrow last pass)
        STRA,R0 SC0              ; Save tentatively
        LODA,R0 EXPH
        SUBA,R0 P10_HI-1,R2      ; Subtract high byte with the low byte's borrow
        BCTR,LT BORROW           ; Borrow (C=0): result LT, stop subtracting

        ; No borrow: commit result and subtract again
        STRA,R0 EXPH
        LODA,R0 SC0
        STRA,R0 EXPL
        BIRR,R1 SUB_LOOP         ; digit++ (flags untouched); never wraps to 0, always loops

BORROW:
        CPSL $08                 ; WC=0 from here, also for COUT (its RRR would chain carry)
        LODZ,R1
        IORZ,R3
        STRZ,R3                  ; R3 |= digit count; CC=EQ while every digit so far is zero
        BCFR,EQ PRINT_IT         ; non-zero digit seen: print
        COMI,R2 1                ; final (1s) column?
        BCFR,EQ NEXT_DIG         ; no: skip the leading zero
PRINT_IT:
        LODZ,R1
        ADDI,R0 A'0'
        ZBSR *VCOUT              ; Print the character
NEXT_DIG:
        BDRR,R2 DIGIT_LOOP       ; next column down to the 1s
        RETC,UN                  ; Return to caller

P10_HI:                          ; powers of 10 (1, 10, 100, 1000, 10000), indexed R2-1
        db $00, $00, $00, $03, $27
P10_LO:
        db $01, $0A, $64, $E8, $10

; =============================================================================
;  RDLINE -- Read a line from input into IBUF with echo and backspace
; In:  nothing
; Out: IBUF = NUL-terminated input line (at most 63 characters; extra input is dropped);
;      IPH:IPL = IBUF.  IBUF is the top 64 bytes of LBUF: SWBASE and GCACHE are below it
;      and are not touched.
; Clobbers: R0, R1, R3
; Notes: R3 is the index into IBUF; $FF = empty (",R3+" pre-increments before the access).
;   BS ($08) deletes; any other character below $0E ends the line.
RDLINE:
        LODI,R0 <IBUF
        STRA,R0 IPH
        LODI,R0 >IBUF
        STRA,R0 IPL
        LODI,R3 $FF                      ; R3 = empty-buffer sentinel (pre-inc convention)
RL_LP:
        BSTA,UN PRE_CHIN                     ; [+1] blocking read
        STRZ,R1
        COMI,R1 BS
        BCTR,EQ RL_BS
        COMI,R1 CR+1
        BCTR,LT RL_EOL
        ; buffer full check: room while R3 < 62 (last slot reserved for NUL)
        ; signed compare (COM=0) so that R3=$FF (empty) tests below 62
        CPSL $02
        COMI,R3 62
        PPSL $02
        BCFR,LT RL_LP
        LODZ,R1                          ; R0 = char (indexed store always uses R0)
        STRA,R0 IBUF,R3+                 ; R3++ (pre-inc); IBUF[R3]=char
        ZBSR *VCOUT  
        BCTR,UN RL_LP
RL_BS:
        COMI,R3 $FF
        BCTR,EQ RL_LP                    ; empty: ignore backspace
        SUBI,R3 1
        BSTA,UN PRT_BS
        ZBSR *VPRT_SPACE  
        BSTA,UN PRT_BS
        BCTR,UN RL_LP
RL_EOL:
        EORZ,R0
        STRA,R0 IBUF,R3+                 ; R3++ (pre-inc, one past last char); NUL-terminate
        BCTA,UN PRT_CRLF                ; tail call

; =============================================================================
;  EATWORD -- Consume [A-Z$] chars at IP
; In:  IPH:IPL -> current position
; Out: IP advanced past word
; Clobbers: R0
EATWORD:
        LODA,R0 *IPH
        SUBI,R0 A'A'                     ; shift 'A' down to 0 (COM=1: one unsigned test)
        COMI,R0 A'Z'-A'A'
        BCFR,GT EW_ADV                   ; A-Z: consume
        COMI,R0 A'$'-A'A'                ; '$' in the same shifted frame (wraps mod 256)
        BCFR,EQ WSKIPRET
EW_ADV:
        ZBSR *VINC_IP 
        BCTR,UN EATWORD

; =============================================================================
;  WORD_START -- can the text at IP be a multi-letter keyword/function word?
;  Every table row is 2+ letters, so a single letter (a variable) or anything
;  that does not start with two letters (constant, number, token) can never
;  match: callers use this to skip the table scan.
; In:  IPH:IPL -> char (after WSKIP)
; Out: CC=EQ: two letters A-Z in a row (maybe a word).  CC=GT: no.
; Clobbers: R0 and CC only -- works in register bank 1, so R1-R3 survive.
; RAS: leaf.  Relies on the global COM=1 (unsigned) set in MAIN.
WORD_START:
        PPSL PSW_RS
        LODI,R1 $FF                      ; pre-increment index
WS_LP:
        LODA,R0 *IPH,R1+
        SUBI,R0 A'A'
        COMI,R0 A'Z'-A'A'
        BCTR,GT WS_RET                   ; not a letter: CC=GT
        COMI,R1 1
        BCFR,EQ WS_LP                    ; test the 2nd char too
WS_RET:
        CPSL PSW_RS                      ; CPSL leaves CC alone
        RETC,UN

; =============================================================================
;  WSKIP -- Skip spaces at IP
; In:  IPH:IPL -> current position
; Out: IPH:IPL -> first non-space char; R0 = that char, CC=EQ iff NUL
; Clobbers: R0, CC.  RAS: +1 (INC_IP)
WSKIP_LOOP:
        ZBSR *VINC_IP           ; advance IP
WSKIP:
        LODA,R0 *IPH            ; read char at IP
        COMI,R0 SP              ; space?
        BCTR,EQ WSKIP_LOOP      ; yes: advance and test again
        IORZ,R0                 ; refresh CC (EQ iff NUL)
WSKIPRET:
        RETC,UN                 ; return

; =============================================================================
;  INC_IP -- IPH:IPL += 1   (dedicated fast path: IP is advanced from nearly every loop)
; In:  IP.  Out: IP += 1.  Clobbers: R0, CC.  Leaf (no RAS use).
; Only a wrap of IPL reaches the hi-byte increment (after ADDI, CC is EQ only for 0).
;
;  INC_EXP / INC_TMP -- EXPH:EXPL += 1 / TMPH:TMPL += 1 (shared INC_ET body, run in the
;  alternate register bank)
; In:  EXP / TMP.  Out: incremented.  Clobbers: R0, CC.  Leaf.  R1-R3 are preserved.
; The offsets (EXPH-IPH, TMPH-IPH) are assembly-time expressions, so the order of the
; IPH..EXPL RAM block must be preserved.  No BSTA inside: no extra RAS depth.
INC_IP:
        LODA,R0 IPL
        ADDI,R0 1
        STRA,R0 IPL
        RETC,GT                          ; result 1..127: no wrap
        RETC,LT                          ; result 128..255: no wrap
        LODA,R0 IPH                      ; result 0: carry into the hi byte
        ADDI,R0 1
        STRA,R0 IPH
        RETC,UN

INC_EXP:
        LODI,R0 EXPH-IPH        ; EXP offset from IPH (= 8); assembly-time expression
        db $EC                  ; COMA,R0 -- consume next 2 bytes (skip to INC_ET)
INC_TMP:
        LODI,R0 TMPH-IPH        ; TMP offset from IPH (= 2); assembly-time expression
; Can jump in here with R0 set for offset (INC_ET with R0=0 still works for IP)
INC_ET:
        PPSL PSW_RS                 ; switch to alternate register bank
        STRZ R1                 ; R1 = offset
        LODA,R0 IPL,R1          ; load lo byte
        ADDI,R0 1
        STRA,R0 IPL,R1
        BCFR,EQ ET_RET          ; ADDI result != 0: no carry out of the low byte, done
        LODA,R0 IPH,R1          ; carry: increment hi byte
        ADDI,R0 1
        STRA,R0 IPH,R1
ET_RET:
        CPSL PSW_RS                 ; switch back to primary bank
        RETC,UN

; =============================================================================
;  EXP_TO_LNUM / TMP_TO_EXP -- REG16_TO_REG16 with the packed byte preloaded.
;  Entered by ZBSR/ZBRR *VEXP_TO_LNUM / *VTMP_TO_EXP (2 bytes) instead of
;  LODI,R0 + ZBSR *VREG16_TO_REG16 (4 bytes); worth it only for pairs with 3+ sites.
; In:  none of its own (R0 is loaded here); WC must be 0 as for REG16_TO_REG16
; Out: EXP_TO_LNUM: LNUMH:LNUML = EXPH:EXPL      TMP_TO_EXP: EXPH:EXPL = TMPH:TMPL
;      (source unchanged; CC as REG16_TO_REG16)
; Clobbers: R0 (and CC).  RAS: same as REG16_TO_REG16.
; =============================================================================
EXP_TO_LNUM:
        LODI,R0 (IDX_EXP*16)+IDX_LNUM
        DB $EC                          ; COMA,R0: swallows the next LODI
TMP_TO_EXP:
        LODI,R0 (IDX_TMP*16)+IDX_EXP
        ; drop through
; =============================================================================
;  REG16_TO_REG16 -- generic 16-bit copy between any two IPH-relative RAM
;  register pairs, addressed by a packed nibble pair.
; In:  R0 = (SRC_IDX<<4)|DST_IDX; each idx*2 = byte offset of the pair's hi
;      byte from IPH.  Use the IDX_* EQUs, e.g.
;          LODI,R0 (IDX_EXP*16)+IDX_TMP
;          ZBSR *VREG16_TO_REG16            ; TMP = EXP
;      Valid idx 0-15 (offsets 0-30); pair = hi byte then lo byte.
;      WC (PSL bit 3) must be 0 so RRR/RRL are plain 8-bit rotates.
; Out: destination pair = source pair (source unchanged).
;      CC = flags of the last byte loaded (the source low byte).
; Clobbers: R0 (and CC).  Primary-bank R1/R2/R3 preserved (alternate bank
;      used, restored by CPSL PSW_RS).
; RAS: leaf routine -- no BSTA inside, costs exactly one level at the caller.
; =============================================================================
REG16_TO_REG16:
        PPSL PSW_RS             ; switch to alternate register bank
        STRZ,R2                 ; alt-R2 = R0 (packed byte SSSSDDDD)
        ANDI,R2 $0F             ; isolate dest nibble -> R2 = 0000DDDD
        EORZ,R2                 ; R0 ^= R2 -> SSSS0000 (cancels the low nibble)
        RRR,R0                  ; /2
        RRR,R0                  ; /2
        RRR,R0                  ; /2 -> R0 = source offset (SSSS*2)
        STRZ,R1                 ; alt-R1 = source offset
        RRL,R2                  ; *2 -> alt-R2 = dest offset (DDDD*2)
        LODA,R0 IPH,R1
        STRA,R0 IPH,R2
        LODA,R0 IPL,R1
        STRA,R0 IPL,R2
        CPSL PSW_RS             ; restore primary bank
        RETC,UN

; =============================================================================
;  FETCH16_IP -- fetch the 16-bit word at [IP] (hi byte first) into EXPH:EXPL
;  and advance IP past it.
; In:  IPH:IPL -> hi byte of the word
; Out: EXPH:EXPL = the word; IPH:IPL += 2
; Clobbers: R0, EXPH:EXPL (INC_IP uses R0 only; R1-R3 kept)
; RAS: +2 when ZBSR'd (INC_IP nests inside); +1 when reached by ZBRR tail.
; =============================================================================
FETCH16_IP:
        LODA,R0 *IPH                    ; word hi
        STRA,R0 EXPH
        ZBSR *VINC_IP                   ; [+1]
        LODA,R0 *IPH                    ; word lo
        STRA,R0 EXPL
        ZBRR *VINC_IP                   ; tail: IP past the word, return to caller

; =============================================================================
;  Error exits -- each loads its error code and falls into DO_ERROR (COMA skip chain):
;  JERRNEST ?8 (SW stack full)   JERR_NXT ?7 (NEXT without FOR)   JERRVAR ?4 (not A-Z)
;  JSYNERR ?0 (syntax)   JERROOM ?3 (store full)   JFORERR ?6 (FOR stack full)
;  DRT_UNDERFLOW ?5 (RETURN without GOSUB)
; In:  nothing.  Out: jumps to DO_ERROR (never returns).  Clobbers: R0
JERRNEST:
        LODI,R0 ERR_NEST
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to BCTA
JERR_NXT:
        LODI,R0 ERR_NXT
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to BCTA
JERRVAR:
        LODI,R0 ERR_VAR
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to BCTA
JSYNERR:
        LODI,R0 ERR_SYN
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to BCTA
JERROOM:
        LODI,R0 ERR_OOM
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to BCTA
JFORERR:
        LODI,R0 ERR_FOR
        db $EC                  ; COMA,R0: consume next 2 bytes, skip to BCTA
DRT_UNDERFLOW:
        LODI,R0 ERR_RET
        ; drop through

; =============================================================================
;  DO_ERROR -- Print "?n" (or "?n@line" if running), clear run state, return to REPL
; In:  R0 = error code character ('0'..'9')
; Out: jumps to REPL (which clears the whole hardware RAS); never returns
; Clobbers: all
DO_ERROR:
        STRZ,R1                         ; Save ASCII error code
        BSTR,UN PRT_QUEST               ; Print Question mark
        LODZ,R1
        ZBSR *VCOUT                     ; print error code
        LODA,R0 RUNFLG                  ; running?
        BCTR,EQ DE_NL                   ; not running, no line number
        LODI,R0 '@'                     
        ZBSR *VCOUT                     ; Print at line
        LODI,R0 (IDX_CUR*16)+IDX_EXP
        ZBSR *VREG16_TO_REG16             ; EXPH:EXPL = CURH:CURL
        ZBSR *VPRINT_S16                ; [+1]
DE_NL:
        BSTR,UN PRT_CRLF
        BSTA,UN DO_END                   ; [+1] clears SWSP, FORSP, RUNFLG
        BCTA,UN REPL                     ; REPL resets RAS (PSU SP bits) on entry

; =============================================================================
;  DO_FREE -- print the free program-store bytes: PROGLIM - PEH:PEL
; Syntax: FREE
; In:  PEH:PEL = program end pointer
; Out: decimal count printed, then CR/LF (falls into PRT_CRLF)
; Clobbers: R0-R3, EXP, SC0 (via PRINT_S16)
; Note: the byte-wise subtraction assumes PROGLIM = $1FFF (its low byte $FF never borrows).
; =============================================================================
DO_FREE:
        ; EXPL = $FF - PEL (never borrows)
        LODI,R0 >PROGLIM                 ; Load $FF
        SUBA,R0 PEL 
        STRA,R0 EXPL 

        ; EXPH = $1F - PEH
        LODI,R0 <PROGLIM                 ; Load $1F
        SUBA,R0 PEH 
        STRA,R0 EXPH 

        ; Print the result
        ZBSR *VPRINT_S16                ; Print decimal value
        ; drop through

; =============================================================================
;  PRT_CRLF / PRT_LF / PRT_QUEST / PRT_CR / PRT_SPACE / PRT_BS -- print one character
;  (PRT_CRLF prints CR then LF).  Each entry loads its character and skips the next
;  entry's LODI with the $EC (COMA) byte-skip trick; PRT_BS tail-calls COUT.
; In:  nothing.  Out: character(s) written.  Clobbers: R0 (and COUT's).
PRT_CRLF:
        BSTR,UN PRT_CR                   ; Print CR/LF
        ; drop through
PRT_LF:
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
        db $EC
PRT_BS:
        LODI,R0 BS
        ZBRR *VCOUT 

; =============================================================================
;  DIGIT_CHECK -- Is *IPH a decimal digit '0'-'9'?
; In:  IP
; Out: R0 = char - '0'; CC=GT if not a digit (single unsigned range test, COM=1)
; Clobbers: R0.  Leaf.
DIGIT_CHECK:
        LODA,R0 *IPH
        SUBI,R0 A'0'
        COMI,R0 9
        RETC,UN

; =============================================================================
;  CLR_EXP -- EXPH:EXPL = 0
; In:  nothing.  Out: EXP = 0.  Clobbers: R0.  Leaf.
CLR_EXP:
        EORZ,R0
        STRA,R0 EXPH
        STRA,R0 EXPL
        RETC,UN

; =============================================================================
;  CHECK_POW -- power operator (^) hook, called after each atom
; In:  EXPH:EXPL = just-parsed atom value; IPH:IPL = current parse position
; Out: EXPH:EXPL = base^exponent if '^' found and consumed, else unchanged
; Clobbers: R0, R1, R3, TMP, SAVE (= SC0:SC1), POWCNT (= TK)
; Error: negative exponent -> ERR_OV
; Notes: repeated multiplication (MUL16), once per unit of the exponent.  Chained ^ is
;   left-associative: 2^3^2 = (2^3)^2.  The loop calls only MUL16 and REG16_TO_REG16,
;   which is what lets SAVE and POWCNT overlay SC0:SC1 and TK.
CHECK_POW:
        LODA,R0 *IPH
        COMI,R0 A'^'
        BCTR,EQ CP_HIT
        RETC,UN
CP_HIT:
        ZBSR *VPUSH_EXP           ; push base(EXPH:EXPL) onto SWBASE; tail
                                    ; branch inside also advances IP past '^'
        LODI,R0 >CP_EXP_RET
        LODI,R1 <CP_EXP_RET
        ZBSR *VPUSH_RET
        ZBRR *VEAM_ATOM             ; parse exponent atom -> EXPH:EXPL
CP_EXP_RET:
        BSTA,UN POP_SAVE_TO_TMP            ; [+1] TMP = base (TMP is dead mid-expression); R3 -= 2
        LODI,R0 (IDX_TMP*16)+IDX_SAVE
        ZBSR *VREG16_TO_REG16             ; [+1] SAVE = TMP
        ; SAVEH:SAVEL = base; EXPH:EXPL = exponent; SW stack balanced

        LODA,R0 EXPH               ; negative exponent -> error (undefined
        ANDI,R0 $80                 ; for an integer-only result)
        BCTR,EQ CP_EXP_OK
        LODI,R0 ERR_OV
        ZBRR *VDO_ERROR 
CP_EXP_OK:
        LODA,R1 EXPH                ; exponent == 0 -> result = 1
        LODA,R0 EXPL                 ; (standard convention, incl. 0^0 == 1)
        IORZ R1
        BCFR,EQ CP_POW_LOOP
        EORZ,R0
        STRA,R0 EXPH
        LODI,R0 1
        STRA,R0 EXPL
        BCTR,UN CP_CHAIN

CP_POW_LOOP:
        LODI,R0 (IDX_EXP*16)+IDX_POW
        ZBSR *VREG16_TO_REG16             ; POWCNT = exponent (EXPH:EXPL becomes the result
        EORZ,R0                    ; result accumulator = 1
        STRA,R0 EXPH
        LODI,R0 1
        STRA,R0 EXPL

CP_MUL_LOOP:
        LODI,R0 (IDX_SAVE*16)+IDX_TMP
        ZBSR *VREG16_TO_REG16             ; TMP = base (SAVEH:SAVEL)
        BSTA,UN MUL16

        LODA,R0 POWCNTL                ; exponent counter -= 1 (16-bit)
        SUBI,R0 1
        STRA,R0 POWCNTL
        BCFR,LT CP_CNT_NB
        LODA,R0 POWCNTH
        SUBI,R0 1
        STRA,R0 POWCNTH
CP_CNT_NB:
        LODA,R1 POWCNTH
        LODA,R0 POWCNTL
        IORZ R1
        BCFR,EQ CP_MUL_LOOP
        ; fall through: loop done

CP_CHAIN:
        LODA,R0 *IPH                   ; chained ^ is left-associative:
        COMI,R0 A'^'                    ; (2^3^2) == (2^3)^2, so loop back
        BCTA,EQ CP_HIT                  ; to CP_HIT with the current result
        RETC,UN                          ; as the new base instead of recursing

; =============================================================================
;  TABLES 
BANNER:
        DB CR, LF, "uBASIC 2650 V5.11", CR, LF, "Bytes Free:",NUL

; -- Keyword / function row table.
; Row: [full word text][NUL][token]; the table ends at a lone NUL.  KW_TAB holds the
; statements, FUNC_TAB the functions: separate scan starts for MATCH_ROW (statement /
; expression position), one contiguous table for LIST's detokenizer.
; The token's low 5 bits index VEC (statement k -> row k, function f -> row NSTMT+f); the
; TOK_xxx EQUs are derived from VEC's row labels, so VEC order is the only truth.
; THEN is a token consumed by DO_IF (typed text: raw letters).  TO and STEP are raw letters
; (DO_FOR).  TAB, CHR$ and HEX$ are matched by PRINT.
KW_TAB:
        DB "END",    NUL, TOK_END
        DB "FOR",    NUL, TOK_FOR
        DB "FREE",   NUL, TOK_FREE
        DB "GOSUB",  NUL, TOK_GOSUB
        DB "GOTO",   NUL, TOK_GOTO
        DB "IF",     NUL, TOK_IF
        DB "INPUT",  NUL, TOK_INPUT
        DB "LET",    NUL, TOK_LET
        DB "LIST",   NUL, TOK_LIST
        DB "NEW",    NUL, TOK_NEW
        DB "NEXT",   NUL, TOK_NEXT
        DB "POKE",   NUL, TOK_POKE
        DB "PRINT",  NUL, TOK_PRINT
        DB "REM",    NUL, TOK_REM
        DB "RETURN", NUL, TOK_RETURN
        DB "RUN",    NUL, TOK_RUN
        DB "THEN",   NUL, TOK_THEN
FUNC_TAB:
        DB "ABS",    NUL, TOK_ABS
        DB "NEG",    NUL, TOK_NEG
        DB "NOT",    NUL, TOK_NOT
        DB "PEEK",   NUL, TOK_PEEK
        DB "RND",    NUL, TOK_RND
        DB "USR",    NUL, TOK_USR
        DB "AND",    NUL, TOK_AND
        DB "OR",     NUL, TOK_OR
        DB "XOR",    NUL, TOK_XOR
        DB NUL                          ; terminator (MATCH_ROW stops at the NUL)

; -- Handler vector table: 2 bytes per row (hi, lo), entered by SE_HI_LO with
; R1 = 2*row-1 (pre-increment).  Rows 0..NSTMT-1 = statements in KW_TAB order (token-$80);
; rows NSTMT.. = functions by the flat low-5-bit token index.  A statement handler is
; jumped to by STMT_EXEC; a function handler is called by FB_CALL (EXP = argument or b,
; TMP = a) and returns the result in EXP with RETC.  LET has no handler of its own:
; its row sends the dispatcher on to SE_NOTKW (the assignment code).
VEC:
VS_END:
        DB <DO_END,    >DO_END          ; TOK_END
VS_FOR:
        DB <DO_FOR,    >DO_FOR          ; TOK_FOR
VS_FREE:
        DB <DO_FREE,   >DO_FREE         ; TOK_FREE
VS_GOSUB:
        DB <DO_GOSUB,  >DO_GOSUB        ; TOK_GOSUB
VS_GOTO:
        DB <DO_GOTO,   >DO_GOTO         ; TOK_GOTO
VS_IF:
        DB <DO_IF,     >DO_IF           ; TOK_IF
VS_INPUT:
        DB <DO_INPUT,  >DO_INPUT        ; TOK_INPUT
VS_LET:
        DB <SE_NOTKW,  >SE_NOTKW        ; TOK_LET
VS_LIST:
        DB <DO_LIST,   >DO_LIST         ; TOK_LIST
VS_NEW:
        DB <DO_NEW,    >DO_NEW          ; TOK_NEW
VS_NEXT:
        DB <DO_NEXT,   >DO_NEXT         ; TOK_NEXT
VS_POKE:
        DB <DO_POKE,   >DO_POKE         ; TOK_POKE
VS_PRINT:
        DB <DO_PRINT,  >DO_PRINT        ; TOK_PRINT
VS_REM:
        DB <DO_REM,    >DO_REM          ; TOK_REM
VS_RETURN:
        DB <DO_RETURN, >DO_RETURN       ; TOK_RETURN
VS_RUN:
        DB <DO_RUN,    >DO_RUN          ; TOK_RUN
VS_THEN:
        DB <JSYNERR,   >JSYNERR         ; TOK_THEN as a statement: syntax error
VEC_FN:
VF_ABS:
        DB <ABS_EXP,      >ABS_EXP      ; TOK_ABS   $C0  flat index 0
VF_NEG:
        DB <NEG_EXP_BODY, >NEG_EXP_BODY ; TOK_NEG   $C1  1
VF_NOT:
        DB <DO_NOT,       >DO_NOT       ; TOK_NOT   $C2  2
VF_PEEK:
        DB <DO_PEEK,      >DO_PEEK      ; TOK_PEEK  $C3  3
VF_RND:
        DB <DO_RND,       >DO_RND       ; TOK_RND   $C4  4
VF_USR:
        DB <DO_USR,       >DO_USR       ; TOK_USR   $C5  5
VF_AND:
        DB <OP_AND,       >OP_AND       ; TOK_AND   $E6  6
VF_OR:
        DB <OP_OR,        >OP_OR        ; TOK_OR    $E7  7
VF_XOR:
        DB <OP_XOR,       >OP_XOR       ; TOK_XOR   $E8  8
VF_POKEOP:
        DB <OP_POKE,      >OP_POKE      ; TOK_POKEOP $E9 9   (internal)
VF_LISTOP:
        DB <OP_LIST,      >OP_LIST      ; TOK_LISTOP $EA 10  (internal)
VEC_END:
; Build-time capacity checks: RES with a negative count is an assembler error, and these
; expressions are 0 (emit nothing) while there are at most 32 statement / function rows.
        RES (((32-NSTMT)+32)/32)-1
        RES (((32-((VEC_END-VEC_FN)/2))+32)/32)-1
ROMEND: 

;  RAM variables -- sequential RES block 
 
        ORG     4096    ; half a 2650 8kbyte page

; --- Ordered group: 16-bit register pairs at IDX_xx*2 bytes from IPH.
;     Used by INC_ET/NEG_SHARED (offset = XH-IPH, assembler-computed) and
;     by REG16_TO_REG16's packed-nibble scheme (IDX_* EQUs near the top).
;     Every pair is hi byte then lo byte on an EVEN offset with idx <= 15.
;     The IDX_* EQUs MUST match this order: reorder both together (IDX_POW and
;     IDX_SAVE alias IDX_TK / IDX_SC0, see RAM OVERLAYS in the header).
IPH     RES 1       ; interpreter pointer hi       (offset  0, IDX_IP=0)
IPL     RES 1       ; interpreter pointer lo
TMPH    RES 1       ; temp 16-bit hi               (offset  2, IDX_TMP=1)
TMPL    RES 1       ; temp 16-bit lo
; Slot 2 (offset 4-5) holds two single bytes, no pair (no IDX). FORVAR must stay
; immediately before LNUMH: DO_FOR/DO_NEXT copy FORVAR,LNUM,EXP,SWSTK (7 bytes) as-is.
RUNFLG  RES 1       ; $01=running $00=immediate
FORVAR  RES 1       ; FOR loop var's VARS byte offset
LNUMH   RES 1       ; scratch line number hi       (offset  6, IDX_LNUM=3)
LNUML   RES 1       ; scratch line number lo
EXPH    RES 1       ; expression result hi         (offset  8, IDX_EXP=4)
EXPL    RES 1       ; expression result lo
SWSTK   RES 2       ; next-line pointer cache [NLP_H][NLP_L] written by DR_EXEC
                    ;                              (offset 10, IDX_SWSTK=5)
GOTOH   RES 1       ; pending target hi            (offset 12, IDX_GOTO=6)
GOTOL   RES 1       ; pending target lo
CURH    RES 1       ; current line hi  (error reporting)  (offset 14, IDX_CUR=7)
CURL    RES 1       ; current line lo
SC0     RES 1       ; Scratch byte 0               (offset 16, IDX_SC0=8)
SC1     RES 1       ; Scratch byte 1
PEH     DB <SHOWCASE_END       ; Program end pointer hi   (offset 18, IDX_PE=9)
PEL     DB >SHOWCASE_END       ; Program end pointer lo
TKH     RES 1       ; table scan pointer (MATCH_ROW), TL_MULADD scale temp
TKL     RES 1       ; (kept off TMP: TRY_STORE_LINE's insertion point lives there)
                     ;   (offset 20, IDX_TK=10)
POWCNTH EQU TKH      ; CHECK_POW: exponent down-counter (hi); overlays TKH
POWCNTL EQU TKL      ; CHECK_POW: exponent down-counter (lo); overlays TKL
SAVEH   EQU SC0      ; CHECK_POW: base (hi); overlays SC0
SAVEL   EQU SC1      ; CHECK_POW: base (lo); overlays SC1
RNDSEED RES 2       ; 16 bit Random                (offset 22, IDX_RND=11)
VARS    RES 52      ; A-Z variables 2 bytes each.  Kept within 255 bytes of IPH so
                    ;   ADD_CORE can reach VARS[n] as VARS-IPH+2n (DO_NEXT)

; --- Remaining (not nibble-indexed) ---
TEMPRETH EQU GOTOH   ; SW return address hi; overlays GOTOH (both are jump trampolines)
TEMPRETL EQU GOTOL   ; SW return address lo; overlays GOTOL

;  --- Flags & Stuff --- 
R3SAVE  RES 1       ; R3 across TL_MULADD and FB_CALL's handler
R2SAVE  EQU TKH      ; FB_CALL: R2 (an assignment's VARS offset) across the handler; overlays TKH
NEGFLG  RES 1       ; Sign flag
FORSP   RES 1       ; FOR stack pointer (bytes used, 0=empty; frame offsets 0/7/14/21)
SWSP    RES 1       ; GOSUB stack pointer (bytes used, 0=empty)
RELOP   RES 1       ; Relational op bitmask: bit0=LT bit1=EQ bit2=GT (DO_IF)
TSL_BLEN EQU RELOP  ; TRY_STORE_LINE's saved body length (R3 is reused for OPEN_GAP/copy);
                    ;   overlays RELOP: the prompt vs statement execution

;  SW stack -- the expression machinery's operand AND return-address stack
; R3 = index of the top byte ($FF=empty, grows up).  Operands and continuation
; addresses are [lo][hi] pairs; a function call also keeps its token byte here.
; Push: STRA,R0 SWBASE,R3+ (pre-increment; lo first).  Pop: LODA,R0 SWBASE,R3- (hi first).
; PUSH_RET refuses to run past SW_LIMIT (ERR_NEST).
; SWBASE (SW_SIZE bytes) is an overlay on TOKBUF, defined after it.

;  GOSUB stack -- managed by SWSP
GSBASE  RES 16      ; GOSUB stack base: 16 bytes

;  FOR/NEXT stack -- managed by FORSP
FORBASE RES 28      ; FOR stack base: 28 bytes

; Buffers
; LBUF: ONE buffer for everything line-shaped.  Low to high:
;   SWBASE   SW_SIZE (64) bytes   SW stack: expression nesting (live in direct mode and RUN)
;   GCACHE   8 bytes              2 slots x [lnH][lnL][ptrH][ptrL], reset by RUN; checked
;                                 inline at DG_TOP (live for a whole RUN)
;   IBUF     64 bytes             typed / INPUT line, 63 characters + NUL (live in direct
;                                 mode, and in a RUN while INPUT reads)
; TOKBUF is LBUF from byte 0: TOKENIZE_LINE's output (tokenized body, NUL-terminated), which
; TRY_STORE_LINE copies into the program store.  It is live only inside TRY_STORE_LINE at the
; prompt, where nothing else in LBUF is live (the typed line is consumed as it is tokenized).
; Worst case 125 bytes: 62 body chars of (1+1+... = 31 constants x (3-byte token +
; separator) + NUL, which also fits LBUF (136), so no bounds check.
LBUF    RES SW_SIZE+8+64
SWBASE  EQU LBUF             ; SW stack, SW_SIZE bytes
GCACHE  EQU LBUF+SW_SIZE     ; 8 bytes; stale/garbage entries at the prompt are harmless
IBUF    EQU LBUF+SW_SIZE+8   ; 64 bytes
TOKBUF  EQU LBUF             ; tokenizer output, written in place behind the IBUF reader
; Build checks (RES with a negative count is an assembler error; these emit nothing):
;   in place: IBUF must start >= 61 bytes into LBUF (62-char body gains <= 62 bytes + the
;   line number prefix >= 1)
        RES ((((IBUF-LBUF)-61)+128)/128)-1
GC_NEXT RES 1       ; round-robin next-slot write offset: always 0 or 4.  NOT overlaid: it
                    ;   indexes GCACHE, so a garbage value would store outside it

; =============================================================================
;  Pre-loaded SHOWCASE program
;
;  Line format: <lineno_hi> <lineno_lo> <len> <tokenized body> <NUL>
;  Pre-tokenized: the bytes TOKENIZE_LINE stores for typed lines.  <len> = body byte count
;  (NUL not counted; TOK_DEC/TOK_HEX count 3), as ADV_TMP_PAST_REC requires.
;  Generated by tools/tokshowcase.py from showcase.bas, then tools/showcase_addlen.py
;  (do not hand-edit; 'showcase_addlen.py verify' re-checks it).
;  Lines  10-190: feature demos (PRINT, CHR$, arithmetic, comparisons, GOTO loop)
;  Lines 192-218: FOR/NEXT and GOSUB/RETURN demos
;  Lines 220-240: function demos (ABS/NEG/AND/OR/XOR/NOT/PEEK/POKE/RND/LIST; 233 = nested calls)
;  Lines 300-510: Mandelbrot set renderer (C=-144..28 step 4, 44 columns; I=-64..56 step 6)
;  Line  530:     GOSUB subroutine (PRINT "sub"; / RETURN)
;
;  Format: DB hi,lo,len,<items>,$00  -- hi-then-lo matches DR_EXEC record format.
;  Items: TOK_xxx keyword tokens, TOK_DEC/TOK_HEX + hi,lo payload, "text" runs,
;  $22=DQ $3B=semicolon (in-string chars that need escaping). A record may
;  span several DB lines; the trailing comment shows the source line.
; =============================================================================
PROG:
        DB 0,10,27,TOK_REM," uBASIC 2650 - SHOWCASE V5",$00                                   ; 10  REM uBASIC 2650 - SHOWCASE V5
        DB 0,20,33,TOK_PRINT," ",$22,"-- uBASIC 2650 V5 Showcase --",$22,$00                  ; 20  PRINT "-- uBASIC 2650 V5 Showcase --"
        DB 0,30,24,TOK_PRINT," ",$22,"--- PRINT / CHR$ ---",$22,$00                             ; 30  PRINT "--- PRINT / CHR$ ---"
        DB 0,40,31,TOK_PRINT," CHR$(",TOK_DEC,0,65,")",$3B,"CHR$(",TOK_DEC,0,66,")",$3B,"CHR$("  ; 40  PRINT CHR$(65);CHR$(66);CHR$(67)
        DB TOK_DEC,0,67,")",$00
        DB 0,50,22,TOK_PRINT," ",$22,"--- ARITHMETIC ---",$22,$00                               ; 50  PRINT "--- ARITHMETIC ---"
        DB 0,60,51,TOK_PRINT," ",$22,"3+4=",$22,$3B,TOK_DEC,0,3,"+",TOK_DEC,0,4,$3B,$22,"  10-3=",$22  ; 60  PRINT "3+4=";3+4;"  10-3=";10-3;"  6*7=";6*7
        DB $3B,TOK_DEC,0,10,"-",TOK_DEC,0,3,$3B,$22,"  6*7=",$22,$3B,TOK_DEC,0,6,"*",TOK_DEC,0,7,$00
        DB 0,70,35,TOK_PRINT," ",$22,"20/4=",$22,$3B,TOK_DEC,0,20,"/",TOK_DEC,0,4,$3B,$22,"  17%5=",$22  ; 70  PRINT "20/4=";20/4;"  17%5=";17%5
        DB $3B,TOK_DEC,0,17,"%",TOK_DEC,0,5,$00
        DB 0,80,23,TOK_PRINT," ",$22,"--- COMPARISONS ---",$22,$00                              ; 80  PRINT "--- COMPARISONS ---"
        DB 0,90,22,TOK_IF," ",TOK_DEC,0,5,">",TOK_DEC,0,3," ",TOK_THEN," ",TOK_PRINT," ",$22,"5>3 ok",$22,$00  ; 90  IF 5>3 THEN PRINT "5>3 ok"
        DB 0,100,22,TOK_IF," ",TOK_DEC,0,3,"<",TOK_DEC,0,5," ",TOK_THEN," ",TOK_PRINT," ",$22,"3<5 ok",$22,$00  ; 100 IF 3<5 THEN PRINT "3<5 ok"
        DB 0,110,24,TOK_IF," ",TOK_DEC,0,3,">=",TOK_DEC,0,3," ",TOK_THEN," ",TOK_PRINT," ",$22,"3>=3 ok",$22  ; 110 IF 3>=3 THEN PRINT "3>=3 ok"
        DB $00
        DB 0,120,24,TOK_IF," ",TOK_DEC,0,4,"<>",TOK_DEC,0,3," ",TOK_THEN," ",TOK_PRINT," ",$22,"4<>3 ok",$22  ; 120 IF 4<>3 THEN PRINT "4<>3 ok"
        DB $00
        DB 0,130,22,TOK_IF," ",TOK_DEC,0,3,"=",TOK_DEC,0,3," ",TOK_THEN," ",TOK_PRINT," ",$22,"3=3 ok",$22,$00  ; 130 IF 3=3 THEN PRINT "3=3 ok"
        DB 0,140,25,TOK_PRINT," ",$22,"--- LOOP via GOTO ---",$22,$00                           ; 140 PRINT "--- LOOP via GOTO ---"
        DB 0,150,5,"I=",TOK_DEC,0,1,$00                                                         ; 150 I=1
        DB 0,160,15,TOK_IF," I>",TOK_DEC,0,5," ",TOK_THEN," ",TOK_GOTO," ",TOK_DEC,0,190,$00            ; 160 IF I>5 THEN GOTO 190
        DB 0,170,4,TOK_PRINT," I",$3B,$00                                                       ; 170 PRINT I;
        DB 0,180,7,"I=I+",TOK_DEC,0,1,$00                                                       ; 180 I=I+1
        DB 0,185,5,TOK_GOTO," ",TOK_DEC,0,160,$00                                               ; 185 GOTO 160
        DB 0,190,4,TOK_PRINT," ",$22,$22,$00                                                    ; 190 PRINT ""
        DB 0,192,25,TOK_PRINT," ",$22,"--- FOR/NEXT LOOP ---",$22,$00                           ; 192 PRINT "--- FOR/NEXT LOOP ---"
        DB 0,194,14,TOK_FOR," I=",TOK_DEC,0,1," TO ",TOK_DEC,0,5,$00                            ; 194 FOR I=1 TO 5
        DB 0,196,4,TOK_PRINT," I",$3B,$00                                                       ; 196 PRINT I;
        DB 0,198,3,TOK_NEXT," I",$00                                                            ; 198 NEXT I
        DB 0,199,4,TOK_PRINT," ",$22,$22,$00                                                    ; 199 PRINT ""
        DB 0,201,22,TOK_PRINT," ",$22,"--- FOR STEP 2 ---",$22,$00                              ; 201 PRINT "--- FOR STEP 2 ---"
        DB 0,203,23,TOK_FOR," I=",TOK_DEC,0,0," TO ",TOK_DEC,0,10," STEP ",TOK_DEC,0,2,$00      ; 203 FOR I=0 TO 10 STEP 2
        DB 0,205,4,TOK_PRINT," I",$3B,$00                                                       ; 205 PRINT I;
        DB 0,207,3,TOK_NEXT," I",$00                                                            ; 207 NEXT I
        DB 0,208,4,TOK_PRINT," ",$22,$22,$00                                                    ; 208 PRINT ""
        DB 0,210,24,TOK_PRINT," ",$22,"--- GOSUB/RETURN ---",$22,$00                            ; 210 PRINT "--- GOSUB/RETURN ---"
        DB 0,212,5,TOK_GOSUB," ",TOK_DEC,2,18,$00                                               ; 212 GOSUB 530
        DB 0,214,5,TOK_GOSUB," ",TOK_DEC,2,18,$00                                               ; 214 GOSUB 530
        DB 0,216,4,TOK_PRINT," ",$22,$22,$00                                                    ; 216 PRINT ""
        DB 0,220,21,TOK_PRINT," ",$22,"--- FUNCTIONS ---",$22,$00                               ; 220 PRINT "--- FUNCTIONS ---"
        DB 0,222,39,TOK_PRINT," ",$22,"ABS(-7)=",$22,$3B,TOK_ABS,"(-",TOK_DEC,0,7,")",$3B,$22   ; 222 PRINT "ABS(-7)=";ABS(-7);"  NEG(7)=";NEG(7)
        DB "  NEG(7)=",$22,$3B,TOK_NEG,"(",TOK_DEC,0,7,")",$00
        DB 0,224,52,TOK_PRINT," ",$22,"AND(12,10)=",$22,$3B,TOK_AND,"(",TOK_DEC,0,12,",",TOK_DEC,0,10  ; 224 PRINT "AND(12,10)=";AND(12,10);"  OR(12,10)=";OR(12,10)
        DB ")",$3B,$22,"  OR(12,10)=",$22,$3B,TOK_OR,"(",TOK_DEC,0,12,",",TOK_DEC,0,10,")",$00
        DB 0,226,45,TOK_PRINT," ",$22,"XOR(12,10)=",$22,$3B,TOK_XOR,"(",TOK_DEC,0,12,",",TOK_DEC,0,10  ; 226 PRINT "XOR(12,10)=";XOR(12,10);"  NOT(0)=";NOT(0)
        DB ")",$3B,$22,"  NOT(0)=",$22,$3B,TOK_NOT,"(",TOK_DEC,0,0,")",$00
        DB 0,228,9,TOK_POKE," ",TOK_DEC,31,64,",",TOK_DEC,0,42,$00                              ; 228 POKE 8000,42
        DB 0,230,28,TOK_PRINT," ",$22,"PEEK(8000)=",$22,$3B,"HEX$(",TOK_PEEK,"(",TOK_DEC,31,64,"))",$00  ; 230 PRINT "PEEK(8000)=";HEX$(PEEK(8000))
        DB 0,232,41,TOK_PRINT," ",$22,"RND(100)=",$22,$3B,TOK_RND,"(",TOK_DEC,0,100,")",$3B,$22  ; 232 PRINT "RND(100)=";RND(100);"  RND(100)=";RND(100)
        DB "  RND(100)=",$22,$3B,TOK_RND,"(",TOK_DEC,0,100,")",$00
        DB 0,233,45,TOK_PRINT," ",$22,"NESTED: ",$22,$3B,TOK_AND,"(",TOK_DEC,0,7,",",TOK_OR,"(",TOK_DEC,0,8,",",TOK_DEC,0,1,"))"  ; 233 PRINT "NESTED: ";AND(7,OR(8,1));" ";ABS(NEG(-4))
        DB $3B,$22," ",$22,$3B,TOK_ABS,"(",TOK_NEG,"(-",TOK_DEC,0,4,"))",$00
        DB 0,234,4,TOK_PRINT," ",$22,$22,$00                                                    ; 234 PRINT ""
        DB 0,236,22,TOK_PRINT," ",$22,"--- LIST 40,60 ---",$22,$00                              ; 236 PRINT "--- LIST 40,60 ---"
        DB 0,238,9,TOK_LIST," ",TOK_DEC,0,40,",",TOK_DEC,0,60,$00                               ; 238 LIST 40,60
        DB 0,240,5,TOK_GOTO," ",TOK_DEC,1,44,$00                                                ; 240 GOTO 300
        DB 1,44,22,TOK_PRINT," ",$22,"--- MANDELBROT ---",$22,$00                               ; 300 PRINT "--- MANDELBROT ---"
        DB 1,54,6,"I=-",TOK_DEC,0,64,$00                                                        ; 310 I=-64
        DB 1,64,15,TOK_IF," I>",TOK_DEC,0,56," ",TOK_THEN," ",TOK_GOTO," ",TOK_DEC,1,254,$00            ; 320 IF I>56 THEN GOTO 510
        DB 1,74,3,"D=I",$00                                                                     ; 330 D=I
        DB 1,84,6,"C=-",TOK_DEC,0,144,$00                                                       ; 340 C=-144
        DB 1,94,15,TOK_IF," C>",TOK_DEC,0,28," ",TOK_THEN," ",TOK_GOTO," ",TOK_DEC,1,224,$00            ; 350 IF C>28 THEN GOTO 480
        DB 1,104,3,"A=C",$00                                                                    ; 360 A=C
        DB 1,105,3,"B=D",$00                                                                    ; 361 B=D
        DB 1,106,5,"E=",TOK_DEC,0,0,$00                                                         ; 362 E=0
        DB 1,107,5,"N=",TOK_DEC,0,1,$00                                                         ; 363 N=1
        DB 1,114,15,TOK_IF," N>",TOK_DEC,0,16," ",TOK_THEN," ",TOK_GOTO," ",TOK_DEC,1,164,$00           ; 370 IF N>16 THEN GOTO 420
        DB 1,124,15,TOK_IF," E>",TOK_DEC,0,0," ",TOK_THEN," ",TOK_GOTO," ",TOK_DEC,1,154,$00            ; 380 IF E>0 THEN GOTO 410
        DB 1,134,19,"T=A*A/",TOK_DEC,0,64,"-B*B/",TOK_DEC,0,64,"+C",$00                         ; 390 T=A*A/64-B*B/64+C
        DB 1,144,15,"B=",TOK_DEC,0,2,"*A*B/",TOK_DEC,0,64,"+D",$00                              ; 400 B=2*A*B/64+D
        DB 1,145,3,"A=T",$00                                                                    ; 401 A=T
        DB 1,154,37,TOK_IF," A*A/",TOK_DEC,0,64,"+B*B/",TOK_DEC,0,64,">",TOK_DEC,1,0," ",TOK_THEN," ",TOK_IF  ; 410 IF A*A/64+B*B/64>256 THEN IF E=0 THEN E=N
        DB " E=",TOK_DEC,0,0," ",TOK_THEN," E=N",$00
        DB 1,164,7,"N=N+",TOK_DEC,0,1,$00                                                       ; 420 N=N+1
        DB 1,165,16,TOK_IF," N<=",TOK_DEC,0,16," ",TOK_THEN," ",TOK_GOTO," ",TOK_DEC,1,114,$00          ; 421 IF N<=16 THEN GOTO 370
        DB 1,174,24,TOK_IF," E>",TOK_DEC,0,0," ",TOK_THEN," ",TOK_PRINT," CHR$(E+",TOK_DEC,0,32,")",$3B,$00  ; 430 IF E>0 THEN PRINT CHR$(E+32);
        DB 1,184,22,TOK_IF," E=",TOK_DEC,0,0," ",TOK_THEN," ",TOK_PRINT," CHR$(",TOK_DEC,0,32,")",$3B,$00  ; 440 IF E=0 THEN PRINT CHR$(32);
        DB 1,194,7,"C=C+",TOK_DEC,0,4,$00                                                       ; 450 C=C+4
        DB 1,204,5,TOK_GOTO," ",TOK_DEC,1,94,$00                                                ; 460 GOTO 350
        DB 1,224,1,TOK_PRINT,$00                                                                ; 480 PRINT
        DB 1,234,7,"I=I+",TOK_DEC,0,6,$00                                                       ; 490 I=I+6
        DB 1,244,5,TOK_GOTO," ",TOK_DEC,1,64,$00                                                ; 500 GOTO 320
        DB 1,254,1,TOK_END,$00                                                                  ; 510 END
        DB 2,18,8,TOK_PRINT," ",$22,"sub",$22,$3B,$00                                           ; 530 PRINT "sub";
        DB 2,20,1,TOK_RETURN,$00                                                                ; 532 RETURN
SHOWCASE_END:

        END
