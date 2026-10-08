/* ============================================================================
 * asm2650.c  —  Signetics 2650 cross-assembler
 * Version: 1.22
 * Build:   gcc -Wall -O2 -o asm2650 asm2650.c
 *
 * USAGE
 *   asm2650 [options] source.asm [output.hex]
 *     Intel-hex goes to stdout unless output.hex is given. A listing sidecar
 *     <source>.LST is written next to the source (also when there are errors).
 *     Exit status 0 = assembled, 1 = errors (hex/binary withheld).
 *
 *   Output options:
 *     -s                     dump the symbol table to stderr
 *     --binary               write a flat 32768-byte binary image to stdout
 *     -o <file>              write the binary image to <file> (implies --binary)
 *     -r $HHHH-$HHHH         limit binary output to this inclusive range
 *                            (requires --binary or -o)
 *     -NoList                suppress the .LST listing sidecar
 *     -h, --help             show usage and exit
 *
 *   Warning options (see WARNINGS AND HINTS below):
 *     --warn=LIST            enable the named warnings/hints (comma-separated)
 *     --no-warn=LIST         disable the named warnings/hints
 *     --no-warn-inline-label, --no-warn-local-branch, --no-warn-tail-call,
 *     --no-warn-branch-skip  older per-warning switches, same as
 *                            --no-warn=label / rel / tail / skip
 *
 *   Examples:
 *     asm2650 prog.asm prog.hex
 *     asm2650 --warn=advisory prog.asm          (also run the advisory hints)
 *     asm2650 --warn=all --no-warn=dead prog.asm
 *
 * Supported assembler directives:
 *   ORG, EQU, DS, RES, DB, DW, END
 *
 * WARNINGS AND HINTS  (all go to stderr as "WARN line N: ..."; none change the
 * emitted code. Names are used by --warn= / --no-warn=; groups: advisory =
 * condtail,loop,brn,thunk,dead; peephole = the 12 hint names; all = everything.)
 *   Always on (not switchable): register omitted (BXA/BSXA default to R3);
 *     ANDZ,R0 replaced with HALT; STRZ,R0 replaced with NOP; LODZ,R0 replaced
 *     with IORZ,R0.
 *   Older warnings, default on:
 *     label     LABEL: INSTR on one line (colon-terminated labels only)
 *     rel       absolute branch whose target is reachable by relative form
 *     skip      BCTR/BCTA,cc that skips an unconditional branch (use BCF)
 *     tail      unconditional call + RETC,UN: BSTx,UN -> BCTx,UN,
 *               ZBSR -> ZBRR, BSXA -> BXA (the RETC,UN can be dropped)
 *   Peephole hints, default on (reported after pass 2, sorted by line, tagged
 *   "[name]", followed by a "Hints:" summary with the potential byte saving):
 *     clear     LODI,R0 0 / ANDI,R0 0 -> EORZ,R0
 *     test      IORI,R0 0 / ANDI,R0 $FF / EORI,R0 0 -> IORZ,R0
 *     zpage     BCTA,UN / BSTA,UN to the zero page -> ZBRR / ZBSR
 *     next      branch (not call) to the next instruction -> delete
 *     psw       CPSL a / CPSL b (also CPSU, PPSL, PPSU) -> one instruction
 *     retcc     BCTx,cc to a RETC,UN -> RETC,cc
 *     skipbyte  BCTR/BCTA,UN over 1-2 bytes -> DB $E4 / DB $EC (skip byte)
 *   Advisory hints, default off (need knowledge the assembler does not have,
 *   e.g. that CC/carry/R0 are dead, or that a thunk has no outside users):
 *     condtail  conditional call + RETC,UN -> conditional jump (RETC,UN kept)
 *     loop      SUBI/ADDI,Rn 1 + BCFx,EQ -> BDRx/BIRx,Rn
 *     brn       COMI,Rn 0 / LODZ,Rn / IORZ,R0 + BCFx,EQ -> BRNx,Rn
 *     thunk     branch/call to a jump thunk -> branch/call the final target
 *     dead      unlabelled code after an unconditional transfer
 *
 * VERSION HISTORY  (summary; the full change notes follow, newest first)
 *   1.22  New default-on hint "skipbyte": BCTx,UN over 1-2 bytes -> DB $E4 / $EC.
 *   1.21  Peephole hints (clear, test, zpage, next, psw, retcc, condtail, loop,
 *         brn, thunk, dead); --warn= / --no-warn=; label reference counts;
 *         header and help text brought up to date.
 *   1.20  Tail-call warning: unconditional call + RETC,UN (--no-warn-tail-call).
 *   1.19  BUG-ASM-18: indexed absolute addressing with a target other than R0
 *         is now an error.
 *   1.18  BUG-ASM-16 (DB forward reference sized 0 bytes), BUG-ASM-17 (offset
 *         shown by the "relative form" hint was off by one).
 *   1.17  --no-warn-branch-skip; BUG-ASM-14/15 expression parser rewrite.
 *   1.16  BUG-ASM-08..13: indexed-mode detection, silent expression failures,
 *         silent-emission gaps, duplicate labels, buffer truncation, bare '$'.
 *   1.15  Hex output covers only bytes actually emitted.
 *   1.14  BUG-ASM-07: ORG backward over emitted code is an error.
 *   1.13  BUG-ASM-05/06: quoted ';' in literals; DW forward-reference sizing.
 *   1.12  (no change notes recorded)
 *   1.11  BUG-ASM-02/03/04: TMI mask, ZBRR and ZBSR encoding.
 *   1.10  .LST listing sidecar, -NoList.
 *   1.9   DB "string"; previously silent register errors reported.
 *   1.8   Inline-label warning only for colon-terminated labels.
 *   1.7   --no-warn-inline-label/-local-branch, --binary, -o, -r, -h.
 *   1.6   -s symbol table dump.
 *   1.5   RES directive; EORZ checked against the datasheet.
 *   1.4   BUG-ASM-01: LABEL: INSTR on one line.
 *
 * HI/LO OPERATOR CONVENTION (WinArcadia/asm2650.py standard):
 *   <ADDR = HIGH byte  (bits 15:8)   e.g. <$1584 = $15
 *   >ADDR = LOW  byte  (bits  7:0)   e.g. >$1584 = $84
 *
 * BARE '$' TOKEN:
 *   '$' alone (not followed by a hex digit) = address of the current source
 *   line, i.e. the classic assembler "here" token. E.g. "BDRR,R0 $" decrements
 *   R0 and branches back to itself — a busy-wait delay loop timed by R0.
 *
 * EXPRESSION GRAMMAR (eval_expr, used by every operand — DB/DW/EQU/ORG/DS/RES
 * and every instruction's immediate/relative/absolute field):
 *   Primary: $HHHH (hex), %BBBB (binary), decimal, 'c' or A'c' (char literal),
 *            label name, or bare '$' (= this line's start address).
 *   Prefix:  unary '-'; '<' / '>' (HI/LO byte of the REST of the expression).
 *   Binary:  '+' '-' '*' '/', standard precedence, left-to-right; '(' ')'
 *            grouping. Division by zero is an error. Trailing text the
 *            grammar can't parse is a hard error (was silently dropped
 *            pre-1.17 — BUG-ASM-15).
 *
 * Changes v1.21 -> v1.22:
 *   Added hint "skipbyte" (default on; --no-warn=skipbyte to disable): an
 *   unconditional BCTR,UN / BCTA,UN that jumps over exactly 1 or 2 bytes (whole
 *   instructions, at least one of them labelled, i.e. entered from elsewhere)
 *   can be replaced by a one-byte skip opcode that swallows those bytes as its
 *   operand, saving 1 byte (BCTR,UN) or 2 bytes (BCTA,UN):
 *       BCTR,UN SKIP / LODI,R0 4 / SKIP:   ->   DB $EC / LODI,R0 4 / SKIP:
 *       BCTR,UN SKIP / EORZ,R0   / SKIP:   ->   DB $E4 / EORZ,R0   / SKIP:
 *   $EC is COMA,R0 absolute (consumes 2 bytes, reads one memory byte, sets CC);
 *   $E4 is COMI,R0 immediate (consumes 1 byte, sets CC only). All other
 *   registers, PSW bits and memory are unchanged; entering the skipped
 *   instruction directly still executes it normally. Verified in pipbug_wrap.
 *   Reported only when it is safe by construction:
 *     - CC is provably overwritten before it can be read (the instructions at
 *       the skip target are scanned: LOD/ALU/COM/TMI/TPSx set CC, NOP/STR/
 *       rotate/PSU ops are transparent; any conditional or unknown instruction
 *       suppresses the hint), because the skip opcode clobbers CC where the
 *       branch left it alone;
 *     - for the 2-byte form the first skipped byte must have bits 6:5 clear
 *       ($00-$1F, $80-$9F). As the high address byte of COMA,R0 those bits mean
 *       auto-increment/decrement/index with R0 as the index register, which
 *       changes R0 (seen in the simulator for EORI,R0 $24, ANDI,R0 $44,
 *       SUBI,R0 $A4 and ZBSR $BB). Bit 7 only makes the read indirect;
 *     - the skipped bytes are labelled somewhere (otherwise the code is dead
 *       and should simply be deleted - see the optional "dead" hint).
 *   The message shows the address COMA would read, so memory-mapped I/O with
 *   read side effects can be ruled out by eye. Cost: the 2-byte form takes
 *   4 cycles against 3 for BCTR,UN; the 1-byte $E4 form takes 2 (faster).
 *
 * Changes v1.20 -> v1.21:
 *   Added peephole / optimisation hints. Every instruction is recorded in
 *   pass 2 and, if there were no errors, the final machine code is decoded and
 *   checked after pass 2. Hints are warnings only (output bytes are unchanged),
 *   are printed sorted by line as "WARN line N: ... [name]", followed by one
 *   "Hints: name=count ... (about N byte(s) potential saving)" summary line.
 *   Default on:
 *     clear     LODI,R0 0 / ANDI,R0 0 -> EORZ,R0 (1 byte shorter, same cycles/CC)
 *     test      IORI,R0 0 / ANDI,R0 $FF / EORI,R0 0 -> IORZ,R0
 *     zpage     BCTA,UN / BSTA,UN (direct or *) to $0000-$003F / $1FC0-$1FFF that
 *               is not reachable by relative form -> ZBRR / ZBSR
 *     next      branch (not call) to the very next instruction -> delete
 *     psw       CPSL a / CPSL b (same for CPSU, PPSL, PPSU) -> one op with a|b
 *     retcc     BCTR/BCTA,cc to a label whose instruction is RETC,UN -> RETC,cc
 *   Advisory, default off (enable with --warn=name or --warn=advisory):
 *     condtail  conditional call + RETC,UN -> conditional jump, RETC,UN kept
 *               (saves a return-stack level and cycles; no size change)
 *     loop      SUBI,Rn 1 + BCFx,EQ -> BDRx,Rn ; ADDI,Rn 1 + BCFx,EQ -> BIRx,Rn
 *               (BDRx/BIRx leave CC and carry unchanged: only if they are dead)
 *     brn       COMI,Rn 0 / LODZ,Rn / IORZ,R0 + BCFx,EQ -> BRNx,Rn
 *     thunk     branch/call to a jump thunk (ZBRR x; BCTx,UN x; ZBSR/BSTx,UN x +
 *               RETC,UN; chains followed) -> branch/call the final target. Only
 *               reported when the thunk is not entered by fall-through, has no
 *               other (non-branch) label references, and the total byte change
 *               over all its callers minus the dropped thunk is <= 0.
 *     dead      unlabelled instruction right after BCTx,UN / ZBRR / BXA /
 *               RETC,UN / RETE,UN / HALT (jump-table-like runs not reported)
 *   New options: --warn=LIST and --no-warn=LIST, comma-separated names from
 *   clear,test,zpage,next,psw,retcc,condtail,loop,brn,thunk,dead plus the older
 *   label,rel,skip,tail and the groups advisory, peephole, all. The existing
 *   --no-warn-* options still work.
 *   Labels now carry a reference count (used by the thunk analysis).
 *
 * Changes v1.19 -> v1.20:
 *   Added --no-warn-tail-call (on by default): warns when an UNCONDITIONAL
 *     call is immediately followed by an unconditional return, i.e. a tail
 *     call that can be a plain jump:
 *       BSTA,UN addr / RETC,UN   ->  BCTA,UN addr   (also BSTR,UN -> BCTR,UN)
 *       ZBSR (*)addr / RETC,UN   ->  ZBRR (*)addr   (direct or indirect)
 *       BSXA addr,R3 / RETC,UN   ->  BXA  addr,R3
 *     Saves the RETC,UN byte, a return-stack level and its cycles; the jump
 *     form has the same operand and range as the call form, so the suggestion
 *     is always encodable. Message format:
 *       WARN line 120: BSTA,UN FOO followed by RETC,UN (line 121) -- tail call:
 *         suggest BCTA,UN FOO and drop RETC,UN
 *     Detected in assemble_line() (pass 2) from the decoded mnemonic/operands,
 *     so labels without colons, case and spacing are handled, and blank or
 *     comment-only lines between the pair are ignored. Deliberately NOT
 *     warned: conditional calls (BSTx,cc / BSFx), conditional returns
 *     (RETC,EQ/GT/LT), RETE (also re-enables interrupts), any other
 *     instruction or directive between the pair, and any LABEL between the
 *     call and the RETC,UN (label-only line, or a label on the RETC,UN line
 *     itself) since the RETC,UN is then a branch target and must stay.
 *     Warnings only: the emitted .hex/.bin/.LST bytes are unchanged.
 *
 * Changes v1.18 -> v1.19:
 *   BUG-ASM-18 FIXED: register-indexed absolute addressing (,Rn[+/-]) accepted
 *     any target register. On the 2650 the index register occupies the opcode's
 *     register field, so the target is implicitly R0 and only ,R0 is
 *     encodable. The alu[] handler overwrote r with the index register and
 *     dropped the written target, so e.g. LODA,R1 ADDR,R2 assembled to
 *     $0E hh ll, byte-identical to LODA,R0 ADDR,R2, with no diagnostic
 *     (v1.16 BUG-ASM-08 only restricted the mode, not the target). Now a hard
 *     ERROR on pass 2 when an indexed A-mode operand is used with a target
 *     other than R0. Applies to LODA/EORA/ANDA/IORA/ADDA/SUBA/COMA/STRA, with
 *     plain ,Rn and auto ,Rn+ / ,Rn- forms, direct or indirect (*). The error
 *     is unconditional: no command-line option affects it, errors gate the
 *     .hex/.bin output as for every other ERROR. The instruction is still
 *     sized and emitted as before so later addresses stay aligned and no
 *     cascading errors appear. Non-indexed forms (LODA,R1 ADDR) unaffected.
 *
 * Changes v1.17 -> v1.18:
 *   BUG-ASM-16 FIXED: a DB whose operand can't be evaluated yet in pass 1 (a
 *     FORWARD reference to a label defined later in the file, e.g.
 *       PEH  DB <SHOWCASE_END      ; SHOWCASE_END defined further down
 *     ) was counted as ZERO bytes: the !ok branch of the DB handler did
 *     "continue" without advancing pc, whereas the DW handler already did
 *     emit(pc,0);pc++ in the same situation. Every label after such a DB was
 *     therefore assigned an address too low (one byte per affected operand) in
 *     pass 1, pass 2 then emitted the bytes at the wrong place, and the run
 *     still ended "0 error(s)". Minimal repro:
 *       A: DB <B      ; A, C, D all came out $0000 (should be $0000, $0001, $0002)
 *       C: DB >B
 *       D: DB 1
 *          ORG 16
 *       B: DB 5
 *     Fixed by advancing pc (emitting a placeholder 0) in that branch, exactly
 *     as DW does; pass 2 overwrites it with the real value. DW was never affected.
 *   BUG-ASM-17 FIXED: the "can use relative form (offset N)" hint for absolute
 *     branches (BCTA/BCFA/BSTA/BSFA/BRNA/BIRA/BDRA/BSNA...) computed its offset
 *     as target-(pc+1) with pc still at the START of the 3-byte instruction, but
 *     the real relative encoding (emit_rel, called after the opcode byte has been
 *     emitted) uses target-(start+2). The hint was therefore one too high: it
 *     reported a reachable branch at real offset -65 as "-64" (converting it then
 *     failed with "relative offset -65 out of range") and failed to hint a valid
 *     real +63 (hint +64). Fixed by passing pc+1 at all three call sites, so the
 *     reported offset is exactly the one emit_rel would use.
 *   Verified: pBASIC2650 v0.30/v0.31 (which use neither construct) assemble to
 *     byte-identical hex under v1.16, v1.17 and v1.18; the two repros above now
 *     behave correctly.
 *
 * Changes v1.16 -> v1.17:
 *   Added --no-warn-branch-skip (on by default): warns when a BCTR/BCTA
 *     positive-condition branch skips over an immediately-following
 *     unconditional BCTR/BCTA,UN whose fall-through target is the very next
 *     line, e.g.:
 *       BCTR,cc SKIP1     (cc = EQ/GT/LT, not UN)
 *       BCTA,UN FAR
 *     SKIP1:
 *     This can be replaced by a single BCFR/BCFA,cc FAR (same condition,
 *     opposite true/false sense), dropping the unconditional branch. Purely
 *     a textual, line-oriented check (mirrors the user's grep) over three
 *     strictly consecutive raw source lines — does not consult pc/label
 *     values, so it does NOT verify FAR is in relative range for BCFR, and
 *     (like the grep it replaces) will not match if a blank or comment-only
 *     line sits between any of the three lines. A hint, not an auto-fix.
 *
 * Changes v1.16 -> v1.17 (BUG-ASM-14/15 folded into same version per request):
 *   BUG-ASM-14 FIXED: eval_expr()'s '+'/'-' chain recursed on the remainder
 *     for every operator, making it right-associative instead of
 *     left-to-right: "10-3-2" evaluated as 10-(3-2)=9, not (10-3)-2=5; any
 *     expression with a '-' followed by another '+' or '-' was affected
 *     (confirmed empirically: "DB 10-3-2" emitted $09, "DB 10-3+2" emitted
 *     $05). Fixed by rewriting the expression parser as an iterative
 *     precedence-climbing parser (parse_expr/parse_term/parse_factor/
 *     parse_primary) instead of a single self-recursive function; the '+'/'-'
 *     and new '*'/'/' levels both now genuinely iterate left-to-right.
 *   BUG-ASM-15 FIXED: eval_expr() had no unsupported-operator or
 *     trailing-garbage check — anything after a successfully-parsed prefix
 *     was silently dropped with *ok left at 1 and no diagnostic at all, e.g.
 *     "DB $10*2" quietly assembled as a single $10 byte, "0 error(s)". Given
 *     the same rewrite, this was fixed by implementing real '*' and '/'
 *     support (standard precedence over '+'/'-', left-to-right, division by
 *     zero is an error) and real '(' ')' grouping, then having the public
 *     eval_expr() entry point require the operand text be fully consumed —
 *     anything left over is now "ERROR line %d: unexpected 'c' in expression
 *     '...'" on pass 2, not a silent truncation.
 *   Side effect of the rewrite: unary '-' applied to a HI/LO term (e.g.
 *     "-<$1234") previously discarded the '-' silently (the old HI/LO branch
 *     returned before the neg flag was ever applied); now correctly negates.
 *     '<'/'>' (HI/LO of the rest of the expression) keep their documented
 *     v1.16 meaning and are now valid as a factor anywhere, not only at the
 *     very start of an operand — e.g. "2*<FOO" now works instead of silently
 *     truncating to "2" (see BUG-ASM-15).
 *
 * Changes v1.15 -> v1.16:
 *   BUG-ASM-08 FIXED: register-indexed addressing (,Rn[+/-]) detection in the
 *     alu[] family (LOD/EOR/AND/IOR/ADD/SUB/COM/STR) ran unconditionally for
 *     all four modes (Z/I/R/A) instead of only mode A, the only mode that
 *     architecturally supports it. For Z/I/R this silently overwrote the
 *     opcode's register field with the index register and then discarded the
 *     index info entirely (modes 0/1/2 never honoured idxctl). E.g. LODR,R0
 *     MSG_MIN-1,R1+ assembled as a valid-looking LODR with a wrong register
 *     field and no error. Fixed: ,Rn[+/-] is now only accepted when mode==3;
 *     any other mode is a hard error.
 *   BUG-ASM-09 FIXED: eval_expr()'s catch-all failure path (malformed/empty
 *     expression, e.g. "LODI,R0 <" with nothing after the operator) set
 *     *ok=0 with no diagnostic, unlike the undefined-label path which does
 *     report. Every call site that didn't check ok either emitted a garbage
 *     byte (CPSU/CPSL/PPSU/PPSL/TPSU/TPSL, alu[] immediate mode) or a $00
 *     placeholder with no diagnostic (ZBRR/ZBSR, alu[] relative/absolute,
 *     br[]/bra[] branch families, BRNA/BIRA/BDRA/BSNA, BXA/BSXA) — "0
 *     error(s)" while silently emitting wrong code. Fixed: every site now
 *     checks ok and reports "ERROR line %d: bad %s operand '%s'" on pass 2
 *     while still emitting the same placeholder byte(s) it did before (so
 *     addresses stay aligned for further error detection in the same run).
 *   BUG-ASM-10 FIXED: two related silent-emission gaps found while auditing
 *     DS/RES/ORG: (1) "DS -5" / "RES -5" with a negative count fell through
 *     the emit loop's "i<n" test with n<0, silently reserving 0 bytes instead
 *     of erroring, throwing off every later label address with no diagnostic.
 *     (2) "ORG" to a negative or >MAX_ROM address had no bounds check at all;
 *     it only ever surfaced later via emit()'s pass-2-only range check, and
 *     only if something was actually emitted afterward — a bad ORG followed
 *     only by labels or EOF was completely silent. Both now report an error
 *     on pass 2 (DS/RES: "count must be non-negative"; ORG: "address $XXXX
 *     out of range").
 *   BUG-ASM-11 FIXED: label_define() had no duplicate-name detection at all —
 *     redefining an existing label or EQU constant on a different source line
 *     silently overwrote its value with zero diagnostic (confirmed empirically:
 *     two "FOO:" labels or two "BAR EQU" lines resolved to the last one seen,
 *     "0 error(s)"). Added a def_line field to Label; a name redefined on the
 *     SAME source line (EQU legitimately re-evaluating across pass 1/2 as
 *     forward refs resolve) is still allowed silently, but a name defined on
 *     a DIFFERENT line is now "ERROR line %d: '%s' already defined at line %d"
 *     and the original value is kept.
 *   BUG-ASM-12 FIXED: several fixed-size buffers truncated their input with
 *     no diagnostic: label names (32 chars), mnemonics (16 chars), and
 *     operand text (64 chars) — e.g. a 40-char label name would silently
 *     collide with any other name sharing the same first 31 characters.
 *     Buffers enlarged (label names/mnemonics to 64 chars, operands to 128)
 *     and each now reports "ERROR line %d: ... too long" on pass 2 if the
 *     input still doesn't fit, rather than truncating silently.
 *   Version string (ASM2650_VERSION) was stale at "1.13" despite the header
 *     already documenting v1.14/v1.15 changes — corrected, now matches the
 *     header version on every release.
 *
 * Changes v1.15 -> v1.16 (BUG-ASM-13 folded into same version per request):
 *   BUG-ASM-13 FIXED: bare '$' (e.g. "BDRR,R0 $") was never supported as a
 *     "current address" token — eval_expr required a hex digit after '$' and
 *     silently failed otherwise (*ok=0, no diagnostic pre-BUG-ASM-09; a hard
 *     error post-BUG-ASM-09). Confirmed against uBASIC2650_v47.asm: its 110-
 *     baud serial delay routine uses "BDRR,R0 $" x4 as a decrement-and-branch-
 *     to-self busy-wait loop. The OLD assembler (pre-1.16) emitted a literal
 *     $00 displacement byte for the failed expression, which resolves to an
 *     offset of 0 — i.e. branches to the byte immediately after the
 *     instruction, NOT back to itself. Every one of those "delay loops" has
 *     silently never looped; R0 was decremented once and execution fell
 *     straight through. Fixed: '$' with no following hex digit now resolves
 *     to line_start_pc, a new global capturing pc at the start of the current
 *     source line (before anything is emitted for it) — chosen over reading
 *     the live pc directly because different instruction families emit their
 *     opcode byte at different points relative to their eval_expr() call, so
 *     the live pc would make '$' mean different things in different contexts.
 *     line_start_pc gives '$' one consistent meaning everywhere: "the address
 *     this source line started at."
 *
 * Changes v1.14 -> v1.15:
 *   write_hex() previously walked the full [rom_lo, rom_hi] "tide mark" range
 *   in fixed 16-byte records, including any never-emitted bytes within that
 *   span (e.g. the gap between two separately-ORG'd blocks, or RES regions
 *   that were reserved but never written). Those bytes were emitted as
 *   whatever rom[] happened to hold (0xFF from the initial memset). Now uses
 *   rom_emitted[] (added in v1.14) to split output into records that only
 *   cover genuinely emitted bytes, splitting/starting a new record at any
 *   gap. Pure output-size improvement; does not affect emitted byte values
 *   for addresses that were actually written.
 *
 * Changes v1.13 -> v1.14:
 *   BUG-ASM-07 FIXED: ORG moving pc backward over addresses already written
 *     by emit() in the same pass silently overwrote that code with no
 *     diagnostic. Added rom_emitted[MAX_ROM], checked/set in emit() during
 *     pass 2 only; re-emitting an already-emitted address is now reported as
 *     "ERROR line N: addr $XXXX already emitted (ORG moved backward over
 *     existing code?)" (one error per clobbered byte). This is an ERROR, not
 *     a warning, but the .LST sidecar is still written even when errors are
 *     present (moved ahead of the error-gate in main()) since the listing is
 *     often the fastest way to see both the original and overlapping code
 *     side by side; only the .hex/binary output is withheld on error.
 *
 * Changes v1.12 -> v1.13:
 *   BUG-ASM-06 FIXED: DW with an unresolved forward reference emitted 0 bytes
 *     on pass 1 instead of reserving 2 bytes. This caused all subsequent label
 *     addresses to be wrong on pass 1, which in turn made BCTR/BSTR relative
 *     displacements incorrect on pass 2 (they used the pass-1 address of the
 *     target). Fix: emit two $00 placeholder bytes on pass 1 when the DW
 *     operand does not resolve, matching the behaviour of DB and RES.
 *   BUG-ASM-05 FIXED: Semicolons inside quoted literals were treated as comments.
 *     Added quote-aware comment stripping and operand splitting so DB "PRINT ;",
 *     DB strings containing commas/semicolons, and single-quoted character literals
 *     such as LODI,R1 ';' encode correctly while real comments still work.
 *
 * Changes v1.10 -> v1.11:
 *   BUG-ASM-02 FIXED: TMI mask byte was always emitting $00 regardless of operand.
 *     Root cause: mask and register were both in ops[0] space-separated; ops[1] was
 *     empty. Fixed by using ops0_after_reg() to extract mask from after register token.
 *   BUG-ASM-03 FIXED: ZBRR was emitting only 1 byte with no displacement operand.
 *     Fixed: now correctly emits 2 bytes with signed 7-bit zero-page displacement
 *     plus indirect flag in bit 7, per Signetics 2650 User Manual.
 *   BUG-ASM-04 FIXED: ZBSR was applying PC-relative range validation causing false
 *     out-of-range errors. Fixed: validates as -64..+63 zero-page signed displacement;
 *     indirect flag (*) correctly sets bit 7 of displacement byte.
 *
 * Changes v1.9 -> v1.10:
 *   Automatically writes a source listing sidecar (.LST) unless -NoList is used.
 *   Listing output includes addresses, opcode bytes, and a label summary with
 *   unused labels marked.
 *
 * Changes v1.8 -> v1.9:
 *   db "string" supported, reggedize previosuly silent errors
 *
 * Changes v1.7 -> v1.8:
 *   Inline label+instruction warning now applies only to explicit
 *   colon-terminated labels ("LABEL: OPCODE ...").
 *   Non-colon forms like "LABEL EQU 42" no longer emit the warning.
 *
 * Changes v1.6 -> v1.7:
 *   Added warning controls:
 *     --no-warn-inline-label
 *     --no-warn-local-branch
 *   Added output/CLI controls:
 *     --binary, -o <file>, -r $HHHH-$HHHH, -h/--help
 *   Help now prints version and options; binary mode supports full 32K image
 *   output or optional ranged output.
 *
 * Changes v1.5 -> v1.6:
 *   -s flag: dump full symbol table (labels and addresses) to stderr after
 *     assembly. Useful for finding breakpoint addresses for sim2650 -b.
 *
 * Changes v1.4 -> v1.5:
 *   RES directive added as alias for DS (reserve N zero bytes, define label).
 *     Usage: LABEL: RES N  — identical to DS N, suits ROM/RAM split layout.
 *   EORZ Rn confirmed correct against 2650 datasheet (opcode $20+n).
 *
 * Changes v1.3 -> v1.4:
 *   BUG-ASM-01 FIXED: Same-line label+instruction now assembled correctly.
 *     "LABEL: OPCODE operands" previously dropped the instruction silently.
 * ============================================================================ */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

#define MAX_LABELS  512
#define MAX_LINE    256
#define MAX_ROM   32768
#define UNDEF      (-1)
#define ASM2650_VERSION "1.22"

typedef struct { char name[64]; int value; int referenced; int def_line; int refs; } Label;
static Label labels[MAX_LABELS];
static int   nlabels = 0;

typedef struct {
    int lineno;
    int addr;
    int nbytes;
    unsigned char *bytes;
    char *source;
} ListLine;

static ListLine *list_lines = NULL;
static int nlist_lines = 0;
static int list_cap = 0;
static int list_line_addr = -1;
static int list_line_nbytes = 0;
static unsigned char list_line_bytes[MAX_ROM];

static unsigned char rom[MAX_ROM];
static unsigned char rom_emitted[MAX_ROM];  /* tracks which addresses pass 2 has written, for ORG-overwrite detection */
static int rom_lo = MAX_ROM, rom_hi = -1;

static int  pc     = 0;
static int  line_start_pc = 0;  /* BUG-ASM-13: pc captured at the start of the current source
                                  * line, before any bytes are emitted for it — this is what a
                                  * bare '$' token resolves to. Captured once per line rather
                                  * than reading the live pc directly, so '$' means the same
                                  * thing (this statement's address) regardless of how far a
                                  * given instruction family has already advanced pc internally
                                  * before calling eval_expr(). */
static int  pass   = 0;
static int  errors = 0;
static int  lineno = 0;
static int  warn_inline_label = 1;
static int  warn_local_abs_branch = 1;
static int  warn_branch_skip = 1;
static int  warn_tail_call = 1;
/* Tail-call warning state (pass 2 only; see tailcall_check). tc_pending is set
 * by an unconditional call and cleared by anything else, including a label. */
static int  tc_pending = 0;
static int  tc_line = 0;           /* source line of the pending call */
static char tc_call[16], tc_jump[16], tc_opnd[260];
/* Per-line facts exported by assemble_line() for the peephole recorder. */
static char cur_mn[32], cur_op0[128], cur_op1[128];
static int  cur_nops = 0, cur_lbl_defined = 0;
static char *xstrdup(const char *s);
static int  list_enabled = 1;

    /* Upcase the assembler line but preserve content inside single-quoted literals.
     * Handles both 'x' and A'x' (Signetics ASCII) so A'u' stays 0x75 not 0x55. */
static void upcase(char *s)
{
    int quote = 0;   /* 0 = not in quote, otherwise quote char (' or ") */

    for (; *s; s++) {

        if (quote) {
            /* Inside quoted text */
            if (*s == quote)
                quote = 0;

            continue;
        }

        /* Enter quoted text */
        if (*s == '\'' || *s == '"') {
            quote = *s;
            continue;
        }

        /* Normal assembler text */
        *s = (char)toupper((unsigned char)*s);
    
    }
}

static char *skip_ws(char *s){ while(*s==' '||*s=='\t') s++; return s; }

static void emit(int addr, unsigned char b){
    if(addr<0||addr>=MAX_ROM){ if(pass==2){fprintf(stderr,"ERROR line %d: addr $%04X out of range\n",lineno,addr); errors++;} return; }
    if(pass==2){
        if(rom_emitted[addr]){
            fprintf(stderr,"ERROR line %d: addr $%04X already emitted (ORG moved backward over existing code?)\n",lineno,addr);
            errors++;
        }
        rom_emitted[addr]=1;
    }
    rom[addr]=b;
    if(addr<rom_lo) rom_lo=addr;
    if(addr>rom_hi) rom_hi=addr;
    if(pass==2 && list_enabled){
        if(list_line_addr<0) list_line_addr=addr;
        if(list_line_nbytes<MAX_ROM) list_line_bytes[list_line_nbytes++]=b;
    }
}

static int label_find_index(const char *n){
    for(int i=0;i<nlabels;i++) if(strcmp(labels[i].name,n)==0) return i;
    return -1;
}
static int label_find(const char *n){
    int i=label_find_index(n);
    return i>=0 ? labels[i].value : UNDEF;
}
static void label_mark_referenced(const char *n){
    int i=label_find_index(n);
    if(i>=0){ labels[i].referenced=1; labels[i].refs++; }
}
/* label_define: create or update a label/constant's value.
 * Inputs:  n = label name, v = value to assign.
 * Outputs: none (mutates global labels[] / nlabels).
 * Clobbers: labels[], nlabels, errors (via BUG-ASM-11 duplicate check and table-full check).
 * BUG-ASM-11: a name already defined on a DIFFERENT source line is a genuine
 * duplicate (typo/copy-paste) and is now rejected with an error, keeping the
 * original value. A name redefined on the SAME source line (EQU re-evaluated
 * on pass 2, possibly to a different value once forward refs resolve) is the
 * expected two-pass behaviour and is allowed silently. Duplicate errors are
 * only printed on pass==1, since plain-label/ORG-label definitions only ever
 * run on pass 1 — printing on pass==2 as well would double-report EQU-based
 * duplicates (EQU runs on both passes). */
static void label_define(const char *n, int v){
    for(int i=0;i<nlabels;i++) if(strcmp(labels[i].name,n)==0){
        if(labels[i].def_line!=lineno){
            if(pass==1){ fprintf(stderr,"ERROR line %d: '%s' already defined at line %d\n",lineno,n,labels[i].def_line); errors++; }
            return;
        }
        labels[i].value=v; return;
    }
    if(nlabels>=MAX_LABELS){ fprintf(stderr,"ERROR: label table full\n"); errors++; return; }
    strncpy(labels[nlabels].name,n,63); labels[nlabels].name[63]=0;
    labels[nlabels].value=v; labels[nlabels].referenced=0; labels[nlabels].refs=0; labels[nlabels].def_line=lineno; nlabels++;
}

/* ---------------------------------------------------------------------------
 * Expression parser (BUG-ASM-14/15 rewrite, v1.17).
 * Grammar, standard precedence, all left-to-right at each level:
 *   expr   := term (('+' | '-') term)*
 *   term   := factor (('*' | '/') factor)*
 *   factor := '-' factor | ('<'|'>') expr | '(' expr ')' | primary
 *   primary:= $HEX | %BIN | decimal | 'c' | A'c' | label | bare '$'
 * '<'/'>' (HI/LO) keep their pre-1.17 meaning: they consume a full nested
 * expr (not just the next factor), matching the WinArcadia/asm2650.py
 * convention documented in the file header, but are now valid as a factor
 * anywhere (after '*', inside parens, as an operand of unary '-', etc.),
 * not only at the very start of an operand.
 * ------------------------------------------------------------------------- */

static int parse_expr(char **sp, int *ok);

/* parse_primary: $hex / %bin / decimal / 'c' / A'c' / label / bare '$'.
 * Inputs:   *sp = current parse position.
 * Outputs:  *sp advanced past the consumed token on success; *ok=0 on
 *           failure (undefined/too-long label reports its own ERROR line,
 *           same as pre-1.17 — callers still print their own "bad operand"
 *           message on top of that, unchanged double-report convention).
 * Clobbers: errors (via label_find/label name-length check on pass 2). */
static int parse_primary(char **sp, int *ok){
    char *s=*sp; *ok=1; int val=0;
    if(*s=='$'){ s++;
        if(!isxdigit((unsigned char)*s)){
            val = line_start_pc;  /* bare '$' = this line's start address */
        } else {
            while(isxdigit((unsigned char)*s)) val=val*16+(isdigit((unsigned char)*s)?*s-'0':toupper((unsigned char)*s)-'A'+10), s++;
        }
    }
    else if(*s=='%'){ s++; while(*s=='0'||*s=='1') val=val*2+(*s++-'0'); }
    else if(isdigit((unsigned char)*s)){ while(isdigit((unsigned char)*s)) val=val*10+(*s++-'0'); }
    else if(isalpha((unsigned char)*s)||*s=='_'){
        if(toupper((unsigned char)*s)=='A' && *(s+1)=='\''){
            s+=2; val=(unsigned char)*s;
            if(*s) s++;
            if(*s=='\'') s++;
        } else {
            char nm[64]; int i=0;
            while((isalnum((unsigned char)*s)||*s=='_')&&i<63) nm[i++]=*s++;
            nm[i]=0;
            if(isalnum((unsigned char)*s)||*s=='_'){
                if(pass==2){ fprintf(stderr,"ERROR line %d: label name too long (max 63 chars) near '%s'\n",lineno,nm); errors++; }
                *ok=0; *sp=s; return 0;
            }
            int lv=label_find(nm);
            if(lv==UNDEF){ if(pass==2){fprintf(stderr,"ERROR line %d: undefined '%s'\n",lineno,nm); errors++;} *ok=0; *sp=s; return 0; }
            if(pass==2) label_mark_referenced(nm);
            val=lv;
        }
    } else if(*s=='\''){
        s++; val=(unsigned char)*s;
        if(*s) s++;
        if(*s=='\'') s++;
    } else { *ok=0; *sp=s; return 0; }
    *sp=s; return val;
}

/* parse_factor: unary '-' (recurses on itself), '<'/'>' HI/LO of a full
 * nested expr, '(' expr ')', or a primary.
 * Inputs/Outputs/Clobbers: as parse_primary; additionally reports a missing
 * ')' as its own ERROR line on pass 2 (same double-report convention). */
static int parse_factor(char **sp, int *ok){
    char *s=skip_ws(*sp);
    if(*s=='-'){
        s++; s=skip_ws(s);
        int v=parse_factor(&s,ok);
        *sp=s;
        return *ok?-v:0;
    }
    if(*s=='<'||*s=='>'){
        int hi=(*s=='<'); s++;
        int v=parse_expr(&s,ok);
        *sp=s;
        if(!*ok) return 0;
        return hi?((v>>8)&0xFF):(v&0xFF);
    }
    if(*s=='('){
        s++;
        int v=parse_expr(&s,ok);
        if(!*ok){ *sp=s; return 0; }
        s=skip_ws(s);
        if(*s!=')'){
            if(pass==2){ fprintf(stderr,"ERROR line %d: missing ')'\n",lineno); errors++; }
            *ok=0; *sp=s; return 0;
        }
        s++; *sp=s;
        return v;
    }
    int v=parse_primary(&s,ok);
    *sp=s;
    return v;
}

/* parse_term: left-to-right '*'/'/' chain over parse_factor.
 * Clobbers: errors (division by zero, or a failing factor, on pass 2). */
static int parse_term(char **sp, int *ok){
    char *s=*sp;
    int val=parse_factor(&s,ok);
    if(!*ok){ *sp=s; return 0; }
    for(;;){
        char *p=skip_ws(s);
        if(*p=='*'){
            p=skip_ws(p+1);
            int ok2; int rhs=parse_factor(&p,&ok2);
            if(!ok2){ *ok=0; *sp=p; return 0; }
            val*=rhs; s=p;
        } else if(*p=='/'){
            p=skip_ws(p+1);
            int ok2; int rhs=parse_factor(&p,&ok2);
            if(!ok2){ *ok=0; *sp=p; return 0; }
            if(rhs==0){
                if(pass==2){ fprintf(stderr,"ERROR line %d: division by zero\n",lineno); errors++; }
                *ok=0; *sp=p; return 0;
            }
            val/=rhs; s=p;
        } else break;
    }
    *sp=s; return val;
}

/* parse_expr: left-to-right '+'/'-' chain over parse_term. Iterative (not
 * recursive) specifically so chained '-' is left-associative — this is the
 * BUG-ASM-14 fix: pre-1.17, "A-B-C" evaluated as A-(B-C) instead of
 * (A-B)-C because the old eval_expr recursed on the remainder for every
 * '+'/'-' it saw. */
static int parse_expr(char **sp, int *ok){
    char *s=*sp;
    int val=parse_term(&s,ok);
    if(!*ok){ *sp=s; return 0; }
    for(;;){
        char *p=skip_ws(s);
        if(*p=='+'){
            p=skip_ws(p+1);
            int ok2; int rhs=parse_term(&p,&ok2);
            if(!ok2){ *ok=0; *sp=p; return 0; }
            val+=rhs; s=p;
        } else if(*p=='-'){
            p=skip_ws(p+1);
            int ok2; int rhs=parse_term(&p,&ok2);
            if(!ok2){ *ok=0; *sp=p; return 0; }
            val-=rhs; s=p;
        } else break;
    }
    *sp=s; return val;
}

/* eval_expr: public entry point, signature unchanged so none of the ~30
 * call sites in assemble_line need to change. Parses a full expression and
 * now REQUIRES the entire string to be consumed — the BUG-ASM-15 fix.
 * Pre-1.17, trailing text the grammar couldn't parse (e.g. an operator that
 * wasn't implemented, like '*' before this version) was silently dropped
 * with *ok left at 1, so e.g. "DB $10*2" quietly assembled as just $10 with
 * no diagnostic at all. Now it's a hard ERROR, same double-report
 * convention as every other failure path here (this prints the specific
 * "unexpected 'c'" line; the call site's own generic "bad operand" message
 * still follows). Inputs/Outputs/Clobbers: as before this rewrite. */
static int eval_expr(char *s, int *ok){
    char *p=s;
    int val=parse_expr(&p,ok);
    if(!*ok) return 0;
    p=skip_ws(p);
    if(*p){
        if(pass==2){ fprintf(stderr,"ERROR line %d: unexpected '%c' in expression '%s'\n",lineno,*p,s); errors++; }
        *ok=0; return 0;
    }
    return val;
}

/* Strip comments from an assembler line while preserving semicolons inside
 * single-quoted character literals and double-quoted DB strings.  The assembler
 * does not define escape sequences, so a matching quote always closes the
 * current quoted region; only semicolons seen outside quotes start comments. */
static void strip_comment(char *s)
{
    int quote = 0;   /* 0 = not in quote, otherwise quote char (' or ") */

    for (; *s; s++) {
        if (quote) {
            if (*s == quote)
                quote = 0;
            continue;
        }

        if (*s == '\'' || *s == '"') {
            quote = *s;
            continue;
        }

        if (*s == ';') {
            *s = 0;
            return;
        }
    }
}

/* Split operands on commas and stop at semicolon comments, but only when those
 * delimiter characters are outside quoted text.  This keeps DB strings such as
 * "A,B;C" and character literals such as ';' intact for later evaluation. */
static int split_ops(char *s, char ops[][128], int maxops){
    int n=0; s=skip_ws(s);
    while(*s&&n<maxops){
        int i=0;
        int quote=0;
        while(*s&&i<127){
            if(quote){
                ops[n][i++]=*s;
                if(*s==quote) quote=0;
                s++;
                continue;
            }
            if(*s=='\''||*s=='"'){
                quote=*s;
                ops[n][i++]=*s++;
                continue;
            }
            if(*s==','||*s==';') break;
            ops[n][i++]=*s++;
        }
        if(i>=127 && *s && *s!=','&&*s!=';'){
            if(pass==2){ fprintf(stderr,"ERROR line %d: operand too long (max 127 chars)\n",lineno); errors++; }
        }
        ops[n][i]=0;
        for(int j=i-1;j>=0&&(ops[n][j]==' '||ops[n][j]=='\t');j--) ops[n][j]=0;
        n++;
        if(*s==',') s++;
        else if(*s==';') break;
        s=skip_ws(s);
    }
    return n;
}

static int cc_val(const char *s){
    if(strcmp(s,"EQ")==0) return 0;
    if(strcmp(s,"GT")==0) return 1;
    if(strcmp(s,"LT")==0) return 2;
    if(strcmp(s,"UN")==0) return 3;
    return -1;
}

static int reg_val(const char *s){
    if(s[0]=='R'&&s[1]>='0'&&s[1]<='3'&&(s[2]==0||s[2]==' '||s[2]=='\t')) return s[1]-'0';
    return -1;
}

static char *ops0_after_reg(char *s){
    if(s[0]=='R'&&s[1]>='0'&&s[1]<='3'){ char *p=s+2; while(*p==' '||*p=='\t') p++; return p; }
    return s;
}

static void emit_rel(int target, int ind){
    int off=target-(pc+1);
    if(pass==2&&(off<-64||off>63)){ fprintf(stderr,"ERROR line %d: relative offset %d out of range\n",lineno,off); errors++; }
    emit(pc,(unsigned char)((off&0x7F)|(ind?0x80:0))); pc++;
}

static void emit_abs(int addr, int ind, int cc_or_pp){
    unsigned char b1=(unsigned char)(((addr>>8)&0x1F)|((cc_or_pp&3)<<5)|(ind?0x80:0));
    unsigned char b2=(unsigned char)(addr&0xFF);
    emit(pc,b1); pc++; emit(pc,b2); pc++;
}

static int rel_offset_if_possible(int target, int base_pc, int *off_out){
    int off=target-(base_pc+1);
    if(off_out) *off_out=off;
    return (off>=-64 && off<=63);
}

/* tailcall_reset: forget any pending call (start of each pass).
 * Inputs: none.  Outputs: none.  Clobbers: tc_pending. */
static void tailcall_reset(void){ tc_pending=0; }

/* tailcall_label: a label-only line was seen; a label between call and
 * RETC,UN makes the RETC,UN a branch target, so the warning is suppressed.
 * Inputs: none.  Outputs: none.  Clobbers: tc_pending. */
static void tailcall_label(void){ tc_pending=0; }

/* tailcall_check: tail-call detector, called once per line that has a
 * mnemonic (after split_ops, before dispatch). See header v1.19 -> v1.20.
 * Inputs:  mn, ops, nops = mnemonic and operands as parsed by assemble_line;
 *          has_label = nonzero if this line carries a label definition.
 * Outputs: WARN on stderr when mn is RETC,UN directly after a pending
 *          unconditional call and the RETC,UN line has no label.
 * Clobbers: tc_pending, tc_line, tc_call, tc_jump, tc_opnd. No-op in pass 1
 *          or with --no-warn-tail-call. */
static void tailcall_check(const char *mn, char ops[][128], int nops, int has_label){
    if(pass!=2 || !warn_tail_call) return;
    if(tc_pending && !has_label && strcmp(mn,"RETC")==0 && strcmp(ops[0],"UN")==0){
        fprintf(stderr,"WARN line %d: %s %s followed by RETC,UN (line %d) -- tail call: suggest %s %s and drop RETC,UN\n",
                tc_line,tc_call,tc_opnd,lineno,tc_jump,tc_opnd);
        tc_pending=0; return;
    }
    tc_pending=0;
    if(strcmp(mn,"BSTR")==0 || strcmp(mn,"BSTA")==0){
        /* operand layout as PARSE_FIELD: "UN,ADDR" or "UN ADDR" */
        char field[128]; const char *a;
        if(nops>1 && ops[1][0]){ snprintf(field,sizeof(field),"%.127s",ops[0]); a=ops[1]; }
        else {
            const char *q=ops[0]; size_t n;
            while(*q && *q!=' ' && *q!='\t') q++;
            n=(size_t)(q-ops[0]); if(n>=sizeof(field)) n=sizeof(field)-1;
            memcpy(field,ops[0],n); field[n]=0;
            while(*q==' '||*q=='\t') q++;
            a=q;
        }
        if(strcmp(field,"UN")!=0 || !*a) return;
        snprintf(tc_call,sizeof(tc_call),"%.8s,UN",mn);
        snprintf(tc_jump,sizeof(tc_jump),"%s,UN",strcmp(mn,"BSTR")==0?"BCTR":"BCTA");
        snprintf(tc_opnd,sizeof(tc_opnd),"%.127s",a);
    } else if(strcmp(mn,"ZBSR")==0){
        if(!ops[0][0]) return;
        snprintf(tc_call,sizeof(tc_call),"ZBSR");
        snprintf(tc_jump,sizeof(tc_jump),"ZBRR");
        snprintf(tc_opnd,sizeof(tc_opnd),"%.127s",ops[0]);
    } else if(strcmp(mn,"BSXA")==0){
        if(!ops[0][0]) return;
        snprintf(tc_call,sizeof(tc_call),"BSXA");
        snprintf(tc_jump,sizeof(tc_jump),"BXA");
        if(nops>1 && ops[1][0]) snprintf(tc_opnd,sizeof(tc_opnd),"%.127s,%.127s",ops[0],ops[1]);
        else snprintf(tc_opnd,sizeof(tc_opnd),"%.127s",ops[0]);
    } else return;
    tc_pending=1; tc_line=lineno;
}

/* ===========================================================================
 * Peephole / optimisation hints (v1.21)
 *
 * Every instruction assembled in pass 2 is recorded (address, length, source
 * line, "label before it", source operand text). After pass 2 completes
 * (and only if there were no errors) peep_report() decodes the final machine
 * code in rom[] and runs the checks below. Hints are warnings only: the
 * emitted bytes never change. They are collected, sorted by source line and
 * printed together as "WARN line N: ... [name]" followed by a summary line.
 * Names (for --warn= / --no-warn=):
 *   clear     LODI,R0 0 / ANDI,R0 0            -> EORZ,R0              (default on)
 *   test      IORI,R0 0 / ANDI,R0 $FF / EORI,R0 0 -> IORZ,R0           (default on)
 *   zpage     BCTA,UN / BSTA,UN to zero page   -> ZBRR / ZBSR           (default on)
 *   next      branch to the next instruction   -> delete                (default on)
 *   psw       CPSL a + CPSL b (also CPSU/PPSL/PPSU) -> one instruction  (default on)
 *   retcc     BCTx,cc to a RETC,UN             -> RETC,cc               (default on)
 *   condtail  BSxx,cc / RETC,UN  -> BCxx,cc / RETC,UN (tail jump)       (advisory)
 *   loop      SUBI,Rn 1 / BCFx,EQ -> BDRx ; ADDI,Rn 1 / BCFx,EQ -> BIRx (advisory)
 *   brn       COMI,Rn 0 / LODZ,Rn / IORZ,R0 + BCFx,EQ -> BRNx,Rn       (advisory)
 *   thunk     branch/call to a jump thunk with few callers -> go direct (advisory)
 *   dead      unlabelled code after an unconditional transfer           (advisory)
 *   skipbyte  BCTR/BCTA,UN over 1-2 bytes -> DB $E4 / DB $EC skip byte  (default on)
 * ======================================================================== */
enum { PH_CLEAR=1, PH_TEST=2, PH_ZPAGE=4, PH_NEXT=8, PH_PSW=16, PH_RETCC=32,
       PH_CONDTAIL=64, PH_LOOP=128, PH_BRN=256, PH_THUNK=512, PH_DEAD=1024, PH_SKIPBYTE=2048 };
#define PH_DEFAULT  (PH_CLEAR|PH_TEST|PH_ZPAGE|PH_NEXT|PH_PSW|PH_RETCC|PH_SKIPBYTE)
#define PH_ADVISORY (PH_CONDTAIL|PH_LOOP|PH_BRN|PH_THUNK|PH_DEAD)
#define PH_ALL      (PH_DEFAULT|PH_ADVISORY)
static int ph_enabled = PH_DEFAULT;
static const struct { const char *name; int bit; } ph_names[] = {
    {"clear",PH_CLEAR},{"test",PH_TEST},{"zpage",PH_ZPAGE},{"next",PH_NEXT},{"psw",PH_PSW},
    {"retcc",PH_RETCC},{"condtail",PH_CONDTAIL},{"loop",PH_LOOP},{"brn",PH_BRN},
    {"thunk",PH_THUNK},{"dead",PH_DEAD},{"skipbyte",PH_SKIPBYTE},{NULL,0}};
static int ph_count[12];
static int ph_saved = 0;

typedef struct { int addr, len, line, lab_before; char *opnd; } PInst;
static PInst *pins = NULL;
static int npins = 0, pinscap = 0;
static int peep_pend_label = 0;
static int ph_ins_at[MAX_ROM];

typedef struct { int line, seq; char *text; } PWarn;
static PWarn *pw = NULL;
static int npw = 0, pwcap = 0;

typedef struct { const char *name; unsigned char base, call, reg, rel; } BrFam;
static const char *ph_cc[4] = {"EQ","GT","LT","UN"};

/* ph_rb: read a byte of the assembled image, 0xFF if outside it.
 * Inputs: a = address.  Outputs: byte.  Clobbers: none. */
static int ph_rb(int a){ return (a>=0 && a<MAX_ROM) ? rom[a] : 0xFF; }

/* ph_brfam: classify an opcode as a conditional/register branch or call.
 * Inputs: op = opcode.  Outputs: family record (name, call/reg/rel flags) or
 * NULL if op is not one. ZBRR/ZBSR/BXA/BSXA ($9B/$BB/$9F/$BF) are excluded.
 * Clobbers: none. */
static const BrFam *ph_brfam(unsigned char op){
    static const BrFam t[] = {
        {"BCTR",0x18,0,0,1},{"BCTA",0x1C,0,0,0},{"BSTR",0x38,1,0,1},{"BSTA",0x3C,1,0,0},
        {"BCFR",0x98,0,0,1},{"BCFA",0x9C,0,0,0},{"BSFR",0xB8,1,0,1},{"BSFA",0xBC,1,0,0},
        {"BRNR",0x58,0,1,1},{"BRNA",0x5C,0,1,0},{"BSNR",0x78,1,1,1},{"BSNA",0x7C,1,1,0},
        {"BIRR",0xD8,0,1,1},{"BIRA",0xDC,0,1,0},{"BDRR",0xF8,0,1,1},{"BDRA",0xFC,0,1,0},
        {NULL,0,0,0,0}};
    if(op==0x9B||op==0xBB||op==0x9F||op==0xBF) return NULL;
    for(int i=0;t[i].name;i++) if((op&0xFC)==t[i].base) return &t[i];
    return NULL;
}

/* ph_mn: mnemonic text for an opcode (e.g. "BCFR,EQ", "BDRA,R1", "ZBRR").
 * Inputs: b/n = output buffer and size, op = opcode.
 * Outputs: text in b.  Clobbers: b. */
static void ph_mn(char *b, size_t n, unsigned char op){
    const BrFam *f = ph_brfam(op);
    if(f){ if(f->reg) snprintf(b,n,"%s,R%d",f->name,op&3); else snprintf(b,n,"%s,%s",f->name,ph_cc[op&3]); }
    else if(op==0x9B) snprintf(b,n,"ZBRR");
    else if(op==0xBB) snprintf(b,n,"ZBSR");
    else if(op==0x9F) snprintf(b,n,"BXA");
    else if(op==0xBF) snprintf(b,n,"BSXA");
    else if(op==0x17) snprintf(b,n,"RETC,UN");
    else if(op==0x37) snprintf(b,n,"RETE,UN");
    else if(op==0x40) snprintf(b,n,"HALT");
    else snprintf(b,n,"$%02X",op);
}

/* ph_decode_target: decode the target of a branch/call/ZBRR/ZBSR at addr.
 * Inputs: addr = instruction address.
 * Outputs: *t = target address (for indirect: the pointer address),
 *          *ind = 1 if indirect; returns 1 if the opcode is a branch, else 0.
 * Clobbers: none. */
static int ph_decode_target(int addr, int *t, int *ind){
    unsigned char op=(unsigned char)ph_rb(addr), b1=(unsigned char)ph_rb(addr+1), b2=(unsigned char)ph_rb(addr+2);
    const BrFam *f = ph_brfam(op);
    if(f){
        *ind = (b1&0x80)!=0;
        if(f->rel){ int d=b1&0x7F; if(d&0x40) d-=128; *t=addr+2+d; }
        else *t=((b1&0x7F)<<8)|b2;
        return 1;
    }
    if(op==0x9B||op==0xBB){ int d=b1&0x7F; if(d&0x40) d-=128; *ind=(b1&0x80)!=0; *t=d&0x1FFF; return 1; }
    return 0;
}

/* ph_can_fall: can execution continue into the next instruction?
 * Inputs: op = opcode.  Outputs: 0 for unconditional jump/return/HALT, else 1.
 * Clobbers: none. */
static int ph_can_fall(unsigned char op){
    return !(op==0x1B||op==0x1F||op==0x9B||op==0x9F||op==0x17||op==0x37||op==0x40);
}

/* ph_emit: queue one hint.
 * Inputs: bit = PH_* category, line = source line, saving = bytes saved
 *         (0 if speed only), fmt/... = message text (tag " [name]" appended).
 * Outputs: none.  Clobbers: pw[], npw, pwcap, ph_count[], ph_saved. */
static void ph_emit(int bit, int line, int saving, const char *fmt, ...){
    char buf[640], tag[40]; va_list ap; const char *nm="?"; int bi=0;
    for(int i=0;ph_names[i].name;i++) if(ph_names[i].bit==bit){ nm=ph_names[i].name; bi=i; }
    va_start(ap,fmt); vsnprintf(buf,sizeof(buf)-40,fmt,ap); va_end(ap);
    snprintf(tag,sizeof(tag)," [%s]",nm); strcat(buf,tag);
    if(npw>=pwcap){
        pwcap = pwcap ? pwcap*2 : 64;
        pw = (PWarn*)realloc(pw,(size_t)pwcap*sizeof(PWarn));
        if(!pw){ fprintf(stderr,"ERROR: out of memory\n"); exit(1); }
    }
    pw[npw].line=line; pw[npw].seq=npw; pw[npw].text=xstrdup(buf); npw++;
    ph_count[bi]++; ph_saved+=saving;
}

/* is_directive_mn: non-instruction mnemonics (emit data or nothing).
 * Inputs: mn.  Outputs: 1 if directive.  Clobbers: none. */
static int is_directive_mn(const char *mn){
    return !strcmp(mn,"ORG")||!strcmp(mn,"EQU")||!strcmp(mn,"DB")||!strcmp(mn,"DW")||
           !strcmp(mn,"DS")||!strcmp(mn,"RES")||!strcmp(mn,"END");
}

/* ph_split_addr: address part of a "cc,addr" / "Rn,addr" operand list.
 * Inputs: o0/o1 = first two operands, nops = operand count.
 * Outputs: malloc'd text of the address operand (keeps a leading '*').
 * Clobbers: heap. */
static char *ph_split_addr(const char *o0, const char *o1, int nops){
    if(nops>1 && o1[0]) return xstrdup(o1);
    const char *q=o0;
    while(*q && *q!=' ' && *q!='\t') q++;
    while(*q==' '||*q=='\t') q++;
    return xstrdup(q);
}

/* peep_label_only / peep_line_done: record state after each source line.
 * peep_line_done is called by main() after assemble_line() on pass 2.
 * Inputs: cur_* globals set by assemble_line().
 * Outputs: appends a PInst for each instruction line; label-only lines set
 *          the pending-label flag.  Clobbers: pins[], npins, peep_pend_label. */
static void peep_line_done(void){
    if(pass!=2) return;
    if(!cur_mn[0]){ if(cur_lbl_defined) peep_pend_label=1; return; }
    if(is_directive_mn(cur_mn) || pc<=line_start_pc) return;
    if(npins>=pinscap){
        pinscap = pinscap ? pinscap*2 : 512;
        pins = (PInst*)realloc(pins,(size_t)pinscap*sizeof(PInst));
        if(!pins){ fprintf(stderr,"ERROR: out of memory\n"); exit(1); }
    }
    PInst *r=&pins[npins++];
    r->addr=line_start_pc; r->len=pc-line_start_pc; r->line=lineno;
    r->lab_before = cur_lbl_defined || peep_pend_label; peep_pend_label=0;
    r->opnd=NULL;
    unsigned char op=(unsigned char)ph_rb(r->addr);
    if(ph_brfam(op)) r->opnd=ph_split_addr(cur_op0,cur_op1,cur_nops);
    else if(op==0x9B||op==0xBB) r->opnd=xstrdup(cur_op0);
}

/* ph_cc_dead_from: is CC provably overwritten before anything can read it,
 * scanning forward from instruction index ix? Behaviour below was verified
 * in pipbug_wrap (baseline CC=GT, R0=$80).
 *   Setters  : LOD/EOR/AND/IOR/ADD/SUB/COM in all forms, TMI, TPSU, TPSL,
 *              STRZ,R1-R3, and CPSL/PPSL whose mask has both CC bits ($C0).
 *   Neutral  : NOP, STRR, STRA, RRL, RRR, CPSU/PPSU, CPSL/PPSL not touching
 *              the CC bits (keep scanning).
 *   HALT     : ends the scan as dead.
 *   Anything else (conditional branch/call/return, unconditional transfer,
 *   SPSL/SPSU, DAR, port I/O, a gap between instructions, more than 8
 *   instructions) is treated as "CC may be read".
 * Inputs: ix = index into pins[].
 * Outputs: source line of the instruction that overwrites CC (or of HALT);
 *          0 if CC may be live.  Clobbers: none. */
static int ph_cc_dead_from(int ix){
    for(int k=0; k<8 && ix<npins; k++, ix++){
        const PInst *r=&pins[ix];
        if(k>0 && r->addr!=pins[ix-1].addr+pins[ix-1].len) return 0;
        unsigned char op=(unsigned char)ph_rb(r->addr), m=(unsigned char)ph_rb(r->addr+1);
        unsigned char hi=(unsigned char)(op&0xF0);
        if(op==0x40) return r->line;                                        /* HALT */
        if(hi==0x00||hi==0x20||hi==0x40||hi==0x60||hi==0x80||hi==0xA0||hi==0xE0)
            return r->line;                                                 /* LOD EOR AND IOR ADD SUB COM */
        if((op>=0xF4&&op<=0xF7)||op==0xB4||op==0xB5) return r->line;       /* TMI, TPSU, TPSL */
        if(op>=0xC1&&op<=0xC3) return r->line;                              /* STRZ,R1-R3 */
        if(op==0xC0||(op>=0xC8&&op<=0xCF)) continue;                        /* NOP, STRR, STRA */
        if((op>=0xD0&&op<=0xD3)||(op>=0x50&&op<=0x53)) continue;            /* RRL, RRR */
        if(op==0x74||op==0x76) continue;                                    /* CPSU, PPSU */
        if(op==0x75||op==0x77){                                             /* CPSL, PPSL */
            if((m&0xC0)==0xC0) return r->line;
            if((m&0xC0)==0) continue;
            return 0;
        }
        return 0;
    }
    return 0;
}

typedef struct { int zp, len, levels; const char *opnd; char desc[220]; } Thunk;

/* ph_thunk: is the instruction at address t a "jump thunk" (ZBRR x; BCTx,UN x;
 * or ZBSR/BSTx,UN x followed by RETC,UN)? Follows chains of thunks.
 * Inputs: t = address, depth = recursion guard.
 * Outputs: returns 1 and fills *th (final target operand text, whether the
 *          final target is reachable by ZBRR/ZBSR, size in bytes of THIS
 *          thunk, number of levels, description); else 0.
 * Clobbers: *th. */
static int ph_thunk(int t, Thunk *th, int depth){
    if(t<0||t>=MAX_ROM||ph_ins_at[t]<0||depth>8) return 0;
    int ix=ph_ins_at[t]; const PInst *r=&pins[ix]; unsigned char op=(unsigned char)ph_rb(t);
    const PInst *n=(ix+1<npins && pins[ix+1].addr==t+r->len && !pins[ix+1].lab_before)?&pins[ix+1]:NULL;
    int tailret=(n && ph_rb(n->addr)==0x17);
    char nm[48];
    memset(th,0,sizeof(*th));
    if(op==0x9B){ th->zp=1; th->len=r->len; }
    else if(op==0xBB && tailret){ th->zp=1; th->len=r->len+n->len; }
    else if(op==0x1B||op==0x1F){ th->len=r->len; }
    else if((op==0x3B||op==0x3F) && tailret){ th->len=r->len+n->len; }
    else return 0;
    if(!r->opnd||!r->opnd[0]) return 0;
    th->opnd=r->opnd; th->levels=1;
    ph_mn(nm,sizeof(nm),op);
    snprintf(th->desc,sizeof(th->desc),"%s %.120s%s",nm,r->opnd,tailret&&(op==0xBB||op==0x3B||op==0x3F)?" / RETC,UN":"");
    int tt,ind;
    if(ph_decode_target(t,&tt,&ind) && !ind && tt!=t){
        Thunk t2;
        if(ph_thunk(tt,&t2,depth+1)){ th->opnd=t2.opnd; th->zp=t2.zp; th->levels=t2.levels+1; }
    }
    return 1;
}

/* ph_caller: classify an instruction as a branch/call that could be
 * retargeted. Inputs: a = record. Outputs: returns 1 and sets *t (direct
 * target), *is_call, *is_un (unconditional, i.e. could become ZBRR/ZBSR);
 * 0 if not a direct branch. Clobbers: *t,*is_call,*is_un. */
static int ph_caller(const PInst *a, int *t, int *is_call, int *is_un){
    unsigned char op=(unsigned char)ph_rb(a->addr); int ind;
    const BrFam *f=ph_brfam(op);
    if(f){ *is_call=f->call; *is_un=(!f->reg && (op&3)==3); }
    else if(op==0x9B){ *is_call=0; *is_un=1; }
    else if(op==0xBB){ *is_call=1; *is_un=1; }
    else return 0;
    if(!ph_decode_target(a->addr,t,&ind) || ind) return 0;
    return 1;
}

/* ph_thunks: thunk-collapse analysis (see header v1.20 -> v1.21).
 * A caller of a jump thunk can branch/call the final target directly: the
 * caller grows by at most 1 byte (relative -> absolute), the thunk can be
 * dropped when nothing else enters or references it. Reported only when the
 * total byte change over all callers is <= 0.
 * Inputs: pins[], ph_ins_at[], labels[] (with ref counts).
 * Outputs: hints via ph_emit().  Clobbers: static scratch arrays. */
static void ph_thunks(void){
    static int ncall[MAX_ROM], sumd[MAX_ROM];
    static unsigned char rep[MAX_ROM];
    static int lsites[MAX_LABELS];
    memset(ncall,0,sizeof(ncall)); memset(sumd,0,sizeof(sumd));
    memset(rep,0,sizeof(rep)); memset(lsites,0,sizeof(lsites));
    for(int pass2=0;pass2<2;pass2++){
        for(int i=0;i<npins;i++){
            const PInst *a=&pins[i]; int t,is_call,is_un; Thunk th;
            if(!ph_caller(a,&t,&is_call,&is_un)) continue;
            if(t<0||t>=MAX_ROM||!ph_thunk(t,&th,0)) continue;
            int newlen=(is_un&&th.zp)?2:3;
            if(pass2==0){
                ncall[t]++; sumd[t]+=newlen-a->len;
                if(a->opnd){
                    const char *s=a->opnd; char id[64]; int k=0;
                    if(*s=='*') s++;
                    if(isalpha((unsigned char)*s)||*s=='_'){
                        while((isalnum((unsigned char)*s)||*s=='_')&&k<63) id[k++]=*s++;
                        id[k]=0;
                        int li=label_find_index(id); if(li>=0) lsites[li]++;
                    }
                }
                continue;
            }
            /* reporting pass */
            int ix=ph_ins_at[t], falls=0, blocked=0;
            if(ix>0){ const PInst *pv=&pins[ix-1]; if(pv->addr+pv->len==t && ph_can_fall((unsigned char)ph_rb(pv->addr))) falls=1; }
            for(int j=0;j<nlabels;j++) if(labels[j].value==t && labels[j].refs-lsites[j]>0) blocked=1;
            int delta=sumd[t]-th.len;
            if(falls||blocked||delta>0) continue;
            unsigned char op=(unsigned char)ph_rb(a->addr);
            char oldm[48], newm[48], lv[40]="", sv[48];
            ph_mn(oldm,sizeof(oldm),op);
            if(is_un&&th.zp) snprintf(newm,sizeof(newm),"%s",is_call?"ZBSR":"ZBRR");
            else { const BrFam *f=ph_brfam(op); unsigned char nop=f?(unsigned char)(op+(f->rel?4:0)):(unsigned char)(is_call?0x3F:0x1F); ph_mn(newm,sizeof(newm),nop); }
            if(th.levels>1) snprintf(lv,sizeof(lv),", via %d thunk levels",th.levels);
            if(delta<0) snprintf(sv,sizeof(sv),"saves %d byte(s)",-delta); else snprintf(sv,sizeof(sv),"same size, faster");
            ph_emit(PH_THUNK,a->line,rep[t]?0:-delta,
                "%s %.100s goes via a thunk at line %d (%s%s) -- suggest %s %.100s; if all %d caller(s) are redirected and the thunk dropped: %s",
                oldm,a->opnd?a->opnd:"?",pins[ix].line,th.desc,lv,newm,th.opnd,ncall[t],sv);
            rep[t]=1;
        }
    }
}

static int ph_cmp(const void *x, const void *y){
    const PWarn *a=(const PWarn*)x, *b=(const PWarn*)y;
    if(a->line!=b->line) return a->line<b->line?-1:1;
    return a->seq<b->seq?-1:(a->seq>b->seq);
}

/* peep_report: run all enabled peephole checks, print the hints (sorted by
 * source line) and a one-line summary, then free the working storage.
 * Inputs: pins[] from pass 2, rom[], labels[].  Outputs: stderr.
 * Clobbers: heap, ph_ins_at[]. */
static void peep_report(void){
    if(ph_enabled && !errors && npins>0){
        for(int i=0;i<MAX_ROM;i++) ph_ins_at[i]=-1;
        for(int i=0;i<npins;i++) ph_ins_at[pins[i].addr]=i;
        for(int i=0;i<npins;i++){
            const PInst *a=&pins[i];
            unsigned char op=(unsigned char)ph_rb(a->addr), b1=(unsigned char)ph_rb(a->addr+1);
            const PInst *n=(i+1<npins && pins[i+1].addr==a->addr+a->len)?&pins[i+1]:NULL;
            const BrFam *f=ph_brfam(op);
            char m1[48], m2[48], m3[48];
            int t=0, ind=0, isbr=ph_decode_target(a->addr,&t,&ind);
            const char *ao=a->opnd?a->opnd:"?";
            /* clear / test (R0 idioms) */
            if((ph_enabled&PH_CLEAR) && ((op==0x04&&b1==0)||(op==0x44&&b1==0)))
                ph_emit(PH_CLEAR,a->line,1,"%s,R0 0 can be EORZ,R0 -- 1 byte shorter, same cycles and CC",op==0x04?"LODI":"ANDI");
            if((ph_enabled&PH_TEST) && ((op==0x64&&b1==0)||(op==0x44&&b1==0xFF)||(op==0x24&&b1==0)))
                ph_emit(PH_TEST,a->line,1,"%s,R0 %s only tests R0 -- use IORZ,R0 (1 byte shorter, same CC)",
                    op==0x64?"IORI":(op==0x44?"ANDI":"EORI"),op==0x44?"$FF":"0");
            /* zpage */
            if((ph_enabled&PH_ZPAGE) && (op==0x1F||op==0x3F)){
                int T=((b1&0x7F)<<8)|ph_rb(a->addr+2), off=T-(a->addr+2);
                int zp=(T<=0x3F)||(T>=0x1FC0&&T<=0x1FFF);
                if(zp && !(off>=-64&&off<=63))
                    ph_emit(PH_ZPAGE,a->line,1,"%s %s targets zero page -- use %s %s (1 byte shorter, same cycles)",
                        op==0x1F?"BCTA,UN":"BSTA,UN",ao,op==0x1F?"ZBRR":"ZBSR",ao);
            }
            /* next */
            if((ph_enabled&PH_NEXT) && f && !f->call && (!f->reg||f->base==0x58||f->base==0x5C) && isbr && !ind && t==a->addr+a->len){
                ph_mn(m1,sizeof(m1),op);
                ph_emit(PH_NEXT,a->line,a->len,"%s %s branches to the next instruction -- no-op, delete it (%d byte(s))",m1,ao,a->len);
            }
            /* psw merge */
            if((ph_enabled&PH_PSW) && op>=0x74 && op<=0x77 && n && ph_rb(n->addr)==op && !n->lab_before){
                static const char *pn[4]={"CPSU","CPSL","PPSU","PPSL"};
                int b2=ph_rb(n->addr+1);
                ph_emit(PH_PSW,a->line,2,"%s $%02X followed by %s $%02X (line %d) -- merge into one %s $%02X (saves 2 bytes)",
                    pn[op-0x74],b1,pn[op-0x74],b2,n->line,pn[op-0x74],b1|b2);
            }
            /* retcc: conditional/unconditional branch to a bare RETC,UN */
            if((ph_enabled&PH_RETCC) && f && !f->call && !f->reg && (f->base==0x18||f->base==0x1C) && isbr && !ind
               && t>=0 && t<MAX_ROM && ph_ins_at[t]>=0 && ph_rb(t)==0x17){
                ph_mn(m1,sizeof(m1),op);
                ph_emit(PH_RETCC,a->line,a->len-1,"%s %s branches to RETC,UN (line %d) -- use RETC,%s (saves %d byte(s))",
                    m1,ao,pins[ph_ins_at[t]].line,ph_cc[op&3],a->len-1);
            }
            /* condtail: conditional call + RETC,UN -> conditional jump */
            if((ph_enabled&PH_CONDTAIL) && f && f->call && (f->reg||(op&3)!=3) && n && ph_rb(n->addr)==0x17){
                ph_mn(m1,sizeof(m1),op); ph_mn(m2,sizeof(m2),(unsigned char)(op-0x20));
                ph_emit(PH_CONDTAIL,a->line,0,"%s %s followed by RETC,UN (line %d) -- tail call: suggest %s %s and keep the RETC,UN (saves a return-stack level and cycles, no size change)",
                    m1,ao,n->line,m2,ao);
            }
            /* loop: SUBI/ADDI,Rn 1 + BCFx,EQ -> BDRx/BIRx */
            if((ph_enabled&PH_LOOP) && ((op>=0xA4&&op<=0xA7)||(op>=0x84&&op<=0x87)) && b1==1 && n && !n->lab_before
               && (ph_rb(n->addr)==0x98||ph_rb(n->addr)==0x9C)){
                int dec=(op>=0xA4), rel=(ph_rb(n->addr)==0x98);
                snprintf(m3,sizeof(m3),"%s",dec?(rel?"BDRR":"BDRA"):(rel?"BIRR":"BIRA"));
                ph_mn(m2,sizeof(m2),(unsigned char)ph_rb(n->addr));
                ph_emit(PH_LOOP,a->line,2,"%s,R%d 1 / %s %s (line %d) -> %s,R%d %s -- saves 2 bytes and is faster; %s leaves CC and carry unchanged, so use only if they are dead afterwards",
                    dec?"SUBI":"ADDI",op&3,m2,n->opnd?n->opnd:"?",n->line,m3,op&3,n->opnd?n->opnd:"?",m3);
            }
            /* brn: test-register + BCFx,EQ -> BRNx,Rn */
            if((ph_enabled&PH_BRN) && n && !n->lab_before && (ph_rb(n->addr)==0x98||ph_rb(n->addr)==0x9C)){
                int rg=-1, sv=0, lodz=0; char lhs[40]="";
                if(op>=0xE4&&op<=0xE7&&b1==0){ rg=op&3; sv=2; snprintf(lhs,sizeof(lhs),"COMI,R%d 0",rg); }
                else if(op==0x60){ rg=0; sv=1; snprintf(lhs,sizeof(lhs),"IORZ,R0"); }
                else if(op>=0x01&&op<=0x03){ rg=op&3; sv=1; lodz=1; snprintf(lhs,sizeof(lhs),"LODZ,R%d",rg); }
                if(rg>=0){
                    int rel=(ph_rb(n->addr)==0x98); const char *no=n->opnd?n->opnd:"?";
                    ph_mn(m2,sizeof(m2),(unsigned char)ph_rb(n->addr));
                    ph_emit(PH_BRN,a->line,sv,"%s / %s %s (line %d) -> %s,R%d %s -- saves %d byte(s); %s leaves CC unchanged%s",
                        lhs,m2,no,n->line,rel?"BRNR":"BRNA",rg,no,sv,rel?"BRNR":"BRNA",
                        lodz?" and R0 is no longer loaded":"");
                }
            }
            /* skipbyte: BCTR/BCTA,UN over 1-2 bytes -> DB $E4 / DB $EC (v1.22) */
            if((ph_enabled&PH_SKIPBYTE) && (op==0x1B||op==0x1F) && isbr && !ind){
                int nb=t-(a->addr+a->len);
                if(nb==1||nb==2){
                    int pos=a->addr+a->len, j=i+1, lab=0, tiled=1;
                    while(pos<t){
                        if(j>=npins||pins[j].addr!=pos){ tiled=0; break; }
                        if(pins[j].lab_before) lab=1;
                        pos+=pins[j].len; j++;
                    }
                    int s1=ph_rb(a->addr+a->len), s2=ph_rb(a->addr+a->len+1);
                    /* a 2-byte skip is clean only if the first skipped byte, seen as the
                     * high address byte of COMA,R0, has no index-control bits (bits 6:5) */
                    int clean=(nb==1)||(((s1>>5)&3)==0);
                    if(tiled && pos==t && j<npins && pins[j].addr==t && lab && clean){
                        int ccl=ph_cc_dead_from(j);
                        if(ccl){
                            ph_mn(m1,sizeof(m1),op);
                            if(nb==1)
                                ph_emit(PH_SKIPBYTE,a->line,a->len-1,"%s %s skips 1 byte ($%02X, line %d) -- use DB $E4 (COMI,R0: sets CC only) instead of the branch, saves %d byte(s); CC is overwritten at line %d",
                                    m1,ao,s1,pins[i+1].line,a->len-1,ccl);
                            else {
                                int ea=(a->addr&0x6000)|((s1&0x1F)<<8)|s2;
                                ph_emit(PH_SKIPBYTE,a->line,a->len-1,"%s %s skips 2 bytes ($%02X $%02X, line %d) -- use DB $EC (COMA,R0: sets CC, reads %s$%04X) instead of the branch, saves %d byte(s); CC is overwritten at line %d",
                                    m1,ao,s1,s2,pins[i+1].line,(s1&0x80)?"pointer at ":"",ea,a->len-1,ccl);
                            }
                        }
                    }
                }
            }
            /* dead: unlabelled instruction right after an unconditional transfer */
            if((ph_enabled&PH_DEAD) && i>0 && !a->lab_before && ph_can_fall(op)){
                const PInst *pv=&pins[i-1]; unsigned char po=(unsigned char)ph_rb(pv->addr);
                if(pv->addr+pv->len==a->addr && !ph_can_fall(po)){
                    ph_mn(m1,sizeof(m1),po);
                    ph_emit(PH_DEAD,a->line,0,"unreachable code: follows %s (line %d) with no label before it",m1,pv->line);
                }
            }
        }
        if(ph_enabled&PH_THUNK) ph_thunks();
        if(npw){
            qsort(pw,(size_t)npw,sizeof(PWarn),ph_cmp);
            for(int i=0;i<npw;i++) fprintf(stderr,"WARN line %d: %s\n",pw[i].line,pw[i].text);
            fprintf(stderr,"Hints:");
            for(int i=0;ph_names[i].name;i++) if(ph_count[i]) fprintf(stderr," %s=%d",ph_names[i].name,ph_count[i]);
            fprintf(stderr," (about %d byte(s) potential saving)\n",ph_saved);
        }
    }
    for(int i=0;i<npw;i++) free(pw[i].text);
    for(int i=0;i<npins;i++) free(pins[i].opnd);
    free(pw); free(pins); pw=NULL; pins=NULL; npw=npins=0;
}

/* ph_set_list: apply a --warn= / --no-warn= comma list.
 * Inputs: list = comma-separated names (clear,test,zpage,next,psw,retcc,
 *         condtail,loop,brn,thunk,dead; legacy label,rel,skip,tail; groups
 *         advisory,peephole,all), on = 1 enable / 0 disable.
 * Outputs: returns 1 ok, 0 on unknown name (message printed).
 * Clobbers: ph_enabled, warn_* flags. */
static int ph_set_list(const char *list, int on){
    char tmp[256]; snprintf(tmp,sizeof(tmp),"%s",list);
    for(char *tok=strtok(tmp,",");tok;tok=strtok(NULL,",")){
        int bits=0, ok=1;
        if(!strcmp(tok,"all")){ bits=PH_ALL; warn_inline_label=warn_local_abs_branch=warn_branch_skip=warn_tail_call=on; }
        else if(!strcmp(tok,"peephole")) bits=PH_ALL;
        else if(!strcmp(tok,"advisory")) bits=PH_ADVISORY;
        else if(!strcmp(tok,"label")) warn_inline_label=on;
        else if(!strcmp(tok,"rel")) warn_local_abs_branch=on;
        else if(!strcmp(tok,"skip")) warn_branch_skip=on;
        else if(!strcmp(tok,"tail")) warn_tail_call=on;
        else {
            ok=0;
            for(int i=0;ph_names[i].name;i++) if(!strcmp(tok,ph_names[i].name)){ bits=ph_names[i].bit; ok=1; }
        }
        if(!ok){ fprintf(stderr,"ERROR: unknown warning name '%s'\n",tok); return 0; }
        if(on) ph_enabled|=bits; else ph_enabled&=~bits;
    }
    return 1;
}

static void assemble_line(char *line){
    cur_mn[0]=0; cur_op0[0]=0; cur_op1[0]=0; cur_nops=0; cur_lbl_defined=0;
    line_start_pc = pc;  /* BUG-ASM-13: fix '$' to this line's start address before anything emits */
    char buf[MAX_LINE]; strncpy(buf,line,MAX_LINE-1); buf[MAX_LINE-1]=0;
    upcase(buf);
    strip_comment(buf);
    char *p=buf;
    p=skip_ws(buf); if(!*p) return;
    char lbl[64]="";
    int lbl_has_colon = 0;
    if(!isspace((unsigned char)buf[0])&&buf[0]){
        int i=0;
        while((isalnum((unsigned char)*p)||*p=='_')&&i<63) lbl[i++]=*p++;
        lbl[i]=0;
        if(isalnum((unsigned char)*p)||*p=='_'){
            if(pass==2){ fprintf(stderr,"ERROR line %d: label name too long (max 63 chars) near '%s'\n",lineno,lbl); errors++; }
        }
        if(*p==':'){
            lbl_has_colon = 1;
            p++;
        }
        p=skip_ws(p);
        /* EQU and ORG each call label_define() themselves (EQU: constant
         * value; ORG: address AFTER the org change) — skip the generic
         * pc-based definition here for those two mnemonics so a genuine
         * duplicate name isn't reported twice (once from here, once from
         * the mnemonic's own handler). Peek at the upcoming token only;
         * mnemonic parsing itself still happens normally below. */
        int is_equ_ahead = (strncmp(p,"EQU",3)==0 && !(isalnum((unsigned char)p[3])||p[3]=='_'));
        int is_org_ahead = (strncmp(p,"ORG",3)==0 && !(isalnum((unsigned char)p[3])||p[3]=='_'));
        if(pass==1 && !is_equ_ahead && !is_org_ahead) label_define(lbl,pc);
        if(*lbl && lbl_has_colon && *p && pass==2 && warn_inline_label){
            fprintf(stderr,"WARN line %d: label and instruction on same line\n",lineno);
        }
    }
    cur_lbl_defined = (*lbl!=0);
    /* v1.4 FIX: allow "LABEL: OPCODE operands" on one line.
     * After defining the label, continue to assemble any instruction that follows.
     * A colon with nothing after it (label-only line) is handled by the !*p check. */
    if(!*p){ if(pass==2) tailcall_label(); return; }
    char mn[32]=""; int mi=0;
    while((isalpha((unsigned char)*p)||isdigit((unsigned char)*p))&&mi<31) mn[mi++]=*p++;
    mn[mi]=0;
    if(isalpha((unsigned char)*p)||isdigit((unsigned char)*p)){
        if(pass==2){ fprintf(stderr,"ERROR line %d: mnemonic too long (max 31 chars) near '%s'\n",lineno,mn); errors++; }
    }
    p=skip_ws(p); if(*p==',') p++; p=skip_ws(p);
    char ops[64][128];
    for(int _i=0;_i<64;_i++) ops[_i][0]=0;
    int nops=split_ops(p,ops,64);
    snprintf(cur_mn,sizeof(cur_mn),"%s",mn); snprintf(cur_op0,sizeof(cur_op0),"%.127s",ops[0]);
    snprintf(cur_op1,sizeof(cur_op1),"%.127s",ops[1]); cur_nops=nops;
    tailcall_check(mn,ops,nops,*lbl!=0);

    if(strcmp(mn,"ORG")==0){
        int ok,v=eval_expr(ops[0],&ok);
        if(ok){
            if(v<0||v>=MAX_ROM){ if(pass==2){fprintf(stderr,"ERROR line %d: ORG address $%04X out of range\n",lineno,v); errors++;} }
            pc=v; if(pass==1&&*lbl) label_define(lbl,pc);
        } else if(pass==2){fprintf(stderr,"ERROR line %d: bad ORG expression '%s'\n",lineno,ops[0]); errors++;}
        return;
    }
    if(strcmp(mn,"EQU")==0){ int ok,v=eval_expr(ops[0],&ok); if(ok) label_define(lbl,v); else if(pass==2){fprintf(stderr,"ERROR line %d: bad EQU expression '%s'\n",lineno,ops[0]); errors++;} return; }
    if(strcmp(mn,"DS")==0||strcmp(mn,"RES")==0){
        int ok,n=eval_expr(ops[0],&ok);
        if(ok){
            if(n<0){ if(pass==2){fprintf(stderr,"ERROR line %d: %s count %d must be non-negative\n",lineno,mn,n); errors++;} }
            else { for(int i=0;i<n;i++){emit(pc,0);pc++;} }
        } else if(pass==2){fprintf(stderr,"ERROR line %d: bad %s expression '%s'\n",lineno,mn,ops[0]); errors++;}
        return;
    }
    if(strcmp(mn,"DB" )==0){
        for(int i=0;i<nops;i++){
            char *s=skip_ws(ops[i]);
            if(*s=='"'){
                s++; /* skip opening quote */
                while(*s && *s!='"'){
                    emit(pc,(unsigned char)*s);
                    pc++;
                    s++;
                }
                if(*s!='"' && pass==2){
                    fprintf(stderr,"ERROR line %d: unterminated string literal in DB\n",lineno);
                    errors++;
                }
            } else {
                int ok,v=eval_expr(ops[i],&ok);
                if(!ok){
                    if(pass==2){ fprintf(stderr,"ERROR line %d: bad DB expression '%s'\n",lineno,ops[i]); errors++; }
                    emit(pc,0); pc++;   /* BUG-ASM-16: keep pc advancing (forward ref in pass 1), as DW does */
                    continue;
                }
                emit(pc,(unsigned char)(v&0xFF)); pc++;
            }
        }
        return;
    }
    if(strcmp(mn,"DW" )==0){ for(int i=0;i<nops;i++){int ok,v=eval_expr(ops[i],&ok); if(!ok){ if(pass==2){fprintf(stderr,"ERROR line %d: bad DW expression '%s'\n",lineno,ops[i]); errors++;} emit(pc,0);pc++; emit(pc,0);pc++; continue; } emit(pc,(unsigned char)((v>>8)&0xFF));pc++; emit(pc,(unsigned char)(v&0xFF));pc++;} return; }
    if(strcmp(mn,"END")==0) return;
    /* 2650 hardware constraints — warn on architecturally invalid encodings */
    if(strcmp(mn,"NOP" )==0){ emit(pc,0xC0);pc++; return; }
    if(strcmp(mn,"HALT")==0){ emit(pc,0x40);pc++; return; }
    if(strcmp(mn,"SPSU")==0){ emit(pc,0x12);pc++; return; }
    if(strcmp(mn,"SPSL")==0){ emit(pc,0x13);pc++; return; }
    if(strcmp(mn,"LPSU")==0){ emit(pc,0x92);pc++; return; }
    if(strcmp(mn,"LPSL")==0){ emit(pc,0x93);pc++; return; }
    /* ZBRR and ZBSR - Both Buggy but fix/test later.
    * 2 byte isntruction, 2nd byte bit 7 is indirect, bits 6-0 is SIGNED offset
    * from 2650 User manual:
        ZBSR (*)a - ZERO BRANCH TO SUBROUTINE, RELATIVE
        ZBRR (*)a - ZERO BRANCH, RELATIVE
        The specified value, a, is interpreted as a relative displacement from
        page zero, byte zero. Therefore, displacement may be specified from -64 to
        +63 bytes. The address calculation is modulo 8192, so the negative
        displacement actually will develop addresses at the end of page zero. For
        example, ZBRR -8 will develop an effective address of 8184, and ZBRR +52
        will develop an effective address of 52.
        This instruction causes the processor to clear address bits #13 and #14,
        the page address bits, and may be executed anywhere within addressable
        memory.
        Indirect addressing may be specified. (Bit 7 2nd byte)
    * ZBSR replaces BSTA,UN & ZBRR replaces BCTA,UN both unconditional
    * short form 2 byte replacements instead of 3, with indirect (*) lookup table*/
    /* ZBRR / ZBSR: 2-byte instructions. Second byte = indirect flag (bit7) + signed 7-bit
     * displacement from address zero (NOT PC-relative). Range -64..+63.
     * Negative displacements address end of page zero via modulo 8192. */
    if(strcmp(mn,"ZBRR")==0||strcmp(mn,"ZBSR")==0){
        emit(pc,(strcmp(mn,"ZBRR")==0)?0x9B:0xBB); pc++;
        char *a=ops[0]; int ind=0;
        if(*a=='*'){ind=1;a++;}
        int ok,v=eval_expr(a,&ok);
        if(!ok&&pass==2){fprintf(stderr,"ERROR line %d: bad %s operand '%s'\n",lineno,mn,a); errors++;}
        if(pass==2&&ok&&(v<-64||v>63)){
            fprintf(stderr,"ERROR line %d: %s displacement %d out of range (-64..+63)\n",lineno,mn,v);
            errors++;
        }
        unsigned char disp=(unsigned char)(v&0x7F);
        if(ind) disp|=0x80;
        emit(pc,disp); pc++;
        return;
    }
    if(strcmp(mn,"RETC")==0){ int cc=cc_val(ops[0]); if(cc<0){fprintf(stderr,"ERROR line %d: RETC needs EQ/GT/LT/UN\n",lineno);errors++;return;} emit(pc,(unsigned char)(0x14|cc));pc++;return; }
    if(strcmp(mn,"RETE")==0){ int cc=cc_val(ops[0]); if(cc<0){fprintf(stderr,"ERROR line %d: RETE needs EQ/GT/LT/UN\n",lineno);errors++;return;} emit(pc,(unsigned char)(0x34|cc));pc++;return; }
    if(strcmp(mn,"CPSU")==0){ emit(pc,0x74);pc++; int ok,v=eval_expr(ops[0],&ok); if(!ok&&pass==2){fprintf(stderr,"ERROR line %d: bad %s operand '%s'\n",lineno,mn,ops[0]);errors++;} emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"CPSL")==0){ emit(pc,0x75);pc++; int ok,v=eval_expr(ops[0],&ok); if(!ok&&pass==2){fprintf(stderr,"ERROR line %d: bad %s operand '%s'\n",lineno,mn,ops[0]);errors++;} emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"PPSU")==0){ emit(pc,0x76);pc++; int ok,v=eval_expr(ops[0],&ok); if(!ok&&pass==2){fprintf(stderr,"ERROR line %d: bad %s operand '%s'\n",lineno,mn,ops[0]);errors++;} emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"PPSL")==0){ emit(pc,0x77);pc++; int ok,v=eval_expr(ops[0],&ok); if(!ok&&pass==2){fprintf(stderr,"ERROR line %d: bad %s operand '%s'\n",lineno,mn,ops[0]);errors++;} emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"TPSU")==0){ emit(pc,0xB4);pc++; int ok,v=eval_expr(ops[0],&ok); if(!ok&&pass==2){fprintf(stderr,"ERROR line %d: bad %s operand '%s'\n",lineno,mn,ops[0]);errors++;} emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"TPSL")==0){ emit(pc,0xB5);pc++; int ok,v=eval_expr(ops[0],&ok); if(!ok&&pass==2){fprintf(stderr,"ERROR line %d: bad %s operand '%s'\n",lineno,mn,ops[0]);errors++;} emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"DAR")==0){ int r=reg_val(ops[0]); if(r<0){fprintf(stderr,"ERROR line %d: DAR needs Rn\n",lineno);errors++;return;} emit(pc,(unsigned char)(0x94|r));pc++;return; }
    if(strcmp(mn,"TMI")==0){
        int r=reg_val(ops[0]); if(r<0){fprintf(stderr,"ERROR line %d: TMI needs Rn\n",lineno);errors++;return;}
        emit(pc,(unsigned char)(0xF4|r)); pc++;
        /* mask is space-separated after register in ops[0], or in ops[1] if comma-separated */
        char *mask_s = (nops>=2 && ops[1][0]) ? ops[1] : ops0_after_reg(ops[0]);
        if(!mask_s||!mask_s[0]){fprintf(stderr,"ERROR line %d: TMI needs mask operand\n",lineno);errors++;emit(pc,0);pc++;return;}
        int ok,v=eval_expr(mask_s,&ok);
        if(!ok&&pass==2){fprintf(stderr,"ERROR line %d: bad TMI mask '%s'\n",lineno,mask_s);errors++;}
        emit(pc,(unsigned char)(v&0xFF)); pc++;
        return;
    }
    if(strcmp(mn,"RRL")==0){ int r=reg_val(ops[0]); if(r<0){fprintf(stderr,"ERROR line %d: RRL needs Rn\n",lineno);errors++;return;} emit(pc,(unsigned char)(0xD0|r));pc++;return; }
    if(strcmp(mn,"RRR")==0){ int r=reg_val(ops[0]); if(r<0){fprintf(stderr,"ERROR line %d: RRR needs Rn\n",lineno);errors++;return;} emit(pc,(unsigned char)(0x50|r));pc++;return; }
    struct { const char *mn; int base; } io[]={{"REDC",0x30},{"REDD",0x70},{"REDE",0x54},{"WRTC",0xB0},{"WRTD",0xF0},{"WRTE",0xD4},{NULL,0}};
    for(int i=0;io[i].mn;i++){ if(strcmp(mn,io[i].mn)==0){ int r=reg_val(ops[0]); if(r<0){fprintf(stderr,"ERROR line %d: %s needs Rn\n",lineno,mn);errors++;return;} emit(pc,(unsigned char)(io[i].base|r));pc++;return; } }
    struct { const char *mn; int base_r; int base_a; int uses_cc; } br[]={{"BCTR",0x18,0x1C,1},{"BCFR",0x98,0x9C,1},{"BSTR",0x38,0x3C,1},{"BSFR",0xB8,0xBC,1},{"BRNR",0x58,0x5C,0},{"BIRR",0xD8,0xDC,0},{"BDRR",0xF8,0xFC,0},{"BSNR",0x78,0x7C,0},{NULL,0,0,0}};
    #define PARSE_FIELD(ops, nops, field_str, addr_out) do { if((nops)>1 && (ops)[1][0]) { (field_str)=(ops)[0]; (addr_out)=(ops)[1]; } else { char *_p=(ops)[0]; while(*_p && *_p!=' ' && *_p!='\t') _p++; static char _fbuf[8]; int _fl=(int)(_p-(ops)[0]); if(_fl>7)_fl=7; strncpy(_fbuf,(ops)[0],_fl); _fbuf[_fl]=0; (field_str)=_fbuf; while(*_p==' '||*_p=='\t') _p++; (addr_out)=_p; } } while(0)
    for(int i=0;br[i].mn;i++){
        int blen=strlen(br[i].mn);
        if(strncmp(mn,br[i].mn,blen)==0){
            char *suf=mn+blen; int is_abs=(strcmp(suf,"A")==0); int is_rel=(strcmp(suf,"R")==0||strcmp(suf,"")==0);
            if(!is_abs&&!is_rel) break;
            char *field_str, *addr_s; PARSE_FIELD(ops, nops, field_str, addr_s);
            int field;
            if(br[i].uses_cc){ field=cc_val(field_str); if(field<0){fprintf(stderr,"ERROR line %d: %s needs EQ/GT/LT/UN\n",lineno,mn);errors++;return;} }
            else { field=reg_val(field_str); if(field<0){fprintf(stderr,"ERROR line %d: %s needs Rn\n",lineno,mn);errors++;return;} }
            int ind=0; if(*addr_s=='*'){ind=1;addr_s++;} int ok,v=eval_expr(addr_s,&ok);
            if(!ok&&pass==2){fprintf(stderr,"ERROR line %d: bad %s operand '%s'\n",lineno,mn,addr_s); errors++;}
            if(is_rel){ emit(pc,(unsigned char)(br[i].base_r|field));pc++; if(ok) emit_rel(v,ind); else{emit(pc,0);pc++;} }
            else {
                if(pass==2 && warn_local_abs_branch && ok){
                    int off=0;
                    if(rel_offset_if_possible(v,pc+1,&off)){
                        fprintf(stderr,"WARN line %d: %s can use relative form (offset %d)\n",lineno,mn,off);
                    }
                }
                emit(pc,(unsigned char)(br[i].base_a|field));pc++;
                if(ok) emit_abs(v,ind,0); else{emit(pc,0);pc++;emit(pc,0);pc++;}
            }
            return;
        }
    }
    struct { const char *mn; int base; } bra[]={{"BCTA",0x1C},{"BCFA",0x9C},{"BSTA",0x3C},{"BSFA",0xBC},{NULL,0}};
    for(int i=0;bra[i].mn;i++){
        if(strcmp(mn,bra[i].mn)==0){
            char *cc_s, *addr_s; PARSE_FIELD(ops, nops, cc_s, addr_s);
            int cc=cc_val(cc_s); if(cc<0){fprintf(stderr,"ERROR line %d: %s needs EQ/GT/LT/UN\n",lineno,mn);errors++;return;}
            int ind=0; if(*addr_s=='*'){ind=1;addr_s++;} int ok,v=eval_expr(addr_s,&ok);
            if(!ok&&pass==2){fprintf(stderr,"ERROR line %d: bad %s operand '%s'\n",lineno,mn,addr_s); errors++;}
            if(pass==2 && warn_local_abs_branch && ok){
                int off=0;
                if(rel_offset_if_possible(v,pc+1,&off)){
                    fprintf(stderr,"WARN line %d: %s can use relative form (offset %d)\n",lineno,mn,off);
                }
            }
            emit(pc,(unsigned char)(bra[i].base|cc));pc++;
            if(ok) emit_abs(v,ind,0); else{emit(pc,0);pc++;emit(pc,0);pc++;}
            return;
        }
    }
    if(strcmp(mn,"BRNA")==0||strcmp(mn,"BIRA")==0||strcmp(mn,"BDRA")==0||strcmp(mn,"BSNA")==0){
        int base=(strcmp(mn,"BRNA")==0)?0x5C:(strcmp(mn,"BIRA")==0)?0xDC:(strcmp(mn,"BDRA")==0)?0xFC:0x7C;
        int r=reg_val(ops[0]); if(r<0){fprintf(stderr,"ERROR line %d: %s needs Rn\n",lineno,mn);errors++;return;}
        char *addr_s=ops[1]; int ind=0; if(*addr_s=='*'){ind=1;addr_s++;} int ok,v=eval_expr(addr_s,&ok);
        if(!ok&&pass==2){fprintf(stderr,"ERROR line %d: bad %s operand '%s'\n",lineno,mn,addr_s); errors++;}
        if(pass==2 && warn_local_abs_branch && ok){
            int off=0;
            if(rel_offset_if_possible(v,pc+1,&off)){
                fprintf(stderr,"WARN line %d: %s can use relative form (offset %d)\n",lineno,mn,off);
            }
        }
        emit(pc,(unsigned char)(base|r));pc++; if(ok) emit_abs(v,ind,0); else{emit(pc,0);pc++;emit(pc,0);pc++;} return;
    }
    /*
     * BXA/BSXA are non-orthogonal: index register is fixed to R3 in hardware.
     * Accept optional explicit R3 and warn when omitted.
     * Reject R0-R2 and reject autoincrement/decrement suffixes.
    */
    if(strcmp(mn,"BXA")==0||strcmp(mn,"BSXA")==0){
        int ind=0, ok=0, v=0;
        int is_bsx = (strcmp(mn,"BSXA")==0);
        char *addr_s = ops[0];
        char *reg_s = NULL;

        if(nops>=2 && ops[1][0]) {
            if(reg_val(ops[0])>=0){ reg_s = ops[0]; addr_s = ops[1]; } /* BXA R3,ADDR */
            else { addr_s = ops[0]; reg_s = ops[1]; }                   /* BXA ADDR,R3 */
        }

        if(reg_s){
            int r = -1;
            if(reg_s[0]=='R' && reg_s[1]>='0' && reg_s[1]<='3') r = reg_s[1]-'0';
            if(r < 0){ fprintf(stderr,"ERROR line %d: %s register must be R3\n",lineno,mn); errors++; return; }
            if(reg_s[2]=='+' || reg_s[2]=='-'){
                fprintf(stderr,"ERROR line %d: %s does not support auto +/- on R3\n",lineno,mn);
                errors++; return;
            }
            if(r != 3){ fprintf(stderr,"ERROR line %d: %s only supports R3\n",lineno,mn); errors++; return; }
        } else if(pass==2) {
            fprintf(stderr,"WARN line %d: %s register omitted, defaulting to R3\n",lineno,mn);
        }

        if(*addr_s=='*'){ind=1;addr_s++;}
        v=eval_expr(addr_s,&ok);
        if(!ok&&pass==2){fprintf(stderr,"ERROR line %d: bad %s operand '%s'\n",lineno,mn,addr_s); errors++;}
        emit(pc,is_bsx?0xBF:0x9F);pc++;
        if(ok) emit_abs(v,ind,0); else{emit(pc,0);pc++;emit(pc,0);pc++;}
        return;
    }
    /* 2650 silicon constraints on Z-mode register-to-register instructions */
    if(strcmp(mn,"ANDZ")==0&&nops>=1){
        int r=reg_val(ops[0]);
        if(r==0){
            if(pass==2) fprintf(stderr,"WARN line %d: ANDZ,R0 replaced with HALT ($40)\n",lineno);
            emit(pc,0x40); pc++; return;
        }
    }
    if(strcmp(mn,"STRZ")==0&&nops>=1){
        int r=reg_val(ops[0]);
        if(r==0){
            if(pass==2) fprintf(stderr,"WARN line %d: STRZ,R0 replaced with NOP ($C0)\n",lineno);
            emit(pc,0xC0); pc++; return;
        }
    }
    if(strcmp(mn,"LODZ")==0&&nops>=1){
        int r=reg_val(ops[0]);
        if(r==0){
            if(pass==2) fprintf(stderr,"WARN line %d: LODZ,R0 replaced with $60 (IORZ,R0)\n",lineno);
            emit(pc,0x60); pc++; return;
        }
    }
    struct { const char *pfx; int base; int no_imm; } alu[]={{"LOD",0x00,0},{"EOR",0x20,0},{"AND",0x40,0},{"IOR",0x60,0},{"ADD",0x80,0},{"SUB",0xA0,0},{"COM",0xE0,0},{"STR",0xC0,1},{NULL,0,0}};
    for(int i=0;alu[i].pfx;i++){
        int plen=strlen(alu[i].pfx);
        if(strncmp(mn,alu[i].pfx,plen)==0){
            char *suf=mn+plen; int mode=-1;
            if(strcmp(suf,"Z")==0) mode=0; else if(strcmp(suf,"I")==0) mode=1; else if(strcmp(suf,"R")==0) mode=2; else if(strcmp(suf,"A")==0) mode=3; else break;
            if(alu[i].no_imm&&mode==1){fprintf(stderr,"ERROR line %d: STRI not valid\n",lineno);errors++;return;}
            int r=reg_val(ops[0]); if(r<0){fprintf(stderr,"ERROR line %d: %s needs Rn\n",lineno,mn);errors++;return;}
            /* Detect indexed mode BEFORE emitting opcode byte.
             * Per 2650 manual and asm2650.py: in indexed absolute mode the register
             * field in the opcode byte = the INDEX register, NOT dest (R0 implied).
             * LODA,R0 ADDR,R2 -> opcode $0E (R2 in field), not $0C (R0 in field). */
            int idxctl=0;
            char *addr_s=ops0_after_reg(ops[0]);
            if(nops>=2 && ops[1][0]=='R' && ops[1][1]>='0' && ops[1][1]<='3') {
                /* BUG-ASM-08: register-indexed addressing (,Rn[+/-]) is only
                 * architecturally valid in mode A (absolute). Detecting it for
                 * Z/I/R modes previously clobbered the register field silently
                 * (r got overwritten with the index register) and the index
                 * info was then discarded by the mode 0/1/2 emission cases. */
                if(mode!=3){
                    if(pass==2){ fprintf(stderr,"ERROR line %d: %s does not support indexed addressing (,Rn) — only A-mode does\n",lineno,mn); errors++; }
                    return;
                }
                /* BUG-ASM-18: the index register takes the opcode's register
                 * field, so the target is implicitly R0. Any other written
                 * target cannot be encoded -> hard error (pass 2). Encoding
                 * continues below so pc stays aligned across passes. */
                if(r!=0 && pass==2){
                    fprintf(stderr,"ERROR line %d: %s,R%d with indexed addressing (,R%c%s) — target must be R0 (index register replaces the register field)\n",
                            lineno,mn,r,ops[1][1],ops[1][2]=='+'?"+":ops[1][2]=='-'?"-":"");
                    errors++;
                }
                r=ops[1][1]-'0';  /* register field = index register */
                if     (ops[1][2]=='+') idxctl=1;
                else if(ops[1][2]=='-') idxctl=2;
                else                     idxctl=3;
                addr_s=ops0_after_reg(ops[0]);
                if(!addr_s[0] && nops>=3) addr_s=ops[2];
            } else if(nops>=2 && ops[1][0]) {
                addr_s=ops[1];
            }
            /* Emit opcode with correct register field (index reg if indexed) */
            unsigned char ob=(unsigned char)(alu[i].base+(mode<<2)+r); emit(pc,ob); pc++;
            int ind=0; if(*addr_s=='*'){ind=1;addr_s++;} int ok,v=eval_expr(addr_s,&ok);
            /* BUG-ASM-09: eval_expr's failure path was silently ignored for
             * modes I/R/A (mode Z has no address operand). This one check
             * covers all three cases below. */
            if(mode!=0 && !ok && pass==2){ fprintf(stderr,"ERROR line %d: bad %s operand '%s'\n",lineno,mn,addr_s); errors++; }
            switch(mode){
                case 0: break;
                case 1: emit(pc,(unsigned char)(v&0xFF));pc++; break;
                case 2: if(ok) emit_rel(v,ind); else{emit(pc,0);pc++;} break;
                case 3: if(ok) emit_abs(v,ind,idxctl); else{emit(pc,0);pc++;emit(pc,0);pc++;} break;
            }
            return;
        }
    }
    if(pass==2){ fprintf(stderr,"ERROR line %d: unknown mnemonic '%s'\n",lineno,mn); errors++; }
}

/* ---------------------------------------------------------------------------
 * Branch-skip warning: three small line-oriented text matchers.
 * These work on raw, unprocessed source lines (their own upcased local copy)
 * and are independent of pc/label state — they only ever run from main()'s
 * pass-2 read loop, never from assemble_line(). See the v1.16->v1.17 header
 * changelog entry for the pattern being detected and its known limits.
 * ------------------------------------------------------------------------- */

/* match_skip_branch: does 'raw' look like "BCTR,cc TARGET" / "BCTA,cc TARGET"
 * with cc one of EQ/GT/LT (i.e. NOT UN)?
 * Inputs:  raw       = one raw source line, unmodified.
 *          target_sz = capacity of target_out.
 * Outputs: *is_abs_out = 0 for BCTR, 1 for BCTA (valid only if returns 1).
 *          *cc_out     = 0=EQ, 1=GT, 2=LT (valid only if returns 1).
 *          target_out  = the branch's target token, upcased, NUL-terminated.
 *          return 1 on match, 0 otherwise.
 * Clobbers: none (local buffer only). */
static int match_skip_branch(const char *raw, int *is_abs_out, int *cc_out, char *target_out, size_t target_sz){
    char buf[MAX_LINE]; strncpy(buf,raw,sizeof(buf)-1); buf[sizeof(buf)-1]=0;
    upcase(buf);
    char *p=skip_ws(buf);
    int is_abs;
    if(strncmp(p,"BCTR",4)==0) is_abs=0;
    else if(strncmp(p,"BCTA",4)==0) is_abs=1;
    else return 0;
    p+=4;
    if(*p!=',') return 0;
    p=skip_ws(p+1);
    static const char *cc_names[3]={"EQ","GT","LT"};
    int cc=-1;
    for(int i=0;i<3;i++){
        int l=(int)strlen(cc_names[i]);
        if(strncmp(p,cc_names[i],l)==0 && !isalnum((unsigned char)p[l])){ cc=i; p+=l; break; }
    }
    if(cc<0) return 0;   /* UN, or not a recognised cc -> not a skip-branch candidate */
    p=skip_ws(p);
    if(*p==',') p=skip_ws(p+1);
    if(!*p) return 0;
    size_t i=0;
    while(*p && !isspace((unsigned char)*p) && *p!=';' && i<target_sz-1) target_out[i++]=*p++;
    target_out[i]=0;
    if(!i) return 0;
    *is_abs_out=is_abs; *cc_out=cc;
    return 1;
}

/* match_uncond_branch: does 'raw' look like "BCTR,UN TARGET" / "BCTA,UN TARGET"?
 * Inputs:  raw = one raw source line, unmodified. target_sz = capacity of target_out.
 * Outputs: target_out = the branch's target token, upcased, NUL-terminated
 *          (valid only if returns 1). return 1 on match, 0 otherwise.
 * Clobbers: none. */
static int match_uncond_branch(const char *raw, char *target_out, size_t target_sz){
    char buf[MAX_LINE]; strncpy(buf,raw,sizeof(buf)-1); buf[sizeof(buf)-1]=0;
    upcase(buf);
    char *p=skip_ws(buf);
    if(strncmp(p,"BCTR",4)!=0 && strncmp(p,"BCTA",4)!=0) return 0;
    p+=4;
    if(*p!=',') return 0;
    p=skip_ws(p+1);
    if(strncmp(p,"UN",2)!=0 || isalnum((unsigned char)p[2])) return 0;
    p=skip_ws(p+2);
    if(*p==',') p=skip_ws(p+1);
    if(!*p) return 0;
    size_t i=0;
    while(*p && !isspace((unsigned char)*p) && *p!=';' && i<target_sz-1) target_out[i++]=*p++;
    target_out[i]=0;
    return i>0;
}

/* line_defines_label: does 'raw' begin with "NAME:" (colon-terminated label
 * definition), where NAME matches 'want' (already upcased) case-insensitively?
 * Inputs:  raw = one raw source line, unmodified. want = upcased label name to match.
 * Outputs: return 1 if raw defines that label at column-start, 0 otherwise.
 * Clobbers: none. */
static int line_defines_label(const char *raw, const char *want){
    char buf[MAX_LINE]; strncpy(buf,raw,sizeof(buf)-1); buf[sizeof(buf)-1]=0;
    upcase(buf);
    char *p=skip_ws(buf);
    char lbl[64]; size_t i=0;
    while((isalnum((unsigned char)*p)||*p=='_') && i<sizeof(lbl)-1) lbl[i++]=*p++;
    lbl[i]=0;
    if(!i || *p!=':') return 0;
    return strcmp(lbl,want)==0;
}

static void write_hex(FILE *f){
    if(rom_hi<rom_lo){ fprintf(f,":00000001FF\n"); return; }
    int addr=rom_lo;
    while(addr<=rom_hi){
        if(!rom_emitted[addr]){ addr++; continue; }              /* skip un-emitted gap bytes */
        int n=0;
        while(n<16 && addr+n<=rom_hi && rom_emitted[addr+n]) n++; /* contiguous emitted run, max 16 */
        unsigned char sum=(unsigned char)(n+(addr>>8)+(addr&0xFF));
        fprintf(f,":%02X%04X00",n,addr);
        for(int i=0;i<n;i++){ fprintf(f,"%02X",rom[addr+i]); sum+=rom[addr+i]; }
        fprintf(f,"%02X\n",(unsigned char)(-sum));
        addr+=n;
    }
    fprintf(f,":00000001FF\n");
}

static void write_binary(FILE *f, int lo, int hi){
    if(lo<0) lo=0;
    if(hi>=MAX_ROM) hi=MAX_ROM-1;
    if(hi<lo) return;
    fwrite(&rom[lo],1,(size_t)(hi-lo+1),f);
}

static char *xstrdup(const char *s){
    size_t n=strlen(s)+1;
    char *p=(char *)malloc(n);
    if(p) memcpy(p,s,n);
    return p;
}

static char *list_path_for_source(const char *src_file){
    const char *slash1=strrchr(src_file,'/');
    const char *slash2=strrchr(src_file,'\\');
    const char *slash=slash1;
    if(!slash || (slash2 && slash2>slash)) slash=slash2;
    const char *base=slash?slash+1:src_file;
    const char *dot=strrchr(base,'.');
    size_t stem_len=dot ? (size_t)(dot-src_file) : strlen(src_file);
    char *path=(char *)malloc(stem_len+5);
    if(!path) return NULL;
    memcpy(path,src_file,stem_len);
    memcpy(path+stem_len,".LST",5);
    return path;
}

static void list_begin_line(void){
    list_line_addr=-1;
    list_line_nbytes=0;
}

static int list_add_line(int src_lineno, const char *source){
    ListLine *ll;
    if(nlist_lines>=list_cap){
        int new_cap=list_cap?list_cap*2:256;
        ListLine *new_lines=(ListLine *)realloc(list_lines,(size_t)new_cap*sizeof(*list_lines));
        if(!new_lines){ fprintf(stderr,"ERROR: out of memory storing listing\n"); errors++; return 0; }
        list_lines=new_lines;
        list_cap=new_cap;
    }
    ll=&list_lines[nlist_lines++];
    ll->lineno=src_lineno;
    ll->addr=list_line_addr;
    ll->nbytes=list_line_nbytes;
    ll->bytes=NULL;
    ll->source=xstrdup(source);
    if(!ll->source){ fprintf(stderr,"ERROR: out of memory storing listing source\n"); errors++; return 0; }
    if(list_line_nbytes>0){
        ll->bytes=(unsigned char *)malloc((size_t)list_line_nbytes);
        if(!ll->bytes){ fprintf(stderr,"ERROR: out of memory storing listing bytes\n"); errors++; return 0; }
        memcpy(ll->bytes,list_line_bytes,(size_t)list_line_nbytes);
    }
    return 1;
}

static int write_listing(const char *src_file){
    char *lst_file=list_path_for_source(src_file);
    FILE *f;
    if(!lst_file){ fprintf(stderr,"ERROR: out of memory creating list filename\n"); return 0; }
    f=fopen(lst_file,"w");
    if(!f){ fprintf(stderr,"Cannot create '%s'\n",lst_file); free(lst_file); return 0; }

    fprintf(f,"asm2650 v%s listing for %s\n\n",ASM2650_VERSION,src_file);
    fprintf(f,"Line  Addr   Opcodes                  Source\n");
    fprintf(f,"----  -----  -----------------------  ------\n");
    for(int i=0;i<nlist_lines;i++){
        ListLine *ll=&list_lines[i];
        if(ll->nbytes<=0){
            fprintf(f,"%4d         %-23s  %s\n",ll->lineno,"",ll->source?ll->source:"");
            continue;
        }
        int offset=0;
        while(offset<ll->nbytes){
            int chunk=ll->nbytes-offset;
            if(chunk>8) chunk=8;
            char opbuf[3*8+1];
            int pos=0;
            for(int j=0;j<chunk;j++) pos+=sprintf(opbuf+pos,"%02X%s",ll->bytes[offset+j],j==chunk-1?"":" ");
            if(offset==0){
                fprintf(f,"%4d  $%04X  %-23s  %s\n",ll->lineno,ll->addr+offset,opbuf,ll->source?ll->source:"");
            } else {
                fprintf(f,"%4s  $%04X  %-23s\n","",ll->addr+offset,opbuf);
            }
            offset+=chunk;
        }
    }

    fprintf(f,"\nLabels:\n");
    fprintf(f,"Name                             Value  Status\n");
    fprintf(f,"-------------------------------  -----  ------\n");
    for(int i=0;i<nlabels;i++){
        fprintf(f,"%-31s  $%04X  %s\n",labels[i].name,labels[i].value,labels[i].referenced?"USED":"UNUSED");
    }

    if(fclose(f)!=0){ fprintf(stderr,"Cannot finish writing '%s'\n",lst_file); free(lst_file); return 0; }
    fprintf(stderr,"List: %s\n",lst_file);
    free(lst_file);
    return 1;
}

static void free_listing(void){
    for(int i=0;i<nlist_lines;i++){
        free(list_lines[i].bytes);
        free(list_lines[i].source);
    }
    free(list_lines);
    list_lines=NULL;
    nlist_lines=0;
    list_cap=0;
}

static void print_usage(FILE *f){
    fprintf(f,"asm2650 v%s - Signetics 2650 cross-assembler\n", ASM2650_VERSION);
    fprintf(f,"Usage: asm2650 [options] source.asm [output.hex]\n");
    fprintf(f,"  Hex goes to stdout unless output.hex is given; <source>.LST is written beside the source.\n");
    fprintf(f,"  Exit status: 0 = assembled, 1 = errors (hex/binary output withheld).\n");
    fprintf(f,"\nOutput options:\n");
    fprintf(f,"  -s                             Dump symbol table to stderr\n");
    fprintf(f,"  --binary                       Write flat 32768-byte binary image to stdout\n");
    fprintf(f,"  -o <file>                      Write binary image to <file> (implies --binary)\n");
    fprintf(f,"  -r $HHHH-$HHHH                 Limit binary output address range (inclusive; needs --binary/-o)\n");
    fprintf(f,"  -NoList                        Suppress default .LST listing sidecar\n");
    fprintf(f,"  -h, --help                     Show this help and exit\n");
    fprintf(f,"\nWarning options:\n");
    fprintf(f,"  --warn=LIST                    Enable named warnings/hints (comma-separated names, see below)\n");
    fprintf(f,"  --no-warn=LIST                 Disable named warnings/hints\n");
    fprintf(f,"  --no-warn-inline-label         Same as --no-warn=label\n");
    fprintf(f,"  --no-warn-local-branch         Same as --no-warn=rel\n");
    fprintf(f,"  --no-warn-tail-call            Same as --no-warn=tail\n");
    fprintf(f,"  --no-warn-branch-skip          Same as --no-warn=skip\n");
    fprintf(f,"\nWarning names (all on by default except the advisory hints):\n");
    fprintf(f,"  label     LABEL: INSTR on one line\n");
    fprintf(f,"  rel       absolute branch that could be relative\n");
    fprintf(f,"  skip      BCTR/BCTA,cc skipping an unconditional branch (use BCF)\n");
    fprintf(f,"  tail      BSTx,UN / ZBSR / BSXA followed by RETC,UN (use BCTx / ZBRR / BXA)\n");
    fprintf(f,"Peephole hints (default on):\n");
    fprintf(f,"  clear     LODI,R0 0 / ANDI,R0 0 -> EORZ,R0\n");
    fprintf(f,"  test      IORI,R0 0 / ANDI,R0 $FF / EORI,R0 0 -> IORZ,R0\n");
    fprintf(f,"  zpage     BCTA,UN / BSTA,UN to zero page -> ZBRR / ZBSR\n");
    fprintf(f,"  next      branch to the next instruction -> delete\n");
    fprintf(f,"  psw       CPSL a / CPSL b (also CPSU, PPSL, PPSU) -> one instruction\n");
    fprintf(f,"  retcc     BCTx,cc to a RETC,UN -> RETC,cc\n");
    fprintf(f,"  skipbyte  BCTR/BCTA,UN over 1-2 bytes -> DB $E4 / DB $EC skip byte (CC must be dead)\n");
    fprintf(f,"Advisory hints (default off; may need CC/carry/R0 to be dead):\n");
    fprintf(f,"  condtail  conditional call + RETC,UN -> conditional jump, RETC,UN kept\n");
    fprintf(f,"  loop      SUBI/ADDI,Rn 1 + BCFx,EQ -> BDRx/BIRx,Rn\n");
    fprintf(f,"  brn       COMI,Rn 0 / LODZ,Rn / IORZ,R0 + BCFx,EQ -> BRNx,Rn\n");
    fprintf(f,"  thunk     branch/call to a jump thunk -> branch/call the final target\n");
    fprintf(f,"  dead      unlabelled code after an unconditional transfer\n");
    fprintf(f,"Groups: advisory (the 5 advisory hints), peephole (all 12 hints), all (everything)\n");
    fprintf(f,"\nExamples:\n");
    fprintf(f,"  asm2650 prog.asm prog.hex\n");
    fprintf(f,"  asm2650 --warn=advisory prog.asm\n");
    fprintf(f,"  asm2650 --warn=all --no-warn=dead prog.asm\n");
}

static int parse_range(const char *s, int *lo, int *hi){
    unsigned int a,b;
    if(sscanf(s,"$%x-$%x",&a,&b)!=2) return 0;
    if(a>=MAX_ROM || b>=MAX_ROM || a>b) return 0;
    *lo=(int)a; *hi=(int)b;
    return 1;
}

int dump_syms=0;
int main(int argc,char *argv[]){
    const char *src_file=NULL;
    const char *hex_file=NULL;
    const char *bin_file=NULL;
    int binary_mode=0;
    int range_set=0, range_lo=0, range_hi=MAX_ROM-1;

    for(int i=1;i<argc;i++){
        if(!strcmp(argv[i],"-s")) dump_syms=1;
        else if(!strcmp(argv[i],"--binary")) binary_mode=1;
        else if(!strcmp(argv[i],"-NoList")) list_enabled=0;
        else if(!strcmp(argv[i],"-o")){
            if(i+1>=argc){ fprintf(stderr,"ERROR: -o requires a file path\n"); return 1; }
            bin_file=argv[++i];
            binary_mode=1;
        } else if(!strcmp(argv[i],"-r")){
            if(i+1>=argc){ fprintf(stderr,"ERROR: -r requires a range like $0000-$00FF\n"); return 1; }
            if(!parse_range(argv[++i],&range_lo,&range_hi)){
                fprintf(stderr,"ERROR: invalid range '%s' (expected $HHHH-$HHHH within $0000-$7FFF)\n",argv[i]);
                return 1;
            }
            range_set=1;
        } else if(!strcmp(argv[i],"--no-warn-inline-label")) warn_inline_label=0;
        else if(!strcmp(argv[i],"--no-warn-local-branch")) warn_local_abs_branch=0;
        else if(!strcmp(argv[i],"--no-warn-branch-skip")) warn_branch_skip=0;
        else if(!strcmp(argv[i],"--no-warn-tail-call")) warn_tail_call=0;
        else if(!strncmp(argv[i],"--warn=",7)){ if(!ph_set_list(argv[i]+7,1)) return 1; }
        else if(!strncmp(argv[i],"--no-warn=",10)){ if(!ph_set_list(argv[i]+10,0)) return 1; }
        else if(!strcmp(argv[i],"-h") || !strcmp(argv[i],"--help")){
            print_usage(stdout);
            return 0;
        } else if(argv[i][0]=='-'){
            fprintf(stderr,"ERROR: unknown option '%s'\n",argv[i]);
            print_usage(stderr);
            return 1;
        } else if(!src_file) src_file=argv[i];
        else if(!hex_file) hex_file=argv[i];
        else { fprintf(stderr,"ERROR: unexpected argument '%s'\n",argv[i]); return 1; }
    }

    if(!src_file){ print_usage(stderr); return 1; }
    if(range_set && !binary_mode){
        fprintf(stderr,"ERROR: -r requires --binary or -o\n");
        return 1;
    }

    memset(rom,0xFF,sizeof(rom));
    for(pass=1;pass<=2;pass++){
        if(pass==2) memset(rom_emitted,0,sizeof(rom_emitted));
        FILE *f=fopen(src_file,"r"); if(!f){fprintf(stderr,"Cannot open '%s'\n",src_file);return 1;}
        pc=0; lineno=0; char line[MAX_LINE];
        tailcall_reset();
        /* Rolling 3-line window for the branch-skip warning (match_skip_branch /
         * match_uncond_branch / line_defines_label above). Fresh each pass since
         * these are ordinary locals re-created every time this for-loop body
         * runs; only ever acted on during pass 2 (below), so pass 1 leaves them
         * unused. */
        int  bsw_pending=0, bsw_is_abs=0, bsw_cc=0, bsw_lineno=0, bsw_saw_uncond=0;
        char bsw_target1[128]="", bsw_target2[128]="";
        while(fgets(line,MAX_LINE,f)){
            lineno++;
            int l=strlen(line);
            while(l>0&&(line[l-1]=='\r'||line[l-1]=='\n')) line[--l]=0;
            if(pass==2 && warn_branch_skip){
                static const char *bsw_cc_name[3]={"EQ","GT","LT"};
                if(bsw_pending && bsw_saw_uncond && line_defines_label(line,bsw_target1)){
                    fprintf(stderr,
                        "WARN line %d: %s,%s %s skips an unconditional branch (to %s) to reach %s: on line %d"
                        " -- consider BCF%s,%s %s instead, dropping the unconditional branch\n",
                        bsw_lineno, bsw_is_abs?"BCTA":"BCTR", bsw_cc_name[bsw_cc], bsw_target1,
                        bsw_target2, bsw_target1, lineno,
                        bsw_is_abs?"A":"R", bsw_cc_name[bsw_cc], bsw_target2);
                    bsw_pending=0; bsw_saw_uncond=0;
                } else if(bsw_pending && !bsw_saw_uncond && match_uncond_branch(line,bsw_target2,sizeof(bsw_target2))){
                    bsw_saw_uncond=1;
                } else {
                    bsw_pending=match_skip_branch(line,&bsw_is_abs,&bsw_cc,bsw_target1,sizeof(bsw_target1));
                    bsw_lineno=lineno;
                    bsw_saw_uncond=0;
                }
            }
            if(pass==2 && list_enabled) list_begin_line();
            assemble_line(line);
            if(pass==2) peep_line_done();
            if(pass==2 && list_enabled) list_add_line(lineno,line);
        }
        fclose(f);
    }
    peep_report();
    fprintf(stderr,"Pass complete: %d error(s), %d label(s)\n",errors,nlabels);
    if(dump_syms){ for(int i=0;i<nlabels;i++) fprintf(stderr,"  %-20s $%04X\n",labels[i].name,labels[i].value); }
    if(rom_hi>=rom_lo) fprintf(stderr,"Code: $%04X-$%04X (%d bytes)\n",rom_lo,rom_hi,rom_hi-rom_lo+1);
    /* Write the .LST sidecar even on error -- it's a debugging aid, and an ORG-overwrite
     * or other error is often easiest to diagnose by reading the listing itself. Only the
     * hex/binary machine-code output is withheld on error, since writing it out would
     * present unreliable bytes (overlapping emits, partial assembly) as a finished artifact. */
    if(list_enabled && !write_listing(src_file)){ free_listing(); return 1; }
    if(errors){ free_listing(); return 1; }
    FILE *out=stdout;
    if(binary_mode){
        if(bin_file){
            out=fopen(bin_file,"wb");
            if(!out){fprintf(stderr,"Cannot create '%s'\n",bin_file);return 1;}
        }
        write_binary(out, range_set?range_lo:0, range_set?range_hi:(MAX_ROM-1));
    } else {
        if(hex_file&&!dump_syms){ out=fopen(hex_file,"w"); if(!out){fprintf(stderr,"Cannot create '%s'\n",hex_file);return 1;} }
        write_hex(out);
    }
    if(out!=stdout) fclose(out);
    free_listing();
    return 0;
}
