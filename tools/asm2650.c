/* ============================================================================
 * asm2650.c  —  Signetics 2650 cross-assembler
 * Version: 1.16
 * Build: gcc -Wall -O2 -o asm2650 asm2650.c
 *
 * Usage: asm2650 source.asm [output.hex]   (stdout if no output file)
 *        asm2650 source.asm -s             (dump symbol table to stderr)
 *        asm2650 source.asm -NoList        (suppress default .LST file)
 *
 * Supported assembler directives:
 *   ORG, EQU, DS, RES, DB, DW, END
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

#define MAX_LABELS  512
#define MAX_LINE    256
#define MAX_ROM   32768
#define UNDEF      (-1)
#define ASM2650_VERSION "1.16"

typedef struct { char name[64]; int value; int referenced; int def_line; } Label;
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
    if(i>=0) labels[i].referenced=1;
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
    labels[nlabels].value=v; labels[nlabels].referenced=0; labels[nlabels].def_line=lineno; nlabels++;
}

static int eval_expr(char *s, int *ok){
    s=skip_ws(s); *ok=1;
    int neg=0;
    if(*s=='-'){ neg=1; s++; s=skip_ws(s); }
    /* HI/LO operators — Signetics/WinArcadia/asm2650.py standard:
     *   <ADDR = HIGH byte  (asm2650.py UPPER: value >>= 8)
     *   >ADDR = LOW  byte  (asm2650.py LOWER: value &= 0xFF)
     * WinArcadia docs: "<FOO for the high byte, >FOO for the low byte" */
    if(*s=='<'||*s=='>'){
        int hi=(*s=='<'); s++;
        int ok2=0; int v=eval_expr(s,&ok2);
        if(!ok2){ *ok=0; return 0; }
        return hi?((v>>8)&0xFF):(v&0xFF);
    }
    int val=0;
    if(*s=='$'){ s++;
        if(!isxdigit((unsigned char)*s)){
            /* BUG-ASM-13: bare '$' (not followed by a hex digit) = address of
             * the current source line — the classic assembler "here" token.
             * E.g. "BDRR,R0 $" decrements R0 and branches back to itself: a
             * busy-wait delay loop timed by the initial R0 value. */
            val = line_start_pc;
        } else {
            while(isxdigit((unsigned char)*s)) val=val*16+(isdigit((unsigned char)*s)?*s-'0':toupper((unsigned char)*s)-'A'+10), s++;
        }
    }
    else if(*s=='%'){ s++; while(*s=='0'||*s=='1') val=val*2+(*s++-'0'); }
    else if(isdigit((unsigned char)*s)){ while(isdigit((unsigned char)*s)) val=val*10+(*s++-'0'); }
    else if(isalpha((unsigned char)*s)||*s=='_'){
        /* A'x' — Signetics ASCII character literal (asm2650.py ASCII token) */
        if(toupper((unsigned char)*s)=='A' && *(s+1)=='\''){
            s+=2; /* skip A' */
            val=(unsigned char)*s;
            if(*s) s++;
            if(*s=='\'') s++; /* skip closing ' */
        } else {
            char nm[64]; int i=0;
            while((isalnum((unsigned char)*s)||*s=='_')&&i<63) nm[i++]=*s++;
            nm[i]=0;
            if(isalnum((unsigned char)*s)||*s=='_'){
                if(pass==2){ fprintf(stderr,"ERROR line %d: label name too long (max 63 chars) near '%s'\n",lineno,nm); errors++; }
                *ok=0; return 0;
            }
            int lv=label_find(nm);
            if(lv==UNDEF){ if(pass==2){fprintf(stderr,"ERROR line %d: undefined '%s'\n",lineno,nm); errors++;} *ok=0; return 0; }
            if(pass==2) label_mark_referenced(nm);
            val=lv;
        }
    } else if(*s=='\''){
        s++; val = (unsigned char)*s;
        if(*s) s++;
        if(*s=='\'') s++;
    } else { *ok=0; return 0; }
    if(neg) val=-val;
    s=skip_ws(s);
    if(*s=='+'||*s=='-'){
        int sub=(*s=='-'); s++; s=skip_ws(s);
        int ok2; int rhs=eval_expr(s,&ok2);
        if(!ok2){ *ok=0; return 0; }
        val=sub?val-rhs:val+rhs;
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

static void assemble_line(char *line){
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
    /* v1.4 FIX: allow "LABEL: OPCODE operands" on one line.
     * After defining the label, continue to assemble any instruction that follows.
     * A colon with nothing after it (label-only line) is handled by the !*p check. */
    if(!*p) return;
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
                    if(rel_offset_if_possible(v,pc,&off)){
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
                if(rel_offset_if_possible(v,pc,&off)){
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
            if(rel_offset_if_possible(v,pc,&off)){
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
    fprintf(f,"Options:\n");
    fprintf(f,"  -s                             Dump symbol table to stderr\n");
    fprintf(f,"  --binary                       Write flat 32768-byte binary image\n");
    fprintf(f,"  -o <file>                      Write binary image to <file>\n");
    fprintf(f,"  -r $HHHH-$HHHH                 Limit binary output address range (inclusive)\n");
    fprintf(f,"  -NoList                        Suppress default .LST listing sidecar\n");
    fprintf(f,"  --no-warn-inline-label         Disable warning for LABEL: INSTR on same line\n");
    fprintf(f,"  --no-warn-local-branch         Disable warning when absolute branch could be relative\n");
    fprintf(f,"  -h, --help                     Show this help and exit\n");
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
        while(fgets(line,MAX_LINE,f)){
            lineno++;
            int l=strlen(line);
            while(l>0&&(line[l-1]=='\r'||line[l-1]=='\n')) line[--l]=0;
            if(pass==2 && list_enabled) list_begin_line();
            assemble_line(line);
            if(pass==2 && list_enabled) list_add_line(lineno,line);
        }
        fclose(f);
    }
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
