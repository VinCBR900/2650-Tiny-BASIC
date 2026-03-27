/* ============================================================================
 * asm2650_v1.3.c  —  Signetics 2650 cross-assembler
 * ============================================================================
 *
 * VERSION HISTORY
 *   v1.0  initial skeleton
 *   v1.1  opcode table corrections
 *   v1.2  character literals, high/low byte extractors, two-pass labels
 *   v1.3  2026-03-27
 *         - Add absolute-indexed addressing syntax:  addr,Rn+  addr,Rn-  addr,Rn
 *           (idxctl bits 6-5 in address byte 1 of 3-byte non-branch instructions)
 *         - Add subroutine/function header comments (In / Out / Clobbers)
 *         - Add architecture and reference document links in header
 *
 * PURPOSE
 *   Two-pass cross-assembler that converts 2650 assembly source into Intel HEX.
 *
 * ARCHITECTURE & REFERENCE LINKS
 *   User Manual:         https://amigan.yatho.com/2650UM.html#main.html
 *   Addressing modes:    https://en.wikibooks.org/wiki/Signetics_2650_%26_2636_programming/2650_processor#Indexed_addressing
 *   Indexed branching:   https://en.wikibooks.org/wiki/Signetics_2650_%26_2636_programming/Indexed_branching
 *   Project arch doc:    docs/ARCHITECTURE.md
 *   Session trace/log:   docs/TRACE_LOG.md
 *
 * HOST / BUILD
 *   ANSI C, tested with gcc/clang on Linux.
 *   Build:  gcc -Wall -O2 -o asm2650 asm2650_v1.3.c
 *
 * USAGE
 *   asm2650 source.asm [output.hex]
 *     source.asm   input assembly source
 *     output.hex   optional Intel HEX output path (stdout if omitted)
 *
 * ASSEMBLER CAPABILITIES
 *   - Two-pass label resolution.
 *   - Directives: ORG, EQU, DB, DW, DS, END.
 *   - Core numeric expressions with +/-.
 *   - Number formats: hex ($), binary (%), decimal.
 *   - Character literals: 'A' style operands.
 *   - High/low byte extractors: >expr and <expr.
 *   - Signetics-style operand syntax (e.g., LODI,R0 $41).
 *   - Absolute indexed addressing: addr,Rn+  addr,Rn-  addr,Rn
 *     (idxctl=01/10/11 in address byte, per 2650 User Manual p.Addressing Modes)
 *
 * INSTRUCTION SET
 *   Opcode encoding follows the 2650 user manual mappings.
 *   Condition suffixes: EQ, GT, LT, UN.
 *   Listing/debug diagnostics are emitted to stderr.
 *
 * INDEXED ADDRESSING (v1.3)
 *   Non-branch absolute instructions (LODA, STRA, etc.) support:
 *     LODA,Rn  addr,Rm+   Rn pre-incremented, EA = addr + Rn  (idxctl=01)
 *     LODA,Rn  addr,Rm-   Rn pre-decremented, EA = addr + Rn  (idxctl=10)
 *     LODA,Rn  addr,Rm    EA = addr + Rn, no modify            (idxctl=11)
 *   The 2650 encodes the index register as the same Rn in bits 1-0 of the
 *   opcode byte.  Only R1, R2, R3 make sense as index registers (R0 is
 *   the implicit accumulator destination for Z-mode ops).
 *   Reference: https://amigan.yatho.com/2650UM.html#main.html Addressing Modes
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define MAX_LABELS  512
#define MAX_LINE    256
#define MAX_ROM   32768
#define UNDEF      (-1)

typedef struct { char name[32]; int value; } Label;
static Label labels[MAX_LABELS];
static int   nlabels = 0;

static unsigned char rom[MAX_ROM];
static int rom_lo = MAX_ROM, rom_hi = -1;

static int  pc     = 0;
static int  pass   = 0;
static int  errors = 0;
static int  lineno = 0;

/* ── utilities ───────────────────────────────────────────────── */

/*
 * upcase — convert string to uppercase in-place
 * In:  s = pointer to NUL-terminated string
 * Out: s modified in-place
 * Clobbers: nothing
 */
static void upcase(char *s){ for(;*s;s++) *s=(char)toupper((unsigned char)*s); }

/*
 * skip_ws — skip leading whitespace
 * In:  s = pointer into string
 * Out: returns pointer to first non-space/tab character
 * Clobbers: nothing
 */
static char *skip_ws(char *s){ while(*s==' '||*s=='\t') s++; return s; }

/*
 * emit — write one byte to the ROM image
 * In:  addr = target address (0..MAX_ROM-1)
 *      b    = byte value
 * Out: rom[addr] = b; rom_lo/rom_hi updated
 * Clobbers: nothing (errors++ on out-of-range in pass 2)
 */
static void emit(int addr, unsigned char b){
    if(addr<0||addr>=MAX_ROM){ if(pass==2){fprintf(stderr,"ERROR line %d: addr $%04X out of range\n",lineno,addr); errors++;} return; }
    rom[addr]=b;
    if(addr<rom_lo) rom_lo=addr;
    if(addr>rom_hi) rom_hi=addr;
}

/*
 * label_find — look up label value
 * In:  n = label name (NUL-terminated)
 * Out: returns value, or UNDEF if not found
 * Clobbers: nothing
 */
static int label_find(const char *n){
    for(int i=0;i<nlabels;i++) if(strcmp(labels[i].name,n)==0) return labels[i].value;
    return UNDEF;
}

/*
 * label_define — define or update a label
 * In:  n = name, v = value
 * Out: label table updated
 * Clobbers: nothing (errors++ if table full)
 */
static void label_define(const char *n, int v){
    for(int i=0;i<nlabels;i++) if(strcmp(labels[i].name,n)==0){ labels[i].value=v; return; }
    if(nlabels>=MAX_LABELS){ fprintf(stderr,"ERROR: label table full\n"); errors++; return; }
    strncpy(labels[nlabels].name,n,31); labels[nlabels].name[31]=0;
    labels[nlabels].value=v; nlabels++;
}

/*
 * eval_expr — evaluate a numeric expression string
 * In:  s  = pointer to expression text (NUL-terminated)
 *      ok = pointer to success flag (set 0 on failure, 1 on success)
 * Out: returns integer value; *ok=0 on unresolved symbol or syntax error
 * Clobbers: nothing
 * Supports: $hex  %binary  decimal  'char'  label  >expr  <expr  +/-
 */
static int eval_expr(char *s, int *ok){
    s=skip_ws(s); *ok=1;
    int neg=0;
    if(*s=='-'){ neg=1; s++; s=skip_ws(s); }
    if(*s=='>'||*s=='<'){
        int hi=(*s=='>');
        s++;
        int ok2=0;
        int v=eval_expr(s,&ok2);
        if(!ok2){ *ok=0; return 0; }
        return hi?((v>>8)&0xFF):(v&0xFF);
    }
    int val=0;
    if(*s=='$'){ s++; if(!isxdigit((unsigned char)*s)){*ok=0;return 0;}
        while(isxdigit((unsigned char)*s)) val=val*16+(isdigit((unsigned char)*s)?*s-'0':toupper((unsigned char)*s)-'A'+10), s++; }
    else if(*s=='%'){ s++; while(*s=='0'||*s=='1') val=val*2+(*s++-'0'); }
    else if(isdigit((unsigned char)*s)){ while(isdigit((unsigned char)*s)) val=val*10+(*s++-'0'); }
    else if(isalpha((unsigned char)*s)||*s=='_'){
        char nm[32]; int i=0;
        while((isalnum((unsigned char)*s)||*s=='_')&&i<31) nm[i++]=*s++;
        nm[i]=0;
        int lv=label_find(nm);
        if(lv==UNDEF){ if(pass==2){fprintf(stderr,"ERROR line %d: undefined '%s'\n",lineno,nm); errors++;} *ok=0; return 0; }
        val=lv;
    } else if(*s=='\''){
        /* character literal: 'A' → 65 */
        s++;
        val = (unsigned char)*s;
        if(*s) s++;
        if(*s=='\'') s++;   /* consume closing quote if present */
    } else { *ok=0; return 0; }
    if(neg) val=-val;
    s=skip_ws(s);
    if(*s=='+'||*s=='-'){ int sub=(*s=='-'); s++; s=skip_ws(s); int ok2; int rhs=eval_expr(s,&ok2); val=sub?val-rhs:val+rhs; }
    return val;
}

/*
 * split_ops — split operand string on commas into an array of strings
 * In:  s      = operand text
 *      ops    = output array [maxops][64]
 *      maxops = maximum number of operand slots
 * Out: ops[] filled; returns count of operands parsed
 * Clobbers: nothing
 */
static int split_ops(char *s, char ops[][64], int maxops){
    int n=0; s=skip_ws(s);
    while(*s&&n<maxops){
        int i=0;
        while(*s&&*s!=','&&*s!=';'&&i<63) ops[n][i++]=*s++;
        ops[n][i]=0;
        for(int j=i-1;j>=0&&(ops[n][j]==' '||ops[n][j]=='\t');j--) ops[n][j]=0;
        n++; if(*s==',') s++; s=skip_ws(s);
    }
    return n;
}

/*
 * cc_val — convert condition code name to 2-bit value
 * In:  s = condition name string ("EQ","GT","LT","UN")
 * Out: returns 0..3, or -1 if unrecognised
 * Clobbers: nothing
 * Encoding: EQ=00 GT=01 LT=10 UN=11 (matches RETC/BCTR/BSTA opcode bit encoding)
 */
static int cc_val(const char *s){
    if(strcmp(s,"EQ")==0) return 0;
    if(strcmp(s,"GT")==0) return 1;
    if(strcmp(s,"LT")==0) return 2;
    if(strcmp(s,"UN")==0) return 3;
    return -1;
}

/*
 * reg_val — convert register name to 0-3
 * In:  s = register name string (e.g. "R0", "R1")
 * Out: returns 0..3, or -1 if not a valid register name
 * Clobbers: nothing
 * Note: Accepts "R0 expr" where expr follows whitespace (Signetics syntax)
 */
static int reg_val(const char *s){
    if(s[0]=='R'&&s[1]>='0'&&s[1]<='3'&&(s[2]==0||s[2]==' '||s[2]=='\t')){
        return s[1]-'0';
    }
    return -1;
}

/*
 * ops0_after_reg — return pointer to the expression that follows a register in ops[0]
 * In:  s = ops[0] string, e.g. "R0  $41"
 * Out: pointer to the "$41" portion (after the register and whitespace)
 * Clobbers: nothing
 */
static char *ops0_after_reg(char *s){
    if(s[0]=='R'&&s[1]>='0'&&s[1]<='3'){
        char *p=s+2;
        while(*p==' '||*p=='\t') p++;
        return p;
    }
    return s;
}

/*
 * emit_rel — emit signed 7-bit relative offset byte
 * In:  target = absolute target address
 *      ind    = indirect flag (1 = set bit 7 of offset byte)
 * Out: one byte emitted at pc; pc incremented
 * Clobbers: errors++ if offset out of range [-64,+63]
 */
static void emit_rel(int target, int ind){
    int off=target-(pc+1);
    if(pass==2&&(off<-64||off>63)){
        fprintf(stderr,"ERROR line %d: relative offset %d out of range [-64,+63]\n",lineno,off);
        errors++;
    }
    emit(pc,(unsigned char)((off&0x7F)|(ind?0x80:0))); pc++;
}

/*
 * emit_abs — emit 13-bit absolute address as 2 bytes (non-branch instructions)
 * In:  addr       = 15-bit target address
 *      ind        = indirect flag (1 = set bit 7 of byte 1)
 *      idxctl     = index control field (bits 6-5 of byte 1), 0..3
 *                   0=none  1=Rn++ then add  2=Rn-- then add  3=add Rn
 * Out: two bytes emitted at pc, pc += 2
 * Clobbers: nothing
 * Byte 1 layout: i(7) idxctl(6-5) addr12-8(4-0)
 * Byte 2 layout: addr7-0
 * Reference: https://amigan.yatho.com/2650UM.html#main.html (Addressing Modes)
 */
static void emit_abs(int addr, int ind, int idxctl){
    unsigned char b1=(unsigned char)(((addr>>8)&0x1F)|((idxctl&3)<<5)|(ind?0x80:0));
    unsigned char b2=(unsigned char)(addr&0xFF);
    emit(pc,b1); pc++;
    emit(pc,b2); pc++;
}

/*
 * emit_abs_br — emit 13-bit absolute address for BRANCH instructions
 * In:  addr = 15-bit target address
 *      ind  = indirect flag
 *      pp   = page bits (bits 6-5 of byte 1)
 * Out: two bytes emitted at pc, pc += 2
 * Clobbers: nothing
 * Byte 1 layout: i(7) pp(6-5) addr12-8(4-0)
 * Byte 2 layout: addr7-0
 */
static void emit_abs_br(int addr, int ind, int pp){
    unsigned char b1=(unsigned char)(((addr>>8)&0x1F)|((pp&3)<<5)|(ind?0x80:0));
    unsigned char b2=(unsigned char)(addr&0xFF);
    emit(pc,b1); pc++;
    emit(pc,b2); pc++;
}

/*
 * parse_abs_addr — parse an absolute address operand that may include indexing
 * In:  s       = pointer to address string, e.g. "$1400"  "$1400,R2+"  "$1400,R2"
 *      ind_out = output: indirect flag (1 if '*' prefix found)
 *      idx_out = output: idxctl value (0=none, 1=Rn+, 2=Rn-, 3=Rn)
 *      val_out = output: evaluated address value
 *      ok_out  = output: 1 if address parsed OK, 0 on error
 * Out: fills ind_out, idx_out, val_out, ok_out
 * Clobbers: nothing
 *
 * Indexed syntax (v1.3):
 *   addr        → idxctl=0  (plain absolute)
 *   addr,Rn+    → idxctl=1  (pre-increment Rn, then EA = addr + Rn)
 *   addr,Rn-    → idxctl=2  (pre-decrement Rn, then EA = addr + Rn)
 *   addr,Rn     → idxctl=3  (EA = addr + Rn, Rn unchanged)
 *
 * Reference: https://amigan.yatho.com/2650UM.html#main.html
 *            https://en.wikibooks.org/wiki/Signetics_2650_%26_2636_programming/2650_processor#Indexed_addressing
 */
static void parse_abs_addr(char *s, int *ind_out, int *idx_out, int *val_out, int *ok_out){
    *ind_out=0; *idx_out=0; *val_out=0; *ok_out=0;
    s=skip_ws(s);
    if(*s=='*'){ *ind_out=1; s++; }

    /* copy base address portion (everything before a ',R' suffix) */
    char base[64]; int bi=0;
    char *p=s;
    /* scan forward to find ",Rn" suffix if present */
    /* The address part may contain expressions like LABEL+1, so we look for
       the pattern ",R[0-3]" optionally followed by '+' or '-' */
    while(*p && bi<63){
        /* check for ",R[0-3]" */
        if(*p==','&&(*(p+1)=='R')&&(*(p+2)>='0'&&*(p+2)<='3')){
            break; /* found index suffix */
        }
        base[bi++]=*p++;
    }
    base[bi]=0;

    /* evaluate the base address */
    int ok=0;
    *val_out=eval_expr(base,&ok);
    *ok_out=ok;

    /* parse optional ",Rn[+-]" index suffix */
    if(*p==','){
        p++; /* skip comma */
        /* expect R[0-3] */
        if(*p=='R'&&*(p+1)>='0'&&*(p+1)<='3'){
            p+=2; /* skip "Rn" */
            if(*p=='+'){       *idx_out=1; p++; }
            else if(*p=='-'){ *idx_out=2; p++; }
            else {             *idx_out=3;      }
        }
        /* ignore any remaining characters (e.g. trailing spaces) */
    }
}

/* ══════════════════════════════════════════════════════════════
 * assemble_line — assemble one line of source
 * In:  line = NUL-terminated source line string
 * Out: emits bytes to rom[]; pc advanced; labels defined on pass 1
 * Clobbers: global pc, errors, lineno
 * ══════════════════════════════════════════════════════════════ */
static void assemble_line(char *line){
    char buf[MAX_LINE];
    strncpy(buf,line,MAX_LINE-1); buf[MAX_LINE-1]=0;
    upcase(buf);
    char *p=buf;
    while(*p){ if(*p==';'){*p=0;break;} p++; }
    p=skip_ws(buf);
    if(!*p) return;

    /* label */
    char lbl[32]="";
    if(!isspace((unsigned char)buf[0])&&buf[0]){
        int i=0;
        while((isalnum((unsigned char)*p)||*p=='_')&&i<31) lbl[i++]=*p++;
        lbl[i]=0;
        if(*p==':') p++;
        p=skip_ws(p);
        if(pass==1) label_define(lbl,pc);
    }
    if(!*p) return;

    /* mnemonic — read until space, tab, or end
     * Signetics syntax uses comma between mnemonic and register:
     *   LODI,R0  $41   →  mnemonic="LODI"  ops[0]="R0"  ops[1]="$41"
     * So after reading mnemonic, skip a leading comma in operands. */
    char mn[16]=""; int mi=0;
    while((isalpha((unsigned char)*p)||isdigit((unsigned char)*p))&&mi<15)
        mn[mi++]=*p++;
    mn[mi]=0;
    p=skip_ws(p);
    if(*p==',') p++;   /* skip Signetics-style mnemonic,register comma */
    p=skip_ws(p);

    /* operands */
    char ops[6][64]={"","","","","",""};
    int nops=split_ops(p,ops,6);

    /* ── DIRECTIVES ────────────────────────────────────────── */
    if(strcmp(mn,"ORG")==0){ int ok,v=eval_expr(ops[0],&ok); if(ok){pc=v; if(pass==1&&*lbl) label_define(lbl,pc);} return; }
    if(strcmp(mn,"EQU")==0){ int ok,v=eval_expr(ops[0],&ok); if(ok) label_define(lbl,v); return; }
    if(strcmp(mn,"DS" )==0){ int ok,n=eval_expr(ops[0],&ok); if(ok){for(int i=0;i<n;i++){emit(pc,0);pc++;}} return; }
    if(strcmp(mn,"DB" )==0){ for(int i=0;i<nops;i++){int ok,v=eval_expr(ops[i],&ok); emit(pc,(unsigned char)(v&0xFF));pc++;} return; }
    if(strcmp(mn,"DW" )==0){ for(int i=0;i<nops;i++){int ok,v=eval_expr(ops[i],&ok); emit(pc,(unsigned char)((v>>8)&0xFF));pc++; emit(pc,(unsigned char)(v&0xFF));pc++;} return; }
    if(strcmp(mn,"END")==0) return;

    /* ── SPECIAL 1-BYTE INSTRUCTIONS ──────────────────────── */
    if(strcmp(mn,"NOP" )==0){ emit(pc,0xC0);pc++; return; }
    if(strcmp(mn,"HALT")==0){ emit(pc,0x40);pc++; return; }
    if(strcmp(mn,"SPSU")==0){ emit(pc,0x12);pc++; return; }
    if(strcmp(mn,"SPSL")==0){ emit(pc,0x13);pc++; return; }
    if(strcmp(mn,"LPSU")==0){ emit(pc,0x92);pc++; return; }
    if(strcmp(mn,"LPSL")==0){ emit(pc,0x93);pc++; return; }
    if(strcmp(mn,"ZBRR")==0){ emit(pc,0x9B);pc++; return; }
    if(strcmp(mn,"ZBSR")==0){ emit(pc,0xBB);pc++; return; }

    /* RETC,cc  ($14+cc) */
    if(strcmp(mn,"RETC")==0){
        int cc=cc_val(ops[0]);
        if(cc<0){ fprintf(stderr,"ERROR line %d: RETC needs EQ/GT/LT/UN, got '%s'\n",lineno,ops[0]); errors++; return; }
        emit(pc,(unsigned char)(0x14|cc)); pc++; return;
    }
    if(strcmp(mn,"RETE")==0){
        int cc=cc_val(ops[0]);
        if(cc<0){ fprintf(stderr,"ERROR line %d: RETE needs EQ/GT/LT/UN\n",lineno); errors++; return; }
        emit(pc,(unsigned char)(0x34|cc)); pc++; return;
    }

    /* 2-BYTE PSW INSTRUCTIONS: CPSU CPSL PPSU PPSL TPSU TPSL */
    if(strcmp(mn,"CPSU")==0){ emit(pc,0x74);pc++; int ok,v=eval_expr(ops[0],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"CPSL")==0){ emit(pc,0x75);pc++; int ok,v=eval_expr(ops[0],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"PPSU")==0){ emit(pc,0x76);pc++; int ok,v=eval_expr(ops[0],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"PPSL")==0){ emit(pc,0x77);pc++; int ok,v=eval_expr(ops[0],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"TPSU")==0){ emit(pc,0xB4);pc++; int ok,v=eval_expr(ops[0],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"TPSL")==0){ emit(pc,0xB5);pc++; int ok,v=eval_expr(ops[0],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }

    /* DAR,Rn  ($94-$97) */
    if(strcmp(mn,"DAR")==0){
        int r=reg_val(ops[0]);
        if(r<0){ fprintf(stderr,"ERROR line %d: DAR needs Rn\n",lineno); errors++; return; }
        emit(pc,(unsigned char)(0x94|r)); pc++; return;
    }

    /* TMI,Rn  ($F4-$F7) */
    if(strcmp(mn,"TMI")==0){
        int r=reg_val(ops[0]);
        if(r<0){ fprintf(stderr,"ERROR line %d: TMI needs Rn\n",lineno); errors++; return; }
        emit(pc,(unsigned char)(0xF4|r)); pc++;
        int ok,v=eval_expr(ops[1],&ok); emit(pc,(unsigned char)(v&0xFF)); pc++;
        return;
    }

    /* RRL,Rn  ($D0-$D3)  /  RRR,Rn  ($50-$53) */
    if(strcmp(mn,"RRL")==0){
        int r=reg_val(ops[0]);
        if(r<0){ fprintf(stderr,"ERROR line %d: RRL needs Rn\n",lineno); errors++; return; }
        emit(pc,(unsigned char)(0xD0|r)); pc++; return;
    }
    if(strcmp(mn,"RRR")==0){
        int r=reg_val(ops[0]);
        if(r<0){ fprintf(stderr,"ERROR line %d: RRR needs Rn\n",lineno); errors++; return; }
        emit(pc,(unsigned char)(0x50|r)); pc++; return;
    }

    /* I/O: REDC REDD REDE WRTC WRTD WRTE  (1-byte, Rn) */
    struct { const char *mn; int base; } io[]={
        {"REDC",0x30},{"REDD",0x70},{"REDE",0x54},
        {"WRTC",0xB0},{"WRTD",0xF0},{"WRTE",0xD4},
        {NULL,0}
    };
    for(int i=0;io[i].mn;i++){
        if(strcmp(mn,io[i].mn)==0){
            int r=reg_val(ops[0]);
            if(r<0){ fprintf(stderr,"ERROR line %d: %s needs Rn\n",lineno,mn); errors++; return; }
            emit(pc,(unsigned char)(io[i].base|r)); pc++; return;
        }
    }

    /* ── BRANCH instructions ────────────────────────────────
     *
     * Conditional branches encode cc in bits 1-0:
     *   EQ=00 GT=01 LT=10 UN=11
     *
     * BCTR,cc  rel      $18+cc  (2 bytes)
     * BCTA,cc  abs      $1C+cc  (3 bytes)
     * BCFR,cc  rel      $98+cc  (2 bytes)  [branch if cc NOT set]
     * BCFA,cc  abs      $9C+cc  (3 bytes)
     * BSTR,cc  rel      $38+cc  (2 bytes)  [branch-subroutine]
     * BSTA,cc  abs      $3C+cc  (3 bytes)
     * BSFR,cc  rel      $B8+cc  (2 bytes)
     * BSFA,cc  abs      $BC+cc  (3 bytes)
     *
     * Loop branches (Rn = loop counter in bits 1-0 of opcode):
     * BRNR,Rn  rel      $58+r   (2 bytes)
     * BRNA,Rn  abs      $5C+r   (3 bytes)
     * BIRR,Rn  rel      $D8+r   (2 bytes)
     * BIRA,Rn  abs      $DC+r   (3 bytes)
     * BDRR,Rn  rel      $F8+r   (2 bytes)
     * BDRA,Rn  abs      $FC+r   (3 bytes)
     * BSNR,Rn  rel      $78+r   (2 bytes)
     * BSNA,Rn  abs      $7C+r   (3 bytes)
     * BXA,R3   abs      $9F     (3 bytes)  [indexed branch]
     * BSXA,R3  abs      $BF     (3 bytes)
     *
     * Note: branch absolute instructions use emit_abs_br() NOT emit_abs()
     * because bits 6-5 of address byte 1 are page bits, not idxctl.
     * Reference: https://en.wikibooks.org/wiki/Signetics_2650_%26_2636_programming/Indexed_branching
     * ──────────────────────────────────────────────────────── */

    struct { const char *mn; int base_r; int base_a; int uses_cc; } br[]={
        {"BCTR",0x18,0x1C,1},
        {"BCFR",0x98,0x9C,1},
        {"BSTR",0x38,0x3C,1},
        {"BSFR",0xB8,0xBC,1},
        {"BRNR",0x58,0x5C,0},
        {"BIRR",0xD8,0xDC,0},
        {"BDRR",0xF8,0xFC,0},
        {"BSNR",0x78,0x7C,0},
        {NULL,0,0,0}
    };

    #define PARSE_FIELD(ops, nops, field_str, addr_out) do { \
        if((nops)>1 && (ops)[1][0]) { \
            (field_str)=(ops)[0]; (addr_out)=(ops)[1]; \
        } else { \
            char *_p=(ops)[0]; \
            while(*_p && *_p!=' ' && *_p!='\t') _p++; \
            static char _fbuf[8]; \
            int _fl=(int)(_p-(ops)[0]); if(_fl>7)_fl=7; \
            strncpy(_fbuf,(ops)[0],_fl); _fbuf[_fl]=0; \
            (field_str)=_fbuf; \
            while(*_p==' '||*_p=='\t') _p++; \
            (addr_out)=_p; \
        } \
    } while(0)

    for(int i=0;br[i].mn;i++){
        int blen=strlen(br[i].mn);
        if(strncmp(mn,br[i].mn,blen)==0){
            char *suf=mn+blen;
            int is_abs=(strcmp(suf,"A")==0);
            int is_rel=(strcmp(suf,"R")==0||strcmp(suf,"")==0);
            if(!is_abs&&!is_rel) break;
            char *field_str, *addr_s;
            PARSE_FIELD(ops, nops, field_str, addr_s);
            int field;
            if(br[i].uses_cc){
                field=cc_val(field_str);
                if(field<0){ fprintf(stderr,"ERROR line %d: %s needs EQ/GT/LT/UN, got '%s'\n",lineno,mn,field_str); errors++; return; }
            } else {
                field=reg_val(field_str);
                if(field<0){ fprintf(stderr,"ERROR line %d: %s needs Rn\n",lineno,mn); errors++; return; }
            }
            int ind=0; if(*addr_s=='*'){ind=1;addr_s++;}
            int ok,v=eval_expr(addr_s,&ok);
            if(is_rel){
                emit(pc,(unsigned char)(br[i].base_r|field)); pc++;
                if(ok) emit_rel(v,ind); else {emit(pc,0);pc++;}
            } else {
                emit(pc,(unsigned char)(br[i].base_a|field)); pc++;
                /* branches use emit_abs_br: bits 6-5 are PAGE bits, not idxctl */
                if(ok) emit_abs_br(v,ind,0); else {emit(pc,0);pc++;emit(pc,0);pc++;}
            }
            return;
        }
    }

    /* BCTA / BCFA / BSTA / BSFA (abs-only forms with cc) */
    struct { const char *mn; int base; } bra[]={
        {"BCTA",0x1C},{"BCFA",0x9C},{"BSTA",0x3C},{"BSFA",0xBC},
        {NULL,0}
    };
    for(int i=0;bra[i].mn;i++){
        if(strcmp(mn,bra[i].mn)==0){
            char *cc_s, *addr_s;
            PARSE_FIELD(ops, nops, cc_s, addr_s);
            int cc=cc_val(cc_s);
            if(cc<0){ fprintf(stderr,"ERROR line %d: %s needs EQ/GT/LT/UN\n",lineno,mn); errors++; return; }
            int ind=0; if(*addr_s=='*'){ind=1;addr_s++;}
            int ok,v=eval_expr(addr_s,&ok);
            emit(pc,(unsigned char)(bra[i].base|cc)); pc++;
            /* branches: emit_abs_br (pp bits, not idxctl) */
            if(ok) emit_abs_br(v,ind,0); else {emit(pc,0);pc++;emit(pc,0);pc++;}
            return;
        }
    }
    if(strcmp(mn,"BRNA")==0||strcmp(mn,"BIRA")==0||strcmp(mn,"BDRA")==0||strcmp(mn,"BSNA")==0){
        int base=(strcmp(mn,"BRNA")==0)?0x5C:(strcmp(mn,"BIRA")==0)?0xDC:(strcmp(mn,"BDRA")==0)?0xFC:0x7C;
        int r=reg_val(ops[0]); if(r<0){fprintf(stderr,"ERROR line %d: %s needs Rn\n",lineno,mn);errors++;return;}
        char *addr_s=ops[1]; int ind=0; if(*addr_s=='*'){ind=1;addr_s++;}
        int ok,v=eval_expr(addr_s,&ok);
        emit(pc,(unsigned char)(base|r)); pc++;
        if(ok) emit_abs_br(v,ind,0); else {emit(pc,0);pc++;emit(pc,0);pc++;}
        return;
    }
    if(strcmp(mn,"BXA")==0){ int ok,v=eval_expr(ops[0],&ok); int ind=0; char *a=ops[0]; if(*a=='*'){ind=1;a++;v=eval_expr(a,&ok);}
        emit(pc,0x9F);pc++; if(ok) emit_abs_br(v,ind,0); else{emit(pc,0);pc++;emit(pc,0);pc++;} return; }
    if(strcmp(mn,"BSXA")==0){ int ok,v=eval_expr(ops[0],&ok); int ind=0; char *a=ops[0]; if(*a=='*'){ind=1;a++;v=eval_expr(a,&ok);}
        emit(pc,0xBF);pc++; if(ok) emit_abs_br(v,ind,0); else{emit(pc,0);pc++;emit(pc,0);pc++;} return; }

    /* ── ALU group: LOD STR EOR AND IOR ADD SUB COM ──────────
     *
     * Mnemonic = OPmode,Rn  operand
     *   mode: Z=register I=immediate R=relative A=absolute
     *
     * Opcode bases:
     *   LODZ=$00 EORZ=$20 ANDZ=$40 IORZ=$60
     *   ADDZ=$80 SUBZ=$A0 COMZ=$E0
     *   STRZ=$C0 (no STRI)
     *   (+$04=immediate, +$08=relative, +$0C=absolute)
     *
     * Absolute mode (A) now supports indexed addressing (v1.3):
     *   LODA,Rn  addr         idxctl=0  plain
     *   LODA,Rn  addr,Rm+     idxctl=1  pre-increment
     *   LODA,Rn  addr,Rm-     idxctl=2  pre-decrement
     *   LODA,Rn  addr,Rm      idxctl=3  add without modify
     * Reference: https://amigan.yatho.com/2650UM.html#main.html
     * ──────────────────────────────────────────────────────── */
    struct { const char *pfx; int base; int no_imm; } alu[]={
        {"LOD",0x00,0},{"EOR",0x20,0},{"AND",0x40,0},{"IOR",0x60,0},
        {"ADD",0x80,0},{"SUB",0xA0,0},{"COM",0xE0,0},{"STR",0xC0,1},
        {NULL,0,0}
    };
    for(int i=0;alu[i].pfx;i++){
        int plen=strlen(alu[i].pfx);
        if(strncmp(mn,alu[i].pfx,plen)==0){
            char *suf=mn+plen;
            int mode=-1;
            if(strcmp(suf,"Z")==0) mode=0;
            else if(strcmp(suf,"I")==0) mode=1;
            else if(strcmp(suf,"R")==0) mode=2;
            else if(strcmp(suf,"A")==0) mode=3;
            else break;
            if(alu[i].no_imm&&mode==1){ fprintf(stderr,"ERROR line %d: STRI not valid\n",lineno); errors++; return; }

            int r=reg_val(ops[0]);
            if(r<0){ fprintf(stderr,"ERROR line %d: %s needs Rn\n",lineno,mn); errors++; return; }

            if(alu[i].base==0x00&&mode==0&&r==0&&pass==2)
                fprintf(stderr,"WARNING line %d: LODZ R0 ($00) is indeterminate on 2650\n",lineno);

            unsigned char ob=(unsigned char)(alu[i].base+(mode<<2)+r);
            emit(pc,ob); pc++;

            /* operand: may be in ops[1] (comma-separated) or after reg in ops[0] */
            char *addr_s=(nops>1&&ops[1][0]) ? ops[1] : ops0_after_reg(ops[0]);

            switch(mode){
                case 0: /* Z: register-to-register, no extra byte */ break;
                case 1: /* I: immediate */
                    { int ok,v=eval_expr(addr_s,&ok);
                      emit(pc,(unsigned char)(v&0xFF)); pc++; break; }
                case 2: /* R: relative */
                    { int ind=0; if(*addr_s=='*'){ind=1;addr_s++;}
                      int ok,v=eval_expr(addr_s,&ok);
                      if(ok) emit_rel(v,ind); else{emit(pc,0);pc++;} break; }
                case 3: /* A: absolute — parse with optional indexed suffix (v1.3) */
                    { int ind=0, idxctl=0, v=0, ok=0;
                      parse_abs_addr(addr_s,&ind,&idxctl,&v,&ok);
                      if(ok) emit_abs(v,ind,idxctl); else{emit(pc,0);pc++;emit(pc,0);pc++;}
                      break; }
            }
            return;
        }
    }

    /* ── Unknown ────────────────────────────────────────────── */
    if(pass==2){ fprintf(stderr,"ERROR line %d: unknown mnemonic '%s'\n",lineno,mn); errors++; }
}

/* ── Intel HEX output ─────────────────────────────────────────
 * write_hex — write Intel HEX records to file
 * In:  f = output FILE*
 * Out: Intel HEX written including EOF record
 * Clobbers: nothing
 */
static void write_hex(FILE *f){
    if(rom_hi<rom_lo){ fprintf(f,":00000001FF\n"); return; }
    for(int addr=rom_lo;addr<=rom_hi;){
        int n=rom_hi-addr+1; if(n>16) n=16;
        unsigned char sum=(unsigned char)(n+(addr>>8)+(addr&0xFF));
        fprintf(f,":%02X%04X00",n,addr);
        for(int i=0;i<n;i++){ fprintf(f,"%02X",rom[addr+i]); sum+=rom[addr+i]; }
        fprintf(f,"%02X\n",(unsigned char)(-sum));
        addr+=n;
    }
    fprintf(f,":00000001FF\n");
}

/* ── main ─────────────────────────────────────────────────────
 * In:  argv[1] = source .asm file
 *      argv[2] = optional output .hex file (stdout if omitted)
 * Out: 0=success  1=errors/cannot open
 */
int main(int argc,char *argv[]){
    if(argc<2){ fprintf(stderr,"Usage: asm2650 source.asm [output.hex]\n"); return 1; }
    memset(rom,0xFF,sizeof(rom));
    for(pass=1;pass<=2;pass++){
        FILE *f=fopen(argv[1],"r"); if(!f){fprintf(stderr,"Cannot open '%s'\n",argv[1]);return 1;}
        pc=0; lineno=0;
        char line[MAX_LINE];
        while(fgets(line,MAX_LINE,f)){
            lineno++;
            int l=strlen(line);
            while(l>0&&(line[l-1]=='\r'||line[l-1]=='\n')) line[--l]=0;
            assemble_line(line);
        }
        fclose(f);
    }
    fprintf(stderr,"Pass complete: %d error(s), %d label(s)\n",errors,nlabels);
    if(errors) return 1;
    if(rom_hi>=rom_lo) fprintf(stderr,"Code: $%04X-$%04X (%d bytes)\n",rom_lo,rom_hi,rom_hi-rom_lo+1);
    FILE *out=stdout;
    if(argc>=3){ out=fopen(argv[2],"w"); if(!out){fprintf(stderr,"Cannot create '%s'\n",argv[2]);return 1;} }
    write_hex(out);
    if(out!=stdout) fclose(out);
    return 0;
}
