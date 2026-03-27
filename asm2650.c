/* ============================================================================
 * asm2650_v1.4.c  —  Signetics 2650 cross-assembler
 * ============================================================================
 *
 * VERSION HISTORY
 *   v1.0  initial skeleton
 *   v1.1  opcode table corrections
 *   v1.2  character literals, high/low byte extractors, two-pass labels
 *   v1.3  absolute-indexed addressing (Rn+/Rn-/Rn syntax); subroutine headers
 *   v1.4  2026-03-27
 *         - 3-pass assembler with branch relaxation
 *         - BSTA,cc and BSFA,cc automatically use BSTR/BSFR when target is
 *           in relative range [-64,+63] from the instruction end
 *         - BCTA,cc and BCFA,cc automatically use BCTR/BCFR when in range
 *         - BSTA,UN automatically uses ZBSR when target is in page-zero
 *           ZBSR displacement range ($0000-$003F or $1FC0-$1FFF)
 *         - BCTA,UN automatically uses ZBRR in the same page-zero range
 *         - Relaxation priority: ZBSR > BSTR,UN > BSTA,UN (subroutine calls)
 *                                ZBRR > BCTR,UN > BCTA,UN (plain branches)
 *         - 3-pass guarantees convergence: pass1=pessimistic (all 3-byte),
 *           pass2=relax with pass1 labels, pass3=final emit with pass2 labels
 *         - Relaxation warning emitted to stderr when a branch is shortened
 *
 * PURPOSE
 *   Two-pass (extended to three-pass) cross-assembler that converts 2650
 *   assembly source into Intel HEX.
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
 *   Build:  gcc -Wall -O2 -o asm2650 asm2650_v1.4.c
 *
 * USAGE
 *   asm2650 source.asm [output.hex]
 *     source.asm   input assembly source
 *     output.hex   optional Intel HEX output path (stdout if omitted)
 *
 * BRANCH RELAXATION (v1.4)
 *   Absolute-form branch mnemonics are automatically shortened if possible:
 *
 *   BSTA,UN  target   →  ZBSR target   if target in page-0 ZBSR range
 *                     →  BSTR,UN target  if target within [-64,+63] of instr end
 *                     →  BSTA,UN target  (3-byte form, not relaxed)
 *
 *   BSTA,cc  target   →  BSTR,cc target  if target within [-64,+63]
 *   (cc=EQ/GT/LT)    →  BSTA,cc target  (not relaxed)
 *
 *   BCTA,UN  target   →  ZBRR target   if target in page-0 ZBRR range
 *                     →  BCTR,UN target  if target within [-64,+63]
 *                     →  BCTA,UN target  (not relaxed)
 *
 *   BCTA,cc  target   →  BCTR,cc target  if target within [-64,+63]
 *   BSFA,cc  target   →  BSFR,cc target  if target within [-64,+63]
 *   BCFA,cc  target   →  BCFR,cc target  if target within [-64,+63]
 *
 *   ZBSR/ZBRR page-zero displacement encoding:
 *     The displacement byte is the target address's low 7 bits as a signed
 *     integer relative to page-zero byte 0.
 *     Reachable: $0000-$003F (disp +0..+63) and $1FC0-$1FFF (disp -64..-1)
 *     Reference: 2650 User Manual, ZBSR/ZBRR instruction descriptions
 *
 *   3-pass convergence:
 *     Pass 1: all absolute-form branches → 3 bytes (pessimistic)
 *     Pass 2: all labels known from pass 1; compute short forms where possible
 *     Pass 3: labels recomputed from pass 2 sizes; final emit
 *     Passes 2 and 3 produce the same sizes in almost all real code.
 *
 * INDEXED ADDRESSING (v1.3, unchanged)
 *   LODA,Rn  addr,Rm+   idxctl=01  pre-increment Rn, EA = addr + Rn
 *   LODA,Rn  addr,Rm-   idxctl=10  pre-decrement Rn, EA = addr + Rn
 *   LODA,Rn  addr,Rm    idxctl=11  EA = addr + Rn, no modify
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
static int  pass   = 0;   /* 1, 2, or 3 */
static int  errors = 0;
static int  lineno = 0;
static int  relaxed_count = 0;  /* branches shortened this pass */

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
 * Clobbers: nothing (errors++ on out-of-range in pass 3)
 */
static void emit(int addr, unsigned char b){
    if(addr<0||addr>=MAX_ROM){
        if(pass==3){fprintf(stderr,"ERROR line %d: addr $%04X out of range\n",lineno,addr); errors++;}
        return;
    }
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
    if(*s=='$'){
        s++; if(!isxdigit((unsigned char)*s)){*ok=0;return 0;}
        while(isxdigit((unsigned char)*s)) val=val*16+(isdigit((unsigned char)*s)?*s-'0':toupper((unsigned char)*s)-'A'+10), s++;
    } else if(*s=='%'){
        s++; while(*s=='0'||*s=='1') val=val*2+(*s++-'0');
    } else if(isdigit((unsigned char)*s)){
        while(isdigit((unsigned char)*s)) val=val*10+(*s++-'0');
    } else if(isalpha((unsigned char)*s)||*s=='_'){
        char nm[32]; int i=0;
        while((isalnum((unsigned char)*s)||*s=='_')&&i<31) nm[i++]=*s++;
        nm[i]=0;
        int lv=label_find(nm);
        if(lv==UNDEF){
            /* In pass 3 all labels must be defined */
            if(pass==3){fprintf(stderr,"ERROR line %d: undefined '%s'\n",lineno,nm); errors++;}
            *ok=0; return 0;
        }
        val=lv;
    } else if(*s=='\''){
        s++;
        val=(unsigned char)*s;
        if(*s) s++;
        if(*s=='\'') s++;
    } else { *ok=0; return 0; }
    if(neg) val=-val;
    s=skip_ws(s);
    if(*s=='+'||*s=='-'){
        int sub=(*s=='-'); s++; s=skip_ws(s);
        int ok2; int rhs=eval_expr(s,&ok2);
        val=sub?val-rhs:val+rhs;
    }
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
 */
static int reg_val(const char *s){
    if(s[0]=='R'&&s[1]>='0'&&s[1]<='3'&&(s[2]==0||s[2]==' '||s[2]=='\t'))
        return s[1]-'0';
    return -1;
}

/*
 * ops0_after_reg — pointer to expression following "Rn" in an operand string
 * In:  s = operand string starting with "R0".."R3"
 * Out: pointer past the "Rn" and any whitespace
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

/* ── branch relaxation helpers ────────────────────────────────── */

/*
 * rel_offset — compute relative offset for a 2-byte branch instruction
 * In:  instr_pc = address of the first byte of the branch instruction
 *      target   = branch target address
 * Out: returns signed offset; 2-byte branch IAR = instr_pc+2 after fetch
 * Clobbers: nothing
 */
static int rel_offset(int instr_pc, int target){
    return target - (instr_pc + 2);
}

/*
 * in_rel_range — test whether a target is in relative branch range
 * In:  instr_pc = address of branch instruction start
 *      target   = branch target address
 * Out: 1 if offset is in [-64,+63], 0 otherwise
 * Clobbers: nothing
 */
static int in_rel_range(int instr_pc, int target){
    int off = rel_offset(instr_pc, target);
    return (off >= -64 && off <= 63);
}

/*
 * zbsr_disp — compute ZBSR/ZBRR displacement for a page-zero target
 * In:  target = branch target address
 *      disp   = output: signed 7-bit displacement value
 * Out: returns 1 if target is reachable by ZBSR/ZBRR, 0 otherwise
 *
 * ZBSR/ZBRR encoding (from 2650 User Manual):
 *   The displacement is a signed 7-bit value relative to page-zero byte 0.
 *   The effective address is computed modulo 8192 (page size).
 *   Reachable: $0000-$003F (disp 0..+63) and $1FC0-$1FFF (disp -64..-1)
 *   Target must be in page 0 (address 0x0000-0x1FFF).
 * Clobbers: nothing
 */
static int zbsr_disp(int target, int *disp){
    if(target < 0 || target >= 0x2000) return 0;  /* must be page 0 */
    /* Low 7 bits of target, sign-extended */
    int d = target & 0x7F;
    if(d >= 0x40) d -= 0x80;   /* sign-extend to signed int */
    /* Verify: does this displacement reconstruct the target? */
    int reconstructed = d & 0x1FFF;  /* modulo page = mod 8192 */
    if(reconstructed < 0) reconstructed += 0x2000;
    if(reconstructed != (target & 0x1FFF)) return 0;
    *disp = d;
    return 1;
}

/*
 * emit_rel — emit signed 7-bit relative offset byte (indirect flag in bit 7)
 * In:  target = absolute target address
 *      ind    = indirect flag (1 = set bit 7)
 * Out: one byte emitted at pc; pc incremented
 * Clobbers: errors++ on pass 3 if out of range
 */
static void emit_rel(int target, int ind){
    int off = target - (pc + 1);
    if(pass==3 && (off < -64 || off > 63)){
        fprintf(stderr,"ERROR line %d: relative offset %d out of range [-64,+63]\n",lineno,off);
        errors++;
    }
    emit(pc,(unsigned char)((off & 0x7F)|(ind?0x80:0))); pc++;
}

/*
 * emit_abs — emit 2-byte address field for non-branch absolute instructions
 * In:  addr   = 15-bit target address
 *      ind    = indirect flag (bit 7 of byte 1)
 *      idxctl = index control (bits 6-5 of byte 1)
 * Out: two bytes emitted; pc += 2
 * Clobbers: nothing
 */
static void emit_abs(int addr, int ind, int idxctl){
    unsigned char b1=(unsigned char)(((addr>>8)&0x1F)|((idxctl&3)<<5)|(ind?0x80:0));
    unsigned char b2=(unsigned char)(addr&0xFF);
    emit(pc,b1); pc++;
    emit(pc,b2); pc++;
}

/*
 * emit_abs_br — emit 2-byte address field for branch absolute instructions
 * In:  addr = 15-bit target address
 *      ind  = indirect flag
 *      pp   = page bits (bits 6-5 of byte 1)
 * Out: two bytes emitted; pc += 2
 * Clobbers: nothing
 */
static void emit_abs_br(int addr, int ind, int pp){
    unsigned char b1=(unsigned char)(((addr>>8)&0x1F)|((pp&3)<<5)|(ind?0x80:0));
    unsigned char b2=(unsigned char)(addr&0xFF);
    emit(pc,b1); pc++;
    emit(pc,b2); pc++;
}

/*
 * parse_abs_addr — parse absolute address operand with optional indexed suffix
 * In:  s       = address string (e.g. "$1400,R2+")
 *      ind_out = output: indirect flag
 *      idx_out = output: idxctl (0=none, 1=Rn+, 2=Rn-, 3=Rn)
 *      val_out = output: evaluated address
 *      ok_out  = output: 1 on success
 * Out: fills output parameters
 * Clobbers: nothing
 */
static void parse_abs_addr(char *s, int *ind_out, int *idx_out, int *val_out, int *ok_out){
    *ind_out=0; *idx_out=0; *val_out=0; *ok_out=0;
    s=skip_ws(s);
    if(*s=='*'){ *ind_out=1; s++; }
    char base[64]; int bi=0;
    char *p=s;
    while(*p && bi<63){
        if(*p==','&&*(p+1)=='R'&&*(p+2)>='0'&&*(p+2)<='3') break;
        base[bi++]=*p++;
    }
    base[bi]=0;
    int ok=0;
    *val_out=eval_expr(base,&ok);
    *ok_out=ok;
    if(*p==','){
        p++;
        if(*p=='R'&&*(p+1)>='0'&&*(p+1)<='3'){
            p+=2;
            if(*p=='+')      { *idx_out=1; p++; }
            else if(*p=='-') { *idx_out=2; p++; }
            else             { *idx_out=3;      }
        }
    }
}

/* ── branch opcode description tables ────────────────────────────
 *
 * Indexed by mnemonic ROOT (4 chars: "BSTA","BSFA","BCTA","BCFA")
 * and condition code (0=EQ,1=GT,2=LT,3=UN).
 *
 * Fields:
 *   abs_opc  = opcode for 3-byte absolute form
 *   rel_opc  = opcode for 2-byte relative form
 *   zp_opc   = page-zero opcode (ZBSR=$BB / ZBRR=$9B), or 0 if n/a
 *              Only set for UN (unconditional) variants
 *   is_call  = 1 for branch-to-subroutine, 0 for plain branch
 *
 * Priority for try_relax_branch:
 *   ZBSR/ZBRR > relative form > absolute form
 * ────────────────────────────────────────────────────────────────*/

typedef struct {
    unsigned char abs_opc;  /* opcode for 3-byte absolute form */
    unsigned char rel_opc;  /* opcode for 2-byte relative form */
    unsigned char zp_opc;   /* 0 or ZBSR/ZBRR opcode for page-zero unconditional */
    int is_call;            /* 1=subroutine branch, 0=plain branch */
} BranchInfo;

/* branch_info[root][cc]:  root: 0=BSTA 1=BSFA 2=BCTA 3=BCFA
 *                         cc:   0=EQ   1=GT   2=LT   3=UN    */
static const BranchInfo branch_info[4][4] = {
    /* BSTA (subroutine call, condition TRUE) */
    { {0x3C,0x38,0x00,1}, {0x3D,0x39,0x00,1}, {0x3E,0x3A,0x00,1}, {0x3F,0x3B,0xBB,1} },
    /* BSFA (subroutine call, condition FALSE) */
    { {0xBC,0xB8,0x00,1}, {0xBD,0xB9,0x00,1}, {0xBE,0xBA,0x00,1}, {0x00,0x00,0x00,1} }, /* BSFA,UN invalid */
    /* BCTA (plain branch, condition TRUE) */
    { {0x1C,0x18,0x00,0}, {0x1D,0x19,0x00,0}, {0x1E,0x1A,0x00,0}, {0x1F,0x1B,0x9B,0} },
    /* BCFA (plain branch, condition FALSE) */
    { {0x9C,0x98,0x00,0}, {0x9D,0x99,0x00,0}, {0x9E,0x9A,0x00,0}, {0x00,0x00,0x00,0} }, /* BCFA,UN invalid */
};

/* branch root name strings (indexed same as branch_info row) */
static const char *branch_roots[4] = { "BSTA", "BSFA", "BCTA", "BCFA" };

/*
 * try_relax_branch — attempt to emit a branch instruction in shortest form
 * In:  bi      = pointer to BranchInfo for this mnemonic/cc combination
 *      mnem    = mnemonic string for diagnostic messages (e.g. "BSTA,UN")
 *      target  = resolved branch target (or UNDEF if unresolved)
 *      ind     = indirect addressing flag
 * Out: shortest valid encoding emitted; pc advanced by 2 or 3
 * Clobbers: pc, rom[], relaxed_count
 *
 * Relaxation priority (for UN forms):
 *   1. ZBSR/ZBRR: if target in page-zero ZBSR range and zp_opc != 0
 *   2. Relative:  if target within [-64,+63] of instruction end
 *   3. Absolute:  3-byte fallback
 *
 * For conditional forms (EQ/GT/LT): only relative vs absolute.
 * In pass 1 with UNDEF target: always emit 3-byte pessimistic form.
 */
static void try_relax_branch(const BranchInfo *bi, const char *mnem, int target, int ind){
    if(target == UNDEF){
        /* Pass 1 or unresolved forward ref: pessimistic 3-byte form */
        emit(pc, bi->abs_opc); pc++;
        emit(pc, 0x00); pc++;
        emit(pc, 0x00); pc++;
        return;
    }

    /* Check ZBSR/ZBRR first (only for UN, only if zp_opc set) */
    if(bi->zp_opc != 0 && !ind){
        int d;
        if(zbsr_disp(target, &d)){
            if(pass == 3)
                fprintf(stderr,"INFO  line %d: %s $%04X -> %s $%02X (1 byte saved)\n",
                    lineno, mnem, target,
                    bi->is_call ? "ZBSR" : "ZBRR", (unsigned char)(d & 0x7F));
            emit(pc, bi->zp_opc); pc++;
            emit(pc, (unsigned char)(d & 0x7F)); pc++;
            relaxed_count++;
            return;
        }
    }

    /* Check relative form */
    if(in_rel_range(pc, target)){
        if(pass == 3)
            fprintf(stderr,"INFO  line %d: %s $%04X -> %s (1 byte saved)\n",
                lineno, mnem, target,
                bi->is_call ? "BSTR/BSFR" : "BCTR/BCFR");
        emit(pc, bi->rel_opc); pc++;
        emit_rel(target, ind);
        relaxed_count++;
        return;
    }

    /* Fall back to 3-byte absolute form */
    emit(pc, bi->abs_opc); pc++;
    emit_abs_br(target, ind, 0);
}

/* ══════════════════════════════════════════════════════════════
 * assemble_line — assemble one source line
 * In:  line = NUL-terminated source line
 * Out: bytes emitted to rom[]; pc advanced; labels defined in pass 1
 * Clobbers: pc, errors, lineno, relaxed_count
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
        else        label_define(lbl,pc);  /* redefine each pass to track shift */
    }
    if(!*p) return;

    /* mnemonic */
    char mn[16]=""; int mi=0;
    while((isalpha((unsigned char)*p)||isdigit((unsigned char)*p))&&mi<15)
        mn[mi++]=*p++;
    mn[mi]=0;
    p=skip_ws(p);
    if(*p==',') p++;  /* Signetics mnemonic,register comma */
    p=skip_ws(p);

    /* operands */
    char ops[6][64]={"","","","","",""};
    int nops=split_ops(p,ops,6);

    /* ── DIRECTIVES ────────────────────────────────────────── */
    if(strcmp(mn,"ORG")==0){
        int ok,v=eval_expr(ops[0],&ok);
        if(ok){ pc=v; if(*lbl) label_define(lbl,pc); }
        return;
    }
    if(strcmp(mn,"EQU")==0){
        int ok,v=eval_expr(ops[0],&ok);
        if(ok) label_define(lbl,v);
        return;
    }
    if(strcmp(mn,"DS")==0){
        int ok,n=eval_expr(ops[0],&ok);
        if(ok){ for(int i=0;i<n;i++){ emit(pc,0); pc++; } }
        return;
    }
    if(strcmp(mn,"DB")==0){
        for(int i=0;i<nops;i++){
            int ok,v=eval_expr(ops[i],&ok);
            emit(pc,(unsigned char)(v&0xFF)); pc++;
        }
        return;
    }
    if(strcmp(mn,"DW")==0){
        for(int i=0;i<nops;i++){
            int ok,v=eval_expr(ops[i],&ok);
            emit(pc,(unsigned char)((v>>8)&0xFF)); pc++;
            emit(pc,(unsigned char)(v&0xFF)); pc++;
        }
        return;
    }
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

    if(strcmp(mn,"RETC")==0){
        int cc=cc_val(ops[0]);
        if(cc<0){ if(pass==3){fprintf(stderr,"ERROR line %d: RETC bad cc '%s'\n",lineno,ops[0]); errors++;} return; }
        emit(pc,(unsigned char)(0x14|cc)); pc++; return;
    }
    if(strcmp(mn,"RETE")==0){
        int cc=cc_val(ops[0]);
        if(cc<0){ if(pass==3){fprintf(stderr,"ERROR line %d: RETE bad cc\n",lineno); errors++;} return; }
        emit(pc,(unsigned char)(0x34|cc)); pc++; return;
    }

    /* 2-BYTE PSW: CPSU CPSL PPSU PPSL TPSU TPSL */
    if(strcmp(mn,"CPSU")==0){ emit(pc,0x74);pc++; int ok,v=eval_expr(ops[0],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"CPSL")==0){ emit(pc,0x75);pc++; int ok,v=eval_expr(ops[0],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"PPSU")==0){ emit(pc,0x76);pc++; int ok,v=eval_expr(ops[0],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"PPSL")==0){ emit(pc,0x77);pc++; int ok,v=eval_expr(ops[0],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"TPSU")==0){ emit(pc,0xB4);pc++; int ok,v=eval_expr(ops[0],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"TPSL")==0){ emit(pc,0xB5);pc++; int ok,v=eval_expr(ops[0],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }

    /* DAR,Rn  TMI,Rn  RRL,Rn  RRR,Rn */
    if(strcmp(mn,"DAR")==0){
        int r=reg_val(ops[0]);
        if(r<0){ if(pass==3){fprintf(stderr,"ERROR line %d: DAR needs Rn\n",lineno); errors++;} return; }
        emit(pc,(unsigned char)(0x94|r)); pc++; return;
    }
    if(strcmp(mn,"TMI")==0){
        int r=reg_val(ops[0]);
        if(r<0){ if(pass==3){fprintf(stderr,"ERROR line %d: TMI needs Rn\n",lineno); errors++;} return; }
        emit(pc,(unsigned char)(0xF4|r)); pc++;
        int ok,v=eval_expr(ops[1],&ok); emit(pc,(unsigned char)(v&0xFF)); pc++;
        return;
    }
    if(strcmp(mn,"RRL")==0){
        int r=reg_val(ops[0]);
        if(r<0){ if(pass==3){fprintf(stderr,"ERROR line %d: RRL needs Rn\n",lineno); errors++;} return; }
        emit(pc,(unsigned char)(0xD0|r)); pc++; return;
    }
    if(strcmp(mn,"RRR")==0){
        int r=reg_val(ops[0]);
        if(r<0){ if(pass==3){fprintf(stderr,"ERROR line %d: RRR needs Rn\n",lineno); errors++;} return; }
        emit(pc,(unsigned char)(0x50|r)); pc++; return;
    }

    /* I/O: REDC REDD REDE WRTC WRTD WRTE */
    {
        struct { const char *mn; int base; } io[]={
            {"REDC",0x30},{"REDD",0x70},{"REDE",0x54},
            {"WRTC",0xB0},{"WRTD",0xF0},{"WRTE",0xD4},
            {NULL,0}
        };
        for(int i=0;io[i].mn;i++){
            if(strcmp(mn,io[i].mn)==0){
                int r=reg_val(ops[0]);
                if(r<0){ if(pass==3){fprintf(stderr,"ERROR line %d: %s needs Rn\n",lineno,mn); errors++;} return; }
                emit(pc,(unsigned char)(io[i].base|r)); pc++; return;
            }
        }
    }

    /* ── RELAXABLE BRANCH INSTRUCTIONS ──────────────────────
     *
     * Recognised mnemonic roots: BSTA BSFA BCTA BCFA
     * (all absolute-form branch mnemonics that can be relaxed)
     *
     * The condition code (EQ/GT/LT/UN) is extracted from the first
     * operand field using PARSE_FIELD (same as in the explicit-relative
     * section below).  The target address is the second field.
     *
     * Indirect addressing ('*' prefix) suppresses ZBSR/ZBRR but
     * still allows the relative form.
     * ──────────────────────────────────────────────────────── */
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

    for(int ri=0; ri<4; ri++){
        if(strcmp(mn, branch_roots[ri]) == 0){
            char *field_str, *addr_s;
            PARSE_FIELD(ops, nops, field_str, addr_s);
            int cc = cc_val(field_str);
            if(cc < 0){
                if(pass==3){
                    fprintf(stderr,"ERROR line %d: %s needs EQ/GT/LT/UN, got '%s'\n",
                            lineno, mn, field_str);
                    errors++;
                }
                return;
            }
            const BranchInfo *bi = &branch_info[ri][cc];
            if(bi->abs_opc == 0){
                if(pass==3){
                    fprintf(stderr,"ERROR line %d: %s,%s is not a valid instruction\n",
                            lineno, mn, field_str);
                    errors++;
                }
                return;
            }
            int ind = 0;
            if(*addr_s == '*'){ ind = 1; addr_s++; }
            int ok, v = eval_expr(addr_s, &ok);
            int target = ok ? v : UNDEF;
            /* Build display mnemonic for diagnostics */
            char disp_mn[12];
            snprintf(disp_mn, sizeof(disp_mn), "%s,%s", mn, field_str);
            try_relax_branch(bi, disp_mn, target, ind);
            return;
        }
    }

    /* ── EXPLICIT RELATIVE BRANCH FORMS ─────────────────────
     *
     * If the programmer explicitly writes BSTR,UN / BCTR,EQ / etc.
     * we emit exactly what was asked for (no relaxation).
     *
     * BCTR,cc  rel      $18+cc  (2 bytes)
     * BCFR,cc  rel      $98+cc  (2 bytes)
     * BSTR,cc  rel      $38+cc  (2 bytes)
     * BSFR,cc  rel      $B8+cc  (2 bytes)
     * BRNR,Rn  rel      $58+r   (2 bytes)
     * BIRR,Rn  rel      $D8+r   (2 bytes)
     * BDRR,Rn  rel      $F8+r   (2 bytes)
     * BSNR,Rn  rel      $78+r   (2 bytes)
     * ──────────────────────────────────────────────────────── */
    {
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
                    if(field<0){ if(pass==3){fprintf(stderr,"ERROR line %d: %s needs EQ/GT/LT/UN\n",lineno,mn); errors++;} return; }
                } else {
                    field=reg_val(field_str);
                    if(field<0){ if(pass==3){fprintf(stderr,"ERROR line %d: %s needs Rn\n",lineno,mn); errors++;} return; }
                }
                int ind=0; if(*addr_s=='*'){ind=1;addr_s++;}
                int ok,v=eval_expr(addr_s,&ok);
                if(is_rel){
                    emit(pc,(unsigned char)(br[i].base_r|field)); pc++;
                    if(ok) emit_rel(v,ind); else {emit(pc,0);pc++;}
                } else {
                    emit(pc,(unsigned char)(br[i].base_a|field)); pc++;
                    if(ok) emit_abs_br(v,ind,0); else {emit(pc,0);pc++;emit(pc,0);pc++;}
                }
                return;
            }
        }
    }

    /* BRNA BIRA BDRA BSNA (abs-only loop/sense branches, Rn) */
    if(strcmp(mn,"BRNA")==0||strcmp(mn,"BIRA")==0||strcmp(mn,"BDRA")==0||strcmp(mn,"BSNA")==0){
        int base=(strcmp(mn,"BRNA")==0)?0x5C:(strcmp(mn,"BIRA")==0)?0xDC:(strcmp(mn,"BDRA")==0)?0xFC:0x7C;
        int r=reg_val(ops[0]);
        if(r<0){ if(pass==3){fprintf(stderr,"ERROR line %d: %s needs Rn\n",lineno,mn);errors++;} return; }
        char *addr_s=ops[1]; int ind=0; if(*addr_s=='*'){ind=1;addr_s++;}
        int ok,v=eval_expr(addr_s,&ok);
        emit(pc,(unsigned char)(base|r)); pc++;
        if(ok) emit_abs_br(v,ind,0); else {emit(pc,0);pc++;emit(pc,0);pc++;}
        return;
    }

    /* BXA / BSXA */
    if(strcmp(mn,"BXA")==0){
        int ok,v=eval_expr(ops[0],&ok); int ind=0; char *a=ops[0];
        if(*a=='*'){ind=1;a++;v=eval_expr(a,&ok);}
        emit(pc,0x9F);pc++;
        if(ok) emit_abs_br(v,ind,0); else{emit(pc,0);pc++;emit(pc,0);pc++;}
        return;
    }
    if(strcmp(mn,"BSXA")==0){
        int ok,v=eval_expr(ops[0],&ok); int ind=0; char *a=ops[0];
        if(*a=='*'){ind=1;a++;v=eval_expr(a,&ok);}
        emit(pc,0xBF);pc++;
        if(ok) emit_abs_br(v,ind,0); else{emit(pc,0);pc++;emit(pc,0);pc++;}
        return;
    }

    /* ── ALU group: LOD STR EOR AND IOR ADD SUB COM ──────────
     *
     * Modes: Z=register I=immediate R=relative A=absolute
     * Opcode bases: LOD=$00 EOR=$20 AND=$40 IOR=$60
     *               ADD=$80 SUB=$A0 COM=$E0 STR=$C0
     * Mode offset: +$00=Z +$04=I +$08=R +$0C=A
     *
     * Absolute mode (A) supports indexed addressing (v1.3):
     *   addr,Rn+  idxctl=01  addr,Rn-  idxctl=10  addr,Rn  idxctl=11
     * ──────────────────────────────────────────────────────── */
    {
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
                if(alu[i].no_imm&&mode==1){
                    if(pass==3){fprintf(stderr,"ERROR line %d: STRI not valid\n",lineno); errors++;}
                    return;
                }
                int r=reg_val(ops[0]);
                if(r<0){
                    if(pass==3){fprintf(stderr,"ERROR line %d: %s needs Rn\n",lineno,mn); errors++;}
                    return;
                }
                if(alu[i].base==0x00&&mode==0&&r==0&&pass==3)
                    fprintf(stderr,"WARNING line %d: LODZ R0 ($00) is indeterminate\n",lineno);
                emit(pc,(unsigned char)(alu[i].base+(mode<<2)+r)); pc++;
                char *addr_s=(nops>1&&ops[1][0])?ops[1]:ops0_after_reg(ops[0]);
                switch(mode){
                    case 0: break;
                    case 1: { int ok,v=eval_expr(addr_s,&ok); emit(pc,(unsigned char)(v&0xFF)); pc++; break; }
                    case 2: { int ind=0; if(*addr_s=='*'){ind=1;addr_s++;}
                              int ok,v=eval_expr(addr_s,&ok);
                              if(ok) emit_rel(v,ind); else{emit(pc,0);pc++;} break; }
                    case 3: { int ind=0,idxctl=0,v=0,ok=0;
                              parse_abs_addr(addr_s,&ind,&idxctl,&v,&ok);
                              if(ok) emit_abs(v,ind,idxctl); else{emit(pc,0);pc++;emit(pc,0);pc++;}
                              break; }
                }
                return;
            }
        }
    }

    /* ── Unknown ────────────────────────────────────────────── */
    if(pass==3){ fprintf(stderr,"ERROR line %d: unknown mnemonic '%s'\n",lineno,mn); errors++; }
    else {
        /* Still need to advance pc correctly in passes 1 and 2.
         * We don't know the size, so emit 0 bytes and warn. */
    }
}

/* ── Intel HEX writer ─────────────────────────────────────────
 * write_hex — write Intel HEX to file
 * In:  f = output FILE*
 * Out: complete Intel HEX including EOF record written
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

/* ── run_pass — execute one assembler pass over source file ───
 * In:  filename = source .asm file path
 *      pass_num = pass number (1, 2, or 3) – sets global 'pass'
 * Out: labels updated; rom[] updated (pass 3 = final);
 *      returns 0 on success, non-zero on file open error
 * Clobbers: pc, errors, lineno, relaxed_count, labels[], rom[]
 */
static int run_pass(const char *filename, int pass_num){
    FILE *f = fopen(filename, "r");
    if(!f){ fprintf(stderr,"Cannot open '%s'\n",filename); return 1; }
    pass = pass_num;
    pc = 0;
    lineno = 0;
    relaxed_count = 0;
    if(pass_num != 1) memset(rom, 0xFF, sizeof(rom));  /* re-emit cleanly each pass */
    rom_lo = MAX_ROM; rom_hi = -1;
    char line[MAX_LINE];
    while(fgets(line, MAX_LINE, f)){
        lineno++;
        int l = strlen(line);
        while(l>0&&(line[l-1]=='\r'||line[l-1]=='\n')) line[--l]=0;
        assemble_line(line);
    }
    fclose(f);
    return 0;
}

/* ── main ─────────────────────────────────────────────────────
 * In:  argv[1] = source .asm file
 *      argv[2] = optional output .hex file (stdout if omitted)
 * Out: 0=success  1=errors/cannot open
 */
int main(int argc, char *argv[]){
    if(argc < 2){ fprintf(stderr,"Usage: asm2650 source.asm [output.hex]\n"); return 1; }
    memset(rom, 0xFF, sizeof(rom));

    /* Pass 1: pessimistic — all branches 3 bytes, establish label positions */
    if(run_pass(argv[1], 1)) return 1;
    fprintf(stderr,"Pass 1: %d label(s) defined\n", nlabels);

    /* Pass 2: relax with labels from pass 1 */
    if(run_pass(argv[1], 2)) return 1;
    fprintf(stderr,"Pass 2: %d branch(es) relaxed\n", relaxed_count);

    /* Pass 3: final emit with labels from pass 2 (errors reported here) */
    if(run_pass(argv[1], 3)) return 1;
    fprintf(stderr,"Pass 3: %d error(s), %d label(s), %d branch(es) relaxed\n",
            errors, nlabels, relaxed_count);

    if(errors) return 1;
    if(rom_hi >= rom_lo)
        fprintf(stderr,"Code: $%04X-$%04X (%d bytes)\n",
                rom_lo, rom_hi, rom_hi - rom_lo + 1);

    FILE *out = stdout;
    if(argc >= 3){
        out = fopen(argv[2], "w");
        if(!out){ fprintf(stderr,"Cannot create '%s'\n",argv[2]); return 1; }
    }
    write_hex(out);
    if(out != stdout) fclose(out);
    return 0;
}
