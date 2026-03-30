/* ============================================================================
 * asm2650_v1.2.c  (assembler core version 1.3)
 * Signetics 2650 cross-assembler — from project repo
 * Build: gcc -Wall -O2 -o asm2650 asm2650_v1.2.c
 * ============================================================================ */
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

static void upcase(char *s){ for(;*s;s++) *s=(char)toupper((unsigned char)*s); }
static char *skip_ws(char *s){ while(*s==' '||*s=='\t') s++; return s; }

static void emit(int addr, unsigned char b){
    if(addr<0||addr>=MAX_ROM){ if(pass==2){fprintf(stderr,"ERROR line %d: addr $%04X out of range\n",lineno,addr); errors++;} return; }
    rom[addr]=b;
    if(addr<rom_lo) rom_lo=addr;
    if(addr>rom_hi) rom_hi=addr;
}

static int label_find(const char *n){
    for(int i=0;i<nlabels;i++) if(strcmp(labels[i].name,n)==0) return labels[i].value;
    return UNDEF;
}
static void label_define(const char *n, int v){
    for(int i=0;i<nlabels;i++) if(strcmp(labels[i].name,n)==0){ labels[i].value=v; return; }
    if(nlabels>=MAX_LABELS){ fprintf(stderr,"ERROR: label table full\n"); errors++; return; }
    strncpy(labels[nlabels].name,n,31); labels[nlabels].name[31]=0;
    labels[nlabels].value=v; nlabels++;
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
        s++; val = (unsigned char)*s;
        if(*s) s++;
        if(*s=='\'') s++;
    } else { *ok=0; return 0; }
    if(neg) val=-val;
    s=skip_ws(s);
    if(*s=='+'||*s=='-'){ int sub=(*s=='-'); s++; s=skip_ws(s); int ok2; int rhs=eval_expr(s,&ok2); val=sub?val-rhs:val+rhs; }
    return val;
}

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

static void assemble_line(char *line){
    char buf[MAX_LINE]; strncpy(buf,line,MAX_LINE-1); buf[MAX_LINE-1]=0;
    upcase(buf);
    char *p=buf; while(*p){ if(*p==';'){*p=0;break;} p++; }
    p=skip_ws(buf); if(!*p) return;
    char lbl[32]="";
    if(!isspace((unsigned char)buf[0])&&buf[0]){
        int i=0;
        while((isalnum((unsigned char)*p)||*p=='_')&&i<31) lbl[i++]=*p++;
        lbl[i]=0; if(*p==':') p++; p=skip_ws(p);
        if(pass==1) label_define(lbl,pc);
    }
    if(!*p) return;
    char mn[16]=""; int mi=0;
    while((isalpha((unsigned char)*p)||isdigit((unsigned char)*p))&&mi<15) mn[mi++]=*p++;
    mn[mi]=0; p=skip_ws(p); if(*p==',') p++; p=skip_ws(p);
    char ops[6][64]={"","","","","",""};
    int nops=split_ops(p,ops,6);

    if(strcmp(mn,"ORG")==0){ int ok,v=eval_expr(ops[0],&ok); if(ok){pc=v; if(pass==1&&*lbl) label_define(lbl,pc);} return; }
    if(strcmp(mn,"EQU")==0){ int ok,v=eval_expr(ops[0],&ok); if(ok) label_define(lbl,v); return; }
    if(strcmp(mn,"DS" )==0){ int ok,n=eval_expr(ops[0],&ok); if(ok){for(int i=0;i<n;i++){emit(pc,0);pc++;}} return; }
    if(strcmp(mn,"DB" )==0){ for(int i=0;i<nops;i++){int ok,v=eval_expr(ops[i],&ok); emit(pc,(unsigned char)(v&0xFF));pc++;} return; }
    if(strcmp(mn,"DW" )==0){ for(int i=0;i<nops;i++){int ok,v=eval_expr(ops[i],&ok); emit(pc,(unsigned char)((v>>8)&0xFF));pc++; emit(pc,(unsigned char)(v&0xFF));pc++;} return; }
    if(strcmp(mn,"END")==0) return;
    /* 2650 hardware constraints — warn on architecturally invalid encodings */
    if(strcmp(mn,"NOP" )==0){ emit(pc,0xC0);pc++; return; }
    if(strcmp(mn,"HALT")==0){ emit(pc,0x40);pc++; return; }
    if(strcmp(mn,"SPSU")==0){ emit(pc,0x12);pc++; return; }
    if(strcmp(mn,"SPSL")==0){ emit(pc,0x13);pc++; return; }
    if(strcmp(mn,"LPSU")==0){ emit(pc,0x92);pc++; return; }
    if(strcmp(mn,"LPSL")==0){ emit(pc,0x93);pc++; return; }
    /* ZBRR: 1-byte instruction $9B, no operand (per 2650 manual).
     * WinArcadia assembler quirk: expects ZBRR with a dummy arg — use RETC,UN
     * in WinArcadia source for compatibility. Our asm2650 is correct. */
    if(strcmp(mn,"ZBRR")==0){ emit(pc,0x9B);pc++; return; }
    /* ZBSR: 2-byte instruction $BB + signed 7-bit page-relative target.
     * Syntax: ZBSR *offset  where offset is 0..$3F (positive) or negative wrap.
     * WinArcadia range check: signed 7-bit = -64..+63 only.
     * Positive range 0..$3F reaches addresses $0000-$003F in page 0. */
    if(strcmp(mn,"ZBSR")==0){
        emit(pc,0xBB); pc++;
        char *a=ops[0]; if(*a=='*') a++;  /* strip optional * prefix */
        int ok,v=eval_expr(a,&ok);
        if(pass==2&&ok&&(v<-64||v>63)){
            fprintf(stderr,"ERROR line %d: ZBSR offset %d out of range (-64..+63)\n",lineno,v);
            errors++;
        }
        emit(pc,(unsigned char)(v&0x7F)); pc++;
        return;
    }
    if(strcmp(mn,"RETC")==0){ int cc=cc_val(ops[0]); if(cc<0){fprintf(stderr,"ERROR line %d: RETC needs EQ/GT/LT/UN\n",lineno);errors++;return;} emit(pc,(unsigned char)(0x14|cc));pc++;return; }
    if(strcmp(mn,"RETE")==0){ int cc=cc_val(ops[0]); if(cc<0){fprintf(stderr,"ERROR line %d: RETE needs EQ/GT/LT/UN\n",lineno);errors++;return;} emit(pc,(unsigned char)(0x34|cc));pc++;return; }
    if(strcmp(mn,"CPSU")==0){ emit(pc,0x74);pc++; int ok,v=eval_expr(ops[0],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"CPSL")==0){ emit(pc,0x75);pc++; int ok,v=eval_expr(ops[0],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"PPSU")==0){ emit(pc,0x76);pc++; int ok,v=eval_expr(ops[0],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"PPSL")==0){ emit(pc,0x77);pc++; int ok,v=eval_expr(ops[0],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"TPSU")==0){ emit(pc,0xB4);pc++; int ok,v=eval_expr(ops[0],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"TPSL")==0){ emit(pc,0xB5);pc++; int ok,v=eval_expr(ops[0],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }
    if(strcmp(mn,"DAR")==0){ int r=reg_val(ops[0]); if(r<0){fprintf(stderr,"ERROR line %d: DAR needs Rn\n",lineno);errors++;return;} emit(pc,(unsigned char)(0x94|r));pc++;return; }
    if(strcmp(mn,"TMI")==0){ int r=reg_val(ops[0]); if(r<0){fprintf(stderr,"ERROR line %d: TMI needs Rn\n",lineno);errors++;return;} emit(pc,(unsigned char)(0xF4|r));pc++; int ok,v=eval_expr(ops[1],&ok); emit(pc,(unsigned char)(v&0xFF));pc++; return; }
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
            if(is_rel){ emit(pc,(unsigned char)(br[i].base_r|field));pc++; if(ok) emit_rel(v,ind); else{emit(pc,0);pc++;} }
            else { emit(pc,(unsigned char)(br[i].base_a|field));pc++; if(ok) emit_abs(v,ind,0); else{emit(pc,0);pc++;emit(pc,0);pc++;} }
            return;
        }
    }
    struct { const char *mn; int base; } bra[]={{"BCTA",0x1C},{"BCFA",0x9C},{"BSTA",0x3C},{"BSFA",0xBC},{NULL,0}};
    for(int i=0;bra[i].mn;i++){
        if(strcmp(mn,bra[i].mn)==0){
            char *cc_s, *addr_s; PARSE_FIELD(ops, nops, cc_s, addr_s);
            int cc=cc_val(cc_s); if(cc<0){fprintf(stderr,"ERROR line %d: %s needs EQ/GT/LT/UN\n",lineno,mn);errors++;return;}
            int ind=0; if(*addr_s=='*'){ind=1;addr_s++;} int ok,v=eval_expr(addr_s,&ok);
            emit(pc,(unsigned char)(bra[i].base|cc));pc++;
            if(ok) emit_abs(v,ind,0); else{emit(pc,0);pc++;emit(pc,0);pc++;}
            return;
        }
    }
    if(strcmp(mn,"BRNA")==0||strcmp(mn,"BIRA")==0||strcmp(mn,"BDRA")==0||strcmp(mn,"BSNA")==0){
        int base=(strcmp(mn,"BRNA")==0)?0x5C:(strcmp(mn,"BIRA")==0)?0xDC:(strcmp(mn,"BDRA")==0)?0xFC:0x7C;
        int r=reg_val(ops[0]); if(r<0){fprintf(stderr,"ERROR line %d: %s needs Rn\n",lineno,mn);errors++;return;}
        char *addr_s=ops[1]; int ind=0; if(*addr_s=='*'){ind=1;addr_s++;} int ok,v=eval_expr(addr_s,&ok);
        emit(pc,(unsigned char)(base|r));pc++; if(ok) emit_abs(v,ind,0); else{emit(pc,0);pc++;emit(pc,0);pc++;} return;
    }
    if(strcmp(mn,"BXA")==0||strcmp(mn,"BSXA")==0){
        int ind=0; char *a=ops[0]; if(*a=='*'){ind=1;a++;} int ok,v=eval_expr(a,&ok);
        emit(pc,(strcmp(mn,"BXA")==0)?0x9F:0xBF);pc++;
        if(ok) emit_abs(v,ind,0); else{emit(pc,0);pc++;emit(pc,0);pc++;} return;
    }
    /* 2650 silicon constraints on Z-mode register-to-register instructions */
    if(strcmp(mn,"ANDZ")==0&&nops>=1){int r=reg_val(ops[0]); if(r==0&&pass==2){fprintf(stderr,"WARN line %d: ANDZ,R0 not valid (would emit HALT $40); use IORZ,R0\n",lineno);}}
    if(strcmp(mn,"STRZ")==0&&nops>=1){int r=reg_val(ops[0]); if(r==0&&pass==2){fprintf(stderr,"WARN line %d: STRZ,R0 not valid (would emit NOP $C0)\n",lineno);}}
    if(strcmp(mn,"LODZ")==0&&nops>=1){int r=reg_val(ops[0]); if(r==0&&pass==2){fprintf(stderr,"WARN line %d: LODZ,R0 result undefined per 2650 manual; use IORZ,R0 to clear R0\n",lineno);}}
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
    for(int addr=rom_lo;addr<=rom_hi;){
        int n=rom_hi-addr+1; if(n>16) n=16;
        unsigned char sum=(unsigned char)(n+(addr>>8)+(addr&0xFF));
        fprintf(f,":%02X%04X00",n,addr);
        for(int i=0;i<n;i++){ fprintf(f,"%02X",rom[addr+i]); sum+=rom[addr+i]; }
        fprintf(f,"%02X\n",(unsigned char)(-sum)); addr+=n;
    }
    fprintf(f,":00000001FF\n");
}

int main(int argc,char *argv[]){
    if(argc<2){ fprintf(stderr,"Usage: asm2650 source.asm [output.hex]\n"); return 1; }
    memset(rom,0xFF,sizeof(rom));
    for(pass=1;pass<=2;pass++){
        FILE *f=fopen(argv[1],"r"); if(!f){fprintf(stderr,"Cannot open '%s'\n",argv[1]);return 1;}
        pc=0; lineno=0; char line[MAX_LINE];
        while(fgets(line,MAX_LINE,f)){ lineno++; int l=strlen(line); while(l>0&&(line[l-1]=='\r'||line[l-1]=='\n')) line[--l]=0; assemble_line(line); }
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
