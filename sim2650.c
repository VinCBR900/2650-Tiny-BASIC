/* ============================================================================
 * sim2650.c  (internal version 1.7)
 * Signetics 2650 simulator for uBASIC2650 / PIPBUG 1 project
 * Build: gcc -Wall -O2 -o sim2650 sim2650.c
 *
 * Usage: sim2650 [-t] [-e addr] [-b addr] [-rx file]
 *                [--allow-ram-image] [--pipbug] image.hex
 *   -t                 CPU trace to stderr
 *   -e addr            Entry point (hex); default $0440 with --pipbug, else $0000
 *   -b addr            Breakpoint (hex)
 *   -rx file           Redirect stdin from file
 *   --allow-ram-image  Allow loading hex outside ROM range (auto-set by --pipbug)
 *   --pipbug           PIPBUG 1 mode:
 *                        · Memory map: ROM $0000-$03FF, RAM $0400-$1BFF
 *                        · ROM-protect only $0000-$03FF
 *                        · Entry default $0440 (after Pipbug 1kB+64B)
 *                        · Intercept COUT=$02B4  CHIN=$0286  CRLF=$008A
 *
 * Changes v1.6 -> v1.7  (2650 manual compliance corrections):
 *   BUG-SIM-07 FIXED: Corrected CC encoding to match manual:
 *              POS=01 ($40), ZERO=00 ($00), NEG=10 ($80).
 *   BUG-SIM-08 FIXED: TPSU/TPSL/TMI now set CC as specified:
 *              EQ if all tested bits are 1, otherwise LT.
 *   BUG-SIM-09 FIXED: ADD/SUB now update IDC from bit-3 carry/borrow.
 *   BUG-SIM-10 FIXED: CPSU/PPSU no longer modify PSU.S (sense) or
 *              reserved PSU bits; only F/II/SP are software-controlled.
 *
 * Changes v1.5 -> v1.6  (Pipbug 1 integration):
 *   BUG-SIM-04 FIXED: Memory map corrected for Pipbug: ROM $0000-$03FF (1kB),
 *              RAM $0400-$1BFF; --pipbug sets entry default to $0440.
 *   BUG-SIM-05 FIXED: STR immediate (mode=1) now consumes the immediate byte
 *              (previously returned without fetching, leaving PC wrong).
 *   BUG-SIM-06 FIXED: --pipbug implicitly enables --allow-ram-image so the
 *              user program hex (loaded at $0440+) is accepted without error.
 *   NOTE: Emu2650.cs ADD/SUB CC uses sign-based SetCCFor — this is WRONG per
 *         the 2650 datasheet. sim2650 carry-based set_cc_add/set_cc_sub are
 *         correct and intentionally differ from Emu2650.cs on those opcodes.
 *
 * Changes v1.4 -> v1.5:
 *   BUG-SIM-01 FIXED: ZBSR fetches signed 7-bit operand as page-relative target
 *   BUG-SIM-02 FIXED: -e <addr> flag sets entry point
 *   BUG-SIM-03 FIXED: --pipbug installs COUT/CHIN/CRLF intercepts
 *   Register banking FIXED: PPSL/CPSL RS bit swaps R1-R3 / R1'-R3'
 *   Indexed LODA dest FIXED: mode-A indexed always delivers to R0
 *   alu_sub carry FIXED: C=1 means no borrow (correct per 2650 manual)
 *
 * CC semantics (correct per 2650 datasheet):
 *   ADD: no_carry->GT(01); carry+zero->EQ(10); carry+nonzero->LT(00)
 *   SUB: no_borrow+nonzero->GT(01); no_borrow+zero->EQ(10); borrow->LT(00)
 *   COM: signed compare unless PSL.COM=1 (unsigned)
 *
 * PSL: CC1[7] CC0[6] IDC[5] RS[4] WC[3] OVF[2] COM[1] C[0]
 * PSU: S[7] F[6] II[5] - - SP[2:0]
 *
 * PIPBUG 1 memory map (Motorola MK3880 / Signetics reference board):
 *   $0000-$03FF  PIPBUG ROM  (1 kB, read-only)
 *   $0400-$043F  PIPBUG RAM  (64 B, stack/scratch)
 *   $0440-$1BFF  User RAM    (program+data, writable)
 *   COUT  $02B4  putchar(R0)
 *   CHIN  $0286  R0 = getchar() — blocking, waits for keypress
 *   CRLF  $008A  print CR LF
 * ============================================================================ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define SIM_VER   "1.7"
#define MEM_SIZE  0x8000

/* Default (non-Pipbug) memory map — a generic 2650 board */
#define DEF_ROM_START 0x0000
#define DEF_ROM_END   0x13FF
#define DEF_RAM_START 0x1400
#define DEF_RAM_END   0x1BFF

/* PIPBUG 1 memory map */
#define PB_ROM_START  0x0000
#define PB_ROM_END    0x03FF   /* 1 kB Pipbug ROM */
#define PB_RAM_START  0x0400   /* 64 B Pipbug RAM + user area */
#define PB_RAM_END    0x1BFF

/* Active map (set at startup) */
static unsigned short ROM_START_A = DEF_ROM_START;
static unsigned short ROM_END_A   = DEF_ROM_END;
static unsigned short RAM_START_A = DEF_RAM_START;
static unsigned short RAM_END_A   = DEF_RAM_END;

static unsigned char mem[MEM_SIZE];
static int rom_protect = 1;
static int mem_warn_count = 0;

/* Registers: [0..3]=bank0, [4..6]=bank1 (R1'..R3') */
static struct {
    unsigned char  R[8];
    unsigned short IAR;
    unsigned char  PSU;
    unsigned char  PSL;
    unsigned short RAS[8];
    int            SP;
} cpu;

/* Register index respecting RS bank bit */
static int ri(int n) {
    if (n == 0) return 0;
    return (cpu.PSL & 0x10) ? (n + 3) : n;
}
#define R(n) cpu.R[ri(n)]

/* PSL/PSU masks */
#define PSU_S   0x80
#define PSU_F   0x40
#define PSU_II  0x20
#define PSU_SP  0x07
#define PSL_CC  0xC0
#define PSL_IDC 0x20
#define PSL_RS  0x10
#define PSL_WC  0x08
#define PSL_OVF 0x04
#define PSL_COM 0x02
#define PSL_C   0x01
#define CC_NEG  0x80
#define CC_POS  0x40
#define CC_ZERO 0x00
#define COND_EQ 0
#define COND_GT 1
#define COND_LT 2
#define COND_UN 3

static int  trace      = 0;
static int  running    = 1;
static int  breakpt    = -1;
static long icount     = 0;
static long maxinstr   = 5000000L;
static int  strict_rom = 1;
static int  run_fault  = 0;
static int  use_pipbug = 0;
static unsigned short entry_point = 0x0000;

/* --- Memory ---------------------------------------------------------------- */
static int addr_ok(unsigned short a) {
    a &= 0x7FFF;
    return (a >= ROM_START_A && a <= ROM_END_A) ||
           (a >= RAM_START_A && a <= RAM_END_A);
}
static unsigned char mrd(unsigned short a) {
    a &= 0x7FFF;
    if (!addr_ok(a)) {
        if (mem_warn_count++ < 16) fprintf(stderr,"WARN: unmap rd $%04X\n",a);
        return 0xFF;
    }
    return mem[a];
}
static void mwr(unsigned short a, unsigned char v) {
    a &= 0x7FFF;
    if (!addr_ok(a)) {
        if (mem_warn_count++ < 16) fprintf(stderr,"WARN: unmap wr $%04X\n",a);
        return;
    }
    if (rom_protect && a >= ROM_START_A && a <= ROM_END_A) {
        fprintf(stderr,"WARN: ROM wr $%04X ignored\n",a); return;
    }
    mem[a] = v;
}
static unsigned char fetch(void) {
    unsigned char b = mrd(cpu.IAR);
    cpu.IAR = (cpu.IAR+1) & 0x7FFF;
    return b;
}

/* --- I/O ------------------------------------------------------------------- */
static unsigned char io_in(void)       { int c = getchar(); return c==EOF ? 0 : (unsigned char)c; }
static void          io_out(unsigned char v) { putchar(v); fflush(stdout); }

/* --- PIPBUG stubs ---------------------------------------------------------- */
static void pb_ret(void) {
    cpu.IAR = cpu.RAS[cpu.SP];
    cpu.SP  = (cpu.SP-1) & 7;
    cpu.PSU = (cpu.PSU & ~PSU_SP) | (cpu.SP & 7);
}
static void pb_cout(void) { io_out(R(0)); pb_ret(); }
static void pb_chin(void) { R(0) = io_in(); pb_ret(); }   /* blocking: waits for keypress */
static void pb_crlf(void) { io_out('\r'); io_out('\n'); pb_ret(); }

/* --- CC -------------------------------------------------------------------- */
static void set_cc(unsigned char r) {
    cpu.PSL &= ~PSL_CC;
    if      (r == 0)                  cpu.PSL |= CC_ZERO;
    else if ((signed char)r > 0)      cpu.PSL |= CC_POS;
    /* else CC_NEG = 0 already */
}
static void set_cc_add(unsigned char r) {
    int c = (cpu.PSL & PSL_C) ? 1 : 0;
    if      (!c)     cpu.PSL = (cpu.PSL & ~PSL_CC) | CC_POS;
    else if (r == 0) cpu.PSL = (cpu.PSL & ~PSL_CC) | CC_ZERO;
    else             cpu.PSL = (cpu.PSL & ~PSL_CC) | CC_NEG;
}
static void set_cc_sub(unsigned char r) {
    int c = (cpu.PSL & PSL_C) ? 1 : 0;
    if      (c && r==0) cpu.PSL = (cpu.PSL & ~PSL_CC) | CC_ZERO;
    else if (c)         cpu.PSL = (cpu.PSL & ~PSL_CC) | CC_POS;
    else                cpu.PSL = (cpu.PSL & ~PSL_CC) | CC_NEG;
}
static int test_cc(int cond) {
    unsigned char cc = cpu.PSL & PSL_CC;
    switch (cond) {
        case COND_EQ: return cc == CC_ZERO;
        case COND_GT: return cc == CC_POS;
        case COND_LT: return cc == CC_NEG;
        default:      return 1;
    }
}

/* --- RAS ------------------------------------------------------------------- */
static void push_ras(unsigned short a) {
    cpu.SP = (cpu.SP+1)&7; cpu.RAS[cpu.SP] = a;
    cpu.PSU = (cpu.PSU & ~PSU_SP) | (cpu.SP & 7);
}
static unsigned short pop_ras(void) {
    unsigned short a = cpu.RAS[cpu.SP];
    cpu.SP = (cpu.SP-1)&7; cpu.PSU = (cpu.PSU & ~PSU_SP) | (cpu.SP & 7);
    return a;
}

/* --- Operand fetch --------------------------------------------------------- */
static int fetch_rel(int *ind) {
    unsigned char b = fetch();
    *ind = (b & 0x80) ? 1 : 0;
    int off = b & 0x7F;
    if (off & 0x40) off |= ~0x7F;
    return off;
}
static unsigned short fetch_abs_nb(int *ind, int *idx) {
    unsigned char b1 = fetch(), b2 = fetch();
    *ind = (b1 & 0x80) ? 1 : 0;
    *idx = (b1 >> 5) & 3;
    return (unsigned short)(((b1 & 0x1F) << 8) | b2);
}
static unsigned short fetch_abs_br(int *ind) {
    unsigned char b1 = fetch(), b2 = fetch();
    *ind = (b1 & 0x80) ? 1 : 0;
    return (unsigned short)(((b1 & 0x7F) << 8) | b2) & 0x7FFF;
}
static unsigned short resolve(unsigned short base, int ind) {
    if (!ind) return base;
    return (unsigned short)(((mrd(base) & 0x7F) << 8) | mrd((base+1) & 0x7FFF));
}

/* --- ALU ------------------------------------------------------------------- */
static unsigned char alu_add(unsigned char a, unsigned char b, int wc) {
    int carry_in = (wc && (cpu.PSL & PSL_C)) ? 1 : 0;
    unsigned int s = a + b + carry_in;
    unsigned int lo = (a & 0x0F) + (b & 0x0F) + carry_in;
    if (s > 255) cpu.PSL |= PSL_C; else cpu.PSL &= ~PSL_C;
    if (lo > 0x0F) cpu.PSL |= PSL_IDC; else cpu.PSL &= ~PSL_IDC;
    unsigned char r = s & 0xFF;
    if ((!(a&0x80)&&!(b&0x80)&&(r&0x80))||((a&0x80)&&(b&0x80)&&!(r&0x80)))
        cpu.PSL |= PSL_OVF; else cpu.PSL &= ~PSL_OVF;
    return r;
}
static unsigned char alu_sub(unsigned char a, unsigned char b, int wb) {
    /* C=1 means no borrow input; borrow output: C=0 means borrow occurred */
    int borrow_in = (wb && !(cpu.PSL & PSL_C)) ? 1 : 0;
    int d = (int)a - (int)b - borrow_in;
    int lo = (int)(a & 0x0F) - (int)(b & 0x0F) - borrow_in;
    /* Set C=1 if no borrow (result fits in 0..255 and a >= b+borrow_in). */
    if (d >= 0) cpu.PSL |= PSL_C; else cpu.PSL &= ~PSL_C;
    /* IDC follows low-nibble borrow for subtraction: 1=no borrow, 0=borrow. */
    if (lo >= 0) cpu.PSL |= PSL_IDC; else cpu.PSL &= ~PSL_IDC;
    unsigned char r = (unsigned char)(d & 0xFF);
    if (((a&0x80)&&!(b&0x80)&&!(r&0x80))||(!(a&0x80)&&(b&0x80)&&(r&0x80)))
        cpu.PSL |= PSL_OVF; else cpu.PSL &= ~PSL_OVF;
    return r;
}

/* --- Execute --------------------------------------------------------------- */
static void execute(void) {
    unsigned short op_pc = cpu.IAR;

    /* PIPBUG intercepts (before opcode fetch so we don't consume ROM bytes) */
    if (use_pipbug) {
        if (op_pc == 0x02B4) { pb_cout(); return; }
        if (op_pc == 0x0286) { pb_chin(); return; }
        if (op_pc == 0x008A) { pb_crlf(); return; }
    }

    unsigned char op = fetch();
    int rn = op & 3;

    if (trace) fprintf(stderr,"[%04X] %02X  R0=%02X R1=%02X R2=%02X R3=%02X PSL=%02X SP=%d\n",
        op_pc, op, R(0),R(1),R(2),R(3), cpu.PSL, cpu.SP);

    if (op == 0x40) { running = 0; return; }          /* HALT */
    if (op == 0xC0) { return; }                        /* NOP */
    if (op == 0x12) { R(0)=cpu.PSU; set_cc(R(0)); return; }  /* SPSU */
    if (op == 0x13) { R(0)=cpu.PSL; set_cc(R(0)); return; }  /* SPSL */
    if (op == 0x92) {                                         /* LPSU */
        /* Manual: only F, II and SP are loaded; preserve external S bit. */
        cpu.PSU = (cpu.PSU & PSU_S) | (R(0) & (PSU_F | PSU_II | PSU_SP));
        return;
    }
    if (op == 0x93) { cpu.PSL=R(0); return; }                 /* LPSL */

    /* RETC / RETE */
    if (op>=0x14&&op<=0x17){ if(test_cc(op&3)) cpu.IAR=pop_ras(); return; }
    if (op>=0x34&&op<=0x37){ if(test_cc(op&3)){ cpu.IAR=pop_ras(); cpu.PSU&=~PSU_II; } return; }

    /* CPSU/CPSL/PPSU/PPSL */
    if (op>=0x74&&op<=0x77){
        unsigned char m=fetch();
        unsigned char pm = m & (PSU_F | PSU_II | PSU_SP); /* SW-writable PSU fields only */
        switch(op){ case 0x74:cpu.PSU&=~pm;break; case 0x75:cpu.PSL&=~m;break;
                    case 0x76:cpu.PSU|=pm;break;  case 0x77:cpu.PSL|=m;break; }
        return;
    }

    /* TPSU / TPSL */
    if (op==0xB4){ unsigned char m=fetch(); cpu.PSL=(cpu.PSL&~PSL_CC)|(((cpu.PSU&m)==m)?CC_ZERO:CC_NEG); return; }
    if (op==0xB5){ unsigned char m=fetch(); cpu.PSL=(cpu.PSL&~PSL_CC)|(((cpu.PSL&m)==m)?CC_ZERO:CC_NEG); return; }

    /* DAR */
    if (op>=0x94&&op<=0x97){
        unsigned char r=R(rn);
        if((r&0x0F)>9||(cpu.PSL&PSL_IDC)) r+=6;
        if((r>>4)>9||(cpu.PSL&PSL_C)) r+=0x60;
        R(rn)=r; set_cc(r); return;
    }

    /* TMI */
    if (op>=0xF4&&op<=0xF7){
        unsigned char m=fetch(), res=R(rn)&m;
        cpu.PSL=(cpu.PSL&~PSL_CC)|((res==m)?CC_ZERO:CC_NEG);
        return;
    }

    /* RRR */
    if (op>=0x50&&op<=0x53){
        unsigned char r=R(rn), old=r, b0=r&1;
        if(cpu.PSL&PSL_WC){
            r=(r>>1)|((cpu.PSL&PSL_C)?0x80:0);
            if(b0)cpu.PSL|=PSL_C;else cpu.PSL&=~PSL_C;
            if(((old^r)&0x80)!=0) cpu.PSL|=PSL_OVF; else cpu.PSL&=~PSL_OVF;
            if(r&0x20) cpu.PSL|=PSL_IDC; else cpu.PSL&=~PSL_IDC;
        }
        else r=(r>>1)|(b0<<7);
        R(rn)=r; set_cc(r); return;
    }

    /* RRL */
    if (op>=0xD0&&op<=0xD3){
        unsigned char r=R(rn), old=r, b7=(r>>7)&1;
        if(cpu.PSL&PSL_WC){
            r=(r<<1)|((cpu.PSL&PSL_C)?1:0);
            if(b7)cpu.PSL|=PSL_C;else cpu.PSL&=~PSL_C;
            if(((old^r)&0x80)!=0) cpu.PSL|=PSL_OVF; else cpu.PSL&=~PSL_OVF;
            if(r&0x20) cpu.PSL|=PSL_IDC; else cpu.PSL&=~PSL_IDC;
        }
        else r=(r<<1)|b7;
        R(rn)=r; set_cc(r); return;
    }

    /* ZBRR */
    if (op==0x9B){
        unsigned char ob=fetch();
        int ind=(ob&0x80)?1:0;
        int off=ob&0x7F; if(off&0x40) off|=~0x7F;
        unsigned short t=(unsigned short)((cpu.IAR & 0x6000) | (off & 0x1FFF));
        if(ind) t=resolve(t,1);
        cpu.IAR=t;
        return;
    }

    /* ZBSR — signed 7-bit page-relative call */
    if (op==0xBB){
        unsigned char ob=fetch();
        int ind=(ob&0x80)?1:0;
        int off=ob&0x7F; if(off&0x40) off|=~0x7F;
        unsigned short target=(unsigned short)((cpu.IAR & 0x6000) | (off & 0x1FFF));
        if(ind) target=resolve(target,1);
        push_ras(cpu.IAR);
        cpu.IAR=target;
        return;
    }

    /* I/O */
    if((op>=0x30&&op<=0x33)||(op>=0x70&&op<=0x73)||(op>=0x54&&op<=0x57))
        { R(rn)=io_in(); set_cc(R(rn)); return; }
    if((op>=0xB0&&op<=0xB3)||(op>=0xF0&&op<=0xF3)||(op>=0xD4&&op<=0xD7))
        { io_out(R(rn)); set_cc(R(rn)); return; }

    /* Branches */
#define BR_BODY_R(cond_expr) do{ int ind,off=fetch_rel(&ind); unsigned short t=(unsigned short)(cpu.IAR+off)&0x7FFF; if(ind)t=resolve(t,1); if(cond_expr)cpu.IAR=t; }while(0)
#define BR_BODY_A(cond_expr) do{ int ind; unsigned short t=fetch_abs_br(&ind); if(ind)t=resolve(t,1); if(cond_expr)cpu.IAR=t; }while(0)
#define BS_BODY_R(cond_expr) do{ int ind,off=fetch_rel(&ind); unsigned short t=(unsigned short)(cpu.IAR+off)&0x7FFF; if(ind)t=resolve(t,1); if(cond_expr){push_ras(cpu.IAR);cpu.IAR=t;} }while(0)
#define BS_BODY_A(cond_expr) do{ int ind; unsigned short t=fetch_abs_br(&ind); if(ind)t=resolve(t,1); if(cond_expr){push_ras(cpu.IAR);cpu.IAR=t;} }while(0)

    if(op>=0x18&&op<=0x1B){ BR_BODY_R( test_cc(op&3)); return; } /* BCTR */
    if(op>=0x1C&&op<=0x1F){ BR_BODY_A( test_cc(op&3)); return; } /* BCTA */
    if(op>=0x98&&op<=0x9A){ BR_BODY_R(!test_cc(op&3)); return; } /* BCFR */
    if(op>=0x9C&&op<=0x9E){ BR_BODY_A(!test_cc(op&3)); return; } /* BCFA */
    if(op>=0x38&&op<=0x3B){ BS_BODY_R( test_cc(op&3)); return; } /* BSTR */
    if(op>=0x3C&&op<=0x3F){ BS_BODY_A( test_cc(op&3)); return; } /* BSTA */
    if(op>=0xB8&&op<=0xBA){ BS_BODY_R(!test_cc(op&3)); return; } /* BSFR */
    if(op>=0xBC&&op<=0xBE){ BS_BODY_A(!test_cc(op&3)); return; } /* BSFA */

    /* BSNR/BSNA — branch to subroutine if register != 0 */
    if(op>=0x78&&op<=0x7B){
        int ind,off=fetch_rel(&ind);
        unsigned short t=(unsigned short)(cpu.IAR+off)&0x7FFF;
        if(ind) t=resolve(t,1);
        if(R(rn)!=0){ push_ras(cpu.IAR); cpu.IAR=t; }
        return;
    }
    if(op>=0x7C&&op<=0x7F){
        int ind; unsigned short t=fetch_abs_br(&ind);
        if(ind) t=resolve(t,1);
        if(R(rn)!=0){ push_ras(cpu.IAR); cpu.IAR=t; }
        return;
    }

    /* BRNR/BRNA — decrement, branch if != 0 */
    if(op>=0x58&&op<=0x5B){ int ind,off=fetch_rel(&ind); unsigned short t=(unsigned short)(cpu.IAR+off)&0x7FFF; if(ind)t=resolve(t,1); R(rn)--; if(R(rn)!=0){cpu.IAR=t;} return; }
    if(op>=0x5C&&op<=0x5F){ int ind; unsigned short t=fetch_abs_br(&ind); if(ind)t=resolve(t,1); R(rn)--; if(R(rn)!=0)cpu.IAR=t; return; }

    /* BIRR/BIRA — increment, branch if != 0 */
    if(op>=0xD8&&op<=0xDB){ int ind,off=fetch_rel(&ind); unsigned short t=(unsigned short)(cpu.IAR+off)&0x7FFF; if(ind)t=resolve(t,1); R(rn)++; if(R(rn)!=0){cpu.IAR=t;} return; }
    if(op>=0xDC&&op<=0xDF){ int ind; unsigned short t=fetch_abs_br(&ind); if(ind)t=resolve(t,1); R(rn)++; if(R(rn)!=0)cpu.IAR=t; return; }

    /* BDRR/BDRA — decrement, branch if != 0 */
    if(op>=0xF8&&op<=0xFB){ int ind,off=fetch_rel(&ind); unsigned short t=(unsigned short)(cpu.IAR+off)&0x7FFF; if(ind)t=resolve(t,1); R(rn)--; if(R(rn)!=0){cpu.IAR=t;} return; }
    if(op>=0xFC){ int ind; unsigned short t=fetch_abs_br(&ind); if(ind)t=resolve(t,1); R(rn)--; if(R(rn)!=0)cpu.IAR=t; return; }

    /* BXA/BSXA */
    if(op==0x9F||op==0xBF){ int ind; unsigned short t=fetch_abs_br(&ind); if(ind)t=resolve(t,1); t=(t+R(rn))&0x7FFF; if(op==0xBF)push_ras(cpu.IAR); cpu.IAR=t; return; }

    /* ALU */
    int grp=-1, mode=(op>>2)&3;
    if              (op<=0x0F)                           grp=0; /* LOD */
    else if(op>=0x20&&op<=0x2F)                          grp=1; /* EOR */
    else if(op>=0x40&&op<=0x4F)                          grp=2; /* AND */
    else if(op>=0x60&&op<=0x6F)                          grp=3; /* IOR */
    else if(op>=0x80&&op<=0x8F)                          grp=4; /* ADD */
    else if(op>=0xA0&&op<=0xAF)                          grp=5; /* SUB */
    else if(op>=0xC0&&op<=0xCF&&op!=0xC0&&(op<0xC4||op>0xC7)) grp=6; /* STR */
    else if(op>=0xE0&&op<=0xEF)                          grp=7; /* COM */

    if (grp >= 0) {
        unsigned char operand=0;
        unsigned short eff=0;
        int ind=0, idx=0;

        /* STR group — no immediate mode */
        if (grp==6) {
            switch(mode) {
                case 0: cpu.R[ri(rn)]=cpu.R[ri(0)]; return;       /* STRZ */
                case 1: fetch(); return;                            /* invalid — consume byte so PC stays correct */
                case 2: { int off=fetch_rel(&ind); eff=(unsigned short)(cpu.IAR+off)&0x7FFF;
                          if(ind){eff=resolve(eff,1);} mwr(eff,R(rn)); return; }
                case 3: { eff=fetch_abs_nb(&ind,&idx); if(ind)eff=resolve(eff,1);
                          if(idx==1){R(rn)++;eff=(eff+R(rn))&0x7FFF;}
                          else if(idx==2){R(rn)--;eff=(eff+R(rn))&0x7FFF;}
                          else if(idx==3){eff=(eff+R(rn))&0x7FFF;}
                          mwr(eff,R(rn)); return; }
            }
        }

        /* Fetch operand for LOD/EOR/AND/IOR/ADD/SUB/COM */
        switch(mode) {
            case 0: operand=R(rn); break;
            case 1: operand=fetch(); break;
            case 2: { int off=fetch_rel(&ind); eff=(unsigned short)(cpu.IAR+off)&0x7FFF;
                      if(ind){eff=resolve(eff,1);} operand=mrd(eff); break; }
            case 3: { eff=fetch_abs_nb(&ind,&idx); if(ind)eff=resolve(eff,1);
                      if(idx==1){R(rn)++;eff=(eff+R(rn))&0x7FFF;}
                      else if(idx==2){R(rn)--;eff=(eff+R(rn))&0x7FFF;}
                      else if(idx==3){eff=(eff+R(rn))&0x7FFF;}
                      operand=mrd(eff); break; }
        }

        /* Indexed A-mode delivers result to R0 per 2650 manual */
        int to_r0 = (mode==3 && idx!=0);
        unsigned char a=(mode==0)?R(0):R(rn);
        unsigned char b=(mode==0)?R(rn):operand;
        unsigned char res; int wc=(cpu.PSL&PSL_WC)?1:0;

        switch(grp) {
            case 0: res=b; if(mode==0||to_r0)cpu.R[ri(0)]=res; else cpu.R[ri(rn)]=res; set_cc(res); break;
            case 1: res=R(0)^b; cpu.R[ri(0)]=res; set_cc(res); break;
            case 2: res=a&b; if(mode==0||to_r0)cpu.R[ri(0)]=res; else cpu.R[ri(rn)]=res; set_cc(res); break;
            case 3: res=a|b; if(mode==0||to_r0)cpu.R[ri(0)]=res; else cpu.R[ri(rn)]=res; set_cc(res); break;
            case 4: res=alu_add(a,b,wc); if(mode==0||to_r0)cpu.R[ri(0)]=res; else cpu.R[ri(rn)]=res; set_cc_add(res); break;
            case 5: res=alu_sub(a,b,wc); if(mode==0||to_r0)cpu.R[ri(0)]=res; else cpu.R[ri(rn)]=res; set_cc_sub(res); break;
            case 7: {
                int com=(cpu.PSL&PSL_COM)?1:0;
                if(com){ if(a>b)cpu.PSL=(cpu.PSL&~PSL_CC)|CC_POS; else if(a==b)cpu.PSL=(cpu.PSL&~PSL_CC)|CC_ZERO; else cpu.PSL=(cpu.PSL&~PSL_CC)|CC_NEG; }
                else { signed char sa=a,sb=b; if(sa>sb)cpu.PSL=(cpu.PSL&~PSL_CC)|CC_POS; else if(sa==sb)cpu.PSL=(cpu.PSL&~PSL_CC)|CC_ZERO; else cpu.PSL=(cpu.PSL&~PSL_CC)|CC_NEG; }
                return;
            }
        }
        return;
    }

    fprintf(stderr,"WARN [%04X]: unhandled opcode $%02X\n",op_pc,op);
    run_fault=1; running=0;
}

/* --- HEX loader ------------------------------------------------------------ */
static int load_hex(const char *fn) {
    FILE *f=fopen(fn,"r"); if(!f){fprintf(stderr,"Cannot open '%s'\n",fn);return 0;}
    char line[128]; int loaded=0,viol=0;
    while(fgets(line,128,f)){
        if(line[0]!=':') continue;
        int n,addr,type; sscanf(line+1,"%02x%04x%02x",&n,&addr,&type);
        if(type==1) break; else if(type!=0) continue;
        for(int i=0;i<n;i++){
            int b; unsigned short a=(unsigned short)(addr&0x7FFF);
            sscanf(line+9+i*2,"%02x",&b);
            if(strict_rom&&(a<ROM_START_A||a>ROM_END_A)) viol++;
            else if(addr_ok(a)){mem[a]=(unsigned char)b;loaded++;}
            else fprintf(stderr,"WARN: unmapped $%04X\n",a);
            addr++;
        }
    }
    fclose(f);
    if(viol){fprintf(stderr,"ERROR: %d byte(s) outside ROM $%04X-$%04X\n",viol,ROM_START_A,ROM_END_A);return 0;}
    fprintf(stderr,"Loaded %d bytes from '%s'\n",loaded,fn);
    return loaded;
}

/* --- main ------------------------------------------------------------------ */
int main(int argc, char *argv[]) {
    fprintf(stderr,"sim2650 v%s\n",SIM_VER);
    if(argc<2){
        fprintf(stderr,"Usage: sim2650 [-t] [-e addr] [-b addr] [-rx file]\n"
                       "               [--allow-ram-image] [--pipbug] image.hex\n");
        return 1;
    }
    const char *hexfile=NULL, *rxfile=NULL;
    int entry_explicit=0;
    for(int i=1;i<argc;i++){
        if     (!strcmp(argv[i],"-t"))              trace=1;
        else if(!strcmp(argv[i],"-e")&&i+1<argc)  { entry_point=(unsigned short)strtol(argv[++i],NULL,16); entry_explicit=1; }
        else if(!strcmp(argv[i],"-b")&&i+1<argc)    breakpt=(int)strtol(argv[++i],NULL,16);
        else if(!strcmp(argv[i],"-rx")&&i+1<argc)   rxfile=argv[++i];
        else if(!strcmp(argv[i],"--allow-ram-image")) strict_rom=0;
        else if(!strcmp(argv[i],"--pipbug"))         use_pipbug=1;
        else hexfile=argv[i];
    }

    /* Apply Pipbug 1 memory map when --pipbug active */
    if(use_pipbug){
        ROM_START_A = PB_ROM_START;
        ROM_END_A   = PB_ROM_END;
        RAM_START_A = PB_RAM_START;
        RAM_END_A   = PB_RAM_END;
        strict_rom  = 0;              /* user prog loads at $0440, not in ROM range */
        if(!entry_explicit) entry_point = 0x0440;
        fprintf(stderr,"PIPBUG 1 mode: ROM $%04X-$%04X  RAM $%04X-$%04X\n",
                PB_ROM_START, PB_ROM_END, PB_RAM_START, PB_RAM_END);
        fprintf(stderr,"  COUT=$02B4  CHIN=$0286  CRLF=$008A  entry=$%04X\n", entry_point);
    }

    if(!hexfile){fprintf(stderr,"No HEX file\n");return 1;}
    if(rxfile){if(!freopen(rxfile,"r",stdin)){fprintf(stderr,"Cannot open RX '%s'\n",rxfile);return 1;}fprintf(stderr,"RX: '%s'\n",rxfile);}

    memset(mem,0xFF,sizeof(mem));
    if(!load_hex(hexfile)) return 1;
    memset(&cpu,0,sizeof(cpu));
    cpu.IAR=entry_point; cpu.PSU=PSU_II;
    fprintf(stderr,"Running from $%04X...\n\n",entry_point);

    while(running&&icount<maxinstr){
        if(breakpt>=0&&(int)cpu.IAR==breakpt){fprintf(stderr,"\n*** BREAKPOINT $%04X ***\n",cpu.IAR);break;}
        execute(); icount++;
    }
    if(icount>=maxinstr) fprintf(stderr,"\n*** Instruction limit (%ld) ***\n",maxinstr);
    fprintf(stderr,"\nHalted after %ld instructions\n",icount);
    fprintf(stderr,"R0=$%02X R1=$%02X R2=$%02X R3=$%02X\n",R(0),R(1),R(2),R(3));
    fprintf(stderr,"IAR=$%04X PSU=$%02X PSL=$%02X CC=%d\n",cpu.IAR,cpu.PSU,cpu.PSL,(cpu.PSL&PSL_CC)>>6);
    if(run_fault) return 2;
    if(icount>=maxinstr) return 3;
    return 0;
}
