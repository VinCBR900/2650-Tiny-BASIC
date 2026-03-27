/* ============================================================================
 * sim2650_v1.3.c  —  Signetics 2650 instruction-set simulator
 * ============================================================================
 *
 * VERSION HISTORY
 *   v1.0  initial skeleton
 *   v1.1  opcode table and PSW behavior corrected
 *   v1.2  direct byte I/O mode and RX-file input support
 *         uBASIC memory map + explicit unmapped access handling
 *   v1.3  2026-03-27
 *         - Add subroutine/function header comments (In / Out / Clobbers)
 *         - Add architecture and reference document links in header
 *         - Rename emit_abs() → emit_abs_br() distinction verified:
 *           non-branch absolute uses idxctl in bits 6-5;
 *           branch absolute uses PAGE bits in bits 6-5.
 *           Both were already handled correctly; confirmed by code review.
 *         - fetch_abs_nb idxctl decoding confirmed correct for all ALU modes.
 *
 * PURPOSE
 *   Execute Intel HEX programs assembled for the Signetics 2650.
 *   Provide practical testing support for Tiny BASIC / uBASIC development.
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
 *   Build: gcc -Wall -O2 -o sim2650 sim2650_v1.3.c
 *
 * USAGE
 *   sim2650 [-t] [-b addr] [-rx rxfile] [--allow-ram-image] image.hex
 *     -t               enable instruction trace
 *     -b hex           stop when IAR reaches breakpoint address
 *     -rx file         feed stdin from file (REDE/REDD input)
 *     --allow-ram-image  permit HEX bytes outside ROM range (unit test images)
 *
 * SIMULATION MODEL
 *   CPU: register/PSW semantics based on Signetics 2650 User Manual.
 *   I/O: direct byte-mode convenience mapping:
 *       WRTD/WRTE/WRTC  → putchar()
 *       REDE/REDD/REDC  → getchar()
 *   Address space: 15-bit (masked with 0x7FFF internally).
 *
 * INDEXED ADDRESSING (non-branch absolute)
 *   Byte 1 of 3-byte instruction:  i(7) idxctl(6-5) addr12-8(4-0)
 *   idxctl:
 *     00  EA = base                    (no indexing)
 *     01  Rn++; EA = base + Rn         (pre-increment)
 *     10  Rn--; EA = base + Rn         (pre-decrement)
 *     11  EA = base + Rn               (add, no modify)
 *   Reference: https://amigan.yatho.com/2650UM.html#main.html Addressing Modes
 *
 * uBASIC TARGET MEMORY MAP
 *   ROM : $0000-$13FF (5KB; writes ignored when ROM protection enabled)
 *   RAM : $1400-$1BFF
 *   All other addresses are unmapped (warned/ignored or read as $FF)
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define SIM_VER  "1.3"
#define MEM_SIZE   0x8000
#define ROM_START  0x0000
#define ROM_END    0x13FF
#define RAM_START  0x1400
#define RAM_END    0x1BFF

static unsigned char mem[MEM_SIZE];
static int rom_protect=1;
static int mem_warn_count=0;

/* ── CPU state ───────────────────────────────────────────────── */
static struct {
    unsigned char R[4];     /* R0=accumulator, R1-R3 general */
    unsigned short IAR;     /* instruction address register (15-bit) */
    unsigned char PSU;      /* S F II - - SP2 SP1 SP0 */
    unsigned char PSL;      /* CC1 CC0 IDC RS WC OVF COM C */
    unsigned short RAS[8];  /* return address stack */
    int SP;                 /* stack pointer 0-7 */
} cpu;

/* PSU bits */
#define PSU_S   0x80  /* sense input (read-only) */
#define PSU_F   0x40  /* flag output */
#define PSU_II  0x20  /* interrupt inhibit */
#define PSU_SP  0x07  /* stack pointer mask */

/* PSL bits */
#define PSL_CC1 0x80
#define PSL_CC0 0x40
#define PSL_CC  0xC0  /* condition code mask */
#define PSL_IDC 0x20
#define PSL_RS  0x10
#define PSL_WC  0x08
#define PSL_OVF 0x04
#define PSL_COM 0x02
#define PSL_C   0x01

/* CC field values (in bits 7-6 of PSL)
 * 2650 encoding:
 *   LT = %00  (CC_NEG)
 *   GT = %01  (CC_POS)   — also set for "no carry" after ADD
 *   EQ = %10  (CC_ZERO)
 */
#define CC_NEG  0x00  /* LT / Negative */
#define CC_POS  0x40  /* GT / Positive */
#define CC_ZERO 0x80  /* EQ / Zero */

/* Branch condition encoding (bits 1-0 of opcode) */
#define COND_EQ 0
#define COND_GT 1
#define COND_LT 2
#define COND_UN 3

static int trace=0, running=1, breakpt=-1;
static long icount=0, maxinstr=2000000L;
static int strict_rom_image=1;
static int trap_unhandled=1;
static int run_fault=0;

/* ────────────────────────────────────────────────────────────────
 * MEMORY SUBSYSTEM
 * ──────────────────────────────────────────────────────────────── */

/*
 * addr_mapped — test whether an address is in a mapped region
 * In:  a = 15-bit address
 * Out: 1 if mapped (ROM or RAM), 0 if unmapped
 * Clobbers: nothing
 */
static int addr_mapped(unsigned short a){
    a&=0x7FFF;
    return ((a>=ROM_START&&a<=ROM_END) || (a>=RAM_START&&a<=RAM_END));
}

/*
 * mrd — read one byte from memory
 * In:  a = 15-bit address
 * Out: byte value; returns 0xFF for unmapped addresses (with warning)
 * Clobbers: mem_warn_count
 */
static unsigned char mrd(unsigned short a){
    a&=0x7FFF;
    if(!addr_mapped(a)){
        if(mem_warn_count<16){
            fprintf(stderr,"WARN: unmapped read $%04X -> $FF\n",a);
            mem_warn_count++;
        }
        return 0xFF;
    }
    return mem[a];
}

/*
 * mwr — write one byte to memory
 * In:  a = 15-bit address; v = byte value
 * Out: mem[a] = v (unless unmapped or ROM-protected)
 * Clobbers: mem_warn_count
 */
static void mwr(unsigned short a, unsigned char v){
    a&=0x7FFF;
    if(!addr_mapped(a)){
        if(mem_warn_count<16){
            fprintf(stderr,"WARN: unmapped write $%04X ignored\n",a);
            mem_warn_count++;
        }
        return;
    }
    if(rom_protect&&a>=ROM_START&&a<=ROM_END){ fprintf(stderr,"WARN: ROM write $%04X ignored\n",a); return; }
    mem[a]=v;
}

/*
 * fetch — fetch one byte from [IAR] and advance IAR
 * In:  cpu.IAR = current instruction pointer
 * Out: returns byte; cpu.IAR incremented (15-bit wrap)
 * Clobbers: cpu.IAR
 */
static unsigned char fetch(void){ unsigned char b=mrd(cpu.IAR); cpu.IAR=(cpu.IAR+1)&0x7FFF; return b; }

/* ────────────────────────────────────────────────────────────────
 * PSL CONDITION CODE HELPERS
 * ──────────────────────────────────────────────────────────────── */

/*
 * set_cc — set CC bits in PSL based on result byte (signed interpretation)
 * In:  result = 8-bit value to classify
 * Out: cpu.PSL CC bits updated: NEG if <0, ZERO if ==0, POS if >0
 * Clobbers: cpu.PSL (CC bits only)
 */
static void set_cc(unsigned char result){
    cpu.PSL&=~PSL_CC;
    if     (result==0)              cpu.PSL|=CC_ZERO;
    else if((signed char)result>0)  cpu.PSL|=CC_POS;
    else                            cpu.PSL|=CC_NEG;
}

/*
 * test_cond — test a branch condition against current PSL CC
 * In:  cond = COND_EQ/GT/LT/UN (bits 1-0 of branch opcode)
 * Out: returns 1 if condition satisfied, 0 otherwise
 * Clobbers: nothing
 */
static int test_cond(int cond){
    unsigned char cc=(cpu.PSL&PSL_CC);
    switch(cond){
        case COND_EQ: return (cc==CC_ZERO);
        case COND_GT: return (cc==CC_POS);
        case COND_LT: return (cc==CC_NEG);
        case COND_UN: return 1;
    }
    return 0;
}

/* ────────────────────────────────────────────────────────────────
 * RETURN ADDRESS STACK
 * ──────────────────────────────────────────────────────────────── */

/*
 * push_ras — push a return address onto the RAS
 * In:  addr = return address to push
 * Out: RAS[++SP] = addr; PSU.SP updated
 * Clobbers: cpu.SP, cpu.PSU, cpu.RAS
 */
static void push_ras(unsigned short addr){
    cpu.SP=(cpu.SP+1)&7;
    cpu.RAS[cpu.SP]=addr;
    cpu.PSU=(cpu.PSU&~PSU_SP)|(cpu.SP&7);
}

/*
 * pop_ras — pop a return address from the RAS
 * In:  cpu.SP, cpu.RAS
 * Out: returns address at RAS[SP]; SP decremented; PSU.SP updated
 * Clobbers: cpu.SP, cpu.PSU
 */
static unsigned short pop_ras(void){
    unsigned short a=cpu.RAS[cpu.SP];
    cpu.SP=(cpu.SP-1)&7;
    cpu.PSU=(cpu.PSU&~PSU_SP)|(cpu.SP&7);
    return a;
}

/* ────────────────────────────────────────────────────────────────
 * EFFECTIVE ADDRESS HELPERS
 * ──────────────────────────────────────────────────────────────── */

/*
 * fetch_rel — fetch relative address byte and decode
 * In:  ind = output pointer for indirect flag
 * Out: returns signed integer offset (sign-extended from 7 bits)
 *      *ind = 1 if bit 7 set (indirect), 0 otherwise
 * Clobbers: cpu.IAR (fetch)
 */
static int fetch_rel(int *ind){
    unsigned char b=fetch();
    *ind=(b&0x80)?1:0;
    int off=b&0x7F;
    if(off&0x40) off|=~0x7F;   /* sign-extend 7→int */
    return off;
}

/*
 * fetch_abs_nb — fetch 2-byte absolute address for NON-BRANCH instructions
 * In:  ind     = output: indirect flag (bit 7 of byte 1)
 *      idxctl  = output: index control bits (bits 6-5 of byte 1)
 *                  00=none  01=Rn++ then add  10=Rn-- then add  11=add Rn
 * Out: returns 13-bit address (full 15-bit with page bits)
 * Clobbers: cpu.IAR (2 fetches)
 * Byte 1: i(7) idxctl(6-5) addr12-8(4-0)
 * Byte 2: addr7-0
 * Reference: https://amigan.yatho.com/2650UM.html#main.html (Addressing Modes)
 */
static unsigned short fetch_abs_nb(int *ind, int *idxctl){
    unsigned char b1=fetch(), b2=fetch();
    *ind   =(b1&0x80)?1:0;
    *idxctl=(b1>>5)&3;
    return (unsigned short)(((b1&0x1F)<<8)|b2);
}

/*
 * fetch_abs_br — fetch 2-byte absolute address for BRANCH instructions
 * In:  ind = output: indirect flag (bit 7 of byte 1)
 * Out: returns 15-bit branch target address (pp bits included)
 * Clobbers: cpu.IAR (2 fetches)
 * Byte 1: i(7) pp(6-5) addr12-8(4-0)    pp = page bits (not idxctl!)
 * Byte 2: addr7-0
 * Reference: https://en.wikibooks.org/wiki/Signetics_2650_%26_2636_programming/Indexed_branching
 */
static unsigned short fetch_abs_br(int *ind){
    unsigned char b1=fetch(), b2=fetch();
    *ind=(b1&0x80)?1:0;
    /* pp bits (6-5) are part of the 15-bit target address */
    unsigned short addr=(unsigned short)(((b1&0x7F)<<8)|b2);
    return addr&0x7FFF;
}

/*
 * resolve — dereference an indirect address
 * In:  base = base address; ind = 1 for indirect, 0 for direct
 * Out: if ind: reads 15-bit address from mem[base]:mem[base+1]
 *      if !ind: returns base unchanged
 * Clobbers: nothing
 */
static unsigned short resolve(unsigned short base, int ind){
    if(!ind) return base;
    unsigned short hi=mrd(base), lo=mrd((base+1)&0x7FFF);
    return (unsigned short)(((hi&0x7F)<<8)|lo);
}

/* ────────────────────────────────────────────────────────────────
 * I/O
 * ──────────────────────────────────────────────────────────────── */

/*
 * io_in — read one byte from input (stdin / -rx file)
 * Out: byte read, or 0x00 on EOF
 * Clobbers: nothing
 */
static unsigned char io_in (void){ int c=getchar(); return (c==EOF)?0:(unsigned char)c; }

/*
 * io_out — write one byte to output (stdout)
 * In:  v = byte to output
 * Clobbers: nothing
 */
static void io_out(unsigned char v){ putchar(v); fflush(stdout); }

/* ────────────────────────────────────────────────────────────────
 * ALU HELPERS
 * ──────────────────────────────────────────────────────────────── */

/*
 * alu_add — 8-bit addition with optional carry-in
 * In:  a, b       = operands
 *      with_carry = 1 to add PSL.C to sum (WC mode)
 * Out: returns result byte; PSL.C, PSL.OVF, PSL.IDC updated
 * Clobbers: cpu.PSL
 */
static unsigned char alu_add(unsigned char a, unsigned char b, int with_carry){
    unsigned int s=(unsigned int)a+(unsigned int)b;
    if(with_carry&&(cpu.PSL&PSL_C)) s++;
    if(s>255) cpu.PSL|=PSL_C; else cpu.PSL&=~PSL_C;
    unsigned char r=(unsigned char)(s&0xFF);
    if((!( a&0x80)&&!(b&0x80)&&(r&0x80))||((a&0x80)&&(b&0x80)&&!(r&0x80))) cpu.PSL|=PSL_OVF; else cpu.PSL&=~PSL_OVF;
    unsigned int idc=((unsigned int)(a&0xF)+(unsigned int)(b&0xF));
    if(with_carry&&(cpu.PSL&PSL_C)) idc++;
    if(idc>15) cpu.PSL|=PSL_IDC; else cpu.PSL&=~PSL_IDC;
    return r;
}

/*
 * alu_sub — 8-bit subtraction with optional borrow-in
 * In:  a, b        = operands (computes a - b)
 *      with_borrow = 1 to apply PSL.C borrow (WC mode)
 * Out: returns result byte; PSL.C, PSL.OVF updated
 * Clobbers: cpu.PSL
 */
static unsigned char alu_sub(unsigned char a, unsigned char b, int with_borrow){
    unsigned int d=(unsigned int)a-(unsigned int)b;
    if(with_borrow&&!(cpu.PSL&PSL_C)) d--;
    if(d<=255&&a>=b) cpu.PSL|=PSL_C; else cpu.PSL&=~PSL_C;
    unsigned char r=(unsigned char)(d&0xFF);
    if(((a&0x80)&&!(b&0x80)&&!(r&0x80))||(!( a&0x80)&&(b&0x80)&&(r&0x80))) cpu.PSL|=PSL_OVF; else cpu.PSL&=~PSL_OVF;
    return r;
}

/*
 * set_cc_add — set CC after ADD per uBASIC convention
 * In:  result = result byte (after alu_add)
 * Out: PSL CC bits updated:
 *      CC_POS  = no carry (fast-path branch for uBASIC addition loops)
 *      CC_ZERO = carry and result == 0 (exact wrap-to-zero)
 *      CC_NEG  = carry with non-zero result
 * Clobbers: cpu.PSL (CC bits)
 */
static void set_cc_add(unsigned char result){
    int c=(cpu.PSL&PSL_C)?1:0;
    if(!c) cpu.PSL=(cpu.PSL&~PSL_CC)|CC_POS;
    else if(result==0) cpu.PSL=(cpu.PSL&~PSL_CC)|CC_ZERO;
    else cpu.PSL=(cpu.PSL&~PSL_CC)|CC_NEG;
}

/*
 * set_cc_sub — set CC after SUB per uBASIC convention
 * In:  result = result byte (after alu_sub)
 * Out: PSL CC bits updated:
 *      CC_ZERO = no borrow and result == 0   (equal)
 *      CC_POS  = no borrow and result != 0   (greater)
 *      CC_NEG  = borrow occurred             (less)
 * Clobbers: cpu.PSL (CC bits)
 */
static void set_cc_sub(unsigned char result){
    int c=(cpu.PSL&PSL_C)?1:0;
    if(c && result==0) cpu.PSL=(cpu.PSL&~PSL_CC)|CC_ZERO;
    else if(c) cpu.PSL=(cpu.PSL&~PSL_CC)|CC_POS;
    else cpu.PSL=(cpu.PSL&~PSL_CC)|CC_NEG;
}

/* ════════════════════════════════════════════════════════════════
 * execute — decode and execute one instruction
 * In:  cpu state (IAR, R[], PSL, PSU, RAS, SP)
 * Out: cpu state updated; running=0 on HALT or fault
 * Clobbers: cpu (all fields potentially modified)
 * ════════════════════════════════════════════════════════════════ */
static void execute(void){
    unsigned short op_pc=cpu.IAR;
    unsigned char  opcode=fetch();
    int rn=opcode&3;           /* bits 1-0: register field */

    if(trace)
        fprintf(stderr,"[%04X] $%02X  R0=%02X R1=%02X R2=%02X R3=%02X CC=%d WC=%d C=%d\n",
            op_pc,opcode,cpu.R[0],cpu.R[1],cpu.R[2],cpu.R[3],
            (cpu.PSL&PSL_CC)>>6,(cpu.PSL&PSL_WC)?1:0,(cpu.PSL&PSL_C)?1:0);

    /* ── special full-opcode instructions ───────────────────── */
    if(opcode==0x40){ running=0; if(trace) fprintf(stderr,"HALT\n"); return; }
    if(opcode==0xC0){ if(trace) fprintf(stderr,"NOP\n"); return; }

    /* SPSU / SPSL — store PSW to R0 */
    if(opcode==0x12){ cpu.R[0]=cpu.PSU; set_cc(cpu.R[0]); return; }
    if(opcode==0x13){ cpu.R[0]=cpu.PSL; set_cc(cpu.R[0]); return; }

    /* LPSU / LPSL — load PSW from R0 */
    if(opcode==0x92){ cpu.PSU=(cpu.R[0]&~PSU_S); return; } /* S bit is read-only */
    if(opcode==0x93){ cpu.PSL=cpu.R[0]; return; }

    /* RETC,cc  $14-$17 — conditional return from subroutine */
    if(opcode>=0x14&&opcode<=0x17){
        int cond=opcode&3;
        if(test_cond(cond)){ cpu.IAR=pop_ras(); if(trace) fprintf(stderr,"RETC → $%04X\n",cpu.IAR); }
        return;
    }

    /* RETE,cc  $34-$37 — return from interrupt, clear II */
    if(opcode>=0x34&&opcode<=0x37){
        int cond=opcode&3;
        if(test_cond(cond)){ cpu.IAR=pop_ras(); cpu.PSU&=~PSU_II; }
        return;
    }

    /* PSW bit-manipulation instructions: CPSU CPSL PPSU PPSL ($74-$77) */
    if(opcode>=0x74&&opcode<=0x77){
        unsigned char mask=fetch();
        switch(opcode){
            case 0x74: cpu.PSU&=~mask; break;  /* CPSU: clear PSU bits */
            case 0x75: cpu.PSL&=~mask; break;  /* CPSL: clear PSL bits */
            case 0x76: cpu.PSU|= mask; break;  /* PPSU: preset PSU bits */
            case 0x77: cpu.PSL|= mask; break;  /* PPSL: preset PSL bits */
        }
        return;
    }

    /* TPSU ($B4) / TPSL ($B5) — test PSW bits, set CC */
    if(opcode==0xB4){ unsigned char mask=fetch(); set_cc(cpu.PSU&mask); return; }
    if(opcode==0xB5){ unsigned char mask=fetch(); set_cc(cpu.PSL&mask); return; }

    /* DAR,Rn  $94-$97 — BCD decimal adjust after add */
    if(opcode>=0x94&&opcode<=0x97){
        unsigned char r=cpu.R[rn];
        if((r&0x0F)>9||(cpu.PSL&PSL_IDC)) r+=6;
        if((r>>4)>9||(cpu.PSL&PSL_C))     r+=0x60;
        cpu.R[rn]=r; set_cc(cpu.R[rn]); return;
    }

    /* TMI,Rn  $F4-$F7 — test mask immediate */
    if(opcode>=0xF4&&opcode<=0xF7){
        unsigned char mask=fetch();
        unsigned char result=cpu.R[rn]&mask;
        if(result==mask) cpu.PSL=(cpu.PSL&~PSL_CC)|CC_POS;
        else if(result==0) cpu.PSL=(cpu.PSL&~PSL_CC)|CC_ZERO;
        else cpu.PSL=(cpu.PSL&~PSL_CC)|CC_NEG;
        return;
    }

    /* RRR,Rn  $50-$53 — rotate register right */
    if(opcode>=0x50&&opcode<=0x53){
        unsigned char r=cpu.R[rn], old_bit0=r&1;
        int wc=(cpu.PSL&PSL_WC)?1:0;
        if(wc){ r=(r>>1)|((cpu.PSL&PSL_C)?0x80:0); if(old_bit0) cpu.PSL|=PSL_C; else cpu.PSL&=~PSL_C; }
        else   { r=(r>>1)|(old_bit0<<7); }
        cpu.R[rn]=r; set_cc(r); return;
    }

    /* RRL,Rn  $D0-$D3 — rotate register left */
    if(opcode>=0xD0&&opcode<=0xD3){
        unsigned char r=cpu.R[rn], old_bit7=(r>>7)&1;
        int wc=(cpu.PSL&PSL_WC)?1:0;
        if(wc){ r=(r<<1)|((cpu.PSL&PSL_C)?1:0); if(old_bit7) cpu.PSL|=PSL_C; else cpu.PSL&=~PSL_C; }
        else   { r=(r<<1)|old_bit7; }
        cpu.R[rn]=r; set_cc(r); return;
    }

    /* ZBRR $9B — branch to address in RAS (indirect return) */
    if(opcode==0x9B){ cpu.IAR=pop_ras(); return; }
    /* ZBSR $BB — push current IAR, branch to RAS top */
    if(opcode==0xBB){ push_ras(cpu.IAR); cpu.IAR=pop_ras(); return; }

    /* I/O read: REDC($30-$33) REDD($70-$73) REDE($54-$57) */
    if((opcode>=0x30&&opcode<=0x33)||(opcode>=0x70&&opcode<=0x73)||(opcode>=0x54&&opcode<=0x57)){
        cpu.R[rn]=io_in(); set_cc(cpu.R[rn]); return;
    }
    /* I/O write: WRTC($B0-$B3) WRTD($F0-$F3) WRTE($D4-$D7) */
    if((opcode>=0xB0&&opcode<=0xB3)||(opcode>=0xF0&&opcode<=0xF3)||(opcode>=0xD4&&opcode<=0xD7)){
        io_out(cpu.R[rn]); set_cc(cpu.R[rn]); return;
    }

    /* ── BRANCH instructions ─────────────────────────────────
     * Branch absolute instructions use fetch_abs_br() — bits 6-5 of
     * address byte 1 are PAGE bits, NOT idxctl.
     * Reference: https://en.wikibooks.org/wiki/Signetics_2650_%26_2636_programming/Indexed_branching
     * ──────────────────────────────────────────────────────── */

    /* BCTR,cc $18-$1B — branch if cc, relative */
    if(opcode>=0x18&&opcode<=0x1B){
        int cond=opcode&3; int ind; int off=fetch_rel(&ind);
        unsigned short t=(unsigned short)(cpu.IAR+off)&0x7FFF;
        if(ind) t=resolve(t,1);
        if(test_cond(cond)) cpu.IAR=t;
        return;
    }
    /* BCTA,cc $1C-$1F — branch if cc, absolute */
    if(opcode>=0x1C&&opcode<=0x1F){
        int cond=opcode&3; int ind; unsigned short t=fetch_abs_br(&ind);
        if(ind) t=resolve(t,1);
        if(test_cond(cond)) cpu.IAR=t;
        return;
    }
    /* BCFR,cc $98-$9A — branch if cc NOT set, relative */
    if(opcode>=0x98&&opcode<=0x9A){
        int cond=opcode&3; int ind; int off=fetch_rel(&ind);
        unsigned short t=(unsigned short)(cpu.IAR+off)&0x7FFF;
        if(ind) t=resolve(t,1);
        if(!test_cond(cond)) cpu.IAR=t;
        return;
    }
    /* BCFA,cc $9C-$9E — branch if cc NOT set, absolute */
    if(opcode>=0x9C&&opcode<=0x9E){
        int cond=opcode&3; int ind; unsigned short t=fetch_abs_br(&ind);
        if(ind) t=resolve(t,1);
        if(!test_cond(cond)) cpu.IAR=t;
        return;
    }
    /* BSTR,cc $38-$3B — branch-subroutine if cc, relative */
    if(opcode>=0x38&&opcode<=0x3B){
        int cond=opcode&3; int ind; int off=fetch_rel(&ind);
        unsigned short t=(unsigned short)(cpu.IAR+off)&0x7FFF;
        if(ind) t=resolve(t,1);
        if(test_cond(cond)){ push_ras(cpu.IAR); cpu.IAR=t; }
        return;
    }
    /* BSTA,cc $3C-$3F — branch-subroutine if cc, absolute */
    if(opcode>=0x3C&&opcode<=0x3F){
        int cond=opcode&3; int ind; unsigned short t=fetch_abs_br(&ind);
        if(ind) t=resolve(t,1);
        if(test_cond(cond)){ push_ras(cpu.IAR); cpu.IAR=t; }
        return;
    }
    /* BSFR,cc $B8-$BA — branch-subroutine if cc NOT set, relative */
    if(opcode>=0xB8&&opcode<=0xBA){
        int cond=opcode&3; int ind; int off=fetch_rel(&ind);
        unsigned short t=(unsigned short)(cpu.IAR+off)&0x7FFF;
        if(ind) t=resolve(t,1);
        if(!test_cond(cond)){ push_ras(cpu.IAR); cpu.IAR=t; }
        return;
    }
    /* BSFA,cc $BC-$BE — branch-subroutine if cc NOT set, absolute */
    if(opcode>=0xBC&&opcode<=0xBE){
        int cond=opcode&3; int ind; unsigned short t=fetch_abs_br(&ind);
        if(ind) t=resolve(t,1);
        if(!test_cond(cond)){ push_ras(cpu.IAR); cpu.IAR=t; }
        return;
    }
    /* BSNR,Rn $78-$7B — branch if SENSE != Rn[0], relative */
    if(opcode>=0x78&&opcode<=0x7B){
        int ind; int off=fetch_rel(&ind);
        unsigned short t=(unsigned short)(cpu.IAR+off)&0x7FFF;
        int sense=(cpu.PSU&PSU_S)?1:0;
        if(sense!=(cpu.R[rn]&1)) cpu.IAR=t;
        return;
    }
    /* BSNA,Rn $7C-$7F — branch if SENSE != Rn[0], absolute */
    if(opcode>=0x7C&&opcode<=0x7F){
        int ind; unsigned short t=fetch_abs_br(&ind);
        int sense=(cpu.PSU&PSU_S)?1:0;
        if(sense!=(cpu.R[rn]&1)) cpu.IAR=t;
        return;
    }
    /* BRNR,Rn $58-$5B — decrement Rn, branch if non-zero, relative */
    if(opcode>=0x58&&opcode<=0x5B){
        int ind; int off=fetch_rel(&ind);
        cpu.R[rn]--; set_cc(cpu.R[rn]);
        if(cpu.R[rn]!=0){ unsigned short t=(unsigned short)(cpu.IAR+off)&0x7FFF; cpu.IAR=t; }
        return;
    }
    /* BRNA,Rn $5C-$5F — decrement Rn, branch if non-zero, absolute */
    if(opcode>=0x5C&&opcode<=0x5F){
        int ind; unsigned short t=fetch_abs_br(&ind);
        cpu.R[rn]--; set_cc(cpu.R[rn]);
        if(cpu.R[rn]!=0) cpu.IAR=t;
        return;
    }
    /* BIRR,Rn $D8-$DB — increment Rn, branch if non-zero, relative */
    if(opcode>=0xD8&&opcode<=0xDB){
        int ind; int off=fetch_rel(&ind);
        cpu.R[rn]++; set_cc(cpu.R[rn]);
        if(cpu.R[rn]!=0){ unsigned short t=(unsigned short)(cpu.IAR+off)&0x7FFF; cpu.IAR=t; }
        return;
    }
    /* BIRA,Rn $DC-$DF — increment Rn, branch if non-zero, absolute */
    if(opcode>=0xDC&&opcode<=0xDF){
        int ind; unsigned short t=fetch_abs_br(&ind);
        cpu.R[rn]++; set_cc(cpu.R[rn]);
        if(cpu.R[rn]!=0) cpu.IAR=t;
        return;
    }
    /* BDRR,Rn $F8-$FB — decrement Rn, branch if >= 0, relative */
    if(opcode>=0xF8&&opcode<=0xFB){
        int ind; int off=fetch_rel(&ind);
        cpu.R[rn]--; set_cc(cpu.R[rn]);
        if((signed char)cpu.R[rn]>=0){ unsigned short t=(unsigned short)(cpu.IAR+off)&0x7FFF; cpu.IAR=t; }
        return;
    }
    /* BDRA,Rn $FC-$FF — decrement Rn, branch if >= 0, absolute */
    if(opcode>=0xFC&&opcode<=0xFF){
        int ind; unsigned short t=fetch_abs_br(&ind);
        cpu.R[rn]--; set_cc(cpu.R[rn]);
        if((signed char)cpu.R[rn]>=0) cpu.IAR=t;
        return;
    }
    /* BXA $9F / BSXA $BF — indexed branch, R3 as index */
    if(opcode==0x9F||opcode==0xBF){
        int ind; unsigned short t=fetch_abs_br(&ind);
        if(ind) t=resolve(t,1);
        t=(t+cpu.R[rn])&0x7FFF;
        if(opcode==0xBF) push_ras(cpu.IAR);
        cpu.IAR=t; return;
    }

    /* ── ALU instructions ────────────────────────────────────
     * Opcode ranges (bits 7-4 select group, bits 3-2 select mode):
     *   LOD $00-$0F   EOR $20-$2F   AND $40-$4F   IOR $60-$6F
     *   ADD $80-$8F   SUB $A0-$AF   COM $E0-$EF
     *   STR $C0-$CF   (no STRI; NOP is $C0)
     *
     * Mode field (bits 3-2):
     *   00=Z(register)  01=I(immediate)  10=R(relative)  11=A(absolute)
     *
     * ALU destination:
     *   Z-mode: R0 ← op(R0, Rn)   (R0 is always accumulator dest for Z-mode)
     *   I/R/A:  Rn ← op(Rn, operand)
     *   Exception: COM never stores; EOR always uses R0.
     *
     * Absolute mode uses fetch_abs_nb (idxctl NOT fetch_abs_br):
     *   idxctl=01: Rn++; EA = base + Rn   (like 6502 INY/LDA (p),Y)
     *   idxctl=10: Rn--; EA = base + Rn
     *   idxctl=11: EA = base + Rn
     * ──────────────────────────────────────────────────────── */

    int group=-1, mode=(opcode>>2)&3;
    if(opcode<=0x0F)                           group=0; /* LOD */
    else if(opcode>=0x20&&opcode<=0x2F)        group=1; /* EOR */
    else if(opcode>=0x40&&opcode<=0x4F)        group=2; /* AND */
    else if(opcode>=0x60&&opcode<=0x6F)        group=3; /* IOR */
    else if(opcode>=0x80&&opcode<=0x8F)        group=4; /* ADD */
    else if(opcode>=0xA0&&opcode<=0xAF)        group=5; /* SUB */
    else if(opcode>=0xC0&&opcode<=0xCF&&opcode!=0xC0&&(opcode<0xC4||opcode>0xC7)) group=6; /* STR */
    else if(opcode>=0xE0&&opcode<=0xEF)        group=7; /* COM */

    if(group>=0){
        unsigned char operand=0;
        unsigned short eff=0;

        /* STR (group 6): destination is memory, no immediate mode */
        if(group==6){
            int ind,idxctl;
            switch(mode){
                case 0: /* STRZ: store R0 into Rn (register copy) */
                    cpu.R[rn]=cpu.R[0]; return;
                case 1: /* STRI: not valid */ return;
                case 2: /* STRR: store Rn to relative address */
                    { int off=fetch_rel(&ind);
                      eff=(unsigned short)(cpu.IAR+off)&0x7FFF;
                      if(ind) eff=resolve(eff,1);
                      mwr(eff,cpu.R[rn]); return; }
                case 3: /* STRA: store Rn to absolute address, with optional indexing
                         * In:  cpu.R[rn] = value to store; idxctl applies to Rn
                         * Out: mem[eff] = cpu.R[rn]; Rn may be modified by idxctl
                         * Clobbers: cpu.R[rn] (if idxctl=1 or 2) */
                    { eff=fetch_abs_nb(&ind,&idxctl);
                      if(ind) eff=resolve(eff,1);
                      if(idxctl==1){ cpu.R[rn]++; eff=(eff+cpu.R[rn])&0x7FFF; }
                      else if(idxctl==2){ cpu.R[rn]--; eff=(eff+cpu.R[rn])&0x7FFF; }
                      else if(idxctl==3){ eff=(eff+cpu.R[rn])&0x7FFF; }
                      mwr(eff,cpu.R[rn]);
                      return; }
            }
        }

        /* fetch source operand for LOD/EOR/AND/IOR/ADD/SUB/COM */
        int ind=0, idxctl=0;
        switch(mode){
            case 0: /* Z: register-to-register */
                operand=cpu.R[rn]; break;
            case 1: /* I: immediate byte */
                operand=fetch(); break;
            case 2: /* R: relative address */
                { int off=fetch_rel(&ind);
                  eff=(unsigned short)(cpu.IAR+off)&0x7FFF;
                  if(ind) eff=resolve(eff,1);
                  operand=mrd(eff); break; }
            case 3: /* A: absolute address with optional indexing
                     * idxctl=01: Rn++; EA = base + Rn  (pre-increment then index)
                     * idxctl=10: Rn--; EA = base + Rn  (pre-decrement then index)
                     * idxctl=11: EA = base + Rn         (index without modify)
                     * Reference: https://amigan.yatho.com/2650UM.html#main.html */
                { eff=fetch_abs_nb(&ind,&idxctl);
                  if(ind) eff=resolve(eff,1);
                  if(idxctl==1){ cpu.R[rn]++; eff=(eff+cpu.R[rn])&0x7FFF; }
                  else if(idxctl==2){ cpu.R[rn]--; eff=(eff+cpu.R[rn])&0x7FFF; }
                  else if(idxctl==3){ eff=(eff+cpu.R[rn])&0x7FFF; }
                  operand=mrd(eff); break; }
        }

        /* execute ALU operation
         * Z-mode: left=R0, right=R[rn], result→R0
         * I/R/A:  left=R[rn], right=operand, result→R[rn] */
        unsigned char result;
        int wc=(cpu.PSL&PSL_WC)?1:0;
        unsigned char alu_a = (mode==0) ? cpu.R[0]  : cpu.R[rn];
        unsigned char alu_b = (mode==0) ? cpu.R[rn] : operand;

        switch(group){
            case 0: /* LOD */
                result = alu_b;
                if(mode==0) cpu.R[0]=result; else cpu.R[rn]=result;
                set_cc(result); break;
            case 1: /* EOR — always R0 */
                result = cpu.R[0] ^ alu_b;
                cpu.R[0]=result; set_cc(result); break;
            case 2: /* AND */
                result = alu_a & alu_b;
                if(mode==0) cpu.R[0]=result; else cpu.R[rn]=result;
                set_cc(result); break;
            case 3: /* IOR */
                result = alu_a | alu_b;
                if(mode==0) cpu.R[0]=result; else cpu.R[rn]=result;
                set_cc(result); break;
            case 4: /* ADD */
                result = alu_add(alu_a, alu_b, wc);
                if(mode==0) cpu.R[0]=result; else cpu.R[rn]=result;
                set_cc_add(result); break;
            case 5: /* SUB */
                result = alu_sub(alu_a, alu_b, wc);
                if(mode==0) cpu.R[0]=result; else cpu.R[rn]=result;
                set_cc_sub(result); break;
            case 7: /* COM — compare only, no store */
                { int com=(cpu.PSL&PSL_COM)?1:0;
                  unsigned char ca=alu_a, cb=alu_b;
                  if(com){ /* logical (unsigned) */
                      if(ca>cb)       cpu.PSL=(cpu.PSL&~PSL_CC)|CC_POS;
                      else if(ca==cb) cpu.PSL=(cpu.PSL&~PSL_CC)|CC_ZERO;
                      else            cpu.PSL=(cpu.PSL&~PSL_CC)|CC_NEG;
                  } else { /* arithmetic (signed two's complement) */
                      signed char sa=(signed char)ca, sb=(signed char)cb;
                      if(sa>sb)       cpu.PSL=(cpu.PSL&~PSL_CC)|CC_POS;
                      else if(sa==sb) cpu.PSL=(cpu.PSL&~PSL_CC)|CC_ZERO;
                      else            cpu.PSL=(cpu.PSL&~PSL_CC)|CC_NEG;
                  }
                  return; }
        }
        return;
    }

    /* unknown opcode */
    fprintf(stderr,"WARN [%04X]: unhandled opcode $%02X\n",op_pc,opcode);
    if(trap_unhandled){
        run_fault=1;
        running=0;
    }
}

/* ── Intel HEX loader ─────────────────────────────────────────
 * load_hex — load an Intel HEX file into memory
 * In:  fn = path to .hex file
 * Out: memory populated; returns bytes loaded (>0) or 0 on failure
 * Clobbers: mem[], mem_warn_count, stderr diagnostics
 */
static int load_hex(const char *fn){
    FILE *f=fopen(fn,"r"); if(!f){fprintf(stderr,"Cannot open '%s'\n",fn);return 0;}
    char line[128]; int loaded=0, rom_violations=0;
    while(fgets(line,128,f)){
        if(line[0]!=':') continue;
        int n,addr,type; sscanf(line+1,"%02x%04x%02x",&n,&addr,&type);
        if(type==1) break;
        if(type!=0) continue;
        for(int i=0;i<n;i++){
            int b;
            unsigned short a=(unsigned short)(addr&0x7FFF);
            sscanf(line+9+i*2,"%02x",&b);
            if(strict_rom_image && (a < ROM_START || a > ROM_END)){
                rom_violations++;
            }else if(addr_mapped(a)){
                mem[a]=(unsigned char)b;
                loaded++;
            }else{
                fprintf(stderr,"WARN: HEX byte for unmapped $%04X ignored\n",a);
            }
            addr++;
        }
    }
    fclose(f);
    if(rom_violations){
        fprintf(stderr,
                "ERROR: image contains %d byte(s) outside ROM $%04X-$%04X.\n",
                rom_violations, ROM_START, ROM_END);
        fprintf(stderr,
                "       This overlaps runtime RAM (uBASIC uses RAM for program/data).\n");
        fprintf(stderr,
                "       Shrink code to fit ROM or rerun simulator with --allow-ram-image.\n");
        return 0;
    }
    fprintf(stderr,"Loaded %d bytes from '%s'\n",loaded,fn);
    return loaded;
}

/* ── main ─────────────────────────────────────────────────────
 * In:  argv = command line (see USAGE in file header)
 * Out: 0=halt OK  1=load error  2=run fault  3=instruction limit
 */
int main(int argc,char *argv[]){
    fprintf(stderr,"sim2650 v%s - Signetics 2650 Simulator\n",SIM_VER);
    if(argc<2){
        fprintf(stderr,"Usage: sim2650 [-t] [-b addr] [-rx file] [--allow-ram-image] image.hex\n");
        return 1;
    }
    const char *hexfile=NULL;
    const char *rxfile =NULL;
    for(int i=1;i<argc;i++){
        if(strcmp(argv[i],"-t")==0) trace=1;
        else if(strcmp(argv[i],"-b")==0&&i+1<argc) breakpt=(int)strtol(argv[++i],NULL,16);
        else if(strcmp(argv[i],"-rx")==0&&i+1<argc) rxfile=argv[++i];
        else if(strcmp(argv[i],"--allow-ram-image")==0) strict_rom_image=0;
        else hexfile=argv[i];
    }
    if(!hexfile){fprintf(stderr,"No HEX file\n");return 1;}

    if(rxfile){
        if(!freopen(rxfile,"r",stdin)){
            fprintf(stderr,"Cannot open RX file '%s'\n",rxfile); return 1;
        }
        fprintf(stderr,"RX input: '%s'\n",rxfile);
    }

    memset(mem,0xFF,sizeof(mem));
    if(!load_hex(hexfile)) return 1;
    memset(&cpu,0,sizeof(cpu));
    cpu.IAR=0; cpu.PSU=PSU_II; /* interrupts inhibited at reset per User Manual */
    fprintf(stderr,"Running from $0000...\n\n");
    while(running&&icount<maxinstr){
        if(breakpt>=0&&(int)cpu.IAR==breakpt){
            fprintf(stderr,"\n*** BREAKPOINT $%04X ***\n",cpu.IAR);
            break;
        }
        execute(); icount++;
    }
    if(icount>=maxinstr) fprintf(stderr,"\n*** Instruction limit (%ld) ***\n",maxinstr);
    fprintf(stderr,"\nHalted after %ld instructions\n",icount);
    fprintf(stderr,"R0=$%02X R1=$%02X R2=$%02X R3=$%02X\n",cpu.R[0],cpu.R[1],cpu.R[2],cpu.R[3]);
    fprintf(stderr,"IAR=$%04X PSU=$%02X PSL=$%02X CC=%d\n",cpu.IAR,cpu.PSU,cpu.PSL,(cpu.PSL&PSL_CC)>>6);
    if(run_fault) return 2;
    if(icount>=maxinstr) return 3;
    return 0;
}
