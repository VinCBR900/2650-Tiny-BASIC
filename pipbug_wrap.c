/* pipbug_wrap.c — PIPBUG 1 simulator using the WinArcadia 2650 CPU core
 * Version: 1.2
 * Date:    2026-04-15
 *
 * Purpose:
 *   Wraps 2650.c (the WinArcadia CPU core, compiled -DGAMER to strip all UI
 *   code) with just enough scaffold to run uBASIC2650 under PIPBUG 1 and
 *   produce identical results to WinArcadia.
 *
 * Usage:
 *   ./pipbug_wrap [options] program.hex
 *
 * Options:
 *   -t              Trace every instruction to stderr
 *   -b 0xADDR       Breakpoint at address (hex); pauses and dumps state
 *   -m 0xADDR LEN   Dump LEN bytes from address at halt
 *   -s              Step mode: pause at every instruction, press Enter to continue
 *   -n N            Instruction limit (default 5000000, 0=unlimited)
 *   --chin 0xADDR   CHIN intercept address (default 0x0286)
 *   --cout 0xADDR   COUT intercept address (default 0x02B4)
 *   --crlf 0xADDR   CRLF intercept address (default 0x008A)
 *   --entry 0xADDR  Program entry address (default 0x0440)
 *   -h, --help      Show usage help
 *   -v, --version   Show version number
 *
 * PIPBUG 1 intercepts (entry points match Oracle / uBASIC2650 EQUs):
 *   COUT  $02B4  putchar(R0)
 *   CHIN  $0286  R0 = getchar() (blocking)
 *   CRLF  $008A  putchar('\r'); putchar('\n')
 *
 * Build:
 *   gcc -Wall -O2 -DGAMER -o pipbug_wrap pipbug_wrap.c
 *
 * Change history:
 *   v1.2  Added configurable program entry address switch (--entry).
 *   v1.1  Added configurable CHIN/COUT/CRLF intercept addresses and
 *         explicit help/version command-line switches.
 *   v1.0  Initial release.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/* ── aa.h stubs ─────────────────────────────────────────────────────────── */

/* Basic types */
typedef unsigned char   UBYTE;
typedef unsigned short  UWORD;
typedef unsigned long   ULONG;
typedef char            TEXT;
typedef char *          STRPTR;
typedef int             FLAG;
typedef unsigned char   MEMFLAG;
typedef unsigned char   ASCREEN;    /* unused in GAMER mode */

/* Storage-class macros */
#define IMPORT          extern
#define EXPORT          /* nothing */
#define MODULE          static
#define FAST            /* nothing */
#define PERSIST         static
#define DISCARD         (void)

/* Syntax helpers */
#define acase           break; case
#define elif            else if
#define EOS             '\0'

/* PSL bits (per 2650 data sheet and 2650.c source) */
#define PSL_CC          0xC0    /* bits 7:6 — condition code               */
#define PSL_IDC         0x20    /* bit 5    — inter-digit carry             */
#define PSL_RS          0x10    /* bit 4    — register select               */
#define PSL_WC          0x08    /* bit 3    — with carry                    */
#define PSL_OVF         0x04    /* bit 2    — overflow                      */
#define PSL_COM         0x02    /* bit 1    — compare mode (0=signed)       */
#define PSL_C           0x01    /* bit 0    — carry / borrow                */

/* PSU bits */
#define PSU_S           0x80    /* bit 7 — SENSE input                      */
#define PSU_F           0x40    /* bit 6 — FLAG output                      */
#define PSU_II          0x20    /* bit 5 — interrupt inhibit                */
#define PSU_SP          0x07    /* bits 2:0 — stack pointer                 */
#define PSU_WRITABLE_A  0x60    /* F + II are software-writable via PPSU    */
#define PSU_WRITABLE_B  0x60

/* memflags bit masks (only the ones 2650.c tests internally) */
#define ASIC            0x01
#define NOREAD          0x02
#define NOWRITE         0x04
#define READONCE        0x08
#define AUDIBLE         0x10
#define VISIBLE         0x20
#define SPECIALREAD     0x40
#define SPECIALWRITE    0x80
#define WATCHPOINT      0x00    /* no watchpoints in GAMER mode */
#define STEPPOINT       0x00
#define BIOS            0x00

/* Coverage flags — needed only for GAMER mode array size (coverage[] itself
   is declared EXPORT in 2650.c and we declare the array here).
   Values are never used in GAMER mode so any non-zero definitions are fine. */
#define COVERAGE_OPCODE     0x0001
#define COVERAGE_OPERAND    0x0002
#define COVERAGE_READ       0x0004
#define COVERAGE_WRITE      0x0008
#define COVERAGE_ADDRESS    0x0010
#define COVERAGE_CALLS      0x0020
#define COVERAGE_CALLSINT   0x0040
#define COVERAGE_JUMPS      0x0080
#define COVERAGE_JUMPSINT   0x0100
#define COVERAGE_LOOPSTART  0x0200
#define COVERAGE_LOOPEND    0x0400
#define COVERAGE_DEPTH      0x0800
#define COVERAGE_BITSHIFT   12

/* Dimension constants — only array sizes matter in GAMER mode */
#define ALLTOKENS       32800   /* > 32768 + a few register tokens          */
#define FRIENDLYLENGTH  32
#define MACHINES        64
#define KNOWNGAMES      1024
#define CPUTIPS         16

/* 8K page arithmetic — verbatim from aa.h */
#define PAGE            0x6000
#define NONPAGE         0x1FFF
#define PLEN            0x2000
#define AMSK            0x7FFF
/* Uncomment QUICK to skip page-wrap arithmetic (faster but less accurate) */
/* #define QUICK */
#ifdef QUICK
    #define WRAPMEM(x)  (iar + x)
#else
    #define WRAPMEM(x)  ((iar & PAGE) + ((iar + x) & NONPAGE))
#endif

/* LLL() — localisation stub, returns the fallback string directly */
#define LLL(id, fallback)   (fallback)

/* ── Opaque struct stubs (arrays declared but never accessed in GAMER mode) */
struct ConditionalStruct { int dummy; };
struct FlagNameStruct    { int dummy; };
struct IOPortStruct      { int dummy; int contents; };
struct KnownStruct       { int complang; };
struct MachineStruct     { int dummy; int readonce; };
struct OpcodeStruct      { const char *name; };

/* Machine IDs (only PIPBUG used) */
#define PIPBUG          10
#define BINBUG          11
#define TWIN            12
#define CD2650          13
#define ARCADIA         14
#define INTERTON        15
#define ELEKTOR         16
#define MALZAK          17
#define ZACCARIA        18
#define INSTRUCTOR      19
#define PHUNSY          20
#define SELBST          21
#define CL_8KB13        1


/* DOS/BIOS version constants referenced in 2650.c */
#define PIPBUG_PIPBUG2BIOS  2
#define DOS_P1DOS           1
#define CD2650_IPL          1

/* zprintf — all logging goes to /dev/null in our wrapper */
#define zprintf(pen, ...)   ((void)0)
#define TEXTPEN_LOG         0
#define TEXTPEN_DEBUG       0
#define TEXTPEN_TRACE       0

/* set_pause — no-op */
#define set_pause(t)        ((void)0)
#define TYPE_LOG            0
#define TYPE_BP             0
#define TYPE_RUNTO          0

/* ISQWERTY — keyboard style check, never true */
#define ISQWERTY            0

/* Watch modes */
#define WATCH_NONE          0
#define WATCH_ALL           1

/* ── IMPORT variables — globals provided by this wrapper ─────────────────── */

/* All int flags */
int  binbug_baudrate   = 0;
int  binbug_biosver    = 0;
int  cd2650_biosver    = 0;
int  cd2650_dosver     = 0;
int  connected         = 0;
int  cpuy              = 0;     /* raster line — unused */
int  fastcd2650        = 0;
int  logbios           = 0;
int  loginefficient    = 0;
int  log_illegal       = 0;
int  log_interrupts    = 0;
int  logsubroutines    = 0;
int  logreadwrites     = 0;
int  machine           = PIPBUG;
int  memmap            = 0;
int  malzak_x          = 0;
int  n1=0, n2=0, n3=0, n4=0;
int  otl               = 0;
int  pausebreak        = 0;
int  pauselog          = 0;
int  pipbug_biosver    = PIPBUG_PIPBUG2BIOS;
int  phunsy_biosver    = 0;
int  pvibase           = 0;
int  recmode           = 0;
int  runtointerrupt    = 0;
int  runtoloopend      = 0;
int  s_id              = 0;
int  s_io              = 0;
int  selbst_biosver    = 0;
int  starscroll        = 0;
int  step              = 0;
int  style             = 0;
int  supercpu          = 0;
int  tapemode          = 0;
int  tapekind          = 0;
int  trace             = 0;
int  useguideray       = 0;
int  verbosity         = 0;
int  watchreads        = 0;
int  watchwrites       = WATCH_NONE;
int  wheremusicmouse[2]= {0,0};
int  whichcpu          = 0;
int  whichgame         = -1;

/* Text buffers */
TEXT addrstr[32 + 1 + 1]   = {0};
TEXT friendly[FRIENDLYLENGTH + 1] = {0};

/* UBYTE arrays */
UBYTE g_bank1[1024]     = {0};
UBYTE g_bank2[16]       = {0};
UBYTE glow              = 0;
UBYTE ininterrupt       = 8;    /* 8 = not in interrupt */
UBYTE lb_bank           = 0;
UBYTE malzak_field[16][16] = {{0}};
UBYTE memory_effects[512]  = {0};
UBYTE s_tapeport        = 0;
UBYTE s_toggles         = 0;
UBYTE keys_column[7]    = {0};
UBYTE tapeskewage       = 0;
UBYTE tt_scrncode       = 0;

/* UWORD arrays */
UWORD console[4]        = {0};
UWORD mirror_r[32768];          /* identity map, filled at startup */
UWORD mirror_w[32768];          /* identity map, filled at startup */

/* ULONG imports */
ULONG binbug_interface  = 0;
ULONG cpb               = 0;
ULONG frames            = 0;
ULONG inverttape        = 0;
ULONG oldcycles         = 0;
ULONG paused            = 0;
ULONG sound             = 0;
ULONG tapewriteprotect  = 0;
ULONG tt_scrntill       = 0;
ULONG verbosetape       = 0;

/* FLAG imports */
FLAG  halted            = 0;
FLAG  priflag[32]       = {0};

/* Pointer imports */
UBYTE *TapeCopy         = NULL;
FILE  *TapeHandle       = NULL;

/* Struct arrays (GAMER mode: allocated but never dereferenced) */
struct ConditionalStruct bp[32768];
struct ConditionalStruct wp[ALLTOKENS];
struct FlagNameStruct    flagname[CPUTIPS];
struct IOPortStruct      ioport[258];
const  struct KnownStruct known[KNOWNGAMES] = {{0}};
struct MachineStruct     machines[MACHINES];
struct OpcodeStruct      opcodes[3][256];
TEXT   asciiname_short[259][3 + 1];
ASCREEN screen[1][1];           /* dummy — BOXWIDTH/BOXHEIGHT unknown */
const STRPTR pristring[32] = {0};

/* Forward declarations for static functions in 2650.c that are called before
   their definition in GAMER mode (no #ifndef GAMER guard on the call sites) */
static void logindirectbios(void);

/* ── Include the WinArcadia CPU core ─────────────────────────────────────── */
#include "2650.c"

/* ── After including 2650.c the EXPORT variables are now in scope ────────── */
/* psu, psl, r[7], iar, ras[8], memory[32768], memflags[], cycles_2650 etc. */

/* ── Wrapper-specific globals ────────────────────────────────────────────── */

static int  bp_addr       = -1;     /* -b breakpoint address (-1 = none)   */
static int  dump_addr     = -1;     /* -m dump base address                 */
static int  dump_len      = 0;
static long inst_limit    = 5000000;
static int  step_mode     = 0;      /* -s step mode                        */
static int  trace_mode    = 0;      /* -t trace mode                       */
static int  eof_hit       = 0;
static UWORD chin_addr    = 0x0286; /* --chin                              */
static UWORD cout_addr    = 0x02B4; /* --cout                              */
static UWORD crlf_addr    = 0x008A; /* --crlf                              */
static UWORD entry_addr   = 0x0440; /* --entry                             */

static void print_usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [options] program.hex\n"
        "Options:\n"
        "  -t              Trace every instruction to stderr\n"
        "  -s              Step mode: pause each instruction\n"
        "  -b 0xADDR       Breakpoint at address (hex)\n"
        "  -m 0xADDR LEN   Dump LEN bytes from address at halt\n"
        "  -n LIMIT        Instruction limit (default 5000000, 0=unlimited)\n"
        "  --chin 0xADDR   CHIN intercept address (default 0x0286)\n"
        "  --cout 0xADDR   COUT intercept address (default 0x02B4)\n"
        "  --crlf 0xADDR   CRLF intercept address (default 0x008A)\n"
        "  --entry 0xADDR  Program entry address (default 0x0440)\n"
        "  -h, --help      Show this help message\n"
        "  -v, --version   Show version\n",
        prog);
}

static void print_version(void)
{
    fprintf(stderr, "pipbug_wrap v1.2\n");
}

/* ── Minimal PIPBUG stubs ─────────────────────────────────────────────────
 *  We do NOT simulate PIPBUG ROM.  Instead we intercept the three entry
 *  points before each instruction fetch and handle them ourselves.
 *  After handling, we pop the RAS and continue (mimicking RETC,UN).
 */
static void pb_ret(void)
{
    pullras();  /* pops iar from RAS, matching BSTA that called us */
}

static void pb_cout(void)
{
    putchar((unsigned char)r[0]);
    fflush(stdout);
    pb_ret();
}

static void pb_chin(void)
{
    int c = getchar();
    if (c == EOF) { eof_hit = 1; r[0] = 0; }
    else          { r[0] = (UBYTE)c; }
    pb_ret();
}

static void pb_crlf(void)
{
    putchar('\r');
    putchar('\n');
    fflush(stdout);
    pb_ret();
}

/* ── Trace / debug helpers ───────────────────────────────────────────────── */

static const char *cc_name(void)
{
    switch (psl & PSL_CC) {
        case 0x00: return "EQ";
        case 0x40: return "GT";
        case 0x80: return "LT";
        default:   return "UN";
    }
}

static void print_state(void)
{
    fprintf(stderr,
        "[%04X] %02X  R0=%02X R1=%02X R2=%02X R3=%02X "
        "PSL=%02X(%s) PSU=%02X SP=%d\n",
        iar, memory[WRAPMEM(0)],
        r[0], r[1], r[2], r[3],
        psl, cc_name(),
        psu, (int)(psu & PSU_SP));
}

static void print_halt_summary(long count)
{
    fprintf(stderr, "\nHalted after %ld instructions\n", count);
    fprintf(stderr, "R0=$%02X R1=$%02X R2=$%02X R3=$%02X\n",
            r[0], r[1], r[2], r[3]);
    fprintf(stderr, "IAR=$%04X PSU=$%02X PSL=$%02X CC=%s\n",
            iar, psu, psl, cc_name());
    if (dump_addr >= 0) {
        fprintf(stderr, "\nMEM $%04X+%d:\n  ", dump_addr, dump_len);
        for (int i = 0; i < dump_len; i++) {
            fprintf(stderr, "%02X ", memory[(dump_addr + i) & AMSK]);
            if ((i & 15) == 15 && i < dump_len-1) fprintf(stderr, "\n  ");
        }
        fprintf(stderr, "\n");
    }
}

/* ── Intel HEX loader ────────────────────────────────────────────────────── */

static int load_hex(const char *filename)
{
    FILE *f = fopen(filename, "r");
    if (!f) { fprintf(stderr, "Cannot open '%s'\n", filename); return 0; }
    char line[256];
    int  loaded = 0, lo = 0x7FFF, hi = 0;
    while (fgets(line, sizeof(line), f)) {
        if (line[0] != ':') continue;
        int count   = 0, addr = 0, type = 0;
        sscanf(line+1, "%02x%04x%02x", &count, &addr, &type);
        if (type == 1) break;   /* EOF record */
        if (type != 0) continue;
        for (int i = 0; i < count; i++) {
            int byte = 0;
            sscanf(line + 9 + i*2, "%02x", &byte);
            int a = (addr + i) & AMSK;
            memory[a] = (UBYTE)byte;
            loaded++;
            if (a < lo) lo = a;
            if (a > hi) hi = a;
        }
    }
    fclose(f);
    if (loaded == 0) { fprintf(stderr, "No data records in '%s'\n", filename); return 0; }
    fprintf(stderr, "Loaded %d bytes from '%s' ($%04X-$%04X)\n",
            loaded, filename, lo, hi);
    return 1;
}

/* ── Main ────────────────────────────────────────────────────────────────── */

int main(int argc, char **argv)
{
    const char *hexfile = NULL;

    /* ── Parse args ── */
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-t")) {
            trace_mode = 1;
        } else if (!strcmp(argv[i], "-s")) {
            step_mode = 1;
        } else if (!strcmp(argv[i], "-b") && i+1 < argc) {
            bp_addr = (int)strtol(argv[++i], NULL, 16);
        } else if (!strcmp(argv[i], "-m") && i+2 < argc) {
            dump_addr = (int)strtol(argv[++i], NULL, 16);
            dump_len  = (int)strtol(argv[++i], NULL, 10);
        } else if (!strcmp(argv[i], "-n") && i+1 < argc) {
            inst_limit = atol(argv[++i]);
        } else if (!strcmp(argv[i], "--chin") && i+1 < argc) {
            chin_addr = (UWORD)(strtol(argv[++i], NULL, 16) & AMSK);
        } else if (!strcmp(argv[i], "--cout") && i+1 < argc) {
            cout_addr = (UWORD)(strtol(argv[++i], NULL, 16) & AMSK);
        } else if (!strcmp(argv[i], "--crlf") && i+1 < argc) {
            crlf_addr = (UWORD)(strtol(argv[++i], NULL, 16) & AMSK);
        } else if (!strcmp(argv[i], "--entry") && i+1 < argc) {
            entry_addr = (UWORD)(strtol(argv[++i], NULL, 16) & AMSK);
        } else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            print_usage(argv[0]);
            return 0;
        } else if (!strcmp(argv[i], "-v") || !strcmp(argv[i], "--version")) {
            print_version();
            return 0;
        } else if (argv[i][0] != '-') {
            hexfile = argv[i];
        } else {
            fprintf(stderr, "Unknown option '%s'\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }
    if (!hexfile) {
        print_usage(argv[0]);
        return 1;
    }

    /* ── Initialise CPU state ── */
    memset(memory,   0, sizeof(memory));
    memset(memflags, 0, sizeof(memflags));
    for (int i = 0; i < 32768; i++) {
        mirror_r[i] = (UWORD)i;   /* identity map — no mirroring */
        mirror_w[i] = (UWORD)i;
    }
    psu = PSU_SP & 0;   /* SP=0, no II */
    psl = 0;
    for (int i = 0; i < 7; i++) r[i] = 0;
    cycles_2650  = 0;
    interrupt_2650 = 0;
    ininterrupt  = 8;
    halted       = 0;

    /* ── Load program ── */
    if (!load_hex(hexfile)) return 1;

    /* ── Entry point (default $0440, override with --entry) ── */
    iar = entry_addr;
    print_version();
    fprintf(stderr,
        "PIPBUG 1 mode: COUT=$%04X  CHIN=$%04X  CRLF=$%04X  entry=$%04X\n",
        cout_addr, chin_addr, crlf_addr, entry_addr);

    /* ── Run loop ── */
    long count = 0;
    for (;;) {
        /* PIPBUG entry-point intercepts (before instruction fetch) */
        if (iar == cout_addr) { pb_cout(); continue; }
        if (iar == chin_addr) { pb_chin(); if (eof_hit) break; continue; }
        if (iar == crlf_addr) { pb_crlf(); continue; }

        /* Breakpoint check */
        if (bp_addr >= 0 && iar == (UWORD)bp_addr) {
            fprintf(stderr, "\n*** BREAKPOINT $%04X ***\n", bp_addr);
            print_halt_summary(count);
            break;
        }

        /* Instruction limit */
        if (inst_limit > 0 && count >= inst_limit) {
            fprintf(stderr, "\n*** Instruction limit (%ld) ***\n", inst_limit);
            print_halt_summary(count);
            break;
        }

        /* Trace */
        if (trace_mode) print_state();

        /* Step mode: pause, wait for Enter */
        if (step_mode) {
            print_state();
            fprintf(stderr, "[Enter to step, q+Enter to quit] ");
            char buf[8];
            if (fgets(buf, sizeof(buf), stdin) == NULL || buf[0]=='q') break;
        }

        /* Execute one instruction.
         * Under -DGAMER, one_instruction() only fetches opcode into `opcode`
         * and advances nothing. cpu_emu() does the actual decode and IAR advance.
         * We call both so any future non-GAMER sections still work.          */
        oldcycles = cycles_2650;
        opcode = memory[iar];   /* one_instruction() does this but we need it before cpu_emu() */
        one_instruction();      /* fetches opcode; no-ops all #ifndef GAMER blocks             */
        cpu_emu();              /* decodes and executes; advances IAR via ONE/TWO/THREE_BYTES  */
        count++;

        /* HALT check */
        if (halted) {
            fprintf(stderr, "\n*** HALT ($40) at $%04X ***\n", iar);
            print_halt_summary(count);
            break;
        }
    }

    if (eof_hit)
        fprintf(stderr, "\n*** EOF on stdin -- halted ***\n\n");

    /* Final state on clean exit */
    if (!trace_mode && !step_mode && bp_addr < 0 && !halted &&
        inst_limit > 0 && count < inst_limit && !eof_hit) {
        print_halt_summary(count);
    }

    return 0;
}
