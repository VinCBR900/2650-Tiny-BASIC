/* pipbug_wrap.c — PIPBUG 1 simulator using the WinArcadia 2650 CPU core
 * Version: 2.1
 * Date:    2026-06-09
 *
 * Purpose:
 *   Wraps 2650.c (the WinArcadia CPU core, compiled -DGAMER to strip all UI
 *   code) with just enough scaffold to run uBASIC2650 under PIPBUG 1 and
 *   produce identical results to WinArcadia.
 *
 * Usage:
 *   ./pipbug_wrap [options] program.hex
 *   ./pipbug_wrap [options] program.asm
 *
 * Options:
 *   -t              Trace every instruction to stderr
 *   -b 0xADDR       Breakpoint at address (hex); pauses and dumps state
 *   -m 0xADDR LEN   Dump LEN bytes from address at halt
 *   -s              Step mode: pause at every instruction, press Enter to continue
 *   -i              Interactive terminal mode; raw I/O, Ctrl-] exits
 *                   and defaults to unlimited instructions
 *   -n N            Instruction limit (default 5000000, 0=unlimited)
 *   --chin 0xADDR   CHIN intercept address (default 0x0286)
 *   --cout 0xADDR   COUT intercept address (default 0x02B4)
 *   --crlf 0xADDR   CRLF intercept address (default 0x008A)
 *   --entry 0xADDR  Program entry address (default 0x0440)
 *   -h, --help      Show usage help
 *   -v, --version   Show version number
 *
 * ASM input:
 *   Native builds accept .asm input by invoking asm2650 beside this wrapper,
 *   loading the generated .hex file, and reading the generated .LST file to
 *   auto-detect CHIN, COUT and program entry labels unless overridden.
 *
 * PIPBUG 1 intercepts (entry points match Oracle / uBASIC2650 EQUs):
 *   COUT  $02B4  putchar(R0)
 *   CHIN  $0286  R0 = host input byte
 *   CRLF  $008A  putchar('\r'); putchar('\n')
 *
 * Build:
 *   gcc -Wall -O2 -DGAMER -o pipbug_wrap pipbug_wrap.c
 *
 * Change history:
 *   v2.1  Added native .asm input support via asm2650 plus .LST auto-detection
 *         for CHIN, COUT and program entry addresses.
 *   v2.0  Added cross-platform interactive terminal mode (-i) with raw
 *         byte input/output and Ctrl-] graceful exit. Browser builds use
 *         non-blocking FIFO polling so WASM never blocks the event loop.
 *   v1.1  Added configurable program entry address switch (--entry),
 *         configurable CHIN/COUT/CRLF intercept addresses and
 *         explicit help/version command-line switches.
 *   v1.0  Initial release.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#if defined(__EMSCRIPTEN__)
#define PIPBUG_WRAP_EMSCRIPTEN 1
#else
#define PIPBUG_WRAP_EMSCRIPTEN 0
#endif

#if !PIPBUG_WRAP_EMSCRIPTEN && (defined(_WIN32) || defined(_WIN64) || defined(WIN32))
#define PIPBUG_WRAP_WIN32 1
#else
#define PIPBUG_WRAP_WIN32 0
#endif

#if PIPBUG_WRAP_EMSCRIPTEN
#include <emscripten.h>
#else
#define EMSCRIPTEN_KEEPALIVE
#endif

#if PIPBUG_WRAP_WIN32
#include <conio.h>
#include <fcntl.h>
#include <io.h>
#include <windows.h>
#ifndef ENABLE_EXTENDED_FLAGS
#define ENABLE_EXTENDED_FLAGS 0x0080
#endif
#elif !PIPBUG_WRAP_EMSCRIPTEN
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#endif

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

/* Forward declarations for functions in 2650.c that are called before their
   definition in GAMER mode (no #ifndef GAMER guard on the call sites). */
static void logindirectbios(void);
EXPORT void one_instruction(void);
EXPORT void do_tape(void);
EXPORT void pullras(void);
EXPORT void pushras(void);
EXPORT void checkinterrupt(void);

/* ── Include the WinArcadia CPU core ───────────────────────────────────────
 *  Keep WinArcadia's own WIN32 sections disabled in this standalone wrapper.
 *  They include UI/resource headers that are not needed for -DGAMER builds.
 */
#ifdef WIN32
#define PIPBUG_WRAP_RESTORE_WIN32 1
#undef WIN32
#endif
#include "2650.c"
#ifdef PIPBUG_WRAP_RESTORE_WIN32
#define WIN32 1
#undef PIPBUG_WRAP_RESTORE_WIN32
#endif

/* ── After including 2650.c the EXPORT variables are now in scope ────────── */
/* psu, psl, r[7], iar, ras[8], memory[32768], memflags[], cycles_2650 etc. */

/* ── Wrapper-specific globals ────────────────────────────────────────────── */

static int  bp_addr       = -1;     /* -b breakpoint address (-1 = none)   */
static int  dump_addr     = -1;     /* -m dump base address                 */
static int  dump_len      = 0;
static long inst_limit    = 5000000;
static int  inst_limit_set = 0;      /* explicit -n was supplied             */
static int  step_mode     = 0;      /* -s step mode                        */
static int  trace_mode    = 0;      /* -t trace mode                       */
static int  interactive_mode = 0;   /* -i raw terminal mode                */
static int  eof_hit       = 0;
static int  interactive_exit = 0;
static int  run_finished = 0;       /* emulator has reached a terminal state */
static int  final_report_printed = 0;
static long instruction_count = 0;
static UWORD chin_addr    = 0x0286; /* --chin                              */
static UWORD cout_addr    = 0x02B4; /* --cout                              */
static UWORD crlf_addr    = 0x008A; /* --crlf                              */
static UWORD entry_addr   = 0x0440; /* --entry                             */
static int  chin_addr_set = 0;      /* explicit --chin was supplied        */
static int  cout_addr_set = 0;      /* explicit --cout was supplied        */
static int  entry_addr_set = 0;     /* explicit --entry was supplied       */

#if PIPBUG_WRAP_EMSCRIPTEN
static int terminal_restore_needed = 0;
#elif PIPBUG_WRAP_WIN32
static DWORD saved_input_mode = 0;
static int saved_stdin_mode = -1;
static int saved_stdout_mode = -1;
static int terminal_restore_needed = 0;
static HANDLE console_input = INVALID_HANDLE_VALUE;
#else
static struct termios saved_termios;
static int terminal_restore_needed = 0;
#endif

#if PIPBUG_WRAP_EMSCRIPTEN
EM_JS(int, pipbug_browser_char_available, (void), {
    if (typeof Module !== 'undefined' && Module.pipbugCharAvailable)
        return Module.pipbugCharAvailable() ? 1 : 0;
    return 0;
});

EM_JS(int, pipbug_browser_getchar_nonblock, (void), {
    if (typeof Module !== 'undefined' && Module.pipbugGetcharNonblock)
        return Module.pipbugGetcharNonblock();
    return -1;
});

EM_JS(int, pipbug_browser_putchar, (int ch), {
    if (typeof Module !== 'undefined' && Module.pipbugPutchar) {
        Module.pipbugPutchar(ch & 0xff);
        return 1;
    }
    return 0;
});
#endif

/*
 * Purpose:
 *   Restore the host terminal/console state after interactive mode.
 * Inputs:
 *   None.
 * Outputs:
 *   None.
 */
static void terminal_restore(void)
{
    if (!terminal_restore_needed) return;

#if PIPBUG_WRAP_EMSCRIPTEN
    /* Browser builds do not modify a host terminal. */
#elif PIPBUG_WRAP_WIN32
    if (console_input != INVALID_HANDLE_VALUE)
        SetConsoleMode(console_input, saved_input_mode);
    if (saved_stdin_mode >= 0)
        _setmode(_fileno(stdin), saved_stdin_mode);
    if (saved_stdout_mode >= 0)
        _setmode(_fileno(stdout), saved_stdout_mode);
#else
    tcsetattr(STDIN_FILENO, TCSANOW, &saved_termios);
#endif

    terminal_restore_needed = 0;
}

/*
 * Purpose:
 *   Enable raw, byte-oriented host terminal input for interactive mode.
 * Inputs:
 *   None.
 * Outputs:
 *   Returns 1 on success, or 0 when interactive mode cannot be enabled.
 */
static int terminal_enable_interactive(void)
{
#if PIPBUG_WRAP_EMSCRIPTEN
    return 1;
#elif PIPBUG_WRAP_WIN32
    DWORD mode;

    if (!_isatty(_fileno(stdin))) {
        fprintf(stderr, "Interactive mode requires a console stdin.\n");
        return 0;
    }

    console_input = GetStdHandle(STD_INPUT_HANDLE);
    if (console_input == INVALID_HANDLE_VALUE ||
        !GetConsoleMode(console_input, &saved_input_mode)) {
        fprintf(stderr, "Interactive mode could not read console mode.\n");
        return 0;
    }

    saved_stdin_mode = _setmode(_fileno(stdin), _O_BINARY);
    saved_stdout_mode = _setmode(_fileno(stdout), _O_BINARY);
    terminal_restore_needed = 1;

    mode = saved_input_mode;
    mode &= ~(ENABLE_LINE_INPUT | ENABLE_ECHO_INPUT | ENABLE_PROCESSED_INPUT);
#ifdef ENABLE_QUICK_EDIT_MODE
    mode &= ~ENABLE_QUICK_EDIT_MODE;
#endif
    mode |= ENABLE_EXTENDED_FLAGS;

    if (!SetConsoleMode(console_input, mode)) {
        fprintf(stderr, "Interactive mode could not set console mode.\n");
        terminal_restore();
        return 0;
    }

    atexit(terminal_restore);
    return 1;
#else
    struct termios raw;

    if (!isatty(STDIN_FILENO)) {
        fprintf(stderr, "Interactive mode requires a terminal stdin.\n");
        return 0;
    }

    if (tcgetattr(STDIN_FILENO, &saved_termios) != 0) {
        perror("tcgetattr");
        return 0;
    }

    raw = saved_termios;
    raw.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
#ifdef IXOFF
    raw.c_iflag &= ~IXOFF;
#endif
#ifdef INLCR
    raw.c_iflag &= ~INLCR;
#endif
#ifdef IGNCR
    raw.c_iflag &= ~IGNCR;
#endif
    raw.c_oflag &= ~OPOST;
    raw.c_cflag |= CS8;
    raw.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
        perror("tcsetattr");
        return 0;
    }

    terminal_restore_needed = 1;
    atexit(terminal_restore);
    return 1;
#endif
}

/*
 * Purpose:
 *   Report whether a browser/WASM input byte is already queued. Native builds
 *   return true because their legacy batch and terminal input paths may block.
 * Inputs:
 *   None.
 * Outputs:
 *   Returns non-zero when an input byte can be read without waiting.
 */
int platform_char_available(void)
{
#if PIPBUG_WRAP_EMSCRIPTEN
    return pipbug_browser_char_available();
#else
    return 1;
#endif
}

/*
 * Purpose:
 *   Read one input byte through the platform input abstraction. Browser/WASM
 *   builds poll a JavaScript FIFO and never block; native builds preserve the
 *   existing blocking stdin/terminal behavior for batch and interactive use.
 * Inputs:
 *   None.
 * Outputs:
 *   Returns the byte value 0-255, or -1 when no byte/EOF is available.
 */
int platform_getchar_nonblock(void)
{
#if PIPBUG_WRAP_EMSCRIPTEN
    if (!platform_char_available()) return -1;
    return pipbug_browser_getchar_nonblock();
#elif PIPBUG_WRAP_WIN32
    if (interactive_mode) {
        int c = _getch();
        return c == EOF ? -1 : (c & 0xff);
    }
    {
        int c = getchar();
        return c == EOF ? -1 : (c & 0xff);
    }
#else
    if (interactive_mode) {
        unsigned char c;
        for (;;) {
            ssize_t got = read(STDIN_FILENO, &c, 1);
            if (got == 1) return c;
            if (got == 0) return -1;
            if (errno != EINTR) return -1;
        }
    }
    {
        int c = getchar();
        return c == EOF ? -1 : (c & 0xff);
    }
#endif
}

/*
 * Purpose:
 *   Write one output byte through the platform output abstraction. Emscripten
 *   uses an optional JS hook to avoid stdout line buffering and display
 *   terminal output character-by-character.
 * Inputs:
 *   ch - byte to write.
 * Outputs:
 *   Writes the byte to stdout or the browser terminal callback.
 */
void platform_putchar(char ch)
{
#if PIPBUG_WRAP_EMSCRIPTEN
    if (pipbug_browser_putchar((unsigned char)ch)) return;
#endif
    putchar((unsigned char)ch);
}

/*
 * Purpose:
 *   Print command-line usage information.
 * Inputs:
 *   prog - program name to display in the usage banner.
 * Outputs:
 *   None.
 */
static void print_usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [options] program.hex|program.asm\n"
        "Options:\n"
        "  -t              Trace every instruction to stderr\n"
        "  -s              Step mode: pause each instruction\n"
        "  -i              Interactive terminal mode; raw I/O, Ctrl-] exits\n"
        "                  and defaults to unlimited instructions\n"
        "  -b 0xADDR       Breakpoint at address (hex)\n"
        "  -m 0xADDR LEN   Dump LEN bytes from address at halt\n"
        "  -n LIMIT        Instruction limit (default 5000000, 0=unlimited)\n"
        "  --chin 0xADDR   CHIN intercept address (default 0x0286)\n"
        "  --cout 0xADDR   COUT intercept address (default 0x02B4)\n"
        "  --crlf 0xADDR   CRLF intercept address (default 0x008A)\n"
        "  --entry 0xADDR  Program entry address (default 0x0440)\n"
#if PIPBUG_WRAP_EMSCRIPTEN
        "  ASM input is not available in Emscripten/browser builds.\n"
#else
        "  ASM input invokes asm2650 and reads the generated .LST to auto-detect\n"
        "                  CHIN, COUT and entry labels unless explicitly overridden.\n"
#endif
        "  -h, --help      Show this help message\n"
        "  -v, --version   Show version\n",
        prog);
}

/*
 * Purpose:
 *   Print the wrapper version.
 * Inputs:
 *   None.
 * Outputs:
 *   None.
 */
static void print_version(void)
{
    fprintf(stderr, "pipbug_wrap v2.1\n");
}

/* ── Minimal PIPBUG stubs ─────────────────────────────────────────────────
 *  We do NOT simulate PIPBUG ROM.  Instead we intercept the three entry
 *  points before each instruction fetch and handle them ourselves.
 *  After handling, we pop the RAS and continue (mimicking RETC,UN).
 */
/*
 * Purpose:
 *   Return from an intercepted PIPBUG entry point by popping the RAS.
 * Inputs:
 *   None.
 * Outputs:
 *   Updates IAR through pullras().
 */
static void pb_ret(void)
{
    pullras();  /* pops iar from RAS, matching BSTA that called us */
}

/*
 * Purpose:
 *   Handle the PIPBUG COUT intercept by writing R0 to host stdout.
 * Inputs:
 *   R0 contains the byte to output.
 * Outputs:
 *   Writes one byte to stdout and returns from the intercepted call.
 */
static void pb_cout(void)
{
    platform_putchar((char)r[0]);
    fflush(stdout);
    pb_ret();
}

/*
 * Purpose:
 *   Handle the PIPBUG CHIN intercept by reading one host input byte.
 * Inputs:
 *   None.
 * Outputs:
 *   Stores the input byte in R0, requests wrapper exit on EOF/Ctrl-], or
 *   returns 0 in browser builds when no FIFO input byte is available yet.
 */
static int pb_chin(void)
{
    int c;

#if PIPBUG_WRAP_EMSCRIPTEN
    /* Browser/WASM cannot safely block in getchar(): blocking would suspend or
     * monopolize the single browser thread. Instead JavaScript keyboard events
     * enqueue bytes and CHIN polls that FIFO. If it is empty, leave IAR at the
     * CHIN entry point so the browser scheduler can retry on a later frame.
     */
    c = platform_getchar_nonblock();
    if (c < 0) return 0;
#else
    c = platform_getchar_nonblock();
    if (c < 0) {
        eof_hit = 1;
        r[0] = 0;
        pb_ret();
        return 1;
    }
#endif

    /* PIPBUG and Tiny BASIC expect CR (0x0D) to terminate input lines.
     * Mapping LF to CR keeps standard text-file batch input and browser Enter
     * keys compatible without changing existing CR-terminated input.
     */
    if (c == '\n') c = '\r';

    if (interactive_mode && c == 0x1D) {
        interactive_exit = 1;
        r[0] = 0;
    } else {
        r[0] = (UBYTE)c;
    }
    pb_ret();
    return 1;
}

/*
 * Purpose:
 *   Handle the PIPBUG CRLF intercept by writing CR and LF to host stdout.
 * Inputs:
 *   None.
 * Outputs:
 *   Writes CR/LF to stdout and returns from the intercepted call.
 */
static void pb_crlf(void)
{
    platform_putchar('\r');
    platform_putchar('\n');
    fflush(stdout);
    pb_ret();
}

/* ── Trace / debug helpers ───────────────────────────────────────────────── */

/*
 * Purpose:
 *   Convert the current 2650 condition-code bits into a printable name.
 * Inputs:
 *   None.
 * Outputs:
 *   Returns a static string for the current condition code.
 */
static const char *cc_name(void)
{
    switch (psl & PSL_CC) {
        case 0x00: return "EQ";
        case 0x40: return "GT";
        case 0x80: return "LT";
        default:   return "UN";
    }
}

/*
 * Purpose:
 *   Print the current CPU state for tracing or step mode.
 * Inputs:
 *   None.
 * Outputs:
 *   Writes the formatted CPU state to stderr.
 */
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

/*
 * Purpose:
 *   Print a halt/break summary and optional memory dump.
 * Inputs:
 *   count - number of emulated instructions executed.
 * Outputs:
 *   Writes the summary to stderr.
 */
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


/* ── ASM input support ───────────────────────────────────────────────────── */

/*
 * Description:
 *   Compare two NUL-terminated strings without regard to ASCII case.
 * Inputs:
 *   a - first string to compare.
 *   b - second string to compare.
 * Outputs:
 *   Returns non-zero when both strings are equal ignoring case, otherwise 0.
 */
static int str_case_equal(const char *a, const char *b)
{
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return 0;
        a++;
        b++;
    }
    return *a == '\0' && *b == '\0';
}

/*
 * Description:
 *   Test whether a path's final filename component ends with a given extension.
 * Inputs:
 *   path - filesystem path to inspect.
 *   ext  - extension to match, including the leading dot.
 * Outputs:
 *   Returns non-zero when the filename extension matches case-insensitively,
 *   otherwise 0.
 */
static int has_extension(const char *path, const char *ext)
{
    const char *slash1 = strrchr(path, '/');
    const char *slash2 = strrchr(path, '\\');
    const char *slash = slash1;
    const char *dot;
    if (!slash || (slash2 && slash2 > slash)) slash = slash2;
    dot = strrchr(slash ? slash + 1 : path, '.');
    return dot && str_case_equal(dot, ext);
}

/*
 * Description:
 *   Build a new path by replacing the extension of the final filename component.
 * Inputs:
 *   path - source filesystem path.
 *   ext  - replacement extension, including the leading dot.
 * Outputs:
 *   Returns a heap-allocated path string that the caller must free, or NULL on
 *   allocation failure.
 */
static char *replace_extension(const char *path, const char *ext)
{
    const char *slash1 = strrchr(path, '/');
    const char *slash2 = strrchr(path, '\\');
    const char *slash = slash1;
    const char *base;
    const char *dot;
    size_t stem_len;
    size_t ext_len = strlen(ext);
    char *out;

    if (!slash || (slash2 && slash2 > slash)) slash = slash2;
    base = slash ? slash + 1 : path;
    dot = strrchr(base, '.');
    stem_len = dot ? (size_t)(dot - path) : strlen(path);

    out = (char *)malloc(stem_len + ext_len + 1);
    if (!out) return NULL;
    memcpy(out, path, stem_len);
    memcpy(out + stem_len, ext, ext_len + 1);
    return out;
}

/*
 * Description:
 *   Derive the asm2650 executable path that should live beside pipbug_wrap.
 * Inputs:
 *   prog - argv[0] path used to launch the wrapper.
 * Outputs:
 *   Returns a heap-allocated assembler executable path that the caller must
 *   free, or NULL on allocation failure.
 */
static char *assembler_path_for_wrapper(const char *prog)
{
    const char *slash1 = strrchr(prog, '/');
    const char *slash2 = strrchr(prog, '\\');
    const char *slash = slash1;
#if PIPBUG_WRAP_WIN32
    const char *name = "asm2650.exe";
#else
    const char *name = "asm2650";
#endif
    size_t dir_len = 0;
    char *out;

    if (!slash || (slash2 && slash2 > slash)) slash = slash2;
    if (slash) dir_len = (size_t)(slash - prog + 1);

    if (!dir_len) {
#if PIPBUG_WRAP_WIN32
        const char *curdir = ".\\";
#else
        const char *curdir = "./";
#endif
        dir_len = strlen(curdir);
        out = (char *)malloc(dir_len + strlen(name) + 1);
        if (!out) return NULL;
        memcpy(out, curdir, dir_len);
    } else {
        out = (char *)malloc(dir_len + strlen(name) + 1);
        if (!out) return NULL;
        memcpy(out, prog, dir_len);
    }
    memcpy(out + dir_len, name, strlen(name) + 1);
    return out;
}

/*
 * Description:
 *   Append literal text to a growable command-line buffer.
 * Inputs:
 *   cmd  - address of the heap buffer pointer to grow and append to.
 *   len  - address of the current command string length.
 *   cap  - address of the allocated buffer capacity.
 *   text - NUL-terminated text to append.
 * Outputs:
 *   Updates cmd, len and cap as needed. Returns 1 on success, or 0 on
 *   allocation failure.
 */
static int append_to_command(char **cmd, size_t *len, size_t *cap, const char *text)
{
    size_t need = strlen(text);
    if (*len + need + 1 > *cap) {
        size_t new_cap = *cap ? *cap : 128;
        char *new_cmd;
        while (*len + need + 1 > new_cap) new_cap *= 2;
        new_cmd = (char *)realloc(*cmd, new_cap);
        if (!new_cmd) return 0;
        *cmd = new_cmd;
        *cap = new_cap;
    }
    memcpy(*cmd + *len, text, need + 1);
    *len += need;
    return 1;
}

/*
 * Description:
 *   Append one shell-quoted command argument using native Win32 or POSIX rules.
 * Inputs:
 *   cmd - address of the heap command buffer pointer to grow and append to.
 *   len - address of the current command string length.
 *   cap - address of the allocated buffer capacity.
 *   arg - unquoted argument text to append.
 * Outputs:
 *   Updates cmd, len and cap as needed. Returns 1 on success, or 0 on
 *   allocation failure.
 */
static int append_quoted_arg(char **cmd, size_t *len, size_t *cap, const char *arg)
{
#if PIPBUG_WRAP_WIN32
    if (!append_to_command(cmd, len, cap, "\"")) return 0;
    for (const char *p = arg; *p; p++) {
        if (*p == '"') {
            if (!append_to_command(cmd, len, cap, "\\\"")) return 0;
        } else {
            char tmp[2] = {*p, '\0'};
            if (!append_to_command(cmd, len, cap, tmp)) return 0;
        }
    }
    return append_to_command(cmd, len, cap, "\"");
#else
    if (!append_to_command(cmd, len, cap, "'")) return 0;
    for (const char *p = arg; *p; p++) {
        if (*p == '\'') {
            if (!append_to_command(cmd, len, cap, "'\\''")) return 0;
        } else {
            char tmp[2] = {*p, '\0'};
            if (!append_to_command(cmd, len, cap, tmp)) return 0;
        }
    }
    return append_to_command(cmd, len, cap, "'");
#endif
}

/*
 * Description:
 *   Parse one asm2650 listing symbol-table line into a label name and value.
 * Inputs:
 *   line      - listing line to parse.
 *   name      - destination buffer for the parsed symbol name.
 *   name_size - size of the destination name buffer in bytes.
 *   value     - destination for the parsed symbol address/value.
 * Outputs:
 *   Writes name and value when parsing succeeds. Returns 1 for a symbol line,
 *   otherwise 0.
 */
static int parse_symbol_line(const char *line, char *name, size_t name_size, unsigned *value)
{
    char sym[128];
    unsigned val;
    int consumed = 0;

    if (sscanf(line, " %127s $%x %n", sym, &val, &consumed) < 2) return 0;
    if (consumed <= 0) return 0;
    if (name_size > 0) {
        snprintf(name, name_size, "%s", sym);
    }
    *value = val & AMSK;
    return 1;
}

/*
 * Description:
 *   Parse a listing source line and identify the first emitted code address.
 * Inputs:
 *   line  - listing line to inspect.
 *   value - destination for the parsed code address.
 * Outputs:
 *   Writes value when the line contains an address and opcode byte. Returns 1
 *   on success, otherwise 0.
 */
static int parse_first_listing_code_addr(const char *line, unsigned *value)
{
    int lineno;
    unsigned addr;
    unsigned byte;
    if (sscanf(line, " %d $%x %x", &lineno, &addr, &byte) == 3) {
        if (byte <= 0xff) {
            *value = addr & AMSK;
            return 1;
        }
    }
    return 0;
}

/*
 * Description:
 *   Check whether a label name is one of the recognized program entry labels.
 * Inputs:
 *   name - symbol name from the assembler listing.
 * Outputs:
 *   Returns non-zero when name matches a recognized entry label, otherwise 0.
 */
static int is_entry_symbol(const char *name)
{
    static const char *const names[] = {
        "ENTRY", "START", "RESET", "MAIN", "INIT", "BEGIN", "BOOT", NULL
    };
    for (int i = 0; names[i]; i++) {
        if (str_case_equal(name, names[i])) return 1;
    }
    return 0;
}

/*
 * Description:
 *   Read an asm2650 .LST file and apply auto-detected simulator addresses.
 * Inputs:
 *   lst_file - path to the assembler listing sidecar.
 * Outputs:
 *   Updates CHIN, COUT and entry globals when matching symbols are found and
 *   the user did not override them. Returns non-zero when useful listing data
 *   was found, otherwise 0.
 */
static int apply_listing_config(const char *lst_file)
{
    FILE *f = fopen(lst_file, "r");
    char line[512];
    int in_labels = 0;
    int found_any = 0;
    int found_chin = 0, found_cout = 0, found_entry = 0;
    unsigned first_code = 0;
    int have_first_code = 0;

    if (!f) {
        fprintf(stderr, "Cannot open assembler listing '%s' for auto-detection\n", lst_file);
        return 0;
    }

    while (fgets(line, sizeof(line), f)) {
        if (!have_first_code && parse_first_listing_code_addr(line, &first_code))
            have_first_code = 1;

        if (!in_labels) {
            if (strncmp(line, "Labels:", 7) == 0) in_labels = 1;
            continue;
        }

        {
            char name[128];
            unsigned value;
            if (!parse_symbol_line(line, name, sizeof(name), &value)) continue;

            if (!chin_addr_set && str_case_equal(name, "CHIN")) {
                chin_addr = (UWORD)value;
                found_chin = 1;
                found_any = 1;
            } else if (!cout_addr_set && str_case_equal(name, "COUT")) {
                cout_addr = (UWORD)value;
                found_cout = 1;
                found_any = 1;
            } else if (!entry_addr_set && !found_entry && is_entry_symbol(name)) {
                entry_addr = (UWORD)value;
                found_entry = 1;
                found_any = 1;
            }
        }
    }
    fclose(f);

    if (!entry_addr_set && !found_entry && have_first_code) {
        entry_addr = (UWORD)first_code;
        found_any = 1;
        fprintf(stderr, "ASM auto-detect: entry=$%04X (first code address)\n", entry_addr);
    }
    if (found_chin || found_cout || found_entry) {
        fprintf(stderr, "ASM auto-detect:%s%s%s\n",
                found_cout ? " COUT" : "",
                found_chin ? " CHIN" : "",
                found_entry ? " entry" : "");
    }
    return found_any || have_first_code;
}

/*
 * Description:
 *   Assemble an ASM input file with asm2650 and prepare the generated HEX file.
 * Inputs:
 *   prog     - argv[0] path used to locate the sibling asm2650 executable.
 *   asm_file - source ASM file to assemble.
 *   hex_out  - destination for the generated heap-allocated HEX path.
 * Outputs:
 *   Invokes asm2650 on native builds and applies .LST auto-detection. Writes
 *   *hex_out on success. Returns 1 on success, or 0 on failure.
 */
static int assemble_source(const char *prog, const char *asm_file, char **hex_out)
{
#if PIPBUG_WRAP_EMSCRIPTEN
    (void)prog;
    (void)asm_file;
    (void)hex_out;
    fprintf(stderr, "ASM input is not supported in Emscripten/browser builds. Assemble to HEX first.\n");
    return 0;
#else
    char *asm_exe = assembler_path_for_wrapper(prog);
    char *hex_file = replace_extension(asm_file, ".hex");
    char *lst_file = replace_extension(asm_file, ".LST");
    char *cmd = NULL;
    size_t len = 0, cap = 0;
    int rc;

    if (!asm_exe || !hex_file || !lst_file) {
        fprintf(stderr, "Out of memory preparing ASM input.\n");
        free(asm_exe);
        free(hex_file);
        free(lst_file);
        return 0;
    }

    if (!append_quoted_arg(&cmd, &len, &cap, asm_exe) ||
        !append_to_command(&cmd, &len, &cap, " ") ||
        !append_quoted_arg(&cmd, &len, &cap, asm_file) ||
        !append_to_command(&cmd, &len, &cap, " ") ||
        !append_quoted_arg(&cmd, &len, &cap, hex_file)) {
        fprintf(stderr, "Out of memory building assembler command.\n");
        free(asm_exe);
        free(hex_file);
        free(lst_file);
        free(cmd);
        return 0;
    }

    fprintf(stderr, "Assembling '%s' -> '%s'\n", asm_file, hex_file);
    rc = system(cmd);
    free(cmd);
    free(asm_exe);
    if (rc != 0) {
        fprintf(stderr, "asm2650 failed for '%s' (status %d)\n", asm_file, rc);
        free(hex_file);
        free(lst_file);
        return 0;
    }

    (void)apply_listing_config(lst_file);
    free(lst_file);
    *hex_out = hex_file;
    return 1;
#endif
}

/* ── Intel HEX loader ────────────────────────────────────────────────────── */

/*
 * Purpose:
 *   Load an Intel HEX file into emulated 2650 memory.
 * Inputs:
 *   filename - path to the Intel HEX file to load.
 * Outputs:
 *   Returns 1 when data records were loaded, otherwise 0.
 */
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


#define PIPBUG_RUN_RUNNING 0
#define PIPBUG_RUN_WAITING_INPUT 1
#define PIPBUG_RUN_DONE 2

/*
 * Purpose:
 *   Print final run messages exactly once when the emulator reaches a terminal
 *   state.
 * Inputs:
 *   None.
 * Outputs:
 *   Writes any final status/summary to stderr.
 */
static void print_final_report(void)
{
    if (final_report_printed) return;
    final_report_printed = 1;

    if (interactive_exit)
        fprintf(stderr, "\n*** Ctrl-] -- interactive exit ***\n\n");
    else if (eof_hit)
        fprintf(stderr, "\n*** EOF on stdin -- halted ***\n\n");

    if (!trace_mode && !step_mode && bp_addr < 0 && !halted &&
        inst_limit > 0 && instruction_count < inst_limit &&
        !eof_hit && !interactive_exit) {
        print_halt_summary(instruction_count);
    }
}

/*
 * Purpose:
 *   Mark the emulator as finished and emit final status if needed.
 * Inputs:
 *   None.
 * Outputs:
 *   Sets run_finished and prints the final report.
 */
static void finish_run(void)
{
    run_finished = 1;
    print_final_report();
}

/*
 * Purpose:
 *   Execute a bounded chunk of the wrapper run loop. Browser builds call this
 *   from requestAnimationFrame() so the WASM module never owns the browser
 *   thread indefinitely; native builds pass 0 to run until a terminal state.
 * Inputs:
 *   instruction_budget - maximum CPU instructions to execute, or 0 for no
 *                        chunk limit.
 * Outputs:
 *   Returns PIPBUG_RUN_RUNNING, PIPBUG_RUN_WAITING_INPUT, or PIPBUG_RUN_DONE.
 */
EMSCRIPTEN_KEEPALIVE int pipbug_run_chunk(int instruction_budget)
{
    int executed = 0;

    if (run_finished) return PIPBUG_RUN_DONE;

    for (;;) {
        if (iar == cout_addr) { pb_cout(); continue; }
        if (iar == chin_addr) {
            if (!pb_chin()) return PIPBUG_RUN_WAITING_INPUT;
            if (eof_hit || interactive_exit) {
                finish_run();
                return PIPBUG_RUN_DONE;
            }
            continue;
        }
        if (iar == crlf_addr) { pb_crlf(); continue; }

        if (bp_addr >= 0 && iar == (UWORD)bp_addr) {
            fprintf(stderr, "\n*** BREAKPOINT $%04X ***\n", bp_addr);
            print_halt_summary(instruction_count);
            finish_run();
            return PIPBUG_RUN_DONE;
        }

        if (inst_limit > 0 && instruction_count >= inst_limit) {
            fprintf(stderr, "\n*** Instruction limit (%ld) ***\n", inst_limit);
            print_halt_summary(instruction_count);
            finish_run();
            return PIPBUG_RUN_DONE;
        }

        if (trace_mode) print_state();

        if (step_mode) {
            print_state();
            fprintf(stderr, "[Enter to step, q+Enter to quit] ");
            char buf[8];
            if (fgets(buf, sizeof(buf), stdin) == NULL || buf[0]=='q') {
                finish_run();
                return PIPBUG_RUN_DONE;
            }
        }

        oldcycles = cycles_2650;
        opcode = memory[iar];
        one_instruction();
        cpu_emu();
        instruction_count++;
        executed++;

        if (halted) {
            fprintf(stderr, "\n*** HALT ($40) at $%04X ***\n", iar);
            print_halt_summary(instruction_count);
            finish_run();
            return PIPBUG_RUN_DONE;
        }

        if (instruction_budget > 0 && executed >= instruction_budget)
            return PIPBUG_RUN_RUNNING;
    }
}

/* ── Main ────────────────────────────────────────────────────────────────── */

/*
 * Purpose:
 *   Parse options, load the target HEX file, and run the PIPBUG wrapper loop.
 * Inputs:
 *   argc - command-line argument count.
 *   argv - command-line argument vector.
 * Outputs:
 *   Returns process exit status 0 on success, non-zero on setup failure.
 */
int main(int argc, char **argv)
{
    const char *input_file = NULL;
    const char *hexfile = NULL;
    char *generated_hexfile = NULL;

    /* ── Parse args ── */
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-t")) {
            trace_mode = 1;
        } else if (!strcmp(argv[i], "-s")) {
            step_mode = 1;
        } else if (!strcmp(argv[i], "-i")) {
            interactive_mode = 1;
        } else if (!strcmp(argv[i], "-b") && i+1 < argc) {
            bp_addr = (int)strtol(argv[++i], NULL, 16);
        } else if (!strcmp(argv[i], "-m") && i+2 < argc) {
            dump_addr = (int)strtol(argv[++i], NULL, 16);
            dump_len  = (int)strtol(argv[++i], NULL, 10);
        } else if (!strcmp(argv[i], "-n") && i+1 < argc) {
            inst_limit = atol(argv[++i]);
            inst_limit_set = 1;
        } else if (!strcmp(argv[i], "--chin") && i+1 < argc) {
            chin_addr = (UWORD)(strtol(argv[++i], NULL, 16) & AMSK);
            chin_addr_set = 1;
        } else if (!strcmp(argv[i], "--cout") && i+1 < argc) {
            cout_addr = (UWORD)(strtol(argv[++i], NULL, 16) & AMSK);
            cout_addr_set = 1;
        } else if (!strcmp(argv[i], "--crlf") && i+1 < argc) {
            crlf_addr = (UWORD)(strtol(argv[++i], NULL, 16) & AMSK);
        } else if (!strcmp(argv[i], "--entry") && i+1 < argc) {
            entry_addr = (UWORD)(strtol(argv[++i], NULL, 16) & AMSK);
            entry_addr_set = 1;
        } else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            print_usage(argv[0]);
            return 0;
        } else if (!strcmp(argv[i], "-v") || !strcmp(argv[i], "--version")) {
            print_version();
            return 0;
        } else if (argv[i][0] != '-') {
            input_file = argv[i];
        } else {
            fprintf(stderr, "Unknown option '%s'\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }
    if (!input_file) {
        print_usage(argv[0]);
        return 1;
    }

    if (interactive_mode && step_mode) {
        fprintf(stderr, "Options -i and -s cannot be used together.\n");
        return 1;
    }
    if (interactive_mode && !inst_limit_set)
        inst_limit = 0;
    if (interactive_mode && !terminal_enable_interactive())
        return 1;

    if (has_extension(input_file, ".asm")) {
        if (!assemble_source(argv[0], input_file, &generated_hexfile)) return 1;
        hexfile = generated_hexfile;
    } else {
        hexfile = input_file;
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
    if (!load_hex(hexfile)) {
        free(generated_hexfile);
        return 1;
    }

    /* ── Entry point (default $0440, override with --entry) ── */
    iar = entry_addr;
    print_version();
    fprintf(stderr,
        "Config mode: COUT=$%04X  CHIN=$%04X  CRLF=$%04X  entry=$%04X%s  limit=%s\n",
        cout_addr, chin_addr, crlf_addr, entry_addr,
        interactive_mode ? "  interactive=on" : "",
        inst_limit > 0 ? "set" : "unlimited");

    if(interactive_mode) fprintf(stderr,"\n*** Press Ctrl-] to exit Interactive Mode ***\n");
   
    /* ── Run loop ── */
#if PIPBUG_WRAP_EMSCRIPTEN
    /* In browser builds, main() only initializes state. JavaScript drives
     * execution with pipbug_run_chunk() from requestAnimationFrame(), allowing
     * keyboard events to fill the FIFO between chunks.
     */
    free(generated_hexfile);
    return 0;
#else
    while (!run_finished)
        (void)pipbug_run_chunk(0);
    free(generated_hexfile);
    return 0;
#endif
}
