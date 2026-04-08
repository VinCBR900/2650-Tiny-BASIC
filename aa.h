/* aa.h stub — all definitions provided by pipbug_wrap.c before #include "2650.c" */

/* Boolean */
#define TRUE    1
#define FALSE   0

/* Signed byte */
typedef signed char SBYTE;

/* Colour constants (opcode colour table — values unused in GAMER build) */
#define RED     0
#define GREEN   1
#define BLUE    2
#define YELLOW  3
#define PURPLE  4
#define CYAN    5
#define WHITE   6
#define GREY1   7

/* Token constants */
#define FIRSTTOKEN  0
#define TOKEN_R0    0
#define LASTTOKEN   19
#define STYLES      5

/* Display dimensions */
#define BOXWIDTH    1
#define BOXHEIGHT   1

/* OPERAND: the current instruction's first operand byte */
#define OPERAND     (memory[WRAPMEM(1)])

/* GET_RR: decode register field from opcode, honouring PSL RS bank-switch */
#define GET_RR      rr = (opcode & 3); if (rr && (psl & PSL_RS)) rr += 3;

/* BRANCHCODE: condition field is simply opcode bits 1:0, compared against
   the 2-bit CC field returned by CCFIELD ((psl & PSL_CC) >> 6)           */
#define BRANCHCODE  (opcode & 3)

/* Machine / BIOS version constants (values not used for PIPBUG path) */
#define BINBUG_61               0
#define BINBUG_BAUDRATE_2400    0
#define PHUNSY_PHUNSY           0
#define INTDIR_DIRECT           0
#define INTDIR_INDIRECT         1

/* Tape / audio constants */
#define KIND_8SVX               0
#define KIND_AIFF               1
#define TAPEMODE_PLAY           0
#define TAPEMODE_STOP           1
#define RECMODE_PLAY            0
#define RECMODE_RECORD          1
#define NET_CLIENT              0
#define NET_SERVER              1
#define PARALLEL_MEMMAPPED      0
#define BANKED                  0

/* Memory map constants — all given unique values away from machine IDs */
#define MEMMAP_ASTROWARS        100
#define MEMMAP_GALAXIA          101
#define MEMMAP_LASERBATTLE      102
#define MEMMAP_LAZARIAN         103
#define MEMMAP_MALZAK1          105
#define MEMMAP_MALZAK2          106
#define MEMMAP_N                107
#define MEMMAP_O                108

/* I/O port constants */
#define PORTC                   0
#define PORTD                   1
#define PRT_INT                 0x01
#define DISK_INT                0x02
#define CRT_TTY                 0x04
#define IE_NOISE                0x08
#define E_CASIN                 0
#define E_CASOUT                1

/* Game-specific position constants (whichgame values, never match -1) */
#define PIPBUG_MORSEPOS         -2
#define PIANOPOS                -3
#define MUSIC1POS               -4
#define MUSIC2POS               -5
#define MIKITP_MUSICPOS         -6
#define MIKITMUSICPOS           -6
#define RYTMONPOS               -7
#define BELMACHPOS              -8
#define BELMACH0POS             -9
#define SI50_THEMEPOS           -10
#define PHUNSY_THEMEPOS         -11
#define ACOS                    -12

/* ASIC register write flags (only size matters) */
#define ASIC_UVI_SPRITE0Y       0x0001
#define ASIC_UVI_SPRITE0X       0x0002
#define ASIC_UVI_SPRITE1Y       0x0004
#define ASIC_UVI_SPRITE1X       0x0008
#define ASIC_UVI_SPRITE2Y       0x0010
#define ASIC_UVI_SPRITE2X       0x0020
#define ASIC_UVI_SPRITE3Y       0x0040
#define ASIC_UVI_SPRITE3X       0x0080
#define ASIC_UVI_VSCROLL        0x0100
#define ASIC_UVI_PITCH          0x0200
#define ASIC_UVI_BGCOLOUR       0x0400
#define ASIC_UVI_BGCOLLIDE      0x0800
#define ASIC_UVI_SPRITECOLLIDE  0x1000
#define ASIC_UVI_GFXMODE        0x2000
#define ASIC_UVI_CHARLINE       0x4000
#define ASIC_UVI_CONSOLE        0x8000
#define ASIC_UVI_P1LEFTKEYS     0x00010000
#define ASIC_UVI_P1MIDDLEKEYS   0x00020000
#define ASIC_UVI_P1RIGHTKEYS    0x00040000
#define ASIC_UVI_P2LEFTKEYS     0x00080000
#define ASIC_UVI_P2MIDDLEKEYS   0x00100000
#define ASIC_UVI_P2RIGHTKEYS    0x00200000
#define ASIC_UVI_P1PADDLE       0x00400000
#define ASIC_UVI_P2PADDLE       0x00800000
#define ASIC_UVI_P1PALLADIUM    0x01000000
#define ASIC_UVI_P2PALLADIUM    0x02000000
#define ASIC_PVI_SPRITE0AX      0
#define ASIC_PVI_SPRITE0AY      0
#define ASIC_PVI_SPRITE0BX      0
#define ASIC_PVI_SPRITE0BY      0
#define ASIC_PVI_SPRITE1AX      0
#define ASIC_PVI_SPRITE1AY      0
#define ASIC_PVI_SPRITE1BX      0
#define ASIC_PVI_SPRITE1BY      0
#define ASIC_PVI_SPRITE2AX      0
#define ASIC_PVI_SPRITE2AY      0
#define ASIC_PVI_SPRITE2BX      0
#define ASIC_PVI_SPRITE2BY      0
#define ASIC_PVI_SPRITE3AX      0
#define ASIC_PVI_SPRITE3AY      0
#define ASIC_PVI_SPRITE3BX      0
#define ASIC_PVI_SPRITE3BY      0
#define ASIC_PVI_SIZES          0
#define ASIC_PVI_SPR01COLOURS   0
#define ASIC_PVI_SPR23COLOURS   0
#define ASIC_PVI_BGCOLOUR       0
#define ASIC_PVI_BGCOLLIDE      0
#define ASIC_PVI_SPRITECOLLIDE  0
#define ASIC_PVI_SCORELT        0
#define ASIC_PVI_SCORERT        0
#define ASIC_PVI_SCORECTRL      0
#define ASIC_PVI_PITCH          0
#define ASIC_PVI_P1PADDLE       0
#define ASIC_PVI_P2PADDLE       0
#define ASIC_IE_CONSOLE         0
#define ASIC_IE_NOISE           0
#define ASIC_IE_P1KEYS          0
#define ASIC_IE_P2KEYS          0

/* ── Stub functions (all no-ops or minimal in GAMER build) ─────────────── */
/* NOTE: do_tape, check_handler, logindirectbios are defined later in 2650.c
   — do NOT stub them here.                                                  */
static inline void checkstep(void)         {}
static inline void checkabsbranch(void)    {}
static inline void checkrelbranch(void)    {}
static inline void playsound(int x)        { (void)x; }
static inline void master_to_slave(void)   {}
static inline void getfriendly(int x)      { (void)x; }
static inline void dec_to_hex(int x)       { (void)x; }
static inline int  guestchar(int x)        { (void)x; return x; }
static inline void number_to_friendly(int a, char *b, int c, int d)
                                           { (void)a;(void)b;(void)c;(void)d; }
static inline int  conditional(void *w, int v, int wr, int x)
                                           { (void)w;(void)v;(void)wr;(void)x; return 0; }
static inline void cd2650_biosdetails1(unsigned long x) { (void)x; }
static inline void elektor_biosdetails(unsigned long x)  { (void)x; }
static inline void pipbin_biosdetails(unsigned long x)   { (void)x; }
static inline void phunsy_biosdetails(unsigned long x)   { (void)x; }
static inline void selbst_biosdetails(unsigned long x)   { (void)x; }
static inline void twin_biosdetails(unsigned long x)     { (void)x; }
/* readport/writeport: one address argument */
static inline int  readport(int a)         { (void)a; return 0; }
static inline void writeport(int a, int b) { (void)a;(void)b; }
static inline int  loadbyte(void)          { return 0; }
static inline void savebyte(int x)        { (void)x; }
static inline void verbosetape_load(void)  {}
static inline void verbosetape_save(void)  {}
static inline int  NetSendByte(int x)      { (void)x; return 0; }
static inline int  NetReceiveByte(void)    { return 0; }
