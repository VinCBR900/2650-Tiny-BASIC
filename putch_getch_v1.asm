; putch_getch_v1.asm  -  Bit-bang serial I/O for Signetics 2650
; AS50 technique: FLAG pin = TX, SENSE pin = RX
; Format: 8N1 (8 data bits, no parity, 1 stop bit)
; Baud rate: calibrated for 1 MHz clock
;
; REGISTER ALLOCATION (PUTCH and GETCH):
;   R0  ALU scratch / delay inner counter / bit test result
;   R1  data byte (TX) / received byte (RX)
;   R2  rotating bit mask ($01,$02,$04...$80)
;   R3  bit counter (8 down to 1) / half-bit flag
;
; RAM usage (at $0800):
;   TXBUF   $0800   saved TX byte (PUTCH preserves it across mask ops)
;   RXBUF   $0801   received byte accumulator (GETCH)
;   SAVR3   $0802   temporary R3 save during delay sub
;
; TIMING (1 MHz clock, 300 baud):
;   Bit period = 3333 cycles
;   BITDLY subroutine:
;     inner loop: SUBI,R0 1 (3cy) + BCTR,GT (3cy taken / 2cy fall)
;                 = 5 cy per iteration
;     BSTA overhead = 3 cy, RETC overhead = 3 cy → 6 cy total overhead
;     To hit 3333 cy: need (count * 5) + overhead ≈ 3333
;     outer call: LODI,R0 N (3cy) + BITDLY body + RETC (3cy)
;     N = (3333 - 6) / 5 = 665  → use $0299 = 665  ✓
;   Half-bit (start bit centering):
;     N = 665 / 2 = 332 → use $014C = 332
;
; CYCLE ACCURACY NOTE:
;   The PUTCH loop body has some overhead (STRA, bit test, PPSU/CPSU,
;   LODI for delay) — approximately 15-20 cycles per bit before BITDLY.
;   At 300 baud this is <1% error; fine for async serial.
;   For higher baud rates, reduce BITDLY count accordingly.
;
; Build:  asm2650 putch_getch_v1.asm putch_getch_v1.hex
; Run:    sim2650 -t putch_getch_v1.hex
; ---------------------------------------------------------------

; ── Constants ───────────────────────────────────────────────────
BDLY_N  EQU     $0299       ; full bit delay count (665)
HDLY_N  EQU     $014C       ; half bit delay count (332)

FLAG    EQU     $40         ; PSU bit 6: FLAG output (TX)
SENSE   EQU     $80         ; PSU bit 7: SENSE input (RX)
IINHIB  EQU     $20         ; PSU bit 5: interrupt inhibit

CR      EQU     $0D
LF      EQU     $0A

; ── RAM layout ($0800 = start of 2K RAM on real hardware) ───────
TXBUF   EQU     $0800       ; saved TX byte
RXBUF   EQU     $0801       ; received byte accumulator
SAVR3   EQU     $0802       ; R3 save for delay sub
BITCNT  EQU     $0803       ; bit loop counter

        ORG     $0000

; ════════════════════════════════════════════════════════════════
; RESET / MAIN
; ════════════════════════════════════════════════════════════════
RESET:
        PPSU    IINHIB      ; disable interrupts
        PPSU    FLAG        ; TX idle (mark = logic 1)

        ; send "HI\r\n" as proof-of-life
        LODI,R1 'H'
        BSTA,UN PUTCH
        LODI,R1 'I'
        BSTA,UN PUTCH
        LODI,R1 CR
        BSTA,UN PUTCH
        LODI,R1 LF
        BSTA,UN PUTCH

        ; receive one char, echo it back, repeat
ECHO:
        BSTA,UN GETCH
        BSTA,UN PUTCH
        BCTA,UN ECHO

        ; (not reached)
        HALT

; ════════════════════════════════════════════════════════════════
; BITDLY  —  fixed bit-period delay subroutine
; Trashes R0 only.
; Count passed in R0 by caller (load before BSTA,UN BITDLY)
; ════════════════════════════════════════════════════════════════
BITDLY:
BITDLY_LP:
        SUBI,R0 1
        BCTR,GT BITDLY_LP
        RETC,UN

; ════════════════════════════════════════════════════════════════
; PUTCH  —  transmit R1 via FLAG pin (8N1, LSB first)
;
; Sequence:
;   1. Save R1 to TXBUF (mask ops clobber R0, but not R1 — wait,
;      ANDZ and LODZ do clobber R0 not R1, so R1 is safe. No save needed.)
;   2. START BIT: CPSU FLAG, delay full bit
;   3. 8 DATA BITS: R2=mask, R3=counter
;      test bit, set FLAG accordingly, delay full bit
;      shift mask left (R2 = R2 + R2 via R0)
;   4. STOP BIT: PPSU FLAG, delay full bit
;   5. RETC,UN
;
; Registers on entry:  R1 = byte to send
; Registers on exit:   R1 = preserved, R0/R2/R3 = undefined
; ════════════════════════════════════════════════════════════════
PUTCH:
        ; ── START BIT ────────────────────────────────────────────
        CPSU    FLAG        ; FLAG=0 (space = start bit)
        LODI,R0 BDLY_N      ; full bit delay count
        BSTA,UN BITDLY

        ; ── 8 DATA BITS (LSB first) ──────────────────────────────
        LODI,R2 $01         ; initial mask = bit 0
        LODI,R3 8           ; 8 bits to send

PUTCH_BIT:
        ; Test: is R1 & R2 (current bit) zero or non-zero?
        LODZ    R1          ; R0 = R1 (data byte)
        ANDZ    R2          ; R0 = R0 & R2 (= R1 & mask)  CC set

        BCTR,EQ PUTCH_ZERO  ; CC=zero → send a 0 bit
        PPSU    FLAG        ; bit=1: FLAG high (mark)
        BCTA,UN PUTCH_BIT_WAIT
PUTCH_ZERO:
        CPSU    FLAG        ; bit=0: FLAG low (space)

PUTCH_BIT_WAIT:
        ; delay 1 full bit period
        LODI,R0 BDLY_N
        BSTA,UN BITDLY

        ; rotate mask left: R2 = R2 * 2
        LODZ    R2          ; R0 = R2 (current mask)
        ADDZ    R2          ; R0 = R0 + R2 = R2 * 2  (left shift)
        STRZ    R2          ; R2 = R0 (updated mask)

        SUBI,R3 1           ; decrement bit counter
        BCTR,GT PUTCH_BIT   ; loop if bits remain

        ; ── STOP BIT ─────────────────────────────────────────────
        PPSU    FLAG        ; FLAG=1 (mark = stop bit)
        LODI,R0 BDLY_N
        BSTA,UN BITDLY

        RETC,UN             ; return to caller

; ════════════════════════════════════════════════════════════════
; GETCH  —  receive byte from SENSE pin (8N1, LSB first)
; Returns received byte in R1.
;
; Sequence:
;   1. Wait for line IDLE: loop while SENSE=0 (busy/break)
;   2. Wait for START BIT: loop while SENSE=1
;   3. Delay HALF bit (sample in bit-cell centre)
;   4. 8 DATA BITS: for each bit:
;      - delay full bit
;      - sample SENSE; if 1, OR mask into RXBUF
;      - shift mask left
;   5. Delay through stop bit
;   6. Load RXBUF → R1, return
;
; Registers on entry:  (none)
; Registers on exit:   R1 = received byte; R0/R2/R3 = undefined
; ════════════════════════════════════════════════════════════════
GETCH:
        ; clear receive accumulator in RAM
        LODI,R0 $00
        STRA,R0 RXBUF

        ; ── Wait for idle (SENSE=1) ───────────────────────────────
GETCH_IDLE:
        SPSU                ; R0 = PSU register
        ANDI,R0 SENSE       ; test bit 7 (SENSE)
        BCTR,EQ GETCH_IDLE  ; zero → SENSE=0 → not idle yet, loop

        ; ── Wait for start bit (SENSE=0) ─────────────────────────
GETCH_START:
        SPSU
        ANDI,R0 SENSE
        BCTR,GT GETCH_START ; positive → SENSE=1 → no start bit yet

        ; ── Half-bit delay (centre first data bit) ───────────────
        LODI,R0 HDLY_N
        BSTA,UN BITDLY

        ; ── 8 DATA BITS ──────────────────────────────────────────
        LODI,R2 $01         ; initial mask = bit 0
        LODI,R3 8           ; bit counter

GETCH_BIT:
        ; delay 1 full bit
        LODI,R0 BDLY_N
        BSTA,UN BITDLY

        ; sample SENSE
        SPSU                ; R0 = PSU
        ANDI,R0 SENSE       ; R0 = SENSE bit (0 or $80)
        BCTR,EQ GETCH_BIT0  ; zero → received bit is 0

        ; received bit = 1: OR current mask into RXBUF
        LODA,R0 RXBUF       ; R0 = accumulator
        IORZ    R2          ; R0 = R0 | R2 (set the bit)
        STRA,R0 RXBUF       ; store back
        BCTA,UN GETCH_NEXT

GETCH_BIT0:
        ; received bit = 0: nothing to OR (bit already clear in RXBUF)

GETCH_NEXT:
        ; rotate mask left: R2 = R2 * 2
        LODZ    R2          ; R0 = R2
        ADDZ    R2          ; R0 = R2 * 2
        STRZ    R2          ; R2 = updated mask

        SUBI,R3 1           ; bit counter--
        BCTR,GT GETCH_BIT   ; loop if more bits

        ; ── Stop bit: delay through it ───────────────────────────
        LODI,R0 BDLY_N
        BSTA,UN BITDLY

        ; ── Return received byte in R1 ───────────────────────────
        LODA,R1 RXBUF       ; R1 = received byte

        RETC,UN

        END
