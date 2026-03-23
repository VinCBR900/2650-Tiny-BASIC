; putch_getch_v2.asm  -  Bit-bang serial I/O for Signetics 2650
; AS50 technique: FLAG pin = TX, SENSE pin = RX
; Format: 8N1, 300 baud @ 1 MHz clock
;
; v1 BUG FIXED: LODI is 8-bit only — can't load BDLY_N=$0299.
;   Solution: two-level delay loop, both counts fit in one byte.
;
; TWO-LEVEL DELAY (300 baud @ 1 MHz = 3333 cycles/bit):
;   BITDLY structure:
;     outer: LODI,R0 BDLY_I   (3 cy)
;   inner: SUBI,R0 1           (3 cy)
;            BCTR,GT inner     (3 cy taken / 2 fall)
;     SUBI,R3 1                (3 cy) outer counter
;     BCTR,GT outer            (3 cy taken / 2 fall)
;     RETC,UN                  (3 cy)
;
;   Per outer iteration: 3 + inner*(5) + 5 = inner*5 + 8
;   Total: outer * (inner*5 + 8) + 3 (RETC)
;   outer=4, inner=165: 4*(165*5+8)+3 = 4*833+3 = 3335 cy  (0.06% err)
;   Half-bit: outer=2, inner=165: 2*833+3 = 1669 cy
;
;   Both BDLY_O=4 and BDLY_I=165 fit in one byte. ✓
;
; REGISTER ALLOCATION:
;   R0  ALU scratch / inner delay counter
;   R1  TX data byte / RX result (preserved across bit loop)
;   R2  rotating bit mask ($01→$80)
;   R3  outer delay counter / bit loop counter
;       (R3 is reloaded from RAM between uses — see BITCNT)
;
; RAM layout:
;   BITCNT  $0800  bit loop counter save/restore
;   RXBUF   $0801  received byte accumulator
;
; Build:  asm2650 putch_getch_v2.asm putch_getch_v2.hex
; Run:    sim2650 -s putch_getch_v2.hex
; ---------------------------------------------------------------

FLAG    EQU     $40         ; PSU bit 6: FLAG output (TX)
SENSE   EQU     $80         ; PSU bit 7: SENSE input (RX)
IINHIB  EQU     $20         ; PSU bit 5: interrupt inhibit

BDLY_O  EQU     4           ; outer delay count (fits in 1 byte)
BDLY_I  EQU     165         ; inner delay count (fits in 1 byte)
HDLY_O  EQU     2           ; half-bit outer count

CR      EQU     $0D
LF      EQU     $0A

BITCNT  EQU     $0800       ; bit loop counter (RAM)
RXBUF   EQU     $0801       ; RX accumulator (RAM)

        ORG     $0000

; ════════════════════════════════════════════════════════════════
; RESET / MAIN — send "HI\r\n" then halt
; ════════════════════════════════════════════════════════════════
RESET:
        PPSU    IINHIB      ; disable interrupts
        PPSU    FLAG        ; TX idle (mark = 1)

        LODI,R1 'H'
        BSTA,UN PUTCH
        LODI,R1 'I'
        BSTA,UN PUTCH
        LODI,R1 CR
        BSTA,UN PUTCH
        LODI,R1 LF
        BSTA,UN PUTCH
        HALT

; ════════════════════════════════════════════════════════════════
; BITDLY — two-level bit-period delay
;
; Entry: R3 = outer count (4 = full bit, 2 = half bit)
; Uses:  R0 (inner counter), R3 (outer counter)
; Cycles: R3*(BDLY_I*5 + 8) + 3  ≈ 3335 for R3=4
; ════════════════════════════════════════════════════════════════
BITDLY:
BDLY_OUT:
        LODI,R0 BDLY_I      ; reload inner counter each outer pass
BDLY_IN:
        SUBI,R0 1
        BCTR,GT BDLY_IN     ; inner loop
        SUBI,R3 1
        BCTR,GT BDLY_OUT    ; outer loop
        RETC,UN

; ════════════════════════════════════════════════════════════════
; PUTCH — transmit R1 via FLAG pin (8N1, LSB first)
;
; Entry: R1 = byte to transmit
; Exit:  R1 preserved; R0/R2/R3 clobbered
; Uses BITCNT ($0800) to save/restore R3 across delay calls
; ════════════════════════════════════════════════════════════════
PUTCH:
        ; ── start bit ────────────────────────────────────────────
        CPSU    FLAG                ; FLAG=0 (space)
        LODI,R3 BDLY_O
        BSTA,UN BITDLY

        ; ── 8 data bits, LSB first ────────────────────────────────
        LODI,R2 $01                 ; initial mask = bit 0
        LODI,R3 8
        STRA,R3 BITCNT              ; save bit counter to RAM

PUTCH_BIT:
        LODZ    R1                  ; R0 = data byte
        ANDZ    R2                  ; R0 = data & mask  (sets CC)
        BCTR,EQ PUTCH_ZERO          ; CC=zero → bit is 0
        PPSU    FLAG                ; bit=1: FLAG high (mark)
        BCTA,UN PUTCH_WAIT
PUTCH_ZERO:
        CPSU    FLAG                ; bit=0: FLAG low (space)

PUTCH_WAIT:
        LODI,R3 BDLY_O              ; delay one bit period
        BSTA,UN BITDLY              ; (R3 used by BITDLY — clobbers it)

        ; shift mask left (R2 = R2*2), using R0 as scratch
        LODZ    R2                  ; R0 = mask
        ADDZ    R2                  ; R0 = mask*2
        STRZ    R2                  ; R2 = updated mask

        ; restore and decrement bit counter from RAM
        LODA,R3 BITCNT              ; R3 = bit count
        SUBI,R3 1                   ; R3--
        STRA,R3 BITCNT              ; save back
        BCTR,GT PUTCH_BIT           ; loop if bits remain

        ; ── stop bit ─────────────────────────────────────────────
        PPSU    FLAG                ; FLAG=1 (mark)
        LODI,R3 BDLY_O
        BSTA,UN BITDLY

        RETC,UN

; ════════════════════════════════════════════════════════════════
; GETCH — receive byte from SENSE pin (8N1, LSB first)
;
; Entry: (none)
; Exit:  R1 = received byte; R0/R2/R3 clobbered
; ════════════════════════════════════════════════════════════════
GETCH:
        ; clear receive accumulator
        LODI,R0 $00
        STRA,R0 RXBUF

        ; ── wait for line idle (SENSE=1 = mark) ──────────────────
GETCH_IDLE:
        SPSU                        ; R0 = PSU
        ANDI,R0 SENSE               ; test SENSE bit
        BCTR,EQ GETCH_IDLE          ; zero → SENSE=0 → line busy, wait

        ; ── wait for start bit (SENSE falls to 0) ────────────────
GETCH_START:
        SPSU
        ANDI,R0 SENSE
        BCTR,GT GETCH_START         ; positive → SENSE=1 → no start yet

        ; ── half-bit delay — sample in centre of bit cells ───────
        LODI,R3 HDLY_O
        BSTA,UN BITDLY

        ; ── 8 data bits ──────────────────────────────────────────
        LODI,R2 $01                 ; initial mask = bit 0
        LODI,R3 8
        STRA,R3 BITCNT              ; save bit counter

GETCH_BIT:
        ; delay one full bit period before sampling
        LODI,R3 BDLY_O
        BSTA,UN BITDLY

        ; sample SENSE
        SPSU                        ; R0 = PSU
        ANDI,R0 SENSE               ; test SENSE bit
        BCTR,EQ GETCH_BIT0          ; zero → received bit is 0

        ; received bit=1: OR mask into RXBUF
        LODA,R0 RXBUF
        IORZ    R2                  ; R0 = RXBUF | mask
        STRA,R0 RXBUF
        BCTA,UN GETCH_NEXT

GETCH_BIT0:                         ; received bit=0: nothing to OR

GETCH_NEXT:
        ; shift mask left
        LODZ    R2
        ADDZ    R2
        STRZ    R2

        ; restore, decrement, save bit counter
        LODA,R3 BITCNT
        SUBI,R3 1
        STRA,R3 BITCNT
        BCTR,GT GETCH_BIT

        ; ── delay through stop bit ────────────────────────────────
        LODI,R3 BDLY_O
        BSTA,UN BITDLY

        ; ── return received byte in R1 ────────────────────────────
        LODA,R1 RXBUF

        RETC,UN

        END
