; =============================================================================
; Signetics 2650 Fixed Point Decimal Arithmetic Routines (AS55)
; Consolidating Signed Arithmetic, Multiplication, Division, and Alignment
; =============================================================================

; --- Global Definitions ---
R0      EQU     0               ; Register 0 (Accumulator)
R1      EQU     1               ; Register 1
R2      EQU     2               ; Register 2
R3      EQU     3               ; Register 3

; Program Status Word Lower (PSL) bit masks
C       EQU     H'01'           ; Carry/Borrow flag
COM     EQU     H'02'           ; Logical/Arithmetic Compare
OVF     EQU     H'04'           ; Overflow flag
WC      EQU     H'08'           ; With Carry flag
RS      EQU     H'10'           ; Register Bank Select
IDC     EQU     H'20'           ; Interdigit Carry flag

; Condition Codes
EQ      EQU     0               ; Equal
GT      EQU     1               ; Greater Than
LT      EQU     2               ; Less Than
UN      EQU     3               ; Unconditional

; Sign Convention (Sign-Magnitude)
; MS-Byte: Bit 7 is sign. (0 = +, 1 = -)
; Note: In some AS55 implementations, H'00' is + and H'F0' is -.
PLUS    EQU     H'00'
MINUS   EQU     H'F0'

; -----------------------------------------------------------------------------
; ROUTINE: SIGNED BCD ADDITION / SUBTRACTION
; Operation: Result = Oper1 +/- Oper2
; -----------------------------------------------------------------------------
SGAD    PPSL    WC+COM          ; Arith with Carry, Logical Compare
        LODA,R0 OPR1            ; Load sign/MSB of Operand 1
        STRA,R0 RSGN            ; Store result sign (preliminary)
        LODA,R0 OPR2            ; Load sign/MSB of Operand 2
        EORA,R0 RSGN            ; Check if signs are same
        BCTA,LT S_SUB           ; If signs different, perform subtraction
        
        ; Same signs: Perform Addition
        LODI,R3 LENG-1          ; Load index for multi-byte loop
S_ADD   LODA,R0 OPR1,R3,-       ; Fetch OPR1 byte
        ADDI,R0 H'66'           ; BCD offset
        ADDA,R0 OPR2,R3         ; Add OPR2 byte
        DAR,R0                  ; Decimal Adjust
        STRA,R0 RSLT,R3         ; Store result
        BRNR,R3 S_ADD           ; Loop until done
        RETC,UN                 ; Return

S_SUB   ; Different signs: Perform Subtraction
        LODI,R3 LENG-1
S_SB1   LODA,R0 OPR1,R3,-
        SUBA,R0 OPR2,R3         ; Subtract bytes
        DAR,R0                  ; Decimal Adjust for borrow
        STRA,R0 RSLT,R3
        BRNR,R3 S_SB1
        RETC,UN

; -----------------------------------------------------------------------------
; ROUTINE: DECIMAL MULTIPLICATION (SIGNED)
; Operation: Multiplier = Multiplicand * Multiplier
; -----------------------------------------------------------------------------
DMLT    PPSL    NC+COM          ; Setup status
        LODA,R0 MPLR            ; Fetch Multiplier sign
        EORA,R0 MPLC            ; XOR with Multiplicand sign
        STRA,R0 RSGN            ; Save result sign
        
        ; Core multiplication logic (repeated addition and shifting)
        ; This routine typically clears a product area and iterates
        ; per digit of the multiplier.
        LODI,R3 (2*LENG)-1      ; Clear result area (Double length)
M_CLR   LODI,R0 0
        STRA,R0 RSLT,R3,-
        BRNR,R3 M_CLR
        
        LODI,R2 LENG*2          ; Digit counter (2 digits per byte)
M_LOOP  BSTA,UN SHRG            ; Shift Multiplier right one digit
        ; (Addition of multiplicand to product based on digit value)
        ; ... [Implementation details from Figure 9] ...
        BDRR,R2 M_LOOP          ; Continue for all digits
        RETC,UN

; -----------------------------------------------------------------------------
; ROUTINE: DECIMAL DIVISION (SIGNED)
; Operation: Dividend = Dividend / Divisor
; -----------------------------------------------------------------------------
SDIV    PPSL    NC+COM          ; Setup status
        BSTA,UN TZER            ; Test Divisor for Zero
        BCTA,EQ Z_DIV_ERR       ; Branch if Division by Zero
        
        LODA,R0 DVSR            ; Fetch Divisor Sign
        EORA,R0 DVDN            ; Determine result sign
        STRA,R0 RSGN            ; Save Quotient Sign
        
        ; Core division (Shift and Subtract)
        LODI,R2 (LENG*2)        ; Loop for number of digits
D_LOOP  BSTA,UN SHFL            ; Shift Dividend left one digit
        ; ... [Subtraction logic from Figure 11] ...
        BDRR,R2 D_LOOP
        RETC,UN

; -----------------------------------------------------------------------------
; ROUTINE: FIXED-POINT ALIGNMENT (ALGN)
; Moves BCD digits to align decimal points based on shift count in R0.
; -----------------------------------------------------------------------------
ALGN    BCTR,EQ AL_RET          ; Return if shift count is 0
        BCTR,P  AL_LEFT         ; Positive: Shift Left
        
AL_RGHT ; Shift Right logic
        LODI,R3 LENG-1          ; Index for right shift
AR_01   LODA,R1 OPR,R3          ; Fetch byte
        ; ... [Digit shifting logic from Figure 14] ...
        BDRR,R0 AL_RGHT         ; Repeat for shift count
        RETC,UN

AL_LEFT ; Shift Left logic
        LODI,R3 0               ; Index for left shift
AL_01   LODA,R1 OPR,R3          ; Fetch byte
        ; ... [Digit shifting logic from Figure 14] ...
        BDRR,R0 AL_LEFT
AL_RET  RETC,UN                 ; Return

; --- Hardware Constraints Note ---
; Use IORZ,R0 for NOP as STRZ,R0 is undefined behavior