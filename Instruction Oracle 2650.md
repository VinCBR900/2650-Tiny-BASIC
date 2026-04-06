# Signetics 2650 Instruction Set Oracle
#### Version: 1.1, Updated: 2026-04-06

### Changes v1.0 -> v1.1:
*   BUG-ORACLE-01 FIXED: BRNR/BRNA pseudocode was wrong.
    *    v1.0 said: rn--; if(rn != 0) PC = rel  (incorrectly described BDRR behaviour)
    *    Correct:   if(rn != 0) PC = rel         (pure test, NO modification to rn)
    *    Source: 2650 User Manual, confirmed by WinArcadia hardware validation.
*   BDRR/BDRA and BIRR/BIRA verified correct (those DO modify rn).
*   Clarified LODZ,R0 / ANDZ,R0 / STRZ,R0 hardware constraints.
*   Corrected HI/LO operator convention (< = HIGH, > = LOW).
*   Added loop pattern examples for BRNR, BIRR, BDRR.
*   Added RAS (hardware stack) notes and PIPBUG interaction.

## 1. Definitions & Addressing Modes

### **Registers & Values**
* **rn**: General Purpose Register (r0, r1, r2, r3). Note: **r0** is the implicit source/destination for indexed operations and all **Z-group (e.g. ADDZ, LODZ)** instructions.
* **abs**: 15-bit Absolute Address ($0000 - $7FFF).
* **rel**: 7-bit signed displacement (-64 to +63). The origin is the address of the byte *immediately following* the instruction.
* **imm**: 8-bit Immediate value.
* **x**: Index Register (r1, r2, r3).
* **+ / -**: Auto-increment or auto-decrement of the index register (occurs BEFORE the memory access).
* **Zxxx**: Zero Page Branch instructions is signed MOD 8192, so accesses first 63 bytes of ROM memory ($0000-003F) and top 63 bytes ($7FC0-7FFF).

### **The Addressing Toolkit**
----------------------

The 2650 uses four primary classes of addressing. The **Absolute (A-group)** is the most flexible.

#### **A. Absolute Addressing (3-Byte Instructions)**

Used by: `LODA`, `STRA`, `COMA`, `ADDA`, `SUBA`, `ANDA`, `IORA`, `EORA`.

Format: `[Opcode+Reg] [Indirect+IndexControl+HighAddr] [LowAddr]`

| **Mode** | **Syntax** | **Logic** |
| --- | --- | --- |
| **Direct** | `OP,rn abs` | `rn = rn OP *(abs)` |
| **Indirect** | `OP,rn *abs` | `rn = rn OP *(*(abs))` |
| **Indexed** | `OP,r0 abs,rx` | `r0 = r0 OP *(abs + rx)` |
| **Indirect Indexed** | `OP,r0 *abs,rx` | `r0 = r0 OP *(*(abs) + rx)` |
| **Auto-Increment** | `OP,r0 abs,rx+` | `rx++; r0 = r0 OP *(abs + rx)` |
| **Auto-Decrement** | `OP,r0 abs,rx-` | `rx--; r0 = r0 OP *(abs + rx)` |

-   **Note on Indexing:** When indexing is used, the instruction's register field specifies the **Index Register (rx)**. The data source/destination is hardware-locked to **r0**.

#### **B. Relative Addressing (2-Byte Instructions)**

Used by: `LODR`, `STRR`, `COMR`, `ADDR`, `SUBR`, `ANDR`, `IORR`, `EORR`.

-   **Target:** PC of next instruction + signed 7-bit displacement (-64 to +63).

-   **Indirect:** `OP,rn *rel` is supported.

#### **C. Immediate Addressing (2-Byte Instructions)**

Used by: `LODI`, `COMI`, `ADDI`, `SUBI`, `ANDI`, `IORI`, `EORI`.

-   **Target:** The 8-bit literal value in the second byte of the instruction.

#### **D. Register Addressing (1-Byte Instructions)**

Used by: `LODZ`, `STRZ`, `COMZ`, `ADDZ`, `SUBZ`, `ANDZ`, `IORZ`, `EORZ`.

-   **Logic:** Always involves **r0**. E.g., `ADDZ,r1` performs `r0 = r0 + r1`

#### **The Indirect Flag (*)**
* The asterisk denotes **Indirect Addressing**. This sets Bit 7 of the address/displacement byte. The CPU fetches a 15-bit pointer from the calculated effective address and uses that as the final target.
* **WARNING**: When building an indirect pointer in RAM, the HIGH byte of the pointer must have bit 7 CLEAR. If bit 7 of the high byte is set, the CPU interprets it as a second level of indirection (double-indirect), fetching another pointer. This will cause incorrect behaviour or crashes if not intended.

### **HI/LO Byte Operators** *(WinArcadia / asm2650.py / Signetics standard)*
* **`<ADDR`** = **HIGH byte** (bits 15:8) — e.g. `<$1584` = `$15`
* **`>ADDR`** = **LOW byte**  (bits  7:0) — e.g. `>$1584` = `$84`
* This is confirmed by WinArcadia docs: *"<FOO for the high byte, >FOO for the low byte"*.
* When storing a 16-bit pointer for indirect addressing: store `<ADDR` at the lower memory address (HI byte), `>ADDR` at the higher address (LO byte).

### **Condition Codes (CC)**
The Condition Code (CC) is a 2-bit field in the PSL (Program Status Lower), bits 7:6, updated after most arithmetic/load/compare operations:
* **00 (Zero/Equal)**:    Result is zero, or comparison was equal.         CC field = $00
* **01 (Positive/GT)**:   Result is positive (bit 7=0, not zero), or Reg > Memory.  CC field = $40
* **10 (Negative/LT)**:   Result is negative (bit 7=1), or Reg < Memory.  CC field = $80
* **11 (Unconditional)**: Used in branch fields only; not set by ALU.

### **CC After ADD** *(critical for 16-bit arithmetic)*
```
  No carry (C=0):              CC = GT  ← use BCTA,GT to SKIP hi-byte increment
  Carry + result zero (C=1):   CC = EQ
  Carry + result nonzero (C=1):CC = LT
```
*16-bit increment idiom:*
```asm
ADDI,R0 1 ; increment lo byte
STRA,R0 PTR_LO
BCTA,GT   skip  ; GT = no carry -> skip hi-byte increment
LODA,R0 PTR_HI
ADDI,R0 1
STRA,R0 PTR_HI
skip:
```

### **CC After SUB** *(critical for 16-bit arithmetic)*
```
  No borrow, result nonzero (C=1): CC = GT
  No borrow, result zero    (C=1): CC = EQ
  Borrow (C=0):                    CC = LT  ← use BCFA,LT to SKIP hi-byte decrement
```
*NEVER use BCTA,GT for a borrow skip — it misses the EQ case.*

### **Branch Masks (cond)**
Used in `BCT`, `BCF`, `BST`, `BSF`, `RET` instructions:
* **EQ / Z**: Zero (Mask %00)
* **GT / P**: Positive (Mask %01)
* **LT / N**: Negative (Mask %10)
* **UN**:     Unconditional (Mask %11)

### **Hardware Stack (RAS)**
The Return Address Stack is 8 entries deep, wrapping. PIPBUG 1 uses 3 slots on entry, leaving 5 free for user code. COUT uses up to 2 more levels internally (COUT→DLAY), so maximum safe user call depth from PIPBUG context is 5 levels. COUT and CHIN both use register bank 1 (PPSL RS / CPSL RS) internally and restore RS on exit — bank-0 registers R1-R3 survive COUT/CHIN calls unchanged.

---

## 2. Arithmetic & Logical Instructions

### **ADD (Addition)**
```asm
ADDA,rn abs         ;rn += *(abs);                             ;4,3
ADDA,r0 abs,x       ;r0 += *(abs + x);                         ;4,3
ADDA,r0 abs,x+      ;r0 += *(abs + ++x);                       ;4,3
ADDA,r0 abs,x-      ;r0 += *(abs + --x);                       ;4,3
ADDA,rn *abs        ;rn += *(*(abs));                          ;6,3
ADDI,rn imm         ;rn += imm;                                ;2,2
ADDR,rn rel         ;rn += *(rel);                             ;3,2
ADDR,rn *rel        ;rn += *(*(rel));                          ;5,2
ADDZ,rn             ;r0 += rn;                                 ;2,1
```

### **SUB (Subtraction)**
```asm
SUBA,rn abs         ;rn -= *(abs);                             ;4,3
SUBA,r0 abs,x       ;r0 -= *(abs + x);                         ;4,3
SUBA,r0 abs,x+      ;r0 -= *(abs + ++x);                       ;4,3
SUBA,r0 abs,x-      ;r0 -= *(abs + --x);                       ;4,3
SUBA,rn *abs        ;rn -= *(*(abs));                          ;6,3
SUBI,rn imm         ;rn -= imm;                                ;2,2
SUBR,rn rel         ;rn -= *(rel);                             ;3,2
SUBZ,rn             ;r0 -= rn;                                 ;2,1
```

### **AND / IOR / EOR (Bitwise)**
```asm
ANDA,rn abs         ;rn &= *(abs);                             ;4,3
ANDI,rn imm         ;rn &= imm;                                ;2,2
ANDR,rn rel         ;rn &= *(rel);                             ;3,2
ANDZ,rn             ;r0 &= rn;                                 ;2,1
; NOTE: ANDZ,R0 is NOT a valid instruction — encodes as HALT ($40). Assembler warns.

IORA,rn abs         ;rn |= *(abs);                             ;4,3
IORI,rn imm         ;rn |= imm;                                ;2,2
IORR,rn rel         ;rn |= *(rel);                             ;3,2
IORZ,rn             ;r0 |= rn;                                 ;2,1
; Use IORZ,R0 to clear R0 to zero (R0 ^= R0 also works via EORZ,R0)

EORA,rn abs         ;rn ^= *(abs);                             ;4,3
EORI,rn imm         ;rn ^= imm;                                ;2,2
EORR,rn rel         ;rn ^= *(rel);                             ;3,2
EORZ,rn             ;r0 ^= rn;                                 ;2,1
```

---

## 3. Load, Store & Compare

### **LOD (Load)**
```asm
LODA,rn abs         ;rn = *(abs);                              ;4,3
LODI,rn imm         ;rn = imm;                                 ;2,2
LODR,rn rel         ;rn = *(rel);                              ;3,2
LODZ,rn             ;r0 = rn;    (destination is ALWAYS r0)   ;2,1
; NOTE: LODZ,R0 result is undefined per 2650 manual (R0=R0 self-load). Assembler warns.
; Use IORZ,R0 (R0 ^= R0) to guarantee R0=0, or EORZ,R0 to toggle.

; Indexed absolute (R0 is always the destination, rn in opcode = index register):
LODA,r0 abs,x       ;r0 = *(abs + x);    x unchanged           ;4,3
LODA,r0 abs,x+      ;x++; r0 = *(abs + x);  pre-increment      ;4,3
LODA,r0 abs,x-      ;x--; r0 = *(abs + x);  pre-decrement      ;4,3
LODA,rn *abs        ;rn = *(*(abs));     indirect               ;6,3
LODA,r0 *abs,x      ;r0 = *(*(abs) + x);                       ;6,3
```
**KEY**: In indexed absolute mode the register field in the opcode byte encodes the **index register**, NOT the destination. Destination is always R0. `LODA,R0 BASE,R2` emits opcode `$0E` (R2 in field), not `$0C` (R0). Confirmed by asm2650.py and WinArcadia.

### **STR (Store)**
```asm
STRA,rn abs         ;*(abs) = rn;                              ;4,3
STRR,rn rel         ;*(rel) = rn;                              ;3,2
STRA,r0 abs,x       ;*(abs + x) = r0;                          ;4,3
STRA,r0 abs,x+      ;x++; *(abs + x) = r0;                    ;4,3
STRA,r0 abs,x-      ;x--; *(abs + x) = r0;                    ;4,3
STRA,rn *abs        ;*(*(abs)) = rn;                           ;6,3
STRZ,rn             ;rn = r0;   (source is ALWAYS r0)          ;2,1
; NOTE: STRZ,R0 is NOT valid — encodes as NOP ($C0). Assembler warns.
```

### **COM (Compare — sets CC only, no register write)**
```asm
COMA,rn abs         ;rn : *(abs) -> sets CC;                   ;4,3
COMI,rn imm         ;rn : imm   -> sets CC;                    ;2,2
COMR,rn rel         ;rn : *(rel) -> sets CC;                   ;3,2
COMZ,rn             ;r0 : rn    -> sets CC;                    ;2,1
COMA,r0 abs,x       ;r0 : *(abs+x) -> sets CC;                 ;4,3
```
COM uses signed comparison by default (PSL COM=0). Set COM=1 via `PPSL $02` for unsigned.

---

## 4. Branching & Subroutine Calls

### **Conditional Branching**
```asm
BCTA,cond abs       ;if(cond) PC = abs;                        ;3,3
BCTR,cond rel       ;if(cond) PC += rel;                       ;3,2
BCFA,cond abs       ;if(!cond) PC = abs;                       ;3,3
BCFR,cond rel       ;if(!cond) PC += rel;                      ;3,2
```

### **Subroutine Calls / Returns**
```asm
BSTA,cond abs       ;if(cond) { push(PC); PC = abs; }          ;3,3
BSTR,cond rel       ;if(cond) { push(PC); PC += rel; }         ;3,2
BSFA,cond abs       ;if(!cond){ push(PC); PC = abs; }          ;3,3
BSFR,cond rel       ;if(!cond){ push(PC); PC += rel; }         ;3,2
RETC,cond           ;if(cond) PC = pop();                      ;3,1
RETE,cond           ;if(cond) { PC = pop(); enable ints; }     ;3,1
```

### **Zero-Page Branches** *(target must be within $0000-$007F)*
```asm
ZBRR    offset      ;PC = page_base + sign_extend_7bit(offset)          ;2,2
ZBSR    offset      ;push(PC); PC = page_base + sign_extend_7bit(offset) ;3,2
```
* Offset is a signed 7-bit value: range -64 to +63.
* Under PIPBUG, $0000-$007F is ROM — no user-placed stubs possible. ZBSR/ZBRR are Phase 2 standalone only.
* ZBRR: WinArcadia assembler requires an operand (quirk). Use `RETC,UN` instead for PIPBUG-phase source.

### **Indexed Branch**
```asm
BXA     abs,r3      ;PC = abs + r3;                            ;3,3
BSXA    abs,r3      ;push(PC); PC = abs + r3;                  ;3,3
```

---

## 5. Register Branch Instructions *(CRITICAL — read carefully)*

These instructions branch based on a register value. They are **not** based on the CC flags.

### **BRNR / BRNA — Branch if Register Non-Zero**
```asm
BRNR,rn rel         ;if (rn != 0) PC += rel;                   ;3,2
BRNA,rn abs         ;if (rn != 0) PC = abs;                    ;3,3
BRNA,rn *abs        ;if (rn != 0) PC = *(abs);                 ;5,3
```
**CRITICAL**: Register `rn` is **TESTED BUT NOT MODIFIED**. This is a pure conditional branch.
To use as a counted loop you MUST decrement separately:
```asm
        LODI,R1 5
LOOP:
        ; body
        SUBI,R1 1       ; explicit decrement (2 bytes)
        BRNR,R1 LOOP    ; branch if R1 != 0 (2 bytes) — total 4 bytes per loop-close
```
Compare: `SUBI,R1 1 / BCFA,EQ LOOP` = 5 bytes. BRNR saves 1 byte per loop-close.
*Confirmed by 2650 User Manual pseudocode: `if (rn != 0) goto abs` — no decrement.*

### **BIRR / BIRA — Branch and Increment Register**
```asm
BIRR,rn rel         ;rn++; if (rn != 0) PC += rel;             ;3,2
BIRA,rn abs         ;rn++; if (rn != 0) PC = abs;              ;3,3
```
Register IS incremented before test. Loop exits when `rn` wraps from $FF to $00.
```asm
        LODI,R1 $FD     ; will take 3 iterations: FD->FE->FF->00 (exits)
LOOP:
        ; body
        BIRR,R1 LOOP    ; R1++, branch if R1 != 0 (2 bytes per loop-close)
```

### **BDRR / BDRA — Branch and Decrement Register** *(used by PIPBUG delays)*
```asm
BDRR,rn rel         ;rn--; if (rn >= 0) PC += rel;             ;3,2
BDRA,rn abs         ;rn--; if (rn >= 0) PC = abs;              ;3,3
```
Register IS decremented before SIGNED test. Loop exits when `rn` underflows to $FF (-1 signed).
This is the most compact loop instruction — just 2 bytes, decrement is free:
```asm
        LODI,R1 4       ; will take 5 iterations: 4->3->2->1->0->FF(exits)
LOOP:
        ; body
        BDRR,R1 LOOP    ; R1--, branch if R1 >= 0 signed (2 bytes per loop-close)
; After loop: R1 = $FF (-1). If you need R1=0 after, add LODI,R1 0.
```
PIPBUG uses `BDRR,R0 $` (self-branch) as a single-instruction counted delay.

### **Summary: register branch family**
| Instruction | Modifies rn? | Branch condition | Exit at |
|---|---|---|---|
| BRNR/BRNA | No  | rn != 0 | rn == 0 (you decrement) |
| BIRR/BIRA | Yes (++) | rn != 0 after inc | rn wraps $FF→$00 |
| BDRR/BDRA | Yes (--) | rn >= 0 signed | rn wraps $00→$FF |

### **BSNR / BSNA — Branch to Subroutine if Register Non-Zero**
```asm
BSNR,rn rel         ;if (rn != 0) { push(PC); PC += rel; }     ;3,2
BSNA,rn abs         ;if (rn != 0) { push(PC); PC = abs; }      ;3,3
```
Same non-modifying semantics as BRNR. Tests register, no side effect on rn.

---

## 6. System, Status & I/O

### **PSW Management**
```asm
SPSU / SPSL         ;r0 = PSU / PSL;  (Store PSW to r0)        ;2,1
LPSU / LPSL         ;PSU / PSL = r0;  (Load PSW from r0)       ;2,1
CPSU / CPSL imm     ;PSU/PSL &= ~imm; (Clear bits)             ;3,2
PPSU / PPSL imm     ;PSU/PSL |=  imm; (Preset/Set bits)        ;3,2
TPSU / TPSL imm     ;test: CC=EQ if all bits set, else CC=LT   ;3,2
```

### **PSL Bit Layout**
```
PSL: CC1[7] CC0[6] IDC[5] RS[4] WC[3] OVF[2] COM[1] C[0]
  CC  = bits 7:6 — condition code
  IDC = bit 5    — inter-digit carry (BCD)
  RS  = bit 4    — register select (bank switching, R1-R3 only; R0 always bank 0)
  WC  = bit 3    — with carry (enables carry-in for ADD/SUB)
  OVF = bit 2    — overflow
  COM = bit 1    — compare mode (0=signed, 1=unsigned)
  C   = bit 0    — carry / borrow
```

### **PSU Bit Layout**
```
PSU: S[7] F[6] II[5] -[4] -[3] SP[2:0]
  S   = SENSE input pin state
  F   = FLAG output pin
  II  = interrupt inhibit
  SP  = return address stack pointer (0-7)
```

### **Register Bank Switching**
```asm
PPSL $10    ; RS=1: R1, R2, R3 now address bank-1 registers (R1', R2', R3')
CPSL $10    ; RS=0: R1, R2, R3 address bank-0 registers again
```
R0 is always bank-0 regardless of RS. PIPBUG COUT uses bank-1 internally and restores RS on exit.

### **I/O Operations**
```asm
REDD,rn             ;rn = Data  input port;                    ;3,1
REDC,rn             ;rn = Control input port;                  ;3,1
REDE,rn             ;rn = Extended input (SENSE-pin serial);   ;2,1
WRTD,rn             ;Data  output port = rn;                   ;3,1
WRTC,rn             ;Control output = rn;                      ;3,1
WRTE,rn             ;Extended output (FLAG-pin serial);        ;2,1
```

### **Miscellaneous**
```asm
HALT                ;Stop CPU;         opcode $40             ;1,1
NOP                 ;No Operation;     opcode $C0             ;2,1
DAR,rn              ;Decimal Adjust rn (BCD correction);       ;3,1
RRR,rn              ;Rotate Right rn (circular or w/carry);    ;2,1
RRL,rn              ;Rotate Left  rn (circular or w/carry);    ;2,1
TMI,rn imm          ;Test mask: CC=EQ if (rn&imm)==imm, else LT ;3,2
```

---

## 7. Hardware Constraints (Silicon Bugs / Undefined Behaviour)

These are encoded as stated but produce undefined or incorrect results:

| Instruction | Opcode | Issue | Assembler behaviour |
|---|---|---|---|
| `LODZ,R0` | `$00` | R0=R0 self-load, result undefined | Warns, emits `$00` |
| `ANDZ,R0` | `$40` | Encodes as HALT | Warns, emits `$40` (HALT) |
| `STRZ,R0` | `$C0` | Encodes as NOP  | Warns, emits `$C0` (NOP)  |

Use `IORZ,R0` (R0 \|= R0, no-op) or `EORZ,R0` (R0 ^= R0 → clears R0) instead of `LODZ,R0`.

---

## 8. PIPBUG 1 Environment (WinArcadia)

```
COUT  $02B4   BSTA,UN $02B4   R0 → terminal         confirmed working
CHIN  $0286   BSTA,UN $0286   R0 ← terminal         non-blocking in WinArcadia
CRLF  $008A   BSTA,UN $008A   print CR+LF
```
User code: `ORG $0C00`, run via `G 0C00`. PIPBUG ROM `$0000-$03FF` (read-only).
First free RAM: `$0440`. PIPBUG RAM `$0400-$043F` (reserved).
ZBSR/ZBRR target range `$0000-$007F` is entirely within PIPBUG ROM — unusable under PIPBUG.

---

*This document is a validated reference for the uBASIC2650 project.
All entries confirmed against asm2650.py v2.4.1 and WinArcadia v36.04 PIPBUG 1.*
