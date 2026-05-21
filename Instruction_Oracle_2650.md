# **Signetics 2650 Instruction Set Oracle v1.5**

## **1\. Definitions & Addressing Modes**

**Registers & Values**

* **rn**: General Purpose Register (r0, r1, r2, r3). Note: **r0** is the implicit source/destination for indexed operations and all **Z-group (e.g. ADDZ, LODZ)** instructions.  
* **abs**: 15-bit Absolute Address ($0000 \- $7FFF).  
* **rel**: 7-bit signed displacement (-64 to \+63). The origin is the address of the byte *immediately following* the instruction.  
* **imm**: 8-bit Immediate value.  
* **x**: Index Register (r1, r2, r3).  
* **\+ / \-**: Auto-increment or auto-decrement of the index register (occurs BEFORE the memory access).

**The Addressing Toolkit**  
The 2650 uses four primary classes of addressing. The **Absolute (A-group)** is the most flexible.

### **A. Absolute Addressing (3-Byte Instructions)**

Used by: LODA, STRA, COMA, ADDA, SUBA, ANDA, IORA, EORA.  
Format: \[Opcode+Reg\] \[Indirect+IndexControl+HighAddr\] \[LowAddr\]

| Mode | Syntax | Logic   |
| :---- | :---- | :---- |
| **Direct** | OP,rn abs | rn \= rn OP \*(abs) |
| **Indirect** | OP,rn \*abs | rn \= rn OP \*(\*(abs)) |
| **Indexed** | OP,r0 abs,rx | r0 \= r0 OP \*(abs \+ rx) |
| **Indirect Indexed** | OP,r0 \*abs,rx | r0 \= r0 OP \*(\*(abs) \+ rx) |
| **Auto-Increment** | OP,r0 abs,rx+ | rx++; r0 \= r0 OP \*(abs \+ rx) |
| **Auto-Decrement** | OP,r0 abs,rx- | rx--; r0 \= r0 OP \*(abs \+ rx) |

**Note on Indexing:** When indexing is used, the instruction's register field specifies the **Index Register (rx)**. The data source/destination is hardware-locked to **r0**.

### **B. Relative Addressing (2-Byte Instructions)**

* **Target:** PC of next instruction \+ signed 7-bit displacement (-64 to \+63).  
* **Indirect:** OP,rn \*rel is supported.

### **C. Immediate Addressing (2-Byte Instructions)**

* **Target:** The 8-bit literal value in the second byte of the instruction.

### **D. Register Addressing (1-Byte Instructions)**

* **Logic:** Always involves **r0**. E.g., ADDZ,r1 performs r0 \= r0 \+ r1.

### **The Indirect Flag (\*)**

* The asterisk denotes **Indirect Addressing**. This sets Bit 7 of the address/displacement byte. The CPU fetches a 15-bit pointer from the calculated effective address and uses that as the final target.  
* **WARNING:** When building an indirect pointer in RAM, the HIGH byte of the pointer must have bit 7 CLEAR. If bit 7 of the high byte is set, the CPU interprets it as a second level of indirection (double-indirect), fetching another pointer. This will cause incorrect behaviour or crashes if not intended.

Pointer Allocation Rule:   
Any RAM vector blocks allocated to hold target addresses for indirect references   
(\*abs) must reside strictly within physical addresses where the resolved pointer value's   
high byte has bit 7 clear (e.g., target address \<= $7FFF). If you attempt to point   
to an address with Bit 15 set, the 2650 will enter an indirect resolution loop,   
clobbering execution tracking.

**HI/LO Byte Operators** *(WinArcadia / asm2650.py / Signetics standard)*

* **\<ADDR** \= **HIGH byte** (bits 15:8) — e.g. \<$1584 \= $15.  
* **\>ADDR** \= **LOW byte** (bits 7:0) — e.g. \>$1584 \= $84.  
* This is confirmed by WinArcadia docs: *"\<FOO for the high byte, \>FOO for the low byte"*.  
* When storing a 16-bit pointer for indirect addressing: store \<ADDR at the lower memory address (HI byte), \>ADDR at the higher address (LO byte).

## **2\. Condition Codes (CC) & Core Execution Model**

The Condition Code (CC) is a 2-bit field in the PSL (Program Status Lower), bits 7:6.  
**IMPORTANT:** The semantic meaning of the CC field depends completely on the instruction class that produced it.  
**CC Producer Semantics**

| Instruction Class | EQ ($00) | GT ($40) | LT ($80)   |
| :---- | :---- | :---- | :---- |
| **Arithmetic / Logical / Load / Rotate** | Result is zero | Result is positive nonzero (bit 7 clear) | Result is negative (bit 7 set) |
| **Compare (COMx)** | Equal | Greater Than | Less Than |
| **Bit-Test (TPSL, TPSU, TMI)** | All selected bits are 1 | *NEVER generated* | One or more selected bits are clear |

This distinction is critical. LT after a bit-test instruction (TPSL/TPSU/TMI) means **test failed** (false), NOT "negative".  
**Core Conditional Execution Model**  
Most conditional instructions do NOT directly test raw arithmetic state, carry, or independent PSL bits. Instead:

1. A producer instruction modifies and establishes the 2-bit CC state.  
2. Branch, call, or return instructions evaluate that CC field.

### **CC After ADD (WinArcadia hardware-validated)**

CC is set from the result byte VALUE (not from carry state):  
  result \>= 128 (bit 7 set) → CC \= LT   ← even if no carry\!  
  result \> 0 and \< 128      → CC \= GT  
  result \== 0               → CC \= EQ   (only when carry wraps to 0\)

Carry bit (PSL\_C, bit 0\) is set independently:  
  dest\_int \>= 256  → PSL\_C \= 1  (carry occurred)  
  dest\_int \< 256   → PSL\_C \= 0  (no carry)

**CORRECT 16-bit increment idiom** (works for ALL lo-byte values $00–$FF):

        ADDI,R0 1         ; increment lo byte  
        STRA,R0 PTR\_LO    ; store (does NOT clobber CC or PSL\_C)  
        TPSL $01          ; CC=EQ if C=1 (carry), CC=LT if C=0 (no carry)  
        BCTR,LT skip      ; branch if no carry (C=0) → skip hi-byte increment  
        LODA,R0 PTR\_HI  
        ADDI,R0 1  
        STRA,R0 PTR\_HI  
skip:

**WRONG idiom** (only works when lo-byte result is $01–$7F):

        ADDI,R0 1  
        STRA,R0 PTR\_LO  
        RETC,GT           ; BROKEN: GT only fires for result 0x01-0x7F  
                          ; result $80-$FE (no carry\!) gives CC=LT → falls through

### **CC After SUB (WinArcadia hardware-validated)**

CC is set from the result byte VALUE:  
  result \>= 128 (bit 7 set) → CC \= LT  
  result \> 0 and \< 128      → CC \= GT  
  result \== 0               → CC \= EQ

Borrow: PSL\_C=1 means NO borrow (dest\_int \>= 0), PSL\_C=0 means borrow.  
The carry flag acts as NOT-borrow after subtraction:  
  C=1 \-\> no borrow occurred  
  C=0 \-\> borrow occurred

**Correct borrow-skip idiom:**

        SUBI,R0 1  
        STRA,R0 PTR\_LO  
        BCFR,LT skip      ; branch if C=1 (no borrow) — PSL\_C set independently

BCFR,LT (branch if NOT LT) catches both GT and EQ from the result value, which together cover all no-borrow cases. This works correctly because borrow (C=0) always produces a result with bit 7 set (result=$FF for $00-1), giving CC=LT, and no-borrow always produces a result in the visible 0-247 range.

### **STR CC Behaviour (WinArcadia hardware-validated)**

* **STRA,rn / STRR,rn**: Do **NOT** modify CC or PSL. Uses cpuwrite\_2650() only.  
* **STRZ,rn**: **DOES** modify CC via WRITEREGCC. Sets CC from the stored value.  
* ⚠️ **HAZARD:** This differs from STRA/STRR and can unintentionally destroy arithmetic result states or status flags from prior operations.

### **Branch Masks (cond)**

Used in BCT, BCF, BST, BSF, RET, and RETE instructions:

* **EQ / Z**: Zero / Equal / Test-True (Mask %00)  
* **GT / P**: Positive / Greater (Mask %01)  
* **LT / N**: Negative / Less / Test-False (Mask %10)  
* **UN**: Unconditional (Mask %11)

BCF and BSF instructions do NOT test a distinct, separate "false" condition bit. They simply invert the CC comparison evaluation logic:

* BCTR,v : branch if CC \== v  
* BCFR,v : branch if CC \!= v (Note: v=3 \[UN\] is reserved for branch-false)

### **Hardware Stack (RAS)**

The Return Address Stack is 8 entries deep, wrapping. PIPBUG 1 uses 3 slots on entry, leaving 5 free for user code. COUT uses up to 2 more levels internally (COUT→DLAY), so maximum safe user call depth from PIPBUG context is 5 levels. COUT and CHIN both use register bank 1 (PPSL RS / CPSL RS) internally and restore RS on exit — bank-0 registers R1-R3 survive COUT/CHIN calls unchanged.

## **3\. Instruction Set Reference**

### **Arithmetic & Logical Instructions**

ADDA,rn abs         ;rn \+= \*(abs);                             ;4,3  
ADDA,r0 abs,x       ;r0 \+= \*(abs \+ x);                         ;4,3  
ADDA,r0 abs,x+      ;r0 \+= \*(abs \+ \++x);                       ;4,3  
ADDA,r0 abs,x-      ;r0 \+= \*(abs \+ \--x);                       ;4,3  
ADDA,rn \*abs        ;rn \+= \*(\*(abs));                          ;6,3  
ADDI,rn imm         ;rn \+= imm;                                ;2,2  
ADDR,rn rel         ;rn \+= \*(rel);                             ;3,2  
ADDR,rn \*rel        ;rn \+= \*(\*(rel));                          ;5,2  
ADDZ,rn             ;r0 \+= rn;                                 ;2,1

SUBA,rn abs         ;rn \-= \*(abs);                             ;4,3  
SUBA,r0 abs,x       ;r0 \-= \*(abs \+ x);                         ;4,3  
SUBA,r0 abs,x+      ;r0 \-= \*(abs \+ \++x);                       ;4,3  
SUBA,r0 abs,x-      ;r0 \-= \*(abs \+ \--x);                       ;4,3  
SUBA,rn \*abs        ;rn \-= \*(\*(abs));                          ;6,3  
SUBI,rn imm         ;rn \-= imm;                                ;2,2  
SUBR,rn rel         ;rn \-= \*(rel);                             ;3,2  
SUBZ,rn             ;r0 \-= rn;                                 ;2,1

ANDA,rn abs         ;rn &= \*(abs);                             ;4,3  
ANDI,rn imm         ;rn &= imm;                                ;2,2  
ANDR,rn rel         ;rn &= \*(rel);                             ;3,2  
ANDZ,rn             ;r0 &= rn;                                 ;2,1  
; NOTE: ANDZ,R0 is NOT a valid instruction — encodes as HALT ($40).

IORA,rn abs         ;rn |= \*(abs);                             ;4,3  
IORI,rn imm         ;rn |= imm;                                ;2,2  
IORR,rn rel         ;rn |= \*(rel);                             ;3,2  
IORZ,rn             ;r0 |= rn;                                 ;2,1

EORA,rn abs         ;rn ^= \*(abs);                             ;4,3  
EORI,rn imm         ;rn ^= imm;                                ;2,2  
EORR,rn rel         ;rn ^= \*(rel);                             ;3,2  
EORZ,rn             ;r0 ^= rn;                                 ;2,1

### **Load, Store & Compare**

LODA,rn abs         ;rn \= \*(abs);                              ;4,3  
LODI,rn imm         ;rn \= imm;                                 ;2,2  
LODR,rn rel         ;rn \= \*(rel);                              ;3,2  
LODZ,rn             ;r0 \= rn;    (destination is ALWAYS r0)   ;2,1  
; NOTE: LODZ,R0 result is undefined per 2650 manual.

LODA,r0 abs,x       ;r0 \= \*(abs \+ x);    x unchanged           ;4,3  
LODA,r0 abs,x+      ;x++; r0 \= \*(abs \+ x);  pre-increment      ;4,3  
LODA,r0 abs,x-      ;x--; r0 \= \*(abs \+ x);  pre-decrement      ;4,3  
LODA,rn \*abs        ;rn \= \*(\*(abs));     indirect              ;6,3  
LODA,r0 \*abs,x      ;r0 \= \*(\*(abs) \+ x);                       ;6,3

STRA,rn abs         ;\*(abs) \= rn;         CC NOT modified      ;4,3  
STRR,rn rel         ;\*(rel) \= rn;         CC NOT modified      ;3,2  
STRA,r0 abs,x       ;\*(abs \+ x) \= r0;     CC NOT modified      ;4,3  
STRA,r0 abs,x+      ;x++; \*(abs \+ x) \= r0; CC NOT modified     ;4,3  
STRA,r0 abs,x-      ;x--; \*(abs \+ x) \= r0; CC NOT modified     ;4,3  
STRA,rn \*abs        ;\*(\*(abs)) \= rn;      CC NOT modified      ;6,3  
STRZ,rn             ;rn \= r0;   (source is ALWAYS r0)          ;2,1

COMA,rn abs         ;rn : \*(abs) \-\> sets CC;                   ;4,3  
COMI,rn imm         ;rn : imm   \-\> sets CC;                    ;2,2  
COMR,rn rel         ;rn : \*(rel) \-\> sets CC;                   ;3,2  
COMZ,rn             ;r0 : rn    \-\> sets CC;                    ;2,1  
COMA,r0 abs,x       ;r0 : \*(abs+x) \-\> sets CC;                 ;4,3

**KEY:** In indexed absolute mode the register field in the opcode byte encodes the **index register**, NOT the destination. Destination is always R0. LODA,R0 BASE,R2 emits opcode $0E (R2 in field), not $0C (R0). Confirmed by asm2650.py and WinArcadia.

### **Branching & Subroutine Calls**

BCTA,cond abs       ;if(cond) PC \= abs;                        ;3,3  
BCTA,cond \*abs      ;if(cond) PC \= \*abs;                       ;3,3  
BCTR,cond rel       ;if(cond) PC \+= rel;                       ;3,2  
BCFA,cond abs       ;if(\!cond) PC \= abs;                       ;3,3  
BCFA,cond \*abs      ;if(\!cond) PC \= \*abs;                      ;3,3  
BCFR,cond rel       ;if(\!cond) PC \+= rel;                      ;3,2

BSTA,cond abs       ;if(cond) { push(PC); PC \= abs; }          ;3,3  
BSTA,cond \*abs      ;if(cond) { push(PC); PC \= \*abs; }         ;3,3  
BSTR,cond rel       ;if(cond) { push(PC); PC \+= rel; }         ;3,2  
BSFA,cond abs       ;if(\!cond){ push(PC); PC \= abs; }          ;3,3  
BSFA,cond \*abs      ;if(\!cond){ push(PC); PC \= \*abs; }         ;3,3  
BSFR,cond rel       ;if(\!cond){ push(PC); PC \+= rel; }         ;3,2  
RETC,cond           ;if(cond) PC \= pop();                      ;3,1  
RETE,cond           ;if(cond) { PC \= pop(); enable ints; }     ;3,1

ZBRR    offset      ;PC \= page\_base \+/- sign\_extend\_7bit (offset))          ;2,2  
ZBRR    \*offset     ;PC \= \*(page\_base \+/- sign\_extend\_7bit (offset))          ;2,2  
ZBSR    offset      ;push(PC); PC \= page\_base \+/- sign\_extend\_7bit(offset) ;3,2  
ZBSR    \*offset     ;push(PC); PC \= \*(page\_base \+/- sign\_extend\_7bit(offset)) ;3,2

BXA     abs,r3       ;goto abs \+ r3;                          ;3,3  
BXA     \*abs,r3      ;goto \*(abs) \+ r3;                       ;5,3  
BSXA    abs,r3       ;gosub abs \+ r3;                         ;3,3  
BSXA    \*abs,r3      ;gosub \*(abs) \+ r3;                      ;5,3

## **4\. Register Branch Instructions (CRITICAL)**

These instructions branch based directly on a register value. They are **not** based on the CC flags.

### **BRNR / BRNA — Branch if Register Non-Zero**

BRNR,rn rel         ;if (rn \!= 0\) PC \+= rel;                   ;3,2  
BRNA,rn abs         ;if (rn \!= 0\) PC \= abs;                    ;3,3  
BRNA,rn \*abs        ;if (rn \!= 0\) PC \= \*(abs);                 ;5,3

**CRITICAL:** Register rn is **TESTED BUT NOT MODIFIED**. This is a pure conditional branch. To use as a counted loop you MUST decrement separately:

        LODI,R1 5  
LOOP:  
        ; body  
        SUBI,R1 1       ; explicit decrement (2 bytes)  
        BRNR,R1 LOOP    ; branch if R1 \!= 0 (2 bytes) — total 4 bytes per loop-close

### **BIRR / BIRA — Branch and Increment Register**

BIRR,rn rel         ;rn++; if (rn \!= 0\) PC \+= rel;             ;3,2  
BIRA,rn abs         ;rn++; if (rn \!= 0\) PC \= abs;              ;3,3  
BIRA,rn \*abs        ;rn++; if (rn \!= 0\) PC \= \*abs;             ;5,3

### **BDRR / BDRA — Branch and Decrement Register**

BDRR,rn rel         ;rn--; if (rn \>= 0\) PC \+= rel;             ;3,2  
BDRA,rn abs         ;rn--; if (rn \>= 0\) PC \= abs;              ;3,3  
BDRA,rn \*abs        ;rn--; if (rn \>= 0\) PC \= \*abs;             ;5,3

Register IS decremented before SIGNED test. **Practical behavior:** Loop continues while rn \!= $FF. Loop exits when rn underflows from $00 to $FF (-1 signed). This is the most compact loop instruction — just 2 bytes, decrement is free:

        LODI,R1 4       ; will take 5 iterations: 4-\>3-\>2-\>1-\>0-\>FF(exits)  
LOOP:  
        ; body  
        BDRR,R1 LOOP    ; R1--, branch if R1 \>= 0 signed (2 bytes per loop-close)

## **5\. System, Status & I/O**

SPSU / SPSL         ;r0 \= PSU / PSL;  (Store PSW to r0)        ;2,1  
LPSU / LPSL         ;PSU / PSL \= r0;  (Load PSW from r0)       ;2,1  
CPSU / CPSL imm     ;PSU/PSL &= \~imm; (Clear bits)             ;3,2  
PPSU / PPSL imm     ;PSU/PSL |=  imm; (Preset/Set bits)        ;3,2  
TPSU / TPSL imm     ;test: CC=EQ if all bits set, else CC=LT   ;3,2

REDD,rn             ;rn \= Data  input port;                    ;3,1  
REDC,rn             ;rn \= Control input port;                  ;3,1  
REDE,rn             ;rn \= Extended input (SENSE-pin serial);   ;2,1  
WRTD,rn             ;Data  output port \= rn;                   ;3,1  
WRTC,rn             ;Control output \= rn;                      ;3,1  
WRTE,rn             ;Extended output (FLAG-pin serial);        ;2,1

HALT                ;Stop CPU;         opcode $40             ;1,1  
NOP                 ;No Operation;     opcode $C0             ;2,1  
DAR,rn              ;Decimal Adjust rn (BCD correction);       ;3,1  
RRR,rn              ;Rotate Right rn (circular or w/carry);    ;2,1  
RRL,rn              ;Rotate Left  rn (circular or w/carry);    ;2,1  
TMI,rn imm          ;Test mask: CC=EQ if (rn\&imm)==imm, else LT ;3,2

### **PSL Bit Layout**

PSL: CC1\[7\] CC0\[6\] IDC\[5\] RS\[4\] WC\[3\] OVF\[2\] COM\[1\] C\[0\]

* **CC** \= bits 7:6 — condition code  
* **IDC** \= bit 5 — inter-digit carry (BCD)  
* **RS** \= bit 4 — register select (bank switching, R1-R3 only; R0 always bank 0\)  
* **WC** \= bit 3 — with carry (enables carry-in for ADD/SUB)  
* **OVF** \= bit 2 — overflow  
* **COM** \= bit 1 — compare mode (0=signed, 1=unsigned)  
* **C** \= bit 0 — carry / borrow

⚠️ **CRITICAL PITFALL:** The COM status bit exclusively impacts the execution of explicit comparison instructions (COMA, COMR, COMI, COMZ). It has **no effect** on how standard math (ADD, SUB) or register-branching instructions (BDRR, BRNR) evaluate signed vs. unsigned values. BDRR and BDRA always perform a signed arithmetic test (rn \>= 0), regardless of the COM bit state.

### **Multi-Byte Arithmetic Control (WC alternative)**

Instead of evaluating the carry/borrow flags explicitly via branching loops, multi-byte math routines can toggle the **WC bit (With Carry, PSL bit 3\)**. When WC \= 1, standard ADD and SUB operations automatically inject the carry/borrow bit value into the computation.

        ; Canonical 16-bit Unsigned Addition using the WC bit  
        CPSL $08          ; Clear WC bit (ensure normal 8-bit add for low byte)  
        LODA,R0 VAL1\_LO  
        ADDA,R0 VAL2\_LO  
        STRA,R0 RES\_LO    ; PSL\_C is set/cleared by this operation  
        PPSL $08          ; Set WC bit (enables Carry-In for the next addition)  
        LODA,R0 VAL1\_HI  
        ADDA,R0 VAL2\_HI   ; Performs: R0 \= VAL1\_HI \+ VAL2\_HI \+ Carry  
        STRA,R0 RES\_HI  
        CPSL $08          ; Clear WC bit to restore standard 8-bit math mode

## **6\. Hardware Constraints & Environment**

| Instruction | Opcode | Issue | Assembler Behaviour   |
| :---- | :---- | :---- | :---- |
| LODZ,R0 | \`$00\` | R0=R0 self-load, result undefined | Warns, emits \`$00\` |
| ANDZ,R0 | \`$40\` | Encodes as HALT | Warns, emits \`$40\` (HALT) |
| STRZ,R0 | \`$C0\` | Encodes as NOP | Warns, emits \`$C0\` (NOP) |

Use IORZ,R0 (R0 |= R0, no-op) or EORZ,R0 (R0 ^= R0 → clears R0) instead of LODZ,R0.

### **PIPBUG 1 Environment (WinArcadia)**

COUT  $02B4   BSTA,UN $02B4   R0 → terminal         confirmed working  
CHIN  $0286   BSTA,UN $0286   R0 ← terminal         non-blocking in WinArcadia  
CRLF  $008A   BSTA,UN $008A   print CR+LF

User code: ORG $0C00, run via G 0C00. PIPBUG ROM $0000-$03FF (read-only). First free RAM: $0440. PIPBUG RAM $0400-$043F (reserved). ZBSR/ZBRR target range $0000-$007F is entirely within PIPBUG ROM — unusable under PIPBUG.

## **7\. Version History**

### **Changes v1.4 \-\> v1.5 Updated: 2026-05-21**

* **FIXED Placeholder Code Bug:** Corrected BCx,LT skip placeholder within the 16-bit increment idiom code-block to run the explicit BCTR,LT skip command.  
* **UPDATED CC Semantics Table:** Directly structured the canonical "CC Producer Semantics" table into the root documentation; integrated TMI into the Bit-Test operational grouping.  
* **ADDED Multi-Byte Math Controls:** Documented code-density optimization routines utilizing the WC (With Carry) status bit configuration to execute multi-byte arithmetic without pipeline branch penalties.  
* **ADDED Operational Pitfall Warns:** Appended architectural alerts identifying that the COM bit state exclusively influences COMx execution paths and has no control footprint over standard arithmetic or BDRR loops.  
* **ADDED Pointer Allocation Boundaries:** Established a hardware design rule outlining the constraint of address resolution loops if target pointer high-bytes violate 15-bit boundary limits (Bit 7 / Bit 15 set).

### **Changes v1.3 \-\> v1.4 Updated: 2026-05-21**

* Clarified that CC semantics depend on instruction class, not a universal meaning.  
* Added dedicated CC semantics breakdown profiles for arithmetic/load, compare, and bit-test (TPSL/TPSU/TMI).  
* Added "Core Conditional Execution Model" conceptual anchor definitions.  
* Clarified that BCFR/BSFR do NOT process separate hardware configurations, but rather evaluate inverse logic states (CC \!= v).  
* Clarified that TPSL/TPSU/TMI do not emit GT configurations under any execution environment.  
* Expanded subtraction carry/borrow wording to denote C=1 means NO borrow, C=0 means borrow.  
* Improved BDRR loop tracking definitions to model actual machine-state exits (00 \-\> FF).  
* Expanded STRZ hazard descriptions regarding unintentional destruction of prior arithmetic states via the internal ALU bus write.
