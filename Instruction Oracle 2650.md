# Signetics 2650 Instruction Set Oracle

## 1. Definitions & Addressing Modes

### **Registers & Values**
* **rn**: General Purpose Register (r0, r1, r2, r3). Note: **r0** is the implicit source/destination for indexed operations and all **Z-group e.g ADDZ, LODZ** instructions.
* **abs**: 15-bit Absolute Address ($0000 - $7FFF).
* **rel**: 7-bit signed displacement (-64 to +63). The origin is the address of the byte *immediately following* the instruction.
* **imm**: 8-bit Immediate value.
* **x**: Index Register (r1, r2, r3).
* **+ / -**: Auto-increment or auto-decrement of the index register.
* **Zxxx**: Zero Page Branch instructions only access first 63 bytes of memory
  
### **The Indirect Flag (*)**
* The asterisk denotes **Indirect Addressing**. This sets Bit 7 of the address/displacement byte. The CPU fetches a 15-bit pointer from the calculated effective address and uses that as the final target.

### **Condition Codes (CC)**
The Condition Code (CC) is a 2-bit field in the PSL (Program Status Lower) updated after most arithmetic/load operations:
* **00 (Zero/Equal)**: Result is zero or comparison was equal.
* **01 (Positive/Greater)**: Result is positive (Bit 7=0, not zero) or Reg > Memory.
* **10 (Negative/Less)**: Result is negative (Bit 7=1) or Reg < Memory.
* **11 (Unconditional/UN)**: Used in branch fields to ignore CC status.

### **Branch Masks (cond)**
Used in `BCT`, `BCF`, `BSR`, and `RET` instructions:
* **EQ / Z**: Zero (Mask %00)
* **GT / P**: Positive (Mask %01)
* **LT / N**: Negative (Mask %10)
* **UN**: Unconditional (Mask %11)

### **Hardware Stack**
Unfortunately a dedicated HW subroutine stack is implemented which is only 8 deep before wrapping.  PIPBUG uses 3 slots so care needed.

### **Timing & Size Notation**
* **(Cycles, Bytes)**: e.g., `;4,3` = 4 CPU cycles, 3 bytes of memory.

---



## 2. Arithmetic & Logical Instructions

### **ADD (Addition)**
```asm
ADDA,rn abs         ;rn += *(abs);                             ;4,3
ADDA,r0 abs,x       ;r0 += *(abs + x);                         ;4,3
ADDA,r0 abs,x+      ;r0 += *(abs + x++);                       ;4,3
ADDA,r0 abs,x-      ;r0 += *(abs + x--);                       ;4,3
ADDA,rn *abs        ;rn += *(*(abs));                          ;6,3
ADDI,rn imm         ;rn += imm;                                ;2,2
ADDR,rn rel         ;rn += *(rel);                             ;3,2
ADDR,rn *rel        ;rn += *(*(rel));                          ;5,2
ADDZ,rn             ;r0 += rn;                                 ;2,1
```
### **SUB (Subtraction)**
```asm
SUBA,rn abs         ;rn -= *(abs);                             ;4,3
SUBI,rn imm         ;rn -= imm;                                ;2,2
SUBR,rn rel         ;rn -= *(rel);                             ;3,2
SUBA,rn abs          ;rn -= *(abs);                           ;4,3
SUBA,r0 abs,x       ;r0 -= *(abs + x);                      ;4,3
SUBA,r0 abs,x+      ;r0 -= *(abs + ++x);                    ;4,3
                         ;or, x++; r0 -= *(abs + x);
SUBA,r0 abs,x-      ;r0 -= *(abs + --x);                    ;4,3
                         ;or, x--; r0 -= *(abs + x);
SUBA,rn *abs         ;rn -= *(*(abs));                        ;6,3
SUBA,r0 *abs,x      ;r0 -= *(*(abs) + x);                   ;6,3
SUBA,r0 *abs,x+     ;r0 -= *(*(abs) + ++x);                 ;6,3
                         ;or, x++; r0 -= *(*(abs) + x);
SUBA,r0 *abs,x-     ;r0 -= *(*(abs) + --x);                 ;6,3
                         ;or, x--; r0 -= *(*(abs) + x);
SUBZ,rn             ;r0 -= rn;                                 ;2,1
```
### **AND / IOR / EOR (Bitwise)**
```asm
ANDA,rn abs         ;rn &= *(abs);                             ;4,3
ANDI,rn imm         ;rn &= imm;                                ;2,2
ANDR,rn rel         ;rn &= *(rel);                             ;3,2
ANDZ,rn             ;r0 &= rn;                                 ;2,1

IORA,rn abs         ;rn |= *(abs);                             ;4,3
IORI,rn imm         ;rn |= imm;                                ;2,2
IORR,rn rel         ;rn |= *(rel);                             ;3,2
IORZ,rn             ;r0 |= rn;                                 ;2,1

EORA,rn abs         ;rn ^= *(abs);                             ;4,3
EORI,rn imm         ;rn ^= imm;                                ;2,2
EORR,rn rel         ;rn ^= *(rel);                             ;3,2
EORZ,rn             ;r0 ^= rn;                                 ;2,1
```

### **Load, Store & Compare**
**LOD (Load)**
```asm
LODA,rn abs         ;rn = *(abs);                              ;4,3
LODI,rn imm         ;rn = imm;                                 ;2,2
LODR,rn rel         ;rn = *(rel);                              ;3,2
LODZ,rn             ;r0 = rn;                                  ;2,1
LODA,rn abs          ;rn = *(abs);                            ;4,3
LODA,r0 abs,x       ;r0 = *(abs + x);                       ;4,3
LODA,r0 abs,x+      ;r0 = *(abs + ++rn);                     ;4,3
                         ;or, x++; r0 = *(abs + x);
LODA,r0 abs,x-      ;r0 = *(abs + --rn);                     ;4,3
                         ;or, x--; r0 = *(abs + x);
LODA,rn *abs        ;x = *(*(abs));                         ;6,3
LODA,r0 *abs,x      ;r0 = *(*(abs) + x);                    ;6,3
LODA,r0 *abs,x+     ;r0 = *(*(abs) + ++x);                  ;6,3
                        ;or, x++; r0 = *(*(abs) + x);
LODA,r0 *abs,x-     ;r0 = *(*(abs) + --x);                  ;6,3
                         ;or, x--; r0 = *(*(abs) + x);
```
**STR (Store)**
```asm
STRA,rn abs         ;*(abs) = rn;                              ;4,3
STRR,rn rel         ;*(rel) = rn;                              ;3,2
STRA,rn abs          ;*(abs) = rn;                            ;4,3
STRA,r0 abs,x       ;*(abs + x) = r0;                       ;4,3
STRA,r0 abs,x+      ;*(abs + ++x) = r0;                     ;4,3
                         ;or, x++; *(abs + x) = r0;
STRA,r0 abs,x-      ;*(abs + --x) = r0;                     ;4,3
                         ;or, x--; *(abs + x) = r0;
STRA,rn *abs         ;*(*(abs)) = rn;                         ;6,3
STRA,r0 *abs,x      ;*(*(abs) + x) = r0;                    ;6,3
STRA,r0 *abs,x+     ;*(*(abs) + ++x) = r0;                  ;6,3
                         ;or, x++; *(*(abs) + x) = r0;
STRA,r0 *abs,x-     ;*(*(abs) + --x) = r0;                  ;6,3
                         ;or, x--; *(*(abs) + x) = r0;
STRZ,x             ;rn = r0;                                  ;2,1
```
**COM (Compare)**
```asm
COMA,rn abs         ;rn : *(abs) -> Sets CC;                   ;4,3
COMI,rn imm         ;rn : imm -> Sets CC;                      ;2,2
COMR,rn rel         ;rn : *(rel) -> Sets CC;                   ;3,2
COMA,rn abs          ;compare rn against *(abs);              ;4,3
COMA,r0 abs,x       ;compare r0 against *(abs + x);         ;4,3
COMA,r0 abs,x+      ;compare r0 against *(abs + ++x);       ;4,3
                         ;or, x++; compare r0 against *(abs + x);
COMA,r0 abs,x-      ;compare r0 against *(abs + --x);       ;4,3
                         ;or, x--; compare r0 against *(abs + x);
COMA,rn *abs         ;compare rn against *(*(abs));           ;6,3
COMA,r0 *abs,x      ;compare r0 against *(*(abs) + x);      ;6,3
COMA,r0 *abs,x+     ;compare r0 against *(*(abs) + ++x);    ;6,3
                         ;or, x++; compare r0 against *(*(abs) + x);
COMA,r0 *abs,x-     ;compare r0 against *(*(abs) + --x);    ;6,3
                         ;or, x--; compare r0 against *(*(abs) + x);
COMZ,rn             ;r0 : rn -> Sets CC;                       ;2,1
```
### **4. Branching & Subroutine Calls**
**Conditional Branching**
```asm
BCTA,cond abs       ;if(cond) PC = abs;                        ;3,3
BCTA,cond *abs       ;if(cond) PC = *abs;                        ;3,3
BCTR,cond rel       ;if(cond) PC = rel;                        ;3,2
BCFA,cond abs       ;if(!cond) PC = abs;                       ;3,3
BCFA,cond *abs       ;if(!cond) PC = *abs;                       ;3,3
BCFR,cond rel       ;if(!cond) PC = rel;                       ;3,2
```
**Subroutine Calls**
```asm
BSTA,cond abs       ;if(cond) {Push(PC+3); PC = abs;};         ;3,3
BSTA,cond *abs       ;if(cond) {Push(PC+3); PC = *abs;};         ;3,3
BSTR,cond rel       ;if(cond) {Push(PC+2); PC = rel;};         ;3,2
```
**Special & Zero Page Branches**
```asm
BSXA    abs,r3       ;gosub abs + r3;                         ;3,3
BSXA    *abs,r3      ;gosub *(abs) + r3;                      ;5,3
BXA     abs,r3       ;goto abs + r3;                          ;3,3
BXA     *abs,r3      ;goto *(abs) + r3;                       ;5,3
ZBRR    zero         ;goto zero;                              ;3,2
ZBRR    *zero        ;goto *(zero);                           ;5,2
ZBSR    zero         ;gosub zero;                             ;3,2
ZBSR    *zero        ;gosub *(zero);                          ;5,2
*Note: ZBSR *$0001 effectively performs JSR ($0001).
```
**Increment/Decrement Branches**
```asm
BIRA,rn abs         ;rn++; if(rn != 0) PC = abs;               ;3,3
BIRA,rn *abs         ;rn++; if(rn != 0) PC = *abs;               ;3,3
BIRR,rn rel         ;rn++; if(rn != 0) PC = rel;               ;3,2
BDRA,rn abs         ;rn--; if(rn != 0) PC = abs;               ;3,3
BDRA,rn *abs         ;rn--; if(rn != 0) PC = *abs;               ;3,3
BDRR,rn rel         ;rn--; if(rn != 0) PC = rel;               ;3,2
```
**Return Instructions**
```asm
RETC,cond           ;if(cond) PC = Pop();                      ;3,1
RETE,cond           ;if(cond) {PC = Pop(); Enable Ints;};      ;3,1
```
### **5. System, Status & I/O**
**PSW Management**
```asm
LPSU / LPSL         ;Load PSU/PSL from r0;                     ;2,1
SPSU / SPSL         ;Store PSU/PSL to r0;                      ;2,1
CPSU / CPSL imm     ;Status &= ~imm; (Clear bits);             ;3,2
PPSU / PPSL imm     ;Status |= imm; (Set bits);                ;3,2
TPSU / TPSL imm     ;Test Status bits with imm;                ;3,2
```
**I/O Operations**
```asm
REDD,rn             ;rn = Input(Data Port);                    ;3,1
REDC,rn             ;rn = Input(Control Port);                 ;3,1
WRTD,rn             ;Output(Data Port) = rn;                   ;3,1
WRTC,rn             ;Output(Control Port) = rn;                ;3,1
```
**Miscellaneous**
```asm
HALT                ;Stop CPU;                                 ;0,1
NOP                 ;No Operation;                             ;2,1
DAR,rn              ;Decimal Adjust rn;                        ;3,1
RRR,rn              ;Rotate Right rn (Circular or with Carry); ;2,1
RRL,rn              ;Rotate Left rn (Circular or with Carry);  ;2,1
```
This document is designed to be a "cold start" reference. 
