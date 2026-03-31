# asm2650.c correctness audit (against Signetics 2650 user manual)

Manual reference: <https://amigan.yatho.com/2650UM.html>

## Scope
This audit checks whether `asm2650.c` encodes instruction forms consistent with the 2650 manual.

## Implemented policy checks in assembler

The assembler now enforces/normalizes several non-orthogonal encodings discussed during review:

- `BXA`/`BSXA` (`BSX` alias accepted) now validate the optional register operand and enforce **R3 only**.
- Explicit `R0`/`R1`/`R2` with `BXA`/`BSXA` now produce an error.
- Auto increment/decrement suffixes on the BXA/BSXA register operand (e.g. `R3+`, `R3-`) now produce an error.
- If no BXA/BSXA register is specified, assembler warns and defaults to `R3`.
- `LODZ R0` now warns and is emitted as `$60` (`IORZ,R0`).
- `STRZ R0` now warns and is emitted as `NOP` (`$C0`).
- `ANDZ R0` now warns and is emitted as `HALT` (`$40`).

## High-impact findings

### 1) `ZBRR` is encoded as a 1-byte instruction (missing operand byte)
**Status:** ❌ Incorrect

- Manual specifies `ZBRR (*)a` as a 2-byte form (opcode + 7-bit signed displacement with optional indirect bit in bit 7).
- Current assembler emits only opcode `0x9B` and returns immediately.

**Code evidence:** `if(strcmp(mn,"ZBRR")==0){ emit(pc,0x9B);pc++; return; }`.

**Behavior evidence:** assembling

```asm
 ORG $0000
 ZBRR 5
 ZBSR *-1
 END
```

produces `9B BB 7F` (3 bytes total), proving `ZBRR` consumed only one byte.

---

### 2) `ZBSR` ignores indirect (`*`) when encoding second byte
**Status:** ❌ Incorrect

- Manual allows indirect addressing for `ZBSR`; operand byte bit 7 must be set for indirect.
- Current implementation strips leading `*` but always emits `(v & 0x7F)`, never setting bit 7.
- Example `ZBSR *-1` should encode low 7 bits `0x7F` **with** indirect bit => `0xFF`; assembler emits `0x7F`.

**Code evidence:** `if(strcmp(mn,"ZBSR")==0){ ... if(*a=='*') a++; ... emit(pc,(unsigned char)(v&0x7F)); ... }`.

---

## Medium-impact findings

### 3) Source line beginning at column 0 is treated as a label candidate first
**Status:** ⚠️ Potentially problematic syntax behavior

- Any non-whitespace first token is parsed as a label name before mnemonic detection.
- This means plain left-aligned instructions like `ORG $0000` are misparsed unless indented.
- In test runs this caused `unknown mnemonic ''`/`'5'` errors until instructions were prefixed with a leading space.

**Code evidence:** label parsing gate `if(!isspace((unsigned char)buf[0])&&buf[0]){ ... if(pass==1) label_define(lbl,pc); }`.

---

## Areas that look reasonable

- Relative branch families (`BCTR/BCFR/BSTR/BSFR/BRNR/BIRR/BDRR/BSNR`) route through shared tables and emit relative vs absolute forms consistently.
- Absolute operand packing helper `emit_abs()` places indirect in bit 7 and keeps two selector bits in bits 6..5, matching 2650 encoded format conventions.
- ALU family table (`LOD/EOR/AND/IOR/ADD/SUB/COM/STR`) has sensible mode mapping and blocks invalid `STRI`.

## Recommended fixes

1. Rework `ZBRR` to parse operand and always emit second byte with signed 7-bit displacement and optional indirect bit.
2. Fix `ZBSR` to preserve/encode `*` into bit 7 of the operand byte.
3. Decide whether column-0 mnemonic parsing should be supported; if yes, adjust label parsing so normal instruction lines do not require indentation.
