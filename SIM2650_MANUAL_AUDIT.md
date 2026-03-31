# sim2650.c vs Signetics 2650 User Manual audit

Source manual reviewed: <https://amigan.yatho.com/2650UM.html>.

## Scope
This audit checks behavioral consistency of `sim2650.c` against documented 2650 instruction semantics (especially branch, status-bit, and ALU/rotate side-effects).

## Findings

### 1) `ZBRR` (opcode `0x9B`) is implemented as return-from-subroutine
**Status:** ❌ Inconsistent

- Manual: `ZBRR` is a **zero-page relative branch** with optional indirection; it is not a return.
- Simulator currently executes `cpu.IAR = pop_ras();` for opcode `0x9B`, which is return semantics.

**Evidence in code:** `op==0x9B` block in `execute()`.

---

### 2) `ZBSR` (opcode `0xBB`) target computation is not zero-page and ignores indirect bit
**Status:** ❌ Inconsistent

- Manual: displacement is relative to **page zero, byte zero** (modulo 8192), and indirect form is valid.
- Simulator computes target from `(cpu.IAR & 0x6000) + off`, effectively using current page latches, and does not apply indirect resolution for bit7.

**Evidence in code:** `op==0xBB` block in `execute()`.

---

### 3) `BSNR/BSNA` opcodes (`0x78-0x7F`) implement wrong condition and wrong control-flow class
**Status:** ❌ Inconsistent

- Manual: BSNR/BSNA = **Branch to Subroutine on Register Non-Zero** (push return address when branch taken), optional indirect.
- Simulator uses PSU sense-bit comparison (`PSU_S` vs `R(rn)&1`) and does **not** push RAS; this is not BSNR/BSNA behavior.

**Evidence in code:** two `if(op>=0x78&&op<=0x7B)` and `if(op>=0x7C&&op<=0x7F)` blocks.

---

### 4) `BRNR/BIRR/BDRR` relative forms ignore indirect addressing flag
**Status:** ❌ Inconsistent

- Manual: these relative branch families accept `(*)a` (indirect allowed).
- Simulator parses indirect bit (`ind`) but never resolves through indirect pointer for these opcodes.

**Evidence in code:** `0x58-0x5B`, `0xD8-0xDB`, `0xF8-0xFB` handlers.

---

### 5) `BDRR/BDRA` branch condition implemented as signed `>= 0` instead of `!= 0`
**Status:** ❌ Inconsistent

- Manual: decrement register, branch if the new register value is **non-zero**.
- Simulator branches on `(signed char)R(rn) >= 0` for relative and absolute forms.

**Evidence in code:** `0xF8-0xFB` and `0xFC-0xFF` handlers.

---

### 6) `BRNR/BIRR/BDRR` incorrectly modify CC
**Status:** ❌ Inconsistent

- Manual: processor registers affected = **None** for these families; CC not modified.
- Simulator calls `set_cc(R(rn))` after increment/decrement operations.

**Evidence in code:** same `BRNR/BIRR/BDRR/BDRA` blocks.

---

### 7) Rotate instructions with `WC=1` do not update `OVF` and `IDC`
**Status:** ❌ Inconsistent

- Manual: with `WC=1`, rotate affects `C`, `OVF`, and `IDC`; OVF tracks sign-bit change and IDC gets new bit5.
- Simulator updates carry and CC, but does not update `OVF` or `IDC` in RRR/RRL paths.

**Evidence in code:** `RRR` (`0x50-0x53`) and `RRL` (`0xD0-0xD3`) handlers.

---

### 8) `LPSU` handling of PSU bits is likely incorrect for `S`, and does not guarantee reserved bits stay zero
**Status:** ⚠️ Likely inconsistent

- Manual: LPSU affects `F`, `II`, and `SP`; bits 4 and 3 are unassigned and considered zero.
- Simulator computes `cpu.PSU = (R(0) & ~PSU_S)`:
  - forces `S` to 0 instead of preserving external sense input behavior,
  - allows bits 4 and 3 to be loaded from R0.

**Evidence in code:** `op==0x92` in `execute()`.

---

## Areas that appear consistent

- RS register banking mechanism (`R1..R3` vs `R1'..R3'`) is modeled via `ri()` and `PSL_RS`.
- `ADD/SUB` carry/borrow-oriented CC mapping helper functions (`set_cc_add`, `set_cc_sub`) align with documented C/borrow interpretation.
- `LPSL`/`SPSL`/`SPSU` instruction family basic data movement behavior appears plausible.

## Recommended next actions

1. Correct `ZBRR` and `ZBSR` first (high-impact control-flow correctness).
2. Re-implement `BSNR/BSNA` and relative register-branch families with proper indirect handling.
3. Fix `BDRR/BDRA` condition and remove CC writes for BRNR/BIRR/BDRR families.
4. Implement rotate side-effects for `OVF` and `IDC` when `WC=1`.
5. Refine LPSU masking to match documented writable/observable PSU bits.

