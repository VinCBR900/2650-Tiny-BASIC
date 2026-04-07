# sim2650.c vs WinArcadia/MAME 2650 core (s2650.cpp) comparison

Date: 2026-04-07

Compared local `sim2650.c` against the MAME/WinArcadia-style Signetics 2650 core logic in:
- https://github.com/mamedev/mame/blob/master/src/devices/cpu/s2650/s2650.cpp

## High-impact behavioral differences

1. **Relative effective address page handling differs**
   - Reference core `_REL_EA(page)` keeps relative EA within current 8K page via:
     `m_ea = page + ((m_iar + sext(hr,7)) & PMSK)`.
   - `sim2650.c` computes relative EA with full 15-bit add:
     `eff = (cpu.IAR + off) & 0x7FFF`.
   - Impact: relative branches/loads/stores can cross page boundaries in `sim2650` when reference core wraps in-page.

2. **Non-branch absolute addressing base page differs**
   - Reference `_ABS_EA()` builds non-branch absolute addresses as:
     `m_page + (((hr<<8)+dr) & PMSK)`.
   - `sim2650.c` `fetch_abs_nb()` returns only the literal 13-bit address and never adds current page.
   - Impact: all non-branch absolute ops (LOD/STR/etc mode 3) are effectively forced to page 0 in `sim2650`.

3. **Indirect pointer second-byte fetch wrap behavior differs**
   - Reference core wraps indirect pointer high/low fetch at page end:
     `if ((++addr & PMSK) == 0) addr -= PLEN;`.
   - `sim2650.c` always reads second byte at `(base+1) & 0x7FFF`.
   - Impact: indirect vectors at page end will resolve differently.

4. **DAR semantics differ**
   - Reference macro `M_DAR(dest)` adjusts based on `C`/`IDC` clear conditions and does **not** set CC.
   - `sim2650.c` performs BCD nibble-threshold adjust (`>9` tests), then calls `set_cc(r)`.
   - Impact: BCD adjust results and CC state can diverge from WinArcadia behavior.

## Likely lower-impact / environment differences

5. **PSU bit model differs**
   - Reference PSU bit7 is `SI` (sense input), bit6 is `FO` (flag output).
   - `sim2650.c` models PSU bit7 as `S` and preserves it in LPSU/CPSU/PPSU logic.
   - Impact: if software depends on SI/FO behavior, results can differ.

6. **Interrupt model missing in sim2650**
   - Reference has IRQ line checks, vector handling, and REDE/II interactions.
   - `sim2650.c` is currently a standalone no-external-IRQ execution model.
   - Impact: interrupt-driven code will diverge.

## Direct places to inspect in local source

- Relative branch/EA macros usage and `fetch_rel` callers.
- `fetch_abs_nb()` and all mode-3 ALU/STR paths.
- `resolve()` indirect pointer implementation.
- `DAR` handler (`op 0x94..0x97`).

## Recommendation: port behavior, not codebase

Short answer: **incorporate `s2650.cpp` behavior into `sim2650.c`**, not a wholesale switch to `s2650.cpp`.

Why:
- `sim2650.c` is a compact, project-specific simulator with PIPBUG hooks and simple I/O flow already aligned to this repo.
- `s2650.cpp` is embedded in the MAME device framework (state save, debugger, IRQ plumbing, callbacks), so direct drop-in replacement is high effort.
- The divergence appears to be in a small set of semantic edge cases, so targeted parity fixes are lower risk and faster to validate.

Suggested migration strategy:
1. Keep `sim2650.c` architecture.
2. Patch only semantic mismatches first:
   - relative EA page semantics
   - absolute non-branch page base semantics
   - indirect pointer page-wrap semantics
   - DAR flag/CC semantics
3. Add focused micro-tests per opcode/addressing mode near page boundaries.
4. Re-run against WinArcadia traces and iterate.

When to consider adopting `s2650.cpp` wholesale:
- only if you need full MAME-level features (IRQ model fidelity, debugger integration, callbacks, save-state compatibility) and are willing to absorb significant refactor cost.
