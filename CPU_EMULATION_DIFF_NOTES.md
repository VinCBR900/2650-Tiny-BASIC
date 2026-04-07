# 2650 CPU emulation comparison: `sim2650.c` vs `2650.c`

This note compares CPU-core behavior only (not UI/framework/machine integrations).

## Scope and method

- Reviewed opcode handlers and core helpers in:
  - `sim2650.c`
  - `2650.c` (winarcadia-derived core in this repo)
- Focused on instruction semantics, flag behavior, effective-address generation, and control-flow behavior.

## Summary

`sim2650.c` is close to `2650.c` for most integer ALU operations, conditional branching families, page-relative/absolute effective address generation, and carry/borrow/IDC semantics in ADD/SUB.

I found **five meaningful CPU-emulation differences** that should be decided explicitly if exact behavioral parity with `2650.c` is the goal.

---

## 1) HALT semantics differ

- `sim2650.c`: opcode `0x40` stops execution immediately (`running = 0`).
- `2650.c`: HALT is treated like a normal instruction timing-wise and execution continues; it logs/flags but does not terminate the CPU loop.

Impact:
- Programs that rely on HALT as a non-fatal state (interrupt wakeup model, monitor behavior, or timing loops) may terminate early under `sim2650.c`.

Evidence:
- `sim2650.c` HALT early stop. 
- `2650.c` HALT does `ZERO_BYTES; REG_CYCLES;` and falls through to next instruction flow.

---

## 2) DAR updates CC in `2650.c`, but not in `sim2650.c`

- `sim2650.c`: DAR updates register value but **explicitly avoids CC updates**.
- `2650.c`: DAR uses `WRITEREGCC(...)`, therefore CC is updated after each decimal-adjust step.

Impact:
- Any code branching on CC immediately after DAR may diverge between simulators.

---

## 3) STRZ condition-code behavior differs

- `sim2650.c`: STR group mode-0 (`STRZ`) copies `R0` into `Rn` and returns with no CC update.
- `2650.c`: STRZ path uses `WRITEREGCC(...)`, which updates CC from destination register.

Impact:
- Code that inspects CC after STRZ can diverge.

---

## 4) Undefined opcodes 0x90/0x91 behavior differs

- `2650.c`: explicitly handles `0x90` and `0x91` as undefined-but-consumed 1-byte instructions (does not halt emulator).
- `sim2650.c`: these end up in "unhandled opcode" and set fault/halt.

Impact:
- Binaries containing these bytes (intentionally or as data-flow side effects) may continue in `2650.c` but trap in `sim2650.c`.

---

## 5) RRL/RRR overflow-bit edge behavior in WC mode is not identical

- Both implementations update C/IDC/CC similarly in WC mode.
- However, OVF condition checks are not the same:
  - `sim2650.c` uses a generic sign-bit-change test for both RRR and RRL.
  - `2650.c` uses narrower conditions (especially for RRL), with additional value-range constraints.

Impact:
- PSL.OVF may differ for specific rotate-through-carry patterns.

---

## Areas that look aligned

- Relative addressing wraps within current 8K page; absolute non-branch A-mode uses current page base + 13-bit offset.
- Indirect fetch wraps second byte at page end.
- BRNR/BRNA test-only behavior (no register decrement) is aligned.
- ADD/SUB C and IDC conventions are aligned with `2650.c` helper logic.
- COM signed/unsigned compare behavior follows PSL.COM in both cores.

---

## Recommendation for parity mode

If you want strict `sim2650.c` ≈ `2650.c` CPU parity, prioritize:

1. Make HALT non-terminating (or gate with a simulator option).
2. Update DAR to modify CC identically to `2650.c`.
3. Update STRZ to set CC.
4. Add explicit 0x90/0x91 compatibility handling.
5. Match RRL/RRR OVF formulas exactly to `2650.c`.

