# sim2650.c Compliance Review vs Signetics 2650 User Manual

Scope: static review of `sim2650.c` against the Signetics 2650 user manual at https://amigan.yatho.com/2650UM.html.

## Verdict

`sim2650.c` is **partially compliant**: many core decode/execute paths are present, but there are several architectural mismatches that will produce incorrect behavior for standards-accurate 2650 binaries.

## Major compliance issues

1. **Condition Code (CC) encoding is inverted for Zero vs Negative.**
   - Manual encoding is effectively: Positive=`01`, Zero=`00`, Negative=`10` (CC1:bit7, CC0:bit6).
   - Simulator constants are: `CC_POS=0x40`, `CC_ZERO=0x80`, `CC_NEG=0x00`, so Zero/Negative are swapped.
   - Impact: any instruction that sets or tests CC can branch incorrectly (e.g., EQ/LT behavior).

2. **`test_cc()` evaluates EQ/LT against the simulator’s swapped encodings.**
   - `COND_EQ` checks `cc == CC_ZERO` and `COND_LT` checks `cc == CC_NEG`.
   - With swapped constants, conditional branches/returns may invert equality vs less-than semantics.

3. **`TMI`, `TPSL`, and `TPSU` condition code results are not per manual.**
   - Manual behavior is binary: EQ if all selected bits are 1, else LT.
   - Simulator sets three-way values (POS/ZERO/NEG), which does not match the ISA-defined outcomes.

4. **IDC flag is not updated by ADD/SUB operations.**
   - Manual states IDC reflects carry/borrow out of bit 3 after add/sub.
   - `alu_add()` and `alu_sub()` update `C` and `OVF`, but do not compute/update IDC.
   - Impact: BCD workflows (`DAR`) and any code depending on IDC after arithmetic may misbehave.

5. **`CPSU`/`PPSU` can modify the PSU Sense bit (S), which should be immutable from software.**
   - Manual states S is tied to the sense pin and not affected by `LPSU`, `PPSU`, `CPSU`.
   - Simulator directly applies masks to the full `cpu.PSU`, so software can clear/set S.

## Areas that look correct or intentionally modeled

- **RS register banking model** (`ri()`) aligns with R0 fixed + banked R1..R3 design.
- **HALT/NOP opcodes** (`0x40`, `0xC0`) are correctly identified.
- **`LPSU` handling** preserves `S` and masks loaded bits to `F`, `II`, and `SP` fields.
- **ZBRR/ZBSR page-zero branch behavior** appears aligned with the “clear page bits #13/#14” concept.

## Overall assessment

For running this project’s Tiny BASIC + PIPBUG flows, the simulator may be “good enough” where code paths avoid edge-case PSW behavior. However, for **manual-accurate 2650 emulation**, the current implementation is **not yet compliant** due to CC/IDC/PSU-semantic defects that affect control flow and arithmetic correctness.

## Recommended fix order

1. Correct CC encoding constants and all dependent comparisons (`set_cc*`, `test_cc`, test instructions).
2. Implement IDC updates in add/sub ALU paths.
3. Mask `CPSU`/`PPSU` so S and reserved bits are not software-modifiable.
4. Add focused opcode-level regression tests for CC/branch matrices and PSW bit-manipulation instructions.
