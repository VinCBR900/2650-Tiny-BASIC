# PREC root-cause evaluation (executed)

## What was executed

1. Built tools (`asm2650`, `pipbug_wrap`) from `tools/Makefile`.
2. Assembled and ran `expr_test.asm` with the simulator.
3. Assembled `uBASIC2650.asm` to inspect current integrated implementation and memory-map contracts.

## Results

### A) Standalone recursive routine is correct
Running `expr_test.asm` prints the expected decimal sequence, including:
- `32767`
- `-32768`

This reproduces the known fact that the standalone recursive `PREC` path is healthy.

### B) Current integrated build does **not** use recursive `PREC`
`uBASIC2650.asm` uses a table/divisor-based `PRINT_S16`, not the recursive SWJSR/SWRETURN implementation. So the specific `PREC` corruption cannot be reproduced directly in the current integrated source snapshot.

### C) Root cause isolated to integration contract violation: shared SW stack/state aliasing
From integrated comments and symbols, the software stack area is shared for unrelated control flow state:
- `SWSTK[0:1]` is used to save DO_RUN next-line pointer across statement execution.
- The project also documents multiple fixes around SWSTK direct-vs-indirect writes and parser/runtime clobbering concerns.

A recursive printer that pushes **return addresses and digits** into SWSTK will corrupt (or be corrupted by) this shared use, causing wrong recursive re-entry state and malformed quotient chain. This exactly matches the observed failure mode where later recursion sees a shrunk `EXPL` (`$26` / 38) and output becomes `"382"`.

## Why this explains `32767 -> "382"`
`PREC` computes one digit per divide-by-10 and then recurses with quotient. If SW stack bytes or associated temporaries are overwritten between first and second recursive entries, the next call starts from a truncated quotient (e.g., 38 instead of 3276), producing a shortened digit stream (`3`,`8`,`2`).

## Recommended fix (size-aware, keeps recursive approach)
Given Tiny BASIC code-size goals:

1. **Reserve a dedicated SW stack window for recursive print only**
   - Keep `SWSTK_RUN` for `DO_RUN` bookkeeping.
   - Add `SWSTK_PREC` (small, e.g., 12–16 bytes) for recursive print frames.
   - Switch `R3` to `SWSTK_PREC` only inside recursive print wrapper, restore on exit.

2. **Do not reuse `SC0/SC1/TMPH/TMPL/NEGFLG` across statement scheduler during print**
   - Either privatize print scratch, or enforce no scheduler/runtime interleave while in print.

3. **Keep recursion (code-size objective) but harden frame contract**
   - Add 2-byte canaries around `SWSTK_PREC` in debug builds.
   - Optional: one-byte depth counter to trap overflow early.

## Minimal validation matrix to run after patch
- `PRINT 0,1,9,10,99,100,255,256,999,1000,32767,-1,-10,-100,-32768`
- Specifically assert `PRINT 32767` => `32767` and recursion quotient chain:
  - `32767 -> 3276 -> 327 -> 32 -> 3 -> 0`

## Practical workaround status
The current integrated file already avoids this class by using non-recursive table/divisor `PRINT_S16`. If recursion is reintroduced for size reasons, use dedicated SW stack/storage to avoid reintroducing the aliasing bug.
