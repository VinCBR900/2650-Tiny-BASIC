# 2650-Tiny-BASIC

Signetics 2650 Tiny BASIC **WORK IN PRGRESS**.

## Current behavior notes

- Program line numbers are 16-bit values constrained to `1..32767` and are stored/sorted as unsigned 16-bit values.
- Program-memory insertions enforce `PROGLIM`; when insertion would exceed capacity, interpreter reports `?1`.
- Runtime `GOTO` to missing line reports `?2`.
- Runtime malformed stored-line records report `?3` and stop `RUN`.
- Expression parser supports:
  - `+`, `-`, `*`, `/`
  - parentheses `(...)`
  - unary `+` and unary `-`
  - precedence: unary > `*`/`/` > `+`/`-`
- Numeric arithmetic uses 16-bit two's-complement wraparound semantics.
  - Example: `32767+1` => `-32768`

## Instruction micro-tests (simulator semantics)

A targeted simulator micro-test suite lives at `tests/instruction_microtests.py`.

It exercises parser-sensitive instruction semantics directly:

- `COMI` + `BCTA,EQ/GT/LT`
- `SUBI` + `BCTA,EQ/GT/LT`
- `ADDI` wrap/non-wrap carry+zero branch expectations
- `BSTA`/`RETC` nested return-stack behavior
- conditional `BSTA` non-taken path (must not push RAS)

Run it with:

```bash
python3 tests/instruction_microtests.py
```

For end-to-end regression probing of the current uBASIC `RUN`/`LIST` behavior
on the simulator, use:

```bash
python3 tests/ubasic_run_list_regression.py
```

## Regression scenarios

See `regression_scenarios.md` for manual scenarios covering:

- replace with longer/shorter lines,
- delete first/middle/last lines,
- sorted inserts,
- out-of-memory insert handling,
- bad-`GOTO` and malformed-record runtime defenses,
- expression precedence/parentheses/unary handling.
