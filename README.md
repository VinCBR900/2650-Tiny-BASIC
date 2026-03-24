# 2650-Tiny-BASIC

Signetics 2650 Tiny BASIC.

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

## Regression scenarios

See `regression_scenarios.md` for manual scenarios covering:

- replace with longer/shorter lines,
- delete first/middle/last lines,
- sorted inserts,
- out-of-memory insert handling,
- bad-`GOTO` and malformed-record runtime defenses,
- expression precedence/parentheses/unary handling.
